"""同源因果 IMU FIFO，附带可选的、由控制 owner 消费的安全信箱。

职责：缓存 `/livox/imu` 上通过私有 validator 的回调，为同一个 pose 源时间配对
最近的一条 IMU；并在开启 `r85_imu_source_buffer.enabled` 时把标量交给
`SafetyIntake`（见 `imu_mailbox.py`）。
边界：不写 `node.raw_imu`、gyro、pose 或 tilt；配对只是选择已有回调，绝不重打时间戳。
"""

import copy
import time
from collections import Counter, deque
from threading import Lock

import numpy as np

# 200 Hz x 原有 0.25 s 新鲜度 = 50 条消息，64 是有限余量（硬编码）。
FIFO_DEPTH = 64
# 历史行最多 128 条；配对窗口 20 ms；事件去重表上限 64 条（均为硬编码）。
HISTORY_DEPTH = 128
PAIR_NS = 20_000_000
EVENT_CAP = 64


def enabled(profile):
    """`profile["r85_imu_source_buffer"]["enabled"] is True` 时启用；缺省或类型不符视为关闭。

    这是 profile（本次运行快照）里的开关，不是 launch 参数；它只决定是否构造
    `CausalImuBuffer`。
    """
    config = profile.get("r85_imu_source_buffer", {})
    return isinstance(config, dict) and config.get("enabled") is True


class CausalImuBuffer:
    """同源 IMU 的私有接收器；回调与配对均在锁内，读侧只返回已有记录。

    构造时创建独立 validator（不触碰控制 owner 的 `node.raw_imu`）。可选安全
    信箱只在控制 owner 的 guard 内消费；本类不发布命令、不做停车判定。
    """

    def __init__(self, node, validator_type):
        """`node` 提供 `stamp`/`imu_body_rotation`/`raw_imu.timeout`/`evidence`；
        `validator_type` 是 IMU 校验器类。开关关闭时 `safety_intake` 为 `None`。
        """
        self.node = node
        self._lock = Lock()
        # 私有接收器 validator：绝不写 node.raw_imu、gyro、pose 或 tilt。可选信箱
        # 只由控制 owner 消费。
        self.validator = validator_type(
            node.imu_body_rotation, node.imu_reference_rotation, timeout=node.raw_imu.timeout
        )
        self.history = deque(maxlen=HISTORY_DEPTH)
        self.recent = deque(maxlen=16)
        self.counts = Counter()
        self.events = set()
        self.last_event = None
        from embodied_navigation.control.safety.imu_mailbox import (
            enabled as safety_enabled,
            SafetyIntake,
        )

        self.safety_intake = None
        if safety_enabled():
            # 保留原 latest 端口实际使用的 validator 对象；只有 guard 消费时才会
            # 更新这里独立的安全读取视图。
            reading_view = validator_type(
                node.imu_body_rotation, node.imu_reference_rotation, timeout=node.raw_imu.timeout
            )
            self.safety_intake = SafetyIntake(node.raw_imu, reading_view)

    def callback(self, msg, info):
        """订阅回调：校验并记录一条 IMU；不在此应用保护。

        `wall` 是单调墙钟 ns，`clock` 是 ROS 时钟 ns，`stamp` 是消息源时间 ns，
        三者不同域，分别保留。orientation 校验通过且开启信箱时，先把标量交给
        `SafetyIntake`；之后 gyro 非有限也会拒绝入 history。
        """
        wall = time.monotonic_ns()
        clock = self.node.get_clock().now().nanoseconds
        with self._lock:
            stamp = int(self.node.stamp(msg))
            q, v = msg.orientation, msg.angular_velocity
            self.counts["fifo_callbacks"] += 1
            accepted = self.validator.update(
                [q.x, q.y, q.z, q.w],
                stamp / 1e9,
                wall / 1e9,
                clock / 1e9,
                msg.orientation_covariance[0] >= 0,
            )
            # 倾角安全只依赖 orientation 校验，与 gyro 是否有限无关：因此先 offer
            # 标量。独立接收器只提交标量，绝不改原控制 owner 的 raw_imu、tilt hold
            # 或发布状态。
            if accepted and self.safety_intake is not None:
                self.safety_intake.offer(
                    self.counts["fifo_callbacks"], stamp, wall, clock, self.validator.tilt
                )
            # gyro 转到 body 系后再判有限；非有限时记录 reason（validator 的拒绝原因
            # 优先，否则 "nonfinite_gyro"）。
            gyro = self.node.imu_body_rotation @ np.array([v.x, v.y, v.z])
            accepted = bool(accepted and np.isfinite(gyro).all())
            reason = None if accepted else self.validator.last_rejected or "nonfinite_gyro"
            # 保留 RMW 提供的原始时间/序号字段（各自时钟域），不做换算。
            meta = {}
            for key in (
                "source_timestamp",
                "received_timestamp",
                "publication_sequence_number",
                "reception_sequence_number",
            ):
                value = info.get(key) if isinstance(info, dict) else getattr(info, key, None)
                if type(value) is int:
                    meta["rmw_" + key] = value
            summary = dict(
                port="causal_fifo",
                stamp_ns=stamp,
                callback_wall_ns=wall,
                callback_ros_clock_ns=clock,
                accepted=accepted,
                reason=reason,
                **meta,
            )
            # recent 只保留最近 16 条摘要（deque maxlen），用于事件回执。
            self.recent.append(summary)
            self.counts["fifo_accepted" if accepted else "fifo_rejected"] += 1
            if not accepted:
                return
            # 本次真实回调收据只捕获一次；pair/consume 之后都不会重打时间戳。
            # RMW 时间戳保留自己的时钟域，不做虚构换算。
            row = dict(
                stamp_ns=stamp,
                received_wall_ns=wall,
                frame_id="body",
                angular_velocity_body=gyro.tolist(),
                source_frame_id=msg.header.frame_id,
                gyro_source_xyz=[v.x, v.y, v.z],
                orientation_xyzw=[q.x, q.y, q.z, q.w],
                orientation_covariance=list(msg.orientation_covariance),
                imu_to_body_rotation=self.node.imu_body_rotation.tolist(),
                callback_ros_clock_ns=clock,
                source_topic="/livox/imu",
                navigation_ground_truth_used=False,
                source_port="same_topic_causal_fifo",
                receipt_semantics="actual FIFO callback entry, not publication/RMW receipt",
                fifo_callback_sequence=self.counts["fifo_callbacks"],
                **meta,
            )
            # header 晚于回调时钟的样本只进 history，不参与配对（标记后仍保留证据）。
            if stamp > clock:
                row["r85_original_future_header_history_only"] = True
            self.history.append(row)

    def note_latest(self, stamp, wall, clock, recorded, rejection):
        """记录原 latest 端口的一次回调（诊断用），时间单位 ns。

        `recorded` 表示该回调是否真的进了 history，`rejection` 是原 validator 的
        拒绝原因（可能为 `None`）。本方法不改变任何控制状态。
        """
        with self._lock:
            self.counts["latest_callbacks"] += 1
            self.counts["latest_history_recorded"] += int(recorded)
            self.recent.append(
                dict(
                    port="original_latest",
                    stamp_ns=stamp,
                    callback_wall_ns=wall,
                    callback_ros_clock_ns=clock,
                    accepted_history_recorded=recorded,
                    raw_rejection=rejection,
                )
            )

    def pair(self, pose_stamp, original, clock, *, snapshot=False):
        """为 `pose_stamp` 选一条同源 IMU；无可用记录返回 `None`。

        `pose_stamp`/`clock` 单位 ns。pose 源时间晚于当前控制时钟时直接返回 None
        （未来 pose 不与过去 IMU 配对）。`original`（原端口数据）优先；否则在
        `[pose_stamp, pose_stamp + 20 ms]` 内选最新 stamp，同一 stamp 取最早收据。
        `snapshot=True` 时用 `scalar_snapshot` 转成标量快照。
        """
        if snapshot:
            from embodied_navigation.control.parking.causal_join import scalar_snapshot
        # 未来 pose 不与过去 IMU 配对；源时间晚于控制时钟即视为不可因果。
        if pose_stamp > clock:
            return None
        if original is not None:
            return (
                scalar_snapshot(original) if snapshot else original
            )  # Original port retains priority.
        with self._lock:
            # 只接受 pose 之前 0~20 ms 内的 IMU；不使用未来样本做“预测”。
            options = [r for r in self.history if 0 <= pose_stamp - r["stamp_ns"] <= PAIR_NS]
            if not options:
                return None
            newest = max(r["stamp_ns"] for r in options)
            # 重复源 header 不能在消费时获得更新的收据：同 stamp 取最早 wall 收据。
            selected = min(
                (r for r in options if r["stamp_ns"] == newest), key=lambda r: r["received_wall_ns"]
            )
            return scalar_snapshot(selected) if snapshot else selected

    def event(self, pose_stamp, clock, wall, phase, pair):
        """记录一次配对/失配事件到证据流；同一 `(pose_stamp, phase)` 只记一次。

        `phase` 是事件阶段标签，`pair` 为选中的 IMU 记录或 `None`。事件表达到
        `EVENT_CAP`（64）后只保留 `last_event` 并计 `events_omitted_by_cap`，保证
        最后的失配/恢复仍可查。写盘走 `node.evidence.append`，默认是空操作。
        """
        with self._lock:
            key = (pose_stamp, phase)
            if key in self.events:
                return
            row = dict(
                schema="same_source_causal_imu_timing/v1",
                phase=phase,
                pose_stamp_ns=pose_stamp,
                control_clock_ns=clock,
                observed_wall_ns=wall,
                original_pair_max_ns=PAIR_NS,
                source_topic="/livox/imu",
                paired_stamp_ns=None if pair is None else pair["stamp_ns"],
                paired_received_wall_ns=None if pair is None else pair["received_wall_ns"],
                paired_source_port=None
                if pair is None
                else pair.get("source_port", "original_latest"),
                recent_callbacks=copy.deepcopy(list(self.recent)),
                counters=dict(self.counts),
                navigation_ground_truth_used=False,
            )
            # 即使事件表已达上限，也保留最后一条失配/恢复行供回执使用。
            self.last_event = row
            if len(self.events) >= EVENT_CAP:
                self.counts["events_omitted_by_cap"] += 1
                return
            self.events.add(key)
        self.node.evidence.append(self.node.run / "imu_source_timing_events.jsonl", row)

    def finish(self):
        """写两个回执文件（`evidence.atomic`）：IMU 配对时序与安全信箱。

        回执 schema 为 `same_source_causal_imu_timing_receipt/v1`；路径在
        `node.run/imu_source_timing_receipt.json` 与 `r103_imu_safety_intake.json`。
        默认 `RuntimeTelemetry.atomic` 会真正落盘（与多数 append 证据不同）。
        """
        with self._lock:
            receipt = dict(
                schema="same_source_causal_imu_timing_receipt/v1",
                fifo_depth=FIFO_DEPTH,
                history_depth=HISTORY_DEPTH,
                event_cap=EVENT_CAP,
                recorded_events=len(self.events),
                counters=dict(self.counts),
                last_pairing_event=copy.deepcopy(self.last_event),
                original_latest_tilt_subscription_preserved=True,
                pair_max_ns=PAIR_NS,
                original_freshness_and_timestamp_checks_unchanged=True,
                navigation_ground_truth_used=False,
            )
        self.node.evidence.atomic(self.node.run / "imu_source_timing_receipt.json", receipt)
        if self.safety_intake is not None:
            self.node.evidence.atomic(
                self.node.run / "r103_imu_safety_intake.json", self.safety_intake.receipt()
            )
