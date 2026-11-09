"""有界 IMU 安全交接：接收方采样，原控制 owner 应用。

职责：把同源因果 FIFO 端口与原始 latest 端口的 IMU 标量交给控制 owner 选择性取用，
并保留“未消费的最大倾角”以便中断恢复。
边界：本邮箱从不发布命令、不改 gyro/PID、不伪造收据；每个输入保留自己的
validator/watermark，只有通过 orientation 校验的样本才会进入。任何 tilt >= 0.18
都会中断恢复，因此未被消费的最大值在任一端口 latest 槽被替换后仍然保留。
"""

from collections import Counter, deque
import math
import os
from threading import Lock

# 环境变量开关；缺省关闭，非 "0"/"1" 抛错。configs/launch/settings.json 的
# features 目前把它设为 "1"，但只有部分探针会被 launch 强制改写。
SWITCH = "R103_IMU_SAFETY_INTAKE"
# 观察窗口最多保留 32 条（deque maxlen，硬编码）。
WINDOWS = 32


def enabled():
    """读取 `R103_IMU_SAFETY_INTAKE`；未设置/为 "0" 时关闭，非法值抛 `ValueError`。"""
    value = os.environ.get(SWITCH, "0")
    if value not in ("0", "1"):
        raise ValueError(SWITCH + " must be 0 or 1")
    return value == "1"


def scalar_fresh(stamp, receipt, timeout, now, ros_now):
    """复刻未改动的 `RawImuTilt.fresh` 不等式，单位 s。

    `receipt` 是单调墙钟收据时刻，`stamp` 是 ROS 源时间，两者不同域。判定为：
    收据有限且 `0 <= now - receipt <= timeout`，源时间在
    `-0.10 <= ros_now - stamp <= timeout`（-0.10 是未来容差，硬编码）。
    本函数不缓存，也不替调用方把两个时钟域相减。
    """
    return (
        math.isfinite(receipt)
        and 0 <= now - receipt <= timeout
        and -0.10 <= ros_now - stamp <= timeout
    )


class SafetyIntake:
    """有界 IMU 安全信箱：保存各端口最新样本与未消费的倾角峰值。

    三个槽位：`latest`（causal_fifo 端口）、`original_latest`（原始端口）、
    `peak`（tilt >= 0.18 的最大样本）。`version` 每次 `offer` 递增，
    `last_consumed` 记录 owner 已消费到哪一版；`windows` 只保留最多 32 条观察。
    内部用 `Lock` 串行化；不写盘、不发布。
    """

    def __init__(self, original_raw_imu=None, reading_view=None):
        """可选注入原始 raw_imu 与一个独立的 reading view（安全端口的读取副本）。"""
        self._lock = Lock()
        self.original_raw_imu = original_raw_imu
        self.reading_view = reading_view
        self.original_latest = None
        self.original_sequence = self.version = 0
        self.latest = self.peak = None
        self.last_consumed = 0
        self.counts = Counter()
        self.windows = deque(maxlen=WINDOWS)
        self.first_window = None

    def offer(self, sequence, stamp_ns, receipt_ns, callback_clock_ns, tilt, port="causal_fifo"):
        """提交一个已通过 orientation 校验的样本（时间单位 ns，tilt 单位 rad）。

        `port` 决定写入哪个槽：`"causal_fifo"` 或 `"original_latest"`。
        源 header 未前进（`stamp_ns <= 同端口上一 stamp`）的样本被丢弃并计
        `nonadvancing_offer`，因此重复 header 不会获得更晚的 callback 收据。
        tilt >= 0.18 且大于当前峰值时更新 `peak`；该方法只记录，不应用保护。
        """
        sample = (sequence, stamp_ns, receipt_ns, callback_clock_ns, tilt, port)
        with self._lock:
            # 重复的源 header 不会获得更晚的 callback 收据。
            previous = self.latest if port == "causal_fifo" else self.original_latest
            if previous is not None and stamp_ns <= previous[1]:
                self.counts["nonadvancing_offer"] += 1
                return
            if port == "causal_fifo":
                self.latest = sample
            else:
                self.original_latest = sample
            self.version += 1
            self.counts["offered"] += 1
            self.counts["offered_" + port] += 1
            if tilt >= 0.18 and (self.peak is None or tilt > self.peak[4]):
                self.peak = sample

    def note_original_accepted(self, callback_clock_ns):
        """在原始 on_imu 的 accepted 分支、其 guard 之前调用，登记原始端口样本。

        从 `original_raw_imu` 读取 `stamp`/`updated`/`tilt`（s），四舍五入转成 ns；
        `original_sequence` 单调递增，仅用于本端口排序。
        """
        raw = self.original_raw_imu
        self.original_sequence += 1
        self.offer(
            self.original_sequence,
            round(raw.stamp * 1e9),
            round(raw.updated * 1e9),
            callback_clock_ns,
            raw.tilt,
            "original_latest",
        )

    @staticmethod
    def _port_observation(node, now, ros_now, original, original_fresh, merged_fresh):
        """生成两端口对照的只读观察行（时间统一转 ns，缺失值保留 `None`）。

        `original` 为 `(stamp, updated, timeout)`（s）；`original_fresh`/
        `merged_fresh` 分别是原端口与合并视图的 scalar fresh 结果。该函数不写入任何
        状态，只把“旧端口已过期、新视图仍新鲜”的间隙记录清楚。
        """

        def ns(value):
            return None if not math.isfinite(value) else round(value * 1e9)

        return dict(
            original_stamp_ns=ns(original[0]),
            original_receipt_ns=ns(original[1]),
            original_timeout_ns=round(original[2] * 1e9),
            original_scalar_fresh=original_fresh,
            merged_stamp_ns=ns(node.raw_imu.stamp),
            merged_receipt_ns=ns(node.raw_imu.updated),
            merged_scalar_fresh=merged_fresh,
            old_port_stale_new_view_fresh=not original_fresh and merged_fresh,
            consume_wall_ns=round(now * 1e9),
            consume_clock_ns=round(ros_now * 1e9),
            r73_publication_sequence_before=getattr(node, "publication_records", None),
        )

    def consume(self, node, now, ros_now):
        """由控制 owner 在读 guard 时调用；返回未消费的倾角峰值或 `None`。

        没有任何 intake 回调会调用本方法，所有 `node` 写入仍发生在原互斥控制组里。
        `now` 是单调墙钟（s），`ros_now` 是 ROS 时钟（s）。返回的 peak 由调用方用于
        tilt hold；本方法自身不做停车判定。
        """
        # 本方法不由任何 intake 回调调用；所有 node 写入都留在原互斥控制组内。
        original_raw = self.original_raw_imu if self.original_raw_imu is not None else node.raw_imu
        original = (original_raw.stamp, original_raw.updated, original_raw.timeout)
        original_fresh = scalar_fresh(*original, now, ros_now)
        # 先取本端口快照与 peak，并清空 peak（交给本次 guard 消费）。
        with self._lock:
            self.counts["guard_reads"] += 1
            self.counts["original_port_comparison_reads"] += 1
            samples = tuple(s for s in (self.latest, self.original_latest) if s is not None)
            peak = self.peak
            self.peak = None
            # 没有新样本：只统计，不挑选/不改时间；若旧端口已过期但合并视图仍新鲜，
            # 记录一次端口间隙观察（这是诊断，不是保护动作）。
            if not samples or self.version == self.last_consumed:
                self.counts["no_new_sample"] += 1
                merged_fresh = scalar_fresh(
                    node.raw_imu.stamp, node.raw_imu.updated, node.raw_imu.timeout, now, ros_now
                )
                gap = not original_fresh and merged_fresh
                self.counts["old_port_stale_new_view_fresh"] += int(gap)
                if gap:
                    # 本次 guard 没有选择或重打任何样本的时间戳；既有视图真实的
                    # 先前收据仍然是新鲜的。
                    row = self._port_observation(
                        node, now, ros_now, original, original_fresh, merged_fresh
                    )
                    row.update(
                        selected_this_guard=False,
                        transferred=False,
                        guard_state_before=node.state,
                        tilt_hold_before=node.tilt_hold,
                    )
                    self.windows.append(row)
                return None
            self.last_consumed = self.version
        before = (node.raw_imu.stamp, node.raw_imu.updated)
        before_fresh = scalar_fresh(*before, node.raw_imu.timeout, now, ros_now)
        if self.reading_view is not None:
            # 启动期新鲜度沿用原对象，直到第一次真实 guard 读；running 之前的回调
            # 不依赖“为了观察一条原始 accepted IMU 而调度 guard”。
            node.raw_imu = self.reading_view
        raw = node.raw_imu
        # 优先从仍新鲜的样本里挑：最晚源时间，同源时间取更早的收据（重复 header 不能
        # 借新收据“变新鲜”）。一个都不新鲜时退回全部样本，由下面的 transferred 判定。
        fresh_samples = tuple(
            s for s in samples if scalar_fresh(s[1] / 1e9, s[2] / 1e9, raw.timeout, now, ros_now)
        )
        sample = max(fresh_samples or samples, key=lambda s: (s[1], -s[2]))
        sequence, stamp_ns, receipt_ns, callback_clock_ns, tilt, port = sample
        stamp, receipt = stamp_ns / 1e9, receipt_ns / 1e9
        sample_fresh = scalar_fresh(stamp, receipt, raw.timeout, now, ros_now)
        # 只有样本新鲜且源时间真的前进时才写回 raw_imu，避免用旧样本覆盖新状态。
        transferred = sample_fresh and stamp > raw.stamp
        if transferred:
            raw.stamp, raw.updated, raw.tilt = stamp, receipt, tilt
            raw.max_tilt = max(raw.max_tilt, tilt)
            raw.last_rejected = None
        # peak 即使本次没有转写，也要并入 max_tilt：未消费的最大倾角不能丢。
        if peak is not None:
            raw.max_tilt = max(raw.max_tilt, peak[4])
        after_fresh = scalar_fresh(raw.stamp, raw.updated, raw.timeout, now, ros_now)
        old_port_gap = not original_fresh and after_fresh
        self.counts["old_port_stale_new_view_fresh"] += int(old_port_gap)
        self.counts["consumed"] += 1
        self.counts["transferred"] += int(transferred)
        self.counts["expired_or_future_receipt"] += int(not sample_fresh)
        self.counts["stale_to_fresh"] += int(not before_fresh and after_fresh)
        self.counts["peak_delivered"] += int(peak is not None)
        self.counts["severe_delivered"] += int(peak is not None and peak[4] >= 0.50)
        # 只在“首个窗口”或相关状态转换（由新鲜变新鲜、出现 peak、端口间隙）时才
        # 分配一条观察，避免每帧都膨胀 windows。
        if self.first_window is None or not before_fresh or peak is not None or old_port_gap:
            row = dict(
                sequence=sequence,
                source_stamp_ns=stamp_ns,
                selected_port=port,
                selected_this_guard=True,
                callback_wall_ns=receipt_ns,
                callback_clock_ns=callback_clock_ns,
                consume_wall_ns=round(now * 1e9),
                consume_clock_ns=round(ros_now * 1e9),
                before_stamp_ns=None if not math.isfinite(before[0]) else round(before[0] * 1e9),
                before_receipt_ns=None if not math.isfinite(before[1]) else round(before[1] * 1e9),
                before_raw_scalar_fresh=before_fresh,
                sample_scalar_fresh=sample_fresh,
                after_raw_scalar_fresh=after_fresh,
                transferred=transferred,
                pending_peak=None if peak is None else list(peak),
                publication_count_before=node.counts.get("commands", 0),
                r73_publication_sequence_before=getattr(node, "publication_records", None),
                guard_state_before=node.state,
                tilt_hold_before=node.tilt_hold,
            )
            row.update(
                self._port_observation(node, now, ros_now, original, original_fresh, after_fresh)
            )
            if self.first_window is None:
                self.first_window = row
            self.windows.append(row)
        return peak

    def receipt(self):
        """返回 `r103_imu_safety_intake/v1` 回执（只读快照，不改变状态）。

        字段含计数、首个窗口、最近窗口、各端口 latest、未消费 peak、
        `last_consumed_mailbox_version` 以及若干语义说明字符串；由
        `imu_causal_buffer.finish()` 真正写盘。
        """
        with self._lock:
            return dict(
                schema="r103_imu_safety_intake/v1",
                enabled=True,
                # 保留槽位固定 3 个（latest/original_latest/peak）；窗口上限即 WINDOWS。
                retained_sample_slots=3,
                maximum_windows=WINDOWS,
                counts=dict(self.counts),
                first_window=self.first_window,
                windows=list(self.windows),
                latest=None if self.latest is None else list(self.latest),
                original_latest=None
                if self.original_latest is None
                else list(self.original_latest),
                pending_peak=None if self.peak is None else list(self.peak),
                last_consumed_mailbox_version=self.last_consumed,
                before_shadow="scalar raw state immediately before handoff, not a rerun of the OFF controller",
                original_comparison_scope="every guard consume entry at its fixed caller clocks; original validator scalars, not a rerun of the OFF controller",
                original_latest_rejection_observation="controller.on_imu note_latest reads intake.original_raw_imu.last_rejected when enabled",
                outcome_join="r73_publications sequence after r73_publication_sequence_before and consume_wall_ns; guard entry alone does not claim a publication",
                source_and_receipt_times_preserved=True,
                original_latest_validator_and_watermark_independent=True,
                no_direct_gyro_PID_or_pair_history_writes=True,
                original_gyro_and_history_acceptance_uses_original_latest_validator=True,
            )
