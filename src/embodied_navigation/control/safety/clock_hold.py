"""墙钟 tick 与 ROS 时钟的边界；重复时钟只能发出精确零。

职责：比较已接受的时钟/header 进度，分类 advance/duplicate/stalled/backwards；
重复或停滞时在**任何控制器数学与原生几何之前**发出精确零命令并冻结状态。
边界：本模块不含 ROS、策略、几何或控制器数学，也不发布非零命令；调用方必须
在到达/航向/PI/native guard 代码之前调用 `before_control`。源 header 与收据时间
作为不可变证据比较，绝不重打时间戳；旧 native 几何时钟契约保持严格递增。
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from embodied_navigation.common.input_freshness import (
    enabled as minimal_baseline_enabled,
    raw_imu_fresh,
    wall_stale,
    wall_stale_ns,
)

# 300 ms（硬编码）：源/收据 TTL，对应 CONTRACT 的 freshness_sim_and_wall_s=0.3。
TTL_NS = 300_000_000
# 与 safety_configuration.clock() 同源的一份契约副本；本文件内没有引用它，
# 因此它只是文档/对照用的常量表（见模块 README 的“重复实现”一节）。
CONTRACT = {
    "schema": "teacher_control_clock_exact_zero_hold/v1",
    "same_clock_policy": "exact_zero_before_math_and_native_geometry",
    "freshness_sim_and_wall_s": 0.3,
    "continuous_stale_fail_wall_s": 8.0,
    "backward_clock_policy": "latch_failure",
    "resume_policy": "actual_clock_advance_and_fresh_original_sources_and_native_geometry",
    "short_hold_integral_and_dwell": "frozen",
    "long_hold_integral_and_dwell": "protect_reset",
    "source_receipt_refresh_permitted": False,
}


def identity(value):
    """对 JSON 可序列化值做规范化 SHA-256；用于判定同一 header 的内容身份。"""
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


class ControlClockHold:
    """比较已接受的时钟/header 进度，决定保持还是放行。

    `classify` 返回 `"advance"`、`"duplicate"`、`"stalled"` 或 `"backwards"`；
    重复/停滞会在任何数学或原生几何之前强制精确零控制。本类只做判定与冻结快照，
    不调用控制器、不写源时间。
    """

    def __init__(self):
        """初始化时钟进度、保持标志与来源内容身份表。"""
        self.last_clock_ns = None
        self.last_wall_ns = None
        self.last_progress_wall_ns = None
        self.holding = False
        self.guard_required = False
        self.records = 0
        self.sources = {}

    def source(self, name, stamp, value):
        """登记一个来源的 header 时间与内容摘要（不可变证据）。

        `stamp` 单位 ns。同名来源 header 不得回退；同一 header 不得更换 payload/
        收据（摘要变化即抛 `ValueError`）。`value is None` 时跳过登记。
        """
        if value is None:
            return
        digest = identity(value)
        old = self.sources.get(name)
        if old is not None:
            if stamp < old[0]:
                raise ValueError(name + "_accepted_header_backwards")
            if stamp == old[0] and digest != old[1]:
                raise ValueError(name + "_same_header_changed_payload_or_receipt")
        self.sources[name] = (stamp, digest)

    def classify(self, clock_ns, wall_ns, actual_guard_ns):
        """分类本次时钟进度；返回四种状态之一。

        `clock_ns`/`wall_ns` 必须是非负整数（ns），否则抛 `ValueError`；
        `actual_guard_ns` 是上一次真实几何 guard 的时钟（可为 `None`）。比较规则：
        时钟或墙钟回退，以及 clock 小于实际 guard 时钟，都判 `"backwards"`；
        clock 与上一次或实际 guard 时钟相等判重复；重复持续到墙钟
        `>= TTL_NS`（300 ms）升级为 `"stalled"`。重复/停滞都会置
        `holding` 与 `guard_required`。
        """
        if any(type(x) is not int or x < 0 for x in (clock_ns, wall_ns)):
            raise ValueError("Original nonnegative integer clocks required")
        previous = self.last_clock_ns
        if (
            (previous is not None and clock_ns < previous)
            or (actual_guard_ns is not None and clock_ns < actual_guard_ns)
            or (self.last_wall_ns is not None and wall_ns < self.last_wall_ns)
        ):
            return "backwards"
        duplicate = (previous is not None and clock_ns == previous) or (
            actual_guard_ns is not None and clock_ns == actual_guard_ns
        )
        # 只有 clock 真正前进才记录“最后一次进度”的墙钟，用于 300 ms 停滞判定。
        if previous is None or clock_ns > previous:
            self.last_progress_wall_ns = wall_ns
        self.last_clock_ns, self.last_wall_ns = clock_ns, wall_ns
        if duplicate:
            self.holding = self.guard_required = True
            return "stalled" if wall_ns - self.last_progress_wall_ns >= TTL_NS else "duplicate"
        self.holding = False
        return "advance"

    def guard_completed(self):
        """真实几何 guard 完成后清除“需要 guard”标志（由调用方在检查后调用）。"""
        self.guard_required = False

    @staticmethod
    def snapshot(node):
        """冻结与时钟保持相关的控制器状态，用于前后对照回执；只读，不修改 node。"""
        c = node.cascade
        return dict(
            velocity_integral=None if c is None else c.velocity_integral.tolist(),
            hold_integral=None if c is None else c.hold_integral.tolist(),
            math_pose_stamp_ns=None if c is None else c.last_math_stamp,
            capture_dwell_pose_stamp_ns=None if c is None else c.capture_dwell,
            turn_dwell_pose_stamp_ns=None if c is None else c.turn_dwell,
            parking_hold_pose_stamp_ns=None if c is None else c.hold_stamp,
            obstacle_clear_since_s=node.obstacle_clear_since,
            arrival_since_s=node.arrival_since,
            region_arrival_since_stamp_ns=None
            if node.region_arrival is None
            else node.region_arrival.since,
            pid_records=node.pid_records,
            actual_guard_records=node.guard_sequence,
        )

    def before_control(self, node, clock_ns, wall_ns):
        """本 tick 完全由精确零发布消费时返回 True。

        短重复会冻结所有数学与 dwell；墙钟停滞 >=300 ms 或原来源过期会重置受保护
        的 dwell 与 PI，并要求原有两个新鲜来源 header 才能恢复。连续失联 >8 s 的
        既有失败规则保持不变。每次保持仍会评估原 IMU/SLAM/bridge 的严重保护和证据
        写入错误。参数 `clock_ns`/`wall_ns` 单位 ns；调用方须保证在到达/航向/PI/
        native guard 之前调用。
        """
        # 只接管 running/succeeded；idle/failed 等状态不在此重写，避免掩盖原状态机。
        if node.state not in ("running", "succeeded"):
            return False
        now = wall_ns / 1e9
        previous_clock = self.last_clock_ns
        previous_guard = node.last_actual_guard_clock_ns
        before = self.snapshot(node)
        try:
            # 登记两个几何来源的 header 与内容摘要；同一 header 换内容会判为不可变证据异常。
            self.source("SLAM", int(node.pose_stamp), node.feedback)
            c = node.guard_cloud_receipt
            if c is not None:
                self.source(
                    "registered_cloud",
                    int(node.cloud_stamp),
                    {
                        k: copy.deepcopy(c[k])
                        for k in (
                            "stamp_ns",
                            "received_monotonic_wall",
                            "filtered_xyz_float64_sha256",
                            "nearest_slam_pose_stamp_ns",
                        )
                    },
                )
            # 与上一次真实几何 guard 的时钟一起比较，防止绕过几何检查“跳回”旧时钟。
            decision = self.classify(clock_ns, wall_ns, previous_guard)
        except (ValueError, TypeError, KeyError) as error:
            decision = "invalid_immutable_source:" + str(error)
        if decision == "advance":
            # 强制实际几何检查仍待完成，直到真正检查过的路径可用；
            # select_pid_velocity 会把该标志做 OR 合并（本函数不自行清除）。
            return False
        # 进入保持：强制要求一次真实 guard；先跑原 tilt 保护，再用下面的过期检查累积原因。
        self.guard_required = True
        protected = node.apply_tilt_guard(now, clock_ns / 1e9)
        stale = []
        # pose/点云超时来自 profile 的 pose_cloud_timeout_s（本次运行配置），单位 s。
        timeout = node.profile["pose_cloud_timeout_s"]
        for name, stamp, receipt in (
            ("SLAM", node.pose_stamp, node.pose_updated),
            ("registered_cloud", node.cloud_stamp, node.cloud_updated),
        ):
            # wall_stale 带 0.05 s 未来容差；仿真侧允许 -50 ms 未来容差，
            # 并要求 clock-stamp < timeout。
            if wall_stale(
                node,
                name + "_clock_hold",
                now,
                receipt,
                timeout,
                not math.isfinite(receipt) or not -0.05 <= now - receipt < timeout,
                future_tolerance=0.05,
            ) or not -50_000_000 <= clock_ns - stamp < round(timeout * 1e9):
                stale.append(name + "_sim_or_wall_TTL")
        # RAW IMU 仍按既有新鲜度判定（sim 与 wall 双窗口），不因本模块放宽。
        if not raw_imu_fresh(node, now, clock_ns / 1e9, node.raw_imu.fresh(now, clock_ns / 1e9)):
            stale.append("raw_IMU_sim_or_wall_TTL")
        # 配对来源携带原始整数收据时间。零发布不会更新这些时间、bridge 的 cascade
        # 源文件或任何 ACK。
        for name, source in (("feedback", node.feedback), ("causal_IMU", node.paired_imu)):
            if (
                source is None
                or not 0 <= clock_ns - source["stamp_ns"] <= TTL_NS
                or wall_stale_ns(
                    node,
                    name + "_clock_hold",
                    wall_ns,
                    source["received_wall_ns"],
                    TTL_NS,
                    not 0 <= wall_ns - source["received_wall_ns"] <= TTL_NS,
                )
            ):
                stale.append(name + "_sim_or_wall_TTL")
        # backwards 与“不可变来源异常”是致命错误：状态置 failed，不等待恢复。
        fatal = decision == "backwards" or decision.startswith("invalid_immutable_source:")
        # 证据写入错误同样致命：无法留下回执时不能继续放行控制。
        if node.evidence.error:
            fatal = True
            stale.append("evidence_writer:" + str(node.evidence.error))
        if fatal:
            node.state = "failed"
            node.message = "控制时钟倒退或实际输入身份异常，停车：" + decision
        # stalled 需要墙钟确实停滞（300 ms TTL）才计入过期原因，避免误报。
        if decision == "stalled" and wall_stale(
            node, "control_clock_stall", now, self.last_progress_wall_ns / 1e9, TTL_NS / 1e9, True
        ):
            stale.append("control_ROS_clock_wall_stall_300ms")
        # 任一保护/过期/致命条件成立都重置受保护的 dwell 与 PI 状态。
        reset = fatal or protected or bool(stale) or node.state == "failed"
        if reset:
            node.obstacle_clear_since = None
            node.reset_region_arrival(
                "failed" if node.state == "failed" else "stale" if stale else "protected"
            )
            # 重置原因按优先级取一个稳定码，交给原 cascade._protect 处理（不改其阈值）。
            if node.cascade is not None:
                reason = (
                    "clock_backwards"
                    if decision == "backwards"
                    else "actual_source_identity_invalid"
                    if decision.startswith("invalid_immutable_source:")
                    else "evidence_writer"
                    if node.evidence.error
                    else "control_clock_stall_or_source_stale"
                    if stale
                    else "external_hold"
                )
                node.cascade._protect(reason, fatal=fatal)
            if stale:
                node.stale_since = node.stale_since or now
                # 连续失联 8.0 s（硬编码，对应 CONTRACT continuous_stale_fail_wall_s）。
                if now - node.stale_since > 8.0:
                    node.state = "failed"
                    node.message = "实际SLAM/点云/时钟输入连续失联超过8秒，停车"
            elif not fatal:
                node.stale_since = None
        elif node.state != "failed":
            node.message = "ROS仿真时钟本帧未前进，零速度保护等待下一时刻完整重验点云"
        # 不得把已存在的行当作本帧新算出的命令来源；正常单线程控制会在下一 tick
        # 之前清掉这些行。
        if node.pid_row is not None or node.pending_guard_row is not None:
            raise RuntimeError("Clock hold entered with an unflushed control/geometry row")
        old_command = list(node.command)
        # 即使本帧不走数学，也必须让下游知道“需要一次真实 guard 评估”。
        node.pid_guard_needs_evaluation = True
        source_preservation = None
        if (
            getattr(node, "automatic_reference", None) is not None
            and decision == "duplicate"
            and not reset
        ):
            # 只有短重复且未触发重置时，才尝试保留既有 ordinary 零源字节；
            # 该能力仅限本次栈内发布，由 duplicate_source 再逐项校验。
            from embodied_navigation.control.safety.duplicate_source import CONTEXT_SCHEMA

            duplicate_context = dict(
                schema=CONTEXT_SCHEMA,
                decision=decision,
                protected_or_stale_reset=reset,
                source_stale_reasons=list(stale),
                compute_ros_clock_ns=clock_ns,
                compute_monotonic_wall_ns=wall_ns,
                command_before=list(old_command),
            )
            try:
                source_preservation = node.publish_command(
                    duplicate_ordinary_hold=duplicate_context
                )
            finally:
                # 该能力只属于本次栈内发布；不允许任何 node 缓存把它带进之后的
                # 显式停车或保护路径。
                duplicate_context.clear()
        else:
            node.publish_command()
        # 记录本次保持：只写证据，不改来源收据；默认 append 是空操作，不默认落盘。
        self.records += 1
        node.evidence.append(
            node.run / "navigation_control_clock_hold.jsonl",
            dict(
                schema="teacher_control_clock_exact_zero_hold/v1",
                sequence=self.records,
                compute_ros_clock_ns=clock_ns,
                compute_monotonic_wall=now,
                publish_ros_clock_ns=node.last_publication_ros_clock_ns,
                # 以下两个标记由发布观察工具识别，勿改文字：它们界定“实际发布链接”。
                # PUBLICATION_OBSERVER_BEGIN actual_publication_link
                command_publication_sequence=getattr(node, "publication_records", None),
                # PUBLICATION_OBSERVER_END actual_publication_link
                previous_control_clock_ns=previous_clock,
                previous_actual_guard_clock_ns=previous_guard,
                last_clock_progress_monotonic_wall=None
                if self.last_progress_wall_ns is None
                else self.last_progress_wall_ns / 1e9,
                decision=decision,
                source_stale_reasons=stale,
                original_300ms_TTL_preserved=not minimal_baseline_enabled(node.profile),
                **(
                    {
                        "r73_upper_wall_last_observations": copy.deepcopy(
                            getattr(node, "_r73_upper_wall_last_observations", {})
                        )
                    }
                    if minimal_baseline_enabled(node.profile)
                    else {}
                ),
                controller_math_called=False,
                native_geometry_called=False,
                source_receipts_refreshed=False,
                native_guard_required_before_nonzero_resume=True,
                source_pose_stamp_ns=int(node.pose_stamp),
                source_cloud_stamp_ns=int(node.cloud_stamp),
                source_pose_received_monotonic_wall=node.pose_updated
                if math.isfinite(node.pose_updated)
                else None,
                source_cloud_received_monotonic_wall=node.cloud_updated
                if math.isfinite(node.cloud_updated)
                else None,
                request_id=node.request_id,
                waypoint_index=node.waypoint_index,
                command_before=old_command,
                command_after=list(node.command),
                state_after=node.state,
                protected_or_stale_reset=reset,
                frozen_control_before=before,
                frozen_control_after=self.snapshot(node),
                navigation_ground_truth_used=False,
                **(
                    {}
                    if source_preservation is None
                    else {"r85_ordinary_source_preservation": source_preservation}
                ),
            ),
        )
        return True
