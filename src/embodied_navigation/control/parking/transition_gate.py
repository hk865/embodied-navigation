"""教师停/航向相位门：只用新鲜 SLAM 与原始机体陀螺证明停/转向转换。

证据边界（不做什么）：
- 不用导航真值；不用"仅计时器"认定静止；不做 CHAMP 相位复位；不拿 IMU yaw 当定位。
- 导航 heading 只取 SLAM 机体 yaw；陀螺只参与停/倾判定，从不进入定位。

相位状态机（由 update() 推进）：
- drive：航向误差在 drive_heading_max_rad 内则继续直行；超出即停车、从 pre_turn 重来。
- pre_turn：必须先由实测静止（或诊断观察分支）放行，才允许进入 align。
- align：以 max_yaw_rate_radps 限幅转向，误差进入 turn_complete_rad 后进入 settle。
- settle：受控收敛 / 主动刹车 / 零速握手；只有释放判据通过才回到 drive。
失败即保持：任何握手失败都把 phase 置 failed，之后 update 返回 (False, 0.0)；
只有新的 request/waypoint 身份才会清掉失败状态。

独立仿真实验：profile 允许把转向上限开到 0.6 rad/s。这个 NAV 0.6 上限尚无物理验证；
在 0.3 rad/s 下，全部过渡数学、实测停止条件与 300 ms 新鲜度都保持不变。

模块级依赖：navigation_stop_window 提供 1 s 稳态统计窗与 eligible_purposes，
common.input_freshness 只做 pass-through（从不放宽新鲜度）。其余扩展与下层 ACK
都在函数内延迟 import，用于压住 parking <-> tracking / robot 的包级环。
"""

from collections import deque
import copy
import math
from numbers import Real
from embodied_navigation.control.parking.navigation_stop_window import NavigationStopWindow, SPEC
from embodied_navigation.common.input_freshness import enabled as minimal_baseline_enabled


def stance_support_handoff_configuration(profile):
    """校验实验性 stance support 内部目标配置（profile 键 r85_stance_support_handoff）。

    参数：profile 为合并后的运行 profile。
    返回：合法的配置副本；enabled=false 时返回 None（此时只接受 {"enabled"} 单键）。
    失败：kind 不是 locked_heading_PD_stance_support_v1、依赖开关组合不成立（要求
    r85_active_turn_braking/r85_quiet_handoff_target 开启、r85_neutral_rate_brake_capture
    显式关闭、cascade 外部 heading 门与 locked_heading_pd_v1 参考齐备），或缺少
    原始协调积分卸载配置时，直接 raise。
    边界：只替换内部目标，原始业务停车 owner 与验收阈值保持不变。
    """
    if "r85_stance_support_handoff" not in profile:
        return None
    value = profile["r85_stance_support_handoff"]
    if not isinstance(value, dict) or type(value.get("enabled")) is not bool:
        raise ValueError("r85_stance_support_handoff requires an explicit boolean enabled")
    # 关闭态必须"干净"：多带任何键都说明配置意图不明确，直接拒绝。
    if value["enabled"] is False:
        if set(value) != {"enabled"}:
            raise ValueError("Disabled stance support admits only enabled false")
        return None
    if set(value) != {"enabled", "kind"} or value["kind"] != "locked_heading_PD_stance_support_v1":
        raise ValueError("Unknown stance-support handoff configuration")
    cascade = profile.get("cascade", {})
    if (
        any(
            not isinstance(profile.get(key), dict)
            or set(profile[key]) != {"enabled"}
            or profile[key]["enabled"] is not required
            for key, required in (
                ("r85_active_turn_braking", True),
                ("r85_quiet_handoff_target", True),
                ("r85_neutral_rate_brake_capture", False),
            )
        )
        or not isinstance(cascade, dict)
        or cascade.get("external_heading_gate") is not True
        or cascade.get("r85_turn_brake_reference") != {"kind": "locked_heading_pd_v1"}
    ):
        raise ValueError(
            "Stance support requires active quiet locked-heading PD and explicit neutral off"
        )
    from embodied_navigation.control.parking.integral_unload import configuration

    if configuration(profile) is None:
        raise ValueError("Stance support requires original coordinated integral unload")
    return copy.deepcopy(value)


class TeacherHeadingGate:
    """教师停/转向相位门：drive -> pre_turn -> align -> settle 的实测判据与握手。

    由 navigation/ros_node.py 构造（初始化时替换 base_node 里的同名 gate），
    localization_mapping/known_scene_hold_guard.py 延迟引用。
    输入：每帧 SLAM 位姿/速度/机体 yaw 角速度、原始机体陀螺、调用方的零命令标志，
    以及下层 ACK 与 profile；输出：update() 的 (release, yaw_command) 与 evidence()。
    副作用：只维护相位、历史窗口、握手事件与失败原因；不写盘、不发命令、不做 IO。
    失败行为：握手失败进入 failed 并保持，直到 request/waypoint 身份变化。
    """

    def __init__(self, profile):
        """从 profile 冻结本门配置并初始化相位状态。

        读取的 profile 键（configs/control/settings.json 是基值，启动器/运行上下文
        可覆盖）：
        - max_yaw_rate_radps：转向上限，rad/s，只接受 (0, 0.6]；
        - teacher_transition：9 个停/转向判据键（单位见 update/stopped 内注释）；
        - navigation_settle_statistics_contract：交给 NavigationStopWindow 的 1 s 统计窗；
        - r85_* / ordinary_turn_control_handoff：实验扩展开关，只接受显式形状。
        构造期 raise 是"配置错误即失败"契约：这里不降级、不猜默认值。
        """
        # r73 最小诊断基线：input_freshness.enabled() 恒返回 False，重新开启会直接
        # raise，所以本进程里这个放宽分支不可达（只保留历史语义）。
        self.minimal_simulation_baseline = minimal_baseline_enabled(profile)
        # 转向上限来自 profile 键 max_yaw_rate_radps（基值 0.6 rad/s，单位 rad/s）。
        # 只接受 (0, 0.6]：0.6 是 NAV 上限且尚无物理验证；0.3 是过渡数学与停止条件
        # 保持不变的历史口径。
        self.max_turn_rate = profile["max_yaw_rate_radps"]
        if not math.isfinite(self.max_turn_rate) or not 0 < self.max_turn_rate <= 0.6:
            raise ValueError(
                "Simulation experimental turn limit must be finite and within (0,0.6]rad/s; NAV0.6 is not physically validated"
            )
        # teacher_transition 的 9 个判据键，直接取自 profile 且不做范围校验；单位分别
        # 为 rad(0.2/0.1)、s(0.15/0.2/0.3)、rad/s(0.05/0.05)、m/s(0.03)、个(3)。
        self.config = profile["teacher_transition"]
        # 历史窗口容量硬编码 64，只用于限制内存；与控制回调频率的换算关系未说明。
        self.history = deque(maxlen=64)
        self.neutral_rate_brake_capture_config = None
        if "r85_neutral_rate_brake_capture" in profile:
            from embodied_navigation.control.tracking.cascade_core import (
                neutral_rate_brake_capture_configuration,
            )

            self.neutral_rate_brake_capture_config = neutral_rate_brake_capture_configuration(
                profile
            )
        # 三组实验扩展（neutral rate brake capture / stance support / quiet target）
        # 各自维护 64 容量的"旧口径"影子历史，只做诊断对照，不参与新决策。
        self.stance_support_handoff_config = stance_support_handoff_configuration(profile)
        self.stance_old_brake_history = deque(maxlen=64)
        self.stance_old_quiet_history = deque(maxlen=64)
        self.stance_old_capture_last_decision = None
        self.stance_old_capture_last_sample = None
        self.neutral_first_natural_zero_publication = None
        self.neutral_first_natural_zero_ack = None
        self.neutral_old_brake_history = deque(maxlen=64)
        self.neutral_old_quiet_history = deque(maxlen=64)
        self.neutral_old_capture_last_decision = None
        self.neutral_old_capture_last_sample = None
        # r85_active_turn_braking：默认 false；开启后转向结束走"主动刹车 + 零速握手"，
        # 而不是直接放行。只接受 {"enabled": bool} 单键。
        braking = profile.get("r85_active_turn_braking", {"enabled": False})
        if (
            not isinstance(braking, dict)
            or set(braking) != {"enabled"}
            or type(braking.get("enabled")) is not bool
        ):
            raise ValueError("r85_active_turn_braking requires only explicit boolean enabled")
        self.active_braking_enabled = braking["enabled"]
        # ordinary_turn_control_handoff：基值 enabled=true + schema v1，允许转向结束时
        # 用连续控制直接交回既有 PID drive，不插入零速边沿；它与最小诊断基线、主动
        # 刹车、非外部 heading 门互斥，非法组合在下面直接 raise。
        continuous = profile.get("ordinary_turn_control_handoff", {"enabled": False})
        if not isinstance(continuous, dict) or type(continuous.get("enabled")) is not bool:
            raise ValueError("Explicit ordinary turn control handoff configuration required")
        expected = (
            {"enabled": True, "schema": "ordinary_turn_continuous_control/v1"}
            if continuous["enabled"]
            else {"enabled": False}
        )
        if continuous != expected:
            raise ValueError("Unknown ordinary turn control handoff configuration")
        self.continuous_turn_handoff_enabled = continuous["enabled"]
        if self.continuous_turn_handoff_enabled and (
            self.minimal_simulation_baseline
            or self.active_braking_enabled
            or profile.get("cascade", {}).get("external_heading_gate") is not True
        ):
            raise ValueError(
                "Continuous ordinary handoff requires original formal external heading control"
            )
        self.continuous_turn_handoff_count = 0
        if self.minimal_simulation_baseline and self.active_braking_enabled:
            raise ValueError("R73 diagnostic baseline requires active braking disabled")
        self.active_braking_lower_ack_config = None
        self.active_braking_command_limits = None
        if self.active_braking_enabled:
            from embodied_navigation.robot.unitree.policy.lower_loop_ack import (
                configuration as lower_ack_configuration,
            )

            cascade = profile.get("cascade")
            if not isinstance(cascade, dict):
                raise ValueError("Active braking requires the original cascade ACK configuration")
            self.active_braking_lower_ack_config = lower_ack_configuration(
                cascade.get("r85_student_lower_loop_ack")
            )
            # cascade.command_limits 的顺序是 [前向 m/s, 侧向 m/s, 偏航 rad/s]（与
            # cascade_core 中 [1.0, 0.4, 1.0] 的同序上限检查一致）；基值 [0.6, 0.2, 0.6]，
            # 这里的 [0.8, 0.35, 0.8] 只是缺键时的回退默认值。
            limits = cascade.get("command_limits", [0.8, 0.35, 0.8])
            if (
                self.active_braking_lower_ack_config is None
                or not isinstance(limits, (list, tuple))
                or len(limits) != 3
                or any(
                    type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in limits
                )
            ):
                raise ValueError(
                    "Active braking requires frozen Student lower ACK and command limits"
                )
            self.active_braking_command_limits = copy.deepcopy(limits)
        # r85_quiet_handoff_target：默认 false；开启后额外要求"低总命令 + 低 yaw 速率"
        # 的静默窗口才允许零速交接，并要求主动刹车与 locked_heading_pd_v1 参考。
        quiet = profile.get("r85_quiet_handoff_target", {"enabled": False})
        if (
            not isinstance(quiet, dict)
            or set(quiet) != {"enabled"}
            or type(quiet.get("enabled")) is not bool
        ):
            raise ValueError("r85_quiet_handoff_target requires only explicit boolean enabled")
        self.quiet_target_enabled = quiet["enabled"]
        self.integral_unload_config = None
        self.integral_unload_event = None
        self.integral_unload_previous_zero = None
        self.integral_unload_failed_event_id = None
        if profile.get("r85_pre_zero_integral_unload", {}).get("enabled") is True:
            from embodied_navigation.control.parking.integral_unload import configuration

            self.integral_unload_config = configuration(profile)
        if self.quiet_target_enabled and (
            not self.active_braking_enabled
            or profile.get("cascade", {}).get("external_heading_gate") is not True
            or profile.get("cascade", {}).get("r85_turn_brake_reference")
            != {"kind": "locked_heading_pd_v1"}
        ):
            raise ValueError("Quiet handoff target requires active locked-heading braking")
        # 取原始 5 s 漂移预算的一半再摊到 5 s：xy = 0.05/2/5 = 0.005 m/s，
        # yaw = 0.1/2/5 = 0.01 rad/s，heading = 0.005 rad。这些只是保守的命令目标，
        # 不声称"给出这样的速度命令就一定能得到对应的物理漂移"。
        self.quiet_limits = dict(xy=0.05 / 2 / 5.0, yaw=0.1 / 2 / 5.0, heading=0.005)
        # 两个实验口径会改写静默带：neutral 放宽到完整 turn_complete_rad(0.1 rad)；
        # stance 收紧到 0.25*turn_complete_rad(0.025 rad)，并把命令 yaw 上限降到
        # 0.035 rad/s（默认 quiet_command_yaw_cap 仍是 0.01 rad/s）。
        if self.neutral_rate_brake_capture_config is not None:
            self.quiet_limits["heading"] = self.config["turn_complete_rad"]
        self.quiet_command_yaw_cap = self.quiet_limits["yaw"]
        if self.stance_support_handoff_config is not None:
            self.quiet_limits["heading"] = 0.25 * self.config["turn_complete_rad"]
            self.quiet_command_yaw_cap = 0.035
        self.quiet_history = deque(maxlen=64)
        self.quiet_publication = None
        self.quiet_binding = None
        self.quiet_pose_context = None
        self.quiet_observed_stamp_ns = None
        self.quiet_ack_watermark = None
        self.quiet_reason = "not_observed"
        self.last_quiet_qualification = None
        self.brake_history = deque(maxlen=64)
        self.brake_last_stamp = None
        self.handoff_failure = None
        self.handoff_event = None
        self.handoff_deadline_s = None
        self.handoff_deadline_candidate = None
        self.handoff_deadline_binding = None
        self.zero_ack_candidate = None
        self.zero_ack_error = None
        self.zero_ack_highwater = None
        self.last_brake_qualification = None
        self.final_zero_ack = None
        diagnostic = profile.get("diagnostic_observe_only", {})
        self.diagnostic_profile = dict(
            diagnostic_observe_only=dict(
                enabled=bool(
                    isinstance(diagnostic, dict)
                    and type(diagnostic.get("enabled")) is bool
                    and diagnostic["enabled"]
                )
            )
        )
        self.diagnostic_run = None
        self.diagnostic_zero_ack_ready = False
        # 1 s 稳态统计窗：与 legacy 300 ms 判据是 OR 关系（见 settle 的释放分支），
        # 每样本有效性仍沿用 teacher_transition 的瞬时门。
        self.stop_window = NavigationStopWindow(
            profile.get("navigation_settle_statistics_contract"), self.config
        )
        self.transition_purpose = "unknown"
        self.transition_context = {}
        self.last_release = None
        self.zero_reconvergence_count = 0
        self.last_zero_reconvergence = None
        self.last_stamp = None
        self.latest_motion = None
        self.zero_handoff_count = 0
        # 初始相位是 pre_turn：第一次 update 必须先证明静止才会转向（drive 除外）。
        self.reset()

    @staticmethod
    def error(target, yaw):
        """返回 target - yaw 的归一化角度差，单位 rad，落在闭区间 [-pi, pi]。"""
        return math.atan2(math.sin(target - yaw), math.cos(target - yaw))

    def reset(self):
        """回到 pre_turn 并清空本相位的观测与握手状态。

        副作用：进行中的主动刹车握手先被标记为失败（active_handoff_reset）；已经
        存在失败时只把相位钉在 failed 并返回，不再清理任何证据。
        """
        if self.handoff_active:
            self.fail_handoff("active_handoff_reset")
        if self.handoff_failure:
            self.phase = "failed"
            return
        self.handoff_event = None
        self.handoff_deadline_s = None
        if self.quiet_target_enabled:
            self._clear_quiet("reset")
            self.quiet_publication = None
            self.quiet_binding = None
            self.quiet_ack_watermark = None
            self.quiet_pose_context = None
        self.brake_history.clear()
        self.brake_last_stamp = None
        self.final_zero_ack = None
        self.phase = "pre_turn"
        self.heading = None
        self.settle_until = None
        self.outside_since = None
        self.stop_window.clear()
        self.last_release = None
        self.settle_control_stage = None
        self.last_zero_handoff = None
        # goal/replan 复位只重置 heading，不清除真实观测到的静止历史：history 在 reset
        # 里被有意保留（stop_window 的统计窗则清空），避免用一次复位抹掉实测证据。

    def set_transition_context(self, purpose, context):
        """绑定本帧的转移目的与身份（request_id、waypoint_index 等）。

        参数：purpose 为转移目的，决定统计分支是否生效（取值见
        navigation_stop_window.SPEC["eligible_purposes"]）；context 为身份字典。
        语义：进行中握手遇到目的变化或上下文变化即失败；身份变化会清掉旧失败、旧
        握手与旧 ACK 水位；目的或上下文任一变化都会清空统计窗。
        """
        changed = context != self.transition_context
        old_identity = (
            self.transition_context.get("request_id"),
            self.transition_context.get("waypoint_index"),
        )
        new_identity = (context.get("request_id"), context.get("waypoint_index"))
        was_active = self.handoff_active
        if was_active and (changed or purpose != self.transition_purpose):
            self.fail_handoff("active_handoff_binding_changed")
        elif old_identity != new_identity:
            self.handoff_failure = None
            self.handoff_event = None
            self.handoff_deadline_s = None
            self.handoff_deadline_candidate = None
            self.handoff_deadline_binding = None
            self.zero_ack_candidate = None
            self.zero_ack_highwater = None
            self.final_zero_ack = None
            self.brake_history.clear()
            self.brake_last_stamp = None
            if self.phase == "failed":
                self.phase = "pre_turn"
                self.heading = None
                self.settle_control_stage = None
        if purpose != self.transition_purpose or changed:
            self.stop_window.clear()
        self.transition_purpose = purpose
        self.transition_context = dict(context)

    @property
    def controlled_braking(self):
        """当前是否处于 settle 的主动刹车阶段（controlled_braking）。"""
        return self.phase == "settle" and self.settle_control_stage == "controlled_braking"

    @property
    def handoff_active(self):
        """是否存在进行中（status=pending）的主动刹车握手。"""
        return bool(
            self.active_braking_enabled
            and self.handoff_event is not None
            and self.handoff_event.get("status") == "pending"
        )

    def fail_handoff(self, reason, sim_time=None):
        """把进行中的握手标记为失败，并把相位钉在 failed。

        参数：reason 为失败原因字符串；sim_time 为失败时的控制时间（秒），写进事件。
        返回：确实从 pending 转为失败时 True，否则 False（幂等，重复调用无副作用）。
        副作用：清空历史、统计窗、刹车历史与静默历史；积分卸载失败 id 只保留首个。
        """
        if not self.handoff_active:
            return False
        self.handoff_failure = self.handoff_failure or str(reason)
        if self.integral_unload_config is not None and self.integral_unload_event is not None:
            self.integral_unload_failed_event_id = (
                self.integral_unload_failed_event_id or self.integral_unload_event["event_id"]
            )
        self.handoff_event.update(
            status="failed", failure=self.handoff_failure, failure_control_s=sim_time
        )
        self.phase = "failed"
        self.settle_control_stage = None
        self.history.clear()
        self.stop_window.clear()
        self.brake_history.clear()
        if self.quiet_target_enabled:
            self._clear_quiet("handoff_failed")
        return True

    def bind_integral_unload(self, run_id, scope, path_sha256, goal_sha256, clock_ns):
        """把当前握手绑定到 R16 协调积分卸载事件。

        参数：run_id/scope/path_sha256/goal_sha256 为身份；clock_ns 为控制时钟（纳秒）。
        返回：事件副本；未启用积分卸载、或当前不在主动刹车时返回 None。
        语义：同一个 pending 条目始终保留最初的路径身份、过期时间与哈希；合法的
        路径刷新只重启静默采样，不重建握手事件。
        失败：先前事件已失败或绑定被改写时判握手失败并返回 None。
        """
        if self.integral_unload_config is None:
            return None
        if not self.handoff_active or not self.controlled_braking:
            return None
        if self.integral_unload_failed_event_id is not None:
            self.fail_handoff("pre_zero_integral_unload_previous_failure", clock_ns / 1e9)
            return None
        from embodied_navigation.control.parking.integral_unload import make_event

        old = self.integral_unload_event
        # 同一个受管的 pending 条目始终保留最初的路径身份、过期时间与哈希；合法的
        # 路径刷新只重启静默采样，不改写事件身份。
        initial_path = (
            old["initial_path_sha256"]
            if old is not None
            and old["brake_entry_clock_ns"]
            == round(self.handoff_event["brake_entry_control_s"] * 1e9)
            else path_sha256
        )
        event = make_event(
            run_id,
            scope,
            self.handoff_event,
            initial_path,
            goal_sha256,
            self.zero_handoff_count,
            self.integral_unload_previous_zero,
        )
        if old is None:
            self.integral_unload_event = event
        elif event != old:
            from embodied_navigation.control.parking.integral_unload import successor

            if successor(old, event):
                self.integral_unload_event = event
            else:
                self.fail_handoff("pre_zero_integral_unload_binding_changed", clock_ns / 1e9)
                return None
        self.check_integral_unload_deadline(clock_ns)
        return copy.deepcopy(self.integral_unload_event) if self.handoff_active else None

    def check_integral_unload_deadline(self, clock_ns):
        """主动刹车期间检查积分卸载事件是否过期；过期即判握手失败。"""
        event = self.integral_unload_event
        if (
            event is not None
            and self.handoff_active
            and self.controlled_braking
            and clock_ns >= event["expires_clock_ns"]
        ):
            self.fail_handoff("pre_zero_integral_unload_timeout", clock_ns / 1e9)

    def _unloaded_actual_pair(self, publication, ack):
        """核对发布侧与 ACK 侧的积分卸载回执是否同属一个零事件。

        返回：两侧都证明 yaw 积分归零、且路径/目标哈希一致时 True；未启用时恒 True。
        语义：原生"精确零 I 观测"分支只观察复位状态，本身不授权事件；当前 Teacher
        事件与原有因果/静默前提仍然约束这一对证据。
        失败：缺回执、哈希不一致或形状非法一律返回 False（不向调用方抛出）。
        """
        if self.integral_unload_config is None:
            return True
        try:
            from embodied_navigation.control.parking.integral_unload import validate_receipt

            e = self.integral_unload_event
            if e is None:
                return False
            teacher = validate_receipt(
                publication.get("pre_zero_integral_unload"),
                event=e,
                side="teacher",
                scope=publication["scope"],
                clock_ns=publication["control_clock_ns"],
            )
            native = ack.get("pre_zero_integral_unload")
            if (
                isinstance(native, dict)
                and native.get("kind") == "original_exact_zero_I_observation"
            ):
                from embodied_navigation.control.parking.integral_unload import (
                    validate_native_extension,
                )

                validate_native_extension(
                    native, ack["run_id"], ack["input_clock_ns"], ack["input_frame_serial"]
                )
                # 这一支只观察原生复位状态，本身不授权事件；当前发布的 Teacher 事件与
                # 所有原有因果/静默前提仍然约束这一对证据。
                return bool(
                    teacher["yaw_integral_after"] == 0.0
                    and ack["exact_zero"]
                    and ack["input_clock_ns"] > e["brake_entry_clock_ns"]
                    and teacher["path_sha256"] == publication["path_sha256"]
                    and teacher["goal_sha256"] == publication["goal_sha256"]
                )
            native = validate_receipt(
                native,
                event=e,
                side="student",
                scope=ack["run_id"],
                clock_ns=ack["input_clock_ns"],
                frame=ack["input_frame_serial"],
            )
            return bool(
                teacher["yaw_integral_after"] == 0.0
                and native["yaw_integral_after"] == 0.0
                and teacher["path_sha256"] == publication["path_sha256"] == native["path_sha256"]
                and teacher["goal_sha256"] == publication["goal_sha256"] == native["goal_sha256"]
            )
        except (ValueError, KeyError, TypeError):
            return False

    def _post_entry_brake_heading_band(self):
        """刹车入口之后的航向中心带（rad）：默认 0.25*turn_complete_rad；neutral 口径取全带。"""
        return (
            self.config["turn_complete_rad"]
            if self.neutral_rate_brake_capture_config is not None
            else 0.25 * self.config["turn_complete_rad"]
        )

    def _clear_quiet(self, reason):
        """清空静默历史并记录原因；实验扩展的影子历史一并清空。"""
        self.quiet_history.clear()
        self.quiet_reason = str(reason)
        if self.neutral_rate_brake_capture_config is not None:
            self.neutral_old_quiet_history.clear()
            self.neutral_old_brake_history.clear()
        if self.stance_support_handoff_config is not None:
            self.stance_old_quiet_history.clear()
            self.stance_old_brake_history.clear()

    @staticmethod
    def _quiet_binding(context, purpose, scope, path_id, path_sha256, goal_sha256):
        """组装静默证据的绑定额（上下文/目的/scope/路径 id/路径与目标哈希）。"""
        return dict(
            context=copy.deepcopy(context),
            purpose=purpose,
            scope=scope,
            path_id=path_id,
            path_sha256=path_sha256,
            goal_sha256=goal_sha256,
        )

    def _low_total_command(self, value, yaw_cap=None):
        """判断一条三元素机体命令是否"总命令足够低"。

        判据：平面速度范数 <= quiet_limits["xy"]（0.005 m/s），且 |yaw| <= yaw_cap
        （默认 quiet_command_yaw_cap，单位 rad/s）。形状非法或非有限值直接 False。
        """
        return bool(
            isinstance(value, (list, tuple))
            and len(value) == 3
            and all(
                isinstance(v, Real) and not isinstance(v, bool) and math.isfinite(v) for v in value
            )
            and math.hypot(value[0], value[1]) <= self.quiet_limits["xy"]
            and abs(value[2]) <= (self.quiet_command_yaw_cap if yaw_cap is None else yaw_cap)
        )

    @staticmethod
    def _quiet_publication_fresh(record, clock_ns, wall_ns):
        """检查发布记录在仿真时钟与墙钟两个域是否都足够新鲜（均 < 300 ms）。

        300 ms 是来源新鲜度上限（硬编码 3e8 ns），不是 pose/gyro 配对窗（停车样本的
        配对窗是 teacher_transition.gyro_pose_pair_max_s 的 150 ms，零契约里另用
        20 ms 判配对）。两个时钟域分别相减，绝不相减。
        """
        return bool(
            all(
                0 <= clock_ns - record[k] < 300_000_000
                for k in (
                    "publish_clock_ns",
                    "control_clock_ns",
                    "source_pose_stamp_ns",
                    "paired_imu_stamp_ns",
                )
            )
            and all(
                0 <= wall_ns - record[k] < 300_000_000
                for k in (
                    "publish_wall_ns",
                    "control_wall_ns",
                    "source_pose_received_wall_ns",
                    "source_imu_received_wall_ns",
                )
            )
        )

    def note_quiet_publication(self, record):
        """登记一条"低总命令 + 来源新鲜"的正常刹车发布，作为静默交接的发布侧证据。

        参数：record 为发布记录（含命令、上下文、时钟与路径身份）。
        返回：登记成功 True；未启用静默目标或任一门不通过则 False（并清空静默历史）。
        语义：上下文/目的/scope/路径任一不符即判不静默；同一目标的路径刷新只重启
        内部静默窗口，不重置握手事件与预算。
        """
        if not self.quiet_target_enabled:
            return False
        self.quiet_publication = None
        try:
            if not (
                self.controlled_braking
                and self.handoff_active
                and isinstance(record, dict)
                and record["normal_braking"] is True
                and self._low_total_command(record["command_body"])
                and record["context"] == self.transition_context
                and record["purpose"] == self.transition_purpose
                and record["context"] == self.handoff_event["context"]
            ):
                raise ValueError("publication_not_quiet_brake_math")
            clocks = (
                "sequence",
                "publish_clock_ns",
                "publish_wall_ns",
                "control_clock_ns",
                "control_wall_ns",
                "source_pose_stamp_ns",
                "paired_imu_stamp_ns",
                "source_pose_received_wall_ns",
                "source_imu_received_wall_ns",
            )
            if any(type(record[k]) is not int or record[k] < 0 for k in clocks):
                raise ValueError("publication_integer_clocks_required")
            if not (
                self._quiet_publication_fresh(
                    record, record["publish_clock_ns"], record["publish_wall_ns"]
                )
                # 20 ms 是 pose 与配对陀螺的因果配对上限（硬编码），与 300 ms 源龄、
                # 150 ms 停车配对窗回答的是不同问题；发布还必须晚于刹车入口。
                and 0
                <= record["source_pose_stamp_ns"] - record["paired_imu_stamp_ns"]
                <= 20_000_000
                and record["control_clock_ns"] / 1e9 >= self.handoff_event["brake_entry_control_s"]
            ):
                raise ValueError("publication_source_not_fresh")
            binding = self._quiet_binding(
                *(
                    record[k]
                    for k in (
                        "context",
                        "purpose",
                        "scope",
                        "path_id",
                        "path_sha256",
                        "goal_sha256",
                    )
                )
            )
            if not (
                isinstance(record["path_id"], str)
                and record["path_id"]
                and all(
                    isinstance(record[k], str) and len(record[k]) == 64
                    for k in ("scope", "path_sha256", "goal_sha256")
                )
            ):
                raise ValueError("publication_source_binding_missing")
            if self.quiet_binding is not None and binding != self.quiet_binding:
                if any(
                    binding[k] != self.quiet_binding[k]
                    for k in ("context", "purpose", "scope", "goal_sha256")
                ):
                    raise ValueError("publication_source_binding_changed")
                # 同目标、合法的 SCAN 路径刷新只重启这个内部目标窗口，绝不重置握手
                # 事件或预算（预算被刷新等于无限延长等待）。
                self._clear_quiet("publication_path_refreshed")
            # 启用积分卸载时，Teacher 实际发布的 I 必须与事件回执一致且为 0，
            # 否则这条发布不能当静默证据。
            if self.integral_unload_config is not None:
                from embodied_navigation.control.parking.integral_unload import validate_receipt

                proof = validate_receipt(
                    record.get("pre_zero_integral_unload"),
                    event=self.integral_unload_event,
                    side="teacher",
                    scope=record["scope"],
                    clock_ns=record["control_clock_ns"],
                )
                if (
                    proof["yaw_integral_after"] != 0.0
                    or record.get("yaw_integral_state") != proof["yaw_integral_after"]
                    or record.get("integral_dt_s") != proof["dt_s"]
                ):
                    raise ValueError("actual_published_Teacher_I_not_zero")
            self.quiet_binding = copy.deepcopy(binding)
            self.quiet_publication = copy.deepcopy(record)
            # 两个位姿之间若出现高支撑发布，即使它之后、下一位姿之前还有低发布，
            # 也必须打断旧的严格影子（否则旧口径影子会被"夹"出来）。
            if self.stance_support_handoff_config is not None and not self._low_total_command(
                record["command_body"], 0.01
            ):
                self.stance_old_quiet_history.clear()
            if (
                self.neutral_rate_brake_capture_config is not None
                and not any(record["command_body"])
                and self.neutral_first_natural_zero_publication is None
            ):
                self.neutral_first_natural_zero_publication = copy.deepcopy(record)
            return True
        except (ValueError, TypeError, KeyError) as error:
            self._clear_quiet(error)
            return False

    def begin_quiet_pose(
        self,
        stamp_ns,
        control_clock_ns,
        wall_ns,
        context,
        purpose,
        scope,
        path_id,
        path_sha256,
        goal_sha256,
    ):
        """记录本次位姿观测的绑定额；必须在每个接受位姿的观测开始前调用。

        参数：stamp_ns 为位姿 stamp（纳秒）；control_clock_ns 为控制时钟（纳秒）；
        wall_ns 为接收墙钟（纳秒）；其余为身份绑定字段。
        """
        if not self.quiet_target_enabled:
            return
        self.quiet_observed_stamp_ns = None
        self.quiet_pose_context = dict(
            stamp_ns=stamp_ns,
            clock_ns=control_clock_ns,
            wall_ns=wall_ns,
            binding=self._quiet_binding(context, purpose, scope, path_id, path_sha256, goal_sha256),
        )

    def finish_quiet_pose(self, stamp_ns, accepted):
        """位姿观测收尾：接受的位姿若没有对应的合格运动观测，就清空静默历史。"""
        if self.quiet_target_enabled and accepted and self.quiet_observed_stamp_ns != stamp_ns:
            self._clear_quiet("accepted_pose_without_qualified_motion_observation")

    def _observe_quiet(self, stamp_s, wall_s, brake_valid, observed_yaw):
        """在主动刹车中观察一次"静默"样本：发布、ACK、运动与航向必须同时成立。

        参数：stamp_s 为位姿 stamp（秒，仿真时钟）；wall_s 为接收墙钟（秒）；
        brake_valid 为本次刹车样本是否合格；observed_yaw 为四元数解析出的偏航（rad）。
        语义：任一门不通过都会清空静默历史并记录原因；通过则追加样本、推进 ACK 水位
        并同步实验扩展的影子样本。失败不抛异常，只"清空 + 记原因"。
        """
        stamp_ns = round(stamp_s * 1e9)
        self.quiet_observed_stamp_ns = stamp_ns
        p = self.quiet_publication
        obs = self.quiet_pose_context
        ack = self.zero_ack_candidate
        m = self.latest_motion
        try:
            if not (
                brake_valid
                and p is not None
                and obs is not None
                and ack is not None
                and obs["stamp_ns"] == stamp_ns
                and obs["binding"] == self.quiet_binding
                and p["context"] == self.transition_context
                and p["purpose"] == self.transition_purpose
                and abs(self.error(self.heading, observed_yaw)) <= self.quiet_limits["heading"]
                and abs(m["slam_wz"]) <= self.quiet_limits["yaw"]
                and abs(m["body_gyro"][2]) <= self.quiet_limits["yaw"]
            ):
                raise ValueError("motion_heading_or_publication_not_quiet")
            # 发布与 ACK 都要在两个时钟域内 300 ms 新鲜；ACK 必须属于同一 run/request/
            # waypoint、晚于刹车入口，且请求命令与实际命令都"总命令足够低"。
            if not (
                self._quiet_publication_fresh(p, obs["clock_ns"], round(wall_s * 1e9))
                and 0 <= stamp_ns - ack["input_clock_ns"] < 300_000_000
                and 0 <= round(wall_s * 1e9) - ack["received_wall_ns"] < 300_000_000
                and ack["run_id"] == p["scope"]
                and (ack["request_id"], ack["waypoint_index"])
                == (
                    self.transition_context.get("request_id"),
                    self.transition_context.get("waypoint_index"),
                )
                and ack["input_clock_ns"] > round(self.handoff_event["brake_entry_control_s"] * 1e9)
                and self._low_total_command(ack["requested_command_body"])
                and self._low_total_command(ack["applied_command_body"])
            ):
                raise ValueError("actual_commands_or_causal_ACK_not_quiet")
            if not self._unloaded_actual_pair(p, ack):
                raise ValueError("actual_same_event_integrals_not_zero")
            watermark = tuple(
                ack[k]
                for k in ("sequence", "input_clock_ns", "input_frame_serial", "inference_sequence")
            )
            # ACK 水位四元组必须严格前进：同一推理帧不得被两个静默样本重复消费。
            if self.quiet_ack_watermark is not None and any(
                a <= b for a, b in zip(watermark, self.quiet_ack_watermark)
            ):
                raise ValueError("quiet_ACK_frame_not_independent")
            self.quiet_ack_watermark = watermark
            # 与上一样本间隔超过 stop_pose_gap_max_s（基值 0.2 s）即视为窗口断裂，
            # 之前累积的样本不能跨过这个空档继续计数。
            if (
                self.quiet_history
                and stamp_s - self.quiet_history[-1]["stamp_s"] > self.config["stop_pose_gap_max_s"]
            ):
                self._clear_quiet("quiet_pose_gap")
            self.quiet_history.append(
                dict(
                    stamp_s=stamp_s,
                    wall_s=wall_s,
                    publication_sequence=p["sequence"],
                    ack_watermark=list(watermark),
                    heading_error_rad=self.error(self.heading, observed_yaw),
                    slam_wz=m["slam_wz"],
                    body_wz=m["body_gyro"][2],
                )
            )
            self.quiet_reason = None
            # neutral 口径的影子样本：只在"新判定"通过后记录旧口径的两个瞬时条件，
            # 0.25*turn_complete_rad 记 brake，0.005 rad 记 quiet。
            if self.neutral_rate_brake_capture_config is not None:
                error = self.quiet_history[-1]["heading_error_rad"]
                if abs(error) <= 0.25 * self.config["turn_complete_rad"]:
                    self.neutral_old_brake_history.append((stamp_s, wall_s))
                else:
                    self.neutral_old_brake_history.clear()
                if abs(error) <= 0.005:
                    self.neutral_old_quiet_history.append(copy.deepcopy(self.quiet_history[-1]))
                else:
                    self.neutral_old_quiet_history.clear()
                self.neutral_old_capture_last_sample.update(
                    source_and_ACK_qualified=True,
                    ACK_watermark=list(watermark),
                    old_brake_instantaneous=abs(error) <= 0.25 * self.config["turn_complete_rad"],
                    old_quiet_instantaneous=abs(error) <= 0.005,
                )
            # stance 口径的影子样本：旧上限还要求发布/请求/实际三条命令都在
            # 0.01 rad/s 以内（与新的 quiet_command_yaw_cap 不同）。
            if self.stance_support_handoff_config is not None:
                sample = copy.deepcopy(self.quiet_history[-1])
                error = sample["heading_error_rad"]
                old_commands = all(
                    self._low_total_command(command, 0.01)
                    for command in (
                        p["command_body"],
                        ack["requested_command_body"],
                        ack["applied_command_body"],
                    )
                )
                old_brake = abs(error) <= 0.25 * self.config["turn_complete_rad"]
                old_quiet = abs(error) <= 0.005 and old_commands
                if old_brake:
                    self.stance_old_brake_history.append((stamp_s, wall_s))
                else:
                    self.stance_old_brake_history.clear()
                if old_quiet:
                    self.stance_old_quiet_history.append(sample)
                else:
                    self.stance_old_quiet_history.clear()
                self.stance_old_capture_last_sample.update(
                    source_and_ACK_qualified=True,
                    ACK_watermark=list(watermark),
                    old_brake_instantaneous=old_brake,
                    old_quiet_instantaneous=old_quiet,
                    old_command_caps_passed=old_commands,
                    published_command_body=list(p["command_body"]),
                    ACK_requested_command_body=list(ack["requested_command_body"]),
                    ACK_applied_command_body=list(ack["applied_command_body"]),
                )
        except (ValueError, TypeError, KeyError) as error:
            self._clear_quiet(error)

    def _quiet_qualified(self, sim_time, wall_time, error):
        """判断静默目标是否已连续成立（发布、ACK、运动与锁定航向的联合判定）。

        返回：样本数 >= stop_min_pose_samples 且跨度 >= stop_window_sim_s 时 True。
        语义：任一来源/绑定/命令不再满足即清空静默历史并返回 False；这里只证明
        "内部目标成立"，不证明原始停车 PASS（last_quiet_qualification 里显式标注）。
        """
        if not self.quiet_target_enabled:
            return True
        h = self.quiet_history
        p = self.quiet_publication
        ack = self.zero_ack_candidate
        clock_ns = round(sim_time * 1e9)
        wall_ns = round(wall_time * 1e9)
        if not (
            h
            and p is not None
            and ack is not None
            and abs(error) <= self.quiet_limits["heading"]
            and h[-1]["stamp_s"] == self.latest_motion["stamp_s"]
            # 墙钟域只看 300 ms 上界；仿真域额外允许 50 ms 未来容差（-0.05），
            # 与导航侧新鲜度表达式同形，都是硬编码字面量。
            and 0 <= wall_time - h[-1]["wall_s"] < 0.3
            and -0.05 <= sim_time - h[-1]["stamp_s"] < 0.3
            and self._quiet_publication_fresh(p, clock_ns, wall_ns)
            and 0 <= clock_ns - ack["input_clock_ns"] < 300_000_000
            and 0 <= wall_ns - ack["received_wall_ns"] < 300_000_000
            and ack["ack_native_clock_ns"] <= clock_ns
            and ack["input_clock_ns"] <= round(self.latest_motion["stamp_s"] * 1e9)
            # 最近静默样本记录的 ACK 水位必须与当前 ACK 完全一致：判定用的样本与
            # 判定时刻的 ACK 不能是两帧。
            and [
                ack[k]
                for k in ("sequence", "input_clock_ns", "input_frame_serial", "inference_sequence")
            ]
            == h[-1]["ack_watermark"]
            and ack["run_id"] == p["scope"]
            and (ack["request_id"], ack["waypoint_index"])
            == (
                self.transition_context.get("request_id"),
                self.transition_context.get("waypoint_index"),
            )
            and p["context"] == self.transition_context
            and p["purpose"] == self.transition_purpose
            and self._low_total_command(p["command_body"])
            and self._low_total_command(ack["requested_command_body"])
            and self._low_total_command(ack["applied_command_body"])
        ):
            self._clear_quiet("quiet_target_not_current")
            return False
        if not self._unloaded_actual_pair(p, ack):
            self._clear_quiet("actual_same_event_integrals_not_zero")
            return False
        # 两个条件共用 teacher_transition：至少 stop_min_pose_samples(3) 个样本，且首尾
        # 跨度 >= stop_window_sim_s(0.3 s)；1e-9 只是浮点窗口边界松弛，不是几何容差。
        ready = (
            len(h) >= self.config["stop_min_pose_samples"]
            and h[-1]["stamp_s"] - h[0]["stamp_s"] >= self.config["stop_window_sim_s"] - 1e-9
        )
        if ready:
            self.last_quiet_qualification = dict(
                first_stamp_s=h[0]["stamp_s"],
                last_stamp_s=h[-1]["stamp_s"],
                samples=len(h),
                actual_control_s=sim_time,
                last_publication=copy.deepcopy(p),
                last_pose=copy.deepcopy(h[-1]),
                low_total_command_target_only=True,
                original_stop_PASS=False,
            )
        return ready

    def _record_neutral_old_capture_decision(
        self, sim_time, wall_time, error, new_brake_ready, new_quiet_ready
    ):
        """记录 neutral 旧口径的诊断对照决策；不改变任何新判定。

        参数：new_brake_ready/new_quiet_ready 为本次新判定的中间结果；error 为锁定
        航向误差（rad）。纯记录：结果字典显式带 diagnostic_only 与
        affects_new_decision=False。
        """
        if self.neutral_rate_brake_capture_config is None:
            return
        b = self.neutral_old_brake_history
        q = self.neutral_old_quiet_history
        c = self.config
        latest = self.latest_motion["stamp_s"] if self.latest_motion is not None else None
        old_brake = bool(
            new_brake_ready
            and abs(error) <= 0.25 * c["turn_complete_rad"]
            and len(b) >= c["stop_min_pose_samples"]
            and b[-1][0] == latest
            and b[-1][0] - b[0][0] >= c["stop_window_sim_s"] - 1e-9
        )
        old_quiet = bool(
            new_quiet_ready
            and abs(error) <= 0.005
            and len(q) >= c["stop_min_pose_samples"]
            and q[-1]["stamp_s"] == latest
            and q[-1]["stamp_s"] - q[0]["stamp_s"] >= c["stop_window_sim_s"] - 1e-9
        )
        self.neutral_old_capture_last_decision = dict(
            control_s=sim_time,
            wall_s=wall_time,
            locked_heading_error_rad=error,
            new_brake_ready=bool(new_brake_ready),
            new_quiet_ready=bool(new_quiet_ready),
            old_brake_ready=old_brake,
            old_quiet_ready=old_quiet,
            old_internal_capture_ready=old_brake and old_quiet,
            diagnostic_only=True,
            ACK_watermark_consumed_again=False,
            affects_new_decision=False,
        )

    def _record_stance_old_capture_decision(
        self, sim_time, wall_time, error, new_brake_ready, new_quiet_ready
    ):
        """记录 stance 旧口径的诊断对照决策；不改变任何新判定。

        参数同 _record_neutral_old_capture_decision；结果字典显式标注 diagnostic_only
        与 affects_new_decision=False，避免被误读成交替判定。
        """
        if self.stance_support_handoff_config is None:
            return
        b = self.stance_old_brake_history
        q = self.stance_old_quiet_history
        c = self.config
        latest = self.latest_motion["stamp_s"] if self.latest_motion is not None else None
        old_brake = bool(
            new_brake_ready
            and abs(error) <= 0.25 * c["turn_complete_rad"]
            and len(b) >= c["stop_min_pose_samples"]
            and b[-1][0] == latest
            and b[-1][0] - b[0][0] >= c["stop_window_sim_s"] - 1e-9
        )
        old_quiet = bool(
            new_quiet_ready
            and abs(error) <= 0.005
            and len(q) >= c["stop_min_pose_samples"]
            and q[-1]["stamp_s"] == latest
            and q[-1]["stamp_s"] - q[0]["stamp_s"] >= c["stop_window_sim_s"] - 1e-9
        )
        self.stance_old_capture_last_decision = dict(
            control_s=sim_time,
            wall_s=wall_time,
            locked_heading_error_rad=error,
            new_brake_ready=bool(new_brake_ready),
            new_quiet_ready=bool(new_quiet_ready),
            old_brake_ready=old_brake,
            old_quiet_ready=old_quiet,
            old_internal_capture_ready=old_brake and old_quiet,
            diagnostic_only=True,
            ACK_watermark_consumed_again=False,
            affects_new_decision=False,
        )

    def bind_handoff_deadline(self, deadline_s, binding):
        """绑定本次转移的绝对握手截止时间（秒）与身份。

        参数：deadline_s 为绝对控制时钟（秒，>=0 的有限数）；binding 为身份字典。
        语义：身份与本帧 transition_context 不一致时不绑定（进行中握手会失败）；
        握手已在进行中时，重绑定只能在原冻结截止时间上收紧，绝不延长。
        失败：deadline 或 binding 形状非法即 raise。
        """
        if not self.active_braking_enabled:
            return
        if (
            isinstance(deadline_s, bool)
            or not isinstance(deadline_s, (int, float))
            or not math.isfinite(deadline_s)
            or deadline_s < 0
            or not isinstance(binding, dict)
        ):
            raise ValueError("Original finite absolute handoff deadline and binding required")
        bound = copy.deepcopy(binding)
        identity = (bound.get("request_id"), bound.get("waypoint_index"))
        expected = (
            self.transition_context.get("request_id"),
            self.transition_context.get("waypoint_index"),
        )
        # 身份不符：不绑定新截止时间；若正在握手则视为绑定被改写，直接失败。
        if identity != expected:
            if self.handoff_active:
                self.fail_handoff("active_handoff_binding_changed")
            self.handoff_deadline_candidate = None
            return
        if self.handoff_active:
            if bound != self.handoff_event["deadline_binding"]:
                self.fail_handoff("active_handoff_binding_changed")
                return
            # 重绑定只能收紧已冻结的原始截止时间，绝不能延长（延长等于放宽超时）。
            self.handoff_deadline_s = min(self.handoff_deadline_s, float(deadline_s))
            self.handoff_event["deadline_s"] = self.handoff_deadline_s
        self.handoff_deadline_candidate = float(deadline_s)
        self.handoff_deadline_binding = bound

    def bind_zero_ack(
        self, ack, expected_scope, request_id, waypoint_index, control_clock_ns, wall_ns
    ):
        """登记一份"当前帧、当前 scope"的下层零输入 ACK 候选。

        参数：ack 为下层 ACK；expected_scope 为冻结的来源 scope；request_id/
        waypoint_index 为当前身份；control_clock_ns 为控制时钟（纳秒）；wall_ns 为
        接收墙钟（纳秒）。
        返回：登记成功 True；任一校验失败则记录 zero_ack_error 并返回 False（不抛）。
        语义：ACK 必须属于当前 request/waypoint 与 scope；两个时钟域都 300 ms 内新鲜；
        原生时钟严格晚于输入时钟且间隔固定 20 ms；水位只能前进。
        """
        self.zero_ack_candidate = None
        self.zero_ack_error = None
        try:
            from embodied_navigation.robot.unitree.policy.lower_loop_ack import input_binding

            receipt, stamp, frame, inference, applied, scope = input_binding(ack)
            if self.active_braking_enabled:
                from embodied_navigation.robot.unitree.policy.lower_loop_ack import (
                    residual as lower_ack_residual,
                )

                # 最终精确零走核心 hold 路径，它的保护性提前返回早于常规 ACK 校验；
                # 这里复用同一个 v1 校验器，避免另写一套放宽的校验。
                lower_ack_residual(
                    ack, self.active_braking_lower_ack_config, self.active_braking_command_limits
                )
            if (
                type(control_clock_ns) is not int
                or type(wall_ns) is not int
                or type(ack["sequence"]) is not int
                or ack["sequence"] <= 0
                or type(ack["received_wall_ns"]) is not int
                or type(ack["ack_native_clock_ns"]) is not int
            ):
                raise ValueError("Original ACK/control integer clocks and sequence required")
            context = (request_id, waypoint_index)
            expected = (
                self.transition_context.get("request_id"),
                self.transition_context.get("waypoint_index"),
            )
            if context != expected or scope != expected_scope:
                raise ValueError("ACK current context or frozen source scope differs")
            # 两个时钟域各自的 300 ms 源龄：仿真域(control-stamp)与墙钟域(接收时刻)；
            # 原生时钟不得晚于本帧控制时钟，且与输入时钟固定相差 20 ms（硬编码
            # 20_000_000 ns 的接口契约，代码与现有文档未说明该间隔的推导来源）。
            if (
                not 0 <= control_clock_ns - stamp < 300_000_000
                or not 0 <= wall_ns - ack["received_wall_ns"] < 300_000_000
                or ack["ack_native_clock_ns"] > control_clock_ns
                or ack["ack_native_clock_ns"] - stamp != 20_000_000
            ):
                raise ValueError("ACK stale, future, or causal native interval differs")
            request = list(ack["requested_command_body"])
            if len(request) != 3 or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in request
            ):
                raise ValueError("Finite truthful ACK request required")
            watermark = dict(
                sequence=ack["sequence"],
                input_clock_ns=stamp,
                input_frame_serial=frame,
                inference_sequence=inference,
            )
            # 水位契约：sequence 只能前进；新 sequence 必须同时严格推进其余三个字段
            # （否则就是换了帧却没换序号）；同 sequence 不得改变内容。
            old = self.zero_ack_highwater
            if old is not None:
                keys = ("sequence", "input_clock_ns", "input_frame_serial", "inference_sequence")
                if any(watermark[k] < old[k] for k in keys):
                    raise ValueError("Actual ACK watermark went backwards")
                if watermark["sequence"] > old["sequence"] and any(
                    watermark[k] <= old[k] for k in keys[1:]
                ):
                    raise ValueError("New ACK reused an old actual inference frame")
                if watermark["sequence"] == old["sequence"] and watermark != old:
                    raise ValueError("Repeated ACK sequence changed its actual frame")
            self.zero_ack_highwater = copy.deepcopy(watermark)
            self.zero_ack_candidate = dict(
                **watermark,
                run_id=scope,
                request_id=request_id,
                waypoint_index=waypoint_index,
                received_wall_ns=ack["received_wall_ns"],
                ack_native_clock_ns=ack["ack_native_clock_ns"],
                requested_command_body=request,
                applied_command_body=list(applied),
                exact_zero=not any(request) and not any(applied),
            )
            if self.integral_unload_config is not None:
                extension = receipt.get("pre_zero_integral_unload")
                if extension is not None:
                    self.zero_ack_candidate["pre_zero_integral_unload"] = copy.deepcopy(extension)
            if (
                self.neutral_rate_brake_capture_config is not None
                and self.controlled_braking
                and self.handoff_active
                and self.zero_ack_candidate["exact_zero"]
                and stamp > round(self.handoff_event["brake_entry_control_s"] * 1e9)
                and self.neutral_first_natural_zero_ack is None
            ):
                self.neutral_first_natural_zero_ack = copy.deepcopy(self.zero_ack_candidate)
            return True
        except (ValueError, TypeError, KeyError, AttributeError) as error:
            self.zero_ack_error = str(error)
            return False

    def note_zero_publication(self, clock_ns):
        """登记最后零命令发布的控制时钟（纳秒），只记第一次。

        返回：登记成功 True；不在 authoritative_zero 阶段或没有零命令边沿时 False。
        失败：发布早于零命令控制边沿时 raise。
        """
        if (
            not self.handoff_active
            or self.settle_control_stage != "authoritative_zero"
            or self.last_zero_handoff is None
        ):
            return False
        if type(clock_ns) is not int or clock_ns < round(
            self.last_zero_handoff["actual_control_s"] * 1e9
        ):
            raise ValueError("Final zero publication precedes its control edge")
        if self.last_zero_handoff.get("zero_publication_clock_ns") is None:
            self.last_zero_handoff["zero_publication_clock_ns"] = clock_ns
            self.handoff_event["zero_publication_clock_ns"] = clock_ns
        return True

    def _post_handoff_zero_ack(self, stamp_s, wall_s):
        """检查零命令发布之后是否出现了一帧独立、精确零、身份匹配的 ACK。

        返回：满足则 True，并在首次满足时锁存 final_zero_ack；否则 False。
        语义：ACK 输入时钟必须晚于零发布、不晚于当前位姿、墙钟 300 ms 内，且四元水位
        全部严格前进；这里不改写任何命令。
        """
        edge = self.last_zero_handoff
        ack = self.zero_ack_candidate
        if not self.handoff_active or edge is None or ack is None or not ack["exact_zero"]:
            return False
        publication = edge.get("zero_publication_clock_ns")
        old = edge.get("ack_watermark")
        context = edge["context"]
        if (
            publication is None
            or old is None
            or ack["input_clock_ns"] <= publication
            or ack["input_clock_ns"] > round(stamp_s * 1e9)
            or not 0 <= wall_s - ack["received_wall_ns"] / 1e9 < 0.3
            or ack["run_id"] != edge.get("run_id")
            or ack["request_id"] != context.get("request_id")
            or ack["waypoint_index"] != context.get("waypoint_index")
            or any(
                ack[k] <= old[k]
                for k in ("sequence", "input_clock_ns", "input_frame_serial", "inference_sequence")
            )
        ):
            return False
        if self.final_zero_ack is None:
            self.final_zero_ack = copy.deepcopy(ack)
        return True

    def _begin_braking(self, sim_time):
        """从 controlled_convergence 进入主动刹车，并冻结本次 handoff 事件。

        前置：截止时间与身份绑定必须已就绪且与当前上下文一致，否则直接判失败。
        副作用：写入 brake_entry_control_s/deadline/context/purpose，清空历史与统计窗，
        把阶段置 controlled_braking，并重置静默与实验扩展状态。
        返回：成功进入刹车且未越过截止时间时 True；否则 False（相位已被置 failed）。
        """
        binding = self.handoff_deadline_binding
        if (
            self.handoff_deadline_candidate is None
            or binding is None
            or (binding.get("request_id"), binding.get("waypoint_index"))
            != (
                self.transition_context.get("request_id"),
                self.transition_context.get("waypoint_index"),
            )
        ):
            # 先造一个 pending 事件，fail_handoff 才会真正落失败原因与相位
            # （否则 handoff_active 为 False，失败会被静默丢弃）。
            self.handoff_event = dict(status="pending")
            self.fail_handoff("active_handoff_deadline_missing", sim_time)
            return False
        # 冻结本次握手事件：之后所有 ACK/发布/证据都以这里的控制边沿与身份为准。
        self.handoff_deadline_s = self.handoff_deadline_candidate
        self.handoff_event = dict(
            status="pending",
            brake_entry_control_s=sim_time,
            deadline_s=self.handoff_deadline_s,
            deadline_binding=copy.deepcopy(binding),
            context=copy.deepcopy(self.transition_context),
            purpose=self.transition_purpose,
        )
        self.brake_history.clear()
        self.brake_last_stamp = self.last_stamp
        self.history.clear()
        self.stop_window.clear()
        self.last_release = None
        self.settle_control_stage = "controlled_braking"
        if self.neutral_rate_brake_capture_config is not None:
            self.neutral_first_natural_zero_publication = None
            self.neutral_first_natural_zero_ack = None
            self.neutral_old_brake_history.clear()
            self.neutral_old_quiet_history.clear()
            self.neutral_old_capture_last_decision = None
            self.neutral_old_capture_last_sample = None
        if self.stance_support_handoff_config is not None:
            self.stance_old_brake_history.clear()
            self.stance_old_quiet_history.clear()
            self.stance_old_capture_last_decision = None
            self.stance_old_capture_last_sample = None
        if self.quiet_target_enabled:
            self._clear_quiet("brake_entry")
            self.quiet_publication = None
            self.quiet_binding = None
            self.last_quiet_qualification = None
            self.quiet_ack_watermark = (
                tuple(
                    self.zero_ack_highwater[k]
                    for k in (
                        "sequence",
                        "input_clock_ns",
                        "input_frame_serial",
                        "inference_sequence",
                    )
                )
                if self.zero_ack_highwater is not None
                else None
            )
        # 进入刹车时若已经晚于截止时间，直接判超时失败，不做"先刹车再看"。
        if sim_time > self.handoff_deadline_s:
            self.fail_handoff("active_handoff_deadline_exceeded", sim_time)
            return False
        return True

    def _braking_qualified(self, sim_time, wall_time, error, center_band):
        """判断主动刹车是否已连续成立（航向在中心带内 + 运动门 + 样本窗口足够）。

        返回：样本数 >= stop_min_pose_samples 且首尾跨度 >= stop_window_sim_s。
        语义：任一样本不合格即清空刹车历史；neutral 口径强制使用全带宽中心带。
        """
        m = self.latest_motion
        c = self.config
        if self.neutral_rate_brake_capture_config is not None:
            center_band = self._post_entry_brake_heading_band()
        valid = abs(error) <= center_band and self._motion_ready_for_zero(sim_time, wall_time)
        if not valid:
            self.brake_history.clear()
            return False
        return bool(
            len(self.brake_history) >= c["stop_min_pose_samples"]
            and self.brake_history[-1][0] - self.brake_history[0][0]
            >= c["stop_window_sim_s"] - 1e-9
        )

    def observe(
        self,
        stamp_s,
        wall_s,
        velocity,
        slam_wz,
        body_gyro,
        gyro_stamp_s,
        zero_command,
        position=None,
        quaternion=None,
        *,
        positive_qualification_allowed=True,
    ):
        """喂入一帧 SLAM 位姿/速度与配对机体陀螺，更新各相位判据的样本窗口。

        参数（调用点见 navigation/ros_node.py 的里程计回调）：
        - stamp_s：SLAM header 时间（秒，仿真/ROS 时钟）；wall_s：接收墙钟（秒，monotonic）；
        - velocity：机体线速度 [vx, vy, vz]（m/s）；slam_wz：SLAM 机体 yaw 角速度（rad/s）；
        - body_gyro：原始机体陀螺 [wx, wy, wz]（rad/s）；gyro_stamp_s：配对陀螺 stamp（秒）；
        - zero_command：调用方给出的零命令标志（命令为零且不在受控刹车）；
        - position/quaternion：可选 SLAM 位姿（米 / xyzw），四元数用于解析机体偏航；
        - positive_qualification_allowed：False 时本帧只算运动观测，不得产生新资格。
        返回：是否满足 legacy 每样本停止门。
        副作用：更新 latest_motion、history、brake_history、静默历史与统计窗。
        拒绝：输入非有限或时间不前进时返回 False，且不更新任何窗口。
        """
        values = list(velocity) + [stamp_s, wall_s, slam_wz, gyro_stamp_s] + list(body_gyro)
        if len(velocity) != 3 or len(body_gyro) != 3 or not all(math.isfinite(x) for x in values):
            return False
        # 时间必须严格前进：重复或倒退的 header 不参与任何窗口，避免同一帧被重复计数。
        if self.last_stamp is not None and stamp_s <= self.last_stamp:
            return False
        self.last_stamp = stamp_s
        c = self.config
        self.latest_motion = dict(
            stamp_s=stamp_s,
            wall_s=wall_s,
            velocity=list(velocity),
            slam_wz=slam_wz,
            body_gyro=list(body_gyro),
            gyro_stamp_s=gyro_stamp_s,
            zero_command=bool(zero_command),
            positive_qualification_allowed=positive_qualification_allowed is True,
        )
        # 来自其他命令/请求 scope 的延迟输入仍算一次运动观测，但不得作为新的刹车/
        # 静默/零速资格种子（否则旧 scope 的帧会污染当前握手）。
        zero_command = bool(zero_command and positive_qualification_allowed)
        # 每个被接受的 header 都要参与：坏样本不能因为控制回调之前又来了一个里程计
        # 回调就"消失"，否则窗口会漏掉实际发生的跳变。
        if self.controlled_braking:
            self.brake_last_stamp = stamp_s
            q = list(quaternion or [])
            qnorm = (
                sum(v * v for v in q) if len(q) == 4 and all(math.isfinite(v) for v in q) else 0.0
            )
            heading_valid = False
            observed_yaw = None
            # 四元数范数窗 0.98..1.02 与导航侧位姿校验同区间；范数非法时 heading_valid
            # 保持 False（本帧不算合格刹车样本，但仍在下面被计入 history 的候选门）。
            if 0.98 <= qnorm <= 1.02:
                x, y, z, w = [v / math.sqrt(qnorm) for v in q]
                observed_yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                heading_valid = (
                    abs(self.error(self.heading, observed_yaw))
                    <= self._post_entry_brake_heading_band()
                )
            # 合格刹车样本 = 允许资格 + 瞬时运动门 + 航向在中心带内 + 握手确实存在 +
            # 晚于刹车入口（入口那一帧本身不算刹车样本）。
            brake_valid = (
                positive_qualification_allowed
                and self._motion_ready_for_zero(stamp_s, wall_s)
                and heading_valid
                and self.handoff_event is not None
                and stamp_s > self.handoff_event["brake_entry_control_s"]
            )
            if (
                not brake_valid
                or self.brake_history
                and stamp_s - self.brake_history[-1][0] > c["stop_pose_gap_max_s"]
            ):
                self.brake_history.clear()
            if brake_valid:
                self.brake_history.append((stamp_s, wall_s))
            if self.quiet_target_enabled:
                if self.neutral_rate_brake_capture_config is not None:
                    error = None if observed_yaw is None else self.error(self.heading, observed_yaw)
                    self.neutral_old_capture_last_sample = dict(
                        pose_stamp_s=stamp_s,
                        wall_s=wall_s,
                        locked_heading_error_rad=error,
                        new_brake_motion_valid=bool(brake_valid),
                        source_and_ACK_qualified=False,
                        old_brake_instantaneous=False,
                        old_quiet_instantaneous=False,
                    )
                if self.stance_support_handoff_config is not None:
                    error = None if observed_yaw is None else self.error(self.heading, observed_yaw)
                    self.stance_old_capture_last_sample = dict(
                        pose_stamp_s=stamp_s,
                        wall_s=wall_s,
                        locked_heading_error_rad=error,
                        new_brake_motion_valid=bool(brake_valid),
                        source_and_ACK_qualified=False,
                        old_brake_instantaneous=False,
                        old_quiet_instantaneous=False,
                        old_command_caps_passed=False,
                    )
                self._observe_quiet(stamp_s, wall_s, brake_valid, observed_yaw)
        # 刹车输出（包括自然出现的精确零）永远不算停车证据：这是本门最关键的
        # 反自证规则——不能因为"我们刹出了零"就认定"机器人静止了"。
        if self.active_braking_enabled and self.handoff_failure:
            zero_command = False
        if self.active_braking_enabled and self.phase == "settle":
            # 受控收敛/受控刹车阶段一律不认零命令；只有 authoritative_zero 阶段且
            # 拿到零发布之后的独立 ACK，才把零命令继续传下去。
            if self.settle_control_stage in ("controlled_convergence", "controlled_braking"):
                zero_command = False
            elif (
                self.settle_control_stage == "authoritative_zero" and self.handoff_event is not None
            ):
                zero_command = bool(zero_command and self._post_handoff_zero_ack(stamp_s, wall_s))
        self.latest_motion["zero_command"] = bool(zero_command)
        self.stop_window.observe(
            stamp_s,
            wall_s,
            velocity,
            slam_wz,
            body_gyro,
            gyro_stamp_s,
            zero_command,
            position,
            quaternion,
        )
        # legacy 每样本门（单位：m/s、rad/s、s）：零命令 + 平面速度 <= 0.03 + |SLAM wz|
        # <= 0.05 + 陀螺范数 <= 0.05 + pose/gyro 配对 <= 0.15。这些值来自
        # teacher_transition，是"连续 0.3 s 实测静止"判据的样本级前提。
        valid = (
            zero_command
            and math.hypot(*velocity[:2]) <= c["stop_planar_speed_mps"]
            and abs(slam_wz) <= c["stop_slam_yaw_rate_radps"]
            and math.sqrt(sum(x * x for x in body_gyro)) <= c["stop_body_gyro_radps"]
            and abs(stamp_s - gyro_stamp_s) <= c["gyro_pose_pair_max_s"]
        )
        # 样本不合格，或与上一样本间隔超过 stop_pose_gap_max_s(0.2 s)，都清空历史：
        # 窗口必须连续，不能跨过空档把两段静止拼成一段。
        if not valid or self.history and stamp_s - self.history[-1][0] > c["stop_pose_gap_max_s"]:
            self.history.clear()
        if valid:
            self.history.append((stamp_s, wall_s))
        return valid

    @property
    def controlled_settle(self):
        """当前是否处于 settle 的受控收敛阶段（controlled_convergence）。"""
        return self.phase == "settle" and self.settle_control_stage == "controlled_convergence"

    def _ordinary_handoff_enabled(self):
        """判断本帧转移是否属于"普通"交回范围（统计/连续交接分支的准入条件）。

        要求：purpose 在 SPEC["eligible_purposes"] 内、context 带字符串 request_id 与
        整数 waypoint_index、且 dynamic_recovery 为 False。
        """
        # 既有的动态/倾斜/unknown 安全保持仍走原来的精确零路径：只有普通转移才允许
        # 统计窗或连续交接这些"更宽松"的分支。
        return (
            self.transition_purpose in SPEC["eligible_purposes"]
            and isinstance(self.transition_context.get("request_id"), str)
            and type(self.transition_context.get("waypoint_index")) is int
            and self.transition_context.get("dynamic_recovery") is False
        )

    def _motion_ready_for_zero(self, sim_time, wall_time):
        """检查最新一帧运动观测是否满足"瞬时静止"门（不含零命令驻留）。

        判据：墙钟 300 ms 内、仿真时钟在 [-0.05, 0.3) 内、pose/gyro 配对窗内，且
        平面速度/|SLAM wz|/陀螺范数都在 teacher_transition 的瞬时上限内。
        返回：布尔；不修改任何状态。
        """
        # 与原始"实测停止"判据使用同一组瞬时上限与新鲜度，但不在这里声称交接之前
        # 已经存在零命令驻留（那是握手与 ACK 负责证明的事）。
        m = self.latest_motion
        c = self.config
        return bool(
            m is not None
            and 0 <= wall_time - m["wall_s"] < 0.3
            and -0.05 <= sim_time - m["stamp_s"] < 0.3
            and abs(m["stamp_s"] - m["gyro_stamp_s"]) <= c["gyro_pose_pair_max_s"]
            and math.hypot(*m["velocity"][:2]) <= c["stop_planar_speed_mps"]
            and abs(m["slam_wz"]) <= c["stop_slam_yaw_rate_radps"]
            and math.sqrt(sum(x * x for x in m["body_gyro"])) <= c["stop_body_gyro_radps"]
        )

    def _observe_stop_wait(self, sim_time, wall_time, phase):
        """诊断观察分支：原始停止判据未通过、但实测确实静止时记录一条观察证据。

        参数：phase 为当前相位名（写进观测 record 的 "teacher_heading_<phase>"）。
        返回：记录成功 True；诊断开关关闭或条件不满足时 False。
        语义：只在 diagnostic_run 存在、诊断 ACK 就绪、瞬时静止门通过且本帧为零命令
        时记录；它记录的是"业务停止未通过"的事实，不能当原始 PASS。
        """
        config = self.diagnostic_profile.get("diagnostic_observe_only", {})
        if not (
            isinstance(config, dict)
            and type(config.get("enabled")) is bool
            and config["enabled"]
            and self.diagnostic_run is not None
            and self.diagnostic_zero_ack_ready is True
            and self._motion_ready_for_zero(sim_time, wall_time)
            and self.latest_motion.get("zero_command") is True
        ):
            return False
        from embodied_navigation.common.observation import api as _observer_api

        record = _observer_api("diagnostic_observation").record
        record(
            self.diagnostic_run,
            self.diagnostic_profile,
            "teacher_heading_" + phase,
            "original_measured_stop_business_wait",
            dict(
                actual_control_s=sim_time,
                original_legacy_stop_passed=False,
                statistics=self.stop_window.evidence(sim_time, wall_time),
                actual_fresh_zero_actor_ack=True,
                fresh_instantaneous_stop_limits_passed=True,
                zero_command=True,
                control_heading_math_unchanged=True,
            ),
        )
        return True

    def stopped(self, sim_time, wall_time):
        """legacy 连续 300 ms 实测停止判据。

        要求：样本数 >= stop_min_pose_samples(3)、最近样本墙钟 300 ms 内、仿真时钟在
        [-0.05, 0.3) 内、且首尾跨度 >= stop_window_sim_s(0.3 s) 减去 1e-9 浮点松弛。
        返回：布尔；不修改状态。这是释放判据里与统计窗 OR 的那一支。
        """
        c = self.config
        return (
            len(self.history) >= c["stop_min_pose_samples"]
            and 0 <= wall_time - self.history[-1][1] < 0.3
            and -0.05 <= sim_time - self.history[-1][0] < 0.3
            and self.history[-1][0] - self.history[0][0] >= c["stop_window_sim_s"] - 1e-9
        )

    def resume_after_stop(self, yaw, fresh_path_heading, stopped_duration):
        """动态恢复后重新进入 drive：重置相位并把锁定航向换成新路径朝向。

        参数：yaw 为当前实测偏航（rad）；fresh_path_heading 为新路径朝向（rad）；
        stopped_duration 为已停止时长（秒）。
        返回：恒定 False（本方法不释放，只重置状态）。
        边界：调用方仍保留原云清晰度/新 SCAN 守卫；单靠一个时长永远不能为 Teacher
        建立"物理已停"。
        """
        self.reset()
        self.transition_purpose = "dynamic_recovery"
        self.heading = fresh_path_heading
        return False

    def update(self, sim_time, yaw, current_path_heading, wall_time=None):
        """推进相位状态机，返回 (release, yaw_command)。

        参数：sim_time 为控制时钟（秒，仿真时钟）；yaw 为实测机体偏航（rad，来自 SLAM
        位姿）；current_path_heading 为当前路径朝向（rad，来自规划）；wall_time 为单调
        墙钟（秒），缺省取 time.monotonic()。
        返回：(False, 0.0) 表示继续停车/转向、不释放；(True, yaw_command) 表示回到
        drive，yaw_command 为限幅在 ±0.08 rad/s 的航向修正。
        失败与保持：握手失败时恒返回 (False, 0.0) 且不自动重试；只有新的
        request/waypoint 身份才会解除 failed。
        副作用：修改 phase/heading/settle_control_stage、各类历史与握手事件。
        """
        import time

        # 缺省用进程单调钟；调用方（navigation/ros_node.py）传的是同一单调时钟的接收时刻。
        wall_time = time.monotonic() if wall_time is None else wall_time
        c = self.config
        if self.minimal_simulation_baseline and not all(
            math.isfinite(v) for v in (sim_time, yaw, current_path_heading, wall_time)
        ):
            raise ValueError("Finite actual diagnostic heading and clocks required")
        current_error = self.error(current_path_heading, yaw)
        # 失败即保持：不复位、不重试，也不返回任何命令，直到身份变化（见
        # set_transition_context）。这是"证据不足就不动"的落点。
        if self.handoff_failure:
            return False, 0.0
        # 截止时间在任何相位都检查：超时即判失败，避免无限期停在 settle。
        if self.handoff_active and sim_time > self.handoff_deadline_s:
            self.fail_handoff("active_handoff_deadline_exceeded", sim_time)
            return False, 0.0
        if self.phase == "drive":
            # drive：小误差直行，命令 = 0.5*误差并限幅 ±0.08 rad/s（硬编码增益与上限）。
            if abs(current_error) <= c["drive_heading_max_rad"]:
                return True, max(-0.08, min(0.08, 0.5 * current_error))
            # 误差超过 drive_heading_max_rad(0.2 rad) 时先停车再转向：reset 会清窗口并
            # 回到 pre_turn，随后必须重新证明实测静止才允许 align。
            self.reset()  # Stop immediately before a substantial new heading.
            if self.transition_purpose not in ("dynamic_recovery", "tilt_recovery"):
                self.transition_purpose = "heading_change"
        if self.phase == "pre_turn":
            # 转向前的实测静止门：不静止就保持 (False, 0.0)，绝不"边走边转"。
            # 诊断观察分支只记录"原始判据未通过"的静止事实，默认关闭。
            if (
                not self.minimal_simulation_baseline
                and not self.stopped(sim_time, wall_time)
                and not self._observe_stop_wait(sim_time, wall_time, "pre_turn")
            ):
                return False, 0.0
            # 放行后把锁定航向设为当前路径朝向；之后对齐都用这个锁存值，不跟随路径漂移。
            self.phase = "align"
            self.heading = current_path_heading
        if self.heading is None:
            self.heading = current_path_heading
        error = self.error(self.heading, yaw)
        if self.phase == "align":
            # 对齐：命令 = 1.3*误差（1.3 是硬编码比例增益，代码与文档未说明来源），
            # 再按 max_yaw_rate_radps 限幅；误差进入 turn_complete_rad(0.1 rad) 才算转完。
            if abs(error) > c["turn_complete_rad"]:
                return False, max(-self.max_turn_rate, min(self.max_turn_rate, 1.3 * error))
            # 最小诊断基线只按几何航向放行，并显式记录"未证明实测停止"；该分支不可达。
            if self.minimal_simulation_baseline:
                self.phase = "drive"
                self.heading = None
                self.settle_control_stage = None
                self.last_release = dict(
                    predicate="r73_geometric_heading_only_diagnostic",
                    actual_control_s=sim_time,
                    original_measured_stop_qualified=False,
                    original_quiet_handoff_qualified=False,
                    formal_acceptance_pass=False,
                )
                return True, max(-0.08, min(0.08, 0.5 * current_error))
            self.phase = "settle"
            self.stop_window.clear()
            self.last_release = None
            # 普通转移先进入受控收敛（继续用既有 PID/PI 向同一锁定中心收敛），
            # 其它情形（动态恢复/倾斜恢复等）直接进入零速权威阶段。
            self.settle_control_stage = (
                "controlled_convergence"
                if self._ordinary_handoff_enabled()
                else "authoritative_zero"
            )
        if self.phase == "settle":
            # 路径朝向相对锁定航向偏得太多：握手中直接判失败（不能一边握手一边改目标）；
            # 未握手则回到 pre_turn，重新证明静止后才允许再次转向。
            if abs(current_error) > c["drive_heading_max_rad"]:
                if self.handoff_active:
                    self.fail_handoff("active_handoff_heading_rearm", sim_time)
                    return False, 0.0
                self.phase = "pre_turn"
                self.heading = None
                self.settle_control_stage = None
                self.stop_window.clear()
                self.last_release = None
                return False, 0.0
            if (
                self.settle_control_stage == "authoritative_zero"
                and not self.active_braking_enabled
                and self._ordinary_handoff_enabled()
                and abs(error) > c["turn_complete_rad"]
            ):
                # 已经在零速权威阶段等待，但转完的航向又丢失了。放行前先回到同一锁定
                # 目标重新收敛；yaw 控制一旦恢复，旧的零速证据就不能再为下一次尝试
                # 背书，因此下面把相关证据全部清空。
                self.zero_reconvergence_count += 1
                self.last_zero_reconvergence = dict(
                    reason="turn_complete_band_lost_during_authoritative_zero",
                    actual_control_s=sim_time,
                    source_pose_stamp_s=self.last_stamp,
                    locked_heading_rad=self.heading,
                    locked_heading_error_rad=error,
                    original_turn_complete_rad=c["turn_complete_rad"],
                    discarded_stop_samples=len(self.history),
                )
                self.history.clear()
                self.stop_window.clear()
                self.last_release = None
                self.last_zero_handoff = None
                self.final_zero_ack = None
                self.zero_ack_candidate = None
                self.last_brake_qualification = None
                self.last_quiet_qualification = None
                self._clear_quiet("heading_reconvergence")
                self.quiet_publication = None
                self.quiet_binding = None
                self.quiet_pose_context = None
                self.quiet_ack_watermark = None
                self.settle_control_stage = "controlled_convergence"
            if self.controlled_settle:
                # 这里只改变控制交接口径：既有 cascade yaw PD/PI 与 Student 修正继续
                # 作用在同一个锁定中心上，不是新的验收阈值，也不是定时等待。
                # 中心带取 0.25*turn_complete_rad = 0.025 rad，留在原 0.1 rad 带内，
                # 为实测约 0.11 rad 的 Actor 零输入回弹留出余量。
                center_band = 0.25 * c["turn_complete_rad"]
                if abs(error) > center_band or not self._motion_ready_for_zero(sim_time, wall_time):
                    return False, max(-self.max_turn_rate, min(self.max_turn_rate, 1.3 * error))
                # 连续交接分支：只在 path_rebind/heading_change 且普通转移时允许"无缝"
                # 交回 drive；本帧必须带正向资格，否则继续用转向命令收敛。
                if (
                    self.continuous_turn_handoff_enabled
                    and self._ordinary_handoff_enabled()
                    and self.transition_purpose in ("path_rebind", "heading_change")
                ):
                    if self.latest_motion.get("positive_qualification_allowed") is not True:
                        return False, max(-self.max_turn_rate, min(self.max_turn_rate, 1.3 * error))
                    # 普通运动直接交给既有受检 PID drive 路径继续，两个活跃控制模式之间
                    # 不需要开环零命令边沿，也不声称出现过静止。
                    self.continuous_turn_handoff_count += 1
                    self.last_release = dict(
                        predicate="ordinary_turn_continuous_control/v1",
                        actual_control_s=sim_time,
                        locked_heading_rad=self.heading,
                        locked_heading_error_rad=error,
                        fresh_motion=dict(self.latest_motion),
                        purpose=self.transition_purpose,
                        context=dict(self.transition_context),
                        legacy_measured_stop=False,
                        statistical_stop=False,
                        zero_command_dwell=False,
                        parking_acceptance=False,
                    )
                    self.history.clear()
                    self.stop_window.clear()
                    self.phase = "drive"
                    self.heading = None
                    self.settle_control_stage = None
                    return True, max(-0.08, min(0.08, 0.5 * current_error))
                # 启用主动刹车时先进入 controlled_braking（本帧仍不释放）；
                # 否则直接进入零速权威阶段，开始零命令驻留。
                if self.active_braking_enabled:
                    self._begin_braking(sim_time)
                    return False, 0.0
                self.settle_control_stage = "authoritative_zero"
                self.zero_handoff_count += 1
                self.history.clear()
                self.stop_window.clear()
                self.last_release = None
                self.last_zero_handoff = dict(
                    actual_control_s=sim_time,
                    locked_heading_rad=self.heading,
                    locked_heading_error_rad=error,
                    center_target_band_rad=center_band,
                    fresh_motion=dict(self.latest_motion),
                    zero_command_dwell_begins_after_this_edge=True,
                )
                return False, 0.0
            if self.controlled_braking:
                # 主动刹车阶段：先要刹车样本窗口连续，再要静默目标连续，最后要真实
                # 冻结的 Actor ACK；三者缺一都继续输出零命令并等待（不释放）。
                center_band = self._post_entry_brake_heading_band()
                brake_ready = self._braking_qualified(sim_time, wall_time, error, center_band)
                if not brake_ready:
                    self._record_neutral_old_capture_decision(
                        sim_time, wall_time, error, False, False
                    )
                    self._record_stance_old_capture_decision(
                        sim_time, wall_time, error, False, False
                    )
                    return False, 0.0
                quiet_ready = self._quiet_qualified(sim_time, wall_time, error)
                self._record_neutral_old_capture_decision(
                    sim_time, wall_time, error, brake_ready, quiet_ready
                )
                self._record_stance_old_capture_decision(
                    sim_time, wall_time, error, brake_ready, quiet_ready
                )
                if not quiet_ready:
                    return False, 0.0
                # 没有真实冻结的 Actor ACK 水位就不给最终零命令边沿：零速交接不能靠
                # "我们发了零"自证。
                if self.zero_ack_candidate is None:
                    return False, 0.0  # No final edge without a real frozen Actor watermark.
                self.last_brake_qualification = dict(
                    actual_control_s=sim_time,
                    first_stamp_s=self.brake_history[0][0],
                    last_stamp_s=self.brake_history[-1][0],
                    samples=len(self.brake_history),
                    original_stop_pass=False,
                    center_target_band_rad=center_band,
                    locked_heading_error_rad=error,
                )
                self.settle_control_stage = "authoritative_zero"
                self.zero_handoff_count += 1
                if self.integral_unload_config is not None:
                    self.integral_unload_previous_zero = dict(
                        event_id=self.integral_unload_event["event_id"],
                        zero_control_clock_ns=round(sim_time * 1e9),
                        zero_handoff_count=self.zero_handoff_count,
                    )
                self.history.clear()
                self.stop_window.clear()
                self.last_release = None
                self.final_zero_ack = None
                self.last_zero_handoff = dict(
                    actual_control_s=sim_time,
                    locked_heading_rad=self.heading,
                    locked_heading_error_rad=error,
                    center_target_band_rad=center_band,
                    fresh_motion=dict(self.latest_motion),
                    zero_command_dwell_begins_after_this_edge=True,
                    zero_publication_clock_ns=None,
                    ack_watermark=copy.deepcopy(self.zero_ack_highwater),
                    run_id=self.zero_ack_candidate["run_id"],
                    context=copy.deepcopy(self.transition_context),
                    deadline_s=self.handoff_deadline_s,
                )
                # 冻结最终零命令时间；此后只有 post-handoff 独立 ACK 才能把它变成停车证据。
                self.handoff_event.update(final_zero_control_s=sim_time)
                return False, 0.0
            # 非主动刹车路径（含默认配置）：legacy 300 ms 判据 OR 1 s 统计窗 OR
            # 诊断观察分支，任一成立即释放。三支的 predicate 会写进 last_release。
            legacy = self.stopped(sim_time, wall_time)
            stats = self.stop_window.evidence(sim_time, wall_time)
            # 这里把 _ordinary_handoff_enabled 的准入条件又写了一遍：释放处显式可见，
            # 避免读者要跳一层才知道统计分支对谁生效。
            eligible = (
                self.transition_purpose in SPEC["eligible_purposes"]
                and isinstance(self.transition_context.get("request_id"), str)
                and type(self.transition_context.get("waypoint_index")) is int
                and self.transition_context.get("dynamic_recovery") is False
            )
            # 统计分支只在 eligible purpose/身份下生效；观察分支要求当前没有进行中握手。
            statistical = bool(eligible and stats["enabled"] and stats["passed"])
            observed = (
                not self.handoff_active
                and not legacy
                and not statistical
                and self._observe_stop_wait(sim_time, wall_time, "settle")
            )
            if not legacy and not statistical and not observed:
                return False, 0.0
            # predicate 三选一：诊断观察 / legacy 连续 300 ms / 1 s 统计窗；
            # 三者都不是"任务成功"，只是允许本门回到 drive。
            self.last_release = dict(
                predicate=(
                    "diagnostic_observe_only_original_stop_not_passed"
                    if observed
                    else "legacy_continuous_300ms"
                    if legacy
                    else SPEC["schema"]
                ),
                legacy_measured_stop=legacy,
                statistical_stop=statistical,
                purpose=self.transition_purpose,
                context=dict(self.transition_context),
                actual_control_s=sim_time,
                statistics=stats,
            )
            if self.handoff_active:
                self.handoff_event.update(status="passed", release_control_s=sim_time)
            # 释放：回到 drive 并清掉锁定航向；本帧给一次限幅 ±0.08 rad/s 的航向修正。
            self.phase = "drive"
            self.heading = None
            return True, max(-0.08, min(0.08, 0.5 * current_error))
        # 未知相位（不应出现）：不释放、不输出命令。
        return False, 0.0

    def evidence(self, sim_time, wall_time):
        """汇总本门的全部证据字典（只读，不修改状态）。

        参数：sim_time 为控制时钟（秒）；wall_time 为墙钟（秒）。
        返回：证据字典；键名是英文并属于对外证据接口，不能翻译或改名。
        内容：相位与实测停止、样本计数、handoff 事件与失败原因、静默目标、积分卸载
        以及各实验扩展的诊断影子。
        """
        result = {
            "transition": "teacher_measured_stop_heading_v1",
            "phase": self.phase,
            "measured_stop": self.stopped(sim_time, wall_time),
            "stop_valid_pose_samples": len(self.history),
            "stop_first_stamp_s": self.history[0][0] if self.history else None,
            "stop_last_stamp_s": self.history[-1][0] if self.history else None,
            # 两行是证据语义声明：heading 只来自 SLAM 机体 yaw；陀螺只用于停/倾判定。
            "navigation_heading_source": "SLAM body yaw only",
            "gyro_use": "stop/tilt only; never localization",
            "transition_purpose": self.transition_purpose,
            "transition_context": dict(self.transition_context),
            "navigation_settle_statistics": self.stop_window.evidence(sim_time, wall_time),
            "last_settle_release": self.last_release,
            "settle_control_stage": self.settle_control_stage,
            "controlled_settle": self.controlled_settle,
            "controlled_braking": self.controlled_braking,
            "active_braking_enabled": self.active_braking_enabled,
            "handoff_active": self.handoff_active,
            "handoff_failure": self.handoff_failure,
            "handoff_event": copy.deepcopy(self.handoff_event),
            "brake_valid_pose_samples": len(self.brake_history),
            "last_brake_qualification": copy.deepcopy(self.last_brake_qualification),
            "final_zero_ack": copy.deepcopy(self.final_zero_ack),
            "zero_ack_error": self.zero_ack_error,
            "zero_handoff_count": self.zero_handoff_count,
            "last_zero_handoff": self.last_zero_handoff,
            "zero_reconvergence_count": self.zero_reconvergence_count,
            "last_zero_reconvergence": self.last_zero_reconvergence,
            "continuous_turn_handoff_enabled": self.continuous_turn_handoff_enabled,
            "continuous_turn_handoff_count": self.continuous_turn_handoff_count,
            "settle_center_target_band_rad": 0.25 * self.config["turn_complete_rad"],
            "acceptance_thresholds_changed": False,
        }
        if self.neutral_rate_brake_capture_config is not None:
            result["neutral_rate_brake_capture"] = dict(
                configuration=copy.deepcopy(self.neutral_rate_brake_capture_config),
                new_internal_capture_changed=True,
                original_business_acceptance_unchanged=True,
                acceptance_scope="Original teacher_transition fields, source/motion/ACK and terminal fixed5 owner; R16 internal capture is changed.",
                brake_entry_heading_band_rad=0.25 * self.config["turn_complete_rad"],
                post_entry_capture_heading_band_rad=self._post_entry_brake_heading_band(),
                locked_heading_rad=self.heading,
                locked_heading_target_moved=False,
                original_R16_internal_capture_claimed=False,
                old_internal_capture_shadow=dict(
                    diagnostic_only=True,
                    brake_heading_band_rad=0.25 * self.config["turn_complete_rad"],
                    quiet_heading_band_rad=0.005,
                    brake_samples=len(self.neutral_old_brake_history),
                    brake_first_stamp_s=self.neutral_old_brake_history[0][0]
                    if self.neutral_old_brake_history
                    else None,
                    brake_last_stamp_s=self.neutral_old_brake_history[-1][0]
                    if self.neutral_old_brake_history
                    else None,
                    quiet_samples=len(self.neutral_old_quiet_history),
                    quiet_first_stamp_s=self.neutral_old_quiet_history[0]["stamp_s"]
                    if self.neutral_old_quiet_history
                    else None,
                    quiet_last_stamp_s=self.neutral_old_quiet_history[-1]["stamp_s"]
                    if self.neutral_old_quiet_history
                    else None,
                    last_control_evaluation=copy.deepcopy(self.neutral_old_capture_last_decision),
                    last_pose_observation=copy.deepcopy(self.neutral_old_capture_last_sample),
                    old_numeric_conditions_on_new_control_trajectory=True,
                    old_R16_control_counterfactual_reproduced=False,
                    same_validated_source_and_ACK_samples=True,
                    independent_ACK_consumption=False,
                    new_decision_affected=False,
                ),
                heading_error_band_is_not_relative_T0_drift=True,
                first_qualified_brake_zero_publication=copy.deepcopy(
                    self.neutral_first_natural_zero_publication
                ),
                first_observed_validated_brake_zero_ack=copy.deepcopy(
                    self.neutral_first_natural_zero_ack
                ),
                natural_zero_witness_scope="First validator-qualified publication or selected ACK observed here; not earliest actual wire zero in the full stream.",
                natural_zero_is_not_formal_T0=True,
                natural_zero_adds_stop_evidence=False,
                terminal_fixed5_owner_unchanged=True,
            )
        if self.stance_support_handoff_config is not None:
            b = self.stance_old_brake_history
            q = self.stance_old_quiet_history
            result["stance_support_handoff"] = dict(
                configuration=copy.deepcopy(self.stance_support_handoff_config),
                new_internal_capture_changed=True,
                original_business_acceptance_unchanged=True,
                reference_selection="restored locked_heading_pd_v1; neutral rate explicitly off",
                reference_restored_from_R34_neutral=True,
                core_controller_Student_sources_unchanged=True,
                existing_PD_PI_formulas_unchanged=True,
                locked_heading_rad=self.heading,
                goal_or_locked_target_moved=False,
                formal_T0_moved=False,
                original_R16_internal_capture_claimed=False,
                internal_limits=dict(
                    brake_entry_heading_rad=0.25 * self.config["turn_complete_rad"],
                    post_entry_brake_heading_rad=self._post_entry_brake_heading_band(),
                    quiet_heading_rad=self.quiet_limits["heading"],
                    command_yaw_cap_radps=self.quiet_command_yaw_cap,
                    motion_SLAM_and_body_yaw_cap_radps=self.quiet_limits["yaw"],
                    command_XY_norm_cap_mps=self.quiet_limits["xy"],
                ),
                old_internal_limits=dict(
                    brake_heading_rad=0.25 * self.config["turn_complete_rad"],
                    quiet_heading_rad=0.005,
                    command_yaw_cap_radps=0.01,
                    motion_SLAM_and_body_yaw_cap_radps=0.01,
                ),
                old_strict_shadow=dict(
                    diagnostic_only=True,
                    brake_samples=len(b),
                    quiet_samples=len(q),
                    current_publication_old_command_quiet=bool(
                        self.quiet_publication is not None
                        and self._low_total_command(self.quiet_publication["command_body"], 0.01)
                    ),
                    brake_first_stamp_s=b[0][0] if b else None,
                    brake_last_stamp_s=b[-1][0] if b else None,
                    quiet_first_stamp_s=q[0]["stamp_s"] if q else None,
                    quiet_last_stamp_s=q[-1]["stamp_s"] if q else None,
                    last_control_evaluation=copy.deepcopy(self.stance_old_capture_last_decision),
                    last_pose_observation=copy.deepcopy(self.stance_old_capture_last_sample),
                    history_scope="Subset of the same primary-qualified source/ACK samples, not an independent full old brake history.",
                    old_numeric_conditions_on_candidate_trajectory=True,
                    old_control_counterfactual_reproduced=False,
                    same_validated_source_and_ACK_samples=True,
                    independent_ACK_consumption=False,
                    new_decision_affected=False,
                ),
                physical_stance_support_proven=False,
                post_turn_fixed5_success_claimed=False,
                absolute_heading_and_relative_T0_drift_are_distinct=True,
                natural_wire_zero_diagnosed_from_original_stream=True,
                terminal_fixed5_owner_unchanged=True,
            )
        if self.quiet_target_enabled:
            h = self.quiet_history
            result["quiet_handoff_target"] = dict(
                enabled=True,
                target="low_total_command_only",
                limits=copy.deepcopy(self.quiet_limits),
                samples=len(h),
                first_stamp_s=h[0]["stamp_s"] if h else None,
                last_stamp_s=h[-1]["stamp_s"] if h else None,
                reason=self.quiet_reason,
                last_qualification=copy.deepcopy(self.last_quiet_qualification),
                original_stop_PASS=False,
                physical_drift_guaranteed=False,
            )
        if self.integral_unload_config is not None:
            result["pre_zero_integral_unload"] = dict(
                enabled=True,
                configuration=copy.deepcopy(self.integral_unload_config),
                event=copy.deepcopy(self.integral_unload_event),
                previous_owned_zero_identity=copy.deepcopy(self.integral_unload_previous_zero),
                failed_event_id=self.integral_unload_failed_event_id,
                new_deployment_math=True,
                stop_PASS=False,
                event_hash_is_publication_proof=False,
            )
        return result
