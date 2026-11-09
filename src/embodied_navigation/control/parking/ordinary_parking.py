"""经过原始 SLAM/IMU/ACK 门之后的普通终端所有权；从不设置 T0。

owner 只使用新鲜的实测导航位姿/运动，不改变路线成功判定，也不启动正式停车
窗口；失败的零保持不会重试。原生物理运动仍由 acceptance 独立检查。
"""

import math
import json
from embodied_navigation.control.parking.source_hold import (
    PREQUALIFICATION_SOURCE_HOLD_ENABLED,
    UNSPECIFIED_ZERO_EVENT,
    eligible as prequalification_source_hold_eligible,
)


# 只有这 5 个“业务”原因会进入诊断观察；来源/ACK/中断类故障从不作为评分对象。
DIAGNOSTIC_BUSINESS_FAILURES = frozenset(
    (
        "parking_fixed_target_lost",
        "capture_fixed_goal_out_of_range",
        "ordinary_actual_zero_ACK_timeout",
        "ordinary_precise_capture_deceleration_timeout",
        "ordinary_zero_hold_actual_motion_or_drift_exceeded",
    )
)


def diagnostic_business_failure(reason):
    """仅接受精确的业务失败原因；来源/ACK/中断故障永不作为评分。"""
    return type(reason) is str and reason in DIAGNOSTIC_BUSINESS_FAILURES


def _diagnostic_enabled(diagnostic):
    """诊断观察是否启用：需要 diagnostic.run、dict profile 与 enabled=True。"""
    if not isinstance(diagnostic, dict) or diagnostic.get("run") is None:
        return False
    profile = diagnostic.get("profile")
    if not isinstance(profile, dict):
        return False
    value = profile.get("diagnostic_observe_only")
    return (
        isinstance(value, dict) and type(value.get("enabled")) is bool and value["enabled"] is True
    )


import numpy as np


class OrdinaryParking:
    """有状态的普通终端零保持 owner：只观察实测运动，无 T0 权限。

    阶段在 capture / controlled_deceleration / handoff_zero / zero_hold / failed
    之间迁移，内部跟踪 dwell、deadline 与可选的 pose10Hz 捕获证据。成功要求实测
    位姿与 ACK 事实；超时或保护事件也会锁存失败。数值门槛来自 profile 的
    ordinary_terminal_parking_contract / cascade.parking_pose10hz，并由
    safety_configuration 精确校验。
    """

    def __init__(self):
        self.prequalification_source_hold_enabled = PREQUALIFICATION_SOURCE_HOLD_ENABLED
        self.prequalification_source_hold_count = 0
        self.prequalification_source_hold_first = None
        self.prequalification_source_hold_last = None
        self.enabled = False
        self.stage = None
        self.dwell = None
        self.failure = None
        self.start_clock = None
        self.deadline = None
        self.ready_stamp = None
        self.ready_clock = None
        self.handoff_clock = None
        self.origin = None
        self.origin_yaw = None
        self.first_zero_publication_clock = None
        self.first_actor_zero_ack = None
        self.preT0_attempts_disqualified = 0
        self.preT0_first_disqualification = None
        self.preT0_last_disqualification = None
        self.deceleration_trigger = None
        self.diagnostic_business_failure_reason = None
        self.diagnostic_business_run = None
        self.diagnostic_business_observation_count = 0
        self.diagnostic_last_business_rejection = None
        self.pose10hz = False
        self.capture_grid = None
        self.pose_grid = None
        self.capture_goal = None
        self.capture_heading = None
        self.pose10hz_last_source = None

    def configure_pose10hz(self, value):
        """按 cascade.parking_pose10hz 配置启用/关闭 10 Hz 捕获。

        value 交给 parking_pose10hz.configuration() 精确校验；启用时新建
        CaptureGrid，后续 capture 阶段由它给出几何捕获证据。
        """
        from embodied_navigation.control.parking.parking_pose10hz import configuration, CaptureGrid

        self.pose10hz = configuration(value)
        if self.pose10hz:
            self.capture_grid = CaptureGrid()

    def start_pose10hz_window(self, proof):
        """用首个资格证据建立唯一的 10 Hz 固定窗口。

        仅在 pose10hz 启用时生效；origin 取 proof["qualification_origin"]，同时
        冻结 origin/origin_yaw 供后续漂移判定。窗口时长/栅格/漂移由
        parking_pose10hz.CONFIG 定义。
        """
        if not self.pose10hz:
            return
        from embodied_navigation.control.parking.parking_pose10hz import PoseGrid

        # 首个 10 Hz 锚点不可重置：重复进入直接失败。
        if self.pose_grid is not None:
            raise ValueError("Never reset the first 10Hz parking anchor")
        origin = proof["qualification_origin"]
        self.pose_grid = PoseGrid(proof["qualification_clock_ns"], origin)
        self.origin = np.asarray(origin["position"], float).copy()
        self.origin_yaw = self.pose_grid.base_yaw

    def observe_pose10hz_source(self, row):
        """每个已接受提交向唯一网格喂一帧；不产生任何正交接。

        未启用 10 Hz 或 owner 未启用时直接返回。时钟倒退 raise；同一 stamp 但
        位姿不同 raise，完全相同则忽略（重复提交）。窗口建立前喂 CaptureGrid，
        建立后喂 PoseGrid。
        """
        if not self.pose10hz or not self.enabled:
            return
        from embodied_navigation.control.parking.parking_pose10hz import pose

        stamp, p, yaw = pose(row)
        # 同一 10 Hz 序列必须单调；重复头允许但内容必须一致。
        if self.pose10hz_last_source is not None:
            old_stamp, old_p, old_yaw = pose(self.pose10hz_last_source)
            if stamp < old_stamp:
                raise ValueError("10Hz committed SLAM clock went backwards")
            if stamp == old_stamp:
                if (p, yaw) != (old_p, old_yaw):
                    raise ValueError("10Hz repeated committed header changed pose")
                return
        # 有固定窗口后只喂该窗口；否则喂捕获栅格。
        if self.pose_grid is not None:
            self.pose_grid.observe(row)
        else:
            self.capture_grid.observe(row, self.capture_goal, self.capture_heading)
        self.pose10hz_last_source = row

    def invalidate_pose10hz_capture(self):
        """在固定窗口建立前丢弃捕获证据，清空上一帧来源。

        只在 pose10hz 启用且尚未建立 PoseGrid 时生效：重建 CaptureGrid、清空
        pose10hz_last_source，使捕获必须重新累积；已建立的固定窗口不受影响。
        """
        if self.pose10hz and self.pose_grid is None:
            from embodied_navigation.control.parking.parking_pose10hz import CaptureGrid

            self.capture_grid = CaptureGrid()
            self.pose10hz_last_source = None

    def update_pose10hz(self, stamp, clock, position, yaw, goal_distance, yaw_error, sensor_pose):
        """10 Hz 分支：推进捕获/零保持阶段并返回 stage 字符串。

        sensor_pose 必须是原始 SLAM 位姿记录，None 直接 raise。已有固定窗口时：
        数据缺失或固定锚点漂移超限 → failed，否则进入 zero_hold。尚无窗口时：
        超过 deadline → 按是否已有 ready_stamp 区分 ACK 超时与捕获减速超时；
        CaptureGrid 就绪 → handoff_zero 并冻结 origin；否则按迟滞带（进入 15 mm、
        保持 20 mm，航向 0.035 rad）选择 controlled_deceleration/capture。

        这里只控制减速，正资格来自上面的 10 Hz 几何捕获，不来自瞬时运动。
        """
        if sensor_pose is None:
            raise ValueError("10Hz parking needs its exact original SLAM pose record")
        self.observe_pose10hz_source(sensor_pose)
        if self.pose_grid is not None:
            if self.pose_grid.missing:
                self.failure = "parking_pose10hz_data_unavailable"
                self.stage = "failed"
            elif self.pose_grid.motion_failure:
                self.failure = "parking_pose10hz_fixed_anchor_exceeded"
                self.stage = "failed"
            else:
                self.stage = "zero_hold"
            return self.stage
        # 超时归类：已 ready 说明零已交接、但 ACK 未到；否则是捕获/减速超时。
        # daily_navigation 且已 ready 时不做此超时判定（原实现的例外分支）。
        if clock > self.deadline and not (
            getattr(self, "daily_navigation", False) and self.ready_stamp is not None
        ):
            self.failure = (
                "ordinary_actual_zero_ACK_timeout"
                if self.ready_stamp is not None
                else "ordinary_precise_capture_deceleration_timeout"
            )
            self.stage = "failed"
            return self.stage
        if self.capture_goal is None:
            raise ValueError("10Hz capture requires the fixed goal")
        ready = self.capture_grid.ready
        self.dwell = self.capture_grid.since
        if self.ready_stamp is not None:
            self.stage = "zero_hold"
            return self.stage
        if ready:
            selected = self.capture_grid.last
            self.ready_stamp = selected["source_stamp_ns"]
            self.ready_clock = clock
            self.handoff_clock = clock
            self.origin = np.asarray(selected["position"], float).copy()
            self.origin_yaw = selected["yaw_rad"]
            self.stage = "handoff_zero"
        else:
            # 这只控制减速；正资格来自上面的 10 Hz 几何捕获，绝不来自瞬时运动。
            # 迟滞：进入减速带 15 mm，已在减速中则放宽到 20 mm。
            band = 0.020 if self.stage == "controlled_deceleration" else 0.015
            self.stage = (
                "controlled_deceleration"
                if goal_distance <= band and abs(yaw_error) <= 0.035
                else "capture"
            )
        return self.stage

    def begin(self, clock, timeout_ns):
        """启动普通停车捕获窗口；返回是否由本次调用启动。

        clock/timeout_ns 必须是整数纳秒，timeout_ns>0，否则 raise。timeout_ns 由
        调用方按原 goal 的 timeout_sim_s 换算传入（本文件不解释其来源），
        deadline = clock + timeout_ns。已启动过则返回 False，不重置窗口。
        """
        # 只启动一次：已启动时不重置任何状态。
        if self.enabled:
            return False
        if type(clock) is not int or type(timeout_ns) is not int or timeout_ns <= 0:
            raise ValueError("Original integer parking clock/deadline required")
        self.enabled = True
        self.stage = "capture"
        self.start_clock = clock
        self.deadline = clock + timeout_ns
        return True

    def diagnostic_failure_is_observed(self, diagnostic):
        """当前锁存失败是否为已记录、且属于同一诊断 run 的业务失败。"""
        return (
            diagnostic_business_failure(self.failure)
            and self.failure == self.diagnostic_business_failure_reason
            and _diagnostic_enabled(diagnostic)
            and str(diagnostic["run"]) == self.diagnostic_business_run
        )

    def observe_business_rejection(
        self,
        reason,
        stamp,
        clock,
        evidence,
        diagnostic,
        *,
        terminal_zero_event=UNSPECIFIED_ZERO_EVENT,
    ):
        """在执行许可之前记录业务拒绝；绝不擦除正式失败事实。

        仅当诊断观察启用、reason 是业务原因时生效；若已有失败且不属于当前已记录的
        诊断业务失败，则直接返回 False。时钟必须是因果整数纳秒。R106 来源保持命中
        时只累加保持计数并返回 False，让调用方走原有严格保护；否则通过
        common.observation 记账、锁存 failure，并返回 True 表示已接住该拒绝。
        """
        if not _diagnostic_enabled(diagnostic):
            return False
        if not diagnostic_business_failure(reason):
            return False
        if self.failure and not self.diagnostic_failure_is_observed(diagnostic):
            return False
        if type(stamp) is not int or stamp < 0 or type(clock) is not int or clock < stamp:
            raise ValueError("Original causal integer parking observation clocks required")
        # 共享账本在 JSON 编码前做去重；即便同原因上一行 <1 s，也要校验本条观察。
        json.dumps(evidence, sort_keys=True, allow_nan=False)
        if prequalification_source_hold_eligible(self, reason, terminal_zero_event):
            # 返回 False 会调用各调用方原有的严格 _protect / guard 路径；这里不修改
            # 失败、来源、deadline、dwell 或 T0。
            self.prequalification_source_hold_count += 1
            self.prequalification_source_hold_last = dict(
                reason=reason,
                original_stage=self.stage,
                source_pose_stamp_ns=stamp,
                control_stamp_ns=clock,
                goal_distance_xy_m=evidence.get("goal_distance_xy_m"),
                goal_z_error_m=evidence.get("goal_z_error_m"),
                goal_yaw_error_rad=evidence.get("goal_yaw_error_rad"),
                original_goal_z_tolerance_m=evidence.get("original_goal_z_tolerance_m"),
                queued_pose_checked=evidence.get("queued_pose_checked", False),
                action="original_strict_protection_required",
                positive_qualification_granted=False,
            )
            if self.prequalification_source_hold_first is None:
                self.prequalification_source_hold_first = dict(
                    self.prequalification_source_hold_last
                )
            return False
        from embodied_navigation.common.observation import api as _observer_api

        record = _observer_api("diagnostic_observation").record
        recorded = record(
            diagnostic["run"],
            diagnostic["profile"],
            "ordinary_terminal_parking_business",
            reason,
            dict(
                original_stage=self.stage,
                original_failure=reason,
                source_pose_stamp_ns=stamp,
                control_stamp_ns=clock,
                original_first_failure=self.failure,
                measurements=evidence,
                formal_parking_pass=False,
                navigation_PASS=False,
            ),
        )
        if recorded is not True:
            raise RuntimeError("Business parking observation was not recorded")
        self.failure = self.failure or reason
        self.diagnostic_business_failure_reason = self.failure
        self.diagnostic_business_run = str(diagnostic["run"])
        self.diagnostic_business_observation_count += 1
        self.diagnostic_last_business_rejection = dict(
            reason=reason,
            source_pose_stamp_ns=stamp,
            control_stamp_ns=clock,
            formal_parking_pass=False,
        )
        return True

    def recapture_before_first_qualification(self, record):
        """在首次资格前撤销一次未合格的零尝试；原捕获时钟保持不变。

        仅当尚未产生 first_actor_zero_ack、无 failure、已 ready 且处于
        handoff_zero/zero_hold 时生效。计数饱和在 2**63-1；清空 ready/handoff/
        origin/dwell 并退回 capture，同时失效 10 Hz 捕获证据（固定窗口不受影响），
        但 start_clock/deadline 不重置。返回是否执行了撤销。
        """
        if (
            self.first_actor_zero_ack is not None
            or self.failure is not None
            or self.ready_stamp is None
            or self.stage not in ("handoff_zero", "zero_hold")
        ):
            return False
        # 计数饱和，避免长跑溢出。
        self.preT0_attempts_disqualified = min(self.preT0_attempts_disqualified + 1, 2**63 - 1)
        if self.preT0_first_disqualification is None:
            self.preT0_first_disqualification = record
        self.preT0_last_disqualification = record
        self.ready_stamp = self.ready_clock = self.handoff_clock = None
        self.first_zero_publication_clock = None
        self.origin = self.origin_yaw = None
        self.dwell = None
        self.stage = "capture"
        self.deceleration_trigger = None
        self.invalidate_pose10hz_capture()
        return True

    def guard(self, reason):
        """保护/中断入口：已推进则锁存失败，未推进则退回 capture。

        仅 enabled 时生效。已有 ready_stamp（已进入零交接）时把原因写成
        ordinary_zero_hold_interrupted，并进入 failed；否则只把阶段退回 capture、
        清空 dwell、失效捕获证据，不修改 deadline，也不锁存失败。
        """
        if not self.enabled:
            return
        if self.ready_stamp is not None:
            # 新的实际保护优先于已观察到的业务执行许可；首个业务事实仍留在账本/标记里。
            if (
                diagnostic_business_failure(self.failure)
                and self.failure == self.diagnostic_business_failure_reason
            ):
                self.failure = "ordinary_zero_hold_interrupted: " + str(reason)
            else:
                self.failure = self.failure or "ordinary_zero_hold_interrupted: " + str(reason)
            self.stage = "failed"
        else:
            self.stage = "capture"
            self.dwell = None
            self.invalidate_pose10hz_capture()

    def update(
        self,
        stamp,
        clock,
        position,
        yaw,
        velocity,
        yawdot,
        body_wz,
        goal_distance,
        yaw_error,
        *,
        diagnostic=None,
        sensor_pose=None,
    ):
        """推进普通停车阶段并返回 stage 字符串；未启用时返回 None。

        参数：stamp 位姿头时间（ns），clock 控制时钟（ns），position 世界坐标（m），
        yaw 航向（rad），velocity body 速度（m/s），yawdot 欧拉 yaw 角速度（rad/s），
        body_wz body 角速度（rad/s），goal_distance 到目标平面距离（m），yaw_error
        航向误差（rad）。

        已锁存失败且不属于已记录的诊断业务失败时，直接返回 failed。启用 pose10hz
        时委托 update_pose10hz。否则：
        已有 ready_stamp 时只做零保持漂移/超时检查（xy>0.05 m、dyaw>0.1 rad、平面
        速度>0.08 m/s、yawdot 或 body_wz>0.1 rad/s 即失败）；尚未 ready 时先看
        deadline，再按 close=25 mm/35 mrad、low=30 mm/s/60 mrad/s 与 600 ms 驻留
        判定，满足即进入 handoff_zero。诊断观察开启时会额外严格校验实测数值有限
        且因果。
        """
        if not self.enabled:
            return None
        if self.failure and not self.diagnostic_failure_is_observed(diagnostic):
            return "failed"
        if self.pose10hz:
            return self.update_pose10hz(
                stamp, clock, position, yaw, goal_distance, yaw_error, sensor_pose
            )
        if _diagnostic_enabled(diagnostic):
            # 仅评分的诊断例外也不接受畸形实测状态。
            p, v = np.asarray(position, dtype=float), np.asarray(velocity, dtype=float)
            scalars = np.asarray([yaw, yawdot, body_wz, goal_distance, yaw_error], dtype=float)
            if (
                type(stamp) is not int
                or stamp < 0
                or type(clock) is not int
                or clock < stamp
                or p.shape != (3,)
                or v.shape != (3,)
                or not np.isfinite(p).all()
                or not np.isfinite(v).all()
                or not np.isfinite(scalars).all()
                or goal_distance < 0
            ):
                raise ValueError("Finite causal actual parking measurements required")
        # 已进入零交接：只允许在实测漂移/速度超限或 ACK 超时处失败。
        if self.ready_stamp is not None:
            if clock > self.deadline and self.first_actor_zero_ack is None:
                if not self.observe_business_rejection(
                    "ordinary_actual_zero_ACK_timeout",
                    stamp,
                    clock,
                    dict(deadline_clock_ns=self.deadline, first_actor_zero_ack_missing=True),
                    diagnostic,
                ):
                    self.failure = "ordinary_actual_zero_ACK_timeout"
                    self.stage = "failed"
                    return self.stage
            # 零保持漂移/运动上限：xy 0.05 m、yaw 0.1 rad、平面速度 0.08 m/s、
            # yawdot/body_wz 0.1 rad/s（硬编码）。
            xy = float(np.linalg.norm(np.asarray(position)[:2] - self.origin[:2]))
            dyaw = abs(math.atan2(math.sin(yaw - self.origin_yaw), math.cos(yaw - self.origin_yaw)))
            if (
                xy > 0.05
                or dyaw > 0.1
                or np.linalg.norm(np.asarray(velocity)[:2]) > 0.08
                or abs(yawdot) > 0.1
                or abs(body_wz) > 0.1
            ):
                if self.observe_business_rejection(
                    "ordinary_zero_hold_actual_motion_or_drift_exceeded",
                    stamp,
                    clock,
                    dict(
                        xy_drift_m=xy,
                        yaw_drift_rad=dyaw,
                        velocity_body=np.asarray(velocity).tolist(),
                        euler_yawrate_radps=yawdot,
                        body_wz_radps=body_wz,
                    ),
                    diagnostic,
                ):
                    self.stage = "zero_hold"
                else:
                    self.failure = "ordinary_zero_hold_actual_motion_or_drift_exceeded"
                    self.stage = "failed"
            else:
                self.stage = "zero_hold"
            return self.stage
        # 尚未 ready：超过 deadline 即捕获/减速超时。
        if clock > self.deadline:
            if not self.observe_business_rejection(
                "ordinary_precise_capture_deceleration_timeout",
                stamp,
                clock,
                dict(
                    deadline_clock_ns=self.deadline,
                    goal_distance_xy_m=goal_distance,
                    yaw_error_rad=yaw_error,
                    velocity_body=np.asarray(velocity).tolist(),
                    euler_yawrate_radps=yawdot,
                ),
                diagnostic,
            ):
                self.failure = "ordinary_precise_capture_deceleration_timeout"
                self.stage = "failed"
                return self.stage
        # 严格资格：平面 25 mm、航向 35 mrad；低运动：平面速度 <0.03 m/s、
        # yawdot <0.06 rad/s。
        close = goal_distance <= 0.025 and abs(yaw_error) <= 0.035
        low = np.linalg.norm(np.asarray(velocity)[:2]) < 0.03 and abs(yawdot) < 0.06
        # 触发迟滞落在未改动的严格 25 mm 资格之内。
        was_decelerating = self.stage == "controlled_deceleration"
        band = 0.020 if was_decelerating else 0.015
        braking_band = goal_distance <= band and abs(yaw_error) <= 0.035
        self.stage = "controlled_deceleration" if braking_band else "capture"
        # 记录本次减速触发证据：进入 15 mm、退出 20 mm，资格仍是 25 mm。
        self.deceleration_trigger = dict(
            entry_xy_m=0.015,
            exit_xy_m=0.020,
            was_decelerating=was_decelerating,
            current_xy_limit_m=band,
            actual_goal_distance_m=float(goal_distance),
            inside=bool(braking_band),
            qualification_xy_m=0.025,
            qualification_changed=False,
        )
        # 驻留 600 ms 且期间持续满足 close/low，才进入 handoff_zero。
        self.dwell = (stamp if self.dwell is None else self.dwell) if close and low else None
        if self.dwell is not None and stamp - self.dwell >= 600_000_000:
            self.ready_stamp = stamp
            self.ready_clock = clock
            self.handoff_clock = clock
            self.origin = np.asarray(position, float).copy()
            self.origin_yaw = float(yaw)
            self.stage = "handoff_zero"
        return self.stage

    def evidence(self):
        """返回停车阶段证据字典（纯拼装，无 IO）。

        stage/failure/各时钟与 ACK 原样输出；formal_T0_set_here 与
        navigation_ground_truth_used 恒为 False。R106 来源保持仅在启用时输出计数
        与首末记录；pose10hz 启用时输出 parking_rule 和窗口摘要，并声明高频运动
        不参与决策。
        """
        result = dict(
            schema="ordinary_terminal_parking_stages/v1",
            enabled=self.enabled,
            stage=self.stage,
            capture_begin_clock_ns=self.start_clock,
            capture_deadline_clock_ns=self.deadline,
            precise_dwell_start_pose_stamp_ns=self.dwell,
            ready_pose_stamp_ns=self.ready_stamp,
            ready_clock_ns=self.ready_clock,
            handoff_clock_ns=self.handoff_clock,
            first_zero_publication_clock_ns=self.first_zero_publication_clock,
            first_actor_zero_ack=self.first_actor_zero_ack,
            failure=self.failure,
            controlled_deceleration_trigger=self.deceleration_trigger,
            formal_T0_set_here=False,
            navigation_ground_truth_used=False,
            diagnostic_business_failure_reason=self.diagnostic_business_failure_reason,
            diagnostic_business_run=self.diagnostic_business_run,
            diagnostic_business_observation_count=self.diagnostic_business_observation_count,
            diagnostic_last_business_rejection=self.diagnostic_last_business_rejection,
            formal_parking_acceptance_invalid=self.diagnostic_business_failure_reason is not None,
        )
        if self.preT0_attempts_disqualified:
            result["preT0_attempts_disqualified"] = self.preT0_attempts_disqualified
            result["preT0_first_disqualification"] = self.preT0_first_disqualification
            result["preT0_last_disqualification"] = self.preT0_last_disqualification
        if self.prequalification_source_hold_enabled:
            result["r106_prequalification_source_hold"] = dict(
                prequalification_source_hold_enabled=self.prequalification_source_hold_enabled,
                count=self.prequalification_source_hold_count,
                first=None
                if self.prequalification_source_hold_first is None
                else dict(self.prequalification_source_hold_first),
                last=None
                if self.prequalification_source_hold_last is None
                else dict(self.prequalification_source_hold_last),
            )
        if self.pose10hz:
            result["parking_rule"] = "ordinary_pose10hz_zero_ack_fixed5/v1"
            if self.pose_grid is not None:
                summary = self.pose_grid.evidence()
                summary.pop("samples", None)
                result["pose10hz_window"] = summary
            result["high_frequency_motion_used_for_decision"] = False
        return result
