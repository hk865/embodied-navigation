#!/usr/bin/env python3
"""Teacher-specific finite wrapper around existing measured-pose Go2 SCAN.

The shared controller consumes SLAM, actual registered clouds and checked SCAN
trajectories. This wrapper narrows scope, fixes duplicate-stamp acceptance and
requires measured stopping before obstacle recovery. It owns no joint actuator.
"""

# 中文模块说明（首轮补充；上方 docstring 恢复 HEAD 原文，以保持 --help/--doc 文案不变）：
# 在既有 measured-pose Go2 SCAN 之上、面向 Teacher 的有限包装。
#
# 共享控制器消费 SLAM、真实配准点云与已检查的 SCAN 轨迹。本包装只收窄适用范围、
# 修正重复时间戳的接受逻辑，要求障碍恢复前先实测停车。它不拥有任何关节执行器。


from embodied_navigation.localization_mapping.known_scene_hold_guard import record_clear_release
import embodied_navigation.common.paths as _package_paths
import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
import numpy as np
from embodied_navigation.common.observation import api as _observer_api

_r87_diag = _observer_api("r87_diagnostic_probe")

HERE = Path(__file__).resolve().parent
ROOT = _package_paths.TEACHER_ROOT
DEMO = _package_paths.DEMO_ROOT
NAV = _package_paths.NAVIGATION_ROOT
from embodied_navigation.navigation import base_node as base
from embodied_navigation.common.runtime_context import load as load_context
from embodied_navigation.common.input_freshness import (
    enabled as minimal_baseline_enabled,
    raw_imu_fresh,
    control_sources_stale,
    pose_cloud_fresh,
    wall_stale,
)
from embodied_navigation.control.tracking.control_core import rotation_xyzw, body_tilt
from embodied_navigation.common.runtime_io import EvidenceWriter, latest_sensor_qos
from embodied_navigation.control.io import make_evidence_writer
from embodied_navigation.control.parking.transition_gate import TeacherHeadingGate
from embodied_navigation.control.safety.guard_audit import NativeGuardAudit, result_json
from std_msgs.msg import String
from embodied_navigation.navigation.mission.dynamic_stop_event import published_stop_event
from embodied_navigation.control.parking.navigation_stop_window import (
    reset_transition,
    bind_transition_context,
)

NATIVE_STEERING_GUARD = base.steering_obstacle_ahead
_SCAN_SHORT_INTAKE = os.environ.get("SCAN_SHORT_INTAKE", "0")
if _SCAN_SHORT_INTAKE not in ("0", "1"):
    raise ValueError("SCAN_SHORT_INTAKE must be 0 or 1")
SCAN_SHORT_INTAKE_ENABLED = _SCAN_SHORT_INTAKE == "1"


class RecordedPublisher:
    """在每帧状态 JSON 中补充本次运行不可变证据的状态发布者。

    被包装的发布者是共享控制器原有的发布者；本类只把实测回执、已发布的停止事实等
    字段加进 JSON 主体，不改变原状态字段的语义。
    """

    def __init__(self, publisher, run, node):
        self.publisher = publisher
        self.run = run
        self.node = node

    def publish(self, msg):
        """发布增强后的状态，并为本次 tick 追加/原子写入证据。"""
        data = json.loads(msg.data)
        # 每次发布都显式声明实验模式与范围，并固定 ground truth 未被使用。
        data.update(
            mode=self.node.profile["controller_kind"],
            scope=self.node.profile["experiment"],
            navigation_ground_truth_used=False,
            policy_observations=self.node.profile["policy_observations"],
        )
        data["known_scene_localization"] = copy.deepcopy(
            getattr(self.node, "known_scene_inputs", None)
        )
        data["known_scene_runtime_status"] = copy.deepcopy(
            getattr(self.node, "known_scene_status", None)
        )
        data["known_scene_scan_epoch"] = copy.deepcopy(
            getattr(self.node, "known_scene_scan_epoch", None)
        )
        bridge_receipt = getattr(self.node, "bridge_updated", None)
        data["bridge_callback_received_monotonic_wall"] = (
            bridge_receipt
            if type(bridge_receipt) in (int, float) and math.isfinite(bridge_receipt)
            else None
        )
        data["dynamic_stop_event"] = published_stop_event(self.node, data)
        data["last_obstacle_clear_release"] = getattr(self.node, "last_clear_release", None)
        data["controller_kind"] = self.node.profile["controller_kind"]
        data["outer_velocity_controller"] = (
            "Actual SLAM header-event geometric path/yaw PD and COM speed PI; causal IMU and actual Teacher input ACK; native SCAN collision guard"
        )
        data["cascade_parking"] = copy.deepcopy(getattr(self.node, "cascade_last_row", None))
        data["ordinary_terminal_failure_latched"] = copy.deepcopy(
            getattr(self.node, "ordinary_terminal_failure_latched", None)
        )
        data["path_admission"] = dict(
            hold=getattr(self.node, "path_admission_hold", False),
            replan_pending=getattr(self.node, "path_admission_replan_pending", False),
            decision_sequence=getattr(self.node, "path_admission_sequence", 0),
            last_decision=copy.deepcopy(getattr(self.node, "path_admission_last_decision", None)),
            active_reference_stamp=getattr(self.node, "active_reference_stamp", None),
            pending_reference_stamp=self.node.trajectory_association.reference_stamp,
            occupancy_or_terrain_support_certified=False,
        )
        # TeacherHeadingGate 的转向交接证据（相位、锁定航向、零速确认等）。
        data["teacher_transition"] = self.node.heading_gate.evidence(
            self.node.get_clock().now().nanoseconds / 1e9, time.monotonic()
        )
        # after_measured_dwell 策略下，纯转速率上限取自 Teacher gate；到点仍必须由
        # 原 ArrivalWindow 确认。
        if self.node.profile.get("arrival_stop_policy") == "after_measured_dwell":
            data["motion_limits"]["pure_turn_rate_rad_s"] = self.node.heading_gate.max_turn_rate
            data["arrival_stop_policy"] = "after_measured_dwell"
            data["arrival_stop_policy_source"] = (
                "Actual raw SLAM original control-region ArrivalWindow; no tracking-pose arrival"
            )
        # 先发 JSON（allow_nan=False：出现 NaN 直接抛错，而不是发出非法状态），
        # 再追加 JSONL 并原子写快照。
        self.publisher.publish(String(data=json.dumps(data, ensure_ascii=False, allow_nan=False)))
        self.node.evidence.append(
            self.run / "navigation_status.jsonl",
            {
                **data,
                "monotonic_wall": time.monotonic(),
                "ros_sim_time": self.node.get_clock().now().nanoseconds / 1e9,
            },
        )
        self.node.evidence.atomic(self.run / "navigation_status.json", data)


class TeacherNavigation(base.Navigation):
    """Teacher 专用包装：实测位姿门控、guard 回执，不拥有执行器。

    路径/控制计算委托给共享基类控制器；本子类保留实测输入、倾角/执行桥/证据门，
    并为隔离仿真记录“被接受的输入”与“原生 guard”回执。与基类的主要差异是：用
    TeacherHeadingGate 替换基类 HeadingGate、用 profile 覆盖 ROS 参数、把状态
    发布者换成 RecordedPublisher。
    """

    def __init__(self, run):
        self._bridge_safety_handoff = None
        self.run = run
        # profile 来自本次 run 的 runtime_context.json（业务阈值/开关由 prepare
        # 生成后 pin 进该文件）。
        receipt = load_context(run / "runtime_context.json")
        self.profile = receipt["profile"]
        super().__init__()
        self.evidence = make_evidence_writer(self.run, self.profile)
        self.gyro_body = None
        self.gyro_stamp = None
        self.gyro_wall = None
        # 替换基类 HeadingGate：转向交接、零速确认与主动制动由 Teacher gate 负责。
        self.heading_gate = TeacherHeadingGate(self.profile)
        self.handoff_scope_sha256 = (
            self.run.name
        )  # 交接范围用运行实例名标识；不用设置或源码字节。
        # 包装原发布者：状态 JSON 增加本次 run 的证据字段并落盘。
        self.status_pub = RecordedPublisher(self.status_pub, run, self)
        self.bridge_updated = -math.inf
        self.guard_cloud = None
        self.guard_cloud_receipt = None
        self.pose_quaternion = None
        self.guard_sequence = 0
        self.pending_guard_row = None
        self.trajectory_archive_reference = None
        self.last_actual_guard_clock_ns = None
        self.guard_continuity_resets = 0
        self.native_guard_audit = NativeGuardAudit(NATIVE_STEERING_GUARD)
        # 本模块实例只属于独立的 Teacher 进程；共享的 camera 模式文件与原生几何
        # 都保持原样。原生 guard 通过显式子类方法调用，不修改任何全局状态。

    def destroy_node(self):
        """销毁前关闭 bridge safety handoff，并把交接快照写到 run 目录。"""
        try:
            handoff = getattr(self, "_bridge_safety_handoff", None)
            if handoff is not None:
                handoff.close(self, service_negative=base.rclpy.ok())
                (self.run / "bridge_safety_handoff.json").write_text(
                    json.dumps(handoff.snapshot(), allow_nan=False) + "\n"
                )
        finally:
            result = super().destroy_node()
        return result

    def create_subscription(self, msg_type, topic, callback, qos_profile, *args, **kwargs):
        # SCAN_SHORT_INTAKE=1 时，四类 SCAN 输入订阅必须复用 R86 原回调组，且要求
        # R88 异步物理已开启（=1）、R116 接收压力关闭（=0）；不满足则启动即报错。
        if SCAN_SHORT_INTAKE_ENABLED and topic in (
            "/demo/navigation/reference_attempt",
            "/demo/navigation/bspline",
            "/demo/navigation/trajectory_metadata",
            "/demo/navigation/route_definition",
        ):
            if (
                os.environ.get("R88_ASYNC_PHYSICAL", "0") != "1"
                or os.environ.get("R116_RECEIVE_PRESSURE", "0") != "0"
            ):
                raise ValueError("SCAN short intake requires R88 and R116 disabled")
            options = self._r86_subscription_options()
            group = options.get("callback_group")
            if group is None:
                raise ValueError("SCAN short intake requires the original R86 group")
            kwargs["callback_group"] = group
        # 高频传感器统一用 depth=1 BEST_EFFORT（只保留最新一帧）。
        if topic in ("/demo/slam/body_odom", "/cloud_registered_full", "/livox/imu"):
            qos_profile = latest_sensor_qos()
        # /demo/control/safety 用 depth=1；启用 handoff 时挂到 R86 intake 回调组，
        # 并在订阅建立后绑定。
        if topic == "/demo/control/safety":
            qos_profile = 1
            from embodied_navigation.control.safety.bridge_safety_handoff import selected, Handoff

            if selected():
                group = getattr(self, "_r86_intake_group", None)
                if group is None:
                    raise ValueError("Bridge ingress requires the existing R86 intake group")
                self._bridge_safety_handoff = Handoff(self)
                kwargs["callback_group"] = group
        subscription = super().create_subscription(
            msg_type, topic, callback, qos_profile, *args, **kwargs
        )
        if topic == "/demo/control/safety" and self._bridge_safety_handoff is not None:
            self._bridge_safety_handoff.bind_subscription(subscription)
        return subscription

    def create_causal_imu_subscription(self, callback):
        # 同一 IMU 源的独立历史消费者；原来的“最新倾角”仍是 depth=1，并独占
        # 物理安全状态的判定权。
        from embodied_navigation.control.safety.imu_causal_buffer import enabled, FIFO_DEPTH

        if not enabled(self.profile):
            raise RuntimeError("causal_imu_buffer_not_enabled")
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

        # FIFO_DEPTH 由 imu_causal_buffer 决定；BEST_EFFORT + 独立互斥回调组。
        self.causal_imu_callback_group = MutuallyExclusiveCallbackGroup()
        qos = base.QoSProfile(depth=FIFO_DEPTH, reliability=base.ReliabilityPolicy.BEST_EFFORT)
        return super().create_subscription(
            base.Imu, "/livox/imu", callback, qos, callback_group=self.causal_imu_callback_group
        )

    def declare_parameter(self, name, value=None, *args, **kwargs):
        """把基类声明的 ROS 参数替换为本次 run 的 profile 取值。

        scenario_path 指向 run 目录下的 navigation_scenario.json；max_speed 取
        profile["max_speed_mps"]；pose_timeout 与 cloud_timeout 共用
        profile["pose_cloud_timeout_s"]。因此 Teacher 路径的数值来自 profile，
        不是 launch 参数或基类默认值。
        """
        if name == "scenario_path":
            value = str(self.run / "navigation_scenario.json")
        if name == "max_speed":
            value = self.profile["max_speed_mps"]
        if name in ("pose_timeout", "cloud_timeout"):
            value = self.profile["pose_cloud_timeout_s"]
        return super().declare_parameter(name, value, *args, **kwargs)

    def on_bridge_safety(self, msg):
        """接收执行桥安全状态；只接受本控制器类型、来源 scan_slam 且状态合法的消息。"""
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        # 三个字段任一不符即丢弃：controller_kind、source、state 白名单。
        if (
            data.get("mode") != self.profile["controller_kind"]
            or data.get("source") != "scan_slam"
            or data.get("state") not in ("hold", "ready", "failed")
        ):
            return
        # 墙钟新鲜度：消息自带的 monotonic_wall 必须落在 [now-0.3, now+0.05) 秒内；
        # 0.3 s 与 0.05 s 未来容差为本文件硬编码。
        try:
            if minimal_baseline_enabled(self.profile):
                now = time.monotonic()
                receipt = float(data["monotonic_wall"])
                if wall_stale(
                    self,
                    "bridge_status_message",
                    now,
                    receipt,
                    0.3,
                    not -0.05 <= now - receipt < 0.3,
                    future_tolerance=0.05,
                ):
                    return
            else:
                if not -0.05 <= time.monotonic() - float(data["monotonic_wall"]) < 0.3:
                    return
        except (ValueError, TypeError, KeyError):
            return
        receipt = time.monotonic()
        handoff = getattr(self, "_bridge_safety_handoff", None)
        if handoff is not None:
            handoff.receive(data, receipt)
            return
        self._accept_bridge_safety(data, receipt)

    def _accept_bridge_safety(self, data, receipt, *, force_protection=False, service_wall=None):
        """采用一条已通过门控的 bridge 状态；必要时立即执行倾角/执行桥保护。"""
        # bridge_updated 是墙钟回执，供 apply_tilt_guard 的 bridge 新鲜度门使用。
        self.bridge_updated = receipt
        self.bridge_safety = data
        if (force_protection or self.state == "running") and self.apply_tilt_guard(
            receipt if service_wall is None else service_wall,
            self.get_clock().now().nanoseconds / 1e9,
        ):
            self.reset_region_arrival("protected")
            # 不把“bridge hold → 零命令 → bridge 状态”回灌成新的零命令；正在移动的
            # 命令仍然立即停。
            if force_protection or any(self.command):
                self.publish_command()
            if self.state == "failed":
                self.publish_status()

    def on_imu(self, msg):
        """接收原始 IMU：更新物理安全用倾角缓存，并在运行中立即执行保护停车。"""
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds / 1e9
        q = msg.orientation
        # 物理安全状态只由“最新原始样本”决定；存在因果缓冲时取它的原始入口。
        intake = getattr(getattr(self, "imu_source_buffer", None), "safety_intake", None)
        original_raw = self.raw_imu if intake is None else intake.original_raw_imu
        accepted = original_raw.update(
            [q.x, q.y, q.z, q.w],
            self.stamp(msg) / 1e9,
            now,
            ros_now,
            msg.orientation_covariance[0] >= 0,
        )
        if accepted:
            if intake is not None:
                intake.note_original_accepted(round(ros_now * 1e9))
            self.counts["imu"] += 1
            # 角速度转到机身系后缓存，供 TeacherHeadingGate 观察；非有限值不更新。
            v = msg.angular_velocity
            gyro = self.imu_body_rotation @ np.array([v.x, v.y, v.z])
            if np.isfinite(gyro).all():
                self.gyro_body = gyro.tolist()
                self.gyro_stamp = self.stamp(msg) / 1e9
                self.gyro_wall = now
        if self.state == "running" and self.apply_tilt_guard(now, ros_now):
            self.reset_region_arrival("protected")
            # 保护在本回调内就停住移动中的机器人；已经是零的保持命令交给 20 Hz
            # 定时器心跳即可，不在这里重复发。
            if any(self.command):
                self.publish_command()
            if self.state == "failed":
                self.publish_status()

    def apply_tilt_guard(self, now, ros_now):
        """合并实测 SLAM/IMU 倾角与执行桥/证据故障；True 表示立即停车。"""
        buffer = getattr(self, "imu_source_buffer", None)
        intake = getattr(buffer, "safety_intake", None)
        pending_peak = None if intake is None else intake.consume(self, now, ros_now)
        # 通信 hold 不是实测倾角：不能因为新点云延迟就重置已停住 Teacher 的航向/路径；
        # 物理 IMU/SLAM 倾角保护保留原有阈值。
        slam_tilt = body_tilt(self.rotation)
        fresh = raw_imu_fresh(self, now, ros_now, self.raw_imu.fresh(now, ros_now))
        sources = [("SLAM", slam_tilt)] + ([("raw_imu", self.raw_imu.tilt)] if fresh else [])
        # 恢复中断/高倾角/严重倾角是一个事件：后续任一端口来的正常样本不能把它抹掉。
        # 仍按原阈值与恢复状态机消费该事件。
        if pending_peak is not None:
            sources.append(("raw_imu_pending_peak", pending_peak[4]))
        self.max_tilt = max(self.max_tilt, *(tilt for _, tilt in sources))
        # 阈值与基类一致：0.50 rad 锁定 failed，0.30 rad 保持，0.18 rad 且 0.8 s 恢复；
        # 代码与现有文档未说明这些阈值的标定依据。
        severe = [name for name, tilt in sources if tilt >= 0.50]
        if self.bridge_safety.get("state") == "failed":
            severe.append("execution_bridge")
        # 证据写入错误被当作 severe 故障：无法留证时直接锁定 failed。
        if self.evidence.error:
            severe.append("evidence_writer")
        if severe:
            self.state = "failed"
            self.tilt_source = "+".join(severe)
            self.message = self.tilt_source + "保护已锁定，停车"
            return True
        high = [name for name, tilt in sources if tilt >= 0.30]
        if high:
            if not self.tilt_hold:
                self.tilt_stops += 1
            self.tilt_hold = True
            self.tilt_source = "+".join(high)
            self.tilt_clear_since = None
        # 原始 IMU 超时与倾角无关，单独按停车处理。
        if not fresh:
            self.message = "原始IMU超时，停车"
            self.tilt_clear_since = None
            return True
        # 恢复要求两路 < 0.18 rad、SLAM 位姿新鲜且连续 0.8 s；恢复后清空轨迹并
        # 重新请求 SCAN。
        if self.tilt_hold:
            if all(tilt < 0.18 for _, tilt in sources) and pose_cloud_fresh(
                self, "pose", now, ros_now, now - self.pose_updated < self.pose_timeout
            ):
                if self.tilt_clear_since is None:
                    self.tilt_clear_since = ros_now
                if ros_now - self.tilt_clear_since >= 0.8:
                    self.tilt_hold = False
                    self.tilt_source = None
                    reset_transition(self, "tilt_recovery")
                    self.samples = None
                    self.last_reference = -math.inf
            else:
                self.tilt_clear_since = None
            self.message = "实际倾角保护，停车等待稳定"
            return True
        # bridge 回执超过 0.30 s 或状态不是 ready 时停车，等待执行门/传感器预热。
        if (
            wall_stale(
                self,
                "bridge_callback",
                now,
                self.bridge_updated,
                0.30,
                now - self.bridge_updated > 0.30,
            )
            or self.bridge_safety.get("state") != "ready"
        ):
            self.message = "Teacher 执行命令门或 SLAM 传感器预热未就绪，停车"
            return True
        self.tilt_source = None
        return False

    @_r87_diag.observe("teacher.odom")
    def on_odom(self, msg):
        """按原门控顺序校验一帧实测位姿回调，然后委托基类处理。"""
        stamp = self.stamp(msg)
        ns = self.get_clock().now().nanoseconds
        received = time.monotonic()
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        p = msg.pose.pose.position
        # 保持原有短路顺序与同一个 ROS 时钟；observer 只记录实际走到的分支，
        # 不新增第二道门。
        rejection = None
        if msg.header.frame_id != "camera_init":
            rejection = "frame_id"
        elif msg.child_frame_id != "demo_slam_body":
            rejection = "child_frame_id"
        elif stamp <= self.pose_stamp:
            rejection = "source_not_advancing"
        # header 年龄允许 0.05 s 未来容差，上限为 pose_timeout 秒。
        elif not -0.05e9 <= ns - stamp < self.pose_timeout * 1e9:
            rejection = "header_age"
        # 四元数模长允许 0.98..1.02，超出即视为非法姿态。
        elif not 0.98 <= q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w <= 1.02:
            rejection = "quaternion_norm"
        elif not all(math.isfinite(x) for x in (p.x, p.y, p.z, v.x, v.y, v.z, w.x, w.y, w.z)):
            rejection = "nonfinite_pose_or_twist"
        _r87_diag.mark(
            "teacher.odom.decision",
            source_ns=stamp,
            decision_ros_ns=ns,
            prior_pose_stamp_ns=self.pose_stamp,
            header_age_ns=ns - stamp,
            timeout_s=self.pose_timeout,
            rejection=rejection,
        )
        if rejection is not None:
            _r87_diag.anomaly("teacher_odom_rejected:" + rejection, source_ns=stamp)
            return
        with _r87_diag.span("teacher.odom.shared", source_ns=stamp):
            super().on_odom(msg)
        _r87_diag.mark(
            "teacher.odom.shared_return", source_ns=stamp, accepted_stamp_ns=self.pose_stamp
        )
        if self.pose_stamp == stamp:
            self.pose_quaternion = [q.x, q.y, q.z, q.w]
        # stop_zero 是“实测停车”的候选事实：命令为零、未处于受控制动、且允许
        # 正向停车资格（_r86_positive_stop_allowed）。
        stop_zero = (
            not any(self.command)
            and not getattr(self.heading_gate, "controlled_braking", False)
            and getattr(self, "_r86_positive_stop_allowed", True)
        )
        if self.gyro_body is not None and received - self.gyro_wall < 0.3:
            if getattr(self.heading_gate, "handoff_active", False) and hasattr(self, "get_ack"):
                self.heading_gate.bind_zero_ack(
                    self.get_ack(pose_stamp_ns=stamp),
                    self.handoff_scope_sha256,
                    self.request_id,
                    self.waypoint_index,
                    ns,
                    round(received * 1e9),
                )
            # 只有 IMU 角速度足够新（< 0.3 s）时才把实测运动喂给 Teacher gate；
            # 位置/四元数一并提供，用于零速与航向收敛判定。
            self.heading_gate.observe(
                stamp / 1e9,
                received,
                [v.x, v.y, v.z],
                w.z,
                self.gyro_body,
                self.gyro_stamp,
                stop_zero,
                position=[p.x, p.y, p.z],
                quaternion=[q.x, q.y, q.z, q.w],
                positive_qualification_allowed=getattr(self, "_r86_positive_stop_allowed", True),
            )
        # 逐帧位姿回执：原始 header、机身速度、零命令与停车证据一并归档。
        row = {
            "stamp_ns": stamp,
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "position": [p.x, p.y, p.z],
            "quaternion": [q.x, q.y, q.z, q.w],
            "body_velocity": [v.x, v.y, v.z],
            "body_angular_velocity": [w.x, w.y, w.z],
            "received_monotonic_wall": received,
            "callback_ros_clock_ns": ns,
            "sim_age_at_callback_s": (ns - stamp) / 1e9,
            "body_gyro": self.gyro_body,
            "gyro_stamp_s": self.gyro_stamp,
            "zero_velocity_command": not any(self.command),
            "heading_stop_zero_candidate": stop_zero,
            "controlled_braking": bool(getattr(self.heading_gate, "controlled_braking", False)),
            "measured_stop_evidence": self.heading_gate.evidence(ns / 1e9, received),
        }
        self.evidence.append(self.run / "navigation_slam_poses.jsonl", row)

    def on_cloud(self, msg):
        """记录 header 新鲜度/重复判定，并归档被接受点云的回执。"""
        stamp = self.stamp(msg)
        ns = self.get_clock().now().nanoseconds
        received = time.monotonic()
        if getattr(self, "_r86_committing_bundle", None) is not None:
            from embodied_navigation.perception.sensors.feedback_mailbox import receipt_ns

            received = receipt_ns(self, "cloud", round(received * 1e9)) / 1e9
        # 只接受严格推进的 header：重复或回退的点云直接拒绝并记录判定。
        if stamp <= self.cloud_stamp:
            self.note_cloud_decision(
                "rejected_duplicate_or_backward_header",
                decision_ros_clock_ns=ns,
                previous_accepted_stamp_ns=int(self.cloud_stamp),
            )
            return
        # header 年龄窗口：[now-cloud_timeout, now+0.05 s)；超出即视为过期点云。
        if not -0.05e9 <= ns - stamp < self.cloud_timeout * 1e9:
            self.note_cloud_decision(
                "rejected_header_age",
                decision_ros_clock_ns=ns,
                actual_header_age_ns=ns - stamp,
                timeout_s=self.cloud_timeout,
            )
            return
        super().on_cloud(msg)
        if self.cloud_stamp == stamp and self.cloud_input_context is not None:
            c = self.cloud_input_context
            row = {
                "stamp_ns": stamp,
                "frame_id": msg.header.frame_id,
                "input_points": int(msg.width * msg.height),
                "filtered_points": c["filtered_points"],
                "self_filtered_points": c["self_filtered_points"],
                "nearest_slam_pose_stamp_ns": c["filtering_body_stamp_ns"],
                "received_monotonic_wall": received,
                "decoded_monotonic_wall": time.monotonic(),
                "callback_ros_clock_ns": ns,
                "sim_age_at_callback_s": (ns - stamp) / 1e9,
                "filtered_xyz_float64_sha256": hashlib.sha256(
                    np.asarray(self.cloud, dtype="<f8").tobytes()
                ).hexdigest(),
            }
            # guard_cloud / guard_cloud_receipt 是 native guard 允许使用的唯一输入
            # 身份：之后 record_native_guard 会核对对象身份与 stamp 是否仍然一致。
            self.guard_cloud = self.cloud
            self.guard_cloud_receipt = {
                **row,
                "filtering_body_pose": c["filtering_body_pose"].tolist(),
                "filtering_body_rotation": c["filtering_body_rotation"].tolist(),
            }
            self.evidence.append(self.run / "navigation_cloud_history.jsonl", row)

    def note_cloud_decision(self, reason, **details):
        # 只读钩子：记录实际发生的拒绝/接受分支。它从不重试消息、不重新解码点云，
        # 也不改变任何新鲜度门。
        row = getattr(self, "pending_cloud_callback_row", None)
        if row is not None:
            row.update(reason=reason, decision_details=details)

    def accept_spline(self, msg, metadata):
        """基类接受轨迹后，为本次新轨迹归档压缩 npz 与 JSON 摘要。"""
        before = self.replans
        super().accept_spline(msg, metadata)
        # 只有 replans 增加且 samples 就位（确实接受了一条新轨迹）时才归档。
        if self.replans == before or self.samples is None:
            return
        directory = self.run / "navigation_trajectories"
        array_path = directory / f"{self.replans:06d}_trajectory_{self.active_trajectory_id}.npz"
        coefficients = np.array([[p.x, p.y, p.z] for p in msg.pos_pts])
        knots = np.array(msg.knots)
        samples = self.samples.copy()
        document = {
            "trajectory_id": self.active_trajectory_id,
            "order": int(msg.order),
            "metadata": metadata,
            "reference_stamp": self.active_reference_stamp,
            "request_id": self.request_id,
            "waypoint_index": self.waypoint_index,
            "accepted_slam_stamp_ns": self.pose_stamp,
            "array_file": array_path.name,
            "source": "actual SCAN committed B-spline and associated metadata; sensor SLAM feedback",
        }
        self.trajectory_archive_reference = str(array_path.relative_to(self.run))

        # 文件写盘放进 evidence.enqueue 的异步队列，避免占用控制回调。
        def archive():
            directory.mkdir(exist_ok=True)
            np.savez_compressed(array_path, coefficients=coefficients, knots=knots, samples=samples)
            document["array_sha256"] = hashlib.sha256(array_path.read_bytes()).hexdigest()
            array_path.with_suffix(".json").write_text(
                json.dumps(document, allow_nan=False, ensure_ascii=False) + "\n"
            )

        self.evidence.enqueue(archive)

    def on_request(self, msg):
        """只接受与本次 run 冻结的 navigation_request.json 完全一致的请求。"""
        try:
            # 与冻结请求逐字段比较；不一致直接拒绝，不进入基类状态机。
            requested = json.loads(msg.data)
            frozen = json.loads((self.run / "navigation_request.json").read_text())
            if requested != frozen:
                raise ValueError("Only this run frozen SLAM-relative route is eligible")
        except (OSError, ValueError, TypeError) as error:
            self.last_rejected = str(error)
            self.publish_status()
            return
        super().on_request(msg)

    def measured_region_arrival(self):
        """after_measured_dwell 策略下，必须由原 ArrivalWindow 确认后才允许推进。"""
        inside, arrived = super().measured_region_arrival()
        if self.profile.get("arrival_stop_policy") == "after_measured_dwell":
            # 原 raw-SLAM ArrivalWindow、保护标志、整数时间戳、观测间隔与停顿都
            # 不变；在同一个窗口确认到达之前，只允许沿已检查的 SCAN 路线继续，
            # 下面的原生路径终点/障碍/倾角/超时停车全部照常生效。
            return inside and arrived, arrived
        return inside, arrived

    def control(self, now):
        """Teacher 控制封装：证据/guard 连续性门通过后才调用基类状态机。

        先做两项 Teacher 专属检查：证据写入错误直接 failed；障碍保持期间要求真实
        guard 输入连续新鲜。之后调用基类 control()，并在返回后补齐 guard 行证据与
        pre_turn/settle 的零速说明。
        """
        # 共享恢复流程已经要求 1 s 无阻断与一条新的已检查轨迹；这里再要求实测
        # 机身运动足够小才允许恢复。
        # 证据队列出错时不再运动：进入 failed 并发布零速。
        if self.evidence.error:
            self.state = "failed"
            self.message = self.evidence.error
            self.publish_command()
            return
        # 只有配置了 clear_guard_gap_max_sim_s 的 run 才启用 guard 连续性检查。
        if self.profile.get("clear_guard_gap_max_sim_s") is not None and self.obstacle_hold:
            ns = self.get_clock().now().nanoseconds
            if not self.guard_continuity(ns, now):
                # 即使被额外的实测源新鲜度检查挡在本回调，物理保护仍然执行；连续
                # 8 s 失联升级为 failed。
                self.apply_tilt_guard(now, ns / 1e9)
                self.reset_region_arrival("stale")
                self.stale_since = self.stale_since or now
                if now - self.stale_since > 8.0:
                    self.state = "failed"
                    self.message = "实际SLAM/点云guard输入连续失联超过8秒，停车"
                self.publish_command()
                return
            if (
                self.obstacle_clear_since is not None
                and ns / 1e9 - self.obstacle_clear_since >= 1.0
                and not now - self.last_obstacle_check >= 0.05
            ):
                # 缓存的无障碍结果不能单独完成原来的 1 s 门：必须等下一次已排定的
                # 原生评估，不额外跑几何检查，也不刷新输入时间戳。
                protected = self.apply_tilt_guard(now, ns / 1e9)
                stale = control_sources_stale(
                    self,
                    now,
                    ns / 1e9,
                    (
                        now - self.pose_updated > self.pose_timeout
                        or now - self.cloud_updated > self.cloud_timeout
                        or not self.raw_imu.fresh(now, ns / 1e9)
                    ),
                )
                if protected or stale:
                    self.reset_region_arrival("protected" if protected else "stale")
                    self.obstacle_clear_since = None
                self.publish_command()
                return
        # 未确认实测停住前不允许累计障碍清除时间。
        if self.obstacle_hold and not self.heading_gate.stopped(
            self.get_clock().now().nanoseconds / 1e9, now
        ):
            self.obstacle_clear_since = None
        # 每周期先清空上一行待归档 guard 回执；基类 control() 评估 native guard
        # 时会重新设置它。
        self.pending_guard_row = None
        try:
            super().control(now)
        finally:
            if self.pending_guard_row is not None:
                row = self.pending_guard_row
                after_ns = self.get_clock().now().nanoseconds
                row.update(
                    obstacle_hold_after=bool(self.obstacle_hold),
                    obstacle_stops_after=int(self.obstacle_stops),
                    clear_start_after_s=self.obstacle_clear_since,
                    clear_elapsed_after_s=None
                    if self.obstacle_clear_since is None
                    else after_ns / 1e9 - self.obstacle_clear_since,
                    after_control_ros_clock_ns=after_ns,
                    command_after_control=list(self.command),
                    zero_requested_after_control=not any(self.command),
                    command_ros_sim_time_s=self.last_command_time,
                    state_after_control=self.state,
                    obstacle_resumes_after=self.obstacle_resumes,
                    command_source="actual sensor SLAM/SCAN controller; no mover or simulator pose navigation input",
                    evidence_queue_error=self.evidence.error,
                )
                record_clear_release(self, row)
                self.evidence.append(self.run / "navigation_guard_history.jsonl", row)
                self.pending_guard_row = None
        # pre_turn/settle 期间用实测速度与角速度确认停车；消息与基类不同，但命令
        # 仍然是零速。
        if (
            self.state == "running"
            and not getattr(self, "reference_consistency_hold", False)
            and self.heading_gate.phase in ("pre_turn", "settle")
            and not (
                getattr(self.heading_gate, "controlled_settle", False)
                or getattr(self.heading_gate, "controlled_braking", False)
            )
        ):
            self.message = "Teacher 等待实际SLAM机身速度和IMU角速确认停车，保持零速度"

    def record_native_guard(self, cloud, pose, checked_target, steering_direction, route):
        """在接受过的输入上运行未改动的原生 guard，并冻结一行回执。"""
        ns = self.get_clock().now().nanoseconds
        received = time.monotonic()
        previous_guard_ns = self.last_actual_guard_clock_ns
        if self.profile.get("clear_guard_gap_max_sim_s") is not None:
            # 实际 guard 时钟重复或倒退是硬故障：先置 failed，再抛错终止本周期的
            # guard 记录。
            if previous_guard_ns is not None and ns <= previous_guard_ns:
                self.state = "failed"
                self.message = "实际native guard时钟重复或倒退，停车"
                raise RuntimeError(self.message)
            # 障碍保持期间，guard 输入必须仍然新鲜；否则拒绝这次评估。
            if self.obstacle_hold and not self.guard_continuity(ns, received):
                raise RuntimeError(
                    "Actual native guard source ceased being fresh before evaluation"
                )
        # 输入身份校验：点云对象、回执 stamp 与当前已接受点云、以及 SLAM 四元数
        # 必须完全对应，否则不允许评估。
        if (
            cloud is not self.guard_cloud
            or self.guard_cloud_receipt is None
            or self.guard_cloud_receipt["stamp_ns"] != self.cloud_stamp
            or self.pose_quaternion is None
        ):
            raise RuntimeError(
                "Actual native guard lacks an exact accepted cloud/SLAM input receipt"
            )
        with _r87_diag.span("teacher.native_geometry", source_ns=self.pose_stamp):
            result = self.native_guard_audit(cloud, pose, checked_target, steering_direction, route)
        self.last_actual_guard_clock_ns = ns
        from embodied_navigation.control.safety.clear_guard_window import ClearGuardWindow

        # 清除窗要求 >=10 次真实评估、跨度 >=1 s、且尾随 1.35 s 内无阻断；未满足
        # 时不会放行障碍恢复。
        if not hasattr(self, "_actual_clear_guard_window"):
            self._actual_clear_guard_window = ClearGuardWindow()
        if self.obstacle_clear_since is None:
            self._actual_clear_guard_window.clocks.clear()
        self._actual_clear_guard_ready = self._actual_clear_guard_window.observe(
            ns, bool(result[0]), True
        )
        self.guard_sequence += 1
        c = self.guard_cloud_receipt
        # 冻结这一行回执：字段与原始几何结果一一对应，之后只读、不再补算。
        self.pending_guard_row = {
            "schema": "teacher_native_steering_guard/v1",
            "sequence": self.guard_sequence,
            "compute_ros_clock_ns": ns,
            "compute_sim_time_s": ns / 1e9,
            "compute_monotonic_wall": received,
            "request_id": self.request_id,
            "waypoint_index": self.waypoint_index,
            "trajectory_id": self.active_trajectory_id,
            "reference_stamp": self.active_reference_stamp,
            "trajectory_archive_file": self.trajectory_archive_reference,
            "control_pose_stamp_ns": int(self.pose_stamp),
            "control_pose": np.asarray(pose).tolist(),
            "control_quaternion": list(self.pose_quaternion),
            "control_rotation": self.rotation.tolist(),
            "tracking_pose": self.tracking_pose.tolist(),
            "control_pose_received_monotonic_wall": self.pose_updated,
            "cloud_header_stamp_ns": int(c["stamp_ns"]),
            "cloud_frame_id": c["frame_id"],
            "cloud_received_monotonic_wall": c["received_monotonic_wall"],
            "cloud_callback_ros_clock_ns": c["callback_ros_clock_ns"],
            "cloud_filtering_body_stamp_ns": c["nearest_slam_pose_stamp_ns"],
            "cloud_filtering_body_pose": c["filtering_body_pose"],
            "cloud_filtering_body_rotation": c["filtering_body_rotation"],
            "filtered_xyz_float64_sha256": c["filtered_xyz_float64_sha256"],
            "filtered_points": c["filtered_points"],
            "self_filtered_points": c["self_filtered_points"],
            "checked_target": np.asarray(checked_target).tolist(),
            "steering_direction": np.asarray(steering_direction).tolist(),
            "goal": self.waypoints[self.waypoint_index].tolist(),
            "goal_corridor_result": result_json(self.native_guard_audit.calls[0][1]),
            "motion_corridor_result": result_json(self.native_guard_audit.calls[1][1]),
            "motion_corridor_target": self.native_guard_audit.calls[1][0].tolist(),
            "union_result": result_json(result),
            "obstacle_hold_before": bool(self.obstacle_hold),
            "obstacle_stops_before": int(self.obstacle_stops),
            "clear_start_before_s": self.obstacle_clear_since,
            "clear_elapsed_before_s": None
            if self.obstacle_clear_since is None
            else ns / 1e9 - self.obstacle_clear_since,
            "native_geometry_source": "unchanged control_core.steering_obstacle_ahead code object and original two obstacle_ahead evaluations",
        }
        reference = getattr(self, "_program_guard_route_reference", None)
        if reference is not None and route is getattr(self, "_program_guard_route", None):
            self.pending_guard_row.update(
                route_reference=dict(reference),
                route_float64_sha256=reference["route_float64_sha256"],
            )
        else:
            # 未绑定路线引用的 legacy 输入仍保留完整的可重放行（含路线坐标与哈希）。
            self.pending_guard_row.update(
                route=np.asarray(route).tolist(),
                route_float64_sha256=hashlib.sha256(
                    np.asarray(route, dtype="<f8").tobytes()
                ).hexdigest(),
            )
        if self.profile.get("clear_guard_gap_max_sim_s") is not None:
            self.pending_guard_row.update(
                previous_actual_guard_clock_ns=previous_guard_ns,
                clear_guard_gap_max_sim_s=self.profile["clear_guard_gap_max_sim_s"],
                clear_guard_continuity_resets=self.guard_continuity_resets,
                clear_release_requires_this_callback_actual_guard=True,
            )
        return result

    def guard_continuity(self, ns, wall):
        """V44 专属：原清除计时器不能跨越缺失的实际评估。"""
        # 允许的最大实际评估间隔（profile.clear_guard_gap_max_sim_s，转成 ns）。
        maximum_ns = round(self.profile["clear_guard_gap_max_sim_s"] * 1e9)
        previous = self.last_actual_guard_clock_ns
        if previous is not None and ns < previous:
            self.state = "failed"
            self.message = "实际native guard时钟倒退，停车"
            self.obstacle_clear_since = None
            return False
        reset = previous is None or ns - previous > maximum_ns
        fresh_ns = round(self.profile["pose_cloud_timeout_s"] * 1e9)
        # 源新鲜窗口：[ns-pose_cloud_timeout, ns+0.05 s]；stamp<0 表示从未收到。
        source_fresh = all(
            stamp >= 0 and -50_000_000 <= ns - stamp < fresh_ns
            for stamp in (self.pose_stamp, self.cloud_stamp)
        )
        # 间隔超限或源过期都会清空清除起点并计数；这里只记录，不重算任何几何。
        if reset or not source_fresh:
            before = self.obstacle_clear_since
            self.obstacle_clear_since = None
            if before is not None:
                self.guard_continuity_resets += 1
                self.evidence.append(
                    self.run / "navigation_guard_continuity_history.jsonl",
                    {
                        "schema": "teacher_actual_guard_continuity/v1",
                        "compute_ros_clock_ns": ns,
                        "compute_monotonic_wall": wall,
                        "previous_actual_guard_clock_ns": previous,
                        "actual_guard_gap_ns": None if previous is None else ns - previous,
                        "pose_stamp_ns": self.pose_stamp,
                        "cloud_stamp_ns": self.cloud_stamp,
                        "clear_start_before_s": before,
                        "clear_start_after_s": None,
                        "reason": "actual_source_stale" if not source_fresh else "actual_guard_gap",
                        "extra_geometry_computed": False,
                    },
                )
        if not source_fresh:
            self.message = "实际SLAM/点云header超时，停车等待fresh guard输入"
        return source_fresh


def main():
    """CLI 入口：绑定冻结的 run 目录、自旋节点，然后排空证据队列。"""
    import rclpy

    # description 取自模块 docstring，因此模块文档的措辞会反映到 --help 输出。
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run", type=Path, required=True)
    args, ros = p.parse_known_args()
    run = args.run.resolve()
    # 导出解析后的 run 目录：同进程组件与子进程（如 geometry_worker）都通过它
    # 定位本次 run。
    os.environ["DEMO_RUN_DIR"] = str(run)
    rclpy.init(args=ros)
    node = TeacherNavigation(run)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_command()
        node.event_archive.close()
        try:
            node.evidence.close()
        finally:
            # dynamic_fixture 运行时额外写一份 guard writer 回执，声明队列已排空。
            if node.profile.get("dynamic_fixture"):
                (run / "navigation_guard_writer_receipt.json").write_text(
                    json.dumps(
                        {
                            "schema": "teacher_native_steering_guard_writer/v1",
                            "expected_records": node.guard_sequence,
                            "status": "failed" if node.evidence.error else "drained",
                            "queue_error": node.evidence.error,
                            "silent_record_loss_permitted": False,
                            "clear_guard_continuity_resets": node.guard_continuity_resets,
                            "source": "actual dynamic controller shutdown after bounded evidence queue drain",
                            "navigation_verified": False,
                        },
                        allow_nan=False,
                        ensure_ascii=False,
                    )
                    + "\n"
                )
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
