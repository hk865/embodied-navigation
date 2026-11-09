#!/usr/bin/env python3
"""按实测位姿执行 SCAN 轨迹：输入新鲜度看门狗与 LiDAR 障碍停车/恢复。

职责：本模块持有在线导航状态机。``Navigation.control()`` 每个控制周期运行一次
（默认 20 Hz，周期 0.05 s），用 idle/running/succeeded/failed/stopped 五个任务
状态与 drive/pre_turn/align/settle 四个转向相位，对 SCAN 轨迹做门控与替换；同时
负责输入超时停车、倾角保护、障碍停车/恢复与状态发布。
默认原生 launch 经 ``ros_node.TeacherNavigation`` 与
``pid_navigation.PIDNavigation`` 到达本类。

边界（不做什么）：节点从不设置仿真位姿、从不订阅真值，也绝不在 SCAN 尚未产出
已关联轨迹时直接驶向未经检查的航点。关节/力矩执行不在本模块：本模块只发布
``/demo/cmd_vel`` 速度指令、冻结标志与导航状态。

时间口径：``now`` 是 ``time.monotonic()`` 墙钟秒，``ros_now`` 是 ROS（仿真）
时钟秒；两者在本文件中分别传递，超时判定各自沿用原有来源。
"""

from __future__ import annotations

import copy
import json
import hashlib
import math
import time
from embodied_navigation.perception.sensors.feedback_mailbox import receipt_ns as _r86_receipt_ns
from collections import deque
from dataclasses import replace
from pathlib import Path as FilePath
import numpy as np
from embodied_navigation.localization_mapping.known_scene_hold_guard import (
    arm_teacher_recovery,
    start_teacher_recovery,
    complete_teacher_recovery,
)
from embodied_navigation.common.input_freshness import (
    enabled as minimal_baseline_enabled,
    raw_imu_fresh,
    control_sources_stale,
    pose_cloud_fresh,
)
from scipy.interpolate import BSpline
from embodied_navigation.common.observation import api as _observer_api

_callback_cost_observe = _observer_api("r74_callback_cost_witness").observe

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _ros
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    HistoryPolicy,
)
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String, Bool, UInt64MultiArray, MultiArrayDimension
from scan_planner_msgs.msg import Bspline

from embodied_navigation.control.tracking.control_core import (
    rotation_xyzw,
    validate_request,
    follow_trajectory,
    obstacle_ahead,
    remove_go2_self_returns,
    body_tilt,
    limit_acceleration,
    HeadingGate,
    TrackingPositionFilter,
    trajectory_has_progress,
    RawImuTilt,
)
from embodied_navigation.control.tracking.control_core import steering_obstacle_ahead
from embodied_navigation.planning.trajectory_contract import TrajectoryAssociation
from embodied_navigation.navigation.mission.goal_regions import (
    parse_request as parse_goal_request,
    definitions_sha256,
    contains as goal_contains,
    contains_control as goal_contains_control,
    ArrivalWindow,
)
import embodied_navigation.control.tracking.control_core as control_core
from embodied_navigation.control.tracking.replan_policy import low_command_is_exhausted_path
from embodied_navigation.control.parking.navigation_stop_window import (
    reset_transition,
    bind_transition_context,
)
from embodied_navigation.common.observation import api as _observer_api

_r85_intake_begin = _observer_api("r85_protocol_intake_trace").begin
_r85_intake_ignored = _observer_api("r85_protocol_intake_trace").ignored
_r85_intake_associating = _observer_api("r85_protocol_intake_trace").associating
_r85_intake_parsed = _observer_api("r85_protocol_intake_trace").parsed
_r85_intake_associated = _observer_api("r85_protocol_intake_trace").associated
_r85_intake_exception = _observer_api("r85_protocol_intake_trace").exception
_r85_intake_accepting = _observer_api("r85_protocol_intake_trace").accepting
_r85_intake_returned = _observer_api("r85_protocol_intake_trace").returned
_r85_intake_done = _observer_api("r85_protocol_intake_trace").done
_r85_intake_enabled = _observer_api("r85_protocol_intake_trace").enabled
_r85_intake_summary = _observer_api("r85_protocol_intake_trace").summary


from embodied_navigation.common.observation import api as _observer_api

_r74_protocol_observation = _observer_api("r74_protocol_observation_python")


class Navigation(Node):
    """默认的 measured-pose 导航节点，同时充当 Teacher 子类的基类。

    任务状态（``self.state``，即状态话题的 ``state`` 字段）：``idle`` 等待任务；
    ``running`` 已接受请求、正在执行；``succeeded`` 全部航点由实测 SLAM 位置确认
    到达；``failed`` 倾角/传感器/交接/证据等保护或超时失败并停车；``stopped``
    收到 ``/demo/navigation/stop`` 主动停车。只有 ``running`` 会下发非零速度，
    其余状态每个控制周期都发布零速。

    转向相位（``self.heading_gate.phase``）：``drive`` 可带平移跟随；``pre_turn``
    与 ``settle`` 要求零速停稳 1 s；``align`` 只转不走。Teacher 子类会用
    control.parking.transition_gate.TeacherHeadingGate 替换这个 gate。

    参数：``max_speed``（m/s）、``arrival_radius``（m）、``pose_timeout``（s）、
    ``cloud_timeout``（s）、``scenario_path`` 都是 ROS 参数声明默认值，启动时
    读取一次；``ros_node.TeacherNavigation`` 会用 profile 覆盖前四项与
    ``scenario_path``。

    本类只提供公共执行骨架：轨迹选择、PID 输出、guard 几何等由子类钩子提供
    （如 ``select_pid_velocity``、``evaluate_motion_guard``、
    ``execution_guard_route``；基类本身不定义它们）。
    """

    def destroy_node(self):
        """节点销毁前先冲刷一次协议观测缓冲，再调用父类销毁。"""
        try:
            _r74_protocol_observation.flush_once(self)
        finally:
            result = super().destroy_node()
        return result

    def create_subscription(self, msg_type, topic, callback, qos_profile, *args, **kwargs):
        # 统一订阅入口：只对 ``/clock`` 等少数 topic 注入订阅选项（独立回调组），
        # 其余 topic 的 QoS 与回调语义原样透传。
        from embodied_navigation.common.time.clock_groups import subscription_options

        options = subscription_options(self, topic, kwargs)
        return super().create_subscription(msg_type, topic, callback, qos_profile, *args, **options)

    def subscribe_slam_body(self, msg_type, topic, callback, qos_profile, *args, **kwargs):
        """显式的机身反馈输入端口；订阅语义与默认 create_subscription 完全一致。"""
        return self.create_subscription(msg_type, topic, callback, qos_profile, *args, **kwargs)

    def subscribe_registered_cloud(self, msg_type, topic, callback, qos_profile, *args, **kwargs):
        """显式的配准点云输入端口；保留原有 QoS 与回调归属。"""
        return self.create_subscription(msg_type, topic, callback, qos_profile, *args, **kwargs)

    def __init__(self):
        super().__init__("demo_navigation", parameter_overrides=[])
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        # 以下四项为 ROS 参数声明默认值（启动时读取一次，运行中不刷新）：
        # max_speed m/s、arrival_radius m、pose_timeout s、cloud_timeout s。
        # Teacher 子类在 declare_parameter 中用 profile 值覆盖它们。
        self.max_speed = self.declare_parameter("max_speed", 0.12).value
        self.arrival_radius = self.declare_parameter("arrival_radius", 0.22).value
        self.pose_timeout = self.declare_parameter("pose_timeout", 0.8).value
        self.cloud_timeout = self.declare_parameter("cloud_timeout", 1.0).value
        # scenario_path 默认指向包内 simulation/scenario.json；本节点只从该文件
        # 读取 IMU 的固定安装/参考旋转，不读取其它场景字段。
        scenario_path = self.declare_parameter(
            "scenario_path",
            str(FilePath(__file__).resolve().parent.parent / "simulation/scenario.json"),
        ).value
        imu_config = json.loads(FilePath(scenario_path).read_text())["sensors"]["imu"][
            "orientation_reference"
        ]
        self.imu_body_rotation = rotation_xyzw(imu_config["body_imu_quaternion"])
        self.imu_reference_rotation = rotation_xyzw(imu_config["world_quaternion"])
        if (
            not np.isfinite(self.imu_body_rotation).all()
            or not np.isfinite(self.imu_reference_rotation).all()
        ):
            raise ValueError("scenario IMU fixed rotations must be finite")
        self.raw_imu = RawImuTilt(self.imu_body_rotation, self.imu_reference_rotation)
        self.tilt_source = None
        self.bridge_safety = {}
        self.request_id = None
        # 初始为 idle：在收到请求前，control() 每个周期只发布零速。
        self.state = "idle"
        self.message = "等待任务"
        self.waypoints = np.empty((0, 3))
        self.goals = ()
        self.goals_definition_sha256 = None
        self.region_arrivals = []
        self.region_arrival = None
        self.region_arrival_evidence = None
        self.region_raw_pose = None
        self.waypoint_index = 0
        self.pose = None
        self.rotation = np.eye(3)
        self.pose_history = deque(maxlen=50)
        self.self_filtered_points = 0
        self.pose_updated = self.cloud_updated = -math.inf
        self.pose_stamp = self.cloud_stamp = -1
        self.cloud = None
        self.cloud_input_context = None
        self.obstacle_guard_context = None
        self.pending_obstacle_event = None
        self.last_obstacle_event_id = None
        self.obstacle_event_capture_error = None
        from embodied_navigation.common.runtime_telemetry import RuntimeEvents

        self.event_archive = RuntimeEvents()
        self.samples = None
        self.execution_path_exhaustion = None
        from embodied_navigation.planning.reference.automatic_reference_runtime import (
            enabled as r85_enabled,
        )

        if r85_enabled(getattr(self, "profile", {})):
            from embodied_navigation.planning.reference_protocol import R85TrajectoryAssociation

            self.trajectory_association = R85TrajectoryAssociation(self.run.name)
        else:
            self.trajectory_association = TrajectoryAssociation()
        self.last_reference_stamp = -1
        self.planning_start_state = None
        self.steering = None
        self.active_trajectory_id = None
        self.active_reference_stamp = None
        self.obstacle_resume_pending = False
        self.obstacle_hold_started_ros = None
        self.obstacle_stopped_duration = 0.0
        self.aligned_obstacle_resumes = 0
        self.replans = self.reference_requests = 0
        self.degenerate_splines = 0
        self.last_spline_rejected = None
        self.obstacle_hold = False
        self.obstacle_stops = self.obstacle_resumes = 0
        self.min_obstacle_clearance = None
        self.obstacle_clear_since = None
        self.arrival_since = None
        self.stale_since = None
        self.last_reference = -math.inf
        self.last_status = -math.inf
        self.last_obstacle_check = -math.inf
        self.obstacle_result = (False, None, 0)
        self.segment_start = None
        self.segment_started_ros = 0.0
        self.last_rejected = None
        self.counts = {"odom": 0, "cloud": 0, "imu": 0, "bspline": 0, "commands": 0}
        self.command = [0.0, 0.0, 0.0]
        self.last_command_time = self.get_clock().now().nanoseconds / 1e9
        self.tilt_hold = False
        self.tilt_stops = 0
        self.max_tilt = 0.0
        self.tilt_clear_since = None
        self.alignment_hold = True
        self.heading_gate = HeadingGate()
        self.tracking_filter = TrackingPositionFilter()
        self.tracking_pose = None
        self.yaw_rate = 0.0
        # 命令与冻结标志保持 10 的队列深度；状态/路径用 TRANSIENT_LOCAL(depth=1)
        # 锁存最后一条，迟到订阅者也能拿到当前快照。
        self.cmd_pub = self.create_publisher(Twist, "/demo/cmd_vel", 10)
        self.freeze_pub = self.create_publisher(Bool, "/demo/navigation/execution_frozen", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "/demo/navigation/status", latched)
        self.path_pub = self.create_publisher(Path, "/demo/navigation/path", latched)
        self.reference_pub = self.create_publisher(Path, "/demo/navigation/scan_reference", 1)
        if r85_enabled(getattr(self, "profile", {})):
            self.local_replan_pub = self.create_publisher(
                UInt64MultiArray, "/demo/navigation/local_replan_request", 10
            )
            self.create_subscription(
                String, "/demo/navigation/reference_attempt", self.on_reference_attempt, 10
            )
            definition_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                String,
                "/demo/navigation/route_definition",
                self.on_route_definition,
                definition_qos,
            )
        self.scan_odom_pub = self.create_publisher(Odometry, "/demo/navigation/scan_body_odom", 10)
        self.scan_cloud_pub = self.create_publisher(
            PointCloud2, "/demo/navigation/cloud", qos_profile_sensor_data
        )
        # 请求/停止是普通 reliable 深度 10 的消息；停止话题只在 data=True 时生效。
        self.create_subscription(String, "/demo/navigation/request", self.on_request, 10)
        self.create_subscription(Bool, "/demo/navigation/stop", self.on_stop, 10)
        self.subscribe_slam_body(
            Odometry, "/demo/slam/body_odom", self.on_odom, qos_profile_sensor_data
        )
        # 仅处理最新的原始样本（depth=1 BEST_EFFORT）；路线反馈仍以 SLAM/SCAN 为准。
        imu_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, "/livox/imu", self.on_imu, imu_qos)
        self.create_subscription(String, "/demo/control/safety", self.on_bridge_safety, 10)
        self.subscribe_registered_cloud(
            PointCloud2, "/cloud_registered_full", self.on_cloud, qos_profile_sensor_data
        )
        self.create_subscription(Bspline, "/demo/navigation/bspline", self.on_spline, 10)
        self.create_subscription(
            String, "/demo/navigation/trajectory_metadata", self.on_trajectory_metadata, 10
        )
        _r74_protocol_observation.setup(self)
        if r85_enabled(getattr(self, "profile", {})):
            # 在 steady 时钟上轮询，但只有实际 ROS 时钟同时越过此前的 control 与
            # guard 记录时才进入真正的 control；短重复周期保留原墙钟看门狗与回执不动。
            last_control_wall = None
            last_dispatched_clock_ns = None

            def r85_control_tick():
                nonlocal last_control_wall, last_dispatched_clock_ns
                last_control_wall = time.monotonic()
                last_dispatched_clock_ns = self.get_clock().now().nanoseconds
                self.tick()

            def r85_steady_control_tick():
                clock_ns = self.get_clock().now().nanoseconds
                prior_control = getattr(
                    getattr(self, "control_clock_hold", None), "last_clock_ns", None
                )
                prior_guard = getattr(self, "last_actual_guard_clock_ns", None)
                previous = (last_dispatched_clock_ns, prior_control, prior_guard)
                if any(value is not None and clock_ns < value for value in previous) or all(
                    value is None or clock_ns > value for value in previous
                ):
                    r85_control_tick()

            def r85_wall_watchdog():
                now = time.monotonic()
                clock_ns = self.get_clock().now().nanoseconds
                prior_control = getattr(
                    getattr(self, "control_clock_hold", None), "last_clock_ns", None
                )
                prior_guard = getattr(self, "last_actual_guard_clock_ns", None)
                backwards = any(
                    previous is not None and clock_ns < previous
                    for previous in (prior_control, prior_guard)
                )
                if (
                    last_control_wall is None
                    or now < last_control_wall
                    or now - last_control_wall >= 0.3
                    or backwards
                ):
                    r85_control_tick()

            # 两个 0.05 s（20 Hz）steady 定时器：一个按 ROS 时钟推进执行，一个做
            # 0.3 s 墙钟兜底（见 r85_wall_watchdog）。
            self.create_timer(
                0.05, r85_steady_control_tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
            )
            self.create_timer(
                0.05, r85_wall_watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME)
            )
        else:
            # 非 R85 路径：单个 20 Hz steady 定时器直接调用 tick()。
            self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def stamp(msg):
        """把 ROS header.stamp 转成整数纳秒；输入门用整数比较保证严格单调。"""
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def on_imu(self, msg):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds / 1e9
        q = msg.orientation
        accepted = self.raw_imu.update(
            [q.x, q.y, q.z, q.w],
            self.stamp(msg) / 1e9,
            now,
            ros_now,
            msg.orientation_covariance[0] >= 0,
        )
        if accepted:
            self.counts["imu"] += 1
        if self.state == "running":
            if self.apply_tilt_guard(now, ros_now):
                self.reset_region_arrival("protected")
                # 倾角保护在本回调内立即生效：先发精确零速，不等 20 Hz tick；
                # 若因此进入 failed，再补发一次状态。
                self.publish_command()
            if self.state == "failed":
                self.publish_status()

    def on_bridge_safety(self, msg):
        """接收执行桥安全状态；只在 state 为 hold/ready/failed 时采用。"""
        try:
            safety = json.loads(msg.data)
            if safety.get("state") not in ("hold", "ready", "failed"):
                return
        except (ValueError, AttributeError):
            return
        # JSON 解析失败或状态取值不在白名单时直接丢弃，不更新已有回执。
        self.bridge_safety = safety
        if self.state == "running" and self.apply_tilt_guard(
            time.monotonic(), self.get_clock().now().nanoseconds / 1e9
        ):
            self.reset_region_arrival("protected")
            self.publish_command()
            self.publish_status()

    def apply_tilt_guard(self, now, ros_now):
        """合并 SLAM/原始 IMU 倾角与执行桥状态；返回 True 表示本周期必须停车。

        阈值均为本文件硬编码：任一路倾角 ≥ 0.50 rad（约 28.6°）判为 severe 并锁定
        ``failed``；≥ 0.30 rad（约 17.2°）进入 tilt_hold 零速保持；恢复要求两路
        都 < 0.18 rad、执行桥不在 hold、持续 0.8 s ROS 时间且 SLAM 位姿新鲜。
        原始 IMU 缺失/超时同样返回 True。代码与现有文档未说明这些阈值的标定依据。
        """
        slam_tilt = body_tilt(self.rotation)
        raw_tilt = self.raw_imu.tilt
        self.max_tilt = max(self.max_tilt, slam_tilt, raw_tilt or 0.0)
        fresh = raw_imu_fresh(self, now, ros_now, self.raw_imu.fresh(now, ros_now))
        sources = [("SLAM", slam_tilt)]
        if fresh:
            sources.append(("raw_imu", raw_tilt))
        # 0.50 rad 超限是终态失败，不会自动恢复；需要新请求重新开始。
        severe = [source for source, tilt in sources if tilt >= 0.50]
        if self.bridge_safety.get("state") == "failed":
            severe.append("execution_bridge")
        if severe:
            self.tilt_source = "+".join(severe)
            self.state = "failed"
            self.message = (
                "执行桥安全保护已锁定失败，停车"
                if "execution_bridge" in severe
                else f"{self.tilt_source} 检测机身倾角超过 0.50 rad，停车"
            )
            return True
        # 0.30 rad 只触发可恢复的零速保持；恢复后要重新获取 SCAN 轨迹。
        high = [source for source, tilt in sources if tilt >= 0.30]
        bridge_hold = self.bridge_safety.get("state") == "hold"
        if bridge_hold:
            high.append("execution_bridge")
        if high:
            if not self.tilt_hold:
                self.tilt_stops += 1
            self.tilt_hold = True
            self.tilt_source = "+".join(high)
            if not bridge_hold or any(tilt >= 0.30 for _, tilt in sources):
                self.tilt_clear_since = None
        # 原始 IMU 缺失/时间戳超时与倾角无关，单独按停车处理。
        if not fresh:
            self.tilt_source = "raw_imu_stale"
            self.tilt_clear_since = None
            self.message = "原始 IMU 缺失或时间戳超时，停车等待传感器"
            return True
        # 恢复判据：两路倾角都 < 0.18 rad、SLAM 位姿新鲜、且连续 0.8 s ROS 时间；
        # 恢复后清空轨迹并重新请求 SCAN（alignment_hold=True 先做零速转向交接）。
        if self.tilt_hold:
            self.message = f"{self.tilt_source} 倾角保护，停车等待两路姿态稳定"
            if all(tilt < 0.18 for _, tilt in sources) and pose_cloud_fresh(
                self, "pose", now, ros_now, now - self.pose_updated <= self.pose_timeout
            ):
                if self.tilt_clear_since is None:
                    self.tilt_clear_since = ros_now
                if ros_now - self.tilt_clear_since >= 0.8 and not bridge_hold:
                    self.tilt_hold = False
                    self.tilt_source = None
                    self.alignment_hold = True
                    reset_transition(self, "tilt_recovery")
                    self.samples = None
                    self.execution_path_exhaustion = None
                    self.last_reference = -math.inf
            else:
                self.tilt_clear_since = None
            return True
        self.tilt_source = None
        return False

    def on_odom(self, msg):
        """处理一帧 SLAM 机身里程计；只接受 camera_init 系，其它坐标系直接拒绝。

        时间戳只推进时刷新 pose_updated 回执与跟踪滤波；重复/回退帧不刷新新鲜度，
        但仍会覆盖 self.pose 并转发给 SCAN。输入门统一在 control() 的 pose_timeout 上。
        """
        # 坐标系白名单：非 camera_init 的位姿直接拒绝，消息只记录用途。
        if msg.header.frame_id != "camera_init":
            self.message = "拒绝非 camera_init 的 SLAM 位姿"
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = np.array([p.x, p.y, p.z])
        if not np.isfinite(pose).all():
            return
        try:
            rotation = rotation_xyzw([q.x, q.y, q.z, q.w])
        except ValueError:
            return
        stamp = self.stamp(msg)
        if stamp > self.pose_stamp:
            self.region_raw_pose = pose.copy()
            self.pose_stamp = stamp
            self.pose_updated = (
                _r86_receipt_ns(self, "body", time.monotonic_ns()) / 1e9
                if getattr(self, "_r86_committing_bundle", None) is not None
                else time.monotonic()
            )
            self.tracking_pose = self.tracking_filter.update(pose, stamp / 1e9)
        self.pose, self.rotation = pose, rotation
        # 0.75/0.25 一阶低通，只用于状态上报的 filtered_yaw_rate，不参与控制。
        self.yaw_rate = 0.75 * self.yaw_rate + 0.25 * float(msg.twist.twist.angular.z)
        self.pose_history.append((stamp, pose.copy(), rotation.copy()))
        self.counts["odom"] += 1
        # SCAN 上游把 twist 当作世界系速度消费，而 ROS Odometry 规定 twist 位于
        # child_frame_id 系。这里把这一非标准输入显式转换后再转发：先旋转到世界系，
        # 再用自定义 child_frame_id 标明语义。
        scan = copy.deepcopy(msg)
        v = msg.twist.twist.linear
        world_velocity = rotation @ np.array([v.x, v.y, v.z])
        scan.twist.twist.linear.x, scan.twist.twist.linear.y, scan.twist.twist.linear.z = map(
            float, world_velocity
        )
        scan.child_frame_id = "demo_scan_body_world_twist"
        self.scan_odom_pub.publish(scan)

    def on_cloud(self, msg):
        """处理一帧配准点云：解码、按最近 SLAM 位姿做自体点过滤后转发。"""
        if msg.header.frame_id != "camera_init":
            self.message = "拒绝非 camera_init 的点云"
            self.note_cloud_decision("rejected_frame_id")
            return
        try:
            cloud = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            cloud = np.asarray(cloud).reshape((-1, 3))
            cloud = cloud[np.isfinite(cloud).all(axis=1)]
        except Exception as exc:
            self.message = f"点云解码失败: {exc}"
            self.note_cloud_decision("rejected_decode", error=type(exc).__name__ + ": " + str(exc))
            return
        if not len(cloud):
            self.note_cloud_decision("rejected_no_finite_points")
            return
        stamp = self.stamp(msg)
        if not self.pose_history:
            self.note_cloud_decision("rejected_no_slam_pose_history")
            return
        pose_stamp, body_pose, body_rotation = min(
            self.pose_history, key=lambda sample: abs(sample[0] - stamp)
        )
        # 150_000_000 ns = 150 ms 硬编码窗口：找不到与点云时间相邻的 SLAM 姿态时，
        # 不做自体过滤、不发布点云，保持停车。
        if abs(pose_stamp - stamp) > 150_000_000:
            self.message = "点云无邻近时刻SLAM机身姿态，自体过滤暂停并停车"
            self.note_cloud_decision(
                "rejected_no_nearby_slam_pose",
                closest_pose_stamp_ns=int(pose_stamp),
                pose_cloud_gap_ns=abs(pose_stamp - stamp),
            )
            return
        cloud, self.self_filtered_points = remove_go2_self_returns(cloud, body_pose, body_rotation)
        if not len(cloud):
            self.note_cloud_decision(
                "rejected_all_points_self_filtered",
                filtering_pose_stamp_ns=int(pose_stamp),
                self_filtered_points=int(self.self_filtered_points),
            )
            return
        if stamp > self.cloud_stamp:
            self.cloud_stamp = stamp
            self.cloud_updated = (
                _r86_receipt_ns(self, "cloud", time.monotonic_ns()) / 1e9
                if getattr(self, "_r86_committing_bundle", None) is not None
                else time.monotonic()
            )
        self.cloud = cloud
        # 每条消息独占不可变缓冲，后续回调只会整体替换它。保留这份精确缓冲可以
        # 避免把更晚的扫描记到一个缓存的 guard 结果上；裁剪/体素化不会进原生事件证据。
        try:
            self.cloud.setflags(write=False)
            self.cloud_input_context = dict(
                message_stamp_ns=stamp,
                frame_id=msg.header.frame_id,
                filtering_body_stamp_ns=pose_stamp,
                filtering_body_pose=body_pose.copy(),
                filtering_body_rotation=body_rotation.copy(),
                filtered_points=len(cloud),
                self_filtered_points=self.self_filtered_points,
            )
        except Exception as exc:
            self.cloud_input_context = None
            self.obstacle_event_capture_error = f"{type(exc).__name__}: {exc}"
        filtered = point_cloud2.create_cloud_xyz32(msg.header, cloud.astype(np.float32))
        self.scan_cloud_pub.publish(filtered)
        self.counts["cloud"] += 1
        self.note_cloud_decision(
            "accepted",
            filtering_pose_stamp_ns=int(pose_stamp),
            filtered_points=int(len(cloud)),
            self_filtered_points=int(self.self_filtered_points),
            cloud_input_context_available=self.cloud_input_context is not None,
        )

    def on_request(self, msg):
        """接受一个新任务请求并进入 running；重复或冲突请求只更新拒绝原因。

        请求 JSON 由 ``mission.goal_regions.parse_request`` 解析。legacy 目标沿用
        ROS 参数 ``arrival_radius``，新 schema 目标使用自带的到达定义。同一个
        ``request_id`` 的重发是幂等的，但目标定义哈希变化会被拒绝；``running`` 期间
        的新 ``request_id`` 一律拒绝，需要先 stop 或等任务结束。
        """
        try:
            request_id, goals = parse_goal_request(json.loads(msg.data))
            # 既有自定义 legacy 任务仍由 arrival_radius 参数决定到点半径。
            goals = tuple(replace(g, radius=self.arrival_radius) if g.legacy else g for g in goals)
            goal_hash = definitions_sha256(goals)
        except (ValueError, TypeError) as exc:
            self.last_rejected = str(exc)
            self.publish_status()
            return
        if request_id == self.request_id:
            if goal_hash != self.goals_definition_sha256:
                self.last_rejected = f"{request_id}: 同一任务 ID 的目标定义哈希不同，拒绝变更"
            self.publish_status()  # 幂等重发：状态照常发布，便于上游对账。
            return
        if self.state == "running":
            self.last_rejected = f"{request_id}: 已有任务正在运行"
            self.publish_status()
            return
        # 新请求从零开始：下面的赋值清空上一任务的轨迹、关联、障碍/倾角保持与计数。
        self.request_id, self.goals = request_id, goals
        self.waypoints = np.array([g.center for g in goals], dtype=float)
        self.goals_definition_sha256 = goal_hash
        self.region_arrivals = []
        self.region_arrival_evidence = None
        # legacy 请求用 0.4 s 计时到点（见 control()），新 schema 用 ArrivalWindow
        # 的实测停顿窗。
        self.region_arrival = None if goals[0].legacy else ArrivalWindow(goals[0])
        self.waypoint_index = 0
        # 进入 running 后，control() 才会在输入新鲜时请求 SCAN 规划。
        self.state = "running"
        self.message = "等待新鲜 SLAM 和点云，再请求 SCAN 规划"
        self.samples = None
        self.execution_path_exhaustion = None
        self.trajectory_association.reset()
        self.planning_start_state = None
        self.steering = None
        self.active_trajectory_id = None
        self.active_reference_stamp = None
        self.obstacle_resume_pending = False
        self.obstacle_hold_started_ros = None
        self.aligned_obstacle_resumes = 0
        self.replans = self.reference_requests = 0
        self.degenerate_splines = 0
        self.last_spline_rejected = None
        self.obstacle_stops = self.obstacle_resumes = 0
        self.min_obstacle_clearance = None
        self.obstacle_hold = False
        self.pending_obstacle_event = None
        self.obstacle_guard_context = None
        self.last_obstacle_event_id = None
        self.obstacle_event_capture_error = None
        self.tilt_hold = False
        self.tilt_stops = 0
        self.max_tilt = 0.0
        self.raw_imu.max_tilt = self.raw_imu.tilt or 0.0
        self.tilt_source = None
        self.tilt_clear_since = None
        self.alignment_hold = True
        reset_transition(self, "request_start")
        self.obstacle_clear_since = self.arrival_since = self.stale_since = None
        self.last_reference = -math.inf
        self.segment_start = None
        self.last_rejected = None
        self.publish_status()

    def on_stop(self, msg):
        """stop=True 时立即进入 stopped 并发布零速；stop=False 不改变状态。

        stopped 只能由新的 request 退出（on_request 允许非 running 状态重新接受
        任务），本回调不会自动恢复。
        """
        if msg.data:
            self.reset_region_arrival("stopped")
            self.state, self.message = "stopped", "任务已停止"
            self.samples = None
            self.execution_path_exhaustion = None
            self.obstacle_resume_pending = False
            self.publish_command()
            self.publish_status()

    def reset_region_arrival(self, reason):
        """清空到点停顿窗并记录原因；只作用于新 schema 的 ArrivalWindow。"""
        # evidence 中的 start_stamp_ns / dwell_ns 同步归零，避免把旧停顿算到新事件上。
        if self.region_arrival is not None:
            self.region_arrival.reset(reason)
            if self.region_arrival_evidence is not None:
                self.region_arrival_evidence["reason"] = reason
                self.region_arrival_evidence["start_stamp_ns"] = None
                self.region_arrival_evidence["dwell_ns"] = 0

    def measured_region_arrival(self):
        """仅新 schema 使用：位置与整数时间戳来自同一条被接受的 odom。

        返回 ``(control_inside, arrived)``：前者表示实测位置在控制到达带内（决定
        能否推进相位），后者表示 ArrivalWindow 已确认停顿。障碍保持、倾角保持或
        执行桥非 ready 时按 protected 传入，窗口会清零而不会累计到点时间。
        """
        g = self.goals[self.waypoint_index]
        pose = self.region_raw_pose
        if pose is None:
            self.reset_region_arrival("no_measured_pose")
            return False, False
        # protected 有三个来源：障碍保持、倾角保持、执行桥未处于 ready。
        protected = (
            self.obstacle_hold or self.tilt_hold or self.bridge_safety.get("state") != "ready"
        )
        inside = goal_contains(g, pose)
        control_inside = goal_contains_control(g, pose)
        arrived = self.region_arrival.observe(pose, self.pose_stamp, protected=bool(protected))
        since = self.region_arrival.since
        # 逐帧到点证据：写入状态话题，并在确认到达时进入 region_arrivals 归档。
        self.region_arrival_evidence = dict(
            request_id=self.request_id,
            goal_id=g.goal_id,
            waypoint_index=self.waypoint_index,
            goals_definition_sha256=self.goals_definition_sha256,
            goal_activated_ros_clock_ns=getattr(self, "segment_started_ros_ns", None),
            segment_start_pose_stamp_ns=getattr(self, "segment_start_pose_stamp_ns", None),
            stamp_ns=int(self.pose_stamp),
            start_stamp_ns=since,
            dwell_ns=0 if since is None else int(self.pose_stamp - since),
            raw_position=pose.tolist(),
            center_error_m=float(np.linalg.norm(pose - np.array(g.center))),
            region_inside=inside,
            control_region_inside=control_inside,
            protected=bool(protected),
            reason=self.region_arrival.reason,
            max_observation_gap_ns=self.region_arrival.max_gap_ns,
            arrival_definition=g.definition()["arrival"],
            control_arrival_definition=g.control_arrival_definition(),
        )
        return control_inside and not protected, arrived

    def on_spline(self, msg):
        """接收 SCAN B 样条；仅 running 且能关联到本次请求时才接受。

        与 metadata/route_definition 的配对由 ``trajectory_association`` 完成；
        关联失败、字段缺失或样条非法只记录拒绝原因，不改变已在执行的轨迹。
        """
        _r85_intake_context = _r85_intake_begin(self, "bspline", msg)
        _r74_protocol_context = _r74_protocol_observation.begin(
            self, "bspline", msg, _r85_intake_context
        )
        self.counts["bspline"] += 1
        # 非 running 时收到的轨迹一律忽略，可能是上一任务或其它运行的残留。
        if self.state != "running":
            _r85_intake_ignored(_r85_intake_context, "state_not_running")
            return
        try:
            _r85_intake_associating(_r85_intake_context)
            matched = self.trajectory_association.add_spline(msg)
            _r74_protocol_observation.associated(self, _r74_protocol_context, matched)
            _r85_intake_associated(_r85_intake_context, matched)
            if matched is not None:
                _r85_intake_accepting(_r85_intake_context)
                self.accept_spline(*matched)
                _r85_intake_returned(_r85_intake_context)
        except (ValueError, TypeError, KeyError) as exc:
            self.trajectory_association.reject(f"invalid B-spline association: {exc}")
            _r85_intake_exception(_r85_intake_context, exc)
            _r74_protocol_observation.exception(_r74_protocol_context, exc)
        _r85_intake_done(_r85_intake_context)

    def on_trajectory_metadata(self, msg):
        """接收轨迹元数据；与 B 样条组成一对后才交给 accept_spline。"""
        _r85_intake_context = _r85_intake_begin(self, "trajectory_metadata", msg)
        _r74_protocol_context = _r74_protocol_observation.begin(
            self, "trajectory_metadata", msg, _r85_intake_context
        )
        if self.state != "running":
            _r85_intake_ignored(_r85_intake_context, "state_not_running")
            return
        try:
            _r85_intake_associating(_r85_intake_context, parsing_json=True)
            value = _r74_protocol_observation.parsed(
                _r74_protocol_context, _r85_intake_parsed(_r85_intake_context, json.loads(msg.data))
            )
            if hasattr(self.trajectory_association, "add_definition"):
                # 有 R88 delivery 回执时用其接收墙钟，否则退回当前 monotonic 时间。
                delivery = getattr(getattr(self, "_r88_admission", None), "delivery", None)
                received_wall_ns = (
                    delivery["received_wall_ns"]
                    if delivery is not None and delivery["kind"] == "metadata"
                    else time.monotonic_ns()
                )
                matched = self.trajectory_association.add_metadata(
                    value, received_wall_ns=received_wall_ns
                )
            else:
                matched = self.trajectory_association.add_metadata(value)
            _r74_protocol_observation.associated(self, _r74_protocol_context, matched)
            _r85_intake_associated(_r85_intake_context, matched)
            if matched is not None:
                _r85_intake_accepting(_r85_intake_context)
                self.accept_spline(*matched)
                _r85_intake_returned(_r85_intake_context)
        except (ValueError, TypeError, KeyError) as exc:
            self.trajectory_association.reject(f"invalid trajectory metadata: {exc}")
            _r85_intake_exception(_r85_intake_context, exc)
            _r74_protocol_observation.exception(_r74_protocol_context, exc)
        _r85_intake_done(_r85_intake_context)

    def on_route_definition(self, msg):
        """低频的路线定义边界；R88 的派发由 PID 包装层负责。"""
        if self.state != "running" or not hasattr(self, "automatic_reference"):
            return
        association = self.trajectory_association
        try:
            value = json.loads(msg.data)
            previous = association.route_key
            matched = association.add_definition(value)
            if association.route_key != previous:
                hook = getattr(self, "_route_definition_bound", None)
                if callable(hook):
                    hook()
                self.evidence.append(
                    self.run / "scan_route_definitions.jsonl", association.route_definition
                )
            if matched is not None:
                self.accept_spline(*matched)
        except (ValueError, TypeError, KeyError) as exc:
            association.reject("invalid route definition: " + str(exc))

    def pump_route_definition_pending(self):
        """尝试消费一个已存在的 pending 配对；调用时机由原 PID/R88 持有者决定。"""
        if self.state != "running" or not hasattr(self.trajectory_association, "match_pending"):
            return False
        matched = self.trajectory_association.match_pending()
        if matched is None:
            return False
        self.accept_spline(*matched)
        return True

    @_callback_cost_observe("reference_attempt")
    def on_reference_attempt(self, msg):
        """接收 R85 参考请求尝试记录；与样条配对成功后同样走 accept_spline。"""
        _r85_intake_context = _r85_intake_begin(self, "reference_attempt", msg)
        _r74_protocol_context = _r74_protocol_observation.begin(
            self, "reference_attempt", msg, _r85_intake_context
        )
        if self.state != "running" or not hasattr(self, "automatic_reference"):
            _r85_intake_ignored(_r85_intake_context, "state_not_running_or_no_automatic_reference")
            return
        try:
            _r85_intake_associating(_r85_intake_context, parsing_json=True)
            value = json.loads(msg.data)
            _r85_intake_parsed(_r85_intake_context, value)
            _r74_protocol_observation.parsed(_r74_protocol_context, value)
            self.evidence.append(self.run / "r85_reference_attempts.jsonl", value)
            matched = self.trajectory_association.add_attempt(value)
            _r74_protocol_observation.associated(self, _r74_protocol_context, matched)
            _r85_intake_associated(_r85_intake_context, matched)
            if matched is not None:
                _r85_intake_accepting(_r85_intake_context)
                self.accept_spline(*matched)
                _r85_intake_returned(_r85_intake_context)
        except (ValueError, TypeError, KeyError) as exc:
            self.trajectory_association.reject("invalid scoped reference attempt: " + str(exc))
            _r85_intake_exception(_r85_intake_context, exc)
            _r74_protocol_observation.exception(_r74_protocol_context, exc)
        _r85_intake_done(_r85_intake_context)

    def make_local_replan_message(self, epoch, revision, sequence, stamp):
        """构造本地重规划请求：layout 固定 5 个 uint64（epoch/revision/sequence + stamp 两段）。

        ``stamp`` 是 R85 参考请求的二元时间戳（秒/纳秒），由自动参考运行时传入。
        """
        msg = UInt64MultiArray()
        msg.layout.dim = [MultiArrayDimension(label=self.run.name, size=5, stride=5)]
        msg.layout.data_offset = 0
        msg.data = [int(epoch), int(revision), int(sequence), int(stamp[0]), int(stamp[1])]
        return msg

    def accept_spline(self, msg, metadata):
        """校验一条已关联的 SCAN 三次 B 样条，通过后替换当前执行轨迹。

        校验项：order 必须为 3、节点数必须等于控制点数 + order + 1、系数与节点
        有限且参数区间非空；按约 0.08 的参数间隔采样，最少 2 点、最多 2000 点。
        起点距当前实测位姿超过 1.2 m（硬编码）的轨迹会被丢弃，或按诊断开关记录；
        没有几何前进量的样条按“原地停车样条”拒绝并等待重规划。校验失败只更新
        消息与拒绝原因，不会清掉已在执行的轨迹（无前进量分支除外）。
        """
        try:
            coefficients = np.array([[p.x, p.y, p.z] for p in msg.pos_pts])
            knots = np.array(msg.knots)
            if msg.order != 3 or len(knots) != len(coefficients) + msg.order + 1:
                raise ValueError("invalid cubic B-spline dimensions")
            begin, end = knots[msg.order], knots[-msg.order - 1]
            if not np.isfinite(coefficients).all() or not np.isfinite(knots).all() or end <= begin:
                raise ValueError("nonfinite or empty trajectory")
            curve = BSpline(knots, coefficients, msg.order)
            # 以约 0.08（参数单位）的间隔采样，限幅到 2..2000 点。
            samples = curve(
                np.linspace(begin, end, min(2000, max(2, int((end - begin) / 0.08) + 1)))
            )
            # 上面已校验来源；这里额外拒绝起点远离实测位姿的轨迹。1.2 m 为本文件
            # 硬编码门限，它不替代碰撞检查。
            if self.pose is not None and np.linalg.norm(samples[0] - self.pose) > 1.2:
                config = getattr(self, "profile", {}).get("diagnostic_observe_only", {})
                if (
                    isinstance(config, dict)
                    and type(config.get("enabled")) is bool
                    and config["enabled"]
                ):
                    if (
                        not np.isfinite(samples).all()
                        or not np.isfinite(self.pose).all()
                        or not math.isfinite(float(np.linalg.norm(samples[0] - self.pose)))
                        or not self._known_generation_ready()
                    ):
                        raise ValueError("nonfinite_or_unready_lower_execution_start_observation")
                    from embodied_navigation.common.observation import api as _observer_api

                    record = _observer_api("diagnostic_observation").record
                    if (
                        record(
                            self.run,
                            self.profile,
                            "scan_lower_execution_start_distance",
                            "execution_start_far_from_actual_pose",
                            dict(
                                distance_m=float(np.linalg.norm(samples[0] - self.pose)),
                                original_maximum_m=1.2,
                                trajectory_id=int(msg.traj_id),
                                navigation_PASS=False,
                            ),
                        )
                        is not True
                    ):
                        raise RuntimeError("lower_execution_start_observation_not_recorded")
                else:
                    # 非诊断模式：直接丢弃这条旧轨迹，保持现有轨迹与状态不变。
                    self.message = "忽略起点远离当前 SLAM 位姿的旧轨迹"
                    return
            # 样条存在但没有几何前进量：按原地停车处理，清空轨迹并等待新规划。
            if not trajectory_has_progress(
                samples, self.pose, self.waypoints[self.waypoint_index], self.arrival_radius
            ):
                self.degenerate_splines += 1
                self.last_spline_rejected = "SCAN 返回原地停车样条，目标尚未到达；等待有效规划"
                self.message = self.last_spline_rejected
                self.samples = None
                self.execution_path_exhaustion = None
                self.active_trajectory_id = None
                self.active_reference_stamp = None
                self.publish_command()
                self.path_pub.publish(self.make_path([]))
                return
            # 接受后立即发布路径话题；replans 计数同时用作归档序号与轨迹序号。
            self.samples = samples
            self.execution_path_exhaustion = None
            self.active_trajectory_id = int(msg.traj_id)
            self.active_reference_stamp = tuple(metadata["reference_stamp"])
            self.planning_start_state = metadata.get("start_state")
            self.last_spline_rejected = None
            self.replans += 1
            self.path_pub.publish(self.make_path(samples))
        except Exception as exc:
            self.message = f"拒绝无效 SCAN 轨迹: {exc}"

    def mark_execution_path_exhausted(self, reason, now, ros_now):
        """只在两条真实“路径耗尽且已清空”的分支上生成内部证据。

        定时器触发或命令归零本身不会把路径标记为耗尽；写入前先钉住 samples 与
        不可变的执行身份。该回执没有运动授权，也从不刷新任何源时间戳。
        """
        self.execution_path_exhaustion = None
        values = np.asarray(self.samples)
        reference = self.active_reference_stamp
        # 只接受两个白名单原因，且要求 samples/轨迹身份仍然完整，否则不生成证据。
        if (
            reason not in ("steering_exhausted", "low_command_exhausted_path")
            or values.ndim != 2
            or values.shape[1] != 3
            or not 2 <= len(values) <= 2000
            or not np.isfinite(values).all()
            or self.active_trajectory_id is None
            or reference is None
            or len(reference) != 2
        ):
            return
        self.execution_path_exhaustion = dict(
            schema="internal_checked_path_exhaustion/v1",
            reason=reason,
            request_id=self.request_id,
            waypoint_index=self.waypoint_index,
            goal_xyz=self.waypoints[self.waypoint_index].tolist(),
            active_trajectory_id=self.active_trajectory_id,
            active_reference_stamp=list(reference),
            previous_samples_sha256=hashlib.sha256(
                np.asarray(values, dtype="<f8").tobytes()
            ).hexdigest(),
            previous_samples_count=len(values),
            source_pose_stamp_ns=int(self.pose_stamp),
            control_stamp_ns=round(ros_now * 1e9),
            monotonic_wall=now,
            steering_exhausted=bool(self.steering["exhausted"]),
            cascade_mode=(getattr(self, "cascade_last_row", None) or {}).get("mode"),
            source="actual_shared_control_exhausted_clear_branch",
            navigation_ground_truth_used=False,
        )

    def make_path(self, points, height_offset=0.0):
        """构造 frame_id=camera_init 的 Path；朝向四元数固定 w=1，z 统一减去 height_offset。"""
        msg = Path()
        msg.header.frame_id = "camera_init"
        msg.header.stamp = self.get_clock().now().to_msg()
        for p in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, p)
            pose.pose.position.z -= height_offset
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def planner_reference_points(self, goal):
        # 子类可以提供“仅供规划”的引导点；执行阶段仍要求一条已关联、已检查、
        # 且终点为原目标的 SCAN 曲线。
        return [goal]

    def planner_goal(self):
        return self.waypoints[self.waypoint_index]

    def request_plan(self):
        """向 SCAN 请求一次参考轨迹；每个任务/航段可被多次调用。

        先检查 SCAN 订阅者是否就绪，再把（规划用）目标点作为参考路径发布。
        header stamp 是请求身份，必须严格单调（即使 ROS 时间暂停或同一 tick 内
        多次请求）。R85 自动参考路径优先走 ``automatic_reference``，其异常按 hold
        记录、零速停车并保留原状。
        """
        if getattr(self, "automatic_reference", None) is not None:
            try:
                return self.automatic_reference.request_plan(self)
            except (OSError, ValueError, KeyError, TypeError, ImportError, AttributeError) as exc:
                self.message = "R85 reference hold: " + type(exc).__name__ + ": " + str(exc)
                self.evidence.append(
                    self.run / "r85_reference_holds.jsonl",
                    dict(
                        reason=self.message,
                        source_pose_stamp_ns=int(self.pose_stamp),
                        control_clock_ns=self.get_clock().now().nanoseconds,
                        original_global_reference_republished=False,
                    ),
                )
                self.publish_command()
                return
        if self.reference_pub.get_subscription_count() == 0:
            self.message = "等待 SCAN 规划器订阅就绪"
            return
        goal = self.planner_goal()
        # SCAN REFERENCE_PATH 会给每个输入 z 加上 grid_map.body_height。公共任务
        # 航点本身就是机体质心，因此在这个边界抵消该内部约定（不改变请求坐标）。
        # 0.4 为本文件硬编码常量，代码与现有文档未注明其单位与来源。
        reference = self.make_path(self.planner_reference_points(goal), height_offset=0.4)
        # stamp 是规划器回传的不可变请求身份；即使 ROS 时间暂停或同一 tick 内
        # 多次请求，也要保证严格递增。
        stamp = max(self.last_reference_stamp + 1, self.stamp(reference))
        self.last_reference_stamp = stamp
        reference.header.stamp.sec, reference.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        self.trajectory_association.request(
            [reference.header.stamp.sec, reference.header.stamp.nanosec], goal
        )
        self.reference_pub.publish(reference)
        # last_reference 是单调墙钟；control() 用它判断 3 s 的重发节奏。
        self.last_reference = time.monotonic()
        self.reference_requests += 1
        self.message = "已请求 SCAN 三维占据地图规划"

    def publish_command(self, velocity=None, yaw_rate=0.0):
        """发布速度指令并更新冻结标志；velocity=None 表示显式发布零速。

        ``velocity`` 为 m/s（世界系 x/y），``yaw_rate`` 为 rad/s。非零目标先经
        control_core.limit_acceleration 按距上次发布的时间差限幅（其默认增量
        系数为线 0.15、偏航 0.25，dt 上限 0.10 s）。frozen=True 当且仅当任务不在
        running、处于障碍保持、或平面速度 < 0.015 m/s（硬编码门限）。
        """
        msg = Twist()
        now = self.get_clock().now().nanoseconds / 1e9
        if velocity is not None:
            desired = [float(velocity[0]), float(velocity[1]), float(yaw_rate)]
            limited = limit_acceleration(self.command, desired, now - self.last_command_time)
            msg.linear.x, msg.linear.y, msg.angular.z = map(float, limited)
        self.command = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.last_command_time = now
        self.cmd_pub.publish(msg)
        self.counts["commands"] += 1
        frozen = (
            self.state != "running"
            or self.obstacle_hold
            or np.linalg.norm(self.command[:2]) < 0.015
        )
        self.freeze_pub.publish(Bool(data=bool(frozen)))

    def tick(self):
        """20 Hz 控制入口：跑一次 control()；异常则失败停车，并按节流发布状态。"""
        now = time.monotonic()
        try:
            self.control(now)
        except Exception as exc:
            # 控制器异常必须停车并进入 failed，不能沿用上一周期的命令继续走。
            self.state, self.message = "failed", f"控制器异常，已停车: {exc}"
            self.publish_command()
            self.get_logger().error(self.message)
        # 固定窗口停车要求 300 ms 内覆盖实际状态。这里只调整发布节流，不改任何源
        # 时间戳：普通零命令保持（ordinary_zero_hold）时 0.1 s 发一次，否则 0.25 s。
        parking = getattr(self, "cascade_last_row", None) or {}
        status_period = 0.1 if parking.get("mode") == "ordinary_zero_hold" else 0.25
        if now - self.last_status >= status_period:
            self.publish_status()
            self.last_status = now

    def _diagnostic_business_observed(self, gate, reason, evidence):
        """诊断观测开关开启时记录业务门被绕过的事实；否则返回 False。"""
        profile = getattr(self, "profile", {})
        config = profile.get("diagnostic_observe_only", {})
        if not (
            isinstance(config, dict)
            and type(config.get("enabled")) is bool
            and config["enabled"]
            and getattr(self, "run", None) is not None
        ):
            # 未开启诊断或 run 未绑定：直接返回 False，不做任何记录。
            return False
        from embodied_navigation.common.observation import api as _observer_api

        record = _observer_api("diagnostic_observation").record
        record(self.run, profile, gate, reason, evidence)
        return True

    def interrupt_active_handoff(self, reason, clock_ns=None):
        """若有进行中的主动转向交接，则把它判为失败并进入 failed。

        返回 True 表示确实打断了一个活动交接，调用方应只发零速并结束本周期。
        """
        gate = getattr(self, "heading_gate", None)
        if not getattr(gate, "handoff_active", False):
            return False
        clock_ns = self.get_clock().now().nanoseconds if clock_ns is None else clock_ns
        gate.fail_handoff(reason, clock_ns / 1e9)
        self.state = "failed"
        self.message = "主动转向交接失败：" + str(gate.handoff_failure)
        return True

    def raw_handoff_deadline(self):
        """返回当前航段的绝对截止时间（ROS 秒）与绑定信息；无截止时返回 None。

        优先取 continuous_route 账本下一段的激活时间 + timeout；否则用本段起点
        ROS 时间 + 目标 timeout。legacy 目标使用硬编码兜底公式
        max(90 s, 距离/0.06 + 30 s)，新 schema 目标使用自己的 timeout_sim_s。
        """
        # 直接读取原始绝对截止时间。诊断观测可以绕过业务门，但绕不过这个活动
        # 事件预算。
        ledger = getattr(self, "continuous_route", None)
        binding = dict(request_id=self.request_id, waypoint_index=self.waypoint_index)
        if ledger is not None:
            if ledger.next_index >= len(ledger.goals) or ledger.next_activated_ns is None:
                return None, binding
            activation = int(ledger.next_activated_ns)
            timeout = round(float(ledger.goals[ledger.next_index].timeout_sim_s) * 1e9)
            binding.update(
                deadline_source="continuous_route_original",
                coverage_index=ledger.next_index,
                activated_ns=activation,
                timeout_ns=timeout,
            )
            return (activation + timeout) / 1e9, binding
        if self.segment_start is None:
            return None, binding
        goal = self.goals[self.waypoint_index]
        # 0.06 m/s、30 s、90 s 是 legacy 路径的硬编码兜底参数；代码与现有文档未
        # 说明其标定来源。
        activation = float(self.segment_started_ros)
        timeout = (
            max(
                90.0,
                float(np.linalg.norm(self.waypoints[self.waypoint_index] - self.segment_start))
                / 0.06
                + 30.0,
            )
            if goal.legacy
            else float(goal.timeout_sim_s)
        )
        binding.update(
            deadline_source="legacy_segment_original" if goal.legacy else "region_original",
            activated_ns=round(activation * 1e9),
            timeout_ns=round(timeout * 1e9),
        )
        return activation + timeout, binding

    def segment_deadline_exceeded(self, clock_ns):
        """判断当前航段是否已超过仿真时间截止；诊断开关可把它降级为仅观测。"""
        ledger = getattr(self, "continuous_route", None)
        exceeded = (
            ledger.deadline_exceeded(clock_ns)
            if ledger is not None
            else clock_ns / 1e9 - self.segment_started_ros
            > self.goals[self.waypoint_index].timeout_sim_s
        )
        # 诊断模式下记录“本应超时”的事实并返回 False，即不真正判失败。
        if exceeded and self._diagnostic_business_observed(
            "navigation_waypoint_business_deadline",
            "original_waypoint_timeout_would_fail",
            dict(
                control_stamp_ns=clock_ns,
                segment_started_ros=self.segment_started_ros,
                waypoint_index=self.waypoint_index,
                navigation_PASS=False,
                resource_deadlines_unchanged=True,
            ),
        ):
            return False
        return exceeded

    def control(self, now):
        """单个控制周期的任务状态机：输入门 → 到点判定 → 转向/障碍 → 发布命令。

        各停车分支彼此独立且顺序固定：非 running 零速；航段截止超时 failed；
        倾角/执行桥保护；输入新鲜度（连续失联 8 s 升级 failed）；到点判定
        （legacy 用 0.4 s 计时，新 schema 用 ArrivalWindow）；无轨迹时每 3 s 重发
        规划请求；跟随轨迹后由 heading 相位与障碍 guard 决定是否允许平移。

        ``now`` 为 monotonic 墙钟秒；ROS 仿真时钟在函数内自取。所有停车分支都在
        返回前发布零速（或保持零速），并同步刷新状态与消息。
        """
        if self.state != "running":
            # 非 running 状态每个周期仍发布零速，避免下游沿用旧速度。
            self.publish_command()
            return
        control_clock_ns = self.get_clock().now().nanoseconds
        ros_now = control_clock_ns / 1e9
        # 已声明的区域截止时间同样约束本段真正开始后的到点停顿重置与受保护等待；
        # legacy 目标的计时/到点顺序在下面保持原样。
        if (
            self.segment_start is not None
            and not self.goals[self.waypoint_index].legacy
            and self.segment_deadline_exceeded(control_clock_ns)
        ):
            self.reset_region_arrival("failed")
            self.state, self.message = "failed", "当前航点超过仿真时间限制，停车"
            self.publish_command()
            return
        protected = self.apply_tilt_guard(now, ros_now)
        if protected:
            # 保护期不排队恢复，避免故障持续期间反复触发障碍恢复流程。
            self.obstacle_resume_pending = False
        # apply_tilt_guard 可能已把状态改为 failed：本周期只发零速并结束。
        if self.state == "failed":
            self.reset_region_arrival("failed")
            self.publish_command()
            return
        stale = control_sources_stale(
            self,
            now,
            ros_now,
            (
                now - self.pose_updated > self.pose_timeout
                or now - self.cloud_updated > self.cloud_timeout
                or not self.raw_imu.fresh(now, ros_now)
            ),
        )
        # 输入门：pose/cloud 超过各自超时，或原始 IMU 不新鲜 → 停车；持续 8 s
        # 才升级为 failed（8.0 为本文件硬编码）。
        if stale:
            self.reset_region_arrival("stale")
            self.publish_command()
            self.stale_since = self.stale_since or now
            self.message = "SLAM、点云或原始 IMU 超时，停车等待传感器"
            if now - self.stale_since > 8.0:
                self.state, self.message = "failed", "SLAM、点云或原始 IMU 连续失联超过 8 秒"
            return
        # 任一输入恢复即重置失联计时。
        self.stale_since = None
        if protected:
            self.reset_region_arrival("protected")
            self.publish_command()
            return
        # continuous_route 子类钩子：本段开始前先执行连续路线逻辑。
        if hasattr(self, "continuous_before_region") and self.continuous_before_region(
            now, control_clock_ns
        ):
            if self.interrupt_active_handoff(
                "active_handoff_continuous_region_transition", control_clock_ns
            ):
                self.publish_command()
                return
            self.publish_command()
            self.request_plan()
            return
        goal = self.waypoints[self.waypoint_index]
        # 首次进入本航段时记录起点位姿与 ROS 时间，作为截止时间与超时公式的基准。
        if self.segment_start is None:
            self.segment_start = self.pose.copy()
            self.segment_started_ros = ros_now
            self.segment_started_ros_ns = control_clock_ns
            self.segment_start_pose_stamp_ns = int(self.pose_stamp)
        distance = float(np.linalg.norm(goal - self.pose))
        # legacy 目标：在 arrival_radius（默认 0.22 m）内持续 0.4 s ROS 时间计为
        # 到达；新 schema 由 ArrivalWindow 用实测停顿确认。
        if self.goals[self.waypoint_index].legacy:
            inside = distance < self.arrival_radius
            if inside:
                self.arrival_since = self.arrival_since or ros_now
            arrived = inside and ros_now - self.arrival_since >= 0.4
        else:
            inside, arrived = self.measured_region_arrival()
        if inside:
            if self.interrupt_active_handoff("active_handoff_region_arrival", control_clock_ns):
                self.publish_command()
                return
            self.publish_command()
            # 到达后推进到下一航点，并清空轨迹/关联/相位/保持状态；下一段必须重新
            # 获取一条已关联的 SCAN 轨迹。
            if arrived:
                if self.region_arrival is not None:
                    if hasattr(self, "continuous_record_hard"):
                        self.continuous_record_hard(self.region_arrival_evidence, control_clock_ns)
                    self.region_arrivals.append(copy.deepcopy(self.region_arrival_evidence))
                self.waypoint_index += 1
                self.samples = None
                self.execution_path_exhaustion = None
                self.trajectory_association.reset()
                self.planning_start_state = None
                self.steering = None
                self.active_trajectory_id = None
                self.active_reference_stamp = None
                self.obstacle_resume_pending = False
                self.arrival_since = None
                self.segment_start = None
                self.alignment_hold = True
                reset_transition(self, "region_transition")
                self.last_reference = -math.inf
                self.region_arrival = (
                    ArrivalWindow(self.goals[self.waypoint_index])
                    if self.waypoint_index < len(self.goals)
                    and not self.goals[self.waypoint_index].legacy
                    else None
                )
                if self.region_arrival is not None:
                    # 上一目标的最后一条 odom 不能当作下一段停顿窗的起点。
                    self.region_arrival.last_stamp = int(self.pose_stamp)
                if self.waypoint_index < len(self.goals):
                    self.region_arrival_evidence = None
                # 最后一个航点确认到达即 succeeded，之后不再下发非零速度。
                if self.waypoint_index == len(self.waypoints):
                    self.state, self.message = "succeeded", "所有航点均由实际 SLAM 位置确认到达"
                else:
                    self.message = "当前航点已到达，准备下一段"
            return
        self.arrival_since = None
        # legacy 目标没有声明 timeout，改用与 raw_handoff_deadline 相同的硬编码公式。
        max_segment_time = max(90.0, np.linalg.norm(goal - self.segment_start) / 0.06 + 30.0)
        if (
            self.goals[self.waypoint_index].legacy
            and ros_now - self.segment_started_ros > max_segment_time
            and not self._diagnostic_business_observed(
                "navigation_legacy_waypoint_business_deadline",
                "original_legacy_waypoint_timeout_would_fail",
                dict(
                    control_stamp_ns=control_clock_ns,
                    segment_started_ros=self.segment_started_ros,
                    original_max_segment_time_s=float(max_segment_time),
                    waypoint_index=self.waypoint_index,
                    navigation_PASS=False,
                    resource_deadlines_unchanged=True,
                ),
            )
        ):
            self.reset_region_arrival("failed")
            self.state, self.message = "failed", "当前航点超过仿真时间限制，停车"
            self.publish_command()
            return
        # 无轨迹：保持零速；若刚拒绝过样条则把原因透出，并每 3 s 重发一次规划请求。
        if self.samples is None:
            if self.interrupt_active_handoff(
                "active_handoff_execution_path_missing", control_clock_ns
            ):
                self.publish_command()
                return
            self.publish_command()
            if self.last_spline_rejected is not None:
                self.message = self.last_spline_rejected
            if now - self.last_reference > 3.0:
                self.request_plan()
            return
        # 跟随已检查轨迹只产生参考速度/角速度；是否允许平移由下面的 heading gate
        # 与障碍 guard 决定。
        velocity, yaw_rate, target, self.steering = self.follow_checked_trajectory(
            self.pose,
            self.rotation,
            self.samples,
            goal,
            max_speed=self.max_speed,
            tracking_pose=self.tracking_pose,
            gate_translation=False,
            return_steering=True,
        )
        current_yaw = math.atan2(self.rotation[1, 0], self.rotation[0, 0])
        self.steering.update(
            stamp=ros_now, odom_stamp=self.pose_stamp / 1e9, trajectory_id=self.active_trajectory_id
        )
        desired_yaw = self.steering["heading"]
        # 新 schema 目标不能停在 obstacle_hold 期间：即使已检查路径终结在到达窗内，
        # 也要先重查真实点云再返回，这样清除的障碍可以走完原有 1 s 门。保持期间命令
        # 仍为精确零速；清除后丢弃这条路径，重新取一条已检查的 SCAN 才恢复运动。
        if self.steering["exhausted"] and not (
            self.obstacle_hold and not self.goals[self.waypoint_index].legacy
        ):
            if self.interrupt_active_handoff(
                "active_handoff_execution_path_exhausted", control_clock_ns
            ):
                self.publish_command()
                return
            self.publish_command()
            self.message = "实际位置尚未到达，SCAN 已检查的路径结束；等待重新规划"
            if now - self.last_reference > 3.0:
                self.mark_execution_path_exhausted("steering_exhausted", now, ros_now)
                self.samples = None
                self.request_plan()
            return
        # Teacher 恢复交接（known_scene_hold_guard）优先于普通障碍恢复。
        teacher_recovery = start_teacher_recovery(self, current_yaw, desired_yaw)
        # 障碍清除后的恢复必须先经过一次完整实测停车；resume_after_stop 成功才
        # 重新进入 drive 相位。无论成功与否，待恢复标记只消费一次。
        if self.obstacle_resume_pending and not teacher_recovery:
            if self.heading_gate.resume_after_stop(
                current_yaw, desired_yaw, self.obstacle_stopped_duration
            ):
                self.aligned_obstacle_resumes += 1
            self.obstacle_resume_pending = False
        if getattr(self, "reference_consistency_hold", False):
            # 方向不一致时不允许用弦线替代；转向状态也一并保持，并重新获取一条
            # 已检查的 SCAN 曲线。
            allow_translation, yaw_rate = False, 0.0
        else:
            # 绑定本次转向交接的上下文；启用主动制动时把原始截止时间绑给 gate。
            bind_transition_context(self)
            if getattr(self.heading_gate, "active_braking_enabled", False):
                deadline, binding = self.raw_handoff_deadline()
                if deadline is not None:
                    self.heading_gate.bind_handoff_deadline(deadline, binding)
                elif self.interrupt_active_handoff(
                    "active_handoff_original_deadline_missing", control_clock_ns
                ):
                    self.publish_command()
                    return
            self.heading_gate.diagnostic_run = getattr(self, "run", None)
            self.heading_gate.diagnostic_zero_ack_ready = False
            diagnostic = getattr(self, "profile", {}).get("diagnostic_observe_only", {})
            if (
                isinstance(diagnostic, dict)
                and type(diagnostic.get("enabled")) is bool
                and diagnostic["enabled"]
                and hasattr(self, "get_ack")
            ):
                try:
                    ack = self.get_ack()
                    from embodied_navigation.robot.unitree.policy.lower_loop_ack import (
                        input_binding,
                    )

                    # 保留真实的 Student/输入绑定，供 gate 的零速 ACK 判据使用。
                    input_binding(ack)
                    values = ack["requested_command_body"] + ack["applied_command_body"]
                    self.heading_gate.diagnostic_zero_ack_ready = bool(
                        len(values) == 6
                        and all(
                            type(v) in (int, float) and math.isfinite(v) and v == 0.0
                            for v in values
                        )
                        and type(ack["stamp_ns"]) is int
                        and 0 <= control_clock_ns - ack["stamp_ns"] < 300_000_000
                        and ack["stamp_ns"] <= self.pose_stamp
                        and 0 <= now - ack["received_wall_ns"] / 1e9 < 0.3
                    )
                except (ValueError, TypeError, KeyError, AttributeError):
                    pass
            # 返回 (allow_translation, yaw_rate)：False 时本周期速度清零，但仍可转向。
            allow_translation, yaw_rate = self.heading_gate.update(
                ros_now, current_yaw, desired_yaw
            )
        # 交接失败是终态 failed：必须零速并停止后续周期。
        if getattr(self.heading_gate, "handoff_failure", None):
            self.state = "failed"
            self.message = "主动转向交接失败：" + str(self.heading_gate.handoff_failure)
            self.publish_command()
            return
        # alignment_hold 反映当前是否禁止平移，会写入状态话题。
        self.alignment_hold = not allow_translation
        if not allow_translation:
            velocity = np.zeros(2)
        # 子类钩子：把参考速度/角速度与“是否允许平移”合成 PID 输出（基类不实现）。
        velocity, yaw_rate = self.select_pid_velocity(velocity, yaw_rate, target, allow_translation)
        if getattr(self.heading_gate, "handoff_failure", None):
            self.state = "failed"
            self.message = "主动转向交接失败：" + str(self.heading_gate.handoff_failure)
            self.publish_command()
            return
        actual_guard_evaluated = False
        # guard 评估周期：障碍保持时 0.05 s（20 Hz），否则 0.10 s（10 Hz）；
        # pid_guard_needs_evaluation 可要求本周期立即评估。
        guard_period = 0.05 if self.obstacle_hold else 0.10
        if now - self.last_obstacle_check >= guard_period or getattr(
            self, "pid_guard_needs_evaluation", False
        ):
            route = (
                self.execution_guard_route(self.segment_start, goal)
                if hasattr(self, "execution_guard_route")
                else np.vstack([self.segment_start, goal])
            )
            self.obstacle_result = self.evaluate_motion_guard(
                self.cloud, self.pose, target, self.steering["direction"], route
            )
            self.last_obstacle_check = now
            actual_guard_evaluated = True
            if self.obstacle_result[0] and not self.obstacle_hold:
                # 在计算时刻冻结输入（包括缓存结果对应的点云），而不是等后续
                # tick 再读 self.cloud。冻结失败只记录错误，不改变停车结论。
                try:
                    if self.cloud_input_context is None:
                        raise ValueError("native event lacks matching cloud input context")
                    self.obstacle_guard_context = (
                        dict(
                            cloud=self.cloud,
                            pose=self.pose.copy(),
                            rotation=self.rotation.copy(),
                            tracking_pose=self.tracking_pose.copy(),
                            checked_target=target.copy(),
                            steering_direction=np.asarray(self.steering["direction"]).copy(),
                            route=route.copy(),
                            segment_start=self.segment_start.copy(),
                            goal=goal.copy(),
                        ),
                        dict(
                            guard_compute_stamp=ros_now,
                            odom_stamp_ns=self.pose_stamp,
                            cloud_input=self.cloud_input_context,
                            guard_result=self.obstacle_result,
                            request_id=self.request_id,
                            waypoint_index=self.waypoint_index,
                            trajectory_id=self.active_trajectory_id,
                            reference_stamp=self.active_reference_stamp,
                            raw_imu_stamp=self.raw_imu.stamp,
                            raw_imu_tilt=self.raw_imu.tilt,
                            protect_result=protected,
                            tilt_hold=self.tilt_hold,
                            tilt_source=self.tilt_source,
                            execution_bridge_safety=copy.deepcopy(self.bridge_safety),
                        ),
                    )
                    self.obstacle_event_capture_error = None
                except Exception as exc:
                    self.obstacle_guard_context = None
                    self.obstacle_event_capture_error = f"{type(exc).__name__}: {exc}"
        blocked, clearance, point_count = self.obstacle_result
        clear_observations_ready = actual_guard_evaluated and getattr(
            self, "_actual_clear_guard_ready", False
        )
        if blocked:
            # 首次阻断时开始计时并捕获事件证据；连续阻断只更新最小间距。
            if not self.obstacle_hold:
                self.obstacle_stops += 1
                self.obstacle_hold_started_ros = ros_now
                self.pending_obstacle_event = self.obstacle_guard_context
            self.obstacle_hold = True
            self.obstacle_clear_since = None
            if self.min_obstacle_clearance is None or clearance < self.min_obstacle_clearance:
                self.min_obstacle_clearance = clearance
        elif self.obstacle_hold:
            # 清除要同时满足：持续 1.0 s ROS 时间无阻断，且本次控制周期真实评估过
            # guard（_actual_clear_guard_ready 由 Teacher 子类在 record_native_guard
            # 中设置）。满足后先停一次、丢弃旧路径并请求新轨迹再恢复。
            self.obstacle_clear_since = self.obstacle_clear_since or ros_now
            if ros_now - self.obstacle_clear_since >= 1.0 and clear_observations_ready:
                arm_teacher_recovery(self)
                self.obstacle_hold = False
                self.obstacle_resumes += 1
                self.obstacle_stopped_duration = ros_now - (
                    ros_now
                    if self.obstacle_hold_started_ros is None
                    else self.obstacle_hold_started_ros
                )
                self.obstacle_resume_pending = True
                reset_transition(self, "dynamic_recovery")
                self.samples = None
                self.execution_path_exhaustion = None
                self.last_reference = -math.inf
        if self.obstacle_hold:
            self.publish_command()
            # 先发零速再入队：压缩、JSON 转换与文件 IO 都不在这个安全/控制回调里执行。
            if self.pending_obstacle_event is not None:
                event, metadata = self.pending_obstacle_event
                self.pending_obstacle_event = None
                try:
                    metadata = dict(
                        metadata,
                        obstacle_edge_stamp=ros_now,
                        edge_request_id=self.request_id,
                        edge_waypoint_index=self.waypoint_index,
                        zero_command_stamp=self.last_command_time,
                        zero_command=self.command.copy(),
                    )
                    self.last_obstacle_event_id = self.event_archive.enqueue(event, metadata)
                except Exception as exc:
                    self.get_logger().warning(f"障碍证据记录失败，停车保护保持: {exc}")
            self.message = f"雷达检测路径障碍，停车等待；有效点 {point_count}"
            return
        if self.samples is None:
            if self.interrupt_active_handoff(
                "active_handoff_execution_path_lost", control_clock_ns
            ):
                self.publish_command()
                return
            self.publish_command()
            self.request_plan()
            return
        if getattr(self, "reference_consistency_hold", False):
            self.message = "SCAN 近端方向不一致，零速等待新的可执行参考"
            self.publish_command()
            return
        # pre_turn（转向开始前）与 settle（转向完成后）都要求零速停稳 1 s；
        # controlled_settle / controlled_braking 是 Teacher 的受控例外。
        if (
            self.heading_gate.phase == "pre_turn"
            or self.heading_gate.phase == "settle"
            and not (
                getattr(self.heading_gate, "controlled_settle", False)
                or getattr(self.heading_gate, "controlled_braking", False)
            )
        ):
            self.message = (
                "转向前制动，保持零速停稳 1 秒"
                if self.heading_gate.phase == "pre_turn"
                else "转向完成，保持零速停稳 1 秒"
            )
            self.publish_command()  # Exact zero, as in the independently tested physical turn/drive sequence.
            return
        # 若上游轨迹在实测到达前结束，就从当前实测位姿重新请求轨迹，而不是按时间
        # 判成功。这里复用 steering 的 exhausted 与低命令判定，避免把“已停住”误当到达。
        if (
            not self.alignment_hold
            and np.linalg.norm(velocity) < 0.015
            and abs(yaw_rate) < 0.05
            and now - self.last_reference > 3.0
            and low_command_is_exhausted_path(
                cascade_mode=getattr(self, "cascade_last_row", {}).get("mode"),
                exhausted=self.steering["exhausted"],
                has_geometric_progress=trajectory_has_progress(
                    self.samples, self.pose, goal, self.arrival_radius
                ),
            )
        ):
            self.mark_execution_path_exhausted("low_command_exhausted_path", now, ros_now)
            self.samples = None
            if self.interrupt_active_handoff(
                "active_handoff_execution_path_exhausted", control_clock_ns
            ):
                self.publish_command()
                return
            self.publish_command()
            self.request_plan()
            return
        self.message = (
            "转向零角速度参考主动制动，尚未交接原零速停稳验收"
            if getattr(self.heading_gate, "controlled_braking", False)
            else "转向闭环收敛至同一锁定航向中心，再交接原零速停稳验收"
            if getattr(self.heading_gate, "controlled_settle", False)
            else f"实际 SLAM 反馈跟随 SCAN 路线，剩余 {distance:.2f} m"
        )
        self.publish_command(velocity, yaw_rate)
        # 正常下发速度后才结束 Teacher 恢复交接。
        complete_teacher_recovery(self)

    @_callback_cost_observe("publish_status")
    def publish_status(self):
        """把完整运行状态序列化为 JSON 发布到锁存的状态话题。

        字段覆盖任务/航点、到点证据、轨迹关联、障碍与倾角保护、传感器年龄、
        命令与计数。frame_id 固定为 camera_init；feedback_source、planner 与
        avoidance 说明反馈来源与避障策略。周期发布由 tick() 节流控制（不是每个
        控制周期都发），状态回调里也会补发。Teacher 子类会把发布者换成
        RecordedPublisher，额外写 JSONL 与原子快照。
        """
        now = time.monotonic()
        # age() 把 monotonic 回执时间转成秒龄；从未收到过的源保持 None。
        age = lambda updated: None if not math.isfinite(updated) else round(now - updated, 3)
        status = dict(
            execution_path_exhaustion=copy.deepcopy(self.execution_path_exhaustion),
            request_id=self.request_id,
            state=self.state,
            goals_definition_sha256=self.goals_definition_sha256,
            goal_activated_ros_clock_ns=getattr(self, "segment_started_ros_ns", None),
            segment_start_pose_stamp_ns=getattr(self, "segment_start_pose_stamp_ns", None),
            goals_definitions=[g.definition() for g in self.goals],
            current_goal=None
            if self.waypoint_index >= len(self.goals)
            else self.goals[self.waypoint_index].definition(),
            region_arrival_evidence=self.region_arrival_evidence,
            region_arrivals=self.region_arrivals,
            continuous_route=(
                self.continuous_route.status()
                if getattr(self, "continuous_route", None) is not None
                and self.continuous_route.request_id == self.request_id
                else None
            ),
            waypoint_index=self.waypoint_index,
            total=len(self.waypoints),
            message=self.message,
            obstacle_hold=self.obstacle_hold,
            obstacle_stops=self.obstacle_stops,
            obstacle_resumes=self.obstacle_resumes,
            min_obstacle_clearance=self.min_obstacle_clearance,
            replans=self.replans,
            reference_requests=self.reference_requests,
            degenerate_splines=self.degenerate_splines,
            last_spline_rejected=self.last_spline_rejected,
            trajectory_reference_stamp=self.active_reference_stamp,
            pending_trajectory_reference_stamp=self.trajectory_association.reference_stamp,
            trajectory_association_rejected=self.trajectory_association.rejected,
            last_trajectory_association_rejected=self.trajectory_association.last_rejected,
            planning_start_state=self.planning_start_state,
            steering=self.steering,
            accepted_trajectory_id=self.active_trajectory_id,
            obstacle_resume_pending=self.obstacle_resume_pending,
            aligned_obstacle_resumes=self.aligned_obstacle_resumes,
            pose_age=age(self.pose_updated),
            cloud_age=age(self.cloud_updated),
            actual_cloud_message_stamp_ns=None
            if self.cloud_input_context is None
            else self.cloud_input_context["message_stamp_ns"],
            obstacle_guard_cloud_stamp_ns=None
            if self.obstacle_guard_context is None
            else self.obstacle_guard_context[1]["cloud_input"]["message_stamp_ns"],
            last_obstacle_event_id=self.last_obstacle_event_id,
            obstacle_event_capture_error=self.obstacle_event_capture_error,
            obstacle_event_archive=self.event_archive.status(request_id=self.request_id),
            pose=None if self.pose is None else self.pose.tolist(),
            command=self.command,
            counts=self.counts,
            last_rejected=self.last_rejected,
            self_filtered_points=self.self_filtered_points,
            tilt_hold=self.tilt_hold,
            tilt_stops=self.tilt_stops,
            max_tilt_rad=self.max_tilt,
            tilt_source=self.tilt_source,
            raw_imu_age=age(self.raw_imu.updated),
            raw_imu_stamp_age=None
            if not math.isfinite(self.raw_imu.stamp)
            else round(self.get_clock().now().nanoseconds / 1e9 - self.raw_imu.stamp, 3),
            raw_imu_tilt_rad=self.raw_imu.tilt,
            raw_imu_max_tilt_rad=self.raw_imu.max_tilt,
            raw_imu_last_rejected=self.raw_imu.last_rejected,
            raw_imu_fixed_body_rotation=self.imu_body_rotation.tolist(),
            execution_bridge_safety=self.bridge_safety,
            alignment_hold=self.alignment_hold,
            filtered_yaw_rate=self.yaw_rate,
            alignment_phase=self.heading_gate.phase,
            locked_heading=self.heading_gate.heading,
            tracking_pose=None if self.tracking_pose is None else self.tracking_pose.tolist(),
            heading_deviation_since=self.heading_gate.outside_since,
            motion_limits=dict(
                walk_yaw_rate_rad_s=control_core.MAX_WALK_YAW_RATE,
                pure_turn_rate_rad_s=control_core.MAX_TURN_RATE,
                forward_speed_m_s=self.max_speed,
            ),
            frame_id="camera_init",
            feedback_source="/demo/slam/body_odom",
            planner="SCAN with measured-pose trajectory execution",
            avoidance="registered-LiDAR stop, wait for clearance, replan and resume",
        )
        if hasattr(self, "_r74_clock_setup_receipt"):
            status["r74_independent_ros_clock"] = copy.deepcopy(self._r74_clock_setup_receipt)
        if minimal_baseline_enabled(getattr(self, "profile", {})):
            status["r73_upper_wall_last_observations"] = copy.deepcopy(
                getattr(self, "_r73_upper_wall_last_observations", {})
            )
        if _r85_intake_enabled(getattr(self, "profile", None)):
            status["r85_protocol_intake_trace"] = _r85_intake_summary(self)
        # ensure_ascii=False 保留中文 message 原文；话题为 TRANSIENT_LOCAL，迟到的
        # 订阅者也能收到最后一份完整快照。
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main():
    """独立运行入口：初始化 rclpy、自旋，并在退出前补发零速、关闭事件归档。"""
    rclpy.init()
    node = Navigation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except _ros.RCLError:
        # SIGINT 可能在 executor 的 wait-set 调用之间让 context 失效；context 仍
        # 有效时的错误必须让这个必需节点失败退出。
        if rclpy.ok():
            raise
    finally:
        try:
            if rclpy.ok():
                node.publish_command()
        finally:
            node.event_archive.close()
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
