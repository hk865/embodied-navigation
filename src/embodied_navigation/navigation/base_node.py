#!/usr/bin/env python3
"""Measured-pose SCAN execution with freshness watchdog and LiDAR stop/resume.

The node never sets a simulator pose, never subscribes to ground truth and never
drives directly to an unchecked waypoint when SCAN has not produced a trajectory.
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
    def destroy_node(self):
        try:
            _r74_protocol_observation.flush_once(self)
        finally:
            result = super().destroy_node()
        return result

    def create_subscription(self, msg_type, topic, callback, qos_profile, *args, **kwargs):
        from embodied_navigation.common.time.clock_groups import subscription_options

        options = subscription_options(self, topic, kwargs)
        return super().create_subscription(msg_type, topic, callback, qos_profile, *args, **options)

    def subscribe_slam_body(self, msg_type, topic, callback, qos_profile, *args, **kwargs):
        """Explicit body-input port; default subscription semantics are unchanged."""
        return self.create_subscription(msg_type, topic, callback, qos_profile, *args, **kwargs)

    def subscribe_registered_cloud(self, msg_type, topic, callback, qos_profile, *args, **kwargs):
        """Explicit registered-cloud port; preserves QoS and callback ownership."""
        return self.create_subscription(msg_type, topic, callback, qos_profile, *args, **kwargs)

    def __init__(self):
        super().__init__("demo_navigation", parameter_overrides=[])
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.max_speed = self.declare_parameter("max_speed", 0.12).value
        self.arrival_radius = self.declare_parameter("arrival_radius", 0.22).value
        self.pose_timeout = self.declare_parameter("pose_timeout", 0.8).value
        self.cloud_timeout = self.declare_parameter("cloud_timeout", 1.0).value
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
        self.create_subscription(String, "/demo/navigation/request", self.on_request, 10)
        self.create_subscription(Bool, "/demo/navigation/stop", self.on_stop, 10)
        self.subscribe_slam_body(
            Odometry, "/demo/slam/body_odom", self.on_odom, qos_profile_sensor_data
        )
        # Process only the newest raw sample; SLAM/SCAN remains the route feedback.
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
            # Poll on steady time, but enter unchanged control only when the
            # actual ROS clock advances beyond both prior control and guard.
            # Short repeats leave the original wall watchdog and receipts alone.
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

            self.create_timer(
                0.05, r85_steady_control_tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
            )
            self.create_timer(
                0.05, r85_wall_watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME)
            )
        else:
            self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def stamp(msg):
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
                # Exact zero immediately in this callback, before the 20 Hz tick.
                self.publish_command()
            if self.state == "failed":
                self.publish_status()

    def on_bridge_safety(self, msg):
        try:
            safety = json.loads(msg.data)
            if safety.get("state") not in ("hold", "ready", "failed"):
                return
        except (ValueError, AttributeError):
            return
        self.bridge_safety = safety
        if self.state == "running" and self.apply_tilt_guard(
            time.monotonic(), self.get_clock().now().nanoseconds / 1e9
        ):
            self.reset_region_arrival("protected")
            self.publish_command()
            self.publish_status()

    def apply_tilt_guard(self, now, ros_now):
        slam_tilt = body_tilt(self.rotation)
        raw_tilt = self.raw_imu.tilt
        self.max_tilt = max(self.max_tilt, slam_tilt, raw_tilt or 0.0)
        fresh = raw_imu_fresh(self, now, ros_now, self.raw_imu.fresh(now, ros_now))
        sources = [("SLAM", slam_tilt)]
        if fresh:
            sources.append(("raw_imu", raw_tilt))
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
        if not fresh:
            self.tilt_source = "raw_imu_stale"
            self.tilt_clear_since = None
            self.message = "原始 IMU 缺失或时间戳超时，停车等待传感器"
            return True
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
        self.yaw_rate = 0.75 * self.yaw_rate + 0.25 * float(msg.twist.twist.angular.z)
        self.pose_history.append((stamp, pose.copy(), rotation.copy()))
        self.counts["odom"] += 1
        # SCAN upstream consumes twist as world velocity; ROS Odometry specifies
        # twist in child_frame_id. Make the nonstandard SCAN input explicit here.
        scan = copy.deepcopy(msg)
        v = msg.twist.twist.linear
        world_velocity = rotation @ np.array([v.x, v.y, v.z])
        scan.twist.twist.linear.x, scan.twist.twist.linear.y, scan.twist.twist.linear.z = map(
            float, world_velocity
        )
        scan.child_frame_id = "demo_scan_body_world_twist"
        self.scan_odom_pub.publish(scan)

    def on_cloud(self, msg):
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
        # Own immutable per-message buffer; later callbacks only replace it.
        # Retaining this exact buffer avoids recording a later scan for a cached
        # guard result. No cropping/voxelization changes native event evidence.
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
        try:
            request_id, goals = parse_goal_request(json.loads(msg.data))
            # Existing custom legacy arrival_radius still governs old requests.
            goals = tuple(replace(g, radius=self.arrival_radius) if g.legacy else g for g in goals)
            goal_hash = definitions_sha256(goals)
        except (ValueError, TypeError) as exc:
            self.last_rejected = str(exc)
            self.publish_status()
            return
        if request_id == self.request_id:
            if goal_hash != self.goals_definition_sha256:
                self.last_rejected = f"{request_id}: 同一任务 ID 的目标定义哈希不同，拒绝变更"
            self.publish_status()  # Idempotent retransmission.
            return
        if self.state == "running":
            self.last_rejected = f"{request_id}: 已有任务正在运行"
            self.publish_status()
            return
        self.request_id, self.goals = request_id, goals
        self.waypoints = np.array([g.center for g in goals], dtype=float)
        self.goals_definition_sha256 = goal_hash
        self.region_arrivals = []
        self.region_arrival_evidence = None
        self.region_arrival = None if goals[0].legacy else ArrivalWindow(goals[0])
        self.waypoint_index = 0
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
        if msg.data:
            self.reset_region_arrival("stopped")
            self.state, self.message = "stopped", "任务已停止"
            self.samples = None
            self.execution_path_exhaustion = None
            self.obstacle_resume_pending = False
            self.publish_command()
            self.publish_status()

    def reset_region_arrival(self, reason):
        if self.region_arrival is not None:
            self.region_arrival.reset(reason)
            if self.region_arrival_evidence is not None:
                self.region_arrival_evidence["reason"] = reason
                self.region_arrival_evidence["start_stamp_ns"] = None
                self.region_arrival_evidence["dwell_ns"] = 0

    def measured_region_arrival(self):
        """New schema only. Position and integer stamp are one accepted odom."""
        g = self.goals[self.waypoint_index]
        pose = self.region_raw_pose
        if pose is None:
            self.reset_region_arrival("no_measured_pose")
            return False, False
        protected = (
            self.obstacle_hold or self.tilt_hold or self.bridge_safety.get("state") != "ready"
        )
        inside = goal_contains(g, pose)
        control_inside = goal_contains_control(g, pose)
        arrived = self.region_arrival.observe(pose, self.pose_stamp, protected=bool(protected))
        since = self.region_arrival.since
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
        _r85_intake_context = _r85_intake_begin(self, "bspline", msg)
        _r74_protocol_context = _r74_protocol_observation.begin(
            self, "bspline", msg, _r85_intake_context
        )
        self.counts["bspline"] += 1
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
        """Low-frequency definition boundary; PID wrapper owns R88 dispatch."""
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
        """One existing pending pair; original PID/R88 owner gates the call."""
        if self.state != "running" or not hasattr(self.trajectory_association, "match_pending"):
            return False
        matched = self.trajectory_association.match_pending()
        if matched is None:
            return False
        self.accept_spline(*matched)
        return True

    @_callback_cost_observe("reference_attempt")
    def on_reference_attempt(self, msg):
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
        msg = UInt64MultiArray()
        msg.layout.dim = [MultiArrayDimension(label=self.run.name, size=5, stride=5)]
        msg.layout.data_offset = 0
        msg.data = [int(epoch), int(revision), int(sequence), int(stamp[0]), int(stamp[1])]
        return msg

    def accept_spline(self, msg, metadata):
        try:
            coefficients = np.array([[p.x, p.y, p.z] for p in msg.pos_pts])
            knots = np.array(msg.knots)
            if msg.order != 3 or len(knots) != len(coefficients) + msg.order + 1:
                raise ValueError("invalid cubic B-spline dimensions")
            begin, end = knots[msg.order], knots[-msg.order - 1]
            if not np.isfinite(coefficients).all() or not np.isfinite(knots).all() or end <= begin:
                raise ValueError("nonfinite or empty trajectory")
            curve = BSpline(knots, coefficients, msg.order)
            samples = curve(
                np.linspace(begin, end, min(2000, max(2, int((end - begin) / 0.08) + 1)))
            )
            # Provenance was checked above; additionally reject a start far
            # from actual measured pose. This does not replace collision checks.
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
                    self.message = "忽略起点远离当前 SLAM 位姿的旧轨迹"
                    return
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
        """Internal evidence from the two real exhausted-clear branches only.

        No timer/command zero by itself labels a path exhausted. Samples and
        immutable execution identity are pinned before the inherited clear.
        This receipt has no motion authority and never refreshes source times.
        """
        self.execution_path_exhaustion = None
        values = np.asarray(self.samples)
        reference = self.active_reference_stamp
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
        # Subclasses may provide planning-only guides. Execution still requires
        # an associated checked SCAN curve ending at this original goal.
        return [goal]

    def planner_goal(self):
        return self.waypoints[self.waypoint_index]

    def request_plan(self):
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
        # SCAN REFERENCE_PATH adds grid_map.body_height to every input z. Public
        # mission waypoints are already body centers, so cancel that internal
        # convention at this boundary (without altering requested coordinates).
        reference = self.make_path(self.planner_reference_points(goal), height_offset=0.4)
        # Stamp is an immutable request identity echoed by the planner. Ensure
        # uniqueness even when ROS time pauses or multiple requests share a tick.
        stamp = max(self.last_reference_stamp + 1, self.stamp(reference))
        self.last_reference_stamp = stamp
        reference.header.stamp.sec, reference.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        self.trajectory_association.request(
            [reference.header.stamp.sec, reference.header.stamp.nanosec], goal
        )
        self.reference_pub.publish(reference)
        self.last_reference = time.monotonic()
        self.reference_requests += 1
        self.message = "已请求 SCAN 三维占据地图规划"

    def publish_command(self, velocity=None, yaw_rate=0.0):
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
        now = time.monotonic()
        try:
            self.control(now)
        except Exception as exc:
            self.state, self.message = "failed", f"控制器异常，已停车: {exc}"
            self.publish_command()
            self.get_logger().error(self.message)
        # Fixed-window parking needs actual status coverage within 300 ms.
        # Keep source stamps intact; only reduce the publisher's throttle while
        # the existing controller is in its ordinary zero-command hold.
        parking = getattr(self, "cascade_last_row", None) or {}
        status_period = 0.1 if parking.get("mode") == "ordinary_zero_hold" else 0.25
        if now - self.last_status >= status_period:
            self.publish_status()
            self.last_status = now

    def _diagnostic_business_observed(self, gate, reason, evidence):
        profile = getattr(self, "profile", {})
        config = profile.get("diagnostic_observe_only", {})
        if not (
            isinstance(config, dict)
            and type(config.get("enabled")) is bool
            and config["enabled"]
            and getattr(self, "run", None) is not None
        ):
            return False
        from embodied_navigation.common.observation import api as _observer_api

        record = _observer_api("diagnostic_observation").record
        record(self.run, profile, gate, reason, evidence)
        return True

    def interrupt_active_handoff(self, reason, clock_ns=None):
        gate = getattr(self, "heading_gate", None)
        if not getattr(gate, "handoff_active", False):
            return False
        clock_ns = self.get_clock().now().nanoseconds if clock_ns is None else clock_ns
        gate.fail_handoff(reason, clock_ns / 1e9)
        self.state = "failed"
        self.message = "主动转向交接失败：" + str(gate.handoff_failure)
        return True

    def raw_handoff_deadline(self):
        # Read the original absolute deadline directly. Diagnostic observation
        # may bypass its legacy business gate, never this active-event budget.
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
        ledger = getattr(self, "continuous_route", None)
        exceeded = (
            ledger.deadline_exceeded(clock_ns)
            if ledger is not None
            else clock_ns / 1e9 - self.segment_started_ros
            > self.goals[self.waypoint_index].timeout_sim_s
        )
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
        if self.state != "running":
            self.publish_command()
            return
        control_clock_ns = self.get_clock().now().nanoseconds
        ros_now = control_clock_ns / 1e9
        # A declared region deadline also bounds in-region dwell resets and
        # protected waits after this segment has actually started. Legacy
        # timer/arrival ordering retains its original behavior below.
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
            self.obstacle_resume_pending = False
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
        if stale:
            self.reset_region_arrival("stale")
            self.publish_command()
            self.stale_since = self.stale_since or now
            self.message = "SLAM、点云或原始 IMU 超时，停车等待传感器"
            if now - self.stale_since > 8.0:
                self.state, self.message = "failed", "SLAM、点云或原始 IMU 连续失联超过 8 秒"
            return
        self.stale_since = None
        if protected:
            self.reset_region_arrival("protected")
            self.publish_command()
            return
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
        if self.segment_start is None:
            self.segment_start = self.pose.copy()
            self.segment_started_ros = ros_now
            self.segment_started_ros_ns = control_clock_ns
            self.segment_start_pose_stamp_ns = int(self.pose_stamp)
        distance = float(np.linalg.norm(goal - self.pose))
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
                    # The prior goal's last odom cannot start the next dwell.
                    self.region_arrival.last_stamp = int(self.pose_stamp)
                if self.waypoint_index < len(self.goals):
                    self.region_arrival_evidence = None
                if self.waypoint_index == len(self.waypoints):
                    self.state, self.message = "succeeded", "所有航点均由实际 SLAM 位置确认到达"
                else:
                    self.message = "当前航点已到达，准备下一段"
            return
        self.arrival_since = None
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
        # A v2 goal cannot finish during obstacle_hold. Even when the checked
        # path ends inside its arrival region, recheck the real cloud before
        # returning so a cleared obstacle can complete the existing 1 s gate.
        # Held commands remain exact zero; clearance below discards this path
        # and obtains a new checked SCAN before any movement resumes.
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
        teacher_recovery = start_teacher_recovery(self, current_yaw, desired_yaw)
        if self.obstacle_resume_pending and not teacher_recovery:
            if self.heading_gate.resume_after_stop(
                current_yaw, desired_yaw, self.obstacle_stopped_duration
            ):
                self.aligned_obstacle_resumes += 1
            self.obstacle_resume_pending = False
        if getattr(self, "reference_consistency_hold", False):
            # Direction inconsistency has no authority to substitute a chord.
            # Hold the heading state too, and obtain a new checked SCAN curve.
            allow_translation, yaw_rate = False, 0.0
        else:
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

                    input_binding(ack)  # Keeps the actual frozen Student/input binding.
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
            allow_translation, yaw_rate = self.heading_gate.update(
                ros_now, current_yaw, desired_yaw
            )
        if getattr(self.heading_gate, "handoff_failure", None):
            self.state = "failed"
            self.message = "主动转向交接失败：" + str(self.heading_gate.handoff_failure)
            self.publish_command()
            return
        self.alignment_hold = not allow_translation
        if not allow_translation:
            velocity = np.zeros(2)
        velocity, yaw_rate = self.select_pid_velocity(velocity, yaw_rate, target, allow_translation)
        if getattr(self.heading_gate, "handoff_failure", None):
            self.state = "failed"
            self.message = "主动转向交接失败：" + str(self.heading_gate.handoff_failure)
            self.publish_command()
            return
        actual_guard_evaluated = False
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
                # Freeze the input at computation time, including a cached
                # result's scan, rather than reading self.cloud on a later tick.
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
            if not self.obstacle_hold:
                self.obstacle_stops += 1
                self.obstacle_hold_started_ros = ros_now
                self.pending_obstacle_event = self.obstacle_guard_context
            self.obstacle_hold = True
            self.obstacle_clear_since = None
            if self.min_obstacle_clearance is None or clearance < self.min_obstacle_clearance:
                self.min_obstacle_clearance = clearance
        elif self.obstacle_hold:
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
            # Zero is sent before queueing. Compression, JSON conversion and
            # file I/O never run in this safety/control callback.
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
        # If upstream ends its trajectory before measured arrival, request a new
        # trajectory from measured pose instead of declaring time-based success.
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
        complete_teacher_recovery(self)

    @_callback_cost_observe("publish_status")
    def publish_status(self):
        now = time.monotonic()
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
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main():
    rclpy.init()
    node = Navigation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except _ros.RCLError:
        # SIGINT can invalidate the context between executor wait-set calls.
        # Errors while the context is live must still fail this required node.
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
