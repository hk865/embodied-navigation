"""Building navigation from canonical FAST-LIVO2 pose/cloud.

Simulation truth is never published as a SLAM input. Legacy wire names remain
for compatibility. Navigation uses the frozen PCT route, accepted SCAN spline
and measured FAST-LIVO2 pose/cloud, with a separate contact-safety input. Shared
control/parking components perform tracking; this wrapper is not a planner.
"""

from embodied_navigation.project import project_root as _project_root
from pathlib import Path
import json, os, time, math, collections
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.interpolate import BSpline
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path as NavPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from sensor_msgs_py import point_cloud2
from scan_planner_msgs.msg import Bspline
from embodied_navigation.control.parking.parking_pose10hz import CaptureGrid, PoseGrid
from embodied_navigation.control.safety.contact_mailbox import ContactMailbox
from embodied_navigation.control.tracking.terrain_speed import StairSpeedLimit
from embodied_navigation.control.tracking.planned_step_filter import PlannedStepObstacle

ROOT = _project_root()
run = Path(os.environ["DEMO_RUN_DIR"]).resolve()
settings = json.loads((run / "experiment_setup.json").read_text())
route = json.loads((run / "PCT_route.json").read_text())
origin = np.array(settings["navigation_world_origin"])
world_points = np.array(route["world_body_points"])
ref = world_points - origin
ground = ref - [0, 0, 0.32]
goal = ref[-1]
final_heading = math.atan2(ref[-1, 1] - ref[-2, 1], ref[-1, 0] - ref[-2, 0])
sensor = json.loads((run / "sensor_contract.json").read_text())
pl = np.array(sensor["lidar"]["body_xyz"])
rl = Rotation.from_euler("xyz", sensor["lidar"]["body_rpy"]).as_matrix()
if settings.get("navigation_mode") != "slam":
    raise ValueError(
        "Production navigation requires canonical SLAM; simulation truth is safety/evaluation only"
    )
SLAM = True


def stamp(ns):
    from builtin_interfaces.msg import Time

    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def atomic(p, obj):
    tmp = p.with_suffix(".tmp")
    tmp.write_text(json.dumps(obj, allow_nan=False) + "\n")
    tmp.replace(p)


rclpy.init()


class Navigation(Node):
    def __init__(self):
        super().__init__("building_slam_navigation")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.stair_speed = StairSpeedLimit(ref, settings.get("stair_speed_cap_mps", 0.25))
        self.step_obstacle = PlannedStepObstacle(ref)
        self.route_guard = (
            self.step_obstacle.progress if route.get("require_connected_terminal") else None
        )
        self.pose = None
        self.pose_wall = None
        self.safety_ready = False
        self.cache = collections.deque(maxlen=120)
        self.pending_cloud = collections.deque(maxlen=4)
        self.cloud = None
        self.obstacle_pose = None
        self.cloud_ns = None
        self.cloud_wall = None
        self.curve = None
        self.curve_ns = None
        self.progress = 0.0
        self.sent = False
        self.path_sent_ns = None
        self.command = np.zeros(3)
        self.seq = 0
        self.stage = "waiting"
        self.failure = None
        self.capture = CaptureGrid()
        self.park = None
        self.park_result = None
        self.park_zero_ack = None
        self.capture_proof = None
        self.last_status = 0.0
        self.zero_issued = None
        self.latest_ack = None
        self.candidate_count = 0
        self.candidate_rejections = 0
        self.last_candidate_reason = None
        self.contact_since = None
        self.pose_source_file = None
        self.ack_file = None
        self.first_pose_ns = None
        self.last_published = -1
        self.route_progress = 0.0
        self.pathlog = (run / "accepted_SCAN_curves.jsonl").open("w", buffering=1)
        self.history = (run / "navigation_history.jsonl").open("w", buffering=1)
        self.poselog = (run / "navigation_slam_poses.jsonl").open("w", buffering=1)
        self.contact_mailbox = ContactMailbox(os.environ["BUILDING_SAFETY_SOCKET"])
        self.pathpub = self.create_publisher(NavPath, "/truth/scan_reference", 10)
        self.frozen = self.create_publisher(Bool, "/truth/execution_frozen", 10)
        self.create_subscription(
            Odometry, "/demo/slam/body_odom", self.slam_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "/cloud_registered_full", self.slam_cloud, qos_profile_sensor_data
        )
        self.create_subscription(Bspline, "/truth/bspline", self.on_curve, 10)
        self.create_timer(0.01, self.tick)

    def slam_pose(self, msg):
        p = msg.pose.pose
        v = msg.twist.twist
        omega = np.array([v.angular.x, v.angular.y, v.angular.z])
        velocity = np.array([v.linear.x, v.linear.y, v.linear.z]) + np.cross(
            omega, [0.0552309203455, -0.0018690019194, 0.0060952991843]
        )
        row = dict(
            stamp_ns=msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec,
            position=(np.array([p.position.x, p.position.y, p.position.z]) + origin).tolist(),
            quaternion_wxyz=[p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z],
            velocity_body_com=velocity.tolist(),
            angular_velocity_body=omega.tolist(),
            contacts=[0.0, 0.0, 0.0, 0.0, 0.0],
            source="FAST_LIVO2_and_async_SE3_correction",
        )
        self.accept_pose(row, False)
        if self.cloud_ns == row["stamp_ns"]:
            self.bind_obstacle_pose(row)

    def bind_obstacle_pose(self, row):
        q = row["quaternion_wxyz"]
        self.obstacle_pose = (
            np.asarray(row["position"]) - origin,
            Rotation.from_quat(q[1:] + q[:1]).as_matrix(),
        )

    def slam_cloud(self, msg):
        self.cloud = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        self.cloud_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        self.cloud_wall = time.monotonic()
        self.obstacle_pose = None
        for row in reversed(self.cache):
            if row["stamp_ns"] == self.cloud_ns:
                self.bind_obstacle_pose(row)
                break

    def native_rows(self):
        self.contact_mailbox.drain()
        self.safety_ready = self.contact_mailbox.fresh(self.get_clock().now().nanoseconds)
        if self.pose is not None:
            self.pose["contacts"] = list(self.contact_mailbox.contacts)
        # The code below is the original JSONL replay path. It is unreachable
        # here because of this early return; live pose/cloud arrive through the
        # /demo/slam subscriptions created in __init__.
        return
        if self.pose_source_file is None:
            try:
                self.pose_source_file = (run / "native_pose_source.jsonl").open()
            except OSError:
                return
        while True:
            pos = self.pose_source_file.tell()
            line = self.pose_source_file.readline()
            if not line:
                return
            if not line.endswith("\n"):
                self.pose_source_file.seek(pos)
                return
            self.accept_pose(json.loads(line), True)

    def accept_pose(self, row, publish):
        self.pose = row
        self.pose_wall = time.monotonic()
        self.cache.append(row)
        if self.first_pose_ns is None:
            self.first_pose_ns = row["stamp_ns"]
        if publish:
            self.publish_pose(row)
        self.poselog.write(json.dumps(row) + "\n")
        if self.park is not None and self.stage == "parking":
            self.park.observe(self.pose_row(row))
            ev = self.park.evidence()
            if ev.get("motion_failure") is not None or ev.get("missing", []):
                self.failure = "original_parking_10Hz_window_failed"
                self.stage = "failed"
                self.park_result = ev
            if row["stamp_ns"] >= self.park.start_ns + 5000000000:
                self.park_result = self.park.evidence()
                self.stage = "succeeded" if self.park_result.get("passed") else "failed"
                if self.stage == "failed":
                    self.failure = "original_parking_10Hz_window_not_passed"
                atomic(
                    run / "parking_10Hz_result.json",
                    {
                        "navigation_ground_truth_used": False,
                        "capture": self.capture_proof,
                        "zero_ack": self.park_zero_ack,
                        "window": self.park_result,
                        "T0_ns": self.park.start_ns,
                        "goal_world_xyz": world_points[-1].tolist(),
                        "final_slam_world_xyz": row["position"],
                    },
                )

    def pose_row(self, row):
        return {
            "stamp_ns": row["stamp_ns"],
            "position": (np.array(row["position"]) - origin).tolist(),
            "quaternion_xyzw": row["quaternion_wxyz"][1:] + row["quaternion_wxyz"][:1],
            "source_topic": "/demo/slam/body_odom",
            "pose_producer": "fast_livo2_async_backend",
            "navigation_ground_truth_used": False,
            "received_sim_ns": self.get_clock().now().nanoseconds,
            "received_wall_ns": time.monotonic_ns(),
        }

    def publish_pose(self, row):
        """Original odometry re-publish path (body/lidar/cloud/scan publishers).

        The publishers it needs (``bodypub``/``sensorpub``/``cloudpub``/
        ``scanpub``) are not created by this node, and ``slam_pose`` always calls
        ``accept_pose(..., publish=False)``; treat this as retained legacy code.
        """
        ns = row["stamp_ns"]
        q = row["quaternion_wxyz"]
        rot = Rotation.from_quat(q[1:] + q[:1]).as_matrix()
        body = np.array(row["position"]) - origin
        for pub, pos, child in [
            (self.bodypub, body, "demo_slam_body"),
            (self.sensorpub, body + rot @ pl, "demo_slam_lidar"),
        ]:
            msg = Odometry()
            msg.header.stamp = stamp(ns)
            msg.header.frame_id = "camera_init"
            msg.child_frame_id = child
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(
                float, pos
            )
            (
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
            ) = map(float, q)
            v = np.array(row["velocity_body_com"]) - np.cross(
                row["angular_velocity_body"], [0.0552309203455, -0.0018690019194, 0.0060952991843]
            )
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = map(
                float, v
            )
            msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = map(
                float, row["angular_velocity_body"]
            )
            pub.publish(msg)
            if pub is self.bodypub:
                import copy

                scan = copy.deepcopy(msg)
                scan.child_frame_id = "demo_scan_body"
                world_v = rot @ v
                world_w = rot @ np.array(row["angular_velocity_body"])
                scan.twist.twist.linear.x, scan.twist.twist.linear.y, scan.twist.twist.linear.z = (
                    map(float, world_v)
                )
                (
                    scan.twist.twist.angular.x,
                    scan.twist.twist.angular.y,
                    scan.twist.twist.angular.z,
                ) = map(float, world_w)
                self.scanpub.publish(scan)

    def process_clouds(self):
        return
        while self.pending_cloud and len(self.cache) > 1:
            msg = self.pending_cloud[0]
            ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            if ns > self.cache[-1]["stamp_ns"]:
                return
            self.pending_cloud.popleft()
            left = None
            right = None
            for a, b in zip(list(self.cache)[:-1], list(self.cache)[1:]):
                if a["stamp_ns"] <= ns <= b["stamp_ns"]:
                    left, right = (a, b)
                    break
            if left is None:
                continue
            u = (ns - left["stamp_ns"]) / (right["stamp_ns"] - left["stamp_ns"])
            pos = (1 - u) * np.array(left["position"]) + u * np.array(right["position"])
            qs = [v["quaternion_wxyz"][1:] + v["quaternion_wxyz"][:1] for v in [left, right]]
            rot = Slerp([0, 1], Rotation.from_quat(qs))([u]).as_matrix()[0]
            xyz = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            xyz = xyz[np.isfinite(xyz).all(1)]
            body = xyz @ rl.T + pl
            inside = (
                (abs(body[:, 0]) < 0.5)
                & (abs(body[:, 1]) < 0.3)
                & (body[:, 2] > -0.45)
                & (body[:, 2] < 0.3)
            )
            body = body[~inside]
            world = body @ rot.T + pos
            nav = world - origin
            if len(nav):
                key = np.floor(nav / 0.07).astype(np.int64)
                _, idx = np.unique(key, axis=0, return_index=True)
                nav = nav[idx].astype(np.float32)
            header = msg.header
            header.frame_id = "camera_init"
            out = point_cloud2.create_cloud_xyz32(header, nav)
            self.cloudpub.publish(out)
            self.cloud = nav
            self.cloud_ns = ns
            self.cloud_wall = time.monotonic()
            self.obstacle_pose = (pos - origin, rot)

    def on_curve(self, msg):
        self.candidate_count += 1
        points = np.array([[p.x, p.y, p.z] for p in msg.pos_pts])
        knots = np.asarray(msg.knots)
        try:
            if (
                msg.order != 3
                or len(points) < 4
                or len(knots) != len(points) + 4
                or (not np.isfinite(points).all())
            ):
                raise ValueError("invalid spline structure")
            curve = BSpline(knots, points, 3)
            lo = knots[3]
            hi = knots[-4]
            ts = np.linspace(lo, hi, max(20, int((hi - lo) / 0.025) + 1))
            samples = curve(ts)
            delta = ref[1:] - ref[:-1]
            den = (delta * delta).sum(1)
            offset = samples[:, None, :] - ref[:-1]
            u = np.clip((offset * delta).sum(2) / den, 0, 1)
            projection = ref[:-1] + u[:, :, None] * delta
            dist = np.linalg.norm(samples[:, None, :] - projection, axis=2).min(1)
            if float(dist.max()) > 0.5:
                raise ValueError("actual_SCAN_outside_PCT_3D_fence:" + str(float(dist.max())))
            self.curve = (curve, ts, samples)
            self.curve_ns = msg.start_time.sec * 1000000000 + msg.start_time.nanosec
            self.progress = 0.0
            self.last_candidate_reason = None
            self.pathlog.write(
                json.dumps(
                    {
                        "traj_id": msg.traj_id,
                        "source_start_ns": self.curve_ns,
                        "order": msg.order,
                        "knots": knots.tolist(),
                        "pos_pts": points.tolist(),
                        "sample_z_minmax": [float(samples[:, 2].min()), float(samples[:, 2].max())],
                        "maximum_PCT_route_distance_m": float(dist.max()),
                        "navigation_ground_truth_used": False,
                    }
                )
                + "\n"
            )
        except Exception as e:
            self.candidate_rejections += 1
            self.last_candidate_reason = str(e)

    def send_path(self):
        msg = NavPath()
        msg.header.frame_id = "map"
        msg.header.stamp = stamp(self.get_clock().now().nanoseconds)
        for p in ref:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(
                float, p - [0, 0, 0.4]
            )
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pathpub.publish(msg)
        self.sent = True
        self.path_sent_ns = self.get_clock().now().nanoseconds
        atomic(
            run / "SCAN_reference_binding.json",
            {
                "native_body_height_transport_offset_m": 0.4,
                "body_reference_world": world_points.tolist(),
                "published_SCAN_reference_nav": (ref - [0, 0, 0.4]).tolist(),
                "PCT_ground_world": route["world_ground_points"],
                "navigation_origin_world": origin.tolist(),
                "no_layer_index_height_substitution": True,
            },
        )

    def obstacle(self, p, rot):
        if self.cloud is None:
            return False
        if self.obstacle_pose is None:
            return True
        p, rot = self.obstacle_pose
        return self.step_obstacle.obstacle(self.cloud, p, rot, advance=False)

    def read_ack(self):
        if self.ack_file is None:
            try:
                self.ack_file = (run / "actual_command_ack.jsonl").open()
            except OSError:
                return
        while True:
            pos = self.ack_file.tell()
            l = self.ack_file.readline()
            if not l:
                return
            if not l.endswith("\n"):
                self.ack_file.seek(pos)
                return
            self.latest_ack = json.loads(l)

    def tick(self):
        self.native_rows()
        self.read_ack()
        self.process_clouds()
        now_ns = self.get_clock().now().nanoseconds
        now = time.monotonic()
        cmd = np.zeros(3)
        reason = "initializing"
        freeze = True
        if self.pose is not None:
            p = np.array(self.pose["position"]) - origin
            q = self.pose["quaternion_wxyz"]
            rot = Rotation.from_quat(q[1:] + q[:1]).as_matrix()
            yaw = math.atan2(rot[1, 0], rot[0, 0])
            elapsed = (self.pose["stamp_ns"] - self.first_pose_ns) / 1000000000.0
            if self.pose["contacts"][0] > 0:
                if self.contact_since is None:
                    self.contact_since = self.pose["stamp_ns"]
                if self.pose["stamp_ns"] - self.contact_since > 250000000:
                    self.failure = "sustained_nonfoot_collision"
                    self.stage = "failed"
            else:
                self.contact_since = None
            fresh = (
                0 <= now - self.pose_wall < 0.3
                and -0.05 <= (now_ns - self.pose["stamp_ns"]) / 1000000000.0 < 0.3
                and (self.cloud_wall is not None)
                and (0 <= now - self.cloud_wall < 0.3)
                and (-0.05 <= (now_ns - self.cloud_ns) / 1000000000.0 < 0.3)
            )
            slam_ready = True
            if settings.get("backend_capture_only"):
                try:
                    slam_ready = (
                        json.loads((run / "slam_status.json").read_text()).get("pairs", 0) > 0
                    )
                except (OSError, ValueError):
                    slam_ready = False
            fresh = fresh and slam_ready and self.safety_ready and (self.obstacle_pose is not None)
            if (
                elapsed > 3
                and fresh
                and (
                    not self.sent
                    or (self.candidate_count == 0 and now_ns - self.path_sent_ns > 5000000000)
                )
            ):
                self.send_path()
                self.stage = "planning"
            if self.sent and elapsed > settings["timeout_sim_s"]:
                self.failure = "mission_simulation_timeout"
                self.stage = "failed"
            distance = float(np.linalg.norm(p[:2] - goal[:2]))
            height_error = abs(float(p[2] - goal[2]))
            self.step_obstacle.progress.update(p)
            if fresh and self.stage not in ["failed", "succeeded", "parking"]:
                if (
                    distance < 0.45
                    and height_error < 0.18
                    and (self.route_guard is None or self.route_guard.remaining < 1.0)
                ):
                    self.stage = "terminal_capture"
                    error = (goal - p) @ rot
                    heading_error = wrap(final_heading - yaw)
                    origin_v = np.array(self.pose["velocity_body_com"]) - np.cross(
                        self.pose["angular_velocity_body"],
                        [0.0552309203455, -0.0018690019194, 0.0060952991843],
                    )
                    cmd = np.array(
                        [
                            np.clip(error[0] * 4.0 - 0.4 * origin_v[0], -0.16, 0.16),
                            np.clip(error[1] * 4.0 - 0.4 * origin_v[1], -0.12, 0.12),
                            np.clip(heading_error * 1.5, -0.2, 0.2),
                        ]
                    )
                    reason = "slam_goal_pose_PD_feedback"
                    if abs(heading_error) > 0.25:
                        cmd[:2] = 0
                    ready = self.capture.observe(self.pose_row(self.pose), goal, final_heading)
                    if ready:
                        cmd[:] = 0.0
                        reason = "original_10Hz_capture_ready_wait_zero_ACK"
                        if self.zero_issued is None:
                            self.zero_issued = now_ns
                        ack = self.latest_ack
                        if (
                            ack is not None
                            and ack["stamp_ns"] >= self.zero_issued
                            and ack["actor_inferred"]
                            and (ack["fault"] is None)
                            and (ack["command"] == [0.0, 0.0, 0.0])
                        ):
                            self.park_zero_ack = dict(ack)
                            self.capture_proof = self.capture.evidence()
                            self.park = PoseGrid(
                                ack["stamp_ns"], self.capture_proof["samples"][-1]["source"]
                            )
                            self.stage = "parking"
                            reason = "fixed5s_10Hz_parking"
                    freeze = False
                elif self.curve is not None:
                    freeze = False
                    curve, ts, samples = self.curve
                    dist = np.linalg.norm(samples - p, axis=1)
                    idx = int(dist.argmin())
                    ahead = idx
                    length = 0.0
                    while ahead + 1 < len(samples) and length < 0.36:
                        length += np.linalg.norm(samples[ahead + 1, :2] - samples[ahead, :2])
                        ahead += 1
                    target = samples[ahead]
                    delta = target - p
                    heading = math.atan2(delta[1], delta[0])
                    he = wrap(heading - yaw)
                    local = delta @ rot
                    if np.linalg.norm(delta[:2]) < 0.06:
                        reason = "SCAN_local_end_wait_replan"
                    else:
                        vx = self.stair_speed.cruise(p) * max(0.0, math.cos(he))
                        vy = float(np.clip(0.8 * local[1], -0.12, 0.12))
                        w = float(np.clip(1.8 * he, -0.3, 0.3))
                        if abs(he) > 0.3:
                            vx = 0.0
                            vy = 0.0
                        else:
                            w = float(np.clip(w, -0.16, 0.16))
                        cmd = np.array([vx, vy, w])
                        self.stage = "tracking"
                        reason = "real_SCAN_spatial_lookahead"
                        freeze = False
                else:
                    reason = "waiting_actual_SCAN_candidate"
                if np.any(cmd) and self.obstacle(p, rot):
                    cmd[:] = 0.0
                    reason = "actual_registered_cloud_obstacle_hold"
                    freeze = True
            elif not fresh:
                reason = (
                    "source_freshness_hold"
                    if slam_ready
                    else "waiting_stationary_SLAM_initialization"
                )
            if self.stage == "parking":
                reason = "fixed5s_10Hz_parking"
                cmd[:] = 0
            if self.stage in ["succeeded", "failed"]:
                reason = self.failure or "all_original_10Hz_parking_conditions_passed"
                cmd[:] = 0
            status = {
                "state": self.stage,
                "reason": reason,
                "failure": self.failure,
                "sim_s": self.pose["stamp_ns"] / 1000000000.0,
                "clock_s": now_ns / 1000000000.0,
                "pose_source_age_ms": (now_ns - self.pose["stamp_ns"]) / 1000000.0,
                "cloud_source_age_ms": None
                if self.cloud_ns is None
                else (now_ns - self.cloud_ns) / 1000000.0,
                "connected_route_remaining_m": None
                if self.route_guard is None
                else self.route_guard.remaining,
                "last_planned_step_points_ignored": self.step_obstacle.ignored,
                "last_unexplained_obstacle_points": self.step_obstacle.remaining,
                "reference_progress_m": self.step_obstacle.progress.progress,
                "body_speed_mps": float(np.linalg.norm(self.pose["velocity_body_com"])),
                "angular_speed_rad_s": float(np.linalg.norm(self.pose["angular_velocity_body"])),
                "pose_nav": p.tolist(),
                "pose_world": self.pose["position"],
                "yaw": yaw,
                "goal_world": world_points[-1].tolist(),
                "goal_xy_error_m": distance,
                "goal_height_error_m": height_error,
                "goal_yaw_error_rad": wrap(final_heading - yaw),
                "command": cmd.tolist(),
                "candidate_count": self.candidate_count,
                "candidate_rejections": self.candidate_rejections,
                "last_candidate_reason": self.last_candidate_reason,
                "navigation_ground_truth_used": False,
                "SLAM_or_global_matching_enabled": True,
                "parking_window": None if self.park is None else self.park.evidence(),
            }
            if now - self.last_status > 0.1:
                atomic(run / "truth_navigation_status.json", status)
                self.history.write(json.dumps(status) + "\n")
                self.last_status = now
        self.seq += 1
        self.command = cmd
        atomic(
            run / "truth_navigation_command.json",
            {
                "run_id": run.name,
                "sequence": self.seq,
                "stamp_ns": now_ns,
                "monotonic_wall": now,
                "command": cmd.tolist(),
                "goal_index": 0,
                "navigation_ground_truth_used": False,
                "source": "PCT->native SCAN->FAST_LIVO2 spatial feedback",
            },
        )
        self.frozen.publish(Bool(data=freeze))


n = Navigation()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
finally:
    for f in [n.pose_source_file, n.ack_file, n.pathlog, n.history, n.poselog]:
        if f:
            f.close()
    if n.contact_mailbox is not None:
        n.contact_mailbox.close()
    n.destroy_node()
    rclpy.try_shutdown()
