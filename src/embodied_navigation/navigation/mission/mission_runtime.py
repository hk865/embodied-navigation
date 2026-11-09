"""Publish the current global goal/reference after live localization is ready.

There is no experiment score, evidence capture, archived source or PASS output.
"""

from pathlib import Path
import argparse, copy, json, math, time
import numpy as np
from embodied_navigation.common.runtime_context import load
from embodied_navigation.common.config import ROOT


def atomic(path, value):
    """Write JSON with ``allow_nan=False`` via a sibling temp file + replace."""
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, allow_nan=False) + "\n")
    temporary.replace(path)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String, Bool
    from embodied_navigation.common.runtime_io import latest_sensor_qos

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    args, ros = parser.parse_known_args()
    directory = args.run.resolve()
    profile = load(directory / "runtime_context.json")["profile"]
    route = json.loads((directory / "global_route.json").read_text())
    points = np.asarray(route["body_points"], float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError("Invalid global route geometry")

    class Mission(Node):
        def __init__(self):
            super().__init__("teacher_navigation_request")
            self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
            self.pose = None
            self.imu = None
            self.localization = None
            self.epoch = None
            self.health = None
            self.received = {}
            self.initial_samples = []
            self.request = None
            self.anchor = None
            self.reference_scope = None
            self.last_emit = -math.inf
            self.request_id = directory.name + ":navigation:1"
            self.request_pub = self.create_publisher(String, "/demo/navigation/request", 10)
            self.anchor_pub = self.create_publisher(
                String,
                "/demo/teacher/navigation/anchor",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            self.stop_pub = self.create_publisher(Bool, "/demo/navigation/stop", 10)
            self.create_subscription(
                Odometry,
                "/demo/slam/body_odom",
                self.on_pose,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.create_subscription(Imu, "/livox/imu", self.on_imu, latest_sensor_qos())
            for role, topic in [
                ("localization", "/demo/navigation/localization_status"),
                ("epoch", "/demo/navigation/scan_epoch_ready"),
                ("health", "/demo/teacher/navigation/sensor_health"),
            ]:
                self.create_subscription(
                    String, topic, lambda message, k=role: self.on_json(k, message), 10
                )
            self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def on_json(self, role, message):
            """Record one JSON status document (localization/epoch/health) by role."""
            try:
                data = json.loads(message.data)
            except (ValueError, TypeError):
                return
            setattr(self, role, data)
            self.received[role] = time.monotonic()

        def on_imu(self, message):
            """Keep the latest IMU sample with its receive wall time."""
            self.imu = (message, time.monotonic())

        def on_pose(self, message):
            """Keep one finite SLAM body-pose sample; non-finite samples are dropped."""
            ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            p = message.pose.pose.position
            q = message.pose.pose.orientation
            v = message.twist.twist.linear
            value = dict(
                stamp_ns=ns,
                position=[p.x, p.y, p.z],
                quaternion_xyzw=[q.x, q.y, q.z, q.w],
                body_velocity=[v.x, v.y, v.z],
                frame_id=message.header.frame_id,
                child_frame_id=message.child_frame_id,
                source_topic="/demo/slam/body_odom",
                navigation_ground_truth_used=False,
            )
            if not np.isfinite(
                value["position"] + value["quaternion_xyzw"] + value["body_velocity"]
            ).all():
                return
            self.pose = (value, time.monotonic())

        def ready(self, clock, now):
            """Gate live pose/IMU and the localization, epoch and health statuses.

            Pose and IMU must be causal within 300 ms of the node clock. Those
            samples and all three status receipts must be less than 0.3 s old
            in wall time. Localization must be initialized without transition,
            the ready epoch must match its generation, and health must be ready.
            """
            if self.pose is None or self.imu is None:
                return False
            if any(
                not 0 <= now - self.received.get(role, -math.inf) < 0.3
                for role in ["localization", "epoch", "health"]
            ):
                return False
            source, received = self.pose
            imu, imu_wall = self.imu
            stamp = imu.header.stamp.sec * 1_000_000_000 + imu.header.stamp.nanosec
            if not (
                0 <= clock - source["stamp_ns"] <= 300_000_000
                and 0 <= clock - stamp <= 300_000_000
                and 0 <= now - received < 0.3
                and 0 <= now - imu_wall < 0.3
            ):
                return False
            return (
                self.localization.get("initialized") is True
                and self.localization.get("correction_transition") is False
                and self.epoch.get("ready") is True
                and self.epoch.get("localization_generation") == self.localization.get("generation")
                and self.health.get("ready") is True
            )

        def initialize_goal(self, clock, now):
            """Derive and freeze the goal once the body has been measured stationary.

            Requires body speed <= 0.03 m/s and |gyro| <= 0.05 rad/s; >= 10
            samples spanning >= 0.8 s are needed. The anchor and goal request
            files are each atomically replaced; the request then stays fixed.
            """
            value = self.pose[0]
            imu = self.imu[0]
            if (
                np.linalg.norm(value["body_velocity"]) > 0.03
                or np.linalg.norm(
                    [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]
                )
                > 0.05
            ):
                self.initial_samples = []
                return
            if not self.initial_samples or value["stamp_ns"] > self.initial_samples[-1]["stamp_ns"]:
                self.initial_samples.append(copy.deepcopy(value))
            self.initial_samples = self.initial_samples[-32:]
            if (
                len(self.initial_samples) < 10
                or self.initial_samples[-1]["stamp_ns"] - self.initial_samples[0]["stamp_ns"]
                < 800_000_000
            ):
                return
            cfg = profile["mission"]
            origin = value["position"]
            goal = dict(
                goal_id=cfg["goal_id"],
                center=(np.asarray(origin) + cfg["relative_offset_xyz"]).tolist(),
                arrival=dict(
                    type="disc_prism",
                    axes=np.eye(3).tolist(),
                    radius_m=cfg["arrival_radius_m"],
                    height_half_span_m=cfg["arrival_height_half_span_m"],
                    dwell_sim_s=cfg["dwell_sim_s"],
                    control_band=dict(
                        type="disc_prism",
                        axes=np.eye(3).tolist(),
                        radius_m=cfg["control_radius_m"],
                        height_half_span_m=cfg["arrival_height_half_span_m"],
                        dwell_sim_s=cfg["dwell_sim_s"],
                    ),
                ),
                timeout_sim_s=cfg["timeout_sim_s"],
                route_surface_id=cfg["surface_id"],
                legacy=False,
            )
            self.goal = goal
            self.anchor = dict(
                schema=1,
                experiment="navigation",
                run_dir=str(directory),
                origin=origin,
                yaw=0.0,
                pose_stamp_ns=value["stamp_ns"],
                frame_id="camera_init",
                source="/demo/slam/body_odom",
                ground_truth_navigation_used=False,
                initial_actual_slam_source=value,
                registered_route_camera_init_xyz=[origin] + route["body_points"],
                T_navigation_world=route["T_navigation_world"],
            )
            self.request = dict(
                schema_version=2, request_id=self.request_id, frame_id="camera_init", goals=[goal]
            )
            atomic(directory / "navigation_anchor.json", self.anchor)
            atomic(directory / "navigation_request.json", self.request)

        def update_reference(self):
            """Publish the route reference once per (epoch, pid, generation) scope.

            The scope is derived from the planner epoch, its PID and the
            localization generation; the document is written atomically and the
            typed terminal heading reference is attached unchanged.
            """
            epoch = self.epoch.get("planner_epoch")
            generation = self.localization.get("generation")
            pid = (self.epoch.get("planner_identity") or {}).get("pid")
            if any(type(v) is not int or v < 1 for v in [epoch, generation, pid]):
                return
            scope = (epoch, pid, generation)
            if scope == self.reference_scope:
                return
            document = dict(
                schema="r85_pct_scan_global_reference/v1",
                run_id=directory.name,
                frame_id="camera_init",
                epoch=epoch,
                localization_generation=generation,
                reference_id=self.request_id
                + ":epoch:"
                + str(epoch)
                + ":generation:"
                + str(generation),
                body_points=route["body_points"],
                ground_points=route["ground_points"],
                source_route_file=str(ROOT / "assets/geometry/route.json"),
                physical_body_ground_offset_m=0.32,
                scan_transport_offset_m=0.4,
                navigation_ground_truth_used=False,
                physical_sources=route["physical_sources"],
                T_navigation_world=route["T_navigation_world"],
                endpoint_support_connectors=route["endpoint_support_connectors"],
                task_goal_body_xyz=self.goal["center"],
                original_F3_goal_body_xyz=self.goal["center"],
                reference_scope="full_route",
            )
            from embodied_navigation.control.parking.terminal_heading_contract import (
                attach_reference_heading,
            )

            document = attach_reference_heading(profile, document)
            atomic(directory / "global_reference.json", document)
            self.reference_scope = scope

        def tick(self):
            """One control-loop step: gate freshness, init goal, publish reference."""
            now = time.monotonic()
            clock = self.get_clock().now().nanoseconds
            if not self.ready(clock, now):
                return
            if self.request is None:
                self.initialize_goal(clock, now)
            if self.request is None:
                return
            self.update_reference()
            if now - self.last_emit >= 1.0:
                self.anchor_pub.publish(String(data=json.dumps(self.anchor)))
                self.request_pub.publish(String(data=json.dumps(self.request)))
                self.last_emit = now

    rclpy.init(args=ros)
    node = Mission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop_pub.publish(Bool(data=True))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
