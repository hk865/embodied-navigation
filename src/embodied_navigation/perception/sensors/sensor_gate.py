#!/usr/bin/env python3
"""Warm up actual sensor SLAM before permitting a bounded route request."""

from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import time
import sys
from embodied_navigation.common.runtime_io import BackgroundCheck, latest_sensor_qos
from embodied_navigation.common.runtime_telemetry import RuntimeTelemetry
from embodied_navigation.common.input_freshness import enabled as minimal_baseline_enabled

TOPIC = "/demo/teacher/navigation/sensor_health"


class ImageInfoPairs:
    """Bounded original-stamp pairs; a newer unpaired Info does not erase one."""

    def __init__(self, capacity=32):
        self.capacity = capacity
        self.images = {}
        self.infos = {}

    def observe(self, name, stamp, frame, width, height, wall):
        """Cache one image/info sample under its source stamp.

        ``name`` selects the image or the camera-info cache; the stored tuple is
        (frame_id, width, height, received_wall_s) and only the newest
        ``capacity`` stamps are kept (the lowest stamp is evicted first).
        """
        cache = self.images if name == "image" else self.infos
        cache[stamp] = (frame, width, height, wall)
        while len(cache) > self.capacity:
            cache.pop(min(cache))

    def latest_fresh(self, clock_ns, wall_now, sim_limit=0.3, wall_limit=0.3, observe_wall=False):
        if clock_ns is None:
            return None
        for image_stamp, image in sorted(self.images.items(), reverse=True):
            if not (
                -50_000_000 <= clock_ns - image_stamp < round(sim_limit * 1e9)
                and 0 <= wall_now - image[3]
                and (observe_wall or wall_now - image[3] < wall_limit)
            ):
                continue
            candidates = [
                (abs(stamp - image_stamp), stamp, info)
                for stamp, info in self.infos.items()
                if info[:3] == image[:3]
                and abs(stamp - image_stamp) <= 50_000_000
                and -50_000_000 <= clock_ns - stamp < round(sim_limit * 1e9)
                and 0 <= wall_now - info[3]
                and (observe_wall or wall_now - info[3] < wall_limit)
            ]
            if candidates:
                difference, info_stamp, info = min(candidates)
                return {
                    "image_stamp_ns": image_stamp,
                    "camera_info_stamp_ns": info_stamp,
                    "pair_stamp_difference_ns": difference,
                    "frame_id": image[0],
                    "width": image[1],
                    "height": image[2],
                    "image_wall_age_s": wall_now - image[3],
                    "camera_info_wall_age_s": wall_now - info[3],
                }
        return None


class WarmupRecovery:
    """Cumulative real initialization history; current release remains strict.

    An isolated cloud gap does not erase samples FAST-LIVO already initialized
    from. It still immediately stops motion and resets fresh recovery samples.
    """

    def __init__(self, warmup_s=2.0, recovery_samples=2, recovery_span_s=0.1, requirements=None):
        self.warmup_ns = round(warmup_s * 1e9)
        self.recovery_samples = recovery_samples
        self.recovery_span_ns = round(recovery_span_s * 1e9)
        self.complete = False
        self.good_since = None
        self.first_pose = None
        self.last_pose = None
        self.samples = 0
        self.requirements = requirements or {
            "body_odom": {"min_samples": 10, "min_span_sim_s": warmup_s},
            "cloud": {"min_samples": 10, "min_span_sim_s": warmup_s},
        }
        self.history = {}

    def record(self, name, stamp_ns):
        if name not in self.requirements:
            return
        previous = self.history.get(name)
        if previous is None:
            self.history[name] = {
                "first_stamp_ns": stamp_ns,
                "last_stamp_ns": stamp_ns,
                "samples": 1,
            }
        elif stamp_ns > previous["last_stamp_ns"]:
            previous["last_stamp_ns"] = stamp_ns
            previous["samples"] += 1
        self.complete = all(
            name in self.history
            and self.history[name]["samples"] >= rule["min_samples"]
            and self.history[name]["last_stamp_ns"] - self.history[name]["first_stamp_ns"]
            >= round(rule["min_span_sim_s"] * 1e9)
            for name, rule in self.requirements.items()
        )

    def observe(self, healthy, clock_ns, pose_stamp_ns):
        if not healthy or clock_ns is None or pose_stamp_ns is None:
            self.good_since = None
            self.first_pose = None
            self.last_pose = None
            self.samples = 0
            return False
        if self.good_since is None:
            self.good_since = clock_ns
        if self.last_pose is None or pose_stamp_ns > self.last_pose:
            self.samples += 1
            self.last_pose = pose_stamp_ns
            if self.first_pose is None:
                self.first_pose = pose_stamp_ns
        return (
            self.complete
            and self.samples >= self.recovery_samples
            and self.last_pose - self.first_pose >= self.recovery_span_ns
        )


def atomic(path, data):
    """Write JSON via a sibling ``.tmp`` file so readers never see partial text."""
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, allow_nan=False, ensure_ascii=False) + "\n")
    tmp.replace(path)


from embodied_navigation.common.shutdown import (
    ShutdownRequested as _ShutdownRequested,
    ShutdownSignals as _ShutdownSignals,
)


def _main(shutdown_signals):
    """Build the ROS gate node from the frozen run context and spin to shutdown."""
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from sensor_msgs.msg import Image, CameraInfo, Imu, PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    import numpy as np
    import yaml
    from sensor_msgs_py import point_cloud2

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    args, ros = parser.parse_known_args()
    run = args.run.resolve()
    from embodied_navigation.common.runtime_context import load as load_context

    profile = load_context(run / "runtime_context.json")["profile"]
    minimal_baseline = minimal_baseline_enabled(profile)
    actual_camera_rate = json.loads((run / "sensor_contract.json").read_text())["camera"]["rate_hz"]
    calib = yaml.safe_load((run / "navigation_camera.yaml").read_text())["/**"]["ros__parameters"]
    rclpy.init(args=ros, signal_handler_options=SignalHandlerOptions.NO)

    class Gate(Node):
        def __init__(self):
            super().__init__("teacher_navigation_sensor_gate")
            self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
            self.clock_ns = None
            self.clock_wall = None
            self.stamps = {}
            self.walls = {}
            self.counts = {}
            self.errors = {}
            self.pairs = ImageInfoPairs()
            self.failure = None
            self.last_report = None
            self.warmup = WarmupRecovery(
                profile["sensor_warmup_sim_s"],
                profile["recovery_fresh_pose_samples"],
                profile["recovery_min_pose_span_sim_s"],
                profile["warmup_history"],
            )
            self.evidence = RuntimeTelemetry()
            self.pub = self.create_publisher(String, TOPIC, 10)
            for name, typ, topic in [
                ("imu", Imu, "/livox/imu"),
                ("raw_lidar", PointCloud2, "/demo/teacher/raw_lidar"),
                ("image", Image, "/demo/camera"),
                ("camera_info", CameraInfo, "/demo/camera_info"),
                ("body_odom", Odometry, "/demo/slam/body_odom"),
                ("cloud", PointCloud2, "/cloud_registered_full"),
            ]:
                self.create_subscription(
                    typ, topic, lambda m, n=name: self.receive(n, m), latest_sensor_qos()
                )
            self.graph_check = BackgroundCheck(self.graph)
            self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def sync_clock(self):
            # Use rclpy's sole ROS TimeSource /clock(depth=1). A second custom
            # clock queue previously lagged 460ms behind the current ROS clock.
            ns = self.get_clock().now().nanoseconds
            if self.clock_ns is not None and ns < self.clock_ns:
                self.failure = "Simulation clock moved backward"
            if self.clock_ns is None or ns > self.clock_ns:
                self.clock_ns = ns
                self.clock_wall = time.monotonic()

        def receive(self, name, msg):
            self.sync_clock()
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if stamp <= self.stamps.get(name, -1):
                return
            error = None
            if name == "body_odom":
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                if (
                    msg.header.frame_id != "camera_init"
                    or msg.child_frame_id != "demo_slam_body"
                    or not all(math.isfinite(v) for v in values)
                    or not 0.98 <= sum(v * v for v in values[3:]) <= 1.02
                ):
                    error = "Invalid measured SLAM body pose/frame"
            elif name == "cloud":
                if msg.header.frame_id != "camera_init" or msg.width * msg.height < 20:
                    error = "Registered cloud is empty or in wrong frame"
                else:
                    try:
                        xyz = point_cloud2.read_points_numpy(
                            msg, field_names=("x", "y", "z"), skip_nans=True
                        )
                        if len(xyz) < 20 or not np.isfinite(xyz).all():
                            error = "Registered cloud XYZ invalid"
                    except (ValueError, TypeError):
                        error = "Registered cloud cannot be decoded"
            elif name == "image":
                if (
                    msg.header.frame_id != "demo_camera_optical_frame"
                    or msg.width != 640
                    or msg.height != 480
                    or not msg.data
                ):
                    error = "Vehicle image frame/dimensions invalid"
                else:
                    self.pairs.observe(
                        name, stamp, msg.header.frame_id, msg.width, msg.height, time.monotonic()
                    )
            elif name == "camera_info":
                expected = [
                    calib["cam_fx"],
                    0.0,
                    calib["cam_cx"],
                    0.0,
                    calib["cam_fy"],
                    calib["cam_cy"],
                    0.0,
                    0.0,
                    1.0,
                ]
                if (
                    msg.header.frame_id != "demo_camera_optical_frame"
                    or msg.width != 640
                    or msg.height != 480
                    or msg.distortion_model != "plumb_bob"
                    or len(msg.d) < 4
                    or any(abs(v) > 1e-12 for v in msg.d)
                    # Actual Gazebo float32 K quantization is 1.065e-5 px;
                    # this 2e-5 px tolerance does not change geometry bounds.
                    or not np.allclose(msg.k, expected, rtol=0, atol=2e-5)
                ):
                    error = "Actual vehicle CameraInfo does not match pinhole K/zero D"
                else:
                    self.pairs.observe(
                        name, stamp, msg.header.frame_id, msg.width, msg.height, time.monotonic()
                    )
            elif name == "raw_lidar":
                if msg.header.frame_id != "velodyne" or msg.width * msg.height < 20:
                    error = "Raw actual LiDAR invalid"
            elif name == "imu":
                values = [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ]
                if not all(math.isfinite(v) for v in values):
                    error = "Actual IMU nonfinite"
            if error:
                self.errors[name] = error
                return
            self.errors.pop(name, None)
            self.stamps[name] = stamp
            self.walls[name] = time.monotonic()
            self.counts[name] = self.counts.get(name, 0) + 1
            self.warmup.record(name, stamp)

        def graph(self):
            expected = {
                "/aft_mapped_to_init": "laserMapping",
                "/demo/slam/body_odom": "demo_slam_odom_adapter",
                "/cloud_registered_full": (
                    "demo_slam_odom_adapter"
                    if profile.get("known_scene_localization_contract", {}).get("enabled")
                    else "laserMapping"
                ),
                "/demo/slam/lidar_filtered": "demo_slam_self_echo_filter",
                "/demo/teacher/slam/image": "teacher_slam_actual_sensor_relay",
                "/demo/teacher/slam/imu": "teacher_slam_actual_sensor_relay",
            }
            results = {}
            if profile.get("known_scene_localization_contract", {}).get("enabled"):
                expected.update(
                    {
                        "/demo/slam/raw_body_odom": "demo_slam_raw_odom_adapter",
                        "/demo/slam/raw_registered_full": "laserMapping",
                    }
                )
            for topic, name in expected.items():
                infos = self.get_publishers_info_by_topic(topic)
                results[topic] = {
                    "expected": name,
                    "publishers": [i.node_namespace + "/" + i.node_name for i in infos],
                    "passed": len(infos) == 1
                    and infos[0].node_name == name
                    and infos[0].node_namespace == "/",
                }
            return results

        def tick(self):
            self.sync_clock()
            now = time.monotonic()
            reasons = []
            ages = {}
            ns = self.clock_ns
            wall_observations = []
            if self.evidence.error:
                self.failure = self.evidence.error
            if self.failure:
                reasons.append(self.failure)
            clock_age = None if self.clock_wall is None else now - self.clock_wall
            if (
                minimal_baseline
                and clock_age is not None
                and math.isfinite(clock_age)
                and clock_age >= 0.3
            ):
                wall_observations.append(
                    {"field": "clock_wall_s", "age_s": clock_age, "original_limit_s": 0.3}
                )
            if (
                self.clock_wall is None
                or (not minimal_baseline and clock_age >= 0.3)
                or (minimal_baseline and (not math.isfinite(clock_age) or clock_age < 0))
            ):
                reasons.append("Actual simulation clock stale")
            for name in ("imu", "raw_lidar", "image", "camera_info", "body_odom", "cloud"):
                wall_age = None if name not in self.walls else now - self.walls[name]
                sim_age = (
                    None
                    if ns is None or name not in self.stamps
                    else (ns - self.stamps[name]) / 1e9
                )
                ages[name] = {
                    "wall_s": wall_age,
                    "sim_s": sim_age,
                    "stamp_ns": self.stamps.get(name),
                }
                # 2 Hz RGB cannot use the 300ms motion feedback watchdog.
                wall_limit, sim_limit = (
                    ((2.0, 0.75) if actual_camera_rate < 10 else (0.3, 0.3))
                    if name in ("image", "camera_info")
                    else (0.3, 0.3)
                )
                if (
                    minimal_baseline
                    and wall_age is not None
                    and math.isfinite(wall_age)
                    and wall_age >= wall_limit
                ):
                    wall_observations.append(
                        {
                            "field": name + "_wall_s",
                            "age_s": wall_age,
                            "original_limit_s": wall_limit,
                        }
                    )
                wall_bad = (
                    wall_age is None
                    or not 0 <= wall_age
                    or not minimal_baseline
                    and wall_age >= wall_limit
                    or minimal_baseline
                    and not math.isfinite(wall_age)
                )
                if wall_bad or sim_age is None or not -0.05 <= sim_age < sim_limit:
                    reasons.append(name + " missing/stale")
                if name in self.errors:
                    reasons.append(self.errors[name])
            pair = self.pairs.latest_fresh(
                ns,
                now,
                sim_limit=0.75 if actual_camera_rate < 10 else 0.3,
                wall_limit=2.0 if actual_camera_rate < 10 else 0.3,
                observe_wall=minimal_baseline,
            )
            if pair is None:
                reasons.append("Vehicle image and actual CameraInfo are not paired within 50ms")
            checked = self.graph_check.result
            graph = {} if checked is None or checked["data"] is None else checked["data"]
            graph_age = None if checked is None else now - checked["started_wall"]
            if (
                minimal_baseline
                and graph_age is not None
                and math.isfinite(graph_age)
                and graph_age >= 0.3
            ):
                wall_observations.append(
                    {"field": "publisher_graph_wall_s", "age_s": graph_age, "original_limit_s": 0.3}
                )
            if (
                checked is None
                or checked["error"]
                or (not minimal_baseline and graph_age >= 0.3)
                or (minimal_baseline and (not math.isfinite(graph_age) or graph_age < 0))
            ):
                reasons.append("Actual SLAM publisher graph check missing/stale")
            for topic, value in graph.items():
                if not value["passed"]:
                    reasons.append("Unexpected actual SLAM publisher chain: " + topic)
            warmed = self.warmup.observe(not reasons, ns, self.stamps.get("body_odom"))
            if not warmed and not reasons:
                reasons.append(
                    "Waiting for fresh SLAM recovery samples"
                    if self.warmup.complete
                    else "Waiting for cumulative actual sensor SLAM initialization history"
                )
            status = {
                "schema": 1,
                "mode": profile["controller_kind"],
                "state": "failed" if self.failure else "ready" if warmed else "hold",
                "ready": warmed,
                "reason": "; ".join(reasons)
                if reasons
                else "Actual vehicle RGB/K, IMU, LiDAR and sensor SLAM healthy",
                "monotonic_wall": now,
                "ros_sim_time_ns": ns,
                "good_since_sim_ns": self.warmup.good_since,
                "ages": ages,
                "initial_warmup_complete": self.warmup.complete,
                "recovery_fresh_pose_samples": self.warmup.samples,
                "initialization_history": self.warmup.history,
                "initialization_requirements": self.warmup.requirements,
                "clock_source": "single rclpy ROS TimeSource /clock, KEEP_LAST depth=1",
                "graph_check_started_wall": None if checked is None else checked["started_wall"],
                "graph_check_duration_wall_s": None
                if checked is None
                else checked["completed_wall"] - checked["started_wall"],
                "recovery_first_pose_stamp_ns": self.warmup.first_pose,
                "recovery_last_pose_stamp_ns": self.warmup.last_pose,
                "actual_image_camera_info_pair": pair,
                "counts": self.counts,
                "publisher_graph": graph,
                "ground_truth_navigation_used": False,
                "overview_used_for_vio": False,
                "camera_intrinsics_scale": "Original640x480 K; FAST-LIVO2 internally applies scale=0.5",
            }
            if minimal_baseline:
                status["simulation_wall_age_observations"] = wall_observations
                if not reasons:
                    status["reason"] = (
                        "Simulation baseline: sensor structure/simulation-time valid; original wall ages observed"
                    )
            self.evidence.atomic(run / "navigation_sensor_gate.json", status)
            self.evidence.append(run / "navigation_sensor_gate_history.jsonl", status)
            self.pub.publish(String(data=json.dumps(status, allow_nan=False)))
            self.last_report = status

    node = Gate()
    try:
        rclpy.spin(node)
    except _ShutdownRequested:
        pass
    finally:
        shutdown_signals.begin_cleanup()
        if node.last_report:
            status = {
                **node.last_report,
                "ready": False,
                "state": "hold",
                "reason": "Sensor SLAM gate stopped",
                "monotonic_wall": time.monotonic(),
            }
            node.evidence.atomic(run / "navigation_sensor_gate.json", status)
        node.graph_check.close()
        node.evidence.close()
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    with _ShutdownSignals() as shutdown_signals:
        _main(shutdown_signals)


if __name__ == "__main__":
    main()
