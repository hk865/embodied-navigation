"""Live map-assisted localization; sensor validity and geometry, no audit receipts."""

import argparse, copy, json, time, uuid
from collections import OrderedDict
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from embodied_navigation.localization_mapping.known_scene_matcher import (
    SceneReference,
    KnownSceneMatcher,
    CorrectionState,
    transform_distance,
    checked_transform,
    policy_for_profile,
)
from embodied_navigation.localization_mapping.sensor_snapshot import (
    SensorSnapshots,
    BackgroundMatcher,
    stamp as stamp_ns,
    transform as odometry_transform,
)
from embodied_navigation.common.integration_limits import (
    KNOWN_BUNDLE_FRESHNESS_NS,
    KNOWN_BUNDLE_FRESHNESS_S,
)
from embodied_navigation.common.runtime_context import load


def atomic(path, value):
    """Write JSON through a same-directory temporary file, then replace the target."""
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, allow_nan=False) + "\n")
    temporary.replace(path)


def proposal_epoch_valid(result, generation, pending_target, transition):
    if not transition:
        return result.get("parent_generation") == generation
    if pending_target is None:
        return False
    distance, angle = transform_distance(np.asarray(result["C_world_odom"]), pending_target)
    return distance < 0.08 and angle < 0.05


def proposal_body_effect(result, applied):
    """Measure a correction on its captured body, independently of world origin.

    The worker preserves the exact body pose that produced C. Never use a latest
    or nearest pose to decide a transaction on an older matching result.
    """
    source = result.get("source_ns")
    if type(source) is not int or source <= 0 or result.get("T_odom_body_source_ns") != source:
        raise ValueError("Correction trigger requires the actual same-source body pose")
    raw = checked_transform(result["T_odom_body"])
    candidate = checked_transform(result["C_world_odom"])
    matched = checked_transform(result["T_world_body"])
    current = checked_transform(applied)
    proposed = candidate @ raw
    if not np.allclose(proposed, matched, rtol=0, atol=1e-7):
        raise ValueError("Correction body pose and candidate association inconsistent")
    before = current @ raw
    translation = float(np.linalg.norm(proposed[:3, 3] - before[:3, 3]))
    angle = float(Rotation.from_matrix(proposed[:3, :3] @ before[:3, :3].T).magnitude())
    return dict(
        schema="known_scene_same_source_body_correction_effect/v1",
        source_ns=source,
        translation_m=translation,
        rotation_rad=angle,
        translation_trigger_m=0.04,
        rotation_trigger_rad=0.025,
        requires_transaction=translation > 0.04 or angle > 0.025,
        origin_translation_diagnostic_m=float(np.linalg.norm(candidate[:3, 3] - current[:3, 3])),
        before_world_body_position=before[:3, 3].tolist(),
        after_world_body_position=proposed[:3, 3].tolist(),
        pose_association="exact_worker_captured_body_stamp",
        robot_truth_pose_used=False,
    )


def transaction_convergence(applied, target, captured_body, source_ns):
    """Require convergence at the exact body captured when this target froze."""
    if type(source_ns) is not int or source_ns <= 0:
        raise ValueError("Transaction release requires its frozen body source stamp")
    current = checked_transform(applied)
    goal = checked_transform(target)
    body = checked_transform(captured_body)
    origin, angle = transform_distance(current, goal)
    before = current @ body
    after = goal @ body
    residual = float(np.linalg.norm(after[:3, 3] - before[:3, 3]))
    return dict(
        schema="known_scene_same_source_transaction_convergence/v1",
        frozen_body_source_ns=source_ns,
        body_residual_m=residual,
        origin_residual_m=origin,
        rotation_residual_rad=angle,
        body_limit_m=0.005,
        original_origin_limit_m=0.005,
        original_rotation_limit_rad=0.003,
        converged=residual < 0.005 and origin < 0.005 and angle < 0.003,
        robot_truth_pose_used=False,
        pose_association="frozen_transaction_worker_body",
    )


def corrected_odom(msg, c):
    out = copy.deepcopy(msg)
    t = c @ odometry_transform(msg)
    q = Rotation.from_matrix(t[:3, :3]).as_quat()
    out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(
        float, t[:3, 3]
    )
    (
        out.pose.pose.orientation.x,
        out.pose.pose.orientation.y,
        out.pose.pose.orientation.z,
        out.pose.pose.orientation.w,
    ) = map(float, q)
    # Child/body twist is physical and unchanged; no derivative of correction C.
    cov = np.asarray(out.pose.covariance).reshape(6, 6)
    a = np.zeros((6, 6))
    a[:3, :3] = a[3:, 3:] = c[:3, :3]
    out.pose.covariance = (a @ cov @ a.T).reshape(-1).tolist()
    return out


def corrected_cloud(msg, c):
    out = copy.deepcopy(msg)
    payload = bytearray(msg.data)
    endian = ">" if msg.is_bigendian else "<"
    fields = {f.name: f for f in msg.fields}
    views = []
    for n in ("x", "y", "z"):
        f = fields[n]
        if f.datatype != 7 or f.count != 1:
            raise ValueError("Registered XYZ must remain float32")
        views.append(
            np.ndarray(
                (msg.height, msg.width),
                dtype=endian + "f4",
                buffer=payload,
                offset=f.offset,
                strides=(msg.row_step, msg.point_step),
            )
        )
    xyz = np.stack(views, -1)
    changed = xyz @ c[:3, :3].T + c[:3, 3]
    for k, v in enumerate(views):
        v[:] = changed[..., k]
    out.data = bytes(payload)
    return out


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    args, ros = parser.parse_known_args()
    directory = args.run.resolve()
    profile = load(directory / "runtime_context.json")["profile"]
    import embodied_navigation.perception.sensors.async_writers as r119_metadata_async

    r119_metadata_async.configure(directory)
    rclpy.init(args=ros)

    class Localization(Node):
        """ROS adapter: measured odometry/cloud in, corrected pose/cloud out, no map writes."""

        def __init__(self):
            super().__init__("demo_slam_odom_adapter")
            self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
            scenario = json.loads((directory / "navigation_scenario.json").read_text())
            origin = np.asarray(scenario["slam_origin_in_world"], float)
            self.nav_world = np.eye(4)
            self.nav_world[:3, 3] = -origin
            initial = np.eye(4)
            initial[:3, 3] = origin
            self.correction = CorrectionState(initial_c=initial)
            self.matcher = KnownSceneMatcher(
                SceneReference.load(directory / "known_scene_reference.json"),
                policy_for_profile(profile),
            )
            self.worker = BackgroundMatcher(self.matcher)
            self.snapshots = SensorSnapshots()
            self.slots = OrderedDict()
            self.initialized = False
            self.transition = False
            self.pending_target = None
            self.pending_body = None
            self.pending_source_ns = None
            self.stop_since_ns = None
            self.last_submit = 0
            self.last_published = 0
            self.bundle_count = 0
            self.transaction_count = 0
            self.last_result = None
            self.publisher_epoch = uuid.uuid4().hex
            self.pubs = {
                name: self.create_publisher(Odometry, "/demo/slam/" + name + "_odom", 10)
                for name in ["body", "lidar"]
            }
            self.cloud_pub = self.create_publisher(
                PointCloud2, "/cloud_registered_full", qos_profile_sensor_data
            )
            self.bundle_pub = self.create_publisher(
                String, "/demo/navigation/localization_bundle", 10
            )
            self.status_pub = self.create_publisher(
                String, "/demo/navigation/localization_status", 10
            )
            self.pubs["status"] = self.status_pub
            for role, message, topic in [
                ("body", Odometry, "/demo/slam/raw_body_odom"),
                ("lidar", Odometry, "/demo/slam/raw_lidar_odom"),
                ("registered", PointCloud2, "/demo/slam/raw_registered_full"),
            ]:
                self.create_subscription(
                    message,
                    topic,
                    lambda data, key=role: self.receive(key, data),
                    qos_profile_sensor_data,
                )
            self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            r119_metadata_async.verify(self, directory)

        def idle_or_stopped(self, body, initial=False):
            try:
                status = json.loads((directory / "navigation_status.json").read_text())
            except (OSError, ValueError):
                return False
            if initial:
                return status.get("state") == "idle" and self.bundle_count == 0
            try:
                command = json.loads((directory / "navigation_command.json").read_text())
            except (OSError, ValueError):
                return False
            values = command.get("command_body", command.get("command"))
            v = body.twist.twist.linear
            w = body.twist.twist.angular
            stopped = (
                0 <= time.monotonic() - float(command.get("monotonic_wall", 0)) < 0.3
                and isinstance(values, list)
                and len(values) == 3
                and not any(values)
                and not any(command.get("requested", [1, 1, 1]))
                and np.linalg.norm([v.x, v.y, v.z]) < 0.08
                and abs(w.z) < 0.1
            )
            if not stopped:
                self.stop_since_ns = None
                return False
            source = stamp_ns(body)
            if self.stop_since_ns is None:
                self.stop_since_ns = source
            return source - self.stop_since_ns >= 100_000_000

        def receive(self, role, message):
            source = stamp_ns(message)
            clock = self.get_clock().now().nanoseconds
            self.snapshots.put(role, message)
            if source <= self.last_published:
                return
            slot = self.slots.setdefault(source, {})
            slot[role] = (message, time.monotonic())
            while len(self.slots) > 12:
                self.slots.popitem(last=False)
            if not all(key in slot for key in ["body", "lidar", "registered"]):
                return
            if (
                not self.initialized
                or not -50_000_000 <= clock - source < KNOWN_BUNDLE_FRESHNESS_NS
            ):
                return
            if any(
                not 0 <= time.monotonic() - wall < KNOWN_BUNDLE_FRESHNESS_S
                for _, wall in slot.values()
            ):
                return
            body, lidar, cloud = (slot[key][0] for key in ["body", "lidar", "registered"])
            if (
                any(stamp_ns(value) != source for value in [body, lidar, cloud])
                or body.header.frame_id != cloud.header.frame_id
            ):
                return
            # A map correction is applied only while actual commands and motion
            # are stopped. Source timestamps and physical twist are unchanged.
            if self.transition and not self.idle_or_stopped(body):
                target = self.correction.target.copy()
                self.correction.target = self.correction.applied.copy()
                snapshot = self.correction.tick(source)
                self.correction.target = target
            else:
                snapshot = self.correction.tick(source)
            if self.transition and not snapshot["correction_stale"]:
                convergence = transaction_convergence(
                    self.correction.applied,
                    self.pending_target,
                    self.pending_body,
                    self.pending_source_ns,
                )
                if convergence["converged"] and self.idle_or_stopped(body):
                    self.correction.target = self.correction.applied.copy()
                    self.transition = False
                    self.transaction_count += 1
            correction = self.nav_world @ np.asarray(snapshot["C_world_odom"])
            corrected_body = corrected_odom(body, correction)
            corrected_lidar = corrected_odom(lidar, correction)
            corrected_points = corrected_cloud(cloud, correction)
            metadata = dict(
                schema="known_scene_navigation_bundle/v1",
                run_id=directory.name,
                source_ns=source,
                generation=snapshot["generation"],
                C_world_odom=snapshot["C_world_odom"],
                T_navigation_world=self.nav_world.tolist(),
                match_source_ns=snapshot["match_source_ns"],
                correction_stale=snapshot["correction_stale"],
                correction_transition=self.transition,
                transaction_count=self.transaction_count,
                control_allowed=not self.transition and not snapshot["correction_stale"],
                exact_body_lidar_registered_stamp=True,
                known_scene_map_assisted_navigation=True,
                robot_truth_pose_used=False,
                frontend_state_reset=False,
                frontend_velocity_or_bias_repaired=False,
                source_commit=dict(
                    producer_epoch=self.publisher_epoch,
                    commit=source,
                    stamp_ns=source,
                    explicit_message_commit_token=True,
                    stage="post_lio_dense",
                ),
            )
            self.bundle_pub.publish(String(data=json.dumps(metadata, allow_nan=False)))
            self.pubs["body"].publish(corrected_body)
            self.pubs["lidar"].publish(corrected_lidar)
            self.cloud_pub.publish(corrected_points)
            self.last_published = source
            self.bundle_count += 1
            for ns in list(self.slots):
                if ns <= source:
                    del self.slots[ns]

        def tick(self):
            clock = self.get_clock().now().nanoseconds
            result = self.worker.poll()
            if result is not None:
                self.last_result = result
                if result.get("accepted") is True:
                    latest = self.slots.get(max(self.slots), {}) if self.slots else {}
                    body = latest.get("body", (None,))[0]
                    candidate = checked_transform(result["C_world_odom"])
                    if not self.initialized:
                        if body is not None and self.idle_or_stopped(body, initial=True):
                            self.correction.propose(result)
                            self.correction.applied = candidate.copy()
                            self.correction.generation += 1
                            self.initialized = True
                            self.transaction_count += 1
                    elif proposal_epoch_valid(
                        result, self.correction.generation, self.pending_target, self.transition
                    ):
                        self.correction.last_match_ns = result["source_ns"]
                        effect = proposal_body_effect(result, self.correction.applied)
                        try:
                            state = json.loads(
                                (directory / "navigation_status.json").read_text()
                            ).get("state")
                        except (OSError, ValueError):
                            state = None
                        if (
                            not self.transition
                            and state == "running"
                            and effect["requires_transaction"]
                        ):
                            self.correction.target = candidate.copy()
                            self.pending_target = candidate.copy()
                            self.pending_body = checked_transform(result["T_odom_body"])
                            self.pending_source_ns = result["source_ns"]
                            self.stop_since_ns = None
                            self.transition = True
            if clock - self.last_submit >= 1_000_000_000:
                self.worker.submit(
                    self.snapshots.pop_latest(clock),
                    self.correction.applied,
                    self.correction.generation,
                )
                self.last_submit = clock
            status = dict(
                schema="known_scene_navigation_status/v1",
                run_id=directory.name,
                initialized=self.initialized,
                generation=self.correction.generation,
                correction_transition=self.transition,
                transaction_count=self.transaction_count,
                bundles_published=self.bundle_count,
                last_match_source_ns=self.correction.last_match_ns,
                last_result_reason=None
                if self.last_result is None
                else self.last_result.get("reason"),
                known_scene_map_assisted_navigation=True,
                robot_truth_pose_used=False,
                monotonic_wall=time.monotonic(),
                ros_clock_ns=clock,
                error=None,
            )
            self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
            atomic(directory / "known_scene_status.json", status)

    node = Localization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.worker.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
