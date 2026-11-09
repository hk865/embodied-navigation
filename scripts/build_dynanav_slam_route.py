#!/usr/bin/env python3
"""Create a route reference from recorded estimated Odometry pose measurements.

ROS is imported lazily for CDR decoding. The SQLite source is opened read-only;
no simulator state, image archives, native poses, or truth transforms are read.
The final stopped endpoint is confirmed by an explicit measured-pose window,
which is separate from the estimator's recorded twist output.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import math
import sqlite3
from dataclasses import dataclass
from pathlib import Path

ODOM_TOPIC = "/dynanav/lio_observer/aft_mapped_to_init"
MAP_FRAME = "dynanav_lio_map"
SENSOR_FRAME = "go2_imu"
POSE_SOURCES = ("lio_estimate", "livo_post_vio")


def _finite_vector(values, size, name):
    if len(values) != size or any(
        isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x)
        for x in values
    ):
        raise ValueError(f"{name} must be a finite {size}-vector")
    return [float(value) for value in values]


def _positive(value, name):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0
    ):
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


@dataclass(frozen=True)
class MeasuredPose:
    stamp_ns: int
    position_xyz_m: list[float]
    quaternion_wxyz: list[float]
    estimated_velocity_xyz_mps: list[float] | None
    estimated_yaw_rate_rps: float | None
    storage: str
    message_id: int
    bag_timestamp_ns: int

    @property
    def twist_available(self):
        return self.estimated_velocity_xyz_mps is not None

    def receipt(self):
        return {
            "stamp_ns": self.stamp_ns,
            "position_xyz_m": self.position_xyz_m,
            "quaternion_wxyz": self.quaternion_wxyz,
            "estimated_velocity_xyz_mps": self.estimated_velocity_xyz_mps,
            "estimated_yaw_rate_rps": self.estimated_yaw_rate_rps,
            "twist_available": self.twist_available,
            "source_message": {
                "storage": self.storage,
                "message_id": self.message_id,
                "bag_timestamp_ns": self.bag_timestamp_ns,
            },
        }


def odometry_pose(
    message,
    *,
    map_frame,
    sensor_frame,
    storage="",
    message_id=0,
    bag_timestamp_ns=0,
    pose_only=False,
):
    """Read only admitted fields; pose-only sources never access ``twist``."""
    if type(pose_only) is not bool:
        raise ValueError("pose_only must be boolean")
    stamp = message.header.stamp
    if (
        type(stamp.sec) is not int
        or type(stamp.nanosec) is not int
        or stamp.sec < 0
        or not 0 <= stamp.nanosec < 1_000_000_000
    ):
        raise ValueError("invalid odometry header stamp")
    if message.header.frame_id != map_frame or message.child_frame_id != sensor_frame:
        raise ValueError("odometry frame mismatch")
    p, q = message.pose.pose.position, message.pose.pose.orientation
    position = _finite_vector([p.x, p.y, p.z], 3, "estimated position")
    # nav_msgs stores x,y,z,w; the reference and controller require w,x,y,z.
    quaternion = _finite_vector([q.w, q.x, q.y, q.z], 4, "estimated quaternion wxyz")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError("estimated quaternion must have unit norm")
    quaternion = [value / norm for value in quaternion]
    velocity, yaw_rate = None, None
    if not pose_only:
        v, omega = message.twist.twist.linear, message.twist.twist.angular
        velocity = _finite_vector([v.x, v.y, v.z], 3, "estimated velocity")
        angular = _finite_vector(
            [omega.x, omega.y, omega.z], 3, "estimated angular velocity"
        )
        yaw_rate = angular[2]
    return MeasuredPose(
        stamp.sec * 1_000_000_000 + stamp.nanosec,
        position,
        quaternion,
        velocity,
        yaw_rate,
        storage,
        message_id,
        bag_timestamp_ns,
    )


def bag_files(path):
    path = Path(path).resolve()
    files = sorted(path.glob("*.db3")) if path.is_dir() else [path]
    if not files or any(not file.is_file() or file.suffix != ".db3" for file in files):
        raise ValueError(
            "bag must be a SQLite .db3 file or directory containing .db3 files"
        )
    for file in files:
        wal = Path(str(file) + "-wal")
        if wal.exists() and wal.stat().st_size:
            raise ValueError(
                f"bag has a live/uncheckpointed WAL; finalize recording first: {file}"
            )
    return files


def _decoder():
    try:
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message
    except ImportError as error:
        raise RuntimeError(
            "ROS decoder unavailable; source /opt/ros/jazzy/setup.bash first"
        ) from error
    return lambda data: deserialize_message(data, Odometry)


def read_odometry(files, *, topic, map_frame, sensor_frame, pose_only=False):
    """Decode only the selected estimated Odometry topic, preserving correspondence."""
    decode = _decoder()
    poses = []
    for file in files:
        connection = sqlite3.connect(file.resolve().as_uri() + "?mode=ro", uri=True)
        try:
            connection.execute("PRAGMA query_only=ON")
            topics = connection.execute(
                "SELECT id,type,serialization_format FROM topics WHERE name=?", (topic,)
            ).fetchall()
            if len(topics) > 1:
                raise ValueError(f"ambiguous selected topic in {file}")
            for topic_id, message_type, encoding in topics:
                if message_type != "nav_msgs/msg/Odometry" or encoding != "cdr":
                    raise ValueError("selected topic must be CDR nav_msgs/msg/Odometry")
                for message_id, timestamp, data in connection.execute(
                    "SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id",
                    (topic_id,),
                ):
                    try:
                        pose = odometry_pose(
                            decode(data),
                            map_frame=map_frame,
                            sensor_frame=sensor_frame,
                            storage=str(file),
                            message_id=message_id,
                            bag_timestamp_ns=timestamp,
                            pose_only=pose_only,
                        )
                    except (
                        ValueError,
                        TypeError,
                        AttributeError,
                        RuntimeError,
                    ) as error:
                        raise ValueError(
                            f"invalid selected odometry {file}:{message_id}: {error}"
                        ) from error
                    poses.append(pose)
        finally:
            connection.close()
    if not poses:
        raise ValueError(f"no measured odometry on {topic}")
    # Check recording order before sorting: repeated or regressing source stamps
    # cannot silently produce a different anchor or goal correspondence.
    poses.sort(key=lambda pose: (pose.bag_timestamp_ns, pose.storage, pose.message_id))
    for before, after in itertools.pairwise(poses):
        if after.stamp_ns <= before.stamp_ns:
            raise ValueError("duplicate or backward measured odometry header stamps")
    return poses


def _yaw(quaternion):
    w, x, y, z = quaternion
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def confirm_stopped(
    poses, goal_index, *, dwell_s, position_range_m, yaw_range_rad, max_sample_gap_s
):
    """Check the endpoint against all measured poses spanning the requested dwell.

    The witness includes the closest preceding sample at/before the dwell start;
    no time interpolation, velocity extrapolation, or truth stop flag is used.
    """
    goal = poses[goal_index]
    start = goal_index
    dwell_ns = round(dwell_s * 1e9)
    while start > 0 and goal.stamp_ns - poses[start].stamp_ns < dwell_ns:
        start -= 1
    window = poses[start : goal_index + 1]
    duration_s = (goal.stamp_ns - window[0].stamp_ns) / 1e9
    if duration_s < dwell_s:
        raise ValueError("goal lacks the required measured stop dwell")
    maximum_sample_gap = max(
        (after.stamp_ns - before.stamp_ns) / 1e9
        for before, after in itertools.pairwise(window)
    )
    if maximum_sample_gap > max_sample_gap_s:
        raise ValueError("goal stop window contains a gap in measured pose evidence")
    maximum_position_range = max(
        math.dist(pose.position_xyz_m, goal.position_xyz_m) for pose in window
    )
    goal_yaw = _yaw(goal.quaternion_wxyz)
    maximum_yaw_range = max(
        abs(
            math.atan2(
                math.sin(_yaw(pose.quaternion_wxyz) - goal_yaw),
                math.cos(_yaw(pose.quaternion_wxyz) - goal_yaw),
            )
        )
        for pose in window
    )
    if maximum_position_range > position_range_m or maximum_yaw_range > yaw_range_rad:
        raise ValueError(
            "selected endpoint is not stopped within the measured pose window"
        )
    return {
        "method": "estimated_pose_window"
        if goal.twist_available
        else "measured_pose_window",
        "start_stamp_ns": window[0].stamp_ns,
        "end_stamp_ns": goal.stamp_ns,
        "sample_count": len(window),
        "duration_s": duration_s,
        "maximum_sample_gap_s": maximum_sample_gap,
        "maximum_distance_from_endpoint_m": maximum_position_range,
        "maximum_yaw_difference_from_endpoint_rad": maximum_yaw_range,
        "estimated_twist_speed_at_endpoint_mps": (
            math.sqrt(sum(x * x for x in goal.estimated_velocity_xyz_mps))
            if goal.twist_available
            else None
        ),
        "twist_available": goal.twist_available,
        "twist_used_to_confirm_stop": False,
        "parameters": {
            "dwell_s": dwell_s,
            "position_range_m": position_range_m,
            "yaw_range_rad": yaw_range_rad,
            "max_sample_gap_s": max_sample_gap_s,
        },
    }


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_reference(args):
    output = Path(args.output).resolve()
    if output.exists():
        raise ValueError("output must be a new nonexistent file")
    for key in ("odom_topic", "map_frame", "sensor_frame"):
        if not isinstance(getattr(args, key), str) or not getattr(args, key).strip():
            raise ValueError(f"{key} must be explicit and nonempty")
    pose_source = getattr(args, "pose_source", "lio_estimate")
    if pose_source not in POSE_SOURCES:
        raise ValueError(f"pose source must be one of {POSE_SOURCES}")
    pose_only = getattr(args, "pose_only", False)
    if type(pose_only) is not bool:
        raise ValueError("pose_only must be boolean")
    # Native/derived post-VIO messages explicitly have unavailable twist. Do not
    # inspect or infer availability from their placeholder values or covariance.
    pose_only = pose_only or pose_source == "livo_post_vio"
    dwell = _positive(args.stop_dwell_s, "stop dwell")
    position_range = _positive(args.stop_position_range_m, "stop position range")
    yaw_range = _positive(args.stop_yaw_range_rad, "stop yaw range")
    max_sample_gap = _positive(args.stop_max_sample_gap_s, "stop max sample gap")
    files = bag_files(args.bag)
    # Hash before/after reading to detect source mutation during construction.
    digests_before = [sha256_file(file) for file in files]
    metadata_files = [
        Path(path).resolve() for path in getattr(args, "source_receipt", [])
    ]
    metadata_digests_before = [sha256_file(path) for path in metadata_files]
    poses = read_odometry(
        files,
        topic=args.odom_topic,
        map_frame=args.map_frame,
        sensor_frame=args.sensor_frame,
        pose_only=pose_only,
    )
    if len(poses) < 2:
        raise ValueError("at least an anchor and a later measured goal are required")
    requested = args.goal_stamp_ns
    if requested:
        if any(type(value) is not int or value < 0 for value in requested):
            raise ValueError("goal stamps must be nonnegative integer nanoseconds")
        if any(after <= before for before, after in itertools.pairwise(requested)):
            raise ValueError("goal stamps must be strictly increasing")
        index = {pose.stamp_ns: i for i, pose in enumerate(poses)}
        try:
            goal_indices = [index[value] for value in requested]
        except KeyError as error:
            raise ValueError(
                f"goal stamp has no exact measured odometry correspondence: {error.args[0]}"
            ) from error
    else:
        goal_indices = [len(poses) - 1]
    if goal_indices[0] == 0:
        raise ValueError("goal must be later than the measured anchor")
    selected = []
    for i in goal_indices:
        pose = poses[i]
        if (
            math.dist(
                poses[0].position_xyz_m[:2]
                if len(selected) == 0
                else selected[-1]["position_xyz_m"][:2],
                pose.position_xyz_m[:2],
            )
            < 1e-6
        ):
            raise ValueError("route requires nonzero measured XY segments")
        receipt = pose.receipt()
        receipt["stop_evidence"] = confirm_stopped(
            poses,
            i,
            dwell_s=dwell,
            position_range_m=position_range,
            yaw_range_rad=yaw_range,
            max_sample_gap_s=max_sample_gap,
        )
        selected.append(receipt)
    digests_after = [sha256_file(file) for file in files]
    if digests_before != digests_after:
        raise ValueError("source bag changed while constructing reference")
    metadata_digests_after = [sha256_file(path) for path in metadata_files]
    if metadata_digests_before != metadata_digests_after:
        raise ValueError("source metadata changed while constructing reference")
    reference = {
        "version": 1,
        "pose_source": pose_source,
        "no_gt_pose_used": True,
        "repeat_spawn_assumption": True,
        "map_registration_enabled": False,
        "localization_scope": "repeat_spawn_lab_assumption_not_general_global_localization",
        "source": {
            "bag": str(Path(args.bag).resolve()),
            "files": [
                {"path": str(file), "sha256": digest}
                for file, digest in zip(files, digests_after)
            ],
            "topic": args.odom_topic,
            "map_frame": args.map_frame,
            "sensor_frame": args.sensor_frame,
            "pose_stage": "post_vio" if pose_source == "livo_post_vio" else "lio",
            "pose_only": pose_only,
            "twist_available": not pose_only,
            "metadata_files": [
                {"path": str(path), "sha256": digest}
                for path, digest in zip(metadata_files, metadata_digests_after)
            ],
            "message_count": len(poses),
            "first_stamp_ns": poses[0].stamp_ns,
            "last_stamp_ns": poses[-1].stamp_ns,
        },
        "anchor": poses[0].receipt(),
        "goals": selected,
        "goal_selection": {
            "method": "exact_header_stamps" if requested else "final_measured_pose",
            "requested_stamp_ns": requested or [],
        },
    }
    encoded = json.dumps(reference, indent=2, allow_nan=False) + "\n"
    output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive create is the final guard against accidental replacement.
    with output.open("x", encoding="utf-8") as stream:
        stream.write(encoded)
    return reference


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--bag", type=Path, required=True)
    result.add_argument("--output", type=Path, required=True)
    result.add_argument("--odom-topic", default=ODOM_TOPIC)
    result.add_argument("--map-frame", default=MAP_FRAME)
    result.add_argument("--sensor-frame", default=SENSOR_FRAME)
    result.add_argument(
        "--pose-source",
        choices=POSE_SOURCES,
        default="lio_estimate",
        help="Explicit estimator stage; livo_post_vio always uses pose-only fields",
    )
    result.add_argument(
        "--pose-only",
        action="store_true",
        help="Never read unavailable twist; stop evidence comes only from measured pose windows",
    )
    result.add_argument(
        "--source-receipt",
        type=Path,
        action="append",
        default=[],
        help="Hash an actual export/launch provenance receipt (repeatable; not used as pose data)",
    )
    result.add_argument(
        "--goal-stamp-ns",
        type=int,
        action="append",
        help="exact stopped measured header stamp; may repeat in route order",
    )
    result.add_argument("--stop-dwell-s", type=float, default=1.0)
    result.add_argument("--stop-position-range-m", type=float, default=0.03)
    result.add_argument("--stop-yaw-range-rad", type=float, default=0.03)
    result.add_argument("--stop-max-sample-gap-s", type=float, default=0.25)
    return result


def main():
    args = parser().parse_args()
    reference = build_reference(args)
    print(
        json.dumps(
            {
                "output": str(args.output.resolve()),
                "anchor_stamp_ns": reference["anchor"]["stamp_ns"],
                "goal_stamps_ns": [goal["stamp_ns"] for goal in reference["goals"]],
                "pose_source": reference["pose_source"],
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
