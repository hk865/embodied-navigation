#!/usr/bin/env python3
"""Rebuild observed point layers using recorded LIO estimates, never archive GT poses."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sqlite3
import sys
from bisect import bisect_left
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from embodied_navigation.localization_mapping.semantic_mapping import (
    DYNAMIC,
    STATIC,
    UNKNOWN,
    LayeredMap,
    Observation,
    PointMask,
)

ODOM_TOPIC = "/dynanav/lio_observer/aft_mapped_to_init"
MASK_PROVENANCE = {
    "kind": "sim_oracle",
    "producer": "isaac.instance_id_segmentation",
    "instance_id_scope": "epoch",
}
MAX_NUMERIC_TOLERANCE_NS = 1000


@dataclass(frozen=True)
class OdomPose:
    source_ns: int | None
    transform: np.ndarray | None
    frame_id: str
    child_frame_id: str
    storage: str = ""
    message_id: int = 0
    bag_timestamp_ns: int = 0
    error: str | None = None


@dataclass(frozen=True)
class PoseMatch:
    pose: OdomPose | None
    status: str
    odom_source_ns: int | None = None
    delta_ns: int | None = None


def stamp_tolerance(value):
    """A tolerance is for numeric timestamp roundoff, never motion extrapolation."""
    if type(value) is not int or not 0 <= value <= MAX_NUMERIC_TOLERANCE_NS:
        raise ValueError(
            f"stamp tolerance must be integer 0..{MAX_NUMERIC_TOLERANCE_NS} ns"
        )
    return value


class PoseIndex:
    """Pair the nearest measured header stamp; reject invalid or ambiguous evidence."""

    def __init__(self, poses):
        self.groups = {}
        for pose in poses:
            if pose.source_ns is not None:
                self.groups.setdefault(pose.source_ns, []).append(pose)
        self.stamps = sorted(self.groups)

    def match(self, source_ns, tolerance_ns=0):
        stamp_tolerance(tolerance_ns)
        if type(source_ns) is not int or source_ns < 0:
            raise ValueError("keyframe source_ns must be a nonnegative integer")
        if not self.stamps:
            return PoseMatch(None, "unmatched_pose")
        insertion = bisect_left(self.stamps, source_ns)
        candidates = self.stamps[max(0, insertion - 1) : insertion + 1]
        distance = min(abs(stamp - source_ns) for stamp in candidates)
        nearest = [stamp for stamp in candidates if abs(stamp - source_ns) == distance]
        if distance > tolerance_ns:
            stamp = nearest[0]
            return PoseMatch(None, "unmatched_pose", stamp, stamp - source_ns)
        if len(nearest) != 1:
            return PoseMatch(None, "ambiguous_pose")
        stamp = nearest[0]
        group = self.groups[stamp]
        if len(group) != 1:
            return PoseMatch(None, "duplicate_pose_stamp", stamp, stamp - source_ns)
        pose = group[0]
        if pose.error is not None:
            return PoseMatch(None, pose.error, stamp, stamp - source_ns)
        return PoseMatch(pose, "matched", stamp, stamp - source_ns)


def odometry_pose(message, *, map_frame, sensor_frame, **origin):
    """Validate one decoded nav_msgs/Odometry without importing any ROS package."""
    stamp = message.header.stamp
    source_ns = None
    error = None
    if (
        type(stamp.sec) is not int
        or type(stamp.nanosec) is not int
        or stamp.sec < 0
        or not 0 <= stamp.nanosec < 1_000_000_000
    ):
        error = "invalid_odom_stamp"
    else:
        source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    frame, child = message.header.frame_id, message.child_frame_id
    if error is None and (frame != map_frame or child != sensor_frame):
        error = "frame_mismatch"
    transform = None
    if error is None:
        position, quaternion = message.pose.pose.position, message.pose.pose.orientation
        xyz = np.array([position.x, position.y, position.z], dtype=float)
        q = np.array(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=float
        )
        norm = float(np.linalg.norm(q))
        if (
            not np.isfinite(xyz).all()
            or not np.isfinite(q).all()
            or abs(norm - 1) > 1e-3
        ):
            error = "invalid_odom_pose"
        else:
            x, y, z, w = q / norm
            transform = np.eye(4)
            transform[:3, :3] = [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
            transform[:3, 3] = xyz
            transform.flags.writeable = False
    return OdomPose(source_ns, transform, frame, child, error=error, **origin)


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


def read_odometry(files, *, topic, map_frame, sensor_frame):
    """Read only the selected topic via SQLite; ROS dependencies are decoder-only."""
    try:
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message
    except ImportError as error:
        raise RuntimeError(
            "ROS decoder unavailable; source /opt/ros/jazzy/setup.bash first"
        ) from error
    poses, counts = [], Counter()
    previous = None
    for file in files:
        connection = sqlite3.connect(file.resolve().as_uri() + "?mode=ro", uri=True)
        try:
            topics = connection.execute(
                "SELECT id,type,serialization_format FROM topics WHERE name=?", (topic,)
            ).fetchall()
            if not topics:
                continue
            for topic_id, message_type, encoding in topics:
                if message_type != "nav_msgs/msg/Odometry" or encoding != "cdr":
                    raise ValueError(
                        f"selected topic must be CDR nav_msgs/msg/Odometry: {file}"
                    )
                rows = connection.execute(
                    "SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id",
                    (topic_id,),
                )
                for message_id, timestamp, data in rows:
                    counts["messages"] += 1
                    try:
                        message = deserialize_message(data, Odometry)
                        pose = odometry_pose(
                            message,
                            map_frame=map_frame,
                            sensor_frame=sensor_frame,
                            storage=str(file),
                            message_id=message_id,
                            bag_timestamp_ns=timestamp,
                        )
                    except (ValueError, TypeError, AttributeError, RuntimeError):
                        counts["decode_failures"] += 1
                        continue
                    poses.append(pose)
                    counts[pose.error or "valid_poses"] += 1
                    if pose.source_ns is not None:
                        if previous is not None and pose.source_ns < previous:
                            counts["backward_header_stamps"] += 1
                        previous = pose.source_ns
        finally:
            connection.close()
    if not counts["messages"]:
        raise ValueError(f"no odometry messages on {topic}")
    counts["duplicate_header_stamps"] = sum(
        len(group) - 1 for group in PoseIndex(poses).groups.values()
    )
    return poses, dict(counts)


def _scalar(archive, key):
    value = archive[key]
    if value.shape != ():
        raise ValueError(f"{key} must be scalar")
    return value.item()


def load_keyframe(path, *, sensor_frame, calibration_id, pedestrian_roots, max_points):
    """Read an explicit allowlist: the archive's simulator pose is never accessed."""
    sequence_match = re.fullmatch(r"rgbd_(\d+)\.npz", Path(path).name)
    if sequence_match is None:
        raise ValueError("unexpected keyframe filename")
    with np.load(path, allow_pickle=False) as archive:
        source_ns = _scalar(archive, "source_ns")
        if type(source_ns) is not int or source_ns < 0:
            raise ValueError("source_ns must be a nonnegative integer")
        if _scalar(archive, "semantic_oracle") is not True:
            raise ValueError("archive must explicitly identify semantic_oracle=true")
        for key, expected in (
            ("frame_id", sensor_frame),
            ("calibration_id", calibration_id),
        ):
            if key in archive and _scalar(archive, key) != expected:
                raise ValueError(f"archive {key} mismatch")
        points = np.array(archive["points_sensor"], dtype=float, copy=True)
        states = np.array(archive["states"], copy=True)
        paths = np.array(archive["instance_paths"], copy=True)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError("points_sensor must be finite Nx3")
        if len(points) > max_points:
            raise ValueError("keyframe exceeds configured point budget")
        if (
            states.shape != (len(points),)
            or not np.issubdtype(states.dtype, np.integer)
            or not np.isin(states, [UNKNOWN, STATIC, DYNAMIC]).all()
        ):
            raise ValueError("states must preserve per-point UNKNOWN/STATIC/DYNAMIC")
        if (
            paths.shape != (len(points),)
            or paths.dtype.kind != "U"
            or any(path and not path.startswith("/") for path in paths)
        ):
            raise ValueError(
                "instance_paths must be per-point absolute path strings or empty"
            )
        visible = set(paths) - {""}
        labels, label_source = {}, "legacy_generic_and_configured_pedestrian_roots"
        if "labels_json" in archive:
            labels = json.loads(_scalar(archive, "labels_json"))
            if not isinstance(labels, dict) or any(
                key not in visible or not isinstance(value, str) or not value.strip()
                for key, value in labels.items()
            ):
                raise ValueError(
                    "labels_json must map visible paths to recorded nonempty classes"
                )
            label_source = "recorded_labels_json"
        else:
            labels = {
                path: (
                    "person"
                    if any(
                        path == root or path.startswith(root + "/")
                        for root in pedestrian_roots
                    )
                    else "scene_geometry"
                )
                for path in visible
            }
        provenance, provenance_source = dict(MASK_PROVENANCE), "legacy_capture_contract"
        if "mask_provenance_json" in archive:
            recorded = json.loads(_scalar(archive, "mask_provenance_json"))
            if not isinstance(recorded, dict) or any(
                recorded.get(k) != v for k, v in MASK_PROVENANCE.items()
            ):
                raise ValueError(
                    "recorded mask provenance differs from instance oracle contract"
                )
            json.dumps(recorded, allow_nan=False)
            provenance.update(recorded)
            provenance_source = "recorded_mask_provenance_json"
    return {
        "sequence": int(sequence_match[1]),
        "source_ns": source_ns,
        "points": points,
        "states": states,
        "paths": tuple(paths),
        "labels": labels,
        "provenance": provenance,
        "label_source": label_source,
        "provenance_source": provenance_source,
    }


def integrate_keyframe(store, frame, match, *, epoch, sensor_frame, calibration_id):
    if match.status != "matched" or match.pose is None or match.pose.transform is None:
        raise ValueError("only a matched measured estimate may supply the pose")
    source_ns, sequence = frame["source_ns"], frame["sequence"]
    observation = Observation(
        epoch=epoch,
        sequence=sequence,
        source_ns=source_ns,
        pose_source_ns=source_ns,
        points_sensor=frame["points"],
        T_map_sensor=match.pose.transform,
        anchor_id=f"rgbd:{sequence}",
        frame_id=sensor_frame,
        calibration_id=calibration_id,
        pose_source="slam",
    )
    provenance = dict(
        frame["provenance"],
        label_source=frame["label_source"],
        mask_provenance_source=frame["provenance_source"],
        estimated_pose_header_ns=match.odom_source_ns,
        estimated_pose_stamp_delta_ns=match.delta_ns,
    )
    mask = PointMask(
        epoch=epoch,
        sequence=sequence,
        source_ns=source_ns,
        produced_ns=source_ns,
        frame_id=sensor_frame,
        calibration_id=calibration_id,
        states=frame["states"],
        instance_ids=frame["paths"],
        labels=frame["labels"],
        provenance=provenance,
    )
    return store.integrate(observation, mask, now_ns=source_ns)


def source_digest(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return {
        "path": str(Path(path).resolve()),
        "bytes": Path(path).stat().st_size,
        "sha256": digest.hexdigest(),
    }


def reconstruct(args):
    stamp_tolerance(args.stamp_tolerance_ns)
    keyframes = sorted(args.keyframes.resolve().glob("rgbd_*.npz"))
    if not keyframes:
        raise ValueError("no rgbd_*.npz keyframes")
    if args.output.exists():
        raise ValueError("output must be a new, nonexistent directory")
    databases = bag_files(args.bag)
    metadata = [path.resolve() for path in args.observer_metadata]
    input_paths = databases + keyframes + metadata
    digests = {path: source_digest(path) for path in input_paths}
    poses, odom_counts = read_odometry(
        databases,
        topic=args.odom_topic,
        map_frame=args.map_frame,
        sensor_frame=args.sensor_frame,
    )
    index = PoseIndex(poses)
    store = LayeredMap(
        map_id=args.map_id or args.output.name,
        frame_id=args.map_frame,
        voxel_size=args.voxel_size,
        max_observations=len(keyframes),
        max_points_per_observation=args.max_points,
        max_objects=args.max_objects,
    )
    records, totals, labels = [], Counter(), Counter()
    for path in keyframes:
        record = {"keyframe": path.name}
        try:
            frame = load_keyframe(
                path,
                sensor_frame=args.sensor_frame,
                calibration_id=args.calibration_id,
                pedestrian_roots=args.pedestrian_root,
                max_points=args.max_points,
            )
            record.update(sequence=frame["sequence"], source_ns=frame["source_ns"])
            match = index.match(frame["source_ns"], args.stamp_tolerance_ns)
            record.update(
                status=match.status,
                odom_source_ns=match.odom_source_ns,
                stamp_delta_ns=match.delta_ns,
            )
            if match.pose is not None:
                record.update(
                    odom_storage=match.pose.storage,
                    odom_message_id=match.pose.message_id,
                    bag_timestamp_ns=match.pose.bag_timestamp_ns,
                )
            if match.status == "matched":
                result = integrate_keyframe(
                    store,
                    frame,
                    match,
                    epoch=args.epoch,
                    sensor_frame=args.sensor_frame,
                    calibration_id=args.calibration_id,
                )
                record.update(
                    mask_status=result["mask_status"],
                    persisted=result["persisted"],
                    label_source=frame["label_source"],
                    mask_provenance_source=frame["provenance_source"],
                    point_counts={
                        key: len(result[key + "_points"])
                        for key in ("static", "dynamic", "unknown")
                    },
                )
                labels[frame["label_source"]] += 1
        except (ValueError, KeyError, OSError, TypeError) as error:
            record.update(status="invalid_keyframe", error=str(error))
        totals[record["status"]] += 1
        records.append(record)
    if not totals["matched"]:
        raise ValueError("no valid keyframe/estimated-pose matches; no map written")
    for path, before in digests.items():
        if source_digest(path) != before:
            raise ValueError(
                f"input changed during reconstruction; finalize capture/bag first: {path}"
            )
    if (
        sorted(args.keyframes.resolve().glob("rgbd_*.npz")) != keyframes
        or bag_files(args.bag) != databases
    ):
        raise ValueError(
            "input files changed during reconstruction; finalize recording first"
        )
    points = store.static_points()
    receipt = {
        "schema": "dynanav_estimated_pose_reconstruction/v1",
        "observer_label": args.observer_label,
        "observer_label_source": "user_supplied; inspect hashed observer metadata for actual build identity",
        "observer_metadata": [digests[path] for path in metadata],
        "odometry": {
            "topic": args.odom_topic,
            "frame_id": args.map_frame,
            "child_frame_id": args.sensor_frame,
            "timestamp_source": "message.header.stamp; bag receipt time is never a pose timestamp",
            "counts": odom_counts,
            "storage": [digests[path] for path in databases],
        },
        "keyframe_sources": [digests[path] for path in keyframes],
        "source_code": source_digest(Path(__file__)),
        "pairing": {
            "tolerance_ns": args.stamp_tolerance_ns,
            "interpolation": False,
            "extrapolation": False,
            "numeric_timestamp_policy": "exact by default; accepted numeric delta is canonicalized to sensor source_ns in Observation only; original odom stamps retained here and in mask provenance",
            "counts": dict(totals),
            "keyframes": records,
        },
        "pose_source": "slam",
        "mask_source": "sim_oracle",
        "no_GT_pose_used": True,
        "semantic_oracle": True,
        "learned_perception": False,
        "complete_full_scene": False,
        "localization_success_verified": False,
        "static_frontend_dynamic_filter_verified": False,
        "label_sources": dict(labels),
        "configured_legacy_pedestrian_roots": args.pedestrian_root,
        "epoch": args.epoch,
        "calibration_id": args.calibration_id,
        "static_map_points": len(points),
        "retained_observations": len(store.observations),
        "scope": "Offline reconstruction from measured estimated poses and recorded RGB-D point/mask evidence; no localization accuracy, full-scene coverage, route completion, or live control claim",
    }
    args.output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(
        args.output / "static_map.npz",
        points=points,
        pose_source="slam",
        mask_source="sim_oracle",
        voxel_size=store.voxel_size,
        map_frame=args.map_frame,
        no_GT_pose_used=np.bool_(True),
        semantic_oracle=np.bool_(True),
        complete_full_scene=np.bool_(False),
    )
    scene_path = args.output / "scene_map.json"
    snapshot = store.summary()
    snapshot.update(
        no_GT_pose_used=True,
        semantic_oracle=True,
        complete_full_scene=False,
        reconstruction_receipt="receipt.json",
        observer_label=args.observer_label,
    )
    scene_path.write_text(json.dumps(snapshot, ensure_ascii=False, allow_nan=False))
    receipt["outputs"] = [
        source_digest(args.output / name)
        for name in ("static_map.npz", "scene_map.json")
    ]
    (args.output / "receipt.json").write_text(
        json.dumps(receipt, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    )
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--keyframes", type=Path, required=True, help="capture/mapping/keyframes"
    )
    parser.add_argument(
        "--bag",
        type=Path,
        required=True,
        help="finalized SQLite ROS bag directory or .db3",
    )
    parser.add_argument(
        "--output", type=Path, required=True, help="new, nonexistent output directory"
    )
    parser.add_argument(
        "--observer-label",
        required=True,
        help="explicit actual run/build label, e.g. pre-time-origin-fix",
    )
    parser.add_argument(
        "--observer-metadata",
        type=Path,
        action="append",
        default=[],
        help="hash launch/build receipt (repeatable)",
    )
    parser.add_argument("--odom-topic", default=ODOM_TOPIC)
    parser.add_argument("--map-frame", default="dynanav_lio_map")
    parser.add_argument("--sensor-frame", default="go2_imu")
    parser.add_argument("--calibration-id", default="rgbd:v1")
    parser.add_argument(
        "--pedestrian-root",
        action="append",
        default=None,
        help="legacy label fallback root; repeat for multiple roots",
    )
    parser.add_argument(
        "--stamp-tolerance-ns",
        type=int,
        default=0,
        help="explicit numeric-roundoff allowance, 0..1000 ns; default exact",
    )
    parser.add_argument("--epoch", type=int, default=1)
    parser.add_argument("--map-id")
    parser.add_argument("--voxel-size", type=float, default=0.08)
    parser.add_argument("--max-points", type=int, default=8192)
    parser.add_argument("--max-objects", type=int, default=4096)
    args = parser.parse_args(argv)
    args.pedestrian_root = args.pedestrian_root or ["/World/DynamicPedestrian"]
    if any(
        not root.startswith("/") or root.endswith("/") for root in args.pedestrian_root
    ):
        parser.error(
            "pedestrian roots must be absolute prim paths without trailing slash"
        )
    if args.epoch < 1 or args.max_points < 1 or args.max_objects < 1:
        parser.error("epoch and capacity bounds must be positive")
    try:
        receipt = reconstruct(args)
    except (OSError, ValueError, RuntimeError, sqlite3.Error) as error:
        parser.error(str(error))
    print(
        json.dumps(
            {
                "output": str(args.output.resolve()),
                "counts": receipt["pairing"]["counts"],
                "static_map_points": receipt["static_map_points"],
                "no_GT_pose_used": True,
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
