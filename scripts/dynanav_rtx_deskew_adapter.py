#!/usr/bin/env python3
"""LEGACY read-only adapter; production uses rtx_deskewed_perception instead.

Superseded by src/embodied_navigation/simulation/dynanav/rtx_deskewed_perception.py
(native /cloud_registered_full + same-stage /aft_mapped_to_init, exact integer
stamp pair). Nothing in the production capture/map path imports this file; it is
kept only as a historical helper. Its DESKEWED_TOPICS entry names the coloured
camera-FOV /cloud_registered topic and must not be used as the navigation cloud.

Read-only adapter from a deskewed map-frame LiDAR cloud to base-frame evidence.

This module does not publish, write perception.json or mutate the operator map.
It defines the minimum contract needed to consume FAST-LIVO2's deskewed
/cloud_registered cloud (coloured, map/odom frame, stamp = LIO reference time)
plus the paired post-VIO pose:

* the raw rotating scan topic /dynanav/lidar_rtx is REJECTED: a rolling scan has
  per-point times and must not be treated as one rigid cloud at the header pose;
* deskew must be explicit (deskewed=True) and the cloud frame must equal the
  odometry/map frame; the pairing window stays the existing strict 0.10 s and is
  never extrapolated;
* map-frame points -> base frame: p_imu = R^T (p_map - t_map_base) with the full
  post-VIO quaternion, then p_base = p_imu - imu_mount (measured pure-translation
  mount). The IMU is never relabelled as the base.
* LiDAR perception uses direction coverage and point counts, never a
  depth_valid_ratio; unknown directions stay unknown and block the matching
  translation.
"""
import math

import numpy as np

RAW_ROTATING_TOPIC = "/dynanav/lidar_rtx"
DESKEWED_TOPICS = ("/cloud_registered",)
MAX_PAIR_DT_S = 0.10
DIRECTIONS = ("front", "left", "rear", "right")


class AdapterError(ValueError):
    """Inputs cannot be turned into defensible deskewed evidence."""


def validate_cloud(source_topic, header_frame, odom_frame, *, deskewed, mode):
    """Refuse raw rotating clouds, non-deskew or cross-stage clouds."""
    if source_topic == RAW_ROTATING_TOPIC:
        raise AdapterError("raw rotating LiDAR requires native deskew; refusing per-point-times scan")
    if source_topic not in DESKEWED_TOPICS:
        raise AdapterError("unsupported deskewed cloud topic: %r" % (source_topic,))
    if deskewed is not True:
        raise AdapterError("cloud is not explicitly marked as deskewed")
    if str(mode) not in ("LIO", "VIO"):
        raise AdapterError("unsupported estimator stage: %r" % (mode,))
    if not header_frame or header_frame != odom_frame:
        raise AdapterError("cloud frame %r does not match the odometry frame %r"
                           % (header_frame, odom_frame))
    return {"source_topic": source_topic, "frame": header_frame, "deskewed": True,
            "estimator_stage": str(mode)}


def validate_pairing(cloud_stamp_s, pose_stamp_s, *, max_pair_dt_s=MAX_PAIR_DT_S):
    for name, value in (("cloud_stamp_s", cloud_stamp_s), ("pose_stamp_s", pose_stamp_s)):
        if value is None or not math.isfinite(float(value)):
            raise AdapterError("%s must be finite" % name)
    dt = float(pose_stamp_s) - float(cloud_stamp_s)
    if abs(dt) > float(max_pair_dt_s):
        raise AdapterError("post-VIO pose is %.6f s from the cloud reference; the strict "
                           "%.2f s pairing window is not extrapolated" % (dt, float(max_pair_dt_s)))
    return float(dt)


def map_cloud_to_base(points_map, position_map, quat_xyzw, imu_mount_xyz):
    """Transform deskewed map-frame points into the base frame exactly once."""
    points = np.asarray(points_map, dtype=float)
    if points.ndim != 2 or points.shape[1] < 3:
        raise AdapterError("points must be a Nx3+ array")
    if not np.all(np.isfinite(points[:, :3])):
        raise AdapterError("cloud contains non-finite points")
    position = np.asarray(position_map, dtype=float)
    quat = np.asarray(quat_xyzw, dtype=float)
    mount = np.asarray(imu_mount_xyz, dtype=float)
    if position.shape != (3,) or quat.shape != (4,) or mount.shape != (3,):
        raise AdapterError("pose, quaternion and mount must be 3/4/3 element vectors")
    if not (np.all(np.isfinite(position)) and np.all(np.isfinite(quat)) and np.all(np.isfinite(mount))):
        raise AdapterError("non-finite pose, quaternion or mount")
    x, y, z, w = quat
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise AdapterError("degenerate quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    relative = points[:, :3] - position[:3]
    points_imu = relative @ rotation  # R^T applied to the column vectors
    points_base = points_imu + mount[:3]
    return points_base


def direction_evidence(
    points_base,
    *,
    base_to_floor_m=0.35,
    floor_band_m=0.10,
    collision_low_m=0.10,
    collision_high_m=1.20,
    corridor_half_rad=math.pi / 4.0,
    corridor_bins=4,
    corridor_length_m=3.0,
    min_points_per_bin=20,
):
    """Per-direction collision coverage and floor support inside the corridor.

    Floor-height points (near the measured floor) are NOT obstacles; collision
    heights are [collision_low_m, collision_high_m] above the floor. A direction
    is only 'covered' when every longitudinal bin of its corridor sector (not
    just the total point count) has at least min_points_per_bin collision-height
    points. Dense points in one narrow azimuth therefore do not cover the
    corridor, and floor support inside the corridor is reported separately so a
    missing floor, step or drop is a hold rather than a guess.
    """
    points = np.asarray(points_base, dtype=float)
    if points.ndim != 2 or points.shape[1] < 3 or points.shape[0] == 0:
        raise AdapterError("no base-frame points to evaluate")
    if not np.all(np.isfinite(points[:, :3])):
        raise AdapterError("non-finite base-frame points")
    if not (corridor_bins >= 2 and min_points_per_bin >= 1
            and corridor_length_m > 0.0 and floor_band_m >= 0.0):
        raise AdapterError("invalid corridor geometry")
    floor_z = -float(base_to_floor_m)
    floor_low = floor_z - float(floor_band_m)
    floor_high = floor_z + float(floor_band_m)
    xy = points[:, :2]
    ranges = np.linalg.norm(xy, axis=1)
    z = points[:, 2]
    angles = np.arctan2(xy[:, 1], xy[:, 0])
    floor_mask = (z >= floor_low) & (z <= floor_high) & (ranges <= float(corridor_length_m))
    collision_mask = (
        (z >= floor_z + float(collision_low_m))
        & (z <= floor_z + float(collision_high_m))
        & (ranges <= float(corridor_length_m))
        & (ranges > 0.05)
    )
    bin_edges = np.linspace(0.0, float(corridor_length_m), int(corridor_bins) + 1)
    result = {}
    for name, centre in (("front", 0.0), ("left", math.pi / 2), ("rear", math.pi),
                         ("right", -math.pi / 2)):
        delta = np.abs(np.arctan2(np.sin(angles - centre), np.cos(angles - centre)))
        sector = delta <= float(corridor_half_rad)
        bins = []
        min_range = None
        for low, high in zip(bin_edges, bin_edges[1:]):
            hit = sector & collision_mask & (ranges >= low) & (ranges < high)
            count = int(np.count_nonzero(hit))
            bins.append(count)
            if count and min_range is None:
                min_range = float(np.min(ranges[hit]))
        covered = all(count >= int(min_points_per_bin) for count in bins)
        floor_hits = sector & floor_mask
        result[name] = {
            "points_collision": int(np.count_nonzero(sector & collision_mask)),
            "bin_points": bins,
            "bins_required": int(corridor_bins),
            "min_points_per_bin": int(min_points_per_bin),
            "observation_state": "measured" if covered else "insufficient_observation",
            "clearance_m": min_range,
            "floor_observation_state": "supported" if np.count_nonzero(floor_hits) >= int(min_points_per_bin) else "unsupported",
            "floor_points": int(np.count_nonzero(floor_hits)),
        }
    return result


def validate_same_pair_contract(meta):
    """Require producer metadata proving the same stage, reference time and pair.

    A topic name or a boolean is not runtime evidence: the producer must supply
    stage, integer reference stamp ns, identical cloud/pose stamps, a pair/slice
    identity, the source binary sha and deskewed=True. A pose a few milliseconds
    away is a different pair and is rejected.
    """
    required = ("stage", "reference_stamp_ns", "cloud_stamp_ns", "pose_stamp_ns",
                "pair_id", "binary_sha256", "deskewed")
    missing = [key for key in required if key not in meta]
    if missing:
        raise AdapterError("missing producer metadata: %s" % ",".join(missing))
    if meta["stage"] not in ("LIO", "VIO"):
        raise AdapterError("unsupported producer stage: %r" % (meta["stage"],))
    if meta["deskewed"] is not True:
        raise AdapterError("producer did not mark the slice deskewed")
    try:
        ref = int(meta["reference_stamp_ns"])
        cloud = int(meta["cloud_stamp_ns"])
        pose = int(meta["pose_stamp_ns"])
    except (TypeError, ValueError):
        raise AdapterError("stamps must be integer nanoseconds")
    if ref < 0 or cloud != ref or pose != ref:
        raise AdapterError("cloud/pose stamps are not the same reference stamp")
    if not isinstance(meta["pair_id"], (str, int)) or str(meta["pair_id"]) == "":
        raise AdapterError("pair identity missing")
    if not isinstance(meta["binary_sha256"], str) or len(meta["binary_sha256"]) != 64:
        raise AdapterError("source binary identity missing")
    return {"stage": meta["stage"], "reference_stamp_ns": ref,
            "pair_id": str(meta["pair_id"]), "binary_sha256": meta["binary_sha256"]}


def build_operator_map_input(cloud, *, source_topic, header_frame, odom_frame, mode, deskewed,
                             cloud_stamp_s, pose_position, pose_quat_xyzw, pose_stamp_s,
                             imu_mount_xyz, max_pair_dt_s=MAX_PAIR_DT_S):
    validate_cloud(source_topic, header_frame, odom_frame, deskewed=deskewed, mode=mode)
    pair_dt = validate_pairing(cloud_stamp_s, pose_stamp_s, max_pair_dt_s=max_pair_dt_s)
    points_base = map_cloud_to_base(cloud, pose_position, pose_quat_xyzw, imu_mount_xyz)
    return {
        "frame": "sensor",
        "points": points_base,
        "cloud_stamp_s": float(cloud_stamp_s),
        "pose_stamp_s": float(pose_stamp_s),
        "pair_dt_s": pair_dt,
        "provenance": {"source": "fastlivo2_deskewed_cloud_registered",
                       "source_topic": source_topic, "grid_frame": header_frame,
                       "estimator_stage": mode, "deskewed": True,
                       "extrinsic": "T_map_imu_to_base once; IMU not relabelled as base"},
    }


def build_perception(*, direction_evidence_map, stamp_s, now_sim_s, max_sim_age_s,
                     source="rtx_lidar_deskewed_cloud_registered"):
    if not math.isfinite(float(stamp_s)) or not math.isfinite(float(now_sim_s)):
        raise AdapterError("perception stamp/now must be finite")
    age = float(now_sim_s) - float(stamp_s)
    if age < 0.0 or age > float(max_sim_age_s):
        raise AdapterError("deskewed cloud is stale or future")
    unknown = [name for name in DIRECTIONS
               if (direction_evidence_map.get(name) or {}).get("observation_state") != "measured"]
    front = direction_evidence_map.get("front") or {}
    return {
        "source": source,
        "stamp_s": float(stamp_s),
        "sim_age_s": round(age, 6),
        "valid": not unknown,
        "unknown_directions": unknown,
        "front_clearance_m": front.get("clearance_m"),
        "directions": direction_evidence_map,
        "depth_valid_ratio": None,
        "note": "LiDAR direction coverage replaces depth_valid_ratio; unknown directions stay unknown",
    }
