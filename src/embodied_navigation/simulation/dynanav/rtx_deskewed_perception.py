"""Production consumer contract for FAST-LIVO2 native deskewed LiDAR clouds.

This module is the only place that turns a native FAST-LIVO2 deskewed map cloud
(/cloud_registered_full) plus its same-stage LIO pose (/aft_mapped_to_init) into
(a) a traceable per-direction LiDAR clearance sample for the interactive
operator and (b) the map-frame cloud that the operator map accumulates.  Pure
numpy: no Isaac and no ROS import, so the contract is testable offline.

Semantics fixed here (never a caller boolean):

* the cloud is the LIO-stage world cloud, already native-IMU deskewed and placed
  in the odometry map frame by FAST-LIVO2 with the same state that publishes
  /aft_mapped_to_init. Both messages carry sec2Stamp(last_lio_update_time) as
  their header stamp, so the pair is accepted only when the integer nanosecond
  stamps are identical. A pose one nanosecond away is a different pair and is
  refused, never interpolated.
* a raw rotating wire scan is refused twice: by topic role (the raw topic is not
  a deskewed topic) and by ROS message content (the raw wire carries per-point
  t/ring/range/ambient/reflectivity fields, while the native deskewed publisher
  emits PointXYZINormal fields including a non-zero per-point curvature time).
  A raw payload republished under a deskewed topic name therefore still fails.
* map to base is applied exactly once: p_imu = R^T (p_map - t_map_imu) then
  p_base = p_imu + imu_mount (the measured IMU origin in the base frame). The
  operator map consumes the cloud as frame="map" and applies no second
  transform.
* a full-circle scan (or the 296-degree native sweep observed in this build) is
  NOT proof that every azimuth was observed. Direction coverage is derived from
  the returns themselves: azimuth-bin coverage, radial-bin coverage, floor
  support and drop/void indicators. A direction missing any of them is
  insufficient_observation and can never authorise the matching translation.
"""

from __future__ import annotations

import math

import numpy as np

from embodied_navigation.common.geometry import quaternion_matrix

# Raw rotating wire layout (capture/lidar.py OUSTER_DTYPE). A deskewed native
# cloud must not carry any of these fields; the native PointXYZINormal publisher
# emits x/y/z/normal_*/intensity/curvature instead.
RAW_WIRE_FIELDS = ("t", "ring", "range", "ambient", "reflectivity")
NATIVE_DESKEWED_REQUIRED_FIELDS = ("x", "y", "z", "curvature")
NATIVE_DESKEWED_FIELDS = (
    "x",
    "y",
    "z",
    "intensity",
    "curvature",
    "normal_x",
    "normal_y",
    "normal_z",
)

# Topic roles. The leaf names are the native FAST-LIVO2 publishers; a caller may
# prefix a namespace but cannot rename the leaf.
DESKEWED_CLOUD_TOPIC_LEAF = "cloud_registered_full"
LIO_POSE_TOPIC_LEAF = "aft_mapped_to_init"
RAW_ROTATING_TOPIC = "/dynanav/lidar_rtx"
DESKEWED_STAGE = "LIO"

MIN_CURVATURE_SPREAD_MS = 1.0


class DeskewedCloudError(ValueError):
    """The inputs cannot be turned into defensible deskewed evidence."""


def _finite(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise DeskewedCloudError("%s must be a real number" % name)
    if not math.isfinite(number):
        raise DeskewedCloudError("%s must be finite" % name)
    return number


def message_stamp_ns(message):
    """Return the ROS header stamp as an exact integer nanosecond count."""
    header = message.header
    seconds = int(header.stamp.sec)
    nanoseconds = int(header.stamp.nanosec)
    if seconds < 0 or nanoseconds < 0:
        raise DeskewedCloudError("negative ROS header stamp")
    return seconds * 1000000000 + nanoseconds


def verify_topic_roles(cloud_topic, pose_topic):
    """Fix the stage from the native topic roles; refuse raw or VIO substitutes."""
    cloud = str(cloud_topic or "").strip()
    pose = str(pose_topic or "").strip()
    if cloud == RAW_ROTATING_TOPIC or cloud.rsplit("/", 1)[-1] == "lidar_rtx":
        raise DeskewedCloudError(
            "raw rotating LiDAR requires native deskew; refusing the raw wire scan"
        )
    if cloud.rsplit("/", 1)[-1] != DESKEWED_CLOUD_TOPIC_LEAF:
        raise DeskewedCloudError(
            "deskewed map cloud must be the native %s publisher, got %r"
            % (DESKEWED_CLOUD_TOPIC_LEAF, cloud)
        )
    if pose.rsplit("/", 1)[-1] != LIO_POSE_TOPIC_LEAF:
        raise DeskewedCloudError(
            "same-stage LIO pose must be the native %s publisher, got %r"
            % (LIO_POSE_TOPIC_LEAF, pose)
        )
    return {"cloud_topic": cloud, "pose_topic": pose, "stage": DESKEWED_STAGE}


def verify_cloud_layout(field_names):
    """Refuse a raw wire payload even when published on a deskewed topic."""
    names = [str(name) for name in field_names]
    raw_present = [name for name in RAW_WIRE_FIELDS if name in names]
    if raw_present:
        raise DeskewedCloudError(
            "cloud carries raw rotating wire fields %s; a per-point-time scan is "
            "not a deskewed cloud" % ",".join(raw_present)
        )
    missing = [name for name in NATIVE_DESKEWED_REQUIRED_FIELDS if name not in names]
    if missing:
        raise DeskewedCloudError(
            "cloud lacks native deskewed fields " + ",".join(missing)
        )
    unexpected = [name for name in names if name not in NATIVE_DESKEWED_FIELDS]
    if unexpected:
        raise DeskewedCloudError(
            "cloud has non-native fields %s; refusing an undeclared payload"
            % ",".join(unexpected)
        )
    return tuple(names)


def verify_deskew_marker(curvature):
    """Require the native per-point time spread of a deskewed slice.

    The native PointXYZINormal cloud keeps the per-point time in curvature; a zero
    or constant curvature means the payload was rebuilt rather than taken from the
    native deskewed publisher.
    """
    values = np.asarray(curvature, dtype=float).reshape(-1)
    if values.size == 0:
        raise DeskewedCloudError("no curvature values to verify")
    if not np.isfinite(values).all():
        raise DeskewedCloudError("non-finite curvature values")
    spread = float(values.max() - values.min())
    if spread <= MIN_CURVATURE_SPREAD_MS:
        raise DeskewedCloudError(
            "deskewed cloud carries a constant per-point curvature (spread %.6f); "
            "the native IMU-deskewed slice has a non-zero time spread" % spread
        )
    return {"curvature_spread_ms": spread,
            "curvature_min_ms": float(values.min()),
            "curvature_max_ms": float(values.max())}


def verify_same_stage_pair(
    *,
    cloud_frame,
    pose_frame,
    child_frame,
    cloud_stamp_ns,
    pose_stamp_ns,
    expected_map_frame,
    expected_child_frame,
    stage,
):
    """Require identical integer stamps, identical frames and the LIO stage."""
    if str(stage) != DESKEWED_STAGE:
        raise DeskewedCloudError("deskewed map cloud must be paired with the LIO stage")
    expected_map = str(expected_map_frame or "").strip()
    if not expected_map:
        raise DeskewedCloudError("expected map frame must be given")
    cloud_label = str(cloud_frame or "").strip()
    pose_label = str(pose_frame or "").strip()
    child_label = str(child_frame or "").strip()
    if not cloud_label or not pose_label or not child_label:
        raise DeskewedCloudError("cloud/pose frame labels must be present")
    if cloud_label != expected_map or pose_label != expected_map:
        raise DeskewedCloudError(
            "cloud frame %r / pose frame %r do not both equal the odometry map frame %r"
            % (cloud_label, pose_label, expected_map)
        )
    if child_label != str(expected_child_frame):
        raise DeskewedCloudError(
            "pose child frame %r is not the expected %r"
            % (child_label, expected_child_frame)
        )
    try:
        cloud_ns = int(cloud_stamp_ns)
        pose_ns = int(pose_stamp_ns)
    except (TypeError, ValueError):
        raise DeskewedCloudError("cloud/pose stamps must be integer nanoseconds")
    if cloud_ns <= 0 or pose_ns <= 0:
        raise DeskewedCloudError("cloud/pose stamps must be positive")
    if cloud_ns != pose_ns:
        raise DeskewedCloudError(
            "cloud stamp %d ns and pose stamp %d ns are not the same reference; a "
            "near stamp is a different pair and is never interpolated"
            % (cloud_ns, pose_ns)
        )
    return {
        "stage": DESKEWED_STAGE,
        "reference_stamp_ns": cloud_ns,
        "cloud_stamp_ns": cloud_ns,
        "pose_stamp_ns": pose_ns,
        "map_frame": expected_map,
        "child_frame": child_label,
    }


def rotation_from_quaternion_xyzw(quat_xyzw):
    """Return the 3x3 rotation of a normalised (x, y, z, w) quaternion."""
    quat = np.asarray(quat_xyzw, dtype=float).reshape(-1)
    if quat.shape != (4,):
        raise DeskewedCloudError("quaternion must have four components")
    if not np.isfinite(quat).all():
        raise DeskewedCloudError("quaternion must be finite")
    x, y, z, w = quat
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise DeskewedCloudError("degenerate quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return quaternion_matrix(w, x, y, z)


def map_points_to_base(points_map, position_map, quat_xyzw, imu_mount_xyz):
    """map to base exactly once: p_imu = R^T(p_map - t), p_base = p_imu + mount."""
    points = np.asarray(points_map, dtype=float)
    if points.ndim != 2 or points.shape[1] < 3:
        raise DeskewedCloudError("points must be an Nx3+ array")
    points = points[:, :3]
    if not np.isfinite(points).all():
        raise DeskewedCloudError("cloud contains non-finite points")
    position = np.asarray(position_map, dtype=float).reshape(-1)
    mount = np.asarray(imu_mount_xyz, dtype=float).reshape(-1)
    if position.shape != (3,) or mount.shape != (3,):
        raise DeskewedCloudError("position and imu mount must be 3-vectors")
    if not (np.isfinite(position).all() and np.isfinite(mount).all()):
        raise DeskewedCloudError("position and imu mount must be finite")
    rotation = rotation_from_quaternion_xyzw(quat_xyzw)
    relative = points - position.reshape(1, 3)
    points_imu = relative @ rotation
    return points_imu + mount.reshape(1, 3)


def state_age_gate(
    *,
    now_sim_s,
    stamp_s,
    arrival_wall_s,
    now_wall_s,
    max_sim_age_s,
    max_wall_age_s,
):
    """Return (ok, reason, ages). Original clock roles: source vs arrival."""
    try:
        now_sim = _finite(now_sim_s, "now_sim_s")
        stamp = _finite(stamp_s, "stamp_s")
        arrival = _finite(arrival_wall_s, "arrival_wall_s")
        now_wall = _finite(now_wall_s, "now_wall_s")
    except DeskewedCloudError as error:
        return False, "invalid_clock", {"error": str(error)}
    sim_age = now_sim - stamp
    wall_age = now_wall - arrival
    ages = {"sim_age_s": sim_age, "wall_age_s": wall_age}
    if sim_age < -2e-6:
        return False, "cloud_source_in_future", ages
    if wall_age < -2e-6:
        return False, "cloud_arrival_in_future", ages
    if sim_age > float(max_sim_age_s):
        return False, "cloud_sim_expired", ages
    if wall_age > float(max_wall_age_s):
        return False, "cloud_wall_expired", ages
    return True, None, ages
