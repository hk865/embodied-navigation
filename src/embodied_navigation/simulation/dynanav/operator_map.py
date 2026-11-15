"""Operator-side measured occupancy and frontier map for waypoint selection.

This module is the operator counterpart of the simulator-side interactive
waypoint explorer. The simulator publishes measured topics (estimator
odometry plus live depth/lidar); the operator turns them into an explicit,
gate-checked occupancy snapshot so the next waypoint can be chosen from what
was really observed. It is pure numpy and stdlib at module scope: no rclpy, no
Isaac and no ROS message classes are imported here, which keeps the offline
unit tests independent of a running ROS graph. A ROS adapter may call these
functions from inside a node, but nothing in this file touches ROS itself.

The hard invariants, each of which exists to prevent a plausible-looking but
fake map:

* The base pose is always a measured estimator pose. Ground-truth pose names
  are rejected before any geometry is used, and ground_truth_used is
  hard-coded false in every receipt.
* The sensor->base extrinsic is never optional and never guessed. It is a
  4x4 homogeneous transform supplied by the caller (or an equivalent
  translation plus quaternion) and is validated for orthonormality (1e-3) and
  finite translation before use. Missing or invalid -> ValueError.
* A point cloud declares its frame. "map" means the points already carry
  the map pose; "base" means a measured base pose is applied; "sensor"
  means the base pose and the extrinsic are applied. Unknown frames raise
  ValueError, and a declared sensor/base cloud whose points are obviously in
  map coordinates (out of sensor range) is rejected instead of silently
  transformed.
* A point may only be occupied in the level whose measurement produced it.
  Levels are created from the *measured pose z* only, with a 0.3 m band and a
  0.15 m hysteresis. An independently evidenced support floor may exclude an
  incompatible old height band without moving that band's grid or anchor.
  Points are never assigned to a level from a hard-coded
  building floor height. If one pose lies in two level bands the tracker
  reports ambiguous_level and refuses to produce an actionable result.
* A level's floor offset is estimated from the low quantile of the level's
  observed point z values, never from a world constant, but the estimate is
  only *used* when it is backed by independent credible evidence in the same
  frame and level. A surface being below the base and densely sampled is not
  enough: a kerb, step edge, pallet or low panel sits there too. One of three
  measured evidence routes must hold: (1) the candidate agrees with the floor
  already established for the level AND the support band lies inside that
  established floor's free band; (2) the caller supplies a measured
  body-support/stand height that agrees with the candidate; or (3) with no
  established floor and no proof, the measured base stands at least
  MIN_BODY_SUPPORT_STANDOFF_M above the candidate and the support is observed
  across both horizontal axes (not a narrow ledge). Without one of these the
  frame is refused with floor_evidence_unconfirmed, the cells stay unknown,
  the level is not actionable, and the receipt/snapshot carries the explicit
  reason. No floor height is ever injected from ground truth or a building
  floor constant.
* Occupied cells are points 0.15 m to max_height_m above that evidenced floor,
  below it the floor returns are observed free, and everything else stays
  unknown.
* Free-space ray carving starts at the measured *sensor* origin of the frame:
  the measured base pose composed with the declared sensor->base extrinsic
  (T_map_base @ T_base_sensor), never at the base origin. A point between the
  base origin and the sensor origin is therefore not carved free.
* A frontier cell is an observed-free cell adjacent to a never-observed cell.
  It is *actionable* only when every gate passes: pose fresh (sim age <= 0.25
  s and wall age <= 0.5 s, supplied by the caller), cloud/pose pairing within
  max_pair_dt (0.10 s), valid extrinsic, unambiguous level, at least
  min_points measured points in the level, and credible floor evidence for the
  level. A failed gate still returns the frontier cells, but with
  actionable=False and explicit reasons.
* Freshness is re-evaluated on every snapshot. Clock authority is explicit:
  every arrival is recorded on time.monotonic(); a caller-supplied now_wall_s
  must be that same monotonic clock, and now_sim_s is the source/estimator
  clock of the ingested stamps. The two clocks are never substituted for each
  other. Each age is now minus the reference stamp of its own clock, no cached
  now and no cached age is reused. When no caller now is given, the recorded
  arrival is compared with time.monotonic(), so a stalled stream still goes
  stale. The cloud reference is kept separately from the pose stream and per
  level, so a fresh pose - or a fresh frame on another level - never stands in
  for a fresh cloud on this level. When either the source age or the arrival
  age of either stream cannot be evaluated (missing arrival, or a present but
  non-finite now), the snapshot is not actionable and names the reason. A
  refused latest frame keeps the previously observed map geometry but makes
  the level non-actionable until a new frame is accepted: map contents and
  current actionability are separate facts.

The JSON snapshot intentionally separates measured path length from observed
area, and separates a single-frame snapshot from an accumulated map. A
snapshot is evidence of what was observed; it is not the accumulated map
unless the caller says so. path_length_is_not_coverage and
snapshot_is_accumulated_map record that distinction. occupied_cells and
free_cells are cell counts; frontier.cells carries positions, and
unknown_estimate_area_m2 is a local boundary lower bound, not the total unseen
area.

Quaternions are always (x, y, z, w) unless a helper says otherwise.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from bisect import bisect_right
from collections import deque
from dataclasses import dataclass

import numpy as np

from embodied_navigation.common.geometry import quaternion_matrix

__all__ = [
    "DEFAULT_BAND_WIDTH_M",
    "DEFAULT_EXTRINSIC_TOLERANCE",
    "DEFAULT_FLOOR_QUANTILE",
    "DEFAULT_FREE_MODE",
    "DEFAULT_HYSTERESIS_M",
    "DEFAULT_MAP_FRAME",
    "DEFAULT_MAX_HEIGHT_M",
    "DEFAULT_MAX_PAIR_DT_S",
    "DEFAULT_MAX_POSE_SIM_AGE_S",
    "DEFAULT_MAX_POSE_WALL_AGE_S",
    "DEFAULT_MAX_RANGE_M",
    "DEFAULT_MAX_SENSOR_RANGE_M",
    "DEFAULT_MIN_POINTS",
    "DEFAULT_RESOLUTION_M",
    "FREE",
    "FREE_MODES",
    "FORBIDDEN_CLOUD_SOURCES",
    "FORBIDDEN_POSE_SOURCES",
    "GROUND_FLOOR_CONSISTENCY_M",
    "MIN_BODY_SUPPORT_STANDOFF_M",
    "MIN_GROUND_SUPPORT_POINTS",
    "MIN_GROUND_SUPPORT_SPAN_M",
    "MIN_OBSTACLE_CLEARANCE_M",
    "STAND_PROOF_CONSISTENCY_M",
    "OCCUPIED",
    "POINT_FRAMES",
    "SCHEMA_VERSION",
    "Extrinsic",
    "Level",
    "LevelGrid",
    "LevelMatch",
    "LevelTracker",
    "NormalizedPoints",
    "OccupancyAccumulator",
    "PairingResult",
    "PosePairer",
    "build_map_snapshot",
    "depth_image_to_points",
    "points_to_map",
    "quaternion_xyzw_from_matrix",
    "quaternion_xyzw_from_yaw",
    "slerp_quaternion_xyzw",
    "transform_from_position_quaternion",
    "validate_map_cloud_source",
    "UNKNOWN_ESTIMATE_DEFINITION",
]

SCHEMA_VERSION = 1

# Grid codes. Unknown cells are never stored: absence from the grid means
# never observed, which is the only honest representation of unknown.
FREE = 0
OCCUPIED = 1
UNKNOWN = -1

DEFAULT_RESOLUTION_M = 0.05
DEFAULT_MAX_PAIR_DT_S = 0.10
DEFAULT_MAX_POSE_SIM_AGE_S = 0.25
DEFAULT_MAX_POSE_WALL_AGE_S = 0.50
DEFAULT_BAND_WIDTH_M = 0.30
DEFAULT_HYSTERESIS_M = 0.15
DEFAULT_MIN_POINTS = 200
DEFAULT_FLOOR_QUANTILE = 0.05
DEFAULT_MAX_HEIGHT_M = 2.2
DEFAULT_MAX_RANGE_M = 12.0
DEFAULT_MAX_SENSOR_RANGE_M = 20.0
DEFAULT_EXTRINSIC_TOLERANCE = 1e-3
MIN_OBSTACLE_CLEARANCE_M = 0.15
FREE_FLOOR_BELOW_M = 0.20
# Credible ground evidence. A low percentile is only a floor candidate; it
# becomes the level floor only through independent measured evidence: agreement
# with the level's established floor (once one exists) with the support band
# inside that floor's free band, or a caller-supplied measured body-support
# height, or - for a level's first floor - a measured base standoff of two
# obstacle clearances plus observed lateral extent. "Below the base plus 50
# points" alone is not evidence: a kerb, step edge, pallet or low panel also
# sits below the base. These are fail-closed evidence rules derived from the
# existing clearance scale, never building floor heights.
MIN_GROUND_SUPPORT_POINTS = 50
MIN_BODY_SUPPORT_STANDOFF_M = 2.0 * MIN_OBSTACLE_CLEARANCE_M
MIN_GROUND_SUPPORT_SPAN_M = 0.50
# A caller-measured body-support height must agree with the candidate within
# the obstacle-clearance scale: a 0.15 m low surface is exactly the kind of
# mismatch this route has to reject.
STAND_PROOF_CONSISTENCY_M = MIN_OBSTACLE_CLEARANCE_M
GROUND_FLOOR_CONSISTENCY_M = 2.0 * MIN_OBSTACLE_CLEARANCE_M
# Explicit reasons. Existing reasons keep their meaning; these are added for
# the new fail-closed paths (never silently reusing an old value).
REASON_FLOOR_EVIDENCE_MISSING = "floor_evidence_missing"
REASON_FLOOR_EVIDENCE_INCONSISTENT = "floor_evidence_inconsistent"
REASON_FLOOR_EVIDENCE_UNCONFIRMED = "floor_evidence_unconfirmed"
REASON_FLOOR_EVIDENCE_UNKNOWN = "floor_evidence_unknown"
REASON_STALE_CLOUD_SIM_AGE = "stale_cloud_sim_age"
REASON_STALE_CLOUD_WALL_AGE = "stale_cloud_wall_age"
REASON_CLOUD_AGE_UNKNOWN = "cloud_age_unknown"
REASON_CLOUD_AGE_UNEVALUATED = "cloud_age_unevaluated"
REASON_UNTRUSTED_GENERATION = "untrusted_generation"
REASON_CLOUD_AGE_INVALID = "cloud_age_invalid"
REASON_POSE_AGE_INVALID = "pose_age_invalid"
REASON_CLOCK_INVALID = "clock_invalid"
REASON_LATEST_FRAME_REFUSED = "latest_frame_refused"
REASON_SIM_CLOCK_REGRESSION = "sim_clock_regression"
REASON_WALL_CLOCK_REGRESSION = "wall_clock_regression"
# A level's floor search window around the measured pose z. It is wide enough
# to hold the floor below the base and the obstacle band above it, and narrow
# enough that a point cloud from an adjacent floor cannot leak into the floor
# estimate. These are search bounds for the estimator, not building constants.
FLOOR_SEARCH_BELOW_M = 2.5
FLOOR_SEARCH_ABOVE_M = 4.0

DEFAULT_SENSOR_FRAME = "sensor"
DEFAULT_BASE_FRAME = "base"
DEFAULT_MAP_FRAME = "map"
POINT_FRAMES = ("sensor", "base", "map")

FREE_MODES = ("floor", "rays", "both", "none")
DEFAULT_FREE_MODE = "both"

# Mirrors the simulator-side and operator-client gate: a ground-truth pose
# name can never become a map or waypoint frame.
FORBIDDEN_POSE_SOURCES = (
    "native_truth",
    "native_pose",
    "simulator_native_pose",
    "ground_truth",
    "gt_diagnostic",
)
FORBIDDEN_CLOUD_SOURCES = ("ground_truth", "gt_mesh", "oracle_mesh")

DEFAULT_NOTE = (
    "operator-side measured map snapshot; levels, floor offset and frontier "
    "are derived from measured estimator pose and live points, never from "
    "simulator ground truth or a navigation map"
)

UNKNOWN_ESTIMATE_DEFINITION = (
    "area of unknown cells adjacent to observed cells (8-connected) times the "
    "cell area; a local lower bound on unseen area, not the total unseen area"
)

NEIGHBOURS_8 = ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1))


def _finite_float(value, name):
    """Return value as a finite float or raise ValueError."""
    if isinstance(value, bool) or not isinstance(
        value, (int, float, np.integer, np.floating)
    ):
        raise ValueError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def _optional_finite_float(value, name):
    """Return a finite float, or None when absent or non-finite.

    Optional clock and age inputs may be absent. A present but non-finite
    value (NaN/Inf) is never clipped to a number and never passes a
    comparison: it becomes None so the caller records that age as unevaluable
    and the corresponding gate fails closed. A non-numeric value is still a
    programming error and raises ValueError.
    """
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(
        value, (int, float, np.integer, np.floating)
    ):
        raise ValueError(f"{name} must be a finite number or None")
    number = float(value)
    return number if math.isfinite(number) else None


def _round(value, digits=6):
    """Round a float for a JSON receipt, preserving None."""
    if value is None:
        return None
    return round(float(value), digits)


def _as_point_array(points, name="points"):
    """Return a finite (N, 3) float64 copy of a declared point cloud."""
    try:
        array = np.asarray(points, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be an (N, 3) array of finite numbers") from error
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f"{name} must be an (N, 3) array, got shape {array.shape}")
    return np.array(array, dtype=float, copy=True)


def _frame_name(frame):
    """Normalise and validate a declared point-cloud frame."""
    if frame is None:
        raise ValueError(
            "point cloud frame declaration is required: one of "
            + ", ".join(POINT_FRAMES)
        )
    if not isinstance(frame, str):
        raise ValueError("point cloud frame declaration must be a string")
    name = frame.strip().lower()
    if name not in POINT_FRAMES:
        raise ValueError(
            f"unknown point cloud frame {frame!r}; expected one of "
            + ", ".join(POINT_FRAMES)
        )
    return name


def _free_mode(mode):
    """Normalise and validate an observed-free construction mode."""
    if mode is None:
        return DEFAULT_FREE_MODE
    if not isinstance(mode, str) or mode.strip().lower() not in FREE_MODES:
        raise ValueError(
            f"unknown free mode {mode!r}; expected one of " + ", ".join(FREE_MODES)
        )
    return mode.strip().lower()


def validate_map_cloud_source(source):
    """Raw rotating scans cannot use this map's single-pose frame transform."""
    if isinstance(source, str) and source.strip().lower() in (
        "/dynanav/lidar_rtx", "lidar_rtx", "rtx_lidar"
    ):
        raise ValueError(
            "raw RTX LiDAR scans require FAST-LIVO IMU deskew before operator mapping; "
            "one scan-start pose cannot represent every return. A correctly framed "
            "deskewed-output adapter is required, not a topic rename"
        )
    return source


def _check_source(source, forbidden, name):
    """Reject a forbidden (ground-truth) source name; return a clean name."""
    if not isinstance(source, str) or not source.strip():
        raise ValueError(f"{name} must be a non-empty string")
    name_clean = source.strip()
    if name_clean.lower() in forbidden:
        raise ValueError(f"{name} {source!r} is a ground-truth source and is refused")
    return name_clean


# ---------------------------------------------------------------------------
# Quaternion and transform helpers (numpy only; no scipy dependency).
# ---------------------------------------------------------------------------


def quaternion_xyzw_from_yaw(yaw_rad):
    """Return the unit quaternion (x, y, z, w) of a yaw rotation."""
    angle = _finite_float(yaw_rad, "yaw_rad")
    return np.array(
        [0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0)], dtype=float
    )


def quaternion_xyzw_from_matrix(matrix):
    """Return the unit quaternion (x, y, z, w) of a 3x3 rotation matrix."""
    rotation = np.asarray(matrix, dtype=float)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError("rotation matrix must be a finite 3x3 array")
    trace = float(rotation[0, 0] + rotation[1, 1] + rotation[2, 2])
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] >= rotation[1, 1] and rotation[0, 0] >= rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] >= rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    quaternion = np.array([x, y, z, w], dtype=float)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("rotation matrix cannot be converted to a quaternion")
    return quaternion / norm


def slerp_quaternion_xyzw(start, end, alpha):
    """Spherical interpolation of (x, y, z, w) quaternions, shortest arc."""
    q_start = np.asarray(start, dtype=float)
    q_end = np.asarray(end, dtype=float)
    if q_start.shape != (4,) or q_end.shape != (4,):
        raise ValueError("quaternions must contain 4 numbers")
    if not np.isfinite(q_start).all() or not np.isfinite(q_end).all():
        raise ValueError("quaternions must contain finite numbers")
    norm_start = float(np.linalg.norm(q_start))
    norm_end = float(np.linalg.norm(q_end))
    if norm_start <= 0.0 or norm_end <= 0.0:
        raise ValueError("quaternions must have non-zero norm")
    q_start = q_start / norm_start
    q_end = q_end / norm_end
    fraction = _finite_float(alpha, "alpha")
    fraction = max(0.0, min(1.0, fraction))
    dot = float(q_start @ q_end)
    if dot < 0.0:
        q_end = -q_end
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 1.0 - 1e-9:
        quaternion = q_start
    elif dot > 0.9995:
        quaternion = q_start + fraction * (q_end - q_start)
    else:
        angle_start = math.acos(dot)
        sine_start = math.sin(angle_start)
        angle = angle_start * fraction
        quaternion = (
            math.sin(angle_start - angle) / sine_start * q_start
            + math.sin(angle) / sine_start * q_end
        )
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("quaternion interpolation produced a zero quaternion")
    return quaternion / norm


def transform_from_position_quaternion(position, quaternion_xyzw):
    """Return the 4x4 rigid transform of a position and (x, y, z, w) quat."""
    xyz = np.asarray(position, dtype=float)
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    if xyz.shape != (3,) or not np.isfinite(xyz).all():
        raise ValueError("position must contain 3 finite numbers")
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("quaternion_xyzw must contain 4 finite numbers")
    norm = float(np.linalg.norm(quaternion))
    if abs(norm - 1.0) > DEFAULT_EXTRINSIC_TOLERANCE:
        raise ValueError("quaternion_xyzw must be unit length within 1e-3")
    x, y, z, w = quaternion / norm
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = quaternion_matrix(w, x, y, z)
    transform[:3, 3] = xyz
    return transform


def _validate_rotation(rotation, name, tolerance=DEFAULT_EXTRINSIC_TOLERANCE):
    """Raise ValueError unless a 3x3 block is orthonormal with det +1."""
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError(f"{name} must be a finite 3x3 rotation")
    orthonormal_error = float(np.max(np.abs(matrix.T @ matrix - np.eye(3))))
    if orthonormal_error > tolerance:
        raise ValueError(
            f"{name} is not orthonormal within {tolerance:g} "
            f"(max |R^T R - I| = {orthonormal_error:.3g})"
        )
    determinant = float(np.linalg.det(matrix))
    if abs(determinant - 1.0) > tolerance:
        raise ValueError(
            f"{name} determinant must be +1 within {tolerance:g}, got {determinant:.6g}"
        )
    return matrix


def _as_transform(value, name="transform"):
    """Return a validated 4x4 homogeneous transform.

    Accepts a 4x4 array-like, a dict with position/translation and a
    quaternion (quaternion_xyzw, quaternion_wxyz, {x,y,z,w} or a sequence
    documented as xyzw), or a (position, quaternion) pair. Rotation
    orthonormality is not demanded here because measured estimator poses may
    be slightly noisy; the extrinsic path validates it explicitly.
    """
    if isinstance(value, dict):
        position = value.get("position", value.get("translation"))
        if position is None:
            raise ValueError(f"{name} dict must contain position or translation")
        if "quaternion_xyzw" in value:
            quaternion = value["quaternion_xyzw"]
        elif all(key in value for key in ("x", "y", "z", "w")):
            quaternion = [value["x"], value["y"], value["z"], value["w"]]
        elif "quaternion_wxyz" in value:
            w, x, y, z = value["quaternion_wxyz"]
            quaternion = [x, y, z, w]
        elif "quaternion" in value:
            quaternion = value["quaternion"]
        else:
            raise ValueError(f"{name} dict must contain a quaternion")
        return transform_from_position_quaternion(position, quaternion)
    if isinstance(value, (tuple, list)) and len(value) == 2:
        return transform_from_position_quaternion(value[0], value[1])
    try:
        matrix = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a 4x4 homogeneous transform") from error
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{name} must be a 4x4 homogeneous transform")
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-6):
        raise ValueError(f"{name} bottom row must be [0, 0, 0, 1]")
    return np.array(matrix, dtype=float, copy=True)


class Extrinsic:
    """Explicit, validated sensor->base extrinsic.

    The transform maps a point expressed in the sensor frame to the base
    frame: p_base = T_base_sensor @ p_sensor. It must be supplied by the
    caller; this class never guesses a camera mount, never drops a rotation
    and never treats a yaw-only assumption as a calibration.
    """

    def __init__(
        self,
        transform=None,
        *,
        translation=None,
        quaternion_xyzw=None,
        sensor_frame=DEFAULT_SENSOR_FRAME,
        base_frame=DEFAULT_BASE_FRAME,
        tolerance=DEFAULT_EXTRINSIC_TOLERANCE,
    ):
        if transform is None:
            if translation is None or quaternion_xyzw is None:
                raise ValueError(
                    "extrinsic_missing: sensor->base extrinsic requires a 4x4 "
                    "transform or translation plus quaternion_xyzw"
                )
            transform = transform_from_position_quaternion(
                translation, quaternion_xyzw
            )
        matrix = _as_transform(transform, "extrinsic")
        _validate_rotation(matrix[:3, :3], "extrinsic rotation", tolerance)
        if not isinstance(sensor_frame, str) or not sensor_frame.strip():
            raise ValueError("sensor_frame must be a non-empty string")
        if not isinstance(base_frame, str) or not base_frame.strip():
            raise ValueError("base_frame must be a non-empty string")
        if sensor_frame.strip() == base_frame.strip():
            raise ValueError("sensor_frame and base_frame must differ")
        self._matrix = np.array(matrix, dtype=float, copy=True)
        self._matrix.setflags(write=False)
        self.sensor_frame = sensor_frame.strip()
        self.base_frame = base_frame.strip()
        self.sha256 = hashlib.sha256(
            np.ascontiguousarray(self._matrix, dtype="<f8").tobytes()
        ).hexdigest()

    @property
    def matrix(self):
        """Return a writable copy of the sensor->base transform."""
        return np.array(self._matrix, dtype=float, copy=True)

    @property
    def inverse(self):
        """Return the base->sensor inverse transform."""
        rotation = self._matrix[:3, :3]
        translation = self._matrix[:3, 3]
        inverse = np.eye(4, dtype=float)
        inverse[:3, :3] = rotation.T
        inverse[:3, 3] = -(rotation.T @ translation)
        return inverse

    def to_dict(self):
        """Return a JSON-friendly description of the calibration."""
        return {
            "sensor_frame": self.sensor_frame,
            "base_frame": self.base_frame,
            "matrix": self.matrix.tolist(),
            "sha256": self.sha256,
        }

    def __repr__(self):
        return (
            f"Extrinsic(sensor_frame={self.sensor_frame!r}, "
            f"base_frame={self.base_frame!r}, sha256={self.sha256[:12]!r}...)"
        )


@dataclass(frozen=True)
class PairingResult:
    """Outcome of pairing one cloud stamp with the measured pose stream."""

    ok: bool
    pose: object
    query_stamp_s: float
    used_pose_stamp_s: object
    dt_s: object
    interpolated: bool
    reason: object
    bracket_stamps: object = None

    def as_dict(self):
        return {
            "ok": self.ok,
            "query_stamp_s": _round(self.query_stamp_s),
            "used_pose_stamp_s": _round(self.used_pose_stamp_s),
            "dt_s": _round(self.dt_s),
            "interpolated": self.interpolated,
            "reason": self.reason,
            "bracket_stamps": (
                None
                if self.bracket_stamps is None
                else [_round(self.bracket_stamps[0]), _round(self.bracket_stamps[1])]
            ),
        }


class PosePairer:
    """Nearest-neighbour / linear-and-slerp pose pairing in stamp time.

    Samples are (stamp_s, pose) pairs of measured estimator poses. For a
    query stamp the pairer returns the nearest pose within max_pair_dt_s;
    when the two bracketing samples are both within the bound it linearly
    interpolates the position and spherically interpolates the rotation.
    Beyond the bound it returns None (pair_pose) / a failed PairingResult with
    reason="pairing_gap". In both cases the used sample stamp, the query stamp
    and the signed dt = used - query are recorded so a caller can see *why* a
    frame was rejected instead of silently using the nearest pose.

    dt_s is used_pose_stamp_s - query_stamp_s: positive means the pose is
    newer than the cloud. The pairing gate uses abs(dt_s) <= max_pair_dt.
    """

    def __init__(self, max_pair_dt_s=DEFAULT_MAX_PAIR_DT_S, samples=None):
        self.max_pair_dt_s = _finite_float(max_pair_dt_s, "max_pair_dt_s")
        if self.max_pair_dt_s <= 0.0:
            raise ValueError("max_pair_dt_s must be positive")
        self._stamps = []
        self._poses = []
        self._last = PairingResult(
            False, None, float("nan"), None, None, False, "no_pose", None
        )
        if samples is not None:
            self.extend(samples)

    def __len__(self):
        return len(self._stamps)

    @property
    def stamps(self):
        """Return the sorted sample stamps as a float64 array."""
        return np.asarray(self._stamps, dtype=float)

    @property
    def last_pairing(self):
        """Return the most recent pairing outcome."""
        return self._last

    def add(self, stamp_s, pose):
        """Add one measured pose sample; out-of-order stamps stay sorted."""
        stamp = _finite_float(stamp_s, "pose stamp")
        transform = _as_transform(pose, "pose")
        position = bisect_right(self._stamps, stamp)
        self._stamps.insert(position, stamp)
        self._poses.insert(position, transform)
        return None

    def extend(self, samples):
        """Add an iterable of (stamp_s, pose) pairs or sample dicts."""
        for sample in samples:
            if isinstance(sample, dict):
                self.add(sample["stamp_s"], sample["pose"])
            else:
                stamp, pose = sample
                self.add(stamp, pose)
        return None

    def _record(self, result):
        self._last = result
        return result

    def preview(self, query_stamp_s):
        """Return the pairing outcome for a query without keeping it recorded.

        Runs exactly the same pairing algorithm as pair(); only the stored
        last_pairing is rolled back, so a caller can ask whether a cloud would pair
        without mutating the pairer or any map state.
        """
        previous = self._last
        try:
            return self.pair(query_stamp_s)
        finally:
            self._last = previous

    def pair(self, query_stamp_s):
        """Return the full pairing outcome for a query stamp."""
        query = _finite_float(query_stamp_s, "query stamp")
        stamps = np.asarray(self._stamps, dtype=float)
        if stamps.size == 0:
            return self._record(
                PairingResult(False, None, query, None, None, False, "no_pose", None)
            )
        position = int(np.searchsorted(stamps, query, side="left"))
        lower = position - 1 if position > 0 else None
        upper = position if position < stamps.size else None
        candidates = [index for index in (lower, upper) if index is not None]
        nearest = min(candidates, key=lambda index: (abs(stamps[index] - query), index))
        nearest_dt = float(stamps[nearest] - query)
        if lower is not None and upper is not None:
            lower_dt = abs(float(stamps[lower] - query))
            upper_dt = abs(float(stamps[upper] - query))
            if lower_dt <= self.max_pair_dt_s and upper_dt <= self.max_pair_dt_s:
                span = float(stamps[upper] - stamps[lower])
                if span > 0.0:
                    alpha = (query - float(stamps[lower])) / span
                    if 1e-9 < alpha < 1.0 - 1e-9:
                        pose = _interpolate_pose(
                            self._poses[lower], self._poses[upper], alpha
                        )
                        return self._record(
                            PairingResult(
                                True,
                                pose,
                                query,
                                float(stamps[nearest]),
                                nearest_dt,
                                True,
                                None,
                                (float(stamps[lower]), float(stamps[upper])),
                            )
                        )
        if abs(nearest_dt) <= self.max_pair_dt_s:
            return self._record(
                PairingResult(
                    True,
                    np.array(self._poses[nearest], dtype=float, copy=True),
                    query,
                    float(stamps[nearest]),
                    nearest_dt,
                    False,
                    None,
                    None,
                )
            )
        return self._record(
            PairingResult(
                False,
                None,
                query,
                float(stamps[nearest]),
                nearest_dt,
                False,
                "pairing_gap",
                None,
            )
        )

    def pair_pose(self, query_stamp_s):
        """Return the paired 4x4 pose or None when the pairing gap is too wide."""
        result = self.pair(query_stamp_s)
        return result.pose if result.ok else None


def _interpolate_pose(start, end, alpha):
    """Linear position and slerp rotation between two 4x4 poses."""
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    position = (1.0 - alpha) * start[:3, 3] + alpha * end[:3, 3]
    q_start = quaternion_xyzw_from_matrix(start[:3, :3])
    q_end = quaternion_xyzw_from_matrix(end[:3, :3])
    quaternion = slerp_quaternion_xyzw(q_start, q_end, alpha)
    return transform_from_position_quaternion(position, quaternion)


@dataclass(frozen=True)
class Level:
    """One measured level tracked from the measured pose z."""

    index: int
    z_m: float
    observations: int


@dataclass(frozen=True)
class LevelMatch:
    """Result of assigning one measured pose z to a level."""

    level_index: object
    level_z_m: object
    observed_z_m: float
    ambiguous: bool
    created: bool
    reasons: tuple = ()

    def as_dict(self):
        return {
            "level_index": self.level_index,
            "level_z_m": _round(self.level_z_m),
            "observed_z_m": _round(self.observed_z_m),
            "ambiguous": self.ambiguous,
            "created": self.created,
            "reasons": list(self.reasons),
        }


class LevelTracker:
    """Dynamic level clustering from measured pose z only.

    A level is created the first time a measured pose z is farther than the
    band half width plus hysteresis from every tracked level. An observation
    within the band of exactly one level is assigned to it; within the
    hysteresis margin it stays with the nearest level instead of creating a
    spurious floor. When an observation is claimed with a near tie by two
    levels (their bands are closer than the hysteresis) the tracker reports
    ambiguous_level and returns no level: the caller must not act on a map
    whose floor membership is unresolved.

    No building floor constant ever enters this class. The only inputs are
    measured pose z values and optional compatibility of already evidenced
    floors; level z is the running mean of the measured poses assigned to it.
    """

    def __init__(
        self,
        band_width_m=DEFAULT_BAND_WIDTH_M,
        hysteresis_m=DEFAULT_HYSTERESIS_M,
        max_levels=64,
    ):
        self.band_width_m = _finite_float(band_width_m, "band_width_m")
        self.hysteresis_m = _finite_float(hysteresis_m, "hysteresis_m")
        if self.band_width_m <= 0.0:
            raise ValueError("band_width_m must be positive")
        if self.hysteresis_m < 0.0:
            raise ValueError("hysteresis_m must be non-negative")
        self.max_levels = int(max_levels)
        if self.max_levels < 1:
            raise ValueError("max_levels must be at least 1")
        self._sums = []
        self._counts = []

    @property
    def band_half_width_m(self):
        return self.band_width_m / 2.0

    @property
    def claim_radius_m(self):
        """Distance within which a level keeps an observation before spawning one."""
        return self.band_half_width_m + self.hysteresis_m

    @property
    def levels(self):
        """Return the tracked levels sorted by z."""
        levels = [
            Level(index, self._sums[index] / self._counts[index], self._counts[index])
            for index in range(len(self._sums))
            if self._counts[index] > 0
        ]
        levels.sort(key=lambda level: level.z_m)
        return tuple(levels)

    def level_z_m(self, level_index):
        """Return the tracked z of one level index, or None."""
        if not isinstance(level_index, int) or level_index < 0:
            return None
        if level_index >= len(self._sums) or self._counts[level_index] <= 0:
            return None
        return self._sums[level_index] / self._counts[level_index]

    def observe(self, pose_z_m, *, compatible_levels=None):
        """Assign one measured pose z to a level, creating or refusing as needed."""
        observed = _finite_float(pose_z_m, "pose z")
        if not self._sums:
            self._sums.append(observed)
            self._counts.append(1)
            return LevelMatch(0, observed, observed, False, True)
        centers = [self.level_z_m(index) for index in range(len(self._sums))]
        # A caller may exclude a previously anchored height band when the
        # current independently observed support surface belongs to another
        # floor. Centers are still created only from this measured pose z;
        # an incompatible old grid and its immutable floor are never moved.
        distances = [
            abs(observed - center)
            if compatible_levels is None or index in compatible_levels
            else math.inf
            for index, center in enumerate(centers)
        ]
        order = sorted(range(len(distances)), key=lambda index: (distances[index], index))
        nearest = order[0]
        claim_radius = self.claim_radius_m
        if len(order) > 1:
            second = order[1]
            if (
                distances[nearest] <= claim_radius
                and distances[second] <= claim_radius
                and abs(distances[nearest] - distances[second])
                <= self.hysteresis_m
            ):
                return LevelMatch(
                    None,
                    None,
                    observed,
                    True,
                    False,
                    ("ambiguous_level",),
                )
        if distances[nearest] <= claim_radius:
            self._sums[nearest] += observed
            self._counts[nearest] += 1
            return LevelMatch(
                nearest,
                self._sums[nearest] / self._counts[nearest],
                observed,
                False,
                False,
            )
        if len(self._sums) >= self.max_levels:
            return LevelMatch(
                None,
                None,
                observed,
                False,
                False,
                ("too_many_levels",),
            )
        self._sums.append(observed)
        self._counts.append(1)
        return LevelMatch(len(self._sums) - 1, observed, observed, False, True)

    def to_dict(self):
        """Return a JSON-friendly level summary."""
        return [
            {
                "level_index": level.index,
                "z_m": _round(level.z_m),
                "observations": level.observations,
            }
            for level in self.levels
        ]


class LevelGrid:
    """Sparse 2D occupancy grid for one level.

    Cells are indexed by floor(x / resolution) and store only FREE or
    OCCUPIED. Absence means never observed. Marking is monotone in the safe
    direction: a cell that was observed occupied is never downgraded to free.
    """

    def __init__(self, resolution_m=DEFAULT_RESOLUTION_M):
        self.resolution_m = _finite_float(resolution_m, "resolution_m")
        if not 0.02 <= self.resolution_m <= 0.5:
            raise ValueError("resolution_m must be within [0.02, 0.5]")
        self._cells = {}

    def __len__(self):
        return len(self._cells)

    def index_of(self, x, y):
        """Return the integer cell index of a map-frame XY position."""
        return (
            int(math.floor(_finite_float(x, "x") / self.resolution_m)),
            int(math.floor(_finite_float(y, "y") / self.resolution_m)),
        )

    def center_of(self, ix, iy):
        """Return the world XY center of a cell index."""
        return (
            (int(ix) + 0.5) * self.resolution_m,
            (int(iy) + 0.5) * self.resolution_m,
        )

    def code_at(self, ix, iy):
        """Return FREE, OCCUPIED or UNKNOWN for a cell index."""
        return self._cells.get((int(ix), int(iy)), UNKNOWN)

    def is_empty(self):
        return not self._cells

    def counts(self):
        """Return (occupied_cells, free_cells)."""
        occupied = 0
        free = 0
        for code in self._cells.values():
            if code == OCCUPIED:
                occupied += 1
            elif code == FREE:
                free += 1
        return occupied, free

    def _mark(self, points_xy, code):
        array = np.asarray(points_xy, dtype=float)
        if array.size == 0:
            return 0
        if array.ndim != 2 or array.shape[1] != 2:
            array = array.reshape(-1, 2)
        finite = np.isfinite(array).all(axis=1)
        array = array[finite]
        if array.size == 0:
            return 0
        indices = np.floor(array / self.resolution_m).astype(np.int64)
        unique = np.unique(indices, axis=0)
        cells = self._cells
        for ix, iy in unique:
            key = (int(ix), int(iy))
            if code == OCCUPIED:
                cells[key] = OCCUPIED
            else:
                cells.setdefault(key, FREE)
        return int(unique.shape[0])

    def mark_occupied_points(self, points_xy):
        """Mark the cells of map-frame XY points occupied; return cell count."""
        return self._mark(points_xy, OCCUPIED)

    def mark_free_points(self, points_xy):
        """Mark the cells of observed floor XY points free; return cell count."""
        return self._mark(points_xy, FREE)

    def mark_free_rays(self, origin_xy, target_cells):
        """Mark cells along measured rays from the sensor origin to hits free.

        origin_xy must be the map-frame measured *sensor* origin of the frame
        (measured base pose composed with the declared sensor->base extrinsic),
        not the base origin: points between the base and the sensor were not
        traversed by the ray and must not be carved free. The ray endpoint (the
        obstacle cell) is never marked free. This turns an obstacle hit into
        the traversed free space in front of it, which is what a frontier
        needs, without inventing free space beyond a hit.
        """
        origin = np.asarray(origin_xy, dtype=float).reshape(-1)
        if origin.shape != (2,) or not np.isfinite(origin).all():
            raise ValueError("origin_xy must contain 2 finite numbers")
        ox = float(origin[0])
        oy = float(origin[1])
        resolution = self.resolution_m
        cells = self._cells
        for tx, ty in target_cells:
            target_key = (int(tx), int(ty))
            cx, cy = self.center_of(tx, ty)
            dx = cx - ox
            dy = cy - oy
            distance = math.hypot(dx, dy)
            if distance <= resolution:
                continue
            usable = distance - resolution * 0.5
            steps = int(math.ceil(usable / resolution))
            for step in range(1, steps + 1):
                traveled = step * resolution
                if traveled >= usable:
                    break
                fraction = traveled / distance
                x = ox + dx * fraction
                y = oy + dy * fraction
                key = (int(math.floor(x / resolution)), int(math.floor(y / resolution)))
                if key == target_key:
                    continue
                cells.setdefault(key, FREE)
        return None

    def frontier(self, pose_xy):
        """Return observed-free cells adjacent to unknown, sorted by distance."""
        cells = self._cells
        frontier = []
        for (ix, iy), code in cells.items():
            if code != FREE:
                continue
            for dx, dy in NEIGHBOURS_8:
                if (ix + dx, iy + dy) not in cells:
                    frontier.append((ix, iy))
                    break
        px = float(pose_xy[0])
        py = float(pose_xy[1])
        resolution = self.resolution_m
        frontier.sort(
            key=lambda cell: (
                ((cell[0] + 0.5) * resolution - px) ** 2
                + ((cell[1] + 0.5) * resolution - py) ** 2,
                cell,
            )
        )
        return frontier

    def unknown_boundary_cells(self):
        """Return unknown cells touching any observed cell (lower-bound unknown)."""
        known = self._cells
        unknown = set()
        for ix, iy in known:
            for dx, dy in NEIGHBOURS_8:
                neighbour = (ix + dx, iy + dy)
                if neighbour not in known:
                    unknown.add(neighbour)
        return unknown

    def unknown_estimate_area_m2(self):
        """Return the boundary-unknown area estimate in square metres."""
        return len(self.unknown_boundary_cells()) * self.resolution_m ** 2

    def cells(self):
        """Return a copy of the observed cell code mapping (never unknown)."""
        return dict(self._cells)

    def copy(self):
        """Return an independent copy of this grid."""
        clone = LevelGrid(self.resolution_m)
        clone._cells = dict(self._cells)
        return clone

    def merge(self, other):
        """Merge another grid of the same resolution; occupied wins."""
        if not isinstance(other, LevelGrid):
            raise TypeError("merge expects a LevelGrid")
        if abs(other.resolution_m - self.resolution_m) > 1e-12:
            raise ValueError("cannot merge grids with different resolutions")
        cells = self._cells
        for key, code in other._cells.items():
            if code == OCCUPIED:
                cells[key] = OCCUPIED
            else:
                cells.setdefault(key, FREE)
        return self

    def to_dict(self):
        """Return a JSON-friendly grid summary."""
        occupied, free = self.counts()
        return {
            "resolution_m": self.resolution_m,
            "occupied_cells": occupied,
            "free_cells": free,
            "known_cells": occupied + free,
        }


@dataclass(frozen=True)
class NormalizedPoints:
    """Map-frame points plus the declaration and transform that produced them."""

    points_map: object
    declared_frame: str
    applied_transform: object
    dropped_nonfinite: int = 0
    dropped_range: int = 0


def points_to_map(
    points,
    frame,
    *,
    pose=None,
    extrinsic=None,
    max_range_m=None,
    max_sensor_range_m=DEFAULT_MAX_SENSOR_RANGE_M,
):
    """Transform a declared point cloud into the map frame.

    frame="map": the points already carry the measured map pose; pose is
    optional (and only used for the range filter).
    frame="base": a measured 4x4 base pose is required.
    frame="sensor": a measured base pose and a validated Extrinsic are both
    required; the applied transform is T_map_base @ T_base_sensor.

    A declaration that cannot be honoured raises ValueError: unknown frame,
    missing pose/extrinsic, wrong array shape, or a cloud declared as
    sensor/base but far outside the sensor range (the usual sign that map
    coordinates were passed with the wrong declaration). Non-finite points are
    dropped and counted; points beyond max_range_m from the sensor origin are
    dropped and counted.
    """
    declared = _frame_name(frame)
    array = _as_point_array(points)
    finite = np.isfinite(array).all(axis=1)
    dropped_nonfinite = int((~finite).sum())
    array = array[finite]

    if declared in ("sensor", "base"):
        ranges = np.linalg.norm(array, axis=1) if array.size else np.zeros(0)
        if ranges.size and float(ranges.max()) > float(max_sensor_range_m):
            raise ValueError(
                f"declared {declared!r} frame but point range "
                f"{float(ranges.max()):.3f} m exceeds {float(max_sensor_range_m):.3f} m; "
                "the cloud is probably already in map coordinates"
            )

    if declared == "sensor":
        if extrinsic is None:
            raise ValueError(
                "extrinsic_missing: a sensor-frame cloud requires the "
                "sensor->base extrinsic"
            )
        sensor_to_base = (
            extrinsic if isinstance(extrinsic, Extrinsic) else Extrinsic(extrinsic)
        )
        if pose is None:
            raise ValueError(
                "pose_required: a sensor-frame cloud requires the measured "
                "base pose"
            )
        base_pose = _as_transform(pose, "pose")
        applied = base_pose @ sensor_to_base.matrix
        origin = base_pose[:3, 3]
    elif declared == "base":
        if extrinsic is not None and not isinstance(extrinsic, Extrinsic):
            extrinsic = Extrinsic(extrinsic)
        if pose is None:
            raise ValueError(
                "pose_required: a base-frame cloud requires the measured base pose"
            )
        base_pose = _as_transform(pose, "pose")
        applied = base_pose
        origin = base_pose[:3, 3]
    else:
        applied = np.eye(4, dtype=float)
        base_pose = None if pose is None else _as_transform(pose, "pose")
        origin = None if base_pose is None else base_pose[:3, 3]

    if array.shape[0] == 0:
        points_map = np.zeros((0, 3), dtype=float)
        return NormalizedPoints(points_map, declared, applied, dropped_nonfinite, 0)

    homogeneous = np.hstack([array, np.ones((array.shape[0], 1))])
    points_map = (applied @ homogeneous.T).T[:, :3]
    dropped_range = 0
    if max_range_m is not None and origin is not None:
        limit = _finite_float(max_range_m, "max_range_m")
        distances = np.linalg.norm(points_map - np.asarray(origin, dtype=float), axis=1)
        keep = distances <= limit
        dropped_range = int((~keep).sum())
        points_map = points_map[keep]
    return NormalizedPoints(
        np.array(points_map, dtype=float, copy=True),
        declared,
        np.array(applied, dtype=float, copy=True),
        dropped_nonfinite,
        dropped_range,
    )


def depth_image_to_points(
    depth,
    *,
    fx,
    fy,
    cx,
    cy,
    stride=1,
    min_range_m=0.05,
    max_range_m=DEFAULT_MAX_RANGE_M,
):
    """Convert a depth image to sensor-frame (N, 3) points, numpy only.

    Points are returned in the depth camera optical convention: x right,
    y down, z forward. The caller's explicit Extrinsic is responsible for
    the optical->base rotation, so this helper does not bake in a mount.
    Zero, non-finite and out-of-range depths are dropped.
    """
    image = np.asarray(depth, dtype=float)
    if image.ndim != 2:
        raise ValueError("depth must be a 2D array")
    for name, value in (("fx", fx), ("fy", fy), ("cx", cx), ("cy", cy)):
        _finite_float(value, name)
    step = int(stride)
    if step < 1:
        raise ValueError("stride must be at least 1")
    height, width = image.shape
    rows = np.arange(0, height, step)
    columns = np.arange(0, width, step)
    uu, vv = np.meshgrid(columns, rows)
    zz = image[vv, uu]
    valid = np.isfinite(zz) & (zz > float(min_range_m)) & (zz <= float(max_range_m))
    z_valid = zz[valid]
    if z_valid.size == 0:
        return np.zeros((0, 3), dtype=float)
    u_valid = uu[valid].astype(float)
    v_valid = vv[valid].astype(float)
    x = (u_valid - float(cx)) * z_valid / float(fx)
    y = (v_valid - float(cy)) * z_valid / float(fy)
    return np.stack([x, y, z_valid], axis=1)


class OccupancyAccumulator:
    """Multi-frame, per-level operator map accumulated from measured frames.

    The accumulator owns a PosePairer, a LevelTracker, one sparse LevelGrid
    per level and a bounded sample of each level's observed z for the floor
    estimate. add_frame is the only grid mutator (add_pose feeds the pairing
    stream and merge combines grids); a frame is accepted only when it can be
    transformed (measured pose + valid extrinsic) and when its pose z resolves
    to one unambiguous level. Rejected frames never change the grids but are
    recorded as the current gate evidence, so a snapshot can explain why it is
    not actionable.

    snapshot(level) returns the map for one level. It is not the same thing
    as the accumulated map unless more than one frame was accepted; the
    snapshot_is_accumulated_map and accumulated_frames fields make that
    distinction explicit.
    """

    def __init__(
        self,
        *,
        extrinsic=None,
        resolution_m=DEFAULT_RESOLUTION_M,
        level_tracker=None,
        max_pair_dt_s=DEFAULT_MAX_PAIR_DT_S,
        min_points=DEFAULT_MIN_POINTS,
        max_range_m=DEFAULT_MAX_RANGE_M,
        max_sensor_range_m=DEFAULT_MAX_SENSOR_RANGE_M,
        floor_quantile=DEFAULT_FLOOR_QUANTILE,
        max_height_m=DEFAULT_MAX_HEIGHT_M,
        free_mode=DEFAULT_FREE_MODE,
        map_frame=DEFAULT_MAP_FRAME,
        z_sample_limit=20000,
    ):
        # An absent extrinsic is allowed here only so a caller can obtain a
        # refusal receipt instead of an exception. Every snapshot is then
        # non-actionable with reason extrinsic_missing/extrinsic_invalid.
        self.extrinsic = None if extrinsic is None else _as_extrinsic(extrinsic)
        self.resolution_m = _finite_float(resolution_m, "resolution_m")
        if not 0.02 <= self.resolution_m <= 0.5:
            raise ValueError("resolution_m must be within [0.02, 0.5]")
        if level_tracker is not None and not isinstance(level_tracker, LevelTracker):
            raise ValueError("level_tracker must be a LevelTracker")
        self.level_tracker = (
            level_tracker if isinstance(level_tracker, LevelTracker) else LevelTracker()
        )
        self.max_pair_dt_s = _finite_float(max_pair_dt_s, "max_pair_dt_s")
        if self.max_pair_dt_s <= 0.0:
            raise ValueError("max_pair_dt_s must be positive")
        self.min_points = int(min_points)
        if self.min_points < 1:
            raise ValueError("min_points must be at least 1")
        self.max_range_m = _finite_float(max_range_m, "max_range_m")
        if self.max_range_m <= 0.0:
            raise ValueError("max_range_m must be positive")
        self.max_sensor_range_m = _finite_float(
            max_sensor_range_m, "max_sensor_range_m"
        )
        self.floor_quantile = _finite_float(floor_quantile, "floor_quantile")
        if not 0.0 < self.floor_quantile <= 0.5:
            raise ValueError("floor_quantile must be within (0, 0.5]")
        self.max_height_m = _finite_float(max_height_m, "max_height_m")
        if self.max_height_m <= MIN_OBSTACLE_CLEARANCE_M:
            raise ValueError("max_height_m must exceed the 0.15 m obstacle clearance")
        self.free_mode = _free_mode(free_mode)
        if not isinstance(map_frame, str) or not map_frame.strip():
            raise ValueError("map_frame must be a non-empty string")
        self.map_frame = map_frame.strip()
        self._z_sample_limit = int(z_sample_limit)
        if self._z_sample_limit < 1:
            raise ValueError("z_sample_limit must be at least 1")

        self._pairer = PosePairer(max_pair_dt_s=self.max_pair_dt_s)
        self._grids = {}
        self._level_z = {}
        self._z_samples = {}
        self._accepted_frames = 0
        self._accepted_points = 0
        self._level_point_counts = {}
        self._last_declared_cloud_frame = None
        self._frames = []
        self._path_length_m = 0.0
        self._last_pose = None
        self._last_pose_stamp_s = None
        self._last_pairing = None
        self._last_level_match = None
        self._last_ages = (None, None)
        self._last_stamps = {"pose": None, "cloud": None, "paired_dt": None}
        self._last_point_count = 0
        self._last_level_point_count = 0
        self._last_pose_source = "lio_estimate"
        self._last_cloud_source = "depth"
        self._last_reasons = ()
        # Credible ground evidence, per level: the established floor and the
        # evidence of the most recent frame that resolved to that level. A
        # level with missing evidence is not actionable even if it has cells.
        self._level_floor_z = {}
        self._level_floor_evidence = {}
        # Learned from the first evidenced floor in this generation. This is
        # only a candidate-search window, never a stand/contact proof. It
        # prevents a lower landing seen down a stairwell from becoming the
        # first anchor of a newly entered height band.
        self._measured_floor_standoff_m = None
        # Freshness references, per clock and per stream: the last accepted
        # frame's cloud stamp/arrival and the pose stream's latest stamp and
        # arrival. The source sim clock and the arrival monotonic clock are
        # separate and are never substituted for each other.
        self._last_valid_cloud_sim_s = None
        self._last_valid_cloud_wall_s = None
        self._pose_stream_sim_s = None
        self._pose_stream_wall_s = None
        # True when the pose stream's latest arrival could not be evaluated
        # (no arrival recorded, or a present but non-finite one). It is kept
        # apart from "no sample yet" so a bogus arrival is never replaced by an
        # older fresh-looking age.
        self._pose_stream_wall_invalid = False
        # Per level: the last accepted observation of that level (cloud stamp,
        # arrival and ingestion sim age, plus the pose stamp/source that
        # produced it). Fresh data on another level must never refresh this
        # level's own age.
        self._level_observations = {}
        self._untrusted_reason = None
        self._untrusted_source = None
        # Per level: the outcome of the most recent frame that resolved to the
        # level (accepted or refused). A refused latest frame keeps the old
        # grid but makes the level non-actionable until a new frame lands.
        self._level_frame_status = {}
        # Previous caller-provided now per clock, so a rolled-back clock is
        # reported instead of silently refreshing an old snapshot.
        self._last_snapshot_sim_now_s = None
        self._last_snapshot_wall_now_s = None

    # -- read-only state -------------------------------------------------

    @property
    def accepted_frames(self):
        return self._accepted_frames

    @property
    def measured_path_length_m(self):
        return self._path_length_m

    @property
    def frames(self):
        """Return the accepted frame receipts (bounded history)."""
        return tuple(self._frames)

    @property
    def levels(self):
        """Return (level_index, level_z_m) pairs sorted by z."""
        return tuple(
            (index, self._level_z[index])
            for index in sorted(self._level_z, key=self._level_z.get)
        )

    def level_z_m(self, level_index):
        """Return the accumulated level z for one level index, or None."""
        return self._level_z.get(level_index)

    def grid_for(self, level_index):
        """Return the level grid (a live object) or None."""
        return self._grids.get(level_index)

    # -- pose stream -----------------------------------------------------

    def preview_pose_pair(self, stamp_s):
        """Return the PosePairer outcome for one cloud stamp without recording it.

        Uses the same pairer instance and the same strict max_pair_dt_s as add_frame,
        so the preview cannot disagree with the real pairing and cannot mutate the
        pairer's last outcome or any map state.
        """
        return self._pairer.preview(stamp_s)

    def add_pose(
        self, stamp_s, pose, *, pose_source=None, arrival_wall_s=None
    ):
        """Register one measured estimator pose for future cloud pairing.

        arrival_wall_s is the arrival monotonic time of that pose sample. It
        must be on time.monotonic() (the same clock snapshot() compares
        against when the caller passes no wall now). It is used only to
        re-evaluate the pose stream's wall age at snapshot time, so a stream
        that stops advancing becomes stale even when the source sim clock
        itself is frozen. A present but non-finite arrival is recorded as
        unevaluable rather than clipped to now. A later stamp becomes the pose
        stream reference; an older out-of-order sample never moves it
        backwards.
        """
        if pose_source is not None:
            _check_source(pose_source, FORBIDDEN_POSE_SOURCES, "pose_source")
        stamp = _finite_float(stamp_s, "pose stamp")
        if arrival_wall_s is None:
            arrival = time.monotonic()
            arrival_invalid = False
        else:
            arrival = _optional_finite_float(arrival_wall_s, "arrival_wall_s")
            arrival_invalid = arrival is None
        self._pairer.add(stamp, pose)
        if self._pose_stream_sim_s is None or stamp >= self._pose_stream_sim_s:
            self._pose_stream_sim_s = stamp
            self._pose_stream_wall_s = arrival
            self._pose_stream_wall_invalid = arrival_invalid
        return None

    def _gate_state(
        self,
        *,
        ages,
        level_match,
        pairing,
        extrinsic_ok,
        point_count,
        level_available,
        minimum_points=None,
        cloud_ages=None,
    ):
        """Return (gates, reasons) for one frame evidence set.

        cloud_ages is optional (sim_age, wall_age) for the cloud stream. The
        receipt paths of add_frame/refusal do not pass it (a single ingestion
        is not a snapshot age); every snapshot passes it, including a snapshot
        with no caller now, so the cloud stream is always re-evaluated. Both
        cloud components must be evaluable: a missing component makes the age
        unevaluable (cloud_age_unevaluated) and a NaN/Inf component is never
        compared (cloud_age_invalid). Cloud staleness is folded into the
        existing pose_fresh gate so that actionable == all(gates.values())
        stays true, and it always adds its own explicit reason.
        """
        minimum = self.min_points if minimum_points is None else int(minimum_points)
        reasons = []

        def add_reason(reason):
            if reason not in reasons:
                reasons.append(reason)

        sim_age, wall_age = ages
        pose_fresh = True
        if sim_age is None or wall_age is None:
            pose_fresh = False
            add_reason("pose_age_unknown")
        elif not (
            isinstance(sim_age, (int, float, np.integer, np.floating))
            and isinstance(wall_age, (int, float, np.integer, np.floating))
        ):
            pose_fresh = False
            add_reason(REASON_POSE_AGE_INVALID)
        else:
            sim_age = float(sim_age)
            wall_age = float(wall_age)
            if not math.isfinite(sim_age) or not math.isfinite(wall_age):
                # NaN/Inf never pass a threshold comparison.
                pose_fresh = False
                add_reason(REASON_POSE_AGE_INVALID)
            else:
                if sim_age < -self.max_pair_dt_s:
                    pose_fresh = False
                    add_reason(REASON_SIM_CLOCK_REGRESSION)
                elif sim_age > DEFAULT_MAX_POSE_SIM_AGE_S:
                    pose_fresh = False
                    add_reason("stale_pose_sim_age")
                if wall_age < 0.0:
                    pose_fresh = False
                    add_reason(REASON_WALL_CLOCK_REGRESSION)
                elif wall_age > DEFAULT_MAX_POSE_WALL_AGE_S:
                    pose_fresh = False
                    add_reason("stale_pose_wall_age")
        if cloud_ages is not None:
            cloud_sim_age, cloud_wall_age = cloud_ages
            if cloud_sim_age is None or cloud_wall_age is None:
                pose_fresh = False
                add_reason(REASON_CLOUD_AGE_UNEVALUATED)
            elif not (
                isinstance(
                    cloud_sim_age, (int, float, np.integer, np.floating)
                )
                and isinstance(
                    cloud_wall_age, (int, float, np.integer, np.floating)
                )
            ):
                pose_fresh = False
                add_reason(REASON_CLOUD_AGE_INVALID)
            else:
                cloud_sim_age = float(cloud_sim_age)
                cloud_wall_age = float(cloud_wall_age)
                if not math.isfinite(cloud_sim_age) or not math.isfinite(
                    cloud_wall_age
                ):
                    pose_fresh = False
                    add_reason(REASON_CLOUD_AGE_INVALID)
                else:
                    if cloud_sim_age < -self.max_pair_dt_s:
                        pose_fresh = False
                        add_reason(REASON_SIM_CLOCK_REGRESSION)
                    elif cloud_sim_age > DEFAULT_MAX_POSE_SIM_AGE_S:
                        pose_fresh = False
                        add_reason(REASON_STALE_CLOUD_SIM_AGE)
                    if cloud_wall_age < 0.0:
                        pose_fresh = False
                        add_reason(REASON_WALL_CLOCK_REGRESSION)
                    elif cloud_wall_age > DEFAULT_MAX_POSE_WALL_AGE_S:
                        pose_fresh = False
                        add_reason(REASON_STALE_CLOUD_WALL_AGE)
        pairing_ok = bool(
            pairing is not None
            and pairing.ok
            and pairing.dt_s is not None
            and abs(float(pairing.dt_s)) <= self.max_pair_dt_s
        )
        if not pairing_ok:
            reasons.append("pairing_gap")
        if not extrinsic_ok:
            reasons.append("extrinsic_invalid")
        if level_match is not None and level_match.ambiguous:
            level_clear = False
            reasons.append("ambiguous_level")
        elif not level_available:
            level_clear = False
            if level_match is not None and level_match.reasons:
                reasons.extend(level_match.reasons)
            else:
                reasons.append("level_unresolved")
        else:
            level_clear = True
        points_ok = int(point_count) >= minimum
        if not points_ok:
            reasons.append("insufficient_points")
        gates = {
            "pose_fresh": pose_fresh,
            "pairing": pairing_ok,
            "extrinsic": bool(extrinsic_ok),
            "level": level_clear,
            "points": points_ok,
        }
        return gates, reasons

    # -- credible ground evidence ----------------------------------------

    def _assess_floor_evidence(
        self, level_index, ground_points, base_z, stand_proof_z_m=None
    ):
        """Return the floor-evidence receipt for one frame and level.

        The candidate is the low quantile of the points strictly below this
        frame's measured base origin and within the existing floor search
        depth. Being below the base and having support points is only a
        candidate: a kerb, step edge, pallet or low panel also satisfies that.
        The candidate becomes the floor only through one of three independent
        measured evidence routes:

        * established_floor - the candidate agrees with the floor already
          established for this level AND at least MIN_GROUND_SUPPORT_POINTS of
          the support band lie inside that floor's free band.
        * measured_stand_proof - the caller supplied a measured body-support
          height for this frame (e.g. from leg odometry or contact) and the
          candidate agrees with it. This is measured evidence, not a
          ground-truth or building floor constant.
        * measured_standoff_and_extent - used only for a level's first floor:
          the measured base stands at least MIN_BODY_SUPPORT_STANDOFF_M above
          the candidate and the supporting surface is observed across at least
          MIN_GROUND_SUPPORT_SPAN_M in both horizontal axes, so a narrow ledge
          or a low step is not mistaken for a floor.

        Otherwise the frame is refused with floor_evidence_unconfirmed and the
        caller must not classify any cell: an unproven floor may never make a
        cell free or occupied. Every input here is the measured pose z, an
        observed point z or a caller-measured stand proof; no ground truth and
        no building floor height is ever used.
        """
        proof = (
            None
            if stand_proof_z_m is None
            else _finite_float(stand_proof_z_m, "stand_proof_z_m")
        )
        points = np.asarray(ground_points, dtype=float)
        established = self._level_floor_z.get(level_index)
        evidence = {
            "ok": False,
            "reason": REASON_FLOOR_EVIDENCE_MISSING,
            "candidate_floor_z_m": None,
            "floor_z_m": None,
            "established_floor_z_m": established,
            "measured_base_z_m": float(base_z),
            "support_points": 0,
            "min_support_points": int(MIN_GROUND_SUPPORT_POINTS),
            "consistency_m": float(GROUND_FLOOR_CONSISTENCY_M),
            "points_below_base": int(points.shape[0]),
            "standoff_m": None,
            "min_standoff_m": float(MIN_BODY_SUPPORT_STANDOFF_M),
            "support_span_x_m": None,
            "support_span_y_m": None,
            "min_support_span_m": float(MIN_GROUND_SUPPORT_SPAN_M),
            "support_points_in_established_free_band": 0,
            "stand_proof_z_m": proof,
            "stand_proof_consistency_m": float(STAND_PROOF_CONSISTENCY_M),
            "evidence_route": None,
        }
        if points.shape[0] == 0:
            return evidence
        candidate = float(np.quantile(points[:, 2], self.floor_quantile))
        evidence["candidate_floor_z_m"] = candidate
        if not math.isfinite(candidate):
            evidence["reason"] = "floor_estimate_gap"
            return evidence
        support = points[
            (points[:, 2] >= candidate - FREE_FLOOR_BELOW_M)
            & (points[:, 2] < candidate + MIN_OBSTACLE_CLEARANCE_M)
        ]
        evidence["support_points"] = int(support.shape[0])
        evidence["standoff_m"] = float(base_z) - candidate
        if support.shape[0] > 0:
            evidence["support_span_x_m"] = float(np.ptp(support[:, 0]))
            evidence["support_span_y_m"] = float(np.ptp(support[:, 1]))
        if evidence["support_points"] < MIN_GROUND_SUPPORT_POINTS:
            # A sparse tail (a wall edge, a single beam) is not a supporting
            # surface at all.
            return evidence
        if established is not None:
            if abs(candidate - float(established)) > GROUND_FLOOR_CONSISTENCY_M:
                evidence["reason"] = REASON_FLOOR_EVIDENCE_INCONSISTENT
                return evidence
            free_low = float(established) - FREE_FLOOR_BELOW_M
            free_high = float(established) + MIN_OBSTACLE_CLEARANCE_M
            inside = support[
                (support[:, 2] >= free_low) & (support[:, 2] < free_high)
            ]
            evidence["support_points_in_established_free_band"] = int(
                inside.shape[0]
            )
            if inside.shape[0] < MIN_GROUND_SUPPORT_POINTS:
                # The candidate is near the established floor, but the
                # surface it was estimated from is not inside the free band
                # that floor already established. It may be a kerb or a
                # step edge sitting on top of that floor.
                evidence["reason"] = REASON_FLOOR_EVIDENCE_UNCONFIRMED
                return evidence
            route = "established_floor"
        elif proof is not None:
            if abs(candidate - proof) > STAND_PROOF_CONSISTENCY_M:
                evidence["reason"] = REASON_FLOOR_EVIDENCE_UNCONFIRMED
                return evidence
            route = "measured_stand_proof"
        else:
            if evidence["standoff_m"] < MIN_BODY_SUPPORT_STANDOFF_M - 1e-9:
                # "Below the base by one obstacle clearance" is not enough:
                # a 0.15 m kerb/pallet/step edge would qualify. Without an
                # established floor or a measured stand proof, require a
                # body-support standoff of two obstacle clearances.
                evidence["reason"] = REASON_FLOOR_EVIDENCE_UNCONFIRMED
                return evidence
            span_x = evidence["support_span_x_m"] or 0.0
            span_y = evidence["support_span_y_m"] or 0.0
            if min(span_x, span_y) < MIN_GROUND_SUPPORT_SPAN_M:
                # A long narrow surface is a ledge/kerb edge, not an observed
                # floor patch the body can be standing on.
                evidence["reason"] = REASON_FLOOR_EVIDENCE_UNCONFIRMED
                return evidence
            route = "measured_standoff_and_extent"
        evidence["ok"] = True
        evidence["reason"] = None
        # The classification reference is the credible anchor, not this frame's
        # low quantile: once a level floor is established, or a measured stand
        # proof exists, the candidate must never shift the free/occupied band.
        if route == "established_floor":
            evidence["floor_z_m"] = float(established)
        elif route == "measured_stand_proof":
            evidence["floor_z_m"] = float(proof)
        else:
            evidence["floor_z_m"] = candidate
        evidence["evidence_route"] = route
        return evidence

    @staticmethod
    def _floor_evidence_receipt(evidence):
        """Return the JSON-friendly floor evidence of one frame or snapshot."""
        if evidence is None:
            return {
                "ok": False,
                "reason": REASON_FLOOR_EVIDENCE_UNKNOWN,
                "candidate_floor_z_m": None,
                "floor_z_m": None,
                "established_floor_z_m": None,
                "measured_base_z_m": None,
                "support_points": 0,
                "min_support_points": int(MIN_GROUND_SUPPORT_POINTS),
                "consistency_m": float(GROUND_FLOOR_CONSISTENCY_M),
                "points_below_base": 0,
                "standoff_m": None,
                "min_standoff_m": float(MIN_BODY_SUPPORT_STANDOFF_M),
                "support_span_x_m": None,
                "support_span_y_m": None,
                "min_support_span_m": float(MIN_GROUND_SUPPORT_SPAN_M),
                "support_points_in_established_free_band": 0,
                "stand_proof_z_m": None,
                "stand_proof_consistency_m": float(STAND_PROOF_CONSISTENCY_M),
                "evidence_route": None,
            }
        return {
            "ok": bool(evidence.get("ok")),
            "reason": evidence.get("reason"),
            "candidate_floor_z_m": _round(evidence.get("candidate_floor_z_m")),
            "floor_z_m": _round(evidence.get("floor_z_m")),
            "established_floor_z_m": _round(evidence.get("established_floor_z_m")),
            "measured_base_z_m": _round(evidence.get("measured_base_z_m")),
            "support_points": int(evidence.get("support_points", 0)),
            "min_support_points": int(MIN_GROUND_SUPPORT_POINTS),
            "consistency_m": float(GROUND_FLOOR_CONSISTENCY_M),
            "points_below_base": int(evidence.get("points_below_base", 0)),
            "standoff_m": _round(evidence.get("standoff_m")),
            "min_standoff_m": float(MIN_BODY_SUPPORT_STANDOFF_M),
            "support_span_x_m": _round(evidence.get("support_span_x_m")),
            "support_span_y_m": _round(evidence.get("support_span_y_m")),
            "min_support_span_m": float(MIN_GROUND_SUPPORT_SPAN_M),
            "support_points_in_established_free_band": int(
                evidence.get("support_points_in_established_free_band", 0)
            ),
            "stand_proof_z_m": _round(evidence.get("stand_proof_z_m")),
            "stand_proof_consistency_m": float(STAND_PROOF_CONSISTENCY_M),
            "evidence_route": evidence.get("evidence_route"),
        }

    # -- snapshot freshness ----------------------------------------------

    def _resolve_snapshot_ages(
        self,
        *,
        sim_age_s,
        wall_age_s,
        now_sim_s,
        now_wall_s,
        target_level=None,
    ):
        """Re-evaluate and return both stream ages from the caller's clocks.

        Clock authority: every arrival is recorded on time.monotonic(). A
        caller-supplied now_wall_s must be that same monotonic clock, while
        now_sim_s is the source/estimator clock of the ingested stamps. The
        two clocks are separate and are never substituted for each other.

        Every call re-derives the ages; no cached now and no cached age is
        reused. The cloud stream reference is the target level's own last
        accepted observation (source stamp, arrival and ingestion sim age),
        falling back to the accumulator-wide cloud reference only when this
        level has none, so fresh data on another level cannot make this level
        look fresh. When the caller passes no now_wall_s the recorded arrival
        is compared with time.monotonic(). When it passes no now_sim_s the
        recorded ingestion sim age is reported, because this process has no
        authoritative source clock. A present but non-finite now is not
        clipped to a number: the affected age becomes None and the snapshot
        fails closed with clock_invalid.

        Returns ((pose_sim_age, pose_wall_age), (cloud_sim_age, cloud_wall_age),
        regressions, freshness_facts). The cloud tuple is always returned, so
        every snapshot - including one with no caller now - re-evaluates the
        cloud stream instead of skipping it.
        """
        now_sim_supplied = now_sim_s is not None
        now_wall_supplied = now_wall_s is not None
        now_sim = _optional_finite_float(now_sim_s, "now_sim_s")
        now_wall = _optional_finite_float(now_wall_s, "now_wall_s")
        regressions = []
        if now_sim_supplied and now_sim is None:
            regressions.append(REASON_CLOCK_INVALID)
        if (
            now_wall_supplied
            and now_wall is None
            and REASON_CLOCK_INVALID not in regressions
        ):
            regressions.append(REASON_CLOCK_INVALID)
        observation = (
            self._level_observations.get(target_level)
            if target_level is not None
            else None
        )
        level_sim_ref = (
            None if observation is None else observation.get("cloud_stamp_s")
        )
        level_wall_ref = (
            None if observation is None else observation.get("cloud_arrival_wall_s")
        )
        level_recorded_sim_age = (
            None if observation is None else observation.get("cloud_sim_age_s")
        )
        if level_sim_ref is None:
            level_sim_ref = self._last_valid_cloud_sim_s
        if level_wall_ref is None:
            level_wall_ref = self._last_valid_cloud_wall_s
        if now_sim_supplied:
            pose_sim_age = (
                None
                if (now_sim is None or self._pose_stream_sim_s is None)
                else now_sim - float(self._pose_stream_sim_s)
            )
        elif sim_age_s is not None:
            pose_sim_age = _optional_finite_float(sim_age_s, "sim_age_s")
            if pose_sim_age is None:
                regressions.append(REASON_POSE_AGE_INVALID)
        elif observation is not None:
            pose_sim_age = (
                None
                if level_recorded_sim_age is None
                else float(level_recorded_sim_age)
            )
        else:
            pose_sim_age = self._last_ages[0]
        if now_wall_supplied:
            pose_wall_age = (
                None
                if (now_wall is None or self._pose_stream_wall_s is None)
                else now_wall - float(self._pose_stream_wall_s)
            )
        elif wall_age_s is not None:
            pose_wall_age = _optional_finite_float(wall_age_s, "wall_age_s")
            if pose_wall_age is None:
                regressions.append(REASON_POSE_AGE_INVALID)
        elif self._pose_stream_wall_s is not None:
            pose_wall_age = time.monotonic() - float(self._pose_stream_wall_s)
        elif self._pose_stream_wall_invalid:
            pose_wall_age = None
            if REASON_POSE_AGE_INVALID not in regressions:
                regressions.append(REASON_POSE_AGE_INVALID)
        else:
            pose_wall_age = self._last_ages[1]
        # Cloud stream ages: always resolved, independent of the caller now.
        if now_sim_supplied:
            cloud_sim_age = (
                None
                if (now_sim is None or level_sim_ref is None)
                else now_sim - float(level_sim_ref)
            )
        elif level_recorded_sim_age is not None:
            cloud_sim_age = float(level_recorded_sim_age)
        else:
            cloud_sim_age = None
        if now_wall_supplied:
            cloud_wall_age = (
                None
                if (now_wall is None or level_wall_ref is None)
                else now_wall - float(level_wall_ref)
            )
        elif level_wall_ref is not None:
            cloud_wall_age = time.monotonic() - float(level_wall_ref)
        else:
            cloud_wall_age = None
        cloud_ages = (cloud_sim_age, cloud_wall_age)
        if now_sim is not None:
            if (
                self._last_snapshot_sim_now_s is not None
                and now_sim < self._last_snapshot_sim_now_s
            ):
                regressions.append(REASON_SIM_CLOCK_REGRESSION)
            else:
                self._last_snapshot_sim_now_s = now_sim
        if now_wall is not None:
            if (
                self._last_snapshot_wall_now_s is not None
                and now_wall < self._last_snapshot_wall_now_s
            ):
                regressions.append(REASON_WALL_CLOCK_REGRESSION)
            else:
                self._last_snapshot_wall_now_s = now_wall
        freshness = {
            "now_sim_s": _round(now_sim),
            "now_wall_s": _round(now_wall),
            "now_source": (
                "caller"
                if (now_sim_supplied or now_wall_supplied)
                else "recorded_arrival"
            ),
            "pose_reference_sim_s": _round(self._pose_stream_sim_s),
            "pose_reference_wall_s": _round(self._pose_stream_wall_s),
            "cloud_reference_sim_s": _round(level_sim_ref),
            "cloud_reference_wall_s": _round(level_wall_ref),
            "level_sim_age_s": _round(level_recorded_sim_age),
            "target_level": None if target_level is None else int(target_level),
            "level_pose_source": (
                None if observation is None else observation.get("pose_source")
            ),
            "level_cloud_source": (
                None if observation is None else observation.get("cloud_source")
            ),
            "pose_sim_age_s": _round(pose_sim_age),
            "pose_wall_age_s": _round(pose_wall_age),
            "cloud_sim_age_s": _round(cloud_sim_age),
            "cloud_wall_age_s": _round(cloud_wall_age),
            "regressions": list(regressions),
        }
        return (pose_sim_age, pose_wall_age), cloud_ages, regressions, freshness

    def _update_current(
        self,
        *,
        pose,
        pose_stamp_s,
        cloud_stamp_s,
        pairing,
        level_match,
        ages,
        point_count,
        level_point_count,
        pose_source,
        cloud_source,
        reasons,
    ):
        if pose is not None:
            self._last_pose = np.array(pose, dtype=float, copy=True)
        self._last_pose_stamp_s = pose_stamp_s
        self._last_pairing = pairing
        self._last_level_match = level_match
        self._last_ages = ages
        self._last_stamps = {
            "pose": pose_stamp_s,
            "cloud": cloud_stamp_s,
            "paired_dt": pairing.dt_s if pairing is not None else None,
        }
        self._last_point_count = int(point_count)
        self._last_level_point_count = int(level_point_count)
        self._last_pose_source = pose_source
        self._last_cloud_source = cloud_source
        self._last_reasons = tuple(reasons)

    def _record_level_frame(
        self,
        level_index,
        *,
        accepted,
        reasons,
        cloud_stamp_s=None,
        arrival_wall_s=None,
        pose_stamp_s=None,
        pose_source=None,
        cloud_source=None,
    ):
        """Record the latest frame outcome that resolved to one level.

        A refused frame keeps the previously observed grid geometry, but this
        record is what a later snapshot uses to decide whether the level is
        *currently* actionable. It is per level on purpose: a refusal on one
        level must not make another level's fresh map look stale, and fresh
        data on another level must not revive a refused level.
        """
        if level_index is None:
            return
        self._level_frame_status[int(level_index)] = {
            "accepted": bool(accepted),
            "reasons": tuple(reason for reason in reasons if reason),
            "cloud_stamp_s": (
                None if cloud_stamp_s is None else float(cloud_stamp_s)
            ),
            "arrival_wall_s": (
                None if arrival_wall_s is None else float(arrival_wall_s)
            ),
            "pose_stamp_s": None if pose_stamp_s is None else float(pose_stamp_s),
            "pose_source": pose_source,
            "cloud_source": cloud_source,
        }

    def _refusal(
        self,
        reasons,
        *,
        pose=None,
        pose_stamp_s=None,
        cloud_stamp_s=None,
        pairing=None,
        level_match=None,
        ages=(None, None),
        point_count=0,
        level_point_count=0,
        pose_source=None,
        cloud_source=None,
        floor_evidence=None,
        level_index=None,
    ):
        reasons = list(reasons)
        self._update_current(
            pose=pose,
            pose_stamp_s=pose_stamp_s,
            cloud_stamp_s=cloud_stamp_s,
            pairing=pairing,
            level_match=level_match,
            ages=ages,
            point_count=point_count,
            level_point_count=level_point_count,
            pose_source=pose_source or self._last_pose_source,
            cloud_source=cloud_source or self._last_cloud_source,
            reasons=reasons,
        )
        # A frame that resolved to a level but was refused still records the
        # refusal for that level: the grid is kept, the level's *current*
        # actionability is not.
        self._record_level_frame(
            level_index,
            accepted=False,
            reasons=reasons,
            cloud_stamp_s=cloud_stamp_s,
            pose_stamp_s=pose_stamp_s,
            pose_source=pose_source,
            cloud_source=cloud_source,
        )
        gates, _ = self._gate_state(
            ages=ages,
            level_match=level_match,
            pairing=pairing,
            extrinsic_ok=self.extrinsic is not None,
            point_count=level_point_count,
            level_available=False,
        )
        return {
            "accepted": False,
            "reasons": reasons,
            "level_index": None,
            "refused_level_index": (
                None if level_index is None else int(level_index)
            ),
            "level_z_m": None,
            "stamps": dict(self._last_stamps),
            "point_count": 0,
            "level_point_count": 0,
            "gates": dict(gates),
            "floor_evidence": self._floor_evidence_receipt(floor_evidence),
            "note": "frame refused; grids unchanged",
        }

    # -- frame ingestion -------------------------------------------------

    def add_frame(
        self,
        points,
        *,
        frame,
        cloud_stamp_s=None,
        pose=None,
        pose_stamp_s=None,
        sim_age_s=None,
        wall_age_s=None,
        arrival_wall_s=None,
        pose_arrival_wall_s=None,
        pose_source="lio_estimate",
        cloud_source="depth",
        min_points=None,
        free_mode=None,
        pose_pairer=None,
        stand_proof_z_m=None,
        note=None,
    ):
        """Accumulate one measured point cloud into its unambiguous level.

        pose plus pose_stamp_s is the explicit measured pose; when omitted,
        the internal PosePairer (fed by add_pose) or a caller supplied
        pose_pairer provides it. A pairing gap wider than max_pair_dt_s does
        not prevent mapping an explicitly supplied measured pose, but it marks
        the resulting frontier non-actionable.

        Clock authority: arrival_wall_s and pose_arrival_wall_s are arrival
        times on time.monotonic(); that is the clock snapshot() compares
        against when the caller passes no now. They are recorded so a later
        snapshot() can re-evaluate wall ages against the current time instead
        of reusing the age measured at ingestion. When neither is given, the
        arrival is recorded from time.monotonic(); when wall_age_s is given
        without arrival_wall_s, the arrival is reconstructed as monotonic now
        minus that age so the recorded age is never understated. A present but
        non-finite age or arrival is recorded as unevaluable (never clipped to
        zero) and the corresponding gate fails closed.

        stand_proof_z_m is optional caller-measured evidence of the
        body-support height (e.g. from leg odometry or a contact sensor). It is
        only used as a floor evidence route; it is never a ground-truth or
        building floor constant.

        Trust boundary: this proof is trusted as caller-supplied measurement,
        exactly like the pose. This module cannot prove that the proof was
        measured rather than synthesised, so a caller that passes an arbitrary
        stand proof can turn a low surface into a floor. The interactive client
        and the tracker never pass it, and a navigation caller must not invent
        one: a real proof has to be traced to the current stand gate, contact
        state or a recorded calibration, never to ground truth or a static
        building height.

        The frame is only accumulated when it carries credible ground evidence
        for its level; without it the frame is refused and no cell changes.
        """
        declared_frame = _frame_name(frame)
        self._last_declared_cloud_frame = declared_frame
        cloud_stamp = _finite_float(cloud_stamp_s, "cloud_stamp_s")
        pose_source = _check_source(pose_source, FORBIDDEN_POSE_SOURCES, "pose_source")
        cloud_source = _check_source(
            validate_map_cloud_source(cloud_source), FORBIDDEN_CLOUD_SOURCES, "cloud_source"
        )
        mode = _free_mode(free_mode) if free_mode is not None else self.free_mode
        minimum = self.min_points if min_points is None else int(min_points)
        if minimum < 1:
            raise ValueError("min_points must be at least 1")
        sim_age = _optional_finite_float(sim_age_s, "sim_age_s")
        wall_age = _optional_finite_float(wall_age_s, "wall_age_s")
        ages = (sim_age, wall_age)
        age_reasons = []
        if sim_age_s is not None and sim_age is None:
            age_reasons.append(REASON_POSE_AGE_INVALID)
        if wall_age_s is not None and wall_age is None:
            age_reasons.append(REASON_CLOUD_AGE_INVALID)
        if arrival_wall_s is not None:
            arrival_wall = _optional_finite_float(
                arrival_wall_s, "arrival_wall_s"
            )
            if arrival_wall is None:
                age_reasons.append(REASON_CLOUD_AGE_INVALID)
        elif wall_age_s is None:
            arrival_wall = time.monotonic()
        elif wall_age is None:
            # A caller-supplied age that cannot be evaluated must not be
            # silently replaced by "arrived now": the cloud arrival stays
            # unevaluable and later snapshots fail closed.
            arrival_wall = None
            age_reasons.append(REASON_CLOUD_AGE_INVALID)
        else:
            arrival_wall = time.monotonic() - float(wall_age)
        pose_arrival_wall = (
            arrival_wall
            if pose_arrival_wall_s is None
            else _optional_finite_float(pose_arrival_wall_s, "pose_arrival_wall_s")
        )
        explicit_transform = None
        explicit_stamp = None
        if pose is not None:
            explicit_transform = _as_transform(pose, "pose")
            if pose_stamp_s is None:
                raise ValueError("pose_stamp_s is required when pose is provided")
            explicit_stamp = _finite_float(pose_stamp_s, "pose_stamp_s")
            if pose_pairer is None:
                self._pairer.add(explicit_stamp, explicit_transform)
        pairer = pose_pairer if pose_pairer is not None else self._pairer
        pairing = pairer.pair(cloud_stamp)
        if pairing.ok:
            pose_used = pairing.pose
            pose_stamp_used = float(pairing.used_pose_stamp_s)
        elif explicit_transform is not None:
            pose_used = explicit_transform
            pose_stamp_used = explicit_stamp
        else:
            return self._refusal(
                ["pose_unavailable"],
                pose_stamp_s=explicit_stamp,
                cloud_stamp_s=cloud_stamp,
                pairing=pairing,
                ages=ages,
                pose_source=pose_source,
                cloud_source=cloud_source,
            )
        if self.extrinsic is None:
            return self._refusal(
                ["extrinsic_missing"],
                pose=pose_used,
                pose_stamp_s=pose_stamp_used,
                cloud_stamp_s=cloud_stamp,
                pairing=pairing,
                ages=ages,
                pose_source=pose_source,
                cloud_source=cloud_source,
            )
        normalized = points_to_map(
            points,
            declared_frame,
            pose=pose_used,
            extrinsic=self.extrinsic,
            max_range_m=self.max_range_m,
            max_sensor_range_m=self.max_sensor_range_m,
        )
        mapped = normalized.points_map
        point_count = int(mapped.shape[0])
        pose_z = float(pose_used[2, 3])
        z_values = mapped[:, 2]
        ground_search = (z_values > pose_z - FLOOR_SEARCH_BELOW_M) & (
            z_values < pose_z
        )
        unbounded_ground_points = mapped[ground_search]
        if self._measured_floor_standoff_m is not None:
            expected_floor = pose_z - self._measured_floor_standoff_m
            ground_search &= (
                np.abs(z_values - expected_floor) <= MIN_OBSTACLE_CLEARANCE_M
            )
        ground_points = mapped[ground_search]
        independent_floor = self._assess_floor_evidence(
            None, ground_points, pose_z, stand_proof_z_m=stand_proof_z_m
        )
        compatible_levels = None
        if independent_floor["ok"]:
            # An independently evidenced different support surface starts its
            # own chart; it cannot shift an old chart's classification bands.
            # In particular, a low-floor anchor established while climbing
            # must not permanently claim the later flat landing's pose band.
            compatible_levels = {
                level.index
                for level in self.level_tracker.levels
                if level.index not in self._level_floor_z
                or self._assess_floor_evidence(
                    level.index, ground_points, pose_z,
                    stand_proof_z_m=stand_proof_z_m,
                )["ok"]
            }
        level_match = self.level_tracker.observe(
            pose_z, compatible_levels=compatible_levels
        )
        if level_match.level_index is None or level_match.ambiguous:
            reasons = list(level_match.reasons) or ["level_unresolved"]
            return self._refusal(
                reasons,
                pose=pose_used,
                pose_stamp_s=pose_stamp_used,
                cloud_stamp_s=cloud_stamp,
                pairing=pairing,
                level_match=level_match,
                ages=ages,
                point_count=point_count,
                pose_source=pose_source,
                cloud_source=cloud_source,
            )
        level_index = int(level_match.level_index)
        level_z = float(level_match.level_z_m)
        self._level_z[level_index] = level_z
        if mapped.shape[0] == 0:
            return self._refusal(
                ["no_points"],
                pose=pose_used,
                pose_stamp_s=pose_stamp_used,
                cloud_stamp_s=cloud_stamp,
                pairing=pairing,
                level_match=level_match,
                ages=ages,
                pose_source=pose_source,
                cloud_source=cloud_source,
                level_index=level_index,
            )
        search = (z_values >= level_z - FLOOR_SEARCH_BELOW_M) & (
            z_values <= level_z + FLOOR_SEARCH_ABOVE_M
        )
        level_points = mapped[search]
        if level_points.shape[0] == 0:
            return self._refusal(
                ["floor_estimate_gap"],
                pose=pose_used,
                pose_stamp_s=pose_stamp_used,
                cloud_stamp_s=cloud_stamp,
                pairing=pairing,
                level_match=level_match,
                ages=ages,
                point_count=point_count,
                pose_source=pose_source,
                cloud_source=cloud_source,
                level_index=level_index,
            )
        # Credible ground evidence: only points strictly below this frame's
        # measured base origin, inside the existing floor search depth, may
        # support the floor. A surface at or above the base origin (desk,
        # shelf, mezzanine, the floor of the level above) is never this
        # level's ground, and a sparse low-percentile tail of a wall is not
        # ground either. Without evidence the frame is refused below; no cell
        # is classified and the level stays non-actionable.
        base_z = pose_z
        floor_evidence = self._assess_floor_evidence(
            level_index, ground_points, base_z, stand_proof_z_m=stand_proof_z_m
        )
        if ground_points.shape[0] == 0 and unbounded_ground_points.shape[0] > 0:
            # Preserve the explicit old-anchor mismatch diagnostic, but never
            # let a far lower surface bypass the measured-standoff search.
            floor_evidence = self._assess_floor_evidence(
                level_index, unbounded_ground_points, base_z,
                stand_proof_z_m=stand_proof_z_m,
            )
            floor_evidence["ok"] = False
            floor_evidence["reason"] = (
                floor_evidence["reason"] or REASON_FLOOR_EVIDENCE_UNCONFIRMED
            )
            floor_evidence["floor_z_m"] = None
            floor_evidence["evidence_route"] = None
        self._level_floor_evidence[level_index] = floor_evidence
        if not floor_evidence["ok"]:
            return self._refusal(
                [floor_evidence["reason"]],
                pose=pose_used,
                pose_stamp_s=pose_stamp_used,
                cloud_stamp_s=cloud_stamp,
                pairing=pairing,
                level_match=level_match,
                ages=ages,
                point_count=point_count,
                level_point_count=int(level_points.shape[0]),
                pose_source=pose_source,
                cloud_source=cloud_source,
                floor_evidence=floor_evidence,
                level_index=level_index,
            )
        established_floor = self._level_floor_z.get(level_index)
        floor_z = (
            float(established_floor)
            if established_floor is not None
            else float(floor_evidence["floor_z_m"])
        )
        self._level_floor_z.setdefault(level_index, floor_z)
        if self._measured_floor_standoff_m is None:
            self._measured_floor_standoff_m = base_z - floor_z
        height_above_floor = z_values - floor_z
        occupied_mask = (height_above_floor >= MIN_OBSTACLE_CLEARANCE_M) & (
            height_above_floor <= self.max_height_m
        )
        free_mask = (height_above_floor >= -FREE_FLOOR_BELOW_M) & (
            height_above_floor < MIN_OBSTACLE_CLEARANCE_M
        )
        grid = self._grids.get(level_index)
        if grid is None:
            grid = LevelGrid(self.resolution_m)
            self._grids[level_index] = grid
        occupied_added = grid.mark_occupied_points(mapped[occupied_mask][:, :2])
        free_added = 0
        if mode in ("floor", "both"):
            free_added = grid.mark_free_points(mapped[free_mask][:, :2])
        if mode in ("rays", "both"):
            targets = np.unique(
                np.floor(mapped[occupied_mask][:, :2] / self.resolution_m).astype(
                    np.int64
                ),
                axis=0,
            )
            if targets.shape[0]:
                # The cloud was measured at the sensor origin of this frame,
                # not at the base origin: compose the measured base pose with
                # the declared sensor->base extrinsic (T_map_base @
                # T_base_sensor). A point between the base origin and the
                # sensor origin must not be carved free.
                sensor_in_map = pose_used @ self.extrinsic.matrix
                grid.mark_free_rays(
                    (float(sensor_in_map[0, 3]), float(sensor_in_map[1, 3])),
                    [(int(ix), int(iy)) for ix, iy in targets],
                )
        samples = self._z_samples.get(level_index)
        if samples is None:
            samples = deque(maxlen=self._z_sample_limit)
            self._z_samples[level_index] = samples
        stride = max(1, int(ground_points.shape[0]) // 512)
        samples.extend(ground_points[::stride, 2].tolist())
        if self._last_pose is not None:
            step = float(np.linalg.norm(pose_used[:3, 3] - self._last_pose[:3, 3]))
            if math.isfinite(step):
                self._path_length_m += step
        self._last_pose = np.array(pose_used, dtype=float, copy=True)
        self._accepted_frames += 1
        self._accepted_points += int(level_points.shape[0])
        self._level_point_counts[level_index] = int(level_points.shape[0])
        # Freshness references for later snapshots: the accepted frame's cloud
        # stamp/arrival and the latest pose stream stamp/arrival. An older pose
        # used by a pairing never moves the pose stream reference backwards.
        self._last_valid_cloud_sim_s = cloud_stamp
        self._last_valid_cloud_wall_s = arrival_wall
        # Per-level observation: this level's own cloud stamp, arrival and
        # ingestion sim age. Another level's newer frame must never refresh
        # this record.
        self._level_observations[level_index] = {
            "cloud_stamp_s": float(cloud_stamp),
            "cloud_arrival_wall_s": (
                None if arrival_wall is None else float(arrival_wall)
            ),
            "cloud_sim_age_s": None if sim_age is None else float(sim_age),
            "pose_stamp_s": float(pose_stamp_used),
            "pose_arrival_wall_s": (
                None if pose_arrival_wall is None else float(pose_arrival_wall)
            ),
            "pose_source": pose_source,
            "cloud_source": cloud_source,
        }
        pose_wall_reference = pose_arrival_wall
        if explicit_transform is None:
            # The pose came from the registered pose stream (add_pose), which
            # recorded the authoritative arrival of that sample. A cloud
            # arriving later must not make the pose stream look younger, and a
            # bogus stream arrival must not be repaired by a fresh cloud.
            if self._pose_stream_wall_invalid:
                pose_wall_reference = None
            elif self._pose_stream_wall_s is not None:
                pose_wall_reference = self._pose_stream_wall_s
        if (
            self._pose_stream_sim_s is None
            or pose_stamp_used >= self._pose_stream_sim_s
        ):
            self._pose_stream_sim_s = float(pose_stamp_used)
            self._pose_stream_wall_s = pose_wall_reference
            self._pose_stream_wall_invalid = pose_wall_reference is None
        gates, gate_reasons = self._gate_state(
            ages=ages,
            level_match=level_match,
            pairing=pairing,
            extrinsic_ok=self.extrinsic is not None,
            point_count=int(level_points.shape[0]),
            level_available=True,
            minimum_points=minimum,
        )
        for reason in age_reasons:
            if reason not in gate_reasons:
                gate_reasons.append(reason)
        self._record_level_frame(
            level_index,
            accepted=True,
            reasons=gate_reasons,
            cloud_stamp_s=cloud_stamp,
            arrival_wall_s=arrival_wall,
            pose_stamp_s=pose_stamp_used,
            pose_source=pose_source,
            cloud_source=cloud_source,
        )
        self._update_current(
            pose=pose_used,
            pose_stamp_s=pose_stamp_used,
            cloud_stamp_s=cloud_stamp,
            pairing=pairing,
            level_match=level_match,
            ages=ages,
            point_count=point_count,
            level_point_count=int(level_points.shape[0]),
            pose_source=pose_source,
            cloud_source=cloud_source,
            reasons=gate_reasons,
        )
        receipt = {
            "accepted": True,
            "reasons": list(gate_reasons),
            "level_index": level_index,
            "level_z_m": _round(level_z),
            "floor_offset_m": _round(floor_z - level_z),
            "stamps": dict(self._last_stamps),
            "point_count": point_count,
            "level_point_count": int(level_points.shape[0]),
            "occupied_cells_added": int(occupied_added),
            "free_cells_added": int(free_added),
            "gates": dict(gates),
            "floor_evidence": self._floor_evidence_receipt(floor_evidence),
            # Read-only diagnostics for offline recomputation of what this
            # frame was paired with and which floor anchor classified it.
            "pose_stamp_s": _round(pose_stamp_used),
            "paired_dt_s": _round(None if pairing is None else pairing.dt_s),
            "pairing_interpolated": None
            if pairing is None
            else bool(pairing.interpolated),
            "pairing_bracket": None
            if pairing is None or pairing.bracket_stamps is None
            else [_round(pairing.bracket_stamps[0]), _round(pairing.bracket_stamps[1])],
            "classification_floor_z_m": _round(floor_z),
            "level_floor_anchor_m": _round(self._level_floor_z.get(level_index)),
            "note": note or "frame accumulated",
        }
        self._frames.append(
            {
                "cloud_stamp_s": _round(cloud_stamp),
                "pose_stamp_s": _round(pose_stamp_used),
                "paired_dt_s": _round(pairing.dt_s),
                "pairing_ok": gates["pairing"],
                "level_index": level_index,
                "level_z_m": _round(level_z),
                "point_count": point_count,
                "level_point_count": int(level_points.shape[0]),
            }
        )
        if len(self._frames) > 5000:
            del self._frames[:1000]
        return receipt

    # -- merge and snapshot ---------------------------------------------

    def merge(self, other):
        """Merge another accumulator's per-level grids into this one.

        Levels are matched by measured z within the level band. An unmatched
        other level is registered with this accumulator's LevelTracker (from
        its measured z) before its grid is merged, so a later frame on that
        floor resolves to the same level instead of creating a colliding
        index. A level that the tracker cannot resolve unambiguously is
        refused. The extrinsic of both accumulators must have the same
        calibration digest when both are present, because grids in different
        base frames cannot be mixed.
        """
        if not isinstance(other, OccupancyAccumulator):
            raise TypeError("merge expects an OccupancyAccumulator")
        if abs(other.resolution_m - self.resolution_m) > 1e-12:
            raise ValueError("cannot merge accumulators with different resolutions")
        # Fail closed before any write: two grids may only be merged when both
        # sides classify against the same floor anchor on the target level. A
        # different anchor means a different free/occupied band, and mixing the
        # grids would silently reclassify real obstacles as free space. The
        # tolerance below is float noise only, not the floor consistency window.
        merge_anchor_tolerance_m = 1e-9
        for other_level, other_z in other.levels:
            target = None
            for own_level, own_z in self.levels:
                if abs(own_z - other_z) <= self.level_tracker.band_width_m:
                    target = own_level
                    break
            if target is None:
                continue
            own_anchor = self._level_floor_z.get(target)
            other_anchor = other._level_floor_z.get(other_level)
            if own_anchor is not None and other_anchor is not None:
                if abs(float(own_anchor) - float(other_anchor)) > merge_anchor_tolerance_m:
                    raise ValueError(
                        "cannot merge level {}: classification floor anchors differ "
                        "({} vs {})".format(target, own_anchor, other_anchor)
                    )
            if other_level in other._grids and other_anchor is None:
                raise ValueError(
                    "cannot merge level {}: the other accumulator has cells but no "
                    "established floor anchor".format(target)
                )
            if target in self._grids and own_anchor is None:
                raise ValueError(
                    "cannot merge level {}: this accumulator has cells but no "
                    "established floor anchor".format(target)
                )
        if (
            self.extrinsic is not None
            and other.extrinsic is not None
            and self.extrinsic.sha256 != other.extrinsic.sha256
        ):
            raise ValueError(
                "cannot merge accumulators with different sensor->base extrinsics"
            )
        for other_level, other_z in other.levels:
            target = None
            for own_level, own_z in self.levels:
                if abs(own_z - other_z) <= self.level_tracker.band_width_m:
                    target = own_level
                    break
            if target is None:
                match = self.level_tracker.observe(float(other_z))
                if match.level_index is None or match.ambiguous:
                    raise ValueError(
                        "cannot merge level z={:.3f}: the level tracker cannot "
                        "resolve it unambiguously".format(float(other_z))
                    )
                target = int(match.level_index)
                self._level_z[target] = float(match.level_z_m)
            if other_level in other._grids:
                grid = self._grids.get(target)
                if grid is None:
                    grid = LevelGrid(self.resolution_m)
                    self._grids[target] = grid
                grid.merge(other._grids[other_level])
            if other_level in other._z_samples:
                samples = self._z_samples.get(target)
                if samples is None:
                    samples = deque(maxlen=self._z_sample_limit)
                    self._z_samples[target] = samples
                samples.extend(list(other._z_samples[other_level]))
            # Floor evidence travels with the level it was measured on; a level
            # that already has an established floor keeps it.
            if target not in self._level_floor_z:
                if other_level in other._level_floor_z:
                    self._level_floor_z[target] = float(
                        other._level_floor_z[other_level]
                    )
            if target not in self._level_floor_evidence:
                if other_level in other._level_floor_evidence:
                    self._level_floor_evidence[target] = dict(
                        other._level_floor_evidence[other_level]
                    )
            # The level's own freshness/observation record and latest frame
            # outcome travel with it, so a merged level keeps its real age
            # instead of inheriting this accumulator's newest frame.
            if (
                target not in self._level_observations
                and other_level in other._level_observations
            ):
                self._level_observations[target] = dict(
                    other._level_observations[other_level]
                )
            if (
                target not in self._level_frame_status
                and other_level in other._level_frame_status
            ):
                self._level_frame_status[target] = dict(
                    other._level_frame_status[other_level]
                )
            self._level_point_counts[target] = self._level_point_counts.get(
                target, 0
            ) + other._level_point_counts.get(other_level, 0)
        self._accepted_frames += other._accepted_frames
        self._accepted_points += other._accepted_points
        self._frames.extend(other._frames)
        self._path_length_m += other.measured_path_length_m
        # Freshness references: an accumulator with no frames of its own
        # adopts the other's stream references instead of reporting unknown.
        if self._last_valid_cloud_sim_s is None:
            self._last_valid_cloud_sim_s = other._last_valid_cloud_sim_s
            self._last_valid_cloud_wall_s = other._last_valid_cloud_wall_s
        if self._pose_stream_sim_s is None:
            self._pose_stream_sim_s = other._pose_stream_sim_s
            self._pose_stream_wall_s = other._pose_stream_wall_s
            self._pose_stream_wall_invalid = other._pose_stream_wall_invalid
        return self

    def level_summary(self):
        """Return per-level occupancy summaries."""
        summary = []
        for level_index, level_z in self.levels:
            grid = self._grids.get(level_index)
            if grid is None:
                occupied = free = 0
            else:
                occupied, free = grid.counts()
            summary.append(
                {
                    "level_index": level_index,
                    "level_z_m": _round(level_z),
                    "occupied_cells": int(occupied),
                    "free_cells": int(free),
                }
            )
        return summary

    @staticmethod
    def _snapshot_freshness(freshness, gates, reasons, cloud_ages):
        """Combine the resolved ages with the gate verdict into JSON facts."""
        facts = dict(freshness) if freshness is not None else {}
        stale_reasons = [
            reason
            for reason in reasons
            if reason
            in (
                "pose_age_unknown",
                "stale_pose_sim_age",
                "stale_pose_wall_age",
                REASON_STALE_CLOUD_SIM_AGE,
                REASON_STALE_CLOUD_WALL_AGE,
                REASON_CLOUD_AGE_UNKNOWN,
                REASON_CLOUD_AGE_UNEVALUATED,
                REASON_CLOUD_AGE_INVALID,
                REASON_POSE_AGE_INVALID,
                REASON_CLOCK_INVALID,
                REASON_SIM_CLOCK_REGRESSION,
                REASON_WALL_CLOCK_REGRESSION,
            )
        ]
        cloud_evaluated = cloud_ages is not None and (
            cloud_ages[0] is not None or cloud_ages[1] is not None
        )
        cloud_complete = (
            cloud_ages is not None
            and cloud_ages[0] is not None
            and cloud_ages[1] is not None
            and math.isfinite(float(cloud_ages[0]))
            and math.isfinite(float(cloud_ages[1]))
        )
        # cloud_age_evaluated says the cloud stream was actually re-evaluated
        # (and at least one of its ages is available); cloud_age_complete says
        # both its source and arrival ages are available. Every snapshot now
        # evaluates the cloud stream, including one with no caller now.
        facts["cloud_age_evaluated"] = bool(cloud_evaluated)
        facts["cloud_age_complete"] = bool(cloud_complete)
        facts["pose_fresh"] = bool(gates.get("pose_fresh", False))
        facts["stale"] = not facts["pose_fresh"]
        facts["stale_reasons"] = stale_reasons
        return facts

    def mark_untrusted(self, reason, *, source=None):
        """Refuse every later snapshot of this accumulator generation.

        A cloud or odometry frame change means the accumulated geometry may mix
        coordinate frames, so the generation is no longer a navigation input. The
        geometry and the refusal evidence are kept; there is deliberately no
        auto-clear, because a later good frame must not mask the mixed history. A
        fresh generation is a new accumulator with its own provenance.
        """
        if self._untrusted_reason is None:
            self._untrusted_reason = str(reason)
            self._untrusted_source = None if source is None else str(source)
        return self._untrusted_reason


    def _empty_snapshot(
        self,
        *,
        pose,
        ages,
        note,
        cloud_ages=None,
        regressions=(),
        freshness=None,
        target_level=None,
    ):
        level_match = self._last_level_match
        level_available = bool(
            level_match is not None
            and level_match.level_index is not None
            and not level_match.ambiguous
        )
        gates, reasons = self._gate_state(
            ages=ages,
            level_match=level_match,
            pairing=self._last_pairing,
            extrinsic_ok=self.extrinsic is not None,
            point_count=0,
            level_available=level_available,
            cloud_ages=cloud_ages,
        )
        for reason in regressions:
            if reason not in reasons:
                reasons.append(reason)
        if regressions:
            gates["pose_fresh"] = False
        if "no_map" not in reasons:
            reasons.insert(0, "no_map")
        for reason in self._last_reasons:
            if reason not in reasons:
                reasons.append(reason)
        evidence = None
        if target_level is not None:
            evidence = self._level_floor_evidence.get(target_level)
        elif level_match is not None and level_match.level_index is not None:
            evidence = self._level_floor_evidence.get(level_match.level_index)
        floor_evidence = self._floor_evidence_receipt(evidence)
        if not floor_evidence["ok"] and floor_evidence["reason"] not in reasons:
            reasons.append(floor_evidence["reason"])
        status_level = target_level
        if status_level is None and level_match is not None:
            status_level = level_match.level_index
        status = (
            None
            if status_level is None
            else self._level_frame_status.get(status_level)
        )
        latest_frame_accepted = None if status is None else bool(status["accepted"])
        latest_frame_reasons = [] if status is None else list(status["reasons"])
        if latest_frame_accepted is False:
            for reason in latest_frame_reasons:
                if reason not in reasons:
                    reasons.append(reason)
            if REASON_LATEST_FRAME_REFUSED not in reasons:
                reasons.append(REASON_LATEST_FRAME_REFUSED)
        freshness_facts = self._snapshot_freshness(
            freshness, gates, reasons, cloud_ages
        )
        freshness_facts["latest_frame_accepted"] = latest_frame_accepted
        freshness_facts["latest_frame_reasons"] = latest_frame_reasons
        extrinsic = self.extrinsic
        return {
            "schema_version": SCHEMA_VERSION,
            "frames": {
                "sensor": extrinsic.sensor_frame if extrinsic else DEFAULT_SENSOR_FRAME,
                "base": extrinsic.base_frame if extrinsic else DEFAULT_BASE_FRAME,
                "map": self.map_frame,
            },
            "sources": {
                "pose_source": self._last_pose_source,
                "cloud_source": self._last_cloud_source,
            },
            "stamps": dict(self._last_stamps),
            "level_z_m": None,
            "floor_offset_m": None,
            "floor_z_m": None,
            "floor_z_source": None,
            "observed_ground_quantile_m": None,
            "resolution_m": self.resolution_m,
            "occupied_cells": 0,
            "free_cells": 0,
            "unknown_estimate_area_m2": 0.0,
            "coverage_area_m2": 0.0,
            "frontier": {
                "cells": [],
                "cell_indices": [],
                "count": 0,
                "returned_cells": 0,
                "min_distance_m": None,
                "actionable": False,
                "reasons": reasons,
            },
            "measured_path_length_m": _round(self._path_length_m),
            "path_length_is_not_coverage": True,
            "snapshot_is_accumulated_map": bool(self._accepted_frames > 1),
            "ground_truth_used": False,
            "extrinsic_sha256": extrinsic.sha256 if extrinsic else None,
            "note": note or DEFAULT_NOTE,
            "point_count": 0,
            "level_point_count": 0,
            "accumulated_points": int(self._accepted_points),
            "accumulated_frames": int(self._accepted_frames),
            "level_index": None,
            "gates": gates,
            "floor_evidence": floor_evidence,
            "floor_evidence_ok": bool(floor_evidence["ok"]),
            "latest_frame_accepted": latest_frame_accepted,
            "latest_frame_reasons": latest_frame_reasons,
            "level_stale": True,
            "snapshot_freshness": freshness_facts,
            "pose_ages": {
                "sim_s": _round(ages[0]),
                "wall_s": _round(ages[1]),
            },
            "declared_cloud_frame": self._last_declared_cloud_frame,
            "pairing": (
                None if self._last_pairing is None else self._last_pairing.as_dict()
            ),
            "unknown_estimate_definition": UNKNOWN_ESTIMATE_DEFINITION,
            "levels": self.level_summary(),
            "snapshot_kind": "empty_refusal",
        }

    def snapshot(
        self,
        level=None,
        *,
        pose=None,
        sim_age_s=None,
        wall_age_s=None,
        frontier_limit=None,
        note=None,
        now_sim_s=None,
        now_wall_s=None,
    ):
        """Return the JSON snapshot of one level (or a no-map refusal).

        pose defaults to the most recent measured frame.

        Freshness is re-evaluated on every call. now_sim_s is the current
        source sim time and now_wall_s the current arrival monotonic time; they
        are separate clocks and each age is now minus the reference stamp of
        that clock, so no previous now and no previous age is reused. A caller
        that passes a later now therefore turns the snapshot stale even if it
        repeats nothing else. When no wall now is passed the wall age is still
        re-evaluated against the arrival monotonic time recorded at ingestion;
        when no sim now is passed the ingested sim age is reported because the
        process has no authoritative source clock. sim_age_s / wall_age_s are
        the legacy explicit ingestion ages and are used only when the
        corresponding now is not passed.

        Clock authority (also for the no-argument path): arrivals are recorded
        on time.monotonic(); a caller now_wall_s must be that same clock and
        now_sim_s is the source clock of the ingested stamps. Both streams are
        re-evaluated on every call. With no caller now the recorded cloud
        arrival is compared with time.monotonic() and the target level's own
        recorded ingestion sim age is reported. If a stream age cannot be
        evaluated (missing arrival, non-finite now) the snapshot is not
        actionable with an explicit reason.

        A level whose last frame had no credible ground evidence, whose latest
        frame was refused, or whose input streams are stale, is returned with
        actionable=False and the explicit reason in frontier.reasons; the
        frontier cells already observed are still listed so the operator can
        see the map and the refusal.
        """
        target_level = level
        if target_level is None and self._last_level_match is not None:
            target_level = self._last_level_match.level_index
        if target_level is None:
            for index in sorted(self._level_z, key=self._level_z.get):
                target_level = index
        ages, cloud_ages, regressions, freshness = self._resolve_snapshot_ages(
            sim_age_s=sim_age_s,
            wall_age_s=wall_age_s,
            now_sim_s=now_sim_s,
            now_wall_s=now_wall_s,
            target_level=target_level,
        )
        snapshot_pose = None
        if pose is not None:
            snapshot_pose = _as_transform(pose, "pose")
        elif self._last_pose is not None:
            snapshot_pose = np.array(self._last_pose, dtype=float, copy=True)
        grid = self._grids.get(target_level) if target_level is not None else None
        level_z = self._level_z.get(target_level) if target_level is not None else None
        if grid is None or level_z is None:
            return self._empty_snapshot(
                pose=snapshot_pose,
                ages=ages,
                note=note,
                cloud_ages=cloud_ages,
                regressions=regressions,
                freshness=freshness,
                target_level=target_level,
            )
        gates, reasons = self._gate_state(
            ages=ages,
            level_match=self._last_level_match,
            pairing=self._last_pairing,
            extrinsic_ok=self.extrinsic is not None,
            point_count=self._level_point_counts.get(target_level, 0),
            level_available=True,
            cloud_ages=cloud_ages,
        )
        for reason in regressions:
            if reason not in reasons:
                reasons.append(reason)
        if regressions:
            gates["pose_fresh"] = False
        # Only the target level's own evidence is relevant here; the reasons of
        # a frame that resolved to another level must not leak into this level's
        # snapshot (the empty snapshot still reports the last refusal reasons).
        floor_evidence = self._floor_evidence_receipt(
            self._level_floor_evidence.get(target_level)
        )
        floor_ok = bool(floor_evidence["ok"])
        if not floor_ok and floor_evidence["reason"] not in reasons:
            reasons.append(floor_evidence["reason"])
        # Map geometry and *current* actionability are separate: a refused
        # latest frame for this level keeps the old cells but must not leave
        # the level actionable on the strength of an earlier accepted frame.
        status = self._level_frame_status.get(target_level)
        latest_frame_accepted = None if status is None else bool(status["accepted"])
        latest_frame_reasons = [] if status is None else list(status["reasons"])
        if latest_frame_accepted is False:
            for reason in latest_frame_reasons:
                if reason not in reasons:
                    reasons.append(reason)
            if REASON_LATEST_FRAME_REFUSED not in reasons:
                reasons.append(REASON_LATEST_FRAME_REFUSED)
        freshness_facts = self._snapshot_freshness(
            freshness, gates, reasons, cloud_ages
        )
        freshness_facts["latest_frame_accepted"] = latest_frame_accepted
        freshness_facts["latest_frame_reasons"] = latest_frame_reasons
        if self._untrusted_reason is not None:
            reasons.append(REASON_UNTRUSTED_GENERATION)
        actionable = (
            all(gates.values())
            and floor_ok
            and latest_frame_accepted is not False
            and self._untrusted_reason is None
        )
        pose_xy = (
            (float(snapshot_pose[0, 3]), float(snapshot_pose[1, 3]))
            if snapshot_pose is not None
            else (0.0, 0.0)
        )
        frontier_cells = grid.frontier(pose_xy)
        limit = None if frontier_limit is None else max(0, int(frontier_limit))
        returned = frontier_cells if limit is None else frontier_cells[:limit]
        resolution = self.resolution_m
        distances = [
            math.hypot(
                (ix + 0.5) * resolution - pose_xy[0],
                (iy + 0.5) * resolution - pose_xy[1],
            )
            for ix, iy in returned
        ]
        occupied, free = grid.counts()
        sample = self._z_samples.get(target_level)
        observed_quantile = (
            float(np.quantile(np.asarray(sample, dtype=float), self.floor_quantile))
            if sample
            else None
        )
        established_floor = self._level_floor_z.get(target_level)
        if established_floor is not None:
            floor_z = float(established_floor)
            floor_z_source = "established_level_anchor"
        elif observed_quantile is not None:
            floor_z = observed_quantile
            floor_z_source = "observed_ground_quantile_no_anchor"
        else:
            floor_z = None
            floor_z_source = None
        floor_offset = None if floor_z is None else floor_z - float(level_z)
        extrinsic = self.extrinsic
        return {
            "schema_version": SCHEMA_VERSION,
            "frames": {
                "sensor": extrinsic.sensor_frame if extrinsic else DEFAULT_SENSOR_FRAME,
                "base": extrinsic.base_frame if extrinsic else DEFAULT_BASE_FRAME,
                "map": self.map_frame,
            },
            "sources": {
                "pose_source": self._last_pose_source,
                "cloud_source": self._last_cloud_source,
            },
            "stamps": dict(self._last_stamps),
            "level_z_m": _round(level_z),
            "floor_offset_m": _round(floor_offset),
            "floor_z_m": _round(floor_z),
            "floor_z_source": floor_z_source,
            "observed_ground_quantile_m": _round(observed_quantile),
            "resolution_m": resolution,
            "occupied_cells": int(occupied),
            "free_cells": int(free),
            "unknown_estimate_area_m2": _round(grid.unknown_estimate_area_m2()),
            "coverage_area_m2": _round((occupied + free) * resolution ** 2),
            "frontier": {
                "cells": [
                    [
                        _round((ix + 0.5) * resolution),
                        _round((iy + 0.5) * resolution),
                    ]
                    for ix, iy in returned
                ],
                "cell_indices": [[int(ix), int(iy)] for ix, iy in returned],
                "count": len(frontier_cells),
                "returned_cells": len(returned),
                "min_distance_m": _round(distances[0]) if distances else None,
                "actionable": bool(actionable),
                "reasons": list(reasons),
            },
            "measured_path_length_m": _round(self._path_length_m),
            "path_length_is_not_coverage": True,
            "snapshot_is_accumulated_map": bool(self._accepted_frames > 1),
            "ground_truth_used": False,
            "extrinsic_sha256": extrinsic.sha256 if extrinsic else None,
            "note": note or DEFAULT_NOTE,
            "declared_cloud_frame": self._last_declared_cloud_frame,
            "point_count": int(self._last_point_count),
            "level_point_count": int(
                self._level_point_counts.get(target_level, 0)
            ),
            "accumulated_points": int(self._accepted_points),
            "accumulated_frames": int(self._accepted_frames),
            "level_index": int(target_level),
            "gates": gates,
            "floor_evidence": floor_evidence,
            "floor_evidence_ok": floor_ok,
            "latest_frame_accepted": latest_frame_accepted,
            "latest_frame_reasons": latest_frame_reasons,
            "level_stale": bool(not actionable),
            "snapshot_freshness": freshness_facts,
            "pose_ages": {"sim_s": _round(ages[0]), "wall_s": _round(ages[1])},
            "pairing": (
                None if self._last_pairing is None else self._last_pairing.as_dict()
            ),
            "unknown_estimate_definition": UNKNOWN_ESTIMATE_DEFINITION,
            "levels": self.level_summary(),
            "snapshot_kind": "accumulated" if self._accepted_frames > 1 else "single_frame",
        }

    def to_json(self, level=None, **kwargs):
        """Return the JSON string of one snapshot (convenience for receipts)."""
        return json.dumps(self.snapshot(level, **kwargs), sort_keys=True)


def _as_extrinsic(value):
    """Coerce an Extrinsic / 4x4 / dict into an Extrinsic, refusing absence."""
    if value is None:
        raise ValueError(
            "extrinsic_missing: an explicit sensor->base extrinsic is required"
        )
    if isinstance(value, Extrinsic):
        return value
    return Extrinsic(value)


def build_map_snapshot(
    points,
    *,
    frame,
    extrinsic,
    cloud_stamp_s,
    pose=None,
    pose_stamp_s=None,
    sim_age_s=None,
    wall_age_s=None,
    resolution_m=DEFAULT_RESOLUTION_M,
    level_tracker=None,
    min_points=DEFAULT_MIN_POINTS,
    max_range_m=DEFAULT_MAX_RANGE_M,
    max_sensor_range_m=DEFAULT_MAX_SENSOR_RANGE_M,
    floor_quantile=DEFAULT_FLOOR_QUANTILE,
    max_height_m=DEFAULT_MAX_HEIGHT_M,
    free_mode=DEFAULT_FREE_MODE,
    pose_source="lio_estimate",
    cloud_source="depth",
    pose_pairer=None,
    stand_proof_z_m=None,
    map_frame=DEFAULT_MAP_FRAME,
    frontier_limit=None,
    snapshot_is_accumulated_map=None,
    note=None,
):
    """Build one operator-side map snapshot from one measured point cloud.

    This is the one-shot path for the operator: validate the explicit
    sensor->base extrinsic, pair the cloud stamp with a measured pose, resolve
    the measured level, estimate that level's floor, accumulate occupied/free
    cells and return the frontier with its actionable gates. Missing or
    invalid extrinsic raises ValueError before any geometry is used; a failed
    freshness/pairing/level/point gate returns the snapshot with
    frontier.actionable=False and the gate reasons.

    snapshot_is_accumulated_map defaults to False because this call only sees
    one frame; a caller that accumulates itself may pass True explicitly.
    """
    sensor_to_base = _as_extrinsic(extrinsic)
    accumulator = OccupancyAccumulator(
        extrinsic=sensor_to_base,
        resolution_m=resolution_m,
        level_tracker=level_tracker,
        min_points=min_points,
        max_range_m=max_range_m,
        max_sensor_range_m=max_sensor_range_m,
        floor_quantile=floor_quantile,
        max_height_m=max_height_m,
        free_mode=free_mode,
        map_frame=map_frame,
    )
    accumulator.add_frame(
        points,
        frame=frame,
        cloud_stamp_s=cloud_stamp_s,
        pose=pose,
        pose_stamp_s=pose_stamp_s,
        sim_age_s=sim_age_s,
        wall_age_s=wall_age_s,
        pose_source=pose_source,
        cloud_source=cloud_source,
        pose_pairer=pose_pairer,
        stand_proof_z_m=stand_proof_z_m,
        note=note,
    )
    snapshot = accumulator.snapshot(frontier_limit=frontier_limit, note=note)
    snapshot["declared_cloud_frame"] = _frame_name(frame)
    if snapshot_is_accumulated_map is not None:
        snapshot["snapshot_is_accumulated_map"] = bool(snapshot_is_accumulated_map)
    return snapshot
