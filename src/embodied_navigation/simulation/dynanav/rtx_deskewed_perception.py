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
from collections import OrderedDict

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

COLLISION_LOW_M = 0.10
COLLISION_HIGH_M = 1.20
FLOOR_BAND_M = 0.10
CORRIDOR_MIN_M = 0.35
CORRIDOR_LENGTH_M = 3.0
CORRIDOR_BINS = 4
CORRIDOR_HALF_DEG = 45.0
AZIMUTH_BIN_DEG = 5.0
MIN_RETURNS_PER_AZIMUTH_BIN = 5
REQUIRED_AZIMUTH_FRACTION = 0.85
MIN_RADIAL_POINTS_PER_BIN = 10
MIN_FLOOR_POINTS = 30
FLOOR_SEARCH_RADIUS_M = 4.0
FLOOR_MIN_BELOW_BASE_M = 0.12
FLOOR_MAX_BELOW_BASE_M = 1.20
FLOOR_BIN_M = 0.04
MIN_FLOOR_SEED_POINTS = 150
SENSOR_MAX_RANGE_M = 40.0
MIN_CURVATURE_SPREAD_MS = 1.0
DIRECTIONS = ("front", "left", "rear", "right")


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


def detect_floor_z(
    points_base,
    *,
    search_radius_m=FLOOR_SEARCH_RADIUS_M,
    min_below_base_m=FLOOR_MIN_BELOW_BASE_M,
    max_below_base_m=FLOOR_MAX_BELOW_BASE_M,
    bin_m=FLOOR_BIN_M,
    min_points=MIN_FLOOR_SEED_POINTS,
):
    """Measure the floor height below the base from the same cloud.

    Only a dense horizontal surface below the base origin can be the ground.
    A missing floor never becomes an assumed constant.
    """
    points = np.asarray(points_base, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise DeskewedCloudError("points must be an Nx3 array")
    if points.shape[0] == 0:
        return None, {"reason": "no_points", "seed_points": 0}
    ranges = np.linalg.norm(points[:, :2], axis=1)
    near = points[ranges <= float(search_radius_m)]
    z = near[:, 2]
    candidates = z[(z >= -float(max_below_base_m)) & (z <= -float(min_below_base_m))]
    if candidates.size < int(min_points):
        return None, {
            "reason": "insufficient_floor_candidates",
            "candidate_points": int(candidates.size),
            "required_points": int(min_points),
        }
    edges = np.arange(
        -float(max_below_base_m),
        -float(min_below_base_m) + float(bin_m) / 2.0,
        float(bin_m),
    )
    counts, edges = np.histogram(candidates, bins=edges)
    near_ranges = np.linalg.norm(near[:, :2], axis=1)
    dense = [
        index for index, count in enumerate(counts) if int(count) >= int(min_points)
    ]
    if not dense:
        return None, {
            "reason": "no_dense_floor_bin",
            "best_bin_points": int(counts.max()),
            "required_points": int(min_points),
        }
    # A ground plane is a dense *horizontal* surface: its points span a radial
    # interval, while a wall at a fixed height is a thin ring. Prefer the lowest
    # dense bin that spans radii, never the globally densest bin.
    spanning = []
    for index in dense:
        low, high = float(edges[index]), float(edges[index + 1])
        in_bin = (z >= low - float(bin_m) / 2.0) & (z <= high + float(bin_m) / 2.0)
        if int(np.count_nonzero(in_bin)) < int(min_points):
            continue
        spread = float(np.ptp(near_ranges[in_bin]))
        if spread >= 0.35:
            spanning.append((float(edges[index]), int(index), spread))
    if not spanning:
        # A dense ring of returns at one height is a wall, not a ground plane.
        return None, {
            "reason": "no_sprawling_ground_surface",
            "best_bin_points": int(counts.max()),
            "required_points": int(min_points),
        }
    # The supporting ground is the closest dense horizontal surface below the
    # base. The *lowest* surface is wrong at a stair edge or a window: the lower
    # ground outside is also dense and spanning, but the robot is not standing on
    # it, and using it would shift the whole collision band.
    _edge, best, spread = max(spanning, key=lambda item: item[0])
    low, high = float(edges[best]), float(edges[best + 1])
    in_bin = candidates[(candidates >= low) & (candidates <= high)]
    floor_z = float(np.median(in_bin))
    return floor_z, {
        "reason": None,
        "floor_z_m": floor_z,
        "seed_points": int(in_bin.size),
        "candidate_points": int(candidates.size),
        "radial_spread_m": None if spread is None else round(float(spread), 4),
        "search_radius_m": float(search_radius_m),
        "bin_m": float(bin_m),
        "source": "measured_dense_ground_surface_below_base",
    }


def floor_visible_min_range(
    points_base,
    floor_z_m=None,
    *,
    floor_plane=None,
    floor_band_m=FLOOR_BAND_M,
    min_points=MIN_FLOOR_POINTS,
):
    """Measure the nearest horizontal range at which the floor is observed.

    A rotating LiDAR mounted above the floor cannot see the ground inside its
    lowest-ray cone. The blind inner annulus is measured from the returns (not
    assumed) and reported as unknown floor, so it can never be presented as
    observed ground support.
    """
    points = np.asarray(points_base, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise DeskewedCloudError("points must be an Nx3 array")
    ranges = np.linalg.norm(points[:, :2], axis=1)
    if floor_plane is not None:
        reference = floor_height_at(floor_plane, points)
        floor_z_m = float(np.asarray(floor_plane, dtype=float).reshape(-1)[2])
    else:
        reference = np.full(len(points), _finite(floor_z_m, "floor_z_m"))
    floor = (np.abs(points[:, 2] - reference) <= float(floor_band_m)) & (ranges > 0.05)
    floor_ranges = ranges[floor]
    if floor_ranges.size < int(min_points):
        return None, {"floor_points": int(floor_ranges.size), "required": int(min_points)}
    observed_min = float(np.percentile(floor_ranges, 2))
    return observed_min, {
        "floor_points": int(floor_ranges.size),
        "nearest_floor_return_m": round(float(floor_ranges.min()), 4),
        "observed_min_range_m": round(observed_min, 4),
        "method": "2nd percentile of measured floor-band horizontal ranges",
    }


def fit_floor_plane(
    points_base,
    *,
    search_radius_m=FLOOR_SEARCH_RADIUS_M,
    min_below_m=0.08,
    max_below_m=1.40,
    sectors=24,
    radial_bands=((1.0, 1.8), (1.8, 2.6), (2.6, 3.4)),
    min_points_band=30,
    min_samples=8,
    max_slope=0.30,
    max_residual_m=0.10,
    lower_percentile=5.0,
):
    """Fit a broad, unambiguous measured plane using actual return coordinates.

    A sector's low height cannot be attached to its nominal radial/azimuth
    centre: on a tilted floor those are different points, and mixing stairs
    biases the result further. Actual low-envelope returns seed deterministic
    robust hypotheses. Only cells with a majority of consistent real returns
    contribute to refinement, with equal cell weight rather than point density.

    Multiple coherent surfaces spanning azimuth and radius are ambiguous even
    when one has more returns. A narrow tread/platform or a wall ring cannot
    establish the floor. This classifies observed points only; neither a fitted
    plane nor its extrapolation supplies missing near-field support.
    """
    points = np.asarray(points_base, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise DeskewedCloudError("points must be an Nx3 array")
    if not np.isfinite(points).all():
        raise DeskewedCloudError("non-finite floor points")
    if points.shape[0] == 0:
        return None, {"reason": "no_points", "samples": 0}
    ranges = np.linalg.norm(points[:, :2], axis=1)
    z = points[:, 2]
    azimuth = np.degrees(np.arctan2(points[:, 1], points[:, 0])) % 360.0
    candidates = (
        (ranges <= float(search_radius_m)) & (ranges > 0.5)
        & (z < -float(min_below_m)) & (z > -float(max_below_m))
    )
    cells, samples, sample_cells, labels = [], [], [], []
    for band, (low, high) in enumerate(radial_bands):
        radial = candidates & (ranges >= float(low)) & (ranges < float(high))
        band_points = points[radial]
        band_azimuth = azimuth[radial]
        for sector in range(int(sectors)):
            selected = (band_azimuth >= sector * 360.0 / sectors) & (
                band_azimuth < (sector + 1) * 360.0 / sectors
            )
            cloud = band_points[selected]
            if len(cloud) < int(min_points_band):
                continue
            # Lower, middle and upper observed heights also expose two real
            # levels sharing a cell. Looking only at the lowest quantile would
            # conceal an upper platform behind a denser lower surface.
            for target in np.percentile(cloud[:, 2], [float(lower_percentile), 50.0, 95.0]):
                samples.append(cloud[np.argmin(np.abs(cloud[:, 2] - target))])
                sample_cells.append(len(cells))
            cells.append(cloud)
            labels.append((band, sector))
    labels = np.asarray(labels, dtype=int).reshape(-1, 2)
    evidence = {
        "reason": None, "samples": len(cells),
        "radial_bands_used": sorted(set(labels[:, 0].tolist())),
        "source": "measured_actual_return_robust_ground_plane",
        "near_field_extrapolation_used": False,
    }
    if len(cells) < int(min_samples) or len(evidence["radial_bands_used"]) < 2:
        evidence["reason"] = "floor_not_observed_across_radii"
        return None, evidence
    sample = np.asarray(samples)
    sample_cells = np.asarray(sample_cells, dtype=int)
    design = np.column_stack([sample[:, :2], np.ones(len(sample))])
    # This is stricter than the original residual limit, never a relaxed gate.
    tolerance = min(0.03, float(max_residual_m))
    if tolerance <= 0:
        raise DeskewedCloudError("floor residual bound must be positive")
    rng = np.random.default_rng(0)
    hypotheses = []
    for _ in range(256):
        chosen = rng.choice(len(sample), 3, replace=False)
        if abs(float(np.linalg.det(design[chosen]))) < 1e-6:
            continue
        plane = np.linalg.solve(design[chosen], sample[chosen, 2])
        if np.linalg.norm(plane[:2]) > float(max_slope) or plane[2] > -float(min_below_m):
            continue
        residual = np.abs(design @ plane - sample[:, 2])
        inliers = residual <= tolerance
        inlier_cells = np.unique(sample_cells[inliers])
        if len(inlier_cells) < int(min_samples):
            continue
        if len(set(labels[inlier_cells, 0])) < 2:
            continue
        # Avoid repeatedly refining the same hypothesis. The comparison uses
        # all observed sample coordinates, not just the intercept at the robot.
        if any(np.max(np.abs(design @ (plane - other))) <= tolerance
               for other in hypotheses):
            continue
        hypotheses.append(plane)

    all_points = np.concatenate(cells)
    cell_sizes = np.asarray([len(cloud) for cloud in cells])
    cell_starts = np.r_[0, np.cumsum(cell_sizes)[:-1]]

    # Per-call, exact-input LRU. Eviction only recomputes the same validation;
    # entries never cross frames, and at most eight support arrays are retained.
    supporting_cache = OrderedDict()
    def supporting_cells(plane, *, independent_of=None):
        key = (plane.tobytes(), None if independent_of is None else independent_of.tobytes())
        if key in supporting_cache:
            supporting_cache.move_to_end(key)
            return supporting_cache[key]
        residual = np.abs(all_points[:, 2] - all_points[:, 0] * plane[0]
                          - all_points[:, 1] * plane[1] - plane[2])
        inlier_mask = residual <= tolerance
        if independent_of is not None:
            other_residual = np.abs(all_points[:, 2] - all_points[:, 0] * independent_of[0]
                                    - all_points[:, 1] * independent_of[1] - independent_of[2])
            inlier_mask &= other_residual > tolerance
        counts = np.add.reduceat(inlier_mask.astype(np.int64), cell_starts)
        indices = np.flatnonzero((counts >= int(min_points_band)) & (counts >= 0.5 * cell_sizes))
        if len(indices) < int(min_samples):
            return []
        # Reject thin intersections before the comparatively expensive refit.
        used = labels[indices]
        multi_band_sectors = sum(len(set(used[used[:, 1] == sector, 0])) >= 2
                                 for sector in set(used[:, 1]))
        if multi_band_sectors < 4:
            return []
        result = [(int(index), cells[index][inlier_mask[cell_starts[index]:
                 cell_starts[index] + cell_sizes[index]]]) for index in indices]
        if len(supporting_cache) >= 8:
            supporting_cache.popitem(last=False)
        supporting_cache[key] = result
        return result

    coherent = []
    for plane in hypotheses:
        supported = []
        previous_cloud = None
        previous_cells = None
        for _ in range(3):
            supported = supporting_cells(plane)
            if len(supported) < int(min_samples):
                break
            cloud = np.concatenate([part for _, part in supported])
            current_cells = tuple((index, len(part)) for index, part in supported)
            # Identical ordered points and cell weights repeat the identical
            # least-squares problem. Do not stop for merely similar supports.
            if current_cells == previous_cells and np.array_equal(cloud, previous_cloud):
                break
            previous_cloud = cloud
            previous_cells = current_cells
            weights = np.concatenate([np.full(len(part), 1.0 / math.sqrt(len(part)))
                                      for _, part in supported])
            matrix = np.column_stack([cloud[:, :2], np.ones(len(cloud))])
            plane, _, rank, _ = np.linalg.lstsq(
                matrix * weights[:, None], cloud[:, 2] * weights, rcond=None
            )
            if rank < 3:
                supported = []
                break
        supported = supporting_cells(plane) if supported else []
        if len(supported) < int(min_samples):
            continue
        used = labels[[index for index, _ in supported]]
        bands_used = sorted(set(used[:, 0].tolist()))
        sectors_used = sorted(set(used[:, 1].tolist()))
        spanning_sectors = []
        for sector in sectors_used:
            portions = [(labels[index, 0], part) for index, part in supported
                        if labels[index, 1] == sector]
            if len({band for band, _ in portions}) < 2:
                continue
            radial = np.concatenate([np.linalg.norm(part[:, :2], axis=1)
                                     for _, part in portions])
            if float(np.ptp(radial)) >= 0.35:
                spanning_sectors.append(sector)
        # Even an alternative surface must have real two-dimensional support,
        # not a plane crossing one thin strip of a wall or the main floor.
        if len(bands_used) < 2 or len(spanning_sectors) < 4:
            continue
        cloud = np.concatenate([part for _, part in supported])
        residual = float(np.median(np.abs(cloud[:, 2] - cloud[:, :2] @ plane[:2] - plane[2])))
        slope = float(np.linalg.norm(plane[:2]))
        if (slope > float(max_slope) or residual > float(max_residual_m)
                or plane[2] > -float(min_below_m)):
            continue
        if any(np.max(np.abs(design @ (plane - item[0]))) <= 2 * tolerance
               for item in coherent):
            continue
        coherent.append((plane, {
            "plane_coefficients_z_equals_ax_plus_by_plus_c": plane.tolist(),
            "slope_rad": math.atan(slope),
            "slope_deg": round(math.degrees(math.atan(slope)), 3),
            "fit_residual_m": round(residual, 6),
            "samples": len(supported), "inlier_points": len(cloud),
            "radial_bands_used": bands_used, "azimuth_sectors_used": sectors_used,
            "sectors_with_radial_span": spanning_sectors,
        }))
    evidence["candidate_surface_count"] = len(coherent)
    required_spanning_sectors = math.ceil(int(sectors) / 2)
    evidence["required_spanning_sectors"] = required_spanning_sectors
    # Qualification precedes ambiguity: a plane crossing single thin strips
    # across many sectors is not a broad floor. Half the sectors must each
    # contain real support across two radial bands and >=0.35 m radial spread.
    broad = [item for item in coherent
             if len(item[1]["sectors_with_radial_span"]) >= required_spanning_sectors]
    narrow = [item for item in coherent
              if len(item[1]["sectors_with_radial_span"]) < required_spanning_sectors]
    evidence["coherent_surface_count"] = len(broad)
    evidence["rejected_candidate_surfaces"] = [
        dict(details, rejection_reason="insufficient_spanning_azimuth_sectors")
        for _, details in narrow
    ]
    if len(broad) > 1:
        evidence["reason"] = "floor_ambiguous_surfaces"
        evidence["candidate_surfaces"] = [details for _, details in broad]
        return None, evidence
    if not broad:
        evidence["reason"] = ("floor_not_observed_across_sectors" if coherent
                              else "floor_plane_inconsistent")
        return None, evidence
    plane, details = broad[0]
    # A genuinely independent smaller second level remains ambiguous. A tilted
    # hypothesis borrowing most of the broad floor's own points does not: its
    # independent support must still meet the original multi-cell/radius gates.
    # This is not selecting the largest/highest/lowest plane.
    independent_surfaces = []
    for alternative, alternative_details in narrow:
        supported = supporting_cells(alternative, independent_of=plane)
        if len(supported) < int(min_samples):
            continue
        spanning = []
        for sector in set(labels[index, 1] for index, _ in supported):
            portions = [(labels[index, 0], part) for index, part in supported
                        if labels[index, 1] == sector]
            if len({band for band, _ in portions}) < 2:
                continue
            radial = np.concatenate([np.linalg.norm(part[:, :2], axis=1)
                                     for _, part in portions])
            if float(np.ptp(radial)) >= 0.35:
                spanning.append(int(sector))
        if len(spanning) >= 4:
            independent_surfaces.append(dict(
                alternative_details, independent_support_cells=len(supported),
                independent_spanning_sectors=sorted(spanning),
            ))
    if independent_surfaces:
        evidence["reason"] = "floor_ambiguous_surfaces"
        evidence["coherent_surface_count"] = 1 + len(independent_surfaces)
        evidence["candidate_surfaces"] = [details, *independent_surfaces]
        return None, evidence
    evidence.update(details)
    return plane, evidence


def floor_height_at(floor_plane, points_base):
    """Return the measured ground height under every base-frame point."""
    points = np.asarray(points_base, dtype=float)
    a, b, c = (float(value) for value in np.asarray(floor_plane, dtype=float).reshape(-1))
    return a * points[:, 0] + b * points[:, 1] + c


def azimuth_gaps(
    points_base,
    *,
    bin_deg=AZIMUTH_BIN_DEG,
    min_returns=MIN_RETURNS_PER_AZIMUTH_BIN,
    min_gap_deg=25.0,
    max_range_m=SENSOR_MAX_RANGE_M,
):
    """Return contiguous azimuth spans with no returns (sensor blind sectors).

    A 296-degree native sweep (or a republished partial circle) leaves a fixed
    sensor-frame wedge with no returns; treating that wedge as observed space is
    exactly the failure this contract refuses. The gaps are measured from this
    cloud, not configured.
    """
    points = np.asarray(points_base, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise DeskewedCloudError("points must be an Nx3 array")
    if points.shape[0] == 0:
        return []
    ranges = np.linalg.norm(points[:, :2], axis=1)
    usable = (ranges > 0.05) & (ranges <= float(max_range_m))
    azimuth = np.degrees(np.arctan2(points[usable, 1], points[usable, 0])) % 360.0
    total_bins = int(round(360.0 / float(bin_deg)))
    if total_bins <= 0:
        raise DeskewedCloudError("invalid azimuth bin size")
    histogram, _ = np.histogram(azimuth, bins=total_bins, range=(0.0, 360.0))
    covered = histogram >= int(min_returns)
    if bool(covered.all()):
        return []
    if not bool(covered.any()):
        return [{"start_deg": 0.0, "end_deg": 360.0, "width_deg": 360.0}]
    first = int(np.flatnonzero(covered)[0])
    order = np.roll(np.arange(total_bins), -first)
    rolled = covered[order]
    gaps = []
    index = 0
    while index < total_bins:
        if rolled[index]:
            index += 1
            continue
        start = index
        while index < total_bins and not rolled[index]:
            index += 1
        width = (index - start) * float(bin_deg)
        if width >= float(min_gap_deg):
            start_deg = float(order[start] % total_bins) * float(bin_deg)
            gaps.append(
                {
                    "start_deg": round(start_deg, 3),
                    "end_deg": round((start_deg + width) % 360.0, 3),
                    "width_deg": round(width, 3),
                }
            )
    return gaps


def _direction_centres():
    return (("front", 0.0), ("left", 90.0), ("rear", 180.0), ("right", -90.0))


def direction_evidence(
    points_base,
    *,
    floor_z_m=None,
    floor_plane=None,
    corridor_min_m=CORRIDOR_MIN_M,
    corridor_length_m=CORRIDOR_LENGTH_M,
    corridor_bins=CORRIDOR_BINS,
    half_deg=CORRIDOR_HALF_DEG,
    radial_min_m=None,
    floor_visible_min_m=None,
    collision_low_m=COLLISION_LOW_M,
    collision_high_m=COLLISION_HIGH_M,
    floor_band_m=FLOOR_BAND_M,
    azimuth_bin_deg=AZIMUTH_BIN_DEG,
    min_returns_per_azimuth_bin=MIN_RETURNS_PER_AZIMUTH_BIN,
    required_azimuth_fraction=REQUIRED_AZIMUTH_FRACTION,
    min_radial_points_per_bin=MIN_RADIAL_POINTS_PER_BIN,
    min_floor_points=MIN_FLOOR_POINTS,
):
    """Per-direction clearance, azimuth/radial coverage and floor support.

    A direction is measured only when every azimuth sub-bin and every radial bin
    of its own 90-degree corridor carries returns, the floor is visible under it
    and no radial bin shows returns only below the measured floor (a drop or stair
    void is unknown, never free). Dense returns in one narrow beam do not cover the
    corridor. Support is assessed at the fixed corridor/bin resolution, not a
    continuous traversability guarantee. ``radial_min_m`` is retained only for
    call compatibility; neither it nor floor visibility can shrink the corridor.
    """
    if floor_plane is not None:
        plane = np.asarray(floor_plane, dtype=float).reshape(-1)
        if plane.shape != (3,) or not np.isfinite(plane).all():
            raise DeskewedCloudError("floor plane must be three finite coefficients")
        floor_z = float(plane[2])
    elif floor_z_m is not None:
        floor_z = _finite(floor_z_m, "floor_z_m")
    else:
        raise DeskewedCloudError("a measured floor height or plane is required")
    if not (corridor_min_m > 0.0 and corridor_length_m > corridor_min_m):
        raise DeskewedCloudError("invalid corridor radial bounds")
    if not (0 <= collision_low_m < collision_high_m):
        raise DeskewedCloudError("invalid collision height band")
    points = np.asarray(points_base, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise DeskewedCloudError("points must be an Nx3 array")
    if points.shape[0] == 0:
        raise DeskewedCloudError("no base-frame points to evaluate")
    if not np.isfinite(points).all():
        raise DeskewedCloudError("non-finite base-frame points")
    ranges = np.linalg.norm(points[:, :2], axis=1)
    z = points[:, 2]
    azimuth = np.degrees(np.arctan2(points[:, 1], points[:, 0])) % 360.0
    corridor = (ranges > 0.05) & (ranges <= float(corridor_length_m))
    if floor_plane is not None:
        above_floor = z - floor_height_at(floor_plane, points)
    else:
        above_floor = z - floor_z
    floor_mask = (np.abs(above_floor) <= float(floor_band_m)) & corridor
    collision_mask = (
        (above_floor >= float(collision_low_m))
        & (above_floor <= float(collision_high_m))
        & corridor
    )
    below_floor_mask = (above_floor < -float(floor_band_m)) & corridor
    radial_start = float(corridor_min_m)
    # Visibility describes the sensor, not ground support. Never move the
    # corridor origin outward to excuse an unobserved near-field floor.
    if not (radial_start < float(corridor_length_m)):
        raise DeskewedCloudError("no observable radial corridor remains")
    radial_edges = np.linspace(
        radial_start, float(corridor_length_m), int(corridor_bins) + 1
    )
    azimuth_half_bins = int(round(2.0 * float(half_deg) / float(azimuth_bin_deg)))
    half = float(half_deg)
    result = {}
    for name, centre_deg in _direction_centres():
        delta = np.abs(((azimuth - centre_deg + 180.0) % 360.0) - 180.0)
        sector = (delta <= half) & corridor
        offsets = ((azimuth - centre_deg + 180.0) % 360.0) - 180.0
        covered_azimuth_bins = 0
        for index in range(azimuth_half_bins):
            low = -half + index * float(azimuth_bin_deg)
            high = low + float(azimuth_bin_deg)
            count = int(
                np.count_nonzero(
                    sector & (offsets >= low) & (offsets < high)
                )
            )
            if count >= int(min_returns_per_azimuth_bin):
                covered_azimuth_bins += 1
        azimuth_fraction = covered_azimuth_bins / float(azimuth_half_bins)
        radial_counts = []
        radial_floor = []
        radial_collision = []
        radial_below_floor = []
        for low, high in zip(radial_edges, radial_edges[1:]):
            in_bin = sector & (ranges >= low) & (ranges < high)
            radial_counts.append(int(np.count_nonzero(in_bin)))
            radial_floor.append(int(np.count_nonzero(in_bin & floor_mask)))
            radial_collision.append(int(np.count_nonzero(in_bin & collision_mask)))
            radial_below_floor.append(int(np.count_nonzero(in_bin & below_floor_mask)))
        radial_covered = all(
            count >= int(min_radial_points_per_bin) for count in radial_counts
        )
        sector_floor_ranges = ranges[sector & floor_mask]
        observed_start = (
            float(np.min(sector_floor_ranges)) if sector_floor_ranges.size else
            float(corridor_length_m)
        )
        drop_bins = [
            index
            for index in range(len(radial_counts))
            if radial_below_floor[index] > 0
            and radial_floor[index] == 0
            and radial_collision[index] == 0
        ]
        floor_points = int(np.count_nonzero(sector & floor_mask))
        collision_points = int(np.count_nonzero(sector & collision_mask))
        if collision_points:
            clearance = float(np.min(ranges[sector & collision_mask]))
        else:
            clearance = float(corridor_length_m)
        observed_floor_supported = floor_points >= int(min_floor_points)
        # Ground support is required in every fixed radial bin. A wall return
        # cannot replace floor, and a fitted far plane cannot fill a blind bin.
        floor_supported = (
            observed_floor_supported
            and observed_start <= float(corridor_min_m) + 1e-9
            and all(count >= int(min_radial_points_per_bin) for count in radial_floor)
        )
        floor_state = (
            "supported" if floor_supported else
            "partial" if observed_floor_supported else "unsupported"
        )
        reasons = []
        if azimuth_fraction + 1e-9 < float(required_azimuth_fraction):
            reasons.append("azimuth_uncovered")
        if not radial_covered:
            reasons.append("radial_uncovered")
        if not floor_supported:
            reasons.append("floor_unsupported")
        if observed_start > float(corridor_min_m) + 1e-9:
            reasons.append("near_field_floor_unobserved")
        if drop_bins:
            reasons.append("drop_unobserved")
        measured = not reasons
        result[name] = {
            "rays": int(np.count_nonzero(sector)),
            "corridor_points": int(
                np.count_nonzero(sector & (ranges > float(corridor_min_m)))
            ),
            "azimuth_bins": int(azimuth_half_bins),
            "azimuth_bins_covered": int(covered_azimuth_bins),
            "azimuth_coverage_fraction": round(float(azimuth_fraction), 4),
            "radial_bin_points": radial_counts,
            "radial_start_m": round(float(radial_start), 4),
            "near_field_floor_unobserved_m": [
                0.0,
                round(float(observed_start), 4),
            ],
            "floor_visible_min_range_m": (
                None if floor_visible_min_m is None
                else round(float(floor_visible_min_m), 4)
            ),
            "radial_bin_floor_points": radial_floor,
            "radial_bin_collision_points": radial_collision,
            "radial_bins_required": int(corridor_bins),
            "floor_points": floor_points,
            "collision_points": collision_points,
            "observation_state": "measured" if measured else "insufficient_observation",
            "clearance_m": round(float(clearance), 4) if measured else None,
            "floor_observation_state": floor_state,
            "floor_support_complete": bool(floor_supported),
            "observed_floor_supported": bool(observed_floor_supported),
            "nearest_collision_return_m": (
                round(float(clearance), 4) if collision_points else None
            ),
            "drop_bins": drop_bins,
            "reason": None if measured else ",".join(reasons),
        }
    return result


def corridor_contract():
    """The fixed body corridor geometry used by every sample."""
    return {
        "forward_m": [CORRIDOR_MIN_M, CORRIDOR_LENGTH_M],
        "radial_bins": CORRIDOR_BINS,
        "half_angle_deg": CORRIDOR_HALF_DEG,
        "collision_height_above_floor_m": [COLLISION_LOW_M, COLLISION_HIGH_M],
        "floor_band_m": FLOOR_BAND_M,
        "azimuth_bin_deg": AZIMUTH_BIN_DEG,
        "min_returns_per_azimuth_bin": MIN_RETURNS_PER_AZIMUTH_BIN,
        "required_azimuth_fraction": REQUIRED_AZIMUTH_FRACTION,
        "min_radial_points_per_bin": MIN_RADIAL_POINTS_PER_BIN,
        "min_floor_points_per_direction": MIN_FLOOR_POINTS,
    }


def fail_closed_payload(*, stamp_s, arrival_wall_s, error):
    """A broken producer chain is unknown space, never a clearance."""
    return {
        "version": 1,
        "source": "rtx_lidar_lio_deskewed_cloud_registered_full",
        "frame_id": "go2_base",
        "stamp_s": None if stamp_s is None else _finite(stamp_s, "stamp_s"),
        "arrival_wall_s": (
            None if arrival_wall_s is None else _finite(arrival_wall_s, "arrival_wall_s")
        ),
        "clearance_m": None,
        "observation_state": "producer_geometry_invalid",
        "depth_valid_ratio": None,
        "directions": {
            name: {
                "rays": 0,
                "observation_state": "insufficient_observation",
                "clearance_m": None,
                "floor_observation_state": "unsupported",
                "reason": "producer_geometry_invalid",
            }
            for name in DIRECTIONS
        },
        "corridor_points": 0,
        "cloud_points": 0,
        "corridor": corridor_contract(),
        "includes_every_obstacle": False,
        "usable_as_pose": False,
        "fail_closed": True,
        "producer_error": str(error) or "deskewed cloud could not be paired",
        "note": "fail-closed LiDAR perception sample; the tracker must hold",
    }


def build_perception_payload(
    points_base,
    *,
    stamp_s,
    arrival_wall_s,
    producer_metadata,
    source_topic=DESKEWED_CLOUD_TOPIC_LEAF,
):
    """Build one LiDAR direction-coverage sample in the base frame."""
    stamp = _finite(stamp_s, "stamp_s")
    arrival = _finite(arrival_wall_s, "arrival_wall_s")
    if not isinstance(producer_metadata, dict) or not producer_metadata:
        raise DeskewedCloudError("producer metadata is required")
    if "reference_stamp_ns" not in producer_metadata:
        raise DeskewedCloudError("producer metadata lacks the reference stamp")
    floor_plane, floor_evidence = fit_floor_plane(points_base)
    floor_z = None if floor_plane is None else float(np.asarray(floor_plane)[2])
    gaps = azimuth_gaps(points_base)
    floor_visible_min = None
    floor_visible_evidence = {"reason": "floor_unmeasured"}
    if floor_plane is not None:
        floor_visible_min, floor_visible_evidence = floor_visible_min_range(
            points_base, floor_plane=floor_plane
        )
    geometry = {
        "floor_z_m": None if floor_z is None else round(float(floor_z), 6),
        "floor_plane_z_equals_ax_plus_by_plus_c": (
            None
            if floor_plane is None
            else [round(float(value), 6) for value in np.asarray(floor_plane).reshape(-1)],
        ),
        "floor_evidence": floor_evidence,
        "floor_visible_min_range_m": (
            None if floor_visible_min is None else round(float(floor_visible_min), 4)
        ),
        "floor_visibility_evidence": floor_visible_evidence,
        "near_field_floor_unobserved_m": [
            0.0,
            round(
                float(floor_visible_min) if floor_visible_min is not None else CORRIDOR_LENGTH_M,
                4,
            ),
        ],
        "blind_azimuth_sectors_deg": gaps,
        "collision_band_above_floor_m": [COLLISION_LOW_M, COLLISION_HIGH_M],
        "measured_from_returns_only": True,
    }
    points = np.asarray(points_base, dtype=float)
    if floor_z is None:
        directions = {
            name: {
                "rays": 0,
                "observation_state": "insufficient_observation",
                "clearance_m": None,
                "floor_observation_state": "unsupported",
                "reason": "floor_unmeasured",
            }
            for name in DIRECTIONS
        }
        front = directions["front"]
    else:
        directions = direction_evidence(
            points,
            floor_plane=floor_plane,
            floor_visible_min_m=floor_visible_min,
        )
        front = directions["front"]
    measured = front["observation_state"] == "measured"
    return {
        "version": 1,
        "source": "rtx_lidar_" + source_topic.rsplit("/", 1)[-1],
        "frame_id": "go2_base",
        "stamp_s": float(stamp),
        "arrival_wall_s": float(arrival),
        "clearance_m": front["clearance_m"] if measured else None,
        "observation_state": front["observation_state"],
        "depth_valid_ratio": None,
        "directions": directions,
        "corridor_points": int(front.get("corridor_points", 0)),
        "cloud_points": int(points.shape[0]),
        "corridor": corridor_contract(),
        "includes_every_obstacle": False,
        "usable_as_pose": False,
        "fail_closed": not measured,
        "lidar_geometry": geometry,
        "frame_chain": {
            "map": producer_metadata.get("map_frame"),
            "imu": producer_metadata.get("child_frame", "go2_imu"),
            "base": "go2_base",
            "transform": "p_imu = R_map_imu^T*(p_map - t_map_imu); p_base = p_imu + imu_mount",
            "deskew": "native FAST-LIVO2 IMU deskew consumed as published (no second deskew)",
            "operator_map": "cloud consumed as frame=map with the same-stage LIO pose; no second transform",
        },
        "producer_metadata": {
            "stage": producer_metadata.get("stage"),
            "reference_stamp_ns": int(producer_metadata["reference_stamp_ns"]),
            "cloud_stamp_ns": int(
                producer_metadata.get(
                    "cloud_stamp_ns", producer_metadata["reference_stamp_ns"]
                )
            ),
            "pose_stamp_ns": int(
                producer_metadata.get(
                    "pose_stamp_ns", producer_metadata["reference_stamp_ns"]
                )
            ),
            "cloud_topic": producer_metadata.get("cloud_topic"),
            "pose_topic": producer_metadata.get("pose_topic"),
            "binary_sha256": producer_metadata.get("binary_sha256"),
        },
        "note": (
            "LiDAR direction coverage replaces depth_valid_ratio; a full-circle-scan "
            "topic is not proof that every azimuth was observed, and unknown "
            "directions or missing floor support must hold"
        ),
    }


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
