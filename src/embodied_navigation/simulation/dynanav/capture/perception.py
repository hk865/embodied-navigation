"""Static-perception direction coverage for the interactive operator.

Pure numpy helpers with no Isaac and no ROS imports, so the producer logic that
labels each direction can be tested offline. The RGB-D camera looks forward: a
healthy frame proves that the camera's own rays were observed, never that the
rear or the sides were. Each direction therefore gets its own observed/unknown
state, and an unobserved direction must never be reported as free space.
"""

from __future__ import annotations

import numpy as np

DIRECTIONS = ("front", "rear", "left", "right")

DEFAULT_MIN_SECTOR_POINTS = 50
DEFAULT_FORWARD_RANGE_M = (0.30, 4.0)
DEFAULT_HALF_WIDTH_M = 0.45
DEFAULT_HEIGHT_BAND_M = (-0.05, 1.20)
DEFAULT_EMPTY_CLEAR_M = 4.0


def classify_directions(points):
    """Return the sector name of every base-frame point.

    A point belongs to the axis it spreads along most; the front and rear
    halves meet at the body origin, so a forward-only camera can never own a
    rear sector point.
    """
    array = np.asarray(points, dtype=float).reshape(-1, 3)
    x = array[:, 0]
    y = array[:, 1]
    labels = np.full(len(array), "front", dtype=object)
    rear = (x < 0.0) & (np.abs(x) >= np.abs(y))
    labels[rear] = "rear"
    left = (y > 0.0) & (np.abs(y) > np.abs(x))
    labels[left] = "left"
    right = (y < 0.0) & (np.abs(y) > np.abs(x))
    labels[right] = "right"
    return labels


def _corridor(points, name, forward_range, half_width, height_band):
    """Return the along-axis distances of the points inside one corridor."""
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    low, high = forward_range
    if name == "front":
        along, across = x, y
    elif name == "rear":
        along, across = -x, y
    elif name == "left":
        along, across = y, x
    else:
        along, across = -y, x
    selected = (
        (along > low)
        & (along < high)
        & (np.abs(across) <= half_width)
        & (z >= height_band[0])
        & (z <= height_band[1])
    )
    return along[selected]


def _near_field(points, name, forward_range, half_width, height_band):
    """Return along-axis distances of returns inside the near band.

    The corridor starts at forward_range[0] because closer returns were assumed
    to be the robot body. The body self-echo box is narrower than the corridor
    half width, so a real obstacle inside the near band can survive self-masking
    and still be excluded from the corridor: it was then invisible, and a healthy
    frame reported clear_free far beyond it. A surviving near return is a real
    obstacle, so it must set the clearance instead of being dropped.
    """
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    low = float(forward_range[0])
    if name == "front":
        along, across = x, y
    elif name == "rear":
        along, across = -x, y
    elif name == "left":
        along, across = y, x
    else:
        along, across = -y, x
    selected = (
        (along > 0.0)
        & (along <= low)
        & (np.abs(across) <= half_width)
        & (z >= height_band[0])
        & (z <= height_band[1])
    )
    return along[selected]


def direction_coverage(
    points,
    valid_mask=None,
    self_mask=None,
    *,
    min_sector_points=DEFAULT_MIN_SECTOR_POINTS,
    forward_range=DEFAULT_FORWARD_RANGE_M,
    half_width=DEFAULT_HALF_WIDTH_M,
    height_band=DEFAULT_HEIGHT_BAND_M,
    empty_clear_m=DEFAULT_EMPTY_CLEAR_M,
):
    """Per-direction clearance with an explicit observed/unknown state.

    points is the base-frame cloud with one row per depth pixel, so valid_mask
    and self_mask filter it exactly like the publisher does. A direction is
    'observed' only when at least min_sector_points of its own rays returned a
    valid depth; otherwise its state is insufficient_observation with no
    clearance, regardless of how healthy the forward frame is.
    """
    array = np.asarray(points, dtype=float).reshape(-1, 3)
    if array.size == 0:
        array = np.zeros((0, 3), dtype=float)
    count = len(array)
    if valid_mask is None:
        valid = np.ones(count, dtype=bool)
    else:
        valid = np.asarray(valid_mask, dtype=bool).reshape(-1)[:count]
    if self_mask is None:
        own = np.zeros(count, dtype=bool)
    else:
        own = np.asarray(self_mask, dtype=bool).reshape(-1)[:count]
    usable = valid & ~own
    cloud = array[: len(usable)][usable]
    labels = classify_directions(cloud)
    result = {}
    for name in DIRECTIONS:
        sector = cloud[labels == name]
        ray_count = int(len(sector))
        distances = _corridor(
            sector, name, forward_range, half_width, height_band
        )
        near = _near_field(sector, name, forward_range, half_width, height_band)
        entry = {
            "rays": ray_count,
            "points": int(len(distances)),
        }
        if ray_count < int(min_sector_points):
            entry.update(
                observation_state="insufficient_observation",
                clearance_m=None,
                reason="no_ray_coverage",
            )
        elif len(near):
            entry.update(
                observation_state="measured",
                clearance_m=float(np.min(near)),
                reason="near_field_return",
            )
        elif len(distances):
            entry.update(
                observation_state="measured",
                clearance_m=float(np.min(distances)),
                reason=None,
            )
        else:
            entry.update(
                observation_state="clear_free",
                clearance_m=float(empty_clear_m),
                reason="observed_rays_without_returns",
            )
        result[name] = entry
    return result

FAIL_CLOSED_OBSERVATION_STATE = "producer_geometry_invalid"

CORRIDOR = {
    "forward_m": [DEFAULT_FORWARD_RANGE_M[0], DEFAULT_FORWARD_RANGE_M[1]],
    "half_width_m": DEFAULT_HALF_WIDTH_M,
    "height_m": [DEFAULT_HEIGHT_BAND_M[0], DEFAULT_HEIGHT_BAND_M[1]],
    "empty_means_clear_to_m": DEFAULT_EMPTY_CLEAR_M,
    "near_field_m": [0.0, DEFAULT_FORWARD_RANGE_M[0]],
    "near_field_returns_are_obstacles": True,
}


def _vector3(value, name):
    """Validate one calibration vector; a wrong or missing extrinsic must raise."""
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (3,):
        raise ValueError(f"{name} must have three components, got {array.shape}")
    if not np.isfinite(array).all():
        raise ValueError(f"{name} must contain finite values")
    return array


def base_frame_cloud(packet, imu_mount):
    """Return the producer cloud in the base frame with pixel-exact masks.

    The producer (perception.depth.depth_cloud_packet) returns points in the
    Go2 IMU frame, already filtered by valid depth, by the body self-echo box and
    by the point-budget stride; pixel_flat_indices carries the C-order image
    index of every surviving row; depth_keep_mask is the image-shaped validity
    mask and self_mask indexes the valid-depth points only. The mask arrays and
    the point array therefore live in two different index spaces, so pairing
    them by array position is meaningless.

    The base cloud is the IMU cloud plus the measured IMU origin in the base
    frame (imu_mount): the IMU is installed axis-aligned with the base, so this
    is a pure translation, never a relabelling. The per-point validity and
    self-echo flags are gathered through pixel_flat_indices, so a stale or
    misaligned index raises instead of producing a plausible-looking subset.
    """
    points = np.asarray(packet.points, dtype=float)
    indices = np.asarray(packet.pixel_flat_indices)
    keep = np.asarray(packet.depth_keep_mask)
    self_mask = np.asarray(packet.self_mask)
    imu = _vector3(imu_mount, "imu_mount")
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points must be N x 3, got {points.shape}")
    if not np.isfinite(points).all():
        raise ValueError("points must contain finite values")
    if keep.ndim != 2 or keep.size == 0:
        raise ValueError(
            f"depth_keep_mask must be the image-shaped mask, got {keep.shape}"
        )
    if keep.dtype != np.bool_:
        raise ValueError("depth_keep_mask must be boolean")
    if self_mask.ndim != 1 or self_mask.dtype != np.bool_:
        raise ValueError(
            "self_mask must be a one-dimensional boolean mask over valid-depth "
            f"points, got shape {self_mask.shape} dtype {self_mask.dtype}"
        )
    flat_keep = keep.reshape(-1)
    valid_pixels = np.flatnonzero(flat_keep)
    if self_mask.size != valid_pixels.size:
        raise ValueError(
            "self_mask must have one entry per valid-depth pixel, got "
            f"{self_mask.size} for {valid_pixels.size} valid points"
        )
    if indices.ndim != 1 or not np.issubdtype(indices.dtype, np.integer):
        raise ValueError("pixel_flat_indices must be a one-dimensional integer array")
    if indices.size != points.shape[0]:
        raise ValueError(
            "pixel_flat_indices must have one entry per point, got "
            f"{indices.size} indices for {points.shape[0]} points"
        )
    if indices.size == 0:
        raise ValueError("an empty cloud cannot be published as observed space")
    if int(indices.min()) < 0 or int(indices.max()) >= flat_keep.size:
        raise ValueError("pixel_flat_indices point outside the depth image")
    valid = flat_keep[indices]
    if not bool(valid.all()):
        raise ValueError("pixel_flat_indices refer to invalid-depth pixels")
    self_flat = np.zeros(flat_keep.size, dtype=bool)
    self_flat[valid_pixels[self_mask]] = True
    own = self_flat[indices]
    if bool(own.any()):
        raise ValueError("pixel_flat_indices refer to self-echo pixels")
    return points + imu.reshape(1, 3), valid, own


def _unobserved_directions(reason):
    return {
        name: {
            "rays": 0,
            "points": 0,
            "observation_state": "insufficient_observation",
            "clearance_m": None,
            "reason": reason,
        }
        for name in DIRECTIONS
    }


def fail_closed_perception_payload(
    *, stamp_s, arrival_wall_s, error, depth_valid_ratio=None
):
    """Publish unpaired arrays as unknown space, never as free space.

    The tracker holds whenever the observation state is neither measured nor
    clear_free, so a broken producer chain becomes a hold instead of a
    clearance that happens to look safe.
    """
    message = str(error) or "unpaired perception arrays"
    return {
        "version": 1,
        "source": "live_rgbd_obstacle_cloud",
        "frame_id": "go2_base",
        "stamp_s": None if stamp_s is None else float(stamp_s),
        "arrival_wall_s": None if arrival_wall_s is None else float(arrival_wall_s),
        "clearance_m": None,
        "observation_state": FAIL_CLOSED_OBSERVATION_STATE,
        "depth_valid_ratio": (
            None if depth_valid_ratio is None else float(depth_valid_ratio)
        ),
        "directions": _unobserved_directions(FAIL_CLOSED_OBSERVATION_STATE),
        "corridor_points": 0,
        "cloud_points": 0,
        "corridor": dict(CORRIDOR),
        "includes_every_obstacle": False,
        "usable_as_pose": False,
        "fail_closed": True,
        "producer_error": message,
        "frame_provenance": {
            "frame_id": "go2_base",
            "derivation": (
                "refused: the producer cloud could not be paired with its "
                "pixel indices"
            ),
            "self_mask_excluded": None,
            "relabelled_by_cli": False,
        },
        "note": (
            "fail-closed perception sample: unknown or unpaired arrays are "
            "never reported as free space and the tracker must hold"
        ),
    }


def interactive_perception_payload(
    packet,
    *,
    imu_mount,
    stamp_s,
    arrival_wall_s,
    depth_valid_ratio=None,
    min_sector_points=DEFAULT_MIN_SECTOR_POINTS,
):
    """Build the live perception sample from the production depth packet.

    Returns a JSON-ready dict in the base frame. Any pairing or calibration
    failure is converted into a fail-closed sample instead of an exception, so
    the 10 Hz producer keeps running while the gate holds.
    """
    try:
        base, valid, own = base_frame_cloud(packet, imu_mount)
    except (AttributeError, TypeError, ValueError) as error:
        return fail_closed_perception_payload(
            stamp_s=stamp_s,
            arrival_wall_s=arrival_wall_s,
            error=error,
            depth_valid_ratio=depth_valid_ratio,
        )
    directions = direction_coverage(
        base, valid, own, min_sector_points=min_sector_points
    )
    front = directions["front"]
    imu = _vector3(imu_mount, "imu_mount")
    return {
        "version": 1,
        "source": "live_rgbd_obstacle_cloud",
        "frame_id": "go2_base",
        "stamp_s": None if stamp_s is None else float(stamp_s),
        "arrival_wall_s": None if arrival_wall_s is None else float(arrival_wall_s),
        "clearance_m": front["clearance_m"],
        "observation_state": front["observation_state"],
        "depth_valid_ratio": (
            None if depth_valid_ratio is None else float(depth_valid_ratio)
        ),
        "directions": directions,
        "corridor_points": int(front["points"]),
        "cloud_points": int(len(base)),
        "corridor": dict(CORRIDOR),
        "includes_every_obstacle": True,
        "usable_as_pose": False,
        "fail_closed": False,
        "frame_chain": {
            "sensor": "go2_camera_optical",
            "imu": "go2_imu",
            "base": "go2_base",
            "imu_origin_in_base_m": imu.tolist(),
            "transform": (
                "base = imu + imu_origin_in_base_m (IMU axes are base axes)"
            ),
        },
        "pixel_correspondence": {
            "indices": "packet.pixel_flat_indices",
            "gathered_from": [
                "depth_keep_mask (image pixels)",
                "self_mask (valid-depth points)",
            ],
            "paired_points": int(len(base)),
            "verified": True,
        },
        "frame_provenance": {
            "frame_id": "go2_base",
            "derivation": (
                "depth_cloud_packet IMU cloud translated by the measured IMU "
                "origin in the base frame, with per-point masks gathered through "
                "pixel_flat_indices"
            ),
            "self_mask_excluded": True,
            "relabelled_by_cli": False,
        },
        "note": (
            "clearance sample from the live RGB-D obstacle cloud in the base "
            "frame; gates local motion, never a pose or a map"
        ),
    }
