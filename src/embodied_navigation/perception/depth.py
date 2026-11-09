"""RGB-D point cloud in the actual Go2 IMU frame, from the saved source."""

from dataclasses import dataclass

import numpy as np

_POINT_BUDGET = 8000


@dataclass(frozen=True)
class DepthCloudPacket:
    """A sampled cloud and its correspondence to the original depth image.

    ``pixel_flat_indices`` indexes the C-order flattened input image and has
    one entry per row of ``points``. ``depth_keep_mask`` has the image shape;
    ``self_mask`` indexes the valid-depth points, before self-filtering/stride.
    Array contents remain ordinary NumPy arrays owned by the caller.
    """

    points: np.ndarray
    pixel_flat_indices: np.ndarray
    depth_keep_mask: np.ndarray
    self_mask: np.ndarray


def _point_stride(count):
    return max(1, (count + _POINT_BUDGET - 1) // _POINT_BUDGET)


def depth_to_cloud(
    depth,
    sensor_yaw,
    camera_mount,
    imu_mount,
    focal_px=512 * 24 / 42,
    center_px=(256, 256),
):
    """Unproject RGB-D depth into the Go2 IMU frame and bound the point budget.

    Keeps finite depth in the 0.05 m..20 m validity window, drops returns inside
    the body self-mask box and strides to about 8000 points. Returns
    ``(points_in_imu_frame, depth_keep_mask, self_mask)``.
    """
    depth = np.asarray(depth)
    row, col = np.indices(depth.shape)
    # Valid range window and the pinhole unprojection (camera y/z axes inverted).
    keep = np.isfinite(depth) & (depth > 0.05) & (depth < 20)
    points = np.column_stack(
        [
            depth[keep],
            -(col[keep] - center_px[0]) / focal_px * depth[keep],
            -(row[keep] - center_px[1]) / focal_px * depth[keep],
        ]
    ) @ sensor_yaw.T + (camera_mount - imu_mount)
    body = points + imu_mount
    # Robot self-returns inside this body box are removed before sampling.
    self_mask = (
        (np.abs(body[:, 0]) < 0.5)
        & (np.abs(body[:, 1]) < 0.3)
        & (body[:, 2] > -0.45)
        & (body[:, 2] < 0.3)
    )
    points = points[~self_mask]
    # Uniform stride keeps at most about 8000 points for downstream consumers.
    return points[:: _point_stride(len(points))], keep, self_mask


def _calibration_array(value, name, shape):
    array = np.asarray(value)
    if array.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {array.shape}")
    try:
        valid = np.isrealobj(array) and bool(np.isfinite(array).all())
    except TypeError:
        valid = False
    if not valid:
        raise ValueError(f"{name} must contain finite real values")
    return array


def depth_cloud_packet(
    depth,
    sensor_yaw,
    camera_mount,
    imu_mount,
    focal_px=512 * 24 / 42,
    center_px=(256, 256),
):
    """Return the legacy cloud with exact original-pixel correspondence.

    The caller supplies the same camera intrinsics/extrinsics used by
    :func:`depth_to_cloud`. Pixel indices follow valid-depth filtering, then
    the body self-mask, then the very same stride as the returned points.
    This function validates dimensions/calibration; nonfinite depth values
    are filtered by the existing validity mask. It does not change the range,
    body box, coordinate convention or point budget of the legacy API.
    """
    depth = np.asarray(depth)
    if depth.ndim != 2:
        raise ValueError(f"depth must be two-dimensional, got shape {depth.shape}")
    if depth.dtype.kind not in "fiu":
        raise ValueError("depth must contain real numeric values")
    rotation = _calibration_array(sensor_yaw, "sensor_yaw", (3, 3))
    camera = _calibration_array(camera_mount, "camera_mount", (3,))
    imu = _calibration_array(imu_mount, "imu_mount", (3,))
    center = _calibration_array(center_px, "center_px", (2,))
    focal = _calibration_array(focal_px, "focal_px", ())
    if focal <= 0:
        raise ValueError("focal_px must be positive")
    points, keep, self_mask = depth_to_cloud(
        depth, rotation, camera, imu, focal_px, center
    )
    surviving_pixels = np.flatnonzero(keep)[~self_mask]
    pixel_flat_indices = surviving_pixels[:: _point_stride(len(surviving_pixels))]
    return DepthCloudPacket(points, pixel_flat_indices, keep, self_mask)
