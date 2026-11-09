"""RGB-D point cloud in the actual Go2 IMU frame, from the saved source."""

import numpy as np


def depth_to_cloud(
    depth, sensor_yaw, camera_mount, imu_mount, focal_px=512 * 24 / 42, center_px=(256, 256)
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
    stride = max(1, (len(points) + 7999) // 8000)
    return points[::stride], keep, self_mask
