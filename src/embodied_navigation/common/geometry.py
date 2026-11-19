import math
import numpy as np

def wrap_angle(value):
    """Wrap radians to [-pi, pi] with atan2(sin, cos); either signed pi endpoint is possible."""
    return math.atan2(math.sin(value), math.cos(value))

def quaternion_matrix(w, x, y, z):
    """3x3 rotation of an already-normalised quaternion given as separate w, x, y, z components.

    Callers keep their own validation and normalisation; this is only the shared formula."""
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def pose_transform(position, quat_wxyz):
    """Validate finite XYZ and a unit WXYZ quaternion; return the 4x4 SE(3) transform."""
    try:
        xyz = np.asarray(position, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError("position must contain 3 finite numbers") from error
    if xyz.shape != (3,) or not np.isfinite(xyz).all():
        raise ValueError("position must contain 3 finite numbers")
    try:
        quaternion = np.asarray(quat_wxyz, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError("quaternion_wxyz must contain 4 finite numbers") from error
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("quaternion_wxyz must contain 4 finite numbers")
    norm = float(np.linalg.norm(quaternion))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError("quaternion_wxyz must be unit length within 1e-3")
    w, x, y, z = quaternion / norm
    transform = np.eye(4)
    transform[:3, :3] = quaternion_matrix(w, x, y, z)
    transform[:3, 3] = xyz
    return transform
