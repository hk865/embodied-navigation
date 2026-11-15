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
