"""Header-event PID of measured SLAM XY/yaw into bounded body velocity only."""

import math
import numpy as np


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def slew(previous, desired, dt, acceleration):
    """Rate-limit ``previous`` toward ``desired`` using the per-axis acceleration caps.

    The timestep is clamped to [0, 0.1] s; a near-zero planar request snaps the
    horizontal part to exact zero so the original stop semantics are preserved.
    """
    previous = np.asarray(previous, dtype=float)
    desired = np.asarray(desired, dtype=float)
    result = previous + np.clip(
        desired - previous,
        -np.asarray(acceleration) * max(0.0, min(dt, 0.1)),
        np.asarray(acceleration) * max(0.0, min(dt, 0.1)),
    )
    if np.linalg.norm(desired[:2]) < 1e-9:
        result[:2] = 0.0
    return result
