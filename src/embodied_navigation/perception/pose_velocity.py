"""Source-time pose-window velocity shared by measured navigation consumers."""

from __future__ import annotations

import numpy as np


def update_pose_velocity(history, stamp_s, position, window_s, *, distinct=True) -> np.ndarray | None:
    """Update a caller-owned deque and return map-frame velocity in metres/second.

    Inputs are already validated source timestamps and positions in one frame.
    Callers own freshness, frame/generation changes and history resets. Retain
    the endpoint preceding the window and require the full source-time span;
    repeated samples do not extend history. An incomplete window returns None.
    """
    position = np.asarray(position, dtype=float)
    if distinct:
        history.append((stamp_s, position.copy()))
        cutoff = stamp_s - window_s
        while len(history) > 2 and history[1][0] <= cutoff:
            history.popleft()
    if history:
        start, first = history[0]
        if stamp_s - start >= window_s:
            return (position - first) / (stamp_s - start)
    return None
