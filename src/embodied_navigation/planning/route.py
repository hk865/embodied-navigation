"""Polyline progress and lookahead, extracted from the tested Go2 shuttle."""

import numpy as np


def lookahead_target(position_xy, segment_start, segment_end, lookahead_m=0.7):
    """Return the point ``lookahead_m`` ahead of the clipped XY projection on the segment.

    All distances are metres; progress is clamped to ``[0, segment length]`` so a
    position behind the start or beyond the end still yields a point on the segment.
    """
    start = np.asarray(segment_start, dtype=float)
    end = np.asarray(segment_end, dtype=float)
    delta = end - start
    length = np.linalg.norm(delta)
    if length == 0:
        raise ValueError("A route segment needs distinct endpoints")
    unit = delta / length
    progress = float(np.clip((np.asarray(position_xy) - start) @ unit, 0, length))
    return start + unit * min(length, progress + lookahead_m)
