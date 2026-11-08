"""Polyline progress and lookahead, extracted from the tested Go2 shuttle."""
import numpy as np

def lookahead_target(position_xy, segment_start, segment_end, lookahead_m=0.7):
    start = np.asarray(segment_start, dtype=float)
    end = np.asarray(segment_end, dtype=float)
    delta = end - start
    length = np.linalg.norm(delta)
    if length == 0:
        raise ValueError("A route segment needs distinct endpoints")
    unit = delta / length
    progress = float(np.clip((np.asarray(position_xy) - start) @ unit, 0, length))
    return start + unit * min(length, progress + lookahead_m)
