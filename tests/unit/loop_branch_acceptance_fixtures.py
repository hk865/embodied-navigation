"""Synthetic observations for independent loop branch acceptance."""
import numpy as np
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe

class Trace:
    def __init__(self):
        self.rows = []

    def emit(self, event_name, **fields):
        self.rows.append((event_name, fields))


def room(floor_height=.05):
    first, second = np.meshgrid(np.arange(.075, 2.98, .04), np.arange(.075, 2.98, .04))
    first, second = first.ravel(), second.ravel()
    flat = lambda height: np.full(len(first), height)
    return np.vstack([
        np.column_stack([first, second, flat(floor_height)]),
        np.column_stack([first, second, flat(3.05)]),
        np.column_stack([flat(.05), first, second]),
        np.column_stack([flat(3.05), first, second]),
        np.column_stack([first, flat(.05), second]),
        np.column_stack([first, flat(3.05), second]),
    ])


def frame(sequence, cloud, stamp_ns=None):
    return Keyframe((1, sequence), sequence * 30_000_000_000 if stamp_ns is None else stamp_ns,
                    np.eye(4), np.eye(4), cloud)
