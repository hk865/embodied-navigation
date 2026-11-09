"""Cruise reserve near planned 3D stair geometry; no terrain or GT sensor input."""

import numpy as np


class StairSpeedLimit:
    def __init__(self, reference, speed=0.25):
        p = np.asarray(reference, dtype=float)
        delta = np.diff(p, axis=0)
        horizontal = np.linalg.norm(delta[:, :2], axis=1)
        stairs = (abs(delta[:, 2]) > 0.01) & (abs(delta[:, 2]) / np.maximum(horizontal, 1e-6) > 0.1)
        self.centers = ((p[:-1] + p[1:]) * 0.5)[stairs]
        self.speed = float(speed)

    def cruise(self, position, requested=0.4):
        if not len(self.centers):
            return requested
        difference = self.centers - np.asarray(position)
        near = (np.linalg.norm(difference[:, :2], axis=1) < 0.8) & (abs(difference[:, 2]) < 0.45)
        return min(requested, self.speed) if near.any() else requested
