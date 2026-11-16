"""Contiguous measured submaps; all points retain their contributing frame keys."""
from dataclasses import dataclass
import numpy as np
from .keyframes import transform_points
from .pose_graph import invert, pose_difference


@dataclass(frozen=True)
class SubmapConfig:
    max_frames: int = 5
    max_translation_m: float = 2.0
    max_height_m: float = 0.5
    max_rotation_rad: float = 0.35
    max_gap_s: float = 3.0
    voxel_m: float = 0.15

    def __post_init__(self):
        # JSON configuration is external: zero voxels or empty windows cannot
        # produce a measured cloud and must fail before processing keyframes.
        values = (self.max_translation_m, self.max_height_m, self.max_rotation_rad,
                  self.max_gap_s, self.voxel_m)
        if self.max_frames < 1 or not all(np.isfinite(v) and v > 0 for v in values):
            raise ValueError("submap window and metric budgets must be positive")


@dataclass(frozen=True)
class MeasuredSubmap:
    points_body: np.ndarray
    source_keys: tuple
    point_sources: tuple


def build_submap(keyframes, anchor, config=None):
    """Walk backward from anchor, stopping at the first broken continuity budget.

    Motion budgets use frontend relative poses, never corrected poses from other
    traversals. They limit exposure to relative-pose drift, not certify its absence.
    A voxel contributes once regardless of repeated observations.
    """
    c = config or SubmapConfig()
    frames = list(keyframes)
    index = next(i for i, frame in enumerate(frames) if frame.key == anchor.key)
    selected, clouds, labels = [], [], []
    previous = anchor
    for frame in reversed(frames[:index + 1]):
        delta = invert(anchor.T_odom_body) @ frame.T_odom_body
        translation, rotation = pose_difference(np.eye(4), delta)
        gap = (previous.stamp_ns - frame.stamp_ns) * 1e-9
        if (frame.key[0] != anchor.key[0] or gap < 0 or gap > c.max_gap_s
                or translation > c.max_translation_m
                or abs(frame.T_odom_body[2, 3] - anchor.T_odom_body[2, 3]) > c.max_height_m
                or rotation > c.max_rotation_rad):
            break
        selected.append(frame.key)
        points = transform_points(delta, frame.points_body())
        clouds.append(points)
        labels.extend([frame.key] * len(points))
        previous = frame
        if len(selected) >= c.max_frames:
            break
    points = np.vstack(clouds)
    _, inverse, counts = np.unique(np.floor(points / c.voxel_m).astype(np.int64),
                                   axis=0, return_inverse=True, return_counts=True)
    inverse = inverse.reshape(-1)
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, inverse, points)
    provenance = [set() for _ in counts]
    for cell, key in zip(inverse, labels):
        provenance[cell].add(key)
    return MeasuredSubmap(sums / counts[:, None], tuple(selected),
                          tuple(tuple(sorted(keys)) for keys in provenance))
