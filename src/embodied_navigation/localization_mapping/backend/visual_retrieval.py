"""Bounded image-only place proposals; descriptors never supply metric poses."""
from collections import OrderedDict
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VisualRetrievalConfig:
    max_entries: int = 2000
    candidates: int = 3
    max_distance: float = .25
    max_time_delta_s: float = .05
    min_contrast: float = 5.

    def __post_init__(self):
        # External JSON can otherwise create an unbounded index or accept stale
        # observations; fail before processing the first sensor frame.
        if (self.max_entries < 1 or self.candidates < 1
                or not 0 <= self.max_distance <= 2
                or not np.isfinite(self.max_time_delta_s) or self.max_time_delta_s < 0
                or not np.isfinite(self.min_contrast) or self.min_contrast < 0):
            raise ValueError('invalid visual retrieval configuration')


def image_descriptor(keyframe, config=None):
    """Normalized 16x16 mean-intensity grid from rectified pixels only.

    This small baseline tolerates brightness offset/scale, not viewpoint or
    lighting changes generally. Missing, stale, tiny or untextured observations
    yield no proposal. No LiDAR, odometry or ground truth enters the descriptor.
    """
    c = config or VisualRetrievalConfig()
    observation = keyframe.visual
    if observation is None or abs(observation.stamp_ns-keyframe.stamp_ns) > c.max_time_delta_s*1e9:
        return None
    gray = np.asarray(observation.gray, dtype=float)
    if min(gray.shape) < 16 or np.std(gray) < c.min_contrast:
        return None
    y = np.linspace(0, gray.shape[0], 17, dtype=int)
    x = np.linspace(0, gray.shape[1], 17, dtype=int)
    integral = np.pad(gray.cumsum(0).cumsum(1), ((1, 0), (1, 0)))
    blocks = (integral[y[1:, None], x[None, 1:]] - integral[y[:-1, None], x[None, 1:]]
              - integral[y[1:, None], x[None, :-1]] + integral[y[:-1, None], x[None, :-1]])
    descriptor = (blocks / (np.diff(y)[:, None]*np.diff(x)[None, :])).ravel()
    descriptor -= descriptor.mean()
    norm = np.linalg.norm(descriptor)
    if norm <= 1e-8:
        return None
    return descriptor / norm


class VisualIndex:
    """Oldest-entry eviction bounds descriptor memory and query work.

    Prior-map candidates use the same index; the caller supplies mapping versus
    relocalization exclusions. Eviction can reduce recall on long routes.
    """
    def __init__(self, config=None):
        self.config = config or VisualRetrievalConfig()
        self.descriptors = OrderedDict()

    def add(self, keyframe):
        descriptor = image_descriptor(keyframe, self.config)
        self.descriptors.pop(keyframe.key, None)
        if descriptor is None:
            return
        self.descriptors[keyframe.key] = descriptor
        while len(self.descriptors) > self.config.max_entries:
            self.descriptors.popitem(last=False)

    def query(self, keyframe, exclude=None):
        descriptor = image_descriptor(keyframe, self.config)
        if descriptor is None:
            return []
        keys = [key for key in self.descriptors if key != keyframe.key and (exclude is None or not exclude(key))]
        if not keys:
            return []
        distances = np.clip(1 - np.vstack([self.descriptors[key] for key in keys]) @ descriptor, 0, 2)
        return [(keys[i], float(distances[i])) for i in np.argsort(distances, kind='stable')[:self.config.candidates]
                if distances[i] <= self.config.max_distance]
