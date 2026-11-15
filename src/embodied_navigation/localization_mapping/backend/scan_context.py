"""ScanContext place recognition (Kim & Kim, IROS 2018), body-frame clouds.

A descriptor is a ``rings x sectors`` polar grid around the body z axis:
ring ``i`` covers ranges ``[i, i+1) * max_range_m / rings`` and sector ``j``
covers azimuths ``[j, j+1) * 2pi / sectors`` measured counter-clockwise from
body +x. Each cell holds the maximum point height ``z`` (0 when empty or when
every point lies below the body origin).

Yaw convention: ``distance(a, b)`` returns the column shift ``s`` for which
column ``j`` of ``a`` best matches column ``j + s`` of ``b``. If ``a`` is the
scan of frame A and ``b`` of frame B at the same place, then
``Rz(shift_to_yaw(s, config))`` approximates the rotation of ``T_B_A``, i.e.
it is the yaw part of an ICP initial guess with source A and target B.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class ScanContextConfig:
    rings: int = 20
    sectors: int = 60
    max_range_m: float = 20.0


def descriptor(points_body: np.ndarray, config: ScanContextConfig) -> np.ndarray:
    """(rings, sectors) max-height grid of a body-frame cloud."""
    desc = np.zeros((config.rings, config.sectors))
    radius = np.hypot(points_body[:, 0], points_body[:, 1])
    inside = radius < config.max_range_m
    points, radius = points_body[inside], radius[inside]
    azimuth = np.mod(np.arctan2(points[:, 1], points[:, 0]), 2 * np.pi)
    ring = (radius / config.max_range_m * config.rings).astype(np.int64)
    sector = np.minimum((azimuth / (2 * np.pi) * config.sectors).astype(np.int64), config.sectors - 1)
    np.maximum.at(desc, (ring, sector), points[:, 2])
    return desc


def ring_key(desc: np.ndarray) -> np.ndarray:
    """Rotation-invariant (rings,) summary: mean cell height of each ring."""
    return desc.mean(axis=1)


def distance(a: np.ndarray, b: np.ndarray) -> tuple[float, int]:
    """Minimum column-wise cosine distance over all column shifts.

    Returns ``(dist, shift)`` with ``dist`` in [0, 1]; only column pairs where
    both columns are non-empty vote. See the module docstring for ``shift``.
    """
    sectors = a.shape[1]
    norm_a, norm_b = np.linalg.norm(a, axis=0), np.linalg.norm(b, axis=0)
    similarity = (a / np.maximum(norm_a, 1e-12)).T @ (b / np.maximum(norm_b, 1e-12))
    valid = (norm_a[:, None] > 0) & (norm_b[None, :] > 0)
    columns = np.arange(sectors)
    pairs = (columns[None, :] + columns[:, None]) % sectors  # [shift, column of a]
    votes = valid[columns[None, :], pairs].sum(axis=1)
    score = similarity[columns[None, :], pairs].sum(axis=1) / np.maximum(votes, 1)
    score[votes == 0] = 0.0
    shift = int(np.argmax(score))
    return float(1.0 - score[shift]), shift


def shift_to_yaw(shift: int, config: ScanContextConfig) -> float:
    """Yaw in (-pi, pi] of ``T_B_A`` for a shift returned by ``distance(a, b)``."""
    yaw = shift * 2 * np.pi / config.sectors
    return float(np.pi - np.mod(np.pi - yaw, 2 * np.pi))


class ScanContextIndex:
    """Descriptors keyed by arbitrary hashable keys, ring-key KD-tree prefilter."""

    def __init__(self, config: ScanContextConfig):
        self.config = config
        self.keys: list = []
        self.descriptors: list[np.ndarray] = []
        self.ring_keys: list[np.ndarray] = []

    def add(self, key, desc: np.ndarray) -> None:
        self.keys.append(key)
        self.descriptors.append(desc)
        self.ring_keys.append(ring_key(desc))

    def query(
        self,
        desc: np.ndarray,
        *,
        exclude: Callable[[object], bool] | None = None,
        candidates: int = 10,
    ) -> list[tuple[object, float, float]]:
        """``(key, dist, yaw_rad)`` sorted by ``dist``; yaw is that of T_entry_query.

        The ``candidates`` nearest ring keys are re-ranked by full distance.
        """
        allowed = [i for i, key in enumerate(self.keys) if exclude is None or not exclude(key)]
        if not allowed:
            return []
        tree = cKDTree(np.asarray([self.ring_keys[i] for i in allowed]))
        _, nearest = tree.query(ring_key(desc), k=min(candidates, len(allowed)))
        results = []
        for i in np.atleast_1d(nearest):
            index = allowed[int(i)]
            dist, shift = distance(desc, self.descriptors[index])
            results.append((self.keys[index], dist, shift_to_yaw(shift, self.config)))
        return sorted(results, key=lambda row: row[1])
