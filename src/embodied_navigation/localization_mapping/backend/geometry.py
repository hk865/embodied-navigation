"""Small, explicit SE(3) helpers. T_AB maps coordinates in B into A.

Validation belongs at the backend boundary, never on the frontend point loop.
There are no fingerprints, source-version bindings, or whole-cloud checksums.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def checked_transform(value) -> np.ndarray:
    t = np.asarray(value, dtype=np.float64)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError("transform must be a finite 4x4 matrix")
    if not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8, rtol=0):
        raise ValueError("invalid homogeneous last row")
    r = t[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-6, rtol=0) or abs(np.linalg.det(r) - 1) > 1e-6:
        raise ValueError("rotation must belong to SO(3)")
    return t.copy()


def inverse(t) -> np.ndarray:
    t = np.asarray(t, dtype=np.float64)
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ t[:3, 3]
    return out


def transform_points(points, t) -> np.ndarray:
    p = np.asarray(points)
    t = np.asarray(t)
    return p @ t[:3, :3].T + t[:3, 3]


def pose_distance(a, b) -> tuple[float, float]:
    a, b = np.asarray(a), np.asarray(b)
    return (float(np.linalg.norm(a[:3, 3] - b[:3, 3])),
            float(Rotation.from_matrix(a[:3, :3] @ b[:3, :3].T).magnitude()))


def rigid_fit(source, target) -> np.ndarray:
    """Least-squares proper rigid transform, without a scale degree of freedom."""
    source, target = np.asarray(source), np.asarray(target)
    if len(source) < 3 or source.shape != target.shape:
        raise ValueError("rigid fit needs at least three associated XYZ points")
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - a).T @ (target - b))
    sign = np.eye(3)
    sign[2, 2] = np.linalg.det(vt.T @ u.T)
    r = vt.T @ sign @ u.T
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = r, b - r @ a
    return out


def bounded_sample(points, maximum: int) -> np.ndarray:
    p = np.asarray(points, dtype=np.float64)
    if len(p) <= maximum:
        return p.copy()
    # Spatially indifferent, deterministic budget; no full-cloud fingerprint.
    return p[np.linspace(0, len(p) - 1, maximum, dtype=np.int64)].copy()


def voxel_sample(points, size: float) -> np.ndarray:
    """Sort-based voxel sampling, performed only while building/consuming maps."""
    p = np.asarray(points, dtype=np.float64)
    if size <= 0 or not len(p):
        return p.copy()
    cell = np.floor(p / size).astype(np.int64)
    order = np.lexsort((cell[:, 2], cell[:, 1], cell[:, 0]))
    sorted_cell = cell[order]
    keep = np.r_[True, np.any(sorted_cell[1:] != sorted_cell[:-1], axis=1)]
    return p[order[keep]].copy()
