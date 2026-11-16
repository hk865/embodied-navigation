"""Measured local surfaces in a gravity-aligned frame, for loop verification.

The caller supplies gravity alignment: body z is not a ground normal. Cells
with sparse, curved, edge-mixed or sloping returns remain unknown. Horizontal
patches can be treads, platforms or other horizontal objects; this module does
not infer walkability, semantic floor identity, or unobserved stair geometry.
"""
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class SurfaceConfig:
    cell_m: float = 0.3
    min_points: int = 6
    max_plane_rmse_m: float = 0.025
    min_spread_m: float = 0.025
    normal_tolerance_rad: float = 0.2
    merge_distance_m: float = 0.04
    correspondence_m: float = 0.3
    max_residual_m: float = 0.06
    min_coverage: float = 0.6
    min_patches: int = 1


@dataclass(frozen=True)
class SurfacePatch:
    kind: str
    centroid: np.ndarray
    normal: np.ndarray
    points: np.ndarray
    source_keys: tuple
    height_m: float | None

    @property
    def support_count(self):
        return len(self.points)


@dataclass(frozen=True)
class SurfaceModel:
    patches: tuple
    unknown_points: int
    observed_points: int


@dataclass(frozen=True)
class SurfaceComparison:
    accepted: bool
    reason: str
    matched_patches: int
    compared_patches: int
    rmse_m: float
    coverage_by_kind: dict = field(default_factory=dict)


def model_surfaces(points_world, source_keys=None, config=None):
    """Fit and connect observed planar cells; never fill holes between returns.

    ``source_keys`` optionally gives one hashable provenance key per point.
    Finite Nx3 validation protects external point-cloud/offline-file input.
    """
    c = config or SurfaceConfig()
    points = np.asarray(points_world, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('surface points must be finite Nx3')
    if source_keys is not None and len(source_keys) != len(points):
        raise ValueError('surface source keys must align with points')
    cells = {}
    for i, cell in enumerate(np.floor(points / c.cell_m).astype(np.int64)):
        cells.setdefault(tuple(cell), []).append(i)
    planar = {}
    for cell, indices in cells.items():
        group = points[indices]
        if len(group) < c.min_points:
            continue
        center = group.mean(axis=0)
        centered = group - center
        eigen, vectors = np.linalg.eigh(centered.T @ centered / len(group))
        if eigen[1] < c.min_spread_m**2 or eigen[0] > c.max_plane_rmse_m**2:
            continue
        normal = vectors[:, 0]
        z = abs(normal[2])
        kind = ('horizontal' if z >= np.cos(c.normal_tolerance_rad) else
                'vertical' if z <= np.sin(c.normal_tolerance_rad) else None)
        if kind is not None:
            if normal[np.argmax(abs(normal))] < 0:
                normal = -normal
            planar[cell] = (indices, center, normal, kind)
    patches = []
    remaining = set(planar)
    while remaining:
        first = min(remaining)
        remaining.remove(first)
        indices, center, normal, kind = planar[first]
        members = list(indices)
        frontier = [first]
        while frontier:
            cell = frontier.pop()
            for axis in range(3):
                for direction in (-1, 1):
                    neighbor = list(cell)
                    neighbor[axis] += direction
                    neighbor = tuple(neighbor)
                    if neighbor not in remaining:
                        continue
                    other_indices, other_center, other_normal, other_kind = planar[neighbor]
                    if (other_kind == kind and abs(normal @ other_normal) >= np.cos(c.normal_tolerance_rad)
                            and abs((other_center - center) @ normal) <= c.merge_distance_m):
                        remaining.remove(neighbor)
                        frontier.append(neighbor)
                        members.extend(other_indices)
        observed = points[members].copy()
        centroid = observed.mean(axis=0)
        keys = () if source_keys is None else tuple(dict.fromkeys(source_keys[i] for i in members))
        patches.append(SurfacePatch(kind, centroid, normal, observed, keys,
                                    float(centroid[2]) if kind == 'horizontal' else None))
    support = sum(p.support_count for p in patches)
    return SurfaceModel(tuple(patches), len(points) - support, len(points))


def compare_surfaces(query, target, config=None):
    """Check measured overlap, including each observed surface orientation.

    The result is a gate only, not an independent information matrix. Nearby
    repeated treads can remain ambiguous; visual/ICP and motion gates are still
    required. Missing observations are rejected rather than counted as agreement.
    """
    c = config or SurfaceConfig()
    if len(query.patches) < c.min_patches or len(target.patches) < c.min_patches:
        return SurfaceComparison(False, 'insufficient_surface_observations', 0, len(query.patches), float('inf'))
    matched, errors, coverage = 0, [], {}
    for kind in ('horizontal', 'vertical'):
        query_patches = [p for p in query.patches if p.kind == kind]
        if not query_patches:
            continue
        target_patches = [p for p in target.patches if p.kind == kind]
        total = sum(p.support_count for p in query_patches)
        accepted = 0
        if target_patches:
            target_points = np.vstack([p.points for p in target_patches])
            normals = np.vstack([np.tile(p.normal, (p.support_count, 1)) for p in target_patches])
            tree = cKDTree(target_points)
            for patch in query_patches:
                distance, index = tree.query(patch.points)
                residual = np.abs(np.einsum('ij,ij->i', patch.points-target_points[index], normals[index]))
                consistent = ((distance <= c.correspondence_m) & (residual <= c.max_residual_m)
                              & (np.abs(normals[index] @ patch.normal) >= np.cos(c.normal_tolerance_rad)))
                count = int(consistent.sum())
                accepted += count
                errors.extend(residual[consistent])
                matched += count / patch.support_count >= c.min_coverage
        coverage[kind] = accepted / total
    rmse = float(np.sqrt(np.mean(np.square(errors)))) if errors else float('inf')
    ok = matched >= c.min_patches and all(v >= c.min_coverage for v in coverage.values())
    return SurfaceComparison(ok, 'consistent' if ok else 'surface_mismatch', int(matched), len(query.patches), rmse, coverage)
