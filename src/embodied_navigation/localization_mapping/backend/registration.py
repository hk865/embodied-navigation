"""Bounded, CPU geometric relocalization against trajectory-free map blocks.

This is an asynchronous Python backend implementation, not a claim of frontend
or C++ real-time throughput. Local distance descriptors propose hypotheses;
RANSAC/PCA only propose transforms. Independent holdout geometry, overlap,
point-to-plane observability, and competing poses decide whether to accept.
An unobservable or ambiguous scene returns UNKNOWN rather than a best guess.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from itertools import permutations, product
import time

import numpy as np
from scipy.spatial import cKDTree

from .geometry import (bounded_sample, checked_transform, pose_distance,
                       rigid_fit, transform_points, voxel_sample)


@dataclass(frozen=True)
class RegistrationConfig:
    voxel_size: float = 0.12
    max_scan_points: int = 1800
    max_feature_points: int = 450
    descriptor_neighbors: int = 12
    ransac_iterations: int = 1600
    max_candidates: int = 24
    coarse_distance: float = 0.45
    correspondence_distance: float = 0.22
    min_fitness: float = 0.70
    max_rmse: float = 0.12
    min_inliers: int = 50
    min_observability: float = 0.001
    min_spatial_eigenratio: float = 0.0003
    icp_iterations: int = 35
    ambiguity_fitness_margin: float = 0.04
    ambiguity_rmse_margin: float = 0.035
    distinct_translation: float = 0.5
    distinct_rotation: float = 0.20
    random_seed: int = 17


@dataclass(frozen=True)
class RegistrationResult:
    accepted: bool
    reason: str
    T_map_lidar: np.ndarray | None = None
    fitness: float = 0.0
    rmse: float = float("inf")
    observability: float = 0.0
    elapsed_s: float = 0.0
    candidate_count: int = 0
    diagnostics: dict = field(default_factory=dict)


def _local_distances(points, neighbors):
    """Absolute local distance signature: invariant to translation/rotation.

    Unlike a normalized shape descriptor, it preserves metric scale. It is a
    proposal mechanism, and is deliberately not sufficient for acceptance.
    """
    k = min(neighbors + 1, len(points))
    distances, _ = cKDTree(points).query(points, k=k, workers=1)
    if distances.ndim == 1:
        return np.zeros((len(points), neighbors))
    descriptor = distances[:, 1:]
    if descriptor.shape[1] < neighbors:
        descriptor = np.pad(descriptor, ((0, 0), (0, neighbors - descriptor.shape[1])), mode="edge")
    return descriptor


def _shape(points):
    centered = points - np.median(points, axis=0)
    eigen = np.linalg.eigvalsh(centered.T @ centered / len(points))
    scale = max(float(eigen.sum()), 1e-12)
    radial = np.linalg.norm(centered, axis=1)
    return np.r_[eigen / scale, np.quantile(radial, [0.25, 0.5, 0.75]) / max(np.quantile(radial, 0.9), 1e-8)]


def _pca_hypotheses(source, target):
    a, b = source.mean(axis=0), target.mean(axis=0)
    _, va = np.linalg.eigh(np.cov((source - a).T))
    _, vb = np.linalg.eigh(np.cov((target - b).T))
    for perm in permutations(range(3)):
        for signs in product((-1, 1), repeat=3):
            r = vb[:, perm] @ np.diag(signs) @ va.T
            if np.linalg.det(r) < 0.999:
                continue
            t = np.eye(4)
            t[:3, :3], t[:3, 3] = r, b - r @ a
            yield t


def _score(source, tree, t, distance):
    d, _ = tree.query(transform_points(source, t), k=1, workers=1)
    valid = d < distance
    return float(np.mean(valid)), float(np.sqrt(np.mean(d[valid] ** 2))) if valid.any() else float("inf")


def _coarse_hypotheses(source, target, descriptor_source, descriptor_target, config, seed):
    target_tree = cKDTree(target)
    score_points = bounded_sample(source, 220)
    proposals = []

    def retain(t):
        fit, rmse = _score(score_points, target_tree, t, config.coarse_distance)
        if fit < max(0.35, config.min_fitness - 0.30):
            return
        # Distinct seeds protect against one repetitive scene occupying every slot.
        for i, (_, _, old) in enumerate(proposals):
            d, angle = pose_distance(t, old)
            if d < 0.20 and angle < 0.10:
                if (fit, -rmse) > (proposals[i][0], -proposals[i][1]):
                    proposals[i] = (fit, rmse, t)
                return
        proposals.append((fit, rmse, t))
        proposals.sort(key=lambda v: (-v[0], v[1]))
        del proposals[10:]

    for t in _pca_hypotheses(source, target):
        retain(t)
    ids = np.linspace(0, len(source) - 1, min(len(source), config.max_feature_points), dtype=np.int64)
    sd = descriptor_source[ids]
    distances, matches = cKDTree(descriptor_target).query(sd, k=3, workers=1)
    # Include non-ratio matches as low-confidence proposals; geometry will reject.
    ratio = distances[:, 0] / np.maximum(distances[:, 1], 1e-12)
    ordering = np.argsort(ratio)
    count = max(20, min(len(ids), int(np.sum(ratio < 0.96))))
    ids = ids[ordering[:count]]
    matches = matches[ordering[:count]]
    distances = distances[ordering[:count]]
    if len(ids) < 3:
        return proposals
    rng = np.random.default_rng(seed)
    for _ in range(config.ransac_iterations):
        chosen = rng.choice(len(ids), 3, replace=False)
        alternatives = distances[chosen] <= distances[chosen, :1] * 1.25 + 1e-9
        choices = np.asarray([rng.choice(np.flatnonzero(row)) for row in alternatives])
        a, b = source[ids[chosen]], target[matches[chosen, choices]]
        sa = np.linalg.norm(a[:, None] - a[None, :], axis=2)
        sb = np.linalg.norm(b[:, None] - b[None, :], axis=2)
        if np.max(np.abs(sa - sb)) > config.coarse_distance * 0.5:
            continue
        if np.linalg.norm(np.cross(a[1] - a[0], a[2] - a[0])) < 0.02:
            continue
        retain(rigid_fit(a, b))
    return proposals


def _refine(source, target, initial, config, tree=None):
    tree = cKDTree(target)if tree is None else tree
    t = initial.copy()
    previous = float("inf")
    for _ in range(config.icp_iterations):
        moved = transform_points(source, t)
        distance, ids = tree.query(moved, k=1, workers=1)
        limit = max(config.correspondence_distance, config.coarse_distance * 0.7)
        valid = distance < limit
        if np.sum(valid) < config.min_inliers:
            break
        # Trim the worst fifth; gross clutter never gets a full least-squares vote.
        cut = np.quantile(distance[valid], 0.8)
        valid &= distance <= max(cut, config.correspondence_distance * 0.25)
        if np.sum(valid) < 6:
            break
        delta = rigid_fit(moved[valid], target[ids[valid]])
        t = delta @ t
        error = float(np.mean(distance[valid] ** 2))
        shift, angle = pose_distance(delta, np.eye(4))
        if abs(previous - error) < 1e-10 and shift < 1e-5 and angle < 1e-5:
            break
        previous = error
    return t


def _observability(moved, target, ids, tree=None):
    matched = target[ids]
    eigen = np.linalg.eigvalsh(np.cov(moved.T))
    spatial = float(eigen[0] / max(eigen[-1], 1e-12))
    k = min(15, len(target))
    _, neighborhoods = (cKDTree(target)if tree is None else tree).query(matched, k=k, workers=1)
    groups = target[neighborhoods]
    centered = groups - groups.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered)
    _, vectors = np.linalg.eigh(covariance)
    normals = vectors[:, :, 0]
    # Center and normalize rotational lever arms so the ratio is not an origin
    # distance artifact and rotations/metres do not silently change with map size.
    centered_points = moved - moved.mean(axis=0)
    scale = max(float(np.sqrt(np.mean(np.sum(centered_points ** 2, axis=1)))), 0.1)
    jacobian = np.column_stack((np.cross(centered_points / scale, normals), normals))
    values = np.linalg.eigvalsh(jacobian.T @ jacobian / len(jacobian))
    return float(max(0, values[0]) / max(values[-1], 1e-12)), spatial


class Relocalizer:
    def __init__(self, map, config=None):
        self.map = map
        self.config = config or RegistrationConfig()
        c = self.config
        if (c.max_scan_points < max(80, c.min_inliers * 2) or c.max_candidates < 1
                or c.ransac_iterations < 0 or c.icp_iterations < 1
                or c.max_feature_points < 3 or c.descriptor_neighbors < 3 or c.min_inliers < 20):
            raise ValueError("invalid registration budget")
        if (not 0 < c.min_fitness <= 1 or c.correspondence_distance <= 0 or c.max_rmse <= 0
                or c.coarse_distance < c.correspondence_distance or c.voxel_size < 0
                or c.min_observability <= 0 or c.min_spatial_eigenratio <= 0
                or c.distinct_translation <= 0 or c.distinct_rotation <= 0):
            raise ValueError("invalid geometric acceptance limits")
        # Offline/worker-owned cached descriptors, never frontend work.
        self._blocks = [(b, _local_distances(b.points, c.descriptor_neighbors), _shape(b.points))
                        for b in map.blocks]

    def localize(self, scan_lidar, T_prior=None):
        started = time.monotonic()
        c = self.config

        def result(accepted, reason, t=None, fit=0.0, rmse=float("inf"), observable=0.0, diagnostics=None):
            return RegistrationResult(accepted, reason, t, fit, rmse, observable,
                                      time.monotonic() - started, len(candidates), diagnostics or {})

        candidates = []
        raw = np.asarray(scan_lidar, dtype=np.float64)
        if raw.ndim != 2 or raw.shape[1] != 3 or not np.isfinite(raw).all():
            return result(False, "invalid_scan")
        scan = bounded_sample(voxel_sample(raw, c.voxel_size), c.max_scan_points)
        if len(scan) < max(80, c.min_inliers * 2):
            return result(False, "insufficient_scan_geometry")
        spatial = np.linalg.eigvalsh(np.cov(scan.T))
        if spatial[0] / max(spatial[-1], 1e-12) < c.min_spatial_eigenratio:
            return result(False, "degenerate_scan_geometry")
        # Holdout correspondences do not enter rigid-transform fitting or ICP.
        # Neighbor signatures see the snapshot's sampling density, rather than
        # an artificial 25% thinning. They remain proposals, not acceptance.
        verify = scan[::4]
        fitting = np.arange(len(scan)) % 4 != 0
        fit_points = scan[fitting]
        descriptor = _local_distances(scan, c.descriptor_neighbors)[fitting]
        shape = _shape(fit_points)
        ranked = sorted(self._blocks, key=lambda b: np.linalg.norm(shape - b[2]))
        candidates = ranked[:c.max_candidates]
        prior = None if T_prior is None else checked_transform(T_prior)
        verified = []
        best_failed = None
        tested_hypotheses = 0
        for block, target_descriptor, _ in candidates:
            hypotheses = _coarse_hypotheses(fit_points, block.points, descriptor, target_descriptor,
                                            c, c.random_seed + block.index)
            if prior is not None:
                hypotheses.append((0, 0, prior))
            for _, _, initial in hypotheses:
                tested_hypotheses += 1
                t = _refine(fit_points, block.points, initial, c)
                moved = transform_points(verify, t)
                distance, ids = cKDTree(block.points).query(moved, k=1, workers=1)
                valid = distance < c.correspondence_distance
                fitness = float(np.mean(valid))
                rmse = float(np.sqrt(np.mean(distance[valid] ** 2))) if valid.any() else float("inf")
                inliers = int(np.sum(valid))
                diagnostic = dict(block=block.index, holdout_inliers=inliers, holdout_count=len(verify))
                record = (fitness, rmse, t, 0.0, diagnostic)
                if best_failed is None or (fitness, -rmse) > (best_failed[0], -best_failed[1]):
                    best_failed = record
                if fitness < c.min_fitness or rmse > c.max_rmse or inliers < max(20, c.min_inliers // 4):
                    continue
                observable, spatial_ratio = _observability(moved[valid], block.points, ids[valid])
                diagnostic.update(spatial_eigenratio=spatial_ratio, observability=observable)
                if observable < c.min_observability or spatial_ratio < c.min_spatial_eigenratio:
                    continue
                record = (fitness, rmse, t, observable, diagnostic)
                duplicate = next((i for i, old in enumerate(verified)
                                  if pose_distance(t, old[2])[0] < c.distinct_translation
                                  and pose_distance(t, old[2])[1] < c.distinct_rotation), None)
                if duplicate is None:
                    verified.append(record)
                elif (fitness, -rmse) > (verified[duplicate][0], -verified[duplicate][1]):
                    verified[duplicate] = record
        common = dict(tested_hypotheses=tested_hypotheses, available_blocks=len(self._blocks),
                      searched_blocks=len(candidates), bounded_search=len(candidates) < len(self._blocks),
                      algorithm="local_distance_ransac_pca_trimmed_icp_holdout")
        if not verified:
            if best_failed is None:
                return result(False, "no_geometric_candidate", diagnostics=common)
            return result(False, "geometry_rejected", fit=best_failed[0], rmse=best_failed[1],
                          diagnostics={**common, **best_failed[4]})
        verified.sort(key=lambda v: (-v[0], v[1]))
        best = verified[0]
        common.update(best[4])
        common["verified_distinct_poses"] = len(verified)
        if any(other[0] >= best[0] - c.ambiguity_fitness_margin
               and other[1] <= best[1] + c.ambiguity_rmse_margin for other in verified[1:]):
            return result(False, "ambiguous_geometry", fit=best[0], rmse=best[1],
                          observable=best[3], diagnostics=common)
        # A bounded retrieval pass may fail to discover a repetitive distant
        # place. Do not claim global uniqueness when blocks were left unsearched.
        if len(candidates) < len(self._blocks):
            return result(False, "global_search_budget_exhausted", fit=best[0], rmse=best[1],
                          observable=best[3], diagnostics=common)
        t = best[2].copy()
        t.setflags(write=False)
        return result(True, "geometrically_verified", t, best[0], best[1], best[3], common)


def surface_normals(points, tree, neighbors=12):
    """Worker-startup normal cache for arbitrary XYZ maps, no pose chain needed."""
    points = np.asarray(points)
    normals = np.zeros_like(points)
    planar = np.zeros(len(points), dtype=bool)
    k = min(neighbors, len(points))
    if k < 6:
        return normals, planar
    for start in range(0, len(points), 8192):
        stop = min(start + 8192, len(points))
        _, ids = tree.query(points[start:stop], k=k, workers=1)
        groups = points[ids]
        centered = groups - groups.mean(axis=1, keepdims=True)
        covariance = np.einsum('nki,nkj->nij', centered, centered) / k
        eigen, vectors = np.linalg.eigh(covariance)
        normals[start:stop] = vectors[:, :, 0]
        planar[start:stop] = ((eigen[:, 1] - eigen[:, 0]) / np.maximum(eigen[:, 2], 1e-12) > .12)
    return normals, planar


def refine_planes(source, target, tree, normals, planar, initial, iterations=12,
                  correspondence_distance=.30):
    """Robust point-to-plane SE3 refinement, strictly on the asynchronous worker."""
    from scipy.spatial.transform import Rotation
    t = np.array(initial, copy=True)
    for _ in range(iterations):
        moved = transform_points(source, t)
        distance, ids = tree.query(moved, workers=1)
        valid = (distance < correspondence_distance) & planar[ids]
        if valid.sum() < 60:
            break
        p, q, n = moved[valid], target[ids[valid]], normals[ids[valid]]
        residual = np.einsum('ij,ij->i', p - q, n)
        weights = np.minimum(1., .04 / np.maximum(np.abs(residual), 1e-8))
        center = p.mean(axis=0)
        jacobian = np.column_stack((n, np.cross(p - center, n)))
        weighted = jacobian * np.sqrt(weights[:, None])
        solution, _, rank, _ = np.linalg.lstsq(weighted, -residual * np.sqrt(weights), rcond=1e-5)
        if rank < 6 or not np.isfinite(solution).all():
            break
        translation, rotation = solution[:3], solution[3:]
        scale = min(1., .15 / max(np.linalg.norm(translation), 1e-12),
                    .06 / max(np.linalg.norm(rotation), 1e-12))
        delta = np.eye(4)
        delta[:3, :3] = Rotation.from_rotvec(rotation * scale).as_matrix()
        delta[:3, 3] = translation * scale + center - delta[:3, :3] @ center
        t = delta @ t
        if np.linalg.norm(solution[:3]) < 1e-5 and np.linalg.norm(solution[3:]) < 1e-5:
            break
    return t
