"""Point-to-plane ICP between two clouds.

``source`` is in frame S, ``target`` in frame T; the result ``T_target_source``
maps source points into the target frame. Target normals come from a PCA of
the k nearest target neighbours. Each Gauss-Newton step linearises
``n . (exp(delta) T p - q)`` with ``delta = (rho, phi)`` applied on the left
(in the target frame).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from .keyframes import transform_points, voxel_downsample
from .pose_graph import hat, se3_exp

MIN_POINTS = 50
NORMAL_NEIGHBORS = 10


@dataclass(frozen=True)
class IcpResult:
    T_target_source: np.ndarray
    fitness: float  # inlier fraction of the downsampled source
    rmse_m: float  # point-to-plane RMS over inliers
    converged: bool


def icp_point_to_plane(
    source: np.ndarray,
    target: np.ndarray,
    T_init: np.ndarray,
    *,
    max_correspondence_m: float = 1.0,
    iterations: int = 30,
    voxel_m: float = 0.2,
    source_up_body: np.ndarray | None = None,
    gravity_sigma_rad: float | None = None,
    gravity_ref_point_sigma_m: float = 0.05,
) -> IcpResult:
    """Refine ``T_init`` (an estimate of T_target_source).

    When ``source_up_body`` (unit gravity-up direction in the source frame) and
    ``gravity_sigma_rad`` are given, each Gauss-Newton step adds a soft world-Z
    Roll/Pitch prior ``u_xy = (R @ g_b)[:2] -> 0`` while leaving world Yaw and
    translation unconstrained.

    Returns ``converged=False`` (never raises) when either cloud has fewer than
    50 points after downsampling, too few correspondences remain, or the
    normal equations are singular.
    """
    source = voxel_downsample(source, voxel_m)
    target = voxel_downsample(target, voxel_m)
    T = np.array(T_init, dtype=np.float64)
    if len(source) < MIN_POINTS or len(target) < MIN_POINTS:
        return IcpResult(T, 0.0, float("inf"), False)
    up_body = None
    if source_up_body is not None and gravity_sigma_rad is not None:
        vec = np.asarray(source_up_body, dtype=np.float64).reshape(3)
        norm = float(np.linalg.norm(vec))
        if norm > 1e-6:
            up_body = vec / norm
    tree = cKDTree(target)
    normals = _normals(target, tree)
    for _ in range(iterations):
        moved = transform_points(T, source)
        points, residual, plane = _correspondences(moved, target, normals, tree, max_correspondence_m)
        if len(points) < MIN_POINTS:
            return IcpResult(T, 0.0, float("inf"), False)
        jacobian = np.hstack([plane, np.cross(points, plane)])
        hessian = jacobian.T @ jacobian
        rhs = jacobian.T @ residual
        if up_body is not None and gravity_sigma_rad is not None:
            u = T[:3, :3] @ up_body
            # Left perturbation: u(exp(delta) T) ≈ u + phi x u = u - [u]_x phi.
            J_g = np.zeros((3, 6), dtype=np.float64)
            J_g[:, 3:] = -hat(u)
            r_g = np.array([u[0], u[1], u[2] - 1.0], dtype=np.float64)
            w_g = float(len(points)) * (gravity_ref_point_sigma_m / gravity_sigma_rad) ** 2
            hessian = hessian + w_g * (J_g.T @ J_g)
            rhs = rhs + w_g * (J_g.T @ r_g)
        eigen = np.linalg.eigvalsh(hessian)
        if eigen[0] <= 1e-9 * eigen[-1]:
            return IcpResult(T, 0.0, float("inf"), False)
        step = np.linalg.solve(hessian, -rhs)
        T = se3_exp(step) @ T
        if np.linalg.norm(step[:3]) < 1e-3 and np.linalg.norm(step[3:]) < 1e-4:
            return _score(T, source, target, normals, tree, max_correspondence_m, True)
    return _score(T, source, target, normals, tree, max_correspondence_m, False)


def _normals(points: np.ndarray, tree: cKDTree) -> np.ndarray:
    """Unit normals: smallest-eigenvalue direction of each k-neighbourhood."""
    _, neighbors = tree.query(points, k=NORMAL_NEIGHBORS)
    groups = points[neighbors]
    centered = groups - groups.mean(axis=1, keepdims=True)
    _, vectors = np.linalg.eigh(np.einsum("nki,nkj->nij", centered, centered))
    return vectors[:, :, 0]


def _correspondences(moved, target, normals, tree, max_distance):
    """Inlier source points, their signed plane residuals, and target normals."""
    distance, nearest = tree.query(moved, distance_upper_bound=max_distance)
    inlier = np.isfinite(distance)
    points, nearest = moved[inlier], nearest[inlier]
    plane = normals[nearest]
    residual = np.einsum("ij,ij->i", points - target[nearest], plane)
    return points, residual, plane


def _score(T, source, target, normals, tree, max_distance, converged) -> IcpResult:
    points, residual, _ = _correspondences(transform_points(T, source), target, normals, tree, max_distance)
    fitness = len(points) / len(source)
    rmse = float(np.sqrt(np.mean(residual**2))) if len(points) else float("inf")
    return IcpResult(T, fitness, rmse, converged)
