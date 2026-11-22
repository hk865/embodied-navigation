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
    hessian: np.ndarray | None = None


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
    return_hessian: bool = False,
    use_cpp: bool = True,
) -> IcpResult:
    """Refine ``T_init`` (an estimate of T_target_source).

    When ``source_up_body`` (unit gravity-up direction in the source frame) and
    ``gravity_sigma_rad`` are given, each Gauss-Newton step adds a soft 3D
    world-Z Roll/Pitch prior ``r_g = (R @ g_b) - e_z -> 0`` with exact MAP
    weight ``(gravity_ref_point_sigma_m / gravity_sigma_rad)^2`` while leaving
    world Yaw and translation unconstrained.

    Linearization is centered at the target centroid so the normal equations
    and conditioning check are invariant to arbitrary world-origin offsets.

    Returns ``converged=False`` (never raises) when either cloud has fewer than
    50 points after downsampling, too few correspondences remain, or the
    normal equations are singular.
    """
    if use_cpp:
        from .accel import cpp_icp_point_to_plane

        fast = cpp_icp_point_to_plane(
            source,
            target,
            T_init,
            max_correspondence_m=max_correspondence_m,
            iterations=iterations,
            voxel_m=voxel_m,
            source_up_body=source_up_body,
            gravity_sigma_rad=gravity_sigma_rad,
            gravity_ref_point_sigma_m=gravity_ref_point_sigma_m,
            return_hessian=return_hessian,
        )
        if fast is not None:
            T_out, fitness, rmse_m, converged, hess = fast
            return IcpResult(T_out, fitness, rmse_m, converged, hessian=hess)
    source = voxel_downsample(source, voxel_m, use_cpp=False)
    target = voxel_downsample(target, voxel_m, use_cpp=False)
    T = np.array(T_init, dtype=np.float64)
    if len(source) < MIN_POINTS or len(target) < MIN_POINTS:
        empty_h = np.zeros((6, 6), dtype=np.float64) if return_hessian else None
        return IcpResult(T, 0.0, float("inf"), False, hessian=empty_h)
    up_body = None
    if source_up_body is not None and gravity_sigma_rad is not None:
        vec = np.asarray(source_up_body, dtype=np.float64).reshape(3)
        norm = float(np.linalg.norm(vec))
        if norm > 1e-6:
            up_body = vec / norm
    center = target.mean(axis=0)
    target_c = target - center
    T_c = T.copy()
    T_c[:3, 3] -= center
    tree = cKDTree(target_c)
    normals = _normals(target_c, tree)
    last_geom_h = np.zeros((6, 6), dtype=np.float64)
    for _ in range(iterations):
        moved = transform_points(T_c, source)
        points, residual, plane = _correspondences(moved, target_c, normals, tree, max_correspondence_m)
        if len(points) < MIN_POINTS:
            T_out = T_c.copy()
            T_out[:3, 3] += center
            return IcpResult(
                T_out,
                0.0,
                float("inf"),
                False,
                hessian=(last_geom_h.copy() if return_hessian else None),
            )
        jacobian = np.hstack([plane, np.cross(points, plane)])
        geom_h = jacobian.T @ jacobian
        last_geom_h = geom_h
        hessian = geom_h
        rhs = jacobian.T @ residual
        if up_body is not None and gravity_sigma_rad is not None:
            u = T_c[:3, :3] @ up_body
            # Left perturbation: u(exp(delta) T) ≈ u + phi x u = u - [u]_x phi.
            J_g = np.zeros((3, 6), dtype=np.float64)
            J_g[:, 3:] = -hat(u)
            r_g = np.array([u[0], u[1], u[2] - 1.0], dtype=np.float64)
            w_g = (gravity_ref_point_sigma_m / gravity_sigma_rad) ** 2
            hessian = hessian + w_g * (J_g.T @ J_g)
            rhs = rhs + w_g * (J_g.T @ r_g)
        scale = max(float(np.sqrt(np.mean(np.sum(points * points, axis=1)))), 1.0)
        inv_scale = np.array([1.0, 1.0, 1.0, 1.0 / scale, 1.0 / scale, 1.0 / scale], dtype=np.float64)
        scaled_h = hessian * np.outer(inv_scale, inv_scale)
        eigen = np.linalg.eigvalsh(scaled_h)
        if eigen[0] <= 1e-9 * eigen[-1]:
            T_out = T_c.copy()
            T_out[:3, 3] += center
            return IcpResult(
                T_out,
                0.0,
                float("inf"),
                False,
                hessian=(last_geom_h.copy() if return_hessian else None),
            )
        step = np.linalg.solve(hessian, -rhs)
        T_c = se3_exp(step) @ T_c
        if np.linalg.norm(step[:3]) < 1e-3 and np.linalg.norm(step[3:]) < 1e-4:
            res = _score(
                T_c,
                source,
                target_c,
                normals,
                tree,
                max_correspondence_m,
                True,
                return_hessian=return_hessian,
                fallback_hessian=last_geom_h,
            )
            T_out = res.T_target_source.copy()
            T_out[:3, 3] += center
            return IcpResult(T_out, res.fitness, res.rmse_m, res.converged, hessian=res.hessian)
    res = _score(
        T_c,
        source,
        target_c,
        normals,
        tree,
        max_correspondence_m,
        False,
        return_hessian=return_hessian,
        fallback_hessian=last_geom_h,
    )
    T_out = res.T_target_source.copy()
    T_out[:3, 3] += center
    return IcpResult(T_out, res.fitness, res.rmse_m, res.converged, hessian=res.hessian)


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


def _score(
    T,
    source,
    target,
    normals,
    tree,
    max_distance,
    converged,
    *,
    return_hessian: bool = False,
    fallback_hessian: np.ndarray | None = None,
) -> IcpResult:
    points, residual, plane = _correspondences(transform_points(T, source), target, normals, tree, max_distance)
    fitness = len(points) / len(source)
    rmse = float(np.sqrt(np.mean(residual**2))) if len(points) else float("inf")
    hess = None
    if return_hessian:
        if len(points) > 0:
            jacobian = np.hstack([plane, np.cross(points, plane)])
            hess = jacobian.T @ jacobian
        elif fallback_hessian is not None:
            hess = fallback_hessian.copy()
        else:
            hess = np.zeros((6, 6), dtype=np.float64)
    return IcpResult(T, fitness, rmse, converged, hessian=hess)
