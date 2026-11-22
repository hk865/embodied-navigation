"""Wide-span horizontal floor-plane detection and pairwise coplanar odometry leveling.

Key physical insight:
- Quadrupeds (e.g. Unitree Go2) walk with ~6-8 deg natural body pitch, so slicing
  raw ``z_body`` chops tilted floor clouds and biases normals by 3-4 deg.
- Rotating ``points_body`` by the coarse odometry attitude ``R_odom_body`` ONLY
  to select the dominant horizontal floor slab via a 4 cm Z-histogram mode, then
  fitting the plane directly in ``body`` coordinates yields ~0.04-0.13 deg
  floor-normal accuracy (5-8x more accurate than noisy IMU attitude) and ~3 mm
  step-to-step height precision.
- Between two consecutive flat-floor keyframes ``(a, b)`` on the same level slab,
  the true displacement along ``n_a`` equals ``h_b - h_a``; projecting out the
  excess component along ``n_a`` eliminates open-loop vertical Z drift across
  arbitrarily many floors without fragile floor-slab ID heuristics.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class FloorPlaneMeasurement:
    """Horizontal ground plane measured in the body frame (``n . p + height_below_body_m = 0``)."""

    height_below_body_m: float
    normal_body: np.ndarray
    residual_mad_m: float
    point_count: int
    span_min_m: float
    span_max_m: float
    inlier_sample_body: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float64))


def _fit_plane_pca(points: np.ndarray) -> tuple[np.ndarray, float] | None:
    """Fit ``n . p + d = 0`` by PCA with ``n[2] > 0``; return ``(n, d)`` or None."""
    if len(points) < 3:
        return None
    mean = points.mean(axis=0)
    centered = points - mean
    eigvals, eigvecs = np.linalg.eigh(centered.T @ centered)
    if not np.isfinite(eigvals).all() or eigvals[1] <= 1e-8:
        return None
    normal = eigvecs[:, 0].copy()
    if normal[2] < 0.0:
        normal = -normal
    norm = float(np.linalg.norm(normal))
    if norm <= 1e-9 or not np.isfinite(norm):
        return None
    normal /= norm
    d_plane = -float(normal @ mean)
    return normal, d_plane


def detect_floor_plane(
    points_body: np.ndarray,
    R_odom_body: np.ndarray,
    *,
    max_body_tilt_deg: float = 14.0,
    max_plane_world_tilt_deg: float = 6.0,
    min_radius_m: float = 1.0,
    max_radius_m: float = 15.0,
    min_z_level_m: float = -0.70,
    max_z_level_m: float = -0.15,
    min_points: int = 80,
    max_mad_m: float = 0.015,
    min_span_short_m: float = 1.8,
    min_span_long_m: float = 3.5,
) -> FloorPlaneMeasurement | None:
    """Detect a wide, flat horizontal floor slab under the robot in body coordinates.

    Uses ``R_odom_body`` only to select candidate ground points in an approximate
    gravity-levelled frame (tolerant of Go2's natural ~6-8 deg pitch), then fits
    the exact plane directly in ``body`` coordinates so the resulting normal and
    height are independent of IMU/odometry tilt bias.
    """
    R = np.asarray(R_odom_body, dtype=np.float64)
    if R.shape == (4, 4):
        R = R[:3, :3]
    if R.shape != (3, 3) or not np.isfinite(R).all():
        return None
    if float(R[2, 2]) < math.cos(math.radians(max_body_tilt_deg)):
        return None

    pts = np.asarray(points_body, dtype=np.float64).reshape(-1, 3)
    if len(pts) < min_points:
        return None
    finite = np.all(np.isfinite(pts), axis=1)
    if not np.all(finite):
        pts = pts[finite]
        if len(pts) < min_points:
            return None

    p_lvl = pts @ R.T
    r_xy = np.hypot(p_lvl[:, 0], p_lvl[:, 1])
    idx = np.flatnonzero(
        (r_xy >= min_radius_m)
        & (r_xy <= max_radius_m)
        & (p_lvl[:, 2] >= min_z_level_m)
        & (p_lvl[:, 2] <= max_z_level_m)
    )
    if len(idx) < min_points:
        return None

    z_lvl = p_lvl[idx, 2]
    bins = np.floor(z_lvl / 0.04).astype(np.int64)
    uniq, counts = np.unique(bins, return_counts=True)
    z_mode = (float(uniq[int(np.argmax(counts))]) + 0.5) * 0.04
    seed_mask = np.abs(z_lvl - z_mode) <= 0.08
    if int(np.sum(seed_mask)) < max(30, min_points // 2):
        return None

    band_b = pts[idx]
    fit1 = _fit_plane_pca(band_b[seed_mask])
    if fit1 is None:
        return None
    n1, d1 = fit1

    inlier_mask = np.abs(band_b @ n1 + d1) <= 0.025
    inliers_b = band_b[inlier_mask]
    if len(inliers_b) < min_points:
        return None
    fit2 = _fit_plane_pca(inliers_b)
    if fit2 is None:
        return None
    normal, d_plane = fit2

    inlier_mask = np.abs(band_b @ normal + d_plane) <= 0.025
    inliers_b = band_b[inlier_mask]
    if len(inliers_b) < min_points:
        return None

    n_lvl = R @ normal
    if float(n_lvl[2]) < math.cos(math.radians(max_plane_world_tilt_deg)):
        return None

    signed_residuals = inliers_b @ normal + d_plane
    mad_m = float(np.median(np.abs(signed_residuals)))
    if not np.isfinite(mad_m) or mad_m > max_mad_m:
        return None

    # Rotation-invariant 2D PCA principal spans on the fitted floor plane.
    centered = inliers_b - inliers_b.mean(axis=0)
    ref = (
        np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(float(normal[0])) < 0.9
        else np.array([0.0, 1.0, 0.0], dtype=np.float64)
    )
    u_axis = np.cross(normal, ref)
    u_axis /= float(np.linalg.norm(u_axis))
    v_axis = np.cross(normal, u_axis)
    coords_2d = np.column_stack([centered @ u_axis, centered @ v_axis])
    _, eigvecs_2d = np.linalg.eigh(coords_2d.T @ coords_2d)
    pc_coords = coords_2d @ eigvecs_2d
    span_short = float(pc_coords[:, 0].max() - pc_coords[:, 0].min())
    span_long = float(pc_coords[:, 1].max() - pc_coords[:, 1].min())
    if span_short < min_span_short_m or span_long < min_span_long_m:
        return None

    height_below_body_m = float(d_plane)
    if not np.isfinite(height_below_body_m) or height_below_body_m <= 0.0:
        return None

    n_inliers = len(inliers_b)
    if n_inliers <= 48:
        inlier_sample = inliers_b.copy()
    else:
        sample_idx = np.linspace(0, n_inliers - 1, 48, dtype=np.intp)
        inlier_sample = inliers_b[sample_idx].copy()

    return FloorPlaneMeasurement(
        height_below_body_m=height_below_body_m,
        normal_body=normal,
        residual_mad_m=mad_m,
        point_count=int(n_inliers),
        span_min_m=span_short,
        span_max_m=span_long,
        inlier_sample_body=inlier_sample,
    )


def project_coplanar_odom_delta(
    delta_odom: np.ndarray,
    prev_floor: FloorPlaneMeasurement,
    cur_floor: FloorPlaneMeasurement,
    *,
    max_step_dz_m: float = 0.06,
    max_normal_disagreement_deg: float = 3.0,
) -> tuple[np.ndarray, bool]:
    """Level a consecutive flat-floor odometry edge using measured floor geometry.

    When both ``prev_floor`` and ``cur_floor`` observe the same horizontal floor
    slab, the true component of ``delta_odom[:3, 3]`` along ``prev_floor.normal_body``
    equals ``cur_floor.height_below_body_m - prev_floor.height_below_body_m``.
    Rejects steps or disconnected slabs when ``|dz_odom - dz_floor| > max_step_dz_m``
    or the direct transformed footprint plane distance exceeds 0.04 m.

    Returns ``(delta_out, applied)``; never mutates ``delta_odom``.
    """
    R = np.array(delta_odom[:3, :3], dtype=np.float64, copy=True)
    t = np.array(delta_odom[:3, 3], dtype=np.float64, copy=True)
    n_a = prev_floor.normal_body
    n_b = cur_floor.normal_body

    # Verify the two frames truly share the same horizontal slab (rejects stairs/steps)
    dz_odom = float(n_a @ t)
    dz_floor = float(cur_floor.height_below_body_m - prev_floor.height_below_body_m)
    if abs(dz_odom - dz_floor) > max_step_dz_m:
        return delta_odom, False

    # Verify relative rotation maps n_b close to n_a (rejects stair/ramp transitions)
    n_b_in_a = R @ n_b
    cos_ang = float(np.clip(n_a @ n_b_in_a, -1.0, 1.0))
    if cos_ang < math.cos(math.radians(max_normal_disagreement_deg)):
        return delta_odom, False

    # Verify direct surface overlap of cur_floor footprint against prev_floor plane
    cur_sample = np.asarray(cur_floor.inlier_sample_body, dtype=np.float64).reshape(-1, 3)
    if len(cur_sample) == 0:
        return delta_odom, False
    pts_in_prev = cur_sample @ R.T + t
    med_plane_dist = float(np.median(np.abs(pts_in_prev @ n_a + prev_floor.height_below_body_m)))
    if not np.isfinite(med_plane_dist) or med_plane_dist > 0.04:
        return delta_odom, False

    # Project out vertical drift along n_a ONLY; keep R untouched so yaw is never perturbed!
    t_clean = t + (dz_floor - dz_odom) * n_a

    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = R
    out[:3, 3] = t_clean
    return out, True
