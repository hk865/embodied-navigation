"""Causal horizontal floor-plane detection and multi-floor slab tracking.

Detects flat ground planes directly from body-frame LiDAR returns within a
near-field annulus and maintains a causal list of floor slabs anchored to their
first confirmed keyframes. Slow odometry Z drift on a flat floor (low vertical
slope |dz|/dxy) never splits a slab; a new slab is created (or an existing one
rejoined) only after sustained steep stair climbing (accumulated across any
intermediate mid-stair landings) followed by confirmed flat-floor travel.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class FloorPlaneMeasurement:
    """Horizontal ground plane measured in the body frame."""

    height_below_body_m: float
    normal_body: np.ndarray
    residual_mad_m: float
    point_count: int


@dataclass
class FloorSlab:
    """Persistent horizontal floor slab anchored to its first confirmed keyframe."""

    slab_id: int
    anchor_key: Any
    anchor_floor_offset_m: float
    count: int = 1


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
    max_body_tilt_deg: float = 6.0,
    min_radius_m: float = 0.5,
    max_radius_m: float = 4.5,
    min_z_body_m: float = -0.75,
    max_z_body_m: float = -0.08,
    min_points: int = 40,
    min_normal_z: float = 0.985,
    max_mad_m: float = 0.025,
) -> FloorPlaneMeasurement | None:
    """Detect a flat horizontal ground plane under the robot in body coordinates."""
    R = np.asarray(R_odom_body, dtype=np.float64)
    if R.shape == (4, 4):
        R = R[:3, :3]
    if R.shape != (3, 3) or not np.isfinite(R[2, 2]):
        return None
    if float(R[2, 2]) < math.cos(math.radians(max_body_tilt_deg)):
        return None

    pts = np.asarray(points_body, dtype=np.float64).reshape(-1, 3)
    if len(pts) < min_points:
        return None
    finite = np.all(np.isfinite(pts), axis=1)
    r_xy = np.hypot(pts[:, 0], pts[:, 1])
    mask = (
        finite
        & (r_xy >= min_radius_m)
        & (r_xy <= max_radius_m)
        & (pts[:, 2] >= min_z_body_m)
        & (pts[:, 2] <= max_z_body_m)
    )
    band = pts[mask]
    if len(band) < min_points:
        return None

    z_p40 = float(np.percentile(band[:, 2], 40.0))
    seed_pts = band[np.abs(band[:, 2] - z_p40) <= 0.12]
    if len(seed_pts) < min(10, min_points):
        return None
    fit1 = _fit_plane_pca(seed_pts)
    if fit1 is None:
        return None
    n1, d1 = fit1

    inliers = band[np.abs(band @ n1 + d1) <= 0.04]
    if len(inliers) < min_points:
        return None
    fit2 = _fit_plane_pca(inliers)
    if fit2 is None:
        return None
    normal, d_plane = fit2

    inliers = band[np.abs(band @ normal + d_plane) <= 0.04]
    if len(inliers) < min_points:
        return None
    if float(normal[2]) < min_normal_z:
        return None

    signed_residuals = inliers @ normal + d_plane
    mad_m = float(np.median(np.abs(signed_residuals)))
    if not np.isfinite(mad_m) or mad_m > max_mad_m:
        return None

    height_below_body_m = float(d_plane)
    if not np.isfinite(height_below_body_m) or height_below_body_m <= 0.0:
        return None

    return FloorPlaneMeasurement(
        height_below_body_m=height_below_body_m,
        normal_body=normal,
        residual_mad_m=mad_m,
        point_count=int(len(inliers)),
    )


class FloorSlabTracker:
    """Causally tracks horizontal floor slabs using slope-integrated stair climb detection.

    Key physical invariant:
    - Slow odometry Z drift on a flat floor has tiny local vertical slope (|dz|/dxy < 0.08).
    - Climbing or descending stairs has steep local slope (|dz|/dxy >= 0.16, stairs ~0.55)
      and/or non-level body frames, accumulating |climb_accum_z| >= stair_climb_min_m
      across both flights without falsely resetting on a small mid-stair landing.
    """

    def __init__(
        self,
        *,
        stair_climb_min_m: float = 1.2,
        slab_rejoin_tol_m: float = 1.2,
        level_confirm_frames: int = 3,
        steep_slope_min: float = 0.16,
        landing_reset_travel_m: float = 3.5,
    ):
        self.stair_climb_min_m = float(stair_climb_min_m)
        self.slab_rejoin_tol_m = float(slab_rejoin_tol_m)
        self.level_confirm_frames = int(level_confirm_frames)
        self.steep_slope_min = float(steep_slope_min)
        self.landing_reset_travel_m = float(landing_reset_travel_m)
        self.slabs: dict[int, FloorSlab] = {}
        self.active_slab_id: int | None = None
        self._fallback_anchor_pose_z: dict[int, float] = {}

        self._prev_odom_xyz: np.ndarray | None = None
        self.climb_accum_z: float = 0.0
        self.in_stair_transition: bool = False
        self.level_streak: int = 0
        self.level_travel_since_steep_m: float = 0.0

    def slab_elevation(
        self,
        slab_id: int,
        pose_z_lookup: Callable[[Any], float] | None = None,
    ) -> float:
        """Current world-Z floor elevation of ``slab_id`` from its anchor keyframe."""
        slab = self.slabs[slab_id]
        anchor_z = (
            float(pose_z_lookup(slab.anchor_key))
            if pose_z_lookup is not None
            else self._fallback_anchor_pose_z[slab_id]
        )
        return anchor_z - slab.anchor_floor_offset_m

    def _create_slab(self, key: Any, height_below_body_m: float, pose_z: float) -> FloorSlab:
        slab_id = len(self.slabs)
        slab = FloorSlab(
            slab_id=slab_id,
            anchor_key=key,
            anchor_floor_offset_m=float(height_below_body_m),
            count=1,
        )
        self.slabs[slab_id] = slab
        self.active_slab_id = slab_id
        self._fallback_anchor_pose_z[slab_id] = float(pose_z)
        return slab

    def update(
        self,
        key: Any,
        meas: FloorPlaneMeasurement | None,
        *,
        odom_z: float,
        pose_z: float,
        odom_xy: tuple[float, float] | None = None,
        zupt_active: bool = False,
        pose_z_lookup: Callable[[Any], float] | None = None,
    ) -> tuple[int, float] | None:
        """Update slab state for ``key`` and return ``(slab_id, target_body_z)`` when constrained."""
        ox = 0.0 if odom_xy is None else float(odom_xy[0])
        oy = 0.0 if odom_xy is None else float(odom_xy[1])
        oz = float(odom_z)
        pz = float(pose_z)
        cur_odom = np.array([ox, oy, oz], dtype=np.float64)

        dxy = 0.0
        dz = 0.0
        is_steep = False
        if self._prev_odom_xyz is not None:
            dz = oz - float(self._prev_odom_xyz[2])
            if odom_xy is not None:
                dxy = float(math.hypot(ox - self._prev_odom_xyz[0], oy - self._prev_odom_xyz[1]))
                slope = abs(dz) / max(dxy, 0.25)
                if (slope >= self.steep_slope_min and abs(dz) >= 0.04) or (meas is None and abs(dz) >= 0.04):
                    is_steep = True
            else:
                if abs(dz) >= 0.12 or (meas is None and abs(dz) >= 0.04):
                    is_steep = True
        self._prev_odom_xyz = cur_odom

        if is_steep or meas is None:
            if is_steep:
                self.climb_accum_z += dz
                self.level_travel_since_steep_m = 0.0
                if abs(self.climb_accum_z) >= 0.35:
                    self.in_stair_transition = True
            self.level_streak = 0
            return None

        # Valid horizontal flat floor observed
        self.level_streak += 1
        self.level_travel_since_steep_m += dxy

        if self.active_slab_id is None:
            self._create_slab(key, meas.height_below_body_m, pz)
            self.climb_accum_z = 0.0
            self.in_stair_transition = False
            return None

        if self.in_stair_transition:
            # While climbing/on a mid-stair landing, NEVER emit z_priors tied to the old floor!
            if abs(self.climb_accum_z) >= self.stair_climb_min_m and self.level_streak >= self.level_confirm_frames:
                est_floor_z = pz - meas.height_below_body_m
                best_slab_id: int | None = None
                best_diff = self.slab_rejoin_tol_m
                for sid in self.slabs:
                    diff = abs(est_floor_z - self.slab_elevation(sid, pose_z_lookup))
                    if diff <= best_diff:
                        best_diff = diff
                        best_slab_id = sid
                self.climb_accum_z = 0.0
                self.in_stair_transition = False
                self.level_travel_since_steep_m = 0.0
                if best_slab_id is not None:
                    self.active_slab_id = best_slab_id
                else:
                    self._create_slab(key, meas.height_below_body_m, pz)
                    return None
            elif (
                abs(self.climb_accum_z) < 0.8
                and self.level_travel_since_steep_m >= self.landing_reset_travel_m
                and self.level_streak >= self.level_confirm_frames
            ):
                # Minor bump / threshold, not a real staircase; resume active slab cleanly
                self.climb_accum_z = 0.0
                self.in_stair_transition = False
            else:
                # Still on stairs or mid-landing: wait before constraining Z
                return None

        if zupt_active:
            return None

        slab = self.slabs[self.active_slab_id]
        if key == slab.anchor_key:
            return None
        slab.count += 1
        current_slab_z = self.slab_elevation(slab.slab_id, pose_z_lookup)
        target_body_z = current_slab_z + meas.height_below_body_m
        return slab.slab_id, float(target_body_z)
