"""Habitat-Sim HM3D closed-loop adapter for L0-L3 hierarchical memory & VLM planners.

Strictly enforces zero ground-truth leakage:
- The agent receives ONLY rendered egocentric RGB, depth-derived 2D occupancy maps,
  non-blacklisted frontier/memory candidates (F*/M*), and odometry poses.
- ``semantic_sensor`` is strictly private to the offline evaluator for constructing
  real 3D object ground truth and computing exact NavMesh geodesic SPL / SR / OSR.
- Observed landmarks reported by the VLM are independently grounded via real Habitat
  depth raycasting before entering :class:`HierarchicalMemoryStore`.
- Supports GOAT-Bench style sequential multi-subtask episodes with optional memory
  carryover ablation (persistent hierarchical memory vs. per-subtask memory reset).
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import time
from typing import Any, Sequence

import numpy as np

from embodied_navigation.common.runtime_io import atomic_write_json
from embodied_navigation.localization_mapping.semantic_mapping import (
    HierarchicalMemoryStore,
    KeyframeSceneGraph,
)
from embodied_navigation.localization_mapping.semantic_mapping.hierarchical_memory import (
    _label_strict_match,
)
from embodied_navigation.perception.vision_tools import (
    GroundedDetection,
    VisionToolClient,
    build_vision_prompts,
    ground_detections_with_depth,
    render_visual_enhancement_overlay,
)
from embodied_navigation.planning.exploration.streaming_task_agent import (
    OpenCodeTaskPlanner,
    TaskDecision,
)

# ---------------------------------------------------------------------------
# Coordinate Conversion: Habitat (Y-up, -Z forward) <-> Memory (Z-up, +X fwd)
# ---------------------------------------------------------------------------


def hab_to_mem(xyz_hab: Sequence[float]) -> tuple[float, float, float]:
    """Convert Habitat Y-up world coordinates to Z-up robotics coordinates."""
    return (float(xyz_hab[0]), -float(xyz_hab[2]), float(xyz_hab[1]))


def mem_to_hab(xyz_mem: Sequence[float]) -> tuple[float, float, float]:
    """Convert Z-up robotics coordinates back to Habitat Y-up world coordinates."""
    return (float(xyz_mem[0]), float(xyz_mem[2]), -float(xyz_mem[1]))


def hab_quat_to_mem_yaw(quat_wxyz: Sequence[float]) -> float:
    """Extract Z-up heading yaw (rad) from a Habitat agent quaternion (w, x, y, z).

    Habitat's forward axis at identity rotation is (0, 0, -1), which maps to
    (0, +1, 0) in Z-up memory coordinates (yaw = +pi/2).
    """
    w, x, y, z = (float(quat_wxyz[0]), float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3]))
    # Rotate v = (0, 0, -1) by unit quaternion (w, x, y, z)
    fx = -2.0 * (x * z + w * y)
    fy = -2.0 * (y * z - w * x)
    fz = -(1.0 - 2.0 * (x * x + y * y))
    # Map forward vector to Z-up memory frame: (fx, -fz, fy)
    return math.atan2(-fz, fx)


def _pose_matrix_zup(xyz_mem: Sequence[float], yaw_rad: float) -> np.ndarray:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    mat = np.eye(4, dtype=np.float64)
    mat[0, 0], mat[0, 1] = c, -s
    mat[1, 0], mat[1, 1] = s, c
    mat[:3, 3] = [float(xyz_mem[0]), float(xyz_mem[1]), float(xyz_mem[2])]
    return mat


# ---------------------------------------------------------------------------
# Pure-Numpy Depth Raycast & 2D Occupancy / Frontier Builder
# ---------------------------------------------------------------------------


def measure_depth_bearing_range(
    depth_m: np.ndarray,
    bearing_deg: float,
    *,
    hfov_deg: float = 90.0,
    min_range_m: float = 0.25,
    max_range_m: float = 10.0,
    band_half_deg: float = 5.0,
) -> float | None:
    """Measure real planar camera depth along a camera-relative horizontal bearing.

    ``bearing_deg`` uses robotics convention: 0 straight ahead, +left, -right.
    Returns ``None`` if ``bearing_deg`` lies outside the camera's horizontal FOV.
    """
    if depth_m is None or depth_m.ndim < 2 or depth_m.size == 0:
        return None
    half_fov = 0.5 * float(hfov_deg)
    b_deg = float(bearing_deg)
    if abs(b_deg) > half_fov:
        return None
    h, w = depth_m.shape[:2]
    arr = depth_m[:, :, 0] if depth_m.ndim == 3 else depth_m
    fx = (0.5 * w) / math.tan(math.radians(half_fov))
    b_lo = math.radians(max(-half_fov + 0.5, b_deg - band_half_deg))
    b_hi = math.radians(min(half_fov - 0.5, b_deg + band_half_deg))
    # In image coords: +x_col is RIGHT, so +bearing (left) corresponds to smaller col
    col_a = int(round(0.5 * w - fx * math.tan(b_hi)))
    col_b = int(round(0.5 * w - fx * math.tan(b_lo)))
    c0 = max(0, min(w - 1, min(col_a, col_b)))
    c1 = max(c0 + 1, min(w, max(col_a, col_b) + 1))
    r0 = int(0.25 * h)
    r1 = max(r0 + 1, int(0.75 * h))
    patch = arr[r0:r1, c0:c1]
    valid = patch[np.isfinite(patch) & (patch >= min_range_m) & (patch <= max_range_m)]
    if valid.size == 0:
        return None
    return round(float(np.percentile(valid, 25)), 3)


def depth_bearing_to_local_xy(planar_depth_m: float, bearing_deg: float) -> tuple[float, float]:
    """Convert pinhole planar depth ``d`` and horizontal bearing ``b`` (+left) to robot-local ``(x, y)``.

    Matches ``DepthOccupancyGrid`` backprojection: ``x_cam = d``, ``y_cam = d * tan(b)``.
    """
    d = float(planar_depth_m)
    b_rad = math.radians(float(bearing_deg))
    return (d, d * math.tan(b_rad))


def measure_depth_bearing_patch(
    depth_m: np.ndarray,
    bearing_deg: float,
    *,
    hfov_deg: float = 90.0,
    min_range_m: float = 0.25,
    max_range_m: float = 10.0,
    band_half_deg: float = 6.0,
) -> tuple[float, tuple[float, float, float]] | None:
    """Measure planar depth and approximate 3D patch extent ``(dx, dy, dz)`` along a bearing.

    Returns ``None`` if ``bearing_deg`` lies outside the horizontal FOV or lacks valid depth.
    """
    d_med = measure_depth_bearing_range(
        depth_m,
        bearing_deg,
        hfov_deg=hfov_deg,
        min_range_m=min_range_m,
        max_range_m=max_range_m,
        band_half_deg=band_half_deg,
    )
    if d_med is None:
        return None
    half_fov = 0.5 * float(hfov_deg)
    h, w = depth_m.shape[:2]
    arr = depth_m[:, :, 0] if depth_m.ndim == 3 else depth_m
    fx = (0.5 * w) / math.tan(math.radians(half_fov))
    b_lo = math.radians(max(-half_fov + 0.5, float(bearing_deg) - band_half_deg))
    b_hi = math.radians(min(half_fov - 0.5, float(bearing_deg) + band_half_deg))
    col_a = int(round(0.5 * w - fx * math.tan(b_hi)))
    col_b = int(round(0.5 * w - fx * math.tan(b_lo)))
    c0 = max(0, min(w - 1, min(col_a, col_b)))
    c1 = max(c0 + 1, min(w, max(col_a, col_b) + 1))
    r0 = int(0.25 * h)
    r1 = max(r0 + 1, int(0.75 * h))
    patch = arr[r0:r1, c0:c1]
    near_mask = np.isfinite(patch) & (np.abs(patch - d_med) <= max(0.35, 0.2 * d_med))
    ys, xs = np.nonzero(near_mask)
    if ys.size < 4:
        return (d_med, (0.2, 0.2, 0.2))
    dx = max(0.1, min(3.0, float(np.percentile(patch[near_mask], 90) - np.percentile(patch[near_mask], 10))))
    dy = max(0.1, min(3.0, float(np.percentile(xs, 90) - np.percentile(xs, 10)) * d_med / fx))
    dz = max(0.1, min(2.5, float(np.percentile(ys, 90) - np.percentile(ys, 10)) * d_med / fx))
    return (d_med, (round(dx, 2), round(dy, 2), round(dz, 2)))


class DepthOccupancyGrid:
    """Sensor-only 2D occupancy grid built incrementally from Habitat Depth frames."""

    def __init__(
        self,
        *,
        resolution_m: float = 0.10,
        hfov_deg: float = 90.0,
        camera_height_m: float = 0.88,
        max_depth_m: float = 5.5,
        floor_band_m: float = 0.25,
        obstacle_max_h_m: float = 1.60,
    ) -> None:
        self.resolution_m = float(resolution_m)
        self.hfov_deg = float(hfov_deg)
        self.camera_height_m = float(camera_height_m)
        self.max_depth_m = float(max_depth_m)
        self.floor_band_m = float(floor_band_m)
        self.obstacle_max_h_m = float(obstacle_max_h_m)
        # Per floor-bucket (rounded 1.8 m bins) cell dict: (ix, iy) -> 0 (free) or 1 (occupied)
        self._grids: dict[int, dict[tuple[int, int], int]] = {}
        self._visited_trail: list[tuple[float, float, float]] = []
        # Coarse 0.5 m stair-portal evidence per floor: (floor_key, sx, sy) -> stats dict
        self._stair_cells: dict[tuple[int, int, int], dict[str, Any]] = {}

    def _floor_key(self, z_mem: float) -> int:
        return int(round(float(z_mem) / 1.8))

    def _grid_for(self, z_mem: float) -> dict[tuple[int, int], int]:
        fk = self._floor_key(z_mem)
        return self._grids.setdefault(fk, {})

    def _to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor(x / self.resolution_m)), int(math.floor(y / self.resolution_m)))

    def _from_cell(self, ix: int, iy: int) -> tuple[float, float]:
        return ((ix + 0.5) * self.resolution_m, (iy + 0.5) * self.resolution_m)

    def update(
        self,
        depth_m: np.ndarray,
        robot_xyz_mem: Sequence[float],
        yaw_rad: float,
        *,
        stride: int = 8,
    ) -> None:
        """Backproject a depth image into free/occupied 2D cells on the robot's current floor."""
        rx, ry, rz = float(robot_xyz_mem[0]), float(robot_xyz_mem[1]), float(robot_xyz_mem[2])
        self._visited_trail.append((rx, ry, rz))
        grid = self._grid_for(rz)
        # Mark robot footprint as free
        rcx, rcy = self._to_cell(rx, ry)
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                if dx * dx + dy * dy <= 5:
                    grid[(rcx + dx, rcy + dy)] = 0

        if depth_m is None or depth_m.ndim < 2 or depth_m.size == 0:
            return
        arr = depth_m[:, :, 0] if depth_m.ndim == 3 else depth_m
        h, w = arr.shape
        fx = (0.5 * w) / math.tan(math.radians(self.hfov_deg) * 0.5)
        fy = fx
        cx, cy = 0.5 * w, 0.5 * h

        sub = arr[::stride, ::stride]
        vs, us = np.mgrid[0:h:stride, 0:w:stride]
        valid = np.isfinite(sub) & (sub >= 0.25) & (sub <= self.max_depth_m)
        if not np.any(valid):
            return
        d = sub[valid].astype(np.float64)
        u = us[valid].astype(np.float64)
        v = vs[valid].astype(np.float64)

        # Camera frame (Z-up robotics): forward = +x_cam, left = +y_cam, up = +z_cam
        x_cam = d
        y_cam = -(u - cx) * d / fx
        z_cam = -(v - cy) * d / fy + self.camera_height_m

        c_y, s_y = math.cos(yaw_rad), math.sin(yaw_rad)
        wx = rx + c_y * x_cam - s_y * y_cam
        wy = ry + s_y * x_cam + c_y * y_cam

        is_obs = (z_cam >= self.floor_band_m) & (z_cam <= self.obstacle_max_h_m)
        is_free = (z_cam >= -0.25) & (z_cam < self.floor_band_m)

        # Pure depth-geometry stair evidence:
        # 1) Descending stairs: surfaces strictly below current floor plane (-2.4m..-0.38m)
        fk = self._floor_key(rz)
        is_stair_down = (z_cam >= -2.40) & (z_cam <= -0.38) & (d >= 0.65) & (d <= 4.5)
        if np.any(is_stair_down):
            for xw, yw, zc in zip(wx[is_stair_down], wy[is_stair_down], z_cam[is_stair_down]):
                sx, sy = int(math.floor(float(xw) / 0.5)), int(math.floor(float(yw) / 0.5))
                skey = (fk, sx, sy)
                rec = self._stair_cells.get(skey)
                zw = rz + float(zc)
                if rec is None:
                    self._stair_cells[skey] = {
                        "direction": "down",
                        "count": 1,
                        "sum_x": float(xw),
                        "sum_y": float(yw),
                        "extreme_z": zw,
                        "floor_z": rz,
                    }
                else:
                    rec["count"] += 1
                    rec["sum_x"] += float(xw)
                    rec["sum_y"] += float(yw)
                    rec["extreme_z"] = min(float(rec["extreme_z"]), zw)

        # 2) Ascending stairs: columns with sloped/stepped positive depth-vs-height gradient
        is_stair_up_mask = np.zeros_like(is_obs, dtype=bool)
        if np.any(is_obs):
            # Group valid subsampled points by image column u to distinguish sloped stairs from vertical walls
            cand_up = (z_cam >= 0.18) & (z_cam <= 1.35) & (d >= 0.65) & (d <= 4.5)
            if np.any(cand_up):
                up_idxs = np.flatnonzero(cand_up)
                u_vals = u[up_idxs]
                z_vals = z_cam[up_idxs]
                d_vals = d[up_idxs]
                for col_u in np.unique(u_vals):
                    cm = u_vals == col_u
                    if int(np.sum(cm)) < 4:
                        continue
                    cz = z_vals[cm]
                    cd = d_vals[cm]
                    dz_span = float(np.max(cz) - np.min(cz))
                    if dz_span < 0.30:
                        continue
                    order = np.argsort(cz)
                    cz_s = cz[order]
                    cd_s = cd[order]
                    dd_span = float(cd_s[-1] - cd_s[0])
                    slope = dd_span / max(1e-3, dz_span)
                    # Vertical walls have slope ~ 0; ascending stairs increase in depth with height
                    if 0.45 <= slope <= 3.0 and float(np.mean(np.diff(cd_s) >= -0.05)) >= 0.7:
                        col_global = up_idxs[cm]
                        is_stair_up_mask[col_global] = True
                        mid_i = col_global[len(col_global) // 2]
                        xw_u, yw_u = float(wx[mid_i]), float(wy[mid_i])
                        zw_u = rz + float(np.max(cz_s))
                        sx, sy = int(math.floor(xw_u / 0.5)), int(math.floor(yw_u / 0.5))
                        skey = (fk, sx, sy)
                        rec = self._stair_cells.get(skey)
                        if rec is None:
                            self._stair_cells[skey] = {
                                "direction": "up",
                                "count": 1,
                                "sum_x": xw_u,
                                "sum_y": yw_u,
                                "extreme_z": zw_u,
                                "floor_z": rz,
                            }
                        elif rec["direction"] == "up":
                            rec["count"] += 1
                            rec["sum_x"] += xw_u
                            rec["sum_y"] += yw_u
                            rec["extreme_z"] = max(float(rec["extreme_z"]), zw_u)

        # Carve free space along horizontal rays up to 85% of obstacle distance
        for xw, yw in zip(wx[is_free], wy[is_free]):
            cell = self._to_cell(float(xw), float(yw))
            if grid.get(cell) != 1:
                grid[cell] = 0

        # Do not paint ascending stair treads as solid obstacle walls
        eff_obs = is_obs & (~is_stair_up_mask)
        for xw, yw in zip(wx[eff_obs], wy[eff_obs]):
            # Carve intermediate ray points as free (helps open doorways cleanly)
            dist = math.hypot(float(xw) - rx, float(yw) - ry)
            steps = max(1, int(dist / (self.resolution_m * 1.5)))
            for s in range(1, steps):
                frac = s / float(steps + 1)
                c_mid = self._to_cell(rx + frac * (float(xw) - rx), ry + frac * (float(yw) - ry))
                if grid.get(c_mid) != 1:
                    grid[c_mid] = 0
            grid[self._to_cell(float(xw), float(yw))] = 1

    def estimate_clearance_profile(
        self,
        pose_xyz_mem: Sequence[float],
        *,
        num_sectors: int = 16,
        max_range_m: float = 4.5,
    ) -> tuple[float, np.ndarray]:
        """Ray-cast 360-degree obstacle clearance on the floor occupancy grid.

        Returns ``(passage_clearance_m, sector_clearances)`` where ``passage_clearance_m``
        measures minimum opposite-ray half-width across wall-bounded pairs so standing near
        one wall inside an open room is not misclassified as a doorway bottleneck.
        """
        n_sec = max(8, int(num_sectors))
        if n_sec % 2 != 0:
            n_sec += 1
        max_r = max(1.0, float(max_range_m))
        rx, ry, rz = float(pose_xyz_mem[0]), float(pose_xyz_mem[1]), float(pose_xyz_mem[2])
        grid = self._grid_for(rz)
        sectors = np.full(n_sec, max_r, dtype=np.float64)
        if not grid:
            return round(max_r * 0.5, 3), sectors

        step = max(0.05, self.resolution_m * 0.5)
        n_steps = max(1, int(math.ceil(max_r / step)))
        for k in range(n_sec):
            ang = (2.0 * math.pi * k) / float(n_sec)
            ca, sa = math.cos(ang), math.sin(ang)
            hit_d = max_r
            for s in range(1, n_steps + 1):
                r = s * step
                if r < 0.12:
                    continue
                cell = self._to_cell(rx + r * ca, ry + r * sa)
                if grid.get(cell) == 1:
                    hit_d = r
                    break
            sectors[k] = hit_d

        half = n_sec // 2
        bounded_half_widths: list[float] = []
        for k in range(half):
            d1 = float(sectors[k])
            d2 = float(sectors[k + half])
            if d1 < (max_r - 0.15) and d2 < (max_r - 0.15):
                bounded_half_widths.append(0.5 * (d1 + d2))

        if bounded_half_widths:
            clearance = min(bounded_half_widths)
        else:
            clearance = float(np.percentile(sectors, 25))
        return round(max(0.20, min(max_r, float(clearance))), 3), sectors

    def extract_frontiers(
        self,
        robot_xyz_mem: Sequence[float],
        *,
        max_candidates: int = 6,
        min_cluster_cells: int = 3,
        min_dist_m: float = 0.8,
        max_dist_m: float = 8.0,
        visited_suppress_m: float = 0.9,
    ) -> list[dict[str, Any]]:
        """Extract clustered free-to-unknown frontier candidates F1..F*."""
        rx, ry, rz = float(robot_xyz_mem[0]), float(robot_xyz_mem[1]), float(robot_xyz_mem[2])
        grid = self._grid_for(rz)
        if not grid:
            return []

        same_floor_trail = [
            (px, py) for (px, py, pz) in self._visited_trail if abs(pz - rz) <= 1.0
        ]

        frontier_cells: set[tuple[int, int]] = set()
        for (ix, iy), val in grid.items():
            if val != 0:
                continue
            # Must touch unknown AND stay at least 2 cells away from occupied walls
            has_unknown = any(
                (ix + dx, iy + dy) not in grid for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
            )
            if not has_unknown:
                continue
            near_occ = any(
                grid.get((ix + dx, iy + dy)) == 1
                for dx in range(-2, 3)
                for dy in range(-2, 3)
            )
            if not near_occ:
                frontier_cells.add((ix, iy))

        # Connected-component clustering (8-neighborhood)
        clusters: list[list[tuple[int, int]]] = []
        unvisited = set(frontier_cells)
        while unvisited:
            seed = unvisited.pop()
            comp = [seed]
            stack = [seed]
            while stack:
                cx, cy = stack.pop()
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nb = (cx + dx, cy + dy)
                        if nb in unvisited:
                            unvisited.remove(nb)
                            comp.append(nb)
                            stack.append(nb)
            if len(comp) >= min_cluster_cells:
                clusters.append(comp)

        fresh_cands: list[tuple[float, dict[str, Any]]] = []
        suppressed_cands: list[tuple[float, dict[str, Any]]] = []
        for comp in clusters:
            mx = sum(c[0] for c in comp) / len(comp)
            my = sum(c[1] for c in comp) / len(comp)
            # Pick medoid cell inside cluster so candidate is guaranteed a valid free cell
            med_ix, med_iy = min(
                comp, key=lambda c: (c[0] - mx) * (c[0] - mx) + (c[1] - my) * (c[1] - my)
            )
            wx, wy = self._from_cell(med_ix, med_iy)
            dist = math.hypot(wx - rx, wy - ry)
            if dist < min_dist_m or dist > max_dist_m:
                continue
            # Prefer larger frontiers at moderate exploration distance (~2-5 m)
            score = len(comp) * 1.5 - abs(dist - 3.0) * 2.0
            cand = {
                "kind": "frontier",
                "xy": [round(wx, 2), round(wy, 2)],
                "z": round(rz, 3),
                "dist_m": round(dist, 2),
                "frontier_cells": len(comp),
                "clearance_m": round(self.resolution_m * 2.5, 2),
                "turn_safe": True,
            }
            near_visited = (
                visited_suppress_m > 0.0
                and any(
                    math.hypot(wx - vx, wy - vy) < visited_suppress_m
                    for (vx, vy) in same_floor_trail
                )
            )
            if near_visited:
                suppressed_cands.append((dist, cand))
            else:
                fresh_cands.append((score, cand))

        if fresh_cands:
            raw_cands = sorted(fresh_cands, key=lambda item: -item[0])
        elif suppressed_cands:
            # Keep only the single farthest suppressed frontier to avoid restoring ping-pong loops
            best_sup = max(suppressed_cands, key=lambda item: item[0])
            best_sup[1]["revisit_fallback"] = True
            raw_cands = [best_sup]
        else:
            raw_cands = []
        out: list[dict[str, Any]] = []
        for idx, (_score, cand) in enumerate(raw_cands[: max(1, int(max_candidates))], start=1):
            cand["id"] = f"F{idx}"
            out.append(cand)
        return out

    def extract_stair_portals(
        self,
        robot_xyz_mem: Sequence[float],
        *,
        max_portals: int = 3,
        min_samples: int = 4,
    ) -> list[dict[str, Any]]:
        """Cluster depth-observed stair openings (S1..S*) on the current floor."""
        rx, ry, rz = float(robot_xyz_mem[0]), float(robot_xyz_mem[1]), float(robot_xyz_mem[2])
        fk = self._floor_key(rz)
        pts: list[tuple[int, int, dict[str, Any]]] = [
            (sx, sy, rec)
            for (rfk, sx, sy), rec in self._stair_cells.items()
            if rfk == fk and int(rec.get("count", 0)) >= int(min_samples)
        ]
        if not pts:
            return []

        # Group nearby 0.5m cells of the same direction into stair portal clusters
        by_dir_key = {(sx, sy, str(rec["direction"])): rec for sx, sy, rec in pts}
        unvisited = set(by_dir_key.keys())
        clusters: list[tuple[str, list[dict[str, Any]]]] = []
        while unvisited:
            seed = unvisited.pop()
            sdir = seed[2]
            comp = [by_dir_key[seed]]
            stack = [seed]
            while stack:
                cx, cy, cd = stack.pop()
                for dx in (-2, -1, 0, 1, 2):
                    for dy in (-2, -1, 0, 1, 2):
                        nb = (cx + dx, cy + dy, cd)
                        if nb in unvisited:
                            unvisited.remove(nb)
                            comp.append(by_dir_key[nb])
                            stack.append(nb)
            clusters.append((sdir, comp))

        scored: list[tuple[float, dict[str, Any]]] = []
        for sdir, comp in clusters:
            total_cnt = sum(int(r["count"]) for r in comp)
            wx = sum(float(r["sum_x"]) for r in comp) / max(1, total_cnt)
            wy = sum(float(r["sum_y"]) for r in comp) / max(1, total_cnt)
            dist = math.hypot(wx - rx, wy - ry)
            if dist > 12.0:
                continue
            target_z = (
                min(float(r["extreme_z"]) for r in comp)
                if sdir == "down"
                else max(float(r["extreme_z"]) for r in comp)
            )
            scored.append(
                (
                    total_cnt - 0.5 * dist,
                    {
                        "kind": f"stair_{sdir}",
                        "direction": sdir,
                        "xy": [round(wx, 2), round(wy, 2)],
                        "z": round(target_z, 3),
                        "dist_m": round(dist, 2),
                        "description": f"observed staircase leading {sdir} (depth evidence={total_cnt})",
                    },
                )
            )
        scored.sort(key=lambda item: -item[0])
        out: list[dict[str, Any]] = []
        for idx, (_sc, cand) in enumerate(scored[: max(1, int(max_portals))], start=1):
            cand["id"] = f"S{idx}"
            out.append(cand)
        return out

    def render_local_map(
        self,
        out_path: str | Path,
        robot_xyz_mem: Sequence[float],
        yaw_rad: float,
        candidates: Sequence[dict[str, Any]],
        *,
        half_extent_m: float = 6.5,
    ) -> Path:
        """Render a top-down 2D annotated local map PNG for VLM spatial grounding."""
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        rx, ry, rz = float(robot_xyz_mem[0]), float(robot_xyz_mem[1]), float(robot_xyz_mem[2])
        grid = self._grid_for(rz)
        fig, ax = plt.subplots(figsize=(4.2, 4.2), dpi=110)
        ax.set_facecolor("#20242b")

        fx_list, fy_list, ox_list, oy_list = [], [], [], []
        for (ix, iy), val in grid.items():
            wx, wy = self._from_cell(ix, iy)
            if abs(wx - rx) > half_extent_m or abs(wy - ry) > half_extent_m:
                continue
            if val == 0:
                fx_list.append(wx)
                fy_list.append(wy)
            elif val == 1:
                ox_list.append(wx)
                oy_list.append(wy)

        if fx_list:
            ax.scatter(fx_list, fy_list, s=6, c="#d7dee8", marker="s", linewidths=0)
        if ox_list:
            ax.scatter(ox_list, oy_list, s=8, c="#111318", marker="s", linewidths=0)

        trail = [p for p in self._visited_trail if abs(p[2] - rz) <= 1.0]
        if len(trail) >= 2:
            ax.plot([p[0] for p in trail], [p[1] for p in trail], c="#3b82f6", lw=1.5, alpha=0.8)

        # Robot position & heading arrow
        ax.scatter([rx], [ry], s=60, c="#22c55e", edgecolors="white", zorder=6)
        ax.arrow(
            rx,
            ry,
            0.55 * math.cos(yaw_rad),
            0.55 * math.sin(yaw_rad),
            head_width=0.22,
            head_length=0.25,
            fc="#22c55e",
            ec="white",
            zorder=7,
        )

        for cand in candidates:
            xy = cand.get("xy")
            cid = cand.get("id")
            if not isinstance(xy, (list, tuple)) or len(xy) < 2 or not cid:
                continue
            color = "#f97316" if str(cid).startswith("M") else "#eab308"
            ax.scatter([xy[0]], [xy[1]], s=55, c=color, edgecolors="black", zorder=8)
            ax.text(
                float(xy[0]) + 0.14,
                float(xy[1]) + 0.14,
                str(cid),
                color="white",
                fontsize=8,
                weight="bold",
                bbox={"facecolor": color, "alpha": 0.85, "pad": 1.2, "edgecolor": "none"},
                zorder=9,
            )

        ax.set_xlim(rx - half_extent_m, rx + half_extent_m)
        ax.set_ylim(ry - half_extent_m, ry + half_extent_m)
        ax.set_aspect("equal")
        ax.set_title(f"2D Depth Map (z={rz:.2f}m)", fontsize=9, color="white")
        ax.tick_params(colors="white", labelsize=7)
        fig.patch.set_facecolor("#15181e")
        out = Path(out_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out, bbox_inches="tight", facecolor=fig.get_facecolor())
        plt.close(fig)
        return out

    def render_global_floorplan(
        self,
        out_path: str | Path,
        robot_xyz_mem: Sequence[float],
        yaw_rad: float,
        *,
        candidates: Sequence[dict[str, Any]] = (),
        zones: Sequence[dict[str, Any]] = (),
        objects: Sequence[dict[str, Any]] = (),
    ) -> Path:
        """Render a global 2.5D semantic floorplan covering all explored cells on the current floor."""
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        rx, ry, rz = float(robot_xyz_mem[0]), float(robot_xyz_mem[1]), float(robot_xyz_mem[2])
        grid = self._grid_for(rz)
        fig, ax = plt.subplots(figsize=(5.0, 5.0), dpi=115)
        ax.set_facecolor("#1d2128")

        fx_list, fy_list, ox_list, oy_list = [], [], [], []
        for (ix, iy), val in grid.items():
            wx, wy = self._from_cell(ix, iy)
            if val == 0:
                fx_list.append(wx)
                fy_list.append(wy)
            elif val == 1:
                ox_list.append(wx)
                oy_list.append(wy)

        if fx_list:
            ax.scatter(fx_list, fy_list, s=5, c="#dce3ec", marker="s", linewidths=0)
        if ox_list:
            ax.scatter(ox_list, oy_list, s=7, c="#0f1116", marker="s", linewidths=0)

        all_x = [*fx_list, *ox_list, rx]
        all_y = [*fy_list, *oy_list, ry]
        min_x, max_x = min(all_x) - 1.5, max(all_x) + 1.5
        min_y, max_y = min(all_y) - 1.5, max(all_y) + 1.5
        if (max_x - min_x) < 10.0:
            mid_x = 0.5 * (min_x + max_x)
            min_x, max_x = mid_x - 5.0, mid_x + 5.0
        if (max_y - min_y) < 10.0:
            mid_y = 0.5 * (min_y + max_y)
            min_y, max_y = mid_y - 5.0, mid_y + 5.0

        trail = [p for p in self._visited_trail if abs(p[2] - rz) <= 1.2]
        if len(trail) >= 2:
            ax.plot([p[0] for p in trail], [p[1] for p in trail], c="#3b82f6", lw=1.4, alpha=0.85)

        same_floor_zones = [
            z
            for z in zones
            if isinstance(z.get("center_xyz"), (list, tuple))
            and len(z["center_xyz"]) >= 3
            and abs(float(z["center_xyz"][2]) - rz) <= 1.5
        ]
        zone_pos_map: dict[str, tuple[float, float]] = {}
        for z in same_floor_zones[:14]:
            cxyz = z.get("center_xyz")
            zid = str(z.get("zone_id") or z.get("region_id") or "")
            if not zid:
                continue
            zone_pos_map[zid] = (float(cxyz[0]), float(cxyz[1]))
        # Draw inter-region topological connections (doorway / open_passage / stair)
        drawn_edges: set[tuple[str, str]] = set()
        for z in same_floor_zones[:14]:
            zid = str(z.get("zone_id") or z.get("region_id") or "")
            if zid not in zone_pos_map:
                continue
            p1 = zone_pos_map[zid]
            for conn in z.get("connected_regions") or ():
                if not isinstance(conn, dict):
                    continue
                to_id = str(conn.get("to") or "")
                if to_id not in zone_pos_map:
                    continue
                ek = (min(zid, to_id), max(zid, to_id))
                if ek in drawn_edges:
                    continue
                drawn_edges.add(ek)
                p2 = zone_pos_map[to_id]
                via = str(conn.get("via") or "")
                ls = "--" if via == "open_passage" else "-"
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], c="#60a5fa", lw=1.1, ls=ls, alpha=0.6, zorder=4)
        for z in same_floor_zones[:14]:
            cxyz = z.get("center_xyz")
            zid = z.get("zone_id") or z.get("region_id")
            if not zid:
                continue
            ztype = str(z.get("primary_label") or z.get("zone_type") or "open")[:14]
            status_tag = "*" if z.get("status") == "inspected_closeup" else "~"
            ax.text(
                float(cxyz[0]),
                float(cxyz[1]),
                f"{zid}{status_tag}:{ztype}",
                color="#93c5fd",
                fontsize=6.8,
                ha="center",
                va="center",
                bbox={"facecolor": "#0f172a", "alpha": 0.62, "pad": 1.2, "edgecolor": "#334155"},
                zorder=5,
            )

        for obj in objects[:16]:
            oxyz = obj.get("world_xyz")
            lbl = str(obj.get("label") or "").strip()
            if not isinstance(oxyz, (list, tuple)) or len(oxyz) < 3 or not lbl:
                continue
            if abs(float(oxyz[2]) - rz) > 1.5:
                continue
            ax.scatter([oxyz[0]], [oxyz[1]], s=24, c="#10b981", marker="o", alpha=0.85, zorder=6)
            ax.text(
                float(oxyz[0]) + 0.12,
                float(oxyz[1]) - 0.14,
                lbl[:12],
                color="#a7f3d0",
                fontsize=6.2,
                zorder=6,
            )

        ax.scatter([rx], [ry], s=65, c="#22c55e", edgecolors="white", zorder=7)
        ax.arrow(
            rx,
            ry,
            0.60 * math.cos(yaw_rad),
            0.60 * math.sin(yaw_rad),
            head_width=0.25,
            head_length=0.28,
            fc="#22c55e",
            ec="white",
            zorder=8,
        )

        for cand in candidates:
            xy = cand.get("xy")
            cid = cand.get("id")
            if not isinstance(xy, (list, tuple)) or len(xy) < 2 or not cid:
                continue
            cid_s = str(cid)
            if cid_s.startswith("M"):
                color, marker = "#f97316", "o"
            elif cid_s.startswith("S"):
                color, marker = "#d946ef", "D"
            else:
                color, marker = "#eab308", "o"
            ax.scatter([xy[0]], [xy[1]], s=56, c=color, marker=marker, edgecolors="black", zorder=9)
            ax.text(
                float(xy[0]) + 0.14,
                float(xy[1]) + 0.14,
                str(cid),
                color="white",
                fontsize=7.5,
                weight="bold",
                bbox={"facecolor": color, "alpha": 0.85, "pad": 1.0, "edgecolor": "none"},
                zorder=10,
            )

        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
        ax.set_aspect("equal")
        ax.set_xlabel("X (m)", fontsize=7, color="#cbd5e1")
        ax.set_ylabel("Y (m)", fontsize=7, color="#cbd5e1")
        ax.set_title(f"Global 2.5D Floorplan (z={rz:.2f}m | 1 div=grid m)", fontsize=9, color="white")
        ax.grid(color="#334155", linestyle=":", linewidth=0.5, alpha=0.45)
        ax.tick_params(colors="white", labelsize=7)
        fig.patch.set_facecolor("#15181e")
        out = Path(out_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out, bbox_inches="tight", facecolor=fig.get_facecolor())
        plt.close(fig)
        return out


# ---------------------------------------------------------------------------
# Private Semantic Ground-Truth Scanner (Strictly Hidden from Agent!)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class HabitatObjectGT:
    """Private ground-truth 3D surface points, centroid, and visible viewpoints of one HM3D object."""

    object_id: str
    category: str
    center_hab: tuple[float, float, float]
    center_mem: tuple[float, float, float]
    viewpoints_hab: tuple[tuple[float, float, float], ...]
    surface_points_mem: tuple[tuple[float, float, float], ...] = ()


def is_geodesic_target_reached(
    geo_to_vp_m: float,
    euc_to_obj_m: float,
    stop_radius_m: float,
    *,
    viewpoint_tol_m: float | None = None,
) -> bool:
    """True iff agent is both geodesically close to an unoccluded viewpoint AND within stop radius.

    Enforces two simultaneous invariants:
    1. ``euc_to_obj_m <= stop_radius_m`` — success zone never exceeds the promised stop radius.
    2. ``geo_to_vp_m <= vp_tol`` — strictly prevents through-wall Euclidean false positives.
    """
    r = float(stop_radius_m)
    vp_tol = min(1.0, r * 0.6) if viewpoint_tol_m is None else float(viewpoint_tol_m)
    return (
        math.isfinite(geo_to_vp_m)
        and math.isfinite(euc_to_obj_m)
        and geo_to_vp_m <= vp_tol
        and euc_to_obj_m <= r
    )


def compute_spl(success: bool, optimal_geo_m: float, traveled_m: float) -> float:
    """Compute honest Habitat SPL without cross-floor or zero-optimal inflation."""
    if not success or not math.isfinite(optimal_geo_m) or optimal_geo_m < 0.0:
        return 0.0
    tr = max(0.0, float(traveled_m))
    if optimal_geo_m <= 1e-6:
        return 1.0 if tr <= 1e-6 else 0.0
    return float(optimal_geo_m / max(optimal_geo_m, tr))


def _dedup_viewpoints(
    vps: Sequence[tuple[float, float, float]],
    *,
    min_spacing_m: float = 0.4,
    max_keep: int = 24,
) -> tuple[tuple[float, float, float], ...]:
    kept: list[tuple[float, float, float]] = []
    for vp in vps:
        if all(
            math.hypot(vp[0] - k[0], vp[2] - k[2]) >= min_spacing_m or abs(vp[1] - k[1]) >= 0.8
            for k in kept
        ):
            kept.append(vp)
            if len(kept) >= max_keep:
                break
    return tuple(kept)


def build_private_hm3d_gt(
    sim: Any,
    *,
    target_categories: Sequence[str],
    num_samples: int = 160,
    seed: int = 42,
    hfov_deg: float = 90.0,
    min_pixels: int = 180,
    max_view_dist_m: float = 3.5,
) -> dict[str, list[HabitatObjectGT]]:
    """Build private 3D object ground truth by sampling navigable viewpoints on GPU.

    Never exposed to the agent; used exclusively for computing honest NavMesh
    optimal geodesic length, OSR, SR, and SPL.
    """
    import quaternion

    sem = sim.semantic_scene
    id_to_cat: dict[int, tuple[str, str]] = {}
    target_set = {c.strip().lower() for c in target_categories}
    for obj in sem.objects:
        if obj is None or obj.category is None:
            continue
        cname = obj.category.name().strip().lower()
        if cname not in target_set:
            continue
        raw_sem_id = getattr(obj, "semantic_id", None)
        if isinstance(raw_sem_id, int) and not isinstance(raw_sem_id, bool):
            sem_int = raw_sem_id
        else:
            # Fallback for HM3D object IDs like "chair_149" -> numeric semantic id 149
            try:
                sem_int = int(str(obj.id).split("_")[-1])
            except ValueError:
                continue
        id_to_cat[sem_int] = (str(obj.id), cname)

    if not id_to_cat:
        return {}

    sim.pathfinder.seed(seed)
    saved_state = sim.agents[0].get_state()

    accum_pts: dict[int, list[np.ndarray]] = {}
    accum_surf_mem: dict[int, list[tuple[float, float, float]]] = {}
    accum_vps: dict[int, list[tuple[float, float, float]]] = {}

    # Sample navigable points and 4 cardinal yaw orientations per point
    for _ in range(num_samples):
        pt = np.asarray(sim.pathfinder.get_random_navigable_point(), dtype=np.float64)
        if not np.all(np.isfinite(pt)):
            continue
        for k_yaw in range(4):
            yaw = k_yaw * (math.pi * 0.5)
            st = sim.agents[0].get_state()
            st.position = pt.astype(np.float32)
            st.rotation = quaternion.from_rotation_vector([0.0, yaw, 0.0])
            sim.agents[0].set_state(st)
            obs = sim.get_sensor_observations()
            sem_img = obs["semantic_sensor"]
            depth_img = obs["depth_sensor"]
            h, w = depth_img.shape[:2]
            fx = (0.5 * w) / math.tan(math.radians(hfov_deg) * 0.5)
            # Agent forward in Habitat at yaw around +Y: (-sin(yaw), 0, -cos(yaw))
            # Right in Habitat: (cos(yaw), 0, -sin(yaw))
            fwd = np.array([-math.sin(yaw), 0.0, -math.cos(yaw)], dtype=np.float64)
            right = np.array([math.cos(yaw), 0.0, -math.sin(yaw)], dtype=np.float64)
            up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            cam_pos = pt + np.array([0.0, 0.88, 0.0], dtype=np.float64)

            uniq, counts = np.unique(sem_img, return_counts=True)
            for uid, cnt in zip(uniq, counts):
                uid_i = int(uid)
                if uid_i not in id_to_cat or int(cnt) < min_pixels:
                    continue
                mask = (sem_img == uid_i) & np.isfinite(depth_img) & (depth_img >= 0.3) & (depth_img <= max_view_dist_m)
                if int(np.sum(mask)) < min_pixels:
                    continue
                ys, xs = np.nonzero(mask)
                # Subsample up to 40 pixels for accurate 3D surface & centroid
                step_s = max(1, len(ys) // 40)
                ys = ys[::step_s]
                xs = xs[::step_s]
                ds = depth_img[ys, xs].astype(np.float64)
                xc = (xs - 0.5 * w) * ds / fx
                yc = -(ys - 0.5 * h) * ds / fx
                pts_hab = (
                    cam_pos[None, :]
                    + ds[:, None] * fwd[None, :]
                    + xc[:, None] * right[None, :]
                    + yc[:, None] * up[None, :]
                )
                med_pt = np.median(pts_hab, axis=0)
                if float(np.linalg.norm(med_pt - cam_pos)) > float(max_view_dist_m):
                    continue
                dists_3d = np.linalg.norm(pts_hab - cam_pos[None, :], axis=1)
                accum_pts.setdefault(uid_i, []).append(med_pt)
                surf_bucket = accum_surf_mem.setdefault(uid_i, [])
                if len(surf_bucket) < 24:
                    in_range_pts = pts_hab[dists_3d <= float(max_view_dist_m)]
                    for sp in in_range_pts[:: max(1, len(in_range_pts) // 4)]:
                        sm = hab_to_mem(sp)
                        if all(
                            math.hypot(sm[0] - k[0], sm[1] - k[1]) >= 0.25 or abs(sm[2] - k[2]) >= 0.35
                            for k in surf_bucket
                        ):
                            surf_bucket.append(sm)
                            if len(surf_bucket) >= 24:
                                break
                accum_vps.setdefault(uid_i, []).append((float(pt[0]), float(pt[1]), float(pt[2])))

    sim.agents[0].set_state(saved_state)

    by_cat: dict[str, list[HabitatObjectGT]] = {}
    for uid_i, pts_list in accum_pts.items():
        oid, cname = id_to_cat[uid_i]
        center_hab = np.median(np.vstack(pts_list), axis=0)
        vps = _dedup_viewpoints(accum_vps.get(uid_i, []))
        if not vps:
            continue
        by_cat.setdefault(cname, []).append(
            HabitatObjectGT(
                object_id=oid,
                category=cname,
                center_hab=(float(center_hab[0]), float(center_hab[1]), float(center_hab[2])),
                center_mem=hab_to_mem(center_hab),
                viewpoints_hab=vps,
                surface_points_mem=tuple(accum_surf_mem.get(uid_i, [])),
            )
        )
    return by_cat


# ---------------------------------------------------------------------------
# Closed-Loop Habitat Sequential Multi-Task Memory Runner (GOAT-Bench Style)
# ---------------------------------------------------------------------------


@dataclass
class HabitatSubtaskSpec:
    subtask_id: str
    target_category: str
    instruction: str
    stop_radius_m: float = 1.8
    max_steps: int | None = None
    max_primitive_steps: int = 500


class HabitatMemoryRunner:
    """Run GOAT-Bench style sequential multi-task navigation inside real Habitat-Sim HM3D."""

    def __init__(
        self,
        *,
        scene_dataset_config: str | Path,
        scene_id: str | Path,
        planner: OpenCodeTaskPlanner,
        output_dir: str | Path,
        stop_radius_m: float = 1.8,
        memory_carryover: bool = True,
        reset_occupancy_per_subtask: bool = False,
        vision_client: VisionToolClient | None = None,
        vision_enhance_overlay: bool = True,
        vision_score_threshold: float = 0.25,
        vision_fuse_threshold: float = 0.35,
        vision_max_prompts: int = 6,
        vision_deferred_l3: bool = True,
        gpu_device_id: int = 0,
        seed: int = 42,
    ) -> None:
        self.scene_dataset_config = str(scene_dataset_config)
        self.scene_id = str(scene_id)
        self.planner = planner
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.stop_radius_m = float(stop_radius_m)
        self.memory_carryover = bool(memory_carryover)
        self.reset_occupancy_per_subtask = bool(reset_occupancy_per_subtask)
        self.vision_client = vision_client
        self.vision_enhance_overlay = bool(vision_enhance_overlay)
        self.vision_score_threshold = float(vision_score_threshold)
        self.vision_fuse_threshold = float(vision_fuse_threshold)
        self.vision_max_prompts = max(1, int(vision_max_prompts))
        self.vision_deferred_l3 = bool(vision_deferred_l3)
        self.gpu_device_id = int(gpu_device_id)
        self.seed = int(seed)

        if self.planner.allowed_image_roots is None:
            self.planner.allowed_image_roots = (self.output_dir.resolve(),)
        else:
            roots = list(self.planner.allowed_image_roots)
            out_res = self.output_dir.resolve()
            if out_res not in roots:
                roots.append(out_res)
            self.planner.allowed_image_roots = tuple(roots)

    def _create_sim(self) -> Any:
        import habitat_sim

        sim_cfg = habitat_sim.SimulatorConfiguration()
        sim_cfg.scene_dataset_config_file = self.scene_dataset_config
        sim_cfg.scene_id = self.scene_id
        sim_cfg.enable_physics = True
        sim_cfg.gpu_device_id = self.gpu_device_id

        rgb_spec = habitat_sim.CameraSensorSpec()
        rgb_spec.uuid = "color_sensor"
        rgb_spec.sensor_type = habitat_sim.SensorType.COLOR
        rgb_spec.resolution = [512, 512]
        rgb_spec.position = [0.0, 0.88, 0.0]

        depth_spec = habitat_sim.CameraSensorSpec()
        depth_spec.uuid = "depth_sensor"
        depth_spec.sensor_type = habitat_sim.SensorType.DEPTH
        depth_spec.resolution = [512, 512]
        depth_spec.position = [0.0, 0.88, 0.0]

        # Strictly private for offline GT scan; NEVER read by agent or planner!
        sem_spec = habitat_sim.CameraSensorSpec()
        sem_spec.uuid = "semantic_sensor"
        sem_spec.sensor_type = habitat_sim.SensorType.SEMANTIC
        sem_spec.resolution = [512, 512]
        sem_spec.position = [0.0, 0.88, 0.0]

        agent_cfg = habitat_sim.agent.AgentConfiguration()
        agent_cfg.sensor_specifications = [rgb_spec, depth_spec, sem_spec]
        agent_cfg.action_space = {
            "move_forward": habitat_sim.agent.ActionSpec(
                "move_forward", habitat_sim.agent.ActuationSpec(amount=0.25)
            ),
            "turn_left": habitat_sim.agent.ActionSpec(
                "turn_left", habitat_sim.agent.ActuationSpec(amount=15.0)
            ),
            "turn_right": habitat_sim.agent.ActionSpec(
                "turn_right", habitat_sim.agent.ActuationSpec(amount=15.0)
            ),
        }
        return habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg]))

    def _agent_pose_mem(self, sim: Any) -> tuple[list[float], float, np.ndarray]:
        st = sim.agents[0].get_state()
        p_hab = np.asarray(st.position, dtype=np.float64)
        q = st.rotation
        yaw_mem = hab_quat_to_mem_yaw((q.w, q.x, q.y, q.z))
        return list(hab_to_mem(p_hab)), yaw_mem, p_hab

    def _geodesic_dist(self, sim: Any, start_hab: Sequence[float], end_hab: Sequence[float]) -> float:
        import habitat_sim

        sp = habitat_sim.ShortestPath()
        sp.requested_start = np.asarray(start_hab, dtype=np.float32)
        sp.requested_end = np.asarray(end_hab, dtype=np.float32)
        if sim.pathfinder.find_path(sp) and math.isfinite(float(sp.geodesic_distance)):
            return float(sp.geodesic_distance)
        return float("inf")

    def _step_toward_goal(
        self,
        sim: Any,
        occ_grid: DepthOccupancyGrid,
        goal_xyz_mem: Sequence[float],
        *,
        max_primitives: int = 24,
    ) -> dict[str, Any]:
        """Step toward agent-chosen goal using real discrete Habitat primitives.

        Updates the live depth occupancy grid every 4 primitives and stops early
        if stuck or arrived within 0.45 m of the requested waypoint.
        """
        import habitat_sim

        goal_hab = np.asarray(mem_to_hab(goal_xyz_mem), dtype=np.float32)
        snapped = np.asarray(sim.pathfinder.snap_point(goal_hab), dtype=np.float32)
        if not np.all(np.isfinite(snapped)) or not sim.pathfinder.is_navigable(snapped):
            return {"status": "failed", "reason": "unnavigable_goal", "traveled_m": 0.0, "primitives": 0}

        start_mem, _, start_hab = self._agent_pose_mem(sim)
        path_dist = self._geodesic_dist(sim, start_hab, snapped)
        if not math.isfinite(path_dist):
            return {"status": "failed", "reason": "disconnected_navmesh", "traveled_m": 0.0, "primitives": 0}

        follower = habitat_sim.GreedyGeodesicFollower(
            sim.pathfinder, sim.agents[0], goal_radius=0.45
        )
        traveled = 0.0
        prev_hab = start_hab
        prims_taken = 0
        arrived = False
        follower_error: str | None = None
        traj_hab: list[tuple[float, float, float]] = [
            (float(start_hab[0]), float(start_hab[1]), float(start_hab[2]))
        ]
        for idx in range(max_primitives):
            try:
                act = follower.next_action_along(snapped)
            except habitat_sim.errors.GreedyFollowerError:
                follower_error = "greedy_follower_error"
                break
            if act is None or act == "stop":
                arrived = True
                break
            obs = sim.step(act)
            prims_taken += 1
            cur_mem, cur_yaw, cur_hab = self._agent_pose_mem(sim)
            traj_hab.append((float(cur_hab[0]), float(cur_hab[1]), float(cur_hab[2])))
            traveled += float(np.linalg.norm(cur_hab - prev_hab))
            prev_hab = cur_hab
            if idx % 4 == 3:
                occ_grid.update(obs["depth_sensor"], cur_mem, cur_yaw)

        cur_mem, cur_yaw, cur_hab = self._agent_pose_mem(sim)
        obs = sim.get_sensor_observations()
        occ_grid.update(obs["depth_sensor"], cur_mem, cur_yaw)
        rem_geo = self._geodesic_dist(sim, cur_hab, snapped)
        rem_out = None if not math.isfinite(rem_geo) else round(rem_geo, 3)
        if arrived or (math.isfinite(rem_geo) and rem_geo <= 0.65):
            status = "completed"
            reason = "waypoint_reached"
        elif prims_taken == 0:
            status = "failed"
            reason = follower_error or "follower_no_action"
        elif follower_error is not None and traveled < 0.25:
            status = "failed"
            reason = follower_error
        elif traveled < 0.25 and prims_taken >= 4:
            status = "stuck"
            reason = "no_forward_progress"
        else:
            status = "completed"
            reason = "partial_progress"
        return {
            "status": status,
            "reason": reason,
            "traveled_m": round(traveled, 3),
            "primitives": prims_taken,
            "remaining_m": rem_out,
            "_traj_hab": traj_hab,
        }

    def _eval_gt_proximity(
        self,
        sim: Any,
        robot_hab: Sequence[float],
        gt_list: Sequence[HabitatObjectGT],
        stop_radius_m: float,
    ) -> tuple[float, float, bool, HabitatObjectGT | None]:
        """Return ``(min_euc_m, min_geo_to_vp_m, reached_same_object, nearest_gt)``.

        Evaluates NavMesh geodesic across ALL valid viewpoints (including cross-floor
        stair routes) without height-filtering so cross-floor SPL is exact.
        """
        best_euc = float("inf")
        best_geo = float("inf")
        best_gt: HabitatObjectGT | None = None
        reached = False
        r_mem = np.asarray(hab_to_mem(robot_hab), dtype=np.float64)
        for gt in gt_list:
            if gt.surface_points_mem:
                surf_arr = np.asarray(gt.surface_points_mem, dtype=np.float64)
                d_euc = float(np.min(np.linalg.norm(surf_arr - r_mem[None, :], axis=1)))
            else:
                d_euc = float(np.linalg.norm(r_mem - np.asarray(gt.center_mem, dtype=np.float64)))
            obj_best_geo = float("inf")
            for vp in gt.viewpoints_hab:
                d_geo = self._geodesic_dist(sim, robot_hab, vp)
                if d_geo < obj_best_geo:
                    obj_best_geo = d_geo
            if obj_best_geo < best_geo:
                best_geo = obj_best_geo
                best_gt = gt
            if d_euc < best_euc:
                best_euc = d_euc
            if is_geodesic_target_reached(obj_best_geo, d_euc, stop_radius_m):
                reached = True
        return best_euc, best_geo, reached, best_gt

    def _deferred_vision_scan_keyframes(
        self,
        query: str,
        top_keyframes: Sequence[dict[str, Any]],
        *,
        has_seg: bool,
        max_kfs: int = 2,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        """Run free local vision-tool segmentation on Top-K historical L3 keyframes."""
        if self.vision_client is None or not has_seg or not self.vision_deferred_l3:
            return [], []
        out_root = self.output_dir.resolve()
        hits: list[dict[str, Any]] = []
        hit_rgbs: list[str] = []
        scanned = 0
        for kf in top_keyframes:
            if scanned >= max(1, int(max_kfs)):
                break
            rp = kf.get("rgb_path")
            if not isinstance(rp, str) or not rp:
                continue
            try:
                resolved = Path(rp).resolve()
                resolved.relative_to(out_root)
            except (OSError, ValueError):
                continue
            if not resolved.is_file():
                continue
            scanned += 1
            dets = self.vision_client.segment_text(
                resolved,
                [query],
                score_threshold=self.vision_fuse_threshold,
                max_detections=3,
                include_masks=False,
                include_crop_embeddings=False,
            )
            if not dets:
                continue
            best = max(dets, key=lambda d: float(d.score))
            hits.append(
                {
                    "keyframe_id": str(kf.get("keyframe_id", "")),
                    "pose_xyz": kf.get("pose_xyz"),
                    "label": best.label,
                    "score": round(float(best.score), 2),
                    "box_px": [int(round(v)) for v in best.box_xyxy],
                    "backend": best.backend,
                }
            )
            if str(resolved) not in hit_rgbs:
                hit_rgbs.append(str(resolved))
        return hits, hit_rgbs

    def run_sequential_episode(
        self,
        subtasks: Sequence[HabitatSubtaskSpec],
        *,
        start_hab: Sequence[float] | None = None,
        independent_subtasks: bool = False,
    ) -> dict[str, Any]:
        """Execute a sequential multi-subtask (or independent single-target) Habitat-Sim episode."""
        from PIL import Image
        import quaternion

        t_ep = time.perf_counter()
        vision_health: dict[str, Any] = {"ok": False, "backend": "none", "embed_model": ""}
        if self.vision_client is not None:
            vision_health = self.vision_client.health()
        vision_ok = bool(vision_health.get("ok"))
        vision_has_seg = vision_ok and str(vision_health.get("backend", "none")) != "none"
        vision_has_embed = vision_ok and bool(str(vision_health.get("embed_model", "")).strip())
        total_vision_dets = 0
        total_vision_fused = 0
        total_deferred_hits = 0

        sim = self._create_sim()
        try:
            target_cats = sorted({st.target_category.strip().lower() for st in subtasks})
            max_stop_r = max((float(st.stop_radius_m) for st in subtasks), default=self.stop_radius_m)
            private_gt = build_private_hm3d_gt(
                sim,
                target_categories=target_cats,
                num_samples=260,
                seed=self.seed,
                max_view_dist_m=max_stop_r,
            )

            # Place agent at a navigable starting point reachable to all available target categories
            sim.pathfinder.seed(self.seed + 7)
            if start_hab is not None:
                init_pt = np.asarray(start_hab, dtype=np.float32)
            else:
                avail_cats = [c for c in target_cats if private_gt.get(c)]
                init_pt = np.asarray(sim.pathfinder.get_random_navigable_point(), dtype=np.float32)
                for _ in range(40):
                    cand_pt = np.asarray(sim.pathfinder.get_random_navigable_point(), dtype=np.float32)
                    all_reach = True
                    for cat in avail_cats:
                        gts = private_gt.get(cat, [])
                        _, d_geo, _, _ = self._eval_gt_proximity(sim, cand_pt, gts, max_stop_r)
                        if not math.isfinite(d_geo):
                            all_reach = False
                            break
                    if all_reach:
                        init_pt = cand_pt
                        break

            st0 = sim.agents[0].get_state()
            st0.position = init_pt
            st0.rotation = quaternion.from_rotation_vector([0.0, 0.0, 0.0])
            sim.agents[0].set_state(st0)

            memory = HierarchicalMemoryStore(
                KeyframeSceneGraph(), default_stop_radius_m=self.stop_radius_m
            )
            occ_grid = DepthOccupancyGrid()
            subtask_reports: list[dict[str, Any]] = []

            for sub_idx, subtask in enumerate(subtasks, start=1):
                if independent_subtasks and sub_idx > 1:
                    # Cold-start independent episode: re-sample navigable start & reset memory/map
                    memory = HierarchicalMemoryStore(
                        KeyframeSceneGraph(), default_stop_radius_m=subtask.stop_radius_m
                    )
                    occ_grid = DepthOccupancyGrid()
                    cat_gts = private_gt.get(subtask.target_category.strip().lower(), [])
                    sub_init_pt = init_pt
                    for _ in range(40):
                        cand_pt = np.asarray(
                            sim.pathfinder.get_random_navigable_point(), dtype=np.float32
                        )
                        if not cat_gts:
                            sub_init_pt = cand_pt
                            break
                        _, d_g, _, _ = self._eval_gt_proximity(
                            sim, cand_pt, cat_gts, subtask.stop_radius_m
                        )
                        if math.isfinite(d_g) and d_g >= 1.5:
                            sub_init_pt = cand_pt
                            break
                    st_r = sim.agents[0].get_state()
                    st_r.position = sub_init_pt
                    st_r.rotation = quaternion.from_rotation_vector([0.0, 0.0, 0.0])
                    sim.agents[0].set_state(st_r)
                else:
                    if not self.memory_carryover and sub_idx > 1:
                        memory = HierarchicalMemoryStore(
                            KeyframeSceneGraph(), default_stop_radius_m=subtask.stop_radius_m
                        )
                    if self.reset_occupancy_per_subtask and sub_idx > 1:
                        occ_grid = DepthOccupancyGrid()

                gt_list = private_gt.get(subtask.target_category.strip().lower(), [])
                _, _, sub_start_hab = self._agent_pose_mem(sim)
                _, optimal_geo_m, _, _ = self._eval_gt_proximity(
                    sim, sub_start_hab, gt_list, subtask.stop_radius_m
                )
                if not gt_list or not math.isfinite(optimal_geo_m):
                    subtask_reports.append(
                        {
                            "subtask_id": subtask.subtask_id,
                            "target_category": subtask.target_category,
                            "valid_gt": False,
                            "skipped_reason": "no_reachable_gt",
                            "sr": 0.0,
                            "first_stop_sr": 0.0,
                            "osr": 0.0,
                            "spl": 0.0,
                            "optimal_geodesic_m": None,
                            "traveled_m": 0.0,
                            "steps_used": 0,
                            "memory_recalls": 0,
                            "fallbacks": 0,
                            "memory_diagnostics": memory.diagnostic_metrics(),
                            "steps": [],
                        }
                    )
                    continue

                # Safe directory naming independent of user-supplied category text
                sub_dir = self.output_dir / f"subtask_{sub_idx:02d}"
                sub_dir.mkdir(parents=True, exist_ok=True)
                traveled_m = 0.0
                primitives_used = 0
                max_prim_budget = max(1, int(subtask.max_primitive_steps))
                oracle_reached = False
                stop_verified_gt = False
                first_stop_attempted = False
                first_stop_success = False
                memory_recalls = 0
                l3_selective_triggers = 0
                l3_bypassed = 0
                fallbacks = 0
                last_feedback = (
                    f"Started subtask {sub_idx}/{len(subtasks)}: find '{subtask.target_category}' "
                    f"(budget: {max_prim_budget} atomic steps)."
                )
                pending_recall: dict[str, Any] | None = None
                extra_l3_rgb: list[str] = []
                step_traces: list[dict[str, Any]] = []

                # Scene-Q confidence-aware coarse-to-fine search at subtask start when memory carries over
                if (
                    not independent_subtasks
                    and self.memory_carryover
                    and sub_idx > 1
                    and memory.graph.keyframe_poses
                ):
                    _, _, init_sub_hab = self._agent_pose_mem(sim)
                    init_sub_mem = list(hab_to_mem(init_sub_hab))
                    q_emb_init = (
                        self.vision_client.embed_text(f"a photo of a {subtask.target_category}")
                        if (self.vision_client is not None and vision_has_embed)
                        else None
                    )
                    c2f_init = memory.coarse_to_fine_search(
                        subtask.target_category,
                        robot_xyz=init_sub_mem,
                        query_embedding=q_emb_init,
                        top_k_objects=4,
                        top_k_keyframes=3,
                        max_l3_images=2,
                    )
                    def_hits_init, def_rgbs_init = self._deferred_vision_scan_keyframes(
                        subtask.target_category,
                        c2f_init["top_keyframes"],
                        has_seg=vision_has_seg,
                        max_kfs=2,
                    )
                    total_deferred_hits += len(def_hits_init)
                    if c2f_init["top_objects"] or c2f_init["top_keyframes"] or def_hits_init:
                        # Strip host filesystem paths before passing recalled memory to the planner
                        safe_top_objs = [
                            {k: v for k, v in o.items() if k != "rgb_path"}
                            for o in c2f_init["top_objects"][:4]
                        ]
                        pending_recall = {
                            "mode": "coarse_to_fine_init",
                            "untrusted_observation_data": True,
                            "query": subtask.target_category,
                            "top1_prob": c2f_init["top1_prob"],
                            "confidence_margin": c2f_init["confidence_margin"],
                            "needs_l3_verification": c2f_init["needs_l3_verification"],
                            "top_objects": safe_top_objs,
                            "top_keyframes": [
                                {
                                    "keyframe_id": kf["keyframe_id"],
                                    "room_type": kf.get("room_type", ""),
                                    "scene_description": kf.get("scene_description", ""),
                                    "surfaces": kf.get("surfaces", []),
                                    "pose_xyz": kf["pose_xyz"],
                                    "match_reasons": kf.get("match_reasons", []),
                                }
                                for kf in c2f_init["top_keyframes"][:3]
                            ],
                        }
                        if def_hits_init:
                            pending_recall["deferred_vision_hits"] = def_hits_init
                        merged_l3 = list(
                            dict.fromkeys([*def_rgbs_init, *c2f_init["l3_verify_rgb_paths"]])
                        )
                        if (c2f_init["needs_l3_verification"] or def_rgbs_init) and merged_l3:
                            extra_l3_rgb = merged_l3[:2]
                            l3_selective_triggers += 1
                        else:
                            l3_bypassed += 1

                for step in range(1, int(subtask.max_steps or 25) + 1):
                    if primitives_used >= max_prim_budget:
                        break
                    robot_mem, yaw_rad, robot_hab = self._agent_pose_mem(sim)
                    obs = sim.get_sensor_observations()
                    rgb = obs["color_sensor"][:, :, :3]
                    depth = obs["depth_sensor"]
                    occ_grid.update(depth, robot_mem, yaw_rad)

                    # Check private Oracle proximity (NavMesh geodesic to unoccluded viewpoint + stop_radius)
                    euc_gt, geo_gt, reached_gt, nearest_gt = self._eval_gt_proximity(
                        sim, robot_hab, gt_list, subtask.stop_radius_m
                    )
                    if reached_gt:
                        oracle_reached = True

                    rgb_path = sub_dir / f"step_{step:02d}_rgb.png"
                    Image.fromarray(rgb).save(rgb_path)

                    # Register current keyframe with raw RGB archive (never polluted by visual overlays)
                    kf_seq = memory.allocate_step_seq()
                    kf_id = f"hab_s{sub_idx}_step_{kf_seq:04d}"
                    memory.graph.add_keyframe(
                        kf_id,
                        _pose_matrix_zup(robot_mem, yaw_rad),
                        clearance_m=0.8,
                        rgb_path=str(rgb_path),
                    )
                    if len(memory.graph.keyframe_poses) >= 3:
                        memory.graph.segment_floors()
                        memory.graph.partition_rooms()

                    # Optional Layer-2 local vision foundation model tools (SAM 3 / YOLOE + SigLIP)
                    grounded_vdets: list[GroundedDetection] = []
                    vision_fused_step = 0
                    planner_rgb_path: Path = rgb_path
                    if self.vision_client is not None:
                        if vision_has_seg:
                            v_prompts = build_vision_prompts(
                                subtask.target_category,
                                max_prompts=self.vision_max_prompts,
                            )
                            raw_vdets = self.vision_client.segment_text(
                                rgb,
                                v_prompts,
                                score_threshold=self.vision_score_threshold,
                                max_detections=10,
                                include_masks=True,
                                include_crop_embeddings=vision_has_embed,
                            )
                            grounded_vdets = ground_detections_with_depth(raw_vdets, depth)
                            total_vision_dets += len(grounded_vdets)
                            for gd in grounded_vdets:
                                # Only fuse true pixel-mask backprojections into verified 3D memory;
                                # box-only fallbacks remain VLM prompt hints only.
                                if (
                                    gd.range_source == "vision_mask_depth_backproj"
                                    and gd.score >= self.vision_fuse_threshold
                                    and gd.valid_pixels >= 30
                                    and gd.surface_distance_m <= 8.0
                                ):
                                    memory.add_or_fuse_object(
                                        label=gd.label,
                                        keyframe_id=kf_id,
                                        local_xyz=list(gd.local_xyz),
                                        local_extent_xyz=gd.extent_xyz_m,
                                        confidence=gd.score,
                                        producer=f"{gd.backend}_depth_grounded",
                                        range_source=gd.range_source,
                                        evidence_frame="habitat_world",
                                        evidence_stamp_s=float(kf_seq),
                                        embedding=gd.embedding,
                                    )
                                    vision_fused_step += 1
                                    total_vision_fused += 1
                            if self.vision_enhance_overlay and grounded_vdets:
                                planner_rgb_path = render_visual_enhancement_overlay(
                                    rgb,
                                    grounded_vdets,
                                    sub_dir / f"step_{step:02d}_rgb_vis.png",
                                    target_label=subtask.target_category,
                                )
                        if vision_has_embed:
                            kf_emb = self.vision_client.embed_image(rgb)
                            if kf_emb is not None:
                                memory.annotate_keyframe(
                                    kf_id,
                                    embedding=kf_emb,
                                    wall_s=float(kf_seq),
                                )

                    f_cands = occ_grid.extract_frontiers(robot_mem, max_candidates=6)
                    m_cands = memory.memory_goal_candidates(
                        robot_mem,
                        target_label=subtask.target_category,
                        standoff_m=min(1.0, subtask.stop_radius_m * 0.5),
                    )
                    allowed_cands, _blocked = memory.filter_candidates(
                        [*m_cands, *f_cands], default_z=robot_mem[2]
                    )
                    allowed_m = [c for c in allowed_cands if c.get("kind") == "memory_target"]
                    map_path = occ_grid.render_local_map(
                        sub_dir / f"step_{step:02d}_map.png",
                        robot_mem,
                        yaw_rad,
                        allowed_cands,
                    )
                    mem_summary = memory.narrative_summary(
                        robot_xyz=robot_mem,
                        target_label=subtask.target_category,
                        memory_candidates=allowed_m,
                    )

                    decision: TaskDecision = self.planner.decide(
                        task_instruction=subtask.instruction,
                        step_index=step,
                        robot_xyz=robot_mem,
                        yaw_rad=yaw_rad,
                        candidates=allowed_cands,
                        memory_summary=mem_summary,
                        memory_l0=memory.l0_global_topology(),
                        target_label=subtask.target_category,
                        stop_radius_m=subtask.stop_radius_m,
                        estimated_travel_m=traveled_m,
                        last_feedback=last_feedback,
                        tool_recall_result=pending_recall,
                        vision_detections=[g.to_prompt_dict() for g in grounded_vdets],
                        rgb_path=planner_rgb_path,
                        local_map_path=map_path,
                        extra_rgb_paths=extra_l3_rgb,
                    )
                    pending_recall = None
                    extra_l3_rgb = []
                    if decision.fallback:
                        fallbacks += 1

                    # Merge VLM rich keyframe annotations + depth-grounded landmarks onto kf_id
                    if decision.room_type or decision.scene_description or decision.visible_surfaces:
                        memory.annotate_keyframe(
                            kf_id,
                            room_type=decision.room_type,
                            scene_description=decision.scene_description,
                            surfaces=(decision.visible_surfaces if decision.visible_surfaces else None),
                            wall_s=float(kf_seq),
                        )

                    grounded_count = vision_fused_step
                    for item in decision.observed_objects:
                        b_deg = float(item.get("bearing_deg", 0.0))
                        patch_res = measure_depth_bearing_patch(depth, b_deg)
                        if patch_res is not None:
                            d_meas, ext_xyz = patch_res
                            lx, ly = depth_bearing_to_local_xy(d_meas, b_deg)
                            prod = "vlm_depth_grounded"
                            r_src = "habitat_depth_raycast"
                            grounded_count += 1
                        else:
                            dist_use = float(item.get("dist_m", 2.0))
                            b_rad = math.radians(b_deg)
                            lx, ly = dist_use * math.cos(b_rad), dist_use * math.sin(b_rad)
                            ext_xyz = (0.0, 0.0, 0.0)
                            prod = "opencode_vlm_unverified"
                            r_src = "vlm_unverified_guess"
                        memory.add_or_fuse_object(
                            label=str(item["label"]),
                            keyframe_id=kf_id,
                            local_xyz=[lx, ly, 0.0],
                            local_extent_xyz=ext_xyz,
                            confidence=float(item.get("confidence", 0.8)),
                            producer=prod,
                            range_source=r_src,
                            evidence_frame="habitat_world",
                            evidence_stamp_s=float(kf_seq),
                            description=str(item.get("desc", "")),
                            attributes=tuple(item.get("attrs", ())),
                            spatial_context=str(item.get("on", "")),
                        )

                    exec_info: dict[str, Any] = {}
                    if decision.action == "noop":
                        exec_info = {"status": "noop", "reason": decision.reason}
                        last_feedback = f"Planner noop ({decision.reason}); holding."
                    elif decision.action == "query_memory":
                        memory_recalls += 1
                        q = decision.query or subtask.target_category
                        q_l1 = memory.l1_submap_summaries(query=q, near_xyz=robot_mem, top_k=3)
                        q_emb = (
                            self.vision_client.embed_text(f"a photo of a {q}")
                            if (self.vision_client is not None and vision_has_embed)
                            else None
                        )
                        c2f = memory.coarse_to_fine_search(
                            q,
                            robot_xyz=robot_mem,
                            query_embedding=q_emb,
                            top_k_objects=4,
                            top_k_keyframes=3,
                            max_l3_images=2,
                        )
                        def_hits_q, def_rgbs_q = self._deferred_vision_scan_keyframes(
                            q,
                            c2f["top_keyframes"],
                            has_seg=vision_has_seg,
                            max_kfs=2,
                        )
                        total_deferred_hits += len(def_hits_q)
                        merged_l3_q = list(
                            dict.fromkeys([*def_rgbs_q, *c2f["l3_verify_rgb_paths"]])
                        )
                        if (c2f["needs_l3_verification"] or def_rgbs_q) and merged_l3_q:
                            extra_l3_rgb = merged_l3_q[:2]
                            l3_selective_triggers += 1
                        else:
                            l3_bypassed += 1
                        safe_top_objs = [
                            {k: v for k, v in o.items() if k != "rgb_path"}
                            for o in c2f["top_objects"][:4]
                        ]
                        pending_recall = {
                            "mode": "coarse_to_fine_query",
                            "untrusted_observation_data": True,
                            "query": q,
                            "top1_prob": c2f["top1_prob"],
                            "confidence_margin": c2f["confidence_margin"],
                            "needs_l3_verification": c2f["needs_l3_verification"],
                            "l1_rooms": q_l1[:3],
                            "top_objects": safe_top_objs,
                            "top_keyframes": [
                                {
                                    "keyframe_id": kf["keyframe_id"],
                                    "room_type": kf.get("room_type", ""),
                                    "scene_description": kf.get("scene_description", ""),
                                    "surfaces": kf.get("surfaces", []),
                                    "pose_xyz": kf["pose_xyz"],
                                    "match_reasons": kf.get("match_reasons", []),
                                }
                                for kf in c2f["top_keyframes"][:3]
                            ],
                        }
                        if def_hits_q:
                            pending_recall["deferred_vision_hits"] = def_hits_q
                        exec_info = {
                            "status": "memory_queried",
                            "l2_hits": len(c2f["top_objects"]),
                            "kf_hits": len(c2f["top_keyframes"]),
                            "deferred_vision_hits": len(def_hits_q),
                            "needs_l3_verification": c2f["needs_l3_verification"],
                        }
                        last_feedback = (
                            f"query_memory('{q}') -> {len(c2f['top_objects'])} 3D objs, "
                            f"{len(c2f['top_keyframes'])} keyframes, "
                            f"{len(def_hits_q)} deferred_vision_hits "
                            f"(L3_attached={len(extra_l3_rgb)})."
                        )
                    elif decision.action == "stop_claim":
                        is_first_stop = not first_stop_attempted
                        first_stop_attempted = True
                        if (
                            decision.target_label is not None
                            and not _label_strict_match(subtask.target_category, decision.target_label)
                        ):
                            exec_info = {"allowed": False, "reason": "target_label_mismatch"}
                            last_feedback = f"STOP rejected: claimed '{decision.target_label}' != '{subtask.target_category}'."
                        else:
                            ver = memory.verify_stop_claim(
                                robot_mem,
                                target_label=subtask.target_category,
                                stop_radius_m=subtask.stop_radius_m,
                                require_current_keyframe_id=kf_id,
                                excluded_producers=("opencode_vlm", "opencode_vlm_unverified"),
                            )
                            exec_info = ver.to_dict()
                            exec_info["private_gt_euc_m"] = (
                                None if not math.isfinite(euc_gt) else round(euc_gt, 3)
                            )
                            exec_info["private_gt_geo_m"] = (
                                None if not math.isfinite(geo_gt) else round(geo_gt, 3)
                            )
                            if ver.allowed:
                                # Agent committed to STOP using sensor-only evidence.
                                # Terminate subtask immediately (never leak private GT feedback).
                                gt_ok = bool(reached_gt)
                                exec_info["gt_verified"] = gt_ok
                                if gt_ok:
                                    stop_verified_gt = True
                                    oracle_reached = True
                                    if is_first_stop:
                                        first_stop_success = True
                                else:
                                    exec_info["reason"] = "gt_target_not_within_stop_radius"
                                step_traces.append(
                                    {
                                        "step": step,
                                        "robot_mem": [round(v, 2) for v in robot_mem],
                                        "action": decision.action,
                                        "target_id": decision.target_id,
                                        "reason": decision.reason,
                                        "grounded_obs": grounded_count,
                                        "gt_euc_m": (
                                            None if not math.isfinite(euc_gt) else round(euc_gt, 3)
                                        ),
                                        "gt_geo_m": (
                                            None if not math.isfinite(geo_gt) else round(geo_gt, 3)
                                        ),
                                        "latency_ms": round(decision.latency_ms, 1),
                                        "execution": exec_info,
                                    }
                                )
                                break
                            else:
                                if ver.reason == "requires_current_frame_visual_confirmation":
                                    last_feedback = (
                                        f"STOP rejected ({ver.reason}, dist={ver.distance_m}): "
                                        f"target '{subtask.target_category}' was only in old memory; "
                                        "turn/scan or move so it is directly visible in the CURRENT front RGB and list it in observed_objects before calling stop_claim."
                                    )
                                else:
                                    last_feedback = f"STOP rejected ({ver.reason}, dist={ver.distance_m}). Move closer."
                    elif decision.action in ("goto_candidate", "goto_xyz"):
                        goal_mem: list[float] | None = None
                        if decision.action == "goto_candidate":
                            for c in allowed_cands:
                                if c.get("id") == decision.target_id:
                                    xy = c.get("xy")
                                    if isinstance(xy, (list, tuple)) and len(xy) >= 2:
                                        goal_mem = [float(xy[0]), float(xy[1]), float(c.get("z", robot_mem[2]))]
                                    break
                        elif decision.xyz is not None:
                            goal_mem = [float(v) for v in decision.xyz]

                        if goal_mem is None:
                            exec_info = {"status": "skipped", "reason": "unresolved_goal"}
                            last_feedback = "Unresolved goto goal."
                        elif memory.match_blacklist(goal_mem):
                            exec_info = {"status": "blacklisted_goal"}
                            last_feedback = "Goal blocked by failure blacklist; pick another candidate."
                        else:
                            rem_prims = max(1, max_prim_budget - primitives_used)
                            exec_info = self._step_toward_goal(
                                sim, occ_grid, goal_mem, max_primitives=min(24, rem_prims)
                            )
                            traj_hab = exec_info.pop("_traj_hab", [])
                            primitives_used += int(exec_info.get("primitives", 0))
                            traveled_m += float(exec_info.get("traveled_m", 0.0))
                            memory.record_attempt(
                                "goto",
                                goal_mem,
                                str(exec_info.get("status", "unknown")),
                                reason=str(exec_info.get("reason", "")),
                                keyframe_id=kf_id,
                            )
                            if not oracle_reached:
                                for pt_h in traj_hab:
                                    _, _, r_pt, _ = self._eval_gt_proximity(
                                        sim, pt_h, gt_list, subtask.stop_radius_m
                                    )
                                    if r_pt:
                                        oracle_reached = True
                                        break
                            if (
                                exec_info.get("status") == "completed"
                                and int(exec_info.get("primitives", 0)) == 0
                                and float(exec_info.get("traveled_m", 0.0)) < 0.05
                            ):
                                last_feedback = (
                                    f"Already at goal {[round(v, 2) for v in goal_mem]} (0 steps needed). "
                                    "Do NOT repeat the same goto_candidate; if target is visible within stop_radius call stop_claim, "
                                    "otherwise scan or explore a different frontier F*."
                                )
                            else:
                                last_feedback = (
                                    f"goto {[round(v, 2) for v in goal_mem]} -> "
                                    f"{exec_info['status']} (+{exec_info['traveled_m']}m, "
                                    f"atomic_steps={primitives_used}/{max_prim_budget})."
                                )
                    elif decision.action in ("scan", "maneuver"):
                        # Turn 60 deg (4 x 15 deg) or short forward step using real primitives
                        rem_prims = max(0, max_prim_budget - primitives_used)
                        req_prims = 4 if decision.action == "scan" else max(1, min(6, int(round(decision.duration * 2))))
                        prims = min(req_prims, rem_prims)
                        act_name = "turn_left" if (decision.action == "scan" or decision.yaw_rate >= 0.0) else "turn_right"
                        if decision.action == "maneuver" and decision.forward > 0.15:
                            act_name = "move_forward"
                        t_step = 0.0
                        _, _, prev_h = self._agent_pose_mem(sim)
                        for _ in range(prims):
                            o_s = sim.step(act_name)
                            primitives_used += 1
                            cm, cy, ch = self._agent_pose_mem(sim)
                            t_step += float(np.linalg.norm(ch - prev_h))
                            prev_h = ch
                            occ_grid.update(o_s["depth_sensor"], cm, cy)
                            if not oracle_reached:
                                _, _, r_pt, _ = self._eval_gt_proximity(
                                    sim, ch, gt_list, subtask.stop_radius_m
                                )
                                if r_pt:
                                    oracle_reached = True
                        traveled_m += t_step
                        exec_info = {
                            "status": "completed",
                            "action": act_name,
                            "primitives": prims,
                            "traveled_m": round(t_step, 3),
                        }
                        last_feedback = (
                            f"{decision.action} ({act_name} x{prims}) completed "
                            f"(atomic_steps={primitives_used}/{max_prim_budget})."
                        )

                    print(
                        f"[Habitat sub {sub_idx}:{subtask.target_category} | step {step}] "
                        f"pos={[round(v, 2) for v in robot_mem]} act={decision.action} "
                        f"tgt={decision.target_id or decision.target_label} obs={grounded_count} "
                        f"gt_geo={round(geo_gt, 2) if math.isfinite(geo_gt) else 'inf'}m "
                        f"llm={decision.latency_ms:.0f}ms -> {exec_info.get('status', exec_info.get('reason'))}",
                        flush=True,
                    )
                    trace_entry: dict[str, Any] = {
                        "step": step,
                        "robot_mem": [round(v, 2) for v in robot_mem],
                        "action": decision.action,
                        "target_id": decision.target_id,
                        "reason": decision.reason,
                        "grounded_obs": grounded_count,
                        "gt_euc_m": (None if not math.isfinite(euc_gt) else round(euc_gt, 3)),
                        "gt_geo_m": (None if not math.isfinite(geo_gt) else round(geo_gt, 3)),
                        "latency_ms": round(decision.latency_ms, 1),
                        "execution": exec_info,
                    }
                    if decision.fallback:
                        trace_entry["fallback_raw_preview"] = decision.raw_text[:200]
                    step_traces.append(trace_entry)

                sr = 1.0 if stop_verified_gt else 0.0
                first_stop_sr = 1.0 if first_stop_success else 0.0
                osr = 1.0 if oracle_reached else 0.0
                spl = compute_spl(stop_verified_gt, optimal_geo_m, traveled_m)
                subtask_reports.append(
                    {
                        "subtask_id": subtask.subtask_id,
                        "target_category": subtask.target_category,
                        "valid_gt": True,
                        "sr": sr,
                        "first_stop_sr": first_stop_sr,
                        "osr": osr,
                        "spl": round(spl, 4),
                        "optimal_geodesic_m": round(optimal_geo_m, 2),
                        "traveled_m": round(traveled_m, 2),
                        "steps_used": len(step_traces),
                        "primitives_used": primitives_used,
                        "max_primitive_steps": max_prim_budget,
                        "memory_recalls": memory_recalls,
                        "l3_selective_triggers": l3_selective_triggers,
                        "l3_bypassed": l3_bypassed,
                        "fallbacks": fallbacks,
                        "memory_diagnostics": memory.diagnostic_metrics(),
                        "steps": step_traces,
                    }
                )

            valid_reports = [r for r in subtask_reports if r.get("valid_gt", True)]
            invalid_count = len(subtask_reports) - len(valid_reports)
            denom = max(1, len(valid_reports))
            summary = {
                "scene_id": self.scene_id,
                "eval_mode": ("independent_cold_start" if independent_subtasks else "sequential_multi_task"),
                "navigation_executor": "habitat_navmesh_greedy_geodesic_follower",
                "stop_protocol": "current_frame_confirmed_sensor_gate_with_first_stop_tracking",
                "retrieval_protocol": "scene_q_rich_index_coarse_to_fine_selective_l3",
                "memory_carryover": (False if independent_subtasks else self.memory_carryover),
                "occupancy_carryover": (False if independent_subtasks else not self.reset_occupancy_per_subtask),
                "vision_tools": {
                    "enabled": bool(self.vision_client is not None and vision_ok),
                    "backend": str(vision_health.get("backend", "none")),
                    "embed_model": str(vision_health.get("embed_model", "")),
                    "enhance_overlay": self.vision_enhance_overlay,
                    "deferred_l3": self.vision_deferred_l3,
                    "total_detections": total_vision_dets,
                    "total_fused": total_vision_fused,
                    "total_deferred_hits": total_deferred_hits,
                },
                "planner_config": self.planner.config.to_safe_dict(),
                "subtasks_count": len(subtask_reports),
                "valid_subtasks_count": len(valid_reports),
                "invalid_subtasks_count": invalid_count,
                "sr": round(sum(r["sr"] for r in valid_reports) / denom, 4) if valid_reports else 0.0,
                "first_stop_sr": (
                    round(sum(r["first_stop_sr"] for r in valid_reports) / denom, 4)
                    if valid_reports
                    else 0.0
                ),
                "osr": round(sum(r["osr"] for r in valid_reports) / denom, 4) if valid_reports else 0.0,
                "spl": round(sum(r["spl"] for r in valid_reports) / denom, 4) if valid_reports else 0.0,
                "total_traveled_m": round(sum(r["traveled_m"] for r in valid_reports), 2),
                "total_steps": sum(r["steps_used"] for r in valid_reports),
                "total_primitives": sum(int(r.get("primitives_used", 0)) for r in valid_reports),
                "l3_selective_triggers": sum(r.get("l3_selective_triggers", 0) for r in valid_reports),
                "l3_bypassed": sum(r.get("l3_bypassed", 0) for r in valid_reports),
                "wall_s": round(time.perf_counter() - t_ep, 2),
                "private_gt_counts": {k: len(v) for k, v in private_gt.items()},
                "subtasks": subtask_reports,
            }
            memory.save(self.output_dir / "habitat_hierarchical_memory.json")
            atomic_write_json(self.output_dir / "habitat_eval_report.json", summary)
            return summary
        finally:
            sim.close()
