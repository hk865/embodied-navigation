"""Keyframe-anchored hierarchical 3D scene graph.

Pure CPU (stdlib + numpy + scipy.spatial.cKDTree) representation linking SLAM
keyframes to floors, stairs, rooms, doorways, and objects so loop-closure pose
updates reproject all semantic entities deterministically without duplicating
geometry.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from scipy.spatial import cKDTree

from .core import validate_rigid_pose

SCHEMA = "keyframe_scene_graph/v1"


@dataclass
class FloorNode:
    floor_id: str
    level_index: int
    z_mean_m: float
    z_min_m: float
    z_max_m: float
    keyframe_ids: list[str]


@dataclass
class StairEdge:
    edge_id: str
    from_floor: str
    to_floor: str
    keyframe_ids: list[str]
    start_xyz: list[float]
    end_xyz: list[float]


@dataclass
class RoomNode:
    room_id: str
    floor_id: str
    alias: str
    anchor_keyframe_ids: list[str]
    representative_rgb_paths: list[str]
    centroid_xyz: list[float]
    bounds_min_xyz: list[float]
    bounds_max_xyz: list[float]
    clearance_m: float
    merged_aliases: list[str] = field(default_factory=list)


@dataclass
class DoorwayEdge:
    room_a: str
    room_b: str
    gateway_xyz: list[float]
    clearance_m: float
    bottleneck_anchor_ids: list[str] = field(default_factory=list)


@dataclass
class ObjectNode:
    object_id: str
    label: str
    anchor_keyframe_id: str
    local_center_xyz: list[float]
    local_extent_xyz: list[float]
    world_center_xyz: list[float]
    confidence: float
    producer: str
    room_id: str | None = None


def _validate_pose(matrix: Any) -> np.ndarray:
    """Validate finite 4x4 SE(3) rigid transform and return writable float64 copy."""
    return np.array(validate_rigid_pose(matrix), dtype=np.float64, copy=True)


def _validate_vec3(values: Any, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != (3,) or not np.isfinite(arr).all():
        raise ValueError(f"{name} must be a finite 3D vector")
    return arr


def _validate_clearance(clearance_m: Any) -> float:
    if isinstance(clearance_m, bool):
        raise ValueError("clearance_m must be finite and nonnegative")
    val = float(clearance_m)
    if not math.isfinite(val) or val < 0.0:
        raise ValueError("clearance_m must be finite and nonnegative")
    return val


def _validate_sector_clearances(values: Any) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.ndim != 1 or arr.size == 0:
        raise ValueError("sector_clearances must be a non-empty 1D array")
    if np.any(np.isnan(arr)) or np.any(arr < 0.0):
        raise ValueError("sector_clearances must be nonnegative without NaN")
    return np.array(arr, dtype=np.float64, copy=True)


def _infer_epoch(keyframe_id: str, explicit_epoch: int | None) -> int | None:
    if explicit_epoch is not None:
        if isinstance(explicit_epoch, bool) or not isinstance(explicit_epoch, int):
            raise ValueError("epoch must be an integer")
        return explicit_epoch
    prefix, sep, suffix = keyframe_id.partition(":")
    if sep == ":" and prefix.isdigit() and suffix.isdigit():
        return int(prefix)
    return None


def _dedup_ordered(items: Sequence[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for item in items:
        if item not in seen:
            seen.add(item)
            result.append(item)
    return result


def _segment_blocked_by_point(
    p_u: np.ndarray,
    p_v: np.ndarray,
    p_b: np.ndarray,
    clearance_radius: float,
) -> bool:
    uv = p_v - p_u
    len_sq = float(np.dot(uv, uv))
    if len_sq < 1e-12:
        return False
    t = float(np.dot(p_b - p_u, uv) / len_sq)
    if not (0.02 < t < 0.98):
        return False
    proj = p_u + t * uv
    return float(np.linalg.norm(p_b - proj)) <= clearance_radius


def _point_to_segment_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    len_sq = float(np.dot(ab, ab))
    if len_sq < 1e-12:
        return float(np.linalg.norm(p - a))
    t = max(0.0, min(1.0, float(np.dot(p - a, ab) / len_sq)))
    proj = a + t * ab
    return float(np.linalg.norm(p - proj))


def _dist_to_polyline(point: Sequence[float], vertices: Sequence[Sequence[float]]) -> float:
    if not vertices:
        return float("inf")
    p = np.asarray(point, dtype=np.float64)
    pts = [np.asarray(v, dtype=np.float64) for v in vertices]
    if len(pts) == 1:
        return float(np.linalg.norm(p - pts[0]))
    return min(_point_to_segment_dist(p, pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def _bbox_2d_iou(
    min_a: Sequence[float],
    max_a: Sequence[float],
    min_b: Sequence[float],
    max_b: Sequence[float],
) -> float:
    ax0, ay0 = float(min_a[0]), float(min_a[1])
    ax1, ay1 = float(max_a[0]), float(max_a[1])
    bx0, by0 = float(min_b[0]), float(min_b[1])
    bx1, by1 = float(max_b[0]), float(max_b[1])

    # Pad degenerate 1D/0D room footprints so IoU remains meaningful.
    if ax1 - ax0 < 1e-6 or bx1 - bx0 < 1e-6:
        ax0, ax1 = ax0 - 0.5, ax1 + 0.5
        bx0, bx1 = bx0 - 0.5, bx1 + 0.5
    if ay1 - ay0 < 1e-6 or by1 - by0 < 1e-6:
        ay0, ay1 = ay0 - 0.5, ay1 + 0.5
        by0, by1 = by0 - 0.5, by1 + 0.5

    ix = max(0.0, min(ax1, bx1) - max(ax0, bx0))
    iy = max(0.0, min(ay1, by1) - max(ay0, by0))
    inter = ix * iy
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, ax1 - ax0) * max(0.0, ay1 - ay0)
    area_b = max(0.0, bx1 - bx0) * max(0.0, by1 - by0)
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0


def _dist_to_aabb(point: Sequence[float], bmin: Sequence[float], bmax: Sequence[float]) -> float:
    dx = max(bmin[0] - point[0], 0.0, point[0] - bmax[0])
    dy = max(bmin[1] - point[1], 0.0, point[1] - bmax[1])
    dz = max(bmin[2] - point[2], 0.0, point[2] - bmax[2])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _select_representative_rgbs(
    keyframe_ids: Sequence[str],
    rgb_paths: dict[str, str],
    max_rep_images: int,
) -> list[str]:
    unique_paths = _dedup_ordered(
        [rgb_paths[k] for k in keyframe_ids if k in rgb_paths and rgb_paths[k]]
    )
    if max_rep_images <= 0:
        return []
    if len(unique_paths) <= max_rep_images:
        return unique_paths
    indices = np.linspace(0, len(unique_paths) - 1, max_rep_images, dtype=int)
    return [unique_paths[int(i)] for i in indices]


class KeyframeSceneGraph:
    """Hierarchical 3D scene graph anchored to SLAM keyframe poses."""

    def __init__(
        self,
        *,
        keyframe_poses: dict[str, np.ndarray] | None = None,
        keyframe_clearances: dict[str, float] | None = None,
        keyframe_rgb_paths: dict[str, str] | None = None,
        keyframe_stamps: dict[str, int] | None = None,
        keyframe_epochs: dict[str, int] | None = None,
        keyframe_sector_clearances: dict[str, Sequence[float]] | None = None,
        anchor_to_keyframe_map: dict[str, str] | None = None,
        anchor_extrinsics: dict[str, Any] | None = None,
    ) -> None:
        self.keyframe_poses: dict[str, np.ndarray] = {}
        if keyframe_poses:
            for kf_id, pose in keyframe_poses.items():
                self.keyframe_poses[str(kf_id)] = _validate_pose(pose)

        self.keyframe_clearances: dict[str, float] = {}
        if keyframe_clearances:
            for kf_id, clr in keyframe_clearances.items():
                if clr is not None:
                    self.keyframe_clearances[str(kf_id)] = _validate_clearance(clr)

        self.keyframe_rgb_paths: dict[str, str] = (
            {str(k): str(v) for k, v in keyframe_rgb_paths.items()}
            if keyframe_rgb_paths
            else {}
        )
        self.keyframe_stamps: dict[str, int] = {}
        if keyframe_stamps:
            for kf_id, stamp in keyframe_stamps.items():
                if stamp is not None:
                    self.keyframe_stamps[str(kf_id)] = int(stamp)

        self.keyframe_epochs: dict[str, int] = {}
        if keyframe_epochs:
            for kf_id, ep in keyframe_epochs.items():
                if ep is not None:
                    self.keyframe_epochs[str(kf_id)] = int(ep)

        self.keyframe_sector_clearances: dict[str, np.ndarray] = {}
        if keyframe_sector_clearances:
            for kf_id, sectors in keyframe_sector_clearances.items():
                if sectors is not None:
                    self.keyframe_sector_clearances[str(kf_id)] = _validate_sector_clearances(sectors)

        self.anchor_to_keyframe_map: dict[str, str] = (
            {str(k): str(v) for k, v in anchor_to_keyframe_map.items()}
            if anchor_to_keyframe_map
            else {}
        )
        self.anchor_extrinsics: dict[str, np.ndarray] = {}
        if anchor_extrinsics:
            for anchor_id, T in anchor_extrinsics.items():
                self.anchor_extrinsics[str(anchor_id)] = _validate_pose(T)

        self.floors: dict[str, FloorNode] = {}
        self.stairs: list[StairEdge] = []
        self.rooms: dict[str, RoomNode] = {}
        self.doorways: list[DoorwayEdge] = []
        self.objects: dict[str, ObjectNode] = {}
        self._next_object_seq: int = 1
        self._segmented_keyframe_ids: set[str] = set()
        self._floor_params: dict[str, Any] = {
            "z_bin_m": 0.25,
            "floor_sep_m": 1.8,
            "level_band_m": 0.65,
            "min_floor_keyframes": 3,
            "max_floor_slope": 0.03,
            "min_floor_support_ratio": 0.20,
            "max_trajectory_gap_s": 15.0,
        }
        self._room_params: dict[str, Any] = {
            "doorway_clearance_m": 0.85,
            "link_radius_m": 2.5,
            "min_room_keyframes": 2,
            "max_rep_images": 4,
        }

    def register_anchor_link(
        self,
        anchor_id: str,
        keyframe_id: str,
        T_keyframe_sensor: Any = None,
    ) -> None:
        if not isinstance(anchor_id, str) or not anchor_id:
            raise ValueError("anchor_id must be a non-empty string")
        if not isinstance(keyframe_id, str) or not keyframe_id:
            raise ValueError("keyframe_id must be a non-empty string")
        self.anchor_to_keyframe_map[anchor_id] = keyframe_id
        if T_keyframe_sensor is not None:
            self.anchor_extrinsics[anchor_id] = _validate_pose(T_keyframe_sensor)

    def add_keyframe(
        self,
        keyframe_id: str,
        T_world_body: np.ndarray,
        *,
        clearance_m: float | None = None,
        rgb_path: str | None = None,
        stamp_ns: int | None = None,
        epoch: int | None = None,
        sector_clearances: Sequence[float] | np.ndarray | None = None,
    ) -> None:
        if not isinstance(keyframe_id, str) or not keyframe_id:
            raise ValueError("keyframe_id must be a non-empty string")
        self.keyframe_poses[keyframe_id] = _validate_pose(T_world_body)
        if clearance_m is not None:
            self.keyframe_clearances[keyframe_id] = _validate_clearance(clearance_m)
        if rgb_path is not None:
            self.keyframe_rgb_paths[keyframe_id] = str(rgb_path)
        if stamp_ns is not None:
            if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns < 0:
                raise ValueError("stamp_ns must be a nonnegative integer")
            self.keyframe_stamps[keyframe_id] = stamp_ns
        inferred_epoch = _infer_epoch(keyframe_id, epoch)
        if inferred_epoch is not None:
            self.keyframe_epochs[keyframe_id] = inferred_epoch
        if sector_clearances is not None:
            self.keyframe_sector_clearances[keyframe_id] = _validate_sector_clearances(
                sector_clearances
            )

    def _continuous_segments(
        self, kf_ids: Sequence[str], max_trajectory_gap_s: float
    ) -> list[list[int]]:
        """Split ordered keyframe indices into continuous trajectory segments."""
        if not kf_ids:
            return []
        segments: list[list[int]] = [[0]]
        prev_epoch = _infer_epoch(kf_ids[0], self.keyframe_epochs.get(kf_ids[0]))
        prev_stamp = self.keyframe_stamps.get(kf_ids[0])
        for idx in range(1, len(kf_ids)):
            kf = kf_ids[idx]
            epoch = _infer_epoch(kf, self.keyframe_epochs.get(kf))
            stamp = self.keyframe_stamps.get(kf)
            new_seg = False
            if epoch is not None and prev_epoch is not None and epoch != prev_epoch:
                new_seg = True
            elif (epoch is None) != (prev_epoch is None):
                new_seg = True
            elif stamp is not None and prev_stamp is not None:
                dt_ns = stamp - prev_stamp
                if dt_ns < 0 or (dt_ns * 1e-9) > max_trajectory_gap_s:
                    new_seg = True
            if new_seg:
                segments.append([idx])
            else:
                segments[-1].append(idx)
            prev_epoch = epoch
            prev_stamp = stamp
        return segments

    def _plateau_sample_indices(
        self,
        kf_ids: Sequence[str],
        pts_xyz: np.ndarray,
        segments: Sequence[Sequence[int]],
        max_floor_slope: float,
    ) -> list[int]:
        """Return deduplicated moving-sample indices with low local slope |dz/ds|."""
        flat_indices: list[int] = []
        min_step_xy = 0.15
        slope_baseline_m = 1.0
        for seg in segments:
            if not seg:
                continue
            kept = [seg[0]]
            for idx in seg[1:]:
                if float(np.linalg.norm(pts_xyz[idx, :2] - pts_xyz[kept[-1], :2])) >= min_step_xy:
                    kept.append(idx)
            if len(kept) == 1:
                continue
            for pos, idx in enumerate(kept):
                lo = pos
                ds_b = 0.0
                while lo > 0 and ds_b < slope_baseline_m:
                    lo -= 1
                    ds_b += float(
                        np.linalg.norm(pts_xyz[kept[lo + 1], :2] - pts_xyz[kept[lo], :2])
                    )
                hi = pos
                ds_f = 0.0
                while hi < len(kept) - 1 and ds_f < slope_baseline_m:
                    hi += 1
                    ds_f += float(
                        np.linalg.norm(pts_xyz[kept[hi], :2] - pts_xyz[kept[hi - 1], :2])
                    )
                sb = (
                    abs(float(pts_xyz[idx, 2] - pts_xyz[kept[lo], 2])) / ds_b
                    if ds_b >= 0.5
                    else None
                )
                sf = (
                    abs(float(pts_xyz[kept[hi], 2] - pts_xyz[idx, 2])) / ds_f
                    if ds_f >= 0.5
                    else None
                )
                side_slopes = [s for s in (sb, sf) if s is not None]
                if not side_slopes:
                    continue
                if min(side_slopes) <= max_floor_slope:
                    flat_indices.append(idx)
        return flat_indices

    def segment_floors(
        self,
        z_bin_m: float = 0.25,
        floor_sep_m: float = 1.8,
        level_band_m: float = 0.65,
        *,
        min_floor_keyframes: int = 3,
        max_floor_slope: float = 0.03,
        min_floor_support_ratio: float = 0.20,
        max_trajectory_gap_s: float = 15.0,
    ) -> dict[str, FloorNode]:
        if z_bin_m <= 0.0 or floor_sep_m <= 0.0 or level_band_m <= 0.0:
            raise ValueError("z_bin_m, floor_sep_m, and level_band_m must be positive")
        if min_floor_keyframes < 1 or max_floor_slope < 0.0 or not (0.0 <= min_floor_support_ratio <= 1.0):
            raise ValueError("invalid floor plateau parameters")
        self._floor_params = {
            "z_bin_m": float(z_bin_m),
            "floor_sep_m": float(floor_sep_m),
            "level_band_m": float(level_band_m),
            "min_floor_keyframes": int(min_floor_keyframes),
            "max_floor_slope": float(max_floor_slope),
            "min_floor_support_ratio": float(min_floor_support_ratio),
            "max_trajectory_gap_s": float(max_trajectory_gap_s),
        }

        self.floors = {}
        self.stairs = []
        kf_ids = list(self.keyframe_poses.keys())
        self._segmented_keyframe_ids = set(kf_ids)
        if not kf_ids:
            return self.floors

        pts_xyz = np.array([self.keyframe_poses[k][:3, 3] for k in kf_ids], dtype=np.float64)
        zs = pts_xyz[:, 2]
        segments = self._continuous_segments(kf_ids, max_trajectory_gap_s)
        plateau_indices = self._plateau_sample_indices(kf_ids, pts_xyz, segments, max_floor_slope)

        peaks: list[float] = []
        if plateau_indices:
            plateau_zs = zs[plateau_indices]
            z_ref = float(np.min(plateau_zs))
            bin_indices = np.unique(np.floor((plateau_zs - z_ref) / z_bin_m).astype(np.int64))
            half_win = max(z_bin_m, 0.5 * level_band_m)

            # Refine every candidate peak first so NMS uses true medians (Z-shift invariant).
            candidates: list[tuple[int, float, float]] = []
            for b in bin_indices:
                center = z_ref + (float(b) + 0.5) * z_bin_m
                support = int(np.sum(np.abs(plateau_zs - center) <= half_win))
                in_band = plateau_zs[np.abs(plateau_zs - center) <= level_band_m]
                refined = float(np.median(in_band)) if len(in_band) > 0 else center
                candidates.append((support, -(refined - z_ref), refined))
            candidates.sort(reverse=True)

            max_support = candidates[0][0]
            min_support = max(
                min_floor_keyframes,
                int(math.ceil(min_floor_support_ratio * max_support)),
            )

            for support, _, refined in candidates:
                if support < min_support:
                    continue
                if any(abs(refined - p) < floor_sep_m for p in peaks):
                    continue
                peaks.append(refined)

        if not peaks:
            peaks = [float(np.median(zs))]
        peaks.sort()

        raw_assignments: list[int | None] = []
        for z in zs:
            diffs = [abs(float(z) - p) for p in peaks]
            best_idx = int(np.argmin(diffs))
            raw_assignments.append(best_idx if diffs[best_idx] <= level_band_m else None)

        active_peak_indices = sorted({idx for idx in raw_assignments if idx is not None})
        peak_to_floor_id: dict[int, str] = {}
        for level_idx, peak_idx in enumerate(active_peak_indices):
            floor_id = f"f{level_idx}"
            peak_to_floor_id[peak_idx] = floor_id
            member_indices = [i for i, a in enumerate(raw_assignments) if a == peak_idx]
            member_kfs = [kf_ids[i] for i in member_indices]
            member_zs = zs[member_indices]
            self.floors[floor_id] = FloorNode(
                floor_id=floor_id,
                level_index=level_idx,
                z_mean_m=float(np.mean(member_zs)),
                z_min_m=float(np.min(member_zs)),
                z_max_m=float(np.max(member_zs)),
                keyframe_ids=member_kfs,
            )

        floor_seq = [
            peak_to_floor_id[a] if a is not None else None for a in raw_assignments
        ]
        # Never bridge across epoch or time discontinuities.
        for seg in segments:
            last_floor: str | None = None
            last_floor_idx: int | None = None
            for idx in seg:
                fl_id = floor_seq[idx]
                if fl_id is None:
                    continue
                if last_floor is not None and fl_id != last_floor and last_floor_idx is not None:
                    seg_kfs = [kf_ids[k] for k in range(last_floor_idx, idx + 1)]
                    start_xyz = self.keyframe_poses[seg_kfs[0]][:3, 3].tolist()
                    end_xyz = self.keyframe_poses[seg_kfs[-1]][:3, 3].tolist()
                    self.stairs.append(
                        StairEdge(
                            edge_id=f"stair_{len(self.stairs):02d}",
                            from_floor=last_floor,
                            to_floor=fl_id,
                            keyframe_ids=seg_kfs,
                            start_xyz=start_xyz,
                            end_xyz=end_xyz,
                        )
                    )
                last_floor = fl_id
                last_floor_idx = idx

        return self.floors

    def _compute_room_geometry(
        self, keyframe_ids: Sequence[str]
    ) -> tuple[list[float], list[float], list[float], float]:
        pts = np.array(
            [self.keyframe_poses[k][:3, 3] for k in keyframe_ids if k in self.keyframe_poses],
            dtype=np.float64,
        )
        if len(pts) == 0:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0
        clrs = [
            self.keyframe_clearances[k]
            for k in keyframe_ids
            if k in self.keyframe_clearances and math.isfinite(self.keyframe_clearances[k])
        ]
        clearance_m = float(np.mean(clrs)) if clrs else 0.0
        return (
            np.mean(pts, axis=0).tolist(),
            np.min(pts, axis=0).tolist(),
            np.max(pts, axis=0).tolist(),
            clearance_m,
        )

    @staticmethod
    def _connected_components(
        nodes: Sequence[str], adj: dict[str, set[str]]
    ) -> list[list[str]]:
        allowed = set(nodes)
        visited: set[str] = set()
        components: list[list[str]] = []
        for start in nodes:
            if start in visited:
                continue
            comp: list[str] = []
            queue = deque([start])
            visited.add(start)
            while queue:
                cur = queue.popleft()
                comp.append(cur)
                for nbr in adj.get(cur, ()):
                    if nbr in allowed and nbr not in visited:
                        visited.add(nbr)
                        queue.append(nbr)
            components.append(comp)
        return components

    def _los_unblocked_by_sectors(
        self, u: str, v: str, pu: np.ndarray, pv: np.ndarray, dist: float
    ) -> bool:
        if dist < 1e-6:
            return True
        d_xy = pv - pu
        for kf_id, delta in ((u, d_xy), (v, -d_xy)):
            sectors = self.keyframe_sector_clearances.get(kf_id)
            if sectors is None:
                continue
            rot_2d = self.keyframe_poses[kf_id][:2, :2]
            d_body = rot_2d.T @ delta
            ang = math.atan2(float(d_body[1]), float(d_body[0])) + math.pi
            n_sec = int(sectors.size)
            sec = min(n_sec - 1, max(0, int(ang / (2.0 * math.pi) * n_sec)))
            sec_dist = float(sectors[sec])
            if math.isfinite(sec_dist) and sec_dist < 0.85 * dist:
                return False
        return True

    def _edge_free_space_ok(
        self,
        u: str,
        v: str,
        pu: np.ndarray,
        pv: np.ndarray,
        dist: float,
        c_u: float | None,
        c_v: float | None,
        is_traj_adj: bool,
    ) -> bool:
        if c_u is None or c_v is None:
            return False
        if is_traj_adj:
            if dist >= 0.95 * (c_u + c_v):
                return False
            if dist > min(c_u, c_v) and not self._los_unblocked_by_sectors(u, v, pu, pv, dist):
                return False
            return True
        if dist >= 0.75 * (c_u + c_v):
            return False
        return self._los_unblocked_by_sectors(u, v, pu, pv, dist)

    def _build_floor_proximity(
        self,
        kfs: Sequence[str],
        link_radius_m: float,
    ) -> tuple[np.ndarray, dict[str, int], dict[str, set[str]], list[tuple[int, int, float]]]:
        pts_xy = np.array([self.keyframe_poses[k][:2, 3] for k in kfs], dtype=np.float64)
        idx_map = {k: i for i, k in enumerate(kfs)}
        adj: dict[str, set[str]] = {k: set() for k in kfs}
        pair_dists: list[tuple[int, int, float]] = []
        if len(kfs) >= 2:
            tree = cKDTree(pts_xy)
            pairs = tree.query_pairs(r=link_radius_m, output_type="ndarray")
            for i, j in pairs:
                ii, jj = int(i), int(j)
                dist = float(np.linalg.norm(pts_xy[jj] - pts_xy[ii]))
                u, v = kfs[ii], kfs[jj]
                adj[u].add(v)
                adj[v].add(u)
                pair_dists.append((ii, jj, dist))
        return pts_xy, idx_map, adj, pair_dists

    def _segment_id_map(self) -> dict[str, int]:
        all_kfs = list(self.keyframe_poses.keys())
        max_gap = float(self._floor_params.get("max_trajectory_gap_s", 15.0))
        segments = self._continuous_segments(all_kfs, max_gap)
        seg_id: dict[str, int] = {}
        for s_idx, seg in enumerate(segments):
            for idx in seg:
                seg_id[all_kfs[idx]] = s_idx
        return seg_id

    def _build_doorways_for_floor(
        self,
        kfs: Sequence[str],
        kf_to_room: dict[str, str],
        bottlenecks: Sequence[str],
        adj: dict[str, set[str]],
        clr: dict[str, float | None],
        kf_order: dict[str, int],
        seg_id_map: dict[str, int],
        link_radius_m: float,
    ) -> list[DoorwayEdge]:
        bottleneck_set = set(bottlenecks)
        pair_clusters: dict[tuple[str, str], list[list[str]]] = {}

        def _record_transition(ra: str, rb: str, anchors: list[str]) -> None:
            if not anchors or ra == rb:
                return
            pair = (ra, rb) if ra < rb else (rb, ra)
            clusters = pair_clusters.setdefault(pair, [])
            pts = np.array([self.keyframe_poses[k][:3, 3] for k in anchors], dtype=np.float64)
            center = np.mean(pts, axis=0)
            for existing in clusters:
                ex_pts = np.array(
                    [self.keyframe_poses[k][:3, 3] for k in existing], dtype=np.float64
                )
                if float(np.linalg.norm(center - np.mean(ex_pts, axis=0))) <= link_radius_m:
                    existing.extend(anchors)
                    return
            clusters.append(list(anchors))

        # 1) Trajectory-order transitions connect consecutive rooms along the path.
        last_room: str | None = None
        last_seg: int | None = None
        pending_bns: list[str] = []
        for k in sorted(kfs, key=lambda item: kf_order.get(item, 0)):
            cur_seg = seg_id_map.get(k)
            if cur_seg != last_seg:
                last_room = None
                pending_bns = []
                last_seg = cur_seg
            if k in kf_to_room:
                curr_room = kf_to_room[k]
                if last_room is not None and curr_room != last_room and pending_bns:
                    _record_transition(last_room, curr_room, pending_bns)
                last_room = curr_room
                pending_bns = []
            elif k in bottleneck_set and last_room is not None:
                pending_bns.append(k)

        # 2) Multi-source BFS on the bottleneck subgraph handles out-of-order/branched contacts
        # without generating all-pairs cliques along narrow corridors.
        owner: dict[str, str] = {}
        queue: deque[str] = deque()
        for b in bottlenecks:
            touched = sorted({kf_to_room[n] for n in adj.get(b, ()) if n in kf_to_room})
            if len(touched) == 1:
                owner[b] = touched[0]
                queue.append(b)
            elif len(touched) >= 2:
                # Single bottleneck directly bridges multiple rooms; connect closest pair(s)
                # only if no trajectory transition already links those rooms.
                ranked = sorted(
                    touched,
                    key=lambda rid: min(
                        float(
                            np.linalg.norm(
                                self.keyframe_poses[b][:2, 3] - self.keyframe_poses[rk][:2, 3]
                            )
                        )
                        for rk in adj[b]
                        if kf_to_room.get(rk) == rid
                    ),
                )
                owner[b] = ranked[0]
                queue.append(b)
                for other in ranked[1:]:
                    pair = (ranked[0], other) if ranked[0] < other else (other, ranked[0])
                    if pair not in pair_clusters:
                        _record_transition(ranked[0], other, [b])

        while queue:
            cur = queue.popleft()
            cur_owner = owner[cur]
            for nbr in sorted(adj.get(cur, ()), key=lambda item: kf_order.get(item, 0)):
                if nbr not in bottleneck_set:
                    continue
                if nbr not in owner:
                    owner[nbr] = cur_owner
                    queue.append(nbr)
                elif owner[nbr] != cur_owner:
                    ra, rb = owner[nbr], cur_owner
                    pair = (ra, rb) if ra < rb else (rb, ra)
                    if pair not in pair_clusters:
                        _record_transition(ra, rb, [cur, nbr])

        edges: list[DoorwayEdge] = []
        for (ra, rb), clusters in sorted(pair_clusters.items()):
            for anchors in clusters:
                ordered = sorted(_dedup_ordered(anchors), key=lambda item: kf_order.get(item, 0))
                gw_pts = np.array(
                    [self.keyframe_poses[b][:3, 3] for b in ordered], dtype=np.float64
                )
                gw_xyz = np.mean(gw_pts, axis=0).tolist()
                clrs = [clr[b] for b in ordered if clr.get(b) is not None]
                gw_clr = float(min(clrs)) if clrs else 0.0
                edges.append(
                    DoorwayEdge(
                        room_a=ra,
                        room_b=rb,
                        gateway_xyz=gw_xyz,
                        clearance_m=gw_clr,
                        bottleneck_anchor_ids=ordered,
                    )
                )
        return edges

    def partition_rooms(
        self,
        doorway_clearance_m: float = 0.85,
        link_radius_m: float = 2.5,
        min_room_keyframes: int = 2,
        max_rep_images: int = 4,
    ) -> dict[str, RoomNode]:
        if doorway_clearance_m < 0.0 or link_radius_m <= 0.0 or min_room_keyframes < 1:
            raise ValueError("invalid room partitioning parameters")
        if max_rep_images < 0:
            raise ValueError("max_rep_images must be nonnegative")
        self._room_params = {
            "doorway_clearance_m": float(doorway_clearance_m),
            "link_radius_m": float(link_radius_m),
            "min_room_keyframes": int(min_room_keyframes),
            "max_rep_images": int(max_rep_images),
        }

        if not self.floors and self.keyframe_poses:
            self.segment_floors(**self._floor_params)

        self.rooms = {}
        self.doorways = []
        kf_order = {kf_id: idx for idx, kf_id in enumerate(self.keyframe_poses.keys())}
        seg_id_map = self._segment_id_map()

        for floor_id, floor in sorted(
            self.floors.items(), key=lambda kv: kv[1].level_index
        ):
            kfs = [k for k in floor.keyframe_ids if k in self.keyframe_poses]
            if not kfs:
                continue
            clr: dict[str, float | None] = {
                k: self.keyframe_clearances.get(k) for k in kfs
            }
            bottlenecks = [
                k for k in kfs if clr[k] is not None and clr[k] < doorway_clearance_m
            ]
            wide = [
                k for k in kfs if clr[k] is not None and clr[k] >= doorway_clearance_m
            ]
            wide_set = set(wide)

            pts_xy, _, adj, pair_dists = self._build_floor_proximity(kfs, link_radius_m)

            bn_xy = (
                np.array([self.keyframe_poses[b][:2, 3] for b in bottlenecks], dtype=np.float64)
                if bottlenecks
                else None
            )
            bn_tree = cKDTree(bn_xy) if bn_xy is not None and len(bn_xy) > 0 else None

            wide_adj: dict[str, set[str]] = {k: set() for k in wide}
            for i, j, dist in pair_dists:
                u, v = kfs[i], kfs[j]
                if u not in wide_set or v not in wide_set:
                    continue
                pu, pv = pts_xy[i], pts_xy[j]
                is_traj_adj = (
                    abs(kf_order[u] - kf_order[v]) == 1
                    and seg_id_map.get(u) == seg_id_map.get(v)
                )
                if not self._edge_free_space_ok(
                    u, v, pu, pv, dist, clr[u], clr[v], is_traj_adj
                ):
                    continue
                if bn_tree is not None:
                    mid = 0.5 * (pu + pv)
                    cand_bns = bn_tree.query_ball_point(mid, r=0.5 * dist + doorway_clearance_m)
                    if any(
                        _segment_blocked_by_point(pu, pv, bn_xy[b_idx], doorway_clearance_m)
                        for b_idx in cand_bns
                    ):
                        continue
                wide_adj[u].add(v)
                wide_adj[v].add(u)

            raw_comps = self._connected_components(wide, wide_adj)
            valid_comps = [c for c in raw_comps if len(c) >= min_room_keyframes]

            # Fallback only when no clearance measurements exist on the floor at all.
            has_any_clearance = any(clr[k] is not None for k in kfs)
            if not valid_comps and not has_any_clearance and len(kfs) >= min_room_keyframes:
                valid_comps = [
                    c for c in self._connected_components(kfs, adj) if len(c) >= min_room_keyframes
                ]

            for comp in valid_comps:
                comp.sort(key=lambda k: kf_order.get(k, 0))
            valid_comps.sort(key=lambda c: (kf_order.get(c[0], 0), c[0]))

            floor_rooms: dict[str, RoomNode] = {}
            for idx, comp in enumerate(valid_comps, start=1):
                room_id = f"{floor_id}_room_{idx:02d}"
                centroid, bmin, bmax, mean_clr = self._compute_room_geometry(comp)
                rep_rgbs = _select_representative_rgbs(
                    comp, self.keyframe_rgb_paths, max_rep_images
                )
                room = RoomNode(
                    room_id=room_id,
                    floor_id=floor_id,
                    alias=room_id,
                    anchor_keyframe_ids=list(comp),
                    representative_rgb_paths=rep_rgbs,
                    centroid_xyz=centroid,
                    bounds_min_xyz=bmin,
                    bounds_max_xyz=bmax,
                    clearance_m=mean_clr,
                    merged_aliases=[],
                )
                floor_rooms[room_id] = room
                self.rooms[room_id] = room

            if bottlenecks and len(floor_rooms) >= 2:
                kf_to_room = {
                    k: rid for rid, room in floor_rooms.items() for k in room.anchor_keyframe_ids
                }
                self.doorways.extend(
                    self._build_doorways_for_floor(
                        kfs=kfs,
                        kf_to_room=kf_to_room,
                        bottlenecks=bottlenecks,
                        adj=adj,
                        clr=clr,
                        kf_order=kf_order,
                        seg_id_map=seg_id_map,
                        link_radius_m=link_radius_m,
                    )
                )

        for obj in self.objects.values():
            obj.room_id = self._resolve_room_for_keyframe(
                obj.anchor_keyframe_id, obj.world_center_xyz
            )

        return self.rooms

    def _resolve_room_for_keyframe(
        self, anchor_keyframe_id: str, world_xyz: Sequence[float]
    ) -> str | None:
        for room_id in sorted(self.rooms):
            if anchor_keyframe_id in self.rooms[room_id].anchor_keyframe_ids:
                return room_id
        owner_floor = None
        for floor in self.floors.values():
            if anchor_keyframe_id in floor.keyframe_ids:
                owner_floor = floor.floor_id
                break
        if owner_floor is None:
            return None
        same_floor_rooms = [
            r for r in self.rooms.values() if r.floor_id == owner_floor
        ]
        if not same_floor_rooms:
            return None
        nearest = min(
            same_floor_rooms,
            key=lambda r: (math.dist(world_xyz, r.centroid_xyz), r.room_id),
        )
        return nearest.room_id

    def add_object(
        self,
        label: str,
        anchor_keyframe_id: str,
        local_center_xyz: Sequence[float],
        local_extent_xyz: Sequence[float] = (0.0, 0.0, 0.0),
        *,
        object_id: str | None = None,
        confidence: float = 1.0,
        producer: str = "unknown",
        room_id: str | None = None,
    ) -> ObjectNode:
        if anchor_keyframe_id not in self.keyframe_poses:
            raise ValueError(f"unknown anchor_keyframe_id: {anchor_keyframe_id}")
        if not isinstance(label, str) or not label:
            raise ValueError("label must be a non-empty string")
        if room_id is not None and room_id not in self.rooms:
            raise ValueError(f"unknown room_id: {room_id}")
        if object_id is not None and (not isinstance(object_id, str) or not object_id):
            raise ValueError("object_id must be a non-empty string")
        local_c = _validate_vec3(local_center_xyz, "local_center_xyz")
        local_e = _validate_vec3(local_extent_xyz, "local_extent_xyz")
        conf = float(confidence)
        if not math.isfinite(conf):
            raise ValueError("confidence must be finite")

        T_world_kf = self.keyframe_poses[anchor_keyframe_id]
        world_c = (T_world_kf[:3, :3] @ local_c + T_world_kf[:3, 3]).tolist()

        if object_id is None:
            while f"obj_{self._next_object_seq:04d}" in self.objects:
                self._next_object_seq += 1
            object_id = f"obj_{self._next_object_seq:04d}"
            self._next_object_seq += 1

        resolved_room = (
            room_id
            if room_id is not None
            else self._resolve_room_for_keyframe(anchor_keyframe_id, world_c)
        )
        node = ObjectNode(
            object_id=object_id,
            label=label,
            anchor_keyframe_id=anchor_keyframe_id,
            local_center_xyz=local_c.tolist(),
            local_extent_xyz=local_e.tolist(),
            world_center_xyz=world_c,
            confidence=conf,
            producer=str(producer),
            room_id=resolved_room,
        )
        self.objects[object_id] = node
        return node

    def rename_room(self, room_id: str, alias: str) -> RoomNode:
        if room_id not in self.rooms:
            raise KeyError(room_id)
        if not isinstance(alias, str) or not alias.strip():
            raise ValueError("alias must be a non-empty string")
        room = self.rooms[room_id]
        room.alias = alias
        return room

    def _recompute_doorway_geometry(self, edge: DoorwayEdge) -> None:
        valid = [k for k in edge.bottleneck_anchor_ids if k in self.keyframe_poses]
        if valid:
            pts = np.array([self.keyframe_poses[k][:3, 3] for k in valid], dtype=np.float64)
            edge.gateway_xyz = np.mean(pts, axis=0).tolist()
            clrs = [
                self.keyframe_clearances[k]
                for k in valid
                if k in self.keyframe_clearances and math.isfinite(self.keyframe_clearances[k])
            ]
            if clrs:
                edge.clearance_m = float(min(clrs))

    def _floors_or_stairs_need_recompute(
        self,
        changed_kfs: set[str],
        prev_poses: dict[str, np.ndarray],
        new_kfs: set[str],
    ) -> bool:
        if new_kfs:
            return True
        if not self.floors:
            return bool(self.keyframe_poses)
        floor_sep_m = float(self._floor_params["floor_sep_m"])
        level_band_m = float(self._floor_params["level_band_m"])

        for k in changed_kfs:
            if k in prev_poses and abs(float(self.keyframe_poses[k][2, 3] - prev_poses[k][2, 3])) > 0.25:
                return True

        z_means: list[float] = []
        for floor in self.floors.values():
            zs = [
                float(self.keyframe_poses[k][2, 3])
                for k in floor.keyframe_ids
                if k in self.keyframe_poses
            ]
            if not zs:
                continue
            if max(zs) - min(zs) > 2.2 * level_band_m:
                return True
            z_means.append(float(np.mean(zs)))
        z_means.sort()
        for i in range(len(z_means) - 1):
            if z_means[i + 1] - z_means[i] < floor_sep_m:
                return True

        for stair in self.stairs:
            if stair.from_floor == stair.to_floor:
                return True
            valid_kfs = [k for k in stair.keyframe_ids if k in self.keyframe_poses]
            if len(valid_kfs) >= 2:
                dz = abs(
                    float(self.keyframe_poses[valid_kfs[-1]][2, 3])
                    - float(self.keyframe_poses[valid_kfs[0]][2, 3])
                )
                if dz < 0.5 * floor_sep_m:
                    return True
        return False

    def _any_room_internally_disconnected(self) -> bool:
        link_radius_m = float(self._room_params["link_radius_m"])
        for room in self.rooms.values():
            kfs = [k for k in room.anchor_keyframe_ids if k in self.keyframe_poses]
            if len(kfs) <= 1:
                continue
            pts_xy = np.array([self.keyframe_poses[k][:2, 3] for k in kfs], dtype=np.float64)
            tree = cKDTree(pts_xy)
            adj: dict[str, set[str]] = {k: set() for k in kfs}
            for i, j in tree.query_pairs(r=link_radius_m, output_type="ndarray"):
                u, v = kfs[int(i)], kfs[int(j)]
                adj[u].add(v)
                adj[v].add(u)
            if len(self._connected_components(kfs, adj)) > 1:
                return True
        return False

    def _migrate_rooms_by_anchor_overlap(
        self, old_rooms: dict[str, RoomNode]
    ) -> list[dict[str, str]]:
        """Preserve old room IDs and aliases across re-partitioning via shared anchors."""
        if not old_rooms or not self.rooms:
            return []
        new_rooms = dict(self.rooms)
        old_sets = {rid: set(r.anchor_keyframe_ids) for rid, r in old_rooms.items()}
        new_sets = {rid: set(r.anchor_keyframe_ids) for rid, r in new_rooms.items()}

        matches: list[tuple[int, str, str]] = []
        for old_id, o_set in old_sets.items():
            for new_id, n_set in new_sets.items():
                overlap = len(o_set & n_set)
                if overlap > 0:
                    matches.append((overlap, old_id, new_id))
        matches.sort(key=lambda item: (-item[0], item[1], item[2]))

        assigned_old_to_new: dict[str, str] = {}
        primary_old_for_new: dict[str, str] = {}
        for _, old_id, new_id in matches:
            if old_id in assigned_old_to_new or new_id in primary_old_for_new:
                continue
            assigned_old_to_new[old_id] = new_id
            primary_old_for_new[new_id] = old_id

        # Secondary old rooms whose largest overlap falls into an already-claimed new room
        # represent drift-split rooms that merged during re-partitioning.
        merged_old_into_new: dict[str, list[str]] = {}
        for old_id in sorted(old_rooms):
            if old_id in assigned_old_to_new:
                continue
            best = max(
                ((len(old_sets[old_id] & new_sets[nid]), nid) for nid in new_sets),
                default=(0, ""),
                key=lambda x: (x[0], x[1]),
            )
            if best[0] > 0:
                merged_old_into_new.setdefault(best[1], []).append(old_id)

        used_ids: set[str] = set()
        new_to_final_id: dict[str, str] = {}
        for new_id in sorted(new_rooms):
            prim_old_id = primary_old_for_new.get(new_id)
            if prim_old_id is not None and prim_old_id not in used_ids:
                final_id = prim_old_id
            else:
                final_id = new_id
                if final_id in used_ids:
                    idx = 1
                    while f"{new_rooms[new_id].floor_id}_room_{idx:02d}" in used_ids:
                        idx += 1
                    final_id = f"{new_rooms[new_id].floor_id}_room_{idx:02d}"
            used_ids.add(final_id)
            new_to_final_id[new_id] = final_id

        migrated_rooms: dict[str, RoomNode] = {}
        merged_pairs: list[dict[str, str]] = []
        for new_id, new_room in new_rooms.items():
            final_id = new_to_final_id[new_id]
            prim_old_id = primary_old_for_new.get(new_id)
            prim_old = old_rooms.get(prim_old_id) if prim_old_id else None
            alias = final_id
            merged_aliases: list[str] = []
            if prim_old is not None:
                alias = (
                    prim_old.alias
                    if prim_old.alias != prim_old.room_id
                    else final_id
                )
                merged_aliases.extend(prim_old.merged_aliases)
            for sec_old_id in merged_old_into_new.get(new_id, ()):
                sec_old = old_rooms[sec_old_id]
                for candidate_alias in (sec_old.alias, *sec_old.merged_aliases):
                    if candidate_alias and candidate_alias != alias:
                        merged_aliases.append(candidate_alias)
                merged_pairs.append({"kept": final_id, "merged": sec_old_id})

            migrated_rooms[final_id] = RoomNode(
                room_id=final_id,
                floor_id=new_room.floor_id,
                alias=alias,
                anchor_keyframe_ids=list(new_room.anchor_keyframe_ids),
                representative_rgb_paths=list(new_room.representative_rgb_paths),
                centroid_xyz=list(new_room.centroid_xyz),
                bounds_min_xyz=list(new_room.bounds_min_xyz),
                bounds_max_xyz=list(new_room.bounds_max_xyz),
                clearance_m=new_room.clearance_m,
                merged_aliases=_dedup_ordered(merged_aliases),
            )

        self.rooms = migrated_rooms
        for edge in self.doorways:
            edge.room_a = new_to_final_id.get(edge.room_a, edge.room_a)
            edge.room_b = new_to_final_id.get(edge.room_b, edge.room_b)
            if edge.room_a > edge.room_b:
                edge.room_a, edge.room_b = edge.room_b, edge.room_a

        for obj in self.objects.values():
            obj.room_id = self._resolve_room_for_keyframe(
                obj.anchor_keyframe_id, obj.world_center_xyz
            )
        return merged_pairs

    def reproject_from_poses(
        self,
        updated_poses: dict[str, np.ndarray],
        *,
        merge_iou_min: float = 0.25,
        merge_centroid_m: float = 2.0,
    ) -> dict[str, Any]:
        validated: dict[str, np.ndarray] = {
            str(kf_id): _validate_pose(pose) for kf_id, pose in updated_poses.items()
        }
        new_kfs = {k for k in validated if k not in self.keyframe_poses}
        unsegmented_kfs = (
            set(self.keyframe_poses.keys()) - self._segmented_keyframe_ids
            if self.floors
            else set()
        )
        changed_kfs = {
            k
            for k, pose in validated.items()
            if k in self.keyframe_poses
            and not np.allclose(self.keyframe_poses[k], pose, rtol=0.0, atol=1e-7)
        }
        updated_count = len(new_kfs) + len(changed_kfs)
        rooms_before = len(self.rooms)

        if not new_kfs and not changed_kfs and not unsegmented_kfs:
            return {
                "updated_keyframes": 0,
                "reprojected_objects": len(self.objects),
                "rooms_before": rooms_before,
                "rooms_after": rooms_before,
                "merged_rooms": [],
                "merged_count": 0,
            }

        prev_poses = {k: self.keyframe_poses[k].copy() for k in changed_kfs}
        prev_centroids = {rid: list(r.centroid_xyz[:2]) for rid, r in self.rooms.items()}

        for kf_id, pose in validated.items():
            self.keyframe_poses[kf_id] = pose
            inferred_epoch = _infer_epoch(kf_id, self.keyframe_epochs.get(kf_id))
            if inferred_epoch is not None:
                self.keyframe_epochs[kf_id] = inferred_epoch

        for obj in self.objects.values():
            T = self.keyframe_poses[obj.anchor_keyframe_id]
            local_c = np.asarray(obj.local_center_xyz, dtype=np.float64)
            obj.world_center_xyz = (T[:3, :3] @ local_c + T[:3, 3]).tolist()

        floors_need_recompute = self._floors_or_stairs_need_recompute(
            changed_kfs, prev_poses, new_kfs | unsegmented_kfs
        )
        rooms_disconnected = self._any_room_internally_disconnected()

        if floors_need_recompute or (rooms_disconnected and self.rooms):
            old_rooms = dict(self.rooms)
            self.segment_floors(**self._floor_params)
            if old_rooms or new_kfs or unsegmented_kfs:
                self.partition_rooms(**self._room_params)
                merged_pairs = self._migrate_rooms_by_anchor_overlap(old_rooms)
            else:
                merged_pairs = []
            return {
                "updated_keyframes": updated_count,
                "reprojected_objects": len(self.objects),
                "rooms_before": rooms_before,
                "rooms_after": len(self.rooms),
                "merged_rooms": merged_pairs,
                "merged_count": len(merged_pairs),
            }

        # Update floor/stair/room/doorway geometry in place when topology didn't split.
        for floor in self.floors.values():
            zs = [
                self.keyframe_poses[k][2, 3]
                for k in floor.keyframe_ids
                if k in self.keyframe_poses
            ]
            if zs:
                floor.z_mean_m = float(np.mean(zs))
                floor.z_min_m = float(np.min(zs))
                floor.z_max_m = float(np.max(zs))

        for stair in self.stairs:
            valid_kfs = [k for k in stair.keyframe_ids if k in self.keyframe_poses]
            if valid_kfs:
                stair.start_xyz = self.keyframe_poses[valid_kfs[0]][:3, 3].tolist()
                stair.end_xyz = self.keyframe_poses[valid_kfs[-1]][:3, 3].tolist()

        for room in self.rooms.values():
            centroid, bmin, bmax, mean_clr = self._compute_room_geometry(
                room.anchor_keyframe_ids
            )
            room.centroid_xyz = centroid
            room.bounds_min_xyz = bmin
            room.bounds_max_xyz = bmax
            room.clearance_m = mean_clr

        for edge in self.doorways:
            self._recompute_doorway_geometry(edge)

        # Merge only drift-split room pairs whose relative distance actually decreased
        # due to loop closure and that are not separated by a doorway.
        doorway_pairs = {
            (min(d.room_a, d.room_b), max(d.room_a, d.room_b)) for d in self.doorways
        }
        room_changed = {
            rid: any(k in changed_kfs for k in room.anchor_keyframe_ids)
            for rid, room in self.rooms.items()
        }
        merged_pairs = []
        changed = True
        while changed:
            changed = False
            room_ids = sorted(self.rooms)
            for i in range(len(room_ids)):
                id_a = room_ids[i]
                room_a = self.rooms.get(id_a)
                if room_a is None:
                    continue
                for j in range(i + 1, len(room_ids)):
                    id_b = room_ids[j]
                    room_b = self.rooms.get(id_b)
                    if room_b is None or room_a.floor_id != room_b.floor_id:
                        continue
                    if not (room_changed.get(id_a) or room_changed.get(id_b)):
                        continue
                    pair_key = (id_a, id_b) if id_a < id_b else (id_b, id_a)
                    if pair_key in doorway_pairs:
                        continue
                    new_dist = math.dist(room_a.centroid_xyz[:2], room_b.centroid_xyz[:2])
                    if id_a in prev_centroids and id_b in prev_centroids:
                        old_dist = math.dist(prev_centroids[id_a], prev_centroids[id_b])
                        if new_dist >= old_dist - 1e-3:
                            continue
                    iou = _bbox_2d_iou(
                        room_a.bounds_min_xyz,
                        room_a.bounds_max_xyz,
                        room_b.bounds_min_xyz,
                        room_b.bounds_max_xyz,
                    )
                    if iou >= merge_iou_min or new_dist <= merge_centroid_m:
                        self._merge_rooms(id_a, id_b)
                        room_changed[id_a] = True
                        merged_pairs.append({"kept": id_a, "merged": id_b})
                        doorway_pairs = {
                            (min(d.room_a, d.room_b), max(d.room_a, d.room_b))
                            for d in self.doorways
                        }
                        changed = True
                        break
                if changed:
                    break

        return {
            "updated_keyframes": updated_count,
            "reprojected_objects": len(self.objects),
            "rooms_before": rooms_before,
            "rooms_after": len(self.rooms),
            "merged_rooms": merged_pairs,
            "merged_count": len(merged_pairs),
        }

    def _merge_rooms(self, keep_id: str, drop_id: str) -> None:
        keep = self.rooms[keep_id]
        drop = self.rooms.pop(drop_id)
        keep.anchor_keyframe_ids = _dedup_ordered(
            [*keep.anchor_keyframe_ids, *drop.anchor_keyframe_ids]
        )
        max_rep = int(self._room_params.get("max_rep_images", 4))
        keep.representative_rgb_paths = _select_representative_rgbs(
            keep.anchor_keyframe_ids, self.keyframe_rgb_paths, max_rep
        )
        extra_aliases = [
            a
            for a in (drop.alias, *drop.merged_aliases)
            if a and a != keep.alias
        ]
        keep.merged_aliases = _dedup_ordered([*keep.merged_aliases, *extra_aliases])

        centroid, bmin, bmax, mean_clr = self._compute_room_geometry(
            keep.anchor_keyframe_ids
        )
        keep.centroid_xyz = centroid
        keep.bounds_min_xyz = bmin
        keep.bounds_max_xyz = bmax
        keep.clearance_m = mean_clr

        for obj in self.objects.values():
            if obj.room_id == drop_id:
                obj.room_id = keep_id

        remapped_doorways: list[DoorwayEdge] = []
        for edge in self.doorways:
            ra = keep_id if edge.room_a == drop_id else edge.room_a
            rb = keep_id if edge.room_b == drop_id else edge.room_b
            if ra == rb:
                continue
            if ra > rb:
                ra, rb = rb, ra
            remapped = DoorwayEdge(
                room_a=ra,
                room_b=rb,
                gateway_xyz=list(edge.gateway_xyz),
                clearance_m=edge.clearance_m,
                bottleneck_anchor_ids=list(edge.bottleneck_anchor_ids),
            )
            self._recompute_doorway_geometry(remapped)
            remapped_doorways.append(remapped)
        self.doorways = remapped_doorways

    def query(
        self,
        *,
        floor_id: str | None = None,
        room_id: str | None = None,
        label: str | None = None,
        near_xyz: Sequence[float] | None = None,
        radius_m: float | None = None,
    ) -> dict[str, Any]:
        if (near_xyz is None) != (radius_m is None):
            raise ValueError("near_xyz and radius_m must be supplied together")
        near_vec: list[float] | None = None
        if near_xyz is not None:
            near_vec = _validate_vec3(near_xyz, "near_xyz").tolist()
            if not math.isfinite(float(radius_m)) or float(radius_m) < 0.0:
                raise ValueError("radius_m must be finite and nonnegative")

        floors = [
            asdict(f)
            for f in sorted(self.floors.values(), key=lambda x: x.level_index)
            if floor_id is None or f.floor_id == floor_id
        ]

        rooms = []
        for r in sorted(self.rooms.values(), key=lambda x: x.room_id):
            if floor_id is not None and r.floor_id != floor_id:
                continue
            if room_id is not None and r.room_id != room_id:
                continue
            if (
                near_vec is not None
                and _dist_to_aabb(near_vec, r.bounds_min_xyz, r.bounds_max_xyz) > radius_m
            ):
                continue
            rooms.append(asdict(r))

        matched_room_ids = {r["room_id"] for r in rooms}
        floor_kf_set = (
            set(self.floors[floor_id].keyframe_ids)
            if floor_id is not None and floor_id in self.floors
            else None
        )

        objects = []
        for o in sorted(self.objects.values(), key=lambda x: x.object_id):
            if floor_id is not None:
                in_floor_room = o.room_id in matched_room_ids
                in_floor_kf = floor_kf_set is not None and o.anchor_keyframe_id in floor_kf_set
                if not (in_floor_room or in_floor_kf):
                    continue
            if room_id is not None and o.room_id != room_id:
                continue
            if label is not None and o.label != label:
                continue
            if (
                near_vec is not None
                and math.dist(o.world_center_xyz, near_vec) > radius_m
            ):
                continue
            objects.append(asdict(o))

        stairs = []
        for s in sorted(self.stairs, key=lambda x: x.edge_id):
            if floor_id is not None and s.from_floor != floor_id and s.to_floor != floor_id:
                continue
            if near_vec is not None:
                vertices = [
                    self.keyframe_poses[k][:3, 3]
                    for k in s.keyframe_ids
                    if k in self.keyframe_poses
                ]
                if not vertices:
                    vertices = [s.start_xyz, s.end_xyz]
                if _dist_to_polyline(near_vec, vertices) > radius_m:
                    continue
            stairs.append(asdict(s))

        doorways = []
        for d in sorted(self.doorways, key=lambda x: (x.room_a, x.room_b)):
            if (
                (floor_id is not None or room_id is not None)
                and d.room_a not in matched_room_ids
                and d.room_b not in matched_room_ids
            ):
                continue
            if near_vec is not None and math.dist(d.gateway_xyz, near_vec) > radius_m:
                continue
            doorways.append(asdict(d))

        return {
            "schema": "keyframe_scene_graph_query/v1",
            "filters": {
                "floor_id": floor_id,
                "room_id": room_id,
                "label": label,
                "near_xyz": near_vec,
                "radius_m": float(radius_m) if radius_m is not None else None,
            },
            "floors": floors,
            "stairs": stairs,
            "rooms": rooms,
            "doorways": doorways,
            "objects": objects,
        }

    def compact_agent_summary(self) -> dict[str, Any]:
        room_objects: dict[str, list[str]] = {}
        for obj in sorted(self.objects.values(), key=lambda x: x.object_id):
            if obj.room_id:
                room_objects.setdefault(obj.room_id, []).append(obj.label)

        return {
            "schema": "keyframe_scene_graph_summary/v1",
            "counts": {
                "keyframes": len(self.keyframe_poses),
                "floors": len(self.floors),
                "stairs": len(self.stairs),
                "rooms": len(self.rooms),
                "doorways": len(self.doorways),
                "objects": len(self.objects),
            },
            "floors": [
                {
                    "floor_id": f.floor_id,
                    "level_index": f.level_index,
                    "z_mean_m": round(f.z_mean_m, 2),
                    "keyframes": len(f.keyframe_ids),
                }
                for f in sorted(self.floors.values(), key=lambda x: x.level_index)
            ],
            "stairs": [
                {
                    "edge_id": s.edge_id,
                    "from_floor": s.from_floor,
                    "to_floor": s.to_floor,
                    "start_xyz": [round(v, 2) for v in s.start_xyz],
                    "end_xyz": [round(v, 2) for v in s.end_xyz],
                }
                for s in self.stairs
            ],
            "rooms": [
                {
                    "room_id": r.room_id,
                    "floor_id": r.floor_id,
                    "alias": r.alias,
                    "centroid_xyz": [round(v, 2) for v in r.centroid_xyz],
                    "clearance_m": round(r.clearance_m, 2),
                    "keyframes": len(r.anchor_keyframe_ids),
                    "rep_rgb": list(r.representative_rgb_paths),
                    "objects": room_objects.get(r.room_id, []),
                }
                for r in sorted(self.rooms.values(), key=lambda x: x.room_id)
            ],
            "doorways": [
                {
                    "room_a": d.room_a,
                    "room_b": d.room_b,
                    "gateway_xyz": [round(v, 2) for v in d.gateway_xyz],
                    "clearance_m": round(d.clearance_m, 2),
                }
                for d in sorted(self.doorways, key=lambda x: (x.room_a, x.room_b))
            ],
            "objects": [
                {
                    "object_id": o.object_id,
                    "label": o.label,
                    "room_id": o.room_id,
                    "xyz": [round(v, 2) for v in o.world_center_xyz],
                    "confidence": round(o.confidence, 2),
                }
                for o in sorted(self.objects.values(), key=lambda x: x.object_id)
            ],
        }

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "schema": SCHEMA,
            "keyframe_poses": {
                kf_id: pose.tolist() for kf_id, pose in self.keyframe_poses.items()
            },
            "keyframe_clearances": dict(self.keyframe_clearances),
            "keyframe_rgb_paths": dict(self.keyframe_rgb_paths),
            "keyframe_stamps": dict(self.keyframe_stamps),
            "keyframe_epochs": dict(self.keyframe_epochs),
            "floor_params": dict(self._floor_params),
            "room_params": dict(self._room_params),
            "floors": [
                asdict(f)
                for f in sorted(self.floors.values(), key=lambda x: x.level_index)
            ],
            "stairs": [asdict(s) for s in self.stairs],
            "rooms": [
                asdict(r) for r in sorted(self.rooms.values(), key=lambda x: x.room_id)
            ],
            "doorways": [asdict(d) for d in self.doorways],
            "objects": [
                asdict(o)
                for o in sorted(self.objects.values(), key=lambda x: x.object_id)
            ],
        }
        if self.keyframe_sector_clearances:
            payload["keyframe_sector_clearances"] = {
                kf_id: [None if not math.isfinite(float(v)) else float(v) for v in vec]
                for kf_id, vec in self.keyframe_sector_clearances.items()
            }
        if self.anchor_to_keyframe_map:
            payload["anchor_to_keyframe_map"] = dict(self.anchor_to_keyframe_map)
        if self.anchor_extrinsics:
            payload["anchor_extrinsics"] = {
                aid: pose.tolist() for aid, pose in self.anchor_extrinsics.items()
            }
        return payload

    def _validate_integrity(self) -> None:
        for floor in self.floors.values():
            for kf_id in floor.keyframe_ids:
                if kf_id not in self.keyframe_poses:
                    raise ValueError(f"floor {floor.floor_id} references unknown keyframe {kf_id}")

        for stair in self.stairs:
            if stair.from_floor not in self.floors or stair.to_floor not in self.floors:
                raise ValueError(f"stair {stair.edge_id} references unknown floor")
            _validate_vec3(stair.start_xyz, "stair.start_xyz")
            _validate_vec3(stair.end_xyz, "stair.end_xyz")
            for kf_id in stair.keyframe_ids:
                if kf_id not in self.keyframe_poses:
                    raise ValueError(f"stair {stair.edge_id} references unknown keyframe {kf_id}")

        for room in self.rooms.values():
            if room.floor_id not in self.floors:
                raise ValueError(f"room {room.room_id} references unknown floor {room.floor_id}")
            _validate_vec3(room.centroid_xyz, "room.centroid_xyz")
            _validate_vec3(room.bounds_min_xyz, "room.bounds_min_xyz")
            _validate_vec3(room.bounds_max_xyz, "room.bounds_max_xyz")
            if not math.isfinite(float(room.clearance_m)):
                raise ValueError(f"room {room.room_id} clearance_m must be finite")
            for kf_id in room.anchor_keyframe_ids:
                if kf_id not in self.keyframe_poses:
                    raise ValueError(f"room {room.room_id} references unknown keyframe {kf_id}")

        for edge in self.doorways:
            if edge.room_a not in self.rooms or edge.room_b not in self.rooms:
                raise ValueError(f"doorway ({edge.room_a}, {edge.room_b}) references unknown room")
            _validate_vec3(edge.gateway_xyz, "doorway.gateway_xyz")
            if not math.isfinite(float(edge.clearance_m)):
                raise ValueError("doorway clearance_m must be finite")
            for kf_id in edge.bottleneck_anchor_ids:
                if kf_id not in self.keyframe_poses:
                    raise ValueError(
                        f"doorway ({edge.room_a}, {edge.room_b}) references unknown bottleneck {kf_id}"
                    )

        for obj in self.objects.values():
            if obj.anchor_keyframe_id not in self.keyframe_poses:
                raise ValueError(
                    f"object {obj.object_id} references unknown anchor {obj.anchor_keyframe_id}"
                )
            if obj.room_id is not None and obj.room_id not in self.rooms:
                raise ValueError(
                    f"object {obj.object_id} references unknown room {obj.room_id}"
                )
            _validate_vec3(obj.local_center_xyz, "object.local_center_xyz")
            _validate_vec3(obj.local_extent_xyz, "object.local_extent_xyz")
            _validate_vec3(obj.world_center_xyz, "object.world_center_xyz")
            if not math.isfinite(float(obj.confidence)):
                raise ValueError(f"object {obj.object_id} confidence must be finite")

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> KeyframeSceneGraph:
        if not isinstance(data, dict) or data.get("schema") != SCHEMA:
            raise ValueError(f"scene graph schema must be {SCHEMA}")
        raw_sectors = data.get("keyframe_sector_clearances")
        parsed_sectors = (
            {
                k: [np.inf if v is None else float(v) for v in vec]
                for k, vec in raw_sectors.items()
            }
            if raw_sectors
            else None
        )
        graph = cls(
            keyframe_poses=data.get("keyframe_poses"),
            keyframe_clearances=data.get("keyframe_clearances"),
            keyframe_rgb_paths=data.get("keyframe_rgb_paths"),
            keyframe_stamps=data.get("keyframe_stamps"),
            keyframe_epochs=data.get("keyframe_epochs"),
            keyframe_sector_clearances=parsed_sectors,
            anchor_to_keyframe_map=data.get("anchor_to_keyframe_map"),
            anchor_extrinsics=data.get("anchor_extrinsics"),
        )
        if isinstance(data.get("floor_params"), dict):
            graph._floor_params.update(data["floor_params"])
        if isinstance(data.get("room_params"), dict):
            graph._room_params.update(data["room_params"])
        for row in data.get("floors", []):
            node = FloorNode(**row)
            graph.floors[node.floor_id] = node
        for row in data.get("stairs", []):
            graph.stairs.append(StairEdge(**row))
        for row in data.get("rooms", []):
            node = RoomNode(**row)
            graph.rooms[node.room_id] = node
        for row in data.get("doorways", []):
            graph.doorways.append(DoorwayEdge(**row))
        for row in data.get("objects", []):
            node = ObjectNode(**row)
            graph.objects[node.object_id] = node
            if node.object_id.startswith("obj_") and node.object_id[4:].isdigit():
                graph._next_object_seq = max(
                    graph._next_object_seq, int(node.object_id[4:]) + 1
                )
        graph._validate_integrity()
        if graph.floors:
            graph._segmented_keyframe_ids = set(graph.keyframe_poses.keys())
        return graph

    def save(self, path: str | Path) -> Path:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(self.to_dict(), allow_nan=False).encode("utf-8")
        with tempfile.NamedTemporaryFile(
            dir=target.parent, prefix=".scene_graph_", delete=False
        ) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.replace(temporary, target)
        finally:
            temporary.unlink(missing_ok=True)
        return target

    @classmethod
    def load(cls, path: str | Path) -> KeyframeSceneGraph:
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))
