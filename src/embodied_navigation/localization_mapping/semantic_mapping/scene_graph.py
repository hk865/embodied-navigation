"""Keyframe-anchored hierarchical 3D scene graph.

Pure CPU (stdlib + numpy) representation linking SLAM keyframes to floors,
stairs, rooms, doorways, and objects so loop-closure pose updates reproject
all semantic entities deterministically without duplicating geometry.
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
    arr = np.array(matrix, dtype=np.float64, copy=True)
    if arr.shape != (4, 4) or not np.isfinite(arr).all():
        raise ValueError("pose must be a finite 4x4 matrix")
    return arr


def _validate_vec3(values: Any, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != (3,) or not np.isfinite(arr).all():
        raise ValueError(f"{name} must be a finite 3D vector")
    return arr


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

    # Pad degenerate 1D/0D room footprints (e.g., strictly collinear corridor)
    # so IoU remains meaningful instead of collapsing to 0/0.
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
    ) -> None:
        self.keyframe_poses: dict[str, np.ndarray] = {}
        if keyframe_poses:
            for kf_id, pose in keyframe_poses.items():
                self.keyframe_poses[str(kf_id)] = _validate_pose(pose)
        self.keyframe_clearances: dict[str, float] = {}
        if keyframe_clearances:
            for kf_id, clr in keyframe_clearances.items():
                val = float(clr)
                if not math.isfinite(val) or val < 0.0:
                    raise ValueError("clearance_m must be finite and nonnegative")
                self.keyframe_clearances[str(kf_id)] = val
        self.keyframe_rgb_paths: dict[str, str] = (
            {str(k): str(v) for k, v in keyframe_rgb_paths.items()}
            if keyframe_rgb_paths
            else {}
        )
        self.floors: dict[str, FloorNode] = {}
        self.stairs: list[StairEdge] = []
        self.rooms: dict[str, RoomNode] = {}
        self.doorways: list[DoorwayEdge] = []
        self.objects: dict[str, ObjectNode] = {}
        self._next_object_seq: int = 1

    def add_keyframe(
        self,
        keyframe_id: str,
        T_world_body: np.ndarray,
        *,
        clearance_m: float | None = None,
        rgb_path: str | None = None,
    ) -> None:
        if not isinstance(keyframe_id, str) or not keyframe_id:
            raise ValueError("keyframe_id must be a non-empty string")
        self.keyframe_poses[keyframe_id] = _validate_pose(T_world_body)
        if clearance_m is not None:
            val = float(clearance_m)
            if not math.isfinite(val) or val < 0.0:
                raise ValueError("clearance_m must be finite and nonnegative")
            self.keyframe_clearances[keyframe_id] = val
        if rgb_path is not None:
            self.keyframe_rgb_paths[keyframe_id] = str(rgb_path)

    def segment_floors(
        self,
        z_bin_m: float = 0.25,
        floor_sep_m: float = 1.8,
        level_band_m: float = 0.65,
    ) -> dict[str, FloorNode]:
        if z_bin_m <= 0.0 or floor_sep_m <= 0.0 or level_band_m <= 0.0:
            raise ValueError("z_bin_m, floor_sep_m, and level_band_m must be positive")
        self.floors = {}
        self.stairs = []
        kf_ids = list(self.keyframe_poses.keys())
        if not kf_ids:
            return self.floors

        zs = np.array([self.keyframe_poses[k][2, 3] for k in kf_ids], dtype=np.float64)
        bin_indices = np.unique(np.floor(zs / z_bin_m).astype(np.int64))
        half_win = max(z_bin_m, 0.5 * level_band_m)

        candidates: list[tuple[int, float, float]] = []
        for b in bin_indices:
            center = (float(b) + 0.5) * z_bin_m
            support = int(np.sum(np.abs(zs - center) <= half_win))
            candidates.append((support, -abs(center), center))
        candidates.sort(reverse=True)

        max_support = candidates[0][0]
        min_support = 2 if max_support >= 2 else 1

        peaks: list[float] = []
        for support, _, center in candidates:
            if support < min_support:
                continue
            if any(abs(center - p) < floor_sep_m for p in peaks):
                continue
            in_band = zs[np.abs(zs - center) <= level_band_m]
            refined = float(np.median(in_band)) if len(in_band) > 0 else center
            if any(abs(refined - p) < floor_sep_m for p in peaks):
                refined = center
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
        last_floor: str | None = None
        last_floor_idx: int | None = None
        for idx, fl_id in enumerate(floor_seq):
            if fl_id is None:
                continue
            if last_floor is not None and fl_id != last_floor and last_floor_idx is not None:
                seg_kfs = kf_ids[last_floor_idx : idx + 1]
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

    def partition_rooms(
        self,
        doorway_clearance_m: float = 0.85,
        link_radius_m: float = 2.5,
        min_room_keyframes: int = 2,
        max_rep_images: int = 4,
    ) -> dict[str, RoomNode]:
        if doorway_clearance_m < 0.0 or link_radius_m <= 0.0 or min_room_keyframes < 1:
            raise ValueError("invalid room partitioning parameters")
        if not self.floors and self.keyframe_poses:
            self.segment_floors()

        self.rooms = {}
        self.doorways = []
        kf_order = {kf_id: idx for idx, kf_id in enumerate(self.keyframe_poses.keys())}

        for floor_id, floor in sorted(
            self.floors.items(), key=lambda kv: kv[1].level_index
        ):
            kfs = [k for k in floor.keyframe_ids if k in self.keyframe_poses]
            if not kfs:
                continue
            pos_xy = {k: self.keyframe_poses[k][:2, 3] for k in kfs}
            clr = {k: self.keyframe_clearances.get(k, float("inf")) for k in kfs}
            bottlenecks = [k for k in kfs if clr[k] < doorway_clearance_m]
            bottleneck_set = set(bottlenecks)
            wide = [k for k in kfs if k not in bottleneck_set]

            # Build full 2D proximity graph over floor keyframes.
            adj: dict[str, set[str]] = {k: set() for k in kfs}
            for i in range(len(kfs)):
                u = kfs[i]
                pu = pos_xy[u]
                for j in range(i + 1, len(kfs)):
                    v = kfs[j]
                    if float(np.linalg.norm(pu - pos_xy[v])) <= link_radius_m:
                        adj[u].add(v)
                        adj[v].add(u)

            # Build wide-only graph, severing edges blocked by a bottleneck gateway.
            wide_adj: dict[str, set[str]] = {k: set() for k in wide}
            for i in range(len(wide)):
                u = wide[i]
                pu = pos_xy[u]
                for j in range(i + 1, len(wide)):
                    v = wide[j]
                    if v not in adj[u]:
                        continue
                    pv = pos_xy[v]
                    if any(
                        _segment_blocked_by_point(pu, pv, pos_xy[b], doorway_clearance_m)
                        for b in bottlenecks
                    ):
                        continue
                    wide_adj[u].add(v)
                    wide_adj[v].add(u)

            raw_comps = self._connected_components(wide, wide_adj)
            valid_comps = [c for c in raw_comps if len(c) >= min_room_keyframes]

            if not valid_comps and len(kfs) >= min_room_keyframes:
                valid_comps = [
                    c for c in self._connected_components(kfs, adj) if len(c) >= min_room_keyframes
                ]
                bottlenecks = []
                bottleneck_set = set()

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
                b_comps = self._connected_components(bottlenecks, adj)
                for b_comp in b_comps:
                    touched_rooms = sorted(
                        {
                            kf_to_room[nbr]
                            for b in b_comp
                            for nbr in adj[b]
                            if nbr in kf_to_room
                        }
                    )
                    if len(touched_rooms) < 2:
                        continue
                    gw_pts = np.array(
                        [self.keyframe_poses[b][:3, 3] for b in b_comp],
                        dtype=np.float64,
                    )
                    gw_xyz = np.mean(gw_pts, axis=0).tolist()
                    gw_clr = float(min(clr[b] for b in b_comp))
                    for i in range(len(touched_rooms)):
                        for j in range(i + 1, len(touched_rooms)):
                            self.doorways.append(
                                DoorwayEdge(
                                    room_a=touched_rooms[i],
                                    room_b=touched_rooms[j],
                                    gateway_xyz=gw_xyz,
                                    clearance_m=gw_clr,
                                )
                            )

        # Re-bind existing objects onto the freshly partitioned rooms.
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

    def reproject_from_poses(
        self,
        updated_poses: dict[str, np.ndarray],
        *,
        merge_iou_min: float = 0.25,
        merge_centroid_m: float = 2.0,
    ) -> dict[str, Any]:
        updated_count = 0
        for kf_id, pose in updated_poses.items():
            self.keyframe_poses[str(kf_id)] = _validate_pose(pose)
            updated_count += 1

        for obj in self.objects.values():
            T = self.keyframe_poses[obj.anchor_keyframe_id]
            local_c = np.asarray(obj.local_center_xyz, dtype=np.float64)
            obj.world_center_xyz = (T[:3, :3] @ local_c + T[:3, 3]).tolist()

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

        rooms_before = len(self.rooms)
        merged_pairs: list[dict[str, str]] = []

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
                    iou = _bbox_2d_iou(
                        room_a.bounds_min_xyz,
                        room_a.bounds_max_xyz,
                        room_b.bounds_min_xyz,
                        room_b.bounds_max_xyz,
                    )
                    centroid_dist = math.dist(
                        room_a.centroid_xyz[:2], room_b.centroid_xyz[:2]
                    )
                    if iou >= merge_iou_min or centroid_dist <= merge_centroid_m:
                        self._merge_rooms(id_a, id_b)
                        merged_pairs.append({"kept": id_a, "merged": id_b})
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
        keep.representative_rgb_paths = _dedup_ordered(
            [*keep.representative_rgb_paths, *drop.representative_rgb_paths]
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
            remapped_doorways.append(
                DoorwayEdge(
                    room_a=ra,
                    room_b=rb,
                    gateway_xyz=list(edge.gateway_xyz),
                    clearance_m=edge.clearance_m,
                )
            )
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

        stairs = [
            asdict(s)
            for s in sorted(self.stairs, key=lambda x: x.edge_id)
            if floor_id is None or s.from_floor == floor_id or s.to_floor == floor_id
        ]
        doorways = [
            asdict(d)
            for d in sorted(self.doorways, key=lambda x: (x.room_a, x.room_b))
            if (
                (floor_id is None and room_id is None)
                or d.room_a in matched_room_ids
                or d.room_b in matched_room_ids
            )
        ]

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
        return {
            "schema": SCHEMA,
            "keyframe_poses": {
                kf_id: pose.tolist() for kf_id, pose in self.keyframe_poses.items()
            },
            "keyframe_clearances": dict(self.keyframe_clearances),
            "keyframe_rgb_paths": dict(self.keyframe_rgb_paths),
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

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> KeyframeSceneGraph:
        if not isinstance(data, dict) or data.get("schema") != SCHEMA:
            raise ValueError(f"scene graph schema must be {SCHEMA}")
        graph = cls(
            keyframe_poses=data.get("keyframe_poses"),
            keyframe_clearances=data.get("keyframe_clearances"),
            keyframe_rgb_paths=data.get("keyframe_rgb_paths"),
        )
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
