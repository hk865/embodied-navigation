"""L0-L3 hierarchical submap memory, failure ledger, and STOP verification.

Wraps :class:`KeyframeSceneGraph` with:
- Monotonic ``map_revision`` versioning and staleness tracking across loop updates.
- Four-tier retrieval pyramid (L0 global topology <1 KB -> L1 room/submap summaries
  -> L2 object instances & failure ledger -> L3 on-demand keyframe RGB references).
- In-episode ``Attempt & Failure Ledger`` geographic blacklist to stop repeat loops.
- Pre-STOP geometric & evidence verification to separate Oracle Stop from active STOP.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import time
from typing import Any, Iterable, Sequence

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping.scene_graph import (
    KeyframeSceneGraph,
    ObjectNode,
    RoomNode,
)

MEMORY_SCHEMA = "embodied_navigation.hierarchical_memory.v1"

FAILURE_OUTCOMES = frozenset(
    {"failed", "rejected", "timeout", "cancelled", "stuck", "dropped", "expired"}
)


def _finite_xyz(values: Sequence[float], name: str = "xyz") -> tuple[float, float, float]:
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    if arr.shape != (3,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must be a finite 3-vector")
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def _tokenize(text: str | None) -> set[str]:
    if not text:
        return set()
    cleaned = "".join(ch.lower() if ch.isalnum() else " " for ch in str(text))
    return {tok for tok in cleaned.split() if tok}


def _label_match_score(query: str | None, candidate_label: str) -> float:
    """Return semantic match specificity (>0 if matched, 0.0 if unrelated)."""
    if not query:
        return 1.0
    q_norm = " ".join(str(query).lower().split())
    c_norm = " ".join(str(candidate_label).lower().split())
    if not q_norm or not c_norm:
        return 0.0
    if q_norm == c_norm:
        return 100.0
    if q_norm in c_norm or c_norm in q_norm:
        return 50.0 + 10.0 * (min(len(q_norm), len(c_norm)) / max(len(q_norm), len(c_norm)))
    q_toks = _tokenize(q_norm)
    c_toks = _tokenize(c_norm)
    if not q_toks or not c_toks:
        return 0.0
    overlap = len(q_toks & c_toks)
    if overlap == 0:
        return 0.0
    recall = overlap / len(q_toks)
    if len(q_toks) >= 2 and recall < 0.5:
        return 0.0
    jaccard = overlap / len(q_toks | c_toks)
    return 20.0 * recall + 10.0 * jaccard


def _xy_dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


@dataclass
class AttemptRecord:
    """One navigation or STOP attempt recorded in the episode ledger."""

    attempt_id: str
    action: str
    goal_xyz: tuple[float, float, float]
    outcome: str
    reason: str = ""
    floor_id: str | None = None
    room_id: str | None = None
    keyframe_id: str | None = None
    map_revision: int = 0
    wall_s: float = 0.0
    radius_m: float = 0.65
    stale: bool = False

    @property
    def is_failure(self) -> bool:
        return self.outcome in FAILURE_OUTCOMES

    def to_dict(self) -> dict[str, Any]:
        return {
            "attempt_id": self.attempt_id,
            "action": self.action,
            "goal_xyz": [round(v, 3) for v in self.goal_xyz],
            "outcome": self.outcome,
            "reason": self.reason,
            "floor_id": self.floor_id,
            "room_id": self.room_id,
            "keyframe_id": self.keyframe_id,
            "map_revision": self.map_revision,
            "wall_s": round(self.wall_s, 3),
            "radius_m": round(self.radius_m, 3),
            "stale": self.stale,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> AttemptRecord:
        return cls(
            attempt_id=str(data["attempt_id"]),
            action=str(data["action"]),
            goal_xyz=_finite_xyz(data["goal_xyz"], "goal_xyz"),
            outcome=str(data["outcome"]),
            reason=str(data.get("reason", "")),
            floor_id=data.get("floor_id"),
            room_id=data.get("room_id"),
            keyframe_id=data.get("keyframe_id"),
            map_revision=int(data.get("map_revision", 0)),
            wall_s=float(data.get("wall_s", 0.0)),
            radius_m=float(data.get("radius_m", 0.65)),
            stale=bool(data.get("stale", False)),
        )


@dataclass
class ObjectEvidenceMeta:
    """Revision and freshness metadata attached to a KeyframeSceneGraph object."""

    object_id: str
    map_revision: int = 0
    observed_wall_s: float = 0.0
    confidence: float = 1.0
    stale: bool = False
    stale_reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "object_id": self.object_id,
            "map_revision": self.map_revision,
            "observed_wall_s": round(self.observed_wall_s, 3),
            "confidence": round(self.confidence, 3),
            "stale": self.stale,
            "stale_reason": self.stale_reason,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ObjectEvidenceMeta:
        return cls(
            object_id=str(data["object_id"]),
            map_revision=int(data.get("map_revision", 0)),
            observed_wall_s=float(data.get("observed_wall_s", 0.0)),
            confidence=float(data.get("confidence", 1.0)),
            stale=bool(data.get("stale", False)),
            stale_reason=str(data.get("stale_reason", "")),
        )


@dataclass(frozen=True)
class StopVerificationResult:
    """Structured pre-STOP geometric & evidence verification verdict."""

    allowed: bool
    reason: str
    matched_object_id: str | None = None
    matched_room_id: str | None = None
    distance_m: float | None = None
    same_floor: bool = False
    evidence_stale: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "allowed": self.allowed,
            "reason": self.reason,
            "matched_object_id": self.matched_object_id,
            "matched_room_id": self.matched_room_id,
            "distance_m": (None if self.distance_m is None else round(self.distance_m, 3)),
            "same_floor": self.same_floor,
            "evidence_stale": self.evidence_stale,
        }


class HierarchicalMemoryStore:
    """Versioned L0-L3 spatial memory and failure ledger over KeyframeSceneGraph."""

    def __init__(
        self,
        graph: KeyframeSceneGraph,
        *,
        map_revision: int = 0,
        default_blacklist_radius_m: float = 0.65,
        default_stop_radius_m: float = 1.5,
        same_floor_dz_m: float = 1.0,
    ) -> None:
        if default_blacklist_radius_m <= 0.0:
            raise ValueError("default_blacklist_radius_m must be positive")
        if default_stop_radius_m <= 0.0:
            raise ValueError("default_stop_radius_m must be positive")
        if same_floor_dz_m <= 0.0:
            raise ValueError("same_floor_dz_m must be positive")

        self.graph = graph
        self.map_revision = int(map_revision)
        self.default_blacklist_radius_m = float(default_blacklist_radius_m)
        self.default_stop_radius_m = float(default_stop_radius_m)
        self.same_floor_dz_m = float(same_floor_dz_m)

        self._attempts: list[AttemptRecord] = []
        self._object_meta: dict[str, ObjectEvidenceMeta] = {}
        self._room_visits: dict[str, int] = {}
        self._repeat_failure_attempts = 0
        self._premature_stop_rejects = 0
        self._stale_belief_rejects = 0

        for obj_id in self.graph.objects:
            self._object_meta[obj_id] = ObjectEvidenceMeta(
                object_id=obj_id,
                map_revision=self.map_revision,
            )

    def add_object(
        self,
        object_id: str,
        label: str,
        keyframe_id: str,
        local_xyz: Sequence[float],
        *,
        local_extent_xyz: Sequence[float] = (0.0, 0.0, 0.0),
        confidence: float = 1.0,
        producer: str = "hierarchical_memory",
        room_id: str | None = None,
        wall_s: float | None = None,
    ) -> ObjectNode:
        """Register an object in the underlying scene graph with version metadata."""
        node = self.graph.add_object(
            label=label,
            anchor_keyframe_id=keyframe_id,
            local_center_xyz=local_xyz,
            local_extent_xyz=local_extent_xyz,
            object_id=object_id,
            confidence=confidence,
            producer=producer,
            room_id=room_id,
        )
        self._object_meta[node.object_id] = ObjectEvidenceMeta(
            object_id=node.object_id,
            map_revision=self.map_revision,
            observed_wall_s=(time.time() if wall_s is None else float(wall_s)),
            confidence=float(confidence),
            stale=False,
            stale_reason="",
        )
        return node

    def mark_object_stale(self, object_id: str, reason: str = "reobserved_absent") -> None:
        meta = self._object_meta.get(object_id)
        if meta is None:
            meta = ObjectEvidenceMeta(object_id=object_id, map_revision=self.map_revision)
            self._object_meta[object_id] = meta
        meta.stale = True
        meta.stale_reason = str(reason)

    def sync_poses(
        self,
        updated_poses: dict[str, np.ndarray],
        *,
        failure_invalidation_shift_m: float = 0.25,
        object_staleness_shift_m: float | None = None,
    ) -> dict[str, Any]:
        """Reproject scene graph from backend loop poses and advance map_revision."""
        shifts_m: dict[str, float] = {}
        for kf_id, new_pose in updated_poses.items():
            old_pose = self.graph.keyframe_poses.get(kf_id)
            if old_pose is not None:
                arr_new = np.asarray(new_pose, dtype=np.float64)
                if arr_new.shape == (4, 4) and np.all(np.isfinite(arr_new)):
                    shifts_m[kf_id] = float(np.linalg.norm(arr_new[:3, 3] - old_pose[:3, 3]))

        report = self.graph.reproject_from_poses(updated_poses)
        self.map_revision += 1

        invalidated_failures = 0
        for rec in self._attempts:
            if rec.stale or not rec.is_failure:
                continue
            if rec.keyframe_id and shifts_m.get(rec.keyframe_id, 0.0) >= failure_invalidation_shift_m:
                rec.stale = True
                invalidated_failures += 1

        stale_objects = 0
        if object_staleness_shift_m is not None:
            for obj_id, obj in self.graph.objects.items():
                if shifts_m.get(obj.anchor_keyframe_id, 0.0) >= object_staleness_shift_m:
                    self.mark_object_stale(obj_id, reason="loop_shift_exceeded")
                    stale_objects += 1

        return {
            "map_revision": self.map_revision,
            "max_shift_m": max(shifts_m.values(), default=0.0),
            "invalidated_failures": invalidated_failures,
            "stale_objects": stale_objects,
            "reproject_report": report,
        }

    def record_room_visit(self, room_id: str) -> None:
        if room_id:
            self._room_visits[room_id] = self._room_visits.get(room_id, 0) + 1

    def record_attempt(
        self,
        action: str,
        goal_xyz: Sequence[float],
        outcome: str,
        *,
        reason: str = "",
        floor_id: str | None = None,
        room_id: str | None = None,
        keyframe_id: str | None = None,
        radius_m: float | None = None,
        wall_s: float | None = None,
    ) -> AttemptRecord:
        """Append an attempt to the ledger and track repeat failures."""
        xyz = _finite_xyz(goal_xyz, "goal_xyz")
        eff_radius = (
            self.default_blacklist_radius_m if radius_m is None else float(radius_m)
        )
        prior_matches = self.match_blacklist(xyz, floor_id=floor_id, radius_m=eff_radius)
        rec = AttemptRecord(
            attempt_id=f"att_{len(self._attempts) + 1:04d}",
            action=str(action),
            goal_xyz=xyz,
            outcome=str(outcome).lower(),
            reason=str(reason),
            floor_id=floor_id,
            room_id=room_id,
            keyframe_id=keyframe_id,
            map_revision=self.map_revision,
            wall_s=(time.time() if wall_s is None else float(wall_s)),
            radius_m=eff_radius,
            stale=False,
        )
        if rec.is_failure and prior_matches:
            self._repeat_failure_attempts += 1
        self._attempts.append(rec)
        return rec

    def match_blacklist(
        self,
        goal_xyz: Sequence[float],
        *,
        floor_id: str | None = None,
        radius_m: float | None = None,
    ) -> list[AttemptRecord]:
        """Return active failure records overlapping a proposed goal location."""
        xyz = _finite_xyz(goal_xyz, "goal_xyz")
        matches: list[AttemptRecord] = []
        for rec in self._attempts:
            if rec.stale or not rec.is_failure:
                continue
            if floor_id is not None and rec.floor_id is not None and rec.floor_id != floor_id:
                continue
            if abs(xyz[2] - rec.goal_xyz[2]) > self.same_floor_dz_m:
                continue
            limit = max(rec.radius_m, 0.0 if radius_m is None else float(radius_m))
            if _xy_dist(xyz, rec.goal_xyz) <= limit:
                matches.append(rec)
        return matches

    def filter_candidates(
        self,
        candidates: Iterable[dict[str, Any]],
        *,
        floor_id: str | None = None,
        radius_m: float | None = None,
        default_z: float = 0.0,
    ) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        """Split candidate goals into (allowed, blacklisted) using the failure ledger."""
        allowed: list[dict[str, Any]] = []
        blocked: list[dict[str, Any]] = []
        for cand in candidates:
            xyz = cand.get("xyz") or cand.get("goal_xyz_m") or cand.get("center_xyz")
            if xyz is None:
                xy = cand.get("xy")
                if isinstance(xy, (list, tuple)) and len(xy) >= 2:
                    xyz = [float(xy[0]), float(xy[1]), float(cand.get("z", default_z))]
            if xyz is None:
                allowed.append(dict(cand))
                continue
            hits = self.match_blacklist(xyz, floor_id=floor_id, radius_m=radius_m)
            item = dict(cand)
            if hits:
                item["blacklisted"] = True
                item["blacklist_hits"] = len(hits)
                item["last_failure_reason"] = hits[-1].reason or hits[-1].outcome
                blocked.append(item)
            else:
                item["blacklisted"] = False
                allowed.append(item)
        return allowed, blocked

    # ------------------------------------------------------------------
    # L0 - L3 Four-Layer Submap Retrieval Pyramid
    # ------------------------------------------------------------------

    def _objects_in_room(self, room_id: str) -> list[ObjectNode]:
        return [obj for obj in self.graph.objects.values() if obj.room_id == room_id]

    def _floor_for_object(self, obj: ObjectNode) -> str | None:
        if obj.room_id and obj.room_id in self.graph.rooms:
            return self.graph.rooms[obj.room_id].floor_id
        for floor in self.graph.floors.values():
            if obj.anchor_keyframe_id in floor.keyframe_ids:
                return floor.floor_id
        return None

    def l0_global_topology(self, *, max_rooms: int = 24) -> dict[str, Any]:
        """L0 (<1 KB target): compact floor, room-id, stair, and doorway topology."""
        rooms_per_floor: dict[str, int] = {}
        for r in self.graph.rooms.values():
            rooms_per_floor[r.floor_id] = rooms_per_floor.get(r.floor_id, 0) + 1
        objs_per_room: dict[str, int] = {}
        for obj in self.graph.objects.values():
            if obj.room_id:
                objs_per_room[obj.room_id] = objs_per_room.get(obj.room_id, 0) + 1

        floors = [
            {
                "id": f.floor_id,
                "z": round(float(f.z_mean_m), 2),
                "rooms": rooms_per_floor.get(f.floor_id, 0),
            }
            for f in self.graph.floors.values()
        ]
        rooms = []
        for r in list(self.graph.rooms.values())[:max_rooms]:
            entry: dict[str, Any] = {
                "id": r.room_id,
                "f": r.floor_id,
                "c": [round(float(v), 1) for v in r.centroid_xyz[:2]],
                "o": objs_per_room.get(r.room_id, 0),
            }
            if r.alias and r.alias != r.room_id:
                entry["a"] = r.alias
            rooms.append(entry)
        doors = [[d.room_a, d.room_b] for d in self.graph.doorways]
        stairs = [[s.from_floor, s.to_floor] for s in self.graph.stairs]
        active_failures = sum(1 for r in self._attempts if r.is_failure and not r.stale)
        return {
            "rev": self.map_revision,
            "floors": floors,
            "rooms": rooms,
            "doors": doors,
            "stairs": stairs,
            "fail_cnt": active_failures,
        }

    def l1_submap_summaries(
        self,
        *,
        query: str | None = None,
        floor_id: str | None = None,
        near_xyz: Sequence[float] | None = None,
        top_k: int = 5,
    ) -> list[dict[str, Any]]:
        """L1: ranked compact room/submap summaries for coarse retrieval."""
        q_tokens = _tokenize(query)
        ref_xyz = None if near_xyz is None else _finite_xyz(near_xyz, "near_xyz")

        scored: list[tuple[float, dict[str, Any]]] = []
        for room in self.graph.rooms.values():
            if floor_id is not None and room.floor_id != floor_id:
                continue
            label_counts: dict[str, int] = {}
            room_tokens = _tokenize(room.alias) | _tokenize(room.room_id)
            for obj in self._objects_in_room(room.room_id):
                meta = self._object_meta.get(obj.object_id)
                if meta is not None and meta.stale:
                    continue
                label_counts[obj.label] = label_counts.get(obj.label, 0) + 1
                room_tokens |= _tokenize(obj.label)

            fail_cnt = sum(
                1
                for rec in self._attempts
                if rec.is_failure and not rec.stale and rec.room_id == room.room_id
            )
            score = 0.0
            if q_tokens:
                overlap = len(q_tokens & room_tokens)
                score += 10.0 * overlap
                joined = " ".join([room.alias or "", room.room_id, *label_counts.keys()]).lower()
                for tok in q_tokens:
                    if tok in joined:
                        score += 3.0
            dist_m = None
            if ref_xyz is not None:
                dist_m = float(np.linalg.norm(np.asarray(room.centroid_xyz) - np.asarray(ref_xyz)))
                score += 1.0 / (1.0 + dist_m)
            score += 0.1 * len(label_counts)

            summary = {
                "room_id": room.room_id,
                "floor_id": room.floor_id,
                "alias": room.alias,
                "center_xyz": [round(float(v), 2) for v in room.centroid_xyz],
                "objects": label_counts,
                "keyframes": len(room.anchor_keyframe_ids),
                "visits": self._room_visits.get(room.room_id, 0),
                "active_failures": fail_cnt,
                "map_revision": self.map_revision,
            }
            if dist_m is not None:
                summary["distance_m"] = round(dist_m, 2)
            scored.append((score, summary))

        scored.sort(key=lambda item: (-item[0], item[1]["room_id"]))
        return [item[1] for item in scored[: max(1, int(top_k))]]

    def l2_submap_entities(
        self,
        *,
        room_id: str | None = None,
        label: str | None = None,
        near_xyz: Sequence[float] | None = None,
        radius_m: float | None = None,
        include_stale: bool = False,
    ) -> dict[str, Any]:
        """L2: exact keyframe-anchored objects and failure-ledger entries."""
        q_tokens = _tokenize(label)
        ref_xyz = None if near_xyz is None else _finite_xyz(near_xyz, "near_xyz")
        objects_out: list[dict[str, Any]] = []

        for obj in self.graph.objects.values():
            if room_id is not None and obj.room_id != room_id:
                continue
            meta = self._object_meta.get(obj.object_id)
            is_stale = bool(meta.stale) if meta is not None else False
            if is_stale and not include_stale:
                continue
            match_score = 1.0
            if label is not None:
                match_score = max(
                    _label_match_score(label, obj.label),
                    100.0 if str(label).strip().lower() == obj.object_id.lower() else 0.0,
                )
                if match_score <= 0.0:
                    continue
            dist_m = None
            if ref_xyz is not None:
                dist_m = float(
                    np.linalg.norm(np.asarray(obj.world_center_xyz) - np.asarray(ref_xyz))
                )
                if radius_m is not None and dist_m > float(radius_m):
                    continue
            row = {
                "object_id": obj.object_id,
                "label": obj.label,
                "room_id": obj.room_id,
                "floor_id": self._floor_for_object(obj),
                "keyframe_id": obj.anchor_keyframe_id,
                "world_xyz": [round(float(v), 3) for v in obj.world_center_xyz],
                "confidence": round(float(obj.confidence), 3),
                "match_score": round(match_score, 2),
                "map_revision": (meta.map_revision if meta is not None else self.map_revision),
                "stale": is_stale,
            }
            if dist_m is not None:
                row["distance_m"] = round(dist_m, 3)
            objects_out.append(row)
        objects_out.sort(key=lambda r: (-float(r["match_score"]), float(r.get("distance_m", 0.0))))

        failures_out: list[dict[str, Any]] = []
        for rec in self._attempts:
            if not rec.is_failure:
                continue
            if rec.stale and not include_stale:
                continue
            if room_id is not None and rec.room_id != room_id:
                continue
            if ref_xyz is not None and radius_m is not None:
                if _xy_dist(rec.goal_xyz, ref_xyz) > float(radius_m):
                    continue
            failures_out.append(rec.to_dict())

        return {
            "map_revision": self.map_revision,
            "objects": objects_out,
            "failures": failures_out,
        }

    def l3_on_demand_keyframes(
        self,
        *,
        object_id: str | None = None,
        room_id: str | None = None,
        near_xyz: Sequence[float] | None = None,
        max_images: int = 3,
    ) -> list[dict[str, Any]]:
        """L3: return at most max_images relevant keyframe RGB paths on demand."""
        limit = max(1, int(max_images))
        candidate_kfs: list[str] = []

        if object_id is not None and object_id in self.graph.objects:
            obj = self.graph.objects[object_id]
            candidate_kfs.append(obj.anchor_keyframe_id)
            if obj.room_id and obj.room_id in self.graph.rooms:
                candidate_kfs.extend(self.graph.rooms[obj.room_id].anchor_keyframe_ids)
        elif room_id is not None and room_id in self.graph.rooms:
            room: RoomNode = self.graph.rooms[room_id]
            candidate_kfs.extend(room.anchor_keyframe_ids)
        else:
            candidate_kfs.extend(self.graph.keyframe_poses.keys())

        ref_xyz = None if near_xyz is None else _finite_xyz(near_xyz, "near_xyz")
        seen_paths: set[str] = set()
        rows: list[tuple[float, dict[str, Any]]] = []

        for kf_id in candidate_kfs:
            rgb_path = self.graph.keyframe_rgb_paths.get(kf_id)
            if not rgb_path or rgb_path in seen_paths:
                continue
            seen_paths.add(rgb_path)
            pose = self.graph.keyframe_poses.get(kf_id)
            pos = pose[:3, 3] if pose is not None else np.zeros(3, dtype=np.float64)
            dist = 0.0 if ref_xyz is None else float(np.linalg.norm(pos - np.asarray(ref_xyz)))
            rows.append(
                (
                    dist,
                    {
                        "keyframe_id": kf_id,
                        "rgb_path": rgb_path,
                        "pose_xyz": [round(float(v), 3) for v in pos],
                        "map_revision": self.map_revision,
                    },
                )
            )

        if ref_xyz is not None:
            rows.sort(key=lambda item: item[0])
        return [item[1] for item in rows[:limit]]

    def narrative_summary(
        self,
        *,
        robot_xyz: Sequence[float] | None = None,
        target_label: str | None = None,
    ) -> str:
        """Return ONE concise prose paragraph summarizing global spatial memory."""
        n_floors = len(self.graph.floors)
        n_rooms = len(self.graph.rooms)
        n_stairs = len(self.graph.stairs)
        n_doors = len(self.graph.doorways)
        active_objs = [
            obj
            for obj in self.graph.objects.values()
            if not (
                self._object_meta.get(obj.object_id) is not None
                and self._object_meta[obj.object_id].stale
            )
        ]
        active_fails = sum(1 for r in self._attempts if r.is_failure and not r.stale)

        parts: list[str] = [
            f"当前空间记忆版本 rev={self.map_revision}，已构建 {n_floors} 个楼层、"
            f"{n_rooms} 个房间/区域、{n_doors} 个门洞与 {n_stairs} 条楼梯连接，"
            f"共记录 {len(active_objs)} 个有效语义地标。"
        ]

        if robot_xyz is not None and self.graph.rooms:
            pos = np.asarray(_finite_xyz(robot_xyz, "robot_xyz"))
            nearest_room = min(
                self.graph.rooms.values(),
                key=lambda r: float(np.linalg.norm(np.asarray(r.centroid_xyz) - pos)),
            )
            r_dist = float(np.linalg.norm(np.asarray(nearest_room.centroid_xyz) - pos))
            alias_str = (
                f"（{nearest_room.alias}）"
                if nearest_room.alias and nearest_room.alias != nearest_room.room_id
                else ""
            )
            parts.append(
                f"机器人当前最接近 {nearest_room.floor_id}/{nearest_room.room_id}{alias_str}"
                f"（距中心约 {r_dist:.1f} m）。"
            )

        if target_label:
            matched = self.l2_submap_entities(label=target_label, include_stale=False)["objects"]
            if matched:
                if robot_xyz is not None:
                    pos = np.asarray(_finite_xyz(robot_xyz, "robot_xyz"))
                    matched.sort(
                        key=lambda o: float(np.linalg.norm(np.asarray(o["world_xyz"]) - pos))
                    )
                brief = "；".join(
                    f"{o['label']}({o['object_id']}@{o['room_id'] or 'open'}, "
                    f"[{o['world_xyz'][0]:.1f},{o['world_xyz'][1]:.1f},{o['world_xyz'][2]:.1f}])"
                    for o in matched[:3]
                )
                parts.append(
                    f"全局记忆中已命中与目标“{target_label}”相关的地标：{brief}（已转为 M* 直达候选）。"
                )
            else:
                parts.append(f"全局记忆中暂未记录与目标“{target_label}”匹配的地标，需继续探索。")
        elif active_objs:
            top_labels = sorted({o.label for o in active_objs})[:5]
            parts.append(f"代表性已记住地标包括：{', '.join(top_labels)}。")

        if active_fails > 0:
            parts.append(f"当前存在 {active_fails} 处活跃失败黑名单区域（已自动过滤高危候选）。")
        parts.append("如需查询特定房间完整物体列表、精确坐标或历史关键帧原图，请调用 query_memory 工具回想。")
        return "".join(parts)

    def memory_goal_candidates(
        self,
        robot_xyz: Sequence[float],
        *,
        target_label: str | None = None,
        standoff_m: float = 1.0,
        max_candidates: int = 3,
    ) -> list[dict[str, Any]]:
        """Project globally remembered target objects into safe standoff M1..M* goto candidates."""
        pos = np.asarray(_finite_xyz(robot_xyz, "robot_xyz"), dtype=np.float64)
        matches = self.l2_submap_entities(label=target_label, include_stale=False)["objects"]
        if not matches:
            return []

        ranked: list[tuple[float, dict[str, Any]]] = []
        for obj in matches:
            obj_xyz = np.asarray(obj["world_xyz"], dtype=np.float64)
            delta = pos - obj_xyz
            xy_norm = math.hypot(float(delta[0]), float(delta[1]))
            if xy_norm > max(0.25, standoff_m):
                ux, uy = float(delta[0]) / xy_norm, float(delta[1]) / xy_norm
                goal_x = float(obj_xyz[0] + ux * standoff_m)
                goal_y = float(obj_xyz[1] + uy * standoff_m)
            else:
                goal_x, goal_y = float(pos[0]), float(pos[1])
            goal_z = float(obj_xyz[2])
            dist_to_obj = float(np.linalg.norm(obj_xyz - pos))
            ranked.append(
                (
                    -float(obj.get("match_score", 1.0)),
                    dist_to_obj,
                    {
                        "kind": "memory_target",
                        "object_id": obj["object_id"],
                        "label": obj["label"],
                        "room_id": obj["room_id"],
                        "xy": [round(goal_x, 2), round(goal_y, 2)],
                        "z": round(goal_z, 3),
                        "object_xyz": obj["world_xyz"],
                        "dist_m": round(dist_to_obj, 2),
                        "clearance_m": 1.0,
                        "turn_safe": True,
                    },
                )
            )

        ranked.sort(key=lambda item: (item[0], item[1]))
        out: list[dict[str, Any]] = []
        for idx, (_, _, cand) in enumerate(ranked[: max(1, int(max_candidates))], start=1):
            cand["id"] = f"M{idx}"
            out.append(cand)
        return out

    # ------------------------------------------------------------------
    # Pre-STOP Geometric & Evidence Verification
    # ------------------------------------------------------------------

    def verify_stop_claim(
        self,
        robot_xyz: Sequence[float],
        *,
        target_label: str | None = None,
        target_object_id: str | None = None,
        target_room_id: str | None = None,
        stop_radius_m: float | None = None,
        require_fresh: bool = True,
    ) -> StopVerificationResult:
        """Verify whether an agent's STOP claim is geometrically & freshly supported."""
        pos = _finite_xyz(robot_xyz, "robot_xyz")
        max_dist = self.default_stop_radius_m if stop_radius_m is None else float(stop_radius_m)

        candidates: list[ObjectNode] = []
        if target_object_id is not None:
            obj = self.graph.objects.get(target_object_id)
            if obj is not None:
                candidates.append(obj)
        elif target_label is not None:
            scored_objs: list[tuple[float, ObjectNode]] = []
            for obj in self.graph.objects.values():
                if target_room_id is not None and obj.room_id != target_room_id:
                    continue
                ms = _label_match_score(target_label, obj.label)
                if ms > 0.0:
                    scored_objs.append((ms, obj))
            if scored_objs:
                max_ms = max(ms for ms, _ in scored_objs)
                candidates = [obj for ms, obj in scored_objs if ms >= max_ms - 1e-6]

        if candidates:
            best_obj: ObjectNode | None = None
            best_dist = float("inf")
            for obj in candidates:
                dist = float(np.linalg.norm(np.asarray(obj.world_center_xyz) - np.asarray(pos)))
                if dist < best_dist:
                    best_dist = dist
                    best_obj = obj
            assert best_obj is not None
            meta = self._object_meta.get(best_obj.object_id)
            is_stale = bool(meta.stale) if meta is not None else False
            same_floor = abs(float(best_obj.world_center_xyz[2]) - pos[2]) <= self.same_floor_dz_m
            if require_fresh and is_stale:
                self._stale_belief_rejects += 1
                self._premature_stop_rejects += 1
                return StopVerificationResult(
                    allowed=False,
                    reason="target_evidence_stale",
                    matched_object_id=best_obj.object_id,
                    matched_room_id=best_obj.room_id,
                    distance_m=best_dist,
                    same_floor=same_floor,
                    evidence_stale=True,
                )
            if not same_floor:
                self._premature_stop_rejects += 1
                return StopVerificationResult(
                    allowed=False,
                    reason="different_floor_z_mismatch",
                    matched_object_id=best_obj.object_id,
                    matched_room_id=best_obj.room_id,
                    distance_m=best_dist,
                    same_floor=False,
                    evidence_stale=is_stale,
                )
            if best_dist > max_dist:
                self._premature_stop_rejects += 1
                return StopVerificationResult(
                    allowed=False,
                    reason="target_beyond_stop_radius",
                    matched_object_id=best_obj.object_id,
                    matched_room_id=best_obj.room_id,
                    distance_m=best_dist,
                    same_floor=True,
                    evidence_stale=is_stale,
                )
            return StopVerificationResult(
                allowed=True,
                reason="verified_object_proximity",
                matched_object_id=best_obj.object_id,
                matched_room_id=best_obj.room_id,
                distance_m=best_dist,
                same_floor=True,
                evidence_stale=is_stale,
            )

        if target_room_id is not None and target_room_id in self.graph.rooms:
            room = self.graph.rooms[target_room_id]
            dist = float(np.linalg.norm(np.asarray(room.centroid_xyz) - np.asarray(pos)))
            same_floor = abs(float(room.centroid_xyz[2]) - pos[2]) <= self.same_floor_dz_m
            room_half_diag = 0.5 * float(
                np.linalg.norm(
                    np.asarray(room.bounds_max_xyz[:2]) - np.asarray(room.bounds_min_xyz[:2])
                )
            )
            if same_floor and dist <= max(max_dist, room_half_diag):
                return StopVerificationResult(
                    allowed=True,
                    reason="verified_room_occupancy",
                    matched_room_id=room.room_id,
                    distance_m=dist,
                    same_floor=True,
                    evidence_stale=False,
                )
            self._premature_stop_rejects += 1
            return StopVerificationResult(
                allowed=False,
                reason=("different_floor_z_mismatch" if not same_floor else "outside_target_room"),
                matched_room_id=room.room_id,
                distance_m=dist,
                same_floor=same_floor,
                evidence_stale=False,
            )

        self._premature_stop_rejects += 1
        return StopVerificationResult(
            allowed=False,
            reason="no_matching_target_in_memory",
        )

    # ------------------------------------------------------------------
    # Diagnostics & Persistence
    # ------------------------------------------------------------------

    def diagnostic_metrics(self) -> dict[str, Any]:
        """Compute compact memory & failure-governance diagnostic counters."""
        l0_json = json.dumps(self.l0_global_topology(), ensure_ascii=False, separators=(",", ":"))
        total_attempts = len(self._attempts)
        total_failures = sum(1 for r in self._attempts if r.is_failure)
        active_failures = sum(1 for r in self._attempts if r.is_failure and not r.stale)
        stale_failures = sum(1 for r in self._attempts if r.is_failure and r.stale)
        total_visits = sum(self._room_visits.values())
        extra_revisits = sum(max(0, c - 1) for c in self._room_visits.values())
        return {
            "map_revision": self.map_revision,
            "l0_bytes": len(l0_json.encode("utf-8")),
            "rooms": len(self.graph.rooms),
            "objects_total": len(self.graph.objects),
            "objects_stale": sum(1 for m in self._object_meta.values() if m.stale),
            "total_attempts": total_attempts,
            "total_failures": total_failures,
            "active_blacklist_count": active_failures,
            "stale_failures": stale_failures,
            "repeat_failure_attempts": self._repeat_failure_attempts,
            "repeat_failure_rate": (
                round(self._repeat_failure_attempts / total_failures, 4)
                if total_failures > 0
                else 0.0
            ),
            "revisit_waste_rate": (
                round(extra_revisits / total_visits, 4) if total_visits > 0 else 0.0
            ),
            "premature_stop_rejects": self._premature_stop_rejects,
            "stale_belief_rejects": self._stale_belief_rejects,
        }

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": MEMORY_SCHEMA,
            "map_revision": self.map_revision,
            "default_blacklist_radius_m": self.default_blacklist_radius_m,
            "default_stop_radius_m": self.default_stop_radius_m,
            "same_floor_dz_m": self.same_floor_dz_m,
            "scene_graph": self.graph.to_dict(),
            "attempts": [rec.to_dict() for rec in self._attempts],
            "object_meta": {oid: meta.to_dict() for oid, meta in self._object_meta.items()},
            "room_visits": dict(self._room_visits),
            "counters": {
                "repeat_failure_attempts": self._repeat_failure_attempts,
                "premature_stop_rejects": self._premature_stop_rejects,
                "stale_belief_rejects": self._stale_belief_rejects,
            },
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> HierarchicalMemoryStore:
        if data.get("schema") != MEMORY_SCHEMA:
            raise ValueError(f"unsupported memory schema: {data.get('schema')!r}")
        graph = KeyframeSceneGraph.from_dict(data["scene_graph"])
        store = cls(
            graph,
            map_revision=int(data.get("map_revision", 0)),
            default_blacklist_radius_m=float(data.get("default_blacklist_radius_m", 0.65)),
            default_stop_radius_m=float(data.get("default_stop_radius_m", 1.5)),
            same_floor_dz_m=float(data.get("same_floor_dz_m", 1.0)),
        )
        store._attempts = [AttemptRecord.from_dict(r) for r in data.get("attempts", [])]
        store._object_meta = {
            str(k): ObjectEvidenceMeta.from_dict(v)
            for k, v in data.get("object_meta", {}).items()
        }
        store._room_visits = {
            str(k): int(v) for k, v in data.get("room_visits", {}).items()
        }
        counters = data.get("counters", {})
        store._repeat_failure_attempts = int(counters.get("repeat_failure_attempts", 0))
        store._premature_stop_rejects = int(counters.get("premature_stop_rejects", 0))
        store._stale_belief_rejects = int(counters.get("stale_belief_rejects", 0))
        return store

    def save(self, path: str | Path) -> Path:
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(self.to_dict(), indent=2, ensure_ascii=False), encoding="utf-8")
        return out

    @classmethod
    def load(cls, path: str | Path) -> HierarchicalMemoryStore:
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))
