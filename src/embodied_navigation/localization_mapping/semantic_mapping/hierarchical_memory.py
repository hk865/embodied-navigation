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

from embodied_navigation.common.runtime_io import atomic_write_json
from embodied_navigation.localization_mapping.semantic_mapping.scene_graph import (
    KeyframeSceneGraph,
    ObjectNode,
    RoomNode,
)

MEMORY_SCHEMA = "embodied_navigation.hierarchical_memory.v1"

FAILURE_OUTCOMES = frozenset(
    {"failed", "rejected", "timeout", "cancelled", "stuck", "dropped", "expired"}
)
GEOGRAPHIC_FAILURE_OUTCOMES = frozenset(
    {"failed", "timeout", "stuck", "dropped", "expired"}
)
GEOGRAPHIC_FAILURE_ACTIONS = frozenset(
    {"goto", "waypoint", "goto_candidate", "goto_xyz", "maneuver"}
)
NON_GEO_REJECTION_REASONS = frozenset(
    {
        "stale_action_epoch",
        "duplicate_or_stale_sequence",
        "run_id_mismatch",
        "pose_source_mismatch",
        "invalid_ttl",
        "unsupported_action",
        "run_terminal",
        "target_label_mismatch",
        "no_matching_target_in_memory",
        "unverified_self_reported_evidence",
        "target_evidence_stale",
        "different_floor_z_mismatch",
        "target_beyond_stop_radius",
    }
)


def _finite_xyz(values: Sequence[float], name: str = "xyz") -> tuple[float, float, float]:
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    if arr.shape != (3,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must be a finite 3-vector")
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def _tokenize_list(text: str | None) -> list[str]:
    if not text:
        return []
    cleaned = "".join(ch.lower() if ch.isalnum() else " " for ch in str(text))
    return [tok for tok in cleaned.split() if tok]


def _tokenize(text: str | None) -> set[str]:
    return set(_tokenize_list(text))


def _is_contiguous_token_subsequence(short: Sequence[str], long: Sequence[str]) -> bool:
    if not short or len(short) > len(long):
        return False
    n = len(short)
    for idx in range(len(long) - n + 1):
        if tuple(long[idx : idx + n]) == tuple(short):
            return True
    return False


def _label_match_score(query: str | None, candidate_label: str) -> float:
    """Return semantic match specificity (>0 if matched, 0.0 if unrelated).

    Uses whole-token matching so partial character substrings (e.g. 'board' vs
    'cardboard') never produce false matches.
    """
    if not query:
        return 1.0
    q_list = _tokenize_list(query)
    c_list = _tokenize_list(candidate_label)
    if not q_list or not c_list:
        return 0.0
    if q_list == c_list:
        return 100.0
    if _is_contiguous_token_subsequence(q_list, c_list) or _is_contiguous_token_subsequence(
        c_list, q_list
    ):
        return 50.0 + 10.0 * (min(len(q_list), len(c_list)) / max(len(q_list), len(c_list)))
    q_toks = set(q_list)
    c_toks = set(c_list)
    overlap = len(q_toks & c_toks)
    if overlap == 0:
        return 0.0
    recall = overlap / len(q_toks)
    if len(q_toks) >= 2 and recall < 0.5:
        return 0.0
    jaccard = overlap / len(q_toks | c_toks)
    return 20.0 * recall + 10.0 * jaccard


def _label_strict_match(
    target_label: str | None,
    candidate_label: str,
    allowed_aliases: Sequence[str] = (),
) -> bool:
    """Strict whole-token target acceptance matcher for STOP verification."""
    if not target_label:
        return False
    c_list = _tokenize_list(candidate_label)
    if not c_list:
        return False
    c_set = set(c_list)
    references = [target_label, *allowed_aliases]
    for ref in references:
        r_list = _tokenize_list(ref)
        if not r_list:
            continue
        if r_list == c_list:
            return True
        if set(r_list).issubset(c_set):
            return True
    return False


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
    anchor_local_xyz: tuple[float, float, float] | None = None
    recorded_goal_xyz: tuple[float, float, float] | None = None
    blocks_location: bool | None = None

    def __post_init__(self) -> None:
        if self.recorded_goal_xyz is None:
            self.recorded_goal_xyz = self.goal_xyz
        if self.blocks_location is None:
            if self.action not in GEOGRAPHIC_FAILURE_ACTIONS:
                self.blocks_location = False
            elif self.outcome in GEOGRAPHIC_FAILURE_OUTCOMES:
                self.blocks_location = True
            elif self.outcome == "rejected":
                self.blocks_location = self.reason not in NON_GEO_REJECTION_REASONS
            else:
                self.blocks_location = False

    @property
    def is_failure(self) -> bool:
        return self.outcome in FAILURE_OUTCOMES

    @property
    def is_geographic_failure(self) -> bool:
        return bool(self.blocks_location)

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
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
            "blocks_location": self.is_geographic_failure,
        }
        if self.anchor_local_xyz is not None:
            out["anchor_local_xyz"] = [round(v, 4) for v in self.anchor_local_xyz]
        if self.recorded_goal_xyz is not None:
            out["recorded_goal_xyz"] = [round(v, 4) for v in self.recorded_goal_xyz]
        return out

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> AttemptRecord:
        anchor_local = (
            _finite_xyz(data["anchor_local_xyz"], "anchor_local_xyz")
            if data.get("anchor_local_xyz") is not None
            else None
        )
        recorded_goal = (
            _finite_xyz(data["recorded_goal_xyz"], "recorded_goal_xyz")
            if data.get("recorded_goal_xyz") is not None
            else None
        )
        blocks_raw = data.get("blocks_location")
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
            anchor_local_xyz=anchor_local,
            recorded_goal_xyz=recorded_goal,
            blocks_location=(None if blocks_raw is None else bool(blocks_raw)),
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
    producer: str = "hierarchical_memory"
    range_source: str = "memory_anchor"
    evidence_frame: str | None = None
    evidence_stamp_s: float | None = None
    recorded_world_xyz: tuple[float, float, float] | None = None

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "object_id": self.object_id,
            "map_revision": self.map_revision,
            "observed_wall_s": round(self.observed_wall_s, 3),
            "confidence": round(self.confidence, 3),
            "stale": self.stale,
            "stale_reason": self.stale_reason,
            "producer": self.producer,
            "range_source": self.range_source,
            "evidence_frame": self.evidence_frame,
            "evidence_stamp_s": self.evidence_stamp_s,
        }
        if self.recorded_world_xyz is not None:
            out["recorded_world_xyz"] = [round(v, 4) for v in self.recorded_world_xyz]
        return out

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ObjectEvidenceMeta:
        rec_xyz = (
            _finite_xyz(data["recorded_world_xyz"], "recorded_world_xyz")
            if data.get("recorded_world_xyz") is not None
            else None
        )
        stamp_raw = data.get("evidence_stamp_s")
        return cls(
            object_id=str(data["object_id"]),
            map_revision=int(data.get("map_revision", 0)),
            observed_wall_s=float(data.get("observed_wall_s", 0.0)),
            confidence=float(data.get("confidence", 1.0)),
            stale=bool(data.get("stale", False)),
            stale_reason=str(data.get("stale_reason", "")),
            producer=str(data.get("producer", "hierarchical_memory")),
            range_source=str(data.get("range_source", "memory_anchor")),
            evidence_frame=data.get("evidence_frame"),
            evidence_stamp_s=(None if stamp_raw is None else float(stamp_raw)),
            recorded_world_xyz=rec_xyz,
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
    range_source: str | None = None
    evidence_frame: str | None = None
    evidence_stamp_s: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "allowed": self.allowed,
            "reason": self.reason,
            "matched_object_id": self.matched_object_id,
            "matched_room_id": self.matched_room_id,
            "distance_m": (None if self.distance_m is None else round(self.distance_m, 3)),
            "same_floor": self.same_floor,
            "evidence_stale": self.evidence_stale,
            "range_source": self.range_source,
            "evidence_frame": self.evidence_frame,
            "evidence_stamp_s": self.evidence_stamp_s,
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
        map_id: str | None = None,
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
        self.map_id = map_id

        self._attempts: list[AttemptRecord] = []
        self._object_meta: dict[str, ObjectEvidenceMeta] = {}
        self._room_visits: dict[str, int] = {}
        self._repeat_failure_attempts = 0
        self._premature_stop_rejects = 0
        self._stale_belief_rejects = 0
        self._next_step_seq = 1
        self._next_object_seq = 1
        self._next_attempt_seq = 1

        for obj_id, obj in self.graph.objects.items():
            self._object_meta[obj_id] = ObjectEvidenceMeta(
                object_id=obj_id,
                map_revision=self.map_revision,
                producer=obj.producer,
                recorded_world_xyz=tuple(float(v) for v in obj.world_center_xyz),
            )

    def allocate_step_seq(self) -> int:
        seq = self._next_step_seq
        self._next_step_seq += 1
        return seq

    def allocate_object_seq(self) -> int:
        seq = self._next_object_seq
        self._next_object_seq += 1
        return seq

    def clear_episode_attempts(self) -> None:
        """Reset transient per-episode attempts when reusing a prior map in a new episode."""
        self._attempts.clear()
        self._room_visits.clear()
        self._repeat_failure_attempts = 0
        self._premature_stop_rejects = 0
        self._stale_belief_rejects = 0

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
        range_source: str = "memory_anchor",
        evidence_frame: str | None = None,
        evidence_stamp_s: float | None = None,
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
            producer=str(producer),
            range_source=str(range_source),
            evidence_frame=evidence_frame,
            evidence_stamp_s=(None if evidence_stamp_s is None else float(evidence_stamp_s)),
            recorded_world_xyz=tuple(float(v) for v in node.world_center_xyz),
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
        """Reproject scene graph from backend loop poses and advance map_revision.

        Uses true SE(3) anchor reprojection against each record's creation-time
        world position so pure rotations and cumulative small shifts properly
        invalidate stale geographic failure records. Repeated calls with
        identical poses are idempotent and do not bump map_revision.
        """
        shifts_m: dict[str, float] = {}
        max_pose_delta = 0.0
        for kf_id, new_pose in updated_poses.items():
            old_pose = self.graph.keyframe_poses.get(kf_id)
            if old_pose is not None:
                arr_new = np.asarray(new_pose, dtype=np.float64)
                if arr_new.shape == (4, 4) and np.all(np.isfinite(arr_new)):
                    shifts_m[kf_id] = float(np.linalg.norm(arr_new[:3, 3] - old_pose[:3, 3]))
                    pose_delta = float(np.linalg.norm(arr_new - old_pose))
                    if pose_delta > max_pose_delta:
                        max_pose_delta = pose_delta

        if max_pose_delta <= 1e-9:
            return {
                "map_revision": self.map_revision,
                "changed": False,
                "max_shift_m": 0.0,
                "invalidated_failures": 0,
                "stale_objects": 0,
                "reproject_report": {"updated_keyframes": 0},
            }

        report = self.graph.reproject_from_poses(updated_poses)
        self.map_revision += 1

        max_effective_shift = max(shifts_m.values(), default=0.0)
        invalidated_failures = 0
        for rec in self._attempts:
            if rec.stale or not rec.is_geographic_failure:
                continue
            eff_shift = 0.0
            if rec.keyframe_id and rec.keyframe_id in self.graph.keyframe_poses:
                if rec.anchor_local_xyz is not None:
                    kf_pose = self.graph.keyframe_poses[rec.keyframe_id]
                    local_h = np.array(
                        [rec.anchor_local_xyz[0], rec.anchor_local_xyz[1], rec.anchor_local_xyz[2], 1.0],
                        dtype=np.float64,
                    )
                    reproj = (kf_pose @ local_h)[:3]
                    orig = np.asarray(rec.recorded_goal_xyz or rec.goal_xyz, dtype=np.float64)
                    eff_shift = float(np.linalg.norm(reproj - orig))
                    rec.goal_xyz = (float(reproj[0]), float(reproj[1]), float(reproj[2]))
                else:
                    eff_shift = shifts_m.get(rec.keyframe_id, 0.0)
            if eff_shift > max_effective_shift:
                max_effective_shift = eff_shift
            if eff_shift >= failure_invalidation_shift_m:
                rec.stale = True
                invalidated_failures += 1

        stale_objects = 0
        if object_staleness_shift_m is not None:
            for obj_id, obj in self.graph.objects.items():
                meta = self._object_meta.get(obj_id)
                if meta is not None and meta.recorded_world_xyz is not None:
                    obj_shift = float(
                        np.linalg.norm(
                            np.asarray(obj.world_center_xyz, dtype=np.float64)
                            - np.asarray(meta.recorded_world_xyz, dtype=np.float64)
                        )
                    )
                else:
                    obj_shift = shifts_m.get(obj.anchor_keyframe_id, 0.0)
                if obj_shift >= object_staleness_shift_m:
                    self.mark_object_stale(obj_id, reason="loop_shift_exceeded")
                    stale_objects += 1

        return {
            "map_revision": self.map_revision,
            "changed": True,
            "max_shift_m": max_effective_shift,
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
        blocks_location: bool | None = None,
    ) -> AttemptRecord:
        """Append an attempt to the ledger and track repeat geographic failures."""
        xyz = _finite_xyz(goal_xyz, "goal_xyz")
        eff_radius = (
            self.default_blacklist_radius_m if radius_m is None else float(radius_m)
        )
        prior_matches = self.match_blacklist(xyz, floor_id=floor_id, radius_m=eff_radius)

        anchor_local_xyz: tuple[float, float, float] | None = None
        if keyframe_id and keyframe_id in self.graph.keyframe_poses:
            kf_pose = self.graph.keyframe_poses[keyframe_id]
            try:
                inv_pose = np.linalg.inv(kf_pose)
                local_pt = (inv_pose @ np.array([xyz[0], xyz[1], xyz[2], 1.0], dtype=np.float64))[:3]
                if np.all(np.isfinite(local_pt)):
                    anchor_local_xyz = (float(local_pt[0]), float(local_pt[1]), float(local_pt[2]))
            except np.linalg.LinAlgError:
                anchor_local_xyz = None

        att_seq = self._next_attempt_seq
        self._next_attempt_seq += 1
        rec = AttemptRecord(
            attempt_id=f"att_{att_seq:04d}",
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
            anchor_local_xyz=anchor_local_xyz,
            recorded_goal_xyz=xyz,
            blocks_location=blocks_location,
        )
        if rec.is_geographic_failure and prior_matches:
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
        """Return active geographic failure records overlapping a proposed goal location."""
        xyz = _finite_xyz(goal_xyz, "goal_xyz")
        matches: list[AttemptRecord] = []
        for rec in self._attempts:
            if rec.stale or not rec.is_geographic_failure:
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
        active_failures = sum(1 for r in self._attempts if r.is_geographic_failure and not r.stale)
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
                if rec.is_geographic_failure and not rec.stale and rec.room_id == room.room_id
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
            if not rec.is_geographic_failure:
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

    def _support_z_for_object(self, obj: ObjectNode) -> float:
        """Return navigable floor/support Z for an object (anchor KF or room centroid)."""
        kf_pose = self.graph.keyframe_poses.get(obj.anchor_keyframe_id)
        if kf_pose is not None and np.all(np.isfinite(kf_pose[:3, 3])):
            return float(kf_pose[2, 3])
        if obj.room_id and obj.room_id in self.graph.rooms:
            return float(self.graph.rooms[obj.room_id].centroid_xyz[2])
        return float(obj.world_center_xyz[2])

    def _floor_for_robot(self, robot_xyz: Sequence[float]) -> str | None:
        pos = np.asarray(_finite_xyz(robot_xyz, "robot_xyz"), dtype=np.float64)
        if self.graph.rooms:
            best_room = min(
                self.graph.rooms.values(),
                key=lambda r: (
                    abs(float(r.centroid_xyz[2]) - float(pos[2])) > self.same_floor_dz_m,
                    float(np.linalg.norm(np.asarray(r.centroid_xyz) - pos)),
                ),
            )
            if abs(float(best_room.centroid_xyz[2]) - float(pos[2])) <= self.same_floor_dz_m:
                return best_room.floor_id
        if self.graph.floors:
            best_floor = min(
                self.graph.floors.values(),
                key=lambda f: abs(float(f.z_mean_m) - float(pos[2])),
            )
            if abs(float(best_floor.z_mean_m) - float(pos[2])) <= self.same_floor_dz_m:
                return best_floor.floor_id
        return None

    def narrative_summary(
        self,
        *,
        robot_xyz: Sequence[float] | None = None,
        target_label: str | None = None,
        memory_candidates: Sequence[dict[str, Any]] | None = None,
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
        active_fails = sum(1 for r in self._attempts if r.is_geographic_failure and not r.stale)

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
            if memory_candidates is not None:
                cands = list(memory_candidates)
            elif robot_xyz is not None:
                cands = self.memory_goal_candidates(robot_xyz, target_label=target_label)
            else:
                cands = []
            if cands:
                brief = "；".join(
                    f"{c.get('id', 'M?')}: {c['label']}({c['object_id']}@{c.get('room_id') or 'open'}, "
                    f"[{c['object_xyz'][0]:.1f},{c['object_xyz'][1]:.1f},{c['object_xyz'][2]:.1f}])"
                    for c in cands[:3]
                )
                parts.append(
                    f"全局记忆中已命中与目标“{target_label}”相关的地标：{brief}。"
                )
            else:
                matched = self.l2_submap_entities(label=target_label, include_stale=False)["objects"]
                if matched:
                    brief = "；".join(
                        f"{o['label']}({o['object_id']}@{o['room_id'] or 'open'}, "
                        f"[{o['world_xyz'][0]:.1f},{o['world_xyz'][1]:.1f},{o['world_xyz'][2]:.1f}])"
                        for o in matched[:3]
                    )
                    parts.append(
                        f"全局记忆中已命中与目标“{target_label}”相关的地标：{brief}。"
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

        ranked: list[tuple[float, float, dict[str, Any]]] = []
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
            node = self.graph.objects.get(obj["object_id"])
            goal_z = (
                self._support_z_for_object(node)
                if node is not None
                else float(obj_xyz[2])
            )
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
                        "keyframe_id": obj.get("keyframe_id"),
                        "xy": [round(goal_x, 2), round(goal_y, 2)],
                        "z": round(goal_z, 3),
                        "object_xyz": obj["world_xyz"],
                        "dist_m": round(dist_to_obj, 2),
                        "clearance_m": None,
                        "clearance_known": False,
                        "turn_safe": None,
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
        allowed_aliases: Sequence[str] = (),
        excluded_producers: Iterable[str] = (),
    ) -> StopVerificationResult:
        """Verify whether an agent's STOP claim is geometrically & freshly supported."""
        pos = _finite_xyz(robot_xyz, "robot_xyz")
        max_dist = self.default_stop_radius_m if stop_radius_m is None else float(stop_radius_m)
        excluded_set = {str(p).strip() for p in excluded_producers if str(p).strip()}
        robot_floor = self._floor_for_robot(pos)

        object_requested = (target_object_id is not None) or (target_label is not None)
        if object_requested:
            raw_matches: list[ObjectNode] = []
            if target_object_id is not None:
                obj = self.graph.objects.get(target_object_id)
                if obj is not None:
                    raw_matches.append(obj)
            elif target_label is not None:
                for obj in self.graph.objects.values():
                    if target_room_id is not None and obj.room_id != target_room_id:
                        continue
                    if _label_strict_match(target_label, obj.label, allowed_aliases):
                        raw_matches.append(obj)

            if not raw_matches:
                self._premature_stop_rejects += 1
                return StopVerificationResult(
                    allowed=False,
                    reason="no_matching_target_in_memory",
                )

            verified_matches: list[ObjectNode] = []
            for obj in raw_matches:
                meta = self._object_meta.get(obj.object_id)
                prod = meta.producer if meta is not None else obj.producer
                if prod in excluded_set or obj.producer in excluded_set:
                    continue
                verified_matches.append(obj)

            if not verified_matches:
                self._premature_stop_rejects += 1
                return StopVerificationResult(
                    allowed=False,
                    reason="unverified_self_reported_evidence",
                )

            fresh_matches: list[ObjectNode] = []
            stale_matches: list[ObjectNode] = []
            for obj in verified_matches:
                meta = self._object_meta.get(obj.object_id)
                if meta is not None and meta.stale:
                    stale_matches.append(obj)
                else:
                    fresh_matches.append(obj)

            if require_fresh and not fresh_matches:
                nearest_stale = min(
                    stale_matches,
                    key=lambda o: float(np.linalg.norm(np.asarray(o.world_center_xyz) - np.asarray(pos))),
                )
                dist = float(np.linalg.norm(np.asarray(nearest_stale.world_center_xyz) - np.asarray(pos)))
                obj_floor = self._floor_for_object(nearest_stale)
                support_z = self._support_z_for_object(nearest_stale)
                same_floor = (
                    (robot_floor == obj_floor)
                    if (robot_floor is not None and obj_floor is not None)
                    else (abs(support_z - pos[2]) <= self.same_floor_dz_m)
                )
                meta = self._object_meta.get(nearest_stale.object_id)
                self._stale_belief_rejects += 1
                self._premature_stop_rejects += 1
                return StopVerificationResult(
                    allowed=False,
                    reason="target_evidence_stale",
                    matched_object_id=nearest_stale.object_id,
                    matched_room_id=nearest_stale.room_id,
                    distance_m=dist,
                    same_floor=same_floor,
                    evidence_stale=True,
                    range_source=(meta.range_source if meta is not None else "memory_anchor"),
                    evidence_frame=(meta.evidence_frame if meta is not None else None),
                    evidence_stamp_s=(meta.evidence_stamp_s if meta is not None else None),
                )

            pool = fresh_matches if require_fresh else verified_matches
            evaluated: list[tuple[bool, float, ObjectNode, bool]] = []
            for obj in pool:
                dist = float(np.linalg.norm(np.asarray(obj.world_center_xyz) - np.asarray(pos)))
                obj_floor = self._floor_for_object(obj)
                support_z = self._support_z_for_object(obj)
                same_floor = (
                    (robot_floor == obj_floor)
                    if (robot_floor is not None and obj_floor is not None)
                    else (abs(support_z - pos[2]) <= self.same_floor_dz_m)
                )
                meta = self._object_meta.get(obj.object_id)
                is_stale = bool(meta.stale) if meta is not None else False
                evaluated.append((same_floor, dist, obj, is_stale))

            # Prefer same-floor candidates first, then shortest distance
            evaluated.sort(key=lambda item: (not item[0], item[1]))
            same_floor, best_dist, best_obj, is_stale = evaluated[0]
            meta = self._object_meta.get(best_obj.object_id)
            r_src = meta.range_source if meta is not None else "memory_anchor"
            ev_frame = meta.evidence_frame if meta is not None else None
            ev_stamp = meta.evidence_stamp_s if meta is not None else None

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
                    range_source=r_src,
                    evidence_frame=ev_frame,
                    evidence_stamp_s=ev_stamp,
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
                    range_source=r_src,
                    evidence_frame=ev_frame,
                    evidence_stamp_s=ev_stamp,
                )
            return StopVerificationResult(
                allowed=True,
                reason="verified_object_proximity",
                matched_object_id=best_obj.object_id,
                matched_room_id=best_obj.room_id,
                distance_m=best_dist,
                same_floor=True,
                evidence_stale=is_stale,
                range_source=r_src,
                evidence_frame=ev_frame,
                evidence_stamp_s=ev_stamp,
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
                    range_source="room_occupancy",
                )
            self._premature_stop_rejects += 1
            return StopVerificationResult(
                allowed=False,
                reason=("different_floor_z_mismatch" if not same_floor else "outside_target_room"),
                matched_room_id=room.room_id,
                distance_m=dist,
                same_floor=same_floor,
                evidence_stale=False,
                range_source="room_occupancy",
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
        geo_failures = sum(1 for r in self._attempts if r.is_geographic_failure)
        active_failures = sum(1 for r in self._attempts if r.is_geographic_failure and not r.stale)
        stale_failures = sum(1 for r in self._attempts if r.is_geographic_failure and r.stale)
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
            "geographic_failures": geo_failures,
            "active_blacklist_count": active_failures,
            "stale_failures": stale_failures,
            "repeat_failure_attempts": self._repeat_failure_attempts,
            "repeat_failure_rate": (
                round(self._repeat_failure_attempts / geo_failures, 4)
                if geo_failures > 0
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
            "map_id": self.map_id,
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
                "next_step_seq": self._next_step_seq,
                "next_object_seq": self._next_object_seq,
                "next_attempt_seq": self._next_attempt_seq,
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
            map_id=data.get("map_id"),
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
        store._next_step_seq = int(counters.get("next_step_seq", len(graph.keyframe_poses) + 1))
        store._next_object_seq = int(counters.get("next_object_seq", len(graph.objects) + 1))
        store._next_attempt_seq = int(counters.get("next_attempt_seq", len(store._attempts) + 1))
        return store

    def save(self, path: str | Path) -> Path:
        out = Path(path)
        atomic_write_json(out, self.to_dict())
        return out

    @classmethod
    def load(cls, path: str | Path) -> HierarchicalMemoryStore:
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))
