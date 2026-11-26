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
        "requires_current_frame_visual_confirmation",
    }
)


SUPPORT_SURFACE_KEYWORDS = frozenset(
    {
        "table",
        "desk",
        "counter",
        "countertop",
        "nightstand",
        "shelf",
        "shelves",
        "dresser",
        "cabinet",
        "couch",
        "sofa",
        "bed",
        "bench",
        "island",
        "vanity",
        "sink",
        "mantel",
        "windowsill",
        "stand",
    }
)


def _normalize_embedding(vec: Sequence[float] | None) -> tuple[float, ...] | None:
    if vec is None:
        return None
    arr = np.asarray(vec, dtype=np.float64).reshape(-1)
    if arr.size == 0 or not np.all(np.isfinite(arr)):
        return None
    norm = float(np.linalg.norm(arr))
    if norm <= 1e-12:
        return None
    return tuple(float(v) for v in (arr / norm))


def _cosine_similarity(
    a: Sequence[float] | None, b: Sequence[float] | None
) -> float | None:
    if a is None or b is None or len(a) != len(b) or len(a) == 0:
        return None
    va = np.asarray(a, dtype=np.float64)
    vb = np.asarray(b, dtype=np.float64)
    na = float(np.linalg.norm(va))
    nb = float(np.linalg.norm(vb))
    if na <= 1e-12 or nb <= 1e-12:
        return None
    return float(np.dot(va, vb) / (na * nb))


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
class KeyframeRichMeta:
    """Rich multi-faceted index attached to one SLAM keyframe for coarse-to-fine search."""

    keyframe_id: str
    room_type: str = ""
    scene_description: str = ""
    surfaces: tuple[str, ...] = ()
    embedding: tuple[float, ...] | None = None
    map_revision: int = 0
    observed_wall_s: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "keyframe_id": self.keyframe_id,
            "room_type": self.room_type,
            "scene_description": self.scene_description,
            "surfaces": list(self.surfaces),
            "map_revision": self.map_revision,
            "observed_wall_s": round(self.observed_wall_s, 3),
        }
        if self.embedding is not None:
            out["embedding"] = [round(v, 5) for v in self.embedding]
        return out

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> KeyframeRichMeta:
        raw_surfaces = data.get("surfaces") or ()
        surfaces = tuple(str(s).strip() for s in raw_surfaces if str(s).strip())
        return cls(
            keyframe_id=str(data["keyframe_id"]),
            room_type=str(data.get("room_type", "")).strip(),
            scene_description=str(data.get("scene_description", "")).strip(),
            surfaces=surfaces,
            embedding=_normalize_embedding(data.get("embedding")),
            map_revision=int(data.get("map_revision", 0)),
            observed_wall_s=float(data.get("observed_wall_s", 0.0)),
        )


@dataclass
class ObjectEvidenceMeta:
    """Revision, freshness, and rich indexing metadata attached to a KeyframeSceneGraph object."""

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
    description: str = ""
    attributes: tuple[str, ...] = ()
    spatial_context: str = ""
    embedding: tuple[float, ...] | None = None
    observation_count: int = 1
    last_observed_keyframe_id: str | None = None
    last_verified_label: str = ""
    last_verified_world_xyz: tuple[float, float, float] | None = None
    keyframe_local_obs: dict[str, tuple[float, float, float]] = field(default_factory=dict)

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
            "observation_count": int(self.observation_count),
        }
        if self.recorded_world_xyz is not None:
            out["recorded_world_xyz"] = [round(v, 4) for v in self.recorded_world_xyz]
        if self.description:
            out["description"] = self.description
        if self.attributes:
            out["attributes"] = list(self.attributes)
        if self.spatial_context:
            out["spatial_context"] = self.spatial_context
        if self.embedding is not None:
            out["embedding"] = [round(v, 5) for v in self.embedding]
        if self.last_observed_keyframe_id is not None:
            out["last_observed_keyframe_id"] = self.last_observed_keyframe_id
        if self.last_verified_label:
            out["last_verified_label"] = self.last_verified_label
        if self.last_verified_world_xyz is not None:
            out["last_verified_world_xyz"] = [round(v, 4) for v in self.last_verified_world_xyz]
        if self.keyframe_local_obs:
            out["keyframe_local_obs"] = {
                kid: [round(v, 4) for v in xyz] for kid, xyz in self.keyframe_local_obs.items()
            }
        return out

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> ObjectEvidenceMeta:
        rec_xyz = (
            _finite_xyz(data["recorded_world_xyz"], "recorded_world_xyz")
            if data.get("recorded_world_xyz") is not None
            else None
        )
        last_ver_xyz = (
            _finite_xyz(data["last_verified_world_xyz"], "last_verified_world_xyz")
            if data.get("last_verified_world_xyz") is not None
            else None
        )
        raw_kf_obs = data.get("keyframe_local_obs") or {}
        kf_obs: dict[str, tuple[float, float, float]] = {}
        if isinstance(raw_kf_obs, dict):
            for kid, xyz in raw_kf_obs.items():
                try:
                    kf_obs[str(kid)] = _finite_xyz(xyz, "keyframe_local_obs")
                except ValueError:
                    continue
        stamp_raw = data.get("evidence_stamp_s")
        raw_attrs = data.get("attributes") or ()
        attrs = tuple(str(a).strip() for a in raw_attrs if str(a).strip())
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
            description=str(data.get("description", "")).strip(),
            attributes=attrs,
            spatial_context=str(data.get("spatial_context", "")).strip(),
            embedding=_normalize_embedding(data.get("embedding")),
            observation_count=max(1, int(data.get("observation_count", 1))),
            last_observed_keyframe_id=data.get("last_observed_keyframe_id"),
            last_verified_label=str(data.get("last_verified_label", "")).strip(),
            last_verified_world_xyz=last_ver_xyz,
            keyframe_local_obs=kf_obs,
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
        self._keyframe_meta: dict[str, KeyframeRichMeta] = {}
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
                last_observed_keyframe_id=obj.anchor_keyframe_id,
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

    def annotate_keyframe(
        self,
        keyframe_id: str,
        *,
        room_type: str = "",
        scene_description: str = "",
        surfaces: Sequence[str] | None = None,
        embedding: Sequence[float] | None = None,
        wall_s: float | None = None,
    ) -> KeyframeRichMeta:
        """Attach or update rich scene/surface descriptions and visual embedding on a keyframe.

        Passing ``surfaces=None`` preserves existing surfaces; passing an explicit
        sequence replaces them so callers can correct stale surface lists.
        """
        kf_id = str(keyframe_id).strip()
        prev = self._keyframe_meta.get(kf_id)
        new_room = str(room_type or "").strip()[:80]
        new_desc = str(scene_description or "").strip()[:280]
        if surfaces is None:
            resolved_surfaces = prev.surfaces if prev else ()
        else:
            resolved_surfaces = tuple(
                dict.fromkeys(str(s).strip() for s in surfaces if str(s).strip())
            )[:10]
        new_emb = _normalize_embedding(embedding)
        meta = KeyframeRichMeta(
            keyframe_id=kf_id,
            room_type=(new_room or (prev.room_type if prev else "")),
            scene_description=(new_desc or (prev.scene_description if prev else "")),
            surfaces=resolved_surfaces,
            embedding=(new_emb if new_emb is not None else (prev.embedding if prev else None)),
            map_revision=self.map_revision,
            observed_wall_s=(time.time() if wall_s is None else float(wall_s)),
        )
        self._keyframe_meta[kf_id] = meta
        return meta

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
        description: str = "",
        attributes: Sequence[str] = (),
        spatial_context: str = "",
        embedding: Sequence[float] | None = None,
    ) -> ObjectNode:
        """Register an object in the underlying scene graph with version & rich metadata."""
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
        clean_attrs = tuple(
            dict.fromkeys(str(a).strip() for a in attributes if str(a).strip())
        )
        local_t = _finite_xyz(local_xyz, "local_xyz")
        world_t = tuple(float(v) for v in node.world_center_xyz)
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
            recorded_world_xyz=world_t,
            description=str(description or "").strip()[:220],
            attributes=clean_attrs[:8],
            spatial_context=str(spatial_context or "").strip()[:140],
            embedding=_normalize_embedding(embedding),
            observation_count=1,
            last_observed_keyframe_id=keyframe_id,
            last_verified_label=str(label).strip(),
            last_verified_world_xyz=world_t,
            keyframe_local_obs={keyframe_id: local_t},
        )
        return node

    def add_or_fuse_object(
        self,
        label: str,
        keyframe_id: str,
        local_xyz: Sequence[float],
        *,
        object_id: str | None = None,
        local_extent_xyz: Sequence[float] = (0.0, 0.0, 0.0),
        confidence: float = 1.0,
        producer: str = "hierarchical_memory",
        room_id: str | None = None,
        wall_s: float | None = None,
        range_source: str = "memory_anchor",
        evidence_frame: str | None = None,
        evidence_stamp_s: float | None = None,
        description: str = "",
        attributes: Sequence[str] = (),
        spatial_context: str = "",
        embedding: Sequence[float] | None = None,
        fusion_radius_m: float = 0.75,
        unverified_producers: Iterable[str] = ("opencode_vlm", "opencode_vlm_unverified"),
    ) -> tuple[ObjectNode, bool]:
        """Insert a new 3D object or fuse into a matching nearby instance.

        Returns ``(node, fused_into_existing)``. Multi-view fusion prevents duplicate
        fragmentation and tracks ``observation_count`` and ``last_observed_keyframe_id``.
        """
        kf_pose = self.graph.keyframe_poses.get(keyframe_id)
        if kf_pose is None:
            raise ValueError(f"unknown keyframe_id for fusion: {keyframe_id!r}")
        local_arr = np.asarray(_finite_xyz(local_xyz, "local_xyz"), dtype=np.float64)
        world_new = (kf_pose @ np.array([local_arr[0], local_arr[1], local_arr[2], 1.0]))[:3]
        unverified_set = {str(p).strip() for p in unverified_producers if str(p).strip()}
        new_verified = str(producer) not in unverified_set

        best_match: ObjectNode | None = None
        best_dist = float("inf")
        for existing in self.graph.objects.values():
            meta = self._object_meta.get(existing.object_id)
            if meta is not None and meta.stale:
                continue
            if _label_match_score(label, existing.label) < 50.0:
                continue
            ex_xyz = np.asarray(existing.world_center_xyz, dtype=np.float64)
            if abs(float(ex_xyz[2]) - float(world_new[2])) > min(0.8, self.same_floor_dz_m):
                continue
            d_xy = math.hypot(float(ex_xyz[0] - world_new[0]), float(ex_xyz[1] - world_new[1]))
            if d_xy <= float(fusion_radius_m) and d_xy < best_dist:
                best_dist = d_xy
                best_match = existing

        if best_match is None:
            oid = object_id or f"obj_{self.allocate_object_seq():04d}"
            node = self.add_object(
                object_id=oid,
                label=label,
                keyframe_id=keyframe_id,
                local_xyz=local_xyz,
                local_extent_xyz=local_extent_xyz,
                confidence=confidence,
                producer=producer,
                room_id=room_id,
                wall_s=wall_s,
                range_source=range_source,
                evidence_frame=evidence_frame,
                evidence_stamp_s=evidence_stamp_s,
                description=description,
                attributes=attributes,
                spatial_context=spatial_context,
                embedding=embedding,
            )
            return node, False

        meta = self._object_meta.get(best_match.object_id)
        if meta is None:
            meta = ObjectEvidenceMeta(
                object_id=best_match.object_id,
                map_revision=self.map_revision,
                producer=best_match.producer,
                recorded_world_xyz=tuple(float(v) for v in best_match.world_center_xyz),
            )
            self._object_meta[best_match.object_id] = meta

        n_prev = max(1, int(meta.observation_count))
        old_verified = meta.producer not in unverified_set
        is_new_kf = keyframe_id not in meta.keyframe_local_obs
        local_t = (float(local_arr[0]), float(local_arr[1]), float(local_arr[2]))

        # Maintain per-keyframe local observations so non-rigid loop updates reproject fused means accurately
        if new_verified and not old_verified:
            meta.keyframe_local_obs = {keyframe_id: local_t}
            meta.producer = str(producer)
            meta.range_source = str(range_source)
            best_match.producer = str(producer)
            fused_world = world_new
            meta.observation_count = 1
        elif new_verified == old_verified:
            if len(meta.keyframe_local_obs) < 8 or keyframe_id in meta.keyframe_local_obs:
                meta.keyframe_local_obs[keyframe_id] = local_t
            # Recompute fused world center directly from stored per-KF observations
            reproj_pts: list[np.ndarray] = []
            for kid, lxyz in meta.keyframe_local_obs.items():
                kp = self.graph.keyframe_poses.get(kid)
                if kp is not None:
                    reproj_pts.append((kp @ np.array([lxyz[0], lxyz[1], lxyz[2], 1.0]))[:3])
            if reproj_pts:
                fused_world = np.mean(np.vstack(reproj_pts), axis=0)
            else:
                old_xyz = np.asarray(best_match.world_center_xyz, dtype=np.float64)
                fused_world = (old_xyz * n_prev + world_new) / float(n_prev + 1)
            if is_new_kf:
                meta.observation_count = max(n_prev + 1, len(meta.keyframe_local_obs))
        else:
            # Unverified guess fusing into verified object: never alter verified geometry or counts
            fused_world = np.asarray(best_match.world_center_xyz, dtype=np.float64)

        # Re-anchor fused world coordinate cleanly into latest keyframe if verified
        if new_verified or not old_verified:
            try:
                inv_kf = np.linalg.inv(kf_pose)
                fused_local = (inv_kf @ np.array([fused_world[0], fused_world[1], fused_world[2], 1.0]))[:3]
                if np.all(np.isfinite(fused_local)):
                    best_match.anchor_keyframe_id = keyframe_id
                    best_match.local_center_xyz = (
                        float(fused_local[0]),
                        float(fused_local[1]),
                        float(fused_local[2]),
                    )
            except np.linalg.LinAlgError:
                pass
        best_match.world_center_xyz = (
            float(fused_world[0]),
            float(fused_world[1]),
            float(fused_world[2]),
        )
        ext_arr = np.asarray(local_extent_xyz, dtype=np.float64).reshape(-1)
        if (
            (new_verified or not old_verified)
            and ext_arr.shape == (3,)
            and np.all(np.isfinite(ext_arr))
            and float(np.max(ext_arr)) > 0.0
        ):
            old_ext = np.asarray(best_match.local_extent_xyz, dtype=np.float64)
            if float(np.max(old_ext)) <= 0.0:
                best_match.local_extent_xyz = (float(ext_arr[0]), float(ext_arr[1]), float(ext_arr[2]))
            else:
                fused_ext = (old_ext * n_prev + ext_arr) / float(n_prev + 1)
                best_match.local_extent_xyz = (
                    float(fused_ext[0]),
                    float(fused_ext[1]),
                    float(fused_ext[2]),
                )

        # Keep more specific label if new label is a stricter phrase
        if (
            (new_verified or not old_verified)
            and len(_tokenize_list(label)) > len(_tokenize_list(best_match.label))
            and _label_match_score(best_match.label, label) >= 50.0
        ):
            best_match.label = str(label).strip()

        if new_verified or not old_verified:
            meta.confidence = max(float(meta.confidence), float(confidence))
            best_match.confidence = meta.confidence
        meta.observed_wall_s = time.time() if wall_s is None else float(wall_s)
        meta.map_revision = self.map_revision
        meta.recorded_world_xyz = best_match.world_center_xyz
        if new_verified or not old_verified:
            meta.last_observed_keyframe_id = keyframe_id
            meta.last_verified_label = str(label).strip()
            meta.last_verified_world_xyz = (
                float(world_new[0]),
                float(world_new[1]),
                float(world_new[2]),
            )
            if evidence_frame is not None:
                meta.evidence_frame = evidence_frame
            if evidence_stamp_s is not None:
                meta.evidence_stamp_s = float(evidence_stamp_s)
        new_desc = str(description or "").strip()[:220]
        if len(new_desc) > len(meta.description):
            meta.description = new_desc
        new_ctx = str(spatial_context or "").strip()[:140]
        if len(new_ctx) > len(meta.spatial_context):
            meta.spatial_context = new_ctx
        if attributes:
            merged_attrs = tuple(
                dict.fromkeys([*meta.attributes, *(str(a).strip() for a in attributes if str(a).strip())])
            )
            meta.attributes = merged_attrs[:8]
        new_emb = _normalize_embedding(embedding)
        if new_emb is not None:
            if meta.embedding is not None and len(meta.embedding) == len(new_emb):
                avg_emb = (
                    np.asarray(meta.embedding, dtype=np.float64) * n_prev
                    + np.asarray(new_emb, dtype=np.float64)
                ) / float(n_prev + 1)
                meta.embedding = _normalize_embedding(avg_emb)
            else:
                meta.embedding = new_emb
        return best_match, True

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
        # Recompute multi-view fused object world centers from per-KF local observations
        for obj_id, obj in self.graph.objects.items():
            meta = self._object_meta.get(obj_id)
            if meta is not None and len(meta.keyframe_local_obs) >= 2:
                reproj_pts: list[np.ndarray] = []
                for kid, lxyz in meta.keyframe_local_obs.items():
                    kp = self.graph.keyframe_poses.get(kid)
                    if kp is not None:
                        reproj_pts.append((kp @ np.array([lxyz[0], lxyz[1], lxyz[2], 1.0]))[:3])
                if reproj_pts:
                    mean_w = np.mean(np.vstack(reproj_pts), axis=0)
                    obj.world_center_xyz = (float(mean_w[0]), float(mean_w[1]), float(mean_w[2]))
            if (
                meta is not None
                and meta.last_observed_keyframe_id
                and meta.last_observed_keyframe_id in meta.keyframe_local_obs
                and meta.last_observed_keyframe_id in self.graph.keyframe_poses
            ):
                kp_last = self.graph.keyframe_poses[meta.last_observed_keyframe_id]
                l_last = meta.keyframe_local_obs[meta.last_observed_keyframe_id]
                w_last = (kp_last @ np.array([l_last[0], l_last[1], l_last[2], 1.0]))[:3]
                meta.last_verified_world_xyz = (float(w_last[0]), float(w_last[1]), float(w_last[2]))
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
                desc_score = 0.0
                if meta is not None:
                    rich_text = " ".join(
                        p
                        for p in (meta.description, " ".join(meta.attributes), meta.spatial_context)
                        if p
                    )
                    if rich_text:
                        desc_score = 0.75 * _label_match_score(label, rich_text)
                match_score = max(
                    _label_match_score(label, obj.label),
                    desc_score,
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
                "extent_xyz_m": [round(float(v), 3) for v in obj.local_extent_xyz],
                "confidence": round(float(obj.confidence), 3),
                "observation_count": (meta.observation_count if meta is not None else 1),
                "match_score": round(match_score, 2),
                "map_revision": (meta.map_revision if meta is not None else self.map_revision),
                "stale": is_stale,
            }
            if meta is not None:
                if meta.description:
                    row["description"] = meta.description
                if meta.attributes:
                    row["attributes"] = list(meta.attributes)
                if meta.spatial_context:
                    row["spatial_context"] = meta.spatial_context
            if dist_m is not None:
                row["distance_m"] = round(dist_m, 3)
            objects_out.append(row)
        objects_out.sort(
            key=lambda r: (
                -float(r["match_score"]),
                -int(r.get("observation_count", 1)),
                float(r.get("distance_m", 0.0)),
            )
        )

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

    def coarse_to_fine_search(
        self,
        query: str,
        *,
        robot_xyz: Sequence[float] | None = None,
        query_embedding: Sequence[float] | None = None,
        top_k_objects: int = 4,
        top_k_keyframes: int = 3,
        max_l3_images: int = 2,
        spatial_nms_m: float = 0.8,
        expected_max_extent_m: float | None = None,
        temperature: float = 15.0,
        confidence_margin_threshold: float = 0.22,
        confidence_min_score: float = 65.0,
        include_stale: bool = False,
    ) -> dict[str, Any]:
        """Scene-Q style confidence-aware coarse-to-fine retrieval across L1/L2/L3.

        Stage 1 (Coarse): Scores 3D objects using label + rich description/attributes/
        spatial context + multi-view consistency + optional continuous embedding + optional
        3D extent filter, with spatial NMS to preserve distinct 3D hypotheses. Also ranks
        keyframes using rich scene descriptions, visible supporting surfaces (for unsegmented
        small objects like phones/vases), and optional visual embeddings.

        Stage 2 (Confidence Gate): Computes temperature-scaled softmax margin over top
        object candidates. Sets ``needs_l3_verification=True`` when matches are absent,
        low-confidence, or ambiguous across multiple competing 3D instances, returning
        deduplicated ``l3_verify_rgb_paths`` for selective VLM/task-conditioned reranking.
        """
        q_str = str(query or "").strip()
        q_tokens = _tokenize(q_str)
        q_emb = _normalize_embedding(query_embedding)
        ref_xyz = None if robot_xyz is None else _finite_xyz(robot_xyz, "robot_xyz")
        k_obj = max(1, int(top_k_objects))
        k_kf = max(1, int(top_k_keyframes))
        temp = max(1e-3, float(temperature))

        # --- Stage 1A: Score 3D object instances ---
        raw_obj_hits: list[tuple[float, dict[str, Any]]] = []
        for obj in self.graph.objects.values():
            meta = self._object_meta.get(obj.object_id)
            is_stale = bool(meta.stale) if meta is not None else False
            if is_stale and not include_stale:
                continue

            lbl_score = _label_match_score(q_str, obj.label) if q_str else 1.0
            if q_str and _label_strict_match(q_str, obj.label):
                # Avoid penalizing richer multi-word labels (e.g. 'glass vase' vs 'vase')
                # by a 45-point cliff that would mask competing 3D instances.
                q_n = max(1, len(q_tokens))
                c_n = max(q_n, len(_tokenize_list(obj.label)))
                lbl_score = max(lbl_score, 85.0 + 15.0 * (q_n / float(c_n)))
            desc_score = 0.0
            ctx_score = 0.0
            emb_bonus = 0.0
            obs_cnt = 1
            if meta is not None:
                obs_cnt = max(1, int(meta.observation_count))
                desc_text = " ".join(p for p in (meta.description, " ".join(meta.attributes)) if p)
                if q_str and desc_text:
                    desc_score = 0.85 * _label_match_score(q_str, desc_text)
                if q_str and meta.spatial_context:
                    ctx_score = 0.55 * _label_match_score(q_str, meta.spatial_context)
                cos = _cosine_similarity(q_emb, meta.embedding)
                if cos is not None and cos > 0.0:
                    emb_bonus = 60.0 * cos

            base_sem = max(lbl_score, desc_score, ctx_score) + 0.15 * min(lbl_score, desc_score) + emb_bonus
            if base_sem <= 0.0:
                continue

            # Multi-view fusion bonus (rewards instances confirmed across multiple frames)
            mv_bonus = 8.0 * min(1.0, math.log1p(obs_cnt - 1) / math.log1p(3))
            # Optional 3D extent filter (penalize oversized mismatches, e.g. 2m couch for 'vase')
            extent_penalty = 0.0
            max_ext = float(max(obj.local_extent_xyz)) if obj.local_extent_xyz else 0.0
            if expected_max_extent_m is not None and max_ext > float(expected_max_extent_m) * 1.5:
                extent_penalty = 25.0

            dist_m = None
            prox_bonus = 0.0
            if ref_xyz is not None:
                dist_m = float(
                    np.linalg.norm(np.asarray(obj.world_center_xyz) - np.asarray(ref_xyz))
                )
                prox_bonus = 2.0 / (1.0 + 0.25 * dist_m)

            total_score = max(0.0, base_sem + mv_bonus + prox_bonus - extent_penalty)
            kf_best = (
                meta.last_observed_keyframe_id
                if (meta is not None and meta.last_observed_keyframe_id)
                else obj.anchor_keyframe_id
            )
            rgb_path = self.graph.keyframe_rgb_paths.get(kf_best) or self.graph.keyframe_rgb_paths.get(
                obj.anchor_keyframe_id
            )
            row: dict[str, Any] = {
                "object_id": obj.object_id,
                "label": obj.label,
                "room_id": obj.room_id,
                "floor_id": self._floor_for_object(obj),
                "keyframe_id": kf_best,
                "rgb_path": rgb_path,
                "world_xyz": [round(float(v), 3) for v in obj.world_center_xyz],
                "extent_xyz_m": [round(float(v), 3) for v in obj.local_extent_xyz],
                "observation_count": obs_cnt,
                "confidence": round(float(obj.confidence), 3),
                "coarse_score": round(total_score, 2),
                "label_score": round(lbl_score, 2),
            }
            if meta is not None:
                if meta.description:
                    row["description"] = meta.description
                if meta.attributes:
                    row["attributes"] = list(meta.attributes)
                if meta.spatial_context:
                    row["spatial_context"] = meta.spatial_context
            if dist_m is not None:
                row["distance_m"] = round(dist_m, 2)
            raw_obj_hits.append((total_score, row))

        raw_obj_hits.sort(key=lambda item: (-item[0], float(item[1].get("distance_m", 0.0))))
        # Spatial NMS over all distinct 3D hypotheses BEFORE top-K display truncation
        nms_objects: list[dict[str, Any]] = []
        for _score, cand in raw_obj_hits:
            c_xyz = cand["world_xyz"]
            if any(
                abs(c_xyz[2] - prev["world_xyz"][2]) <= 0.8
                and _xy_dist(c_xyz, prev["world_xyz"]) < float(spatial_nms_m)
                for prev in nms_objects
            ):
                continue
            nms_objects.append(cand)
        top_objects = nms_objects[:k_obj]

        # --- Stage 1B: Score Keyframes (rich descriptions + surface/container priors) ---
        kf_scores: list[tuple[float, dict[str, Any]]] = []
        for kf_id, pose in self.graph.keyframe_poses.items():
            rgb_path = self.graph.keyframe_rgb_paths.get(kf_id)
            if not rgb_path:
                continue
            kmeta = self._keyframe_meta.get(kf_id)
            score = 0.0
            reasons: list[str] = []

            if kmeta is not None:
                if q_str and kmeta.scene_description:
                    s_desc = _label_match_score(q_str, kmeta.scene_description)
                    if s_desc > 0.0:
                        score += s_desc
                        reasons.append("scene_description")
                if q_str and kmeta.room_type:
                    s_room = 0.6 * _label_match_score(q_str, kmeta.room_type)
                    if s_room > 0.0:
                        score += s_room
                        reasons.append("room_type")
                if q_str and kmeta.surfaces:
                    surf_joined = " ".join(kmeta.surfaces)
                    s_surf = 0.8 * _label_match_score(q_str, surf_joined)
                    if s_surf > 0.0:
                        score += s_surf
                        reasons.append("surface_match")
                    else:
                        surf_toks = _tokenize(surf_joined)
                        if surf_toks & SUPPORT_SURFACE_KEYWORDS:
                            # Container/support surface prior for unsegmented small objects
                            score += 6.0 + min(4.0, 1.0 * len(kmeta.surfaces))
                            reasons.append("support_surface_prior")
                cos_kf = _cosine_similarity(q_emb, kmeta.embedding)
                if cos_kf is not None and cos_kf > 0.0:
                    score += 60.0 * cos_kf
                    reasons.append("visual_embedding")

            # Also reward keyframes hosting matched L2 objects or known supporting furniture
            kf_objs = [
                o
                for o in self.graph.objects.values()
                if o.anchor_keyframe_id == kf_id
                or (
                    self._object_meta.get(o.object_id) is not None
                    and self._object_meta[o.object_id].last_observed_keyframe_id == kf_id
                )
            ]
            for o in kf_objs:
                ometa = self._object_meta.get(o.object_id)
                if ometa is not None and ometa.stale and not include_stale:
                    continue
                if q_str and _label_match_score(q_str, o.label) > 0.0:
                    score += 25.0
                    if "linked_object" not in reasons:
                        reasons.append("linked_object")
                elif _tokenize(o.label) & SUPPORT_SURFACE_KEYWORDS:
                    score += 4.0
                    if "object_surface_prior" not in reasons:
                        reasons.append("object_surface_prior")

            if score <= 0.0:
                continue
            pos = pose[:3, 3]
            dist_kf = None
            if ref_xyz is not None:
                dist_kf = float(np.linalg.norm(pos - np.asarray(ref_xyz)))
                score += 1.5 / (1.0 + 0.25 * dist_kf)
            kf_entry: dict[str, Any] = {
                "keyframe_id": kf_id,
                "rgb_path": rgb_path,
                "pose_xyz": [round(float(v), 3) for v in pos],
                "coarse_score": round(score, 2),
                "match_reasons": reasons,
            }
            if kmeta is not None:
                if kmeta.room_type:
                    kf_entry["room_type"] = kmeta.room_type
                if kmeta.scene_description:
                    kf_entry["scene_description"] = kmeta.scene_description
                if kmeta.surfaces:
                    kf_entry["surfaces"] = list(kmeta.surfaces)
            if dist_kf is not None:
                kf_entry["distance_m"] = round(dist_kf, 2)
            kf_scores.append((score, kf_entry))

        kf_scores.sort(key=lambda item: (-item[0], float(item[1].get("distance_m", 0.0))))
        top_keyframes: list[dict[str, Any]] = []
        seen_kf_rgb: set[str] = set()
        for _sc, entry in kf_scores:
            if entry["rgb_path"] in seen_kf_rgb:
                continue
            seen_kf_rgb.add(entry["rgb_path"])
            top_keyframes.append(entry)
            if len(top_keyframes) >= k_kf:
                break

        # --- Stage 2: Scene-Q Temperature-Scaled Confidence Gate ---
        # Compute softmax & margin over ALL spatially-NMS'd competitors BEFORE top_k truncation.
        unverified_producers = {"opencode_vlm", "opencode_vlm_unverified"}
        if nms_objects:
            logits = np.array([float(o["coarse_score"]) / temp for o in nms_objects], dtype=np.float64)
            logits -= float(np.max(logits))
            probs = np.exp(logits)
            probs /= float(np.sum(probs))
            for obj_row, p in zip(top_objects, probs[: len(top_objects)]):
                obj_row["prob"] = round(float(p), 3)
            top1_prob = float(probs[0])
            margin = float(probs[0] - probs[1]) if len(nms_objects) >= 2 else top1_prob
            top1_obj = nms_objects[0]
            top1_score = float(top1_obj["coarse_score"])
            top1_obs = int(top1_obj.get("observation_count", 1))
            top1_conf = float(top1_obj.get("confidence", 0.0))
            top1_meta = self._object_meta.get(str(top1_obj["object_id"]))
            top1_prod = top1_meta.producer if top1_meta is not None else ""
            top1_unverified = top1_prod in unverified_producers
            needs_l3 = (
                top1_unverified
                or top1_conf < 0.65
                or top1_score < float(confidence_min_score)
                or (len(nms_objects) >= 2 and margin < float(confidence_margin_threshold))
                or (top1_obs <= 1 and float(top1_obj.get("label_score", 0.0)) < 100.0)
            )
        else:
            top1_prob = 0.0
            margin = 0.0
            needs_l3 = bool(top_keyframes)

        # Deduplicated Top-K L3 RGB paths for selective VLM / task-conditioned verification
        l3_paths: list[str] = []
        if needs_l3:
            for o in nms_objects:
                rp = o.get("rgb_path")
                if isinstance(rp, str) and rp and rp not in l3_paths:
                    l3_paths.append(rp)
                if len(l3_paths) >= max(1, int(max_l3_images)):
                    break
            if len(l3_paths) < max(1, int(max_l3_images)):
                for kf in top_keyframes:
                    rp = kf.get("rgb_path")
                    if isinstance(rp, str) and rp and rp not in l3_paths:
                        l3_paths.append(rp)
                    if len(l3_paths) >= max(1, int(max_l3_images)):
                        break

        return {
            "query": q_str,
            "map_revision": self.map_revision,
            "top_objects": top_objects,
            "top_keyframes": top_keyframes,
            "top1_prob": round(top1_prob, 3),
            "confidence_margin": round(margin, 3),
            "needs_l3_verification": bool(needs_l3),
            "l3_verify_rgb_paths": l3_paths,
        }

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
                    -int(obj.get("observation_count", 1)),
                    dist_to_obj,
                    {
                        "kind": "memory_target",
                        "object_id": obj["object_id"],
                        "label": obj["label"],
                        "description": str(obj.get("description", ""))[:80],
                        "spatial_context": str(obj.get("spatial_context", ""))[:60],
                        "observation_count": int(obj.get("observation_count", 1)),
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

        ranked.sort(key=lambda item: (item[0], item[1], item[2]))
        out: list[dict[str, Any]] = []
        for idx, (_, _, _, cand) in enumerate(ranked[: max(1, int(max_candidates))], start=1):
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
        require_current_keyframe_id: str | None = None,
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
            evaluated: list[tuple[bool, bool, float, ObjectNode, bool]] = []
            for obj in pool:
                meta = self._object_meta.get(obj.object_id)
                is_stale = bool(meta.stale) if meta is not None else False
                in_cur_kf = True
                eval_xyz = np.asarray(obj.world_center_xyz, dtype=np.float64)
                if require_current_keyframe_id is not None:
                    last_kf = meta.last_observed_keyframe_id if meta is not None else None
                    kf_match = (
                        obj.anchor_keyframe_id == require_current_keyframe_id
                        or last_kf == require_current_keyframe_id
                    )
                    # Current-frame STOP must match the CURRENT frame's observed label & measured point
                    # (never allowing a coarser current detection or older closer measurement to vouch).
                    cur_label = (
                        meta.last_verified_label
                        if (meta is not None and meta.last_verified_label)
                        else obj.label
                    )
                    lbl_ok = True if target_label is None else _label_strict_match(
                        target_label, cur_label, allowed_aliases
                    )
                    in_cur_kf = bool(kf_match and lbl_ok)
                    if in_cur_kf and meta is not None and meta.last_verified_world_xyz is not None:
                        eval_xyz = np.asarray(meta.last_verified_world_xyz, dtype=np.float64)
                dist = float(np.linalg.norm(eval_xyz - np.asarray(pos)))
                obj_floor = self._floor_for_object(obj)
                support_z = self._support_z_for_object(obj)
                same_floor = (
                    (robot_floor == obj_floor)
                    if (robot_floor is not None and obj_floor is not None)
                    else (abs(support_z - pos[2]) <= self.same_floor_dz_m)
                )
                evaluated.append((same_floor, in_cur_kf, dist, obj, is_stale))

            # Prefer same-floor + current-frame-confirmed candidates first, then shortest distance
            evaluated.sort(key=lambda item: (not item[0], not item[1], item[2]))
            same_floor, in_cur_kf, best_dist, best_obj, is_stale = evaluated[0]
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
            if require_current_keyframe_id is not None and not in_cur_kf:
                self._premature_stop_rejects += 1
                return StopVerificationResult(
                    allowed=False,
                    reason="requires_current_frame_visual_confirmation",
                    matched_object_id=best_obj.object_id,
                    matched_room_id=best_obj.room_id,
                    distance_m=best_dist,
                    same_floor=True,
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
            "keyframe_meta": {kid: kmeta.to_dict() for kid, kmeta in self._keyframe_meta.items()},
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
        store._keyframe_meta = {
            str(k): KeyframeRichMeta.from_dict(v)
            for k, v in data.get("keyframe_meta", {}).items()
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
