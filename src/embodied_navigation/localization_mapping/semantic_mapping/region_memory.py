"""Geometric-connectivity + semantic global region partitioning and summary archive.

Builds structured indoor regions (stairwell, hallway, living_room, kitchen, dining_room,
bedroom, bathroom, office, lobby) directly on top of ``KeyframeSceneGraph`` geometric
floors, stairs, doorways, and rooms, combined with semantic checks (anchored 3D object
co-occurrence + optional SigLIP room-prototype cosine scores) and Set-Cover minimal
texture keyframes from ``TextureKeyframeSelector``.

Key capabilities:
1. Hard geometric boundaries first: relies on real clearance-derived doorways, wall
   sector checks, and stair slopes in ``KeyframeSceneGraph`` instead of blind Euclidean
   circles across walls.
2. Open-plan semantic split: when one large geometric room (e.g. open living+kitchen
   space with span >= 5.5 m) contains spatially separated, semantically distinct object/
   visual clusters, splits it into adjacent sub-regions linked by an ``open_passage``.
3. Event-driven region summaries: every region maintains a high-level ``primary_label``,
   a rich ``detailed_description``, its minimal Set-Cover ``texture_keyframe_ids``,
   anchored objects, connected neighbor regions, and unexplored exits count. Optional
   VLM visual summarization triggers only when leaving a region or on major updates.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Callable, Mapping, Optional, Sequence, Tuple

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping.hierarchical_memory import (
    HierarchicalMemoryStore,
)
from embodied_navigation.localization_mapping.semantic_mapping.texture_keyframe_selector import (
    TextureKeyframeSelector,
)


ROOM_PROTOTYPE_PROMPTS: dict[str, str] = {
    "living_room": "a photo of a living room with a sofa couch coffee table and television",
    "bedroom": "a photo of a bedroom with a bed pillows mattress nightstand and dresser",
    "kitchen": "a photo of a kitchen with countertops stove refrigerator cabinets and sink",
    "dining_room": "a photo of a dining room with a dining table and dining chairs",
    "bathroom": "a photo of a bathroom with a toilet sink mirror bathtub or shower",
    "office": "a photo of a home office or study with a desk office chair bookshelf and monitor",
    "hallway": "a photo of a narrow indoor hallway or corridor with walls and doors",
    "stairwell": "a photo of an indoor staircase stairwell with steps and handrails",
    "lobby": "a photo of a house entryway foyer or open lobby near a front door",
}

# Deterministic object-to-room semantic priors (word -> {room_type: weight})
_OBJECT_ROOM_PRIORS: dict[str, dict[str, float]] = {
    "bed": {"bedroom": 3.5},
    "mattress": {"bedroom": 3.0},
    "headboard": {"bedroom": 2.5},
    "nightstand": {"bedroom": 2.2},
    "dresser": {"bedroom": 1.6},
    "wardrobe": {"bedroom": 1.8},
    "couch": {"living_room": 3.2},
    "sofa": {"living_room": 3.2},
    "loveseat": {"living_room": 2.8},
    "sectional": {"living_room": 3.0},
    "coffee table": {"living_room": 2.2},
    "armchair": {"living_room": 1.8},
    "tv": {"living_room": 1.8, "bedroom": 0.6},
    "fireplace": {"living_room": 2.2},
    "ottoman": {"living_room": 1.5},
    "dining table": {"dining_room": 3.2},
    "dining chair": {"dining_room": 2.0},
    "refrigerator": {"kitchen": 3.2},
    "fridge": {"kitchen": 3.2},
    "stove": {"kitchen": 3.2},
    "oven": {"kitchen": 3.0},
    "microwave": {"kitchen": 2.4},
    "kitchen island": {"kitchen": 3.0},
    "bar stool": {"kitchen": 1.8, "dining_room": 0.8},
    "counter": {"kitchen": 2.0, "bathroom": 0.8},
    "countertop": {"kitchen": 2.0, "bathroom": 0.8},
    "cabinet": {"kitchen": 1.3, "bathroom": 0.8},
    "toilet": {"bathroom": 4.0},
    "bathtub": {"bathroom": 3.8},
    "shower": {"bathroom": 3.5},
    "vanity": {"bathroom": 2.5},
    "sink": {"bathroom": 1.8, "kitchen": 1.6},
    "desk": {"office": 3.2, "bedroom": 0.7},
    "office chair": {"office": 2.5},
    "monitor": {"office": 2.5},
    "computer": {"office": 2.5},
    "bookshelf": {"office": 1.8, "living_room": 1.0},
    "stair": {"stairwell": 3.0},
    "staircase": {"stairwell": 3.5},
    "stair railing": {"stairwell": 3.0},
    "handrail": {"stairwell": 2.6},
    "table": {"dining_room": 1.1, "living_room": 0.7, "office": 0.6},
    "chair": {"dining_room": 0.9, "living_room": 0.6, "office": 0.6},
}


@dataclass
class RegionArchiveEntry:
    """Structured global memory record for one semantic/geometric indoor region."""

    region_id: str
    floor_id: str
    primary_label: str
    label_confidence: float
    label_source: str
    detailed_description: str
    keyframe_ids: tuple[str, ...] = ()
    texture_keyframe_ids: tuple[str, ...] = ()
    texture_coverage_ratio: float = 0.0
    texture_voxels_count: int = 0
    coverage_status: str = "glimpsed"  # glimpsed | partial | inspected_closeup
    centroid_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    bounds_xy: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    clearance_m: float = 0.0
    objects: tuple[str, ...] = ()
    connected_regions: tuple[dict[str, Any], ...] = ()
    unexplored_exits_count: int = 0
    vlm_summarized: bool = False
    summary_object_count: int = 0
    summary_voxel_count: int = 0
    summary_revision: int = 0
    geom_room_id: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "region_id": self.region_id,
            "zone_id": self.region_id,
            "floor_id": self.floor_id,
            "primary_label": self.primary_label,
            "zone_type": self.primary_label,
            "label_confidence": round(float(self.label_confidence), 3),
            "label_source": self.label_source,
            "detailed_description": self.detailed_description,
            "summary": self.detailed_description,
            "status": self.coverage_status,
            "centroid_xyz": [round(v, 2) for v in self.centroid_xyz],
            "center_xyz": [round(v, 2) for v in self.centroid_xyz],
            "bounds_xy": [round(v, 2) for v in self.bounds_xy],
            "clearance_m": round(float(self.clearance_m), 2),
            "keyframe_count": len(self.keyframe_ids),
            "texture_keyframe_ids": list(self.texture_keyframe_ids),
            "texture_coverage_ratio": round(float(self.texture_coverage_ratio), 3),
            "texture_voxels_count": int(self.texture_voxels_count),
            "objects": list(self.objects),
            "connected_regions": [dict(c) for c in self.connected_regions],
            "unexplored_exits_count": int(self.unexplored_exits_count),
            "vlm_summarized": bool(self.vlm_summarized),
        }


RegionSummarizerFn = Callable[
    [RegionArchiveEntry, Sequence[str]],
    Optional[Tuple[Optional[str], Optional[str]]],
]


def _score_objects_for_room(obj_labels: Sequence[str]) -> dict[str, float]:
    scores: dict[str, float] = {}
    chair_cnt = 0
    table_cnt = 0
    for raw in obj_labels:
        lbl = " ".join(str(raw or "").strip().lower().split())
        if not lbl:
            continue
        if "chair" in lbl:
            chair_cnt += 1
        if "table" in lbl and "coffee" not in lbl:
            table_cnt += 1
        matched = False
        for key, weights in _OBJECT_ROOM_PRIORS.items():
            if key == lbl or key in lbl:
                matched = True
                for rtype, w in weights.items():
                    scores[rtype] = scores.get(rtype, 0.0) + float(w)
        if not matched:
            for tok in lbl.split():
                if tok in _OBJECT_ROOM_PRIORS:
                    for rtype, w in _OBJECT_ROOM_PRIORS[tok].items():
                        scores[rtype] = scores.get(rtype, 0.0) + 0.6 * float(w)
    # Multi-chair + table co-occurrence strongly signals dining_room
    if table_cnt >= 1 and chair_cnt >= 2:
        scores["dining_room"] = scores.get("dining_room", 0.0) + 2.5
    return scores


def _score_embeddings_for_room(
    kf_embeddings: Sequence[tuple[float, ...]],
    prototype_embeddings: Mapping[str, tuple[float, ...]],
) -> dict[str, float]:
    if not kf_embeddings or not prototype_embeddings:
        return {}
    mat = np.asarray(kf_embeddings, dtype=np.float64)
    mean_v = np.mean(mat, axis=0)
    norm = float(np.linalg.norm(mean_v))
    if norm <= 1e-12:
        return {}
    mean_v = mean_v / norm
    raw_cos: dict[str, float] = {}
    for rtype, pvec in prototype_embeddings.items():
        if len(pvec) != mean_v.shape[0]:
            continue
        raw_cos[rtype] = float(np.dot(mean_v, np.asarray(pvec, dtype=np.float64)))
    if not raw_cos:
        return {}
    # Shift by median cosine so only above-average semantic affinity adds positive signal
    med = float(np.median(list(raw_cos.values())))
    return {
        rtype: max(0.0, (val - med) * 18.0)
        for rtype, val in raw_cos.items()
    }


def _classify_region_label(
    *,
    is_stairwell: bool,
    span_x: float,
    span_y: float,
    clearance_m: float,
    object_labels: Sequence[str],
    kf_embeddings: Sequence[tuple[float, ...]],
    prototype_embeddings: Mapping[str, tuple[float, ...]],
) -> tuple[str, float, str]:
    """Fuse geometric cues (stairs, corridor narrowness), 3D objects, and SigLIP prototypes."""
    if is_stairwell:
        return "stairwell", 0.92, "geometry_stair"

    obj_scores = _score_objects_for_room(object_labels)
    emb_scores = _score_embeddings_for_room(kf_embeddings, prototype_embeddings)

    # Narrow elongated geometry with few/no heavy room furniture -> hallway
    short_side = min(span_x, span_y)
    long_side = max(span_x, span_y)
    aspect = (long_side / max(0.3, short_side)) if long_side > 0.1 else 1.0
    geom_hallway = 0.0
    if (clearance_m > 0.0 and clearance_m <= 1.05 and short_side <= 1.9 and long_side >= 2.2) or (
        short_side <= 1.6 and aspect >= 2.1
    ):
        geom_hallway = 2.6

    combined: dict[str, float] = {}
    all_keys = set(obj_scores) | set(emb_scores) | ({"hallway"} if geom_hallway > 0.0 else set())
    for k in all_keys:
        combined[k] = obj_scores.get(k, 0.0) + emb_scores.get(k, 0.0)
    if geom_hallway > 0.0:
        combined["hallway"] = combined.get("hallway", 0.0) + geom_hallway

    if not combined:
        return "open_area", 0.25, "geometry_fallback"

    ranked = sorted(combined.items(), key=lambda kv: -kv[1])
    best_lbl, best_sc = ranked[0]
    second_sc = ranked[1][1] if len(ranked) > 1 else 0.0
    if best_sc < 0.35:
        return "open_area", 0.30, "geometry_fallback"

    conf = min(0.95, 0.45 + 0.12 * best_sc + 0.08 * max(0.0, best_sc - second_sc))
    if geom_hallway > 0.0 and best_lbl == "hallway" and obj_scores.get("hallway", 0.0) == 0.0:
        src = "geometry_corridor"
    elif obj_scores.get(best_lbl, 0.0) > 0.0 and emb_scores.get(best_lbl, 0.0) > 0.0:
        src = "object_siglip_fused"
    elif obj_scores.get(best_lbl, 0.0) > 0.0:
        src = "object_prior"
    else:
        src = "siglip_prototype"
    return best_lbl, round(conf, 3), src


class GlobalRegionMemory:
    """Topology-grounded semantic region partitioner and event-driven summary archive."""

    def __init__(
        self,
        memory: HierarchicalMemoryStore,
        texture_selector: TextureKeyframeSelector,
        *,
        prototype_embeddings: Mapping[str, tuple[float, ...]] | None = None,
        open_plan_split_span_m: float = 5.5,
        open_plan_min_subcluster_sep_m: float = 2.6,
        set_cover_window_kfs: int = 5,
    ) -> None:
        self.memory = memory
        self.texture_selector = texture_selector
        self.prototype_embeddings: dict[str, tuple[float, ...]] = (
            dict(prototype_embeddings) if prototype_embeddings else {}
        )
        self.open_plan_split_span_m = float(open_plan_split_span_m)
        self.open_plan_min_subcluster_sep_m = float(open_plan_min_subcluster_sep_m)
        self.set_cover_window_kfs = max(2, int(set_cover_window_kfs))

        self.regions: dict[str, RegionArchiveEntry] = {}
        self._kf_to_region: dict[str, str] = {}
        self._kf_median_depth: dict[str, float] = {}
        self._preserved_annotations: dict[str, dict[str, Any]] = {}
        self._last_pruned_kf_count: dict[str, int] = {}
        self.current_region_id: str | None = None
        self._revision: int = 0

    def set_prototype_embeddings(self, protos: Mapping[str, tuple[float, ...]]) -> None:
        self.prototype_embeddings = dict(protos)

    def record_keyframe_depth(self, keyframe_id: str, median_depth_m: float) -> None:
        if math.isfinite(float(median_depth_m)):
            self._kf_median_depth[str(keyframe_id)] = float(median_depth_m)

    def annotate_region(
        self,
        region_id: str,
        *,
        primary_label: str = "",
        detailed_description: str = "",
        source: str = "agent_annotated",
        vlm_summarized: bool = False,
    ) -> RegionArchiveEntry | None:
        rid = str(region_id or "").strip()
        reg = self.regions.get(rid)
        if reg is None:
            return None
        clean_lbl = " ".join(str(primary_label or "").strip().lower().split()).replace(" ", "_")
        clean_desc = str(detailed_description or "").strip()[:420]
        if clean_lbl and clean_lbl not in ("unknown", "open_area"):
            reg.primary_label = clean_lbl
            reg.label_confidence = max(reg.label_confidence, 0.90)
            reg.label_source = source
        if clean_desc:
            reg.detailed_description = clean_desc
        if vlm_summarized:
            reg.vlm_summarized = True
            reg.summary_object_count = len(reg.objects)
            reg.summary_voxel_count = reg.texture_voxels_count
        reg.summary_revision += 1
        self._preserved_annotations[rid] = {
            "primary_label": reg.primary_label,
            "label_confidence": reg.label_confidence,
            "label_source": reg.label_source,
            "detailed_description": reg.detailed_description,
            "vlm_summarized": reg.vlm_summarized,
            "summary_object_count": reg.summary_object_count,
            "summary_voxel_count": reg.summary_voxel_count,
            "centroid_xyz": reg.centroid_xyz,
        }
        return reg

    def _find_preserved_annotation(
        self, rid: str, centroid_xyz: tuple[float, float, float]
    ) -> dict[str, Any] | None:
        direct = self._preserved_annotations.get(rid)
        if direct is not None:
            c = direct["centroid_xyz"]
            if abs(c[2] - centroid_xyz[2]) <= 1.2 and math.hypot(c[0] - centroid_xyz[0], c[1] - centroid_xyz[1]) <= 3.0:
                return direct
        best: dict[str, Any] | None = None
        best_d = 2.0
        for ann in self._preserved_annotations.values():
            c = ann["centroid_xyz"]
            if abs(c[2] - centroid_xyz[2]) > 1.2:
                continue
            d = math.hypot(c[0] - centroid_xyz[0], c[1] - centroid_xyz[1])
            if d < best_d:
                best_d = d
                best = ann
        return best

    def _maybe_split_open_plan_room(
        self,
        geom_id: str,
        floor_id: str,
        kfs: list[str],
        objs_by_kf: Mapping[str, list[str]],
    ) -> list[tuple[str, list[str], bool]]:
        """Split an oversized geometric room when two distant sub-clusters have distinct labels."""
        graph = self.memory.graph
        pts = np.asarray([graph.keyframe_poses[k][:3, 3] for k in kfs], dtype=np.float64)
        span_x = float(np.max(pts[:, 0]) - np.min(pts[:, 0]))
        span_y = float(np.max(pts[:, 1]) - np.min(pts[:, 1]))
        if max(span_x, span_y) < self.open_plan_split_span_m or len(kfs) < 4:
            return [(geom_id, kfs, False)]

        # Split along principal XY axis using median projection
        xy = pts[:, :2]
        centered = xy - np.mean(xy, axis=0, keepdims=True)
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        proj = centered @ vt[0]
        med = float(np.median(proj))
        group_a = [kfs[i] for i in range(len(kfs)) if proj[i] <= med]
        group_b = [kfs[i] for i in range(len(kfs)) if proj[i] > med]
        if len(group_a) < 2 or len(group_b) < 2:
            return [(geom_id, kfs, False)]

        ca = np.mean([graph.keyframe_poses[k][:2, 3] for k in group_a], axis=0)
        cb = np.mean([graph.keyframe_poses[k][:2, 3] for k in group_b], axis=0)
        if float(np.linalg.norm(ca - cb)) < self.open_plan_min_subcluster_sep_m:
            return [(geom_id, kfs, False)]

        def _sub_label(sub_kfs: list[str]) -> tuple[str, float]:
            objs = [lbl for k in sub_kfs for lbl in objs_by_kf.get(k, ())]
            embs = [
                self.memory._keyframe_meta[k].embedding
                for k in sub_kfs
                if k in self.memory._keyframe_meta and self.memory._keyframe_meta[k].embedding is not None
            ]
            lbl, conf, _ = _classify_region_label(
                is_stairwell=False,
                span_x=span_x * 0.5,
                span_y=span_y * 0.5,
                clearance_m=1.5,
                object_labels=objs,
                kf_embeddings=embs,  # type: ignore[arg-type]
                prototype_embeddings=self.prototype_embeddings,
            )
            return lbl, conf

        lbl_a, conf_a = _sub_label(group_a)
        lbl_b, conf_b = _sub_label(group_b)
        if (
            lbl_a != lbl_b
            and lbl_a not in ("open_area", "hallway")
            and lbl_b not in ("open_area", "hallway")
            and conf_a >= 0.55
            and conf_b >= 0.55
        ):
            return [(f"{geom_id}a", group_a, True), (f"{geom_id}b", group_b, True)]
        return [(geom_id, kfs, False)]

    def sync_from_scene_graph(
        self,
        robot_xyz: Sequence[float],
        *,
        active_frontiers: Sequence[dict[str, Any]] = (),
        summarizer_fn: RegionSummarizerFn | None = None,
    ) -> dict[str, Any]:
        """Rebuild geometric+semantic regions from SceneGraph, run delayed Set-Cover, and refresh summaries."""
        graph = self.memory.graph
        prev_region = self.current_region_id
        self._revision += 1

        # 1. Trigger SceneGraph geometric floor & room partitioning whenever >= 3 keyframes exist
        if len(graph.keyframe_poses) >= 3:
            graph.segment_floors()
            graph.partition_rooms()

        # Map non-stale, non-refuted objects to their observing keyframes
        objs_by_kf: dict[str, list[str]] = {}
        objs_world: list[tuple[str, np.ndarray, str]] = []
        for obj in graph.objects.values():
            meta = self.memory._object_meta.get(obj.object_id)
            if meta is not None and (meta.stale or meta.epistemic_status == "refuted"):
                continue
            kf_ref = (
                meta.last_observed_keyframe_id
                if (meta is not None and meta.last_observed_keyframe_id)
                else obj.anchor_keyframe_id
            )
            objs_by_kf.setdefault(kf_ref, []).append(obj.label)
            objs_world.append(
                (obj.label, np.asarray(obj.world_center_xyz, dtype=np.float64), kf_ref)
            )

        # Collect stair keyframes directly from SceneGraph StairEdges
        stair_kfs: set[str] = set()
        for st in graph.stairs:
            stair_kfs.update(st.keyframe_ids)

        # 2. Build raw geometric groups from SceneGraph rooms + unassigned keyframes (e.g. stairs/early frames)
        raw_groups: list[tuple[str, str, list[str], bool, float]] = []
        assigned_kfs: set[str] = set()

        for room_id, room in sorted(graph.rooms.items()):
            kfs = [k for k in room.anchor_keyframe_ids if k in graph.keyframe_poses]
            if not kfs:
                continue
            assigned_kfs.update(kfs)
            split_list = self._maybe_split_open_plan_room(room_id, room.floor_id, kfs, objs_by_kf)
            for sub_id, sub_kfs, _was_split in split_list:
                raw_groups.append((sub_id, room.floor_id, sub_kfs, False, float(room.clearance_m)))

        # Group stair keyframes into explicit stairwell regions
        unassigned_stairs = [k for k in graph.keyframe_poses if k in stair_kfs and k not in assigned_kfs]
        if unassigned_stairs:
            assigned_kfs.update(unassigned_stairs)
            raw_groups.append(("R_stair_0", "stair", unassigned_stairs, True, 0.75))

        # Any remaining keyframes (e.g. first 1-2 frames or pure corridor bottlenecks before rooms form)
        leftover = [k for k in graph.keyframe_poses if k not in assigned_kfs]
        if leftover:
            # Attach each leftover keyframe to nearest existing same-floor group if within 2.5m; else form seed region
            still_leftover: list[str] = []
            for k in leftover:
                kp = graph.keyframe_poses[k][:3, 3]
                best_idx = -1
                best_d = 2.5
                for idx, (_gid, _fid, gkfs, _is_st, _clr) in enumerate(raw_groups):
                    gc = np.mean([graph.keyframe_poses[gk][:3, 3] for gk in gkfs], axis=0)
                    if abs(float(gc[2] - kp[2])) <= 1.2:
                        d = float(math.hypot(gc[0] - kp[0], gc[1] - kp[1]))
                        if d < best_d:
                            best_d = d
                            best_idx = idx
                if best_idx >= 0:
                    raw_groups[best_idx][2].append(k)
                else:
                    still_leftover.append(k)
            if still_leftover:
                clrs = [
                    float(graph.keyframe_clearances[k])
                    for k in still_leftover
                    if k in graph.keyframe_clearances
                ]
                med_clr = float(np.median(clrs)) if clrs else 1.2
                raw_groups.append(("R_seed_0", "floor_0", still_leftover, False, med_clr))

        # 3. Assign stable user-facing Region IDs (R1, R2, ...) and build connectivity edges
        new_regions: dict[str, RegionArchiveEntry] = {}
        new_kf_to_reg: dict[str, str] = {}
        geom_to_rids: dict[str, list[str]] = {}
        split_pairs: list[tuple[str, str, list[float]]] = []

        # Track open-plan split siblings so we link them via "open_passage"
        base_split_map: dict[str, list[tuple[str, np.ndarray]]] = {}

        for seq_idx, (geom_id, floor_id, kfs, is_stair, clr_m) in enumerate(raw_groups, start=1):
            rid = f"R{seq_idx}"
            base_geom = geom_id[:-1] if (len(geom_id) > 1 and geom_id[-1] in ("a", "b") and geom_id[:-1] in graph.rooms) else geom_id
            geom_to_rids.setdefault(base_geom, []).append(rid)

            pts = np.asarray([graph.keyframe_poses[k][:3, 3] for k in kfs], dtype=np.float64)
            cx, cy, cz = float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1])), float(np.mean(pts[:, 2]))
            min_x, max_x = float(np.min(pts[:, 0])), float(np.max(pts[:, 0]))
            min_y, max_y = float(np.min(pts[:, 1])), float(np.max(pts[:, 1]))
            span_x = max(0.5, max_x - min_x)
            span_y = max(0.5, max_y - min_y)

            if base_geom != geom_id:
                base_split_map.setdefault(base_geom, []).append((rid, np.array([cx, cy, cz])))

            kf_set = set(kfs)
            for k in kfs:
                new_kf_to_reg[k] = rid

            # Collect objects anchored to this region's keyframes OR geometrically inside its bounding box
            obj_labels_raw: list[str] = []
            for olbl, oxyz, okf in objs_world:
                if okf in kf_set:
                    obj_labels_raw.append(olbl)
                elif (
                    abs(float(oxyz[2]) - cz) <= 1.2
                    and (min_x - 1.5) <= float(oxyz[0]) <= (max_x + 1.5)
                    and (min_y - 1.5) <= float(oxyz[1]) <= (max_y + 1.5)
                ):
                    obj_labels_raw.append(olbl)

            # Summarize object counts concisely (e.g. "chair x3", "dining table")
            counts: dict[str, int] = {}
            for ol in obj_labels_raw:
                counts[ol] = counts.get(ol, 0) + 1
            obj_summary = tuple(
                f"{k} x{v}" if v > 1 else k
                for k, v in sorted(counts.items(), key=lambda kv: (-kv[1], kv[0]))[:10]
            )

            # Gather keyframe embeddings
            embs = [
                self.memory._keyframe_meta[k].embedding
                for k in kfs
                if k in self.memory._keyframe_meta and self.memory._keyframe_meta[k].embedding is not None
            ]

            lbl, conf, src = _classify_region_label(
                is_stairwell=is_stair,
                span_x=span_x,
                span_y=span_y,
                clearance_m=clr_m,
                object_labels=obj_labels_raw,
                kf_embeddings=embs,  # type: ignore[arg-type]
                prototype_embeddings=self.prototype_embeddings,
            )

            # Determine texture keyframes for this region via Set-Cover
            tex_cands = [k for k in kfs if k in self.texture_selector.records]
            prev_cnt = self._last_pruned_kf_count.get(rid, 0)
            if len(tex_cands) >= self.set_cover_window_kfs and (len(tex_cands) - prev_cnt) >= 3:
                prune_res = self.texture_selector.prune_region_set_cover(rid, tex_cands)
                self._last_pruned_kf_count[rid] = len(tex_cands)
                tex_kfs = prune_res.selected_keyframe_ids
                tex_vox = prune_res.total_voxels
                tex_cov = prune_res.weighted_coverage_ratio
            else:
                tex_kfs = tuple(
                    k for k in tex_cands if self.texture_selector.records[k].retained
                )[: self.texture_selector.max_keyframes_per_region]
                if not tex_kfs and kfs:
                    tex_kfs = (kfs[0], kfs[-1]) if len(kfs) > 1 else (kfs[0],)
                tex_vox, tex_cov = self.texture_selector.region_surface_stats(tex_cands)

            # Coverage status from close-up depth observations + texture voxels
            close_cnt = sum(1 for k in kfs if self._kf_median_depth.get(k, 3.0) <= 2.2)
            if close_cnt >= 2 or (len(tex_kfs) >= 2 and tex_vox >= 180):
                cov_status = "inspected_closeup"
            elif len(kfs) >= 2 or close_cnt >= 1:
                cov_status = "partial"
            else:
                cov_status = "glimpsed"

            # Count active frontiers near this region's footprint
            unexp_cnt = 0
            for fr in active_frontiers:
                fxy = fr.get("xy")
                fz = fr.get("z", cz)
                if not isinstance(fxy, (list, tuple)) or len(fxy) < 2:
                    continue
                if abs(float(fz) - cz) <= 1.2 and (min_x - 2.0) <= float(fxy[0]) <= (max_x + 2.0) and (min_y - 2.0) <= float(fxy[1]) <= (max_y + 2.0):
                    unexp_cnt += 1

            # Check preserved VLM/agent annotations
            ann = self._find_preserved_annotation(rid, (cx, cy, cz))
            vlm_done = False
            s_obj_cnt = 0
            s_vox_cnt = 0
            if ann is not None:
                if ann.get("label_source") in ("vlm_summarized", "agent_annotated"):
                    lbl = str(ann["primary_label"])
                    conf = float(ann["label_confidence"])
                    src = str(ann["label_source"])
                vlm_done = bool(ann.get("vlm_summarized", False))
                s_obj_cnt = int(ann.get("summary_object_count", 0))
                s_vox_cnt = int(ann.get("summary_voxel_count", 0))

            reg = RegionArchiveEntry(
                region_id=rid,
                floor_id=str(floor_id),
                primary_label=lbl,
                label_confidence=conf,
                label_source=src,
                detailed_description=(str(ann["detailed_description"]) if (ann and ann.get("detailed_description")) else ""),
                keyframe_ids=tuple(kfs),
                texture_keyframe_ids=tex_kfs,
                texture_coverage_ratio=tex_cov,
                texture_voxels_count=tex_vox,
                coverage_status=cov_status,
                centroid_xyz=(round(cx, 3), round(cy, 3), round(cz, 3)),
                bounds_xy=(round(min_x, 2), round(max_x, 2), round(min_y, 2), round(max_y, 2)),
                clearance_m=round(clr_m, 2),
                objects=obj_summary,
                unexplored_exits_count=unexp_cnt,
                vlm_summarized=vlm_done,
                summary_object_count=s_obj_cnt,
                summary_voxel_count=s_vox_cnt,
                summary_revision=self._revision,
                geom_room_id=geom_id,
            )
            new_regions[rid] = reg

        # 4. Wire connectivity graph: doorways + open-plan passages + stair transitions
        adj_map: dict[str, list[dict[str, Any]]] = {rid: [] for rid in new_regions}

        def _add_edge(ra: str, rb: str, via: str, xyz: Sequence[float]) -> None:
            if ra == rb or ra not in adj_map or rb not in adj_map:
                return
            if any(e["to"] == rb and e["via"] == via for e in adj_map[ra]):
                return
            pos_r = [round(float(xyz[0]), 2), round(float(xyz[1]), 2), round(float(xyz[2]), 2)]
            adj_map[ra].append({"to": rb, "to_label": new_regions[rb].primary_label, "via": via, "xyz": pos_r})
            adj_map[rb].append({"to": ra, "to_label": new_regions[ra].primary_label, "via": via, "xyz": pos_r})

        for dw in graph.doorways:
            rids_a = geom_to_rids.get(dw.room_a, [])
            rids_b = geom_to_rids.get(dw.room_b, [])
            if rids_a and rids_b:
                # Connect closest sub-regions if either side was open-plan split
                gxy = np.asarray(dw.gateway_xyz[:2], dtype=np.float64)
                ra = min(rids_a, key=lambda r: float(np.linalg.norm(np.asarray(new_regions[r].centroid_xyz[:2]) - gxy)))
                rb = min(rids_b, key=lambda r: float(np.linalg.norm(np.asarray(new_regions[r].centroid_xyz[:2]) - gxy)))
                _add_edge(ra, rb, "doorway", dw.gateway_xyz)

        for siblings in base_split_map.values():
            if len(siblings) == 2:
                (ra, ca), (rb, cb) = siblings
                mid = 0.5 * (ca + cb)
                _add_edge(ra, rb, "open_passage", mid)

        # Link sequential trajectory region transitions (catches stair/corridor links)
        ordered_kfs = list(graph.keyframe_poses.keys())
        for ka, kb in zip(ordered_kfs, ordered_kfs[1:]):
            ra, rb = new_kf_to_reg.get(ka), new_kf_to_reg.get(kb)
            if ra and rb and ra != rb:
                pa = graph.keyframe_poses[ka][:3, 3]
                pb = graph.keyframe_poses[kb][:3, 3]
                via = "stair" if (new_regions[ra].primary_label == "stairwell" or new_regions[rb].primary_label == "stairwell") else "passage"
                _add_edge(ra, rb, via, 0.5 * (pa + pb))

        for rid, reg in new_regions.items():
            reg.connected_regions = tuple(adj_map.get(rid, ()))
            if not reg.detailed_description or not reg.vlm_summarized:
                reg.detailed_description = self._build_deterministic_description(reg)

        self.regions = new_regions
        self._kf_to_region = new_kf_to_reg

        # 5. Locate robot's current region
        rpos = np.asarray(robot_xyz, dtype=np.float64)
        cur_rid: str | None = None
        best_d = float("inf")
        for rid, reg in self.regions.items():
            if abs(reg.centroid_xyz[2] - float(rpos[2])) > 1.5:
                continue
            d = math.hypot(reg.centroid_xyz[0] - float(rpos[0]), reg.centroid_xyz[1] - float(rpos[1]))
            if d < best_d:
                best_d = d
                cur_rid = rid
        self.current_region_id = cur_rid
        region_changed = (prev_region is not None and cur_rid is not None and prev_region != cur_rid)

        # 6. Delayed Set-Cover prune + optional event-driven VLM summary when exiting a region or on major growth
        pruned_regions: list[str] = []
        summarized_regions: list[str] = []
        targets_to_check: list[str] = []
        if region_changed and prev_region in self.regions:
            targets_to_check.append(prev_region)
        if cur_rid in self.regions:
            targets_to_check.append(cur_rid)

        for rid in dict.fromkeys(targets_to_check):
            reg = self.regions[rid]
            tex_cands = [k for k in reg.keyframe_ids if k in self.texture_selector.records]
            # Always run Set-Cover when exiting a region with >= 3 texture candidates
            if (rid == prev_region and len(tex_cands) >= 3) or (
                len(tex_cands) >= self.set_cover_window_kfs
                and len(tex_cands) > self._last_pruned_kf_count.get(rid, 0)
            ):
                p_res = self.texture_selector.prune_region_set_cover(rid, tex_cands)
                self._last_pruned_kf_count[rid] = len(tex_cands)
                reg.texture_keyframe_ids = p_res.selected_keyframe_ids
                reg.texture_coverage_ratio = p_res.weighted_coverage_ratio
                reg.texture_voxels_count = p_res.total_voxels
                pruned_regions.append(rid)
                if not reg.vlm_summarized:
                    reg.detailed_description = self._build_deterministic_description(reg)

            if summarizer_fn is not None and len(reg.texture_keyframe_ids) >= 1:
                needs_vlm = (
                    (rid == prev_region and not reg.vlm_summarized and len(reg.texture_keyframe_ids) >= 1)
                    or (
                        reg.vlm_summarized
                        and (
                            len(reg.objects) >= reg.summary_object_count + 3
                            or reg.texture_voxels_count >= int(max(1, reg.summary_voxel_count) * 1.35)
                        )
                    )
                )
                if needs_vlm:
                    rgb_paths = [
                        graph.keyframe_rgb_paths[k]
                        for k in reg.texture_keyframe_ids[:3]
                        if k in graph.keyframe_rgb_paths
                    ]
                    if rgb_paths:
                        out = summarizer_fn(reg, rgb_paths)
                        if out is not None:
                            new_lbl, new_desc = out
                            self.annotate_region(
                                rid,
                                primary_label=(new_lbl or ""),
                                detailed_description=(new_desc or ""),
                                source="vlm_summarized",
                                vlm_summarized=True,
                            )
                            summarized_regions.append(rid)

        return {
            "current_region_id": self.current_region_id,
            "region_changed": region_changed,
            "regions_count": len(self.regions),
            "pruned_regions": pruned_regions,
            "summarized_regions": summarized_regions,
        }

    @staticmethod
    def _build_deterministic_description(reg: RegionArchiveEntry) -> str:
        sx = max(0.5, reg.bounds_xy[1] - reg.bounds_xy[0])
        sy = max(0.5, reg.bounds_xy[3] - reg.bounds_xy[2])
        obj_txt = ", ".join(reg.objects[:6]) if reg.objects else "no confirmed furniture yet"
        conn_txt = (
            ", ".join(f"{c['via']}->{c['to']}({c['to_label']})" for c in reg.connected_regions[:4])
            if reg.connected_regions
            else "none mapped"
        )
        exit_txt = (
            f"; {reg.unexplored_exits_count} unexplored frontier(s)"
            if reg.unexplored_exits_count > 0
            else ""
        )
        kfs_txt = ",".join(reg.texture_keyframe_ids[:4])
        cov_pct = int(round(reg.texture_coverage_ratio * 100.0))
        return (
            f"{reg.primary_label} (~{sx:.1f}x{sy:.1f}m, clr={reg.clearance_m:.1f}m, {reg.coverage_status}): "
            f"objects=[{obj_txt}]; links=[{conn_txt}]{exit_txt}; "
            f"texture_kfs=[{kfs_txt}] ({cov_pct}% surface cov)."
        )

    def region_summaries(self, robot_xyz: Sequence[float] | None = None) -> list[dict[str, Any]]:
        ref = None if robot_xyz is None else np.asarray(robot_xyz, dtype=np.float64)
        rows: list[dict[str, Any]] = []
        for reg in self.regions.values():
            d = reg.to_dict()
            if ref is not None:
                d["distance_m"] = round(
                    float(np.linalg.norm(np.asarray(reg.centroid_xyz) - ref)), 2
                )
            rows.append(d)
        if ref is not None:
            rows.sort(key=lambda r: float(r.get("distance_m", 0.0)))
        return rows

    def search_regions(self, query: str, *, top_k: int = 4) -> list[dict[str, Any]]:
        """Retrieve relevant regions by matching big label, detailed summary, and objects."""
        q = " ".join(str(query or "").strip().lower().split())
        q_toks = set(q.replace("_", " ").split())
        scored: list[tuple[float, dict[str, Any]]] = []
        for reg in self.regions.values():
            hay = " ".join(
                [
                    reg.primary_label.replace("_", " "),
                    reg.detailed_description.lower(),
                    " ".join(o.lower() for o in reg.objects),
                ]
            )
            score = 0.0
            if q and q in hay:
                score += 10.0
            for tok in q_toks:
                if tok in reg.primary_label.replace("_", " "):
                    score += 5.0
                if tok in hay:
                    score += 2.0
            if score > 0.0 or not q:
                scored.append((score, reg.to_dict()))
        scored.sort(key=lambda kv: -kv[0])
        return [item[1] for item in scored[: max(1, int(top_k))]]

    def format_global_digest(self, *, max_regions: int = 10) -> str:
        """Compact global semantic region index for persistent LLM orientation & compaction."""
        if not self.regions:
            return "No regions mapped yet."
        lines: list[str] = []
        for reg in list(self.regions.values())[:max_regions]:
            mark = "*" if reg.region_id == self.current_region_id else " "
            lines.append(
                f"{mark}{reg.region_id}[{reg.primary_label}|{reg.coverage_status}|"
                f"kfs={list(reg.texture_keyframe_ids[:4])}]: {reg.detailed_description}"
            )
        return "\n".join(lines)
