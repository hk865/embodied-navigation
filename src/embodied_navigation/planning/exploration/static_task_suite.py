"""30-task multi-tier static embodied navigation benchmark & 8-metric evaluator.

Evaluates L0-L3 hierarchical spatial memory, 6-zone structured context,
global M* memory candidates, Attempt & Failure Ledger blacklisting, and
pre-STOP geometric verification across three difficulty tiers grounded in
a real multi-floor :class:`KeyframeSceneGraph`:

- Tier 1 (10 tasks): Single-floor landmark search & active STOP verification.
- Tier 2 (10 tasks): Unreset multi-target sequential chains (GOAT-Bench style).
- Tier 3 (10 tasks): Cross-floor tasks + stale-belief / wrong-floor / blacklist traps.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import time
from typing import Any, Callable, Sequence

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    HierarchicalMemoryStore,
    KeyframeSceneGraph,
)
from embodied_navigation.planning.exploration.streaming_task_agent import (
    OpenCodeTaskPlanner,
    TaskDecision,
)


@dataclass(frozen=True)
class SeedObject:
    object_id: str
    label: str
    keyframe_id: str
    local_xyz: tuple[float, float, float] = (0.5, 0.0, 0.0)
    confidence: float = 0.9
    stale: bool = False
    stale_reason: str = ""
    is_ground_truth: bool = True


@dataclass(frozen=True)
class StaticTaskSpec:
    """One reproducible static navigation/memory evaluation task."""

    task_id: str
    tier: str  # tier1_single_floor | tier2_multi_target | tier3_cross_floor_adv
    subtype: str
    instruction: str
    start_keyframe_id: str
    target_labels: tuple[str, ...]
    target_object_ids: tuple[str, ...]
    seed_objects: tuple[SeedObject, ...]
    blacklist_traps: tuple[tuple[float, float, float, str], ...] = ()
    stop_radius_m: float = 1.8
    max_steps: int = 8
    preload_memory: bool = True


def _pick_kf(kfs: Sequence[str], frac: float) -> str:
    if not kfs:
        raise ValueError("empty keyframe list")
    idx = min(len(kfs) - 1, max(0, int(round(frac * (len(kfs) - 1)))))
    return kfs[idx]


def _room_kfs(graph: KeyframeSceneGraph, floor_id: str) -> list[tuple[str, list[str]]]:
    rooms = [
        (r.room_id, list(r.anchor_keyframe_ids))
        for r in graph.rooms.values()
        if r.floor_id == floor_id and r.anchor_keyframe_ids
    ]
    rooms.sort(key=lambda item: (-len(item[1]), item[0]))
    return rooms


def build_default_30_task_suite(graph: KeyframeSceneGraph) -> list[StaticTaskSpec]:
    """Construct 30 grounded static tasks (10 Tier-1 + 10 Tier-2 + 10 Tier-3)."""
    f0_rooms = _room_kfs(graph, "f0")
    f1_rooms = _room_kfs(graph, "f1")
    if not f0_rooms:
        raise ValueError("graph must have at least one floor 'f0' with rooms")
    if not f1_rooms:
        f1_rooms = f0_rooms

    def _kf_at(room_list: list[tuple[str, list[str]]], r_idx: int, frac: float = 0.5) -> str:
        _, kfs = room_list[r_idx % len(room_list)]
        return _pick_kf(kfs, frac)

    # Canonical landmark bank grounded in real room anchor keyframes across both floors
    landmarks = [
        SeedObject("lm_f0_door", "red double door", _kf_at(f0_rooms, 0, 0.15), (0.8, 0.0, 0.0)),
        SeedObject("lm_f0_vend", "vending machine", _kf_at(f0_rooms, 0, 0.75), (0.6, 0.2, 0.0)),
        SeedObject("lm_f0_hose", "red fire hose cabinet", _kf_at(f0_rooms, 1, 0.45), (0.5, -0.3, 0.0)),
        SeedObject("lm_f0_desk", "basement office desk", _kf_at(f0_rooms, 2, 0.50), (0.7, 0.1, 0.0)),
        SeedObject("lm_f0_shelf", "dark storage shelving", _kf_at(f0_rooms, 3, 0.50), (0.6, 0.0, 0.0)),
        SeedObject("lm_f0_stair", "stairwell entrance", _kf_at(f0_rooms, 0, 0.90), (0.5, 0.0, 0.0)),
        SeedObject("lm_f1_conf", "conference desk", _kf_at(f1_rooms, 0, 0.30), (0.8, 0.0, 0.0)),
        SeedObject("lm_f1_print", "east hall printer", _kf_at(f1_rooms, 1, 0.50), (0.6, 0.0, 0.0)),
        SeedObject("lm_f1_board", "north whiteboard", _kf_at(f1_rooms, 2, 0.50), (0.6, 0.2, 0.0)),
        SeedObject("lm_f1_chair", "ergonomic chair", _kf_at(f1_rooms, 3, 0.50), (0.5, 0.0, 0.0)),
        SeedObject("lm_f1_sofa", "lounge sofa", _kf_at(f1_rooms, 4, 0.50), (0.7, 0.0, 0.0)),
        SeedObject("lm_f1_monitor", "workstation monitor", _kf_at(f1_rooms, 5, 0.50), (0.6, 0.0, 0.0)),
    ]
    lm_by_id = {lm.object_id: lm for lm in landmarks}
    base_bank = tuple(landmarks)

    tasks: list[StaticTaskSpec] = []

    # ------------------------------------------------------------------
    # Tier 1: 10 Single-Floor Landmark Search & Active STOP Tasks
    # ------------------------------------------------------------------
    tier1_specs = [
        ("T1_01", "f0_start_door", _kf_at(f0_rooms, 0, 0.0), "lm_f0_door"),
        ("T1_02", "f0_start_vend", _kf_at(f0_rooms, 0, 0.05), "lm_f0_vend"),
        ("T1_03", "f0_hall_hose", _kf_at(f0_rooms, 0, 0.35), "lm_f0_hose"),
        ("T1_04", "f0_hall_desk", _kf_at(f0_rooms, 0, 0.20), "lm_f0_desk"),
        ("T1_05", "f0_office_shelf", _kf_at(f0_rooms, 1, 0.20), "lm_f0_shelf"),
        ("T1_06", "f0_far_stair", _kf_at(f0_rooms, 2, 0.70), "lm_f0_stair"),
        ("T1_07", "f1_landing_conf", _kf_at(f1_rooms, 0, 0.05), "lm_f1_conf"),
        ("T1_08", "f1_hall_printer", _kf_at(f1_rooms, 0, 0.60), "lm_f1_print"),
        ("T1_09", "f1_corridor_board", _kf_at(f1_rooms, 1, 0.20), "lm_f1_board"),
        ("T1_10", "f1_north_sofa", _kf_at(f1_rooms, 2, 0.20), "lm_f1_sofa"),
    ]
    for tid, sub, start_kf, lm_id in tier1_specs:
        lm = lm_by_id[lm_id]
        tasks.append(
            StaticTaskSpec(
                task_id=tid,
                tier="tier1_single_floor",
                subtype=sub,
                instruction=(
                    f"Navigate on the current floor to the '{lm.label}', confirm proximity "
                    f"within 1.8 m, and issue stop_claim with target_label='{lm.label}'."
                ),
                start_keyframe_id=start_kf,
                target_labels=(lm.label,),
                target_object_ids=(lm.object_id,),
                seed_objects=base_bank,
                max_steps=6,
            )
        )

    # ------------------------------------------------------------------
    # Tier 2: 10 Unreset Multi-Target Sequential Memory Chains (2-3 goals)
    # ------------------------------------------------------------------
    tier2_chains = [
        ("T2_01", "chain2_f0_door_vend", _kf_at(f0_rooms, 0, 0.0), ("lm_f0_door", "lm_f0_vend")),
        ("T2_02", "chain2_f0_vend_hose", _kf_at(f0_rooms, 0, 0.1), ("lm_f0_vend", "lm_f0_hose")),
        ("T2_03", "chain2_f0_hose_desk", _kf_at(f0_rooms, 0, 0.3), ("lm_f0_hose", "lm_f0_desk")),
        ("T2_04", "chain2_f0_desk_stair", _kf_at(f0_rooms, 1, 0.5), ("lm_f0_desk", "lm_f0_stair")),
        ("T2_05", "chain3_f0_tour", _kf_at(f0_rooms, 0, 0.0), ("lm_f0_door", "lm_f0_hose", "lm_f0_desk")),
        ("T2_06", "chain2_f1_conf_print", _kf_at(f1_rooms, 0, 0.0), ("lm_f1_conf", "lm_f1_print")),
        ("T2_07", "chain2_f1_print_board", _kf_at(f1_rooms, 0, 0.4), ("lm_f1_print", "lm_f1_board")),
        ("T2_08", "chain2_f1_board_chair", _kf_at(f1_rooms, 1, 0.2), ("lm_f1_board", "lm_f1_chair")),
        ("T2_09", "chain2_f1_sofa_mon", _kf_at(f1_rooms, 3, 0.3), ("lm_f1_sofa", "lm_f1_monitor")),
        ("T2_10", "chain3_f1_tour", _kf_at(f1_rooms, 0, 0.0), ("lm_f1_conf", "lm_f1_print", "lm_f1_board")),
    ]
    for tid, sub, start_kf, chain_ids in tier2_chains:
        labels = tuple(lm_by_id[i].label for i in chain_ids)
        tasks.append(
            StaticTaskSpec(
                task_id=tid,
                tier="tier2_multi_target",
                subtype=sub,
                instruction=(
                    "Sequential unreset memory task. Visit and stop_claim each target in order: "
                    + " -> ".join(labels)
                ),
                start_keyframe_id=start_kf,
                target_labels=labels,
                target_object_ids=chain_ids,
                seed_objects=base_bank,
                max_steps=4 * len(chain_ids) + 2,
            )
        )

    # ------------------------------------------------------------------
    # Tier 3: 10 Cross-Floor + Stale-Memory + Wrong-Floor & Blacklist Traps
    # ------------------------------------------------------------------
    # 3A: 4 Cross-Floor tasks (0->1 and 1->0)
    cross_specs = [
        ("T3_01", "cross_f0_to_f1_conf", _kf_at(f0_rooms, 0, 0.1), "lm_f1_conf"),
        ("T3_02", "cross_f0_to_f1_print", _kf_at(f0_rooms, 1, 0.3), "lm_f1_print"),
        ("T3_03", "cross_f1_to_f0_door", _kf_at(f1_rooms, 0, 0.2), "lm_f0_door"),
        ("T3_04", "cross_f1_to_f0_desk", _kf_at(f1_rooms, 2, 0.5), "lm_f0_desk"),
    ]
    for tid, sub, start_kf, lm_id in cross_specs:
        lm = lm_by_id[lm_id]
        tasks.append(
            StaticTaskSpec(
                task_id=tid,
                tier="tier3_cross_floor_adv",
                subtype=sub,
                instruction=(
                    f"Cross-floor task: travel via stairs to reach '{lm.label}' on the target "
                    f"floor within 1.8 m, then call stop_claim with target_label='{lm.label}'."
                ),
                start_keyframe_id=start_kf,
                target_labels=(lm.label,),
                target_object_ids=(lm.object_id,),
                seed_objects=base_bank,
                max_steps=8,
            )
        )

    # 3B: 3 Stale-Memory / Moved-Object Adversarial Tasks (Phase 5 Memory Probe 1!)
    stale_specs = [
        ("T3_05", "stale_moved_box_f0", _kf_at(f0_rooms, 0, 0.1), _kf_at(f0_rooms, 0, 0.2), _kf_at(f0_rooms, 2, 0.4), "delivery box"),
        ("T3_06", "stale_moved_cart_f1", _kf_at(f1_rooms, 0, 0.1), _kf_at(f1_rooms, 0, 0.2), _kf_at(f1_rooms, 2, 0.5), "service cart"),
        ("T3_07", "stale_moved_laptop_cross", _kf_at(f0_rooms, 0, 0.2), _kf_at(f0_rooms, 1, 0.3), _kf_at(f1_rooms, 1, 0.4), "silver laptop"),
    ]
    for tid, sub, start_kf, old_kf, new_kf, label in stale_specs:
        stale_obj = SeedObject(
            f"{tid}_old", label, old_kf, (0.4, 0.0, 0.0), confidence=0.85,
            stale=True, stale_reason="reobserved_absent_moved", is_ground_truth=False,
        )
        fresh_obj = SeedObject(
            f"{tid}_new", label, new_kf, (0.5, 0.0, 0.0), confidence=0.95,
            stale=False, is_ground_truth=True,
        )
        tasks.append(
            StaticTaskSpec(
                task_id=tid,
                tier="tier3_cross_floor_adv",
                subtype=sub,
                instruction=(
                    f"Find the CURRENT location of '{label}' (ignore stale/moved records), "
                    f"approach within 1.8 m, and call stop_claim with target_label='{label}'."
                ),
                start_keyframe_id=start_kf,
                target_labels=(label,),
                target_object_ids=(fresh_obj.object_id,),
                seed_objects=(*base_bank, stale_obj, fresh_obj),
                max_steps=8,
            )
        )

    # 3C: 3 Wrong-Floor Decoy & Failure-Blacklist Trap Tasks
    # T3_08: Robot spawns directly below Floor-1 target (same XY, different Z -> must NOT stop on Floor 0!)
    f1_target_kf = _kf_at(f1_rooms, 0, 0.4)
    f1_xyz = graph.keyframe_poses[f1_target_kf][:3, 3]
    nearest_f0_under = min(
        graph.floors["f0"].keyframe_ids,
        key=lambda kid: math.hypot(
            float(graph.keyframe_poses[kid][0, 3] - f1_xyz[0]),
            float(graph.keyframe_poses[kid][1, 3] - f1_xyz[1]),
        ),
    )
    decoy_obj = SeedObject("T3_08_target", "projector screen", f1_target_kf, (0.3, 0.0, 0.0))
    tasks.append(
        StaticTaskSpec(
            task_id="T3_08",
            tier="tier3_cross_floor_adv",
            subtype="wrong_floor_same_xy_trap",
            instruction=(
                "Reach 'projector screen' on the SAME FLOOR within 1.8 m (do NOT stop directly "
                "underneath it on Floor 0!), then issue stop_claim."
            ),
            start_keyframe_id=nearest_f0_under,
            target_labels=("projector screen",),
            target_object_ids=("T3_08_target",),
            seed_objects=(*base_bank, decoy_obj),
            max_steps=8,
        )
    )

    # T3_09 & T3_10: Blacklist trap near an intermediate corner (must route around without repeat failures)
    for tid, sub, start_kf, trap_kf, lm_id in [
        ("T3_09", "blacklist_trap_f0_door", _kf_at(f0_rooms, 2, 0.5), _kf_at(f0_rooms, 1, 0.5), "lm_f0_door"),
        ("T3_10", "blacklist_trap_f1_board", _kf_at(f1_rooms, 0, 0.2), _kf_at(f1_rooms, 1, 0.5), "lm_f1_board"),
    ]:
        lm = lm_by_id[lm_id]
        trap_xyz = graph.keyframe_poses[trap_kf][:3, 3]
        tasks.append(
            StaticTaskSpec(
                task_id=tid,
                tier="tier3_cross_floor_adv",
                subtype=sub,
                instruction=(
                    f"Navigate to '{lm.label}' while avoiding blacklisted stuck corners, "
                    f"approach within 1.8 m, and issue stop_claim."
                ),
                start_keyframe_id=start_kf,
                target_labels=(lm.label,),
                target_object_ids=(lm.object_id,),
                seed_objects=base_bank,
                blacklist_traps=((float(trap_xyz[0]), float(trap_xyz[1]), float(trap_xyz[2]), "narrow_door_jam"),),
                max_steps=7,
            )
        )

    assert len(tasks) == 30, f"expected 30 tasks, got {len(tasks)}"
    return tasks


def _build_room_adjacency(graph: KeyframeSceneGraph) -> dict[str, set[str]]:
    adj: dict[str, set[str]] = {rid: set() for rid in graph.rooms}
    for d in graph.doorways:
        if d.room_a in adj and d.room_b in adj:
            adj[d.room_a].add(d.room_b)
            adj[d.room_b].add(d.room_a)
    for st in graph.stairs:
        fa_rooms = [r for r in graph.rooms.values() if r.floor_id == st.from_floor]
        fb_rooms = [r for r in graph.rooms.values() if r.floor_id == st.to_floor]
        if fa_rooms and fb_rooms:
            ra = min(fa_rooms, key=lambda r: float(np.linalg.norm(np.asarray(r.centroid_xyz) - np.asarray(st.start_xyz))))
            rb = min(fb_rooms, key=lambda r: float(np.linalg.norm(np.asarray(r.centroid_xyz) - np.asarray(st.end_xyz))))
            adj[ra.room_id].add(rb.room_id)
            adj[rb.room_id].add(ra.room_id)
    return adj


def _room_geodesic_distance(
    graph: KeyframeSceneGraph,
    adj: dict[str, set[str]],
    start_xyz: Sequence[float],
    goal_xyz: Sequence[float],
) -> float:
    """Estimate shortest traversable distance over the room/doorway/stair graph."""
    if not graph.rooms:
        return float(np.linalg.norm(np.asarray(goal_xyz) - np.asarray(start_xyz)))
    s_pos = np.asarray(start_xyz, dtype=np.float64)
    g_pos = np.asarray(goal_xyz, dtype=np.float64)
    s_room = min(graph.rooms.values(), key=lambda r: float(np.linalg.norm(np.asarray(r.centroid_xyz) - s_pos))).room_id
    g_room = min(graph.rooms.values(), key=lambda r: float(np.linalg.norm(np.asarray(r.centroid_xyz) - g_pos))).room_id
    if s_room == g_room:
        return float(np.linalg.norm(g_pos - s_pos))

    # Dijkstra on room centroids
    dist: dict[str, float] = {s_room: 0.0}
    queue: list[tuple[float, str]] = [(0.0, s_room)]
    while queue:
        queue.sort(key=lambda x: x[0])
        d_cur, cur = queue.pop(0)
        if cur == g_room:
            return d_cur + float(np.linalg.norm(np.asarray(graph.rooms[s_room].centroid_xyz) - s_pos)) * 0.5 + float(np.linalg.norm(np.asarray(graph.rooms[g_room].centroid_xyz) - g_pos)) * 0.5
        if d_cur > dist.get(cur, float("inf")):
            continue
        c_cur = np.asarray(graph.rooms[cur].centroid_xyz)
        for nxt in adj.get(cur, ()):
            step_d = float(np.linalg.norm(np.asarray(graph.rooms[nxt].centroid_xyz) - c_cur))
            cand = d_cur + step_d
            if cand < dist.get(nxt, float("inf")):
                dist[nxt] = cand
                queue.append((cand, nxt))
    return float(np.linalg.norm(g_pos - s_pos))


def evaluate_single_task_offline(
    base_graph: KeyframeSceneGraph,
    task: StaticTaskSpec,
    planner: OpenCodeTaskPlanner,
    *,
    memory_mode: str = "hierarchical_6zone",
    rgb_lookup: Callable[[Sequence[float]], str | None] | None = None,
) -> dict[str, Any]:
    """Run one grounded closed-loop task over the real scene graph topology.

    Supported ``memory_mode`` values for clean ablations:
    - ``"hierarchical_6zone"``: Full 1-paragraph summary + global M* targets + staleness/blacklist gate.
    - ``"stale_unfiltered"``: Ablation exposing stale objects as valid M* targets and disabling stale rejection.
    - ``"no_memory_targets"``: Ablation omitting M* memory candidates (frontier/room exploration only).
    """
    graph = KeyframeSceneGraph.from_dict(base_graph.to_dict())
    store = HierarchicalMemoryStore(graph, default_stop_radius_m=task.stop_radius_m)

    gt_positions: dict[str, np.ndarray] = {}
    for seed in task.seed_objects:
        node = store.add_object(
            seed.object_id,
            seed.label,
            seed.keyframe_id,
            seed.local_xyz,
            confidence=seed.confidence,
            producer="seed_memory",
        )
        if seed.stale and memory_mode != "stale_unfiltered":
            store.mark_object_stale(seed.object_id, reason=seed.stale_reason or "moved")
        if seed.is_ground_truth:
            gt_positions[seed.object_id] = np.asarray(node.world_center_xyz, dtype=np.float64)

    for tx, ty, tz, reason in task.blacklist_traps:
        store.record_attempt("goto", [tx, ty, tz], "stuck", reason=reason, radius_m=1.2)

    adj = _build_room_adjacency(graph)
    cur_pose = graph.keyframe_poses[task.start_keyframe_id]
    robot_xyz = [float(v) for v in cur_pose[:3, 3]]
    yaw_rad = math.atan2(float(cur_pose[1, 0]), float(cur_pose[0, 0]))

    # Compute honest geodesic optimal length L* over the target chain
    optimal_len = 0.0
    prev_ref = list(robot_xyz)
    for oid in task.target_object_ids:
        tgt_xyz = gt_positions[oid]
        optimal_len += max(0.0, _room_geodesic_distance(graph, adj, prev_ref, tgt_xyz) - task.stop_radius_m * 0.5)
        prev_ref = [float(v) for v in tgt_xyz]

    goal_idx = 0
    traveled_m = 0.0
    oracle_reached_flags = [False] * len(task.target_object_ids)
    verified_flags = [False] * len(task.target_object_ids)
    stale_flip_errors = 0
    memory_recall_calls = 0
    steps_used = 0
    last_feedback = "Episode started."
    pending_recall: dict[str, Any] | None = None
    step_latencies: list[float] = []
    fallback_count = 0

    for step in range(1, task.max_steps + 1):
        steps_used = step
        cur_label = task.target_labels[goal_idx]
        cur_gt_oid = task.target_object_ids[goal_idx]
        cur_gt_xyz = gt_positions[cur_gt_oid]

        # Track Oracle Stop (OSR) whenever robot physically passes within stop_radius_m on same floor
        if (
            abs(robot_xyz[2] - float(cur_gt_xyz[2])) <= 1.0
            and float(np.linalg.norm(np.asarray(robot_xyz) - cur_gt_xyz)) <= task.stop_radius_m
        ):
            oracle_reached_flags[goal_idx] = True

        # Build navigable room-center frontier candidates (F1..F6)
        cur_room = min(
            graph.rooms.values(),
            key=lambda r: float(np.linalg.norm(np.asarray(r.centroid_xyz) - np.asarray(robot_xyz))),
        )
        store.record_room_visit(cur_room.room_id)
        neighbor_ids = sorted(adj.get(cur_room.room_id, set()))
        f_cands: list[dict[str, Any]] = []
        for idx_n, rid in enumerate(neighbor_ids[:6], start=1):
            rm = graph.rooms[rid]
            d_m = float(np.linalg.norm(np.asarray(rm.centroid_xyz) - np.asarray(robot_xyz)))
            f_cands.append(
                {
                    "id": f"F{idx_n}",
                    "kind": "frontier",
                    "label": rm.room_id,
                    "xy": [round(float(rm.centroid_xyz[0]), 2), round(float(rm.centroid_xyz[1]), 2)],
                    "z": round(float(rm.centroid_xyz[2]), 3),
                    "dist_m": round(d_m, 2),
                    "clearance_m": round(float(rm.clearance_m), 2),
                    "turn_safe": True,
                }
            )
        # Include any blacklist trap location as an explicit decoy candidate to test filtering!
        for t_idx, (tx, ty, tz, _reason) in enumerate(task.blacklist_traps, start=1):
            f_cands.append(
                {
                    "id": f"W{t_idx}",
                    "kind": "waypoint",
                    "label": "tight_shortcut",
                    "xy": [round(tx, 2), round(ty, 2)],
                    "z": round(tz, 3),
                    "dist_m": round(float(np.linalg.norm(np.asarray([tx, ty, tz]) - np.asarray(robot_xyz))), 2),
                    "clearance_m": 0.45,
                    "turn_safe": False,
                }
            )

        if memory_mode == "no_memory_targets":
            mem_cands: list[dict[str, Any]] = []
            prose = (
                f"当前在 {cur_room.floor_id}/{cur_room.room_id}；本对照基线未启用 M* 全局记忆直达。"
            )
        else:
            mem_cands = store.memory_goal_candidates(
                robot_xyz,
                target_label=cur_label,
                standoff_m=min(1.0, task.stop_radius_m * 0.55),
            )
            prose = store.narrative_summary(robot_xyz=robot_xyz, target_label=cur_label)

        allowed, blocked = store.filter_candidates([*mem_cands, *f_cands], default_z=robot_xyz[2])
        cands = allowed if allowed else blocked
        rgb_path = rgb_lookup(robot_xyz) if rgb_lookup is not None else None

        sub_instruction = (
            f"{task.instruction} (Current active sub-goal {goal_idx + 1}/{len(task.target_labels)}: "
            f"reach '{cur_label}' within {task.stop_radius_m:.1f} m on the same floor and call stop_claim.)"
        )
        decision: TaskDecision = planner.decide(
            task_instruction=sub_instruction,
            step_index=step,
            robot_xyz=robot_xyz,
            yaw_rad=yaw_rad,
            candidates=cands,
            memory_summary=prose,
            memory_l0=store.l0_global_topology(),
            target_label=cur_label,
            stop_radius_m=task.stop_radius_m,
            estimated_travel_m=traveled_m,
            last_feedback=last_feedback,
            tool_recall_result=pending_recall,
            rgb_path=rgb_path,
        )
        pending_recall = None
        step_latencies.append(decision.latency_ms)
        if decision.fallback:
            fallback_count += 1

        if decision.action == "query_memory":
            memory_recall_calls += 1
            q = decision.query or cur_label
            q_l1 = store.l1_submap_summaries(query=q, near_xyz=robot_xyz, top_k=3)
            q_l2 = store.l2_submap_entities(label=q, include_stale=(memory_mode == "stale_unfiltered"))
            pending_recall = {"query": q, "l1_rooms": q_l1, "l2_objects": q_l2["objects"][:5]}
            last_feedback = f"query_memory('{q}') -> {len(q_l1)} rooms, {len(q_l2['objects'])} objects."

        elif decision.action == "stop_claim":
            ver = store.verify_stop_claim(
                robot_xyz,
                target_label=decision.target_label or cur_label,
                stop_radius_m=task.stop_radius_m,
                require_fresh=(memory_mode != "stale_unfiltered"),
            )
            # Check true physical ground-truth proximity (prevents stale_unfiltered from cheating!)
            gt_same_floor = abs(robot_xyz[2] - float(cur_gt_xyz[2])) <= 1.0
            gt_dist = float(np.linalg.norm(np.asarray(robot_xyz) - cur_gt_xyz))
            truly_at_target = gt_same_floor and gt_dist <= task.stop_radius_m

            if ver.allowed and truly_at_target:
                verified_flags[goal_idx] = True
                oracle_reached_flags[goal_idx] = True
                goal_idx += 1
                if goal_idx >= len(task.target_object_ids):
                    break
                last_feedback = (
                    f"Sub-goal '{cur_label}' VERIFIED! Now proceed to next target: "
                    f"'{task.target_labels[goal_idx]}'."
                )
            elif ver.allowed and not truly_at_target:
                # Stale unfiltered memory tricked the agent into stopping at a moved object!
                stale_flip_errors += 1
                last_feedback = (
                    f"FALSE STOP at stale/moved location (true '{cur_label}' is {gt_dist:.1f} m away)."
                )
                break
            else:
                store.record_attempt("stop_claim", robot_xyz, "rejected", reason=ver.reason)
                last_feedback = f"STOP REJECTED ({ver.reason}, dist={ver.distance_m}). Move closer on the same floor."

        elif decision.action in ("goto_candidate", "goto_xyz"):
            dest: list[float] | None = None
            if decision.action == "goto_candidate":
                for c in cands:
                    if c.get("id") == decision.target_id:
                        xy = c.get("xy")
                        if isinstance(xy, (list, tuple)) and len(xy) >= 2:
                            dest = [float(xy[0]), float(xy[1]), float(c.get("z", robot_xyz[2]))]
                        break
            elif decision.xyz is not None:
                dest = [float(v) for v in decision.xyz]

            if dest is None:
                last_feedback = "Unresolved goto target."
                continue

            # Check if destination hits an active blacklist trap
            trap_hit = any(
                abs(dest[2] - tz) <= 1.0 and math.hypot(dest[0] - tx, dest[1] - ty) <= 1.2
                for tx, ty, tz, _ in task.blacklist_traps
            )
            if trap_hit:
                store.record_attempt("goto", dest, "stuck", reason="blacklist_trap_hit")
                last_feedback = f"goto {dest} STUCK in narrow corner trap; choose another route."
                continue

            leg_dist = _room_geodesic_distance(graph, adj, robot_xyz, dest)
            traveled_m += leg_dist
            robot_xyz = list(dest)
            store.record_attempt("goto", dest, "completed")
            # Re-check oracle arrival after moving
            if (
                abs(robot_xyz[2] - float(cur_gt_xyz[2])) <= 1.0
                and float(np.linalg.norm(np.asarray(robot_xyz) - cur_gt_xyz)) <= task.stop_radius_m
            ):
                oracle_reached_flags[goal_idx] = True
            last_feedback = f"goto {[round(v, 2) for v in dest]} completed (+{leg_dist:.1f} m)."

        else:
            last_feedback = f"action {decision.action} executed in place."

    sr = 1.0 if all(verified_flags) else 0.0
    osr = 1.0 if all(oracle_reached_flags) else 0.0
    spl = (
        sr * (optimal_len / max(optimal_len, traveled_m))
        if optimal_len > 1e-6
        else sr
    )
    diag = store.diagnostic_metrics()
    return {
        "task_id": task.task_id,
        "tier": task.tier,
        "subtype": task.subtype,
        "memory_mode": memory_mode,
        "sr": sr,
        "osr": osr,
        "osr_minus_sr": round(osr - sr, 4),
        "spl": round(spl, 4),
        "optimal_len_m": round(optimal_len, 2),
        "traveled_m": round(traveled_m, 2),
        "steps_used": steps_used,
        "max_steps": task.max_steps,
        "memory_recall_calls": memory_recall_calls,
        "stale_flip_errors": stale_flip_errors,
        "fallback_count": fallback_count,
        "mean_latency_ms": round(float(np.mean(step_latencies)) if step_latencies else 0.0, 1),
        "diagnostics": diag,
    }


def aggregate_suite_results(results: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Aggregate 30-task evaluation metrics overall and per difficulty tier."""
    def _summarize(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
        if not rows:
            return {"count": 0}
        n = len(rows)
        sr = sum(r["sr"] for r in rows) / n
        osr = sum(r["osr"] for r in rows) / n
        spl = sum(r["spl"] for r in rows) / n
        lats = sorted(r["mean_latency_ms"] for r in rows if r["mean_latency_ms"] > 0)
        p50 = lats[len(lats) // 2] if lats else 0.0
        p95 = lats[min(len(lats) - 1, int(0.95 * len(lats)))] if lats else 0.0
        return {
            "count": n,
            "sr": round(sr, 4),
            "osr": round(osr, 4),
            "osr_minus_sr": round(osr - sr, 4),
            "spl": round(spl, 4),
            "avg_steps": round(sum(r["steps_used"] for r in rows) / n, 2),
            "avg_traveled_m": round(sum(r["traveled_m"] for r in rows) / n, 2),
            "total_memory_recalls": sum(r["memory_recall_calls"] for r in rows),
            "stale_flip_errors": sum(r["stale_flip_errors"] for r in rows),
            "premature_stop_rejects": sum(
                r["diagnostics"].get("premature_stop_rejects", 0) for r in rows
            ),
            "repeat_failure_attempts": sum(
                r["diagnostics"].get("repeat_failure_attempts", 0) for r in rows
            ),
            "fallback_steps": sum(r["fallback_count"] for r in rows),
            "latency_p50_ms": round(p50, 1),
            "latency_p95_ms": round(p95, 1),
        }

    tiers = sorted({r["tier"] for r in results})
    return {
        "overall": _summarize(results),
        "by_tier": {t: _summarize([r for r in results if r["tier"] == t]) for t in tiers},
    }
