"""30-task multi-tier static embodied navigation benchmark & evaluator.

Evaluates L0-L3 hierarchical spatial memory, 6-zone structured context,
global M* memory candidates, Attempt & Failure Ledger blacklisting, and
pre-STOP geometric verification across three difficulty tiers grounded in
a real multi-floor :class:`KeyframeSceneGraph`:

- Tier 1 (10 tasks): Single-floor known-prior landmark localization & STOP verification.
- Tier 2 (10 tasks): Memory governance (relocated unobserved, undiscovered search, blacklist avoidance).
- Tier 3 (10 tasks): Cross-floor single-goal, sequential chains, and cross-floor relocation/search.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import heapq
import math
from typing import Any, Callable, Sequence

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    HierarchicalMemoryStore,
    KeyframeSceneGraph,
    RoomNode,
)
from embodied_navigation.planning.exploration.llm_agent_client import LlmClientConfig
from embodied_navigation.planning.exploration.streaming_task_agent import (
    OpenCodeTaskPlanner,
    TaskDecision,
)

VALID_MEMORY_MODES = (
    "hierarchical_6zone",
    "no_m_candidates",
    "no_stale_gate",
    "no_blacklist_gate",
)


@dataclass(frozen=True)
class SeedObject:
    """Grounded semantic object spec separating prior memory from private GT."""

    object_id: str
    label: str
    keyframe_id: str
    local_xyz: tuple[float, float, float] = (0.5, 0.0, 0.0)
    confidence: float = 0.9
    stale: bool = False
    stale_reason: str = ""
    is_ground_truth: bool = True
    in_prior_memory: bool = True
    prior_marked_stale: bool = False


SeedObjectSpec = SeedObject


@dataclass(frozen=True)
class StaticTaskSpec:
    """One reproducible static navigation/memory evaluation task."""

    task_id: str
    tier: str
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
            ra = min(
                fa_rooms,
                key=lambda r: (
                    float(np.linalg.norm(np.asarray(r.centroid_xyz) - np.asarray(st.start_xyz))),
                    r.room_id,
                ),
            )
            rb = min(
                fb_rooms,
                key=lambda r: (
                    float(np.linalg.norm(np.asarray(r.centroid_xyz) - np.asarray(st.end_xyz))),
                    r.room_id,
                ),
            )
            adj[ra.room_id].add(rb.room_id)
            adj[rb.room_id].add(ra.room_id)
    return adj


def _nearest_same_floor_room(
    graph: KeyframeSceneGraph, xyz: Sequence[float]
) -> RoomNode | None:
    """Bind a 3D point to the nearest room on its floor (dz <= 1.0 m); None if unavailable."""
    if not graph.rooms:
        return None
    pos = np.asarray(xyz, dtype=np.float64)
    z = float(pos[2])

    if graph.floors:
        matching_floors = {
            fid
            for fid, fl in graph.floors.items()
            if abs(float(fl.z_mean_m) - z) <= 1.0
        }
        if matching_floors:
            cands = [r for r in graph.rooms.values() if r.floor_id in matching_floors]
            if not cands:
                return None
            return min(
                cands,
                key=lambda r: (float(np.linalg.norm(np.asarray(r.centroid_xyz) - pos)), r.room_id),
            )

    cands = [
        r
        for r in graph.rooms.values()
        if abs(float(r.centroid_xyz[2]) - z) <= 1.0
    ]
    if not cands:
        return None
    return min(
        cands,
        key=lambda r: (float(np.linalg.norm(np.asarray(r.centroid_xyz) - pos)), r.room_id),
    )


def _stair_room_pairs(
    graph: KeyframeSceneGraph,
) -> dict[tuple[str, str], list[tuple[np.ndarray, np.ndarray]]]:
    """Map directed (u_room, v_room) stair connections to (entry_xyz, exit_xyz) portals."""
    mapping: dict[tuple[str, str], list[tuple[np.ndarray, np.ndarray]]] = {}
    for st in graph.stairs:
        fa_rooms = [r for r in graph.rooms.values() if r.floor_id == st.from_floor]
        fb_rooms = [r for r in graph.rooms.values() if r.floor_id == st.to_floor]
        if not fa_rooms or not fb_rooms:
            continue
        p_start = np.asarray(st.start_xyz, dtype=np.float64)
        p_end = np.asarray(st.end_xyz, dtype=np.float64)
        ra = min(
            fa_rooms,
            key=lambda r: (float(np.linalg.norm(np.asarray(r.centroid_xyz) - p_start)), r.room_id),
        )
        rb = min(
            fb_rooms,
            key=lambda r: (float(np.linalg.norm(np.asarray(r.centroid_xyz) - p_end)), r.room_id),
        )
        mapping.setdefault((ra.room_id, rb.room_id), []).append((p_start, p_end))
        mapping.setdefault((rb.room_id, ra.room_id), []).append((p_end, p_start))
    return mapping


def _plan_room_path_with_waypoints(
    graph: KeyframeSceneGraph,
    adj: dict[str, set[str]],
    start_xyz: Sequence[float],
    goal_xyz: Sequence[float],
) -> tuple[bool, list[str], float, list[np.ndarray]]:
    """Plan shortest traversable polyline over room/doorway/stair topology.

    Never falls back to straight-line Euclidean distance when disconnected.
    """
    s_room_node = _nearest_same_floor_room(graph, start_xyz)
    g_room_node = _nearest_same_floor_room(graph, goal_xyz)
    if s_room_node is None or g_room_node is None:
        return (False, [], 0.0, [])

    s_pos = np.asarray(start_xyz, dtype=np.float64)
    g_pos = np.asarray(goal_xyz, dtype=np.float64)
    s_room = s_room_node.room_id
    g_room = g_room_node.room_id

    if s_room == g_room:
        return (True, [s_room], float(np.linalg.norm(g_pos - s_pos)), [s_pos, g_pos])

    door_map: dict[tuple[str, str], list[tuple[int, np.ndarray]]] = {}
    for d_idx, d in enumerate(graph.doorways):
        gw = np.asarray(d.gateway_xyz, dtype=np.float64)
        door_map.setdefault((d.room_a, d.room_b), []).append((d_idx, gw))
        door_map.setdefault((d.room_b, d.room_a), []).append((d_idx, gw))
    stair_map = _stair_room_pairs(graph)

    start_state: tuple[Any, ...] = ("START", s_room)
    best_dist: dict[tuple[Any, ...], float] = {start_state: 0.0}
    seq = 0
    heap: list[tuple[float, int, tuple[Any, ...], np.ndarray, list[str], list[np.ndarray]]] = [
        (0.0, seq, start_state, s_pos, [s_room], [s_pos])
    ]

    while heap:
        d_cur, _, state, p_cur, path, wps = heapq.heappop(heap)
        if d_cur > best_dist.get(state, float("inf")) + 1e-12:
            continue
        if state[0] == "GOAL":
            return (True, path, float(d_cur), wps)

        u = state[1]
        if u == g_room:
            fin_dist = d_cur + float(np.linalg.norm(g_pos - p_cur))
            goal_state: tuple[Any, ...] = ("GOAL", g_room)
            if fin_dist < best_dist.get(goal_state, float("inf")):
                best_dist[goal_state] = fin_dist
                seq += 1
                heapq.heappush(heap, (fin_dist, seq, goal_state, g_pos, path, [*wps, g_pos]))

        for v in sorted(adj.get(u, ())):
            if v not in graph.rooms or v in path:
                continue

            doors = door_map.get((u, v), [])
            stairs = stair_map.get((u, v), [])
            transitions: list[tuple[tuple[Any, ...], float, np.ndarray, list[np.ndarray]]] = []

            for d_idx, gw in doors:
                step_d = float(np.linalg.norm(gw - p_cur))
                transitions.append((("DOOR", v, d_idx), step_d, gw, [gw]))

            for st_idx, (p_in, p_out) in enumerate(stairs):
                step_d = float(np.linalg.norm(p_in - p_cur)) + float(np.linalg.norm(p_out - p_in))
                transitions.append((("STAIR", v, st_idx), step_d, p_out, [p_in, p_out]))

            if not transitions:
                c_u = np.asarray(graph.rooms[u].centroid_xyz, dtype=np.float64)
                c_v = np.asarray(graph.rooms[v].centroid_xyz, dtype=np.float64)
                if state[0] == "START":
                    step_d = float(np.linalg.norm(c_u - p_cur)) + float(np.linalg.norm(c_v - c_u))
                    added_wps = [c_u, c_v]
                else:
                    step_d = float(np.linalg.norm(c_v - p_cur))
                    added_wps = [c_v]
                transitions.append((("CENTROID", v), step_d, c_v, added_wps))

            for nxt_state, step_d, nxt_pos, added_wps in transitions:
                cand = d_cur + step_d
                if cand < best_dist.get(nxt_state, float("inf")):
                    best_dist[nxt_state] = cand
                    seq += 1
                    heapq.heappush(
                        heap,
                        (cand, seq, nxt_state, nxt_pos, [*path, v], [*wps, *added_wps]),
                    )

    return (False, [], 0.0, [])


def _plan_room_path(
    graph: KeyframeSceneGraph,
    adj: dict[str, set[str]],
    start_xyz: Sequence[float],
    goal_xyz: Sequence[float],
) -> tuple[bool, list[str], float]:
    """Plan shortest reachable path over room/doorway/stair topology."""
    reachable, path, dist_m, _ = _plan_room_path_with_waypoints(graph, adj, start_xyz, goal_xyz)
    return (reachable, path, dist_m)


def _room_geodesic_distance(
    graph: KeyframeSceneGraph,
    adj: dict[str, set[str]],
    start_xyz: Sequence[float],
    goal_xyz: Sequence[float],
) -> float:
    """Shortest traversable distance over the room/doorway/stair graph (inf if unreachable)."""
    reachable, _path, dist_m = _plan_room_path(graph, adj, start_xyz, goal_xyz)
    return dist_m if reachable else float("inf")


def _seed_world_xyz(graph: KeyframeSceneGraph, seed: SeedObject) -> np.ndarray:
    pose = graph.keyframe_poses[seed.keyframe_id]
    local_c = np.asarray(seed.local_xyz, dtype=np.float64)
    return pose[:3, :3] @ local_c + pose[:3, 3]


def build_30_static_tasks(graph: KeyframeSceneGraph) -> list[StaticTaskSpec]:
    """Construct 30 grounded static tasks (10 Tier-1 + 10 Tier-2 + 10 Tier-3)."""
    f0_rooms = _room_kfs(graph, "f0")
    f1_rooms = _room_kfs(graph, "f1")
    if "f0" not in graph.floors or "f1" not in graph.floors or not f0_rooms or not f1_rooms:
        raise ValueError("build_30_static_tasks requires a multi-floor graph with f0 and f1")

    def _kf_at(room_list: list[tuple[str, list[str]]], r_idx: int, frac: float = 0.5) -> str:
        _, kfs = room_list[r_idx % len(room_list)]
        return _pick_kf(kfs, frac)

    def _distinct_kf(
        primary: list[tuple[str, list[str]]],
        secondary: list[tuple[str, list[str]]],
        preferred_idx: int,
        exclude_rooms: set[str],
        frac: float = 0.4,
    ) -> tuple[str, str]:
        ordered = [*primary[preferred_idx % len(primary) :], *primary[: preferred_idx % len(primary)], *secondary]
        for rid, kfs in ordered:
            if rid not in exclude_rooms:
                return rid, _pick_kf(kfs, frac)
        rid, kfs = primary[preferred_idx % len(primary)]
        return rid, _pick_kf(kfs, frac)

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
    search_budget_single = max(18, len(graph.rooms) + 8)
    search_budget_cross = max(24, len(graph.rooms) * 2 + 6)

    # ------------------------------------------------------------------
    # Tier 1: 10 Known-Prior Single-Floor Localization & STOP Tasks
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
                max_steps=8,
            )
        )

    # ------------------------------------------------------------------
    # Tier 2: 10 Memory Governance Tasks
    #   - 4 relocated_unobserved (prior NOT marked stale; new pos NOT in prior)
    #   - 3 undiscovered_search (target NOT in prior; must explore rooms)
    #   - 3 blacklist_avoidance (narrow trap filtering)
    # ------------------------------------------------------------------
    relocated_t2_specs = [
        ("T2_01", f0_rooms, f1_rooms, 0, 1, 2, "delivery box"),
        ("T2_02", f0_rooms, f1_rooms, 2, 1, 3, "toolbox"),
        ("T2_03", f1_rooms, f0_rooms, 0, 1, 3, "service cart"),
        ("T2_04", f1_rooms, f0_rooms, 2, 4, 5, "medical kit"),
    ]
    for tid, pri_rooms, sec_rooms, s_idx, o_idx, n_idx, label in relocated_t2_specs:
        s_rid, s_kfs = pri_rooms[s_idx % len(pri_rooms)]
        start_kf = _pick_kf(s_kfs, 0.15)
        old_rid, old_kf = _distinct_kf(pri_rooms, sec_rooms, o_idx, {s_rid}, frac=0.35)
        _, new_kf = _distinct_kf(pri_rooms, sec_rooms, n_idx, {s_rid, old_rid}, frac=0.50)
        old_obj = SeedObject(
            f"{tid}_old",
            label,
            old_kf,
            (0.4, 0.0, 0.0),
            confidence=0.85,
            stale=False,
            stale_reason="",
            is_ground_truth=False,
            in_prior_memory=True,
            prior_marked_stale=False,
        )
        new_obj = SeedObject(
            f"{tid}_new",
            label,
            new_kf,
            (0.5, 0.0, 0.0),
            confidence=0.95,
            stale=False,
            is_ground_truth=True,
            in_prior_memory=False,
            prior_marked_stale=False,
        )
        tasks.append(
            StaticTaskSpec(
                task_id=tid,
                tier="tier2_memory_governance",
                subtype="relocated_unobserved",
                instruction=(
                    f"Find the CURRENT location of '{label}'. If the remembered spot is empty "
                    f"upon arrival, explore to locate it within 1.8 m and call stop_claim."
                ),
                start_keyframe_id=start_kf,
                target_labels=(label,),
                target_object_ids=(new_obj.object_id,),
                seed_objects=(*base_bank, old_obj, new_obj),
                max_steps=search_budget_single,
            )
        )

    undiscovered_t2_specs = [
        ("T2_05", f0_rooms, f1_rooms, 0, 3, "cleaning robot"),
        ("T2_06", f1_rooms, f0_rooms, 0, 4, "emergency radio"),
        ("T2_07", f1_rooms, f0_rooms, 5, 2, "laser scanner"),
    ]
    for tid, pri_rooms, sec_rooms, s_idx, t_idx, label in undiscovered_t2_specs:
        s_rid, s_kfs = pri_rooms[s_idx % len(pri_rooms)]
        start_kf = _pick_kf(s_kfs, 0.15)
        _, tgt_kf = _distinct_kf(pri_rooms, sec_rooms, t_idx, {s_rid}, frac=0.50)
        tgt_obj = SeedObject(
            f"{tid}_target",
            label,
            tgt_kf,
            (0.5, 0.0, 0.0),
            confidence=0.95,
            stale=False,
            is_ground_truth=True,
            in_prior_memory=False,
            prior_marked_stale=False,
        )
        tasks.append(
            StaticTaskSpec(
                task_id=tid,
                tier="tier2_memory_governance",
                subtype="undiscovered_search",
                instruction=(
                    f"'{label}' is NOT in prior memory. Explore unvisited rooms to discover it, "
                    f"approach within 1.8 m, and call stop_claim with target_label='{label}'."
                ),
                start_keyframe_id=start_kf,
                target_labels=(label,),
                target_object_ids=(tgt_obj.object_id,),
                seed_objects=(*base_bank, tgt_obj),
                max_steps=search_budget_single,
            )
        )

    blacklist_t2_specs = [
        ("T2_08", _kf_at(f0_rooms, 2, 0.5), _kf_at(f0_rooms, 1, 0.5), "lm_f0_door"),
        ("T2_09", _kf_at(f1_rooms, 0, 0.2), _kf_at(f1_rooms, 1, 0.5), "lm_f1_board"),
        ("T2_10", _kf_at(f1_rooms, 1, 0.2), _kf_at(f1_rooms, 2, 0.5), "lm_f1_monitor"),
    ]
    for tid, start_kf, trap_kf, lm_id in blacklist_t2_specs:
        lm = lm_by_id[lm_id]
        trap_xyz = graph.keyframe_poses[trap_kf][:3, 3]
        tasks.append(
            StaticTaskSpec(
                task_id=tid,
                tier="tier2_memory_governance",
                subtype="blacklist_avoidance",
                instruction=(
                    f"Navigate to '{lm.label}' while avoiding blacklisted stuck corners, "
                    f"approach within 1.8 m, and issue stop_claim."
                ),
                start_keyframe_id=start_kf,
                target_labels=(lm.label,),
                target_object_ids=(lm.object_id,),
                seed_objects=base_bank,
                blacklist_traps=((float(trap_xyz[0]), float(trap_xyz[1]), float(trap_xyz[2]), "narrow_door_jam"),),
                max_steps=10,
            )
        )

    # ------------------------------------------------------------------
    # Tier 3: 10 Cross-Floor Single-Goal, Dual-Goal Chains & Relocation
    #   - 4 cross-floor known targets (including same-XY wrong-floor trap)
    #   - 3 cross-floor 2-target sequential chains (same-floor + cross-floor)
    #   - 3 cross-floor relocated / undiscovered targets
    # ------------------------------------------------------------------
    cross_known_specs = [
        ("T3_01", "cross_floor_known", _kf_at(f0_rooms, 0, 0.1), "lm_f1_conf"),
        ("T3_02", "cross_floor_known", _kf_at(f0_rooms, 1, 0.3), "lm_f1_print"),
        ("T3_03", "cross_floor_known", _kf_at(f1_rooms, 0, 0.2), "lm_f0_door"),
    ]
    for tid, sub, start_kf, lm_id in cross_known_specs:
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
                max_steps=10,
            )
        )

    # T3_04: Wrong-floor same-XY trap (robot spawns on f0 directly underneath f1 target)
    f1_target_kf = _kf_at(f1_rooms, 0, 0.4)
    f1_xyz = graph.keyframe_poses[f1_target_kf][:3, 3]
    nearest_f0_under = min(
        graph.floors["f0"].keyframe_ids,
        key=lambda kid: math.hypot(
            float(graph.keyframe_poses[kid][0, 3] - f1_xyz[0]),
            float(graph.keyframe_poses[kid][1, 3] - f1_xyz[1]),
        ),
    )
    decoy_obj = SeedObject("T3_04_target", "projector screen", f1_target_kf, (0.3, 0.0, 0.0))
    tasks.append(
        StaticTaskSpec(
            task_id="T3_04",
            tier="tier3_cross_floor_adv",
            subtype="wrong_floor_same_xy_trap",
            instruction=(
                "Reach 'projector screen' on the SAME FLOOR within 1.8 m (do NOT stop directly "
                "underneath it on Floor 0!), then issue stop_claim."
            ),
            start_keyframe_id=nearest_f0_under,
            target_labels=("projector screen",),
            target_object_ids=("T3_04_target",),
            seed_objects=(*base_bank, decoy_obj),
            max_steps=10,
        )
    )

    cross_chain_specs = [
        ("T3_05", _kf_at(f0_rooms, 0, 0.0), ("lm_f0_hose", "lm_f1_conf")),
        ("T3_06", _kf_at(f0_rooms, 1, 0.2), ("lm_f1_conf", "lm_f1_board")),
        ("T3_07", _kf_at(f1_rooms, 2, 0.3), ("lm_f1_print", "lm_f0_desk")),
    ]
    for tid, start_kf, chain_ids in cross_chain_specs:
        labels = tuple(lm_by_id[i].label for i in chain_ids)
        tasks.append(
            StaticTaskSpec(
                task_id=tid,
                tier="tier3_cross_floor_adv",
                subtype="cross_floor_chain",
                instruction=(
                    "Cross-floor sequential chain. Visit and stop_claim each target in order: "
                    + " -> ".join(labels)
                ),
                start_keyframe_id=start_kf,
                target_labels=labels,
                target_object_ids=chain_ids,
                seed_objects=base_bank,
                max_steps=14,
            )
        )

    cross_reloc_specs = [
        ("T3_08", f0_rooms, f1_rooms, 0, 1, 1, "silver laptop"),
        ("T3_09", f1_rooms, f0_rooms, 0, 2, 2, "battery pack"),
    ]
    for tid, start_fl_rooms, new_fl_rooms, s_idx, o_idx, n_idx, label in cross_reloc_specs:
        s_rid, s_kfs = start_fl_rooms[s_idx % len(start_fl_rooms)]
        start_kf = _pick_kf(s_kfs, 0.2)
        old_rid, old_kf = _distinct_kf(start_fl_rooms, new_fl_rooms, o_idx, {s_rid}, frac=0.35)
        _, new_kf = _distinct_kf(new_fl_rooms, start_fl_rooms, n_idx, {s_rid, old_rid}, frac=0.45)
        old_obj = SeedObject(
            f"{tid}_old",
            label,
            old_kf,
            (0.4, 0.0, 0.0),
            confidence=0.85,
            stale=False,
            is_ground_truth=False,
            in_prior_memory=True,
            prior_marked_stale=False,
        )
        new_obj = SeedObject(
            f"{tid}_new",
            label,
            new_kf,
            (0.5, 0.0, 0.0),
            confidence=0.95,
            stale=False,
            is_ground_truth=True,
            in_prior_memory=False,
            prior_marked_stale=False,
        )
        tasks.append(
            StaticTaskSpec(
                task_id=tid,
                tier="tier3_cross_floor_adv",
                subtype="cross_floor_relocated",
                instruction=(
                    f"Find the CURRENT location of '{label}' across floors; if absent at its "
                    f"remembered spot, explore across floors within 1.8 m and call stop_claim."
                ),
                start_keyframe_id=start_kf,
                target_labels=(label,),
                target_object_ids=(new_obj.object_id,),
                seed_objects=(*base_bank, old_obj, new_obj),
                max_steps=search_budget_cross,
            )
        )

    t3_10_obj = SeedObject(
        "T3_10_target",
        "pallet jack",
        _kf_at(f1_rooms, 3, 0.45),
        (0.5, 0.0, 0.0),
        confidence=0.95,
        stale=False,
        is_ground_truth=True,
        in_prior_memory=False,
        prior_marked_stale=False,
    )
    tasks.append(
        StaticTaskSpec(
            task_id="T3_10",
            tier="tier3_cross_floor_adv",
            subtype="cross_floor_undiscovered",
            instruction=(
                "Find undiscovered 'pallet jack' across the building (not in prior memory), "
                "approach within 1.8 m, and call stop_claim with target_label='pallet jack'."
            ),
            start_keyframe_id=_kf_at(f0_rooms, 0, 0.1),
            target_labels=("pallet jack",),
            target_object_ids=(t3_10_obj.object_id,),
            seed_objects=(*base_bank, t3_10_obj),
            max_steps=search_budget_cross,
        )
    )

    if len(tasks) != 30:
        raise AssertionError(f"expected 30 tasks, got {len(tasks)}")
    if len({t.task_id for t in tasks}) != 30:
        raise ValueError("duplicate task_id in build_30_static_tasks")

    # Pre-validate reachability of every task's ground-truth target chain over adj
    adj = _build_room_adjacency(graph)
    for task in tasks:
        seed_map = {s.object_id: s for s in task.seed_objects}
        prev_xyz = graph.keyframe_poses[task.start_keyframe_id][:3, 3]
        for oid in task.target_object_ids:
            seed = seed_map.get(oid)
            if seed is None or not seed.is_ground_truth:
                raise ValueError(f"task {task.task_id} missing ground-truth target {oid}")
            tgt_xyz = _seed_world_xyz(graph, seed)
            ok, _path, _dist = _plan_room_path(graph, adj, prev_xyz, tgt_xyz)
            if not ok:
                raise ValueError(f"task {task.task_id} target {oid} is unreachable from start")
            prev_xyz = tgt_xyz

    return tasks


build_default_30_task_suite = build_30_static_tasks


class _BaseDeterministicFrontierPlanner(OpenCodeTaskPlanner):
    """Shared BFS unvisited-room frontier exploration logic for deterministic baselines."""

    def __init__(self) -> None:
        super().__init__(
            LlmClientConfig(provider="opencode", api_key="offline-deterministic", supports_vision=False)
        )
        self._visited_rooms: set[str] = set()
        self._adj: dict[str, set[str]] = {}
        self._known_rooms: set[str] = set()

    def _reset_topology(self) -> None:
        self._visited_rooms.clear()
        self._adj.clear()
        self._known_rooms.clear()

    def _update_topology(
        self,
        robot_xyz: Sequence[float],
        candidates: Sequence[dict[str, Any]],
        memory_l0: dict[str, Any] | None,
    ) -> str | None:
        floor_z: dict[str, float] = {}
        rooms_meta: list[dict[str, Any]] = []
        if isinstance(memory_l0, dict):
            for fl in memory_l0.get("floors", []):
                if isinstance(fl, dict) and fl.get("id") is not None:
                    floor_z[str(fl["id"])] = float(fl.get("z", 0.0))
            for rm in memory_l0.get("rooms", []):
                if isinstance(rm, dict) and rm.get("id"):
                    rid = str(rm["id"])
                    self._known_rooms.add(rid)
                    self._adj.setdefault(rid, set())
                    rooms_meta.append(rm)
            for door in memory_l0.get("doors", []):
                if isinstance(door, (list, tuple)) and len(door) >= 2:
                    ra, rb = str(door[0]), str(door[1])
                    self._adj.setdefault(ra, set()).add(rb)
                    self._adj.setdefault(rb, set()).add(ra)

        cur_room: str | None = None
        same_floor_rooms = [
            rm
            for rm in rooms_meta
            if abs(floor_z.get(str(rm.get("f", "")), float(robot_xyz[2])) - float(robot_xyz[2])) <= 1.0
        ]
        if not same_floor_rooms:
            same_floor_rooms = rooms_meta
        if same_floor_rooms:
            best_rm = min(
                same_floor_rooms,
                key=lambda rm: (
                    math.hypot(
                        float(rm["c"][0]) - float(robot_xyz[0]),
                        float(rm["c"][1]) - float(robot_xyz[1]),
                    ),
                    str(rm["id"]),
                ),
            )
            cur_room = str(best_rm["id"])
            self._visited_rooms.add(cur_room)

        for cand in candidates:
            if cand.get("kind") == "frontier" and cand.get("label"):
                nbr = str(cand["label"])
                self._known_rooms.add(nbr)
                self._adj.setdefault(nbr, set())
                if cur_room is not None:
                    self._adj.setdefault(cur_room, set()).add(nbr)
                    self._adj.setdefault(nbr, set()).add(cur_room)
        return cur_room

    def _next_frontier_candidate(
        self, cur_room: str | None, candidates: Sequence[dict[str, Any]]
    ) -> dict[str, Any] | None:
        f_cands = [c for c in candidates if c.get("kind") == "frontier" and c.get("id")]
        if not f_cands:
            return None

        unvisited_direct = [
            c for c in f_cands if c.get("label") and str(c["label"]) not in self._visited_rooms
        ]
        if unvisited_direct:
            chosen = min(unvisited_direct, key=lambda c: (float(c.get("dist_m", 999.0)), str(c["id"])))
            self._visited_rooms.add(str(chosen["label"]))
            return chosen

        if cur_room is not None:
            queue: deque[tuple[str, str | None]] = deque([(cur_room, None)])
            seen = {cur_room}
            while queue:
                node, first_hop = queue.popleft()
                for nbr in sorted(self._adj.get(node, ())):
                    if nbr in seen:
                        continue
                    seen.add(nbr)
                    hop = nbr if first_hop is None else first_hop
                    if nbr not in self._visited_rooms:
                        for c in f_cands:
                            if str(c.get("label")) == hop:
                                return c
                    queue.append((nbr, hop))

        # All known reachable rooms visited: reset visited set (keeping cur_room) so multi-goal chains can continue
        self._visited_rooms = {cur_room} if cur_room else set()
        return f_cands[0]


class DeterministicGreedyM1Planner(_BaseDeterministicFrontierPlanner):
    """Model-free baseline: stop or approach M1 if present, else BFS-explore unvisited F* rooms."""

    def decide(
        self,
        *,
        task_instruction: str,
        step_index: int,
        robot_xyz: Sequence[float],
        yaw_rad: float,
        candidates: Sequence[dict[str, Any]],
        memory_summary: str = "",
        memory_l0: dict[str, Any] | None = None,
        memory_l1: Sequence[dict[str, Any]] = (),
        memory_l2: dict[str, Any] | None = None,
        target_label: str | None = None,
        stop_radius_m: float = 1.5,
        estimated_travel_m: float = 0.0,
        last_feedback: str = "",
        tool_recall_result: dict[str, Any] | None = None,
        rgb_path: str | None = None,
        local_map_path: str | None = None,
        extra_rgb_paths: Sequence[str] = (),
    ) -> TaskDecision:
        if step_index <= 1:
            self._reset_topology()
        cur_room = self._update_topology(robot_xyz, candidates, memory_l0)

        for cand in candidates:
            if cand.get("id") == "M1":
                same_floor = abs(float(cand.get("z", robot_xyz[2])) - float(robot_xyz[2])) <= 1.0
                if same_floor and float(cand.get("dist_m", 99.0)) <= float(stop_radius_m):
                    return TaskDecision(
                        action="stop_claim",
                        target_label=target_label,
                        latency_ms=1.0,
                        reason="det_m1_stop",
                    )
                return TaskDecision(
                    action="goto_candidate",
                    target_id="M1",
                    latency_ms=1.0,
                    reason="det_m1_goto",
                )

        nxt = self._next_frontier_candidate(cur_room, candidates)
        if nxt is not None:
            return TaskDecision(
                action="goto_candidate",
                target_id=str(nxt["id"]),
                latency_ms=1.0,
                reason="det_m1_bfs_explore",
            )
        if candidates:
            return TaskDecision(
                action="goto_candidate",
                target_id=str(candidates[0]["id"]),
                latency_ms=1.0,
                reason="det_m1_fallback_cand",
            )
        return TaskDecision(
            action="stop_claim",
            target_label=target_label,
            latency_ms=1.0,
            reason="det_m1_no_cands_stop",
        )


class DeterministicRecallGotoXyzPlanner(_BaseDeterministicFrontierPlanner):
    """Model-free baseline: query_memory -> goto_xyz -> stop_claim, else BFS-explore F* rooms."""

    def __init__(self) -> None:
        super().__init__()
        self._prev_target_label: str | None = None
        self._recalled: bool = False
        self._target_xyz: tuple[float, float, float] | None = None
        self._failed_xyzs: list[tuple[float, float, float]] = []

    def decide(
        self,
        *,
        task_instruction: str,
        step_index: int,
        robot_xyz: Sequence[float],
        yaw_rad: float,
        candidates: Sequence[dict[str, Any]],
        memory_summary: str = "",
        memory_l0: dict[str, Any] | None = None,
        memory_l1: Sequence[dict[str, Any]] = (),
        memory_l2: dict[str, Any] | None = None,
        target_label: str | None = None,
        stop_radius_m: float = 1.5,
        estimated_travel_m: float = 0.0,
        last_feedback: str = "",
        tool_recall_result: dict[str, Any] | None = None,
        rgb_path: str | None = None,
        local_map_path: str | None = None,
        extra_rgb_paths: Sequence[str] = (),
    ) -> TaskDecision:
        if step_index <= 1:
            self._reset_topology()
            self._prev_target_label = target_label
            self._recalled = False
            self._target_xyz = None
            self._failed_xyzs = []
        elif target_label != self._prev_target_label:
            self._prev_target_label = target_label
            self._recalled = False
            self._target_xyz = None

        cur_room = self._update_topology(robot_xyz, candidates, memory_l0)

        if (
            "marked stale" in last_feedback
            or "reobserved_absent" in last_feedback
            or "STOP REJECTED" in last_feedback
            or "STUCK" in last_feedback
            or "unreachable" in last_feedback
        ):
            if self._target_xyz is not None:
                self._failed_xyzs.append(self._target_xyz)
            self._target_xyz = None
            self._recalled = False
        elif "Observed '" in last_feedback and self._target_xyz is None:
            self._recalled = False

        if isinstance(tool_recall_result, dict):
            objs = tool_recall_result.get("l2_objects") or []
            picked: tuple[float, float, float] | None = None
            for obj in objs:
                w = obj.get("world_xyz")
                if not isinstance(w, (list, tuple)) or len(w) < 3:
                    continue
                cand_xyz = (float(w[0]), float(w[1]), float(w[2]))
                if any(math.dist(cand_xyz, fx) <= 0.6 for fx in self._failed_xyzs):
                    continue
                picked = cand_xyz
                break
            self._target_xyz = picked

        if self._target_xyz is not None:
            same_floor = abs(float(robot_xyz[2]) - self._target_xyz[2]) <= 1.0
            dist = math.dist(robot_xyz, self._target_xyz)
            if same_floor and dist <= float(stop_radius_m):
                return TaskDecision(
                    action="stop_claim",
                    target_label=target_label,
                    latency_ms=1.0,
                    reason="det_recall_stop",
                )
            return TaskDecision(
                action="goto_xyz",
                xyz=self._target_xyz,
                latency_ms=1.0,
                reason="det_recall_goto_xyz",
            )

        if not self._recalled:
            self._recalled = True
            return TaskDecision(
                action="query_memory",
                query=target_label,
                latency_ms=1.0,
                reason="det_recall_query",
            )

        nxt = self._next_frontier_candidate(cur_room, candidates)
        if nxt is not None:
            return TaskDecision(
                action="goto_candidate",
                target_id=str(nxt["id"]),
                latency_ms=1.0,
                reason="det_recall_bfs_explore",
            )
        if candidates:
            return TaskDecision(
                action="goto_candidate",
                target_id=str(candidates[0]["id"]),
                latency_ms=1.0,
                reason="det_recall_fallback_cand",
            )
        return TaskDecision(
            action="stop_claim",
            target_label=target_label,
            latency_ms=1.0,
            reason="det_recall_no_cands_stop",
        )


def _point_to_segment_3d(
    p: np.ndarray, a: np.ndarray, b: np.ndarray
) -> tuple[float, np.ndarray]:
    ab = b - a
    len_sq = float(np.dot(ab, ab))
    if len_sq < 1e-12:
        return float(np.linalg.norm(p - a)), a
    t = max(0.0, min(1.0, float(np.dot(p - a, ab) / len_sq)))
    proj = a + t * ab
    return float(np.linalg.norm(p - proj)), proj


def _observe_current_room(
    graph: KeyframeSceneGraph,
    store: HierarchicalMemoryStore,
    robot_xyz: Sequence[float],
    *,
    gt_seeds: dict[str, tuple[SeedObject, np.ndarray, str | None]],
    gt_positions: dict[str, np.ndarray],
    stop_radius_m: float,
    memory_mode: str,
) -> list[str]:
    """Simulate real local room perception: discover GT objects and flag absent prior beliefs."""
    cur_room = _nearest_same_floor_room(graph, robot_xyz)
    cur_room_id = cur_room.room_id if cur_room is not None else None
    r_pos = np.asarray(robot_xyz, dtype=np.float64)
    obs_radius = max(3.5, float(stop_radius_m) + 1.5)
    events: list[str] = []

    for oid, (seed, world_xyz, obj_room_id) in gt_seeds.items():
        if oid in store.graph.objects:
            continue
        same_room = cur_room_id is not None and obj_room_id == cur_room_id
        nearby_same_floor = (
            abs(float(world_xyz[2]) - float(r_pos[2])) <= 1.0
            and float(np.linalg.norm(world_xyz - r_pos)) <= obs_radius
        )
        if same_room or nearby_same_floor:
            store.add_object(
                seed.object_id,
                seed.label,
                seed.keyframe_id,
                seed.local_xyz,
                confidence=seed.confidence,
                producer="offline_room_sensor",
                room_id=obj_room_id,
            )
            events.append(
                f"Observed '{seed.label}' ({seed.object_id}) at "
                f"{[round(float(v), 2) for v in world_xyz]} in {obj_room_id or 'current area'}."
            )

    if memory_mode != "no_stale_gate":
        for oid, obj in list(store.graph.objects.items()):
            if oid in gt_positions:
                continue
            meta = store._object_meta.get(oid)
            if meta is not None and meta.stale:
                continue
            obj_xyz = np.asarray(obj.world_center_xyz, dtype=np.float64)
            same_room = cur_room_id is not None and obj.room_id == cur_room_id
            nearby_same_floor = (
                abs(float(obj_xyz[2]) - float(r_pos[2])) <= 1.0
                and float(np.linalg.norm(obj_xyz - r_pos)) <= 2.5
            )
            if same_room or nearby_same_floor:
                store.mark_object_stale(oid, "reobserved_absent")
                events.append(
                    f"Re-observed '{obj.label}' ({oid}) absent in {obj.room_id or 'current area'}; marked stale."
                )

    return events


def evaluate_single_task_offline(
    base_graph: KeyframeSceneGraph,
    task: StaticTaskSpec,
    planner: OpenCodeTaskPlanner,
    *,
    memory_mode: str = "hierarchical_6zone",
    rgb_lookup: Callable[[Sequence[float]], str | None] | None = None,
) -> dict[str, Any]:
    """Run one grounded closed-loop task over the real scene graph topology."""
    if memory_mode not in VALID_MEMORY_MODES:
        raise ValueError(f"unsupported memory_mode: {memory_mode!r}")

    graph = KeyframeSceneGraph.from_dict(base_graph.to_dict())
    store = HierarchicalMemoryStore(graph, default_stop_radius_m=task.stop_radius_m)

    gt_positions: dict[str, np.ndarray] = {}
    gt_seeds: dict[str, tuple[SeedObject, np.ndarray, str | None]] = {}

    for seed in task.seed_objects:
        world_xyz = _seed_world_xyz(graph, seed)
        obj_room_id = graph._resolve_room_for_keyframe(seed.keyframe_id, world_xyz)
        if seed.is_ground_truth:
            gt_positions[seed.object_id] = world_xyz
            gt_seeds[seed.object_id] = (seed, world_xyz, obj_room_id)
        if seed.in_prior_memory and task.preload_memory:
            store.add_object(
                seed.object_id,
                seed.label,
                seed.keyframe_id,
                seed.local_xyz,
                confidence=seed.confidence,
                producer="seed_memory",
            )
            if (seed.prior_marked_stale or seed.stale) and memory_mode != "no_stale_gate":
                store.mark_object_stale(seed.object_id, reason=seed.stale_reason or "moved")

    for tx, ty, tz, reason in task.blacklist_traps:
        store.record_attempt("goto", [tx, ty, tz], "stuck", reason=reason, radius_m=1.2)

    adj = _build_room_adjacency(graph)
    cur_pose = graph.keyframe_poses[task.start_keyframe_id]
    robot_xyz = [float(v) for v in cur_pose[:3, 3]]
    yaw_rad = math.atan2(float(cur_pose[1, 0]), float(cur_pose[0, 0]))

    # Compute honest optimal length L* using the exact same _plan_room_path movement model
    optimal_len = 0.0
    gt_reachable = True
    prev_ref = list(robot_xyz)
    for oid in task.target_object_ids:
        tgt_xyz = gt_positions.get(oid)
        if tgt_xyz is None:
            gt_reachable = False
            break
        ok, _path, path_dist = _plan_room_path(graph, adj, prev_ref, tgt_xyz)
        if not ok:
            gt_reachable = False
            break
        optimal_len += max(0.0, path_dist - task.stop_radius_m)
        prev_ref = [float(v) for v in tgt_xyz]

    goal_idx = 0
    oracle_idx = 0
    traveled_m = 0.0
    verified_flags = [False] * len(task.target_object_ids)
    stale_flip_errors = 0
    memory_recall_calls = 0
    unreachable_moves = 0
    fallback_count = 0
    steps_used = 0
    last_feedback = "Episode started."
    pending_recall: dict[str, Any] | None = None
    step_latencies: list[float] = []
    step_traces: list[dict[str, Any]] = []

    def _advance_oracle_cursor(waypoints: Sequence[np.ndarray]) -> None:
        nonlocal oracle_idx
        if len(waypoints) == 1:
            segs = [(waypoints[0], waypoints[0])]
        else:
            segs = [(waypoints[i], waypoints[i + 1]) for i in range(len(waypoints) - 1)]
        for a, b in segs:
            while oracle_idx < len(task.target_object_ids):
                oid = task.target_object_ids[oracle_idx]
                gt_xyz = gt_positions.get(oid)
                if gt_xyz is None:
                    break
                dist_m, proj = _point_to_segment_3d(gt_xyz, a, b)
                if abs(float(proj[2]) - float(gt_xyz[2])) <= 1.0 and dist_m <= task.stop_radius_m:
                    oracle_idx += 1
                else:
                    break

    _advance_oracle_cursor([np.asarray(robot_xyz, dtype=np.float64)])

    for step in range(1, task.max_steps + 1):
        steps_used = step
        cur_label = task.target_labels[goal_idx]

        # Local room observation at start of step
        obs_events = _observe_current_room(
            graph,
            store,
            robot_xyz,
            gt_seeds=gt_seeds,
            gt_positions=gt_positions,
            stop_radius_m=task.stop_radius_m,
            memory_mode=memory_mode,
        )
        if obs_events:
            last_feedback = f"{last_feedback} {' '.join(obs_events)}".strip()

        _advance_oracle_cursor([np.asarray(robot_xyz, dtype=np.float64)])

        cur_room = _nearest_same_floor_room(graph, robot_xyz)
        if cur_room is None and graph.rooms:
            cur_room = min(
                graph.rooms.values(),
                key=lambda r: float(np.linalg.norm(np.asarray(r.centroid_xyz) - np.asarray(robot_xyz))),
            )
        if cur_room is not None:
            store.record_room_visit(cur_room.room_id)
            neighbor_ids = sorted(adj.get(cur_room.room_id, set()))
        else:
            neighbor_ids = []

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

        for t_idx, (tx, ty, tz, _reason) in enumerate(task.blacklist_traps, start=1):
            if abs(tz - robot_xyz[2]) > 1.0:
                continue
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

        # Single-variable ablation control:
        # - narrative_summary, L2 query_memory, stale gate, blacklist gate stay identical unless explicitly ablated
        prose = store.narrative_summary(robot_xyz=robot_xyz, target_label=cur_label)
        if memory_mode == "no_m_candidates":
            mem_cands: list[dict[str, Any]] = []
        else:
            mem_cands = store.memory_goal_candidates(
                robot_xyz,
                target_label=cur_label,
                standoff_m=min(1.0, task.stop_radius_m * 0.55),
            )

        raw_cands = [*mem_cands, *f_cands]
        if memory_mode == "no_blacklist_gate":
            cands = [{**c, "blacklisted": False} for c in raw_cands]
        else:
            allowed, blocked = store.filter_candidates(raw_cands, default_z=robot_xyz[2])
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
        step_latencies.append(float(decision.latency_ms))
        cand_ids = [str(c.get("id")) for c in cands if c.get("id")]

        if decision.fallback:
            fallback_count += 1
            last_feedback = (
                f"Planner fallback ({decision.reason or 'invalid_output'}); robot stayed in place."
            )
            step_traces.append(
                {
                    "step": step,
                    "robot_xyz": [round(float(v), 3) for v in robot_xyz],
                    "action": decision.action,
                    "target_id": decision.target_id,
                    "candidate_ids": cand_ids,
                    "fallback": True,
                    "outcome": "fallback_noop",
                    "latency_ms": round(float(decision.latency_ms), 1),
                }
            )
            continue

        outcome = "noop"
        should_break = False

        if decision.action == "query_memory":
            memory_recall_calls += 1
            q = decision.query or cur_label
            q_l1 = store.l1_submap_summaries(query=q, near_xyz=robot_xyz, top_k=3)
            q_l2 = store.l2_submap_entities(
                label=q, include_stale=(memory_mode == "no_stale_gate")
            )
            pending_recall = {"query": q, "l1_rooms": q_l1, "l2_objects": q_l2["objects"][:5]}
            last_feedback = (
                f"query_memory('{q}') -> {len(q_l1)} rooms, {len(q_l2['objects'])} objects."
            )
            outcome = "query_memory_ok"

        elif decision.action == "stop_claim":
            claimed_label = decision.target_label
            if (
                claimed_label is not None
                and " ".join(claimed_label.lower().split()) != " ".join(cur_label.lower().split())
            ):
                store.record_attempt(
                    "stop_claim", robot_xyz, "rejected", reason="target_label_mismatch"
                )
                last_feedback = (
                    f"STOP REJECTED (target_label_mismatch: claimed '{claimed_label}', "
                    f"expected '{cur_label}')."
                )
                outcome = "stop_rejected:target_label_mismatch"
            else:
                ver = store.verify_stop_claim(
                    robot_xyz,
                    target_label=cur_label,
                    stop_radius_m=task.stop_radius_m,
                    require_fresh=(memory_mode != "no_stale_gate"),
                )
                cur_gt_oid = task.target_object_ids[goal_idx]
                cur_gt_xyz = gt_positions[cur_gt_oid]
                gt_same_floor = abs(robot_xyz[2] - float(cur_gt_xyz[2])) <= 1.0
                gt_dist = float(np.linalg.norm(np.asarray(robot_xyz) - cur_gt_xyz))
                truly_at_target = gt_same_floor and gt_dist <= task.stop_radius_m

                if ver.allowed and truly_at_target:
                    verified_flags[goal_idx] = True
                    goal_idx += 1
                    outcome = "stop_verified"
                    if goal_idx >= len(task.target_object_ids):
                        should_break = True
                    else:
                        last_feedback = (
                            f"Sub-goal '{cur_label}' VERIFIED! Now proceed to next target: "
                            f"'{task.target_labels[goal_idx]}'."
                        )
                elif ver.allowed and not truly_at_target:
                    stale_flip_errors += 1
                    outcome = "stop_stale_flip_error"
                    last_feedback = (
                        f"FALSE STOP at stale/moved location (true '{cur_label}' is {gt_dist:.1f} m away)."
                    )
                    should_break = True
                else:
                    store.record_attempt("stop_claim", robot_xyz, "rejected", reason=ver.reason)
                    outcome = f"stop_rejected:{ver.reason}"
                    last_feedback = (
                        f"STOP REJECTED ({ver.reason}, dist={ver.distance_m}). "
                        f"Move closer on the same floor."
                    )

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
                outcome = "goto_unresolved"
                last_feedback = "Unresolved goto target."
            else:
                trap_hit = any(
                    abs(dest[2] - tz) <= 1.0 and math.hypot(dest[0] - tx, dest[1] - ty) <= 1.2
                    for tx, ty, tz, _ in task.blacklist_traps
                )
                if trap_hit:
                    store.record_attempt("goto", dest, "stuck", reason="blacklist_trap_hit")
                    outcome = "goto_stuck_trap"
                    last_feedback = f"goto {dest} STUCK in narrow corner trap; choose another route."
                else:
                    reachable, _room_path, leg_dist, path_wps = _plan_room_path_with_waypoints(
                        graph, adj, robot_xyz, dest
                    )
                    if not reachable:
                        unreachable_moves += 1
                        store.record_attempt(
                            "goto", dest, "unreachable", reason="disconnected_floor_room_graph"
                        )
                        outcome = "unreachable"
                        last_feedback = "goto unreachable: disconnected floor/room graph."
                    else:
                        traveled_m += leg_dist
                        robot_xyz = list(dest)
                        store.record_attempt("goto", dest, "completed")
                        _advance_oracle_cursor(path_wps)
                        post_obs = _observe_current_room(
                            graph,
                            store,
                            robot_xyz,
                            gt_seeds=gt_seeds,
                            gt_positions=gt_positions,
                            stop_radius_m=task.stop_radius_m,
                            memory_mode=memory_mode,
                        )
                        obs_suffix = f" {' '.join(post_obs)}" if post_obs else ""
                        outcome = "goto_completed"
                        last_feedback = (
                            f"goto {[round(v, 2) for v in dest]} completed (+{leg_dist:.1f} m).{obs_suffix}"
                        )
        else:
            outcome = f"in_place:{decision.action}"
            last_feedback = f"action {decision.action} executed in place."

        step_traces.append(
            {
                "step": step,
                "robot_xyz": [round(float(v), 3) for v in robot_xyz],
                "action": decision.action,
                "target_id": decision.target_id,
                "candidate_ids": cand_ids,
                "fallback": False,
                "outcome": outcome,
                "latency_ms": round(float(decision.latency_ms), 1),
            }
        )
        if should_break:
            break

    n_targets = len(task.target_object_ids)
    osr = 1.0 if (n_targets > 0 and oracle_idx == n_targets) else 0.0
    subgoal_osr = round((oracle_idx / n_targets) if n_targets > 0 else 0.0, 4)

    if not gt_reachable:
        sr = 0.0
        spl = 0.0
    else:
        sr = 1.0 if all(verified_flags) else 0.0
        spl = (
            sr * (optimal_len / max(optimal_len, traveled_m))
            if optimal_len > 1e-6
            else sr
        )

    no_fallback_sr = sr if fallback_count == 0 else 0.0
    diag = store.diagnostic_metrics()
    return {
        "task_id": task.task_id,
        "tier": task.tier,
        "subtype": task.subtype,
        "memory_mode": memory_mode,
        "sr": sr,
        "no_fallback_sr": no_fallback_sr,
        "osr": osr,
        "subgoal_osr": subgoal_osr,
        "osr_minus_sr": round(osr - sr, 4),
        "spl": round(spl, 4),
        "gt_reachable": gt_reachable,
        "optimal_len_m": round(optimal_len, 2),
        "traveled_m": round(traveled_m, 2),
        "steps_used": steps_used,
        "max_steps": task.max_steps,
        "memory_recall_calls": memory_recall_calls,
        "stale_flip_errors": stale_flip_errors,
        "unreachable_moves": unreachable_moves,
        "fallback_count": fallback_count,
        "mean_latency_ms": round(float(np.mean(step_latencies)) if step_latencies else 0.0, 1),
        "request_latencies_ms": [round(float(x), 1) for x in step_latencies],
        "step_traces": step_traces,
        "diagnostics": diag,
    }


def aggregate_suite_results(results: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Aggregate 30-task evaluation metrics overall and per difficulty tier."""

    def _percentiles(vals: Sequence[float]) -> tuple[float, float]:
        if not vals:
            return 0.0, 0.0
        arr = np.asarray(vals, dtype=np.float64)
        return float(np.percentile(arr, 50)), float(np.percentile(arr, 95))

    def _summarize(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
        if not rows:
            return {"count": 0}
        n = len(rows)
        sr = sum(r["sr"] for r in rows) / n
        no_fb_sr = sum(
            r.get("no_fallback_sr", r["sr"] if r.get("fallback_count", 0) == 0 else 0.0)
            for r in rows
        ) / n
        osr = sum(r["osr"] for r in rows) / n
        subgoal_osr = sum(r.get("subgoal_osr", r["osr"]) for r in rows) / n
        spl = sum(r["spl"] for r in rows) / n

        req_lats = [
            float(x)
            for r in rows
            for x in r.get("request_latencies_ms", ())
        ]
        task_mean_lats = [
            float(r["mean_latency_ms"])
            for r in rows
            if r.get("request_latencies_ms") or r.get("mean_latency_ms", 0.0) > 0.0
        ]
        req_p50, req_p95 = _percentiles(req_lats)
        task_p50, task_p95 = _percentiles(task_mean_lats)

        total_fallbacks = sum(r.get("fallback_count", 0) for r in rows)
        return {
            "count": n,
            "sr": round(sr, 4),
            "no_fallback_sr": round(no_fb_sr, 4),
            "osr": round(osr, 4),
            "avg_subgoal_osr": round(subgoal_osr, 4),
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
            "total_unreachable_moves": sum(r.get("unreachable_moves", 0) for r in rows),
            "total_fallbacks": total_fallbacks,
            "fallback_steps": total_fallbacks,
            "per_request_latency_p50_ms": round(req_p50, 1),
            "per_request_latency_p95_ms": round(req_p95, 1),
            "per_task_mean_latency_p50_ms": round(task_p50, 1),
            "per_task_mean_latency_p95_ms": round(task_p95, 1),
        }

    tiers = sorted({r["tier"] for r in results})
    return {
        "overall": _summarize(results),
        "by_tier": {t: _summarize([r for r in results if r["tier"] == t]) for t in tiers},
    }
