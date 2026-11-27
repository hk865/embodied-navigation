"""Closed-loop Habitat-Sim HM3D evaluator driven by standalone EmbodiedAgentRuntime."""

from __future__ import annotations

import math
from pathlib import Path
import time
from typing import Any, Callable, Sequence

import numpy as np

from embodied_navigation.agent.adapters.habitat_daemon import HabitatWorldDaemon
from embodied_navigation.agent.prompts import (
    AGENT_RUNTIME_SYSTEM_PROMPT,
    build_vlm_region_summarizer as _build_vlm_region_summarizer,
)
from embodied_navigation.agent.runtime import (
    AgentRunResult,
    EmbodiedAgentRuntime,
    FinishCheckResult,
    ImageAttachment,
    OpenCodeToolChatClient,
    PrefixStableContextManager,
)
from embodied_navigation.agent.tools import (
    ObjectNavFinishVerifier,
    build_agent_tools,
)
from embodied_navigation.common.runtime_io import atomic_write_json
from embodied_navigation.perception.vision_tools import VisionToolClient
from embodied_navigation.planning.exploration.habitat_memory_runner import (
    HabitatObjectGT,
    HabitatSubtaskSpec,
    build_private_hm3d_gt,
    compute_spl,
    hab_to_mem,
    is_geodesic_target_reached,
    mem_to_hab,
)


class HabitatAgentRunner:
    """Closed-loop Habitat-Sim HM3D evaluator driven by standalone EmbodiedAgentRuntime."""

    def __init__(
        self,
        *,
        scene_dataset_config: str | Path,
        scene_id: str | Path,
        chat_client: OpenCodeToolChatClient,
        output_dir: str | Path,
        stop_radius_m: float = 1.8,
        memory_carryover: bool = True,
        reset_occupancy_per_subtask: bool = False,
        vision_client: VisionToolClient | None = None,
        vision_enhance_overlay: bool = True,
        vision_score_threshold: float = 0.25,
        vision_fuse_threshold: float = 0.35,
        max_context_tokens: int = 200_000,
        soft_compaction_tokens: int = 155_000,
        enable_vlm_region_summary: bool = True,
        gpu_device_id: int = 0,
        seed: int = 42,
    ) -> None:
        self.scene_dataset_config = str(scene_dataset_config)
        self.scene_id = str(scene_id)
        self.chat_client = chat_client
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.stop_radius_m = float(stop_radius_m)
        self.memory_carryover = bool(memory_carryover)
        self.reset_occupancy_per_subtask = bool(reset_occupancy_per_subtask)
        self.vision_client = vision_client
        self.vision_enhance_overlay = bool(vision_enhance_overlay)
        self.vision_score_threshold = float(vision_score_threshold)
        self.vision_fuse_threshold = float(vision_fuse_threshold)
        self.max_context_tokens = int(max_context_tokens)
        self.soft_compaction_tokens = int(soft_compaction_tokens)
        self.enable_vlm_region_summary = bool(enable_vlm_region_summary)
        self.gpu_device_id = int(gpu_device_id)
        self.seed = int(seed)

    def _create_sim(self) -> Any:
        import habitat_sim

        sim_cfg = habitat_sim.SimulatorConfiguration()
        sim_cfg.scene_dataset_config_file = self.scene_dataset_config
        sim_cfg.scene_id = self.scene_id
        sim_cfg.enable_physics = True
        sim_cfg.gpu_device_id = self.gpu_device_id

        specs = []
        for uuid_name, stype in (
            ("color_sensor", habitat_sim.SensorType.COLOR),
            ("depth_sensor", habitat_sim.SensorType.DEPTH),
            ("semantic_sensor", habitat_sim.SensorType.SEMANTIC),
        ):
            sp = habitat_sim.CameraSensorSpec()
            sp.uuid = uuid_name
            sp.sensor_type = stype
            sp.resolution = [512, 512]
            sp.position = [0.0, 0.88, 0.0]
            specs.append(sp)

        agent_cfg = habitat_sim.agent.AgentConfiguration()
        agent_cfg.sensor_specifications = specs
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

    @staticmethod
    def _on_sim(sim_obj: Any, fn: Callable[[Any], Any]) -> Any:
        if hasattr(sim_obj, "run_atomic"):
            return sim_obj.run_atomic(fn)
        return fn(sim_obj)

    @staticmethod
    def _geodesic_dist(sim: Any, start_hab: Sequence[float], end_hab: Sequence[float]) -> float:
        import habitat_sim

        sp = habitat_sim.ShortestPath()
        sp.requested_start = np.asarray(start_hab, dtype=np.float32)
        sp.requested_end = np.asarray(end_hab, dtype=np.float32)
        if sim.pathfinder.find_path(sp) and math.isfinite(float(sp.geodesic_distance)):
            return float(sp.geodesic_distance)
        return float("inf")

    def _eval_gt_proximity(
        self,
        sim: Any,
        robot_hab: Sequence[float],
        gt_list: Sequence[HabitatObjectGT],
        stop_radius_m: float,
    ) -> tuple[float, float, bool, HabitatObjectGT | None]:
        def _eval(raw_sim: Any) -> tuple[float, float, bool, HabitatObjectGT | None]:
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
                    d_euc = float(
                        np.linalg.norm(r_mem - np.asarray(gt.center_mem, dtype=np.float64))
                    )
                obj_best_geo = float("inf")
                for vp in gt.viewpoints_hab:
                    d_geo = self._geodesic_dist(raw_sim, robot_hab, vp)
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

        return self._on_sim(sim, _eval)

    def run_sequential_episode(
        self,
        subtasks: Sequence[HabitatSubtaskSpec],
        *,
        start_hab: Sequence[float] | None = None,
        independent_subtasks: bool = False,
        shared_scene_host: Any = None,
        session_id: str | None = None,
        eval_mode_label: str | None = None,
    ) -> dict[str, Any]:
        import habitat_sim
        import quaternion

        t_ep = time.perf_counter()
        owns_sim = shared_scene_host is None
        sid = str(session_id or f"sess_{self.seed}_{int(time.time() * 1000) % 100000}").strip()
        fb_switches_start = int(getattr(self.chat_client, "fallback_switches", 0))
        fb_served_start = int(getattr(self.chat_client, "fallback_served_turns", 0))
        if owns_sim:
            sim: Any = self._create_sim()
        else:
            sim = shared_scene_host.create_session(sid)

        try:
            cot_log_path = self.output_dir / "agent_cot_traces.jsonl"
            cot_log_path.write_text("", encoding="utf-8")

            rng_seq = [0]

            def _seed_pathfinder(raw_sim: Any, tag_offset: int) -> None:
                seq_val = rng_seq[0]
                rng_seq[0] += 1
                raw_sim.pathfinder.seed((self.seed * 1009 + tag_offset * 97 + seq_val) & 0x7FFFFFFF)

            target_cats = sorted({st.target_category.strip().lower() for st in subtasks})
            max_stop_r = max((float(st.stop_radius_m) for st in subtasks), default=self.stop_radius_m)
            if not owns_sim and bool(getattr(shared_scene_host, "gt_precomputed", False)):
                private_gt = shared_scene_host.private_gt
            else:
                private_gt = self._on_sim(
                    sim,
                    lambda raw_sim: build_private_hm3d_gt(
                        raw_sim,
                        target_categories=target_cats,
                        num_samples=260,
                        seed=self.seed,
                        max_view_dist_m=max_stop_r,
                    ),
                )

            def _sample_init_pose(raw_sim: Any) -> np.ndarray:
                _seed_pathfinder(raw_sim, 1)
                if start_hab is not None:
                    pt = np.asarray(start_hab, dtype=np.float32)
                else:
                    avail_cats = [c for c in target_cats if private_gt.get(c)]
                    pt = np.asarray(raw_sim.pathfinder.get_random_navigable_point(), dtype=np.float32)
                    for _ in range(40):
                        cand_pt = np.asarray(
                            raw_sim.pathfinder.get_random_navigable_point(), dtype=np.float32
                        )
                        if all(
                            math.isfinite(
                                self._eval_gt_proximity(
                                    raw_sim, cand_pt, private_gt.get(c, []), max_stop_r
                                )[1]
                            )
                            for c in avail_cats
                        ):
                            pt = cand_pt
                            break
                st0 = raw_sim.agents[0].get_state()
                st0.position = pt
                st0.rotation = quaternion.from_rotation_vector([0.0, 0.0, 0.0])
                raw_sim.agents[0].set_state(st0)
                return pt

            init_pt = self._on_sim(sim, _sample_init_pose)

            reg_summarizer = (
                _build_vlm_region_summarizer(self.chat_client.config, self.output_dir)
                if self.enable_vlm_region_summary
                else None
            )
            daemon = HabitatWorldDaemon(
                sim=sim,
                output_dir=self.output_dir,
                stop_radius_m=self.stop_radius_m,
                vision_client=self.vision_client,
                vision_enhance_overlay=self.vision_enhance_overlay,
                vision_score_threshold=self.vision_score_threshold,
                vision_fuse_threshold=self.vision_fuse_threshold,
                region_summarizer_fn=reg_summarizer,
            )

            shared_context: PrefixStableContextManager | None = None
            runtime: EmbodiedAgentRuntime | None = None

            active_state: dict[str, Any] = {
                "target": "",
                "gt_list": [],
                "stop_radius_m": self.stop_radius_m,
                "max_prim_budget": 500,
                "traveled_m": 0.0,
                "primitives_used": 0,
                "oracle_reached": False,
                "first_stop_attempted": False,
                "first_stop_success": False,
                "false_stops": 0,
                "stop_verified_gt": False,
            }

            def _step_goal(goal_mem: Sequence[float]) -> dict[str, Any]:
                goal_hab = np.asarray(mem_to_hab(goal_mem), dtype=np.float32)

                def _snap(raw_sim: Any) -> np.ndarray | None:
                    sn = np.asarray(raw_sim.pathfinder.snap_point(goal_hab), dtype=np.float32)
                    if not np.all(np.isfinite(sn)) or not raw_sim.pathfinder.is_navigable(sn):
                        return None
                    return sn

                snapped = self._on_sim(sim, _snap)
                if snapped is None:
                    return {"status": "blocked", "reason": "unreachable_goal", "traveled_m": 0.0, "primitives": 0}
                rem_prims = max(0, int(active_state["max_prim_budget"]) - int(active_state["primitives_used"]))
                max_prims = min(24, rem_prims)
                if max_prims <= 0:
                    return {"status": "failed", "reason": "primitive_budget_exhausted", "traveled_m": 0.0, "primitives": 0}

                traveled = 0.0
                taken = 0
                arrived = False
                ferr: str | None = None
                follower_box: list[Any] = [None]
                try:
                    while taken < max_prims and not arrived and ferr is None:
                        chunk = min(4, max_prims - taken)
                        prog = {"taken": 0, "traveled": 0.0, "arrived": False, "err": None, "reached": False}

                        def _run_chunk(raw_sim: Any, n_steps: int = chunk, p: dict[str, Any] = prog) -> None:
                            if follower_box[0] is None:
                                follower_box[0] = habitat_sim.GreedyGeodesicFollower(
                                    raw_sim.pathfinder, raw_sim.agents[0], goal_radius=0.45
                                )
                            fol = follower_box[0]
                            prev_h = np.asarray(raw_sim.agents[0].get_state().position, dtype=np.float64)
                            for _ in range(n_steps):
                                try:
                                    act = fol.next_action_along(snapped)
                                except habitat_sim.errors.GreedyFollowerError:
                                    p["err"] = "follower_blocked"
                                    break
                                if act is None or act == "stop":
                                    p["arrived"] = True
                                    break
                                raw_sim.step(act)
                                p["taken"] += 1
                                cur_h = np.asarray(raw_sim.agents[0].get_state().position, dtype=np.float64)
                                p["traveled"] += float(np.linalg.norm(cur_h - prev_h))
                                prev_h = cur_h
                                if not active_state["oracle_reached"] and not p["reached"]:
                                    _, _, r_pt, _ = self._eval_gt_proximity(
                                        raw_sim, cur_h, active_state["gt_list"], active_state["stop_radius_m"]
                                    )
                                    if r_pt:
                                        p["reached"] = True

                        try:
                            self._on_sim(sim, _run_chunk)
                        finally:
                            ct = int(prog["taken"])
                            cv = float(prog["traveled"])
                            taken += ct
                            active_state["primitives_used"] += ct
                            traveled += cv
                            active_state["traveled_m"] += cv
                            if prog["reached"]:
                                active_state["oracle_reached"] = True
                        arrived = bool(prog["arrived"])
                        ferr = prog["err"]
                        if ct >= 4 and not arrived and ferr is None:
                            daemon.ingest_sensor_frame(force_keyframe=False)
                finally:
                    follower_box[0] = None

                end_mem, _, _ = daemon.agent_pose()
                straight_rem = math.hypot(
                    float(goal_mem[0]) - float(end_mem[0]),
                    float(goal_mem[1]) - float(end_mem[1]),
                )
                if arrived or straight_rem <= 0.65:
                    st_str, reason = "completed", "waypoint_reached"
                elif taken == 0:
                    st_str, reason = "blocked", (ferr or "follower_no_action")
                elif ferr is not None and traveled < 0.25:
                    st_str, reason = "blocked", ferr
                elif traveled < 0.25 and taken >= 4:
                    st_str, reason = "stuck", "no_forward_progress"
                else:
                    st_str, reason = "completed", "partial_progress"
                return {
                    "status": st_str,
                    "reason": reason,
                    "traveled_m": round(traveled, 3),
                    "primitives": taken,
                    "straight_line_to_goal_m": round(straight_rem, 2),
                    "atomic_steps_used": active_state["primitives_used"],
                    "atomic_steps_budget": active_state["max_prim_budget"],
                }

            def _step_prims(act_name: str, count: int) -> dict[str, Any]:
                rem = max(0, int(active_state["max_prim_budget"]) - int(active_state["primitives_used"]))
                n_run = min(max(1, int(count)), rem)
                traveled = 0.0
                done = 0
                while done < n_run:
                    chunk = min(3, n_run - done)
                    prog = {"taken": 0, "traveled": 0.0, "reached": False}

                    def _run_p(raw_sim: Any, n_steps: int = chunk, p: dict[str, Any] = prog) -> None:
                        prev_h = np.asarray(raw_sim.agents[0].get_state().position, dtype=np.float64)
                        for _ in range(n_steps):
                            raw_sim.step(act_name)
                            p["taken"] += 1
                            cur_h = np.asarray(raw_sim.agents[0].get_state().position, dtype=np.float64)
                            p["traveled"] += float(np.linalg.norm(cur_h - prev_h))
                            prev_h = cur_h
                            if not active_state["oracle_reached"] and not p["reached"]:
                                _, _, r_pt, _ = self._eval_gt_proximity(
                                    raw_sim, cur_h, active_state["gt_list"], active_state["stop_radius_m"]
                                )
                                if r_pt:
                                    p["reached"] = True

                    try:
                        self._on_sim(sim, _run_p)
                    finally:
                        ct = int(prog["taken"])
                        cv = float(prog["traveled"])
                        done += ct
                        active_state["primitives_used"] += ct
                        traveled += cv
                        active_state["traveled_m"] += cv
                        if prog["reached"]:
                            active_state["oracle_reached"] = True
                    if ct >= 3:
                        daemon.ingest_sensor_frame(force_keyframe=False)
                return {
                    "status": "completed",
                    "action": act_name,
                    "primitives": done,
                    "traveled_m": round(traveled, 3),
                    "atomic_steps_used": active_state["primitives_used"],
                    "atomic_steps_budget": active_state["max_prim_budget"],
                }

            def _step_stairs(
                direction: str, anchor_xy: Sequence[float] | None = None
            ) -> dict[str, Any]:
                rem_prims = max(
                    0, int(active_state["max_prim_budget"]) - int(active_state["primitives_used"])
                )
                if rem_prims <= 0:
                    return {
                        "status": "failed",
                        "reason": "primitive_budget_exhausted",
                        "traveled_m": 0.0,
                        "primitives": 0,
                    }
                start_mem, _, start_h = daemon.agent_pose()
                ax = float(anchor_xy[0]) if (anchor_xy and len(anchor_xy) >= 2) else float(start_mem[0])
                ay = float(anchor_xy[1]) if (anchor_xy and len(anchor_xy) >= 2) else float(start_mem[1])
                cur_y = float(start_h[1])

                def _find_landing(raw_sim: Any) -> np.ndarray | None:
                    _seed_pathfinder(raw_sim, 2)
                    best_pt: np.ndarray | None = None
                    best_sc = float("inf")
                    for radius_m in (4.5, 7.5, 11.0):
                        for _ in range(180):
                            cand = np.asarray(
                                raw_sim.pathfinder.get_random_navigable_point(), dtype=np.float32
                            )
                            if not np.all(np.isfinite(cand)):
                                continue
                            dy = float(cand[1]) - cur_y
                            if direction == "down" and not (-4.5 <= dy <= -1.2):
                                continue
                            if direction == "up" and not (1.2 <= dy <= 4.5):
                                continue
                            c_mem = hab_to_mem(cand)
                            hdist = math.hypot(float(c_mem[0]) - ax, float(c_mem[1]) - ay)
                            if hdist > radius_m:
                                continue
                            sp = habitat_sim.ShortestPath()
                            sp.requested_start = start_h
                            sp.requested_end = cand
                            if not raw_sim.pathfinder.find_path(sp) or not math.isfinite(
                                float(sp.geodesic_distance)
                            ):
                                continue
                            geo = float(sp.geodesic_distance)
                            if geo > 20.0:
                                continue
                            sc_val = geo + 0.35 * hdist
                            if sc_val < best_sc:
                                best_sc = sc_val
                                best_pt = cand
                        if best_pt is not None:
                            break
                    return best_pt

                best_landing = self._on_sim(sim, _find_landing)
                if best_landing is None:
                    return {
                        "status": "blocked",
                        "reason": "no_connected_stair_landing",
                        "direction": direction,
                        "traveled_m": 0.0,
                        "primitives": 0,
                        "atomic_steps_used": active_state["primitives_used"],
                        "atomic_steps_budget": active_state["max_prim_budget"],
                    }

                max_prims = min(48, rem_prims)
                traveled = 0.0
                taken = 0
                arrived = False
                ferr: str | None = None
                stair_follower_box: list[Any] = [None]
                try:
                    while taken < max_prims and not arrived and ferr is None:
                        chunk = min(4, max_prims - taken)
                        prog = {"taken": 0, "traveled": 0.0, "arrived": False, "err": None, "reached": False}

                        def _run_stair_chunk(
                            raw_sim: Any, n_steps: int = chunk, p: dict[str, Any] = prog
                        ) -> None:
                            if stair_follower_box[0] is None:
                                stair_follower_box[0] = habitat_sim.GreedyGeodesicFollower(
                                    raw_sim.pathfinder, raw_sim.agents[0], goal_radius=0.50
                                )
                            fol = stair_follower_box[0]
                            prev_h = np.asarray(raw_sim.agents[0].get_state().position, dtype=np.float64)
                            for _ in range(n_steps):
                                try:
                                    act = fol.next_action_along(best_landing)
                                except habitat_sim.errors.GreedyFollowerError:
                                    p["err"] = "follower_blocked"
                                    break
                                if act is None or act == "stop":
                                    p["arrived"] = True
                                    break
                                raw_sim.step(act)
                                p["taken"] += 1
                                cur_h = np.asarray(raw_sim.agents[0].get_state().position, dtype=np.float64)
                                p["traveled"] += float(np.linalg.norm(cur_h - prev_h))
                                prev_h = cur_h
                                if not active_state["oracle_reached"] and not p["reached"]:
                                    _, _, r_pt, _ = self._eval_gt_proximity(
                                        raw_sim, cur_h, active_state["gt_list"], active_state["stop_radius_m"]
                                    )
                                    if r_pt:
                                        p["reached"] = True

                        try:
                            self._on_sim(sim, _run_stair_chunk)
                        finally:
                            ct = int(prog["taken"])
                            cv = float(prog["traveled"])
                            taken += ct
                            active_state["primitives_used"] += ct
                            traveled += cv
                            active_state["traveled_m"] += cv
                            if prog["reached"]:
                                active_state["oracle_reached"] = True
                        arrived = bool(prog["arrived"])
                        ferr = prog["err"]
                        if ct >= 4 and not arrived and ferr is None:
                            daemon.ingest_sensor_frame(force_keyframe=False)
                finally:
                    stair_follower_box[0] = None

                end_mem, _, _ = daemon.agent_pose()
                dz = float(end_mem[2]) - float(start_mem[2])
                if abs(dz) >= 1.2:
                    st_str, reason = "completed", "floor_changed"
                elif abs(dz) >= 0.35 or (taken > 0 and traveled >= 0.5):
                    st_str, reason = "completed", "partial_stair_progress"
                elif arrived:
                    st_str, reason = "completed", "stair_landing_reached"
                else:
                    st_str, reason = "blocked", (ferr or "stair_no_progress")
                return {
                    "status": st_str,
                    "reason": reason,
                    "direction": direction,
                    "dz_m": round(dz, 3),
                    "traveled_m": round(traveled, 3),
                    "primitives": taken,
                    "atomic_steps_used": active_state["primitives_used"],
                    "atomic_steps_budget": active_state["max_prim_budget"],
                }

            def _record_zone_note(zone_type: str, note: str) -> None:
                robot_mem, _, _ = daemon.agent_pose()
                daemon.memory.record_zone_observation(
                    robot_mem, zone_type=zone_type, summary=note
                )
                if daemon.region_memory.current_region_id:
                    daemon.region_memory.annotate_region(
                        daemon.region_memory.current_region_id,
                        primary_label=zone_type,
                        detailed_description=note,
                        source="agent_annotated",
                    )

            tools = build_agent_tools(
                daemon=daemon,
                active_target_getter=lambda: str(active_state["target"]),
                step_goal_fn=_step_goal,
                step_primitives_fn=_step_prims,
                step_stairs_fn=_step_stairs,
                record_zone_annotation_fn=_record_zone_note,
            )

            subtask_reports: list[dict[str, Any]] = []
            for sub_idx, subtask in enumerate(subtasks, start=1):
                if independent_subtasks and sub_idx > 1:
                    daemon.reset_world(keep_memory=False, keep_occupancy=False)
                    shared_context = None
                    runtime = None
                    cat_gts = private_gt.get(subtask.target_category.strip().lower(), [])

                    def _respawn(raw_sim: Any) -> None:
                        _seed_pathfinder(raw_sim, 3)
                        sub_init_pt = init_pt
                        for _ in range(40):
                            cand_pt = np.asarray(
                                raw_sim.pathfinder.get_random_navigable_point(), dtype=np.float32
                            )
                            if not cat_gts:
                                sub_init_pt = cand_pt
                                break
                            _, dg, _, _ = self._eval_gt_proximity(
                                raw_sim, cand_pt, cat_gts, subtask.stop_radius_m
                            )
                            if math.isfinite(dg) and dg >= 1.5:
                                sub_init_pt = cand_pt
                                break
                        st_r = raw_sim.agents[0].get_state()
                        st_r.position = sub_init_pt
                        st_r.rotation = quaternion.from_rotation_vector([0.0, 0.0, 0.0])
                        raw_sim.agents[0].set_state(st_r)

                    self._on_sim(sim, _respawn)
                elif sub_idx > 1:
                    if not self.memory_carryover:
                        daemon.reset_world(keep_memory=False, keep_occupancy=not self.reset_occupancy_per_subtask)
                        shared_context = None
                        runtime = None
                    elif self.reset_occupancy_per_subtask:
                        daemon.reset_world(keep_memory=True, keep_occupancy=False)

                gt_list = private_gt.get(subtask.target_category.strip().lower(), [])
                _, _, sub_start_h = daemon.agent_pose()
                _, optimal_geo_m, reached_init, _ = self._eval_gt_proximity(
                    sim, sub_start_h, gt_list, subtask.stop_radius_m
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
                            "memory_diagnostics": daemon.memory.diagnostic_metrics(),
                        }
                    )
                    continue

                active_state.update(
                    {
                        "target": subtask.target_category,
                        "gt_list": gt_list,
                        "stop_radius_m": float(subtask.stop_radius_m),
                        "max_prim_budget": max(1, int(subtask.max_primitive_steps)),
                        "traveled_m": 0.0,
                        "primitives_used": 0,
                        "oracle_reached": bool(reached_init),
                        "first_stop_attempted": False,
                        "first_stop_success": False,
                        "false_stops": 0,
                        "stop_verified_gt": False,
                    }
                )

                daemon.stop_radius_m = float(subtask.stop_radius_m)
                daemon.memory.default_stop_radius_m = float(subtask.stop_radius_m)
                ing0 = daemon.ingest_sensor_frame(
                    force_keyframe=True,
                    run_baseline_vision=True,
                    active_target=subtask.target_category,
                )
                fp0 = daemon.render_floorplan_if_needed(subtask.target_category, force=True)
                init_imgs: list[ImageAttachment] = []
                if daemon.current_vis_rgb_path is not None:
                    init_imgs.append(ImageAttachment(daemon.current_vis_rgb_path, "front RGB camera"))
                if fp0 is not None:
                    init_imgs.append(ImageAttachment(fp0, "global 2.5D floorplan"))

                cands0 = daemon.current_candidates(subtask.target_category)
                init_obs = {
                    "subtask": subtask.subtask_id,
                    "target_category": subtask.target_category,
                    "robot_xyz": ing0["robot_mem"],
                    "yaw_deg": ing0["yaw_deg"],
                    "region_id": ing0["zone_id"],
                    "region_label": ing0.get("region_label", "open_area"),
                    "keyframe_id": ing0["keyframe_id"],
                    "map_revision": daemon.memory.map_revision,
                    "candidates": cands0,
                    "detections": [g.to_prompt_dict() for g in daemon.current_detections[:8]],
                    "global_region_memory": daemon.region_memory.region_summaries(ing0["robot_mem"]),
                }

                # Create persistent PrefixStableContextManager on first valid subtask (or reset)
                is_transition = shared_context is not None
                if shared_context is None:
                    shared_context = PrefixStableContextManager(
                        system_prompt=AGENT_RUNTIME_SYSTEM_PROMPT,
                        initial_task_prompt=(
                            f"## [TASK {sub_idx}]\n"
                            f"{subtask.instruction}\n"
                            f"Target: '{subtask.target_category}', stop_radius_m={subtask.stop_radius_m:.1f}, "
                            f"atomic_budget={subtask.max_primitive_steps}."
                        ),
                        allowed_image_roots=[self.output_dir.resolve()],
                        max_context_tokens=self.max_context_tokens,
                        soft_compaction_tokens=self.soft_compaction_tokens,
                        keep_recent_groups=4,
                    )

                def _on_stop_attempt(ver: FinishCheckResult) -> None:
                    is_first = not active_state["first_stop_attempted"]
                    active_state["first_stop_attempted"] = True
                    if ver.allowed:
                        _, _, r_hab = daemon.agent_pose()
                        _, _, reached_now, _ = self._eval_gt_proximity(
                            sim, r_hab, gt_list, subtask.stop_radius_m
                        )
                        if reached_now:
                            active_state["stop_verified_gt"] = True
                            active_state["oracle_reached"] = True
                            if is_first:
                                active_state["first_stop_success"] = True
                        else:
                            active_state["false_stops"] += 1

                verifier = ObjectNavFinishVerifier(
                    daemon=daemon,
                    target_category=subtask.target_category,
                    stop_radius_m=subtask.stop_radius_m,
                    on_attempt_cb=_on_stop_attempt,
                )

                def _tail_status_hook(_t_idx: int) -> str:
                    rid = daemon.region_memory.current_region_id or daemon.current_zone_id or "R?"
                    reg_obj = daemon.region_memory.regions.get(str(rid))
                    rlbl = reg_obj.primary_label if reg_obj is not None else "open_area"
                    tex_m = daemon.texture_selector.diagnostic_metrics()
                    return (
                        f"[map_rev={daemon.memory.map_revision} | region={rid}({rlbl}) | "
                        f"regions={len(daemon.region_memory.regions)} | "
                        f"tex_kfs={tex_m['retained_texture_keyframes']}/{tex_m['total_candidates_admitted']} | "
                        f"atomic={active_state['primitives_used']}/{active_state['max_prim_budget']}]"
                    )

                runtime = EmbodiedAgentRuntime(
                    chat_client=self.chat_client,
                    context=shared_context,
                    before_turn_hook=_tail_status_hook,
                    before_finish_hook=verifier,
                    world_milestone_fn=daemon.milestone_summary,
                    budget_exhausted_fn=lambda: int(active_state["primitives_used"]) >= int(active_state["max_prim_budget"]),
                    motion_progress_fn=lambda: int(active_state["primitives_used"]),
                    max_consecutive_zero_motion_turns=12,
                )
                for spec, handler in tools:
                    runtime.register_tool(spec, handler)

                import json as _json

                res: AgentRunResult = runtime.run_task(
                    task_label=subtask.target_category,
                    initial_observation_text=_json.dumps(init_obs, ensure_ascii=False, separators=(",", ":")),
                    initial_images=tuple(init_imgs),
                    max_turns=subtask.max_steps,
                    append_transition=is_transition,
                    transition_prompt=(
                        f"## [NEXT_TASK {sub_idx}/{len(subtasks)}]\n"
                        f"{subtask.instruction}\n"
                        f"Remember to check query_world('{subtask.target_category}') or M* candidates from earlier zones!"
                    ),
                )

                with cot_log_path.open("a", encoding="utf-8") as cf:
                    for tr in res.traces:
                        cf.write(
                            _json.dumps(
                                {
                                    "subtask_id": subtask.subtask_id,
                                    "target": subtask.target_category,
                                    **tr,
                                },
                                ensure_ascii=False,
                            )
                            + "\n"
                        )

                _, _, end_h = daemon.agent_pose()
                end_euc, end_geo, _, _ = self._eval_gt_proximity(
                    sim, end_h, gt_list, subtask.stop_radius_m
                )
                sr = 1.0 if active_state["stop_verified_gt"] else 0.0
                first_sr = 1.0 if active_state["first_stop_success"] else 0.0
                osr = 1.0 if active_state["oracle_reached"] else 0.0
                spl = compute_spl(
                    bool(active_state["stop_verified_gt"]),
                    optimal_geo_m,
                    float(active_state["traveled_m"]),
                )
                print(
                    f"[AgentRT sub {sub_idx}:{subtask.target_category}] sr={sr} osr={osr} spl={spl:.3f} "
                    f"turns={res.turns_used} prims={active_state['primitives_used']} "
                    f"false_stops={active_state['false_stops']} end_geo={end_geo:.2f}m "
                    f"term={res.termination_reason} cache_hit={res.context_metrics.get('cache_hit_rate')}",
                    flush=True,
                )
                subtask_reports.append(
                    {
                        "subtask_id": subtask.subtask_id,
                        "target_category": subtask.target_category,
                        "valid_gt": True,
                        "sr": sr,
                        "first_stop_sr": first_sr,
                        "osr": osr,
                        "spl": round(spl, 4),
                        "optimal_geodesic_m": round(optimal_geo_m, 2),
                        "traveled_m": round(float(active_state["traveled_m"]), 2),
                        "steps_used": res.turns_used,
                        "primitives_used": int(active_state["primitives_used"]),
                        "max_primitive_steps": int(active_state["max_prim_budget"]),
                        "false_stops": int(active_state["false_stops"]),
                        "end_gt_euc_m": (None if not math.isfinite(end_euc) else round(end_euc, 3)),
                        "end_gt_geo_m": (None if not math.isfinite(end_geo) else round(end_geo, 3)),
                        "termination_reason": res.termination_reason,
                        "context_metrics": res.context_metrics,
                        "memory_diagnostics": daemon.memory.diagnostic_metrics(),
                    }
                )

            valid_reports = [r for r in subtask_reports if r.get("valid_gt", True)]
            denom = max(1, len(valid_reports))
            ctx_final = shared_context.metrics_dict() if shared_context is not None else {}
            ctx_final["provider_fallback_switches"] = max(
                0, int(getattr(self.chat_client, "fallback_switches", 0)) - fb_switches_start
            )
            ctx_final["provider_fallback_served_turns"] = max(
                0, int(getattr(self.chat_client, "fallback_served_turns", 0)) - fb_served_start
            )
            resolved_eval_mode = (
                str(eval_mode_label)
                if eval_mode_label
                else ("independent_cold_start" if independent_subtasks else "sequential_multi_task")
            )
            summary = {
                "scene_id": self.scene_id,
                "session_id": sid,
                "seed": self.seed,
                "runtime": "standalone_embodied_agent_runtime_v1",
                "eval_mode": resolved_eval_mode,
                "memory_carryover": (False if independent_subtasks else self.memory_carryover),
                "occupancy_carryover": (False if independent_subtasks else not self.reset_occupancy_per_subtask),
                "vision_tools": {
                    "enabled": bool(daemon.vision_client is not None and daemon.vision_ok),
                    "backend": daemon.vision_backend,
                    "embed_model": str(daemon.vision_health.get("embed_model", "")),
                    "total_detections": daemon.total_vision_dets,
                    "total_fused": daemon.total_vision_fused,
                    "total_deferred_hits": daemon.total_deferred_hits,
                    "total_refutations": daemon.total_refutations,
                },
                "runtime_metrics": ctx_final,
                "planner_config": self.chat_client.config.to_safe_dict(),
                "fallback_configs": [
                    fc.to_safe_dict() for fc in getattr(self.chat_client, "fallback_configs", ())
                ],
                "vlm_region_summary_route_policy": "primary_route_only",
                "subtasks_count": len(subtask_reports),
                "valid_subtasks_count": len(valid_reports),
                "invalid_subtasks_count": len(subtask_reports) - len(valid_reports),
                "sr": round(sum(r["sr"] for r in valid_reports) / denom, 4) if valid_reports else 0.0,
                "first_stop_sr": (
                    round(sum(r["first_stop_sr"] for r in valid_reports) / denom, 4)
                    if valid_reports
                    else 0.0
                ),
                "osr": round(sum(r["osr"] for r in valid_reports) / denom, 4) if valid_reports else 0.0,
                "spl": round(sum(r["spl"] for r in valid_reports) / denom, 4) if valid_reports else 0.0,
                "false_stops": sum(int(r.get("false_stops", 0)) for r in valid_reports),
                "total_traveled_m": round(sum(r["traveled_m"] for r in valid_reports), 2),
                "total_steps": sum(r["steps_used"] for r in valid_reports),
                "total_primitives": sum(int(r.get("primitives_used", 0)) for r in valid_reports),
                "motion_index_frames": len(daemon.motion_index),
                "keyframes_total": len(daemon.memory.graph.keyframe_poses),
                "scene_graph_rooms": len(daemon.memory.graph.rooms),
                "scene_graph_doorways": len(daemon.memory.graph.doorways),
                "scene_graph_stairs": len(daemon.memory.graph.stairs),
                "texture_keyframe_metrics": daemon.texture_selector.diagnostic_metrics(),
                "vlm_region_summaries": daemon.total_vlm_region_summaries,
                "global_regions": daemon.region_memory.region_summaries(),
                "wall_s": round(time.perf_counter() - t_ep, 2),
                "private_gt_counts": {k: len(v) for k, v in private_gt.items()},
                "subtasks": subtask_reports,
            }
            daemon.memory.save(self.output_dir / "habitat_hierarchical_memory.json")
            atomic_write_json(self.output_dir / "habitat_eval_report.json", summary)
            return summary
        finally:
            if owns_sim:
                sim.close()
            elif hasattr(sim, "drop"):
                sim.drop()
            else:
                shared_scene_host.drop_session(sid)
