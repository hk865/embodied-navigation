"""Unit tests for the 30-task static memory & navigation benchmark suite."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys
import unittest

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    KeyframeSceneGraph,
)
from embodied_navigation.planning.exploration.llm_agent_client import LlmClientConfig
from embodied_navigation.planning.exploration.static_task_suite import (
    DeterministicGreedyM1Planner,
    DeterministicRecallGotoXyzPlanner,
    SeedObject,
    StaticTaskSpec,
    VALID_MEMORY_MODES,
    _build_room_adjacency,
    _plan_room_path,
    _room_geodesic_distance,
    aggregate_suite_results,
    build_30_static_tasks,
    evaluate_single_task_offline,
)
from embodied_navigation.planning.exploration.streaming_task_agent import (
    OpenCodeTaskPlanner,
    TaskDecision,
)


def _pose(x: float, y: float, z: float) -> np.ndarray:
    m = np.eye(4, dtype=np.float64)
    m[:3, 3] = [x, y, z]
    return m


def _synthetic_two_floor_graph() -> KeyframeSceneGraph:
    g = KeyframeSceneGraph()
    for r_idx in range(4):
        x0 = r_idx * 6.0
        for k in range(4):
            g.add_keyframe(f"f0_r{r_idx}_k{k}", _pose(x0 + k * 0.8, 0.0, 0.0), clearance_m=1.8)
        if r_idx < 3:
            g.add_keyframe(f"f0_door_{r_idx}", _pose(x0 + 4.2, 0.0, 0.0), clearance_m=0.45)
    for s in range(4):
        g.add_keyframe(f"stair_{s}", _pose(18.0, 1.0 + s, 0.65 * (s + 1)), clearance_m=1.0)
    for r_idx in range(6):
        x0 = r_idx * 6.0
        for k in range(4):
            g.add_keyframe(f"f1_r{r_idx}_k{k}", _pose(x0 + k * 0.8, 6.0, 3.2), clearance_m=1.8)
        if r_idx < 5:
            g.add_keyframe(f"f1_door_{r_idx}", _pose(x0 + 4.2, 6.0, 3.2), clearance_m=0.45)
    g.segment_floors()
    g.partition_rooms()
    return g


def _synthetic_single_floor_graph() -> KeyframeSceneGraph:
    g = KeyframeSceneGraph()
    for r_idx in range(3):
        x0 = r_idx * 6.0
        for k in range(4):
            g.add_keyframe(f"f0_r{r_idx}_k{k}", _pose(x0 + k * 0.8, 0.0, 0.0), clearance_m=1.8)
        if r_idx < 2:
            g.add_keyframe(f"f0_door_{r_idx}", _pose(x0 + 4.2, 0.0, 0.0), clearance_m=0.45)
    g.segment_floors()
    g.partition_rooms()
    return g


def _load_eval_script_module():
    script_path = Path(__file__).resolve().parents[2] / "scripts" / "eval_static_task_suite.py"
    spec = importlib.util.spec_from_file_location("eval_static_task_suite", script_path)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules["eval_static_task_suite"] = mod
    spec.loader.exec_module(mod)
    return mod


class _ScriptedPlanner(OpenCodeTaskPlanner):
    """Test helper planner driven by a callback."""

    def __init__(self, fn) -> None:
        super().__init__(LlmClientConfig(provider="opencode", api_key="sk-test", supports_vision=False))
        self._fn = fn

    def decide(self, **kwargs) -> TaskDecision:
        return self._fn(**kwargs)


class TestStaticTaskSuite(unittest.TestCase):
    def test_single_floor_graph_fail_closed(self) -> None:
        g1 = _synthetic_single_floor_graph()
        with self.assertRaises(ValueError):
            build_30_static_tasks(g1)

    def test_plan_room_path_portals_and_disconnected_no_fallback(self) -> None:
        g = _synthetic_two_floor_graph()
        adj = _build_room_adjacency(g)

        # Same room: exact Euclidean distance
        p0 = g.keyframe_poses["f0_r0_k0"][:3, 3]
        p1 = g.keyframe_poses["f0_r0_k3"][:3, 3]
        ok, path, dist = _plan_room_path(g, adj, p0, p1)
        self.assertTrue(ok)
        self.assertEqual(len(path), 1)
        self.assertAlmostEqual(dist, float(np.linalg.norm(p1 - p0)), places=6)

        # Cross-room with doorway: polyline via doorway gateway (no 0.5 scaling)
        p_r1 = g.keyframe_poses["f0_r1_k2"][:3, 3]
        gw = np.asarray(g.doorways[0].gateway_xyz, dtype=np.float64)
        expected_door_poly = float(np.linalg.norm(gw - p0) + np.linalg.norm(p_r1 - gw))
        ok_d, path_d, dist_d = _plan_room_path(g, adj, p0, p_r1)
        self.assertTrue(ok_d)
        self.assertEqual(len(path_d), 2)
        self.assertAlmostEqual(dist_d, expected_door_poly, places=5)

        # Disconnected graph: must return (False, [], 0.0) and inf geodesic, NEVER straight-line fallback
        g.doorways.clear()
        g.stairs.clear()
        adj_empty = _build_room_adjacency(g)
        p_f1 = g.keyframe_poses["f1_r0_k0"][:3, 3]
        ok_disc, path_disc, dist_disc = _plan_room_path(g, adj_empty, p0, p_f1)
        self.assertFalse(ok_disc)
        self.assertEqual(path_disc, [])
        self.assertEqual(dist_disc, 0.0)
        self.assertTrue(math.isinf(_room_geodesic_distance(g, adj_empty, p0, p_f1)))

    def test_disconnected_graph_blocks_cross_floor_teleport_on_all_t3(self) -> None:
        g = _synthetic_two_floor_graph()
        suite = build_30_static_tasks(g)
        t3_tasks = [t for t in suite if t.tier == "tier3_cross_floor_adv"]
        self.assertEqual(len(t3_tasks), 10)

        # Delete ALL doorways and stairs from the graph
        g.doorways.clear()
        g.stairs.clear()

        def _cross_floor_goto_fn(**kwargs) -> TaskDecision:
            cands = kwargs.get("candidates") or []
            for c in cands:
                if c.get("id") == "M1":
                    return TaskDecision(action="goto_candidate", target_id="M1", latency_ms=2.0)
            # For undiscovered/cross-floor tasks, attempt direct goto_xyz onto the other floor
            rz = float(kwargs["robot_xyz"][2])
            target_z = 3.2 if rz < 1.5 else 0.0
            return TaskDecision(action="goto_xyz", xyz=(0.5, 6.0 if target_z > 1.5 else 0.0, target_z), latency_ms=2.0)

        planner = _ScriptedPlanner(_cross_floor_goto_fn)
        for task in t3_tasks:
            res = evaluate_single_task_offline(g, task, planner, memory_mode="hierarchical_6zone")
            self.assertFalse(res["gt_reachable"], msg=task.task_id)
            self.assertEqual(res["sr"], 0.0, msg=task.task_id)
            self.assertEqual(res["spl"], 0.0, msg=task.task_id)
            self.assertEqual(res["traveled_m"], 0.0, msg=task.task_id)
            self.assertGreaterEqual(res["unreachable_moves"], 1, msg=task.task_id)
            self.assertTrue(
                any(tr["outcome"] == "unreachable" for tr in res["step_traces"]),
                msg=task.task_id,
            )

    def test_relocated_unobserved_private_gt_and_stale_gate(self) -> None:
        g = _synthetic_two_floor_graph()
        suite = build_30_static_tasks(g)
        reloc_tasks = [t for t in suite if t.subtype == "relocated_unobserved"]
        self.assertEqual(len(reloc_tasks), 4)

        for task in reloc_tasks:
            old_seeds = [s for s in task.seed_objects if s.object_id.endswith("_old")]
            new_seeds = [s for s in task.seed_objects if s.object_id.endswith("_new")]
            self.assertEqual(len(old_seeds), 1)
            self.assertEqual(len(new_seeds), 1)
            self.assertTrue(old_seeds[0].in_prior_memory)
            self.assertFalse(old_seeds[0].is_ground_truth)
            self.assertFalse(old_seeds[0].prior_marked_stale)
            self.assertFalse(old_seeds[0].stale)
            self.assertFalse(new_seeds[0].in_prior_memory)
            self.assertTrue(new_seeds[0].is_ground_truth)

        for planner_cls in (DeterministicGreedyM1Planner, DeterministicRecallGotoXyzPlanner):
            for task in reloc_tasks:
                res_full = evaluate_single_task_offline(
                    g, task, planner_cls(), memory_mode="hierarchical_6zone"
                )
                self.assertEqual(res_full["sr"], 1.0, msg=f"{planner_cls.__name__} {task.task_id}")
                self.assertEqual(res_full["stale_flip_errors"], 0)
                self.assertGreaterEqual(res_full["diagnostics"]["objects_stale"], 1)

                res_no_stale = evaluate_single_task_offline(
                    g, task, planner_cls(), memory_mode="no_stale_gate"
                )
                self.assertEqual(res_no_stale["sr"], 0.0, msg=f"{planner_cls.__name__} {task.task_id}")
                self.assertGreaterEqual(res_no_stale["stale_flip_errors"], 1)

    def test_deterministic_baselines_on_all_30_tasks(self) -> None:
        g = _synthetic_two_floor_graph()
        suite = build_30_static_tasks(g)
        self.assertEqual(len(suite), 30)
        self.assertEqual(sum(1 for t in suite if t.tier == "tier1_single_floor"), 10)
        self.assertEqual(sum(1 for t in suite if t.tier == "tier2_memory_governance"), 10)
        self.assertEqual(sum(1 for t in suite if t.tier == "tier3_cross_floor_adv"), 10)

        for planner_cls in (DeterministicGreedyM1Planner, DeterministicRecallGotoXyzPlanner):
            rows = [
                evaluate_single_task_offline(g, t, planner_cls(), memory_mode="hierarchical_6zone")
                for t in suite
            ]
            agg = aggregate_suite_results(rows)
            self.assertEqual(agg["overall"]["sr"], 1.0, msg=planner_cls.__name__)
            self.assertEqual(agg["overall"]["no_fallback_sr"], 1.0, msg=planner_cls.__name__)
            self.assertEqual(agg["overall"]["stale_flip_errors"], 0, msg=planner_cls.__name__)

            t1_steps = [r["steps_used"] for r in rows if r["tier"] == "tier1_single_floor"]
            undisc_steps = [
                r["steps_used"]
                for r in rows
                if r["subtype"] in ("undiscovered_search", "relocated_unobserved")
            ]
            # Known prior tasks complete noticeably faster than undiscovered/relocated exploration tasks
            self.assertLess(sum(t1_steps) / len(t1_steps), sum(undisc_steps) / len(undisc_steps))

        # Under no_m_candidates, Recall+GotoXyz uses query_memory + goto_xyz while GreedyM1 cannot use M1
        t1_03 = next(t for t in suite if t.task_id == "T1_03")
        res_recall_nom = evaluate_single_task_offline(
            g, t1_03, DeterministicRecallGotoXyzPlanner(), memory_mode="no_m_candidates"
        )
        self.assertEqual(res_recall_nom["sr"], 1.0)
        self.assertGreaterEqual(res_recall_nom["memory_recall_calls"], 1)

    def test_independent_oracle_cursor_without_stop_claim(self) -> None:
        g = _synthetic_two_floor_graph()
        suite = build_30_static_tasks(g)
        chain_task = next(t for t in suite if t.task_id == "T3_05")
        seed_map = {s.object_id: s for s in chain_task.seed_objects}
        gt1_pose = g.keyframe_poses[seed_map[chain_task.target_object_ids[0]].keyframe_id]
        gt2_pose = g.keyframe_poses[seed_map[chain_task.target_object_ids[1]].keyframe_id]
        gt1_xyz = tuple(
            float(v)
            for v in (
                gt1_pose[:3, :3] @ np.asarray(seed_map[chain_task.target_object_ids[0]].local_xyz)
                + gt1_pose[:3, 3]
            )
        )
        gt2_xyz = tuple(
            float(v)
            for v in (
                gt2_pose[:3, :3] @ np.asarray(seed_map[chain_task.target_object_ids[1]].local_xyz)
                + gt2_pose[:3, 3]
            )
        )

        def _visit_without_stop_fn(**kwargs) -> TaskDecision:
            step = kwargs["step_index"]
            if step == 1:
                return TaskDecision(action="goto_xyz", xyz=gt1_xyz, latency_ms=1.0)
            if step == 2:
                return TaskDecision(action="goto_xyz", xyz=gt2_xyz, latency_ms=1.0)
            return TaskDecision(action="scan", latency_ms=1.0)

        res = evaluate_single_task_offline(
            g, chain_task, _ScriptedPlanner(_visit_without_stop_fn), memory_mode="hierarchical_6zone"
        )
        self.assertEqual(res["osr"], 1.0)
        self.assertEqual(res["subgoal_osr"], 1.0)
        self.assertEqual(res["sr"], 0.0)

    def test_fallback_noop_label_mismatch_and_ablation_single_variable(self) -> None:
        g = _synthetic_two_floor_graph()
        suite = build_30_static_tasks(g)
        t1_01 = next(t for t in suite if t.task_id == "T1_01")

        # 1. Fallback must NOT secretly move to M1
        fb_planner = _ScriptedPlanner(
            lambda **kw: TaskDecision(
                action="goto_candidate", target_id="M1", fallback=True, reason="timeout", latency_ms=5.0
            )
        )
        res_fb = evaluate_single_task_offline(g, t1_01, fb_planner, memory_mode="hierarchical_6zone")
        self.assertEqual(res_fb["sr"], 0.0)
        self.assertEqual(res_fb["no_fallback_sr"], 0.0)
        self.assertEqual(res_fb["traveled_m"], 0.0)
        self.assertEqual(res_fb["fallback_count"], t1_01.max_steps)
        self.assertTrue(all(tr["outcome"] == "fallback_noop" for tr in res_fb["step_traces"]))

        # 2. Mismatched target_label on stop_claim must be rejected
        mismatch_planner = _ScriptedPlanner(
            lambda **kw: TaskDecision(action="stop_claim", target_label="vending machine", latency_ms=2.0)
        )
        res_mm = evaluate_single_task_offline(g, t1_01, mismatch_planner, memory_mode="hierarchical_6zone")
        self.assertEqual(res_mm["sr"], 0.0)
        self.assertTrue(
            all(tr["outcome"] == "stop_rejected:target_label_mismatch" for tr in res_mm["step_traces"])
        )

        # 3. Single-variable ablation checks: no_m_candidates preserves full prose; no_blacklist_gate keeps W*
        t2_08 = next(t for t in suite if t.subtype == "blacklist_avoidance")
        seen_nom: list[tuple[str, list[str]]] = []
        seen_nobl: list[list[str]] = []

        def _inspect_nom(**kw) -> TaskDecision:
            seen_nom.append((kw["memory_summary"], [c["id"] for c in kw["candidates"]]))
            return TaskDecision(action="scan", latency_ms=1.0)

        def _inspect_nobl(**kw) -> TaskDecision:
            seen_nobl.append([c["id"] for c in kw["candidates"]])
            return TaskDecision(action="scan", latency_ms=1.0)

        evaluate_single_task_offline(g, t1_01, _ScriptedPlanner(_inspect_nom), memory_mode="no_m_candidates")
        self.assertIn("当前空间记忆版本", seen_nom[0][0])
        self.assertFalse(any(cid.startswith("M") for cid in seen_nom[0][1]))

        evaluate_single_task_offline(g, t2_08, _ScriptedPlanner(_inspect_nobl), memory_mode="no_blacklist_gate")
        self.assertTrue(any(cid.startswith("W") for cid in seen_nobl[0]))

    def test_invalid_mode_rejected_and_separated_percentiles(self) -> None:
        g = _synthetic_two_floor_graph()
        suite = build_30_static_tasks(g)
        with self.assertRaises(ValueError):
            evaluate_single_task_offline(
                g, suite[0], DeterministicGreedyM1Planner(), memory_mode="typo_mode"
            )

        eval_mod = _load_eval_script_module()
        with self.assertRaises(SystemExit):
            eval_mod.parse_args(
                ["--map-dir", "dummy_map", "--output", "out.json", "--modes", "hierarchical_6zone", "typo"]
            )
        parsed = eval_mod.parse_args(["--map-dir", "dummy_map", "--output", "out.json"])
        self.assertEqual(tuple(parsed.modes), VALID_MEMORY_MODES)
        self.assertTrue(parsed.include_deterministic_baselines)

        parsed_no_base = eval_mod.parse_args(
            ["--map-dir", "dummy_map", "--output", "out.json", "--no-deterministic-baselines"]
        )
        self.assertFalse(parsed_no_base.include_deterministic_baselines)

        # Verify separated latency percentiles and aggregate fields
        r1 = evaluate_single_task_offline(
            g, suite[0], DeterministicGreedyM1Planner(), memory_mode="hierarchical_6zone"
        )
        r2 = evaluate_single_task_offline(
            g, suite[10], DeterministicRecallGotoXyzPlanner(), memory_mode="hierarchical_6zone"
        )
        self.assertIn("request_latencies_ms", r1)
        self.assertIn("step_traces", r1)
        agg = aggregate_suite_results([r1, r2])
        overall = agg["overall"]
        for key in (
            "no_fallback_sr",
            "avg_subgoal_osr",
            "total_unreachable_moves",
            "total_fallbacks",
            "per_request_latency_p50_ms",
            "per_request_latency_p95_ms",
            "per_task_mean_latency_p50_ms",
            "per_task_mean_latency_p95_ms",
        ):
            self.assertIn(key, overall)


if __name__ == "__main__":
    unittest.main()
