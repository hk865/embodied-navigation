"""Unit tests for the 30-task static memory & navigation benchmark suite."""

from __future__ import annotations

import unittest
import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    KeyframeSceneGraph,
)
from embodied_navigation.planning.exploration.llm_agent_client import LlmClientConfig
from embodied_navigation.planning.exploration.static_task_suite import (
    aggregate_suite_results,
    build_default_30_task_suite,
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
    # Floor 0: 4 rooms separated by narrow doorways
    for r_idx in range(4):
        x0 = r_idx * 6.0
        for k in range(4):
            g.add_keyframe(f"f0_r{r_idx}_k{k}", _pose(x0 + k * 0.8, 0.0, 0.0), clearance_m=1.8)
        if r_idx < 3:
            g.add_keyframe(f"f0_door_{r_idx}", _pose(x0 + 4.2, 0.0, 0.0), clearance_m=0.45)
    # Stairs up to Floor 1
    for s in range(4):
        g.add_keyframe(f"stair_{s}", _pose(18.0, 1.0 + s, 0.65 * (s + 1)), clearance_m=1.0)
    # Floor 1: 6 rooms
    for r_idx in range(6):
        x0 = r_idx * 6.0
        for k in range(4):
            g.add_keyframe(f"f1_r{r_idx}_k{k}", _pose(x0 + k * 0.8, 6.0, 3.2), clearance_m=1.8)
        if r_idx < 5:
            g.add_keyframe(f"f1_door_{r_idx}", _pose(x0 + 4.2, 6.0, 3.2), clearance_m=0.45)
    g.segment_floors()
    g.partition_rooms()
    return g


class _GreedyMemoryPlanner(OpenCodeTaskPlanner):
    """Deterministic test planner: selects M1 if present and close else moves to M1 or stops."""

    def __init__(self) -> None:
        super().__init__(LlmClientConfig(provider="opencode", api_key="sk-test", supports_vision=False))

    def decide(self, **kwargs) -> TaskDecision:
        cands = kwargs.get("candidates") or []
        label = kwargs.get("target_label")
        for c in cands:
            if c.get("id") == "M1":
                if float(c.get("dist_m", 99.0)) <= float(kwargs.get("stop_radius_m", 1.8)):
                    return TaskDecision(action="stop_claim", target_label=label, latency_ms=10.0)
                return TaskDecision(action="goto_candidate", target_id="M1", latency_ms=10.0)
        if cands:
            return TaskDecision(action="goto_candidate", target_id=cands[0]["id"], latency_ms=10.0)
        return TaskDecision(action="stop_claim", target_label=label, latency_ms=10.0)


class TestStaticTaskSuite(unittest.TestCase):
    def test_default_suite_has_30_balanced_tasks_and_runs_ablation(self) -> None:
        graph = _synthetic_two_floor_graph()
        suite = build_default_30_task_suite(graph)
        self.assertEqual(len(suite), 30)
        self.assertEqual(sum(1 for t in suite if t.tier == "tier1_single_floor"), 10)
        self.assertEqual(sum(1 for t in suite if t.tier == "tier2_multi_target"), 10)
        self.assertEqual(sum(1 for t in suite if t.tier == "tier3_cross_floor_adv"), 10)

        planner = _GreedyMemoryPlanner()
        res_full = [evaluate_single_task_offline(graph, t, planner, memory_mode="hierarchical_6zone") for t in suite]
        res_stale = [evaluate_single_task_offline(graph, t, planner, memory_mode="stale_unfiltered") for t in suite]

        agg_full = aggregate_suite_results(res_full)
        agg_stale = aggregate_suite_results(res_stale)

        # Full hierarchical memory with staleness governance achieves 100% on deterministic oracle-greedy planner
        self.assertEqual(agg_full["overall"]["sr"], 1.0)
        self.assertEqual(agg_full["overall"]["stale_flip_errors"], 0)
        # Unfiltered stale baseline MUST fail on the 3 moved-object stale tasks!
        self.assertGreaterEqual(agg_stale["overall"]["stale_flip_errors"], 1)
        self.assertLess(agg_stale["overall"]["sr"], agg_full["overall"]["sr"])


if __name__ == "__main__":
    unittest.main()
