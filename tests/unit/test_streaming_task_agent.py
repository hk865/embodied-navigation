"""Unit tests for StaticStreamingTaskRunner and OpenCodeTaskPlanner."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

from embodied_navigation.localization_mapping.semantic_mapping import (
    HierarchicalMemoryStore,
    KeyframeSceneGraph,
)
from embodied_navigation.planning.exploration import waypoint_mailbox
from embodied_navigation.planning.exploration.llm_agent_client import LlmClientConfig
from embodied_navigation.planning.exploration.streaming_task_agent import (
    OpenCodeTaskPlanner,
    StaticStreamingTaskRunner,
    TaskDecision,
    parse_task_decision,
    select_active_chart_and_candidates,
)


class _ScriptedPlanner(OpenCodeTaskPlanner):
    def __init__(self, decisions: list[TaskDecision]) -> None:
        super().__init__(LlmClientConfig(provider="opencode", api_key="sk-unit-test"))
        self._decisions = list(decisions)
        self.calls: list[dict] = []

    def decide(self, **kwargs) -> TaskDecision:
        self.calls.append(kwargs)
        self._latencies_ms.append(42.0)
        if self._decisions:
            return self._decisions.pop(0)
        return TaskDecision(action="scan", reason="exhausted")


class TestStreamingTaskAgent(unittest.TestCase):
    def test_parse_task_decision_valid_and_fallback(self) -> None:
        raw = (
            '{"action":"goto_candidate","target_id":"F2",'
            '"observed_objects":[{"label":"desk","dist_m":2.4,"bearing_deg":-15,"confidence":0.9}],'
            '"reason":"head to open doorway"}'
        )
        dec = parse_task_decision(raw, valid_candidate_ids=["F1", "F2"], latency_ms=120.5)
        self.assertEqual(dec.action, "goto_candidate")
        self.assertEqual(dec.target_id, "F2")
        self.assertFalse(dec.fallback)
        self.assertEqual(len(dec.observed_objects), 1)
        self.assertEqual(dec.observed_objects[0]["label"], "desk")

        bad = parse_task_decision("not json at all", valid_candidate_ids=["F1"])
        self.assertTrue(bad.fallback)
        self.assertEqual(bad.action, "goto_candidate")
        self.assertEqual(bad.target_id, "F1")

        # Partial stream with complete inner object must NOT early-match the inner object
        from embodied_navigation.planning.exploration.streaming_task_agent import (
            _extract_outer_json_object,
        )

        partial = '{"action":"goto_candidate","observed_objects":[{"label":"door","dist_m":4.0}],'
        self.assertIsNone(_extract_outer_json_object(partial))
        complete = partial + '"target_id":"F1","reason":"ok"}'
        parsed_outer = _extract_outer_json_object(complete)
        self.assertIsNotNone(parsed_outer)
        self.assertEqual(parsed_outer["action"], "goto_candidate")

    def test_select_active_chart_prefers_closest_z_with_candidates(self) -> None:
        snap = {
            "products": {
                "height_charts": [
                    {"level_key": "+0.00", "image": "/tmp/f0.png", "candidates": []},
                    {
                        "level_key": "+3.20",
                        "local_image": "/tmp/f1_local.png",
                        "candidates": [{"id": "F1", "xy": [1.0, 2.0], "z": 3.2}],
                    },
                ]
            }
        }
        img, cands = select_active_chart_and_candidates(snap, robot_z=3.1)
        self.assertEqual(img, "/tmp/f1_local.png")
        self.assertEqual(len(cands), 1)

    def test_closed_loop_runner_rejects_premature_stop_then_succeeds(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            mailbox = run_dir / "capture" / "interactive"
            mailbox.mkdir(parents=True, exist_ok=True)

            # Write initial observation at (0, 0, 0)
            def _write_obs(xyz):
                (mailbox / "observation.json").write_text(
                    json.dumps(
                        {
                            "run_id": "test-run",
                            "pose": {"position_xyz_m": xyz, "yaw_rad": 0.0},
                            "waypoints": {"phase": "idle", "current_epoch": 0},
                        }
                    ),
                    encoding="utf-8",
                )

            _write_obs([0.0, 0.0, 0.0])

            # Background fake robot tracker answering goto requests and advancing pose
            stop_flag = threading.Event()

            def _fake_tracker():
                req_path = mailbox / "request.json"
                rec_path = mailbox / "receipts.jsonl"
                seen_seqs: set[int] = set()
                while not stop_flag.is_set():
                    if req_path.is_file():
                        try:
                            req = json.loads(req_path.read_text(encoding="utf-8"))
                        except (OSError, ValueError):
                            time.sleep(0.05)
                            continue
                        seq = int(req.get("seq", 0))
                        if seq not in seen_seqs:
                            seen_seqs.add(seq)
                            act = req.get("action")
                            if act == "goto":
                                goal = req["goal_xyz_m"]
                                _write_obs(goal)
                                event = {
                                    "event": "waypoint_reached",
                                    "seq": seq,
                                    "run_id": req.get("run_id"),
                                    "status": "completed",
                                    "success": True,
                                }
                                with rec_path.open("a", encoding="utf-8") as fh:
                                    fh.write(json.dumps(event) + "\n")
                            elif act == "stop":
                                event = {
                                    "event": "stop_accepted",
                                    "seq": seq,
                                    "run_id": req.get("run_id"),
                                }
                                with rec_path.open("a", encoding="utf-8") as fh:
                                    fh.write(json.dumps(event) + "\n")
                    time.sleep(0.05)

            tracker_thread = threading.Thread(target=_fake_tracker, daemon=True)
            tracker_thread.start()

            try:
                planner = _ScriptedPlanner(
                    [
                        # Step 1: premature STOP before reaching target -> rejected by gate
                        TaskDecision(
                            action="stop_claim",
                            target_label="whiteboard",
                            reason="premature guess",
                        ),
                        # Step 2: move to candidate F1 and spot whiteboard 1.0 m ahead
                        TaskDecision(
                            action="goto_candidate",
                            target_id="F1",
                            observed_objects=[
                                {
                                    "label": "whiteboard",
                                    "dist_m": 1.0,
                                    "bearing_deg": 0.0,
                                    "confidence": 0.95,
                                }
                            ],
                            reason="move closer",
                        ),
                        # Step 3: now at (2.0, 0.0, 0.0), whiteboard was logged at (1.0, 0.0, 0.0) -> 1.0 m <= 1.5 m -> verified!
                        TaskDecision(
                            action="stop_claim",
                            target_label="whiteboard",
                            reason="reached whiteboard",
                        ),
                    ]
                )

                def _fake_snap(tag: str) -> dict:
                    return {
                        "tag": tag,
                        "products": {
                            "rgb": None,
                            "height_charts": [
                                {
                                    "level_key": "+0.00",
                                    "candidates": [
                                        {
                                            "id": "F1",
                                            "kind": "frontier",
                                            "xy": [2.0, 0.0],
                                            "z": 0.0,
                                            "clearance_m": 1.5,
                                            "dist_m": 2.0,
                                            "turn_safe": True,
                                        }
                                    ],
                                }
                            ],
                        },
                    }

                runner = StaticStreamingTaskRunner(
                    run_dir,
                    task_instruction="Find the whiteboard.",
                    target_label="whiteboard",
                    planner=planner,
                    memory_store=HierarchicalMemoryStore(KeyframeSceneGraph()),
                    pose_source="backend_corrected",
                    snapshot_fn=_fake_snap,
                    step_timeout_s=10.0,
                )
                report = runner.run(max_steps=5)
                self.assertTrue(report["task_succeeded"])
                self.assertEqual(report["terminal_status"], "verified_stop_success")
                self.assertEqual(report["steps_executed"], 3)
                # Premature STOP on step 1 was caught and rejected!
                self.assertEqual(report["memory_diagnostics"]["premature_stop_rejects"], 1)
                self.assertTrue((run_dir / "static_task_report.json").is_file())
                self.assertTrue((run_dir / "hierarchical_memory.json").is_file())
            finally:
                stop_flag.set()
                tracker_thread.join(timeout=2.0)

    def test_timeout_records_blacklist_and_cancels_cleanly(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            mailbox = run_dir / "capture" / "interactive"
            mailbox.mkdir(parents=True, exist_ok=True)
            (mailbox / "observation.json").write_text(
                json.dumps(
                    {
                        "run_id": "timeout-run",
                        "pose": {"position_xyz_m": [0.0, 0.0, 0.0], "yaw_rad": 0.0},
                        "waypoints": {"phase": "idle", "current_epoch": 0},
                    }
                ),
                encoding="utf-8",
            )
            planner = _ScriptedPlanner(
                [TaskDecision(action="goto_candidate", target_id="F1", reason="will timeout")]
            )

            def _snap(tag: str) -> dict:
                return {
                    "tag": tag,
                    "products": {
                        "rgb": None,
                        "height_charts": [
                            {
                                "level_key": "+0.00",
                                "candidates": [{"id": "F1", "xy": [3.0, 0.0], "z": 0.0}],
                            }
                        ],
                    },
                }

            runner = StaticStreamingTaskRunner(
                run_dir,
                task_instruction="Timeout test",
                planner=planner,
                memory_store=HierarchicalMemoryStore(KeyframeSceneGraph()),
                pose_source="backend_corrected",
                snapshot_fn=_snap,
                step_timeout_s=0.35,
            )
            report = runner.run(max_steps=1)
            self.assertEqual(report["steps"][0]["execution"]["status"], "timeout")
            self.assertEqual(report["memory_diagnostics"]["active_blacklist_count"], 1)

    def test_scan_and_maneuver_wait_for_action_signatures(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            mailbox = run_dir / "capture" / "interactive"
            mailbox.mkdir(parents=True, exist_ok=True)
            (mailbox / "observation.json").write_text(
                json.dumps(
                    {
                        "run_id": "action-run",
                        "pose": {"position_xyz_m": [0.0, 0.0, 0.0], "yaw_rad": 0.0},
                        "waypoints": {"phase": "idle", "current_epoch": 0},
                    }
                ),
                encoding="utf-8",
            )
            stop_flag = threading.Event()

            def _responder():
                req_path = mailbox / "request.json"
                rec_path = mailbox / "receipts.jsonl"
                seen: set[int] = set()
                while not stop_flag.is_set():
                    if req_path.is_file():
                        try:
                            req = json.loads(req_path.read_text(encoding="utf-8"))
                        except (OSError, ValueError):
                            time.sleep(0.03)
                            continue
                        seq = int(req.get("seq", 0))
                        if seq not in seen:
                            seen.add(seq)
                            act = req.get("action")
                            ev = (
                                "yaw_complete"
                                if act == "yaw"
                                else ("maneuver_complete" if act == "maneuver" else "stop_accepted")
                            )
                            with rec_path.open("a", encoding="utf-8") as fh:
                                fh.write(
                                    json.dumps(
                                        {"event": ev, "seq": seq, "run_id": req.get("run_id")}
                                    )
                                    + "\n"
                                )
                    time.sleep(0.03)

            t = threading.Thread(target=_responder, daemon=True)
            t.start()
            try:
                planner = _ScriptedPlanner(
                    [
                        TaskDecision(action="scan", reason="look around"),
                        TaskDecision(action="maneuver", forward=0.3, yaw_rate=0.0, duration=1.0),
                    ]
                )
                runner = StaticStreamingTaskRunner(
                    run_dir,
                    task_instruction="Scan and maneuver test",
                    planner=planner,
                    memory_store=HierarchicalMemoryStore(KeyframeSceneGraph()),
                    pose_source="backend_corrected",
                    snapshot_fn=lambda tag: {"tag": tag, "products": {"rgb": None, "height_charts": []}},
                    step_timeout_s=5.0,
                )
                report = runner.run(max_steps=2)
                self.assertEqual(report["steps"][0]["execution"]["event"], "yaw_complete")
                self.assertEqual(report["steps"][1]["execution"]["event"], "maneuver_complete")
            finally:
                stop_flag.set()
                t.join(timeout=2.0)

    def test_six_zone_context_prose_memory_and_global_memory_candidates(self) -> None:
        import numpy as np

        graph = KeyframeSceneGraph()
        pose = np.eye(4, dtype=np.float64)
        graph.add_keyframe("kf_0", pose, clearance_m=1.5)
        store = HierarchicalMemoryStore(graph)
        store.add_object("obj_door", "red double door", "kf_0", [5.5, -0.5, 0.0], confidence=0.95)

        # Even from 15 m away, global memory_goal_candidates still surfaces M1 with 1.0 m standoff
        m_cands = store.memory_goal_candidates([15.5, -0.5, 0.0], target_label="door", standoff_m=1.0)
        self.assertEqual(len(m_cands), 1)
        self.assertEqual(m_cands[0]["id"], "M1")
        self.assertEqual(m_cands[0]["kind"], "memory_target")
        self.assertAlmostEqual(m_cands[0]["xy"][0], 6.5, places=1)

        prose = store.narrative_summary(robot_xyz=[15.5, -0.5, 0.0], target_label="door")
        self.assertIn("red double door", prose)
        self.assertIn("query_memory", prose)

        planner = OpenCodeTaskPlanner(
            LlmClientConfig(provider="opencode", api_key="sk-test", supports_vision=False)
        )
        msgs = planner.build_messages(
            task_instruction="Reach the red double door.",
            step_index=3,
            robot_xyz=[15.5, -0.5, 0.0],
            yaw_rad=0.0,
            candidates=m_cands,
            memory_summary=prose,
            target_label="door",
            last_feedback="goto completed",
            tool_recall_result={"query": "door", "l2_objects": [{"object_id": "obj_door"}]},
        )
        sys_txt = msgs[0]["content"]
        usr_txt = msgs[1]["content"]
        self.assertIn("## [SYSTEM]", sys_txt)
        self.assertIn("## [SKILL]", sys_txt)
        self.assertIn("## [TASK]", usr_txt)
        self.assertIn("## [MEMORY_SUMMARY]", usr_txt)
        self.assertIn("## [INTERMEDIATE_STATE]", usr_txt)
        self.assertIn("## [TOOL_RESULTS]", usr_txt)
        self.assertIn("memory_tool_recall", usr_txt)


if __name__ == "__main__":
    unittest.main()
