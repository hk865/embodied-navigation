"""Regression tests covering Group 2 (Memory & Streaming Runner) Astra audit findings."""

from __future__ import annotations

import http.server
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    HierarchicalMemoryStore,
    KeyframeSceneGraph,
)
from embodied_navigation.localization_mapping.semantic_mapping.hierarchical_memory import (
    _label_match_score,
    _label_strict_match,
)
from embodied_navigation.planning.exploration import waypoint_mailbox
from embodied_navigation.planning.exploration.llm_agent_client import (
    LlmClientConfig,
    SentinelVerdict,
)
from embodied_navigation.planning.exploration.streaming_task_agent import (
    OpenCodeTaskPlanner,
    StaticStreamingTaskRunner,
    TaskDecision,
    _extract_robot_pose,
    parse_task_decision,
)


def _pose(x: float, y: float, z: float, yaw: float = 0.0) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    m = np.eye(4, dtype=np.float64)
    m[0, 0], m[0, 1] = c, -s
    m[1, 0], m[1, 1] = s, c
    m[:3, 3] = [x, y, z]
    return m


class _ScriptedPlanner(OpenCodeTaskPlanner):
    def __init__(self, decisions: list[TaskDecision], **kwargs) -> None:
        super().__init__(
            LlmClientConfig(provider="opencode", api_key="sk-group2-test"), **kwargs
        )
        self._decisions = list(decisions)
        self.calls: list[dict] = []

    def decide(self, **kwargs) -> TaskDecision:
        self.calls.append(kwargs)
        self._latencies_ms.append(15.0)
        if self._decisions:
            return self._decisions.pop(0)
        return TaskDecision(action="noop", reason="exhausted")


class TestHierarchicalMemoryAuditFixes(unittest.TestCase):
    def test_word_boundary_matching_rejects_cardboard_for_board(self) -> None:
        self.assertEqual(_label_match_score("board", "cardboard"), 0.0)
        self.assertFalse(_label_strict_match("board", "cardboard"))
        self.assertGreater(_label_match_score("board", "white board"), 0.0)
        self.assertTrue(_label_strict_match("board", "white board"))

        graph = KeyframeSceneGraph()
        graph.add_keyframe("kf0", _pose(0.0, 0.0, 0.0), clearance_m=1.5)
        store = HierarchicalMemoryStore(graph)
        store.add_object("obj_box", "cardboard box", "kf0", [0.3, 0.0, 0.0])

        res = store.verify_stop_claim([0.0, 0.0, 0.0], target_label="board", stop_radius_m=1.5)
        self.assertFalse(res.allowed)
        self.assertEqual(res.reason, "no_matching_target_in_memory")

    def test_fresh_and_stale_colocated_prefers_fresh_object(self) -> None:
        graph = KeyframeSceneGraph()
        graph.add_keyframe("kf0", _pose(0.0, 0.0, 0.0), clearance_m=1.5)
        store = HierarchicalMemoryStore(graph)
        store.add_object("door_old", "red double door", "kf0", [0.2, 0.0, 0.0])
        store.mark_object_stale("door_old", reason="moved")
        store.add_object("door_new", "red double door", "kf0", [0.4, 0.0, 0.0])

        res = store.verify_stop_claim(
            [0.0, 0.0, 0.0], target_label="red double door", stop_radius_m=1.5
        )
        self.assertTrue(res.allowed)
        self.assertEqual(res.matched_object_id, "door_new")
        self.assertFalse(res.evidence_stale)

    def test_object_request_never_falls_back_to_room_occupancy(self) -> None:
        graph = KeyframeSceneGraph()
        for i in range(4):
            graph.add_keyframe(f"kf_{i}", _pose(float(i), 0.0, 0.0), clearance_m=1.5)
        graph.segment_floors()
        graph.partition_rooms()
        room_id = next(iter(graph.rooms))
        store = HierarchicalMemoryStore(graph)

        # Requesting a non-existent object in an occupied room must fail!
        res_obj = store.verify_stop_claim(
            [1.0, 0.0, 0.0], target_label="missing printer", target_room_id=room_id
        )
        self.assertFalse(res_obj.allowed)
        self.assertEqual(res_obj.reason, "no_matching_target_in_memory")

        # Pure room task (no target_label/target_object_id) succeeds
        res_room = store.verify_stop_claim([1.0, 0.0, 0.0], target_room_id=room_id)
        self.assertTrue(res_room.allowed)
        self.assertEqual(res_room.reason, "verified_room_occupancy")

    def test_se3_pure_rotation_and_cumulative_drift_invalidation_and_idempotency(self) -> None:
        graph = KeyframeSceneGraph()
        graph.add_keyframe("kf0", _pose(0.0, 0.0, 0.0, yaw=0.0), clearance_m=1.5)
        store = HierarchicalMemoryStore(graph)
        store.record_attempt("goto", [3.0, 0.0, 0.0], "stuck", keyframe_id="kf0")
        self.assertEqual(store.diagnostic_metrics()["active_blacklist_count"], 1)

        # Pure 180-degree rotation at same translation -> goal shifts 6.0 m -> must invalidate!
        rep_rot = store.sync_poses({"kf0": _pose(0.0, 0.0, 0.0, yaw=math.pi)}, failure_invalidation_shift_m=0.25)
        self.assertTrue(rep_rot["changed"])
        self.assertEqual(rep_rot["invalidated_failures"], 1)
        self.assertEqual(store.diagnostic_metrics()["active_blacklist_count"], 0)

        # Repeated identical sync is idempotent (map_revision unchanged)
        rev_before = store.map_revision
        rep_same = store.sync_poses({"kf0": _pose(0.0, 0.0, 0.0, yaw=math.pi)})
        self.assertFalse(rep_same["changed"])
        self.assertEqual(store.map_revision, rev_before)

        # Cumulative small translations (3 x 0.10 m = 0.30 m >= 0.25 m) must invalidate
        store.record_attempt("goto", [1.0, 0.0, 0.0], "stuck", keyframe_id="kf0")
        self.assertEqual(store.diagnostic_metrics()["active_blacklist_count"], 1)
        store.sync_poses({"kf0": _pose(0.10, 0.0, 0.0, yaw=math.pi)}, failure_invalidation_shift_m=0.25)
        self.assertEqual(store.diagnostic_metrics()["active_blacklist_count"], 1)
        store.sync_poses({"kf0": _pose(0.20, 0.0, 0.0, yaw=math.pi)}, failure_invalidation_shift_m=0.25)
        self.assertEqual(store.diagnostic_metrics()["active_blacklist_count"], 1)
        rep_cum = store.sync_poses({"kf0": _pose(0.30, 0.0, 0.0, yaw=math.pi)}, failure_invalidation_shift_m=0.25)
        self.assertEqual(rep_cum["invalidated_failures"], 1)
        self.assertEqual(store.diagnostic_metrics()["active_blacklist_count"], 0)

    def test_non_geographic_failures_never_pollute_blacklist(self) -> None:
        graph = KeyframeSceneGraph()
        graph.add_keyframe("kf0", _pose(0.0, 0.0, 0.0), clearance_m=1.5)
        store = HierarchicalMemoryStore(graph)
        store.record_attempt("stop_claim", [1.0, 0.0, 0.0], "rejected", reason="target_beyond_stop_radius")
        store.record_attempt("goto", [1.0, 0.0, 0.0], "cancelled", reason="operator_cancel")
        store.record_attempt("goto", [1.0, 0.0, 0.0], "rejected", reason="stale_action_epoch")

        self.assertEqual(store.diagnostic_metrics()["active_blacklist_count"], 0)
        self.assertEqual(store.match_blacklist([1.0, 0.0, 0.0]), [])

    def test_memory_goal_candidates_unknown_clearance_and_aligned_summary(self) -> None:
        graph = KeyframeSceneGraph()
        graph.add_keyframe("kf0", _pose(0.0, 0.0, 0.1), clearance_m=1.5)
        store = HierarchicalMemoryStore(graph)
        # Object center is elevated (wall-mounted at Z=1.8 m), support Z must come from anchor KF (0.1 m)
        store.add_object("obj_sign", "exit sign", "kf0", [4.0, 0.0, 1.7])
        cands = store.memory_goal_candidates([0.0, 0.0, 0.1], target_label="exit sign")
        self.assertEqual(len(cands), 1)
        self.assertAlmostEqual(cands[0]["z"], 0.1, places=2)
        self.assertIsNone(cands[0]["clearance_m"])
        self.assertFalse(cands[0]["clearance_known"])
        self.assertIsNone(cands[0]["turn_safe"])
        prose = store.narrative_summary(robot_xyz=[0.0, 0.0, 0.1], target_label="exit sign", memory_candidates=cands)
        self.assertIn("M1:", prose)

    def test_atomic_save_preserves_existing_file_on_write_failure(self) -> None:
        graph = KeyframeSceneGraph()
        graph.add_keyframe("kf0", _pose(0.0, 0.0, 0.0), clearance_m=1.5)
        store = HierarchicalMemoryStore(graph)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "memory.json"
            store.save(path)
            before = path.read_text(encoding="utf-8")
            with mock.patch(
                "embodied_navigation.localization_mapping.semantic_mapping.hierarchical_memory.HierarchicalMemoryStore.to_dict",
                side_effect=RuntimeError("boom"),
            ):
                with self.assertRaises(RuntimeError):
                    store.save(path)
            self.assertEqual(path.read_text(encoding="utf-8"), before)


class TestStreamingRunnerAuditFixes(unittest.TestCase):
    def _setup_mailbox(self, run_dir: Path, xyz=(0.0, 0.0, 0.0), phase="idle", fresh=True, epoch=0) -> Path:
        mb = run_dir / "capture" / "interactive"
        mb.mkdir(parents=True, exist_ok=True)
        run_id = waypoint_mailbox.run_id_for(mb)
        (mb / "observation.json").write_text(
            json.dumps(
                {
                    "run_id": run_id,
                    "pose": {"position_xyz_m": list(xyz), "yaw_rad": 0.0, "fresh": fresh},
                    "waypoints": {"phase": phase, "current_epoch": epoch},
                }
            ),
            encoding="utf-8",
        )
        return mb

    def test_unverified_vlm_guess_and_target_override_rejected_on_stop(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            self._setup_mailbox(run_dir)
            graph = KeyframeSceneGraph()
            graph.add_keyframe("kf_prior", _pose(0.0, 0.0, 0.0), clearance_m=1.5)
            store = HierarchicalMemoryStore(graph)
            # A chair is right next to the robot in prior memory, but the task is for a red double door!
            store.add_object("obj_chair", "chair", "kf_prior", [0.3, 0.0, 0.0], producer="prior_map")

            planner = _ScriptedPlanner(
                [
                    # Step 1: Model tries to hijack STOP by claiming "chair" instead of "red double door"
                    TaskDecision(action="stop_claim", target_label="chair", reason="found chair"),
                    # Step 2: Model hallucinates seeing "red double door" at 0.5 m with NO LiDAR height chart
                    TaskDecision(
                        action="stop_claim",
                        target_label="red double door",
                        observed_objects=[
                            {"label": "red double door", "dist_m": 0.5, "bearing_deg": 0.0, "confidence": 0.99}
                        ],
                        reason="hallucinated door",
                    ),
                ]
            )
            runner = StaticStreamingTaskRunner(
                run_dir,
                task_instruction="Find the red double door.",
                target_label="red double door",
                planner=planner,
                memory_store=store,
                snapshot_fn=lambda tag: {"tag": tag, "products": {"rgb": None, "height_charts": []}},
                stop_simulator_on_exit=False,
            )
            report = runner.run(max_steps=2)
            self.assertFalse(report["task_succeeded"])
            self.assertEqual(report["steps"][0]["execution"]["reason"], "target_label_mismatch")
            self.assertEqual(report["steps"][1]["execution"]["reason"], "unverified_self_reported_evidence")

    def test_sentinel_stop_locks_main_loop_and_waits_for_receipt(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            mb = self._setup_mailbox(run_dir)
            rec_path = mb / "receipts.jsonl"

            class _FakeSentinelCoord:
                action_epoch = 1

                def __init__(self) -> None:
                    self.submitted_rgb: list[str | None] = []
                    self.call_order: list[str] = []
                    self._dispatched = False

                def submit_observation(self, **kwargs):
                    self.call_order.append("submit")
                    self.submitted_rgb.append(kwargs.get("rgb_path"))
                    return {"submitted": True}

                def poll_and_dispatch(self):
                    self.call_order.append("poll")
                    if not self._dispatched:
                        self._dispatched = True
                        # Simulate simulator immediately writing stop_accepted for seq=99
                        with rec_path.open("a", encoding="utf-8") as fh:
                            fh.write(json.dumps({"event": "stop_accepted", "seq": 99, "run_id": waypoint_mailbox.run_id_for(mb)}) + "\n")
                        return {
                            "dispatched": True,
                            "verdict": SentinelVerdict(act="STOP", reason="obstacle_ahead", latency_ms=12.0),
                            "payload": {"seq": 99, "action_epoch": 2},
                        }
                    return None

            planner = _ScriptedPlanner(
                [
                    TaskDecision(action="goto_candidate", target_id="F1", reason="go"),
                    TaskDecision(action="goto_candidate", target_id="F1", reason="should never run"),
                ]
            )
            runner = StaticStreamingTaskRunner(
                run_dir,
                task_instruction="Go forward",
                planner=planner,
                memory_store=HierarchicalMemoryStore(KeyframeSceneGraph()),
                snapshot_fn=lambda tag: {
                    "tag": tag,
                    "products": {"rgb": None, "height_charts": [{"level_key": "+0.0", "candidates": [{"id": "F1", "xy": [3.0, 0.0], "z": 0.0}]}]},
                },
                step_timeout_s=5.0,
                control_receipt_timeout_s=2.0,
                stop_simulator_on_exit=True,
            )
            fake_coord = _FakeSentinelCoord()
            runner._sentinel_coord = fake_coord  # type: ignore[assignment]

            report = runner.run(max_steps=3)
            self.assertEqual(report["terminal_status"], "sentinel_stop_locked")
            self.assertEqual(report["steps_executed"], 1)
            self.assertEqual(fake_coord.call_order[0], "poll")
            self.assertEqual(report["steps"][0]["execution"]["stop_receipt"]["event"], "stop_accepted")
            self.assertEqual(report["cleanup"]["reason"], "already_stopped_by_sentinel")

    def test_sentinel_preempt_tracks_new_seq_until_real_receipt(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            mb = self._setup_mailbox(run_dir)
            rec_path = mb / "receipts.jsonl"
            # Put a real live frame in interactive_frames to verify latest_sim_frame["latest"] wiring!
            frames_dir = mb.parent / "interactive_frames"
            frames_dir.mkdir(parents=True, exist_ok=True)
            live_rgb = frames_dir / "rgb_000001.png"
            live_rgb.write_bytes(b"\x89PNG\r\n\x1a\n" + b"\x00" * 16)

            class _PreemptCoord:
                action_epoch = 1

                def __init__(self) -> None:
                    self.seen_rgb: list[str | None] = []
                    self._emitted = False

                def submit_observation(self, **kwargs):
                    self.seen_rgb.append(kwargs.get("rgb_path"))
                    return {"submitted": True}

                def poll_and_dispatch(self):
                    if not self._emitted and self.seen_rgb:
                        self._emitted = True
                        # Write waypoint_failed for the new preempt seq=77 shortly after
                        def _emit_fail():
                            time.sleep(0.15)
                            with rec_path.open("a", encoding="utf-8") as fh:
                                fh.write(
                                    json.dumps(
                                        {"event": "waypoint_failed", "seq": 77, "run_id": waypoint_mailbox.run_id_for(mb), "reason": "blocked"}
                                    )
                                    + "\n"
                                )

                        threading.Thread(target=_emit_fail, daemon=True).start()
                        return {
                            "dispatched": True,
                            "verdict": SentinelVerdict(act="PREEMPT", target_id="F2", reason="reroute", latency_ms=9.0),
                            "payload": {"seq": 77, "action_epoch": 2},
                        }
                    return None

            planner = _ScriptedPlanner([TaskDecision(action="goto_candidate", target_id="F1", reason="go")])
            runner = StaticStreamingTaskRunner(
                run_dir,
                task_instruction="Preempt tracking test",
                planner=planner,
                memory_store=HierarchicalMemoryStore(KeyframeSceneGraph()),
                snapshot_fn=lambda tag: {
                    "tag": tag,
                    "products": {"rgb": None, "height_charts": [{"level_key": "+0.0", "candidates": [{"id": "F1", "xy": [3.0, 0.0], "z": 0.0}]}]},
                },
                step_timeout_s=5.0,
                sentinel_poll_interval_s=0.05,
                stop_simulator_on_exit=False,
            )
            coord = _PreemptCoord()
            runner._sentinel_coord = coord  # type: ignore[assignment]
            report = runner.run(max_steps=1)

            # Live RGB was actually passed to sentinel from "latest" key!
            self.assertEqual(coord.seen_rgb[0], str(live_rgb))
            # Execution waited for new seq=77's real waypoint_failed receipt (NOT instant success=True)!
            self.assertEqual(report["steps"][0]["execution"]["status"], "failed")
            self.assertFalse(report["steps"][0]["execution"]["success"])
            self.assertEqual(report["steps"][0]["execution"]["seq"], 77)

    def test_goto_xyz_respects_blacklist_and_invalid_pose_refuses_origin_fallback(self) -> None:
        self.assertIsNone(_extract_robot_pose(None))
        self.assertIsNone(_extract_robot_pose({"pose": {"position_xyz_m": [1, 2, 3], "fresh": False}}))
        self.assertIsNone(
            _extract_robot_pose({"pose": {"position_xyz_m": [1, 2, 3], "fresh": True}, "waypoints": {"phase": "terminal"}})
        )

        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)
            self._setup_mailbox(run_dir)
            store = HierarchicalMemoryStore(KeyframeSceneGraph())
            store.record_attempt("goto", [2.0, 2.0, 0.0], "stuck", reason="narrow")

            planner = _ScriptedPlanner([TaskDecision(action="goto_xyz", xyz=(2.0, 2.0, 0.0), reason="bypass")])
            runner = StaticStreamingTaskRunner(
                run_dir,
                task_instruction="Blacklist test",
                planner=planner,
                memory_store=store,
                snapshot_fn=lambda tag: {"tag": tag, "products": {"rgb": None, "height_charts": []}},
                stop_simulator_on_exit=False,
            )
            report = runner.run(max_steps=1)
            self.assertEqual(report["steps"][0]["execution"]["status"], "blacklisted_goal")

    def test_prior_memory_second_episode_preserves_old_anchors_and_rejects_external_l3(self) -> None:
        with tempfile.TemporaryDirectory() as tmp, tempfile.TemporaryDirectory() as external_dir:
            ext_file = Path(external_dir) / "secret.png"
            ext_file.write_bytes(b"\x89PNG\r\n\x1a\n" + b"\x00" * 16)

            run1 = Path(tmp) / "run1"
            self._setup_mailbox(run1, xyz=(1.0, 0.0, 0.0))
            store1 = HierarchicalMemoryStore(KeyframeSceneGraph())
            store1.graph.add_keyframe("kf_ext", _pose(5.0, 0.0, 0.0), rgb_path=str(ext_file))
            store1.add_object("obj_ext", "cabinet", "kf_ext", [0.0, 0.0, 0.0], producer="prior_map")

            p1 = _ScriptedPlanner(
                [
                    TaskDecision(
                        action="query_memory",
                        query="cabinet",
                        observed_objects=[{"label": "desk", "dist_m": 2.0, "bearing_deg": 0.0}],
                    )
                ]
            )
            r1 = StaticStreamingTaskRunner(
                run1,
                task_instruction="Episode 1",
                planner=p1,
                memory_store=store1,
                snapshot_fn=lambda tag: {"tag": tag, "products": {"rgb": None, "height_charts": []}},
                stop_simulator_on_exit=False,
                episode_id="ep1",
            )
            rep1 = r1.run(max_steps=1)
            # External L3 image outside run1 must be rejected!
            self.assertEqual(rep1["steps"][0]["execution"]["l3_images"], 0)
            self.assertEqual(rep1["steps"][0]["execution"]["l3_rejected"], 1)

            # Reload saved memory into Episode 2 -> old anchors/objects must NOT be overwritten
            reloaded = HierarchicalMemoryStore.load(run1 / "hierarchical_memory.json")
            old_kfs = set(reloaded.graph.keyframe_poses.keys())
            old_objs = set(reloaded.graph.objects.keys())
            run2 = Path(tmp) / "run2"
            self._setup_mailbox(run2, xyz=(2.0, 0.0, 0.0))
            p2 = _ScriptedPlanner(
                [TaskDecision(action="noop", observed_objects=[{"label": "chair", "dist_m": 1.5, "bearing_deg": 0.0}])]
            )
            r2 = StaticStreamingTaskRunner(
                run2,
                task_instruction="Episode 2",
                planner=p2,
                memory_store=reloaded,
                snapshot_fn=lambda tag: {"tag": tag, "products": {"rgb": None, "height_charts": []}},
                stop_simulator_on_exit=False,
                episode_id="ep2",
            )
            r2.run(max_steps=1)
            self.assertTrue(old_kfs.issubset(set(reloaded.graph.keyframe_poses.keys())))
            self.assertTrue(old_objs.issubset(set(reloaded.graph.objects.keys())))
            self.assertGreater(len(reloaded.graph.keyframe_poses), len(old_kfs))
            self.assertGreater(len(reloaded.graph.objects), len(old_objs))

    def test_http_error_body_never_leaks_into_decision_or_report(self) -> None:
        sentinel_secret = "SYNTHETIC_SECRET_BODY_DO_NOT_LEAK_9911"

        class _ErrHandler(http.server.BaseHTTPRequestHandler):
            def do_POST(self):
                self.send_response(500)
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(sentinel_secret.encode("utf-8"))

            def log_message(self, format, *args):
                return

        server = http.server.HTTPServer(("127.0.0.1", 0), _ErrHandler)
        port = server.server_address[1]
        th = threading.Thread(target=server.serve_forever, daemon=True)
        th.start()
        try:
            cfg = LlmClientConfig(
                provider="openai_compatible",
                base_url=f"http://127.0.0.1:{port}/v1",
                model="test-model",
                api_key="sk-local-only",
                timeout_s=2.0,
                supports_vision=False,
            )
            planner = OpenCodeTaskPlanner(cfg)
            dec = planner.decide(
                task_instruction="Test",
                step_index=1,
                robot_xyz=[0.0, 0.0, 0.0],
                yaw_rad=0.0,
                candidates=[],
            )
            self.assertTrue(dec.fallback)
            self.assertEqual(dec.action, "noop")
            self.assertIn("http_500", dec.reason)
            self.assertNotIn(sentinel_secret, dec.reason)
            self.assertNotIn(sentinel_secret, json.dumps(dec.to_dict()))
        finally:
            server.shutdown()
            server.server_close()


if __name__ == "__main__":
    unittest.main()
