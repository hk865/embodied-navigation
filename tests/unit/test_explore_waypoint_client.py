"""Offline tests for the operator-side exploration client (CPU only).

The client never touches Isaac or the GPU. These tests lock its protocol
behaviour (monotonic sequence, atomic request write, ground-truth refusal,
heartbeat clock) and the operator-side occupancy accumulator that turns live
depth plus measured odometry into observed-free, occupied and frontier cells.
"""

import contextlib
import io
import json
import math
import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

import explore_waypoint_client as client


class ClientProtocolTests(unittest.TestCase):
    """Mailbox protocol behaviour of the operator client."""

    def setUp(self):
        self._temporary = tempfile.TemporaryDirectory()
        self.run_dir = Path(self._temporary.name) / "run-20261110"
        self.mailbox = self.run_dir / "interactive"
        self.mailbox.mkdir(parents=True)

    def tearDown(self):
        self._temporary.cleanup()

    def test_next_sequence_starts_at_one_and_increments(self):
        self.assertEqual(1, client.next_sequence(self.mailbox))
        client.submit(
            self.mailbox,
            action="goto",
            goal=[1.0, 2.0, 3.0],
            pose_source="lio_estimate",
        )
        self.assertEqual(2, client.next_sequence(self.mailbox))

    def test_submit_writes_a_complete_request_atomically(self):
        payload = client.submit(
            self.mailbox,
            action="goto",
            goal=[1.0, 2.0, 3.0],
            ttl_s=12.0,
            pose_source="lio_estimate",
        )
        stored = json.loads((self.mailbox / "request.json").read_text(encoding="utf-8"))
        self.assertEqual(payload, stored)
        self.assertEqual(client.PROTOCOL_VERSION, stored["version"])
        self.assertEqual("run-20261110", stored["run_id"])
        self.assertEqual([1.0, 2.0, 3.0], stored["goal_xyz_m"])
        self.assertEqual(12.0, stored["ttl_s"])
        self.assertEqual("lio_estimate", stored["pose_source"])
        self.assertFalse((self.mailbox / "request.json.tmp").exists())

    def test_submit_refuses_a_ground_truth_waypoint_frame(self):
        with self.assertRaises(ValueError):
            client.submit(
                self.mailbox,
                action="goto",
                goal=[1.0, 0.0, 0.0],
                pose_source="native_truth",
            )
        self.assertFalse((self.mailbox / "request.json").exists())

    def test_stop_request_carries_no_goal(self):
        payload = client.submit(self.mailbox, action="stop")
        self.assertNotIn("goal_xyz_m", payload)
        self.assertEqual("stop", payload["action"])

    def test_invalid_ttl_is_refused_before_writing(self):
        with self.assertRaises(ValueError):
            client.submit(
                self.mailbox,
                action="goto",
                goal=[1.0, 0.0, 0.0],
                ttl_s=0.2,
                pose_source="lio_estimate",
            )
        self.assertFalse((self.mailbox / "request.json").exists())

    def test_heartbeat_uses_the_shared_monotonic_clock(self):
        first = client.write_heartbeat(self.mailbox)
        second = client.write_heartbeat(self.mailbox)
        self.assertGreaterEqual(second["wall_s"], first["wall_s"])
        self.assertEqual("run-20261110", second["run_id"])

    def test_tail_receipts_returns_the_last_entries(self):
        lines = [json.dumps({"event": "e" + str(index)}) for index in range(5)]
        (self.mailbox / "receipts.jsonl").write_text(
            "\n".join(lines) + "\n", encoding="utf-8"
        )
        self.assertEqual(
            [{"event": "e3"}, {"event": "e4"}], client.tail_receipts(self.mailbox, 2)
        )

    def test_observe_summary_reports_source_freshness_and_waypoint_state(self):
        observation = {
            "protocol_version": client.PROTOCOL_VERSION,
            "run_id": "run-20261110",
            "sim_s": 12.5,
            "pose": {
                "source": "lio_estimate",
                "map_frame": "map",
                "position_xyz_m": [1.0, 2.0, 3.0],
                "yaw_rad": 0.5,
                "sim_age_s": 0.01,
                "wall_age_s": 0.02,
                "fresh": True,
            },
            "speed": {
                "pose_window_mps": 0.83,
                "command_ceiling_mps": 1.0,
            },
            "waypoints": {
                "active_seq": 4,
                "active_goal_xyz_m": [2.0, 3.0, 3.0],
                "queued": 1,
                "phase": "cruise",
                "distance_to_goal_m": 1.4,
                "z_error_m": 0.0,
                "completed_goals": 3,
                "last_result": {"event": "waypoint_reached"},
                "terminal_reason": None,
            },
            "command": {"last_request": [0.9, 0.0, 0.1]},
            "link": {"established": True, "heartbeat_age_s": 0.4},
            "coverage": {"pose_samples": 120, "path_length_m": 9.5},
            "map": {"provider": "operator_side_from_live_measured_topics"},
            "guards": {"native": {"usable_for_navigation": False}},
        }
        (self.mailbox / "observation.json").write_text(
            json.dumps(observation), encoding="utf-8"
        )
        summary = client.summarize_observation(client.read_observation(self.mailbox))
        self.assertTrue(summary["available"])
        self.assertEqual("lio_estimate", summary["pose"]["source"])
        self.assertEqual("map", summary["pose"]["frame"])
        self.assertTrue(summary["pose"]["fresh"])
        self.assertEqual(0.83, summary["measured_speed_mps"])
        self.assertEqual(4, summary["waypoints"]["active_seq"])
        self.assertEqual("cruise", summary["waypoints"]["phase"])
        self.assertEqual(9.5, summary["coverage"]["path_length_m"])
        self.assertFalse(summary["native_guards"]["usable_for_navigation"])

    def test_observe_summary_without_bundle(self):
        self.assertEqual({"available": False}, client.summarize_observation(None))

    def test_cli_goto_round_trip(self):
        code = client.main(
            [
                "--mailbox",
                str(self.mailbox),
                "goto",
                "--x",
                "3.0",
                "--y",
                "-1.5",
                "--z",
                "-3.3",
                "--ttl",
                "20",
            ]
        )
        self.assertEqual(0, code)
        stored = json.loads((self.mailbox / "request.json").read_text(encoding="utf-8"))
        self.assertEqual([3.0, -1.5, -3.3], stored["goal_xyz_m"])
        self.assertEqual(20.0, stored["ttl_s"])
        self.assertEqual(1, stored["seq"])


class LocalActionCliTests(unittest.TestCase):
    """The scan/manoeuvre protocol must be reachable from the CLI."""

    def setUp(self):
        self._temporary = tempfile.TemporaryDirectory()
        self.run_dir = Path(self._temporary.name) / "run-20261110"
        self.mailbox = self.run_dir / "interactive"
        self.mailbox.mkdir(parents=True)

    def tearDown(self):
        self._temporary.cleanup()

    def test_yaw_request_uses_a_relative_turn(self):
        payload = client.submit(
            self.mailbox,
            action="yaw",
            pose_source="lio_estimate",
            extra={"delta_yaw_rad": 1.5707963267948966},
        )
        self.assertEqual(1, payload["seq"])
        self.assertAlmostEqual(1.5707963267948966, payload["delta_yaw_rad"], places=9)
        stored = json.loads((self.mailbox / "request.json").read_text(encoding="utf-8"))
        self.assertEqual("yaw", stored["action"])
        self.assertNotIn("goal_xyz_m", stored)

    def test_yaw_above_one_turn_is_refused(self):
        with self.assertRaises(ValueError):
            client.submit(
                self.mailbox,
                action="yaw",
                pose_source="lio_estimate",
                extra={"delta_yaw_rad": 5.0},
            )
        self.assertFalse((self.mailbox / "request.json").exists())

    def test_maneuver_request_is_bounded(self):
        payload = client.submit(
            self.mailbox,
            action="maneuver",
            pose_source="lio_estimate",
            extra={
                "body_forward_mps": 0.4,
                "body_lateral_mps": 0.02,
                "yaw_rate_rps": 0.1,
                "duration_s": 0.5,
            },
        )
        self.assertEqual(0.4, payload["body_forward_mps"])
        self.assertEqual(0.5, payload["duration_s"])

    def test_maneuver_above_the_ceiling_is_refused(self):
        with self.assertRaises(ValueError):
            client.submit(
                self.mailbox,
                action="maneuver",
                pose_source="lio_estimate",
                extra={"body_forward_mps": 1.5, "duration_s": 0.5},
            )

    def test_maneuver_above_the_lateral_bound_is_refused(self):
        with self.assertRaises(ValueError):
            client.submit(
                self.mailbox,
                action="maneuver",
                pose_source="lio_estimate",
                extra={"body_lateral_mps": 0.2, "duration_s": 0.5},
            )

    def test_maneuver_duration_is_bounded(self):
        with self.assertRaises(ValueError):
            client.submit(
                self.mailbox,
                action="maneuver",
                pose_source="lio_estimate",
                extra={"body_forward_mps": 0.2, "duration_s": 5.0},
            )

    def test_latest_sim_frame_reports_the_newest_still(self):
        frames = self.run_dir / "interactive_frames"
        frames.mkdir()
        (frames / "rgb_000000.png").write_bytes(b"png")
        (frames / "rgb_000050.png").write_bytes(b"png")
        (frames / "index.jsonl").write_text("{}\n", encoding="utf-8")
        described = client.latest_sim_frame(self.run_dir)
        self.assertTrue(described["latest"].endswith("rgb_000050.png"))
        self.assertEqual(2, described["count"])
        self.assertFalse(described["pose_included"])

    def test_latest_sim_frame_without_stills(self):
        self.assertIsNone(client.latest_sim_frame(self.run_dir))

    def test_wait_for_action_finds_the_completion_receipt(self):
        path = self.mailbox / "receipts.jsonl"
        path.write_text(
            json.dumps({"event": "yaw_accepted", "seq": 9})
            + "\n"
            + json.dumps({"event": "yaw_complete", "seq": 9, "yaw_error_rad": 0.01})
            + "\n",
            encoding="utf-8",
        )
        receipt = client.wait_for_action(self.mailbox, 9, 1.0)
        self.assertEqual("yaw_complete", receipt["event"])

    def test_wait_for_action_times_out(self):
        receipt = client.wait_for_action(self.mailbox, 4, 0.08)
        self.assertEqual("timeout", receipt["event"])

    def test_scan_without_a_simulation_times_out_with_exit_code_two(self):
        code = client.main(
            [
                "scan",
                "--mailbox",
                str(self.mailbox),
                "--steps",
                "1",
                "--timeout-per-step",
                "0.08",
                "--settle-s",
                "0",
            ]
        )
        # A pending turn is a timeout, never a clean success.
        self.assertEqual(2, code)
        self.assertTrue((self.mailbox / "scan").is_dir())

    def test_scan_reports_a_rejected_turn_with_exit_code_one(self):
        (self.mailbox / "receipts.jsonl").write_text(
            json.dumps(
                {
                    "event": "request_rejected",
                    "seq": 1,
                    "reason": "pose_source_mismatch",
                    "run_id": "run-20261110",
                }
            )
            + "\n",
            encoding="utf-8",
        )
        code = client.main(
            [
                "scan",
                "--mailbox",
                str(self.mailbox),
                "--steps",
                "1",
                "--timeout-per-step",
                "0.2",
                "--settle-s",
                "0",
            ]
        )
        self.assertEqual(1, code)

    def test_scan_rejects_zero_steps(self):
        code = client.main(
            ["scan", "--mailbox", str(self.mailbox), "--steps", "0"]
        )
        self.assertEqual(3, code)


class WaitForActionResultTests(unittest.TestCase):
    """A rejected or cancelled action must never look like a running one."""

    def setUp(self):
        self._temporary = tempfile.TemporaryDirectory()
        self.run_dir = Path(self._temporary.name) / "run-20261110"
        self.mailbox = self.run_dir / "interactive"
        self.mailbox.mkdir(parents=True)

    def tearDown(self):
        self._temporary.cleanup()

    def write_receipts(self, *receipts):
        (self.mailbox / "receipts.jsonl").write_text(
            "\n".join(json.dumps(receipt) for receipt in receipts) + "\n",
            encoding="utf-8",
        )

    def wait(self, seq, timeout_s):
        started = time.perf_counter()
        result = client.wait_for_action(self.mailbox, seq, timeout_s)
        return result, time.perf_counter() - started

    def run_cli(self, argv):
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            code = client.main(argv)
        printed = [
            json.loads(line)
            for line in buffer.getvalue().splitlines()
            if line.strip()
        ]
        return code, printed

    def test_request_rejected_returns_immediately_with_the_reason(self):
        self.write_receipts(
            {
                "protocol_version": client.PROTOCOL_VERSION,
                "event": "request_rejected",
                "run_id": "run-20261110",
                "seq": 1,
                "reason": "pose_source_mismatch",
                "action": "goto",
            }
        )
        result, elapsed = self.wait(1, 2.0)
        self.assertEqual("rejected", result["status"])
        self.assertFalse(result["success"])
        self.assertEqual("pose_source_mismatch", result["reason"])
        self.assertEqual("request_rejected", result["event"])
        self.assertEqual(1, result["seq"])
        self.assertEqual("run-20261110", result["run_id"])
        self.assertLess(elapsed, 0.5)

    def test_action_cancelled_for_this_seq_returns_cancelled(self):
        self.write_receipts(
            {"event": "yaw_accepted", "seq": 4, "run_id": "run-20261110"},
            {
                "event": "action_cancelled",
                "seq": 4,
                "reason": "obstacle_too_close",
                "run_id": "run-20261110",
            },
        )
        result, elapsed = self.wait(4, 2.0)
        self.assertEqual("cancelled", result["status"])
        self.assertFalse(result["success"])
        self.assertEqual("obstacle_too_close", result["reason"])
        self.assertEqual("action_cancelled", result["event"])
        self.assertEqual(4, result["seq"])
        self.assertLess(elapsed, 0.5)

    def test_waypoint_dropped_for_this_seq_returns_dropped(self):
        self.write_receipts(
            {
                "event": "waypoint_dropped",
                "seq": 2,
                "reason": "stalled",
                "run_id": "run-20261110",
            }
        )
        result, elapsed = self.wait(2, 2.0)
        self.assertEqual("dropped", result["status"])
        self.assertFalse(result["success"])
        self.assertEqual("stalled", result["reason"])
        self.assertLess(elapsed, 0.5)

    def test_native_waypoint_failure_closes_only_matching_sequence_immediately(self):
        self.write_receipts(
            {"event": "waypoint_failed", "seq": 1, "run_id": "run-20261110",
             "reason": "route_low_progress_stall", "run_continues": True},
            {"event": "waypoint_accepted", "seq": 2, "run_id": "run-20261110"},
        )
        result, elapsed = self.wait(1, 2.)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["reason"], "route_low_progress_stall")
        self.assertTrue(result["run_continues"])
        self.assertEqual(client.action_exit_code(result), client.EXIT_ACTION_FAILED)
        self.assertLess(elapsed, .5)
        result, _ = self.wait(2, .08)
        self.assertEqual(result["status"], "timeout")

    def test_global_terminal_without_a_seq_returns_immediately(self):
        self.write_receipts(
            {
                "event": "terminal",
                "reason": "interactive_link_lost",
                "run_id": "run-20261110",
            }
        )
        result, elapsed = self.wait(7, 2.0)
        self.assertEqual("terminal", result["status"])
        self.assertFalse(result["success"])
        self.assertEqual("interactive_link_lost", result["reason"])
        self.assertEqual("terminal", result["event"])
        self.assertEqual(7, result["seq"])
        self.assertEqual("run-20261110", result["run_id"])
        self.assertLess(elapsed, 0.5)

    def test_hold_after_acceptance_returns_held(self):
        self.write_receipts(
            {"event": "waypoint_accepted", "seq": 3, "run_id": "run-20261110"},
            {
                "event": "hold",
                "reason": "stale_wall_odometry",
                "run_id": "run-20261110",
            },
        )
        result, elapsed = self.wait(3, 2.0)
        self.assertEqual("held", result["status"])
        self.assertFalse(result["success"])
        self.assertEqual("stale_wall_odometry", result["reason"])
        self.assertLess(elapsed, 0.5)

    def test_a_startup_hold_before_acceptance_does_not_end_the_wait(self):
        self.write_receipts(
            {
                "event": "hold",
                "reason": "operator_link_not_established",
                "run_id": "run-20261110",
            }
        )
        result, _ = self.wait(1, 0.08)
        self.assertEqual("timeout", result["status"])

    def test_receipt_from_another_run_is_ignored(self):
        self.write_receipts(
            {
                "event": "request_rejected",
                "seq": 1,
                "reason": "run_id_mismatch",
                "run_id": "run-other",
            },
            {
                "event": "terminal",
                "reason": "interactive_link_lost",
                "run_id": "run-other",
            },
        )
        result, _ = self.wait(1, 0.08)
        self.assertEqual("timeout", result["status"])
        self.assertFalse(result["success"])

    def test_older_sequence_number_is_not_mistaken_for_this_action(self):
        self.write_receipts(
            {
                "event": "request_rejected",
                "seq": 2,
                "reason": "duplicate_or_stale_sequence",
                "run_id": "run-20261110",
            },
            {
                "event": "action_cancelled",
                "seq": 2,
                "reason": "obstacle_too_close",
                "run_id": "run-20261110",
            },
        )
        result, _ = self.wait(5, 0.08)
        self.assertEqual("timeout", result["status"])

    def test_successful_completion_keeps_the_receipt_fields(self):
        self.write_receipts(
            {"event": "yaw_accepted", "seq": 9, "run_id": "run-20261110"},
            {
                "event": "yaw_complete",
                "seq": 9,
                "yaw_error_rad": 0.01,
                "run_id": "run-20261110",
            },
        )
        result, elapsed = self.wait(9, 2.0)
        self.assertEqual("completed", result["status"])
        self.assertTrue(result["success"])
        self.assertEqual("yaw_complete", result["event"])
        self.assertEqual(9, result["seq"])
        self.assertAlmostEqual(0.01, result["yaw_error_rad"])
        self.assertLess(elapsed, 0.5)

    def test_timeout_is_distinguishable_from_a_failure(self):
        result, elapsed = self.wait(6, 0.08)
        self.assertEqual("timeout", result["status"])
        self.assertFalse(result["success"])
        self.assertEqual("timeout", result["event"])
        self.assertEqual(6, result["seq"])
        self.assertEqual("run-20261110", result["run_id"])
        self.assertGreaterEqual(elapsed, 0.05)
        self.assertEqual(2, client.action_exit_code(result))

    def test_missing_receipts_file_does_not_raise(self):
        result, _ = self.wait(1, 0.05)
        self.assertEqual("timeout", result["status"])

    def test_corrupt_receipts_file_does_not_raise(self):
        (self.mailbox / "receipts.jsonl").write_text(
            '{"event": "request_rejected", "seq": 1, "reason": "broken"'
            + "\n"
            + "[1, 2, 3]\n"
            + '"not a receipt"\n'
            + '{"event": "terminal", "reason": "after_corruption"}\n',
            encoding="utf-8",
        )
        result, elapsed = self.wait(1, 2.0)
        self.assertEqual("terminal", result["status"])
        self.assertEqual("after_corruption", result["reason"])
        self.assertLess(elapsed, 0.5)

    def test_undecodable_receipts_file_does_not_raise(self):
        (self.mailbox / "receipts.jsonl").write_bytes(
            b"{\"event\": \"terminal\", \"reason\": \"truncated\"\n"
            b"\xff\xfe\x00garbage\n"
        )
        result, _ = self.wait(1, 0.05)
        self.assertEqual("timeout", result["status"])

    def test_exit_codes_separate_success_failure_and_timeout(self):
        self.assertEqual(0, client.action_exit_code({"status": "completed", "success": True}))
        self.assertEqual(1, client.action_exit_code({"status": "rejected", "success": False}))
        self.assertEqual(1, client.action_exit_code({"status": "terminal", "success": False}))
        self.assertEqual(1, client.action_exit_code({"status": "held", "success": False}))
        self.assertEqual(2, client.action_exit_code({"status": "timeout", "success": False}))

    def test_goto_wait_exits_one_on_a_rejected_action(self):
        self.write_receipts(
            {
                "event": "request_rejected",
                "seq": 1,
                "reason": "request_expired",
                "run_id": "run-20261110",
            }
        )
        code, printed = self.run_cli(
            [
                "--mailbox",
                str(self.mailbox),
                "goto",
                "--x",
                "1.0",
                "--y",
                "2.0",
                "--z",
                "3.0",
                "--wait",
                "--timeout",
                "0.2",
            ]
        )
        self.assertEqual(1, code)
        result = printed[-1]
        self.assertEqual("rejected", result["status"])
        self.assertEqual("request_expired", result["reason"])
        self.assertEqual(1, result["seq"])

    def test_goto_wait_exits_zero_on_arrival(self):
        self.write_receipts(
            {"event": "waypoint_reached", "seq": 1, "run_id": "run-20261110"}
        )
        code, printed = self.run_cli(
            [
                "--mailbox",
                str(self.mailbox),
                "goto",
                "--x",
                "1.0",
                "--y",
                "2.0",
                "--z",
                "3.0",
                "--wait",
                "--timeout",
                "0.2",
            ]
        )
        self.assertEqual(0, code)
        self.assertEqual("completed", printed[-1]["status"])

    def test_goto_wait_exits_two_when_nothing_answers(self):
        code, printed = self.run_cli(
            [
                "--mailbox",
                str(self.mailbox),
                "goto",
                "--x",
                "1.0",
                "--y",
                "2.0",
                "--z",
                "3.0",
                "--wait",
                "--timeout",
                "0.05",
            ]
        )
        self.assertEqual(2, code)
        self.assertEqual("timeout", printed[-1]["status"])


class OperatorMapParsingTests(unittest.TestCase):
    """The operator map reads the live cloud; parsing must be exact and fail closed."""

    def make_cloud(self, values, point_step=16, intensity=False):
        numpy = __import__("numpy")
        from types import SimpleNamespace

        array = numpy.asarray(values, dtype=numpy.float32)
        fields = [
            SimpleNamespace(name="x", offset=0, datatype=7),
            SimpleNamespace(name="y", offset=4, datatype=7),
            SimpleNamespace(name="z", offset=8, datatype=7),
        ]
        if intensity:
            fields.append(SimpleNamespace(name="intensity", offset=12, datatype=7))
        return SimpleNamespace(
            fields=fields,
            point_step=point_step,
            width=len(array),
            height=1,
            data=array.tobytes(),
        )

    def test_cloud_to_xyz_parses_the_float32_layout(self):
        message = self.make_cloud([[1.0, 2.0, 3.0, 9.0], [0.5, -0.5, 0.25, 1.0]], intensity=True)
        xyz = client.cloud_to_xyz(message)
        self.assertEqual((2, 3), xyz.shape)
        self.assertAlmostEqual(1.0, float(xyz[0][0]))
        self.assertAlmostEqual(-0.5, float(xyz[1][1]))

    def test_cloud_to_xyz_drops_non_finite_points(self):
        message = self.make_cloud(
            [[1.0, 0.0, 0.0, 0.0], [float("nan"), 0.0, 0.0, 0.0]], intensity=True
        )
        xyz = client.cloud_to_xyz(message)
        self.assertEqual(1, len(xyz))

    def test_cloud_to_xyz_keeps_a_large_scan_whole_and_rejects_over_budget(self):
        numpy = __import__("numpy")
        points = numpy.zeros((54_000, 4), dtype=numpy.float32)
        points[:, 0] = numpy.arange(len(points))
        message = self.make_cloud(points, intensity=True)
        xyz = client.cloud_to_xyz(message)
        self.assertEqual(len(xyz), 54_000)
        self.assertEqual(xyz[-1, 0], 53_999)
        message.width = client.OPERATOR_CLOUD_MAX_POINTS + 1
        with self.assertRaisesRegex(ValueError, "not truncating or resampling"):
            client.cloud_to_xyz(message)

    def test_map_refuses_raw_rtx_before_starting_ros(self):
        from types import SimpleNamespace

        with self.assertRaisesRegex(ValueError, "IMU deskew"):
            client._cmd_map(SimpleNamespace(cloud_topic="/dynanav/lidar_rtx"))

    def test_cloud_to_xyz_requires_xyz_fields(self):
        from types import SimpleNamespace

        message = SimpleNamespace(
            fields=[SimpleNamespace(name="intensity", offset=0, datatype=7)],
            point_step=4,
            width=1,
            height=1,
            data=b"\x00\x00\x00\x00",
        )
        with self.assertRaises(ValueError):
            client.cloud_to_xyz(message)

    def test_map_requires_an_explicit_extrinsic(self):
        with tempfile.TemporaryDirectory() as temporary:
            mailbox = Path(temporary) / "run" / "interactive"
            mailbox.mkdir(parents=True)
            code = client.main(
                [
                    "map",
                    "--mailbox",
                    str(mailbox),
                    "--odometry-topic",
                    "/dynanav/lio_observer/aft_mapped_to_init",
                    "--duration",
                    "0.1",
                ]
            )
            self.assertEqual(3, code)
            self.assertFalse((Path(temporary) / "run" / "operator_map").exists())


class CloudFrameTests(unittest.TestCase):
    """The cloud header, not a CLI flag, decides which extrinsic applies."""

    def test_matching_frame_is_returned(self):
        self.assertEqual("go2_imu", client.verify_cloud_frame("go2_imu", "go2_imu"))

    def test_mismatched_frame_is_refused(self):
        with self.assertRaises(ValueError):
            client.verify_cloud_frame("go2_base", "go2_imu")

    def test_missing_expected_frame_is_refused(self):
        with self.assertRaises(ValueError):
            client.verify_cloud_frame("go2_imu", "")

    def test_missing_declared_frame_is_refused(self):
        with self.assertRaises(ValueError):
            client.verify_cloud_frame("", "go2_imu")


class OccupancyGridTests(unittest.TestCase):
    """The operator aid must separate floor, obstacle and unknown cells honestly."""

    def test_floor_points_become_free_and_obstacles_occupied(self):
        grid = client.OccupancyGrid(resolution_m=0.10)
        grid.add_pose([0.0, 0.0, 0.0])
        grid.add_points(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.5, 1.0]],
            [0.0, 0.0, 0.0],
            0.0,
        )
        snapshot = grid.to_dict(pose_xyz=[0.0, 0.0, 0.0])
        self.assertEqual(2, snapshot["free_cells"])
        self.assertEqual(1, snapshot["occupied_cells"])
        self.assertEqual(1, snapshot["scans"])
        self.assertEqual(3, snapshot["points"])
        self.assertFalse(snapshot["ground_truth_used"])
        self.assertEqual(
            "live_depth_points_transformed_by_measured_estimator_pose",
            snapshot["map_source"],
        )

    def test_points_below_the_measured_pose_are_ignored(self):
        grid = client.OccupancyGrid()
        grid.add_points([[1.0, 0.0, -1.0]], [0.0, 0.0, 0.0], 0.0)
        self.assertEqual({}, grid.cells)

    def test_points_beyond_range_are_ignored(self):
        grid = client.OccupancyGrid(max_range_m=5.0)
        grid.add_points([[9.0, 0.0, 0.0]], [0.0, 0.0, 0.0], 0.0)
        self.assertEqual({}, grid.cells)

    def test_frontier_marks_observed_free_cells_next_to_unknown(self):
        grid = client.OccupancyGrid(resolution_m=0.10)
        for index in range(5):
            grid.add_points(
                [[index * 0.1, 0.0, 0.0]], [0.0, 0.0, 0.0], 0.0
            )
        snapshot = grid.to_dict(pose_xyz=[0.0, 0.0, 0.0])
        self.assertEqual(5, snapshot["free_cells"])
        self.assertGreater(snapshot["frontier"]["cell_count"], 0)
        self.assertAlmostEqual(
            0.05, snapshot["frontier"]["cells"][0]["xy_m"][0], places=2
        )

    def test_snapshot_measured_path_length(self):
        grid = client.OccupancyGrid()
        for position in ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 2.0, 0.0]):
            grid.add_pose(position)
        self.assertAlmostEqual(
            3.0, grid.to_dict()["measured_path_length_m"], places=6
        )

    def test_snapshot_is_empty_and_honest_without_scans(self):
        snapshot = client.OccupancyGrid().to_dict()
        self.assertEqual(0, snapshot["free_cells"])
        self.assertEqual(0, snapshot["frontier"]["cell_count"])
        self.assertEqual(0.0, snapshot["observed_area_m2"])
        self.assertFalse(snapshot["ground_truth_used"])



class OperatorMapSnapshotClockTests(unittest.TestCase):
    """The client must feed the map a real source clock and unclipped ages.

    These tests drive the production helper the map loop calls, with a real
    OccupancyAccumulator, and assert both the manifest and the written level file:
    a snapshot the map refuses must never be exported as actionable.
    """

    @staticmethod
    def _pose(z=0.3):
        import numpy as np
        from embodied_navigation.simulation.dynanav import operator_map as om

        return om.transform_from_position_quaternion(
            [0.0, 0.0, z], om.quaternion_xyzw_from_yaw(0.0)
        )

    @staticmethod
    def _floor(z=0.0):
        import numpy as np

        xs = ((np.arange(40) - 20) + 0.5) * 0.1
        ys = ((np.arange(40) - 20) + 0.5) * 0.1
        gx, gy = np.meshgrid(xs, ys)
        return np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, float(z))], axis=1)

    def _accumulator(self):
        from embodied_navigation.simulation.dynanav.operator_map import (
            Extrinsic,
            OccupancyAccumulator,
        )

        return OccupancyAccumulator(
            extrinsic=Extrinsic(__import__("numpy").eye(4)), resolution_m=0.1
        )

    def _args(self):
        from types import SimpleNamespace

        return SimpleNamespace(
            cloud_topic="/cloud",
            odometry_topic="/odom",
            sensor_frame="map",
            base_frame="go2_base",
            pose_source="lio_estimate",
            frontier_limit=10,
        )

    def _state(self, **overrides):
        state = {
            "clock": None,
            "clock_arrival_wall": None,
            "declared_frame": "map",
            "frame_mismatch": None,
            "frames": 1,
            "accepted": 1,
            "rejected": 0,
            "last": None,
        }
        state.update(overrides)
        return state

    def _write(self, accumulator, state):
        with tempfile.TemporaryDirectory() as temporary:
            out_dir = Path(temporary) / "operator_map"
            manifest = client._write_operator_map_snapshots(
                accumulator, out_dir, self._args(), state, event="operator_map_accumulated"
            )
            snapshots = {}
            for key, entry in manifest["levels"].items():
                snapshots[key] = json.loads(Path(entry["path"]).read_text())
            return manifest, snapshots

    def _level(
        self, accumulator, *, cloud_stamp, sim_age, wall_age, pose_stamp=None, floor_z=0.0
    ):
        now = time.monotonic()
        stamp = cloud_stamp if pose_stamp is None else pose_stamp
        accumulator.add_pose(
            stamp, self._pose(floor_z + 0.3), arrival_wall_s=now - wall_age
        )
        return accumulator.add_frame(
            self._floor(floor_z),
            frame="map",
            cloud_stamp_s=cloud_stamp,
            pose=self._pose(floor_z + 0.3),
            pose_stamp_s=stamp,
            sim_age_s=sim_age,
            wall_age_s=wall_age,
            arrival_wall_s=now - wall_age,
            pose_arrival_wall_s=now - wall_age,
        )

    def test_the_current_sim_clock_is_passed_to_the_snapshot(self):
        accumulator = self._accumulator()
        self._level(accumulator, cloud_stamp=10.0, sim_age=0.05, wall_age=0.05)
        manifest, snapshots = self._write(
            accumulator,
            self._state(clock=10.05, clock_arrival_wall=time.perf_counter()),
        )
        self.assertEqual(10.05, manifest["source_clock_used"])
        self.assertEqual("current_clock_sample", manifest["source_clock_note"])
        level = next(iter(snapshots.values()))
        self.assertTrue(level["frontier"]["actionable"])
        self.assertEqual([], level["frontier"]["reasons"])

    def test_a_missing_clock_topic_is_not_used_as_now(self):
        accumulator = self._accumulator()
        self._level(accumulator, cloud_stamp=10.0, sim_age=0.05, wall_age=0.05)
        manifest, _ = self._write(accumulator, self._state())
        self.assertIsNone(manifest["source_clock_used"])
        self.assertEqual("no_clock_topic_sample", manifest["source_clock_note"])

    def test_a_stale_clock_sample_is_not_used_as_now(self):
        accumulator = self._accumulator()
        self._level(accumulator, cloud_stamp=10.0, sim_age=0.05, wall_age=0.05)
        manifest, _ = self._write(
            accumulator,
            self._state(
                clock=10.0, clock_arrival_wall=time.perf_counter() - 1.0
            ),
        )
        self.assertIsNone(manifest["source_clock_used"])
        self.assertEqual("clock_sample_stale", manifest["source_clock_note"])

    def test_a_future_source_stamp_is_not_actionable(self):
        accumulator = self._accumulator()
        self._level(accumulator, cloud_stamp=4010.0, sim_age=10.0 - 4010.0, wall_age=0.05)
        manifest, snapshots = self._write(
            accumulator,
            self._state(clock=10.0, clock_arrival_wall=time.perf_counter()),
        )
        self.assertEqual(10.0, manifest["source_clock_used"])
        level = next(iter(snapshots.values()))
        self.assertFalse(level["frontier"]["actionable"])
        self.assertIn("sim_clock_regression", level["frontier"]["reasons"])
        self.assertLess(level["snapshot_freshness"]["cloud_sim_age_s"], 0.0)

    def test_a_sim_delay_beyond_the_window_is_not_actionable(self):
        accumulator = self._accumulator()
        self._level(accumulator, cloud_stamp=10.0, sim_age=0.4, wall_age=0.05)
        manifest, snapshots = self._write(
            accumulator,
            self._state(clock=10.4, clock_arrival_wall=time.perf_counter()),
        )
        self.assertEqual(10.4, manifest["source_clock_used"])
        level = next(iter(snapshots.values()))
        self.assertFalse(level["frontier"]["actionable"])
        self.assertIn("stale_cloud_sim_age", level["frontier"]["reasons"])

    def test_a_cloud_stall_with_a_fresh_pose_stays_not_actionable(self):
        accumulator = self._accumulator()
        self._level(accumulator, cloud_stamp=10.0, sim_age=0.05, wall_age=0.6)
        accumulator.add_pose(10.0, self._pose(), arrival_wall_s=time.monotonic() - 0.001)
        _, snapshots = self._write(
            accumulator,
            self._state(clock=10.05, clock_arrival_wall=time.perf_counter()),
        )
        level = next(iter(snapshots.values()))
        self.assertFalse(level["frontier"]["actionable"])
        self.assertIn("stale_cloud_wall_age", level["frontier"]["reasons"])
        self.assertGreater(level["snapshot_freshness"]["cloud_wall_age_s"], 0.5)

    def test_each_level_ages_from_its_own_observation(self):
        accumulator = self._accumulator()
        self._level(accumulator, cloud_stamp=10.0, sim_age=0.7, wall_age=0.7)
        self._level(accumulator, cloud_stamp=10.7, sim_age=0.05, wall_age=0.05, floor_z=3.3)
        _, snapshots = self._write(
            accumulator,
            self._state(clock=10.75, clock_arrival_wall=time.perf_counter()),
        )
        actionable = {
            key: snapshot["frontier"]["actionable"]
            for key, snapshot in snapshots.items()
        }
        self.assertEqual(2, len(snapshots))
        self.assertEqual(1, sum(1 for value in actionable.values() if value))

    def test_a_refused_empty_frame_keeps_the_export_not_actionable(self):
        accumulator = self._accumulator()
        self._level(accumulator, cloud_stamp=10.0, sim_age=0.05, wall_age=0.05)
        import numpy as np

        receipt = accumulator.add_frame(
            np.zeros((0, 3)),
            frame="map",
            cloud_stamp_s=10.1,
            pose=self._pose(),
            pose_stamp_s=10.1,
            sim_age_s=0.05,
            wall_age_s=0.05,
            arrival_wall_s=time.monotonic() - 0.05,
            pose_arrival_wall_s=time.monotonic() - 0.05,
        )
        self.assertFalse(receipt["accepted"])
        _, snapshots = self._write(
            accumulator,
            self._state(clock=10.15, clock_arrival_wall=time.perf_counter()),
        )
        level = next(iter(snapshots.values()))
        self.assertFalse(level["frontier"]["actionable"])
        self.assertIn("latest_frame_refused", level["frontier"]["reasons"])



class MapPoseFrameContractTests(unittest.TestCase):
    """The operator map may not treat an IMU pose as the base pose."""

    @staticmethod
    def yaw_pose(yaw, t):
        import numpy as np

        pose = np.eye(4)
        pose[:3, :3] = np.array(
            [
                [math.cos(yaw), -math.sin(yaw), 0.0],
                [math.sin(yaw), math.cos(yaw), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        pose[:3, 3] = t
        return pose

    def test_a_base_child_frame_is_used_as_is(self):
        import numpy as np

        pose = self.yaw_pose(0.3, [1.0, 2.0, 0.5])
        mapped, note = client._map_pose_from_odometry(
            pose, "dynanav_lio_map", "go2_base", base_frame="go2_base", imu_mount=None
        )
        np.testing.assert_allclose(pose, mapped, atol=1e-12)
        self.assertEqual("child_frame_is_base", note)

    def test_an_imu_pose_is_converted_once_with_the_measured_mount(self):
        import numpy as np

        mount = np.array([0.02, 0.0, -0.005])
        pose = self.yaw_pose(math.pi / 2.0, [1.0, 2.0, 0.3])
        mapped, note = client._map_pose_from_odometry(
            pose, "dynanav_lio_map", "go2_imu", base_frame="go2_base", imu_mount=mount
        )
        self.assertEqual("imu_pose_converted_with_measured_imu_mount", note)
        np.testing.assert_allclose(mapped[:3, :3], pose[:3, :3], atol=1e-12)
        expected = pose[:3, :3] @ (-mount) + pose[:3, 3]
        np.testing.assert_allclose(mapped[:3, 3], expected, atol=1e-12)

    def test_an_imu_pose_without_a_measured_mount_is_refused(self):
        pose = self.yaw_pose(0.0, [0.0, 0.0, 0.0])
        mapped, note = client._map_pose_from_odometry(
            pose, "dynanav_lio_map", "go2_imu", base_frame="go2_base", imu_mount=None
        )
        self.assertIsNone(mapped)
        self.assertEqual("imu_pose_without_measured_imu_mount", note)

    def test_an_unknown_child_frame_is_refused(self):
        pose = self.yaw_pose(0.0, [0.0, 0.0, 0.0])
        mapped, note = client._map_pose_from_odometry(
            pose,
            "dynanav_lio_map",
            "base_footprint",
            base_frame="go2_base",
            imu_mount=[0.02, 0.0, -0.005],
        )
        self.assertIsNone(mapped)
        self.assertEqual("odometry_child_frame_not_base:base_footprint", note)

    def test_unlabelled_frames_are_refused(self):
        pose = self.yaw_pose(0.0, [0.0, 0.0, 0.0])
        for frame_id, child, reason in (
            ("", "go2_base", "odometry_map_frame_unlabelled"),
            ("dynanav_lio_map", "", "odometry_child_frame_unlabelled"),
            (None, "go2_base", "odometry_map_frame_unlabelled"),
        ):
            mapped, note = client._map_pose_from_odometry(
                pose, frame_id, child, base_frame="go2_base", imu_mount=None
            )
            self.assertIsNone(mapped)
            self.assertEqual(reason, note)

    def test_a_non_finite_mount_is_refused(self):
        pose = self.yaw_pose(0.0, [0.0, 0.0, 0.0])
        mapped, note = client._map_pose_from_odometry(
            pose,
            "dynanav_lio_map",
            "go2_imu",
            base_frame="go2_base",
            imu_mount=[0.02, float("nan"), -0.005],
        )
        self.assertIsNone(mapped)
        self.assertEqual("imu_mount_invalid", note)



class ProductionMapStreamTests(unittest.TestCase):
    """Drive the real map callbacks with ROS-shaped messages, not just helpers.

    The raw cloud is IMU points (header go2_imu) and must reach the accumulator as a
    sensor-frame input; the pose must be converted once with the measured imu_mount.
    A frame change makes the accumulated generation untrusted and non-actionable.
    """

    IMU_MOUNT = [0.020153064280748367, 0.0, -0.005108955316245556]

    @staticmethod
    def cloud_message(frame_id="go2_imu", stamp=10.0, z=-0.3):
        import struct

        data = bytearray()
        for ix in range(40):
            for iy in range(40):
                x = ((ix - 20) + 0.5) * 0.1
                y = ((iy - 20) + 0.5) * 0.1
                data += struct.pack("<fff", x, y, z)
        field = lambda name, offset: type(
            "F", (), {"name": name, "datatype": 7, "offset": offset}
        )()
        header = type(
            "H",
            (),
            {"frame_id": frame_id, "stamp": type("S", (), {"sec": int(stamp), "nanosec": int((stamp % 1) * 1e9)})()},
        )()
        return type(
            "M",
            (),
            {
                "header": header,
                "fields": [field("x", 0), field("y", 4), field("z", 8)],
                "point_step": 12,
                "width": 1600,
                "height": 1,
                "data": bytes(data),
            },
        )()

    @staticmethod
    def odometry_message(frame_id="dynanav_lio_map", child="go2_imu", z=0.3, stamp=10.0):
        header = type(
            "H",
            (),
            {"frame_id": frame_id, "stamp": type("S", (), {"sec": int(stamp), "nanosec": int((stamp % 1) * 1e9)})()},
        )()
        position = type("P", (), {"x": 0.0, "y": 0.0, "z": z})()
        orientation = type("Q", (), {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})()
        inner = type("I", (), {"position": position, "orientation": orientation})()
        return type("M", (), {"header": header, "child_frame_id": child, "pose": type("O", (), {"pose": inner})()})()

    def _args(self):
        from types import SimpleNamespace

        return SimpleNamespace(
            cloud_topic="/dynanav/lidar_filtered",
            odometry_topic="/dynanav/livo_navigation/post_vio_odometry",
            sensor_frame="go2_imu",
            base_frame="go2_base",
            pose_source="livo_post_vivo",
            imu_mount=list(self.IMU_MOUNT),
            frontier_limit=10,
        )

    def _accumulator(self):
        import numpy as np

        from embodied_navigation.simulation.dynanav.operator_map import (
            Extrinsic,
            OccupancyAccumulator,
        )

        extrinsic = Extrinsic(
            translation=list(self.IMU_MOUNT),
            quaternion_xyzw=[0.0, 0.0, 0.0, 1.0],
            sensor_frame="go2_imu",
            base_frame="go2_base",
        )
        return OccupancyAccumulator(extrinsic=extrinsic, resolution_m=0.1)

    def _state(self):
        return {
            "clock": 10.05,
            "clock_arrival_wall": time.perf_counter(),
            "declared_frame": None,
            "frame_mismatch": None,
            "frames": 0,
            "accepted": 0,
            "rejected": 0,
            "last": None,
            "pose_frame_accepted": 0,
            "pose_frame_rejected": 0,
            "pose_frame_note": None,
            "odometry_map_frame": None,
            "frame_rejections": 0,
            "frame_rejection_reason": None,
        }

    def _write(self, accumulator, state):
        with tempfile.TemporaryDirectory() as temporary:
            manifest = client._write_operator_map_snapshots(
                accumulator,
                Path(temporary) / "operator_map",
                self._args(),
                state,
                event="operator_map_accumulated",
            )
            snapshot = json.loads(
                Path(next(iter(manifest["levels"].values()))["path"]).read_text()
            )
            return manifest, snapshot

    def test_a_real_imu_cloud_and_converted_pose_reach_the_map(self):
        accumulator = self._accumulator()
        state = self._state()
        now = time.perf_counter()
        self.assertTrue(
            client._ingest_odometry(
                self.odometry_message(), accumulator, state, self._args(), now
            )
        )
        receipt = client._ingest_cloud(
            self.cloud_message(), now, accumulator, state, self._args()
        )
        self.assertTrue(receipt["accepted"])
        self.assertIn("ros_cloud_frame=go2_imu", str(receipt))
        self.assertEqual(1, state["accepted"])
        self.assertEqual(1, state["pose_frame_accepted"])
        self.assertEqual("imu_pose_converted_with_measured_imu_mount", state["pose_frame_note"])
        manifest, snapshot = self._write(accumulator, state)
        self.assertTrue(snapshot["frontier"]["actionable"], snapshot["frontier"]["reasons"])
        contract = manifest["pose_frame_contract"]
        self.assertEqual("dynanav_lio_map", contract["odometry_map_frame"])
        self.assertEqual("sensor", contract["cloud_input_semantics"])

    def test_a_changed_map_frame_untrusts_the_generation(self):
        accumulator = self._accumulator()
        state = self._state()
        now = time.perf_counter()
        self.assertTrue(
            client._ingest_odometry(self.odometry_message(), accumulator, state, self._args(), now)
        )
        client._ingest_cloud(self.cloud_message(), now, accumulator, state, self._args())
        self.assertFalse(
            client._ingest_odometry(
                self.odometry_message(frame_id="different_map"),
                accumulator,
                state,
                self._args(),
                now,
            )
        )
        self.assertEqual(1, state["frame_rejections"])
        self.assertIn("odometry_map_frame_changed", state["frame_rejection_reason"])
        _, snapshot = self._write(accumulator, state)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("untrusted_generation", snapshot["frontier"]["reasons"])

    def test_an_unexpected_child_frame_untrusts_the_generation(self):
        accumulator = self._accumulator()
        state = self._state()
        now = time.perf_counter()
        client._ingest_odometry(self.odometry_message(), accumulator, state, self._args(), now)
        client._ingest_cloud(self.cloud_message(), now, accumulator, state, self._args())
        self.assertFalse(
            client._ingest_odometry(
                self.odometry_message(child="unexpected_sensor"),
                accumulator,
                state,
                self._args(),
                now,
            )
        )
        self.assertEqual(1, state["pose_frame_rejected"])
        self.assertIn("odometry_child_frame_not_base", state["frame_rejection_reason"])
        _, snapshot = self._write(accumulator, state)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("untrusted_generation", snapshot["frontier"]["reasons"])

    def test_a_cloud_frame_mismatch_untrusts_the_generation(self):
        accumulator = self._accumulator()
        state = self._state()
        now = time.perf_counter()
        client._ingest_odometry(self.odometry_message(), accumulator, state, self._args(), now)
        client._ingest_cloud(self.cloud_message(), now, accumulator, state, self._args())
        receipt = client._ingest_cloud(
            self.cloud_message(frame_id="go2_base", stamp=10.1),
            now,
            accumulator,
            state,
            self._args(),
        )
        self.assertIsNone(receipt)
        self.assertEqual(1, state["rejected"])
        self.assertEqual("cloud_frame_mismatch", state["frame_rejection_reason"])
        _, snapshot = self._write(accumulator, state)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("untrusted_generation", snapshot["frontier"]["reasons"])



class CloudPoseLatencyTests(unittest.TestCase):
    """Admission happens before any geometry write, and waiting only waits for a pose.

    A cloud is written only when its own source stamp and arrival still pass the map
    freshness windows against the current authoritative clocks and a measured pose pair
    is already available. Everything else is refused once with a specific reason and no
    level, cell or accepted counter changes.
    """

    def _setup(self):
        helpers = ProductionMapStreamTests()
        self.accumulator = helpers._accumulator()
        self.args = helpers._args()
        self.state = helpers._state()
        self.cloud_message = helpers.cloud_message
        self.odometry_message = helpers.odometry_message
        return helpers

    def _assert_no_geometry(self):
        self.assertEqual((), self.accumulator.levels)
        self.assertEqual(0, self.state.get("accepted", 0))
        self.assertEqual(0, self.state.get("frames", 0) - self.state.get("rejected", 0))

    def test_a_cloud_waits_for_its_late_pose_and_then_maps(self):
        self._setup()
        now = time.perf_counter()
        pending = [(self.cloud_message(stamp=10.0), now)]
        pending = client._retry_pending_clouds(pending, self.accumulator, self.state, self.args, clock=lambda: now)
        self.assertEqual(1, len(pending))
        self.assertEqual(0, self.state["frames"])
        self.assertTrue(
            client._ingest_odometry(
                self.odometry_message(stamp=10.0), self.accumulator, self.state, self.args, now
            )
        )
        pending = client._retry_pending_clouds(pending, self.accumulator, self.state, self.args, clock=lambda: now + 0.2)
        self.assertEqual([], pending)
        self.assertEqual(1, self.state["frames"])
        self.assertEqual(1, self.state["accepted"])
        self.assertEqual(0, self.state["rejected"])

    def test_a_bracketing_pose_pair_maps_the_cloud(self):
        self._setup()
        now = time.perf_counter()
        client._ingest_odometry(self.odometry_message(stamp=9.95), self.accumulator, self.state, self.args, now)
        client._ingest_odometry(self.odometry_message(stamp=10.05), self.accumulator, self.state, self.args, now)
        pending = client._retry_pending_clouds(
            [(self.cloud_message(stamp=10.0), now)], self.accumulator, self.state, self.args, now
        )
        self.assertEqual([], pending)
        self.assertEqual(1, self.state["accepted"])

    def test_a_cloud_without_a_pose_times_out_once_before_geometry(self):
        self._setup()
        now = time.perf_counter()
        pending = [(self.cloud_message(stamp=10.0), now)]
        pending = client._retry_pending_clouds(pending, self.accumulator, self.state, self.args, clock=lambda: now + 0.6)
        self.assertEqual([], pending)
        self._assert_no_geometry()
        self.assertEqual(1, self.state["rejected"])
        self.assertEqual("cloud_wall_expired", self.state["last_cloud_rejection"]["reason"])

    def test_a_cloud_past_the_wall_deadline_is_rejected_before_geometry(self):
        self._setup()
        now = time.perf_counter()
        client._ingest_odometry(
            self.odometry_message(stamp=10.0), self.accumulator, self.state, self.args, now
        )
        pending = client._retry_pending_clouds(
            [(self.cloud_message(stamp=10.0), now - 0.6)],
            self.accumulator,
            self.state,
            self.args,
            clock=lambda: now,
        )
        self.assertEqual([], pending)
        self._assert_no_geometry()
        self.assertEqual(1, self.state["rejected"])
        self.assertEqual("cloud_wall_expired", self.state["last_cloud_rejection"]["reason"])

    def test_a_sim_expired_cloud_is_rejected_before_geometry(self):
        self._setup()
        self.state["clock"] = self.state["clock"] + 0.4
        now = time.perf_counter()
        client._ingest_odometry(self.odometry_message(stamp=10.0), self.accumulator, self.state, self.args, now)
        pending = client._retry_pending_clouds(
            [(self.cloud_message(stamp=10.0), now)], self.accumulator, self.state, self.args, now
        )
        self.assertEqual([], pending)
        self._assert_no_geometry()
        self.assertEqual("cloud_sim_expired", self.state["last_cloud_rejection"]["reason"])

    def test_a_future_arrival_is_rejected_before_geometry(self):
        self._setup()
        now = time.perf_counter()
        client._ingest_odometry(self.odometry_message(stamp=10.0), self.accumulator, self.state, self.args, now)
        pending = client._retry_pending_clouds(
            [(self.cloud_message(stamp=10.0), now + 4.0)], self.accumulator, self.state, self.args, now
        )
        self.assertEqual([], pending)
        self._assert_no_geometry()
        self.assertEqual("arrival_in_future", self.state["last_cloud_rejection"]["reason"])

    def test_a_non_finite_arrival_is_rejected_before_geometry(self):
        self._setup()
        now = time.perf_counter()
        client._ingest_odometry(self.odometry_message(stamp=10.0), self.accumulator, self.state, self.args, now)
        pending = client._retry_pending_clouds(
            [(self.cloud_message(stamp=10.0), float("nan"))], self.accumulator, self.state, self.args, now
        )
        self.assertEqual([], pending)
        self._assert_no_geometry()
        self.assertEqual("arrival_invalid", self.state["last_cloud_rejection"]["reason"])

    def test_a_future_source_stamp_is_rejected_before_geometry(self):
        self._setup()
        now = time.perf_counter()
        client._ingest_odometry(self.odometry_message(stamp=10.0), self.accumulator, self.state, self.args, now)
        pending = client._retry_pending_clouds(
            [(self.cloud_message(stamp=10.5), now)], self.accumulator, self.state, self.args, now
        )
        self.assertEqual([], pending)
        self._assert_no_geometry()
        self.assertEqual("cloud_source_in_future", self.state["last_cloud_rejection"]["reason"])

    def test_a_missing_clock_is_rejected_before_geometry(self):
        self._setup()
        self.state["clock"] = None
        self.state["clock_arrival_wall"] = None
        now = time.perf_counter()
        client._ingest_odometry(self.odometry_message(stamp=10.0), self.accumulator, self.state, self.args, now)
        pending = client._retry_pending_clouds(
            [(self.cloud_message(stamp=10.0), now)], self.accumulator, self.state, self.args, now
        )
        self.assertEqual([], pending)
        self._assert_no_geometry()
        self.assertEqual("sim_clock_unavailable", self.state["last_cloud_rejection"]["reason"])

    def test_the_pending_cache_is_bounded_with_an_overflow_reason(self):
        self._setup()
        now = time.perf_counter()
        pending = [(self.cloud_message(stamp=10.0 - i * 0.001), now) for i in range(80)]
        pending = client._retry_pending_clouds(pending, self.accumulator, self.state, self.args, now)
        self.assertLessEqual(len(pending), client.CLOUD_PENDING_MAX_COUNT)
        self.assertGreaterEqual(self.state["pending_overflow_rejections"], 16)
        self.assertEqual("pending_overflow", self.state["last_cloud_rejection"]["reason"])
        self._assert_no_geometry()

    def test_a_bad_frame_cloud_is_rejected_immediately_and_untrusts(self):
        helpers = self._setup()
        now = time.perf_counter()
        client._ingest_odometry(self.odometry_message(stamp=10.0), self.accumulator, self.state, self.args, now)
        client._retry_pending_clouds(
            [(self.cloud_message(stamp=10.0), now)], self.accumulator, self.state, self.args, now
        )
        pending = client._retry_pending_clouds(
            [(self.cloud_message(frame_id="go2_base", stamp=10.1), now)],
            self.accumulator,
            self.state,
            self.args,
            now,
        )
        self.assertEqual([], pending)
        self.assertEqual(1, self.state["rejected"])
        self.assertEqual("cloud_frame_mismatch", self.state["frame_rejection_reason"])
        _, snapshot = helpers._write(self.accumulator, self.state)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("untrusted_generation", snapshot["frontier"]["reasons"])

    def test_a_gap_in_the_pose_stream_is_not_extrapolated(self):
        self._setup()
        self.state["clock"] = 10.30
        now = time.perf_counter()
        client._ingest_odometry(self.odometry_message(stamp=9.95), self.accumulator, self.state, self.args, now)
        client._ingest_odometry(self.odometry_message(stamp=10.30), self.accumulator, self.state, self.args, now)
        pending = client._retry_pending_clouds(
            [(self.cloud_message(stamp=10.15), now)],
            self.accumulator,
            self.state,
            self.args,
            clock=lambda: now,
        )
        self.assertEqual(1, len(pending))
        pending = client._retry_pending_clouds(
            pending, self.accumulator, self.state, self.args, clock=lambda: now + 0.6
        )
        self.assertEqual([], pending)
        self._assert_no_geometry()
        self.assertEqual("cloud_wall_expired", self.state["last_cloud_rejection"]["reason"])

class PerItemClockRevalidationTests(unittest.TestCase):
    """Every geometry write re-reads the real clock and re-validates the sample.

    A fresh batch entry time is not evidence for the next item: a previous item's
    processing or the cloud decode can push the arrival past the wall deadline, and the
    write must then be refused with no level/cell/accepted change.
    """

    def _setup(self):
        helpers = ProductionMapStreamTests()
        self.helpers = helpers
        self.accumulator = helpers._accumulator()
        self.args = helpers._args()
        self.state = helpers._state()
        self.state["clock"] = 10.1
        self.cloud_message = helpers.cloud_message
        self.odometry_message = helpers.odometry_message
        return helpers

    def _step_clock(self, values):
        iterator = iter(values)

        def clock():
            return next(iterator, values[-1])

        return clock

    def test_a_batch_delay_expires_the_next_cloud(self):
        self._setup()
        now = time.perf_counter()
        for stamp in (10.0, 10.1):
            client._ingest_odometry(
                self.odometry_message(stamp=stamp), self.accumulator, self.state, self.args, now
            )
        pending = [
            (self.cloud_message(stamp=10.0), now),
            (self.cloud_message(stamp=10.1), now),
        ]
        # item 1 admission, item 1 write re-check, item 2 admission (delayed).
        clock = self._step_clock([now, now, now + 0.6])
        pending = client._retry_pending_clouds(
            pending, self.accumulator, self.state, self.args, clock=clock
        )
        self.assertEqual([], pending)
        self.assertEqual(1, self.state["accepted"])
        self.assertEqual(1, self.state["rejected"])
        self.assertEqual(1, len(self.accumulator.levels))
        self.assertEqual("cloud_wall_expired", self.state["last_cloud_rejection"]["reason"])
        self.assertEqual(1, len(self.accumulator.levels))

    def test_a_slow_cloud_decode_crosses_the_deadline(self):
        self._setup()
        now = time.perf_counter()
        client._ingest_odometry(
            self.odometry_message(stamp=10.0), self.accumulator, self.state, self.args, now
        )
        clock = self._step_clock([now + 0.6])
        receipt = client._ingest_cloud(
            self.cloud_message(stamp=10.0), now, self.accumulator, self.state, self.args, clock=clock
        )
        self.assertFalse(receipt["accepted"])
        self.assertTrue(receipt.get("admission_rejected"))
        self.assertEqual("cloud_wall_expired", receipt["reasons"][0])
        self.assertEqual((), self.accumulator.levels)
        self.assertEqual(0, self.state.get("accepted", 0))

    def test_a_clock_sample_just_inside_the_window_is_accepted(self):
        self._setup()
        now = time.perf_counter()
        self.state["clock_arrival_wall"] = now - 0.20
        client._ingest_odometry(
            self.odometry_message(stamp=10.0), self.accumulator, self.state, self.args, now
        )
        pending = client._retry_pending_clouds(
            [(self.cloud_message(stamp=10.0), now)],
            self.accumulator,
            self.state,
            self.args,
            clock=lambda: now,
        )
        self.assertEqual([], pending)
        self.assertEqual(1, self.state["accepted"])

    def test_an_accepted_consumption_receipt_keeps_the_map_diagnostics(self):
        helpers = ProductionMapStreamTests()
        self.accumulator = helpers._accumulator()
        self.args = helpers._args()
        self.state = helpers._state()
        self.state["clock"] = 10.1
        now = time.perf_counter()
        client._ingest_odometry(
            helpers.odometry_message(stamp=10.0), self.accumulator, self.state, self.args, now
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "cloud_consumption.jsonl"
            pending = client._retry_pending_clouds(
                [(helpers.cloud_message(stamp=10.0), now)],
                self.accumulator,
                self.state,
                self.args,
                clock=lambda: now,
                receipt_path=path,
            )
            self.assertEqual([], pending)
            line = json.loads(path.read_text().splitlines()[0])
            self.assertTrue(line["accepted"])
            self.assertIn("floor_evidence", line)
            self.assertIn("pose_stamp_s", line)
            self.assertIn("paired_dt_s", line)
            self.assertIn("classification_floor_z_m", line)

    def test_a_clock_sample_just_outside_the_window_is_refused(self):
        self._setup()
        now = time.perf_counter()
        self.state["clock_arrival_wall"] = now - 0.36
        client._ingest_odometry(
            self.odometry_message(stamp=10.0), self.accumulator, self.state, self.args, now
        )
        pending = client._retry_pending_clouds(
            [(self.cloud_message(stamp=10.0), now)],
            self.accumulator,
            self.state,
            self.args,
            now_wall=now,
        )
        self.assertEqual([], pending)
        self.assertEqual((), self.accumulator.levels)
        self.assertEqual(0, self.state.get("accepted", 0))
        self.assertEqual("sim_clock_unavailable", self.state["last_cloud_rejection"]["reason"])




class PairPreviewConsistencyTests(unittest.TestCase):
    """The admission preview must be the real pairer algorithm, side-effect free."""

    def _setup(self):
        helpers = ProductionMapStreamTests()
        self.accumulator = helpers._accumulator()
        self.args = helpers._args()
        self.state = helpers._state()
        self.state["clock"] = 19.95
        self.cloud_message = helpers.cloud_message
        self.odometry_message = helpers.odometry_message
        return helpers

    def test_the_f32_boundary_preview_keeps_the_cloud_pending(self):
        self._setup()
        now = time.perf_counter()
        # 0.100000382 s apart: strictly beyond the 0.10 s window.
        client._ingest_odometry(
            self.odometry_message(stamp=19.815000534), self.accumulator, self.state, self.args, now
        )
        pending = client._retry_pending_clouds(
            [(self.cloud_message(stamp=19.915000916), now)],
            self.accumulator,
            self.state,
            self.args,
            clock=lambda: now,
        )
        self.assertEqual(1, len(pending))
        self.assertEqual(0, self.state["frames"])
        client._ingest_odometry(
            self.odometry_message(stamp=19.915000916), self.accumulator, self.state, self.args, now
        )
        pending = client._retry_pending_clouds(
            pending, self.accumulator, self.state, self.args, clock=lambda: now + 0.05
        )
        self.assertEqual([], pending)
        self.assertEqual(1, self.state["frames"])
        self.assertEqual(1, self.state["accepted"])
        self.assertEqual(1, len(self.accumulator.levels))

    def test_inside_the_window_pairs_and_outside_waits(self):
        self._setup()
        now = time.perf_counter()
        client._ingest_odometry(
            self.odometry_message(stamp=19.8151), self.accumulator, self.state, self.args, now
        )
        pending = client._retry_pending_clouds(
            [(self.cloud_message(stamp=19.915000916), now)],
            self.accumulator,
            self.state,
            self.args,
            clock=lambda: now,
        )
        self.assertEqual([], pending)
        self.assertEqual(1, self.state["accepted"])
        self._setup()
        client._ingest_odometry(
            self.odometry_message(stamp=19.8149), self.accumulator, self.state, self.args, now
        )
        pending = client._retry_pending_clouds(
            [(self.cloud_message(stamp=19.915000916), now)],
            self.accumulator,
            self.state,
            self.args,
            clock=lambda: now,
        )
        self.assertEqual(1, len(pending))

    def test_the_preview_does_not_touch_the_pairer_or_a_valid_map(self):
        self._setup()
        now = time.perf_counter()
        client._ingest_odometry(
            self.odometry_message(stamp=19.915000916), self.accumulator, self.state, self.args, now
        )
        client._retry_pending_clouds(
            [(self.cloud_message(stamp=19.915000916), now)],
            self.accumulator,
            self.state,
            self.args,
            clock=lambda: now,
        )
        before = self.accumulator.snapshot(0, frontier_limit=5)
        last = self.accumulator._pairer.last_pairing
        preview = self.accumulator.preview_pose_pair(99.0)
        self.assertFalse(preview.ok)
        self.assertIs(last, self.accumulator._pairer.last_pairing)
        after = self.accumulator.snapshot(0, frontier_limit=5)
        self.assertEqual(
            before["frontier"]["actionable"], after["frontier"]["actionable"]
        )
        self.assertEqual(before["frontier"]["returned_cells"], after["frontier"]["returned_cells"])

    def test_each_final_consumption_is_logged_once(self):
        self._setup()
        now = time.perf_counter()
        client._ingest_odometry(
            self.odometry_message(stamp=19.915000916), self.accumulator, self.state, self.args, now
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "cloud_consumption.jsonl"
            pending = client._retry_pending_clouds(
                [(self.cloud_message(stamp=19.915000916), now)],
                self.accumulator,
                self.state,
                self.args,
                clock=lambda: now,
                receipt_path=path,
            )
            self.assertEqual([], pending)
            lines = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(1, len(lines))
            self.assertTrue(lines[0]["accepted"])
            self.assertFalse(lines[0]["admission_stage_rejected"])
            self.assertAlmostEqual(19.915000916, lines[0]["cloud_source_stamp_s"], places=6)


if __name__ == "__main__":
    unittest.main()


class _FakeLevelAccumulator:
    """Minimal stand-in for OccupancyAccumulator used by the writer test."""

    class _Extrinsic:
        sha256 = "0123456789abcdef"

    extrinsic = _Extrinsic()

    def __init__(self):
        self.levels = [(0, 0.0), (1, 3.2)]

    def snapshot(self, index, frontier_limit=None, now_sim_s=None):
        self.last_now_sim_s = now_sim_s
        return {
            "level_z_m": self.levels[index][1],
            "accumulated_frames": 4,
            "snapshot_is_accumulated_map": True,
            "frontier": {
                "actionable": False,
                "reasons": ["no_frontier"],
                "returned_cells": 0,
            },
        }


class OperatorMapSnapshotWriterTests(unittest.TestCase):
    """The operator must be able to read a fresh map without waiting for exit."""

    def _args(self, out_dir):
        from types import SimpleNamespace

        return SimpleNamespace(
            cloud_topic="/dynanav/lidar_filtered",
            odometry_topic="/dynanav/lio_observer/post_vio_odometry",
            sensor_frame="go2_imu",
            base_frame="go2_base",
            pose_source="livo_post_vio",
            frontier_limit=200,
            out=str(out_dir),
        )

    def test_partial_snapshots_and_manifest_are_written_during_the_run(self):
        with tempfile.TemporaryDirectory() as temporary:
            out_dir = Path(temporary) / "operator_map"
            state = {
                "clock": 10.0,
                "frames": 5,
                "accepted": 4,
                "rejected": 1,
                "last": {"accepted": True},
                "frame_mismatch": None,
                "declared_frame": "go2_imu",
            }
            manifest = client._write_operator_map_snapshots(
                _FakeLevelAccumulator(),
                out_dir,
                self._args(out_dir),
                state,
                event="operator_map_partial",
            )
            self.assertTrue((out_dir / Path("level_%+.2f.json" % 0.0)).is_file())
            self.assertTrue((out_dir / Path("level_%+.2f.json" % 3.2)).is_file())
            latest = json.loads((out_dir / "latest.json").read_text())
            self.assertEqual("operator_map_partial", latest["event"])
            self.assertTrue(latest["snapshot_is_partial_until_final"])
            self.assertFalse(latest["ground_truth_used"])
            self.assertEqual("0123456789abcdef", latest["extrinsic_sha256"])
            self.assertEqual(4, latest["accepted_frames"])
            self.assertEqual(2, len(latest["levels"]))

    def test_the_final_write_is_marked_accumulated(self):
        with tempfile.TemporaryDirectory() as temporary:
            out_dir = Path(temporary) / "operator_map"
            state = {
                "clock": 10.0,
                "frames": 5,
                "accepted": 5,
                "rejected": 0,
                "last": None,
                "frame_mismatch": None,
                "declared_frame": "go2_imu",
            }
            manifest = client._write_operator_map_snapshots(
                _FakeLevelAccumulator(),
                out_dir,
                self._args(out_dir),
                state,
                event="operator_map_accumulated",
            )
            self.assertFalse(manifest["snapshot_is_partial_until_final"])
            latest = json.loads((out_dir / "latest.json").read_text())
            self.assertEqual("operator_map_accumulated", latest["event"])


if __name__ == "__main__":
    unittest.main()
