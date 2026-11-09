"""Recorded-motion metrics exclude standing and never infer mission success."""

import contextlib
import importlib.util
import io
import json
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / "scripts/summarize_dynanav_experiment.py"
spec = importlib.util.spec_from_file_location("summarize_dynanav_experiment", SCRIPT)
summary_script = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary_script)


class DynaNavSummaryTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.run = Path(self.temporary.name)

    def write_json(self, name, value):
        path = self.run / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value))

    def write_stream(self, name, rows):
        path = self.run / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("".join(json.dumps(row) + "\n" for row in rows))

    def test_only_nonzero_actual_command_time_counts(self):
        self.write_json("result.json", {"physics_dt": 0.25, "policy": {"policy_hz": 1}})
        velocities = {1.25: [1, 0, 0], 1.5: [0, 2, 0], 1.75: [0, 0, 3], 2.0: [3, 4, 0]}
        self.write_stream(
            "truth_scoring_stream.jsonl",
            [
                {
                    "time": index / 4,
                    "velocity": velocities.get(index / 4, [100, 0, 0]),
                    "position": [0, 0, index / 4],
                }
                for index in range(1, 13)
            ],
        )
        self.write_stream(
            "policy_frames.jsonl",
            [
                {
                    "measured_time": t,
                    "actual_command": [int(t == 1), 0, 0],
                    "request": [1, 0, 0],
                }
                for t in range(4)
            ],
        )
        result = summary_script.summarize(self.run)
        motion = result["commanded_motion"]
        self.assertEqual(motion["nonzero_command_intervals_s"], [[1, 2]])
        self.assertAlmostEqual(motion["measured_duration_s"], 1)
        self.assertAlmostEqual(motion["xy"]["average_mps"], 2)
        self.assertAlmostEqual(motion["three_dimensional"]["average_mps"], 2.75)
        self.assertEqual(motion["xy"]["p95_mps"], 5)
        self.assertEqual(motion["xy"]["max_mps"], 5)
        self.assertAlmostEqual(result["native_displacement"]["delta_z_m"], 2.75)

    def test_average_is_time_weighted(self):
        self.write_json("result.json", {"physics_dt": 0.5, "policy": {"policy_hz": 1}})
        self.write_stream(
            "policy_frames.jsonl",
            [
                {"measured_time": 0, "actual_command": [1, 0, 0]},
                {"measured_time": 1, "actual_command": [0, 0, 0]},
            ],
        )
        self.write_stream(
            "truth_scoring_stream.jsonl",
            [
                {"time": t, "velocity": [v, 0, 0]}
                for t, v in [(0.1, 1), (0.5, 3), (1, 5)]
            ],
        )
        motion = summary_script.summarize(self.run)["commanded_motion"]
        self.assertAlmostEqual(motion["xy"]["average_mps"], 3.8)
        self.assertAlmostEqual(motion["measured_duration_s"], 1)

    def test_missing_and_empty_evidence_stays_unknown(self):
        result = summary_script.summarize(self.run)
        self.assertEqual(result["evidence"]["result"]["status"], "missing")
        self.assertEqual(result["evidence"]["truth"]["status"], "missing")
        self.assertIsNone(result["commanded_motion"]["xy"]["average_mps"])
        self.assertIsNone(result["native_displacement"]["delta_z_m"])
        self.assertIsNone(result["navigation"]["arrival_reported"])
        self.assertIsNone(result["rgbd"]["empty_rgbd_frames"])
        self.assertFalse(result["navigation"]["mission_success_verified"])
        self.write_stream("truth_scoring_stream.jsonl", [])
        result = summary_script.summarize(self.run)
        self.assertEqual(result["evidence"]["truth"]["status"], "empty")

    def test_zero_commands_do_not_report_zero_speed_as_success(self):
        self.write_json(
            "result.json",
            {"stage": "articulation_contact_probe_complete", "returncode": 0},
        )
        self.write_stream(
            "policy_frames.jsonl",
            [{"measured_time": t, "actual_command": [0, 0, 0]} for t in [0, 0.02]],
        )
        self.write_stream(
            "truth_scoring_stream.jsonl", [{"time": 0.01, "velocity": [0, 0, 0]}]
        )
        result = summary_script.summarize(self.run)
        self.assertEqual(result["commanded_motion"]["status"], "no_nonzero_commands")
        self.assertIsNone(result["commanded_motion"]["xy"]["average_mps"])
        self.assertIsNone(result["navigation"]["arrival_reported"])
        self.assertFalse(result["navigation"]["mission_success_verified"])

    def test_gaps_and_corrupt_lines_are_reported_without_extending_commands(self):
        self.write_json(
            "result.json", {"physics_dt": 0.01, "policy": {"policy_hz": 50}}
        )
        self.write_stream(
            "policy_frames.jsonl",
            [
                {"measured_time": 0, "actual_command": [1, 0, 0]},
                {"measured_time": 1, "actual_command": [0, 0, 0]},
                {"measured_time": 1, "actual_command": [1, 0, 0]},
                {"measured_time": 2, "actual_command": [float("nan"), 0, 0]},
            ],
        )
        with (self.run / "policy_frames.jsonl").open("a") as stream:
            stream.write('{"partial":')
        self.write_stream(
            "truth_scoring_stream.jsonl",
            [
                {"time": t, "velocity": [v, 0, 0]}
                for t, v in [(0.01, 1), (0.02, 1), (0.5, 100)]
            ],
        )
        result = summary_script.summarize(self.run)
        motion = result["commanded_motion"]
        self.assertEqual(motion["nonzero_command_intervals_s"], [[0, 0.02]])
        self.assertEqual(motion["command_gap_count"], 1)
        self.assertEqual(motion["invalid_command_records"], 2)
        self.assertEqual(motion["native_gap_count"], 1)
        self.assertAlmostEqual(motion["xy"]["average_mps"], 1)
        self.assertEqual(result["evidence"]["commands"]["status"], "partial")

    def test_arrival_parking_error_and_conflicting_legacy_receipt_are_separate(self):
        self.write_json(
            "result.json",
            {
                "route_tracker": {"arrived": True, "failure": None, "phase": "arrived"},
                "navigation_terminal_request": {
                    "arrived": False,
                    "failure": "route_arrived",
                },
                "end_reason": "navigation_terminal_and_parked",
                "error": "encoder failed",
            },
        )
        nav = summary_script.summarize(self.run)["navigation"]
        self.assertTrue(nav["arrival_reported"])
        self.assertTrue(nav["arrival_evidence_conflict"])
        self.assertTrue(nav["stopped_and_parked_reported"])
        self.assertEqual(nav["terminal_request_failure"], "route_arrived")
        self.assertEqual(nav["execution_error"], "encoder failed")
        self.assertFalse(nav["mission_success_verified"])

    def test_rgbd_counts_provenance_and_failure_joint_speeds(self):
        self.write_json(
            "result.json",
            {
                "ground_truth_navigation": True,
                "mapping": {
                    "pose_source": "sim_oracle",
                    "mask_source": "sim_oracle",
                    "learned_perception": False,
                },
                "error": "AssertionError('Joint speed exceeds Go2HV motor envelope')",
                "actuator_failure_state": {
                    "qd_isaac": [31, -29],
                    "dof_names": ["a", "b"],
                    "physics_s": 9,
                },
            },
        )
        self.write_stream(
            "mapping/frames.jsonl",
            [{"navigation": n} for n in [0, 10, 1000, 1001, -1, True, None]],
        )
        result = summary_script.summarize(self.run)
        self.assertEqual(result["rgbd"]["empty_rgbd_frames"], 1)
        self.assertEqual(result["rgbd"]["low_count_rgbd_frames_including_empty"], 3)
        self.assertEqual(result["rgbd"]["low_nonempty_rgbd_frames"], 2)
        self.assertEqual(result["rgbd"]["invalid_point_count_records"], 3)
        self.assertFalse(result["rgbd"]["full_scene_coverage_verified"])
        self.assertTrue(result["provenance"]["mapping_pose_oracle"])
        self.assertTrue(result["provenance"]["mapping_mask_oracle"])
        self.assertTrue(result["actuator_failure"]["captured"])
        self.assertEqual(
            result["actuator_failure"]["joints_above_capture_envelope"], ["a"]
        )
        self.assertTrue(result["actuator_failure"]["joint_speed_failure_reported"])

    def test_cli_does_not_overwrite_raw_result_or_existing_summary(self):
        self.write_json("result.json", {"stage": "startup"})
        original = (self.run / "result.json").read_bytes()
        output = self.run / "summary.json"
        self.assertEqual(
            summary_script.main([str(self.run), "--output", str(output)]), 0
        )
        self.assertTrue(output.is_file())
        with (
            contextlib.redirect_stderr(io.StringIO()),
            self.assertRaises(SystemExit) as raised,
        ):
            summary_script.main(
                [str(self.run), "--output", str(self.run / "result.json")]
            )
        self.assertEqual(raised.exception.code, 2)
        self.assertEqual((self.run / "result.json").read_bytes(), original)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            summary_script.main([str(self.run), "--output", str(output)])


if __name__ == "__main__":
    unittest.main()
