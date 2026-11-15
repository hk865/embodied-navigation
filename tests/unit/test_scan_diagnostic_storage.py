"""Diagnostic storage changes preserve the fixed native follower decisions."""
from pathlib import Path
import hashlib
import json
import tempfile
import unittest

from embodied_navigation.simulation.dynanav.pct_scan_waypoints import PctScanWaypointTracker, atomic_json
from scan_diagnostic_fixtures import BASELINE_COMMANDS, command_sequence, large_diagnostic_values


class ScanDiagnosticStorageTests(unittest.TestCase):
    def test_native_follower_commands_and_states_match_fixed_baseline(self):
        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual(command_sequence(Path(directory)), BASELINE_COMMANDS)


    def test_hundred_revision_metadata_stays_light_and_full_provenance_round_trips(self):
        with tempfile.TemporaryDirectory() as directory:
            mailbox = Path(directory) / "run" / "mailbox"
            tracker = PctScanWaypointTracker(dict(slam_pose_source="livo_post_vio",
                speed_mps=1., motion_end_s=None, interactive_mailbox=str(mailbox)))
            tracker._handle_request(dict(version=1, run_id="run", seq=7, action="goto",
                pose_source="livo_post_vio", goal_xyz_m=[4., 0., 0.]))

            def tick(stamp, position):
                return tracker.tick(current_sim_s=stamp, odom_stamp_s=stamp, position=position,
                    quat_wxyz=[1., 0., 0., 0.], estimated_velocity=None, measured_yaw_rate=0.,
                    ready=True, now_wall=stamp, arrival_wall=stamp, frame_id="camera_init",
                    child_frame_id="body", pose_source="livo_post_vio")

            tick(10., [0., 0., 0.])
            identity = json.loads((mailbox / "planner_request.json").read_text())
            history, expected = [], []
            journal = mailbox / "planner_provenance.jsonl"
            for revision in range(100):
                stamp = 10.1 + revision * .05
                provenance = dict(local_planner="native_SCAN", global_planner="native_PCT_Astar",
                    trace="diagnostic-payload-%03d-" % revision + "x" * 65536,
                    native_reference_content={"points_xyz_m": [[index * .01, .1, 0.] for index in range(64)]})
                payload = dict(identity, revision=revision, status="tracking", source_stamp_s=stamp,
                    points_xyz_m=[[0., 0., 0.], [4., 0., 0.]], provenance=provenance)
                atomic_json(mailbox / "planner_path.json", payload)
                position = [revision * .02, 0., 0.]
                command, metadata = tick(stamp, position)
                self.assertGreater(command[0], 0.)
                history.append(metadata)
                reference = metadata["planner_reference"]
                self.assertEqual(reference["provenance"], {
                    "local_planner": "native_SCAN", "global_planner": "native_PCT_Astar"})
                self.assertEqual(reference["revision"], revision)
                self.assertEqual(reference["source_stamp_s"], stamp)
                self.assertEqual(reference["samples"], 2)
                locator = reference["provenance_record"]
                self.assertEqual(locator["file"], "planner_provenance.jsonl")
                with journal.open("rb") as stream:
                    stream.seek(locator["offset_bytes"])
                    encoded = stream.read(locator["length_bytes"])
                self.assertTrue(encoded.endswith(b"\n"))
                self.assertEqual(hashlib.sha256(encoded).hexdigest(), locator["sha256"])
                record = json.loads(encoded)
                row = dict(version=1, **{key: identity[key] for key in
                    ("run_id", "seq", "frame_id", "child_frame_id", "pose_source")},
                    revision=revision, source_stamp_s=stamp, samples=2, provenance=provenance)
                self.assertEqual(record, row)
                expected.append(row)
                before = journal.stat().st_size
                # Same revision with changed raw bytes remains the same consumed
                # reference; the duplicate pose must not append evidence again.
                atomic_json(mailbox / "planner_path.json", dict(payload, producer_note="same revision"))
                repeated_command, repeated_metadata = tick(stamp, position)
                self.assertEqual(repeated_command, command)
                self.assertEqual(repeated_metadata["planner_reference"], reference)
                self.assertEqual(journal.stat().st_size, before)

            self.assertEqual([json.loads(line) for line in journal.read_text().splitlines()], expected)
            events = [event for event in tracker.events if event["event"] == "native_scan_reference"]
            self.assertEqual(len(events), 100)
            self.assertEqual([event["provenance_record"] for event in events],
                             [row["planner_reference"]["provenance_record"] for row in history])
            self.assertNotIn("diagnostic-payload-", json.dumps(history))
            self.assertNotIn("diagnostic-payload-", json.dumps(events))
            self.assertLess(len(json.dumps(history)), journal.stat().st_size // 10)
            self.assertLess(len(json.dumps(history[-1])), 2 * len(json.dumps(history[0])))
            # Latest raw input may remain cached. Historic full dictionaries
            # must not remain reachable via events, tracker or saved metadata.
            self.assertLessEqual(len(large_diagnostic_values([tracker, history])), 1)


if __name__ == "__main__":
    unittest.main()
