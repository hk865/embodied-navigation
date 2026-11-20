"""CPU protocol, source freshness and continuous native-SCAN following checks."""
import json
import math
import tempfile
import unittest
from pathlib import Path

from embodied_navigation.simulation.dynanav.pct_scan_waypoints import (
    ContinuousScanFollower, PctScanWaypointTracker, atomic_json,
)


class ContinuousScanTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.box = Path(self.tmp.name)
        self.identity = dict(run_id="run", seq=1, frame_id="camera_init", child_frame_id="body",
            pose_source="livo_post_vio")
        self.follower = ContinuousScanFollower(self.box, self.identity, [4., 0., 0.],
            speed=1., motion_end=100.)

    def path(self, points=None, **changes):
        row = dict(version=1, **self.identity, revision=0, status="tracking",
            source_stamp_s=10., points_xyz_m=points if points is not None else [[i * .1, 0., 0.] for i in range(41)],
            provenance=dict(global_planner="native_PCT_Astar", local_planner="native_SCAN"))
        row.update(changes)
        atomic_json(self.box / "planner_path.json", row)

    def test_dense_path_samples_never_trigger_intermediate_stops(self):
        self.path()
        for i in range(36):
            command = self.follower.tick(10. + i * .1, [i * .1, 0., 0.], 0., .3, 0.)
            self.assertGreater(command[0], 0.)
            self.assertEqual(self.follower.phase, "cruise")
        self.assertFalse(self.follower.arrived)
        self.assertFalse(any(e["event"] == "entered_corner_capture" for e in self.follower.events))

    def test_local_path_tail_is_not_goal_and_does_not_extrapolate(self):
        self.path([[0., 0., 0.], [.5, 0., 0.]])
        self.assertGreater(self.follower.tick(10., [.1, 0., 0.], 0., .2, 0.)[0], 0.)
        self.assertEqual(self.follower.tick(10.1, [.5, 0., 0.], 0., .2, 0.), [0., 0., 0.])
        self.assertFalse(self.follower.arrived)
        self.path([[.4, 0., 0.], [1.5, 0., 0.]], revision=1)
        self.assertGreater(self.follower.tick(10.2, [.5, 0., 0.], 0., .2, 0.)[0], 0.)

    def test_request_accepted_before_map_path_exists(self):
        self.assertEqual(self.follower.tick(10., [0., 0., 0.], 0., 0., 0.), [0., 0., 0.])
        self.assertEqual(self.follower.phase, "planning")
        self.assertIsNone(self.follower.failure)
        request = json.loads((self.box / "planner_request.json").read_text())
        self.assertEqual(request["goal_xyz_m"], [4., 0., 0.])

    def test_wrong_sequence_frame_or_non_native_path_never_drives(self):
        for changes in (dict(seq=2), dict(frame_id="truth"), dict(provenance={"local_planner":"mock"}),
                        dict(points_xyz_m=[[0., 0., 0.], [float("inf"), 0., 0.]])):
            # Nonfinite test cannot use strict atomic writer, simulates malformed producer bytes.
            if "points_xyz_m" in changes:
                (self.box / "planner_path.json").write_text(json.dumps(dict(version=1, **self.identity,
                    revision=0, status="tracking", provenance={"local_planner":"native_SCAN"}, **changes)))
            else:
                self.path(**changes)
            self.assertEqual(self.follower.tick(10., [0., 0., 0.], 0., 0., 0.), [0., 0., 0.])

    def test_final_goal_requires_height_and_measured_stop_dwell(self):
        self.path()
        self.follower.tick(10., [3.9, 0., .5], 0., 0., 0.)
        self.assertFalse(self.follower.arrived)
        for t in (10.1, 10.4):
            self.follower.tick(t, [3.9, 0., 0.], 0., 0., 0.)
            self.assertFalse(self.follower.arrived)
        self.follower.tick(10.7, [3.9, 0., 0.], 0., 0., 0.)
        self.assertTrue(self.follower.arrived)

    def test_spline_replacement_keeps_one_goal_and_forward_motion(self):
        self.path()
        first = self.follower.tick(10., [1., 0., 0.], 0., .2, 0.)
        self.path([[.9, 0., 0.], [2., .1, 0.], [4., 0., 0.]], revision=1)
        second = self.follower.tick(10.1, [1.02, 0., 0.], 0., .2, 0.)
        self.assertGreater(first[0], 0.)
        self.assertGreater(second[0], 0.)
        self.assertEqual(self.follower.revision, 1)
        self.assertLessEqual(abs(second[2]), .15)

    def test_native_emergency_hold_invalidates_previously_executable_path(self):
        self.path()
        self.assertGreater(self.follower.tick(10., [.1, 0., 0.], 0., .2, 0.)[0], 0.)
        self.path(status="hold", reason="native_scan_emergency_stop", revision=1)
        self.assertEqual(self.follower.tick(10.1, [.12, 0., 0.], 0., .2, 0.), [0., 0., 0.])
        self.assertIsNone(self.follower.points)
        self.path(revision=2)
        self.assertGreater(self.follower.tick(10.2, [.12, 0., 0.], 0., .2, 0.)[0], 0.)

    def test_planner_failure_closes_without_a_straight_line_fallback(self):
        self.path(status="failed", reason="pct_no_route")
        self.assertEqual(self.follower.tick(10., [0., 0., 0.], 0., 0., 0.), [0., 0., 0.])
        self.assertEqual(self.follower.failure, "planner_pct_no_route")


class PctScanMailboxTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.box = Path(self.tmp.name) / "run" / "mailbox"
        self.profile = dict(slam_pose_source="livo_post_vio", speed_mps=1., motion_end_s=200.,
            interactive_mailbox=str(self.box))
        self.adapter = PctScanWaypointTracker(self.profile)

    def request(self, seq=1, action="goto"):
        self.adapter._handle_request(dict(version=1, run_id="run", seq=seq, action=action,
            pose_source="livo_post_vio", goal_xyz_m=[4., 0., 0.]))

    def tick(self, stamp=10., **changes):
        args = dict(current_sim_s=stamp, odom_stamp_s=stamp, position=[0., 0., 0.],
            quat_wxyz=[1., 0., 0., 0.], estimated_velocity=None, measured_yaw_rate=0.,
            ready=True, now_wall=stamp, arrival_wall=stamp, frame_id="camera_init", child_frame_id="body",
            pose_source="livo_post_vio")
        args.update(changes)
        return self.adapter.tick(**args)

    def path(self):
        request = json.loads((self.box / "planner_request.json").read_text())
        atomic_json(self.box / "planner_path.json", dict(request, revision=1, status="tracking",
            points_xyz_m=[[0., 0., 0.], [4., 0., 0.]], provenance={"local_planner":"native_SCAN"}))

    def test_same_native_receipts_but_planner_execution_source(self):
        self.request()
        self.assertEqual(self.tick()[0], [0., 0., 0.])
        self.path()
        command, receipt = self.tick(10.1)
        self.assertGreater(command[0], 0.)
        self.assertEqual(receipt["controller"], "pct_global_scan_local")
        self.assertFalse(receipt.get("ground_truth_navigation", False))
        observation = json.loads(self.adapter.observation_path.read_text())
        self.assertEqual(observation["command"]["source"], "pct_global_scan_local")

    def test_stale_pose_zeroes_even_with_good_native_path(self):
        self.request(); self.tick(); self.path()
        self.assertGreater(self.tick(10.1)[0][0], 0.)
        command, receipt = self.tick(10.1, current_sim_s=10.4, now_wall=10.4)
        self.assertEqual(command, [0., 0., 0.])
        self.assertEqual(receipt["reason"], "stale_sim_odometry")

    def test_native_hold_revokes_cached_command_even_on_duplicate_pose(self):
        self.request(); self.tick(); self.path()
        self.assertGreater(self.tick(10.1)[0][0], 0.)
        payload = json.loads((self.box / "planner_path.json").read_text())
        atomic_json(self.box / "planner_path.json", dict(payload, revision=2, status="hold"))
        command, receipt = self.tick(10.1, current_sim_s=10.11, now_wall=10.11)
        self.assertEqual(command, [0., 0., 0.])
        self.assertFalse(receipt["distinct_measurement"])

    def test_cancel_and_stop_clear_execution_without_external_velocity(self):
        self.request(); self.tick(); self.path(); self.tick(10.1)
        self.request(2, "cancel")
        self.assertEqual(self.tick(10.2)[0], [0., 0., 0.])
        self.assertIsNone(self.adapter.active_waypoint)
        self.request(3, "stop")
        self.assertEqual(self.tick(10.3)[0], [0., 0., 0.])
        self.assertEqual(self.adapter.terminal_reason, "interactive_stop")

    def test_post_vio_only(self):
        with self.assertRaises(ValueError):
            PctScanWaypointTracker(dict(self.profile, slam_pose_source="lio_estimate"))


if __name__ == "__main__":
    unittest.main()
