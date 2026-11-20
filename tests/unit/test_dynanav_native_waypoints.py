"""CPU-only native waypoint protocol and measured-source regression tests."""
import json
import tempfile
import unittest
from pathlib import Path

from embodied_navigation.navigation.robogauge.route_tracker import RouteTracker
from embodied_navigation.simulation.dynanav.native_waypoints import NativeWaypointTracker


class NativeWaypointsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.mailbox = Path(self.tmp.name) / "run" / "mailbox"
        self.profile = dict(slam_pose_source="lio_estimate", speed_mps=.15,
            motion_end_s=200., interactive_mailbox=str(self.mailbox),
            interactive_require_perception=True)
        self.adapter = NativeWaypointTracker(self.profile)

    def request(self, seq=1, **changes):
        payload = dict(version=1, run_id="run", seq=seq, action="goto",
            pose_source="lio_estimate", goal_xyz_m=[2., 0., 0.])
        payload.update(changes)
        self.adapter._handle_request(payload)

    def tick(self, stamp=10., position=None, **changes):
        args = dict(current_sim_s=stamp, odom_stamp_s=stamp,
            position=position or [0., 0., 0.], quat_wxyz=[1., 0., 0., 0.],
            estimated_velocity=None, measured_yaw_rate=0., ready=True,
            now_wall=stamp, arrival_wall=stamp, frame_id="camera_init",
            child_frame_id="body", pose_source="lio_estimate")
        args.update(changes)
        return self.adapter.tick(**args)

    def receipts(self):
        return [json.loads(line) for line in self.adapter.receipts_path.read_text().splitlines()]

    def test_native_commands_are_identical_without_perception_floor_or_heartbeat(self):
        self.request(created_wall_s=-1000., ttl_s=.01)
        native = RouteTracker([[2., 0., 0.]], [0., 0.], speed=.15, motion_end=200.)
        for i in range(20):
            stamp = 10. + i * .1
            p = [i * .01, 0., 0.]
            command, receipt = self.tick(stamp, p)
            speed = receipt["pose_window_speed_mps"]
            expected = native.tick(stamp, p, 0., .04 if speed is None else speed, 0.)
            self.assertEqual(command, expected)
        self.assertGreater(command[0], 0.)
        self.assertFalse((self.mailbox / "heartbeat.json").exists())

    def test_duplicate_source_does_not_advance_dwell(self):
        self.request(goal_xyz_m=[.25, 0., 0.])
        self.tick()
        self.tick(10.1, [.1, 0., 0.])
        tracker = self.adapter.tracker
        before = (tracker.phase, tracker.since, len(tracker.events))
        for i in range(5):
            self.tick(10.1, [.1, 0., 0.], current_sim_s=10.11+i*.01, now_wall=10.11+i*.01)
        self.assertEqual((tracker.phase, tracker.since, len(tracker.events)), before)
        self.assertFalse(self.adapter.arrived)

    def test_stale_source_holds_zero_and_keeps_goal(self):
        self.request()
        self.assertGreater(self.tick()[0][0], 0.)
        command, row = self.tick(10., current_sim_s=10.4, now_wall=10.4)
        self.assertEqual(command, [0., 0., 0.])
        self.assertEqual(row["reason"], "stale_sim_odometry")
        self.assertIsNotNone(self.adapter.active_waypoint)

    def test_sequence_malformed_and_identity_rejections(self):
        self.request()
        self.request()
        self.request(2, goal_xyz_m=[float("nan"), 0., 0.])
        self.request(3, run_id="other")
        self.request(4, pose_source="native_truth")
        self.adapter.request_path.write_text("{broken")
        self.adapter.poll_request()
        reasons = [r.get("reason") for r in self.receipts()]
        self.assertIn("duplicate_or_stale_sequence", reasons)
        self.assertIn("run_id_mismatch", reasons)
        self.assertIn("pose_source_mismatch", reasons)
        self.assertIn("invalid_json", reasons)
        self.assertEqual(len(self.adapter._pending), 1)

    def test_arrival_returns_idle_and_accepts_next_goal_then_cancel_stop(self):
        self.request(goal_xyz_m=[.25, 0., 0.])
        self.tick()
        for i in range(1, 30):
            self.tick(10.+i*.1, [.1, 0., 0.])
        self.assertEqual(self.adapter.phase, "idle")
        self.assertEqual(self.adapter.stage, 1)
        self.assertFalse(self.adapter.arrived)
        self.assertIsNone(self.adapter.terminal_reason)
        self.request(2)
        self.tick(13., [.1, 0., 0.])
        self.assertIsNotNone(self.adapter.tracker)
        self.request(3, action="cancel")
        self.assertEqual(self.tick(13.1)[0], [0., 0., 0.])
        self.assertEqual(self.adapter.phase, "idle")
        self.request(4, action="stop")
        self.assertTrue(self.adapter.arrived)
        self.assertEqual(self.adapter.terminal_reason, "interactive_stop")

    def test_pose_frame_source_and_quaternion_validation(self):
        self.request()
        self.tick()
        for changes in [dict(frame_id="other"), dict(pose_source="native_truth"),
                        dict(quat_wxyz=[0., 0., 0., 0.]), dict(measured_yaw_rate=float("nan"))]:
            command, receipt = self.tick(10.1, **changes)
            self.assertEqual(command, [0., 0., 0.])
            self.assertEqual(receipt["status"], "hold")

    def test_observation_contains_live_measured_pose_and_goal(self):
        self.request()
        self.tick()
        data = json.loads(self.adapter.observation_path.read_text())
        self.assertEqual(data["pose"]["position_xyz_m"], [0., 0., 0.])
        self.assertEqual(data["pose"]["stamp_ns"], 10_000_000_000)
        self.assertEqual(data["active_waypoint"]["seq"], 1)
        self.assertIsInstance(data["command"]["last_request"], list)
        self.assertEqual(data["waypoints"]["distance_to_goal_m"], 2.)
        self.assertEqual(data["waypoints"]["active_goal_xyz_m"], [2., 0., 0.])

    def test_native_stall_fails_only_this_goal_and_holds_until_new_choice(self):
        self.request()
        self.tick()
        for i in range(1, 82):
            command, row = self.tick(10. + i * .1)
        self.assertEqual(command, [0., 0., 0.])
        self.assertEqual(self.adapter.phase, "idle")
        self.assertIsNone(self.adapter.failure)
        self.assertIsNone(self.adapter.terminal_reason)
        self.assertFalse(self.adapter.arrived)
        self.assertIsNone(self.adapter.tracker)
        failures = [r for r in self.receipts() if r["event"] == "waypoint_failed"]
        self.assertEqual(len(failures), 1)
        self.assertEqual(failures[0]["reason"], "route_low_progress_stall")
        self.assertEqual(failures[0]["command"], [0., 0., 0.])
        self.assertTrue(failures[0]["run_continues"])
        for i in range(82, 150):
            command, row = self.tick(10. + i * .1)
            self.assertEqual(command, [0., 0., 0.])
            self.assertEqual(row["status"], "idle")
        observation = json.loads(self.adapter.observation_path.read_text())
        self.assertTrue(observation["pose"]["fresh"])
        self.assertEqual(observation["waypoints"]["failed_goals"], 1)
        self.assertEqual(observation["waypoints"]["last_failure"], failures[0])
        self.request(2, goal_xyz_m=[.25, 0., 0.])
        self.tick(25.)
        for i in range(1, 30):
            self.tick(25. + i * .1, [.1, 0., 0.])
        self.assertEqual(self.adapter.stage, 1)
        self.assertEqual(self.adapter.phase, "idle")
        self.assertEqual([r["seq"] for r in self.receipts() if r["event"] == "waypoint_reached"], [2])
        self.request(3, action="stop")
        self.assertTrue(self.adapter.arrived)

    def test_native_cross_track_failure_drops_old_queue_without_auto_motion(self):
        self.request()
        self.tick()
        self.request(2, goal_xyz_m=[3., 0., 0.])
        command, row = self.tick(10.1, [0., .61, 0.])
        self.assertEqual(command, [0., 0., 0.])
        self.assertEqual(row["reason"], "waypoint_failed")
        self.assertIsNone(row["failure"])
        self.assertEqual(self.receipts()[-2]["reason"], "route_cross_track_bound")
        self.assertEqual(self.receipts()[-1]["event"], "waypoint_dropped")
        self.assertEqual(self.receipts()[-1]["seq"], 2)
        self.assertEqual(self.tick(10.2, [0., .61, 0.])[0], [0., 0., 0.])
        self.request(3, goal_xyz_m=[2., .61, 0.])
        self.assertGreater(self.tick(10.3, [0., .61, 0.])[0][0], 0.)
        self.assertIsNone(self.adapter.failure)

    def test_motion_budget_remains_a_run_failure_not_a_recoverable_goal(self):
        self.request()
        self.tick()
        self.assertEqual(self.tick(200.)[0], [0., 0., 0.])
        self.assertEqual(self.adapter.failure, "route_motion_budget")
        self.assertEqual(self.receipts()[-1]["event"], "terminal")
        self.request(2)
        self.assertEqual(self.receipts()[-1]["reason"], "run_terminal")

    def test_source_time_budget_inside_future_tolerance_still_ends_run(self):
        self.request()
        self.tick()
        command, row = self.tick(200., current_sim_s=199.999999)
        self.assertEqual(command, [0., 0., 0.])
        self.assertEqual(row["status"], "terminal")
        self.assertEqual(self.adapter.failure, "route_motion_budget")
        self.assertEqual(self.receipts()[-1]["event"], "terminal")
        self.assertFalse(any(r["event"] == "waypoint_failed" for r in self.receipts()))


if __name__ == "__main__":
    unittest.main()
