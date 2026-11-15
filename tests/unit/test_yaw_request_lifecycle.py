"""Independent yaw protocol cases frozen before implementation review."""
import tempfile
import unittest

from yaw_acceptance_fixtures import YawRig


class YawRequestLifecycleAcceptance(unittest.TestCase):
    def test_relative_or_absolute_accepted_before_clearance_but_stays_zero(self):
        for fields in (dict(delta_yaw_rad=.2),
                       dict(target_yaw_rad=.2, frame_id="camera_init")):
            with self.subTest(fields=fields), tempfile.TemporaryDirectory() as directory:
                rig = YawRig(directory)
                rig.request(**fields)
                self.assertEqual([row["seq"] for row in rig.events("yaw_accepted")], [1])
                for stamp in (10., 10.01, 10.02):
                    self.assertEqual(rig.tick(stamp)[0], [0., 0., 0.])
                self.assertEqual(rig.events("yaw_complete"), [])
                self.assertIsNone(rig.adapter.terminal_reason)

    def test_missing_ambiguous_nonfinite_and_unframed_absolute_targets_rejected(self):
        cases = ({}, dict(delta_yaw_rad=.1, target_yaw_rad=.2, frame_id="camera_init"),
                 dict(delta_yaw_rad=float("nan")), dict(target_yaw_rad=float("inf"), frame_id="camera_init"),
                 dict(target_yaw_rad=.2))
        for fields in cases:
            with self.subTest(fields=fields), tempfile.TemporaryDirectory() as directory:
                rig = YawRig(directory)
                rig.request(**fields)
                self.assertEqual(rig.events("yaw_accepted"), [])
                self.assertEqual([row["seq"] for row in rig.events("request_rejected")], [1])
                self.assertEqual(rig.tick()[0], [0., 0., 0.])

    def test_absolute_wrong_measured_frame_never_runs_or_completes(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = YawRig(directory)
            rig.request(target_yaw_rad=.2, frame_id="other_map")
            self.assertEqual(rig.tick()[0], [0., 0., 0.])
            self.assertEqual(rig.events("yaw_complete"), [])
            self.assertTrue(rig.events("request_rejected") or rig.events("yaw_failed"))
            self.assertIsNone(rig.adapter.terminal_reason)

    def test_cancel_closes_exact_yaw_and_never_resumes_preempted_goto(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = YawRig(directory)
            rig.request(action="goto", goal_xyz_m=[4., 0., 0.])
            rig.tick(); rig.native_path()
            self.assertGreater(rig.tick(10.01)[0][0], 0.)
            rig.request(2, delta_yaw_rad=.2)
            self.assertEqual([row["seq"] for row in rig.events("yaw_accepted")], [2])
            self.assertEqual(rig.tick(10.02)[0], [0., 0., 0.])
            rig.request(3, action="cancel")
            self.assertEqual([row["seq"] for row in rig.events("action_cancelled")], [2])
            for index in range(1, 10):
                self.assertEqual(rig.tick(10.02 + index * .01)[0], [0., 0., 0.])
            self.assertIsNone(rig.adapter.terminal_reason)
            self.assertIsNone(rig.adapter.active_waypoint)
            self.assertEqual(rig.events("yaw_complete"), [])

    def test_stop_with_yaw_pending_uses_original_terminal_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = YawRig(directory)
            rig.request(delta_yaw_rad=.2)
            self.assertEqual([row["seq"] for row in rig.events("yaw_accepted")], [1])
            rig.tick()
            rig.request(2, action="stop")
            self.assertEqual(rig.tick(10.01)[0], [0., 0., 0.])
            self.assertEqual(rig.adapter.terminal_reason, "interactive_stop")
            self.assertTrue(rig.adapter.arrived)
            rig.request(3, delta_yaw_rad=-.2)
            self.assertEqual(rig.tick(10.02)[0], [0., 0., 0.])
            self.assertEqual([row["seq"] for row in rig.events("yaw_accepted")], [1])


    def test_yaw_own_ttl_failure_returns_idle_without_ending_the_run(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = YawRig(directory)
            rig.request(delta_yaw_rad=.2)
            self.assertEqual([row["seq"] for row in rig.events("yaw_accepted")], [1])
            rig.tick()
            self.assertEqual(rig.tick(40.01)[0], [0., 0., 0.])
            failures = rig.events("yaw_failed")
            self.assertEqual([row["seq"] for row in failures], [1])
            self.assertEqual(failures[0]["reason"], "yaw_ttl_expired")
            self.assertIsNone(rig.adapter.terminal_reason)
            self.assertEqual(rig.events("yaw_complete"), [])


if __name__ == "__main__":
    unittest.main()
