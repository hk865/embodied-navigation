"""Real controller mailbox boundary with scoped native-clearance responses."""
import copy
import math
import tempfile
import unittest

from yaw_acceptance_fixtures import YawRig


class YawClearanceFeedbackAcceptance(unittest.TestCase):
    def test_absolute_and_relative_freeze_on_first_new_measured_pose(self):
        for fields in (dict(delta_yaw_rad=.07),
                       dict(target_yaw_rad=.37, frame_id="camera_init")):
            with self.subTest(fields=fields), tempfile.TemporaryDirectory() as directory:
                rig = YawRig(directory)
                rig.tick(9.9, yaw=-1.)
                rig.request(**fields)
                self.assertEqual(rig.tick(10., yaw=.3)[0], [0., 0., 0.])
                query = rig.query()
                self.assertEqual(query["body_stamp_ns"], 10_000_000_000)
                self.assertAlmostEqual(query["target_yaw_rad"], .37, places=12)
                rig.feedback()
                command, _ = rig.tick(10.01, yaw=.3)
                self.assertEqual(command[:2], [0., 0.])
                self.assertAlmostEqual(command[2], .084, places=12)

    def test_matching_query_refreshes_body_source_without_reissuing_target(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = YawRig(directory); rig.request(delta_yaw_rad=.3); rig.tick()
            original = rig.query()
            for index in range(1, 10):
                rig.feedback()
                command, _ = rig.tick(10.+index*.01, yaw=index*.005)
                self.assertEqual(command[:2], [0., 0.]); self.assertGreater(command[2], 0.)
                self.assertEqual(rig.query(), original)
            self.assertGreater(rig.feedback()["body_stamp_ns"], original["body_stamp_ns"])

    def test_bad_or_pending_feedback_revokes_old_clear_even_on_duplicate_pose(self):
        mutations = [
            ("other_run", lambda row: row.update(run_id="other")),
            ("old_action", lambda row: row.update(seq=0)),
            ("other_query", lambda row: row.update(query_id=row["query_id"]+1)),
            ("other_target", lambda row: row.update(target_yaw_rad=row["target_yaw_rad"]+.01)),
            ("other_frame", lambda row: row["source"].update(body_frame_id="other")),
            ("other_child", lambda row: row["source"].update(body_child_frame_id="other")),
            ("stamp_disagrees", lambda row: row["source"].update(body_stamp_ns=row["body_stamp_ns"]-1)),
            ("before_first_query", lambda row: (row.update(body_stamp_ns=9_000_000_000), row["source"].update(body_stamp_ns=9_000_000_000))),
            ("future_body", lambda row: (row.update(body_stamp_ns=11_000_000_000), row["source"].update(body_stamp_ns=11_000_000_000))),
            ("future_receipt", lambda row: row.update(received_wall_s=11.)),
            ("old_receipt", lambda row: row.update(received_wall_s=9.)),
            ("pending", lambda row: row.update(status="pending")),
            ("clear_but_occupied", lambda row: row.update(occupancy=1)),
        ]
        with tempfile.TemporaryDirectory() as directory:
            rig = YawRig(directory); rig.request(delta_yaw_rad=.3); rig.tick()
            rig.feedback(); self.assertGreater(rig.tick(10.01)[0][2], 0.)
            for label, mutate in mutations:
                with self.subTest(label=label):
                    good = rig.feedback()
                    self.assertGreater(rig.tick(10.01)[0][2], 0.)
                    invalid = copy.deepcopy(good); mutate(invalid); rig.write_feedback(invalid)
                    self.assertEqual(rig.tick(10.01)[0], [0., 0., 0.])
                    self.assertEqual(rig.events("yaw_complete"), [])
                    self.assertEqual(rig.events("yaw_failed"), [])

    def test_source_older_than_point_two_five_holds_without_adding_pose_drift_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = YawRig(directory); rig.request(delta_yaw_rad=.3); rig.tick()
            rig.feedback(); rig.tick(10.1)
            old = rig.feedback()
            # Still after the original request, and wall arrival is current;
            # only the existing physical source-age limit rejects this result.
            old["received_wall_s"] = 10.4
            rig.write_feedback(old)
            self.assertEqual(rig.tick(10.4)[0], [0., 0., 0.])
            self.assertEqual(rig.events("yaw_failed"), [])

    def test_matching_blocked_or_failed_closes_exact_yaw_and_late_clear_cannot_revive(self):
        for status in ("blocked", "failed"):
            with self.subTest(status=status), tempfile.TemporaryDirectory() as directory:
                rig = YawRig(directory); rig.request(delta_yaw_rad=.3); rig.tick()
                good = rig.feedback()
                rejected = dict(good, status=status, reason="native_rotation_"+status,
                    occupancy=1 if status=="blocked" else -1, first_blocking_yaw_rad=.1)
                rig.write_feedback(rejected)
                self.assertEqual(rig.tick(10.01)[0], [0., 0., 0.])
                self.assertEqual([row["seq"] for row in rig.events("yaw_failed")], [1])
                self.assertIsNone(rig.adapter.terminal_reason)
                rig.write_feedback(good)
                for index in range(2, 10):
                    self.assertEqual(rig.tick(10.+index*.01)[0], [0., 0., 0.])
                self.assertEqual(rig.events("yaw_complete"), [])

    def test_old_action_feedback_after_cancel_does_not_release_new_action(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = YawRig(directory); rig.request(delta_yaw_rad=.3); rig.tick()
            old = rig.feedback()
            rig.request(2, action="cancel"); self.assertEqual(rig.tick(10.01)[0], [0., 0., 0.])
            rig.request(3, delta_yaw_rad=-.3); self.assertEqual(rig.tick(10.02)[0], [0., 0., 0.])
            rig.write_feedback(old)
            self.assertEqual(rig.tick(10.03)[0], [0., 0., 0.])
            rig.feedback()
            self.assertLess(rig.tick(10.04)[0][2], 0.)
            self.assertEqual([row["seq"] for row in rig.events("action_cancelled")], [1])

    def test_real_adapter_emits_measured_completion_after_source_dwell(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = YawRig(directory); rig.request(delta_yaw_rad=.03); rig.tick()
            for index in range(1, 52):
                rig.feedback()
                command, _ = rig.tick(10.+index*.01, measured_yaw_rate=.029)
                self.assertEqual(command[:2], [0., 0.])
                if index < 51:
                    self.assertEqual(rig.events("yaw_complete"), [])
            self.assertEqual(command, [0., 0., 0.])
            completed = rig.events("yaw_complete")
            self.assertEqual(len(completed), 1)
            receipt = completed[0]
            self.assertEqual(receipt["seq"], 1)
            self.assertAlmostEqual(receipt["target_yaw_rad"], .03, places=12)
            self.assertAlmostEqual(receipt["yaw_rad"], 0., places=12)
            self.assertAlmostEqual(receipt["yaw_error_rad"], .03, places=12)
            self.assertEqual(receipt["pose_source"], "livo_post_vio")
            self.assertEqual(receipt["frame_id"], "camera_init")
            self.assertEqual(receipt["imu_frame"], "go2_imu")
            self.assertAlmostEqual(receipt["measured_yaw_rate_rps"], .029, places=12)
            self.assertIsNone(rig.adapter.terminal_reason)

    def test_gyro_loss_resets_adapter_dwell_and_duplicate_poses_never_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            rig = YawRig(directory); rig.request(delta_yaw_rad=.01); rig.tick()
            for index in range(1, 31):
                rig.feedback(); rig.tick(10.+index*.01)
            rig.feedback()
            self.assertEqual(rig.tick(10.31, imu_valid=False)[0], [0., 0., 0.])
            for index in range(32, 71):
                rig.feedback(); rig.tick(10.+index*.01)
                self.assertEqual(rig.events("yaw_complete"), [])
        with tempfile.TemporaryDirectory() as directory:
            rig = YawRig(directory); rig.request(delta_yaw_rad=.01); rig.tick()
            rig.feedback()
            for index in range(1, 61):
                clock = 10.+index*.01
                command, _ = rig.tick(10., current_sim_s=clock, now_wall=clock,
                    imu_source_stamp_s=clock, imu_arrival_wall_s=clock)
                self.assertEqual(command[:2], [0., 0.])
                self.assertEqual(rig.events("yaw_complete"), [])


if __name__ == "__main__":
    unittest.main()
