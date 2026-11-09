"""Parking pose source contract: only canonical SLAM feedback may become a pose row."""

import unittest
from embodied_navigation.control.parking.parking_pose10hz import from_feedback, pose


class ParkingSourceContractTests(unittest.TestCase):
    def feedback(self):
        return dict(
            stamp_ns=1000000000,
            frame_id="camera_init",
            position_world_xyz=[1.0, 2.0, 0.3],
            quaternion_wxyz=[1.0, 0.0, 0.0, 0.0],
            origin_velocity_body=[0.0, 0.0, 0.0],
            callback_ros_clock_ns=1010000000,
            received_wall_ns=2000000000,
            navigation_ground_truth_used=False,
            pose_producer="fast_livo2_async_backend",
        )

    def test_real_slam_remains_non_truth_and_retains_clocks(self):
        feedback = self.feedback()
        row = from_feedback(feedback)
        self.assertIs(row["navigation_ground_truth_used"], False)
        self.assertEqual(row["pose_producer"], "fast_livo2_async_backend")
        self.assertEqual(row["stamp_ns"], feedback["stamp_ns"])
        self.assertEqual(row["received_sim_ns"], feedback["callback_ros_clock_ns"])
        self.assertEqual(row["source_received_wall_ns"], feedback["received_wall_ns"])
        self.assertEqual(row["quaternion_xyzw"], [0.0, 0.0, 0.0, 1.0])
        pose(row)

    def test_truth_feedback_is_rejected(self):
        feedback = self.feedback()
        feedback["navigation_ground_truth_used"] = True
        with self.assertRaises(ValueError):
            from_feedback(feedback)

    def test_missing_or_foreign_source_is_rejected(self):
        for field, replacement in [
            ("pose_producer", "gazebo_truth"),
            ("frame_id", "world"),
            ("navigation_ground_truth_used", None),
        ]:
            with self.subTest(field=field):
                feedback = self.feedback()
                feedback[field] = replacement
                with self.assertRaises(ValueError):
                    from_feedback(feedback)
        feedback = self.feedback()
        del feedback["pose_producer"]
        with self.assertRaises(ValueError):
            from_feedback(feedback)


if __name__ == "__main__":
    unittest.main()
