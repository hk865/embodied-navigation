"""Independent source-time velocity-window contracts; no ROS or truth inputs."""
from collections import deque
import tempfile
import unittest
from pathlib import Path

import numpy as np

from embodied_navigation.perception.pose_velocity import update_pose_velocity
from embodied_navigation.simulation.dynanav.native_waypoints import NativeWaypointTracker


class PoseVelocityContract(unittest.TestCase):
    def test_source_time_multiaxis_velocity_requires_full_window(self):
        history = deque()
        origin = np.array([4., -2., .3])
        velocity = np.array([.2, -.1, .05])
        for stamp in [10., 10.1, 10.2, 10.4]:
            result = update_pose_velocity(history, stamp, origin + velocity * (stamp - 10.), .5)
            self.assertIsNone(result)
        result = update_pose_velocity(history, 10.5, origin + velocity * .5, .5)
        np.testing.assert_allclose(result, velocity, atol=1e-12, rtol=0)
        self.assertIsInstance(result, np.ndarray)

    def test_duplicate_does_not_advance_source_window(self):
        history = deque()
        for stamp in [10., 10.1, 10.2]:
            update_pose_velocity(history, stamp, np.array([stamp, 2 * stamp, -stamp]), .5)
        before = [(stamp, point.copy()) for stamp, point in history]
        for _ in range(8):
            self.assertIsNone(update_pose_velocity(history, 10.2, np.array([10.2, 20.4, -10.2]), .5, distinct=False))
        self.assertEqual([p[0] for p in history], [p[0] for p in before])
        for actual, expected in zip(history, before):
            np.testing.assert_array_equal(actual[1], expected[1])

    def test_history_retains_copy_and_original_cutoff_anchor(self):
        history = deque()
        point = np.array([0., 0., 0.])
        update_pose_velocity(history, 10., point, .5)
        point[:] = 100.
        np.testing.assert_array_equal(history[0][1], [0., 0., 0.])
        for stamp in [10.1, 10.2, 10.4, 10.5, 10.6]:
            result = update_pose_velocity(history, stamp, np.array([stamp - 10., 0., 0.]), .5)
        self.assertEqual(history[0][0], 10.1)
        np.testing.assert_allclose(result, [1., 0., 0.], atol=1e-12, rtol=0)

    def test_reset_history_requires_new_full_window(self):
        history = deque()
        update_pose_velocity(history, 20., np.array([0., 0., 0.]), .5)
        self.assertIsNotNone(update_pose_velocity(history, 20.5, np.array([1., 0., 0.]), .5))
        history.clear()
        self.assertIsNone(update_pose_velocity(history, 30., np.array([2., 0., 0.]), .5))
        self.assertIsNone(update_pose_velocity(history, 30.4, np.array([3., 0., 0.]), .5))

    def test_native_scalar_remains_norm_of_source_time_vector(self):
        with tempfile.TemporaryDirectory() as temp:
            box = Path(temp) / 'run' / 'interactive'
            tracker = NativeWaypointTracker(dict(slam_pose_source='livo_post_vio',
                speed_mps=.15, motion_end_s=None, interactive_mailbox=str(box)))
            tracker._handle_request(dict(version=1, run_id='run', seq=1, action='goto',
                pose_source='livo_post_vio', goal_xyz_m=[4., 0., 0.]))
            velocity = np.array([.2, -.1, .05])
            for stamp in [10., 10.1, 10.2, 10.3, 10.4, 10.5]:
                _, receipt = tracker.tick(current_sim_s=stamp, odom_stamp_s=stamp,
                    position=(velocity * (stamp - 10.)).tolist(), quat_wxyz=[1., 0., 0., 0.],
                    estimated_velocity=[0., 0., 0.], measured_yaw_rate=0., ready=True,
                    now_wall=stamp, arrival_wall=stamp, frame_id='camera_init',
                    child_frame_id='body', pose_source='livo_post_vio')
                if stamp < 10.5:
                    self.assertIsNone(receipt['pose_window_speed_mps'])
            self.assertAlmostEqual(receipt['pose_window_speed_mps'], float(np.linalg.norm(velocity)), places=12)


if __name__ == '__main__':
    unittest.main()
