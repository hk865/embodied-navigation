"""NavigationTask contract tests with synthetic observations and a fixed predicate."""

import unittest
import numpy as np
from embodied_navigation.navigation.task import NavigationTask, Observation


def observation(t, xy=(0, 0), yaw=0, obstacles=()):
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return Observation(
        t, t, np.r_[xy, 0.35], R, np.zeros(3), yaw, np.asarray(obstacles).reshape(-1, 3)
    )


class TaskTests(unittest.TestCase):
    def test_wait_drive_and_stale_failure(self):
        task = NavigationTask([[0, 0], [6, 0]], lambda points: True)
        self.assertEqual(task.step(8, 8)[1]["reason"], "initializing")
        task.update(observation(8))
        self.assertGreater(task.step(8, 8)[0][0], 0)
        self.assertEqual(task.step(8.3, 8.3)[1]["reason"], "freshness_hold")
        task.step(9.4, 9.4)
        self.assertEqual(task.failure, "persistent_SLAM_or_depth_staleness")

    def test_terminal_time_budget(self):
        task = NavigationTask([[0, 0], [6, 0]], lambda points: True)
        task.update(observation(8))
        cmd, _ = task.step(270, 270)
        np.testing.assert_array_equal(cmd, [0, 0, 0])
        self.assertEqual(task.failure, "motion_time_budget_exhausted")

    def test_settle_then_complete(self):
        task = NavigationTask([[0, 0], [6, 0]], lambda points: True)
        for t in np.arange(8, 9, 0.05):
            task.update(observation(float(t), (6, 0)))
            task.step(float(t), float(t))
        self.assertTrue(task.arrived)
        self.assertEqual(task.phase, "done")
        self.assertEqual(len(task.stages), 1)

    def test_turn_between_waypoints(self):
        task = NavigationTask([[0, 0], [6, 0], [0, 0]], lambda points: True)
        for t in np.arange(8, 9, 0.05):
            task.update(observation(float(t), (6, 0)))
            task.step(float(t), float(t))
        self.assertEqual(task.phase, "turning")
        self.assertAlmostEqual(task.turn_target, np.pi)
        self.assertFalse(task.arrived)

    def test_live_depth_collision_hold(self):
        task = NavigationTask([[0, 0], [6, 0]], lambda points: True)
        task.update(observation(8, obstacles=[[0.4, 0, 0.6]]))
        cmd, diag = task.step(8, 8)
        self.assertEqual(diag["reason"], "live_depth_collision_hold")
        np.testing.assert_array_equal(cmd, [0, 0, 0])

    def test_nonfinite_raw_points_cannot_hide_after_height_filter(self):
        from dataclasses import replace

        task = NavigationTask([[0, 0], [6, 0]], lambda points: True)
        task.update(replace(observation(8), sensor_points_finite=False))
        self.assertEqual(task.failure, "invalid_sensor_or_odometry")

    def test_jump_and_uncertified_route(self):
        task = NavigationTask([[0, 0], [6, 0]], lambda points: True)
        task.update(observation(8))
        task.update(observation(8.02, (1, 0)))
        self.assertEqual(task.failure, "severe_relative_odometry_jump")
        with self.assertRaises(ValueError):
            NavigationTask([[0, 0], [6, 0]], lambda points: False)


if __name__ == "__main__":
    unittest.main()
