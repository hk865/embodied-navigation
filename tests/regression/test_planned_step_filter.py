import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from embodied_navigation.control.tracking.planned_step_filter import PlannedStepObstacle


class PlannedStepFlows(unittest.TestCase):
    def setUp(self):
        self.position = np.array([0.0, 0.0, 0.32])
        self.rotation = Rotation.from_euler("y", 0.30).as_matrix()
        self.tread = np.c_[np.full(6, 0.43), np.linspace(-0.1, 0.1, 6), np.full(6, 0.137)]
        self.route = np.array(
            [[0, 0, 0.32], [0.2, 0, 0.32], [0.3, 0, 0.457], [0.8, 0, 0.457], [1.2, 0, 0.457]]
        )

    def test_planned_tread_no_emergency_brake_but_wall_still_blocks(self):
        check = PlannedStepObstacle(self.route)
        self.assertFalse(check.obstacle(self.tread, self.position, self.rotation))
        self.assertEqual(check.ignored, 6)
        wall = self.tread.copy()
        wall[:, 2] = 0.5
        self.assertTrue(check.obstacle(np.r_[self.tread, wall], self.position, self.rotation))
        self.assertEqual(check.remaining, 6)

    def test_flat_obstacles_unchanged_and_other_floor_cannot_explain_lower_obstacle(self):
        check = PlannedStepObstacle(
            [[0, 0, 0.32], [0.5, 0, 0.32], [1, 0, 0.32], [1, 0, 1.82], [0, 0, 1.82]]
        )
        self.assertTrue(check.obstacle(self.tread, self.position, self.rotation))
        self.assertEqual(check.ignored, 0)
        cloud = np.array(
            [
                [x, y, z]
                for x in [0.05, 0.2, 0.5, 0.6]
                for y in [-0.3, 0, 0.3]
                for z in [0.1, 0.3, 0.7]
            ]
        )
        local = (cloud - self.position) @ self.rotation
        original = (
            (local[:, 0] > 0.10)
            & (local[:, 0] < 0.55)
            & (abs(local[:, 1]) < 0.27)
            & (local[:, 2] > -0.10)
            & (local[:, 2] < 0.50)
        )
        self.assertEqual(
            check.obstacle(cloud, self.position, self.rotation), int(original.sum()) >= 4
        )
        self.assertEqual(check.ignored, 0)

    def test_known_support_surface_at_low_posture_is_not_wall(self):
        check = PlannedStepObstacle([[0, 0, 0.32], [0.8, 0, 0.32], [1.6, 0, 0.32]])
        body = np.array([0.0, 0.0, 0.18])
        ground = self.tread.copy()
        ground[:, 2] = 0.0
        self.assertFalse(check.obstacle(ground, body, self.rotation))
        self.assertEqual(check.ignored, 6)
        wall = ground.copy()
        wall[:, 2] = 0.3
        self.assertTrue(check.obstacle(np.r_[ground, wall], body, self.rotation))
        self.assertEqual(check.remaining, 6)

    def test_unknown_lateral_region_keeps_original_stop(self):
        check = PlannedStepObstacle(self.route)
        position = np.array([0.0, 2.0, 0.32])
        cloud = self.tread + [0, 2, 0]
        self.assertTrue(check.obstacle(cloud, position, self.rotation))
        self.assertEqual(check.ignored, 0)


if __name__ == "__main__":
    unittest.main()
