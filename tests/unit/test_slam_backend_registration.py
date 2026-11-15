"""SLAM-BE point-to-plane ICP (R-ICP-*) against the frozen registration.py interface."""
import math
import unittest

import numpy as np

import slam_backend_fixtures as F
from embodied_navigation.localization_mapping.backend.registration import IcpResult, icp_point_to_plane


class RegistrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        world = F.make_world()
        cls.T_world_lidar = F.loop_pose_at(14.0) @ F.T_BODY_LIDAR
        cls.source = world.scan(cls.T_world_lidar, 7)                       # lidar frame
        target = []
        for i, arc in enumerate((12.0, 14.6, 16.0)):                         # a small world-frame submap
            T = F.loop_pose_at(arc) @ F.T_BODY_LIDAR
            target.append(F.transform_points(T, world.scan(T, 30 + i)))
        cls.target = np.concatenate(target)

    def test_r_icp_1_converges_from_0p8m_0p25rad(self):
        # Catches: wrong Jacobian (sign / column order / rotation linearisation) -- ICP then drifts or stalls.
        offsets = [(0.8, 0.0, 0.25), (0.0, 0.8, -0.25), (0.8 / math.sqrt(2), 0.8 / math.sqrt(2), 0.25),
                   (-0.8, 0.0, -0.25)]
        for dx, dy, dyaw in offsets:
            T_init = F.pose(dx, dy) @ self.T_world_lidar @ F.pose(yaw=dyaw)
            t0, r0 = F.pose_error(T_init, self.T_world_lidar)
            self.assertGreater(t0, 0.79)
            self.assertAlmostEqual(r0, 0.25, places=9)
            with self.subTest(offset=(dx, dy, dyaw)):
                result = icp_point_to_plane(self.source, self.target, T_init)
                self.assertIsInstance(result, IcpResult)
                self.assertTrue(result.converged)
                self.assertEqual(result.T_target_source.shape, (4, 4))
                t_err, r_err = F.pose_error(result.T_target_source, self.T_world_lidar)
                self.assertLess(t_err, 0.05)
                self.assertLess(r_err, 0.01)
                self.assertGreater(result.fitness, 0.0)
                self.assertLessEqual(result.fitness, 1.0)
                self.assertTrue(np.isfinite(result.rmse_m))

    def test_r_icp_2_fewer_than_50_points_not_converged(self):
        # Catches: crashing (exception / LinAlgError) on degenerate input instead of reporting converged=False.
        few = self.source[:30]
        for label, src, tgt in (("source", few, self.target), ("target", self.source, few),
                                ("empty", np.zeros((0, 3)), self.target)):
            with self.subTest(label):
                result = icp_point_to_plane(src, tgt, self.T_world_lidar)
                self.assertFalse(result.converged)

    def test_contract_singular_normal_equations_not_converged(self):
        # Catches: raising on a singular J^T J (a single flat plane leaves x, y, yaw unobservable).
        g = np.linspace(-3.0, 3.0, 40)
        X, Y = np.meshgrid(g, g)
        plane = np.stack([X.ravel(), Y.ravel(), np.zeros(X.size)], -1)
        result = icp_point_to_plane(plane, plane, np.eye(4))
        self.assertFalse(result.converged)


if __name__ == "__main__":
    unittest.main()
