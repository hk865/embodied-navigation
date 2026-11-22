"""Equivalence and regression tests for the C++17/Eigen3 SLAM backend accelerator."""

from __future__ import annotations

import unittest
import numpy as np

from embodied_navigation.localization_mapping.backend.accel import get_accel_lib
from embodied_navigation.localization_mapping.backend.keyframes import transform_points, voxel_downsample
from embodied_navigation.localization_mapping.backend.pose_graph import PoseGraph, se3_exp
from embodied_navigation.localization_mapping.backend.registration import icp_point_to_plane


def _make_room_cloud(seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    pts = []
    # Floor (z = 0)
    pts.append(np.column_stack([rng.uniform(-4, 4, 800), rng.uniform(-4, 4, 800), np.zeros(800)]))
    # Ceiling (z = 3)
    pts.append(np.column_stack([rng.uniform(-4, 4, 500), rng.uniform(-4, 4, 500), np.full(500, 3.0)]))
    # Wall x = 4
    pts.append(np.column_stack([np.full(600, 4.0), rng.uniform(-4, 4, 600), rng.uniform(0, 3, 600)]))
    # Wall y = -4
    pts.append(np.column_stack([rng.uniform(-4, 4, 600), np.full(600, -4.0), rng.uniform(0, 3, 600)]))
    return np.vstack(pts)


class CppAccelEquivalenceTests(unittest.TestCase):
    def test_cpp_accelerator_compiles_and_loads(self):
        lib = get_accel_lib()
        self.assertIsNotNone(lib, "C++17/Eigen3 accelerator must compile and load on this host")

    def test_voxel_downsample_matches_python_exactly(self):
        pts = _make_room_cloud(7)
        py = voxel_downsample(pts, 0.2, use_cpp=False)
        cpp = voxel_downsample(pts, 0.2, use_cpp=True)
        self.assertEqual(py.shape, cpp.shape)
        np.testing.assert_allclose(cpp, py, atol=1e-12, rtol=1e-12)

    def test_icp_point_to_plane_matches_python_with_gravity_and_hessian(self):
        target = _make_room_cloud(11)
        T_true = se3_exp(np.array([0.35, -0.25, 0.12, 0.02, -0.03, 0.18]))
        source = transform_points(np.linalg.inv(T_true), target)
        T_init = se3_exp(np.array([0.1, -0.1, 0.02, 0.0, 0.0, 0.08]))
        up_body = np.array([0.02, -0.03, 0.9993])
        up_body /= np.linalg.norm(up_body)

        r_py = icp_point_to_plane(
            source, target, T_init,
            max_correspondence_m=1.0, iterations=30, voxel_m=0.2,
            source_up_body=up_body, gravity_sigma_rad=0.02,
            use_cpp=False,
        )
        r_cpp = icp_point_to_plane(
            source, target, T_init,
            max_correspondence_m=1.0, iterations=30, voxel_m=0.2,
            source_up_body=up_body, gravity_sigma_rad=0.02,
            return_hessian=True, use_cpp=True,
        )
        self.assertTrue(r_py.converged)
        self.assertTrue(r_cpp.converged)
        self.assertAlmostEqual(r_cpp.fitness, r_py.fitness, places=6)
        self.assertAlmostEqual(r_cpp.rmse_m, r_py.rmse_m, places=6)
        np.testing.assert_allclose(r_cpp.T_target_source, r_py.T_target_source, atol=1e-5)
        self.assertIsNotNone(r_cpp.hessian)
        self.assertEqual(r_cpp.hessian.shape, (6, 6))
        np.testing.assert_allclose(r_cpp.hessian, r_cpp.hessian.T, atol=1e-9)

    def test_pose_graph_optimize_matches_python_with_edges_gravity_and_z_priors(self):
        def build_graph():
            g = PoseGraph(cauchy_scale=2.0)
            info_odom = np.diag([400.0, 400.0, 400.0, 1000.0, 1000.0, 1000.0])
            info_loop = np.diag([25.0, 25.0, 25.0, 250.0, 250.0, 250.0])
            T = np.eye(4)
            step = se3_exp(np.array([1.0, 0.0, 0.01, 0.0, 0.0, 0.314159265]))
            for i in range(20):
                g.add_node(i, T)
                if i > 0:
                    g.add_edge(i - 1, i, step, info_odom, robust=False)
                if i % 4 == 0:
                    g.add_gravity_prior(i, np.array([0.0, 0.0, 1.0]), sigma_rad=0.05, robust=True)
                if i >= 10:
                    g.add_z_prior(i, target_z_m=0.0, sigma_m=0.08, robust=True)
                T = T @ step
            # Loop closure back to node 0 with small perturbation
            g.add_edge(0, 19, se3_exp(np.array([-0.9, 0.1, -0.05, 0.01, -0.01, -0.31])), info_loop, robust=True)
            return g

        g_py = build_graph()
        g_cpp = build_graph()
        s_py = g_py.optimize(iterations=20, use_cpp=False)
        s_cpp = g_cpp.optimize(iterations=20, use_cpp=True)
        self.assertTrue(s_py["converged"])
        self.assertTrue(s_cpp["converged"])
        self.assertAlmostEqual(s_cpp["error_before"], s_py["error_before"], places=6)
        self.assertAlmostEqual(s_cpp["error_after"], s_py["error_after"], places=6)
        for i in range(20):
            np.testing.assert_allclose(g_cpp.pose(i), g_py.pose(i), atol=1e-6)


if __name__ == "__main__":
    unittest.main()
