"""Equivalence and regression tests for the C++17/Eigen3 SLAM backend accelerator."""

from __future__ import annotations

import unittest
import numpy as np

from embodied_navigation.localization_mapping.backend.accel import (
    cpp_icp_point_to_plane,
    cpp_optimize_pose_graph,
    cpp_voxel_downsample,
    get_accel_lib,
)
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

    def test_voxel_boundary_059_060_matches_python_exactly(self):
        pts = np.array(
            [
                [0.59, 0.59, 0.59],
                [0.60, 0.60, 0.60],
                [-0.59, -0.59, -0.59],
                [-0.60, -0.60, -0.60],
                [0.599999999999, 0.2, -0.2],
            ],
            dtype=np.float64,
        )
        py = voxel_downsample(pts, 0.2, use_cpp=False)
        cpp = voxel_downsample(pts, 0.2, use_cpp=True)
        self.assertEqual(py.shape, cpp.shape)
        np.testing.assert_allclose(cpp, py, atol=1e-15, rtol=0.0)

    def test_accel_rejects_bad_shapes_nonfinite_and_bad_indices(self):
        with self.assertRaises(ValueError):
            cpp_voxel_downsample(np.zeros((4, 2), dtype=np.float64), 0.2)
        with self.assertRaises(ValueError):
            cpp_voxel_downsample(np.array([[0.0, np.nan, 0.0]], dtype=np.float64), 0.2)
        with self.assertRaises(ValueError):
            cpp_voxel_downsample(np.zeros((1, 3), dtype=np.float64), 0.0)
        with self.assertRaises(ValueError):
            cpp_voxel_downsample(np.zeros((1, 3), dtype=np.float64), float("nan"))

        good_pts = np.zeros((10, 3), dtype=np.float64)
        with self.assertRaises(ValueError):
            cpp_icp_point_to_plane(
                np.zeros((10, 2), dtype=np.float64),
                good_pts,
                np.eye(4),
                max_correspondence_m=1.0,
                iterations=5,
                voxel_m=0.2,
                source_up_body=None,
                gravity_sigma_rad=None,
                gravity_ref_point_sigma_m=0.05,
            )
        with self.assertRaises(ValueError):
            cpp_icp_point_to_plane(
                good_pts,
                good_pts,
                np.full((4, 4), np.inf),
                max_correspondence_m=1.0,
                iterations=5,
                voxel_m=0.2,
                source_up_body=None,
                gravity_sigma_rad=None,
                gravity_ref_point_sigma_m=0.05,
            )
        with self.assertRaises(ValueError):
            cpp_icp_point_to_plane(
                good_pts,
                good_pts,
                np.eye(4),
                max_correspondence_m=1.0,
                iterations=5,
                voxel_m=0.2,
                source_up_body=np.array([0.0, np.nan, 1.0]),
                gravity_sigma_rad=0.05,
                gravity_ref_point_sigma_m=0.05,
            )

        with self.assertRaises(ValueError):
            cpp_optimize_pose_graph(
                [np.eye(3)],
                [],
                [],
                [],
                cauchy_scale=1.0,
                iterations=5,
            )
        with self.assertRaises(ValueError):
            cpp_optimize_pose_graph(
                [np.eye(4), np.eye(4)],
                [(0, 5, np.eye(4), np.eye(6), False)],
                [],
                [],
                cauchy_scale=1.0,
                iterations=5,
            )
        with self.assertRaises(ValueError):
            cpp_optimize_pose_graph(
                [np.eye(4), np.eye(4)],
                [(0, 1, np.eye(4), np.full((6, 6), np.nan), False)],
                [],
                [],
                cauchy_scale=1.0,
                iterations=5,
            )

    def test_icp_body_origin_hessian_matches_python_under_rotation_and_translation(self):
        # Place target far from world origin so centroid != body origin and lever arm is large.
        target_origin = _make_room_cloud(23)
        T_true_origin = se3_exp(np.array([0.35, -0.25, 0.12, 0.05, -0.04, 0.65]))
        source = transform_points(np.linalg.inv(T_true_origin), target_origin)
        T_init_origin = se3_exp(np.array([0.15, -0.10, 0.05, 0.01, -0.01, 0.55]))

        T_shift = np.eye(4, dtype=np.float64)
        T_shift[:3, 3] = [25.0, -14.0, 6.0]
        target = target_origin + T_shift[:3, 3]
        T_init = T_shift @ T_init_origin

        r_py = icp_point_to_plane(
            source, target, T_init, max_correspondence_m=1.2, iterations=30, voxel_m=0.2,
            return_hessian=True, use_cpp=False,
        )
        r_cpp = icp_point_to_plane(
            source, target, T_init, max_correspondence_m=1.2, iterations=30, voxel_m=0.2,
            return_hessian=True, use_cpp=True,
        )
        self.assertTrue(r_py.converged and r_cpp.converged)
        self.assertIsNotNone(r_py.hessian)
        self.assertIsNotNone(r_cpp.hessian)
        np.testing.assert_allclose(r_cpp.T_target_source, r_py.T_target_source, atol=1e-5)
        np.testing.assert_allclose(r_cpp.hessian, r_py.hessian, rtol=1e-3, atol=1e-3)

        # Body-origin Hessian must be invariant to shifting the world origin / target centroid.
        r_origin = icp_point_to_plane(
            source, target_origin, T_init_origin, max_correspondence_m=1.2, iterations=30, voxel_m=0.2,
            return_hessian=True, use_cpp=True,
        )
        np.testing.assert_allclose(r_cpp.hessian, r_origin.hessian, rtol=1e-4, atol=1e-3)

    def test_pose_graph_iteration_cap_and_zero_diag_damping(self):
        g = PoseGraph(cauchy_scale=1.0)
        g.add_node(0, np.eye(4))
        g.add_node(1, se3_exp(np.array([1.0, 0.0, 0.5, 0.0, 0.0, 0.2])))
        # Isolated node 1 with only a Z prior (XY + rotation diagonal == 0.0) must not blow up or exceed max_iterations.
        g.add_z_prior(1, target_z_m=0.0, sigma_m=0.05, robust=False)
        summary = g.optimize(iterations=3, use_cpp=True)
        self.assertLessEqual(summary["iterations"], 3)
        self.assertTrue(np.isfinite(g.pose(1)).all())


if __name__ == "__main__":
    unittest.main()
