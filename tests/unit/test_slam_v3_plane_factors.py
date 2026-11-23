"""Unit tests for wide-span floor-plane leveling, floor-normal gravity, fine ICP, and X-ICP."""

from __future__ import annotations

import math
from pathlib import Path
import unittest

import numpy as np

from embodied_navigation.localization_mapping.backend.backend import (
    BackendConfig,
    SlamBackend,
)
from embodied_navigation.localization_mapping.backend.floor_plane import (
    detect_floor_plane,
    project_coplanar_odom_delta,
)
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe, transform_points
from embodied_navigation.localization_mapping.backend.pose_graph import se3_exp
from embodied_navigation.localization_mapping.backend.registration import icp_point_to_plane


def _rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def _world_floor_cloud(seed: int = 0, *, floor_z: float = -0.34, span: float = 8.0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    xs = rng.uniform(-span, span, 600)
    ys = rng.uniform(-span, span, 600)
    keep = np.hypot(xs, ys) >= 1.1
    floor = np.column_stack([xs[keep], ys[keep], np.full(int(keep.sum()), floor_z)])
    walls = np.vstack([
        np.column_stack([np.full(200, span), rng.uniform(-span, span, 200), rng.uniform(floor_z + 0.15, 2.5, 200)]),
        np.column_stack([rng.uniform(-span, span, 200), np.full(200, -span), rng.uniform(floor_z + 0.15, 2.5, 200)]),
    ])
    return np.vstack([floor, walls])


class WideFloorPlaneTests(unittest.TestCase):
    def test_detect_floor_plane_recovers_true_normal_under_go2_pitch(self):
        # Simulate Go2 pitched 7 deg nose-down on a flat horizontal floor (z_world = -0.34 m)
        R_true = _rot_y(7.0)
        pts_world = _world_floor_cloud(1, floor_z=-0.34)
        pts_body = pts_world @ R_true  # R_true^T @ p_world
        # Even with 1.0 deg IMU odom error in R_odom, fitted body normal matches true gravity < 0.05 deg!
        R_odom = _rot_y(8.0)
        meas = detect_floor_plane(pts_body, R_odom)
        self.assertIsNotNone(meas)
        gt_up_body = R_true.T @ np.array([0.0, 0.0, 1.0])
        ang_deg = math.degrees(math.acos(np.clip(float(meas.normal_body @ gt_up_body), -1.0, 1.0)))
        self.assertLess(ang_deg, 0.05)
        self.assertAlmostEqual(meas.height_below_body_m, 0.34, delta=0.005)

    def test_detect_floor_plane_rejects_steep_stairs_and_narrow_steps(self):
        rng = np.random.default_rng(2)
        # Steep 28 deg stair ramp
        x = rng.uniform(1.0, 6.0, 500)
        y = rng.uniform(-1.5, 1.5, 500)
        z = -0.34 + np.tan(math.radians(28.0)) * x
        pts_stairs = np.column_stack([x, y, z])
        self.assertIsNone(detect_floor_plane(pts_stairs, np.eye(3)))
        # Too tilted body (> 14 deg)
        self.assertIsNone(detect_floor_plane(_world_floor_cloud(3), _rot_y(20.0)))

    def test_project_coplanar_odom_delta_removes_z_drift_and_rejects_stairs(self):
        pts_world = _world_floor_cloud(4, floor_z=-0.34)
        R_a = _rot_y(6.5)
        R_b = _rot_y(7.2)
        m_a = detect_floor_plane(pts_world @ R_a, R_a)
        m_b = detect_floor_plane((pts_world - np.array([0.0, 0.0, 0.01])) @ R_b, R_b)  # body rose 1 cm
        self.assertIsNotNone(m_a)
        self.assertIsNotNone(m_b)
        # Odometry has +3 cm fake vertical drift along world Z (within 0.04 m overlap gate)
        T_a = np.eye(4); T_a[:3, :3] = R_a
        T_b_drifted = np.eye(4); T_b_drifted[:3, :3] = R_b; T_b_drifted[:3, 3] = [1.0, 0.0, 0.04]
        delta_drifted = np.linalg.inv(T_a) @ T_b_drifted
        delta_clean, applied = project_coplanar_odom_delta(delta_drifted, m_a, m_b)
        self.assertTrue(applied)
        T_b_clean = T_a @ delta_clean
        self.assertAlmostEqual(T_b_clean[2, 3], 0.01, delta=0.005)
        # Stair jump (+0.35 m) must NOT be projected
        T_b_stair = np.eye(4); T_b_stair[:3, :3] = R_b; T_b_stair[:3, 3] = [0.6, 0.0, 0.35]
        _, applied_stair = project_coplanar_odom_delta(np.linalg.inv(T_a) @ T_b_stair, m_a, m_b)
        self.assertFalse(applied_stair)

    def test_detect_floor_plane_rejects_45deg_rotated_narrow_stair_tread(self):
        rng = np.random.default_rng(15)
        # 5.0 m x 0.2 m narrow tread (world X/Y span ~3.68 m at 45 deg, but PCA short span is 0.2 m)
        u = rng.uniform(1.2, 6.2, 400)
        v = rng.uniform(-0.10, 0.10, 400)
        theta = math.radians(45.0)
        c, s = math.cos(theta), math.sin(theta)
        pts = np.column_stack([c * u - s * v, s * u + c * v, np.full(400, -0.34)])
        self.assertIsNone(detect_floor_plane(pts, np.eye(3)))

    def test_project_coplanar_odom_delta_rejects_8cm_parallel_step_between_platforms(self):
        pts_a = _world_floor_cloud(20, floor_z=-0.34)
        pts_b = _world_floor_cloud(21, floor_z=-0.34)
        m_a = detect_floor_plane(pts_a, np.eye(3))
        m_b = detect_floor_plane(pts_b, np.eye(3))
        self.assertIsNotNone(m_a)
        self.assertIsNotNone(m_b)
        self.assertGreater(len(m_b.inlier_sample_body), 0)
        self.assertLessEqual(len(m_b.inlier_sample_body), 48)
        # Robot stepped onto an 8 cm higher flat platform (body rose 8 cm -> height_below_body_m identical)
        delta_step = np.eye(4, dtype=np.float64)
        delta_step[:3, 3] = [1.0, 0.0, 0.08]
        out, applied = project_coplanar_odom_delta(delta_step, m_a, m_b, max_step_dz_m=0.12)
        self.assertFalse(applied)
        np.testing.assert_allclose(out, delta_step, atol=1e-12)


class SlamBackendPlaneAndXicpTests(unittest.TestCase):
    def test_coplanar_leveling_eliminates_open_loop_z_drift_and_startup_pitch(self):
        cfg = BackendConfig(
            floor_plane_enabled=True,
            floor_normal_gravity_enabled=True,
            floor_normal_graph_sigma_rad=0.015,
            zupt_enabled=False,
        )
        backend = SlamBackend(cfg)
        pts_world = _world_floor_cloud(10, floor_z=-0.34)
        # Simulate 15 steps along +X with +2 cm/step fake odom Z drift (0.28 m total)
        R_bias = np.eye(3)
        for i in range(15):
            T_true = np.eye(4)
            T_true[:3, 3] = [float(i), 0.0, 0.0]
            T_odom = np.eye(4)
            T_odom[:3, :3] = R_bias
            T_odom[:3, 3] = [float(i), 0.0, -0.02 * i]
            pts_body = transform_points(np.linalg.inv(T_true), pts_world)
            kf = Keyframe(key=(0, i), stamp_ns=i * 10**9, T_odom_body=T_odom, T_body_lidar=np.eye(4), points_lidar=pts_body)
            ev = backend.ingest(kf)
            self.assertNotIn("error", ev)
        z_final = abs(float(backend.graph.pose((0, 14))[2, 3]))
        self.assertLess(z_final, 0.02)
        self.assertGreaterEqual(backend.observability_stats()["floor_coplanar_edges"], 12)

    def test_xicp_attenuates_degenerate_corridor_axis_and_preserves_spd(self):
        H = np.diag([0.5, 500.0, 500.0, 200.0, 200.0, 200.0])
        I_base = np.diag([25.0, 25.0, 25.0, 100.0, 100.0, 100.0])
        backend = SlamBackend(BackendConfig(xicp_enabled=True, xicp_min_scale=0.15, xicp_cond_floor=1e-3))
        I_eff, degenerate = backend._modulate_loop_information(I_base, H, np.eye(3))
        self.assertTrue(degenerate)
        np.testing.assert_allclose(I_eff, I_eff.T, atol=1e-12)
        self.assertGreater(float(np.linalg.eigvalsh(I_eff)[0]), 0.0)
        self.assertAlmostEqual(I_eff[0, 0], 25.0 * 0.15, places=6)
        self.assertAlmostEqual(I_eff[1, 1], 25.0, places=6)

    def test_python_and_cpp_icp_return_matching_hessians(self):
        tgt = _world_floor_cloud(22)
        T_true = se3_exp(np.array([0.2, -0.15, 0.05, 0.01, -0.01, 0.1]))
        src = transform_points(np.linalg.inv(T_true), tgt)
        r_py = icp_point_to_plane(src, tgt, np.eye(4), voxel_m=0.2, return_hessian=True, use_cpp=False)
        r_cpp = icp_point_to_plane(src, tgt, np.eye(4), voxel_m=0.2, return_hessian=True, use_cpp=True)
        self.assertTrue(r_py.converged and r_cpp.converged)
        self.assertIsNotNone(r_py.hessian)
        self.assertIsNotNone(r_cpp.hessian)
        np.testing.assert_allclose(r_cpp.hessian, r_py.hessian, rtol=1e-3, atol=1e-3)

    def test_strict_config_validation_and_v3_presets(self):
        repo = Path(__file__).resolve().parents[2]
        for rel in (
            "configs/localization_mapping/slam_v3_plane_factors.json",
            "configs/localization_mapping/slam_v3_plane_factors_tight.json",
        ):
            cfg = BackendConfig.from_json(repo / rel)
            self.assertTrue(cfg.floor_plane_enabled)
            self.assertTrue(cfg.xicp_enabled)
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"unknown_v3_key": True})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"floor_normal_graph_sigma_rad": -0.01})

    def test_zupt_measured_exit_skips_coplanar_leveling(self):
        cfg = BackendConfig(
            zupt_enabled=True,
            zupt_full_stop_rot_rad=0.0,
            floor_plane_enabled=True,
        )
        backend = SlamBackend(cfg)
        pts_world = _world_floor_cloud(31, floor_z=-0.34)
        T0 = np.eye(4)
        T1 = np.eye(4)
        # Frame 0 -> Frame 1: stationary ZUPT pivot/stop
        backend.ingest(Keyframe((0, 0), 0, T0, np.eye(4), pts_world))
        ev1 = backend.ingest(Keyframe((0, 1), 1_000_000_000, T1, np.eye(4), pts_world))
        self.assertEqual(ev1.get("zupt"), "pivot")
        # Frame 2: genuine exit (0.30 m step forward) -> parent_key is anchor (0, 0) != previous (0, 1)
        T2 = np.eye(4)
        T2[:3, 3] = [0.30, 0.0, 0.0]
        pts2 = transform_points(np.linalg.inv(T2), pts_world)
        ev2 = backend.ingest(Keyframe((0, 2), 2_000_000_000, T2, np.eye(4), pts2))
        self.assertNotIn("zupt", ev2)
        self.assertEqual(backend.observability_stats()["floor_coplanar_edges"], 0)

    def test_gravity_world_calibration_and_periodic_open_loop_leveling(self):
        # Simulate a tilted initial odom frame (3 deg pitch bias at kf_0) plus open-loop pitch drift
        # on a flat horizontal floor with zero loop closures.
        pts_world = _world_floor_cloud(42, floor_z=-0.34, span=9.0)
        R_err0 = _rot_y(3.0)
        cfg = BackendConfig(
            zupt_enabled=False,
            floor_plane_enabled=True,
            floor_normal_gravity_enabled=True,
            floor_gravity_world_calibration=True,
            floor_normal_graph_sigma_rad=0.015,
            gravity_optimize_every=5,
        )
        backend = SlamBackend(cfg)
        for i in range(12):
            T_true = np.eye(4)
            T_true[:3, 3] = [0.8 * i, 0.0, 0.0]
            # Odom starts tilted 3 deg AND drifts an extra +0.3 deg/step in pitch
            R_odom = R_err0 @ _rot_y(0.3 * i)
            T_odom = np.eye(4)
            T_odom[:3, :3] = R_odom
            T_odom[:3, 3] = R_err0 @ np.array([0.8 * i, 0.0, 0.0])
            pts_body = transform_points(np.linalg.inv(T_true), pts_world)
            ev = backend.ingest(Keyframe((0, i), i * 1_000_000_000, T_odom, np.eye(4), pts_body))
            self.assertNotIn("error", ev)

        # Compare against the exact same config with gravity_optimize_every=0 (no open-loop optimize)
        cfg_no_opt = BackendConfig(
            zupt_enabled=False,
            floor_plane_enabled=True,
            floor_normal_gravity_enabled=True,
            floor_gravity_world_calibration=True,
            floor_normal_graph_sigma_rad=0.015,
            gravity_optimize_every=0,
        )
        backend_no_opt = SlamBackend(cfg_no_opt)
        for i in range(12):
            T_true = np.eye(4)
            T_true[:3, 3] = [0.8 * i, 0.0, 0.0]
            R_odom = R_err0 @ _rot_y(0.3 * i)
            T_odom = np.eye(4)
            T_odom[:3, :3] = R_odom
            T_odom[:3, 3] = R_err0 @ np.array([0.8 * i, 0.0, 0.0])
            pts_body = transform_points(np.linalg.inv(T_true), pts_world)
            backend_no_opt.ingest(Keyframe((0, i), i * 1_000_000_000, T_odom, np.eye(4), pts_body))

        # g_world should match R_err0 @ e_z to < 0.05 deg
        g_expected = R_err0 @ np.array([0.0, 0.0, 1.0])
        ang_g = math.degrees(math.acos(np.clip(float(backend.graph.gravity_world @ g_expected), -1.0, 1.0)))
        self.assertLess(ang_g, 0.05)
        # Periodic gravity optimization reduces open-loop tilt vs unoptimized open-loop trajectory
        R_opt = backend.graph.pose((0, 9))[:3, :3]
        R_raw = backend_no_opt.graph.pose((0, 9))[:3, :3]
        ang_opt = math.degrees(math.acos(np.clip(float((R_opt @ np.array([0.0, 0.0, 1.0])) @ g_expected), -1.0, 1.0)))
        ang_raw = math.degrees(math.acos(np.clip(float((R_raw @ np.array([0.0, 0.0, 1.0])) @ g_expected), -1.0, 1.0)))
        self.assertGreater(ang_raw, 2.5)
        self.assertLess(ang_opt, ang_raw * 0.65)

    def test_cpp_and_python_non_ez_gravity_world_parity(self):
        from embodied_navigation.localization_mapping.backend.pose_graph import PoseGraph
        from embodied_navigation.localization_mapping.backend.backend import information

        g_w = _rot_y(4.0) @ np.array([0.0, 0.0, 1.0])
        # 1. PoseGraph C++ vs Python parity with tilted gravity_world
        def _run_graph(use_cpp: bool) -> np.ndarray:
            g = PoseGraph(cauchy_scale=3.0)
            g.set_gravity_world(g_w)
            g.add_node(0, np.eye(4))
            T1 = np.eye(4); T1[:3, :3] = _rot_y(1.0); T1[:3, 3] = [1.0, 0.0, 0.0]
            g.add_node(1, T1)
            g.add_edge(0, 1, T1, information(0.2, 0.05), robust=False)
            g.add_gravity_prior(1, np.array([0.0, 0.0, 1.0]), 0.01, robust=True)
            g.optimize(iterations=20, use_cpp=use_cpp)
            return g.pose(1)

        np.testing.assert_allclose(_run_graph(True), _run_graph(False), atol=1e-6)

        # 2. ICP C++ vs Python parity with non-e_z target_up_world
        tgt = _world_floor_cloud(55)
        T_init = se3_exp(np.array([0.1, -0.05, 0.02, 0.0, 0.05, 0.0]))
        r_py = icp_point_to_plane(
            tgt, tgt, T_init, voxel_m=0.2, source_up_body=np.array([0.0, 0.0, 1.0]),
            target_up_world=g_w, gravity_sigma_rad=0.02, use_cpp=False,
        )
        r_cpp = icp_point_to_plane(
            tgt, tgt, T_init, voxel_m=0.2, source_up_body=np.array([0.0, 0.0, 1.0]),
            target_up_world=g_w, gravity_sigma_rad=0.02, use_cpp=True,
        )
        np.testing.assert_allclose(r_cpp.T_target_source, r_py.T_target_source, atol=1e-5)



if __name__ == "__main__":
    unittest.main()
