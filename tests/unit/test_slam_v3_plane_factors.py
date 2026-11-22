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
        # Odometry has +6 cm fake vertical drift along world Z
        T_a = np.eye(4); T_a[:3, :3] = R_a
        T_b_drifted = np.eye(4); T_b_drifted[:3, :3] = R_b; T_b_drifted[:3, 3] = [1.0, 0.0, 0.07]
        delta_drifted = np.linalg.inv(T_a) @ T_b_drifted
        delta_clean, applied = project_coplanar_odom_delta(delta_drifted, m_a, m_b)
        self.assertTrue(applied)
        T_b_clean = T_a @ delta_clean
        self.assertAlmostEqual(T_b_clean[2, 3], 0.01, delta=0.005)
        # Stair jump (+0.35 m) must NOT be projected
        T_b_stair = np.eye(4); T_b_stair[:3, :3] = R_b; T_b_stair[:3, 3] = [0.6, 0.0, 0.35]
        _, applied_stair = project_coplanar_odom_delta(np.linalg.inv(T_a) @ T_b_stair, m_a, m_b)
        self.assertFalse(applied_stair)


class SlamBackendV3PlaneAndXicpTests(unittest.TestCase):
    def test_coplanar_leveling_eliminates_open_loop_z_drift_and_startup_pitch(self):
        cfg = BackendConfig(
            floor_plane_enabled=True,
            floor_normal_gravity_enabled=True,
            floor_normal_graph_sigma_rad=0.015,
            zupt_enabled=False,
        )
        backend = SlamBackend(cfg)
        pts_world = _world_floor_cloud(10, floor_z=-0.34)
        # Simulate 15 steps along +X with 1.0 deg startup pitch bias and +2 cm/step fake odom Z drift
        R_bias = _rot_y(1.0)
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


if __name__ == "__main__":
    unittest.main()
