"""End-to-end integration tests for the unified Direction-3 SLAM backend presets."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import unittest

import numpy as np

from embodied_navigation.localization_mapping.backend.backend import (
    BackendConfig,
    SlamBackend,
)
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe, transform_points
from embodied_navigation.localization_mapping.backend.pose_graph import invert
from embodied_navigation.localization_mapping.backend.visual import VisualObservation

REPO_ROOT = Path(__file__).resolve().parents[2]


def _yaw_pose(yaw_rad: float = 0.0, xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    T[0, 0], T[0, 1] = c, -s
    T[1, 0], T[1, 1] = s, c
    T[:3, 3] = xyz
    return T


def _textured_scene(seed: int = 0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Asymmetric 3D indoor room (wide flat floor + 3 orthogonal walls) with RGB/gray texture."""
    rng = np.random.default_rng(seed)
    xs = np.linspace(-3.0, 5.0, 48)
    ys = np.linspace(-3.0, 3.5, 40)
    gx, gy = np.meshgrid(xs, ys)
    keep = np.hypot(gx.ravel(), gy.ravel()) >= 1.05
    floor = np.column_stack([gx.ravel()[keep], gy.ravel()[keep], np.full(int(keep.sum()), -0.34)])

    zs = np.linspace(-0.20, 2.20, 22)
    gy_w, gz_w = np.meshgrid(ys, zs)
    wall_px = np.column_stack([np.full(gy_w.size, 5.0), gy_w.ravel(), gz_w.ravel()])
    wall_nx = np.column_stack([np.full(gy_w.size, -3.0), gy_w.ravel(), gz_w.ravel()])
    gx_w, gz_wy = np.meshgrid(xs, zs)
    wall_py = np.column_stack([gx_w.ravel(), np.full(gx_w.size, 3.5), gz_wy.ravel()])
    wall_ny = np.column_stack([gx_w.ravel(), np.full(gx_w.size, -3.0), gz_wy.ravel()])
    pts_world = np.vstack([floor, wall_px, wall_nx, wall_py, wall_ny])

    h, w = 96, 128
    base = rng.integers(30, 230, size=(h // 4, w // 4, 3), dtype=np.uint8)
    rgb = np.repeat(np.repeat(base, 4, axis=0), 4, axis=1)
    gray = (rgb.astype(np.float64) @ np.array([0.299, 0.587, 0.114])).astype(np.float64)
    return pts_world, gray, rgb


def _camera_calibration() -> tuple[np.ndarray, np.ndarray]:
    K = np.array([[90.0, 0.0, 64.0], [0.0, 90.0, 48.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    # Camera looks along body +X (optical z = +X_body, x = -Y_body, y = -Z_body)
    T_bc = np.array(
        [
            [0.0, 0.0, 1.0, 0.2],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.1],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    return K, T_bc


class SlamV3CombinedTests(unittest.TestCase):
    def test_combined_presets_load_strictly_and_expose_expected_fields(self) -> None:
        lidar_cfg = BackendConfig.from_json(
            REPO_ROOT / "configs" / "localization_mapping" / "slam_v3_combined_lidar.json"
        )
        self.assertTrue(lidar_cfg.zupt_enabled)
        self.assertTrue(lidar_cfg.use_measured_submaps)
        self.assertEqual(lidar_cfg.measured_submap.submap_window, 2)
        self.assertEqual(lidar_cfg.measured_submap.anchor_mode, "candidate")
        self.assertAlmostEqual(lidar_cfg.measured_submap.submap_max_span_s, 4.0)
        self.assertAlmostEqual(lidar_cfg.measured_submap.submap_max_yaw_span_deg, 35.0)
        self.assertTrue(lidar_cfg.use_surface_verification)
        self.assertTrue(lidar_cfg.gravity_aligned)
        self.assertAlmostEqual(lidar_cfg.surface.min_coverage, 0.35)
        self.assertAlmostEqual(lidar_cfg.surface.normal_cos_min, 0.82)
        self.assertTrue(lidar_cfg.floor_plane_enabled)
        self.assertTrue(lidar_cfg.floor_normal_gravity_enabled)
        self.assertAlmostEqual(lidar_cfg.floor_normal_graph_sigma_rad, 0.015)
        self.assertTrue(lidar_cfg.xicp_enabled)
        self.assertAlmostEqual(lidar_cfg.map_voxel_m, 0.20)
        self.assertAlmostEqual(lidar_cfg.fine_icp_voxel_m, 0.15)
        self.assertFalse(lidar_cfg.use_visual_retrieval)
        self.assertFalse(lidar_cfg.color_sc_enabled)

        fine_cfg = BackendConfig.from_json(
            REPO_ROOT / "configs" / "localization_mapping" / "slam_v3_combined_lidar_fine.json"
        )
        self.assertAlmostEqual(fine_cfg.map_voxel_m, 0.12)
        self.assertAlmostEqual(fine_cfg.fine_icp_voxel_m, 0.10)
        self.assertAlmostEqual(fine_cfg.floor_normal_graph_sigma_rad, 0.012)
        self.assertTrue(fine_cfg.floor_plane_enabled)
        self.assertTrue(fine_cfg.use_measured_submaps)
        self.assertTrue(fine_cfg.use_surface_verification)
        self.assertTrue(fine_cfg.xicp_enabled)

        hybrid_cfg = BackendConfig.from_json(
            REPO_ROOT / "configs" / "localization_mapping" / "slam_v3_combined_hybrid.json"
        )
        self.assertTrue(hybrid_cfg.use_visual_retrieval)
        self.assertEqual(hybrid_cfg.visual_retrieval.feature_type, "akaze")
        self.assertTrue(hybrid_cfg.color_sc_enabled)
        self.assertAlmostEqual(hybrid_cfg.color_sc_weight, 0.25)
        self.assertEqual(hybrid_cfg.visual_multiview_radius, 1)
        self.assertAlmostEqual(hybrid_cfg.hybrid_heading_gate_deg, 65.0)
        self.assertTrue(hybrid_cfg.floor_plane_enabled)
        self.assertTrue(hybrid_cfg.use_measured_submaps)
        self.assertTrue(hybrid_cfg.use_surface_verification)
        self.assertTrue(hybrid_cfg.xicp_enabled)

    def test_combined_lidar_and_fine_presets_run_end_to_end_with_z_drift_and_loop(self) -> None:
        pts_world, _, _ = _textured_scene(seed=7)
        for preset_name in ("slam_v3_combined_lidar.json", "slam_v3_combined_lidar_fine.json"):
            with self.subTest(preset=preset_name):
                base_cfg = BackendConfig.from_json(
                    REPO_ROOT / "configs" / "localization_mapping" / preset_name
                )
                cfg = replace(
                    base_cfg,
                    min_loop_travel_m=4.0,
                    min_loop_gap_s=4.0,
                    exclude_recent=2,
                    min_fitness=0.55,
                    max_rmse_m=0.12,
                )
                backend = SlamBackend(cfg)
                # Build an outbound + Z-drifted walk (0..6) then return near origin (7..9)
                true_poses = [
                    _yaw_pose(0.0, (0.0, 0.0, 0.0)),
                    _yaw_pose(0.0, (1.0, 0.0, 0.0)),
                    _yaw_pose(0.0, (2.0, 0.0, 0.0)),
                    _yaw_pose(0.0, (2.0, 0.0, 0.0)),  # stationary ZUPT frame
                    _yaw_pose(0.0, (3.0, 0.0, 0.0)),
                    _yaw_pose(0.0, (4.0, 0.0, 0.0)),
                    _yaw_pose(math.pi, (2.5, 0.0, 0.0)),
                    _yaw_pose(math.pi, (1.0, 0.0, 0.0)),
                    _yaw_pose(math.pi, (0.05, 0.02, 0.0)),
                ]
                events = []
                for idx, T_true in enumerate(true_poses):
                    T_odom = T_true.copy()
                    # Inject +2.5 cm/step spurious Z drift after index 0
                    T_odom[2, 3] -= 0.025 * idx
                    pts_body = transform_points(invert(T_true), pts_world)
                    kf = Keyframe(
                        key=(0, idx),
                        stamp_ns=idx * 1_500_000_000,
                        T_odom_body=T_odom,
                        T_body_lidar=np.eye(4),
                        points_lidar=pts_body,
                    )
                    ev = backend.ingest(kf)
                    self.assertNotIn("error", ev)
                    events.append(ev)

                stats = backend.observability_stats()
                self.assertGreaterEqual(stats["floor_planes_detected"], 8)
                self.assertGreaterEqual(stats["floor_coplanar_edges"], 5)
                self.assertGreaterEqual(stats["floor_gravity_priors_added"], 7)
                self.assertGreaterEqual(stats["measured_query_submaps"], 1)
                self.assertGreaterEqual(stats["measured_target_submaps"], 1)
                self.assertGreaterEqual(stats["surface_checks"], 1)
                self.assertGreaterEqual(stats["surface_accepted"], 1)
                self.assertTrue(any(ev["loop"] for ev in events), events)
                # Coplanar floor projection + floor-normal gravity must cancel the injected 0.20 m Z drift
                z_err = abs(float(backend.graph.pose((0, 8))[2, 3]))
                self.assertLess(z_err, 0.02)

    def test_combined_hybrid_runs_all_three_directions_simultaneously(self) -> None:
        pts_world, gray_fwd, rgb_fwd = _textured_scene(seed=19)
        _, gray_rev, rgb_rev = _textured_scene(seed=29)
        K, T_bc = _camera_calibration()

        base_cfg = BackendConfig.from_json(
            REPO_ROOT / "configs" / "localization_mapping" / "slam_v3_combined_hybrid.json"
        )
        # Use patch7x7 for deterministic textured-pattern feature matching on synthetic noise images,
        # keeping all other combined hybrid + submap + surface + floor_plane + X-ICP settings active.
        cfg = replace(
            base_cfg,
            visual_retrieval=replace(base_cfg.visual_retrieval, feature_type="patch7x7", max_distance=0.35),
            min_loop_travel_m=3.0,
            min_loop_gap_s=4.0,
            exclude_recent=2,
            min_fitness=0.55,
            max_rmse_m=0.12,
        )
        backend = SlamBackend(cfg)

        # Outbound trajectory (0..4) with Z drift, then:
        # - frame 5: same-heading revisit at origin with flat (untextured) camera -> must reject via hybrid visual gate
        # - frame 6: same-heading revisit at origin with matching visual obs -> must accept via visual + submap + surface + X-ICP
        # - frame 7..8: turn 180 deg and walk > min_loop_spacing_m (0.5 m) with distinct camera -> must accept via 360 LiDAR + submap + surface + X-ICP
        poses_and_vis = [
            (_yaw_pose(0.0, (0.0, 0.0, 0.0)), gray_fwd, rgb_fwd),
            (_yaw_pose(0.0, (1.0, 0.0, 0.0)), gray_rev, rgb_rev),
            (_yaw_pose(0.0, (2.0, 0.0, 0.0)), gray_rev, rgb_rev),
            (_yaw_pose(0.0, (3.0, 0.0, 0.0)), gray_rev, rgb_rev),
            (_yaw_pose(0.0, (4.0, 0.0, 0.0)), gray_rev, rgb_rev),
            (_yaw_pose(0.0, (0.05, 0.0, 0.0)), np.full_like(gray_fwd, 120.0), None),
            (_yaw_pose(0.0, (0.02, 0.02, 0.0)), gray_fwd, rgb_fwd),
            (_yaw_pose(math.pi, (0.70, 0.0, 0.0)), gray_rev, rgb_rev),
            (_yaw_pose(math.pi, (0.04, 0.0, 0.0)), gray_rev, rgb_rev),
        ]
        events = []
        for idx, (T_true, g, r) in enumerate(poses_and_vis):
            T_odom = T_true.copy()
            T_odom[2, 3] -= 0.02 * idx
            pts_body = transform_points(invert(T_true), pts_world)
            stamp = idx * 1_500_000_000
            vis = VisualObservation(gray=g, K=K, T_body_camera=T_bc, stamp_ns=stamp, rgb=r)
            kf = Keyframe(
                key=(0, idx),
                stamp_ns=stamp,
                T_odom_body=T_odom,
                T_body_lidar=np.eye(4),
                points_lidar=pts_body,
                visual=vis,
            )
            ev = backend.ingest(kf)
            self.assertNotIn("error", ev)
            events.append(ev)

        self.assertFalse(events[5]["loop"])
        self.assertTrue(events[6]["loop"])
        self.assertTrue(events[7]["loop"] or events[8]["loop"], events[7:])
        stats = backend.observability_stats()
        self.assertGreaterEqual(stats["hybrid_same_heading_visual_rejected"], 1)
        self.assertGreaterEqual(stats["hybrid_same_heading_visual_accepted"], 1)
        self.assertGreaterEqual(stats["hybrid_reverse_lidar_accepted"], 1)
        self.assertGreaterEqual(stats["color_sc_rescored"], 1)
        self.assertGreaterEqual(stats["measured_query_submaps"], 1)
        self.assertGreaterEqual(stats["measured_target_submaps"], 1)
        self.assertGreaterEqual(stats["surface_checks"], 1)
        self.assertGreaterEqual(stats["surface_accepted"], 1)
        self.assertGreaterEqual(stats["floor_planes_detected"], 8)
        self.assertGreaterEqual(stats["floor_coplanar_edges"], 6)
        self.assertGreaterEqual(stats["floor_gravity_priors_added"], 7)
        self.assertLess(abs(float(backend.graph.pose((0, 8))[2, 3])), 0.02)


if __name__ == "__main__":
    unittest.main()
