"""Unit tests for oriented-normal 3D planar bundle adjustment."""

from __future__ import annotations

from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from embodied_navigation.localization_mapping.backend.accel import get_accel_lib
from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe
from embodied_navigation.localization_mapping.backend.planar_ba import (
    refine_poses_oriented_planar_ba,
    refine_window_oriented_planar_ba,
)
from embodied_navigation.localization_mapping.backend.pose_graph import PoseGraph, invert


REPO_ROOT = Path(__file__).resolve().parents[2]


def _make_pose(xyz: tuple[float, float, float], rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    T[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return T


def _room_world_cloud(rng: np.random.Generator) -> np.ndarray:
    """Synthetic corner of a room: floor z=0, wall x=4, wall y=4."""
    pts = []
    # Floor z=0, x in [0, 3.8], y in [0, 3.8]
    u = np.linspace(0.2, 3.6, 20)
    v = np.linspace(0.2, 3.6, 20)
    gx, gy = np.meshgrid(u, v)
    pts.append(np.column_stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)]))
    # Wall x=4, y in [0.2, 3.6], z in [0.2, 2.6]
    wy, wz = np.meshgrid(np.linspace(0.2, 3.6, 18), np.linspace(0.2, 2.6, 14))
    pts.append(np.column_stack([np.full(wy.size, 4.0), wy.ravel(), wz.ravel()]))
    # Wall y=4, x in [0.2, 3.6], z in [0.2, 2.6]
    wx, wz2 = np.meshgrid(np.linspace(0.2, 3.6, 18), np.linspace(0.2, 2.6, 14))
    pts.append(np.column_stack([wx.ravel(), np.full(wx.size, 4.0), wz2.ravel()]))
    cloud = np.vstack(pts)
    cloud += rng.normal(0.0, 0.002, size=cloud.shape)
    return cloud


def _transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return (T[:3, :3] @ pts.T).T + T[:3, 3]


class OrientedPlanarBaTests(unittest.TestCase):
    def test_synthetic_room_reduces_pose_drift(self) -> None:
        if get_accel_lib() is None:
            self.skipTest("C++ accelerator unavailable")
        rng = np.random.default_rng(7)
        world_cloud = _room_world_cloud(rng)
        gt_poses = [
            _make_pose((0.5 + 0.4 * i, 0.6 + 0.25 * i, 0.6), (0.0, 0.0, 4.0 * i))
            for i in range(6)
        ]
        # Perturb nodes 1..5 by 4-6 cm and ~1.5 deg
        est_poses = [gt_poses[0].copy()]
        for i in range(1, len(gt_poses)):
            delta = _make_pose(
                (0.04 * (i % 3 - 1), -0.045 * ((i + 1) % 2), 0.05 * (1 if i % 2 else -1)),
                (1.2 * (1 if i % 2 else -1), -1.0, 1.5),
            )
            est_poses.append(gt_poses[i] @ delta)

        graph = PoseGraph()
        pts_body_list = []
        odom_info = np.diag([25.0, 25.0, 25.0, 2500.0, 2500.0, 2500.0])  # loose odom
        for i, (gt_T, est_T) in enumerate(zip(gt_poses, est_poses)):
            graph.add_node((0, i), est_T)
            pts_body_list.append(_transform_points(invert(gt_T), world_cloud))
            if i > 0:
                graph.add_edge((0, i - 1), (0, i), invert(est_poses[i - 1]) @ est_poses[i], odom_info)

        err_before = np.sqrt(
            np.mean([np.sum((est_poses[i][:3, 3] - gt_poses[i][:3, 3]) ** 2) for i in range(1, len(gt_poses))])
        )
        res = refine_poses_oriented_planar_ba(
            graph,
            pts_body_list,
            voxel_m=1.0,
            min_frame_pts=8,
            plane_sigma_m=0.015,
            outer_rounds=3,
            inner_iterations=12,
        )
        self.assertTrue(res.used_cpp)
        self.assertGreater(res.clusters_used, 10)
        self.assertLess(res.error_after, res.error_before)

        err_after = np.sqrt(
            np.mean([np.sum((res.poses[i][:3, 3] - gt_poses[i][:3, 3]) ** 2) for i in range(1, len(gt_poses))])
        )
        self.assertLess(err_after, 0.012, f"expected < 1.2 cm after BA, got {err_after*100:.2f} cm (before {err_before*100:.2f} cm)")
        self.assertLess(err_after, err_before * 0.25)

    def test_thin_two_floor_slab_does_not_crush_ceiling_and_floor(self) -> None:
        """Basement ceiling (z=-0.15) and Floor 2 floor (z=+0.01) share a 1m voxel
        and are 16cm apart (< max_plane_offset_m=0.25), but have opposite ray-oriented
        normals so they MUST NOT merge or collapse toward each other."""
        if get_accel_lib() is None:
            self.skipTest("C++ accelerator unavailable")
        u = np.linspace(0.15, 2.85, 18)
        v = np.linspace(0.15, 2.85, 18)
        gx, gy = np.meshgrid(u, v)
        basement_ceiling = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, -0.15)])
        floor2_floor = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, +0.01)])

        # 2 basement poses looking UP at z=-0.15; 2 floor-2 poses looking DOWN at z=+0.01
        poses = [
            _make_pose((1.0, 1.0, -1.50)),
            _make_pose((1.8, 1.5, -1.50)),
            _make_pose((1.0, 1.0, +0.35)),
            _make_pose((1.8, 1.5, +0.35)),
        ]
        pts_body = [
            _transform_points(invert(poses[0]), basement_ceiling),
            _transform_points(invert(poses[1]), basement_ceiling),
            _transform_points(invert(poses[2]), floor2_floor),
            _transform_points(invert(poses[3]), floor2_floor),
        ]
        graph = PoseGraph()
        weak_odom = np.diag([10.0, 10.0, 10.0, 500.0, 500.0, 500.0])
        for i, T in enumerate(poses):
            graph.add_node((0, i), T)
            if i > 0:
                graph.add_edge((0, i - 1), (0, i), invert(poses[i - 1]) @ poses[i], weak_odom)

        res = refine_poses_oriented_planar_ba(
            graph,
            pts_body,
            voxel_m=1.0,
            min_frame_pts=8,
            min_normal_cos=0.85,
            max_plane_offset_m=0.25,  # intentionally wider than the 16cm slab!
            plane_sigma_m=0.01,
            outer_rounds=2,
            inner_iterations=10,
        )
        self.assertTrue(res.used_cpp)
        self.assertGreater(res.clusters_used, 0)
        # Floor 2 poses MUST NOT be pulled down toward basement ceiling!
        for idx in (1, 2, 3):
            dz = abs(res.poses[idx][2, 3] - poses[idx][2, 3])
            self.assertLess(dz, 0.005, f"node {idx} Z shifted by {dz*100:.2f} cm (slab crushed!)")

    def test_thin_partition_wall_two_sides_do_not_crush(self) -> None:
        """Wall face A (x=0.00) observed from x=-2 and face B (x=+0.12) observed
        from x=+2 share a 1m voxel (< max_plane_offset_m=0.25) but have opposing
        ray-oriented normals, so wall thickness must remain intact."""
        if get_accel_lib() is None:
            self.skipTest("C++ accelerator unavailable")
        wy, wz = np.meshgrid(np.linspace(0.2, 2.8, 16), np.linspace(0.2, 2.2, 14))
        wall_a = np.column_stack([np.zeros(wy.size), wy.ravel(), wz.ravel()])
        wall_b = np.column_stack([np.full(wy.size, 0.12), wy.ravel(), wz.ravel()])

        poses = [
            _make_pose((-2.0, 1.0, 1.0)),
            _make_pose((-2.0, 1.8, 1.0)),
            _make_pose((+2.0, 1.0, 1.0)),
            _make_pose((+2.0, 1.8, 1.0)),
        ]
        pts_body = [
            _transform_points(invert(poses[0]), wall_a),
            _transform_points(invert(poses[1]), wall_a),
            _transform_points(invert(poses[2]), wall_b),
            _transform_points(invert(poses[3]), wall_b),
        ]
        graph = PoseGraph()
        weak_odom = np.diag([10.0, 10.0, 10.0, 500.0, 500.0, 500.0])
        for i, T in enumerate(poses):
            graph.add_node((0, i), T)
            if i > 0:
                graph.add_edge((0, i - 1), (0, i), invert(poses[i - 1]) @ poses[i], weak_odom)

        res = refine_poses_oriented_planar_ba(
            graph,
            pts_body,
            voxel_m=1.0,
            min_frame_pts=8,
            min_normal_cos=0.85,
            max_plane_offset_m=0.25,  # intentionally wider than the 12cm wall!
            plane_sigma_m=0.01,
            outer_rounds=2,
            inner_iterations=10,
        )
        self.assertTrue(res.used_cpp)
        self.assertGreater(res.clusters_used, 0)
        for idx in (1, 2, 3):
            dx = abs(res.poses[idx][0, 3] - poses[idx][0, 3])
            self.assertLess(dx, 0.005, f"node {idx} X shifted by {dx*100:.2f} cm (thin wall crushed!)")

    def test_config_validation_and_presets(self) -> None:
        for name in (
            "slam_combined_lidar_gravity_ba.json",
            "slam_combined_lidar_fine_gravity_ba.json",
        ):
            cfg = BackendConfig.from_json(REPO_ROOT / "configs" / "localization_mapping" / name)
            self.assertTrue(cfg.planar_ba_enabled)
            self.assertTrue(cfg.floor_gravity_world_calibration)

        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"planar_ba_enabled": "yes"})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"planar_ba_voxel_m": 0.0})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"planar_ba_min_normal_cos": -1.0})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"planar_ba_min_frame_pts": 2})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"planar_ba_max_frames_per_cluster": 1})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"planar_ba_window_size": 1})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"planar_ba_every": -1})
        with self.assertRaises(ValueError):
            BackendConfig.from_dict({"planar_ba_global_every": -1})

    def test_sliding_window_ba_anchors_prefix_and_refines_tail(self) -> None:
        if get_accel_lib() is None:
            self.skipTest("C++ accelerator unavailable")
        rng = np.random.default_rng(19)
        world_cloud = _room_world_cloud(rng)
        gt_poses = [_make_pose((0.5 + 0.3 * i, 0.5 + 0.2 * i, 0.6), (0.0, 0.0, 3.0 * i)) for i in range(8)]
        # First 4 nodes exact; last 4 nodes perturbed by 5 cm
        est_poses = [gt_poses[i].copy() for i in range(4)]
        for i in range(4, 8):
            est_poses.append(gt_poses[i] @ _make_pose((0.04, -0.04, 0.05), (1.0, -0.8, 1.2)))

        graph = PoseGraph()
        pts_body = []
        loose = np.diag([25.0, 25.0, 25.0, 2500.0, 2500.0, 2500.0])
        for i, (gt_T, est_T) in enumerate(zip(gt_poses, est_poses)):
            graph.add_node((0, i), est_T)
            pts_body.append(_transform_points(invert(gt_T), world_cloud))
            if i > 0:
                graph.add_edge((0, i - 1), (0, i), invert(est_poses[i - 1]) @ est_poses[i], loose)

        updated_map, res = refine_window_oriented_planar_ba(
            graph,
            pts_body,
            window_size=5,
            voxel_m=1.0,
            plane_sigma_m=0.015,
            outer_rounds=2,
            inner_iterations=10,
        )
        self.assertTrue(res.used_cpp)
        self.assertEqual(sorted(updated_map.keys()), [4, 5, 6, 7])
        err_after = np.sqrt(
            np.mean([np.sum((updated_map[i][:3, 3] - gt_poses[i][:3, 3]) ** 2) for i in range(4, 8)])
        )
        self.assertLess(err_after, 0.015, f"expected window tail < 1.5 cm, got {err_after*100:.2f} cm")

    def test_online_backend_runs_window_and_global_ba_during_ingest(self) -> None:
        if get_accel_lib() is None:
            self.skipTest("C++ accelerator unavailable")
        rng = np.random.default_rng(23)
        world_cloud = _room_world_cloud(rng)
        cfg = BackendConfig.from_dict(
            {
                "map_voxel_m": 0.15,
                "planar_ba_enabled": True,
                "planar_ba_voxel_m": 1.0,
                "planar_ba_sigma_m": 0.02,
                "planar_ba_window_size": 5,
                "planar_ba_every": 2,
                "planar_ba_global_every": 6,
                "planar_ba_online_iterations": 6,
            }
        )
        backend = SlamBackend(cfg)
        gt_poses = [_make_pose((0.5 + 0.3 * i, 0.5 + 0.15 * i, 0.6)) for i in range(8)]
        for i, gt_T in enumerate(gt_poses):
            odom_T = gt_T if i == 0 else (gt_T @ _make_pose((0.02 * i, -0.01 * i, 0.03)))
            kf = Keyframe(
                key=(0, i),
                stamp_ns=int((i + 1) * 1e9),
                T_odom_body=odom_T,
                T_body_lidar=np.eye(4, dtype=np.float64),
                points_lidar=_transform_points(invert(gt_T), world_cloud),
            )
            backend.ingest(kf)
        stats = backend.observability_stats()
        self.assertGreaterEqual(stats["planar_ba_window_runs"], 1)
        self.assertGreaterEqual(stats["planar_ba_global_runs"], 1)
        self.assertIsNotNone(backend.T_world_odom)

    def test_odom_icp_repair_adopts_immediately_without_single_frame_spike(self) -> None:
        rng = np.random.default_rng(31)
        world_cloud = _room_world_cloud(rng)
        cfg = BackendConfig.from_dict(
            {
                "map_voxel_m": 0.15,
                "odometry_icp_repair_enabled": True,
                "odometry_robust": True,
                "confirmations": 2,
                "loop_max_jump_translation_m": 3.0,
            }
        )
        backend = SlamBackend(cfg)
        gt_poses = [
            _make_pose((1.0, 1.0, 0.6)),
            _make_pose((1.4, 1.0, 0.6)),
            _make_pose((1.8, 1.0, 0.6)),
        ]
        # Frame 2 frontend odom suddenly slips -0.80 m in Z while true pose stays at z=0.6
        odom_poses = [
            gt_poses[0].copy(),
            gt_poses[1].copy(),
            _make_pose((1.8, 1.0, -0.20)),
        ]
        events = []
        for i, (gt_T, odom_T) in enumerate(zip(gt_poses, odom_poses)):
            kf = Keyframe(
                key=(0, i),
                stamp_ns=int((i + 1) * 1e9),
                T_odom_body=odom_T,
                T_body_lidar=np.eye(4, dtype=np.float64),
                points_lidar=_transform_points(invert(gt_T), world_cloud),
            )
            events.append(backend.ingest(kf))

        self.assertTrue(events[2].get("odom_repaired", False))
        self.assertTrue(events[2]["adopted"], "slip repair must adopt immediately on the repaired frame")
        est_world = backend.T_world_odom @ odom_poses[2]
        err_m = float(np.linalg.norm(est_world[:3, 3] - gt_poses[2][:3, 3]))
        self.assertLess(err_m, 0.05, f"expected < 5 cm online error on repaired frame, got {err_m*100:.1f} cm")

        # Jump gate must still reject out-of-bounds jumps on propose_immediate
        huge_C = _make_pose((10.0, 0.0, 0.0))
        self.assertIsNone(backend.admission.propose_immediate(huge_C, odom_poses[2], int(4e9)))
        self.assertGreaterEqual(len(backend.admission.rejections), 1)

    def test_p0_audit_fixes_z_priors_nan_gate_self_loop_finalize_snapshot(self) -> None:
        from embodied_navigation.localization_mapping.backend.correction import CorrectionAdmission
        from embodied_navigation.localization_mapping.backend.evaluation import replay_backend

        # P0-7: NaN pose must be rejected by CorrectionAdmission (propose & propose_immediate)
        adm = CorrectionAdmission(confirmations=2, max_jump_translation_m=1.0, max_jump_rotation_rad=0.35)
        nan_C = np.eye(4, dtype=np.float64)
        nan_C[0, 3] = float("nan")
        self.assertIsNone(adm.propose_immediate(nan_C, np.eye(4), int(1e9), reference_C=np.eye(4)))
        self.assertIsNone(adm.current)
        self.assertEqual(len(adm.rejections), 1)

        # P0-8: Pure-Python PoseGraph handles 2-node empty-factor graph & self-loop safely
        pg_empty = PoseGraph()
        pg_empty.add_node((0, 0), np.eye(4))
        pg_empty.add_node((0, 1), _make_pose((1.0, 0.0, 0.0)))
        pg_empty.add_edge((0, 1), (0, 1), _make_pose((0.1, 0.0, 0.0)), np.eye(6))
        summary_py = pg_empty.optimize(5, use_cpp=False)
        self.assertTrue(summary_py["converged"])

        # P0-4: Planar BA preserves PoseGraph z_priors instead of dropping them
        if get_accel_lib() is not None:
            rng = np.random.default_rng(41)
            world_cloud = _room_world_cloud(rng)
            pg = PoseGraph()
            gt_p = [_make_pose((0.5 + 0.4 * i, 0.6, 0.6)) for i in range(4)]
            pts_b = [_transform_points(invert(T), world_cloud) for T in gt_p]
            loose = np.diag([10.0, 10.0, 10.0, 500.0, 500.0, 500.0])
            for i, T in enumerate(gt_p):
                est = T.copy() if i == 0 else (T @ _make_pose((0.0, 0.0, 0.08)))
                pg.add_node((0, i), est)
                if i > 0:
                    pg.add_edge((0, i - 1), (0, i), invert(gt_p[i - 1]) @ gt_p[i], loose)
                    pg.add_z_prior((0, i), 0.60, 0.005, robust=False)
            res = refine_poses_oriented_planar_ba(pg, pts_b, voxel_m=1.0, plane_sigma_m=0.05, outer_rounds=2, inner_iterations=8)
            self.assertTrue(res.used_cpp)
            for i in range(1, 4):
                self.assertAlmostEqual(res.poses[i][2, 3], 0.60, delta=0.015)

            # P0-1 & P0-3 & P0-6: finalize appends matching snapshot, keeps admission consistent,
            # and replay_backend survives frame-0 error without np.stack([]) crash.
            cfg = BackendConfig.from_dict(
                {
                    "map_voxel_m": 0.15,
                    "planar_ba_enabled": True,
                    "planar_ba_voxel_m": 1.0,
                    "planar_ba_sigma_m": 0.02,
                }
            )
            be = SlamBackend(cfg)
            kfs = []
            for i, T in enumerate(gt_p):
                odom_T = T if i == 0 else (T @ _make_pose((0.03 * i, -0.02 * i, 0.04 * (1 if i % 2 else -1))))
                kfs.append(
                    Keyframe(
                        key=(0, i),
                        stamp_ns=int((i + 1) * 1e9),
                        T_odom_body=odom_T,
                        T_body_lidar=np.eye(4),
                        points_lidar=pts_b[i],
                    )
                )
            rec = replay_backend(be, kfs)
            # Final snapshot must match final_poses after finalize()
            np.testing.assert_allclose(rec.snapshot_pos[-1], rec.final_poses[:, :3, 3].astype(np.float32), atol=1e-4)
            # Second finalize() on unchanged graph is idempotent
            self.assertFalse(be.finalize()["ran"])

            # P0-6: Frame 0 fails (empty cloud) -> replay_backend does not crash
            be_err = SlamBackend(BackendConfig())
            bad_kf0 = Keyframe(
                key=(0, 0),
                stamp_ns=int(1e9),
                T_odom_body=np.eye(4),
                T_body_lidar=np.eye(4),
                points_lidar=np.empty((0, 3), dtype=np.float64),
            )
            rec_err = replay_backend(be_err, [bad_kf0, kfs[1]])
            self.assertEqual(rec_err.errors, 1)
            self.assertEqual(rec_err.missing_graph_nodes, 1)

    def test_backend_finalize_runs_planar_ba_and_updates_stats(self) -> None:
        if get_accel_lib() is None:
            self.skipTest("C++ accelerator unavailable")
        rng = np.random.default_rng(13)
        world_cloud = _room_world_cloud(rng)
        cfg = BackendConfig.from_dict(
            {
                "map_voxel_m": 0.15,
                "planar_ba_enabled": True,
                "planar_ba_voxel_m": 1.0,
                "planar_ba_sigma_m": 0.02,
                "planar_ba_outer_rounds": 2,
                "planar_ba_iterations": 8,
            }
        )
        backend = SlamBackend(cfg)
        gt_poses = [_make_pose((0.5 + 0.3 * i, 0.5 + 0.2 * i, 0.6)) for i in range(4)]
        odom_poses = [gt_poses[0].copy()]
        for i in range(1, 4):
            odom_poses.append(gt_poses[i] @ _make_pose((0.03 * i, -0.02 * i, 0.04 * (1 if i % 2 else -1))))

        for i, (gt_T, odom_T) in enumerate(zip(gt_poses, odom_poses)):
            kf = Keyframe(
                key=(0, i),
                stamp_ns=int((i + 1) * 1e9),
                T_odom_body=odom_T,
                T_body_lidar=np.eye(4, dtype=np.float64),
                points_lidar=_transform_points(invert(gt_T), world_cloud),
            )
            backend.ingest(kf)

        summary = backend.finalize()
        self.assertTrue(summary["ran"])
        self.assertTrue(summary["used_cpp"])
        self.assertGreater(summary["clusters_used"], 0)
        stats = backend.observability_stats()
        self.assertEqual(stats["planar_ba_runs"], 1)
        self.assertGreater(stats["planar_ba_clusters"], 0)


if __name__ == "__main__":
    unittest.main()
