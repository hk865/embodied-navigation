"""Unit tests for v3 bounded measured submaps, soft surface gate, X-ICP degeneracy check, and fine-voxel ICP."""

from __future__ import annotations

from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from embodied_navigation.localization_mapping.backend.backend import (
    BackendConfig,
    SlamBackend,
)
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe, transform_points, voxel_downsample
from embodied_navigation.localization_mapping.backend.pose_graph import se3_exp
from embodied_navigation.localization_mapping.backend.registration import icp_point_to_plane
from embodied_navigation.localization_mapping.backend.submaps import SubmapConfig, build_submap
from embodied_navigation.localization_mapping.backend.surfaces import (
    SurfaceConfig,
    compare_surfaces,
    model_surfaces,
)
import slam_backend_fixtures as F

REPO_ROOT = Path(__file__).resolve().parents[2]


def _yaw_pose(x: float = 0.0, y: float = 0.0, z: float = 0.0, yaw_deg: float = 0.0) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    rad = np.radians(yaw_deg)
    T[0, 0] = np.cos(rad)
    T[0, 1] = -np.sin(rad)
    T[1, 0] = np.sin(rad)
    T[1, 1] = np.cos(rad)
    T[:3, 3] = [x, y, z]
    return T


def _room_cloud(z_offset: float = 0.0) -> np.ndarray:
    """Asymmetric dense 3D room (floor + 3 orthogonal walls) for surface & ICP observability tests."""
    xs = np.linspace(-2.5, 4.5, 45)
    ys = np.linspace(-2.0, 3.0, 35)
    zs = np.linspace(0.05, 2.05, 21) + z_offset
    gx, gy = np.meshgrid(xs, ys)
    floor = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, z_offset)])
    gy_w, gz_w = np.meshgrid(ys, zs)
    wall_px = np.column_stack([np.full(gy_w.size, 4.5), gy_w.ravel(), gz_w.ravel()])
    wall_nx = np.column_stack([np.full(gy_w.size, -2.5), gy_w.ravel(), gz_w.ravel()])
    gx_w, gz_wy = np.meshgrid(xs, zs)
    wall_py = np.column_stack([gx_w.ravel(), np.full(gx_w.size, 3.0), gz_wy.ravel()])
    return np.vstack([floor, wall_px, wall_nx, wall_py])


def _corridor_cloud() -> np.ndarray:
    """Featureless corridor along X (floor + two Y walls only, zero X constraint)."""
    xs = np.linspace(-10.0, 10.0, 81)
    ys = np.linspace(-2.0, 2.0, 21)
    zs = np.linspace(0.05, 2.05, 15)
    gx, gy = np.meshgrid(xs, ys)
    floor = np.column_stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)])
    gwx, gwz = np.meshgrid(xs, zs)
    wall_pos = np.column_stack([gwx.ravel(), np.full(gwx.size, 2.0), gwz.ravel()])
    wall_neg = np.column_stack([gwx.ravel(), np.full(gwx.size, -2.0), gwz.ravel()])
    return np.vstack([floor, wall_pos, wall_neg])


class BoundedMeasuredSubmapsTests(unittest.TestCase):
    def _make_kf(
        self,
        seq: int,
        stamp_s: float,
        *,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw_deg: float = 0.0,
        epoch: int = 1,
    ) -> Keyframe:
        pts = np.array([[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [2.0, 0.0, 0.5]], dtype=np.float64)
        return Keyframe(
            key=(epoch, seq),
            stamp_ns=int(round(stamp_s * 1e9)),
            T_odom_body=_yaw_pose(x=x, y=y, z=z, yaw_deg=yaw_deg),
            T_body_lidar=np.eye(4),
            points_lidar=pts,
        )

    def test_motion_bounded_submap_excludes_distant_time_and_large_yaw_frames(self) -> None:
        """Neighbor selection respects submap_window, |dt| <= submap_max_span_s, and rotation <= submap_max_yaw_span_deg."""
        # Anchor is frame 3 at t=10.0s, yaw=0 deg.
        # Frame 1 (t=4.5s, |dt|=5.5s > 4.0s) -> excluded by span bound even though consecutive gap <= 3.0s.
        # Frame 2 (t=8.5s, |dt|=1.5s, yaw=25 deg <= 35 deg) -> included! (note 25 deg > legacy 0.35 rad)
        # Frame 4 (t=11.5s, |dt|=1.5s, yaw=15 deg) -> included!
        # Frame 5 (t=13.0s, |dt|=3.0s, yaw=40 deg > 35 deg) -> excluded by yaw span bound!
        frames = [
            self._make_kf(1, 4.5, x=0.1, yaw_deg=5.0),
            self._make_kf(2, 8.5, x=0.5, yaw_deg=25.0),
            self._make_kf(3, 10.0, x=1.0, yaw_deg=0.0),
            self._make_kf(4, 11.5, x=1.4, yaw_deg=15.0),
            self._make_kf(5, 13.0, x=1.8, yaw_deg=40.0),
        ]
        cfg = SubmapConfig(
            submap_window=2,
            anchor_mode="candidate",
            submap_max_span_s=4.0,
            submap_max_yaw_span_deg=35.0,
            max_gap_s=4.5,
        )
        submap = build_submap(frames, frames[2], cfg)
        self.assertEqual(submap.source_keys, ((1, 3), (1, 2), (1, 4)))
        self.assertGreater(len(submap.points_body), 0)
        self.assertEqual(len(submap.point_sources), len(submap.points_body))

        # Check forward span exclusion too: frame 6 at t=14.5s (|dt|=4.5s > 4.0s) must be cut off.
        frames_fwd = [
            self._make_kf(2, 8.5, x=0.5, yaw_deg=10.0),
            self._make_kf(3, 10.0, x=1.0, yaw_deg=0.0),
            self._make_kf(4, 12.0, x=1.3, yaw_deg=10.0),
            self._make_kf(6, 14.5, x=1.6, yaw_deg=10.0),
        ]
        submap_fwd = build_submap(frames_fwd, frames_fwd[1], cfg)
        self.assertEqual(submap_fwd.source_keys, ((1, 3), (1, 2), (1, 4)))

    def test_failed_keyframe_does_not_pollute_query_submap_cache(self) -> None:
        backend = SlamBackend(BackendConfig(use_measured_submaps=True))
        ok_kf = self._make_kf(1, 1.0, x=0.0)
        bad_kf = Keyframe(
            key=(1, 2),
            stamp_ns=2_000_000_000,
            T_odom_body=np.eye(4),
            T_body_lidar=np.eye(4),
            points_lidar=np.empty((0, 3), dtype=np.float64),
        )
        backend.ingest(ok_kf)
        ev_bad = backend.ingest(bad_kf)
        self.assertIn("error", ev_bad)
        self.assertEqual([f.key for f in backend._source_frames], [(1, 1)])


class SoftSurfaceGateTests(unittest.TestCase):
    def test_soft_surface_gate_passes_aligned_and_rejects_misaligned_or_low_coverage(self) -> None:
        soft_cfg = SurfaceConfig(
            cell_m=0.35,
            min_points=5,
            max_plane_rmse_m=0.035,
            min_spread_m=0.025,
            normal_tolerance_rad=0.25,
            normal_cos_min=0.82,
            merge_distance_m=0.05,
            correspondence_m=0.35,
            max_residual_m=0.08,
            min_coverage=0.35,
            min_patches=1,
        )
        ref = model_surfaces(_room_cloud(z_offset=0.0), config=soft_cfg)

        # Slightly tilted (25 deg around X -> cos(25 deg) = 0.906 >= 0.82) partial patch still within max_residual
        # Test true aligned with 2 cm jitter passes:
        aligned = model_surfaces(_room_cloud(z_offset=0.02), config=soft_cfg)
        ok = compare_surfaces(aligned, ref, soft_cfg)
        self.assertTrue(ok.accepted)
        self.assertEqual(ok.reason, "consistent")

        # Vertical Z shift (wrong floor level by 0.4 m) -> surface_mismatch
        shifted = model_surfaces(_room_cloud(z_offset=0.40), config=soft_cfg)
        bad_shift = compare_surfaces(shifted, ref, soft_cfg)
        self.assertFalse(bad_shift.accepted)
        self.assertEqual(bad_shift.reason, "surface_mismatch")

        # Low-coverage query: floor extends far beyond target [-2.5, 4.5] -> [-12, 12]
        xs_big = np.linspace(-12.0, 12.0, 161)
        ys_big = np.linspace(-12.0, 12.0, 161)
        gx, gy = np.meshgrid(xs_big, ys_big)
        big_floor = np.column_stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)])
        bad_cov = compare_surfaces(model_surfaces(big_floor, config=soft_cfg), ref, soft_cfg)
        self.assertFalse(bad_cov.accepted)
        self.assertEqual(bad_cov.reason, "surface_mismatch")

    def test_surface_gate_works_with_single_frame_and_bounded_measured_submaps(self) -> None:
        loop = F.closed_loop_with_drift(drift_m=1.0, step_m=1.0)
        rows = F.keyframe_stream(loop["scene"], loop["odom"], epoch=7)

        for use_submaps in (False, True):
            with self.subTest(use_measured_submaps=use_submaps):
                cfg = BackendConfig(
                    use_measured_submaps=use_submaps,
                    measured_submap=SubmapConfig(
                        submap_window=2,
                        anchor_mode="candidate",
                        submap_max_span_s=4.0,
                        submap_max_yaw_span_deg=35.0,
                        max_translation_m=2.5,
                    ),
                    use_surface_verification=True,
                    gravity_aligned=True,
                    map_voxel_m=0.12,
                    fine_icp_voxel_m=0.10,
                    surface=SurfaceConfig(
                        cell_m=0.35,
                        min_points=4,
                        max_plane_rmse_m=0.05,
                        min_spread_m=0.02,
                        normal_tolerance_rad=0.25,
                        normal_cos_min=0.82,
                        merge_distance_m=0.06,
                        correspondence_m=0.35,
                        max_residual_m=0.08,
                        min_coverage=0.35,
                        min_patches=1,
                    ),
                    loop_sigma_rad=0.03,
                    loop_sigma_yaw_rad=0.01,
                    cauchy_scale=3.0,
                )
                backend = SlamBackend(cfg)
                events = [backend.ingest(Keyframe(**row)) for row in rows]
                self.assertFalse([e for e in events if "error" in e])
                stats = backend.observability_stats()
                self.assertGreater(stats["surface_checks"], 0)
                if use_submaps:
                    self.assertGreater(stats["measured_query_submaps"], 0)
                    self.assertGreater(stats["measured_target_submaps"], 0)

    def test_query_and_target_submaps_have_disjoint_source_keys(self) -> None:
        """Even with exclude_recent=1 and min_loop_gap_s=0.0, query and target submaps never share a key."""
        room = _room_cloud()
        cfg = BackendConfig(
            use_measured_submaps=True,
            measured_submap=SubmapConfig(
                submap_window=2,
                anchor_mode="candidate",
                max_frames=5,
                max_translation_m=5.0,
                max_gap_s=10.0,
            ),
            min_loop_travel_m=0.0,
            min_loop_spacing_m=0.0,
            min_loop_gap_s=0.0,
            exclude_recent=1,
            min_fitness=0.5,
            max_rmse_m=0.15,
        )
        backend = SlamBackend(cfg)
        poses = [
            _yaw_pose(x=0.0),
            _yaw_pose(x=0.4),
            _yaw_pose(x=0.8),
            _yaw_pose(x=0.05),
        ]
        for idx, T in enumerate(poses):
            ev = backend.ingest(
                Keyframe((1, idx), idx * 1_000_000_000, T, np.eye(4), transform_points(np.linalg.inv(T), room))
            )
            self.assertNotIn("error", ev)
        self.assertTrue(backend._last_loop_query_keys)
        self.assertTrue(backend._last_loop_target_keys)
        self.assertTrue(
            set(backend._last_loop_query_keys).isdisjoint(backend._last_loop_target_keys),
            f"overlap: query={backend._last_loop_query_keys}, target={backend._last_loop_target_keys}",
        )

    def test_evaluation_separates_surface_and_degenerate_rejections_from_odom(self) -> None:
        from scipy.spatial.transform import Rotation

        from embodied_navigation.localization_mapping.backend.evaluation import (
            ReplayRecord,
            TruthTrajectory,
            evaluate_replay,
        )

        n = 6
        stamps = np.arange(n, dtype=np.float64)
        poses = np.repeat(np.eye(4)[None, :, :], n, axis=0)
        poses[:, 0, 3] = stamps
        record = ReplayRecord(
            stamps_s=stamps,
            keys=[(0, i) for i in range(n)],
            frontend_poses=poses,
            online_adopted_poses=poses,
            online_graph_poses=poses,
            final_poses=poses,
            snapshot_kf_indices=np.array([n - 1], dtype=np.int64),
            snapshot_is_loop=np.array([False], dtype=bool),
            snapshot_pos=poses[:, :3, 3][None, :, :].astype(np.float32),
            snapshot_quat_xyzw=np.zeros((1, n, 4), dtype=np.float32),
            step_wall_s=np.full(n, 0.025),
            loop_events=[
                {"kf_idx": 1, "loop": False, "loop_rejected": "odom", "loop_rejected_kind": "odom"},
                {"kf_idx": 2, "loop": False, "loop_rejected": "surface", "loop_rejected_kind": "surface"},
                {"kf_idx": 3, "loop": False, "loop_rejected": "degen", "loop_rejected_kind": "degenerate"},
                {"kf_idx": 4, "loop": False, "loop_rejected": "tilt", "loop_rejected_kind": "tilt"},
            ],
            errors=0,
            missing_graph_nodes=0,
            total_wall_s=0.1,
        )
        record.snapshot_quat_xyzw[:, :, 3] = 1.0
        truth = TruthTrajectory(
            stamps_s=stamps,
            positions=poses[:, :3, 3],
            rotations=Rotation.from_matrix(poses[:, :3, :3]),
        )
        summary, _ = evaluate_replay(record, truth, init_window_s=2.0)
        rt = summary["runtime"]
        self.assertEqual(rt["loops_rejected_odom"], 1)
        self.assertEqual(rt["loops_rejected_tilt"], 1)
        self.assertEqual(rt["loops_rejected_surface"], 1)
        self.assertEqual(rt["loops_rejected_degenerate"], 1)


class DegeneracyZuptAndPresetsTests(unittest.TestCase):
    def test_degenerate_corridor_hessian_rejected_when_enabled(self) -> None:
        """Corridor with zero X-observability must be rejected and counted in loops_rejected_degenerate."""
        corridor = _corridor_cloud()

        def _run_corridor(min_obs: float | None):
            cfg = BackendConfig(
                min_loop_travel_m=0.0,
                min_loop_spacing_m=0.0,
                min_loop_gap_s=1.0,
                exclude_recent=1,
                loop_submap_neighbors=0,
                min_fitness=0.5,
                max_rmse_m=0.1,
                min_translation_observability=min_obs,
            )
            backend = SlamBackend(cfg)
            backend.ingest(Keyframe((1, 0), 0, _yaw_pose(x=0.0), np.eye(4), corridor))
            # Intermediate distant frame outside corridor overlap so exclude_recent=1 leaves only kf 0
            T1 = _yaw_pose(x=6.0)
            backend.ingest(
                Keyframe((1, 1), 2_000_000_000, T1, np.eye(4), transform_points(np.linalg.inv(T1), corridor))
            )
            T2 = _yaw_pose(x=0.05)
            ev2 = backend.ingest(
                Keyframe((1, 2), 4_000_000_000, T2, np.eye(4), transform_points(np.linalg.inv(T2), corridor))
            )
            return backend, ev2

        b_no_gate, ev_ok = _run_corridor(None)
        self.assertTrue(ev_ok["loop"])
        self.assertEqual(b_no_gate.observability_stats()["loops_rejected_degenerate"], 0)

        b_gated, ev_rej = _run_corridor(0.02)
        self.assertFalse(ev_rej["loop"])
        self.assertEqual(ev_rej.get("loop_rejected_kind"), "degenerate")
        self.assertGreaterEqual(b_gated.observability_stats()["loops_rejected_degenerate"], 1)

        # Well-constrained 3D room must pass the same 0.02 observability gate.
        room = _room_cloud()
        cfg_room = BackendConfig(
            min_loop_travel_m=0.0,
            min_loop_spacing_m=0.0,
            min_loop_gap_s=1.0,
            exclude_recent=1,
            loop_submap_neighbors=0,
            min_fitness=0.5,
            max_rmse_m=0.1,
            min_translation_observability=0.02,
        )
        b_room = SlamBackend(cfg_room)
        b_room.ingest(Keyframe((1, 0), 0, _yaw_pose(x=0.0), np.eye(4), room))
        T1 = _yaw_pose(x=6.0)
        b_room.ingest(Keyframe((1, 1), 2_000_000_000, T1, np.eye(4), transform_points(np.linalg.inv(T1), room)))
        T2 = _yaw_pose(x=0.05)
        ev_room = b_room.ingest(Keyframe((1, 2), 4_000_000_000, T2, np.eye(4), transform_points(np.linalg.inv(T2), room)))
        self.assertTrue(ev_room["loop"])
        self.assertEqual(b_room.observability_stats()["loops_rejected_degenerate"], 0)

    def test_python_icp_fallback_returns_hessian_matching_cpp(self) -> None:
        target = _room_cloud()
        T_true = se3_exp(np.array([0.2, -0.15, 0.05, 0.01, -0.02, 0.1]))
        source = transform_points(np.linalg.inv(T_true), target)
        T_init = np.eye(4)
        r_py = icp_point_to_plane(source, target, T_init, voxel_m=0.2, return_hessian=True, use_cpp=False)
        r_cpp = icp_point_to_plane(source, target, T_init, voxel_m=0.2, return_hessian=True, use_cpp=True)
        self.assertIsNotNone(r_py.hessian)
        self.assertIsNotNone(r_cpp.hessian)
        self.assertEqual(r_py.hessian.shape, (6, 6))
        np.testing.assert_allclose(r_py.hessian[:3, :3], r_cpp.hessian[:3, :3], rtol=0.05, atol=5.0)
        eig_py = np.linalg.eigvalsh(r_py.hessian[:3, :3])
        eig_cpp = np.linalg.eigvalsh(r_cpp.hessian[:3, :3])
        self.assertAlmostEqual(float(eig_py[0] / eig_py[-1]), float(eig_cpp[0] / eig_cpp[-1]), delta=0.02)

    def test_zupt_voxel_decoupled_and_fine_icp_voxel_respected(self) -> None:
        """ZUPT must downsample at >= 0.2 m even when map_voxel_m=0.12, while fine ICP uses fine_icp_voxel_m=0.10."""
        room = _room_cloud()
        cfg = BackendConfig.from_json(REPO_ROOT / "configs/localization_mapping/slam_v3_surface_submap_fine.json")
        self.assertAlmostEqual(cfg.map_voxel_m, 0.12)
        self.assertAlmostEqual(cfg.fine_icp_voxel_m, 0.10)
        backend = SlamBackend(cfg)

        voxel_sizes_in_zupt: list[float] = []
        orig_voxel = voxel_downsample

        def _spy_voxel(points, voxel_m, *args, **kwargs):
            voxel_sizes_in_zupt.append(float(voxel_m))
            return orig_voxel(points, voxel_m, *args, **kwargs)

        backend.ingest(Keyframe((1, 0), 0, _yaw_pose(x=0.0), np.eye(4), room))
        with patch(
            "embodied_navigation.localization_mapping.backend.backend.voxel_downsample",
            side_effect=_spy_voxel,
        ):
            ev_zupt = backend.ingest(
                Keyframe((1, 1), 1_000_000_000, _yaw_pose(x=0.0, yaw_deg=2.0), np.eye(4), transform_points(_yaw_pose(yaw_deg=-2.0), room))
            )
        self.assertEqual(ev_zupt.get("zupt"), "pivot")
        self.assertTrue(voxel_sizes_in_zupt)
        self.assertTrue(all(v >= 0.2 for v in voxel_sizes_in_zupt), voxel_sizes_in_zupt)

        # Verify _register passes fine_icp_voxel_m=0.10 to fine ICP while coarse stays 2*icp_voxel_m.
        icp_voxels: list[float] = []
        orig_icp = icp_point_to_plane

        def _spy_icp(*args, **kwargs):
            icp_voxels.append(float(kwargs["voxel_m"]))
            return orig_icp(*args, **kwargs)

        with patch(
            "embodied_navigation.localization_mapping.backend.backend.icp_point_to_plane",
            side_effect=_spy_icp,
        ):
            res = backend._register(room, room, np.eye(4))
        self.assertIsNotNone(res)
        self.assertEqual(icp_voxels, [2 * cfg.icp_voxel_m, 0.10])

    def test_presets_load_cleanly_and_strict_validation_rejects_unknown_keys(self) -> None:
        gate_cfg = BackendConfig.from_json(
            REPO_ROOT / "configs/localization_mapping/slam_v3_surface_submap_gate.json"
        )
        self.assertTrue(gate_cfg.use_measured_submaps)
        self.assertEqual(gate_cfg.measured_submap.submap_window, 2)
        self.assertEqual(gate_cfg.measured_submap.anchor_mode, "candidate")
        self.assertAlmostEqual(gate_cfg.measured_submap.submap_max_span_s, 4.0)
        self.assertAlmostEqual(gate_cfg.measured_submap.submap_max_yaw_span_deg, 35.0)
        self.assertTrue(gate_cfg.use_surface_verification)
        self.assertTrue(gate_cfg.gravity_aligned)
        self.assertAlmostEqual(gate_cfg.surface.min_coverage, 0.35)
        self.assertAlmostEqual(gate_cfg.surface.normal_cos_min, 0.82)
        self.assertAlmostEqual(gate_cfg.min_translation_observability, 0.02)
        self.assertIsNone(gate_cfg.fine_icp_voxel_m)

        fine_cfg = BackendConfig.from_json(
            REPO_ROOT / "configs/localization_mapping/slam_v3_surface_submap_fine.json"
        )
        self.assertAlmostEqual(fine_cfg.map_voxel_m, 0.12)
        self.assertAlmostEqual(fine_cfg.fine_icp_voxel_m, 0.10)
        self.assertAlmostEqual(fine_cfg.min_translation_observability, 0.02)

        for bad in (
            {"unknown_top_level": 1},
            {"measured_submap": {"unknown_submap_key": 1}},
            {"surface": {"unknown_surface_key": 1}},
            {"use_surface_verification": True, "gravity_aligned": False},
            {"min_translation_observability": 0.0},
            {"min_translation_observability": 1.5},
            {"min_translation_observability": True},
            {"fine_icp_voxel_m": 0.0},
            {"fine_icp_voxel_m": -0.1},
            {"surface": {"normal_cos_min": 0.0}},
            {"surface": {"normal_cos_min": 1.2}},
            {"measured_submap": {"submap_max_span_s": 0.0}},
            {"measured_submap": {"submap_max_yaw_span_deg": -5.0}},
            {"measured_submap": {"submap_window": -1}},
        ):
            with self.assertRaises(ValueError, msg=repr(bad)):
                BackendConfig.from_dict(bad)


if __name__ == "__main__":
    unittest.main()
