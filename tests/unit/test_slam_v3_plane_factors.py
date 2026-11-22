"""Unit tests for floor-plane slab priors, X-ICP loop information, and fine-voxel ICP."""

from __future__ import annotations

from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from slam_backend_fixtures import pose_deg as _make_pose
from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from embodied_navigation.localization_mapping.backend.floor_plane import (
    FloorPlaneMeasurement,
    FloorSlabTracker,
    detect_floor_plane,
)
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe
from embodied_navigation.localization_mapping.backend.pose_graph import se3_exp
from embodied_navigation.localization_mapping.backend.registration import icp_point_to_plane

REPO_ROOT = Path(__file__).resolve().parents[2]


def _flat_floor_body_cloud(
    *,
    height_above_floor_m: float = 0.34,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    noise_m: float = 0.004,
    seed: int = 101,
    include_walls: bool = True,
) -> np.ndarray:
    """Synthetic body-frame scan over a flat horizontal world floor at z=0."""
    rng = np.random.default_rng(seed)
    radii = np.linspace(0.6, 4.2, 18)
    angles = np.linspace(0.0, 2.0 * np.pi, 36, endpoint=False)
    rr, aa = np.meshgrid(radii, angles, indexing="ij")
    fx = (rr * np.cos(aa)).ravel()
    fy = (rr * np.sin(aa)).ravel()
    fz = np.zeros_like(fx)
    world_pts = [np.column_stack([fx, fy, fz])]
    if include_walls:
        wx = np.linspace(-4.0, 4.0, 20)
        wz = np.linspace(0.0, 2.5, 14)
        XX, ZZ = np.meshgrid(wx, wz, indexing="ij")
        world_pts.append(np.column_stack([XX.ravel(), np.full(XX.size, 3.8), ZZ.ravel()]))
        world_pts.append(np.column_stack([np.full(XX.size, 4.8), XX.ravel(), ZZ.ravel()]))
    pts_world = np.vstack(world_pts)
    pts_world = pts_world + rng.normal(0.0, noise_m, size=pts_world.shape)
    R_wb = Rotation.from_euler("xyz", [roll_deg, pitch_deg, 0.0], degrees=True).as_matrix()
    t_wb = np.array([0.0, 0.0, height_above_floor_m], dtype=np.float64)
    return (pts_world - t_wb) @ R_wb


def _corridor_cloud(*, x_len: float = 12.0, half_w: float = 1.6, height: float = 2.6, seed: int = 21) -> np.ndarray:
    """Open-ended corridor along world X (degenerate in X translation, constrained in Y and Z)."""
    rng = np.random.default_rng(seed)
    xs = np.linspace(-x_len * 0.5, x_len * 0.5, 45)
    ys = np.linspace(-half_w, half_w, 13)
    zs = np.linspace(0.0, height, 11)
    XX_f, YY_f = np.meshgrid(xs, ys, indexing="ij")
    floor = np.column_stack([XX_f.ravel(), YY_f.ravel(), np.zeros(XX_f.size)])
    XX_w, ZZ_w = np.meshgrid(xs, zs, indexing="ij")
    wall_p = np.column_stack([XX_w.ravel(), np.full(XX_w.size, half_w), ZZ_w.ravel()])
    wall_n = np.column_stack([XX_w.ravel(), np.full(XX_w.size, -half_w), ZZ_w.ravel()])
    pts = np.vstack([floor, wall_p, wall_n])
    return pts + rng.normal(0.0, 0.003, size=pts.shape)


class FloorPlaneDetectionTests(unittest.TestCase):
    def test_detect_floor_plane_accepts_flat_and_mildly_tilted_body(self) -> None:
        pts = _flat_floor_body_cloud(height_above_floor_m=0.34, roll_deg=1.5, pitch_deg=-2.0, seed=1)
        R = Rotation.from_euler("xyz", [1.5, -2.0, 25.0], degrees=True).as_matrix()
        meas = detect_floor_plane(pts, R)
        self.assertIsNotNone(meas)
        assert meas is not None
        self.assertAlmostEqual(meas.height_below_body_m, 0.34, delta=0.015)
        expected_n = R[:3, :3].T @ np.array([0.0, 0.0, 1.0])
        # Note: R in body cloud had yaw=0, while R_odom_body had yaw=25 deg; compare Z and tilt.
        self.assertGreaterEqual(meas.normal_body[2], 0.985)
        self.assertAlmostEqual(float(np.linalg.norm(meas.normal_body)), 1.0, places=6)
        self.assertLessEqual(meas.residual_mad_m, 0.025)
        self.assertGreaterEqual(meas.point_count, 40)

    def test_detect_floor_plane_rejects_tilted_body_and_stair_cloud(self) -> None:
        flat_pts = _flat_floor_body_cloud(height_above_floor_m=0.34, seed=2)
        R_tilted = Rotation.from_euler("xyz", [0.0, 12.0, 0.0], degrees=True).as_matrix()
        self.assertIsNone(detect_floor_plane(flat_pts, R_tilted))

        # Sloped 22-deg stair ramp in front of level body -> normal_z < 0.985
        xs = np.linspace(0.6, 3.5, 20)
        ys = np.linspace(-1.5, 1.5, 16)
        XX, YY = np.meshgrid(xs, ys, indexing="ij")
        ZZ = -0.55 + 0.40 * (XX - 0.6)  # ~22 deg slope inside [-0.75, -0.08]
        ramp = np.column_stack([XX.ravel(), YY.ravel(), ZZ.ravel()])
        ramp = ramp[(ramp[:, 2] >= -0.75) & (ramp[:, 2] <= -0.08)]
        self.assertIsNone(detect_floor_plane(ramp, np.eye(3)))

        # Stepped stair treads arranged monotonically along X (4 treads in [-0.75, -0.08], ~30 pts each)
        rng = np.random.default_rng(5)
        step_x = rng.uniform(0.6, 1.72, 120)
        step_y = rng.uniform(-1.0, 1.0, 120)
        step_idx = np.floor((step_x - 0.6) / 0.28).astype(int)
        step_z = -0.68 + 0.16 * step_idx
        steps = np.column_stack([step_x, step_y, step_z])
        self.assertIsNone(detect_floor_plane(steps, np.eye(3)))

        # Sparse cloud < min_points
        self.assertIsNone(detect_floor_plane(flat_pts[:20], np.eye(3)))


class FloorSlabTrackerTests(unittest.TestCase):
    def test_keeps_drifted_flat_floor_on_slab0_then_discovers_and_rejoins(self) -> None:
        tracker = FloorSlabTracker(stair_climb_min_m=1.5, slab_rejoin_tol_m=1.2)
        poses_z: dict[int, float] = {}
        meas = FloorPlaneMeasurement(
            height_below_body_m=0.34,
            normal_body=np.array([0.0, 0.0, 1.0]),
            residual_mad_m=0.005,
            point_count=200,
        )

        # Frames 0..29: flat floor with -1.8 m accumulated slow odom drift + 1 bump dropout at frame 12
        for k in range(30):
            odom_z = 0.34 - 0.06 * k
            poses_z[k] = 0.34
            out = tracker.update(
                k,
                None if k == 12 else meas,
                odom_z=odom_z,
                pose_z=poses_z[k],
                pose_z_lookup=poses_z.__getitem__,
            )
            if k in (0, 12):
                self.assertIsNone(out)
            else:
                self.assertIsNotNone(out)
                assert out is not None
                self.assertEqual(out[0], 0)
                self.assertAlmostEqual(out[1], 0.34, places=6)
        self.assertEqual(len(tracker.slabs), 1)
        self.assertEqual(tracker.active_slab_id, 0)

        # Frames 30..36: climb stairs +3.2 m (non-level)
        for idx, k in enumerate(range(30, 37)):
            odom_z = -1.40 + 0.55 * idx
            poses_z[k] = 0.34 + 0.55 * idx
            self.assertIsNone(
                tracker.update(
                    k,
                    None,
                    odom_z=odom_z,
                    pose_z=poses_z[k],
                    pose_z_lookup=poses_z.__getitem__,
                )
            )

        # Frames 37..40: land on Floor 1 (3.54 m body Z => 3.20 m floor Z).
        # Frames 37 and 38 confirm level landing without pulling Floor 1 down to Slab 0.
        for k in (37, 38):
            poses_z[k] = 3.54
            self.assertIsNone(
                tracker.update(
                    k,
                    meas,
                    odom_z=1.90,
                    pose_z=poses_z[k],
                    pose_z_lookup=poses_z.__getitem__,
                )
            )
        # Frame 39 (3rd consecutive level frame) discovers Slab 1 anchored at k=39
        poses_z[39] = 3.54
        self.assertIsNone(
            tracker.update(39, meas, odom_z=1.90, pose_z=3.54, pose_z_lookup=poses_z.__getitem__)
        )
        self.assertEqual(len(tracker.slabs), 2)
        self.assertEqual(tracker.active_slab_id, 1)
        self.assertEqual(tracker.slabs[1].anchor_key, 39)

        # Frame 40 on Floor 1 gets constrained to Slab 1
        poses_z[40] = 3.50
        out40 = tracker.update(40, meas, odom_z=1.86, pose_z=3.50, pose_z_lookup=poses_z.__getitem__)
        self.assertIsNotNone(out40)
        assert out40 is not None
        self.assertEqual(out40[0], 1)
        self.assertAlmostEqual(out40[1], 3.54, places=6)

        # If loop closure shifts Slab 1's anchor pose by +0.12 m, slab_elevation shifts dynamically
        poses_z[39] = 3.66
        self.assertAlmostEqual(tracker.slab_elevation(1, poses_z.__getitem__), 3.32, places=6)

        # Frames 41..46: descend back down -3.2 m to Floor 0
        for idx, k in enumerate(range(41, 47)):
            odom_z = 1.86 - 0.60 * idx
            poses_z[k] = 3.54 - 0.60 * idx
            self.assertIsNone(
                tracker.update(k, None, odom_z=odom_z, pose_z=poses_z[k], pose_z_lookup=poses_z.__getitem__)
            )

        # Frames 47..49: land back near Floor 0 (within 1.2 m of Slab 0 -> rejoins Slab 0!)
        for k in (47, 48):
            poses_z[k] = 0.45
            self.assertIsNone(
                tracker.update(k, meas, odom_z=-1.14, pose_z=0.45, pose_z_lookup=poses_z.__getitem__)
            )
        poses_z[49] = 0.45
        out49 = tracker.update(49, meas, odom_z=-1.14, pose_z=0.45, pose_z_lookup=poses_z.__getitem__)
        self.assertIsNotNone(out49)
        assert out49 is not None
        self.assertEqual(out49[0], 0)
        self.assertEqual(tracker.active_slab_id, 0)
        self.assertEqual(len(tracker.slabs), 2)
        self.assertAlmostEqual(out49[1], 0.34, places=6)


class SlamBackendV3PlaneAndXicpTests(unittest.TestCase):
    def test_floor_plane_eliminates_open_loop_vertical_z_drift(self) -> None:
        pts_body = _flat_floor_body_cloud(height_above_floor_m=0.34, seed=9)
        cfg = BackendConfig.from_dict(
            {
                "floor_plane_enabled": True,
                "floor_plane_sigma_m": 0.03,
                "odometry_sigma_m": 0.2,
                "odometry_sigma_z_m": 0.2,
            }
        )
        backend = SlamBackend(cfg)
        for k in range(25):
            # True height is 0.34 m, but frontend odom drifts -0.03 m/frame (-0.72 m total!).
            T_odom = _make_pose([0.6 * k, 0.0, 0.34 - 0.03 * k])
            ev = backend.ingest(Keyframe((1, k), int(k * 1e9), T_odom, np.eye(4), pts_body))
            self.assertNotIn("error", ev)

        stats = backend.observability_stats()
        self.assertEqual(stats["floor_planes_detected"], 25)
        self.assertEqual(stats["floor_slabs_discovered"], 1)
        self.assertEqual(stats["floor_z_priors_added"], 24)
        z_errors = [abs(backend.graph.pose((1, k))[2, 3] - 0.34) for k in range(25)]
        self.assertLess(max(z_errors), 0.03)
        self.assertIsNotNone(backend.T_world_odom)
        assert backend.T_world_odom is not None
        online_pose = backend.T_world_odom @ _make_pose([0.6 * 24, 0.0, 0.34 - 0.03 * 24])
        self.assertLess(abs(online_pose[2, 3] - 0.34), 0.08)

    def test_floor_normal_gravity_calibration_supplies_measured_up_vector(self) -> None:
        # Cloud observed from a truly level body (normal_body = [0, 0, 1]), while odom attitude has drifted!
        pts_level = _flat_floor_body_cloud(height_above_floor_m=0.34, roll_deg=0.0, pitch_deg=0.0, seed=11)
        cfg = BackendConfig.from_dict(
            {
                "floor_plane_enabled": True,
                "floor_normal_gravity_enabled": True,
                "icp_gravity_sigma_rad": 0.02,
                "exclude_recent": 1,
                "min_loop_gap_s": 1.0,
            }
        )
        backend = SlamBackend(cfg)
        backend.ingest(Keyframe((1, 0), 0, _make_pose([0.0, 0.0, 0.34]), np.eye(4), pts_level))
        # Second frame has +4 deg roll/pitch odom error (within 6 deg tilt gate)
        T_drifted = _make_pose([1.0, 0.0, 0.34], rpy_deg=(4.0, -3.0, 0.0))
        backend.ingest(Keyframe((1, 1), int(2e9), T_drifted, np.eye(4), pts_level))
        # Query frame sees flat floor -> floor_normal_gravity_enabled overrides noisy _up_body
        T_q = _make_pose([2.0, 0.0, 0.34], rpy_deg=(4.0, -3.0, 0.0))
        captured_up: list[np.ndarray | None] = []

        def fake_register(source_body, target_world, T_init, *, source_up_body=None, return_hessian=False):
            captured_up.append(source_up_body)
            return None

        with (
            patch.object(backend.index, "query", return_value=[((1, 0), 0.1, 0.0)]),
            patch.object(backend, "_register", side_effect=fake_register),
        ):
            backend.ingest(Keyframe((1, 2), int(10e9), T_q, np.eye(4), pts_level))
        self.assertEqual(len(captured_up), 1)
        assert captured_up[0] is not None
        np.testing.assert_allclose(captured_up[0], [0.0, 0.0, 1.0], atol=0.01)

    def test_fine_icp_voxel_and_decoupled_zupt_voxel(self) -> None:
        pts_body = _flat_floor_body_cloud(height_above_floor_m=0.34, seed=13)
        cfg = BackendConfig.from_dict(
            {
                "map_voxel_m": 0.10,
                "fine_icp_voxel_m": 0.08,
                "zupt_enabled": True,
                "zupt_full_stop_rot_rad": 0.0,
            }
        )
        backend = SlamBackend(cfg)
        self.assertAlmostEqual(backend.map.voxel_m, 0.08)

        # Stationary frame with Z drift must still trigger ZUPT cleanly at map_voxel_m=0.10
        backend.ingest(Keyframe((1, 0), 0, _make_pose([0.0, 0.0, 0.34]), np.eye(4), pts_body))
        ev1 = backend.ingest(
            Keyframe((1, 1), int(2e9), _make_pose([0.004, -0.002, 0.20]), np.eye(4), pts_body)
        )
        self.assertEqual(ev1.get("zupt"), "pivot")
        self.assertAlmostEqual(backend.graph.pose((1, 1))[2, 3], 0.34, places=4)

    def test_python_and_cpp_icp_return_matching_hessians(self) -> None:
        target = _flat_floor_body_cloud(height_above_floor_m=0.0, seed=17)
        T_true = se3_exp(np.array([0.2, -0.15, 0.05, 0.0, 0.0, 0.12]))
        source = (target - T_true[:3, 3]) @ T_true[:3, :3]
        T_init = se3_exp(np.array([0.05, -0.05, 0.0, 0.0, 0.0, 0.04]))
        r_py = icp_point_to_plane(
            source, target, T_init, max_correspondence_m=0.8, voxel_m=0.2, return_hessian=True, use_cpp=False
        )
        r_cpp = icp_point_to_plane(
            source, target, T_init, max_correspondence_m=0.8, voxel_m=0.2, return_hessian=True, use_cpp=True
        )
        self.assertIsNotNone(r_py.hessian)
        self.assertIsNotNone(r_cpp.hessian)
        assert r_py.hessian is not None and r_cpp.hessian is not None
        np.testing.assert_allclose(r_py.hessian, r_cpp.hessian, rtol=1e-4, atol=1e-4)

    def test_xicp_attenuates_degenerate_corridor_axis_and_preserves_spd(self) -> None:
        corridor = _corridor_cloud()
        cfg = BackendConfig.from_dict(
            {
                "xicp_enabled": True,
                "xicp_min_scale": 0.15,
                "loop_sigma_m": 0.05,
                "loop_sigma_rad": 0.03,
                "loop_sigma_yaw_rad": 0.01,
                "exclude_recent": 1,
                "min_loop_gap_s": 5.0,
            }
        )
        backend = SlamBackend(cfg)
        I_base = cfg.loop_information()
        # Evaluate ICP Hessian on the corridor at yaw=0 and yaw=90 deg
        res_0 = icp_point_to_plane(corridor, corridor, np.eye(4), voxel_m=0.2, return_hessian=True)
        self.assertIsNotNone(res_0.hessian)
        I_eff_0, is_deg_0 = backend._modulate_loop_information(I_base, res_0.hessian, np.eye(3))
        self.assertTrue(is_deg_0)
        np.testing.assert_allclose(I_eff_0, I_eff_0.T, atol=1e-10)
        # Degenerate X axis must be attenuated close to xicp_min_scale * I_base[0, 0], while Y/Z remain high.
        self.assertLess(I_eff_0[0, 0], 0.25 * I_base[0, 0])
        self.assertGreater(I_eff_0[1, 1], 0.50 * I_base[1, 1])
        self.assertGreater(I_eff_0[2, 2], 0.50 * I_base[2, 2])
        # SPD lower bound: min eigenvalue >= xicp_min_scale * min_eig(I_base)
        min_eig_eff = float(np.linalg.eigvalsh(I_eff_0)[0])
        min_eig_base = float(np.linalg.eigvalsh(I_base)[0])
        self.assertGreaterEqual(min_eig_eff, (cfg.xicp_min_scale - 1e-9) * min_eig_base)

        # At yaw=90 deg, world-X corridor degeneracy maps to body-Y in PoseGraph tangent frame
        R_90 = Rotation.from_euler("z", 90.0, degrees=True).as_matrix()
        I_eff_90, is_deg_90 = backend._modulate_loop_information(I_base, res_0.hessian, R_90)
        self.assertTrue(is_deg_90)
        self.assertLess(I_eff_90[1, 1], 0.25 * I_base[1, 1])
        self.assertGreater(I_eff_90[0, 0], 0.50 * I_base[0, 0])

        # Rich 3D room must NOT be flagged degenerate
        room = _flat_floor_body_cloud(height_above_floor_m=0.0, seed=25, include_walls=True)
        res_room = icp_point_to_plane(room, room, np.eye(4), voxel_m=0.2, return_hessian=True)
        _, is_deg_room = backend._modulate_loop_information(I_base, res_room.hessian, np.eye(3))
        self.assertFalse(is_deg_room)

        # End-to-end loop closure in backend increments xicp_degenerate_loops
        backend.ingest(Keyframe((1, 0), 0, np.eye(4), np.eye(4), corridor))
        backend.ingest(Keyframe((1, 1), int(1e9), _make_pose([0.5, 0.0, 0.0]), np.eye(4), corridor))
        with patch.object(backend.index, "query", return_value=[((1, 0), 0.05, 0.0)]):
            ev_loop = backend.ingest(Keyframe((1, 2), int(10e9), np.eye(4), np.eye(4), corridor))
        self.assertTrue(ev_loop["loop"])
        self.assertEqual(backend.observability_stats()["xicp_degenerate_loops"], 1)

    def test_strict_config_validation_and_v3_presets(self) -> None:
        for preset_name in ("slam_v3_plane_factors.json", "slam_v3_plane_factors_tight.json"):
            cfg = BackendConfig.from_json(REPO_ROOT / "configs/localization_mapping" / preset_name)
            self.assertTrue(cfg.floor_plane_enabled)
            self.assertTrue(cfg.xicp_enabled)
            self.assertTrue(cfg.floor_normal_gravity_enabled)
            self.assertIsNotNone(cfg.fine_icp_voxel_m)

        for bad in (
            {"unknown_v3_flag": True},
            {"floor_plane_enabled": "yes"},
            {"floor_normal_gravity_enabled": 1},
            {"xicp_enabled": 0},
            {"floor_plane_sigma_m": 0.0},
            {"floor_plane_max_tilt_deg": 95.0},
            {"floor_plane_stair_climb_min_m": -1.0},
            {"floor_plane_slab_rejoin_tol_m": 0.0},
            {"fine_icp_voxel_m": 0.0},
            {"xicp_min_scale": 1.2},
            {"xicp_min_scale": 0.0},
            {"xicp_cond_floor": -1e-3},
        ):
            with self.assertRaises(ValueError, msg=repr(bad)):
                BackendConfig.from_dict(bad)


if __name__ == "__main__":
    unittest.main()
