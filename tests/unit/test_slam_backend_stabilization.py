"""Unit tests for Phase 4 SLAM backend stabilization (B1 loop throttling, B2 gravity prior, B3 ZUPT)."""

from __future__ import annotations

import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from slam_backend_fixtures import pose_deg as _make_pose
from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe
from embodied_navigation.localization_mapping.backend.map_store import MapStore
from embodied_navigation.localization_mapping.backend.pose_graph import PoseGraph, hat, se3_exp
from embodied_navigation.localization_mapping.backend.registration import IcpResult, icp_point_to_plane
from embodied_navigation.localization_mapping.backend.scan_context import descriptor


def _room_cloud(seed: int = 7) -> np.ndarray:
    """Rich 3D room (floor + 3 orthogonal walls) in world coordinates."""
    rng = np.random.default_rng(seed)
    xs = np.linspace(-4.0, 4.0, 18)
    ys = np.linspace(-4.0, 4.0, 18)
    zs = np.linspace(0.0, 2.8, 10)
    floor = np.column_stack([np.repeat(xs, len(ys)), np.tile(ys, len(xs)), np.zeros(len(xs) * len(ys))])
    wall_x = np.column_stack([np.full(len(ys) * len(zs), 4.0), np.repeat(ys, len(zs)), np.tile(zs, len(ys))])
    wall_y = np.column_stack([np.repeat(xs, len(zs)), np.full(len(xs) * len(zs), 4.0), np.tile(zs, len(xs))])
    wall_nx = np.column_stack([np.full(len(ys) * len(zs), -4.0), np.repeat(ys, len(zs)), np.tile(zs, len(ys))])
    pts = np.vstack([floor, wall_x, wall_y, wall_nx])
    return pts + rng.normal(0.0, 0.002, size=pts.shape)


class SlamBackendStabilizationTests(unittest.TestCase):
    def test_pose_graph_gravity_prior_jacobian_matches_finite_difference(self) -> None:
        T = _make_pose([1.2, -3.4, 0.5], rpy_deg=(7.5, -5.2, 41.0))
        g_body = Rotation.from_euler("xyz", (2.0, -3.0, 0.0), degrees=True).as_matrix()[2, :]
        R = T[:3, :3]
        J_analytic = np.zeros((3, 6), dtype=np.float64)
        J_analytic[:, 3:] = -(R @ hat(g_body))

        eps = 1e-6
        J_fd = np.zeros((3, 6), dtype=np.float64)
        for col in range(6):
            delta = np.zeros(6)
            delta[col] = eps
            Tp = T @ se3_exp(delta)
            Tm = T @ se3_exp(-delta)
            ep = (Tp[:3, :3] @ g_body) - np.array([0.0, 0.0, 1.0])
            em = (Tm[:3, :3] @ g_body) - np.array([0.0, 0.0, 1.0])
            J_fd[:, col] = (ep - em) / (2.0 * eps)

        np.testing.assert_allclose(J_analytic, J_fd, atol=1e-6)
        # Pure world-yaw perturbation (rotation around g_body in body frame) must lie in nullspace.
        yaw_vec = np.r_[0.0, 0.0, 0.0, g_body]
        np.testing.assert_allclose(J_analytic @ yaw_vec, np.zeros(3), atol=1e-12)

    def test_pose_graph_gravity_prior_penalizes_inverted_pose(self) -> None:
        graph = PoseGraph(cauchy_scale=3.0)
        graph.add_node(0, np.eye(4))
        graph.add_node(1, _make_pose([0.0, 0.0, 0.0], rpy_deg=(180.0, 0.0, 0.0)))
        graph.add_gravity_prior(1, np.array([0.0, 0.0, 1.0]), sigma_rad=0.1, robust=False)
        # Inverted pose must have strictly positive cost (4 / sigma^2 = 400.0), never zero!
        self.assertAlmostEqual(graph._cost(graph.poses), 400.0, places=5)

    def test_pose_graph_gravity_prior_removes_tilt_while_preserving_yaw_and_translation(self) -> None:
        graph = PoseGraph(cauchy_scale=3.0)
        T0 = _make_pose([0.0, 0.0, 0.0], rpy_deg=(0.0, 0.0, 0.0))
        # Node 1 initialized with a bogus 8 deg roll + 6 deg pitch tilt, 35 deg yaw, at (2, 1, 0.5)
        T1_bad = _make_pose([2.0, 1.0, 0.5], rpy_deg=(8.0, 6.0, 35.0))
        graph.add_node(0, T0)
        graph.add_node(1, T1_bad)
        # Weak odometry edge matching T1_bad
        weak_info = np.diag([100.0, 100.0, 100.0, 25.0, 25.0, 10000.0])
        graph.add_edge(0, 1, T1_bad, weak_info, robust=False)
        # Strong gravity prior saying body's up vector is [0, 0, 1] (robot is actually level)
        graph.add_gravity_prior(1, np.array([0.0, 0.0, 1.0]), sigma_rad=0.01, robust=False)
        res = graph.optimize(20)
        self.assertTrue(res["converged"])

        T1_opt = graph.pose(1)
        u_world = T1_opt[:3, :3] @ np.array([0.0, 0.0, 1.0])
        tilt_deg = float(np.degrees(np.arctan2(np.hypot(u_world[0], u_world[1]), u_world[2])))
        self.assertLess(tilt_deg, 0.25)
        # Yaw and translation must remain close to 35 deg and (2, 1, 0.5)
        yaw_deg = float(Rotation.from_matrix(T1_opt[:3, :3]).as_euler("xyz", degrees=True)[2])
        self.assertAlmostEqual(yaw_deg, 35.0, delta=0.5)
        np.testing.assert_allclose(T1_opt[:3, 3], [2.0, 1.0, 0.5], atol=0.05)

    def test_icp_gravity_prior_prevents_degenerate_wall_tilt(self) -> None:
        # Single vertical wall + noisy horizontal floor where unconstrained 6-DoF ICP could tilt easily
        target = _room_cloud(11)
        T_true = _make_pose([0.25, -0.15, 0.05], rpy_deg=(0.0, 0.0, 12.0))
        source = (target - T_true[:3, 3]) @ T_true[:3, :3]  # exact body cloud
        # Bad initial guess tilted by 6 deg roll & -5 deg pitch
        T_init = _make_pose([0.1, 0.0, 0.0], rpy_deg=(6.0, -5.0, 8.0))
        res = icp_point_to_plane(
            source,
            target,
            T_init,
            max_correspondence_m=0.8,
            iterations=30,
            voxel_m=0.2,
            source_up_body=np.array([0.0, 0.0, 1.0]),
            gravity_sigma_rad=0.015,
        )
        self.assertTrue(res.converged)
        u_world = res.T_target_source[:3, :3] @ np.array([0.0, 0.0, 1.0])
        tilt_deg = float(np.degrees(np.arctan2(np.hypot(u_world[0], u_world[1]), u_world[2])))
        self.assertLess(tilt_deg, 0.3)
        np.testing.assert_allclose(res.T_target_source[:3, 3], T_true[:3, 3], atol=0.05)

    def test_b3_zupt_cancels_stationary_and_pivot_z_drift_without_affecting_walking(self) -> None:
        cloud = _room_cloud(3)
        cfg = BackendConfig.from_dict(
            {
                "zupt_enabled": True,
                "zupt_max_odom_xy_m": 0.06,
                "zupt_max_median_dist_m": 0.08,
                "zupt_inlier_dist_m": 0.18,
                "zupt_min_fitness": 0.85,
                "min_loop_travel_m": 5.0,
            }
        )
        backend = SlamBackend(cfg)

        # KF 0: origin
        kf0 = Keyframe((1, 0), 0, np.eye(4), np.eye(4), cloud)
        backend.ingest(kf0)

        # KF 1: physically stationary, but frontend odom drifted -0.18 m in Z!
        T1_drifted = _make_pose([0.005, -0.003, -0.18], rpy_deg=(0.2, -0.1, 0.3))
        kf1 = Keyframe((1, 1), int(2e9), T1_drifted, np.eye(4), cloud)
        ev1 = backend.ingest(kf1)
        self.assertEqual(ev1.get("zupt"), "full_stop")
        self.assertAlmostEqual(backend.graph.pose((1, 1))[2, 3], 0.0, places=5)

        # KF 2: in-place yaw pivot by +15 deg, frontend odom drifted another -0.15 m in Z!
        R_piv = Rotation.from_euler("z", 15.0, degrees=True).as_matrix()
        cloud_piv = cloud @ R_piv  # points in body frame rotated by R_piv^T -> world = R_piv @ body
        T2_drifted = _make_pose([0.008, 0.004, -0.33], rpy_deg=(0.2, -0.1, 15.3))
        kf2 = Keyframe((1, 2), int(4e9), T2_drifted, np.eye(4), cloud_piv)
        ev2 = backend.ingest(kf2)
        self.assertEqual(ev2.get("zupt"), "pivot")
        self.assertAlmostEqual(backend.graph.pose((1, 2))[2, 3], 0.0, places=4)

        # KF 3: genuine walking forward +0.8 m -> MUST NOT trigger ZUPT!
        T3_walk = T2_drifted @ _make_pose([0.8, 0.0, 0.0])
        cloud_walk = (cloud - np.array([0.8, 0.0, 0.0])) @ R_piv
        kf3 = Keyframe((1, 3), int(6e9), T3_walk, np.eye(4), cloud_walk)
        ev3 = backend.ingest(kf3)
        self.assertNotIn("zupt", ev3)
        self.assertGreater(np.linalg.norm(backend.graph.pose((1, 3))[:2, 3]), 0.7)

    def test_b1_min_loop_travel_blocks_stationary_self_loops(self) -> None:
        cloud = _room_cloud(5)
        cfg = BackendConfig.from_dict(
            {
                "exclude_recent": 2,
                "min_loop_gap_s": 5.0,
                "min_loop_travel_m": 5.0,
            }
        )
        backend = SlamBackend(cfg)
        # 8 keyframes 4 s apart (span 28 s > min_loop_gap_s) all at the exact same spot (travel = 0 m)
        for i in range(8):
            kf = Keyframe((1, i), int(i * 4e9), np.eye(4), np.eye(4), cloud)
            ev = backend.ingest(kf)
            self.assertFalse(ev["loop"])
        self.assertEqual(backend.loop_count, 0)

    def test_zupt_fails_closed_on_vertical_wall_dominated_true_ascent(self) -> None:
        # Codex P1 counterexample: mostly vertical walls + small floor patch while robot genuinely ascends +0.18 m/frame.
        xs = np.linspace(-3.0, 3.0, 20)
        zs = np.linspace(0.0, 4.0, 20)
        wall1 = np.column_stack([np.full(len(xs) * len(zs), 3.0), np.repeat(xs, len(zs)), np.tile(zs, len(xs))])
        wall2 = np.column_stack([np.repeat(xs, len(zs)), np.full(len(xs) * len(zs), 3.0), np.tile(zs, len(xs))])
        fx = np.linspace(-1.0, 1.0, 6)
        fy = np.linspace(-1.0, 1.0, 6)
        floor = np.column_stack([np.repeat(fx, len(fy)), np.tile(fy, len(fx)), np.zeros(len(fx) * len(fy))])
        world_pts = np.vstack([wall1, wall2, floor])

        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True}))
        for i in range(4):
            z = 0.18 * i
            T = _make_pose([0.0, 0.0, z])
            pts_body = world_pts - np.array([0.0, 0.0, z])
            ev = backend.ingest(Keyframe((1, i), int(i * 2e9), T, np.eye(4), pts_body))
            self.assertNotIn("zupt", ev)
        self.assertAlmostEqual(backend.graph.pose((1, 3))[2, 3], 0.54, places=3)

    def test_zupt_continuous_drift_updates_online_correction_and_handles_tilted_body(self) -> None:
        # Codex P2 #1 & #2: tilted body (25 deg roll) stationary while odom drifts -0.18 m/frame in world Z.
        world_pts = _room_cloud(9)
        R_tilt = Rotation.from_euler("x", 25.0, degrees=True).as_matrix()
        pts_body = world_pts @ R_tilt  # body cloud of stationary 25-deg tilted robot

        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": 0.0}))
        for i in range(6):
            T_odom = np.eye(4)
            T_odom[:3, :3] = R_tilt
            T_odom[2, 3] = -0.18 * i  # continuous vertical drift in odom frame
            ev = backend.ingest(Keyframe((1, i), int(i * 2e9), T_odom, np.eye(4), pts_body))
            if i >= 1:
                self.assertEqual(ev.get("zupt"), "pivot")
            if i >= 2:
                self.assertTrue(ev["adopted"])
                online_pose = backend.T_world_odom @ T_odom
                self.assertAlmostEqual(online_pose[2, 3], 0.0, places=3)

    def test_tilt_gate_filters_candidates_before_rmse_selection(self) -> None:
        # Codex P2 #3: lowest-RMSE candidate is tilted 5 deg, 2nd candidate is level -> 2nd candidate must win!
        cloud = _room_cloud(13)
        backend = SlamBackend(BackendConfig(max_loop_tilt_rad=0.05, exclude_recent=1, min_loop_gap_s=5.0))
        for i in range(3):
            backend.ingest(Keyframe((1, i), int(i * 1e9), np.eye(4), np.eye(4), cloud))
        q = Keyframe((1, 3), int(30e9), np.eye(4), np.eye(4), cloud)
        with (
            patch.object(backend.index, "query", return_value=[((1, 0), 0.1, 0.0), ((1, 1), 0.1, 0.0)]),
            patch.object(
                backend,
                "_register",
                side_effect=[
                    IcpResult(_make_pose([0, 0, 0], (5.0, 0.0, 0.0)), 1.0, 0.01, True),
                    IcpResult(np.eye(4), 1.0, 0.02, True),
                ],
            ),
        ):
            ev = backend.ingest(q)
        self.assertTrue(ev["loop"])
        self.assertEqual(backend.loop_count, 1)
        self.assertEqual(backend.observability_stats()["loops_rejected_tilt"], 1)

    def test_zupt_does_not_freeze_real_5cm_ascent_when_odom_overestimates_18cm(self) -> None:
        # Codex R2 P1: true ascent +5 cm/frame while odom reports +18 cm/frame -> MUST NOT freeze to 0!
        world_pts = _room_cloud(15)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": 0.0}))
        for i in range(4):
            z_true = 0.05 * i
            z_odom = 0.18 * i
            ev = backend.ingest(
                Keyframe((1, i), int(i * 2e9), _make_pose([0.0, 0.0, z_odom]), np.eye(4), world_pts - np.array([0.0, 0.0, z_true]))
            )
            self.assertNotIn("zupt", ev)
        self.assertGreater(backend.graph.pose((1, 3))[2, 3], 0.4)

    def test_zupt_respects_window_s_and_does_not_leak_unconfirmed_loop(self) -> None:
        # Codex R2 P2 #1: window_s=1.5 with 2 s keyframes must not confirm ZUPT adoption.
        cloud = _room_cloud(17)
        backend = SlamBackend(
            BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": 0.0, "window_s": 1.5, "confirmations": 2})
        )
        backend.ingest(Keyframe((1, 0), 0, np.eye(4), np.eye(4), cloud))
        ev1 = backend.ingest(Keyframe((1, 1), int(2e9), _make_pose([0.0, 0.0, -0.18]), np.eye(4), cloud))
        ev2 = backend.ingest(Keyframe((1, 2), int(4e9), _make_pose([0.0, 0.0, -0.36]), np.eye(4), cloud))
        self.assertEqual(ev1.get("zupt"), "pivot")
        self.assertEqual(ev2.get("zupt"), "pivot")
        self.assertFalse(ev2["adopted"])

    def test_zupt_full_stop_preserves_real_small_rotation_under_vertical_drift(self) -> None:
        # Codex R2 P2 #2: true 0.8 deg in-place yaw with +18 cm odom Z drift must pick pivot, not full_stop.
        world_pts = _room_cloud(19)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": 0.026}))
        backend.ingest(Keyframe((1, 0), 0, np.eye(4), np.eye(4), world_pts))
        R_yaw = Rotation.from_euler("z", 0.8, degrees=True).as_matrix()
        T_odom = _make_pose([0.0, 0.0, 0.18], rpy_deg=(0.0, 0.0, 0.8))
        ev = backend.ingest(Keyframe((1, 1), int(2e9), T_odom, np.eye(4), world_pts @ R_yaw))
        self.assertEqual(ev.get("zupt"), "pivot")
        yaw_deg = float(Rotation.from_matrix(backend.graph.pose((1, 1))[:3, :3]).as_euler("xyz", degrees=True)[2])
        self.assertAlmostEqual(yaw_deg, 0.8, delta=0.1)

    def test_max_loop_tilt_does_not_block_global_relocalization(self) -> None:
        # Codex R2/R3 P2: mapping tilt gate & ICP gravity prior must NOT block tilted prior-map relocalization.
        world_pts = _room_cloud(21)
        prior = MapStore(voxel_m=0.2)
        T_prior = _make_pose([0.0, 0.0, 0.0], rpy_deg=(6.0, 0.0, 0.0))
        kf_prior = Keyframe((1, 0), 0, T_prior, np.eye(4), world_pts)
        prior.add(kf_prior, T_prior, descriptor(kf_prior.points_body(), BackendConfig().scan_context))

        cfg = BackendConfig(
            max_loop_tilt_rad=0.05,
            icp_gravity_sigma_rad=0.02,
            confirmations=2,
        )
        backend = SlamBackend(cfg, prior_map=prior)
        q0 = Keyframe((2, 0), int(1e9), np.eye(4), np.eye(4), world_pts)
        q1 = Keyframe((2, 1), int(2e9), np.eye(4), np.eye(4), world_pts)
        backend.ingest(q0)
        ev1 = backend.ingest(q1)
        self.assertTrue(ev1["adopted"])
        self.assertFalse(ev1["lost"])

    def test_zupt_does_not_freeze_slow_continuous_creep(self) -> None:
        # Codex R3 P1 #1: 2 cm/frame continuous creep over 11 frames (22 cm total) must break ZUPT.
        world_pts = _room_cloud(23)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": 0.0}))
        for i in range(12):
            x = 0.02 * i
            T = _make_pose([x, 0.0, 0.0])
            backend.ingest(Keyframe((1, i), int(i * 2e9), T, np.eye(4), world_pts - np.array([x, 0.0, 0.0])))
        self.assertGreater(backend.graph.pose((1, 11))[0, 3], 0.14)

    def test_pending_loop_still_confirms_when_stopping_for_zupt(self) -> None:
        # Codex R3/R4 P1/P2: pending loop correction must confirm during ZUPT even under continuous -18 cm Z drift.
        cloud = _room_cloud(25)
        backend = SlamBackend(
            BackendConfig.from_dict(
                {"zupt_enabled": True, "zupt_full_stop_rot_rad": 0.0, "confirmations": 2, "window_s": 10.0}
            )
        )
        T0 = np.eye(4)
        backend.ingest(Keyframe((1, 0), 0, T0, np.eye(4), cloud))
        T1 = _make_pose([0.5, 0.0, 0.0])
        backend.ingest(Keyframe((1, 1), int(2e9), T1, np.eye(4), cloud - np.array([0.5, 0.0, 0.0])))
        C_loop = _make_pose([0.0, 0.0, 0.30])
        backend.admission.propose(C_loop, T1, int(4e9))
        backend.admission.seed_anchored(C_loop, T1, int(4e9))
        backend.graph.poses[-1] = C_loop @ T1
        # Next frame stops (ZUPT) while frontend odom drifts -0.18 m in Z: loop must still confirm!
        T2_drifted = _make_pose([0.5, 0.0, -0.18])
        ev = backend.ingest(Keyframe((1, 2), int(6e9), T2_drifted, np.eye(4), cloud - np.array([0.5, 0.0, 0.0])))
        self.assertTrue(ev["adopted"])
        self.assertAlmostEqual((backend.T_world_odom @ T2_drifted)[2, 3], 0.30, places=3)

    def test_zupt_does_not_erase_2cm_real_ascent_even_when_odom_reports_18cm(self) -> None:
        # Codex R4 P2 #1: 2 cm real ascent vs 18 cm odom must NOT be treated as stationary.
        world_pts = _room_cloud(29)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": 0.0}))
        for i in range(6):
            ev = backend.ingest(
                Keyframe(
                    (1, i),
                    int(i * 2e9),
                    _make_pose([0.0, 0.0, 0.18 * i]),
                    np.eye(4),
                    world_pts - np.array([0.0, 0.0, 0.02 * i]),
                )
            )
            self.assertNotIn("zupt", ev)

    def test_startup_zupt_respects_jump_gate_against_published_identity(self) -> None:
        # Codex R4 P2 #4: startup ZUPT must jump-check against the already-published identity C.
        cloud = _room_cloud(31)
        backend = SlamBackend(
            BackendConfig.from_dict(
                {
                    "zupt_enabled": True,
                    "zupt_full_stop_rot_rad": 0.0,
                    "loop_max_jump_translation_m": 0.25,
                    "confirmations": 2,
                }
            )
        )
        backend.ingest(Keyframe((1, 0), 0, np.eye(4), np.eye(4), cloud))
        backend.ingest(Keyframe((1, 1), int(2e9), _make_pose([0.0, 0.0, -0.18]), np.eye(4), cloud))
        ev2 = backend.ingest(Keyframe((1, 2), int(4e9), _make_pose([0.0, 0.0, -0.36]), np.eye(4), cloud))
        # 0.36 m exceeds loop_max_jump_translation_m=0.25 m -> must be rejected, not adopted!
        self.assertFalse(ev2["adopted"])
        self.assertGreaterEqual(len(backend.admission.rejections), 1)
        # Codex R5 P2: resuming walking must STILL enforce the 0.25 m jump gate against published identity!
        ev3 = backend.ingest(
            Keyframe((1, 3), int(6e9), _make_pose([0.5, 0.0, -0.36]), np.eye(4), cloud - np.array([0.5, 0.0, 0.0]))
        )
        ev4 = backend.ingest(
            Keyframe((1, 4), int(8e9), _make_pose([1.0, 0.0, -0.36]), np.eye(4), cloud - np.array([1.0, 0.0, 0.0]))
        )
        self.assertFalse(ev3["adopted"])
        self.assertFalse(ev4["adopted"])

    def test_zupt_exit_recovers_cumulative_motion_under_noisy_slow_ascent(self) -> None:
        # Codex R5 P1: 2 cm/frame true ascent with 1 cm point noise over 30 frames (0.60 m total).
        rng = np.random.default_rng(42)
        base_room = _room_cloud(35)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": 0.0}))
        for i in range(31):
            z = 0.02 * i
            noisy = base_room - np.array([0.0, 0.0, z]) + rng.normal(0.0, 0.01, size=base_room.shape)
            backend.ingest(Keyframe((1, i), int(i * 2e9), _make_pose([0.0, 0.0, z]), np.eye(4), noisy))
        self.assertGreater(backend.graph.pose((1, 30))[2, 3], 0.54)

    def test_full_stop_checks_cumulative_rotation_from_anchor(self) -> None:
        # Codex R4 P3 #1: slow 0.1 deg/frame rotation must accumulate past anchor full_stop threshold.
        world_pts = _room_cloud(33)
        backend = SlamBackend(
            BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": np.radians(0.5)})
        )
        for i in range(15):
            ang = 0.1 * i
            R = Rotation.from_euler("z", ang, degrees=True).as_matrix()
            T = _make_pose([0.0, 0.0, 0.0], (0.0, 0.0, ang))
            backend.ingest(Keyframe((1, i), int(i * 2e9), T, np.eye(4), world_pts @ R))
        yaw_deg = float(Rotation.from_matrix(backend.graph.pose((1, 14))[:3, :3]).as_euler("xyz", degrees=True)[2])
        self.assertGreater(yaw_deg, 0.9)

    def test_propose_anchored_enforces_rotation_agreement(self) -> None:
        # Codex R3 P2 #2: full_stop rotation drift exceeding agree_rotation_rad must not confirm.
        cloud = _room_cloud(27)
        backend = SlamBackend(
            BackendConfig.from_dict(
                {
                    "zupt_enabled": True,
                    "zupt_full_stop_rot_rad": 0.05,
                    "agree_rotation_rad": 0.01,
                    "confirmations": 2,
                }
            )
        )
        backend.ingest(Keyframe((1, 0), 0, np.eye(4), np.eye(4), cloud))
        # Stationary cloud, but frontend yaw drifts +1 deg (0.0175 rad > 0.01 rad) per frame
        ev1 = backend.ingest(Keyframe((1, 1), int(2e9), _make_pose([0, 0, 0], (0, 0, 1.0)), np.eye(4), cloud))
        ev2 = backend.ingest(Keyframe((1, 2), int(4e9), _make_pose([0, 0, 0], (0, 0, 2.0)), np.eye(4), cloud))
        self.assertEqual(ev1.get("zupt"), "full_stop")
        self.assertEqual(ev2.get("zupt"), "full_stop")
        self.assertFalse(ev2["adopted"])

    def test_zupt_exit_separates_stationary_drift_from_real_ascent(self) -> None:
        # Codex R6 P2 #2: repeated drifted stops followed by noisy 2 cm slow ascent must recover ~0.36 m.
        rng = np.random.default_rng(42)
        cloud = _room_cloud(35)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": 0.0}))
        i = 0
        for j in range(9):
            for k in range(5):
                z = 0.04 * j + 0.02 * max(k - 2, 0)
                drift = 0.36 * j + 0.18 * min(k, 2)
                x = 0.14 * j
                T = _make_pose([x, 0.0, z - drift])
                pts = cloud - np.array([x, 0.0, z]) + rng.normal(0.0, 0.01, size=cloud.shape)
                backend.ingest(Keyframe((1, i), int(i * 2e9), T, np.eye(4), pts))
                i += 1
        self.assertGreater(backend.graph.poses[-1][2, 3], 0.30)

    def test_full_stop_rejects_bogus_yaw_drift_past_threshold(self) -> None:
        # Codex R6 P2 #3: static cloud with 1.6 deg odom yaw drift must stay full_stop, not pivot.
        cloud = _room_cloud(37)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": np.radians(1.5)}))
        backend.ingest(Keyframe((1, 0), 0, np.eye(4), np.eye(4), cloud))
        backend.ingest(Keyframe((1, 1), int(2e9), _make_pose([0, 0, 0], (0, 0, 1.0)), np.eye(4), cloud))
        ev2 = backend.ingest(Keyframe((1, 2), int(4e9), _make_pose([0, 0, 0], (0, 0, 1.6)), np.eye(4), cloud))
        self.assertEqual(ev2.get("zupt"), "full_stop")
        yaw_deg = abs(float(Rotation.from_matrix(backend.graph.pose((1, 2))[:3, :3]).as_euler("xyz", degrees=True)[2]))
        self.assertLess(yaw_deg, 0.05)

    def test_full_stop_turn_back_to_anchor_uses_anchor_frame(self) -> None:
        # Codex R7 P2 #1: 0 deg -> 15 deg pivot -> 0 deg turn-back must return to 0 deg, not 15 deg.
        world_pts = _room_cloud(39)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True}))
        R15 = Rotation.from_euler("z", 15.0, degrees=True).as_matrix()
        backend.ingest(Keyframe((1, 0), 0, np.eye(4), np.eye(4), world_pts))
        ev1 = backend.ingest(Keyframe((1, 1), int(2e9), _make_pose([0, 0, 0], (0, 0, 15.0)), np.eye(4), world_pts @ R15))
        ev2 = backend.ingest(Keyframe((1, 2), int(4e9), np.eye(4), np.eye(4), world_pts))
        self.assertEqual(ev1.get("zupt"), "pivot")
        self.assertEqual(ev2.get("zupt"), "full_stop")
        yaw_deg = abs(float(Rotation.from_matrix(backend.graph.pose((1, 2))[:3, :3]).as_euler("xyz", degrees=True)[2]))
        self.assertLess(yaw_deg, 0.05)

    def test_zupt_exit_rotates_init_shift_into_anchor_frame_after_90deg_pivot(self) -> None:
        # Codex R7 P2 #2: 90 deg pivot in place then 20 cm walk forward in body-X (= world +Y).
        world_pts = _room_cloud(41)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True}))
        R90 = Rotation.from_euler("z", 90.0, degrees=True).as_matrix()
        backend.ingest(Keyframe((1, 0), 0, np.eye(4), np.eye(4), world_pts))
        ev1 = backend.ingest(Keyframe((1, 1), int(2e9), _make_pose([0, 0, 0], (0, 0, 90.0)), np.eye(4), world_pts @ R90))
        self.assertEqual(ev1.get("zupt"), "pivot")
        # Walk +0.20 m along body-X (= world +Y) with clean cloud
        pts2 = (world_pts - np.array([0.0, 0.20, 0.0])) @ R90
        ev2 = backend.ingest(Keyframe((1, 2), int(4e9), _make_pose([0.0, 0.20, 0.0], (0, 0, 90.0)), np.eye(4), pts2))
        self.assertIsNone(ev2.get("zupt"))
        pos = backend.graph.pose((1, 2))[:3, 3]
        self.assertAlmostEqual(float(pos[0]), 0.0, delta=0.02)
        self.assertAlmostEqual(float(pos[1]), 0.20, delta=0.02)
        self.assertAlmostEqual(float(pos[2]), 0.0, delta=0.02)

    def test_full_stop_exit_preserves_eliminated_yaw_drift(self) -> None:
        # Codex R7 P2 #2: static cloud with 6 deg bogus odom yaw drift followed by 6.5 cm walk along world +X.
        world_pts = _room_cloud(43)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": np.radians(1.5)}))
        backend.ingest(Keyframe((1, 0), 0, np.eye(4), np.eye(4), world_pts))
        ev1 = backend.ingest(Keyframe((1, 1), int(2e9), _make_pose([0, 0, 0], (0, 0, 6.0)), np.eye(4), world_pts))
        self.assertEqual(ev1.get("zupt"), "full_stop")
        # Now robot walks +0.065 m along true world +X while frontend keeps the +6 deg bias (delta_step = pure +0.065 X)
        R6 = Rotation.from_euler("z", 6.0, degrees=True).as_matrix()
        t_odom2 = R6 @ np.array([0.065, 0.0, 0.0])
        pts2 = world_pts - np.array([0.065, 0.0, 0.0])
        ev2 = backend.ingest(Keyframe((1, 2), int(4e9), _make_pose(t_odom2.tolist(), (0, 0, 6.0)), np.eye(4), pts2))
        self.assertIsNone(ev2.get("zupt"))
        pose2 = backend.graph.pose((1, 2))
        yaw_deg = abs(float(Rotation.from_matrix(pose2[:3, :3]).as_euler("xyz", degrees=True)[2]))
        self.assertLess(yaw_deg, 0.1)
        self.assertAlmostEqual(float(pose2[0, 3]), 0.065, delta=0.02)

    def test_pivot_after_drifted_full_stop_uses_anchor_rotation(self) -> None:
        # Codex R8 P2: true 0 deg -> 0 deg -> 15 deg, odom 0 deg -> 6 deg -> 15 deg must land on 15 deg, not 9 deg.
        world_pts = _room_cloud(45)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": np.radians(1.5)}))
        R15 = Rotation.from_euler("z", 15.0, degrees=True).as_matrix()
        backend.ingest(Keyframe((1, 0), 0, np.eye(4), np.eye(4), world_pts))
        ev1 = backend.ingest(Keyframe((1, 1), int(2e9), _make_pose([0, 0, 0], (0, 0, 6.0)), np.eye(4), world_pts))
        ev2 = backend.ingest(Keyframe((1, 2), int(4e9), _make_pose([0, 0, 0], (0, 0, 15.0)), np.eye(4), world_pts @ R15))
        self.assertEqual(ev1.get("zupt"), "full_stop")
        self.assertEqual(ev2.get("zupt"), "pivot")
        yaw_deg = float(Rotation.from_matrix(backend.graph.pose((1, 2))[:3, :3]).as_euler("xyz", degrees=True)[2])
        self.assertAlmostEqual(yaw_deg, 15.0, delta=0.1)

    def test_pivot_after_biased_full_stop_preserves_eliminated_yaw_drift(self) -> None:
        # Codex R9 P2 #1: true 0 deg -> 0 deg -> 15 deg, odom 0 deg -> 1 deg -> 16 deg must land on 15 deg, not 16 deg.
        world_pts = _room_cloud(47)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": np.radians(1.5)}))
        R15 = Rotation.from_euler("z", 15.0, degrees=True).as_matrix()
        backend.ingest(Keyframe((1, 0), 0, np.eye(4), np.eye(4), world_pts))
        ev1 = backend.ingest(Keyframe((1, 1), int(2e9), _make_pose([0, 0, 0], (0, 0, 1.0)), np.eye(4), world_pts))
        ev2 = backend.ingest(Keyframe((1, 2), int(4e9), _make_pose([0, 0, 0], (0, 0, 16.0)), np.eye(4), world_pts @ R15))
        self.assertEqual(ev1.get("zupt"), "full_stop")
        self.assertEqual(ev2.get("zupt"), "pivot")
        yaw_deg = float(Rotation.from_matrix(backend.graph.pose((1, 2))[:3, :3]).as_euler("xyz", degrees=True)[2])
        self.assertAlmostEqual(yaw_deg, 15.0, delta=0.1)

    def test_alternating_stationary_and_moving_frames_mutually_confirm_online_correction(self) -> None:
        # Codex R9 P1: alternating ZUPT (+0.18 m Z drift) and moving frames must adopt online C, not stall at identity.
        cloud = _room_cloud(49)
        backend = SlamBackend(
            BackendConfig.from_dict({"zupt_enabled": True, "confirmations": 2, "agree_translation_m": 0.15})
        )
        backend.ingest(Keyframe((1, 0), 0, np.eye(4), np.eye(4), cloud))
        x = 0.0
        drift = 0.0
        for step in range(5):
            drift -= 0.18
            t_s = (2 * step + 1) * 2
            ev_s = backend.ingest(
                Keyframe((1, 2 * step + 1), int(t_s * 1e9), _make_pose([x, 0.0, drift]), np.eye(4), cloud - np.array([x, 0.0, 0.0]))
            )
            self.assertEqual(ev_s.get("zupt"), "full_stop")
            x += 0.30
            t_m = (2 * step + 2) * 2
            ev_m = backend.ingest(
                Keyframe((1, 2 * step + 2), int(t_m * 1e9), _make_pose([x, 0.0, drift]), np.eye(4), cloud - np.array([x, 0.0, 0.0]))
            )
            self.assertTrue(ev_m["adopted"])
        online_pose = backend.T_world_odom @ _make_pose([x, 0.0, drift])
        self.assertAlmostEqual(float(online_pose[2, 3]), 0.0, delta=0.02)

    def test_zupt_exit_retains_recovery_anchor_after_sparse_exit_frame(self) -> None:
        # Codex R9 P2 #2: 2 cm/frame noisy ascent with a sparse 40-pt exit frame recovers full 6 cm on next full frame.
        rng = np.random.default_rng(42)
        cloud = _room_cloud(51)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": 0.0}))
        pts = [cloud - np.array([0.0, 0.0, 0.02 * i]) + rng.normal(0.0, 0.01, size=cloud.shape) for i in range(4)]
        backend.ingest(Keyframe((1, 0), 0, _make_pose([0.0, 0.0, 0.00]), np.eye(4), pts[0]))
        ev1 = backend.ingest(Keyframe((1, 1), int(2e9), _make_pose([0.0, 0.0, 0.02]), np.eye(4), pts[1]))
        self.assertIsNotNone(ev1.get("zupt"))
        # Sparse 40-point exit frame at z=0.04
        ev2 = backend.ingest(Keyframe((1, 2), int(4e9), _make_pose([0.0, 0.0, 0.04]), np.eye(4), pts[2][:40]))
        self.assertIsNone(ev2.get("zupt"))
        # Next full cloud at z=0.06 must recover full 0.06 m against anchor (1, 0)
        ev3 = backend.ingest(Keyframe((1, 3), int(6e9), _make_pose([0.0, 0.0, 0.06]), np.eye(4), pts[3]))
        self.assertIsNone(ev3.get("zupt"))
        self.assertAlmostEqual(float(backend.graph.pose((1, 3))[2, 3]), 0.06, delta=0.01)

    def test_zupt_exit_recovers_ascent_even_on_large_horizontal_exit_step(self) -> None:
        # Codex R10 P2 #1: 2 cm slow ascent in ZUPT followed by 0.55 m horizontal exit step x 6 recovers ~0.12 m.
        rng = np.random.default_rng(42)
        cloud = _room_cloud(53)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": 0.0}))
        x, z, k = 0.0, 0.0, 0
        backend.ingest(Keyframe((1, k), 0, _make_pose([x, 0.0, z]), np.eye(4), cloud + rng.normal(0.0, 0.01, size=cloud.shape)))
        for _ in range(6):
            k += 1
            z += 0.02
            p1 = cloud - np.array([x, 0.0, z]) + rng.normal(0.0, 0.01, size=cloud.shape)
            backend.ingest(Keyframe((1, k), int(k * 2e9), _make_pose([x, 0.0, z]), np.eye(4), p1))
            k += 1
            x += 0.55
            p2 = cloud - np.array([x, 0.0, z]) + rng.normal(0.0, 0.01, size=cloud.shape)
            backend.ingest(Keyframe((1, k), int(k * 2e9), _make_pose([x, 0.0, z]), np.eye(4), p2))
        self.assertGreater(float(backend.graph.pose((1, k))[2, 3]), 0.09)

    def test_zupt_recovery_turn_back_keeps_cumulative_path_monotonic(self) -> None:
        # Codex R11 P2: stop -> sparse move to +0.80 m -> walk back to +0.15 m (recovers against anchor)
        # must keep cumulative travel path at ~1.45 m, never rewinding to 0.15 m.
        cloud = _room_cloud(55)
        backend = SlamBackend(BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": 0.0}))
        backend.ingest(Keyframe((1, 0), 0, _make_pose([0.0, 0.0, 0.0]), np.eye(4), cloud))
        ev1 = backend.ingest(Keyframe((1, 1), int(2e9), _make_pose([0.0, 0.0, 0.0]), np.eye(4), cloud))
        self.assertIsNotNone(ev1.get("zupt"))
        # Sparse exit frame at x=+0.80 m
        ev2 = backend.ingest(Keyframe((1, 2), int(4e9), _make_pose([0.80, 0.0, 0.0]), np.eye(4), (cloud - np.array([0.80, 0.0, 0.0]))[:40]))
        self.assertIsNone(ev2.get("zupt"))
        # Turn back to x=+0.15 m with full cloud -> aligns against anchor (1, 0)
        ev3 = backend.ingest(Keyframe((1, 3), int(6e9), _make_pose([0.15, 0.0, 0.0]), np.eye(4), cloud - np.array([0.15, 0.0, 0.0])))
        self.assertIsNone(ev3.get("zupt"))
        self.assertAlmostEqual(float(backend.graph.pose((1, 3))[0, 3]), 0.15, delta=0.02)
        self.assertGreater(backend._path_m[(1, 3)], 1.40)

    def test_zupt_recovery_stationary_drift_does_not_double_count_path(self) -> None:
        # Codex R12 P2 #1: sparse stationary frame with +0.22 m Z drift then recovery to 0 must NOT count 0.44 m.
        cloud = _room_cloud(57)
        backend = SlamBackend(
            BackendConfig.from_dict({"zupt_enabled": True, "zupt_full_stop_rot_rad": 0.0, "min_loop_travel_m": 0.30})
        )
        backend.ingest(Keyframe((1, 0), 0, _make_pose([0.0, 0.0, 0.0]), np.eye(4), cloud))
        backend.ingest(Keyframe((1, 1), int(2e9), _make_pose([0.0, 0.0, 0.0]), np.eye(4), cloud))
        # Sparse stationary frame with 0.22 m Z drift
        backend.ingest(Keyframe((1, 2), int(4e9), _make_pose([0.0, 0.0, 0.22]), np.eye(4), cloud[:40]))
        # Full cloud still stationary at z=0 (odom stays at 0.22, step delta=0)
        backend.ingest(Keyframe((1, 3), int(6e9), _make_pose([0.0, 0.0, 0.22]), np.eye(4), cloud))
        self.assertLess(backend._path_m[(1, 3)], 0.25)

    def test_full_stop_exit_preserves_corrected_gravity_prior_under_persistent_roll_bias(self) -> None:
        # Codex R12 P2 #2: persistent 6 deg odom roll bias after full_stop must not pull graph roll back to ~3 deg.
        world_pts = _room_cloud(59)
        backend = SlamBackend(
            BackendConfig.from_dict(
                {"zupt_enabled": True, "zupt_full_stop_rot_rad": np.radians(1.5), "graph_gravity_sigma_rad": 0.02}
            )
        )
        backend.ingest(Keyframe((1, 0), 0, np.eye(4), np.eye(4), world_pts))
        ev1 = backend.ingest(Keyframe((1, 1), int(2e9), _make_pose([0, 0, 0], (6.0, 0.0, 0.0)), np.eye(4), world_pts))
        self.assertEqual(ev1.get("zupt"), "full_stop")
        # Exit walking +0.20 m along world +X while odom retains 6 deg roll bias
        R6 = Rotation.from_euler("x", 6.0, degrees=True).as_matrix()
        t2 = R6 @ np.array([0.20, 0.0, 0.0])
        ev2 = backend.ingest(
            Keyframe((1, 2), int(4e9), _make_pose(t2.tolist(), (6.0, 0.0, 0.0)), np.eye(4), world_pts - np.array([0.20, 0.0, 0.0]))
        )
        self.assertIsNone(ev2.get("zupt"))
        backend.graph.optimize(10)
        roll_deg = abs(float(Rotation.from_matrix(backend.graph.pose((1, 2))[:3, :3]).as_euler("xyz", degrees=True)[0]))
        self.assertLess(roll_deg, 0.2)

    def test_tilt_rejection_is_counted_separately_from_odom_rejection(self) -> None:
        # Codex R12 P2 #3: tilt-gate rejection must be tagged kind="tilt" and not counted as loops_rejected_odom.
        from embodied_navigation.localization_mapping.backend.evaluation import (
            TruthTrajectory,
            evaluate_replay,
            replay_backend,
        )

        cloud = _room_cloud(61)
        kfs = [
            Keyframe((1, 0), 0, _make_pose([0.0, 0.0, 0.0]), np.eye(4), cloud),
            Keyframe((1, 1), int(10e9), _make_pose([1.0, 0.0, 0.0]), np.eye(4), cloud - np.array([1.0, 0.0, 0.0])),
            Keyframe((1, 2), int(25e9), _make_pose([0.0, 0.0, 0.0], (8.0, 0.0, 0.0)), np.eye(4), cloud),
        ]
        cfg = BackendConfig.from_dict({"exclude_recent": 1, "min_loop_gap_s": 1.0, "max_loop_tilt_rad": np.radians(2.0)})
        rec = replay_backend(SlamBackend(cfg), kfs)
        truth = TruthTrajectory(
            stamps_s=np.array([0.0, 10.0, 25.0]),
            positions=np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=np.float64),
            rotations=Rotation.from_matrix(np.stack([np.eye(3), np.eye(3), np.eye(3)], axis=0)),
        )
        summary, _ = evaluate_replay(rec, truth)
        self.assertEqual(summary["runtime"]["loops_accepted"], 0)
        self.assertEqual(summary["runtime"]["loops_rejected_odom"], 0)
        self.assertGreaterEqual(summary["runtime"]["loops_rejected_tilt"], 1)

    def test_icp_gravity_prior_does_not_override_well_observed_floor_plane(self) -> None:
        # Regression: MAP gravity weight must NOT scale with len(points); a well-observed 3D room
        # (floor + walls) must correct a 1.0 deg biased source_up_body instead of locking to it.
        room = _room_cloud(67)
        biased_up = Rotation.from_euler("y", 1.0, degrees=True).as_matrix()[2, :3]
        T_init = _make_pose([0.05, -0.05, 0.05], (0.0, 1.0, 0.0))
        res = icp_point_to_plane(
            room,
            room,
            T_init,
            max_correspondence_m=0.6,
            voxel_m=0.25,
            source_up_body=biased_up,
            gravity_sigma_rad=0.02,
        )
        self.assertTrue(res.converged)
        pitch_deg = abs(float(Rotation.from_matrix(res.T_target_source[:3, :3]).as_euler("xyz", degrees=True)[1]))
        self.assertLess(pitch_deg, 0.15)

    def test_config_strict_validation_rejects_bad_stabilization_values(self) -> None:
        for bad in (
            {"min_loop_travel_m": -1.0},
            {"min_loop_spacing_m": True},
            {"icp_gravity_sigma_rad": 0.0},
            {"icp_gravity_sigma_rad": 1e-200},
            {"graph_gravity_sigma_rad": -0.02},
            {"graph_gravity_sigma_rad": 1e-200},
            {"graph_gravity_sigma_rad": 1e200},
            {"zupt_sigma_m": 1e-200},
            {"graph_gravity_robust": "yes"},
            {"zupt_enabled": 1},
            {"zupt_min_fitness": 1.5},
        ):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    BackendConfig.from_dict(bad)

    def test_pose_graph_exact_se3_jacobians_and_zero_gradient_at_convergence(self) -> None:
        # P0-2 regression: verify exact SE(3) right-Jacobian-inverse derivatives on both endpoints
        # and verify true cost gradient vanishes at convergence on an anisotropic robust two-node graph.
        from embodied_navigation.localization_mapping.backend.backend import anisotropic_information
        from embodied_navigation.localization_mapping.backend.pose_graph import (
            adjoint,
            invert,
            se3_exp,
            se3_log,
            se3_right_jacobian_inv,
        )

        T_a = _make_pose([1.2, -0.7, 0.4], (6.0, -9.0, 28.0))
        T_b = _make_pose([2.7, 0.5, 1.1], (-4.0, 12.0, -17.0))
        Z = _make_pose([0.9, 0.8, 0.3], (2.0, -3.0, 14.0))
        Z_inv = invert(Z)
        e0 = se3_log(Z_inv @ invert(T_a) @ T_b)
        Jr_inv = se3_right_jacobian_inv(e0)
        Ja_ana = -(Jr_inv @ adjoint(invert(T_b) @ T_a))
        Jb_ana = Jr_inv

        eps = 1e-7
        Ja_num = np.zeros((6, 6))
        Jb_num = np.zeros((6, 6))
        for k in range(6):
            d = np.zeros(6)
            d[k] = eps
            Ja_num[:, k] = (
                se3_log(Z_inv @ invert(T_a @ se3_exp(d)) @ T_b)
                - se3_log(Z_inv @ invert(T_a @ se3_exp(-d)) @ T_b)
            ) / (2.0 * eps)
            Jb_num[:, k] = (
                se3_log(Z_inv @ invert(T_a) @ (T_b @ se3_exp(d)))
                - se3_log(Z_inv @ invert(T_a) @ (T_b @ se3_exp(-d)))
            ) / (2.0 * eps)
        np.testing.assert_allclose(Ja_ana, Ja_num, atol=1e-6)
        np.testing.assert_allclose(Jb_ana, Jb_num, atol=1e-6)

        # Two-node graph with one ordinary edge + one conflicting anisotropic robust edge.
        graph = PoseGraph(cauchy_scale=2.0)
        graph.add_node(0, np.eye(4))
        graph.add_node(1, _make_pose([1.0, 0.0, 0.0], (0.0, 0.0, 15.0)))
        info_odom = anisotropic_information(0.05, 0.15, 0.02, 0.05)
        info_loop = anisotropic_information(0.10, 0.25, 0.03, 0.04)
        graph.add_edge(0, 1, _make_pose([1.0, 0.0, 0.0], (0.0, 0.0, 15.0)), info_odom, robust=False)
        graph.add_edge(0, 1, _make_pose([2.0, 0.2, -0.1], (4.0, -5.0, 32.0)), info_loop, robust=True)
        res = graph.optimize(iterations=40)
        self.assertTrue(res["converged"])

        # True finite-difference cost gradient wrt node 1 must be near zero.
        grad_num = np.zeros(6)
        p0, p1 = graph.poses[0], graph.poses[1]
        for k in range(6):
            d = np.zeros(6)
            d[k] = eps
            cp = graph._cost([p0, p1 @ se3_exp(d)])
            cm = graph._cost([p0, p1 @ se3_exp(-d)])
            grad_num[k] = (cp - cm) / (2.0 * eps)
        self.assertLess(float(np.max(np.abs(grad_num))), 1e-4)

    def test_pose_graph_rejects_duplicate_node_key(self) -> None:
        graph = PoseGraph()
        graph.add_node((1, 0), np.eye(4))
        with self.assertRaises(ValueError):
            graph.add_node((1, 0), np.eye(4))

    def test_icp_is_invariant_to_large_world_origin_offset(self) -> None:
        # P2-1 regression: shifting target and T_init by +300 m along X must not trigger rank failure.
        room = _room_cloud(71)
        offset = np.array([300.0, -150.0, 25.0], dtype=np.float64)
        target_shifted = room + offset
        T_init = _make_pose(offset + np.array([0.08, -0.06, 0.04]), (0.5, -0.5, 2.0))
        res = icp_point_to_plane(room, target_shifted, T_init, max_correspondence_m=0.8, voxel_m=0.25)
        self.assertTrue(res.converged)
        self.assertGreater(res.fitness, 0.95)
        np.testing.assert_allclose(res.T_target_source[:3, 3], offset, atol=0.02)

    def test_exclude_recent_zero_allows_loop_closure(self) -> None:
        # P1-9a regression: exclude_recent=0 must NOT exclude all history via keys[-0:].
        room = _room_cloud(73)
        cfg = BackendConfig(
            exclude_recent=0,
            min_loop_gap_s=0.0,
            min_loop_travel_m=0.0,
            min_loop_spacing_m=0.0,
            zupt_enabled=False,
        )
        backend = SlamBackend(cfg)
        backend.ingest(Keyframe((1, 0), 0, np.eye(4), np.eye(4), room))
        ev = backend.ingest(Keyframe((1, 1), 1_000_000_000, _make_pose([0.05, 0.0, 0.0]), np.eye(4), room))
        self.assertTrue(ev["loop"])

    def test_correction_admission_rejects_backward_stamp_and_non_mutual_3_cluster(self) -> None:
        # P1-10 & P2-3 regressions: backward timestamps cannot confirm, and confirmations=3
        # requires pairwise mutual agreement within the cluster.
        from embodied_navigation.localization_mapping.backend.correction import CorrectionAdmission

        adm = CorrectionAdmission(confirmations=3, window_s=10.0, agree_translation_m=0.15)
        body = np.eye(4)
        self.assertIsNone(adm.propose(_make_pose([-0.14, 0.0, 0.0]), body, 1_000_000_000))
        self.assertIsNone(adm.propose(_make_pose([0.14, 0.0, 0.0]), body, 2_000_000_000))
        # 0.0 agrees with both -0.14 and +0.14 individually, but |-0.14 - 0.14| = 0.28 > 0.15!
        self.assertIsNone(adm.propose(_make_pose([0.0, 0.0, 0.0]), body, 3_000_000_000))
        # Backward timestamp must not confirm either.
        adm2 = CorrectionAdmission(confirmations=2, window_s=10.0, agree_translation_m=0.15)
        self.assertIsNone(adm2.propose(np.eye(4), body, 100_000_000_000))
        self.assertIsNone(adm2.propose(np.eye(4), body, 1_000_000_000))

    def test_all_committed_stabilization_presets_load_cleanly(self) -> None:
        from pathlib import Path

        cfg_dir = Path(__file__).resolve().parents[2] / "configs" / "localization_mapping"
        presets = sorted(cfg_dir.glob("slam_backend_*.json"))
        self.assertGreaterEqual(len(presets), 4)
        for path in presets:
            cfg = BackendConfig.from_json(path)
            self.assertIsInstance(cfg, BackendConfig)
        stab = BackendConfig.from_json(cfg_dir / "slam_backend_stabilized.json")
        self.assertIsNone(stab.graph_gravity_sigma_rad)
        self.assertAlmostEqual(stab.icp_gravity_sigma_rad, 0.02)
        self.assertAlmostEqual(stab.min_loop_travel_m, 5.0)

    def test_observability_stats_tracks_travel_loop_age_and_zupt_active(self) -> None:
        room = _room_cloud(79)
        cfg = BackendConfig(
            exclude_recent=0,
            min_loop_gap_s=0.0,
            min_loop_travel_m=0.0,
            min_loop_spacing_m=0.0,
            zupt_enabled=True,
            zupt_skip_loop_search=True,
        )
        backend = SlamBackend(cfg)
        s0 = backend.observability_stats()
        self.assertEqual(s0["travel_m"], 0.0)
        self.assertEqual(s0["travel_since_last_loop_m"], 0.0)
        self.assertIsNone(s0["last_loop_age_s"])
        self.assertFalse(s0["zupt_active"])

        # Frame 0 at origin
        backend.ingest(Keyframe((1, 0), 10_000_000_000, np.eye(4), np.eye(4), room))
        # Frame 1 stationary -> triggers ZUPT full_stop
        ev1 = backend.ingest(Keyframe((1, 1), 11_000_000_000, _make_pose([0.0, 0.0, 0.02]), np.eye(4), room))
        self.assertEqual(ev1.get("zupt"), "full_stop")
        s1 = backend.observability_stats()
        self.assertTrue(s1["zupt_active"])
        self.assertEqual(s1["travel_m"], 0.0)
        self.assertEqual(s1["travel_since_last_loop_m"], 0.0)
        self.assertIsNone(s1["last_loop_age_s"])

        # Frame 2 walks +0.25 m -> exits ZUPT and closes loop against frame 0/1
        T2 = _make_pose([0.25, 0.0, 0.0])
        pts2 = (room - T2[:3, 3]) @ T2[:3, :3]
        ev2 = backend.ingest(Keyframe((1, 2), 12_500_000_000, T2, np.eye(4), pts2))
        self.assertTrue(ev2["loop"], ev2)
        s2 = backend.observability_stats()
        self.assertFalse(s2["zupt_active"])
        self.assertGreater(s2["travel_m"], 0.2)
        self.assertAlmostEqual(s2["travel_since_last_loop_m"], 0.0, places=6)
        self.assertAlmostEqual(s2["last_loop_age_s"], 0.0, places=6)

        # Frame 3 walks further (+1.5 m) with loop spacing blocking a new loop -> travel_since_last_loop_m and age grow
        backend.config = BackendConfig(
            exclude_recent=0,
            min_loop_gap_s=0.0,
            min_loop_travel_m=0.0,
            min_loop_spacing_m=10.0,
            zupt_enabled=True,
        )
        T3 = _make_pose([1.75, 0.0, 0.0])
        pts3 = (room - T3[:3, 3]) @ T3[:3, :3]
        ev3 = backend.ingest(Keyframe((1, 3), 15_000_000_000, T3, np.eye(4), pts3))
        self.assertFalse(ev3["loop"])
        s3 = backend.observability_stats()
        self.assertAlmostEqual(s3["travel_since_last_loop_m"], 1.5, delta=0.05)
        self.assertAlmostEqual(s3["last_loop_age_s"], 2.5, places=6)

        # Backward stamp after loop must report last_loop_age_s=None instead of 0.0
        T4 = _make_pose([2.4, 0.0, 0.0])
        pts4 = (room - T4[:3, 3]) @ T4[:3, :3]
        backend.ingest(Keyframe((1, 4), 10_000_000_000, T4, np.eye(4), pts4))
        s4 = backend.observability_stats()
        self.assertIsNone(s4["last_loop_age_s"])

        # Non-finite path length must sanitize to None so strict JSON serialization never crashes
        backend._path_m[(1, 4)] = float("inf")
        s_inf = backend.observability_stats()
        self.assertIsNone(s_inf["travel_m"])
        self.assertIsNone(s_inf["travel_since_last_loop_m"])

        # Prior-map mode reports None for mapping-only travel metrics
        prior_store = MapStore(0.2)
        prior_store.add(Keyframe((1, 0), 0, np.eye(4), np.eye(4), room), np.eye(4), descriptor(room, cfg.scan_context))
        reloc_backend = SlamBackend(cfg, prior_map=prior_store)
        s_prior = reloc_backend.observability_stats()
        self.assertIsNone(s_prior["travel_m"])
        self.assertIsNone(s_prior["travel_since_last_loop_m"])
        self.assertIsNone(s_prior["last_loop_age_s"])


if __name__ == "__main__":
    unittest.main()
