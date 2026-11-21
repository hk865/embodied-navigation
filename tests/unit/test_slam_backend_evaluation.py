"""Unit tests for the ADR-0004 3-layer offline SLAM backend evaluator."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from slam_backend_fixtures import pose_deg as _pose
from embodied_navigation.localization_mapping.backend.evaluation import (
    ReplayRecord,
    TruthTrajectory,
    compute_initial_se3_alignment,
    compute_trajectory_metrics,
    compute_umeyama_se3_alignment,
    evaluate_replay,
    load_truth_stream,
    save_evaluation_bundle,
)


def _make_truth(n: int = 40, dt: float = 0.5) -> tuple[np.ndarray, TruthTrajectory, np.ndarray]:
    stamps = np.arange(n, dtype=np.float64) * dt
    poses = np.stack(
        [
            _pose(
                [0.5 * t, 0.2 * np.sin(0.3 * t), 0.1 * t],
                rpy_deg=(1.0 * np.sin(0.2 * t), -2.0, 5.0 * t),
            )
            for t in stamps
        ],
        axis=0,
    )
    truth = TruthTrajectory(
        stamps_s=stamps,
        positions=poses[:, :3, 3].copy(),
        rotations=Rotation.from_matrix(poses[:, :3, :3]),
    )
    return stamps, truth, poses


class SlamBackendEvaluationTests(unittest.TestCase):
    def test_zero_error_under_arbitrary_world_frame_offset(self) -> None:
        stamps, truth, truth_poses = _make_truth(30, dt=0.5)
        offset = _pose([12.5, -4.2, 3.1], rpy_deg=(2.0, -3.5, 47.0))
        est = np.einsum("ij,njk->nik", np.linalg.inv(offset), truth_poses)

        A_init = compute_initial_se3_alignment(est, truth_poses, stamps, init_window_s=2.0)
        A_um = compute_umeyama_se3_alignment(est, truth_poses)
        for A in (A_init, A_um):
            aligned = np.einsum("ij,njk->nik", A, est)
            metrics, _ = compute_trajectory_metrics(aligned, truth_poses, stamps)
            self.assertLess(metrics["ate_3d_m"]["max"], 1e-9)
            self.assertLess(metrics["aoe_deg"]["max"], 1e-7)
            self.assertLess(metrics["tilt_deg"]["max"], 1e-7)
            self.assertLess(metrics["rpe_distance"]["5m"]["rte_max_m"], 1e-9)
            self.assertLess(metrics["rpe_time"]["5s"]["roe_max_deg"], 1e-7)

    def test_detects_gravity_tilt_and_z_drift_and_layer23_revisions(self) -> None:
        stamps, truth, truth_poses = _make_truth(30, dt=1.0)
        n = len(stamps)
        # Frontend has mild linear Z drift
        frontend = truth_poses.copy()
        frontend[:, 2, 3] += np.linspace(0.0, 0.30, n)

        # Snapshot 0 (k=14): no loop yet, equals frontend
        # Snapshot 1 (k=29, loop): introduces a 6 deg roll tilt & -0.7 m Z jump on the second half
        final = frontend.copy()
        tilt_rot = Rotation.from_euler("x", 6.0, degrees=True).as_matrix()
        final[15:, :3, :3] = np.einsum("ij,njk->nik", tilt_rot, truth_poses[15:, :3, :3])
        final[15:, 2, 3] -= 0.70

        snap_pos = np.full((2, n, 3), np.nan, dtype=np.float32)
        snap_quat = np.full((2, n, 4), np.nan, dtype=np.float32)
        snap_pre_pos = np.full((2, n, 3), np.nan, dtype=np.float32)
        snap_pre_quat = np.full((2, n, 4), np.nan, dtype=np.float32)
        snap_pos[0, :15] = frontend[:15, :3, 3]
        snap_quat[0, :15] = Rotation.from_matrix(frontend[:15, :3, :3]).as_quat()
        snap_pre_pos[1, : n - 1] = frontend[: n - 1, :3, 3]
        snap_pre_quat[1, : n - 1] = Rotation.from_matrix(frontend[: n - 1, :3, :3]).as_quat()
        snap_pos[1, :n] = final[:, :3, 3]
        snap_quat[1, :n] = Rotation.from_matrix(final[:, :3, :3]).as_quat()

        record = ReplayRecord(
            stamps_s=stamps,
            keys=[(1, i) for i in range(n)],
            frontend_poses=frontend,
            online_adopted_poses=frontend.copy(),
            online_graph_poses=frontend.copy(),
            final_poses=final,
            snapshot_kf_indices=np.array([14, 29], dtype=np.int32),
            snapshot_is_loop=np.array([False, True], dtype=bool),
            snapshot_pos=snap_pos,
            snapshot_quat_xyzw=snap_quat,
            step_wall_s=np.full(n, 0.01, dtype=np.float64),
            loop_events=[{"kf_idx": 29, "stamp_s": float(stamps[-1]), "loop": True, "loop_key": [1, 0], "adopted": True}],
            total_wall_s=0.3,
            snapshot_pre_pos=snap_pre_pos,
            snapshot_pre_quat_xyzw=snap_pre_quat,
        )

        summary, arrays = evaluate_replay(record, truth, init_window_s=0.0, bucket_s=1.0)
        final_m = summary["layer1_init_se3"]["final"]
        # 6 deg world-X tilt must be caught cleanly by tilt_deg
        self.assertGreater(final_m["tilt_deg"]["max"], 5.5)
        self.assertLess(final_m["tilt_deg"]["max"], 6.5)
        # Negative optimization loop step must be counted in Layer 2
        self.assertEqual(summary["layer2_evolution"]["negative_loop_steps"], 1)
        self.assertGreater(summary["layer2_evolution"]["worst_loop_ate_jump_m"], 0.1)
        # Layer 3 bucket count matches 30 one-second buckets
        self.assertEqual(summary["layer3_revisions"]["buckets"], 30)
        self.assertIn("bucket_err_p50_m", arrays)

        with tempfile.TemporaryDirectory() as tmp:
            out = save_evaluation_bundle(Path(tmp), summary, arrays, export_tum=True)
            self.assertTrue((out / "summary.json").is_file())
            self.assertTrue((out / "metrics.npz").is_file())
            tum_lines = (out / "tum" / "final.tum").read_text().strip().splitlines()
            self.assertEqual(len(tum_lines), n)
            self.assertEqual(len(tum_lines[0].split()), 8)

    def test_load_truth_stream_jsonl_and_interpolate(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "truth.jsonl"
            rows = [
                {"time": 1.0, "position": [0.0, 0.0, 0.0], "quat_wxyz": [1.0, 0.0, 0.0, 0.0]},
                {"time": 2.0, "position": [2.0, 0.0, 0.0], "quat_wxyz": [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]},
            ]
            path.write_text("\n".join(json.dumps(r) for r in rows) + "\n")
            truth = load_truth_stream(path)
            mid = truth.interpolate(np.array([1.5]))[0]
            np.testing.assert_allclose(mid[:3, 3], [1.0, 0.0, 0.0], atol=1e-9)
            yaw_deg = Rotation.from_matrix(mid[:3, :3]).as_euler("xyz", degrees=True)[2]
            self.assertAlmostEqual(yaw_deg, 45.0, places=5)

    def test_imu_lever_arm_pure_yaw_gives_zero_base_ate(self) -> None:
        # P0-1 regression: robot rotates 180 deg in place around base origin while IMU sits
        # at imu_in_base_xyz; without origin conversion ATE is > 2 cm, with conversion ATE = 0.
        n = 19
        stamps = np.arange(n, dtype=np.float64) * 1.0
        base_poses = np.stack([_pose([0.0, 0.0, 0.3], rpy_deg=(0.0, 0.0, 10.0 * i)) for i in range(n)], axis=0)
        imu_in_base = np.array([0.0201530643, 0.0, -0.0051089553], dtype=np.float64)
        T_base_imu = np.eye(4, dtype=np.float64)
        T_base_imu[:3, 3] = imu_in_base
        imu_poses = np.einsum("nij,jk->nik", base_poses, T_base_imu)

        truth = TruthTrajectory(
            stamps_s=stamps,
            positions=base_poses[:, :3, 3].copy(),
            rotations=Rotation.from_matrix(base_poses[:, :3, :3]),
        )
        snap_pos = np.full((1, n, 3), np.nan, dtype=np.float32)
        snap_quat = np.full((1, n, 4), np.nan, dtype=np.float32)
        snap_pos[0] = imu_poses[:, :3, 3]
        snap_quat[0] = Rotation.from_matrix(imu_poses[:, :3, :3]).as_quat()
        record = ReplayRecord(
            stamps_s=stamps,
            keys=[(1, i) for i in range(n)],
            frontend_poses=imu_poses,
            online_adopted_poses=imu_poses.copy(),
            online_graph_poses=imu_poses.copy(),
            final_poses=imu_poses.copy(),
            snapshot_kf_indices=np.array([n - 1], dtype=np.int32),
            snapshot_is_loop=np.array([False], dtype=bool),
            snapshot_pos=snap_pos,
            snapshot_quat_xyzw=snap_quat,
            step_wall_s=np.full(n, 0.01, dtype=np.float64),
        )
        s_raw, _ = evaluate_replay(record, truth, init_window_s=0.0)
        self.assertGreater(s_raw["layer1_init_se3"]["final"]["ate_3d_m"]["max"], 0.03)

        s_base, _ = evaluate_replay(record, truth, init_window_s=0.0, imu_in_base_xyz=imu_in_base)
        self.assertEqual(s_base["alignment"]["eval_origin"], "base_via_imu_in_base")
        self.assertLess(s_base["layer1_init_se3"]["frontend"]["ate_3d_m"]["max"], 1e-12)
        self.assertLess(s_base["layer1_init_se3"]["final"]["ate_3d_m"]["max"], 1e-12)

    def test_extending_prefix_without_graph_change_has_zero_loop_delta(self) -> None:
        # P0-3 regression: appending high-error new frames without altering historical nodes
        # must report 0.0 same-prefix loop delta and 0 negative loop steps.
        stamps, truth, truth_poses = _make_truth(20, dt=1.0)
        n = len(stamps)
        est = truth_poses.copy()
        est[10:, 2, 3] += 0.40  # drift in later frames, never modified by loop

        snap_pos = np.full((2, n, 3), np.nan, dtype=np.float32)
        snap_quat = np.full((2, n, 4), np.nan, dtype=np.float32)
        snap_pre_pos = np.full((2, n, 3), np.nan, dtype=np.float32)
        snap_pre_quat = np.full((2, n, 4), np.nan, dtype=np.float32)
        snap_pos[0, :10] = est[:10, :3, 3]
        snap_quat[0, :10] = Rotation.from_matrix(est[:10, :3, :3]).as_quat()
        snap_pre_pos[1, : n - 1] = est[: n - 1, :3, 3]
        snap_pre_quat[1, : n - 1] = Rotation.from_matrix(est[: n - 1, :3, :3]).as_quat()
        snap_pos[1, :n] = est[:, :3, 3]
        snap_quat[1, :n] = Rotation.from_matrix(est[:, :3, :3]).as_quat()

        record = ReplayRecord(
            stamps_s=stamps,
            keys=[(1, i) for i in range(n)],
            frontend_poses=est,
            online_adopted_poses=est.copy(),
            online_graph_poses=est.copy(),
            final_poses=est.copy(),
            snapshot_kf_indices=np.array([9, n - 1], dtype=np.int32),
            snapshot_is_loop=np.array([False, True], dtype=bool),
            snapshot_pos=snap_pos,
            snapshot_quat_xyzw=snap_quat,
            step_wall_s=np.full(n, 0.01, dtype=np.float64),
            snapshot_pre_pos=snap_pre_pos,
            snapshot_pre_quat_xyzw=snap_pre_quat,
        )
        summary, arrays = evaluate_replay(record, truth, init_window_s=0.0)
        self.assertEqual(summary["layer2_evolution"]["negative_loop_steps"], 0)
        self.assertAlmostEqual(summary["layer2_evolution"]["worst_loop_ate_jump_m"], 0.0, places=6)
        self.assertAlmostEqual(float(arrays["same_prefix_loop_delta_ate_m"][0]), 0.0, places=6)

    def test_layer3_captures_first_revision_from_insertion_pose_and_json_has_no_nan(self) -> None:
        # P1-13 & P2-4: keyframe 2 inserted at online_graph_poses[2] and moved before snapshot at idx 4;
        # short trajectory has empty 100s RPE window which must serialize as null (no NaN).
        stamps, truth, truth_poses = _make_truth(6, dt=1.0)
        n = len(stamps)
        online_g = truth_poses.copy()
        final = truth_poses.copy()
        final[2, 2, 3] += 0.25  # shifted before snapshot at idx 5

        snap_pos = np.full((2, n, 3), np.nan, dtype=np.float32)
        snap_quat = np.full((2, n, 4), np.nan, dtype=np.float32)
        snap_pos[0, :1] = online_g[:1, :3, 3]
        snap_quat[0, :1] = Rotation.from_matrix(online_g[:1, :3, :3]).as_quat()
        snap_pos[1, :n] = final[:, :3, 3]
        snap_quat[1, :n] = Rotation.from_matrix(final[:, :3, :3]).as_quat()

        record = ReplayRecord(
            stamps_s=stamps,
            keys=[(1, i) for i in range(n)],
            frontend_poses=online_g,
            online_adopted_poses=online_g.copy(),
            online_graph_poses=online_g,
            final_poses=final,
            snapshot_kf_indices=np.array([0, n - 1], dtype=np.int32),
            snapshot_is_loop=np.array([False, False], dtype=bool),
            snapshot_pos=snap_pos,
            snapshot_quat_xyzw=snap_quat,
            step_wall_s=np.full(n, 0.01, dtype=np.float64),
        )
        summary, arrays = evaluate_replay(record, truth, init_window_s=0.0)
        self.assertGreater(summary["layer3_revisions"]["max_shift_m"], 0.24)
        self.assertGreater(summary["layer3_revisions"]["max_single_step_shift_m"], 0.24)
        self.assertIsNone(summary["layer1_init_se3"]["final"]["rpe_time"]["100s"]["rte_rmse_m"])
        with tempfile.TemporaryDirectory() as tmp:
            out = save_evaluation_bundle(Path(tmp), summary, arrays, export_tum=False, loop_events=[])
            text = (out / "summary.json").read_text(encoding="utf-8")
            self.assertNotIn("NaN", text)
            self.assertTrue((out / "loop_events.json").is_file())


if __name__ == "__main__":
    unittest.main()
