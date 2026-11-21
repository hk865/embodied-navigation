"""Unit tests for the ADR-0004 3-layer offline SLAM backend evaluator."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

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


def _pose(xyz, rpy_deg=(0.0, 0.0, 0.0)) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_euler("xyz", rpy_deg, degrees=True).as_matrix()
    T[:3, 3] = xyz
    return T


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
        # Snapshot 1 (k=29, loop): introduces a 6 deg roll tilt & -0.5 m Z jump on the second half
        final = frontend.copy()
        tilt_rot = Rotation.from_euler("x", 6.0, degrees=True).as_matrix()
        final[15:, :3, :3] = np.einsum("ij,njk->nik", tilt_rot, truth_poses[15:, :3, :3])
        final[15:, 2, 3] -= 0.50

        snap_pos = np.full((2, n, 3), np.nan, dtype=np.float32)
        snap_quat = np.full((2, n, 4), np.nan, dtype=np.float32)
        snap_pos[0, :15] = frontend[:15, :3, 3]
        snap_quat[0, :15] = Rotation.from_matrix(frontend[:15, :3, :3]).as_quat()
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


if __name__ == "__main__":
    unittest.main()
