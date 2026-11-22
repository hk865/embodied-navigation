"""Unit tests for pct_scan_worker.update_measured_voxels stale Z-drift eviction."""
import unittest
import numpy as np

from embodied_navigation.simulation.dynanav.experiment import slam_odometry_topic
from embodied_navigation.simulation.dynanav.pct_scan_worker import (
    ALLOWED_POSE_SOURCES,
    compute_backend_correction,
    correction_delta_exceeds,
    select_recent_voxels,
    transform_measured_voxels,
    update_measured_voxels,
)


class MeasuredVoxelUpdateTests(unittest.TestCase):
    def test_evicts_stale_adjacent_z_drift_ghost_on_flat_surface(self):
        voxels, stamps = {}, {}
        # Earlier scan from afar observes 2nd-floor slab +6.5 cm high (iz=37)
        old_pts = np.array([[1.02, 9.02, 2.985]])
        update_measured_voxels(voxels, stamps, old_pts, stamp_ns=280_000_000_000)
        self.assertEqual(len(voxels), 1)
        old_key = next(iter(voxels))

        # Newer scan 80 s later observes exact same slab at 2.920 m (iz=36)
        new_pts = np.array([[1.03, 9.03, 2.920]])
        update_measured_voxels(voxels, stamps, new_pts, stamp_ns=360_000_000_000)
        self.assertEqual(len(voxels), 1)
        self.assertNotIn(old_key, voxels)
        self.assertNotIn(old_key, stamps)
        retained = next(iter(voxels.values()))
        self.assertAlmostEqual(float(retained[2]), 2.920, places=6)

    def test_preserves_simultaneous_vertical_wall_and_obstacles_above_10cm(self):
        voxels, stamps = {}, {}
        # Real vertical wall observed simultaneously across adjacent Z voxels
        wall_pts = np.array([
            [1.02, 9.02, 0.02],
            [1.02, 9.02, 0.10],
            [1.02, 9.02, 0.18],
            [1.02, 9.02, 0.40],
        ])
        update_measured_voxels(voxels, stamps, wall_pts, stamp_ns=10_000_000_000)
        self.assertEqual(len(voxels), 4)

        # Even 30 s later, a floor point at 0.01 does NOT evict an obstacle at 0.13 (>0.10 m)
        voxels2, stamps2 = {}, {}
        update_measured_voxels(voxels2, stamps2, np.array([[1.02, 9.02, 0.13]]), stamp_ns=10_000_000_000)
        update_measured_voxels(voxels2, stamps2, np.array([[1.02, 9.02, 0.01]]), stamp_ns=40_000_000_000)
        self.assertEqual(len(voxels2), 2)

    def test_preserves_multifloor_slabs(self):
        voxels, stamps = {}, {}
        update_measured_voxels(voxels, stamps, np.array([[1.02, 9.02, 0.00]]), stamp_ns=10_000_000_000)
        update_measured_voxels(voxels, stamps, np.array([[1.02, 9.02, 2.60]]), stamp_ns=20_000_000_000)
        update_measured_voxels(voxels, stamps, np.array([[1.02, 9.02, 2.93]]), stamp_ns=50_000_000_000)
        self.assertEqual(len(voxels), 3)

    def test_local_window_and_radius_pruning(self):
        voxels, stamps = {}, {}
        update_measured_voxels(
            voxels, stamps,
            np.array([[0.0, 0.0, 0.0], [20.0, 0.0, 0.0]]),
            stamp_ns=10_000_000_000,
        )
        # 50 s later with 30 s window and 10 m radius: old points expire, far new point is pruned
        update_measured_voxels(
            voxels, stamps,
            np.array([[1.0, 0.0, -0.42], [15.0, 0.0, -0.42]]),
            stamp_ns=60_000_000_000,
            local_window_s=30.0,
            local_radius_m=10.0,
            center_xy=(1.0, 0.0),
        )
        self.assertEqual(len(voxels), 1)
        kept = next(iter(voxels.values()))
        self.assertAlmostEqual(float(kept[0]), 1.0, places=6)
        self.assertAlmostEqual(float(kept[2]), -0.42, places=6)

    def test_select_recent_voxels_isolates_post_drift_local_floor(self):
        voxels, stamps = {}, {}
        update_measured_voxels(voxels, stamps, np.array([[-5.0, -3.3, -0.33]]), stamp_ns=250_000_000_000)
        update_measured_voxels(voxels, stamps, np.array([[-5.0, -3.3, -0.75]]), stamp_ns=440_000_000_000)
        self.assertEqual(len(voxels), 2)
        recent = select_recent_voxels(voxels, stamps, latest_stamp_ns=445_000_000_000, window_s=30.0)
        self.assertEqual(recent.shape, (1, 3))
        self.assertAlmostEqual(float(recent[0, 2]), -0.75, places=6)

    def test_backend_correction_realigns_cached_voxels(self):
        raw_vio = np.eye(4)
        raw_vio[:3, 3] = [-5.0, -3.3, -0.43]
        backend = np.eye(4)
        backend[:3, 3] = [-5.0, -3.3, 0.05]
        C = compute_backend_correction(backend, raw_vio)
        self.assertTrue(correction_delta_exceeds(C))
        self.assertFalse(correction_delta_exceeds(np.eye(4)))

        voxels, stamps = {}, {}
        update_measured_voxels(voxels, stamps, np.array([[-5.0, -3.3, -0.75]]), stamp_ns=440_000_000_000)
        transform_measured_voxels(voxels, stamps, C)
        self.assertEqual(len(voxels), 1)
        moved = next(iter(voxels.values()))
        self.assertAlmostEqual(float(moved[2]), -0.27, places=6)

    def test_backend_pose_source_allowlist_and_default_topic(self):
        self.assertIn("backend_corrected", ALLOWED_POSE_SOURCES)
        self.assertEqual(
            slam_odometry_topic("backend_corrected"),
            "/dynanav/backend/corrected_odometry",
        )


if __name__ == "__main__":
    unittest.main()
