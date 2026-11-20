"""Unit tests for pct_scan_worker.update_measured_voxels stale Z-drift eviction."""
import unittest
import numpy as np

from embodied_navigation.simulation.dynanav.pct_scan_worker import update_measured_voxels


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


if __name__ == "__main__":
    unittest.main()
