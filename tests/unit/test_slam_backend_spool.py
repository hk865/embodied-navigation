"""SLAM-BE spool reading (R-SPOOL-*) against the frozen keyframes.py interface."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

import slam_backend_fixtures as F
from embodied_navigation.localization_mapping.backend.keyframes import (
    Keyframe, read_keyframe, read_spool, voxel_downsample)

SEC = 1_000_000_000


def _cloud(seed, n=120):
    return np.random.default_rng(seed).uniform(-5.0, 5.0, (n, 3))


class SpoolTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.spool = Path(self._tmp.name) / "native_keyframes"
        self.spool.mkdir()

    def tearDown(self):
        self._tmp.cleanup()

    def _write(self, seq, *, epoch=0, ready=True, write_metadata=True):
        return F.write_spool_keyframe(self.spool, epoch, seq, seq * SEC, F.pose(seq, 0.5 * seq, 0.1 * seq),
                                      F.T_BODY_LIDAR, _cloud(seq), ready=ready, write_metadata=write_metadata)

    def test_r_spool_1_numeric_sequence_order(self):
        # Catches: sorting kf_* directory names lexicographically (kf_0_10 before kf_0_2), i.e. B-1.
        for seq in [12, 3, 10, 1, 7, 11, 2, 9, 5, 4, 8, 6]:
            self._write(seq)
        F.write_bridge_status(self.spool)  # bridge also leaves its status files in the spool root
        frames = read_spool(self.spool)
        self.assertEqual([kf.key for kf in frames], [(0, s) for s in range(1, 13)])
        for kf in frames:
            self.assertIsInstance(kf, Keyframe)
            self.assertIsInstance(kf.key[0], int)
            self.assertIsInstance(kf.key[1], int)
            self.assertEqual(kf.stamp_ns, kf.key[1] * SEC)

    def test_r_spool_2_temporary_directory_not_read(self):
        # Catches: reading directories that the bridge has not renamed to kf_* yet (no readiness check).
        for seq in (1, 2, 3):
            self._write(seq)
        self._write(4, ready=False)  # complete content, still under tmp_0_4
        self.assertEqual([kf.key for kf in read_spool(self.spool)], [(0, 1), (0, 2), (0, 3)])

    def test_r_spool_2_directory_without_metadata_not_read(self):
        # Catches: treating a half-written keyframe (cloud present, metadata.json missing) as ready.
        for seq in (1, 2, 3):
            self._write(seq)
        self._write(4, write_metadata=False)
        self.assertEqual([kf.key for kf in read_spool(self.spool)], [(0, 1), (0, 2), (0, 3)])

    def test_r_spool_3_cpp_format_row_major_matrix(self):
        # Catches: transposing the row-major nested JSON rows written by snapshot_bridge.cpp matrix().
        # Hand-written exactly as SnapshotBridge::write_snapshot emits it (setprecision 17 / 9).
        directory = self.spool / "kf_7_3"
        directory.mkdir()
        (directory / "points_lidar.pcd").write_text(
            "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 3\n"
            "HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 3\nDATA ascii\n"
            "1.5 -2.25 0.125\n10 0 -0.5\n-3.75 4.5 2\n", newline="\n")
        (directory / "metadata.json").write_text(
            '{"schema":"livo_keyframe/v1","epoch":7,"sequence":3,"stamp_ns":1731650000123456789,'
            '"T_odom_body":[[0,-1,0,1.5],[1,0,0,-2.25],[0,0,1,0.125],[0,0,0,1]],'
            '"T_body_lidar":[[1,0,0,0.10000000000000001],[0,1,0,0],[0,0,1,0.40000000000000002],[0,0,0,1]],'
            '"final_stage":"livo","point_count":3,"source_point_count":5210,"copy_ns":48211,'
            '"cloud_frame":"lidar"}\n', newline="\n")
        expected_T = np.array([[0.0, -1.0, 0.0, 1.5], [1.0, 0.0, 0.0, -2.25],
                               [0.0, 0.0, 1.0, 0.125], [0.0, 0.0, 0.0, 1.0]])
        expected_lidar = np.array([[1.0, 0.0, 0.0, 0.1], [0.0, 1.0, 0.0, 0.0],
                                   [0.0, 0.0, 1.0, 0.4], [0.0, 0.0, 0.0, 1.0]])
        expected_points = np.array([[1.5, -2.25, 0.125], [10.0, 0.0, -0.5], [-3.75, 4.5, 2.0]])
        for kf in (read_keyframe(directory), read_spool(self.spool)[0]):
            self.assertEqual(kf.key, (7, 3))
            self.assertEqual(kf.stamp_ns, 1731650000123456789)
            self.assertEqual(kf.T_odom_body.shape, (4, 4))
            np.testing.assert_array_equal(kf.T_odom_body, expected_T)
            np.testing.assert_array_equal(kf.T_body_lidar, expected_lidar)
            self.assertEqual(kf.points_lidar.dtype, np.float64)
            np.testing.assert_array_equal(kf.points_lidar, expected_points)

    def test_r_spool_3_fixture_writer_roundtrip(self):
        # Catches: losing precision / transposing when parsing the fixture's cpp-faithful writer output.
        T = F.pose(3.25, -1.5, 0.7, z=0.2, roll=0.05, pitch=-0.03)
        pts = F.make_world().scan(F.loop_pose_at(14.0) @ F.T_BODY_LIDAR, 3)
        path = F.write_spool_keyframe(self.spool, 2, 9, 9 * SEC, T, F.T_BODY_LIDAR, pts)
        kf = read_keyframe(path)
        np.testing.assert_array_equal(kf.T_odom_body, T)  # %.17g round-trips float64 exactly
        np.testing.assert_array_equal(kf.T_body_lidar, F.T_BODY_LIDAR)
        np.testing.assert_allclose(kf.points_lidar, pts.astype(np.float32), rtol=0, atol=1e-6)

    # -- frozen contract: malformed entries raise ValueError naming the path ----------
    def _assert_value_error_with_path(self, directory):
        with self.assertRaises(ValueError) as caught:
            read_keyframe(directory)
        self.assertIn(directory.name, str(caught.exception))

    def test_contract_bad_schema_raises_value_error_with_path(self):
        # Catches: silently accepting an unknown metadata schema.
        path = self._write(1)
        meta = (path / "metadata.json").read_text().replace("livo_keyframe/v1", "livo_keyframe/v0")
        (path / "metadata.json").write_text(meta)
        self._assert_value_error_with_path(path)

    def test_contract_non_4x4_matrix_raises_value_error_with_path(self):
        # Catches: accepting a 3x4 pose matrix.
        path = self._write(1)
        meta = (path / "metadata.json").read_text().replace(",[0,0,0,1]]", "]", 1)
        (path / "metadata.json").write_text(meta)
        self._assert_value_error_with_path(path)

    def test_contract_non_finite_point_raises_value_error_with_path(self):
        # Catches: letting NaN coordinates into the map.
        path = self._write(1)
        lines = (path / "points_lidar.pcd").read_text().splitlines()
        lines[-1] = "nan 0 0"
        (path / "points_lidar.pcd").write_text("\n".join(lines) + "\n")
        self._assert_value_error_with_path(path)

    def test_contract_voxel_downsample_reduces_points(self):
        # Catches: voxel_downsample returning its input unchanged or points outside the input hull.
        pts = np.random.default_rng(1).uniform(0.01, 0.09, (500, 3))
        out = voxel_downsample(pts, 0.1)
        self.assertEqual(out.shape, (1, 3))
        self.assertTrue(np.all((out >= 0.01) & (out <= 0.09)))


if __name__ == "__main__":
    unittest.main()
