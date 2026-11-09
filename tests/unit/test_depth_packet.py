"""CPU checks for depth/cloud/pixel correspondence without ROS or Isaac."""

import unittest

import numpy as np

from embodied_navigation.perception.depth import depth_cloud_packet, depth_to_cloud


class DepthPacketTests(unittest.TestCase):
    def test_calibrated_points_keep_their_original_pixels(self):
        depth = np.array([[1.0, 2.0, np.nan], [0.1, 20.0, 0.05]])
        rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        camera = np.array([1.0, 0.2, 0.4])
        imu = np.array([0.1, 0.0, 0.1])
        args = (depth, rotation, camera, imu, 2.0, (1.0, 0.5))
        packet = depth_cloud_packet(*args)
        np.testing.assert_array_equal(packet.pixel_flat_indices, [0, 1, 3])
        np.testing.assert_array_equal(
            packet.depth_keep_mask, [[True, True, False], [True, False, False]]
        )
        np.testing.assert_array_equal(packet.self_mask, [False, False, False])
        np.testing.assert_allclose(
            packet.points,
            [[0.4, 1.2, 0.55], [0.9, 2.2, 0.8], [0.85, 0.3, 0.275]],
            rtol=0,
            atol=1e-14,
        )
        for actual, legacy in zip(
            (packet.points, packet.depth_keep_mask, packet.self_mask),
            depth_to_cloud(*args),
        ):
            np.testing.assert_array_equal(actual, legacy)

    def test_invalid_and_self_pixels_are_removed_before_stride(self):
        depth = np.ones((100, 100), dtype=np.float32)
        depth.flat[0] = np.nan
        depth.flat[5] = 0.1  # Valid depth, but inside the body self box.
        depth.flat[6] = 0.05
        depth.flat[8000] = np.inf
        args = (depth, np.eye(3), np.zeros(3), np.zeros(3), 100.0, (50.0, 50.0))
        packet = depth_cloud_packet(*args)
        # 9996 surviving pixels require stride two. These removals deliberately
        # shift its phase; striding the input/valid pixels first would be wrong.
        expected = np.delete(np.arange(10000), [0, 5, 6, 8000])[::2]
        np.testing.assert_array_equal(packet.pixel_flat_indices, expected)
        self.assertEqual(packet.self_mask.sum(), 1)
        self.assertEqual(packet.points.shape, (4998, 3))
        rows, cols = np.divmod(packet.pixel_flat_indices, depth.shape[1])
        np.testing.assert_allclose(
            packet.points[:, 0], depth.ravel()[expected], rtol=0, atol=0
        )
        np.testing.assert_allclose(
            packet.points[:, 1], -(cols - 50.0) / 100.0, rtol=0, atol=0
        )
        np.testing.assert_allclose(
            packet.points[:, 2], -(rows - 50.0) / 100.0, rtol=0, atol=0
        )
        for actual, legacy in zip(
            (packet.points, packet.depth_keep_mask, packet.self_mask),
            depth_to_cloud(*args),
        ):
            np.testing.assert_array_equal(actual, legacy)

    def test_empty_valid_or_all_self_cloud_has_empty_indices(self):
        for depth, valid_count in [(np.zeros((2, 3)), 0), (np.full((2, 3), 0.1), 6)]:
            with self.subTest(valid_count=valid_count):
                packet = depth_cloud_packet(depth, np.eye(3), np.zeros(3), np.zeros(3))
                self.assertEqual(packet.points.shape, (0, 3))
                self.assertEqual(packet.pixel_flat_indices.shape, (0,))
                self.assertTrue(
                    np.issubdtype(packet.pixel_flat_indices.dtype, np.integer)
                )
                self.assertEqual(packet.depth_keep_mask.sum(), valid_count)
                self.assertEqual(len(packet.self_mask), valid_count)

    def test_float32_calibration_preserves_legacy_output_exactly(self):
        depth = np.full((3, 4), 1.25, dtype=np.float32)
        rotation = np.eye(3, dtype=np.float32)
        camera = np.array([0.010001, 0.02, 0.03], dtype=np.float32)
        imu = np.array([0.009999, -0.01, -0.02], dtype=np.float32)
        args = (depth, rotation, camera, imu, np.float32(123.5), np.array([2, 1]))
        packet = depth_cloud_packet(*args)
        legacy = depth_to_cloud(*args)
        self.assertEqual(len(legacy), 3)
        for actual, old in zip(
            (packet.points, packet.depth_keep_mask, packet.self_mask), legacy
        ):
            np.testing.assert_array_equal(actual, old)

    def test_invalid_depth_and_calibration_shapes_have_clear_errors(self):
        defaults = {
            "sensor_yaw": np.eye(3),
            "camera_mount": np.zeros(3),
            "imu_mount": np.zeros(3),
        }
        for depth in [np.zeros(3), np.zeros((2, 3, 1)), np.array(1.0)]:
            with (
                self.subTest(shape=depth.shape),
                self.assertRaisesRegex(ValueError, "depth must be two-dimensional"),
            ):
                depth_cloud_packet(depth, **defaults)
        for name, value in [
            ("sensor_yaw", np.eye(2)),
            ("camera_mount", np.zeros(2)),
            ("imu_mount", np.zeros((1, 3))),
            ("center_px", np.zeros(3)),
            ("focal_px", np.ones(2)),
        ]:
            with self.subTest(field=name):
                options = {**defaults, name: value}
                with self.assertRaisesRegex(ValueError, name + " must have shape"):
                    depth_cloud_packet(np.ones((2, 3)), **options)

    def test_nonpositive_focal_length_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "focal_px must be positive"):
            depth_cloud_packet(
                np.ones((2, 3)), np.eye(3), np.zeros(3), np.zeros(3), focal_px=0
            )


if __name__ == "__main__":
    unittest.main()
