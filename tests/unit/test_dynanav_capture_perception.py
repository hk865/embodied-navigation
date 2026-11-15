"""Producer-side tests for the interactive perception sample.

Two layers are covered: the direction labelling itself (a forward camera cannot
observe the rear or the sides) and the production path that feeds it, i.e. the
depth_cloud_packet IMU cloud plus its original-pixel correspondence translated
into the base frame. The second layer is where a prefix-sliced mask pairing used
to silently drop or keep unrelated points.
"""

import unittest
from dataclasses import replace

import numpy as np

from embodied_navigation.perception.depth import depth_cloud_packet
from embodied_navigation.simulation.dynanav.capture.perception import (
    base_frame_cloud,
    direction_coverage,
    interactive_perception_payload,
)


def forward_cloud(count=400, distance=2.0, height=0.0, spread=0.3, near=1.0):
    """Rays that a forward-looking depth camera can plausibly return."""
    x = np.linspace(near, distance, count)
    y = np.linspace(-spread, spread, count)
    z = np.full(count, height)
    return np.stack([x, y, z], axis=1)


class ForwardOnlyCameraTests(unittest.TestCase):
    def test_rear_and_sides_are_unknown_for_a_forward_camera(self):
        points = forward_cloud()
        directions = direction_coverage(points)
        self.assertEqual("measured", directions["front"]["observation_state"])
        for name in ("rear", "left", "right"):
            entry = directions[name]
            self.assertEqual("insufficient_observation", entry["observation_state"])
            self.assertIsNone(entry["clearance_m"])
            self.assertEqual(0, entry["points"])
            self.assertEqual(0, entry["rays"])
            self.assertEqual("no_ray_coverage", entry["reason"])

    def test_a_healthy_frame_does_not_make_the_rear_free(self):
        # Dense, perfectly valid forward rays: this is exactly the case that used
        # to publish clear_free for every direction.
        points = forward_cloud(count=2000, distance=4.0, spread=1.0)
        directions = direction_coverage(points)
        self.assertGreater(directions["front"]["rays"], 0)
        self.assertEqual(
            "insufficient_observation", directions["rear"]["observation_state"]
        )
        self.assertIsNone(directions["rear"]["clearance_m"])

    def test_front_obstacle_sets_the_clearance(self):
        points = forward_cloud(count=400, distance=3.0, near=2.0)
        obstacle = np.array([[1.2, 0.0, 0.2]] * 10)
        directions = direction_coverage(np.vstack([points, obstacle]))
        self.assertEqual("measured", directions["front"]["observation_state"])
        self.assertAlmostEqual(1.2, directions["front"]["clearance_m"], places=6)

    def test_front_rays_without_corridor_returns_are_clear_free(self):
        points = forward_cloud(count=400, distance=6.0, near=4.5)  # beyond the corridor
        directions = direction_coverage(points)
        self.assertEqual("clear_free", directions["front"]["observation_state"])
        self.assertAlmostEqual(4.0, directions["front"]["clearance_m"], places=6)
        self.assertEqual("observed_rays_without_returns", directions["front"]["reason"])

    def test_floor_returns_count_as_rays_but_never_as_obstacles(self):
        floor = np.stack(
            [
                np.linspace(0.5, 3.0, 300),
                np.linspace(-0.2, 0.2, 300),
                np.full(300, -0.30),
            ],
            axis=1,
        )
        directions = direction_coverage(floor)
        self.assertEqual(300, directions["front"]["rays"])
        self.assertEqual("clear_free", directions["front"]["observation_state"])
        self.assertAlmostEqual(4.0, directions["front"]["clearance_m"], places=6)

    def test_sector_below_the_ray_threshold_is_unknown(self):
        few = np.array([[1.0, 0.0, 0.1]] * 10)
        directions = direction_coverage(few, min_sector_points=50)
        self.assertEqual("insufficient_observation", directions["front"]["observation_state"])
        self.assertIsNone(directions["front"]["clearance_m"])

    def test_side_coverage_is_reported_when_rays_exist(self):
        left = np.stack(
            [
                np.full(200, 0.1),
                np.linspace(0.5, 3.0, 200),
                np.full(200, 0.0),
            ],
            axis=1,
        )
        front = forward_cloud(count=200)
        directions = direction_coverage(np.vstack([front, left]))
        self.assertEqual("measured", directions["left"]["observation_state"])
        self.assertEqual("insufficient_observation", directions["right"]["observation_state"])

    def test_self_echo_points_are_not_rays(self):
        points = forward_cloud(count=100)
        valid = np.ones(100, dtype=bool)
        own = np.zeros(100, dtype=bool)
        own[:95] = True
        directions = direction_coverage(points, valid, own)
        self.assertEqual("insufficient_observation", directions["front"]["observation_state"])

    def test_invalid_depth_pixels_are_not_rays(self):
        points = forward_cloud(count=100)
        valid = np.zeros(100, dtype=bool)
        valid[:10] = True
        directions = direction_coverage(points, valid)
        self.assertEqual("insufficient_observation", directions["front"]["observation_state"])




class NearFieldBoundaryTests(unittest.TestCase):
    """What the clearance gate can and cannot see near the body.

    The corridor starts at 0.30 m along each direction and the producer drops the
    body self-echo box (|x| < 0.5 m, |y| < 0.3 m, z > -0.45 m) and everything below
    base z - 0.05 m. Three boundaries follow, and each one is pinned here so that
    a later reader cannot mistake "clear_free 4.0 m" for a proof that the path is
    free: a surviving near return sets the clearance, an obstacle inside the body
    box is indistinguishable from self-echo, and a low surface under the height
    band is not obstacle evidence at all.
    """

    @staticmethod
    def rays(near=2.0):
        return np.stack(
            [
                np.linspace(near, 4.0, 200),
                np.linspace(-0.1, 0.1, 200),
                np.zeros(200),
            ],
            axis=1,
        )

    def test_a_surviving_near_return_sets_the_clearance(self):
        cloud = np.vstack([self.rays(), np.array([[0.28, 0.30, 0.10]] * 100)])
        directions = direction_coverage(cloud)
        entry = directions["left"]
        self.assertEqual("measured", entry["observation_state"])
        self.assertEqual("near_field_return", entry["reason"])
        self.assertAlmostEqual(0.30, entry["clearance_m"], places=6)

    def test_a_near_return_inside_the_half_width_is_not_dropped(self):
        # Exactly at the corridor start the return is still inside the near band.
        cloud = np.vstack([self.rays(), np.array([[0.10, 0.30, 0.10]] * 100)])
        entry = direction_coverage(cloud)["left"]
        self.assertEqual("near_field_return", entry["reason"])
        self.assertAlmostEqual(0.30, entry["clearance_m"], places=6)

    def test_a_return_just_past_the_corridor_start_is_measured_normally(self):
        cloud = np.vstack([self.rays(), np.array([[0.10, 0.44, 0.10]] * 100)])
        entry = direction_coverage(cloud)["left"]
        self.assertEqual("measured", entry["observation_state"])
        self.assertIsNone(entry["reason"])
        self.assertAlmostEqual(0.44, entry["clearance_m"], places=6)

    def test_an_obstacle_inside_the_body_box_is_indistinguishable_from_self_echo(
        self,
    ):
        cloud = np.vstack([self.rays(), np.array([[0.30, 0.10, 0.10]] * 100)])
        directions = direction_coverage(cloud, self_mask=np.arange(len(cloud)) >= 200)
        entry = directions["front"]
        self.assertEqual("measured", entry["observation_state"])
        # Only the far rays are visible: the 0.30 m obstacle is inside the body box.
        self.assertAlmostEqual(2.0, entry["clearance_m"], places=6)

    def test_a_surface_below_the_height_band_is_not_obstacle_evidence(self):
        cloud = np.vstack([self.rays(), np.array([[1.00, 0.0, -0.15]] * 100)])
        entry = direction_coverage(cloud)["front"]
        self.assertAlmostEqual(2.0, entry["clearance_m"], places=6)
        self.assertEqual("measured", entry["observation_state"])

if __name__ == "__main__":
    unittest.main()


FOCAL_PX = 512 * 24 / 42
CENTER_PX = (256, 256)
CAMERA_MOUNT_BASE_M = np.array([0.3, 0.0, 0.13])
# The measured Go2 base centre of mass in capture6 result.json.
MEASURED_IMU_MOUNT_BASE_M = np.array([0.020153064280748367, 0.0, -0.005108955316245556])


def yaw_rotation(radians):
    cos, sin = np.cos(radians), np.sin(radians)
    return np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])


def expected_base_point(depth_m, row, col, yaw_rad):
    """Independent sensor->base reconstruction: rotation, then camera mount."""
    focal = FOCAL_PX
    p_cam = np.array(
        [
            depth_m,
            -(col - CENTER_PX[0]) / focal * depth_m,
            -(row - CENTER_PX[1]) / focal * depth_m,
        ]
    )
    return yaw_rotation(yaw_rad) @ p_cam + CAMERA_MOUNT_BASE_M


def empty_depth(fill=np.nan):
    return np.full((512, 512), fill, dtype=float)


def packet_for(depth, yaw_rad=0.0, imu_mount=MEASURED_IMU_MOUNT_BASE_M):
    return depth_cloud_packet(
        depth, yaw_rotation(yaw_rad), CAMERA_MOUNT_BASE_M, np.asarray(imu_mount, float)
    )


class PacketFrameAndPixelPairingTests(unittest.TestCase):
    """The published sample must be the real base cloud, paired by pixel index."""

    def test_sensor_to_base_chain_is_a_real_transform(self):
        depth = empty_depth()
        pixels = [(200, 300, 1.5), (100, 256, 2.0), (300, 100, 0.9)]
        for row, col, value in pixels:
            depth[row, col] = value
        # Surviving points follow the C-order flat pixel index, not insertion order.
        ordered = sorted(pixels, key=lambda item: item[0] * 512 + item[1])
        for yaw in (0.0, np.pi / 2):
            packet = packet_for(depth, yaw)
            base, valid, own = base_frame_cloud(packet, MEASURED_IMU_MOUNT_BASE_M)
            self.assertEqual((3, 3), base.shape)
            self.assertTrue(bool(valid.all()))
            self.assertFalse(bool(own.any()))
            np.testing.assert_array_equal(
                np.asarray(packet.pixel_flat_indices),
                np.array([row * 512 + col for row, col, _ in ordered]),
            )
            for (row, col, value), point in zip(ordered, base):
                expected = expected_base_point(value, row, col, yaw)
                np.testing.assert_allclose(point, expected, atol=1e-9)
            # The producer cloud is the IMU frame: the base cloud differs from it
            # by exactly the measured IMU origin, not by a relabelled string.
            np.testing.assert_allclose(
                base - packet.points, np.tile(MEASURED_IMU_MOUNT_BASE_M, (3, 1)), atol=1e-12
            )

    def test_base_cloud_does_not_depend_on_the_imu_offset(self):
        depth = empty_depth()
        depth[256, 256] = 2.0
        # The producer subtracts the IMU origin it is given, the publisher adds it
        # back, so a consistent pair must publish the camera-mount-only geometry.
        for imu in (np.zeros(3), MEASURED_IMU_MOUNT_BASE_M):
            packet = packet_for(depth, 0.0, imu)
            base, _, _ = base_frame_cloud(packet, imu)
            np.testing.assert_allclose(
                base[0], expected_base_point(2.0, 256, 256, 0.0), atol=1e-12
            )

    def test_a_rotated_mount_puts_the_point_on_the_expected_base_axis(self):
        depth = empty_depth()
        depth[256, 256] = 2.0
        packet = packet_for(depth, np.pi / 2)
        base, _, _ = base_frame_cloud(packet, MEASURED_IMU_MOUNT_BASE_M)
        # Camera forward is base +y for this mount; x only carries the mount.
        np.testing.assert_allclose(base[0][0], CAMERA_MOUNT_BASE_M[0], atol=1e-9)
        np.testing.assert_allclose(base[0][1], 2.0, atol=1e-9)

    def test_stride_keeps_the_pixel_index_of_every_surviving_point(self):
        depth = empty_depth(1.5)
        packet = packet_for(depth, 0.0)
        self.assertGreater(len(packet.points), 7000)
        self.assertEqual(len(packet.points), len(packet.pixel_flat_indices))
        indices = np.asarray(packet.pixel_flat_indices)
        self.assertTrue(np.all(np.diff(indices) > 1), "stride must skip pixels")
        base, _, _ = base_frame_cloud(packet, MEASURED_IMU_MOUNT_BASE_M)
        for row_index in (0, len(base) // 2, len(base) - 1):
            flat = int(indices[row_index])
            row, col = divmod(flat, 512)
            np.testing.assert_allclose(
                base[row_index],
                expected_base_point(1.5, row, col, 0.0),
                atol=1e-9,
            )

    def test_invalid_depth_and_self_echo_are_dropped_at_scattered_pixels(self):
        depth = empty_depth(2.0)
        invalid = [0, 777, 5000, 100000, 262143]
        for flat in invalid:
            depth[np.unravel_index(flat, depth.shape)] = np.nan
        # 0.15 m unprojects inside the body self-echo box, so these are self
        # returns; 0.20 m lands exactly on the box face and would not be dropped.
        self_echo = [1000, 30000, 200000]
        for flat in self_echo:
            depth[np.unravel_index(flat, depth.shape)] = 0.15
        packet = packet_for(depth, 0.0)
        indices = set(int(value) for value in packet.pixel_flat_indices)
        for flat in invalid + self_echo:
            self.assertNotIn(flat, indices)
        self.assertEqual(int(np.asarray(packet.self_mask).sum()), len(self_echo))
        base, valid, own = base_frame_cloud(packet, MEASURED_IMU_MOUNT_BASE_M)
        self.assertTrue(bool(valid.all()))
        self.assertFalse(bool(own.any()))
        valid_count = int(np.asarray(packet.depth_keep_mask).sum())
        surviving = valid_count - len(self_echo)
        stride = max(1, (surviving + 7999) // 8000)
        self.assertEqual(int(np.ceil(surviving / stride)), len(base))
        # No surviving point may sit inside the body self-echo box in base axes.
        self.assertFalse(
            bool(
                (
                    (np.abs(base[:, 0]) < 0.5)
                    & (np.abs(base[:, 1]) < 0.3)
                    & (base[:, 2] > -0.45)
                    & (base[:, 2] < 0.3)
                ).any()
            )
        )

    def test_payload_reports_the_base_frame_clearance_of_a_front_obstacle(self):
        depth = empty_depth()
        depth[236:276, 200:320] = 1.2
        packet = packet_for(depth, 0.0)
        payload = interactive_perception_payload(
            packet,
            imu_mount=MEASURED_IMU_MOUNT_BASE_M,
            stamp_s=12.5,
            arrival_wall_s=99.0,
            depth_valid_ratio=0.01,
        )
        self.assertFalse(payload["fail_closed"])
        self.assertEqual("go2_base", payload["frame_id"])
        self.assertEqual("measured", payload["observation_state"])
        expected = float(expected_base_point(1.2, 236, 200, 0.0)[0])
        self.assertAlmostEqual(expected, payload["clearance_m"], places=9)
        # The obstacle is 1.2 m in front of the camera, which sits 0.3 m ahead of
        # the base origin, so the base-frame clearance is 1.5 m. The IMU cloud
        # alone would have reported 1.4798 m, short by the measured IMU offset.
        self.assertAlmostEqual(1.5, payload["clearance_m"], places=9)
        self.assertAlmostEqual(
            payload["clearance_m"] - 1.4798,
            float(MEASURED_IMU_MOUNT_BASE_M[0]),
            places=4,
        )
        self.assertEqual("insufficient_observation", payload["directions"]["rear"]["observation_state"])
        self.assertIsNone(payload["directions"]["rear"]["clearance_m"])
        self.assertEqual(int(len(packet.points)), payload["cloud_points"])
        self.assertEqual(int(len(packet.points)), payload["directions"]["front"]["rays"])
        self.assertTrue(payload["pixel_correspondence"]["verified"])
        np.testing.assert_allclose(
            payload["frame_chain"]["imu_origin_in_base_m"],
            MEASURED_IMU_MOUNT_BASE_M.tolist(),
            atol=1e-12,
        )

    def test_a_side_mounted_forward_camera_labels_a_left_obstacle_in_base(self):
        depth = empty_depth()
        depth[236:276, 200:320] = 1.2
        packet = packet_for(depth, np.pi / 2)
        payload = interactive_perception_payload(
            packet,
            imu_mount=MEASURED_IMU_MOUNT_BASE_M,
            stamp_s=1.0,
            arrival_wall_s=2.0,
            depth_valid_ratio=0.01,
        )
        # The same obstacle now sits on base +y: the left sector owns it.
        self.assertEqual("measured", payload["directions"]["left"]["observation_state"])
        self.assertEqual("insufficient_observation", payload["directions"]["front"]["observation_state"])
        self.assertGreater(payload["directions"]["left"]["clearance_m"], 1.0)


class FailClosedPairingTests(unittest.TestCase):
    """Unpair-able or uncalibrated arrays must publish unknown, not free space."""

    @staticmethod
    def good_packet():
        depth = empty_depth()
        depth[236:276, 200:320] = 1.2
        return packet_for(depth, 0.0)

    def assert_fail_closed(self, payload):
        self.assertTrue(payload["fail_closed"])
        self.assertIsNone(payload["clearance_m"])
        self.assertNotIn(payload["observation_state"], ("measured", "clear_free"))
        self.assertEqual(0, payload["cloud_points"])
        self.assertTrue(payload["producer_error"])
        for name, entry in payload["directions"].items():
            self.assertEqual("insufficient_observation", entry["observation_state"])
            self.assertIsNone(entry["clearance_m"])

    def test_missing_or_malformed_imu_mount_fails_closed(self):
        packet = self.good_packet()
        for imu in (None, np.array([0.02, 0.0]), np.array([np.nan, 0.0, 0.0])):
            payload = interactive_perception_payload(
                packet, imu_mount=imu, stamp_s=1.0, arrival_wall_s=2.0
            )
            self.assert_fail_closed(payload)

    def test_out_of_range_pixel_indices_fail_closed(self):
        packet = self.good_packet()
        broken = replace(packet, pixel_flat_indices=np.full(len(packet.points), 512 * 512))
        payload = interactive_perception_payload(
            broken, imu_mount=MEASURED_IMU_MOUNT_BASE_M, stamp_s=1.0, arrival_wall_s=2.0
        )
        self.assert_fail_closed(payload)

    def test_index_length_mismatch_fails_closed(self):
        packet = self.good_packet()
        broken = replace(packet, pixel_flat_indices=packet.pixel_flat_indices[:-1])
        payload = interactive_perception_payload(
            broken, imu_mount=MEASURED_IMU_MOUNT_BASE_M, stamp_s=1.0, arrival_wall_s=2.0
        )
        self.assert_fail_closed(payload)

    def test_non_integer_pixel_indices_fail_closed(self):
        packet = self.good_packet()
        broken = replace(
            packet, pixel_flat_indices=packet.pixel_flat_indices.astype(float)
        )
        payload = interactive_perception_payload(
            broken, imu_mount=MEASURED_IMU_MOUNT_BASE_M, stamp_s=1.0, arrival_wall_s=2.0
        )
        self.assert_fail_closed(payload)

    def test_image_mask_shape_mismatch_fails_closed(self):
        packet = self.good_packet()
        broken = replace(packet, depth_keep_mask=packet.depth_keep_mask.reshape(-1))
        payload = interactive_perception_payload(
            broken, imu_mount=MEASURED_IMU_MOUNT_BASE_M, stamp_s=1.0, arrival_wall_s=2.0
        )
        self.assert_fail_closed(payload)

    def test_self_mask_length_mismatch_fails_closed(self):
        packet = self.good_packet()
        broken = replace(packet, self_mask=np.zeros(512 * 512, dtype=bool))
        payload = interactive_perception_payload(
            broken, imu_mount=MEASURED_IMU_MOUNT_BASE_M, stamp_s=1.0, arrival_wall_s=2.0
        )
        self.assert_fail_closed(payload)

    def test_index_pointing_at_an_invalid_pixel_fails_closed(self):
        packet = self.good_packet()
        indices = np.asarray(packet.pixel_flat_indices).copy()
        indices[0] = 0  # an all-NaN image pixel outside the obstacle block
        broken = replace(packet, pixel_flat_indices=indices)
        payload = interactive_perception_payload(
            broken, imu_mount=MEASURED_IMU_MOUNT_BASE_M, stamp_s=1.0, arrival_wall_s=2.0
        )
        self.assert_fail_closed(payload)

    def test_empty_cloud_fails_closed(self):
        packet = packet_for(empty_depth(), 0.0)
        self.assertEqual(0, len(packet.points))
        payload = interactive_perception_payload(
            packet, imu_mount=MEASURED_IMU_MOUNT_BASE_M, stamp_s=1.0, arrival_wall_s=2.0
        )
        self.assert_fail_closed(payload)

    def test_non_finite_points_fail_closed(self):
        packet = self.good_packet()
        points = np.array(packet.points, dtype=float, copy=True)
        points[0, 0] = np.inf
        broken = replace(packet, points=points)
        payload = interactive_perception_payload(
            broken, imu_mount=MEASURED_IMU_MOUNT_BASE_M, stamp_s=1.0, arrival_wall_s=2.0
        )
        self.assert_fail_closed(payload)


if __name__ == "__main__":
    unittest.main()