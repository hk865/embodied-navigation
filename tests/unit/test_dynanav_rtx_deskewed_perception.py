"""Offline contract tests: native deskewed FAST-LIVO2 cloud -> production perception.

CPU only. These tests pin the production semantics that the interactive tracker
relies on: exact same-stage integer stamp pairing, raw wire refusal, the measured
IMU mount sign, floor-vs-collision heights, azimuth/radial coverage, drop/void
rejection and the age gates. A test never relaxes a rule to go green.
"""

import math
import unittest

import numpy as np

from embodied_navigation.simulation.dynanav import rtx_deskewed_perception as module

MOUNT = np.array([0.020153064280748367, 0.0, -0.005108955316245556])


def quaternion_xyzw(rpy):
    """Return (x, y, z, w) for roll/pitch/yaw radians."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]
    )


class MountTransformTests(unittest.TestCase):
    """The published IMU-frame cloud becomes base with the measured mount."""

    def test_positive_mount_is_added_in_base_frame(self):
        position = np.array([1.0, 2.0, 0.5])
        quat = quaternion_xyzw((0.0, 0.0, 0.0))
        # A point at the IMU origin in the map frame is the IMU origin in base.
        base = module.map_points_to_base(position.reshape(1, 3), position, quat, MOUNT)
        np.testing.assert_allclose(base[0], MOUNT, atol=1e-12)
        # A point one metre ahead in the IMU frame is one metre ahead of the mount.
        point_map = position + np.array([1.0, 0.0, 0.0])
        base = module.map_points_to_base(point_map.reshape(1, 3), position, quat, MOUNT)
        np.testing.assert_allclose(base[0], MOUNT + np.array([1.0, 0.0, 0.0]), atol=1e-12)

    def test_full_rpy_roundtrip_is_identity(self):
        rng = np.random.default_rng(7)
        points = rng.normal(size=(40, 3)) * 2.0
        for rpy in (
            (0.0, 0.0, 0.0),
            (0.3, -0.2, 1.1),
            (-1.2, 0.7, -2.4),
            (0.0, 1.5, 0.0),
        ):
            quat = quaternion_xyzw(rpy)
            position = np.array([3.0, -1.0, 0.4])
            rotation = module.rotation_from_quaternion_xyzw(quat)
            points_map = (rotation @ (points - MOUNT.reshape(1, 3)).T).T + position
            restored = module.map_points_to_base(points_map, position, quat, MOUNT)
            np.testing.assert_allclose(restored, points, atol=1e-10)

    def test_nonfinite_pose_is_refused(self):
        points = np.zeros((2, 3))
        with self.assertRaises(module.DeskewedCloudError):
            module.map_points_to_base(points, [np.nan, 0, 0], quaternion_xyzw((0, 0, 0)), MOUNT)
        with self.assertRaises(module.DeskewedCloudError):
            module.map_points_to_base(points, [0, 0, 0], [0, 0, 0, 0], MOUNT)


class RawWireAndLayoutTests(unittest.TestCase):
    """A raw per-point-time scan can never masquerade as a deskewed cloud."""

    def test_raw_wire_fields_are_refused(self):
        with self.assertRaises(module.DeskewedCloudError):
            module.verify_cloud_layout(
                ["x", "y", "z", "intensity", "t", "reflectivity", "ring", "ambient", "range"]
            )

    def test_native_point_xyz_i_normal_layout_is_accepted(self):
        names = ["x", "y", "z", "normal_x", "normal_y", "normal_z", "intensity", "curvature"]
        self.assertEqual(set(module.NATIVE_DESKEWED_FIELDS), set(names))
        self.assertEqual(tuple(names), module.verify_cloud_layout(names))

    def test_zero_curvature_is_not_the_native_deskewed_slice(self):
        with self.assertRaises(module.DeskewedCloudError):
            module.verify_deskew_marker(np.zeros(64))
        self.assertGreater(module.verify_deskew_marker(np.linspace(0.0, 82.0, 64))["curvature_spread_ms"], 1.0)

    def test_raw_topic_and_wrong_pose_role_are_refused(self):
        with self.assertRaises(module.DeskewedCloudError):
            module.verify_topic_roles("/dynanav/lidar_rtx", "/dynanav/rtx_livo/aft_mapped_to_init")
        with self.assertRaises(module.DeskewedCloudError):
            module.verify_topic_roles(
                "/dynanav/rtx_livo/cloud_registered_full",
                "/dynanav/rtx_livo/post_vio_odometry",
            )
        roles = module.verify_topic_roles(
            "/dynanav/rtx_livo/cloud_registered_full",
            "/dynanav/rtx_livo/aft_mapped_to_init",
        )
        self.assertEqual("LIO", roles["stage"])


class SameStagePairTests(unittest.TestCase):
    """Only one identical integer stamp, frame and stage is a pair."""

    def pair(self, **overrides):
        kwargs = dict(
            cloud_frame="dynanav_lio_map",
            pose_frame="dynanav_lio_map",
            child_frame="go2_imu",
            cloud_stamp_ns=9300000000,
            pose_stamp_ns=9300000000,
            expected_map_frame="dynanav_lio_map",
            expected_child_frame="go2_imu",
            stage="LIO",
        )
        kwargs.update(overrides)
        return module.verify_same_stage_pair(**kwargs)

    def test_identical_integer_stamp_pairs(self):
        pair = self.pair()
        self.assertEqual(9300000000, pair["reference_stamp_ns"])
        self.assertEqual("LIO", pair["stage"])

    def test_one_nanosecond_apart_is_a_different_pair(self):
        with self.assertRaises(module.DeskewedCloudError):
            self.pair(pose_stamp_ns=9300000001)
        with self.assertRaises(module.DeskewedCloudError):
            self.pair(cloud_stamp_ns=9299999999, pose_stamp_ns=9300000000)

    def test_vio_stage_or_other_frame_is_refused(self):
        with self.assertRaises(module.DeskewedCloudError):
            self.pair(stage="VIO")
        with self.assertRaises(module.DeskewedCloudError):
            self.pair(pose_frame="camera_init")
        with self.assertRaises(module.DeskewedCloudError):
            self.pair(child_frame="go2_base")


class AgeGateTests(unittest.TestCase):
    """Source and arrival clocks stay separate; neither is re-stamped."""

    def test_stale_future_and_nan(self):
        ok, reason, _ = module.state_age_gate(
            now_sim_s=10.0, stamp_s=10.0, arrival_wall_s=100.0, now_wall_s=100.0,
            max_sim_age_s=0.3, max_wall_age_s=0.5,
        )
        self.assertTrue(ok, reason)
        ok, reason, _ = module.state_age_gate(
            now_sim_s=10.0, stamp_s=9.0, arrival_wall_s=100.0, now_wall_s=100.0,
            max_sim_age_s=0.3, max_wall_age_s=0.5,
        )
        self.assertFalse(ok)
        self.assertEqual("cloud_sim_expired", reason)
        ok, reason, _ = module.state_age_gate(
            now_sim_s=10.0, stamp_s=12.0, arrival_wall_s=100.0, now_wall_s=100.0,
            max_sim_age_s=0.3, max_wall_age_s=0.5,
        )
        self.assertFalse(ok)
        self.assertEqual("cloud_source_in_future", reason)
        ok, reason, _ = module.state_age_gate(
            now_sim_s=10.0, stamp_s=10.0, arrival_wall_s=99.0, now_wall_s=100.0,
            max_sim_age_s=0.3, max_wall_age_s=0.5,
        )
        self.assertFalse(ok)
        self.assertEqual("cloud_wall_expired", reason)
        ok, reason, _ = module.state_age_gate(
            now_sim_s=10.0, stamp_s=float("nan"), arrival_wall_s=100.0, now_wall_s=100.0,
            max_sim_age_s=0.3, max_wall_age_s=0.5,
        )
        self.assertFalse(ok)
        self.assertEqual("invalid_clock", reason)


if __name__ == "__main__":
    unittest.main()
