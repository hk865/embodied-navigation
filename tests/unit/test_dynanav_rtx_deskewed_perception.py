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


def floor_wall_cloud(floor_z=-0.385, wall_range=2.4, hole_deg=None, azimuth=(-180.0, 180.0),
                     floor_min=1.55, floor_max=3.0, step=0.08, wall_height=1.0):
    """Synthetic base-frame cloud: dense floor annulus plus a wall ring."""
    rows = []
    for angle in np.arange(azimuth[0], azimuth[1], 0.5):
        radians = math.radians(angle)
        if hole_deg is not None and hole_deg[0] <= angle <= hole_deg[1]:
            continue
        for radius in np.arange(floor_min, floor_max, step):
            rows.append(
                [radius * math.cos(radians), radius * math.sin(radians), floor_z + 0.004 * math.sin(radius)]
            )
        for height in np.arange(-0.30, wall_height, 0.06):
            rows.append(
                [wall_range * math.cos(radians), wall_range * math.sin(radians), height]
            )
    return np.asarray(rows, dtype=float)


def empty_room_cloud(blind_deg=None):
    """Floor + wall ring at 2.4 m; optional azimuth span with no returns."""
    if blind_deg is None:
        return floor_wall_cloud()
    rows = []
    for angle in np.arange(-180.0, 180.0, 0.5):
        if blind_deg[0] <= angle <= blind_deg[1]:
            continue
        radians = math.radians(angle)
        for radius in np.arange(1.55, 3.0, 0.08):
            rows.append([radius * math.cos(radians), radius * math.sin(radians), -0.385])
        for height in np.arange(-0.30, 1.0, 0.06):
            rows.append([2.4 * math.cos(radians), 2.4 * math.sin(radians), height])
    return np.asarray(rows, dtype=float)


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


class GeometryTests(unittest.TestCase):
    """Floor is not an obstacle and a missing floor/support is never free space."""

    def test_floor_is_not_a_collision_and_support_is_measured(self):
        cloud = floor_wall_cloud(floor_min=0.35)
        floor_z, evidence = module.detect_floor_z(cloud)
        self.assertAlmostEqual(-0.385, floor_z, delta=0.02)
        self.assertIsNone(evidence["reason"])
        floor_min, floor_evidence = module.floor_visible_min_range(cloud, floor_z)
        self.assertGreaterEqual(floor_min, 0.35)
        directions = module.direction_evidence(
            cloud, floor_z_m=floor_z, radial_min_m=floor_min
        )
        for name in ("front", "left", "rear", "right"):
            entry = directions[name]
            self.assertEqual("measured", entry["observation_state"], name)
            self.assertEqual("supported", entry["floor_observation_state"], name)
            self.assertAlmostEqual(2.4, entry["clearance_m"], delta=0.05)

    def test_thin_obstacle_sets_clearance(self):
        cloud = floor_wall_cloud(floor_min=0.35)
        floor_z, _ = module.detect_floor_z(cloud)
        floor_min, _ = module.floor_visible_min_range(cloud, floor_z)
        thin = np.array([[1.05, 0.0, -0.10], [1.06, 0.01, 0.10], [1.04, -0.01, 0.30]])
        directions = module.direction_evidence(
            np.vstack([cloud, thin]), floor_z_m=floor_z, radial_min_m=floor_min
        )
        self.assertAlmostEqual(1.04, directions["front"]["clearance_m"], delta=0.02)

    def test_narrow_dense_beam_does_not_cover_the_corridor(self):
        cloud = floor_wall_cloud()
        floor_z, _ = module.detect_floor_z(cloud)
        floor_min, _ = module.floor_visible_min_range(cloud, floor_z)
        # Remove every return outside a narrow front beam: dense there, unknown elsewhere.
        azimuth = np.degrees(np.arctan2(cloud[:, 1], cloud[:, 0]))
        narrow = cloud[np.abs(azimuth) <= 2.0]
        directions = module.direction_evidence(
            np.vstack([narrow, narrow]), floor_z_m=floor_z, radial_min_m=floor_min
        )
        self.assertEqual("insufficient_observation", directions["front"]["observation_state"])
        self.assertIn("azimuth_uncovered", directions["front"]["reason"])

    def test_drop_or_void_bin_is_unknown_not_free(self):
        # A narrow front wedge only: a dense floor annulus plus a wall, with one
        # radial hole whose only returns are far below the measured floor (stairs
        # going down). A hole must never be reported as free space.
        floor_z = -0.385
        rows = []
        for angle in np.arange(-6.0, 6.0, 0.25):
            radians = math.radians(angle)
            for radius in np.arange(1.6, 3.0, 0.05):
                if 1.95 <= radius < 2.25:
                    rows.append([radius * math.cos(radians), radius * math.sin(radians), floor_z - 0.45])
                else:
                    rows.append([radius * math.cos(radians), radius * math.sin(radians), floor_z])
            for height in np.arange(-0.30, 1.0, 0.06):
                rows.append([2.5 * math.cos(radians), 2.5 * math.sin(radians), height])
        cloud = np.asarray(rows, dtype=float)
        floor, _ = module.detect_floor_z(cloud)
        floor_min, _ = module.floor_visible_min_range(cloud, floor)
        directions = module.direction_evidence(
            cloud, floor_z_m=floor, radial_min_m=floor_min
        )
        front = directions["front"]
        self.assertEqual("insufficient_observation", front["observation_state"])
        self.assertIsNone(front["clearance_m"])

    def test_no_floor_means_no_clearance(self):
        wall_only = np.array(
            [
                [2.0 * math.cos(math.radians(a)), 2.0 * math.sin(math.radians(a)), h]
                for a in np.arange(-180.0, 180.0, 1.0)
                for h in np.arange(-0.2, 1.0, 0.1)
            ]
        )
        payload = module.build_perception_payload(
            wall_only,
            stamp_s=1.0,
            arrival_wall_s=2.0,
            producer_metadata={
                "stage": "LIO",
                "reference_stamp_ns": 1,
                "map_frame": "dynanav_lio_map",
                "child_frame": "go2_imu",
            },
        )
        self.assertTrue(payload["fail_closed"])
        self.assertIsNone(payload["clearance_m"])
        self.assertIsNone(payload["depth_valid_ratio"])
        for entry in payload["directions"].values():
            self.assertEqual("unsupported", entry["floor_observation_state"])

    def test_blind_sector_is_reported_not_covered(self):
        cloud = empty_room_cloud(blind_deg=(150.0, 210.0))
        payload = module.build_perception_payload(
            cloud,
            stamp_s=1.0,
            arrival_wall_s=2.0,
            producer_metadata={
                "stage": "LIO",
                "reference_stamp_ns": 1,
                "map_frame": "dynanav_lio_map",
                "child_frame": "go2_imu",
            },
        )
        gaps = payload["lidar_geometry"]["blind_azimuth_sectors_deg"]
        self.assertTrue(any(gap["start_deg"] <= 170.0 <= gap["end_deg"] for gap in gaps) or gaps)
        self.assertEqual("insufficient_observation", payload["directions"]["rear"]["observation_state"])
        self.assertIsNone(payload["directions"]["rear"]["clearance_m"])

    def test_tilted_standing_base_fits_a_floor_plane(self):
        # A slightly tilted base sees a flat floor as a tilted plane; a single
        # height threshold would shift the collision band. The plane fit must
        # recover the tilt and keep the floor out of the collision band.
        pitch = math.radians(6.0)
        cloud = floor_wall_cloud(floor_min=0.35)
        tilted = np.column_stack([
            cloud[:, 0],
            cloud[:, 1],
            -math.sin(pitch) * cloud[:, 0] + math.cos(pitch) * cloud[:, 2],
        ])
        plane, evidence = module.fit_floor_plane(tilted)
        self.assertIsNotNone(plane, evidence)
        self.assertAlmostEqual(6.0, evidence["slope_deg"], delta=0.5)
        self.assertLess(evidence["fit_residual_m"], 0.02)
        floor_min, _ = module.floor_visible_min_range(tilted, floor_plane=plane)
        directions = module.direction_evidence(
            tilted, floor_plane=plane, radial_min_m=floor_min
        )
        front = directions["front"]
        self.assertEqual("measured", front["observation_state"], front["reason"])
        self.assertEqual("supported", front["floor_observation_state"])
        self.assertAlmostEqual(2.4, front["clearance_m"], delta=0.15)

    def test_wall_ring_alone_is_not_a_floor(self):
        wall_only = np.array(
            [
                [2.0 * math.cos(math.radians(a)), 2.0 * math.sin(math.radians(a)), h]
                for a in np.arange(-180.0, 180.0, 0.5)
                for h in np.arange(-0.6, 0.5, 0.05)
            ]
        )
        plane, evidence = module.fit_floor_plane(wall_only)
        self.assertIsNone(plane)
        self.assertEqual("floor_not_observed_across_radii", evidence["reason"])

    def test_near_field_floor_blind_ring_is_disclosed(self):
        cloud = floor_wall_cloud()
        payload = module.build_perception_payload(
            cloud,
            stamp_s=1.0,
            arrival_wall_s=2.0,
            producer_metadata={
                "stage": "LIO",
                "reference_stamp_ns": 1,
                "map_frame": "dynanav_lio_map",
                "child_frame": "go2_imu",
            },
        )
        near = payload["lidar_geometry"]["near_field_floor_unobserved_m"]
        self.assertEqual(0.0, near[0])
        self.assertGreater(near[1], 1.0)

    def test_far_floor_cannot_excuse_unobserved_near_field(self):
        cloud = floor_wall_cloud(floor_min=1.84)
        payload = module.build_perception_payload(
            cloud, stamp_s=1.0, arrival_wall_s=2.0,
            producer_metadata={"reference_stamp_ns": 1},
        )
        front = payload["directions"]["front"]
        self.assertEqual([0, 0], front["radial_bin_floor_points"][:2])
        self.assertTrue(front["observed_floor_supported"])
        self.assertEqual("partial", front["floor_observation_state"])
        self.assertFalse(front["floor_support_complete"])
        self.assertEqual(module.CORRIDOR_MIN_M, front["radial_start_m"])
        self.assertGreater(front["near_field_floor_unobserved_m"][1], 1.8)
        self.assertIsNone(front["clearance_m"])
        self.assertTrue(payload["fail_closed"])

    def test_partial_first_bin_does_not_hide_near_field_blindness(self):
        cloud = floor_wall_cloud(floor_min=0.8)
        front = module.direction_evidence(cloud, floor_z_m=-0.385)["front"]
        self.assertTrue(all(n >= 10 for n in front["radial_bin_floor_points"]))
        self.assertEqual("partial", front["floor_observation_state"])
        self.assertIn("near_field_floor_unobserved", front["reason"])
        self.assertIsNone(front["clearance_m"])

    def test_radial_hole_cannot_be_supported_by_collision_returns(self):
        cloud = floor_wall_cloud(floor_min=0.35)
        ranges = np.linalg.norm(cloud[:, :2], axis=1)
        hole = (ranges >= 1.0125) & (ranges < 1.675)
        # Dense obstacle returns in the hole still provide radial coverage, but
        # there is no measured ground beneath them.
        cloud[hole, 2] = 0.0
        front = module.direction_evidence(cloud, floor_z_m=-0.385)["front"]
        self.assertGreater(front["radial_bin_points"][1], 10)
        self.assertEqual(0, front["radial_bin_floor_points"][1])
        self.assertEqual("partial", front["floor_observation_state"])
        self.assertIsNone(front["clearance_m"])

    def test_thin_obstacle_in_blind_floor_remains_measured_but_not_authorized(self):
        cloud = np.vstack([floor_wall_cloud(), [[0.6, 0.0, -0.1]]])
        front = module.direction_evidence(
            cloud, floor_z_m=-0.385, radial_min_m=1.55,
        )["front"]
        self.assertAlmostEqual(0.6, front["nearest_collision_return_m"])
        self.assertEqual("partial", front["floor_observation_state"])
        self.assertIsNone(front["clearance_m"])


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

    def test_fail_closed_payload_is_never_free_space(self):
        payload = module.fail_closed_payload(stamp_s=1.0, arrival_wall_s=2.0, error="boom")
        self.assertTrue(payload["fail_closed"])
        self.assertIsNone(payload["clearance_m"])
        self.assertNotIn(payload["observation_state"], ("measured", "clear_free"))
        self.assertIsNone(payload["depth_valid_ratio"])


class RobustFloorPlaneTests(unittest.TestCase):
    """Actual coordinates, broad support and ambiguity must precede clearance."""

    @staticmethod
    def plane_cloud(*, azimuth=(0, 360), intercept=-0.35, slope=(-0.10, -0.02),
                    radial=(1.05, 3.35), step=.06):
        angles = np.deg2rad(np.arange(*azimuth, 1.0))
        radii = np.arange(*radial, step)
        angle, radius = np.meshgrid(angles, radii)
        x, y = radius.ravel() * np.cos(angle.ravel()), radius.ravel() * np.sin(angle.ravel())
        return np.column_stack([x, y, slope[0] * x + slope[1] * y + intercept])

    def test_actual_coordinates_preserve_slope_despite_nonuniform_cells(self):
        # Returns lie at cell edges, not the nominal midpoints used by the old
        # fitter. No invented x/y may be attached to a measured height.
        cloud = self.plane_cloud(step=.17)
        plane, evidence = module.fit_floor_plane(cloud)
        self.assertIsNotNone(plane, evidence)
        np.testing.assert_allclose(plane, [-.10, -.02, -.35], atol=1e-10)
        self.assertLess(evidence['fit_residual_m'], 1e-6)
        self.assertFalse(evidence['near_field_extrapolation_used'])

    def test_two_broad_levels_are_ambiguous_not_largest_or_highest(self):
        # A larger lower surface outside a ledge is not proof it is the robot's
        # supporting level. The smaller upper surface spans real radii too.
        upper = self.plane_cloud(azimuth=(0, 120), intercept=-.35, slope=(0, 0))
        lower = self.plane_cloud(azimuth=(120, 360), intercept=-.75, slope=(0, 0))
        for cloud in (np.vstack([upper, lower]), np.vstack([lower, upper])):
            plane, evidence = module.fit_floor_plane(cloud)
            self.assertIsNone(plane)
            self.assertEqual('floor_ambiguous_surfaces', evidence['reason'])
            self.assertGreaterEqual(evidence['coherent_surface_count'], 2)

    def test_two_levels_sharing_cells_are_not_hidden_by_low_quantile(self):
        lower = self.plane_cloud(intercept=-.75, slope=(0, 0))
        upper = self.plane_cloud(intercept=-.35, slope=(0, 0))
        plane, evidence = module.fit_floor_plane(np.vstack([lower, upper]))
        self.assertIsNone(plane)
        self.assertEqual('floor_ambiguous_surfaces', evidence['reason'])

    def test_two_independently_broad_half_circle_levels_still_ambiguous(self):
        upper = self.plane_cloud(azimuth=(0, 180), intercept=-.35, slope=(0, 0))
        lower = self.plane_cloud(azimuth=(180, 360), intercept=-.75, slope=(0, 0))
        plane, evidence = module.fit_floor_plane(np.vstack([upper, lower]))
        self.assertIsNone(plane)
        self.assertEqual('floor_ambiguous_surfaces', evidence['reason'])
        self.assertGreaterEqual(evidence['coherent_surface_count'], 2)

    def test_narrow_tread_points_remain_obstacles_against_broad_floor(self):
        floor = self.plane_cloud(intercept=-.55, slope=(0, 0))
        # A tread is narrow in radius; it cannot make another surrounding-floor
        # candidate, but its unmodified endpoints must still be collision points.
        tread = self.plane_cloud(azimuth=(0, 90), intercept=-.35, slope=(0, 0),
                                 radial=(1.1, 1.35), step=.01)
        cloud = np.vstack([floor, tread])
        plane, evidence = module.fit_floor_plane(cloud)
        self.assertIsNotNone(plane, evidence)
        np.testing.assert_allclose(plane, [0, 0, -.55], atol=.002)
        directions = module.direction_evidence(cloud, floor_plane=plane)
        self.assertGreater(directions['front']['collision_points'], 0)
        self.assertAlmostEqual(1.1, directions['front']['nearest_collision_return_m'], places=3)
        self.assertFalse(directions['front']['floor_support_complete'])

    def test_heading_zero_real_floor_rejects_shared_intersection_hypotheses(self):
        from pathlib import Path
        import json
        path = Path(__file__).resolve().parents[1] / 'fixtures/dynanav_rtx_floor_heading0_20261110_5200000000.npz'
        with np.load(path, allow_pickle=False) as fixture:
            provenance = json.loads(str(fixture['provenance_json']))
            self.assertEqual(int(fixture['source_ns']), provenance['source_pose']['source_ns'])
            cloud = module.map_points_to_base(fixture['points_map'], fixture['position'],
                                             fixture['quaternion_xyzw'], fixture['imu_mount'])
        plane, evidence = module.fit_floor_plane(cloud)
        self.assertIsNotNone(plane, evidence)
        self.assertEqual(1, evidence['coherent_surface_count'])
        self.assertGreaterEqual(len(evidence['sectors_with_radial_span']), 12)
        self.assertGreaterEqual(len(evidence['rejected_candidate_surfaces']), 3)
        for rejected in evidence['rejected_candidate_surfaces']:
            self.assertLess(len(rejected['sectors_with_radial_span']), 12)
        np.testing.assert_allclose(plane, [-.1032, -.0223, -.3435], atol=.002)
        for direction in module.direction_evidence(cloud, floor_plane=plane).values():
            self.assertFalse(direction['floor_support_complete'])
            self.assertIsNone(direction['clearance_m'])

    def test_narrow_platform_cannot_establish_surrounding_floor(self):
        cloud = self.plane_cloud(azimuth=(0, 90), slope=(0, 0))
        plane, evidence = module.fit_floor_plane(cloud)
        self.assertIsNone(plane)
        self.assertEqual('floor_not_observed_across_sectors', evidence['reason'])

    def test_stair_treads_do_not_become_a_continuous_ground_plane(self):
        cloud = self.plane_cloud(azimuth=(-40, 40), intercept=-.8, slope=(0, 0))
        cloud[:, 2] += .16 * np.floor((cloud[:, 0] - .8) / .3)
        plane, evidence = module.fit_floor_plane(cloud)
        self.assertIsNone(plane, evidence)

    def test_original_slope_and_residual_bounds_remain_enforced(self):
        for kwargs in ({'max_slope': .05}, {'max_residual_m': .00001}):
            cloud = self.plane_cloud()
            if 'max_residual_m' in kwargs:
                cloud[:, 2] += np.random.default_rng(4).normal(0, .005, len(cloud))
            plane, evidence = module.fit_floor_plane(cloud, **kwargs)
            self.assertIsNone(plane, evidence)

    def test_real_same_stamp_floor_fixture_stays_fail_closed_in_near_field(self):
        from pathlib import Path
        import json
        path = Path(__file__).resolve().parents[1] / 'fixtures/dynanav_rtx_floor_20261110_5200000000.npz'
        with np.load(path, allow_pickle=False) as fixture:
            provenance = json.loads(str(fixture['provenance_json']))
            self.assertEqual('LIO', provenance['stage'])
            self.assertEqual(5200000000, int(fixture['source_ns']))
            self.assertEqual(int(fixture['source_ns']), provenance['source_pose']['source_ns'])
            module.verify_deskew_marker(fixture['curvature'])
            cloud = module.map_points_to_base(fixture['points_map'], fixture['position'],
                                             fixture['quaternion_xyzw'], fixture['imu_mount'])
        plane, evidence = module.fit_floor_plane(cloud)
        self.assertIsNotNone(plane, evidence)
        # Independent rear/left/right plane fits of this real cloud agree at
        # c=-.343m and ~6 deg tilt. The old cell-centre fit gave c=-.267m/1.48deg.
        np.testing.assert_allclose(plane, [-.10324, -.02249, -.34351], atol=.002)
        self.assertGreater(evidence['inlier_points'], 5000)
        self.assertLess(evidence['fit_residual_m'], .002)
        directions = module.direction_evidence(cloud, floor_plane=plane)
        for entry in directions.values():
            self.assertEqual(0, entry['radial_bin_floor_points'][0])
            self.assertFalse(entry['floor_support_complete'])
            self.assertIsNone(entry['clearance_m'])
            self.assertIn('near_field_floor_unobserved', entry['reason'])


class FloorEquivalentOptimizationTests(unittest.TestCase):
    """Optimizations preserve every payload field and bound per-frame memory."""

    @staticmethod
    def fixture_cloud(name):
        from pathlib import Path
        with np.load(Path(__file__).resolve().parents[1] / 'fixtures' / name,
                     allow_pickle=False) as fixture:
            return module.map_points_to_base(fixture['points_map'], fixture['position'],
                                            fixture['quaternion_xyzw'], fixture['imu_mount'])

    def test_cache_convergence_and_batched_percentiles_preserve_full_payload(self):
        import json
        from collections import OrderedDict
        from unittest.mock import patch

        class NoCacheHits(OrderedDict):
            def __contains__(self, key):
                return False

        percentile = np.percentile

        def scalar_percentiles(array, quantiles, *args, **kwargs):
            if np.ndim(quantiles) == 0:
                return percentile(array, quantiles, *args, **kwargs)
            return np.asarray([percentile(array, value, *args, **kwargs)
                               for value in quantiles])

        for name in ('dynanav_rtx_floor_20261110_5200000000.npz',
                     'dynanav_rtx_floor_heading0_20261110_5200000000.npz'):
            cloud = self.fixture_cloud(name)
            kwargs = dict(stamp_s=5.2, arrival_wall_s=100.,
                          producer_metadata={'reference_stamp_ns': 5200000000})
            actual = module.build_perception_payload(cloud, **kwargs)
            # Recompute every support and all three refinement iterations;
            # evaluate the quantiles separately as in the prior implementation.
            with patch.object(module, 'OrderedDict', NoCacheHits), \
                    patch.object(module.np, 'array_equal', return_value=False), \
                    patch.object(module.np, 'percentile', side_effect=scalar_percentiles):
                expected = module.build_perception_payload(cloud, **kwargs)
            self.assertEqual(json.dumps(actual, sort_keys=True),
                             json.dumps(expected, sort_keys=True))

    def test_support_lru_has_eight_entries_and_never_crosses_frames(self):
        from collections import OrderedDict
        from unittest.mock import patch
        instances = []

        class ObservedLru(OrderedDict):
            def __init__(self):
                super().__init__()
                self.peak = self.evictions = self.hits = 0
                instances.append(self)

            def __setitem__(self, key, value):
                super().__setitem__(key, value)
                self.peak = max(self.peak, len(self))

            def popitem(self, last=True):
                self.evictions += 1
                return super().popitem(last=last)

            def move_to_end(self, key, last=True):
                self.hits += 1
                return super().move_to_end(key, last=last)

        cloud = self.fixture_cloud('dynanav_rtx_floor_heading0_20261110_5200000000.npz')
        with patch.object(module, 'OrderedDict', ObservedLru):
            first, _ = module.fit_floor_plane(cloud)
            second, _ = module.fit_floor_plane(cloud.copy())
        self.assertEqual(len(instances), 2)
        np.testing.assert_array_equal(first, second)
        for cache in instances:
            self.assertLessEqual(cache.peak, 8)
            self.assertGreater(cache.evictions, 0)
            self.assertGreater(cache.hits, 0)


if __name__ == "__main__":
    unittest.main()
