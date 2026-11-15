"""Offline tests for the operator-side measured occupancy/frontier map.

CPU only, no ROS graph, no Isaac and no GPU. The tests lock the honest-map
contract: explicit sensor->base extrinsic, stamp-paired measured poses,
measured-pose-only level clustering, low-quantile floor estimates, and a
frontier that is only actionable when every freshness/pairing/level/point gate
passes.

Grid scenarios are generated at cell centers so cell membership is exact
rather than dependent on floating-point rounding at cell boundaries.
"""

import json
import math
import time
import unittest

import numpy as np

from embodied_navigation.simulation.dynanav import operator_map as om


RES = 0.1


def pose(position=(0.0, 0.0, 0.0), yaw=0.0):
    """Measured 4x4 pose from a position and a yaw."""
    return om.transform_from_position_quaternion(
        position, om.quaternion_xyzw_from_yaw(yaw)
    )


def block(i0, j0, nx, ny, z, step=RES):
    """Points at cell centers of a rectangular block of cells."""
    xs = ((i0 + np.arange(nx)) + 0.5) * step
    ys = ((j0 + np.arange(ny)) + 0.5) * step
    gx, gy = np.meshgrid(xs, ys)
    return np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, float(z))], axis=1)


def floor_patch(nx=40, ny=40, z=0.0):
    return block(-nx // 2, -ny // 2, nx, ny, z)


def wall_cloud(half=5.0, z_low=0.3, z_high=2.0, step=RES):
    """Four walls at +-half, sampled in z; returns (points, wall_coords, zs)."""
    coords = -half + step * np.arange(int(round(2 * half / step)) + 1)
    zs = z_low + step * np.arange(int(round((z_high - z_low) / step)) + 1)
    rows = []
    for z in zs:
        for sign in (1.0, -1.0):
            rows.append(
                np.stack(
                    [np.full_like(coords, half * sign), coords, np.full_like(coords, z)],
                    axis=1,
                )
            )
            rows.append(
                np.stack(
                    [coords, np.full_like(coords, half * sign), np.full_like(coords, z)],
                    axis=1,
                )
            )
    return np.vstack(rows), coords, zs


def cells_of(points, step=RES):
    """Independent cell index set of map-frame points."""
    return {
        (math.floor(float(point[0]) / step), math.floor(float(point[1]) / step))
        for point in points
    }


IDENTITY_EXTRINSIC = om.Extrinsic(np.eye(4))


def snapshot_with(points, **overrides):
    """Build a one-shot snapshot with the shared fresh-pose defaults."""
    options = {
        "frame": "map",
        "extrinsic": IDENTITY_EXTRINSIC,
        "cloud_stamp_s": 10.0,
        "pose": pose((0.0, 0.0, 0.3)),
        "pose_stamp_s": 10.0,
        "sim_age_s": 0.05,
        "wall_age_s": 0.10,
        "resolution_m": RES,
        "free_mode": "floor",
    }
    options.update(overrides)
    return om.build_map_snapshot(points, **options)


class PosePairerTests(unittest.TestCase):
    def test_interpolates_position_and_orientation(self):
        pairer = om.PosePairer()
        pairer.add(0.0, pose((0.0, 0.0, 0.0), 0.0))
        pairer.add(0.1, pose((1.0, 0.0, 0.0), math.pi / 2.0))
        result = pairer.pair(0.05)
        self.assertTrue(result.ok)
        self.assertTrue(result.interpolated)
        self.assertAlmostEqual(result.dt_s, -0.05, places=9)
        self.assertAlmostEqual(result.used_pose_stamp_s, 0.0, places=9)
        self.assertAlmostEqual(result.query_stamp_s, 0.05, places=9)
        np.testing.assert_allclose(result.pose[:3, 3], [0.5, 0.0, 0.0], atol=1e-9)
        # A 45 degree yaw maps base +x to (cos45, sin45, 0).
        np.testing.assert_allclose(
            result.pose[:3, 0], [math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0], atol=1e-9
        )
        self.assertEqual(list(result.bracket_stamps), [0.0, 0.1])

    def test_uses_nearest_pose_within_pair_dt(self):
        pairer = om.PosePairer()
        pairer.add(0.0, pose((0.0, 0.0, 0.0)))
        pairer.add(0.1, pose((1.0, 0.0, 0.0)))
        pairer.add(0.3, pose((2.0, 0.0, 0.0)))
        result = pairer.pair(0.28)
        self.assertTrue(result.ok)
        self.assertFalse(result.interpolated)
        self.assertAlmostEqual(result.dt_s, 0.02, places=9)
        np.testing.assert_allclose(result.pose[:3, 3], [2.0, 0.0, 0.0], atol=1e-9)

    def test_interpolation_at_exact_pair_dt_boundary(self):
        pairer = om.PosePairer(max_pair_dt_s=0.10)
        pairer.add(0.0, pose((0.0, 0.0, 0.0)))
        pairer.add(0.2, pose((2.0, 0.0, 0.0)))
        result = pairer.pair(0.1)
        self.assertTrue(result.ok)
        self.assertTrue(result.interpolated)
        self.assertLessEqual(abs(result.dt_s), 0.10 + 1e-12)

    def test_refuses_beyond_pair_dt_and_records_reason(self):
        pairer = om.PosePairer(max_pair_dt_s=0.10)
        pairer.add(0.0, pose((0.0, 0.0, 0.0)))
        pairer.add(0.1, pose((1.0, 0.0, 0.0)))
        self.assertIsNone(pairer.pair_pose(0.35))
        last = pairer.last_pairing
        self.assertFalse(last.ok)
        self.assertEqual(last.reason, "pairing_gap")
        self.assertAlmostEqual(last.query_stamp_s, 0.35, places=9)
        self.assertAlmostEqual(last.used_pose_stamp_s, 0.1, places=9)
        self.assertAlmostEqual(last.dt_s, 0.1 - 0.35, places=9)
        recorded = last.as_dict()
        self.assertEqual(recorded["reason"], "pairing_gap")
        self.assertIsNotNone(recorded["dt_s"])

    def test_no_samples_refused(self):
        pairer = om.PosePairer()
        self.assertIsNone(pairer.pair_pose(1.0))
        self.assertEqual(pairer.last_pairing.reason, "no_pose")
        self.assertEqual(len(pairer), 0)

    def test_samples_sorted_and_dict_poses_accepted(self):
        pairer = om.PosePairer()
        identity = [0.0, 0.0, 0.0, 1.0]
        pairer.extend(
            [
                {
                    "stamp_s": 0.2,
                    "pose": {"position": [2.0, 0.0, 0.0], "quaternion_xyzw": identity},
                },
                {
                    "stamp_s": 0.0,
                    "pose": {"position": [0.0, 0.0, 0.0], "quaternion_xyzw": identity},
                },
            ]
        )
        np.testing.assert_allclose(pairer.stamps, [0.0, 0.2])
        result = pairer.pair(0.01)
        self.assertTrue(result.ok)
        np.testing.assert_allclose(result.pose[:3, 3], [0.0, 0.0, 0.0], atol=1e-9)


class ExtrinsicTests(unittest.TestCase):
    def test_valid_extrinsic_roundtrip_and_sha256(self):
        rotation = om.quaternion_xyzw_from_yaw(math.pi / 2.0)
        extrinsic = om.Extrinsic(translation=(1.0, 2.0, 3.0), quaternion_xyzw=rotation)
        expected = om.transform_from_position_quaternion((1.0, 2.0, 3.0), rotation)
        np.testing.assert_allclose(extrinsic.matrix, expected, atol=1e-12)
        self.assertEqual(len(extrinsic.sha256), 64)
        np.testing.assert_allclose(extrinsic.matrix @ extrinsic.inverse, np.eye(4), atol=1e-12)
        same = om.Extrinsic(expected)
        self.assertEqual(same.sha256, extrinsic.sha256)
        different_matrix = np.eye(4)
        different_matrix[:3, 3] = (1.0, 2.0, 4.0)
        different = om.Extrinsic(different_matrix)
        self.assertNotEqual(different.sha256, extrinsic.sha256)

    def test_missing_transform_refused(self):
        with self.assertRaises(ValueError):
            om.Extrinsic(None)
        with self.assertRaises(ValueError):
            om.Extrinsic(translation=(1.0, 0.0, 0.0))

    def test_non_orthonormal_rotation_refused(self):
        matrix = np.eye(4)
        matrix[:3, :3] *= 1.2
        with self.assertRaises(ValueError):
            om.Extrinsic(matrix)

    def test_non_finite_translation_refused(self):
        matrix = np.eye(4)
        matrix[0, 3] = float("nan")
        with self.assertRaises(ValueError):
            om.Extrinsic(matrix)

    def test_accepts_dict_and_pair_inputs(self):
        rotation = om.quaternion_xyzw_from_yaw(0.3)
        first = om.Extrinsic(
            {"position": [1.0, 2.0, 3.0], "quaternion_xyzw": rotation.tolist()}
        )
        second = om.Extrinsic(((1.0, 2.0, 3.0), rotation))
        self.assertEqual(first.sha256, second.sha256)

    def test_sensor_and_base_frames_must_differ(self):
        with self.assertRaises(ValueError):
            om.Extrinsic(np.eye(4), sensor_frame="base", base_frame="base")


class PointCloudDeclarationTests(unittest.TestCase):
    def test_unknown_and_missing_frame_refused(self):
        points = floor_patch(2, 2)
        for frame in ("world", "", None):
            with self.assertRaises(ValueError):
                om.points_to_map(points, frame)

    def test_wrong_shape_refused(self):
        with self.assertRaises(ValueError):
            om.points_to_map(np.zeros((3, 2)), "map")

    def test_sensor_frame_applies_pose_and_extrinsic(self):
        extrinsic = om.Extrinsic(
            translation=(0.0, 0.0, 0.5),
            quaternion_xyzw=om.quaternion_xyzw_from_yaw(math.pi / 2.0),
        )
        points = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        normalized = om.points_to_map(
            points, "sensor", pose=pose((2.0, 3.0, 4.0)), extrinsic=extrinsic
        )
        np.testing.assert_allclose(
            normalized.points_map,
            [[2.0, 4.0, 4.5], [1.0, 3.0, 4.5], [2.0, 3.0, 5.5]],
            atol=1e-9,
        )

    def test_base_and_map_frames(self):
        points = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        base = om.points_to_map(points, "base", pose=pose((2.0, 3.0, 4.0)))
        np.testing.assert_allclose(base.points_map, [[3.0, 3.0, 4.0], [2.0, 4.0, 4.0], [2.0, 3.0, 5.0]], atol=1e-9)
        mapped = om.points_to_map(points, "map", pose=pose((2.0, 3.0, 4.0)))
        np.testing.assert_allclose(mapped.points_map, points, atol=1e-12)

    def test_sensor_declaration_with_map_scale_refused(self):
        points = np.array([[100.0, 0.0, 0.0], [0.0, 100.0, 0.0]])
        with self.assertRaises(ValueError):
            om.points_to_map(
                points, "sensor", pose=pose((0.0, 0.0, 0.0)), extrinsic=IDENTITY_EXTRINSIC
            )

    def test_missing_extrinsic_or_pose_refused(self):
        points = np.array([[1.0, 0.0, 0.0]])
        with self.assertRaises(ValueError):
            om.points_to_map(points, "sensor", pose=pose())
        with self.assertRaises(ValueError):
            om.points_to_map(points, "sensor", extrinsic=IDENTITY_EXTRINSIC)
        with self.assertRaises(ValueError):
            om.points_to_map(points, "base")

    def test_non_finite_points_dropped(self):
        points = np.array([[0.5, 0.5, 0.0], [float("nan"), 0.0, 0.0]])
        normalized = om.points_to_map(points, "map")
        self.assertEqual(normalized.points_map.shape[0], 1)
        self.assertEqual(normalized.dropped_nonfinite, 1)

    def test_depth_image_to_points_analytic(self):
        depth = np.array(
            [[1.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 2.0, 1.0]], dtype=float
        )
        points = om.depth_image_to_points(depth, fx=1.0, fy=1.0, cx=1.0, cy=1.0)
        self.assertEqual(points.shape[0], 9)
        # Pixel (u=1, v=2) with depth 2: x=0, y=2, z=2 in optical convention.
        matches = np.all(np.isclose(points, [0.0, 2.0, 2.0]), axis=1)
        self.assertEqual(int(matches.sum()), 1)
        zeros = om.depth_image_to_points(
            np.zeros((2, 2)), fx=1.0, fy=1.0, cx=0.5, cy=0.5
        )
        self.assertEqual(zeros.shape, (0, 3))


class LevelTrackerTests(unittest.TestCase):
    def test_three_measured_clusters_stay_separate(self):
        tracker = om.LevelTracker()
        for expected, z in enumerate((-3.3, 0.0, 3.3)):
            match = tracker.observe(z)
            self.assertFalse(match.ambiguous)
            self.assertEqual(match.level_index, expected)
            self.assertAlmostEqual(match.level_z_m, z, places=9)
        levels = tracker.levels
        self.assertEqual(len(levels), 3)
        for level, expected in zip(levels, (-3.3, 0.0, 3.3)):
            self.assertAlmostEqual(level.z_m, expected, places=9)

    def test_jitter_hysteresis_and_spurious_level(self):
        tracker = om.LevelTracker(band_width_m=0.3, hysteresis_m=0.15)
        self.assertEqual(tracker.observe(0.0).level_index, 0)
        for z in (0.02, -0.03, 0.01):
            self.assertEqual(tracker.observe(z).level_index, 0)
        self.assertEqual(len(tracker.levels), 1)
        # 0.29 m is inside the claimed band even though it is beyond the
        # 0.15 m band half width: hysteresis keeps it on the same level.
        match = tracker.observe(0.29)
        self.assertEqual(match.level_index, 0)
        self.assertFalse(match.created)
        # Far beyond the claimed radius a genuinely new level is created.
        match = tracker.observe(1.4)
        self.assertEqual(match.level_index, 1)
        self.assertTrue(match.created)
        self.assertEqual(len(tracker.levels), 2)

    def test_ambiguous_level_reported_without_update(self):
        tracker = om.LevelTracker()
        tracker.observe(0.0)
        tracker.observe(0.5)
        self.assertEqual(len(tracker.levels), 2)
        before = [(level.index, level.z_m, level.observations) for level in tracker.levels]
        match = tracker.observe(0.25)
        self.assertTrue(match.ambiguous)
        self.assertIsNone(match.level_index)
        self.assertIsNone(match.level_z_m)
        self.assertIn("ambiguous_level", match.reasons)
        after = [(level.index, level.z_m, level.observations) for level in tracker.levels]
        self.assertEqual(before, after)

    def test_level_z_comes_from_measured_pose_only(self):
        tracker = om.LevelTracker()
        match = tracker.observe(1.234)
        self.assertAlmostEqual(match.level_z_m, 1.234, places=9)
        self.assertAlmostEqual(tracker.level_z_m(0), 1.234, places=9)

    def test_too_many_levels_refused(self):
        tracker = om.LevelTracker(max_levels=2)
        tracker.observe(0.0)
        tracker.observe(1.0)
        match = tracker.observe(2.0)
        self.assertIsNone(match.level_index)
        self.assertFalse(match.ambiguous)
        self.assertIn("too_many_levels", match.reasons)


class LevelGridTests(unittest.TestCase):
    def test_frontier_adjacency_and_sorting(self):
        grid = om.LevelGrid(RES)
        grid.mark_free_points(block(0, 0, 3, 3, 0.0)[:, :2])
        frontier = grid.frontier((-0.05, -0.05))
        self.assertEqual(len(frontier), 8)  # 3x3 block: only the border touches unknown
        self.assertNotIn((1, 1), frontier)  # center is surrounded by known free cells
        distances = [
            math.hypot((ix + 0.5) * RES + 0.05, (iy + 0.5) * RES + 0.05)
            for ix, iy in frontier
        ]
        self.assertEqual(distances, sorted(distances))
        self.assertEqual(grid.code_at(1, 1), om.FREE)
        self.assertEqual(grid.code_at(999, 999), om.UNKNOWN)

    def test_occupied_wins_over_free(self):
        grid = om.LevelGrid(RES)
        grid.mark_free_points(np.array([[0.05, 0.05]]))
        grid.mark_occupied_points(np.array([[0.05, 0.05]]))
        self.assertEqual(grid.code_at(0, 0), om.OCCUPIED)
        grid.mark_free_points(np.array([[0.05, 0.05]]))
        self.assertEqual(grid.code_at(0, 0), om.OCCUPIED)

    def test_rays_mark_free_but_not_obstacle_cell(self):
        grid = om.LevelGrid(RES)
        target = (20, 0)
        grid.mark_occupied_points(np.array([[(20 + 0.5) * RES, 0.05]]))
        grid.mark_free_rays((0.05, 0.05), [target])
        self.assertEqual(grid.code_at(*target), om.OCCUPIED)
        free_between = [
            (ix, 0)
            for ix in range(1, 20)
            if grid.code_at(ix, 0) == om.FREE
        ]
        self.assertTrue(free_between)
        self.assertEqual(len(grid.cells()), 1 + len(free_between))

    def test_unknown_boundary_area(self):
        grid = om.LevelGrid(RES)
        grid.mark_free_points(block(-20, -20, 40, 40, 0.0)[:, :2])
        # Rectangle of 40x40 cells has a 42x42 boundary ring of unknown cells.
        self.assertEqual(len(grid.unknown_boundary_cells()), 42 * 42 - 40 * 40)
        self.assertAlmostEqual(
            grid.unknown_estimate_area_m2(), (42 * 42 - 40 * 40) * RES ** 2, places=12
        )


class KnownSceneSnapshotTests(unittest.TestCase):
    def setUp(self):
        floor = floor_patch(60, 60, 0.0)
        self.walls, self.wall_coords, self.wall_z = wall_cloud()
        self.cloud = np.vstack([floor, self.walls])
        self.snapshot = snapshot_with(self.cloud, pose=pose((0.0, 0.0, 0.3)))

    def test_occupancy_free_unknown_and_frontier_counts(self):
        snapshot = self.snapshot
        expected_occupied = cells_of(self.walls)
        self.assertEqual(snapshot["occupied_cells"], len(expected_occupied))
        self.assertEqual(snapshot["free_cells"], 60 * 60)
        self.assertEqual(snapshot["free_cells"], 3600)
        free_cells = {(i, j) for i in range(-30, 30) for j in range(-30, 30)}
        self.assertEqual(len(free_cells), 3600)
        self.assertEqual(snapshot["frontier"]["count"], 4 * 60 - 4)
        self.assertAlmostEqual(
            snapshot["coverage_area_m2"], 0.01 * (len(expected_occupied) + 3600), places=9
        )
        # Unknown boundary ring around the observed free patch and the walls.
        known = free_cells | expected_occupied
        boundary = set()
        for ix, iy in known:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbour = (ix + dx, iy + dy)
                    if neighbour not in known:
                        boundary.add(neighbour)
        self.assertAlmostEqual(
            snapshot["unknown_estimate_area_m2"], len(boundary) * RES ** 2, places=9
        )
        self.assertEqual(snapshot["level_z_m"], 0.3)
        self.assertAlmostEqual(snapshot["floor_offset_m"], -0.3, places=9)
        self.assertTrue(snapshot["frontier"]["actionable"])
        self.assertEqual(snapshot["frontier"]["reasons"], [])

    def test_occupied_cell_positions_are_on_the_walls(self):
        snapshot = self.snapshot
        for cell in snapshot["frontier"]["cell_indices"]:
            self.assertIn(tuple(cell), {(i, j) for i in range(-30, 30) for j in range(-30, 30)})
        # Recover occupied cells from their reported frontier? The snapshot only
        # exposes counts; use an accumulator to inspect the cells themselves.
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        accumulator.add_frame(
            self.cloud,
            frame="map",
            cloud_stamp_s=10.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=10.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        grid = accumulator.grid_for(0)
        occupied = [cell for cell, code in grid.cells().items() if code == om.OCCUPIED]
        self.assertEqual(len(occupied), snapshot["occupied_cells"])
        for ix, iy in occupied:
            x = (ix + 0.5) * RES
            y = (iy + 0.5) * RES
            distance = min(abs(abs(x) - 5.0), abs(abs(y) - 5.0))
            self.assertLessEqual(distance, RES + 1e-9)
        free = [cell for cell, code in grid.cells().items() if code == om.FREE]
        for ix, iy in free:
            x = (ix + 0.5) * RES
            y = (iy + 0.5) * RES
            self.assertLessEqual(abs(x), 3.0 + 1e-9)
            self.assertLessEqual(abs(y), 3.0 + 1e-9)

    def test_frontier_sorted_by_distance_and_on_patch_boundary(self):
        frontier = self.snapshot["frontier"]
        distances = [math.hypot(cell[0], cell[1]) for cell in frontier["cells"]]
        self.assertEqual(distances, sorted(distances))
        self.assertAlmostEqual(frontier["min_distance_m"], distances[0], places=6)
        for cell in frontier["cells"]:
            self.assertAlmostEqual(max(abs(cell[0]), abs(cell[1])), 2.95, places=6)

    def test_required_snapshot_schema_and_json(self):
        required = {
            "schema_version",
            "frames",
            "sources",
            "stamps",
            "level_z_m",
            "floor_offset_m",
            "resolution_m",
            "occupied_cells",
            "free_cells",
            "unknown_estimate_area_m2",
            "coverage_area_m2",
            "frontier",
            "measured_path_length_m",
            "path_length_is_not_coverage",
            "snapshot_is_accumulated_map",
            "ground_truth_used",
            "extrinsic_sha256",
            "note",
        }
        snapshot = self.snapshot
        self.assertTrue(required.issubset(snapshot))
        self.assertEqual(snapshot["schema_version"], om.SCHEMA_VERSION)
        self.assertEqual(set(snapshot["frames"]), {"sensor", "base", "map"})
        self.assertEqual(set(snapshot["stamps"]), {"pose", "cloud", "paired_dt"})
        self.assertTrue({"cells", "count", "actionable", "reasons"}.issubset(snapshot["frontier"]))
        self.assertEqual(snapshot["sources"], {"pose_source": "lio_estimate", "cloud_source": "depth"})
        self.assertEqual(snapshot["extrinsic_sha256"], IDENTITY_EXTRINSIC.sha256)
        self.assertIs(snapshot["path_length_is_not_coverage"], True)
        self.assertIs(snapshot["ground_truth_used"], False)
        self.assertFalse(snapshot["snapshot_is_accumulated_map"])
        self.assertIsInstance(snapshot["note"], str)
        self.assertAlmostEqual(snapshot["stamps"]["paired_dt"], 0.0, places=9)
        json.dumps(snapshot)
        overridden = snapshot_with(
            floor_patch(20, 20), snapshot_is_accumulated_map=True
        )
        self.assertTrue(overridden["snapshot_is_accumulated_map"])

    def test_raw_rotating_rtx_scan_is_not_mapped_with_one_pose(self):
        with self.assertRaisesRegex(ValueError, "IMU deskew"):
            snapshot_with(floor_patch(20, 20), cloud_source="/dynanav/lidar_rtx")

    def test_ground_truth_sources_refused(self):
        for source in ("ground_truth", "native_truth", "simulator_native_pose"):
            with self.assertRaises(ValueError):
                snapshot_with(floor_patch(20, 20), pose_source=source)
        with self.assertRaises(ValueError):
            snapshot_with(floor_patch(20, 20), cloud_source="gt_mesh")


class FloorAndClearanceTests(unittest.TestCase):
    def test_floor_offset_from_low_quantile_relative_to_measured_level(self):
        # Floor 0.4 m below the measured base; the level comes from the pose,
        # the floor from the low quantile of this level's points.
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        floor = block(0, 0, 10, 10, 1.2)
        low = block(20, 0, 10, 10, 1.3)
        wall = block(40, 0, 10, 10, 1.7)
        high = block(60, 0, 10, 10, 3.5)
        below = block(80, 0, 10, 10, 0.9)[:10]
        cloud = np.vstack([floor, low, wall, high, below])
        receipt = accumulator.add_frame(
            cloud,
            frame="map",
            cloud_stamp_s=1.0,
            pose=pose((0.0, 0.0, 1.6)),
            pose_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertTrue(receipt["accepted"])
        self.assertEqual(receipt["level_z_m"], 1.6)
        self.assertAlmostEqual(receipt["floor_offset_m"], -0.4, places=9)
        snapshot = accumulator.snapshot()
        self.assertEqual(snapshot["occupied_cells"], 100)  # only the 1.7 m block
        self.assertEqual(snapshot["free_cells"], 200)  # floor and 0.1 m-above-floor block
        self.assertAlmostEqual(snapshot["floor_offset_m"], -0.4, places=9)
        grid = accumulator.grid_for(0)
        self.assertEqual(grid.code_at(*grid.index_of(0.05, 0.05)), om.FREE)
        self.assertEqual(grid.code_at(*grid.index_of(2.05, 0.05)), om.FREE)
        self.assertEqual(grid.code_at(*grid.index_of(4.05, 0.05)), om.OCCUPIED)
        self.assertEqual(grid.code_at(*grid.index_of(6.05, 0.05)), om.UNKNOWN)
        self.assertEqual(grid.code_at(*grid.index_of(8.05, 0.05)), om.UNKNOWN)

    def test_level_z_uses_measured_pose_not_a_world_constant(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        cloud = block(-10, -10, 20, 20, 0.834)
        receipt = accumulator.add_frame(
            cloud,
            frame="map",
            cloud_stamp_s=1.0,
            pose=pose((0.0, 0.0, 1.234)),
            pose_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertEqual(receipt["level_index"], 0)
        self.assertAlmostEqual(receipt["level_z_m"], 1.234, places=9)
        self.assertAlmostEqual(receipt["floor_offset_m"], 0.834 - 1.234, places=9)
        snapshot = accumulator.snapshot(0)
        self.assertAlmostEqual(snapshot["level_z_m"], 1.234, places=9)
        self.assertAlmostEqual(snapshot["floor_offset_m"], -0.4, places=9)


class ActionableGateTests(unittest.TestCase):
    def setUp(self):
        self.cloud = floor_patch(40, 40)

    def test_baseline_all_gates_pass(self):
        snapshot = snapshot_with(self.cloud)
        self.assertTrue(snapshot["frontier"]["actionable"])
        self.assertEqual(snapshot["frontier"]["reasons"], [])
        self.assertEqual(
            snapshot["gates"],
            {
                "pose_fresh": True,
                "pairing": True,
                "extrinsic": True,
                "level": True,
                "points": True,
            },
        )
        self.assertEqual(snapshot["frontier"]["count"], 156)

    def test_stale_sim_age_gate(self):
        snapshot = snapshot_with(self.cloud, sim_age_s=0.30)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("stale_pose_sim_age", snapshot["frontier"]["reasons"])
        self.assertFalse(snapshot["gates"]["pose_fresh"])
        # The frontier is still returned, only marked non-actionable.
        self.assertEqual(snapshot["frontier"]["count"], 156)

    def test_stale_wall_age_gate(self):
        snapshot = snapshot_with(self.cloud, wall_age_s=0.8)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("stale_pose_wall_age", snapshot["frontier"]["reasons"])
        self.assertEqual(snapshot["frontier"]["count"], 156)

    def test_missing_pose_age_gate_is_fail_closed(self):
        snapshot = snapshot_with(self.cloud, sim_age_s=None, wall_age_s=None)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("pose_age_unknown", snapshot["frontier"]["reasons"])

    def test_pairing_gap_gate(self):
        snapshot = snapshot_with(
            self.cloud, cloud_stamp_s=10.0, pose_stamp_s=10.5
        )
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("pairing_gap", snapshot["frontier"]["reasons"])
        self.assertFalse(snapshot["gates"]["pairing"])
        self.assertAlmostEqual(snapshot["stamps"]["paired_dt"], 0.5, places=9)
        self.assertEqual(snapshot["frontier"]["count"], 156)

    def test_insufficient_points_gate_and_override(self):
        small = floor_patch(10, 10)  # 100 points
        snapshot = snapshot_with(small)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("insufficient_points", snapshot["frontier"]["reasons"])
        self.assertEqual(snapshot["frontier"]["count"], 36)
        relaxed = snapshot_with(small, min_points=50)
        self.assertTrue(relaxed["frontier"]["actionable"])
        self.assertEqual(relaxed["frontier"]["reasons"], [])

    def test_ambiguous_level_gate(self):
        tracker = om.LevelTracker()
        tracker.observe(0.0)
        tracker.observe(0.5)
        snapshot = snapshot_with(
            self.cloud, level_tracker=tracker, pose=pose((0.0, 0.0, 0.25))
        )
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("ambiguous_level", snapshot["frontier"]["reasons"])
        self.assertFalse(snapshot["gates"]["level"])
        self.assertIsNone(snapshot["level_z_m"])
        self.assertEqual(snapshot["frontier"]["count"], 0)

    def test_missing_extrinsic_refused(self):
        with self.assertRaises(ValueError):
            om.build_map_snapshot(
                self.cloud,
                frame="map",
                extrinsic=None,
                cloud_stamp_s=10.0,
                pose=pose((0.0, 0.0, 0.3)),
                pose_stamp_s=10.0,
                sim_age_s=0.05,
                wall_age_s=0.10,
                resolution_m=RES,
            )
        accumulator = om.OccupancyAccumulator(extrinsic=None, resolution_m=RES)
        receipt = accumulator.add_frame(
            self.cloud,
            frame="map",
            cloud_stamp_s=10.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=10.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertFalse(receipt["accepted"])
        self.assertIn("extrinsic_missing", receipt["reasons"])
        snapshot = accumulator.snapshot()
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("extrinsic_invalid", snapshot["frontier"]["reasons"])


class AccumulatorTests(unittest.TestCase):
    def test_levels_have_independent_grids_without_mixing(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        for index, z in enumerate((-3.3, 0.0, 3.3)):
            floor = block(index * 40, 0, 15, 15, z - 0.4)
            wall = block(index * 40 + 20, 0, 15, 15, z + 0.5)
            receipt = accumulator.add_frame(
                np.vstack([floor, wall]),
                frame="map",
                cloud_stamp_s=1.0 + index,
                pose=pose((index * 4.0, 0.0, z)),
                pose_stamp_s=1.0 + index,
                sim_age_s=0.05,
                wall_age_s=0.10,
            )
            self.assertTrue(receipt["accepted"])
            self.assertEqual(receipt["level_index"], index)
            self.assertAlmostEqual(receipt["level_z_m"], z, places=9)
        self.assertEqual([z for _, z in accumulator.levels], [-3.3, 0.0, 3.3])
        for index in range(3):
            snapshot = accumulator.snapshot(index)
            self.assertEqual(snapshot["level_index"], index)
            self.assertEqual(snapshot["occupied_cells"], 15 * 15)
            self.assertEqual(snapshot["free_cells"], 15 * 15)
            self.assertAlmostEqual(snapshot["floor_offset_m"], -0.4, places=9)
            self.assertTrue(snapshot["frontier"]["actionable"])
        # Level 0 must not contain the cells observed for level 1 (cell 40..54
        # hold level 1's floor block) but must hold its own wall block (20..34).
        grid0 = accumulator.grid_for(0)
        self.assertEqual(grid0.code_at(40, 0), om.UNKNOWN)
        self.assertEqual(grid0.code_at(20, 0), om.OCCUPIED)

    def test_two_frames_monotonic_area_and_frontier(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        first = block(0, 0, 20, 20, 0.0)
        second = np.vstack(
            [block(-20, 0, 20, 20, 0.0), block(0, 0, 10, 10, 1.0)]
        )
        accumulator.add_frame(
            first,
            frame="map",
            cloud_stamp_s=1.0,
            pose=pose((0.5, 0.5, 0.3)),
            pose_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        after_first = accumulator.snapshot()
        accumulator.add_frame(
            second,
            frame="map",
            cloud_stamp_s=2.0,
            pose=pose((-1.5, 0.5, 0.3)),
            pose_stamp_s=2.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        after_second = accumulator.snapshot()
        self.assertFalse(after_first["snapshot_is_accumulated_map"])
        self.assertTrue(after_second["snapshot_is_accumulated_map"])
        self.assertEqual(after_second["accumulated_frames"], 2)
        self.assertGreater(
            after_second["coverage_area_m2"], after_first["coverage_area_m2"]
        )
        self.assertGreaterEqual(
            after_second["free_cells"], after_first["free_cells"]
        )
        self.assertGreaterEqual(
            after_second["occupied_cells"], after_first["occupied_cells"]
        )
        self.assertGreaterEqual(
            after_second["frontier"]["count"], after_first["frontier"]["count"]
        )
        self.assertGreater(after_second["unknown_estimate_area_m2"], 0.0)
        self.assertTrue(after_second["frontier"]["actionable"])
        self.assertAlmostEqual(after_second["measured_path_length_m"], 2.0, places=9)

    def test_path_length_is_not_coverage(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        for index, x in enumerate((0.0, 1.0)):
            accumulator.add_frame(
                block(0, 0, 10, 10, 0.0),
                frame="map",
                cloud_stamp_s=1.0 + index,
                pose=pose((x, 0.0, 0.3)),
                pose_stamp_s=1.0 + index,
                sim_age_s=0.05,
                wall_age_s=0.10,
            )
        snapshot = accumulator.snapshot()
        self.assertAlmostEqual(snapshot["measured_path_length_m"], 1.0, places=9)
        self.assertAlmostEqual(snapshot["coverage_area_m2"], 1.0, places=9)
        self.assertIs(snapshot["path_length_is_not_coverage"], True)

    def test_merge_accumulators(self):
        first = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        first.add_frame(
            block(0, 0, 10, 10, 0.0),
            frame="map",
            cloud_stamp_s=1.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        second = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        second.add_frame(
            block(20, 0, 10, 10, 0.0),
            frame="map",
            cloud_stamp_s=1.0,
            pose=pose((2.0, 0.0, 0.3)),
            pose_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        first.merge(second)
        snapshot = first.snapshot()
        self.assertEqual(snapshot["free_cells"], 200)
        self.assertEqual(snapshot["accumulated_frames"], 2)
        self.assertTrue(snapshot["snapshot_is_accumulated_map"])
        self.assertAlmostEqual(snapshot["coverage_area_m2"], 2.0, places=9)
        grid = first.grid_for(0)
        self.assertEqual(grid.code_at(*grid.index_of(0.05, 0.05)), om.FREE)
        self.assertEqual(grid.code_at(*grid.index_of(2.05, 0.05)), om.FREE)

    def test_merge_registers_unmatched_level_for_future_frames(self):
        first = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        first.add_frame(
            block(0, 0, 10, 10, 0.0),
            frame="map",
            cloud_stamp_s=1.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        upper = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        upper.add_frame(
            block(40, 0, 15, 15, 2.9),
            frame="map",
            cloud_stamp_s=1.0,
            pose=pose((0.0, 0.0, 3.3)),
            pose_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        first.merge(upper)
        self.assertEqual([z for _, z in first.levels], [0.3, 3.3])
        upper_snapshot = first.snapshot(1)
        self.assertAlmostEqual(upper_snapshot["level_z_m"], 3.3, places=9)
        self.assertEqual(upper_snapshot["free_cells"], 15 * 15)
        # A later frame on the merged floor must land in the same level grid.
        receipt = first.add_frame(
            block(60, 0, 10, 10, 2.9),
            frame="map",
            cloud_stamp_s=2.0,
            pose=pose((0.0, 0.0, 3.3)),
            pose_stamp_s=2.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertEqual(receipt["level_index"], 1)
        after = first.snapshot(1)
        self.assertEqual(after["free_cells"], 15 * 15 + 10 * 10)
        self.assertEqual(first.snapshot(0)["free_cells"], 10 * 10)

    def test_merge_rejects_mismatched_extrinsic_or_resolution(self):
        base = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        moved = om.OccupancyAccumulator(
            extrinsic=om.Extrinsic(
                translation=(1.0, 0.0, 0.0),
                quaternion_xyzw=om.quaternion_xyzw_from_yaw(0.0),
            ),
            resolution_m=RES,
            free_mode="floor",
        )
        with self.assertRaises(ValueError):
            base.merge(moved)
        finer = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=0.05, free_mode="floor"
        )
        with self.assertRaises(ValueError):
            base.merge(finer)
        with self.assertRaises(TypeError):
            base.merge(object())

    def test_pose_stream_pairing_without_explicit_pose(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        accumulator.add_pose(1.0, pose((0.0, 0.0, 0.3)))
        receipt = accumulator.add_frame(
            floor_patch(40, 40),
            frame="map",
            cloud_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertTrue(receipt["accepted"])
        snapshot = accumulator.snapshot()
        self.assertEqual(snapshot["stamps"]["pose"], 1.0)
        self.assertAlmostEqual(snapshot["stamps"]["paired_dt"], 0.0, places=9)
        self.assertEqual(snapshot["pairing"]["used_pose_stamp_s"], 1.0)
        self.assertTrue(snapshot["frontier"]["actionable"])

    def test_map_frame_without_any_measured_pose_refused(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        receipt = accumulator.add_frame(
            floor_patch(40, 40),
            frame="map",
            cloud_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertFalse(receipt["accepted"])
        self.assertIn("pose_unavailable", receipt["reasons"])
        snapshot = accumulator.snapshot()
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("pose_unavailable", snapshot["frontier"]["reasons"])

    def test_per_level_point_count_gate(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        accumulator.add_frame(
            block(0, 0, 10, 10, 0.0),
            frame="map",
            cloud_stamp_s=1.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        accumulator.add_frame(
            block(40, 0, 20, 20, 2.9),
            frame="map",
            cloud_stamp_s=2.0,
            pose=pose((0.0, 0.0, 3.3)),
            pose_stamp_s=2.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        low = accumulator.snapshot(0)
        high = accumulator.snapshot(1)
        self.assertFalse(low["frontier"]["actionable"])
        self.assertIn("insufficient_points", low["frontier"]["reasons"])
        self.assertTrue(high["frontier"]["actionable"])
        self.assertEqual(low["level_point_count"], 100)
        self.assertEqual(high["level_point_count"], 400)

    def test_custom_frame_names_are_reported(self):
        extrinsic = om.Extrinsic(
            np.eye(4), sensor_frame="depth_optical", base_frame="base_link"
        )
        accumulator = om.OccupancyAccumulator(
            extrinsic=extrinsic, resolution_m=RES, free_mode="floor", map_frame="odom_map"
        )
        snapshot = accumulator.snapshot()
        self.assertEqual(
            snapshot["frames"],
            {"sensor": "depth_optical", "base": "base_link", "map": "odom_map"},
        )

    def test_ambiguous_frame_keeps_previous_frontier_but_non_actionable(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        accumulator.add_frame(
            floor_patch(40, 40),
            frame="map",
            cloud_stamp_s=1.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        accepted = accumulator.snapshot()
        self.assertTrue(accepted["frontier"]["actionable"])
        tracker = accumulator.level_tracker
        tracker.observe(0.65)  # a second measured level, close enough to be ambiguous
        receipt = accumulator.add_frame(
            floor_patch(40, 40),
            frame="map",
            cloud_stamp_s=2.0,
            pose=pose((0.0, 0.0, 0.475)),
            pose_stamp_s=2.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertFalse(receipt["accepted"])
        self.assertIn("ambiguous_level", receipt["reasons"])
        snapshot = accumulator.snapshot()
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("ambiguous_level", snapshot["frontier"]["reasons"])
        # The previously observed frontier is still returned, only marked
        # non-actionable, so the operator can see the map and the refusal.
        self.assertEqual(snapshot["frontier"]["count"], accepted["frontier"]["count"])
        self.assertEqual(snapshot["accumulated_frames"], 1)

    def test_empty_accumulator_refuses_actionable_snapshot(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES
        )
        snapshot = accumulator.snapshot()
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("no_map", snapshot["frontier"]["reasons"])
        self.assertEqual(snapshot["frontier"]["count"], 0)
        self.assertIsNone(snapshot["level_z_m"])

    def test_sensor_frame_matches_map_frame_result(self):
        extrinsic = om.Extrinsic(
            translation=(0.1, -0.2, 0.3),
            quaternion_xyzw=om.quaternion_xyzw_from_yaw(0.3),
        )
        measured = pose((0.4, -0.7, 0.3), 0.2)
        map_points = floor_patch(40, 40)
        transform = measured @ extrinsic.matrix
        homogeneous = np.hstack([map_points, np.ones((map_points.shape[0], 1))])
        sensor_points = (np.linalg.inv(transform) @ homogeneous.T).T[:, :3]
        map_snapshot = snapshot_with(map_points, pose=measured)
        sensor_snapshot = snapshot_with(
            sensor_points, frame="sensor", extrinsic=extrinsic, pose=measured
        )
        self.assertEqual(map_snapshot["free_cells"], sensor_snapshot["free_cells"])
        self.assertEqual(map_snapshot["occupied_cells"], sensor_snapshot["occupied_cells"])
        self.assertEqual(
            map_snapshot["frontier"]["count"], sensor_snapshot["frontier"]["count"]
        )
        self.assertAlmostEqual(
            map_snapshot["level_z_m"], sensor_snapshot["level_z_m"], places=9
        )
        self.assertTrue(sensor_snapshot["frontier"]["actionable"])


class FreeModeTests(unittest.TestCase):
    def setUp(self):
        floor = block(-5, -5, 10, 10, 0.0)
        ring = []
        span = np.arange(-21, 21)
        xs = (span + 0.5) * RES
        for i in (20, -21):
            ring.append(
                np.stack(
                    [np.full_like(xs, (i + 0.5) * RES), xs, np.full_like(xs, 1.0)],
                    axis=1,
                )
            )
        for j in (20, -21):
            ring.append(
                np.stack(
                    [xs, np.full_like(xs, (j + 0.5) * RES), np.full_like(xs, 1.0)],
                    axis=1,
                )
            )
        self.cloud = np.vstack([floor] + ring)

    def test_rays_build_observed_free_without_freeing_obstacles(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="rays"
        )
        receipt = accumulator.add_frame(
            self.cloud,
            frame="map",
            cloud_stamp_s=1.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertTrue(receipt["accepted"])
        snapshot = accumulator.snapshot()
        self.assertGreater(snapshot["free_cells"], 0)
        self.assertGreater(snapshot["frontier"]["count"], 0)
        self.assertTrue(snapshot["frontier"]["actionable"])
        grid = accumulator.grid_for(0)
        occupied = [cell for cell, code in grid.cells().items() if code == om.OCCUPIED]
        self.assertTrue(occupied)
        for cell in occupied:
            self.assertEqual(grid.code_at(*cell), om.OCCUPIED)

    def test_free_mode_none_leaves_everything_unknown_but_occupied(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="none"
        )
        accumulator.add_frame(
            self.cloud,
            frame="map",
            cloud_stamp_s=1.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        snapshot = accumulator.snapshot()
        self.assertGreater(snapshot["occupied_cells"], 0)
        self.assertEqual(snapshot["free_cells"], 0)
        self.assertEqual(snapshot["frontier"]["count"], 0)


class GroundEvidenceTests(unittest.TestCase):
    """Defect 7: a low percentile is a floor *candidate*, not a floor.

    The floor is only used when the same frame and level contain credible
    ground evidence: points strictly below the measured base origin, enough
    support in the free band, a base standoff of at least one obstacle
    clearance, and agreement with the level's established floor. A surface the
    robot does not stand on (desk, shelf, mezzanine, the floor of the level
    above) can never become free floor.
    """

    def _accumulator(self):
        return om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )

    def test_elevated_surface_is_not_floor_and_not_actionable(self):
        # Measured base at 0.3 m; the only surface is an elevated desk/shelf
        # top at 1.0 m and there is no floor return anywhere in the frame.
        accumulator = self._accumulator()
        desk = block(-5, -5, 10, 10, 1.0)
        receipt = accumulator.add_frame(
            desk,
            frame="map",
            cloud_stamp_s=10.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=10.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertFalse(receipt["accepted"])
        self.assertIn(om.REASON_FLOOR_EVIDENCE_MISSING, receipt["reasons"])
        self.assertFalse(receipt["floor_evidence"]["ok"])
        self.assertEqual(
            receipt["floor_evidence"]["reason"], om.REASON_FLOOR_EVIDENCE_MISSING
        )
        self.assertIsNone(receipt["floor_evidence"]["floor_z_m"])
        # Nothing below the measured base origin is evidence; the 1.0 m
        # surface is not a floor candidate at all.
        self.assertEqual(receipt["floor_evidence"]["points_below_base"], 0)

        snapshot = accumulator.snapshot()
        self.assertEqual(snapshot["free_cells"], 0)
        self.assertEqual(snapshot["occupied_cells"], 0)
        self.assertEqual(snapshot["frontier"]["count"], 0)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn(
            om.REASON_FLOOR_EVIDENCE_MISSING, snapshot["frontier"]["reasons"]
        )
        self.assertFalse(snapshot["floor_evidence_ok"])
        self.assertTrue(snapshot["level_stale"])
        self.assertIsNone(snapshot["floor_offset_m"])
        self.assertIsNone(snapshot["level_z_m"])

    def test_visible_floor_patch_behind_an_occluding_desk_still_maps(self):
        # The floor under the desk is occluded, but a real floor patch is
        # visible elsewhere in the same frame: the floor is still credible.
        accumulator = self._accumulator()
        cloud = np.vstack([block(-10, -10, 20, 20, 0.0), block(0, 8, 5, 5, 0.8)])
        receipt = accumulator.add_frame(
            cloud,
            frame="map",
            cloud_stamp_s=10.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=10.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertTrue(receipt["accepted"])
        self.assertNotIn(om.REASON_FLOOR_EVIDENCE_MISSING, receipt["reasons"])
        self.assertTrue(receipt["floor_evidence"]["ok"])
        self.assertAlmostEqual(receipt["floor_evidence"]["floor_z_m"], 0.0, places=9)
        self.assertAlmostEqual(receipt["floor_offset_m"], -0.3, places=9)

        snapshot = accumulator.snapshot()
        self.assertTrue(snapshot["frontier"]["actionable"])
        self.assertEqual(snapshot["frontier"]["reasons"], [])
        self.assertTrue(snapshot["floor_evidence_ok"])
        self.assertAlmostEqual(snapshot["floor_offset_m"], -0.3, places=9)
        self.assertEqual(snapshot["free_cells"], 390)  # floor patch minus desk
        self.assertEqual(snapshot["occupied_cells"], 25)  # the desk top

    def test_mixed_level_points_are_not_floor_evidence_for_each_other(self):
        # A level-0 frame that sees only 20 true-floor points and a dense
        # surface at 2.9 m (the floor of the level above, e.g. through a
        # stairwell opening) must not fund level 0's floor with that surface.
        accumulator = self._accumulator()
        lower = accumulator.add_frame(
            block(-10, -10, 20, 20, 0.0),
            frame="map",
            cloud_stamp_s=1.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        upper = accumulator.add_frame(
            block(40, -10, 20, 20, 2.9),
            frame="map",
            cloud_stamp_s=2.0,
            pose=pose((0.0, 0.0, 3.3)),
            pose_stamp_s=2.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertTrue(lower["accepted"])
        self.assertTrue(upper["accepted"])
        lower_before = accumulator.snapshot(0)
        self.assertEqual(lower_before["free_cells"], 400)

        mixed = np.vstack([block(-10, -10, 4, 5, 0.0), block(20, -10, 20, 20, 2.9)])
        receipt = accumulator.add_frame(
            mixed,
            frame="map",
            cloud_stamp_s=3.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=3.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertFalse(receipt["accepted"])
        self.assertIn(om.REASON_FLOOR_EVIDENCE_MISSING, receipt["reasons"])
        # The sparse true-floor subset is below the support floor, so no free
        # cell is invented and the upper-level points are not used.
        self.assertEqual(receipt["floor_evidence"]["support_points"], 20)

        lower_after = accumulator.snapshot(0)
        self.assertEqual(lower_after["free_cells"], 400)
        self.assertEqual(lower_after["occupied_cells"], 0)
        self.assertAlmostEqual(lower_after["floor_offset_m"], -0.3, places=9)
        self.assertFalse(lower_after["frontier"]["actionable"])
        self.assertIn(
            om.REASON_FLOOR_EVIDENCE_MISSING, lower_after["frontier"]["reasons"]
        )
        # The other level is unaffected, and the refusal reason does not leak
        # into its snapshot.
        upper_after = accumulator.snapshot(1)
        self.assertEqual(upper_after["frontier"]["reasons"], [])
        self.assertTrue(upper_after["frontier"]["actionable"])
        self.assertAlmostEqual(upper_after["floor_offset_m"], -0.4, places=9)
        self.assertEqual(upper_after["free_cells"], 400)

        # A later frame with real floor evidence recovers the level.
        recovered = accumulator.add_frame(
            block(0, -10, 20, 20, 0.0),
            frame="map",
            cloud_stamp_s=4.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=4.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertTrue(recovered["accepted"])
        recovered_snapshot = accumulator.snapshot(0)
        self.assertTrue(recovered_snapshot["frontier"]["actionable"])
        self.assertTrue(recovered_snapshot["floor_evidence_ok"])
        self.assertEqual(recovered_snapshot["free_cells"], 600)

    def test_floor_must_agree_with_the_established_level_floor(self):
        accumulator = self._accumulator()
        established = accumulator.add_frame(
            block(-10, -10, 20, 20, 2.9),
            frame="map",
            cloud_stamp_s=1.0,
            pose=pose((0.0, 0.0, 3.3)),
            pose_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertTrue(established["accepted"])
        self.assertAlmostEqual(established["floor_evidence"]["floor_z_m"], 2.9, places=9)

        # A later frame on the same level only sees a surface 2.1 m below the
        # base, not the established floor: it is not credible ground.
        receipt = accumulator.add_frame(
            block(-10, -10, 20, 20, 1.2),
            frame="map",
            cloud_stamp_s=2.0,
            pose=pose((0.0, 0.0, 3.3)),
            pose_stamp_s=2.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertFalse(receipt["accepted"])
        self.assertIn(om.REASON_FLOOR_EVIDENCE_INCONSISTENT, receipt["reasons"])
        self.assertAlmostEqual(
            receipt["floor_evidence"]["candidate_floor_z_m"], 1.2, places=9
        )
        self.assertAlmostEqual(
            receipt["floor_evidence"]["established_floor_z_m"], 2.9, places=9
        )
        snapshot = accumulator.snapshot()
        self.assertEqual(snapshot["free_cells"], 400)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn(
            om.REASON_FLOOR_EVIDENCE_INCONSISTENT, snapshot["frontier"]["reasons"]
        )
        self.assertAlmostEqual(snapshot["floor_offset_m"], -0.4, places=9)


class SensorRayOriginTests(unittest.TestCase):
    """Defect 8: free-space rays start at the measured sensor origin."""

    def test_ray_carving_uses_measured_sensor_origin_not_base_origin(self):
        # Non-zero sensor->base translation AND non-identity rotation, plus a
        # non-zero measured base yaw so the transform order matters.
        extrinsic = om.Extrinsic(
            translation=(0.5, 0.0, 0.0),
            quaternion_xyzw=om.quaternion_xyzw_from_yaw(math.pi / 2.0),
        )
        measured = pose((0.0, 0.0, 0.3), math.pi / 2.0)
        sensor_in_map = measured @ extrinsic.matrix
        np.testing.assert_allclose(
            sensor_in_map[:3, 3], [0.0, 0.5, 0.3], atol=1e-9
        )
        # The base origin is (0, 0), so the segment base->sensor runs north.
        self.assertAlmostEqual(measured[0, 3], 0.0, places=9)
        self.assertAlmostEqual(measured[1, 3], 0.0, places=9)

        accumulator = om.OccupancyAccumulator(
            extrinsic=extrinsic, resolution_m=RES, free_mode="rays"
        )
        # A real floor patch supplies ground evidence away from the test cells.
        floor = block(0, 20, 20, 20, 0.0)
        # A point lying between the base origin and the sensor origin.
        near_point = np.array([[0.05, 0.25, 0.0]])
        # An obstacle placed on the base-origin->near_point ray, so carving
        # from the base origin would sweep the near point's cell free.
        obstacle = np.array([[0.55, 2.55, 0.5]])
        receipt = accumulator.add_frame(
            np.vstack([floor, near_point, obstacle]),
            frame="map",
            cloud_stamp_s=10.0,
            pose=measured,
            pose_stamp_s=10.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
        )
        self.assertTrue(receipt["accepted"])
        grid = accumulator.grid_for(0)
        near_cell = grid.index_of(0.05, 0.25)
        self.assertEqual(near_cell, (0, 2))
        # Not carved free: the ray starts north of the point, at the sensor.
        self.assertEqual(grid.code_at(*near_cell), om.UNKNOWN)
        # The obstacle itself is occupied and the sensor->obstacle ray is
        # carved free, so carving still works from the correct origin.
        self.assertEqual(grid.code_at(*grid.index_of(0.55, 2.55)), om.OCCUPIED)
        self.assertEqual(grid.code_at(1, 9), om.FREE)


class SnapshotFreshnessTests(unittest.TestCase):
    """Defect 9: every snapshot() re-evaluates staleness from current clocks."""

    def _accumulator(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        receipt = accumulator.add_frame(
            floor_patch(40, 40),
            frame="map",
            cloud_stamp_s=10.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=10.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
            arrival_wall_s=100.0,
        )
        self.assertTrue(receipt["accepted"])
        return accumulator

    def test_wall_clock_past_the_limit_turns_the_snapshot_stale(self):
        accumulator = self._accumulator()
        fresh = accumulator.snapshot(now_sim_s=10.0, now_wall_s=100.2)
        self.assertTrue(fresh["frontier"]["actionable"])
        self.assertFalse(fresh["snapshot_freshness"]["stale"])
        stale = accumulator.snapshot(now_sim_s=10.0, now_wall_s=100.6)
        self.assertFalse(stale["frontier"]["actionable"])
        self.assertIn("stale_pose_wall_age", stale["frontier"]["reasons"])
        self.assertFalse(stale["gates"]["pose_fresh"])
        self.assertTrue(stale["level_stale"])
        self.assertTrue(stale["snapshot_freshness"]["stale"])
        self.assertAlmostEqual(stale["pose_ages"]["wall_s"], 0.6, places=9)

    def test_sim_clock_stopped_still_goes_stale_on_the_wall_clock(self):
        accumulator = self._accumulator()
        fresh = accumulator.snapshot(now_sim_s=10.0, now_wall_s=100.1)
        self.assertTrue(fresh["frontier"]["actionable"])
        # The sim clock does not advance but the arrival clock does.
        stopped = accumulator.snapshot(now_sim_s=10.0, now_wall_s=101.2)
        self.assertFalse(stopped["frontier"]["actionable"])
        self.assertIn("stale_pose_wall_age", stopped["frontier"]["reasons"])
        self.assertEqual(stopped["snapshot_freshness"]["now_sim_s"], 10.0)
        self.assertAlmostEqual(
            stopped["snapshot_freshness"]["pose_sim_age_s"], 0.0, places=9
        )

    def test_sim_clock_rolled_back_is_stale_with_an_explicit_reason(self):
        accumulator = self._accumulator()
        fresh = accumulator.snapshot(now_sim_s=10.0, now_wall_s=100.1)
        self.assertTrue(fresh["frontier"]["actionable"])
        rolled_back = accumulator.snapshot(now_sim_s=9.0, now_wall_s=100.2)
        self.assertFalse(rolled_back["frontier"]["actionable"])
        self.assertIn(
            om.REASON_SIM_CLOCK_REGRESSION, rolled_back["frontier"]["reasons"]
        )
        self.assertAlmostEqual(
            rolled_back["snapshot_freshness"]["pose_sim_age_s"], -1.0, places=9
        )

    def test_two_now_values_differ_and_an_old_now_is_not_reused(self):
        accumulator = self._accumulator()
        first = accumulator.snapshot(now_sim_s=10.0, now_wall_s=100.1)
        second = accumulator.snapshot(now_sim_s=10.0, now_wall_s=100.4)
        third = accumulator.snapshot(now_sim_s=10.0, now_wall_s=100.9)
        self.assertTrue(first["frontier"]["actionable"])
        self.assertTrue(second["frontier"]["actionable"])
        self.assertFalse(third["frontier"]["actionable"])
        self.assertAlmostEqual(first["pose_ages"]["wall_s"], 0.1, places=9)
        self.assertAlmostEqual(second["pose_ages"]["wall_s"], 0.4, places=9)
        self.assertNotEqual(first["pose_ages"]["wall_s"], second["pose_ages"]["wall_s"])
        # Repeating an old now must not resurrect the old fresh verdict; the
        # caller's now is still the one reported, not a cached later now.
        again = accumulator.snapshot(now_sim_s=10.0, now_wall_s=100.1)
        self.assertFalse(again["frontier"]["actionable"])
        self.assertIn(
            om.REASON_WALL_CLOCK_REGRESSION, again["frontier"]["reasons"]
        )
        self.assertAlmostEqual(
            again["snapshot_freshness"]["now_wall_s"], 100.1, places=9
        )

    def test_stalled_pose_stream_while_the_cloud_stream_continues(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        accumulator.add_pose(1.0, pose((0.0, 0.0, 0.3)), arrival_wall_s=100.0)
        first = accumulator.add_frame(
            floor_patch(40, 40),
            frame="map",
            cloud_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
            arrival_wall_s=100.0,
        )
        self.assertTrue(first["accepted"])
        # Clouds keep arriving, but the pose stream is stuck at stamp 1.0.
        second = accumulator.add_frame(
            floor_patch(40, 40),
            frame="map",
            cloud_stamp_s=2.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
            arrival_wall_s=101.0,
        )
        self.assertTrue(second["accepted"])
        snapshot = accumulator.snapshot(now_sim_s=2.0, now_wall_s=101.1)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("stale_pose_sim_age", snapshot["frontier"]["reasons"])
        self.assertAlmostEqual(
            snapshot["snapshot_freshness"]["pose_sim_age_s"], 1.0, places=9
        )
        self.assertAlmostEqual(
            snapshot["snapshot_freshness"]["cloud_sim_age_s"], 0.0, places=9
        )

    def test_stalled_cloud_stream_while_the_pose_stream_continues(self):
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        accumulator.add_pose(1.0, pose((0.0, 0.0, 0.3)), arrival_wall_s=100.0)
        first = accumulator.add_frame(
            floor_patch(40, 40),
            frame="map",
            cloud_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
            arrival_wall_s=100.0,
        )
        self.assertTrue(first["accepted"])
        # Poses keep arriving at stamp 2.0 while the cloud stream is stalled.
        accumulator.add_pose(2.0, pose((0.0, 0.0, 0.3)), arrival_wall_s=100.5)
        snapshot = accumulator.snapshot(now_sim_s=2.0, now_wall_s=100.6)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn(om.REASON_STALE_CLOUD_SIM_AGE, snapshot["frontier"]["reasons"])
        self.assertAlmostEqual(
            snapshot["snapshot_freshness"]["cloud_sim_age_s"], 1.0, places=9
        )
        self.assertAlmostEqual(
            snapshot["snapshot_freshness"]["pose_sim_age_s"], 0.0, places=9
        )

    def test_no_caller_now_still_reevaluates_the_recorded_arrival(self):
        # A frame ingested as already 1.0 s old must not keep a fresh verdict
        # just because snapshot() is called without an explicit now.
        accumulator = om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )
        receipt = accumulator.add_frame(
            floor_patch(40, 40),
            frame="map",
            cloud_stamp_s=10.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=10.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
            arrival_wall_s=time.monotonic() - 1.0,
        )
        self.assertTrue(receipt["accepted"])
        snapshot = accumulator.snapshot()
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("stale_pose_wall_age", snapshot["frontier"]["reasons"])
        self.assertGreaterEqual(snapshot["pose_ages"]["wall_s"], 0.5)



class ClockAuthorityStallTests(unittest.TestCase):
    """Every snapshot re-evaluates both streams, including the no-now path.

    The production CLI calls accumulator.snapshot(level_index,
    frontier_limit=...) with no now_sim_s / now_wall_s. These tests use exactly
    that call shape and let the recorded time.monotonic() arrival decide
    staleness; passing a test-only now is not what is covered.
    """

    def _accumulator(self):
        return om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="both"
        )

    def _accept_floor(
        self, accumulator, *, cloud_stamp_s, arrival_wall_s, sim_age_s=0.05
    ):
        return accumulator.add_frame(
            floor_patch(40, 40),
            frame="map",
            cloud_stamp_s=cloud_stamp_s,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=cloud_stamp_s,
            sim_age_s=sim_age_s,
            wall_age_s=0.10,
            arrival_wall_s=arrival_wall_s,
        )

    def test_cloud_stall_while_pose_continues_without_caller_now(self):
        accumulator = self._accumulator()
        now = time.monotonic()
        receipt = self._accept_floor(
            accumulator, cloud_stamp_s=1.0, arrival_wall_s=now - 0.6
        )
        self.assertTrue(receipt["accepted"])
        # The pose stream keeps arriving while the cloud stream is frozen.
        accumulator.add_pose(1.5, pose((0.0, 0.0, 0.3)), arrival_wall_s=now - 0.3)
        accumulator.add_pose(2.0, pose((0.0, 0.0, 0.3)), arrival_wall_s=now)
        snapshot = accumulator.snapshot(0, frontier_limit=10)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn(
            om.REASON_STALE_CLOUD_WALL_AGE, snapshot["frontier"]["reasons"]
        )
        # The cloud stream was actually re-evaluated instead of skipped.
        self.assertTrue(snapshot["snapshot_freshness"]["cloud_age_evaluated"])
        self.assertTrue(snapshot["snapshot_freshness"]["cloud_age_complete"])
        self.assertGreaterEqual(
            snapshot["snapshot_freshness"]["cloud_wall_age_s"], 0.5
        )
        self.assertLess(snapshot["snapshot_freshness"]["pose_wall_age_s"], 0.5)
        # The pose stream is fresh; a fresh pose must not stand in for the cloud.
        self.assertTrue(snapshot["gates"]["pairing"])
        self.assertTrue(snapshot["latest_frame_accepted"])
        self.assertEqual(snapshot["frontier"]["returned_cells"], 10)

    def test_pose_stall_while_cloud_continues_without_caller_now(self):
        accumulator = self._accumulator()
        now = time.monotonic()
        accumulator.add_pose(1.0, pose((0.0, 0.0, 0.3)), arrival_wall_s=now - 0.6)
        receipt = accumulator.add_frame(
            floor_patch(40, 40),
            frame="map",
            cloud_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
            arrival_wall_s=now,
        )
        self.assertTrue(receipt["accepted"])
        snapshot = accumulator.snapshot(0, frontier_limit=10)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("stale_pose_wall_age", snapshot["frontier"]["reasons"])
        self.assertGreaterEqual(
            snapshot["snapshot_freshness"]["pose_wall_age_s"], 0.5
        )
        # The cloud itself is fresh; only the pose stream is stale.
        self.assertLess(snapshot["snapshot_freshness"]["cloud_wall_age_s"], 0.5)

    def test_both_streams_stopped_without_caller_now(self):
        accumulator = self._accumulator()
        now = time.monotonic()
        accumulator.add_pose(1.0, pose((0.0, 0.0, 0.3)), arrival_wall_s=now - 0.6)
        self._accept_floor(accumulator, cloud_stamp_s=1.0, arrival_wall_s=now - 0.6)
        snapshot = accumulator.snapshot(0, frontier_limit=10)
        self.assertFalse(snapshot["frontier"]["actionable"])
        reasons = snapshot["frontier"]["reasons"]
        self.assertIn("stale_pose_wall_age", reasons)
        self.assertIn(om.REASON_STALE_CLOUD_WALL_AGE, reasons)

    def test_both_streams_continue_without_caller_now_stays_actionable(self):
        accumulator = self._accumulator()
        now = time.monotonic()
        accumulator.add_pose(1.0, pose((0.0, 0.0, 0.3)), arrival_wall_s=now)
        self._accept_floor(accumulator, cloud_stamp_s=1.0, arrival_wall_s=now)
        snapshot = accumulator.snapshot(0, frontier_limit=10)
        self.assertTrue(snapshot["frontier"]["actionable"])
        self.assertEqual(snapshot["frontier"]["reasons"], [])
        self.assertTrue(snapshot["snapshot_freshness"]["cloud_age_evaluated"])
        self.assertTrue(snapshot["snapshot_freshness"]["cloud_age_complete"])

    def test_repeated_snapshots_with_increasing_wall_time_keep_it_stale(self):
        accumulator = self._accumulator()
        arrival = time.monotonic()
        receipt = self._accept_floor(
            accumulator, cloud_stamp_s=10.0, arrival_wall_s=arrival
        )
        self.assertTrue(receipt["accepted"])
        fresh = accumulator.snapshot(0, now_sim_s=10.0, now_wall_s=arrival + 0.1)
        self.assertTrue(fresh["frontier"]["actionable"])
        first_stale = accumulator.snapshot(0, now_sim_s=10.0, now_wall_s=arrival + 0.6)
        second_stale = accumulator.snapshot(
            0, now_sim_s=10.0, now_wall_s=arrival + 1.2
        )
        for snapshot in (first_stale, second_stale):
            self.assertFalse(snapshot["frontier"]["actionable"])
            self.assertIn(
                om.REASON_STALE_CLOUD_WALL_AGE, snapshot["frontier"]["reasons"]
            )
        self.assertGreater(
            second_stale["snapshot_freshness"]["cloud_wall_age_s"],
            first_stale["snapshot_freshness"]["cloud_wall_age_s"],
        )

    def test_cli_call_shape_without_now_arguments(self):
        # Exactly the production call shape: positional level plus
        # frontier_limit, no now_sim_s and no now_wall_s.
        accumulator = self._accumulator()
        now = time.monotonic()
        receipt = self._accept_floor(
            accumulator, cloud_stamp_s=10.0, arrival_wall_s=now
        )
        self.assertTrue(receipt["accepted"])
        snapshot = accumulator.snapshot(0, frontier_limit=3)
        self.assertEqual(
            snapshot["snapshot_freshness"]["now_source"], "recorded_arrival"
        )
        self.assertIsNone(snapshot["snapshot_freshness"]["now_wall_s"])
        self.assertTrue(snapshot["snapshot_freshness"]["cloud_age_evaluated"])
        self.assertTrue(snapshot["frontier"]["actionable"])
        self.assertEqual(snapshot["frontier"]["returned_cells"], 3)


class NonFiniteClockTests(unittest.TestCase):
    """NaN/Inf ages and nows never pass a comparison or get clipped to zero."""

    def _accumulator(self):
        return om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )

    def _frame(self, accumulator, **overrides):
        options = {
            "frame": "map",
            "cloud_stamp_s": 10.0,
            "pose": pose((0.0, 0.0, 0.3)),
            "pose_stamp_s": 10.0,
            "sim_age_s": 0.05,
            "wall_age_s": 0.10,
        }
        options.update(overrides)
        return accumulator.add_frame(floor_patch(40, 40), **options)

    def test_non_finite_sim_age_never_passes(self):
        accumulator = self._accumulator()
        receipt = self._frame(accumulator, sim_age_s=float("nan"))
        self.assertFalse(receipt["gates"]["pose_fresh"])
        self.assertIn(om.REASON_POSE_AGE_INVALID, receipt["reasons"])
        snapshot = accumulator.snapshot(0, frontier_limit=5)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("pose_age_unknown", snapshot["frontier"]["reasons"])
        self.assertIsNone(snapshot["pose_ages"]["sim_s"])

    def test_non_finite_wall_age_is_not_clipped_to_zero(self):
        accumulator = self._accumulator()
        receipt = self._frame(accumulator, wall_age_s=float("nan"))
        self.assertFalse(receipt["gates"]["pose_fresh"])
        snapshot = accumulator.snapshot(0, frontier_limit=5)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn(om.REASON_CLOUD_AGE_UNEVALUATED, snapshot["frontier"]["reasons"])
        self.assertIsNone(snapshot["snapshot_freshness"]["cloud_wall_age_s"])

    def test_non_finite_now_never_yields_actionable(self):
        accumulator = self._accumulator()
        self._frame(accumulator)
        for kwargs in (
            {"now_sim_s": float("nan")},
            {"now_wall_s": float("inf")},
            {"now_sim_s": float("inf"), "now_wall_s": float("nan")},
        ):
            snapshot = accumulator.snapshot(0, **kwargs)
            self.assertFalse(snapshot["frontier"]["actionable"])
            self.assertIn(om.REASON_CLOCK_INVALID, snapshot["frontier"]["reasons"])
            self.assertIn(
                om.REASON_CLOUD_AGE_UNEVALUATED, snapshot["frontier"]["reasons"]
            )

    def test_missing_cloud_arrival_is_unevaluated(self):
        accumulator = self._accumulator()
        self._frame(accumulator, arrival_wall_s=float("nan"))
        snapshot = accumulator.snapshot(0, frontier_limit=5)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn(
            om.REASON_CLOUD_AGE_UNEVALUATED, snapshot["frontier"]["reasons"]
        )
        self.assertFalse(snapshot["snapshot_freshness"]["cloud_age_complete"])
        self.assertEqual(snapshot["free_cells"], 1600)

    def test_invalid_pose_arrival_never_yields_actionable(self):
        accumulator = self._accumulator()
        now = time.monotonic()
        accumulator.add_pose(1.0, pose((0.0, 0.0, 0.3)), arrival_wall_s=float("nan"))
        receipt = accumulator.add_frame(
            floor_patch(40, 40),
            frame="map",
            cloud_stamp_s=1.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
            arrival_wall_s=now,
        )
        self.assertTrue(receipt["accepted"])
        snapshot = accumulator.snapshot(0, frontier_limit=5)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn("pose_age_unknown", snapshot["frontier"]["reasons"])


class LowSurfaceFloorEvidenceTests(unittest.TestCase):
    """A low obstacle surface is not a floor without independent evidence.

    The old rule ("below the base by one clearance plus 50 points") accepted a
    kerb, step edge, pallet or low panel at 0.15 m below the measured base and
    marked it free. These cases lock the fail-closed evidence routes.
    """

    def _accumulator(self):
        return om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )

    def _add(self, accumulator, cloud, **overrides):
        options = {
            "frame": "map",
            "cloud_stamp_s": 10.0,
            "pose": pose((0.0, 0.0, 0.3)),
            "pose_stamp_s": 10.0,
            "sim_age_s": 0.05,
            "wall_age_s": 0.10,
        }
        options.update(overrides)
        return accumulator.add_frame(cloud, **options)

    def test_low_surface_015_below_base_is_refused(self):
        accumulator = self._accumulator()
        # Base at 0.30 m, the only observed surface is a 1 m x 1 m plate at
        # 0.15 m (a kerb / step edge / low panel). The true floor is occluded.
        receipt = self._add(accumulator, block(-10, -10, 20, 20, 0.15))
        self.assertFalse(receipt["accepted"])
        self.assertEqual(
            receipt["floor_evidence"]["reason"],
            om.REASON_FLOOR_EVIDENCE_UNCONFIRMED,
        )
        self.assertIsNone(receipt["floor_evidence"]["floor_z_m"])
        self.assertEqual(receipt["floor_evidence"]["support_points"], 400)
        self.assertLess(
            receipt["floor_evidence"]["standoff_m"],
            om.MIN_BODY_SUPPORT_STANDOFF_M,
        )
        # A 20 x 20 plate is laterally extended; the missing evidence is the
        # measured body standoff, not the extent.
        self.assertGreaterEqual(
            receipt["floor_evidence"]["support_span_x_m"],
            om.MIN_GROUND_SUPPORT_SPAN_M,
        )
        snapshot = accumulator.snapshot(0)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertIn(
            om.REASON_FLOOR_EVIDENCE_UNCONFIRMED, snapshot["frontier"]["reasons"]
        )
        self.assertEqual(snapshot["free_cells"], 0)
        self.assertEqual(snapshot["occupied_cells"], 0)

    def test_low_surface_with_visible_real_floor_patch_maps(self):
        accumulator = self._accumulator()
        # Same low plate is now seen together with a real floor patch. The low
        # quantile picks the true floor, the measured standoff is 0.30 m and the
        # floor patch is laterally extended, so the frame maps normally.
        cloud = np.vstack(
            [block(-20, -10, 20, 20, 0.0), block(0, -10, 20, 20, 0.15)]
        )
        receipt = self._add(accumulator, cloud)
        self.assertTrue(receipt["accepted"])
        self.assertAlmostEqual(receipt["floor_evidence"]["floor_z_m"], 0.0, places=9)
        self.assertEqual(
            receipt["floor_evidence"]["evidence_route"],
            "measured_standoff_and_extent",
        )
        snapshot = accumulator.snapshot(0)
        self.assertTrue(snapshot["frontier"]["actionable"])
        self.assertEqual(snapshot["frontier"]["reasons"], [])
        self.assertEqual(snapshot["free_cells"], 400)
        # The 0.15 m plate is one clearance above the floor: an obstacle.
        self.assertEqual(snapshot["occupied_cells"], 400)

    def test_kerb_strip_is_refused(self):
        accumulator = self._accumulator()
        # A long narrow kerb strip 0.15 m below the base: 60 points, 0.2 m by
        # 1.9 m. Both the standoff and the lateral extent evidence are missing.
        receipt = self._add(accumulator, block(-10, 0, 3, 20, 0.15))
        self.assertFalse(receipt["accepted"])
        self.assertEqual(
            receipt["floor_evidence"]["reason"],
            om.REASON_FLOOR_EVIDENCE_UNCONFIRMED,
        )
        self.assertEqual(receipt["floor_evidence"]["support_points"], 60)
        self.assertLess(
            receipt["floor_evidence"]["support_span_x_m"],
            om.MIN_GROUND_SUPPORT_SPAN_M,
        )
        snapshot = accumulator.snapshot(0)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertEqual(snapshot["free_cells"], 0)

    def test_stair_step_true_floor_maps_step_as_obstacle(self):
        accumulator = self._accumulator()
        # A real 20 x 20 floor patch plus a 0.15 m high step strip: the floor is
        # credible and the step top is an obstacle, not free floor.
        cloud = np.vstack(
            [block(-10, -10, 20, 20, 0.0), block(-10, 0, 3, 20, 0.15)]
        )
        receipt = self._add(accumulator, cloud)
        self.assertTrue(receipt["accepted"])
        self.assertAlmostEqual(receipt["floor_evidence"]["floor_z_m"], 0.0, places=9)
        snapshot = accumulator.snapshot(0)
        self.assertTrue(snapshot["frontier"]["actionable"])
        self.assertAlmostEqual(snapshot["floor_offset_m"], -0.3, places=9)
        # 30 of the 400 floor cells are under the step strip and become
        # occupied; the step itself covers 60 cells in total.
        self.assertEqual(snapshot["free_cells"], 370)
        self.assertEqual(snapshot["occupied_cells"], 60)
        grid = accumulator.grid_for(0)
        self.assertEqual(grid.code_at(*grid.index_of(-0.85, 0.55)), om.OCCUPIED)
        self.assertEqual(grid.code_at(*grid.index_of(0.05, 0.05)), om.FREE)

    def test_multi_level_mixed_points_do_not_fund_a_floor(self):
        accumulator = self._accumulator()
        # Level 0's base at 0.30 m sees only a 0.15 m low plate and the 2.9 m
        # floor of the level above through the stairwell opening. Neither is
        # credible ground for this level.
        mixed = np.vstack(
            [block(-10, -10, 20, 20, 0.15), block(20, -10, 20, 20, 2.9)]
        )
        receipt = self._add(accumulator, mixed)
        self.assertFalse(receipt["accepted"])
        self.assertEqual(
            receipt["floor_evidence"]["reason"],
            om.REASON_FLOOR_EVIDENCE_UNCONFIRMED,
        )
        # Only the sub-base plate is a candidate; the upper level is not.
        self.assertEqual(receipt["floor_evidence"]["points_below_base"], 400)
        snapshot = accumulator.snapshot(0)
        self.assertFalse(snapshot["frontier"]["actionable"])
        self.assertEqual(snapshot["free_cells"], 0)
        # The same frame with a real floor patch visible does map.
        recovered = np.vstack(
            [
                block(-30, -10, 10, 20, 0.0),
                block(-10, -10, 20, 20, 0.15),
                block(20, -10, 20, 20, 2.9),
            ]
        )
        receipt = self._add(
            accumulator, recovered, cloud_stamp_s=11.0, pose_stamp_s=11.0
        )
        self.assertTrue(receipt["accepted"])
        self.assertAlmostEqual(receipt["floor_evidence"]["floor_z_m"], 0.0, places=9)
        snapshot = accumulator.snapshot(0)
        self.assertTrue(snapshot["frontier"]["actionable"])
        self.assertEqual(snapshot["free_cells"], 200)

    def test_measured_stand_proof_is_the_documented_route(self):
        accumulator = self._accumulator()
        receipt = self._add(
            accumulator, block(-10, -10, 20, 20, 0.15), stand_proof_z_m=0.15
        )
        self.assertTrue(receipt["accepted"])
        self.assertAlmostEqual(receipt["floor_evidence"]["floor_z_m"], 0.15, places=9)
        self.assertEqual(
            receipt["floor_evidence"]["evidence_route"], "measured_stand_proof"
        )
        snapshot = accumulator.snapshot(0)
        self.assertTrue(snapshot["frontier"]["actionable"])
        self.assertAlmostEqual(snapshot["floor_offset_m"], -0.15, places=9)

    def test_stand_proof_mismatch_is_refused(self):
        accumulator = self._accumulator()
        receipt = self._add(
            accumulator, block(-10, -10, 20, 20, 0.18), stand_proof_z_m=0.0
        )
        self.assertFalse(receipt["accepted"])
        self.assertEqual(
            receipt["floor_evidence"]["reason"],
            om.REASON_FLOOR_EVIDENCE_UNCONFIRMED,
        )
        snapshot = accumulator.snapshot(0)
        self.assertFalse(snapshot["frontier"]["actionable"])

    def test_established_floor_rejects_surface_outside_its_free_band(self):
        accumulator = self._accumulator()
        first = self._add(
            accumulator,
            block(-10, -10, 20, 20, 0.0),
            cloud_stamp_s=1.0,
            pose_stamp_s=1.0,
        )
        self.assertTrue(first["accepted"])
        # The old rule accepted a candidate 0.15 m below the base as long as it
        # agreed with the established floor within 0.30 m. This surface does
        # agree with the established 0.0 m floor and is one clearance below the
        # base, but its support points are not inside the floor's free band
        # (the band ends at +0.15 m): a low surface above the real floor.
        receipt = self._add(
            accumulator,
            block(-10, -10, 20, 20, 0.15),
            cloud_stamp_s=2.0,
            pose_stamp_s=2.0,
        )
        self.assertFalse(receipt["accepted"])
        self.assertEqual(
            receipt["floor_evidence"]["reason"],
            om.REASON_FLOOR_EVIDENCE_UNCONFIRMED,
        )
        self.assertEqual(
            receipt["floor_evidence"]["support_points_in_established_free_band"], 0
        )
        snapshot = accumulator.snapshot(0)
        self.assertFalse(snapshot["frontier"]["actionable"])
        # The previously observed floor geometry is retained.
        self.assertEqual(snapshot["free_cells"], 400)


class RefusedFrameAndPerLevelFreshnessTests(unittest.TestCase):
    """A refused latest frame and a stale level are separate from map geometry."""

    def _accumulator(self):
        return om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )

    def _accept(self, accumulator, *, z, cloud_stamp_s, arrival_wall_s):
        return accumulator.add_frame(
            block(-20, -20, 20, 20, z - 0.3),
            frame="map",
            cloud_stamp_s=cloud_stamp_s,
            pose=pose((0.0, 0.0, z)),
            pose_stamp_s=cloud_stamp_s,
            sim_age_s=0.05,
            wall_age_s=0.10,
            arrival_wall_s=arrival_wall_s,
        )

    def test_refused_frame_does_not_keep_old_actionable(self):
        accumulator = self._accumulator()
        now = time.monotonic()
        first = self._accept(
            accumulator, z=0.3, cloud_stamp_s=10.0, arrival_wall_s=now
        )
        self.assertTrue(first["accepted"])
        fresh = accumulator.snapshot(0)
        self.assertTrue(fresh["frontier"]["actionable"])
        # A new cloud arrives but carries no points: the frame is refused.
        refused = accumulator.add_frame(
            np.zeros((0, 3)),
            frame="map",
            cloud_stamp_s=11.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=11.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
            arrival_wall_s=now,
        )
        self.assertFalse(refused["accepted"])
        self.assertIn("no_points", refused["reasons"])
        immediate = accumulator.snapshot(0)
        self.assertFalse(immediate["frontier"]["actionable"])
        self.assertFalse(immediate["latest_frame_accepted"])
        self.assertIn("no_points", immediate["frontier"]["reasons"])
        self.assertIn(
            om.REASON_LATEST_FRAME_REFUSED, immediate["frontier"]["reasons"]
        )
        # Geometry is retained even though the level is not currently actionable.
        self.assertEqual(immediate["free_cells"], 400)
        # Past the freshness window it is still non-actionable.
        beyond = accumulator.snapshot(0, now_sim_s=11.0, now_wall_s=now + 1.0)
        self.assertFalse(beyond["frontier"]["actionable"])
        self.assertFalse(beyond["latest_frame_accepted"])

    def test_other_level_new_data_does_not_refresh_stale_level(self):
        accumulator = self._accumulator()
        now = time.monotonic()
        # Level 0's only observation is 1.0 s old; level 1 is observed now and
        # the pose stream is fresh. Level 0 must still be stale.
        receipt_a = self._accept(
            accumulator, z=0.3, cloud_stamp_s=1.0, arrival_wall_s=now - 1.0
        )
        receipt_b = self._accept(
            accumulator, z=3.3, cloud_stamp_s=2.0, arrival_wall_s=now
        )
        self.assertTrue(receipt_a["accepted"])
        self.assertTrue(receipt_b["accepted"])
        level_a = accumulator.snapshot(0)
        level_b = accumulator.snapshot(1)
        self.assertFalse(level_a["frontier"]["actionable"])
        self.assertIn(
            om.REASON_STALE_CLOUD_WALL_AGE, level_a["frontier"]["reasons"]
        )
        self.assertGreaterEqual(
            level_a["snapshot_freshness"]["cloud_wall_age_s"], 0.5
        )
        self.assertEqual(level_a["snapshot_freshness"]["target_level"], 0)
        self.assertTrue(level_b["frontier"]["actionable"])
        self.assertEqual(level_b["frontier"]["reasons"], [])
        self.assertLess(level_b["snapshot_freshness"]["cloud_wall_age_s"], 0.5)
        # Repeated snapshots with increasing wall time stay stale for level 0.
        later = accumulator.snapshot(0, now_sim_s=2.0, now_wall_s=now + 0.5)
        self.assertFalse(later["frontier"]["actionable"])
        self.assertIn(
            om.REASON_STALE_CLOUD_WALL_AGE, later["frontier"]["reasons"]
        )
        self.assertGreater(
            later["snapshot_freshness"]["cloud_wall_age_s"],
            level_a["snapshot_freshness"]["cloud_wall_age_s"],
        )

    def test_refusal_on_one_level_does_not_block_another_level(self):
        accumulator = self._accumulator()
        now = time.monotonic()
        self.assertTrue(
            self._accept(accumulator, z=0.3, cloud_stamp_s=1.0, arrival_wall_s=now)[
                "accepted"
            ]
        )
        self.assertTrue(
            self._accept(accumulator, z=3.3, cloud_stamp_s=2.0, arrival_wall_s=now)[
                "accepted"
            ]
        )
        # An empty frame resolves to level 0 and is refused there.
        accumulator.add_frame(
            np.zeros((0, 3)),
            frame="map",
            cloud_stamp_s=3.0,
            pose=pose((0.0, 0.0, 0.3)),
            pose_stamp_s=3.0,
            sim_age_s=0.05,
            wall_age_s=0.10,
            arrival_wall_s=now,
        )
        level_zero = accumulator.snapshot(0)
        level_one = accumulator.snapshot(1)
        self.assertFalse(level_zero["frontier"]["actionable"])
        self.assertFalse(level_zero["latest_frame_accepted"])
        self.assertTrue(level_one["frontier"]["actionable"])
        self.assertEqual(level_one["frontier"]["reasons"], [])
        self.assertTrue(level_one["latest_frame_accepted"])





class FloorAnchorDriftTests(unittest.TestCase):
    """The classification floor is anchored once a level floor is established."""

    def _accumulator(self):
        return om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )

    def _accept(self, accumulator, points, *, stamp, z=0.4, x=0.0, y=0.0):
        return accumulator.add_frame(
            points,
            frame="map",
            cloud_stamp_s=stamp,
            pose=pose((x, y, z)),
            pose_stamp_s=stamp,
            sim_age_s=0.05,
            wall_age_s=0.10,
            arrival_wall_s=time.monotonic(),
        )

    def _two_frames(self, candidate_z=0.11, obstacle_z=0.20):
        accumulator = self._accumulator()
        first = self._accept(accumulator, block(-20, -20, 40, 40, 0.0), stamp=10.0)
        self.assertTrue(first["accepted"])
        self.assertEqual("measured_standoff_and_extent", first["floor_evidence"]["evidence_route"])
        points = np.vstack(
            [
                block(-20, -20, 40, 40, candidate_z),
                block(80, 0, 1, 1, obstacle_z),
            ]
        )
        second = self._accept(accumulator, points, stamp=10.1)
        return accumulator, first, second

    def test_the_classification_floor_stays_on_the_established_anchor(self):
        accumulator, first, second = self._two_frames()
        self.assertTrue(second["accepted"])
        self.assertEqual("established_floor", second["floor_evidence"]["evidence_route"])
        self.assertAlmostEqual(0.0, second["floor_evidence"]["established_floor_z_m"])
        self.assertAlmostEqual(0.0, second["floor_evidence"]["floor_z_m"])
        self.assertAlmostEqual(0.11, second["floor_evidence"]["candidate_floor_z_m"])
        grid = accumulator.grid_for(0)
        self.assertEqual(om.OCCUPIED, grid.code_at(80, 0))
        snapshot = accumulator.snapshot(0, frontier_limit=5)
        self.assertAlmostEqual(0.0, snapshot["floor_z_m"])
        self.assertEqual("established_level_anchor", snapshot["floor_z_source"])
        self.assertIn("observed_ground_quantile_m", snapshot)

    def test_the_accepted_receipt_carries_pairing_and_floor_facts(self):
        accumulator, first, second = self._two_frames()
        receipt = second
        self.assertAlmostEqual(0.0, receipt["classification_floor_z_m"])
        self.assertAlmostEqual(0.0, receipt["level_floor_anchor_m"])
        self.assertAlmostEqual(10.1, receipt["pose_stamp_s"])
        self.assertAlmostEqual(0.0, receipt["paired_dt_s"])
        self.assertIn("floor_evidence", receipt)
        self.assertEqual("established_floor", receipt["floor_evidence"]["evidence_route"])

    def test_a_candidate_shift_cannot_move_the_free_band(self):
        for candidate_z in (0.11, -0.11):
            accumulator, _, second = self._two_frames(
                candidate_z=candidate_z, obstacle_z=0.20
            )
            self.assertAlmostEqual(0.0, second["floor_evidence"]["floor_z_m"])
            grid = accumulator.grid_for(0)
            self.assertEqual(om.OCCUPIED, grid.code_at(80, 0))
            self.assertAlmostEqual(
                0.0, accumulator.snapshot(0, frontier_limit=1)["floor_z_m"]
            )

    def test_the_obstacle_clearance_boundary_is_relative_to_the_anchor(self):
        for z, expected in ((0.14, om.FREE), (0.15, om.OCCUPIED)):
            accumulator, _, second = self._two_frames(candidate_z=0.0, obstacle_z=z)
            self.assertTrue(second["accepted"])
            self.assertEqual(second["floor_evidence"]["floor_z_m"], 0.0)
            self.assertEqual(expected, accumulator.grid_for(0).code_at(80, 0))

    def test_a_first_floor_still_establishes_and_a_narrow_ledge_is_refused(self):
        accumulator = self._accumulator()
        receipt = self._accept(accumulator, block(-20, -20, 40, 40, 0.0), stamp=10.0)
        self.assertEqual("measured_standoff_and_extent", receipt["floor_evidence"]["evidence_route"])
        self.assertEqual(0, accumulator._level_floor_z[0])
        narrow = self._accumulator()
        receipt = self._accept(narrow, block(-2, -20, 4, 40, 0.0), stamp=10.0)
        self.assertFalse(receipt["accepted"])
        self.assertEqual(
            om.REASON_FLOOR_EVIDENCE_UNCONFIRMED, receipt["reasons"][0]
        )
        self.assertEqual({}, narrow._level_floor_z)
        self.assertIsNone(narrow.snapshot(0, frontier_limit=1)["floor_offset_m"])

    def test_each_level_keeps_its_own_anchor(self):
        accumulator = self._accumulator()
        low = self._accept(accumulator, block(-20, -20, 40, 40, 0.0), stamp=10.0, z=0.4)
        high = self._accept(accumulator, block(-20, -20, 40, 40, 3.0), stamp=10.5, z=3.4)
        self.assertTrue(low["accepted"] and high["accepted"])
        self.assertAlmostEqual(0.0, accumulator._level_floor_z[0])
        self.assertAlmostEqual(3.0, accumulator._level_floor_z[1])
        for candidate_z in (0.11, 3.11):
            level = 0 if candidate_z < 1.0 else 1
            z = 0.4 if level == 0 else 3.4
            receipt = self._accept(
                accumulator,
                np.vstack([block(-20, -20, 40, 40, candidate_z), block(80, 0, 1, 1, candidate_z + 0.20)]),
                stamp=11.0 + candidate_z,
                z=z,
            )
            self.assertEqual(level, receipt["level_index"])
            self.assertAlmostEqual(float(level) * 3.0, receipt["floor_evidence"]["floor_z_m"])
            self.assertEqual(om.OCCUPIED, accumulator.grid_for(level).code_at(80, 0))

    def test_an_empty_accumulator_has_no_fabricated_floor(self):
        snapshot = self._accumulator().snapshot(0, frontier_limit=1)
        self.assertIsNone(snapshot["floor_z_m"])
        self.assertIsNone(snapshot["floor_offset_m"])
        self.assertIsNone(snapshot["floor_z_source"])





class MergeAnchorConsistencyTests(unittest.TestCase):
    """Grid merges are refused unless both sides share the level floor anchor."""

    def _accumulator(self):
        return om.OccupancyAccumulator(
            extrinsic=IDENTITY_EXTRINSIC, resolution_m=RES, free_mode="floor"
        )

    def _accept(self, accumulator, points, *, stamp, z=0.4, x=0.0):
        return accumulator.add_frame(
            points,
            frame="map",
            cloud_stamp_s=stamp,
            pose=pose((x, 0.0, z)),
            pose_stamp_s=stamp,
            sim_age_s=0.05,
            wall_age_s=0.10,
            arrival_wall_s=time.monotonic(),
        )

    def _anchored(self, candidate_z=0.0, obstacle_z=None):
        accumulator = self._accumulator()
        self._accept(accumulator, block(-20, -20, 40, 40, 0.0), stamp=10.0)
        if candidate_z != 0.0 or obstacle_z is not None:
            points = block(-20, -20, 40, 40, candidate_z)
            if obstacle_z is not None:
                points = np.vstack([points, block(80, 0, 1, 1, obstacle_z)])
            z = 0.4 if candidate_z == 0.0 else candidate_z + 0.4
            self._accept(accumulator, points, stamp=10.1, z=z)
        return accumulator

    def test_a_same_anchor_merge_keeps_the_obstacle_occupied(self):
        receiver = self._anchored()
        other = self._anchored(candidate_z=0.11, obstacle_z=0.20)
        receiver.merge(other)
        self.assertAlmostEqual(0.0, receiver._level_floor_z[0])
        self.assertEqual(om.OCCUPIED, receiver.grid_for(0).code_at(80, 0))

    def test_a_different_anchor_is_refused_before_any_write(self):
        receiver = self._anchored()
        other = self._accumulator()
        self._accept(other, block(-20, -20, 40, 40, 0.11), stamp=10.0, z=0.51)
        self.assertAlmostEqual(0.11, other._level_floor_z[0])
        before = {
            "levels": list(receiver.levels),
            "cells": receiver.grid_for(0).counts(),
            "anchors": dict(receiver._level_floor_z),
            "frames": receiver.accepted_frames,
        }
        with self.assertRaises(ValueError):
            receiver.merge(other)
        self.assertEqual(before["levels"], list(receiver.levels))
        self.assertEqual(before["cells"], receiver.grid_for(0).counts())
        self.assertEqual(before["anchors"], dict(receiver._level_floor_z))
        self.assertEqual(before["frames"], receiver.accepted_frames)

    def test_a_conflict_on_a_later_level_does_not_partially_merge_an_earlier_one(self):
        receiver = self._anchored()
        self._accept(receiver, block(-20, -20, 40, 40, 3.0), stamp=10.5, z=3.4)
        other = self._accumulator()
        self._accept(other, block(-20, -20, 40, 40, 0.0), stamp=10.0)
        self._accept(other, block(-20, -20, 40, 40, 3.11), stamp=10.5, z=3.51)
        before = receiver.grid_for(0).counts()
        before_high = receiver.grid_for(1).counts()
        before_anchors = dict(receiver._level_floor_z)
        before_frames = receiver.accepted_frames
        with self.assertRaises(ValueError):
            receiver.merge(other)
        self.assertEqual(before, receiver.grid_for(0).counts())
        self.assertEqual(before_high, receiver.grid_for(1).counts())
        self.assertEqual(before_anchors, dict(receiver._level_floor_z))
        self.assertEqual(before_frames, receiver.accepted_frames)

    def test_a_legacy_grid_without_an_anchor_is_refused(self):
        receiver = self._anchored()
        other = self._anchored()
        del other._level_floor_z[0]
        with self.assertRaises(ValueError):
            receiver.merge(other)


if __name__ == "__main__":
    unittest.main()

