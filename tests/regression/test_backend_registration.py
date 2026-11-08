"""Real geometric business flows, without ROS/Gazebo or truth in acceptance.

The fixtures are synthetic mesh-like surfaces, not a recorded sensor benchmark.
Truth is used only in assertions after the backend returns its own decision.
"""
import json
from pathlib import Path
import sys
import tempfile
import unittest


import numpy as np
from scipy.spatial.transform import Rotation

from embodied_navigation.localization_mapping.backend.external_map import ExternalMap, read_pcd
from embodied_navigation.localization_mapping.backend.geometry import inverse, pose_distance, transform_points
from embodied_navigation.localization_mapping.backend.registration import RegistrationConfig, Relocalizer


def scene(seed=8):
    """Mesh-style XYZ samples: finite floor/walls and one asymmetric object."""
    rng = np.random.default_rng(seed)
    floor = np.c_[rng.uniform(-3, 5, 700), rng.uniform(-2, 4, 700), np.zeros(700)]
    wall = np.c_[np.full(550, -3.), rng.uniform(-2, 4, 550), rng.uniform(0, 3, 550)]
    back = np.c_[rng.uniform(-3, 5, 450), np.full(450, 4.), rng.uniform(0, 3, 450)]
    object_points = rng.normal([1.5, .8, 1.1], [.2, .3, .6], (220, 3))
    return np.r_[floor, wall, back, object_points]


def transform():
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler("xyz", [.13, -.22, .85]).as_matrix()
    t[:3, 3] = [4, -5, 1.2]
    return t


def partial_scan(points=None, T_map_lidar=None, seed=91):
    points = scene() if points is None else points
    t = transform() if T_map_lidar is None else T_map_lidar
    visible = points[(points[:, 0] < 4) & (points[:, 1] < 3.5)]
    rng = np.random.default_rng(seed)
    scan = transform_points(visible, inverse(t)) + rng.normal(0, .01, visible.shape)
    return np.r_[scan, rng.uniform(-3, 3, (70, 3))]


def write_ascii_pcd(path, points, colors=None):
    points = np.asarray(points)
    fields = "x y z" if colors is None else "x y z r g b"
    widths = "4 4 4" if colors is None else "4 4 4 1 1 1"
    types = "F F F" if colors is None else "F F F U U U"
    counts = "1 1 1" if colors is None else "1 1 1 1 1 1"
    with Path(path).open("w") as stream:
        stream.write(f"# .PCD v0.7\nVERSION 0.7\nFIELDS {fields}\nSIZE {widths}\nTYPE {types}\nCOUNT {counts}\nWIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n")
        matrix = points if colors is None else np.c_[points, colors]
        np.savetxt(stream, matrix, fmt="%.9g")


class RegistrationBusinessFlows(unittest.TestCase):
    def test_pcd_asset_roundtrip_then_cold_start_partial_noisy_scan(self):
        points = scene()
        colors = np.tile([80, 120, 210], (len(points), 1)).astype(np.uint8)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            write_ascii_pcd(path / "mesh_rgb.pcd", points, colors)
            external = ExternalMap.from_pcd(path / "mesh_rgb.pcd", block_size=20, overlap=3, voxel_size=.10)
            external.save(path / "asset")
            self.assertIsNone(json.loads((path / "asset/map.json").read_text())["trajectory"])
            loaded = ExternalMap.load(path / "asset")
            np.testing.assert_allclose(loaded.points, points, atol=5e-9)
            np.testing.assert_array_equal(loaded.colors, colors)
            localizer = Relocalizer(loaded, RegistrationConfig(voxel_size=.10, ransac_iterations=2200))
            estimate = localizer.localize(partial_scan(points))  # deliberately no pose prior
        print("partial_mesh_relocalization", estimate.elapsed_s, estimate.fitness, estimate.rmse)
        self.assertTrue(estimate.accepted, estimate)
        self.assertGreaterEqual(estimate.fitness, .90)
        translation, angle = pose_distance(estimate.T_map_lidar, transform())
        self.assertLess(translation, .03)
        self.assertLess(angle, .01)
        self.assertGreater(estimate.observability, .001)
        self.assertFalse(estimate.T_map_lidar.flags.writeable)

    def test_independently_sampled_mesh_scan_not_identical_points(self):
        external = ExternalMap(scene(8), block_size=20, overlap=3, voxel_size=.10)
        localizer = Relocalizer(external, RegistrationConfig(voxel_size=.10, ransac_iterations=2200,
                                                            correspondence_distance=.24, max_rmse=.14))
        scan = transform_points(scene(21), inverse(transform()))
        estimate = localizer.localize(scan)
        print("independently_sampled_mesh", estimate.elapsed_s, estimate.fitness, estimate.rmse)
        self.assertTrue(estimate.accepted, estimate)
        translation, angle = pose_distance(estimate.T_map_lidar, transform())
        self.assertLess(translation, .08)
        self.assertLess(angle, .025)

    def test_wrong_scene_and_unobservable_plane_never_return_pose(self):
        external = ExternalMap(scene(), block_size=20, overlap=3, voxel_size=.10)
        localizer = Relocalizer(external, RegistrationConfig(voxel_size=.10, ransac_iterations=1000))
        rng = np.random.default_rng(8)
        wrong = rng.normal(0, .8, (800, 3))
        wrong[:, 0] *= 4
        estimate = localizer.localize(wrong)
        self.assertFalse(estimate.accepted, estimate)
        self.assertIsNone(estimate.T_map_lidar)
        plane = np.c_[rng.uniform(-4, 4, (800, 2)), np.zeros(800)]
        estimate = localizer.localize(plane)
        self.assertFalse(estimate.accepted)
        self.assertEqual(estimate.reason, "degenerate_scan_geometry")
        self.assertIsNone(estimate.T_map_lidar)

    def test_repeated_places_are_ambiguous_not_arbitrarily_selected(self):
        rng = np.random.default_rng(4)
        place = rng.uniform([-2, -1, -1], [2, 2, 2], (700, 3))
        repeated_map = np.r_[place, place + [12, 0, 0]]
        external = ExternalMap(repeated_map, block_size=22, overlap=3, voxel_size=0)
        estimate = Relocalizer(external, RegistrationConfig(voxel_size=0, ransac_iterations=1800)).localize(
            transform_points(place, inverse(transform())))
        self.assertFalse(estimate.accepted, estimate)
        self.assertEqual(estimate.reason, "ambiguous_geometry")
        self.assertGreaterEqual(estimate.diagnostics["verified_distinct_poses"], 2)
        self.assertIsNone(estimate.T_map_lidar)

    def test_binary_pcd_preserves_packed_rgb_and_rejects_truncation(self):
        points = scene()[:100]
        colors = np.tile(np.uint32(0x1255AB), len(points))
        dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])
        payload = np.zeros(len(points), dtype=dtype)
        for i, key in enumerate(("x", "y", "z")):
            payload[key] = points[:, i]
        payload["rgb"] = colors.view("<f4")
        header = f"FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n".encode()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "map.pcd"
            path.write_bytes(header + payload.tobytes())
            xyz, rgb = read_pcd(path)
            np.testing.assert_allclose(xyz, points, atol=2e-7)
            np.testing.assert_array_equal(rgb, np.tile([0x12, 0x55, 0xAB], (len(points), 1)))
            path.write_bytes(header + payload.tobytes()[:-1])
            with self.assertRaisesRegex(ValueError, "payload size"):
                read_pcd(path)

    def test_unsearched_map_blocks_do_not_claim_global_uniqueness(self):
        points = scene()
        external = ExternalMap(np.r_[points, points + [20, 0, 0]],
                               block_size=12, overlap=4, voxel_size=.10)
        estimate = Relocalizer(external, RegistrationConfig(voxel_size=.10, max_candidates=1,
                                                            ransac_iterations=1000)).localize(
            transform_points(points, inverse(transform())))
        self.assertFalse(estimate.accepted, estimate)
        self.assertEqual(estimate.reason, "global_search_budget_exhausted")
        self.assertTrue(estimate.diagnostics["bounded_search"])
        self.assertIsNone(estimate.T_map_lidar)


class PriorTrackingBusinessFlows(unittest.TestCase):
    def test_explicit_prior_tracking_rejects_wrong_floor(self):
        from embodied_navigation.localization_mapping.backend.tracking import PriorTracker
        points=scene();asset=ExternalMap(points,block_size=20,overlap=3,voxel_size=.10)
        actual=transform();scan=partial_scan(T_map_lidar=actual,seed=51)
        prior=actual.copy();prior[:3,3]+=[.04,-.03,.02]
        tracker=PriorTracker(asset,RegistrationConfig(voxel_size=.10))
        accepted=tracker.localize(scan,prior)
        self.assertTrue(accepted.accepted,accepted)
        self.assertFalse(accepted.diagnostics['global_uniqueness_claimed'])
        self.assertLess(pose_distance(accepted.T_map_lidar,actual)[0],.05)
        wrong=prior.copy();wrong[2,3]+=1.5
        rejected=tracker.localize(scan,wrong)
        self.assertFalse(rejected.accepted,rejected)

if __name__ == "__main__":
    unittest.main()
