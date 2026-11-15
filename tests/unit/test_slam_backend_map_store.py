"""SLAM-BE map persistence (R-MAP-*) against the frozen map_store.py interface.

MapStore has no frozen getters, so the round trip is checked through the
frozen on-disk format (map.json `embodied_navigation_map/v1` + one .npz per
keyframe holding `points_lidar` and `descriptor`) and through global_map /
submap results.
"""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

import slam_backend_fixtures as F
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe
from embodied_navigation.localization_mapping.backend.map_store import MapStore

SEC = 1_000_000_000


def _collect(node, name, out):
    """All values stored under key ``name`` anywhere in a JSON tree."""
    if isinstance(node, dict):
        for key, value in node.items():
            if key == name:
                out.append(value)
            _collect(value, name, out)
    elif isinstance(node, list):
        for value in node:
            _collect(value, name, out)
    return out


def _sorted_rows(points):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    return points[np.lexsort(points.T[::-1])]


def _npz_contents(directory):
    rows = []
    for path in sorted(Path(directory).rglob("*.npz")):
        with np.load(path) as data:
            rows.append((np.array(data["points_lidar"]), np.array(data["descriptor"])))
    return rows


def _match(testcase, expected, actual):
    """Each expected (points, descriptor) pair appears exactly once in actual."""
    testcase.assertEqual(len(expected), len(actual))
    remaining = list(actual)
    for pts, desc in expected:
        hit = [i for i, (p, d) in enumerate(remaining)
               if p.shape == pts.shape and np.array_equal(p, pts) and np.array_equal(d, desc)]
        testcase.assertEqual(len(hit), 1, "keyframe points/descriptor not preserved exactly")
        remaining.pop(hit[0])


class MapStoreTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        world = F.make_world()
        rng = np.random.default_rng(F.SEED)
        self.frames, self.poses, self.updated = [], {}, {}
        for k in range(5):
            T_scene_body = F.loop_pose_at(2.5 * k)
            T_body_lidar = F.T_BODY_LIDAR @ F.pose(0.01 * k, 0.0, 0.02 * k)
            lidar = world.scan(T_scene_body @ T_body_lidar, 500 + k)
            kf = Keyframe(key=(3, k + 1), stamp_ns=(10 + k) * SEC + 123,
                          T_odom_body=F.pose(2.5 * k, 0.1 * k, 0.01 * k),
                          T_body_lidar=T_body_lidar, points_lidar=lidar)
            desc = rng.uniform(0.0, 3.0, (20, 60))
            self.frames.append((kf, desc))
            self.poses[kf.key] = T_scene_body
            self.updated[kf.key] = F.pose(0.02, -0.03, 0.004) @ T_scene_body

    def tearDown(self):
        self._tmp.cleanup()

    def _store(self):
        store = MapStore()
        for kf, desc in self.frames:
            store.add(kf, self.poses[kf.key], desc)
        store.update_poses(self.updated)
        return store

    def test_r_map_1_save_load_roundtrip(self):
        # Catches: dropping or mangling a field on save (pose, T_body_lidar, stamp, points, descriptor).
        store = self._store()
        first, second = self.root / "first", self.root / "second"
        store.save(first)
        meta = json.loads((first / "map.json").read_text())
        self.assertEqual(meta.get("schema"), "embodied_navigation_map/v1")
        expected = [(kf.points_lidar, desc) for kf, desc in self.frames]
        _match(self, expected, _npz_contents(first))
        # poses saved are the *updated* ones; T_body_lidar and stamp_ns are kept
        saved_poses = [np.array(v, dtype=float) for v in _collect(meta, "T_world_body", [])]
        for key, T in self.updated.items():
            self.assertTrue(any(np.array_equal(T, s) for s in saved_poses), f"pose of {key} not saved exactly")
        saved_lidar = [np.array(v, dtype=float) for v in _collect(meta, "T_body_lidar", [])]
        for kf, _ in self.frames:
            self.assertTrue(any(np.array_equal(kf.T_body_lidar, s) for s in saved_lidar))
        saved_stamps = _collect(meta, "stamp_ns", [])
        for kf, _ in self.frames:
            self.assertIn(kf.stamp_ns, saved_stamps)

        loaded = MapStore.load(first)
        loaded.save(second)
        meta2 = json.loads((second / "map.json").read_text())
        _match(self, expected, _npz_contents(second))
        for name in ("T_world_body", "T_body_lidar", "stamp_ns"):
            with self.subTest(field=name):
                a = sorted(json.dumps(v) for v in _collect(meta, name, []))
                b = sorted(json.dumps(v) for v in _collect(meta2, name, []))
                self.assertEqual(a, b)

        g1, g2 = store.global_map(), loaded.global_map()
        self.assertGreater(len(g1), 1000)
        self.assertEqual(g1.shape, g2.shape)
        np.testing.assert_allclose(_sorted_rows(g1), _sorted_rows(g2), rtol=0, atol=1e-9)
        center = self.updated[(3, 3)][:3, 3]
        s1 = store.submap(center, 8.0, exclude_keys=[(3, 3)])
        s2 = loaded.submap(center, 8.0, exclude_keys=[(3, 3)])
        self.assertGreater(len(s1), 0)
        self.assertEqual(s1.shape, s2.shape)
        np.testing.assert_allclose(_sorted_rows(s1), _sorted_rows(s2), rtol=0, atol=1e-9)

    def test_contract_global_map_is_in_world_frame(self):
        # Catches: global_map built from lidar-frame points or ignoring T_body_lidar / update_poses.
        store = self._store()
        g = store.global_map(voxel_m=0.2)
        truth = np.concatenate([F.transform_points(self.updated[kf.key] @ kf.T_body_lidar, kf.points_lidar)
                                for kf, _ in self.frames])
        d, _ = cKDTree(truth).query(g)
        self.assertLess(float(np.percentile(d, 95)), 0.2 * np.sqrt(3))


if __name__ == "__main__":
    unittest.main()
