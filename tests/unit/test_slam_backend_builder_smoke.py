"""Builder smoke tests for the D-path SLAM backend on a synthetic world.

The world is a 30 m x 20 m room (floor, 3 m walls, pillars, boxes) sampled on
its surfaces; a 360 degree lidar with 20 m range keeps the nearest return per
angular cell and adds 0.01 m Gaussian noise. Seeds are fixed.
"""

import json
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend, yaw_pose
from embodied_navigation.localization_mapping.backend.correction import CorrectionAdmission
from embodied_navigation.localization_mapping.backend.keyframes import (
    Keyframe,
    read_keyframe,
    read_spool,
    transform_points,
)
from embodied_navigation.localization_mapping.backend.map_store import MapStore
from embodied_navigation.localization_mapping.backend.pose_graph import PoseGraph, invert, pose_difference
from embodied_navigation.localization_mapping.backend.registration import icp_point_to_plane
from embodied_navigation.localization_mapping.backend.scan_context import (
    ScanContextConfig,
    descriptor,
    distance,
    shift_to_yaw,
)

SC = ScanContextConfig()


def _box(rng, lo, hi, spacing=0.1):
    """Points on the five visible faces (no bottom) of an axis-aligned box."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    faces = []
    for axis in range(3):
        for side in (lo[axis], hi[axis]):
            if axis == 2 and side == lo[2]:
                continue
            others = [a for a in range(3) if a != axis]
            area = np.prod(hi[others] - lo[others])
            n = int(area / spacing**2)
            face = rng.uniform(lo, hi, size=(n, 3))
            face[:, axis] = side
            faces.append(face)
    return np.vstack(faces)


def synthetic_world(seed=7):
    rng = np.random.default_rng(seed)
    parts = [
        np.c_[rng.uniform([0, 0], [30, 20], size=(9600, 2)), np.zeros(9600)],  # floor
        _box(rng, [-0.2, 0, 0], [0, 20, 3]),
        _box(rng, [30, 0, 0], [30.2, 20, 3]),
        _box(rng, [0, -0.2, 0], [30, 0, 3]),
        _box(rng, [0, 20, 0], [30, 20.2, 3]),
    ]
    for x, y in [(9, 9), (15, 8), (21, 10), (12, 17), (18, 2), (27, 17), (2, 17)]:
        parts.append(_box(rng, [x - 0.3, y - 0.3, 0], [x + 0.3, y + 0.3, 3]))
    for lo, hi in [
        ([10, 10, 0], [12, 11.5, 1.0]),
        ([16, 10.5, 0], [19, 12, 1.6]),
        ([22, 6, 0], [23, 8, 0.8]),
        ([7, 6, 0], [8.5, 7, 2.0]),
        ([26, 1, 0], [28, 2.5, 1.2]),
        ([1, 8, 0], [2, 11, 1.4]),
        ([13, 15.5, 0], [16, 16.5, 0.9]),
    ]:
        parts.append(_box(rng, lo, hi))
    return np.vstack(parts)


WORLD = synthetic_world()


def scan(T_world_lidar, seed):
    """Nearest return per (0.5 deg azimuth, 1 deg elevation) cell, lidar frame."""
    rng = np.random.default_rng(seed)
    local = transform_points(invert(T_world_lidar), WORLD)
    rng_xyz = np.linalg.norm(local, axis=1)
    local, rng_xyz = local[(rng_xyz < 20) & (rng_xyz > 0.3)], rng_xyz[(rng_xyz < 20) & (rng_xyz > 0.3)]
    az = np.floor(np.degrees(np.arctan2(local[:, 1], local[:, 0])) / 0.5).astype(int)
    el = np.floor(np.degrees(np.arcsin(local[:, 2] / rng_xyz))).astype(int)
    order = np.argsort(rng_xyz)
    _, first = np.unique(np.c_[az, el][order], axis=0, return_index=True)
    points = local[order][first]
    return points + rng.normal(0, 0.01, points.shape)


def pose(x, y, yaw, z=0.5):
    T = yaw_pose(yaw)
    T[:3, 3] = [x, y, z]
    return T


EXTRINSIC = pose(0.1, 0.0, 0.0, 0.2)


def rectangle_path(step=2.0, extra=4):
    corners = [(5, 4), (25, 4), (25, 14), (5, 14), (5, 4)]
    poses = []
    for (x0, y0), (x1, y1) in zip(corners, corners[1:]):
        heading = np.arctan2(y1 - y0, x1 - x0)
        length = np.hypot(x1 - x0, y1 - y0)
        for s in np.arange(0, length, step):
            poses.append(pose(x0 + s * np.cos(heading), y0 + s * np.sin(heading), heading))
    return poses + poses[1 : 1 + extra]


def drifted(true_poses, yaw_per_step, scale):
    """Odometry starting at the first true pose, with yaw bias and scale error."""
    odom = [true_poses[0]]
    for a, b in zip(true_poses, true_poses[1:]):
        delta = invert(a) @ b
        delta[:3, 3] *= scale
        odom.append(odom[-1] @ delta @ yaw_pose(yaw_per_step))
    return odom


def keyframes(true_poses, odom_poses, epoch=0, seed0=0):
    return [
        Keyframe((epoch, i + 1), (i + 1) * 1_000_000_000, odom, EXTRINSIC.copy(), scan(true @ EXTRINSIC, seed0 + i))
        for i, (true, odom) in enumerate(zip(true_poses, odom_poses))
    ]


def build_map():
    truth = rectangle_path()
    odom = drifted(truth, 0.012, 1.005)
    backend = SlamBackend(BackendConfig())
    events = [backend.ingest(k) for k in keyframes(truth, odom)]
    return backend, events, truth, odom


class SpoolTests(unittest.TestCase):
    def write(self, root, epoch, seq, T, points, ready=True):
        directory = root / (("kf_" if ready else "tmp_") + f"{epoch}_{seq}")
        directory.mkdir()
        rows = ",".join("[" + ",".join(repr(v) for v in row) + "]" for row in T)
        (directory / "points_lidar.pcd").write_text(
            "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
            f"{len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n"
            + "".join(f"{x} {y} {z}\n" for x, y, z in points)
        )
        (directory / "metadata.json").write_text(
            '{"schema":"livo_keyframe/v1","epoch":%d,"sequence":%d,"stamp_ns":%d,"T_odom_body":[%s],'
            '"T_body_lidar":[[1,0,0,0],[0,1,0,0.2],[0,0,1,0],[0,0,0,1]],"final_stage":"livo",'
            '"point_count":%d,"source_point_count":%d,"copy_ns":1,"cloud_frame":"lidar"}\n'
            % (epoch, seq, seq * 10, rows, len(points), len(points))
        )

    def test_numeric_order_ready_only_and_row_major(self):  # R-SPOOL-1/2/3
        T = np.arange(16, dtype=float).reshape(4, 4)
        T[3] = [0, 0, 0, 1]
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            for seq in range(12, 0, -1):
                self.write(root, 0, seq, T, [(0, 1, 2), (3, 4, 5)])
            self.write(root, 0, 13, T, [(0, 1, 2)], ready=False)
            (root / "kf_0_14").mkdir()
            frames = read_spool(root)
            self.assertEqual([k.key[1] for k in frames], list(range(1, 13)))
            np.testing.assert_array_equal(frames[0].T_odom_body, T)
            np.testing.assert_array_equal(frames[0].points_lidar, [[0, 1, 2], [3, 4, 5]])
            (root / "kf_0_1/metadata.json").write_text('{"schema":"other"}')
            with self.assertRaisesRegex(ValueError, "kf_0_1"):
                read_keyframe(root / "kf_0_1")


class ComponentTests(unittest.TestCase):
    def test_scan_context_yaw_and_icp(self):  # R-SC-1/2/3, R-ICP-1/2
        T_a = pose(10, 6, 0.0)
        a, b = scan(T_a, 1), scan(T_a @ yaw_pose(0.7), 2)
        dist, shift = distance(descriptor(a, SC), descriptor(b, SC))
        yaw = shift_to_yaw(shift, SC)  # yaw of T_b_a
        far, _ = distance(descriptor(a, SC), descriptor(scan(pose(22, 15, 0.0), 3), SC))
        print(f"\nR-SC-1 dist={dist:.3f} yaw={yaw:.3f} (want -0.7)  R-SC-3 far={far:.3f}")
        self.assertLess(dist, 0.15)
        self.assertLess(abs(yaw + 0.7), 2 * np.pi / SC.sectors)
        self.assertGreater(far, dist)
        truth = yaw_pose(-0.7)
        result = icp_point_to_plane(a, b, yaw_pose(yaw))
        t, r = pose_difference(result.T_target_source, truth)
        print(f"R-SC-2 icp err {t:.4f} m {r:.4f} rad")
        self.assertTrue(t < 0.1 and r < 0.03)
        offset = pose(0.5, -0.6, 0.25, 0.0)
        start = time.perf_counter()
        result = icp_point_to_plane(a, b, truth @ offset)
        t, r = pose_difference(result.T_target_source, truth)
        print(f"R-ICP-1 err {t:.4f} m {r:.5f} rad fitness {result.fitness:.2f} {time.perf_counter() - start:.2f}s")
        self.assertTrue(result.converged and t < 0.05 and r < 0.01)
        self.assertFalse(icp_point_to_plane(a[:40], b, np.eye(4)).converged)

    def test_pose_graph_loop_and_robust(self):  # R-PG-1/2
        def square(drift_deg_per_m):
            truth, odom = [], [np.eye(4)]
            for i in range(40):
                side, s = divmod(i, 10)
                truth.append(pose(*[(s, 0), (10, s), (10 - s, 10), (0, 10 - s)][side], np.pi / 2 * side, 0))
            for a, b in zip(truth, truth[1:]):
                odom.append(odom[-1] @ invert(a) @ b @ yaw_pose(np.radians(drift_deg_per_m)))
            return truth, odom

        truth, odom = square(0.5)
        info = np.diag([100.0] * 3 + [1000.0] * 3)
        graph = PoseGraph()
        for i, T in enumerate(odom):
            graph.add_node(i, T)
        for i in range(39):
            graph.add_edge(i, i + 1, invert(odom[i]) @ odom[i + 1], info)
        graph.add_edge(39, 0, invert(truth[39]) @ truth[0], info)
        before = pose_difference(graph.pose(39), truth[39])[0]
        report = graph.optimize()
        after = pose_difference(graph.pose(39), truth[39])[0]
        print(f"\nR-PG-1 end err {before:.3f} -> {after:.4f} m {report}")
        self.assertLess(after, before / 10)
        self.assertLess(report["error_after"], report["error_before"])
        truth, _ = square(0.0)
        graph = PoseGraph()
        for i, T in enumerate(truth):
            graph.add_node(i, T)
        strong = np.diag([1e4] * 3 + [1e5] * 3)
        for i in range(39):
            graph.add_edge(i, i + 1, invert(truth[i]) @ truth[i + 1], strong)
        wrong = invert(truth[30]) @ truth[0] @ pose(3, 0, 0, 0)
        graph.add_edge(30, 0, wrong, np.diag([100.0] * 3 + [1000.0] * 3), robust=True)
        graph.optimize()
        worst = max(pose_difference(graph.pose(i), truth[i])[0] for i in range(40))
        print(f"R-PG-2 worst node err {worst:.4f} m")
        self.assertLess(worst, 0.3)

    def test_correction_admission(self):  # R-COR-1/2/3
        T_ob = pose(30, 0, 0, 0)
        world_body = pose(31, 2, 0.3, 0)
        C1 = world_body @ invert(T_ob)
        C2 = world_body @ yaw_pose(0.003) @ invert(T_ob)
        gate = CorrectionAdmission()
        self.assertIsNone(gate.propose(C1, T_ob, 0))
        np.testing.assert_allclose(gate.propose(C2, T_ob, 1_000_000_000), C2)
        jump = pose(3, 0, 0, 0) @ C2
        self.assertIsNone(gate.propose(jump, T_ob, 2_000_000_000))
        self.assertIsNone(gate.propose(jump, T_ob, 3_000_000_000))
        np.testing.assert_allclose(gate.current, C2)
        self.assertEqual(len(gate.rejections), 2)
        late = CorrectionAdmission()
        self.assertIsNone(late.propose(C1, T_ob, 0))
        self.assertIsNone(late.propose(C1, T_ob, 11_000_000_000))

    def test_map_round_trip(self):  # R-MAP-1
        truth = rectangle_path()[:5]
        store = MapStore()
        for k, T in zip(keyframes(truth, truth), truth):
            store.add(k, T, descriptor(k.points_body(), SC))
        with tempfile.TemporaryDirectory() as directory:
            store.save(directory)
            loaded = MapStore.load(directory)
        for key in store.poses:
            np.testing.assert_array_equal(loaded.poses[key], store.poses[key])
            np.testing.assert_array_equal(loaded.keyframes[key].points_lidar, store.keyframes[key].points_lidar)
            np.testing.assert_array_equal(loaded.descriptors[key], store.descriptors[key])
        np.testing.assert_array_equal(loaded.global_map(), store.global_map())


class BackendTests(unittest.TestCase):
    def test_mapping_closes_large_loop_then_relocalizes(self):  # R-BE-1..4
        start = time.perf_counter()
        backend, events, truth, odom = build_map()
        drift = pose_difference(odom[-1], truth[-1])
        loops = [e["key"] for e in events if e.get("loop")]
        err = pose_difference(backend.T_world_odom @ odom[-1], truth[-1])
        print(
            f"\nR-BE-1 frames={len(events)} drift={drift[0]:.2f} m/{drift[1]:.3f} rad loops={loops} "
            f"end err={err[0]:.3f} m/{err[1]:.4f} rad {time.perf_counter() - start:.1f}s"
        )
        self.assertGreater(drift[0], 1.5)
        self.assertTrue(loops)
        self.assertLess(err[0], 0.3)

        with tempfile.TemporaryDirectory() as directory:
            backend.save_map(directory)
            prior = MapStore.load(directory)
        start = time.perf_counter()
        start_pose = pose(3, 4, 1.2)
        true_path = [start_pose @ pose(0.9 * i, 0, 0, 0) for i in range(14)]
        C_true = start_pose
        self.assertAlmostEqual(np.linalg.norm(C_true[:2, 3]), 5.0)
        odom = [invert(C_true) @ T for T in true_path]
        jumped = [T if i < 8 else pose(2, 0, 0, 0) @ T for i, T in enumerate(odom)]
        frames = keyframes(true_path, jumped, epoch=1, seed0=100)
        frames[5] = Keyframe(frames[5].key, frames[5].stamp_ns, frames[5].T_odom_body, EXTRINSIC, np.empty((0, 3)))
        reloc = SlamBackend(BackendConfig(), prior_map=prior)
        first_adopted = None
        for i, k in enumerate(frames):
            event = reloc.ingest(k)
            if i == 5:
                self.assertIn("error", event)
                continue
            if event["adopted"] and first_adopted is None:
                first_adopted = i
            if reloc.T_world_odom is not None:
                err = pose_difference(reloc.T_world_odom @ k.T_odom_body, true_path[i])
                print(f"R-BE-2/3 frame {i} {event['search']} adopted={event['adopted']} err={err[0]:.3f} m/{err[1]:.4f}")
                if i < 8 and first_adopted is not None:
                    self.assertLess(err[0], 0.2)
        print(f"reloc {time.perf_counter() - start:.1f}s")
        self.assertIsNotNone(first_adopted)
        self.assertLess(first_adopted, 10)
        final = pose_difference(reloc.T_world_odom @ frames[-1].T_odom_body, true_path[-1])
        self.assertLess(final[0], 0.2)


if __name__ == "__main__":
    unittest.main()
