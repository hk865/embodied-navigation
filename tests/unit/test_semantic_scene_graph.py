import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from embodied_navigation.localization_mapping.backend.keyframes import Keyframe
from embodied_navigation.localization_mapping.backend.map_store import MapStore
from embodied_navigation.localization_mapping.semantic_mapping import (
    STATIC,
    KeyframeSceneGraph,
    LayeredMap,
    Observation,
    PointMask,
    build_scene_graph_from_map_dir,
    build_scene_graph_from_map_store,
    estimate_keyframe_clearance_m,
    load_rgb_index_report,
    sync_backend_loop_update,
)

REPO_ROOT = Path(__file__).resolve().parents[2]


def _pose(x: float, y: float, z: float, yaw_rad: float = 0.0) -> np.ndarray:
    c, s = float(np.cos(yaw_rad)), float(np.sin(yaw_rad))
    T = np.eye(4, dtype=np.float64)
    T[0, 0], T[0, 1] = c, -s
    T[1, 0], T[1, 1] = s, c
    T[:3, 3] = [x, y, z]
    return T


def _box_points(radius_x: float, radius_y: float, n_per_side: int = 12) -> np.ndarray:
    """Synthetic room/corridor walls around body origin in z in [0.0, 1.0]."""
    pts = []
    xs = np.linspace(-radius_x, radius_x, n_per_side)
    ys = np.linspace(-radius_y, radius_y, n_per_side)
    for x in xs:
        pts.append([x, radius_y, 0.5])
        pts.append([x, -radius_y, 0.5])
    for y in ys:
        pts.append([radius_x, y, 0.5])
        pts.append([-radius_x, y, 0.5])
    return np.asarray(pts, dtype=np.float64)


class KeyframeSceneGraphTests(unittest.TestCase):
    def test_two_floor_segmentation_and_stair_edge_detection(self):
        sg = KeyframeSceneGraph()
        for i in range(5):
            sg.add_keyframe(f"1:{i}", _pose(float(i), 0.0, 0.0), clearance_m=2.0)
        # Stair climb keyframes at z = 0.85, 1.65, 2.45
        for idx, z in enumerate((0.85, 1.65, 2.45), start=5):
            sg.add_keyframe(f"1:{idx}", _pose(5.0, float(idx - 4), z), clearance_m=1.1)
        for j in range(5):
            sg.add_keyframe(f"1:{8 + j}", _pose(5.0 + j, 4.0, 3.3), clearance_m=2.0)

        floors = sg.segment_floors()
        self.assertEqual(sorted(floors.keys()), ["f0", "f1"])
        self.assertEqual(floors["f0"].level_index, 0)
        self.assertEqual(floors["f1"].level_index, 1)
        self.assertAlmostEqual(floors["f0"].z_mean_m, 0.0, places=3)
        self.assertAlmostEqual(floors["f1"].z_mean_m, 3.3, places=3)
        self.assertEqual(floors["f0"].keyframe_ids, [f"1:{i}" for i in range(5)])
        self.assertEqual(floors["f1"].keyframe_ids, [f"1:{8 + j}" for j in range(5)])

        self.assertEqual(len(sg.stairs), 1)
        stair = sg.stairs[0]
        self.assertEqual(stair.from_floor, "f0")
        self.assertEqual(stair.to_floor, "f1")
        self.assertEqual(stair.keyframe_ids, ["1:4", "1:5", "1:6", "1:7", "1:8"])
        self.assertAlmostEqual(stair.start_xyz[2], 0.0, places=3)
        self.assertAlmostEqual(stair.end_xyz[2], 3.3, places=3)

    def test_doorway_bottleneck_room_partitioning_and_rename(self):
        sg = KeyframeSceneGraph()
        # Room 1: 6 wide keyframes with 6 RGB images (should cap at 4 representative images)
        for i in range(6):
            sg.add_keyframe(
                f"1:{i}",
                _pose(float(i), 0.0, 0.0),
                clearance_m=2.2,
                rgb_path=f"rgb/r1_{i:02d}.png",
            )
        # Narrow doorway bottleneck between x=5 and x=7 (within link_radius=2.5 across doorway)
        sg.add_keyframe("1:6", _pose(6.0, 0.0, 0.0), clearance_m=0.55)
        # Room 2: 4 wide keyframes
        for j in range(4):
            sg.add_keyframe(
                f"1:{7 + j}",
                _pose(7.0 + j, 0.0, 0.0),
                clearance_m=2.4,
                rgb_path=f"rgb/r2_{j:02d}.png",
            )

        sg.segment_floors()
        rooms = sg.partition_rooms(doorway_clearance_m=0.85, link_radius_m=2.5, min_room_keyframes=2)

        self.assertEqual(sorted(rooms.keys()), ["f0_room_01", "f0_room_02"])
        self.assertEqual(len(rooms["f0_room_01"].anchor_keyframe_ids), 6)
        self.assertEqual(len(rooms["f0_room_02"].anchor_keyframe_ids), 4)
        self.assertLessEqual(len(rooms["f0_room_01"].representative_rgb_paths), 4)
        self.assertEqual(len(rooms["f0_room_01"].representative_rgb_paths), 4)
        self.assertEqual(len(rooms["f0_room_02"].representative_rgb_paths), 4)

        self.assertEqual(len(sg.doorways), 1)
        doorway = sg.doorways[0]
        self.assertEqual((doorway.room_a, doorway.room_b), ("f0_room_01", "f0_room_02"))
        self.assertAlmostEqual(doorway.gateway_xyz[0], 6.0, places=3)
        self.assertAlmostEqual(doorway.clearance_m, 0.55, places=3)

        renamed = sg.rename_room("f0_room_01", "lobby")
        self.assertEqual(renamed.room_id, "f0_room_01")
        self.assertEqual(renamed.alias, "lobby")

    def test_keyframe_anchored_object_reprojection_on_loop_closure(self):
        sg = KeyframeSceneGraph()
        for i in range(3):
            sg.add_keyframe(f"1:{i}", _pose(float(i), 0.0, 0.0), clearance_m=2.0)
        sg.segment_floors()
        sg.partition_rooms()

        obj = sg.add_object(
            "fire_extinguisher",
            "1:1",
            local_center_xyz=[1.0, -0.5, 0.6],
            local_extent_xyz=[0.3, 0.3, 0.8],
            confidence=0.92,
            producer="vlm_detector",
        )
        self.assertEqual(obj.room_id, "f0_room_01")
        np.testing.assert_allclose(obj.world_center_xyz, [2.0, -0.5, 0.6], atol=1e-6)

        # Loop closure shifts poses by +0.5 m in X, +0.25 m in Y, and rotates kf 1:1 by +90 deg yaw
        shifted_poses = {
            f"1:{i}": _pose(float(i) + 0.5, 0.25, 0.0, yaw_rad=(np.pi / 2 if i == 1 else 0.0))
            for i in range(3)
        }
        stats = sg.reproject_from_poses(shifted_poses)
        self.assertEqual(stats["updated_keyframes"], 3)
        self.assertEqual(stats["reprojected_objects"], 1)
        # Local [1.0, -0.5, 0.6] rotated by +90 deg -> [0.5, 1.0, 0.6], plus [1.5, 0.25, 0.0]
        np.testing.assert_allclose(obj.world_center_xyz, [2.0, 1.25, 0.6], atol=1e-6)
        np.testing.assert_allclose(
            sg.rooms["f0_room_01"].centroid_xyz, [1.5, 0.25, 0.0], atol=1e-6
        )

    def test_loop_closure_merges_drift_split_rooms_on_same_floor(self):
        sg = KeyframeSceneGraph()
        # Pass 1: room keyframes around x=[0..3], y=0
        for i in range(3):
            sg.add_keyframe(
                f"1:{i}",
                _pose(float(i), 0.0, 0.0),
                clearance_m=2.0,
                rgb_path=f"rgb/pass1_{i}.png",
            )
        # Pass 2 (drifted by +4.0 m in Y so initially far apart -> separate room)
        for i in range(3):
            sg.add_keyframe(
                f"1:{10 + i}",
                _pose(float(i), 4.0, 0.0),
                clearance_m=2.0,
                rgb_path=f"rgb/pass2_{i}.png",
            )
        # Third room connected via bottleneck to pass 2 (all within link_radius_m=2.5)
        sg.add_keyframe("1:20", _pose(4.0, 4.0, 0.0), clearance_m=0.5)
        for i in range(3):
            sg.add_keyframe(f"1:{21 + i}", _pose(6.0 + i, 4.0, 0.0), clearance_m=2.0)

        sg.segment_floors()
        sg.partition_rooms(doorway_clearance_m=0.85, link_radius_m=2.5, min_room_keyframes=2)
        self.assertEqual(sorted(sg.rooms.keys()), ["f0_room_01", "f0_room_02", "f0_room_03"])
        self.assertEqual(len(sg.doorways), 1)
        self.assertEqual(
            (sg.doorways[0].room_a, sg.doorways[0].room_b),
            ("f0_room_02", "f0_room_03"),
        )

        sg.rename_room("f0_room_01", "atrium")
        sg.rename_room("f0_room_02", "atrium_drift_copy")
        obj = sg.add_object(
            "sofa",
            "1:11",
            local_center_xyz=[0.0, 0.0, 0.4],
            confidence=0.88,
            producer="fixture",
        )
        self.assertEqual(obj.room_id, "f0_room_02")

        # Loop closure corrects Pass 2 keyframes back to y=0.2 (overlapping Pass 1!)
        corrected = {
            f"1:{10 + i}": _pose(float(i) + 0.1, 0.2, 0.0) for i in range(3)
        }
        stats = sg.reproject_from_poses(corrected, merge_iou_min=0.25, merge_centroid_m=2.0)

        self.assertEqual(stats["rooms_before"], 3)
        self.assertEqual(stats["rooms_after"], 2)
        self.assertEqual(stats["merged_count"], 1)
        self.assertEqual(stats["merged_rooms"], [{"kept": "f0_room_01", "merged": "f0_room_02"}])
        self.assertEqual(sorted(sg.rooms.keys()), ["f0_room_01", "f0_room_03"])
        self.assertEqual(len(sg.doorways), 1)
        self.assertEqual(
            (sg.doorways[0].room_a, sg.doorways[0].room_b),
            ("f0_room_01", "f0_room_03"),
        )

        kept = sg.rooms["f0_room_01"]
        self.assertEqual(kept.alias, "atrium")
        self.assertIn("atrium_drift_copy", kept.merged_aliases)
        self.assertEqual(
            set(kept.anchor_keyframe_ids),
            {"1:0", "1:1", "1:2", "1:10", "1:11", "1:12"},
        )
        self.assertEqual(len(kept.representative_rgb_paths), 4)
        self.assertEqual(len(sg.keyframe_rgb_paths), 6)
        self.assertEqual(obj.room_id, "f0_room_01")
        np.testing.assert_allclose(obj.world_center_xyz, [1.1, 0.2, 0.4], atol=1e-6)

    def test_atomic_save_load_roundtrip_query_and_compact_summary(self):
        sg = KeyframeSceneGraph()
        for i in range(4):
            sg.add_keyframe(
                f"1:{i}",
                _pose(float(i), 0.0, 0.0),
                clearance_m=2.0,
                rgb_path=f"rgb/kf_{i}.png",
            )
        sg.segment_floors()
        sg.partition_rooms()
        sg.rename_room("f0_room_01", "hallway")
        sg.add_object(
            "chair",
            "1:2",
            local_center_xyz=[0.5, 0.5, 0.4],
            local_extent_xyz=[0.6, 0.6, 0.8],
            confidence=0.95,
            producer="fixture",
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            out_path = Path(tmpdir) / "nested" / "scene_graph.json"
            sg.save(out_path)
            loaded = KeyframeSceneGraph.load(out_path)

        self.assertEqual(list(loaded.keyframe_poses.keys()), ["1:0", "1:1", "1:2", "1:3"])
        self.assertEqual(loaded.rooms["f0_room_01"].alias, "hallway")
        self.assertEqual(len(loaded.objects), 1)
        np.testing.assert_allclose(
            loaded.objects["obj_0001"].world_center_xyz, [2.5, 0.5, 0.4], atol=1e-6
        )

        q = loaded.query(floor_id="f0", label="chair", near_xyz=[2.5, 0.5, 0.4], radius_m=0.5)
        self.assertEqual(len(q["objects"]), 1)
        self.assertEqual(q["objects"][0]["room_id"], "f0_room_01")

        summary = loaded.compact_agent_summary()
        self.assertEqual(summary["schema"], "keyframe_scene_graph_summary/v1")
        self.assertEqual(summary["counts"]["rooms"], 1)
        self.assertEqual(summary["counts"]["objects"], 1)
        self.assertEqual(summary["rooms"][0]["alias"], "hallway")
        self.assertEqual(summary["rooms"][0]["objects"], ["chair"])
        # Ensure summary stays compact (no raw 4x4 matrices)
        self.assertNotIn("keyframe_poses", summary)

    def test_build_from_map_store_and_sync_backend_loop_update_e2e(self):
        # Verify clearance estimator directly on wide vs doorway point clouds
        wide_pts = _box_points(2.5, 2.5)
        door_pts = _box_points(4.0, 0.5)
        self.assertGreater(estimate_keyframe_clearance_m(wide_pts), 2.0)
        self.assertLess(estimate_keyframe_clearance_m(door_pts), 0.7)
        self.assertIsNone(estimate_keyframe_clearance_m(np.empty((0, 3))))
        self.assertIsNone(estimate_keyframe_clearance_m(np.full((4, 3), np.nan)))
        self.assertIsNone(estimate_keyframe_clearance_m(np.array([[1.0, 0.0, 5.0]])))

        store = MapStore(voxel_m=0.05)
        layered = LayeredMap(map_id="building")
        rgb_index: dict[int, str] = {}

        # Floor 0: Room 1 (seq 0..3), Doorway (seq 4), Room 2 (seq 5..8)
        # Stairs: seq 9..10
        # Floor 1: Room 1 (seq 11..14)
        keyframes_spec = [
            *((0 + i, float(i), 0.0, 0.0, wide_pts, f"rgb/f0_r1_{i}.png") for i in range(4)),
            (4, 4.0, 0.0, 0.0, door_pts, None),
            *((5 + i, 5.0 + i, 0.0, 0.0, wide_pts, f"rgb/f0_r2_{i}.png") for i in range(4)),
            (9, 9.0, 0.5, 1.1, wide_pts, None),
            (10, 9.0, 1.0, 2.2, wide_pts, None),
            *((11 + i, 9.0 + i, 1.5, 3.3, wide_pts, f"rgb/f1_r1_{i}.png") for i in range(4)),
        ]

        for seq, x, y, z, pts, rgb_path in keyframes_spec:
            key = (1, seq)
            stamp_ns = (seq + 1) * 1_000_000_000
            T = _pose(x, y, z)
            kf = Keyframe(
                key=key,
                stamp_ns=stamp_ns,
                T_odom_body=T,
                T_body_lidar=np.eye(4, dtype=np.float64),
                points_lidar=pts,
            )
            store.add(kf, T, np.zeros(4, dtype=np.float64))
            if rgb_path is not None:
                rgb_index[stamp_ns] = rgb_path

            # Register two keyframes inside LayeredMap to verify anchor sync
            if seq in (1, 12):
                obs = Observation(
                    epoch=1,
                    sequence=seq,
                    source_ns=stamp_ns,
                    pose_source_ns=stamp_ns,
                    points_sensor=np.array([[0.0, 0.0, 0.5]], dtype=np.float64),
                    T_map_sensor=T,
                    anchor_id=f"1:{seq}",
                    frame_id="body",
                    calibration_id="cal:v1",
                )
                layered.integrate(
                    obs,
                    PointMask(
                        1,
                        seq,
                        stamp_ns,
                        stamp_ns,
                        "body",
                        "cal:v1",
                        np.array([STATIC], dtype=np.uint8),
                        {"kind": "geometric", "producer": "fixture"},
                        instance_ids=("marker",),
                        labels={"marker": "pillar"},
                    ),
                )

        sg = build_scene_graph_from_map_store(
            store, rgb_index=rgb_index, layered_map=layered
        )
        self.assertEqual(sorted(sg.floors.keys()), ["f0", "f1"])
        self.assertEqual(len(sg.stairs), 1)
        self.assertEqual(sorted(sg.rooms.keys()), ["f0_room_01", "f0_room_02", "f1_room_01"])
        self.assertEqual(len(sg.doorways), 1)

        obj = sg.add_object(
            "exit_sign",
            "1:12",
            local_center_xyz=[0.0, 0.0, 0.5],
            confidence=0.9,
            producer="fixture",
        )
        self.assertEqual(obj.room_id, "f1_room_01")

        # Apply loop-closure update from backend (shift keyframe (1, 12) by +0.75 m X)
        loop_poses = {key: store.poses[key].copy() for key in store.poses}
        loop_poses[(1, 12)] = _pose(10.75, 1.5, 3.3)
        sync_stats = sync_backend_loop_update(
            sg, loop_poses, layered_map=layered, revision=1
        )
        self.assertEqual(sync_stats["synced_anchors"], 2)
        np.testing.assert_allclose(obj.world_center_xyz, [10.75, 1.5, 3.8], atol=1e-6)
        np.testing.assert_allclose(layered.static_points()[-1], [10.75, 1.5, 3.8], atol=1e-6)

        # Exercise thin CLI build_semantic_scene_graph.py
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            map_dir = tmp / "map_store"
            store.save(map_dir)
            jsonl_path = tmp / "rgb_index.jsonl"
            jsonl_path.write_text(
                "\n".join(
                    json.dumps({"stamp_ns": s, "rgb_path": p})
                    for s, p in sorted(rgb_index.items())
                )
                + "\n",
                encoding="utf-8",
            )
            out_sg = tmp / "scene_graph.json"
            proc = subprocess.run(
                [
                    sys.executable,
                    str(REPO_ROOT / "scripts" / "build_semantic_scene_graph.py"),
                    "--map-dir",
                    str(map_dir),
                    "--rgb-index-jsonl",
                    str(jsonl_path),
                    "--output",
                    str(out_sg),
                    "--summary",
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            cli_summary = json.loads(proc.stdout)
            self.assertEqual(cli_summary["counts"]["floors"], 2)
            self.assertEqual(cli_summary["counts"]["rooms"], 3)
            self.assertTrue(out_sg.is_file())

    def test_continuous_ramp_and_drift_tail_do_not_create_spurious_floors(self):
        sg = KeyframeSceneGraph()
        idx = 0
        for i in range(10):
            sg.add_keyframe(f"1:{idx}", _pose(float(i), 0.0, 0.0), clearance_m=2.0)
            idx += 1
        for k in range(1, 20):
            z = 0.2 * k
            sg.add_keyframe(f"1:{idx}", _pose(10.0 + float(k), 0.0, z), clearance_m=1.2)
            idx += 1
        for i in range(10):
            sg.add_keyframe(f"1:{idx}", _pose(30.0 + float(i), 0.0, 4.0), clearance_m=2.0)
            idx += 1

        floors = sg.segment_floors()
        self.assertEqual(sorted(floors.keys()), ["f0", "f1"])
        self.assertLess(abs(floors["f0"].z_mean_m), 0.2)
        self.assertLess(abs(floors["f1"].z_mean_m - 4.0), 0.2)
        self.assertEqual(len(sg.stairs), 1)

        # Simulate gradual Z drift tail + stationary spin-in-place keyframes at the end.
        sg_drift = KeyframeSceneGraph()
        seq = 0
        for i in range(40):
            sg_drift.add_keyframe(f"1:{seq}", _pose(float(i) * 0.5, 0.0, 0.0), clearance_m=2.0)
            seq += 1
        for i in range(6):
            sg_drift.add_keyframe(f"1:{seq}", _pose(20.0 + i * 0.6, 0.0, 0.55 * (i + 1)), clearance_m=1.2)
            seq += 1
        for i in range(40):
            sg_drift.add_keyframe(f"1:{seq}", _pose(24.0 + i * 0.5, 0.0, 3.3), clearance_m=2.0)
            seq += 1
        # Monotonic drift (0.04 dz/ds) followed by 25 stationary spin-in-place frames at z=5.8
        for i in range(60):
            sg_drift.add_keyframe(
                f"1:{seq}", _pose(44.0 + i * 1.0, 0.0, 3.3 + 0.04 * i), clearance_m=2.0
            )
            seq += 1
        for i in range(25):
            sg_drift.add_keyframe(
                f"1:{seq}", _pose(104.0, 0.0, 5.7, yaw_rad=0.1 * i), clearance_m=2.0
            )
            seq += 1

        drift_floors = sg_drift.segment_floors()
        self.assertEqual(sorted(drift_floors.keys()), ["f0", "f1"])
        self.assertEqual(len(sg_drift.stairs), 1)

    def test_floor_segmentation_invariant_to_global_z_shift(self):
        for z_shift in (0.0, 0.10, -5.3):
            sg = KeyframeSceneGraph()
            for i in range(5):
                sg.add_keyframe(f"1:{i}", _pose(float(i), 0.0, 0.0 + z_shift), clearance_m=2.0)
            for i in range(5):
                sg.add_keyframe(f"1:{10 + i}", _pose(float(i), 2.0, 1.82 + z_shift), clearance_m=2.0)
            floors = sg.segment_floors()
            self.assertEqual(sorted(floors.keys()), ["f0", "f1"], f"failed at z_shift={z_shift}")
            self.assertEqual(len(floors["f0"].keyframe_ids), 5)
            self.assertEqual(len(floors["f1"].keyframe_ids), 5)

    def test_separate_epochs_and_time_gaps_never_create_stairs(self):
        sg = KeyframeSceneGraph()
        for i in range(3):
            sg.add_keyframe(
                f"1:{i + 1}",
                _pose(float(i), 0.0, 0.0),
                clearance_m=2.0,
                stamp_ns=(i + 1) * 1_000_000_000,
                epoch=1,
            )
        for i in range(3):
            sg.add_keyframe(
                f"2:{i + 1}",
                _pose(float(i), 0.0, 3.0),
                clearance_m=2.0,
                stamp_ns=(i + 1) * 1_000_000_000,
                epoch=2,
            )
        floors = sg.segment_floors()
        self.assertEqual(sorted(floors.keys()), ["f0", "f1"])
        self.assertEqual(sg.stairs, [])

        # Same epoch with large time gap should also avoid creating a stair edge
        sg_gap = KeyframeSceneGraph()
        for i in range(3):
            sg_gap.add_keyframe(
                f"1:{i}", _pose(float(i), 0.0, 0.0), clearance_m=2.0, stamp_ns=i * 1_000_000_000
            )
        for i in range(3):
            sg_gap.add_keyframe(
                f"1:{10 + i}",
                _pose(float(i), 0.0, 3.0),
                clearance_m=2.0,
                stamp_ns=100_000_000_000 + i * 1_000_000_000,
            )
        sg_gap.segment_floors()
        self.assertEqual(len(sg_gap.floors), 2)
        self.assertEqual(sg_gap.stairs, [])

    def test_doorway_anchors_persist_and_reproject_on_global_shift(self):
        sg = KeyframeSceneGraph()
        specs = [(-3.0, 2.0), (-2.0, 2.0), (0.0, 0.5), (2.0, 2.0), (3.0, 2.0)]
        for i, (x, clr) in enumerate(specs):
            sg.add_keyframe(f"1:{i}", _pose(x, 0.0, 0.0), clearance_m=clr)
        sg.segment_floors()
        sg.partition_rooms()
        self.assertEqual(len(sg.doorways), 1)
        self.assertEqual(sg.doorways[0].bottleneck_anchor_ids, ["1:2"])

        with tempfile.TemporaryDirectory() as tmpdir:
            p = Path(tmpdir) / "sg.json"
            sg.save(p)
            loaded = KeyframeSceneGraph.load(p)

        self.assertEqual(loaded.doorways[0].bottleneck_anchor_ids, ["1:2"])
        shifted = {
            f"1:{i}": _pose(x + 10.0, 0.0, 0.0) for i, (x, _) in enumerate(specs)
        }
        loaded.reproject_from_poses(shifted)
        self.assertEqual(len(loaded.doorways), 1)
        self.assertAlmostEqual(loaded.doorways[0].gateway_xyz[0], 10.0, places=6)

    def test_reproject_recomputes_floors_splits_rooms_assigns_new_kfs_and_is_idempotent(self):
        # 1) Z correction [0,0,0,3,3,3] -> [0,0,0,0,0,0] collapses floors and removes stair
        sg = KeyframeSceneGraph()
        for i in range(3):
            sg.add_keyframe(f"1:{i}", _pose(float(i), 0.0, 0.0), clearance_m=2.0)
        for i in range(3):
            sg.add_keyframe(f"1:{3 + i}", _pose(10.0 + float(i), 0.0, 3.0), clearance_m=2.0)
        sg.segment_floors()
        sg.partition_rooms()
        sg.rename_room("f1_room_01", "upper_lab")
        self.assertEqual(len(sg.floors), 2)
        self.assertEqual(len(sg.stairs), 1)

        sg.reproject_from_poses({f"1:{3 + i}": _pose(10.0 + float(i), 0.0, 0.0) for i in range(3)})
        self.assertEqual(len(sg.floors), 1)
        self.assertEqual(len(sg.stairs), 0)
        self.assertEqual(len(sg.rooms), 2)
        aliases = {r.alias for r in sg.rooms.values()}
        self.assertIn("upper_lab", aliases)

        # 2) Moving 2 frames of a room 20 m away splits the room and preserves primary alias
        sg_split = KeyframeSceneGraph()
        for i in range(4):
            sg_split.add_keyframe(f"1:{i}", _pose(float(i), 0.0, 0.0), clearance_m=2.0)
        sg_split.segment_floors()
        sg_split.partition_rooms()
        sg_split.rename_room("f0_room_01", "main_hall")
        sg_split.reproject_from_poses({"1:2": _pose(20.0, 0.0, 0.0), "1:3": _pose(21.0, 0.0, 0.0)})
        self.assertEqual(len(sg_split.rooms), 2)
        self.assertEqual(sg_split.rooms["f0_room_01"].alias, "main_hall")

        # 3) Passing new keyframes assigns them to floor and room
        sg_split.add_keyframe("1:4", _pose(1.5, 0.5, 0.0), clearance_m=2.0)
        sg_split.reproject_from_poses({})
        self.assertIn("1:4", sg_split.floors["f0"].keyframe_ids)
        self.assertIn("1:4", sg_split.rooms["f0_room_01"].anchor_keyframe_ids)

        # 4) Adjacent doorway-separated rooms do not merge on empty or identical reprojection
        sg_adj = KeyframeSceneGraph()
        for i, (x, clr) in enumerate([(-1.2, 2.0), (-0.6, 2.0), (0.0, 0.5), (0.6, 2.0), (1.2, 2.0)]):
            sg_adj.add_keyframe(f"1:{i}", _pose(x, 0.0, 0.0), clearance_m=clr)
        sg_adj.segment_floors()
        sg_adj.partition_rooms()
        self.assertEqual(len(sg_adj.rooms), 2)
        self.assertEqual(len(sg_adj.doorways), 1)
        noop = sg_adj.reproject_from_poses({})
        self.assertEqual(noop["merged_count"], 0)
        self.assertEqual(len(sg_adj.rooms), 2)
        self.assertEqual(len(sg_adj.doorways), 1)
        same = sg_adj.reproject_from_poses({"1:0": _pose(-1.2, 0.0, 0.0)})
        self.assertEqual(same["merged_count"], 0)
        self.assertEqual(len(sg_adj.doorways), 1)

    def test_corridor_doorways_only_connect_adjacent_rooms_abc(self):
        sg = KeyframeSceneGraph()
        # Room A (x=0..2), corridor (x=3..5), Room B (x=6..8), corridor (x=9..11), Room C (x=12..14)
        coords = [
            (0.0, 2.0), (1.0, 2.0), (2.0, 2.0),
            (3.0, 0.5), (4.0, 0.5), (5.0, 0.5),
            (6.0, 2.0), (7.0, 2.0), (8.0, 2.0),
            (9.0, 0.5), (10.0, 0.5), (11.0, 0.5),
            (12.0, 2.0), (13.0, 2.0), (14.0, 2.0),
        ]
        for i, (x, clr) in enumerate(coords):
            sg.add_keyframe(f"1:{i}", _pose(x, 0.0, 0.0), clearance_m=clr)
        sg.segment_floors()
        sg.partition_rooms(doorway_clearance_m=0.85, link_radius_m=2.5, min_room_keyframes=2)
        self.assertEqual(sorted(sg.rooms.keys()), ["f0_room_01", "f0_room_02", "f0_room_03"])
        pairs = {(d.room_a, d.room_b) for d in sg.doorways}
        self.assertEqual(pairs, {("f0_room_01", "f0_room_02"), ("f0_room_02", "f0_room_03")})
        self.assertNotIn(("f0_room_01", "f0_room_03"), pairs)

    def test_parallel_corridors_separated_by_wall_remain_separate_rooms(self):
        # 1) Scalar clearance case: 2.2 m apart, each 1.1 m clearance, separate epochs/branches
        sg = KeyframeSceneGraph()
        for i in range(5):
            sg.add_keyframe(f"1:{i}", _pose(float(i), -1.1, 0.0), clearance_m=1.1, epoch=1)
        for i in range(5):
            sg.add_keyframe(f"2:{i}", _pose(float(i), 1.1, 0.0), clearance_m=1.1, epoch=2)
        sg.segment_floors()
        rooms = sg.partition_rooms(doorway_clearance_m=0.85, link_radius_m=2.5, min_room_keyframes=2)
        self.assertEqual(len(rooms), 2)

        # 2) Point-cloud case via MapStore with actual corridor wall at y=0
        corridor_pts = _box_points(4.0, 1.1, n_per_side=24)
        store = MapStore(voxel_m=0.05)
        for ep, y in ((1, -1.1), (2, 1.1)):
            for seq in range(5):
                T = _pose(float(seq), y, 0.0)
                kf = Keyframe((ep, seq), (ep * 10 + seq) * 1_000_000_000, T, np.eye(4), corridor_pts)
                store.add(kf, T, np.zeros(4))
        sg_cloud = build_scene_graph_from_map_store(store)
        self.assertEqual(len(sg_cloud.rooms), 2)

    def test_unknown_clearance_does_not_bridge_rooms(self):
        sg = KeyframeSceneGraph()
        for i in range(3):
            sg.add_keyframe(f"1:{i}", _pose(float(i), 0.0, 0.0), clearance_m=2.0)
        # Missing scan at x=3.5 -> clearance unknown (None)
        sg.add_keyframe("1:3", _pose(3.5, 0.0, 0.0), clearance_m=None)
        for i in range(3):
            sg.add_keyframe(f"1:{4 + i}", _pose(5.0 + float(i), 0.0, 0.0), clearance_m=2.0)
        sg.segment_floors()
        rooms = sg.partition_rooms(doorway_clearance_m=0.85, link_radius_m=2.5, min_room_keyframes=2)
        self.assertEqual(len(rooms), 2)

    def test_atomic_sync_revision_monotonicity_anchor_mapping_and_extrinsics(self):
        sg = KeyframeSceneGraph()
        for i in range(3):
            sg.add_keyframe(f"1:{i}", _pose(float(i), 0.0, 0.0), clearance_m=2.0)
        sg.segment_floors()
        sg.partition_rooms()

        layered = LayeredMap(map_id="ext_test")
        T_kf_sensor = _pose(0.2, -0.1, 0.3, yaw_rad=np.pi / 2)
        T_kf0 = _pose(1.0, 0.0, 0.0)
        T_sensor_init = T_kf0 @ T_kf_sensor
        obs = Observation(
            epoch=1,
            sequence=1,
            source_ns=1_000_000_000,
            pose_source_ns=1_000_000_000,
            points_sensor=np.array([[1.0, 0.0, 0.0]], dtype=np.float64),
            T_map_sensor=T_sensor_init,
            anchor_id="rgbd:1",
            frame_id="camera",
            calibration_id="cal:v1",
        )
        layered.integrate(
            obs,
            PointMask(
                1, 1, 1_000_000_000, 1_000_000_000, "camera", "cal:v1",
                np.array([STATIC], dtype=np.uint8),
                {"kind": "geometric", "producer": "fixture"},
            ),
        )

        # Sync 1 (default revision=None -> 1)
        poses_1 = {"1:1": _pose(2.0, 0.0, 0.0)}
        res1 = sync_backend_loop_update(
            sg,
            poses_1,
            layered_map=layered,
            anchor_to_keyframe_map={"rgbd:1": (1, 1)},
            T_keyframe_sensor=T_kf_sensor,
        )
        self.assertEqual(res1["revision"], 1)
        self.assertEqual(res1["synced_anchors"], 1)

        # Sync 2 (default revision=None -> auto-increments to 2)
        poses_2 = {"1:1": _pose(3.0, 0.0, 0.0)}
        res2 = sync_backend_loop_update(sg, poses_2, layered_map=layered)
        self.assertEqual(res2["revision"], 2)
        self.assertEqual(res2["synced_anchors"], 1)
        expected_pt = (poses_2["1:1"] @ T_kf_sensor @ np.array([1.0, 0.0, 0.0, 1.0]))[:3]
        np.testing.assert_allclose(layered.static_points()[0], expected_pt, atol=1e-6)

        # Idempotent redelivery at identical revision=2 with identical pose
        res_dup = sync_backend_loop_update(sg, poses_2, layered_map=layered, revision=2)
        self.assertEqual(res_dup["synced_anchors"], 0)

        # Stale revision=1 must fail atomically BEFORE touching sg or layered
        poses_3 = {"1:1": _pose(99.0, 0.0, 0.0)}
        with self.assertRaises(ValueError):
            sync_backend_loop_update(sg, poses_3, layered_map=layered, revision=1)
        self.assertAlmostEqual(sg.keyframe_poses["1:1"][0, 3], 3.0, places=6)
        self.assertEqual(layered.anchors["rgbd:1"][1], 2)

    def test_streaming_map_dir_subdirectories_and_rgb_index_resilience(self):
        wide_pts = _box_points(2.5, 2.5)
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            map_dir = tmp / "map_sub"
            (map_dir / "keyframes").mkdir(parents=True)
            rows = []
            for seq in range(4):
                T = _pose(float(seq), 0.0, 0.0)
                rel_name = f"keyframes/kf_1_{seq}.npz"
                np.savez(map_dir / rel_name, points_lidar=wide_pts, descriptor=np.zeros(4))
                rows.append(
                    {
                        "key": [1, seq],
                        "stamp_ns": (seq + 1) * 1_000_000_000,
                        "T_odom_body": T.tolist(),
                        "T_world_body": T.tolist(),
                        "T_body_lidar": np.eye(4).tolist(),
                        "file": rel_name,
                    }
                )
            (map_dir / "map.json").write_text(
                json.dumps({"schema": "embodied_navigation_map/v1", "voxel_m": 0.1, "keyframes": rows})
            )

            # MapStore.load and streaming build_scene_graph_from_map_dir both support subdirs
            loaded_store = MapStore.load(map_dir)
            self.assertEqual(len(loaded_store.keyframes), 4)

            # Create archived RGB directory with producer fields + corrupt lines + NaN timestamp
            rgb_dir = tmp / "archived_frames"
            rgb_dir.mkdir()
            for seq in range(4):
                (rgb_dir / f"rgb_{seq:06d}.png").write_bytes(b"png")
            jsonl = rgb_dir / "index.jsonl"
            jsonl.write_text(
                "\n".join(
                    [
                        json.dumps({"source_time_s": 1.0, "path": f"/old/machine/rgb_000000.png"}),
                        "{corrupt json",
                        json.dumps({"source_time_s": "NaN", "path": "/old/machine/rgb_000001.png"}),
                        json.dumps({"stamp_s": 2.0, "path": f"/old/machine/rgb_000001.png"}),
                        json.dumps({"source_time_s": 3.0, "path": "/old/machine/missing.png"}),
                        json.dumps({"source_time_s": 4.0, "path": f"/old/machine/rgb_000003.png"}),
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            report = load_rgb_index_report(jsonl)
            self.assertEqual(report.skipped_count, 3)
            self.assertEqual([e["line"] for e in report.skipped_lines], [2, 3, 5])
            self.assertEqual(report.loaded_count, 3)

            sg = build_scene_graph_from_map_dir(map_dir, rgb_index=report.index)
            self.assertEqual(len(sg.rooms), 1)
            self.assertEqual(len(sg.rooms["f0_room_01"].representative_rgb_paths), 3)

            # Reject path traversal outside map_dir
            rows[0]["file"] = "../escape.npz"
            (map_dir / "map.json").write_text(
                json.dumps({"schema": "embodied_navigation_map/v1", "voxel_m": 0.1, "keyframes": rows})
            )
            with self.assertRaises(ValueError):
                MapStore.load(map_dir)

    def test_validation_integrity_and_spatial_query_filtering(self):
        sg = KeyframeSceneGraph()
        for i in range(4):
            sg.add_keyframe(f"1:{i}", _pose(float(i), 0.0, 0.0), clearance_m=2.0)
        sg.add_keyframe("1:4", _pose(4.0, 0.0, 0.0), clearance_m=0.5)
        for i in range(4):
            sg.add_keyframe(f"1:{5 + i}", _pose(5.0 + float(i), 0.0, 0.0), clearance_m=2.0)
        sg.add_keyframe("1:9", _pose(9.0, 0.0, 1.5), clearance_m=1.0)
        for i in range(4):
            sg.add_keyframe(f"1:{10 + i}", _pose(10.0 + float(i), 0.0, 3.0), clearance_m=2.0)
        sg.segment_floors()
        sg.partition_rooms()

        # Non-rigid scaling matrix must be rejected
        scaled = np.eye(4, dtype=np.float64)
        scaled[0, 0] = 2.0
        with self.assertRaises(ValueError):
            sg.add_keyframe("1:99", scaled)

        # Dangling room_id must be rejected by add_object
        with self.assertRaises(ValueError):
            sg.add_object("chair", "1:0", [0.0, 0.0, 0.0], room_id="missing_room")

        # Dangling object anchor in snapshot must be rejected by from_dict
        bad_dict = sg.to_dict()
        bad_dict["objects"].append(
            {
                "object_id": "obj_bad",
                "label": "lamp",
                "anchor_keyframe_id": "missing_kf",
                "local_center_xyz": [0.0, 0.0, 0.0],
                "local_extent_xyz": [0.1, 0.1, 0.1],
                "world_center_xyz": [0.0, 0.0, 0.0],
                "confidence": 0.9,
                "producer": "test",
                "room_id": None,
            }
        )
        with self.assertRaises(ValueError):
            KeyframeSceneGraph.from_dict(bad_dict)

        # Far spatial query returns empty rooms, doorways, stairs, and objects
        far = sg.query(near_xyz=[1000.0, 0.0, 0.0], radius_m=1.0)
        self.assertEqual(far["rooms"], [])
        self.assertEqual(far["doorways"], [])
        self.assertEqual(far["stairs"], [])
        self.assertEqual(far["objects"], [])

        # Local spatial query finds doorway or stair near its geometry
        near_door = sg.query(near_xyz=[4.0, 0.0, 0.0], radius_m=0.5)
        self.assertEqual(len(near_door["doorways"]), 1)
        self.assertEqual(len(near_door["stairs"]), 0)
        near_stair = sg.query(near_xyz=[9.0, 0.0, 1.5], radius_m=0.5)
        self.assertEqual(len(near_stair["stairs"]), 1)
        self.assertEqual(len(near_stair["doorways"]), 0)


if __name__ == "__main__":
    unittest.main()
