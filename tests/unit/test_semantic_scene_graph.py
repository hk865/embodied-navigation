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
    build_scene_graph_from_map_store,
    estimate_keyframe_clearance_m,
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
        self.assertEqual(len(kept.representative_rgb_paths), 6)
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
        self.assertEqual(estimate_keyframe_clearance_m(np.empty((0, 3))), 5.0)

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


if __name__ == "__main__":
    unittest.main()
