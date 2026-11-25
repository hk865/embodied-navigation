"""CPU unit tests for Habitat-Sim HM3D coordinate bridge, depth raycast, and occupancy grid."""

from __future__ import annotations

import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    HierarchicalMemoryStore,
    KeyframeSceneGraph,
)
from embodied_navigation.planning.exploration.habitat_memory_runner import (
    DepthOccupancyGrid,
    _dedup_viewpoints,
    compute_spl,
    depth_bearing_to_local_xy,
    hab_quat_to_mem_yaw,
    hab_to_mem,
    is_geodesic_target_reached,
    measure_depth_bearing_patch,
    measure_depth_bearing_range,
    mem_to_hab,
)
from embodied_navigation.planning.exploration.streaming_task_agent import (
    parse_task_decision,
    resolve_thinking_options,
)


class TestHabitatMemoryRunnerPrimitives(unittest.TestCase):
    def test_coordinate_roundtrip_and_quat_yaw(self) -> None:
        pt_hab = (1.5, 2.8, -4.2)
        pt_mem = hab_to_mem(pt_hab)
        self.assertEqual(pt_mem, (1.5, 4.2, 2.8))
        self.assertEqual(mem_to_hab(pt_mem), pt_hab)

        # Identity quaternion in Habitat faces -Z -> +Y in Z-up memory -> yaw = +pi/2
        yaw0 = hab_quat_to_mem_yaw((1.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(yaw0, math.pi / 2.0, places=5)

        # Rotate +90 deg around Habitat +Y (left turn) -> faces -X -> yaw = pi
        c45, s45 = math.cos(math.pi / 4.0), math.sin(math.pi / 4.0)
        yaw_left = hab_quat_to_mem_yaw((c45, 0.0, s45, 0.0))
        self.assertAlmostEqual(abs(yaw_left), math.pi, places=5)

    def test_measure_depth_bearing_range_and_perspective_projection(self) -> None:
        # 100x100 planar depth map: left half is 1.2 m, right half is 4.0 m
        depth = np.full((100, 100), 4.0, dtype=np.float32)
        depth[:, :50] = 1.2
        d_left = measure_depth_bearing_range(depth, bearing_deg=25.0, hfov_deg=90.0)
        d_right = measure_depth_bearing_range(depth, bearing_deg=-25.0, hfov_deg=90.0)
        self.assertIsNotNone(d_left)
        self.assertIsNotNone(d_right)
        self.assertAlmostEqual(d_left, 1.2, places=2)
        self.assertAlmostEqual(d_right, 4.0, places=2)
        # Out-of-HFOV bearing must return None (never clamp to image border)
        self.assertIsNone(measure_depth_bearing_range(depth, bearing_deg=55.0, hfov_deg=90.0))
        self.assertIsNone(measure_depth_bearing_range(depth, bearing_deg=-60.0, hfov_deg=90.0))

        # Perspective local projection: x = d, y = d * tan(b) -> at 45 deg, (2.0, 2.0)
        lx, ly = depth_bearing_to_local_xy(2.0, 45.0)
        self.assertAlmostEqual(lx, 2.0, places=5)
        self.assertAlmostEqual(ly, 2.0, places=5)
        self.assertAlmostEqual(math.hypot(lx, ly), 2.0 * math.sqrt(2.0), places=5)

    def test_depth_occupancy_grid_updates_and_extracts_frontiers(self) -> None:
        grid = DepthOccupancyGrid(resolution_m=0.10, max_depth_m=5.0)
        # Open corridor ahead (wall at 4.0 m) -> carves free space and produces frontier(s)
        depth = np.full((128, 128), 4.0, dtype=np.float32)
        grid.update(depth, [0.0, 0.0, 0.0], yaw_rad=0.0, stride=4)
        frontiers = grid.extract_frontiers([0.0, 0.0, 0.0], max_candidates=4, min_cluster_cells=2)
        self.assertGreaterEqual(len(frontiers), 1)
        self.assertEqual(frontiers[0]["id"], "F1")
        self.assertEqual(frontiers[0]["kind"], "frontier")

        with tempfile.TemporaryDirectory() as tmp:
            out_png = grid.render_local_map(
                Path(tmp) / "local_map.png", [0.0, 0.0, 0.0], 0.0, frontiers
            )
            self.assertTrue(out_png.is_file())
            self.assertGreater(out_png.stat().st_size, 1000)

    def test_visited_trail_suppresses_ping_pong_frontiers(self) -> None:
        grid = DepthOccupancyGrid(resolution_m=0.10, max_depth_m=5.0)
        depth = np.full((128, 128), 4.0, dtype=np.float32)
        # Explore east (+X) and north (+Y) to create multiple distinct frontier clusters
        grid.update(depth, [0.0, 0.0, 0.0], yaw_rad=0.0, stride=4)
        grid.update(depth, [0.0, 0.0, 0.0], yaw_rad=math.pi / 2.0, stride=4)
        initial = grid.extract_frontiers(
            [0.0, 0.0, 0.0], max_candidates=6, min_cluster_cells=2, visited_suppress_m=0.0
        )
        self.assertGreaterEqual(len(initial), 2)
        f1_xy = initial[0]["xy"]

        # Record a visited trail point directly on top of F1's centroid -> F1 must be suppressed
        grid._visited_trail.append((float(f1_xy[0]), float(f1_xy[1]), 0.0))
        filtered = grid.extract_frontiers(
            [0.0, 0.0, 0.0], max_candidates=6, min_cluster_cells=2, visited_suppress_m=0.9
        )
        self.assertTrue(filtered)
        for cand in filtered:
            dist = math.hypot(cand["xy"][0] - f1_xy[0], cand["xy"][1] - f1_xy[1])
            self.assertGreaterEqual(dist, 0.9)

    def test_geodesic_target_reached_spl_and_viewpoint_dedup(self) -> None:
        # Both geodesic viewpoint tol (<= 1.0 m) AND Euclidean <= stop_radius required
        self.assertTrue(is_geodesic_target_reached(0.5, 1.5, 1.8))
        # Through-wall small Euclidean with large geodesic must FAIL
        self.assertFalse(is_geodesic_target_reached(4.2, 1.1, 1.8))
        # Farther than promised stop_radius must FAIL even if close to a viewpoint
        self.assertFalse(is_geodesic_target_reached(0.2, 2.5, 1.8))
        self.assertFalse(is_geodesic_target_reached(float("inf"), 1.0, 1.8))

        # Honest SPL: no inflation on zero-optimal detour or inf optimal
        self.assertAlmostEqual(compute_spl(True, 4.0, 8.0), 0.5, places=5)
        self.assertAlmostEqual(compute_spl(True, 4.0, 2.0), 1.0, places=5)
        self.assertEqual(compute_spl(False, 4.0, 4.0), 0.0)
        self.assertEqual(compute_spl(True, float("inf"), 5.0), 0.0)
        self.assertEqual(compute_spl(True, 0.0, 3.0), 0.0)
        self.assertEqual(compute_spl(True, 0.0, 0.0), 1.0)

        vps = _dedup_viewpoints(
            [
                (0.0, 0.0, 0.0),
                (0.1, 0.0, 0.1),  # duplicate within 0.4 m
                (1.0, 0.0, 0.0),  # kept
                (0.0, 2.5, 0.0),  # different floor -> kept
            ],
            min_spacing_m=0.4,
        )
        self.assertEqual(len(vps), 3)

    def test_resolve_thinking_options_allocates_30k_reasoning_budget(self) -> None:
        extra_h, t_h, tok_h = resolve_thinking_options("deepseek-v4.1-flash", "high")
        self.assertEqual(extra_h, {"reasoning_effort": "high"})
        self.assertGreaterEqual(t_h, 180.0)
        self.assertGreaterEqual(tok_h, 30000)

        extra_m, t_m, tok_m = resolve_thinking_options("deepseek-v4.1-flash", "medium")
        self.assertEqual(extra_m, {"reasoning_effort": "medium"})
        self.assertGreaterEqual(t_m, 120.0)
        self.assertGreaterEqual(tok_m, 30000)

        extra_q, t_q, tok_q = resolve_thinking_options("qwen3.8-flash", "disabled", base_timeout_s=20.0)
        self.assertEqual(extra_q, {"enable_thinking": False})
        self.assertEqual(t_q, 20.0)
        self.assertEqual(tok_q, 380)

    def test_measure_depth_bearing_patch_and_rich_decision_parsing(self) -> None:
        depth = np.full((100, 100), 2.5, dtype=np.float32)
        res = measure_depth_bearing_patch(depth, bearing_deg=0.0, hfov_deg=90.0)
        self.assertIsNotNone(res)
        d_m, ext = res
        self.assertAlmostEqual(d_m, 2.5, places=2)
        self.assertEqual(len(ext), 3)
        self.assertTrue(all(v > 0.0 for v in ext))
        self.assertIsNone(measure_depth_bearing_patch(depth, bearing_deg=60.0, hfov_deg=90.0))

        raw = (
            '{"action":"goto_candidate","target_id":"F1","room_type":"living room",'
            '"scene_description":"wooden coffee table with small black phone and glass vase",'
            '"visible_surfaces":["coffee table","couch"],'
            '"observed_objects":[{"label":"vase","desc":"narrow clear glass flower vase",'
            '"on":"coffee table","attrs":["glass","narrow"],"dist_m":1.5,"bearing_deg":10.0}],'
            '"reason":"explore living room"}'
        )
        dec = parse_task_decision(raw, valid_candidate_ids=["F1"])
        self.assertFalse(dec.fallback)
        self.assertEqual(dec.room_type, "living room")
        self.assertIn("phone", dec.scene_description)
        self.assertEqual(dec.visible_surfaces, ("coffee table", "couch"))
        self.assertEqual(dec.observed_objects[0]["desc"], "narrow clear glass flower vase")
        self.assertEqual(dec.observed_objects[0]["on"], "coffee table")
        self.assertEqual(dec.observed_objects[0]["attrs"], ["glass", "narrow"])

    def test_rich_indexing_3d_fusion_coarse_to_fine_and_current_frame_stop(self) -> None:
        graph = KeyframeSceneGraph()
        p1 = np.eye(4, dtype=np.float64)
        p2 = np.eye(4, dtype=np.float64)
        p2[:3, 3] = [4.0, 0.0, 0.0]
        p3 = np.eye(4, dtype=np.float64)
        p3[:3, 3] = [0.2, 0.0, 0.0]
        graph.add_keyframe("kf1", p1, rgb_path="/tmp/kf1.png")
        graph.add_keyframe("kf2", p2, rgb_path="/tmp/kf2.png")
        graph.add_keyframe("kf3", p3, rgb_path="/tmp/kf3.png")
        mem = HierarchicalMemoryStore(graph, default_stop_radius_m=1.8)

        mem.annotate_keyframe(
            "kf1",
            room_type="living room",
            scene_description="wooden coffee table with small black smartphone and clear vase",
            surfaces=("coffee table", "couch"),
            embedding=[1.0, 0.0, 0.0],
        )
        mem.annotate_keyframe(
            "kf2",
            room_type="kitchen",
            scene_description="countertop with blue plastic water bottle",
            surfaces=("countertop",),
            embedding=[0.0, 1.0, 0.0],
        )

        # 3D multi-view fusion: two close observations of the same vase fuse into 1 instance
        n1, fused1 = mem.add_or_fuse_object(
            label="vase",
            keyframe_id="kf1",
            local_xyz=[1.0, 0.0, 0.0],
            local_extent_xyz=(0.15, 0.15, 0.3),
            producer="vlm_depth_grounded",
            range_source="habitat_depth_raycast",
            description="clear narrow glass flower vase",
            attributes=("clear", "glass"),
            spatial_context="coffee table",
            embedding=[0.9, 0.1, 0.0],
        )
        self.assertFalse(fused1)
        n1_b, fused2 = mem.add_or_fuse_object(
            label="glass vase",
            keyframe_id="kf3",
            local_xyz=[0.9, 0.05, 0.0],
            local_extent_xyz=(0.16, 0.14, 0.32),
            producer="vlm_depth_grounded",
            range_source="habitat_depth_raycast",
            description="clear narrow glass flower vase with dried flowers",
            embedding=[0.95, 0.05, 0.0],
        )
        self.assertTrue(fused2)
        self.assertEqual(n1.object_id, n1_b.object_id)
        self.assertEqual(mem._object_meta[n1.object_id].observation_count, 2)

        # Unverified guess on a new frame must NOT spoof current-frame STOP confirmation
        p4 = np.eye(4, dtype=np.float64)
        p4[:3, 3] = [0.5, 0.0, 0.0]
        graph.add_keyframe("kf4", p4, rgb_path="/tmp/kf4.png")
        _, fused_unver = mem.add_or_fuse_object(
            label="vase",
            keyframe_id="kf4",
            local_xyz=[0.5, 0.0, 0.0],
            producer="opencode_vlm_unverified",
            range_source="vlm_unverified_guess",
        )
        self.assertTrue(fused_unver)
        ver_old = mem.verify_stop_claim(
            [0.5, 0.0, 0.0],
            target_label="vase",
            stop_radius_m=1.8,
            require_current_keyframe_id="kf4",
            excluded_producers=("opencode_vlm", "opencode_vlm_unverified"),
        )
        self.assertFalse(ver_old.allowed)
        self.assertEqual(ver_old.reason, "requires_current_frame_visual_confirmation")

        # Once verified in kf3, current-frame STOP succeeds
        ver_now = mem.verify_stop_claim(
            [0.2, 0.0, 0.0],
            target_label="vase",
            stop_radius_m=1.8,
            require_current_keyframe_id="kf3",
            excluded_producers=("opencode_vlm", "opencode_vlm_unverified"),
        )
        self.assertTrue(ver_now.allowed)

        # Single clear multi-view match -> high confidence, bypasses L3
        res_clear = mem.coarse_to_fine_search("glass vase", robot_xyz=[0.0, 0.0, 0.0])
        self.assertEqual(len(res_clear["top_objects"]), 1)
        self.assertFalse(res_clear["needs_l3_verification"])

        # Add a second competing vase in another room -> low margin -> triggers selective L3
        mem.add_or_fuse_object(
            label="vase",
            keyframe_id="kf2",
            local_xyz=[1.0, 0.0, 0.0],
            producer="vlm_depth_grounded",
            description="tall ceramic vase",
        )
        res_ambig = mem.coarse_to_fine_search("vase", robot_xyz=[2.0, 0.0, 0.0])
        self.assertEqual(len(res_ambig["top_objects"]), 2)
        self.assertTrue(res_ambig["needs_l3_verification"])
        self.assertGreaterEqual(len(res_ambig["l3_verify_rgb_paths"]), 2)

        # Unsegmented small object ('smartphone') -> recalled via rich keyframe description & surfaces
        res_small = mem.coarse_to_fine_search("smartphone", robot_xyz=[0.0, 0.0, 0.0])
        self.assertEqual(len(res_small["top_objects"]), 0)
        self.assertTrue(res_small["needs_l3_verification"])
        self.assertEqual(res_small["top_keyframes"][0]["keyframe_id"], "kf1")
        self.assertIn("/tmp/kf1.png", res_small["l3_verify_rgb_paths"])

        # Persistence roundtrip preserves keyframe_meta and object rich fields
        restored = HierarchicalMemoryStore.from_dict(mem.to_dict())
        self.assertIn("kf1", restored._keyframe_meta)
        self.assertEqual(restored._keyframe_meta["kf1"].surfaces, ("coffee table", "couch"))
        self.assertEqual(restored._object_meta[n1.object_id].observation_count, 2)

    def test_astra_audit_stop_current_measurement_and_nonrigid_reprojection(self) -> None:
        graph = KeyframeSceneGraph()
        p1 = np.eye(4, dtype=np.float64)
        p2 = np.eye(4, dtype=np.float64)
        graph.add_keyframe("kf1", p1, rgb_path="/tmp/kf1.png")
        graph.add_keyframe("kf2", p2, rgb_path="/tmp/kf2.png")
        mem = HierarchicalMemoryStore(graph, default_stop_radius_m=1.5)

        # P1-1 & P1-2: Historical observation at 1.1 m with finer label 'glass vase'
        n, _ = mem.add_or_fuse_object(
            label="glass vase",
            keyframe_id="kf1",
            local_xyz=[1.1, 0.0, 0.0],
            producer="vlm_depth_grounded",
        )
        # Current frame kf2 sees coarser 'vase' at 1.8 m (within 0.75 m fusion radius, fused mean=1.45 m)
        n2, fused = mem.add_or_fuse_object(
            label="vase",
            keyframe_id="kf2",
            local_xyz=[1.8, 0.0, 0.0],
            producer="vlm_depth_grounded",
        )
        self.assertTrue(fused)
        self.assertEqual(n.object_id, n2.object_id)

        # P1-1: STOP at 1.5 m for 'vase' MUST fail because kf2 measured 1.8 m (> 1.5 m), NOT fused 1.45 m
        ver_dist = mem.verify_stop_claim(
            [0.0, 0.0, 0.0],
            target_label="vase",
            stop_radius_m=1.5,
            require_current_keyframe_id="kf2",
        )
        self.assertFalse(ver_dist.allowed)
        self.assertEqual(ver_dist.reason, "target_beyond_stop_radius")

        # P1-2: STOP at 2.0 m for 'glass vase' MUST fail because kf2 only observed coarser 'vase'
        ver_lbl = mem.verify_stop_claim(
            [0.0, 0.0, 0.0],
            target_label="glass vase",
            stop_radius_m=2.0,
            require_current_keyframe_id="kf2",
        )
        self.assertFalse(ver_lbl.allowed)
        self.assertEqual(ver_lbl.reason, "requires_current_frame_visual_confirmation")

        # P2-2: Duplicate same-frame observation does NOT inflate observation_count
        mem.add_or_fuse_object(
            label="vase",
            keyframe_id="kf2",
            local_xyz=[1.8, 0.0, 0.0],
            producer="vlm_depth_grounded",
        )
        self.assertEqual(mem._object_meta[n.object_id].observation_count, 2)

        # P2-1: Non-rigid loop update moving only kf1 by +1.0 m moves fused mean by +0.5 m (1.45 -> 1.95)
        p1_shifted = np.eye(4, dtype=np.float64)
        p1_shifted[:3, 3] = [1.0, 0.0, 0.0]
        mem.sync_poses({"kf1": p1_shifted, "kf2": p2})
        self.assertAlmostEqual(mem.graph.objects[n.object_id].world_center_xyz[0], 1.95, places=3)

        # P2-3: top_k_objects=1 still detects competing instance & unverified single guess triggers L3
        p3 = np.eye(4, dtype=np.float64)
        p3[:3, 3] = [5.0, 0.0, 0.0]
        graph.add_keyframe("kf3", p3, rgb_path="/tmp/kf3.png")
        mem.add_or_fuse_object(
            label="vase",
            keyframe_id="kf3",
            local_xyz=[1.0, 0.0, 0.0],
            producer="vlm_depth_grounded",
        )
        res_k1 = mem.coarse_to_fine_search("vase", robot_xyz=[3.0, 0.0, 0.0], top_k_objects=1)
        self.assertEqual(len(res_k1["top_objects"]), 1)
        self.assertTrue(res_k1["needs_l3_verification"])

        mem.add_or_fuse_object(
            label="lamp",
            keyframe_id="kf3",
            local_xyz=[1.0, 0.0, 0.0],
            producer="opencode_vlm_unverified",
            confidence=0.9,
        )
        res_unver = mem.coarse_to_fine_search("lamp", robot_xyz=[5.0, 0.0, 0.0])
        self.assertTrue(res_unver["needs_l3_verification"])


if __name__ == "__main__":
    unittest.main()
