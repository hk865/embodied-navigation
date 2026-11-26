"""Unit tests for surface-visibility texture keyframes, Set-Cover pruning, and GlobalRegionMemory."""

from __future__ import annotations

import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    GlobalRegionMemory,
    HierarchicalMemoryStore,
    KeyframeSceneGraph,
    TextureKeyframeSelector,
    extract_pointcloud_surface_voxels,
    extract_rgbd_surface_voxels,
)
from embodied_navigation.planning.exploration.habitat_memory_runner import (
    DepthOccupancyGrid,
    _pose_matrix_zup,
)


class TestTextureKeyframeSelector(unittest.TestCase):
    def test_online_marginal_coverage_upgrade_and_set_cover_pruning(self) -> None:
        sel = TextureKeyframeSelector(
            voxel_m=0.20,
            min_marginal_gain=0.16,
            min_new_voxels=12,
            quality_upgrade_ratio=1.45,
            set_cover_target=0.92,
            max_keyframes_per_region=4,
        )
        # Frame 1: looking along +X at a wall 3.5 m away
        d_far = np.full((64, 64), 3.5, dtype=np.float32)
        T0 = _pose_matrix_zup([0.0, 0.0, 0.0], 0.0)
        v1 = extract_rgbd_surface_voxels(d_far, T0, voxel_m=0.20, stride=4)
        self.assertGreater(len(v1), 20)
        dec1 = sel.evaluate_voxels(v1)
        self.assertTrue(dec1.admit_texture)
        self.assertEqual(dec1.reason, "initial_texture_seed")
        sel.commit_candidate("kf_1", v1, pose_xyz=[0.0, 0.0, 0.0], region_hint="R1")

        # Frame 2: almost identical pose (slight 0.05m shift) -> redundant, rejected!
        T_same = _pose_matrix_zup([0.05, 0.0, 0.0], 0.02)
        v2 = extract_rgbd_surface_voxels(d_far, T_same, voxel_m=0.20, stride=4)
        dec2 = sel.evaluate_voxels(v2)
        self.assertFalse(dec2.admit_texture)
        self.assertEqual(dec2.reason, "redundant_surface_view")

        # Frame 3: approach the SAME wall from 3.5m -> 1.2m (huge quality upgrade!) -> admitted
        d_close = np.full((64, 64), 1.2, dtype=np.float32)
        T_close = _pose_matrix_zup([2.3, 0.0, 0.0], 0.0)
        v3 = extract_rgbd_surface_voxels(d_close, T_close, voxel_m=0.20, stride=4)
        dec3 = sel.evaluate_voxels(v3)
        self.assertTrue(dec3.admit_texture)
        self.assertIn(dec3.reason, ("closeup_quality_upgrade", "new_surface_coverage"))
        sel.commit_candidate("kf_3", v3, pose_xyz=[2.3, 0.0, 0.0], region_hint="R1")

        # Frame 4: turn 90 deg left to look at a brand-new wall along +Y -> admitted
        T_left = _pose_matrix_zup([0.0, 0.0, 0.0], math.pi * 0.5)
        v4 = extract_rgbd_surface_voxels(np.full((64, 64), 2.0, dtype=np.float32), T_left, voxel_m=0.20, stride=4)
        dec4 = sel.evaluate_voxels(v4)
        self.assertTrue(dec4.admit_texture)
        self.assertEqual(dec4.reason, "new_surface_coverage")
        sel.commit_candidate("kf_4", v4, pose_xyz=[0.0, 0.0, 0.0], region_hint="R1")

        # Delayed Set-Cover pruning on synthetic overlapping pool:
        # kf_full covers voxels 0..99 at quality 1.0; kf_part1 covers 0..40 at 0.5; kf_part2 covers 40..80 at 0.5;
        # kf_side covers 100..130 at 1.0 -> Set-Cover should pick ONLY {kf_full, kf_side} and prune both partials!
        sel2 = TextureKeyframeSelector(set_cover_target=0.95, max_keyframes_per_region=5)
        sel2.commit_candidate("kf_part1", {(i, 0, 0): 0.5 for i in range(0, 41)}, pose_xyz=[0, 0, 0])
        sel2.commit_candidate("kf_part2", {(i, 0, 0): 0.5 for i in range(40, 81)}, pose_xyz=[0.5, 0, 0])
        sel2.commit_candidate("kf_full", {(i, 0, 0): 1.0 for i in range(0, 100)}, pose_xyz=[1.0, 0, 0])
        sel2.commit_candidate("kf_side", {(i, 1, 0): 1.0 for i in range(0, 35)}, pose_xyz=[1.2, 0, 0])
        prune = sel2.prune_region_set_cover("R1", ["kf_part1", "kf_part2", "kf_full", "kf_side"])
        self.assertEqual(set(prune.selected_keyframe_ids), {"kf_full", "kf_side"})
        self.assertEqual(set(prune.pruned_keyframe_ids), {"kf_part1", "kf_part2"})
        self.assertGreaterEqual(prune.weighted_coverage_ratio, 0.99)

        # Generic SLAM point-cloud surface voxel extractor works cleanly
        pts_w = np.array([[2.0, 0.0, 0.5], [2.0, 0.1, 0.5], [5.0, 0.0, 0.5]], dtype=np.float64)
        pc_vox = extract_pointcloud_surface_voxels(pts_w, [0.0, 0.0, 0.5], voxel_m=0.2)
        self.assertGreaterEqual(len(pc_vox), 2)


class TestOccupancyClearanceAndGlobalRegionMemory(unittest.TestCase):
    def _build_two_room_occupancy_grid(self) -> DepthOccupancyGrid:
        """Create two 4m x 4m rooms separated by a wall with a 1.0m doorway at x=4.0, y=0.0."""
        occ = DepthOccupancyGrid(resolution_m=0.10)
        grid = occ._grid_for(0.0)
        # Fill free space in Room A (x in [0..3.8], y in [-2..2]) and Room B (x in [4.2..8.0], y in [-2..2])
        for ix in range(0, 81):
            for iy in range(-20, 21):
                grid[(ix, iy)] = 0
        # Outer walls
        for ix in range(0, 81):
            grid[(ix, -20)] = 1
            grid[(ix, 20)] = 1
        for iy in range(-20, 21):
            grid[(0, iy)] = 1
            grid[(80, iy)] = 1
        # Dividing wall at x=4.0 (ix=40), leaving doorway opening y in [-0.4, +0.4] (iy in -4..4)
        for iy in range(-20, 21):
            if abs(iy) >= 5:
                grid[(40, iy)] = 1
        return occ

    def test_raycast_clearance_activates_scene_graph_rooms_doorway_and_event_summary(self) -> None:
        occ = self._build_two_room_occupancy_grid()
        # Wide room center -> clearance >= 1.5m
        clr_a, sec_a = occ.estimate_clearance_profile([2.0, 0.0, 0.0], num_sectors=16)
        self.assertGreaterEqual(clr_a, 1.5)
        self.assertEqual(sec_a.shape, (16,))
        # Standing close to ONE wall inside wide Room A (y=1.55, 0.45m from north wall, 3.55m from south wall)
        # Opposite-ray passage half-width stays ~2.0m -> NOT misclassified as a doorway!
        clr_near_wall, _ = occ.estimate_clearance_profile([2.0, 1.55, 0.0], num_sectors=16)
        self.assertGreaterEqual(clr_near_wall, 1.2)
        # Inside the 1.0m doorway (x=4.0, y=0.0) -> clearance ~0.5m < 0.85m bottleneck!
        clr_door, _ = occ.estimate_clearance_profile([4.0, 0.0, 0.0], num_sectors=16)
        self.assertLess(clr_door, 0.85)

        mem = HierarchicalMemoryStore(KeyframeSceneGraph())
        tex = TextureKeyframeSelector(set_cover_target=0.92)
        grm = GlobalRegionMemory(mem, tex)

        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            # Walk across Room A -> Doorway -> Room B
            waypoints = [
                ("kf_1", [1.5, 0.0, 0.0]),
                ("kf_2", [2.3, 0.2, 0.0]),
                ("kf_3", [3.0, 0.0, 0.0]),
                ("kf_4", [4.0, 0.0, 0.0]),  # doorway bottleneck!
                ("kf_5", [5.2, 0.0, 0.0]),
                ("kf_6", [6.2, 0.2, 0.0]),
                ("kf_7", [6.8, -0.2, 0.0]),
            ]
            for idx, (kid, xyz) in enumerate(waypoints):
                rgb_p = root / f"{kid}.png"
                rgb_p.write_bytes(b"png")
                c_m, s_c = occ.estimate_clearance_profile(xyz, num_sectors=16)
                mem.graph.add_keyframe(
                    kid,
                    _pose_matrix_zup(xyz, 0.0),
                    clearance_m=c_m,
                    rgb_path=str(rgb_p),
                    sector_clearances=s_c,
                )
                # Register distinct surface voxels for each keyframe
                tex.commit_candidate(kid, {(idx * 5 + j, 0, 0): 1.0 for j in range(10)}, pose_xyz=xyz)
                grm.record_keyframe_depth(kid, 1.5)

            # Anchor living room objects in Room A and bedroom objects in Room B
            mem.add_or_fuse_object("couch", "kf_2", [0.8, 0.0, 0.0], producer="sam3_depth_grounded")
            mem.add_or_fuse_object("coffee table", "kf_2", [0.5, 0.3, 0.0], producer="sam3_depth_grounded")
            mem.add_or_fuse_object("bed", "kf_6", [0.9, 0.0, 0.0], producer="sam3_depth_grounded")
            mem.add_or_fuse_object("nightstand", "kf_6", [0.9, 0.8, 0.0], producer="sam3_depth_grounded")

            # First sync in Room A sets current_region_id
            grm.sync_from_scene_graph([2.0, 0.0, 0.0])
            self.assertEqual(len(mem.graph.rooms), 2)
            self.assertGreaterEqual(len(mem.graph.doorways), 1)
            self.assertEqual(len(grm.regions), 2)

            # Now move to Room B -> exiting Room A triggers delayed Set-Cover + VLM summarizer!
            vlm_calls: list[tuple[str, list[str]]] = []

            def _mock_summarizer(reg, rgb_paths):
                vlm_calls.append((reg.region_id, list(rgb_paths)))
                return (reg.primary_label, f"VLM summary of {reg.primary_label} with {len(rgb_paths)} texture frames.")

            sync_res = grm.sync_from_scene_graph([6.2, 0.2, 0.0], summarizer_fn=_mock_summarizer)
            self.assertTrue(sync_res["region_changed"])
            self.assertEqual(len(vlm_calls), 1)

            labels = {r.primary_label for r in grm.regions.values()}
            self.assertEqual(labels, {"living_room", "bedroom"})
            # Verify doorway link and region search
            found_bed = grm.search_regions("bedroom bed")
            self.assertTrue(found_bed)
            self.assertEqual(found_bed[0]["primary_label"], "bedroom")
            self.assertTrue(found_bed[0]["connected_regions"])
            self.assertEqual(found_bed[0]["connected_regions"][0]["via"], "doorway")
            self.assertTrue(found_bed[0]["texture_keyframe_ids"])

    def test_open_plan_semantic_split_splits_living_and_kitchen_without_doorway(self) -> None:
        mem = HierarchicalMemoryStore(KeyframeSceneGraph())
        tex = TextureKeyframeSelector()
        grm = GlobalRegionMemory(mem, tex, open_plan_split_span_m=5.0, open_plan_min_subcluster_sep_m=2.5)

        # One single wide open-plan room (all clearances = 2.0m, continuous <=1.3m spacing, span = 6.5m)
        pts = [
            ("kf_1", [0.0, 0.0, 0.0]),
            ("kf_2", [1.2, 0.2, 0.0]),
            ("kf_3", [2.2, -0.1, 0.0]),
            ("kf_mid", [3.5, 0.0, 0.0]),
            ("kf_4", [4.8, 0.1, 0.0]),
            ("kf_5", [5.8, 0.2, 0.0]),
            ("kf_6", [6.5, 0.0, 0.0]),
        ]
        for idx, (kid, xyz) in enumerate(pts):
            mem.graph.add_keyframe(kid, _pose_matrix_zup(xyz, 0.0), clearance_m=2.0)
            tex.commit_candidate(kid, {(idx * 4 + j, 0, 0): 1.0 for j in range(6)}, pose_xyz=xyz)

        # Living room cluster on left (x ~ 0..2), kitchen cluster on right (x ~ 5..6.5)
        mem.add_or_fuse_object("couch", "kf_1", [0.5, 0.0, 0.0], producer="sam3_depth_grounded")
        mem.add_or_fuse_object("tv", "kf_2", [0.5, 0.5, 0.0], producer="sam3_depth_grounded")
        mem.add_or_fuse_object("stove", "kf_5", [0.5, 0.0, 0.0], producer="sam3_depth_grounded")
        mem.add_or_fuse_object("refrigerator", "kf_6", [0.5, 0.5, 0.0], producer="sam3_depth_grounded")

        grm.sync_from_scene_graph([5.8, 0.2, 0.0])
        self.assertEqual(len(mem.graph.rooms), 1)  # Single geometric room
        self.assertEqual(len(grm.regions), 2)  # Semantically split into 2 open-plan sub-regions!
        lbls = {r.primary_label for r in grm.regions.values()}
        self.assertEqual(lbls, {"living_room", "kitchen"})
        # Linked via open_passage
        conns = [c["via"] for r in grm.regions.values() for c in r.connected_regions]
        self.assertIn("open_passage", conns)


if __name__ == "__main__":
    unittest.main()
