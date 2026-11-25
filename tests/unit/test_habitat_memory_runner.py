"""CPU unit tests for Habitat-Sim HM3D coordinate bridge, depth raycast, and occupancy grid."""

from __future__ import annotations

import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from embodied_navigation.planning.exploration.habitat_memory_runner import (
    DepthOccupancyGrid,
    _dedup_viewpoints,
    compute_spl,
    depth_bearing_to_local_xy,
    hab_quat_to_mem_yaw,
    hab_to_mem,
    is_geodesic_target_reached,
    measure_depth_bearing_range,
    mem_to_hab,
)
from embodied_navigation.planning.exploration.streaming_task_agent import (
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


if __name__ == "__main__":
    unittest.main()
