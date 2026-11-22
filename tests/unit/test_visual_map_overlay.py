"""Unit tests for read-only 2D occupancy map visual overlay and Set-of-Mark candidates."""

from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from embodied_navigation.planning.exploration.visual_map_overlay import (
    GO2_LENGTH_M,
    GO2_TURN_RADIUS_M,
    GO2_WIDTH_M,
    extract_visual_candidates,
    render_annotated_2d_maps,
    robot_footprint_polygon,
    turning_circle_clearance,
)


def _room_with_doorway_grid(res: float = 0.05):
    """Build a 6m x 6m room with walls except an open doorway on the east side leading to unknown space."""
    ix_list, iy_list, code_list = [], [], []
    # Interior free: x in [0, 5.0], y in [-2.5, 2.5]; walls at borders except doorway y in [-0.7, 0.7] at x=5.0
    xs = range(0, int(round(5.0 / res)) + 1)
    ys = range(int(round(-2.5 / res)), int(round(2.5 / res)) + 1)
    for gx in xs:
        for gy in ys:
            x, y = (gx + 0.5) * res, (gy + 0.5) * res
            is_border = gx in (xs[0], xs[-1]) or gy in (ys[0], ys[-1])
            is_door = gx == xs[-1] and -0.7 <= y <= 0.7
            if is_border and not is_door:
                ix_list.append(gx)
                iy_list.append(gy)
                code_list.append(1)
            else:
                ix_list.append(gx)
                iy_list.append(gy)
                code_list.append(0)
    # Add a disconnected free pocket behind the north wall (must NEVER be proposed as reachable!)
    for gx in range(20, 60):
        for gy in range(int(round(3.5 / res)), int(round(4.5 / res))):
            ix_list.append(gx)
            iy_list.append(gy)
            code_list.append(0)
    return (
        np.asarray(ix_list, dtype=np.int64),
        np.asarray(iy_list, dtype=np.int64),
        np.asarray(code_list, dtype=np.int64),
    )


class VisualMapOverlayTests(unittest.TestCase):
    def test_robot_footprint_polygon_dimensions_and_rotation(self) -> None:
        poly = robot_footprint_polygon(2.0, -1.0, math.pi / 2.0)
        self.assertEqual(poly.shape, (4, 2))
        # At yaw=+90 deg, front edge (corners 0, 1) must be at y = -1.0 + L/2
        np.testing.assert_allclose(poly[:2, 1], [-1.0 + 0.5 * GO2_LENGTH_M] * 2, atol=1e-9)
        width = float(np.linalg.norm(poly[0] - poly[1]))
        length = float(np.linalg.norm(poly[0] - poly[3]))
        self.assertAlmostEqual(width, GO2_WIDTH_M, places=6)
        self.assertAlmostEqual(length, GO2_LENGTH_M, places=6)

    def test_turning_circle_clearance_detects_clear_vs_tight(self) -> None:
        wall = np.array([[1.0, 0.0], [1.0, 0.2], [1.0, -0.2]], dtype=np.float64)
        clear = turning_circle_clearance(0.0, 0.0, wall, radius_m=GO2_TURN_RADIUS_M)
        self.assertTrue(clear["safe"])
        self.assertEqual(clear["obstacle_count"], 0)
        self.assertAlmostEqual(clear["min_obstacle_dist_m"], 1.0, places=3)

        tight = turning_circle_clearance(0.6, 0.0, wall, radius_m=GO2_TURN_RADIUS_M)
        self.assertFalse(tight["safe"])
        self.assertGreater(tight["obstacle_count"], 0)
        self.assertAlmostEqual(tight["min_obstacle_dist_m"], 0.4, places=3)

    def test_extract_candidates_finds_doorway_frontier_and_excludes_disconnected_pocket(self) -> None:
        ix, iy, codes = _room_with_doorway_grid()
        cands = extract_visual_candidates(
            ix,
            iy,
            codes,
            0.05,
            robot_xyz=[1.5, 0.0, 0.02],
            level_z_m=0.0,
            trace_xy=np.array([[0.8, 0.0], [1.5, 0.0]]),
        )
        frontiers = [c for c in cands if c["kind"] == "frontier"]
        self.assertGreaterEqual(len(frontiers), 1)
        # Doorway frontier must lie inside the room near the east opening (x ~ 4.3..4.7, |y| < 0.7)
        f1 = frontiers[0]
        self.assertEqual(f1["id"], "F1")
        self.assertGreater(f1["xy"][0], 4.0)
        self.assertLess(abs(f1["xy"][1]), 0.8)
        self.assertGreaterEqual(f1["clearance_m"], 0.45)
        # No candidate may leak into the disconnected pocket behind y=2.5 wall
        for c in cands:
            self.assertLess(c["xy"][1], 2.5)

    def test_se3_transform_and_png_rendering_produce_global_and_local_charts(self) -> None:
        ix, iy, codes = _room_with_doorway_grid()
        T = np.eye(4, dtype=np.float64)
        T[:3, 3] = [10.0, -5.0, 0.0]
        with tempfile.TemporaryDirectory() as td:
            g_path = Path(td) / "height_chart_000.png"
            l_path = Path(td) / "height_chart_000_local.png"
            trace = np.array(
                [
                    [10.8, -5.0, 0.02],
                    [11.5, -5.0, 0.02],
                    [11.5, -5.0, 3.20],  # upper floor point must be filtered out of basement chart
                ],
                dtype=np.float64,
            )
            out = render_annotated_2d_maps(
                ix,
                iy,
                codes,
                0.05,
                robot_xyz=[11.5, -5.0, 0.02],
                yaw=0.25,
                level_z_m=0.0,
                global_path=g_path,
                local_path=l_path,
                title="Height chart +0.00 / active",
                trace_xyz=trace,
                transform_se3=T,
            )
            self.assertTrue(g_path.is_file() and g_path.stat().st_size > 5000)
            self.assertTrue(l_path.is_file() and l_path.stat().st_size > 5000)
            self.assertTrue(out["turning_circle"]["safe"])
            frontiers = [c for c in out["candidates"] if c["kind"] == "frontier"]
            self.assertGreaterEqual(len(frontiers), 1)
            # Coordinates must be expressed in transformed map frame (x ~ 14.3..14.7, y ~ -5.0)
            self.assertGreater(frontiers[0]["xy"][0], 14.0)
            self.assertAlmostEqual(frontiers[0]["xy"][1], -5.0, delta=0.8)


if __name__ == "__main__":
    unittest.main()
