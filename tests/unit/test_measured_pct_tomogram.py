"""Real native CPU PCT tests with synthetic measured returns, never scene assets."""
import unittest
import numpy as np
from embodied_navigation.planning.pct.measured_tomogram import MeasuredPctPlanner, native_pct_binary


def plane(nx=30, ny=12, resolution=.15, height=lambda x,y: 0.):
    return np.array([[(x+.5)*resolution, (y+.5)*resolution, height(x,y)]
                     for y in range(ny) for x in range(nx)])


class MeasuredPctTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = native_pct_binary()

    def planner(self, points, **kwargs):
        return MeasuredPctPlanner(points, native_binary=self.binary, **kwargs)

    def test_rectangular_native_axes_and_endpoint_semantics(self):
        start, goal = [.24,.24,.36], [4.24,1.24,.36]
        result = self.planner(plane(), body_support_offset_m=.36).plan(start,goal)
        self.assertTrue(result['found'],result)
        self.assertEqual(result['body_path_xyz'][0], start)
        self.assertEqual(result['body_path_xyz'][-1], goal)
        self.assertFalse(result['provenance']['ground_truth_used'])
        self.assertGreater(len(result['body_path_xyz']),20)

    def test_obstacle_detour(self):
        points = plane()
        wall = plane(3,7) + np.array([1.8,0,.35])
        result = self.planner(np.vstack([points,wall])).plan([.225,.375,.32],[3.975,.375,.32])
        self.assertTrue(result['found'],result)
        self.assertGreater(max(p[1] for p in result['body_path_xyz']),1.05)

    def test_unknown_gap_has_no_fallback(self):
        points=plane()
        points=points[(points[:,0]<1.8)|(points[:,0]>2.25)]
        result=self.planner(points).plan([.225,.375,.32],[3.975,.375,.32])
        self.assertFalse(result['found'])
        self.assertEqual(result['reason'],'native_no_path')
        self.assertEqual(result['body_path_xyz'],[])

    def test_measured_stairs_reach_four_metre_body_goal(self):
        points=plane(36,8,height=lambda x,y: min(x, 24)*.15)
        # Three densely sampled levels may overlap without treating chart index as floor.
        points=np.vstack([points,plane(36,8,height=lambda x,y:-2.)])
        start=[.225,.525,.55]  # .15 support + .4 child-origin offset
        goal=[4.725,.525,4.0]  # 3.60 support + .4
        result=self.planner(points,body_support_offset_m=.4).plan(start,goal)
        self.assertTrue(result['found'],result)
        self.assertEqual(result['body_path_xyz'][-1],goal)
        self.assertGreater(max(p[2] for p in result['body_path_xyz']),3.99)
        self.assertTrue(all(abs(a[2]-b[2]) <= .200001 for a,b in zip(result['body_path_xyz'],result['body_path_xyz'][1:])))

    def test_unobserved_goal_and_sparse_column(self):
        result=self.planner(plane()).plan([.225,.225,.32],[4.125,.225,4.])
        self.assertEqual(result['reason'],'goal_support_unobserved')
        result=self.planner(np.array([[0,0,z] for z in np.arange(0,4,.03)])).plan([0,0,.32],[0,0,4.])
        self.assertEqual(result['reason'],'no_measured_support')

    def test_negative_coordinates_and_height(self):
        points=plane()+np.array([-6,-3,-4])
        start=[-5.775,-2.775,-3.68]
        goal=[-1.875,-1.875,-3.68]
        result=self.planner(points).plan(start,goal)
        self.assertTrue(result['found'],result)
        self.assertEqual(result['body_path_xyz'][-1],goal)

    def test_opt_in_blind_ring_model_preserves_measured_grid(self):
        points=plane(60,40)+np.array([-3.,-3.,0.])
        points=points[np.linalg.norm(points[:,:2],axis=1)>1.3]
        start=[.025,.025,.32]; goal=[4.125,.075,.32]
        strict=self.planner(points)
        self.assertEqual(strict.plan(start,goal)['reason'],'start_support_unobserved')
        model=self.planner(points,start_blind_radius_m=1.5)
        original=model.cost.copy()
        result=model.plan(start,goal)
        self.assertTrue(result['found'],result)
        evidence=result['provenance']['start_surface_model']
        self.assertGreater(evidence['inferred_support_cells'],0)
        self.assertFalse(evidence['observed_free'])
        self.assertFalse(evidence['support_certified'])
        np.testing.assert_array_equal(original,model.cost)
        for _,row,col in evidence['inferred_cells_layer_row_col']:
            xy=model.origin+(np.array([col,row])+.5)*model.resolution
            self.assertLessEqual(np.linalg.norm(xy-np.array(start[:2])),1.5+1e-9)
            self.assertNotIn((row,col),model.measured_buckets)

    def test_blind_model_does_not_fill_gap_beyond_radius(self):
        points=plane(60,40)+np.array([-3.,-3.,0.])
        points=points[(np.linalg.norm(points[:,:2],axis=1)>1.3)&
                      ((points[:,0]<2.4)|(points[:,0]>2.85))]
        result=self.planner(points,start_blind_radius_m=1.5).plan([.025,.025,.32],[4.125,.075,.32])
        self.assertEqual(result['reason'],'native_no_path')
        self.assertEqual(result['body_path_xyz'],[])

    def test_blind_model_does_not_overwrite_measured_obstacle(self):
        points=plane(60,40)+np.array([-3.,-3.,0.])
        points=points[np.linalg.norm(points[:,:2],axis=1)>1.3]
        points=np.vstack([points, [[.075,.075,.3]]])
        result=self.planner(points,start_blind_radius_m=1.5).plan([.025,.025,.32],[4.125,.075,.32])
        self.assertEqual(result['reason'],'start_support_unobserved')
        self.assertEqual(result['provenance']['start_surface_model']['inferred_support_cells'],0)

    def test_start_footprint_snaps_across_single_riser_cell_without_fabricating_blind_support(self):
        points = plane(30, 12, height=lambda x, y: 0.0 if x < 10 else 0.15)
        # Add riser points inside cell (row=5, col=10) so that single cell rejects clustering.
        riser = np.array([[10.5 * 0.15, 5.5 * 0.15, z] for z in np.linspace(0.0, 0.15, 6)])
        planner = self.planner(np.vstack([points, riser]))
        start = [10.5 * 0.15, 5.5 * 0.15, 0.15 + 0.32]
        goal = [24.5 * 0.15, 5.5 * 0.15, 0.15 + 0.32]
        self.assertIsNone(planner._node(np.asarray(start)))
        result = planner.plan(start, goal)
        self.assertTrue(result['found'], result)
        snap = result['provenance']['start_footprint_snap']
        self.assertTrue(snap['applied'])
        self.assertLessEqual(snap['horizontal_distance_m'], 0.25)
        self.assertEqual(result['body_path_xyz'][0], start)
        self.assertEqual(result['body_path_xyz'][-1], goal)
        # Goal on the same rejected riser cell still fails strictly.
        bad_goal = planner.plan(goal, start)
        self.assertFalse(bad_goal['found'])
        self.assertEqual(bad_goal['reason'], 'goal_support_unobserved')

    def test_start_footprint_snap_rejects_wall_cliff_and_diagonal_corner_cut(self):
        base = plane(30, 12, height=lambda x, y: 0.0)
        goal = [24.5 * 0.15, 5.5 * 0.15, 0.32]
        # 1) Thin wall / torso-height obstacle at start cell must be rejected.
        wall_cell = np.array([[10.5 * 0.15, 5.5 * 0.15, 0.40]])
        wall_res = self.planner(np.vstack([base, wall_cell])).plan([10.5 * 0.15, 5.5 * 0.15, 0.32], goal)
        self.assertFalse(wall_res['found'])
        self.assertEqual(wall_res['reason'], 'start_support_unobserved')
        self.assertEqual(wall_res['provenance']['start_footprint_snap']['reason'], 'start_cell_obstacle')

        # 2) Single unobserved belly blind cell on a flat floor snaps cleanly inside footprint;
        # a deep void beyond footprint radius (>0.30 m) without blind-ring model still fails.
        single_blind = base[~((np.floor(base[:, 0] / 0.15) == 10) & (np.floor(base[:, 1] / 0.15) == 5))]
        blind_res = self.planner(single_blind).plan([10.5 * 0.15, 5.5 * 0.15, 0.32], goal)
        self.assertTrue(blind_res['found'], blind_res)
        self.assertTrue(blind_res['provenance']['start_footprint_snap']['applied'])

        # 2b) Cliff-lip overhang (foot-level lip plus deep drop in same start cell) must be rejected.
        cliff_lip = np.vstack([
            single_blind,
            [[10.5 * 0.15, 5.5 * 0.15, z] for z in (0.0, -0.05, -0.10, -0.45)],
        ])
        lip_res = self.planner(cliff_lip).plan([10.5 * 0.15, 5.5 * 0.15, 0.32], goal)
        self.assertFalse(lip_res['found'])
        self.assertEqual(lip_res['reason'], 'start_support_unobserved')
        self.assertEqual(lip_res['provenance']['start_footprint_snap']['reason'], 'start_cell_cliff_drop')

        # 3) Diagonal-only neighbor separated by two orthogonal obstacles must not corner-cut.
        diag_pts = base.copy()
        # Keep foot elevation at (10,5) with a mixed riser so exact _node fails,
        # block all four 4-neighbors (9,5),(11,5),(10,4),(10,6) with walls, leaving diagonals free.
        riser = np.array([[10.5 * 0.15, 5.5 * 0.15, z] for z in np.linspace(0.0, 0.15, 6)])
        orth_walls = np.array([
            [(10 + dc + 0.5) * 0.15, (5 + dr + 0.5) * 0.15, 0.40]
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1))
        ])
        diag_res = self.planner(np.vstack([diag_pts, riser, orth_walls])).plan(
            [10.5 * 0.15, 5.5 * 0.15, 0.15 + 0.32], goal
        )
        self.assertFalse(diag_res['found'])
        self.assertEqual(diag_res['reason'], 'start_support_unobserved')
        self.assertEqual(diag_res['provenance']['start_footprint_snap']['reason'], 'no_adjacent_support')

    def test_start_footprint_snap_handles_indoor_ceiling_only_belly_blind_cell_on_stairs(self):
        # Stair flight where the single cell under the robot has only ceiling returns (z=4.3m).
        stairs = plane(30, 10, height=lambda x, y: min(x, 16) * 0.15)
        mask = (np.floor(stairs[:, 0] / 0.15) == 10) & (np.floor(stairs[:, 1] / 0.15) == 5)
        stairs = stairs[~mask]
        ceiling = np.array([[10.5 * 0.15, 5.5 * 0.15, z] for z in (4.28, 4.31, 4.33)])
        start = [10.5 * 0.15, 5.5 * 0.15, 10 * 0.15 + 0.32]
        goal = [24.5 * 0.15, 5.5 * 0.15, 16 * 0.15 + 0.32]
        res = self.planner(np.vstack([stairs, ceiling])).plan(start, goal)
        self.assertTrue(res['found'], res)
        self.assertTrue(res['provenance']['start_footprint_snap']['applied'])

    def test_blind_ring_model_ignores_overhead_ceiling_above_clearance(self):
        floor = plane(60, 40) + np.array([-3.0, -3.0, 0.0])
        floor = floor[np.linalg.norm(floor[:, :2], axis=1) > 1.3]
        ceiling = plane(20, 20) + np.array([-1.5, -1.5, 3.0])
        start = [0.025, 0.025, 0.32]
        goal = [4.125, 0.075, 0.32]
        res = self.planner(np.vstack([floor, ceiling]), start_blind_radius_m=1.5).plan(start, goal)
        self.assertTrue(res['found'], res)
        self.assertTrue(res['provenance']['start_surface_model']['inferred_support_cells'] > 0)

    def test_empty_and_invalid(self):
        self.assertEqual(self.planner(np.empty((0,3))).plan([0,0,.32],[1,1,.32])['reason'],'no_measured_support')
        with self.assertRaises(ValueError):
            self.planner([[0,0,float('nan')]])

if __name__ == '__main__':
    unittest.main()
