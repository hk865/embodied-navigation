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

    def test_empty_and_invalid(self):
        self.assertEqual(self.planner(np.empty((0,3))).plan([0,0,.32],[1,1,.32])['reason'],'no_measured_support')
        with self.assertRaises(ValueError):
            self.planner([[0,0,float('nan')]])

if __name__ == '__main__':
    unittest.main()
