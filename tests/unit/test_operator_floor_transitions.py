"""Measured support surfaces seen while ascending; no simulator or world map."""
import unittest
import numpy as np
from embodied_navigation.simulation.dynanav import operator_map as om


def patch(z, x=0.0, n=30):
    c=(np.arange(n)+.5)*.05-.5*n*.05
    xx,yy=np.meshgrid(c+x,c)
    return np.column_stack((xx.ravel(),yy.ravel(),np.full(xx.size,z)))


class FloorTransitionTests(unittest.TestCase):
    def setUp(self):
        self.a=om.OccupancyAccumulator(extrinsic=om.Extrinsic(np.eye(4)),free_mode='floor')
        self.stamp=0

    def add(self, floor, *, x=0.0, other=None):
        self.stamp+=1
        points=patch(floor,x)
        if other is not None: points=np.vstack((other,points))
        pose=om.transform_from_position_quaternion((x,0,floor+.35),(0,0,0,1))
        return self.a.add_frame(points,frame='map',cloud_stamp_s=float(self.stamp),
            pose=pose,pose_stamp_s=float(self.stamp),sim_age_s=.02,wall_age_s=.02)

    def test_visible_lower_landing_cannot_anchor_new_stair_height_band(self):
        low=self.add(-.34)
        self.assertTrue(low['accepted'])
        old=low['level_index']
        old_cells=dict(self.a.grid_for(old)._cells)
        # The lower landing has many more returns than the near support
        # surface. A quantile over the whole stairwell would select it.
        lower=patch(-.34,n=70)
        for surface in (.15,.55,.95,1.4):
            reached=self.add(surface,x=3.,other=lower)
            self.assertTrue(reached['accepted'], reached)
            self.assertAlmostEqual(reached['classification_floor_z_m'],surface)
        self.assertAlmostEqual(self.a._level_floor_z[old],-.34)
        self.assertEqual(dict(self.a.grid_for(old)._cells),old_cells)

    def test_new_supported_surface_reuses_its_chart_without_moving_old_grid(self):
        low=self.add(0.)
        low_index=low['level_index']
        old_cells=dict(self.a.grid_for(low_index)._cells)
        # The two measured child origins are only 0.20m apart, within the
        # pose tracker's claim radius. Their independently evidenced floors
        # nevertheless need separate classification references.
        high=self.add(.20)
        self.assertTrue(high['accepted'],high)
        self.assertNotEqual(high['level_index'],low_index)
        repeated=self.add(.20)
        self.assertTrue(repeated['accepted'],repeated)
        self.assertEqual(repeated['level_index'],high['level_index'])
        self.assertAlmostEqual(self.a._level_floor_z[low_index],0.)
        self.assertAlmostEqual(self.a._level_floor_z[high['level_index']],.20)
        self.assertEqual(dict(self.a.grid_for(low_index)._cells),old_cells)


if __name__=='__main__': unittest.main()
