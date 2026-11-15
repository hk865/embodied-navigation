import unittest
import numpy as np
from embodied_navigation.simulation.dynanav import operator_map as om


class FloorChartCapacityTests(unittest.TestCase):
    def test_twenty_measured_support_charts_fit_and_default_capacity_stays_finite(self):
        a=om.OccupancyAccumulator(extrinsic=om.Extrinsic(np.eye(4)),free_mode='floor')
        c=(np.arange(30)+.5)*.05-.75
        x,y=np.meshgrid(c,c)
        # A staircase/support transition may need more charts than physical
        # storeys. Each chart still needs the unchanged independent support
        # evidence; this checks capacity rather than relaxing that evidence.
        for i in range(20):
            floor=.2*i
            points=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,floor)))
            pose=om.transform_from_position_quaternion((0,0,floor+.35),(0,0,0,1))
            r=a.add_frame(points,frame='map',cloud_stamp_s=float(i),pose=pose,
                pose_stamp_s=float(i),sim_age_s=.02,wall_age_s=.02)
            self.assertTrue(r['accepted'],r)
        self.assertEqual(len(a.levels),20)
        tracker=om.LevelTracker()
        for i in range(64):
            self.assertIsNotNone(tracker.observe(float(i)).level_index)
        refused=tracker.observe(64.)
        self.assertIsNone(refused.level_index)
        self.assertIn('too_many_levels',refused.reasons)
        self.assertEqual(len(tracker.levels),64)


if __name__=='__main__': unittest.main()
