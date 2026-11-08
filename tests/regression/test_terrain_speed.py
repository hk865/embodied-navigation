import unittest
import numpy as np
from embodied_navigation.control.tracking.terrain_speed import StairSpeedLimit

class PlannedSpeedFlows(unittest.TestCase):
    def test_cruise_reserve_on_own_stair_but_not_other_floor(self):
        path=np.array([[0,0,.32],[1,0,.32],[1.3,0,.47],[1.6,0,.62],[2.6,0,.62]])
        cap=StairSpeedLimit(path)
        self.assertEqual(cap.cruise([1.1,0,.34]),.25)
        self.assertEqual(cap.cruise([1.1,0,1.84]),.4)
        self.assertEqual(cap.cruise([-2,0,.32]),.4)
        self.assertEqual(cap.cruise([1.1,0,.34],.16),.16)
    def test_flat_route_preserves_original_cruise(self):
        self.assertEqual(StairSpeedLimit([[0,0,.32],[1,0,.32]]).cruise([.5,0,.32]),.4)

if __name__=='__main__':unittest.main()
