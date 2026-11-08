import unittest
from embodied_navigation.evaluation.trajectory import path_length_xy, position_error_xy
class MetricsTests(unittest.TestCase):
 def test_known_path(self):self.assertEqual(path_length_xy([[0,0],[3,0],[3,4]]),7)
 def test_zero_error(self):self.assertEqual(position_error_xy([[0,0],[1,1]],[[0,0],[1,1]])['rmse_m'],0)
if __name__=='__main__':unittest.main()
