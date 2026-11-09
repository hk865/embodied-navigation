"""Regression: planned stair geometry lowers cruise speed only near its own stair."""

import unittest
import numpy as np
from embodied_navigation.control.tracking.terrain_speed import StairSpeedLimit


class PlannedSpeedFlows(unittest.TestCase):
    def test_cruise_reserve_on_own_stair_but_not_other_floor(self):
        path = np.array(
            [[0, 0, 0.32], [1, 0, 0.32], [1.3, 0, 0.47], [1.6, 0, 0.62], [2.6, 0, 0.62]]
        )
        cap = StairSpeedLimit(path)
        self.assertEqual(cap.cruise([1.1, 0, 0.34]), 0.25)
        self.assertEqual(cap.cruise([1.1, 0, 1.84]), 0.4)
        self.assertEqual(cap.cruise([-2, 0, 0.32]), 0.4)
        self.assertEqual(cap.cruise([1.1, 0, 0.34], 0.16), 0.16)

    def test_flat_route_preserves_original_cruise(self):
        self.assertEqual(StairSpeedLimit([[0, 0, 0.32], [1, 0, 0.32]]).cruise([0.5, 0, 0.32]), 0.4)


if __name__ == "__main__":
    unittest.main()
