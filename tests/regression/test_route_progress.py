import unittest
from embodied_navigation.control.tracking.route_progress import ConnectedRouteProgress


class ConnectedProgressFlows(unittest.TestCase):
    def test_closed_route_does_not_complete_at_birth_or_jump_to_return(self):
        p = ConnectedRouteProgress(
            [[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0], [3, 1, 0], [2, 1, 0], [1, 1, 0], [0, 0, 0]]
        )
        p.update([0, 0, 0])
        self.assertGreater(p.remaining, 7.0)
        p.update([0.8, 0.8, 0])
        self.assertEqual(p.progress, 0.0)
        for x in [0.5, 1, 1.5, 2, 2.5, 3]:
            p.update([x, 0, 0])
        for q in [
            [3, 0.5, 0],
            [3, 1, 0],
            [2.5, 1, 0],
            [2, 1, 0],
            [1.5, 1, 0],
            [1, 1, 0],
            [0.5, 0.5, 0],
            [0, 0, 0],
        ]:
            p.update(q)
        self.assertLess(p.remaining, 0.01)

    def test_smoothed_return_turn_witnesses_connected_corner(self):
        p = ConnectedRouteProgress(
            [[0, 0, 0], [0, 1, 0], [0, 2, 0], [0.25, 2.2, 0], [0, 2, 0], [0, 1, 0], [0, 0, 0]]
        )
        for q in [
            [0, 0, 0],
            [0, 0.6, 0],
            [0, 1.2, 0],
            [0, 1.8, 0],
            [0.05, 1.95, 0],
            [0, 1.7, 0],
            [0, 1.1, 0],
            [0, 0.5, 0],
            [0, 0, 0],
        ]:
            p.update(q)
        self.assertLess(p.remaining, 0.01)

    def test_other_floor_and_disconnected_far_observation_do_not_advance(self):
        p = ConnectedRouteProgress(
            [[0, 0, 0], [1, 0, 0], [2, 0, 0], [2, 0, 1.5], [1, 0, 1.5], [0, 0, 1.5]]
        )
        p.update([0, 0, 1.5])
        self.assertEqual(p.progress, 0.0)
        p.update([2, 0, 0])
        self.assertEqual(p.progress, 0.0)
        p.update([0.5, 0, 0])
        self.assertAlmostEqual(p.progress, 0.5)
        p.update([0, 0, 0])
        self.assertAlmostEqual(p.progress, 0.5)


if __name__ == "__main__":
    unittest.main()
