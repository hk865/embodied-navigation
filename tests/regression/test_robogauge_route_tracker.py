import unittest, math, json, pathlib
from embodied_navigation.navigation.robogauge.route_tracker import RouteTracker

P = [[-22.0, -7.5, -1.1441], [-22.0, 1.5, -1.1440], [-13.5, 1.5, 0.3560]]


def route():
    return RouteTracker(P, [-14.53, -7.5])


class RouteChecks(unittest.TestCase):
    def test_stale_interval_clears_continuous_dwell(self):
        r = route()
        r.since = 10.0
        r.stall = (10.0, 0.0)
        r.invalidate_measurement()
        self.assertIsNone(r.since)
        self.assertIsNone(r.stall)

    def test_extended_budget_does_not_stop_at_old_deadline(self):
        r = RouteTracker(P, [-14.53, -7.5], motion_end=390.0)
        r.tick(230, [-14.53, -7.5, -2.95], math.pi, 0, 0)
        self.assertIsNone(r.failure)
        r.tick(390, [-14.53, -7.5, -2.95], math.pi, 0, 0)
        self.assertEqual(r.failure, "route_motion_budget")

    def test_report_only_stall_continues_to_global_limit(self):
        r = RouteTracker(P, [-14.53, -7.5], motion_end=390.0, stop_on_stall=False)
        r.tick(7, [-14.53, -7.5, -2.95], math.pi, 0, 0)
        c = r.tick(16, [-14.40, -7.5, -2.95], math.pi, 0, 0)
        self.assertIsNone(r.failure)
        self.assertGreater(c[0], 0)
        self.assertEqual(r.events[-1]["event"], "observed_low_progress_stall")
        self.assertLess(r.events[-1]["forward_progress_m"], 0)
        r.tick(390, [-14.40, -7.5, -2.95], math.pi, 0, 0)
        self.assertEqual(r.failure, "route_motion_budget")

    def test_wait(self):
        r = route()
        self.assertEqual(r.tick(6, [-14.53, -7.5, -2.95], math.pi, 0, 0), [0, 0, 0])
        self.assertEqual(r.tick(7, [-14.53, -7.5, -2.95], math.pi, 0, 0, False), [0, 0, 0])

    def test_forward_and_lateral_signs(self):
        for yy, sign in [(-7.6, -1), (-7.4, 1)]:
            c = route().tick(7, [-14.53, yy, -2.95], math.pi, 0, 0)
            self.assertGreater(c[0], 0)
            self.assertGreater(c[1] * sign, 0)

    def test_yaw_wrap(self):
        c = route().tick(7, [-14.53, -7.5, -2.95], -math.pi + 0.2, 0, 0)
        self.assertLess(c[2], 0)

    def test_full_three_segment_sequence(self):
        r = route()
        r.tick(7, [-14.53, -7.5, -2.95], math.pi, 0, 0)
        r.tick(60, [-21.85, -7.5, -1.1441], math.pi, 0, 0)
        self.assertEqual(r.phase, "brake")
        r.tick(60.6, [-21.85, -7.5, -1.1441], math.pi, 0, 0)
        self.assertEqual(r.phase, "turn")
        self.assertLess(r.tick(61, [-21.85, -7.5, -1.1441], math.pi, 0, 0)[2], 0)
        r.tick(71, [-21.85, -7.5, -1.1441], math.pi / 2, 0, 0)
        r.tick(71.6, [-21.85, -7.5, -1.1441], math.pi / 2, 0, 0)
        self.assertEqual(r.stage, 1)
        self.assertGreater(r.tick(72, [-22, -7.5, -1.1441], math.pi / 2, 0, 0)[0], 0)
        r.tick(140, [-22, 1.35, -1.144], math.pi / 2, 0, 0)
        r.tick(140.6, [-22, 1.35, -1.144], math.pi / 2, 0, 0)
        self.assertEqual(r.phase, "turn")
        self.assertLess(r.tick(141, [-22, 1.35, -1.144], math.pi / 2, 0, 0)[2], 0)
        r.tick(151, [-22, 1.35, -1.144], 0, 0, 0)
        r.tick(151.6, [-22, 1.35, -1.144], 0, 0, 0)
        self.assertEqual(r.stage, 2)
        r.tick(220, [-13.65, 1.5, 0.356], 0, 0, 0)
        r.tick(220.6, [-13.65, 1.5, 0.356], 0, 0, 0)
        self.assertTrue(r.arrived)

    def test_cross_track_bound(self):
        r = route()
        r.tick(7, [-14.53, -8.2, -2.95], math.pi, 0, 0)
        self.assertEqual(r.failure, "route_cross_track_bound")

    def test_overshoot(self):
        r = route()
        r.tick(60, [-22.5, -7.5, -1.144], math.pi, 0, 0)
        self.assertEqual(r.failure, "route_waypoint_overshoot")

    def test_stall(self):
        r = route()
        r.tick(7, [-14.53, -7.5, -2.95], math.pi, 0, 0)
        r.tick(16, [-14.53, -7.5, -2.95], math.pi, 0, 0)
        self.assertEqual(r.failure, "route_low_progress_stall")

    def test_global_timeout(self):
        r = route()
        self.assertEqual(r.tick(230, [-14.53, -7.5, -2.95], math.pi, 0, 0), [0, 0, 0])
        self.assertEqual(r.failure, "route_motion_budget")

    def test_turn_timeout(self):
        r = route()
        r.tick(60, [-21.85, -7.5, -1.1441], math.pi, 0, 0)
        r.tick(60.6, [-21.85, -7.5, -1.1441], math.pi, 0, 0)
        r.tick(102, [-21.85, -7.5, -1.1441], math.pi, 0, 0)
        self.assertEqual(r.failure, "corner_turn_timeout")

    def test_coordinate_z_not_false_arrival(self):
        r = route()
        r.tick(60, [-21.85, -7.5, -2.95], math.pi, 0, 0)
        self.assertEqual(r.phase, "cruise")
        self.assertFalse(r.arrived)

    def test_output_bounds(self):
        for angle in [-9, -math.pi, 0, math.pi, 9]:
            c = route().tick(7, [-14.53, -7.7, -2.95], angle, 0, 0)
            self.assertLessEqual(abs(c[0]), 0.15)
            self.assertLessEqual(abs(c[1]), 0.05)
            self.assertLessEqual(abs(c[2]), 0.15)


if __name__ == "__main__":
    unittest.main()
