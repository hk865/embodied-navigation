"""Exact legacy-formula and collision-gate regressions for the CPU control core."""

import unittest
import numpy as np
from embodied_navigation.control.relative_pid import follow_segment
from embodied_navigation.control.collision import certified_support, braking_sweep, depth_collision
from embodied_navigation.planning.route import lookahead_target


def original_driving_command(b, R, v, yaw, a, z):
    # Frozen expression copied from the original relative-SLAM multiwaypoint bridge.
    d = float(np.linalg.norm(b[:2] - z))
    unit = (z - a) / np.linalg.norm(z - a)
    s = float(np.clip((b[:2] - a) @ unit, 0, np.linalg.norm(z - a)))
    target = a + unit * min(np.linalg.norm(z - a), s + 0.7)
    e = target - b[:2]
    desired = e - 0.10 * v[:2]
    desired *= min(1.0, 0.5 / max(np.linalg.norm(desired), 1e-9))
    local = R[:2, :2].T @ desired
    heading = float(np.arctan2(e[1], e[0]))
    err = float(np.arctan2(np.sin(heading - yaw), np.cos(heading - yaw)))
    cap = 0.12 if abs(err) > 0.25 else 0.5
    if abs(err) > 0.5:
        cap = 0.0
    brake_cap = max(0.0, -0.25 * 0.25 + np.sqrt((0.25 * 0.25) ** 2 + 2 * 0.25 * d))
    cmd = np.array(
        [
            min(max(local[0], 0.0), cap, 1.2 * d, brake_cap),
            np.clip(local[1], -0.05, 0.05),
            np.clip(1.2 * err, -0.15, 0.15),
        ]
    )
    if abs(err) > 0.5:
        cmd[:2] = 0.0
    return cmd


class ControlTests(unittest.TestCase):
    def test_original_formula_equivalence(self):
        rng = np.random.default_rng(42)
        for _ in range(1000):
            b = rng.uniform(-10, 10, 3)
            v = rng.uniform(-1, 1, 3)
            yaw = rng.uniform(-4 * np.pi, 4 * np.pi)
            c, s = np.cos(yaw), np.sin(yaw)
            R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            a = rng.uniform(-10, 10, 2)
            z = a + rng.uniform(0.1, 6, 2)
            old = original_driving_command(b, R, v, yaw, a, z)
            new, _ = follow_segment(b[:2], R[:2, :2], v[:2], yaw, a, z)
            np.testing.assert_array_equal(old, new)

    def test_large_heading_error_stops_translation(self):
        cmd, _ = follow_segment([0, 0], np.eye(2), [0, 0], np.pi, [0, 0], [2, 0])
        np.testing.assert_array_equal(cmd[:2], [0, 0])

    def test_map_boundary_and_collision(self):
        grid = np.ones((20, 20), bool)
        self.assertTrue(certified_support([[1, 1]], [0, 0], 0.1, grid))
        self.assertFalse(certified_support([[-1, 0]], [0, 0], 0.1, grid))
        sweep, brake = braking_sweep([0, 0], np.eye(2), [0.5, 0, 0], [0, 0])
        self.assertAlmostEqual(brake, 0.775)
        self.assertTrue(depth_collision(sweep, np.array([[0.5, 0, 0]]), np.array([1, 0]))[0])

    def test_degenerate_route_is_explicit(self):
        with self.assertRaises(ValueError):
            lookahead_target([0, 0], [1, 1], [1, 1])


if __name__ == "__main__":
    unittest.main()
