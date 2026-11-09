"""CPU checks of scene-query admission, not simulated contact/traversal tests."""

import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from embodied_navigation.simulation.dynanav.session_collision import (
    SupportPreflightError,
    load_config,
    preflight_support,
)


class Query:
    def __init__(self, config, *, extra=None, extra_distance=0.04, missing=False):
        self.config = config
        self.extra = extra
        self.extra_distance = extra_distance
        self.missing = missing
        self.calls = 0

    def raycast_all(self, origin, direction, length, callback, both_sides):
        sample = self.config["support_preflight"]["samples"][self.calls]
        first = self.calls == 0
        self.calls += 1
        if not (first and self.missing):
            callback(
                SimpleNamespace(
                    collision=sample["expected_colliders"][0],
                    rigid_body="",
                    position=sample["surface_xyz_m"],
                    normal=(0, 0, 1),
                    distance=0.12,
                )
            )
        # Return the nearer obstacle last: query callbacks need not be sorted.
        if first and self.extra:
            callback(
                SimpleNamespace(
                    collision=self.extra,
                    rigid_body="",
                    position=(origin[0], origin[1], origin[2] - self.extra_distance),
                    normal=(0, 0, 1),
                    distance=self.extra_distance,
                )
            )
        return 2


class SessionSupportAdmission(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.carb = patch.dict(
            sys.modules, {"carb": SimpleNamespace(Float3=lambda *xyz: xyz)}
        )
        self.carb.start()
        self.addCleanup(self.carb.stop)

    def test_original_first_flight_selection_and_explicit_robot_exclusion(self):
        self.assertEqual(len(self.config["floor_paths"]), 7)
        self.assertEqual(len(self.config["stair_paths"]), 4)
        samples = self.config["support_preflight"]["samples"]
        self.assertEqual(
            [s["source_waypoint_index"] for s in samples[1:]], list(range(3, 16))
        )
        self.assertTrue(all(s["surface_xyz_m"][1] == -7.5 for s in samples))
        query = Query(self.config, extra="/World/Go2/foot")
        report = preflight_support(query)
        self.assertTrue(report["passed"])
        self.assertEqual(query.calls, len(samples))
        self.assertEqual(len(report["checks"][0]["all_hits"]), 2)
        self.assertEqual(len(report["checks"][0]["robot_hits_excluded"]), 1)
        self.assertFalse(report["physics_advanced"])

    def test_nearer_unexpected_collider_cannot_select_floor_below_it(self):
        query = Query(self.config, extra="/Root/unexpected_obstacle")
        with self.assertRaises(SupportPreflightError) as caught:
            preflight_support(query)
        report = caught.exception.report
        self.assertFalse(report["passed"])
        self.assertEqual(
            len(report["checks"]), len(self.config["support_preflight"]["samples"])
        )
        first = report["checks"][0]
        self.assertEqual(
            first["nearest_scene_hit"]["collision"], "/Root/unexpected_obstacle"
        )
        self.assertEqual(len(first["all_hits"]), 2)

    def test_missing_hit_remains_missing(self):
        with self.assertRaises(SupportPreflightError) as caught:
            preflight_support(Query(self.config, missing=True))
        first = caught.exception.report["checks"][0]
        self.assertFalse(first["passed"])
        self.assertEqual(first["all_hits"], [])
        self.assertIsNone(first["nearest_scene_hit"])
        self.assertFalse(caught.exception.report["synthetic_or_filled_hits"])

    def test_tied_nearest_unknown_collider_is_not_callback_order_dependent(self):
        query = Query(
            self.config, extra="/Root/unexpected_obstacle", extra_distance=0.12
        )
        with self.assertRaises(SupportPreflightError) as caught:
            preflight_support(query)
        first = caught.exception.report["checks"][0]
        self.assertEqual(len(first["nearest_scene_hits"]), 2)
        self.assertFalse(first["passed"])


if __name__ == "__main__":
    unittest.main()
