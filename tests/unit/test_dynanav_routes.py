"""CPU route/config and query-admission checks, never live support evidence."""

import math
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from embodied_navigation.simulation.dynanav.routes import get_route
from embodied_navigation.simulation.dynanav.session_collision import (
    SupportPreflightError,
    load_config,
    preflight_support,
)


class RouteQuery:
    def __init__(self, config, *, lower_support=False, extra=None, missing=False):
        self.config = config
        self.lower_support = lower_support
        self.extra = extra
        self.missing = missing
        self.calls = 0

    def raycast_all(self, origin, direction, length, callback, both_sides):
        sample = self.config["support_preflight"]["samples"][self.calls]
        self.calls += 1
        if self.missing and self.calls == 1:
            return 0
        # Ground-floor carpet is visible but originally has no CollisionAPI.
        # Model the original supporting floor lying about1cm below its top.
        lower = self.lower_support and sample.get("source_waypoint_index") == 28
        position = list(sample["surface_xyz_m"])
        distance = 0.12
        if lower:
            position[2] -= 0.0096
            distance += 0.0096
        callback(
            SimpleNamespace(
                collision=sample["expected_colliders"][0],
                rigid_body="",
                position=position,
                normal=[0, 0, 1],
                distance=distance,
            )
        )
        if self.extra and self.calls == 1:
            callback(
                SimpleNamespace(
                    collision=self.extra,
                    rigid_body="",
                    position=[origin[0], origin[1], origin[2] - 0.04],
                    normal=[0, 0, 1],
                    distance=0.04,
                )
            )
        return 1


class TrackedRoutes(unittest.TestCase):
    def setUp(self):
        self.route = get_route("basement-to-ground")
        self.config = load_config(self.route["config_path"])
        stub = patch.dict(sys.modules, {"carb": SimpleNamespace(Float3=lambda *v: v)})
        stub.start()
        self.addCleanup(stub.stop)

    def test_old_route_and_tolerance_are_preserved(self):
        old = load_config(get_route("first-flight")["config_path"])
        self.assertEqual(old["support_preflight"]["surface_tolerance_m"], 0.003)
        self.assertEqual(len(old["support_preflight"]["samples"]), 14)
        self.assertEqual(
            get_route("first-flight")["waypoints_xyz"], [[-22.0, -7.5, -1.1441]]
        )
        self.assertEqual(self.config["floor_paths"], old["floor_paths"])
        self.assertEqual(self.config["stair_paths"], old["stair_paths"])

    def test_full_route_has_both_flights_landing_and_ground_exit(self):
        self.assertEqual(
            self.route["waypoints_xyz"],
            [
                [-22.0, -7.5, -1.1441],
                [-22.0, 1.5, -1.1441],
                [-13.5, 1.5, 0.35],
            ],
        )
        samples = self.config["support_preflight"]["samples"]
        self.assertEqual(len(samples), 30)
        self.assertEqual(
            [
                s["source_waypoint_index"]
                for s in samples
                if "source_waypoint_index" in s
            ],
            list(range(3, 29)),
        )
        interior = [s for s in samples if s["label"].startswith("landing_interior_")]
        self.assertEqual(
            [s["surface_xyz_m"][1] for s in interior], [-5.25, -3.0, -0.75]
        )
        allowed = set(
            self.config["floor_paths"]
            + self.config["stair_paths"]
            + self.config["walk_surface_paths"]
        )
        self.assertTrue(all(set(s["expected_colliders"]) <= allowed for s in samples))
        self.assertTrue(all(s["expected_colliders"] for s in samples))
        self.assertTrue(
            all(
                len(s["surface_xyz_m"]) == 3
                and all(math.isfinite(v) for v in s["surface_xyz_m"])
                for s in samples
            )
        )
        self.assertIsNone(self.config["route"]["stair_specific_speed_limit"])
        self.assertFalse(self.config["provenance"]["runtime_archive_dependency"])

    def test_routes_return_independent_values_and_reject_unknown_name(self):
        self.route["waypoints_xyz"][0][0] = 0
        self.assertEqual(get_route("basement-to-ground")["waypoints_xyz"][0][0], -22.0)
        with self.assertRaises(ValueError):
            get_route("unlisted")

    def test_measured_nearest_support_is_required_and_one_cm_floor_gap_is_allowed(self):
        query = RouteQuery(self.config, lower_support=True)
        report = preflight_support(query, self.route["config_path"])
        self.assertTrue(report["passed"])
        self.assertEqual(query.calls, 30)
        self.assertFalse(report["physics_advanced"])
        self.assertFalse(report["synthetic_or_filled_hits"])
        with self.assertRaises(SupportPreflightError):
            preflight_support(
                RouteQuery(self.config, missing=True), self.route["config_path"]
            )

    def test_nearer_unknown_or_ground_plane_cannot_be_hidden_by_allowed_support(self):
        for extra in ["/Root/unexpected_obstacle", self.config["ground_plane_path"]]:
            with (
                self.subTest(extra=extra),
                self.assertRaises(SupportPreflightError) as caught,
            ):
                preflight_support(
                    RouteQuery(self.config, extra=extra), self.route["config_path"]
                )
            self.assertEqual(
                caught.exception.report["checks"][0]["nearest_scene_hit"]["collision"],
                extra,
            )


if __name__ == "__main__":
    unittest.main()
