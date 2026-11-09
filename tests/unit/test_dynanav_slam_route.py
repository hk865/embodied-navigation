"""CPU checks of measured odometry control; these do not establish navigation success."""

import json
import math
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

import numpy as np

from embodied_navigation.navigation.robogauge.route_tracker import RouteTracker
from embodied_navigation.simulation.dynanav.slam_route import (
    SlamRouteTracker,
    load_slam_route_reference,
    pose_transform,
    validate_slam_route_reference,
)


def q(yaw):
    return [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]


def measured_pose(stamp, p, yaw=0):
    return {
        "stamp_ns": stamp,
        "position_xyz_m": list(p),
        "quaternion_wxyz": q(yaw),
        "estimated_velocity_xyz_mps": [0, 0, 0],
        "estimated_yaw_rate_rps": 0,
    }


def reference(anchor=(0, 0, 1), goal=(4, 0, 1), yaw=0):
    return {
        "version": 1,
        "pose_source": "lio_estimate",
        "repeat_spawn_assumption": True,
        "map_registration_enabled": False,
        "no_gt_pose_used": True,
        "source": {
            "bag": "/nonexistent/sensor-only-bag",
            "files": [{"path": "bag.db3", "sha256": "a" * 64}],
            "topic": "/dynanav/lio_observer/aft_mapped_to_init",
            "map_frame": "dynanav_lio_map",
            "sensor_frame": "go2_imu",
            "message_count": 100,
        },
        "anchor": measured_pose(1_000_000_000, anchor, yaw),
        "goals": [measured_pose(10_000_000_000, goal, yaw)],
        "source_correspondence": {
            "anchor_stamp_ns": 1_000_000_000,
            "goal_stamps_ns": [10_000_000_000],
        },
    }


class MeasuredSlamRouteTests(unittest.TestCase):
    def tracker(self, ref=None, **kwargs):
        return SlamRouteTracker(
            reference() if ref is None else ref,
            repeat_spawn_assumption=True,
            map_registration_enabled=False,
            **kwargs,
        )

    def tick(self, tracker, t=7, p=(0, 0, 1), yaw=0, **kwargs):
        values = {
            "current_sim_s": t,
            "odom_stamp_s": t,
            "position": p,
            "quat_wxyz": q(yaw),
            "estimated_velocity": (0, 0, 0),
            "measured_yaw_rate": 0,
            "ready": True,
            "now_wall": t + 100,
            "arrival_wall": t + 100,
            "frame_id": "dynanav_lio_map",
            "child_frame_id": "go2_imu",
        }
        values.update(kwargs)
        return tracker.tick(**values)

    def test_explicit_lab_assumption_required_and_never_general_localization(self):
        with self.assertRaises(TypeError):
            SlamRouteTracker(reference())
        for flags in (
            {"repeat_spawn_assumption": False, "map_registration_enabled": False},
            {"repeat_spawn_assumption": True, "map_registration_enabled": True},
        ):
            with self.subTest(flags=flags), self.assertRaises(ValueError):
                SlamRouteTracker(reference(), **flags)
        command, receipt = self.tick(self.tracker())
        self.assertGreater(command[0], 0)
        self.assertFalse(receipt["general_global_localization"])
        self.assertTrue(receipt["repeat_spawn_assumption"])
        self.assertFalse(receipt["map_registration_enabled"])

    def test_full_measured_se3_alignment_once_after_ready_and_input_unchanged(self):
        route = self.tracker(
            reference(anchor=(10, 20, 3), goal=(10, 25, 3), yaw=math.pi / 2)
        )
        p = np.asarray([100.0, -4, 6])
        original = p.copy()
        command, receipt = self.tick(route, p=(99, -7, 8), ready=False)
        self.assertEqual(command, [0, 0, 0])
        self.assertIsNone(receipt["alignment"])
        command, receipt = self.tick(route, t=7.1, p=p)
        np.testing.assert_array_equal(p, original)
        np.testing.assert_allclose(
            receipt["aligned_position_xyz_m"], [10, 20, 3], atol=1e-12
        )
        self.assertAlmostEqual(receipt["aligned_yaw_rad"], math.pi / 2)
        self.assertGreater(command[0], 0)
        first_alignment = receipt["alignment"]
        command, receipt = self.tick(route, t=7.2, p=(102, -4, 6))
        np.testing.assert_allclose(
            receipt["aligned_position_xyz_m"], [10, 22, 3], atol=1e-12
        )
        self.assertEqual(receipt["alignment"], first_alignment)
        self.assertEqual(receipt["alignment_anchor"]["current_odom_stamp_s"], 7.1)

    def test_se3_rotation_includes_tilt_and_wxyz_is_explicit(self):
        tilt = [math.cos(0.2), math.sin(0.2), 0, 0]
        transform = pose_transform([1, 2, 3], tilt)
        np.testing.assert_allclose(
            transform @ [0, 1, 0, 1], [1, 2 + math.cos(0.4), 3 + math.sin(0.4), 1]
        )
        np.testing.assert_allclose(
            pose_transform([0, 0, 0], q(math.pi / 2))[:3, :3] @ [1, 0, 0],
            [0, 1, 0],
            atol=1e-12,
        )

    def test_only_measured_reference_and_current_odometry_used_even_with_poisoned_archives(
        self,
    ):
        with tempfile.TemporaryDirectory() as directory:
            poisoned = Path(directory) / "native-and-GT.npz"
            np.savez(
                poisoned,
                native_pose=np.array([object()], dtype=object),
                T_map_sensor=np.array([object()], dtype=object),
            )
            ref = reference()
            ref["source"]["bag"] = str(poisoned)
            with (
                patch(
                    "numpy.load",
                    side_effect=AssertionError("native or GT archive read"),
                ),
                patch.object(
                    Path,
                    "read_bytes",
                    side_effect=AssertionError("unexpected archive read"),
                ),
            ):
                route = self.tracker(ref)
                command, receipt = self.tick(route)
                self.assertGreater(command[0], 0)
                self.assertEqual(receipt["aligned_position_xyz_m"], [0, 0, 1])
            with self.assertRaises(TypeError):
                route.tick(
                    7,
                    7,
                    (0, 0, 1),
                    q(0),
                    (0, 0, 0),
                    0,
                    True,
                    100,
                    100,
                    native_position=(1000, 0, 0),
                )

    def test_delegate_behavior_matches_existing_controller_on_measured_route(self):
        wrapped = self.tracker(speed_mps=0.12)
        direct = RouteTracker([[4, 0, 1]], [0, 0], speed=0.12)
        for t, p, yaw in [
            (7, (0, 0, 1), 0),
            (7.2, (0.1, 0.1, 1), 0.05),
            (7.4, (0.2, -0.1, 1), -0.1),
        ]:
            actual, _ = self.tick(wrapped, t=t, p=p, yaw=yaw)
            expected = direct.tick(t, p, yaw, 0, 0)
            np.testing.assert_allclose(actual, expected)
            self.assertLessEqual(abs(actual[0]), 0.12)
            self.assertLessEqual(abs(actual[1]), 0.05)
            self.assertLessEqual(abs(actual[2]), 0.15)

    def test_stale_invalid_and_wrong_frame_stop_and_reset_continuous_evidence(self):
        scenarios = [
            ({"odom_stamp_s": 6.49}, "stale_sim_odometry"),
            ({"arrival_wall": 106.49}, "stale_wall_odometry"),
            ({"odom_stamp_s": 7.000003}, "future_odometry"),
            ({"arrival_wall": 107.01}, "future_wall_arrival"),
            ({"frame_id": "native_map"}, "frame_mismatch"),
            ({"child_frame_id": "base_link"}, "frame_mismatch"),
            ({"quat_wxyz": [0, 0, 0, 0]}, "invalid_measured_odometry"),
            ({"position": (float("nan"), 0, 1)}, "invalid_measured_odometry"),
            ({"estimated_velocity": (float("inf"), 0, 0)}, "invalid_measured_odometry"),
            ({"measured_yaw_rate": float("nan")}, "invalid_measured_odometry"),
        ]
        for kwargs, reason in scenarios:
            with self.subTest(reason=reason):
                route = self.tracker()
                route.tracker.since, route.tracker.stall = 6.0, (6.0, 0)
                command, receipt = self.tick(route, **kwargs)
                self.assertEqual(command, [0, 0, 0])
                self.assertEqual(receipt["reason"], reason)
                self.assertIsNone(route.tracker.since)
                self.assertIsNone(route.tracker.stall)
                self.assertIsNone(receipt["alignment"])

    def test_same_physics_tick_float32_stamp_tolerates_only_numeric_future(self):
        route = self.tracker()
        # This is the observed float32 quantization shape, not pose extrapolation.
        stamp = float(np.float32(72.915))
        command, receipt = self.tick(route, t=72.915, odom_stamp_s=stamp)
        self.assertGreater(command[0], 0)
        self.assertLess(receipt["raw_sim_age_s"], 0)
        self.assertEqual(receipt["sim_age_s"], 0)
        self.assertLessEqual(-receipt["raw_sim_age_s"], 2e-6)
        command, receipt = self.tick(self.tracker(), t=72.915, odom_stamp_s=72.916)
        self.assertEqual(command, [0, 0, 0])
        self.assertEqual(receipt["reason"], "future_odometry")

    def test_actual_sdk_horizon_precision_parameters_accept_rounded_same_tick(self):
        from embodied_navigation.simulation.dynanav.experiment import (
            float32_sensor_time_tolerance,
        )

        for horizon in (100.0, 300.0):
            with self.subTest(horizon=horizon):
                tolerance = float32_sensor_time_tolerance(horizon)
                half_ulp = float(np.spacing(np.float32(horizon))) / 2
                self.assertAlmostEqual(tolerance, half_ulp + 1e-7)
                self.assertGreater(tolerance, 2e-6)
                parameters = {
                    "motion_end_s": horizon + 1,
                    "max_sim_age_s": 0.25,
                    "max_wall_age_s": 0.25,
                    "future_tolerance_s": tolerance,
                }
                route = self.tracker(**parameters)
                # Use the real producer's float32 conversion near the largest
                # upward rounding error for the configured experiment horizon.
                sim = float(np.nextafter(horizon - half_ulp, horizon))
                sdk_stamp = float(np.float32(sim))
                source_ns = round(sdk_stamp * 1e9)
                # Capture writes integer sec/nanosec, and latest odometry reads
                # them back to seconds before calling the route adapter.
                odom_stamp = (
                    source_ns // 1_000_000_000 + (source_ns % 1_000_000_000) * 1e-9
                )
                self.assertGreater(odom_stamp - sim, 2e-6)
                command, receipt = self.tick(route, t=sim, odom_stamp_s=odom_stamp)
                self.assertGreater(command[0], 0)
                self.assertEqual(receipt["sim_age_s"], 0)
                self.assertEqual(receipt["parameters"]["future_tolerance_s"], tolerance)
                self.assertLessEqual(odom_stamp - sim, tolerance)

                # This narrow producer precision allowance does not admit a
                # real 1ms future sample or a sample 1ms past freshness budget.
                for stamp, reason in (
                    (sim + 0.001, "future_odometry"),
                    (sim - 0.251, "stale_sim_odometry"),
                ):
                    command, receipt = self.tick(
                        self.tracker(**parameters), t=sim, odom_stamp_s=stamp
                    )
                    self.assertEqual(command, [0, 0, 0])
                    self.assertEqual(receipt["reason"], reason)

    def test_future_precision_tolerance_is_finite_nonnegative_and_below_freshness(self):
        for tolerance in (-1, float("nan"), float("inf"), True, 0.25, 0.251):
            with self.subTest(tolerance=tolerance), self.assertRaises(ValueError):
                self.tracker(max_sim_age_s=0.25, future_tolerance_s=tolerance)
        for tolerance in (0, 2e-6, 15.3587890625e-6, 0.249):
            with self.subTest(tolerance=tolerance):
                route = self.tracker(max_sim_age_s=0.25, future_tolerance_s=tolerance)
                self.assertEqual(route.parameters["future_tolerance_s"], tolerance)

    def test_duplicate_estimate_cannot_create_dwell_and_stale_interval_restarts_it(
        self,
    ):
        route = self.tracker()
        self.tick(route)
        self.tick(route, t=7.1, p=(3.85, 0, 1))
        self.assertEqual(route.phase, "brake")
        _, receipt = self.tick(route, t=7.5, odom_stamp_s=7.1, p=(3.85, 0, 1))
        self.assertFalse(receipt["distinct_measurement"])
        self.assertFalse(route.arrived)
        self.tick(route, t=7.7, odom_stamp_s=7.1, p=(3.85, 0, 1))
        self.assertIsNone(route.tracker.since)
        self.tick(route, t=7.8, p=(3.85, 0, 1))
        self.assertFalse(route.arrived)
        self.tick(route, t=8.1, p=(3.85, 0, 1))
        self.tick(route, t=8.4, p=(3.85, 0, 1))
        self.assertTrue(route.arrived)

    def test_processing_latency_cannot_credit_unobserved_stop_dwell(self):
        route = self.tracker()
        self.tick(route)
        self.tick(route, t=8, p=(3.85, 0, 1))
        self.assertEqual(route.phase, "brake")
        # Processing advanced half a second but measured evidence only 1us.
        self.tick(route, t=8.500001, odom_stamp_s=8.000001, p=(3.85, 0, 1))
        self.assertFalse(route.arrived)
        self.tick(route, t=8.6, odom_stamp_s=8.2, p=(3.85, 0, 1))
        self.assertFalse(route.arrived)
        self.tick(route, t=8.7, odom_stamp_s=8.6, p=(3.85, 0, 1))
        self.assertTrue(route.arrived)

    def test_missing_data_new_failure_and_deadline_never_retain_nonzero_command(self):
        route = self.tracker()
        self.tick(route)
        command, _ = self.tick(route, t=7.1, p=(0, 0.7, 1))
        self.assertEqual(route.failure, "route_cross_track_bound")
        self.assertEqual(command, [0, 0, 0])
        route = self.tracker(motion_end_s=8)
        self.tick(route)
        command, _ = self.tick(route, t=8, odom_stamp_s=None)
        self.assertEqual(command, [0, 0, 0])
        self.assertEqual(route.failure, "route_motion_budget")

    def test_existing_arrival_remains_terminal_after_motion_budget(self):
        route = self.tracker(motion_end_s=9)
        self.tick(route)
        for t in (7.1, 7.3, 7.5, 7.7):
            self.tick(route, t=t, p=(3.85, 0, 1))
        self.assertTrue(route.arrived)
        command, receipt = self.tick(route, t=10)
        self.assertEqual(command, [0, 0, 0])
        self.assertIsNone(route.failure)
        self.assertEqual(receipt["reason"], "arrived")
        self.assertIsNotNone(
            receipt["last_valid_measurement"]["reported_twist_speed_mps"]
        )

    def test_wrong_floor_cannot_arrive_from_measured_xyz(self):
        route = self.tracker()
        self.tick(route)
        self.tick(route, t=7.1, p=(3.85, 0, 2))
        self.tick(route, t=7.3, p=(3.85, 0, 2))
        self.assertEqual(route.phase, "cruise")
        self.assertFalse(route.arrived)

    def test_pose_window_speed_uses_distinct_measured_positions_and_preserves_twist(
        self,
    ):
        route = self.tracker(speed_source="pose_window", pose_speed_window_s=0.5)
        _, receipt = self.tick(route, estimated_velocity=(0.08, 0, 0))
        self.assertIsNone(receipt["pose_window_speed_mps"])
        self.assertIsNone(receipt["estimated_speed_mps"])
        self.assertFalse(receipt["speed_history_ready"])
        self.assertGreaterEqual(receipt["controller_stop_check_speed_mps"], 0.04)
        for t, x in [(7.2, 0.02), (7.4, 0.04), (7.6, 0.06)]:
            _, receipt = self.tick(
                route, t=t, p=(x, 0, 1), estimated_velocity=(0.08, 0, 0)
            )
        self.assertAlmostEqual(receipt["pose_window_speed_mps"], 0.1)
        self.assertAlmostEqual(receipt["reported_twist_speed_mps"], 0.08)
        self.assertAlmostEqual(receipt["estimated_speed_mps"], 0.1)
        self.assertTrue(receipt["speed_history_ready"])
        self.assertLessEqual(len(route._pose_history), 4)
        _, receipt = self.tick(
            route,
            t=7.7,
            odom_stamp_s=7.6,
            p=(0.06, 0, 1),
            estimated_velocity=(0.08, 0, 0),
        )
        self.assertFalse(receipt["distinct_measurement"])
        self.assertAlmostEqual(receipt["pose_window_speed_mps"], 0.1)
        self.tick(route, t=8.2, odom_stamp_s=7.6, p=(0.06, 0, 1))
        self.assertEqual(len(route._pose_history), 0)
        _, receipt = self.tick(route, t=8.3, p=(0.06, 0, 1))
        self.assertFalse(receipt["speed_history_ready"])

    def test_selected_pose_speed_can_confirm_stop_without_relabeling_noisy_twist(self):
        route = self.tracker(speed_source="pose_window")
        self.tick(route)
        for t in [7.1, 7.3, 7.5, 7.7, 7.9, 8.1, 8.3, 8.5]:
            command, _receipt = self.tick(
                route, t=t, p=(3.85, 0, 1), estimated_velocity=(0.08, 0, 0)
            )
        self.assertTrue(route.arrived)
        self.assertEqual(command, [0, 0, 0])
        # Read the fresh receipt which actually established the stop.
        fresh = self.tracker(speed_source="twist")
        self.tick(fresh)
        for t in [7.1, 7.3, 7.5, 7.7, 7.9, 8.1, 8.3, 8.5]:
            self.tick(fresh, t=t, p=(3.85, 0, 1), estimated_velocity=(0.08, 0, 0))
        self.assertFalse(fresh.arrived)

    def test_clock_reset_and_ready_false_clear_pose_window(self):
        for kwargs in (
            {"t": 6.9},
            {"t": 7.3, "ready": False},
            {"t": 7.3, "odom_stamp_s": 6.99},
        ):
            route = self.tracker(speed_source="pose_window")
            self.tick(route)
            self.tick(route, t=7.2, p=(0.01, 0, 1))
            command, _ = self.tick(route, **kwargs)
            self.assertEqual(command, [0, 0, 0])
            self.assertEqual(len(route._pose_history), 0)


class ReferenceValidationTests(unittest.TestCase):
    def test_hash_is_reference_content_not_a_source_or_truth_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference.json"
            path.write_text(json.dumps(reference()))
            loaded = load_slam_route_reference(path)
            self.assertEqual(loaded["reference_file"], str(path))
            self.assertEqual(len(loaded["reference_sha256"]), 64)
            original = loaded["reference_sha256"]
            path.write_text(json.dumps(reference()) + "\n")
            self.assertNotEqual(
                load_slam_route_reference(path)["reference_sha256"], original
            )

    def test_correspondence_pose_frames_and_hash_fail_closed(self):
        changes = [
            lambda r: r["source"].update(map_frame=""),
            lambda r: r["source"]["files"][0].update(sha256="wrong"),
            lambda r: r["source_correspondence"].update(goal_stamps_ns=[123]),
            lambda r: r["goals"][0].update(stamp_ns=r["anchor"]["stamp_ns"]),
            lambda r: r["goals"][0].update(
                position_xyz_m=r["anchor"]["position_xyz_m"][:]
            ),
            lambda r: r["anchor"].update(quaternion_wxyz=[1, 1, 0, 0]),
            lambda r: r.update(pose_source="native"),
            lambda r: r.update(no_gt_pose_used=False),
            lambda r: r["source"].update(
                first_stamp_ns=0, last_stamp_ns=10_000_000_000
            ),
            lambda r: r["source"].update(
                first_stamp_ns=1_000_000_000, last_stamp_ns=9_000_000_000
            ),
            lambda r: r["anchor"].update(
                source_message={
                    "storage": "truth.npz",
                    "message_id": 1,
                    "bag_timestamp_ns": 100,
                }
            ),
        ]
        for mutate in changes:
            ref = deepcopy(reference())
            mutate(ref)
            with self.subTest(reference=ref), self.assertRaises(ValueError):
                validate_slam_route_reference(ref)


if __name__ == "__main__":
    unittest.main()
