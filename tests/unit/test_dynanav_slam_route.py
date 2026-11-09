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


def post_vio_reference(goals=((4, 0, 1),)):
    ref = reference()
    ref["pose_source"] = "livo_post_vio"
    ref["source"].update(pose_stage="post_vio", pose_only=True, twist_available=False)
    ref["goals"] = [
        measured_pose((index + 1) * 10_000_000_000, point)
        for index, point in enumerate(goals)
    ]
    ref["source_correspondence"]["goal_stamps_ns"] = [
        goal["stamp_ns"] for goal in ref["goals"]
    ]
    for pose in [ref["anchor"], *ref["goals"]]:
        pose.update(
            twist_available=False,
            estimated_velocity_xyz_mps=None,
            estimated_yaw_rate_rps=None,
        )
    for goal in ref["goals"]:
        goal["stop_evidence"] = {
            "method": "measured_pose_window",
            "twist_used_to_confirm_stop": False,
            "start_stamp_ns": goal["stamp_ns"] - 1_000_000_000,
            "end_stamp_ns": goal["stamp_ns"],
            "sample_count": 11,
            "duration_s": 1.0,
            "maximum_sample_gap_s": 0.1,
            "maximum_distance_from_endpoint_m": 0.01,
            "maximum_yaw_difference_from_endpoint_rad": 0.01,
            "parameters": {
                "dwell_s": 1.0,
                "position_range_m": 0.03,
                "yaw_range_rad": 0.03,
                "max_sample_gap_s": 0.25,
            },
        }
    return ref


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

    def test_source_binding_uses_dedicated_final_topic_and_rejects_pre_vio(self):
        from embodied_navigation.simulation.dynanav.experiment import (
            slam_odometry_topic,
        )

        self.assertEqual(
            slam_odometry_topic("lio_estimate"),
            "/dynanav/lio_observer/aft_mapped_to_init",
        )
        self.assertEqual(
            slam_odometry_topic("livo_post_vio"),
            "/dynanav/lio_observer/post_vio_odometry",
        )
        topic = "/dynanav/livo_navigation/post_vio_odometry"
        self.assertEqual(slam_odometry_topic("livo_post_vio", topic), topic)
        for requested in (
            "/dynanav/lio_observer/aft_mapped_to_init",
            "",
            "relative",
            "/bad topic",
        ):
            with self.subTest(requested=requested), self.assertRaises(ValueError):
                slam_odometry_topic("livo_post_vio", requested)

    def test_shared_profile_factory_binds_pose_only_and_explicit_capture_parameters(
        self,
    ):
        from embodied_navigation.simulation.dynanav.experiment import (
            make_slam_controller,
        )

        profile = {
            "slam_route_reference": post_vio_reference(),
            "speed_mps": 0.15,
            "motion_end_s": 295.0,
            "repeat_spawn_assumption": True,
            "map_registration_enabled": False,
            "slam_future_tolerance_s": 2e-6,
            "slam_speed_source": "pose_window",
            "slam_pose_speed_window_s": 0.5,
            "slam_capture_remaining_m": 0.05,
            "slam_capture_distance_m": 0.10,
        }
        route = make_slam_controller(profile)
        self.assertFalse(route.twist_available)
        self.assertEqual(route.parameters["pose_source"], "livo_post_vio")
        self.assertEqual(route.tracker.capture_remaining_m, 0.05)
        self.assertEqual(route.tracker.capture_distance_m, 0.10)

    def test_post_vio_pose_only_never_coerces_unavailable_twist(self):
        class PoisonTwist:
            def __array__(self, *args, **kwargs):
                raise AssertionError("unavailable twist must not be consumed")

        for unavailable in (PoisonTwist(), (float("nan"),) * 3, None):
            with self.subTest(unavailable=unavailable):
                route = self.tracker(post_vio_reference(), speed_source="pose_window")
                command, receipt = self.tick(route, estimated_velocity=unavailable)
                self.assertGreater(command[0], 0)
                self.assertEqual(receipt["pose_source"], "livo_post_vio")
                self.assertFalse(receipt["twist_available"])
                self.assertIsNone(receipt["reported_twist_speed_mps"])
                for t in (7.2, 7.4, 7.6, 7.8, 8.0, 8.2, 8.4):
                    command, receipt = self.tick(
                        route, t=t, p=(3.85, 0, 1), estimated_velocity=unavailable
                    )
                self.assertTrue(route.arrived)
                self.assertEqual(command, [0, 0, 0])

    def test_missing_sdk_gyro_holds_and_clears_continuous_stop_evidence(self):
        route = self.tracker(post_vio_reference(), speed_source="pose_window")
        self.tick(route, estimated_velocity=None)
        route.tracker.since = 7.0
        command, receipt = self.tick(
            route, t=7.2, measured_yaw_rate=None, estimated_velocity=None
        )
        self.assertEqual(command, [0, 0, 0])
        self.assertEqual(receipt["reason"], "invalid_measured_odometry")
        self.assertIsNone(route.tracker.since)
        self.assertFalse(route.arrived)

    def test_post_vio_three_goals_two_turns_and_final_stop_from_distinct_poses(self):
        route = self.tracker(
            post_vio_reference(((2, 0, 1), (2, 2, 1), (4, 2, 2))),
            speed_source="pose_window",
            capture_remaining_m=0.05,
            capture_distance_m=0.10,
        )
        self.tick(route, estimated_velocity=None)
        t = 7.0
        for point, heading, next_heading in (
            ((1.96, 0, 1), 0, math.pi / 2),
            ((2, 1.96, 1), math.pi / 2, 0),
            ((3.96, 2, 2), 0, None),
        ):
            for _ in range(8):
                t += 0.2
                self.tick(route, t=t, p=point, yaw=heading, estimated_velocity=None)
            if next_heading is not None:
                self.assertEqual(route.phase, "turn")
                for _ in range(5):
                    t += 0.2
                    self.tick(
                        route, t=t, p=point, yaw=next_heading, estimated_velocity=None
                    )
                self.assertEqual(route.phase, "cruise")
        self.assertTrue(route.arrived)
        self.assertEqual(route.stage, 2)
        self.assertEqual(
            sum(event["event"] == "turn_completed" for event in route.events), 2
        )

    def test_capture_bounds_are_explicit_per_tracker_and_old_defaults_survive(self):
        default = self.tracker()
        tight = self.tracker(capture_remaining_m=0.05, capture_distance_m=0.10)
        for route in (default, tight):
            self.tick(route)
            self.tick(route, t=7.2, p=(3.85, 0, 1))
        self.assertEqual(default.phase, "brake")
        self.assertEqual(tight.phase, "cruise")
        self.assertEqual(default.parameters["capture_remaining_m"], 0.20)
        self.assertEqual(tight.parameters["capture_remaining_m"], 0.05)
        for values in (
            (0, 0.1),
            (0.2, 0.1),
            (0.2, 0.5),
            (True, 0.3),
            (float("nan"), 0.3),
        ):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.tracker(
                    capture_remaining_m=values[0], capture_distance_m=values[1]
                )

    def test_final_capture_changes_only_last_goal_and_retains_pose_stop_floor_and_dwell(
        self,
    ):
        route = self.tracker(
            post_vio_reference(((2, 0, 1), (4, 0, 1))),
            speed_source="pose_window",
            capture_remaining_m=0.05,
            capture_distance_m=0.10,
            final_capture_remaining_m=0.15,
            final_capture_distance_m=0.20,
        )
        self.tick(route, estimated_velocity=None)
        command, _ = self.tick(route, t=7.2, p=(1.85, 0, 1), estimated_velocity=None)
        self.assertEqual(route.stage, 0)
        self.assertEqual(route.phase, "cruise")
        self.assertGreater(command[0], 0)
        self.assertFalse(route.arrived)

        # Reach the intermediate point using distinct measured pose samples.
        # Its stop and turn dwell must complete before the final stage begins.
        t = 7.2
        for _ in range(14):
            t += 0.2
            self.tick(route, t=t, p=(1.96, 0, 1), estimated_velocity=None)
            if route.stage == 1:
                break
        self.assertEqual(route.stage, 1)
        self.assertEqual(route.phase, "cruise")
        t += 0.2
        command, receipt = self.tick(
            route, t=t, p=(3.902, 0, 1), estimated_velocity=None
        )
        self.assertEqual(route.phase, "brake")
        self.assertAlmostEqual(route.events[-1]["distance"], 0.098)
        self.assertGreater(receipt["pose_window_speed_mps"], 0.04)
        self.assertIsNone(route.tracker.since)
        self.assertFalse(route.arrived)

        # A stationary pose on another floor cannot contribute stop dwell.
        for _ in range(4):
            t += 0.2
            _, receipt = self.tick(route, t=t, p=(3.902, 0, 2), estimated_velocity=None)
            self.assertFalse(route.arrived)
            self.assertIsNone(route.tracker.since)
        self.assertTrue(receipt["speed_history_ready"])
        self.assertAlmostEqual(receipt["pose_window_speed_mps"], 0)

        # Returning to the correct floor first needs a fresh measured speed
        # window, then another half-second of continuous low-speed evidence.
        for _ in range(4):
            t += 0.2
            _, receipt = self.tick(route, t=t, p=(3.902, 0, 1), estimated_velocity=None)
            self.assertFalse(route.arrived)
        self.assertTrue(receipt["speed_history_ready"])
        self.assertAlmostEqual(receipt["pose_window_speed_mps"], 0)
        self.assertIsNotNone(route.tracker.since)
        self.assertLess(t - route.tracker.since, 0.5)
        for _ in range(3):
            t += 0.2
            command, receipt = self.tick(
                route, t=t, p=(3.902, 0, 1), estimated_velocity=None
            )
        self.assertTrue(route.arrived)
        self.assertEqual(command, [0, 0, 0])
        self.assertFalse(receipt["twist_available"])
        self.assertIsNone(receipt["reported_twist_speed_mps"])
        self.assertTrue(route.tracker.stop_on_stall)
        self.assertIsNone(route.failure)

    def test_single_goal_uses_final_capture_bounds(self):
        final = self.tracker(
            post_vio_reference(),
            speed_source="pose_window",
            capture_remaining_m=0.05,
            capture_distance_m=0.10,
            final_capture_remaining_m=0.15,
            final_capture_distance_m=0.20,
        )
        tight = self.tracker(
            post_vio_reference(),
            speed_source="pose_window",
            capture_remaining_m=0.05,
            capture_distance_m=0.10,
        )
        for route in (final, tight):
            self.tick(route, estimated_velocity=None)
            self.tick(route, t=7.2, p=(3.902, 0, 1), estimated_velocity=None)
        self.assertEqual(final.stage, 0)
        self.assertEqual(final.phase, "brake")
        self.assertEqual(tight.phase, "cruise")
        self.assertFalse(final.arrived)
        self.assertEqual(final.tracker.final_capture_remaining_m, 0.15)
        self.assertEqual(final.tracker.final_capture_distance_m, 0.20)

    def test_none_final_capture_preserves_existing_defaults_and_explicit_base_bounds(
        self,
    ):
        for remaining, distance in ((0.20, 0.30), (0.05, 0.10)):
            with self.subTest(remaining=remaining, distance=distance):
                options = {
                    "speed_source": "pose_window",
                    "capture_remaining_m": remaining,
                    "capture_distance_m": distance,
                }
                previous = self.tracker(post_vio_reference(), **options)
                explicit_none = self.tracker(
                    post_vio_reference(),
                    **options,
                    final_capture_remaining_m=None,
                    final_capture_distance_m=None,
                )
                self.assertIsNone(explicit_none.tracker.final_capture_remaining_m)
                self.assertIsNone(explicit_none.tracker.final_capture_distance_m)
                self.assertIsNone(explicit_none.parameters["final_capture_remaining_m"])
                self.assertIsNone(explicit_none.parameters["final_capture_distance_m"])
                self.assertEqual(
                    explicit_none.parameters["effective_final_capture_remaining_m"],
                    remaining,
                )
                self.assertEqual(
                    explicit_none.parameters["effective_final_capture_distance_m"],
                    distance,
                )
                for t, x in (
                    (7.0, 0),
                    (7.2, 3.85),
                    (7.4, 3.96),
                    (7.6, 3.96),
                    (7.8, 3.96),
                ):
                    actual, _ = self.tick(
                        explicit_none, t=t, p=(x, 0, 1), estimated_velocity=None
                    )
                    expected, _ = self.tick(
                        previous, t=t, p=(x, 0, 1), estimated_velocity=None
                    )
                    self.assertEqual(actual, expected)
                    self.assertEqual(explicit_none.phase, previous.phase)
                    self.assertEqual(explicit_none.arrived, previous.arrived)

    def test_final_capture_validates_effective_bounds_and_independent_inheritance(self):
        for remaining, distance, effective_remaining, effective_distance in (
            (None, None, 0.05, 0.10),
            (0.08, None, 0.08, 0.10),
            (None, 0.20, 0.05, 0.20),
            (0.15, 0.20, 0.15, 0.20),
            (0.40, 0.40, 0.40, 0.40),
        ):
            with self.subTest(remaining=remaining, distance=distance):
                route = self.tracker(
                    capture_remaining_m=0.05,
                    capture_distance_m=0.10,
                    final_capture_remaining_m=remaining,
                    final_capture_distance_m=distance,
                )
                self.assertEqual(route.tracker.final_capture_remaining_m, remaining)
                self.assertEqual(route.tracker.final_capture_distance_m, distance)
                self.assertEqual(
                    route.parameters["effective_final_capture_remaining_m"],
                    effective_remaining,
                )
                self.assertEqual(
                    route.parameters["effective_final_capture_distance_m"],
                    effective_distance,
                )

        for remaining, distance in (
            (0, 0.20),
            (-0.01, 0.20),
            (0.21, 0.20),
            (0.10, 0.401),
            (0.15, None),
            (None, 0.04),
            (True, 0.20),
            (0.10, False),
            (float("nan"), 0.20),
            (0.10, float("inf")),
            ("0.10", 0.20),
            (0.10, []),
        ):
            options = {
                "capture_remaining_m": 0.05,
                "capture_distance_m": 0.10,
                "final_capture_remaining_m": remaining,
                "final_capture_distance_m": distance,
            }
            with self.subTest(remaining=remaining, distance=distance):
                with self.assertRaises(ValueError):
                    self.tracker(**options)
                with self.assertRaises(ValueError):
                    RouteTracker([[4, 0, 1]], [0, 0], **options)

    def test_profile_factory_forwards_explicit_and_inherited_final_capture_options(
        self,
    ):
        from embodied_navigation.simulation.dynanav.experiment import (
            make_slam_controller,
        )

        profile = {
            "slam_route_reference": post_vio_reference(),
            "speed_mps": 0.15,
            "motion_end_s": 295.0,
            "repeat_spawn_assumption": True,
            "map_registration_enabled": False,
            "slam_future_tolerance_s": 2e-6,
            "slam_speed_source": "pose_window",
            "slam_pose_speed_window_s": 0.5,
            "slam_capture_remaining_m": 0.05,
            "slam_capture_distance_m": 0.10,
        }
        for overrides, expected in (
            ({}, (None, None)),
            (
                {
                    "slam_final_capture_remaining_m": None,
                    "slam_final_capture_distance_m": None,
                },
                (None, None),
            ),
            (
                {
                    "slam_final_capture_remaining_m": 0.15,
                    "slam_final_capture_distance_m": 0.20,
                },
                (0.15, 0.20),
            ),
            ({"slam_final_capture_distance_m": 0.20}, (None, 0.20)),
        ):
            with (
                self.subTest(overrides=overrides),
                patch(
                    "embodied_navigation.simulation.dynanav.slam_route.SlamRouteTracker"
                ) as factory,
            ):
                make_slam_controller({**profile, **overrides})
                kwargs = factory.call_args.kwargs
                self.assertEqual(kwargs["final_capture_remaining_m"], expected[0])
                self.assertEqual(kwargs["final_capture_distance_m"], expected[1])


class ReferenceValidationTests(unittest.TestCase):
    def test_post_vio_contract_requires_explicit_stage_availability_and_stop_witness(
        self,
    ):
        valid = post_vio_reference()
        self.assertEqual(
            validate_slam_route_reference(valid)["pose_source"], "livo_post_vio"
        )
        with self.assertRaisesRegex(ValueError, "pose_window"):
            SlamRouteTracker(
                valid, repeat_spawn_assumption=True, map_registration_enabled=False
            )
        changes = (
            lambda ref: ref["source"].pop("pose_stage"),
            lambda ref: ref["source"].update(twist_available=True),
            lambda ref: ref["goals"][0].update(estimated_velocity_xyz_mps=[0, 0, 0]),
            lambda ref: ref["goals"][0].pop("stop_evidence"),
            lambda ref: ref["goals"][0]["stop_evidence"].update(
                twist_used_to_confirm_stop=True
            ),
            lambda ref: ref["goals"][0]["stop_evidence"].update(duration_s=0.2),
            lambda ref: ref["goals"][0]["stop_evidence"].update(
                maximum_sample_gap_s=0.3
            ),
        )
        for mutate in changes:
            ref = deepcopy(valid)
            mutate(ref)
            with self.subTest(reference=ref), self.assertRaises(ValueError):
                validate_slam_route_reference(ref)

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
