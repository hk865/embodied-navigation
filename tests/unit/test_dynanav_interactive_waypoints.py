"""Offline tests for the interactive waypoint exploration adapter.

CPU-only: no Isaac, no ROS, no GPU. Synthetic measured odometry plus a temporary
mailbox directory lock the operator-interface contract: bounded commands, strict
sequence monotonicity, request TTL, waypoint TTL, operator link loss, measured
pose-source gates, stale/future odometry holds, frame consistency, arrival and
failure receipts, and an observation bundle that carries no ground-truth pose.
"""

import json
import math
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from embodied_navigation.simulation.dynanav.capture.perception import (
    direction_coverage,
)
from embodied_navigation.simulation.dynanav.capture.control import (
    create_route_controller,
)
from embodied_navigation.simulation.dynanav.exploration_waypoints import (
    PROTOCOL_VERSION,
    InteractiveWaypointTracker,
    WaypointQueueController,
    create_interactive_tracker,
    validate_interactive_profile,
)


def make_profile(mailbox, **overrides):
    """Return one interactive exploration profile bound to a mailbox path."""
    profile = {
        "control_mode": "interactive_waypoint_exploration",
        "ground_truth_navigation": False,
        "slam_pose_source": "lio_estimate",
        "navigation_pose_source": "measured_lio",
        "policy": "robogauge",
        "render_backend": "fabric",
        "interactive_mailbox": str(mailbox),
        "speed_mps": 1.0,
        "motion_end_s": 300.0,
        "slam_twist_available": True,
        "slam_max_sim_age_s": 0.25,
        "slam_max_wall_age_s": 0.5,
        "slam_pose_speed_window_s": 0.5,
        "interactive_waypoint_ttl_s": 30.0,
        "interactive_link_timeout_s": 5.0,
        "interactive_pose_loss_timeout_s": 5.0,
        "interactive_yaw_completion_mode": "instant",
        "interactive_max_queue": 4,
        "interactive_max_segment_m": 8.0,
        "interactive_min_segment_m": 0.15,
        "interactive_capture_remaining_m": 0.20,
        "interactive_capture_distance_m": 0.30,
        "interactive_simulation_only": True,
        "stop_on_stall": True,
    }
    profile.update(overrides)
    return profile


def quat_from_yaw(yaw):
    """Return a WXYZ quaternion for a pure yaw rotation."""
    return [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]


class ProfileValidationTests(unittest.TestCase):
    """The profile contract must refuse anything that is not measured-pose."""

    def test_requires_interactive_control_mode(self):
        profile = make_profile("/tmp/dynanav-test/interactive")
        profile["control_mode"] = "slam_pose_diagnostic"
        with self.assertRaises(ValueError):
            validate_interactive_profile(profile)

    def test_refuses_ground_truth_navigation(self):
        profile = make_profile(
            "/tmp/dynanav-test/interactive", ground_truth_navigation=True
        )
        with self.assertRaises(ValueError):
            validate_interactive_profile(profile)

    def test_refuses_unmeasured_pose_source(self):
        profile = make_profile(
            "/tmp/dynanav-test/interactive", slam_pose_source="native_truth"
        )
        with self.assertRaises(ValueError):
            validate_interactive_profile(profile)

    def test_requires_absolute_mailbox(self):
        profile = make_profile("relative/interactive")
        with self.assertRaises(ValueError):
            validate_interactive_profile(profile)

    def test_requires_simulation_only_declaration(self):
        profile = make_profile(
            "/tmp/dynanav-test/interactive", interactive_simulation_only=False
        )
        with self.assertRaises(ValueError):
            validate_interactive_profile(profile)

    def test_rejects_speed_above_ceiling(self):
        profile = make_profile("/tmp/dynanav-test/interactive", speed_mps=1.5)
        with self.assertRaises(ValueError):
            validate_interactive_profile(profile)

    def test_rejects_inverted_capture_bounds(self):
        profile = make_profile(
            "/tmp/dynanav-test/interactive",
            interactive_capture_remaining_m=0.40,
            interactive_capture_distance_m=0.20,
        )
        with self.assertRaises(ValueError):
            validate_interactive_profile(profile)


class WaypointQueueControllerTests(unittest.TestCase):
    """The follower keeps the existing command envelope and fail-closed bounds."""

    def controller(self, **overrides):
        kwargs = {
            "speed_mps": 1.0,
            "motion_end_s": 300.0,
            "capture_remaining_m": 0.20,
            "capture_distance_m": 0.30,
        }
        kwargs.update(overrides)
        return WaypointQueueController(**kwargs)

    def test_commands_stay_inside_the_existing_envelope(self):
        controller = self.controller()
        self.assertTrue(controller.enqueue([5.0, 0.0, 0.0])[0])
        command = controller.tick(7.0, [0.0, 0.0, 0.0], 0.0, 1.0, 0.0)
        self.assertLessEqual(abs(command[0]), 1.0)
        self.assertLessEqual(abs(command[1]), 0.05)
        self.assertLessEqual(abs(command[2]), 0.15)
        self.assertGreater(command[0], 0.9)
        self.assertEqual("cruise", controller.phase)

    def test_intermediate_waypoint_is_passed_through(self):
        controller = self.controller()
        self.assertTrue(controller.enqueue([1.0, 0.0, 0.0])[0])
        self.assertTrue(controller.enqueue([2.0, 0.0, 0.0])[0])
        controller.tick(7.0, [0.0, 0.0, 0.0], 0.0, 1.0, 0.0)
        captured = controller.tick(7.2, [0.85, 0.0, 0.0], 0.0, 0.5, 0.0)
        self.assertEqual([0.0, 0.0, 0.0], captured)
        self.assertEqual(1, controller.completed_goals)
        next_command = controller.tick(7.4, [0.95, 0.0, 0.0], 0.0, 1.0, 0.0)
        self.assertGreater(next_command[0], 0.0)
        self.assertEqual("cruise", controller.phase)

    def test_goal_inside_capture_bounds_completes_without_forward_motion(self):
        controller = self.controller()
        self.assertTrue(controller.enqueue([0.10, 0.0, 0.0])[0])
        command = controller.tick(7.0, [0.0, 0.0, 0.0], 0.0, 1.0, 0.0)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual(1, controller.completed_goals)
        self.assertEqual("idle", controller.phase)

    def test_cross_track_bound_fails_closed(self):
        controller = self.controller()
        self.assertTrue(controller.enqueue([5.0, 0.0, 0.0])[0])
        controller.tick(7.0, [0.0, 0.0, 0.0], 0.0, 1.0, 0.0)
        command = controller.tick(7.2, [1.0, 0.9, 0.0], 0.0, 1.0, 0.0)
        self.assertEqual("route_cross_track_bound", controller.failure)
        self.assertEqual([0.0, 0.0, 0.0], command)

    def test_overshoot_fails_closed(self):
        controller = self.controller()
        self.assertTrue(controller.enqueue([1.0, 0.0, 0.0])[0])
        controller.tick(7.0, [0.0, 0.0, 0.0], 0.0, 1.0, 0.0)
        command = controller.tick(7.2, [1.5, 0.0, 0.0], 0.0, 1.0, 0.0)
        self.assertEqual("route_waypoint_overshoot", controller.failure)
        self.assertEqual([0.0, 0.0, 0.0], command)

    def test_stall_becomes_failure_when_enabled(self):
        controller = self.controller()
        self.assertTrue(controller.enqueue([5.0, 0.0, 0.0])[0])
        controller.tick(7.0, [0.0, 0.0, 0.0], 0.0, 1.0, 0.0)
        for tenth in range(1, 100):
            time_s = 7.0 + tenth * 0.2
            controller.tick(time_s, [0.0, 0.0, 0.0], 0.0, 1.0, 0.0)
            if controller.failure is not None:
                break
        self.assertEqual("route_low_progress_stall", controller.failure)

    def test_motion_budget_deadline(self):
        controller = self.controller(motion_end_s=10.0)
        self.assertTrue(controller.enqueue([5.0, 0.0, 0.0])[0])
        command = controller.tick(10.0, [0.0, 0.0, 0.0], 0.0, 1.0, 0.0)
        self.assertEqual("route_motion_budget", controller.failure)
        self.assertEqual([0.0, 0.0, 0.0], command)

    def test_segment_length_and_queue_depth_bounds(self):
        controller = self.controller(max_queue=2)
        self.assertTrue(controller.enqueue([0.20, 0.0, 0.0])[0])
        accepted, reason, planar = controller.enqueue([9.0, 0.0, 0.0])
        self.assertFalse(accepted)
        self.assertEqual("segment_too_long", reason)
        self.assertAlmostEqual(8.8, planar, places=6)
        self.assertTrue(controller.enqueue([1.0, 0.0, 0.0])[0])
        accepted, reason, _ = controller.enqueue([2.0, 0.0, 0.0])
        self.assertFalse(accepted)
        self.assertEqual("queue_full", reason)

    def test_vertical_step_bound(self):
        controller = self.controller()
        self.assertTrue(controller.enqueue([1.0, 0.0, 0.0])[0])
        accepted, reason, _ = controller.enqueue([2.0, 0.0, 5.0])
        self.assertFalse(accepted)
        self.assertEqual("segment_vertical_jump", reason)

    def test_default_holonomic_mode_keeps_the_existing_lateral_command(self):
        controller = self.controller()
        self.assertTrue(controller.enqueue([5.0, 0.0, 0.0])[0])
        controller.tick(7.0, [0.0, 0.0, 0.0], 0.0, 1.0, 0.0)
        command = controller.tick(7.2, [1.0, 0.35, 0.0], 0.0, 0.5, 0.0)
        self.assertNotEqual(0.0, command[1])

    def test_forward_turn_mode_never_emits_a_lateral_command(self):
        controller = self.controller(allow_lateral=False)
        self.assertTrue(controller.enqueue([5.0, 0.0, 0.0])[0])
        controller.tick(7.0, [0.0, 0.0, 0.0], 0.0, 1.0, 0.0)
        command = controller.tick(7.2, [1.0, 0.35, 0.0], 0.2, 0.5, 0.0)
        self.assertEqual(0.0, command[1])
        self.assertNotEqual(0.0, command[2])
        self.assertFalse(controller.allow_lateral)


class MailboxCase(unittest.TestCase):
    """Base fixture: one run directory, one tracker and a controllable clock."""

    def setUp(self):
        self._temporary = tempfile.TemporaryDirectory()
        self.root = Path(self._temporary.name) / "run-20261110"
        self.mailbox = self.root / "interactive"
        self.profile = make_profile(
            self.mailbox, **getattr(type(self), "PROFILE_OVERRIDES", {})
        )
        self.tracker = InteractiveWaypointTracker(self.profile)
        self.now = 1000.0
        self.sim = 7.0
        self.position = [0.0, 0.0, 0.0]
        self.speed = 1.0
        # The real operator sends a heartbeat before any waypoint; the fixture does
        # the same unless a test asks for a missing/foreign/bad heartbeat.
        self.auto_heartbeat = True

    def tearDown(self):
        self._temporary.cleanup()

    def write_request(
        self,
        *,
        seq,
        action="goto",
        goal=None,
        ttl=30.0,
        created=None,
        pose_source=None,
        version=PROTOCOL_VERSION,
        run_id=None,
        **extra,
    ):
        payload = {
            "version": version,
            "seq": seq,
            "action": action,
            "created_wall_s": self.now if created is None else created,
            "ttl_s": ttl,
        }
        if goal is not None:
            payload["goal_xyz_m"] = list(goal)
        if pose_source is not None:
            payload["pose_source"] = pose_source
        if run_id is not None:
            payload["run_id"] = run_id
        payload.update(extra)
        self.tracker.request_path.write_text(json.dumps(payload), encoding="utf-8")
        return payload

    def write_heartbeat(self, wall=None, run_id=None):
        payload = {"wall_s": self.now if wall is None else wall}
        if run_id is not None:
            payload["run_id"] = run_id
        self.tracker.heartbeat_path.write_text(json.dumps(payload), encoding="utf-8")

    def step(
        self,
        *,
        advance=0.02,
        position=None,
        speed=None,
        yaw=0.0,
        ready=True,
        pose_source="lio_estimate",
        frame="map",
        sensor_frame="go2_base",
        sim=None,
        stamp=None,
        perception=2.0,
        perception_stamp=None,
        perception_source="static_depth",
        perception_frame="go2_base",
        observation_state="measured",
        depth_valid_ratio=1.0,
        directions=None,
        pose_future=0.0,
        yaw_rate=0.0,
        pose_stamp=None,
        imu_stamp=None,
        imu_arrival=None,
        imu_valid=True,
        imu_missing=False,
        heartbeat="auto",
    ):
        self.sim = self.sim + advance if sim is None else sim
        self.now = self.now + advance
        if heartbeat == "auto":
            if self.auto_heartbeat:
                self.write_heartbeat()
        elif heartbeat not in (False, None):
            self.write_heartbeat(wall=float(heartbeat))
        position = self.position if position is None else position
        speed = self.speed if speed is None else speed
        if directions is None:
            directions = {
                name: {
                    "observation_state": observation_state,
                    "clearance_m": 2.0 if observation_state != "insufficient_observation" else None,
                    "points": 100,
                }
                for name in ("front", "rear", "left", "right")
            }
        if perception is not None:
            self.tracker.update_perception(
                now_wall=self.now,
                stamp_s=self.sim if perception_stamp is None else perception_stamp,
                clearance_m=perception,
                source=perception_source,
                frame_id=perception_frame,
                points=4096,
                observation_state=observation_state,
                depth_valid_ratio=depth_valid_ratio,
                directions=directions,
            )
        return self.tracker.tick(
            self.sim,
            (
                pose_stamp
                if pose_stamp is not None
                else ((self.sim + pose_future) if stamp is None else stamp)
            ),
            position,
            quat_from_yaw(yaw),
            [speed, 0.0, 0.0],
            yaw_rate,
            ready,
            self.now,
            self.now,
            frame_id=frame,
            child_frame_id=sensor_frame,
            pose_source=pose_source,
            imu_source_stamp_s=(
                None if imu_missing else (self.sim if imu_stamp is None else imu_stamp)
            ),
            imu_arrival_wall_s=self.now if imu_arrival is None else imu_arrival,
            imu_frame="go2_imu",
            imu_valid=imu_valid,
        )

    def receipts(self):
        if not self.tracker.receipts_path.exists():
            return []
        return [
            json.loads(line)
            for line in self.tracker.receipts_path.read_text(
                encoding="utf-8"
            ).splitlines()
            if line.strip()
        ]

    def events(self):
        return [entry["event"] for entry in self.receipts()]

    def reasons(self):
        return [entry.get("reason") for entry in self.receipts()]


class WaypointRequestTests(MailboxCase):
    """Sequence, TTL and payload validation of the operator mailbox."""

    def test_accepted_waypoint_tracks_and_reaches(self):
        self.write_heartbeat()
        self.write_request(seq=1, goal=[1.0, 0.0, 0.0])
        command, receipt = self.step()
        self.assertIn("waypoint_accepted", self.events())
        self.assertGreaterEqual(command[0], 0.04)
        self.assertEqual("tracking", receipt["status"])
        position = 0.0
        for _ in range(17):
            position += 0.05
            command, receipt = self.step(position=[position, 0.0, 0.0], speed=0.5)
        self.assertIn("waypoint_reached", self.events())
        self.assertEqual(1, self.tracker.controller.completed_goals)
        self.assertFalse(self.tracker.arrived)
        self.assertIsNone(self.tracker.terminal_reason)
        self.assertEqual([0.0, 0.0, 0.0], command)

    def test_duplicate_sequence_is_rejected_and_not_requeued(self):
        self.write_request(seq=1, goal=[1.0, 0.0, 0.0])
        self.step()
        self.assertEqual(1, len(self.tracker.controller.queue))
        self.write_request(seq=1, goal=[2.0, 0.0, 0.0])
        # The mailbox is polled at 10 Hz, so a new request needs 0.1 s of wall time.
        self.step(advance=0.15)
        self.assertIn("duplicate_or_stale_sequence", self.reasons())
        self.assertEqual(1, len(self.tracker.controller.queue))

    def test_stale_sequence_is_rejected(self):
        self.write_request(seq=2, goal=[1.0, 0.0, 0.0])
        self.step()
        self.write_request(seq=1, goal=[2.0, 0.0, 0.0])
        self.step(advance=0.15)
        self.assertIn("duplicate_or_stale_sequence", self.reasons())

    def test_no_request_keeps_the_base_stopped(self):
        command, receipt = self.step()
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIsNone(receipt["terminal_reason"])
        self.assertEqual([], self.events())

    def test_request_expired_before_tracking(self):
        self.write_request(seq=1, goal=[1.0, 0.0, 0.0], created=self.now - 40.0, ttl=30.0)
        self.step()
        self.assertIn("request_expired", self.reasons())
        self.assertEqual(0, len(self.tracker.controller.queue))

    def test_ground_truth_waypoint_frame_is_rejected(self):
        self.write_request(seq=1, goal=[1.0, 0.0, 0.0], pose_source="native_truth")
        self.step()
        self.assertIn("ground_truth_waypoint_frame_forbidden", self.reasons())
        self.assertEqual(0, len(self.tracker.controller.queue))

    def test_waypoint_pose_source_must_match_the_measurement(self):
        self.write_request(seq=1, goal=[1.0, 0.0, 0.0], pose_source="livo_post_vio")
        self.step()
        self.assertIn("pose_source_mismatch", self.reasons())

    def test_invalid_version_is_rejected(self):
        self.write_request(seq=1, goal=[1.0, 0.0, 0.0], version=99)
        self.step()
        self.assertIn("unsupported_protocol_version", self.reasons())

    def test_invalid_action_is_rejected(self):
        self.write_request(seq=1, action="fly", goal=[1.0, 0.0, 0.0])
        self.step()
        self.assertIn("unsupported_action", self.reasons())

    def test_waypoint_beyond_the_absolute_bounds_is_rejected(self):
        self.write_request(seq=1, goal=[150.0, 0.0, 0.0])
        self.step()
        self.assertIn("goal_outside_xy_bounds", self.reasons())

    def test_active_waypoint_ttl_expiry_zeros_the_command(self):
        self.write_heartbeat()
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0], ttl=1.0)
        command, _ = self.step()
        self.assertGreater(command[0], 0.0)
        command, _ = self.step(advance=1.5)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("waypoint_expired", self.events())
        self.assertIsNone(self.tracker.terminal_reason)
        self.assertEqual(0, len(self.tracker.controller.queue))

    def test_first_leg_too_long_is_dropped_instead_of_driven(self):
        self.write_request(seq=1, goal=[50.0, 0.0, 0.0])
        command, receipt = self.step()
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("waypoint_dropped", self.events())
        self.assertIn("first_leg_too_long", self.reasons())
        self.assertIsNone(self.tracker.terminal_reason)
        self.assertEqual(0, len(self.tracker.controller.queue))

    def test_stop_action_is_terminal_with_zero_command(self):
        self.write_request(seq=1, goal=[1.0, 0.0, 0.0])
        self.step()
        self.write_request(seq=2, action="stop")
        command, receipt = self.step(advance=0.15)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("interactive_stop", self.tracker.terminal_reason)
        self.assertTrue(self.tracker.arrived)
        self.assertIn("stop_accepted", self.events())
        self.assertEqual("terminal", receipt["status"])

    def test_cancel_clears_the_queue_but_keeps_the_run_alive(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, _ = self.step()
        self.assertGreater(command[0], 0.0)
        self.write_request(seq=2, action="cancel")
        command, _ = self.step(advance=0.15)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIsNone(self.tracker.terminal_reason)
        self.assertIn("cancel_accepted", self.events())
        self.assertEqual(0, len(self.tracker.controller.queue))


class OperatorLinkTests(MailboxCase):
    """Missing, stale, future or malformed operator heartbeats must never drive."""

    def test_missing_heartbeat_holds_without_moving(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(heartbeat=False)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("operator_link_not_established", receipt["reason"])
        self.assertIsNone(self.tracker.terminal_reason)

    def test_a_missing_heartbeat_still_never_moves_later(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        for _ in range(5):
            command, _ = self.step(advance=2.0, heartbeat=False)
            self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIsNone(self.tracker.terminal_reason)

    def test_link_loss_after_heartbeat_is_terminal(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step()
        command, receipt = self.step(advance=6.0, heartbeat=False)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("interactive_link_lost", self.tracker.terminal_reason)
        self.assertIn("interactive_link_lost", self.reasons())
        self.assertEqual("terminal", receipt["status"])

    def test_heartbeat_from_the_future_is_ignored(self):
        self.write_heartbeat(wall=self.now + 30.0)
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, _ = self.step(heartbeat=False)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("heartbeat_from_future", self.reasons())
        self.assertFalse(self.tracker._link_established)

    def test_a_future_heartbeat_cannot_refresh_an_expired_link(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, _ = self.step()
        self.assertGreater(command[0], 0.0)
        self.assertTrue(self.tracker._link_established)
        # The last valid heartbeat is at self.now - 0.02 s. Future-dated files do
        # not refresh it, so the ordinary 5 s grace runs down and then expires.
        for _ in range(2):
            command, _ = self.step(advance=2.0, heartbeat=self.now + 3600.0)
            self.assertGreater(command[0], 0.0)
            self.assertIsNone(self.tracker.terminal_reason)
        command, _ = self.step(advance=2.0, heartbeat=self.now + 3600.0)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("interactive_link_lost", self.tracker.terminal_reason)

    def _established_tracker_with_payload(self, payload):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step()
        self.assertTrue(self.tracker._link_established)
        self.tracker.heartbeat_path.write_text(payload, encoding="utf-8")

    def test_list_heartbeat_does_not_raise_and_invalidates_the_link(self):
        self._established_tracker_with_payload("[]")
        command, _ = self.step(advance=0.1, heartbeat=False)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("interactive_link_unreadable", self.tracker.terminal_reason)
        self.assertIn("heartbeat_not_an_object", self.reasons())

    def test_empty_object_heartbeat_does_not_raise(self):
        self._established_tracker_with_payload("{}")
        command, _ = self.step(advance=0.1, heartbeat=False)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("interactive_link_unreadable", self.tracker.terminal_reason)

    def test_non_numeric_heartbeat_wall_does_not_raise(self):
        self._established_tracker_with_payload(json.dumps({"wall_s": "abc"}))
        command, _ = self.step(advance=0.1, heartbeat=False)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("interactive_link_unreadable", self.tracker.terminal_reason)
        self.assertIn("heartbeat_unreadable", self.reasons())

    def test_heartbeat_from_another_run_does_not_refresh_liveness(self):
        self._established_tracker_with_payload(
            json.dumps({"wall_s": self.now, "run_id": "another-run"})
        )
        # The foreign heartbeat keeps being rejected, so the deadline still counts
        # from the last valid heartbeat: 2 s and 4 s are inside the 5 s grace.
        for _ in range(2):
            command, _ = self.step(advance=2.0, heartbeat=False)
            self.assertGreater(command[0], 0.0)
            self.assertIsNone(self.tracker.terminal_reason)
        command, _ = self.step(advance=2.0, heartbeat=False)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("interactive_link_lost", self.tracker.terminal_reason)
        self.assertIn("heartbeat_run_id_mismatch", self.reasons())

    def test_unparseable_heartbeat_file_keeps_the_link_until_it_expires(self):
        self._established_tracker_with_payload("not json")
        command, _ = self.step(advance=0.1, heartbeat=False)
        self.assertGreater(command[0], 0.0)
        self.assertIsNone(self.tracker.terminal_reason)
        command, _ = self.step(advance=6.0, heartbeat=False)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("interactive_link_lost", self.tracker.terminal_reason)


class MeasurementGateTests(MailboxCase):
    """Measured-pose gates: source label, freshness, frames, readiness."""

    def test_ground_truth_odometry_label_holds(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(pose_source="native_truth")
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("hold", receipt["status"])
        self.assertEqual("pose_source_mismatch", receipt["reason"])

    def test_stale_odometry_holds_then_terminates(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(stamp=self.sim - 1.0)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("stale_sim_odometry", receipt["reason"])
        for _ in range(4):
            command, receipt = self.step(advance=2.0, stamp=self.sim - 1.0)
        self.assertEqual("interactive_pose_lost", self.tracker.terminal_reason)
        self.assertEqual([0.0, 0.0, 0.0], command)

    def test_future_odometry_holds(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(stamp=self.sim + 1.0)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("future_odometry", receipt["reason"])

    def test_not_ready_holds(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(ready=False)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("not_ready", receipt["reason"])

    def test_unlabelled_frames_hold(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(frame="", sensor_frame="")
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("unlabelled_measurement_frames", receipt["reason"])

    def test_measurement_frames_are_latched_then_enforced(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step(frame="map", sensor_frame="go2_base")
        command, receipt = self.step(frame="odom", sensor_frame="go2_base")
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("measurement_frame_mismatch", receipt["reason"])

    def test_invalid_quaternion_holds(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.write_heartbeat()
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.tracker.tick(
            self.sim, self.sim, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0], 0.0, True, self.now, self.now,
            frame_id="map", child_frame_id="go2_base", pose_source="lio_estimate",
        )
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("invalid_measured_odometry", receipt["reason"])


class PerceptionGateTests(MailboxCase):
    """No fresh static perception means no permission to move."""

    def test_missing_perception_holds_then_terminates(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(perception=None)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_unavailable", receipt["reason"])
        self.assertIsNone(self.tracker.terminal_reason)
        for _ in range(4):
            command, receipt = self.step(advance=2.0, perception=None)
        self.assertEqual("interactive_perception_lost", self.tracker.terminal_reason)
        self.assertEqual([0.0, 0.0, 0.0], command)

    def test_stale_perception_holds(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(perception=2.0, perception_stamp=self.sim - 1.0)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_stale", receipt["reason"])

    def test_unknown_clearance_holds(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step()
        self.tracker.update_perception(
            now_wall=self.now,
            stamp_s=self.sim,
            clearance_m=None,
            source="static_depth",
            frame_id="go2_base",
        )
        command, receipt = self.step(perception=None)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_clearance_unknown", receipt["reason"])

    def test_ground_truth_perception_is_refused(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(perception=2.0, perception_source="native_truth")
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_unlabelled", receipt["reason"])

    def test_obstacle_too_close_holds_then_blocks(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(perception=0.30)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("obstacle_too_close", receipt["reason"])
        for _ in range(4):
            command, receipt = self.step(advance=2.0, perception=0.30)
        self.assertEqual("interactive_obstacle_blocked", self.tracker.terminal_reason)

    def write_perception_file(self, **fields):
        payload = {
            "version": 1,
            "source": "live_rgbd_obstacle_cloud",
            "frame_id": "go2_base",
            "stamp_s": self.sim,
            "arrival_wall_s": self.now,
            "clearance_m": 2.0,
            "corridor_points": 120,
            "cloud_points": 20000,
            "observation_state": "measured",
            "depth_valid_ratio": 1.0,
            "directions": {
                name: {"observation_state": "measured", "clearance_m": 2.0, "points": 100}
                for name in ("front", "rear", "left", "right")
            },
        }
        payload.update(fields)
        (self.tracker.mailbox / "perception.json").write_text(
            json.dumps(payload), encoding="utf-8"
        )

    def test_live_perception_file_is_consumed(self):
        self.write_perception_file()
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(perception=None, advance=0.06)
        self.assertGreater(command[0], 0.0)
        self.assertEqual("tracking", receipt["status"])

    def test_stale_perception_file_holds(self):
        self.write_perception_file(
            arrival_wall_s=self.now - 1.0, stamp_s=self.sim - 1.0
        )
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(perception=None)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_stale", receipt["reason"])

    def test_insufficient_perception_cloud_holds(self):
        self.write_perception_file(cloud_points=500)
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(perception=None, advance=0.06)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_insufficient_cloud", receipt["reason"])

    def test_obstacle_in_the_live_perception_file_holds(self):
        self.write_perception_file(clearance_m=0.30)
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(perception=None, advance=0.06)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("obstacle_too_close", receipt["reason"])

    def test_clearing_the_obstacle_resumes_tracking(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step(perception=0.30)
        command, receipt = self.step(perception=2.0)
        self.assertGreater(command[0], 0.0)
        self.assertIsNone(self.tracker.terminal_reason)
        self.assertEqual("tracking", receipt["status"])

    def test_lidar_floor_unsupported_direction_holds(self):
        directions = {
            name: {
                "observation_state": "measured",
                "clearance_m": 2.0,
                "floor_observation_state": "unsupported",
            }
            for name in ("front", "rear", "left", "right")
        }
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(directions=directions, advance=0.06)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_floor_unsupported", receipt["reason"])

    def test_lidar_floor_supported_direction_tracks(self):
        directions = {
            name: {
                "observation_state": "measured",
                "clearance_m": 2.0,
                "floor_observation_state": "supported",
            }
            for name in ("front", "rear", "left", "right")
        }
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(directions=directions, advance=0.06)
        self.assertGreater(command[0], 0.0)
        self.assertEqual("tracking", receipt["status"])

    def test_cached_local_command_is_gated_when_floor_turns_unsupported(self):
        supported = {
            name: {
                "observation_state": "measured",
                "clearance_m": 2.0,
                "floor_observation_state": "supported",
            }
            for name in ("front", "rear", "left", "right")
        }
        unsupported = {
            name: dict(supported[name], floor_observation_state="unsupported")
            for name in supported
        }
        self.write_request(
            seq=1,
            action="maneuver",
            body_forward_mps=0.08,
            body_lateral_mps=0.0,
            yaw_rate_rps=0.0,
            duration_s=2.0,
            ttl=10.0,
        )
        command, _ = self.step(directions=supported, advance=0.06)
        self.assertGreater(command[0], 0.0)
        # Same odometry tick (cached replay path) with the ground support gone.
        command, _ = self.step(directions=unsupported, advance=0.0)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertTrue(
            any("perception_floor_unsupported" in str(reason) for reason in self.reasons())
        )

    def test_run4_legacy_supported_label_with_empty_near_bins_is_refused(self):
        # Exact front evidence from rtx_live_dsh4_20261110T202726Z,
        # capture/interactive/perception.json, stamp_s=109.9. Clocks below are
        # synthetic/fresh so rejection proves geometry, not stale-file behavior.
        front = {
            "observation_state": "measured", "clearance_m": 3.0,
            "floor_observation_state": "supported", "floor_points": 999,
            "radial_bin_floor_points": [0, 0, 458, 541],
            "radial_bins_required": 4, "radial_start_m": 1.675,
            "near_field_floor_unobserved_m": [0.0, 1.675],
            "floor_visible_min_range_m": 1.8372,
        }
        self.write_request(seq=1, goal=[0.338, -0.02829, 0.0])
        command, receipt = self.step(
            directions={"front": front}, advance=0.06,
            perception_source="rtx_lidar_cloud_registered_full",
        )
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_floor_unsupported", receipt["reason"])

    def test_cached_goto_rejects_legacy_near_floor_unknown(self):
        supported = {
            name: {"observation_state": "measured", "clearance_m": 3.0,
                   "floor_observation_state": "supported",
                   "radial_bin_floor_points": [100, 100, 100, 100],
                   "near_field_floor_unobserved_m": [0.0, 0.35]}
            for name in ("front", "rear", "left", "right")
        }
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, _ = self.step(
            directions=supported, advance=0.06,
            perception_source="rtx_lidar_cloud_registered_full",
        )
        self.assertGreater(command[0], 0.0)
        supported["front"]["radial_bin_floor_points"] = [0, 0, 458, 541]
        command, receipt = self.step(
            directions=supported, advance=0.0,
            perception_source="rtx_lidar_cloud_registered_full",
        )
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_floor_unsupported", receipt["reason"])

    def test_cached_reverse_rechecks_floor_with_front_still_supported(self):
        supported = {
            name: {"observation_state": "measured", "clearance_m": 3.0,
                   "floor_observation_state": "supported",
                   "radial_bin_floor_points": [100, 100, 100, 100],
                   "near_field_floor_unobserved_m": [0.0, 0.35]}
            for name in ("front", "rear", "left", "right")
        }
        self.write_request(
            seq=1, action="maneuver", body_forward_mps=-0.08,
            body_lateral_mps=0.0, yaw_rate_rps=0.0, duration_s=2.0, ttl=10.0,
        )
        command, _ = self.step(
            directions=supported, advance=0.06,
            perception_source="rtx_lidar_cloud_registered_full",
        )
        self.assertLess(command[0], 0.0)
        supported["rear"]["radial_bin_floor_points"] = [0, 100, 100, 100]
        command, _ = self.step(
            directions=supported, advance=0.0,
            perception_source="rtx_lidar_cloud_registered_full",
        )
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("perception_floor_unsupported:rear", self.reasons())

    def test_rtx_missing_floor_evidence_is_not_rgbd(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(
            advance=0.06, perception_source="rtx_lidar_cloud_registered_full",
        )
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_floor_unsupported", receipt["reason"])


class StandGateTests(MailboxCase):
    """The expected pre-motion wait must never be mistaken for a lost pose."""

    def test_stand_gate_wait_is_not_a_terminal_hold(self):
        self.write_heartbeat()
        self.write_request(seq=1, goal=[1.0, 0.0, 0.0])
        for _ in range(300):  # 6 s of 50 Hz ticks before the stand gate opens
            self.write_heartbeat()
            command, receipt = self.step(ready=False, advance=0.02)
            self.assertEqual([0.0, 0.0, 0.0], command)
            self.assertEqual("not_ready", receipt["reason"])
            self.assertIsNone(self.tracker.terminal_reason)
        self.assertEqual(1, self.events().count("hold"))
        command, receipt = self.step(ready=True)
        self.assertGreater(command[0], 0.0)
        self.assertEqual("tracking", receipt["status"])
        self.assertIsNone(self.tracker.terminal_reason)


class SafetyCoverageTests(MailboxCase):
    """Future stamps, frame identity, observation honesty and direction coverage."""

    def directions(self, missing=(), clearance=2.0):
        table = {}
        for name in ("front", "rear", "left", "right"):
            if name in missing:
                table[name] = {
                    "observation_state": "insufficient_observation",
                    "clearance_m": None,
                    "points": 0,
                }
            else:
                table[name] = {
                    "observation_state": "measured",
                    "clearance_m": clearance,
                    "points": 100,
                }
        return table

    def test_future_odometry_uses_the_small_tolerance(self):
        self.write_request(seq=1, goal=[3.0, 0.0, 0.0])
        command, receipt = self.step(pose_future=0.05)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("future_odometry", receipt["reason"])
        self.assertIsNone(self.tracker.terminal_reason)
        command, receipt = self.step(pose_future=1e-6)
        self.assertGreater(command[0], 0.0)
        self.assertEqual("fresh_measured_odometry", receipt["reason"])

    def test_camera_optical_perception_is_not_accepted_as_base_frame(self):
        self.write_request(seq=1, goal=[3.0, 0.0, 0.0])
        command, receipt = self.step(perception_frame="go2_camera_optical")
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_frame_mismatch", receipt["reason"])

    def test_clear_free_needs_a_healthy_depth_frame(self):
        self.write_request(seq=1, goal=[3.0, 0.0, 0.0])
        command, receipt = self.step(
            observation_state="clear_free", depth_valid_ratio=0.30
        )
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_insufficient_observation", receipt["reason"])

    def test_clear_free_with_a_healthy_frame_is_accepted(self):
        self.write_request(seq=1, goal=[3.0, 0.0, 0.0])
        command, receipt = self.step(
            observation_state="clear_free", depth_valid_ratio=0.98
        )
        self.assertGreater(command[0], 0.0)
        self.assertEqual("fresh_measured_odometry", receipt["reason"])

    def test_failed_observation_state_holds(self):
        self.write_request(seq=1, goal=[3.0, 0.0, 0.0])
        command, receipt = self.step(observation_state="insufficient_observation")
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_clearance_unknown", receipt["reason"])

    def test_reverse_maneuver_without_rear_coverage_is_rejected(self):
        self.write_request(
            seq=1, action="maneuver", body_forward_mps=-0.3, duration_s=0.5
        )
        command, _ = self.step(directions=self.directions(missing=("rear",)))
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIsNone(self.tracker._action)
        rejected = [
            item
            for item in self.receipts()
            if item.get("event") == "request_rejected"
        ]
        self.assertTrue(rejected)
        self.assertIn("rear", str(rejected[-1].get("reason")))

    def test_left_strafe_without_left_coverage_is_rejected(self):
        self.write_request(
            seq=1, action="maneuver", body_lateral_mps=0.04, duration_s=0.5
        )
        command, _ = self.step(directions=self.directions(missing=("left",)))
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIsNone(self.tracker._action)
        rejected = [
            item
            for item in self.receipts()
            if item.get("event") == "request_rejected"
        ]
        self.assertTrue(rejected)
        self.assertIn("left", str(rejected[-1].get("reason")))

    def test_yaw_needs_only_front_coverage(self):
        self.write_request(seq=1, action="yaw", delta_yaw_rad=0.5)
        command, _ = self.step(directions=self.directions(missing=("rear",)))
        self.assertAlmostEqual(0.15, command[2], places=6)
        self.assertIsNotNone(self.tracker._action)


class LocalActionTests(MailboxCase):
    """In-place scan turns and short local manoeuvres stay inside the envelope."""

    def test_yaw_action_turns_in_place_and_completes(self):
        self.write_request(seq=1, action="yaw", delta_yaw_rad=1.57)
        command, _ = self.step()
        self.assertEqual(0.0, command[0])
        self.assertEqual(0.0, command[1])
        self.assertAlmostEqual(0.15, command[2], places=6)
        for yaw in (0.4, 1.0, 1.4):
            command, _ = self.step(yaw=yaw)
            self.assertLessEqual(abs(command[2]), 0.15)
            self.assertEqual(0.0, command[0])
        command, receipt = self.step(yaw=1.53)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("yaw_accepted", self.events())
        self.assertIn("yaw_complete", self.events())
        self.assertEqual("fresh_measured_odometry", receipt["reason"])
        self.assertIsNone(self.tracker.terminal_reason)

    def test_yaw_action_rotation_is_relative_to_the_measurement(self):
        self.write_request(seq=1, action="yaw", delta_yaw_rad=-0.5)
        command, _ = self.step(yaw=1.0)
        self.assertLess(command[2], 0.0)
        action = self.tracker._action
        self.assertAlmostEqual(0.5, action["target_yaw_rad"], places=6)

    def test_maneuver_is_bounded_and_expires_on_duration(self):
        self.write_request(
            seq=1,
            action="maneuver",
            body_forward_mps=0.5,
            body_lateral_mps=0.02,
            yaw_rate_rps=0.10,
            duration_s=0.4,
        )
        command, _ = self.step()
        self.assertEqual([0.5, 0.02, 0.10], [round(value, 6) for value in command])
        command, _ = self.step(advance=0.2)
        self.assertEqual([0.5, 0.02, 0.10], [round(value, 6) for value in command])
        command, _ = self.step(advance=0.2)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("maneuver_complete", self.events())
        self.assertIsNone(self.tracker.terminal_reason)

    def test_maneuver_above_the_speed_ceiling_is_rejected(self):
        self.write_request(
            seq=1,
            action="maneuver",
            body_forward_mps=1.50,
            duration_s=0.5,
        )
        self.step()
        self.assertIn("invalid_maneuver_parameters", self.reasons())
        self.assertEqual([], self.tracker.controller.queue)

    def test_maneuver_above_the_lateral_bound_is_rejected(self):
        self.write_request(
            seq=1,
            action="maneuver",
            body_forward_mps=0.2,
            body_lateral_mps=0.20,
            duration_s=0.5,
        )
        self.step()
        self.assertIn("invalid_maneuver_parameters", self.reasons())

    def test_maneuver_above_the_yaw_rate_bound_is_rejected(self):
        self.write_request(
            seq=1,
            action="maneuver",
            body_forward_mps=0.2,
            yaw_rate_rps=0.50,
            duration_s=0.5,
        )
        self.step()
        self.assertIn("invalid_maneuver_parameters", self.reasons())

    def test_yaw_delta_beyond_one_turn_is_rejected(self):
        self.write_request(seq=1, action="yaw", delta_yaw_rad=5.0)
        self.step()
        self.assertIn("invalid_yaw_parameters", self.reasons())

    def test_local_action_clears_queued_waypoints(self):
        self.write_request(seq=1, goal=[2.0, 0.0, 0.0])
        self.step()
        self.assertEqual(1, len(self.tracker.controller.queue))
        self.write_request(seq=2, action="yaw", delta_yaw_rad=0.5)
        self.step(advance=0.15)
        self.assertEqual([], self.tracker.controller.queue)
        self.assertIn("yaw_accepted", self.events())

    def test_local_action_ttl_expiry(self):
        self.write_request(seq=1, action="yaw", delta_yaw_rad=1.0, ttl=1.0)
        self.step()
        command, _ = self.step(advance=1.5)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("action_expired", self.events())
        self.assertIsNone(self.tracker.terminal_reason)

    def test_stop_cancels_an_active_local_action(self):
        self.write_request(seq=1, action="maneuver", body_forward_mps=0.5, duration_s=2.0)
        command, _ = self.step()
        self.assertGreater(command[0], 0.0)
        self.write_request(seq=2, action="stop")
        command, _ = self.step(advance=0.15)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("interactive_stop", self.tracker.terminal_reason)
        self.assertIsNone(self.tracker._action)


class ObservationBundleTests(MailboxCase):
    """The observation bundle is the operator's only feedback channel."""

    def write_bundle(self):
        self.write_heartbeat()
        self.write_request(seq=1, goal=[2.0, 0.0, 0.0])
        self.step()
        self.step(advance=0.6, position=[0.2, 0.0, 0.0], speed=1.0)

    def test_bundle_contains_pose_guards_and_waypoint_state(self):
        self.write_bundle()
        observation = json.loads(
            self.tracker.observation_path.read_text(encoding="utf-8")
        )
        self.assertEqual(PROTOCOL_VERSION, observation["protocol_version"])
        self.assertEqual("lio_estimate", observation["pose"]["source"])
        self.assertEqual("map", observation["pose"]["map_frame"])
        self.assertTrue(observation["pose"]["fresh"])
        self.assertFalse(observation["pose"]["global_localization_claim"])
        self.assertEqual(1.0, observation["speed"]["command_ceiling_mps"])
        self.assertAlmostEqual(0.2, observation["coverage"]["path_length_m"], places=6)
        self.assertEqual(1, observation["waypoints"]["active_seq"])
        self.assertEqual("cruise", observation["waypoints"]["phase"])
        self.assertEqual(
            "deterministic_waypoint_follower", observation["command"]["source"]
        )
        self.assertFalse(observation["map"]["simulator_map_injected"])
        self.assertIn("/dynanav/depth", observation["map"]["topics"])

    def test_bundle_carries_no_ground_truth_pose(self):
        self.write_bundle()
        text = self.tracker.observation_path.read_text(encoding="utf-8")
        observation = json.loads(text)
        self.assertNotIn("native_truth", text)
        self.assertNotIn("ground_truth", text)
        self.assertFalse(observation["guards"]["native"]["usable_for_navigation"])
        self.assertIn("label", observation["guards"]["native"])

    def test_bundle_reports_terminal_state(self):
        self.write_heartbeat()
        self.write_request(seq=1, action="stop")
        self.step()
        observation = json.loads(
            self.tracker.observation_path.read_text(encoding="utf-8")
        )
        self.assertEqual("interactive_stop", observation["waypoints"]["terminal_reason"])
        self.assertEqual([0.0, 0.0, 0.0], observation["command"]["last_request"])


class ControlLoopSurfaceTests(MailboxCase):
    """The 50 Hz capture loop reads these attributes every tick; keep them stable."""

    def test_attributes_used_by_the_control_loop_exist_and_are_typed(self):
        self.write_heartbeat()
        self.write_request(seq=1, goal=[2.0, 0.0, 0.0])
        self.tracker.observe_native_guards(tilt_rad=0.12, native_speed_mps=0.87)
        command, _ = self.step()
        self.assertIsInstance(self.tracker.phase, str)
        self.assertIsInstance(self.tracker.stage, int)
        self.assertIsInstance(self.tracker.events, list)
        self.assertIsNone(self.tracker.failure)
        self.assertFalse(self.tracker.arrived)
        self.assertIsNone(getattr(self.tracker, "terminal_reason", None))
        self.assertGreater(command[0], 0.0)

    def test_native_guards_are_reported_but_never_navigational(self):
        self.write_heartbeat()
        self.write_request(seq=1, goal=[2.0, 0.0, 0.0])
        self.tracker.observe_native_guards(tilt_rad=0.12, native_speed_mps=0.87)
        self.step()
        observation = json.loads(
            self.tracker.observation_path.read_text(encoding="utf-8")
        )
        native = observation["guards"]["native"]
        self.assertAlmostEqual(0.12, native["tilt_rad"], places=6)
        self.assertAlmostEqual(0.87, native["native_speed_mps"], places=6)
        self.assertFalse(native["usable_for_navigation"])
        self.assertIn("label", native)

    def test_measurement_invalidations_reset_the_follower_without_terminating(self):
        self.write_heartbeat()
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step()
        self.tracker.invalidate_measurement()
        self.assertEqual([0.0, 0.0, 0.0], self.tracker._last_command)
        self.assertIsNone(self.tracker.terminal_reason)
        self.assertIsNone(self.tracker.controller.failure)


class WiringTests(unittest.TestCase):
    """The control loop entry must build the explorer only for its own mode."""

    def setUp(self):
        self._temporary = tempfile.TemporaryDirectory()
        self.mailbox = Path(self._temporary.name) / "run-20261110" / "interactive"
        self.profile = make_profile(self.mailbox)

    def tearDown(self):
        self._temporary.cleanup()

    def test_create_route_controller_builds_the_interactive_tracker(self):
        from embodied_navigation.simulation.dynanav.native_waypoints import NativeWaypointTracker
        tracker = create_route_controller(
            self.profile,
            gt_diagnostic=False,
            slam_control=True,
            interactive_control=True,
            target_speed=1.0,
            spawn=np.array([0.0, 0.0, 0.0]),
        )
        self.assertIsInstance(tracker, NativeWaypointTracker)
        self.assertTrue(self.mailbox.is_dir())
        self.assertEqual("lio_estimate", tracker.pose_source)
        self.assertEqual(1.0, tracker.parameters["command_bounds"]["forward_mps"])

    def test_create_interactive_tracker_rejects_a_route_profile(self):
        profile = dict(self.profile)
        profile["control_mode"] = "slam_pose_diagnostic"
        with self.assertRaises(ValueError):
            create_interactive_tracker(profile)

    def test_capture_config_selects_the_interactive_tracker(self):
        from embodied_navigation.simulation.dynanav import capture_go2_rgbd

        experiment = Path(self._temporary.name) / "experiment.json"
        experiment.write_text(json.dumps(self.profile), encoding="utf-8")
        output = Path(self._temporary.name) / "capture-out"
        with mock.patch.dict(
            os.environ, {"DYNANAV_EXPERIMENT": str(experiment)}, clear=False
        ):
            config = capture_go2_rgbd.parse_config(
                ["capture_go2_rgbd", str(output), "/nonexistent/office.usd"]
            )
        self.assertTrue(config.interactive_control)
        self.assertTrue(config.slam_control)
        self.assertFalse(config.gt_diagnostic)
        self.assertTrue(config.robogauge)

    def test_ground_truth_flagged_profile_is_refused_by_the_tracker(self):
        profile = dict(self.profile)
        profile["ground_truth_navigation"] = True
        with self.assertRaises(ValueError):
            create_interactive_tracker(profile)

    def test_slam_mode_profile_is_not_interactive(self):
        from embodied_navigation.simulation.dynanav import capture_go2_rgbd

        profile = dict(self.profile)
        profile["control_mode"] = "slam_pose_diagnostic"
        experiment = Path(self._temporary.name) / "experiment-slam.json"
        experiment.write_text(json.dumps(profile), encoding="utf-8")
        output = Path(self._temporary.name) / "capture-out-slam"
        with mock.patch.dict(
            os.environ, {"DYNANAV_EXPERIMENT": str(experiment)}, clear=False
        ):
            config = capture_go2_rgbd.parse_config(
                ["capture_go2_rgbd", str(output), "/nonexistent/office.usd"]
            )
        self.assertFalse(config.interactive_control)
        self.assertTrue(config.slam_control)




def _measured_direction(clearance, points=100, rays=100):
    return {
        "observation_state": "measured",
        "clearance_m": clearance,
        "points": points,
        "rays": rays,
    }


def _all_directions(clearance=2.0):
    return {
        name: _measured_direction(clearance)
        for name in ("front", "rear", "left", "right")
    }


class DuplicateOdometryDirectionGateTests(MailboxCase):
    """A cached body command may not outlive the perception it was admitted with.

    The control tick runs faster than the estimator publishes, so most ticks see a
    duplicate odometry stamp. Those ticks used to replay the cached command and skip
    every direction gate.
    """

    def accept_maneuver(self, **parameters):
        directions = _all_directions()
        # poll_request runs at 10 Hz wall time, so the request is only consumed
        # on a tick at least 0.1 s after the previous poll.
        self.step(advance=0.12, directions=directions)
        self.write_request(seq=1, action="maneuver", duration_s=2.0, **parameters)
        command, receipt = self.step(advance=0.12, directions=directions)
        self.assertNotIn("rejected", self.events(), self.reasons())
        return command, self.sim

    def test_rear_obstacle_cancels_a_reverse_maneuver_on_a_duplicate_pose(self):
        command, stamp = self.accept_maneuver(body_forward_mps=-0.3)
        self.assertAlmostEqual(-0.3, command[0], places=6)
        directions = _all_directions()
        directions["rear"] = _measured_direction(0.1)
        command, receipt = self.step(directions=directions, stamp=stamp)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertTrue(receipt["direction_gate_reevaluated"])
        self.assertIn("obstacle_too_close:rear", self.reasons())
        self.assertIn("action_cancelled", self.events())
        self.assertIsNone(self.tracker._action)

    def test_left_obstacle_cancels_a_left_strafe_on_a_duplicate_pose(self):
        command, stamp = self.accept_maneuver(body_lateral_mps=0.05)
        self.assertAlmostEqual(0.05, command[1], places=6)
        directions = _all_directions()
        directions["left"] = _measured_direction(0.1)
        command, _ = self.step(directions=directions, stamp=stamp)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("obstacle_too_close:left", self.reasons())

    def test_right_obstacle_cancels_a_right_strafe_on_a_duplicate_pose(self):
        command, stamp = self.accept_maneuver(body_lateral_mps=-0.05)
        directions = _all_directions()
        directions["right"] = _measured_direction(0.1)
        command, _ = self.step(directions=directions, stamp=stamp)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("obstacle_too_close:right", self.reasons())

    def test_a_forward_maneuver_keeps_running_with_a_near_rear_obstacle(self):
        command, stamp = self.accept_maneuver(body_forward_mps=0.3)
        directions = _all_directions()
        directions["rear"] = _measured_direction(0.1)
        command, _ = self.step(directions=directions, stamp=stamp)
        self.assertAlmostEqual(0.3, command[0], places=6)
        self.assertIsNotNone(self.tracker._action)


class OdometryStampMonotonicityTests(MailboxCase):
    """A withdrawn estimator stamp may not keep driving after a rewind."""

    def test_a_rewound_stamp_holds_and_keeps_the_highest_baseline(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step(sim=7.00, stamp=7.00)
        self.assertEqual(7.0, self.tracker._last_odom_stamp_s)
        self.step(sim=7.10, stamp=7.10)
        command, receipt = self.step(sim=7.20, stamp=7.05)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("nonmonotonic_odometry_stamp", receipt["reason"])
        self.assertEqual(7.10, self.tracker._last_odom_stamp_s)
        command, receipt = self.step(sim=7.30, stamp=7.05)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("nonmonotonic_odometry_stamp", receipt["reason"])
        command, _ = self.step(sim=7.35, stamp=7.12)
        self.assertGreater(command[0], 0.0)
        self.assertEqual(7.12, self.tracker._last_odom_stamp_s)

    def test_bad_frames_never_become_the_monotonic_baseline(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step(sim=7.00, stamp=7.00)
        command, receipt = self.step(sim=7.10, stamp=7.60)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("future_odometry", receipt["reason"])
        self.assertEqual(7.00, self.tracker._last_odom_stamp_s)
        command, _ = self.step(sim=7.20, stamp=7.05)
        self.assertGreater(command[0], 0.0)
        command, receipt = self.step(sim=7.30, stamp=6.90)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("stale_sim_odometry", receipt["reason"])
        self.assertEqual(7.05, self.tracker._last_odom_stamp_s)
        command, _ = self.step(sim=7.32, stamp=7.08)
        self.assertGreater(command[0], 0.0)

    def test_equal_stamps_are_duplicates_not_rewinds(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step(sim=7.00, stamp=7.00)
        command, receipt = self.step(sim=7.02, stamp=7.00)
        self.assertGreater(command[0], 0.0)
        self.assertEqual("fresh_duplicate_odometry", receipt["reason"])
        self.assertEqual(7.00, self.tracker._last_odom_stamp_s)

    def test_the_first_sample_sets_the_baseline_without_a_false_rewind(self):
        self.assertIsNone(self.tracker._last_odom_stamp_s)
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        _, receipt = self.step(sim=7.00, stamp=6.90)
        self.assertEqual("tracking", receipt["status"])
        self.assertEqual(6.90, self.tracker._last_odom_stamp_s)
        command, receipt = self.step(sim=7.04, stamp=6.80)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("nonmonotonic_odometry_stamp", receipt["reason"])


class DirectionClearanceValidationTests(MailboxCase):
    """A direction clearance must be a real number before it is compared."""

    def _reverse_against(self, clearance):
        self.step(advance=0.12, directions=_all_directions())
        self.write_request(seq=1, action="maneuver", body_forward_mps=-1.0, duration_s=2.0)
        directions = _all_directions()
        directions["rear"] = dict(_measured_direction(2.0), clearance_m=clearance)
        return self.step(advance=0.12, directions=directions)

    def test_non_finite_rear_clearance_is_refused_by_the_public_api(self):
        for bad in (float("nan"), float("inf"), float("-inf"), "1.0", True):
            with self.subTest(bad=bad):
                self.setUp()
                try:
                    with self.assertRaises(ValueError):
                        self._reverse_against(bad)
                    self.assertIsNone(self.tracker._action)
                finally:
                    self.tearDown()

    def test_a_missing_rear_clearance_is_held_not_authorised(self):
        command, _ = self._reverse_against(None)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIsNone(self.tracker._action)
        self.assertIn("perception_direction_uncovered:rear", self.reasons())


    def test_non_finite_rear_clearance_never_authorises_reverse(self):
        """Defence in depth: a NaN clearance that reached the sample by another
        route still cannot authorise a reverse command."""

        self.step(advance=0.12, directions=_all_directions())
        self.write_request(seq=1, action="maneuver", body_forward_mps=-1.0, duration_s=2.0)
        directions = _all_directions()
        directions["rear"] = dict(_measured_direction(2.0), clearance_m=float("nan"))
        sample = dict(self.tracker._perception)
        sample["directions"] = directions
        self.tracker._perception = sample
        command, receipt = self.step(advance=0.12, perception=None)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIsNone(self.tracker._action)
        self.assertIn("perception_direction_invalid:rear", self.reasons())


class TinyManeuverCoverageTests(MailboxCase):
    """Every non-zero body translation needs coverage of the direction it moves in."""

    def test_a_one_centimetre_reverse_still_needs_rear_coverage(self):
        self.write_request(seq=1, action="maneuver", body_forward_mps=-0.01, duration_s=2.0)
        command, _ = self.step(directions={"front": _measured_direction(2.0)})
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("perception_direction_uncovered:rear", self.reasons())
        self.assertIsNone(self.tracker._action)

    def test_a_tiny_left_strafe_needs_left_coverage(self):
        self.write_request(seq=1, action="maneuver", body_lateral_mps=0.05, duration_s=2.0)
        command, _ = self.step(directions={"front": _measured_direction(2.0)})
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("perception_direction_uncovered:left", self.reasons())

    def test_a_tiny_right_strafe_needs_right_coverage(self):
        self.write_request(seq=1, action="maneuver", body_lateral_mps=-0.05, duration_s=2.0)
        command, _ = self.step(directions={"front": _measured_direction(2.0)})
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("perception_direction_uncovered:right", self.reasons())

    def test_a_tiny_forward_creep_is_allowed_with_front_coverage(self):
        self.write_request(seq=1, action="maneuver", body_forward_mps=0.01, duration_s=2.0)
        command, _ = self.step(directions={"front": _measured_direction(2.0)})
        self.assertAlmostEqual(0.01, command[0], places=6)


class ProductionPerceptionFlowTests(MailboxCase):
    """The producer direction labelling must drive the tracker gates unchanged."""

    @staticmethod
    def producer_cloud(rear_obstacle=None):
        front = np.stack(
            [
                np.linspace(0.6, 2.0, 300),
                np.linspace(-0.15, 0.15, 300),
                np.zeros(300),
            ],
            axis=1,
        )
        rear = np.stack(
            [
                -np.linspace(0.6, 2.0, 300),
                np.zeros(300),
                np.zeros(300),
            ],
            axis=1,
        )
        parts = [front, rear]
        if rear_obstacle is not None:
            parts.append(np.array([[-rear_obstacle, 0.0, 0.0]] * 100))
        return np.vstack(parts)

    def test_a_producer_frame_with_a_clear_front_moves_the_base(self):
        directions = direction_coverage(self.producer_cloud())
        self.assertEqual("measured", directions["front"]["observation_state"])
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, _ = self.step(
            advance=0.12,
            directions=directions,
            perception=directions["front"]["clearance_m"],
        )
        self.assertGreater(command[0], 0.0)

    def test_a_producer_rear_obstacle_cancels_reverse_on_a_duplicate_pose(self):
        clear = direction_coverage(self.producer_cloud())
        clearance = clear["front"]["clearance_m"]
        self.step(advance=0.12, directions=clear, perception=clearance)
        self.write_request(
            seq=1, action="maneuver", body_forward_mps=-0.5, duration_s=2.0
        )
        command, _ = self.step(advance=0.12, directions=clear, perception=clearance)
        self.assertAlmostEqual(-0.5, command[0], places=6)
        stamp = self.sim
        blocked = direction_coverage(self.producer_cloud(rear_obstacle=0.4))
        self.assertEqual("measured", blocked["rear"]["observation_state"])
        self.assertLess(blocked["rear"]["clearance_m"], 0.55)
        command, _ = self.step(
            directions=blocked,
            perception=clearance,
            stamp=stamp,
        )
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("obstacle_too_close:rear", self.reasons())
        self.assertIn("action_cancelled", self.events())




class MotionBudgetTests(MailboxCase):
    """The motion budget is global: it must stop every command path, not only the
    waypoint follower. Yaw and short maneuver actions never reach
    WaypointQueueController.tick(), so they used to keep commanding after the
    budget had run out."""

    def arm_budget(self, sim):
        self.step(directions=_all_directions())
        self.tracker.controller.motion_end = sim

    def test_yaw_action_stops_at_the_motion_budget(self):
        self.arm_budget(7.05)
        self.write_request(seq=1, action="yaw", delta_yaw_rad=0.5)
        command, receipt = self.step(advance=0.12, directions=_all_directions())
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("interactive_motion_budget", self.tracker.terminal_reason)
        self.assertEqual("terminal", receipt["status"])

    def test_maneuver_stops_at_the_motion_budget(self):
        self.arm_budget(7.05)
        self.write_request(
            seq=1, action="maneuver", body_forward_mps=0.4, duration_s=2.0
        )
        command, _ = self.step(advance=0.12, directions=_all_directions())
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("interactive_motion_budget", self.tracker.terminal_reason)

    def test_goto_stops_at_the_motion_budget(self):
        self.arm_budget(7.05)
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, _ = self.step(advance=0.12)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("interactive_motion_budget", self.tracker.terminal_reason)

    def test_duplicate_odometry_after_the_budget_stays_terminal(self):
        self.arm_budget(7.05)
        self.step(advance=0.12)
        stamp = self.sim
        command, receipt = self.step(advance=0.02, stamp=stamp)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("terminal", receipt["status"])
        self.assertEqual("interactive_motion_budget", self.tracker.terminal_reason)

    def test_no_action_can_start_after_the_budget(self):
        self.arm_budget(7.05)
        self.step(advance=0.12)
        self.write_request(seq=2, action="yaw", delta_yaw_rad=0.5)
        command, _ = self.step(advance=0.12)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("interactive_motion_budget", self.tracker.terminal_reason)


class PerceptionFutureClockTests(MailboxCase):
    """A perception sample may not be read from the future, and a negative age is
    never clipped to zero."""

    def write_perception_file(self, *, stamp, arrival, clearance=2.0):
        self.tracker.perception_path.write_text(
            json.dumps(
                {
                    "version": 1,
                    "source": "static_depth",
                    "frame_id": "go2_base",
                    "stamp_s": stamp,
                    "arrival_wall_s": arrival,
                    "clearance_m": clearance,
                    "observation_state": "measured",
                    "points": 4096,
                    "corridor_points": 400,
                    "cloud_points": 4000,
                    "depth_valid_ratio": 1.0,
                    "directions": _all_directions(),
                }
            ),
            encoding="utf-8",
        )

    def test_a_future_arrival_holds_the_base(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.write_perception_file(stamp=self.sim, arrival=self.now + 100.0)
        command, receipt = self.step(perception=None)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_arrival_in_future", receipt["reason"])

    def test_a_future_source_stamp_holds_the_base(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.write_perception_file(stamp=self.sim + 0.5, arrival=self.now)
        command, receipt = self.step(perception=None)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_stamp_in_future", receipt["reason"])

    def test_a_non_finite_arrival_is_unavailable_not_fresh(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        payload = {
            "version": 1,
            "source": "static_depth",
            "frame_id": "go2_base",
            "stamp_s": 7.0,
            "arrival_wall_s": float("nan"),
            "clearance_m": 2.0,
            "observation_state": "measured",
            "points": 4096,
            "cloud_points": 4000,
        }
        self.tracker.perception_path.write_text(
            json.dumps(payload), encoding="utf-8"
        )
        command, receipt = self.step(perception=None)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("perception_unavailable", (receipt["reason"],))

    def test_a_valid_sample_after_a_future_one_resumes_tracking(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.write_perception_file(stamp=self.sim, arrival=self.now + 100.0)
        command, _ = self.step(perception=None, advance=0.12)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.write_perception_file(stamp=self.sim, arrival=self.now)
        command, _ = self.step(perception=None, advance=0.12)
        self.assertGreater(command[0], 0.0)

    def test_a_duplicate_pose_does_not_excuse_a_future_perception(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step()
        stamp = self.sim
        self.write_perception_file(stamp=self.sim, arrival=self.now + 100.0)
        command, receipt = self.step(perception=None, advance=0.12, stamp=stamp)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_arrival_in_future", receipt["reason"])


class GotoLateralCoverageTests(MailboxCase):
    """A lateral correction is real motion towards a side; if that side is not
    observed the follower holds instead of strafing blind."""

    @staticmethod
    def directions_without_side(name):
        directions = _all_directions()
        directions[name] = {
            "observation_state": "insufficient_observation",
            "clearance_m": None,
            "points": 0,
            "rays": 0,
            "reason": "no_ray_coverage",
        }
        return directions

    def test_goto_holds_instead_of_strafing_towards_an_unobserved_side(self):
        directions = self.directions_without_side("right")
        self.step(advance=0.12, directions=directions)
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step(advance=0.12, directions=directions)
        command, receipt = self.step(
            advance=0.12, directions=directions, position=[0.0, 0.1, 0.0]
        )
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("goto_lateral_uncovered", receipt["reason"])
        self.assertEqual(
            "perception_direction_uncovered:right",
            receipt["lateral_direction_reason"],
        )

    def test_goto_strafes_when_the_side_is_observed(self):
        directions = _all_directions()
        self.step(advance=0.12, directions=directions)
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step(advance=0.12, directions=directions)
        command, receipt = self.step(
            advance=0.12, directions=directions, position=[0.0, 0.1, 0.0]
        )
        self.assertAlmostEqual(-0.05, command[1], places=6)
        self.assertEqual("tracking", receipt["status"])

    def test_goto_on_the_segment_needs_no_side_coverage(self):
        directions = self.directions_without_side("left")
        self.step(advance=0.12, directions=directions)
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step(advance=0.12, directions=directions)
        command, _ = self.step(
            advance=0.12, directions=directions, position=[0.1, 0.0, 0.0]
        )
        self.assertGreater(command[0], 0.0)
        self.assertEqual(0.0, command[1])


class NearFieldProducerFlowTests(MailboxCase):
    """A producer return inside the near band must be measurable, not free."""

    @staticmethod
    def producer_directions():
        front = np.stack(
            [
                np.linspace(2.0, 4.0, 200),
                np.linspace(-0.1, 0.1, 200),
                np.zeros(200),
            ],
            axis=1,
        )
        near = np.array([[0.28, 0.30, 0.10]] * 100)
        return direction_coverage(np.vstack([front, near]))

    def test_a_near_field_return_is_measured_not_clear_free(self):
        directions = self.producer_directions()
        self.assertEqual("measured", directions["left"]["observation_state"])
        self.assertAlmostEqual(0.30, directions["left"]["clearance_m"], places=6)
        self.assertEqual("near_field_return", directions["left"]["reason"])

    def test_a_left_strafe_is_refused_by_the_near_field_return(self):
        directions = self.producer_directions()
        self.step(
            advance=0.12,
            directions=directions,
            perception=directions["front"]["clearance_m"],
        )
        self.write_request(
            seq=1, action="maneuver", body_lateral_mps=0.05, duration_s=2.0
        )
        command, _ = self.step(
            advance=0.12,
            directions=directions,
            perception=directions["front"]["clearance_m"],
        )
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("obstacle_too_close:left", self.reasons())





class CachedGotoLateralGateTests(MailboxCase):
    """A replayed goto command is not evidence about the current perception.

    The duplicate-odometry goto branch used to return the cached command without
    any direction check, so a side that became unknown or too close since the
    admission kept being strafed through a fresh pose stamp.
    """

    def admit_goto(self, *, offset_y, directions):
        self.step(advance=0.12, directions=directions)
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step(advance=0.12, directions=directions)
        command, _ = self.step(
            advance=0.12, directions=directions, position=[0.0, offset_y, 0.0]
        )
        return command

    @staticmethod
    def without_side(name):
        directions = _all_directions()
        directions[name] = {
            "observation_state": "insufficient_observation",
            "clearance_m": None,
            "points": 0,
            "rays": 0,
            "reason": "no_ray_coverage",
        }
        return directions

    def test_cached_goto_rechecks_the_side_gate(self):
        directions = _all_directions()
        self.admit_goto(offset_y=0.1, directions=directions)
        stamp = self.sim
        command, receipt = self.step(
            stamp=stamp,
            position=[0.0, 0.1, 0.0],
            directions=self.without_side("right"),
            advance=0.12,
        )
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("goto_lateral_uncovered", receipt["reason"])
        self.assertEqual("hold", receipt["status"])
        self.assertEqual("cached_odometry", receipt["command_source"])
        self.assertEqual(
            "perception_direction_uncovered:right",
            receipt["lateral_direction_reason"],
        )

    def test_cached_goto_rechecks_a_near_side_obstacle(self):
        self.admit_goto(offset_y=0.1, directions=_all_directions())
        stamp = self.sim
        near = _all_directions()
        near["right"] = {
            "observation_state": "measured",
            "clearance_m": 0.1,
            "points": 100,
            "rays": 100,
        }
        command, receipt = self.step(
            stamp=stamp, position=[0.0, 0.1, 0.0], directions=near, advance=0.12
        )
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual(
            "perception_direction_too_close?right" if False else "goto_lateral_uncovered",
            receipt["reason"],
        )
        self.assertEqual(
            "obstacle_too_close:right", receipt["lateral_direction_reason"]
        )

    def test_cached_goto_blocks_a_five_micrometre_strafe(self):
        self.admit_goto(offset_y=1e-5, directions=_all_directions())
        stamp = self.sim
        command, receipt = self.step(
            stamp=stamp,
            position=[0.0, 1e-5, 0.0],
            directions=self.without_side("right"),
            advance=0.12,
        )
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("goto_lateral_uncovered", receipt["reason"])

    def test_cached_goto_rechecks_the_left_side_too(self):
        self.admit_goto(offset_y=-0.1, directions=_all_directions())
        stamp = self.sim
        command, receipt = self.step(
            stamp=stamp,
            position=[0.0, -0.1, 0.0],
            directions=self.without_side("left"),
            advance=0.12,
        )
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("goto_lateral_uncovered", receipt["reason"])

    def test_cached_goto_resumes_when_the_side_is_observed_again(self):
        self.admit_goto(offset_y=0.1, directions=_all_directions())
        stamp = self.sim
        self.step(
            stamp=stamp,
            position=[0.0, 0.1, 0.0],
            directions=self.without_side("right"),
            advance=0.12,
        )
        command, receipt = self.step(
            stamp=self.sim,
            position=[0.0, 0.1, 0.0],
            directions=_all_directions(),
            advance=0.12,
        )
        self.assertAlmostEqual(-0.05, command[1], places=6)
        self.assertEqual("tracking", receipt["status"])

    def test_cached_goto_without_lateral_offset_is_not_held(self):
        self.admit_goto(offset_y=0.0, directions=self.without_side("right"))
        stamp = self.sim
        command, receipt = self.step(
            stamp=stamp,
            position=[0.1, 0.0, 0.0],
            directions=self.without_side("right"),
            advance=0.12,
        )
        self.assertGreater(command[0], 0.0)
        self.assertEqual("fresh_duplicate_odometry", receipt["reason"])


class PerceptionApiValidationTests(MailboxCase):
    """The public update_perception interface must reject broken clocks."""

    def assert_refused(self, **overrides):
        sample = dict(self.tracker._perception)
        payload = {
            "now_wall": sample["arrival_wall_s"],
            "stamp_s": sample["stamp_s"],
            "clearance_m": sample["clearance_m"],
            "source": sample["source"],
            "frame_id": sample["frame_id"],
            "points": sample["points"],
            "observation_state": sample["observation_state"],
            "depth_valid_ratio": sample["depth_valid_ratio"],
            "directions": sample["directions"],
        }
        payload.update(overrides)
        with self.assertRaises(ValueError):
            self.tracker.update_perception(**payload)

    def test_non_finite_arrival_is_refused(self):
        self.step(directions=_all_directions())
        self.assert_refused(now_wall=float("nan"))

    def test_infinite_arrival_is_refused(self):
        self.step(directions=_all_directions())
        self.assert_refused(now_wall=float("inf"))

    def test_non_finite_stamp_is_refused(self):
        self.step(directions=_all_directions())
        self.assert_refused(stamp_s=float("nan"))

    def test_non_finite_clearance_is_refused(self):
        self.step(directions=_all_directions())
        self.assert_refused(clearance_m=float("inf"))

    def test_non_finite_direction_clearance_is_refused(self):
        self.step(directions=_all_directions())
        directions = _all_directions()
        directions["rear"]["clearance_m"] = float("nan")
        self.assert_refused(directions=directions)

    def test_a_valid_sample_is_still_accepted(self):
        self.step(directions=_all_directions())
        sample = dict(self.tracker._perception)
        accepted = self.tracker.update_perception(
            now_wall=sample["arrival_wall_s"],
            stamp_s=sample["stamp_s"],
            clearance_m=sample["clearance_m"],
            source=sample["source"],
            frame_id=sample["frame_id"],
            points=sample["points"],
            observation_state=sample["observation_state"],
            depth_valid_ratio=sample["depth_valid_ratio"],
            directions=sample["directions"],
        )
        self.assertIsNotNone(accepted["stamp_s"])



class PerceptionTelemetryTests(MailboxCase):
    """A healthy run must show the validated perception sample it actually used.

    The observation bundle used to carry perception data only on failure, so a live
    run looked unvalidated (perception.last=null) even while the tracker was
    tracking. The telemetry now reports the validated sample and recomputes its ages
    at write time; it never changes admission or any threshold.
    """

    MEASURED_IMU_MOUNT = [0.020153064280748367, 0.0, -0.005108955316245556]

    def producer_payload(self, *, stamp, arrival):
        import numpy as np

        from embodied_navigation.perception.depth import depth_cloud_packet
        from embodied_navigation.simulation.dynanav.capture.perception import (
            interactive_perception_payload,
        )

        depth = np.full((512, 512), 2.0, dtype=np.float32)
        packet = depth_cloud_packet(
            depth,
            sensor_yaw=np.eye(3),
            camera_mount=np.array([0.3, 0.0, 0.13]),
            imu_mount=np.array(self.MEASURED_IMU_MOUNT),
        )
        return interactive_perception_payload(
            packet,
            imu_mount=self.MEASURED_IMU_MOUNT,
            stamp_s=stamp,
            arrival_wall_s=arrival,
        )

    def test_a_generated_payload_round_trips_to_tracking(self):
        payload = self.producer_payload(stamp=self.sim, arrival=self.now)
        self.tracker.perception_path.write_text(json.dumps(payload), encoding="utf-8")
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(perception=None, advance=0.12)
        self.assertGreater(command[0], 0.0)
        self.assertEqual("tracking", receipt["status"])
        observation = json.loads(self.tracker.observation_path.read_text())
        validated = observation["perception"]["validated"]
        self.assertTrue(validated["current"])
        self.assertTrue(validated["fresh"])
        self.assertEqual(payload["source"], validated["source"])
        self.assertEqual(payload["frame_id"], validated["frame_id"])
        self.assertEqual(payload["corridor_points"], validated["points"])
        self.assertEqual(payload["cloud_points"], validated["cloud_points"])
        self.assertIn("directions", validated)

    def test_a_stale_sample_stays_visible_but_is_not_current(self):
        payload = self.producer_payload(stamp=self.sim, arrival=self.now)
        self.tracker.perception_path.write_text(json.dumps(payload), encoding="utf-8")
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, _ = self.step(perception=None, advance=0.12)
        self.assertGreater(command[0], 0.0)
        command, receipt = self.step(perception=None, advance=0.6)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("perception_stale", (receipt["reason"],))
        observation = json.loads(self.tracker.observation_path.read_text())
        validated = observation["perception"]["validated"]
        self.assertFalse(validated["current"])
        self.assertFalse(validated["fresh"])
        self.assertGreater(validated["sim_age_s"], 0.3)
        self.assertIsNotNone(validated["stamp_s"])

    def test_telemetry_does_not_relax_the_clearance_gate(self):
        payload = self.producer_payload(stamp=self.sim, arrival=self.now)
        payload["clearance_m"] = 0.50
        payload["directions"] = {
            name: dict(entry) for name, entry in payload["directions"].items()
        }
        payload["directions"]["front"]["clearance_m"] = 0.50
        self.tracker.perception_path.write_text(json.dumps(payload), encoding="utf-8")
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(perception=None, advance=0.12)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("obstacle_too_close", (receipt["reason"],))
        self.assertIsNone(self.tracker._last_validated_perception)

    def test_a_broken_payload_is_not_reported_as_validated(self):
        payload = self.producer_payload(stamp=self.sim, arrival=self.now)
        payload["stamp_s"] = float("nan")
        self.tracker.perception_path.write_text(
            "{\"version\": 1, \"source\": \"live_rgbd_obstacle_cloud\", "
            "\"frame_id\": \"go2_base\", \"stamp_s\": NaN, \"arrival_wall_s\": " + str(self.now) + "}",
            encoding="utf-8",
        )
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(perception=None, advance=0.12)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIsNone(self.tracker._last_validated_perception)



class Float32OdomTimeQuantisationTests(MailboxCase):
    """A float32 source stamp for the current tick may lead the float64 sim time.

    The observer publishes its sensor clock as float32, so a same-tick stamp can be
    up to half a float32 ULP ahead of the capture's float64 sim time. That bound is
    only accepted inside the half-ULP window; a different physical tick, a large
    future stamp, stale or rewound samples keep their original gates.
    """

    @staticmethod
    def float32_stamp(sim):
        import numpy as np

        return float(np.float32(sim))

    def test_a_same_tick_float32_stamp_is_accepted_with_its_signed_age(self):
        sim = 64.315
        stamp = self.float32_stamp(sim)
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(sim=sim, stamp=stamp)
        self.assertGreater(command[0], 0.0)
        self.assertEqual("fresh_measured_odometry", receipt["reason"])
        self.assertLess(receipt["raw_sim_age_s"], 0.0)
        self.assertAlmostEqual(sim - stamp, receipt["raw_sim_age_s"], places=12)
        self.assertGreater(receipt["odometry_future_tolerance_s"], abs(receipt["raw_sim_age_s"]))
        self.assertEqual("float32_half_ulp_same_tick", receipt["odometry_future_tolerance_basis"])
        self.assertAlmostEqual(sim - stamp, receipt["same_tick_negative_age_s"], places=12)

    def test_a_stamp_beyond_the_half_ulp_bound_is_held(self):
        sim = 64.315
        tolerance = self.tracker._odometry_future_tolerance(sim)
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(sim=sim, stamp=sim + tolerance + 1e-6)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("future_odometry", receipt["reason"])
        self.assertAlmostEqual(sim - (sim + tolerance + 1e-6), receipt["raw_sim_age_s"], places=12)

    def test_one_physical_sensor_tick_ahead_is_held(self):
        # The physical tick is 200 Hz (0.005 s), not the 20 Hz depth frame.
        sim = 64.315
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(sim=sim, stamp=sim + 0.005)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("future_odometry", receipt["reason"])

    def test_a_large_future_stamp_is_still_held(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(sim=100.0, stamp=101.0)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("future_odometry", receipt["reason"])

    def test_stale_sample_is_unchanged(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        command, receipt = self.step(sim=100.0, stamp=99.7)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("stale_sim_odometry", receipt["reason"])

    def test_a_rewind_is_still_nonmonotonic(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.step(sim=64.315, stamp=self.float32_stamp(64.315))
        command, receipt = self.step(sim=64.415, stamp=64.3)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("nonmonotonic_odometry_stamp", receipt["reason"])

    def test_the_half_ulp_bound_stays_below_half_a_tick(self):
        import math

        previous = 0.0
        for sim in (64.315, 300.0, 1000.0, 8192.0):
            tolerance = self.tracker._odometry_future_tolerance(sim)
            self.assertLess(tolerance, 0.0025)
            self.assertGreaterEqual(tolerance, 2e-6)
            self.assertGreater(tolerance, previous)
            previous = tolerance
        # Beyond the horizon where half a float32 ULP would exceed half a physical
        # tick, the cap keeps it at half a tick and rejects anything larger.
        capped = self.tracker._odometry_future_tolerance(86400.0)
        self.assertAlmostEqual(0.0025, capped, places=12)
        self.assertLess(capped, 0.005)
        self.assertAlmostEqual(
            2e-6,
            self.tracker._odometry_future_tolerance(0.0),
            places=12,
        )



class PerceptionTelemetryTrustTests(MailboxCase):
    """The telemetry must not keep a sample alive or call history current."""

    @staticmethod
    def payload(now, stamp):
        return {
            "version": 1,
            "source": "live_rgbd_obstacle_cloud",
            "frame_id": "go2_base",
            "arrival_wall_s": now,
            "stamp_s": stamp,
            "clearance_m": 2.0,
            "cloud_points": 4096,
            "corridor_points": 100,
            "observation_state": "measured",
            "depth_valid_ratio": 1.0,
            "directions": {
                name: {"observation_state": "measured", "clearance_m": 2.0}
                for name in ("front", "rear", "left", "right")
            },
        }

    def validated(self):
        return json.loads(self.tracker.observation_path.read_text())["perception"][
            "validated"
        ]

    def test_a_sample_is_not_kept_alive_by_its_validation_time(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        arrival = self.now - 0.49
        payload = self.payload(arrival, self.sim)
        self.tracker.perception_path.write_text(json.dumps(payload), encoding="utf-8")
        command, _ = self.step(advance=0.0, perception=None)
        self.assertGreater(command[0], 0.0)
        first = self.validated()
        self.assertTrue(first["current"])
        self.assertAlmostEqual(arrival, first["arrival_wall_s"], places=9)
        self.tracker._last_observation_wall = float("-inf")
        command, receipt = self.step(advance=0.02, perception=None)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_stale", receipt["reason"])
        stale = self.validated()
        self.assertFalse(stale["current"])
        self.assertFalse(stale["fresh"])
        self.assertFalse(stale["gate_current"])
        self.assertAlmostEqual(arrival, stale["arrival_wall_s"], places=9)
        self.assertGreater(stale["sample_wall_age_s"], 0.5)
        self.assertGreater(stale["since_validated_wall_s"], 0.0)

    def test_a_terminal_tick_reports_the_history_as_not_current(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        self.tracker.perception_path.write_text(
            json.dumps(self.payload(self.now, 299.9)), encoding="utf-8"
        )
        self.step(sim=299.9, advance=0.0, perception=None)
        self.assertTrue(self.validated()["current"])
        self.tracker._last_observation_wall = float("-inf")
        self.tracker.perception_path.write_text(
            json.dumps(self.payload(self.now, 299.95)), encoding="utf-8"
        )
        command, receipt = self.step(sim=300.0, advance=0.02, perception=None)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("terminal", receipt["status"])
        self.assertEqual("interactive_motion_budget", self.tracker.terminal_reason)
        validated = self.validated()
        self.assertFalse(validated["current"])
        self.assertFalse(validated["fresh"])
        self.assertIsNotNone(validated["stamp_s"])

    def test_a_not_ready_tick_does_not_inherit_the_validated_flag(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        payload = self.payload(self.now, self.sim)
        self.tracker.perception_path.write_text(json.dumps(payload), encoding="utf-8")
        self.step(advance=0.02, perception=None)
        self.assertTrue(self.validated()["current"])
        self.tracker._last_observation_wall = float("-inf")
        command, receipt = self.step(ready=False, advance=0.02, perception=None)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("not_ready", receipt["reason"])
        validated = self.validated()
        self.assertFalse(validated["current"])
        self.assertFalse(validated["gate_current"])
        self.assertIsNotNone(validated["stamp_id"] if False else validated["stamp_s"])

    def test_a_rejected_replacement_sample_leaves_the_history_not_current(self):
        self.write_request(seq=1, goal=[5.0, 0.0, 0.0])
        payload = self.payload(self.now, self.sim)
        self.tracker.perception_path.write_text(json.dumps(payload), encoding="utf-8")
        self.step(advance=0.02, perception=None)
        self.assertTrue(self.validated()["current"])
        future = self.payload(self.now, self.sim + 4000.0)
        self.tracker.perception_path.write_text(json.dumps(future), encoding="utf-8")
        self.tracker._last_observation_wall = float("-inf")
        command, receipt = self.step(advance=0.1, perception=None)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertEqual("perception_stamp_in_future", receipt["reason"])
        validated = self.validated()
        self.assertFalse(validated["current"])
        self.assertFalse(validated["fresh"])
        self.assertFalse(validated["gate_current"])
        self.assertAlmostEqual(payload["stamp_s"], validated["stamp_s"], places=9)




class YawDwellCompletionTests(MailboxCase):
    """Dwell completion at production cadence: 50 Hz control, 10 Hz pose, real IMU metadata."""

    def setUp(self):
        super().setUp()
        self.tracker = InteractiveWaypointTracker(
            make_profile(
                self.mailbox,
                interactive_yaw_completion_mode="dwell",
                interactive_yaw_control_tolerance_rad=0.02,
                interactive_yaw_completion_tolerance_rad=0.05,
                interactive_yaw_settle_rate_radps=0.03,
                interactive_yaw_dwell_sim_s=0.5,
            )
        )

    def start_yaw(self, delta_deg=12.0):
        self.step(advance=0.12)
        self.write_request(seq=1, action="yaw", delta_yaw_rad=math.radians(delta_deg))
        self.step(advance=0.12)
        self.step(advance=0.12)
        return self.tracker._action["target_yaw_rad"]

    def drive(self, target, *, ticks, pose_every=5, yaw_rate=0.0, freeze_pose=False, imu_stamp=None, imu_valid=True, imu_missing=False, advance=0.02):
        pose_stamp = (self.sim + advance) if freeze_pose else None
        for index in range(ticks):
            if not freeze_pose and index % pose_every == 0:
                pose_stamp = self.sim + advance
            self.step(
                advance=advance,
                yaw=target - 0.005,
                yaw_rate=yaw_rate,
                pose_stamp=pose_stamp,
                imu_stamp=imu_stamp,
                imu_valid=imu_valid,
                imu_missing=imu_missing,
            )

    def completion(self):
        path = self.mailbox / "receipts.jsonl"
        receipts = [json.loads(line) for line in path.read_text().splitlines()]
        completed = [item for item in receipts if item.get("event") == "yaw_complete"]
        return completed[-1] if completed else None

    def test_production_cadence_with_fresh_duplicates_completes(self):
        target = self.start_yaw()
        self.drive(target, ticks=60)
        self.assertIsNone(self.tracker._action)
        receipt = self.completion()
        self.assertGreaterEqual(receipt["valid_sim_s"], 0.4999)
        self.assertGreaterEqual(receipt["last_distinct_stamp_s"] - receipt["start_sim_s"], 0.45)
        self.assertTrue(receipt["distinct_since_start"])
        self.assertEqual("dwell", receipt["completion_mode"])
        self.assertEqual("body_imu_z_rate", receipt["yaw_rate_frame"])
        self.assertLessEqual(receipt["yaw_error_rad"], receipt["tolerance_rad"])

    def test_zero_point_four_eight_sim_seconds_is_not_enough(self):
        target = self.start_yaw()
        self.drive(target, ticks=24)
        self.assertIsNotNone(self.tracker._action)
        self.drive(target, ticks=3)
        self.assertIsNone(self.tracker._action)

    def test_a_one_second_observation_gap_restarts_the_window(self):
        target = self.start_yaw()
        self.drive(target, ticks=15)
        self.step(advance=1.0, yaw=target - 0.005, yaw_rate=0.0)
        self.assertIsNotNone(self.tracker._action)
        self.drive(target, ticks=15)
        self.assertIsNotNone(self.tracker._action)
        self.drive(target, ticks=15)
        self.assertIsNone(self.tracker._action)

    def test_a_frozen_pose_source_cannot_complete(self):
        target = self.start_yaw()
        self.drive(target, ticks=60, freeze_pose=True, pose_every=5)
        self.assertIsNone(self.tracker._yaw_window)
        self.assertIsNone(self.completion())

    def test_invalidate_measurement_clears_the_window(self):
        target = self.start_yaw()
        self.drive(target, ticks=15)
        self.tracker.invalidate_measurement()
        self.drive(target, ticks=4)
        self.assertIsNotNone(self.tracker._action)
        self.drive(target, ticks=4)
        self.assertIsNotNone(self.tracker._action)

    def test_missing_or_future_or_nan_gyro_metadata_blocks_completion(self):
        target = self.start_yaw()
        self.drive(target, ticks=40, imu_stamp=float("nan"))
        self.assertIsNotNone(self.tracker._action)
        target = self.start_yaw()
        self.drive(target, ticks=40, imu_stamp=self.sim + 0.5)
        self.assertIsNotNone(self.tracker._action)
        target = self.start_yaw()
        self.drive(target, ticks=40, imu_missing=True)
        self.assertIsNotNone(self.tracker._action)

    def test_a_legal_float32_ahead_stamp_is_not_rejected_by_the_window(self):
        target = self.start_yaw()
        for index in range(60):
            if index % 5 == 0:
                pose_stamp = self.sim + 2.441e-6
            self.step(
                advance=0.02,
                yaw=target - 0.005,
                yaw_rate=0.0,
                pose_stamp=pose_stamp,
            )
        self.assertIsNone(self.tracker._action)

    def test_the_legacy_instant_mode_is_an_explicit_choice(self):
        tracker = InteractiveWaypointTracker(
            make_profile(self.mailbox, interactive_yaw_completion_mode="instant")
        )
        self.tracker = tracker
        target = self.start_yaw()
        self.step(yaw=target - 0.005, yaw_rate=0.0)
        self.assertIsNone(self.tracker._action)
        receipt = self.completion()
        self.assertEqual("instant", receipt["completion_mode"])


    def accumulate(self, target, ticks):
        self.drive(target, ticks=ticks)

    def assert_invalidation_restarts(self, **step_kwargs):
        target = self.start_yaw()
        self.accumulate(target, 15)
        self.assertIsNotNone(self.tracker._yaw_window)
        self.step(yaw=target - 0.005, yaw_rate=0.0, **step_kwargs)
        self.assertIsNone(self.tracker._yaw_window)
        self.accumulate(target, 24)
        self.assertIsNotNone(self.tracker._action)
        self.accumulate(target, 2)
        self.assertIsNone(self.tracker._action)

    def test_not_ready_invalidates_the_window(self):
        self.assert_invalidation_restarts(ready=False)

    def test_a_stale_pose_invalidates_the_window(self):
        self.assert_invalidation_restarts(pose_stamp=self.sim - 0.30)

    def test_a_nan_gyro_sample_invalidates_the_window(self):
        self.assert_invalidation_restarts(imu_stamp=float("nan"))

    def test_a_skipped_control_period_restarts_the_window(self):
        target = self.start_yaw()
        self.accumulate(target, 15)
        self.step(advance=0.04, yaw=target - 0.005, yaw_rate=0.0)
        window = self.tracker._yaw_window
        self.assertIsNotNone(window)
        self.assertEqual(0.0, window["valid_sim_s"])
        self.assertAlmostEqual(self.sim, window["start_sim_s"], places=9)
        self.accumulate(target, 23)
        self.assertIsNotNone(self.tracker._action)

    def test_the_float32_same_tick_source_at_sixty_four_seconds_completes(self):
        import numpy as np

        self.sim = 64.315
        target = self.start_yaw()
        pose_stamp = float(np.float32(self.sim))
        for index in range(60):
            if index % 5 == 0:
                pose_stamp = float(np.float32(self.sim))
            self.step(
                advance=0.02,
                yaw=target - 0.005,
                yaw_rate=0.0,
                pose_stamp=pose_stamp,
                imu_stamp=float(np.float32(self.sim)),
            )
        self.assertIsNone(self.tracker._action)

    def test_one_physics_tick_ahead_is_refused(self):
        target = self.start_yaw()
        for index in range(40):
            self.step(
                advance=0.02,
                yaw=target - 0.005,
                yaw_rate=0.0,
                pose_stamp=self.sim + 0.025,
            )
        self.assertIsNotNone(self.tracker._action)
        self.assertIsNone(self.completion())

    def test_the_accepted_and_observation_json_report_the_real_criterion(self):
        target = self.start_yaw()
        receipts = [
            json.loads(line)
            for line in (self.mailbox / "receipts.jsonl").read_text().splitlines()
        ]
        accepted = [item for item in receipts if item.get("event") == "yaw_accepted"][-1]
        self.assertAlmostEqual(0.05, accepted["parameters"]["tolerance_rad"], places=9)
        self.assertEqual("dwell", accepted["completion"]["mode"])
        self.assertAlmostEqual(0.05, accepted["completion"]["completion_tolerance_rad"], places=9)
        self.assertAlmostEqual(0.02, accepted["completion"]["control_tolerance_rad"], places=9)
        self.assertAlmostEqual(0.5, accepted["completion"]["dwell_sim_s"], places=9)
        self.assertAlmostEqual(0.03, accepted["completion"]["settle_rate_radps"], places=9)
        self.drive(target, ticks=3)
        observation = json.loads(self.tracker.observation_path.read_text())
        self.assertEqual("dwell", observation["yaw_completion"]["mode"])
        self.assertAlmostEqual(0.05, observation["yaw_completion"]["completion_tolerance_rad"], places=9)
        self.assertAlmostEqual(0.02, observation["yaw_completion"]["control_tolerance_rad"], places=9)
        active = observation["action"]["active"]
        if active is not None:
            self.assertEqual("dwell", active["completion"]["mode"])
            self.assertIn("yaw_window", active)



    def drive_error(self, target, *, error, ticks=60):
        command = [0.0, 0.0, 0.0]
        for index in range(ticks):
            command, _ = self.step(
                advance=0.02,
                yaw=target - error,
                yaw_rate=0.0,
                pose_stamp=(self.sim + 0.02) if index % 5 == 0 else None,
            )
        return command

    def test_the_control_deadband_zeroes_the_command_inside_0_02(self):
        target = self.start_yaw()
        command = self.drive_error(target, error=0.015, ticks=20)
        self.assertEqual(0.0, command[2])
        self.assertIsNotNone(self.tracker._action)
        self.drive_error(target, error=0.015, ticks=10)
        self.assertIsNone(self.tracker._action)

    def test_an_error_between_the_bands_still_corrects_and_can_complete(self):
        target = self.start_yaw()
        command = self.drive_error(target, error=0.03, ticks=10)
        self.assertAlmostEqual(0.036, command[2], places=9)
        self.drive_error(target, error=0.03, ticks=20)
        self.assertIsNone(self.tracker._action)
        receipt = self.completion()
        self.assertFalse(receipt["control_band_met"])
        self.assertTrue(receipt["completion_band_met"])
        self.assertGreater(abs(receipt["yaw_error_rad"]), 0.02)
        self.assertLessEqual(abs(receipt["yaw_error_rad"]), 0.05)
        self.assertNotEqual(0.0, receipt["last_correction_wz"])

    def test_an_error_above_the_completion_band_never_completes(self):
        target = self.start_yaw()
        self.drive_error(target, error=0.06, ticks=60)
        self.assertIsNotNone(self.tracker._action)
        self.assertIsNone(self.completion())

    def test_the_two_tolerance_boundaries(self):
        target = self.start_yaw()
        command = self.drive_error(target, error=0.02, ticks=3)
        self.assertEqual(0.0, command[2])
        self.drive_error(target, error=0.05, ticks=3)
        self.assertIsNotNone(self.tracker._action)

    def test_conflicting_or_invalid_tolerances_are_refused(self):
        with self.assertRaises(ValueError):
            InteractiveWaypointTracker(
                make_profile(
                    self.mailbox,
                    interactive_yaw_completion_mode="dwell",
                    interactive_yaw_control_tolerance_rad=0.06,
                    interactive_yaw_completion_tolerance_rad=0.05,
                )
            )
        with self.assertRaises(ValueError):
            InteractiveWaypointTracker(
                make_profile(
                    self.mailbox,
                    interactive_yaw_tolerance_rad=0.05,
                    interactive_yaw_completion_tolerance_rad=0.05,
                )
            )
        with self.assertRaises(ValueError):
            InteractiveWaypointTracker(
                make_profile(self.mailbox, interactive_yaw_completion_tolerance_rad=float("nan"))
            )

    def test_the_full_p_and_instant_baselines_are_explicit(self):
        tracker = InteractiveWaypointTracker(
            make_profile(
                self.mailbox,
                interactive_yaw_completion_mode="dwell",
                interactive_yaw_control_tolerance_rad=0.0,
                interactive_yaw_completion_tolerance_rad=0.02,
            )
        )
        self.assertEqual(0.0, tracker.config["yaw_control_tolerance_rad"])
        self.assertEqual(0.02, tracker.config["yaw_completion_tolerance_rad"])
        instant = InteractiveWaypointTracker(
            make_profile(self.mailbox, interactive_yaw_completion_mode="instant")
        )
        self.assertEqual(0.0, instant.config["yaw_control_tolerance_rad"])
        self.assertAlmostEqual(0.05, instant.config["yaw_completion_tolerance_rad"], places=9)



    def test_the_completion_log_uses_the_real_deadband_command(self):
        target = self.start_yaw()
        self.drive_error(target, error=0.019, ticks=40)
        self.assertIsNone(self.tracker._action)
        receipt = self.completion()
        self.assertTrue(receipt["control_band_met"])
        self.assertEqual(0.0, receipt["pre_release_wz"])
        self.assertEqual(0.0, receipt["last_correction_wz"])

    def test_the_completion_log_reports_the_p_command_outside_the_deadband(self):
        target = self.start_yaw()
        self.drive_error(target, error=0.03, ticks=40)
        self.assertIsNone(self.tracker._action)
        receipt = self.completion()
        self.assertFalse(receipt["control_band_met"])
        self.assertAlmostEqual(0.036, receipt["pre_release_wz"], places=9)
        self.assertAlmostEqual(0.036, receipt["last_correction_wz"], places=9)


class ForwardTurnModeTests(MailboxCase):
    """Explicit production forward+turn policy: the follower never commands vy.

    Default (off) keeps the existing holonomic follower and its side-direction
    gate untouched; this class only pins the opt-in behaviour.
    """

    PROFILE_OVERRIDES = {"interactive_forward_turn_mode": True}

    def test_lateral_maneuver_is_refused_explicitly(self):
        self.write_request(
            seq=1,
            action="maneuver",
            body_forward_mps=0.08,
            body_lateral_mps=0.05,
            yaw_rate_rps=0.0,
            duration_s=2.0,
            ttl=10.0,
        )
        command, _ = self.step(advance=0.06)
        self.assertEqual([0.0, 0.0, 0.0], command)
        self.assertIn("forward_turn_mode_lateral_refused", self.reasons())

    def test_forward_goto_never_emits_lateral(self):
        self.write_request(seq=1, goal=[0.6, 0.0, 0.0])
        command, receipt = self.step(advance=0.06)
        self.assertEqual(0.0, command[1])
        self.assertEqual("tracking", receipt["status"])
        self.assertFalse(self.tracker.controller.allow_lateral)

    def test_mode_is_reported_in_the_observation_bundle(self):
        self.write_request(seq=1, goal=[0.6, 0.0, 0.0])
        self.step(advance=0.06)
        observation = json.loads(self.tracker.observation_path.read_text())
        self.assertTrue(observation["waypoints"]["forward_turn_mode"])


if __name__ == "__main__":
    unittest.main()
