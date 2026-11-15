"""Interactive waypoint exploration adapter for the isolated DynaNav probe.

Simulation-only, explicit, and opt-in. The high-level operator (the DSH agent)
publishes waypoints; a deterministic local follower tracks them continuously at
50 Hz and reports accepted/running/reached/expired/stop/failure receipts. The
only pose input is a measured estimator estimate (LIO or post-VIO); simulator
ground truth is never accepted as a waypoint frame or as a pose source.

Transport is a directory of small JSON files inside the run directory:

    <mailbox>/heartbeat.json      operator liveness (CLOCK_MONOTONIC seconds)
    <mailbox>/request.json        one waypoint command, atomically replaced
    <mailbox>/receipts.jsonl      append-only accept/reject/reached/terminal
    <mailbox>/observation.json    latest pose/guards/waypoint state (2 Hz)

Wall times are time.perf_counter() values: on Linux that is CLOCK_MONOTONIC,
shared by every process on the host, so the operator client and the simulator
compare the same clock without wall-clock skew. Simulator time is the Isaac
post-step measurement time and stays a separate domain.

What this module does not do: it does not build, own or publish a map, it does
not read ground-truth geometry, and it does not perform global localization.
The live map and the frontier/unknown analysis are built by the operator side
from the published measured topics (/dynanav/depth, /dynanav/camera,
/dynanav/lidar_filtered, /clock and the measured odometry topic). The simulator
only reports state.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import stat
from collections import deque
from pathlib import Path

import numpy as np

from embodied_navigation.common.time import float32_time_half_ulp
from embodied_navigation.common.geometry import quaternion_matrix
from embodied_navigation.common.geometry import wrap_angle as wrap

from embodied_navigation.navigation.yaw_action import (
    LOCAL_ACTION_BOUNDS, finite_number as _finite, yaw_action_step, yaw_config,
    yaw_request_parameters,
)

PROTOCOL_VERSION = 1
# Match the existing lockstep perception-file ingress limit.
MAX_PERCEPTION_FILE_BYTES = 2 * 1024 * 1024
INTERACTIVE_CONTROL_MODE = "interactive_waypoint_exploration"
MEASURED_POSE_SOURCES = ("lio_estimate", "livo_post_vio")
FORBIDDEN_POSE_SOURCES = (
    "native_truth",
    "native_pose",
    "simulator_native_pose",
    "ground_truth",
    "gt_diagnostic",
)
ALLOWED_ACTIONS = ("goto", "waypoint", "cancel", "stop", "yaw", "maneuver")
# Bounds for the optional local actions. They sit strictly inside the existing
# measured-route command envelope; nothing here widens the envelope.
# The IMU/physics tick of the simulator (200 Hz). The float32 source clock is
# quantised at this tick, so the same-tick tolerance must stay below half of it.
ODOMETRY_PHYSICAL_TICK_S = 0.005



# Bound geometry of one operator segment. A segment is the straight leg between
# consecutive waypoints; stairs are expected to be covered by several short
# segments instead of one long diagonal jump.
MAX_GOAL_ABS_XY_M = 100.0
MAX_GOAL_ABS_Z_M = 8.0
MAX_VERTICAL_STEP_M = 4.0


def clip(value, bound):
    """Clamp value into [-bound, bound]."""
    return max(-bound, min(bound, value))




def _vec3(value, name):
    """Return a validated finite XYZ vector as a float64 copy."""
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must contain 3 finite numbers") from error
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain 3 finite numbers")
    return result.copy()


def _bool(value, name):
    """Return a strict boolean or raise ValueError."""
    if type(value) is not bool:
        raise ValueError(f"{name} must be boolean")
    return value


def atomic_write_json(path, payload):
    """Write payload as JSON to path through a same-directory replace."""
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(payload, sort_keys=True, separators=(",", ":")), encoding="utf-8"
    )
    os.replace(temporary, path)


def read_json_file(path):
    """Return (payload, None) or (None, reason) for one small JSON file."""
    try:
        raw = Path(path).read_bytes()
    except FileNotFoundError:
        return None, "missing"
    except OSError:
        return None, "unreadable"
    try:
        return json.loads(raw), None
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None, "invalid_json"


def validate_interactive_profile(profile):
    """Validate an interactive-waypoint experiment profile without any I/O.

    Refuses ground-truth navigation, unmeasured pose sources and any mailbox that
    is not an explicit absolute path. Returns the normalised interaction limits.
    """
    if not isinstance(profile, dict):
        raise ValueError("interactive waypoint control requires an experiment profile")
    if profile.get("control_mode") != INTERACTIVE_CONTROL_MODE:
        raise ValueError(
            "interactive waypoint control requires control_mode="
            + repr(INTERACTIVE_CONTROL_MODE)
        )
    if profile.get("ground_truth_navigation") is True:
        raise ValueError(
            "interactive waypoint control refuses a ground-truth navigation profile"
        )
    pose_source = profile.get("slam_pose_source")
    if pose_source not in MEASURED_POSE_SOURCES:
        raise ValueError(
            "interactive waypoint control requires a measured estimator pose source"
        )
    if profile.get("navigation_pose_source") in (None, "", "native_truth"):
        raise ValueError(
            "interactive waypoint control requires an explicit measured "
            "navigation pose source label"
        )
    mailbox = profile.get("interactive_mailbox")
    if not isinstance(mailbox, str) or not mailbox:
        raise ValueError("interactive waypoint control requires interactive_mailbox")
    if not os.path.isabs(mailbox):
        raise ValueError("interactive_mailbox must be an absolute path")
    speed = _finite(profile.get("speed_mps"), "speed_mps")
    if not 0 < speed <= 1.0:
        raise ValueError("speed_mps must be within (0, 1] m/s")
    motion_end = _finite(profile.get("motion_end_s"), "motion_end_s")
    if motion_end <= 0:
        raise ValueError("motion_end_s must be positive")
    ttl = _finite(profile.get("interactive_waypoint_ttl_s", 30.0), "interactive_waypoint_ttl_s")
    if not 1.0 <= ttl <= 120.0:
        raise ValueError("interactive_waypoint_ttl_s must be within [1, 120] s")
    link = _finite(
        profile.get("interactive_link_timeout_s", 5.0), "interactive_link_timeout_s"
    )
    if not 0.5 <= link <= 60.0:
        raise ValueError("interactive_link_timeout_s must be within [0.5, 60] s")
    pose_loss = _finite(
        profile.get("interactive_pose_loss_timeout_s", 5.0),
        "interactive_pose_loss_timeout_s",
    )
    if not 0.5 <= pose_loss <= 60.0:
        raise ValueError("interactive_pose_loss_timeout_s must be within [0.5, 60] s")
    max_queue = profile.get("interactive_max_queue", 4)
    if isinstance(max_queue, bool) or not isinstance(max_queue, int):
        raise ValueError("interactive_max_queue must be an integer")
    if not 1 <= max_queue <= 8:
        raise ValueError("interactive_max_queue must be within [1, 8]")
    max_segment = _finite(
        profile.get("interactive_max_segment_m", 8.0), "interactive_max_segment_m"
    )
    if not 0.5 <= max_segment <= 12.0:
        raise ValueError("interactive_max_segment_m must be within [0.5, 12] m")
    min_segment = _finite(
        profile.get("interactive_min_segment_m", 0.15), "interactive_min_segment_m"
    )
    if not 0.05 <= min_segment <= 1.0 or min_segment >= max_segment:
        raise ValueError(
            "interactive_min_segment_m must be within [0.05, 1] m and below the maximum"
        )
    capture_remaining = _finite(
        profile.get("interactive_capture_remaining_m", 0.20),
        "interactive_capture_remaining_m",
    )
    capture_distance = _finite(
        profile.get("interactive_capture_distance_m", 0.30),
        "interactive_capture_distance_m",
    )
    if not 0 < capture_remaining <= capture_distance <= 0.40:
        raise ValueError(
            "interactive capture bounds require 0 < remaining <= distance <= 0.40 m"
        )
    if profile.get("interactive_simulation_only", True) is not True:
        raise ValueError(
            "interactive waypoint control is simulation-only; declare "
            "interactive_simulation_only=true"
        )
    require_perception = profile.get("interactive_require_perception", True)
    if not isinstance(require_perception, bool):
        raise ValueError("interactive_require_perception must be a boolean")
    forward_turn_mode = profile.get("interactive_forward_turn_mode", False)
    if not isinstance(forward_turn_mode, bool):
        raise ValueError("interactive_forward_turn_mode must be a boolean")
    min_clearance = _finite(
        profile.get("interactive_min_clearance_m", 0.55), "interactive_min_clearance_m"
    )
    if not 0.2 <= min_clearance <= 2.0:
        raise ValueError("interactive_min_clearance_m must be within [0.2, 2.0] m")
    yaw_settings = yaw_config(profile)
    return {
        "require_perception": require_perception,
        "forward_turn_mode": forward_turn_mode,
        "perception_max_wall_age_s": _finite(
            profile.get("interactive_perception_max_wall_age_s", 0.5),
            "interactive_perception_max_wall_age_s",
        ),
        "perception_max_sim_age_s": _finite(
            profile.get("interactive_perception_max_sim_age_s", 0.30),
            "interactive_perception_max_sim_age_s",
        ),
        "perception_loss_timeout_s": _finite(
            profile.get("interactive_perception_loss_timeout_s", 5.0),
            "interactive_perception_loss_timeout_s",
        ),
        "min_clearance_m": min_clearance,
        "min_cloud_points": int(profile.get("interactive_perception_min_cloud_points", 1000)),
        "perception_base_frame": str(
            profile.get("interactive_perception_base_frame", "go2_base")
        ),
        "min_depth_valid_ratio": _finite(
            profile.get("interactive_perception_min_depth_valid_ratio", 0.90),
            "interactive_perception_min_depth_valid_ratio",
        ),
        # Independent, small future-stamp tolerance, matching the existing route
        # gate (slam_route.future_tolerance_s = 2e-6 for float32 clock noise). A
        # future stamp is never allowed to borrow the staleness allowance.
        "future_tolerance_s": _finite(
            profile.get("interactive_future_tolerance_s", 2e-6),
            "interactive_future_tolerance_s",
        ),
        **yaw_settings,
        "pose_source": pose_source,
        "mailbox": mailbox,
        "speed_mps": speed,
        "motion_end_s": motion_end,
        "ttl_s": ttl,
        "link_timeout_s": link,
        "pose_loss_timeout_s": pose_loss,
        "max_queue": max_queue,
        "max_segment_m": max_segment,
        "min_segment_m": min_segment,
        "capture_remaining_m": capture_remaining,
        "capture_distance_m": capture_distance,
        "max_sim_age_s": _finite(
            profile.get("slam_max_sim_age_s", 0.25), "slam_max_sim_age_s"
        ),
        "max_wall_age_s": _finite(
            profile.get("slam_max_wall_age_s", 0.5), "slam_max_wall_age_s"
        ),
        "pose_speed_window_s": _finite(
            profile.get("slam_pose_speed_window_s", 0.5), "slam_pose_speed_window_s"
        ),
        "twist_available": bool(profile.get("slam_twist_available", True)),
        "stop_on_stall": bool(profile.get("stop_on_stall", True)),
    }


def pose_transform(position, quat_wxyz):
    """Validate measured XYZ and a WXYZ quaternion; return the SE(3) transform.

    This is the single measured-pose conversion used by the tracker and by the
    operator-side map snapshot; both refuse anything that is not a unit
    quaternion, so a mislabelled frame cannot silently become a pose.
    """
    xyz = _vec3(position, "position")
    try:
        quaternion = np.asarray(quat_wxyz, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError("quaternion_wxyz must contain 4 finite numbers") from error
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("quaternion_wxyz must contain 4 finite numbers")
    norm = float(np.linalg.norm(quaternion))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError("quaternion_wxyz must be unit length within 1e-3")
    w, x, y, z = quaternion / norm
    transform = np.eye(4)
    transform[:3, :3] = quaternion_matrix(w, x, y, z)
    transform[:3, 3] = xyz
    return transform


class WaypointQueueController:
    """Deterministic continuous follower over an operator-supplied waypoint queue.

    The command envelope, cross-track/overshoot fail-closed bounds, stall test
    and motion-budget deadline are the ones the existing measured-route
    controller already uses (0.05 m/s lateral, 0.15 rad/s yaw, 0.6 m corridor,
    0.35 m overshoot, 8 s and 0.02 m stall). One difference is explicit: an
    intermediate waypoint is passed through inside the capture bounds instead of
    forcing a full stop and an in-place turn, so the base can keep tracking at
    the commanded ceiling. The last waypoint still commands zero.

    arrived stays False for the whole run: reaching the last waypoint is a
    per-request result, not a terminal state, because the operator is expected to
    publish the next waypoint from the newly observed map.
    """

    def __init__(
        self,
        *,
        speed_mps,
        motion_end_s,
        stop_on_stall=True,
        capture_remaining_m=0.20,
        capture_distance_m=0.30,
        max_queue=4,
        min_segment_m=0.15,
        max_segment_m=8.0,
        initial_wait_s=7.0,
        stall_window_s=8.0,
        stall_min_progress_m=0.02,
        cross_track_limit_m=0.6,
        overshoot_limit_m=0.35,
        capture_z_tolerance_m=0.18,
        allow_lateral=True,
    ):
        """Bind validated limits; performs no I/O and no simulator calls."""
        self.speed = _finite(speed_mps, "speed_mps")
        if self.speed <= 0:
            raise ValueError("speed_mps must be positive")
        self.motion_end = _finite(motion_end_s, "motion_end_s")
        if self.motion_end <= 0:
            raise ValueError("motion_end_s must be positive")
        self.stop_on_stall = _bool(stop_on_stall, "stop_on_stall")
        self.capture_remaining_m = _finite(capture_remaining_m, "capture_remaining_m")
        self.capture_distance_m = _finite(capture_distance_m, "capture_distance_m")
        if not (0 < self.capture_remaining_m <= self.capture_distance_m <= 0.40):
            raise ValueError(
                "capture bounds require 0 < remaining <= distance <= 0.40 m"
            )
        if isinstance(max_queue, bool) or not isinstance(max_queue, int):
            raise ValueError("max_queue must be an integer")
        if not 1 <= max_queue <= 8:
            raise ValueError("max_queue must be within [1, 8]")
        self.max_queue = max_queue
        self.min_segment_m = _finite(min_segment_m, "min_segment_m")
        if self.min_segment_m <= 0:
            raise ValueError("min_segment_m must be positive")
        self.max_segment_m = _finite(max_segment_m, "max_segment_m")
        if self.max_segment_m <= self.min_segment_m:
            raise ValueError("max_segment_m must exceed min_segment_m")
        self.initial_wait_s = _finite(initial_wait_s, "initial_wait_s")
        self.stall_window_s = _finite(stall_window_s, "stall_window_s")
        self.stall_min_progress_m = _finite(
            stall_min_progress_m, "stall_min_progress_m"
        )
        self.cross_track_limit_m = _finite(cross_track_limit_m, "cross_track_limit_m")
        self.overshoot_limit_m = _finite(overshoot_limit_m, "overshoot_limit_m")
        self.capture_z_tolerance_m = _finite(
            capture_z_tolerance_m, "capture_z_tolerance_m"
        )
        # Explicit production forward+turn policy: the follower steers with yaw
        # only and never emits a lateral body command. This is not a post-gate
        # clamp: the policy itself cannot produce vy, so the side-direction gate
        # is never asked to authorise a lateral motion.
        self.allow_lateral = bool(allow_lateral)
        self.queue = []
        self.last_point = None
        self.phase = "idle"
        self.stage = 0
        self.completed_goals = 0
        self.failure = None
        self.arrived = False
        self.events = []
        self.last_reached_point = None
        self.drop_reason = None
        self.distance_to_goal_m = None
        self.remaining_m = None
        self.z_error_m = None
        self.cross_track_m = None
        self._stall = None

    def enqueue(self, goal_xyz):
        """Queue one measured-frame waypoint; return (accepted, reason, planar_m)."""
        point = _vec3(goal_xyz, "goal_xyz")
        if abs(point[0]) > MAX_GOAL_ABS_XY_M or abs(point[1]) > MAX_GOAL_ABS_XY_M:
            return False, "goal_outside_xy_bounds", None
        if abs(point[2]) > MAX_GOAL_ABS_Z_M:
            return False, "goal_outside_z_bounds", None
        if len(self.queue) >= self.max_queue:
            return False, "queue_full", None
        reference = None
        if self.queue:
            reference = self.queue[-1]
        elif self.last_point is not None:
            reference = self.last_point
        planar = None
        if reference is not None:
            planar = math.hypot(point[0] - reference[0], point[1] - reference[1])
            if planar > self.max_segment_m:
                return False, "segment_too_long", planar
            if planar < self.min_segment_m:
                return False, "segment_too_short", planar
            if abs(point[2] - reference[2]) > MAX_VERTICAL_STEP_M:
                return False, "segment_vertical_jump", planar
        self.queue.append(point)
        if self.phase in ("idle", "arrived"):
            self.phase = "waiting"
        return True, "queued", planar

    def clear(self, reason):
        """Drop the queued waypoints and keep the controller reusable."""
        dropped = len(self.queue)
        self.queue.clear()
        self._stall = None
        self.phase = "idle"
        self.distance_to_goal_m = None
        self.remaining_m = None
        self.z_error_m = None
        self.cross_track_m = None
        # drop_reason is reserved for geometry drops detected while tracking; an
        # operator clear is reported through its own receipt instead.
        self.drop_reason = None
        self.events.append(
            {"event": "queue_cleared", "reason": reason, "dropped": dropped}
        )
        return dropped

    def invalidate_measurement(self):
        """Drop the dwell/stall window without changing phase or failure."""
        self._stall = None

    def _finish_goal(self, goal, t, position, distance, z_error, remaining):
        """Pop the head goal and record the reached receipt event."""
        self.queue.pop(0)
        self.stage += 1
        self.completed_goals += 1
        self.last_point = goal.copy()
        self.last_reached_point = goal.copy()
        self._stall = None
        self.events.append(
            {
                "event": "waypoint_reached",
                "stage": self.stage - 1,
                "time_s": t,
                "goal_xyz_m": goal.tolist(),
                "position_xyz_m": list(position),
                "distance_m": distance,
                "remaining_m": remaining,
                "z_error_m": z_error,
                "stop_confirmed": False,
                "queue_remaining": len(self.queue),
            }
        )
        if not self.queue:
            self.phase = "idle"
            self.distance_to_goal_m = None
            self.remaining_m = None
            self.z_error_m = None
            self.cross_track_m = None

    def tick(self, t, p, yaw, speed, omega, ready=True):
        """Return one bounded body command [vx, vy, wz] for a measured pose.

        t is simulator time, p the measured XYZ in the estimator map frame, yaw
        the measured heading, speed the measured speed scalar (or the 0.04 gate
        sentinel when the window is not ready) and omega the measured yaw rate.
        ready is the caller's admitted-sensor-control flag; it is the only
        condition under which the start pose is latched.
        """
        command = [0.0, 0.0, 0.0]
        if self.failure or self.arrived:
            return command
        if t >= self.motion_end:
            self.failure = "route_motion_budget"
            return command
        if not ready or t < self.initial_wait_s:
            return command
        if not self.queue:
            self.phase = "idle"
            return command
        goal = self.queue[0]
        if self.last_point is None:
            # The first admitted sample latches the measured start pose. The
            # first leg can only be length-checked now, so a too long first
            # request is dropped instead of being driven.
            self.last_point = np.array(p, dtype=float)
            planar = math.hypot(goal[0] - p[0], goal[1] - p[1])
            if planar > self.max_segment_m + 1e-9:
                self.drop_reason = "first_leg_too_long"
                self.events.append(
                    {
                        "event": "dropped_first_leg_too_long",
                        "time_s": t,
                        "goal_xyz_m": goal.tolist(),
                        "position_xyz_m": list(p),
                        "planar_m": planar,
                        "max_segment_m": self.max_segment_m,
                    }
                )
                self.queue.clear()
                self.phase = "idle"
                return command
        origin = self.last_point[:2]
        dx = goal[0] - origin[0]
        dy = goal[1] - origin[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            self._finish_goal(goal, t, p, 0.0, abs(p[2] - goal[2]), 0.0)
            return command
        u = (dx / length, dy / length)
        normal = (-u[1], u[0])
        heading = math.atan2(dy, dx)
        error = wrap(heading - yaw)
        remaining = (goal[0] - p[0]) * u[0] + (goal[1] - p[1]) * u[1]
        cross = (p[0] - origin[0]) * normal[0] + (p[1] - origin[1]) * normal[1]
        distance = math.hypot(goal[0] - p[0], goal[1] - p[1])
        z_error = abs(p[2] - goal[2])
        self.distance_to_goal_m = distance
        self.remaining_m = remaining
        self.z_error_m = z_error
        self.cross_track_m = cross
        if abs(cross) > self.cross_track_limit_m:
            self.failure = "route_cross_track_bound"
            return command
        if remaining < -self.overshoot_limit_m:
            self.failure = "route_waypoint_overshoot"
            return command
        if (
            remaining <= self.capture_remaining_m
            and distance <= self.capture_distance_m
            and z_error <= self.capture_z_tolerance_m
        ):
            self._finish_goal(goal, t, p, distance, z_error, remaining)
            return command
        if self.phase in ("waiting", "idle"):
            self.phase = "cruise"
        command[0] = (
            min(self.speed, max(0.04, 0.5 * max(remaining, 0.0)))
            if abs(error) < 0.25
            else 0.0
        )
        correction = -0.5 * cross
        world = (correction * normal[0], correction * normal[1])
        if self.allow_lateral:
            command[1] = clip(-math.sin(yaw) * world[0] + math.cos(yaw) * world[1], 0.05)
        else:
            command[1] = 0.0
        command[2] = clip(0.8 * error, 0.15)
        if command[0] > 0:
            along = (p[0] - origin[0]) * u[0] + (p[1] - origin[1]) * u[1]
            if self._stall is None:
                self._stall = (t, along)
            elif t - self._stall[0] >= self.stall_window_s:
                if along - self._stall[1] < self.stall_min_progress_m:
                    self.events.append(
                        {
                            "event": "observed_low_progress_stall",
                            "stage": self.stage,
                            "time_s": t,
                            "window_start_s": self._stall[0],
                            "forward_progress_m": along - self._stall[1],
                            "position_xyz_m": list(p),
                            "request": command.copy(),
                            "stop_on_stall": self.stop_on_stall,
                        }
                    )
                    if self.stop_on_stall:
                        self.failure = "route_low_progress_stall"
                self._stall = (t, along)
        else:
            self._stall = None
        return command


class InteractiveWaypointTracker:
    """Fail-closed operator mailbox adapter around the waypoint follower.

    Responsibilities, in order: poll the mailbox, enforce sequence/TTL/link and
    measured-pose-source gates, hold zero whenever the measured pose is missing,
    stale, mislabelled or mismatched, delegate tracking to
    WaypointQueueController, and publish an observation bundle the operator can
    act on. Every rejection and every terminal event is written to
    receipts.jsonl; the tracker never re-executes a sequence number.
    """

    def __init__(self, experiment, *, speed_mps=None, mailbox=None):
        """Bind one validated interactive profile and create its mailbox files."""
        config = validate_interactive_profile(experiment)
        self.config = config
        self.pose_source = config["pose_source"]
        self.mailbox = Path(mailbox if mailbox is not None else config["mailbox"])
        self.mailbox.mkdir(parents=True, exist_ok=True)
        self.request_path = self.mailbox / "request.json"
        self.heartbeat_path = self.mailbox / "heartbeat.json"
        self.observation_path = self.mailbox / "observation.json"
        self.perception_path = self.mailbox / "perception.json"
        self.receipts_path = self.mailbox / "receipts.jsonl"
        self.run_id = self.mailbox.parent.name
        self.controller = WaypointQueueController(
            speed_mps=config["speed_mps"] if speed_mps is None else speed_mps,
            motion_end_s=config["motion_end_s"],
            stop_on_stall=config["stop_on_stall"],
            capture_remaining_m=config["capture_remaining_m"],
            capture_distance_m=config["capture_distance_m"],
            allow_lateral=not config["forward_turn_mode"],
            max_queue=config["max_queue"],
            min_segment_m=config["min_segment_m"],
            max_segment_m=config["max_segment_m"],
        )
        self._pending = []
        self._last_seq = 0
        self._request_digest = None
        self._last_poll_wall = -math.inf
        self._last_observation_wall = -math.inf
        self._terminal_reason = None
        self._link_established = False
        self._link_gate_reported = False
        self._last_heartbeat_wall = None
        self._hold_since_wall = None
        self._path_length_m = 0.0
        self._pose_samples = 0
        self._last_position = None
        self._last_command = [0.0, 0.0, 0.0]
        self._last_sim_s = None
        self._last_odom_stamp_s = None
        self._last_consumed_odom_s = None
        self._frame_labels = None
        self._pose_history = deque()
        self._last_valid_measurement = None
        self._native_guards = {
            "tilt_rad": None,
            "native_speed_mps": None,
            "usable_for_navigation": False,
            "label": "native simulator state; policy observation and physical guards only",
        }
        self._io_errors = 0
        self._seen_completed_goals = 0
        self._seen_drop_reason = None
        self._last_waypoint_result = None
        self._observation_writes = 0
        self._action = None
        self._perception = None
        self._action_completed = 0
        self._perception_file_cache = None
        self._perception_file_version = None
        self._perception_file_authoritative = False
        self._perception_admission = None
        self._tick_counter = 0
        self._yaw_context = None
        self._yaw_window = None
        self._yaw_window_reset = None
        self._last_validated_perception = None
        self._perception_current = False
        self._not_ready_latched = False

    # ------------------------------------------------------------- properties
    @property
    def phase(self):
        """Current follower phase (idle/waiting/cruise/terminal)."""
        return "terminal" if self._terminal_reason is not None else self.controller.phase

    @property
    def stage(self):
        """Number of waypoints reached so far."""
        return self.controller.stage

    @property
    def failure(self):
        """Follower failure reason, or None."""
        return self.controller.failure

    @property
    def arrived(self):
        """Whether the operator asked the run to stop (terminal, not a failure)."""
        return self._terminal_reason == "interactive_stop"

    @property
    def terminal_reason(self):
        """Terminal reason that the control loop must turn into a parked stop."""
        return self._terminal_reason

    @property
    def events(self):
        """Recorded follower events."""
        return self.controller.events

    @property
    def parameters(self):
        """Receipt copy of the active interaction limits."""
        return {
            "pose_source": self.pose_source,
            "speed_mps": self.controller.speed,
            "motion_end_s": self.controller.motion_end,
            "capture_remaining_m": self.controller.capture_remaining_m,
            "capture_distance_m": self.controller.capture_distance_m,
            "max_queue": self.controller.max_queue,
            "max_segment_m": self.controller.max_segment_m,
            "min_segment_m": self.controller.min_segment_m,
            "ttl_s": self.config["ttl_s"],
            "link_timeout_s": self.config["link_timeout_s"],
            "pose_loss_timeout_s": self.config["pose_loss_timeout_s"],
            "max_sim_age_s": self.config["max_sim_age_s"],
            "max_wall_age_s": self.config["max_wall_age_s"],
            "command_bounds": {
                "forward_mps": self.controller.speed,
                "lateral_mps": 0.05,
                "yaw_rate_rps": 0.15,
            },
            "command_source": "deterministic_waypoint_follower",
            "require_perception": self.config["require_perception"],
            "forward_turn_mode": self.config["forward_turn_mode"],
            "perception_limits": {
                "max_wall_age_s": self.config["perception_max_wall_age_s"],
                "max_sim_age_s": self.config["perception_max_sim_age_s"],
                "min_clearance_m": self.config["min_clearance_m"],
                "loss_timeout_s": self.config["perception_loss_timeout_s"],
            },
            "local_action_bounds": dict(LOCAL_ACTION_BOUNDS),
            "ground_truth_navigation": False,
            "simulation_only": True,
            "map_source": "operator_side_from_live_measured_topics",
        }

    # ------------------------------------------------------------- operator I/O
    def _write_receipt(self, event, waypoint_result=False, **fields):
        """Append one receipt line; I/O errors never break the control tick."""
        payload = {
            "protocol_version": PROTOCOL_VERSION,
            "event": event,
            "run_id": self.run_id,
        }
        payload.update(fields)
        try:
            with self.receipts_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(payload, sort_keys=True) + "\n")
        except OSError:
            self._io_errors += 1
        if waypoint_result:
            self._last_waypoint_result = {
                "event": event,
                "seq": payload.get("seq"),
                "reason": payload.get("reason"),
                "goal_xyz_m": payload.get("goal_xyz_m"),
            }

    def _reject(self, payload, reason, **extra):
        """Record one rejected operator request without queueing anything."""
        fields = {
            "seq": payload.get("seq"),
            "reason": reason,
            "action": payload.get("action"),
        }
        fields.update(extra)
        self._write_receipt("request_rejected", waypoint_result=True, **fields)
        return False, reason

    def _clear_for(self, reason):
        """Cancel queued work, zero the command and mirror the operator state."""
        self.controller.clear(reason)
        self._action = None
        self._pending.clear()
        self._seen_completed_goals = self.controller.completed_goals
        self._last_command = [0.0, 0.0, 0.0]

    def _terminate(self, reason, **extra):
        self._reset_yaw_window(reason, source="terminal")
        """Latch a terminal reason once, clear the queue and command zero."""
        if self._terminal_reason is not None:
            return
        self._clear_for(reason)
        self._terminal_reason = reason
        self._write_receipt("terminal", reason=reason, **extra)

    def _accept_local_action(self, payload, action, created_wall_s, ttl_s):
        """Accept an in-place scan turn or a short bounded local manoeuvre."""
        try:
            if action == "yaw":
                parameters = yaw_request_parameters(payload)
                parameters["tolerance_rad"] = LOCAL_ACTION_BOUNDS["yaw_tolerance_rad"]
            else:
                forward = _finite(payload.get("body_forward_mps", 0.0), "body_forward")
                lateral = _finite(payload.get("body_lateral_mps", 0.0), "body_lateral")
                yaw_rate = _finite(payload.get("yaw_rate_rps", 0.0), "yaw_rate")
                duration = _finite(payload.get("duration_s"), "duration_s")
                if abs(forward) > self.controller.speed:
                    raise ValueError("forward command exceeds the speed ceiling")
                if abs(lateral) > LOCAL_ACTION_BOUNDS["lateral_mps"]:
                    raise ValueError("lateral command out of bounds")
                if self.config["forward_turn_mode"] and lateral != 0.0:
                    raise ValueError("forward_turn_mode_lateral_refused")
                if abs(yaw_rate) > LOCAL_ACTION_BOUNDS["yaw_rate_rps"]:
                    raise ValueError("yaw rate out of bounds")
                if not 0.1 <= duration <= LOCAL_ACTION_BOUNDS["maneuver_max_duration_s"]:
                    raise ValueError("duration out of bounds")
                parameters = {
                    "body_forward_mps": forward,
                    "body_lateral_mps": lateral,
                    "yaw_rate_rps": yaw_rate,
                    "duration_s": duration,
                }
        except ValueError as error:
            reason = str(error)
            self._reject(
                payload,
                reason
                if reason == "forward_turn_mode_lateral_refused"
                else "invalid_" + action + "_parameters",
            )
            return
        if action == "maneuver":
            direction_reason = self._direction_reason(parameters, created_wall_s)
            if direction_reason is not None:
                self._reject(payload, direction_reason)
                return
        self._clear_for("operator_local_action")
        self._reset_yaw_window("new_action", source="request")
        self._action = {
            "kind": action,
            "seq": payload.get("seq"),
            "since_sim_s": None,
            "target_yaw_rad": None,
            "parameters": parameters,
            "completion": {
                "mode": self.config["yaw_completion_mode"],
                "tolerance_rad": self.config["yaw_tolerance_rad"],
                "settle_rate_radps": self.config["yaw_settle_rate_radps"],
                "dwell_sim_s": self.config["yaw_dwell_sim_s"],
                "eval_max_gap_sim_s": self.config["yaw_eval_max_gap_sim_s"],
                "imu_max_sim_age_s": self.config["imu_max_sim_age_s"],
                "imu_max_wall_age_s": self.config["imu_max_wall_age_s"],
            },
            "expires_wall_s": created_wall_s + ttl_s,
        }
        if action == "yaw":
            # The effective dwell tolerance replaces the request-side default so the
            # accepted receipt and the live observation report the real criterion.
            parameters["tolerance_rad"] = self.config["yaw_tolerance_rad"]
        self._write_receipt(
            action + "_accepted",
            waypoint_result=True,
            seq=payload.get("seq"),
            action=action,
            parameters=parameters,
            completion={
                "mode": self.config["yaw_completion_mode"],
                "tolerance_rad": self.config["yaw_completion_tolerance_rad"],
                "control_tolerance_rad": self.config["yaw_control_tolerance_rad"],
                "completion_tolerance_rad": self.config["yaw_completion_tolerance_rad"],
                "settle_rate_radps": self.config["yaw_settle_rate_radps"],
                "dwell_sim_s": self.config["yaw_dwell_sim_s"],
                "eval_max_gap_sim_s": self.config["yaw_eval_max_gap_sim_s"],
                "imu_max_sim_age_s": self.config["imu_max_sim_age_s"],
                "imu_max_wall_age_s": self.config["imu_max_wall_age_s"],
            },
            ttl_s=ttl_s,
        )

    def update_perception(
        self,
        *,
        now_wall,
        stamp_s=None,
        clearance_m=None,
        source=None,
        frame_id=None,
        points=0,
        observation_state="measured",
        depth_valid_ratio=None,
        directions=None,
    ):
        """Record one fresh static-perception sample for the obstacle gate.

        The gate treats a missing, stale, unlabelled or ground-truth-sourced
        sample as 'no permission to move'. A clearance value is mandatory: the
        caller must measure it from live depth or the live static point cloud.
        """
        # The public entry point validates its own clocks and clearances: a NaN or
        # infinite arrival would make every age comparison false and authorise
        # motion, so a broken sample raises instead of being stored. The
        # production file reader already does this, but the interface must not
        # rely on having no other caller.
        now = _finite(now_wall, "now_wall")
        stamp = None if stamp_s is None else _finite(stamp_s, "stamp_s")
        clearance = None if clearance_m is None else _finite(clearance_m, "clearance_m")
        ratio = (
            None
            if depth_valid_ratio is None
            else _finite(depth_valid_ratio, "depth_valid_ratio")
        )
        if isinstance(points, bool) or not isinstance(points, (int, float)):
            raise ValueError("points must be a non-negative integer")
        if points < 0 or float(points) != int(points):
            raise ValueError("points must be a non-negative integer")
        checked_directions = None
        if directions is not None:
            checked_directions = {}
            for name, value in dict(directions).items():
                entry = copy.deepcopy(dict(value))
                if entry.get("clearance_m") is not None:
                    entry["clearance_m"] = _finite(
                        entry["clearance_m"], f"direction {name} clearance_m"
                    )
                checked_directions[str(name)] = entry
        self._perception = {
            "stamp_s": stamp,
            "arrival_wall_s": now,
            "clearance_m": clearance,
            "source": None if source is None else str(source),
            "frame_id": None if frame_id is None else str(frame_id),
            "points": int(points),
            "observation_state": str(observation_state),
            "depth_valid_ratio": ratio,
            "directions": checked_directions,
        }
        # A caller must publish a new sample, not mutate an admitted version via
        # aliases in its input directions or this public return value.
        return copy.deepcopy(self._perception)

    def invalidate_perception(self):
        """Forget the last perception sample; the next tick holds zero."""
        self._perception = None

    def _read_perception_file(self, now):
        """Read the live perception sample written by the capture process.

        Check the actual file version on every call, but only read/parse changed
        bytes. The source clocks remain untouched and all gates still run on
        every normal tick. A racing replacement holds for this call and retries
        on the next one, without a retry loop or an old-clearance fallback.
        """
        def version(info):
            return (info.st_dev, info.st_ino, info.st_mtime_ns,
                    info.st_ctime_ns, info.st_size)

        try:
            # NONBLOCK keeps a malformed FIFO mailbox from blocking control.
            # Opening even a cached version verifies it is still readable.
            with os.fdopen(os.open(self.perception_path, os.O_RDONLY | os.O_NONBLOCK),
                           "rb") as stream:
                self._perception_file_authoritative = True
                before = os.fstat(stream.fileno())
                file_version = version(before)
                if (not stat.S_ISREG(before.st_mode)
                        or before.st_size > MAX_PERCEPTION_FILE_BYTES):
                    raise OSError("invalid perception file type or size")
                if file_version == self._perception_file_version:
                    if version(self.perception_path.stat()) != file_version:
                        raise OSError("perception file replaced during cache lookup")
                    return self._perception_file_cache
                raw = stream.read(MAX_PERCEPTION_FILE_BYTES + 1)
                if (len(raw) > MAX_PERCEPTION_FILE_BYTES
                        or len(raw) != before.st_size
                        or version(os.fstat(stream.fileno())) != file_version
                        or version(self.perception_path.stat()) != file_version):
                    raise OSError("perception file changed during read")
        except OSError as error:
            # No file ever published retains the explicit in-process API. Once
            # a file was observed, deletion/read errors cannot revive that API's
            # older sample. Neither error nor race gets a cached fingerprint.
            if not isinstance(error, FileNotFoundError):
                self._perception_file_authoritative = True
            self._perception_file_version = None
            self._perception_file_cache = None
            return None
        try:
            payload = json.loads(raw)
        except (ValueError, RecursionError):
            payload = None
        sample = None
        if payload is not None and isinstance(payload, dict):
            try:
                sample = {
                    "stamp_s": _finite(payload.get("stamp_s"), "stamp_s"),
                    "arrival_wall_s": _finite(
                        payload.get("arrival_wall_s"), "arrival_wall_s"
                    ),
                    "clearance_m": (
                        None
                        if payload.get("clearance_m") is None
                        else _finite(payload.get("clearance_m"), "clearance_m")
                    ),
                    "source": payload.get("source"),
                    "safety_fusion": payload.get("safety_fusion"),
                    "corridor": payload.get("corridor"),
                    "frame_id": payload.get("frame_id"),
                    "points": int(payload.get("corridor_points") or 0),
                    "cloud_points": int(payload.get("cloud_points") or 0),
                    "version": payload.get("version"),
                    "observation_state": payload.get("observation_state"),
                    "depth_valid_ratio": (
                        None
                        if payload.get("depth_valid_ratio") is None
                        else _finite(
                            payload.get("depth_valid_ratio"), "depth_valid_ratio"
                        )
                    ),
                    "directions": (
                        None
                        if not isinstance(payload.get("directions"), dict)
                        else {
                            str(name): dict(value)
                            for name, value in payload["directions"].items()
                            if isinstance(value, dict)
                        }
                    ),
                }
            except (ValueError, TypeError, OverflowError):
                sample = None
        self._perception_file_version = file_version
        self._perception_file_cache = sample
        return sample

    @staticmethod
    def _floor_supported(entry, source):
        """Reject partial support, including legacy RTX labels hiding empty bins.

        The fixed RTX producer corridor starts at 0.35 m. Reuse its existing
        bin/count contract, never a payload-selected smaller observable region.
        RGB-D has no floor contract and retains its existing behavior.
        """
        from .rtx_deskewed_perception import (
            CORRIDOR_BINS, CORRIDOR_MIN_M, MIN_RADIAL_POINTS_PER_BIN,
        )
        rtx = str(source or "").startswith(("rtx_lidar_", "rtx_depth_safety_fusion"))
        if not isinstance(entry, dict):
            return not rtx
        state = entry.get("floor_observation_state")
        if state is not None and state != "supported":
            return False
        if entry.get("floor_support_complete") is False:
            return False
        counts = entry.get("radial_bin_floor_points")
        if rtx or counts is not None:
            if state != "supported" or not isinstance(counts, list):
                return False
            if len(counts) != CORRIDOR_BINS:
                return False
            try:
                blind = entry.get("near_field_floor_unobserved_m")
                if rtx and blind is None:
                    return False
                if blind is not None:
                    if not isinstance(blind, list) or len(blind) != 2:
                        return False
                    end = float(blind[1])
                    if not math.isfinite(end) or end > CORRIDOR_MIN_M + 1e-9:
                        return False
                return all(
                    math.isfinite(float(count))
                    and float(count) >= MIN_RADIAL_POINTS_PER_BIN
                    for count in counts
                )
            except (TypeError, ValueError):
                return False
        return True

    def _perception_gate(self, now, sim):
        """Return (reason, fields, terminal_reason) when static perception fails."""
        # Telemetry state: only a sample that passes the gate in this tick may be
        # reported as the current validated input. A rejected or stale sample leaves
        # it False, so an older successful sample is never presented as current.
        self._perception_current = False
        self._perception_admission = None
        if not self.config["require_perception"]:
            return None
        sample = self._read_perception_file(now)
        if sample is None and not self._perception_file_authoritative:
            sample = self._perception
        if sample is None:
            return ("perception_unavailable", {"perception": None},
                    "interactive_perception_lost")
        age_wall = now - sample["arrival_wall_s"]
        stamp = sample["stamp_s"]
        age_sim = None if stamp is None else sim - stamp
        fields = {
            "perception": {
                "source": sample["source"],
                "frame_id": sample["frame_id"],
                "clearance_m": sample["clearance_m"],
                "stamp_s": stamp,
                "points": sample["points"],
                "cloud_points": sample.get("cloud_points"),
                "observation_state": sample.get("observation_state"),
                "depth_valid_ratio": sample.get("depth_valid_ratio"),
                "front_floor_observation_state": (
                    None
                    if not isinstance(sample.get("directions"), dict)
                    or not isinstance(sample["directions"].get("front"), dict)
                    else sample["directions"]["front"].get(
                        "floor_observation_state"
                    )
                ),
            },
            "perception_wall_age_s": age_wall,
            "perception_sim_age_s": age_sim,
        }
        # Neither clock may be read from the future, and a negative age is never
        # clipped to zero: a future arrival or stamp is a broken perception sample.
        if age_wall < -self.config["future_tolerance_s"]:
            return (
                "perception_arrival_in_future",
                fields,
                "interactive_perception_lost",
            )
        if age_sim is not None and age_sim < -self.config["future_tolerance_s"]:
            return (
                "perception_stamp_in_future",
                fields,
                "interactive_perception_lost",
            )
        if age_wall > self.config["perception_max_wall_age_s"]:
            return ("perception_stale", fields, "interactive_perception_lost")
        if age_sim is not None and age_sim > self.config["perception_max_sim_age_s"]:
            return ("perception_stale", fields, "interactive_perception_lost")
        if sample.get("version") not in (None, 1):
            return ("perception_unlabelled", fields, "interactive_perception_lost")
        if (
            not sample["source"]
            or not sample["frame_id"]
            or sample["source"] in FORBIDDEN_POSE_SOURCES
        ):
            return ("perception_unlabelled", fields, "interactive_perception_lost")
        if sample["frame_id"] != self.config["perception_base_frame"]:
            return (
                "perception_frame_mismatch",
                fields,
                "interactive_perception_lost",
            )
        # A new source name must not turn a joint floor/clock contract into the
        # legacy RGB-D path. Recheck every source, including on cached commands.
        if str(sample.get("source") or "").startswith("rtx_depth_safety_fusion"):
            from .depth_safety_fusion import validate_fusion_contract
            try:
                validate_fusion_contract(sample, now_sim_s=sim, now_wall_s=now)
            except (ValueError, TypeError, KeyError) as error:
                fields["fusion_error"] = str(error)
                return ("perception_fusion_invalid", fields, "interactive_perception_lost")
        observation_state = sample.get("observation_state")
        if observation_state not in ("measured", "clear_free"):
            return (
                "perception_clearance_unknown",
                fields,
                "interactive_perception_lost",
            )
        if observation_state == "clear_free":
            ratio = sample.get("depth_valid_ratio")
            if ratio is None or ratio < self.config["min_depth_valid_ratio"]:
                return (
                    "perception_insufficient_observation",
                    fields,
                    "interactive_perception_lost",
                )
        cloud_points = sample.get("cloud_points")
        if cloud_points is not None and cloud_points < self.config["min_cloud_points"]:
            return (
                "perception_insufficient_cloud",
                fields,
                "interactive_perception_lost",
            )
        # A LiDAR sample also reports whether the ground under the travel corridor
        # was actually measured. A missing/dropped floor is unknown space: hold even
        # when a collision clearance happens to be present. RGB-D samples do not
        # carry the field, so the legacy path is unchanged.
        front_entry = (sample.get("directions") or {}).get("front")
        if not self._floor_supported(front_entry, sample.get("source")):
            return (
                "perception_floor_unsupported",
                fields,
                "interactive_obstacle_blocked",
            )
        clearance = sample["clearance_m"]
        if clearance is None or not math.isfinite(clearance):
            return (
                "perception_clearance_unknown",
                fields,
                "interactive_perception_lost",
            )
        if clearance < self.config["min_clearance_m"]:
            return ("obstacle_too_close", fields, "interactive_obstacle_blocked")
        # Keep the sample that actually passed every gate so the observation bundle
        # can show what was validated, not only why a later tick held. This is
        # telemetry only: it never feeds the admission or threshold logic.
        self._last_validated_perception = {
            "source": sample["source"],
            "frame_id": sample["frame_id"],
            "stamp_s": stamp,
            "clearance_m": clearance,
            "points": sample["points"],
            "cloud_points": cloud_points,
            "observation_state": observation_state,
            "depth_valid_ratio": sample.get("depth_valid_ratio"),
            "directions": (
                None
                if sample.get("directions") is None
                else {str(k): dict(v) for k, v in dict(sample["directions"]).items()}
            ),
            "arrival_wall_s": sample["arrival_wall_s"],
            "validated_wall_s": now,
            "validated_sim_s": sim,
            "validated_sim_s": sim,
        }
        self._perception_current = True
        # Bind this evaluation to the exact sample, not just its source clocks.
        # Unchanged files reuse their parsed object; every new file version or
        # update_perception call creates a new one, including in-process-only use.
        self._perception_admission = (now, sample)
        return None

    def _expire_local_action(self, now_wall):
        """Drop an unfinished local action whose request TTL elapsed."""
        action = self._action
        if action is None or action["expires_wall_s"] >= now_wall:
            return
        kind = action["kind"]
        self._action = None
        self._last_command = [0.0, 0.0, 0.0]
        self._write_receipt(
            "action_expired",
            waypoint_result=True,
            seq=action["seq"],
            reason=kind + "_ttl_expired",
        )

    def _direction_reason(self, parameters, now_wall):
        """Return a reason when a translation direction has no fresh coverage.

        A local action may only move in a direction that live perception actually
        observed; front-only evidence never authorises reversing or strafing. Every
        non-zero body translation counts, however small: a 0.01 m/s creep towards an
        unobserved or close obstacle is still a collision command. A clearance that
        is missing, non-finite or not a number is rejected before any comparison,
        never treated as "far".
        """
        sample = self._read_perception_file(now_wall)
        if sample is None and not self._perception_file_authoritative:
            sample = self._perception
        admission = self._perception_admission
        if (self._perception_current and admission is not None
                and admission[0] == now_wall and sample is not admission[1]):
            # Never combine A's full admission with B's unchecked directions.
            # Hold this command; the next normal tick can fully validate B.
            return "perception_changed_during_tick"
        directions = None if sample is None else sample.get("directions")
        if not isinstance(directions, dict):
            return "perception_directions_unavailable"
        required = []
        forward = float(parameters.get("body_forward_mps") or 0.0)
        lateral = float(parameters.get("body_lateral_mps") or 0.0)
        yaw_rate = float(parameters.get("yaw_rate_rps") or 0.0)
        if forward > 0.0:
            required.append("front")
        elif forward < 0.0:
            required.append("rear")
        if lateral > 0.0:
            required.append("left")
        elif lateral < 0.0:
            required.append("right")
        if not required and abs(yaw_rate) > 0.0:
            required.append("front")
        for name in required:
            entry = directions.get(name)
            if not isinstance(entry, dict):
                return "perception_direction_uncovered:" + name
            state = entry.get("observation_state")
            clearance = entry.get("clearance_m")
            if state not in ("measured", "clear_free") or clearance is None:
                return "perception_direction_uncovered:" + name
            # A real LiDAR sample carries its own measured floor-support state; an
            # unobserved/dropped ground surface is unknown and must hold even when a
            # collision-height clearance happens to be reported. RGB-D payloads do
            # not carry the field, so the legacy path is unchanged.
            if not self._floor_supported(entry, sample.get("source")):
                return "perception_floor_unsupported:" + name
            try:
                value = _finite(clearance, "clearance_m")
            except (TypeError, ValueError):
                return "perception_direction_invalid:" + name
            if value < self.config["min_clearance_m"]:
                return "obstacle_too_close:" + name
        return None
    def _local_action_command(self, sim, yaw, now_wall):
        """Return the bounded body command for the active local action."""
        action = self._action
        parameters = action["parameters"]
        if action["kind"] == "yaw":
            if ("frame_id" in parameters
                    and parameters["frame_id"] != self._frame_labels[0]):
                self._reset_yaw_window("yaw_frame_mismatch", source="request")
                self._action = None
                self._write_receipt("yaw_failed", waypoint_result=True,
                                    seq=action["seq"], reason="yaw_frame_mismatch")
                return [0.0, 0.0, 0.0]
            command, self._yaw_window, completion = yaw_action_step(
                action, self.config, self._yaw_context, self._yaw_window,
                yaw=yaw, now_wall=now_wall,
                future_tolerance_s=self._odometry_future_tolerance(sim),
            )
            if completion is not None:
                self._finish_local_action(completion)
            return command
        direction_reason = self._direction_reason(parameters, now_wall)
        if direction_reason == "perception_changed_during_tick":
            # A publication race is a one-tick hold, not an operator cancellation.
            # Keep the original action/TTL for full admission on the next tick.
            return [0.0, 0.0, 0.0]
        if direction_reason is not None and action["kind"] != "yaw":
            seq = action["seq"]
            self._action = None
            self._last_command = [0.0, 0.0, 0.0]
            self._write_receipt(
                "action_cancelled",
                waypoint_result=True,
                seq=seq,
                action=action["kind"],
                reason=direction_reason,
            )
            return [0.0, 0.0, 0.0]
        if action["since_sim_s"] is None:
            action["since_sim_s"] = sim
        elapsed = sim - action["since_sim_s"]
        if elapsed >= parameters["duration_s"]:
            self._finish_local_action({"elapsed_s": elapsed})
            return [0.0, 0.0, 0.0]
        return [
            parameters["body_forward_mps"],
            parameters["body_lateral_mps"],
            parameters["yaw_rate_rps"],
        ]

    def _finish_local_action(self, extra):
        """Close the active local action with a completion receipt."""
        action = self._action
        if action is None:
            return
        self._reset_yaw_window("action_finished", source=action["kind"])
        self._action = None
        self._action_completed += 1
        self._last_command = [0.0, 0.0, 0.0]
        self._write_receipt(
            action["kind"] + "_complete",
            waypoint_result=True,
            seq=action["seq"],
            action=action["kind"],
            **extra,
        )

    def _check_link(self, now_wall):
        """Enforce operator liveness from the heartbeat file.

        A heartbeat that cannot be trusted never refreshes liveness: a malformed
        payload, a future timestamp or a heartbeat from another run leaves the last
        valid heartbeat untouched, and its deadline is still checked on the same
        tick. Otherwise a single future-dated file would keep the link alive while
        the real operator had gone away.
        """
        payload, error = read_json_file(self.heartbeat_path)
        if payload is None:
            self._check_heartbeat_deadline(now_wall)
            return
        if not isinstance(payload, dict):
            self._write_receipt("heartbeat_rejected", reason="heartbeat_not_an_object")
            self._check_heartbeat_deadline(now_wall)
            if self._link_established:
                self._terminate("interactive_link_unreadable")
            return
        try:
            heartbeat_wall = _finite(payload.get("wall_s"), "heartbeat.wall_s")
        except (TypeError, ValueError):
            self._write_receipt("heartbeat_rejected", reason="heartbeat_unreadable")
            self._check_heartbeat_deadline(now_wall)
            if self._link_established:
                self._terminate("interactive_link_unreadable")
            return
        if heartbeat_wall > now_wall + 2.0:
            self._write_receipt("heartbeat_rejected", reason="heartbeat_from_future")
            self._check_heartbeat_deadline(now_wall)
            return
        heartbeat_run = payload.get("run_id")
        if (
            isinstance(heartbeat_run, str)
            and heartbeat_run
            and heartbeat_run != self.run_id
        ):
            self._write_receipt(
                "heartbeat_rejected",
                reason="heartbeat_run_id_mismatch",
                run_id=heartbeat_run,
            )
            self._check_heartbeat_deadline(now_wall)
            return
        self._link_established = True
        self._last_heartbeat_wall = heartbeat_wall
        if now_wall - heartbeat_wall > self.config["link_timeout_s"]:
            self._terminate(
                "interactive_link_lost", heartbeat_age_s=now_wall - heartbeat_wall
            )

    def _check_heartbeat_deadline(self, now_wall):
        """Expire the link from the last valid heartbeat, whatever arrived since."""
        if self._last_heartbeat_wall is None:
            return
        age = now_wall - self._last_heartbeat_wall
        if age > self.config["link_timeout_s"]:
            self._terminate("interactive_link_lost", heartbeat_age_s=age)
    def poll_request(self, now_wall):
        """Consume one new operator request, at most every 0.1 s of wall time."""
        if now_wall - self._last_poll_wall < 0.1:
            return
        self._last_poll_wall = now_wall
        payload, error = read_json_file(self.request_path)
        if payload is None:
            if error not in (None, "missing"):
                self._write_receipt("request_unreadable", reason=error)
            return
        digest = hashlib.sha256(
            json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
        if digest == self._request_digest:
            return
        self._request_digest = digest
        self._handle_request(payload, now_wall)

    def _handle_request(self, payload, now_wall):
        """Validate and apply one new request payload."""
        if not isinstance(payload, dict):
            self._write_receipt("request_rejected", reason="request_not_an_object")
            return
        if payload.get("version") != PROTOCOL_VERSION:
            self._reject(payload, "unsupported_protocol_version")
            return
        request_run = payload.get("run_id")
        if isinstance(request_run, str) and request_run and request_run != self.run_id:
            self._reject(payload, "run_id_mismatch", request_run_id=request_run)
            return
        seq = payload.get("seq")
        if isinstance(seq, bool) or not isinstance(seq, int) or seq <= 0:
            self._reject(payload, "invalid_sequence")
            return
        if seq <= self._last_seq:
            self._write_receipt(
                "request_rejected",
                seq=seq,
                reason="duplicate_or_stale_sequence",
                last_accepted_sequence=self._last_seq,
            )
            return
        self._last_seq = seq
        try:
            created = _finite(payload.get("created_wall_s"), "created_wall_s")
            ttl = _finite(payload.get("ttl_s", self.config["ttl_s"]), "ttl_s")
        except ValueError:
            self._reject(payload, "invalid_request_time")
            return
        if not 1.0 <= ttl <= 120.0:
            self._reject(payload, "invalid_ttl", ttl_s=ttl)
            return
        age = now_wall - created
        if age > ttl:
            self._reject(payload, "request_expired", age_s=age, ttl_s=ttl)
            return
        if created > now_wall + 2.0:
            self._reject(payload, "request_from_future", created_wall_s=created)
            return
        action = payload.get("action")
        if action not in ALLOWED_ACTIONS:
            self._reject(payload, "unsupported_action", action=repr(action))
            return
        if self._terminal_reason is not None:
            self._reject(payload, "run_terminal", terminal_reason=self._terminal_reason)
            return
        if action == "stop":
            self._clear_for("interactive_stop")
            self._terminal_reason = "interactive_stop"
            self._write_receipt("stop_accepted", seq=seq, reason="interactive_stop")
            return
        if action == "cancel":
            self._clear_for("interactive_cancel")
            self._write_receipt("cancel_accepted", seq=seq, reason="operator_cancel")
            return
        request_pose_source = payload.get("pose_source", self.pose_source)
        if request_pose_source in FORBIDDEN_POSE_SOURCES:
            self._reject(
                payload,
                "ground_truth_waypoint_frame_forbidden",
                pose_source=request_pose_source,
            )
            return
        if request_pose_source != self.pose_source:
            self._reject(
                payload, "pose_source_mismatch", pose_source=request_pose_source
            )
            return
        if action in ("yaw", "maneuver"):
            self._accept_local_action(payload, action, created, ttl)
            return
        try:
            goal = _vec3(payload.get("goal_xyz_m"), "goal_xyz_m")
        except ValueError:
            self._reject(payload, "invalid_goal")
            return
        accepted, reason, planar = self.controller.enqueue(goal)
        if not accepted:
            self._reject(payload, reason, planar_m=planar)
            return
        expires = created + ttl
        self._pending.append(
            {"seq": seq, "goal_xyz_m": goal.tolist(), "expires_wall_s": expires}
        )
        self._write_receipt(
            "waypoint_accepted",
            waypoint_result=True,
            seq=seq,
            action=action,
            goal_xyz_m=goal.tolist(),
            ttl_s=ttl,
            expires_wall_s=expires,
            queued=len(self.controller.queue),
            planar_m=planar,
            pose_source=self.pose_source,
            stop_confirmed=False,
        )

    # ------------------------------------------------------------------ guards
    def observe_native_guards(self, *, tilt_rad=None, native_speed_mps=None):
        """Record native guard values, labelled as guard-only (never a pose)."""
        self._native_guards["tilt_rad"] = (
            None if tilt_rad is None else float(tilt_rad)
        )
        self._native_guards["native_speed_mps"] = (
            None if native_speed_mps is None else float(native_speed_mps)
        )

    def invalidate_measurement(self):
        """Interrupt the follower dwell/stall window and zero the command."""
        self._yaw_window = None
        self.controller.invalidate_measurement()
        self._last_command = [0.0, 0.0, 0.0]
        self._last_consumed_odom_s = None
        self._pose_history.clear()

    def _reset_yaw_window(self, reason, *, source=None, sim=None):
        """Clear the yaw dwell window and record why it restarted.

        Every invalidation path (hold, explicit invalidate, cancel, expire, terminal,
        new action, action finish) funnels through here so a stale window can never
        survive a failed tick or a skipped control period.
        """
        self._yaw_window = None
        self._yaw_window_reset = {
            "reason": str(reason),
            "source": None if source is None else str(source),
            "sim": sim,
        }

    def _hold(
        self,
        reason,
        evidence,
        now_wall,
        *,
        terminal_reason="interactive_pose_lost",
        repeat=False,
    ):
        """Command zero, keep receipts bounded and escalate a terminal hold.

        A terminal reason of None means "expected wait" (for example the initial
        stand gate): it never starts or advances the loss clock, so an expected
        wait can never be mistaken for a lost pose.
        """
        self._reset_yaw_window(reason, source="hold", sim=evidence.get("current_sim_s"))

        self._last_command = [0.0, 0.0, 0.0]
        try:
            now = float(now_wall)
        except (TypeError, ValueError):
            now = None
        if now is not None and self._hold_since_wall is None:
            if terminal_reason is not None:
                self._hold_since_wall = now
            if not repeat:
                self._write_receipt("hold", reason=reason)
        elif (
            terminal_reason is not None
            and now is not None
            and self._hold_since_wall is not None
            and now - self._hold_since_wall > self.config["pose_loss_timeout_s"]
        ):
            self._terminate(terminal_reason, hold_reason=reason)
        if now is not None:
            self._maybe_write_observation(now, evidence, self._last_command)
        return [0.0, 0.0, 0.0], dict(
            evidence,
            status="hold",
            reason=reason,
            command=[0.0, 0.0, 0.0],
            parameters=self.parameters,
            terminal_reason=self._terminal_reason,
        )

    # ------------------------------------------------------------- control tick
    def tick(
        self,
        current_sim_s,
        odom_stamp_s,
        position,
        quat_wxyz,
        estimated_velocity,
        measured_yaw_rate,
        ready,
        now_wall,
        arrival_wall,
        *,
        frame_id=None,
        child_frame_id=None,
        pose_source=None,
        imu_source_stamp_s=None,
        imu_arrival_wall_s=None,
        imu_frame=None,
        imu_valid=None,
    ):
        """Return (body_command3, receipt) for one measured-pose sample."""
        """Return (body_command3, receipt) for one measured-pose sample."""
        evidence = {
            "current_sim_s": None,
            "odom_stamp_s": None,
            "sim_age_s": None,
            "wall_age_s": None,
            "frame_id": frame_id,
            "child_frame_id": child_frame_id,
            "pose_source": self.pose_source,
            "distinct_measurement": False,
        }
        # Every tick starts with no validated current perception. A hold that
        # returns before the perception gate (not_ready, link, pose, time) must not
        # inherit the previous tick's "passed this tick" flag.
        self._perception_current = False
        self._tick_counter += 1
        self._yaw_context = None
        self.poll_request(now_wall)
        self._check_link(now_wall)
        if self._terminal_reason is not None:
            self._last_command = [0.0, 0.0, 0.0]
            self._maybe_write_observation(now_wall, evidence, self._last_command)
            return self._terminal_result(evidence)
        try:
            sim = _finite(current_sim_s, "current_sim_s")
            now = _finite(now_wall, "now_wall")
        except ValueError:
            return self._hold("invalid_evaluation_time", evidence, now_wall)
        evidence["current_sim_s"] = sim
        if self._last_sim_s is not None and sim < self._last_sim_s:
            return self._hold("nonmonotonic_sim_time", evidence, now)
        self._last_sim_s = sim
        if sim >= self.controller.motion_end:
            # The motion budget is global. Yaw and short maneuver actions never
            # reach WaypointQueueController.tick(), so checking only there let a
            # yaw or maneuver keep commanding after the budget had run out. Latch a
            # terminal so the runtime parks instead of resuming on the next request.
            self._terminate(
                "interactive_motion_budget",
                motion_end_s=float(self.controller.motion_end),
                current_sim_s=sim,
            )
            self._last_command = [0.0, 0.0, 0.0]
            self._maybe_write_observation(now, evidence, self._last_command)
            return self._terminal_result(evidence)
        if not isinstance(ready, bool) or not ready:
            held = {"status": "hold", "reason": "not_ready"}
            result = self._hold(
                "not_ready",
                dict(evidence, **held),
                now,
                terminal_reason=None,
                repeat=self._not_ready_latched,
            )
            self._not_ready_latched = True
            return result
        self._not_ready_latched = False
        if not self._link_established:
            # No operator heartbeat has ever been accepted, so no non-zero command
            # may be started. This is a wait, not a loss: it never opens the
            # pose-loss clock, and once a heartbeat is accepted the gate disappears.
            result = self._hold(
                "operator_link_not_established",
                dict(evidence, link_established=False),
                now,
                terminal_reason=None,
                repeat=self._link_gate_reported,
            )
            self._link_gate_reported = True
            return result
        if pose_source is not None and pose_source != self.pose_source:
            evidence["reported_pose_source"] = pose_source
            return self._hold("pose_source_mismatch", evidence, now)
        try:
            stamp = _finite(odom_stamp_s, "odom_stamp_s")
            arrival = _finite(arrival_wall, "arrival_wall")
            current = self._pose_transform(position, quat_wxyz)
        except ValueError:
            return self._hold("invalid_measured_odometry", evidence, now)
        raw_age = sim - stamp
        wall_age = now - arrival
        evidence.update(
            odom_stamp_s=stamp,
            raw_sim_age_s=raw_age,
            sim_age_s=max(0.0, raw_age),
            wall_age_s=wall_age,
        )
        future_tolerance = self._odometry_future_tolerance(sim)
        evidence["odometry_future_tolerance_s"] = future_tolerance
        evidence["odometry_future_tolerance_basis"] = (
            "profile_future_tolerance_s"
            if future_tolerance <= self.config["future_tolerance_s"]
            else "float32_half_ulp_same_tick"
        )
        if raw_age < -future_tolerance:
            return self._hold("future_odometry", evidence, now)
        if raw_age > self.config["max_sim_age_s"]:
            return self._hold("stale_sim_odometry", evidence, now)
        if wall_age < 0 or wall_age > self.config["max_wall_age_s"]:
            return self._hold("stale_wall_odometry", evidence, now)
        if (
            not isinstance(frame_id, str)
            or not frame_id
            or not isinstance(child_frame_id, str)
            or not child_frame_id
        ):
            return self._hold("unlabelled_measurement_frames", evidence, now)
        if self._frame_labels is None:
            self._frame_labels = (frame_id, child_frame_id)
        elif (frame_id, child_frame_id) != self._frame_labels:
            return self._hold("measurement_frame_mismatch", evidence, now)
        if self._last_odom_stamp_s is not None and stamp < self._last_odom_stamp_s:
            # A rewind is reported and held; the baseline keeps the highest accepted
            # stamp, so a later sample that is still early keeps holding instead of
            # driving on a pose the estimator has already withdrawn.
            evidence["last_accepted_odom_stamp_s"] = self._last_odom_stamp_s
            return self._hold("nonmonotonic_odometry_stamp", evidence, now)
        if raw_age < 0.0:
            # The stamp is the SDK float32 sensor clock and the sim time is float64,
            # so the current tick can be up to half a float32 ULP "ahead". The signed
            # age is kept as measured; nothing is rounded or rewritten.
            evidence["same_tick_negative_age_s"] = raw_age
        # Only an accepted, fresh, labelled estimate becomes the baseline; future
        # and stale samples returned above and can neither raise nor lower it.
        self._last_odom_stamp_s = stamp
        position_xyz = current[:3, 3]
        yaw = math.atan2(current[1, 0], current[0, 0])
        if self._last_position is not None:
            self._path_length_m += float(
                np.linalg.norm(position_xyz - self._last_position)
            )
        self._last_position = position_xyz.copy()
        self._pose_samples += 1
        distinct = stamp != self._last_consumed_odom_s
        self._yaw_context = {
            "tick": self._tick_counter,
            "sim": sim,
            "stamp": stamp,
            "raw_age": raw_age,
            "wall_age": wall_age,
            "yaw_rate": (
                None
                if measured_yaw_rate is None
                else float(measured_yaw_rate)
                if math.isfinite(float(measured_yaw_rate))
                else None
            ),
            "distinct": distinct,
            "imu_yaw_rate": (
                None
                if imu_valid is False
                else float(measured_yaw_rate)
                if measured_yaw_rate is not None
                and math.isfinite(float(measured_yaw_rate))
                else None
            ),
            "imu_source_stamp_s": imu_source_stamp_s,
            "imu_arrival_wall_s": imu_arrival_wall_s,
            "imu_frame": imu_frame,
            "imu_valid": imu_valid,
        }
        evidence["distinct_measurement"] = distinct
        if distinct:
            self._pose_history.append((stamp, position_xyz.copy()))
            cutoff = stamp - self.config["pose_speed_window_s"]
            while len(self._pose_history) > 2 and self._pose_history[1][0] <= cutoff:
                self._pose_history.popleft()
        pose_speed = None
        span = 0.0
        if self._pose_history:
            first_stamp, first_position = self._pose_history[0]
            span = stamp - first_stamp
            if span >= self.config["pose_speed_window_s"]:
                pose_speed = float(
                    np.linalg.norm(position_xyz - first_position) / span
                )
        reported_speed = None
        if self.config["twist_available"]:
            try:
                reported_speed = float(
                    np.linalg.norm(_vec3(estimated_velocity, "estimated_velocity"))
                )
            except ValueError:
                return self._hold("invalid_reported_twist", evidence, now)
        speed = reported_speed if self.config["twist_available"] else pose_speed
        stop_check_speed = 0.04 if speed is None else speed
        yaw_rate = None
        if measured_yaw_rate is not None:
            try:
                yaw_rate = _finite(measured_yaw_rate, "measured_yaw_rate")
            except ValueError:
                return self._hold("invalid_measured_yaw_rate", evidence, now)
        evidence.update(
            aligned_position_xyz_m=position_xyz.tolist(),
            aligned_yaw_rad=yaw,
            pose_window_speed_mps=pose_speed,
            pose_window_span_s=span,
            reported_twist_speed_mps=reported_speed,
            estimated_speed_mps=speed,
            measured_yaw_rate_rps=yaw_rate,
            speed_source="twist" if self.config["twist_available"] else "pose_window",
            map_frame=frame_id,
            sensor_frame=child_frame_id,
            alignment="identity_in_live_estimator_map_frame",
        )
        self._last_valid_measurement = dict(evidence)
        self._expire_stale_waypoints(now)
        self._expire_local_action(now)
        if self._terminal_reason is not None:
            return self._terminal_result(evidence)
        perception_failure = self._perception_gate(now, sim)
        if perception_failure is not None:
            reason, fields, terminal = perception_failure
            return self._hold(
                reason, dict(evidence, **fields), now, terminal_reason=terminal
            )
        self._hold_since_wall = None
        if not distinct:
            if self._action is not None:
                # A cached body command may not outlive the perception it was
                # admitted with. Every control tick, not every new odometry sample,
                # re-runs the direction gate with the latest sample and the wall
                # clock, so a rear or side obstacle that appeared since the last pose
                # sample cancels the action instead of being driven through.
                command = self._local_action_command(sim, yaw, now)
                bounds = (self.controller.speed, 0.05, 0.15)
                self._last_command = [
                    max(-bound, min(bound, float(value)))
                    for value, bound in zip(command, bounds)
                ]
                self._maybe_write_observation(now, evidence, self._last_command)
                return list(self._last_command), dict(
                    evidence,
                    status="tracking",
                    reason="fresh_duplicate_odometry_local_action",
                    command=list(self._last_command),
                    direction_gate_reevaluated=True,
                    parameters=self.parameters,
                    terminal_reason=self._terminal_reason,
                    phase=self.phase,
                    stage=self.stage,
                )
            # A cached goto command was admitted with an older perception. Every
            # control tick re-checks the lateral direction gate before replaying it,
            # so a side that became unknown or too close since the last pose sample
            # stops the replay instead of being strafed through.
            cached = list(self._last_command)
            hold = self._admit_goto_command(
                cached, now, evidence, command_source="cached_odometry"
            )
            if hold is not None:
                return hold
            self._maybe_write_observation(now, evidence, cached)
            return list(cached), dict(
                evidence,
                status="tracking",
                reason="fresh_duplicate_odometry",
                command=list(cached),
                direction_gate_reevaluated=True,
                parameters=self.parameters,
                terminal_reason=None,
                phase=self.phase,
                stage=self.stage,
            )
        if self._action is not None:
            command = self._local_action_command(sim, yaw, now)
        else:
            command = self.controller.tick(
                stamp, position_xyz.tolist(), yaw, stop_check_speed, yaw_rate
            )
        hold = self._admit_goto_command(
            command, now, evidence, command_source="fresh_odometry"
        )
        if hold is not None:
            return hold
        self._last_consumed_odom_s = stamp
        self._emit_completed_waypoints()
        if (
            self.controller.drop_reason is not None
            and self.controller.drop_reason != self._seen_drop_reason
        ):
            self._seen_drop_reason = self.controller.drop_reason
            dropped = (
                self._pending.pop(0)
                if self._pending
                else {"seq": None, "goal_xyz_m": None}
            )
            self._pending.clear()
            self._write_receipt(
                "waypoint_dropped",
                waypoint_result=True,
                seq=dropped.get("seq"),
                goal_xyz_m=dropped.get("goal_xyz_m"),
                reason=self.controller.drop_reason,
            )
        if self.controller.failure is not None:
            self._terminate(self.controller.failure)
        if self._terminal_reason is not None:
            command = [0.0, 0.0, 0.0]
        self._last_command = self._bounded_command(command)
        self._maybe_write_observation(now, evidence, self._last_command)
        return list(self._last_command), dict(
            evidence,
            status="terminal" if self._terminal_reason is not None else "tracking",
            reason=self._terminal_reason or "fresh_measured_odometry",
            command=list(self._last_command),
            parameters=self.parameters,
            terminal_reason=self._terminal_reason,
            phase=self.phase,
            stage=self.stage,
            active_seq=self._active_seq(),
            queued=len(self.controller.queue),
            distance_to_goal_m=self.controller.distance_to_goal_m,
            z_error_m=self.controller.z_error_m,
            remaining_m=self.controller.remaining_m,
            cross_track_m=self.controller.cross_track_m,
        )

    def _odometry_future_tolerance(self, sim):
        """Half a float32 ULP at the sample sim clock, capped at half a physical tick.

        Units: seconds. The observer publishes its source stamp as a float32 clock
        while the capture hands the tracker a float64 sim time, so a stamp for the
        current physical tick can sit up to half a float32 ULP ahead of it.

        The tick that matters is the sensor/physics tick, not the 50 Hz policy or the
        20 Hz depth frame: IMU and physics run at 200 Hz (ODOMETRY_PHYSICAL_TICK_S =
        0.005 s). The bound max(profile tolerance, half_ulp + 1e-7) stays below half of
        that tick (2.5e-3 s) for sim horizons below 2**16 s, so two different physical
        ticks can never be merged. Above that horizon the cap keeps the tolerance at
        half a tick; a stamp that would need more is then rejected (fail closed)
        instead of being accepted as the same tick. The source stamp is never rounded
        or rewritten and the signed age stays in the receipt.
        """
        half_ulp = float32_time_half_ulp(sim)
        uncapped = max(self.config["future_tolerance_s"], half_ulp + 1e-7)
        return min(uncapped, ODOMETRY_PHYSICAL_TICK_S / 2.0)


    def _admit_goto_command(self, command, now, evidence, *, command_source):
        """Gate a goto command that was just produced or replayed from cache.

        A lateral component is real motion towards one side, so it needs an
        observation of that side whichever branch produced it. The follower path
        and the cached duplicate-odometry path both call this, so a cached command
        cannot carry a side that is now unknown or too close past the gate.
        """
        if self._action is not None or abs(float(command[1])) <= 0.0:
            return None
        lateral_reason = self._direction_reason(
            {"body_forward_mps": 0.0, "body_lateral_mps": float(command[1])},
            now,
        )
        if lateral_reason is None:
            return None
        return self._hold(
            "goto_lateral_uncovered",
            dict(
                evidence,
                command=list(command),
                command_source=command_source,
                lateral_direction_reason=lateral_reason,
                cross_track_m=self.controller.cross_track_m,
            ),
            now,
            terminal_reason=None,
            repeat=False,
        )

    def _bounded_command(self, command):
        """Clamp one body command to the interactive envelope.

        The duplicate-odometry and the distinct-odometry paths both admit a body
        command, so they must share one envelope; a second copy of the bounds could
        drift away from LOCAL_ACTION_BOUNDS.
        """
        bounds = (
            self.controller.speed,
            LOCAL_ACTION_BOUNDS["lateral_mps"],
            LOCAL_ACTION_BOUNDS["yaw_rate_rps"],
        )
        return [
            max(-bound, min(bound, float(value)))
            for value, bound in zip(command, bounds)
        ]

    def _terminal_result(self, evidence):
        """Receipt for the already latched terminal state, always zero command."""
        return [0.0, 0.0, 0.0], dict(
            evidence,
            status="terminal",
            reason=self._terminal_reason,
            command=[0.0, 0.0, 0.0],
            parameters=self.parameters,
            terminal_reason=self._terminal_reason,
        )

    @staticmethod
    def _pose_transform(position, quat_wxyz):
        """Validate measured XYZ and WXYZ quaternion; return its SE(3)."""
        return pose_transform(position, quat_wxyz)

    def _active_seq(self):
        """Sequence number of the waypoint currently being tracked."""
        if not self._pending:
            return None
        return self._pending[0]["seq"]

    def _expire_stale_waypoints(self, now_wall):
        """Drop the plan when its head waypoint TTL elapsed before capture."""
        if not self._pending:
            return
        if self._pending[0]["expires_wall_s"] >= now_wall:
            return
        expired = self._pending[0]
        self._clear_for("waypoint_expired")
        self._write_receipt(
            "waypoint_expired",
            waypoint_result=True,
            seq=expired["seq"],
            goal_xyz_m=expired["goal_xyz_m"],
            expires_wall_s=expired["expires_wall_s"],
            late_by_s=now_wall - expired["expires_wall_s"],
        )

    def _emit_completed_waypoints(self):
        """Mirror follower completions into receipts with their sequence numbers."""
        while self._seen_completed_goals < self.controller.completed_goals:
            if self._pending:
                completed = self._pending.pop(0)
                seq = completed["seq"]
                goal = completed["goal_xyz_m"]
            else:
                seq, goal = None, None
            self._seen_completed_goals += 1
            self._write_receipt(
                "waypoint_reached",
                waypoint_result=True,
                seq=seq,
                goal_xyz_m=goal,
                stage=self.controller.stage,
                queued=len(self.controller.queue),
                stop_confirmed=False,
                note="capture-bounds arrival; the loop parks only on a terminal request",
            )

    def _validated_perception_telemetry(self, sim, now):
        """Describe the last perception sample that passed every gate.

        The sample keeps its own arrival time; freshness is always recomputed from
        that arrival, never from the moment the gate happened to pass it, so a
        sample cannot be kept alive by being re-validated later. current/fresh are
        false when the gate failed in this tick, when the sample is stale on either
        clock, or when the tracker is terminal: a historical sample stays visible as
        last_validated but is never presented as the current input.
        """
        stored = self._last_validated_perception
        if stored is None:
            return None
        entry = {
            "source": stored["source"],
            "frame_id": stored["frame_id"],
            "stamp_s": stored["stamp_s"],
            "arrival_wall_s": stored.get("arrival_wall_s"),
            "clearance_m": stored["clearance_m"],
            "points": stored["points"],
            "cloud_points": stored["cloud_points"],
            "observation_state": stored["observation_state"],
            "depth_valid_ratio": stored["depth_valid_ratio"],
            "directions": stored["directions"],
            "validated_sim_s": stored["validated_sim_s"],
            "validated_wall_s": stored["validated_wall_s"],
        }
        sim_age = (
            None
            if sim is None or stored["stamp_s"] is None
            else float(sim) - float(stored["stamp_s"])
        )
        arrival = stored.get("arrival_wall_s")
        wall_age = None if now is None or arrival is None else float(now) - float(arrival)
        since_validated = (
            None
            if now is None
            else float(now) - float(stored["validated_wall_s"])
        )
        current = bool(
            self._perception_current
            and self._terminal_reason is None
            and sim_age is not None
            and -self.config["future_tolerance_s"] <= sim_age <= self.config["perception_max_sim_age_s"]
            and wall_age is not None
            and 0.0 <= wall_age <= self.config["perception_max_wall_age_s"]
        )
        entry["sim_age_s"] = sim_age
        entry["sample_wall_age_s"] = wall_age
        entry["since_validated_wall_s"] = since_validated
        entry["gate_current"] = bool(self._perception_current)
        entry["current"] = current
        entry["fresh"] = current
        return entry


    def _maybe_write_observation(self, now_wall, evidence, command):
        """Publish the observation bundle at most every 0.5 s of wall time."""
        try:
            now = float(now_wall)
        except (TypeError, ValueError):
            return
        if now - self._last_observation_wall < 0.5:
            return
        self._last_observation_wall = now
        heartbeat_age = (
            None
            if self._last_heartbeat_wall is None
            else now - self._last_heartbeat_wall
        )
        sim_age = evidence.get("sim_age_s")
        wall_age = evidence.get("wall_age_s")
        observation = {
            "protocol_version": PROTOCOL_VERSION,
            "run_id": self.run_id,
            "written_wall_s": now,
            "sim_s": evidence.get("current_sim_s"),
            "pose": {
                "source": self.pose_source,
                "map_frame": evidence.get("map_frame"),
                "sensor_frame": evidence.get("sensor_frame"),
                "stamp_s": evidence.get("odom_stamp_s"),
                "position_xyz_m": evidence.get("aligned_position_xyz_m"),
                "yaw_rad": evidence.get("aligned_yaw_rad"),
                "sim_age_s": sim_age,
                "wall_age_s": wall_age,
                "fresh": bool(
                    sim_age is not None
                    and sim_age <= self.config["max_sim_age_s"]
                    and wall_age is not None
                    and 0 <= wall_age <= self.config["max_wall_age_s"]
                ),
                "alignment": "identity_in_live_estimator_map_frame",
                "global_localization_claim": False,
            },
            "speed": {
                "pose_window_mps": evidence.get("pose_window_speed_mps"),
                "reported_twist_mps": evidence.get("reported_twist_speed_mps"),
                "measured_yaw_rate_rps": evidence.get("measured_yaw_rate_rps"),
                "command_ceiling_mps": self.controller.speed,
                "achieved_speed_note": (
                    "measured here; never inferred from the command ceiling"
                ),
            },
            "waypoints": {
                "forward_turn_mode": self.config["forward_turn_mode"],
                "active_seq": self._active_seq(),
                "active_goal_xyz_m": (
                    self._pending[0]["goal_xyz_m"] if self._pending else None
                ),
                "queued": len(self.controller.queue),
                "phase": self.phase,
                "failure": self.controller.failure,
                "terminal_reason": self._terminal_reason,
                "distance_to_goal_m": self.controller.distance_to_goal_m,
                "remaining_m": self.controller.remaining_m,
                "z_error_m": self.controller.z_error_m,
                "cross_track_m": self.controller.cross_track_m,
                "completed_goals": self.controller.completed_goals,
                "last_result": self._last_waypoint_result,
            },
            "yaw_completion": {
                "mode": self.config["yaw_completion_mode"],
                "tolerance_rad": self.config["yaw_completion_tolerance_rad"],
                "control_tolerance_rad": self.config["yaw_control_tolerance_rad"],
                "completion_tolerance_rad": self.config["yaw_completion_tolerance_rad"],
                "settle_rate_radps": self.config["yaw_settle_rate_radps"],
                "dwell_sim_s": self.config["yaw_dwell_sim_s"],
                "eval_max_gap_sim_s": self.config["yaw_eval_max_gap_sim_s"],
                "imu_max_sim_age_s": self.config["imu_max_sim_age_s"],
                "imu_max_wall_age_s": self.config["imu_max_wall_age_s"],
            },
            "action": {
                "active": None
                if self._action is None
                else {
                    "kind": self._action["kind"],
                    "seq": self._action["seq"],
                    "parameters": dict(self._action["parameters"]),
                    "target_yaw_rad": self._action["target_yaw_rad"],
                    "since_sim_s": self._action["since_sim_s"],
                    "completion": {
                        "mode": self.config["yaw_completion_mode"],
                        "tolerance_rad": self.config["yaw_completion_tolerance_rad"],
                        "control_tolerance_rad": self.config["yaw_control_tolerance_rad"],
                        "completion_tolerance_rad": self.config["yaw_completion_tolerance_rad"],
                        "settle_rate_radps": self.config["yaw_settle_rate_radps"],
                        "dwell_sim_s": self.config["yaw_dwell_sim_s"],
                        "eval_max_gap_sim_s": self.config["yaw_eval_max_gap_sim_s"],
                        "imu_max_sim_age_s": self.config["imu_max_sim_age_s"],
                        "imu_max_wall_age_s": self.config["imu_max_wall_age_s"],
                    },
                    "yaw_window": None
                    if self._yaw_window is None
                    else {
                        "start_sim_s": self._yaw_window["start_sim_s"],
                        "valid_sim_s": self._yaw_window["valid_sim_s"],
                        "distinct_since_start": self._yaw_window["distinct_since_start"],
                        "reset": self._yaw_window_reset,
                    },
                },
                "completed": self._action_completed,
                "supported": ["goto", "waypoint", "yaw", "maneuver", "cancel", "stop"],
                "bounds": dict(LOCAL_ACTION_BOUNDS),
            },
            "perception": {
                "required": self.config["require_perception"],
                "limits": {
                    "max_wall_age_s": self.config["perception_max_wall_age_s"],
                    "max_sim_age_s": self.config["perception_max_sim_age_s"],
                    "min_clearance_m": self.config["min_clearance_m"],
                },
                "last": (
                    None if self._perception is None else dict(self._perception)
                ),
                "validated": self._validated_perception_telemetry(
                    evidence.get("current_sim_s"), now
                ),
                "holds_without_fresh_static_perception": True,
            },
            "command": {
                "last_request": list(command),
                "bounds": {
                    "forward_mps": self.controller.speed,
                    "lateral_mps": 0.05,
                    "yaw_rate_rps": 0.15,
                },
                "source": "deterministic_waypoint_follower",
            },
            "link": {
                "heartbeat_age_s": heartbeat_age,
                "established": self._link_established,
                "request_digest_seen": self._request_digest is not None,
                "last_accepted_sequence": self._last_seq,
                "io_errors": self._io_errors,
            },
            "guards": {
                "terminal_reason": self._terminal_reason,
                "native": dict(self._native_guards),
            },
            "coverage": {
                "pose_samples": self._pose_samples,
                "path_length_m": self._path_length_m,
                "note": (
                    "measured-pose path length; operator side computes "
                    "visited-area coverage"
                ),
            },
            "map": {
                "provider": "operator_side_from_live_measured_topics",
                "simulator_map_injected": False,
                "topics": [
                    "/dynanav/depth",
                    "/dynanav/camera",
                    "/dynanav/camera_info",
                    "/dynanav/lidar_filtered",
                    "/clock",
                    "measured odometry topic (experiment.odometry_topic)",
                ],
                "note": (
                    "no ground-truth geometry and no oracle map are part of "
                    "this bundle"
                ),
            },
        }
        try:
            atomic_write_json(self.observation_path, observation)
            self._observation_writes += 1
        except OSError:
            self._io_errors += 1


def create_interactive_tracker(experiment, *, speed_mps=None, mailbox=None):
    """Construct the interactive waypoint tracker used by the control loop."""
    return InteractiveWaypointTracker(experiment, speed_mps=speed_mps, mailbox=mailbox)
