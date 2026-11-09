"""Measured estimator route replay for an explicit repeat-spawn lab run.

Only current estimated sensor odometry enters this adapter. It aligns the first
ready measured sensor pose with a previously measured anchor and delegates the
bounded 3-D waypoint control to RouteTracker. This SE(3) gauge alignment assumes
the robot repeats the surveyed spawn; it does not perform map registration or
general global localization. No simulator, native pose or archived transform is
accepted as a navigation input.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections import deque
from copy import deepcopy
from numbers import Real
from pathlib import Path

import numpy as np

from embodied_navigation.navigation.robogauge.route_tracker import RouteTracker

POSE_SOURCE = "lio_estimate"
POSE_SOURCES = (POSE_SOURCE, "livo_post_vio")
REFERENCE_VERSION = 1


def _number(value, name, *, positive=False, nonnegative=False):
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if (positive and value <= 0) or (nonnegative and value < 0):
        raise ValueError(f"{name} has an invalid sign")
    return value


def _vector(value, size, name):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must contain {size} finite numbers") from error
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result.copy()


def pose_transform(position, quat_wxyz):
    """Validate measured XYZ and explicit WXYZ quaternion; return its SE(3)."""
    xyz = _vector(position, 3, "position")
    q = _vector(quat_wxyz, 4, "quaternion_wxyz")
    norm = float(np.linalg.norm(q))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError("quaternion_wxyz must be unit length within 1e-3")
    w, x, y, z = q / norm
    transform = np.eye(4)
    transform[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    transform[:3, 3] = xyz
    return transform


def _reference_pose(pose, name):
    if not isinstance(pose, dict):
        raise ValueError(f"{name} must be a measured pose object")  # noqa: TRY004 - reference validation uses one ValueError contract.
    stamp = pose.get("stamp_ns")
    if type(stamp) is not int or stamp < 0:
        raise ValueError(f"{name}.stamp_ns must be a nonnegative integer")
    transform = pose_transform(pose.get("position_xyz_m"), pose.get("quaternion_wxyz"))
    available = pose.get("twist_available", True)
    if type(available) is not bool:
        raise ValueError(f"{name}.twist_available must be boolean")
    if available:
        _vector(
            pose.get("estimated_velocity_xyz_mps"),
            3,
            f"{name}.estimated_velocity_xyz_mps",
        )
        _number(pose.get("estimated_yaw_rate_rps"), f"{name}.estimated_yaw_rate_rps")
    elif (
        pose.get("estimated_velocity_xyz_mps") is not None
        or pose.get("estimated_yaw_rate_rps") is not None
    ):
        raise ValueError(
            f"{name} unavailable twist must be null, never placeholder measurements"
        )
    return transform


def _pose_window_stop(goal):
    """Validate the builder's pose-only stop witness without loading its source."""
    stop = goal.get("stop_evidence")
    if (
        not isinstance(stop, dict)
        or stop.get("method") != "measured_pose_window"
        or stop.get("twist_used_to_confirm_stop") is not False
    ):
        raise ValueError(
            "pose-only goals require measured_pose_window stop evidence without twist"
        )
    start, end, count = (
        stop.get(key) for key in ("start_stamp_ns", "end_stamp_ns", "sample_count")
    )
    if (
        type(start) is not int
        or type(end) is not int
        or not 0 <= start < end
        or end != goal["stamp_ns"]
        or type(count) is not int
        or count < 2
    ):
        raise ValueError(
            "pose-only stop witness must span distinct source samples ending at goal"
        )
    parameters = stop.get("parameters")
    if not isinstance(parameters, dict):
        raise ValueError("pose-only stop witness parameters are required")  # noqa: TRY004 - reference validation retains its ValueError contract.
    duration = _number(stop.get("duration_s"), "stop duration", positive=True)
    dwell = _number(parameters.get("dwell_s"), "stop dwell", positive=True)
    if duration < dwell or not math.isclose(
        duration, (end - start) * 1e-9, rel_tol=0, abs_tol=1e-9
    ):
        raise ValueError(
            "pose-only stop witness has insufficient or inconsistent measured duration"
        )
    for observed, limit in (
        ("maximum_sample_gap_s", "max_sample_gap_s"),
        ("maximum_distance_from_endpoint_m", "position_range_m"),
        ("maximum_yaw_difference_from_endpoint_rad", "yaw_range_rad"),
    ):
        value = _number(stop.get(observed), observed, nonnegative=True)
        bound = _number(parameters.get(limit), limit, positive=True)
        if value > bound:
            raise ValueError(f"pose-only stop witness exceeds {limit}")


def validate_slam_route_reference(reference):
    """Validate route geometry and measured-source correspondence without any I/O."""
    if not isinstance(reference, dict):
        raise ValueError("SLAM reference must be an object")  # noqa: TRY004 - reference validation uses one ValueError contract.
    if (
        type(reference.get("version")) is not int
        or reference.get("version") != REFERENCE_VERSION
        or reference.get("pose_source") not in POSE_SOURCES
    ):
        raise ValueError(
            "reference must identify version 1 measured LIO or post-VIO estimates"
        )
    if (
        reference.get("repeat_spawn_assumption") is not True
        or reference.get("map_registration_enabled") is not False
    ):
        raise ValueError(
            "reference must explicitly declare the repeat-spawn lab assumption and disabled map registration"
        )
    if reference.get("no_gt_pose_used") is not True:
        raise ValueError("reference must declare no GT pose used")
    source = reference.get("source")
    if not isinstance(source, dict):
        raise ValueError("reference source metadata is required")  # noqa: TRY004 - reference validation uses one ValueError contract.
    for field in ("bag", "topic", "map_frame", "sensor_frame"):
        if not isinstance(source.get(field), str) or not source[field]:
            raise ValueError(f"reference source {field} is required")
    pose_only = source.get("pose_only", False)
    twist_available = source.get("twist_available", True)
    if (
        type(pose_only) is not bool
        or type(twist_available) is not bool
        or twist_available == pose_only
    ):
        raise ValueError(
            "source pose_only and twist_available must be opposite booleans"
        )
    if reference["pose_source"] == "livo_post_vio" and (
        source.get("pose_stage") != "post_vio" or not pose_only
    ):
        raise ValueError(
            "post-VIO references require explicit post_vio stage and pose-only evidence"
        )
    files = source.get("files")
    if not isinstance(files, list) or not files:
        raise ValueError("reference source file hashes are required")
    for file in files:
        if (
            not isinstance(file, dict)
            or not isinstance(file.get("path"), str)
            or not file["path"]
        ):
            raise ValueError("reference source file path is required")
        digest = file.get("sha256")
        if (
            not isinstance(digest, str)
            or len(digest) != 64
            or any(c not in "0123456789abcdef" for c in digest)
        ):
            raise ValueError("reference source SHA256 must be lowercase hex")
    first_stamp = source.get("first_stamp_ns")
    last_stamp = source.get("last_stamp_ns")
    if (first_stamp is not None or last_stamp is not None) and (
        type(first_stamp) is not int
        or type(last_stamp) is not int
        or not 0 <= first_stamp < last_stamp
    ):
        raise ValueError("reference source stamp extent is invalid")
    count = source.get("message_count")
    if type(count) is not int or count < 2:
        raise ValueError("reference must contain at least two measured source messages")
    anchor = _reference_pose(reference.get("anchor"), "anchor")
    if first_stamp is not None and first_stamp != reference["anchor"]["stamp_ns"]:
        raise ValueError(
            "reference anchor does not correspond to first measured source stamp"
        )
    goals = reference.get("goals")
    if not isinstance(goals, list) or not goals:
        raise ValueError("reference must contain a measured goal")
    source_paths = {file["path"] for file in files}
    for pose in [reference["anchor"], *goals]:
        if pose.get("twist_available", True) != twist_available:
            raise ValueError("reference pose/source twist availability mismatch")
        message = pose.get("source_message")
        if message is not None and (
            not isinstance(message, dict)
            or message.get("storage") not in source_paths
            or type(message.get("message_id")) is not int
            or message["message_id"] <= 0
            or type(message.get("bag_timestamp_ns")) is not int
            or message["bag_timestamp_ns"] < 0
        ):
            raise ValueError("reference pose/source message correspondence is invalid")
    previous = reference["anchor"]["stamp_ns"]
    origin = anchor[:3, 3]
    for index, goal in enumerate(goals):
        transform = _reference_pose(goal, f"goals[{index}]")
        if last_stamp is not None and goal["stamp_ns"] > last_stamp:
            raise ValueError("reference goal lies outside measured source stamp extent")
        if goal["stamp_ns"] <= previous:
            raise ValueError(
                "reference goal stamps must follow the anchor in strict source order"
            )
        point = transform[:3, 3]
        if float(np.linalg.norm(point[:2] - origin[:2])) <= 1e-6:
            raise ValueError(
                "reference segments require nonzero measured XY displacement"
            )
        previous, origin = goal["stamp_ns"], point
        if pose_only:
            _pose_window_stop(goal)
    # If a builder records endpoint stamps, require them to agree with the poses.
    # Geometry is always taken from the measured pose fields, never a second route.
    correspondence = reference.get("source_correspondence")
    if correspondence is not None:
        if (
            not isinstance(correspondence, dict)
            or correspondence.get("anchor_stamp_ns") != reference["anchor"]["stamp_ns"]
        ):
            raise ValueError("reference anchor/source correspondence mismatch")
        if correspondence.get("goal_stamps_ns") != [goal["stamp_ns"] for goal in goals]:
            raise ValueError("reference goal/source correspondence mismatch")
    return deepcopy(reference)


def load_slam_route_reference(path):
    """Read one measured reference JSON and record the exact reference file hash."""
    path = Path(path).resolve()
    content = path.read_bytes()
    reference = validate_slam_route_reference(json.loads(content))
    reference["reference_file"] = str(path)
    reference["reference_sha256"] = hashlib.sha256(content).hexdigest()
    return reference


class SlamRouteTracker:
    """Fail-closed odometry adapter around the existing measured 3-D tracker.

    The caller must explicitly opt in to repeat-spawn alignment. ``ready`` means
    the caller has finished the stand and admitted sensor control; it is the
    only condition under which the first measured anchor can be latched.
    Duplicate odometry may retain a fresh bounded command but cannot advance
    dwell or stall evidence. Invalid/stale/not-ready intervals clear that
    evidence and the command. The caller retains policy proprioception and all
    simulator scoring outside this adapter.
    """

    def __init__(
        self,
        reference,
        speed_mps=0.15,
        motion_end_s=230.0,
        *,
        repeat_spawn_assumption,
        map_registration_enabled,
        max_sim_age_s=0.5,
        max_wall_age_s=0.5,
        future_tolerance_s=2e-6,
        stop_on_stall=True,
        speed_source="twist",
        pose_speed_window_s=0.5,
        capture_remaining_m=0.20,
        capture_distance_m=0.30,
        final_capture_remaining_m=None,
        final_capture_distance_m=None,
    ):
        self.reference = validate_slam_route_reference(reference)
        if repeat_spawn_assumption is not True or map_registration_enabled is not False:
            raise ValueError(
                "SLAM route requires explicit repeat_spawn_assumption=True and map_registration_enabled=False"
            )
        speed = _number(speed_mps, "speed_mps", positive=True)
        end = _number(motion_end_s, "motion_end_s", positive=True)
        sim_age = _number(max_sim_age_s, "max_sim_age_s", positive=True)
        wall_age = _number(max_wall_age_s, "max_wall_age_s", positive=True)
        tolerance = _number(future_tolerance_s, "future_tolerance_s", nonnegative=True)
        if tolerance >= sim_age:
            raise ValueError("future_tolerance_s must be smaller than max_sim_age_s")
        window = _number(pose_speed_window_s, "pose_speed_window_s", positive=True)
        if speed_source not in ("twist", "pose_window"):
            raise ValueError("speed_source must be twist or pose_window")
        self.twist_available = self.reference["source"].get("twist_available", True)
        if not self.twist_available and speed_source != "pose_window":
            raise ValueError("pose-only references require speed_source=pose_window")
        capture_remaining = _number(
            capture_remaining_m, "capture_remaining_m", positive=True
        )
        capture_distance = _number(
            capture_distance_m, "capture_distance_m", positive=True
        )
        if not capture_remaining <= capture_distance <= 0.40:
            raise ValueError("capture bounds require remaining <= distance <= 0.40 m")
        final_remaining = (
            None
            if final_capture_remaining_m is None
            else _number(
                final_capture_remaining_m, "final_capture_remaining_m", positive=True
            )
        )
        final_distance = (
            None
            if final_capture_distance_m is None
            else _number(
                final_capture_distance_m, "final_capture_distance_m", positive=True
            )
        )
        effective_final_remaining = (
            capture_remaining if final_remaining is None else final_remaining
        )
        effective_final_distance = (
            capture_distance if final_distance is None else final_distance
        )
        if not effective_final_remaining <= effective_final_distance <= 0.40:
            raise ValueError(
                "final capture bounds require remaining <= distance <= 0.40 m"
            )
        if type(stop_on_stall) is not bool:
            raise ValueError("stop_on_stall must be boolean")
        self.parameters = {
            "speed_mps": speed,
            "motion_end_s": end,
            "max_sim_age_s": sim_age,
            "max_wall_age_s": wall_age,
            "future_tolerance_s": tolerance,
            "stop_on_stall": stop_on_stall,
            "repeat_spawn_assumption": True,
            "map_registration_enabled": False,
            "alignment_method": "prior_first_measured_sensor_pose_times_inverse_current_first_ready_measured_sensor_pose",
            "controller": "embodied_navigation.navigation.robogauge.route_tracker.RouteTracker",
            "command_bounds": {
                "forward_mps": speed,
                "lateral_mps": 0.05,
                "yaw_rate_rps": 0.15,
            },
            "controller_initial_wait_s": 7.0,
            "speed_source": speed_source,
            "pose_source": self.reference["pose_source"],
            "twist_available": self.twist_available,
            "capture_remaining_m": capture_remaining,
            "capture_distance_m": capture_distance,
            "final_capture_remaining_m": final_remaining,
            "final_capture_distance_m": final_distance,
            "effective_final_capture_remaining_m": effective_final_remaining,
            "effective_final_capture_distance_m": effective_final_distance,
            "pose_speed_window_s": window,
            "pose_speed_method": "measured_window_endpoint_displacement_over_source_time",
            "pose_speed_limitations": "net_displacement_can_cancel_oscillatory_motion",
        }
        points = [goal["position_xyz_m"][:] for goal in self.reference["goals"]]
        self.tracker = RouteTracker(
            points,
            self.reference["anchor"]["position_xyz_m"][:2],
            speed=speed,
            motion_end=end,
            stop_on_stall=stop_on_stall,
            capture_remaining_m=capture_remaining,
            capture_distance_m=capture_distance,
            final_capture_remaining_m=final_remaining,
            final_capture_distance_m=final_distance,
        )
        self._alignment = None
        self._anchor_receipt = None
        self._last_sim_s = None
        self._last_wall_s = None
        self._last_odom_s = None
        self._last_consumed_odom_s = None
        self._last_command = [0.0, 0.0, 0.0]
        self._pose_history = deque()
        self._last_valid_measurement = None

    @property
    def stage(self):
        return self.tracker.stage

    @property
    def phase(self):
        return self.tracker.phase

    @property
    def failure(self):
        return self.tracker.failure

    @property
    def arrived(self):
        return self.tracker.arrived

    @property
    def events(self):
        return self.tracker.events

    def invalidate_measurement(self):
        """Interrupt continuous measured dwell/stall evidence without reanchoring."""
        self.tracker.invalidate_measurement()
        self._last_command = [0.0, 0.0, 0.0]
        self._last_consumed_odom_s = None
        self._pose_history.clear()

    def _finish(self, command, status, reason, evidence):
        receipt = dict(
            evidence,
            status=status,
            reason=reason,
            pose_source=self.reference["pose_source"],
            stage=self.stage,
            phase=self.phase,
            failure=self.failure,
            arrived=self.arrived,
            command=[float(value) for value in command],
            parameters=deepcopy(self.parameters),
            repeat_spawn_assumption=True,
            map_registration_enabled=False,
            general_global_localization=False,
            no_gt_pose_used=True,
            alignment=None if self._alignment is None else self._alignment.tolist(),
            alignment_anchor=deepcopy(self._anchor_receipt),
            reference_sha256=self.reference.get("reference_sha256"),
            reference_file=self.reference.get("reference_file"),
            last_valid_measurement=deepcopy(self._last_valid_measurement),
            goal_stamp_ns=self.reference["goals"][self.stage]["stamp_ns"],
            goal_position_xyz_m=self.reference["goals"][self.stage]["position_xyz_m"][
                :
            ],
        )
        return receipt["command"], receipt

    def _invalid(self, reason, evidence):
        self.invalidate_measurement()
        return self._finish([0.0, 0.0, 0.0], "hold", reason, evidence)

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
    ):
        """Return ``(body_command3, receipt)`` from current estimated sensor data.

        ``estimated_velocity`` is an XYZ estimated velocity whose Euclidean norm
        supplies the stop check. Its original coordinate basis does not affect
        that norm. ``measured_yaw_rate`` comes from measured IMU or estimated
        odometry, never commanded yaw rate. Frame labels must match the measured
        reference. Times are seconds; wall times share a monotonic clock.
        """
        evidence = {
            "current_sim_s": None,
            "odom_stamp_s": None,
            "sim_age_s": None,
            "raw_sim_age_s": None,
            "wall_age_s": None,
            "frame_id": frame_id,
            "child_frame_id": child_frame_id,
            "distinct_measurement": False,
        }
        try:
            sim = _number(current_sim_s, "current_sim_s", nonnegative=True)
            now = _number(now_wall, "now_wall", nonnegative=True)
        except ValueError:
            return self._invalid("invalid_evaluation_time", evidence)
        evidence["current_sim_s"] = sim
        if self._last_sim_s is not None and sim < self._last_sim_s:
            return self._invalid("nonmonotonic_sim_time", evidence)
        if self._last_wall_s is not None and now < self._last_wall_s:
            return self._invalid("nonmonotonic_wall_time", evidence)
        self._last_sim_s, self._last_wall_s = sim, now
        if self.failure or self.arrived:
            return self._finish(
                [0.0, 0.0, 0.0], "terminal", self.failure or "arrived", evidence
            )
        if sim >= self.parameters["motion_end_s"]:
            self.tracker.failure = "route_motion_budget"
            return self._invalid("route_motion_budget", evidence)
        if type(ready) is not bool or not ready:
            return self._invalid("not_ready", evidence)
        try:
            stamp = _number(odom_stamp_s, "odom_stamp_s", nonnegative=True)
            arrival = _number(arrival_wall, "arrival_wall", nonnegative=True)
        except ValueError:
            return self._invalid("missing_or_invalid_odometry_time", evidence)
        raw_age, wall_age = sim - stamp, now - arrival
        evidence.update(
            odom_stamp_s=stamp,
            raw_sim_age_s=raw_age,
            sim_age_s=max(0.0, raw_age),
            wall_age_s=wall_age,
        )
        if raw_age < -self.parameters["future_tolerance_s"]:
            return self._invalid("future_odometry", evidence)
        if raw_age > self.parameters["max_sim_age_s"]:
            return self._invalid("stale_sim_odometry", evidence)
        if wall_age < 0:
            return self._invalid("future_wall_arrival", evidence)
        if wall_age > self.parameters["max_wall_age_s"]:
            return self._invalid("stale_wall_odometry", evidence)
        if (
            frame_id != self.reference["source"]["map_frame"]
            or child_frame_id != self.reference["source"]["sensor_frame"]
        ):
            return self._invalid("frame_mismatch", evidence)
        if self._last_odom_s is not None and stamp < self._last_odom_s:
            return self._invalid("nonmonotonic_odometry_stamp", evidence)
        try:
            current = pose_transform(position, quat_wxyz)
            # Pose-only native/derived contracts explicitly mark twist unavailable.
            # Do not even coerce those fields: NaNs/zeros are not measurements.
            velocity = (
                _vector(estimated_velocity, 3, "estimated_velocity")
                if self.twist_available
                else None
            )
            omega = _number(measured_yaw_rate, "measured_yaw_rate")
        except ValueError:
            return self._invalid("invalid_measured_odometry", evidence)
        reported_speed = (
            float(np.linalg.norm(velocity)) if velocity is not None else None
        )
        if reported_speed is not None and not math.isfinite(reported_speed):
            return self._invalid("invalid_measured_odometry", evidence)
        if self._alignment is None:
            anchor = pose_transform(
                self.reference["anchor"]["position_xyz_m"],
                self.reference["anchor"]["quaternion_wxyz"],
            )
            self._alignment = anchor @ np.linalg.inv(current)
            self._anchor_receipt = {
                "current_odom_stamp_s": stamp,
                "current_sim_s": sim,
                "current_position_xyz_m": current[:3, 3].tolist(),
                "current_quaternion_wxyz": _vector(
                    quat_wxyz, 4, "quaternion_wxyz"
                ).tolist(),
                "prior_anchor_stamp_ns": self.reference["anchor"]["stamp_ns"],
            }
        aligned = self._alignment @ current
        if not np.isfinite(aligned).all():
            return self._invalid("invalid_aligned_odometry", evidence)
        p = aligned[:3, 3].tolist()
        yaw = math.atan2(aligned[1, 0], aligned[0, 0])
        evidence.update(
            aligned_position_xyz_m=p,
            aligned_yaw_rad=yaw,
            reported_twist_speed_mps=reported_speed,
            twist_available=self.twist_available,
            measured_yaw_rate_rps=omega,
            speed_source=self.parameters["speed_source"],
        )
        previous = self._last_odom_s
        self._last_odom_s = stamp
        distinct = stamp != self._last_consumed_odom_s
        evidence["distinct_measurement"] = distinct
        if previous is not None and stamp - previous > self.parameters["max_sim_age_s"]:
            self.tracker.invalidate_measurement()
            self._pose_history.clear()
            evidence["measurement_gap_reset"] = True
        if distinct:
            self._pose_history.append((stamp, current[:3, 3].copy()))
            cutoff = stamp - self.parameters["pose_speed_window_s"]
            # Retain the nearest preceding endpoint and all following samples.
            while len(self._pose_history) > 2 and self._pose_history[1][0] <= cutoff:
                self._pose_history.popleft()
        pose_speed, span = None, 0.0
        if self._pose_history:
            first_stamp, first_position = self._pose_history[0]
            span = stamp - first_stamp
            if span >= self.parameters["pose_speed_window_s"]:
                pose_speed = float(
                    np.linalg.norm(current[:3, 3] - first_position) / span
                )
        history_ready = pose_speed is not None
        speed = (
            reported_speed if self.parameters["speed_source"] == "twist" else pose_speed
        )
        # Insufficient measured history must never satisfy RouteTracker's <.04
        # stop test. This sentinel is a gate, not a fabricated speed estimate.
        stop_check_speed = 0.04 if speed is None else speed
        evidence.update(
            pose_window_speed_mps=pose_speed,
            pose_window_span_s=span,
            speed_history_ready=history_ready,
            estimated_speed_mps=speed,
            controller_stop_check_speed_mps=stop_check_speed,
        )
        self._last_valid_measurement = deepcopy(evidence)
        if not distinct:
            return self._finish(
                self._last_command, "tracking", "fresh_duplicate_odometry", evidence
            )
        # Source time measures observed stop/progress duration. Processing time
        # can advance while odometry latency changes and must not create dwell.
        # The wrapper has already enforced the deadline using current sim time.
        command = self.tracker.tick(stamp, p, yaw, stop_check_speed, omega, ready=True)
        self._last_consumed_odom_s = stamp
        if self.failure or self.arrived:
            command = [0.0, 0.0, 0.0]
        bounds = (self.parameters["speed_mps"], 0.05, 0.15)
        self._last_command = [
            max(-bound, min(bound, float(value)))
            for value, bound in zip(command, bounds)
        ]
        return self._finish(
            self._last_command,
            "terminal" if self.failure or self.arrived else "tracking",
            self.failure or ("arrived" if self.arrived else "fresh_measured_odometry"),
            evidence,
        )
