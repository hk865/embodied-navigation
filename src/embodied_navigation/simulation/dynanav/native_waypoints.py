"""Live-SLAM mailbox adapter for the existing RoboGauge RouteTracker.

DSH supplies goals, not velocity commands. No map, truth pose, perception proof,
operator heartbeat or request TTL participates in this adapter. RouteTracker
retains its original control law and physical arrival/stall bounds. A completed
or failed single goal returns to zero-command idle; stop requests normal
terminal parking and the run's finite motion budget remains terminal.
"""
from collections import deque
import json
import math
from numbers import Real
from pathlib import Path

import numpy as np

from embodied_navigation.common.geometry import pose_transform
from embodied_navigation.navigation.robogauge.route_tracker import RouteTracker

PROTOCOL_VERSION = 1


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


class NativeWaypointTracker:
    """Adapt measured odometry and goto/cancel/stop files to native tracking."""

    controller_name = "native_route"

    def __init__(self, experiment, *, speed_mps=None, mailbox=None):
        self.pose_source = experiment.get("slam_pose_source")
        if self.pose_source not in ("lio_estimate", "livo_post_vio") or experiment.get("ground_truth_navigation"):
            raise ValueError("Native waypoints require measured SLAM, never ground truth")
        self.mailbox = Path(mailbox or experiment["interactive_mailbox"])
        if not self.mailbox.is_absolute():
            raise ValueError("mailbox must be absolute")
        self.mailbox.mkdir(parents=True, exist_ok=True)
        self.run_id = self.mailbox.parent.name
        self.request_path = self.mailbox / "request.json"
        self.observation_path = self.mailbox / "observation.json"
        self.receipts_path = self.mailbox / "receipts.jsonl"
        self.speed = _number(experiment["speed_mps"] if speed_mps is None else speed_mps, "speed", positive=True)
        if self.speed > 1:
            raise ValueError("speed exceeds native command ceiling")
        self.motion_end = _number(experiment["motion_end_s"], "motion_end", positive=True)
        self.parameters = {
            "command_bounds": {"forward_mps": self.speed, "lateral_mps": .05, "yaw_rate_rps": .15},
            "controller": "embodied_navigation.navigation.robogauge.route_tracker.RouteTracker",
            "require_perception": False,
            "ground_truth_navigation": False,
            "max_sim_age_s": _number(experiment.get("slam_max_sim_age_s", .25), "sim_age", positive=True),
            "max_wall_age_s": _number(experiment.get("slam_max_wall_age_s", .5), "wall_age", positive=True),
            "future_tolerance_s": _number(experiment.get("interactive_future_tolerance_s", experiment.get("slam_future_tolerance_s", 2e-6)), "future", nonnegative=True),
            "pose_speed_window_s": _number(experiment.get("slam_pose_speed_window_s", .5), "window", positive=True),
        }
        self._route_kwargs = dict(speed=self.speed, motion_end=self.motion_end,
            stop_on_stall=experiment.get("stop_on_stall", True),
            capture_remaining_m=experiment.get("interactive_capture_remaining_m", .20),
            capture_distance_m=experiment.get("interactive_capture_distance_m", .30))
        # Validate the native constructor's capture parameters before accepting work.
        RouteTracker([[1., 0., 0.]], [0., 0.], **self._route_kwargs)
        self.tracker = None
        self.active_waypoint = None
        self._pending = deque()
        self._last_seq = 0
        self._request_bytes = None
        self._last_stamp = self._last_consumed = None
        self._last_sim = self._last_wall = None
        self._frames = None
        self._history = deque()
        self._command = [0., 0., 0.]
        self._terminal_reason = None
        self._failure = None
        self._completed = 0
        self._failed = 0
        self._last_waypoint_failure = None
        self._events = []
        self._last_result = None

    @property
    def phase(self):
        return "terminal" if self.terminal_reason or self.failure else self.tracker.phase if self.tracker else "idle"

    @property
    def stage(self):
        return self._completed

    @property
    def failure(self):
        return self._failure

    @property
    def arrived(self):
        return self.terminal_reason == "interactive_stop"

    @property
    def terminal_reason(self):
        return self._terminal_reason

    @property
    def events(self):
        return self._events + (self.tracker.events if self.tracker else [])

    def observe_native_guards(self, **values):
        """Capture owns physical guards; native truth never enters navigation."""

    def invalidate_measurement(self):
        if self.tracker:
            self.tracker.invalidate_measurement()
        self._history.clear()
        self._last_consumed = None
        self._command = [0., 0., 0.]

    def _receipt(self, event, **fields):
        row = dict(protocol_version=PROTOCOL_VERSION, run_id=self.run_id, event=event, **fields)
        self._last_result = row
        with self.receipts_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(row, allow_nan=False) + "\n")
        return row

    def _fail_waypoint(self, reason, evidence):
        """End this goal, not the live sensing/SLAM session or the next choice."""
        failed = self.active_waypoint
        self._events.extend(self.tracker.events)
        self._last_waypoint_failure = self._receipt(
            "waypoint_failed", **failed, reason=reason,
            current_sim_s=evidence["current_sim_s"],
            odom_stamp_s=evidence["odom_stamp_s"],
            position_xyz_m=evidence["aligned_position_xyz_m"],
            command=[0., 0., 0.], run_continues=True)
        self._failed += 1
        # Already queued choices must not resume motion behind the model's
        # back after this failure. Each receives its own closing feedback.
        while self._pending:
            self._receipt("waypoint_dropped", **self._pending.popleft(),
                          reason="preceding_waypoint_failed", failed_seq=failed["seq"])
        self.tracker = self.active_waypoint = None
        self.invalidate_measurement()
        return self._finish("idle", "waypoint_failed", evidence)

    def _fail_run(self, reason, evidence):
        """Keep the original finite session deadline a global terminal event."""
        self._failure = reason
        self._receipt("terminal", reason=reason,
                      current_sim_s=evidence["current_sim_s"],
                      odom_stamp_s=evidence["odom_stamp_s"], command=[0., 0., 0.])
        self.invalidate_measurement()
        return self._finish("terminal", reason, evidence)

    def poll_request(self, now_wall=None):
        """Consume each file revision once; sequence numbers never replay."""
        try:
            raw = self.request_path.read_bytes()
        except FileNotFoundError:
            return
        if raw == self._request_bytes:
            return
        self._request_bytes = raw
        try:
            payload = json.loads(raw)
        except (ValueError, UnicodeDecodeError):
            self._receipt("request_rejected", reason="invalid_json")
            return
        self._handle_request(payload, now_wall)

    def _handle_request(self, payload, now_wall=None):
        seq = payload.get("seq") if isinstance(payload, dict) else None
        try:
            if not isinstance(payload, dict) or type(payload.get("version")) is not int or payload["version"] != PROTOCOL_VERSION:
                raise ValueError("unsupported_protocol_version")
            if payload.get("run_id") != self.run_id:
                raise ValueError("run_id_mismatch")
            if type(seq) is not int or seq <= 0:
                raise ValueError("invalid_sequence")
            if seq <= self._last_seq:
                raise ValueError("duplicate_or_stale_sequence")
            self._last_seq = seq
            if self.terminal_reason or self.failure:
                raise ValueError("run_terminal")
            action = payload.get("action")
            if action not in ("goto", "waypoint", "cancel", "stop"):
                raise ValueError("unsupported_action")
            if payload.get("pose_source", self.pose_source) != self.pose_source:
                raise ValueError("pose_source_mismatch")
            if action in ("stop", "cancel"):
                self._pending.clear()
                self.tracker = self.active_waypoint = None
                self.invalidate_measurement()
                if action == "stop":
                    self._terminal_reason = "interactive_stop"
                self._receipt(action + "_accepted", seq=seq)
                return
            values = payload.get("goal_xyz_m")
            if not isinstance(values, (list, tuple)) or len(values) != 3:
                raise ValueError("invalid_goal")
            goal = [_number(value, "goal_xyz_m") for value in values]
            self._pending.append(dict(seq=seq, goal_xyz_m=goal))
            self._receipt("waypoint_accepted", seq=seq, goal_xyz_m=goal, pose_source=self.pose_source)
        except (ValueError, TypeError) as error:
            self._receipt("request_rejected", seq=seq if type(seq) is int else None, reason=str(error))

    def _create_tracker(self, candidate, position, evidence):
        """Construct the original follower; planned adapters override this hook."""
        return RouteTracker([candidate["goal_xyz_m"]], position[:2].tolist(), **self._route_kwargs)

    def _finish(self, status, reason, evidence, command=None):
        self._command = list(command) if command is not None else [0., 0., 0.]
        result = dict(evidence, status=status, reason=reason, command=self._command,
            phase=self.phase, stage=self.stage, failure=self.failure,
            terminal_reason=self.terminal_reason, pose_source=self.pose_source,
            controller=self.controller_name, no_gt_pose_used=True)
        stamp = evidence.get("odom_stamp_s")
        pose = dict(source=self.pose_source, map_frame=evidence.get("frame_id"),
            sensor_frame=evidence.get("child_frame_id"), stamp_s=stamp,
            stamp_ns=round(stamp * 1e9) if stamp is not None else None,
            position_xyz_m=evidence.get("aligned_position_xyz_m"),
            yaw_rad=evidence.get("aligned_yaw_rad"), fresh=status in ("tracking", "idle"),
            sim_age_s=evidence.get("sim_age_s"), wall_age_s=evidence.get("wall_age_s"),
            alignment="identity_in_live_estimator_map_frame")
        observation = dict(result, protocol_version=PROTOCOL_VERSION, run_id=self.run_id,
            written_wall_s=evidence.get("now_wall_s"), sim_s=evidence.get("current_sim_s"),
            source_ns=pose["stamp_ns"], pose=pose, active_waypoint=self.active_waypoint,
            speed=dict(pose_window_mps=evidence.get("pose_window_speed_mps"), command_ceiling_mps=self.speed),
            waypoints=dict(active_seq=self.active_waypoint["seq"] if self.active_waypoint else None,
                active_goal_xyz_m=self.active_waypoint["goal_xyz_m"] if self.active_waypoint else None,
                queued=len(self._pending), phase=self.phase, completed_goals=self.stage,
                failed_goals=self._failed, last_failure=self._last_waypoint_failure,
                failure=self.failure, terminal_reason=self.terminal_reason, last_result=self._last_result))
        observation["command"] = dict(last_request=self._command[:], bounds=self.parameters["command_bounds"], source=self.controller_name)
        current = evidence.get("aligned_position_xyz_m")
        observation["waypoints"]["distance_to_goal_m"] = (
            math.dist(current[:2], self.active_waypoint["goal_xyz_m"][:2])
            if current is not None and self.active_waypoint is not None else None
        )
        temporary = self.observation_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(observation, allow_nan=False), encoding="utf-8")
        temporary.replace(self.observation_path)
        return self._command[:], result

    def tick(self, current_sim_s, odom_stamp_s, position, quat_wxyz,
             estimated_velocity, measured_yaw_rate, ready, now_wall, arrival_wall,
             *, frame_id=None, child_frame_id=None, pose_source=None,
             imu_source_stamp_s=None, imu_arrival_wall_s=None, imu_frame=None, imu_valid=None, **kwargs):
        """Use source time for native dwell; duplicate poses cannot advance it."""
        self.poll_request(now_wall)
        evidence = dict(current_sim_s=None, odom_stamp_s=None, now_wall_s=None,
            frame_id=frame_id, child_frame_id=child_frame_id, distinct_measurement=False)
        def hold(reason):
            self.invalidate_measurement()
            return self._finish("hold", reason, evidence)
        try:
            sim = _number(current_sim_s, "current_sim_s", nonnegative=True)
            now = _number(now_wall, "now_wall", nonnegative=True)
            evidence.update(current_sim_s=sim, now_wall_s=now)
            if self.terminal_reason or self.failure:
                return self._finish("terminal", self.terminal_reason or self.failure, evidence)
            if self._last_sim is not None and sim < self._last_sim:
                return hold("nonmonotonic_sim_time")
            if self._last_wall is not None and now < self._last_wall:
                return hold("nonmonotonic_wall_time")
            self._last_sim, self._last_wall = sim, now
            if sim >= self.motion_end:
                return self._fail_run("route_motion_budget", evidence)
            if ready is not True:
                return hold("not_ready")
            stamp = _number(odom_stamp_s, "odom_stamp_s", nonnegative=True)
            arrival = _number(arrival_wall, "arrival_wall", nonnegative=True)
            age, wall_age = sim - stamp, now - arrival
            evidence.update(odom_stamp_s=stamp, sim_age_s=max(0., age), wall_age_s=wall_age)
            if age < -self.parameters["future_tolerance_s"]:
                return hold("future_odometry")
            if age > self.parameters["max_sim_age_s"]:
                return hold("stale_sim_odometry")
            if wall_age < 0 or wall_age > self.parameters["max_wall_age_s"]:
                return hold("stale_or_future_wall_odometry")
            if pose_source != self.pose_source:
                return hold("pose_source_mismatch")
            if not isinstance(frame_id, str) or not frame_id or not isinstance(child_frame_id, str) or not child_frame_id:
                return hold("invalid_frame_labels")
            frames = (frame_id, child_frame_id)
            if self._frames is not None and frames != self._frames:
                return hold("frame_mismatch")
            if self._last_stamp is not None and stamp < self._last_stamp:
                return hold("nonmonotonic_odometry_stamp")
            transform = pose_transform(position, quat_wxyz)
            omega = _number(measured_yaw_rate, "measured_yaw_rate")
            if imu_valid is False:
                return hold("invalid_measured_gyro")
        except (ValueError, TypeError):
            return hold("invalid_measured_odometry")
        self._frames = frames
        p = transform[:3, 3]
        yaw = math.atan2(transform[1, 0], transform[0, 0])
        evidence.update(aligned_position_xyz_m=p.tolist(), aligned_yaw_rad=yaw)
        if self._last_stamp is not None and stamp - self._last_stamp > self.parameters["max_sim_age_s"]:
            self.invalidate_measurement()
        self._last_stamp = stamp
        distinct = stamp != self._last_consumed
        evidence["distinct_measurement"] = distinct
        if distinct:
            self._history.append((stamp, p.copy()))
            cutoff = stamp - self.parameters["pose_speed_window_s"]
            while len(self._history) > 2 and self._history[1][0] <= cutoff:
                self._history.popleft()
        speed = None
        if self._history:
            start, first = self._history[0]
            if stamp - start >= self.parameters["pose_speed_window_s"]:
                speed = float(np.linalg.norm(p - first) / (stamp - start))
        evidence["pose_window_speed_mps"] = speed
        # Activation occurs only on an actual measured pose, without frame alignment.
        if self.tracker is None and self._pending:
            candidate = self._pending.popleft()
            goal = candidate["goal_xyz_m"]
            if math.hypot(goal[0] - p[0], goal[1] - p[1]) == 0:
                self._receipt("request_rejected", seq=candidate["seq"], reason="undefined_zero_length_native_segment")
            else:
                self.active_waypoint = candidate
                self.tracker = self._create_tracker(candidate, p, evidence)
                self._command = [0., 0., 0.]
        if self.tracker is None:
            self._last_consumed = stamp
            return self._finish("idle", "awaiting_waypoint", evidence)
        if not distinct:
            return self._finish("tracking", "fresh_duplicate_odometry", evidence, self._command)
        command = self.tracker.tick(stamp, p.tolist(), yaw, .04 if speed is None else speed, omega, ready=True)
        self._last_consumed = stamp
        if self.tracker.failure:
            if self.tracker.failure == "route_motion_budget":
                return self._fail_run(self.tracker.failure, evidence)
            return self._fail_waypoint(self.tracker.failure, evidence)
        if self.tracker.arrived:
            self._receipt("waypoint_reached", **self.active_waypoint)
            self._events.extend(self.tracker.events)
            self._completed += 1
            self.tracker = self.active_waypoint = None
            return self._finish("idle", "waypoint_reached", evidence)
        return self._finish("tracking", "native_route", evidence, command)
