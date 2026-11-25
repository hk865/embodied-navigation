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

from embodied_navigation.common.geometry import pose_transform, wrap_angle
from embodied_navigation.common.runtime_telemetry import RuntimeTelemetry
from embodied_navigation.common.time import float32_time_half_ulp
from embodied_navigation.navigation.robogauge.route_tracker import RouteTracker
from embodied_navigation.navigation.yaw_action import (
    LOCAL_ACTION_BOUNDS,
    resolve_yaw_target,
    yaw_action_step,
    yaw_config,
    yaw_request_parameters,
)
from embodied_navigation.perception.pose_velocity import update_pose_velocity

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
        if self.pose_source not in ("lio_estimate", "livo_post_vio", "backend_corrected", "simulation_truth") or experiment.get("ground_truth_navigation"):
            raise ValueError("Native waypoints require measured SLAM or explicit simulation_truth, never ground truth")
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
        motion_end = experiment["motion_end_s"]
        self.motion_end = (None if motion_end is None
                           else _number(motion_end, "motion_end", positive=True))
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
        self._action = None
        self._action_epoch = 0
        self._inbox_dir = self.mailbox / "waypoint-inbox"
        self._inbox_seqs = set()
        self._yaw_window = None
        self._yaw_query_id = 0
        self._last_pose_evidence = {}
        self._yaw_config = dict(experiment.get("yaw_action") or yaw_config(experiment))
        self._yaw_config.update(max_sim_age_s=self.parameters["max_sim_age_s"],
                                max_wall_age_s=self.parameters["max_wall_age_s"])
        yaw_rate_ceiling = float(self._yaw_config.get("yaw_rate_rps", LOCAL_ACTION_BOUNDS["yaw_rate_rps"]))
        self.parameters["command_bounds"]["yaw_rate_rps"] = yaw_rate_ceiling
        self.parameters["command_bounds"]["route_yaw_rate_rps"] = LOCAL_ACTION_BOUNDS["yaw_rate_rps"]
        self.parameters["command_bounds"]["maneuver_yaw_rate_rps"] = LOCAL_ACTION_BOUNDS["yaw_rate_rps"]

    @property
    def phase(self):
        return ("terminal" if self.terminal_reason or self.failure else self._action["kind"]
                if self._action is not None else self.tracker.phase if self.tracker else "idle")

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
        self._yaw_window = None
        if self._action is not None:
            self._action.pop("_control_state", None)

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
        self._close_yaw("yaw_failed", reason=reason, evidence=evidence)
        self._failure = reason
        self._receipt("terminal", reason=reason,
                      current_sim_s=evidence["current_sim_s"],
                      odom_stamp_s=evidence["odom_stamp_s"], command=[0., 0., 0.])
        self.invalidate_measurement()
        return self._finish("terminal", reason, evidence)

    def _drain_inbox(self, now_wall=None):
        """Consume ordered per-sequence inbox entries before falling back to request.json."""
        if not self._inbox_dir.is_dir():
            return
        try:
            entries = [
                entry for entry in self._inbox_dir.iterdir()
                if entry.is_file() and entry.suffix == ".json"
            ]
        except OSError:
            return
        entries.sort(key=lambda p: (0, int(p.stem)) if p.stem.isdigit() else (1, p.name))
        for entry in entries:
            try:
                raw = entry.read_bytes()
            except OSError:
                continue
            try:
                entry.unlink(missing_ok=True)
            except OSError:
                pass
            try:
                payload = json.loads(raw)
            except (ValueError, UnicodeDecodeError):
                self._receipt("request_rejected", reason="invalid_json")
                continue
            if isinstance(payload, dict) and type(payload.get("seq")) is int:
                self._inbox_seqs.add(payload["seq"])
                if len(self._inbox_seqs) > 2048:
                    cutoff = self._last_seq - 1024
                    self._inbox_seqs = {s for s in self._inbox_seqs if s > cutoff}
            self._handle_request(payload, now_wall)

    def poll_request(self, now_wall=None):
        """Consume ordered inbox entries and legacy request.json revisions once."""
        self._drain_inbox(now_wall)
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
        if isinstance(payload, dict) and payload.get("delivery") == "inbox":
            return
        if (
            isinstance(payload, dict)
            and type(payload.get("seq")) is int
            and payload["seq"] in self._inbox_seqs
        ):
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
            if action not in ("goto", "waypoint", "cancel", "stop", "yaw", "maneuver"):
                raise ValueError("unsupported_action")
            if payload.get("pose_source", self.pose_source) != self.pose_source:
                raise ValueError("pose_source_mismatch")
            preempt = payload.get("preempt", False)
            if type(preempt) is not bool:
                raise ValueError("preempt_must_be_boolean")
            if preempt and action in ("cancel", "stop"):
                raise ValueError("preempt_not_supported_for_" + action)
            raw_epoch = payload.get("action_epoch")
            action_epoch = None
            if raw_epoch is not None:
                if type(raw_epoch) is not int or raw_epoch < 0:
                    raise ValueError("invalid_action_epoch")
                action_epoch = raw_epoch
                if action_epoch < self._action_epoch:
                    raise ValueError("stale_action_epoch")
                if preempt and self._action_epoch > 0 and action_epoch <= self._action_epoch:
                    raise ValueError("stale_action_epoch")
            elif self._action_epoch > 0 and action != "stop":
                raise ValueError("stale_action_epoch")

            if action in ("stop", "cancel"):
                if action_epoch is not None and action_epoch > self._action_epoch:
                    self._action_epoch = action_epoch
                self._cancel_all(
                    reason="operator_" + action,
                    cancelled_by_seq=seq,
                    run_continues=(action != "stop"),
                )
                if action == "stop":
                    self._terminal_reason = "interactive_stop"
                self._receipt(action + "_accepted", seq=seq, action_epoch=self._action_epoch)
                return
            if action == "maneuver":
                forward = _number(payload.get("body_forward_mps", 0.0), "body_forward_mps")
                lateral = _number(payload.get("body_lateral_mps", 0.0), "body_lateral_mps")
                yaw_rate = _number(payload.get("yaw_rate_rps", 0.0), "yaw_rate_rps")
                duration = _number(payload.get("duration_s"), "duration_s")
                if abs(forward) > self.speed:
                    raise ValueError("invalid_body_forward_mps")
                if abs(lateral) > LOCAL_ACTION_BOUNDS["lateral_mps"]:
                    raise ValueError("invalid_body_lateral_mps")
                if abs(yaw_rate) > LOCAL_ACTION_BOUNDS["yaw_rate_rps"]:
                    raise ValueError("invalid_yaw_rate_rps")
                if not 0.1 <= duration <= LOCAL_ACTION_BOUNDS["maneuver_max_duration_s"]:
                    raise ValueError("invalid_duration_s")
                created = _number(payload.get("created_wall_s"), "created_wall_s")
                ttl = _number(payload.get("ttl_s", 30.), "ttl_s")
                if not 1. <= ttl <= 120.:
                    raise ValueError("invalid_ttl")
                if now_wall is not None:
                    if created > now_wall + 2.:
                        raise ValueError("request_from_future")
                    if now_wall - created > ttl:
                        raise ValueError("request_expired")
                item_epoch = self._advance_epoch(action_epoch)
                if preempt:
                    self._preempt_active(preempted_by_seq=seq, action_name="maneuver")
                else:
                    self._supersede_active(action_name="maneuver")
                command = [float(forward), float(lateral), float(yaw_rate)]
                self._action = dict(
                    kind="maneuver", seq=seq, command=command, duration_s=float(duration),
                    start_sim_s=None, start_position_xyz_m=None, expires_wall_s=created + ttl,
                    action_epoch=item_epoch,
                )
                self._receipt(
                    "maneuver_accepted", seq=seq, action="maneuver",
                    body_forward_mps=command[0], body_lateral_mps=command[1],
                    yaw_rate_rps=command[2], duration_s=float(duration),
                    ttl_s=ttl, pose_source=self.pose_source,
                    action_epoch=item_epoch,
                )
                return
            if action == "yaw":
                parameters = yaw_request_parameters(
                    payload,
                    max_yaw_rate_rps=self.parameters["command_bounds"]["yaw_rate_rps"],
                )
                created = _number(payload.get("created_wall_s"), "created_wall_s")
                ttl = _number(payload.get("ttl_s", 30.), "ttl_s")
                if not 1. <= ttl <= 120.:
                    raise ValueError("invalid_ttl")
                if now_wall is not None:
                    if created > now_wall + 2.:
                        raise ValueError("request_from_future")
                    if now_wall - created > ttl:
                        raise ValueError("request_expired")
                item_epoch = self._advance_epoch(action_epoch)
                if preempt:
                    self._preempt_active(preempted_by_seq=seq, action_name="yaw")
                else:
                    self._supersede_active(action_name="yaw")
                self._yaw_query_id += 1
                self._action = dict(kind="yaw", seq=seq, parameters=parameters,
                                    target_yaw_rad=None, expires_wall_s=created + ttl,
                                    query_id=self._yaw_query_id, action_epoch=item_epoch)
                self._receipt("yaw_accepted", seq=seq, action="yaw", parameters=parameters,
                              completion=self._yaw_config, ttl_s=ttl, pose_source=self.pose_source,
                              action_epoch=item_epoch)
                return
            values = payload.get("goal_xyz_m")
            if not isinstance(values, (list, tuple)) or len(values) != 3:
                raise ValueError("invalid_goal")
            goal = [_number(value, "goal_xyz_m") for value in values]
            reverse = payload.get("reverse", False)
            if type(reverse) is not bool:
                raise ValueError("reverse_must_be_boolean")
            if reverse and self.controller_name != "native_route":
                raise ValueError("reverse_requires_native_route")
            item_epoch = self._advance_epoch(action_epoch)
            if preempt:
                self._preempt_active(preempted_by_seq=seq, action_name=action)
            candidate = dict(seq=seq, goal_xyz_m=goal, action_epoch=item_epoch)
            if reverse:
                candidate["reverse"] = True
            self._pending.append(candidate)
            self._receipt("waypoint_accepted", **candidate, pose_source=self.pose_source)
        except (ValueError, TypeError) as error:
            self._receipt("request_rejected", seq=seq if type(seq) is int else None, reason=str(error))

    def _advance_epoch(self, action_epoch):
        if action_epoch is not None and action_epoch > self._action_epoch:
            self._action_epoch = action_epoch
        return self._action_epoch if action_epoch is None else action_epoch

    def _cancel_all(self, *, reason, cancelled_by_seq, run_continues):
        """Emit action_cancelled receipts for active and every queued item."""
        self._close_yaw(
            "action_cancelled",
            reason=reason,
            cancelled_by_seq=cancelled_by_seq,
            cancelled_by_epoch=self._action_epoch,
        )
        if self.active_waypoint is not None:
            if self.tracker is not None:
                self._events.extend(self.tracker.events)
            self._receipt(
                "action_cancelled",
                **self.active_waypoint,
                action="goto",
                reason=reason,
                cancelled_by_seq=cancelled_by_seq,
                cancelled_by_epoch=self._action_epoch,
                command=[0., 0., 0.],
                run_continues=run_continues,
            )
        while self._pending:
            self._receipt(
                "action_cancelled",
                **self._pending.popleft(),
                action="goto",
                reason=reason,
                cancelled_by_seq=cancelled_by_seq,
                cancelled_by_epoch=self._action_epoch,
                command=[0., 0., 0.],
                run_continues=run_continues,
            )
        self.tracker = self.active_waypoint = None
        self.invalidate_measurement()

    def _supersede_active(self, *, action_name):
        """Close active yaw/maneuver and drop queued waypoints for non-preempt local actions."""
        self._close_yaw("action_cancelled", reason="superseded_by_" + action_name)
        if self.tracker is not None:
            self._events.extend(self.tracker.events)
            self._receipt("waypoint_dropped", **self.active_waypoint,
                          reason="operator_" + action_name)
        while self._pending:
            self._receipt("waypoint_dropped", **self._pending.popleft(),
                          reason="operator_" + action_name)
        self.tracker = self.active_waypoint = None
        self.invalidate_measurement()

    def _preempt_active(self, *, preempted_by_seq, action_name):
        """Atomically emit action_preempted for active/queued work and clear state."""
        reason = "preempted_by_" + action_name
        if self._action is not None:
            self._close_yaw(
                "action_preempted",
                reason=reason,
                preempted_by_seq=preempted_by_seq,
                action_epoch=self._action_epoch,
                preempted_by_epoch=self._action_epoch,
                preempted_action_epoch=self._action.get("action_epoch"),
            )
        if self.active_waypoint is not None:
            if self.tracker is not None:
                self._events.extend(self.tracker.events)
            item_fields = dict(self.active_waypoint)
            item_epoch = item_fields.pop("action_epoch", None)
            self._receipt(
                "action_preempted",
                **item_fields,
                action="goto",
                reason=reason,
                preempted_by_seq=preempted_by_seq,
                action_epoch=self._action_epoch,
                preempted_by_epoch=self._action_epoch,
                preempted_action_epoch=item_epoch,
                command=[0., 0., 0.],
                run_continues=True,
            )
        while self._pending:
            item_fields = dict(self._pending.popleft())
            item_epoch = item_fields.pop("action_epoch", None)
            self._receipt(
                "action_preempted",
                **item_fields,
                action="goto",
                reason=reason,
                preempted_by_seq=preempted_by_seq,
                action_epoch=self._action_epoch,
                preempted_by_epoch=self._action_epoch,
                preempted_action_epoch=item_epoch,
                command=[0., 0., 0.],
                run_continues=True,
            )
        self.tracker = self.active_waypoint = None
        self.invalidate_measurement()

    def _create_tracker(self, candidate, position, evidence):
        """Construct the original follower; planned adapters override this hook."""
        return RouteTracker([candidate["goal_xyz_m"]], position[:2].tolist(), reverse=candidate.get("reverse", False), **self._route_kwargs)

    def _close_yaw(self, event, *, reason=None, evidence=None, completion=None, **extra_fields):
        """Close the active local action (yaw or direct maneuver), leaving the sensing session alive."""
        action = self._action
        if action is None:
            return
        evidence = self._last_pose_evidence if evidence is None else evidence
        if action["kind"] == "maneuver":
            close_event = "action_cancelled" if event == "yaw_failed" else event
            start_sim = action.get("start_sim_s")
            cur_sim = evidence.get("current_sim_s")
            elapsed = (
                max(0.0, float(cur_sim) - float(start_sim))
                if start_sim is not None and cur_sim is not None else 0.0
            )
            fields = dict(
                seq=action["seq"], action="maneuver",
                requested_command=list(action["command"]), duration_s=action["duration_s"],
                elapsed_sim_s=elapsed,
                start_position_xyz_m=action.get("start_position_xyz_m"),
                frame_id=evidence.get("frame_id"), child_frame_id=evidence.get("child_frame_id"),
                pose_source=self.pose_source, odom_stamp_s=evidence.get("odom_stamp_s"),
                position_xyz_m=evidence.get("aligned_position_xyz_m"),
                action_epoch=action.get("action_epoch", self._action_epoch),
                command=[0., 0., 0.], run_continues=True,
            )
            if reason is not None:
                fields["reason"] = reason
            fields.update(extra_fields)
            self._receipt(close_event, **fields)
            self._action = None
            self.invalidate_measurement()
            return
        actual = evidence.get("aligned_yaw_rad")
        target = action["target_yaw_rad"]
        fields = dict(completion or {})
        fields.update(seq=action["seq"], action="yaw", target_yaw_rad=target,
                      actual_yaw_rad=actual, achieved_yaw_rad=actual,
                      yaw_error_rad=wrap_angle(target - actual)
                      if target is not None and actual is not None else None,
                      frame_id=evidence.get("frame_id"), child_frame_id=evidence.get("child_frame_id"),
                      pose_source=self.pose_source, odom_stamp_s=evidence.get("odom_stamp_s"),
                      position_xyz_m=evidence.get("aligned_position_xyz_m"),
                      action_epoch=action.get("action_epoch", self._action_epoch),
                      command=[0., 0., 0.], run_continues=True)
        if reason is not None:
            fields["reason"] = reason
        if action.get("space") is not None:
            fields["rotation_space"] = action["space"]
        fields.update(extra_fields)
        self._receipt(event, **fields)
        if event == "yaw_failed":
            while self._pending:
                self._receipt("waypoint_dropped", **self._pending.popleft(),
                              reason="preceding_yaw_failed", failed_seq=action["seq"])
        self._action = None
        self.invalidate_measurement()

    def _tick_maneuver(self, evidence, sim):
        """Execute a bounded direct body-velocity recovery maneuver without PCT/SCAN dependency."""
        action = self._action
        if action["start_sim_s"] is None:
            action["start_sim_s"] = sim
            action["start_position_xyz_m"] = evidence.get("aligned_position_xyz_m")
        if sim - action["start_sim_s"] >= action["duration_s"]:
            self._close_yaw("maneuver_complete", evidence=evidence)
            return self._finish("idle", "maneuver_complete", evidence)
        return self._finish("tracking", "maneuver_recovery", evidence, action["command"])

    def _yaw_clearance(self, evidence):
        """Use the owned SCAN map reply; missing or stale evidence permits no turn."""
        action = self._action
        if "query" not in action:
            action["query"] = dict(version=1, run_id=self.run_id, seq=action["seq"],
                query_id=action["query_id"], frame_id=evidence["frame_id"],
                child_frame_id=evidence["child_frame_id"], pose_source=self.pose_source,
                body_stamp_ns=round(evidence["odom_stamp_s"] * 1e9),
                target_yaw_rad=action["target_yaw_rad"])
            RuntimeTelemetry().atomic(self.mailbox / "planner_yaw_query.json", action["query"])
        query = action["query"]
        pending = dict(status="pending", reason="rotation_space_unavailable")
        try:
            reply = json.loads((self.mailbox / "planner_yaw_space.json").read_text())
        except (OSError, ValueError, UnicodeError):
            return pending
        if (not isinstance(reply, dict) or type(reply.get("version")) is not int
                or reply["version"] != 1
                or any(reply.get(key) != query[key] for key in ("run_id", "seq", "query_id", "target_yaw_rad"))):
            return dict(pending, reason="rotation_space_identity_mismatch")
        status = reply.get("status")
        if status in ("pending", "failed", "blocked"):
            return reply
        if status != "clear":
            return dict(pending, reason="invalid_rotation_space_status")
        try:
            stamp = reply["body_stamp_ns"]
            if type(stamp) is not int or stamp < query["body_stamp_ns"]:
                return dict(pending, reason="rotation_space_old_body_pose")
            age = evidence["current_sim_s"] - stamp / 1e9
            wall_age = evidence["now_wall_s"] - _number(reply["received_wall_s"], "received_wall_s")
            source = reply["source"]
            if (source["body_stamp_ns"] != stamp
                    or source["body_frame_id"] != query["frame_id"]
                    or source["body_child_frame_id"] != query["child_frame_id"]):
                return dict(pending, reason="rotation_space_frame_mismatch")
            if (age < -evidence["odometry_future_tolerance_s"]
                    or age > self.parameters["max_sim_age_s"]
                    or not 0. <= wall_age <= self.parameters["max_wall_age_s"]):
                return dict(pending, reason="rotation_space_stale")
            if reply.get("occupancy") != 0:
                return dict(pending, reason="rotation_space_not_clear")
        except (KeyError, TypeError, ValueError):
            return dict(pending, reason="invalid_rotation_space_reply")
        return reply

    def _tick_yaw(self, evidence, context):
        action = self._action
        parameters = action["parameters"]
        if "frame_id" in parameters and parameters["frame_id"] != evidence["frame_id"]:
            self._close_yaw("yaw_failed", reason="yaw_frame_mismatch", evidence=evidence)
            return self._finish("idle", "yaw_failed", evidence)
        yaw = evidence["aligned_yaw_rad"]
        resolve_yaw_target(action, yaw)
        space = self._yaw_clearance(evidence)
        action["space"] = space
        evidence = dict(evidence, rotation_space=space, target_yaw_rad=action["target_yaw_rad"])
        if space["status"] in ("blocked", "failed"):
            self._close_yaw("yaw_failed", reason=space.get("reason", "rotation_space_blocked"), evidence=evidence)
            return self._finish("idle", "yaw_failed", evidence)
        if space["status"] != "clear":
            self._yaw_window = None
            action.pop("_control_state", None)
            return self._finish("hold", space.get("reason", "rotation_space_pending"), evidence)
        command, self._yaw_window, completion = yaw_action_step(
            action, self._yaw_config, context, self._yaw_window, yaw=yaw,
            now_wall=evidence["now_wall_s"],
            future_tolerance_s=evidence["odometry_future_tolerance_s"],
        )
        if completion is not None:
            self._close_yaw("yaw_complete", evidence=evidence, completion=completion)
            return self._finish("idle", "yaw_complete", evidence)
        return self._finish("tracking", "yaw_adjustment", evidence, command)

    def _finish(self, status, reason, evidence, command=None):
        self._command = list(command) if command is not None else [0., 0., 0.]
        result = dict(evidence, status=status, reason=reason, command=self._command,
            phase=self.phase, stage=self.stage, failure=self.failure,
            terminal_reason=self.terminal_reason, pose_source=self.pose_source,
            controller=self.controller_name, no_gt_pose_used=(self.pose_source != "simulation_truth"))
        stamp = evidence.get("odom_stamp_s")
        pose = dict(source=self.pose_source, map_frame=evidence.get("frame_id"),
            sensor_frame=evidence.get("child_frame_id"), stamp_s=stamp,
            stamp_ns=round(stamp * 1e9) if stamp is not None else None,
            position_xyz_m=evidence.get("aligned_position_xyz_m"),
            yaw_rad=evidence.get("aligned_yaw_rad"), fresh=status in ("tracking", "idle"),
            sim_age_s=evidence.get("sim_age_s"), wall_age_s=evidence.get("wall_age_s"),
            alignment="identity_in_live_estimator_map_frame")
        active_action_epoch = (
            self._action.get("action_epoch")
            if self._action is not None
            else self.active_waypoint.get("action_epoch")
            if self.active_waypoint is not None
            else None
        )
        observation = dict(result, protocol_version=PROTOCOL_VERSION, run_id=self.run_id,
            written_wall_s=evidence.get("now_wall_s"), sim_s=evidence.get("current_sim_s"),
            source_ns=pose["stamp_ns"], pose=pose, active_waypoint=self.active_waypoint,
            speed=dict(pose_window_mps=evidence.get("pose_window_speed_mps"), command_ceiling_mps=self.speed),
            waypoints=dict(active_seq=self._action["seq"] if self._action is not None
                else self.active_waypoint["seq"] if self.active_waypoint else None,
                active_action=self._action["kind"] if self._action is not None else "goto" if self.active_waypoint else None,
                active_goal_xyz_m=self.active_waypoint["goal_xyz_m"] if self.active_waypoint else None,
                queued=len(self._pending), phase=self.phase, completed_goals=self.stage,
                failed_goals=self._failed, last_failure=self._last_waypoint_failure,
                failure=self.failure, terminal_reason=self.terminal_reason, last_result=self._last_result,
                current_epoch=self._action_epoch,
                active_action_epoch=active_action_epoch,
                action_epoch=active_action_epoch if active_action_epoch is not None else self._action_epoch))
        if self._action is not None:
            if self._action["kind"] == "yaw":
                observation["yaw_action"] = dict(seq=self._action["seq"],
                    target_yaw_rad=self._action["target_yaw_rad"], parameters=self._action["parameters"],
                    expires_wall_s=self._action["expires_wall_s"], completion=self._yaw_config,
                    action_epoch=self._action.get("action_epoch"))
            else:
                observation["maneuver_action"] = dict(seq=self._action["seq"],
                    command=list(self._action["command"]), duration_s=self._action["duration_s"],
                    start_sim_s=self._action["start_sim_s"], expires_wall_s=self._action["expires_wall_s"],
                    action_epoch=self._action.get("action_epoch"))
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
            if self._action is not None and now > self._action["expires_wall_s"]:
                expired_kind = self._action["kind"]
                if expired_kind == "maneuver":
                    self._close_yaw("action_expired", reason="maneuver_ttl_expired")
                    return self._finish("hold", "maneuver_ttl_expired", evidence)
                self._close_yaw("yaw_failed", reason="yaw_ttl_expired")
                # This tick has not admitted a pose; do not label an empty pose
                # fresh merely because the expired action returned to idle.
                return self._finish("hold", "yaw_ttl_expired", evidence)
            if self.terminal_reason or self.failure:
                return self._finish("terminal", self.terminal_reason or self.failure, evidence)
            if self._last_sim is not None and sim < self._last_sim:
                return hold("nonmonotonic_sim_time")
            if self._last_wall is not None and now < self._last_wall:
                return hold("nonmonotonic_wall_time")
            self._last_sim, self._last_wall = sim, now
            if self.motion_end is not None and sim >= self.motion_end:
                return self._fail_run("route_motion_budget", evidence)
            if ready is not True:
                return hold("not_ready")
            stamp = _number(odom_stamp_s, "odom_stamp_s", nonnegative=True)
            arrival = _number(arrival_wall, "arrival_wall", nonnegative=True)
            age, wall_age = sim - stamp, now - arrival
            evidence.update(odom_stamp_s=stamp, sim_age_s=max(0., age), wall_age_s=wall_age)
            # RTX source time is float32; sim time is float64. Bound same-tick
            # rounding by half the existing 0.005 s physical tick.
            future_tolerance = min(max(self.parameters["future_tolerance_s"],
                                       float32_time_half_ulp(sim) + 1e-7), 0.0025)
            evidence.update(raw_sim_age_s=age,
                            odometry_future_tolerance_s=future_tolerance)
            if age < -future_tolerance:
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
        velocity = update_pose_velocity(self._history, stamp, p,
                                        self.parameters["pose_speed_window_s"], distinct=distinct)
        speed = None if velocity is None else float(np.linalg.norm(velocity))
        evidence["pose_window_speed_mps"] = speed
        self._last_pose_evidence = dict(evidence)
        if self._action is not None:
            if self._action["kind"] == "maneuver":
                result = self._tick_maneuver(evidence, sim)
                self._last_consumed = stamp
                return result
            context = dict(sim=sim, stamp=stamp, raw_age=age, wall_age=wall_age,
                distinct=distinct, imu_yaw_rate=omega, measured_yaw_rate_rps=omega,
                imu_source_stamp_s=imu_source_stamp_s,
                imu_arrival_wall_s=imu_arrival_wall_s, imu_frame=imu_frame, imu_valid=imu_valid)
            # Evaluate the inherited 50 Hz dwell on fresh duplicates too. Its
            # distinct-source and gyro-span checks still prevent false completion.
            result = self._tick_yaw(evidence, context)
            self._last_consumed = stamp
            return result
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
