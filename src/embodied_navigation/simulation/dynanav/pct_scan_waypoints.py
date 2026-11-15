"""Measured-pose PCT/SCAN mailbox execution, independent of ROS and Isaac.

Goals use the same goto/cancel/stop protocol and measurement checks as the native
adapter. PCT/SCAN planning runs in an owned sidecar. A goal is accepted before
planning: absence of a map/path is a planner wait, never a new goto prerequisite.
Only native SCAN output is followed, with continuous spatial progress and a
single physical stop at the final operator goal, not at spline sample points.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from embodied_navigation.navigation.robogauge.route_tracker import clip, wrap
from embodied_navigation.simulation.dynanav.native_waypoints import NativeWaypointTracker


def atomic_json(path, payload):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


class ContinuousScanFollower:
    """Follow sampled native SCAN curves without treating samples as goals.

    The body command retains the native bounded heading/cross-track feedback
    gains. Projection and lookahead select the continuous curve reference;
    there are no intermediate capture/brake/turn phases. Final arrival still
    requires measured XYZ proximity and a source-time stationary dwell.
    """

    def __init__(self, mailbox, identity, goal, *, speed, motion_end, stop_on_stall=True,
                 lookahead_m=.35, capture_distance_m=.30):
        self.mailbox = Path(mailbox)
        self.identity = dict(identity)
        self.goal = np.asarray(goal, dtype=float)
        self.speed = float(speed)
        self.motion_end = float(motion_end)
        self.stop_on_stall = stop_on_stall
        self.lookahead_m = float(lookahead_m)
        self.capture_distance_m = float(capture_distance_m)
        self.phase = "planning"
        self.failure = None
        self.arrived = False
        self.events = []
        self.points = self.arc = None
        self.progress = 0.
        self.revision = -1
        self.since = self.stall = None
        self._raw = None
        self.reference = None
        self._last_reason = None
        atomic_json(self.mailbox / "planner_request.json", dict(version=1, **self.identity,
            goal_xyz_m=self.goal.tolist()))

    def invalidate_measurement(self):
        self.since = self.stall = None

    def _read_path(self):
        try:
            raw = (self.mailbox / "planner_path.json").read_bytes()
            if raw == self._raw:
                return
            payload = json.loads(raw)
        except (OSError, ValueError, UnicodeError):
            return
        self._raw = raw
        # These fields associate a native planner result to the currently
        # active request. They are not scene, floor or waypoint admission gates.
        if payload.get("version") != 1 or any(payload.get(key) != value for key, value in self.identity.items()):
            return
        revision = payload.get("revision")
        if type(revision) is not int or revision <= self.revision:
            return
        if payload.get("status") == "hold":
            self.revision = revision
            self.points = self.arc = None
            self.reference = None
            self.phase = "planning"
            self.invalidate_measurement()
            self.events.append(dict(event="native_scan_hold", reason=payload.get("reason"), revision=revision))
            return
        if payload.get("status") == "failed":
            self.revision = revision
            self.failure = "planner_" + str(payload.get("reason", "failed"))
            self.events.append(dict(event="planner_failed", reason=self.failure, revision=revision))
            return
        if payload.get("status") != "tracking":
            return
        provenance = payload.get("provenance", {})
        if provenance.get("local_planner") != "native_SCAN":
            return
        try:
            points = np.asarray(payload["points_xyz_m"], dtype=float)
            if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
                return
            # A repeated spline sample carries no horizontal arc direction.
            lengths = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
            points = points[np.r_[True, lengths > 1e-8]]
            if len(points) < 2:
                return
        except (KeyError, TypeError, ValueError):
            return
        self.points = points
        self.arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1))]
        self.revision = revision
        self.progress = 0.
        self.reference = dict(revision=revision, source_stamp_s=payload.get("source_stamp_s"),
            provenance=provenance, samples=len(points))
        self.events.append(dict(event="native_scan_reference", **self.reference))

    def _project(self, p):
        delta = np.diff(self.points, axis=0)
        fraction = np.clip(np.sum((p - self.points[:-1]) * delta, axis=1) /
            np.sum(delta * delta, axis=1), 0., 1.)
        projected = self.points[:-1] + fraction[:, None] * delta
        at = self.arc[:-1] + fraction * np.diff(self.arc)
        eligible = at >= max(0., self.progress - .05)
        distances = np.linalg.norm(projected - p, axis=1)
        distances[~eligible] = np.inf
        index = int(np.argmin(distances))
        if not math.isfinite(float(distances[index])):
            index = len(delta) - 1
        self.progress = max(self.progress, float(at[index]))
        target_s = min(float(self.arc[-1]), self.progress + self.lookahead_m)
        j = min(len(delta) - 1, max(0, int(np.searchsorted(self.arc, target_s, side="right")) - 1))
        u = (target_s - self.arc[j]) / (self.arc[j + 1] - self.arc[j])
        target = self.points[j] + u * delta[j]
        tangent = delta[index, :2] / np.linalg.norm(delta[index, :2])
        return projected[index], target, tangent

    def tick(self, t, p, yaw, speed, omega, ready=True):
        zero = [0., 0., 0.]
        if self.failure or self.arrived:
            return zero
        if t >= self.motion_end:
            self.failure = "route_motion_budget"
            return zero
        if not ready or t < 7:
            return zero
        self._read_path()
        if self.failure:
            return zero
        p = np.asarray(p, dtype=float)
        final_xy = float(np.linalg.norm(p[:2] - self.goal[:2]))
        final_z = abs(float(p[2] - self.goal[2]))
        if final_xy <= self.capture_distance_m and final_z <= .18:
            self.phase = "brake"
            self.stall = None
            if speed < .04:
                if self.since is None:
                    self.since = t
                if t - self.since >= .5:
                    self.arrived = True
                    self.phase = "arrived"
                    self.events.append(dict(event="final_goal_stopped", time=t,
                        position=p.tolist(), distance=final_xy, z_error=final_z))
            else:
                self.since = None
            return zero
        self.since = None
        if self.points is None:
            self.phase = "planning"
            return zero
        projected, target, tangent = self._project(p)
        local_remaining = float(self.arc[-1] - self.progress)
        if local_remaining < .03:
            # SCAN horizon exhaustion is not final-goal arrival and must never
            # extrapolate a straight segment beyond collision-checked output.
            self.phase = "planning"
            self.stall = None
            return zero
        self.phase = "cruise"
        direction = target[:2] - p[:2]
        if np.linalg.norm(direction) < 1e-8:
            direction = tangent
        error = wrap(math.atan2(direction[1], direction[0]) - yaw)
        normal = np.array([-tangent[1], tangent[0]])
        cross = float((p[:2] - projected[:2]) @ normal)
        if abs(cross) > .6:
            self.failure = "route_cross_track_bound"
            return zero
        forward = min(self.speed, max(.04, .5 * final_xy), max(.04, 2 * local_remaining)) if abs(error) < .25 else 0.
        world = -.5 * cross * normal
        command = [forward, clip(-math.sin(yaw) * world[0] + math.cos(yaw) * world[1], .05), clip(.8 * error, .15)]
        if forward > 0:
            if self.stall is None:
                self.stall = (t, p.copy())
            elif t - self.stall[0] >= 8.:
                progress = float(np.linalg.norm(p - self.stall[1]))
                if progress < .02:
                    self.events.append(dict(event="observed_low_progress_stall", time=t,
                        measured_displacement_m=progress, stop_on_stall=self.stop_on_stall))
                    if self.stop_on_stall:
                        self.failure = "route_low_progress_stall"
                        return zero
                self.stall = (t, p.copy())
        else:
            self.stall = None
        return command


class PctScanWaypointTracker(NativeWaypointTracker):
    """Keep native goal receipts/freshness, replace only route production/following."""

    controller_name = "pct_global_scan_local"

    def __init__(self, experiment, **kwargs):
        if experiment.get("slam_pose_source") != "livo_post_vio":
            raise ValueError("PCT/SCAN exploration uses post-VIO navigation poses")
        super().__init__(experiment, **kwargs)
        self.parameters.update(controller=self.controller_name, global_planner="native_PCT_Astar",
            local_planner="native_SCAN", continuous_path_following=True,
            intermediate_sample_stops=False)

    def _create_tracker(self, candidate, position, evidence):
        identity = dict(run_id=self.run_id, seq=candidate["seq"], frame_id=evidence["frame_id"],
            child_frame_id=evidence["child_frame_id"], pose_source=self.pose_source)
        return ContinuousScanFollower(self.mailbox, identity, candidate["goal_xyz_m"],
            speed=self.speed, motion_end=self.motion_end,
            stop_on_stall=self._route_kwargs["stop_on_stall"],
            capture_distance_m=self._route_kwargs["capture_distance_m"])

    def _finish(self, status, reason, evidence, command=None):
        # Native stop/collision output must also revoke a cached command on a
        # duplicate pose tick; duplicate poses still never advance progress/dwell.
        if self.tracker is not None and reason == "fresh_duplicate_odometry":
            self.tracker._read_path()
            if self.tracker.points is None or self.tracker.failure:
                command = [0., 0., 0.]
        if self.tracker is not None:
            evidence = dict(evidence, planner_reference=self.tracker.reference,
                local_path_progress_m=self.tracker.progress)
            if reason == "native_route":
                reason = "pct_scan_" + self.tracker.phase
        return super()._finish(status, reason, evidence, command)
