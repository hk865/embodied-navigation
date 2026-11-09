"""Pure bounded measured-pose route controller; no simulator or policy side effects.

``RouteTracker`` consumes an already measured pose/speed sample per ``tick``
and returns a bounded body-frame command ``[vx, vy, wz]``. It owns no ROS,
simulator, model or admission state and does not identify the pose source or
coordinate frame. Callers supply that contract: DynaNav uses it directly for
native-pose diagnostics and through ``SlamRouteTracker`` for estimator poses
aligned into a measured reference frame. Outputs are the command, phase/failure
state and recorded evidence events.
"""

import math

wrap = lambda a: math.atan2(math.sin(a), math.cos(a))
clip = lambda x, b: max(-b, min(b, x))


class RouteTracker:
    """Bounded waypoint tracker over one measured 3D point list.

    ``points`` are measured XYZ waypoints in metres (z is checked separately, so a
    wrong floor cannot fake arrival), ``start_xy`` is the initial XY pose used as
    the origin of stage 0, ``speed`` is the maximum forward command in m/s,
    ``motion_end`` is the deadline compared with ``t`` in seconds, and ``stop_on_stall``
    chooses whether an observed low-progress stall becomes a failure. Capture
    bounds default to the original 0.20/0.30 m; optional final-only overrides
    inherit those bounds when None and never alter intermediate stages.
    """

    def __init__(
        self,
        points,
        start_xy,
        speed=0.15,
        motion_end=230.0,
        stop_on_stall=True,
        *,
        capture_remaining_m=0.20,
        capture_distance_m=0.30,
        final_capture_remaining_m=None,
        final_capture_distance_m=None,
    ):
        """Bind the route and initial controller state; performs no I/O."""
        self.points = points
        self.start = start_xy
        self.speed = speed
        self.motion_end = motion_end
        self.stage = 0
        self.phase = "waiting"
        self.failure = None
        self.arrived = False
        self.events = []
        self.since = None
        self.phase_start = None
        self.stall = None
        self.stop_on_stall = stop_on_stall
        if (
            any(
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                for value in (capture_remaining_m, capture_distance_m)
            )
            or not 0 < capture_remaining_m <= capture_distance_m <= 0.40
        ):
            raise ValueError(
                "capture bounds require 0 < remaining <= distance <= 0.40 m"
            )
        # Defaults preserve the original controller. Measured-reference callers
        # can explicitly select tighter capture only after checking turn margin.
        self.capture_remaining_m = float(capture_remaining_m)
        self.capture_distance_m = float(capture_distance_m)
        final_values = (final_capture_remaining_m, final_capture_distance_m)
        if any(
            value is not None
            and (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            )
            for value in final_values
        ):
            raise ValueError("final capture overrides must be finite numbers or None")
        self.final_capture_remaining_m = (
            None
            if final_capture_remaining_m is None
            else float(final_capture_remaining_m)
        )
        self.final_capture_distance_m = (
            None
            if final_capture_distance_m is None
            else float(final_capture_distance_m)
        )
        effective_remaining = (
            self.capture_remaining_m
            if self.final_capture_remaining_m is None
            else self.final_capture_remaining_m
        )
        effective_distance = (
            self.capture_distance_m
            if self.final_capture_distance_m is None
            else self.final_capture_distance_m
        )
        if not 0 < effective_remaining <= effective_distance <= 0.40:
            raise ValueError(
                "final capture bounds require 0 < remaining <= distance <= 0.40 m"
            )

    def invalidate_measurement(self):
        """Drop the current dwell/stall window without changing phase or failure.

        Called when a measurement interval is stale: the continuous-dwell timers
        must restart from the next fresh sample.
        """
        self.since = None
        self.stall = None

    def tick(self, t, p, yaw, speed, omega, ready=True):
        """Return one bounded body command for a measured pose sample.

        ``t``      sim time, seconds.
        ``p``      measured XYZ position in the supplied route frame, metres.
        ``yaw``    measured heading, radians.
        ``speed``  caller-provided speed scalar, m/s (compared with .04 to confirm a stop).
        ``omega``  measured yaw rate, rad/s (used only to confirm a stop).
        ``ready``  False skips tracking updates after the terminal/deadline checks.

        Returns ``[vx, vy, wz]``: forward speed capped at ``self.speed``, lateral
        bounded to .05 m/s and yaw rate to .15 rad/s. Calls entered with ``failure``
        or ``arrived`` already set return zero. Existing stall/timeout branches
        may newly set failure after computing a command and return that command
        for the current tick; the next tick is gated at entry.
        """
        command = [0.0, 0.0, 0.0]
        if self.failure or self.arrived:
            return command
        if t >= self.motion_end:
            self.failure = "route_motion_budget"
            return command
        # No command before t=7 s or when the caller says the inputs are not
        # ready; these gates leave the phase untouched.
        if not ready or t < 7:
            return command
        goal = self.points[self.stage]
        # Stage 0 starts from the given start XY; later stages start from the
        # previous waypoint's XY.
        origin = self.start if self.stage == 0 else self.points[self.stage - 1][:2]
        # Segment frame: u is the unit origin->goal direction in XY, normal is
        # its left-hand normal, heading is the segment bearing.
        dx = goal[0] - origin[0]
        dy = goal[1] - origin[1]
        length = math.hypot(dx, dy)
        u = (dx / length, dy / length)
        normal = (-u[1], u[0])
        heading = math.atan2(dy, dx)
        # error: wrapped bearing error, remaining: signed distance to go along
        # u, cross: signed lateral offset from the segment line, distance: 2D
        # distance to the waypoint, z_error: vertical distance to it.
        error = wrap(heading - yaw)
        remaining = (goal[0] - p[0]) * u[0] + (goal[1] - p[1]) * u[1]
        cross = (p[0] - origin[0]) * normal[0] + (p[1] - origin[1]) * normal[1]
        distance = math.hypot(goal[0] - p[0], goal[1] - p[1])
        z_error = abs(p[2] - goal[2])
        final_stage = self.stage == len(self.points) - 1
        capture_remaining = (
            self.final_capture_remaining_m
            if final_stage and self.final_capture_remaining_m is not None
            else self.capture_remaining_m
        )
        capture_distance = (
            self.final_capture_distance_m
            if final_stage and self.final_capture_distance_m is not None
            else self.capture_distance_m
        )
        if self.phase == "waiting":
            self.phase = "cruise"
            self.phase_start = t
        if self.phase == "cruise":
            # Cross-track bounds are fail-closed: leaving the .6 m corridor or
            # passing the waypoint by more than .35 m ends the run instead of
            # attempting a correction.
            if abs(cross) > 0.6:
                self.failure = "route_cross_track_bound"
                return command
            if remaining < -0.35:
                self.failure = "route_waypoint_overshoot"
                return command
            # Corner capture requires all three: close along the segment, close
            # to the waypoint and on the waypoint's floor (z).
            if (
                remaining <= capture_remaining
                and distance <= capture_distance
                and z_error <= 0.18
            ):
                self.phase = "brake"
                self.phase_start = t
                self.since = None
                self.stall = None
                self.events.append(
                    {
                        "event": "entered_corner_capture",
                        "stage": self.stage,
                        "final_stage": final_stage,
                        "capture_remaining_m": capture_remaining,
                        "capture_distance_m": capture_distance,
                        "time": t,
                        "position": list(p),
                        "distance": distance,
                        "z_error": z_error,
                    }
                )
            else:
                # Forward speed is proportional to the remaining segment and
                # floored at .04 m/s, but only while the bearing error is below
                # .25 rad; otherwise forward command is zero while lateral
                # correction and yaw command are still computed.
                command[0] = (
                    min(self.speed, max(0.04, 0.5 * max(remaining, 0.0)))
                    if abs(error) < 0.25
                    else 0.0
                )
                # -.5*cross is a world-frame lateral correction along the
                # segment normal; project it onto the body left axis and bound
                # it to .05 m/s.
                correction = -0.5 * cross
                world = (correction * normal[0], correction * normal[1])
                command[1] = clip(
                    -math.sin(yaw) * world[0] + math.cos(yaw) * world[1], 0.05
                )
                command[2] = clip(0.8 * error, 0.15)
                if command[0] > 0:
                    along = (p[0] - origin[0]) * u[0] + (p[1] - origin[1]) * u[1]
                    if self.stall is None:
                        self.stall = (t, along)
                    elif t - self.stall[0] >= 8.0:
                        # An 8 s window with less than .02 m forward progress is
                        # a stall; it is always recorded, and becomes a failure
                        # only when stop_on_stall is enabled.
                        if along - self.stall[1] < 0.02:
                            self.events.append(
                                {
                                    "event": "observed_low_progress_stall",
                                    "stage": self.stage,
                                    "time": t,
                                    "window_start_s": self.stall[0],
                                    "forward_progress_m": along - self.stall[1],
                                    "position": list(p),
                                    "request": command.copy(),
                                    "stop_on_stall": self.stop_on_stall,
                                }
                            )
                            if self.stop_on_stall:
                                self.failure = "route_low_progress_stall"
                        self.stall = (t, along)
                else:
                    # Not commanding forward motion: no stall window is open.
                    self.stall = None
                return command
        if self.phase == "brake":
            # Braking from corner capture fails closed on too much waypoint
            # distance or on a brake phase that runs past 12 s.
            if distance > 0.40:
                self.failure = "corner_braking_drift"
                return command
            command[2] = clip(0.8 * error, 0.15)
            # A stop is trusted only after .5 s with the supplied speed scalar
            # below .04 on the target floor; otherwise the dwell timer resets.
            if speed < 0.04 and z_error <= 0.18:
                if self.since is None:
                    self.since = t
                if t - self.since >= 0.5:
                    self.events.append(
                        {
                            "event": "corner_stopped",
                            "stage": self.stage,
                            "time": t,
                            "position": list(p),
                            "distance": distance,
                            "z_error": z_error,
                        }
                    )
                    if self.stage == len(self.points) - 1:
                        self.arrived = True
                        self.phase = "arrived"
                        return [0.0, 0.0, 0.0]
                    self.phase = "turn"
                    self.phase_start = t
                    self.since = None
            else:
                self.since = None
            if t - self.phase_start > 12.0:
                self.failure = "corner_brake_timeout"
            return command
        if self.phase == "turn":
            # In-place turn towards the next waypoint; the pose may not drift
            # more than .45 m from the captured waypoint.
            nextgoal = self.points[self.stage + 1]
            nextheading = math.atan2(nextgoal[1] - goal[1], nextgoal[0] - goal[0])
            next_error = wrap(nextheading - yaw)
            if distance > 0.45:
                self.failure = "corner_turn_drift"
                return command
            command[2] = clip(0.8 * next_error, 0.15)
            # Turn completion requires .5 s of small bearing error plus measured
            # yaw-rate and speed-scalar stop checks; otherwise the dwell resets.
            if abs(next_error) < 0.10 and abs(omega) < 0.06 and speed < 0.04:
                if self.since is None:
                    self.since = t
                if t - self.since >= 0.5:
                    self.events.append(
                        {
                            "event": "turn_completed",
                            "stage": self.stage,
                            "time": t,
                            "position": list(p),
                            "heading_error": next_error,
                        }
                    )
                    self.stage += 1
                    self.phase = "cruise"
                    self.phase_start = t
                    self.since = None
                    self.stall = None
            else:
                self.since = None
            if t - self.phase_start > 40.0:
                self.failure = "corner_turn_timeout"
            return command
        return command
