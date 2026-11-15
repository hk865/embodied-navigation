#!/usr/bin/env python3
"""Explicit forward+turn waypoint tracking helper (vx/wz only, vy identically 0).

This is a pure, read-only planner: it does not touch the live tracker, perception
gates or any low-level control. The existing holonomic goto cannot satisfy a
permanently unknown side FOV because it may command vy; this helper defines a
reduced actuation mode that structurally never emits vy.

The mode is NOT wired into production. A caller must pass mode="forward_turn"
explicitly; every other value returns enabled=False and a zero command, so the
default behaviour and all original gates stay unchanged. Any future wiring must
keep the original front/staleness/obstacle/deadline/TTL gates.
"""
import math

MODE = "forward_turn"
WIRED = False
HOUSEKEEPING = "not wired: explicit mode flag required, default holonomic unchanged"

MAX_SPEED_CEILING_MPS = 1.0
MAX_YAW_RATE_RPS = 0.15


def plan_forward_turn(
    *,
    mode,
    pose_xy,
    yaw_rad,
    target_xy,
    pose_age_s,
    max_pose_age_s,
    front_clearance_m,
    front_required_m,
    speed_ceiling_mps=MAX_SPEED_CEILING_MPS,
    wz_gain=1.2,
    max_wz_rps=MAX_YAW_RATE_RPS,
    heading_gate_rad=0.35,
    arrived_radius_m=0.30,
    stalled=False,
    now_sim_s=None,
):
    """Return the bounded vx/wz command or a fail-closed hold.

    vy is always 0.0 by construction. Every guard failure returns a zero command
    with a reason; this function never relaxes or replaces a tracker gate.
    """
    result = {
        "mode": str(mode),
        "enabled": str(mode) == MODE,
        "command": [0.0, 0.0, 0.0],
        "vy": 0.0,
        "reasons": [],
        "arrived": False,
        "housekeeping": HOUSEKEEPING,
    }
    if str(mode) != MODE:
        result["reasons"].append("mode_not_enabled")
        return result
    try:
        pose = (float(pose_xy[0]), float(pose_xy[1]))
        target = (float(target_xy[0]), float(target_xy[1]))
        yaw = float(yaw_rad)
        age = float(pose_age_s)
        max_age = float(max_pose_age_s)
        front = None if front_clearance_m is None else float(front_clearance_m)
        required = float(front_required_m)
        ceiling = float(speed_ceiling_mps)
        gain = float(wz_gain)
        wz_max = float(max_wz_rps)
        gate = float(heading_gate_rad)
    except (TypeError, ValueError, IndexError):
        result["reasons"].append("invalid_input")
        return result
    values = [*pose, *target, yaw, age, max_age, ceiling, gain, wz_max, gate]
    if front is not None:
        values.append(front)
    if required is not None:
        values.append(required)
    if not all(math.isfinite(value) for value in values):
        result["reasons"].append("non_finite_input")
        return result
    if max_age <= 0.0 or ceiling <= 0.0 or wz_max <= 0.0 or gate <= 0.0:
        result["reasons"].append("invalid_limit")
        return result
    if age < 0.0 or age > max_age:
        result["reasons"].append("pose_stale_or_future")
        return result
    if now_sim_s is not None and not math.isfinite(float(now_sim_s)):
        result["reasons"].append("clock_invalid")
        return result
    if stalled:
        result["reasons"].append("stalled")
        return result
    dx, dy = target[0] - pose[0], target[1] - pose[1]
    distance = math.hypot(dx, dy)
    if distance <= float(arrived_radius_m):
        result["arrived"] = True
        result["reasons"].append("arrived")
        return result
    if front is None or front < required:
        result["reasons"].append("front_clearance_insufficient")
        result["front_clearance_m"] = front
        return result
    bearing = math.atan2(dy, dx)
    error = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
    wz = max(-wz_max, min(wz_max, gain * error))
    heading_ok = abs(error) <= gate
    speed = 0.0
    if heading_ok:
        speed = max(0.0, min(ceiling, ceiling * (1.0 - abs(error) / gate)))
    result.update(
        {
            "command": [float(speed), 0.0, float(wz)],
            "distance_m": distance,
            "bearing_rad": bearing,
            "heading_error_rad": error,
            "heading_ok": heading_ok,
            "gates": {
                "pose_age_ok": True,
                "front_clearance_ok": True,
                "front_clearance_m": front,
                "front_required_m": required,
            },
        }
    )
    if not heading_ok:
        result["reasons"].append("turning_in_place")
    return result
