"""Pure driving command from the existing relative-SLAM PID bridge.

State transitions, stale-pose checks and collision checks are separate layers.
This function does not publish to ROS or actuate a robot.
"""

from dataclasses import dataclass
import numpy as np
from ..planning.route import lookahead_target


@dataclass(frozen=True)
class RelativePidConfig:
    lookahead_m: float = 0.7
    velocity_damping: float = 0.10
    max_forward_mps: float = 0.5
    max_lateral_mps: float = 0.05
    max_yaw_rate_rps: float = 0.15
    heading_gain: float = 1.2
    distance_gain: float = 1.2
    turn_slow_threshold_rad: float = 0.25
    turn_stop_threshold_rad: float = 0.5
    turn_forward_cap_mps: float = 0.12
    deceleration_mps2: float = 0.25
    reaction_s: float = 0.25


def wrap_angle(value):
    """Wrap radians with atan2(sin, cos); either signed pi endpoint is possible."""
    return float(np.arctan2(np.sin(value), np.cos(value)))


def follow_segment(
    position_xy,
    rotation_xy,
    velocity_xy,
    yaw,
    segment_start,
    segment_end,
    config=RelativePidConfig(),
):
    """One bounded body-velocity command toward ``segment_end`` on a straight leg.

    Inputs are the measured XY pose/velocity/yaw in the same frame as the
    segment. Speed is capped by turn state, lookahead distance and a
    braking-distance budget; output is (command, diagnostics) with command in
    body axes [m/s, m/s, rad/s]. No state machine or collision check here.
    """
    position = np.asarray(position_xy, dtype=float)
    goal = np.asarray(segment_end, dtype=float)
    distance = float(np.linalg.norm(position - goal))
    target = lookahead_target(position, segment_start, goal, config.lookahead_m)
    error = target - position
    desired = error - config.velocity_damping * np.asarray(velocity_xy)
    desired *= min(1.0, config.max_forward_mps / max(np.linalg.norm(desired), 1e-9))
    local = np.asarray(rotation_xy).T @ desired
    heading = float(np.arctan2(error[1], error[0]))
    heading_error = wrap_angle(heading - yaw)
    cap = (
        config.turn_forward_cap_mps
        if abs(heading_error) > config.turn_slow_threshold_rad
        else config.max_forward_mps
    )
    if abs(heading_error) > config.turn_stop_threshold_rad:
        cap = 0.0
    reaction_loss = config.deceleration_mps2 * config.reaction_s
    brake_cap = max(
        0.0, -reaction_loss + np.sqrt(reaction_loss**2 + 2 * config.deceleration_mps2 * distance)
    )
    command = np.array(
        [
            min(max(local[0], 0.0), cap, config.distance_gain * distance, brake_cap),
            np.clip(local[1], -config.max_lateral_mps, config.max_lateral_mps),
            np.clip(
                config.heading_gain * heading_error,
                -config.max_yaw_rate_rps,
                config.max_yaw_rate_rps,
            ),
        ]
    )
    if abs(heading_error) > config.turn_stop_threshold_rad:
        command[:2] = 0.0
    return command, {"heading_error_rad": heading_error, "target": target.tolist()}
