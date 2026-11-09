"""Pure actual-SLAM/IMU/SCAN cascade. Returns bounded velocity, never actions.

All times are original integer nanoseconds supplied by the wrapper. No ROS,
simulator, Actor, clock sampling, or estimated Teacher slew is available here.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import numpy as np
from embodied_navigation.control.parking.ordinary_parking import OrdinaryParking
from embodied_navigation.control.parking.parking_zero_contract import validate_zero_ack
from embodied_navigation.planning.reference.spatial_reference import (
    configuration as spatial_configuration,
    reference as spatial_direction,
    speed_budget,
)


SCHEMA = "actual_slam_scan_header_event_cascade/v1"
DEFAULT_GAINS = dict(
    cross_kp=0.8,
    cross_kd=0.2,
    yaw_kp=1.3,
    yaw_kd=0.15,
    velocity_kp_x=0.6,
    velocity_ki_x=0.5,
    velocity_kp_y=0.4,
    velocity_ki_y=0.3,
    rate_kp=0.3,
    rate_ki=0.2,
)
DEFAULT_COM = [0.05523092034548942, -0.001869001919385796, 0.006095299184261034]


def neutral_rate_brake_capture_configuration(profile):
    """Explicit profile extension; disabled/absent preserves the original cascade."""
    if not isinstance(profile, dict):
        raise ValueError("Neutral-rate brake capture requires the actual profile")
    if "r85_neutral_rate_brake_capture" not in profile:
        return None
    value = profile["r85_neutral_rate_brake_capture"]
    if value == {"enabled": False} and type(value["enabled"]) is bool:
        return None
    expected = dict(enabled=True, kind="neutral_rate_with_original_heading_band_v1")
    if (
        not isinstance(value, dict)
        or set(value) != set(expected)
        or value != expected
        or type(value["enabled"]) is not bool
        or type(value["kind"]) is not str
    ):
        raise ValueError("Exact explicit neutral-rate brake capture configuration required")
    for key in ("r85_active_turn_braking", "r85_quiet_handoff_target"):
        setting = profile.get(key)
        if (
            not isinstance(setting, dict)
            or set(setting) != {"enabled"}
            or type(setting["enabled"]) is not bool
            or setting["enabled"] is not True
        ):
            raise ValueError("Neutral-rate capture requires original active quiet braking")
    cascade = profile.get("cascade")
    if (
        not isinstance(cascade, dict)
        or cascade.get("external_heading_gate") is not True
        or cascade.get("r85_turn_brake_reference") != {"kind": "locked_heading_pd_v1"}
    ):
        raise ValueError("Neutral-rate capture requires the original locked-heading cascade")
    from embodied_navigation.control.parking.integral_unload import (
        configuration as unload_configuration,
    )

    if unload_configuration(profile) is None:
        raise ValueError("Neutral-rate capture requires the original coordinated unload event")
    return copy.deepcopy(value)


def teacher_pre_zero_heading_p_taper_configuration(profile):
    """Explicit R43 heading-P law; original gains remain the outside-event base."""
    if not isinstance(profile, dict):
        raise ValueError("Actual full profile required for Teacher heading-P taper")
    key = "r85_teacher_pre_zero_heading_p_taper"
    if key not in profile:
        return None
    value = profile[key]
    if isinstance(value, dict) and set(value) == {"enabled"} and value["enabled"] is False:
        return None
    expected = dict(
        enabled=True,
        kind="accepted_pre_zero_event_positive_heading_P_taper_v1",
        base_kp=1.3,
        target_kp=0.9,
        duration_s=1.0,
    )
    if (
        not isinstance(value, dict)
        or set(value) != set(expected)
        or value != expected
        or value["enabled"] is not True
        or type(value["kind"]) is not str
        or any(
            type(value[k]) not in (int, float) or not math.isfinite(value[k])
            for k in ("base_kp", "target_kp", "duration_s")
        )
    ):
        raise ValueError("Exact selected Teacher positive heading-P taper configuration required")
    for key, required in (
        ("r85_active_turn_braking", True),
        ("r85_quiet_handoff_target", True),
        ("r85_neutral_rate_brake_capture", False),
    ):
        setting = profile.get(key)
        if (
            not isinstance(setting, dict)
            or set(setting) != {"enabled"}
            or setting["enabled"] is not required
        ):
            raise ValueError(
                "Teacher taper requires original active quiet locked-heading brake and neutral off"
            )
    cascade = profile.get("cascade")
    if (
        not isinstance(cascade, dict)
        or cascade.get("external_heading_gate") is not True
        or cascade.get("r85_turn_brake_reference") != {"kind": "locked_heading_pd_v1"}
    ):
        raise ValueError("Teacher taper requires the original external locked-heading cascade")
    from embodied_navigation.control.parking.integral_unload import (
        configuration as unload_configuration,
    )

    if unload_configuration(profile) is None:
        raise ValueError("Teacher taper requires the existing owned unload event")
    return copy.deepcopy(value)


def turn_brake_reference_configuration(value):
    """Return the reviewed turn-brake reference kind, or raise on anything else.

    Only ``pure_zero_euler_rate_v1`` (original cascade) and
    ``locked_heading_pd_v1`` (explicit external heading ownership) are allowed.
    """
    if (
        not isinstance(value, dict)
        or set(value) != {"kind"}
        or type(value.get("kind")) is not str
        or value["kind"] not in ("pure_zero_euler_rate_v1", "locked_heading_pd_v1")
    ):
        raise ValueError("Exact explicit reviewed turn-brake reference kind required")
    return value["kind"]


PARKING = dict(
    capture_position_kp=0.8,
    capture_position_kd=0.2,
    capture_reference_xy_norm_limit_mps=0.05,
    capture_command_limits_body=[0.15, 0.07, 0.10],
    position_kp=0.18,
    position_kd=0.2,
    reference_xy_norm_limit_mps=0.025,
    command_limits_body=[0.06, 0.04, 0.10],
    yaw_kp=0.65,
    yaw_kd=0.18,
    reference_yaw_limit_radps=0.07,
    position_deadband_m=0.003,
    yaw_deadband_rad=0.005,
    capture_xy_entry_m=0.025,
    capture_xy_hold_m=0.03,
    capture_yaw_entry_rad=0.035,
    capture_yaw_hold_rad=0.045,
    capture_actual_xy_speed_mps=0.03,
    capture_actual_yawrate_radps=0.06,
    capture_fresh_dwell_s=0.6,
    first_fixed_parking_window_s=5.0,
    maximum_xy_drift_m=0.05,
    maximum_yaw_drift_rad=0.1,
    maximum_native_xy_speed_mps=0.08,
    maximum_native_Euler_yawrate_radps=0.1,
    maximum_native_body_wz_radps=0.1,
)


def canonical(value):
    """Stable compact JSON for identity hashes (sorted keys, no NaN/Infinity)."""
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def sha(value):
    """SHA256 of ``canonical(value)``; used as evidence identity, not security."""
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def wrap(angle):
    """Wrap radians with atan2(sin, cos); either signed pi endpoint is possible."""
    return math.atan2(math.sin(angle), math.cos(angle))


def vector(value, n):
    """Return a finite float64 copy of an ``n``-vector; wrong shape/NaN raises."""
    result = np.asarray(value, float)
    if result.shape != (n,) or not np.isfinite(result).all():
        raise ValueError("Invalid finite vector")
    return result.copy()


def integer(value):
    """Return one original nonnegative integer stamp/sequence (bools rejected)."""
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < 0:
        raise ValueError("Nonnegative original integer timestamp required")
    return int(value)


def rotation(quaternion_wxyz):
    """Build the 3x3 body rotation from a measured wxyz quaternion.

    The squared norm must lie in [0.98, 1.02]. Accepted inputs are normalized
    before the standard quaternion-to-matrix conversion.
    """
    q = vector(quaternion_wxyz, 4)
    if not 0.98 <= float(q @ q) <= 1.02:
        raise ValueError("Invalid actual body quaternion")
    w, x, y, z = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def deadband(value, width):
    """Signed deadband: keep the sign but subtract ``width`` from the magnitude."""
    return np.sign(value) * np.maximum(abs(value) - width, 0.0)


def bounded(raw, limits, forward_only=False):
    """Clip a 3-vector to per-axis ``limits`` and to the XY speed circle.

    ``forward_only`` additionally forbids a negative forward command. The XY
    circle radius is ``limits[0]`` (the longitudinal bound, m/s), matching the
    original cascade's combined forward/lateral envelope.
    """
    out = np.clip(raw, -limits, limits)
    if forward_only:
        out[0] = max(0.0, out[0])
    norm = np.linalg.norm(out[:2])
    if norm > limits[0]:
        out[:2] *= limits[0] / norm
    return out


class Controller:
    """One fixed goal, replaceable checked SCAN paths, source-header math.

    Ack is a causal *actual Teacher velocity input*, not observed velocity.
    Anti-windup uses its requested-minus-applied residual; this core never
    guesses how many actuator frames ran and never performs final slew.
    """

    def __init__(self, config, fixed_goal_xyz, fixed_goal_yaw_rad, goal_id):
        self.config = copy.deepcopy(config)
        # Set only by the full-profile simulation-only entry in controller.py.
        self.minimal_simulation_baseline = False
        self.minimal_baseline_observation = {}
        self.neutral_rate_brake_capture_config = None
        self.teacher_pre_zero_heading_p_taper_config = None
        from embodied_navigation.robot.unitree.policy.lower_loop_ack import (
            configuration as lower_ack_configuration,
        )

        self.student_lower_ack_config = lower_ack_configuration(
            config.get("r85_student_lower_loop_ack")
        )
        self.spatial_config = spatial_configuration(config.get("spatial_reference"))
        self.g = copy.deepcopy(DEFAULT_GAINS)
        if config.get("gains", self.g) != self.g:
            raise ValueError("This candidate preserves the verified cascade gains")
        self.speed = float(config.get("desired_speed", 0.3))
        self.external_heading_gate = config.get("external_heading_gate", False)
        if not isinstance(self.external_heading_gate, bool):
            raise ValueError("Heading gate ownership must be explicit")
        self.turn_brake_reference_kind = turn_brake_reference_configuration(
            config.get("r85_turn_brake_reference", {"kind": "pure_zero_euler_rate_v1"})
        )
        if "r85_turn_brake_reference" in config and self.external_heading_gate is not True:
            raise ValueError("Explicit turn-brake reference requires external heading ownership")
        self.limits = vector(config.get("command_limits", [0.8, 0.35, 0.8]), 3)
        if (
            not 0 < self.speed <= 1.0
            or np.any(self.limits <= 0)
            or np.any(self.limits > [1.0, 0.4, 1.0])
        ):
            raise ValueError("Command outside archived Teacher boundary")
        self.com_offset = vector(config.get("base_com_offset", DEFAULT_COM), 3)
        if np.linalg.norm(self.com_offset) > 0.2:
            raise ValueError("Invalid body COM offset")
        self.ttl = 0.3
        self.gyro_gap = float(config.get("maximum_gyro_pair_gap_s", 0.02))
        self.goal_z_tolerance = float(config.get("goal_z_tolerance_m", 0.2))
        if not 0 < self.gyro_gap <= 0.05 or not 0 < self.goal_z_tolerance <= 0.3:
            raise ValueError("Unsafe source or floor boundary")
        self.goal = vector(fixed_goal_xyz, 3)
        self.goal_yaw = float(fixed_goal_yaw_rad)
        if not math.isfinite(self.goal_yaw) or not isinstance(goal_id, str) or not goal_id:
            raise ValueError("Explicit immutable map goal required")
        self.goal_id = goal_id
        self.goal_record = dict(
            position_world_xyz=self.goal.tolist(),
            heading_rad=self.goal_yaw,
            goal_id=goal_id,
            frame_id="camera_init",
        )
        self.goal_sha = sha(self.goal_record)
        self.terminal_reference = None
        self.path = None
        self.path_id = None
        self.path_sha = None
        self.path_history = {}
        self.path_handoff = None
        self.last_projection_position = None
        self.last_projection_tangent = None
        self.last_reference_speed = None
        self.last_curvature_yaw_reference = 0.0
        self.progress = 0.0
        self.new_path = True
        self.replan_count = 0
        self.command = np.zeros(3)
        self.velocity_integral = np.zeros(3)
        self.hold_integral = np.zeros(3)
        self.filtered = None
        self.inner_filtered = None
        self.last_math_stamp = None
        self.last_math_wall = None
        self.last_seen_pose_stamp = None
        self.last_pose_hash = None
        self.last_clock = None
        self.last_wall = None
        self.last_ack_sequence = None
        self.last_ack_hash = None
        self.last_student_lower_ack_binding = None
        self.protected = True
        self.failure_latched = None
        self.protection_reason = "initializing"
        self.recovery = []
        self.turn_phase = None
        self.turn_dwell = None
        self.capture_stamp = None
        self.capture_wall = None
        self.capture_clock = None
        self.arrival = None
        self.capture_dwell = None
        self.hold_stamp = None
        self.hold_wall = None
        self.hold_clock = None
        self.parking_interrupted = False
        self.row = {}
        self.ordinary_parking = OrdinaryParking()
        self.ordinary_parking.daily_navigation = config.get("daily_navigation", False)
        self.ordinary_parking.configure_pose10hz(config.get("parking_pose10hz"))
        self.ordinary_parking.capture_goal = self.goal.tolist()
        self.ordinary_parking.capture_heading = self.goal_yaw
        self.ordinary_reset_pending = False
        self.ordinary_reset_receipt = None
        self.ordinary_zero_event = None

    def configure_terminal_reference(self, record):
        from embodied_navigation.control.parking.terminal_reference import validate_reference

        validate_reference(record)
        if (
            self.path is not None
            or self.terminal_reference is not None
            or self.last_clock is not None
            or record["fixed_goal_sha256"] != self.goal_sha
            or record["fixed_goal"] != self.goal_record
        ):
            raise ValueError("Terminal reference requires a fresh fixed-goal core without a path")
        self.terminal_reference = copy.deepcopy(record)
        self.path_record = None

    def set_path(
        self, points_xyz, path_id, stamp_ns, received_wall_ns, frame_id="camera_init", position=None
    ):
        """Accept one actually checked SCAN ID. Generation time is provenance,
        not a 300ms path TTL; the wrapper continuously checks cloud/guard health.
        """
        if self.terminal_reference is not None:
            raise ValueError("Terminal-only owner cannot accept SCAN replans")
        points = np.asarray(points_xyz, float).copy()
        stamp, wall = integer(stamp_ns), integer(received_wall_ns)
        if (
            frame_id != "camera_init"
            or not isinstance(path_id, str)
            or not path_id
            or points.ndim != 2
            or points.shape[1] != 3
            or len(points) < 2
            or not np.isfinite(points).all()
        ):
            raise ValueError("Finite registered directed SCAN path required")
        delta = np.diff(points, axis=0)
        lengths = np.linalg.norm(delta[:, :2], axis=1)
        if np.any(lengths < 1e-5):
            raise ValueError("Vertical/degenerate segment is not a ramp path")
        record = dict(
            points_xyz=points.tolist(),
            path_id=path_id,
            stamp_ns=stamp,
            received_wall_ns=wall,
            frame_id=frame_id,
        )
        digest = sha(record)
        if path_id in self.path_history:
            if digest != self.path_history[path_id]:
                raise ValueError("An accepted SCAN path ID is immutable")
            return False
        if self.capture_stamp is not None:
            raise ValueError("Fixed endpoint capture cannot accept a new path")
        # Map a real SLAM pose onto the new path in xyz. Arc coordinates from
        # different replans are unrelated; neither command nor measured speed
        # is fabricated to make their starts agree. Direction breaks exact
        # geometric ties only, never overrides the nearest 3D/floor distance.
        handoff_position = (
            vector(position, 3)
            if position is not None
            else None
            if self.last_projection_position is None
            else self.last_projection_position.copy()
        )
        cumulative = np.r_[0.0, np.cumsum(lengths)]
        handoff = None
        if self.path is not None and handoff_position is not None:
            candidates = []
            for i, (a, d, length) in enumerate(zip(points[:-1], delta, lengths)):
                u = float(np.clip((handoff_position - a) @ d / (d @ d), 0.0, 1.0))
                point = a + u * d
                distance2 = float((handoff_position - point) @ (handoff_position - point))
                tangent = d[:2] / length
                alignment = (
                    0.0
                    if self.last_projection_tangent is None
                    else float(tangent @ self.last_projection_tangent)
                )
                candidates.append(
                    (distance2, -alignment, float(cumulative[i] + u * length), i, point)
                )
            minimum = min(c[0] for c in candidates)
            chosen = min(
                (c for c in candidates if c[0] <= minimum + 1e-12), key=lambda c: (c[1], c[2], c[3])
            )
            handoff = dict(
                source="provided_actual_SLAM_pose"
                if position is not None
                else "last_committed_actual_SLAM_pose",
                position_xyz=handoff_position.tolist(),
                projected_xyz=chosen[4].tolist(),
                segment=chosen[3],
                mapped_horizontal_arc_m=chosen[2],
                projection_distance_m=math.sqrt(chosen[0]),
                previous_path_id=self.path_id,
                new_path_id=path_id,
                commands_and_filters_preserved=True,
            )
        self.path, self.path_delta, self.lengths = points, delta, lengths
        self.cumulative = cumulative
        self.path_record, self.path_id, self.path_sha = record, path_id, digest
        self.path_history[path_id] = digest
        self.path_handoff = handoff
        self.progress = 0.0 if handoff is None else handoff["mapped_horizontal_arc_m"]
        self.new_path = handoff is None
        self.replan_count += 1
        # A replan is not a new velocity experiment: preserve PI and filters.
        return True

    def _project(self, position):
        # Nearest in xyz disambiguates overlapping floor xy. Horizontal arc
        # distance parameter gives signed horizontal cross and dz/ds grade.
        low = 0.0 if self.new_path else max(0.0, self.progress - 0.15)
        high = (
            self.cumulative[-1] if self.new_path else min(self.cumulative[-1], self.progress + 0.8)
        )
        best = None
        for i, (a, d, length) in enumerate(zip(self.path[:-1], self.path_delta, self.lengths)):
            if self.cumulative[i + 1] < low or self.cumulative[i] > high:
                continue
            u0 = max(0.0, (low - self.cumulative[i]) / length)
            u1 = min(1.0, (high - self.cumulative[i]) / length)
            u = float(np.clip((position - a) @ d / (d @ d), u0, u1))
            point = a + u * d
            along = self.cumulative[i] + u * length
            distance2 = float((position - point) @ (position - point))
            candidate = (distance2, float(along), i, point)
            if best is None or candidate[:3] < best[:3]:
                best = candidate
        _, nearest_s, i, point = best
        self.progress = max(self.progress, nearest_s)
        self.new_path = False
        tangent = self.path_delta[i, :2] / self.lengths[i]
        normal = np.array([-tangent[1], tangent[0]])
        cross = float((position[:2] - point[:2]) @ normal)
        self.last_projection_position = np.asarray(position, float).copy()
        self.last_projection_tangent = tangent.copy()
        return (
            i,
            point,
            tangent,
            normal,
            cross,
            float(self.path_delta[i, 2] / self.lengths[i]),
            nearest_s,
        )

    def _reference_from_projection(self, position, projection):
        segment, point, tangent, normal, cross, grade, nearest_s = projection
        result = spatial_direction(
            self.path, self.cumulative, nearest_s, position, point, tangent, self.spatial_config
        )
        remaining = float(self.cumulative[-1] - self.progress)
        endpoint_is_goal = bool(np.linalg.norm(self.path[-1] - self.goal) <= 0.2)
        result.update(
            nearest_s_m=float(nearest_s),
            remaining_m=remaining,
            endpoint_is_fixed_goal=endpoint_is_goal,
            path_id=self.path_id,
            path_sha256=self.path_sha,
            path_handoff=copy.deepcopy(self.path_handoff),
            raw_projection=dict(
                segment=int(segment),
                point_xyz=point.tolist(),
                tangent_xy=tangent.tolist(),
                normal_xy=normal.tolist(),
                cross_m=float(cross),
                grade_dz_ds=float(grade),
            ),
            heading_source="finite_horizontal_arc"
            if result["fallback_reason"] is None
            else "original_segment_fallback",
            actual_motion_guard_required=True,
        )
        if endpoint_is_goal and remaining <= 0.15:
            result.update(
                heading_rad=self.goal_yaw,
                heading_source="fixed_goal_tail",
                curvature_valid=False,
                curvature_per_m=0.0,
                curvature_rate_per_m2=0.0,
            )
        return result

    def preview_reference(self, position):
        """Same heading as update, without progress/filter/integral mutation."""
        position = vector(position, 3)
        if self.path is None:
            raise ValueError("No checked SCAN path")
        preview = copy.deepcopy(self)
        return preview._reference_from_projection(position, preview._project(position))

    def begin_ordinary_terminal(self, clock_ns, timeout_ns):
        if self.ordinary_parking.begin(integer(clock_ns), integer(timeout_ns)):
            self.ordinary_reset_pending = True
            self.capture_dwell = self.hold_stamp = self.hold_clock = self.hold_wall = None
            return True
        return False

    def reset_control_memory(self, reason, measurement=None, actual=None):
        """Clear real control memory; source/clock/ACK high-water marks survive."""
        self.velocity_integral[:] = 0.0
        self.hold_integral[:] = 0.0
        self.command[:] = 0.0
        self.filtered = None if measurement is None else np.asarray(measurement, float).copy()
        self.inner_filtered = None if actual is None else np.asarray(actual, float).copy()
        self.last_reference_speed = None
        self.last_curvature_yaw_reference = 0.0
        self.turn_phase = self.turn_dwell = None
        self.ordinary_reset_receipt = dict(
            reason=reason,
            velocity_integral=[0.0, 0.0, 0.0],
            hold_integral=[0.0, 0.0, 0.0],
            filters="cleared" if measurement is None else "seeded_from_current_real_measurement",
            previous_control_command=[0.0, 0.0, 0.0],
            source_high_water_preserved=True,
        )

    def _protect(self, reason, fatal=False):
        self.ordinary_parking.guard(reason)
        if (
            self.ordinary_parking.failure
            and not self.ordinary_parking.diagnostic_failure_is_observed(
                getattr(self, "diagnostic_observe_only", None)
            )
        ):
            self.failure_latched = self.failure_latched or self.ordinary_parking.failure
        if fatal:
            self.failure_latched = reason
        # Any newly observed protection invalidates a partially seeded recovery.
        # Repeated identical reasons must not preserve a pre-guard time anchor.
        self.velocity_integral[:] = 0.0
        self.hold_integral[:] = 0.0
        self.filtered = self.inner_filtered = None
        self.last_math_stamp = self.last_math_wall = None
        self.last_reference_speed = None
        self.last_curvature_yaw_reference = 0.0
        self.turn_phase = self.turn_dwell = self.capture_dwell = None
        self.recovery = []
        if self.hold_stamp is not None:
            self.parking_interrupted = True
        self.protected = True
        self.protection_reason = reason
        self.command[:] = 0.0

    def _output(self, mode, clock, wall, updated=False, reason=None):
        row = copy.deepcopy(self.row)
        row.update(
            schema=SCHEMA,
            controller_updated=updated,
            mode=mode,
            reason=reason,
            compute_clock_ns=clock,
            compute_wall_ns=wall,
            command_body=self.command.tolist(),
            path_id=self.path_id,
            path_sha256=self.path_sha,
            replan_count=self.replan_count,
            fixed_goal=copy.deepcopy(self.goal_record),
            fixed_goal_sha256=self.goal_sha,
            progress_m=self.progress,
            feedback_source="actual SLAM body-origin odometry + causal actual IMU",
            navigation_ground_truth_used=False,
            actual_SLAM_SCAN_verified=False,
            final_slew_performed_here=False,
            parking_is_old_zero_command_protocol=False,
            parking_capture_pose_stamp_ns=self.capture_stamp,
            parking_capture_clock_ns=self.capture_clock,
            parking_capture_wall_ns=self.capture_wall,
            parking_hold_declared_pose_stamp_ns=self.hold_stamp,
            parking_hold_declared_clock_ns=self.hold_clock,
            parking_hold_declared_wall_ns=self.hold_wall,
            first_fixed_parking_window_s=5.0,
            parking_window_interrupted=self.parking_interrupted,
            arrival_capture_pose=copy.deepcopy(self.arrival),
            protection_active=self.protected,
        )
        row["failure_latched"] = self.failure_latched
        if self.minimal_simulation_baseline:
            row["r73_minimal_diagnostic_baseline"] = dict(
                self.minimal_baseline_observation,
                enabled=True,
                simulation_only=True,
                formal_acceptance_pass=False,
            )
        if self.terminal_reference is not None:
            from embodied_navigation.control.parking.terminal_reference import fields

            row.update(
                fields(self.terminal_reference),
                path_geometry_present=False,
                schema="actual_slam_terminal_fixed_goal_cascade/v1",
            )
        row["ordinary_parking"] = self.ordinary_parking.evidence()
        row["ordinary_parking_stage"] = self.ordinary_parking.stage
        row["ordinary_control_memory_reset"] = copy.deepcopy(self.ordinary_reset_receipt)
        row["ordinary_terminal_zero_event"] = copy.deepcopy(self.ordinary_zero_event)
        self.row = copy.deepcopy(row)
        return self.command.copy(), row

    def configure_neutral_rate_brake_capture(self, profile):
        """Configure once at construction, before any source-clock/control event."""
        setting = neutral_rate_brake_capture_configuration(profile)
        if self.last_clock is not None:
            raise ValueError(
                "Neutral-rate capture configuration cannot change after a control event"
            )
        if setting is not None and (
            profile.get("cascade") != self.config
            or not self.external_heading_gate
            or self.turn_brake_reference_kind != "locked_heading_pd_v1"
        ):
            raise ValueError(
                "Neutral-rate capture must bind the actual original locked-heading cascade"
            )
        self.neutral_rate_brake_capture_config = setting

    def configure_teacher_pre_zero_heading_p_taper(self, profile, run_id=None, scope=None):
        """Bind exact new math before the first control clock; no live retuning."""
        setting = teacher_pre_zero_heading_p_taper_configuration(profile)
        if self.last_clock is not None:
            raise ValueError("Teacher heading-P taper cannot change after a control event")
        if setting is not None and (
            profile.get("cascade") != self.config
            or not self.external_heading_gate
            or self.turn_brake_reference_kind != "locked_heading_pd_v1"
        ):
            raise ValueError("Teacher heading-P taper must bind the actual locked-heading cascade")
        if setting is not None:
            from embodied_navigation.control.parking.integral_unload import valid_sha

            if not isinstance(run_id, str) or not run_id or not valid_sha(scope):
                raise ValueError("Teacher heading-P taper requires the actual run and source scope")
        self.teacher_pre_zero_heading_p_taper_config = setting
        self.teacher_pre_zero_heading_p_taper_run_id = run_id
        self.teacher_pre_zero_heading_p_taper_scope = scope

    def _fresh(self, source, clock, wall):
        stamp, receipt = integer(source["stamp_ns"]), integer(source["received_wall_ns"])
        if not 0 <= clock - stamp <= 300_000_000 or wall - receipt < 0:
            raise ValueError("Source stale or future")
        if wall - receipt > 300_000_000:
            if not self.minimal_simulation_baseline:
                raise ValueError("Source stale or future")
            self.minimal_baseline_observation["source_wall_age_would_reject"] = True
            self.minimal_baseline_observation["maximum_source_wall_age_ns"] = max(
                self.minimal_baseline_observation.get("maximum_source_wall_age_ns", 0),
                wall - receipt,
            )
        return stamp, receipt

    def _pi(
        self,
        target,
        actual,
        dt,
        integral,
        limits,
        ack,
        forward_only,
        turn_only=False,
        pre_zero_unload=None,
    ):
        kp = np.array([0.6, 0.4, 0.3])
        ki = np.array([0.5, 0.3, 0.2])
        error = target - actual
        candidate = np.clip(integral + dt * error, -0.5, 0.5)
        if turn_only:
            candidate[:2] = integral[:2]
        unload_before = None
        if pre_zero_unload is not None:
            from embodied_navigation.control.parking.integral_unload import toward_zero

            unload_before = float(integral[2])
            candidate[2] = toward_zero(unload_before, 0.1, dt)
        raw = target + kp * error + ki * candidate
        limited = bounded(raw, limits, forward_only)
        blocked_limits = error * (raw - limited) > 1e-9
        # The acknowledged request and applied input belong to the SAME actual
        # Teacher frame. Compare that residual, not a new request with old input.
        residual = vector(ack["requested_command_body"], 3) - vector(ack["applied_command_body"], 3)
        lower_ack_diagnostic = None
        if self.student_lower_ack_config is not None:
            from embodied_navigation.robot.unitree.policy.lower_loop_ack import (
                residual as lower_ack_residual,
            )

            residual, lower_ack_diagnostic = lower_ack_residual(
                ack, self.student_lower_ack_config, self.limits
            )
        blocked_ack = error * residual > 1e-9
        candidate = np.where(blocked_limits | blocked_ack, integral, candidate)
        if unload_before is not None:
            candidate[2] = toward_zero(unload_before, 0.1, dt)
        raw = target + kp * error + ki * candidate
        limited = bounded(raw, limits, forward_only)
        terms = dict(
            error=error.tolist(),
            P=(kp * error).tolist(),
            I=(ki * candidate).tolist(),
            integral_state=candidate.tolist(),
            integral_dt_s=dt,
            filtered_actual_body=actual.tolist(),
            blocked_limits=blocked_limits.tolist(),
            blocked_actual_ack=blocked_ack.tolist(),
            acknowledged_downstream_residual=residual.tolist(),
            ack_sequence=int(ack["sequence"]),
            ack_stamp_ns=int(ack["stamp_ns"]),
        )
        if lower_ack_diagnostic is not None:
            terms["student_lower_loop_ack"] = lower_ack_diagnostic
        terms["translation_integral_frozen"] = bool(turn_only)
        if unload_before is not None:
            from embodied_navigation.control.parking.integral_unload import receipt

            terms["pre_zero_integral_unload"] = receipt(
                pre_zero_unload,
                "teacher",
                unload_before,
                candidate[2],
                dt,
                self.last_clock,
                self.path_sha,
                self.goal_sha,
            )
        return limited, raw, candidate, terms

    def update(
        self,
        feedback,
        imu,
        ack,
        clock_ns,
        wall_ns,
        guard_reason=None,
        mode_override=None,
        external_turn_heading=None,
        pre_zero_unload_event=None,
    ):
        """Return (requested velocity ndarray[3], independent diagnostic dict).

        mode_override: None/drive = geometric stop-turn-drive; turn = suppress
        translation; brake = zero Euler-rate reference with original PI;
        hold = protected zero; capture = bounded fixed-goal hold.
        Wrapper must reevaluate safety against the resulting actual direction.
        external_turn_heading: optional locked external-gate reference, used
        only for externally owned turn mode before parking. Drive continues
        following the checked SCAN path; omitted input preserves prior math.
        """
        clock, wall = integer(clock_ns), integer(wall_ns)
        if self.minimal_simulation_baseline:
            self.minimal_baseline_observation = {}
        if mode_override not in (None, "drive", "turn", "brake", "hold", "capture"):
            raise ValueError("Unknown explicit control mode")
        if (self.last_clock is not None and clock < self.last_clock) or (
            self.last_wall is not None and wall < self.last_wall
        ):
            guard_reason = "clock_backwards"
        self.last_clock, self.last_wall = clock, wall
        terminal = self.terminal_reference
        if terminal is not None and mode_override not in ("capture", "hold"):
            guard_reason = "terminal_reference_forbids_route_motion"
        if self.failure_latched or guard_reason or mode_override == "hold":
            reason = str(self.failure_latched or guard_reason or "external_hold")
            self._protect(reason, fatal=reason == "clock_backwards")
            return self._output("protect", clock, wall, reason=self.protection_reason)
        external_turn_heading_value = None
        neutral_rate_capture = None
        teacher_heading_p_taper = None
        try:
            if (
                self.neutral_rate_brake_capture_config is not None
                and mode_override == "brake"
                and pre_zero_unload_event is None
            ):
                raise ValueError("Neutral-rate capture requires the actual authorized unload event")
            if pre_zero_unload_event is not None:
                from embodied_navigation.control.parking.integral_unload import validate_event

                validate_event(pre_zero_unload_event, clock_ns=clock)
                if self.teacher_pre_zero_heading_p_taper_config is not None:
                    validate_event(
                        pre_zero_unload_event,
                        self.teacher_pre_zero_heading_p_taper_run_id,
                        self.teacher_pre_zero_heading_p_taper_scope,
                        clock,
                    )
                if (
                    mode_override != "brake"
                    or not self.external_heading_gate
                    or self.capture_stamp is not None
                    or pre_zero_unload_event["goal_sha256"] != self.goal_sha
                ):
                    raise ValueError("Coordinated unload outside original owned brake")
                old = getattr(self, "pre_zero_unload_event", None)
                if old is not None and old != pre_zero_unload_event:
                    from embodied_navigation.control.parking.integral_unload import successor

                    if not successor(old, pre_zero_unload_event):
                        raise ValueError("Outer unload pending event cannot renew")
                self.pre_zero_unload_event = copy.deepcopy(pre_zero_unload_event)
            if mode_override == "brake" and (
                not self.external_heading_gate or self.capture_stamp is not None
            ):
                raise ValueError(
                    "Active turn braking requires external heading ownership and no capture"
                )
            if (
                mode_override == "brake"
                and self.turn_brake_reference_kind == "locked_heading_pd_v1"
                and external_turn_heading is None
            ):
                raise ValueError(
                    "Locked-heading braking requires the explicit external locked heading"
                )
            if terminal is not None:
                from embodied_navigation.control.parking.terminal_reference import (
                    validate_reference,
                )

                validate_reference(terminal)
                if (
                    self.path is not None
                    or self.path_id is not None
                    or self.path_sha is not None
                    or terminal["fixed_goal_sha256"] != self.goal_sha
                    or terminal["fixed_goal"] != self.goal_record
                    or not self.ordinary_parking.enabled
                    or clock >= terminal["deadline_clock_ns"]
                ):
                    raise ValueError("Terminal reference identity/owner/deadline invalid")
            elif self.path is None:
                raise ValueError("No checked SCAN path")
            if (
                self.external_heading_gate
                and mode_override in ("turn", "brake")
                and self.capture_stamp is None
                and external_turn_heading is not None
            ):
                if (
                    isinstance(external_turn_heading, bool)
                    or not isinstance(external_turn_heading, (int, float, np.integer, np.floating))
                    or not math.isfinite(float(external_turn_heading))
                ):
                    raise ValueError("Invalid finite external turn heading")
                external_turn_heading_value = float(external_turn_heading)
            stamp, receipt = self._fresh(feedback, clock, wall)
            gyro_stamp, _ = self._fresh(imu, clock, wall)
            ack_stamp, _ = self._fresh(ack, clock, wall)
            if feedback.get("frame_id") != "camera_init" or imu.get("frame_id") != "body":
                raise ValueError("Explicit map pose and body gyro frames required")
            if not 0 <= stamp - gyro_stamp <= round(self.gyro_gap * 1e9) or ack_stamp > stamp:
                raise ValueError("IMU or executed command is not causal to pose")
            if (
                terminal["source_pose_stamp_ns"]
                if terminal is not None
                else self.path_record["stamp_ns"]
            ) > stamp:
                raise ValueError("SCAN path generation is future to this pose")
            position = vector(feedback["position_world_xyz"], 3)
            R = rotation(feedback["quaternion_wxyz"])
            origin_body = vector(feedback["origin_velocity_body"], 3)
            gyro = vector(imu["angular_velocity_body"], 3)
            roll = math.atan2(R[2, 1], R[2, 2])
            pitch = math.asin(float(np.clip(-R[2, 0], -1, 1)))
            yaw = math.atan2(R[1, 0], R[0, 0])
            if (
                max(abs(roll), abs(pitch)) > 0.65
                or abs(math.cos(pitch)) < 0.4
                or abs(math.cos(roll)) < 0.4
            ):
                raise ValueError("Unsafe actual body attitude")
            ack_sequence = integer(ack["sequence"])
            request, applied = (
                vector(ack["requested_command_body"], 3),
                vector(ack["applied_command_body"], 3),
            )
            if np.any(abs(request) > [1.0 + 1e-9, 0.4 + 1e-9, 1.0 + 1e-9]) or np.any(
                abs(applied) > [1.0 + 1e-9, 0.4 + 1e-9, 1.0 + 1e-9]
            ):
                raise ValueError("Executed velocity outside training command boundary")
            ack_hash_record = dict(
                stamp_ns=ack_stamp,
                sequence=ack_sequence,
                received_wall_ns=int(ack["received_wall_ns"]),
                requested=request.tolist(),
                applied=applied.tolist(),
            )
            if ack.get("ordinary_zero_before_actor_observation") is not None:
                ack_hash_record.update(
                    ordinary_zero_before_actor_observation=ack[
                        "ordinary_zero_before_actor_observation"
                    ],
                    ack_native_clock_ns=ack.get("ack_native_clock_ns"),
                )
            if self.student_lower_ack_config is not None:
                from embodied_navigation.robot.unitree.policy.lower_loop_ack import (
                    residual as lower_ack_residual,
                )

                lower_ack_residual(ack, self.student_lower_ack_config, self.limits)
                if ack["ack_native_clock_ns"] > clock:
                    raise ValueError("Student native ACK is future to control clock")
                ack_hash_record["r85_student_lower_loop_ack"] = copy.deepcopy(
                    ack["r85_student_lower_loop_ack"]
                )
            ack_hash = sha(ack_hash_record)
            if self.last_ack_sequence is not None and (
                ack_sequence < self.last_ack_sequence
                or (ack_sequence == self.last_ack_sequence and ack_hash != self.last_ack_hash)
            ):
                raise ValueError("Executed command acknowledgement changed or went backwards")
            if self.student_lower_ack_config is not None:
                note = ack["r85_student_lower_loop_ack"]
                binding = (
                    ack_sequence,
                    note["input_clock_ns"],
                    note["input_frame_serial"],
                    note["inference_sequence"],
                )
                old = self.last_student_lower_ack_binding
                if (
                    old is not None
                    and binding[0] > old[0]
                    and any(binding[i] <= old[i] for i in (1, 2, 3))
                ):
                    raise ValueError(
                        "New Student ACK sequence reused/backtracked an actual inference frame"
                    )
                self.last_student_lower_ack_binding = binding
            self.last_ack_sequence, self.last_ack_hash = ack_sequence, ack_hash
            pose_hash = sha(
                dict(
                    position=position.tolist(),
                    quaternion=vector(feedback["quaternion_wxyz"], 4).tolist(),
                    velocity=origin_body.tolist(),
                    receipt_ns=receipt,
                )
            )
            if self.last_seen_pose_stamp is not None and stamp < self.last_seen_pose_stamp:
                raise ValueError("Original SLAM header went backwards")
            duplicate = stamp == self.last_seen_pose_stamp
            if duplicate and pose_hash != self.last_pose_hash:
                raise ValueError("A repeated SLAM header changed data or receipt")
        except (ValueError, TypeError, KeyError) as error:
            reason = str(error)
            recoverable = reason in (
                "No checked SCAN path",
                "Source stale or future",
                "IMU or executed command is not causal to pose",
                "SCAN path generation is future to this pose",
            )
            self._protect(reason, fatal=not recoverable)
            return self._output("protect", clock, wall, reason=self.protection_reason)
        # Check the old receipt BEFORE accepting the fresh one. A late update
        # does not erase a >300ms gap and falsely continue integral or dwell.
        if self.last_math_stamp is not None and (
            stamp - self.last_math_stamp > 300_000_000 or wall - self.last_math_wall > 300_000_000
        ):
            if self.minimal_simulation_baseline and stamp - self.last_math_stamp <= 300_000_000:
                self.minimal_baseline_observation.update(
                    math_wall_gap_would_protect=True,
                    math_wall_gap_ns=wall - self.last_math_wall,
                    math_header_gap_ns=stamp - self.last_math_stamp,
                )
            else:
                self._protect("feedback_gap_before_fresh")
        if duplicate:
            return self._output(
                "protect" if self.protected else self.row.get("mode", "hold"),
                clock,
                wall,
                reason=self.protection_reason if self.protected else "duplicate_SLAM_header",
            )
        self.last_seen_pose_stamp, self.last_pose_hash = stamp, pose_hash
        if self.protected:
            self.recovery.append(stamp)
            self.recovery = self.recovery[-2:]
            if len(self.recovery) < 2:
                # This is a qualified real source sample, not a math update.
                # Seed measurements from its actual motion; zero refers only
                # to the committed reference/output, never to measured motion.
                yawdot = (math.sin(roll) * gyro[1] + math.cos(roll) * gyro[2]) / math.cos(pitch)
                origin_world = R @ origin_body
                com_body = origin_body + np.cross(gyro, self.com_offset)
                self.filtered = np.r_[origin_world[:2], yawdot]
                self.inner_filtered = np.r_[com_body[:2], gyro[2]]
                self.last_math_stamp, self.last_math_wall = stamp, wall
                self.last_reference_speed = self.last_curvature_yaw_reference = 0.0
                self.command[:] = 0.0
                self.row = dict(
                    kind="recovery_source_seed",
                    feedback_pose_stamp_ns=stamp,
                    feedback_received_wall_ns=receipt,
                    imu_stamp_ns=gyro_stamp,
                    ack_stamp_ns=ack_stamp,
                    history_kind="seed",
                    math_step_executed=False,
                    header_dt_s=0.0,
                    integral_dt_s=0.0,
                    reference_velocity_world=[0.0, 0.0, 0.0],
                    reference_COM_velocity_body=[0.0, 0.0, 0.0],
                    raw_command_body=[0.0, 0.0, 0.0],
                    measured_origin_velocity_world=origin_world.tolist(),
                    measured_COM_velocity_body=com_body.tolist(),
                    measured_body_omega=gyro.tolist(),
                )
                return self._output("recovering", clock, wall, reason=self.protection_reason)
            self.protected = False
            self.protection_reason = None
        # Recovery establishes this anchor using a previous actual source.
        # A duplicate source cannot reach this branch or refresh the anchor.
        if self.last_math_stamp is None or stamp <= self.last_math_stamp:
            self._protect("missing_positive_source_math_interval")
            return self._output("protect", clock, wall, reason=self.protection_reason)
        dt = (stamp - self.last_math_stamp) / 1e9
        self.last_math_stamp, self.last_math_wall = stamp, wall
        if dt > 0.20000001:
            self.capture_dwell = self.turn_dwell = None
            self.ordinary_parking.dwell = None
        yawdot = (math.sin(roll) * gyro[1] + math.cos(roll) * gyro[2]) / math.cos(pitch)
        origin_world = R @ origin_body
        com_body = origin_body + np.cross(gyro, self.com_offset)
        measurement = np.r_[origin_world[:2], yawdot]
        actual = np.r_[com_body[:2], gyro[2]]
        ordinary_entry_reset = self.ordinary_reset_pending
        if ordinary_entry_reset:
            self.reset_control_memory("ordinary_terminal_capture_entry", measurement, actual)
            self.ordinary_reset_pending = False
        self.filtered = (
            measurement.copy()
            if self.filtered is None
            else self.filtered + dt / (0.2 + dt) * (measurement - self.filtered)
        )
        self.inner_filtered = (
            actual.copy()
            if self.inner_filtered is None
            else self.inner_filtered + dt / (0.1 + dt) * (actual - self.inner_filtered)
        )
        if terminal is not None:
            # Capture has no route projection. These absent values are emitted
            # as null diagnostics, never fabricated as a checked path/chord.
            segment = point = tangent = normal = cross = grade = nearest_s = None
            control_tangent = control_normal = control_cross = remaining = None
            spatial = dict(
                reference_kind="terminal_fixed_goal/v1",
                heading_rad=self.goal_yaw,
                path_geometry_present=False,
            )
        else:
            try:
                projection = self._project(position)
                spatial = self._reference_from_projection(position, projection)
            except (ValueError, TypeError, IndexError, FloatingPointError) as error:
                self._protect("spatial_reference_invalid: " + str(error), fatal=True)
                return self._output("protect", clock, wall, reason=self.protection_reason)
            segment, point, tangent, normal, cross, grade, nearest_s = projection
            control_tangent = np.asarray(spatial["tangent_xy"])
            control_normal = np.asarray(spatial["normal_xy"])
            control_cross = spatial["cross_ref_m"]
            remaining = float(self.cumulative[-1] - self.progress)
        goal_error = self.goal[:2] - position[:2]
        goal_distance = float(np.linalg.norm(goal_error))
        goal_z_error = float(self.goal[2] - position[2])
        endpoint_is_goal = (
            False if terminal is not None else np.linalg.norm(self.path[-1] - self.goal) <= 0.2
        )
        heading = spatial["heading_rad"]
        error_yaw = wrap(heading - yaw)
        ordinary = self.ordinary_parking
        if ordinary.enabled:
            if (
                sha(self.goal_record) != self.goal_sha
                or not np.array_equal(self.goal, np.asarray(self.goal_record["position_world_xyz"]))
                or self.goal_yaw != self.goal_record["heading_rad"]
            ):
                self._protect("ordinary_fixed_goal_identity_changed", fatal=True)
                return self._output("protect", clock, wall, reason=self.protection_reason)
            if not ordinary.pose10hz and (
                goal_distance > 0.3
                or abs(goal_z_error) > self.goal_z_tolerance
                or abs(wrap(self.goal_yaw - yaw)) > 0.6
            ):
                observed = ordinary.observe_business_rejection(
                    "parking_fixed_target_lost",
                    stamp,
                    clock,
                    dict(
                        goal_distance_xy_m=goal_distance,
                        goal_z_error_m=goal_z_error,
                        goal_yaw_error_rad=wrap(self.goal_yaw - yaw),
                        original_goal_z_tolerance_m=self.goal_z_tolerance,
                    ),
                    getattr(self, "diagnostic_observe_only", None),
                    terminal_zero_event=self.ordinary_zero_event,
                )
                if not observed:
                    self._protect("parking_fixed_target_lost")
                    return self._output("protect", clock, wall, reason=self.protection_reason)
        previous_ordinary_stage = ordinary.stage
        from embodied_navigation.control.parking.parking_pose10hz import from_feedback

        sensor_pose = from_feedback(feedback) if ordinary.pose10hz else None
        stage = ordinary.update(
            stamp,
            clock,
            position,
            yaw,
            origin_body,
            yawdot,
            gyro[2],
            goal_distance,
            wrap(self.goal_yaw - yaw),
            diagnostic=getattr(self, "diagnostic_observe_only", None),
            sensor_pose=sensor_pose,
        )
        if (
            stage == "controlled_deceleration"
            and previous_ordinary_stage != "controlled_deceleration"
        ):
            self.reset_control_memory("ordinary_controlled_deceleration_entry", measurement, actual)
        if stage == "failed":
            self._protect(ordinary.failure, fatal=True)
            return self._output("protect", clock, wall, reason=self.protection_reason)
        if stage in ("handoff_zero", "zero_hold"):
            if stage == "handoff_zero":
                self.reset_control_memory("ordinary_terminal_exact_zero_handoff")
                self.hold_stamp, self.hold_clock, self.hold_wall = stamp, clock, receipt
            self.command[:] = 0.0
            self.row = dict(
                kind="ordinary_zero_hold_actual_source_monitor",
                feedback_pose_stamp_ns=stamp,
                feedback_received_wall_ns=receipt,
                imu_stamp_ns=gyro_stamp,
                ack_stamp_ns=ack_stamp,
                header_dt_s=dt,
                integral_dt_s=0.0,
                raw_command_body=[0.0, 0.0, 0.0],
                reference_velocity_world=[0.0, 0.0, 0.0],
                reference_COM_velocity_body=[0.0, 0.0, 0.0],
                measured_origin_velocity_world=origin_world.tolist(),
                measured_origin_velocity_body=origin_body.tolist(),
                measured_COM_velocity_body=com_body.tolist(),
                measured_Euler_yawrate_radps=yawdot,
                measured_body_omega=gyro.tolist(),
                body_roll_rad=roll,
                body_pitch_rad=pitch,
                goal_distance_xy_m=goal_distance,
                goal_z_error_m=goal_z_error,
                capture_measurement_eligible=bool(
                    ordinary.capture_grid.ready
                    if ordinary.pose10hz
                    else goal_distance <= 0.025
                    and abs(wrap(self.goal_yaw - yaw)) <= 0.035
                    and np.linalg.norm(origin_body[:2]) < 0.03
                    and abs(yawdot) < 0.06
                ),
                precise_capture_completed=True,
                capture_fresh_dwell_start_pose_stamp_ns=ordinary.dwell,
                source_hz_is_math_update_hz=True,
                parking_active_command_need_not_be_zero=False,
            )
            zero_ack = ack.get("ordinary_zero_before_actor_observation")
            if (
                ordinary.first_actor_zero_ack is None
                and self.ordinary_zero_event is not None
                and isinstance(zero_ack, dict)
                and zero_ack.get("event_id") == self.ordinary_zero_event["event_id"]
            ):
                try:
                    qualified = validate_zero_ack(ack, self.ordinary_zero_event, clock)
                except (ValueError, KeyError, TypeError) as error:
                    self._protect("ordinary_zero_ACK_rejected: " + str(error), fatal=True)
                    return self._output("protect", clock, wall, reason=self.protection_reason)
                if self.row["capture_measurement_eligible"]:
                    qualified.update(
                        qualification_clock_ns=clock,
                        qualification_pose_stamp_ns=stamp,
                        qualification_goal_distance_xy_m=goal_distance,
                        qualification_yaw_error_rad=wrap(self.goal_yaw - yaw),
                        qualification_source_yawdot_radps=yawdot,
                        ack_source_fields={
                            key: copy.deepcopy(ack[key])
                            for key in (
                                "stamp_ns",
                                "sequence",
                                "ack_native_clock_ns",
                                "requested_command_body",
                                "applied_command_body",
                                "navigation_ground_truth_used",
                            )
                        },
                        qualification_origin=dict(
                            stamp_ns=stamp,
                            position=position.tolist(),
                            body_velocity=origin_body.tolist(),
                            quaternion_xyzw=vector(feedback["quaternion_wxyz"], 4)[
                                [1, 2, 3, 0]
                            ].tolist(),
                            received_wall_ns=receipt,
                            source_received_wall_ns=receipt,
                            received_sim_ns=clock,
                            source_topic="/demo/slam/body_odom",
                            navigation_ground_truth_used=False,
                        ),
                    )
                    if ordinary.pose10hz:
                        selected = ordinary.capture_grid.last
                        qualified.update(
                            parking_rule="ordinary_pose10hz_zero_ack_fixed5/v1",
                            pose10hz_capture=ordinary.capture_grid.evidence(),
                            qualification_pose_stamp_ns=selected["source_stamp_ns"],
                            qualification_origin=copy.deepcopy(selected["source"]),
                        )
                    ordinary.first_actor_zero_ack = copy.deepcopy(qualified)
                    if ordinary.pose10hz:
                        ordinary.start_pose10hz_window(qualified)
                        ordinary.pose_grid.observe(sensor_pose)
                else:
                    # ACK validity above is unchanged. This attempt never
                    # acquired a first qualifying proof or a formal T0.
                    attempt = dict(
                        reason="ordinary_preT0_precise_qualification_lost",
                        event=copy.deepcopy(self.ordinary_zero_event),
                        source_pose_stamp_ns=stamp,
                        control_clock_ns=clock,
                        ready_pose_stamp_ns=ordinary.ready_stamp,
                        handoff_clock_ns=ordinary.handoff_clock,
                        first_zero_publication_clock_ns=ordinary.first_zero_publication_clock,
                        capture_begin_clock_ns=ordinary.start_clock,
                        capture_deadline_clock_ns=ordinary.deadline,
                        actual_zero_ACK=dict(
                            sequence=ack_sequence,
                            stamp_ns=ack_stamp,
                            ack_native_clock_ns=ack["ack_native_clock_ns"],
                            producer_sequence=qualified["producer_sequence"],
                            event_id=qualified["event_id"],
                            original_validation_passed=True,
                        ),
                        capture_measurement_eligible=self.row["capture_measurement_eligible"],
                        goal_distance_xy_m=goal_distance,
                        goal_yaw_error_rad=wrap(self.goal_yaw - yaw),
                        source_velocity_body=origin_body.tolist(),
                        source_yawdot_radps=yawdot,
                        formal_T0_created=False,
                        navigation_ground_truth_used=False,
                    )
                    if ordinary.recapture_before_first_qualification(attempt):
                        self.ordinary_zero_event = None
                        self.hold_stamp = self.hold_clock = self.hold_wall = self.capture_dwell = (
                            None
                        )
                        self.reset_control_memory("ordinary_preT0_recapture", measurement, actual)
                        self.row.update(
                            kind="ordinary_preT0_recapture",
                            preT0_attempt_disqualified=attempt,
                            precise_capture_completed=False,
                        )
                        # Zero for this source, then the next fresh source
                        # executes the existing capture controller and dwell.
                        return self._output(
                            "capture",
                            clock,
                            wall,
                            updated=True,
                            reason="ordinary_preT0_precise_qualification_lost",
                        )
            return self._output("ordinary_zero_hold", clock, wall, updated=True)
        capture_ready = (
            terminal is None
            and endpoint_is_goal
            and remaining <= 0.15
            and goal_distance <= 0.12
            and abs(goal_z_error) <= self.goal_z_tolerance
            and (ordinary.pose10hz or np.linalg.norm(origin_body[:2]) < 0.08 and abs(yawdot) < 0.1)
            and abs(wrap(self.goal_yaw - yaw)) < 0.2
        )
        entry_reset = False
        if self.capture_stamp is None and (
            capture_ready and mode_override != "brake" or mode_override == "capture"
        ):
            if goal_distance > 0.3 or abs(goal_z_error) > self.goal_z_tolerance:
                observed = ordinary.observe_business_rejection(
                    "capture_fixed_goal_out_of_range",
                    stamp,
                    clock,
                    dict(
                        goal_distance_xy_m=goal_distance,
                        goal_z_error_m=goal_z_error,
                        goal_yaw_error_rad=wrap(self.goal_yaw - yaw),
                        original_goal_z_tolerance_m=self.goal_z_tolerance,
                    ),
                    getattr(self, "diagnostic_observe_only", None),
                    terminal_zero_event=self.ordinary_zero_event,
                )
                if not observed:
                    self._protect("capture_fixed_goal_out_of_range")
                    return self._output("protect", clock, wall, reason=self.protection_reason)
            self.capture_stamp, self.capture_wall, self.capture_clock = stamp, receipt, clock
            self.arrival = dict(
                position_world_xyz=position.tolist(), yaw_rad=yaw, pose_stamp_ns=stamp
            )
            self.hold_integral[:] = 0.0
            self.capture_dwell = None
            entry_reset = True
        position_terms = dict(P=[0.0, 0.0], D=[0.0, 0.0])
        yaw_ff = 0.0
        speed_diagnostic = dict(enabled=False)
        vertical_reference_model = "fixed_goal_world_xy_hold"
        external_turn_reference = None
        if self.capture_stamp is not None:
            if not ordinary.pose10hz and (
                goal_distance > 0.3
                or abs(goal_z_error) > self.goal_z_tolerance
                or abs(wrap(self.goal_yaw - yaw)) > 0.6
            ):
                observed = ordinary.observe_business_rejection(
                    "parking_fixed_target_lost",
                    stamp,
                    clock,
                    dict(
                        goal_distance_xy_m=goal_distance,
                        goal_z_error_m=goal_z_error,
                        goal_yaw_error_rad=wrap(self.goal_yaw - yaw),
                        original_goal_z_tolerance_m=self.goal_z_tolerance,
                    ),
                    getattr(self, "diagnostic_observe_only", None),
                    terminal_zero_event=self.ordinary_zero_event,
                )
                if not observed:
                    self._protect("parking_fixed_target_lost")
                    return self._output("protect", clock, wall, reason=self.protection_reason)
            xy_gate = 0.03 if self.capture_dwell is not None else 0.025
            yaw_gate = 0.045 if self.capture_dwell is not None else 0.035
            eligible = (
                goal_distance <= xy_gate
                and abs(wrap(self.goal_yaw - yaw)) <= yaw_gate
                and np.linalg.norm(origin_body[:2]) < 0.03
                and abs(yawdot) < 0.06
            )
            if ordinary.enabled:
                self.capture_dwell = ordinary.dwell
                mode = "capture"  # OrdinaryParking alone owns readiness and handoff.
            else:
                if self.hold_stamp is None:
                    self.capture_dwell = (
                        (stamp if self.capture_dwell is None else self.capture_dwell)
                        if eligible
                        else None
                    )
                    if self.capture_dwell is not None and stamp - self.capture_dwell >= 600_000_000:
                        self.hold_stamp, self.hold_wall, self.hold_clock = stamp, receipt, clock
                mode = "active_hold" if self.hold_stamp is not None else "capture"
            prefix = "" if mode == "active_hold" else "capture_"
            P = PARKING[prefix + "position_kp"] * deadband(goal_error, 0.003)
            D = -PARKING[prefix + "position_kd"] * self.filtered[:2]
            world_xy = P + D
            cap = PARKING[prefix + "reference_xy_norm_limit_mps"]
            if np.linalg.norm(world_xy) > cap:
                world_xy *= cap / np.linalg.norm(world_xy)
            velocity_world = np.r_[world_xy, 0.0]
            heading, error_yaw = self.goal_yaw, wrap(self.goal_yaw - yaw)
            yaw_P = 0.65 * float(deadband(error_yaw, 0.005))
            yaw_D = -0.18 * self.filtered[2]
            wref = float(np.clip(yaw_P + yaw_D, -0.07, 0.07))
            limits = np.asarray(PARKING[prefix + "command_limits_body"])
            integral, integral_dt = (
                self.hold_integral,
                0.0 if ordinary_entry_reset or stage == "controlled_deceleration" else dt,
            )
            if stage == "controlled_deceleration":
                velocity_world[:] = 0.0
                wref = 0.0
                mode = "controlled_deceleration"
            position_terms = dict(P=P.tolist(), D=D.tolist())
        else:
            eligible = False
            if external_turn_heading_value is not None:
                external_turn_reference = dict(
                    reference_yaw_source="external_heading_gate_locked_reference",
                    external_turn_heading_rad=external_turn_heading_value,
                    ungated_reference_yaw_rad=heading,
                )
                heading = external_turn_heading_value
                error_yaw = wrap(heading - yaw)
            if self.external_heading_gate:
                self.turn_phase = mode_override if mode_override in ("turn", "brake") else None
                self.turn_dwell = None
            if not self.external_heading_gate and self.turn_phase is None and abs(error_yaw) > 0.2:
                self.turn_phase = "pre_turn"
                self.turn_dwell = None
            if mode_override in ("turn", "brake"):
                self.turn_phase = mode_override
                self.turn_dwell = None
            elif self.turn_phase == "pre_turn":
                stopped = np.linalg.norm(origin_body[:2]) < 0.04 and abs(yawdot) < 0.05
                self.turn_dwell = (
                    (stamp if self.turn_dwell is None else self.turn_dwell) if stopped else None
                )
                if self.turn_dwell is not None and stamp - self.turn_dwell >= 300_000_000:
                    self.turn_phase = "turn"
                    self.turn_dwell = None
            elif self.turn_phase == "turn" and abs(error_yaw) < 0.08 and abs(yawdot) < 0.08:
                self.turn_phase = "settle"
                self.turn_dwell = stamp
            elif self.turn_phase == "settle":
                if abs(error_yaw) > 0.2:
                    self.turn_phase = "turn"
                    self.turn_dwell = None
                elif self.turn_dwell is not None and stamp - self.turn_dwell >= 300_000_000:
                    self.turn_phase = None
                    self.turn_dwell = None
            mode = self.turn_phase or "drive"
            speed = min(
                self.speed, 0.9 * max(remaining, 0.0), math.sqrt(0.7 * max(remaining - 0.05, 0.0))
            )
            base_reference_speed = speed
            diagnostic_reference_budget_continued = False
            yaw_P, yaw_D = 1.3 * error_yaw, -0.15 * self.filtered[2]
            if (
                self.teacher_pre_zero_heading_p_taper_config is not None
                and mode == "brake"
                and pre_zero_unload_event is not None
            ):
                # validate_event above owns the fixed entry/expiry. This control
                # clock is never a renewed event or a wall-time estimate.
                setting = self.teacher_pre_zero_heading_p_taper_config
                elapsed_s = (clock - pre_zero_unload_event["brake_entry_clock_ns"]) / 1e9
                progress = min(1.0, max(0.0, elapsed_s / setting["duration_s"]))
                effective_kp = (
                    setting["base_kp"] + (setting["target_kp"] - setting["base_kp"]) * progress
                )
                yaw_P = effective_kp * error_yaw
                teacher_heading_p_taper = dict(
                    schema="r85_teacher_pre_zero_heading_p_taper/v1",
                    kind=setting["kind"],
                    event=copy.deepcopy(pre_zero_unload_event),
                    control_clock_ns=clock,
                    elapsed_s=elapsed_s,
                    progress=progress,
                    base_kp=setting["base_kp"],
                    target_kp=setting["target_kp"],
                    effective_kp=effective_kp,
                    duration_s=setting["duration_s"],
                    kd=0.15,
                    heading_error_rad=float(error_yaw),
                    yaw_P=float(yaw_P),
                    yaw_D=float(yaw_D),
                    inside_authorized_brake=True,
                    event_outside_restores_base_immediately=True,
                    new_heading_P_math=True,
                    stop_credit=False,
                )
            if mode == "drive" and self.spatial_config["curvature_speed_limit_enabled"]:
                if spatial["curvature_valid"]:
                    # Euler yaw budget is converted to the body-z command
                    # boundary with the same measured attitude/omega_y as below.
                    rate_budget = max(
                        0.0,
                        (self.limits[2] * abs(math.cos(roll)) - abs(math.sin(roll) * gyro[1]))
                        / abs(math.cos(pitch)),
                    )
                    feedforward_gain = (
                        1.15 if self.spatial_config["curvature_feedforward_enabled"] else 1.0
                    )
                    speed, speed_diagnostic = speed_budget(
                        speed,
                        feedforward_gain * spatial["curvature_per_m"],
                        feedforward_gain * spatial["curvature_rate_per_m2"],
                        rate_budget,
                        yaw_P + yaw_D,
                        self.last_reference_speed,
                        dt,
                        self.spatial_config,
                        previous_yaw_reference=self.last_curvature_yaw_reference,
                    )
                elif spatial["heading_source"] != "fixed_goal_tail":
                    speed_diagnostic = dict(
                        enabled=True,
                        reference_budget_feasible=False,
                        reason="curvature_unavailable_requires_stop_or_new_path",
                        limited_speed_mps=0.0,
                    )
                    speed = 0.0
            diagnostic = getattr(self, "diagnostic_observe_only", {})
            diagnostic_profile = (
                diagnostic.get("profile", {}) if isinstance(diagnostic, dict) else {}
            )
            observe_config = diagnostic_profile.get("diagnostic_observe_only", {})
            if (
                mode == "drive"
                and speed_diagnostic.get("reference_budget_feasible") is False
                and isinstance(observe_config, dict)
                and type(observe_config.get("enabled")) is bool
                and observe_config["enabled"]
                and diagnostic.get("run") is not None
            ):
                # Score observation cannot admit nonfinite derived references,
                # including same-reason ledger calls that are deduplicated.
                finite_terms = np.asarray(
                    [
                        base_reference_speed,
                        speed,
                        dt,
                        heading,
                        error_yaw,
                        cross,
                        control_cross,
                        grade,
                        remaining,
                        nearest_s,
                        goal_distance,
                        goal_z_error,
                        yaw_P,
                        yaw_D,
                        spatial["curvature_per_m"],
                        spatial["curvature_rate_per_m2"],
                        self.last_curvature_yaw_reference,
                    ],
                    dtype=float,
                )
                if (
                    not np.isfinite(finite_terms).all()
                    or base_reference_speed < 0
                    or dt <= 0
                    or type(segment) not in (int, np.int32, np.int64)
                    or not 0 <= segment < len(self.lengths)
                    or np.asarray(control_tangent).shape != (2,)
                    or not np.isfinite(control_tangent).all()
                    or np.asarray(control_normal).shape != (2,)
                    or not np.isfinite(control_normal).all()
                    or self.last_reference_speed is not None
                    and not math.isfinite(self.last_reference_speed)
                ):
                    raise ValueError("Invalid finite reference-budget diagnostic inputs")
                canonical(speed_diagnostic)
                from embodied_navigation.common.observation import api as _observer_api

                record = _observer_api("diagnostic_observation").record
                recorded = record(
                    diagnostic["run"],
                    diagnostic_profile,
                    "cascade_reference_curvature_budget",
                    str(speed_diagnostic.get("reason", "reference_budget_infeasible")),
                    dict(
                        original_speed_diagnostic=speed_diagnostic,
                        finite_base_speed_mps=base_reference_speed,
                        source_pose_stamp_ns=stamp,
                        control_stamp_ns=clock,
                        original_curvature_valid=spatial["curvature_valid"],
                        navigation_PASS=False,
                    ),
                )
                if recorded is not True:
                    raise RuntimeError("Reference-budget observation was not recorded")
                speed = base_reference_speed
                diagnostic_reference_budget_continued = True
                speed_diagnostic = dict(
                    speed_diagnostic,
                    diagnostic_observe_only=True,
                    diagnostic_execution_continued=True,
                    diagnostic_speed_mps=speed,
                    original_reference_budget_feasible=False,
                    navigation_PASS=False,
                )
            if (
                mode == "drive"
                and self.spatial_config["curvature_feedforward_enabled"]
                and spatial["curvature_valid"]
            ):
                yaw_ff = spatial["curvature_per_m"] * speed
                yaw_D = 0.15 * (yaw_ff - self.filtered[2])
            lateral = float(
                np.clip(
                    -0.8 * control_cross - 0.2 * float(self.filtered[:2] @ control_normal),
                    -0.2,
                    0.2,
                )
            )
            world_xy = speed * control_tangent + lateral * control_normal
            if self.spatial_config["enabled"]:
                # Only the original path directional derivative is known.
                # This local planar extension assumes its horizontal normal
                # is an equal-height direction; it is not measured terrain
                # geometry and never certifies support or clearance.
                world_z = grade * float(world_xy @ tangent)
                vertical_reference_model = (
                    "original_path_grade_gradient_assuming_raw_normal_equal_height"
                )
            else:
                world_z = speed * grade
                vertical_reference_model = "legacy_speed_times_original_grade"
            velocity_world = np.r_[world_xy, world_z]
            wref = yaw_ff + yaw_P + yaw_D
            if mode == "brake":
                if self.neutral_rate_brake_capture_config is not None:
                    neutral_rate_capture = dict(
                        schema="r85_neutral_rate_brake_capture/v1",
                        kind=self.neutral_rate_brake_capture_config["kind"],
                        locked_heading_target_rad=float(heading),
                        locked_heading_error_rad=float(error_yaw),
                        unshaped_locked_heading_PD=dict(
                            P=float(yaw_P),
                            D=float(yaw_D),
                            curvature_FF=float(yaw_ff),
                            reference_Euler_yawrate_radps=float(wref),
                        ),
                        applied_heading_PD=dict(P=0.0, D=0.0, curvature_FF=0.0),
                        reference_Euler_yawrate_radps=0.0,
                        heading_target_moved=False,
                        actual_heading_error_zeroed=False,
                        pre_zero_event_id=pre_zero_unload_event["event_id"],
                        final_zero_stop_evidence=False,
                    )
                    wref = 0.0
                    yaw_P = yaw_D = yaw_ff = 0.0
                elif self.turn_brake_reference_kind == "pure_zero_euler_rate_v1":
                    # Preserve the original A1 rate-only reference by default.
                    wref = 0.0
                    yaw_P = yaw_D = yaw_ff = 0.0
                else:
                    # Keep the same locked heading PD through this one stage.
                    # It restores heading stiffness; original PI, antiwindup,
                    # limits and output slew remain responsible for execution.
                    yaw_ff = 0.0
                    wref = yaw_P + yaw_D
            self.last_reference_speed = speed if mode == "drive" else 0.0
            constraint_hold = (
                mode == "drive"
                and speed_diagnostic.get("reference_budget_feasible") is False
                and not diagnostic_reference_budget_continued
            )
            if constraint_hold:
                mode = "reference_constraint_hold"
                wref = 0.0
                yaw_ff = 0.0
            self.last_curvature_yaw_reference = (
                (1.15 if self.spatial_config["curvature_feedforward_enabled"] else 1.0)
                * spatial["curvature_per_m"]
                * speed
                if mode == "drive" and spatial["curvature_valid"]
                else 0.0
            )
            limits = self.limits
            integral, integral_dt = (
                self.velocity_integral,
                dt if mode in ("drive", "turn", "brake") else 0.0,
            )
            if mode != "drive":
                velocity_world[:] = 0.0
            if remaining < 0.05 and not endpoint_is_goal and mode == "drive":
                mode = "path_end_hold"
                velocity_world[:] = 0.0
                integral_dt = 0.0
        body_wref = (wref * math.cos(pitch) - math.sin(roll) * gyro[1]) / math.cos(roll)
        origin_reference = R.T @ velocity_world
        com_reference = origin_reference + np.cross([0.0, 0.0, body_wref], self.com_offset)
        target = np.r_[com_reference[:2], body_wref]
        requested, raw, candidate, pi = self._pi(
            target,
            self.inner_filtered,
            integral_dt,
            integral,
            limits,
            ack,
            mode == "drive",
            turn_only=mode in ("turn", "brake"),
            pre_zero_unload=pre_zero_unload_event,
        )
        if self.capture_stamp is not None:
            self.hold_integral = candidate
        else:
            self.velocity_integral = candidate
        if mode in ("turn", "brake"):
            requested[:2] = 0.0
        elif mode in ("pre_turn", "settle", "path_end_hold", "reference_constraint_hold"):
            requested[:] = 0.0
            target[:] = 0.0
            velocity_world[:] = 0.0
        self.command = requested
        self.row = dict(
            kind="controller_update",
            feedback_pose_stamp_ns=stamp,
            feedback_received_wall_ns=receipt,
            imu_stamp_ns=gyro_stamp,
            ack_stamp_ns=ack_stamp,
            header_dt_s=dt,
            source_expected_hz=self.config.get("feedback_expected_hz", 10),
            integral_dt_s=integral_dt,
            segment=segment,
            nearest_projection_xyz=None if point is None else point.tolist(),
            nearest_projection_progress_m=nearest_s,
            path_length_m=None if terminal is not None else float(self.cumulative[-1]),
            remaining_horizontal_arc_m=remaining,
            local_horizontal_tangent=None if tangent is None else tangent.tolist(),
            local_horizontal_normal=None if normal is None else normal.tolist(),
            local_grade_dz_ds=grade,
            error_cross_m=cross,
            error_along_remaining_m=remaining,
            error_yaw_rad=error_yaw,
            control_error_cross_m=control_cross,
            control_horizontal_tangent=None
            if control_tangent is None
            else control_tangent.tolist(),
            control_horizontal_normal=None if control_normal is None else control_normal.tolist(),
            spatial_reference=spatial,
            curvature_speed_supervisor=speed_diagnostic,
            vertical_reference_model=vertical_reference_model,
            vertical_reference_terrain_or_support_verified=False,
            reference_yaw_rad=heading,
            reference_velocity_world=velocity_world.tolist(),
            reference_COM_velocity_body=target.tolist(),
            raw_command_body=raw.tolist(),
            command_limits_body=np.asarray(limits).tolist(),
            velocity_PI=pi,
            position_terms=position_terms,
            yaw_PD=dict(P=yaw_P, D=yaw_D, curvature_FF=yaw_ff),
            measured_origin_velocity_world=origin_world.tolist(),
            measured_origin_velocity_body=origin_body.tolist(),
            measured_COM_velocity_body=com_body.tolist(),
            measured_Euler_yawrate_radps=yawdot,
            measured_body_omega=gyro.tolist(),
            body_roll_rad=roll,
            body_pitch_rad=pitch,
            goal_distance_xy_m=goal_distance,
            goal_z_error_m=goal_z_error,
            endpoint_is_fixed_goal=bool(endpoint_is_goal),
            capture_measurement_eligible=bool(eligible),
            capture_fresh_dwell_start_pose_stamp_ns=self.capture_dwell,
            capture_entry_integral_reset=entry_reset,
            command_saturated=(abs(raw - requested) > 1e-9).tolist(),
            COM_reference_angular_assumption="Reference body omega_x/y=0; causal measured omega_y in Euler yaw conversion",
            source_hz_is_math_update_hz=True,
            parking_active_command_need_not_be_zero=True,
        )
        if mode == "brake":
            self.row["active_turn_braking"] = dict(
                schema="r85_active_turn_braking/v1",
                reference_Euler_yawrate_radps=0.0,
                translation_command_body=[0.0, 0.0],
                original_PI_antiwindup_and_limits=True,
                integral_reset_at_entry=False,
                final_zero_stop_evidence=False,
            )
            if self.turn_brake_reference_kind == "locked_heading_pd_v1":
                self.row["active_turn_braking"].update(
                    reference_kind=self.turn_brake_reference_kind,
                    reference_Euler_yawrate_radps=float(wref),
                    locked_heading_restoration=True,
                    locked_heading_error_rad=float(error_yaw),
                    original_heading_PD_gains=dict(kp=1.3, kd=0.15),
                )
        if mode == "brake" and neutral_rate_capture is not None:
            self.row["neutral_rate_brake_capture"] = neutral_rate_capture
            self.row["active_turn_braking"].update(
                reference_kind=self.neutral_rate_brake_capture_config["kind"],
                base_reference_kind=self.turn_brake_reference_kind,
                locked_heading_restoration=False,
                neutral_rate_capture=True,
            )
        if pre_zero_unload_event is not None and mode == "brake":
            self.row["active_turn_braking"].update(
                original_PI_antiwindup_and_limits=False,
                original_gains_and_limits=True,
                new_yaw_integral_recurrence=True,
            )
        if teacher_heading_p_taper is not None:
            self.row["teacher_pre_zero_heading_p_taper"] = teacher_heading_p_taper
            self.row["active_turn_braking"].update(
                original_gains_and_limits=False,
                original_limits_unchanged=True,
                original_heading_PD_gains_role="outside_event_base_only",
                effective_heading_PD_gains=dict(
                    kp=teacher_heading_p_taper["effective_kp"], kd=0.15
                ),
                new_positive_heading_P_taper=True,
            )
        if external_turn_reference is not None:
            self.row.update(external_turn_reference)
        return self._output(mode, clock, wall, updated=True)
