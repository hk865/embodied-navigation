"""R45 conditional two-degree-of-freedom yaw P; no new event/stop authority.

Only an already accepted current owned pre-zero event retains measurement P.
Outside it the caller leaves its original P term untouched, including zero.
"""

import copy
import hashlib
import math
from pathlib import Path

KEY = "student_yaw_measurement_p_retention"
REQUIRED = KEY + "_required"
PROFILE_KEY = "r85_" + KEY
KIND = "accepted_pre_zero_event_yaw_measurement_P_retention_v1"
CONFIG = dict(enabled=True, kind=KIND)
BASE_ACK_CONFIG = dict(
    enabled=True,
    schema="r85_student_lower_loop_ack/v1",
    student_contract_sha256="fedc24a112b1ec599f54d086af074af4fc925ff3a4e2e1d10cfbecec372300dd",
)
MEASUREMENT_ACK_CONFIG = dict(BASE_ACK_CONFIG, measurement_p_retention_required=True)
SCHEMA = "student_yaw_measurement_P_retention/v1"
SOURCE_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def configuration(profile):
    """Return the frozen R85 measurement-retention profile, or None when disabled.

    Disabled means absent or exactly ``{"enabled": false}``. Enabled demands the
    single reviewed CONFIG plus the original coordinated 10 s integral unload and
    quiet locked-heading braking, so a partial profile cannot select it.
    """
    if not isinstance(profile, dict):
        raise ValueError("Actual full profile required")
    trusted = profile.get("cascade", {}).get("r85_student_lower_loop_ack")
    from embodied_navigation.robot.unitree.policy.speed_tiers import (
        configuration as speed_configuration,
        split_ack,
    )

    speed_configuration(profile)
    trusted, _ = split_ack(trusted)
    # A separately selected XY mode extends only the trusted ACK selection.
    # Its strict flag is not a change to the original yaw measurement law.
    if isinstance(trusted, dict) and "xy_stop_takeover_enabled" in trusted:
        if type(trusted["xy_stop_takeover_enabled"]) is not bool:
            raise ValueError("XY stop mode selection requires a boolean")
        trusted = dict(trusted)
        trusted.pop("xy_stop_takeover_enabled")
    if PROFILE_KEY not in profile:
        if isinstance(trusted, dict) and "measurement_p_retention_required" in trusted:
            raise ValueError("Required measurement proof cannot disable its selected control law")
        return None
    value = profile[PROFILE_KEY]
    if isinstance(value, dict) and set(value) == {"enabled"} and value["enabled"] is False:
        if trusted != BASE_ACK_CONFIG or trusted.get("enabled") is not True:
            raise ValueError("Disabled measurement P requires exact original ACK configuration")
        return None
    if (
        not isinstance(value, dict)
        or set(value) != set(CONFIG)
        or value != CONFIG
        or value["enabled"] is not True
        or type(value["kind"]) is not str
    ):
        raise ValueError("Exact selected measurement P retention configuration required")
    if (
        trusted != MEASUREMENT_ACK_CONFIG
        or trusted.get("enabled") is not True
        or trusted.get("measurement_p_retention_required") is not True
    ):
        raise ValueError(
            "Enabled measurement P requires exact trusted mandatory ACK proof configuration"
        )
    if (
        profile.get("r85_student_yaw_pd_fade")
        != {
            "enabled": True,
            "kind": "accepted_pre_zero_event_yaw_PD_fade_v1",
            "maximum_alpha_rate_per_s": 1.0,
        }
        or profile["r85_student_yaw_pd_fade"]["enabled"] is not True
    ):
        raise ValueError("Original selected yaw PD fade required")
    if (
        profile.get("r85_pre_zero_integral_unload")
        != {
            "enabled": True,
            "kind": "coordinated_yaw_toward_zero_v1",
            "teacher_rate_integral_per_s": 0.1,
            "student_rate_integral_per_s": 0.04,
            "maneuver_timeout_s": 10.0,
        }
        or profile["r85_pre_zero_integral_unload"]["enabled"] is not True
    ):
        raise ValueError("Original coordinated I unload rates and10s budget required")
    if (
        profile.get("r85_active_turn_braking") != {"enabled": True}
        or profile.get("r85_quiet_handoff_target") != {"enabled": True}
        or profile["r85_active_turn_braking"]["enabled"] is not True
        or profile["r85_quiet_handoff_target"]["enabled"] is not True
        or profile.get("cascade", {}).get("external_heading_gate") is not True
        or profile.get("cascade", {}).get("r85_turn_brake_reference")
        != {"kind": "locked_heading_pd_v1"}
    ):
        raise ValueError("Original active quiet owned locked-heading braking required")
    return copy.deepcopy(value)


def vector(value):
    """Return the measured 3-vector unchanged, or raise on shape/non-finite input."""
    if (
        not isinstance(value, list)
        or len(value) != 3
        or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)
    ):
        raise ValueError("Finite actual three-vector required")
    return value


class MeasurementPRetention:
    KEY = KEY
    REQUIRED = REQUIRED

    def __init__(self, config, unload_helper, run_id, scope_sha256, identity):
        if config != CONFIG or not isinstance(run_id, str) or not run_id:
            raise ValueError("Actual selected helper/run/scope required")
        self.helper = unload_helper
        self.run_id = run_id
        self.scope = scope_sha256
        self.identity = copy.deepcopy(identity)

    def compute(self, dt, clock, frame, epoch, zero, requested, measured, fade, unload, original_P):
        r = [float(v) for v in requested]
        y = [float(v) for v in measured]
        vector(r)
        vector(y)
        if (
            type(clock) is not int
            or type(frame) is not int
            or frame <= 0
            or type(zero) is not bool
            or zero != (not any(r))
            or not isinstance(fade, dict)
            or fade.get("run_id") != self.run_id
            or fade.get("scope_sha256") != self.scope
            or fade.get("input_clock_ns") != clock
            or fade.get("input_frame_serial") != frame
            or fade.get("motion_epoch") != epoch
            or fade.get("dt_s") != dt
            or fade.get("original_exact_zero_priority") != zero
            or not 0.019999 <= dt <= 0.020001
        ):
            raise ValueError("Actual same-frame fade/PID source binding differs")
        alpha = fade.get("alpha")
        if type(alpha) not in (int, float) or not math.isfinite(alpha) or not 0.0 <= alpha <= 1.0:
            raise ValueError("Actual alpha required")
        event = None
        source_clock = None
        if unload is not None:
            if (
                unload.get("helper") is not self.helper
                or unload.get("input_clock_ns") != clock
                or unload.get("input_frame_serial") != frame
            ):
                raise ValueError("Original accepted unload authority differs")
            a = unload["authorization"]
            source_clock = a["source_control_clock_ns"]
            event = self.helper.validate_event(a["event"], self.run_id, self.scope, source_clock)
            if (
                not source_clock <= clock <= source_clock + 300_000_000
                or a["goal_sha256"] != event["goal_sha256"]
                or not self.helper.valid_sha(a["path_sha256"])
                or epoch
                != event["context"]["request_id"]
                + ":waypoint:"
                + str(event["context"]["waypoint_index"])
            ):
                raise ValueError("Original accepted event source/epoch differs")
        active = bool(
            not zero
            and event is not None
            and event["brake_entry_clock_ns"] <= clock < event["expires_clock_ns"]
        )
        if (
            fade.get("active_current_owned_event") is not active
            or fade.get("event_id") != (None if event is None else event["event_id"])
            or fade.get("source_control_clock_ns") != source_clock
        ):
            raise ValueError("Current event and original fade identity differ")
        # Outside/zero returns the original P exactly. No new tail or state.
        base = 0.0 if zero else float(original_P)
        kr = 0.0 if zero else 0.5 * alpha
        ky = 0.0 if zero else 0.5 if active else 0.5 * alpha
        reference = kr * r[2]
        measurement = -ky * y[2]
        actual = reference + measurement if active else base
        q = dict(
            schema=SCHEMA,
            kind=KIND,
            enabled=True,
            new_control_math=True,
            base_contract_proves_new_math=False,
            placement="before_raw_correction_and_original_antiwindup",
            run_id=self.run_id,
            scope_sha256=self.scope,
            input_clock_ns=clock,
            input_frame_serial=frame,
            motion_epoch=epoch,
            dt_s=float(dt),
            alpha=float(alpha),
            base_yaw_Kp=0.5,
            requested_command_body=r,
            measured_native_com_body=y,
            reference_P_gain=kr,
            measurement_P_gain=ky,
            P_original_single_gain=base,
            P_reference=reference,
            P_measurement=measurement,
            P_actual=actual,
            retention_active=active,
            event=copy.deepcopy(event),
            event_id=None if event is None else event["event_id"],
            source_control_clock_ns=source_clock,
            original_exact_zero_priority=zero,
            stop_or_ACK_credit=False,
            helper_identity=copy.deepcopy(self.identity),
        )
        return actual, q


def validate_receipt(q, fade, unload, scope, clock, frame, zero, requested=None, pid=None):
    keys = {
        "schema",
        "kind",
        "enabled",
        "new_control_math",
        "base_contract_proves_new_math",
        "placement",
        "run_id",
        "scope_sha256",
        "input_clock_ns",
        "input_frame_serial",
        "motion_epoch",
        "dt_s",
        "alpha",
        "base_yaw_Kp",
        "requested_command_body",
        "measured_native_com_body",
        "reference_P_gain",
        "measurement_P_gain",
        "P_original_single_gain",
        "P_reference",
        "P_measurement",
        "P_actual",
        "retention_active",
        "event",
        "event_id",
        "source_control_clock_ns",
        "original_exact_zero_priority",
        "stop_or_ACK_credit",
        "helper_identity",
    }
    if not isinstance(q, dict) or set(q) != keys or not isinstance(fade, dict):
        raise ValueError("Typed actual P split proof missing")
    numbers = (
        "dt_s",
        "alpha",
        "base_yaw_Kp",
        "reference_P_gain",
        "measurement_P_gain",
        "P_original_single_gain",
        "P_reference",
        "P_measurement",
        "P_actual",
    )
    if (
        any(type(q[k]) not in (int, float) or not math.isfinite(q[k]) for k in numbers)
        or q["schema"] != SCHEMA
        or q["kind"] != KIND
        or q["enabled"] is not True
        or q["new_control_math"] is not True
        or q["base_contract_proves_new_math"] is not False
        or q["stop_or_ACK_credit"] is not False
        or q["placement"] != "before_raw_correction_and_original_antiwindup"
        or q["scope_sha256"] != scope
        or type(q["input_clock_ns"]) is not int
        or q["input_clock_ns"] != clock
        or type(q["input_frame_serial"]) is not int
        or q["input_frame_serial"] != frame
        or type(q["original_exact_zero_priority"]) is not bool
        or q["original_exact_zero_priority"] != zero
        or type(q["retention_active"]) is not bool
        or q["base_yaw_Kp"] != 0.5
        or not 0.019999 <= q["dt_s"] <= 0.020001
        or not 0.0 <= q["alpha"] <= 1.0
    ):
        raise ValueError("Actual P split identity/clock/no-credit differs")
    _identity(q["helper_identity"], q["run_id"])
    r = vector(q["requested_command_body"])
    y = vector(q["measured_native_com_body"])
    if zero != (not any(r)):
        raise ValueError("P split exactzero authority differs")
    if (
        q["alpha"] != fade.get("alpha")
        or q["dt_s"] != fade.get("dt_s")
        or q["run_id"] != fade.get("run_id")
        or q["motion_epoch"] != fade.get("motion_epoch")
        or fade.get("scope_sha256") != scope
        or fade.get("input_clock_ns") != clock
        or fade.get("input_frame_serial") != frame
        or fade.get("original_exact_zero_priority") != zero
    ):
        raise ValueError("P split and original fade differ")
    event = q["event"]
    if event is not None:
        from embodied_navigation.control.parking.integral_unload import validate_native_extension

        actual = validate_native_extension(unload, scope, clock, frame)
        if (
            actual.get("event") != event
            or actual.get("source_control_clock_ns") != q["source_control_clock_ns"]
        ):
            raise ValueError("P split original accepted event binding differs")
    elif q["source_control_clock_ns"] is not None:
        raise ValueError("P split invented source clock")
    active = bool(
        not zero
        and event is not None
        and event["brake_entry_clock_ns"] <= clock < event["expires_clock_ns"]
    )
    if (
        q["retention_active"] is not active
        or fade.get("active_current_owned_event") is not active
        or q["event_id"] != (None if event is None else event["event_id"])
        or q["event_id"] != fade.get("event_id")
        or q["source_control_clock_ns"] != fade.get("source_control_clock_ns")
    ):
        raise ValueError("P split outside/current-event applicability differs")
    kr = 0.0 if zero else 0.5 * q["alpha"]
    ky = 0.0 if zero else 0.5 if active else kr
    old = 0.0 if zero else 0.5 * q["alpha"] * (r[2] - y[2])
    expected = kr * r[2] - ky * y[2] if active else old
    for value, want in [
        (q["reference_P_gain"], kr),
        (q["measurement_P_gain"], ky),
        (q["P_original_single_gain"], old),
        (q["P_reference"], kr * r[2]),
        (q["P_measurement"], -ky * y[2]),
        (q["P_actual"], expected),
    ]:
        if abs(value - want) > 1e-12:
            raise ValueError("Actual two-degree-of-freedom P reconstruction differs")
    if requested is not None and list(requested) != r:
        raise ValueError("P split original upstream request differs")
    if pid is not None:
        if (
            pid["dt_s"] != q["dt_s"]
            or pid["motion_epoch"] != q["motion_epoch"]
            or list(pid["requested_command_body"]) != r
            or list(pid["measured_native_com_body"]) != y
            or any(abs(a - (b - c)) > 1e-12 for a, b, c in zip(pid["pid_error"], r, y))
            or abs(pid["pid_P"][2] - q["P_actual"]) > 1e-12
        ):
            raise ValueError("P split and actual same-frame PID terms differ")
    return q
