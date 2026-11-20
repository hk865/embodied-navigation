"""Shared measured-yaw local action; no simulator, planner or transport access."""

import math

from embodied_navigation.common.geometry import wrap_angle as wrap

LOCAL_ACTION_BOUNDS = {
    "lateral_mps": 0.05,
    "yaw_rate_rps": 0.15,
    "maneuver_max_duration_s": 2.0,
    "yaw_max_delta_rad": 3.15,
    "yaw_tolerance_rad": 0.05,
    "yaw_gain": 1.2,
}


def finite_number(value, name):
    """Return value as a finite float or raise ValueError."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)



def yaw_config(profile):
    """Original local-yaw defaults and external profile validation."""
    yaw_mode = str(profile.get("interactive_yaw_completion_mode", "dwell"))
    if yaw_mode not in ("dwell", "instant"):
        raise ValueError(
            "interactive_yaw_completion_mode must be 'dwell' or 'instant'"
        )
    if (
        profile.get("interactive_yaw_completion_tolerance_rad") is not None
        and profile.get("interactive_yaw_tolerance_rad") is not None
    ):
        raise ValueError(
            "set either interactive_yaw_completion_tolerance_rad or the deprecated "
            "interactive_yaw_tolerance_rad, not both"
        )
    # Legacy field maps to the completion tolerance only; the control deadband is
    # a separate knob and never reuses it.
    yaw_completion_tolerance = finite_number(
        profile.get(
            "interactive_yaw_completion_tolerance_rad",
            profile.get(
                "interactive_yaw_tolerance_rad",
                0.05 if yaw_mode == "dwell" else LOCAL_ACTION_BOUNDS["yaw_tolerance_rad"],
            ),
        ),
        "interactive_yaw_completion_tolerance_rad",
    )
    if not 0.0 < yaw_completion_tolerance <= 0.10:
        raise ValueError(
            "interactive_yaw_completion_tolerance_rad must be within (0, 0.10]"
        )
    yaw_control_tolerance = finite_number(
        profile.get(
            "interactive_yaw_control_tolerance_rad",
            0.02 if yaw_mode == "dwell" else 0.0,
        ),
        "interactive_yaw_control_tolerance_rad",
    )
    if not 0.0 <= yaw_control_tolerance <= 0.10:
        raise ValueError(
            "interactive_yaw_control_tolerance_rad must be within [0, 0.10]"
        )
    if yaw_control_tolerance > yaw_completion_tolerance:
        raise ValueError(
            "interactive_yaw_control_tolerance_rad must not exceed the completion tolerance"
        )
    yaw_rate = finite_number(
        profile.get("interactive_yaw_settle_rate_radps", 0.03),
        "interactive_yaw_settle_rate_radps",
    )
    if not 0.0 < yaw_rate <= 0.15:
        raise ValueError(
            "interactive_yaw_settle_rate_radps must be within (0, 0.15]"
        )
    yaw_dwell = finite_number(
        profile.get("interactive_yaw_dwell_sim_s", 0.5),
        "interactive_yaw_dwell_sim_s",
    )
    if not 0.1 <= yaw_dwell <= 5.0:
        raise ValueError("interactive_yaw_dwell_sim_s must be within [0.1, 5.0]")
    return {
        "yaw_completion_mode": str(
            profile.get("interactive_yaw_completion_mode", "dwell")
        ),
        "yaw_control_tolerance_rad": finite_number(
            profile.get(
                "interactive_yaw_control_tolerance_rad",
                0.02
                if str(profile.get("interactive_yaw_completion_mode", "dwell"))
                == "dwell"
                else 0.0,
            ),
            "interactive_yaw_control_tolerance_rad",
        ),
        "yaw_completion_tolerance_rad": finite_number(
            profile.get(
                "interactive_yaw_completion_tolerance_rad",
                profile.get(
                    "interactive_yaw_tolerance_rad",
                    0.05
                    if str(profile.get("interactive_yaw_completion_mode", "dwell"))
                    == "dwell"
                    else LOCAL_ACTION_BOUNDS["yaw_tolerance_rad"],
                ),
            ),
            "interactive_yaw_completion_tolerance_rad",
        ),
        "yaw_tolerance_rad": finite_number(
            profile.get(
                "interactive_yaw_completion_tolerance_rad",
                profile.get(
                    "interactive_yaw_tolerance_rad",
                    0.05
                    if str(profile.get("interactive_yaw_completion_mode", "dwell"))
                    == "dwell"
                    else LOCAL_ACTION_BOUNDS["yaw_tolerance_rad"],
                ),
            ),
            "yaw_tolerance_rad",
        ),
        "yaw_settle_rate_radps": finite_number(
            profile.get("interactive_yaw_settle_rate_radps", 0.03),
            "interactive_yaw_settle_rate_radps",
        ),
        "yaw_dwell_sim_s": finite_number(
            profile.get("interactive_yaw_dwell_sim_s", 0.5),
            "interactive_yaw_dwell_sim_s",
        ),
        "yaw_eval_max_gap_sim_s": finite_number(
            profile.get("interactive_yaw_eval_max_gap_sim_s", 0.020001),
            "interactive_yaw_eval_max_gap_sim_s",
        ),
        "imu_max_sim_age_s": finite_number(
            profile.get("interactive_imu_max_sim_age_s", 0.10),
            "interactive_imu_max_sim_age_s",
        ),
        "imu_max_wall_age_s": finite_number(
            profile.get("interactive_imu_max_wall_age_s", 0.25),
            "interactive_imu_max_wall_age_s",
        ),
        "imu_future_tolerance_s": finite_number(
            profile.get("interactive_imu_future_tolerance_s", 2e-6),
            "interactive_imu_future_tolerance_s",
        ),
    }


def yaw_request_parameters(payload):
    """Validate one external relative or map-frame absolute heading request."""
    relative = "delta_yaw_rad" in payload
    absolute = "target_yaw_rad" in payload
    if relative == absolute:
        raise ValueError("yaw requires exactly one of delta_yaw_rad and target_yaw_rad")
    if relative:
        delta = finite_number(payload["delta_yaw_rad"], "delta_yaw_rad")
        if abs(delta) > LOCAL_ACTION_BOUNDS["yaw_max_delta_rad"]:
            raise ValueError("yaw delta out of bounds")
        return {"delta_yaw_rad": delta}
    target = finite_number(payload["target_yaw_rad"], "target_yaw_rad")
    frame = payload.get("frame_id")
    if not isinstance(frame, str) or not frame:
        raise ValueError("absolute_yaw_requires_frame_id")
    return {"target_yaw_rad": target, "frame_id": frame}


def resolve_yaw_target(action, yaw):
    """Freeze the relative/absolute request against the first admitted pose."""
    if action["target_yaw_rad"] is None:
        parameters = action["parameters"]
        action["target_yaw_rad"] = wrap(
            parameters["target_yaw_rad"] if "target_yaw_rad" in parameters
            else yaw + parameters["delta_yaw_rad"]
        )
    return action["target_yaw_rad"]


def yaw_action_step(action, config, context, window, *, yaw, now_wall,
                    future_tolerance_s):
    """Advance the original yaw control/dwell reducer on caller-admitted inputs.

    The caller owns pose/clearance admission, receipts, cancellation and expiry.
    Only the initial target field is frozen in ``action``. The returned window
    keeps the original distinct-pose, IMU-span and evaluation-gap semantics.
    """
    completion = None
    target = resolve_yaw_target(action, yaw)
    error = wrap(target - yaw)
    mode = config["yaw_completion_mode"]
    completion_tol = config["yaw_completion_tolerance_rad"]
    control_tol = config["yaw_control_tolerance_rad"]
    control_band_met = abs(error) <= control_tol
    limit = LOCAL_ACTION_BOUNDS["yaw_rate_rps"]
    if mode == "instant":
        if abs(error) <= completion_tol:
            completion = (
                {
                    "achieved_yaw_rad": yaw,
                    "yaw_error_rad": error,
                    "completion_mode": "instant",
                    "tolerance_rad": completion_tol,
                    "completion_tolerance_rad": completion_tol,
                    "control_tolerance_rad": control_tol,
                    "control_band_met": control_band_met,
                    "completion_band_met": True,
                }
            )
            return [0.0, 0.0, 0.0], window, completion
    else:
        ctx = context
        settle = config["yaw_settle_rate_radps"]
        dwell_target = config["yaw_dwell_sim_s"]
        # A distinct sample already passed the tick's own odometry gates
        # (including its float32-aware future tolerance); a fresh duplicate
        # only has to stay inside the staleness window. No second, narrower
        # future gate is imposed here.
        pose_ok = (
            ctx is not None
            and (ctx.get("distinct") is True or (
                ctx.get("raw_age") is not None
                and 0.0 <= ctx["raw_age"] <= config["max_sim_age_s"]
            ))
            and ctx.get("wall_age") is not None
            and 0.0 <= ctx["wall_age"] <= config["max_wall_age_s"]
        )
        rate_measured = None if ctx is None else ctx.get("imu_yaw_rate")
        gyro_age = None
        gyro_wall_age = None
        if ctx is not None and ctx.get("imu_source_stamp_s") is not None:
            gyro_age = ctx["sim"] - float(ctx["imu_source_stamp_s"])
        if ctx is not None and ctx.get("imu_arrival_wall_s") is not None:
            gyro_wall_age = now_wall - float(ctx["imu_arrival_wall_s"])
        gyro_ok = (
            ctx is not None
            and ctx.get("imu_valid") is not False
            and rate_measured is not None
            and math.isfinite(rate_measured)
            and abs(rate_measured) <= settle
            and gyro_age is not None
            and -future_tolerance_s
            <= gyro_age
            <= config["imu_max_sim_age_s"]
            and gyro_wall_age is not None
            and 0.0 <= gyro_wall_age <= config["imu_max_wall_age_s"]
        )
        valid = pose_ok and gyro_ok and abs(error) <= completion_tol
        record = window
        if valid:
            if record is not None:
                if ctx["distinct"]:
                    record["distinct_since_start"] = True
                    record["last_distinct_stamp_s"] = ctx["stamp"]
                record["last_gyro_stamp_s"] = max(
                    record["last_gyro_stamp_s"] or ctx["imu_source_stamp_s"],
                    float(ctx["imu_source_stamp_s"]),
                )
            continuous = (
                record is not None
                and ctx["sim"] > record["last_sim_s"]
                and ctx["sim"] - record["last_sim_s"]
                <= config["yaw_eval_max_gap_sim_s"]
            )
            if continuous:
                record["valid_sim_s"] += ctx["sim"] - record["last_sim_s"]
                record["last_sim_s"] = ctx["sim"]
            else:
                record = {
                    "start_sim_s": ctx["sim"],
                    "last_sim_s": ctx["sim"],
                    "valid_sim_s": 0.0,
                    "distinct_since_start": ctx["distinct"],
                    "first_distinct_stamp_s": ctx["stamp"] if ctx["distinct"] else None,
                    "last_distinct_stamp_s": ctx["stamp"] if ctx["distinct"] else None,
                    "first_gyro_stamp_s": float(ctx["imu_source_stamp_s"]),
                    "last_gyro_stamp_s": float(ctx["imu_source_stamp_s"]),
                }
        window = record if valid else None
        if valid and window is not None:
            record = window
            source_span = (
                None
                if record["last_distinct_stamp_s"] is None
                else record["last_distinct_stamp_s"] - record["start_sim_s"]
            )
            gyro_span = record["last_gyro_stamp_s"] - record["first_gyro_stamp_s"]
            if (
                record["valid_sim_s"] + 1e-12 >= dwell_target
                and record["distinct_since_start"]
                and source_span is not None
                and source_span + 1e-9 >= dwell_target - 0.05
                and gyro_span + 1e-9 >= 0.25
                and record["valid_sim_s"] <= config["yaw_dwell_sim_s"] + 1.0
            ):
                # Same deadband as the real command path: inside the control
                # band the released command is exactly zero, never a P value.
                correction = (
                    0.0
                    if control_band_met
                    else max(
                        -limit,
                        min(limit, LOCAL_ACTION_BOUNDS["yaw_gain"] * error),
                    )
                )
                record.update(
                    completion_mode="dwell",
                    target_yaw_rad=target,
                    yaw_rad=yaw,
                    yaw_error_rad=error,
                    measured_yaw_rate_rps=rate_measured,
                    yaw_rate_frame="body_imu_z_rate",
                    imu_source_stamp_s=ctx["imu_source_stamp_s"],
                    imu_source_age_s=gyro_age,
                    imu_wall_age_s=gyro_wall_age,
                    imu_frame=ctx["imu_frame"],
                    pose_source_stamp_s=ctx["stamp"],
                    pose_sim_age_s=ctx["raw_age"],
                    pose_wall_age_s=ctx["wall_age"],
                    tolerance_rad=completion_tol,
                    completion_tolerance_rad=completion_tol,
                    control_tolerance_rad=control_tol,
                    control_band_met=control_band_met,
                    completion_band_met=True,
                    pre_release_wz=correction,
                    settle_rate_radps=settle,
                    dwell_target_sim_s=dwell_target,
                    eval_max_gap_sim_s=config["yaw_eval_max_gap_sim_s"],
                    completion_sim_s=ctx["sim"],
                    completion_wall_s=now_wall,
                    last_correction_wz=correction,
                    released_wz=0.0,
                )
                completion = record
                return [0.0, 0.0, 0.0], window, completion
    if control_band_met:
        return [0.0, 0.0, 0.0], window, completion
    rate = LOCAL_ACTION_BOUNDS["yaw_gain"] * error
    return [0.0, 0.0, max(-limit, min(limit, rate))], window, completion
