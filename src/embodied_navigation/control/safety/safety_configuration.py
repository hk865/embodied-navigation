"""Current numerical safety settings; no release, hash, or approval documents."""

import math


def _number(value, expected, name):
    """Require a finite number exactly equal to the reviewed constant."""
    if type(value) not in (int, float) or not math.isfinite(value) or value != expected:
        raise ValueError("Invalid safety setting: " + name)


def parking(value):
    """Validate frozen fixed-window parking numerics, fixed anchor and 10 Hz pose settings."""
    if not isinstance(value, dict) or value.get("enabled") is not True:
        raise ValueError("Fixed-window parking must be enabled")
    for key, number in dict(
        duration_ns=5_000_000_000,
        capture_xy_m=0.025,
        capture_yaw_rad=0.035,
        capture_dwell_ns=600_000_000,
        source_age_ns=300_000_000,
    ).items():
        _number(value.get(key), number, key)
    if value.get("reset_window") is not False:
        raise ValueError("Parking anchor must remain fixed")
    pose(value.get("pose10hz"))
    return True


def pose(value):
    """Validate causal 10 Hz pose-sampling periods/drifts and sampling policies."""
    if not isinstance(value, dict):
        raise ValueError("Pose sampling settings required")
    for key, number in dict(
        period_ns=100_000_000,
        duration_ns=5_000_000_000,
        xy_drift_m=0.05,
        yaw_drift_rad=0.1,
        source_age_ns=300_000_000,
    ).items():
        _number(value.get(key), number, key)
    if (
        value.get("sampling") != "latest_distinct_header_at_or_before_grid"
        or value.get("interpolation") is not False
        or value.get("high_frequency_motion_gate") is not False
        or value.get("adjacent_increment_gate") is not False
    ):
        raise ValueError("Actual causal pose sampling required")
    return True


def clock(value):
    """Validate clock freshness/backstop numerics and the clock-hold policies."""
    if not isinstance(value, dict):
        raise ValueError("Clock protection settings required")
    _number(value.get("freshness_sim_and_wall_s"), 0.3, "clock source freshness")
    _number(value.get("continuous_stale_fail_wall_s"), 8.0, "continuous stale stop")
    if (
        value.get("same_clock_policy") != "exact_zero_before_math_and_native_geometry"
        or value.get("backward_clock_policy") != "latch_failure"
        or value.get("source_receipt_refresh_permitted") is not False
    ):
        raise ValueError("Clock hold protection required")


def settle(value):
    """Validate settle-window numerics plus the settle phase and measured-stop policy."""
    if not isinstance(value, dict) or value.get("enabled") is not True:
        raise ValueError("Settle window required")
    for key, number in dict(
        window_s=1.0,
        gyro_mean_norm_radps=0.005,
        gyro_rms_norm_radps=0.05,
        gyro_peak_norm_radps=0.1,
        euler_peak_to_peak_rad=0.003,
        euler_net_change_rad=0.0015,
        xy_diameter_m=0.003,
        xy_net_change_m=0.002,
    ).items():
        _number(value.get(key), number, key)
    if (
        value.get("eligible_phase") != "settle"
        or value.get("legacy_stop_predicate_retained") is not True
    ):
        raise ValueError("Settle phase and measured stop protection required")
    return True
