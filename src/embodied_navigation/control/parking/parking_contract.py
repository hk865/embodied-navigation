"""Explicit approved R80 first-qualified-zero-ACK parking contract; no IO."""

import copy
import math
import numpy as np
from embodied_navigation.control.parking.parking_zero_contract import validate_zero_ack, integer

# Retained v1 legacy contract (original zero-ACK fixed-5 s mode). It is no longer
# selected by the live runtime -- see MODE/SPEC below -- and is kept only so the
# old document shape remains checkable.
LEGACY_MODE = "ordinary_zero_ack_fixed5"
LEGACY_SPEC = dict(
    schema="ordinary_terminal_zero_ack_fixed5/v1",
    enabled=True,
    T0="first_core_precise_pose_and_actual_zero_input_ACK_qualification_clock",
    duration_ns=5_000_000_000,
    reset_window=False,
    keep_legacy_arrival_window=True,
    capture_timeout="original_active_goal_timeout_s_from_separate_capture_begin",
    capture_xy_m=0.025,
    capture_yaw_rad=0.035,
    capture_speed_mps=0.03,
    capture_yawdot_radps=0.06,
    capture_dwell_ns=600_000_000,
    xy_drift_m=0.05,
    yaw_drift_rad=0.1,
    native_speed_mps=0.08,
    native_wz_radps=0.1,
    native_yawdot_radps=0.1,
    source_age_ns=300_000_000,
)
MODE = "ordinary_pose10hz_zero_ack_fixed5"
# Import the 10 Hz capture configuration used by the current SPEC.
from embodied_navigation.control.parking.parking_pose10hz import CONFIG as POSE_CONFIG

SPEC = dict(
    schema="ordinary_pose10hz_zero_ack_fixed5/v1",
    enabled=True,
    T0="first_10hz_geometric_capture_and_actual_zero_ACK_qualification_clock",
    duration_ns=5_000_000_000,
    reset_window=False,
    keep_legacy_arrival_window=True,
    capture_timeout="original_active_goal_timeout_s_from_separate_capture_begin",
    capture_xy_m=0.025,
    capture_yaw_rad=0.035,
    capture_dwell_ns=600_000_000,
    pose10hz=copy.deepcopy(POSE_CONFIG),
    source_age_ns=300_000_000,
)


def enabled(profile):
    value = profile.get("ordinary_terminal_parking_contract")
    if value is None:
        return False
    from embodied_navigation.control.safety.safety_configuration import parking

    return parking(value)
