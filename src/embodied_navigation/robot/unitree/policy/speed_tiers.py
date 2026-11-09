"""Explicit two-tier simulation speed selection; no model or safety authority.

The original Student contract remains the base PID/model identity. This typed
extension changes the longitudinal total command bound, not PID gains or slew.

Only two frozen tiers exist (0.2 m/s and 0.5 m/s cruise). A selected tier is
carried under ``KEY`` ('r109_speed_tier') and every consumer must match the
same frozen dict; this module selects and checks that value, it does not grant
model authority or change PID gains or slew.
"""

import copy

KEY = "r109_speed_tier"
SCHEMA = "r109_simulation_speed_tier/v1"


def choice(cruise):
    """Return the frozen speed-tier dict for an explicit 0.2 or 0.5 m/s cruise.

    The check is strict (``type(cruise) is float``), so an int or bool cruise is
    rejected instead of being accepted as a tier. No other tier is implemented.
    """
    if type(cruise) is not float or cruise not in (0.2, 0.5):
        raise ValueError("Only explicit 0.2 or 0.5 m/s speed tiers are implemented")
    return dict(
        schema=SCHEMA,
        simulation_only=True,
        cruise_mps=cruise,
        # Bind the selected cruise to its longitudinal total command bound:
        # .3 m/s for the .2 tier, .6 m/s for the .5 tier.
        student_longitudinal_limit_mps=0.3 if cruise == 0.2 else 0.6,
        planning_horizon_m=0.65,
    )


def validate(value):
    """Return a copy of one frozen speed tier, or raise ValueError.

    The value must equal the dict produced by ``choice`` for its own
    ``cruise_mps``, and the type of every field is compared as well, so a
    look-alike dict with a different numeric type is rejected. The returned
    dict is a fresh copy rather than the caller's object.
    """
    if type(value) is not dict:
        raise ValueError("Typed R109 speed tier required")
    expected = choice(value.get("cruise_mps"))
    if value != expected or any(type(value.get(k)) is not type(v) for k, v in expected.items()):
        raise ValueError("R109 speed tier fields differ from one of the two frozen choices")
    return dict(expected)


def split_ack(value):
    """Split into (ACK value without the tier key, selected tier or None).

    A non-dict input is returned unchanged with None; a dict without ``KEY`` is
    shallow-copied with None. Only when the key is present is the tier popped
    and validated.
    """
    if not isinstance(value, dict):
        return value, None
    if KEY not in value:
        return dict(value), None
    result = dict(value)
    selected = validate(result.pop(KEY))
    return result, selected


def configuration(profile):
    """Startup only: check the selected profile planner/control fields.

    Returns the selected tier dict, or None when the profile has no tier
    extension. Rejects an ACK tier without a profile selection and mismatches
    in the profile fields listed below. Any ACK tier is parsed as a valid frozen
    choice, but this function does not require it or compare it with the profile
    tier; actual same-frame record equality is checked by ``validate_copies``.
    """
    ack = profile.get("cascade", {}).get("r85_student_lower_loop_ack")
    _, ack_tier = split_ack(ack)
    if KEY not in profile:
        if ack_tier is not None:
            raise ValueError("ACK speed tier lacks its selected profile")
        return None
    selected = validate(profile[KEY])
    cascade = profile.get("cascade", {})
    # Check the profile fields below against the selected frozen tier.
    # ACK-to-profile tier equality is not checked in this condition.
    if (
        profile.get("actor_kind") != "student"
        or profile.get("controller_kind") != "student"
        or type(profile.get("navigation_ground_truth_used")) is not bool
        or profile.get("max_speed_mps") != selected["cruise_mps"]
        or cascade.get("desired_speed") != selected["cruise_mps"]
        or cascade.get("command_limits") != [0.6, 0.2, 0.6]
        or cascade.get("command_slew_per_s") != [0.6, 0.6, 0.8]
        or profile.get("continuous_route_contract", {}).get("local_planning_horizon_m")
        != selected["planning_horizon_m"]
        or profile.get("max_lateral_speed_mps") != 0.2
        or profile.get("max_yaw_rate_radps") != 0.6
    ):
        raise ValueError("R109 planner/navigation/Student ACK speed selection is inconsistent")
    return selected


def student_bounds(selected):
    """Body-axis Student PID command bounds; lateral/yaw stay at the base values.

    Without a tier the original .3 m/s longitudinal bound is kept; with a tier
    only the longitudinal bound changes.
    """
    return (
        [0.3, 0.2, 0.3]
        if selected is None
        else [validate(selected)["student_longitudinal_limit_mps"], 0.2, 0.3]
    )


def validate_copies(config, *records):
    """Require every actual same-frame record to carry the selected tier.

    ``records`` are the PID / Actor / ACK dicts of one actual frame. If the
    configuration has no tier, no record may carry one; if it has a tier, every
    record must carry the identical frozen dict. Returns the selected tier, or
    None when no tier is selected.
    """
    _, selected = split_ack(config)
    for record in records:
        if not isinstance(record, dict):
            raise ValueError("Actual same-frame speed-tier record required")
        if selected is None:
            if KEY in record:
                raise ValueError("Unselected ACK cannot carry a speed-tier extension")
        elif validate(record.get(KEY)) != selected:
            raise ValueError("Actual PID/Actor/ACK speed tier differs or is missing")
    return selected


def select_profile(profile, cruise):
    """Copy an existing authorized run profile; never renew its time/resources.

    The profile is deep-copied and only the speed-tier fields are rewritten;
    ``configuration`` then re-validates the result before it is returned.
    """
    selected = choice(cruise)
    result = copy.deepcopy(profile)
    result[KEY] = selected
    result["max_speed_mps"] = cruise
    result["cascade"]["desired_speed"] = cruise
    result["cascade"]["r85_student_lower_loop_ack"][KEY] = dict(selected)
    result["continuous_route_contract"]["local_planning_horizon_m"] = selected["planning_horizon_m"]
    configuration(result)
    return result
