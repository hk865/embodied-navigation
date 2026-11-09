"""Ordinary terminal zero identity; contains no controller, Actor or native safety."""

import hashlib
import json
import math
from numbers import Real

SCHEMA = "ordinary_terminal_zero_event/v1"
TERMINAL_SCHEMA = "ordinary_terminal_zero_event/v2"
TERMINAL_IDENTITY = ("event_domain", "reference_kind", "reference_sha256")
IDENTITY = (
    "run_id",
    "request_id",
    "goal_sha256",
    "path_sha256",
    "event_sequence",
    "ready_pose_stamp_ns",
    "handoff_clock_ns",
    "created_cascade_sequence",
)
FRAME_SCHEMA = "ordinary_zero_validated_input_frame/v1"
FRAME_IDENTITY = ("frame_serial", "input_clock_ns", "producer_sequence")
CONTROL_IDENTITY = (
    "run_id",
    "request_id",
    "event_sequence",
    "ready_pose_stamp_ns",
    "handoff_clock_ns",
    "created_cascade_sequence",
)


def integer(value):
    """Return one nonnegative integer stamp/sequence; bools are rejected."""
    if type(value) is not int or value < 0:
        raise ValueError("Expected original nonnegative integer stamp/sequence")
    return value


def exact_zero(values):
    """True only for three finite real zeros (bools are not numbers here)."""
    return len(values) == 3 and all(
        isinstance(x, Real) and not isinstance(x, bool) and math.isfinite(x) and x == 0
        for x in values
    )


def event_id(identity):
    """Deterministic ``run_id:request_id:zero:<event_sequence>:<created>`` identity."""
    return (
        str(identity["run_id"])
        + ":"
        + str(identity["request_id"])
        + ":zero:"
        + str(identity["event_sequence"])
        + ":"
        + str(identity["created_cascade_sequence"])
    )


def identity_keys(marker):
    if marker.get("schema") == SCHEMA:
        if any(k in marker for k in TERMINAL_IDENTITY):
            raise ValueError("v1 cannot carry terminal reference identity")
        return CONTROL_IDENTITY
    if marker.get("schema") == TERMINAL_SCHEMA:
        from embodied_navigation.control.parking.terminal_reference import KIND

        if (
            marker.get("event_domain") != TERMINAL_SCHEMA
            or marker.get("reference_kind") != KIND
            or marker.get("path_sha256") is not None
        ):
            raise ValueError("Invalid v2 terminal event domain or fake path")
        return CONTROL_IDENTITY
    raise ValueError("Unknown ordinary zero event schema")


def make_event(
    run_id,
    request_id,
    goal_sha256,
    path_sha256,
    event_sequence,
    ready_pose_stamp_ns,
    handoff_clock_ns,
    created_cascade_sequence,
    *,
    terminal_reference=None,
):
    seed = dict(
        zip(
            IDENTITY,
            (
                run_id,
                request_id,
                goal_sha256,
                path_sha256,
                event_sequence,
                ready_pose_stamp_ns,
                handoff_clock_ns,
                created_cascade_sequence,
            ),
        )
    )
    if terminal_reference is None:
        return dict(schema=SCHEMA, **seed, event_id=event_id(seed))
    from embodied_navigation.control.parking.terminal_reference import validate_reference, KIND

    value = validate_reference(terminal_reference)
    if (
        path_sha256 is not None
        or terminal_reference["run_id"] != run_id
        or terminal_reference["request_id"] != request_id
        or terminal_reference["fixed_goal_sha256"] != goal_sha256
    ):
        raise ValueError("v2 terminal event lost reference owner")
    seed.update(event_domain=TERMINAL_SCHEMA, reference_kind=KIND, reference_sha256=value)
    return dict(schema=TERMINAL_SCHEMA, **seed, event_id=event_id(seed))


def validate_event(marker, lineage, run_id, command, native_clock_ns, *, source_max_age_ns):
    """Caller must first retain all ordinary producer/source health and TTL gates."""
    if not isinstance(marker, dict) or marker.get("schema") not in (SCHEMA, TERMINAL_SCHEMA):
        raise ValueError("Not an ordinary terminal zero event")
    if not isinstance(lineage, dict) or lineage.get("navigation_ground_truth_used") is not False:
        raise ValueError("Zero event has no original non-GT controller lineage")
    if not exact_zero(command) or not exact_zero(lineage.get("command_after_slew", [])):
        raise ValueError("Ordinary zero event must bind exact final body command")
    seed = {k: marker[k] for k in identity_keys(marker)}
    if marker.get("event_id") != event_id(seed):
        raise ValueError("Ordinary zero event identity changed")
    if lineage.get("ordinary_zero_event_id") != marker["event_id"] or integer(
        lineage.get("ordinary_zero_event_sequence")
    ) != integer(marker["event_sequence"]):
        raise ValueError("Zero event is not the current controller event")
    if (
        marker["run_id"] != run_id
        or not isinstance(marker["request_id"], str)
        or not marker["request_id"].startswith(run_id + ":")
    ):
        raise ValueError("Wrong zero event run/request")
    for key in ("request_id",):
        if marker[key] != lineage.get(key):
            raise ValueError("Zero event lost current controller identity: " + key)
    if marker["schema"] == TERMINAL_SCHEMA:
        from embodied_navigation.control.parking.terminal_reference import validate_fields

        record = validate_fields(lineage, run_id)
        if record is None or marker["reference_sha256"] != lineage["reference_sha256"]:
            raise ValueError("v2 zero event lost typed terminal lineage")
    elif any(k in lineage for k in ("reference_kind", "terminal_reference", "reference_sha256")):
        raise ValueError("v1 zero event cannot impersonate a terminal reference")
    seq, ready, handoff, created = (
        integer(marker[k])
        for k in (
            "event_sequence",
            "ready_pose_stamp_ns",
            "handoff_clock_ns",
            "created_cascade_sequence",
        )
    )
    pose = integer(lineage["source_pose_stamp_ns"])
    gyro = integer(lineage["paired_imu_stamp_ns"])
    control = integer(lineage["control_stamp_ns"])
    current = integer(lineage["cascade_sequence"])
    clock = integer(native_clock_ns)
    if (
        seq < 1
        or created < 1
        or current < created
        or not ready <= pose <= control
        or not ready <= handoff <= control
    ):
        raise ValueError("Zero handoff/source ordering invalid")
    if (
        not 0 <= clock - pose <= integer(source_max_age_ns)
        or not 0 <= pose - gyro <= 20_000_000
        or control > clock
    ):
        raise ValueError("Ordinary zero source is stale/future or not causal")
    if lineage.get("ordinary_parking_stage") not in ("handoff_zero", "zero_hold"):
        raise ValueError("Zero event not owned by current ordinary terminal stage")
    return dict(marker)


def frame_identity(frame):
    """Return the required current accepted input-frame identity.

    Every field in ``FRAME_IDENTITY`` must be present; serials must be positive
    and all stamps/sequences nonnegative integers. The identity is what binds an
    ACK proof to one concrete controller input frame.
    """
    if not isinstance(frame, dict):
        raise ValueError("No current accepted input-frame identity")
    result = {key: frame[key] for key in FRAME_IDENTITY}
    for key in ("frame_serial", "input_clock_ns", "producer_sequence"):
        integer(result[key])
    if result["frame_serial"] < 1 or result["producer_sequence"] < 1:
        raise ValueError("Input frame/producer sequence must be positive")
    return result


def authorize_frame(marker, lineage, run_id, command, frame, *, source_max_age_ns):
    """Call only after this frame passed original producer/source/health gates.

    The wrapper clears authorization before every request, including duplicate
    clocks, and revokes it on rejection, provider override, dynamic stop/resume,
    or another request. A frame serial is local; it is never a producer stamp.
    """
    current = frame_identity(frame)
    event = validate_event(
        marker,
        lineage,
        run_id,
        command,
        current["input_clock_ns"],
        source_max_age_ns=source_max_age_ns,
    )
    return dict(
        schema=FRAME_SCHEMA,
        event=event,
        frame=current,
        cascade_sequence=lineage["cascade_sequence"],
        source_pose_stamp_ns=lineage["source_pose_stamp_ns"],
        paired_imu_stamp_ns=lineage["paired_imu_stamp_ns"],
        control_stamp_ns=lineage["control_stamp_ns"],
    )


def before_actor_observation(cmd, request, authorized_frame, clock_ns, *, frame):
    """Apply only this accepted frame, after slew and before observation creation."""
    if authorized_frame is None:
        return None
    if (
        not isinstance(authorized_frame, dict)
        or authorized_frame.get("schema") != FRAME_SCHEMA
        or frame_identity(frame) != authorized_frame.get("frame")
    ):
        raise ValueError("Zero authorization belongs to another input frame")
    if not exact_zero(request):
        raise ValueError("A resumed/nonzero request cannot receive zero takeover")
    clock = integer(clock_ns)
    if clock != authorized_frame["frame"]["input_clock_ns"]:
        raise ValueError("Zero authorization clock differs from observation frame")
    authorized_event = authorized_frame["event"]
    if clock < integer(authorized_event["handoff_clock_ns"]):
        raise ValueError("Actor zero takeover precedes its handoff")
    if len(cmd) != 3 or not all(
        isinstance(x, Real) and not isinstance(x, bool) and math.isfinite(x) for x in cmd
    ):
        raise ValueError("Do not conceal a malformed/nonfinite Actor command")
    before = [float(x) for x in cmd]
    cmd[:] = [0.0, 0.0, 0.0]
    # This proves velocity input only. Actor, previous_action, targets and native
    # PD/fault processing remain with their original owners.
    result = dict(
        schema="ordinary_zero_before_actor_observation/v1",
        event_id=authorized_event["event_id"],
        run_id=authorized_event["run_id"],
        request_id=authorized_event["request_id"],
        goal_sha256=authorized_event["goal_sha256"],
        path_sha256=authorized_event["path_sha256"],
        ready_pose_stamp_ns=authorized_event["ready_pose_stamp_ns"],
        handoff_clock_ns=authorized_event["handoff_clock_ns"],
        created_cascade_sequence=authorized_event["created_cascade_sequence"],
        event_sequence=authorized_event["event_sequence"],
        input_clock_ns=clock,
        frame_serial=authorized_frame["frame"]["frame_serial"],
        producer_sequence=authorized_frame["frame"]["producer_sequence"],
        source_pose_stamp_ns=authorized_frame["source_pose_stamp_ns"],
        paired_imu_stamp_ns=authorized_frame["paired_imu_stamp_ns"],
        control_stamp_ns=authorized_frame["control_stamp_ns"],
        cascade_sequence=authorized_frame["cascade_sequence"],
        previous_slew_command=before,
        observed_command_body=[0.0, 0.0, 0.0],
        before_observation_construction=True,
        policy_action_replaced=False,
        previous_action_cleared=False,
        native_fault_policy_changed=False,
    )
    if authorized_event["schema"] == TERMINAL_SCHEMA:
        result.update({k: authorized_event[k] for k in TERMINAL_IDENTITY})
    return result


def validate_zero_observation(proof, marker, input_clock_ns):
    """Verify this actual Actor input frame independently of its later ACK."""
    if (
        not isinstance(proof, dict)
        or proof.get("schema") != "ordinary_zero_before_actor_observation/v1"
    ):
        raise ValueError("Missing actual ordinary zero observation receipt")
    for key in (*identity_keys(marker), "event_id"):
        if proof.get(key) != marker.get(key):
            raise ValueError("Zero observation event identity differs: " + key)
    if marker.get("event_id") != event_id({key: marker[key] for key in identity_keys(marker)}):
        raise ValueError("Zero observation event hash differs")
    frame_identity({key: proof[key] for key in FRAME_IDENTITY})
    for key in (
        "event_sequence",
        "ready_pose_stamp_ns",
        "handoff_clock_ns",
        "created_cascade_sequence",
        "source_pose_stamp_ns",
        "paired_imu_stamp_ns",
        "control_stamp_ns",
        "cascade_sequence",
    ):
        integer(proof[key])
    clock = integer(input_clock_ns)
    pose = proof["source_pose_stamp_ns"]
    control = proof["control_stamp_ns"]
    if (
        proof["input_clock_ns"] != clock
        or not marker["ready_pose_stamp_ns"] <= marker["handoff_clock_ns"] <= clock
        or not marker["ready_pose_stamp_ns"] <= pose <= control <= clock
        or not 0 <= clock - pose <= 300_000_000
        or not 0 <= pose - proof["paired_imu_stamp_ns"] <= 20_000_000
        or proof["cascade_sequence"] < marker["created_cascade_sequence"]
    ):
        raise ValueError("Zero observation input/source clocks are not causal and fresh")
    if not all(
        proof.get(key) is True
        for key in (
            "before_observation_construction",
            "observation_command_exact_zero",
            "actor_inferred_this_frame",
        )
    ):
        raise ValueError("Zero observation lacks actual Actor construction/inference proof")
    if not all(
        proof.get(key) is False
        for key in (
            "policy_action_replaced",
            "previous_action_cleared",
            "native_fault_policy_changed",
        )
    ):
        raise ValueError("Ordinary zero altered Actor/native ownership")
    if not exact_zero(proof["observed_command_body"]) or not exact_zero(
        proof["actual_observation_command_body"]
    ):
        raise ValueError("Actual ordinary observation was not exact zero")
    return dict(proof)


def validate_zero_ack(ack, marker, clock_ns):
    """Verify a completed actual zero-input frame against its current event.

    Caller retains original ACK freshness/ordering and source controls. This
    proof contains known velocity/identity only, never native navigation pose.
    """
    proof = ack.get("ordinary_zero_before_actor_observation")
    if (
        not isinstance(proof, dict)
        or proof.get("schema") != "ordinary_zero_before_actor_observation/v1"
    ):
        raise ValueError("No typed zero observation receipt in actual ACK")
    for key in (*identity_keys(marker), "event_id"):
        if proof.get(key) != marker.get(key):
            raise ValueError("Actual zero ACK lost event identity: " + key)
    if marker.get("event_id") != event_id({key: marker[key] for key in identity_keys(marker)}):
        raise ValueError("Actual zero ACK marker identity changed")
    frame_identity({key: proof[key] for key in FRAME_IDENTITY})
    for key in (
        "event_sequence",
        "ready_pose_stamp_ns",
        "handoff_clock_ns",
        "created_cascade_sequence",
        "source_pose_stamp_ns",
        "paired_imu_stamp_ns",
        "control_stamp_ns",
        "cascade_sequence",
    ):
        integer(proof[key])
    stamp = integer(ack["stamp_ns"])
    native = integer(ack["ack_native_clock_ns"])
    clock = integer(clock_ns)
    validate_zero_observation(proof, marker, stamp)
    if (
        proof["input_clock_ns"] != stamp
        or not marker["handoff_clock_ns"] <= stamp < native <= clock
        or not marker["ready_pose_stamp_ns"] <= proof["source_pose_stamp_ns"] <= stamp
        or not 0 <= proof["source_pose_stamp_ns"] - proof["paired_imu_stamp_ns"] <= 20_000_000
        or not proof["source_pose_stamp_ns"] <= proof["control_stamp_ns"] <= stamp
        or proof["cascade_sequence"] < marker["created_cascade_sequence"]
        or ack.get("navigation_ground_truth_used") is not False
    ):
        raise ValueError("Actual zero ACK is not a causal current input frame")
    if not all(
        proof.get(key) is True
        for key in (
            "before_observation_construction",
            "observation_command_exact_zero",
            "actor_inferred_this_frame",
        )
    ):
        raise ValueError("Actual zero ACK has no verified Actor observation input")
    if not all(
        exact_zero(values)
        for values in (
            ack["requested_command_body"],
            ack["applied_command_body"],
            proof["observed_command_body"],
            proof["actual_observation_command_body"],
        )
    ):
        raise ValueError("Actual zero ACK velocity/observation was not exact zero")
    return dict(proof, ack_native_clock_ns=native, ack_sequence=integer(ack["sequence"]))
