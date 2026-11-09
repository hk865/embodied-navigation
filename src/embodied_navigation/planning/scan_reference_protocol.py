"""Verify real owned-child successful publication, solely for a local request.

This receipt never populates association revisions, curves or admission. Actual
later attempt/metadata/Bspline must satisfy the unchanged protocol validators.
"""

import math
from embodied_navigation.planning.scan_child_consumption import (
    enabled as consumption_enabled,
    gid_hex,
)

FLAG = "require_reference_protocol_ack"


def enabled(profile):
    config = profile.get("r85_auto_reference_contract", {})
    if not isinstance(config, dict):
        raise ValueError("Planner options must be an object")
    value = config.get(FLAG, False)
    if type(value) is not bool:
        raise ValueError("Planner ACK option must be boolean")
    return value


def _integer(value, minimum=0):
    return type(value) is int and value >= minimum


def publication_eligible(ack, revision, stamp):
    """Pure publication phase check, also used by the protocol-only probe.

    This check does NOT verify source readiness, actual process identity, ACK
    freshness, delivery, or curve admission. Production verified_revision must
    first pass every original child-consumption/source gate below. Counts of
    matched readers are observations and never authorize this fallback.
    """
    if not isinstance(ack, dict) or not _integer(revision, 1):
        raise ValueError("Reference protocol ACK/revision malformed")
    snapshot = ack.get("reference_protocol")
    if (
        not isinstance(snapshot, dict)
        or snapshot.get("schema") != "r85_scan_reference_protocol_snapshot/v2"
        or type(snapshot.get("publish_calls_are_delivery_or_admission")) is not bool
        or snapshot["publish_calls_are_delivery_or_admission"] is not False
    ):
        raise ValueError("Reference protocol publication snapshot missing")
    for key, ack_key in [
        ("run_id", "run_id"),
        ("planner_epoch", "planner_epoch"),
        ("localization_generation", "localization_generation"),
        ("planner_process_id", "planner_pid"),
    ]:
        if type(snapshot.get(key)) is not type(ack.get(ack_key)) or snapshot[key] != ack[ack_key]:
            raise ValueError("Reference protocol publication scope mismatch: " + key)
    accepted_stamp = snapshot.get("reference_stamp")
    if (
        not isinstance(stamp, list)
        or len(stamp) != 2
        or not _integer(stamp[0])
        or not _integer(stamp[1])
        or stamp[1] >= 1_000_000_000
        or not isinstance(accepted_stamp, list)
        or len(accepted_stamp) != 2
        or not _integer(accepted_stamp[0])
        or not _integer(accepted_stamp[1])
        or accepted_stamp[1] >= 1_000_000_000
        or type(snapshot.get("committed_reference_revision")) is not int
        or snapshot["committed_reference_revision"] != revision
        or accepted_stamp != stamp
    ):
        raise ValueError("Reference protocol publication reference mismatch")
    counters = [
        "reference_attempt_publish_count",
        "bspline_publish_count",
        "trajectory_metadata_publish_count",
        "motion_bundle_publish_count",
        "first_gen_entry_count",
        "first_gen_clock_ns",
        "first_gen_source_stamp_ns",
    ]
    if any(not _integer(snapshot.get(k)) for k in counters):
        raise ValueError("Reference protocol publication counters malformed")
    matched = snapshot.get("matched_subscription_counts")
    if not isinstance(matched, dict) or any(
        not _integer(matched.get(k))
        for k in ["reference_attempt", "bspline", "trajectory_metadata"]
    ):
        raise ValueError("Reference protocol publisher subscription counters malformed")
    states = ["INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"]
    if snapshot.get("fsm_state") not in states:
        raise ValueError("Reference protocol real FSM state malformed")
    attempts = [snapshot.get(k) for k in ["first_attempt", "last_attempt", "last_motion_bundle"]]
    for item in attempts:
        if (
            not isinstance(item, dict)
            or type(item.get("completed")) is not bool
            or type(item.get("plan_success")) is not bool
            or not _integer(item.get("id"))
            or not _integer(item.get("local_replan_request_sequence"))
            or not isinstance(item.get("origin"), str)
            or not isinstance(item.get("result_reason"), str)
        ):
            raise ValueError("Reference protocol actual attempt snapshot malformed")
    first, last, bundle = attempts
    # A committed reference alone must never preempt its first GEN attempt.
    if (
        snapshot["fsm_state"] != "EXEC_TRAJ"
        or not first["completed"]
        or not last["completed"]
        or not bundle["completed"]
        or last["plan_success"] is not True
        or bundle["plan_success"] is not True
    ):
        return False
    if (
        not all(snapshot[k] > 0 for k in counters)
        or first["id"] < 1
        or not first["id"] <= bundle["id"] == last["id"]
        or bundle != last
        or bundle["origin"] == "emergency_stop"
        or not bundle["origin"]
        or not bundle["result_reason"]
    ):
        raise ValueError("Reference protocol successful publication completion inconsistent")
    trajectory = snapshot.get("motion_bundle_trajectory")
    start = trajectory.get("start_time") if isinstance(trajectory, dict) else None
    if (
        not isinstance(trajectory, dict)
        or not _integer(trajectory.get("traj_id"), 1)
        or not isinstance(start, list)
        or len(start) != 2
        or not _integer(start[0])
        or not _integer(start[1])
        or start[1] >= 1_000_000_000
    ):
        raise ValueError("Reference protocol motion bundle trajectory identity malformed")
    # Exact full curve association remains the unchanged original consumer gate.
    return True


def verified_revision(profile, ready, reference, scope, clock_ns, wall):
    """Return a real committed revision for request publication, or no commit.

    The existing supervisor has already checked actual /proc/ELF/argv identity,
    unique graph GID and two exact source slots. Validate its unmodified scoped
    receipt, freshness, and the original accepted Path stamp again here.
    """
    if not enabled(profile):
        return None
    if not isinstance(ready, dict) or not isinstance(reference, dict):
        raise ValueError("Reference commit ACK receipt missing")
    epoch, pid, generation = scope
    identity, proof = ready.get("planner_identity"), ready.get("child_odometry_consumption")
    if (
        ready.get("schema") != "known_scene_SCAN_epoch_ready/v1"
        or ready.get("run_id") != reference.get("run_id")
        or ready.get("ready") is not True
        or ready.get("robot_truth_pose_used") is not False
        or any(
            type(ready.get(k)) is not int or ready[k] != v
            for k, v in [
                ("planner_epoch", epoch),
                ("generation", generation),
                ("localization_generation", generation),
            ]
        )
        or any(
            type(reference.get(k)) is not int or reference[k] != v
            for k, v in [
                ("planner_epoch", epoch),
                ("planner_process_id", pid),
                ("localization_generation", generation),
            ]
        )
        or not isinstance(identity, dict)
        or identity.get("pid") != pid
        or any(not _integer(identity.get(k), 1) for k in ("pid", "ppid", "pgid", "start_ticks"))
        or not isinstance(proof, dict)
        or proof.get("ready") is not True
        or proof.get("planner_identity") != identity
        or proof.get("source_timestamp_refreshed") is not False
        or proof.get("original_two_source_and_300ms_gates_retained") is not True
    ):
        raise ValueError("Reference commit ACK original ready/process scope mismatch")
    observed_wall, observed_clock = ready.get("monotonic_wall"), ready.get("ros_clock_ns")
    if (
        type(wall) not in (int, float)
        or not math.isfinite(wall)
        or type(observed_wall) not in (int, float)
        or not math.isfinite(observed_wall)
        or not 0 <= wall - observed_wall < 0.3
        or not _integer(clock_ns, 1)
        or not _integer(observed_clock, 1)
        or not 0 <= clock_ns - observed_clock < 300_000_000
    ):
        raise ValueError("Reference commit ACK supervisor receipt stale")
    ack = proof.get("latest_actual_child_ACK")
    if not isinstance(ack, dict):
        raise ValueError("Reference commit ACK actual child payload missing")
    expected = dict(
        schema="r85_scan_child_odometry_consumed/v1",
        run_id=reference["run_id"],
        planner_epoch=epoch,
        localization_generation=generation,
        planner_pid=pid,
        planner_ppid=identity.get("ppid"),
        planner_start_ticks=identity.get("start_ticks"),
        startup_nonce=proof.get("startup_nonce"),
        source_frame_id="camera_init",
        have_odom=True,
    )
    for key, value in expected.items():
        if type(ack.get(key)) is not type(value) or ack[key] != value:
            raise ValueError("Reference commit ACK actual child scope mismatch: " + key)
    nonce = proof.get("startup_nonce")
    if (
        not isinstance(nonce, str)
        or len(nonce) != 32
        or any(c not in "0123456789abcdef" for c in nonce)
        or gid_hex(ack.get("publisher_gid")) != gid_hex(proof.get("publisher_gid"))
        or not _integer(ack.get("ack_sequence"), 1)
    ):
        raise ValueError("Reference commit ACK nonce/GID/sequence mismatch")
    matched, confirmed, source = (
        proof.get("matched_source_stamps"),
        ready.get("confirmed_source_stamps"),
        ack.get("source_stamp_ns"),
    )
    if (
        not isinstance(matched, list)
        or len(matched) < 2
        or not all(_integer(x, 1) for x in matched)
        or len(set(matched)) != len(matched)
        or not isinstance(confirmed, list)
        or any(x not in confirmed for x in matched)
        or not _integer(source, 1)
        or source != matched[-1]
        or not 0 <= clock_ns - source < 300_000_000
    ):
        raise ValueError("Reference commit ACK original fresh source proof mismatch")
    # Not-yet-committed is normal at bootstrap. A historic counter is no proof.
    if type(ack.get("reference_ready")) is not bool:
        raise ValueError("Reference commit ACK real FSM ready state missing")
    if ack["reference_ready"] is False:
        return None
    revision, stamp = ack.get("committed_reference_revision"), ack.get("reference_stamp")
    expected_stamp = reference.get("sent_reference", {}).get("stamp")
    if (
        not _integer(revision, 1)
        or not isinstance(stamp, list)
        or len(stamp) != 2
        or not _integer(stamp[0])
        or not _integer(stamp[1])
        or stamp[1] >= 1_000_000_000
        or stamp != expected_stamp
        or source < stamp[0] * 1_000_000_000 + stamp[1]
    ):
        raise ValueError("Reference commit ACK accepted Path stamp/revision mismatch")
    # This new fallback is verified only for one Path per owned fresh child.
    # An original Association-observed legitimate later revision bypasses this
    # helper in the caller and retains its original behavior. Never replay a
    # duplicate Path or treat the original replacement callback as idempotent.
    if revision != 1:
        return None
    if not publication_eligible(ack, revision, stamp):
        return None
    return revision
