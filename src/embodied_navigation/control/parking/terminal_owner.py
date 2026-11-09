"""Opt-in terminal capture owner after real final-region arrival, without SCAN."""

import copy, os, time
import numpy as np
from embodied_navigation.control.parking.terminal_reference import KIND, digest, validate_reference
from embodied_navigation.localization_mapping.known_scene_hold_guard import _frame

_OPTION = os.environ.get("R105_TERMINAL_PARKING_OWNER", "0")
if _OPTION not in ("0", "1"):
    raise ValueError("R105_TERMINAL_PARKING_OWNER must be 0 or 1")
TERMINAL_PARKING_OWNER_ENABLED = _OPTION == "1"


def selected(node):
    """Whether this node instance has the opt-in terminal owner bound."""
    return getattr(node, "_r105_terminal_owner_enabled", False)


def invalidate(node, reason):
    """For a selected owner, latch failure after advancement and clear its context."""
    if not selected(node):
        return
    owner = getattr(node, "terminal_parking_owner", None)
    core = getattr(node, "cascade", None)
    if (
        owner is not None
        and getattr(node, "terminal_parking_request", None) == node.request_id
        and (
            owner.ready_stamp is not None
            or owner.first_actor_zero_ack is not None
            or core is not None
            and core.ordinary_zero_event is not None
        )
    ):
        owner.guard(reason)
        node.ordinary_terminal_failure_latched = dict(
            request_id=node.request_id,
            reason=owner.failure or reason,
            source="r105_coordinate_owner_invalidated",
            control_stamp_ns=node.get_clock().now().nanoseconds,
        )
        node.state = "failed"
    node._r105_terminal_context = None


def validate_context(node):
    """Recheck reference, ownership, goal, frame and deadline before every owner tick."""
    context = node._r105_terminal_context
    core = node.cascade
    record = context["record"]
    validate_reference(record, context["sha256"])
    index = record["waypoint_index"]
    if (
        core is not context["core"]
        or core.terminal_reference != record
        or core.path is not None
        or core.path_sha is not None
        or core.path_id is not None
        or node.path_receipt is not None
        or node.active_trajectory_id is not None
        or node.samples is not None
        or node.request_id != record["request_id"]
        or node.goals_definition_sha256 != record["goals_definition_sha256"]
        or not np.array_equal(node.waypoints[index], record["fixed_goal"]["position_world_xyz"])
        or digest(_frame(node)) != record["coordinate_frame_sha256"]
        or node.automatic_reference.scope[2] != record["generation"]
        or node.fixed_terminal_heading(index, node.segment_start)
        != record["fixed_goal"]["heading_rad"]
        or node.get_clock().now().nanoseconds >= record["deadline_clock_ns"]
    ):
        raise ValueError("Terminal capture owner/coordinate/goal/deadline changed")
    return record


def control(node, now):
    """True owns this control tick; region success remains an independent fact."""
    if not selected(node) or node.state != "succeeded":
        return False
    context = getattr(node, "_r105_terminal_context", None)
    if context is None and node.final_bundle is not None:
        return False  # original checked SCAN parking
    ns = node.get_clock().now().nanoseconds
    if context is not None:
        validate_context(node)
        node.control_parking(now)
        return True
    if not node.region_arrivals or node.waypoint_index != len(node.waypoints):
        raise ValueError("Terminal owner lacks final-region completion")
    index = len(node.waypoints) - 1
    arrival = node.region_arrivals[-1]
    deadline = arrival["goal_activated_ros_clock_ns"] + round(node.goals[index].timeout_sim_s * 1e9)
    if ns >= deadline:
        raise ValueError("Original terminal goal deadline exceeded")
    old = getattr(node, "terminal_parking_owner", None)
    if old is not None and getattr(node, "terminal_parking_request", None) == node.request_id:
        if old.failure or old.ready_stamp is not None or old.first_actor_zero_ack is not None:
            raise ValueError("Cannot replace an existing qualified/failed terminal owner")
    # Consume real safety intake even when no new execution path is available.
    protected = node.apply_tilt_guard(now, ns / 1e9)
    fresh = (
        node._known_generation_ready()
        and node.pose_stamp == node.cloud_stamp == node.known_scene_inputs["source_ns"]
        and 0 <= ns - node.pose_stamp < 300_000_000
        and all(
            0 <= now - stamp < 0.3
            for stamp in (node.pose_updated, node.cloud_updated, node.bridge_updated)
        )
        and node.raw_imu.fresh(now, ns / 1e9)
        and node.bridge_safety.get("state") == "ready"
    )
    if protected or not fresh or node.obstacle_hold:
        node.publish_command()
        node.stale_since = node.stale_since or now
        if now - node.stale_since > 8.0:
            raise ValueError("Original terminal sources unavailable beyond 8 seconds")
        return True
    node.stale_since = None
    if node.paired_imu is None and node.imu_source_buffer is not None:
        node.paired_imu = copy.deepcopy(node.imu_source_buffer.pair(node.pose_stamp, None, ns))
    imu = node.paired_imu
    feedback = node.feedback
    if (
        imu is None
        or feedback is None
        or feedback["stamp_ns"] != node.pose_stamp
        or not 0 <= node.pose_stamp - imu["stamp_ns"] <= 20_000_000
        or not 0 <= ns - imu["stamp_ns"] < 300_000_000
        or not 0 <= time.monotonic_ns() - imu["received_wall_ns"] < 300_000_000
    ):
        node.publish_command()
        return True  # original task/goal deadline remains fixed
    if (
        node.samples is not None
        or node.path_receipt is not None
        or node.active_trajectory_id is not None
    ):
        raise ValueError("Pathless terminal owner cannot borrow an unbundled trajectory")
    runtime = node.automatic_reference
    if (
        runtime is None
        or runtime.scope is None
        or runtime.scope[2] != node.known_scene_inputs["generation"]
    ):
        raise ValueError("Terminal goal heading lacks the current registered automatic reference")
    from embodied_navigation.control.tracking.cascade_core import Controller
    from embodied_navigation.common.input_freshness import enabled as minimal_enabled
    from embodied_navigation.control.parking.parking_contract import enabled as ordinary_enabled

    if not ordinary_enabled(node.profile):
        raise ValueError("R105 requires original ordinary parking contract")
    goal = node.waypoints[index]
    start = node.pose.copy()
    yaw = node.fixed_terminal_heading(index, start)
    key = (node.request_id, index)
    core = Controller(node.profile["cascade"], goal.tolist(), yaw, ":".join(map(str, key)))
    core.minimal_simulation_baseline = minimal_enabled(node.profile)
    record = dict(
        schema=KIND,
        run_id=node.run.name,
        request_id=node.request_id,
        waypoint_index=index,
        goals_definition_sha256=node.goals_definition_sha256,
        region_receipt=copy.deepcopy(arrival),
        fixed_goal=copy.deepcopy(core.goal_record),
        fixed_goal_sha256=core.goal_sha,
        generation=node.known_scene_inputs["generation"],
        coordinate_frame=copy.deepcopy(_frame(node)),
        coordinate_frame_sha256=digest(_frame(node)),
        source_pose_stamp_ns=node.pose_stamp,
        source_cloud_stamp_ns=node.cloud_stamp,
        paired_imu_stamp_ns=imu["stamp_ns"],
        source_pose_received_wall_ns=feedback["received_wall_ns"],
        source_cloud_received_wall_ns=round(node.cloud_updated * 1e9),
        source_imu_received_wall_ns=imu["received_wall_ns"],
        created_control_clock_ns=ns,
        created_wall_ns=time.monotonic_ns(),
        deadline_clock_ns=deadline,
        path_geometry_present=False,
        navigation_ground_truth_used=False,
    )
    core.configure_terminal_reference(record)
    # A single control-owner transaction; no callback boundary or fabricated SCAN ID.
    node.cascade = core
    node.cascade_key = key
    node.cascade_path_id = None
    node._r105_terminal_context = dict(record=record, sha256=validate_reference(record), core=core)
    node.final_bundle = dict(
        kind=KIND,
        index=index,
        start=start,
        trajectory_id=None,
        array=None,
        path_receipt=None,
        reference_stamp=None,
    )
    node.evidence.append(
        node.run / "r105_terminal_owner.jsonl",
        dict(
            event="created",
            reference=record,
            reference_sha256=node._r105_terminal_context["sha256"],
            region_fact_preserved=True,
            SCAN_motion_bundle_fabricated=False,
            first_T0_set=False,
        ),
    )
    node.control_parking(now)
    return True
