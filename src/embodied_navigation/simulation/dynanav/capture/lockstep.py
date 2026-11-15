"""Bounded render/consumer waits for an opt-in NON-REALTIME static simulation.

This is scheduling only, never a motion permission or a freshness exemption.
No clock/IMU is published here, no sensor stamp/arrival is changed, and no
policy/tracker tick is repeated. A timeout returns to the normal guarded loop.
"""
from __future__ import annotations

import hashlib
import json
import math
import time


MODE = "nonrealtime_static_sensor_lockstep"
DEFAULT_MAX_WAIT_S = 0.15
MAX_WAIT_S = 0.20
# The existing consumer's /clock receipt limit is 0.25 s. Leave margin and
# never emit a paused-clock heartbeat to extend it.
MAX_CLOCK_PUBLISH_AGE_S = 0.20


def validate_profile(profile):
    enabled = profile.get("nonrealtime_sensor_lockstep", False)
    if not isinstance(enabled, bool):
        raise ValueError("nonrealtime_sensor_lockstep must be an explicit boolean")
    if not enabled:
        return None
    if (profile.get("point_cloud_source") != "rtx_lidar"
            or profile.get("render_backend") != "fabric"
            or profile.get("control_mode") != "interactive_waypoint_exploration"
            or profile.get("interactive_simulation_only") is not True
            or profile.get("interactive_perception_owner") != "rtx_deskewed_cloud"
            or profile.get("simulated_nearfield_depth") is not True
            or profile.get("pedestrian")):
        raise ValueError("Nonrealtime lockstep requires static RTX/Fabric interactive simulation and downward depth")
    wait = float(profile.get("sensor_lockstep_max_wait_s", DEFAULT_MAX_WAIT_S))
    if not math.isfinite(wait) or not 0.0 < wait <= MAX_WAIT_S:
        raise ValueError("sensor_lockstep_max_wait_s must be within (0, 0.20] wall seconds")
    return {
        "mode": MODE, "enabled": True, "realtime": False,
        "scope": "static simulation benchmark only; not an asynchronous pipeline or hardware performance fix",
        "max_wait_wall_s": wait,
        "max_clock_publish_age_s": MAX_CLOCK_PUBLISH_AGE_S,
        "sensor_period_sim_s": 0.1,
        "source_stamps_or_arrivals_changed": False,
        "paused_clock_heartbeat": False, "safety_thresholds_changed": False,
        "timeout_behavior": "return to original control/physics loop and its hold/terminal gates",
    }


def simulation_snapshot(art):
    """Read independent clock/state invariants; native pose is audit-only."""
    import numpy as np
    import omni.timeline
    import omni.usd
    import usdrt
    from isaacsim.core.simulation_manager import SimulationManager

    stage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
    prim = stage.GetPrimAtPath("/ExternalSimulationTime")
    if not prim or not prim.HasAttribute("omni:time"):
        raise RuntimeError("Lockstep requires the real Fabric /ExternalSimulationTime")
    fabric_s = float(prim.GetAttribute("omni:time").Get())
    simulation_s = float(SimulationManager.get_simulation_time())
    timeline_s = float(omni.timeline.get_timeline_interface().get_current_time())
    if not all(math.isfinite(value) for value in (fabric_s, simulation_s, timeline_s)):
        raise RuntimeError("Nonfinite lockstep clock invariant")
    if abs(fabric_s - simulation_s) > 1e-6:
        raise RuntimeError("Fabric simulation clock differs from physical time")
    p, q = art.get_world_poses()
    v, w = art.get_velocities()
    arrays = (p, q, v, w, art.get_dof_positions(), art.get_dof_velocities())
    digest = hashlib.sha256()
    for value in arrays:
        array = np.asarray(value.numpy())
        if not np.isfinite(array).all():
            raise RuntimeError("Nonfinite native state during lockstep")
        digest.update(array.tobytes())
    return {
        "simulation_s": simulation_s,
        "physics_steps": int(SimulationManager.get_num_physics_steps()),
        "fabric_s": fabric_s, "timeline_s": timeline_s,
        "native_state_sha256_audit_only": digest.hexdigest(),
    }


def pipeline_hint(*, tracker, odom, now_sim_s, now_wall_s):
    """Read source evidence without mutating the tracker or granting movement.

    The next unique normal tracker tick still checks pose/IMU/link/action/floor,
    direction and all original clocks. Do not call tracker.tick or its stateful
    perception gate here, as that would invent additional control ticks.
    """
    from ..depth_safety_fusion import FUSION_SOURCE, validate_fusion_contract
    from ..rtx_deskewed_perception import state_age_gate

    odom = odom or {}
    ok, reason, ages = state_age_gate(
        now_sim_s=now_sim_s, stamp_s=odom.get("time"),
        arrival_wall_s=odom.get("arrival_wall"), now_wall_s=now_wall_s,
        max_sim_age_s=tracker.config["max_sim_age_s"],
        max_wall_age_s=tracker.config["max_wall_age_s"],
    )
    evidence = {"odom_stamp_s": odom.get("time"), "odom_ages": ages,
                "motion_permission": False}
    if not ok:
        return False, dict(evidence, reason="odom:" + str(reason))
    try:
        path = tracker.perception_path
        if path.stat().st_size > 2 * 1024 * 1024:
            raise ValueError("perception_file_too_large")
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict) or payload.get("source") != FUSION_SOURCE:
            raise ValueError("joint_perception_unavailable")
        validate_fusion_contract(payload, now_sim_s=now_sim_s, now_wall_s=now_wall_s)
        if (payload.get("fail_closed") is not False
                or payload.get("observation_state") != "measured"
                or not tracker._floor_supported((payload.get("directions") or {}).get("front"), FUSION_SOURCE)):
            raise ValueError("perception_floor_or_clearance_unknown")
        clearance = float(payload.get("clearance_m"))
        if not math.isfinite(clearance) or clearance < tracker.config["min_clearance_m"]:
            raise ValueError("perception_obstacle_too_close")
        evidence.update(perception_stamp_s=payload["stamp_s"],
                        perception_arrival_wall_s=payload["arrival_wall_s"])
    except (OSError, ValueError, TypeError, KeyError) as error:
        return False, dict(evidence, reason=str(error))
    return True, dict(evidence, reason="fresh_joint_sources_available_not_motion_permission")


def original_wall_deadline(tracker):
    """Only shorten waiting at existing wall deadlines; never alter their state."""
    deadlines = []
    hold = getattr(tracker, "_hold_since_wall", None)
    if hold is not None:
        deadlines.append(hold + tracker.config["pose_loss_timeout_s"])
    heartbeat = getattr(tracker, "_last_heartbeat_wall", None)
    if heartbeat is not None:
        deadlines.append(heartbeat + tracker.config["link_timeout_s"])
    action = getattr(tracker, "_action", None)
    if action is not None:
        deadlines.append(action["expires_wall_s"])
    pending = getattr(tracker, "_pending", ())
    if pending:
        deadlines.append(pending[0]["expires_wall_s"])
    return min(deadlines) if deadlines else math.inf


def _assert_frozen(before, after, boundary):
    changes = {key: {"before": before.get(key), "after": after.get(key)}
               for key in sorted(set(before) | set(after))
               if before.get(key) != after.get(key)}
    if changes:
        # The existing capture failure receipt persists the exception. Include
        # exact changed fields, not just an opaque state-invariant label.
        raise RuntimeError("Lockstep invariant changed " + boundary + ": "
                           + json.dumps(changes, sort_keys=True, allow_nan=False))


def bounded_wait(*, max_wait_s, clock_published_wall_s, snapshot, pump,
                 ready, cancelled, stop_requested, wall_deadline=math.inf,
                 clock=time.perf_counter):
    """Wait with injected dependencies, CPU-testable without importing Isaac.

    SDK render may itself block beyond the requested duration. We never claim a
    hard preemptive timeout: after it returns, check invariants and expire the
    wait BEFORE accepting readiness. A host supervisor owns the hard wall cap.
    """
    if not math.isfinite(max_wait_s) or not 0 < max_wait_s <= MAX_WAIT_S:
        raise ValueError("Lockstep wait must be within (0, 0.20] wall seconds")
    if not math.isfinite(clock_published_wall_s):
        raise ValueError("Lockstep requires a real prior clock publication")
    begin = clock()
    if clock_published_wall_s > begin:
        raise ValueError("Clock publication wall time is in the future")
    deadline = min(begin + max_wait_s,
                   clock_published_wall_s + MAX_CLOCK_PUBLISH_AGE_S,
                   wall_deadline)
    initial = snapshot()
    rounds = 0
    last_hint = None
    status = "timeout"
    while True:
        cancelled()
        if stop_requested():
            status = "stop_or_terminal"
            break
        now = clock()
        if now >= deadline:
            break
        available, last_hint = ready(now)
        if available:
            # Read/validation work also consumes wall time. It cannot extend a
            # deadline or turn a late render completion into readiness.
            if clock() < deadline:
                status = "pipeline_hint_ready"
            break
        before = snapshot()
        _assert_frozen(initial, before, "before render")
        pump(max(0.0, deadline - now))
        rounds += 1
        after = snapshot()
        _assert_frozen(initial, after, "after render/consumer pump advanced state")
    final = snapshot()
    _assert_frozen(initial, final, "on exit")
    return {
        "mode": MODE, "status": status, "render_pump_rounds": rounds,
        "started_wall_s": begin, "finished_wall_s": clock(),
        "deadline_wall_s": deadline, "clock_published_wall_s": clock_published_wall_s,
        "clock_before": initial, "clock_after": final,
        "last_pipeline_hint": last_hint, "motion_permission": False,
        "policy_ticks_added": 0, "physics_steps_added": 0,
        "clock_messages_added": 0, "source_times_changed": False,
    }
