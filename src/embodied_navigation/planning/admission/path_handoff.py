"""Preview replacement and request overlap; never change control mathematics."""

import copy
import math
import time
import numpy as np
from embodied_navigation.planning.reference.global_route_reference import direction_check
from embodied_navigation.planning.admission.replan_ack import enabled as _early_request_ack_enabled

from embodied_navigation.planning.admission.exhausted_replacement import (
    enabled as _exhausted_drive_replacement_enabled,
)

EARLY_REQUEST_ACK_ENABLED = _early_request_ack_enabled()
EXHAUSTED_DRIVE_REPLACEMENT_ENABLED = _exhausted_drive_replacement_enabled()


def ordinary_coverage_advance(self, old, current):
    ledger = self.continuous_route
    if (
        ledger is None
        or type(old) is not int
        or type(current) is not int
        or not 0 <= old < current <= ledger.target_index
    ):
        return False
    # The existing ledger alone establishes passage. Never grant an index jump.
    for index in range(old, current):
        if index in ledger.hard:
            return False
        matches = [r for r in ledger.receipts if r.get("waypoint_index") == index]
        if len(matches) != 1:
            return False
        r = matches[0]
        if not (
            r.get("schema") == "original_region_ordered_pass/v1"
            and r.get("request_id") == self.request_id
            and r.get("goal_id") == ledger.goals[index].goal_id
            and r.get("goals_definition_sha256") == ledger.goal_hash
            and r.get("route_sha256") == ledger.route_sha
            and r.get("reason") == "passed"
            and r.get("control_region_inside") is True
            and r.get("protected") is False
            and r.get("navigation_ground_truth_used") is False
            and type(r.get("stamp_ns")) is int
            and r["stamp_ns"] <= self.pose_stamp
        ):
            return False
    return True


def drive_exhaustion_replacement_eligibility(self, execution):
    """Pure same-path exhaustion proof; grants only a replacement prerequisite.

    Reuse the caller's single live-source check. Compare already recorded scalar
    identities and sample digest; never rehash samples or refresh source times.
    Capture/parking recovery and old-path execution retain their own gates.
    """
    marker = getattr(self, "execution_path_exhaustion", None)
    receipt = self.path_receipt or {}
    cascade = self.cascade
    conditions = {k: v for k, v in execution["conditions"].items() if k != "samples_present"}
    conditions.update(
        samples_absent=self.samples is None,
        drive_phase=self.heading_gate.phase == "drive",
        no_capture=cascade is not None and cascade.capture_stamp is None,
        cascade_not_protected=cascade is not None and not cascade.protected,
        cascade_failure_not_latched=cascade is not None and cascade.failure_latched is None,
        same_cascade_goal=cascade is not None
        and self.cascade_key == (self.request_id, self.waypoint_index),
        same_accepted_path=cascade is not None and receipt.get("path_id") == cascade.path_id,
        exhaustion_receipt_present=isinstance(marker, dict),
    )
    if isinstance(marker, dict):
        digest = marker.get("previous_samples_sha256")
        reference = self.active_reference_stamp
        conditions.update(
            exhaustion_internal_schema=marker.get("schema") == "internal_checked_path_exhaustion/v1"
            and marker.get("source") == "actual_shared_control_exhausted_clear_branch"
            and marker.get("navigation_ground_truth_used") is False,
            exhaustion_allowed_reason=marker.get("reason")
            in ("steering_exhausted", "low_command_exhausted_path"),
            exhaustion_same_request_waypoint_goal=dict(
                request_id=marker.get("request_id"),
                waypoint_index=marker.get("waypoint_index"),
                goal_xyz=marker.get("goal_xyz"),
            )
            == self.admission_identity(),
            exhaustion_same_active_id=marker.get("active_trajectory_id")
            == self.active_trajectory_id,
            exhaustion_same_active_reference=marker.get("active_reference_stamp")
            == list(reference or ()),
            exhaustion_matches_accepted_samples=isinstance(digest, str)
            and len(digest) == 64
            and all(c in "0123456789abcdef" for c in digest)
            and digest == receipt.get("source_samples_float64_sha256")
            and marker.get("previous_samples_count") == receipt.get("source_samples_count")
            and type(marker.get("previous_samples_count")) is int
            and 2 <= marker["previous_samples_count"] <= 2000,
            exhaustion_matches_accepted_receipt=(
                receipt.get("request_id"),
                receipt.get("waypoint_index"),
                receipt.get("trajectory_id"),
            )
            == (self.request_id, self.waypoint_index, self.active_trajectory_id),
            exhaustion_causal_stamps=type(marker.get("source_pose_stamp_ns")) is int
            and type(marker.get("control_stamp_ns")) is int
            and type(receipt.get("stamp_ns")) is int
            and reference is not None
            and len(reference) == 2
            and receipt["stamp_ns"] == int(reference[0]) * 1_000_000_000 + int(reference[1])
            and 0 <= receipt["stamp_ns"] <= marker["control_stamp_ns"] <= execution["clock_ns"]
            and 0 <= marker["source_pose_stamp_ns"] <= marker["control_stamp_ns"]
            and marker["source_pose_stamp_ns"] <= self.pose_stamp,
        )
    else:
        conditions["exhaustion_internal_schema"] = False
    allowed = all(conditions.values())
    return dict(
        allowed=allowed,
        conditions=conditions,
        reason="recorded_exhausted_drive_can_replace"
        if allowed
        else "drive_exhaustion_replacement_not_eligible",
        authorizes_old_execution=False,
        actual_source_times_refreshed=False,
    )


def context(self):
    active = self.path_admission_active
    inputs = getattr(self, "known_scene_inputs", None)
    cascade = getattr(self, "cascade", None)
    receipt = getattr(self, "path_receipt", None)
    if not isinstance(inputs, dict) or cascade is None or not isinstance(receipt, dict):
        return dict(
            allowed=False, conditions=dict(context_present=False), generation=None, coverage=None
        )
    generation = inputs.get("generation")
    coverage = None if self.continuous_route is None else self.continuous_route.next_index
    conditions = dict(
        source_generation_ready=self._known_generation_ready(),
        same_correction_generation=active is not None
        and active.get("correction_generation") == generation,
        same_coverage=active is not None and active.get("ordered_coverage_index") == coverage,
        same_cascade_goal=self.cascade_key == (self.request_id, self.waypoint_index),
        receipt_present=self.path_receipt is not None,
        same_path=self.path_receipt is not None
        and self.cascade.path_id == self.path_receipt["path_id"],
        same_trajectory=self.path_receipt is not None
        and self.path_receipt["trajectory_id"] == self.active_trajectory_id,
    )
    execution = self.path_execution_eligibility()
    conditions.update(execution["conditions"])
    replacement_conditions = {k: v for k, v in conditions.items() if k != "same_coverage"}
    replacement_conditions["ordered_coverage_verified"] = conditions[
        "same_coverage"
    ] or ordinary_coverage_advance(
        self, None if active is None else active.get("ordered_coverage_index"), coverage
    )
    exhaustion = None
    if EXHAUSTED_DRIVE_REPLACEMENT_ENABLED and not conditions["samples_present"]:
        exhaustion = drive_exhaustion_replacement_eligibility(self, execution)
        replacement_conditions.pop("samples_present")
        replacement_conditions["samples_present_or_recorded_drive_exhaustion"] = exhaustion[
            "allowed"
        ]
    result = dict(
        allowed=all(conditions.values()),
        conditions=conditions,
        replacement_allowed=all(replacement_conditions.values()),
        replacement_conditions=replacement_conditions,
        generation=generation,
        coverage=coverage,
    )
    if exhaustion is not None:
        result["drive_exhaustion_replacement"] = exhaustion
    return result


def preview(self, samples, value, metadata):
    required = (
        self.heading_gate.phase == "drive"
        and self.cascade is not None
        and self.cascade.capture_stamp is None
        and self.path_admission_active is not None
    )
    if not required:
        return dict(required=False, allowed=True)
    eligibility = context(self)
    if not eligibility.get("replacement_allowed", False):
        return dict(
            required=True,
            allowed=False,
            reason="handoff_active_context_not_ready",
            eligibility=eligibility,
        )
    points = np.asarray(samples, dtype=float)
    indices = [0]
    for i in range(1, len(points)):
        if np.linalg.norm(points[i, :2] - points[indices[-1], :2]) >= 1e-5:
            indices.append(i)
    pose = self.pose.copy()
    yaw = math.atan2(self.rotation[1, 0], self.rotation[0, 0])
    old = self.cascade.preview_reference(pose)
    candidate = copy.deepcopy(self.cascade)
    stamp = metadata["reference_stamp"]
    candidate.set_path(
        points[indices].tolist(),
        "handoff_preview:" + str(value["traj_id"]),
        int(stamp[0]) * 1_000_000_000 + int(stamp[1]),
        time.monotonic_ns(),
        "camera_init",
        position=pose,
    )
    new = candidate.preview_reference(pose)
    wrap = lambda x: math.atan2(math.sin(x), math.cos(x))
    body_error = wrap(new["heading_rad"] - yaw)
    change = wrap(new["heading_rad"] - old["heading_rad"])
    if not all(map(math.isfinite, (yaw, body_error, change))):
        raise ValueError("Nonfinite handoff heading")
    limit = self.profile["teacher_transition"]["drive_heading_max_rad"]
    allowed = abs(body_error) <= limit and abs(change) <= limit
    return dict(
        required=True,
        allowed=allowed,
        heading_compatible=allowed,
        reason="handoff_ready" if allowed else "handoff_not_ready",
        body_heading_error_rad=body_error,
        reference_change_rad=change,
        maximum_error_rad=limit,
        old_reference=old,
        new_reference=new,
        eligibility=eligibility,
        source_pose_stamp_ns=int(self.pose_stamp),
        source_cloud_stamp_ns=int(self.cloud_stamp),
        preview_only=True,
    )


def stopped_rebind(self, handoff, proof):
    eligibility = context(self)
    wall = time.monotonic()
    clock = self.get_clock().now().nanoseconds
    command = list(self.command)
    conditions = dict(
        replacement_context=eligibility.get("replacement_allowed", False),
        original_geometry=proof.get("allowed") is True,
        original_ordered_leg=self.continuous_route is None
        or proof.get("ordered_current_leg", {}).get("allowed") is True,
        original_direction=not self.global_reference_settings["enabled"]
        or proof.get("direction_coherence", {}).get("allowed") is True,
        incompatible_heading=handoff.get("required") is True
        and handoff.get("heading_compatible") is False,
        admission_hold=self.path_admission_hold is True,
        actual_zero_command=len(command) == 3
        and all(math.isfinite(x) and x == 0.0 for x in command),
        no_capture=self.cascade is not None and self.cascade.capture_stamp is None,
        no_cascade_failure=self.cascade is not None and self.cascade.failure_latched is None,
        unchanged_source_pose=self.pose_stamp == handoff.get("source_pose_stamp_ns"),
        unchanged_source_cloud=self.cloud_stamp == handoff.get("source_cloud_stamp_ns"),
        original_measured_stop=self.heading_gate.stopped(clock / 1e9, wall),
    )
    return dict(
        schema="measured_stop_path_rebind/v1",
        allowed=all(conditions.values()),
        conditions=conditions,
        eligibility=eligibility,
        teacher=self.heading_gate.evidence(clock / 1e9, wall),
        actual_control_stamp_ns=clock,
        actual_wall=wall,
        source_pose_stamp_ns=int(self.pose_stamp),
        source_cloud_stamp_ns=int(self.cloud_stamp),
        old_trajectory_id=self.active_trajectory_id,
        old_path_id=None if self.path_receipt is None else self.path_receipt["path_id"],
        old_coverage=None
        if self.path_admission_active is None
        else self.path_admission_active.get("ordered_coverage_index"),
        new_coverage=eligibility.get("coverage"),
        navigation_ground_truth_used=False,
        authorizes_commit_only=True,
        Teacher_transition_bypassed=False,
    )


def early_request(self, reference):
    from embodied_navigation.planning.admission.pending_path_commit import waiting

    if waiting(self):
        return None
    if (
        self.heading_gate.phase != "drive"
        or self.cascade.capture_stamp is not None
        or not context(self)["allowed"]
    ):
        return None
    remaining = float(reference["remaining_m"])
    arc = float(self.cascade.spatial_config["arc_length_m"])
    if not 0 < remaining <= arc:
        return None
    # A curve ending at the immutable goal needs its original arrival/capture.
    if reference["endpoint_is_fixed_goal"]:
        return None
    key = (
        self.request_id,
        self.waypoint_index,
        self.known_scene_inputs["generation"],
        None if self.continuous_route is None else self.continuous_route.next_index,
        self.cascade.path_id,
    )
    if self.path_handoff_early_requested_for == key:
        return None
    association = self.trajectory_association
    if association.reference_stamp is not None and association.accepted is None:
        return None
    before = self.reference_requests
    result = self.request_plan()
    requested = self.reference_requests > before
    if EARLY_REQUEST_ACK_ENABLED:
        # Local request sequence advances before publish and is not an ACK.
        # Automatic mode returns True only after all original completion work.
        # Legacy nonautomatic publishers still acknowledge via their counter.
        requested = result is True or (
            getattr(self, "automatic_reference", None) is None and requested
        )
    if requested:
        self.path_handoff_early_requested_for = key
    receipt = dict(
        schema="causal_path_handoff_early_request/v1",
        path_id=self.cascade.path_id,
        requested=requested,
        remaining_m=remaining,
        original_preview_arc_m=arc,
        source_pose_stamp_ns=int(self.pose_stamp),
        actual_source_times_refreshed=False,
        navigation_ground_truth_used=False,
    )
    self.path_handoff_last_check = receipt
    self.evidence.append(self.run / "path_handoff_early_requests.jsonl", receipt)
    return receipt


def retention_reference_eligibility(self):
    """Retain only an old reference permitted by original drive execution gates.
    Preview clones the original controller: no progress, integral or command
    mutation. It grants no replacement, heading or stopping authority.
    """
    drive = (
        self.heading_gate.phase == "drive"
        and self.cascade is not None
        and self.cascade.capture_stamp is None
    )
    settings = getattr(self, "global_reference_settings", {"enabled": False})
    curvature_required = drive and self.cascade.spatial_config["curvature_speed_limit_enabled"]
    direction_required = drive and settings["enabled"]
    if not (curvature_required or direction_required):
        return dict(
            allowed=True, required=False, reason="original_drive_curvature_gate_not_required"
        )
    try:
        reference = self.cascade.preview_reference(self.pose)
        curvature_allowed = (
            not curvature_required
            or reference["curvature_valid"] is True
            or reference["heading_source"] == "fixed_goal_tail"
        )
        direction = (
            direction_check(
                self.cascade,
                reference,
                self.profile["teacher_transition"]["drive_heading_max_rad"],
                settings,
            )
            if direction_required
            else None
        )
        direction_allowed = direction is None or direction["allowed"] is True
    except (ValueError, TypeError, KeyError, IndexError, FloatingPointError) as error:
        return dict(
            allowed=False,
            required=True,
            reason="old_reference_preview_failed",
            error=type(error).__name__ + ": " + str(error),
            preview_only=True,
        )
    allowed = curvature_allowed and direction_allowed
    return dict(
        allowed=allowed,
        required=True,
        reason=(
            "old_reference_curvature_unavailable_requires_stop_or_new_path"
            if not curvature_allowed
            else "old_reference_direction_incoherent_requires_stop_or_new_path"
            if not direction_allowed
            else "original_current_reference_curvature_available"
        ),
        reference=reference,
        direction_coherence=direction,
        original_curvature_required=bool(curvature_required),
        original_direction_required=bool(direction_required),
        source_pose_stamp_ns=int(self.pose_stamp),
        old_path_id=self.cascade.path_id,
        preview_only=True,
        original_curvature_or_heading_limits_changed=False,
    )
