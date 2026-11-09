"""Admission decision application for ``PIDNavigation``.

Contains the single large ``_admit_matched_spline`` body extracted verbatim from
the former monolith. It still calls the admission-retention helper through
``AdmissionMixin`` (same function object, no behavior change).
"""

from .pid_support import (
    admission_direction_check,
    staged_cascade_binding,
    np,
    time,
    trajectory_has_progress,
    payload,
    tail_stop_request,
    preview_path_handoff,
    stopped_path_rebind,
    observe_business_gate,
    reset_transition,
    BSpline,
    handoff_context,
    _r87_diag,
    admission_callback_rejected,
    defer_future_path,
    _callback_cost_observe,
    copy,
    certify_curve,
    candidate,
    hashlib,
)

from .pid_admission import AdmissionMixin


class AdmissionApplyMixin:
    """The admission decision body that matches an incoming SCAN spline against the frozen route."""

    @_callback_cost_observe("admission")
    @_r87_diag.observe("scan.admission")
    def _admit_matched_spline(
        self, msg, metadata, received_wall_ns=None, *, admission_started=None, physical_result=None
    ):
        # Admission is a transaction BEFORE base acceptance, publication, PI
        # path replacement and the accepted trajectory archive. Association's
        # payload high-water is deliberately not rolled back on a rejection.
        started = time.monotonic() if admission_started is None else admission_started
        asynchronous = getattr(self, "_r88_admission", None)
        if asynchronous is not None and physical_result is None:
            result = asynchronous.start(
                msg, metadata, received_wall_ns=received_wall_ns, admission_started=started
            )
            prepared_checkpoint = time.monotonic()
            item = asynchronous.current
            asynchronous.record(
                "prepare_callback_completed",
                job_token=None if item is None else item["token"],
                admission_started_monotonic=started,
                parent_callback_checkpoint_monotonic=prepared_checkpoint,
                parent_callback_elapsed_wall_ms=(prepared_checkpoint - started) * 1000,
                callback_budget_scope="single_synchronous_parent_stage",
            )
            if (
                admission_callback_rejected(
                    self.run,
                    self.profile,
                    prepared_checkpoint - started,
                    "admission_callback_budget_exhausted_during_async_prepare",
                )
                and item is not None
                and item["started"] == started
            ):
                # The submitted result cannot revive this rejected callback.
                # Keep queued FIFO events; its later result is an orphan.
                asynchronous.current = None
                asynchronous.counts["invalidated"] += 1
                asynchronous._transport_mark(12, job_token=item["token"])
                asynchronous.record("prepare_callback_rejected", job_token=item["token"])
                return self._admit_matched_spline(
                    msg,
                    metadata,
                    received_wall_ns=item["received_wall_ns"],
                    admission_started=started,
                    physical_result={
                        "error": "admission_callback_budget_exhausted_during_async_prepare"
                    },
                )
            return result
        value = None
        binding = None
        points = None
        config = None
        route_point_budget = 256
        try:
            if len(msg.pos_pts) > 512 or len(msg.knots) > 516:
                raise ValueError("candidate_input_count_budget")
            if self.automatic_reference is not None:
                from embodied_navigation.planning.reference_protocol import payload as mode3_payload

                with _r87_diag.span("scan.payload_decode", source_ns=self.pose_stamp):
                    value = mode3_payload(msg)
                if (
                    metadata.get("schema") != 3
                    or metadata.get("trajectory_id") != value["traj_id"]
                    or metadata.get("trajectory_start_time") != value["start_time"]
                    or (metadata.get("route_id"), metadata.get("route_version"))
                    != self.trajectory_association.route_key
                    or self.trajectory_association.accepted != value
                    or metadata.get("reference_stamp")
                    != self.trajectory_association.reference_stamp
                ):
                    raise ValueError("r85_candidate_not_bound_by_schema3_association")
            else:
                with _r87_diag.span("scan.payload_decode", source_ns=self.pose_stamp):
                    value = payload(msg)
                    value = candidate(
                        msg,
                        metadata,
                        self.trajectory_association.reference_stamp,
                        self.waypoints[self.waypoint_index]
                        if self.continuous_route is None
                        else self.trajectory_association.goal,
                    )
            if (
                self.continuous_route is not None
                and self.scan_reference_coverage_index != self.continuous_route.next_index
            ):
                raise ValueError("candidate_ordered_coverage_epoch_mismatch")
            if self.continuous_route is not None and not np.array_equal(
                self.trajectory_association.goal, self.waypoints[self.scan_reference_coverage_index]
            ):
                raise ValueError("candidate_ordered_reference_goal_mismatch")
            with _r87_diag.span("scan.fence_load", source_ns=self.pose_stamp):
                points, config, binding = self._route_fence_load()
            if self.automatic_reference is not None:
                # Only the current successfully loaded/frozen R85 source may
                # use its explicit reference capacity. Legacy fences keep 256.
                from embodied_navigation.planning.reference.automatic_reference_runtime import (
                    MAX_REFERENCE_POINTS,
                )

                if (
                    binding.get("schema") != "r85_automatic_route_fence_binding/v1"
                    or self.automatic_reference.config.get("max_reference_points")
                    != MAX_REFERENCE_POINTS
                ):
                    raise ValueError("r85_fence_route_capacity_binding_mismatch")
                route_point_budget = MAX_REFERENCE_POINTS
            with _r87_diag.span("scan.curve_certificate", source_ns=self.pose_stamp):
                proof = certify_curve(value, points, config, max_route_points=route_point_budget)
            proof = observe_business_gate(
                self,
                "scan_route_fence",
                proof,
                {
                    "outside_fence",
                    "depth_budget_exhausted",
                    "patch_budget_exhausted",
                    "wall_budget_exhausted",
                },
            )
            if proof["allowed"] and self.continuous_route is not None:
                index = self.scan_reference_coverage_index
                prefix = self.continuous_route.points[index : index + 2]
                with _r87_diag.span("scan.ordered_curve_certificate", source_ns=self.pose_stamp):
                    ordered_proof = certify_curve(value, prefix, config)
                proof = dict(
                    proof,
                    ordered_current_leg=ordered_proof,
                    ordered_coverage_index=index,
                    local_body_goal=list(self.trajectory_association.goal),
                )
                if not ordered_proof["allowed"]:
                    proof = dict(proof, allowed=False, reason="outside_ordered_current_leg")
                    proof = observe_business_gate(
                        self, "scan_ordered_current_leg", proof, {"outside_ordered_current_leg"}
                    )
            if proof["allowed"]:
                begin, end = value["knots"][3], value["knots"][-4]
                with _r87_diag.span("scan.execution_samples", source_ns=self.pose_stamp):
                    check = BSpline(value["knots"], value["pos_pts"], 3)(
                        np.linspace(begin, end, min(2000, max(2, int((end - begin) / 0.08) + 1)))
                    )
                if not np.isfinite(check).all():
                    proof = dict(allowed=False, reason="nonfinite_execution_samples")
                elif self.pose is not None and np.linalg.norm(check[0] - self.pose) > 1.2:
                    proof = dict(allowed=False, reason="execution_start_far_from_actual_pose")
                    proof = observe_business_gate(
                        self,
                        "scan_execution_start_distance",
                        proof,
                        {"execution_start_far_from_actual_pose"},
                    )
                elif not trajectory_has_progress(
                    check, self.pose, self.waypoints[self.waypoint_index], self.arrival_radius
                ) or not np.any(np.linalg.norm(check[1:, :2] - check[0, :2], axis=1) >= 1e-5):
                    proof = dict(allowed=False, reason="execution_path_degenerate")
            if proof["allowed"] and self.global_reference_settings["enabled"]:
                if self.segment_start is None:
                    raise ValueError("direction_check_requires_original_segment_activation")
                delta = self.waypoints[self.waypoint_index][:2] - self.segment_start[:2]
                with _r87_diag.span("scan.direction_check", source_ns=self.pose_stamp):
                    check_direction = admission_direction_check(
                        check,
                        self.pose,
                        self.waypoints[self.waypoint_index],
                        self.fixed_terminal_heading(self.waypoint_index, self.segment_start),
                        self.profile["cascade"],
                        self.profile["teacher_transition"]["drive_heading_max_rad"],
                        self.global_reference_settings,
                    )
                proof = dict(proof, direction_coherence=check_direction)
                if not check_direction["allowed"]:
                    proof = dict(proof, allowed=False, reason="execution_direction_incoherent")
                    proof = observe_business_gate(
                        self, "scan_execution_direction", proof, {"execution_direction_incoherent"}
                    )
            if proof["allowed"] and self.r85_physical_gate is not None:
                with _r87_diag.span("scan.physical_certificate", source_ns=self.pose_stamp):
                    if physical_result is None:
                        physical = self.r85_physical_gate.certify(self, value, metadata, binding)
                    elif physical_result.get("error"):
                        physical = dict(
                            schema="r85_actual_scan_physical_execution_gate/v1",
                            allowed=False,
                            reason="r88_physical_worker_rejected",
                            error=physical_result["error"],
                        )
                    else:
                        physical = self.r85_physical_gate.complete(
                            self,
                            value,
                            metadata,
                            binding,
                            physical_result["prepared"],
                            physical_result["proof"],
                        )
                certificate = physical.get("physical_certificate", {})
                if (
                    physical.get("reason") == "actual_curve_physical_space_not_certified"
                    and certificate.get("status") == "UNKNOWN_REJECTED"
                    and certificate.get("reason")
                    in ("continuous_outer_bound_not_certified", "patch_or_wall_budget_exhausted")
                ):
                    physical = observe_business_gate(
                        self,
                        "r85_curve_physical_prediction",
                        physical,
                        {"actual_curve_physical_space_not_certified"},
                    )
                proof = dict(proof, r85_physical=physical)
                if not physical["allowed"]:
                    proof = dict(proof, allowed=False, reason=physical["reason"])
            if admission_callback_rejected(
                self.run,
                self.profile,
                time.monotonic() - started,
                "admission_callback_budget_exhausted",
            ):
                proof = dict(allowed=False, reason="admission_callback_budget_exhausted")
        except (
            OSError,
            KeyError,
            ValueError,
            TypeError,
            OverflowError,
            IndexError,
            FloatingPointError,
        ) as error:
            proof = dict(
                allowed=False,
                reason="admission_input_or_anchor:" + type(error).__name__ + ":" + str(error),
            )
        handoff = None
        if proof["allowed"]:
            try:
                with _r87_diag.span("scan.handoff_preview", source_ns=self.pose_stamp):
                    handoff = preview_path_handoff(self, check, value, metadata)
                if not handoff["allowed"] and handoff.get("reason") == "handoff_not_ready":
                    observed = observe_business_gate(
                        self, "scan_path_handoff_heading", handoff, {"handoff_not_ready"}
                    )
                    if observed.get("diagnostic_observe_only"):
                        # Accept the legal curve, then use the unchanged stop/turn
                        # state machine for its heading instead of driving across it.
                        handoff = dict(observed, diagnostic_heading_rebind=True)
                    if not handoff.get("diagnostic_heading_rebind"):
                        rebind = stopped_path_rebind(self, handoff, proof)
                        handoff = dict(
                            handoff,
                            stopped_rebind=rebind,
                            allowed=rebind["allowed"],
                            reason="handoff_measured_stop_rebind"
                            if rebind["allowed"]
                            else "handoff_not_ready",
                        )
                if not handoff["allowed"]:
                    proof = dict(
                        proof,
                        original_geometry_allowed=True,
                        original_geometry_reason=proof.get("reason"),
                        allowed=False,
                        reason=handoff["reason"],
                    )
            except (ValueError, TypeError, KeyError, AttributeError, OverflowError) as error:
                proof = dict(
                    proof,
                    allowed=False,
                    reason="handoff_preview_invalid:" + type(error).__name__ + ":" + str(error),
                )
        capture = (
            self.cascade is not None
            and self.cascade_key == (self.request_id, self.waypoint_index)
            and self.cascade.capture_stamp is not None
        )
        if self.cascade is not None and self.cascade.capture_stamp is not None and not capture:
            proof = dict(allowed=False, reason="capture_goal_identity_mismatch")
        retain = None
        recovery = None
        if capture and proof["allowed"]:
            with _r87_diag.span("scan.retain_path", source_ns=self.pose_stamp):
                retain = self.retain_admitted_path(
                    points, config, max_route_points=route_point_budget, binding=binding
                )
            if not retain["allowed"]:
                recovery = self.capture_replacement_eligibility(retain)
        if (
            proof["allowed"]
            and handoff is not None
            and handoff.get("stopped_rebind", {}).get("allowed")
            and not handoff.get("diagnostic_heading_rebind")
        ):
            commit_recheck = stopped_path_rebind(self, handoff, proof)
            handoff = dict(handoff, commit_stop_recheck=commit_recheck)
            if not commit_recheck["allowed"]:
                proof = dict(
                    proof, allowed=False, reason="handoff_stop_or_source_changed_before_commit"
                )
        if admission_callback_rejected(
            self.run,
            self.profile,
            time.monotonic() - started,
            "admission_callback_budget_exhausted",
        ):
            proof = dict(allowed=False, reason="admission_callback_budget_exhausted")
            recovery = None
        if not proof["allowed"] or capture and (recovery is None or not recovery["allowed"]):
            # Only a geometrically outside candidate (or an existing terminal
            # capture) may keep an independently certified old same-goal path.
            # Invalid anchor/numbers and exhausted proofs always stop/replan.
            retain = (
                retain
                if retain is not None
                else self.retain_admitted_path(
                    points, config, max_route_points=route_point_budget, binding=binding
                )
                if points is not None
                and (
                    proof.get("reason") == "outside_fence"
                    or proof["allowed"]
                    and capture
                    or handoff is not None
                    and not handoff["allowed"]
                    and handoff_context(self)["allowed"]
                )
                else dict(
                    allowed=False,
                    reason="fail_closed_rejection_requires_stop",
                    samples_present=self.samples is not None,
                    conditions=self.path_execution_eligibility()["conditions"],
                )
            )
            if admission_callback_rejected(
                self.run,
                self.profile,
                time.monotonic() - started,
                "retention_callback_budget_exhausted",
            ):
                retain = dict(
                    retain,
                    allowed=False,
                    reason="retention_callback_budget_exhausted",
                    samples_present=self.samples is not None,
                )
            stop_request = tail_stop_request(
                proof,
                handoff,
                getattr(self.cascade, "spatial_config", {}),
                self.pose_stamp,
                self.cloud_stamp,
                capture_active=getattr(self.cascade, "capture_stamp", None) is not None,
                failure_active=getattr(self.cascade, "failure_latched", None) is not None,
            )
            if retain.get("allowed") is True and stop_request["required"]:
                retain = dict(
                    allowed=False,
                    reason="nonfixed_tail_requires_original_measured_stop",
                    original_eligible_retention=retain,
                    handoff_stop_request=stop_request,
                )
            self.path_admission_hold = not retain["allowed"]
            self.last_spline_rejected = "V28 SCAN整条路径接纳拒绝：" + str(proof.get("reason"))
            self.path_admission_replan_pending = True
            if retain["allowed"]:
                self.pid_guard_needs_evaluation = True
            else:
                self.message = self.last_spline_rejected + "；停车等待重规划"
                self.publish_command()
            self.archive_admission(
                value,
                metadata,
                dict(
                    admitted=False,
                    reason=(
                        "terminal_capture_kept_checked_path"
                        if retain["allowed"]
                        else "terminal_capture_stopped_no_eligible_path"
                    )
                    if capture and proof["allowed"]
                    else proof.get("reason"),
                    proof=proof,
                    handoff=handoff,
                    anchor_binding=binding,
                    old_path_retention=retain,
                    capture_replacement=recovery,
                    exact_zero_requested=not retain["allowed"],
                ),
                started,
            )
            return
        if defer_future_path(
            self, msg, metadata, started=started, received_wall_ns=received_wall_ns
        ):
            return
        if self.r85_physical_gate is not None:
            with _r87_diag.span("scan.before_lower_freshness", source_ns=self.pose_stamp):
                before_lower = self.r85_physical_gate.fresh(
                    self, binding, self.known_scene_inputs["generation"]
                )
            proof = dict(proof, r85_before_lower_source_gate=before_lower)
            if not before_lower["allowed"] or admission_callback_rejected(
                self.run,
                self.profile,
                time.monotonic() - started,
                "admission_callback_budget_exhausted_before_lower_execution",
            ):
                self.path_admission_hold = True
                self.path_admission_replan_pending = True
                self.publish_command()
                self.last_spline_rejected = "R85 physical path post-compute source/budget rejected"
                self.message = self.last_spline_rejected
                self.archive_admission(
                    value,
                    metadata,
                    dict(
                        admitted=False,
                        reason="r85_pre_execution_source_or_budget_rejected",
                        proof=proof,
                        handoff=handoff,
                        anchor_binding=binding,
                        exact_zero_requested=True,
                    ),
                    started,
                )
                return
        if self.r85_physical_gate is None:
            previous_execution = (
                dict(
                    samples=self.samples,
                    active_trajectory_id=self.active_trajectory_id,
                    active_reference_stamp=self.active_reference_stamp,
                    planning_start_state=copy.deepcopy(self.planning_start_state),
                    execution_path_exhaustion=copy.deepcopy(
                        getattr(self, "execution_path_exhaustion", None)
                    ),
                )
                if recovery is not None and recovery["allowed"]
                else None
            )
            # A lower accepted archive is not yet the execution receipt commit.
            # Keep its unique replan/archive counter on a rejected final binding.
            binding_execution = dict(
                samples=self.samples,
                active_trajectory_id=self.active_trajectory_id,
                active_reference_stamp=self.active_reference_stamp,
                planning_start_state=copy.deepcopy(self.planning_start_state),
                execution_path_exhaustion=copy.deepcopy(
                    getattr(self, "execution_path_exhaustion", None)
                ),
                trajectory_archive_reference=getattr(self, "trajectory_archive_reference", None),
            )
            binding_generation = self.known_scene_inputs["generation"]
            before = self.replans
            wall = time.monotonic_ns() if received_wall_ns is None else received_wall_ns
            with _r87_diag.span("scan.lower_acceptance", source_ns=self.pose_stamp):
                super().accept_spline(msg, metadata)
            if self.replans > before and self.samples is not None:
                indices = [0]
                for i in range(1, len(self.samples)):
                    if np.linalg.norm(self.samples[i, :2] - self.samples[indices[-1], :2]) >= 1e-5:
                        indices.append(i)
                points = self.samples[indices]
                receipt = dict(
                    points_xyz=points.tolist(),
                    source_sample_indices=indices,
                    source_samples_count=len(self.samples),
                    source_samples_float64_sha256=hashlib.sha256(
                        np.asarray(self.samples, dtype="<f8").tobytes()
                    ).hexdigest(),
                    path_id=str(self.request_id)
                    + ":"
                    + str(self.waypoint_index)
                    + ":"
                    + str(self.replans)
                    + ":"
                    + str(self.active_trajectory_id),
                    stamp_ns=int(self.active_reference_stamp[0]) * 1_000_000_000
                    + int(self.active_reference_stamp[1]),
                    received_wall_ns=wall,
                    frame_id="camera_init",
                    array_file=self.trajectory_archive_reference,
                    request_id=self.request_id,
                    waypoint_index=self.waypoint_index,
                    trajectory_id=self.active_trajectory_id,
                    metadata=copy.deepcopy(metadata),
                )
                receipt["points_float64_sha256"] = hashlib.sha256(
                    np.asarray(points, dtype="<f8").tobytes()
                ).hexdigest()
                try:
                    with _r87_diag.span("scan.cascade_binding", source_ns=self.pose_stamp):
                        staged = staged_cascade_binding(self, receipt)
                    if staged is not None:
                        if admission_callback_rejected(
                            self.run,
                            self.profile,
                            time.monotonic() - started,
                            "admission_callback_budget_exhausted",
                        ):
                            raise ValueError("admission_callback_budget_exhausted")
                        if (
                            not self._known_generation_ready()
                            or self.known_scene_inputs["generation"] != binding_generation
                        ):
                            raise ValueError("admitted_path_source_generation_changed")
                    r85_active = None
                    if self.r85_physical_gate is not None:
                        # Prepare all potentially large immutable copies before the
                        # last freshness/100ms check, including lazy/capture paths.
                        r85_active = dict(
                            payload=copy.deepcopy(value),
                            metadata=copy.deepcopy(metadata),
                            identity=self.admission_identity(),
                            trajectory_id=self.active_trajectory_id,
                            active_reference_stamp=tuple(self.active_reference_stamp),
                            proof=copy.deepcopy(proof),
                            anchor_binding=binding,
                            correction_generation=self.known_scene_inputs["generation"],
                            ordered_coverage_index=None
                            if self.continuous_route is None
                            else self.continuous_route.next_index,
                        )
                        with _r87_diag.span("scan.final_source", source_ns=self.pose_stamp):
                            final_source = self.r85_physical_gate.fresh(
                                self, binding, binding_generation
                            )
                        proof = dict(proof, r85_final_execution_source_gate=final_source)
                        r85_active["proof"]["r85_final_execution_source_gate"] = final_source
                        final_elapsed = time.monotonic() - started
                        final_source.update(
                            admission_started_monotonic_wall=started,
                            final_checked_callback_elapsed_wall_ms=final_elapsed * 1000,
                            callback_budget_s=0.100,
                        )
                        if not final_source["allowed"]:
                            raise ValueError("r85_admitted_path_final_source_not_fresh_or_bound")
                        if admission_callback_rejected(
                            self.run,
                            self.profile,
                            final_elapsed,
                            "admission_callback_budget_exhausted",
                        ):
                            raise ValueError("admission_callback_budget_exhausted")
                except (
                    ValueError,
                    TypeError,
                    KeyError,
                    AttributeError,
                    OverflowError,
                    FloatingPointError,
                ) as error:
                    for key, old in binding_execution.items():
                        setattr(self, key, old)
                    self.path_admission_hold = True
                    self.path_admission_replan_pending = True
                    self.publish_command()
                    self.last_spline_rejected = "V33 admitted path binding rejected: " + str(error)
                    self.message = self.last_spline_rejected + "; wait for a fully bound path"
                    self.archive_admission(
                        value,
                        metadata,
                        dict(
                            admitted=False,
                            reason="admitted_path_binding_rejected",
                            proof=proof,
                            handoff=handoff,
                            anchor_binding=binding,
                            binding_error=str(error),
                            lower_gate_samples_accepted=True,
                            lower_gate_execution_identity_restored=True,
                            unique_lower_archive_counter_retained=True,
                            exact_zero_requested=True,
                        ),
                        started,
                    )
                    return
                capture_clear = (
                    self.clear_committed_exhausted_capture()
                    if recovery is not None and recovery["allowed"]
                    else None
                )
                active = (
                    r85_active
                    if r85_active is not None
                    else dict(
                        payload=copy.deepcopy(value),
                        metadata=copy.deepcopy(metadata),
                        identity=self.admission_identity(),
                        trajectory_id=self.active_trajectory_id,
                        active_reference_stamp=tuple(self.active_reference_stamp),
                        proof=copy.deepcopy(proof),
                        anchor_binding=binding,
                        correction_generation=self.known_scene_inputs["generation"],
                        ordered_coverage_index=None
                        if self.continuous_route is None
                        else self.continuous_route.next_index,
                    )
                )
                # All preparation succeeded; no await or callback boundary occurs
                # between the original cascade binding and its matching receipt.
                if staged is not None:
                    self.cascade = staged[0]
                    self.cascade_path_id = receipt["path_id"]
                self.path_receipt = receipt
                self.path_admission_active = active
                self.path_admission_hold = False
                self.path_admission_replan_pending = False
                self.reference_replan_pending = False
                self.reference_consistency_hold = False
                self.reference_consistency_check = None
                if handoff is not None and (
                    handoff.get("stopped_rebind", {}).get("allowed")
                    or handoff.get("diagnostic_heading_rebind")
                ):
                    reset_transition(self, "path_rebind")
                if staged is not None:
                    self.evidence.append(self.run / "navigation_cascade_paths.jsonl", staged[1])
                self.evidence.append(
                    self.run / "cascade_path_history.jsonl", copy.deepcopy(self.path_receipt)
                )
                self.archive_admission(
                    value,
                    metadata,
                    dict(
                        admitted=True,
                        reason="exhausted_capture_rebound_after_all_execution_gates"
                        if capture_clear is not None
                        else "continuous_fence_and_existing_execution_gates_passed",
                        proof=proof,
                        handoff=handoff,
                        anchor_binding=binding,
                        capture_replacement=recovery,
                        capture_clear=capture_clear,
                        synchronous_original_path_binding=staged is not None,
                    ),
                    started,
                )
            else:
                # Existing execution guards still own final admission. Their
                # refusal never becomes an admitted path in this separate ledger.
                if previous_execution is not None:
                    for key, old in previous_execution.items():
                        setattr(self, key, old)
                self.path_admission_hold = True
                self.path_admission_replan_pending = True
                self.publish_command()
                self.archive_admission(
                    value,
                    metadata,
                    dict(
                        admitted=False,
                        reason="existing_execution_gate_rejected",
                        proof=proof,
                        anchor_binding=binding,
                        capture_replacement=recovery,
                        lower_gate_execution_identity_restored=previous_execution is not None,
                        exact_zero_requested=True,
                    ),
                    started,
                )
            return
        # Lower sampling/archive identity remains distinct from executable release.
        # Every large copy/JSON template and the existing-writer enqueue is part
        # of the same original 100ms transaction. No pass record is durable until
        # the callback settles its single bounded batch outcome.
        binding_execution = dict(
            samples=self.samples,
            active_trajectory_id=self.active_trajectory_id,
            active_reference_stamp=self.active_reference_stamp,
            planning_start_state=copy.deepcopy(self.planning_start_state),
            execution_path_exhaustion=copy.deepcopy(
                getattr(self, "execution_path_exhaustion", None)
            ),
            trajectory_archive_reference=getattr(self, "trajectory_archive_reference", None),
        )
        promotion_snapshot = {
            key: getattr(self, key)
            for key in (
                "cascade",
                "cascade_path_id",
                "path_receipt",
                "path_admission_active",
                "path_admission_hold",
                "path_admission_replan_pending",
                "reference_replan_pending",
                "reference_consistency_hold",
                "reference_consistency_check",
                "heading_gate",
                "pid_guard_needs_evaluation",
            )
        }
        binding_generation = self.known_scene_inputs["generation"]
        before = self.replans
        wall = time.monotonic_ns() if received_wall_ns is None else received_wall_ns
        batch = None
        final_source = None
        source_small_raw = "null"
        completion = {}
        try:
            self.path_admission_hold = True
            with _r87_diag.span("scan.lower_acceptance", source_ns=self.pose_stamp):
                super().accept_spline(msg, metadata)
            if self.replans <= before or self.samples is None:
                raise ValueError("existing_execution_gate_rejected")
            indices = [0]
            for i in range(1, len(self.samples)):
                if np.linalg.norm(self.samples[i, :2] - self.samples[indices[-1], :2]) >= 1e-5:
                    indices.append(i)
            points = self.samples[indices]
            receipt = dict(
                points_xyz=points.tolist(),
                source_sample_indices=indices,
                source_samples_count=len(self.samples),
                source_samples_float64_sha256=hashlib.sha256(
                    np.asarray(self.samples, dtype="<f8").tobytes()
                ).hexdigest(),
                path_id=str(self.request_id)
                + ":"
                + str(self.waypoint_index)
                + ":"
                + str(self.replans)
                + ":"
                + str(self.active_trajectory_id),
                stamp_ns=int(self.active_reference_stamp[0]) * 1_000_000_000
                + int(self.active_reference_stamp[1]),
                received_wall_ns=wall,
                frame_id="camera_init",
                array_file=self.trajectory_archive_reference,
                request_id=self.request_id,
                waypoint_index=self.waypoint_index,
                trajectory_id=self.active_trajectory_id,
                metadata=copy.deepcopy(metadata),
            )
            receipt["points_float64_sha256"] = hashlib.sha256(
                np.asarray(points, dtype="<f8").tobytes()
            ).hexdigest()
            with _r87_diag.span("scan.cascade_binding", source_ns=self.pose_stamp):
                staged = staged_cascade_binding(self, receipt)
            if (
                not self._known_generation_ready()
                or self.known_scene_inputs["generation"] != binding_generation
            ):
                raise ValueError("admitted_path_source_generation_changed")
            active = dict(
                payload=copy.deepcopy(value),
                metadata=copy.deepcopy(metadata),
                identity=self.admission_identity(),
                trajectory_id=self.active_trajectory_id,
                active_reference_stamp=tuple(self.active_reference_stamp),
                proof=copy.deepcopy(proof),
                anchor_binding=binding,
                correction_generation=self.known_scene_inputs["generation"],
                ordered_coverage_index=None
                if self.continuous_route is None
                else self.continuous_route.next_index,
            )
            # Prepare the two mutating legacy post-commit actions privately.
            # Only references/scalars are promoted after the final source gate.
            from types import SimpleNamespace

            capture_clear = None
            prepared_cascade = staged[0] if staged is not None else self.cascade
            prepared_exhaustion = self.execution_path_exhaustion
            prepared_pid_guard = self.pid_guard_needs_evaluation
            if recovery is not None and recovery["allowed"]:
                capture_proxy = SimpleNamespace(
                    cascade=copy.deepcopy(self.cascade),
                    execution_path_exhaustion=copy.deepcopy(self.execution_path_exhaustion),
                    pid_guard_needs_evaluation=self.pid_guard_needs_evaluation,
                )
                capture_clear = AdmissionMixin.clear_committed_exhausted_capture(capture_proxy)
                prepared_cascade = staged[0] if staged is not None else capture_proxy.cascade
                prepared_exhaustion = capture_proxy.execution_path_exhaustion
                prepared_pid_guard = capture_proxy.pid_guard_needs_evaluation
            prepared_heading = self.heading_gate
            if handoff is not None and (
                handoff.get("stopped_rebind", {}).get("allowed")
                or handoff.get("diagnostic_heading_rebind")
            ):
                heading_proxy = SimpleNamespace(
                    heading_gate=copy.deepcopy(self.heading_gate),
                    request_id=self.request_id,
                    waypoint_index=self.waypoint_index,
                    obstacle_resume_pending=getattr(self, "obstacle_resume_pending", False),
                    _teacher_recovery_token=copy.deepcopy(
                        getattr(self, "_teacher_recovery_token", None)
                    ),
                    tilt_hold=getattr(self, "tilt_hold", False),
                )
                reset_transition(heading_proxy, "path_rebind")
                prepared_heading = heading_proxy.heading_gate
            decision = dict(
                admitted=True,
                reason="exhausted_capture_rebound_after_all_execution_gates"
                if capture_clear is not None
                else "continuous_fence_and_existing_execution_gates_passed",
                proof=proof,
                handoff=handoff,
                anchor_binding=binding,
                capture_replacement=recovery,
                capture_clear=capture_clear,
                synchronous_original_path_binding=staged is not None,
            )
            if self.r85_physical_gate is None:
                raise ValueError("prepared_transaction_requires_R85_physical_gate")
            # Commit current in-memory control state after a fresh sensor check.
            # Optional recording and audit serialization never authorize motion.
            final_source = self.r85_physical_gate.fresh(self, binding, binding_generation)
            if not final_source["allowed"]:
                raise ValueError("Current pose/cloud/planner generation is not fresh")
            self.cascade = prepared_cascade
            if staged is not None:
                self.cascade_path_id = receipt["path_id"]
            self.path_receipt = receipt
            self.path_admission_active = active
            self.execution_path_exhaustion = prepared_exhaustion
            self.pid_guard_needs_evaluation = prepared_pid_guard
            self.heading_gate = prepared_heading
            self.path_admission_replan_pending = False
            self.reference_replan_pending = False
            self.reference_consistency_hold = False
            self.reference_consistency_check = None
            self.path_admission_hold = False
            self.path_admission_sequence += 1
            self.path_admission_last_decision = dict(
                decision,
                sequence=self.path_admission_sequence,
                source_fresh=final_source["allowed"],
                callback_elapsed_wall_ms=(time.monotonic() - started) * 1000,
            )
            return
        except (
            ValueError,
            TypeError,
            KeyError,
            AttributeError,
            OverflowError,
            FloatingPointError,
            RuntimeError,
            OSError,
        ) as error:
            for key, old in binding_execution.items():
                setattr(self, key, old)
            for key, old in promotion_snapshot.items():
                setattr(self, key, old)
            self.path_admission_hold = True
            self.path_admission_replan_pending = True
            self.last_spline_rejected = "Current path rejected: " + str(error)
            self.message = self.last_spline_rejected + "; waiting for replan"
            self.path_admission_last_decision = dict(admitted=False, reason=str(error))
            self.publish_command()
            return
