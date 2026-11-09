"""Admission lifecycle for ``PIDNavigation``.

Owns admission identity/eligibility, admitted-path retention, corridor/path
replacement, SCAN definition/spline intake and replan/hold bookkeeping. The
large admission decision body itself is in :mod:`pid_admission_apply`.
"""

from .pid_support import (
    Path,
    np,
    time,
    remaining_certificate,
    json,
    arm_teacher_recovery,
    observe_business_gate,
    minimal_raw_imu_fresh,
    record_clear_release,
    reset_transition,
    minimal_wall_stale,
    finite_retention_reference,
    minimal_pose_cloud_fresh,
    handoff_context,
    math,
    _r102_progress,
    _sensor_hold_end,
    _r87_diag,
    _r85_intake_outer,
    retention_reference_eligibility,
    replayable,
    pending_path_waiting,
    diagnostic_profile_view,
    copy,
    _sensor_hold_begin,
)


class AdmissionMixin:
    """Route/path admission lifecycle, retention and replan/hold handling."""

    def admission_identity(self):
        return dict(
            request_id=self.request_id,
            waypoint_index=self.waypoint_index,
            goal_xyz=self.waypoints[self.waypoint_index].tolist(),
        )

    def path_execution_eligibility(self):
        """Per-condition original live-source/identity evidence, without TTL refresh."""
        active = self.path_admission_active
        now = time.monotonic()
        ns = self.get_clock().now().nanoseconds

        def fresh_wall(value):
            return (
                isinstance(value, (int, float)) and math.isfinite(value) and 0 <= now - value <= 0.3
            )

        conditions = dict(
            active_present=active is not None,
            same_request_waypoint_goal=active is not None
            and active["identity"] == self.admission_identity(),
            same_active_trajectory=active is not None
            and active["trajectory_id"] == self.active_trajectory_id,
            same_active_reference=active is not None
            and tuple(active["active_reference_stamp"]) == tuple(self.active_reference_stamp or ()),
            samples_present=self.samples is not None,
            state_running=self.state == "running",
            pose_source_fresh=minimal_pose_cloud_fresh(
                self, "pose", now, ns / 1e9, fresh_wall(self.pose_updated)
            ),
            cloud_source_fresh=minimal_pose_cloud_fresh(
                self, "cloud", now, ns / 1e9, fresh_wall(self.cloud_updated)
            ),
            raw_imu_source_fresh=minimal_raw_imu_fresh(
                self, now, ns / 1e9, self.raw_imu.fresh(now, ns / 1e9)
            ),
            bridge_source_fresh=not minimal_wall_stale(
                self,
                "bridge_safety",
                now,
                self.bridge_updated,
                0.3,
                not fresh_wall(self.bridge_updated),
            ),
            bridge_ready=self.bridge_safety.get("state") == "ready",
            obstacle_not_held=not self.obstacle_hold,
            tilt_not_held=not self.tilt_hold,
        )
        return dict(
            conditions=conditions,
            samples_present=conditions["samples_present"],
            clock_ns=ns,
            checked_monotonic_wall=now,
            actual_source_times_refreshed=False,
        )

    @_r87_diag.observe("scan.retain_admitted_path")
    def retain_admitted_path(self, points, config, *, max_route_points=256, binding=None):
        if self.r85_physical_gate is not None:
            try:
                frozen_points, frozen_config, frozen_binding = self._route_fence_load()
                owned = (
                    points is frozen_points
                    and config is frozen_config
                    and (binding is None or binding is frozen_binding)
                )
                # Normal candidates carry this program-owned tuple. Retain the
                # original comparison only for the legacy external pending caller.
                if (
                    not owned
                    and (
                        not np.array_equal(points, frozen_points)
                        or config != frozen_config
                        or binding is not None
                        and binding != frozen_binding
                    )
                    or self.automatic_reference.config.get("max_reference_points") != 4096
                ):
                    raise ValueError("r85_retention_fence_capacity_or_binding_mismatch")
                binding = frozen_binding
                max_route_points = 4096
            except (
                OSError,
                KeyError,
                ValueError,
                TypeError,
                AttributeError,
                IndexError,
                OverflowError,
            ) as error:
                return dict(
                    allowed=False,
                    reason="r85_retention_frozen_fence_rejected",
                    error=str(error),
                    samples_present=self.samples is not None,
                )
        retained_context = handoff_context(self)
        if not retained_context["allowed"]:
            return dict(
                allowed=False,
                reason="old_path_coverage_or_context_not_eligible",
                eligibility=retained_context,
                conditions=retained_context["conditions"],
                samples_present=self.samples is not None,
            )
        # New planner reference stamps are pending identities, not new receipts
        # for the path being executed. Progress is projected spatially only.
        eligibility = self.path_execution_eligibility()
        if not all(eligibility["conditions"].values()):
            return dict(allowed=False, reason="old_path_not_eligible_stopped", **eligibility)
        reference_execution = retention_reference_eligibility(self)
        if (
            not reference_execution["allowed"]
            and diagnostic_profile_view(self.profile)["diagnostic_observe_only"]["enabled"]
            and finite_retention_reference(self, reference_execution)
        ):
            reference_execution = observe_business_gate(
                self,
                "retained_SCAN_reference_prediction",
                reference_execution,
                {
                    "old_reference_curvature_unavailable_requires_stop_or_new_path",
                    "old_reference_direction_incoherent_requires_stop_or_new_path",
                },
            )
        if not reference_execution["allowed"]:
            return dict(
                allowed=False,
                reason=reference_execution["reason"],
                reference_execution=reference_execution,
                eligibility=eligibility,
                conditions=eligibility["conditions"],
                samples_present=True,
            )
        observer_options = {}
        if diagnostic_profile_view(self.profile)["diagnostic_observe_only"]["enabled"]:

            def observe_remaining_score(score):
                observed = observe_business_gate(
                    self,
                    "retained_SCAN_route_fence_prediction",
                    score,
                    {
                        "outside_fence",
                        "depth_budget_exhausted",
                        "patch_budget_exhausted",
                        "wall_budget_exhausted",
                    },
                )
                return observed.get("allowed") is True

            observer_options["business_observer"] = observe_remaining_score
        proof = remaining_certificate(
            self.path_admission_active,
            self.samples,
            self.pose,
            points,
            config,
            max_route_points=max_route_points,
            **observer_options,
        )
        if proof["allowed"] and self.r85_physical_gate is not None:
            physical = self.r85_physical_gate.retain(
                self, self.path_admission_active, proof, binding
            )
            proof = dict(
                proof,
                r85_physical_retention=physical,
                allowed=physical["allowed"],
                reason=proof["reason"] if physical["allowed"] else physical["reason"],
            )
        return dict(
            proof,
            eligibility=eligibility,
            conditions=eligibility["conditions"],
            samples_present=True,
            reference_execution=reference_execution,
            retention_action="kept_checked_old_path"
            if proof["allowed"]
            else "stopped_old_path_not_certified",
        )

    @_r87_diag.observe("scan.capture_replacement")
    def capture_replacement_eligibility(self, retention):
        """Recover only the recorded exhaustion of this immutable same-goal path.

        A missing path, source loss, protection, or failed old proof by itself
        cannot create this authorization. New-curve proof and base acceptance
        remain separate necessary gates; this evidence never commands motion.
        """
        marker = getattr(self, "execution_path_exhaustion", None)
        active = self.path_admission_active
        receipt = self.path_receipt or {}
        eligibility = self.path_execution_eligibility()
        conditions = {k: v for k, v in eligibility["conditions"].items() if k != "samples_present"}
        conditions.update(
            samples_absent=self.samples is None,
            capture_same_goal=self.cascade is not None
            and self.cascade_key == (self.request_id, self.waypoint_index)
            and self.cascade.capture_stamp is not None,
            cascade_not_protected=self.cascade is not None and not self.cascade.protected,
            cascade_failure_not_latched=self.cascade is not None
            and self.cascade.failure_latched is None,
            exhaustion_receipt_present=isinstance(marker, dict),
        )
        if isinstance(marker, dict):
            conditions.update(
                exhaustion_internal_schema=marker.get("schema")
                == "internal_checked_path_exhaustion/v1"
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
                == list(self.active_reference_stamp or ()),
                exhaustion_matches_accepted_samples=marker.get("previous_samples_sha256")
                == receipt.get("source_samples_float64_sha256")
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
                and 0
                <= marker["source_pose_stamp_ns"]
                <= marker["control_stamp_ns"]
                <= eligibility["clock_ns"],
            )
        else:
            conditions["exhaustion_internal_schema"] = False
        allowed = all(conditions.values())
        return dict(
            allowed=allowed,
            reason="recorded_exhausted_capture_can_rebind"
            if allowed
            else "capture_replacement_not_eligible_stopped",
            conditions=conditions,
            samples_present=self.samples is not None,
            exhaustion_receipt=copy.deepcopy(marker),
            source_eligibility=eligibility,
            old_retention_reason=retention.get("reason"),
            actual_source_times_refreshed=False,
        )

    def clear_committed_exhausted_capture(self):
        # Only called after all existing execution gates committed a new path.
        # Keep the fixed goal, its original activation/90s deadline, velocity PI,
        # filters, feedback clocks and command slew state. A new capture must
        # obtain fresh original dwell; old parking proof is not transferable.
        cascade = self.cascade
        old = {
            key: copy.deepcopy(getattr(cascade, key))
            for key in (
                "capture_stamp",
                "capture_wall",
                "capture_clock",
                "arrival",
                "capture_dwell",
                "hold_stamp",
                "hold_wall",
                "hold_clock",
            )
        }
        for key in old:
            setattr(cascade, key, None)
        cascade.hold_integral[:] = 0.0
        self.execution_path_exhaustion = None
        self.pid_guard_needs_evaluation = True
        return dict(
            cleared_capture=old,
            fixed_goal_and_deadline_preserved=True,
            velocity_PI_filters_and_slew_preserved=True,
            fresh_capture_dwell_required=True,
        )

    @_r87_diag.observe("scan.archive_admission")
    def archive_admission(self, value, metadata, decision, started):
        self.path_admission_sequence += 1
        sequence = self.path_admission_sequence
        decision = dict(
            decision,
            schema="SCAN_path_execution_admission/v1",
            sequence=sequence,
            request_id=self.request_id,
            waypoint_index=self.waypoint_index,
            association_payload_id=None if value is None else value.get("traj_id"),
            active_trajectory_id=self.active_trajectory_id,
            active_reference_stamp=self.active_reference_stamp,
            admission_replan_pending=self.path_admission_replan_pending,
            pending_reference_stamp=self.trajectory_association.reference_stamp,
            control_stamp_ns=self.get_clock().now().nanoseconds,
            source_pose_stamp_ns=int(self.pose_stamp),
            callback_elapsed_wall_ms=(time.monotonic() - started) * 1000,
            navigation_ground_truth_used=False,
            occupancy_or_terrain_support_certified=False,
        )
        self.path_admission_last_decision = copy.deepcopy(decision)
        document = replayable(
            dict(decision=decision, payload=value, metadata=copy.deepcopy(metadata))
        )
        relative = Path("path_admission_candidates") / (
            str(sequence).zfill(6)
            + "_trajectory_"
            + str(decision["association_payload_id"])
            + ".json"
        )

        def archive():
            target = self.run / relative
            target.parent.mkdir(exist_ok=True)
            with target.open("x") as out:
                out.write(json.dumps(document, allow_nan=False, ensure_ascii=False) + "\n")

        self.evidence.enqueue(archive)
        self.evidence.append(
            self.run / "path_admission_decisions.jsonl",
            dict(document["decision"], candidate_file=str(relative)),
        )
        _r85_intake_outer(self, value, metadata, decision, "original_archive_admission_completed")
        _r102_progress.note(
            self,
            "admission_verdict",
            decision.get("admitted"),
            decision.get("reason"),
            decision.get("control_stamp_ns"),
            decision,
        )

    def _route_definition_bound(self):
        """Low-frequency program binding; never copy/hash the route per candidate."""
        runtime = self.automatic_reference
        key = getattr(self.trajectory_association, "route_key", None)
        if runtime is None or key is None:
            raise ValueError("route_definition_not_bound")
        points, config, binding = self.path_admission_fence.load()
        if self.r85_physical_gate is not None:
            self.r85_physical_gate.bind_route_definition(self, binding)
        self._program_route_fence = (self._program_route_key(), points, config, binding)
        # One ordinary program representation of the same native guard input.
        self._program_guard_route = np.asarray(points, dtype=float)
        self._program_guard_route_reference = None

    def _program_route_key(self):
        runtime = self.automatic_reference
        return (
            id(runtime),
            runtime.file_sha,
            tuple(runtime.scope),
            self.request_id,
            getattr(self.trajectory_association, "route_key", None),
        )

    def _route_fence_load(self):
        if self.automatic_reference is None:
            return self.path_admission_fence.load()
        current = getattr(self, "_program_route_fence", None)
        if current is None or current[0] != self._program_route_key():
            self._route_definition_bound()
            current = self._program_route_fence
        return current[1:]

    @_r87_diag.observe("callback.scan_definition")
    def on_route_definition(self, msg):
        asynchronous = getattr(self, "_r88_admission", None)
        if asynchronous is not None:
            return asynchronous.receive("definition", msg, super().on_route_definition)
        return super().on_route_definition(msg)

    @_r87_diag.observe("callback.scan_metadata")
    def on_trajectory_metadata(self, msg):
        asynchronous = getattr(self, "_r88_admission", None)
        if asynchronous is not None:
            return asynchronous.receive("metadata", msg, super().on_trajectory_metadata)
        return super().on_trajectory_metadata(msg)

    @_r87_diag.observe("callback.scan_spline")
    def on_spline(self, msg):
        asynchronous = getattr(self, "_r88_admission", None)
        if asynchronous is not None:
            return asynchronous.receive("spline", msg, super().on_spline)
        return super().on_spline(msg)

    @_r87_diag.observe("callback.scan_attempt")
    def on_reference_attempt(self, msg):
        asynchronous = getattr(self, "_r88_admission", None)
        if asynchronous is not None:
            return asynchronous.receive("attempt", msg, super().on_reference_attempt)
        return super().on_reference_attempt(msg)

    @_r87_diag.observe("scan.accept_spline")
    def accept_spline(self, msg, metadata):
        self.pending_path_commit = None
        result = self._admit_matched_spline(msg, metadata)
        return result

    def request_admission_replan_if_due(self, now):
        if pending_path_waiting(self):
            return
        pending = (
            self.reference_replan_pending
            or self.path_admission_replan_pending
            or self.path_admission_hold
        )
        # A previous goal's pending replan must wait for inherited control to
        # activate the next segment from a fresh, unprotected actual SLAM pose.
        if (
            pending
            and self.state == "running"
            and now - self.last_reference > 3.0
            and (
                not getattr(self, "global_reference_settings", {"enabled": False})["enabled"]
                or self.segment_start is not None
            )
        ):
            self.evidence.append(
                self.run / "reference_replan_events.jsonl",
                dict(
                    reason="path_admission_rejection"
                    if self.path_admission_replan_pending or self.path_admission_hold
                    else "reference_constraint_hold",
                    admission_reason=None
                    if self.path_admission_last_decision is None
                    else self.path_admission_last_decision["reason"],
                    request_id=self.request_id,
                    waypoint_index=self.waypoint_index,
                    trajectory_id=self.active_trajectory_id,
                    source_pose_stamp_ns=int(self.pose_stamp),
                    control_stamp_ns=self.get_clock().now().nanoseconds,
                    navigation_ground_truth_used=False,
                ),
            )
            before = self.reference_requests
            self.request_plan()
            if self.reference_requests > before:
                self.reference_replan_pending = False
                self.path_admission_replan_pending = False

    def control_admission_hold(self, now):
        # Keep the inherited deadlines, measured IMU/bridge protection and
        # source-loss failure active while no execution-admitted path exists.
        # Holding does not tick the cascade, heading or arrival integrators.
        ns = self.get_clock().now().nanoseconds
        ros_now = ns / 1e9
        if self.state != "running":
            self.publish_command()
            return
        if (
            self.segment_start is not None
            and not self.goals[self.waypoint_index].legacy
            and self.segment_deadline_exceeded(ns)
        ):
            self.reset_region_arrival("failed")
            self.state, self.message = "failed", "当前航点超过仿真时间限制，停车"
            self.publish_command()
            return
        protected = self.apply_tilt_guard(now, ros_now)
        _sensor_hold_row = _sensor_hold_begin(self, "path_admission", now, ns)
        if self.state == "failed":
            _sensor_hold_end(self, _sensor_hold_row, None)
            self.reset_region_arrival("failed")
            self.publish_command()
            return
        stale = (
            now - self.pose_updated > self.pose_timeout
            or now - self.cloud_updated > self.cloud_timeout
            or not self.raw_imu.fresh(now, ros_now)
        )
        _r102_progress.note(
            self,
            "admission_hold",
            True,
            "original_stale"
            if stale
            else "original_protected"
            if protected
            else "path_admission_hold",
            ns,
        )
        if stale:
            self.reset_region_arrival("stale")
            self.stale_since = self.stale_since or now
            self.message = "SLAM、点云或原始 IMU 超时，停车等待传感器"
            if now - self.stale_since > 8.0:
                self.state, self.message = "failed", "SLAM、点云或原始 IMU 连续失联超过 8 秒"
            _sensor_hold_end(self, _sensor_hold_row, stale)
        elif self.state == "failed":
            self.reset_region_arrival("failed")
            _sensor_hold_end(self, _sensor_hold_row, stale)
        elif protected:
            self.reset_region_arrival("protected")
            _sensor_hold_end(self, _sensor_hold_row, stale)
        else:
            self.stale_since = None
            self.reset_region_arrival("path_admission_hold")
            _sensor_hold_end(self, _sensor_hold_row, stale)
            if self.segment_start is None:
                self.segment_start = self.pose.copy()
                self.segment_started_ros = ros_now
                self.segment_started_ros_ns = ns
                self.segment_start_pose_stamp_ns = int(self.pose_stamp)
                self.record_segment_activation()
            # Path admission must not starve the existing obstacle-clear
            # observation gate. This checks the original native geometry at
            # exact zero; it cannot release path admission or arrival dwell.
            if (
                getattr(self, "global_reference_settings", {"enabled": False})["enabled"]
                and self.obstacle_hold
                and (now - self.last_obstacle_check >= 0.05 or self.pid_guard_needs_evaluation)
            ):
                self.publish_command()
                goal = self.waypoints[self.waypoint_index]
                target = np.asarray((self.steering or {}).get("target", goal), dtype=float)
                direction = np.asarray(
                    (self.steering or {}).get("direction", (goal - self.pose)[:2]), dtype=float
                )
                result = self.evaluate_motion_guard(
                    self.cloud,
                    self.pose,
                    target,
                    direction,
                    self.execution_guard_route(self.segment_start, goal),
                )
                self.obstacle_result = result
                self.last_obstacle_check = now
                if result[0] or not self.heading_gate.stopped(ros_now, now):
                    self.obstacle_clear_since = None
                else:
                    self.obstacle_clear_since = self.obstacle_clear_since or ros_now
                    if ros_now - self.obstacle_clear_since >= 1.0 and getattr(
                        self, "_actual_clear_guard_ready", False
                    ):
                        self.obstacle_hold = False
                        self.obstacle_resumes += 1
                        self.obstacle_stopped_duration = ros_now - (
                            ros_now
                            if self.obstacle_hold_started_ros is None
                            else self.obstacle_hold_started_ros
                        )
                        arm_teacher_recovery(self)
                        self.obstacle_resume_pending = True
                        reset_transition(self, "dynamic_recovery")
                        self.samples = None
                        self.execution_path_exhaustion = None
                        # Retain the admission hold. Only a newly associated,
                        # execution-admitted curve can authorize future motion.
                if self.pending_guard_row is not None:
                    row = self.pending_guard_row
                    row.update(
                        obstacle_hold_after=bool(self.obstacle_hold),
                        clear_start_after_s=self.obstacle_clear_since,
                        after_control_ros_clock_ns=ns,
                        command_after_control=list(self.command),
                        zero_requested_after_control=True,
                        state_after_control=self.state,
                        admission_hold_observation_only=True,
                        path_admission_hold_after=self.path_admission_hold,
                        obstacle_resumes_after=self.obstacle_resumes,
                        evidence_queue_error=self.evidence.error,
                    )
                    record_clear_release(self, row)
                    self.evidence.append(self.run / "navigation_guard_history.jsonl", row)
                    self.pending_guard_row = None
        self.publish_command()
