"""Velocity control and publication for ``PIDNavigation``.

Owns cascade setup, ACK cache, PID velocity selection, motion guard, terminal
protection/parking ownership and command publication. The base task state
machine remains in ``base_node.Navigation.control()``.
"""

from .pid_support import (
    constrain_exactzero_launch,
    np,
    time,
    _r86_probe_guard_begin,
    json,
    ordinary_parking_enabled,
    _r47_guard_dimensions,
    _copy_path_receipt_metadata,
    _r47_publication_anchor,
    Controller,
    StableProjectionController,
    _pre_turn_external_hold_seed_wait,
    _append_pid_row_evidence,
    _pid_receipt_json_bytes,
    retain_native_context,
    _sensor_hold_end,
    _r47_bind,
    _r47_end,
    make_event,
    _r86_probe_guard_end,
    _r118_source_age,
    OrderedDict,
    slew,
    _r67_attach_known_frame_failure,
    _r96_cpu,
    CausalHandoffProjectionController,
    os,
    _r105_terminal,
    _r47_begin,
    remember_native_guard,
    diagnostic_profile_view,
    _callback_cost_observe,
    copy,
    commit_pending_path,
    minimal_baseline_enabled,
    _sensor_hold_begin,
    _sensor_parking_protection,
    _r47_mark,
)


class ControlMixin:
    """Cascade/velocity selection, ACK gating, parking, publication and terminal protection."""

    def ensure_cascade(self):
        if getattr(self, "_r105_terminal_context", None) is not None:
            _r105_terminal.validate_context(self)
            self.cascade.diagnostic_observe_only = dict(
                run=self.run, profile=diagnostic_profile_view(self.profile)
            )
            return
        key = (self.request_id, self.waypoint_index)
        if self.cascade_key != key:
            self.reference_replan_pending = False
            self.pid_prepared_output = None
            self.pid_row = None
            self._pid_row_receipt_snapshot = None
            goal = self.waypoints[self.waypoint_index]
            yaw = self.fixed_terminal_heading(self.waypoint_index, self.segment_start)
            core = (
                CausalHandoffProjectionController
                if self.continuous_route is not None
                else StableProjectionController
                if getattr(self, "global_reference_settings", {"enabled": False})["enabled"]
                else Controller
            )
            self.cascade = core(
                self.profile["cascade"], goal.tolist(), yaw, ":".join(map(str, key))
            )
            self.cascade.minimal_simulation_baseline = minimal_baseline_enabled(self.profile)
            if "r85_neutral_rate_brake_capture" in self.profile:
                self.cascade.configure_neutral_rate_brake_capture(self.profile)
            if "r85_teacher_pre_zero_heading_p_taper" in self.profile:
                self.cascade.configure_teacher_pre_zero_heading_p_taper(
                    self.profile, self.run.name, self.handoff_scope_sha256
                )
            self.reference_consistency_hold = False
            self.reference_consistency_check = None
            self.cascade_key = key
            self.cascade_path_id = None
            self.evidence.append(
                self.run / "navigation_fixed_goal_references.jsonl",
                dict(
                    request_id=self.request_id,
                    waypoint_index=self.waypoint_index,
                    segment_start=self.segment_start.tolist(),
                    goal=goal.tolist(),
                    heading_rad=yaw,
                    source_pose_stamp_ns=self.segment_references[key]["source_pose_stamp_ns"],
                    control_stamp_ns=self.segment_references[key]["control_stamp_ns"],
                    goals_definition_sha256=self.goals_definition_sha256,
                    fixed_goal=self.cascade.goal_record,
                    fixed_goal_sha256=self.cascade.goal_sha,
                    navigation_ground_truth_used=False,
                ),
            )
        if (
            self.cascade.capture_stamp is None
            and self.path_receipt
            and (self.path_receipt["request_id"], self.path_receipt["waypoint_index"]) == key
            and self.path_receipt["path_id"] != self.cascade_path_id
        ):
            r = self.path_receipt
            self.cascade.set_path(
                r["points_xyz"],
                r["path_id"],
                r["stamp_ns"],
                r["received_wall_ns"],
                r["frame_id"],
                position=self.pose,
            )
            self.cascade_path_id = r["path_id"]
            self.evidence.append(
                self.run / "navigation_cascade_paths.jsonl",
                dict(
                    **self.cascade.path_record,
                    path_sha256=self.cascade.path_sha,
                    trajectory_id=r["trajectory_id"],
                    array_file=r["array_file"],
                    source_sample_indices=r["source_sample_indices"],
                    source_samples_count=r["source_samples_count"],
                    source_samples_float64_sha256=r["source_samples_float64_sha256"],
                    points_float64_sha256=r["points_float64_sha256"],
                    navigation_ground_truth_used=False,
                ),
            )
        if self.cascade is not None:
            self.cascade.diagnostic_observe_only = dict(
                run=self.run, profile=diagnostic_profile_view(self.profile)
            )

    def _decode_ack_entries_reusing_lines(self, raw):
        # Parse-tree reuse only: causal selection, freshness and selected deepcopy
        # remain in get_ack on every call. Cached mutable trees stay private.
        prefix = b'{"entries":['
        suffix = b'],"schema":"teacher_known_velocity_ack/v1"}\n'
        if not (raw.startswith(prefix) and raw.endswith(suffix)):
            return json.loads(raw)["entries"]
        body = raw[len(prefix) : -len(suffix)]
        if not body:
            return []
        lines = body.split(b",\n", 32)
        if len(lines) > 32 or any(
            not (line.startswith(b"{") and line.endswith(b"}")) or b"\n" in line or b"\r" in line
            for line in lines
        ):
            return json.loads(raw)["entries"]
        cache = getattr(self, "_ack_parsed_line_lru", None)
        if cache is None:
            cache = OrderedDict()
        entries = []
        try:
            for line in lines:
                if line in cache:
                    row = cache[line]
                    cache.move_to_end(line)
                else:
                    row = json.loads(line)
                    if not isinstance(row, dict):
                        return json.loads(raw)["entries"]
                    cache[line] = row
                    if len(cache) > 32:
                        cache.popitem(last=False)
                entries.append(row)
        except (ValueError, TypeError, RecursionError):
            return json.loads(raw)["entries"]
        self._ack_parsed_line_lru = cache
        return entries

    def _read_ack_entries_cached(self, path):
        # Cache parsing only. Every get_ack still selects against its own real
        # pose header; no freshness or causal result is memoized.
        def key(stat):
            return (stat.st_ino, stat.st_mtime_ns, stat.st_size)

        observed = key(path.stat())
        if getattr(self, "_ack_entries_cache_key", None) == observed and hasattr(
            self, "_ack_entries_cache"
        ):
            return self._ack_entries_cache
        with path.open("rb") as stream:
            before = key(os.fstat(stream.fileno()))
            raw = stream.read()
            after = key(os.fstat(stream.fileno()))
        if before != after or len(raw) != after[2]:
            raise ValueError("Actual ACK changed during same-FD read")
        entries = self._decode_ack_entries_reusing_lines(raw)
        if not isinstance(entries, list):
            raise ValueError("Actual ACK entries must be a list")
        self._ack_entries_cache_key = after
        self._ack_entries_cache = entries
        return entries

    @_r96_cpu.observe("ack.get_ack")
    def get_ack(self, pose_stamp_ns=None):
        if pose_stamp_ns is None:
            if not self.feedback:
                return None
            pose_stamp_ns = self.feedback["stamp_ns"]
        try:
            entries = self._read_ack_entries_cached(self.run / "closed_loop_executor_ack.json")
            from embodied_navigation.robot.unitree.policy.lower_loop_ack import (
                configuration as lower_ack_configuration,
            )

            if (
                lower_ack_configuration(self.profile["cascade"].get("r85_student_lower_loop_ack"))
                is not None
            ):
                expected_scope = self.handoff_scope_sha256
                selected = next(
                    (
                        r
                        for r in reversed(entries)
                        if r["stamp_ns"] <= pose_stamp_ns and r.get("run_id") == expected_scope
                    ),
                    None,
                )
            else:
                selected = next(
                    (r for r in reversed(entries) if r["stamp_ns"] <= pose_stamp_ns), None
                )
            # Cached parse trees remain private; callers own only this small ACK.
            return copy.deepcopy(selected)
        except (OSError, KeyError, ValueError, TypeError):
            return None

    @_r96_cpu.observe("select")
    def select_pid_velocity(
        self, velocity, yaw_rate, target, allow_translation, mode_override=None
    ):
        if _r96_cpu.PROBE is None:
            ns = self.get_clock().now().nanoseconds
            wall = time.monotonic_ns()
        else:
            with _r96_cpu.span("select.ros_clock", self):
                ns = self.get_clock().now().nanoseconds
                wall = time.monotonic_ns()
        if _r118_source_age.ENABLED:
            _r118_source_age.controller(self, "select.enter", ns, feedback=True)
        _r47_call = _r47_begin(self, "select", ns, wall)
        try:
            _r47_mark(self, _r47_call, "ensure_cascade_before")
            self.ensure_cascade()
            _r47_mark(self, _r47_call, "ensure_cascade_after")
            controlled_settle = bool(getattr(self.heading_gate, "controlled_settle", False))
            controlled_braking = bool(getattr(self.heading_gate, "controlled_braking", False))
            active_settle = controlled_settle or controlled_braking
            mode = mode_override or (
                "hold"
                if self.obstacle_hold
                or getattr(self, "reference_consistency_hold", False)
                or self.heading_gate.phase == "pre_turn"
                or self.heading_gate.phase == "settle"
                and not active_settle
                else "brake"
                if controlled_braking
                else "drive"
                if allow_translation
                else "turn"
            )
            handoff_failure = getattr(self.heading_gate, "handoff_failure", None)
            if handoff_failure:
                self.state = "failed"
                self.message = "主动转向交接失败：" + str(handoff_failure)
                mode = "hold"
            heading_reference = copy.deepcopy(self.steering)
            gate_heading = self.heading_gate.heading
            if (
                self.imu_source_buffer is not None
                and self.feedback is not None
                and self.paired_imu is None
            ):
                paired = self.imu_source_buffer.pair(self.feedback["stamp_ns"], None, ns)
                if paired is not None:
                    self.paired_imu = copy.deepcopy(paired)
                    self.imu_source_buffer.event(
                        self.feedback["stamp_ns"], ns, wall, "current_pose_pair_from_fifo", paired
                    )
            _r47_mark(self, _r47_call, "get_ack_before")
            ack = self.get_ack()
            _r47_mark(self, _r47_call, "get_ack_after")
            guard = None
            if self.feedback is None or self.paired_imu is None or ack is None:
                guard = "actual_causal_feedback_or_ack_missing"
            if handoff_failure:
                guard = "active_turn_handoff_failure:" + str(handoff_failure)
            context = (self.request_id, self.waypoint_index)
            previous = getattr(self, "_pre_turn_hold_seed_context", None)
            if previous is not None:
                previous = dict(
                    previous,
                    uninterrupted=bool(
                        self.cascade.protected
                        and self.cascade.protection_reason == "external_hold"
                        and not self.cascade.recovery
                        and self.cascade.last_math_stamp is None
                        and self.cascade.last_clock == previous["clock_ns"]
                        and self.cascade.last_wall == previous["wall_ns"]
                    ),
                )
            unload_event = None
            if (
                getattr(self.heading_gate, "integral_unload_config", None) is not None
                and mode == "brake"
                and guard is None
            ):
                unload_event = self.heading_gate.bind_integral_unload(
                    self.run.name,
                    self.handoff_scope_sha256,
                    self.cascade.path_sha,
                    self.cascade.goal_sha,
                    ns,
                )
                if self.heading_gate.handoff_failure:
                    guard = "active_turn_handoff_failure:" + self.heading_gate.handoff_failure
            update_args = dict(
                guard_reason=guard,
                mode_override=mode,
                external_turn_heading=gate_heading
                if mode in ("turn", "brake")
                and (self.heading_gate.phase == "align" or active_settle)
                else None,
            )
            if unload_event is not None:
                update_args["pre_zero_unload_event"] = unload_event
            _r47_mark(self, _r47_call, "core_update_before")
            cmd, diagnostic = self.cascade.update(
                self.feedback, self.paired_imu, ack, ns, wall, **update_args
            )
            _r47_mark(self, _r47_call, "core_update_after")
            if (
                diagnostic.get("controller_updated")
                and diagnostic.get("kind") == "ordinary_preT0_recapture"
            ):
                self.evidence.append(
                    self.run / "ordinary_preT0_attempts.jsonl",
                    diagnostic["preT0_attempt_disqualified"],
                )
            seed_wait = _pre_turn_external_hold_seed_wait(
                previous,
                self.cascade,
                self.heading_gate,
                context,
                mode,
                guard,
                diagnostic,
                cmd,
                ns,
                wall,
            )
            # Consume the token on any non-pre-turn call; a duplicate or another
            # protection cannot reuse this single valid-source seed exception.
            self._pre_turn_hold_seed_context = (
                dict(
                    cascade=self.cascade,
                    context=context,
                    clock_ns=ns,
                    wall_ns=wall,
                    gate_context=copy.deepcopy(self.heading_gate.transition_context),
                    purpose=self.heading_gate.transition_purpose,
                )
                if mode == "hold"
                and self.heading_gate.phase == "pre_turn"
                and guard is None
                and not getattr(self.heading_gate, "handoff_active", False)
                and diagnostic.get("mode") == "protect"
                and diagnostic.get("reason") == "external_hold"
                and diagnostic.get("failure_latched") is None
                else None
            )
            if seed_wait:
                diagnostic["handoff_seed_recovery_wait"] = True
            if (
                getattr(self.heading_gate, "handoff_active", False)
                and (
                    guard is not None
                    or diagnostic.get("mode") in ("protect", "recovering")
                    and not (mode == "hold" and diagnostic.get("reason") == "external_hold")
                )
                and not seed_wait
            ):
                self.heading_gate.fail_handoff("cascade_source_or_control_protection", ns / 1e9)
                self.state = "failed"
                self.message = "主动转向交接失败：" + str(self.heading_gate.handoff_failure)
            if diagnostic.get("failure_latched"):
                self.state = "failed"
                self.message = "串级真实输入保护锁定：" + str(diagnostic["failure_latched"])
            self.cascade_last_row = diagnostic
            self.pid_guard_needs_evaluation = (
                bool(diagnostic.get("controller_updated")) or self.control_clock_hold.guard_required
            )
            self.reference_replan_pending = (
                getattr(self, "reference_consistency_hold", False)
                or diagnostic["mode"] == "reference_constraint_hold"
            )
            _r47_mark(self, _r47_call, "postcore_protection_logic_complete")
            desired = cmd.copy()
            # Sample only time already elapsed at the final slew decision.
            # Later native guard time is neither predicted nor credited here.
            cmd = (
                np.zeros(3)
                if diagnostic["mode"]
                in (
                    "protect",
                    "recovering",
                    "pre_turn",
                    "settle",
                    "path_end_hold",
                    "reference_constraint_hold",
                    "ordinary_zero_hold",
                )
                else slew(
                    self.command,
                    desired,
                    self.get_clock().now().nanoseconds / 1e9 - self.last_command_time,
                    [0.6, 0.6, 0.8],
                )
            )
            if diagnostic["mode"] == "ordinary_zero_hold":
                cmd[:] = 0.0
                launch_constraint = dict(
                    reason="identity_bound_ordinary_terminal_zero", exact_body_zero=True
                )
            else:
                cmd, launch_constraint = constrain_exactzero_launch(
                    self.command, desired, cmd, diagnostic["mode"], self.rotation[:2, :2]
                )
            self.pid_prepared_output = cmd.copy()
            world_direction = self.rotation[:2, :2] @ cmd[:2]
            if np.linalg.norm(world_direction) > 1e-9:
                self.steering["direction"] = world_direction.tolist()
            self.pid_records += 1
            pid_receipt_bytes = _pid_receipt_json_bytes(self)
            self.pid_row = dict(
                schema="teacher_closed_loop_cascade_tick/v1",
                sequence=self.pid_records,
                cascade_sequence=self.pid_records,
                control_pose_stamp_ns=int(self.pose_stamp),
                source_pose_stamp_ns=int(self.pose_stamp),
                paired_imu_stamp_ns=None
                if self.paired_imu is None
                else self.paired_imu["stamp_ns"],
                control_stamp_ns=ns,
                compute_ros_clock_ns=ns,
                compute_monotonic_wall=wall / 1e9,
                feedback=copy.deepcopy(self.feedback),
                imu=copy.deepcopy(self.paired_imu),
                ack=copy.deepcopy(ack),
                heading_gate_reference=dict(
                    phase=self.heading_gate.phase,
                    locked_heading=gate_heading,
                    steering=heading_reference,
                    controlled_settle=controlled_settle,
                    controlled_braking=controlled_braking,
                    settle_control_stage=getattr(self.heading_gate, "settle_control_stage", None),
                    zero_handoff_count=getattr(self.heading_gate, "zero_handoff_count", None),
                    last_zero_handoff=copy.deepcopy(
                        getattr(self.heading_gate, "last_zero_handoff", None)
                    ),
                ),
                path_receipt=None
                if minimal_baseline_enabled(self.profile)
                or pid_receipt_bytes is not None
                or self.path_receipt is None
                else {
                    k: (_copy_path_receipt_metadata(v) if k == "metadata" else copy.deepcopy(v))
                    for k, v in self.path_receipt.items()
                    if k != "points_xyz"
                },
                cascade=diagnostic,
                control_pose=self.pose.tolist(),
                control_quaternion=self.pose_quaternion,
                control_rotation=self.rotation.tolist(),
                request_id=self.request_id,
                waypoint_index=self.waypoint_index,
                trajectory_id=self.active_trajectory_id,
                trajectory_archive_file=self.trajectory_archive_reference,
                mode=diagnostic["mode"],
                checked_target=np.asarray(target).tolist(),
                goal=self.waypoints[self.waypoint_index].tolist(),
                actual_pid_world_direction=world_direction.tolist(),
                steering_direction=self.steering["direction"],
                desired_body_command=desired.tolist(),
                prepared_after_slew_command=cmd.tolist(),
                launch_output_constraint=launch_constraint,
                navigation_ground_truth_used=False,
            )
            self._pid_row_receipt_snapshot = (
                (self.pid_row, pid_receipt_bytes) if pid_receipt_bytes is not None else None
            )
            if getattr(self.heading_gate, "quiet_target_enabled", False):
                self.pid_row["quiet_handoff_binding"] = self.heading_gate._quiet_binding(
                    self.heading_gate.transition_context,
                    self.heading_gate.transition_purpose,
                    self.handoff_scope_sha256,
                    diagnostic.get("path_id"),
                    diagnostic.get("path_sha256"),
                    diagnostic.get("fixed_goal_sha256"),
                )
            _r47_bind(self, _r47_call, self.pid_row)
            _r47_mark(self, _r47_call, "postcore_row_complete")
            if _r118_source_age.ENABLED:
                _r118_source_age.controller(self, "select.row_complete", ns, self.pid_row)
            return cmd[:2], float(cmd[2])
        finally:
            _r47_end(self, _r47_call)

    def evaluate_motion_guard(self, *args):
        _r47_call = _r47_begin(self, "guard")
        _r47_bind(self, _r47_call, getattr(self, "pid_row", None))
        _r47_guard_dimensions(self, _r47_call, args)
        try:
            _r47_mark(self, _r47_call, "native_geometry_guard_before")
            # Default-OFF slow-guard window: per-thread CPU/scheduler and gc
            # deltas at both ends, plus C-watchdog stack blocks only if the
            # guard runs long. The guard call itself is unchanged.
            _guard_window = _r86_probe_guard_begin()
            try:
                result = self.record_native_guard(*args)
            finally:
                _r86_probe_guard_end(_guard_window, _r47_call)
            _r47_mark(self, _r47_call, "native_geometry_guard_after")
            _r47_mark(self, _r47_call, "retain_native_context_before")
            retain_native_context(self, args)
            _r47_mark(self, _r47_call, "retain_native_context_after")
            _r47_mark(self, _r47_call, "remember_native_guard_before")
            remember_native_guard(self)
            _r47_mark(self, _r47_call, "remember_native_guard_after")
            if self.pending_guard_row is not None:
                self.pending_guard_row.update(
                    cloud_array_file=self.guard_cloud_receipt["cloud_array_file"],
                    cloud_raw_payload_file=self.guard_cloud_receipt["cloud_raw_payload_file"],
                    cloud_archive_sequence=self.guard_cloud_receipt["cloud_archive_sequence"],
                )
                if self.pid_row:
                    self.pending_guard_row.update(
                        pid_sequence=self.pid_row["sequence"],
                        cascade_sequence=self.pid_row["sequence"],
                        pid_compute_ros_clock_ns=self.pid_row["compute_ros_clock_ns"],
                        pid_control_pose_stamp_ns=self.pid_row["control_pose_stamp_ns"],
                    )
            self.pid_guard_needs_evaluation = False
            self.control_clock_hold.guard_completed()
            _r47_mark(self, _r47_call, "guard_row_complete")
            return result
        finally:
            _r47_end(self, _r47_call)

    def observe_ordinary_terminal_protection(self, ns):
        owner = getattr(self, "terminal_parking_owner", None)
        if (
            owner is None
            or getattr(self, "terminal_parking_request", None) != self.request_id
            or owner.ready_stamp is None
        ):
            return
        reason = owner.failure
        # Completed normal shutdown is a distinct lifecycle edge. Existing
        # physical/control failure remains recorded even after a formal receipt.
        completed = False
        try:
            final = json.loads((self.run / "mission46_status.json").read_text())
            completed = (
                final.get("run_id") == self.run.name
                and final.get("stage") == "completed"
                and final.get("final_parking_verified") is True
            )
        except (OSError, ValueError, TypeError):
            pass
        if reason is None and not completed:
            if hasattr(self, "_known_generation_ready") and not self._known_generation_ready():
                reason = "known_scene_source_or_epoch_protection"
            elif self.obstacle_hold:
                reason = "dynamic_obstacle_protection"
            elif getattr(self, "tilt_hold", False):
                reason = "tilt_protection"
            elif self.state == "failed":
                reason = "terminal_controller_failure"
        if reason is not None:
            from embodied_navigation.common.observation import api as _observer_api

            diagnostic_enabled = _observer_api("diagnostic_observation").enabled
            diagnostic_record = _observer_api("diagnostic_observation").record
            observed_business = False
            if diagnostic_enabled(self.profile):
                from embodied_navigation.control.parking.ordinary_parking import (
                    diagnostic_business_failure,
                )

                observed_business = (
                    diagnostic_business_failure(reason)
                    and owner.diagnostic_failure_is_observed(
                        dict(run=self.run, profile=diagnostic_profile_view(self.profile))
                    )
                    and getattr(self, "ordinary_terminal_failure_latched", None) is None
                )
            if observed_business:
                now = time.monotonic()
                source_ready = False
                try:
                    from embodied_navigation.robot.unitree.policy.lower_loop_ack import (
                        input_binding,
                    )

                    ack = self.get_ack()
                    input_binding(ack)
                    source_ready = bool(
                        self._known_generation_ready()
                        and not self.obstacle_hold
                        and not getattr(self, "tilt_hold", False)
                        and self.state != "failed"
                        and self.raw_imu.fresh(now, ns / 1e9)
                        and all(
                            0 <= now - stamp < 0.3
                            for stamp in (
                                self.pose_updated,
                                self.cloud_updated,
                                self.bridge_updated,
                            )
                        )
                        and self.bridge_safety.get("state") == "ready"
                        and type(ack["stamp_ns"]) is int
                        and 0 <= ns - ack["stamp_ns"] < 300_000_000
                        and ack["stamp_ns"] <= self.pose_stamp
                        and 0 <= now - ack["received_wall_ns"] / 1e9 < 0.3
                    )
                except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
                    source_ready = False
                if source_ready:
                    if (
                        diagnostic_record(
                            self.run,
                            self.profile,
                            "ordinary_terminal_business_failure_latch",
                            reason,
                            dict(
                                control_stamp_ns=int(ns),
                                request_id=self.request_id,
                                original_business_failure_retained=True,
                                original_owner_failure=owner.failure,
                                actual_source_and_ACK_fresh=True,
                                normal_control_and_zero_phases_unchanged=True,
                                formal_parking_pass=False,
                            ),
                        )
                        is not True
                    ):
                        raise RuntimeError("ordinary_terminal_business_observation_not_recorded")
                    return
                reason = "actual_source_or_ACK_protection_during_observed_parking_business_failure"
            owner.guard(reason)
            if getattr(self, "ordinary_terminal_failure_latched", None) is None:
                self.ordinary_terminal_failure_latched = dict(
                    request_id=self.request_id,
                    reason=owner.failure,
                    clock_ns=ns,
                    ready_pose_stamp_ns=owner.ready_stamp,
                    ordinary_parking=owner.evidence(),
                    navigation_ground_truth_used=False,
                )
                self.evidence.append(
                    self.run / "ordinary_terminal_failure_history.jsonl",
                    self.ordinary_terminal_failure_latched,
                )
            self.state = "failed"
            self.message = "普通停车失败锁定：" + str(owner.failure)

    @_callback_cost_observe("control")
    @_r96_cpu.observe("embodied_navigation.navigation.pid_navigation")
    def control(self, now):
        failed = getattr(self, "ordinary_terminal_failure_latched", None)
        if failed is not None and failed.get("request_id") == self.request_id:
            self.state = "failed"
            self.publish_command()
            return
        gate = self.heading_gate
        clock_ns = self.get_clock().now().nanoseconds
        if getattr(gate, "integral_unload_config", None) is not None:
            gate.check_integral_unload_deadline(clock_ns)
        if (
            getattr(gate, "handoff_active", False)
            and gate.handoff_deadline_s is not None
            and clock_ns / 1e9 > gate.handoff_deadline_s
        ):
            gate.fail_handoff("active_handoff_deadline_exceeded", clock_ns / 1e9)
        if getattr(gate, "handoff_failure", None):
            self.state = "failed"
            self.message = "主动转向交接失败：" + str(gate.handoff_failure)
            self.publish_command()
            return
        try:
            return self._control_candidate(now)
        except Exception as error:
            # A reference/handoff error must stop this tick, not escape the
            # executor and rely only on the 300 ms downstream watchdog.
            self.state = "failed"
            self.message = "V33控制参考异常，停车：" + type(error).__name__ + ": " + str(error)
            self.reference_replan_pending = False
            self.publish_command()
            self.evidence.append(
                self.run / "navigation_controller_errors.jsonl",
                dict(
                    error_type=type(error).__name__,
                    error=str(error),
                    command_after_stop=list(self.command),
                    control_stamp_ns=self.get_clock().now().nanoseconds,
                    source_pose_stamp_ns=int(self.pose_stamp),
                    navigation_ground_truth_used=False,
                ),
            )

    @_r96_cpu.observe("embodied_navigation.control.candidate")
    def _control_candidate(self, now):
        self.poll_corridor(now)
        # The wall timer can run twice while real ROS /clock is unchanged.
        # Before any PI, heading, arrival or geometry operation, emit exact
        # zero and leave original source receipts untouched. On clock advance,
        # a new actual geometry check is mandatory before any nonzero command.
        if self.control_clock_hold.before_control(
            self, self.get_clock().now().nanoseconds, round(now * 1e9)
        ):
            return
        if _r105_terminal.control(self, now):
            return
        commit_pending_path(self)
        self.request_admission_replan_if_due(now)
        if self.path_admission_hold:
            if self.interrupt_active_handoff("active_handoff_path_admission_hold"):
                self.publish_command()
                return
            return self.control_admission_hold(now)
        if self.profile.get("mission46_required"):
            from embodied_navigation.navigation.mission.mission46_guard import mission_hold

            hold, reason = mission_hold(
                self.run,
                self.request_id,
                self.region_arrivals,
                self.goals_definition_sha256,
                self.get_clock().now().nanoseconds,
                now,
            )
            if hold:
                if self.interrupt_active_handoff("active_handoff_mission_hold"):
                    self.publish_command()
                    return
                self.message = "Teacher46 transition: " + reason
                # A reached phase endpoint retains the existing feedback hold.
                # A connector pause is a zero VELOCITY request to the live
                # Teacher, never a zero action or a second joint controller.
                if self.state == "succeeded" and self.final_bundle is not None:
                    return self.control_parking(now)
                self.apply_tilt_guard(now, self.get_clock().now().nanoseconds / 1e9)
                self.publish_command()
                return
        self.continuous_release_target()
        if self.state == "succeeded" and self.final_bundle is not None:
            return self.control_parking(now)
        if (
            self.state == "running"
            and self.samples is not None
            and self.waypoint_index == len(self.waypoints) - 1
            and self.segment_start is not None
        ):
            self.ensure_cascade()
            self.final_bundle = self.final_bundle_reference()
        return super().control(now)

    def control_parking(self, now):
        ns = self.get_clock().now().nanoseconds
        bundle = self.final_bundle
        index = self.waypoint_index
        self.waypoint_index = bundle["index"]
        self.active_reference_stamp = bundle["reference_stamp"]
        self.active_trajectory_id = bundle["trajectory_id"]
        self.trajectory_archive_reference = bundle["array"]
        self.path_receipt = bundle["path_receipt"]
        self.segment_start = bundle["start"]
        goal = self.waypoints[self.waypoint_index]
        if ordinary_parking_enabled(self.profile):
            self.cascade.begin_ordinary_terminal(
                int(ns), round(float(self.goals[self.waypoint_index].timeout_sim_s) * 1e9)
            )
            self.terminal_parking_owner = self.cascade.ordinary_parking
            self.terminal_parking_request = self.request_id
        self.steering = dict(
            direction=(goal - self.pose)[:2].tolist(),
            heading=self.cascade.goal_yaw,
            heading_error=0.0,
            exhausted=False,
        )
        _parking_sensor_row = None
        _parking_stale = None
        try:
            _parking_sensor_row = _sensor_hold_begin(self, "ordinary_parking", now, ns)
            stale = (
                now - self.pose_updated > 0.3
                or now - self.cloud_updated > 0.3
                or self.paired_imu is None
            )
            _parking_stale = stale
            _parking_tilt = self.apply_tilt_guard(now, ns / 1e9)
            _sensor_parking_protection(self, _parking_sensor_row, _parking_tilt)
            if _parking_tilt or stale or self.obstacle_hold:
                self.select_pid_velocity(np.zeros(2), 0.0, goal, False, mode_override="hold")
                self.publish_command()
                return
            velocity, yawrate = self.select_pid_velocity(
                np.zeros(2), 0.0, goal, True, mode_override="capture"
            )
            result = self.evaluate_motion_guard(
                self.cloud,
                self.pose,
                goal,
                self.steering["direction"],
                self.execution_guard_route(bundle["start"], goal),
            )
            if result[0]:
                self.message = "终点实际点云保护停车"
                self.publish_command()  # Stop immediately and archive the guarded candidate.
                self.select_pid_velocity(np.zeros(2), 0.0, goal, False, mode_override="hold")
                self.publish_command()
            else:
                self.message = "实际SLAM固定终点主动捕获/位置朝向保持"
                self.publish_command(velocity, yawrate)
            if self.pending_guard_row:
                self.pending_guard_row.update(
                    command_after_control=list(self.command),
                    zero_requested_after_control=not any(self.command),
                    state_after_control=self.state,
                    after_control_ros_clock_ns=self.get_clock().now().nanoseconds,
                    obstacle_hold_after=self.obstacle_hold,
                    evidence_queue_error=self.evidence.error,
                )
                self.evidence.append(
                    self.run / "navigation_guard_history.jsonl", self.pending_guard_row
                )
                self.pending_guard_row = None
        finally:
            self.waypoint_index = index
            _sensor_hold_end(self, _parking_sensor_row, _parking_stale)

    def publish_command(self, velocity=None, yaw_rate=0.0, *, duplicate_ordinary_hold=None):
        handoff = getattr(self, "_bridge_safety_handoff", None)
        reservation = False
        if velocity is not None and handoff is not None:
            reservation = handoff.reserve(self)
            if not reservation:
                return handoff.cancel_publication(self)
        try:
            join = getattr(self, "_r104_parking_causal_join", None)
            prior_join_source = None if join is None else join.published_source
            # Any unsuccessful publication/source replacement invalidates the cache.
            # A successful original duplicate-preservation transaction may restore it.
            if join is not None:
                join.published(None)
            _r47_call = _r47_begin(self, "publication", stopped=velocity is None)
        except BaseException:
            if reservation:
                handoff.end_reservation(self)
            raise
        try:
            from geometry_msgs.msg import Twist
            from std_msgs.msg import Bool

            # PUBLICATION_OBSERVER_BEGIN entry_snapshot
            from embodied_navigation.control.publication_ledger import PublicationLedger

            publication_before = PublicationLedger.begin(self, velocity is None)
            _r47_publication_anchor(self, _r47_call, publication_before)
            # PUBLICATION_OBSERVER_END entry_snapshot
            msg = Twist()
            ns = self.get_clock().now().nanoseconds
            if velocity is None:
                self.observe_ordinary_terminal_protection(ns)
            self.last_publication_ros_clock_ns = ns
            if velocity is not None:
                output = np.r_[velocity, yaw_rate]
                if self.pid_prepared_output is None or not np.allclose(
                    output, self.pid_prepared_output, rtol=0, atol=1e-12
                ):
                    raise RuntimeError("Cascade command changed after native guard")
                msg.linear.x, msg.linear.y, msg.angular.z = map(float, output)
            elif hasattr(self, "pid"):
                self.pid.reset("exact_zero_stop_or_protection")
                self.pid_prepared_output = None
            self.command = [msg.linear.x, msg.linear.y, msg.angular.z]
            self.last_command_time = ns / 1e9
            gate = self.heading_gate
            if getattr(gate, "handoff_active", False) and velocity is None:
                now = time.monotonic()
                protected = (
                    self.state != "running"
                    or self.obstacle_hold
                    or self.tilt_hold
                    or getattr(self, "reference_consistency_hold", False)
                    or self.samples is None
                    or getattr(self, "path_admission_hold", False)
                    or now - self.pose_updated > self.pose_timeout
                    or now - self.cloud_updated > self.cloud_timeout
                    or not self.raw_imu.fresh(now, ns / 1e9)
                    or now - self.bridge_updated > 0.30
                    or self.bridge_safety.get("state") != "ready"
                )
                if protected or getattr(gate, "settle_control_stage", None) != "authoritative_zero":
                    gate.fail_handoff("explicit_zero_protection_or_stop", ns / 1e9)
                    self.state = "failed"
                    self.message = "主动转向交接失败：" + str(gate.handoff_failure)
            note_final_zero = (
                velocity is None
                and not any(self.command)
                and getattr(gate, "settle_control_stage", None) == "authoritative_zero"
                and getattr(gate, "handoff_active", False)
                and not getattr(gate, "handoff_failure", None)
            )
            row = getattr(self, "pid_row", None)
            _r47_bind(self, _r47_call, row)
            if row is not None:
                row.update(
                    publish_ros_clock_ns=ns,
                    command_after_slew=list(self.command),
                    state=self.state,
                    obstacle_hold=self.obstacle_hold,
                    alignment_hold=self.alignment_hold,
                    stopped=velocity is None,
                    reset_count_after_publish=self.pid.resets,
                    queue_error=self.evidence.error,
                )
                f = row["feedback"]
                i = row["imu"]
                source = dict(
                    schema="actual_SLAM_SCAN_cascade_command_source/v1",
                    cascade_sequence=row["sequence"],
                    source_pose_stamp_ns=row["source_pose_stamp_ns"],
                    paired_imu_stamp_ns=row["paired_imu_stamp_ns"],
                    control_stamp_ns=row["control_stamp_ns"],
                    command_after_slew=list(self.command),
                    navigation_ground_truth_used=False,
                    source_pose_received_wall_ns=None if f is None else f["received_wall_ns"],
                    source_imu_received_wall_ns=None if i is None else i["received_wall_ns"],
                    source_pose_received_monotonic_wall=None
                    if f is None
                    else f["received_wall_ns"] / 1e9,
                    paired_imu_received_monotonic_wall=None
                    if i is None
                    else i["received_wall_ns"] / 1e9,
                    request_id=row["request_id"],
                    trajectory_id=row["trajectory_id"],
                    waypoint_index=row["waypoint_index"],
                    goal_sha256=row["cascade"]["fixed_goal_sha256"],
                    path_sha256=row["cascade"]["path_sha256"],
                    cascade_mode=row["cascade"]["mode"],
                    parking_hold_declared_pose_stamp_ns=row["cascade"][
                        "parking_hold_declared_pose_stamp_ns"
                    ],
                    heading_transition_stage=getattr(gate, "settle_control_stage", None),
                    heading_transition_context=copy.deepcopy(
                        getattr(gate, "transition_context", {})
                    ),
                    heading_zero_handoff_count=getattr(gate, "zero_handoff_count", None),
                    heading_last_zero_handoff=copy.deepcopy(
                        getattr(gate, "last_zero_handoff", None)
                    ),
                )
                if getattr(self.cascade, "terminal_reference", None) is not None:
                    from embodied_navigation.control.parking.terminal_reference import fields

                    source.update(
                        fields(self.cascade.terminal_reference),
                        schema="actual_SLAM_terminal_fixed_goal_command_source/v1",
                        path_geometry_present=False,
                    )
                if (
                    getattr(gate, "integral_unload_config", None) is not None
                    and row["cascade"]["mode"] == "brake"
                ):
                    source["pre_zero_integral_unload"] = copy.deepcopy(gate.integral_unload_event)
                source["ordinary_parking_stage"] = row["cascade"].get("ordinary_parking_stage")
                source["ordinary_terminal_zero_event"] = None
                if (
                    velocity is not None
                    and row["cascade"]["mode"] == "ordinary_zero_hold"
                    and not row["cascade"]["protection_active"]
                ):
                    ordinary = self.cascade.ordinary_parking
                    if self.cascade.ordinary_zero_event is None:
                        self.cascade.ordinary_zero_event = make_event(
                            self.run.name,
                            self.request_id,
                            source["goal_sha256"],
                            source["path_sha256"],
                            row["sequence"],
                            ordinary.ready_stamp,
                            ordinary.handoff_clock,
                            row["sequence"],
                            terminal_reference=getattr(self.cascade, "terminal_reference", None),
                        )
                    marker = self.cascade.ordinary_zero_event
                    source.update(
                        ordinary_terminal_zero_event=dict(marker),
                        ordinary_zero_event_id=marker["event_id"],
                        ordinary_zero_event_sequence=marker["event_sequence"],
                    )
                    if ordinary.first_zero_publication_clock is None:
                        ordinary.first_zero_publication_clock = ns
                elif velocity is None:
                    source["ordinary_parking_stage"] = "revoked"
                tmp = self.run / "cascade_command_source.tmp"
                _r47_mark(self, _r47_call, "source_tmp_write_before")
                tmp.write_text(json.dumps(source, allow_nan=False) + "\n")
                _r47_mark(self, _r47_call, "source_tmp_write_after")
                _r47_mark(self, _r47_call, "source_replace_before")
                tmp.replace(self.run / "cascade_command_source.json")
                _r47_mark(self, _r47_call, "source_replace_after")
                if _r118_source_age.ENABLED:
                    _r118_source_age.controller(self, "source.atomic_replace", ns, row)
                _r47_mark(self, _r47_call, "PID_evidence_append_before")
                _append_pid_row_evidence(self, row)
                _r47_mark(self, _r47_call, "PID_evidence_append_after")
                self.pid_row = None
                self._pid_row_receipt_snapshot = None
            source_preservation = None
            if velocity is None and row is None:
                if duplicate_ordinary_hold is not None:
                    from embodied_navigation.control.safety.duplicate_source import (
                        preserve_existing_source,
                        observe_unchanged_after_publish,
                    )

                    source_preservation = preserve_existing_source(
                        self,
                        duplicate_ordinary_hold,
                        ns,
                        command_before=publication_before["command_before"],
                    )
                if source_preservation is None or not source_preservation["allowed"]:
                    revoked = dict(
                        schema="cascade_command_explicit_zero_revocation/v1",
                        navigation_ground_truth_used=False,
                        command_after_slew=[0.0, 0.0, 0.0],
                        ordinary_terminal_zero_event=None,
                        ordinary_parking_stage="revoked",
                        request_id=self.request_id,
                        publish_ros_clock_ns=ns,
                    )
                    tmp = self.run / "cascade_command_source.tmp"
                    _r47_mark(self, _r47_call, "revoked_source_tmp_write_before")
                    tmp.write_text(json.dumps(revoked, allow_nan=False) + "\n")
                    _r47_mark(self, _r47_call, "revoked_source_tmp_write_after")
                    _r47_mark(self, _r47_call, "revoked_source_replace_before")
                    tmp.replace(self.run / "cascade_command_source.json")
                    _r47_mark(self, _r47_call, "revoked_source_replace_after")
                    if source_preservation is not None:
                        source_preservation.update(
                            source_file_written=True, preservation_performed=False
                        )
                else:
                    source_preservation["preservation_performed"] = True
            # PUBLICATION_OBSERVER_BEGIN actual_publish_start
            publication_started_wall_ns = PublicationLedger.publication_started()
            # PUBLICATION_OBSERVER_END actual_publish_start
            _r47_mark(self, _r47_call, "DDS_publish_call_before")
            self.cmd_pub.publish(msg)
            join = getattr(self, "_r104_parking_causal_join", None)
            if join is not None:
                if row is not None:
                    join.published(source)
                elif source_preservation is not None and source_preservation["allowed"]:
                    join.published(prior_join_source)
            _r47_mark(self, _r47_call, "DDS_publish_return")
            if _r118_source_age.ENABLED:
                _r118_source_age.controller(self, "command.DDS_return", ns, row)
            self.counts["commands"] += 1
            if getattr(gate, "quiet_target_enabled", False):
                record = None
                core = {} if row is None else row.get("cascade", {})
                ref = {} if row is None else row.get("heading_gate_reference", {})
                f = None if row is None else row.get("feedback")
                i = None if row is None else row.get("imu")
                binding = gate._quiet_binding(
                    gate.transition_context,
                    gate.transition_purpose,
                    self.handoff_scope_sha256,
                    self.cascade_path_id,
                    None if self.cascade is None else self.cascade.path_sha,
                    None if self.cascade is None else self.cascade.goal_sha,
                )
                normal = bool(
                    row is not None
                    and isinstance(f, dict)
                    and isinstance(i, dict)
                    and "received_wall_ns" in f
                    and "received_wall_ns" in i
                    and velocity is not None
                    and self.state == "running"
                    and gate.controlled_braking
                    and gate.handoff_active
                    and core.get("mode") == "brake"
                    and core.get("controller_updated") is True
                    and core.get("protection_active") is False
                    and ref.get("phase") == "settle"
                    and ref.get("controlled_braking") is True
                    and ref.get("settle_control_stage") == "controlled_braking"
                    and row.get("quiet_handoff_binding") == binding
                    and row.get("request_id") == self.request_id
                    and row.get("waypoint_index") == self.waypoint_index
                    and not self.obstacle_hold
                    and not self.tilt_hold
                    and not getattr(self, "reference_consistency_hold", False)
                    and not getattr(self, "path_admission_hold", False)
                    and self.samples is not None
                    and not self.control_clock_hold.holding
                    and not self.control_clock_hold.guard_required
                    and not self.pid_guard_needs_evaluation
                    and self.evidence.error is None
                    and not row.get("launch_output_constraint", {}).get("applied", False)
                )
                if normal:
                    record = dict(
                        sequence=self.counts["commands"],
                        publish_clock_ns=self.get_clock().now().nanoseconds,
                        publish_wall_ns=time.monotonic_ns(),
                        control_clock_ns=row["compute_ros_clock_ns"],
                        control_wall_ns=core["compute_wall_ns"],
                        source_pose_stamp_ns=row["source_pose_stamp_ns"],
                        paired_imu_stamp_ns=row["paired_imu_stamp_ns"],
                        source_pose_received_wall_ns=f["received_wall_ns"],
                        source_imu_received_wall_ns=i["received_wall_ns"],
                        command_body=list(self.command),
                        normal_braking=True,
                        **binding,
                    )
                if record is not None and getattr(gate, "integral_unload_config", None) is not None:
                    pi = core.get("velocity_PI", {})
                    record["pre_zero_integral_unload"] = copy.deepcopy(
                        pi.get("pre_zero_integral_unload")
                    )
                    record["yaw_integral_state"] = pi.get("integral_state", [None] * 3)[2]
                    record["integral_dt_s"] = pi.get("integral_dt_s")
                gate.note_quiet_publication(record)
            if note_final_zero:
                gate.note_zero_publication(self.get_clock().now().nanoseconds)
            _r47_mark(self, _r47_call, "quiet_and_zero_note_after")
            # PUBLICATION_OBSERVER_BEGIN actual_publish_completed
            PublicationLedger.completed(self, publication_before, ns, publication_started_wall_ns)
            _r47_publication_anchor(self, _r47_call, completed=True)
            _r67_attach_known_frame_failure(self, _r47_call, publication_before)
            # PUBLICATION_OBSERVER_END actual_publish_completed
            self.freeze_pub.publish(
                Bool(
                    data=bool(
                        self.state != "running"
                        or self.obstacle_hold
                        or np.linalg.norm(self.command[:2]) < 0.015
                    )
                )
            )
            if source_preservation is not None and source_preservation["allowed"]:
                observe_unchanged_after_publish(self, source_preservation)
            return source_preservation
        finally:
            if reservation:
                handoff.end_reservation(self)
            _r47_end(self, _r47_call)
