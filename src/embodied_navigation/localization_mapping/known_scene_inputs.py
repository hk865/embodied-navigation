"""Joined pose/cloud/metadata ingestion and controlled map-frame handoff.

Body pose, registered cloud and metadata are joined by source stamp and checked
before dispatch to the controller callbacks. The commit exposes the accepted
bundle/generation to control; this consumer does not atomically publish ROS topics
or turn simulation truth into a SLAM source.
"""

# Repeated imports bind the same observation API function. Named hook calls
# select the normal runtime shared disabled observation port.
from embodied_navigation.common.integration_limits import KNOWN_BUNDLE_FRESHNESS_S
from embodied_navigation.localization_mapping.known_scene_hold_guard import (
    observe_zero_hold,
    _frame,
)
import json, time, math, copy, hashlib
from embodied_navigation.common.input_freshness import enabled as minimal_baseline_enabled
from embodied_navigation.common.observation import api as _observer_api

_sensor_hold_begin = _observer_api("r74_sensor_hold_witness").begin
_sensor_hold_end = _observer_api("r74_sensor_hold_witness").end
from collections import OrderedDict
from embodied_navigation.common.observation import api as _observer_api

_corrected_create = _observer_api("r74_corrected_delivery_witness").create
_corrected_entry = _observer_api("r74_corrected_delivery_witness").consumer_entry
_corrected_arrival = _observer_api("r74_corrected_delivery_witness").consumer_arrival
_corrected_call = _observer_api("r74_corrected_delivery_witness").consumer_call
_corrected_finish = _observer_api("r74_corrected_delivery_witness").consumer_finish
from std_msgs.msg import String
from embodied_navigation.perception.sensors.metadata_qos import (
    subscription_depth as _corrected_metadata_depth,
)
from embodied_navigation.common.observation import api as _observer_api

_callback_cost_create = _observer_api("r74_callback_cost_witness").create
_callback_cost_observe = _observer_api("r74_callback_cost_witness").observe
from embodied_navigation.localization_mapping.known_scene_adapter import stamp_ns
from embodied_navigation.localization_mapping.point_provenance import verify_corrected
from embodied_navigation.common.observation import api as _observer_api

diagnostic_enabled = _observer_api("diagnostic_observation").enabled
diagnostic_record = _observer_api("diagnostic_observation").record
from embodied_navigation.perception.sensors.feedback_mailbox import (
    enabled as _r86_enabled,
    ros_mailbox as _r86_mailbox,
    control_scope as _r86_control_scope,
    check_negative_parking as _r86_negative_parking,
    expired_prefix_enabled as _r86_expired_enabled,
    expired_prefix_length as _r86_expired_length,
    expired_prefix_timeout_ns as _r86_expired_timeout_ns,
)
from embodied_navigation.common.observation import api as _observer_api

_r87_diag = _observer_api("r87_diagnostic_probe")
from embodied_navigation.common.observation import api as _observer_api

_r96_cpu = _observer_api("r96_control_cpu_probe")
from embodied_navigation.common.observation import api as _observer_api

_r102_progress = _observer_api("r102_progress_probe")
from embodied_navigation.perception.sensors.ready_commit import (
    selected as _r92_selected,
    ReadyCommit as _R92ReadyCommit,
)
from embodied_navigation.common.observation import api as _observer_api

_r99_feedback = _observer_api("r99_clock_feedback")
from embodied_navigation.common.observation import api as _observer_api

_r99_clock_probe = _observer_api("r99_clock_probe")
from embodied_navigation.control.parking.causal_join import (
    PARKING_CAUSAL_JOIN_ENABLED,
    ParkingCausalJoin,
)
from embodied_navigation.control.parking.terminal_owner import TERMINAL_PARKING_OWNER_ENABLED
import embodied_navigation.control.parking.terminal_owner as _r105_terminal


class KnownSceneInputMixin:
    def __init__(self, *args, **kwargs):
        self.known_scene_enabled = True
        self.known_scene_inputs = None
        self.known_scene_status = None
        self.known_scene_transition = False
        self.known_scene_pending = OrderedDict()
        self.known_scene_callbacks = {}
        self.known_scene_reference_generation = None
        self.known_scene_scan_epoch = None
        self._r86_prepared_mailbox = None
        self._r86_committing_bundle = None
        self._r86_intake_group = None
        self._r86_scope = None
        self._r86_scope_since_ns = 0
        self._r92_enabled = _r92_selected()
        self._r92_ready_commit = None
        self._r99_policy = None
        self._r99_timer = None
        self._r99_waiting = False
        self._r99_timer_active = False
        self._r104_parking_causal_join = None
        super().__init__(*args, **kwargs)
        # S1 selection is read once from the frozen profile, like R86/R92/R99.
        self._r86_expired_prefix = _r86_expired_enabled(getattr(self, "profile", None))
        self._r74_corrected_delivery_witness = _corrected_create(self.run, self.profile, "consumer")
        self._r74_callback_cost_witness = _callback_cost_create(self.run, self.profile)
        options = self._r86_subscription_options()
        self.create_subscription(
            String,
            "/demo/navigation/localization_bundle",
            self.on_localization_bundle,
            _corrected_metadata_depth(self.profile),
            **options,
        )
        self.create_subscription(
            String, "/demo/navigation/localization_status", self.on_localization_status, 10
        )
        self.create_subscription(
            String, "/demo/navigation/scan_epoch_ready", self.on_scan_epoch_ready, 10
        )
        self._r86_update_scope()
        if _r99_feedback.selected(
            self.profile,
            pose_timeout_s=getattr(self, "pose_timeout", None),
            cloud_timeout_s=getattr(self, "cloud_timeout", None),
        ):
            from rclpy.clock import Clock, ClockType

            self._r99_policy = _r99_feedback.Policy()
            self._r99_timer = self.create_timer(
                0.05,
                self._r99_retry_feedback,
                clock=Clock(clock_type=ClockType.STEADY_TIME),
                autostart=False,
            )
        if PARKING_CAUSAL_JOIN_ENABLED:
            if (
                self._r86_prepared_mailbox is None
                or self._r86_expired_prefix
                or self._r99_policy is not None
            ):
                raise ValueError("R104 requires R86 with S1 and R99 OFF")
            self._r104_parking_causal_join = ParkingCausalJoin()
        _r102_progress.install(self)

    def _r86_subscription_options(self):
        if not _r86_enabled(self.profile):
            if self._r92_enabled:
                raise ValueError("Ready commit requires R86 prepared feedback")
            return {}
        if self._r86_prepared_mailbox is None:
            from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

            self._r86_prepared_mailbox = _r86_mailbox(verify_corrected)
            self._r86_intake_group = MutuallyExclusiveCallbackGroup()
            if self._r92_enabled:
                self._r92_ready_commit = _R92ReadyCommit(
                    self, self._r86_prepared_mailbox, self._r92_commit_ready
                )
        return dict(callback_group=self._r86_intake_group)

    @_r87_diag.observe("callback.ready_commit")
    def _r92_commit_ready(self):
        self._r86_commit_feedback()
        if self._r86_fail_closed():
            self.publish_command()
        elif self._r99_waiting:
            self._r99_zero_wait()

    def _r99_owner_identity(self):
        data = (
            self.run.name,
            getattr(self, "request_id", None),
            getattr(self, "waypoint_index", None),
            getattr(self, "goals_definition_sha256", None),
            getattr(self, "state", None),
        )
        return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).digest()

    def _r99_decision(self, head, clock, wall, *, observed_future=False):
        policy = self._r99_policy
        fields = (
            {}
            if head is None
            else dict(
                head_source_ns=head.source_ns,
                original_body_arrival_ns=head.arrivals_ns[0],
                scope_digest=self._r86_scope,
                token=head.token,
                head_scopes=head.scopes,
                owner_identity=self._r99_owner_identity(),
            )
        )
        if head is None:
            fields["head_source_ns"] = None
        needs_owner = (
            policy.source_ns is not None
            or observed_future
            or (head is not None and head.source_ns > clock)
        )
        decision = policy.peek_decision(
            clock_ns=clock,
            wall_ns=wall,
            observed_future=observed_future,
            owner_valid=not needs_owner
            or (self.state == "running" and self.request_id is not None),
            **fields,
        )
        if decision.action == "fault":
            self._r86_prepared_mailbox.fail(decision.reason)
            _r99_clock_probe.hard_fault(self, decision.reason, wall)
        return decision

    def _r99_set_waiting(self, waiting):
        if waiting and not self._r99_waiting:
            self._r99_zero_published = False
        self._r99_waiting = waiting
        if waiting and not self._r99_timer_active:
            self._r99_timer.reset()
            self._r99_timer_active = True
        elif not waiting and self._r99_timer_active:
            self._r99_timer.cancel()
            self._r99_timer_active = False

    def _r99_zero_wait(self):
        # Use the original explicit-zero protection, including PID reset,
        # handoff failure and the resulting scope change. Never forge scope.
        if any(self.command) or not getattr(self, "_r99_zero_published", False):
            self.publish_command()
            self._r99_zero_published = True
            if self.state == "failed":
                reason = "r99_original_zero_protection:" + self.message
                self._r86_prepared_mailbox.fail(reason)
                _r99_clock_probe.hard_fault(self, reason)
                self._r99_policy.clear("owner_invalidated")
                self._r99_set_waiting(False)

    def _r99_retry_feedback(self):
        if not self._r99_waiting:
            self._r99_set_waiting(False)
            return
        self._r86_commit_feedback()
        if self._r86_fail_closed():
            self.publish_command()
        elif self._r99_waiting:
            self._r99_zero_wait()

    def _r86_update_scope(self):
        mailbox = self._r86_prepared_mailbox
        if mailbox is None:
            return
        scope = _r86_control_scope(self)
        if scope != self._r86_scope:
            self._r86_scope = scope
            self._r86_scope_since_ns = self.get_clock().now().nanoseconds
        mailbox.set_scope(scope)

    def subscribe_slam_body(self, msg_type, topic, callback, qos_profile, *args, **kwargs):
        self.known_scene_callbacks["body"] = callback

        @_r87_diag.observe("callback.intake.body")
        def wrapped(msg):
            if self._r86_prepared_mailbox is not None:
                return self._r86_prepared_mailbox.receive(
                    "body", stamp_ns(msg), msg, time.monotonic_ns()
                )
            entry = _corrected_entry(self)
            return self._known_receive("body", stamp_ns(msg), msg, _witness_entry_wall_ns=entry)

        kwargs.update(self._r86_subscription_options())
        return super().subscribe_slam_body(msg_type, topic, wrapped, qos_profile, *args, **kwargs)

    def subscribe_registered_cloud(self, msg_type, topic, callback, qos_profile, *args, **kwargs):
        self.known_scene_callbacks["cloud"] = callback

        @_r87_diag.observe("callback.intake.cloud")
        def wrapped(msg):
            if self._r86_prepared_mailbox is not None:
                return self._r86_prepared_mailbox.receive(
                    "cloud", stamp_ns(msg), msg, time.monotonic_ns()
                )
            entry = _corrected_entry(self)
            return self._known_receive("cloud", stamp_ns(msg), msg, _witness_entry_wall_ns=entry)

        kwargs.update(self._r86_subscription_options())
        return super().subscribe_registered_cloud(
            msg_type, topic, wrapped, qos_profile, *args, **kwargs
        )

    @_r87_diag.observe("callback.intake.metadata")
    def on_localization_bundle(self, msg):
        entry = (
            time.monotonic_ns()
            if self._r86_prepared_mailbox is not None
            else _corrected_entry(self)
        )
        if self._r86_prepared_mailbox is not None and len(msg.data) > 4 * 1024 * 1024:
            self._r86_prepared_mailbox.fail("metadata_input_byte_bound")
            return
        try:
            with _r87_diag.span("metadata.json_loads"):
                d = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if (
            d.get("schema") != "known_scene_navigation_bundle/v1"
            or d.get("run_id") != self.run.name
            or type(d.get("source_ns")) is not int
            or type(d.get("generation")) is not int
            or d.get("robot_truth_pose_used") is not False
            or not d.get("exact_body_lidar_registered_stamp")
        ):
            return
        if self._r86_prepared_mailbox is not None:
            return self._r86_prepared_mailbox.receive("metadata", d["source_ns"], d, entry)
        self._known_receive("metadata", d["source_ns"], d, _witness_entry_wall_ns=entry)

    @_r87_diag.observe("embodied_navigation.control.commit_feedback")
    @_r96_cpu.observe("embodied_navigation.control.commit_feedback")
    def _r86_commit_feedback(self):
        handoff = getattr(self, "_bridge_safety_handoff", None)
        if handoff is not None:
            handoff.service(self)
        mailbox = self._r86_prepared_mailbox
        if mailbox is None:
            return
        self._r86_update_scope()
        # One fixed batch; newly arriving frames wait for the next control entry.
        policy = self._r99_policy
        deferred = None
        join = None
        if policy is None:
            join = getattr(self, "_r104_parking_causal_join", None)
            batch = None if join is None else join.prepare(self, mailbox)
            if batch is None:
                batch = mailbox.take_batch()
        else:
            clock = self.get_clock().now().nanoseconds
            decision = self._r99_decision(mailbox.peek_ready(), clock, time.monotonic_ns())
            if decision.action == "fault":
                self._r99_set_waiting(False)
                return
            batch = mailbox.take_batch(clock)
            head = mailbox.peek_ready()
            if head is not None and head.source_ns > clock:
                deferred = head
                _r99_clock_probe.future_wait(self, head.source_ns, clock, time.monotonic_ns())
                _r87_diag.mark(
                    "clock.future_suffix_observed",
                    head.source_ns,
                    entry_ros_clock_ns=clock,
                    original_body_arrival_ns=head.arrivals_ns[0],
                )
        # S1: retire only the clearly expired FIFO head before any business
        # callback. No receipt/stamp/token/scope advances for a retired member.
        retired_count = 0
        if getattr(self, "_r86_expired_prefix", False):
            try:
                retired_count = self._r86_retire_expired_prefix(batch)
            except Exception:
                # Misconfiguration stays fail-loud, but the fixed batch bytes
                # are still released exactly once before the error escapes.
                mailbox.release_batch(batch)
                raise
        for index, bundle in enumerate(batch):
            if mailbox.fault_reason():
                break
            if index < retired_count:
                continue
            mailbox._evidence(5, bundle=bundle, batch_id=mailbox.diagnostic_batch_id)
            _r87_diag.mark(
                "commit.member.enter",
                bundle.source_ns,
                batch_id=getattr(mailbox, "diagnostic_batch_id", None),
                index=index,
                prepared_ns=bundle.prepared_ns,
            )
            try:
                ns = bundle.source_ns
                if ns <= self.pose_stamp:
                    raise ValueError("commit_source_not_monotonic")
                prior = getattr(self, "known_scene_commit_token", None)
                if prior is not None and (
                    bundle.token[0] != prior[0] or bundle.token[1] <= prior[1]
                ):
                    raise ValueError("commit_token_not_monotonic")
                with _r87_diag.span("commit.decode.body", ns):
                    body = mailbox.decode("body", bundle.body)
                with _r87_diag.span("commit.decode.cloud", ns):
                    cloud = mailbox.decode("cloud", bundle.cloud)
                with _r87_diag.span("commit.decode.metadata", ns):
                    metadata = mailbox.decode("metadata", bundle.metadata)
                if policy is not None and policy.source_ns == ns:
                    decision = self._r99_decision(
                        bundle, self.get_clock().now().nanoseconds, time.monotonic_ns()
                    )
                    if decision.action != "ready":
                        raise ValueError("held_head_not_ready:" + decision.reason)
                self._r86_committing_bundle = bundle
                candidate_frame = (
                    metadata["generation"],
                    metadata.get("C_world_odom"),
                    metadata.get("T_navigation_world"),
                )
                authorized_frame = getattr(self, "_last_authorized_hold_frame", None)
                same_control_frame = authorized_frame is None or candidate_frame == authorized_frame
                if join is not None and ns in join.bindings:
                    current = self.known_scene_inputs
                    current_frame = (
                        current["generation"],
                        current.get("C_world_odom"),
                        current.get("T_navigation_world"),
                    )
                    if not same_control_frame or candidate_frame != current_frame:
                        raise ValueError("r104_control_frame_changed")
                self._r86_positive_stop_allowed = (
                    not (join is not None and join.waiting)
                    and ns > self._r86_scope_since_ns
                    and same_control_frame
                    and all(scope == self._r86_scope for scope in bundle.scopes)
                )
                with _r87_diag.span("commit.dispatch.body", ns):
                    self.known_scene_callbacks["body"](body)
                if self.pose_stamp != ns:
                    raise ValueError("body_not_accepted_at_commit")
                with _r87_diag.span("commit.dispatch.cloud", ns):
                    self.known_scene_callbacks["cloud"](cloud)
                if self.cloud_stamp != ns:
                    raise ValueError("cloud_not_accepted_at_commit")
                feedback = getattr(self, "feedback", None)
                if feedback is not None and feedback.get("stamp_ns") != ns:
                    raise ValueError("feedback_not_same_source")
                # Activate generation only here, after both original callbacks.
                self.known_scene_commit_token = bundle.token
                self.known_scene_inputs = {
                    **metadata,
                    "received_monotonic_wall": max(bundle.arrivals_ns) / 1e9,
                }
                pose10hz_intake = _r86_negative_parking(self) if same_control_frame else False
                mailbox.committed(bundle, time.monotonic_ns())
                if join is not None:
                    join.committed(bundle)
                if policy is not None and policy.source_ns == ns:
                    policy.clear("committed")
                self.evidence.append(
                    self.run / "known_scene_controller_inputs.jsonl",
                    dict(
                        source_ns=ns,
                        generation=metadata["generation"],
                        accepted_pose_stamp_ns=self.pose_stamp,
                        accepted_cloud_stamp_ns=self.cloud_stamp,
                        control_allowed=metadata["control_allowed"],
                        intake_arrivals_ns=bundle.arrivals_ns,
                        prepared_ns=bundle.prepared_ns,
                        committed_monotonic_wall_ns=time.monotonic_ns(),
                        positive_stop_scope_qualified=self._r86_positive_stop_allowed,
                        parking_pose10hz_source_intake=pose10hz_intake is True,
                        source_receipts_refreshed=False,
                        navigation_ground_truth_used=False,
                    ),
                )
            except Exception as error:
                if policy is not None and policy.source_ns == bundle.source_ns:
                    policy.clear("commit_rejected" if not policy.pending else "mailbox_fault")
                mailbox.fail("commit:" + type(error).__name__ + ":" + str(error))
                _r99_clock_probe.hard_fault(self, str(error))
                self.known_scene_inputs = None
                break
            finally:
                mailbox._evidence(
                    6,
                    bundle=bundle,
                    batch_id=mailbox.diagnostic_batch_id,
                    outcome=int(
                        self.pose_stamp == bundle.source_ns and self.cloud_stamp == bundle.source_ns
                    ),
                )
                _r87_diag.mark(
                    "commit.member.exit",
                    bundle.source_ns,
                    batch_id=getattr(mailbox, "diagnostic_batch_id", None),
                    index=index,
                    accepted_pose_stamp_ns=getattr(self, "pose_stamp", None),
                    accepted_cloud_stamp_ns=getattr(self, "cloud_stamp", None),
                )
                self._r86_committing_bundle = None
                self._r86_positive_stop_allowed = True
        mailbox.release_batch(batch)
        if join is not None:
            join.end_batch()
        if policy is not None:
            if mailbox.fault_reason():
                policy.clear("mailbox_fault")
                self._r99_set_waiting(False)
            elif deferred is not None:
                # The entry clock defines this fixed prefix. Even if /clock
                # advances during decode, retry the held suffix at a new entry.
                decision = self._r99_decision(
                    deferred,
                    self.get_clock().now().nanoseconds,
                    time.monotonic_ns(),
                    observed_future=True,
                )
                self._r99_set_waiting(decision.action in ("wait", "ready"))
            else:
                self._r99_set_waiting(False)

    def _r86_retire_expired_prefix(self, batch):
        """Retire the expired head of one fixed batch; never refresh receipts.

        Only the original integer decision clock and the same 300 ms freshness
        expression are used. A future head and every member behind the first
        non-expired member stay in the batch in their original order.
        """
        mailbox = self._r86_prepared_mailbox
        if mailbox is None or not batch:
            return 0
        decision_clock = self.get_clock().now().nanoseconds
        timeout_ns = _r86_expired_timeout_ns(getattr(self, "pose_timeout", None))
        count = _r86_expired_length(batch, decision_clock, timeout_ns)
        if not count:
            return 0
        retired = mailbox.retire_expired(batch, count, decision_clock, timeout_ns)
        parking = self._r86_retired_prefix_parking(retired)
        self.evidence.append(
            self.run / "known_scene_expired_prefix.jsonl",
            dict(
                schema="r86_expired_prefix/v1",
                event="retired_expired",
                retired_count=count,
                retired_bytes=sum(frame.size for frame in retired),
                decision_clock_ns=decision_clock,
                pose_timeout_ns=timeout_ns,
                first_retired_source_ns=retired[0].source_ns,
                last_retired_source_ns=retired[-1].source_ns,
                batch_count=len(batch),
                remaining_count=len(batch) - count,
                arrivals_ns=[list(frame.arrivals_ns) for frame in retired],
                tokens=[list(frame.token) for frame in retired],
                parking_action=parking,
                known_scene_commit_token_advanced=False,
                pose_stamp_advanced=False,
                source_receipts_refreshed=False,
                navigation_ground_truth_used=False,
            ),
        )
        _r87_diag.mark(
            "commit.expired_prefix_retired",
            retired[-1].source_ns,
            retired_count=count,
            decision_clock_ns=decision_clock,
            timeout_ns=timeout_ns,
            first_retired_source_ns=retired[0].source_ns,
        )
        return count

    def _r86_retired_prefix_parking(self, retired):
        """A retired stale frame must never buy a positive parking dwell.

        Before the qualified zero hold the original capture dwell is cleared
        (so the 600 ms continuity restarts). After ready_stamp the original
        no-retry guard owns the failure, so the stale gap fails the hold instead
        of being washed away by a later fresh packet. No dwell, T0 or zero ACK
        is invented here and the original negative events stay untouched.
        """
        core = getattr(self, "cascade", None)
        ordinary = getattr(core, "ordinary_parking", None)
        if ordinary is None or not getattr(ordinary, "enabled", False):
            return None
        reason = "expired_prefix_retired"
        if getattr(ordinary, "ready_stamp", None) is not None:
            protect = getattr(core, "_protect", None)
            if callable(protect):
                protect(reason)
            else:
                ordinary.guard(reason)
                if hasattr(core, "failure_latched"):
                    core.failure_latched = core.failure_latched or ordinary.failure
            return "zero_hold_invalidated"
        ordinary.guard(reason)
        return "capture_dwell_invalidated"

    def _r86_fail_closed(self):
        mailbox = self._r86_prepared_mailbox
        fault = None if mailbox is None else mailbox.fault_reason()
        if fault is None:
            return False
        if self._r99_policy is not None:
            self._r99_set_waiting(False)
        self.state = "failed"
        self.message = "R86 corrected input integrity: " + fault
        self.known_scene_inputs = None
        core = getattr(self, "cascade", None)
        if core is not None:
            core._protect(self.message, fatal=True)
        self.pid_row = None
        self._pid_row_receipt_snapshot = None
        self.pid_prepared_output = None
        return True

    @_r87_diag.observe("callback.control_tick")
    @_r96_cpu.observe("embodied_navigation.control.tick")
    def tick(self):
        self._r86_commit_feedback()
        if self._r86_fail_closed():
            self.publish_command()
            now = time.monotonic()
            if now - self.last_status >= 0.25:
                self.publish_status()
                self.last_status = now
            return
        if self._r99_waiting:
            self._r99_zero_wait()
            return
        join = getattr(self, "_r104_parking_causal_join", None)
        if join is not None and join.waiting is not None:
            return
        asynchronous = getattr(self, "_r88_admission", None)
        if asynchronous is not None:
            try:
                asynchronous.poll()
            except Exception as error:
                # New asynchronous work must not bypass the original zero
                # publication on an exceptional timer exit. Re-raise so the
                # existing executor/owner cleanup still reports the failure.
                self.state = "failed"
                self.message = "R88 async admission: " + type(error).__name__ + ":" + str(error)
                self.publish_command()
                raise
        # Original control now is sampled after the fixed batch, not before it.
        return super().tick()

    def publish_command(self, velocity=None, yaw_rate=0.0, **kwargs):
        if self._r86_fail_closed():
            velocity = None
            yaw_rate = 0.0
            if "duplicate_ordinary_hold" in kwargs:
                kwargs["duplicate_ordinary_hold"] = None
        try:
            return super().publish_command(velocity, yaw_rate, **kwargs)
        finally:
            self._r86_update_scope()

    def destroy_node(self):
        join = getattr(self, "_r104_parking_causal_join", None)
        if join is not None:
            join.finish(self)
        _r102_progress.finish(self)
        if self._r99_policy is not None:
            self._r99_set_waiting(False)
            self._r99_policy.clear("shutdown")
            (self.run / "r99_clock_feedback.json").write_text(
                json.dumps(self._r99_policy.snapshot(), allow_nan=False) + "\n"
            )
        ready = self._r92_ready_commit
        if ready is not None:
            ready.close()
            try:
                (self.run / "r92_ready_commit_statistics.json").write_text(
                    json.dumps(ready.snapshot(), allow_nan=False) + "\n"
                )
            except Exception:
                self._r92_statistics_write_failed = True
        mailbox = self._r86_prepared_mailbox
        if mailbox is not None:
            from embodied_navigation.common.observation import api as _observer_api

            r118_feedback_evidence = _observer_api("r118_feedback_evidence")
            try:
                self._r118_feedback_exit = r118_feedback_evidence.finish()
                mailbox.evidence_exit_status = self._r118_feedback_exit
                (self.run / "r118_feedback_exit.json").write_text(
                    json.dumps(self._r118_feedback_exit) + "\n"
                )
            except Exception as error:
                self._r118_feedback_close_error = type(error).__name__
                mailbox.evidence_exit_status = dict(
                    file_complete=False, close_error=type(error).__name__
                )
                # New optional evidence cannot skip the original shutdown writes.
            # Controller shutdown joins callback threads before this exit-only write.
            (self.run / "r86_prepared_feedback.json").write_text(
                json.dumps(mailbox.snapshot(), allow_nan=False) + "\n"
            )
        if _r87_diag.enabled():
            _r87_diag.flush(self.run / "r87_diagnostic_timeline.json")
        return super().destroy_node()

    @_callback_cost_observe("known_receive")
    def _known_receive(self, role, ns, value, _witness_entry_wall_ns=None):
        _corrected_arrival(
            self, role, ns, value if role == "metadata" else None, _witness_entry_wall_ns
        )
        if ns <= self.pose_stamp:
            return
        slot = self.known_scene_pending.setdefault(ns, {})
        slot[role] = value
        while len(self.known_scene_pending) > 24:
            self.known_scene_pending.popitem(last=False)
        if all(k in slot for k in ("body", "cloud", "metadata")):
            try:
                token = _corrected_call(
                    self,
                    ns,
                    "verify_corrected",
                    verify_corrected,
                    slot["body"],
                    slot["cloud"],
                    slot["metadata"],
                )
            except (ValueError, KeyError, TypeError):
                _corrected_finish(self, ns, "original_verify_rejected")
                del self.known_scene_pending[ns]
                return
            previous = getattr(self, "known_scene_commit_token", None)
            if previous is not None and (token[0] != previous[0] or token[1] <= previous[1]):
                _corrected_finish(self, ns, "original_token_rejected")
                del self.known_scene_pending[ns]
                return
            self.known_scene_commit_token = token
            self.known_scene_inputs = {
                **slot["metadata"],
                "received_monotonic_wall": time.monotonic(),
            }
            # Single executor callback: no control timer between the two inputs.
            _corrected_call(
                self, ns, "body_dispatch", self.known_scene_callbacks["body"], slot["body"]
            )
            _corrected_call(
                self, ns, "cloud_dispatch", self.known_scene_callbacks["cloud"], slot["cloud"]
            )
            _corrected_finish(self, ns, "original_dispatch_returned")
            self.evidence.append(
                self.run / "known_scene_controller_inputs.jsonl",
                dict(
                    source_ns=ns,
                    generation=slot["metadata"]["generation"],
                    accepted_pose_stamp_ns=int(self.pose_stamp),
                    accepted_cloud_stamp_ns=int(self.cloud_stamp),
                    control_allowed=slot["metadata"]["control_allowed"],
                    fixed_reference_frame="known scene world minus declared navigation origin",
                    navigation_ground_truth_used=False,
                ),
            )
            for key in list(self.known_scene_pending):
                if key <= ns:
                    del self.known_scene_pending[key]

    def on_localization_status(self, msg):
        try:
            d = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if (
            d.get("schema") != "known_scene_navigation_status/v1"
            or d.get("run_id") != self.run.name
        ):
            return
        self.known_scene_status = {**d, "received_monotonic_wall": time.monotonic()}
        transition = d.get("correction_transition") is True
        if transition and not self.known_scene_transition:
            _r105_terminal.invalidate(self, "known_scene_source_or_epoch_protection")
            self.publish_command()
            self.samples = None
            self.path_admission_active = None
            self.active_trajectory_id = None
            self.cascade = None
            self.cascade_key = None
            self.cascade_last_row = None
            self.path_receipt = None
            self.cascade_path_id = None
            self.final_bundle = None
            self.execution_path_exhaustion = None
            self.pid_prepared_output = None
            self.active_reference_stamp = None
            self.trajectory_association.reset()
            self.planning_start_state = None
            self.last_reference = -float("inf")
            self.reset_region_arrival("known_scene_correction_transaction")
            self.evidence.append(
                self.run / "known_scene_path_transactions.jsonl",
                dict(
                    event="hold_and_invalidate_prior_SCAN_path",
                    generation=d["generation"],
                    source_pose_stamp_ns=int(self.pose_stamp),
                    navigation_ground_truth_used=False,
                ),
            )
        if not transition and self.known_scene_transition:
            self.path_admission_hold = False
            self.path_admission_replan_pending = False
            self.samples = None
            self.last_reference = -float("inf")
            self.evidence.append(
                self.run / "known_scene_path_transactions.jsonl",
                dict(
                    event="commit_same_frame_inputs_then_replan",
                    generation=d["generation"],
                    ordered_region_receipts_preserved=len(self.region_arrivals),
                    navigation_ground_truth_used=False,
                ),
            )
        self.known_scene_transition = transition

    def on_scan_epoch_ready(self, msg):
        try:
            d = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if (
            d.get("schema") == "known_scene_SCAN_epoch_ready/v1"
            and d.get("run_id") == self.run.name
        ):
            self.known_scene_scan_epoch = {**d, "received_monotonic_wall": time.monotonic()}

    def _minimal_known_generation_ready(self):
        """R73: committed coordinate identity gates motion; heartbeat age is observed.

        Real pose/IMU causality, clock, tilt, bridge and checked-path protection
        remain in their existing downstream owners. This does not make a stale
        status heartbeat into a new sensor receipt or accept an old generation.
        """
        d = self.known_scene_inputs
        s = self.known_scene_status
        e = self.known_scene_scan_epoch
        now = time.monotonic()
        try:
            if not all(isinstance(v, dict) for v in (d, s, e)):
                raise ValueError("missing committed bundle/status/SCAN epoch")
            times = [v["received_monotonic_wall"] for v in (d, s, e)]
            if not all(
                type(t) in (int, float) and math.isfinite(t) and 0 <= now - t for t in times
            ):
                raise ValueError("invalid or future receipt clock")
            generation = d["generation"]
            stamp = d["source_ns"]
            parent = d["source_commit"]
            if (
                type(generation) is not int
                or generation < 0
                or type(stamp) is not int
                or stamp < 0
                or any(
                    type(v.get("generation")) is not int or v["generation"] != generation
                    for v in (s, e)
                )
            ):
                raise ValueError("generation or source stamp mismatch")
            if (
                s.get("initialized") is not True
                or e.get("ready") is not True
                or d.get("correction_transition") is not False
                or s.get("correction_transition") is not False
                or self.known_scene_transition
            ):
                raise ValueError("uninitialized or unfinished frame/SCAN transaction")
            if (
                not isinstance(parent, dict)
                or parent.get("explicit_message_commit_token") is not True
                or getattr(self, "known_scene_commit_token", None)
                != (parent.get("producer_epoch"), parent.get("commit"))
                or parent.get("stamp_ns") != stamp
                or self.pose_stamp != self.cloud_stamp
                or self.pose_stamp != stamp
            ):
                raise ValueError("current exact body/cloud commit mismatch")
            allowed = d.get("control_allowed")
            stale = d.get("correction_stale")
            if type(allowed) is not bool or type(stale) is not bool or (not allowed and not stale):
                raise ValueError("unexplained control authorization veto")
            f = self.feedback
            finite = lambda values, n: (
                isinstance(values, (list, tuple))
                and len(values) == n
                and all(type(x) in (int, float) and math.isfinite(x) for x in values)
            )
            if (
                not isinstance(f, dict)
                or f.get("stamp_ns") != stamp
                or f.get("frame_id") != "camera_init"
                or not finite(f.get("position_world_xyz"), 3)
                or not finite(f.get("origin_velocity_body"), 3)
                or not finite(f.get("quaternion_wxyz"), 4)
                or not 0.98 <= sum(x * x for x in f["quaternion_wxyz"]) <= 1.02
            ):
                raise ValueError("invalid actual same-frame SLAM feedback")
            frame = (generation, d["C_world_odom"], d["T_navigation_world"])
            if frame != getattr(self, "_r73_checked_known_frame", None):
                from embodied_navigation.localization_mapping.known_scene_matcher import (
                    checked_transform,
                )

                checked_transform(frame[1])
                checked_transform(frame[2])
                self._r73_checked_known_frame = copy.deepcopy(frame)
            ages = [now - t for t in times]
            original_ready = bool(all(age < KNOWN_BUNDLE_FRESHNESS_S for age in ages) and allowed)
            observation = dict(
                source_ns=stamp,
                generation=generation,
                bundle_wall_age_s=ages[0],
                status_wall_age_s=ages[1],
                SCAN_epoch_wall_age_s=ages[2],
                original_control_allowed=allowed,
                correction_stale=stale,
                original_known_scene_ready=original_ready,
                committed_same_frame=True,
                source_receipts_refreshed=False,
                acceptance_PASS=False,
            )
            self._r73_known_readiness_observation = observation
            if not original_ready:
                diagnostic_record(
                    self.run,
                    self.profile,
                    "r73_known_scene_motion_readiness",
                    "duplicate_heartbeat_age_or_correction_age_business_veto_observed",
                    observation,
                )
            return True
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            self._r73_known_readiness_observation = dict(hard_rejected=True, reason=str(error))
            return False

    def _known_generation_ready(self):
        if minimal_baseline_enabled(self.profile):
            return self._minimal_known_generation_ready()
        d = self.known_scene_inputs
        s = self.known_scene_status
        e = self.known_scene_scan_epoch
        now = time.monotonic()
        # The exact committed bundle is the live motion receipt. Status and
        # SCAN epoch messages retain their identity/authorization until changed;
        # their low-frequency heartbeat age is not another sensor-source TTL.
        receipts_ready = bool(
            d is not None
            and s is not None
            and e is not None
            and all(v.get("run_id") == self.run.name for v in (d, s, e))
            and 0 <= now - d["received_monotonic_wall"] < KNOWN_BUNDLE_FRESHNESS_S
            and all(
                math.isfinite(v["received_monotonic_wall"])
                and 0 <= now - v["received_monotonic_wall"]
                for v in (s, e)
            )
            and s.get("error") is None
        )
        original_ready = bool(
            receipts_ready
            and d.get("control_allowed")
            and not s.get("correction_transition")
            and s.get("initialized")
            and e.get("ready")
            and d["generation"] == s["generation"] == e["generation"]
            and d.get("source_commit", {}).get("explicit_message_commit_token") is True
            and self.pose_stamp == self.cloud_stamp == d["source_ns"]
        )
        if original_ready:
            return True
        # Only the correction-age business veto is observed. A correction
        # transaction, stale source, uninitialized frame, old planner generation
        # or a missing exact commit still takes the original zero/hold path.
        stale_only_ready = bool(
            receipts_ready
            and d.get("control_allowed") is False
            and d.get("correction_stale") is True
            and not d.get("correction_transition")
            and not s.get("correction_transition")
            and s.get("initialized")
            and e.get("ready")
            and d["generation"] == s["generation"] == e["generation"]
            and d.get("source_commit", {}).get("explicit_message_commit_token") is True
            and self.pose_stamp == self.cloud_stamp == d["source_ns"]
        )
        if stale_only_ready and diagnostic_enabled(self.profile):
            return diagnostic_record(
                self.run,
                self.profile,
                "known_scene_controller_authorization",
                "original_same_generation_control_allowed_false_due_to_correction_stale",
                dict(
                    source_ns=d["source_ns"],
                    generation=d["generation"],
                    match_source_ns=d.get("match_source_ns"),
                    original_control_allowed=False,
                    correction_stale=True,
                    correction_transition=False,
                    actual_source_fresh=True,
                    same_generation=True,
                    exact_source_commit_verified=True,
                    old_generation_curve_reused=False,
                ),
            )
        return False

    def _minimal_discard_prior_frame_path(self):
        _r105_terminal.invalidate(self, "known_scene_source_or_epoch_protection")
        # Same invalidation as the existing correction transaction; no fabricated
        # status/source event and no reused path across a committed frame change.
        self.samples = None
        self.path_admission_active = None
        self.active_trajectory_id = None
        self.cascade = None
        self.cascade_key = None
        self.cascade_last_row = None
        self.path_receipt = None
        self.cascade_path_id = None
        self.final_bundle = None
        self.execution_path_exhaustion = None
        self.pid_prepared_output = None
        self.active_reference_stamp = None
        self.trajectory_association.reset()
        self.planning_start_state = None
        self.last_reference = -float("inf")
        self.reset_region_arrival("r73_committed_navigation_frame_changed")

    def _control_candidate(self, now):
        _r102_progress.note(
            self,
            "admission_state",
            getattr(self, "path_admission_hold", None),
            "control_entry_state",
        )
        if minimal_baseline_enabled(self.profile) and self._known_generation_ready():
            frame = _frame(self)
            previous = getattr(self, "_last_authorized_hold_frame", None)
            if previous is not None and frame != previous:
                self._minimal_discard_prior_frame_path()
                self._last_authorized_hold_frame = copy.deepcopy(frame)
                self.publish_command()
                return
            self._last_authorized_hold_frame = copy.deepcopy(frame)
            _r102_progress.note(self, "known_hold", False, "original_ready_branch")
            return super()._control_candidate(now)
        if not self._known_generation_ready():
            if self.control_clock_hold.before_control(
                self, self.get_clock().now().nanoseconds, round(now * 1e9)
            ):
                return
            if _r105_terminal.selected(self) and self.state == "succeeded":
                self.apply_tilt_guard(now, self.get_clock().now().nanoseconds / 1e9)
                self.stale_since = self.stale_since or now
                if now - self.stale_since > 8.0:
                    self.state = "failed"
                    self.message = (
                        "Terminal same-frame sources unavailable beyond original 8 seconds"
                    )
            # Frame holds cannot bypass original deadlines or raw-source/tilt protection.
            if self.state == "running":
                ns = self.get_clock().now().nanoseconds
                if (
                    self.segment_start is not None
                    and not self.goals[self.waypoint_index].legacy
                    and self.segment_deadline_exceeded(ns)
                ):
                    continued = diagnostic_enabled(self.profile) and diagnostic_record(
                        self.run,
                        self.profile,
                        "known_scene_controller_region_timeout",
                        "original_region_sim_timeout_during_localization_wait",
                        dict(
                            control_stamp_ns=int(ns),
                            source_pose_stamp_ns=int(self.pose_stamp),
                            request_id=self.request_id,
                            waypoint_index=self.waypoint_index,
                            original_controller_state="failed",
                            original_zero_command=True,
                        ),
                    )
                    if not continued:
                        self.state, self.message = (
                            "failed",
                            "定位等待期间原区域超过仿真时间限制，停车",
                        )
                        self.reset_region_arrival("failed")
                        self.publish_command()
                        return
                self.apply_tilt_guard(now, ns / 1e9)
                _sensor_hold_row = _sensor_hold_begin(self, "known_scene_wait", now, ns)
                stale = (
                    now - self.pose_updated > self.pose_timeout
                    or now - self.cloud_updated > self.cloud_timeout
                    or not self.raw_imu.fresh(now, ns / 1e9)
                )
                if stale:
                    self.stale_since = self.stale_since or now
                    if now - self.stale_since > 8.0:
                        self.state, self.message = (
                            "failed",
                            "定位等待期间原始传感器连续失联超过8秒，停车",
                        )
                else:
                    self.stale_since = None
                _sensor_hold_end(self, _sensor_hold_row, stale)
                if self.state == "failed":
                    self.reset_region_arrival("failed")
                    self.publish_command()
                    return
            _r102_progress.note(
                self,
                "known_hold",
                True,
                getattr(self, "_r73_known_readiness_observation", {}).get(
                    "reason", "known_scene_not_ready"
                ),
            )
            self.publish_command()
            self.reset_region_arrival("known_scene_fresh_transaction_hold")
            observe_zero_hold(self, now, self.get_clock().now().nanoseconds)
            if self.state != "failed":
                self.message = "等待同一定位版本的新鲜位姿、点云或停车修正事务"
            return
        self._last_authorized_hold_frame = _frame(self)
        _r102_progress.note(self, "known_hold", False, "original_ready_branch")
        return super()._control_candidate(now)

    def request_plan(self):
        if not self._known_generation_ready():
            return
        generation = (
            None if self.known_scene_inputs is None else self.known_scene_inputs["generation"]
        )
        before = self.reference_requests
        result = super().request_plan()
        if self.reference_requests > before:
            self.known_scene_reference_generation = generation
        return result

    def accept_spline(self, msg, metadata):
        if (
            not self._known_generation_ready()
            or self.known_scene_reference_generation != self.known_scene_inputs["generation"]
        ):
            self.trajectory_association.reject(
                "known_scene_epoch_not_ready_or_old_request_generation"
            )
            return False
        return super().accept_spline(msg, metadata)
