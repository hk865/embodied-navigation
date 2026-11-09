"""Single-owner asynchronous physical certificates; no motion authority.

Business methods execute in the original control group. Opt-in short intake
only retains MessageOnly objects and receipts in the same bounded FIFO.
SCAN events are bounded FIFO; feedback is never buffered here. A returned proof
can only enter the original admission transaction after its owner is rechecked.
"""

from collections import deque
import copy
import os
import time
import threading

from embodied_navigation.common.observation import api as _observer_api

probe = _observer_api("r87_diagnostic_probe")
from embodied_navigation.planning.admission.physical_gate import digest
from embodied_navigation.planning.admission.geometry_worker import Worker
import embodied_navigation.planning.admission.queue_budget as r97_queue_budget
from embodied_navigation.common.observation import api as _observer_api

r107_scan_transport_probe = _observer_api("r107_scan_transport_probe")

MAX_EVENTS = 32
MAX_EVENT_BYTES = 1024 * 1024
MAX_QUEUE_BYTES = 2 * 1024 * 1024


def _poll_yield_selection():
    value = os.environ.get("R110_POLL_YIELD", "0")
    if value not in ("0", "1"):
        raise ValueError("R110_POLL_YIELD must be 0 or 1")
    return value == "1"


# Fixed at process import; both runtime binding and the actual instance expose it.
POLL_YIELD_ENABLED = _poll_yield_selection()


def _receive_pressure_selection():
    value = os.environ.get("R116_RECEIVE_PRESSURE", "0")
    if value not in ("0", "1"):
        raise ValueError("R116_RECEIVE_PRESSURE must be 0 or 1")
    return value == "1"


RECEIVE_PRESSURE_ENABLED = _receive_pressure_selection()
_SHORT_INTAKE = os.environ.get("SCAN_SHORT_INTAKE", "0")
if _SHORT_INTAKE not in ("0", "1"):
    raise ValueError("SCAN_SHORT_INTAKE must be 0 or 1")
SHORT_INTAKE_ENABLED = _SHORT_INTAKE == "1"


def enabled():
    """Whether the async physical certificate path is requested (R88_ASYNC_PHYSICAL=1)."""
    return os.environ.get("R88_ASYNC_PHYSICAL", "0") == "1"


def _ready_poll_enabled():
    setting = os.environ.get("R94_SCAN_READY_POLL", "0")
    if setting not in ("0", "1"):
        raise ValueError("R94_SCAN_READY_POLL must be 0 or 1")
    return setting == "1"


def scope(node):
    """Program owner snapshot: ids plus small schema3 metadata, no global graph."""
    association = node.trajectory_association
    runtime = node.automatic_reference
    inputs = node.known_scene_inputs or {}
    route = node.continuous_route
    key = getattr(association, "route_key", None)
    if key is None:
        raise ValueError("r88_route_definition_not_bound")
    return dict(
        admission=node.admission_identity(),
        state=node.state,
        scope=tuple(runtime.scope),
        reference_file_sha256=runtime.file_sha,
        route_key=key,
        association={
            name: copy.deepcopy(getattr(association, name, None))
            for name in (
                "epoch",
                "planner_process_id",
                "localization_generation",
                "reference_stamp",
                "reference_revision",
                "accepted_metadata",
                "accepted_highwater_id",
                "accepted_attempt_highwater_id",
                "goal",
            )
        },
        generation=inputs.get("generation"),
        requested_generation=node.known_scene_reference_generation,
        coverage=None if route is None else route.next_index,
        requested_coverage=node.scan_reference_coverage_index,
        active_trajectory_id=node.active_trajectory_id,
        active_reference_stamp=tuple(node.active_reference_stamp or ()),
        cascade_path_id=node.cascade_path_id,
    )


def source_advances(original, current):
    return (
        isinstance(original, tuple)
        and isinstance(current, tuple)
        and len(original) == len(current) == 2
        and original[0] == current[0]
        and type(original[1]) is int
        and type(current[1]) is int
        and current[1] >= original[1]
    )


class Admission:
    """Serialize one asynchronous physical certificate per owner snapshot.

    Holds at most one in-flight worker job plus a bounded FIFO of small SCAN
    events; every returned proof is revalidated against the live owner before the
    original synchronous admission transaction may run. No motion authority.
    """

    def __init__(self, node, worker=None, *, eager_submit=False):
        if type(eager_submit) is not bool:
            raise ValueError("eager_submit must be a bool")
        self.scan_ready_poll = _ready_poll_enabled()
        self.poll_yield_enabled = POLL_YIELD_ENABLED
        self.receive_pressure_enabled = RECEIVE_PRESSURE_ENABLED
        self.short_intake_enabled = SHORT_INTAKE_ENABLED
        if self.short_intake_enabled and (not enabled() or self.receive_pressure_enabled):
            raise ValueError("Short intake requires R88 and R116 explicitly disabled")
        self.event_lock = threading.Lock()
        self.intake_failure = None
        self.intake_shutdown_fault_attributed = False
        self.intake_scheduled = False
        self.intake_notifications = self.intake_callbacks = self.intake_trigger_errors = 0
        self.receive_pressure_calls = self.receive_pressure_consumed = 0
        self.receive_pressure_still_rejected = 0
        self.poll_yield_count = self.poll_yield_with_queue_count = 0
        self.poll_yield_last_queue_count = self.poll_yield_last_queue_bytes = 0
        self.queue_budget_selection = r97_queue_budget.selection(getattr(node, "profile", None))
        self.queue_byte_limit = self.queue_budget_selection["effective_bytes"]
        if self.short_intake_enabled and (
            self.queue_byte_limit != MAX_QUEUE_BYTES
            or getattr(node, "_r86_prepared_mailbox", None) is None
        ):
            raise ValueError("Short intake requires the original 2MiB FIFO and R86 fault barrier")
        self.queue_budget = (
            r97_queue_budget.Observation(self.queue_budget_selection)
            if self.queue_budget_selection["observation_enabled"]
            else None
        )
        self.queue_budget_observation_errors = 0
        self.node = node
        self.worker = Worker(eager_submit=eager_submit) if worker is None else worker
        self.events = deque()
        self.queue_bytes = 0
        self.current = None
        self.delivery = None
        self.sequence = self.event_sequence = 0
        self.closed = False
        self.fault = None
        self.counts = dict(
            received=0,
            consumed=0,
            submitted=0,
            completed=0,
            invalidated=0,
            discarded_events=0,
            faults=0,
        )
        self.handoff_probe = os.environ.get("R93_HANDOFF_PROBE", "0") == "1"
        self.transport_probe = r107_scan_transport_probe.create(node)
        self.intake_guard = None
        self.intake_subscriptions = ()
        if self.short_intake_enabled:
            topics = {
                "/demo/navigation/reference_attempt",
                "/demo/navigation/bspline",
                "/demo/navigation/trajectory_metadata",
                "/demo/navigation/route_definition",
            }
            subscriptions = [s for s in node.subscriptions if s.topic_name in topics]
            if (
                len(subscriptions) != 4
                or {s.topic_name for s in subscriptions} != topics
                or any(s.callback_group is not node._r86_intake_group for s in subscriptions)
            ):
                self.worker.close()
                raise ValueError("All four SCAN subscriptions must use the existing intake group")
            self.intake_subscriptions = tuple(subscriptions)
            try:
                self.intake_guard = node.create_guard_condition(
                    self._service_intake, callback_group=node.default_callback_group
                )
            except BaseException:
                self.worker.close()
                raise
            if self.intake_guard.callback_group is not node.default_callback_group:
                self.worker.close()
                raise ValueError("SCAN consumer must use the original control group")
        self.queue_probe_counts = dict(attempted=0, returned=0, dropped=0, errors=0, unavailable=0)
        self.queue_probe_available = False
        if self.handoff_probe:
            try:
                self.queue_probe_available = probe.enabled()
            except Exception:
                self.queue_probe_counts["errors"] += 1

    def _transport_mark(self, stage, event=None, **details):
        observer = self.transport_probe
        if observer is not None:
            try:
                observer.mark(self, stage, event, **details)
            except Exception:
                # Observation never replaces the original failure or result.
                try:
                    observer.errors += 1
                except Exception:
                    pass

    def _transport_context(self, method, *args):
        observer = self.transport_probe
        if observer is None:
            return None
        try:
            return getattr(observer, method)(*args)
        except Exception:
            try:
                observer.errors += 1
            except Exception:
                pass
            return None

    def _budget_observe(self, method, *args):
        # Optional observation cannot replace business exceptions or decisions.
        try:
            return getattr(self.queue_budget, method)(*args)
        except Exception:
            self.queue_budget_observation_errors += 1
            return None

    def _queue_state(self):
        # No new clocks when OFF or R87 is unavailable; no payload is retained.
        if not self.handoff_probe or not self.queue_probe_available:
            return None
        try:
            count, size = self.queue_snapshot()
            return dict(
                count=count,
                bytes=size,
                token=None if self.current is None else self.current["token"],
                busy=bool(self.worker.busy),
                wall_ns=time.monotonic_ns(),
            )
        except Exception:
            self.queue_probe_counts["errors"] += 1
            return None

    def _queue_mark(
        self,
        boundary,
        event=None,
        before=None,
        *,
        in_queue_after=None,
        overflow=False,
        count_bound=False,
        bytes_bound=False,
        reason=None,
    ):
        if not self.handoff_probe:
            return
        counts = self.queue_probe_counts
        counts["attempted"] += 1
        if not self.queue_probe_available:
            counts["unavailable"] += 1
            counts["dropped"] += 1
            return
        try:
            after = self._queue_state()
            if after is None:
                counts["dropped"] += 1
                return
            before = after if before is None else before
            row = probe.mark(
                "scan.r93_queue." + boundary,
                source_ns=getattr(self.node, "pose_stamp", None),
                event_index=None if event is None else event["index"],
                kind=None if event is None else event["kind"],
                event_size_bytes=None if event is None else event["size"],
                q_count_before=before["count"],
                q_count_after=after["count"],
                q_bytes_before=before["bytes"],
                q_bytes_after=after["bytes"],
                sampled_mono_before=before["wall_ns"],
                sampled_mono_after=after["wall_ns"],
                current_token_before=before["token"],
                current_token_after=after["token"],
                worker_busy_before=before["busy"],
                worker_busy_after=after["busy"],
                event_in_queue_after=in_queue_after,
                overflow_trigger=overflow,
                count_bound=count_bound,
                bytes_bound=bytes_bound,
                reason=reason,
            )
            # Recorder can return a row even when its nonblocking lock drops it.
            # Those drops are reported by the shared R87 counters at close.
            counts["returned" if row is not None else "dropped"] += 1
        except Exception:
            counts["errors"] += 1
            counts["dropped"] += 1

    def _queue_probe_receipt(self):
        try:
            recorder = probe.RECORDER
            available = self.queue_probe_available and recorder is not None
            global_counts = (
                None
                if recorder is None
                else dict(
                    dropped_lock_contention=recorder.dropped,
                    probe_errors=recorder.error_count,
                    evicted=recorder.evicted,
                )
            )
            loss = (
                self.queue_probe_counts["dropped"] > 0
                or self.queue_probe_counts["errors"] > 0
                or any((global_counts or {}).values())
            )
            return dict(
                enabled=True,
                r87_available=available,
                counts=dict(self.queue_probe_counts),
                r87_global_counts=global_counts,
                validity=(
                    "unavailable"
                    if not available
                    else "bounded_observation_with_loss"
                    if loss
                    else "bounded_observation_no_reported_loss"
                ),
                global_counts_are_not_attributable_to_this_instance=True,
            )
        except Exception:
            self.queue_probe_counts["errors"] += 1
            return dict(
                enabled=True,
                r87_available=False,
                validity="unavailable",
                counts=dict(self.queue_probe_counts),
            )

    def record(self, event, **details):
        try:
            self.node.evidence.append(
                self.node.run / "r88_async_admission.jsonl",
                dict(
                    schema="r88_async_physical_admission/v1",
                    event=event,
                    observed_monotonic_wall_ns=time.monotonic_ns(),
                    control_stamp_ns=int(self.node.get_clock().now().nanoseconds),
                    source_pose_stamp_ns=int(self.node.pose_stamp),
                    navigation_PASS=False,
                    **details,
                ),
            )
        except Exception:
            self.fault = "r88_async_evidence_failed"
            self.counts["faults"] += 1
            probe.anomaly(self.fault, source_ns=getattr(self.node, "pose_stamp", None))
            if not self.closed:
                self.node.state = "failed"
                self.node.message = self.fault
                self.node.publish_command()
            raise

    def queue_snapshot(self):
        with self.event_lock:
            return len(self.events), self.queue_bytes

    def _notify_intake(self, *, arrival=False):
        if not self.short_intake_enabled:
            return
        with self.event_lock:
            notify = (
                not self.closed
                and not self.fault
                and not self.intake_scheduled
                and (
                    self.intake_failure is not None
                    or bool(self.events)
                    and (arrival or self.current is None and not self.worker.busy)
                )
            )
            if notify:
                self.intake_scheduled = True
                self.intake_notifications += 1
        if notify:
            try:
                self.intake_guard.trigger()
            except Exception as error:
                with self.event_lock:
                    self.intake_trigger_errors += 1
                self.node._r86_prepared_mailbox.fail("r88_intake_notify:" + type(error).__name__)

    def _record_intake_received(self, event):
        if not event.get("short_intake") or event.get("received_recorded"):
            return
        event["received_recorded"] = True
        self.record(
            "event_received",
            event_index=event["index"],
            kind=event["kind"],
            original_received_wall_ns=event["received_wall_ns"],
            original_received_ros_ns=event["received_ros_ns"],
        )

    def _apply_intake_failure(self):
        with self.event_lock:
            failure = self.intake_failure
            report = failure is not None and not failure["reported"]
            if report:
                failure["reported"] = True
        if report:
            self.record(
                "event_not_consumed", **{k: v for k, v in failure.items() if k != "reported"}
            )
            self.fail(failure["reason"])
        return failure is not None

    def _service_intake(self):
        with self.event_lock:
            self.intake_scheduled = False
            self.intake_callbacks += 1
        if self.closed or self._apply_intake_failure():
            return
        # Same owner and integrity barrier as the old receive-side ready poll.
        if self.node._r86_fail_closed():
            self.node.publish_command()
            return
        try:
            self.poll()
        except Exception as error:
            self.node.state = "failed"
            self.node.message = "R88 async admission: " + type(error).__name__ + ":" + str(error)
            self.node.publish_command()
            raise

    def _receive_intake(self, kind, msg, callback):
        now = time.monotonic_ns()
        clock = int(self.node.get_clock().now().nanoseconds)
        with self.event_lock:
            self.event_sequence += 1
            index = self.event_sequence
            self.counts["received"] += 1
            if self.closed or self.fault or self.intake_failure is not None:
                self.counts["discarded_events"] += 1
                return
        self._transport_mark(1, kind=kind, msg=msg, event_index=index, received_mono=now)
        reason = None
        if kind == "spline":
            if len(msg.pos_pts) > 512 or len(msg.knots) > 516 or len(msg.yaw_pts) > 512:
                reason = "r88_scan_event_count_bound"
            size = 64 + len(msg.pos_pts) * 24 + (len(msg.knots) + len(msg.yaw_pts)) * 8
        else:
            size = (
                MAX_EVENT_BYTES + 1
                if len(msg.data) > MAX_EVENT_BYTES
                else len(msg.data.encode("utf-8"))
            )
        if size > MAX_EVENT_BYTES:
            reason = reason or "r88_scan_event_bytes_bound"
        event = dict(
            index=index,
            kind=kind,
            msg=msg,
            callback=callback,
            received_wall_ns=now,
            received_ros_ns=clock,
            size=size,
            short_intake=True,
        )
        with self.event_lock:
            if self.closed or self.fault or self.intake_failure is not None:
                self.counts["discarded_events"] += 1
                return
            if reason is None and (
                len(self.events) >= MAX_EVENTS or self.queue_bytes + size > self.queue_byte_limit
            ):
                reason = "r88_scan_event_queue_bound"
            if reason is None:
                # rclpy MessageOnly hands this callback an owned Python message;
                # its strong reference is retained, never mutated or recopied.
                self.events.append(event)
                self.queue_bytes += size
            else:
                self.counts["discarded_events"] += 1
                self.intake_failure = dict(
                    reason=reason,
                    event_index=index,
                    kind=kind,
                    original_received_wall_ns=now,
                    original_received_ros_ns=clock,
                    reported=False,
                )
        if reason is not None:
            self._save_failure_snapshot(reason)
            # No node state, JSON evidence, polling or publication in intake.
            self.node._r86_prepared_mailbox.fail(reason)
            self._transport_mark(8 if reason.endswith("queue_bound") else 2, event)
        else:
            self._transport_mark(3, event)
            self._transport_mark(4, event)
        self._notify_intake(arrival=True)

    def _save_failure_snapshot(self, reason):
        """One bounded fault-only diagnostic; never modifies the queue or gate."""
        try:
            import json

            target = self.node.run / "admission_failure.json"
            if target.exists():
                return
            now = time.monotonic_ns()
            with self.event_lock:
                queue = [
                    dict(
                        index=e["index"],
                        kind=e["kind"],
                        size=e["size"],
                        wall_age_ns=now - e["received_wall_ns"],
                        source_clock_ns=e["received_ros_ns"],
                    )
                    for e in self.events
                ]
                current = (
                    None
                    if self.current is None
                    else {
                        k: self.current.get(k)
                        for k in ("token", "started", "received_wall_ns", "event_index", "owner")
                    }
                )
                state = dict(
                    reason=reason,
                    counts=dict(self.counts),
                    events=queue,
                    bytes=self.queue_bytes,
                    count_limit=MAX_EVENTS,
                    byte_limit=self.queue_byte_limit,
                    notifications=self.intake_notifications,
                    callbacks=self.intake_callbacks,
                    scheduled=self.intake_scheduled,
                    current=current,
                    worker_busy=self.worker.busy,
                    worker_pid=self.worker.pid,
                    worker_failure=self.worker.failure,
                    worker_exit=self.worker._process.poll(),
                )
            target.write_text(json.dumps(state, allow_nan=False, default=str) + "\n")
        except Exception:
            pass  # Observation failure cannot change the original protection.

    def receive(self, kind, msg, callback):
        """Ingress of one small SCAN event; callbacks may run synchronously, with no cloud copy."""
        if self.short_intake_enabled:
            return self._receive_intake(kind, msg, callback)
        now = time.monotonic_ns()
        self._transport_mark(
            1, kind=kind, msg=msg, event_index=self.event_sequence + 1, received_mono=now
        )
        self.event_sequence += 1
        self.counts["received"] += 1
        if kind == "spline":
            if len(msg.pos_pts) > 512 or len(msg.knots) > 516 or len(msg.yaw_pts) > 512:
                self._transport_mark(
                    2,
                    kind=kind,
                    msg=msg,
                    outcome=1,
                    event_index=self.event_sequence,
                    received_mono=now,
                )
                return self.fail("r88_scan_event_count_bound")
            size = 64 + len(msg.pos_pts) * 24 + (len(msg.knots) + len(msg.yaw_pts)) * 8
        else:
            # Check character bound before allocating UTF-8 bytes.
            if len(msg.data) > MAX_EVENT_BYTES:
                self._transport_mark(
                    2, kind=kind, outcome=2, event_index=self.event_sequence, received_mono=now
                )
                return self.fail("r88_scan_event_bytes_bound")
            size = len(msg.data.encode("utf-8"))
        if size > MAX_EVENT_BYTES:
            self._transport_mark(
                2,
                kind=kind,
                msg=msg,
                size=size,
                outcome=3,
                event_index=self.event_sequence,
                received_mono=now,
            )
            return self.fail("r88_scan_event_bytes_bound")
        event = dict(
            index=self.event_sequence,
            kind=kind,
            msg=msg,
            callback=callback,
            received_wall_ns=now,
            received_ros_ns=int(self.node.get_clock().now().nanoseconds),
            size=size,
        )
        self._transport_mark(3, event)
        if self.queue_budget is not None:
            self._budget_observe("received", event, self.events, self.queue_bytes)
        queue_before = self._queue_state()
        self.record(
            "event_received",
            event_index=event["index"],
            kind=kind,
            original_received_wall_ns=now,
            original_received_ros_ns=event["received_ros_ns"],
        )
        self._queue_mark("received_before_disposition", event, queue_before, in_queue_after=False)
        ready_polled = False
        pressure_polled = False
        if not self.closed and not self.fault and self.scan_ready_poll:
            self._queue_mark("r94_ready_check_enter", event, queue_before, in_queue_after=False)
            self._transport_mark(20, event)
            readable = self.worker.result_readable()
            self._transport_mark(21, event, outcome=int(readable))
            self._queue_mark(
                "r94_ready_check_exit",
                event,
                queue_before,
                in_queue_after=False,
                reason="readable" if readable else "not_readable",
            )
            if readable:
                # Intake may have latched an integrity fault before its next
                # control tick. Keep that tick's existing fail-closed ordering.
                if self.node._r86_fail_closed():
                    self.node.publish_command()
                    self.counts["discarded_events"] += 1
                    self._transport_mark(14, event, outcome=1)
                    self.record(
                        "event_not_consumed",
                        event_index=event["index"],
                        reason="r86_integrity_fault_before_ready_poll",
                        original_received_wall_ns=event["received_wall_ns"],
                        original_received_ros_ns=event["received_ros_ns"],
                    )
                    self._queue_mark(
                        "r94_rejected_r86_fault",
                        event,
                        queue_before,
                        in_queue_after=False,
                        reason="r86_integrity_fault_before_ready_poll",
                    )
                    return
                # Same control group, once only. Original poll retains owner/live
                # guards and consumes at most three older association events.
                self._queue_mark("r94_extra_poll_enter", event, queue_before, in_queue_after=False)
                self._transport_mark(22, event)
                try:
                    if self.receive_pressure_enabled:
                        pressure_polled = self.poll(receive_event=event) is True
                    else:
                        self.poll()
                except Exception as error:
                    # Match the original control-tick exceptional poll exit.
                    self.node.state = "failed"
                    self.node.message = (
                        "R88 async admission: " + type(error).__name__ + ":" + str(error)
                    )
                    self.node.publish_command()
                    raise
                finally:
                    self._transport_mark(23, event)
                    self._queue_mark(
                        "r94_extra_poll_exit", event, queue_before, in_queue_after=False
                    )
                ready_polled = True
                queue_before = self._queue_state()
        if self.closed or self.fault:
            self.counts["discarded_events"] += 1
            self._transport_mark(14, event, outcome=2)
            self.record(
                "event_not_consumed", event_index=event["index"], reason=self.fault or "closed"
            )
            self._queue_mark(
                "rejected_closed_or_faulted",
                event,
                queue_before,
                in_queue_after=False,
                reason=self.fault or "closed",
            )
            return
        # After a ready poll, defer this incoming event even if the old queue
        # emptied: otherwise three old callbacks plus this one would make four.
        if ready_polled or self.current is not None or self.worker.busy or self.events:
            if self.queue_budget is not None:
                self._budget_observe(
                    "projected", event, self.events, self.queue_bytes, self.current
                )
            if len(self.events) >= MAX_EVENTS or self.queue_bytes + size > self.queue_byte_limit:
                if pressure_polled:
                    self.receive_pressure_still_rejected += 1
                self._transport_mark(
                    8,
                    event,
                    outcome=(
                        int(len(self.events) >= MAX_EVENTS)
                        + 2 * int(self.queue_bytes + size > self.queue_byte_limit)
                    ),
                )
                self._queue_mark(
                    "overflow_before_fail",
                    event,
                    queue_before,
                    in_queue_after=False,
                    overflow=True,
                    count_bound=len(self.events) >= MAX_EVENTS,
                    bytes_bound=self.queue_bytes + size > self.queue_byte_limit,
                    reason="r88_scan_event_queue_bound",
                )
                return self.fail("r88_scan_event_queue_bound")
            # Only small spline/String messages; never copy any point cloud.
            event["msg"] = copy.deepcopy(msg)
            self.events.append(event)
            self.queue_bytes += size
            self._transport_mark(4, event)
            if self.queue_budget is not None:
                self._budget_observe("appended", event, self.events, self.queue_bytes, self.current)
            self._queue_mark("append_completed", event, queue_before, in_queue_after=True)
            return
        return self.consume(event)

    def consume(self, event):
        """Run the original business callback once, in its original callback group."""
        self._record_intake_received(event)
        self.delivery = event
        self._transport_mark(6, event)
        observer = self.transport_probe
        prior_context = self._transport_context("prior_context", event["kind"])
        self.counts["consumed"] += 1
        self.record(
            "event_consumed",
            event_index=event["index"],
            kind=event["kind"],
            original_received_wall_ns=event["received_wall_ns"],
            original_received_ros_ns=event["received_ros_ns"],
            consumed_wall_ns=time.monotonic_ns(),
        )
        self._queue_mark("before_business_callback", event, in_queue_after=False)
        if self.queue_budget is not None:
            self._budget_observe("consumed", event)
        completed = False
        try:
            result = event["callback"](event["msg"])
            completed = True
            return result
        finally:
            context = self._transport_context("completed_context", event, prior_context)
            self._transport_mark(7, event, outcome=1 if completed else -1, context=context)
            if self.queue_budget is not None:
                self._budget_observe("callback_finished", event, completed)
            self.delivery = None

    def start(self, msg, metadata, *, received_wall_ns=None, admission_started=None):
        """Capture the owner snapshot and submit one physical certificate job."""
        started = time.monotonic() if admission_started is None else admission_started
        receipt = received_wall_ns
        if receipt is None:
            association = self.node.trajectory_association
            original = getattr(association, "accepted_metadata_received_wall_ns", None)
            receipt = (
                original
                if type(original) is int and association.accepted_metadata == metadata
                else time.monotonic_ns()
                if self.delivery is None
                else self.delivery["received_wall_ns"]
            )
        if self.current is not None or self.worker.busy:
            return self.fail("r88_overlapping_admission_owner")
        self.sequence += 1
        token = self.sequence
        try:
            from embodied_navigation.planning.reference_protocol import payload

            if len(msg.pos_pts) > 512 or len(msg.knots) > 516 or len(msg.yaw_pts) > 512:
                raise ValueError("candidate_input_count_budget")
            value = payload(msg)
            with probe.span("scan.async_prepare", source_ns=self.node.pose_stamp, job_token=token):
                _, _, binding = self.node._route_fence_load()
                prepared = self.node.r85_physical_gate.prepare(self.node, value, metadata, binding)
                owner = scope(self.node)
            item = dict(
                token=token,
                msg=copy.deepcopy(msg),
                metadata=copy.deepcopy(metadata),
                prepared=prepared,
                owner=owner,
                source_token=self.node.known_scene_commit_token,
                started=started,
                received_wall_ns=receipt,
                event_index=None if self.delivery is None else self.delivery["index"],
            )
            self.worker.submit(token, prepared["physical_payload"])
        except (
            OSError,
            KeyError,
            ValueError,
            TypeError,
            AttributeError,
            IndexError,
            OverflowError,
            RuntimeError,
        ) as error:
            # Use the unchanged rejection/retention branch; never compute here.
            self.record(
                "submit_rejected", job_token=token, error=type(error).__name__ + ":" + str(error)
            )
            return self.node._admit_matched_spline(
                msg,
                metadata,
                received_wall_ns=receipt,
                admission_started=started,
                physical_result={"error": type(error).__name__ + ":" + str(error)},
            )
        self.current = item
        self.counts["submitted"] += 1
        self._transport_mark(10, self.delivery, job_token=token)
        try:
            self.record(
                "submitted",
                job_token=token,
                trajectory_id=int(msg.traj_id),
                owner=owner,
                source_token=item["source_token"],
                original_received_wall_ns=receipt,
                admission_started_monotonic=started,
                event_index=item["event_index"],
                physical_payload_sha256=digest(prepared["physical_payload"]),
                worker_pid=self.worker.pid,
            )
        except Exception:
            self.current = None
            self.fault = "r88_submission_evidence_failed"
            self.node.state = "failed"
            self.node.message = self.fault
            self.node.publish_command()
            raise
        probe.mark("scan.async_submitted", source_ns=self.node.pose_stamp, job_token=token)

    def valid(self, item):
        try:
            return scope(self.node) == item["owner"] and source_advances(
                item["source_token"], self.node.known_scene_commit_token
            )
        except (AttributeError, KeyError, TypeError, ValueError, IndexError):
            return False

    def poll(self, *, receive_event=None):
        """Service the FIFO and at most one worker result; called from control ticks."""
        if not self.short_intake_enabled:
            return self._poll(receive_event=receive_event)
        if self.closed or self.fault:
            return
        try:
            if self._apply_intake_failure():
                return
            return self._poll(receive_event=receive_event)
        finally:
            self._notify_intake()

    def _poll(self, *, receive_event=None):
        if self.closed or self.fault:
            return
        # Observe owner loss even before a result arrives. Once seen, returning
        # to equal fields cannot revive that job (normal source advance is OK).
        if self.current is not None and not self.valid(self.current):
            self.invalidate("owner_changed_while_compute_pending")
        pressure_after_result = False
        callback_limit = 3
        result = self.worker.poll()
        if result is not None:
            token, proof, metrics = result
            item = self.current
            if item is None or token != item["token"]:
                # No result can acquire another job's authority, including ABA.
                self.record("orphan_result", job_token=token, metrics=metrics)
            elif not self.valid(item):
                self.current = None
                self.counts["invalidated"] += 1
                self.record(
                    "discard_changed_owner",
                    job_token=token,
                    metrics=metrics,
                    original_received_wall_ns=item["received_wall_ns"],
                )
            else:
                # Actual start of this synchronous result-dispatch stage.
                # The original job start remains end-to-end provenance only.
                callback_started = time.monotonic()
                self.current = None
                self.counts["completed"] += 1
                self.record(
                    "result_consumed",
                    job_token=token,
                    metrics=metrics,
                    original_received_wall_ns=item["received_wall_ns"],
                    admission_elapsed_wall_ms=(time.monotonic() - item["started"]) * 1000,
                )
                probe.mark("scan.async_consumed", source_ns=self.node.pose_stamp, job_token=token)
                physical = (
                    {"error": metrics["error"]}
                    if metrics.get("error")
                    else {"prepared": item["prepared"], "proof": proof}
                )
                # Re-run live guards and original atomic commit in this group.
                with probe.span(
                    "scan.async_commit", source_ns=self.node.pose_stamp, job_token=token
                ):
                    self._transport_mark(
                        11,
                        kind="spline",
                        msg=item["msg"],
                        job_token=token,
                        event_index=item["event_index"],
                        received_mono=item["received_wall_ns"],
                    )
                    committed_callback_returned = False
                    try:
                        self.node._admit_matched_spline(
                            item["msg"],
                            item["metadata"],
                            received_wall_ns=item["received_wall_ns"],
                            admission_started=callback_started,
                            physical_result=physical,
                        )
                        committed_callback_returned = True
                    finally:
                        self._transport_mark(
                            15,
                            kind="spline",
                            msg=item["msg"],
                            job_token=token,
                            event_index=item["event_index"],
                            received_mono=item["received_wall_ns"],
                            outcome=1 if committed_callback_returned else -1,
                        )
                    # Observe only a returned callback; an original exception
                    # propagates unchanged without a new evidence call masking it.
                    callback_return_observed = time.monotonic()
                    self.record(
                        "post_callback_return_observed",
                        job_token=token,
                        admission_started_monotonic=item["started"],
                        parent_callback_started_monotonic=callback_started,
                        parent_callback_return_observed_monotonic=callback_return_observed,
                        parent_callback_elapsed_wall_ms=(
                            callback_return_observed - callback_started
                        )
                        * 1000,
                        admission_end_to_end_wall_ms=(callback_return_observed - item["started"])
                        * 1000,
                        callback_budget_scope="single_synchronous_parent_stage",
                        original_received_wall_ns=item["received_wall_ns"],
                    )
            if metrics.get("error"):
                # Transport/worker death is global, even after an owner reset.
                self.fail("r88_physical_worker_failed")
                return
            if self.poll_yield_enabled:
                # Only receive's original incoming projection can borrow one
                # FIFO callback after this result's complete live transaction.
                # No incoming payload is retained here or consumed ahead of FIFO.
                if (
                    self.receive_pressure_enabled
                    and receive_event is not None
                    and not self.closed
                    and not self.fault
                    and self.current is None
                    and not self.worker.busy
                    and self.events
                    and (
                        len(self.events) >= MAX_EVENTS
                        or self.queue_bytes + receive_event["size"] > self.queue_byte_limit
                    )
                ):
                    pressure_after_result = True
                    callback_limit = 1
                    self.receive_pressure_calls += 1
                else:
                    # Count only an actual immediate result yield, not a drain.
                    self.poll_yield_count += 1
                    self.poll_yield_with_queue_count += int(bool(self.events))
                    self.poll_yield_last_queue_count = len(self.events)
                    self.poll_yield_last_queue_bytes = self.queue_bytes
                    return
        # A bounded number of original association callbacks per tick. Once a
        # match submits work, stop; negative candidates cannot be coalesced.
        self._transport_mark(24)
        for _ in range(callback_limit):
            if self.current is not None or self.worker.busy:
                break
            queue_before = self._queue_state()
            with self.event_lock:
                halted = self.closed or self.fault or self.intake_failure is not None
                event = None
                if not halted and self.events:
                    event = self.events.popleft()
                    self.queue_bytes -= event["size"]
            if halted:
                if not self.closed:
                    self._apply_intake_failure()
                return
            if event is None:
                break
            self._transport_mark(5, event)
            if self.queue_budget is not None:
                self._budget_observe("popped", event, self.events, self.queue_bytes, self.current)
            self._queue_mark("popleft_completed", event, queue_before, in_queue_after=False)
            if pressure_after_result:
                self.receive_pressure_consumed += 1
            self.consume(event)
        with self.event_lock:
            pump_ready = (
                not self.closed
                and not self.fault
                and self.intake_failure is None
                and self.current is None
                and not self.worker.busy
                and not self.events
            )
        if pump_ready:
            pump = getattr(self.node, "pump_route_definition_pending", None)
            if callable(pump):
                pump()  # At most one existing pending pair, original owner.
        self._transport_mark(25)
        if self.queue_budget is not None:
            self._budget_observe(
                "idle", self.events, self.queue_bytes, self.current, self.worker.busy
            )
        if pressure_after_result:
            return True  # receive still applies its original closed/fault/bounds checks

    def invalidate(self, reason):
        """Drop the in-flight job and queued events without granting their proofs."""
        queue_before = self._queue_state()
        with self.event_lock:
            item, events = self.current, tuple(self.events)
            previous_bytes = self.queue_bytes
            self.current = None
            self.events.clear()
            self.queue_bytes = 0
            self.counts["discarded_events"] += len(events)
        if self.queue_budget is not None:
            self._budget_observe("cleared", events, previous_bytes, item, reason)
        self._queue_mark("invalidated_queue_cleared", before=queue_before, reason=reason)
        if item is not None:
            self.counts["invalidated"] += 1
            self._transport_mark(12, job_token=item["token"])
            self.record("owner_invalidated", job_token=item["token"], reason=reason)
        for event in events:
            self._record_intake_received(event)
            self._transport_mark(9, event)
            self.record(
                "event_not_consumed",
                event_index=event["index"],
                reason=reason,
                original_received_wall_ns=event["received_wall_ns"],
            )
            self._queue_mark(
                "invalidated_event", event, queue_before, in_queue_after=False, reason=reason
            )

    def fail(self, reason):
        """Latch a terminal admission fault: fail-closed publication, then clear state."""
        self._transport_mark(13)
        queue_before = self._queue_state()
        self._queue_mark("fail_before_invalidate", before=queue_before, reason=reason)
        self.fault = reason
        self.counts["faults"] += 1
        probe.anomaly(reason, source_ns=getattr(self.node, "pose_stamp", None))
        self.node.state = "failed"
        self.node.message = reason
        try:
            self.node.publish_command()
        finally:
            self.invalidate(reason)
        self.record("fail_closed", reason=reason)
        self._queue_mark("fail_after_invalidate", before=queue_before, reason=reason)

    def close(self):
        """Shut down the worker and return its exit receipt; shutdown errors may propagate."""
        with self.event_lock:
            if self.closed:
                return None
            self.closed = True
            self.intake_scheduled = False
            if self.intake_failure is not None and self.fault is None:
                # Executor callbacks have joined; retain a fault that never got
                # an owner opportunity without inventing a late publication.
                self.fault = self.intake_failure["reason"]
                self.counts["faults"] += 1
                self.intake_shutdown_fault_attributed = True
        error = None
        try:
            self.invalidate("controller_exit")
        except Exception as failure:
            error = type(failure).__name__ + ":" + str(failure)
        worker_exit = self.worker.close()
        receipt = dict(
            schema="r88_async_physical_exit/v1",
            counts=self.counts,
            fault=self.fault,
            evidence_error=error,
            worker=worker_exit,
            r97_queue_budget=dict(self.queue_budget_selection),
        )
        if self.short_intake_enabled:
            receipt["scan_short_intake"] = self.intake_snapshot()
        if self.poll_yield_enabled:
            receipt["r110_poll_yield"] = dict(
                enabled=True,
                result_yields=self.poll_yield_count,
                yields_with_queued_events=self.poll_yield_with_queue_count,
                last_yield_queue_count=self.poll_yield_last_queue_count,
                last_yield_queue_bytes=self.poll_yield_last_queue_bytes,
                subsequent_feedback_service_guaranteed=False,
                original_queue_bounds_and_timestamps_preserved=True,
            )
        if self.receive_pressure_enabled:
            receipt["r116_receive_pressure"] = dict(
                enabled=True,
                receive_pressure_after_result_calls=self.receive_pressure_calls,
                consumed=self.receive_pressure_consumed,
                still_rejected=self.receive_pressure_still_rejected,
                maximum_callbacks_per_pressure_result=1,
                wall_time_bound_claimed=False,
                incoming_still_deferred_by_ready_polled=True,
            )
        if self.queue_budget is not None:
            receipt["r97_queue_observation"] = self._budget_observe(
                "snapshot", self.events, self.queue_bytes, self.node, self.fault
            )
            receipt["r97_queue_observation_errors"] = self.queue_budget_observation_errors
            receipt["r97_queue_observation_dropped_calls"] = self.queue_budget_observation_errors
        if self.handoff_probe:
            receipt["r93_queue_probe"] = self._queue_probe_receipt()
        if self.transport_probe is not None:
            try:
                receipt["r107_scan_transport"] = self.transport_probe.close()
                receipt["r107_scan_take"] = getattr(self.node, "_r107_take_receipt", None)
            except Exception:
                receipt["r107_scan_transport"] = dict(enabled=True, close_failed=True)
        return receipt

    def intake_snapshot(self):
        with self.event_lock:
            return dict(
                enabled=self.short_intake_enabled,
                closed=self.closed,
                one_original_fifo=True,
                count_limit=MAX_EVENTS,
                byte_limit=self.queue_byte_limit,
                max_event_bytes=MAX_EVENT_BYTES,
                queue_count=len(self.events),
                queue_bytes=self.queue_bytes,
                same_control_group=(
                    self.intake_guard is not None
                    and self.intake_guard.callback_group is self.node.default_callback_group
                ),
                ingress_group_is_r86=(
                    len(self.intake_subscriptions) == 4
                    and all(
                        s.callback_group is self.node._r86_intake_group
                        for s in self.intake_subscriptions
                    )
                ),
                subscription_count=len(self.intake_subscriptions),
                notifications=self.intake_notifications,
                callbacks=self.intake_callbacks,
                trigger_errors=self.intake_trigger_errors,
                failure=self.intake_failure,
                shutdown_fault_attributed_without_publication=self.intake_shutdown_fault_attributed,
                receive_pressure_enabled=self.receive_pressure_enabled,
                receive_side_ready_poll=False,
                payload_copy=False,
            )
