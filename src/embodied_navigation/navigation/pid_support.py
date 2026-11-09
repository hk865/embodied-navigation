"""Shared imports, evidence helpers and probe wrappers for the live controller.

Formerly the top of ``pid_navigation.py``. The controller class is composed from
``pid_ingress``/``pid_admission``/``pid_admission_apply``/``pid_route``/
``pid_control``; the process lifecycle lives in ``pid_entrypoint``.
"""


import argparse
import copy
import hashlib
import json
import math
import os
import time
import re
import threading
import pickle
from embodied_navigation.control.parking.causal_join import PARKING_CAUSAL_JOIN_ENABLED
from embodied_navigation.control.parking.terminal_owner import TERMINAL_PARKING_OWNER_ENABLED
import embodied_navigation.control.parking.terminal_owner as _r105_terminal
from embodied_navigation.common.observation import api as _observer_api

_r87_diag = _observer_api("r87_diagnostic_probe")

_r96_cpu = _observer_api("r96_control_cpu_probe")

_r102_progress = _observer_api("r102_progress_probe")

_r99_clock_probe = _observer_api("r99_clock_probe")

_r118_source_age = _observer_api("r118_source_age_probe")
import embodied_navigation.common.time.clock_service as _r114_clock_service

_r107_scan_take_probe = _observer_api("r107_scan_take_probe")
from embodied_navigation.common.time.clock_service import CLOCK_SERVICE_ENABLED
from collections import deque, OrderedDict
from pathlib import Path
from embodied_navigation.control.safety.handoff_stop_policy import tail_stop_request
from embodied_navigation.control.safety.launch_command_policy import constrain_exactzero_launch
from embodied_navigation.planning.admission.path_handoff import (
    stopped_rebind as stopped_path_rebind,
    retention_reference_eligibility,
)
import numpy as np
from embodied_navigation.navigation.ros_node import TeacherNavigation
from embodied_navigation.control.tracking.pid_core import slew
from embodied_navigation.control.parking.parking_zero_contract import make_event
from embodied_navigation.control.parking.parking_contract import enabled as ordinary_parking_enabled
from embodied_navigation.localization_mapping.known_scene_hold_guard import (
    retain_native_context,
    remember_native_guard,
    arm_teacher_recovery,
    record_clear_release,
)
from embodied_navigation.control.tracking.cascade_core import Controller
from embodied_navigation.common.input_freshness import (
    enabled as minimal_baseline_enabled,
    pose_cloud_fresh as minimal_pose_cloud_fresh,
    raw_imu_fresh as minimal_raw_imu_fresh,
    wall_stale as minimal_wall_stale,
)
from embodied_navigation.control.tracking.control_core import (
    follow_trajectory,
    trajectory_has_progress,
    RawImuTilt,
)
from embodied_navigation.control.safety.imu_causal_buffer import (
    enabled as imu_buffer_enabled,
    CausalImuBuffer,
)
from scipy.interpolate import BSpline
from embodied_navigation.planning.admission.path_admission import (
    FrozenFence,
    candidate,
    payload,
    replayable,
    certify_curve,
    remaining_certificate,
)
from embodied_navigation.control.safety.clock_hold import ControlClockHold

_sensor_hold_begin = _observer_api("r74_sensor_hold_witness").begin
_sensor_hold_end = _observer_api("r74_sensor_hold_witness").end
_sensor_hold_flush = _observer_api("r74_sensor_hold_witness").flush_once
_sensor_callback_odom = _observer_api("r74_sensor_hold_witness").odom_callback
_sensor_callback_cloud = _observer_api("r74_sensor_hold_witness").cloud_callback
_sensor_parking_protection = _observer_api("r74_sensor_hold_witness").parking_protection

_corrected_delivery_flush = _observer_api("r74_corrected_delivery_witness").flush

_callback_cost_observe = _observer_api("r74_callback_cost_witness").observe
_callback_cost_flush = _observer_api("r74_callback_cost_witness").flush_once
from embodied_navigation.planning.admission.path_handoff import (
    preview as preview_path_handoff,
    context as handoff_context,
    early_request as early_handoff_request,
)

_r85_intake_outer = _observer_api("r85_protocol_intake_trace").outer
from embodied_navigation.planning.reference.global_route_reference import (
    contract as global_reference_contract,
    SegmentGuides,
    StableProjectionController,
    direction_check,
    admission_direction_check,
)


def _plain_json_evidence_graph(value):
    """Accept only builtin JSON containers/scalars for an internal memory clone.

    This is an optimization selector, not a control/source acceptance predicate.
    Unselected types and deep graphs keep the original deepcopy behavior. Memo
    traversal declines repeated containers and cycles, retaining deepcopy for
    those graphs. No objects from an external pickle are read.
    """
    stack = [(value, 0)]
    seen = set()
    while stack:
        item, depth = stack.pop()
        kind = type(item)
        if depth > 64:
            return False
        if item is None or kind is bool or kind is int or kind is str:
            continue
        if kind is float:
            if not math.isfinite(item):
                return False
            continue
        if kind is not dict and kind is not list:
            return False
        identity = id(item)
        if identity in seen:
            return False
        seen.add(identity)
        if kind is dict:
            for key, child in item.items():
                if type(key) is not str:
                    return False
                stack.append((child, depth + 1))
        else:
            stack.extend((child, depth + 1) for child in item)
    return True


def _enqueue_navigation_imu_record(node, row, sequence):
    """Transfer one explicitly owned small record to the existing writer.

    No history row or mutable nested list is borrowed by the writer. Preserve
    original minimal projection. Background JSON failures use writer.error.
    """
    path = node.run / "navigation_imu_history.jsonl"
    writer = node.evidence
    if (
        minimal_baseline_enabled(getattr(node, "profile", {}))
        or not callable(getattr(writer, "enqueue", None))
        or len(row["source_frame_id"]) > 256
    ):
        return writer.append(path, {**row, "sequence": sequence})
    record = dict(
        row,
        sequence=sequence,
        angular_velocity_body=list(row["angular_velocity_body"]),
        gyro_source_xyz=list(row["gyro_source_xyz"]),
        orientation_xyzw=list(row["orientation_xyzw"]),
        orientation_covariance=list(row["orientation_covariance"]),
        imu_to_body_rotation=[list(axis) for axis in row["imu_to_body_rotation"]],
    )

    def write():
        line = json.dumps(record, allow_nan=False, ensure_ascii=False) + "\n"
        with path.open("a") as stream:
            stream.write(line)

    writer.enqueue(write)


def _copy_path_receipt_metadata(value):
    """Return a fresh independent mutable graph; no cache/shared mutable rows.

    The internal C codec preserves builtin types, insertion order and float
    bits. Unknown objects and aliased graphs retain original deepcopy.
    Validation and serialization happen on every call, so a later receipt edit
    is observed on the next row instead of reusing an old serialized snapshot.
    """
    if not _plain_json_evidence_graph(value):
        return copy.deepcopy(value)
    try:
        return pickle.loads(pickle.dumps(value, protocol=4))
    except Exception:
        return copy.deepcopy(value)


def _final_path_receipt_snapshot(receipt):
    """Cheap read-only snapshot of the active path receipt for final_bundle.

    The previous per-waypoint copy.deepcopy(self.path_receipt) was the largest
    single service cost in R97: 149.98 ms span with 142.48 ms gen2 GC nested
    inside, while the copy itself was 4.24/4.77 ms. The control path only ever
    replaces self.path_receipt (accept_spline promotion or the final parking
    rebind) and never mutates a committed receipt in place, so a top-level dict
    copy isolates the bundle from a later replacement without allocating the
    large points_xyz rows or the metadata graph.

    Documented alias: points_xyz/metadata objects are borrowed by reference, so
    an in-place mutation of those nested values would be visible to the bundle.
    No control or downstream consumer mutates them (corridor/path_commit only
    read, and corridor builds a new dict with its own prefix list). Every
    key/value, the original receipt/stamp/hash identity and all downstream
    bindings (control_parking -> corridor/path_commit) are unchanged.
    """
    if receipt is None:
        return None
    if type(receipt) is not dict:
        # Keep the original clone boundary for any non-plain receipt graph.
        return copy.deepcopy(receipt)
    return dict(receipt)


def _pid_receipt_json_bytes(node):
    """Select only current plain values; retain one immutable value-keyed snapshot."""
    if not callable(getattr(node.evidence, "append_pid_snapshot", None)):
        return None
    if node.path_receipt is None:
        return None
    try:
        subset = {k: v for k, v in node.path_receipt.items() if k != "points_xyz"}
        if not _plain_json_evidence_graph(subset):
            return None
        key = pickle.dumps(subset, protocol=4)
        entry = getattr(node, "_pid_receipt_bytes_cache", None)
        if entry is not None and key == entry[0]:
            return entry[1]
        # Encode the key's value, never a later mutable input graph.
        encoded = json.dumps(pickle.loads(key), allow_nan=False, ensure_ascii=False).encode("utf-8")
        node._pid_receipt_bytes_cache = (key, encoded)
        return encoded
    except Exception:
        # The complete original clone/append path still owns unsupported errors.
        return None


def _append_pid_row_evidence(node, row):
    snapshot = getattr(node, "_pid_row_receipt_snapshot", None)
    if snapshot is not None and snapshot[0] is row:
        node.evidence.append_pid_snapshot(
            node.run / "navigation_pid_history.jsonl", row, snapshot[1]
        )
    else:
        node.evidence.append(node.run / "navigation_pid_history.jsonl", row)


# R47 diagnostic observer: no control authority and no hot-path IO.
import sys

_r86_probe_stages = _observer_api("r86_stage_probe").STAGES
_r86_probe_sample = _observer_api("r86_stage_probe").sample
_r86_probe_guard_begin = _observer_api("r86_stage_probe").guard_window_begin
_r86_probe_guard_end = _observer_api("r86_stage_probe").guard_window_end


class _TeacherSubstageTiming:
    FIRST = 2048
    LAST = 1920
    ADMISSION_LAST = 128

    def __init__(self, run, wall_clock=None, cpu_clock=None):
        self.run = run
        self.wall_clock = time.monotonic_ns if wall_clock is None else wall_clock
        self.cpu_clock = time.thread_time_ns if cpu_clock is None else cpu_clock
        self.first = []
        self.last = deque(maxlen=self.LAST)
        self.admission_last = deque(maxlen=self.ADMISSION_LAST)
        self.marker_attempts = 0
        self.markers_recorded = 0
        self.calls_started = 0
        self.calls_ended = 0
        self.error_count = 0
        self.error_prefix = []
        self.last_select = None
        self.flush_attempted = False
        self.flush_status = None
        self.execution_sample_count = 0
        self.execution_sample_span_sum_ns = 0
        self.execution_sample_span_max_ns = 0
        self.execution_sample_errors = 0

    def error(self, where, error):
        try:
            self.error_count += 1
            if len(self.error_prefix) < 8:
                self.error_prefix.append(dict(where=where, error_type=type(error).__name__))
        except Exception:
            pass

    def begin(self, kind, original_ros_ns=None, original_wall_ns=None, stopped=None):
        try:
            self.calls_started += 1
            call = dict(
                call_id=self.calls_started,
                kind=kind,
                pid_sequence=None,
                cascade_sequence=None,
                select_call_id=None,
                publication_sequence=None,
                publication_trigger=None,
                original_publication_entry_wall_ns=None,
                original_ros_clock_ns=original_ros_ns,
                original_compute_wall_ns=original_wall_ns,
                stopped_argument=stopped,
                ended=False,
                exception_type=None,
                valid_markers=0,
                marker_errors=0,
            )
            self.mark(call, "method_entry")
            return call
        except Exception as error:
            self.error("begin", error)
            return None

    def bind_row(self, call, row):
        try:
            if call is None:
                return
            sequence = row.get("sequence") if isinstance(row, dict) else None
            cascade_sequence = row.get("cascade_sequence") if isinstance(row, dict) else None
            call["pid_sequence"] = sequence if type(sequence) is int else None
            call["cascade_sequence"] = cascade_sequence if type(cascade_sequence) is int else None
            call["select_call_id"] = None
            if call["kind"] == "select":
                if call["pid_sequence"] is not None:
                    call["select_call_id"] = call["call_id"]
                    self.last_select = (call["pid_sequence"], call)
            elif (
                call["pid_sequence"] is not None
                and self.last_select is not None
                and call["pid_sequence"] == self.last_select[0]
            ):
                call["select_call_id"] = self.last_select[1]["call_id"]
        except Exception as error:
            self.error("bind_row", error)

    def mark(self, call, stage):
        try:
            self.marker_attempts += 1
            wall_ns = self.wall_clock()
            cpu_ns = self.cpu_clock()
            if type(wall_ns) is not int or type(cpu_ns) is not int:
                raise TypeError("Diagnostic clocks must return integers")
            self.markers_recorded += 1
            row = dict(
                marker_index=self.markers_recorded,
                stage=stage,
                monotonic_wall_ns=wall_ns,
                thread_cpu_ns=cpu_ns,
                call=call,
            )
            # Original clocks stay unchanged. Optional extra evidence may fail
            # independently without losing the original marker or business call.
            if stage in _r86_probe_stages:
                try:
                    detail = _r86_probe_sample()
                    row["r86_execution_sample"] = detail
                    span = detail["wall_end_ns"] - detail["wall_begin_ns"]
                    self.execution_sample_count += 1
                    self.execution_sample_span_sum_ns += span
                    self.execution_sample_span_max_ns = max(self.execution_sample_span_max_ns, span)
                    self.execution_sample_errors += bool(detail["errors"])
                except Exception as error:
                    row["r86_execution_sample_error"] = type(error).__name__
                    self.execution_sample_errors += 1
            if call is not None:
                call["valid_markers"] += 1
            if len(self.first) < self.FIRST:
                self.first.append(row)
            elif call is not None and call.get("kind") == "admission_body":
                self.admission_last.append(row)
            else:
                self.last.append(row)
        except Exception as error:
            if isinstance(call, dict):
                call["marker_errors"] += 1
            self.error("mark:" + stage, error)

    def end(self, call, exception_type=None):
        try:
            if call is None or call["ended"]:
                return
            call["ended"] = True
            call["exception_type"] = None if exception_type is None else exception_type.__name__
            self.calls_ended += 1
            self.mark(call, "method_end")
        except Exception as error:
            self.error("end", error)

    def snapshot(self):
        rows = sorted(
            self.first + list(self.last) + list(self.admission_last),
            key=lambda row: row["marker_index"],
        )
        retained = {}
        for row in rows:
            call = row["call"]
            if call is not None:
                item = retained.setdefault(
                    call["call_id"], dict(call=call, marker_indices=[], stages=[])
                )
                item["marker_indices"].append(row["marker_index"])
                item["stages"].append(row["stage"])
        groups = []
        for item in retained.values():
            call = item["call"]
            item["complete_retained_method_group"] = bool(
                call["ended"]
                and not call["marker_errors"]
                and len(item["marker_indices"]) == call["valid_markers"]
                and item["stages"][0] == "method_entry"
                and item["stages"][-1] == "method_end"
            )
            groups.append(item)
        omitted_ranges = []
        next_index = 1
        for row in rows:
            marker_index = row["marker_index"]
            if marker_index > next_index:
                omitted_ranges.append([next_index, marker_index - 1])
            next_index = marker_index + 1
        if next_index <= self.markers_recorded:
            omitted_ranges.append([next_index, self.markers_recorded])
        omitted = omitted_ranges[0] if len(omitted_ranges) == 1 else None
        return dict(
            schema="teacher_control_substage_timing/v1",
            run=str(self.run),
            diagnostic_only=True,
            control_authority=False,
            business_or_NAV_PASS=False,
            clock_pair_order=["monotonic_wall_ns", "thread_cpu_ns"],
            clock_pair_simultaneous=False,
            original_business_clock_samples_unchanged=True,
            call_metadata_is_finalized_at_shutdown_not_marker_time=True,
            capacity_unit="markers_not_complete_control_rows",
            maximum_retained_markers=self.FIRST + self.LAST + self.ADMISSION_LAST,
            retention_policy="first_then_ordinary_and_admission_reserved_tails/v1",
            first_marker_capacity=self.FIRST,
            last_marker_capacity=self.LAST,
            admission_last_marker_capacity=self.ADMISSION_LAST,
            ordinary_tail_retained_marker_count=len(self.last),
            admission_tail_retained_marker_count=len(self.admission_last),
            marker_attempts=self.marker_attempts,
            markers_recorded=self.markers_recorded,
            retained_marker_count=len(rows),
            dropped_marker_count=self.markers_recorded - len(rows),
            dropped_marker_index_range=omitted,
            dropped_marker_index_ranges=omitted_ranges,
            dropped_marker_index_range_is_exact_only_for_single_gap=True,
            overflowed=self.markers_recorded > len(rows),
            calls_started=self.calls_started,
            calls_ended=self.calls_ended,
            unfinished_call_count=self.calls_started - self.calls_ended,
            error_count=self.error_count,
            error_prefix=self.error_prefix,
            retained_method_groups=groups,
            markers=rows,
            missing_middle_or_partial_groups_must_not_be_interpolated=True,
            thread_CPU_is_not_total_process_CPU=True,
            r86_execution_sampling=dict(
                schema="r86_execution_sampling/v1",
                stages=sorted(_r86_probe_stages),
                sample_count=self.execution_sample_count,
                sample_span_sum_ns=self.execution_sample_span_sum_ns,
                sample_span_max_ns=self.execution_sample_span_max_ns,
                sample_error_count=self.execution_sample_errors,
                sequential_clock_samples=True,
                thread_delta_requires_same_native_tid=True,
                sample_span_excludes_marker_bookkeeping=True,
                no_hot_path_IO=True,
                original_business_clocks_unchanged=True,
            ),
            latency_changes_due_to_instrumentation_not_excluded=True,
        )

    @staticmethod
    def write_exclusive(path, value):
        raw = (json.dumps(value, allow_nan=False, ensure_ascii=False) + "\n").encode()
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        with os.fdopen(fd, "wb") as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())

    def flush_once(self):
        if self.flush_attempted:
            return
        self.flush_attempted = True
        try:
            path = self.run / "teacher_control_substage_timing.json"
            self.flush_status = dict(attempted=True, written=False, error_type=None)
            value = self.snapshot()
            value["flush"] = dict(attempted=True, written=True, exclusive_path=str(path))
            self.write_exclusive(path, value)
            self.flush_status["written"] = True
        except Exception as error:
            self.error("flush", error)
            self.flush_status = dict(attempted=True, written=False, error_type=type(error).__name__)
            try:
                self.write_exclusive(
                    self.run / "teacher_control_substage_timing.flush_error.json",
                    dict(
                        schema="teacher_control_substage_timing_flush_error/v1",
                        run=str(self.run),
                        diagnostic_only=True,
                        business_state_unchanged=True,
                        flush=self.flush_status,
                        error_count=self.error_count,
                        error_prefix=self.error_prefix,
                        markers_recorded=self.markers_recorded,
                        unfinished_call_count=self.calls_started - self.calls_ended,
                        no_retry_or_overwrite=True,
                        main_timing_file_may_be_partial_or_absent=True,
                    ),
                )
            except Exception:
                try:
                    os.write(
                        2,
                        b"R47_DIAGNOSTIC_TIMING_FLUSH_FAILED; business state unchanged; no retry\n",
                    )
                except Exception:
                    pass


def _r47_initialize(node, run):
    profile = getattr(node, "profile", {})
    timing_selected = False
    if "r74_teacher_substage_timing" in profile:
        from embodied_navigation.common.observation import api as _observer_api

        timing_enabled = _observer_api("r74_teacher_substage_timing").enabled
        timing_selected = timing_enabled(profile)
    if minimal_baseline_enabled(getattr(node, "profile", {})) and not timing_selected:
        node._r47_timing = None
        node._r47_timing_intentionally_disabled = True
        return
    if timing_selected:
        node._r47_timing_intentionally_disabled = False
    try:
        node._r47_timing = _TeacherSubstageTiming(run)
    except Exception as error:
        node._r47_timing = None
        node._r47_timing_init_error = type(error).__name__


def _r47_begin(node, kind, ns=None, wall=None, stopped=None):
    if getattr(node, "_r47_timing_intentionally_disabled", False):
        return None
    try:
        call = node._r47_timing.begin(kind, ns, wall, stopped)
        if kind == "select" and call is not None:
            feedback = getattr(node, "feedback", None)
            imu = getattr(node, "paired_imu", None)
            call["feedback_received_wall_ns_at_select_entry"] = (
                feedback.get("received_wall_ns") if isinstance(feedback, dict) else None
            )
            call["paired_IMU_received_wall_ns_at_select_entry"] = (
                imu.get("received_wall_ns") if isinstance(imu, dict) else None
            )
        return call
    except Exception:
        return None


def _r47_guard_dimensions(node, call, args):
    try:
        if call is not None:
            call["actual_cloud_count_at_guard_entry"] = len(args[0])
            call["actual_route_count_at_guard_entry"] = len(args[4])
    except Exception as error:
        try:
            node._r47_timing.error("guard_dimensions", error)
        except Exception:
            pass


def _r47_publication_anchor(node, call, before=None, completed=False):
    try:
        if call is not None:
            if before is not None:
                call["publication_trigger"] = before["trigger"]
                call["original_publication_entry_wall_ns"] = before["entry_monotonic_wall_ns"]
            if completed:
                call["publication_sequence"] = node.publication_records
    except Exception as error:
        try:
            node._r47_timing.error("publication_anchor", error)
        except Exception:
            pass


def _r47_mark(node, call, stage):
    if stage in _r86_probe_stages:
        _r87_diag.mark("control." + stage, source_ns=getattr(node, "pose_stamp", None))
    if call is None:
        return
    try:
        node._r47_timing.mark(call, stage)
    except Exception:
        pass


def _r47_bind(node, call, row):
    if call is None:
        return
    try:
        node._r47_timing.bind_row(call, row)
    except Exception:
        pass


def _r47_end(node, call):
    if call is None:
        return
    try:
        node._r47_timing.end(call, sys.exc_info()[0])
    except Exception:
        pass


def _r56_observe_admission_body(original):
    """Two bounded diagnostic markers around one unchanged admission call."""

    def observed(self, *args, **kwargs):
        call = _r47_begin(self, "admission_body")
        try:
            if call is not None:
                call["entry_native_thread_id"] = threading.get_native_id()
        except Exception:
            pass
        try:
            return original(self, *args, **kwargs)
        finally:
            try:
                if call is not None:
                    native_thread_id = threading.get_native_id()
                    call["exit_native_thread_id"] = native_thread_id
                    call["same_native_thread"] = (
                        call.get("entry_native_thread_id") == native_thread_id
                    )
            except Exception:
                pass
            _r47_end(self, call)

    observed.__wrapped__ = original
    return observed


def _r47_flush(node):
    if getattr(node, "_r47_timing_intentionally_disabled", False):
        return
    try:
        observer = getattr(node, "_r47_timing", None)
        if observer is None:
            _TeacherSubstageTiming.write_exclusive(
                node.run / "teacher_control_substage_timing.flush_error.json",
                dict(
                    schema="teacher_control_substage_timing_flush_error/v1",
                    diagnostic_only=True,
                    business_state_unchanged=True,
                    observer_initialization_failed=True,
                    error_type=getattr(node, "_r47_timing_init_error", "observer_missing"),
                    no_retry_or_overwrite=True,
                ),
            )
        else:
            observer.flush_once()
    except Exception:
        try:
            os.write(2, b"R47_DIAGNOSTIC_TIMING_OBSERVER_UNAVAILABLE; no control change\n")
        except Exception:
            pass


def _pre_turn_external_hold_seed_wait(
    previous, cascade, gate, context, mode, guard, diagnostic, command, clock_ns, wall_ns
):
    """Only the first validated source seed after our ordinary pre-turn hold.

    This recognizes a zero recovery output, never a control/stop qualification.
    All source, attitude and causal ACK checks have already run in core.update.
    """
    if not (
        isinstance(previous, dict)
        and previous.get("cascade") is cascade
        and previous.get("context") == context
        and previous.get("uninterrupted") is True
        and mode == "brake"
        and guard is None
        and gate.phase == "settle"
        and getattr(gate, "controlled_braking", False)
        and getattr(gate, "handoff_active", False)
        and getattr(gate, "handoff_failure", None) is None
    ):
        return False
    event = getattr(gate, "handoff_event", None)
    if not (
        isinstance(event, dict)
        and event.get("status") == "pending"
        and event.get("brake_entry_control_s") == clock_ns / 1e9
        and event.get("context", {}).get("request_id") == context[0]
        and event.get("context", {}).get("waypoint_index") == context[1]
        and event.get("context") == previous.get("gate_context") == gate.transition_context
        and event.get("purpose") == previous.get("purpose") == gate.transition_purpose
        and gate.handoff_deadline_s is not None
        and clock_ns / 1e9 <= gate.handoff_deadline_s
        and 0 <= clock_ns - previous["clock_ns"] <= 300_000_000
        and 0 <= wall_ns - previous["wall_ns"] <= 300_000_000
    ):
        return False
    stamp = diagnostic.get("feedback_pose_stamp_ns")
    return bool(
        diagnostic.get("mode") == "recovering"
        and diagnostic.get("reason") == "external_hold"
        and diagnostic.get("kind") == "recovery_source_seed"
        and diagnostic.get("history_kind") == "seed"
        and diagnostic.get("math_step_executed") is False
        and diagnostic.get("controller_updated") is False
        and diagnostic.get("failure_latched") is None
        and diagnostic.get("protection_active") is True
        and diagnostic.get("header_dt_s") == 0.0
        and diagnostic.get("integral_dt_s") == 0.0
        and cascade.protected is True
        and cascade.protection_reason == "external_hold"
        and cascade.recovery == [stamp]
        and cascade.last_seen_pose_stamp == stamp
        and cascade.last_math_stamp == stamp
        and cascade.last_math_wall == wall_ns
        and np.asarray(command).shape == (3,)
        and np.array_equal(command, np.zeros(3))
        and diagnostic.get("command_body") == [0.0, 0.0, 0.0]
    )


def _r67_capture_known_frame_false(node, now, d, s, e, control_frame, control_candidate_code):
    # One pending raw-scalar snapshot, only the actual control predicate caller.
    if control_frame.f_code is not control_candidate_code or getattr(
        node, "_r67_known_frame_failure_attached", False
    ):
        return
    gate = getattr(node, "heading_gate", None)
    if (
        node.state != "running"
        or not getattr(gate, "handoff_active", False)
        or getattr(gate, "handoff_failure", None) is not None
    ):
        return

    def scalar(value):
        return (
            value
            if (
                value is None
                or type(value) in (bool, int, str)
                or type(value) is float
                and math.isfinite(value)
            )
            else None
        )

    def field(value, key):
        return scalar(value.get(key)) if type(value) is dict else None

    commit = d.get("source_commit") if type(d) is dict else None
    snapshot = dict(
        schema="known_scene_final_false_raw_snapshot/v1",
        predicate_now_monotonic_wall=scalar(now),
        d_present=d is not None,
        s_present=s is not None,
        e_present=e is not None,
        d_received_monotonic_wall=field(d, "received_monotonic_wall"),
        s_received_monotonic_wall=field(s, "received_monotonic_wall"),
        e_received_monotonic_wall=field(e, "received_monotonic_wall"),
        d_generation=field(d, "generation"),
        s_generation=field(s, "generation"),
        e_generation=field(e, "generation"),
        d_control_allowed=field(d, "control_allowed"),
        d_correction_stale=field(d, "correction_stale"),
        d_correction_transition=field(d, "correction_transition"),
        s_correction_transition=field(s, "correction_transition"),
        s_initialized=field(s, "initialized"),
        e_ready=field(e, "ready"),
        explicit_message_commit_token=field(commit, "explicit_message_commit_token"),
        pose_stamp=scalar(node.pose_stamp),
        cloud_stamp=scalar(node.cloud_stamp),
        d_source_ns=field(d, "source_ns"),
    )
    node._r67_known_frame_failure_pending = (control_frame, snapshot)


def _r67_attach_known_frame_failure(node, call, before):
    try:
        pending = getattr(node, "_r67_known_frame_failure_pending", None)
        if pending is None or getattr(node, "_r67_known_frame_failure_attached", False):
            return
        # Keep the actual originating frame alive: neither a reused frame id
        # nor another invocation of the same code can claim this snapshot.
        node._r67_known_frame_failure_pending = None
        control_frame, snapshot = pending
        caller = sys._getframe(1)
        while caller is not None and caller is not control_frame:
            caller = caller.f_back
        if caller is None or before.get("state_before") != "running" or node.state != "failed":
            return
        node._r67_known_frame_failure_attached = True
        if type(call) is dict:
            call["known_frame_final_false"] = snapshot
    except Exception:
        pass


def _r67_observe_known_generation_ready(original, control_candidate_code):
    # Observe the actual selected predicate; never duplicate its business rules.
    time = original.__globals__["time"]

    def observed(self):
        result = original(self)
        if result:
            return result
        try:
            previous = getattr(self, "_r67_known_frame_failure_pending", None)
            _r67_capture_known_frame_false(
                self,
                time.monotonic(),
                self.known_scene_inputs,
                self.known_scene_status,
                self.known_scene_scan_epoch,
                sys._getframe(1),
                control_candidate_code,
            )
            pending = getattr(self, "_r67_known_frame_failure_pending", None)
            if pending is not None and pending is not previous:
                pending[1]["clock_read_boundary"] = "after_original_predicate_return"
        except Exception:
            pass
        return result

    observed.__wrapped__ = original
    return observed


def observe_business_gate(node, gate, decision, reasons):
    """Diagnostic execution permission; original refusal remains in evidence."""
    config = node.profile.get("diagnostic_observe_only", {})
    if (
        decision.get("allowed") is not False
        or decision.get("reason") not in reasons
        or not isinstance(config, dict)
        or type(config.get("enabled")) is not bool
        or config["enabled"] is not True
    ):
        return decision
    # Validate before the deduplicating ledger can return early. Only fresh
    # actual finite same-generation inputs can consume a business observation.
    json.dumps(decision, allow_nan=False)
    pose = np.asarray(node.pose, dtype=float)
    rotation = np.asarray(node.rotation, dtype=float)
    if (
        pose.shape != (3,)
        or rotation.shape != (3, 3)
        or not np.isfinite(pose).all()
        or not np.isfinite(rotation).all()
        or not node._known_generation_ready()
    ):
        return decision
    from embodied_navigation.common.observation import api as _observer_api

    record = _observer_api("diagnostic_observation").record
    if record(node.run, node.profile, gate, str(decision["reason"]), decision) is not True:
        raise RuntimeError("business_gate_observation_not_recorded")
    return dict(
        decision,
        allowed=True,
        diagnostic_original_allowed=False,
        diagnostic_original_reason=decision["reason"],
        diagnostic_observe_only=True,
        diagnostic_execution_continued=True,
        navigation_PASS=False,
    )


def finite_retention_reference(node, decision):
    """Observe an original finite reference score, never a bad preview/source."""
    try:
        reference = decision["reference"]
        cascade = node.cascade
        path = np.asarray(cascade.path, dtype=float)
        cumulative = np.asarray(cascade.cumulative, dtype=float)
        projection = reference["raw_projection"]
        segment = projection["segment"]
        if (
            path.ndim != 2
            or path.shape[1] != 3
            or not 2 <= len(path) <= 2000
            or not np.isfinite(path).all()
            or cumulative.shape != (len(path),)
            or not np.isfinite(cumulative).all()
            or not np.all(np.diff(cumulative) > 0)
            or type(segment) is not int
            or not 0 <= segment < len(path) - 1
            or reference["path_id"] != cascade.path_id
            or reference["path_sha256"] != cascade.path_sha
            or not node._known_generation_ready()
        ):
            return False

        def finite_tree(value):
            if isinstance(value, dict):
                return all(finite_tree(x) for x in value.values())
            if isinstance(value, (list, tuple, np.ndarray)):
                return all(finite_tree(x) for x in value)
            if isinstance(value, (float, np.floating)):
                return math.isfinite(float(value))
            return True

        return finite_tree(reference) and finite_tree(decision.get("direction_coherence"))
    except (AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
        return False


def diagnostic_profile_view(profile):
    """The observer only consumes its explicit boolean; never copy the full profile."""
    config = profile.get("diagnostic_observe_only", {})
    return dict(
        diagnostic_observe_only=dict(
            enabled=bool(
                isinstance(config, dict)
                and type(config.get("enabled")) is bool
                and config["enabled"]
            )
        )
    )


def admission_callback_rejected(run, profile, elapsed_s, reason):
    """100ms for one synchronous parent admission stage; diagnostic policy unchanged."""
    if type(elapsed_s) not in (int, float) or not math.isfinite(elapsed_s) or elapsed_s < 0:
        raise ValueError("admission_callback_elapsed_invalid")
    if elapsed_s <= 0.100:
        return False
    minimal = diagnostic_profile_view(profile)
    if not minimal["diagnostic_observe_only"]["enabled"]:
        return True
    from embodied_navigation.common.observation import api as _observer_api

    record = _observer_api("diagnostic_observation").record
    record(
        run,
        minimal,
        "admission_callback_elapsed",
        reason,
        dict(
            callback_elapsed_wall_ms=elapsed_s * 1000,
            original_limit_wall_ms=100.0,
            original_timing_check_passed=False,
            diagnostic_execution_continued=True,
            source_freshness_checks_unchanged=True,
            navigation_PASS=False,
        ),
    )
    return False


def _admission_metadata_fragment(metadata, receipt, receipt_plain):
    # Both graphs must be typed before pickle, then exact value bytes agree.
    if (
        receipt_plain
        and type(receipt) is dict
        and "metadata" in receipt
        and _plain_json_evidence_graph(metadata)
        and pickle.dumps(metadata, protocol=4) == pickle.dumps(receipt["metadata"], protocol=4)
    ):
        return json.dumps(metadata, allow_nan=False, ensure_ascii=False)
    return None


def _receipt_json_with_metadata_fragment(receipt, fragment):
    return (
        "{"
        + ", ".join(
            json.dumps(k, allow_nan=False, ensure_ascii=False)
            + ": "
            + (fragment if k == "metadata" else json.dumps(v, allow_nan=False, ensure_ascii=False))
            for k, v in receipt.items()
        )
        + "}"
    )


class ResetBookkeeping:
    """Legacy archive reset counter only; cascade_core owns all control maths."""

    def __init__(self):
        self.resets = 0
        self.reset("startup")

    def reset(self, reason):
        self.resets += 1
        self.reset_reason = reason


from embodied_navigation.navigation.mission.goal_regions import ArrivalWindow
from embodied_navigation.navigation.mission.continuous_route import (
    configuration as continuous_configuration,
    RouteProgress,
    CausalHandoffProjectionController,
)


from embodied_navigation.planning.admission.path_commit import staged_cascade_binding
from embodied_navigation.planning.admission.pending_path_commit import (
    defer_future as defer_future_path,
    commit_ready as commit_pending_path,
    waiting as pending_path_waiting,
)
from embodied_navigation.control.parking.navigation_stop_window import (
    reset_transition,
    bind_transition_context,
)


