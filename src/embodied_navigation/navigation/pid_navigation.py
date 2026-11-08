#!/usr/bin/env python3
"""Real SLAM/IMU geometric cascade, protected by original SCAN cloud guards."""
import argparse,copy,hashlib,json,math,os,time,re,threading
import pickle
from embodied_navigation.control.parking.causal_join import PARKING_CAUSAL_JOIN_ENABLED
from embodied_navigation.control.parking.terminal_owner import TERMINAL_PARKING_OWNER_ENABLED
import embodied_navigation.control.parking.terminal_owner as _r105_terminal
from embodied_navigation.common.observation import api as _observer_api; _r87_diag = _observer_api('r87_diagnostic_probe')
from embodied_navigation.common.observation import api as _observer_api; _r96_cpu = _observer_api('r96_control_cpu_probe')
from embodied_navigation.common.observation import api as _observer_api; _r102_progress = _observer_api('r102_progress_probe')
from embodied_navigation.common.observation import api as _observer_api; _r99_clock_probe = _observer_api('r99_clock_probe')
from embodied_navigation.common.observation import api as _observer_api; _r118_source_age = _observer_api('r118_source_age_probe')
import embodied_navigation.common.time.clock_service as _r114_clock_service
from embodied_navigation.common.observation import api as _observer_api; _r107_scan_take_probe = _observer_api('r107_scan_take_probe')
from embodied_navigation.common.time.clock_service import CLOCK_SERVICE_ENABLED
from collections import deque,OrderedDict
from pathlib import Path
from embodied_navigation.control.safety.handoff_stop_policy import tail_stop_request
from embodied_navigation.control.safety.launch_command_policy import constrain_exactzero_launch
from embodied_navigation.planning.admission.path_handoff import stopped_rebind as stopped_path_rebind, retention_reference_eligibility
import numpy as np
from embodied_navigation.navigation.ros_node import TeacherNavigation
from embodied_navigation.control.tracking.pid_core import slew
from embodied_navigation.control.parking.parking_zero_contract import make_event
from embodied_navigation.control.parking.parking_contract import enabled as ordinary_parking_enabled
from embodied_navigation.localization_mapping.known_scene_hold_guard import retain_native_context, remember_native_guard, arm_teacher_recovery, record_clear_release
from embodied_navigation.control.tracking.cascade_core import Controller
from embodied_navigation.common.input_freshness import enabled as minimal_baseline_enabled, pose_cloud_fresh as minimal_pose_cloud_fresh, raw_imu_fresh as minimal_raw_imu_fresh, wall_stale as minimal_wall_stale
from embodied_navigation.control.tracking.control_core import follow_trajectory, trajectory_has_progress, RawImuTilt
from embodied_navigation.control.safety.imu_causal_buffer import enabled as imu_buffer_enabled, CausalImuBuffer
from scipy.interpolate import BSpline
from embodied_navigation.planning.admission.path_admission import FrozenFence, candidate, payload, replayable, certify_curve, remaining_certificate
from embodied_navigation.control.safety.clock_hold import ControlClockHold
from embodied_navigation.common.observation import api as _observer_api; _sensor_hold_begin = _observer_api('r74_sensor_hold_witness').begin; _sensor_hold_end = _observer_api('r74_sensor_hold_witness').end; _sensor_hold_flush = _observer_api('r74_sensor_hold_witness').flush_once; _sensor_callback_odom = _observer_api('r74_sensor_hold_witness').odom_callback; _sensor_callback_cloud = _observer_api('r74_sensor_hold_witness').cloud_callback; _sensor_parking_protection = _observer_api('r74_sensor_hold_witness').parking_protection
from embodied_navigation.common.observation import api as _observer_api; _corrected_delivery_flush = _observer_api('r74_corrected_delivery_witness').flush
from embodied_navigation.common.observation import api as _observer_api; _callback_cost_observe = _observer_api('r74_callback_cost_witness').observe; _callback_cost_flush = _observer_api('r74_callback_cost_witness').flush_once
from embodied_navigation.planning.admission.path_handoff import preview as preview_path_handoff, context as handoff_context, early_request as early_handoff_request
from embodied_navigation.common.observation import api as _observer_api; _r85_intake_outer = _observer_api('r85_protocol_intake_trace').outer
from embodied_navigation.planning.reference.global_route_reference import contract as global_reference_contract, SegmentGuides, StableProjectionController, direction_check, admission_direction_check


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


def _enqueue_navigation_imu_record(node,row,sequence):
    """Transfer one explicitly owned small record to the existing writer.

    No history row or mutable nested list is borrowed by the writer. Preserve
    original minimal projection. Background JSON failures use writer.error.
    """
    path=node.run/'navigation_imu_history.jsonl'
    writer=node.evidence
    if (minimal_baseline_enabled(getattr(node,'profile',{})) or
            not callable(getattr(writer,'enqueue',None)) or
            len(row['source_frame_id'])>256):
        return writer.append(path,{**row,'sequence':sequence})
    record=dict(row,sequence=sequence,
        angular_velocity_body=list(row['angular_velocity_body']),
        gyro_source_xyz=list(row['gyro_source_xyz']),
        orientation_xyzw=list(row['orientation_xyzw']),
        orientation_covariance=list(row['orientation_covariance']),
        imu_to_body_rotation=[list(axis)for axis in row['imu_to_body_rotation']])
    def write():
        line=json.dumps(record,allow_nan=False,ensure_ascii=False)+'\n'
        with path.open('a')as stream:stream.write(line)
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
    if not callable(getattr(node.evidence, 'append_pid_snapshot', None)):
        return None
    if node.path_receipt is None:
        return None
    try:
        subset = {k:v for k,v in node.path_receipt.items() if k != 'points_xyz'}
        if not _plain_json_evidence_graph(subset):
            return None
        key = pickle.dumps(subset, protocol=4)
        entry = getattr(node, '_pid_receipt_bytes_cache', None)
        if entry is not None and key == entry[0]:
            return entry[1]
        # Encode the key's value, never a later mutable input graph.
        encoded = json.dumps(pickle.loads(key), allow_nan=False, ensure_ascii=False).encode('utf-8')
        node._pid_receipt_bytes_cache = (key, encoded)
        return encoded
    except Exception:
        # The complete original clone/append path still owns unsupported errors.
        return None


def _append_pid_row_evidence(node, row):
    snapshot = getattr(node, '_pid_row_receipt_snapshot', None)
    if snapshot is not None and snapshot[0] is row:
        node.evidence.append_pid_snapshot(node.run/'navigation_pid_history.jsonl', row, snapshot[1])
    else:
        node.evidence.append(node.run/'navigation_pid_history.jsonl', row)


# R47 diagnostic observer: no control authority and no hot-path IO.
import sys
from embodied_navigation.common.observation import api as _observer_api; _r86_probe_stages = _observer_api('r86_stage_probe').STAGES; _r86_probe_sample = _observer_api('r86_stage_probe').sample; _r86_probe_guard_begin = _observer_api('r86_stage_probe').guard_window_begin; _r86_probe_guard_end = _observer_api('r86_stage_probe').guard_window_end


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
            call = dict(call_id=self.calls_started, kind=kind, pid_sequence=None,
                        cascade_sequence=None, select_call_id=None,
                        publication_sequence=None, publication_trigger=None,
                        original_publication_entry_wall_ns=None,
                        original_ros_clock_ns=original_ros_ns,
                        original_compute_wall_ns=original_wall_ns,
                        stopped_argument=stopped, ended=False,
                        exception_type=None, valid_markers=0, marker_errors=0)
            self.mark(call, 'method_entry')
            return call
        except Exception as error:
            self.error('begin', error)
            return None

    def bind_row(self, call, row):
        try:
            if call is None:
                return
            sequence = row.get('sequence') if isinstance(row, dict) else None
            cascade_sequence = row.get('cascade_sequence') if isinstance(row, dict) else None
            call['pid_sequence'] = sequence if type(sequence) is int else None
            call['cascade_sequence'] = cascade_sequence if type(cascade_sequence) is int else None
            call['select_call_id'] = None
            if call['kind'] == 'select':
                if call['pid_sequence'] is not None:
                    call['select_call_id'] = call['call_id']
                    self.last_select = (call['pid_sequence'], call)
            elif (call['pid_sequence'] is not None and self.last_select is not None
                  and call['pid_sequence'] == self.last_select[0]):
                call['select_call_id'] = self.last_select[1]['call_id']
        except Exception as error:
            self.error('bind_row', error)

    def mark(self, call, stage):
        try:
            self.marker_attempts += 1
            wall_ns = self.wall_clock()
            cpu_ns = self.cpu_clock()
            if type(wall_ns) is not int or type(cpu_ns) is not int:
                raise TypeError('Diagnostic clocks must return integers')
            self.markers_recorded += 1
            row = dict(marker_index=self.markers_recorded, stage=stage,
                       monotonic_wall_ns=wall_ns, thread_cpu_ns=cpu_ns, call=call)
            # Original clocks stay unchanged. Optional extra evidence may fail
            # independently without losing the original marker or business call.
            if stage in _r86_probe_stages:
                try:
                    detail = _r86_probe_sample()
                    row['r86_execution_sample'] = detail
                    span = detail['wall_end_ns'] - detail['wall_begin_ns']
                    self.execution_sample_count += 1
                    self.execution_sample_span_sum_ns += span
                    self.execution_sample_span_max_ns = max(self.execution_sample_span_max_ns, span)
                    self.execution_sample_errors += bool(detail['errors'])
                except Exception as error:
                    row['r86_execution_sample_error'] = type(error).__name__
                    self.execution_sample_errors += 1
            if call is not None:
                call['valid_markers'] += 1
            if len(self.first) < self.FIRST:
                self.first.append(row)
            elif call is not None and call.get('kind') == 'admission_body':
                self.admission_last.append(row)
            else:
                self.last.append(row)
        except Exception as error:
            if isinstance(call, dict):
                call['marker_errors'] += 1
            self.error('mark:' + stage, error)

    def end(self, call, exception_type=None):
        try:
            if call is None or call['ended']:
                return
            call['ended'] = True
            call['exception_type'] = None if exception_type is None else exception_type.__name__
            self.calls_ended += 1
            self.mark(call, 'method_end')
        except Exception as error:
            self.error('end', error)

    def snapshot(self):
        rows = sorted(self.first + list(self.last) + list(self.admission_last),
                      key=lambda row: row['marker_index'])
        retained = {}
        for row in rows:
            call = row['call']
            if call is not None:
                item = retained.setdefault(call['call_id'], dict(call=call, marker_indices=[], stages=[]))
                item['marker_indices'].append(row['marker_index'])
                item['stages'].append(row['stage'])
        groups = []
        for item in retained.values():
            call = item['call']
            item['complete_retained_method_group'] = bool(call['ended'] and not call['marker_errors']
                and len(item['marker_indices']) == call['valid_markers']
                and item['stages'][0] == 'method_entry' and item['stages'][-1] == 'method_end')
            groups.append(item)
        omitted_ranges = []
        next_index = 1
        for row in rows:
            marker_index = row['marker_index']
            if marker_index > next_index:
                omitted_ranges.append([next_index, marker_index - 1])
            next_index = marker_index + 1
        if next_index <= self.markers_recorded:
            omitted_ranges.append([next_index, self.markers_recorded])
        omitted = omitted_ranges[0] if len(omitted_ranges) == 1 else None
        return dict(schema='teacher_control_substage_timing/v1', run=str(self.run),
            diagnostic_only=True, control_authority=False, business_or_NAV_PASS=False,
            clock_pair_order=['monotonic_wall_ns', 'thread_cpu_ns'], clock_pair_simultaneous=False,
            original_business_clock_samples_unchanged=True,
            call_metadata_is_finalized_at_shutdown_not_marker_time=True,
            capacity_unit='markers_not_complete_control_rows',
            maximum_retained_markers=self.FIRST+self.LAST+self.ADMISSION_LAST,
            retention_policy='first_then_ordinary_and_admission_reserved_tails/v1',
            first_marker_capacity=self.FIRST, last_marker_capacity=self.LAST,
            admission_last_marker_capacity=self.ADMISSION_LAST,
            ordinary_tail_retained_marker_count=len(self.last),
            admission_tail_retained_marker_count=len(self.admission_last),
            marker_attempts=self.marker_attempts, markers_recorded=self.markers_recorded,
            retained_marker_count=len(rows), dropped_marker_count=self.markers_recorded-len(rows),
            dropped_marker_index_range=omitted, dropped_marker_index_ranges=omitted_ranges,
            dropped_marker_index_range_is_exact_only_for_single_gap=True,
            overflowed=self.markers_recorded>len(rows),
            calls_started=self.calls_started, calls_ended=self.calls_ended,
            unfinished_call_count=self.calls_started-self.calls_ended,
            error_count=self.error_count, error_prefix=self.error_prefix,
            retained_method_groups=groups, markers=rows,
            missing_middle_or_partial_groups_must_not_be_interpolated=True,
            thread_CPU_is_not_total_process_CPU=True,
            r86_execution_sampling=dict(schema='r86_execution_sampling/v1',
                stages=sorted(_r86_probe_stages), sample_count=self.execution_sample_count,
                sample_span_sum_ns=self.execution_sample_span_sum_ns,
                sample_span_max_ns=self.execution_sample_span_max_ns,
                sample_error_count=self.execution_sample_errors,
                sequential_clock_samples=True, thread_delta_requires_same_native_tid=True,
                sample_span_excludes_marker_bookkeeping=True,
                no_hot_path_IO=True, original_business_clocks_unchanged=True),
            latency_changes_due_to_instrumentation_not_excluded=True)

    @staticmethod
    def write_exclusive(path, value):
        raw = (json.dumps(value, allow_nan=False, ensure_ascii=False) + '\n').encode()
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        with os.fdopen(fd, 'wb') as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())

    def flush_once(self):
        if self.flush_attempted:
            return
        self.flush_attempted = True
        try:
            path = self.run / 'teacher_control_substage_timing.json'
            self.flush_status = dict(attempted=True, written=False, error_type=None)
            value = self.snapshot()
            value['flush'] = dict(attempted=True, written=True, exclusive_path=str(path))
            self.write_exclusive(path, value)
            self.flush_status['written'] = True
        except Exception as error:
            self.error('flush', error)
            self.flush_status = dict(attempted=True, written=False, error_type=type(error).__name__)
            try:
                self.write_exclusive(self.run / 'teacher_control_substage_timing.flush_error.json', dict(
                    schema='teacher_control_substage_timing_flush_error/v1', run=str(self.run),
                    diagnostic_only=True, business_state_unchanged=True, flush=self.flush_status,
                    error_count=self.error_count, error_prefix=self.error_prefix,
                    markers_recorded=self.markers_recorded, unfinished_call_count=self.calls_started-self.calls_ended,
                    no_retry_or_overwrite=True, main_timing_file_may_be_partial_or_absent=True))
            except Exception:
                try:
                    os.write(2, b'R47_DIAGNOSTIC_TIMING_FLUSH_FAILED; business state unchanged; no retry\n')
                except Exception:
                    pass


def _r47_initialize(node, run):
    profile = getattr(node, 'profile', {})
    timing_selected = False
    if 'r74_teacher_substage_timing' in profile:
        from embodied_navigation.common.observation import api as _observer_api; timing_enabled = _observer_api('r74_teacher_substage_timing').enabled
        timing_selected = timing_enabled(profile)
    if minimal_baseline_enabled(getattr(node, 'profile', {})) and not timing_selected:
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
    if getattr(node, '_r47_timing_intentionally_disabled', False):return None
    try:
        call = node._r47_timing.begin(kind, ns, wall, stopped)
        if kind == 'select' and call is not None:
            feedback = getattr(node, 'feedback', None)
            imu = getattr(node, 'paired_imu', None)
            call['feedback_received_wall_ns_at_select_entry'] = feedback.get('received_wall_ns') if isinstance(feedback, dict) else None
            call['paired_IMU_received_wall_ns_at_select_entry'] = imu.get('received_wall_ns') if isinstance(imu, dict) else None
        return call
    except Exception:
        return None


def _r47_guard_dimensions(node, call, args):
    try:
        if call is not None:
            call['actual_cloud_count_at_guard_entry'] = len(args[0])
            call['actual_route_count_at_guard_entry'] = len(args[4])
    except Exception as error:
        try:
            node._r47_timing.error('guard_dimensions', error)
        except Exception:
            pass


def _r47_publication_anchor(node, call, before=None, completed=False):
    try:
        if call is not None:
            if before is not None:
                call['publication_trigger'] = before['trigger']
                call['original_publication_entry_wall_ns'] = before['entry_monotonic_wall_ns']
            if completed:
                call['publication_sequence'] = node.publication_records
    except Exception as error:
        try:
            node._r47_timing.error('publication_anchor', error)
        except Exception:
            pass


def _r47_mark(node, call, stage):
    if stage in _r86_probe_stages:
        _r87_diag.mark('control.'+stage,source_ns=getattr(node,'pose_stamp',None))
    if call is None:return
    try:
        node._r47_timing.mark(call, stage)
    except Exception:
        pass


def _r47_bind(node, call, row):
    if call is None:return
    try:
        node._r47_timing.bind_row(call, row)
    except Exception:
        pass


def _r47_end(node, call):
    if call is None:return
    try:
        node._r47_timing.end(call, sys.exc_info()[0])
    except Exception:
        pass



def _r56_observe_admission_body(original):
    """Two bounded diagnostic markers around one unchanged admission call."""
    def observed(self, *args, **kwargs):
        call = _r47_begin(self, 'admission_body')
        try:
            if call is not None:
                call['entry_native_thread_id'] = threading.get_native_id()
        except Exception:
            pass
        try:
            return original(self, *args, **kwargs)
        finally:
            try:
                if call is not None:
                    native_thread_id = threading.get_native_id()
                    call['exit_native_thread_id'] = native_thread_id
                    call['same_native_thread'] = call.get('entry_native_thread_id') == native_thread_id
            except Exception:
                pass
            _r47_end(self, call)
    observed.__wrapped__ = original
    return observed


def _r47_flush(node):
    if getattr(node, '_r47_timing_intentionally_disabled', False):return
    try:
        observer = getattr(node, '_r47_timing', None)
        if observer is None:
            _TeacherSubstageTiming.write_exclusive(node.run / 'teacher_control_substage_timing.flush_error.json', dict(
                schema='teacher_control_substage_timing_flush_error/v1', diagnostic_only=True,
                business_state_unchanged=True, observer_initialization_failed=True,
                error_type=getattr(node, '_r47_timing_init_error', 'observer_missing'), no_retry_or_overwrite=True))
        else:
            observer.flush_once()
    except Exception:
        try:
            os.write(2, b'R47_DIAGNOSTIC_TIMING_OBSERVER_UNAVAILABLE; no control change\n')
        except Exception:
            pass


def _pre_turn_external_hold_seed_wait(previous,cascade,gate,context,mode,guard,diagnostic,command,clock_ns,wall_ns):
    """Only the first validated source seed after our ordinary pre-turn hold.

    This recognizes a zero recovery output, never a control/stop qualification.
    All source, attitude and causal ACK checks have already run in core.update.
    """
    if not (isinstance(previous,dict) and previous.get('cascade')is cascade
            and previous.get('context')==context and previous.get('uninterrupted')is True
            and mode=='brake' and guard is None and gate.phase=='settle'
            and getattr(gate,'controlled_braking',False) and getattr(gate,'handoff_active',False)
            and getattr(gate,'handoff_failure',None)is None):return False
    event=getattr(gate,'handoff_event',None)
    if not (isinstance(event,dict) and event.get('status')=='pending'
            and event.get('brake_entry_control_s')==clock_ns/1e9
            and event.get('context',{}).get('request_id')==context[0]
            and event.get('context',{}).get('waypoint_index')==context[1]
            and event.get('context')==previous.get('gate_context')==gate.transition_context
            and event.get('purpose')==previous.get('purpose')==gate.transition_purpose
            and gate.handoff_deadline_s is not None and clock_ns/1e9<=gate.handoff_deadline_s
            and 0<=clock_ns-previous['clock_ns']<=300_000_000
            and 0<=wall_ns-previous['wall_ns']<=300_000_000):return False
    stamp=diagnostic.get('feedback_pose_stamp_ns')
    return bool(diagnostic.get('mode')=='recovering' and diagnostic.get('reason')=='external_hold'
        and diagnostic.get('kind')=='recovery_source_seed' and diagnostic.get('history_kind')=='seed'
        and diagnostic.get('math_step_executed')is False and diagnostic.get('controller_updated')is False
        and diagnostic.get('failure_latched')is None and diagnostic.get('protection_active')is True
        and diagnostic.get('header_dt_s')==0. and diagnostic.get('integral_dt_s')==0.
        and cascade.protected is True and cascade.protection_reason=='external_hold'
        and cascade.recovery==[stamp] and cascade.last_seen_pose_stamp==stamp
        and cascade.last_math_stamp==stamp and cascade.last_math_wall==wall_ns
        and np.asarray(command).shape==(3,) and np.array_equal(command,np.zeros(3))
        and diagnostic.get('command_body')==[0.,0.,0.])


def _r67_capture_known_frame_false(node,now,d,s,e,control_frame,control_candidate_code):
    # One pending raw-scalar snapshot, only the actual control predicate caller.
    if (control_frame.f_code is not control_candidate_code
            or getattr(node,'_r67_known_frame_failure_attached',False)):return
    gate=getattr(node,'heading_gate',None)
    if (node.state!='running' or not getattr(gate,'handoff_active',False)
            or getattr(gate,'handoff_failure',None) is not None):return
    def scalar(value):
        return value if (value is None or type(value) in (bool,int,str)
            or type(value) is float and math.isfinite(value)) else None
    def field(value,key):
        return scalar(value.get(key)) if type(value) is dict else None
    commit=d.get('source_commit') if type(d) is dict else None
    snapshot=dict(
        schema='known_scene_final_false_raw_snapshot/v1',predicate_now_monotonic_wall=scalar(now),
        d_present=d is not None,s_present=s is not None,e_present=e is not None,
        d_received_monotonic_wall=field(d,'received_monotonic_wall'),
        s_received_monotonic_wall=field(s,'received_monotonic_wall'),
        e_received_monotonic_wall=field(e,'received_monotonic_wall'),
        d_generation=field(d,'generation'),s_generation=field(s,'generation'),e_generation=field(e,'generation'),
        d_control_allowed=field(d,'control_allowed'),d_correction_stale=field(d,'correction_stale'),
        d_correction_transition=field(d,'correction_transition'),s_correction_transition=field(s,'correction_transition'),
        s_initialized=field(s,'initialized'),e_ready=field(e,'ready'),
        explicit_message_commit_token=field(commit,'explicit_message_commit_token'),
        pose_stamp=scalar(node.pose_stamp),cloud_stamp=scalar(node.cloud_stamp),d_source_ns=field(d,'source_ns'))
    node._r67_known_frame_failure_pending=(control_frame,snapshot)


def _r67_attach_known_frame_failure(node,call,before):
    try:
        pending=getattr(node,'_r67_known_frame_failure_pending',None)
        if pending is None or getattr(node,'_r67_known_frame_failure_attached',False):return
        # Keep the actual originating frame alive: neither a reused frame id
        # nor another invocation of the same code can claim this snapshot.
        node._r67_known_frame_failure_pending=None
        control_frame,snapshot=pending
        caller=sys._getframe(1)
        while caller is not None and caller is not control_frame:caller=caller.f_back
        if caller is None or before.get('state_before')!='running' or node.state!='failed':return
        node._r67_known_frame_failure_attached=True
        if type(call) is dict:call['known_frame_final_false']=snapshot
    except Exception:
        pass


def _r67_observe_known_generation_ready(original,control_candidate_code):
    # Observe the actual selected predicate; never duplicate its business rules.
    time=original.__globals__['time']
    def observed(self):
        result=original(self)
        if result:return result
        try:
            previous=getattr(self,'_r67_known_frame_failure_pending',None)
            _r67_capture_known_frame_false(self,time.monotonic(),
                self.known_scene_inputs,self.known_scene_status,self.known_scene_scan_epoch,
                sys._getframe(1),control_candidate_code)
            pending=getattr(self,'_r67_known_frame_failure_pending',None)
            if pending is not None and pending is not previous:
                pending[1]['clock_read_boundary']='after_original_predicate_return'
        except Exception:
            pass
        return result
    observed.__wrapped__=original
    return observed

def observe_business_gate(node,gate,decision,reasons):
    """Diagnostic execution permission; original refusal remains in evidence."""
    config=node.profile.get('diagnostic_observe_only',{})
    if (decision.get('allowed') is not False or decision.get('reason') not in reasons
            or not isinstance(config,dict) or type(config.get('enabled'))is not bool
            or config['enabled'] is not True):return decision
    # Validate before the deduplicating ledger can return early. Only fresh
    # actual finite same-generation inputs can consume a business observation.
    json.dumps(decision,allow_nan=False)
    pose=np.asarray(node.pose,dtype=float);rotation=np.asarray(node.rotation,dtype=float)
    if (pose.shape!=(3,) or rotation.shape!=(3,3) or not np.isfinite(pose).all()
            or not np.isfinite(rotation).all() or not node._known_generation_ready()):return decision
    from embodied_navigation.common.observation import api as _observer_api; record = _observer_api('diagnostic_observation').record
    if record(node.run,node.profile,gate,str(decision['reason']),decision)is not True:
        raise RuntimeError('business_gate_observation_not_recorded')
    return dict(decision,allowed=True,diagnostic_original_allowed=False,
        diagnostic_original_reason=decision['reason'],diagnostic_observe_only=True,
        diagnostic_execution_continued=True,navigation_PASS=False)

def finite_retention_reference(node,decision):
    """Observe an original finite reference score, never a bad preview/source."""
    try:
        reference=decision['reference'];cascade=node.cascade
        path=np.asarray(cascade.path,dtype=float);cumulative=np.asarray(cascade.cumulative,dtype=float)
        projection=reference['raw_projection'];segment=projection['segment']
        if (path.ndim!=2 or path.shape[1]!=3 or not 2<=len(path)<=2000
                or not np.isfinite(path).all() or cumulative.shape!=(len(path),)
                or not np.isfinite(cumulative).all() or not np.all(np.diff(cumulative)>0)
                or type(segment)is not int or not 0<=segment<len(path)-1
                or reference['path_id']!=cascade.path_id or reference['path_sha256']!=cascade.path_sha
                or not node._known_generation_ready()):return False
        def finite_tree(value):
            if isinstance(value,dict):return all(finite_tree(x)for x in value.values())
            if isinstance(value,(list,tuple,np.ndarray)):return all(finite_tree(x)for x in value)
            if isinstance(value,(float,np.floating)):return math.isfinite(float(value))
            return True
        return finite_tree(reference) and finite_tree(decision.get('direction_coherence'))
    except (AttributeError,KeyError,TypeError,ValueError,IndexError,OverflowError):return False


def diagnostic_profile_view(profile):
    """The observer only consumes its explicit boolean; never copy the full profile."""
    config=profile.get('diagnostic_observe_only',{})
    return dict(diagnostic_observe_only=dict(enabled=bool(isinstance(config,dict)
        and type(config.get('enabled'))is bool and config['enabled'])))

def admission_callback_rejected(run,profile,elapsed_s,reason):
    """100ms for one synchronous parent admission stage; diagnostic policy unchanged."""
    if type(elapsed_s)not in (int,float) or not math.isfinite(elapsed_s) or elapsed_s<0:
        raise ValueError('admission_callback_elapsed_invalid')
    if elapsed_s<=.100:return False
    minimal=diagnostic_profile_view(profile)
    if not minimal['diagnostic_observe_only']['enabled']:return True
    from embodied_navigation.common.observation import api as _observer_api; record = _observer_api('diagnostic_observation').record
    record(run,minimal,'admission_callback_elapsed',reason,dict(
        callback_elapsed_wall_ms=elapsed_s*1000,original_limit_wall_ms=100.,
        original_timing_check_passed=False,diagnostic_execution_continued=True,
        source_freshness_checks_unchanged=True,navigation_PASS=False))
    return False


def _admission_metadata_fragment(metadata, receipt, receipt_plain):
    # Both graphs must be typed before pickle, then exact value bytes agree.
    if (receipt_plain and type(receipt) is dict and 'metadata' in receipt
            and _plain_json_evidence_graph(metadata)
            and pickle.dumps(metadata, protocol=4)==pickle.dumps(receipt['metadata'], protocol=4)):
        return json.dumps(metadata,allow_nan=False,ensure_ascii=False)
    return None

def _receipt_json_with_metadata_fragment(receipt, fragment):
    return '{'+', '.join(json.dumps(k,allow_nan=False,ensure_ascii=False)+': '+(
        fragment if k=='metadata' else json.dumps(v,allow_nan=False,ensure_ascii=False))
        for k,v in receipt.items())+'}'


class ResetBookkeeping:
    """Legacy archive reset counter only; cascade_core owns all control maths."""
    def __init__(self):
        self.resets=0
        self.reset('startup')
    def reset(self,reason):
        self.resets+=1
        self.reset_reason=reason


from embodied_navigation.navigation.mission.goal_regions import ArrivalWindow
from embodied_navigation.navigation.mission.continuous_route import configuration as continuous_configuration, RouteProgress, CausalHandoffProjectionController


from embodied_navigation.planning.admission.path_commit import staged_cascade_binding
from embodied_navigation.planning.admission.pending_path_commit import defer_future as defer_future_path, commit_ready as commit_pending_path, waiting as pending_path_waiting
from embodied_navigation.control.parking.navigation_stop_window import reset_transition, bind_transition_context

class PIDNavigation(TeacherNavigation):
    def __init__(self,run):
        super().__init__(run);self.pid=ResetBookkeeping()  # no legacy controller mathematics
        self._r105_terminal_owner_enabled=TERMINAL_PARKING_OWNER_ENABLED
        self._r105_terminal_context=None
        _r47_initialize(self, run)
        self.pid_velocity=None;self.pid_pose_stamp=None;self.pid_row=None;self._pid_row_receipt_snapshot=None;self._pid_receipt_bytes_cache=None
        self.pid_records=0;self.pid_guard_needs_evaluation=False;self.pid_prepared_output=None
        self.cloud_callback_records=0;self.pending_cloud_callback_row=None
        self.imu_history=deque(maxlen=512);self.feedback=None;self.paired_imu=None
        self.imu_source_buffer=None;self.causal_imu_subscription=None
        if imu_buffer_enabled(self.profile):
            self.imu_source_buffer=CausalImuBuffer(self,RawImuTilt)
            self.causal_imu_subscription=self.create_causal_imu_subscription(self.imu_source_buffer.callback)
        self.cascade=None;self.cascade_key=None;self.cascade_last_row=None
        self.path_receipt=None;self.cascade_path_id=None;self.final_bundle=None
        self.imu_records=0;self.cloud_archive_records=0
        self.segment_references={}
        self.global_reference_settings=global_reference_contract(self.profile)
        self.global_route_guides=SegmentGuides()
        self.continuous_settings=continuous_configuration(self.profile)
        self.continuous_route=None;self.coverage_replan_index=None
        self.scan_reference_coverage_index=None
        self.reference_consistency_hold=False;self.reference_consistency_check=None
        self.registered_cloud_archive_mode=self.profile.get('engineering_recording',{}).get('registered_cloud_archive','legacy_all')
        if self.registered_cloud_archive_mode not in ('legacy_all','disabled'):
            raise ValueError('Unsupported registered_cloud_archive mode; bounded_event is not yet implemented')
        self.reference_replan_pending=False
        self.path_handoff_early_requested_for=None;self.path_handoff_last_check=None
        self.pending_path_commit=None
        from embodied_navigation.planning.reference.automatic_reference_runtime import enabled as r85_enabled, AutomaticReferenceRuntime, AutomaticRouteFence
        self.automatic_reference = (AutomaticReferenceRuntime(self.run, self.profile, self.trajectory_association)
                                    if r85_enabled(self.profile) else None)
        if self.automatic_reference is not None:
            # Original46 observation must be separate; it cannot drive this task.
            self.continuous_settings = dict(self.continuous_settings, enabled=False)
        self.path_admission_fence=(AutomaticRouteFence(self.automatic_reference) if self.automatic_reference is not None
                                   else FrozenFence(self.run,self.profile))
        self.r85_physical_gate=None
        if self.automatic_reference is not None:
            from embodied_navigation.planning.admission.physical_gate import R85PhysicalPathGate
            self.r85_physical_gate=R85PhysicalPathGate(self.run)
        self._r88_admission=None
        self.path_admission_active=None;self.path_admission_hold=False;self.path_admission_replan_pending=False
        self.path_admission_sequence=0;self.path_admission_last_decision=None
        self.control_clock_hold=ControlClockHold()
        self.corridor_observer=None
        self.corridor_last_certificate=None
        self.corridor_last_request_wall=-math.inf
        self.corridor_snapshot_count=0
        corridor_config=self.profile.get('corridor',{'mode':'off'})
        if corridor_config.get('mode') not in ('off','shadow') or corridor_config.get('retain_valid_plan',False):
            raise ValueError('Unvalidated corridor control authority is not enabled in this revision')
        if corridor_config.get('mode')=='shadow':
            from embodied_navigation.common.observation import api as _observer_api; CorridorObserver = _observer_api('corridor_runtime').CorridorObserver
            from nav_msgs.msg import Path as PathMsg
            from std_msgs.msg import String
            self.corridor_observer=CorridorObserver(dict(
                horizon_m=1.2,max_half_width_m=.30,width_step_m=.04,
                max_speed_mps=float(self.profile['max_speed_mps']),brake_decel_mps2=.15,
                latency_s=.3,stop_margin_m=.10,body_radius_m=.25,body_offset_m=.18,
                body_below_m=.12,body_above_m=.12,position_uncertainty_m=.08,
                yaw_half_range_rad=math.pi,map_max_age_ns=300_000_000,
                cell_max_age_ns=300_000_000,support_max_age_ns=300_000_000,
                body_ground_height_m=.32,support_height_tolerance_m=.10,max_support_slope_rad=.35))
            self.corridor_request_pub=self.create_publisher(PathMsg,'/demo/teacher/corridor/request',1)
            self.corridor_snapshot_sub=self.create_subscription(String,'/demo/teacher/corridor/snapshot',
                self.on_corridor_snapshot,1)
        # PUBLICATION_OBSERVER_BEGIN init_counter
        self.publication_records=0
        # PUBLICATION_OBSERVER_END init_counter
    def on_request(self,msg):
        previous=self.request_id
        super().on_request(msg)
        if self.request_id!=previous and hasattr(self,'cascade'):
            self._r105_terminal_context=None
            asynchronous=getattr(self,'_r88_admission',None)
            if asynchronous is not None:
                asynchronous.invalidate('accepted_request_changed')
            # Each stage has its own measured path and PI state. Source/IMU
            # histories and the immutable original registration remain valid.
            self.cascade=None;self.cascade_key=None;self.cascade_last_row=None
            self.cascade_path_id=None;self.path_receipt=None;self.final_bundle=None
            self.reference_replan_pending=False
            self.pending_path_commit=None
            self.path_admission_active=None;self.path_admission_hold=False;self.path_admission_replan_pending=False
            self.pid_row=None;self._pid_row_receipt_snapshot=None;self.pid_prepared_output=None
            self.reference_consistency_hold=False;self.reference_consistency_check=None
            self.publish_command()
            self.continuous_route=(RouteProgress(self.request_id,self.goals,
                self.goals_definition_sha256,self.continuous_settings)
                if self.continuous_settings["enabled"] else None)
            self.coverage_replan_index=None
    def on_corridor_snapshot(self,msg):
        from embodied_navigation.common.observation import api as _observer_api; decode_snapshot = _observer_api('corridor_runtime').decode_snapshot; sha_text = _observer_api('corridor_runtime').sha_text
        wall=time.monotonic_ns();clock=self.get_clock().now().nanoseconds
        if self.corridor_observer is None:return
        try:
            snapshot=decode_snapshot(msg.data)
            self.corridor_snapshot_count+=1
            if self.cascade is None or self.feedback is None or self.path_receipt is None:return
            reference=self.cascade.preview_reference(self.pose)
            # The route identity disambiguates overlapping floor paths; the
            # actual surface still has to be established by measured points.
            layer_id=str(self.request_id)+':'+str(self.waypoint_index)
            path=dict(points_xyz=self.cascade.path.tolist(),path_id=self.cascade.path_id,
                path_sha256=self.cascade.path_sha,frame_id='camera_init',layer_id=layer_id,
                stamp_ns=self.cascade.path_record['stamp_ns'])
            state=dict(position_world_xyz=self.pose.tolist(),
                yaw_rad=math.atan2(self.rotation[1,0],self.rotation[0,0]),
                speed_mps=float(np.linalg.norm(self.feedback['origin_velocity_body'][:2])),
                progress_m=reference['nearest_s_m'],stamp_ns=int(self.pose_stamp))
            digest=sha_text(msg.data)
            if self.corridor_observer.submit(snapshot,path,state,clock,wall,digest):
                import gzip
                raw=msg.data.encode('utf-8')
                relative=Path('corridor_snapshots')/(str(self.corridor_snapshot_count).zfill(6)+'.json.gz')
                def archive():
                    target=self.run/relative;target.parent.mkdir(exist_ok=True)
                    target.write_bytes(gzip.compress(raw,compresslevel=1,mtime=0))
                self.evidence.enqueue(archive)
                self.evidence.append(self.run/'corridor_inputs.jsonl',dict(snapshot_file=str(relative),
                    snapshot_sha256=digest,path=path,actual_state=state,limits=self.corridor_observer.limits,
                    received_wall_ns=wall,compute_request_clock_ns=clock,
                    navigation_ground_truth_used=False,control_authority=False))
        except Exception as error:
            self.evidence.append(self.run/'corridor_input_rejections.jsonl',dict(
                reason=type(error).__name__+':'+str(error),received_wall_ns=wall,
                clock_ns=clock,control_authority=False))
    def poll_corridor(self,now):
        try:return self._poll_corridor(now)
        except Exception as error:
            self.evidence.append(self.run/'corridor_input_rejections.jsonl',dict(
                reason='shadow_poll:'+type(error).__name__+':'+str(error),
                received_wall_ns=time.monotonic_ns(),control_authority=False))
    def _poll_corridor(self,now):
        if self.corridor_observer is None:return
        result=self.corridor_observer.poll()
        if result is not None:
            result['recorded_clock_ns']=self.get_clock().now().nanoseconds
            result['active_path_id_at_receipt']=None if self.cascade is None else self.cascade.path_id
            self.corridor_last_certificate=result
            self.evidence.append(self.run/'corridor_certificates.jsonl',result)
        if (self.state!='running' or self.cascade is None or self.cascade.path is None or
            now-self.corridor_last_request_wall<float(self.profile['corridor']['snapshot_request_period_s'])):
            return
        from nav_msgs.msg import Path as PathMsg
        from geometry_msgs.msg import PoseStamped
        from embodied_navigation.planning.reference.spatial_reference import point_at
        reference=self.cascade.preview_reference(self.pose)
        start=reference['nearest_s_m'];end=min(start+1.2,float(self.cascade.cumulative[-1]))
        msg=PathMsg();msg.header.frame_id='camera_init';msg.header.stamp=self.get_clock().now().to_msg()
        for point in [self.pose,*[point_at(self.cascade.path,self.cascade.cumulative,s)
                                 for s in np.linspace(start,end,13)]]:
            pose=PoseStamped();pose.header=msg.header
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,point)
            pose.pose.orientation.w=1.;msg.poses.append(pose)
        self.corridor_request_pub.publish(msg)
        self.corridor_last_request_wall=now
    def on_imu(self,msg):
        before=self.gyro_stamp;before_records=self.imu_records;wall=time.monotonic_ns();clock=self.get_clock().now().nanoseconds
        super().on_imu(msg);stamp=int(self.stamp(msg));q=msg.orientation;v=msg.angular_velocity
        # R85 retains original safety-accepted IMU bytes until their headers
        # become causal. The unchanged odometry pairing still requires
        # IMU <= pose <= control clock and the original 20ms pair window.
        r85_history_only=getattr(self,'automatic_reference',None) is not None
        if (self.gyro_stamp is not None and round(self.gyro_stamp*1e9)==stamp and self.gyro_stamp!=before
                and (stamp<=clock or r85_history_only)):
            row=dict(stamp_ns=stamp,received_wall_ns=wall,frame_id='body',
                angular_velocity_body=list(self.gyro_body),source_frame_id=msg.header.frame_id,
                gyro_source_xyz=[v.x,v.y,v.z],orientation_xyzw=[q.x,q.y,q.z,q.w],
                orientation_covariance=list(msg.orientation_covariance),
                imu_to_body_rotation=self.imu_body_rotation.tolist(),callback_ros_clock_ns=clock,
                source_topic='/livox/imu',navigation_ground_truth_used=False)
            if stamp>clock:row['r85_original_future_header_history_only']=True
            self.imu_history.append(row);self.imu_records+=1
            _enqueue_navigation_imu_record(self,row,self.imu_records)
        if self.imu_source_buffer is not None:
            intake=self.imu_source_buffer.safety_intake
            original_raw=self.raw_imu if intake is None else intake.original_raw_imu
            self.imu_source_buffer.note_latest(stamp,wall,clock,self.imu_records>before_records,original_raw.last_rejected)
    def on_odom(self,msg):
        stamp=int(self.stamp(msg))
        if CLOCK_SERVICE_ENABLED:
            self._r114_clock_service.service(stamp)
        _clock_read_before=(_r99_clock_probe.read_before(self)
            if _r99_clock_probe.READ_OPTION == '1' else None)
        clock=self.get_clock().now().nanoseconds;wall=time.monotonic_ns()
        _r99_clock_probe.note(self,stamp,clock,wall,self.pose_stamp,_clock_read_before)
        if stamp>clock:
            _sensor_callback_odom(self,stamp,clock,wall,self.pose_stamp,self.pose_stamp,early_future=True)
            return
        join=getattr(self,'_r104_parking_causal_join',None)
        bound,paired=(False,None) if join is None else join.bound_pair(self,stamp,clock,wall)
        if not bound:
            paired=next((r for r in reversed(self.imu_history)if 0<=stamp-r['stamp_ns']<=20_000_000),None)
            if self.imu_source_buffer is not None:
                original=paired;paired=self.imu_source_buffer.pair(stamp,paired,clock)
                if original is None:
                    self.imu_source_buffer.event(stamp,clock,wall,'odom_pair_missing'if paired is None else 'odom_pair_from_fifo',paired)
        latest=(self.gyro_body,self.gyro_stamp,self.gyro_wall)
        if paired:
            self.gyro_body=paired['angular_velocity_body'];self.gyro_stamp=paired['stamp_ns']/1e9
            self.gyro_wall=paired['received_wall_ns']/1e9
        else:self.gyro_body=None
        before=self.pose_stamp;quiet=getattr(self.heading_gate,'quiet_target_enabled',False)
        if quiet:
            self.heading_gate.begin_quiet_pose(stamp,clock,wall,self.heading_gate.transition_context,
                self.heading_gate.transition_purpose,self.handoff_scope_sha256,self.cascade_path_id,
                None if self.cascade is None else self.cascade.path_sha,
                None if self.cascade is None else self.cascade.goal_sha)
        try:super().on_odom(msg)
        finally:
            self.gyro_body,self.gyro_stamp,self.gyro_wall=latest
            if quiet:self.heading_gate.finish_quiet_pose(stamp,self.pose_stamp>before)
        if self.pose_stamp>before:
            q=msg.pose.pose.orientation;v=msg.twist.twist.linear
            from embodied_navigation.perception.sensors.feedback_mailbox import receipt_ns
            from embodied_navigation.localization_mapping.feedback_state import accepted_slam_feedback
            self.feedback=accepted_slam_feedback(stamp_ns=stamp,received_wall_ns=receipt_ns(self,'body',wall),
                position_world_xyz=self.pose.tolist(),quaternion_wxyz=[q.w,q.x,q.y,q.z],
                origin_velocity_body=[v.x,v.y,v.z],callback_ros_clock_ns=clock)
            self.paired_imu=copy.deepcopy(paired)
            self.pid_velocity=self.rotation@np.array([v.x,v.y,v.z]);self.pid_pose_stamp=stamp
            self.evidence.append(self.run/'navigation_feedback_history.jsonl',dict(
                self.feedback,paired_imu=copy.deepcopy(paired),navigation_ground_truth_used=False))
        _sensor_callback_odom(self,stamp,clock,wall,before,self.pose_stamp,message=msg)
    def on_cloud(self,msg):
        wall=time.monotonic();clock=self.get_clock().now().nanoseconds;before=self.cloud_stamp
        self.cloud_callback_records+=1
        self.pending_cloud_callback_row=dict(schema='pid_actual_cloud_callback/v1',sequence=self.cloud_callback_records,
            producer_stamp_ns=int(self.stamp(msg)),frame_id=msg.header.frame_id,received_monotonic_wall=wall,
            received_ros_clock_ns=clock,source_topic='/cloud_registered_full',
            input_width=int(msg.width),input_height=int(msg.height),previous_accepted_stamp_ns=int(before),
            reason=None,navigation_ground_truth_used=False)
        try:
            super().on_cloud(msg)
            if self.cloud_stamp>before:self.archive_cloud(msg)
        finally:
            row=self.pending_cloud_callback_row;end=time.monotonic()
            row.update(finished_monotonic_wall=end,callback_duration_wall_s=end-wall,
                finished_ros_clock_ns=self.get_clock().now().nanoseconds,
                accepted=row['reason']=='accepted',accepted_stamp_ns=int(self.cloud_stamp),
                accepted_cloud_received_monotonic_wall=self.cloud_updated if math.isfinite(self.cloud_updated)else None)
            if row['reason']is None:row['reason']='callback_returned_without_branch_receipt'
            _sensor_callback_cloud(self,row)
            self.evidence.append(self.run/'navigation_cloud_callbacks.jsonl',row);self.pending_cloud_callback_row=None
    def archive_cloud(self,msg):
        self.cloud_archive_records+=1;seq=self.cloud_archive_records
        if getattr(self,'registered_cloud_archive_mode','legacy_all')=='disabled':
            # Keep the accepted live geometry and exact native guard receipt.
            # Do not allocate an additional point copy or write payload files.
            self.guard_cloud_receipt.update(cloud_array_file=None,cloud_raw_payload_file=None,
                cloud_archive_sequence=None,cloud_archive_collected=False)
            self.evidence.append(self.run/'navigation_cloud_xyz.jsonl',dict(
                schema='actual_registered_cloud_archive_disabled/v1',sequence=seq,
                stamp_ns=int(self.cloud_stamp),frame_id=msg.header.frame_id,
                filtered_xyz_float64_sha256=self.guard_cloud_receipt['filtered_xyz_float64_sha256'],
                filtered_points=int(len(self.guard_cloud)),input_payload_bytes=len(msg.data),
                array_file=None,raw_payload_file=None,archive_mode='disabled',
                cloud_archive_collected=False,cloud_replay_verified=False,
                historical_full_cloud_evidence_pass=False,live_guard_input_preserved=True,
                navigation_ground_truth_used=False))
            return
        xyz=np.array(self.guard_cloud,dtype='<f8',copy=True);raw=bytes(msg.data)
        directory=self.run/'navigation_cloud_arrays';name=f'{seq:06d}_{self.cloud_stamp}'
        array=directory/(name+'.npy');payload=directory/(name+'.bin')
        record=dict(schema='actual_registered_cloud_xyz_archive/v1',sequence=seq,
            stamp_ns=int(self.cloud_stamp),frame_id=msg.header.frame_id,
            received_monotonic_wall=self.guard_cloud_receipt['received_monotonic_wall'],
            array_file=str(array.relative_to(self.run)),raw_payload_file=str(payload.relative_to(self.run)),
            filtered_xyz_float64_sha256=hashlib.sha256(xyz.tobytes()).hexdigest(),
            raw_payload_sha256=hashlib.sha256(raw).hexdigest(),dtype='<f8',shape=list(xyz.shape),
            fields=[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count)for f in msg.fields],
            width=int(msg.width),height=int(msg.height),point_step=int(msg.point_step),row_step=int(msg.row_step),
            is_bigendian=bool(msg.is_bigendian),is_dense=bool(msg.is_dense),
            filtering_body_stamp_ns=self.guard_cloud_receipt['nearest_slam_pose_stamp_ns'],
            filtering_body_pose=self.guard_cloud_receipt['filtering_body_pose'],
            filtering_body_rotation=self.guard_cloud_receipt['filtering_body_rotation'],
            navigation_ground_truth_used=False)
        self.guard_cloud_receipt.update(cloud_array_file=record['array_file'],
            cloud_raw_payload_file=record['raw_payload_file'],cloud_archive_sequence=seq)
        def archive():
            directory.mkdir(exist_ok=True);np.save(array,xyz,allow_pickle=False);payload.write_bytes(raw)
            document={**record,'array_sha256':hashlib.sha256(array.read_bytes()).hexdigest()}
            with (self.run/'navigation_cloud_xyz.jsonl').open('a')as out:out.write(json.dumps(document,allow_nan=False)+'\n')
        self.evidence.enqueue(archive)
    def admission_identity(self):
        return dict(request_id=self.request_id,waypoint_index=self.waypoint_index,
            goal_xyz=self.waypoints[self.waypoint_index].tolist())
    def path_execution_eligibility(self):
        """Per-condition original live-source/identity evidence, without TTL refresh."""
        active=self.path_admission_active;now=time.monotonic();ns=self.get_clock().now().nanoseconds
        def fresh_wall(value):
            return isinstance(value,(int,float)) and math.isfinite(value) and 0<=now-value<=.3
        conditions=dict(active_present=active is not None,
            same_request_waypoint_goal=active is not None and active['identity']==self.admission_identity(),
            same_active_trajectory=active is not None and active['trajectory_id']==self.active_trajectory_id,
            same_active_reference=active is not None and tuple(active['active_reference_stamp'])==tuple(self.active_reference_stamp or ()),
            samples_present=self.samples is not None,state_running=self.state=='running',
            pose_source_fresh=minimal_pose_cloud_fresh(self,'pose',now,ns/1e9,fresh_wall(self.pose_updated)),
            cloud_source_fresh=minimal_pose_cloud_fresh(self,'cloud',now,ns/1e9,fresh_wall(self.cloud_updated)),
            raw_imu_source_fresh=minimal_raw_imu_fresh(self,now,ns/1e9,self.raw_imu.fresh(now,ns/1e9)),
            bridge_source_fresh=not minimal_wall_stale(self,'bridge_safety',now,self.bridge_updated,.3,not fresh_wall(self.bridge_updated)),
            bridge_ready=self.bridge_safety.get('state')=='ready',
            obstacle_not_held=not self.obstacle_hold,tilt_not_held=not self.tilt_hold)
        return dict(conditions=conditions,samples_present=conditions['samples_present'],
            clock_ns=ns,checked_monotonic_wall=now,actual_source_times_refreshed=False)
    @_r87_diag.observe('scan.retain_admitted_path')
    def retain_admitted_path(self,points,config,*,max_route_points=256,binding=None):
        if self.r85_physical_gate is not None:
            try:
                frozen_points,frozen_config,frozen_binding=self._route_fence_load()
                owned=(points is frozen_points and config is frozen_config
                    and (binding is None or binding is frozen_binding))
                # Normal candidates carry this program-owned tuple. Retain the
                # original comparison only for the legacy external pending caller.
                if (not owned and (not np.array_equal(points,frozen_points) or config!=frozen_config
                        or binding is not None and binding!=frozen_binding)
                        or self.automatic_reference.config.get('max_reference_points')!=4096):
                    raise ValueError('r85_retention_fence_capacity_or_binding_mismatch')
                binding=frozen_binding;max_route_points=4096
            except (OSError,KeyError,ValueError,TypeError,AttributeError,IndexError,OverflowError) as error:
                return dict(allowed=False,reason='r85_retention_frozen_fence_rejected',error=str(error),samples_present=self.samples is not None)
        retained_context=handoff_context(self)
        if not retained_context['allowed']:
            return dict(allowed=False,reason='old_path_coverage_or_context_not_eligible',
                eligibility=retained_context,conditions=retained_context['conditions'],samples_present=self.samples is not None)
        # New planner reference stamps are pending identities, not new receipts
        # for the path being executed. Progress is projected spatially only.
        eligibility=self.path_execution_eligibility()
        if not all(eligibility['conditions'].values()):
            return dict(allowed=False,reason='old_path_not_eligible_stopped',**eligibility)
        reference_execution=retention_reference_eligibility(self)
        if (not reference_execution['allowed']
                and diagnostic_profile_view(self.profile)['diagnostic_observe_only']['enabled']
                and finite_retention_reference(self,reference_execution)):
            reference_execution=observe_business_gate(self,'retained_SCAN_reference_prediction',reference_execution,
                {'old_reference_curvature_unavailable_requires_stop_or_new_path',
                 'old_reference_direction_incoherent_requires_stop_or_new_path'})
        if not reference_execution['allowed']:
            return dict(allowed=False,reason=reference_execution['reason'],
                reference_execution=reference_execution,eligibility=eligibility,
                conditions=eligibility['conditions'],samples_present=True)
        observer_options={}
        if diagnostic_profile_view(self.profile)['diagnostic_observe_only']['enabled']:
            def observe_remaining_score(score):
                observed=observe_business_gate(self,'retained_SCAN_route_fence_prediction',score,
                    {'outside_fence','depth_budget_exhausted','patch_budget_exhausted','wall_budget_exhausted'})
                return observed.get('allowed')is True
            observer_options['business_observer']=observe_remaining_score
        proof=remaining_certificate(self.path_admission_active,self.samples,self.pose,points,config,
            max_route_points=max_route_points,**observer_options)
        if proof['allowed'] and self.r85_physical_gate is not None:
            physical=self.r85_physical_gate.retain(self,self.path_admission_active,proof,binding)
            proof=dict(proof,r85_physical_retention=physical,allowed=physical['allowed'],
                reason=proof['reason'] if physical['allowed'] else physical['reason'])
        return dict(proof,eligibility=eligibility,conditions=eligibility['conditions'],samples_present=True,
            reference_execution=reference_execution,
            retention_action='kept_checked_old_path' if proof['allowed'] else 'stopped_old_path_not_certified')
    @_r87_diag.observe('scan.capture_replacement')
    def capture_replacement_eligibility(self,retention):
        """Recover only the recorded exhaustion of this immutable same-goal path.

        A missing path, source loss, protection, or failed old proof by itself
        cannot create this authorization. New-curve proof and base acceptance
        remain separate necessary gates; this evidence never commands motion.
        """
        marker=getattr(self,'execution_path_exhaustion',None);active=self.path_admission_active
        receipt=self.path_receipt or {};eligibility=self.path_execution_eligibility()
        conditions={k:v for k,v in eligibility['conditions'].items() if k!='samples_present'}
        conditions.update(samples_absent=self.samples is None,
            capture_same_goal=self.cascade is not None and self.cascade_key==(self.request_id,self.waypoint_index)
                and self.cascade.capture_stamp is not None,
            cascade_not_protected=self.cascade is not None and not self.cascade.protected,
            cascade_failure_not_latched=self.cascade is not None and self.cascade.failure_latched is None,
            exhaustion_receipt_present=isinstance(marker,dict))
        if isinstance(marker,dict):
            conditions.update(exhaustion_internal_schema=marker.get('schema')=='internal_checked_path_exhaustion/v1'
                    and marker.get('source')=='actual_shared_control_exhausted_clear_branch'
                    and marker.get('navigation_ground_truth_used') is False,
                exhaustion_allowed_reason=marker.get('reason') in ('steering_exhausted','low_command_exhausted_path'),
                exhaustion_same_request_waypoint_goal=dict(request_id=marker.get('request_id'),
                    waypoint_index=marker.get('waypoint_index'),goal_xyz=marker.get('goal_xyz'))==self.admission_identity(),
                exhaustion_same_active_id=marker.get('active_trajectory_id')==self.active_trajectory_id,
                exhaustion_same_active_reference=marker.get('active_reference_stamp')==list(self.active_reference_stamp or ()),
                exhaustion_matches_accepted_samples=marker.get('previous_samples_sha256')==receipt.get('source_samples_float64_sha256')
                    and marker.get('previous_samples_count')==receipt.get('source_samples_count')
                    and type(marker.get('previous_samples_count')) is int and 2<=marker['previous_samples_count']<=2000,
                exhaustion_matches_accepted_receipt=(receipt.get('request_id'),receipt.get('waypoint_index'),receipt.get('trajectory_id'))
                    ==(self.request_id,self.waypoint_index,self.active_trajectory_id),
                exhaustion_causal_stamps=type(marker.get('source_pose_stamp_ns')) is int
                    and type(marker.get('control_stamp_ns')) is int
                    and 0<=marker['source_pose_stamp_ns']<=marker['control_stamp_ns']<=eligibility['clock_ns'])
        else:
            conditions['exhaustion_internal_schema']=False
        allowed=all(conditions.values())
        return dict(allowed=allowed,reason='recorded_exhausted_capture_can_rebind' if allowed else 'capture_replacement_not_eligible_stopped',
            conditions=conditions,samples_present=self.samples is not None,exhaustion_receipt=copy.deepcopy(marker),
            source_eligibility=eligibility,old_retention_reason=retention.get('reason'),actual_source_times_refreshed=False)
    def clear_committed_exhausted_capture(self):
        # Only called after all existing execution gates committed a new path.
        # Keep the fixed goal, its original activation/90s deadline, velocity PI,
        # filters, feedback clocks and command slew state. A new capture must
        # obtain fresh original dwell; old parking proof is not transferable.
        cascade=self.cascade
        old={key:copy.deepcopy(getattr(cascade,key)) for key in
            ('capture_stamp','capture_wall','capture_clock','arrival','capture_dwell','hold_stamp','hold_wall','hold_clock')}
        for key in old:setattr(cascade,key,None)
        cascade.hold_integral[:]=0.
        self.execution_path_exhaustion=None
        self.pid_guard_needs_evaluation=True
        return dict(cleared_capture=old,fixed_goal_and_deadline_preserved=True,
            velocity_PI_filters_and_slew_preserved=True,fresh_capture_dwell_required=True)
    @_r87_diag.observe('scan.archive_admission')
    def archive_admission(self,value,metadata,decision,started):
        self.path_admission_sequence+=1;sequence=self.path_admission_sequence
        decision=dict(decision,schema='SCAN_path_execution_admission/v1',sequence=sequence,
            request_id=self.request_id,waypoint_index=self.waypoint_index,
            association_payload_id=None if value is None else value.get('traj_id'),
            active_trajectory_id=self.active_trajectory_id,
            active_reference_stamp=self.active_reference_stamp,
            admission_replan_pending=self.path_admission_replan_pending,
            pending_reference_stamp=self.trajectory_association.reference_stamp,
            control_stamp_ns=self.get_clock().now().nanoseconds,source_pose_stamp_ns=int(self.pose_stamp),
            callback_elapsed_wall_ms=(time.monotonic()-started)*1000,
            navigation_ground_truth_used=False,occupancy_or_terrain_support_certified=False)
        self.path_admission_last_decision=copy.deepcopy(decision)
        document=replayable(dict(decision=decision,payload=value,metadata=copy.deepcopy(metadata)))
        relative=Path('path_admission_candidates')/(str(sequence).zfill(6)+'_trajectory_'+str(decision['association_payload_id'])+'.json')
        def archive():
            target=self.run/relative;target.parent.mkdir(exist_ok=True)
            with target.open('x')as out:out.write(json.dumps(document,allow_nan=False,ensure_ascii=False)+'\n')
        self.evidence.enqueue(archive)
        self.evidence.append(self.run/'path_admission_decisions.jsonl',dict(document['decision'],candidate_file=str(relative)))
        _r85_intake_outer(self, value, metadata, decision, 'original_archive_admission_completed')
        _r102_progress.note(self,'admission_verdict',decision.get('admitted'),
            decision.get('reason'),decision.get('control_stamp_ns'),decision)
    def _route_definition_bound(self):
        """Low-frequency program binding; never copy/hash the route per candidate."""
        runtime=self.automatic_reference
        key=getattr(self.trajectory_association,'route_key',None)
        if runtime is None or key is None:
            raise ValueError('route_definition_not_bound')
        points,config,binding=self.path_admission_fence.load()
        if self.r85_physical_gate is not None:
            self.r85_physical_gate.bind_route_definition(self,binding)
        self._program_route_fence=(self._program_route_key(),points,config,binding)
        # One ordinary program representation of the same native guard input.
        self._program_guard_route=np.asarray(points,dtype=float)
        self._program_guard_route_reference=None

    def _program_route_key(self):
        runtime=self.automatic_reference
        return (id(runtime),runtime.file_sha,tuple(runtime.scope),self.request_id,
                getattr(self.trajectory_association,'route_key',None))

    def _route_fence_load(self):
        if self.automatic_reference is None:
            return self.path_admission_fence.load()
        current=getattr(self,'_program_route_fence',None)
        if current is None or current[0]!=self._program_route_key():
            self._route_definition_bound()
            current=self._program_route_fence
        return current[1:]

    @_r87_diag.observe('callback.scan_definition')
    def on_route_definition(self,msg):
        asynchronous=getattr(self,'_r88_admission',None)
        if asynchronous is not None:
            return asynchronous.receive('definition',msg,super().on_route_definition)
        return super().on_route_definition(msg)

    @_r87_diag.observe('callback.scan_metadata')
    def on_trajectory_metadata(self,msg):
        asynchronous=getattr(self,'_r88_admission',None)
        if asynchronous is not None:
            return asynchronous.receive('metadata',msg,super().on_trajectory_metadata)
        return super().on_trajectory_metadata(msg)
    @_r87_diag.observe('callback.scan_spline')
    def on_spline(self,msg):
        asynchronous=getattr(self,'_r88_admission',None)
        if asynchronous is not None:
            return asynchronous.receive('spline',msg,super().on_spline)
        return super().on_spline(msg)
    @_r87_diag.observe('callback.scan_attempt')
    def on_reference_attempt(self,msg):
        asynchronous=getattr(self,'_r88_admission',None)
        if asynchronous is not None:
            return asynchronous.receive('attempt',msg,super().on_reference_attempt)
        return super().on_reference_attempt(msg)
    @_r87_diag.observe('scan.accept_spline')
    def accept_spline(self,msg,metadata):
        self.pending_path_commit=None
        result=self._admit_matched_spline(msg,metadata)
        return result
    @_callback_cost_observe('admission')
    @_r87_diag.observe('scan.admission')
    def _admit_matched_spline(self,msg,metadata,received_wall_ns=None,*,admission_started=None,physical_result=None):
        # Admission is a transaction BEFORE base acceptance, publication, PI
        # path replacement and the accepted trajectory archive. Association's
        # payload high-water is deliberately not rolled back on a rejection.
        started=time.monotonic() if admission_started is None else admission_started
        asynchronous=getattr(self,'_r88_admission',None)
        if asynchronous is not None and physical_result is None:
            result=asynchronous.start(msg,metadata,received_wall_ns=received_wall_ns,admission_started=started)
            prepared_checkpoint=time.monotonic()
            item=asynchronous.current
            asynchronous.record('prepare_callback_completed',
                job_token=None if item is None else item['token'],
                admission_started_monotonic=started,
                parent_callback_checkpoint_monotonic=prepared_checkpoint,
                parent_callback_elapsed_wall_ms=(prepared_checkpoint-started)*1000,
                callback_budget_scope='single_synchronous_parent_stage')
            if (admission_callback_rejected(self.run,self.profile,prepared_checkpoint-started,
                    'admission_callback_budget_exhausted_during_async_prepare')
                    and item is not None and item['started']==started):
                # The submitted result cannot revive this rejected callback.
                # Keep queued FIFO events; its later result is an orphan.
                asynchronous.current=None;asynchronous.counts['invalidated']+=1
                asynchronous._transport_mark(12,job_token=item['token'])
                asynchronous.record('prepare_callback_rejected',job_token=item['token'])
                return self._admit_matched_spline(msg,metadata,received_wall_ns=item['received_wall_ns'],
                    admission_started=started,
                    physical_result={'error':'admission_callback_budget_exhausted_during_async_prepare'})
            return result
        value=None;binding=None;points=None;config=None;route_point_budget=256
        try:
            if len(msg.pos_pts)>512 or len(msg.knots)>516:raise ValueError('candidate_input_count_budget')
            if self.automatic_reference is not None:
                from embodied_navigation.planning.reference_protocol import payload as mode3_payload
                with _r87_diag.span('scan.payload_decode',source_ns=self.pose_stamp):
                    value=mode3_payload(msg)
                if (metadata.get('schema') != 3
                        or metadata.get('trajectory_id') != value['traj_id']
                        or metadata.get('trajectory_start_time') != value['start_time']
                        or (metadata.get('route_id'),metadata.get('route_version')) != self.trajectory_association.route_key
                        or self.trajectory_association.accepted != value
                        or metadata.get('reference_stamp') != self.trajectory_association.reference_stamp):
                    raise ValueError('r85_candidate_not_bound_by_schema3_association')
            else:
                with _r87_diag.span('scan.payload_decode',source_ns=self.pose_stamp):
                    value=payload(msg)
                    value=candidate(msg,metadata,self.trajectory_association.reference_stamp,
                        self.waypoints[self.waypoint_index] if self.continuous_route is None else self.trajectory_association.goal)
            if (self.continuous_route is not None and
                    self.scan_reference_coverage_index != self.continuous_route.next_index):
                raise ValueError('candidate_ordered_coverage_epoch_mismatch')
            if (self.continuous_route is not None and not np.array_equal(
                    self.trajectory_association.goal,self.waypoints[self.scan_reference_coverage_index])):
                raise ValueError('candidate_ordered_reference_goal_mismatch')
            with _r87_diag.span('scan.fence_load',source_ns=self.pose_stamp):
                points,config,binding=self._route_fence_load()
            if self.automatic_reference is not None:
                # Only the current successfully loaded/frozen R85 source may
                # use its explicit reference capacity. Legacy fences keep 256.
                from embodied_navigation.planning.reference.automatic_reference_runtime import MAX_REFERENCE_POINTS
                if (binding.get('schema')!='r85_automatic_route_fence_binding/v1'
                        or self.automatic_reference.config.get('max_reference_points')!=MAX_REFERENCE_POINTS):
                    raise ValueError('r85_fence_route_capacity_binding_mismatch')
                route_point_budget=MAX_REFERENCE_POINTS
            with _r87_diag.span('scan.curve_certificate',source_ns=self.pose_stamp):
                proof=certify_curve(value,points,config,max_route_points=route_point_budget)
            proof=observe_business_gate(self,'scan_route_fence',proof,
                {'outside_fence','depth_budget_exhausted','patch_budget_exhausted','wall_budget_exhausted'})
            if proof['allowed'] and self.continuous_route is not None:
                index=self.scan_reference_coverage_index
                prefix=self.continuous_route.points[index:index+2]
                with _r87_diag.span('scan.ordered_curve_certificate',source_ns=self.pose_stamp):
                    ordered_proof=certify_curve(value,prefix,config)
                proof=dict(proof,ordered_current_leg=ordered_proof,
                    ordered_coverage_index=index,local_body_goal=list(self.trajectory_association.goal))
                if not ordered_proof['allowed']:
                    proof=dict(proof,allowed=False,reason='outside_ordered_current_leg')
                    proof=observe_business_gate(self,'scan_ordered_current_leg',proof,{'outside_ordered_current_leg'})
            if proof['allowed']:
                begin,end=value['knots'][3],value['knots'][-4]
                with _r87_diag.span('scan.execution_samples',source_ns=self.pose_stamp):
                    check=BSpline(value['knots'],value['pos_pts'],3)(np.linspace(begin,end,min(2000,max(2,int((end-begin)/.08)+1))))
                if not np.isfinite(check).all():proof=dict(allowed=False,reason='nonfinite_execution_samples')
                elif self.pose is not None and np.linalg.norm(check[0]-self.pose)>1.2:
                    proof=dict(allowed=False,reason='execution_start_far_from_actual_pose')
                    proof=observe_business_gate(self,'scan_execution_start_distance',proof,{'execution_start_far_from_actual_pose'})
                elif (not trajectory_has_progress(check,self.pose,self.waypoints[self.waypoint_index],self.arrival_radius)
                        or not np.any(np.linalg.norm(check[1:,:2]-check[0,:2],axis=1)>=1e-5)):
                    proof=dict(allowed=False,reason='execution_path_degenerate')
            if proof['allowed'] and self.global_reference_settings['enabled']:
                if self.segment_start is None:raise ValueError('direction_check_requires_original_segment_activation')
                delta=self.waypoints[self.waypoint_index][:2]-self.segment_start[:2]
                with _r87_diag.span('scan.direction_check',source_ns=self.pose_stamp):
                    check_direction=admission_direction_check(check,self.pose,self.waypoints[self.waypoint_index],
                        self.fixed_terminal_heading(self.waypoint_index,self.segment_start),self.profile['cascade'],
                        self.profile['teacher_transition']['drive_heading_max_rad'],self.global_reference_settings)
                proof=dict(proof,direction_coherence=check_direction)
                if not check_direction['allowed']:
                    proof=dict(proof,allowed=False,reason='execution_direction_incoherent')
                    proof=observe_business_gate(self,'scan_execution_direction',proof,{'execution_direction_incoherent'})
            if proof['allowed'] and self.r85_physical_gate is not None:
                with _r87_diag.span('scan.physical_certificate',source_ns=self.pose_stamp):
                    if physical_result is None:
                        physical=self.r85_physical_gate.certify(self,value,metadata,binding)
                    elif physical_result.get('error'):
                        physical=dict(schema='r85_actual_scan_physical_execution_gate/v1',allowed=False,
                            reason='r88_physical_worker_rejected',error=physical_result['error'])
                    else:
                        physical=self.r85_physical_gate.complete(self,value,metadata,binding,
                            physical_result['prepared'],physical_result['proof'])
                certificate=physical.get('physical_certificate',{})
                if (physical.get('reason')=='actual_curve_physical_space_not_certified'
                        and certificate.get('status')=='UNKNOWN_REJECTED'
                        and certificate.get('reason') in ('continuous_outer_bound_not_certified','patch_or_wall_budget_exhausted')):
                    physical=observe_business_gate(self,'r85_curve_physical_prediction',physical,
                        {'actual_curve_physical_space_not_certified'})
                proof=dict(proof,r85_physical=physical)
                if not physical['allowed']:
                    proof=dict(proof,allowed=False,reason=physical['reason'])
            if admission_callback_rejected(self.run,self.profile,time.monotonic()-started,
                    'admission_callback_budget_exhausted'):
                proof=dict(allowed=False,reason='admission_callback_budget_exhausted')
        except (OSError,KeyError,ValueError,TypeError,OverflowError,IndexError,FloatingPointError)as error:
            proof=dict(allowed=False,reason='admission_input_or_anchor:'+type(error).__name__+':'+str(error))
        handoff=None
        if proof['allowed']:
            try:
                with _r87_diag.span('scan.handoff_preview',source_ns=self.pose_stamp):
                    handoff=preview_path_handoff(self,check,value,metadata)
                if not handoff['allowed'] and handoff.get('reason')=='handoff_not_ready':
                    observed=observe_business_gate(self,'scan_path_handoff_heading',handoff,{'handoff_not_ready'})
                    if observed.get('diagnostic_observe_only'):
                        # Accept the legal curve, then use the unchanged stop/turn
                        # state machine for its heading instead of driving across it.
                        handoff=dict(observed,diagnostic_heading_rebind=True)
                    if not handoff.get('diagnostic_heading_rebind'):
                        rebind=stopped_path_rebind(self,handoff,proof)
                        handoff=dict(handoff,stopped_rebind=rebind,allowed=rebind['allowed'],
                            reason='handoff_measured_stop_rebind' if rebind['allowed'] else 'handoff_not_ready')
                if not handoff['allowed']:proof=dict(proof,original_geometry_allowed=True,original_geometry_reason=proof.get('reason'),allowed=False,reason=handoff['reason'])
            except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as error:
                proof=dict(proof,allowed=False,reason='handoff_preview_invalid:'+type(error).__name__+':'+str(error))
        capture=(self.cascade is not None and self.cascade_key==(self.request_id,self.waypoint_index)
            and self.cascade.capture_stamp is not None)
        if self.cascade is not None and self.cascade.capture_stamp is not None and not capture:
            proof=dict(allowed=False,reason='capture_goal_identity_mismatch')
        retain=None;recovery=None
        if capture and proof['allowed']:
            with _r87_diag.span('scan.retain_path',source_ns=self.pose_stamp):
                retain=self.retain_admitted_path(points,config,max_route_points=route_point_budget,binding=binding)
            if not retain['allowed']:recovery=self.capture_replacement_eligibility(retain)
        if (proof['allowed'] and handoff is not None and handoff.get('stopped_rebind',{}).get('allowed')
                and not handoff.get('diagnostic_heading_rebind')):
            commit_recheck=stopped_path_rebind(self,handoff,proof)
            handoff=dict(handoff,commit_stop_recheck=commit_recheck)
            if not commit_recheck['allowed']:
                proof=dict(proof,allowed=False,reason='handoff_stop_or_source_changed_before_commit')
        if admission_callback_rejected(self.run,self.profile,time.monotonic()-started,
                'admission_callback_budget_exhausted'):
            proof=dict(allowed=False,reason='admission_callback_budget_exhausted');recovery=None
        if not proof['allowed'] or capture and (recovery is None or not recovery['allowed']):
            # Only a geometrically outside candidate (or an existing terminal
            # capture) may keep an independently certified old same-goal path.
            # Invalid anchor/numbers and exhausted proofs always stop/replan.
            retain=(retain if retain is not None else self.retain_admitted_path(points,config,max_route_points=route_point_budget,binding=binding) if points is not None
                and (proof.get('reason')=='outside_fence' or proof['allowed'] and capture or handoff is not None and not handoff['allowed'] and handoff_context(self)['allowed'])
                else dict(allowed=False,reason='fail_closed_rejection_requires_stop',samples_present=self.samples is not None,
                    conditions=self.path_execution_eligibility()['conditions']))
            if admission_callback_rejected(self.run,self.profile,time.monotonic()-started,
                    'retention_callback_budget_exhausted'):
                retain=dict(retain,allowed=False,reason='retention_callback_budget_exhausted',samples_present=self.samples is not None)
            stop_request=tail_stop_request(proof,handoff,
                getattr(self.cascade,'spatial_config',{}),self.pose_stamp,self.cloud_stamp,
                capture_active=getattr(self.cascade,'capture_stamp',None)is not None,
                failure_active=getattr(self.cascade,'failure_latched',None)is not None)
            if retain.get('allowed')is True and stop_request['required']:
                retain=dict(allowed=False,reason='nonfixed_tail_requires_original_measured_stop',
                    original_eligible_retention=retain,handoff_stop_request=stop_request)
            self.path_admission_hold=not retain['allowed']
            self.last_spline_rejected='V28 SCAN整条路径接纳拒绝：'+str(proof.get('reason'))
            self.path_admission_replan_pending=True
            if retain['allowed']:
                self.pid_guard_needs_evaluation=True
            else:
                self.message=self.last_spline_rejected+'；停车等待重规划'
                self.publish_command()
            self.archive_admission(value,metadata,dict(admitted=False,
                reason=('terminal_capture_kept_checked_path' if retain['allowed'] else 'terminal_capture_stopped_no_eligible_path')
                    if capture and proof['allowed'] else proof.get('reason'),
                proof=proof,handoff=handoff,anchor_binding=binding,old_path_retention=retain,capture_replacement=recovery,
                exact_zero_requested=not retain['allowed']),started)
            return
        if defer_future_path(self,msg,metadata,started=started,received_wall_ns=received_wall_ns):return
        if self.r85_physical_gate is not None:
            with _r87_diag.span('scan.before_lower_freshness',source_ns=self.pose_stamp):
                before_lower=self.r85_physical_gate.fresh(self,binding,self.known_scene_inputs['generation'])
            proof=dict(proof,r85_before_lower_source_gate=before_lower)
            if (not before_lower['allowed'] or admission_callback_rejected(self.run,self.profile,
                    time.monotonic()-started,'admission_callback_budget_exhausted_before_lower_execution')):
                self.path_admission_hold=True;self.path_admission_replan_pending=True;self.publish_command()
                self.last_spline_rejected='R85 physical path post-compute source/budget rejected'
                self.message=self.last_spline_rejected
                self.archive_admission(value,metadata,dict(admitted=False,
                    reason='r85_pre_execution_source_or_budget_rejected',proof=proof,handoff=handoff,
                    anchor_binding=binding,exact_zero_requested=True),started)
                return
        if self.r85_physical_gate is None:
            previous_execution=(dict(samples=self.samples,active_trajectory_id=self.active_trajectory_id,
                active_reference_stamp=self.active_reference_stamp,planning_start_state=copy.deepcopy(self.planning_start_state),
                execution_path_exhaustion=copy.deepcopy(getattr(self,'execution_path_exhaustion',None)))
                if recovery is not None and recovery['allowed'] else None)
            # A lower accepted archive is not yet the execution receipt commit.
            # Keep its unique replan/archive counter on a rejected final binding.
            binding_execution=dict(samples=self.samples,active_trajectory_id=self.active_trajectory_id,
                active_reference_stamp=self.active_reference_stamp,planning_start_state=copy.deepcopy(self.planning_start_state),
                execution_path_exhaustion=copy.deepcopy(getattr(self,'execution_path_exhaustion',None)),
                trajectory_archive_reference=getattr(self,'trajectory_archive_reference',None))
            binding_generation=self.known_scene_inputs['generation']
            before=self.replans
            wall=time.monotonic_ns() if received_wall_ns is None else received_wall_ns
            with _r87_diag.span('scan.lower_acceptance',source_ns=self.pose_stamp):
                super().accept_spline(msg,metadata)
            if self.replans>before and self.samples is not None:
                indices=[0]
                for i in range(1,len(self.samples)):
                    if np.linalg.norm(self.samples[i,:2]-self.samples[indices[-1],:2])>=1e-5:indices.append(i)
                points=self.samples[indices]
                receipt=dict(points_xyz=points.tolist(),
                    source_sample_indices=indices,source_samples_count=len(self.samples),
                    source_samples_float64_sha256=hashlib.sha256(np.asarray(self.samples,dtype='<f8').tobytes()).hexdigest(),
                    path_id=str(self.request_id)+':'+str(self.waypoint_index)+':'+str(self.replans)+':'+str(self.active_trajectory_id),
                    stamp_ns=int(self.active_reference_stamp[0])*1_000_000_000+int(self.active_reference_stamp[1]),
                    received_wall_ns=wall,frame_id='camera_init',array_file=self.trajectory_archive_reference,
                    request_id=self.request_id,waypoint_index=self.waypoint_index,
                    trajectory_id=self.active_trajectory_id,metadata=copy.deepcopy(metadata))
                receipt['points_float64_sha256']=hashlib.sha256(np.asarray(points,dtype='<f8').tobytes()).hexdigest()
                try:
                    with _r87_diag.span('scan.cascade_binding',source_ns=self.pose_stamp):
                        staged=staged_cascade_binding(self,receipt)
                    if staged is not None:
                        if admission_callback_rejected(self.run,self.profile,time.monotonic()-started,
                                'admission_callback_budget_exhausted'):
                            raise ValueError('admission_callback_budget_exhausted')
                        if not self._known_generation_ready() or self.known_scene_inputs['generation']!=binding_generation:
                            raise ValueError('admitted_path_source_generation_changed')
                    r85_active=None
                    if self.r85_physical_gate is not None:
                        # Prepare all potentially large immutable copies before the
                        # last freshness/100ms check, including lazy/capture paths.
                        r85_active=dict(payload=copy.deepcopy(value),metadata=copy.deepcopy(metadata),
                            identity=self.admission_identity(),trajectory_id=self.active_trajectory_id,
                            active_reference_stamp=tuple(self.active_reference_stamp),proof=copy.deepcopy(proof),anchor_binding=binding,
                            correction_generation=self.known_scene_inputs['generation'],
                            ordered_coverage_index=None if self.continuous_route is None else self.continuous_route.next_index)
                        with _r87_diag.span('scan.final_source',source_ns=self.pose_stamp):
                            final_source=self.r85_physical_gate.fresh(self,binding,binding_generation)
                        proof=dict(proof,r85_final_execution_source_gate=final_source)
                        r85_active['proof']['r85_final_execution_source_gate']=final_source
                        final_elapsed=time.monotonic()-started
                        final_source.update(admission_started_monotonic_wall=started,
                            final_checked_callback_elapsed_wall_ms=final_elapsed*1000,callback_budget_s=.100)
                        if not final_source['allowed']:
                            raise ValueError('r85_admitted_path_final_source_not_fresh_or_bound')
                        if admission_callback_rejected(self.run,self.profile,final_elapsed,
                                'admission_callback_budget_exhausted'):
                            raise ValueError('admission_callback_budget_exhausted')
                except (ValueError,TypeError,KeyError,AttributeError,OverflowError,FloatingPointError) as error:
                    for key,old in binding_execution.items():setattr(self,key,old)
                    self.path_admission_hold=True;self.path_admission_replan_pending=True;self.publish_command()
                    self.last_spline_rejected='V33 admitted path binding rejected: '+str(error)
                    self.message=self.last_spline_rejected+'; wait for a fully bound path'
                    self.archive_admission(value,metadata,dict(admitted=False,reason='admitted_path_binding_rejected',
                        proof=proof,handoff=handoff,anchor_binding=binding,binding_error=str(error),
                        lower_gate_samples_accepted=True,lower_gate_execution_identity_restored=True,
                        unique_lower_archive_counter_retained=True,exact_zero_requested=True),started)
                    return
                capture_clear=(self.clear_committed_exhausted_capture() if recovery is not None and recovery['allowed'] else None)
                active=r85_active if r85_active is not None else dict(payload=copy.deepcopy(value),metadata=copy.deepcopy(metadata),
                    identity=self.admission_identity(),trajectory_id=self.active_trajectory_id,
                    active_reference_stamp=tuple(self.active_reference_stamp),proof=copy.deepcopy(proof),anchor_binding=binding,
                    correction_generation=self.known_scene_inputs['generation'],
                    ordered_coverage_index=None if self.continuous_route is None else self.continuous_route.next_index)
                # All preparation succeeded; no await or callback boundary occurs
                # between the original cascade binding and its matching receipt.
                if staged is not None:
                    self.cascade=staged[0];self.cascade_path_id=receipt['path_id']
                self.path_receipt=receipt;self.path_admission_active=active
                self.path_admission_hold=False;self.path_admission_replan_pending=False;self.reference_replan_pending=False
                self.reference_consistency_hold=False;self.reference_consistency_check=None
                if handoff is not None and (handoff.get('stopped_rebind',{}).get('allowed')
                        or handoff.get('diagnostic_heading_rebind')):
                    reset_transition(self,'path_rebind')
                if staged is not None:self.evidence.append(self.run/'navigation_cascade_paths.jsonl',staged[1])
                self.evidence.append(self.run/'cascade_path_history.jsonl',copy.deepcopy(self.path_receipt))
                self.archive_admission(value,metadata,dict(admitted=True,reason='exhausted_capture_rebound_after_all_execution_gates' if capture_clear is not None else 'continuous_fence_and_existing_execution_gates_passed',
                    proof=proof,handoff=handoff,anchor_binding=binding,capture_replacement=recovery,capture_clear=capture_clear,
                    synchronous_original_path_binding=staged is not None),started)
            else:
                # Existing execution guards still own final admission. Their
                # refusal never becomes an admitted path in this separate ledger.
                if previous_execution is not None:
                    for key,old in previous_execution.items():setattr(self,key,old)
                self.path_admission_hold=True;self.path_admission_replan_pending=True;self.publish_command()
                self.archive_admission(value,metadata,dict(admitted=False,reason='existing_execution_gate_rejected',proof=proof,
                    anchor_binding=binding,capture_replacement=recovery,lower_gate_execution_identity_restored=previous_execution is not None,exact_zero_requested=True),started)
            return
        # Lower sampling/archive identity remains distinct from executable release.
        # Every large copy/JSON template and the existing-writer enqueue is part
        # of the same original 100ms transaction. No pass record is durable until
        # the callback settles its single bounded batch outcome.
        binding_execution=dict(samples=self.samples,active_trajectory_id=self.active_trajectory_id,
            active_reference_stamp=self.active_reference_stamp,planning_start_state=copy.deepcopy(self.planning_start_state),
            execution_path_exhaustion=copy.deepcopy(getattr(self,'execution_path_exhaustion',None)),
            trajectory_archive_reference=getattr(self,'trajectory_archive_reference',None))
        promotion_snapshot={key:getattr(self,key) for key in
            ('cascade','cascade_path_id','path_receipt','path_admission_active','path_admission_hold',
             'path_admission_replan_pending','reference_replan_pending','reference_consistency_hold',
             'reference_consistency_check','heading_gate','pid_guard_needs_evaluation')}
        binding_generation=self.known_scene_inputs['generation'];before=self.replans
        wall=time.monotonic_ns() if received_wall_ns is None else received_wall_ns
        batch=None;final_source=None;source_small_raw='null';completion={}
        try:
            self.path_admission_hold=True
            with _r87_diag.span('scan.lower_acceptance',source_ns=self.pose_stamp):
                super().accept_spline(msg,metadata)
            if self.replans<=before or self.samples is None:
                raise ValueError('existing_execution_gate_rejected')
            indices=[0]
            for i in range(1,len(self.samples)):
                if np.linalg.norm(self.samples[i,:2]-self.samples[indices[-1],:2])>=1e-5:indices.append(i)
            points=self.samples[indices]
            receipt=dict(points_xyz=points.tolist(),
                source_sample_indices=indices,source_samples_count=len(self.samples),
                source_samples_float64_sha256=hashlib.sha256(np.asarray(self.samples,dtype='<f8').tobytes()).hexdigest(),
                path_id=str(self.request_id)+':'+str(self.waypoint_index)+':'+str(self.replans)+':'+str(self.active_trajectory_id),
                stamp_ns=int(self.active_reference_stamp[0])*1_000_000_000+int(self.active_reference_stamp[1]),
                received_wall_ns=wall,frame_id='camera_init',array_file=self.trajectory_archive_reference,
                request_id=self.request_id,waypoint_index=self.waypoint_index,
                trajectory_id=self.active_trajectory_id,metadata=copy.deepcopy(metadata))
            receipt['points_float64_sha256']=hashlib.sha256(np.asarray(points,dtype='<f8').tobytes()).hexdigest()
            with _r87_diag.span('scan.cascade_binding',source_ns=self.pose_stamp):
                staged=staged_cascade_binding(self,receipt)
            if not self._known_generation_ready() or self.known_scene_inputs['generation']!=binding_generation:
                raise ValueError('admitted_path_source_generation_changed')
            active=dict(payload=copy.deepcopy(value),metadata=copy.deepcopy(metadata),
                identity=self.admission_identity(),trajectory_id=self.active_trajectory_id,
                active_reference_stamp=tuple(self.active_reference_stamp),proof=copy.deepcopy(proof),anchor_binding=binding,
                correction_generation=self.known_scene_inputs['generation'],
                ordered_coverage_index=None if self.continuous_route is None else self.continuous_route.next_index)
            # Prepare the two mutating legacy post-commit actions privately.
            # Only references/scalars are promoted after the final source gate.
            from types import SimpleNamespace
            capture_clear=None;prepared_cascade=staged[0] if staged is not None else self.cascade
            prepared_exhaustion=self.execution_path_exhaustion
            prepared_pid_guard=self.pid_guard_needs_evaluation
            if recovery is not None and recovery['allowed']:
                capture_proxy=SimpleNamespace(cascade=copy.deepcopy(self.cascade),
                    execution_path_exhaustion=copy.deepcopy(self.execution_path_exhaustion),
                    pid_guard_needs_evaluation=self.pid_guard_needs_evaluation)
                capture_clear=PIDNavigation.clear_committed_exhausted_capture(capture_proxy)
                prepared_cascade=staged[0] if staged is not None else capture_proxy.cascade
                prepared_exhaustion=capture_proxy.execution_path_exhaustion
                prepared_pid_guard=capture_proxy.pid_guard_needs_evaluation
            prepared_heading=self.heading_gate
            if handoff is not None and (handoff.get('stopped_rebind',{}).get('allowed')
                    or handoff.get('diagnostic_heading_rebind')):
                heading_proxy=SimpleNamespace(heading_gate=copy.deepcopy(self.heading_gate),
                    request_id=self.request_id,waypoint_index=self.waypoint_index,
                    obstacle_resume_pending=getattr(self,'obstacle_resume_pending',False),
                    _teacher_recovery_token=copy.deepcopy(getattr(self,'_teacher_recovery_token',None)),
                    tilt_hold=getattr(self,'tilt_hold',False))
                reset_transition(heading_proxy,'path_rebind');prepared_heading=heading_proxy.heading_gate
            decision=dict(admitted=True,
                reason='exhausted_capture_rebound_after_all_execution_gates' if capture_clear is not None else 'continuous_fence_and_existing_execution_gates_passed',
                proof=proof,handoff=handoff,anchor_binding=binding,capture_replacement=recovery,capture_clear=capture_clear,
                synchronous_original_path_binding=staged is not None)
            if self.r85_physical_gate is None:
                raise ValueError('prepared_transaction_requires_R85_physical_gate')
            # Commit current in-memory control state after a fresh sensor check.
            # Optional recording and audit serialization never authorize motion.
            final_source=self.r85_physical_gate.fresh(self,binding,binding_generation)
            if not final_source['allowed']:
                raise ValueError('Current pose/cloud/planner generation is not fresh')
            self.cascade=prepared_cascade
            if staged is not None:self.cascade_path_id=receipt['path_id']
            self.path_receipt=receipt;self.path_admission_active=active
            self.execution_path_exhaustion=prepared_exhaustion;self.pid_guard_needs_evaluation=prepared_pid_guard
            self.heading_gate=prepared_heading
            self.path_admission_replan_pending=False;self.reference_replan_pending=False
            self.reference_consistency_hold=False;self.reference_consistency_check=None
            self.path_admission_hold=False;self.path_admission_sequence+=1
            self.path_admission_last_decision=dict(decision,sequence=self.path_admission_sequence,
                source_fresh=final_source['allowed'],callback_elapsed_wall_ms=(time.monotonic()-started)*1000)
            return
        except (ValueError,TypeError,KeyError,AttributeError,OverflowError,FloatingPointError,RuntimeError,OSError) as error:
            for key,old in binding_execution.items():setattr(self,key,old)
            for key,old in promotion_snapshot.items():setattr(self,key,old)
            self.path_admission_hold=True;self.path_admission_replan_pending=True
            self.last_spline_rejected='Current path rejected: '+str(error)
            self.message=self.last_spline_rejected+'; waiting for replan'
            self.path_admission_last_decision=dict(admitted=False,reason=str(error))
            self.publish_command()
            return

    def follow_checked_trajectory(self,*args,**kwargs):
        kwargs['tracking_pose']=self.pose
        result=follow_trajectory(*args,**kwargs)
        if not kwargs.get('return_steering'):return result
        self.ensure_cascade()
        # The gate and PD/PI use one spatial reference, including its short-tail
        # rule. Preview neither advances progress nor integrates a held sample.
        if _r96_cpu.PROBE is None:
            reference=self.cascade.preview_reference(self.pose)
        else:
            with _r96_cpu.span('embodied_navigation.control.preview_reference',self):
                reference=self.cascade.preview_reference(self.pose)
        yaw=reference['heading_rad']
        if getattr(self,'global_reference_settings',{'enabled':False})['enabled']:
            self.reference_consistency_check=direction_check(self.cascade,reference,
                self.profile['teacher_transition']['drive_heading_max_rad'],self.global_reference_settings)
            observed=observe_business_gate(self,'scan_runtime_direction_coherence',self.reference_consistency_check,
                {'checked_subarc_direction_incoherent_requires_replan'})
            self.reference_consistency_hold=not observed['allowed']
            if self.reference_consistency_hold:self.reference_replan_pending=True
        if not self.reference_consistency_hold:
            early_handoff_request(self,reference)
        velocity,rate,target,steering=result
        original=copy.deepcopy(steering)
        steering=dict(steering,heading=yaw,error=math.atan2(math.sin(yaw-math.atan2(self.rotation[1,0],self.rotation[0,0])),
            math.cos(yaw-math.atan2(self.rotation[1,0],self.rotation[0,0]))),
            heading_reference='same actual SCAN finite-arc spatial reference as cascade',
            original_lookahead_steering=original,
            cascade_projection=dict(path_id=self.cascade.path_id,path_sha256=self.cascade.path_sha,
                reference=copy.deepcopy(reference),
                actual_pose_stamp_ns=int(self.pose_stamp),navigation_ground_truth_used=False),
            direction_coherence=copy.deepcopy(getattr(self,'reference_consistency_check',None)))
        self.evidence.append(self.run/'navigation_heading_projection.jsonl',dict(
            request_id=self.request_id,waypoint_index=self.waypoint_index,
            source_pose_stamp_ns=int(self.pose_stamp),control_stamp_ns=self.get_clock().now().nanoseconds,
            steering=copy.deepcopy(steering),feedback=copy.deepcopy(self.feedback),
            navigation_ground_truth_used=False))
        return velocity,rate,target,steering
    def continuous_before_region(self,now,clock_ns):
        if self.automatic_reference is not None:
            return False
        ledger=self.continuous_route
        if ledger is None:return
        if ledger.points is None:
            ledger.activate(self.region_raw_pose,int(self.pose_stamp),clock_ns)
            self.waypoint_index=ledger.target_index
            self.region_arrival=ArrivalWindow(self.goals[self.waypoint_index])
        receipt=ledger.observe(self.region_raw_pose,int(self.pose_stamp),clock_ns,
            protected=self.obstacle_hold or self.tilt_hold or self.bridge_safety.get("state")!="ready")
        if receipt is not None:
            self.region_arrivals.append(copy.deepcopy(receipt))
            self.evidence.append(self.run/"navigation_region_passes.jsonl",receipt)
            self.path_admission_replan_pending=True
            self.last_reference=-math.inf
        if ledger.missed_pass_requires_replan() and self.coverage_replan_index!=ledger.next_index:
            self.coverage_replan_index=ledger.next_index
            self.evidence.append(self.run/"navigation_coverage_replans.jsonl",dict(
                ledger.status(),reason="ordinary_region_not_observed_replan_through_original_volume",
                source_pose_stamp_ns=int(self.pose_stamp),control_stamp_ns=clock_ns,
                old_trajectory_id=self.active_trajectory_id,safety_gates_preserved=True))
            self.samples=None;self.execution_path_exhaustion=None
            self.trajectory_association.reset();self.planning_start_state=None
            self.active_trajectory_id=None;self.active_reference_stamp=None
            self.last_reference=-math.inf
            return True
        return False
    def continuous_record_hard(self,receipt,clock_ns):
        if self.continuous_route is not None:
            self.continuous_route.accept_hard_receipt(receipt,clock_ns)
    def continuous_release_target(self):
        if self.automatic_reference is not None:
            return
        ledger=self.continuous_route
        if ledger is None or self.state!="running" or ledger.points is None:return
        target=ledger.release_hard_target()
        if target!=self.waypoint_index and ledger.next_index<=target:
            self.waypoint_index=target
            self.region_arrival=ArrivalWindow(self.goals[target])
            self.region_arrival.last_stamp=int(self.pose_stamp)
            self.region_arrival_evidence=None
    def execution_guard_route(self,start,goal):
        if self.automatic_reference is not None:
            bound=getattr(self,'_program_route_fence',None)
            if bound is not None and bound[0]==self._program_route_key():
                return self._program_guard_route
            return np.asarray(self.automatic_reference.points(), dtype=float)
        ledger=self.continuous_route
        return ledger.guard_route() if ledger is not None and ledger.points is not None else np.vstack([start,goal])
    def fixed_terminal_heading(self,index,start):
        if self.automatic_reference is not None:
            return self.automatic_reference.terminal_heading()
        ledger=self.continuous_route
        if ledger is not None and ledger.points is not None:return ledger.terminal_heading(index)
        delta=self.waypoints[index][:2]-start[:2]
        if np.linalg.norm(delta)<1e-6:raise ValueError("Frozen goal heading requires nondegenerate actual SLAM segment")
        return math.atan2(delta[1],delta[0])
    def planner_goal(self):
        if self.automatic_reference is not None:
            return np.asarray(self.automatic_reference.planner_goal(), dtype=float)
        ledger=self.continuous_route
        return (self.waypoints[ledger.next_index] if ledger is not None and ledger.points is not None
            else super().planner_goal())
    def request_plan(self):
        if pending_path_waiting(self):return
        if self.automatic_reference is not None:
            return super().request_plan()
        ledger=self.continuous_route
        index=None if ledger is None else ledger.next_index
        before=self.reference_requests
        result=super().request_plan()
        if self.reference_requests>before:self.scan_reference_coverage_index=index
        return result
    def planner_reference_points(self,goal):
        if self.automatic_reference is not None:
            return self.automatic_reference.points()
        if self.continuous_route is not None:
            points=self.continuous_route.reference_points(end_index=self.continuous_route.next_index)
            self.evidence.append(self.run/"navigation_global_reference_requests.jsonl",dict(
                self.continuous_route.status(),guide_points_xyz=copy.deepcopy(points),guide_count=len(points),
                local_body_goal=list(map(float,goal)),local_endpoint_coverage_index=self.continuous_route.next_index,
                fixed_hard_target_index=self.waypoint_index,
                frame_id="camera_init",guide_only_not_arrival=True,collision_or_support_certified=False,
                source_pose_stamp_ns=int(self.pose_stamp),control_stamp_ns=self.get_clock().now().nanoseconds))
            return points
        if not self.global_reference_settings['enabled']:return [goal]
        if self.segment_start is None:raise ValueError('Global planning reference lacks original measured segment activation')
        points,receipt=self.global_route_guides.remaining((self.request_id,self.waypoint_index),
            self.segment_start,goal,self.pose,self.global_reference_settings)
        self.evidence.append(self.run/'navigation_global_reference_requests.jsonl',dict(receipt,
            source_pose_stamp_ns=int(self.pose_stamp),control_stamp_ns=self.get_clock().now().nanoseconds,
            goals_definition_sha256=self.goals_definition_sha256))
        return points
    def record_segment_activation(self):
        key=(self.request_id,self.waypoint_index)
        if key not in self.segment_references:
            if self.segment_start is None or not np.array_equal(self.segment_start,self.pose):
                raise RuntimeError('Actual segment activation lacks its original SLAM pose')
            self.segment_references[key]=dict(segment_start=self.segment_start.tolist(),
                source_pose_stamp_ns=int(self.segment_start_pose_stamp_ns),control_stamp_ns=int(self.segment_started_ros_ns),
                feedback=copy.deepcopy(self.feedback),request_id=self.request_id,waypoint_index=self.waypoint_index)
            self.evidence.append(self.run/'navigation_segment_activation.jsonl',dict(
                **self.segment_references[key],navigation_ground_truth_used=False))
    def measured_region_arrival(self):
        self.record_segment_activation()
        if self.continuous_route is not None and self.continuous_route.next_index!=self.waypoint_index:
            self.reset_region_arrival("ordered_pass_coverage_pending")
            return False,False
        return super().measured_region_arrival()
    def ensure_cascade(self):
        if getattr(self,'_r105_terminal_context',None) is not None:
            _r105_terminal.validate_context(self)
            self.cascade.diagnostic_observe_only=dict(run=self.run,profile=diagnostic_profile_view(self.profile))
            return
        key=(self.request_id,self.waypoint_index)
        if self.cascade_key!=key:
            self.reference_replan_pending=False
            self.pid_prepared_output=None;self.pid_row=None;self._pid_row_receipt_snapshot=None
            goal=self.waypoints[self.waypoint_index]
            yaw=self.fixed_terminal_heading(self.waypoint_index,self.segment_start)
            core=(CausalHandoffProjectionController if self.continuous_route is not None else
                StableProjectionController if getattr(self,"global_reference_settings",{"enabled":False})["enabled"] else Controller)
            self.cascade=core(self.profile['cascade'],goal.tolist(),yaw,':'.join(map(str,key)))
            self.cascade.minimal_simulation_baseline=minimal_baseline_enabled(self.profile)
            if 'r85_neutral_rate_brake_capture'in self.profile:
                self.cascade.configure_neutral_rate_brake_capture(self.profile)
            if 'r85_teacher_pre_zero_heading_p_taper'in self.profile:
                self.cascade.configure_teacher_pre_zero_heading_p_taper(
                    self.profile,self.run.name,self.handoff_scope_sha256)
            self.reference_consistency_hold=False;self.reference_consistency_check=None
            self.cascade_key=key;self.cascade_path_id=None
            self.evidence.append(self.run/'navigation_fixed_goal_references.jsonl',dict(
                request_id=self.request_id,waypoint_index=self.waypoint_index,
                segment_start=self.segment_start.tolist(),goal=goal.tolist(),heading_rad=yaw,
                source_pose_stamp_ns=self.segment_references[key]['source_pose_stamp_ns'],
                control_stamp_ns=self.segment_references[key]['control_stamp_ns'],
                goals_definition_sha256=self.goals_definition_sha256,fixed_goal=self.cascade.goal_record,
                fixed_goal_sha256=self.cascade.goal_sha,navigation_ground_truth_used=False))
        if (self.cascade.capture_stamp is None and self.path_receipt and
            (self.path_receipt['request_id'],self.path_receipt['waypoint_index'])==key and
            self.path_receipt['path_id']!=self.cascade_path_id):
            r=self.path_receipt
            self.cascade.set_path(r['points_xyz'],r['path_id'],r['stamp_ns'],r['received_wall_ns'],r['frame_id'],
                position=self.pose)
            self.cascade_path_id=r['path_id']
            self.evidence.append(self.run/'navigation_cascade_paths.jsonl',dict(
                **self.cascade.path_record,path_sha256=self.cascade.path_sha,
                trajectory_id=r['trajectory_id'],array_file=r['array_file'],
                source_sample_indices=r['source_sample_indices'],source_samples_count=r['source_samples_count'],
                source_samples_float64_sha256=r['source_samples_float64_sha256'],
                points_float64_sha256=r['points_float64_sha256'],navigation_ground_truth_used=False))
        if self.cascade is not None:
            self.cascade.diagnostic_observe_only=dict(run=self.run,profile=diagnostic_profile_view(self.profile))
    def _decode_ack_entries_reusing_lines(self,raw):
        # Parse-tree reuse only: causal selection, freshness and selected deepcopy
        # remain in get_ack on every call. Cached mutable trees stay private.
        prefix=b'{"entries":['
        suffix=b'],"schema":"teacher_known_velocity_ack/v1"}\n'
        if not(raw.startswith(prefix)and raw.endswith(suffix)):
            return json.loads(raw)['entries']
        body=raw[len(prefix):-len(suffix)]
        if not body:return []
        lines=body.split(b',\n',32)
        if (len(lines)>32 or any(not(line.startswith(b'{')and line.endswith(b'}'))
                or b'\n'in line or b'\r'in line for line in lines)):
            return json.loads(raw)['entries']
        cache=getattr(self,'_ack_parsed_line_lru',None)
        if cache is None:cache=OrderedDict()
        entries=[]
        try:
            for line in lines:
                if line in cache:
                    row=cache[line];cache.move_to_end(line)
                else:
                    row=json.loads(line)
                    if not isinstance(row,dict):return json.loads(raw)['entries']
                    cache[line]=row
                    if len(cache)>32:cache.popitem(last=False)
                entries.append(row)
        except (ValueError,TypeError,RecursionError):
            return json.loads(raw)['entries']
        self._ack_parsed_line_lru=cache
        return entries

    def _read_ack_entries_cached(self,path):
        # Cache parsing only. Every get_ack still selects against its own real
        # pose header; no freshness or causal result is memoized.
        def key(stat):return (stat.st_ino,stat.st_mtime_ns,stat.st_size)
        observed=key(path.stat())
        if (getattr(self,'_ack_entries_cache_key',None)==observed
                and hasattr(self,'_ack_entries_cache')):
            return self._ack_entries_cache
        with path.open('rb') as stream:
            before=key(os.fstat(stream.fileno()));raw=stream.read();after=key(os.fstat(stream.fileno()))
        if before!=after or len(raw)!=after[2]:raise ValueError('Actual ACK changed during same-FD read')
        entries=self._decode_ack_entries_reusing_lines(raw)
        if not isinstance(entries,list):raise ValueError('Actual ACK entries must be a list')
        self._ack_entries_cache_key=after;self._ack_entries_cache=entries
        return entries
    @_r96_cpu.observe('ack.get_ack')
    def get_ack(self,pose_stamp_ns=None):
        if pose_stamp_ns is None:
            if not self.feedback:return None
            pose_stamp_ns=self.feedback['stamp_ns']
        try:
            entries=self._read_ack_entries_cached(self.run/'closed_loop_executor_ack.json')
            from embodied_navigation.robot.unitree.policy.lower_loop_ack import configuration as lower_ack_configuration
            if lower_ack_configuration(self.profile['cascade'].get('r85_student_lower_loop_ack')) is not None:
                expected_scope=self.handoff_scope_sha256
                selected=next((r for r in reversed(entries)if r['stamp_ns']<=pose_stamp_ns
                    and r.get('run_id')==expected_scope),None)
            else:selected=next((r for r in reversed(entries)if r['stamp_ns']<=pose_stamp_ns),None)
            # Cached parse trees remain private; callers own only this small ACK.
            return copy.deepcopy(selected)
        except (OSError,KeyError,ValueError,TypeError):return None
    @_r96_cpu.observe('select')
    def select_pid_velocity(self,velocity,yaw_rate,target,allow_translation,mode_override=None):
        if _r96_cpu.PROBE is None:
            ns=self.get_clock().now().nanoseconds;wall=time.monotonic_ns()
        else:
            with _r96_cpu.span('select.ros_clock',self):
                ns=self.get_clock().now().nanoseconds;wall=time.monotonic_ns()
        if _r118_source_age.ENABLED:_r118_source_age.controller(self,'select.enter',ns,feedback=True)
        _r47_call = _r47_begin(self, 'select', ns, wall)
        try:
            _r47_mark(self, _r47_call, 'ensure_cascade_before')
            self.ensure_cascade()
            _r47_mark(self, _r47_call, 'ensure_cascade_after')
            controlled_settle=bool(getattr(self.heading_gate,'controlled_settle',False))
            controlled_braking=bool(getattr(self.heading_gate,'controlled_braking',False))
            active_settle=controlled_settle or controlled_braking
            mode=mode_override or ('hold'if self.obstacle_hold or getattr(self,'reference_consistency_hold',False) or self.heading_gate.phase=='pre_turn'
                or self.heading_gate.phase=='settle' and not active_settle
                else 'brake'if controlled_braking else 'drive'if allow_translation else 'turn')
            handoff_failure=getattr(self.heading_gate,'handoff_failure',None)
            if handoff_failure:
                self.state='failed';self.message='主动转向交接失败：'+str(handoff_failure);mode='hold'
            heading_reference=copy.deepcopy(self.steering);gate_heading=self.heading_gate.heading
            if self.imu_source_buffer is not None and self.feedback is not None and self.paired_imu is None:
                paired=self.imu_source_buffer.pair(self.feedback['stamp_ns'],None,ns)
                if paired is not None:
                    self.paired_imu=copy.deepcopy(paired)
                    self.imu_source_buffer.event(self.feedback['stamp_ns'],ns,wall,'current_pose_pair_from_fifo',paired)
            _r47_mark(self, _r47_call, 'get_ack_before')
            ack=self.get_ack()
            _r47_mark(self, _r47_call, 'get_ack_after')
            guard=None
            if self.feedback is None or self.paired_imu is None or ack is None:guard='actual_causal_feedback_or_ack_missing'
            if handoff_failure:guard='active_turn_handoff_failure:'+str(handoff_failure)
            context=(self.request_id,self.waypoint_index)
            previous=getattr(self,'_pre_turn_hold_seed_context',None)
            if previous is not None:
                previous=dict(previous,uninterrupted=bool(self.cascade.protected
                    and self.cascade.protection_reason=='external_hold' and not self.cascade.recovery
                    and self.cascade.last_math_stamp is None
                    and self.cascade.last_clock==previous['clock_ns'] and self.cascade.last_wall==previous['wall_ns']))
            unload_event=None
            if getattr(self.heading_gate,'integral_unload_config',None)is not None and mode=='brake' and guard is None:
                unload_event=self.heading_gate.bind_integral_unload(self.run.name,self.handoff_scope_sha256,
                    self.cascade.path_sha,self.cascade.goal_sha,ns)
                if self.heading_gate.handoff_failure:guard='active_turn_handoff_failure:'+self.heading_gate.handoff_failure
            update_args=dict(guard_reason=guard,mode_override=mode,
                external_turn_heading=gate_heading if mode in ('turn','brake') and (self.heading_gate.phase=='align' or active_settle) else None)
            if unload_event is not None:update_args['pre_zero_unload_event']=unload_event
            _r47_mark(self, _r47_call, 'core_update_before')
            cmd,diagnostic=self.cascade.update(self.feedback,self.paired_imu,ack,ns,wall,**update_args)
            _r47_mark(self, _r47_call, 'core_update_after')
            if diagnostic.get('controller_updated') and diagnostic.get('kind')=='ordinary_preT0_recapture':
                self.evidence.append(self.run/'ordinary_preT0_attempts.jsonl',diagnostic['preT0_attempt_disqualified'])
            seed_wait=_pre_turn_external_hold_seed_wait(previous,self.cascade,self.heading_gate,
                context,mode,guard,diagnostic,cmd,ns,wall)
            # Consume the token on any non-pre-turn call; a duplicate or another
            # protection cannot reuse this single valid-source seed exception.
            self._pre_turn_hold_seed_context=(dict(cascade=self.cascade,context=context,clock_ns=ns,wall_ns=wall,
                gate_context=copy.deepcopy(self.heading_gate.transition_context),purpose=self.heading_gate.transition_purpose)
                if mode=='hold' and self.heading_gate.phase=='pre_turn' and guard is None
                and not getattr(self.heading_gate,'handoff_active',False)
                and diagnostic.get('mode')=='protect' and diagnostic.get('reason')=='external_hold'
                and diagnostic.get('failure_latched')is None else None)
            if seed_wait:diagnostic['handoff_seed_recovery_wait']=True
            if (getattr(self.heading_gate,'handoff_active',False)
                    and (guard is not None or diagnostic.get('mode') in ('protect','recovering')
                         and not (mode=='hold' and diagnostic.get('reason')=='external_hold'))
                    and not seed_wait):
                self.heading_gate.fail_handoff('cascade_source_or_control_protection',ns/1e9)
                self.state='failed';self.message='主动转向交接失败：'+str(self.heading_gate.handoff_failure)
            if diagnostic.get('failure_latched'):
                self.state='failed';self.message='串级真实输入保护锁定：'+str(diagnostic['failure_latched'])
            self.cascade_last_row=diagnostic;self.pid_guard_needs_evaluation=(bool(diagnostic.get('controller_updated'))
                or self.control_clock_hold.guard_required)
            self.reference_replan_pending=(getattr(self,'reference_consistency_hold',False)
                or diagnostic['mode']=='reference_constraint_hold')
            _r47_mark(self, _r47_call, 'postcore_protection_logic_complete')
            desired=cmd.copy()
            # Sample only time already elapsed at the final slew decision.
            # Later native guard time is neither predicted nor credited here.
            cmd=(np.zeros(3)if diagnostic['mode']in ('protect','recovering','pre_turn','settle','path_end_hold','reference_constraint_hold','ordinary_zero_hold')
                else slew(self.command,desired,self.get_clock().now().nanoseconds/1e9-self.last_command_time,[.6,.6,.8]))
            if diagnostic['mode']=='ordinary_zero_hold':
                cmd[:]=0.;launch_constraint=dict(reason='identity_bound_ordinary_terminal_zero',exact_body_zero=True)
            else:cmd,launch_constraint=constrain_exactzero_launch(self.command,desired,cmd,diagnostic['mode'],self.rotation[:2,:2])
            self.pid_prepared_output=cmd.copy();world_direction=self.rotation[:2,:2]@cmd[:2]
            if np.linalg.norm(world_direction)>1e-9:self.steering['direction']=world_direction.tolist()
            self.pid_records+=1
            pid_receipt_bytes=_pid_receipt_json_bytes(self)
            self.pid_row=dict(schema='teacher_closed_loop_cascade_tick/v1',sequence=self.pid_records,
                cascade_sequence=self.pid_records,control_pose_stamp_ns=int(self.pose_stamp),source_pose_stamp_ns=int(self.pose_stamp),
                paired_imu_stamp_ns=None if self.paired_imu is None else self.paired_imu['stamp_ns'],
                control_stamp_ns=ns,compute_ros_clock_ns=ns,compute_monotonic_wall=wall/1e9,
                feedback=copy.deepcopy(self.feedback),imu=copy.deepcopy(self.paired_imu),ack=copy.deepcopy(ack),
                heading_gate_reference=dict(phase=self.heading_gate.phase,locked_heading=gate_heading,steering=heading_reference,
                    controlled_settle=controlled_settle,controlled_braking=controlled_braking,settle_control_stage=getattr(self.heading_gate,'settle_control_stage',None),
                    zero_handoff_count=getattr(self.heading_gate,'zero_handoff_count',None),
                    last_zero_handoff=copy.deepcopy(getattr(self.heading_gate,'last_zero_handoff',None))),
                path_receipt=None if minimal_baseline_enabled(self.profile) or pid_receipt_bytes is not None or self.path_receipt is None else {k:(_copy_path_receipt_metadata(v) if k=='metadata' else copy.deepcopy(v)) for k,v in self.path_receipt.items()if k!='points_xyz'},cascade=diagnostic,
                control_pose=self.pose.tolist(),control_quaternion=self.pose_quaternion,control_rotation=self.rotation.tolist(),
                request_id=self.request_id,waypoint_index=self.waypoint_index,trajectory_id=self.active_trajectory_id,
                trajectory_archive_file=self.trajectory_archive_reference,mode=diagnostic['mode'],
                checked_target=np.asarray(target).tolist(),goal=self.waypoints[self.waypoint_index].tolist(),
                actual_pid_world_direction=world_direction.tolist(),steering_direction=self.steering['direction'],
                desired_body_command=desired.tolist(),prepared_after_slew_command=cmd.tolist(),launch_output_constraint=launch_constraint,
                navigation_ground_truth_used=False)
            self._pid_row_receipt_snapshot=(self.pid_row,pid_receipt_bytes) if pid_receipt_bytes is not None else None
            if getattr(self.heading_gate,'quiet_target_enabled',False):
                self.pid_row['quiet_handoff_binding']=self.heading_gate._quiet_binding(self.heading_gate.transition_context,
                    self.heading_gate.transition_purpose,self.handoff_scope_sha256,diagnostic.get('path_id'),
                    diagnostic.get('path_sha256'),diagnostic.get('fixed_goal_sha256'))
            _r47_bind(self, _r47_call, self.pid_row)
            _r47_mark(self, _r47_call, 'postcore_row_complete')
            if _r118_source_age.ENABLED:_r118_source_age.controller(self,'select.row_complete',ns,self.pid_row)
            return cmd[:2],float(cmd[2])
        finally:
            _r47_end(self, _r47_call)
    def evaluate_motion_guard(self,*args):
        _r47_call = _r47_begin(self, 'guard')
        _r47_bind(self, _r47_call, getattr(self, 'pid_row', None))
        _r47_guard_dimensions(self, _r47_call, args)
        try:
            _r47_mark(self, _r47_call, 'native_geometry_guard_before')
            # Default-OFF slow-guard window: per-thread CPU/scheduler and gc
            # deltas at both ends, plus C-watchdog stack blocks only if the
            # guard runs long. The guard call itself is unchanged.
            _guard_window=_r86_probe_guard_begin()
            try:
                result=self.record_native_guard(*args)
            finally:
                _r86_probe_guard_end(_guard_window,_r47_call)
            _r47_mark(self, _r47_call, 'native_geometry_guard_after')
            _r47_mark(self, _r47_call, 'retain_native_context_before')
            retain_native_context(self,args)
            _r47_mark(self, _r47_call, 'retain_native_context_after')
            _r47_mark(self, _r47_call, 'remember_native_guard_before')
            remember_native_guard(self)
            _r47_mark(self, _r47_call, 'remember_native_guard_after')
            if self.pending_guard_row is not None:
                self.pending_guard_row.update(cloud_array_file=self.guard_cloud_receipt['cloud_array_file'],
                    cloud_raw_payload_file=self.guard_cloud_receipt['cloud_raw_payload_file'],
                    cloud_archive_sequence=self.guard_cloud_receipt['cloud_archive_sequence'])
                if self.pid_row:self.pending_guard_row.update(pid_sequence=self.pid_row['sequence'],
                    cascade_sequence=self.pid_row['sequence'],pid_compute_ros_clock_ns=self.pid_row['compute_ros_clock_ns'],
                    pid_control_pose_stamp_ns=self.pid_row['control_pose_stamp_ns'])
            self.pid_guard_needs_evaluation=False;self.control_clock_hold.guard_completed()
            _r47_mark(self, _r47_call, 'guard_row_complete')
            return result
        finally:
            _r47_end(self, _r47_call)
    def observe_ordinary_terminal_protection(self,ns):
        owner=getattr(self,'terminal_parking_owner',None)
        if owner is None or getattr(self,'terminal_parking_request',None)!=self.request_id or owner.ready_stamp is None:return
        reason=owner.failure
        # Completed normal shutdown is a distinct lifecycle edge. Existing
        # physical/control failure remains recorded even after a formal receipt.
        completed=False
        try:
            final=json.loads((self.run/'mission46_status.json').read_text())
            completed=(final.get('run_id')==self.run.name and final.get('stage')=='completed' and
                       final.get('final_parking_verified')is True)
        except (OSError,ValueError,TypeError):pass
        if reason is None and not completed:
            if hasattr(self,'_known_generation_ready') and not self._known_generation_ready():reason='known_scene_source_or_epoch_protection'
            elif self.obstacle_hold:reason='dynamic_obstacle_protection'
            elif getattr(self,'tilt_hold',False):reason='tilt_protection'
            elif self.state=='failed':reason='terminal_controller_failure'
        if reason is not None:
            from embodied_navigation.common.observation import api as _observer_api; diagnostic_enabled = _observer_api('diagnostic_observation').enabled; diagnostic_record = _observer_api('diagnostic_observation').record
            observed_business=False
            if diagnostic_enabled(self.profile):
                from embodied_navigation.control.parking.ordinary_parking import diagnostic_business_failure
                observed_business=(diagnostic_business_failure(reason)
                    and owner.diagnostic_failure_is_observed(dict(run=self.run,profile=diagnostic_profile_view(self.profile)))
                    and getattr(self,'ordinary_terminal_failure_latched',None)is None)
            if observed_business:
                now=time.monotonic();source_ready=False
                try:
                    from embodied_navigation.robot.unitree.policy.lower_loop_ack import input_binding
                    ack=self.get_ack();input_binding(ack)
                    source_ready=bool(self._known_generation_ready() and not self.obstacle_hold
                        and not getattr(self,'tilt_hold',False) and self.state!='failed'
                        and self.raw_imu.fresh(now,ns/1e9)
                        and all(0<=now-stamp<.3 for stamp in (self.pose_updated,self.cloud_updated,self.bridge_updated))
                        and self.bridge_safety.get('state')=='ready'
                        and type(ack['stamp_ns'])is int and 0<=ns-ack['stamp_ns']<300_000_000
                        and ack['stamp_ns']<=self.pose_stamp and 0<=now-ack['received_wall_ns']/1e9<.3)
                except (AttributeError,KeyError,TypeError,ValueError,IndexError,OverflowError):source_ready=False
                if source_ready:
                    if diagnostic_record(self.run,self.profile,'ordinary_terminal_business_failure_latch',reason,
                        dict(control_stamp_ns=int(ns),request_id=self.request_id,
                            original_business_failure_retained=True,original_owner_failure=owner.failure,
                            actual_source_and_ACK_fresh=True,normal_control_and_zero_phases_unchanged=True,
                            formal_parking_pass=False))is not True:
                        raise RuntimeError('ordinary_terminal_business_observation_not_recorded')
                    return
                reason='actual_source_or_ACK_protection_during_observed_parking_business_failure'
            owner.guard(reason)
            if getattr(self,'ordinary_terminal_failure_latched',None)is None:
                self.ordinary_terminal_failure_latched=dict(request_id=self.request_id,
                    reason=owner.failure,clock_ns=ns,ready_pose_stamp_ns=owner.ready_stamp,
                    ordinary_parking=owner.evidence(),navigation_ground_truth_used=False)
                self.evidence.append(self.run/'ordinary_terminal_failure_history.jsonl',self.ordinary_terminal_failure_latched)
            self.state='failed';self.message='普通停车失败锁定：'+str(owner.failure)

    @_callback_cost_observe('control')
    @_r96_cpu.observe('embodied_navigation.navigation.pid_navigation')
    def control(self,now):
        failed=getattr(self,'ordinary_terminal_failure_latched',None)
        if failed is not None and failed.get('request_id')==self.request_id:
            self.state='failed';self.publish_command();return
        gate=self.heading_gate;clock_ns=self.get_clock().now().nanoseconds
        if getattr(gate,'integral_unload_config',None)is not None:gate.check_integral_unload_deadline(clock_ns)
        if (getattr(gate,'handoff_active',False) and gate.handoff_deadline_s is not None
                and clock_ns/1e9>gate.handoff_deadline_s):
            gate.fail_handoff('active_handoff_deadline_exceeded',clock_ns/1e9)
        if getattr(gate,'handoff_failure',None):
            self.state='failed';self.message='主动转向交接失败：'+str(gate.handoff_failure)
            self.publish_command();return
        try:
            return self._control_candidate(now)
        except Exception as error:
            # A reference/handoff error must stop this tick, not escape the
            # executor and rely only on the 300 ms downstream watchdog.
            self.state='failed'
            self.message='V33控制参考异常，停车：'+type(error).__name__+': '+str(error)
            self.reference_replan_pending=False
            self.publish_command()
            self.evidence.append(self.run/'navigation_controller_errors.jsonl',dict(
                error_type=type(error).__name__,error=str(error),command_after_stop=list(self.command),
                control_stamp_ns=self.get_clock().now().nanoseconds,
                source_pose_stamp_ns=int(self.pose_stamp),navigation_ground_truth_used=False))
    @_r96_cpu.observe('embodied_navigation.control.candidate')
    def _control_candidate(self,now):
        self.poll_corridor(now)
        # The wall timer can run twice while real ROS /clock is unchanged.
        # Before any PI, heading, arrival or geometry operation, emit exact
        # zero and leave original source receipts untouched. On clock advance,
        # a new actual geometry check is mandatory before any nonzero command.
        if self.control_clock_hold.before_control(self,self.get_clock().now().nanoseconds,round(now*1e9)):
            return
        if _r105_terminal.control(self,now):return
        commit_pending_path(self)
        self.request_admission_replan_if_due(now)
        if self.path_admission_hold:
            if self.interrupt_active_handoff('active_handoff_path_admission_hold'):
                self.publish_command();return
            return self.control_admission_hold(now)
        if self.profile.get('mission46_required'):
            from embodied_navigation.navigation.mission.mission46_guard import mission_hold
            hold,reason=mission_hold(self.run,self.request_id,self.region_arrivals,
                self.goals_definition_sha256,self.get_clock().now().nanoseconds,now)
            if hold:
                if self.interrupt_active_handoff('active_handoff_mission_hold'):
                    self.publish_command();return
                self.message='Teacher46 transition: '+reason
                # A reached phase endpoint retains the existing feedback hold.
                # A connector pause is a zero VELOCITY request to the live
                # Teacher, never a zero action or a second joint controller.
                if self.state=='succeeded' and self.final_bundle is not None:
                    return self.control_parking(now)
                self.apply_tilt_guard(now,self.get_clock().now().nanoseconds/1e9)
                self.publish_command();return
        self.continuous_release_target()
        if self.state=='succeeded' and self.final_bundle is not None:return self.control_parking(now)
        if self.state=='running'and self.samples is not None and self.waypoint_index==len(self.waypoints)-1 and self.segment_start is not None:
            self.ensure_cascade()
            self.final_bundle=self.final_bundle_reference()
        return super().control(now)
    def final_bundle_reference(self):
        """Build the last-waypoint bundle from a read-only receipt snapshot.

        Only self.path_receipt is replaced elsewhere; it is never mutated in
        place, so the bundle keeps every original key/value and identity without
        a per-waypoint points_xyz/metadata copy. The R96 CPU probe stays an
        observation-only hook: it times the same construction under its own
        stage label but never forces a deep copy.
        """
        receipt=self.path_receipt
        if _r96_cpu.PROBE is None:
            path_receipt=_final_path_receipt_snapshot(receipt)
        else:
            path_receipt=_r96_cpu.receipt_copy(_final_path_receipt_snapshot,receipt,self,
                stage='embodied_navigation.control.final_bundle_receipt_snapshot')
        return dict(index=self.waypoint_index,start=self.segment_start.copy(),
            trajectory_id=self.active_trajectory_id,array=self.trajectory_archive_reference,
            path_receipt=path_receipt,reference_stamp=tuple(self.active_reference_stamp))
    def request_admission_replan_if_due(self,now):
        if pending_path_waiting(self):return
        pending=(self.reference_replan_pending or self.path_admission_replan_pending or self.path_admission_hold)
        # A previous goal's pending replan must wait for inherited control to
        # activate the next segment from a fresh, unprotected actual SLAM pose.
        if (pending and self.state=='running' and now-self.last_reference>3.
            and (not getattr(self,'global_reference_settings',{'enabled':False})['enabled']
                or self.segment_start is not None)):
            self.evidence.append(self.run/'reference_replan_events.jsonl',dict(
                reason='path_admission_rejection' if self.path_admission_replan_pending or self.path_admission_hold else 'reference_constraint_hold',
                admission_reason=None if self.path_admission_last_decision is None else self.path_admission_last_decision['reason'],
                request_id=self.request_id,waypoint_index=self.waypoint_index,trajectory_id=self.active_trajectory_id,
                source_pose_stamp_ns=int(self.pose_stamp),control_stamp_ns=self.get_clock().now().nanoseconds,
                navigation_ground_truth_used=False))
            before=self.reference_requests;self.request_plan()
            if self.reference_requests>before:
                self.reference_replan_pending=False;self.path_admission_replan_pending=False
    def control_admission_hold(self,now):
        # Keep the inherited deadlines, measured IMU/bridge protection and
        # source-loss failure active while no execution-admitted path exists.
        # Holding does not tick the cascade, heading or arrival integrators.
        ns=self.get_clock().now().nanoseconds;ros_now=ns/1e9
        if self.state!='running':self.publish_command();return
        if (self.segment_start is not None and not self.goals[self.waypoint_index].legacy
                and self.segment_deadline_exceeded(ns)):
            self.reset_region_arrival('failed')
            self.state,self.message='failed','当前航点超过仿真时间限制，停车'
            self.publish_command();return
        protected=self.apply_tilt_guard(now,ros_now)
        _sensor_hold_row=_sensor_hold_begin(self,'path_admission',now,ns)
        if self.state=='failed':
            _sensor_hold_end(self,_sensor_hold_row,None)
            self.reset_region_arrival('failed');self.publish_command();return
        stale=(now-self.pose_updated>self.pose_timeout or now-self.cloud_updated>self.cloud_timeout
            or not self.raw_imu.fresh(now,ros_now))
        _r102_progress.note(self,'admission_hold',True,
            'original_stale' if stale else 'original_protected' if protected else 'path_admission_hold',ns)
        if stale:
            self.reset_region_arrival('stale');self.stale_since=self.stale_since or now
            self.message='SLAM、点云或原始 IMU 超时，停车等待传感器'
            if now-self.stale_since>8.:self.state,self.message='failed','SLAM、点云或原始 IMU 连续失联超过 8 秒'
            _sensor_hold_end(self,_sensor_hold_row,stale)
        elif self.state=='failed':self.reset_region_arrival('failed');_sensor_hold_end(self,_sensor_hold_row,stale)
        elif protected:self.reset_region_arrival('protected');_sensor_hold_end(self,_sensor_hold_row,stale)
        else:
            self.stale_since=None;self.reset_region_arrival('path_admission_hold')
            _sensor_hold_end(self,_sensor_hold_row,stale)
            if self.segment_start is None:
                self.segment_start=self.pose.copy();self.segment_started_ros=ros_now
                self.segment_started_ros_ns=ns;self.segment_start_pose_stamp_ns=int(self.pose_stamp)
                self.record_segment_activation()
            # Path admission must not starve the existing obstacle-clear
            # observation gate. This checks the original native geometry at
            # exact zero; it cannot release path admission or arrival dwell.
            if (getattr(self,'global_reference_settings',{'enabled':False})['enabled']
                    and self.obstacle_hold and (now-self.last_obstacle_check>=.05 or self.pid_guard_needs_evaluation)):
                self.publish_command()
                goal=self.waypoints[self.waypoint_index]
                target=np.asarray((self.steering or {}).get('target',goal),dtype=float)
                direction=np.asarray((self.steering or {}).get('direction',(goal-self.pose)[:2]),dtype=float)
                result=self.evaluate_motion_guard(self.cloud,self.pose,target,direction,self.execution_guard_route(self.segment_start,goal))
                self.obstacle_result=result;self.last_obstacle_check=now
                if result[0] or not self.heading_gate.stopped(ros_now,now):
                    self.obstacle_clear_since=None
                else:
                    self.obstacle_clear_since=self.obstacle_clear_since or ros_now
                    if ros_now-self.obstacle_clear_since>=1. and getattr(self,'_actual_clear_guard_ready',False):
                        self.obstacle_hold=False;self.obstacle_resumes+=1
                        self.obstacle_stopped_duration=ros_now-(ros_now if self.obstacle_hold_started_ros is None else self.obstacle_hold_started_ros)
                        arm_teacher_recovery(self)
                        self.obstacle_resume_pending=True;reset_transition(self,'dynamic_recovery')
                        self.samples=None;self.execution_path_exhaustion=None
                        # Retain the admission hold. Only a newly associated,
                        # execution-admitted curve can authorize future motion.
                if self.pending_guard_row is not None:
                    row=self.pending_guard_row
                    row.update(obstacle_hold_after=bool(self.obstacle_hold),clear_start_after_s=self.obstacle_clear_since,
                        after_control_ros_clock_ns=ns,command_after_control=list(self.command),
                        zero_requested_after_control=True,state_after_control=self.state,
                        admission_hold_observation_only=True,path_admission_hold_after=self.path_admission_hold,
                        obstacle_resumes_after=self.obstacle_resumes,evidence_queue_error=self.evidence.error)
                    record_clear_release(self,row)
                    self.evidence.append(self.run/'navigation_guard_history.jsonl',row);self.pending_guard_row=None
        self.publish_command()
    def control_parking(self,now):
        ns=self.get_clock().now().nanoseconds
        bundle=self.final_bundle;index=self.waypoint_index
        self.waypoint_index=bundle['index'];self.active_reference_stamp=bundle['reference_stamp']
        self.active_trajectory_id=bundle['trajectory_id'];self.trajectory_archive_reference=bundle['array']
        self.path_receipt=bundle['path_receipt'];self.segment_start=bundle['start']
        goal=self.waypoints[self.waypoint_index]
        if ordinary_parking_enabled(self.profile):
            self.cascade.begin_ordinary_terminal(int(ns),round(float(self.goals[self.waypoint_index].timeout_sim_s)*1e9))
            self.terminal_parking_owner=self.cascade.ordinary_parking;self.terminal_parking_request=self.request_id
        self.steering=dict(direction=(goal-self.pose)[:2].tolist(),heading=self.cascade.goal_yaw,
            heading_error=0.,exhausted=False)
        _parking_sensor_row=None
        _parking_stale=None
        try:
            _parking_sensor_row=_sensor_hold_begin(self,'ordinary_parking',now,ns)
            stale=(now-self.pose_updated>.3 or now-self.cloud_updated>.3 or self.paired_imu is None)
            _parking_stale=stale
            _parking_tilt=self.apply_tilt_guard(now,ns/1e9)
            _sensor_parking_protection(self,_parking_sensor_row,_parking_tilt)
            if _parking_tilt or stale or self.obstacle_hold:
                self.select_pid_velocity(np.zeros(2),0.,goal,False,mode_override='hold')
                self.publish_command();return
            velocity,yawrate=self.select_pid_velocity(np.zeros(2),0.,goal,True,mode_override='capture')
            result=self.evaluate_motion_guard(self.cloud,self.pose,goal,self.steering['direction'],self.execution_guard_route(bundle['start'],goal))
            if result[0]:
                self.message='终点实际点云保护停车'
                self.publish_command()  # Stop immediately and archive the guarded candidate.
                self.select_pid_velocity(np.zeros(2),0.,goal,False,mode_override='hold')
                self.publish_command()
            else:self.message='实际SLAM固定终点主动捕获/位置朝向保持';self.publish_command(velocity,yawrate)
            if self.pending_guard_row:
                self.pending_guard_row.update(command_after_control=list(self.command),zero_requested_after_control=not any(self.command),
                    state_after_control=self.state,after_control_ros_clock_ns=self.get_clock().now().nanoseconds,
                    obstacle_hold_after=self.obstacle_hold,evidence_queue_error=self.evidence.error)
                self.evidence.append(self.run/'navigation_guard_history.jsonl',self.pending_guard_row);self.pending_guard_row=None
        finally:
            self.waypoint_index=index
            _sensor_hold_end(self,_parking_sensor_row,_parking_stale)
    def publish_command(self,velocity=None,yaw_rate=0.,*,duplicate_ordinary_hold=None):
        handoff=getattr(self,'_bridge_safety_handoff',None)
        reservation=False
        if velocity is not None and handoff is not None:
            reservation=handoff.reserve(self)
            if not reservation:return handoff.cancel_publication(self)
        try:
            join=getattr(self,'_r104_parking_causal_join',None)
            prior_join_source=None if join is None else join.published_source
            # Any unsuccessful publication/source replacement invalidates the cache.
            # A successful original duplicate-preservation transaction may restore it.
            if join is not None:join.published(None)
            _r47_call = _r47_begin(self, 'publication', stopped=velocity is None)
        except BaseException:
            if reservation:handoff.end_reservation(self)
            raise
        try:
            from geometry_msgs.msg import Twist
            from std_msgs.msg import Bool
            # PUBLICATION_OBSERVER_BEGIN entry_snapshot
            from embodied_navigation.control.publication_ledger import PublicationLedger
            publication_before=PublicationLedger.begin(self,velocity is None)
            _r47_publication_anchor(self, _r47_call, publication_before)
            # PUBLICATION_OBSERVER_END entry_snapshot
            msg=Twist();ns=self.get_clock().now().nanoseconds
            if velocity is None:self.observe_ordinary_terminal_protection(ns)
            self.last_publication_ros_clock_ns=ns
            if velocity is not None:
                output=np.r_[velocity,yaw_rate]
                if self.pid_prepared_output is None or not np.allclose(output,self.pid_prepared_output,rtol=0,atol=1e-12):
                    raise RuntimeError('Cascade command changed after native guard')
                msg.linear.x,msg.linear.y,msg.angular.z=map(float,output)
            elif hasattr(self,'pid'):
                self.pid.reset('exact_zero_stop_or_protection');self.pid_prepared_output=None
            self.command=[msg.linear.x,msg.linear.y,msg.angular.z];self.last_command_time=ns/1e9
            gate=self.heading_gate
            if getattr(gate,'handoff_active',False) and velocity is None:
                now=time.monotonic()
                protected=(self.state!='running' or self.obstacle_hold or self.tilt_hold
                    or getattr(self,'reference_consistency_hold',False) or self.samples is None
                    or getattr(self,'path_admission_hold',False)
                    or now-self.pose_updated>self.pose_timeout or now-self.cloud_updated>self.cloud_timeout
                    or not self.raw_imu.fresh(now,ns/1e9)
                    or now-self.bridge_updated>.30 or self.bridge_safety.get('state')!='ready')
                if protected or getattr(gate,'settle_control_stage',None)!='authoritative_zero':
                    gate.fail_handoff('explicit_zero_protection_or_stop',ns/1e9)
                    self.state='failed';self.message='主动转向交接失败：'+str(gate.handoff_failure)
            note_final_zero=(velocity is None and not any(self.command)
                    and getattr(gate,'settle_control_stage',None)=='authoritative_zero'
                    and getattr(gate,'handoff_active',False) and not getattr(gate,'handoff_failure',None))
            row=getattr(self,'pid_row',None)
            _r47_bind(self, _r47_call, row)
            if row is not None:
                row.update(publish_ros_clock_ns=ns,command_after_slew=list(self.command),state=self.state,
                    obstacle_hold=self.obstacle_hold,alignment_hold=self.alignment_hold,stopped=velocity is None,
                    reset_count_after_publish=self.pid.resets,queue_error=self.evidence.error)
                f=row['feedback'];i=row['imu']
                source=dict(schema='actual_SLAM_SCAN_cascade_command_source/v1',cascade_sequence=row['sequence'],
                    source_pose_stamp_ns=row['source_pose_stamp_ns'],paired_imu_stamp_ns=row['paired_imu_stamp_ns'],
                    control_stamp_ns=row['control_stamp_ns'],command_after_slew=list(self.command),navigation_ground_truth_used=False,
                    source_pose_received_wall_ns=None if f is None else f['received_wall_ns'],
                    source_imu_received_wall_ns=None if i is None else i['received_wall_ns'],
                    source_pose_received_monotonic_wall=None if f is None else f['received_wall_ns']/1e9,
                    paired_imu_received_monotonic_wall=None if i is None else i['received_wall_ns']/1e9,
                    request_id=row['request_id'],trajectory_id=row['trajectory_id'],
                    waypoint_index=row['waypoint_index'],
                    goal_sha256=row['cascade']['fixed_goal_sha256'],path_sha256=row['cascade']['path_sha256'],
                    cascade_mode=row['cascade']['mode'],
                    parking_hold_declared_pose_stamp_ns=row['cascade']['parking_hold_declared_pose_stamp_ns'],
                    heading_transition_stage=getattr(gate,'settle_control_stage',None),
                    heading_transition_context=copy.deepcopy(getattr(gate,'transition_context',{})),
                    heading_zero_handoff_count=getattr(gate,'zero_handoff_count',None),
                    heading_last_zero_handoff=copy.deepcopy(getattr(gate,'last_zero_handoff',None)))
                if getattr(self.cascade,'terminal_reference',None) is not None:
                    from embodied_navigation.control.parking.terminal_reference import fields
                    source.update(fields(self.cascade.terminal_reference),
                        schema='actual_SLAM_terminal_fixed_goal_command_source/v1',path_geometry_present=False)
                if getattr(gate,'integral_unload_config',None)is not None and row['cascade']['mode']=='brake':
                    source['pre_zero_integral_unload']=copy.deepcopy(gate.integral_unload_event)
                source['ordinary_parking_stage']=row['cascade'].get('ordinary_parking_stage')
                source['ordinary_terminal_zero_event']=None
                if velocity is not None and row['cascade']['mode']=='ordinary_zero_hold' and not row['cascade']['protection_active']:
                    ordinary=self.cascade.ordinary_parking
                    if self.cascade.ordinary_zero_event is None:
                        self.cascade.ordinary_zero_event=make_event(self.run.name,self.request_id,
                            source['goal_sha256'],source['path_sha256'],row['sequence'],
                            ordinary.ready_stamp,ordinary.handoff_clock,row['sequence'],
                            terminal_reference=getattr(self.cascade,'terminal_reference',None))
                    marker=self.cascade.ordinary_zero_event
                    source.update(ordinary_terminal_zero_event=dict(marker),ordinary_zero_event_id=marker['event_id'],
                                  ordinary_zero_event_sequence=marker['event_sequence'])
                    if ordinary.first_zero_publication_clock is None:ordinary.first_zero_publication_clock=ns
                elif velocity is None:
                    source['ordinary_parking_stage']='revoked'
                tmp=self.run/'cascade_command_source.tmp'
                _r47_mark(self, _r47_call, 'source_tmp_write_before')
                tmp.write_text(json.dumps(source,allow_nan=False)+'\n')
                _r47_mark(self, _r47_call, 'source_tmp_write_after')
                _r47_mark(self, _r47_call, 'source_replace_before')
                tmp.replace(self.run/'cascade_command_source.json')
                _r47_mark(self, _r47_call, 'source_replace_after')
                if _r118_source_age.ENABLED:_r118_source_age.controller(self,'source.atomic_replace',ns,row)
                _r47_mark(self, _r47_call, 'PID_evidence_append_before')
                _append_pid_row_evidence(self,row)
                _r47_mark(self, _r47_call, 'PID_evidence_append_after')
                self.pid_row=None;self._pid_row_receipt_snapshot=None
            source_preservation=None
            if velocity is None and row is None:
                if duplicate_ordinary_hold is not None:
                    from embodied_navigation.control.safety.duplicate_source import preserve_existing_source, observe_unchanged_after_publish
                    source_preservation=preserve_existing_source(self,duplicate_ordinary_hold,ns,
                        command_before=publication_before['command_before'])
                if source_preservation is None or not source_preservation['allowed']:
                    revoked=dict(schema='cascade_command_explicit_zero_revocation/v1',navigation_ground_truth_used=False,
                        command_after_slew=[0.,0.,0.],ordinary_terminal_zero_event=None,ordinary_parking_stage='revoked',
                        request_id=self.request_id,publish_ros_clock_ns=ns)
                    tmp=self.run/'cascade_command_source.tmp'
                    _r47_mark(self, _r47_call, 'revoked_source_tmp_write_before')
                    tmp.write_text(json.dumps(revoked,allow_nan=False)+'\n')
                    _r47_mark(self, _r47_call, 'revoked_source_tmp_write_after')
                    _r47_mark(self, _r47_call, 'revoked_source_replace_before')
                    tmp.replace(self.run/'cascade_command_source.json')
                    _r47_mark(self, _r47_call, 'revoked_source_replace_after')
                    if source_preservation is not None:source_preservation.update(source_file_written=True,preservation_performed=False)
                else:source_preservation['preservation_performed']=True
            # PUBLICATION_OBSERVER_BEGIN actual_publish_start
            publication_started_wall_ns=PublicationLedger.publication_started()
            # PUBLICATION_OBSERVER_END actual_publish_start
            _r47_mark(self, _r47_call, 'DDS_publish_call_before')
            self.cmd_pub.publish(msg)
            join=getattr(self,'_r104_parking_causal_join',None)
            if join is not None:
                if row is not None:join.published(source)
                elif source_preservation is not None and source_preservation['allowed']:join.published(prior_join_source)
            _r47_mark(self, _r47_call, 'DDS_publish_return')
            if _r118_source_age.ENABLED:_r118_source_age.controller(self,'command.DDS_return',ns,row)
            self.counts['commands']+=1
            if getattr(gate,'quiet_target_enabled',False):
                record=None;core={}if row is None else row.get('cascade',{})
                ref={}if row is None else row.get('heading_gate_reference',{})
                f=None if row is None else row.get('feedback');i=None if row is None else row.get('imu')
                binding=gate._quiet_binding(gate.transition_context,gate.transition_purpose,self.handoff_scope_sha256,
                    self.cascade_path_id,None if self.cascade is None else self.cascade.path_sha,
                    None if self.cascade is None else self.cascade.goal_sha)
                normal=bool(row is not None and isinstance(f,dict) and isinstance(i,dict)
                    and 'received_wall_ns'in f and 'received_wall_ns'in i
                    and velocity is not None and self.state=='running' and gate.controlled_braking and gate.handoff_active
                    and core.get('mode')=='brake' and core.get('controller_updated')is True and core.get('protection_active')is False
                    and ref.get('phase')=='settle' and ref.get('controlled_braking')is True
                    and ref.get('settle_control_stage')=='controlled_braking' and row.get('quiet_handoff_binding')==binding
                    and row.get('request_id')==self.request_id and row.get('waypoint_index')==self.waypoint_index
                    and not self.obstacle_hold and not self.tilt_hold and not getattr(self,'reference_consistency_hold',False)
                    and not getattr(self,'path_admission_hold',False) and self.samples is not None
                    and not self.control_clock_hold.holding and not self.control_clock_hold.guard_required
                    and not self.pid_guard_needs_evaluation and self.evidence.error is None
                    and not row.get('launch_output_constraint',{}).get('applied',False))
                if normal:
                    record=dict(sequence=self.counts['commands'],publish_clock_ns=self.get_clock().now().nanoseconds,
                        publish_wall_ns=time.monotonic_ns(),control_clock_ns=row['compute_ros_clock_ns'],
                        control_wall_ns=core['compute_wall_ns'],source_pose_stamp_ns=row['source_pose_stamp_ns'],
                        paired_imu_stamp_ns=row['paired_imu_stamp_ns'],source_pose_received_wall_ns=f['received_wall_ns'],
                        source_imu_received_wall_ns=i['received_wall_ns'],command_body=list(self.command),normal_braking=True,**binding)
                if record is not None and getattr(gate,'integral_unload_config',None)is not None:
                    pi=core.get('velocity_PI',{})
                    record['pre_zero_integral_unload']=copy.deepcopy(pi.get('pre_zero_integral_unload'))
                    record['yaw_integral_state']=pi.get('integral_state',[None]*3)[2]
                    record['integral_dt_s']=pi.get('integral_dt_s')
                gate.note_quiet_publication(record)
            if note_final_zero:gate.note_zero_publication(self.get_clock().now().nanoseconds)
            _r47_mark(self, _r47_call, 'quiet_and_zero_note_after')
            # PUBLICATION_OBSERVER_BEGIN actual_publish_completed
            PublicationLedger.completed(self,publication_before,ns,publication_started_wall_ns)
            _r47_publication_anchor(self, _r47_call, completed=True)
            _r67_attach_known_frame_failure(self,_r47_call,publication_before)
            # PUBLICATION_OBSERVER_END actual_publish_completed
            self.freeze_pub.publish(Bool(data=bool(self.state!='running'or self.obstacle_hold or np.linalg.norm(self.command[:2])<.015)))
            if source_preservation is not None and source_preservation['allowed']:
                observe_unchanged_after_publish(self,source_preservation)
            return source_preservation
        finally:
            if reservation:handoff.end_reservation(self)
            _r47_end(self, _r47_call)


from embodied_navigation.common.shutdown import (
    ShutdownRequested as _ShutdownRequested, ShutdownSignals as _ShutdownSignals,
)


def _drain_executor(executor):
    """Join callbacks before context/node destruction and retrieve real errors."""
    try:
        executor.shutdown()
    finally:
        executor._executor.shutdown(wait=True)
    # Jazzy retains submitted Tasks after the spinner stops. Fetch every done
    # exception, including a callback that finished during pool shutdown.
    tasks = set(getattr(executor, '_futures', ()))
    tasks.update(getattr(executor, '_pending_tasks', {}))
    errors = []
    for task in tasks:
        if task.done() and not task.cancelled():
            try:
                task.result()
            except BaseException as error:
                errors.append(error)
    if errors:
        raise BaseExceptionGroup('Controller callbacks failed while draining', errors)


def _main(shutdown_signals):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.executors import MultiThreadedExecutor
    from embodied_navigation.common.time.cooperative_executor import make_executor, receipt as r90_executor_receipt, finish as r90_executor_finish
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    args,ros=p.parse_known_args();run=args.run.resolve();os.environ['DEMO_RUN_DIR']=str(run)
    from embodied_navigation.localization_mapping.known_scene_inputs import KnownSceneInputMixin
    class IntegratedNavigation(KnownSceneInputMixin,PIDNavigation):pass
    IntegratedNavigation._known_generation_ready = _r67_observe_known_generation_ready(
        KnownSceneInputMixin._known_generation_ready,KnownSceneInputMixin._control_candidate.__code__)
    IntegratedNavigation._admit_matched_spline = _r56_observe_admission_body(IntegratedNavigation._admit_matched_spline)
    rclpy.init(args=ros, signal_handler_options=SignalHandlerOptions.NO);node=IntegratedNavigation(run);executor=None
    try:
        # Start the new child only after the complete ROS node was constructed.
        from embodied_navigation.planning.admission.async_admission import enabled as r88_enabled, Admission
        if r88_enabled() and node.r85_physical_gate is not None:
            eager_send=os.environ.get('R89_EAGER_PHYSICAL_SEND','0')
            if eager_send not in ('0','1'):raise ValueError('R89_EAGER_PHYSICAL_SEND must be 0 or 1')
            node._r88_admission=Admission(node,eager_submit=eager_send=='1')
        from embodied_navigation.common.observation import api as _observer_api; install_r89_runtime_trace = _observer_api('r89_runtime_trace').install
        node._r89_trace_receipt=install_r89_runtime_trace(node)
        node._r96_cpu_receipt=_r96_cpu.install(node)
        executor=make_executor(MultiThreadedExecutor,node.profile,num_threads=2)
        node._r99_clock_probe_receipt=_r99_clock_probe.install(node,executor)
        node._r107_take_receipt=_r107_scan_take_probe.install(node,executor)
        node._r114_clock_service=_r114_clock_service.install(node,executor)
        node._r90_executor_receipt=r90_executor_receipt(executor)
        r91_selection=os.environ.get('R91_EMPTY_RETRY_YIELD','0')
        if r91_selection not in ('0','1'):
            raise ValueError('R91_EMPTY_RETRY_YIELD must be 0 or 1')
        node._r91_empty_retry_yield=None
        if r91_selection=='1':
            if (node._r90_executor_receipt.get('enabled') is not True
                    or node._r90_executor_receipt.get('worker_threads')!=2):
                raise ValueError('R91 requires the selected R90 sole-owner executor with two workers')
            from embodied_navigation.common.time.empty_retry_yield import install as install_r91_empty_retry_yield
            node._r91_empty_retry_yield=install_r91_empty_retry_yield(executor,enabled=True)
        node._r91_executor_receipt=dict(environment_key='R91_EMPTY_RETRY_YIELD',
            environment_value=r91_selection,candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT'),
            candidate=None if node._r91_empty_retry_yield is None else node._r91_empty_retry_yield.snapshot())
        node._r118_source_age_binding=_r118_source_age.bind('controller',run)
        executor.add_node(node)
    except BaseException:
        shutdown_signals.begin_cleanup()
        try:
            if executor is not None:
                try:
                    try:executor.shutdown()
                    finally:executor._executor.shutdown(wait=True)
                except Exception:pass
            _r96_cpu.detach()
            _r107_scan_take_probe.detach()
            _r99_clock_probe.detach()
            try:
                from embodied_navigation.common.observation import api as _observer_api; finish_r89_runtime_trace = _observer_api('r89_runtime_trace').finish
                try:finish_r89_runtime_trace(node)
                finally:
                    try:_r96_cpu.finish(node)
                    finally:_r99_clock_probe.finish(node)
            except Exception:pass
        finally:
            if node._r88_admission is not None:node._r88_admission.close()
        raise
    try:executor.spin()
    except _ShutdownRequested:pass
    finally:
        shutdown_signals.begin_cleanup()
        try:
            try:
                try:_drain_executor(executor)
                finally:
                    try:_r96_cpu.detach()
                    finally:
                        try:_r107_scan_take_probe.detach()
                        finally:_r99_clock_probe.detach()
            finally:
                try:
                    handoff=getattr(node,'_bridge_safety_handoff',None)
                    if handoff is not None:handoff.close(node,service_negative=rclpy.ok())
                    if rclpy.ok():node.publish_command()
                finally:
                    try:
                        if node.corridor_observer is not None:
                            remaining=node.corridor_observer.close()
                            if remaining is not None:node.evidence.append(run/'corridor_certificates.jsonl',remaining)
                            node.evidence.append(run/'corridor_worker_receipt.jsonl',dict(
                                submitted=node.corridor_observer.submitted,completed=node.corridor_observer.completed,
                                busy_diagnostic_snapshots=node.corridor_observer.busy_snapshots,
                                control_authority=False,navigation_ground_truth_used=False))
                    except Exception as error:
                        node.evidence.append(run/'corridor_input_rejections.jsonl',dict(
                            reason='shadow_close:'+type(error).__name__+':'+str(error),control_authority=False))
                    try:
                        try:
                            if node._r88_admission is not None:
                                exit_receipt=node._r88_admission.close()
                                if exit_receipt is not None:
                                    (run/'r88_async_physical_exit.json').write_text(json.dumps(exit_receipt,allow_nan=False)+'\n')
                        finally:
                            # New-resource cleanup cannot skip the existing
                            # source/archive drain when its own receipt fails.
                            if node.imu_source_buffer is not None:node.imu_source_buffer.finish()
                            node.event_archive.close();node.evidence.close()
                    finally:
                        (run/'navigation_pid_writer_receipt.json').write_text(json.dumps(dict(schema=1,
                            expected_pid_records=node.pid_records,expected_guard_records=node.guard_sequence,
                            expected_imu_records=node.imu_records,expected_cloud_archive_records=node.cloud_archive_records,
                            expected_cloud_callback_records=node.cloud_callback_records,queue_error=node.evidence.error,
                            expected_control_clock_hold_records=node.control_clock_hold.records,
                            # PUBLICATION_OBSERVER_BEGIN writer_expected_count
                            expected_command_publication_records=node.publication_records,
                            final_commands_published_count=node.counts['commands'],
                            # PUBLICATION_OBSERVER_END writer_expected_count
                            status='failed'if node.evidence.error else 'drained',navigation_verified=False))+'\n')
                        node.destroy_node();rclpy.try_shutdown()
        finally:
            try:
                try:
                    try:None
                    finally:r90_executor_finish(node,executor)
                finally:
                    from embodied_navigation.common.observation import api as _observer_api; finish_r89_runtime_trace = _observer_api('r89_runtime_trace').finish
                    try:finish_r89_runtime_trace(node)
                    finally:
                        try:_r96_cpu.finish(node)
                        finally:_r99_clock_probe.finish(node)
            finally:
                try:
                    _r118_source_age.finish(run)
                    _r114_clock_service.finish(node)
                    _r47_flush(node)
                finally:
                    try:_sensor_hold_flush(node)
                    finally:
                        try:_corrected_delivery_flush(getattr(node,'_r74_corrected_delivery_witness',None))
                        finally:_callback_cost_flush(getattr(node,'_r74_callback_cost_witness',None))
def main():
    with _ShutdownSignals() as shutdown_signals:
        _main(shutdown_signals)


if __name__=='__main__':main()
