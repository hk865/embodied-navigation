"""Bind an existing stop event to exact guard evidence; no control authority."""
from contextvars import ContextVar
from functools import wraps
import copy
import hashlib
import json
import time

SCHEMA = 'teacher_dynamic_stop_event/v1'
FIELDS = ('request_id', 'waypoint_index', 'sequence', 'clock_ns',
          'cloud_source_ns', 'cloud_sha256', 'trajectory_id',
          'reference_stamp', 'stop_count')
AUDIT_LIMIT_B = 8192
AUDIT_RECORD_LIMIT = 32
_AUDIT = ContextVar('dynamic_stop_window_audit', default=None)


def published_stop_event(node, status):
    token = getattr(node, '_r57_blocked_guard_token', None)
    if (not isinstance(token, dict) or any(key not in token for key in FIELDS)
            or token.get('request_id') != node.request_id
            or token.get('request_id') != status.get('request_id')
            or type(token.get('stop_count')) is not int
            or token['stop_count'] != node.obstacle_stops
            or type(status.get('obstacle_stops')) is not int
            or token['stop_count'] != status.get('obstacle_stops')):
        return None
    event = {key: copy.deepcopy(token[key]) for key in FIELDS}
    event['reference_stamp'] = list(event['reference_stamp'])
    return dict(event, schema=SCHEMA)


def _update(**values):
    audit = _AUDIT.get()
    if audit is not None:
        audit.update(values)


def _reference(record):
    data = record['data']
    return dict(sequence=data.get('sequence'),
                clock_ns=data.get('compute_ros_clock_ns'),
                source_offset=record.get('source_offset'),
                source_length=record.get('source_length'),
                source_line_sha256=record.get('source_line_sha256'))


def audited_dynamic(function):
    @wraps(function)
    def verify(run, request_id, status_rows, pose_rows, guard_records,
               obstacle_records, *, audit_context=None):
        audit = dict(schema='teacher_dynamic_failure_window/v1',
                     request_id=request_id,
                     submission=copy.deepcopy(audit_context or {}),
                     verify_started_wall_ns=time.monotonic_ns())
        saved = _AUDIT.set(audit)
        try:
            return function(run, request_id, status_rows, pose_rows,
                            guard_records, obstacle_records)
        except Exception as error:
            audit.update(verify_failed_wall_ns=time.monotonic_ns(),
                         error=type(error).__name__ + ': ' + str(error))
            raw = json.dumps(audit, allow_nan=False).encode()
            if len(raw) > AUDIT_LIMIT_B - 512:
                audit.pop('selected_guard_refs', None)
                audit['selected_guard_refs_omitted_for_byte_cap'] = True
            if len(json.dumps(audit, allow_nan=False).encode()) > AUDIT_LIMIT_B - 512:
                error_raw = audit['error'].encode()
                request_raw = str(request_id).encode()
                audit = dict(schema=audit['schema'],
                             request_id_prefix=request_raw[:256].decode(errors='ignore'),
                             request_id_bytes=len(request_raw),
                             request_id_sha256=hashlib.sha256(request_raw).hexdigest(),
                             error=error_raw[:512].decode(errors='ignore'),
                             error_bytes=len(error_raw),
                             error_sha256=hashlib.sha256(error_raw).hexdigest(),
                             verify_started_wall_ns=audit['verify_started_wall_ns'],
                             verify_failed_wall_ns=audit['verify_failed_wall_ns'],
                             audit_omitted_for_byte_cap=True)
            error.dynamic_window_audit = audit
            raise
        finally:
            _AUDIT.reset(saved)
    return verify


def select_event_preroll(request_id, held, guards, verify_record):
    """Missing/conflicting events or rows never fall back to nearby blockage."""
    if not held:
        raise ValueError('Actual controller did not stop for the moving obstacle')
    status = held[0]['data']
    received_edge = held[0]['received_sim_ns']
    event = status.get('dynamic_stop_event')
    _update(received_edge_ns=received_edge, stop_event=copy.deepcopy(event))
    if not isinstance(event, dict) or set(event) != set(FIELDS) | {'schema'} or event['schema'] != SCHEMA:
        raise ValueError('Actual dynamic stop event token missing or malformed')
    for key in ('waypoint_index', 'sequence', 'clock_ns', 'cloud_source_ns', 'stop_count'):
        if type(event[key]) is not int or event[key] < (1 if key in ('sequence', 'stop_count') else 0):
            raise ValueError('Actual dynamic stop event integer identity differs: ' + key)
    reference = event['reference_stamp']
    if (not isinstance(reference, (list, tuple)) or len(reference) != 2
            or any(type(value) is not int for value in reference)
            or reference[0] < 0 or not 0 <= reference[1] < 1_000_000_000
            or type(event['trajectory_id']) not in (int, str)
            or event['trajectory_id'] in (None, '')
            or not isinstance(event['cloud_sha256'], str)
            or len(event['cloud_sha256']) != 64
            or any(c not in '0123456789abcdef' for c in event['cloud_sha256'])):
        raise ValueError('Actual dynamic stop event source/reference identity malformed')
    if (not isinstance(request_id, str) or not request_id
            or event['request_id'] != request_id or type(received_edge) is not int
            or event['clock_ns'] > received_edge):
        raise ValueError('Actual dynamic stop event request or causal receive edge differs')
    for record in held:
        data = record['data']
        if (json.dumps(data.get('dynamic_stop_event'), sort_keys=True, allow_nan=False)
                != json.dumps(event, sort_keys=True, allow_nan=False)
                or type(data.get('obstacle_stops')) is not int
                or data['obstacle_stops'] != event['stop_count']
                or type(data.get('waypoint_index')) is not int
                or data['waypoint_index'] != event['waypoint_index']):
            raise ValueError('Actual held statuses mix conflicting stop events')
    matches = [r for r in guards if r['data'].get('sequence') == event['sequence']]
    if len(matches) != 1:
        raise ValueError('Actual dynamic stop event guard missing or duplicated')
    trigger = matches[0]
    data = trigger['data']
    pairs = {'request_id': 'request_id', 'waypoint_index': 'waypoint_index',
             'sequence': 'sequence', 'clock_ns': 'compute_ros_clock_ns',
             'cloud_source_ns': 'cloud_header_stamp_ns',
             'cloud_sha256': 'filtered_xyz_float64_sha256',
             'trajectory_id': 'trajectory_id'}
    for key, source in pairs.items():
        if type(data.get(source)) is not type(event[key]) or data[source] != event[key]:
            raise ValueError('Actual stop event/guard identity mismatch: ' + key)
    guard_reference = data.get('reference_stamp')
    if (not isinstance(guard_reference, (list, tuple))
            or any(type(value) is not int for value in guard_reference)
            or tuple(guard_reference) != tuple(reference)
            or data.get('union_result', {}).get('blocked') is not True
            or data.get('obstacle_hold_before') is not False
            or data.get('obstacle_hold_after') is not True
            or type(data.get('obstacle_stops_before')) is not int
            or type(data.get('obstacle_stops_after')) is not int
            or data['obstacle_stops_before'] + 1 != event['stop_count']
            or data['obstacle_stops_after'] != event['stop_count']):
        raise ValueError('Actual stop event is not the bound false-to-true counted guard')
    verify_record(trigger)
    event_edge = event['clock_ns']
    # Cover the full real 300ms interval using its causal predecessor.
    # Sampling phase must not discard the left-boundary observation.
    boundary = event_edge - 300_000_000
    predecessors = [r for r in guards if r['data']['compute_ros_clock_ns'] <= boundary]
    lower = predecessors[-1]['data']['compute_ros_clock_ns'] if predecessors else boundary
    preroll = [r for r in guards if lower <= r['data']['compute_ros_clock_ns'] <= event_edge]
    before = [r for r in guards if r['data']['compute_ros_clock_ns'] < lower]
    after = [r for r in guards if r['data']['compute_ros_clock_ns'] > event_edge]
    span = (preroll[-1]['data']['compute_ros_clock_ns'] - preroll[0]['data']['compute_ros_clock_ns']) if preroll else 0
    _update(event_edge_ns=event_edge, window_lower_ns=lower,
            window_upper_ns=event_edge, selected_count=len(preroll),
            selected_span_ns=span, event_guard_ref=_reference(trigger),
            guard_source_file=trigger.get('source_file'),
            selected_guard_refs=[_reference(r) for r in preroll[:32]],
            selected_guard_refs_truncated=len(preroll) > 32,
            outside_before=_reference(before[-1]) if before else None,
            outside_after=_reference(after[0]) if after else None)
    if len(preroll) < 3 or span < 300_000_000:
        raise ValueError('Actual dynamic hold lacks the full300ms native SCAN pre-roll')
    if preroll[-1] is not trigger:
        raise ValueError('Actual pre-roll does not end at the bound stop event')
    for previous, current in zip(preroll, preroll[1:]):
        left, right = previous['data'], current['data']
        if (type(left.get('sequence')) is not int or type(right.get('sequence')) is not int
                or right['sequence'] != left['sequence'] + 1
                or type(right.get('previous_actual_guard_clock_ns')) is not int
                or right['previous_actual_guard_clock_ns'] != left['compute_ros_clock_ns']
                or not 0 < right['compute_ros_clock_ns']-left['compute_ros_clock_ns'] <= 300_000_000):
            raise ValueError('Actual pre-roll has a missing or conflicting guard record')
    for record in preroll:
        if type(record['data'].get('compute_ros_clock_ns')) is not int:
            raise ValueError('Actual pre-roll compute clock is not an integer stamp')
        verify_record(record)
    return preroll, trigger, copy.deepcopy(event)
