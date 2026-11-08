"""Wait for actual source causality before the original admission transaction.

One matched payload slot owns no progress, command or execution authorization.
TrajectoryAssociation continues to own its monotonic payload high-water.
"""
import copy
import hashlib
import json
import time
from embodied_navigation.planning.admission.path_admission import candidate


def context(node):
    association = node.trajectory_association
    stamp = tuple(association.reference_stamp or ())
    if len(stamp) != 2 or any(type(x) is not int for x in stamp):
        raise ValueError('pending_reference_integer_stamp_required')
    if stamp[0] < 0 or not 0 <= stamp[1] < 1_000_000_000:
        raise ValueError('pending_reference_stamp_invalid')
    inputs = node.known_scene_inputs
    route = node.continuous_route
    return dict(identity=copy.deepcopy(node.admission_identity()),
        generation=inputs['generation'],
        requested_generation=node.known_scene_reference_generation,
        reference_stamp=stamp, goal=tuple(association.goal),
        coverage=None if route is None else route.next_index,
        requested_coverage=node.scan_reference_coverage_index)


def record(node, event, item, **details):
    node.evidence.append(node.run/'navigation_pending_path_commits.jsonl', dict(
        schema='causal_pending_path_commit/v1', event=event,
        reference_stamp_ns=item['stamp_ns'], trajectory_id=int(item['msg'].traj_id),
        payload_sha256=item['payload_sha256'],
        original_received_wall_ns=item['received_wall_ns'],
        source_pose_stamp_ns=int(node.pose_stamp), source_cloud_stamp_ns=int(node.cloud_stamp),
        execution_admission_authorized=False, actual_source_times_refreshed=False,
        navigation_ground_truth_used=False, **details))


def waiting(node):
    item = getattr(node, 'pending_path_commit', None)
    if item is None or node.state != 'running':
        return False
    try:
        return item['context'] == context(node)
    except (AttributeError, KeyError, TypeError, ValueError):
        return False


def defer_future(node, msg, metadata, *, started=None, received_wall_ns=None):
    # The caller first executes all original structural/geometry/direction checks.
    # The source wrapper already requires an atomic current pose/cloud bundle.
    try:
        if len(msg.pos_pts) > 512 or len(msg.knots) > 516:
            return False
        current = context(node)
        goal = (node.waypoints[node.waypoint_index] if node.continuous_route is None
                else node.trajectory_association.goal)
        value = candidate(msg, metadata, current['reference_stamp'], goal)
        encoded = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
        stamp = current['reference_stamp'][0]*1_000_000_000 + current['reference_stamp'][1]
        if stamp <= node.pose_stamp:
            node.pending_path_commit = None
            return False
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
        return False
    item = dict(context=current, stamp_ns=stamp, msg=copy.deepcopy(msg),
        metadata=copy.deepcopy(metadata), received_wall_ns=time.monotonic_ns() if received_wall_ns is None else received_wall_ns,
        payload_sha256=hashlib.sha256(encoded).hexdigest())
    previous = getattr(node, 'pending_path_commit', None)
    node.pending_path_commit = item
    # Waiting grants only the existing independent old-path retention proof.
    try:
        points, config, _ = node._route_fence_load()
        retain = node.retain_admitted_path(points, config)
    except (OSError, AttributeError, KeyError, TypeError, ValueError, IndexError, OverflowError):
        retain = dict(allowed=False, reason='pending_old_path_retention_unavailable')
    if started is not None:
        elapsed = time.monotonic()-started
        if elapsed > .100:
            from embodied_navigation.common.observation import api as _observer_api; diagnostic_enabled = _observer_api('diagnostic_observation').enabled; diagnostic_record = _observer_api('diagnostic_observation').record
            profile = getattr(node, 'profile', {})
            if diagnostic_enabled(profile):
                diagnostic_record(node.run, profile, 'admission_callback_elapsed',
                    'retention_callback_budget_exhausted', dict(
                        callback_elapsed_wall_ms=elapsed*1000, original_limit_wall_ms=100.,
                        original_timing_check_passed=False, diagnostic_execution_continued=True,
                        source_freshness_checks_unchanged=True, navigation_PASS=False))
            else:
                retain = dict(retain, allowed=False, reason='retention_callback_budget_exhausted')
    node.path_admission_hold = node.path_admission_hold or retain.get('allowed') is not True
    if node.path_admission_hold:
        node.publish_command()
    record(node, 'matched_future_wait', item,
        replaced_pending_payload=previous is not None, old_path_retention=retain,
        exact_zero_requested=node.path_admission_hold)
    return True


def commit_ready(node):
    item = getattr(node, 'pending_path_commit', None)
    if item is None:
        return False
    if not waiting(node):
        node.pending_path_commit = None
        record(node, 'discard_changed_context', item)
        return False
    # Never substitute ROS clock, arrival time, nearest pose or a cached clone.
    if not node._known_generation_ready() or node.pose_stamp < item['stamp_ns']:
        return False
    node.pending_path_commit = None
    record(node, 'actual_source_ready_recheck_original_admission', item)
    node._admit_matched_spline(item['msg'], item['metadata'],
        received_wall_ns=item['received_wall_ns'])
    return True
