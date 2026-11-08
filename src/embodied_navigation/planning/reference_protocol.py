"""Join R85 SCAN candidates with the exact sent reference and optimizer receipt.

This is a provenance contract, not motion admission or an execution ACK.  No ROS
imports are used.  A low-frequency route definition is validated once; schema3 candidates carry
only route identifiers and local fields. State advances only after all inputs
agree; rejected or incomplete candidates cannot become the accepted identity.
"""
from collections import OrderedDict
import copy
import hashlib
import json
import math
import time


SCOPE = "planner_process_lifetime_requires_outer_epoch"
BODY_HEIGHT_OFFSET = 0.4


def _integer(value, name, minimum=0):
    if type(value) is not int or value < minimum:
        raise ValueError(name + "_invalid_integer")
    return value


def _number(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(name + "_nonfinite_number")
    return float(value)


def _vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(name + "_invalid_shape")
    return [_number(x, name) for x in value]


def _stamp(value, name, positive=False):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(name + "_invalid_stamp")
    sec = _integer(value[0], name + "_sec")
    nsec = _integer(value[1], name + "_nanosec")
    if nsec >= 1_000_000_000 or (positive and sec == nsec == 0):
        raise ValueError(name + "_invalid_stamp")
    return [sec, nsec]


def _near(a, b):
    return abs(a - b) <= 1e-8


def _duration_equal(a, b):
    return abs(a - b) <= 1e-6 * max(1.0, abs(a), abs(b))


def _object(value, name):
    if not isinstance(value, dict):
        raise ValueError(name + "_must_be_object")
    return value


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _digest(value):
    return hashlib.sha256(_canonical(value).encode()).hexdigest()


def _sent_reference(value):
    """The exact Path serialization plus its single SCAN height offset."""
    value = _object(value, "sent_reference")
    if value.get("frame_id") != "camera_init":
        raise ValueError("sent_reference_frame_mismatch")
    stamp = _stamp(value.get("stamp"), "sent_reference", positive=True)
    offset = _number(value.get("body_height_offset"), "body_height_offset")
    if offset != BODY_HEIGHT_OFFSET:
        raise ValueError("body_height_offset_mismatch")
    poses = value.get("input_poses")
    if not isinstance(poses, list) or not 1 <= len(poses) <= 4096:
        raise ValueError("sent_reference_pose_count")
    normalized = []
    for pose in poses:
        pose = _object(pose, "reference_pose")
        if pose.get("frame_id") not in ("", "camera_init"):
            raise ValueError("reference_pose_frame_mismatch")
        item = dict(frame_id=pose["frame_id"],
                    stamp=_stamp(pose.get("stamp"), "reference_pose"),
                    position=_vector(pose.get("position"), 3, "reference_position"),
                    orientation_xyzw=_vector(pose.get("orientation_xyzw"), 4, "reference_orientation"))
        if normalized and math.dist(normalized[-1]["position"], item["position"]) <= 1e-6:
            raise ValueError("duplicate_adjacent_reference_pose")
        normalized.append(item)
    return dict(frame_id="camera_init", stamp=stamp, body_height_offset=offset,
                input_poses=normalized)


def _curve(value):
    value = _object(value, "global_curve")
    if value.get("coefficient_order") != "descending_power":
        raise ValueError("global_curve_coefficient_order")
    times = value.get("segment_times")
    if not isinstance(times, list) or not 1 <= len(times) <= 4097:
        raise ValueError("global_curve_segment_count")
    times = [_number(x, "segment_time") for x in times]
    if any(x <= 0.0 for x in times):
        raise ValueError("global_curve_nonpositive_segment")
    duration = math.fsum(times)
    if not math.isfinite(duration) or duration < 1e-3:
        raise ValueError("global_curve_invalid_duration")
    axes = value.get("coefficients_xyz")
    if not isinstance(axes, list) or len(axes) != 3:
        raise ValueError("global_curve_requires_xyz")
    coefficients = []
    for axis in axes:
        if not isinstance(axis, list) or len(axis) != len(times):
            raise ValueError("global_curve_segment_shape")
        normalized = []
        for segment in axis:
            if not isinstance(segment, list) or not 3 <= len(segment) <= 64:
                raise ValueError("global_curve_coefficient_shape")
            normalized.append([_number(c, "global_coefficient") for c in segment])
        coefficients.append(normalized)
    for i in range(len(times)):
        if len({len(axis[i]) for axis in coefficients}) != 1:
            raise ValueError("global_curve_xyz_shape_mismatch")
    return dict(coefficient_order="descending_power", segment_times=times,
                coefficients_xyz=coefficients), duration


def _trajectory(value):
    value = _object(value, "trajectory")
    order = _integer(value.get("order"), "spline_order")
    if order != 3:
        raise ValueError("cubic_spline_required")
    traj_id = _integer(value.get("traj_id"), "trajectory_id")
    stamp = _stamp(value.get("start_time"), "trajectory_start_time")
    points = value.get("pos_pts")
    knots = value.get("knots")
    if not isinstance(points, list) or not 4 <= len(points) <= 4096:
        raise ValueError("spline_control_point_count")
    if not isinstance(knots, list) or len(knots) != len(points) + order + 1:
        raise ValueError("spline_knot_count")
    points = [_vector(p, 3, "spline_point") for p in points]
    knots = [_number(k, "spline_knot") for k in knots]
    if any(b < a for a, b in zip(knots, knots[1:])) or knots[-order-1] <= knots[order]:
        raise ValueError("spline_knot_domain")
    yaw_points = value.get("yaw_pts")
    if not isinstance(yaw_points, list) or len(yaw_points) > 512:
        raise ValueError("spline_yaw_point_count")
    yaw_points = [_number(y, "spline_yaw_point") for y in yaw_points]
    yaw_dt = _number(value.get("yaw_dt"), "spline_yaw_dt")
    if yaw_dt < 0.0:
        raise ValueError("spline_yaw_dt_negative")
    return dict(order=order, traj_id=traj_id, start_time=stamp,
                pos_pts=points, knots=knots, yaw_pts=yaw_points, yaw_dt=yaw_dt)


def payload(msg):
    """Normalize the complete Bspline payload, never just its id or endpoint."""
    return _trajectory(dict(order=msg.order, traj_id=msg.traj_id,
                            start_time=[msg.start_time.sec, msg.start_time.nanosec],
                            pos_pts=[[p.x, p.y, p.z] for p in msg.pos_pts],
                            knots=list(msg.knots), yaw_pts=list(msg.yaw_pts), yaw_dt=msg.yaw_dt))


def key(value):
    return (value["traj_id"], *value["start_time"])


class R85TrajectoryAssociation:
    def __init__(self, run_id, capacity=32):
        if not isinstance(run_id, str) or not run_id.strip():
            raise ValueError("nonempty_run_id_required")
        if type(capacity) is not int or not 1 <= capacity <= 32:
            raise ValueError("capacity_must_be_1_to_32")
        self.run_id, self.capacity = run_id, capacity
        self.epoch = self.planner_process_id = self.localization_generation = None
        self.rejected, self.last_rejected = 0, None
        self.splines, self.metadata, self.attempts = OrderedDict(), OrderedDict(), OrderedDict()
        self.local_replan_requests = OrderedDict()
        self.reset()

    def reset(self):
        """Explicit owner reset. configure_epoch is required again before use."""
        self.epoch = self.planner_process_id = self.localization_generation = None
        self.reference_stamp = self.goal = self.reference_content_expected = None
        self.route_definition = self.route_key = self.route_definition_sha256 = None
        self.route_content_sha256 = None
        self.definition_count = self.definition_duplicates = self.invalidated_pending = 0
        self.accepted_metadata_received_wall_ns = None
        self.accepted = self.accepted_metadata = None
        self.accepted_highwater_id = -1
        self.accepted_attempt_highwater_id = 0
        self.reference_revision = None
        self.observed_reference_revision = None
        self.last_committed_revision = 0
        self.original_reference_content = None
        self.observed_reference_content = None
        self.observed_reference_duration = None
        self.progress_highwater = None
        self.local_replan_request_highwater = 0
        self.splines.clear()
        self.metadata.clear()
        self.attempts.clear()
        self.local_replan_requests.clear()

    def configure_epoch(self, epoch, planner_process_id, localization_generation=None):
        epoch = _integer(epoch, "reference_epoch")
        planner_process_id = _integer(planner_process_id, "planner_process_id", 1)
        if localization_generation is not None:
            localization_generation = _integer(localization_generation, "localization_generation")
        if (epoch, planner_process_id, localization_generation) == (
                self.epoch, self.planner_process_id, self.localization_generation):
            return False
        # A localization/clock generation change invalidates the pending request,
        # but a surviving planner process cannot restart its id/revision counters.
        same_process = planner_process_id == self.planner_process_id
        highwaters = (self.accepted_highwater_id, self.accepted_attempt_highwater_id,
                      self.last_committed_revision)
        self.reset()
        self.epoch, self.planner_process_id, self.localization_generation = (
            epoch, planner_process_id, localization_generation)
        if same_process:
            self.accepted_highwater_id, self.accepted_attempt_highwater_id, self.last_committed_revision = highwaters
        return True

    def request(self, stamp, goal, reference_content_expected=None):
        if self.epoch is None or self.planner_process_id is None:
            raise ValueError("planner_epoch_not_configured")
        stamp = _stamp(stamp, "requested_reference", positive=True)
        goal = _vector(goal, 3, "requested_body_goal")
        expected = _sent_reference(reference_content_expected)
        if expected["stamp"] != stamp:
            raise ValueError("sent_reference_stamp_mismatch")
        endpoint = expected["input_poses"][-1]["position"]
        endpoint = [endpoint[0], endpoint[1], endpoint[2] + BODY_HEIGHT_OFFSET]
        if not all(_near(x, y) for x, y in zip(goal, endpoint)):
            raise ValueError("sent_reference_body_endpoint_mismatch")
        if self.reference_stamp == stamp:
            if self.goal != goal or self.reference_content_expected != expected:
                raise ValueError("same_reference_stamp_content_changed")
            return False  # Idempotent: do not erase pending data or id/progress highwater.
        if self.reference_stamp is not None and tuple(stamp) <= tuple(self.reference_stamp):
            raise ValueError("reference_stamp_does_not_advance_epoch")
        self.reference_stamp, self.goal = stamp, goal
        self.route_definition = self.route_key = self.route_definition_sha256 = None
        self.route_content_sha256 = None
        self.accepted_metadata_received_wall_ns = None
        self.reference_content_expected = copy.deepcopy(expected)
        self.accepted = self.accepted_metadata = None
        self.reference_revision = self.original_reference_content = None
        self.observed_reference_revision = self.observed_reference_content = None
        self.observed_reference_duration = None
        self.progress_highwater = None
        self.local_replan_request_highwater = 0
        self.splines.clear()
        self.metadata.clear()
        self.attempts.clear()
        self.local_replan_requests.clear()
        # SCAN ids are process monotonic across Path replacements too.
        return True

    def _request_scope(self):
        return dict(run_id=self.run_id, reference_epoch=self.epoch,
                    planner_process_id=self.planner_process_id,
                    localization_generation=self.localization_generation,
                    reference_stamp=copy.deepcopy(self.reference_stamp))

    def note_local_replan_request(self, sequence):
        """Register only AFTER the owner successfully publishes this request.

        The owner must publish and register in the same synchronous callback,
        without dispatching incoming ROS messages between the two operations.
        This records an owner assertion of publication, never an execution ACK.
        """
        if self.reference_stamp is None or self.epoch is None or self.planner_process_id is None:
            raise ValueError("reference_not_requested")
        sequence = _integer(sequence, "local_replan_request_sequence", 1)
        scope = self._request_scope()
        if sequence in self.local_replan_requests:
            if self.local_replan_requests[sequence] != scope:
                raise ValueError("registered_local_replan_scope_mismatch")
            return False  # Exact repeated note is idempotent and does not refresh its cache age.
        if sequence <= self.local_replan_request_highwater:
            raise ValueError("local_replan_request_sequence_does_not_advance")
        self.local_replan_requests[sequence] = scope
        self.local_replan_request_highwater = sequence
        while len(self.local_replan_requests) > self.capacity:
            self.local_replan_requests.popitem(last=False)
        return True

    def reject(self, reason):
        self.rejected += 1
        self.last_rejected = str(reason)

    def _put(self, cache, identity, value, content):
        if identity in cache:
            previous = (cache[identity][1] if cache is self.splines else
                        cache[identity][0] if cache is self.metadata else cache[identity])
            if previous != content:
                raise ValueError("duplicate_identity_conflicting_content")
            return
        cache[identity] = value
        while len(cache) > self.capacity:
            cache.popitem(last=False)

    def _scope(self, value, *, definition=False):
        if self.reference_stamp is None:
            raise ValueError("reference_not_requested")
        _object(value, "protocol_message")
        if definition:
            if value.get('schema') != 'scan_route_definition/v1':
                raise ValueError('route_definition_schema_required')
        elif type(value.get('schema')) is not int or value['schema'] != 3:
            raise ValueError('schema3_required')
        if (value.get("run_id") != self.run_id
                or type(value.get("reference_epoch")) is not int
                or value["reference_epoch"] != self.epoch
                or type(value.get("planner_process_id")) is not int
                or value["planner_process_id"] != self.planner_process_id
                or value.get("reference_revision_scope") != SCOPE):
            raise ValueError("planner_scope_mismatch")
        if self.localization_generation is not None and (
                type(value.get("localization_generation")) is not int
                or value["localization_generation"] != self.localization_generation):
            raise ValueError("localization_generation_mismatch")
        if not definition and (value.get("publication_role") != "candidate_only"
                or value.get("actual_active") is not False
                or value.get("outer_admission_ack_received") is not False):
            raise ValueError("candidate_role_required")
        stamp = _stamp(value.get("reference_stamp"), "reference", positive=True)
        if stamp != self.reference_stamp:
            raise ValueError("reference_stamp_mismatch")
        revision = _integer(value.get("committed_reference_revision"), "reference_revision", 1)
        if (_integer(value.get('route_id'),'route_id',1) != stamp[0]*1_000_000_000+stamp[1]
                or _integer(value.get('route_version'),'route_version',1) != revision):
            raise ValueError('route_identity_mismatch')
        if self.route_key is not None:
            if revision < self.route_key[1]:
                raise ValueError('stale_route_version')
        elif revision <= self.last_committed_revision:
            raise ValueError('reference_revision_does_not_advance')
        return revision

    def add_definition(self, value):
        """Install one ordinary route state at its low-frequency version boundary."""
        try:
            revision = self._scope(value, definition=True)
            route_key = (value['route_id'], revision)
            if self.route_key == route_key:
                # Re-delivery cannot overwrite program-owned state. No repeated
                # whole-definition comparison/hash/normalization at this key.
                self.definition_duplicates += 1
                return self.match_pending()
            content = _object(value.get('reference_content'),'reference_content')
            sent = _sent_reference(content)
            if sent != self.reference_content_expected:
                raise ValueError('reference_content_differs_from_sent_path')
            curve, curve_duration = _curve(content.get('global_curve'))
            duration = _number(value.get('reference_original_duration'),'original_duration')
            if duration < 1e-3 or not _duration_equal(duration,curve_duration):
                raise ValueError('global_duration_changed')
            goal = _vector(value.get('body_goal'),3,'body_goal')
            if not all(_near(x,y) for x,y in zip(goal,self.goal)):
                raise ValueError('planner_body_goal_mismatch')
            definition = {k:copy.deepcopy(value[k]) for k in (
                'schema','run_id','reference_epoch','planner_process_id','localization_generation',
                'reference_stamp','committed_reference_revision','reference_revision_scope','route_id','route_version') if k in value}
            definition.update(reference_content=dict(sent,global_curve=curve),
                reference_original_duration=duration,body_goal=goal)
            # Only the boundary computes these optional evidence digests.
            definition_sha = _digest(definition)
            content_sha = _digest(definition['reference_content'])
            stale = [k for k,(m,receipt,failure) in self.metadata.items() if m['route_version'] < revision]
            for k in stale:
                self.metadata.pop(k);self.splines.pop(k,None)
            old_attempts = [k for k in self.attempts if k[0] < revision]
            for k in old_attempts:self.attempts.pop(k)
            self.invalidated_pending += len(stale)+len(old_attempts)
            self.route_definition,self.route_key = definition,route_key
            self.route_definition_sha256,self.route_content_sha256 = definition_sha,content_sha
            self.definition_count += 1
            self.accepted = self.accepted_metadata = None
            self.accepted_metadata_received_wall_ns = None
            self.reference_revision = None
            self.observed_reference_revision = self.last_committed_revision = revision
            self.original_reference_content = self.observed_reference_content = definition['reference_content']
            self.observed_reference_duration = duration
            self.progress_highwater = None
            # Validate small receipts that preceded the definition, including
            # negative attempts. No global graph is touched here again.
            for k,receipt in list(self.attempts.items()):
                if k[0] == revision:
                    try:self._validate_attempt(receipt)
                    except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as error:
                        self.reject(error);self.attempts.pop(k)
            return self.match_pending()
        except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as error:
            self.reject(error)
            return None

    def _duration(self,value,duration,name):
        duration = _number(duration,name)
        if duration < 1e-3:
            raise ValueError('global_duration_changed' if name=='original_duration' else 'attempt_global_state_mismatch')
        if self.route_key == (value['route_id'],value['route_version']):
            if duration != self.route_definition['reference_original_duration']:
                raise ValueError('committed_original_duration_changed')
        return duration

    def _progress(self, value, duration, name):
        value = _number(value, name)
        if not 0.0 <= value <= duration:
            raise ValueError(name + "_outside_duration")
        return value

    def _attempt(self, value, duration):
        attempt = _object(value.get("attempt"), "attempt")
        attempt_id = _integer(attempt.get("id"), "attempt_id", 1)
        if attempt_id <= self.accepted_attempt_highwater_id:
            raise ValueError("attempt_id_does_not_advance")
        if not isinstance(attempt.get("origin"), str) or not attempt["origin"]:
            raise ValueError("attempt_origin_required")
        if type(attempt.get("use_poly_init")) is not bool or type(attempt.get("random_poly")) is not bool:
            raise ValueError("attempt_poly_flags_required")
        sequence = _integer(attempt.get("local_replan_request_sequence"), "local_replan_request_sequence")
        if attempt["origin"].startswith("external_local_replan_"):
            if sequence == 0:
                raise ValueError("external_local_replan_sequence_required")
            if self.local_replan_requests.get(sequence) != self._request_scope():
                raise ValueError("external_local_replan_request_not_registered")
        elif sequence != 0:
            raise ValueError("unrequested_attempt_sequence_must_be_zero")
        before = self._progress(attempt.get("progress_before_target_selection"), duration, "progress_before")
        after = self._progress(attempt.get("progress_after_target_selection"), duration, "progress_after")
        if after < before:
            raise ValueError("progress_regressed_during_target_selection")
        if (self.progress_highwater is not None and self.route_key == (value['route_id'],value['route_version'])
                and before < self.progress_highwater):
            raise ValueError("progress_regressed_from_accepted_candidate")
        return attempt_id, before, after

    def _validate_metadata(self, value):
        revision = self._scope(value)
        if value.get("candidate_kind") != "motion" or value.get("reference_ready") is not True:
            raise ValueError("ready_motion_candidate_required")
        if 'reference_content' in value or 'trajectory' in value:
            raise ValueError('schema3_repeated_definition_or_trajectory')
        state = _object(value.get('global_state'),'global_state')
        if 'current_curve' in state:
            raise ValueError('schema3_repeated_current_curve')
        duration = self._duration(value,value.get('reference_original_duration'),'original_duration')
        if _number(state.get('duration'),'current_duration') != duration:
            raise ValueError('global_duration_changed')
        progress = self._progress(state.get("progress"), duration, "global_progress")
        attempt_id, before, after = self._attempt(value, duration)
        if progress < after:
            raise ValueError("progress_regressed_after_target_selection")
        target = _object(value.get("local_target"), "local_target")
        if target.get("valid") is not True or type(target.get("occupancy")) is not int or target["occupancy"] != 0:
            raise ValueError("free_valid_local_target_required")
        self._progress(target.get("time"), duration, "local_target_time")
        _vector(target.get("position"), 3, "local_target_position")
        _vector(target.get("velocity"), 3, "local_target_velocity")
        goal = _vector(value.get("body_goal"), 3, "body_goal")
        adjusted = _vector(value.get("adjusted_body_goal"), 3, "adjusted_body_goal")
        if not all(_near(x, y) and _near(x, z) for x, y, z in zip(self.goal, goal, adjusted)):
            raise ValueError("planner_body_goal_mismatch")
        start = _object(value.get("start_state"), "start_state")
        if start.get("boundary_kind") != "measured" or type(start.get("frozen_reference_boundary")) is not bool:
            raise ValueError("measured_start_state_required")
        for name in ("position", "measured_velocity", "used_velocity", "used_acceleration"):
            _vector(start.get(name), 3, "start_" + name)
        trajectory_id = _integer(value.get('trajectory_id'),'trajectory_id')
        trajectory_stamp = _stamp(value.get('trajectory_start_time'),'trajectory_start_time')
        return revision,attempt_id,duration,progress,(trajectory_id,*trajectory_stamp)

    def _validate_attempt(self, value):
        revision = self._scope(value)
        if value.get("event") != "attempt_result":
            raise ValueError("attempt_result_event_required")
        if 'reference_content' in value:
            raise ValueError('schema3_repeated_definition')
        attempt = _object(value.get('attempt'),'attempt')
        duration = self._duration(value,attempt.get('duration'),'attempt_duration')
        attempt_id, before, after = self._attempt(value, duration)
        if type(attempt.get("plan_success")) is not bool:
            raise ValueError("attempt_success_boolean_required")
        final = self._progress(attempt.get("progress_after_optimizer"), duration, "progress_after_optimizer")
        if final < after:
            raise ValueError("attempt_global_state_mismatch")
        self._progress(attempt.get("local_target_time"), duration, "attempt_local_target_time")
        _vector(attempt.get("local_target_position"), 3, "attempt_local_target_position")
        _vector(attempt.get("local_target_velocity"), 3, "attempt_local_target_velocity")
        occupancy = attempt.get("local_target_occupancy")
        if type(occupancy) is not int or occupancy not in (-1, 0, 1):
            raise ValueError("attempt_local_target_occupancy_invalid")
        if attempt["plan_success"] and occupancy != 0:
            raise ValueError("successful_attempt_local_target_not_free")
        return revision,attempt_id,duration,final

    def add_spline(self, msg):
        try:
            if self.reference_stamp is None:
                raise ValueError("reference_not_requested")
            value = payload(msg)
            identity = key(value)
            if identity[0] <= self.accepted_highwater_id:
                raise ValueError("trajectory_id_does_not_advance")
            self._put(self.splines, identity, (msg, value), value)
            return self._match(identity)
        except (ValueError, TypeError, KeyError, AttributeError, OverflowError) as error:
            self.reject(error)
            return None

    def add_metadata(self, value, *, received_wall_ns=None):
        try:
            info = self._validate_metadata(value)
            identity = info[-1]
            if identity[0] <= self.accepted_highwater_id:
                raise ValueError('trajectory_id_does_not_advance')
            # Project only known small schema3 fields. Unknown extra graphs are
            # neither retained nor copied into accepted metadata/evidence.
            value = self._small_message(value,metadata=True)
            receipt = time.monotonic_ns() if received_wall_ns is None else _integer(received_wall_ns,'metadata_received_wall_ns',1)
            self._put(self.metadata,identity,(value,receipt,None),value)
            return self._match(identity)
        except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as error:
            self.reject(error)
            return None

    @staticmethod
    def _small_message(value, *, metadata=False):
        fields = ('schema','run_id','reference_epoch','localization_generation','planner_process_id',
            'reference_revision_scope','publication_role','actual_active','outer_admission_ack_received',
            'committed_reference_revision','route_id','route_version','reference_stamp')
        result = {k:copy.deepcopy(value[k]) for k in fields}
        attempt_fields = ('id','origin','use_poly_init','random_poly','local_replan_request_sequence',
            'progress_before_target_selection','progress_after_target_selection')
        if metadata:
            result.update({k:copy.deepcopy(value[k]) for k in ('candidate_kind','reference_ready',
                'reference_original_duration','body_goal','adjusted_body_goal','trajectory_id','trajectory_start_time')})
            result['global_state'] = {k:value['global_state'][k] for k in ('duration','progress')}
            result['local_target'] = {k:copy.deepcopy(value['local_target'][k]) for k in ('valid','occupancy','time','position','velocity')}
            result['start_state'] = {k:copy.deepcopy(value['start_state'][k]) for k in ('boundary_kind','frozen_reference_boundary','position','measured_velocity','used_velocity','used_acceleration')}
        else:
            result['event'] = value['event']
            attempt_fields += ('plan_success','result_reason','local_target_occupancy','progress_after_optimizer',
                'duration','local_target_time','local_target_position','local_target_velocity')
        result['attempt'] = {k:copy.deepcopy(value['attempt'][k]) for k in attempt_fields if k in value['attempt']}
        return result

    def add_attempt(self, value):
        try:
            revision,attempt_id,duration,_ = self._validate_attempt(value)
            value = self._small_message(value)
            identity = (revision,attempt_id)
            self._put(self.attempts,identity,value,value)
            candidates = [k for k,(meta,receipt,failure) in self.metadata.items()
                          if (meta['committed_reference_revision'],meta['attempt']['id']) == identity]
            if len(candidates) > 1:
                raise ValueError('attempt_claimed_by_multiple_trajectories')
            return self._match(candidates[0]) if candidates else None
        except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as error:
            self.reject(error)
            return None

    def match_pending(self):
        """One original pending candidate per owner call, never an eager batch."""
        if self.route_key is None:return None
        for identity,(meta,receipt,failure) in self.metadata.items():
            if (failure is None and (meta['route_id'],meta['route_version']) == self.route_key
                    and identity in self.splines
                    and (meta['route_version'],meta['attempt']['id']) in self.attempts):
                try:return self._match(identity)
                except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as error:
                    self.reject(error)
                    # The failed original entry stays bounded and cannot be
                    # rewritten by a duplicate or retried every idle poll.
                    return None
        return None

    def _match(self, identity):
        try:return self._match_candidate(identity)
        except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as error:
            if identity in self.metadata:
                meta,received,failure=self.metadata[identity]
                if failure is None:
                    self.metadata[identity]=(meta,received,str(error)[:160])
                    self.invalidated_pending += 1
            raise

    def _match_candidate(self, identity):
        if identity not in self.splines or identity not in self.metadata:
            return None
        msg, spline = self.splines[identity]
        meta,received_wall_ns,failure = self.metadata[identity]
        if failure is not None:return None
        if self.route_key != (meta['route_id'],meta['route_version']):
            return None
        revision,attempt_id,duration,progress,trajectory_identity = self._validate_metadata(meta)
        attempt_key = (revision, attempt_id)
        receipt = self.attempts.get(attempt_key)
        if receipt is None:
            return None
        claims = [k for k, (item,receipt_wall,failure) in self.metadata.items()
                  if (item["committed_reference_revision"], item["attempt"]["id"]) == attempt_key]
        if len(claims) != 1:
            raise ValueError("attempt_claimed_by_multiple_trajectories")
        _,_,attempt_duration,final = self._validate_attempt(receipt)
        if receipt["attempt"]["plan_success"] is not True:
            raise ValueError("attempt_optimizer_failed")
        if receipt["attempt"]["local_target_occupancy"] != 0:
            raise ValueError("successful_attempt_local_target_not_free")
        if payload(msg) != spline:
            raise ValueError("cached_spline_message_changed")
        if key(spline) != trajectory_identity:
            raise ValueError("metadata_and_spline_identity_differ")
        if duration != attempt_duration:
            raise ValueError("metadata_and_attempt_duration_differ")
        ma, ra, target = meta["attempt"], receipt["attempt"], meta["local_target"]
        for name in ("id", "origin", "use_poly_init", "random_poly",
                     "progress_before_target_selection", "progress_after_target_selection",
                     "local_replan_request_sequence"):
            if ma[name] != ra[name]:
                raise ValueError("metadata_and_attempt_fields_differ")
        if (progress != final or target["time"] != ra["local_target_time"]
                or target["position"] != ra["local_target_position"]
                or target["velocity"] != ra["local_target_velocity"]
                or target["occupancy"] != ra["local_target_occupancy"]):
            raise ValueError("metadata_and_attempt_target_or_progress_differ")
        if identity[0] <= self.accepted_highwater_id:
            raise ValueError("trajectory_id_does_not_advance")
        self.reference_revision = revision
        self.observed_reference_revision = revision
        self.last_committed_revision = revision
        self.observed_reference_duration = duration
        self.progress_highwater = progress
        self.accepted = copy.deepcopy(spline)
        self.accepted_metadata = copy.deepcopy(meta)
        self.accepted_metadata_received_wall_ns = received_wall_ns
        self.accepted_highwater_id = identity[0]
        self.accepted_attempt_highwater_id = attempt_id
        self.last_rejected = None
        self.splines.pop(identity)
        self.metadata.pop(identity)
        self.attempts.pop(attempt_key)
        # Expire older incomplete trajectories; this does not reset highwater.
        for cache in (self.splines, self.metadata):
            for pending in list(cache):
                if pending[0] <= self.accepted_highwater_id:
                    cache.pop(pending)
        return msg, copy.deepcopy(meta)

    def snapshot(self):
        return dict(schema='r85_schema3_association_status/v1',run_id=self.run_id,
            reference_epoch=self.epoch,planner_process_id=self.planner_process_id,
            localization_generation=self.localization_generation,reference_stamp=copy.deepcopy(self.reference_stamp),
            body_goal=copy.deepcopy(self.goal),route_id=None if self.route_key is None else self.route_key[0],
            route_version=None if self.route_key is None else self.route_key[1],
            route_definition_sha256=self.route_definition_sha256,
            committed_reference_content_sha256=self.route_content_sha256,
            observed_reference_content_sha256=self.route_content_sha256,
            committed_reference_revision=self.reference_revision,observed_reference_revision=self.observed_reference_revision,
            observed_reference_duration=self.observed_reference_duration,accepted_highwater_id=self.accepted_highwater_id,
            accepted_attempt_highwater_id=self.accepted_attempt_highwater_id,
            accepted_trajectory_id=None if self.accepted is None else self.accepted['traj_id'],
            accepted_trajectory_start_time=None if self.accepted is None else list(self.accepted['start_time']),
            accepted_metadata_received_wall_ns=self.accepted_metadata_received_wall_ns,
            progress_highwater=self.progress_highwater,rejected=self.rejected,last_rejected=self.last_rejected,
            definition_count=self.definition_count,definition_duplicates=self.definition_duplicates,
            invalidated_pending=self.invalidated_pending,
            registered_local_replan_request_highwater=self.local_replan_request_highwater,
            registered_local_replan_requests=list(self.local_replan_requests),
            pending=dict(splines=len(self.splines),metadata=len(self.metadata),attempts=len(self.attempts)),
            actual_active=False,motion_admission_verified=False)
