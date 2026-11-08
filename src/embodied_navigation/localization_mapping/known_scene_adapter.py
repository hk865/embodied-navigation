"""Exact-source joining and a bounded background worker for scene matching.

Importing this module creates no ROS node, thread, writer, or simulator action.
The integration owner supplies the runtime witness and starts/polls the worker.
No transform is automatically published or adopted by navigation.
"""
from __future__ import annotations
from embodied_navigation.common.integration_limits import SOURCE_JOIN_SLOT_MAX_AGE_NS, LIDAR_BODY_TRANSLATION_M

from collections import OrderedDict
from pathlib import Path
import hashlib
import json
import threading
import time
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from embodied_navigation.localization_mapping.known_scene_matcher import checked_transform
from embodied_navigation.localization_mapping.point_provenance import exact_indices, commit_messages


def stamp_ns(msg):
    if isinstance(msg,dict):return msg['stamp_ns']
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def pointcloud_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    endian = '>' if msg.is_bigendian else '<'
    xyz = np.empty((msg.height * msg.width, 3), dtype=float)
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height * msg.row_step:
        raise ValueError('truncated pointcloud')
    for index, name in enumerate(('x', 'y', 'z')):
        field = fields[name]
        if field.datatype != 7 or field.count != 1 or field.offset + 4 > msg.point_step:
            raise ValueError('XYZ must be scalar float32')
        xyz[:, index] = np.ndarray((msg.height, msg.width), dtype=endian + 'f4',
                                  buffer=msg.data, offset=field.offset,
                                  strides=(msg.row_step, msg.point_step)).reshape(-1)
    return xyz


def point_records(msg):
    return np.ndarray((msg.height, msg.width), dtype='V' + str(msg.point_step),
                      buffer=msg.data,
                      strides=(msg.row_step, msg.point_step)).reshape(-1)


def odometry_transform(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    xyzw = np.array([q.x, q.y, q.z, q.w])
    if not np.isfinite(xyzw).all() or abs(np.dot(xyzw, xyzw) - 1) > 1e-3:
        raise ValueError('invalid odometry quaternion')
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(xyzw).as_matrix()
    t[:3, 3] = [p.x, p.y, p.z]
    return checked_transform(t)


def validate_runtime_witness(witness,run_id):
    """Check actual loaded roles and process observations, without source pins."""
    reasons=[]
    if witness.get('schema')!='go2_known_scene_runtime_witness/v1'or witness.get('run_id')!=run_id:reasons.append('runtime_witness_identity')
    if witness.get('producer_model')!='gz_gpu_lidar_instantaneous_header_snapshot':reasons.append('producer_time_model')
    for key in ('actual_loaded_libraries_verified','header_snapshot_time_verified','post_lio_single_owner_publication_verified','no_robot_truth_pose'):
        if witness.get(key)is not True:reasons.append(key)
    producer=witness.get('producer_bindings',[])
    if not {'gz_sensors_gpu_lidar','gz_rendering','gz_sim_sensors','ros_gz_bridge'}.issubset({b.get('role')for b in producer}):reasons.append('missing_loaded_producer_roles')
    for b in producer:
        try:
            if type(b.get('observed_pid'))is not int or b['observed_pid']<=0 or b.get('load_observation')not in('proc_maps','loaded_executable'):raise ValueError('missing actual loading observation')
            observation=Path(b['load_observation_path']).read_bytes()
            if len(observation)>2*1024*1024 or str(Path(b['path'])).encode()not in observation:raise ValueError('library absent from actual process observation')
        except(OSError,KeyError,TypeError,ValueError)as error:reasons.append('producer:'+str(b.get('role'))+':'+str(error))
    processes=witness.get('identity_producer_process_bindings',{})
    if witness.get('explicit_point_commit_required')is not True or witness.get('identity_fingerprint_model')!='go2_ros_semantic/v1':reasons.append('explicit_point_commit_witness')
    for role in ('mapper','source','adapter'):
        binding=processes.get(role,{})
        if type(binding.get('pid'))is not int or type(binding.get('start_ticks'))is not int:reasons.append('identity_producer_'+role)
    for role in ('source','adapter'):
        try:
            binding=processes[role];raw=Path(binding['observation']).read_bytes()
            if len(raw)>65536 or binding['source_path'].encode()not in raw:raise ValueError('actual producer command differs')
        except(OSError,ValueError,KeyError):reasons.append('identity_producer_observation_'+role)
    try:
        loaded=json.loads(Path(witness['loaded_SLAM_receipt_path']).read_text())
        if loaded.get('verified')is not True or loaded['mapping_identity']!=processes['mapper']:raise ValueError('mapper actual loaded process')
    except(OSError,ValueError,KeyError):reasons.append('identity_mapper_loaded_witness')
    return dict(valid=not reasons,reasons=reasons,run_id=run_id,mapper_producer_pid=processes.get('mapper',{}).get('pid'),source_producer_pid=processes.get('source',{}).get('pid'),adapter_producer_pid=processes.get('adapter',{}).get('pid'),witness_sha256=hashlib.sha256(json.dumps(witness,sort_keys=True,separators=(',',':')).encode()).hexdigest())


class SourceGeometryMismatch(ValueError):
    """Preserve the original veto while identifying the predicate it failed."""
    def __init__(self, detail, evidence=None):
        super().__init__('registered/source stage geometry inconsistent')
        self.detail = dict(detail)
        self.evidence = evidence  # diagnostic arrays, never a replacement veto


class ExactSourceJoin:
    ROLES = ('raw', 'adapted', 'registered', 'body', 'mapper')
    META_ROLES = ('source_identity','point_identity','odom_identity')
    AUX_ROLES = ('lidar',)

    def __init__(self, witness_status, maximum_stamps=12, maximum_source_age_ns=SOURCE_JOIN_SLOT_MAX_AGE_NS,
                 interval_ns=1_000_000_000, mount=LIDAR_BODY_TRANSLATION_M):
        self.witness = dict(witness_status)
        self.maximum_stamps = maximum_stamps
        self.maximum_age = maximum_source_age_ns
        self.interval_ns = interval_ns
        self.mount = np.array(mount, dtype=float)
        self.slots = OrderedDict()
        self.last_completed_ns = None
        self.latest_source_ns = None
        self.last_by_role = {}
        self.lock = threading.Lock()
        self.rejections = []

    def put(self, role, msg, received_wall_ns=None):
        if role not in (*self.ROLES,*self.META_ROLES,*self.AUX_ROLES):
            raise ValueError('unknown join role')
        ns = stamp_ns(msg)
        wall = time.monotonic_ns() if received_wall_ns is None else received_wall_ns
        with self.lock:
            last = self.last_by_role.get(role)
            if last is not None and ns <= last:
                self.rejections.append(dict(source_ns=ns, reason='duplicate_or_backward_' + role))
                self.rejections = self.rejections[-32:]
                return False
            self.last_by_role[role] = ns
            self.latest_source_ns = max(ns, self.latest_source_ns or ns)
            slot = self.slots.setdefault(ns, {})
            slot[role] = (msg, wall)
            for key in list(self.slots):
                if len(self.slots) > self.maximum_stamps or self.latest_source_ns - key > self.maximum_age:
                    del self.slots[key]
        return True

    def pop_latest(self, clock_ns, wall_ns=None):
        with self.lock:
            wall_ns = time.monotonic_ns() if wall_ns is None else wall_ns
            complete = [ns for ns, slot in self.slots.items()
                        if all(role in slot for role in (*self.ROLES,*self.META_ROLES,*self.AUX_ROLES))]
            if not complete:
                return None
            # The queued request owns its captured ROS clock. Newer messages
            # may arrive before this worker runs; retain them for a later
            # causal request instead of consuming/rejecting them as future.
            causal = [ns for ns in complete if ns <= clock_ns]
            if not causal:
                return None
            ns = max(causal)
            if self.last_completed_ns is not None and ns - self.last_completed_ns < self.interval_ns:
                return None
            slot = self.slots[ns]
            selection = dict(schema='exact_join_causal_selection/v1', captured_clock_ns=clock_ns,
                guard_wall_ns=wall_ns, selected_source_ns=ns, maximum_complete_source_ns=max(complete),
                excluded_future_complete_count=len(complete)-len(causal), signed_source_age_ns=clock_ns-ns,
                maximum_source_age_ns=self.maximum_age)
            for key in list(self.slots):
                if key <= ns:
                    del self.slots[key]
            self.last_completed_ns = ns
        if not 0 <= clock_ns - ns <= self.maximum_age:
            return dict(source_ns=ns, rejected='source_age', join_rejection_detail=dict(
                guard_clock_ns=clock_ns, guard_wall_ns=wall_ns,
                signed_source_age_ns=clock_ns-ns, maximum_source_age_ns=self.maximum_age,
                original_received_wall_ns={role:pair[1] for role,pair in slot.items()}))
        if any(not 0 <= wall_ns - pair[1] <= self.maximum_age for pair in slot.values()):
            return dict(source_ns=ns, rejected='wall_age')
        try:
            checked = self._check(ns, slot)
            checked['join_selection'] = selection
            return checked
        except (ValueError, KeyError, TypeError, AssertionError) as error:
            # Keep the legacy reason stable; a stage-labelled error alone does
            # not distinguish nearest-neighbour collisions from residuals.
            name = 'ValueError' if isinstance(error, SourceGeometryMismatch) else type(error).__name__
            result = dict(source_ns=ns, rejected=name + ':' + str(error))
            if isinstance(error, SourceGeometryMismatch):
                result['join_rejection_detail'] = dict(error.detail,
                    role_source_ns={role:stamp_ns(pair[0]) for role,pair in slot.items()},
                    original_received_wall_ns={role:pair[1] for role,pair in slot.items()},
                    guard_clock_ns=clock_ns, guard_wall_ns=wall_ns,
                    explicit_message_commit_token=False)
            if isinstance(error, SourceGeometryMismatch) and error.evidence is not None:
                result['_diagnostic_failed_source'] = dict(source_ns=ns,
                    source_messages={role:slot[role][0] for role in self.ROLES},
                    source_identity_metadata={role:slot[role][0] for role in self.META_ROLES},
                    original_received_wall_ns={role:pair[1] for role,pair in slot.items()},
                    T_odom_body=error.evidence['T_odom_body'],
                    _diagnostic_evidence=error.evidence)
            return result

    def _check(self, ns, slot):
        check_started_wall_ns = time.monotonic_ns()
        raw, adapted, registered, body, mapper = (slot[role][0] for role in self.ROLES)
        point_commit=slot["point_identity"][0];adapter_commit=slot["odom_identity"][0]
        from embodied_navigation.localization_mapping.point_provenance import commit_messages
        commit_messages(registered,mapper,body,slot['lidar'][0],point_commit,adapter_commit,
            self.witness.get('run_id'),self.witness.get('mapper_producer_pid'))
        if (self.witness.get('adapter_producer_pid')is not None
                and adapter_commit.get('adapter_pid')!=self.witness['adapter_producer_pid']):
            raise ValueError('matching adapter owner differs')
        if point_commit.get('schema')!='post_lio_source_point_commit/v1' or point_commit.get('stage')!='post_lio_dense' or point_commit.get('run_id')!=self.witness.get('run_id') or point_commit.get('navigation_ground_truth_used')is not False:
            raise ValueError('source point commit contract differs')
        from embodied_navigation.localization_mapping.point_provenance import message_sha
        if point_commit.get('registered_message_sha256')!=message_sha(registered):raise ValueError('registered complete stage message differs')
        if (point_commit.get('fingerprint_model')!='go2_ros_semantic/v1'
            or adapter_commit.get('schema')!='mapper_body_lidar_commit/v1'
            or adapter_commit.get('fingerprint_model')!='go2_ros_semantic/v1'
            or adapter_commit.get('run_id')!=self.witness.get('run_id')
            or adapter_commit.get('stamp_ns')!=ns or point_commit.get('stamp_ns')!=ns
            or adapter_commit.get('navigation_ground_truth_used')is not False):
            raise ValueError('adapter commit schema/run/stamp/authority differs')
        source_proof=slot["source_identity"][0]
        if raw.header.frame_id != 'velodyne' or adapted.header.frame_id != 'velodyne':
            raise ValueError('unexpected source lidar frame')
        if body.header.frame_id != mapper.header.frame_id or registered.header.frame_id != body.header.frame_id:
            raise ValueError('original cloud/pose frame differs')
        if body.child_frame_id != 'demo_slam_body':
            raise ValueError('wrong body frame')
        t = odometry_transform(body)
        m = odometry_transform(mapper)
        if np.max(np.abs(m - t)) > 1e-10:
            raise ValueError('mapper/body stage pose differs')
        raw_fields = [(f.name, f.offset, f.datatype, f.count) for f in raw.fields]
        filtered_fields = [(f.name, f.offset, f.datatype, f.count) for f in adapted.fields]
        if raw_fields != filtered_fields or raw.point_step != adapted.point_step:
            raise ValueError('adapter changed source layout')
        if any(name.lower() in ('time', 't', 'timestamp', 'offset_time') for name, *_ in raw_fields):
            raise ValueError('per_point_time_requires_separate_verified_model')
        raw_records, adapted_records = point_records(raw), point_records(adapted)
        # Exact input IDs below prove intact source records without sorting records.
        adapted_xyz = pointcloud_xyz(adapted)
        eligible = np.isfinite(adapted_xyz).all(1) & (np.linalg.norm(adapted_xyz, axis=1) >= .3)
        local = adapted_xyz[eligible] + self.mount
        full = pointcloud_xyz(registered)
        if len(local) != len(full) or not np.isfinite(full).all():
            raise ValueError('Generic/full registered point cardinality differs')
        reconstructed = (full - t[:3, 3]) @ t[:3, :3]
        indices,raw_ordinals=exact_indices(raw,adapted,source_proof,point_commit,registered,eligible,
            self.witness.get('run_id'),self.witness.get('source_producer_pid'))
        # Mapper/body identity is exact; raw lidar pose has the frozen mechanical offset.
        if point_commit['mapper_message_sha256']!=__import__('point_provenance').message_sha(mapper) or adapter_commit['body_message_sha256']!=__import__('point_provenance').message_sha(body):
            raise ValueError('exact source mapper/body commit message differs')
        if (point_commit['producer_epoch'],point_commit['commit'])!=(adapter_commit['producer_epoch'],adapter_commit['commit']):
            raise ValueError('exact source adapter stage commit differs')
        if point_commit.get('producer_pid')!=self.witness.get('mapper_producer_pid',point_commit.get('producer_pid')):
            raise ValueError('exact source mapper owner differs')
        expected_local=adapted_xyz[indices]+self.mount
        distance=np.linalg.norm(reconstructed-expected_local,axis=1);index=indices
        # Original k1 uniqueness is diagnostic only; true ID coverage owns identity.
        nn_distance,nn_index=cKDTree(local).query(reconstructed,workers=1)
        unique_count = len(np.unique(indices))
        # Actual frozen Generic deskew is retained. These bounds allow its
        # measured millimetre same-time discrepancy, not a different pose epoch.
        unique_count = len(np.unique(indices))
        rms_m = float(np.sqrt(np.mean(distance ** 2)))
        maximum_m = float(np.max(distance))
        evidence = None
        if getattr(self, 'diagnostic_pin_enabled', False):
            evidence = dict(index=index, distance=distance, eligible=eligible,
                local=local, full=full, reconstructed=reconstructed, T_odom_body=t,
                check_started_wall_ns=check_started_wall_ns,
                nearest_completed_wall_ns=time.monotonic_ns())
        if (unique_count != len(index) or rms_m > .006 or maximum_m > .030):
            raise SourceGeometryMismatch(dict(
                schema='exact_source_join_original_geometry_veto/v1',
                source_points=len(local), registered_points=len(full),
                unique_nearest_source_indices=unique_count,
                nearest_source_collision_count=len(index)-unique_count,
                nearest_index_bijection_failed=unique_count != len(index),
                registered_source_rms_m=rms_m if np.isfinite(rms_m) else None,
                rms_nonfinite=not bool(np.isfinite(rms_m)), rms_limit_m=.006,
                rms_failed=rms_m > .006,
                registered_source_max_m=maximum_m if np.isfinite(maximum_m) else None,
                maximum_nonfinite=not bool(np.isfinite(maximum_m)), maximum_limit_m=.030,
                maximum_failed=maximum_m > .030,
                original_geometry_thresholds_changed=False,identity_predicate_revision='actual_source_ID_coverage/v1'), evidence=evidence)
        verified = self.witness.get('valid') is True
        provenance = dict(point_time_verified=verified,
                          post_lio_association_verified=verified,
                          no_robot_truth_pose=True,
                          stage='frozen_post_LIO_single_owner_publication',
                          explicit_message_commit_token=True,
                          producer_epoch=point_commit["producer_epoch"],stage_commit=point_commit["commit"],
                          point_identity_coverage_verified=True,
                          legacy_nearest_collisions=len(nn_index)-len(np.unique(nn_index)),
                          integer_stamp_join=True,
                          witness_sha256=self.witness.get('witness_sha256'),
                          registered_source_rms_m=float(np.sqrt(np.mean(distance ** 2))),
                          registered_source_max_m=float(np.max(distance)),
                          source_membership_verified=True)
        checked_job = dict(source_ns=ns, body_points=expected_local, T_odom_body=t,
                    registered_odom_xyz=full, provenance=provenance,
                    witness_status=self.witness,
                    original_received_wall_ns={role: slot[role][1] for role in self.ROLES},
                    source_messages={role:slot[role][0] for role in self.ROLES})
        if evidence is not None:
            checked_job['_diagnostic_evidence'] = evidence
        return checked_job


class MatcherWorker:
    """At most one in-flight fit and one replaceable pending job. No cloud archive.

    Node/control callbacks only enqueue/poll. Set BLAS/OMP threads to 1 before
    importing numpy in the integration launch. Do not call submit/match while
    holding the controller or command publication lock.
    """
    def __init__(self, matcher, diagnostic_capture=None):
        self.matcher = matcher
        self.diagnostic_capture=diagnostic_capture
        self.query_capture=None
        self.condition = threading.Condition()
        self.job = None
        self.result = None
        self.closed = False
        self.in_flight = False
        self.thread = threading.Thread(target=self._run, name='known-scene-match', daemon=True)
        self.thread.start()

    def submit(self, job, current_c):
        if job is None or job.get('rejected'):
            return False
        with self.condition:
            if self.closed:
                return False
            self.job = (job, checked_transform(current_c))
            self.condition.notify()
        return True

    def submit_join(self, join, clock_ns, current_c, parent_generation=None):
        """Defer even byte/layout/geometry verification to this worker."""
        with self.condition:
            if self.closed:
                return False
            self.job = (dict(join_request=join, clock_ns=clock_ns,
                             parent_generation=parent_generation,
                             submitted_monotonic_ns=time.monotonic_ns()), checked_transform(current_c))
            self.condition.notify()
        return True

    def poll(self):
        with self.condition:
            value, self.result = self.result, None
        return value

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.job is not None or self.closed)
                if self.closed:
                    return
                (job, c), self.job = self.job, None
                self.in_flight = True
            started = time.monotonic_ns()
            parent_generation = job.get('parent_generation')
            called_matcher = False
            try:
                if 'join_request' in job:
                    join = job['join_request']
                    join.diagnostic_pin_enabled = bool(self.diagnostic_capture is not None and (getattr(self.diagnostic_capture, 'capture_needed', False) or getattr(self.diagnostic_capture, 'first_collision_evidence_needed', False)))
                    request = job
                    job = join.pop_latest(request['clock_ns'])
                    if job is not None and 'join_selection' in job:
                        job['join_selection']['submitted_monotonic_ns'] = request['submitted_monotonic_ns']
                    if job is not None and job.get('rejected') == 'source_age':
                        job['join_rejection_detail']['submitted_monotonic_ns'] = request['submitted_monotonic_ns']
                if job is None:
                    with self.condition:
                        self.in_flight = False
                    continue
                if job.get('rejected'):
                    result = dict(source_ns=job['source_ns'], accepted=False, control_allowed=False,
                                  reason='exact_join:' + job['rejected'])
                    if 'join_rejection_detail' in job:
                        result['join_rejection_detail'] = job['join_rejection_detail']
                    if self.diagnostic_capture is not None:
                        try:self.diagnostic_capture.consider_rejection(job,result,c,parent_generation)
                        except Exception as error:
                            try:self.diagnostic_capture.reject(job['source_ns'],'P1_rejection_capture:'+str(error))
                            except Exception:pass
                else:
                    if self.diagnostic_capture is not None:
                        try:self.diagnostic_capture.observe_job(job,c,parent_generation)
                        except Exception as error:
                            try:self.diagnostic_capture.reject(job['source_ns'],'capture_pin_exception:'+str(error))
                            except Exception:pass
                    called_matcher = True
                    result = self.matcher.match(job['source_ns'], job['body_points'], job['T_odom_body'],
                                                current_c=c, provenance=job['provenance'])
                    result['T_odom_body']=checked_transform(job['T_odom_body']).tolist()
                    result['T_odom_body_source_ns']=job['source_ns']
                    if self.diagnostic_capture is not None:
                        try:self.diagnostic_capture.consider(job,result,c,parent_generation)
                        except Exception as error:
                            try:self.diagnostic_capture.reject(job['source_ns'],'capture_exception:'+str(error))
                            except Exception:pass
            except Exception as error:
                result = dict(source_ns=None if job is None else job.get('source_ns'),
                              accepted=False, control_allowed=False,
                              reason='matcher_exception:' + type(error).__name__ + ':' + str(error))
            if self.query_capture is not None and called_matcher:
                try:self.query_capture.finish(job or {}, result, parent_generation)
                except Exception:pass  # No observer exception replaces original result.
            result['worker_started_wall_ns'] = started
            result['worker_completed_wall_ns'] = time.monotonic_ns()
            if job is not None and 'join_selection' in job:
                result['join_selection'] = job['join_selection']
            result['parent_generation'] = parent_generation
            with self.condition:
                self.result = result
                self.in_flight = False

    def close(self, timeout_s=2.5):
        with self.condition:
            self.closed = True
            self.job = None
            self.condition.notify_all()
        self.thread.join(timeout=timeout_s)
        return not self.thread.is_alive()
