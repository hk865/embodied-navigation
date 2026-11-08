"""R85 one-shot PCT reference and scoped local-replan wire; no motion authority.

Original46 scores never choose this guide. The existing controller still owns
measured start/heading/collision/freshness/admission and physical arrival.
"""
import copy
import hashlib
import importlib
import json
import math
from pathlib import Path
from embodied_navigation.planning.admission.replan_ack import enabled as _early_request_ack_enabled

EARLY_REQUEST_ACK_ENABLED = _early_request_ack_enabled()

PROFILE_SCHEMA = 'r85_persistent_auto_reference/v1'
REFERENCE_SCHEMA = 'r85_pct_scan_global_reference/v1'
REGISTRATION_SCHEMA = 'r85_reference_source_registration/v1'
CONNECTOR_SCHEMA = 'r85_actual_start_connector/v1'
MAX_REFERENCE_POINTS = 4096


def reference_version_paths(run, epoch, generation):
    """Ordinary program version names, written once by the reference owner."""
    if any(type(x)is not int or x<1 for x in (epoch,generation)):
        raise ValueError('r85_invalid_reference_version_scope')
    run=Path(run).resolve();suffix='_epoch_'+str(epoch)+'_generation_'+str(generation)+'.json'
    return run/('r85_global_reference'+suffix),run/('r85_reference_registration'+suffix)


def enabled(profile):
    from embodied_navigation.control.parking.terminal_heading_contract import enabled as terminal_heading_enabled
    terminal_heading_enabled(profile)
    from embodied_navigation.planning.scan_reference_protocol import enabled as reference_protocol_enabled
    reference_protocol_enabled(profile)
    config = profile.get('r85_auto_reference_contract', {})
    if not isinstance(config, dict):
        raise ValueError('r85_auto_reference_contract_must_be_object')
    if config.get('enabled') is not True:
        return False
    if ('resume_from_actual_route_projection' in config
            and type(config['resume_from_actual_route_projection']) is not bool):
        raise ValueError('r85_resume_from_actual_route_projection_requires_boolean')
    if (config.get('body_ground_offset_m') != .32
            or config.get('scan_transport_offset_m') != .4
            or config.get('max_reference_points') != MAX_REFERENCE_POINTS):
        raise ValueError('r85_auto_reference_profile_contract_mismatch')
    return True


def digest_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def xyz(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in value)):
        raise ValueError('r85_finite_xyz_required')
    return list(map(float, value))


def integer(value, name, minimum=0):
    if type(value) is not int or value < minimum:
        raise ValueError('r85_' + name + '_invalid_integer')
    return value


def exact_sent_path(msg, offset=.4):
    return dict(frame_id=msg.header.frame_id,
                stamp=[int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)],
                body_height_offset=offset,
                input_poses=[dict(frame_id=p.header.frame_id,
                    stamp=[int(p.header.stamp.sec), int(p.header.stamp.nanosec)],
                    position=[float(p.pose.position.x), float(p.pose.position.y), float(p.pose.position.z)],
                    orientation_xyzw=[float(p.pose.orientation.x), float(p.pose.orientation.y),
                                      float(p.pose.orientation.z), float(p.pose.orientation.w)]) for p in msg.poses])


class AutomaticReferenceRuntime:
    def __init__(self, run, profile, association, connector_certifier=None):
        if not enabled(profile):
            raise ValueError('r85_auto_reference_disabled')
        self.run = Path(run).resolve()
        self.profile = profile
        self.config = copy.deepcopy(profile['r85_auto_reference_contract'])
        self.association = association
        name = self.config.get('reference_file', 'r85_global_reference.json')
        if not isinstance(name, str) or Path(name).name != name:
            raise ValueError('r85_reference_must_be_a_run_leaf_file')
        self.path = self.run / name
        self.doc = self.file_sha = self.scope = self.reference = None
        self.replan_sequence = 0
        self.connector_certifier = connector_certifier
        self.last_connector = None

    def _registered_source(self, raw_sha, doc):
        return dict(run_id=self.run.name, reference_id=doc['reference_id'],
                    epoch=doc['epoch'], localization_generation=doc['localization_generation'])

    def load(self, planner_epoch, generation):
        if self.path.stat().st_size > 2_000_000:
            raise ValueError('r85_global_reference_size_budget')
        raw = self.path.read_bytes()
        d = json.loads(raw)
        if (d.get('schema') != REFERENCE_SCHEMA or d.get('run_id') != self.run.name
                or d.get('frame_id') != 'camera_init'
                or integer(d.get('epoch'), 'planner_epoch') != planner_epoch
                or integer(d.get('localization_generation'), 'localization_generation') != generation
                or not isinstance(d.get('reference_id'), str) or not d['reference_id']
                or d.get('physical_body_ground_offset_m') != .32 or d.get('scan_transport_offset_m') != .4
                or d.get('navigation_ground_truth_used') is not False):
            raise ValueError('r85_global_reference_scope_or_height_mismatch')
        body, ground = d.get('body_points'), d.get('ground_points')
        if not isinstance(body, list) or not 2 <= len(body) <= MAX_REFERENCE_POINTS or not isinstance(ground, list) or len(body) != len(ground):
            raise ValueError('r85_global_reference_point_budget')
        body, ground = [xyz(p) for p in body], [xyz(p) for p in ground]
        if any(math.dist(a, b) <= 1e-6 for a, b in zip(body[:-1], body[1:])):
            raise ValueError('r85_duplicate_global_body_segment')
        if any(max(abs(b[i]-g[i]-(.32 if i == 2 else 0.)) for i in range(3)) > 1e-8 for b,g in zip(body,ground)):
            raise ValueError('r85_true_ground_body_offset_mismatch')
        xyz(d.get('task_goal_body_xyz'))
        from embodied_navigation.control.parking.terminal_heading_contract import validate_reference_heading
        validate_reference_heading(self.profile, d)
        registration = dict(run_id=self.run.name, reference_id=d['reference_id'],
                            epoch=planner_epoch, localization_generation=generation)
        d = copy.deepcopy(d)
        self.doc, self.file_sha = d, d['reference_id']
        self.registration = registration
        return self.doc

    def _ready_scope(self, owner):
        if not owner._known_generation_ready():
            raise ValueError('r85_actual_corrected_sources_or_planner_not_ready')
        bundle, ready = owner.known_scene_inputs, owner.known_scene_scan_epoch
        generation = integer(bundle['generation'], 'localization_generation')
        if integer(ready['generation'], 'ready_localization_generation') != generation:
            raise ValueError('r85_ready_localization_generation_mismatch')
        epoch = integer(ready.get('planner_epoch'), 'planner_epoch', 1)
        identity = ready.get('planner_identity')
        if not isinstance(identity, dict):
            raise ValueError('r85_actual_planner_identity_missing')
        pid = integer(identity.get('pid'), 'planner_process_id', 1)
        if ready.get('run_id') != self.run.name or ready.get('ready') is not True:
            raise ValueError('r85_actual_planner_scope_mismatch')
        clock = integer(owner.get_clock().now().nanoseconds, 'control_clock_ns', 1)
        pose_stamp = integer(int(owner.pose_stamp), 'pose_stamp_ns', 1)
        if not 0 <= clock-pose_stamp < 300_000_000:
            raise ValueError('r85_actual_start_pose_not_causal_fresh')
        scope = (epoch, pid, generation)
        if self.scope != scope:
            self.scope, self.doc, self.file_sha, self.reference = scope, None, None, None
            self.replan_sequence = 0
            self.association.configure_epoch(epoch, pid, generation)
        return scope, clock, pose_stamp

    def _physical_connector(self, owner, d, scope, pose_stamp, selected_suffix=None):
        if self.connector_certifier is None:
            module = importlib.import_module('embodied_navigation.planning.admission.static_geometry')
            self.connector_certifier = module.certify_start_connector
        actual = xyz(owner.pose.tolist())
        if selected_suffix is None:
            certificate = self.connector_certifier(d, actual, pose_stamp, scope[2], planner_epoch=scope[0])
        else:
            certificate = self.connector_certifier(d, actual, pose_stamp, scope[2], planner_epoch=scope[0],
                resume_from_actual_route_projection=True)
        if (not isinstance(certificate, dict) or certificate.get('schema') != CONNECTOR_SCHEMA
                or certificate.get('run_id') != self.run.name
                or certificate.get('reference_id') != d['reference_id']
                or certificate.get('actual_start_body_xyz') != actual
                or certificate.get('source_pose_stamp_ns') != pose_stamp
                or certificate.get('localization_generation') != scope[2]
                or certificate.get('planner_epoch') != scope[0]):
            self.last_connector = certificate
            raise ValueError('r85_actual_start_physical_connector_not_certified')
        if selected_suffix is not None and (certificate.get('route_resume') != selected_suffix['receipt']
                or certificate.get('target_first_body_xyz') != selected_suffix['body_points'][0]):
            raise ValueError('r85_actual_start_resume_connector_target_mismatch')
        if certificate.get('allowed') is not True:
            config=self.profile.get('diagnostic_observe_only',{})
            if not (isinstance(config,dict) and type(config.get('enabled'))is bool and config['enabled']
                    and certificate.get('reason')=='physical_connector_not_certified'):
                self.last_connector=certificate
                raise ValueError('r85_actual_start_physical_connector_not_certified')
            # Source/contract identity was checked above. Reject nonfinite
            # derived scores before a deduplicating ledger can grant permission.
            canonical=json.dumps(certificate,allow_nan=False)
            if not canonical or not owner._known_generation_ready():
                raise ValueError('r85_actual_start_observation_source_not_ready')
            from embodied_navigation.common.observation import api as _observer_api; record = _observer_api('diagnostic_observation').record
            if record(self.run,self.profile,'r85_actual_start_physical_prediction',
                    'r85_actual_start_physical_connector_not_certified',certificate)is not True:
                raise RuntimeError('r85_actual_start_observation_not_recorded')
            certificate=dict(certificate,diagnostic_execution_continued=True,
                original_physical_permission_passed=False,navigation_PASS=False)
        self.last_connector = copy.deepcopy(certificate)
        owner.evidence.append(self.run/'r85_actual_start_connectors.jsonl',certificate)
        return certificate

    def request_plan(self, owner):
        scope, clock, pose_stamp = self._ready_scope(owner)
        if owner.reference_pub.get_subscription_count() == 0:
            raise ValueError('r85_SCAN_reference_subscription_missing')
        d = self.load(scope[0], scope[2])
        if self.reference is None:
            selected_suffix = None
            if self.config.get('resume_from_actual_route_projection') is True:
                from embodied_navigation.planning.reference.route_suffix import select_route_suffix
                selected_suffix = select_route_suffix(d['body_points'], d['ground_points'], xyz(owner.pose.tolist()))
            certificate = self._physical_connector(owner, d, scope, pose_stamp, selected_suffix)
            # Geometry work can outlive the source freshness window. Never turn
            # that older spatial receipt into a newly published reference.
            checked_scope, clock, checked_pose_stamp = self._ready_scope(owner)
            if (checked_scope != scope or checked_pose_stamp != pose_stamp
                    or xyz(owner.pose.tolist()) != certificate['actual_start_body_xyz']):
                raise ValueError('r85_actual_start_changed_during_physical_certificate')
            body = d['body_points'] if selected_suffix is None else selected_suffix['body_points']
            index = 0
            while index < len(body) and math.dist(body[index], owner.pose.tolist()) <= 1e-6:
                index += 1
            if index == len(body):
                raise ValueError('r85_nonmoving_global_reference')
            # True ground+.32 is distinct from the paired SCAN transport offset .4.
            msg = owner.make_path(body[index:], height_offset=.4)
            stamp = max(owner.last_reference_stamp+1, owner.stamp(msg))
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp,1_000_000_000)
            expected = exact_sent_path(msg)
            self.association.request(expected['stamp'],body[-1],expected)
            owner.reference_pub.publish(msg)
            owner.last_reference_stamp = stamp
            owner.reference_requests += 1
            owner.last_reference = __import__('time').monotonic()
            self.reference = dict(schema='r85_actual_persistent_reference_publication/v1',
                run_id=self.run.name,reference_id=d['reference_id'],
                planner_epoch=scope[0],planner_process_id=scope[1],localization_generation=scope[2],
                source_pose_stamp_ns=pose_stamp,control_clock_ns=clock,sent_reference=expected,
                planner_body_goal=copy.deepcopy(body[-1]),task_goal_body_xyz=copy.deepcopy(d['task_goal_body_xyz']),
                actual_connector_certificate=certificate,max_reference_points=MAX_REFERENCE_POINTS,
                scoring_drives_reference=False,actual_active=False)
            if selected_suffix is not None:
                self.reference['actual_route_suffix'] = copy.deepcopy(selected_suffix['receipt'])
                self.reference['complete_task_reference_unchanged'] = True
            owner.evidence.append(self.run/'r85_reference_publications.jsonl',copy.deepcopy(self.reference))
            owner.message = '已发送一次 PCT 全局参考，等待匹配的局部 SCAN 候选'
            return True
        revision = getattr(self.association,'observed_reference_revision',None) or self.association.reference_revision
        commit_ack_used = False
        if revision is None:
            from embodied_navigation.planning.scan_reference_protocol import verified_revision
            revision = verified_revision(self.profile, owner.known_scene_scan_epoch, self.reference,
                                         scope, clock, __import__('time').monotonic())
            commit_ack_used = revision is not None
        if revision is None:
            owner.message = '等待规划器提交参考身份，未重复发送全局参考'
            return False
        if owner.local_replan_pub.get_subscription_count() == 0:
            raise ValueError('r85_local_replan_subscription_missing')
        self.replan_sequence += 1
        msg = owner.make_local_replan_message(scope[0], revision, self.replan_sequence, self.reference['sent_reference']['stamp'])
        owner.local_replan_pub.publish(msg)
        # No ROS callback dispatch occurs between this synchronous publish and
        # registration; unsolicited external-sequence metadata stays rejected.
        self.association.note_local_replan_request(self.replan_sequence)
        owner.last_reference = __import__('time').monotonic()
        owner.evidence.append(self.run/'r85_local_replan_requests.jsonl',dict(
            schema='r85_scoped_local_replan_request/v1',run_id=self.run.name,planner_epoch=scope[0],
            planner_process_id=scope[1],localization_generation=scope[2],reference_stamp=self.reference['sent_reference']['stamp'],
            committed_reference_revision=revision,sequence=self.replan_sequence,
            source_pose_stamp_ns=pose_stamp,control_clock_ns=clock,global_reference_republished=False))
        if commit_ack_used:
            owner.evidence.append(self.run/'r85_reference_protocol_authorizations.jsonl', dict(
                planner_epoch=scope[0], planner_process_id=scope[1], localization_generation=scope[2],
                committed_reference_revision=revision, local_replan_request_sequence=self.replan_sequence,
                expected_reference_stamp=self.reference['sent_reference']['stamp'], control_clock_ns=clock,
                actual_child_ACK=copy.deepcopy(owner.known_scene_scan_epoch['child_odometry_consumption']['latest_actual_child_ACK']),
                source='original_owned_child_odometry_ACK', authorizes_local_replan_request_only=True,
                association_revision_assigned=False, trajectory_or_admission_synthesized=False))
        from embodied_navigation.common.observation import api as _observer_api; retry_published = _observer_api('r74_protocol_observation_python').retry_published
        retry_published(owner, scope, revision, self.replan_sequence,
            self.reference['sent_reference']['stamp'], clock, pose_stamp,
            owner.last_reference, commit_ack_used)
        owner.message = '已请求沿同一 PCT 全局参考进行局部重规划'
        # Acknowledge only after publish, association registration and evidence.
        # Waiting or any caught failure above must remain retryable.
        return EARLY_REQUEST_ACK_ENABLED

    def points(self):
        if self.doc is None:
            raise ValueError('r85_route_not_loaded')
        return copy.deepcopy(self.doc['body_points'])

    def planner_goal(self):
        return self.points()[-1]

    def terminal_heading(self):
        if self.doc is None:
            raise ValueError('r85_terminal_heading_route_not_loaded')
        value = self.doc.get('task_terminal_heading_rad')
        if value is not None:
            if type(value) not in (int,float) or not math.isfinite(value):
                raise ValueError('r85_terminal_heading_nonfinite')
            return float(value)
        a,b=self.doc['body_points'][-2:]
        if math.hypot(b[0]-a[0],b[1]-a[1])<1e-6:
            raise ValueError('r85_terminal_heading_requires_horizontal_segment')
        return math.atan2(b[1]-a[1],b[0]-a[0])

    def fence(self):
        if self.doc is None or self.reference is None:
            raise ValueError('r85_reference_not_published_and_bound')
        return self.points(),self.profile['registered_route_fence'],dict(
            schema='r85_automatic_route_fence_binding/v1',reference_id=self.doc['reference_id'],
            planner_epoch=self.scope[0],planner_process_id=self.scope[1],localization_generation=self.scope[2],
            route_prior='supplied_simulation_map_PCT',navigation_ground_truth_used=False,
            occupancy_or_terrain_support_certified=False)


class AutomaticRouteFence:
    def __init__(self, runtime):
        self.runtime=runtime
    def load(self):
        return self.runtime.fence()
