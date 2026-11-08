"""One-way privileged Actor terrain switch authorized by a measured SLAM arrival.

No navigation file, pose, command publisher or actuator is written here. Native
state is used only to verify identical Actor ray inputs on the shared landing.
Safety retains the original all-static height map independently of this proxy.
"""
from __future__ import annotations

from embodied_navigation.project import project_root as _project_root

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


class SwitchingTerrainProvider:
    """Delegate unchanged rays to one of two frozen static geometry maps."""

    def __init__(self, run, scope, map_class, observation):
        self.run = Path(run).resolve()
        self.scope = scope
        self.profile = scope['profile']
        self.config = self.profile['terrain_layer_switch']
        self.current_id = 'initial'
        self.switched = False
        self.failure = None
        self.state = None
        self.event = None
        self.pending = None
        self.last_status_attempt = None
        self.event_sequence = 0
        self.observation = observation
        self.world = self.run / 'world.sdf'
        self._frozen(self.world)
        if self.config.get('completed_goal_id') != 'connector_mid':
            raise ValueError('Terrain switch requires the explicitly reviewed connector_mid arrival')
        root = _project_root()
        initial_source = (root / self.config['initial_manifest']).resolve()
        alternate_source = (root / self.config['alternate_manifest']).resolve()
        self.initial_manifest_path = self.run / 'terrain_target_manifest.json'
        self.alternate_manifest_path = self.run / 'alternate_terrain_target_manifest.json'
        for path in (initial_source, alternate_source, self.initial_manifest_path, self.alternate_manifest_path):
            self._frozen(path)
        initial_raw = self.initial_manifest_path.read_bytes()
        alternate_raw = self.alternate_manifest_path.read_bytes()
        self.initial_manifest = json.loads(initial_raw)
        self.alternate_manifest = json.loads(alternate_raw)
        if (self.initial_manifest != json.loads(initial_source.read_bytes()) or
                self.alternate_manifest != json.loads(alternate_source.read_bytes())):
            raise ValueError('Run terrain manifests differ from their frozen source definitions')
        self.initial_models = self._models(self.initial_manifest)
        self.alternate_models = self._models(self.alternate_manifest)
        if (set(self.initial_models) != {'floor_1', 'ramp_12', 'floor_2'} or
                set(self.alternate_models) != {'floor_2', 'ramp_23', 'floor_3'} or
                self.config.get('alternate_include_models') != self.alternate_models or
                self.profile['terrain_target_manifest'] != self.config['initial_manifest']):
            raise ValueError('Terrain manifests/configuration differ from the reviewed lower12 to upper23 transition')
        self.maps = {
            'initial': map_class.from_sdf(self.world, include_models=self.initial_models),
            'alternate': map_class.from_sdf(self.world, include_models=self.alternate_models),
        }
        for key, expected in (('initial', self.initial_models), ('alternate', self.alternate_models)):
            actual = {shape.name.split('/')[0] for shape in self.maps[key].shapes}
            if actual != set(expected):
                raise ValueError('Actual SDF terrain set differs from frozen ' + key + ' manifest')
        self.goal_regions = self._goal_module()
        self.manifest_hashes = dict(initial=digest(initial_raw), alternate=digest(alternate_raw))
        self.log = (self.run / 'terrain_provider_events.jsonl').open('x', buffering=1)
        self._record(dict(event='initialized', contract=self.contract()))

    def _frozen(self, path):
        path = Path(path).resolve()
        expected = self.scope['references'].get(str(path))
        if expected is None or digest(path.read_bytes()) != expected:
            raise ValueError('Terrain switch requires a matching frozen input: ' + str(path))
        return expected

    @staticmethod
    def _models(manifest):
        values = manifest.get('include_models')
        if (manifest.get('schema_version') != 1 or manifest.get('navigation_truth_used') is not False or
                not isinstance(values, list) or not values or len(values) != len(set(values)) or
                not all(isinstance(value, str) and value for value in values)):
            raise ValueError('Invalid explicit privileged terrain target manifest')
        return values

    def _goal_module(self):
        # Execute the same archived pure geometry normalizer as the navigation
        # controller, so default fields cannot change the goal-definition hash.
        snapshots = json.loads((self.run / 'navigation_source_snapshots.json').read_text())
        self._frozen(self.run / 'navigation_source_snapshots.json')
        candidates = [row for source, row in snapshots.items()
                      if Path(source).name == 'goal_regions.py' and Path(source).parent.name == 'navigation']
        if len(candidates) != 1:
            raise ValueError('No unique archived navigation goal normalizer')
        path = Path(candidates[0]['snapshot']).resolve()
        if not path.is_relative_to(self.run / 'sources'):
            raise ValueError('Goal normalizer snapshot is outside this run')
        self._frozen(path)
        if digest(path.read_bytes()) != candidates[0]['sha256']:
            raise ValueError('Goal normalizer snapshot hash mismatch')
        name = 'teacher_multifloor_frozen_goal_regions'
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        return module

    @property
    def current(self):
        return self.maps[self.current_id]

    @property
    def shapes(self):
        return self.current.shapes

    @property
    def source(self):
        return self.current.source

    @property
    def source_sha256(self):
        return self.current.source_sha256

    @property
    def excluded_models(self):
        return self.current.excluded_models

    def raycast(self, starts):
        return self.current.raycast(starts)

    def height(self, xy, ray_start_z=20.):
        return self.current.height(xy, ray_start_z=ray_start_z)

    def observe_state(self, state):
        value = np.asarray(state, dtype=np.float64)
        if value.shape != (64,) or not np.isfinite(value).all():
            raise ValueError('Terrain provider requires the unmodified finite native state64')
        self.state = value.copy()

    def contract(self):
        return dict(schema='teacher_actor_terrain_switch/v1',
                    initial_manifest=str(self.initial_manifest_path),
                    alternate_manifest=str(self.alternate_manifest_path),
                    manifest_sha256=self.manifest_hashes,
                    completed_goal_id=self.config['completed_goal_id'],
                    trigger='original actual SLAM region_arrivals with request/goal-definition hash binding',
                    equality='all 187 frozen yaw-only grid rays finite and on floor_2; max hit-z difference <=1e-9 m',
                    scan_semantics='unchanged 17x11 grid, ray origin base_z+20, base_z-hit_z-.5, original clip/scale',
                    native_state_use='privileged Actor ray equality only; never navigation feedback or layer selection',
                    failure='latch initial provider and slew requested velocity to zero; continue learned Teacher feedback',
                    safety='independent original all-static map at body-height ray origin',
                    navigation_ground_truth_used=False)

    def evidence(self):
        return dict(schema='teacher_actor_terrain_switch/v1', current_provider=self.current_id,
                    include_models=list(self.initial_models if self.current_id == 'initial' else self.alternate_models),
                    manifest_sha256=self.manifest_hashes[self.current_id], switched=self.switched,
                    failure=self.failure, event_sequence=self.event_sequence,
                    arrival_stamp_ns=None if self.event is None else self.event.get('arrival_stamp_ns'),
                    pending_arrival_stamp_ns=None if self.pending is None else self.pending['stamp_ns'],
                    navigation_ground_truth_used=False)

    def _record(self, value):
        if ('navigation_ground_truth_used' in value and
                value['navigation_ground_truth_used'] is not False):
            raise ValueError('Terrain event cannot authorize ground-truth navigation')
        self.event_sequence += 1
        row = dict(value)
        row.update(sequence=self.event_sequence, recorded_monotonic_wall=time.monotonic(),
                   navigation_ground_truth_used=False)
        self.log.write(canonical(row) + '\n')
        return row

    def _read_json(self, name):
        start = time.monotonic()
        raw = (self.run / name).read_bytes()
        text = raw.decode('utf-8')
        value = json.loads(text)
        return value, dict(path=str(self.run / name), raw_bytes_sha256=digest(raw), raw_utf8=text,
                           read_started_monotonic_wall=start, read_completed_monotonic_wall=time.monotonic())

    def check_switch(self):
        """Return True only when a zero-velocity protective request is needed."""
        if self.failure is not None:
            return True
        if self.switched:
            return False
        if self.state is None:
            return False
        clock_ns = int(round(float(self.state[0]) * 1e9))
        try:
            status, status_read = self._read_json('navigation_status.json')
        except (OSError, ValueError, UnicodeError):
            # Status may legitimately be absent before the SLAM route exists.
            # The separate command freshness gate still protects execution.
            return self.pending is not None
        if not isinstance(status, dict) or not isinstance(status.get('region_arrivals', []), list):
            return self.pending is not None
        matches = [row for row in status.get('region_arrivals', [])
                   if isinstance(row, dict) and row.get('goal_id') == self.config['completed_goal_id']]
        if not matches:
            return self.pending is not None
        proof = dict(status_read=status_read, native_request_clock_ns=clock_ns,
                     native_state_effective_clock_ns=clock_ns - 5_000_000,
                     native_state_use='Actor ray equivalence only', actor_native_base_origin=self.state[1:4].tolist(),
                     actor_native_quaternion_wxyz=self.state[4:8].tolist())
        try:
            if len(matches) != 1:
                raise ValueError('Connector arrival is not unique')
            arrival = matches[0]
            proof['original_region_arrival'] = arrival
            proof['region_arrival_sha256'] = digest(canonical(arrival).encode())
            request, request_read = self._read_json('navigation_request.json')
            proof['request_read'] = request_read
            rid, goals = self.goal_regions.parse_request(request)
            definitions = [goal.definition() for goal in goals]
            goal_hash = self.goal_regions.definitions_sha256(goals)
            if (not rid.startswith(self.run.name + '_') or status.get('request_id') != rid or
                    arrival.get('request_id') != rid or status.get('frame_id') != 'camera_init' or
                    status.get('mode') != 'teacher' or status.get('scope') != self.profile['experiment'] or
                    status.get('navigation_ground_truth_used') is not False):
                raise ValueError('Connector arrival lacks the actual run/SLAM scope identity')
            if (status.get('goals_definition_sha256') != goal_hash or arrival.get('goals_definition_sha256') != goal_hash or
                    canonical(status.get('goals_definitions')) != canonical(definitions)):
                raise ValueError('Connector request and completed goal definitions have different hashes')
            index = arrival.get('waypoint_index')
            if type(index) is not int or not 0 <= index < len(goals) or goals[index].goal_id != self.config['completed_goal_id']:
                raise ValueError('Connector goal index differs from the frozen request')
            goal = goals[index]
            if (canonical(arrival.get('arrival_definition')) != canonical(goal.definition()['arrival']) or
                    canonical(arrival.get('control_arrival_definition')) != canonical(goal.control_arrival_definition())):
                raise ValueError('Connector arrival/control region changed')
            stamp, begin, dwell, gap = [arrival.get(key) for key in
                                       ('stamp_ns', 'start_stamp_ns', 'dwell_ns', 'max_observation_gap_ns')]
            if (not all(type(value) is int for value in (stamp, begin, dwell, gap)) or
                    not 0 <= begin <= stamp or dwell != stamp - begin or
                    dwell < int(round(goal.dwell_sim_s * 1e9)) or not 0 < gap <= 200_000_000 or
                    arrival.get('region_inside') is not True or arrival.get('control_region_inside') is not True or
                    arrival.get('protected') is not False or arrival.get('reason') != 'arrived' or
                    not self.goal_regions.contains_control(goal, arrival.get('raw_position'))):
                raise ValueError('Connector has no valid original measured-stamp dwell/inside/protection evidence')
            proof.update(request_id=rid, goals_definition_sha256=goal_hash, arrival_stamp_ns=stamp,
                         goal_id=goal.goal_id, source_pose=arrival['raw_position'])
            if self.pending is not None and proof['region_arrival_sha256'] != self.pending['sha256']:
                raise ValueError('Pending original arrival changed before causal application')
            if stamp > clock_ns:
                if stamp - clock_ns > 50_000_000:
                    raise ValueError('Connector source stamp exceeds original clock future tolerance')
                if self.pending is None:
                    self.pending = dict(stamp_ns=stamp, sha256=proof['region_arrival_sha256'])
                    self._record(dict(event='waiting_for_causal_native_clock', **proof))
                return True
            # Reproduce the exact original Actor ray starts, with no frame,
            # bounds, height, clipping or coordinate correction.
            rotation = self.observation.quaternion_rotation(self.state[4:8])
            yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
            c, s = np.cos(yaw), np.sin(yaw)
            xy = self.observation.SCAN_GRID_XY @ np.asarray([[c, s], [-s, c]]) + self.state[1:3]
            starts = np.column_stack((xy, np.full(187, self.state[3] + 20.)))
            lower_z, lower_names = self.maps['initial'].raycast(starts)
            upper_z, upper_names = self.maps['alternate'].raycast(starts)
            finite = (lower_z.shape == (187,) and upper_z.shape == (187,) and
                      np.isfinite(lower_z).all() and np.isfinite(upper_z).all())
            shared = (len(lower_names) == len(upper_names) == 187 and
                      all(isinstance(name, str) and name.split('/')[0] == 'floor_2'
                          for name in list(lower_names) + list(upper_names)))
            difference = float(np.max(np.abs(lower_z - upper_z))) if finite else None
            proof['ray_equivalence'] = dict(ray_count=187, finite=bool(finite), shared_floor_2=bool(shared),
                identical_float64_bytes=bool(finite and lower_z.tobytes() == upper_z.tobytes()),
                max_abs_hit_z_difference_m=difference, tolerance_m=1e-9,
                starts=starts.tolist(), initial_hit_z=[float(x) if np.isfinite(x) else None for x in lower_z],
                alternate_hit_z=[float(x) if np.isfinite(x) else None for x in upper_z],
                initial_collision_names=lower_names, alternate_collision_names=upper_names)
            if not finite or not shared or difference > 1e-9:
                raise ValueError('All 187 Actor rays must agree on the actual shared floor_2 landing')
            self.current_id = 'alternate'
            self.switched = True
            self.pending = None
            self.event = proof
            self._record(dict(event='switched_once', **proof, provider_after=self.current_id))
            return False
        except (OSError, ValueError, KeyError, TypeError, IndexError, OverflowError) as error:
            self.failure = type(error).__name__ + ': ' + str(error)
            self.event = proof
            self._record(dict(event='switch_refused_latched', **proof, failure=self.failure,
                              requested_velocity_after=[0., 0., 0.], provider_after=self.current_id))
            return True

    def close(self):
        if self.log.closed:
            return
        result = dict(**self.evidence(),
                      status='failed' if self.failure else 'switched' if self.switched else 'not_reached_unverified',
                      event=self.event, contract=self.contract(), event_log='terrain_provider_events.jsonl')
        self._record(dict(event='closed', status=result['status'], evidence=self.evidence()))
        self.log.close()
        with (self.run / 'terrain_provider_result.json').open('x') as output:
            output.write(canonical(result) + '\n')
