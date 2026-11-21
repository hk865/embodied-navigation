"""Read-only DSH candidate preparation. This module never submits a waypoint.

Visibility is modeled potential novelty, not measured free space or a certificate
of safety, support, coverage, or visual localization quality.
"""
from __future__ import annotations

import hashlib
import io
import json
from pathlib import Path
import time

import numpy as np

from embodied_navigation.planning.pct.measured_tomogram import MeasuredPctPlanner
from .route_information import MeasuredSnapshot, RouteInformationScorer, sample_route


def _xyz(value, label):
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.isfinite(array).all():
        raise ValueError(label + ' must be finite estimator child-origin XYZ')
    return array.tolist()


def run_binding(run_dir):
    path = Path(run_dir).resolve()
    # Accept either experiment root or its native capture directory.
    if not (path / 'interactive').is_dir() and (path / 'capture' / 'interactive').is_dir():
        path = path / 'capture'
    canonical = str(path)
    return path, dict(canonical_run_dir=canonical,
        canonical_run_path_sha256=hashlib.sha256(canonical.encode()).hexdigest())


def frontier_candidates(snapshot, z=None):
    """Cluster supplied frontier cells; never use chart height as waypoint Z."""
    frontier = snapshot.get('frontier') or {}
    resolution = float(snapshot['resolution_m'])
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError('invalid frontier resolution')
    cells = frontier.get('cell_indices')
    if cells is None:
        cells = np.floor(np.asarray(frontier.get('cells', []), dtype=float).reshape(-1, 2) / resolution).astype(int).tolist()
    remaining = {tuple(map(int, xy)) for xy in cells}
    suggestions = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        stack, cluster = [seed], [seed]
        while stack:
            x, y = stack.pop()
            for dx, dy in ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)):
                other = (x+dx,y+dy)
                if other in remaining:
                    remaining.remove(other); stack.append(other); cluster.append(other)
        centre = np.mean(cluster, axis=0)
        representative = min(cluster, key=lambda p: (float(np.sum((np.asarray(p)-centre)**2)), p))
        xy = [(v+.5)*resolution for v in representative]
        item = dict(id='frontier_%04d' % len(suggestions), xy_m=xy,
                    cluster_cell_count=len(cluster), role='auxiliary',
                    reason='representative of supplied measured free/unknown boundary cluster')
        if z is not None:
            item['goal_xyz_m'] = _xyz(xy + [z], 'frontier goal')
        suggestions.append(item)
    return dict(suggestions=suggestions, explicit_waypoint_z_m=z,
        candidate_source='supplied_operator_frontier',
        supplied_cell_count=len(cells), source_total_cell_count=frontier.get('count', len(cells)),
        source_truncated=frontier.get('count',len(cells)) > len(cells),
        candidate_limit=None, actionable_used_as_gate=False,
        limitation='Chart indices and chart heights do not identify physical floors or waypoint Z.')


def remaining_active_route(observation, artifact, binding, current):
    """Bind a diagnostic global route to the active goal; trim at nearest segment."""
    waypoints = observation.get('waypoints') or {}
    seq = waypoints.get('active_seq')
    if seq is None:
        raise ValueError('prepare_next_requires_active_goal')
    for key, expected in [('run_id', observation.get('run_id')), ('seq', seq),
                          ('frame_id', observation['pose'].get('map_frame')),
                          ('child_frame_id', observation['pose'].get('sensor_frame')),
                          ('canonical_run_path_sha256', binding['canonical_run_path_sha256'])]:
        if expected is None or artifact.get(key) != expected:
            raise ValueError('active_global_route_binding_mismatch:' + key)
    goal = _xyz(waypoints.get('active_goal_xyz_m'), 'active goal')
    if artifact.get('goal_xyz_m') != goal:
        raise ValueError('active_global_route_goal_mismatch')
    route = np.asarray(artifact.get('body_path_xyz_m'), dtype=float)
    if route.ndim != 2 or route.shape[1] != 3 or len(route) < 2 or not np.isfinite(route).all():
        raise ValueError('active_global_route_invalid')
    p = np.asarray(current)
    a, delta = route[:-1], np.diff(route, axis=0)
    norm = np.sum(delta*delta,axis=1)
    t = np.clip(np.sum((p-a)*delta,axis=1)/np.maximum(norm,1e-12),0,1)
    closest = a + t[:,None]*delta
    index = int(np.argmin(np.linalg.norm(closest-p,axis=1)))
    # Current-to-projection segment is only a modeled visibility baseline.
    return goal, np.vstack([p, closest[index], route[index+1:]]), seq


def prepare_candidates(run_dir, candidates, *, prepare_next=False, scorer_options=None,
                       planner_options=None, frontier_snapshot=None, frontier_z=None,
                       offline=False, snapshot_path=None, observation_path=None, frontier_path=None,
                       planner_factory=MeasuredPctPlanner, scorer_factory=RouteInformationScorer):
    capture, binding = run_binding(run_dir)
    mailbox = capture / 'interactive'
    source_root = capture.parent if capture.name == 'capture' else capture
    if not offline and (snapshot_path is not None or observation_path is not None):
        raise ValueError('snapshot/observation overrides require explicit offline replay')
    map_path = Path(snapshot_path) if snapshot_path else mailbox / 'planner_measured_map.npz'
    if not map_path.resolve().is_relative_to(source_root):
        raise ValueError('measured_snapshot_outside_current_run')
    raw = map_path.read_bytes()  # One immutable snapshot, including its provenance.
    observation_path = Path(observation_path) if observation_path else mailbox / 'observation.json'
    if not observation_path.resolve().is_relative_to(source_root):
        raise ValueError('observation_outside_current_run')
    if frontier_path is not None:
        frontier_path = Path(frontier_path).resolve()
        if not frontier_path.is_relative_to(source_root):
            raise ValueError('frontier_outside_current_run')
        frontier_snapshot = json.loads(frontier_path.read_text())
    observation = json.loads(observation_path.read_text())
    with np.load(io.BytesIO(raw), allow_pickle=False) as saved:
        provenance = json.loads(str(saved['provenance_json'].item()))
        points = np.asarray(saved['points_xyz_m'], dtype=float).copy()
        history = np.asarray(saved['trajectory_body_xyz_m'],dtype=float).reshape(-1,3).copy()
        lio_poses = np.asarray(saved['lio_pose_xyz_xyzw'],dtype=float).copy() if 'lio_pose_xyz_xyzw' in saved else None
    pose = observation.get('pose') or {}
    if provenance.get('run_id') != observation.get('run_id') or provenance.get('run_id') != capture.name:
        raise ValueError('snapshot_observation_run_mismatch')
    if provenance.get('map_frame') != pose.get('map_frame') or provenance.get('child_frame') != pose.get('sensor_frame'):
        raise ValueError('snapshot_observation_frame_mismatch')
    if pose.get('source') not in ('livo_post_vio', 'backend_corrected'):
        raise ValueError('candidate_start_requires_post_vio')
    current = _xyz(pose.get('position_xyz_m'), 'current pose')
    source = dict(provenance, **binding, map_sha256=hashlib.sha256(raw).hexdigest(),
                  source_path=str(map_path), measured_free_space_available=False)
    snapshot = MeasuredSnapshot(points, generation=binding['canonical_run_path_sha256'],
                                map_frame=provenance['map_frame'], source_metadata=source)
    start, active_seq = current, None
    if lio_poses is not None and lio_poses.ndim == 2 and lio_poses.shape[1] == 7 and len(lio_poses):
        if not np.isfinite(lio_poses).all():
            raise ValueError('nonfinite_lio_baseline')
        baseline_history = lio_poses[:,:3]
        quaternions = lio_poses[:,3:]
        norms = np.linalg.norm(quaternions,axis=1)
        if np.any(norms < 1e-8):
            raise ValueError('invalid_lio_baseline_quaternion')
        x,y,z,w = (quaternions/norms[:,None]).T
        history_yaws = np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
        baseline_pose_source = 'paired_LIO_child_poses_and_measured_quaternion_yaw'
    else:
        baseline_history = history
        delta = np.diff(history[:,:2],axis=0) if len(history)>1 else np.zeros((1,2))
        modeled_yaws = np.arctan2(delta[:,1],delta[:,0])
        history_yaws = np.r_[modeled_yaws,modeled_yaws[-1]][:len(history)]
        baseline_pose_source = 'post_VIO_history_with_modeled_path_tangent_yaw; paired_LIO_yaw_unavailable'
    current_yaw = pose.get('yaw_rad')
    current_yaw_source = 'measured_post_VIO'
    if current_yaw is None:
        current_yaw = float(history_yaws[-1]) if len(history_yaws) else 0.
        current_yaw_source = 'modeled_last_history_yaw_or_zero; current_yaw_unavailable'
    baseline = np.vstack([baseline_history,np.asarray([current])])
    baseline_yaws = np.r_[history_yaws,float(current_yaw)]
    preparation = dict(mode='current_pose', active_seq=None,
                       installs_cached_route=False, automatic_dispatch=False)
    extra_baseline_routes = []
    if prepare_next:
        artifact = json.loads((mailbox / 'planner_global_route.json').read_text())
        start, remaining, active_seq = remaining_active_route(observation, artifact, binding, current)
        options = dict(scorer_options or {})
        spacing = float(options.get('route_spacing_m',1.0))
        sampled_remaining, remaining_yaws, remaining_sampling = sample_route(remaining, spacing_m=spacing,
            max_samples=int(options.get('max_route_samples',32)))
        extra_baseline_routes.append(remaining)
        preparation.update(mode='after_active_goal', active_seq=active_seq,
            remaining_active_route_points=len(remaining), remaining_route_sampling=remaining_sampling,
            marginal_baseline='modeled historical/current poses plus remaining active global route',
            cached_start_policy='advisory only; native planner replans from actual pose on explicit goto')
    options = dict(scorer_options or {})
    options['baseline_poses_xyz'] = baseline
    options['baseline_yaws_rad'] = baseline_yaws
    options['extra_baseline_routes'] = extra_baseline_routes
    scorer = scorer_factory(snapshot, **options)
    planner = planner_factory(points, trajectory_body_xyz=history,
        generation=binding['canonical_run_path_sha256'], source_metadata=source,
        **dict(planner_options or {}))
    candidate_list = list(candidates)
    frontier_report = None
    if frontier_snapshot is not None:
        if frontier_snapshot.get('ground_truth_used') is True:
            raise ValueError('truth_frontier_refused')
        frontier_frame = (frontier_snapshot.get('frames') or {}).get('map',frontier_snapshot.get('map_frame'))
        if frontier_frame is not None and frontier_frame != provenance['map_frame']:
            raise ValueError('frontier_map_frame_mismatch')
        frontier_report = frontier_candidates(frontier_snapshot,frontier_z)
        frontier_report['generation_binding'] = ('same canonical run source path; legacy snapshot has no unique generation ID' if frontier_path else 'caller_supplied; no unique generation identity in legacy frontier snapshot')
        if frontier_path:
            frontier_report['source_path'] = str(frontier_path)
            frontier_report['source_sha256'] = hashlib.sha256(frontier_path.read_bytes()).hexdigest()
        candidate_list.extend({k:v for k,v in row.items() if k in ('id','goal_xyz_m','role','reason')}
                              for row in frontier_report['suggestions'] if 'goal_xyz_m' in row)
    rows, ids = [], set()
    for candidate in candidate_list:
        identifier = candidate.get('id')
        if not isinstance(identifier,str) or not identifier or identifier in ids:
            raise ValueError('candidate IDs must be unique nonempty strings')
        ids.add(identifier)
        if candidate.get('role') not in ('long_goal','auxiliary'):
            raise ValueError('candidate role must be long_goal or auxiliary')
        goal = _xyz(candidate.get('goal_xyz_m'),'candidate goal')
        row = dict(id=identifier, goal_xyz_m=goal, role=candidate['role'],
                   reason=candidate.get('reason',''), score=None)
        try:
            result = planner.plan(start,goal)
            row.update(route_found=bool(result['found']), route_reason=result.get('reason'),
                       body_path_xyz_m=result.get('body_path_xyz',[]),
                       route_provenance=result.get('provenance',{}))
            if result['found']:
                row['score'] = scorer.score_route(result['body_path_xyz'])
        except Exception as error:
            row.update(route_found=False, route_reason='candidate_evaluation_error:'+str(error), body_path_xyz_m=[])
        rows.append(row)
    # A plan can finish after the active sequence changes. Report this race;
    # advice never installs a cached route or blocks the native action interface.
    latest_observation = json.loads(observation_path.read_text())
    latest_seq = (latest_observation.get('waypoints') or {}).get('active_seq')
    preparation['active_seq_at_completion'] = latest_seq
    preparation['binding_unchanged_at_completion'] = (
        latest_observation.get('run_id') == observation.get('run_id') and
        (not prepare_next or latest_seq == active_seq))
    ranked = sorted((row for row in rows if row['score'] is not None),
                    key=lambda row: (-row['score']['utility_voxels_per_m'],row['id']))
    return dict(schema='dynanav_waypoint_candidates/v1', advisory_only=True,
        generated_wall_unix_s=time.time(), source=source, start_xyz_m=start,
        offline_replay=bool(offline), source_times_rewritten=False,
        implementation_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (Path(__file__),Path(__file__).with_name('route_information.py'),
                      Path(__file__).with_name('os1_calibration.json'))},
        preparation=preparation, candidates=rows, ranked_candidate_ids=[row['id'] for row in ranked],
        candidate_source='DSH explicit candidates' if frontier_report is None else 'DSH candidates plus supplied frontier clusters',
        candidate_limit=None, candidate_count=len(rows), frontier=frontier_report,
        visual_localization=dict(status='unavailable',score=None,
            reason='No reliable feature-track or localization-quality evidence supplied.'),
        diagnostics=dict(map_file_wall_age_s=max(0,time.time()-map_path.stat().st_mtime),
            observation_file_wall_age_s=max(0,time.time()-observation_path.stat().st_mtime),
            observation_pose_fresh=pose.get('fresh'), pose_sim_age_s=pose.get('sim_age_s'),
            source_interval_ns=provenance.get('source_interval_ns'), partial_collection=True,
            collection_limitation='Retained current-worker measured cloud; not full-run coverage.',
            modeled_baseline_not_measured_free=True, baseline_pose_source=baseline_pose_source,
            current_yaw_source=current_yaw_source),
        limitations=['Expected LiDAR novelty is model-dependent, not observed free space or coverage.',
            'DSH alone chooses and explicitly submits goals using the existing goto interface.',
            'Source age and utility are advisory diagnostics, not new goto admission gates.'])
