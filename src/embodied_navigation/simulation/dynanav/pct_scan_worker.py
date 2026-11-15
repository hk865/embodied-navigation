"""Measured-generation PCT -> native SCAN sidecar. Never imports Isaac.

Only actual native Bspline messages become follower paths. Path z uses the
SCAN surface convention; output z is the estimator child-origin convention.
No offline scene, truth map, direct-line fallback or waypoint admission gate.
"""
from __future__ import annotations

import argparse
from collections import OrderedDict, deque
from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import numpy as np

from embodied_navigation.perception.pose_velocity import update_pose_velocity
from embodied_navigation.planning.scan_handoff import ReferenceHandoff, reference_waypoints

ROOT = Path(__file__).resolve().parents[4]
CLOUD = '/dynanav/rtx_livo/cloud_registered_full'
LIO = '/dynanav/rtx_livo/aft_mapped_to_init'
VIO = '/dynanav/rtx_livo/post_vio_odometry'


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return {}


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.worker.tmp')
    temporary.write_text(json.dumps(value, allow_nan=False))
    temporary.replace(path)


def active_request(mailbox, run_id):
    """Observation, not a stale request file, owns cancellation/sequence state."""
    observation = read_json(Path(mailbox) / 'observation.json')
    request = read_json(Path(mailbox) / 'planner_request.json')
    if not isinstance(observation, dict) or not isinstance(request, dict):
        return None
    if observation.get('run_id') != run_id or request.get('run_id') != run_id:
        return None
    if observation.get('waypoints', {}).get('active_seq') != request.get('seq'):
        return None
    if request.get('version') != 1 or request.get('pose_source') != 'livo_post_vio':
        return None
    if not isinstance(request.get('seq'), int) or isinstance(request['seq'], bool):
        return None
    if not request.get('frame_id') or not request.get('child_frame_id'):
        return None
    try:
        point = np.asarray(request.get('goal_xyz_m'), dtype=float)
    except (TypeError, ValueError):
        return None
    if point.shape != (3,) or not np.isfinite(point).all():
        return None
    return request


def sample_bspline(order, knots, control_points, step_s=0.04):
    """Evaluate the native knot vector, not a polyline through control points."""
    from scipy.interpolate import BSpline
    points = np.asarray(control_points, dtype=float)
    knots = np.asarray(knots, dtype=float)
    if order != 3 or points.ndim != 2 or points.shape[1] != 3 or len(points) < 4:
        raise ValueError('invalid_native_spline_shape')
    if len(knots) != len(points) + order + 1 or not np.isfinite(points).all() or not np.isfinite(knots).all():
        raise ValueError('invalid_native_spline_knots')
    if np.any(np.diff(knots) < 0) or knots[-order-1] <= knots[order]:
        raise ValueError('invalid_native_spline_interval')
    duration = float(knots[-order-1] - knots[order])
    if duration > 300 or step_s <= 0:
        raise ValueError('invalid_native_spline_duration')
    times = np.linspace(knots[order], knots[-order-1], max(2, math.ceil(duration / step_s) + 1))
    samples = BSpline(knots, points, order, extrapolate=False)(times)
    if not np.isfinite(samples).all():
        raise ValueError('nonfinite_native_spline')
    return samples.tolist(), (times - knots[order]).tolist()


def scan_source_digest(root=ROOT):
    digest = hashlib.sha256()
    for path in sorted((Path(root) / 'third_party/scan').rglob('*')):
        if path.is_file():
            digest.update(str(path.relative_to(root)).encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()


def build_scan(build_root, jobs=2):
    """Build only checked-out native SCAN packages into an explicit external dir."""
    build_root = Path(build_root).resolve()
    if build_root.is_relative_to(ROOT):
        raise ValueError('SCAN build directory must be outside the checkout')
    digest = scan_source_digest()
    receipt = build_root / 'source_receipt.json'
    binary = build_root / 'install/lib/scan_planner/scan_planner_node'
    if binary.exists() and read_json(receipt).get('source_sha256') == digest:
        return binary
    from embodied_navigation.planning.scan_source import stage_scan_source
    staged_source = stage_scan_source(ROOT / 'third_party/scan', build_root)
    command = ['colcon', '--log-base', str(build_root / 'log'), 'build', '--base-paths',
               str(staged_source), '--build-base', str(build_root / 'build'),
               '--install-base', str(build_root / 'install'), '--merge-install',
               '--executor', 'sequential', '--cmake-args', '-DCMAKE_BUILD_TYPE=Release',
               '-DBUILD_TESTING=OFF', '-DPython3_EXECUTABLE=/usr/bin/python3']
    env = dict(os.environ, CMAKE_BUILD_PARALLEL_LEVEL=str(jobs))
    subprocess.run(command, check=True, env=env)
    atomic_json(receipt, {'source_sha256': digest, 'binary': str(binary), 'command': command})
    return binary


class OwnedScan:
    """One process per active request; terminate only the process we created."""
    def __init__(self, binary, mailbox, request, namespace, speed):
        import yaml
        self.process = None
        self.log = None
        self.namespace = namespace
        self.ack_nonce = uuid.uuid4().hex
        parameters = yaml.safe_load((ROOT / 'third_party/scan/plan_manage/config/planner.yaml').read_text())
        params = parameters['scan_planner_node']['ros__parameters']
        params.update({
            'use_sim_time': True, 'fsm.navi_mode': 3, 'fsm.external_reference_protocol': True,
            'fsm.reference_run_id': request['run_id'], 'fsm.reference_epoch': request['seq'],
            'fsm.localization_generation': 0, 'fsm.measured_max_speed': speed,
            'fsm.child_odometry_consumption_ack': True,
            'fsm.child_odometry_ack_nonce': self.ack_nonce,
            'fsm.planning_horizon': 0.65, 'manager.planning_horizon': 0.65,
            'fsm.thresh_replan': 0.35, 'grid_map.frame_id': request['frame_id'],
            'grid_map.sliding_map_frame_id': namespace.strip('/').replace('/', '_') + '_grid',
            'grid_map.body_height': 0.4, 'grid_map.resolution': 0.08,
            'grid_map.obstacles_inflation_z_up': 0.12, 'grid_map.obstacles_inflation_z_down': 0.12,
            'grid_map.cloud_is_world': True, 'grid_map.need_extrinsic': False,
            'manager.max_vel': speed, 'optimization.max_vel': speed,
            'manager.max_acc': 1.5, 'optimization.max_acc': 1.5,
            'optimization.acc_tolerance': 1.0,
        })
        # Wildcard matches the namespaced native node without changing native source.
        filename = Path(mailbox) / ('scan_parameters_%s.yaml' % request['seq'])
        filename.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
        self.log = (Path(mailbox) / ('scan_%s.log' % request['seq'])).open('a')
        self.process = subprocess.Popen([str(binary), '--ros-args', '--params-file', str(filename),
                                         '-r', '__ns:=' + namespace], stdout=self.log, stderr=subprocess.STDOUT)

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        if self.log is not None:
            self.log.close()


def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from nav_msgs.msg import Odometry, Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from scan_planner_msgs.msg import Bspline
    from std_msgs.msg import String
    from .rtx_deskewed_perception import (message_stamp_ns, verify_cloud_layout,
        verify_deskew_marker, verify_same_stage_pair, DESKEWED_STAGE)
    from embodied_navigation.planning.pct.measured_tomogram import MeasuredPctPlanner

    mailbox = Path(args.run_dir).resolve() / args.mailbox
    mailbox.mkdir(parents=True, exist_ok=True)
    run_id = Path(args.run_dir).resolve().name
    binary = Path(args.scan_prefix).resolve() / 'lib/scan_planner/scan_planner_node'
    receipt = read_json(Path(args.scan_prefix).resolve().parent / 'source_receipt.json')
    if not binary.is_file() or receipt.get('source_sha256') != scan_source_digest():
        raise RuntimeError('SCAN must be built from this checkout with --build-scan; source receipt missing/stale')
    rclpy.init()
    node = Node('dynanav_pct_scan_worker')
    velocity_policy = dict(source='post_vio_pose_window', window_s=0.5,
                           reset_gap_s=0.25, frame='estimator_map', input_topic=VIO,
                           raw_odometry_twist_used=False)
    atomic_json(mailbox / 'planner_worker_parameters.json', dict(
        reference_response_timeout_s=args.reference_response_timeout_s,
        body_velocity=velocity_policy, speed=args.speed))
    velocity_history = deque()
    # All accumulations are process-local and begin empty for this run.
    measured_voxels = {}
    measured_stamps = {}
    lio_sources = {}
    trajectory = []
    map_frames = None
    source_interval = [None, None]

    def plan_snapshot(points, track, start, goal, source):
        planner = MeasuredPctPlanner(points, trajectory_body_xyz=track,
            body_support_offset_m=args.body_support_offset, start_blind_radius_m=args.pct_start_blind_radius, native_binary=args.pct_binary,
            generation=run_id, source_metadata=source)
        result = planner.plan(start, goal)
        return dict(status=('ready' if result['found'] else
                    'pending' if result.get('reason') == 'no_measured_support' else 'failed'),
                    points_xyz_m=result['body_path_xyz'], reason=result.get('reason'),
                    provenance=result['provenance'])
    pool = ThreadPoolExecutor(max_workers=1)
    state = dict(request=None, child=None, revision=0, vio=None, pair=None, reference_stamp=None,
                 route_sent=False, future=None, route=None, last_attempt=0.0, handoff=None,
                 last_snapshot=0.0, terminal_failed=False, body_pose=None)
    clouds, poses, splines, metadata = OrderedDict(), OrderedDict(), {}, {}
    namespace = '/dynanav/pct_scan/worker_%s' % os.getpid()
    sensor_pub = node.create_publisher(Odometry, namespace + '/sensor_pose', 10)
    body_pub = node.create_publisher(Odometry, namespace + '/body_pose', 10)
    cloud_pub = node.create_publisher(PointCloud2, namespace + '/cloud', 10)
    path_pub = node.create_publisher(RosPath, namespace + '/initial_path', 1)

    def current():
        request = active_request(mailbox, run_id)
        return request is not None and state['request'] is not None and request == state['request']

    def output(status, reason=None, **extra):
        if not current() or state['terminal_failed']:
            return
        if status == 'failed':
            state['terminal_failed'] = True
            if state['handoff'] is not None:
                state['handoff'].fail(reason)
        request = state['request']
        state['revision'] += 1
        data = dict(version=1, run_id=run_id, seq=request['seq'], frame_id=request['frame_id'],
                    child_frame_id=request['child_frame_id'], pose_source='livo_post_vio', status=status,
                    revision=state['revision'], source_stamp_s=(message_stamp_ns(state['vio']) / 1e9
                    if state['vio'] is not None else None), points_xyz_m=[], **extra)
        if reason:
            data['reason'] = reason
        atomic_json(mailbox / 'planner_path.json', data)

    def publish_body_pose():
        message, handoff = state['body_pose'], state['handoff']
        if message is None or handoff is None or state['terminal_failed']:
            return
        request = state['request']
        if (message.header.frame_id, message.child_frame_id) != (request['frame_id'], request['child_frame_id']):
            return
        p = message.pose.pose.position
        handoff.sent_body_pose(message_stamp_ns(message), [p.x, p.y, p.z])
        body_pub.publish(message)

    def odometry(message):
        previous = state['vio']
        stamp = message_stamp_ns(message)
        frames = (message.header.frame_id, message.child_frame_id)
        if previous is not None and stamp <= message_stamp_ns(previous):
            return
        if previous is not None and (stamp - message_stamp_ns(previous) > 250_000_000 or
                frames != (previous.header.frame_id, previous.child_frame_id)):
            velocity_history.clear()
        p, q = message.pose.pose.position, message.pose.pose.orientation
        position = np.asarray([p.x, p.y, p.z])
        if not all(frames) or not np.isfinite([p.x, p.y, p.z, q.x, q.y, q.z, q.w]).all():
            velocity_history.clear()
            state['body_pose'] = None
            return
        state['vio'] = message
        trajectory.append(position.tolist())
        velocity = update_pose_velocity(velocity_history, stamp / 1e9, position,
                                        velocity_policy['window_s'])
        state['body_pose'] = None
        if velocity is not None:
            # SCAN consumes map-frame velocity. The original post-VIO message
            # stays unchanged; its zero twist is not a measured velocity.
            body = deepcopy(message)
            body.twist.twist.linear.x, body.twist.twist.linear.y, body.twist.twist.linear.z = map(float, velocity)
            state['body_pose'] = body
            publish_body_pose()

    def pair():
        nonlocal map_frames
        for stamp in sorted(set(clouds).intersection(poses)):
            cloud, pose = clouds.pop(stamp), poses.pop(stamp)
            try:
                verify_cloud_layout([field.name for field in cloud.fields])
                verify_same_stage_pair(cloud_frame=cloud.header.frame_id, pose_frame=pose.header.frame_id,
                    child_frame=pose.child_frame_id, cloud_stamp_ns=stamp, pose_stamp_ns=message_stamp_ns(pose),
                    expected_map_frame=pose.header.frame_id, expected_child_frame=pose.child_frame_id,
                    stage=DESKEWED_STAGE)
                raw = point_cloud2.read_points(cloud, field_names=['x', 'y', 'z', 'curvature'], skip_nans=True)
                verify_deskew_marker(raw['curvature'])
                xyz = np.column_stack([raw['x'], raw['y'], raw['z']]).astype(float)
                frames = (pose.header.frame_id, pose.child_frame_id)
                if map_frames is not None and frames != map_frames:
                    raise ValueError('measured_cloud_generation_frame_change')
                map_frames = frames
                if source_interval[1] is not None and stamp <= source_interval[1]:
                    continue
                source_interval[0] = stamp if source_interval[0] is None else source_interval[0]
                source_interval[1] = stamp
                # Retain actual measured representatives, not synthetic voxel centres.
                keys = np.floor(xyz / .08).astype(np.int64)
                for key, point in zip(keys, xyz):
                    measured_voxels[tuple(key)] = point
                    measured_stamps[tuple(key)] = stamp
                position, orientation = pose.pose.pose.position, pose.pose.pose.orientation
                lio_sources[stamp] = [position.x, position.y, position.z,
                                      orientation.x, orientation.y, orientation.z, orientation.w]
                state['pair'] = (cloud, pose)
                if state['child'] is not None:
                    # Native GridMap is latest-pose based, not message-filter synchronized.
                    # No claim of child-side exact-stamp consumption is made.
                    sensor_pub.publish(pose)
                    cloud_pub.publish(cloud)
            except ValueError as error:
                node.get_logger().warning(str(error))
        for cache in (clouds, poses):
            while len(cache) > 128:
                cache.popitem(last=False)

    def cloud(message):
        clouds[message_stamp_ns(message)] = message
        pair()

    def lio(message):
        poses[message_stamp_ns(message)] = message
        pair()

    def deliver(key):
        if (key not in splines or key not in metadata or not current() or
                state['terminal_failed'] or state['handoff'] is None):
            return
        handoff = state['handoff']
        if handoff.commit is None:
            return  # Definitions and spline messages use independent ROS topics.
        spline, info = splines.pop(key), metadata.pop(key)
        if not handoff.accepts_candidate(info):
            return
        if info.get('candidate_kind') == 'emergency_stop':
            output('hold', 'native_scan_emergency_stop')
            return
        if info.get('candidate_kind') != 'motion':
            return
        try:
            samples, times = sample_bspline(spline.order, spline.knots,
                [[p.x, p.y, p.z] for p in spline.pos_pts])
        except ValueError as error:
            output('failed', str(error))
            return
        state['revision'] += 1
        # Build the entire publication once to avoid duplicate keyword fields.
        provenance = dict(planner='native_PCT_plus_native_SCAN', scan_source_sha256=receipt['source_sha256'],
            scan_binary=str(binary), scan_process_id=info.get('planner_process_id'),
            trajectory_id=spline.traj_id, trajectory_start_time=list(key[1:]),
            reference_stamp=info['reference_stamp'], reference_revision=info.get('committed_reference_revision'),
            order=spline.order, knots=list(spline.knots),
            control_points_xyz_m=[[p.x, p.y, p.z] for p in spline.pos_pts],
            native_metadata=info, map_source=CLOUD, navigation_source=VIO,
            body_velocity=velocity_policy, child_odometry_ack=handoff.reference_odometry_ack,
            reference_commit=handoff.commit,
            local_planner='native_SCAN', global_planner='native_PCT_Astar',
            occupancy='native_3d_inflated_voxels', z_policy='native_linear_reference_zero_z_gradient',
            source_stamp_semantics='latest_post_vio_received_by_adapter_at_spline_delivery',
            sensor_pairing='adapter_exact_stamp_native_latest_pose', pct=state['route'].get('provenance', {}))
        request = state['request']
        atomic_json(mailbox / 'planner_path.json', dict(version=1, run_id=run_id, seq=request['seq'],
            frame_id=request['frame_id'], child_frame_id=request['child_frame_id'], pose_source='livo_post_vio',
            status='tracking', revision=state['revision'], source_stamp_s=message_stamp_ns(state['vio']) / 1e9,
            points_xyz_m=samples, sample_times_s=times, provenance=provenance))

    def spline_callback(message):
        key = (message.traj_id, message.start_time.sec, message.start_time.nanosec)
        splines[key] = message
        if len(splines) > 64:
            splines.pop(next(iter(splines)))
        deliver(key)

    def feedback_json(message):
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return None
        return value if isinstance(value, dict) else None

    def metadata_callback(message):
        info = feedback_json(message)
        if info is None:
            return
        try:
            key = (info['trajectory_id'], *info['trajectory_start_time'])
            metadata[key] = info
        except (KeyError, TypeError):
            return
        if len(metadata) > 64:
            metadata.pop(next(iter(metadata)))
        deliver(key)

    def odometry_ack_callback(message):
        handoff = state['handoff']
        if current() and handoff is not None and not state['terminal_failed']:
            handoff.consume_ack(feedback_json(message))

    def route_definition_callback(message):
        handoff = state['handoff']
        if (current() and handoff is not None and not state['terminal_failed'] and
                handoff.accept_commit(feedback_json(message))):
            output('pending', 'native_scan_reference_committed')
            for key in list(splines):
                deliver(key)

    def reference_status_callback(message):
        handoff = state['handoff']
        if (current() and handoff is not None and not state['terminal_failed'] and
                handoff.receive_status(feedback_json(message))):
            output('failed', handoff.failure, native_feedback=handoff.failure_event)

    def poll():
        if measured_voxels and time.monotonic() - state['last_snapshot'] >= 5.0:
            state['last_snapshot'] = time.monotonic()
            tmp = mailbox / 'planner_measured_map.tmp.npz'
            keys = list(measured_voxels)
            np.savez_compressed(tmp, points_xyz_m=np.asarray([measured_voxels[k] for k in keys]),
                point_source_stamp_ns=np.asarray([measured_stamps[k] for k in keys], dtype=np.int64),
                trajectory_body_xyz_m=np.asarray(trajectory),
                lio_pose_stamp_ns=np.asarray(list(lio_sources), dtype=np.int64),
                lio_pose_xyz_xyzw=np.asarray(list(lio_sources.values())),
                provenance_json=json.dumps(dict(run_id=run_id, map_frame=map_frames[0],
                    child_frame=map_frames[1], cloud_topic=CLOUD, pose_topic=LIO,
                    source_interval_ns=source_interval, voxel_m=.08,
                    sampling='latest_actual_measured_point_per_voxel',
                    pairing='exact_integer_nanoseconds', transform='none_already_world')) )
            tmp.replace(mailbox / 'planner_measured_map.npz')
        request = active_request(mailbox, run_id)
        if request != state['request']:
            if state['child']:
                state['child'].close()
            state.update(request=request, child=None, revision=0, reference_stamp=None,
                         route_sent=False, future=None, route=None, last_attempt=0.0, handoff=None,
                         last_snapshot=0.0, terminal_failed=False)
            splines.clear()
            metadata.clear()
            if request is not None:
                output('pending', 'waiting_for_current_measured_map')
        if request is None or state['terminal_failed']:
            return
        vio, paired = state['vio'], state['pair']
        if vio is None or paired is None:
            return
        if (vio.header.frame_id, vio.child_frame_id) != (request['frame_id'], request['child_frame_id']) or map_frames != (request['frame_id'], request['child_frame_id']):
            output('failed', 'post_vio_frame_mismatch')
            return
        if state['route'] is None:
            future = state['future']
            if future is None and time.monotonic() - state['last_attempt'] > 1.0:
                position = vio.pose.pose.position
                state['last_attempt'] = time.monotonic()
                if not measured_voxels:
                    return
                state['future'] = pool.submit(plan_snapshot, np.asarray(list(measured_voxels.values())),
                    np.asarray(trajectory), [position.x, position.y, position.z], request['goal_xyz_m'],
                    dict(cloud_topic=CLOUD, lio_pose_topic=LIO, navigation_topic=VIO,
                         map_frame=map_frames[0], child_frame=map_frames[1],
                         source_interval_ns=list(source_interval), cloud_voxel_m=.08,
                         transform='none_already_world', pairing='exact_integer_nanoseconds',
                         support_offset_kind='configured_mount_assumption_not_measured_floor'))
            elif future is not None and future.done():
                state['future'] = None
                try:
                    result = future.result()
                except Exception as error:
                    output('failed', 'pct_error:' + str(error))
                    return
                if result.get('status') != 'ready':
                    output(result.get('status', 'pending'), result.get('reason', 'pct_pending'))
                    return
                state['route'] = result
                # Diagnostic only: candidate preparation may model the remaining
                # active leg. This artifact never controls or installs a route.
                canonical_run_dir = str(Path(args.run_dir).resolve())
                atomic_json(mailbox / 'planner_global_route.json', dict(
                    version=1, run_id=run_id, seq=request['seq'],
                    frame_id=request['frame_id'], child_frame_id=request['child_frame_id'],
                    canonical_run_dir=canonical_run_dir,
                    canonical_run_path_sha256=hashlib.sha256(canonical_run_dir.encode()).hexdigest(),
                    goal_xyz_m=request['goal_xyz_m'], body_path_xyz_m=result['points_xyz_m'],
                    source_stamp_s=message_stamp_ns(vio) / 1e9,
                    written_wall_unix_s=time.time(), provenance=result.get('provenance', {}),
                    advisory_only=True, artifact_role='active_global_route_diagnostic'))
                state['child'] = OwnedScan(binary, mailbox, request, namespace, args.speed)
                state['handoff'] = ReferenceHandoff(request, state['child'].process.pid,
                    state['child'].ack_nonce, time.monotonic(), args.reference_response_timeout_s)
            return
        child = state['child']
        if child.process.poll() is not None:
            output('failed', 'native_scan_process_exited:%s' % child.process.returncode)
            return
        handoff = state['handoff']
        failure = handoff.check_timeout(time.monotonic())
        if failure:
            output('failed', failure)
            return
        if not state['route_sent']:
            publish_body_pose()
            sensor_pub.publish(paired[1])
            cloud_pub.publish(paired[0])
            if handoff.consumed is None or not path_pub.get_subscription_count():
                return
            path = RosPath()
            path.header.frame_id = request['frame_id']
            stamp = handoff.consumed['source_stamp_ns']
            path.header.stamp.sec, path.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            try:
                points = reference_waypoints(state['route']['points_xyz_m'], handoff.consumed_position)
            except ValueError as error:
                output('failed', str(error))
                return
            for xyz in points:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, xyz - [0, 0, .4])
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            if not path.poses:
                output('failed', 'pct_empty_reference')
                return
            state['reference_stamp'] = [path.header.stamp.sec, path.header.stamp.nanosec]
            handoff.sent_reference(state['reference_stamp'], time.monotonic())
            path_pub.publish(path)
            state['route_sent'] = True
            output('pending', 'native_scan_waiting_for_reference_response')

    def stop_worker(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop_worker)
    subscriptions = [node.create_subscription(PointCloud2, CLOUD, cloud, qos_profile_sensor_data),
        node.create_subscription(Odometry, LIO, lio, qos_profile_sensor_data),
        node.create_subscription(Odometry, VIO, odometry, qos_profile_sensor_data),
        node.create_subscription(Bspline, namespace + '/planning/bspline', spline_callback, 10),
        node.create_subscription(String, namespace + '/planning/trajectory_metadata', metadata_callback, 10),
        node.create_subscription(String, namespace + '/planning/odometry_consumption_ack', odometry_ack_callback, 10),
        node.create_subscription(String, namespace + '/planning/route_definition', route_definition_callback,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(String, namespace + '/planning/reference_status', reference_status_callback, 10)]
    timer = node.create_timer(0.05, poll)
    try:
        rclpy.spin(node)
    finally:
        if state['child']:
            state['child'].close()
        pool.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--mailbox', default='interactive')
    parser.add_argument('--scan-prefix', default='/tmp/dynanav-scan-native/install')
    parser.add_argument('--build-scan', action='store_true')
    parser.add_argument('--build-only', action='store_true')
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--reference-response-timeout-s', type=float, default=10.0,
        help='Wall-time limit for child startup/odometry ACK and reference response only')
    parser.add_argument('--pct-binary', type=Path, default=None)
    parser.add_argument('--body-support-offset', type=float, default=0.32)
    parser.add_argument('--pct-start-blind-radius', type=float, default=0.0,
        help='Explicit opt-in radius for measured-floor start-blind-region inference; 0 keeps strict measured cells')
    args = parser.parse_args(argv)
    if not math.isfinite(args.reference_response_timeout_s) or args.reference_response_timeout_s <= 0:
        parser.error('--reference-response-timeout-s must be finite and positive')
    if args.build_scan:
        build_scan(Path(args.scan_prefix).resolve().parent)
    if not args.build_only:
        try:
            run(args)
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
