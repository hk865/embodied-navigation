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
import inspect
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

import numpy as np
from scipy.spatial.transform import Rotation

from embodied_navigation.common.geometry import pose_transform
from embodied_navigation.perception.pose_velocity import update_pose_velocity
from embodied_navigation.planning.pct.measured_tomogram import DEFAULT_MAX_GRID_CELLS
from embodied_navigation.planning.scan_handoff import ReferenceHandoff, reference_waypoints
from embodied_navigation.simulation.dynanav.truth_deskew import (
    TRUTH_DESKEWED_CLOUD_TOPIC,
    TRUTH_FRAME_ID,
    TRUTH_LIO_ODOM_TOPIC,
    TRUTH_POSE_SOURCE,
    TRUTH_VIO_ODOM_TOPIC,
)

ROOT = Path(__file__).resolve().parents[4]
CLOUD = '/dynanav/rtx_livo/cloud_registered_full'
LIO = '/dynanav/rtx_livo/aft_mapped_to_init'
VIO = '/dynanav/rtx_livo/post_vio_odometry'
BACKEND_POSE_SOURCE = 'backend_corrected'
BACKEND_VIO = '/dynanav/backend/corrected_odometry'
ALLOWED_POSE_SOURCES = ('livo_post_vio', BACKEND_POSE_SOURCE, 'simulation_truth')


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


def read_yaw_query(mailbox, run_id, expected_pose_source='livo_post_vio'):
    """Read the independent yaw mailbox; never synthesize a translation goal."""
    query = read_json(Path(mailbox) / 'planner_yaw_query.json')
    if not isinstance(query, dict) or query.get('version') != 1 or query.get('run_id') != run_id:
        return None
    if query.get('pose_source') != expected_pose_source or not all(
            isinstance(query.get(key), str) and query[key] for key in ('frame_id', 'child_frame_id')):
        return None
    if any(type(query.get(key)) is not int or query[key] < 1 for key in ('seq', 'query_id', 'body_stamp_ns')):
        return None
    angle = query.get('target_yaw_rad')
    if isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle):
        return None
    return query


def rotation_response_matches(reply, request, child_pid, startup_nonce, last_sample_id):
    """Associate a response with one owned process and increasing map query."""
    if not isinstance(reply, dict) or reply.get('version') != 1:
        return False
    if any(reply.get(key) != request.get(key) for key in ('run_id', 'seq', 'query_id', 'target_yaw_rad')):
        return False
    identity = reply.get('native_identity', {})
    if not isinstance(identity, dict) or identity.get('pid') != child_pid or identity.get('startup_nonce') != startup_nonce:
        return False
    return (type(reply.get('sample_id')) is int and reply['sample_id'] > last_sample_id and
            reply.get('status') in ('clear', 'blocked', 'pending', 'failed'))


def active_request(mailbox, run_id, expected_pose_source='livo_post_vio'):
    """Observation, not a stale request file, owns cancellation/sequence state."""
    observation = read_json(Path(mailbox) / 'observation.json')
    request = read_json(Path(mailbox) / 'planner_request.json')
    if not isinstance(observation, dict):
        return None
    if observation.get('run_id') != run_id:
        return None
    waypoints = observation.get('waypoints', {})
    if waypoints.get('active_action') == 'yaw':
        query = read_yaw_query(mailbox, run_id, expected_pose_source=expected_pose_source)
        if query is None or query['seq'] != waypoints.get('active_seq'):
            return None
        return dict(version=1, action='yaw', **{key: query[key] for key in
            ('run_id', 'seq', 'query_id', 'frame_id', 'child_frame_id', 'pose_source', 'target_yaw_rad')})
    if not isinstance(request, dict) or request.get('run_id') != run_id:
        return None
    if observation.get('waypoints', {}).get('active_seq') != request.get('seq'):
        return None
    if request.get('version') != 1 or request.get('pose_source') != expected_pose_source:
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


def update_measured_voxels(
    measured_voxels,
    measured_stamps,
    xyz,
    stamp_ns,
    *,
    voxel_m=0.08,
    stale_vertical_dt_ns=5_000_000_000,
    stale_vertical_dz_m=0.10,
    local_window_s=0.0,
    local_radius_m=0.0,
    center_xy=None,
):
    """Retain actual measured representatives, evict stale Z-drift ghosts, and optionally enforce a local window.

    When noisy-IMU SLAM drifts vertically by a few centimetres across long climbs
    or revisits, an older scan of the same horizontal surface can remain in an
    adjacent 8 cm Z-voxel (iz +- 1) and stack with newer points into a false
    vertical obstacle (> support_cluster_m). Evicting only immediately adjacent
    Z-neighbors that are both strictly older (>= 5 s) and within <= 0.10 m
    vertically preserves walls (observed simultaneously and taller than 0.10 m)
    and multi-floor slabs while keeping measured surfaces crisp.

    When ``local_window_s > 0`` or ``local_radius_m > 0`` (with ``center_xy``),
    older or distant voxels are also pruned so PCT plans strictly over a local
    sliding map immune to historical drift layering.
    """
    if len(xyz) > 0:
        keys = np.floor(xyz / voxel_m).astype(np.int64)
        scan_voxels = dict(zip(map(tuple, keys.tolist()), xyz))
        measured_voxels.update(scan_voxels)
        measured_stamps.update(dict.fromkeys(scan_voxels, stamp_ns))
        for (ix, iy, iz), point in scan_voxels.items():
            pz = float(point[2])
            for dz_i in (-1, 1):
                neighbor = (ix, iy, iz + dz_i)
                old_stamp = measured_stamps.get(neighbor)
                if old_stamp is not None and (stamp_ns - old_stamp) >= stale_vertical_dt_ns:
                    if abs(pz - float(measured_voxels[neighbor][2])) <= stale_vertical_dz_m:
                        del measured_voxels[neighbor]
                        del measured_stamps[neighbor]
    if local_window_s and local_window_s > 0:
        cutoff_ns = int(stamp_ns - round(float(local_window_s) * 1e9))
        expired = [k for k, ts in measured_stamps.items() if ts < cutoff_ns]
        for k in expired:
            measured_voxels.pop(k, None)
            measured_stamps.pop(k, None)
    if local_radius_m and local_radius_m > 0 and center_xy is not None:
        cx, cy = float(center_xy[0]), float(center_xy[1])
        r2 = float(local_radius_m) ** 2
        distant = [
            k for k, pt in measured_voxels.items()
            if (float(pt[0]) - cx) ** 2 + (float(pt[1]) - cy) ** 2 > r2
        ]
        for k in distant:
            measured_voxels.pop(k, None)
            measured_stamps.pop(k, None)


def select_recent_voxels(measured_voxels, measured_stamps, *, latest_stamp_ns, window_s):
    """Return measured voxel representatives within [latest_stamp_ns - window_s, latest_stamp_ns]."""
    if not measured_voxels or latest_stamp_ns is None or not window_s or window_s <= 0:
        return np.empty((0, 3), dtype=float)
    cutoff_ns = int(latest_stamp_ns - round(float(window_s) * 1e9))
    pts = [pt for key, pt in measured_voxels.items() if measured_stamps.get(key, -1) >= cutoff_ns]
    return np.asarray(pts, dtype=float).reshape(-1, 3) if pts else np.empty((0, 3), dtype=float)


def odom_pose_to_matrix(pose):
    """Convert a geometry_msgs/Pose to a 4x4 SE(3) matrix via shared geometry helper."""
    p, q = pose.position, pose.orientation
    return pose_transform([p.x, p.y, p.z], [q.w, q.x, q.y, q.z])


def apply_matrix_to_pose(pose, transform):
    """Overwrite a geometry_msgs/Pose with transform @ pose_matrix(pose)."""
    updated = np.asarray(transform, dtype=float) @ odom_pose_to_matrix(pose)
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in updated[:3, 3])
    qx, qy, qz, qw = Rotation.from_matrix(updated[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        float(qx), float(qy), float(qz), float(qw)
    )


def compute_backend_correction(backend_pose_mat, raw_vio_pose_mat):
    """Compute C = T_world_odom such that T_world_body = C @ T_odom_body."""
    return np.asarray(backend_pose_mat, dtype=float) @ np.linalg.inv(np.asarray(raw_vio_pose_mat, dtype=float))


def correction_delta_exceeds(delta, *, trans_m=0.05, rot_deg=0.5):
    """Check whether an incremental backend correction merits re-aligning cached voxels."""
    delta = np.asarray(delta, dtype=float)
    if float(np.linalg.norm(delta[:3, 3])) >= trans_m:
        return True
    trace = float(np.clip((np.trace(delta[:3, :3]) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(trace)) >= rot_deg


def apply_rigid_transform_points(xyz, transform):
    """Apply a 4x4 SE(3) rigid transform to an (N, 3) point array."""
    pts = np.asarray(xyz, dtype=float).reshape(-1, 3)
    if len(pts) == 0:
        return pts.copy()
    tf = np.asarray(transform, dtype=float)
    return pts @ tf[:3, :3].T + tf[:3, 3]


def transform_measured_voxels(measured_voxels, measured_stamps, delta, *, voxel_m=0.08):
    """Re-project and re-key all cached measured voxels in place under an incremental SE(3) delta."""
    if not measured_voxels:
        return
    keys = list(measured_voxels.keys())
    pts = np.asarray([measured_voxels[k] for k in keys], dtype=float).reshape(-1, 3)
    stamps = [measured_stamps[k] for k in keys]
    moved = apply_rigid_transform_points(pts, delta)
    new_keys = np.floor(moved / voxel_m).astype(np.int64)
    rebuilt_voxels = {}
    rebuilt_stamps = {}
    for key_arr, pt, ts in zip(new_keys, moved, stamps):
        key = tuple(key_arr.tolist())
        prev_ts = rebuilt_stamps.get(key)
        if prev_ts is None or ts >= prev_ts:
            rebuilt_voxels[key] = pt
            rebuilt_stamps[key] = ts
    measured_voxels.clear()
    measured_voxels.update(rebuilt_voxels)
    measured_stamps.clear()
    measured_stamps.update(rebuilt_stamps)


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
            'fsm.planning_horizon': 2.5, 'manager.planning_horizon': 2.5,
            'fsm.thresh_replan': 0.8, 'grid_map.frame_id': request['frame_id'],
            'grid_map.sliding_map_frame_id': namespace.strip('/').replace('/', '_') + '_grid',
            'grid_map.body_height': 0.4, 'grid_map.resolution': 0.05,
            'grid_map.double_cylinder_radius': 0.19, 'grid_map.double_cylinder_offset': 0.14,
            'grid_map.obstacles_inflation_z_up': 0.10, 'grid_map.obstacles_inflation_z_down': 0.15,
            'grid_map.cloud_is_world': True, 'grid_map.need_extrinsic': False,
            'manager.max_vel': speed, 'optimization.max_vel': speed,
            'manager.max_acc': 2.0, 'optimization.max_acc': 2.0,
            'optimization.acc_tolerance': 2.0,
            'diagnostics.voxel_trace_path': str(Path(mailbox) / ('scan_voxel_%s' % request['seq'])),
            'diagnostics.voxel_trace_max_events': 8,
            'diagnostics.voxel_trace_max_bytes': 2 * 1024 * 1024,
            'diagnostics.voxel_trace_period_s': 1.0,
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


class PctPlanCoordinator:
    """Single-worker latest-wins PCT planning coordinator with active cancellation."""

    def __init__(self, plan_fn, *, max_workers: int = 1, retry_interval_s: float = 1.0):
        self._plan_fn = plan_fn
        self._accepts_cancel = "cancel_event" in inspect.signature(plan_fn).parameters
        self._pool = ThreadPoolExecutor(max_workers=max_workers)
        self._retry_interval_s = float(retry_interval_s)
        self.active_seq = None
        self.active_future = None
        self.active_cancel = None
        self.pending_job = None
        self.last_attempt = 0.0

    def on_request_changed(self, new_seq) -> None:
        """Abort obsolete active work and drop superseded pending goals."""
        if self.active_cancel is not None and self.active_seq != new_seq:
            self.active_cancel.set()
        if self.active_future is not None and self.active_seq != new_seq:
            self.active_future.cancel()
        if self.pending_job is not None and self.pending_job[0] != new_seq:
            self.pending_job = None
        self.last_attempt = 0.0

    def _launch(self, seq, args, kwargs, now: float) -> None:
        cancel_event = threading.Event()
        self.active_seq = seq
        self.active_cancel = cancel_event
        self.last_attempt = now
        self.pending_job = None
        call_kwargs = dict(kwargs)
        if self._accepts_cancel:
            call_kwargs["cancel_event"] = cancel_event
        self.active_future = self._pool.submit(self._plan_fn, *args, **call_kwargs)

    def submit(self, seq: int, *args, now_mono: float | None = None, **kwargs) -> bool:
        """Submit planning for seq, or overwrite latest pending goal while active aborts."""
        now = time.monotonic() if now_mono is None else float(now_mono)
        if self.active_future is not None:
            if not self.active_future.done():
                if self.active_seq != seq and self.active_cancel is not None:
                    self.active_cancel.set()
                self.pending_job = (seq, args, kwargs)
                return False
            return False
        if self.last_attempt > 0.0 and (now - self.last_attempt) <= self._retry_interval_s:
            return False
        self._launch(seq, args, kwargs, now)
        return True

    def poll(self, current_seq, *, now_mono: float | None = None):
        """Return finished future only if it belongs to current_seq and was not cancelled."""
        if self.active_future is None or not self.active_future.done():
            return None
        fut = self.active_future
        job_seq = self.active_seq
        was_cancelled = bool(self.active_cancel is not None and self.active_cancel.is_set())
        self.active_future = None
        self.active_seq = None
        self.active_cancel = None

        pending = self.pending_job
        self.pending_job = None
        if pending is not None and pending[0] == current_seq:
            now = time.monotonic() if now_mono is None else float(now_mono)
            self._launch(pending[0], pending[1], pending[2], now)
            return None

        if current_seq is None or job_seq != current_seq or was_cancelled:
            return None
        return fut

    def shutdown(self, wait: bool = True) -> None:
        if self.active_cancel is not None:
            self.active_cancel.set()
        if self.active_future is not None:
            self.active_future.cancel()
        self.pending_job = None
        self._pool.shutdown(wait=wait, cancel_futures=True)


def run(args, *, pct_max_grid_cells=DEFAULT_MAX_GRID_CELLS):
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
    pose_source = getattr(args, 'pose_source', 'livo_post_vio')
    cloud_topic = getattr(args, 'cloud_topic', CLOUD)
    lio_topic = getattr(args, 'lio_topic', LIO)
    vio_topic = getattr(args, 'vio_topic', VIO)
    raw_vio_topic = getattr(args, 'raw_vio_topic', VIO)
    local_map_window_s = float(getattr(args, 'local_map_window_s', 0.0) or 0.0)
    local_map_radius_m = float(getattr(args, 'local_map_radius_m', 0.0) or 0.0)
    local_recovery_window_s = float(getattr(args, 'local_recovery_window_s', 30.0) or 0.0)
    is_backend_pose = pose_source == BACKEND_POSE_SOURCE
    if pose_source not in ALLOWED_POSE_SOURCES:
        raise ValueError(f'Unsupported pose_source: {pose_source!r}')
    if pose_source == TRUTH_POSE_SOURCE and (
            cloud_topic != TRUTH_DESKEWED_CLOUD_TOPIC
            or lio_topic != TRUTH_LIO_ODOM_TOPIC
            or vio_topic != TRUTH_VIO_ODOM_TOPIC):
        raise ValueError(
            'simulation_truth mode requires exact truth topics: '
            f'cloud={TRUTH_DESKEWED_CLOUD_TOPIC}, lio={TRUTH_LIO_ODOM_TOPIC}, vio={TRUTH_VIO_ODOM_TOPIC}'
        )
    if pose_source != TRUTH_POSE_SOURCE and any(
            t.startswith('/dynanav/truth/') for t in (cloud_topic, lio_topic, vio_topic, raw_vio_topic)):
        raise ValueError(f'{pose_source} mode must not consume /dynanav/truth/* topics')
    rclpy.init()
    node = Node('dynanav_pct_scan_worker')
    velocity_policy = dict(
        source=('simulation_truth_pose_window' if pose_source == TRUTH_POSE_SOURCE
                else 'backend_corrected_pose_window' if is_backend_pose
                else 'post_vio_pose_window'),
        window_s=0.5, reset_gap_s=0.25,
        frame=(TRUTH_FRAME_ID if pose_source == TRUTH_POSE_SOURCE
               else 'backend_map' if is_backend_pose
               else 'estimator_map'),
        input_topic=vio_topic,
        raw_odometry_twist_used=False)
    parameters = dict(
        reference_response_timeout_s=args.reference_response_timeout_s,
        pose_source=pose_source, cloud_topic=cloud_topic,
        lio_topic=lio_topic, vio_topic=vio_topic,
        raw_vio_topic=(raw_vio_topic if is_backend_pose else None),
        local_map_window_s=local_map_window_s,
        local_map_radius_m=local_map_radius_m,
        local_recovery_window_s=local_recovery_window_s,
        body_velocity=velocity_policy, speed=args.speed,
        pct_max_grid_cells=pct_max_grid_cells)
    atomic_json(mailbox / 'planner_worker_parameters.json', parameters)
    velocity_history = deque()
    # All accumulations are process-local and begin empty for this run.
    measured_voxels = {}
    measured_stamps = {}
    lio_sources = {}
    trajectory = []
    map_frames = None
    source_interval = [None, None]
    backend_state = dict(
        correction=np.eye(4, dtype=float),
        applied_correction=np.eye(4, dtype=float),
        frame_id=None,
        ready=not is_backend_pose,
        backend_msgs=OrderedDict(),
        raw_vio_msgs=OrderedDict(),
    )

    def plan_snapshot(points, track, start, goal, source, recent_points=None, *, cancel_event=None):
        if cancel_event is not None and cancel_event.is_set():
            return dict(status='cancelled', points_xyz_m=[], reason='cancelled', provenance={})
        planner = MeasuredPctPlanner(points, trajectory_body_xyz=track,
            body_support_offset_m=args.body_support_offset, start_blind_radius_m=args.pct_start_blind_radius, native_binary=args.pct_binary,
            max_grid_cells=parameters['pct_max_grid_cells'],
            generation=run_id, source_metadata=source)
        plan_kwargs = {'cancel_event': cancel_event} if 'cancel_event' in inspect.signature(planner.plan).parameters else {}
        result = planner.plan(start, goal, **plan_kwargs)
        if cancel_event is not None and cancel_event.is_set():
            return dict(status='cancelled', points_xyz_m=[], reason='cancelled', provenance=result.get('provenance', {}))
        if (not result['found'] and result.get('reason') == 'start_support_unobserved'
                and recent_points is not None and len(recent_points) > 0 and len(recent_points) < len(points)):
            local_source = dict(source, local_recovery_window_s=local_recovery_window_s,
                                local_recovery_point_count=int(len(recent_points)))
            local_planner = MeasuredPctPlanner(recent_points, trajectory_body_xyz=track,
                body_support_offset_m=args.body_support_offset, start_blind_radius_m=args.pct_start_blind_radius,
                native_binary=args.pct_binary, max_grid_cells=parameters['pct_max_grid_cells'],
                generation=run_id, source_metadata=local_source)
            local_kwargs = {'cancel_event': cancel_event} if 'cancel_event' in inspect.signature(local_planner.plan).parameters else {}
            local_result = local_planner.plan(start, goal, **local_kwargs)
            if cancel_event is not None and cancel_event.is_set():
                return dict(status='cancelled', points_xyz_m=[], reason='cancelled', provenance=local_result.get('provenance', {}))
            if local_result['found']:
                result = local_result
        return dict(status=('ready' if result['found'] else
                    'pending' if result.get('reason') == 'no_measured_support' else 'failed'),
                    points_xyz_m=result['body_path_xyz'], reason=result.get('reason'),
                    provenance=result['provenance'])
    plan_coordinator = PctPlanCoordinator(plan_snapshot, max_workers=1, retry_interval_s=1.0)
    snapshot_pool = ThreadPoolExecutor(max_workers=1)
    state = dict(request=None, child=None, revision=0, vio=None, pair=None, reference_stamp=None,
                 route_sent=False, future=None, snapshot_future=None, route=None, last_attempt=0.0,
                 handoff=None, last_snapshot=0.0, terminal_failed=False, body_pose=None,
                 yaw_sample_id=0, yaw_last_response_id=0, yaw_last_response_wall=None,
                 yaw_started_wall=None, yaw_last_cloud_sent=None,
                 yaw_initial_cloud_ns=None, yaw_has_clear=False)
    clouds, poses, splines, metadata = OrderedDict(), OrderedDict(), {}, {}
    namespace = '/dynanav/pct_scan/worker_%s' % os.getpid()
    sensor_pub = node.create_publisher(Odometry, namespace + '/sensor_pose', 10)
    body_pub = node.create_publisher(Odometry, namespace + '/body_pose', 10)
    cloud_pub = node.create_publisher(PointCloud2, namespace + '/cloud', 10)
    path_pub = node.create_publisher(RosPath, namespace + '/initial_path', 1)
    rotation_pub = node.create_publisher(String, namespace + '/rotation_query', 10)

    def current():
        request = active_request(mailbox, run_id, expected_pose_source=pose_source)
        return request is not None and state['request'] is not None and request == state['request']

    def yaw_output(status, reason=None, **extra):
        request = state['request']
        if not current() or request is None or request.get('action') != 'yaw':
            return
        data = dict(version=1, run_id=run_id, seq=request['seq'], query_id=request['query_id'],
                    target_yaw_rad=request['target_yaw_rad'], frame_id=request['frame_id'],
                    child_frame_id=request['child_frame_id'], status=status, reason=reason,
                    sample_id=state['yaw_last_response_id'], received_wall_s=time.monotonic())
        data.update(extra)
        data['received_wall_s'] = time.monotonic()
        atomic_json(mailbox / 'planner_yaw_space.json', data)
        if status in ('failed', 'blocked'):
            state['terminal_failed'] = True

    def output(status, reason=None, **extra):
        if state['request'] is not None and state['request'].get('action') == 'yaw':
            yaw_output(status, reason, **extra)
            return
        if not current() or state['terminal_failed']:
            return
        if status == 'failed':
            state['terminal_failed'] = True
            if state['handoff'] is not None:
                state['handoff'].fail(reason)
        request = state['request']
        state['revision'] += 1
        data = dict(version=1, run_id=run_id, seq=request['seq'], frame_id=request['frame_id'],
                    child_frame_id=request['child_frame_id'], pose_source=pose_source, status=status,
                    revision=state['revision'], source_stamp_s=(message_stamp_ns(state['vio']) / 1e9
                    if state['vio'] is not None else None), points_xyz_m=[], **extra)
        if reason:
            data['reason'] = reason
        atomic_json(mailbox / 'planner_path.json', data)

    def publish_body_pose():
        message, handoff = state['body_pose'], state['handoff']
        if message is None or state['terminal_failed']:
            return
        request = state['request']
        if request is not None and request.get('action') == 'yaw':
            if state['child'] is not None and (message.header.frame_id, message.child_frame_id) == (request['frame_id'], request['child_frame_id']):
                body_pub.publish(message)
            return
        if handoff is None:
            return
        if (message.header.frame_id, message.child_frame_id) != (request['frame_id'], request['child_frame_id']):
            return
        p = message.pose.pose.position
        handoff.sent_body_pose(message_stamp_ns(message), [p.x, p.y, p.z])
        body_pub.publish(message)

    def send_yaw_query():
        request, paired, child, body = state['request'], state['pair'], state['child'], state['body_pose']
        if (request is None or request.get('action') != 'yaw' or state['terminal_failed'] or
                child is None or paired is None or body is None or not rotation_pub.get_subscription_count()):
            return
        if (body.header.frame_id, body.child_frame_id) != (request['frame_id'], request['child_frame_id']):
            return
        body_ns = message_stamp_ns(body)
        cloud_ns = message_stamp_ns(paired[0])
        if not state['yaw_has_clear']:
            state['yaw_initial_cloud_ns'] = max(state['yaw_initial_cloud_ns'] or 0, cloud_ns)
        # Before the first clear reply, require the current cloud to fuse. Once
        # running, require the fused map to stay within 0.2 s of body_pose so
        # in-flight raycasting does not stall body_pose clearance while still
        # failing closed if LiDAR or GridMap fusion stops.
        min_cloud_ns = (
            max(state['yaw_initial_cloud_ns'], body_ns - 200_000_000)
            if state['yaw_has_clear'] else cloud_ns
        )
        state['yaw_sample_id'] += 1
        query = dict(version=1, run_id=run_id, seq=request['seq'], query_id=request['query_id'],
            sample_id=state['yaw_sample_id'], startup_nonce=child.ack_nonce,
            frame_id=request['frame_id'], child_frame_id=request['child_frame_id'],
            target_yaw_rad=request['target_yaw_rad'], body_stamp_ns=body_ns,
            minimum_cloud_stamp_ns=min_cloud_ns)
        rotation_pub.publish(String(data=json.dumps(query, allow_nan=False)))

    def update_backend_correction():
        if not is_backend_pose:
            return
        b_msgs, r_msgs = backend_state['backend_msgs'], backend_state['raw_vio_msgs']
        for stamp in sorted(set(b_msgs).intersection(r_msgs)):
            b_msg, r_msg = b_msgs.pop(stamp), r_msgs.pop(stamp)
            C_new = compute_backend_correction(
                odom_pose_to_matrix(b_msg.pose.pose),
                odom_pose_to_matrix(r_msg.pose.pose),
            )
            if not np.isfinite(C_new).all():
                continue
            backend_state['frame_id'] = b_msg.header.frame_id
            backend_state['correction'] = C_new
            if not backend_state['ready']:
                backend_state['applied_correction'] = C_new
                backend_state['ready'] = True
            else:
                delta = C_new @ np.linalg.inv(backend_state['applied_correction'])
                if correction_delta_exceeds(delta):
                    transform_measured_voxels(measured_voxels, measured_stamps, delta)
                    if trajectory:
                        moved_track = apply_rigid_transform_points(np.asarray(trajectory, dtype=float), delta)
                        trajectory[:] = moved_track.tolist()
                    backend_state['applied_correction'] = C_new
        for cache in (b_msgs, r_msgs):
            while len(cache) > 128:
                cache.popitem(last=False)

    def raw_vio_odometry(message):
        stamp = message_stamp_ns(message)
        backend_state['raw_vio_msgs'][stamp] = message
        update_backend_correction()
        pair()

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
        if is_backend_pose:
            backend_state['backend_msgs'][stamp] = message
            update_backend_correction()
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
            send_yaw_query()
        if is_backend_pose:
            pair()

    def publish_yaw_pair(paired):
        cloud, pose = paired
        stamp = message_stamp_ns(cloud)
        if (stamp == state['yaw_last_cloud_sent'] or not sensor_pub.get_subscription_count() or
                not cloud_pub.get_subscription_count()):
            return
        sensor_pub.publish(pose)
        cloud_pub.publish(cloud)
        state['yaw_last_cloud_sent'] = stamp

    def build_transformed_cloud(cloud_msg, xyz_new, curvature, new_frame_id):
        from sensor_msgs.msg import PointField
        header = deepcopy(cloud_msg.header)
        header.frame_id = new_frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='curvature', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        packed = np.column_stack([xyz_new.astype(np.float32), np.asarray(curvature, dtype=np.float32)])
        return point_cloud2.create_cloud(header, fields, packed)

    def pair():
        nonlocal map_frames
        if is_backend_pose and not backend_state['ready']:
            return
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
                if is_backend_pose:
                    C = backend_state['applied_correction']
                    target_frame = backend_state['frame_id']
                    xyz = apply_rigid_transform_points(xyz, C)
                    pose = deepcopy(pose)
                    apply_matrix_to_pose(pose.pose.pose, C)
                    pose.header.frame_id = target_frame
                    cloud = build_transformed_cloud(cloud, xyz, raw['curvature'], target_frame)
                frames = (pose.header.frame_id, pose.child_frame_id)
                if map_frames is not None and frames != map_frames:
                    raise ValueError('measured_cloud_generation_frame_change')
                map_frames = frames
                if source_interval[1] is not None and stamp <= source_interval[1]:
                    continue
                source_interval[0] = stamp if source_interval[0] is None else source_interval[0]
                source_interval[1] = stamp
                state['pair'] = (cloud, pose)
                if state['child'] is not None:
                    # Native GridMap is latest-pose based, not message-filter synchronized.
                    # No claim of child-side exact-stamp consumption is made.
                    if state['request'].get('action') == 'yaw':
                        publish_yaw_pair((cloud, pose))
                    else:
                        sensor_pub.publish(pose)
                        cloud_pub.publish(cloud)
                # Retain actual measured representatives, evict stale Z-drift ghosts,
                # and enforce optional local sliding window / radius pruning.
                position, orientation = pose.pose.pose.position, pose.pose.pose.orientation
                update_measured_voxels(
                    measured_voxels, measured_stamps, xyz, stamp,
                    local_window_s=local_map_window_s,
                    local_radius_m=local_map_radius_m,
                    center_xy=(position.x, position.y),
                )
                lio_sources[stamp] = [position.x, position.y, position.z,
                                      orientation.x, orientation.y, orientation.z, orientation.w]
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
            native_metadata=info, map_source=cloud_topic, navigation_source=vio_topic,
            body_velocity=velocity_policy, child_odometry_ack=handoff.reference_odometry_ack,
            reference_commit=handoff.commit,
            local_planner='native_SCAN', global_planner='native_PCT_Astar',
            occupancy='native_3d_inflated_voxels', z_policy='native_global_reference_z_profile_zero_z_gradient',
            source_stamp_semantics='latest_post_vio_received_by_adapter_at_spline_delivery',
            sensor_pairing='adapter_exact_stamp_native_latest_pose', pct=state['route'].get('provenance', {}))
        request = state['request']
        atomic_json(mailbox / 'planner_path.json', dict(version=1, run_id=run_id, seq=request['seq'],
            frame_id=request['frame_id'], child_frame_id=request['child_frame_id'], pose_source=pose_source,
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

    def rotation_result_callback(message):
        request, child = state['request'], state['child']
        if (not current() or request is None or request.get('action') != 'yaw' or
                child is None or state['terminal_failed']):
            return
        reply = feedback_json(message)
        if not rotation_response_matches(reply, request, child.process.pid, child.ack_nonce,
                                         state['yaw_last_response_id']):
            return
        state['yaw_last_response_id'] = reply['sample_id']
        state['yaw_last_response_wall'] = time.monotonic()
        if reply.get('status') == 'clear':
            state['yaw_has_clear'] = True
        yaw_output(reply['status'], reply.get('reason'), **{key: value for key, value in reply.items()
            if key not in ('status', 'reason')})

    def poll_yaw():
        request, paired = state['request'], state['pair']
        if state['child'] is None:
            state['child'] = OwnedScan(binary, mailbox, request, namespace, args.speed)
            state['yaw_started_wall'] = time.monotonic()
        child = state['child']
        if child.process.poll() is not None:
            yaw_output('failed', 'native_scan_process_exited:%s' % child.process.returncode)
            return
        publish_body_pose()
        publish_yaw_pair(paired)
        now = time.monotonic()
        last = state['yaw_last_response_wall'] or state['yaw_started_wall']
        if now - last > args.reference_response_timeout_s:
            yaw_output('failed', 'native_rotation_response_timeout')
            return
        send_yaw_query()

    def poll():
        if measured_voxels and time.monotonic() - state['last_snapshot'] >= 5.0:
            snap_future = state['snapshot_future']
            if snap_future is None or snap_future.done():
                if snap_future is not None:
                    snap_exc = snap_future.exception()
                    if snap_exc is not None:
                        node.get_logger().warning(f'measured map snapshot failed: {snap_exc}')
                state['last_snapshot'] = time.monotonic()
                pts = np.asarray(list(measured_voxels.values()))
                stamps = np.asarray(list(measured_stamps.values()), dtype=np.int64)
                traj = np.asarray(trajectory)
                lio_stamps = np.asarray(list(lio_sources), dtype=np.int64)
                lio_poses = np.asarray(list(lio_sources.values()))
                prov = json.dumps(dict(run_id=run_id, pose_source=pose_source,
                    map_frame=map_frames[0], child_frame=map_frames[1],
                    cloud_topic=cloud_topic, pose_topic=lio_topic,
                    source_interval_ns=list(source_interval), voxel_m=.08,
                    local_map_window_s=local_map_window_s,
                    local_map_radius_m=local_map_radius_m,
                    sampling='latest_actual_measured_point_per_voxel',
                    pairing='exact_integer_nanoseconds',
                    transform=('backend_rigid_correction' if is_backend_pose else 'none_already_world')))
                def write_snapshot(pts=pts, stamps=stamps, traj=traj,
                                   lio_stamps=lio_stamps, lio_poses=lio_poses, prov=prov):
                    tmp = mailbox / 'planner_measured_map.tmp.npz'
                    np.savez_compressed(tmp, points_xyz_m=pts,
                        point_source_stamp_ns=stamps,
                        trajectory_body_xyz_m=traj,
                        lio_pose_stamp_ns=lio_stamps,
                        lio_pose_xyz_xyzw=lio_poses,
                        provenance_json=prov)
                    tmp.replace(mailbox / 'planner_measured_map.npz')
                state['snapshot_future'] = snapshot_pool.submit(write_snapshot)
        request = active_request(mailbox, run_id, expected_pose_source=pose_source)
        if request != state['request']:
            if state['child']:
                state['child'].close()
            plan_coordinator.on_request_changed(request['seq'] if request is not None else None)
            state.update(request=request, child=None, revision=0, reference_stamp=None,
                         route_sent=False, future=None, route=None, last_attempt=0.0, handoff=None,
                         last_snapshot=0.0, terminal_failed=False,
                         yaw_sample_id=0, yaw_last_response_id=0, yaw_last_response_wall=None,
                         yaw_started_wall=None, yaw_last_cloud_sent=None,
                         yaw_initial_cloud_ns=None, yaw_has_clear=False)
            splines.clear()
            metadata.clear()
            if request is not None:
                output('pending', 'waiting_for_current_measured_map')
        if request is None or state['terminal_failed']:
            plan_coordinator.poll(None)
            return
        vio, paired = state['vio'], state['pair']
        if vio is None or paired is None:
            return
        if (vio.header.frame_id, vio.child_frame_id) != (request['frame_id'], request['child_frame_id']) or map_frames != (request['frame_id'], request['child_frame_id']):
            output('failed', 'post_vio_frame_mismatch')
            return
        if request.get('action') == 'yaw':
            plan_coordinator.poll(None)
            poll_yaw()
            return
        if state['route'] is None:
            done_future = plan_coordinator.poll(request['seq'])
            if done_future is None and measured_voxels:
                if plan_coordinator.active_future is None or plan_coordinator.active_seq != request['seq']:
                    position = vio.pose.pose.position
                    recent_points = (
                        select_recent_voxels(
                            measured_voxels, measured_stamps,
                            latest_stamp_ns=source_interval[1],
                            window_s=local_recovery_window_s,
                        )
                        if local_recovery_window_s > 0 else None
                    )
                    plan_coordinator.submit(
                        request['seq'],
                        np.asarray(list(measured_voxels.values())),
                        np.asarray(trajectory),
                        [position.x, position.y, position.z],
                        request['goal_xyz_m'],
                        dict(cloud_topic=cloud_topic, lio_pose_topic=lio_topic, navigation_topic=vio_topic,
                             map_frame=map_frames[0], child_frame=map_frames[1],
                             source_interval_ns=list(source_interval), cloud_voxel_m=.08,
                             local_map_window_s=local_map_window_s, local_map_radius_m=local_map_radius_m,
                             transform=('backend_rigid_correction' if is_backend_pose else 'none_already_world'),
                             pairing='exact_integer_nanoseconds',
                             support_offset_kind='configured_mount_assumption_not_measured_floor'),
                        recent_points,
                    )
            elif done_future is not None:
                try:
                    result = done_future.result()
                except Exception as error:
                    output('failed', 'pct_error:' + str(error))
                    return
                if result.get('status') == 'cancelled':
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
    subscriptions = [node.create_subscription(PointCloud2, cloud_topic, cloud, qos_profile_sensor_data),
        node.create_subscription(Odometry, lio_topic, lio, qos_profile_sensor_data),
        node.create_subscription(Odometry, vio_topic, odometry, qos_profile_sensor_data),
        node.create_subscription(Bspline, namespace + '/planning/bspline', spline_callback, 10),
        node.create_subscription(String, namespace + '/planning/trajectory_metadata', metadata_callback, 10),
        node.create_subscription(String, namespace + '/planning/odometry_consumption_ack', odometry_ack_callback, 10),
        node.create_subscription(String, namespace + '/planning/route_definition', route_definition_callback,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(String, namespace + '/planning/reference_status', reference_status_callback, 10),
        node.create_subscription(String, namespace + '/rotation_result', rotation_result_callback, 10)]
    if is_backend_pose:
        subscriptions.append(
            node.create_subscription(Odometry, raw_vio_topic, raw_vio_odometry, qos_profile_sensor_data)
        )
    timer = node.create_timer(0.05, poll)
    try:
        rclpy.spin(node)
    finally:
        if state['child']:
            state['child'].close()
        plan_coordinator.shutdown(wait=True)
        snapshot_pool.shutdown(wait=True, cancel_futures=True)
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
    parser.add_argument('--pct-max-grid-cells', type=int, default=DEFAULT_MAX_GRID_CELLS,
        help='Positive dense multilayer allocation budget; does not preallocate or filter the measured map')
    parser.add_argument('--body-support-offset', type=float, default=0.32)
    parser.add_argument('--pct-start-blind-radius', type=float, default=0.0,
        help='Explicit opt-in radius for measured-floor start-blind-region inference; 0 keeps strict measured cells')
    parser.add_argument('--pose-source', choices=ALLOWED_POSE_SOURCES, default='livo_post_vio',
        help='Pose source for navigation requests and yaw queries (livo_post_vio, backend_corrected, or simulation_truth)')
    parser.add_argument('--cloud-topic', default=CLOUD,
        help='World-frame deskewed PointCloud2 topic')
    parser.add_argument('--lio-topic', default=LIO,
        help='Same-stage Odometry topic paired with the deskewed cloud')
    parser.add_argument('--vio-topic', default=VIO,
        help='High-rate Odometry topic driving pose-window body pose/velocity')
    parser.add_argument('--raw-vio-topic', default=VIO,
        help='Raw frontend Odometry topic paired with backend corrected odometry to derive C = T_world_odom')
    parser.add_argument('--local-map-window-s', type=float, default=0.0,
        help='Optional sliding time window in seconds for measured PCT voxels; 0 retains full run history')
    parser.add_argument('--local-map-radius-m', type=float, default=0.0,
        help='Optional horizontal radius in metres around robot for measured PCT voxels; 0 disables radius pruning')
    parser.add_argument('--local-recovery-window-s', type=float, default=30.0,
        help='Recent-scan window in seconds used to recover when full history reports start_support_unobserved; 0 disables')
    args = parser.parse_args(argv)
    if args.pct_max_grid_cells <= 0:
        parser.error('--pct-max-grid-cells must be a positive integer')
    if not math.isfinite(args.reference_response_timeout_s) or args.reference_response_timeout_s <= 0:
        parser.error('--reference-response-timeout-s must be finite and positive')
    for flag, value in (('--local-map-window-s', args.local_map_window_s),
                        ('--local-map-radius-m', args.local_map_radius_m),
                        ('--local-recovery-window-s', args.local_recovery_window_s)):
        if not math.isfinite(value) or value < 0:
            parser.error(f'{flag} must be finite and non-negative')
    if args.pose_source == TRUTH_POSE_SOURCE and (
            args.cloud_topic != TRUTH_DESKEWED_CLOUD_TOPIC
            or args.lio_topic != TRUTH_LIO_ODOM_TOPIC
            or args.vio_topic != TRUTH_VIO_ODOM_TOPIC):
        parser.error(
            'simulation_truth --pose-source requires exact truth topics: '
            f'--cloud-topic {TRUTH_DESKEWED_CLOUD_TOPIC} '
            f'--lio-topic {TRUTH_LIO_ODOM_TOPIC} '
            f'--vio-topic {TRUTH_VIO_ODOM_TOPIC}'
        )
    if args.pose_source != TRUTH_POSE_SOURCE and any(
            t.startswith('/dynanav/truth/') for t in (args.cloud_topic, args.lio_topic, args.vio_topic, args.raw_vio_topic)):
        parser.error(f'{args.pose_source} --pose-source must not subscribe to /dynanav/truth/* topics')
    if args.build_scan:
        build_scan(Path(args.scan_prefix).resolve().parent)
    if not args.build_only:
        try:
            run(args, pct_max_grid_cells=args.pct_max_grid_cells)
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
