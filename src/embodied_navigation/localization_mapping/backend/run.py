"""Opt-in portable worker for the C++ owned-keyframe spool (no ROS required).

Run with PYTHONPATH=src python -m localization.backend.run --help. Heavy Python
registration is a backend reference implementation, not a C++ latency claim.
"""
from __future__ import annotations
import argparse
import json
import shutil
import signal
import time
from pathlib import Path
from .external_map import ExternalMap, read_pcd
from .registration import Relocalizer, RegistrationConfig
from .history import Keyframe
from .worker import BackendWorker, write_json


def decode(directory, max_points=8192):
    directory = Path(directory); row = json.loads((directory/'metadata.json').read_text())
    if row.get('schema') != 'livo_keyframe/v1': raise ValueError('unknown keyframe schema')
    points, _ = read_pcd(directory/'points_lidar.pcd')
    if len(points) != row['point_count'] or len(points) > max_points:
        raise ValueError('spool point count outside configured bound')
    return Keyframe.owned(row['epoch'], row['sequence'], row['stamp_ns'], row['T_odom_body'],
                          row['T_body_lidar'], points[:, :3], row['final_stage'])


def drain_spool_once(spool, worker, counters, max_points=8192):
    # Metadata parsing/cloud allocation is here on a separate process, not the
    # realtime estimator or navigation/control callback.
    for directory in sorted(Path(spool).glob('kf_*')):
        if not directory.is_dir(): continue
        try: snapshot = decode(directory, max_points)
        except (OSError, ValueError, KeyError) as error:
            counters['decode_errors'] += 1
            if sum(1 for p in Path(spool).glob('rejected_kf_*') if p.is_dir()) >= 16:
                # Stop recycling producer slots rather than grow rejected data
                # forever or silently erase reviewable malformed input.
                counters['last_error'] = 'rejected spool bound reached; review inputs before resuming'
                break
            # Keep bad input reviewable, but exclude it from the ready bound.
            directory.rename(directory.with_name('rejected_'+directory.name))
            counters['last_error'] = str(error); continue
        if not worker.try_submit(snapshot): break
        counters['decoded'] += 1
        # Once the bounded worker owns its independent cloud, the producer slot
        # can be recycled. This is best effort: a process crash can lose NEW KF,
        # never an already persisted graph edge. No claims of durable messaging.
        shutil.rmtree(directory)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spool', type=Path, required=True)
    parser.add_argument('--history', type=Path, required=True)
    parser.add_argument('--external-map', type=Path, help='PCD or saved ExternalMap directory; absent=history collection only')
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--initial-world-odom',type=float,nargs=6,help='Explicit configured XYZ/RPY prior, never a GT measurement')
    parser.add_argument('--map-world-offset',type=float,nargs=3,default=[0.,0.,0.])
    parser.add_argument('--queue-capacity', type=int, default=2)
    parser.add_argument('--poll-s', type=float, default=.02)
    parser.add_argument('--maximum-points', type=int, default=8192)
    args = parser.parse_args(argv)
    if args.poll_s <= 0: parser.error('--poll-s must be positive')
    args.spool.mkdir(parents=True, exist_ok=True)
    matcher = None
    if args.external_map is not None:
        asset = (ExternalMap.load(args.external_map) if args.external_map.is_dir()
                 else ExternalMap.from_pcd(args.external_map))
        if args.initial_world_odom is not None:
            from .tracking import PriorTracker
            matcher=PriorTracker(asset,RegistrationConfig())
        else:matcher = Relocalizer(asset, RegistrationConfig())
    import numpy as np
    from scipy.spatial.transform import Rotation
    initial=None
    if args.initial_world_odom is not None:
        initial=np.eye(4);initial[:3,3]=args.initial_world_odom[:3];initial[:3,:3]=Rotation.from_euler('xyz',args.initial_world_odom[3:]).as_matrix()
    world_map=np.eye(4);world_map[:3,3]=args.map_world_offset
    worker = BackendWorker(matcher, args.queue_capacity, args.history, args.result,initial_world_odom=initial,T_world_map=world_map)
    counters = dict(decoded=0, decode_errors=0, last_error=None)
    stopped = False; next_status=0.
    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    try:
        while not stopped:
            drain_spool_once(args.spool, worker, counters, args.maximum_points)
            if time.monotonic()>=next_status:
                write_json(args.result.with_name('backend_status.json'), dict(
                    schema='livo_backend_status/v1', **counters, **worker.stats,
                    backend_alive=worker.thread.is_alive(), published_wall_ns=time.time_ns(),
                    correction_authority=False, realtime_budget_verified=False))
                next_status=time.monotonic()+1.
            time.sleep(args.poll_s)
    finally:
        drained = worker.close(timeout=30.)
        write_json(args.result.with_name('backend_status.json'), dict(
            schema='livo_backend_status/v1', **counters, **worker.stats,
            backend_alive=worker.thread.is_alive(), drained=drained, published_wall_ns=time.time_ns(),
            correction_authority=False, realtime_budget_verified=False))
    return 0 if drained else 2


if __name__ == '__main__': raise SystemExit(main())
