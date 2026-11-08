"""Bounded best-effort backend: own a snapshot before submitting it.

No registration, map I/O, graph optimization or cloud serialization runs in
try_submit. Queue-full/contention drops NEW candidate keyframes, not already
accepted graph edges. The frontend odometry and control never await a result.
"""
from __future__ import annotations
import json
import threading
import time
from collections import deque
from pathlib import Path
from .correction import TemporalAdmission
from .history import History
import numpy as np
from .geometry import inverse


def write_json(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, allow_nan=False) + '\n'); temporary.replace(path)


class BackendWorker:
    def __init__(self, relocalizer=None, capacity=2, history_directory=None, result_path=None,
                 admission=None, T_world_map=None, maximum_history_keyframes=2000,
                 maximum_keyframe_points=8192, initial_world_odom=None, auto_loops=True):
        if min(capacity, maximum_history_keyframes, maximum_keyframe_points) < 1:
            raise ValueError('positive queue/history/point bounds required')
        self.relocalizer = relocalizer; self.capacity = capacity
        self.history_directory = None if history_directory is None else Path(history_directory)
        self.result_path = result_path; self.admission = admission or TemporalAdmission()
        self.T_world_map = T_world_map
        self.world_odom_prior=None if initial_world_odom is None else np.asarray(initial_world_odom,dtype=float)
        self.auto_loops=auto_loops
        self.path_lengths=[];self.pending_loop=None;self.last_loop_position=None
        self.maximum_history_keyframes = maximum_history_keyframes
        self.maximum_keyframe_points = maximum_keyframe_points
        self.history = History(); self.lock = threading.Lock(); self.ready = threading.Event()
        self.jobs = deque(); self.result = None; self.closed = False
        self.active_epoch = None; self.latest_stamp_ns = 0
        self.stats = dict(submitted=0, dropped_full=0, dropped_contention=0, dropped_closed=0,
                          processed=0, errors=0, discarded_epoch=0, discarded_size=0, queue_high_water=0,
                          registration_wall_s=0., worker_cpu_s=0., last_worker_error=None,
                          loop_candidates=0, verified_loops=0, optimized_graphs=0,last_loop_reason=None,loop_registration_wall_s=0.)
        self.thread = threading.Thread(target=self._run, name='livo-backend', daemon=True); self.thread.start()

    def try_submit(self, keyframe):
        # Ownership is supplied by Keyframe.owned/spool decoder, not borrowed
        # frontend arrays. The bounded queue itself performs no point traversal.
        if not self.lock.acquire(False):
            self.stats['dropped_contention'] += 1; return False
        try:
            if self.closed: self.stats['dropped_closed'] += 1; return False
            if len(keyframe.points_lidar) > self.maximum_keyframe_points:
                self.stats['discarded_size'] += 1; return False
            if self.active_epoch is None: self.active_epoch = keyframe.epoch
            if keyframe.epoch != self.active_epoch:
                self.stats['discarded_epoch'] += 1; return False
            self.latest_stamp_ns = max(self.latest_stamp_ns, keyframe.stamp_ns)
            if len(self.jobs) >= self.capacity:
                self.stats['dropped_full'] += 1; return False
            self.jobs.append((keyframe,time.monotonic())); self.stats['submitted'] += 1
            self.stats['queue_high_water'] = max(self.stats['queue_high_water'], len(self.jobs))
            self.ready.set(); return True
        finally: self.lock.release()

    def poll(self):
        with self.lock:
            result, self.result = self.result, None
            return result

    def _run(self):
        while True:
            self.ready.wait()
            with self.lock:
                if self.jobs: keyframe,queued_at = self.jobs.popleft()
                elif self.closed: return
                else: self.ready.clear(); continue
            cpu = time.thread_time(); started = time.monotonic()
            try:
                if len(self.history.keyframes) >= self.maximum_history_keyframes:
                    raise ValueError('resident history bound reached; start a new segment or archive before continuing')
                travel=0. if not self.history.keyframes else float(np.linalg.norm(keyframe.T_odom_body[:3,3]-self.history.keyframes[-1].T_odom_body[:3,3]))
                self.history.add(keyframe)
                self.path_lengths.append((self.path_lengths[-1]if self.path_lengths else 0.)+travel)
                if self.relocalizer is None:
                    result = dict(schema='livo_backend_correction/v1', accepted=False,
                                  epoch=keyframe.epoch, sequence=keyframe.sequence, source_ns=keyframe.stamp_ns,
                                  reason='history_collected_no_global_matcher')
                else:
                    if self.world_odom_prior is None:
                        registration = self.relocalizer.localize(keyframe.points_lidar)
                    else:
                        world_map=np.eye(4)if self.T_world_map is None else self.T_world_map
                        prior=inverse(world_map)@self.world_odom_prior@keyframe.T_odom_body@keyframe.T_body_lidar
                        registration = self.relocalizer.localize(keyframe.points_lidar,T_prior=prior)
                    self.stats['registration_wall_s'] += registration.elapsed_s
                    with self.lock: latest = self.latest_stamp_ns
                    result = self.admission.consider(keyframe, registration, latest, self.T_world_map)
                    if result.get('accepted'):self.world_odom_prior=np.asarray(result['C_world_odom'])
                current=len(self.history.keyframes)-1
                if self.auto_loops and current>=24 and current%8==0 and(self.last_loop_position is None or np.linalg.norm(keyframe.T_odom_body[:3,3]-self.last_loop_position)>=1.):
                    candidates=[(float(np.linalg.norm(self.history.poses[i][:3,3]-self.history.poses[current][:3,3])),i)for i in range(current-20)if self.path_lengths[current]-self.path_lengths[i]>=5.]
                    if candidates:
                        distance,target=min(candidates)
                        if distance<3.:
                            self.stats['loop_candidates']+=1
                            expected=inverse(self.history.poses[target])@self.history.poses[current]
                            lidar_prior=inverse(self.history.keyframes[target].T_body_lidar)@expected@keyframe.T_body_lidar
                            loop=self.history.verify_and_add_loop(target,current,minimum_separation=20,commit=False,T_prior=lidar_prior)
                            self.stats['loop_registration_wall_s']+=loop.elapsed_s
                            result['loop_verification']=dict(accepted=loop.accepted,reason=loop.reason,elapsed_s=loop.elapsed_s,fitness=loop.fitness,rmse=loop.rmse if np.isfinite(loop.rmse)else None,observability=loop.observability)
                            self.stats['last_loop_reason']=loop.reason
                            if loop.accepted:
                                measured=self.history.keyframes[target].T_body_lidar@loop.T_map_lidar@inverse(keyframe.T_body_lidar)
                                expected=inverse(self.history.poses[target])@self.history.poses[current]
                                from .geometry import pose_distance
                                shift,angle=pose_distance(measured,expected)
                                correction=self.history.poses[target]@measured@inverse(keyframe.T_odom_body)
                                pending=self.pending_loop
                                if shift>1. or angle>.3:
                                    self.pending_loop=None;self.stats['last_loop_reason']='loop_inconsistent_with_odometry'
                                elif pending is not None and keyframe.stamp_ns-pending[4]<=30_000_000_000 and pose_distance(correction,pending[3])[0]<.25 and pose_distance(correction,pending[3])[1]<.1:
                                    self.history._add_verified_loop(pending[0],pending[1],pending[2]);self.history._add_verified_loop(target,current,loop)
                                    self.stats['verified_loops']+=2;self.pending_loop=None;self.last_loop_position=keyframe.T_odom_body[:3,3].copy()
                                    graph=self.history.optimize(max_evaluations=30)
                                    result['graph']=graph
                                    self.stats['last_loop_reason']=graph['reason']
                                    if graph.get('optimized'):
                                        self.stats['optimized_graphs']+=1
                                        if self.relocalizer is None:
                                            world_odom=np.eye(4)if self.world_odom_prior is None else self.world_odom_prior
                                            C=world_odom@self.history.poses[current]@inverse(keyframe.T_odom_body)
                                            result.update(accepted=True,reason='two_geometry_verified_loops_and_SE3_graph',correction_kind='verified_loop_graph',T_odom_body_source_ns=keyframe.stamp_ns,T_odom_body=keyframe.T_odom_body.tolist(),C_world_odom=C.tolist(),T_world_body=(C@keyframe.T_odom_body).tolist())
                                else:self.pending_loop=(target,current,loop,correction,keyframe.stamp_ns);self.stats['last_loop_reason']='awaiting_second_loop_geometry'
                            else:self.pending_loop=None
                if self.history_directory is not None:
                    self.history.save(self.history_directory / f'epoch_{keyframe.epoch}')
                result['queue_wait_s']=started-queued_at
                result['published_wall_ns'] = time.time_ns()
                result['backend_wall_s'] = time.monotonic()-started
                self.stats['processed'] += 1
                if self.result_path is not None:
                    write_json(self.result_path, result)
                    with Path(self.result_path).with_name('backend_results.jsonl').open('a') as log:
                        log.write(json.dumps(result,allow_nan=False)+'\n')
            except Exception as error:
                self.stats['errors'] += 1
                self.stats['last_worker_error'] = f'{type(error).__name__}: {error}'
                result = dict(schema='livo_backend_correction/v1', accepted=False,
                              epoch=keyframe.epoch, sequence=keyframe.sequence, source_ns=keyframe.stamp_ns,
                              reason='backend_error', error=f'{type(error).__name__}: {error}')
                result['published_wall_ns'] = time.time_ns()
                if self.result_path is not None:
                    try: write_json(self.result_path, result)
                    except OSError: pass
            finally: self.stats['worker_cpu_s'] += time.thread_time()-cpu
            with self.lock:
                self.result = result
                if not self.jobs: self.ready.clear()
                if self.closed and not self.jobs: return

    def close(self, timeout=10.):
        # Closing/draining is a lifecycle action, never a frontend callback.
        with self.lock: self.closed = True; self.ready.set()
        self.thread.join(timeout)
        return not self.thread.is_alive()
