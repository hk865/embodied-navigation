"""Owned local-LiDAR keyframes and an actual anchored SE(3) pose graph.

Persistence and graph optimization run only on the backend worker. No image or
frontend map pointers cross this boundary. Adjacent odometry edges are not new
independent sensor observations, and no fixed-lag/IMU factor claim is made.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from pathlib import Path
import json
import numpy as np
from .geometry import checked_transform, inverse, transform_points


@dataclass(frozen=True)
class Keyframe:
    epoch: int
    sequence: int
    stamp_ns: int
    T_odom_body: np.ndarray
    T_body_lidar: np.ndarray
    points_lidar: np.ndarray
    final_stage: str = 'lio'

    @classmethod
    def owned(cls, epoch, sequence, stamp_ns, T_odom_body, T_body_lidar,
              points_lidar, final_stage='lio'):
        if (any(type(value) is not int for value in (epoch, sequence, stamp_ns))
                or min(epoch, sequence, stamp_ns) <= 0):
            raise ValueError('keyframe requires integer epoch/sequence/measurement timestamp')
        if final_stage not in ('lio', 'livo', 'lio_vio_skipped'):
            raise ValueError('unknown frontend final stage')
        points = np.array(points_lidar, dtype=np.float64, copy=True)
        if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
            raise ValueError('invalid local LiDAR cloud')
        odom = checked_transform(T_odom_body); extrinsic = checked_transform(T_body_lidar)
        points.flags.writeable = odom.flags.writeable = extrinsic.flags.writeable = False
        return cls(epoch, sequence, stamp_ns, odom, extrinsic, points, final_stage)


@dataclass
class Edge:
    target: int
    source: int
    T_target_source: np.ndarray
    kind: str
    translation_sigma: float
    rotation_sigma: float
    quality: dict = field(default_factory=dict)


class History:
    def __init__(self, submap_size=8):
        if submap_size < 1: raise ValueError('submap_size must be positive')
        self.submap_size = submap_size
        self.keyframes: list[Keyframe] = []
        self.poses: list[np.ndarray] = []
        self.edges: list[Edge] = []
        self.revision = 0
        self._saved_submaps={}

    def add(self, keyframe):
        if self.keyframes:
            last = self.keyframes[-1]
            if keyframe.epoch != last.epoch: raise ValueError('reset requires a separate History')
            if keyframe.sequence <= last.sequence or keyframe.stamp_ns <= last.stamp_ns:
                raise ValueError('duplicate or out-of-order keyframe')
            relative = inverse(last.T_odom_body) @ keyframe.T_odom_body
            self.edges.append(Edge(len(self.keyframes)-1, len(self.keyframes), relative,
                                   'adjacent', .06, .035))
            pose = self.poses[-1] @ relative
        else: pose = keyframe.T_odom_body.copy()
        self.keyframes.append(keyframe); self.poses.append(pose)
        return len(self.keyframes)-1

    @property
    def submaps(self):
        return [list(range(i, min(i+self.submap_size, len(self.keyframes))))
                for i in range(0, len(self.keyframes), self.submap_size)]

    def cloud(self, indices=None, max_points=None):
        indices = range(len(self.keyframes)) if indices is None else indices
        chunks = [transform_points(self.keyframes[i].points_lidar,
                                   self.poses[i] @ self.keyframes[i].T_body_lidar) for i in indices]
        out = np.concatenate(chunks) if chunks else np.empty((0, 3))
        if max_points and len(out) > max_points:
            out = out[np.linspace(0, len(out)-1, max_points, dtype=int)]
        return out

    def verify_and_add_loop(self, target, source, config=None, minimum_separation=3, commit=True, T_prior=None):
        """Explicit candidate -> actual historical-cloud geometry -> graph edge.

        Candidate retrieval/automatic scheduling is deliberately not implied.
        This first phase verifies two keyframes, not a deformable dense submap.
        The source/target clouds are chosen here, so a result from a different
        map cannot accidentally be associated with these graph endpoints.
        """
        if not 0 <= target < source < len(self.keyframes) or source-target < minimum_separation:
            raise ValueError('loop candidate is not temporally separated history')
        from .external_map import ExternalMap
        from .registration import Relocalizer, RegistrationConfig
        target_cloud = self.keyframes[target].points_lidar
        # One bounded historical scan is a target asset, not a fabricated path.
        extent = np.ptp(target_cloud, axis=0)
        block_size = max(20., float(extent.max()) + 2.)
        target_map = ExternalMap(target_cloud, block_size=block_size, overlap=1.)
        if T_prior is None:
            registration = Relocalizer(target_map, config or RegistrationConfig()).localize(self.keyframes[source].points_lidar)
        else:
            from .tracking import PriorTracker
            registration = PriorTracker(target_map, config or RegistrationConfig()).localize(self.keyframes[source].points_lidar,T_prior)

        if registration.accepted and commit: self._add_verified_loop(target, source, registration)
        return registration

    def _add_verified_loop(self, target, source, registration):
        """registration maps source local LiDAR to target local LiDAR.

        Only accepted independent geometric results may enter; caller must use a
        historical target/submap that excludes the source and temporal neighbors.
        """
        if not (0 <= target < source < len(self.keyframes)):
            raise ValueError('loop endpoints must be distinct historical keyframes')
        if not registration.accepted or registration.T_map_lidar is None:
            raise ValueError('unverified registration cannot become a graph edge')
        if not np.isfinite([registration.fitness, registration.rmse, registration.observability]).all():
            raise ValueError('invalid loop quality')
        if registration.fitness < .55 or registration.rmse > .15 or registration.observability <= 0:
            raise ValueError('loop quality outside graph admission limits')
        if any(e.kind == 'loop' and e.target == target and e.source == source for e in self.edges):
            raise ValueError('duplicate loop edge')
        a, b = self.keyframes[target], self.keyframes[source]
        relative = a.T_body_lidar @ checked_transform(registration.T_map_lidar) @ inverse(b.T_body_lidar)
        self.edges.append(Edge(target, source, relative, 'loop', max(.025, registration.rmse), .025,
                               dict(fitness=registration.fitness, rmse=registration.rmse,
                                    observability=registration.observability)))

    def optimize(self, max_evaluations=100):
        if len(self.keyframes)<2 or not any(e.kind=='loop'for e in self.edges):
            return dict(optimized=False,reason='no_verified_loop',revision=self.revision)
        from .native_graph import optimize_graph
        values,report=optimize_graph(self.poses,self.edges,iterations=min(8,max(1,max_evaluations)))
        if values is not None:
            self.poses=[pose.copy()for pose in values];self.revision+=1
        report['revision']=self.revision
        return report

    def save(self, directory):
        directory = Path(directory); directory.mkdir(parents=True, exist_ok=True)
        records = []
        for number,indices in enumerate(self.submaps):
            key=(str(directory.resolve()),number);revision=(len(indices),self.revision)
            if self._saved_submaps.get(key)!=revision:
                anchor=self.poses[indices[0]];cloud=self.cloud(indices)
                local=transform_points(cloud,inverse(anchor));temporary=directory/f'submap_{number:06d}.npy.tmp'
                with temporary.open('wb') as stream:np.save(stream,local,allow_pickle=False)
                temporary.replace(directory/f'submap_{number:06d}.npy')
                self._saved_submaps[key]=revision

        for i, k in enumerate(self.keyframes):
            filename = f'keyframe_{k.epoch}_{k.sequence:08d}.npy'
            if not (directory / filename).exists(): np.save(directory / filename, k.points_lidar, allow_pickle=False)
            records.append(dict(epoch=k.epoch, sequence=k.sequence, stamp_ns=k.stamp_ns,
                                T_odom_body=k.T_odom_body.tolist(), T_body_lidar=k.T_body_lidar.tolist(),
                                T_map_body=self.poses[i].tolist(), final_stage=k.final_stage, cloud=filename))
        data = dict(schema='livo_history/v1', revision=self.revision, submap_size=self.submap_size,
                    keyframes=records, submaps=self.submaps,
                    submap_assets=[dict(cloud=f'submap_{i:06d}.npy',T_map_submap=self.poses[ids[0]].tolist(),keyframes=ids)for i,ids in enumerate(self.submaps)],
                    edges=[dict(target=e.target, source=e.source, T_target_source=e.T_target_source.tolist(),
                                kind=e.kind, translation_sigma=e.translation_sigma,
                                rotation_sigma=e.rotation_sigma, quality=e.quality) for e in self.edges])
        temporary = directory / 'history.json.tmp'
        temporary.write_text(json.dumps(data, allow_nan=False)+'\n'); temporary.replace(directory / 'history.json')

    @classmethod
    def load(cls, directory):
        directory = Path(directory); data = json.loads((directory / 'history.json').read_text())
        if data.get('schema') != 'livo_history/v1': raise ValueError('unknown history schema')
        result = cls(data['submap_size'])
        for record in data['keyframes']:
            filename = Path(record['cloud'])
            if filename.name != str(filename): raise ValueError('cloud path must remain within history')
            result.add(Keyframe.owned(record['epoch'], record['sequence'], record['stamp_ns'],
                record['T_odom_body'], record['T_body_lidar'], np.load(directory / filename, allow_pickle=False),
                record['final_stage']))
        result.poses = [checked_transform(k['T_map_body']) for k in data['keyframes']]
        result.edges = [Edge(e['target'], e['source'], checked_transform(e['T_target_source']),
                            e['kind'], e['translation_sigma'], e['rotation_sigma'], e['quality']) for e in data['edges']]
        for edge in result.edges:
            if not 0 <= edge.target < edge.source < len(result.keyframes): raise ValueError('bad history edge')
            if edge.kind not in ('adjacent', 'loop') or min(edge.translation_sigma, edge.rotation_sigma) <= 0:
                raise ValueError('bad history edge parameters')
        result.revision = data['revision']; return result
