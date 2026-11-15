"""Bounded CPU virtual LiDAR opportunity, NOT observed free space or entropy.

Only measured endpoints occlude rays. Unknown space is optimistically transparent.
Physical firing origins are unavailable in paired LIO snapshots: historical virtual
scans remain a model, never a measurement. This module has no execution APIs.
"""
from dataclasses import dataclass
import hashlib
import io
import json
from pathlib import Path
import numpy as np

CALIBRATION = json.loads(Path(__file__).with_name('os1_calibration.json').read_text())


def _xyz(value, name):
    a = np.asarray(value, dtype=float)
    if a.size == 0:
        return np.empty((0, 3))
    if a.ndim != 2 or a.shape[1] != 3 or not np.isfinite(a).all():
        raise ValueError(name + ' must be finite Nx3')
    return a


def voxel_keys(points, voxel_m):
    if not np.isfinite(voxel_m) or voxel_m <= 0:
        raise ValueError('voxel_m must be positive')
    return {tuple(k) for k in np.floor(_xyz(points, 'points') / voxel_m).astype(np.int64)}


@dataclass(frozen=True)
class MeasuredSnapshot:
    points_xyz: np.ndarray
    generation: str
    map_frame: str = 'map'
    source_metadata: dict = None

    def __post_init__(self):
        if not self.generation or not self.map_frame:
            raise ValueError('generation and map_frame required')
        points = _xyz(self.points_xyz, 'measured endpoints').copy()
        points.flags.writeable = False
        object.__setattr__(self, 'points_xyz', points)
        object.__setattr__(self, 'source_metadata', dict(self.source_metadata or {}))
        if self.source_metadata.get('ground_truth_used') is True:
            raise ValueError('truth-derived snapshot refused')


def load_measured_snapshot(path, expected_generation=None):
    """Load caller-exported measured endpoints; retain original provenance/hash.

    A hash authenticates file identity, not physical provenance. The exporter owns
    exact same-stage LIO pairing; this reader does not invent missing evidence.
    """
    path = Path(path)
    raw = path.read_bytes()
    with np.load(io.BytesIO(raw), allow_pickle=False) as a:
        meta = json.loads(str(a['provenance_json'].item())) if 'provenance_json' in a else {}
        def scalar(key, default=None):
            return a[key].item() if key in a else default
        generation = str(meta.get('run_id', scalar('run_id', '')))
        frame = str(meta.get('map_frame', scalar('cloud_map_frame', '')))
        key = 'points_xyz_m' if 'points_xyz_m' in a else 'xyz'
        points = np.array(a[key], copy=True)
        for k in ('cloud_stage', 'exact_integer_stamp_pair', 'cloud_map_frame', 'pose_map_frame'):
            if k in a:
                meta[k] = scalar(k)
        if meta.get('cloud_stage', 'LIO') != 'LIO':
            raise ValueError('only same-stage LIO endpoints supported')
        if meta.get('exact_integer_stamp_pair') is False:
            raise ValueError('unpaired snapshot refused')
        if meta.get('pose_map_frame', frame) != frame:
            raise ValueError('cloud/pose frame mismatch')
    if expected_generation is not None and generation != expected_generation:
        raise ValueError('snapshot generation mismatch')
    meta.update(source_path=str(path), source_sha256=hashlib.sha256(raw).hexdigest(),
                reader_validates_physical_pairing=False)
    return MeasuredSnapshot(points, generation, frame, meta)


def sample_route(route, spacing_m=1., max_samples=32):
    """Arc-length samples over the entire polyline, never a truncated prefix."""
    route = _xyz(route, 'route')
    if not len(route):
        raise ValueError('route is empty')
    if not np.isfinite(spacing_m) or spacing_m <= 0 or max_samples < 2:
        raise ValueError('invalid sampling budget')
    lengths = np.linalg.norm(np.diff(route, axis=0), axis=1)
    route = route[np.r_[True, lengths > 1e-10]]
    lengths = np.linalg.norm(np.diff(route, axis=0), axis=1)
    total = float(lengths.sum())
    if total == 0:
        return route.copy(), np.zeros(1), dict(route_length_m=0., sample_count=1,
            actual_spacing_m=0., requested_spacing_m=spacing_m, budget_coarsened=False)
    wanted = max(2, int(np.ceil(total / spacing_m)) + 1)
    count = min(wanted, int(max_samples))
    arc = np.r_[0., np.cumsum(lengths)]
    distances = np.linspace(0., total, count)
    indices = np.minimum(np.searchsorted(arc, distances, side='right') - 1, len(lengths)-1)
    fractions = (distances - arc[indices]) / lengths[indices]
    points = route[indices] + fractions[:, None] * (route[indices+1] - route[indices])
    tangents = route[indices+1] - route[indices]
    yaw = np.arctan2(tangents[:, 1], tangents[:, 0])
    return points, yaw, dict(route_length_m=total, sample_count=count,
        actual_spacing_m=total/(count-1), requested_spacing_m=spacing_m,
        budget_coarsened=wanted > count, includes_both_endpoints=True)


class RouteInformationScorer:
    def __init__(self, snapshot, *, voxel_m=.5, max_range_m=8., azimuth_samples=32,
                 route_spacing_m=1., max_route_samples=32,
                 lidar_offset_in_route_frame=(0., 0., .25), baseline_poses_xyz=None,
                 baseline_yaws_rad=None, extra_baseline_routes=None):
        self.snapshot = snapshot
        self.voxel_m = float(voxel_m)
        self.occupied = voxel_keys(snapshot.points_xyz, self.voxel_m)
        self.range = float(max_range_m)
        if not CALIBRATION['near_range_m'] < self.range <= CALIBRATION['far_range_m']:
            raise ValueError('analysis range outside calibrated range')
        if not 1 <= int(azimuth_samples) <= 1024 or int(azimuth_samples) != azimuth_samples:
            raise ValueError('azimuth_samples must be integer 1..1024')
        if self.range/self.voxel_m > 512:
            raise ValueError('range/voxel CPU budget exceeded; use coarser voxels')
        if not 2 <= int(max_route_samples) <= 256:
            raise ValueError('max_route_samples must be 2..256')
        self.spacing, self.max_samples = float(route_spacing_m), int(max_route_samples)
        if not np.isfinite(self.spacing) or self.spacing <= 0:
            raise ValueError('route_spacing_m must be positive')
        self.offset = _xyz([lidar_offset_in_route_frame], 'LiDAR offset')[0]
        elevations = np.deg2rad(CALIBRATION['elevation_deg'])
        columns = np.floor(np.arange(int(azimuth_samples))*1024/int(azimuth_samples))
        angles = np.deg2rad(columns[:, None]*360/1024 + np.array(CALIBRATION['azimuth_offset_deg'])[None, :])
        self.directions = np.stack([np.cos(angles)*np.cos(elevations),
            np.sin(angles)*np.cos(elevations), np.broadcast_to(np.sin(elevations), angles.shape)], axis=-1).reshape(-1,3)
        baseline = _xyz([] if baseline_poses_xyz is None else baseline_poses_xyz, 'baseline poses')
        baseline_yaws = np.zeros(len(baseline)) if baseline_yaws_rad is None else np.asarray(baseline_yaws_rad, dtype=float)
        if baseline_yaws.shape != (len(baseline),) or not np.isfinite(baseline_yaws).all():
            raise ValueError('baseline_yaws_rad must match baseline poses')
        indices = np.unique(np.linspace(0, len(baseline)-1, min(len(baseline), self.max_samples)).astype(int)) if len(baseline) else []
        self.baseline = set()
        for index in indices:
            angle = baseline_yaws[index]
            c,s = np.cos(angle), np.sin(angle)
            dx,dy,dz = self.offset
            origin = baseline[index] + [c*dx-s*dy,s*dx+c*dy,dz]
            self.baseline.update(self._visible_unknown(origin, angle))
        self.baseline_sampling = dict(input_pose_count=len(baseline), used_pose_count=len(indices),
            budget_coarsened=len(indices)<len(baseline),
            yaw_source="explicit" if baseline_yaws_rad is not None else "assumed_zero")
        # Future active legs have their own budget: long history cannot starve
        # the already-committed route visibility subtraction.
        extra_routes = [] if extra_baseline_routes is None else list(extra_baseline_routes)
        if len(extra_routes) > 8:
            raise ValueError('at most eight extra baseline routes are supported')
        self.baseline_sampling['extra_routes'] = []
        for route in extra_routes:
            positions, angles, sampling = sample_route(route, self.spacing, self.max_samples)
            before = len(self.baseline)
            for point, angle in zip(positions, angles):
                c,s = np.cos(angle), np.sin(angle)
                dx,dy,dz = self.offset
                origin = point + [c*dx-s*dy,s*dx+c*dy,dz]
                self.baseline.update(self._visible_unknown(origin, angle))
            self.baseline_sampling['extra_routes'].append(dict(sampling,
                yaw_source='route_tangent', additional_unique_unknown_voxels=len(self.baseline)-before,
                semantic='modeled committed-route visibility, not observed-free'))

    def _visible_unknown(self, origin, yaw):
        """Vectorized 3D voxel DDA, stopping at measured occupied endpoints.

        Voxels intersecting the valid range contribute volume proxies. Boundaries
        use a half-open voxel convention; tied axes advance together.
        """
        c, s = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        rays = self.directions @ rotation.T
        near = CALIBRATION['near_range_m']
        start = origin[None, :] + rays * near
        cells = np.floor(start/self.voxel_m).astype(np.int64)
        step = np.sign(rays).astype(np.int64)
        safe = np.where(abs(rays)>1e-14, rays, np.inf)
        delta = np.where(abs(rays)>1e-14, self.voxel_m/abs(safe), np.inf)
        boundary = (cells + (step > 0))*self.voxel_m
        next_t = np.where(abs(rays)>1e-14, (boundary-origin)/safe, np.inf)
        active = np.ones(len(rays), dtype=bool)
        visible = set()
        for _ in range(int(np.ceil(3*self.range/self.voxel_m))+6):
            ids = np.flatnonzero(active)
            if not len(ids):
                break
            for i in ids:
                key = tuple(cells[i])
                if key in self.occupied:
                    active[i] = False
                else:
                    visible.add(key)
            cross = np.min(next_t, axis=1)
            active &= cross < self.range
            advance = (next_t <= cross[:, None]+1e-12) & active[:, None]
            cells += step*advance
            next_t += np.where(advance, delta, 0.)
        return visible

    def score_route(self, body_path_xyz):
        points, yaw, sampling = sample_route(body_path_xyz, self.spacing, self.max_samples)
        union = set()
        for point, angle in zip(points, yaw):
            c,s = np.cos(angle), np.sin(angle)
            dx,dy,dz = self.offset
            origin = point + [c*dx-s*dy, s*dx+c*dy, dz]
            union.update(self._visible_unknown(origin, angle))
        novel = union - self.baseline
        count = len(novel)
        length = sampling['route_length_m']
        return dict(potential_novel_voxel_count=count,
            potential_novel_volume_m3=count*self.voxel_m**3,
            prospective_unknown_voxel_count=len(union),
            modeled_baseline_unknown_voxel_count=len(self.baseline),
            route_length_m=length, utility_voxels_per_m=count/max(length,self.voxel_m),
            sampling=dict(route=sampling, baseline=self.baseline_sampling,
                          azimuth_samples=len(self.directions)//32, channels=32),
            provenance=dict(generation=self.snapshot.generation, map_frame=self.snapshot.map_frame,
                source_metadata=self.snapshot.source_metadata, sensor_asset_sha256=CALIBRATION['asset_sha256']),
            assumptions=dict(metric='optimistic modeled unmeasured-volume opportunity',
                observed_free_available=False, measured_information_gain=False,
                unknown_is_transparent=True, occluders='measured endpoint voxels only; sparse walls may leak',
                baseline='modeled virtual scans, never measured-free',
                sensor_pose='gravity aligned; route tangent yaw; baseline yaw explicit or disclosed zero; no firing-motion simulation',
                lidar_offset_in_route_frame_m=self.offset.tolist(),
                coordinate_contract='route and baseline positions share declared child origin; default is base',
                voxel_m=self.voxel_m, analysis_range_m=self.range,
                sensor_near_range_m=CALIBRATION['near_range_m'], sensor_far_range_m=CALIBRATION['far_range_m'],
                full_route_sampled=True, inferred_support_used=False, visual_localization_quality_available=False,
                volume_semantics='whole intersected voxel volumes; boundary approximation, not sensed free volume'))


def newly_measured_voxel_delta(before, after, voxel_m=.5):
    if before.generation != after.generation or before.map_frame != after.map_frame:
        raise ValueError('endpoint delta requires same generation and map frame')
    previous = voxel_keys(before.points_xyz, voxel_m)
    current = voxel_keys(after.points_xyz, voxel_m)
    return dict(new_measured_endpoint_voxel_count=len(current-previous),
        before_measured_endpoint_voxel_count=len(previous), after_measured_endpoint_voxel_count=len(current),
        voxel_m=float(voxel_m), generation=before.generation, map_frame=before.map_frame,
        metric='exact endpoint voxel set difference, not newly observed free volume')
