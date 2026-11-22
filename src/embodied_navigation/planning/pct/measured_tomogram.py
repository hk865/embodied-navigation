"""Current-generation measured support grids for the vendored native PCT A*.

This is a point-footprint global planner, not a clearance or execution certificate.
Only measured cells are populated; there is no ray-carved support, hole filling,
scene asset loading, nearest-goal substitution, or direct-line fallback. The caller
owns exact-stamp cloud/LIO pairing and generation resets. Map points already are
world/map coordinates; this adapter never transforms them with post-VIO poses.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import time

import numpy as np

DEFAULT_MAX_GRID_CELLS = 2_000_000


class MeasuredPctError(RuntimeError):
    """Native planner setup, serialization, or execution failure."""


def native_pct_binary():
    """Build the actual vendored C++ A* in a source-hashed external cache."""
    root = Path(__file__).resolve().parents[4]
    source = root / 'third_party/pct'
    files = sorted(p for p in source.rglob('*') if p.is_file())
    if not files:
        raise MeasuredPctError('vendored PCT sources unavailable')
    digest = hashlib.sha256()
    for path in files:
        digest.update(str(path.relative_to(source)).encode())
        digest.update(path.read_bytes())
    cache_root = Path(os.environ.get('PROJECT_DATA_ROOT', tempfile.gettempdir()))
    build = cache_root / 'embodied-navigation-cache' / ('pct-' + digest.hexdigest()[:16])
    binary = build / 'pct_astar'
    if not binary.is_file():
        # Serialize concurrent worker builds without touching the checkout.
        import fcntl
        build.mkdir(parents=True, exist_ok=True)
        with (build / '.build.lock').open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            if not binary.is_file():
                for command in (["cmake", "-S", str(source), "-B", str(build),
                                 "-DCMAKE_BUILD_TYPE=Release"],
                                ["cmake", "--build", str(build), "--parallel", "2"]):
                    try:
                        done = subprocess.run(command, capture_output=True, text=True, timeout=120)
                    except (OSError, subprocess.TimeoutExpired) as exc:
                        raise MeasuredPctError('native PCT build unavailable: ' + str(exc)) from exc
                    if done.returncode:
                        raise MeasuredPctError('native PCT build failed: ' + done.stdout[-2000:] + done.stderr[-2000:])
    return binary


def _xyz(value, name):
    points = np.asarray(value, dtype=float)
    if points.size == 0:
        return np.empty((0, 3), dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError(name + ' must be finite Nx3 map-frame coordinates')
    return points


class MeasuredPctPlanner:
    """Build a bounded multilayer map exclusively from supplied measured XYZ.

    ``body_support_offset_m`` is in the same child-origin convention as start,
    goal and measured trajectory, not an assumed world floor height. Neighboring
    measured support candidates must span both XY axes (within one grid cell,
    <=0.20 m height change). This rejects isolated returns/thin vertical columns,
    but does not prove a surface is load-bearing. Do not use this as a goto gate.
    """
    def __init__(self, points_world, trajectory_body_xyz=None, *, resolution_m=0.15,
                 body_support_offset_m=0.32, native_binary=None, generation=None,
                 source_metadata=None, endpoint_height_tolerance_m=0.22,
                 support_cluster_m=0.06, clearance_m=0.95,
                 max_grid_cells=DEFAULT_MAX_GRID_CELLS, timeout_s=10.0, start_blind_radius_m=0.0):
        self.points = _xyz(points_world, 'points_world')
        self.trajectory = _xyz([] if trajectory_body_xyz is None else trajectory_body_xyz,
                               'trajectory_body_xyz')
        self.resolution = float(resolution_m)
        self.offset = float(body_support_offset_m)
        self.tolerance = float(endpoint_height_tolerance_m)
        self.cluster = float(support_cluster_m)
        self.clearance = float(clearance_m)
        self.timeout = float(timeout_s)
        self.blind_radius = float(start_blind_radius_m)
        if not np.isfinite(self.blind_radius) or self.blind_radius < 0:
            raise ValueError("start_blind_radius_m must be finite and nonnegative")
        if not all(np.isfinite(v) and v > 0 for v in (self.resolution, self.offset,
                         self.tolerance, self.cluster, self.clearance, self.timeout)):
            raise ValueError('metric parameters and timeout must be finite and positive')
        self.binary = Path(native_binary) if native_binary is not None else None
        self.provenance = dict(generation=generation, source_metadata=dict(source_metadata or {}),
            planner='vendored_native_PCT_Astar', map_source='supplied_current_generation_measured_map_xyz',
            ground_truth_used=False, resolution_m=self.resolution,
            body_support_offset_m=self.offset, measured_points=len(self.points),
            measured_trajectory_poses=len(self.trajectory), unknown_support_traversable=False,
            footprint_model='point; local controller owns collision response',
            source_pairing_validated_by='caller', support_model='measured vertical clusters with XY neighbor continuity')
        self.origin = np.zeros(2)
        self.cost = self.height = None
        if not len(self.points):
            return
        indices = np.floor(self.points[:, :2] / self.resolution).astype(np.int64)
        low, high = indices.min(axis=0), indices.max(axis=0)
        self.origin = low.astype(float) * self.resolution
        width, height = (high - low + 1).tolist()
        if width * height > max_grid_cells:
            raise MeasuredPctError('measured map exceeds configured allocation budget')
        buckets = {}
        for (col, row), z in zip(indices - low, self.points[:, 2]):
            buckets.setdefault((int(row), int(col)), []).append(float(z))
        self.measured_buckets = buckets
        candidates = {}
        for cell, zs in buckets.items():
            values = np.sort(zs)
            groups = np.split(values, np.flatnonzero(np.diff(values) > self.cluster) + 1)
            supports = []
            for group in groups:
                # A vertical wall cluster is not a horizontal support patch.
                if group[-1] - group[0] > self.cluster:
                    continue
                z = float(np.median(group))
                overhead = values[(values > z + max(0.10, self.cluster)) & (values < z + self.clearance)]
                if not len(overhead):
                    supports.append(z)
            candidates[cell] = supports
        retained = {}
        for (row, col), zs in candidates.items():
            good = []
            for z in zs:
                x_ok = any(abs(q-z) <= 0.2000001 for dc in (-1, 1)
                           for q in candidates.get((row, col+dc), []))
                y_ok = any(abs(q-z) <= 0.2000001 for dr in (-1, 1)
                           for q in candidates.get((row+dr, col), []))
                if x_ok and y_ok:
                    good.append(z)
            if good:
                retained[(row, col)] = good
        if not retained:
            return
        self.z_origin = min(z for zs in retained.values() for z in zs)
        # Native node identity uses int(z/res), so deduplicate that identity.
        for cell, zs in retained.items():
            unique = {}
            for z in zs:
                unique.setdefault(int((z-self.z_origin)/self.resolution), z)
            retained[cell] = list(unique.values())
        layers = max(map(len, retained.values()))
        if layers * width * height > max_grid_cells:
            raise MeasuredPctError('measured multilayer map exceeds configured allocation budget')
        self.cost = np.full((layers, height, width), 1e6)
        self.height = np.zeros_like(self.cost)
        for (row, col), zs in retained.items():
            for layer, z in enumerate(zs):
                self.cost[layer, row, col] = 0.0
                self.height[layer, row, col] = z - self.z_origin
        if np.max(self.height) / self.resolution >= 210:
            raise MeasuredPctError('native PCT integer height hash range exceeded')
        self.provenance.update(grid_shape=list(self.cost.shape), origin_xy=self.origin.tolist(),
                               native_height_origin_m=self.z_origin,
                               measured_support_cells=int(np.sum(self.cost == 0)))

    def _start_surface_model(self, start):
        """Optional local inference, separate from untouched raw measured grid.

        Assumption: the surrounding locally planar surface continues underneath
        the sensor blind region. Invisible holes cannot be ruled out. Derived
        cells are neither observed free nor certified physical support.
        """
        cost, heights = self.cost.copy(), self.height.copy()
        report = dict(enabled=True, model='local_start_blind_region_plane_continuation',
                      radius_m=self.blind_radius, inferred_support_cells=0,
                      inferred_cells_layer_row_col=[], raw_measured_cells_unchanged=True,
                      observed_free=False, support_certified=False,
                      assumption='surrounding observed plane continues across unobserved start blind region')
        layer, row, col = np.where(self.cost == 0)
        x = self.origin[0] + (col + .5)*self.resolution
        y = self.origin[1] + (row + .5)*self.resolution
        z = self.height[layer,row,col] + self.z_origin
        dx, dy = x-start[0], y-start[1]
        radius = np.hypot(dx,dy)
        select = ((radius >= self.blind_radius*.65) &
                  (radius <= self.blind_radius+.7) &
                  (abs(z-(start[2]-self.offset)) <= .10))
        dx,dy,z = dx[select],dy[select],z[select]
        report['fit_measured_support_cells'] = len(z)
        if len(z)<20:
            report['reason']='insufficient_surrounding_measured_support'
            return cost, heights, report
        quadrants = [(int(sx),int(sy)) for sx,sy in zip(dx>=0,dy>=0)]
        if len(set(quadrants))<4:
            report['reason']='surrounding_support_not_in_four_quadrants'
            return cost, heights, report
        design=np.column_stack([dx,dy,np.ones(len(z))])
        keep=np.ones(len(z),dtype=bool)
        for _ in range(4):
            coefficients=np.linalg.lstsq(design[keep],z[keep],rcond=None)[0]
            residual=abs(design@coefficients-z)
            keep=residual<=max(.015,min(.04,float(np.quantile(residual,.85))))
            if keep.sum()<20:
                report['reason']='plane_fit_insufficient_inliers'
                return cost, heights, report
        # Evaluate all measured candidate samples, not merely fit inliers, so a
        # stair discontinuity is not silently removed as an outlier population.
        residual=abs(design@coefficients-z)
        slope=float(np.linalg.norm(coefficients[:2]))
        error=float(np.quantile(residual,.9))
        report.update(plane_z_equals_a_dx_plus_b_dy_plus_c=coefficients.tolist(),
                      fit_residual_p90_m=error, slope=slope,
                      fit_inlier_cells=int(keep.sum()))
        if error>.035 or slope>.15:
            report['reason']='surrounding_support_not_locally_planar'
            return cost, heights, report
        rr,cc=np.indices(cost.shape[1:])
        xx=self.origin[0]+(cc+.5)*self.resolution-start[0]
        yy=self.origin[1]+(rr+.5)*self.resolution-start[1]
        plane=coefficients[0]*xx+coefficients[1]*yy+coefficients[2]
        allowed=(np.hypot(xx,yy)<=self.blind_radius)
        # Never overwrite a measured cell below overhead clearance, including a
        # measured blocked cell. Ceiling returns above clearance do not occupy
        # the ground blind region. A one-cell halo around incompatible elevations
        # prevents interpolation from bridging observed obstacles or drop edges.
        for (r,c),zs in self.measured_buckets.items():
            values=np.asarray(zs)
            below_ceiling=values < plane[r,c] + self.clearance
            if np.any(below_ceiling):
                allowed[r,c]=False
            incompatible=np.any((values<plane[r,c]-.12) | ((values>plane[r,c]+.10)&below_ceiling))
            if incompatible:
                allowed[max(0,r-1):r+2,max(0,c-1):c+2]=False
        c0,r0=np.floor((start[:2]-self.origin)/self.resolution+1e-9).astype(int)
        if not (0<=r0<allowed.shape[0] and 0<=c0<allowed.shape[1] and allowed[r0,c0]):
            report['reason']='start_cell_not_unobserved_blind_region'
            return cost, heights, report
        # Only the unknown component containing the start may be inferred.
        stack=[(r0,c0)]; component=set(stack)
        while stack:
            r,c=stack.pop()
            for nr,nc in ((r-1,c),(r+1,c),(r,c-1),(r,c+1)):
                if (0<=nr<allowed.shape[0] and 0<=nc<allowed.shape[1] and
                        allowed[nr,nc] and (nr,nc) not in component):
                    component.add((nr,nc)); stack.append((nr,nc))
        for r,c in sorted(component):
            cost[0,r,c]=0.
            heights[0,r,c]=plane[r,c]-self.z_origin
        report['inferred_support_cells']=len(component)
        report['inferred_cells_layer_row_col']=[[0,int(r),int(c)] for r,c in sorted(component)]
        report['reason']=None
        return cost, heights, report

    def _node(self, body, cost=None, heights=None):
        cost = self.cost if cost is None else cost
        heights = self.height if heights is None else heights
        col, row = np.floor((body[:2] - self.origin) / self.resolution + 1e-9).astype(int)
        if row < 0 or col < 0 or row >= cost.shape[1] or col >= cost.shape[2]:
            return None
        target = body[2] - self.offset - self.z_origin
        dz = np.abs(heights[:, row, col] - target)
        eligible = np.flatnonzero((cost[:, row, col] < 20) & (dz <= self.tolerance))
        if not len(eligible):
            return None
        layer = int(eligible[np.argmin(dz[eligible])])
        return [layer, int(col), int(row)]

    def _cell_blocks_start_snap(self, r, c, foot_z):
        """Return blocking reason if cell (r, c) contains a torso obstacle or cliff drop."""
        cell_zs = self.measured_buckets.get((int(r), int(c)))
        if not cell_zs:
            return None
        vals = np.asarray(cell_zs, dtype=float)
        if np.any((vals > foot_z + self.tolerance) & (vals < foot_z + self.clearance)):
            return 'start_cell_obstacle'
        if np.any(vals < foot_z - self.tolerance - 0.10):
            return 'start_cell_cliff_drop'
        return None

    def _has_traversable_neighbor(self, layer, r, c):
        """Return True iff cell (layer, r, c) has >=1 reachable 8-neighbor in PCT."""
        z = float(self.height[layer, r, c])
        rows, cols = self.cost.shape[1], self.cost.shape[2]
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    if np.any((self.cost[:, nr, nc] < 20) & (np.abs(self.height[:, nr, nc] - z) <= 0.200001)):
                        return True
        return False

    def _footprint_start_node(self, body, max_radius_m=0.30):
        """Attach start to a nearby orthogonal measured support cell within Go2's footprint.

        Rejects torso-band obstacles, measured cliff drops below foot level, and
        diagonal-only corner cuts. Allows a start cell whose ground return is
        missing (e.g. LiDAR belly self-filter blind cell or ceiling-only column)
        or split across a step nosing to attach to measured support inside the
        robot's physical standing footprint without widening goal admission.
        """
        c0, r0 = np.floor((body[:2] - self.origin) / self.resolution + 1e-9).astype(int)
        if r0 < 0 or c0 < 0 or r0 >= self.cost.shape[1] or c0 >= self.cost.shape[2]:
            return None, dict(applied=False, max_radius_m=max_radius_m, reason='start_out_of_bounds')
        foot_z = float(body[2] - self.offset)
        block_reason = self._cell_blocks_start_snap(r0, c0, foot_z)
        if block_reason is not None:
            return None, dict(applied=False, max_radius_m=max_radius_m, reason=block_reason)

        target = foot_z - self.z_origin
        best = None
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            for step in (1, 2):
                r, c = int(r0 + dr * step), int(c0 + dc * step)
                if r < 0 or c < 0 or r >= self.cost.shape[1] or c >= self.cost.shape[2]:
                    break
                if step == 2 and self._cell_blocks_start_snap(r0 + dr, c0 + dc, foot_z) is not None:
                    break
                cx = self.origin[0] + (c + 0.5) * self.resolution
                cy = self.origin[1] + (r + 0.5) * self.resolution
                horiz = float(np.hypot(cx - body[0], cy - body[1]))
                if horiz > max_radius_m + 1e-9:
                    break
                dz = np.abs(self.height[:, r, c] - target)
                eligible = np.flatnonzero((self.cost[:, r, c] < 20) & (dz <= self.tolerance))
                if not len(eligible):
                    continue
                layer = int(eligible[np.argmin(dz[eligible])])
                if not self._has_traversable_neighbor(layer, r, c):
                    continue
                vert = float(dz[layer])
                key = (horiz, vert, layer, r, c)
                if best is None or key < best[0]:
                    best = (key, [layer, c, r], horiz, vert)
                break
        if best is None:
            return None, dict(applied=False, max_radius_m=max_radius_m, reason='no_adjacent_support')
        _, node, horiz, vert = best
        return node, dict(
            applied=True,
            max_radius_m=max_radius_m,
            cell_layer_row_col=[node[0], node[2], node[1]],
            horizontal_distance_m=horiz,
            vertical_delta_m=vert,
        )

    def plan(self, start_body_xyz, goal_body_xyz, *, cancel_event=None):
        """Return {found, body_path_xyz, reason, provenance}; never return a partial route."""
        start, goal = _xyz([start_body_xyz, goal_body_xyz], 'endpoints')
        result = dict(found=False, body_path_xyz=[], reason=None, provenance=dict(self.provenance))
        if cancel_event is not None and cancel_event.is_set():
            result['reason'] = 'cancelled'
            return result
        if self.cost is None:
            result['reason'] = 'no_measured_support'
            return result
        cost, heights = self.cost, self.height
        first = self._node(start, cost, heights)
        if first is not None and not self._has_traversable_neighbor(first[0], first[2], first[1]):
            first = None
        footprint_snap = dict(applied=False, max_radius_m=0.30)
        if first is None:
            first, footprint_snap = self._footprint_start_node(start)
        result['provenance']['start_footprint_snap'] = footprint_snap
        if cancel_event is not None and cancel_event.is_set():
            result['reason'] = 'cancelled'
            return result
        if first is None and self.blind_radius > 0:
            cost, heights, model = self._start_surface_model(start)
            result['provenance']['start_surface_model'] = model
            first = self._node(start, cost, heights)
        else:
            result['provenance']['start_surface_model'] = dict(enabled=self.blind_radius > 0, applied=False)
        last = self._node(goal, cost, heights)
        if first is None or last is None:
            result['reason'] = 'start_support_unobserved' if first is None else 'goal_support_unobserved'
            return result
        binary = self.binary or native_pct_binary()
        with tempfile.TemporaryDirectory(prefix='measured-pct-') as directory:
            work = Path(directory)
            with (work/'map.bin').open('wb') as stream:
                stream.write(struct.pack('<iiid', *cost.shape, self.resolution))
                for values in (cost, heights, np.zeros_like(cost)):
                    stream.write(np.asarray(values, dtype='<f8').tobytes(order='C'))
            (work/'query.txt').write_text(' '.join(map(str, first+last)))
            try:
                proc = subprocess.Popen(
                    [str(binary), str(work/'map.bin'), str(work/'query.txt'), str(work/'path.json')],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
            except OSError as exc:
                raise MeasuredPctError('native PCT execution failed: ' + str(exc)) from exc
            deadline = time.monotonic() + self.timeout
            returncode = None
            stdout_data = ''
            stderr_data = ''
            while True:
                if cancel_event is not None and cancel_event.is_set():
                    proc.terminate()
                    try:
                        proc.wait(timeout=0.2)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=0.5)
                    result['reason'] = 'cancelled'
                    return result
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    proc.kill()
                    proc.wait(timeout=0.5)
                    raise MeasuredPctError('native PCT execution failed: timed out')
                try:
                    stdout_data, stderr_data = proc.communicate(timeout=min(0.02, remaining))
                    returncode = proc.returncode
                    break
                except subprocess.TimeoutExpired:
                    continue
            if returncode not in (0, 4):
                raise MeasuredPctError('native PCT returned ' + str(returncode) + ': ' + stderr_data[-2000:])
            try:
                native = json.loads((work/'path.json').read_text())
            except (OSError, ValueError) as exc:
                raise MeasuredPctError('native PCT result unreadable') from exc
        result['provenance']['native_binary'] = str(binary)
        if returncode == 4 or not native.get('found'):
            result['reason'] = 'native_no_path'
            return result
        cells = [[first[0], first[2], first[1]]] + native['cells_layer_row_col']
        route = []
        for layer, row, col in cells:
            route.append([self.origin[0]+(col+.5)*self.resolution,
                          self.origin[1]+(row+.5)*self.resolution,
                          heights[layer,row,col]+self.z_origin+self.offset])
        # Preserve user endpoints in the estimator child-origin convention.
        if len(route) == 1:
            route = [start.tolist(), goal.tolist()]
        else:
            route[0], route[-1] = start.tolist(), goal.tolist()
        result.update(found=True, body_path_xyz=route, reason=None)
        return result
