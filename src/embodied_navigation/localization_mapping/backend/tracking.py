"""Trajectory-free XYZ map tracking in an explicitly supplied SE3 prior basin.

This is not a claim of global uniqueness. Independent holdout geometry and
competing seeds decide admission; the cold global path remains Relocalizer.
"""
import time
from dataclasses import replace
import numpy as np
from scipy.spatial import cKDTree
from .geometry import bounded_sample, checked_transform, pose_distance, transform_points, voxel_sample
from .registration import (RegistrationConfig, RegistrationResult, _refine,
                           _observability, surface_normals, refine_planes)


class PriorTracker:
    def __init__(self, map, config=None):
        self.map = map
        self.config = config or RegistrationConfig()
        self.minimum = map.sampled_points.min(0) - .5
        self.maximum = map.sampled_points.max(0) + .5
        self.levels = []
        for leaf, distance in ((.6, .9), (.3, .5), (.12, .22)):
            points = voxel_sample(map.sampled_points, leaf)
            self.levels.append((leaf, distance, points, cKDTree(points)))
        self.target, self.tree = self.levels[-1][2:]
        self.normals, self.planar = surface_normals(self.target, self.tree)

    def localize(self, scan_lidar, T_prior=None):
        started = time.monotonic()
        c = self.config
        diagnostics = dict(search_scope='explicit_prior_basin', global_uniqueness_claimed=False)
        tested = 0

        def result(ok, reason, T=None, fitness=0., rmse=float('inf'), observability=0., **extra):
            return RegistrationResult(ok, reason, T, fitness, rmse, observability,
                                      time.monotonic() - started, tested, {**diagnostics, **extra})

        if T_prior is None:
            return result(False, 'tracking_requires_explicit_prior')
        prior = checked_transform(T_prior)
        raw = np.asarray(scan_lidar)
        if raw.ndim != 2 or raw.shape[1] != 3 or not len(raw) or not np.isfinite(raw).all():
            return result(False, 'invalid_scan')
        predicted = transform_points(raw, prior)
        # Crop only by fixed asset bounds, never by a fitted transform or NN inliers.
        covered = np.all((predicted >= self.minimum) & (predicted <= self.maximum), axis=1)
        coverage = float(covered.mean())
        diagnostics.update(map_volume_coverage=coverage, covered_input_points=int(covered.sum()),
                           input_points=len(raw))
        if coverage < .3 or covered.sum() < 150:
            return result(False, 'insufficient_map_volume_overlap')
        scan = bounded_sample(voxel_sample(raw[covered], c.voxel_size), c.max_scan_points)
        if len(scan) < 100:
            return result(False, 'insufficient_scan_geometry')
        fit, verify = scan[np.arange(len(scan)) % 4 != 0], scan[::4]
        coarse = prior
        for leaf, distance, points, tree in self.levels:
            cfg = replace(c, coarse_distance=distance,
                          correspondence_distance=min(distance, c.correspondence_distance), icp_iterations=12)
            coarse = _refine(voxel_sample(fit, leaf), points, coarse, cfg, tree)
        # Both seeds get plane refinement. Coarse voxel sampling cannot alone
        # win by moving a scan along a dominant floor's voxel lattice.
        proposals = [refine_planes(fit, self.target, self.tree, self.normals, self.planar, initial)
                     for initial in (prior, coarse)]
        verified = []
        failed = None
        for T in proposals:
            tested += 1
            shift, angle = pose_distance(T, prior)
            if shift > .5 or angle > .2:
                continue
            moved = transform_points(verify, T)
            d, ids = self.tree.query(moved, workers=1)
            good = d < c.correspondence_distance
            fitness = float(good.mean())
            rmse = float(np.sqrt(np.mean(d[good] ** 2))) if good.any() else float('inf')
            record = (fitness, rmse, T, 0.)
            if failed is None or (fitness, -rmse) > (failed[0], -failed[1]):
                failed = record
            if good.sum() < 50 or fitness < c.min_fitness or rmse > c.max_rmse:
                continue
            obs, spatial = _observability(moved[good], self.target, ids[good], self.tree)
            if obs < c.min_observability or spatial < c.min_spatial_eigenratio:
                continue
            if not any(pose_distance(T, old[2])[0] < .10 and pose_distance(T, old[2])[1] < .04
                       for old in verified):
                verified.append((fitness, rmse, T, obs))
        if not verified:
            return result(False, 'tracking_geometry_rejected', fitness=failed[0] if failed else 0.,
                          rmse=failed[1] if failed else float('inf'))
        verified.sort(key=lambda row: (-row[0], row[1]))
        best = verified[0]
        if any(other[0] >= best[0] - c.ambiguity_fitness_margin and
               other[1] <= best[1] + c.ambiguity_rmse_margin for other in verified[1:]):
            return result(False, 'ambiguous_tracking_geometry', fitness=best[0], rmse=best[1],
                          observability=best[3], distinct_candidates=len(verified))
        return result(True, 'prior_geometry_verified', best[2], best[0], best[1], best[3],
                      translation_m=pose_distance(best[2], prior)[0],
                      rotation_rad=pose_distance(best[2], prior)[1], distinct_candidates=len(verified))
