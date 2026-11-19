"""Conservative geometric stair labels from one current, gravity-aligned cloud.

No unobserved surface is generated. Labels are geometric evidence, not foot
support, body clearance, semantic certainty, or permission to execute. The
caller owns source stamp, frame and generation validation.
"""
from __future__ import annotations

import numpy as np
from scipy.signal import find_peaks
from scipy.spatial import cKDTree


def detect_stair_voxels(points_xyz, body_position, *, resolution=.08):
    """Return observed stair voxels and a mask aligned with the input points.

    Only a 3 m XY / 2 m Z neighbourhood is fitted. Fitting uses one actual point
    per 3 cm voxel; mixed-cell rejection uses EVERY original local point. Three
    aligned treads plus two measured joining risers are required. Missing or
    ambiguous evidence yields no labels, never an extrapolated staircase.
    """
    points, body, local, original, output = _local_input(points_xyz, body_position, resolution)
    if len(original) < 100:
        return output
    _, first, inverse = np.unique(np.floor(original / .03).astype(np.int64), axis=0,
                                   return_index=True, return_inverse=True)
    c = original[first]
    normals = _normals(c)
    riser_normals = _normals(c, radius=.085)
    horizontal = abs(normals[:, 2]) > .94
    h = c[horizontal]
    if len(h) < 90:
        return output
    bins = np.arange(h[:, 2].min() - .02, h[:, 2].max() + .041, .02)
    counts, edges = np.histogram(h[:, 2], bins)
    peaks = find_peaks(counts, prominence=12, distance=4)[0]
    planes = []
    for peak in peaks:
        z = (edges[peak] + edges[peak + 1]) / 2
        mask = horizontal & (abs(c[:, 2] - z) < .025)
        p = c[mask]
        if len(p) < 30:
            continue
        values, vectors = np.linalg.eigh(np.cov(p[:, :2].T))
        # A tread has a resolved wide lateral dimension, not a tiny table/patch.
        if values[0] <= 0 or values[1] / values[0] < 2:
            continue
        planes.append(dict(z=float(np.median(p[:, 2])), center=np.mean(p[:, :2], axis=0),
                           axis=vectors[:, 0], mask=mask))
    for start in range(len(planes) - 2):
        trio = planes[start:start + 3]
        axis = trio[0]['axis'].copy()
        centers = np.array([p['center'] for p in trio])
        if (centers[-1] - centers[0]) @ axis < 0:
            axis *= -1
        if min(abs(np.array([p['axis'] for p in trio]) @ axis)) < .97:
            continue
        rise = np.diff([p['z'] for p in trio])
        projected_centers = np.array([np.median(c[p['mask'], :2] @ axis) for p in trio])
        run = np.diff(projected_centers)
        if not (np.all((rise >= .06) & (rise <= .22)) and np.ptp(rise) <= .025
                and np.all((run >= .20) & (run <= .60)) and np.ptp(run) <= .06):
            continue
        side = np.array([-axis[1], axis[0]])
        s, lateral = c[:, :2] @ axis, c[:, :2] @ side
        bounds = [np.quantile(np.column_stack([s[p['mask']], lateral[p['mask']]]), [.05, .95], axis=0)
                  for p in trio]
        if any(not (.12 <= b[1, 0] - b[0, 0] <= .65 and b[1, 1] - b[0, 1] >= .5) for b in bounds):
            continue
        lateral_low = max(b[0, 1] for b in bounds)
        lateral_high = min(b[1, 1] for b in bounds)
        if lateral_high - lateral_low < .5:
            continue
        risers = []
        for i in range(2):
            lower, upper = trio[i]['z'], trio[i + 1]['z']
            possible = ((c[:, 2] > lower + .025) & (c[:, 2] < upper - .025)
                        & (lateral >= lateral_low) & (lateral <= lateral_high)
                        & (abs(riser_normals[:, :2] @ axis) > .94)
                        & (s > projected_centers[i]) & (s < projected_centers[i + 1]))
            if possible.sum() < 10:
                break
            plane_s = float(np.median(s[possible]))
            fit = possible & (abs(s - plane_s) < .025)
            if fit.sum() < 10 or np.ptp(lateral[fit]) < .5:
                break
            risers.append(dict(s=plane_s, lower_z=lower, upper_z=upper,
                               inliers=int(fit.sum())))
        if len(risers) != 2 or abs((risers[1]['s'] - risers[0]['s']) - np.mean(run)) > .06:
            continue
        output['models'].append(dict(axis_xy=axis.tolist(), tread_z=[p['z'] for p in trio],
                                     rise_m=rise.tolist(), run_m=run.tolist(), risers=risers,
                                     tread_run_bounds_m=[[float(b[0, 0]), float(b[1, 0])] for b in bounds],
                                     lateral_bounds_m=[lateral_low, lateral_high]))
    return match_stair_voxels(points, body, output['models'], resolution=resolution)


def match_stair_voxels(points_xyz, body_position, models, *, resolution=.08):
    """Match ONLY current measured points to bounded, previously fitted models.

    Models do not create occupancy or labels without current observations. Both
    surface distance and current local normal must agree; boundaries are never
    extended. The caller must separately retain model and current-cloud stamps.
    """
    points, body, local, original, output = _local_input(points_xyz, body_position, resolution)
    output['models'] = models
    if len(original) < 6 or not models:
        return output
    _, first, inverse = np.unique(np.floor(original / .03).astype(np.int64), axis=0,
                                   return_index=True, return_inverse=True)
    c = original[first]
    horizontal = abs(_normals(c)[:, 2]) > .94
    riser_normals = _normals(c, radius=.085)
    labelled = np.zeros(len(original), dtype=bool)
    for model in models:
        axis = np.asarray(model['axis_xy'])
        side = np.array([-axis[1], axis[0]])
        so, lo = original[:, :2] @ axis, original[:, :2] @ side
        lateral_low, lateral_high = model['lateral_bounds_m']
        lateral_match = (lo >= lateral_low) & (lo <= lateral_high)
        for z, bound in zip(model['tread_z'], model['tread_run_bounds_m']):
            labelled |= ((abs(original[:, 2] - z) < .025) & horizontal[inverse]
                         & (so >= bound[0]) & (so <= bound[1]) & lateral_match)
        vertical = abs(riser_normals[:, :2] @ axis) > .94
        for r in model['risers']:
            labelled |= ((abs(so - r['s']) < .025) & vertical[inverse]
                         & (original[:, 2] >= r['lower_z'])
                         & (original[:, 2] <= r['upper_z']) & lateral_match)
    indices, inv = np.unique(np.floor(original / resolution).astype(np.int64), axis=0, return_inverse=True)
    all_match = np.ones(len(indices), dtype=bool)
    np.logical_and.at(all_match, inv, labelled)
    # Only cells whose entire observed content matches are labels. Do not silently
    # truncate, because a partial snapshot could erase other active labels.
    # A cell crossing the crop boundary could hide unmatched supplied points.
    # Require its entire finite cube to lie inside the fitted neighbourhood.
    lower = indices * resolution
    upper = lower + resolution
    far_xy = np.maximum(abs(lower[:, :2] - body[:2]), abs(upper[:, :2] - body[:2]))
    inside = (np.linalg.norm(far_xy, axis=1) <= 3.) & (lower[:, 2] >= body[2] - 2.) & (upper[:, 2] <= body[2] + 2.)
    output['voxel_indices'] = indices[all_match & inside]
    output['point_stair_mask'][local] = labelled
    return output



def _local_input(points_xyz, body_position, resolution):
    # External sensor arrays may be malformed, nonfinite, or in an invalid shape.
    points = np.asarray(points_xyz, dtype=float)
    body = np.asarray(body_position, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('points_xyz must be finite N x 3 measured points')
    if body.shape != (3,) or not np.isfinite(body).all():
        raise ValueError('body_position must be finite XYZ')
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError('resolution must be positive and finite')
    local = (np.linalg.norm(points[:, :2] - body[:2], axis=1) <= 3.) & (abs(points[:, 2] - body[2]) <= 2.)
    original = points[local]
    output = dict(voxel_indices=np.empty((0, 3), dtype=np.int64),
                  point_stair_mask=np.zeros(len(points), dtype=bool), models=[],
                  observed_point_count=len(original))
    return points, body, local, original, output


def _normals(c, radius=.12):
    """Batched PCA of at most 24 neighbours; no unbounded radius-list loops."""
    if len(c) < 6:
        return np.zeros_like(c)
    distances, indices = cKDTree(c).query(c, k=min(24, len(c)), distance_upper_bound=radius)
    valid = np.isfinite(distances)
    neighbours = c[np.minimum(indices, len(c) - 1)]
    count = valid.sum(axis=1)
    means = (neighbours * valid[:, :, None]).sum(axis=1) / np.maximum(count[:, None], 1)
    delta = (neighbours - means[:, None, :]) * valid[:, :, None]
    covariance = np.einsum('nki,nkj->nij', delta, delta) / np.maximum(count[:, None, None], 1)
    values, vectors = np.linalg.eigh(covariance)
    planar = (count >= 6) & (values[:, 0] / np.maximum(values.sum(axis=1), 1e-12) < .04)
    return vectors[:, :, 0] * planar[:, None]
