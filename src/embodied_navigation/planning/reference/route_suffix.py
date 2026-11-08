"""Deterministic PCT transport suffix from actual corrected SLAM, without ROS.

This selects a position on the immutable ordered 3D guide. It does not search
for a new route, move the measured start, grant physical permission, or change
the original terminal goal. All following source vertices are retained.
"""
import hashlib
import json
import math


def _xyz(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in value)):
        raise ValueError('r85_resume_finite_xyz_required')
    return list(map(float, value))


def _sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def select_route_suffix(body_points, ground_points, actual_pose):
    """Closest 3D segment projection, followed by the exact ordered suffix.

    Adjacent equal minima share a route corner; the earlier edge wins. A
    nonadjacent tied minimum is ambiguous and fails instead of guessing a
    branch. The 1e-12 squared-distance allowance is numerical, not a route
    corridor or a freshness/permission threshold.
    """
    if (not isinstance(body_points, list) or not 2 <= len(body_points) <= 4096
            or not isinstance(ground_points, list) or len(ground_points) != len(body_points)):
        raise ValueError('r85_resume_route_point_budget')
    body, ground = [_xyz(p) for p in body_points], [_xyz(p) for p in ground_points]
    actual = _xyz(actual_pose)
    if any(max(abs(b[k]-g[k]-(.32 if k == 2 else 0.)) for k in range(3)) > 1e-8
           for b, g in zip(body, ground)):
        raise ValueError('r85_resume_body_ground_offset_mismatch')
    candidates, arc = [], 0.0
    for i, (a, b) in enumerate(zip(body, body[1:])):
        delta = [b[k]-a[k] for k in range(3)]
        length_sq = sum(v*v for v in delta)
        if not math.isfinite(length_sq) or length_sq <= 1e-12:
            raise ValueError('r85_resume_duplicate_or_invalid_segment')
        fraction = max(0., min(1., sum((actual[k]-a[k])*delta[k] for k in range(3))/length_sq))
        # Preserve exact source endpoints instead of recomputing their floats.
        projected = list(a) if fraction == 0. else list(b) if fraction == 1. else [a[k]+fraction*delta[k] for k in range(3)]
        projected_ground = (list(ground[i]) if fraction == 0. else list(ground[i+1]) if fraction == 1.
                            else [ground[i][k]+fraction*(ground[i+1][k]-ground[i][k]) for k in range(3)])
        distance_sq = sum((actual[k]-projected[k])**2 for k in range(3))
        if not math.isfinite(distance_sq):
            raise ValueError('r85_resume_projection_nonfinite')
        candidates.append((distance_sq, i, fraction, projected, projected_ground, arc+fraction*math.sqrt(length_sq)))
        arc += math.sqrt(length_sq)
    best = min(candidates, key=lambda row: (row[0], row[1]))
    if any(abs(row[0]-best[0]) <= 1e-12 and abs(row[1]-best[1]) > 1 for row in candidates):
        raise ValueError('r85_resume_projection_ambiguous_route_branches')
    distance_sq, edge, fraction, projected, projected_ground, progress = best
    first_remaining = edge+1
    if fraction == 1.:
        first_remaining += 1
    selected_body = [projected]+body[first_remaining:]
    selected_ground = [projected_ground]+ground[first_remaining:]
    # Match the existing wire's 1e-6 coincident-start exclusion here, before
    # certifying its first transmitted target. Otherwise the SCAN manager would
    # prepend actual==projection and create a zero-duration first segment.
    coincident_prefix = 0
    while selected_body and math.dist(selected_body[0], actual) <= 1e-6:
        selected_body.pop(0); selected_ground.pop(0); coincident_prefix += 1
    if not selected_body:
        raise ValueError('r85_resume_route_already_at_terminal_point')
    receipt = dict(schema='r85_actual_route_suffix/v1', coordinate_frame='camera_init',
        selector='nearest_ordered_3d_polyline_projection', source_body_points_sha256=_sha(body),
        source_ground_points_sha256=_sha(ground), source_point_count=len(body),
        actual_start_body_xyz=actual, source_edge_index=edge, source_edge_fraction=fraction,
        source_progress_m=progress, projection_distance_m=math.sqrt(distance_sq),
        projected_body_xyz=projected, projected_ground_xyz=projected_ground,
        retained_source_vertex_start_index=first_remaining+max(0, coincident_prefix-1), selected_point_count=len(selected_body),
        coincident_start_points_not_transmitted=coincident_prefix,
        selected_body_points_sha256=_sha(selected_body), selected_ground_points_sha256=_sha(selected_ground),
        source_terminal_body_xyz=body[-1], terminal_body_xyz=selected_body[-1],
        all_following_source_vertices_retained=(coincident_prefix <= 1), actual_pose_changed=False,
        only_existing_coincident_start_exclusion_applied=True,
        new_route_search=False, navigation_ground_truth_used=False, physical_permission_granted=False)
    return dict(body_points=selected_body, ground_points=selected_ground, receipt=receipt)
