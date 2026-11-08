"""Preregistered arrival regions for the physical three-platform route.

The original point arrays remain the authoritative planner centers. Bounds
describe body-position arrival, not contact verification or free-space maps.
"""
import copy
import math

from embodied_navigation.navigation.mission.goal_regions import IDENTITY, RigidTransform, parse_goal

ROUTES = ('exploration', 'return_origin', 'navigation_f1_f3')


def add_route_regions(scenario, profile='three_platform_body_arrival_v2'):
    if profile not in ('three_platform_body_arrival_v1', 'three_platform_body_arrival_v2'):
        raise ValueError('unknown arrival profile')
    inner = profile == 'three_platform_body_arrival_v2'
    result = copy.deepcopy(scenario)
    result['goal_region_profile'] = {
        'schema_version': 2, 'name': profile,
        'dwell_sim_s': .4, 'max_gap_sim_s': .2,
        'flat_radius_m': .35, 'flat_height_half_span_m': .10,
        'ramp_half_extents_m': [.35, .30, .10], 'origin_radius_m': .22,
        'description': 'Preregistered body-position volumes; SCAN plans to unchanged centers. Surface IDs are metadata, not measured contacts.'}
    if inner:
        result['goal_region_profile']['control_band'] = {
            'flat_radius_m': .25, 'flat_height_half_span_m': .10,
            'ramp_half_extents_m': [.25, .20, .07], 'origin_radius_m': .17}
    result['route_goals'] = {}
    scale = math.sqrt(1.01)
    for name in ROUTES:
        goals = []
        for index, point in enumerate(result[name]):
            x, y, z = point
            arrival = {'type': 'disc_prism', 'radius_m': .35,
                       'height_half_span_m': .10, 'axes': IDENTITY,
                       'dwell_sim_s': .4}
            surface = 'floor_1' if z < .01 else 'floor_2' if abs(z-1.2) < .01 else 'floor_3'
            # Both ramp seams use the oriented corridor as well as its interior.
            slope = .1 if y == 2 and 2 <= x <= 14 and 0 <= z <= 1.2 else -.1 if y == 7 and 2 <= x <= 14 and 1.2 <= z <= 2.4 else None
            if slope is not None:
                surface = 'ramp_12' if slope > 0 else 'ramp_23'
                arrival = {'type': 'oriented_box', 'dwell_sim_s': .4,
                           'axes': [[1/scale, 0, slope/scale], [0, 1, 0], [-slope/scale, 0, 1/scale]],
                           'half_extents_m': [.35, .30, .10]}
            if name == 'return_origin' and index == len(result[name])-1:
                if point != [0, 0, 0]:
                    raise ValueError('return_origin must end at the initialization origin')
                surface = 'origin'
                arrival = {'type': 'sphere', 'radius_m': .22, 'dwell_sim_s': .4}
            if inner:
                arrival['control_band'] = ({'radius_m': .17} if surface == 'origin' else
                    {'half_extents_m': [.25, .20, .07]} if slope is not None else
                    {'radius_m': .25, 'height_half_span_m': .10})
            raw = {'goal_id': f'{name}:{index}', 'center': point, 'arrival': arrival,
                   'timeout_sim_s': 90., 'route_surface_id': surface}
            goals.append(parse_goal(raw).definition())
        result['route_goals'][name] = goals
    return result


def validate_route_regions(scenario):
    """Reject ambiguous/mismatched routes before snapshotting or moving."""
    if 'route_goals' not in scenario:
        return  # Historical point-only scenarios remain supported.
    configured = scenario['route_goals']
    if not isinstance(configured, dict) or set(configured) != set(ROUTES):
        raise ValueError('route_goals must define exactly all three mission routes')
    profile = scenario.get('goal_region_profile', {}).get('name')
    expected = add_route_regions({name: scenario[name] for name in ROUTES}, profile=profile)
    if scenario.get('goal_region_profile') != expected['goal_region_profile']:
        raise ValueError('arrival profile does not match its declared contract')
    for name in ROUTES:
        raw = configured[name]
        if not isinstance(raw, list) or not raw or len(raw) != len(scenario[name]):
            raise ValueError(name+' goal count differs from planner centers')
        goals = tuple(parse_goal(item) for item in raw)
        if len({g.goal_id for g in goals}) != len(goals):
            raise ValueError(name+' duplicate goal ID')
        for goal, point in zip(goals, scenario[name]):
            if tuple(point) != goal.center:
                raise ValueError(name+' arrival center differs from planner center')
        if [g.definition() for g in goals] != expected['route_goals'][name]:
            raise ValueError(name+' geometry differs from its declared profile')
    final = parse_goal(configured['return_origin'][-1])
    if final.center != (0., 0., 0.) or final.kind != 'sphere' or final.radius != .22:
        raise ValueError('return origin requires the preregistered .22 m sphere')


def transformed_route_goals(scenario, name, origin, heading_alignment):
    validate_route_regions(scenario)
    raw = scenario.get('route_goals', {}).get(name)
    if raw is None:
        return ()
    reference = scenario.get('waypoint_reference', 'relative_initial_body')
    if reference == 'relative_world_axes':
        from embodied_navigation.navigation.mission.goal_regions import relative_world_transform
        transform = relative_world_transform(origin, heading_alignment['yaw_camera_init_from_world'])
    else:
        transform = RigidTransform(IDENTITY, origin if reference == 'relative_initial_body' else (0., 0., 0.))
    return tuple(transform.goal(parse_goal(item)) for item in raw)
