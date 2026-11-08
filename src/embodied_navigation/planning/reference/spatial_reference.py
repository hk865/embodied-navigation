"""Auditable spatial directions on an immutable checked polyline.

The chord is a heading estimate, NEVER a replacement collision-checked path.
The caller must check its actual commanded swept motion against obstacles and
terrain. All distances are horizontal arc length; projection stays in 3D.
"""
import math
import numpy as np


DEFAULTS = dict(enabled=True, arc_length_m=.2, derivative_step_m=.05,
    maximum_chord_deviation_m=.03, minimum_chord_ratio=.5,
    curvature_feedforward_enabled=False, curvature_speed_limit_enabled=False,
    yaw_rate_reserve_radps=.12, yaw_acceleration_reserve_radps2=.2,
    yaw_command_slew_radps2=.8)


def configuration(value=None):
    if value is not None and not isinstance(value, dict):
        raise ValueError('spatial_reference must be a dictionary')
    value = {} if value is None else value
    if set(value)-set(DEFAULTS):
        raise ValueError('Unknown spatial reference configuration')
    result = {**DEFAULTS, **value}
    for name in ('enabled', 'curvature_feedforward_enabled', 'curvature_speed_limit_enabled'):
        if type(result[name]) is not bool:
            raise ValueError('Spatial reference feature flags must be boolean')
    for name in set(DEFAULTS)-{'enabled', 'curvature_feedforward_enabled', 'curvature_speed_limit_enabled'}:
        if isinstance(result[name], bool) or not isinstance(result[name], (int, float)):
            raise ValueError('Spatial reference limits must be finite numbers')
        result[name] = float(result[name])
        if not math.isfinite(result[name]) or result[name] <= 0:
            raise ValueError('Spatial reference limits must be positive and finite')
    if not .01 <= result['arc_length_m'] <= 1.:
        raise ValueError('Finite arc reference must be between .01 and 1 metre')
    if result['derivative_step_m'] > result['arc_length_m']:
        raise ValueError('Curvature difference step must not exceed reference arc')
    if not 0 < result['minimum_chord_ratio'] <= 1:
        raise ValueError('Invalid chord cancellation boundary')
    if result['yaw_acceleration_reserve_radps2'] >= result['yaw_command_slew_radps2']:
        raise ValueError('Yaw acceleration reserve consumes the complete slew budget')
    if not result['enabled'] and (result['curvature_feedforward_enabled'] or result['curvature_speed_limit_enabled']):
        raise ValueError('Curvature features require the spatial reference')
    return result


def point_at(points, cumulative, s):
    s = float(np.clip(s, 0., cumulative[-1]))
    index = min(int(np.searchsorted(cumulative, s, side='right'))-1, len(points)-2)
    index = max(0, index)
    fraction = (s-cumulative[index])/(cumulative[index+1]-cumulative[index])
    return points[index]+fraction*(points[index+1]-points[index])


def _direction(points, cumulative, s, config):
    start = float(np.clip(s, 0., cumulative[-1]))
    end = min(float(cumulative[-1]), start+config['arc_length_m'])
    index = min(max(0, int(np.searchsorted(cumulative, start, side='right'))-1), len(points)-2)
    raw = points[index+1,:2]-points[index,:2]
    raw = raw/np.linalg.norm(raw)
    p0, p1 = point_at(points, cumulative, start), point_at(points, cumulative, end)
    chord = p1[:2]-p0[:2]
    length = float(np.linalg.norm(chord))
    arc = end-start
    inside = points[(cumulative > start) & (cumulative < end)]
    samples = np.vstack([p0, inside, p1])
    deviation = 0.
    if length > 1e-12:
        direction = chord/length
        along = np.clip((samples[:,:2]-p0[:2])@direction, 0., length)
        deviation = float(np.max(np.linalg.norm(samples[:,:2]-p0[:2]-along[:,None]*direction, axis=1)))
    else:
        direction = raw
    reason = None
    if arc <= 1e-8:
        reason = 'path_tail_no_forward_arc'
    elif length/max(arc,1e-12) < config['minimum_chord_ratio']:
        reason = 'forward_chord_cancellation'
    elif deviation > config['maximum_chord_deviation_m']:
        reason = 'corner_chord_deviation'
    if reason:
        direction = raw
    return dict(tangent_xy=direction.tolist(), heading_rad=math.atan2(direction[1],direction[0]),
        start_s_m=start, end_s_m=end, arc_used_m=arc, chord_length_m=length,
        chord_deviation_m=deviation, chord_start_xyz=p0.tolist(), chord_end_xyz=p1.tolist(),
        fallback_reason=reason, curvature_valid=reason is None,
        chord_is_collision_checked_path=False, original_path_replaced=False)


def reference(points, cumulative, s, position, projected_point, raw_tangent, config):
    """Direction and matching normal/cross error, with unchanged 3D projection."""
    result = _direction(points, cumulative, s, config)
    if not config['enabled'] or result['fallback_reason'] is not None:
        result.update(tangent_xy=np.asarray(raw_tangent).tolist(),
            heading_rad=math.atan2(raw_tangent[1],raw_tangent[0]),
            fallback_reason=result['fallback_reason'] if config['enabled'] else 'spatial_reference_disabled', curvature_valid=False)
    tangent = np.asarray(result['tangent_xy'])
    normal = np.array([-tangent[1], tangent[0]])
    result.update(normal_xy=normal.tolist(),
        cross_ref_m=float((np.asarray(position)[:2]-np.asarray(projected_point)[:2])@normal),
        curvature_per_m=0., curvature_rate_per_m2=0.)
    step = config['derivative_step_m']
    lo, hi = max(0., s-step), min(float(cumulative[-1]), s+step)
    left, right = _direction(points,cumulative,lo,config), _direction(points,cumulative,hi,config)
    if result['curvature_valid'] and left['curvature_valid'] and right['curvature_valid'] and hi-lo>1e-8:
        wrap = lambda value: math.atan2(math.sin(value),math.cos(value))
        result['curvature_per_m'] = wrap(right['heading_rad']-left['heading_rad'])/(hi-lo)
        if s-lo>1e-8 and hi-s>1e-8:
            kl = wrap(result['heading_rad']-left['heading_rad'])/(s-lo)
            kr = wrap(right['heading_rad']-result['heading_rad'])/(hi-s)
            result['curvature_rate_per_m2'] = 2.*(kr-kl)/(hi-lo)
    else:
        result['curvature_valid'] = False
    return result


def speed_budget(speed, curvature, curvature_rate, yaw_rate_limit, feedback_yaw,
                 previous_speed, dt, config, previous_yaw_reference=None):
    """Conservative reference budget; does not certify plant acceleration.

    |k|*v <= yaw headroom; |k'|*v^2 + |k|*|dv/dt| <= yaw slew headroom.
    If no speed satisfies the discrete reference budget, return an explicit
    infeasible stop request. Downstream slew and braking guards still apply.
    """
    # Controller limits/filter arithmetic can supply NumPy scalar values.
    # Normalize at the numerical boundary: feasibility is also consumed by
    # the caller's `is False` safety branch, not just by JSON diagnostics.
    speed, curvature, curvature_rate, yaw_rate_limit, feedback_yaw, dt = map(
        float, (speed, curvature, curvature_rate, yaw_rate_limit, feedback_yaw, dt))
    previous_speed = None if previous_speed is None else float(previous_speed)
    previous_yaw_reference = None if previous_yaw_reference is None else float(previous_yaw_reference)
    rate_budget = max(0.,yaw_rate_limit-abs(feedback_yaw)-float(config['yaw_rate_reserve_radps']))
    accel_budget = float(config['yaw_command_slew_radps2'])-float(config['yaw_acceleration_reserve_radps2'])
    k, dk = abs(float(curvature)), abs(float(curvature_rate))
    # Solve only inside the finite requested interval. Compare products
    # first; divide by nonzero curvature only when its boundary cuts it.
    cap = max(0.,float(speed))
    if k*cap > rate_budget:
        cap = rate_budget/k
    if dk*cap*cap > accel_budget:
        cap = math.sqrt(accel_budget/dk)
    lower_bound=0.
    temporal_checked=previous_yaw_reference is not None
    temporal_feasible=True
    if temporal_checked:
        width=accel_budget*max(0.,dt)
        qlo,qhi=previous_yaw_reference-width,previous_yaw_reference+width
        endpoint=curvature*cap
        if max(0.,endpoint)<qlo or min(0.,endpoint)>qhi:
            temporal_feasible=False
        elif curvature>0.:
            if qlo>0.:lower_bound=qlo/curvature
            if endpoint>qhi:cap=qhi/curvature
        elif curvature<0.:
            if qhi<0.:lower_bound=qhi/curvature
            if endpoint<qlo:cap=qlo/curvature
        # Exact zero is the constant affine function, not an epsilon class.
        temporal_feasible=temporal_feasible and cap>=lower_bound and cap>=0.
        if not temporal_feasible:cap=0.
    acceleration_checked = previous_speed is not None and dt>0.
    feasible = temporal_feasible
    if acceleration_checked and feasible:
        # Multiply the original acceleration inequality by its real dt.
        # This avoids k/dt and does not change any physical budget.
        D=dk*dt;budget=accel_budget*dt
        def demand(v):return D*v*v+k*abs(v-previous_speed)
        minimizer=min(cap,max(lower_bound,min(previous_speed,k/(2.*D)))) if D>0. else min(cap,max(lower_bound,previous_speed))
        if demand(minimizer)>budget+1e-12*dt:
            feasible=False;cap=0.
        elif demand(cap)>budget:
            low,high=minimizer,cap
            for _ in range(50):
                middle=(low+high)*.5
                if demand(middle)<=budget:low=middle
                else:high=middle
            cap=low
    # Round boundary quotients inward if their floating products overshoot.
    if feasible:
        for _ in range(4):
            valid=(k*cap<=rate_budget and dk*cap*cap<=accel_budget and
                (not temporal_checked or abs(curvature*cap-previous_yaw_reference)<=accel_budget*max(0.,dt)) and
                (not acceleration_checked or demand(cap)<=budget))
            if valid:break
            inward=math.nextafter(cap,lower_bound)
            if inward==cap:break
            cap=inward
        if not valid:
            feasible=False;cap=0.
    return float(cap),dict(enabled=True,requested_speed_mps=float(speed),limited_speed_mps=float(cap),
        yaw_rate_budget_radps=float(rate_budget),yaw_acceleration_budget_radps2=float(accel_budget),
        reference_acceleration_checked=bool(acceleration_checked),temporal_yaw_slew_checked=bool(temporal_checked),
        previous_curvature_yaw_reference_radps=previous_yaw_reference,
        reference_budget_feasible=bool(feasible),
        physical_response_verified=False)
