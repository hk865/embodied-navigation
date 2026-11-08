"""Explicit two-tier simulation speed selection; no model or safety authority.

The original Student contract remains the base PID/model identity. This typed
extension changes the longitudinal total command bound, not PID gains or slew.
"""
import copy

KEY='r109_speed_tier'
SCHEMA='r109_simulation_speed_tier/v1'

def choice(cruise):
    if type(cruise) is not float or cruise not in (.2,.5):
        raise ValueError('Only explicit 0.2 or 0.5 m/s speed tiers are implemented')
    return dict(schema=SCHEMA,simulation_only=True,cruise_mps=cruise,
                student_longitudinal_limit_mps=.3 if cruise==.2 else .6,
                planning_horizon_m=.65)

def validate(value):
    if type(value) is not dict:
        raise ValueError('Typed R109 speed tier required')
    expected=choice(value.get('cruise_mps'))
    if value!=expected or any(type(value.get(k))is not type(v) for k,v in expected.items()):
        raise ValueError('R109 speed tier fields differ from one of the two frozen choices')
    return dict(expected)

def split_ack(value):
    if not isinstance(value,dict):return value,None
    if KEY not in value:return dict(value),None
    result=dict(value);selected=validate(result.pop(KEY))
    return result,selected

def configuration(profile):
    """Startup only: bind planner, desired reference and actual Actor/ACK tier."""
    ack=profile.get('cascade',{}).get('r85_student_lower_loop_ack')
    _,ack_tier=split_ack(ack)
    if KEY not in profile:
        if ack_tier is not None:raise ValueError('ACK speed tier lacks its selected profile')
        return None
    selected=validate(profile[KEY]);cascade=profile.get('cascade',{})
    if (profile.get('actor_kind')!='student' or profile.get('controller_kind')!='student'
            or type(profile.get('navigation_ground_truth_used'))is not bool
            or profile.get('max_speed_mps')!=selected['cruise_mps']
            or cascade.get('desired_speed')!=selected['cruise_mps']
            or cascade.get('command_limits')!=[.6,.2,.6]
            or cascade.get('command_slew_per_s')!=[.6,.6,.8]
            or profile.get('continuous_route_contract',{}).get('local_planning_horizon_m')!=selected['planning_horizon_m']
            or profile.get('max_lateral_speed_mps')!=.2 or profile.get('max_yaw_rate_radps')!=.6):
        raise ValueError('R109 planner/navigation/Student ACK speed selection is inconsistent')
    return selected

def student_bounds(selected):
    return [.3,.2,.3] if selected is None else [validate(selected)['student_longitudinal_limit_mps'],.2,.3]

def validate_copies(config,*records):
    _,selected=split_ack(config)
    for record in records:
        if not isinstance(record,dict):raise ValueError('Actual same-frame speed-tier record required')
        if selected is None:
            if KEY in record:raise ValueError('Unselected ACK cannot carry a speed-tier extension')
        elif validate(record.get(KEY))!=selected:
            raise ValueError('Actual PID/Actor/ACK speed tier differs or is missing')
    return selected

def select_profile(profile,cruise):
    """Copy an existing authorized run profile; never renew its time/resources."""
    selected=choice(cruise);result=copy.deepcopy(profile)
    result[KEY]=selected;result['max_speed_mps']=cruise
    result['cascade']['desired_speed']=cruise
    result['cascade']['r85_student_lower_loop_ack'][KEY]=dict(selected)
    result['continuous_route_contract']['local_planning_horizon_m']=selected['planning_horizon_m']
    configuration(result)
    return result
