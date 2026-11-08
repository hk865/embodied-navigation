"""Explicit R16 deployment extension: coordinated yaw I release, no stop authority.

The event hash is identity, not proof of DDS publication. Original accepted
source, actual publication, causal ACK, and physical stop gates remain owners.
"""
import copy
import hashlib
import json
import math

KEY='pre_zero_integral_unload'
PROFILE_KEY='r85_pre_zero_integral_unload'
KIND='coordinated_yaw_toward_zero_v1'
CONFIG=dict(enabled=True,kind=KIND,teacher_rate_integral_per_s=.1,
    student_rate_integral_per_s=.04,maneuver_timeout_s=10.)
EVENT_SCHEMA='r85_owned_pre_zero_integral_unload_event/v1'
RECEIPT_SCHEMA='r85_pre_zero_yaw_integral_update/v1'

def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def valid_sha(value):
    return isinstance(value,str) and len(value)==64 and all(c in '0123456789abcdef'for c in value)

def configuration(profile):
    value=profile.get(PROFILE_KEY)
    if value is None:return None
    if value=={'enabled':False} and type(value['enabled'])is bool:return None
    if (not isinstance(value,dict) or set(value)!=set(CONFIG) or value!=CONFIG
            or type(value['enabled'])is not bool
            or any(type(value[k])not in (int,float)for k in CONFIG if k not in ('enabled','kind'))):
        raise ValueError('Exact explicit coordinated unload extension configuration required')
    cascade=profile.get('cascade',{})
    if (profile.get('r85_active_turn_braking')!={'enabled':True}
            or profile.get('r85_quiet_handoff_target')!={'enabled':True}
            or cascade.get('external_heading_gate')is not True
            or cascade.get('r85_turn_brake_reference')!={'kind':'locked_heading_pd_v1'}):
        raise ValueError('Coordinated unload requires original active locked-heading quiet braking')
    return copy.deepcopy(value)

def make_event(run_id,scope,handoff,path_sha256,goal_sha256,zero_count,previous_zero):
    event=dict(schema=EVENT_SCHEMA,kind=KIND,run_id=run_id,scope_sha256=scope,
        context=copy.deepcopy(handoff['context']),purpose=handoff['purpose'],
        brake_entry_clock_ns=round(handoff['brake_entry_control_s']*1e9),
        original_deadline_clock_ns=round(handoff['deadline_s']*1e9),
        initial_path_sha256=path_sha256,goal_sha256=goal_sha256,entry_zero_handoff_count=zero_count,previous_zero_identity=copy.deepcopy(previous_zero))
    event['expires_clock_ns']=min(event['brake_entry_clock_ns']+10_000_000_000,event['original_deadline_clock_ns'])
    event['event_id']=digest(event)
    return validate_event(event,run_id,scope)

def validate_event(value,run_id=None,scope=None,clock_ns=None):
    e=copy.deepcopy(value)
    keys={'schema','kind','run_id','scope_sha256','context','purpose','brake_entry_clock_ns',
        'original_deadline_clock_ns','initial_path_sha256','goal_sha256','expires_clock_ns','event_id','entry_zero_handoff_count','previous_zero_identity'}
    if not isinstance(e,dict) or set(e)!=keys:raise ValueError('Owned unload event shape differs')
    claimed=e.pop('event_id')
    if not valid_sha(claimed) or claimed!=digest(e):raise ValueError('Owned unload event identity differs')
    e['event_id']=claimed;c=e['context']
    if (e['schema']!=EVENT_SCHEMA or e['kind']!=KIND or not isinstance(e['run_id'],str)or not e['run_id']
            or not all(valid_sha(e[k])for k in ('scope_sha256','initial_path_sha256','goal_sha256'))
            or not isinstance(c,dict) or set(c)!={'request_id','waypoint_index','dynamic_recovery','dynamic_event_sequence'}
            or not isinstance(c['request_id'],str) or not c['request_id'].startswith(e['run_id']+':')
            or type(c['waypoint_index'])is not int or c['waypoint_index']<0
            or type(c['dynamic_recovery'])is not bool
            or (not c['dynamic_recovery'] and c['dynamic_event_sequence']is not None)
            or (c['dynamic_recovery'] and c['dynamic_event_sequence']is not None
                and (type(c['dynamic_event_sequence'])is not int or c['dynamic_event_sequence']<=0))
            or (c['dynamic_recovery'] and e['purpose']!='dynamic_recovery')
            or not isinstance(e['purpose'],str)or not e['purpose']
            or any(type(e[k])is not int or e[k]<0 for k in ('brake_entry_clock_ns','expires_clock_ns','original_deadline_clock_ns'))
            or e['original_deadline_clock_ns']<e['brake_entry_clock_ns']
            or e['expires_clock_ns']!=min(e['brake_entry_clock_ns']+10_000_000_000,e['original_deadline_clock_ns'])):
        raise ValueError('Owned unload event binding invalid')
    count=e['entry_zero_handoff_count'];previous=e['previous_zero_identity']
    if type(count)is not int or count<0:raise ValueError('Unload edge count invalid')
    if count==0:
        if previous is not None:raise ValueError('First owned event has a previous zero')
    elif (not isinstance(previous,dict)or set(previous)!={'event_id','zero_control_clock_ns','zero_handoff_count'}
            or not valid_sha(previous['event_id'])or type(previous['zero_control_clock_ns'])is not int
            or not 0<=previous['zero_control_clock_ns']<e['brake_entry_clock_ns']
            or type(previous['zero_handoff_count'])is not int or previous['zero_handoff_count']!=count):
        raise ValueError('Previous owned zero identity invalid')
    if run_id is not None and e['run_id']!=run_id:raise ValueError('Unload run differs')
    if scope is not None and e['scope_sha256']!=scope:raise ValueError('Unload scope differs')
    if clock_ns is not None and not e['brake_entry_clock_ns']<=clock_ns<e['expires_clock_ns']:
        raise ValueError('Prezero integral unload maneuver expired or future')
    return e

def successor(previous,current):
    old=validate_event(previous);new=validate_event(current);zero=new['previous_zero_identity']
    return bool(new['run_id']==old['run_id']and new['scope_sha256']==old['scope_sha256']
        and new['entry_zero_handoff_count']==old['entry_zero_handoff_count']+1
        and zero is not None and zero['event_id']==old['event_id']
        and old['brake_entry_clock_ns']<=zero['zero_control_clock_ns']<new['brake_entry_clock_ns'])

def toward_zero(value,rate,dt):
    if not all(math.isfinite(float(v))for v in (value,rate,dt))or rate<=0 or dt<=0:
        raise ValueError('Finite positive release rate/dt required')
    return math.copysign(max(0.,abs(float(value))-float(rate)*float(dt)),float(value)) if value else 0.

def receipt(event,side,before,after,dt,clock_ns,path_sha256,goal_sha256,input_frame_serial=None,source_control_clock_ns=None):
    source_clock=clock_ns if side=='teacher'else source_control_clock_ns
    e=validate_event(event,clock_ns=source_clock);rate=.1 if side=='teacher'else .04
    r=dict(schema=RECEIPT_SCHEMA,kind=KIND,event=e,side=side,
        yaw_integral_before=float(before),yaw_integral_after=float(after),rate_integral_per_s=rate,
        dt_s=float(dt),input_clock_ns=clock_ns,input_frame_serial=input_frame_serial,source_control_clock_ns=source_clock,
        path_sha256=path_sha256,goal_sha256=goal_sha256,exact_yaw_I_zero=float(after)==0.,
        error_accumulation_enabled=False,new_deployment_math=True)
    validate_receipt(r);return r

def validate_receipt(r,event=None,side=None,scope=None,clock_ns=None,frame=None):
    keys={'schema','kind','event','side','yaw_integral_before','yaw_integral_after','rate_integral_per_s','dt_s',
        'input_clock_ns','input_frame_serial','source_control_clock_ns','path_sha256','goal_sha256','exact_yaw_I_zero',
        'error_accumulation_enabled','new_deployment_math'}
    if not isinstance(r,dict)or set(r)!=keys:raise ValueError('Actual unload recurrence receipt missing')
    if type(r['source_control_clock_ns'])is not int:raise ValueError('Actual accepted Teacher control clock missing')
    e=validate_event(r['event'],scope=scope,clock_ns=r['source_control_clock_ns'])
    rate=.1 if r['side']=='teacher'else .04 if r['side']=='student'else None
    if (r['schema']!=RECEIPT_SCHEMA or r['kind']!=KIND or rate is None
            or r['rate_integral_per_s']!=rate or r['new_deployment_math']is not True
            or r['error_accumulation_enabled']is not False
            or any(type(r[k])not in (int,float)or not math.isfinite(r[k])for k in ('yaw_integral_before','yaw_integral_after','dt_s'))
            or type(r['input_clock_ns'])is not int or type(r['source_control_clock_ns'])is not int
            or not 0<=r['input_clock_ns']-r['source_control_clock_ns']<=300_000_000
            or (r['side']=='teacher' and r['source_control_clock_ns']!=r['input_clock_ns'])
            or r['yaw_integral_after']!=toward_zero(r['yaw_integral_before'],rate,r['dt_s'])
            or type(r['exact_yaw_I_zero'])is not bool or r['exact_yaw_I_zero']!=(r['yaw_integral_after']==0.)
            or not valid_sha(r['path_sha256'])or r['goal_sha256']!=e['goal_sha256']):
        raise ValueError('Actual unload recurrence/binding differs')
    if not 0<r['dt_s']<=.3:raise ValueError('Original integral header gap exceeded in unload receipt')
    if r['side']=='student'and not .019999<=r['dt_s']<=.020001:raise ValueError('Unload receipt is not original 50Hz PID dt')
    if r['side']=='student'and(type(r['input_frame_serial'])is not int or r['input_frame_serial']<=0):
        raise ValueError('Actual Student unload frame missing')
    if r['side']=='teacher'and r['input_frame_serial']is not None:raise ValueError('Teacher receipt has native frame')
    if event is not None and e!=validate_event(event):raise ValueError('Unload actual event differs')
    if side is not None and r['side']!=side:raise ValueError('Unload receipt side differs')
    if clock_ns is not None and r['input_clock_ns']!=clock_ns:raise ValueError('Unload actual input clock differs')
    if frame is not None and r['input_frame_serial']!=frame:raise ValueError('Unload actual frame differs')
    return r

def zero_observation(scope,clock,frame,dt,actual_I):
    r=dict(schema='r85_pre_zero_yaw_integral_observation/v1',kind='original_exact_zero_I_observation',
        unload_applied=False,native_event_authority=False,run_id=scope,
        input_clock_ns=clock,input_frame_serial=frame,dt_s=dt,actual_yaw_integral=float(actual_I))
    validate_native_extension(r,scope,clock,frame);return r

def validate_native_extension(r,scope,clock,frame):
    if isinstance(r,dict)and r.get('kind')=='original_exact_zero_I_observation':
        keys={'schema','kind','unload_applied','native_event_authority','run_id',
            'input_clock_ns','input_frame_serial','dt_s','actual_yaw_integral'}
        if (set(r)!=keys or r['schema']!='r85_pre_zero_yaw_integral_observation/v1'
                or r['unload_applied']is not False or r['native_event_authority']is not False
                or not valid_sha(scope)or r['run_id']!=scope
                or type(r['input_clock_ns'])is not int or r['input_clock_ns']!=clock
                or type(r['input_frame_serial'])is not int or frame<=0 or r['input_frame_serial']!=frame
                or type(r['dt_s'])not in (int,float)or not .019999<=r['dt_s']<=.020001
                or type(r['actual_yaw_integral'])not in (int,float)or r['actual_yaw_integral']!=0.):
            raise ValueError('Actual original-zero integral observation differs')
        return r
    return validate_receipt(r,side='student',scope=scope,clock_ns=clock,frame=frame)

class NativeUnload:
    def __init__(self,scope,scope_sha,run_id):
        self.config=configuration(scope['profile']);self.scope_sha=scope_sha;self.run_id=run_id;self.event=None
    def authorize(self,evidence,request,clock_ns):
        if self.config is None:return None
        if (evidence or {}).get('rejected')is not False or not any(request):return None
        envelope=evidence.get('actual_original_envelope')or{};source=envelope.get('controller_source')or{}
        if source.get('heading_transition_stage')!='controlled_braking':return None
        source_clock=source.get('control_stamp_ns')
        pose_clock=source.get('source_pose_stamp_ns');imu_clock=source.get('paired_imu_stamp_ns')
        if (any(type(v)is not int for v in (source_clock,pose_clock,imu_clock))
                or not 0<=clock_ns-pose_clock<=300_000_000
                or not 0<=pose_clock-imu_clock<=20_000_000
                or not pose_clock<=source_clock<=clock_ns
                or evidence.get('physics_clock_ns')!=clock_ns
                or (evidence.get('actual_read_attempt')or{}).get('read_clock_ns')!=clock_ns):
            raise ValueError('Original accepted unload source/input clock differs or expired')
        # Teacher owns its fixed10s deadline. A still-fresh accepted old frame
        # may arrive after the formal zero edge; never re-date it to native time.
        e=validate_event(source.get(KEY),self.run_id,self.scope_sha,source_clock)
        if (envelope.get('healthy')is not True or envelope.get('stop_requested')is not False
                or envelope.get('acceptance',{}).get('sha256')!=self.scope_sha
                or source.get('cascade_mode')!='brake' or type(source.get('heading_zero_handoff_count'))is not int
                or source['heading_zero_handoff_count']!=e['entry_zero_handoff_count']
                or source.get('heading_transition_context')!=e['context']
                or (source.get('request_id'),source.get('waypoint_index'))!=(e['context']['request_id'],e['context']['waypoint_index'])
                or source.get('goal_sha256')!=e['goal_sha256']or not valid_sha(source.get('path_sha256'))
                or list(source.get('command_after_slew',[]))!=list(request)
                or (evidence.get('actual_read_attempt')or{}).get('status')!='accepted'):
            raise ValueError('Accepted unload source context/frame differs')
        if self.event is not None and e!=self.event and not successor(self.event,e):raise ValueError('Unload pending event may never renew')
        self.event=e;return dict(event=e,path_sha256=source['path_sha256'],goal_sha256=source['goal_sha256'],source_control_clock_ns=source_clock)
