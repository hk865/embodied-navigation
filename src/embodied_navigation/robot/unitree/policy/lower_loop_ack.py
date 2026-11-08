"""Pure R85 Student lower-loop ACK contract, without model/ROS/clock access.

The truthful upstream request and actual Actor input remain in the ACK. Only
the intentional, already correction-bounded Student PID contribution is removed
from the outer antiwindup residual. Total command clipping and worker slew stay.
"""
import copy
import hashlib
import json
import importlib.util
from pathlib import Path
import numpy as np
import embodied_navigation.robot.unitree.policy.student_xy_stop_takeover_ack as xy_stop_ack
import embodied_navigation.robot.unitree.policy.speed_tiers as speed_tiers

SCHEMA = 'r85_student_lower_loop_ack/v1'
CONTRACT_SHA = 'fedc24a112b1ec599f54d086af074af4fc925ff3a4e2e1d10cfbecec372300dd'
EXPORT_SHA = 'd8b51361ada74a80beb5e64f892447ea406d136be4c76a6ab15bae2d9b6c6185'
CHECKPOINT_SHA = '7b59b8c7eafc9ffbc2dc32aaa2df3dadec3269d039338e4f93217bc73af66fd9'
PID_SCHEMA = 'student_velocity_pid_motion_epoch_zero_reset/v2'
CONFIG = dict(enabled=True, schema=SCHEMA)
MEASUREMENT_CONFIG = dict(CONFIG, measurement_p_retention_required=True)
MEASUREMENT_KEY = 'student_yaw_measurement_p_retention'
MEASUREMENT_REQUIRED = MEASUREMENT_KEY+'_required'
_measurement_module = None


def measurement_helper(extension):
    import embodied_navigation.robot.unitree.policy.yaw_measurement_feedback as student_yaw_measurement_feedback
    return student_yaw_measurement_feedback


def measurement_extension(first,second,scope,stamp,frame,unload,zero,requested,pid=None):
    """Bind optional new-law proof; it grants no new ACK or parking authority."""
    present=any(k in obj for obj in(first,second)for k in(MEASUREMENT_KEY,MEASUREMENT_REQUIRED))
    if not present:return None
    if (first.get(MEASUREMENT_REQUIRED)is not True or second.get(MEASUREMENT_REQUIRED)is not True
            or first.get(MEASUREMENT_KEY)!=second.get(MEASUREMENT_KEY)):
        raise ValueError('Measurement P actual PID/receipt/ledger proof differs or missing')
    q=first.get(MEASUREMENT_KEY);fade=first.get('student_yaw_pd_fade')
    validate_yaw_pd_fade(fade,scope,stamp,frame,unload,zero)
    return measurement_helper(q).validate_receipt(q,fade,unload,scope,stamp,frame,zero,requested,pid)


def configuration(value):
    if value is None:
        return None
    value,speed_tier=speed_tiers.split_ack(value)
    value,xy_stop_enabled=xy_stop_ack.split_configuration(value)
    if (not isinstance(value,dict)or type(value.get('enabled'))is not bool
            or not(value==CONFIG or value==MEASUREMENT_CONFIG
                and value.get('measurement_p_retention_required')is True)):
        raise ValueError('Explicit frozen R85 Student lower-loop ACK configuration required')
    selected=copy.deepcopy(value)
    if xy_stop_enabled:selected[xy_stop_ack.FLAG]=True
    if speed_tier is not None:selected[speed_tiers.KEY]=speed_tier
    return selected


def require_measurement_proof(config,first,second):
    """Trusted selected profile forbids downgrade to an optional legacy path."""
    if xy_stop_ack.split_configuration(speed_tiers.split_ack(configuration(config))[0])[0]==MEASUREMENT_CONFIG:
        if any(not isinstance(obj,dict)or obj.get(MEASUREMENT_REQUIRED)is not True
                or not isinstance(obj.get(MEASUREMENT_KEY),dict)for obj in(first,second)):
            raise ValueError('Selected measurement P ACK contract requires both actual proofs')


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def integer(value, positive=False):
    if type(value) is not int or value < int(positive):
        raise ValueError('Original nonnegative integer ACK binding required')
    return value


def vector(value):
    out = np.asarray(value, dtype=float)
    if out.shape != (3,) or not np.isfinite(out).all():
        raise ValueError('Finite lower-loop three-vector required')
    return out


def same(a, b, tolerance=1e-12):
    if not np.allclose(vector(a), vector(b), rtol=0., atol=tolerance):
        raise ValueError('Same-frame lower-loop command binding differs')


def effective_limits(navigation_limits,config=None):
    limits = vector(navigation_limits)
    if np.any(limits <= 0.):
        raise ValueError('Positive original navigation limits required')
    _,speed_tier=speed_tiers.split_ack(config)
    return np.minimum(limits, speed_tiers.student_bounds(speed_tier))


def input_binding(ack):
    if ack.get('actor_kind')!='student':raise ValueError('Actual Student ACK required')
    receipt = ack.get('actual_actor_input')
    if not isinstance(receipt, dict):
        raise ValueError('Actual Student inference receipt missing')
    for key in ('actor_inferred_this_frame', 'exact_native45_used', 'action_finite', 'hidden_finite'):
        if receipt.get(key) is not True:
            raise ValueError('Actual Student inference not established')
    if (receipt.get('actor_kind') != 'student' or receipt.get('actor_observation_dimension') != 45
        or receipt.get('teacher247_used_for_student_inference') is not False
):
        raise ValueError('Actual Student observation identity differs')
    stamp = integer(ack['stamp_ns'])
    if integer(receipt['input_clock_ns']) != stamp:
        raise ValueError('ACK and actual Student input clock differ')
    frame = integer(receipt['frame_serial'], True)
    inference = integer(receipt['inference_sequence'], True)
    applied = vector(ack['applied_command_body'])
    if not np.array_equal(np.asarray(applied, np.float32),
                          np.asarray(vector(receipt['actual_observation_command_body']), np.float32)):
        raise ValueError('ACK applied input differs from actual Student observation')
    scope = ack.get('run_id', receipt.get('run_id'))
    return receipt, stamp, frame, inference, applied, scope


def validate_yaw_pd_fade(extension, scope, stamp, frame, unload, zero, pid=None):
    """Bind the optional new-law diagnostic; alpha grants no parking authority."""
    keys={'schema','enabled','new_control_math','alpha_before','alpha','target_alpha',
        'maximum_alpha_rate_per_s','dt_s','active_current_owned_event','reason','event_id',
        'event_expires_clock_ns','event_validated_at_source_clock','source_control_clock_ns',
        'run_id','scope_sha256','motion_epoch','input_clock_ns','input_frame_serial',
        'original_exact_zero_priority','PID_I_or_D_state_modified_by_helper','stop_or_ACK_credit',
        'base_yaw_Kp','base_yaw_Kd','effective_yaw_Kp','effective_yaw_Kd','Ki_unchanged','base_PID_math_claim'}
    q=extension
    if not isinstance(q,dict) or set(q)!=keys:
        raise ValueError('Typed same-frame yaw PD fade diagnostic required')
    numeric=['alpha_before','alpha','target_alpha','maximum_alpha_rate_per_s','dt_s',
        'base_yaw_Kp','base_yaw_Kd','effective_yaw_Kp','effective_yaw_Kd','Ki_unchanged']
    if (any(type(q[k])not in(int,float)or not np.isfinite(q[k])for k in numeric)
        or q['schema']!='student_yaw_PD_fade_candidate/v1' or q['enabled']is not True
        or q['new_control_math']is not True or q['base_PID_math_claim']is not False
        or q['PID_I_or_D_state_modified_by_helper']is not False or q['stop_or_ACK_credit']is not False
        or type(q['original_exact_zero_priority'])is not bool or q['original_exact_zero_priority']!=zero
        or q['scope_sha256']!=scope or type(q['input_clock_ns'])is not int or q['input_clock_ns']!=stamp
        or type(q['input_frame_serial'])is not int or q['input_frame_serial']!=frame
        or not isinstance(q['run_id'],str)or not q['run_id']
        or not .019999<=q['dt_s']<=.020001 or q['maximum_alpha_rate_per_s']!=1.
        or not 0.<=q['alpha_before']<=1. or not 0.<=q['alpha']<=1.
        or type(q['active_current_owned_event'])is not bool
        or type(q['event_validated_at_source_clock'])is not bool
        or q['base_yaw_Kp']!=.5 or q['base_yaw_Kd']!=.015 or q['Ki_unchanged']!=.15
        or q['effective_yaw_Kp']!=.5*q['alpha'] or q['effective_yaw_Kd']!=.015*q['alpha']):
        raise ValueError('Actual yaw PD fade frame/scope/gains/no-credit differs')
    active=q['active_current_owned_event'];target=0. if active else 1.
    expected=max(target,q['alpha_before']-q['dt_s'])if q['alpha_before']>target else min(target,q['alpha_before']+q['dt_s'])
    if q['target_alpha']!=target or q['alpha']!=expected:
        raise ValueError('Yaw PD fade bounded recurrence differs')
    if q['event_id'] is None:
        if (active or q['event_expires_clock_ns']is not None or q['source_control_clock_ns']is not None
            or q['event_validated_at_source_clock']is not False
            or q['reason']!=('original_exact_zero_priority_restore'if zero else'no_owned_event_restore')):
            raise ValueError('Yaw PD restore must not invent an event')
    else:
        from embodied_navigation.control.parking.integral_unload import validate_native_extension
        original=validate_native_extension(unload,scope,stamp,frame)
        event=original.get('event')
        if (zero or not isinstance(event,dict)or q['event_validated_at_source_clock']is not True
            or q['event_id']!=event['event_id']or q['run_id']!=event['run_id']
            or q['event_expires_clock_ns']!=event['expires_clock_ns']
            or type(q['source_control_clock_ns'])is not int
            or q['source_control_clock_ns']!=original['source_control_clock_ns']
            or q['motion_epoch']!=event['context']['request_id']+':waypoint:'+str(event['context']['waypoint_index'])
            or q['dt_s']!=original['dt_s']
            or active!=(event['brake_entry_clock_ns']<=stamp<event['expires_clock_ns'])
            or q['reason']!=('active_owned_event_fade'if active else'accepted_late_source_restore_only')):
            raise ValueError('Yaw PD fade and original accepted owned event differ')
    if pid is not None:
        if q['dt_s']!=pid['dt_s']or q['motion_epoch']!=pid['motion_epoch']:
            raise ValueError('Yaw PD fade and actual PID clock/epoch differ')
        if MEASUREMENT_KEY in pid or MEASUREMENT_REQUIRED in pid:
            if pid.get(MEASUREMENT_REQUIRED)is not True:
                raise ValueError('Required measurement P proof missing')
            measurement_helper(pid.get(MEASUREMENT_KEY)).validate_receipt(
                pid[MEASUREMENT_KEY],q,unload,scope,stamp,frame,zero,pid['requested_command_body'],pid)
        else:
            expected_P=0. if zero else q['effective_yaw_Kp']*vector(pid['pid_error'])[2]
            if abs(vector(pid['pid_P'])[2]-expected_P)>1e-12:
                raise ValueError('Yaw PD fade does not match actual PID P contribution')
    return q


def annotate_pending(pending, pid, config, navigation_limits):
    """Called only after telemetry binds this inference/action/input frame."""
    selected=configuration(config)
    receipt, stamp, frame, inference, applied, scope = input_binding(pending)
    request = vector(pending['requested_command_body'])
    if (not isinstance(pid, dict) or pid.get('schema') != PID_SCHEMA
        or integer(pid['actual_input_clock_ns']) != stamp
        or integer(pid['actual_frame_serial'], True) != frame):
        raise ValueError('PID annotation is not the same actual Student frame')
    require_measurement_proof(config,pid,receipt)
    same(pid['requested_command_body'], request)
    same(pid['final_after_legacy_slew_body'], applied)
    speed_tier=speed_tiers.validate_copies(selected,pid,receipt)
    limits = effective_limits(navigation_limits,selected)
    same(pid['effective_command_bounds'], limits)
    correction = vector(pid['pid_correction'])
    same(correction, np.clip(vector(pid['pid_raw_correction']), [-.08, -.1, -.08], [.08, .1, .08]))
    before_limit = request + correction
    corrected = vector(pid['corrected_before_legacy_slew_body'])
    same(corrected, np.clip(before_limit, -limits, limits))
    if np.any(abs(applied) > limits + 1e-10):
        raise ValueError('Actual Student input exceeds frozen limits')
    zero, takeover = pid.get('authoritative_zero'), pid.get('ordinary_zero_takeover')
    if type(zero) is not bool or zero != (not bool(np.any(request))) or type(takeover) is not bool:
        raise ValueError('Explicit exact-zero authority differs from actual request')
    if zero:
        same(correction, [0., 0., 0.])
    if takeover:
        if not zero or np.any(applied):
            raise ValueError('Ordinary zero takeover did not produce exact zero')
    annotation = dict(schema=SCHEMA, actor_kind='student', input_clock_ns=stamp,
        input_frame_serial=frame, inference_sequence=inference, run_id=scope,
        requested_command_body=request.tolist(), intentional_bounded_pid_correction_body=correction.tolist(),
        policy_input_before_command_limit_body=before_limit.tolist(),
        corrected_before_worker_slew_body=corrected.tolist(), actual_applied_policy_input_body=applied.tolist(),
        effective_command_bounds_body=limits.tolist(), authoritative_exact_zero=zero,
        ordinary_zero_takeover=takeover,
        total_command_limit_residual_body=(before_limit-corrected).tolist(),
        worker_slew_residual_body=(corrected-applied).tolist(),
        downstream_limiter_slew_residual_body=(before_limit-applied).tolist())
    if 'pre_zero_integral_unload'in pid or 'pre_zero_integral_unload'in receipt:
        from embodied_navigation.control.parking.integral_unload import KEY, validate_native_extension
        extension=pid.get(KEY)
        validate_native_extension(extension,scope,stamp,frame)
        observed=extension.get('kind')=='original_exact_zero_I_observation'
        actual_I=extension['actual_yaw_integral']if observed else extension['yaw_integral_after']
        if (observed and (not zero or vector(pid['pid_integral'])[2]!=0.)
                or receipt.get(KEY)!=extension or vector(pid['pid_integral'])[2]!=actual_I
                or extension['dt_s']!=pid['dt_s']):
            raise ValueError('Unload receipt does not match actual PID and Student frame')
        annotation[KEY]=copy.deepcopy(extension)
    if 'student_yaw_pd_fade'in pid or 'student_yaw_pd_fade'in receipt:
        extension=pid.get('student_yaw_pd_fade')
        validate_yaw_pd_fade(extension,scope,stamp,frame,receipt.get('pre_zero_integral_unload'),zero,pid)
        if extension!=receipt.get('student_yaw_pd_fade'):
            raise ValueError('Yaw PD fade PID and actual input receipt differ')
        annotation['student_yaw_pd_fade']=copy.deepcopy(extension)
    extension=measurement_extension(pid,receipt,scope,stamp,frame,receipt.get('pre_zero_integral_unload'),zero,request,pid)
    if extension is not None:
        annotation[MEASUREMENT_KEY]=copy.deepcopy(extension)
        annotation[MEASUREMENT_REQUIRED]=True
    xy_extension=xy_stop_ack.validate_copies(pid,receipt,selected,scope,stamp,frame,
        request,applied,corrected,correction,pid=pid)
    if xy_extension is not None:
        worker_slew,override=xy_stop_ack.split_residual(corrected,applied,xy_extension)
        annotation[xy_stop_ack.KEY]=copy.deepcopy(xy_extension)
        annotation[xy_stop_ack.REQUIRED]=True
        annotation['worker_slew_residual_body']=worker_slew.tolist()
        annotation['xy_stop_takeover_residual_body']=override.tolist()
    if speed_tier is not None:annotation[speed_tiers.KEY]=dict(speed_tier)
    pending['r85_student_lower_loop_ack'] = annotation
    return annotation


def bind_causal_ack(ack, config):
    """Next native request binds the already captured prior inference ledger."""
    configuration(config)
    annotation = ack.get('r85_student_lower_loop_ack')
    if not isinstance(annotation, dict) or 'ledger_sha256' in annotation:
        raise ValueError('Unbound same-frame Student annotation required')
    seq, native = integer(ack['sequence'], True), integer(ack['ack_native_clock_ns'])
    if native-integer(ack['stamp_ns']) != 20_000_000:
        raise ValueError('Student ACK must follow by one actual 50Hz native request')
    annotation.update(ack_sequence=seq, ack_native_clock_ns=native)
    annotation['ledger_sha256'] = digest(annotation)
    return annotation


def residual(ack, config, navigation_limits):
    """Validate same-frame ledger and return actual limiter/slew residual only.

    Existing outer caller still owns causal pose checks, 300ms freshness and
    sequence monotonicity. No fallback is allowed for the enabled contract.
    """
    selected=configuration(config)
    receipt, stamp, frame, inference, applied, scope = input_binding(ack)
    annotation = ack.get('r85_student_lower_loop_ack')
    if not isinstance(annotation, dict):
        raise ValueError('Required Student lower-loop ACK ledger missing')
    note = copy.deepcopy(annotation)
    claimed = note.pop('ledger_sha256', None)
    if claimed != digest(note):
        raise ValueError('Student lower-loop ACK ledger checksum differs')
    require_measurement_proof(config,note,receipt)
    speed_tier=speed_tiers.validate_copies(selected,note,receipt)
    bindings = dict(schema=SCHEMA, actor_kind='student', input_clock_ns=stamp,
        input_frame_serial=frame, inference_sequence=inference, run_id=scope,
        ack_sequence=integer(ack['sequence'], True), ack_native_clock_ns=integer(ack['ack_native_clock_ns']))
    if any(note.get(k) != v or type(note.get(k)) is not type(v) for k, v in bindings.items()):
        raise ValueError('Student lower-loop ACK frame/sequence/source binding differs')
    if bindings['ack_native_clock_ns']-stamp != 20_000_000:
        raise ValueError('Student lower-loop ACK causal native interval differs')
    request, limits = vector(ack['requested_command_body']), effective_limits(navigation_limits,selected)
    same(note['requested_command_body'], request)
    same(note['actual_applied_policy_input_body'], applied)
    same(note['effective_command_bounds_body'], limits)
    correction = vector(note['intentional_bounded_pid_correction_body'])
    if np.any(abs(correction) > [.080000000001, .100000000001, .080000000001]):
        raise ValueError('Student intentional correction exceeds fixed B bounds')
    before_limit = request+correction
    corrected = np.clip(before_limit, -limits, limits)
    same(note['policy_input_before_command_limit_body'], before_limit)
    same(note['corrected_before_worker_slew_body'], corrected)
    if np.any(abs(applied) > limits + 1e-10):
        raise ValueError('Actual Student input exceeds fixed B total limits')
    zero, takeover = note.get('authoritative_exact_zero'), note.get('ordinary_zero_takeover')
    if type(zero) is not bool or zero != (not bool(np.any(request))) or type(takeover) is not bool:
        raise ValueError('Student lower-loop exact-zero authority differs')
    if zero:
        same(correction, [0., 0., 0.])
    if takeover and (not zero or np.any(applied)):
        raise ValueError('Student ordinary-zero ledger is not actual exact zero')
    xy_extension=xy_stop_ack.validate_copies(note,receipt,selected,scope,stamp,frame,
        request,applied,corrected,correction)
    slew,override=xy_stop_ack.split_residual(corrected,applied,xy_extension)
    total,out = before_limit-corrected,before_limit-applied
    if xy_extension is not None:
        same(note.get('xy_stop_takeover_residual_body'),override)
    elif 'xy_stop_takeover_residual_body'in note:
        raise ValueError('Disabled original ACK cannot carry an override residual')
    same(note['total_command_limit_residual_body'], total)
    same(note['worker_slew_residual_body'], slew)
    same(note['downstream_limiter_slew_residual_body'], out)
    if 'pre_zero_integral_unload'in note or 'pre_zero_integral_unload'in receipt:
        from embodied_navigation.control.parking.integral_unload import KEY, validate_native_extension
        validate_native_extension(note.get(KEY),scope,stamp,frame)
        if (note.get(KEY)!=receipt.get(KEY)or note[KEY].get('kind')=='original_exact_zero_I_observation'and not zero):
            raise ValueError('Unload ACK ledger and actual input receipt differ')
    if 'student_yaw_pd_fade'in note or 'student_yaw_pd_fade'in receipt:
        extension=note.get('student_yaw_pd_fade')
        validate_yaw_pd_fade(extension,scope,stamp,frame,note.get('pre_zero_integral_unload'),zero)
        if extension!=receipt.get('student_yaw_pd_fade'):
            raise ValueError('Yaw PD fade ACK ledger and actual input receipt differ')
    extension=measurement_extension(note,receipt,scope,stamp,frame,note.get('pre_zero_integral_unload'),zero,request)
    diagnostic = dict(schema=SCHEMA, ledger_sha256=claimed, ack_sequence=bindings['ack_sequence'],
        input_clock_ns=stamp, ack_native_clock_ns=bindings['ack_native_clock_ns'],
        input_frame_serial=frame, inference_sequence=inference,
        truthful_requested_minus_applied_body=(request-applied).tolist(),
        intentional_bounded_pid_correction_body=correction.tolist(),
        total_command_limit_residual_body=total.tolist(), worker_slew_residual_body=slew.tolist(),
        normalized_downstream_limiter_slew_residual_body=out.tolist(),
        ordinary_zero_takeover=takeover)
    if 'pre_zero_integral_unload'in note:diagnostic['pre_zero_integral_unload']=copy.deepcopy(note['pre_zero_integral_unload'])
    if 'student_yaw_pd_fade'in note:diagnostic['student_yaw_pd_fade']=copy.deepcopy(note['student_yaw_pd_fade'])
    if extension is not None:
        diagnostic[MEASUREMENT_KEY]=copy.deepcopy(extension)
        diagnostic[MEASUREMENT_REQUIRED]=True
    if xy_extension is not None:
        diagnostic[xy_stop_ack.KEY]=copy.deepcopy(xy_extension)
        diagnostic[xy_stop_ack.REQUIRED]=True
        diagnostic['xy_stop_takeover_residual_body']=override.tolist()
    return out, diagnostic
