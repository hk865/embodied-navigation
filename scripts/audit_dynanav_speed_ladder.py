#!/usr/bin/env python3
"""Offline speed-rung evidence, never an experiment launcher or guard editor.

Run after capture/recorders stop. Physical speeds use native post-step velocity
weighted over the preceding physics step and selected actual-command holds.
The 95% forward-command selection is a reporting selection, not an acceptance
threshold. Partial/GT-controlled runs are never promoted to full SLAM results.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime,timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

HELPER_PATH=Path(__file__).with_name('summarize_dynanav_experiment.py')
spec=importlib.util.spec_from_file_location('dynanav_speed_rung_summary_helpers',HELPER_PATH)
helpers=importlib.util.module_from_spec(spec);spec.loader.exec_module(helpers)
CRUISE_FRACTION=.95


def object_file(path):
    if not path.is_file():return {}
    try:
        value=json.loads(path.read_text())
        return value if isinstance(value,dict) else {}
    except ValueError:return {}


def evidence(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
    return {'path':str(path.resolve()),'sha256':digest.hexdigest(),'size_bytes':path.stat().st_size}


def resolve_capture(path):
    path=Path(path).expanduser().resolve(strict=True)
    if (path/'capture').is_dir():return path/'capture',path
    if path.is_dir():return path,path.parent if path.name=='capture' else path
    raise ValueError('run must be a capture directory or experiment directory')


def target_speed(result,experiment):
    values=[]
    for name,value in [('capture.experiment.speed_mps',experiment.get('speed_mps')),
                        ('result.experiment.speed_mps',(result.get('experiment') or {}).get('speed_mps'))]:
        if value is not None:
            if not helpers.number(value) or value<=0:raise ValueError(f'invalid configured target: {name}')
            values.append((name,float(value)))
    if not values:return None,{'status':'missing','sources':[]}
    if any(abs(value-values[0][1])>1e-12 for _,value in values):
        raise ValueError('conflicting configured target speeds; do not infer a rung from command maxima')
    return values[0][1],{'status':'recorded_configuration','sources':values}


def selected_metrics(truth,commands,result,predicate):
    # Retain every original command timestamp; non-selected rows terminate holds.
    masked=[{'measured_time':row['measured_time'],
             'actual_command':row['actual_command'] if predicate(row) else [0.,0.,0.]}
            for row in commands]
    return helpers.speed_metrics(truth,masked,result)


def entire_native_metrics(truth,result):
    """Whole recorded physical interval, including startup/zero commands/parking."""
    rows,invalid=helpers.ordered(truth,'time',lambda row:helpers.vector(row.get('velocity')))
    period,source=helpers.period_from(rows,'time',result.get('physics_dt'))
    xy,xyz,previous,gaps=[],[],None,0
    first_start=None
    for row in rows:
        end=row['time'];start=previous
        if previous is None:
            start=end-period if period is not None else end
        elif period is not None and end-previous>1.5*period:
            start=end-period;gaps+=1
        previous=end
        weight=end-start
        if weight<=0:continue
        if first_start is None:first_start=start
        velocity=row['velocity']
        xy.append((math.hypot(*velocity[:2]),weight))
        xyz.append((math.sqrt(sum(value*value for value in velocity)),weight))
    return {'selection':'entire available native journal, including startup, zero-command holds, turns and parking',
            'weighting':'post-step velocity over preceding measured physics interval; gaps capped at one expected step',
            'native_records':len(rows),'invalid_native_records':invalid,'physics_period_s':period,
            'physics_period_source':source,'native_gap_count':gaps,
            'recorded_interval_s':[first_start,rows[-1]['time']] if rows else None,
            'measured_duration_s':sum(weight for _,weight in xy) if xy else None,
            'xy':helpers.weighted_stats(xy),'three_dimensional':helpers.weighted_stats(xyz)}


def failure_evidence(result,supervision,run):
    route=result.get('route_tracker') or {};terminal=result.get('navigation_terminal_request') or {}
    error=result.get('error');reported=route.get('failure') or terminal.get('failure')
    text=str(error or '')+' '+str(reported or '')
    if 'Joint speed exceeds' in text:category='recorded_joint_speed_guard'
    elif any(value in text.lower() for value in ['tilt','roll','pitch']):category='recorded_attitude_guard_or_error'
    elif 'physical overspeed' in text.lower() or 'speed safety' in text.lower():category='recorded_physical_speed_guard'
    elif reported:category='recorded_route_failure'
    elif error:category='recorded_capture_error'
    elif supervision.get('success') is False:category='supervisor_reported_failure_without_capture_error'
    elif result.get('end_reason')=='navigation_terminal_and_parked':category='reported_arrived_and_parked'
    else:category='incomplete_or_unclassified_evidence'
    actuator=helpers.actuator_metrics(result)
    return {'category':category,'capture_error':error,'route_failure':reported,
            'end_reason':result.get('end_reason'),'capture_stage':result.get('stage'),
            'supervision_success':supervision.get('success'),'exit_codes':supervision.get('exit_codes'),
            'actuator_failure':actuator,'joint_speed_peak':result.get('joint_speed_peak'),
            'last_state_before_guard':result.get('last_state_before_guard'),
            'attribution_scope':'Recorded assertions/exit evidence only. A near-limit readback does not establish gross motor instability; missing .6 evidence is not a failure. Native process crash is not a speed-capability failure by itself.'}


def audit_run(path):
    capture,run=resolve_capture(path)
    paths=[capture/name for name in ['result.json','experiment.json','truth_scoring_stream.jsonl',
           'policy_frames.jsonl','slam_route_control.jsonl']]
    paths += [run/'supervision.json',run/'livo/observer/launch.json']
    existing=[path for path in paths if path.is_file()]
    before={path:(path.stat().st_size,path.stat().st_mtime_ns) for path in existing}
    result=object_file(capture/'result.json');experiment=object_file(capture/'experiment.json')
    if not experiment:experiment=result.get('experiment') or {}
    supervision=object_file(run/'supervision.json');launch=object_file(run/'livo/observer/launch.json')
    target,target_provenance=target_speed(result,experiment)
    summary=helpers.summarize(capture)
    truth,truth_evidence=helpers.read_stream(capture/'truth_scoring_stream.jsonl')
    commands,command_evidence=helpers.read_stream(capture/'policy_frames.jsonl')
    controls,control_evidence=helpers.read_stream(capture/'slam_route_control.jsonl')
    commands,invalid_commands=helpers.ordered(commands,'measured_time',lambda r:helpers.vector(r.get('actual_command')))
    valid_truth,invalid_truth=helpers.ordered(truth,'time',lambda r:helpers.vector(r.get('velocity')))
    cruise=(selected_metrics(valid_truth,commands,result,lambda row:row['actual_command'][0]>=CRUISE_FRACTION*target)
            if target is not None else None)
    if cruise:
        cruise['selection']=f'actual_command[0] >= {CRUISE_FRACTION} * recorded target {target}; original held-command boundaries retained'
        cruise['command_selection_threshold_mps']=CRUISE_FRACTION*target
        cruise['selected_command_records']=sum(row['actual_command'][0]>=CRUISE_FRACTION*target for row in commands)
    stage_by_time={};bad_controls=0
    for row in controls:
        stamp,stage=row.get('current_sim_s'),row.get('stage')
        if not helpers.number(stamp) or type(stage) is not int or stage<0 or stamp in stage_by_time:
            bad_controls+=1;continue
        stage_by_time[stamp]=stage
    paired=sum(row['measured_time'] in stage_by_time for row in commands)
    by_stage={}
    for stage in sorted(set(stage_by_time.values())):
        by_stage[str(stage)]=selected_metrics(valid_truth,commands,result,
            lambda row,stage=stage:stage_by_time.get(row['measured_time'])==stage)
    tracker=result.get('route_tracker') or {};events=tracker.get('events') or []
    event_counts=Counter(str(event.get('event')) for event in events if isinstance(event,dict))
    observed_stages=sorted(set(stage_by_time.values()))
    zero_tail=None
    if commands and valid_truth and not any(commands[-1]['actual_command']):
        start=len(commands)-1
        while start>0 and not any(commands[start-1]['actual_command']):start-=1
        left=commands[start]['measured_time'];right=valid_truth[-1]['time']
        tail_commands=[{'measured_time':row['measured_time'],
                        'actual_command':[1.,0.,0.] if row['measured_time']>=left else [0.,0.,0.]}
                       for row in commands]
        measured=helpers.speed_metrics(valid_truth,tail_commands,result)
        zero_tail={'interval_available_s':[left,right],'native_motion':measured,
                   'scope':'Synthetic selection mask only; no motion/command was generated. These measured tail velocities are descriptive, not a new parking acceptance threshold.'}
    endpoint=None
    positions,_=helpers.ordered(truth,'time',lambda row:helpers.vector(row.get('position')))
    goals=experiment.get('waypoints_xyz')
    if positions and isinstance(goals,list) and goals and helpers.vector(goals[-1]):
        delta=[a-b for a,b in zip(positions[-1]['position'],goals[-1])]
        endpoint={'native_world_base_position_m':positions[-1]['position'],'configured_world_base_goal_m':goals[-1],
                  'source_time_s':positions[-1]['time'],'delta_xyz_m':delta,'XY_error_m':math.hypot(*delta[:2]),
                  'z_error_m':delta[2],'three_dimensional_error_m':math.sqrt(sum(x*x for x in delta)),
                  'scope':'GT only for offline scoring; do not compare these world/base coordinates directly with estimated map/IMU goals.'}
    params=(result.get('slam_route_control') or {}).get('parameters') or {}
    policy=result.get('policy') or {}
    bounds=params.get('command_bounds') or {}
    comparable={'route':experiment.get('route'),'control_mode':experiment.get('control_mode'),
                'ground_truth_navigation':result.get('ground_truth_navigation'),'SLAM_control_enabled':result.get('SLAM_control_enabled'),
                'pose_source':experiment.get('slam_pose_source'),'reference_sha256':experiment.get('slam_route_reference_sha256'),
                'core_sha256':launch.get('core_sha256'),'runner_sha256':launch.get('runner_sha256'),
                'main_parameter_sha256':launch.get('parameter_sha256'),'camera_config_sha256':launch.get('camera_config_sha256'),
                'capture_calibration_sha256':launch.get('capture_calibration_sha256'),
                'physics_dt':result.get('physics_dt'),'policy_hz':(result.get('policy') or {}).get('policy_hz'),
                'policy_model_sha256':policy.get('model_sha256'),'policy_actor_kind':policy.get('actor_kind'),
                'policy_inference_device':policy.get('inference_device'),'policy_command_scale':policy.get('command_scale'),
                'policy_action_scale':policy.get('action_scale'),'policy_action_clip':policy.get('action_clip'),
                'motor_envelope':policy.get('motor_envelope'),'existing_native_COM_velocity_PID':policy.get('existing_native_COM_velocity_PID'),
                'imu_publish_hz':experiment.get('imu_publish_hz'),'render_backend':experiment.get('render_backend'),
                'periodic_python_stack_dump':result.get('periodic_python_stack_dump'),
                'joint_speed_readback_tolerance_radps':experiment.get('joint_speed_readback_tolerance_radps'),
                'minimum_base_z_m':experiment.get('minimum_base_z_m'),'maximum_displacement_m':experiment.get('maximum_displacement_m'),
                'body_contact_report_only':experiment.get('body_contact_report_only'),
                'stair_speed_cap_enabled':result.get('stair_speed_cap_enabled'),
                'stair_specific_speed_limit':experiment.get('stair_specific_speed_limit'),
                'lateral_command_bound_mps':bounds.get('lateral_mps'),'yaw_command_bound_rps':bounds.get('yaw_rate_rps'),
                'semantic_provider':experiment.get('semantic_provider'),'speed_source':params.get('speed_source'),
                'pose_speed_window_s':params.get('pose_speed_window_s'),
                'capture_remaining_m':params.get('capture_remaining_m'),'capture_distance_m':params.get('capture_distance_m'),
                'final_capture_remaining_m':params.get('final_capture_remaining_m'),'final_capture_distance_m':params.get('final_capture_distance_m'),
                'max_sim_age_s':params.get('max_sim_age_s'),'max_wall_age_s':params.get('max_wall_age_s'),
                'motion_end_s':experiment.get('motion_end_s'),'duration_s':experiment.get('duration_s')}
    files=[evidence(file) for file in existing]
    for file,stat in before.items():
        if (file.stat().st_size,file.stat().st_mtime_ns)!=stat:
            raise ValueError(f'input changed during offline audit; stop/finalize the run first: {file}')
    return {'run_directory':str(run),'capture_directory':str(capture),'target_command_mps':target,
        'target_provenance':target_provenance,'configured_target_is_not_measured_speed':True,
        'source_files':files,'stream_validation':{'truth':truth_evidence,'commands':command_evidence,'controls':control_evidence,
            'invalid_ordered_truth':invalid_truth,'invalid_ordered_commands':invalid_commands,'invalid_or_duplicate_controls':bad_controls,
            'command_control_exact_timestamp_pairs':paired},
        'entire_native_journal':entire_native_metrics(truth,result),
        'all_nonzero_actual_command':summary['commanded_motion'],'cruise_actual_forward_command_selection':cruise,
        'per_stage_nonzero_actual_command':by_stage,
        'actual_command_component_ranges':{axis:{'min':min(row['actual_command'][i] for row in commands),
            'max':max(row['actual_command'][i] for row in commands)} for i,axis in enumerate(['forward_mps','lateral_mps','yaw_rate_rps'])} if commands else None,
        'navigation':summary['navigation'],'observed_stages_from_control_journal':observed_stages,
        'route_event_counts':dict(event_counts),'route_events':events,'zero_actual_command_tail':zero_tail,
        'reported_full_three_stage_measured_closedloop_completion':(result.get('ground_truth_navigation') is False
            and result.get('SLAM_control_enabled') is True and observed_stages==[0,1,2]
            and event_counts.get('turn_completed')==2 and summary['navigation']['arrival_reported'] is True
            and summary['navigation']['stopped_and_parked_reported'] is True
            and bool(commands) and paired==len(commands) and bad_controls==invalid_commands==invalid_truth==0
            and truth_evidence['status']==command_evidence['status']==control_evidence['status']=='ok'),
        'completion_scope':'Source-reported route structure and physical tail metrics; independent source-time dwell/native pose integrity review remains separate.',
        'final_native_world_base_goal_error_scoring_only':endpoint,
        'failure_evidence':failure_evidence(result,supervision,run),
        'guard_and_policy_metadata':{'physical_speed_guard_mps':experiment.get('physical_speed_guard_mps'),
            'joint_speed_readback_tolerance_radps':experiment.get('joint_speed_readback_tolerance_radps'),
            'minimum_base_z_m':experiment.get('minimum_base_z_m'),'maximum_displacement_m':experiment.get('maximum_displacement_m'),
            'stair_speed_cap_enabled':result.get('stair_speed_cap_enabled'),
            'stair_specific_speed_limit':experiment.get('stair_specific_speed_limit'),
            'body_contact_report_only':experiment.get('body_contact_report_only'),'policy':result.get('policy'),
            'joint_speed_peak':result.get('joint_speed_peak')},
        'comparison_signature_without_target_speed':comparable}


def compare_signatures(audits):
    signatures=[audit['comparison_signature_without_target_speed'] for audit in audits]
    keys=sorted(set().union(*(set(value) for value in signatures)))
    differences={key:[value.get(key) for value in signatures] for key in keys
                 if any(value.get(key)!=signatures[0].get(key) for value in signatures[1:])}
    missing={key:[index for index,value in enumerate(signatures) if value.get(key) is None] for key in keys
             if any(value.get(key) is None for value in signatures)}
    return {'different_recorded_configuration_fields':differences,'missing_configuration_fields':missing,
            'comparability_not_automatically_assumed':True,
            'scope':'Target speed deliberately excluded. GT-controlled first-flight evidence is not a same-input full SLAM speed rung. Derived physical guard follows the existing target-dependent formula; no guard/acceptance value is changed by this tool.'}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,action='append',required=True,help='stopped capture or experiment directory; repeat per rung')
    parser.add_argument('--output',type=Path,required=True,help='new report directory, never overwritten')
    args=parser.parse_args(argv)
    try:
        if args.output.exists():raise ValueError('output directory must be new')
        audits=[audit_run(path) for path in args.run]
        audits.sort(key=lambda row:(row['target_command_mps'] is None,row['target_command_mps'] or 0,row['run_directory']))
        report={'schema':'dynanav_speed_ladder_offline_audit/v1','created_utc':datetime.now(timezone.utc).isoformat(),
                'SDK_or_ROS_nodes_started':False,'source_or_guard_modified':False,'GT_used_only_for_offline_endpoint_and_speed_scoring':True,
                'scorer':evidence(Path(__file__)),'existing_speed_summary_helper':evidence(HELPER_PATH),
                'cruise_reporting_fraction':CRUISE_FRACTION,'runs':audits,'comparison':compare_signatures(audits)}
        args.output.mkdir(parents=True,exist_ok=False)
        with (args.output/'speed-ladder-evidence.json').open('x',encoding='utf8') as handle:
            json.dump(report,handle,indent=2,ensure_ascii=False,allow_nan=False);handle.write('\n')
        rows=[]
        for audit in audits:
            cruise=audit['cruise_actual_forward_command_selection'] or {};xy=cruise.get('xy',{})
            rows.append({'run':audit['run_directory'],'target_command_mps':audit['target_command_mps'],
                'route':audit['comparison_signature_without_target_speed']['route'],
                'GT_navigation':audit['comparison_signature_without_target_speed']['ground_truth_navigation'],
                'reported_full_three_stage_SLAM_completion':audit['reported_full_three_stage_measured_closedloop_completion'],
                'cruise_measured_s':cruise.get('measured_duration_s'),'cruise_XY_mean_mps':xy.get('average_mps'),
                'cruise_XY_p95_mps':xy.get('p95_mps'),'cruise_XY_max_mps':xy.get('max_mps'),
                'termination':audit['failure_evidence']['category']})
        with (args.output/'speed-ladder-table.csv').open('x',newline='',encoding='utf8') as handle:
            writer=csv.DictWriter(handle,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        with (args.output/'audit-used.py').open('xb') as handle:handle.write(Path(__file__).read_bytes())
        with (args.output/'summarize_dynanav_experiment.py').open('xb') as handle:handle.write(HELPER_PATH.read_bytes())
        print(json.dumps({'output':str(args.output),'rungs':rows},ensure_ascii=False,indent=2))
        return 0
    except (OSError,ValueError,KeyError) as error:
        print(f'Speed ladder audit error: {error}',file=sys.stderr);return 2


if __name__=='__main__':raise SystemExit(main())
