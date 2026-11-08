#!/usr/bin/env python3
"""Explicit Teacher/Student CPU modes with actual SLAM/SCAN source and causal ACK.

Native proxies supply Actor fields; Student adds explicitly privileged COM velocity
PID correction only. Native position never owns navigation/route/arrival.
"""
from embodied_navigation.common.observation import api as _observer_api; _r118_source_age = _observer_api('r118_source_age_probe')
import argparse,collections,hashlib,importlib.util,json,os,sys,time
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True

def canonical(x):return json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def atomic(p,x):
    tmp=p.with_suffix('.worker.tmp');tmp.write_text(canonical(x)+'\n');tmp.replace(p)

def atomic_ack_rows(p, encoded_rows):
    # Each entry is the completed bind_causal_ack canonical snapshot already
    # given to ACK history. Strings detach all mutable pending/telemetry aliases.
    content='{"entries":['+',\n'.join(encoded_rows)+'],"schema":"teacher_known_velocity_ack/v1"}\n'
    tmp=p.with_suffix('.worker.tmp');tmp.write_text(content);tmp.replace(p)

def initialize_command_reader(run,scope,scope_sha,limits,profile):
    """Construct the actual reader; no source manifests or source pin gates."""
    from embodied_navigation.control.executor_command import CommandReader
    reader=CommandReader(run.name,scope_sha,limits,profile=profile)
    if scope.get('run_dir')!=str(Path(run).resolve()):raise RuntimeError('Foreign task context')
    return reader

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True);ap.add_argument('--socket',required=True);a=ap.parse_args()
    run=a.run.resolve();scope_path=run/'runtime_context.json';scope=json.loads(scope_path.read_text());scope_sha=run.name
    actor_kind=scope.get('actor_kind','teacher')
    teacher_sha='bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
    student_checkpoint='7b59b8c7eafc9ffbc2dc32aaa2df3dadec3269d039338e4f93217bc73af66fd9'
    student_export='d8b51361ada74a80beb5e64f892447ea406d136be4c76a6ab15bae2d9b6c6185'
    if actor_kind not in ('teacher','student'):raise RuntimeError('Unknown actual Actor kind')
    sys.path.insert(0,str(Path(__file__).resolve().parent))
    daily=scope['profile'].get('daily_navigation',False)
    from embodied_navigation.common.runtime_telemetry import NoRecordingContext as RecordingContext, NullHistory as BoundedHistory
    # Optional payload budget: raw command8MiB + height8MiB + histories20MiB.
    # Live telemetry/state/ACK are separate, mandatory consumers and stay sync.
    recording=RecordingContext(run,maximum_command_bytes=8*1024*1024,maximum_height_bytes=8*1024*1024)
    p=scope['profile'];limits=np.asarray(p['cascade']['command_limits']);fingerprints={}
    from embodied_navigation.robot.unitree.policy.lower_loop_ack import configuration as lower_ack_configuration, annotate_pending, bind_causal_ack
    lower_ack_config=lower_ack_configuration(p['cascade'].get('r85_student_lower_loop_ack'))
    if lower_ack_config is not None and actor_kind!='student':
        raise ValueError('R85 Student lower-loop ACK contract requires actual Student actor')
    from embodied_navigation.robot.unitree.policy.native import worker as legacy
    policy_file=Path(legacy.__file__)
    from embodied_navigation.control.tracking.policy_selection import record_actual_loaded
    loaded_sources=record_actual_loaded(run,legacy)
    student=None
    if actor_kind=='student':
        binding=loaded_sources['selected_binding']
        from embodied_navigation.robot.unitree.policy import student as adapter
        student=adapter.StudentRuntime(run,scope,binding,limits)
    provider=None
    if p.get('terrain_layer_switch') is not None or p.get('mission46_required'):
        if p.get('mission46_required'):
            from embodied_navigation.simulation.terrain.mission46 import MissionTerrainProvider as SwitchingTerrainProvider
        else:
            from embodied_navigation.simulation.terrain.provider import SwitchingTerrainProvider
        original_map=legacy.TerrainHeightMap
        observation=sys.modules[legacy.build_observation.__module__]
        try:provider=SwitchingTerrainProvider(run,scope,original_map,observation)
        except Exception as error:
            with (run/'terrain_provider_initialization_failure.json').open('x') as out:
                out.write(canonical({'schema':'teacher_actor_terrain_switch/v1','status':'failed',
                    'stage':'initialization','error':type(error).__name__+': '+str(error),
                    'navigation_ground_truth_used':False})+'\n')
            raise
        class ActorTerrainFactory:
            @staticmethod
            def from_sdf(path,include_models=None,excluded_models=('go2','teacher_go2','moving_obstacle')):
                if include_models is None:
                    # Native clearance/contact safety is independent of Actor
                    # terrain selection and retains all static collisions.
                    return original_map.from_sdf(path,excluded_models=excluded_models)
                if (Path(path).resolve()!=provider.world or list(include_models)!=provider.initial_models or
                    tuple(excluded_models)!=('go2','teacher_go2','moving_obstacle')):
                    raise ValueError('Unexpected Actor terrain constructor; no geometry fallback allowed')
                return provider
        legacy.TerrainHeightMap=ActorTerrainFactory
        original_build=legacy.build_observation
        def build_observation(*args,**kwargs):
            obs,extra=original_build(*args,**kwargs)
            extra.update(actor_terrain_provider=provider.evidence())
            return obs,extra
        legacy.build_observation=build_observation
    recording.install_height_path(legacy)
    observation_builder=legacy.build_observation
    def timed_observation(*args,**kwargs):
        obs,extra=observation_builder(*args,**kwargs)
        if student is not None:
            student.observation_frame(args[0],args[1],args[2],obs,kwargs.get('noisy',False))
        return obs,extra
    def timed_actor_loader():
        actor=student.load_actor(legacy.torch)
        class Actor:
            def __call__(self,*args,**kwargs):
                return actor(*args,**kwargs)
        return Actor()
    original_recv,original_atomic=legacy.recv_exact,legacy.atomic_json
    latest={'clock_ns':None,'pending_ack':None,'metadata':None,'evidence':None,'completion_elapsed_s':None}
    reader=initialize_command_reader(run,scope,scope_sha,limits,p)
    if student is not None:
        original_prepare_velocity_input=legacy.prepare_velocity_input
        def student_prepare_velocity_input(cmd,request,input_clock_ns,ordinary_zero_adapter=None):
            return student.prepare_command(cmd,request,input_clock_ns,ordinary_zero_adapter,
                latest['evidence'],run.name,original_prepare_velocity_input)
    acks=collections.deque(maxlen=32);ack_seq=0
    from embodied_navigation.common.runtime_telemetry import NullStream
    log=NullStream();ack_log=NullStream()

    def receive(conn,n):
        nonlocal ack_seq
        try:
            data=original_recv(conn,n)
        except BaseException as error:
            raise
        if n==512:
            native_state=np.frombuffer(data,dtype='<f8')
            clock=round(float(native_state[0])*1e9)
            if student is not None:student.native_frame(native_state)
            if provider is not None:
                provider.observe_state(native_state)
            latest['clock_ns']=clock;pending=latest['pending_ack']
            if pending is not None and clock>pending['stamp_ns']:
                # Receiving the next actuator request proves the preceding
                # policy response completed. Only known commands are exposed.
                ack_seq+=1
                row={**pending,'sequence':ack_seq,'ack_native_clock_ns':clock,
                    'received_wall_ns':time.monotonic_ns(),'navigation_ground_truth_used':False,
                    'source':'previous actual CPU Actor velocity input acknowledged by next native request',
                    'contains_position_or_attitude':False}
                if lower_ack_config is not None:bind_causal_ack(row,lower_ack_config)
                encoded_ack=canonical(row)
                acks.append(encoded_ack);ack_log.write(encoded_ack+'\n')
                atomic_ack_rows(run/'closed_loop_executor_ack.json',acks)
                latest['pending_ack']=None
        return data

    def read_command():
        started=time.monotonic()
        try:
            raw=(run/'navigation_command.json').read_bytes()
            command,rejected,record=reader.read(raw,latest['clock_ns'],time.monotonic())
            record['reader_validation_sample_monotonic_wall']=record['read_monotonic_wall']
            record['read_monotonic_wall']=started
            return command,rejected,record
        except OSError as error:
            return reader.reject_frame(latest['clock_ns'],type(error).__name__+': '+str(error))

    def request(test,t):
        changed=[]
        cmd,rejected,attempt=read_command()
        if provider is not None and not changed:
            if provider.check_switch():
                cmd=np.zeros(3);rejected=True;reader.revoke()
                attempt.update(execution_override='actor_terrain_switch_protection',
                    reason='Actor terrain switch refused: '+provider.failure if provider.failure else 'Waiting for causal original connector arrival clock')
        evidence={'sim_time':float(t),'physics_clock_ns':latest['clock_ns'],'command':cmd.tolist(),
            'rejected':rejected,'reason':attempt['reason'],'actual_read_attempt':attempt,
            'actual_original_envelope':attempt.get('decoded_envelope'),'navigation_ground_truth_used':False}
        if provider is not None:evidence['actor_terrain_provider']=provider.evidence()
        latest['evidence']=evidence;log.write(canonical(recording.project_evidence(evidence))+'\n')
        # Completion/hold is declared by actual SLAM/IMU source, never native
        # position. First declaration is immutable and cannot shift later.
        try:
            status=json.loads((run/'navigation_status.json').read_text());park=status.get('cascade_parking') or {}
            if p.get('mission46_required'):
                mission=json.loads((run/'mission46_status.json').read_text())
                complete=(mission.get('run_id')==run.name and mission.get('navigation_ground_truth_used') is False
                    and (mission.get('stage')=='failed' or
                         mission.get('stage')=='completed' and mission.get('functional_sequence_completed') is True))
                if complete and latest['completion_elapsed_s'] is None:
                    latest['completion_elapsed_s']=float(t)
            else:
                terrain_ready=provider is None or (provider.switched and provider.failure is None)
                if terrain_ready and status.get('state')=='succeeded' and park.get('mode')=='active_hold' and latest['completion_elapsed_s'] is None:
                    latest['completion_elapsed_s']=float(t)
            if latest['completion_elapsed_s'] is not None and latest['metadata'] is not None:
                latest['metadata']['test_duration_s']=min(float(p['duration_s']),latest['completion_elapsed_s']+7.)
        except (OSError,ValueError,TypeError):pass
        return cmd,'initializing' if t<.1 else 'tracking' if np.any(cmd) else 'stop_transition'

    original_request=request
    def timed_request(test,t):
        return original_request(test,t)

    def finalize_actor_frame(obj):
        if isinstance(obj,dict) and 'world_sim_time' in obj and 'command_expired' in obj:
            obj=dict(obj);e=latest['evidence'] or {}
            if student is not None:obj=student.telemetry(obj)
            else:obj['actor_kind']='teacher'
            obj.update(command_expired=e.get('rejected',True),command_reason=e.get('reason','no input'),
                command_source=('actual SLAM/IMU fixed-goal terminal reference' if
                    ((e.get('actual_original_envelope') or {}).get('controller_source') or {}).get('reference_kind')=='terminal_fixed_goal/v1'
                    else 'actual SLAM/IMU cascade with original SCAN path'),closed_loop_read_evidence=e,
                outer_navigation_ground_truth_used=False,privileged_actor_observations=True)
            if provider is not None:obj['actor_terrain_provider']=provider.evidence()
            latest['pending_ack']={'stamp_ns':round(obj['world_sim_time']*1e9),
                'applied_command_body':obj['command'],'requested_command_body':obj['requested'],
                'actor_kind':actor_kind,'run_id':scope_sha,
                'actual_model_path':str(student.model_path)if student is not None else None}
            if student is not None:
                latest['pending_ack'].update(actual_actor_input=dict(student.input_receipt))
                if lower_ack_config is not None:
                    annotate_pending(latest['pending_ack'],obj['student_pid'],lower_ack_config,limits)
            zero=obj.get('ordinary_zero_before_actor_observation')
            if (zero is not None and zero.get('observation_command_exact_zero')is True
                and zero.get('actor_inferred_this_frame')is True and not obj.get('fault')
                and obj['command']==[0.,0.,0.] and obj['requested']==[0.,0.,0.]):
                if zero['input_clock_ns']!=latest['pending_ack']['stamp_ns']:
                    raise RuntimeError('Ordinary zero ACK input clock mismatch')
                latest['pending_ack']['ordinary_zero_before_actor_observation']=dict(zero)
            obj=recording.project_telemetry(obj)

        return obj

    def metadata(path,obj):
        if path.name=='policy_manifest.json':
            obj.update(test='closed_loop_SLAM_SCAN_cascade',command_source='actual SLAM/IMU cascade with SCAN guarded path',
                high_level_source_is_actual_SLAM=True,navigation_truth_used=False,actor_observations_remain_privileged=True,
                privileged_actor_observation_dimensions=232,test_duration_s=float(p['duration_s']),
                source_timeout_s=.3,run_id=scope_sha,legacy_motion_evaluation_applicable=False,
                command_start_seconds=None,command_end_s=None,acceptance_is_global=False)
            obj['actor_kind']=actor_kind
            if student is not None:obj.update(student.metadata())
            latest['metadata']=obj
            if provider is not None:obj['actor_terrain_layer_switch']=provider.contract()
        if path.name=='worker_result.json':obj.update(controller_hold_observed_worker_elapsed_s=latest['completion_elapsed_s'],
            navigation_ground_truth_used=False,actor_observations_remain_privileged=True,last_closed_loop_read=latest['evidence'])
        if path.name=='worker_result.json':
            obj.update(actor_kind=actor_kind,r85_task_completion_observation=latest.get('r85_completion_observation'))
            if student is not None:
                obj.update(student_export_sha256=student_export,student_actual_inference_count=student.inference_count,
                    student_pid_new_reset_count=student.pid.new_reset_count,
                    student_pid_old_would_reset_count=student.pid.old_would_reset_count)
        if path.name=='worker_result.json' and provider is not None:
            obj.update(actor_terrain_provider=provider.evidence(),actor_terrain_switch_failed=provider.failure is not None)
        original_atomic(path,obj)

    sys.argv=[str(policy_file),'--run',str(run),'--socket',a.socket,
        '--terrain-target-manifest',str(run/'terrain_target_manifest.json')]
    if student is None:raise ValueError('Normal navigation requires the configured Student')
    def prepare_frame(cmd,request,input_clock_ns,unused=None):
        return student_prepare_velocity_input(cmd,request,input_clock_ns,reader.before_observation)
    try:legacy.main(actor_loader=timed_actor_loader,observation_builder=timed_observation,
        command_source=timed_request,receive=receive,publish=metadata,finalize=finalize_actor_frame,
        prepare=prepare_frame,runtime_duration=float(p['duration_s']))
    except KeyboardInterrupt:pass
    finally:
        _r118_source_age.finish(run)
        if student is not None:student.close()
        log.close();ack_log.close()
        if provider is not None:provider.close()
        recording.close()

if __name__=='__main__':main()
