"""Native joint-target loop for RoboGauge MoE-CTS; simulation only.

Command/observation/frame callbacks are explicit dependencies supplied by worker.
Experiment schedules, Teacher loading and old acceptance files are not consulted.
"""
import argparse,json,math,os,socket,time
from pathlib import Path
import numpy as np
import torch
from embodied_navigation.robot.unitree.policy.native.observation import TerrainHeightMap
from embodied_navigation.common.state_contract import StateContractError
from embodied_navigation.simulation.safety_state import from_gazebo_native64
from embodied_navigation.control.safety.simulation_gate import SimulationSafetyGate
QDEFAULT=np.array([-.1,.8,-1.5,.1,.8,-1.5,-.1,1.,-1.5,.1,1.,-1.5])


def prepare_velocity_input(cmd, request, input_clock_ns, ordinary_zero_adapter=None):
    """Original slew, then an explicit frame-local ordinary parking adapter.

    This changes only the three velocity observation inputs. Actions, previous
    action, joint targets and native safety remain owned by the original loop.
    """
    cmd+=np.clip(request-cmd,-np.array([.6,.6,.8])*.02,np.array([.6,.6,.8])*.02)
    return None if ordinary_zero_adapter is None else ordinary_zero_adapter(cmd,request,input_clock_ns)

def recv_exact(conn,n):
    buf=b''
    while len(buf)<n:
        data=conn.recv(n-len(buf))
        if not data:raise EOFError('Gazebo actuator disconnected')
        buf+=data
    return buf

def atomic_json(p,obj):
    tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(obj,ensure_ascii=False,allow_nan=False));tmp.replace(p)

def rpy(q):
    w,x,y,z=q
    return [math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(np.clip(2*(w*y-z*x),-1,1)),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))]

def main(*,actor_loader,observation_builder,command_source,receive,publish,finalize,prepare,runtime_duration,termination_requested=None,argv=None):
    recv_exact=receive;atomic_json=publish;build_observation=observation_builder
    finalize_actor_frame=finalize;prepare_velocity_input=prepare
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True)
    p.add_argument('--socket',required=True);p.add_argument('--terrain-target-manifest',type=Path)
    args=p.parse_args(argv);args.noisy=False;args.seed=42
    actor=actor_loader();args.run.mkdir(parents=True,exist_ok=True)
    safety_terrain=TerrainHeightMap.from_sdf(args.run/'world.sdf');terrain=safety_terrain
    terrain_manifest=None
    if args.terrain_target_manifest:
        terrain_manifest=json.loads(args.terrain_target_manifest.read_text());selected=terrain_manifest['include_models']
        if not isinstance(selected,list)or not selected or any(not isinstance(v,str)for v in selected)or len(set(selected))!=len(selected):
            raise ValueError('Terrain target manifest requires explicit unique model names')
        available={shape.name.split('/')[0]for shape in safety_terrain.shapes}
        if set(selected)-available:raise ValueError('Requested terrain model absent from actual static collisions')
        terrain=TerrainHeightMap.from_sdf(args.run/'world.sdf',include_models=selected)
    recording=os.environ.get('NAVIGATION_RECORD_RAW','0')=='1'
    rng=np.random.default_rng(args.seed)
    last_action=np.zeros(12);cmd=np.zeros(3);fault=None;first_t=None;settled_frames=0
    safety_gate=SimulationSafetyGate(run_id=args.run.name)
    last_command_t=0.;last_request=np.zeros(3);navigation_sequence={};completion_time=None
    metadata=dict(inference_device='cpu',torch_threads=1,actor_kind='robogauge_moe_cts',
        observation_noise='clean',observation_source='IMU orientation/angular velocity and joints; no COM velocity or terrain policy inputs',
        navigation_truth_used=False,action_clip=100.0,seed=42,
        bootstrap_PD_seconds=.1,command_slew_acceleration=[.6,.6,.8],
        physics_state_phase='PreUpdate reads previous Physics; effective state time=world-.005s',
        test_duration_s=runtime_duration,
        safety_state_origin='simulation_truth',safety_state_purpose='simulation_safety',
        safety_decision_owner='control.safety.simulation_gate')
    if terrain_manifest:
        metadata['terrain_target_manifest']={'path':str(args.terrain_target_manifest),
            'definition':terrain_manifest,'selected_collision_names':[shape.name for shape in terrain.shapes],
            'excluded_collision_names':[shape.name for shape in safety_terrain.shapes if shape.name not in {v.name for v in terrain.shapes}],
            'world_source':terrain.source,'safety_source':'all static collisions, ray start at body height',
            'equivalence':'Explicit deployment terrain provider; frozen grid/yaw/+20/clip retained. Terrain target geometry is not strict training equivalence.'}
    atomic_json(args.run/'policy_manifest.json',metadata)
    sock=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);sock.bind(args.socket);sock.listen(1);sock.settimeout(90)
    (args.run/'worker_ready').write_text(str(os.getpid()))
    stream=((args.run/'telemetry.jsonl')if recording else Path('/dev/null')).open('w',buffering=1);scan_stream=(args.run/'height_scan_sources.jsonl').open('w',buffering=1)if terrain_manifest and recording else None;observations=[];actions=[];latencies=[];sample_count=0;last_row=None
    try:
        conn,_=sock.accept();conn.settimeout(10)
        with conn,torch.inference_mode():
            while True:
                s=np.frombuffer(recv_exact(conn,64*8),dtype='<f8').copy()
                if first_t is None:first_t=s[0]
                t=float(s[0]-first_t);request,phase=command_source('navigation',t)
                last_command_t=t;last_request=request.copy()
                command_expired=False;command_reason='current guarded navigation command'
                ordinary_zero_receipt=prepare_velocity_input(cmd,request,round(float(s[0])*1e9),globals().get('ordinary_zero_input_adapter'))
                angles=rpy(s[4:8]);contacts=s[50:55].tolist()
                obs,extra=build_observation(s,cmd,last_action,terrain,noisy=args.noisy,rng=rng)
                if ordinary_zero_receipt is not None:
                    ordinary_zero_receipt.update(actual_observation_command_body=obs[6:9].tolist(),
                        observation_command_exact_zero=bool(np.array_equal(obs[6:9],np.zeros(3,dtype=np.float32))))
                    if not ordinary_zero_receipt['observation_command_exact_zero']:fault=fault or 'ordinary_zero_observation_contract_rejected'
                sensor_evidence=None
                if scan_stream:scan_stream.write(json.dumps({'sim_time':t,**extra},allow_nan=False)+'\n')
                start=time.perf_counter_ns();action=actor(torch.from_numpy(np.asarray(obs,dtype=np.float32)).reshape(1,45)).numpy()[0]if np.isfinite(obs).all()else last_action.copy()
                latency=(time.perf_counter_ns()-start)/1e6;latencies.append(latency)
                # Preserve policy -> actuator -> physical-gate latch priority,
                # including faults already present when the terrain query raises.
                fault=safety_gate.latch_runtime_faults(
                    policy_finite=bool(np.isfinite(action).all() and np.isfinite(obs).all()),
                    actuator_fault_code=float(s[56]),prior_fault=fault)
                # This privileged ray is simulation safety evidence, never SLAM pose.
                ground=float(safety_terrain.height(s[None,1:3],ray_start_z=s[3])[0])
                safety_state=None
                try:
                    safety_state=from_gazebo_native64(s,first_world_time_s=first_t,
                        ground_height_m=ground,run_id=args.run.name)
                except StateContractError as error:
                    safety_result=safety_gate.reject_state(error,prior_fault=fault)
                else:
                    safety_result=safety_gate.evaluate(safety_state,prior_fault=fault)
                fault=safety_result.fault
                observed_fall_or_low=safety_result.observed_fall_or_low
                observed_body_contact=safety_result.observed_body_contact
                observation_end=safety_result.persistent_fall_observation_end
                target=QDEFAULT+.25*action
                if t<.1:target=QDEFAULT.copy();phase='initializing';last_action=np.zeros(12)
                else:last_action=action.copy()
                # Parking keeps the learned posture feedback active. Freezing
                # a single support target was tested, failed, and is archived.
                # A zero VELOCITY COMMAND is never treated as a zero ACTION.
                if phase=='stop_transition' and np.linalg.norm(cmd)<1e-6:
                    settled_frames=settled_frames+1 if all(v>0 for v in contacts[1:]) and np.linalg.norm(s[8:10])<.08 and abs(s[13])<.1 else 0
                    if settled_frames>=15:phase='robogauge_zero_command_stand'
                else:settled_frames=0
                done=t>=metadata['test_duration_s'] or safety_result.stop or(termination_requested is not None and termination_requested())
                response=np.zeros(16,dtype='<f8');response[:12]=target;response[12]=1 if fault else 0;response[13]=1 if done else 0
                row={'sim_time':t,'world_sim_time':float(s[0]),'command':cmd.tolist(),'requested':request.tolist(),
                     'state_physics_world_time':float(s[0]-.005),
                     'measured':[float(s[8]),float(s[9]),float(s[13])], 'body_lin_vel':s[8:11].tolist(),
                     'body_ang_vel':s[11:14].tolist(),'position':s[1:4].tolist(),'quaternion_wxyz':s[4:8].tolist(),
                     'rpy':angles,'contacts':dict(zip(['body','FR','FL','RR','RL'],contacts)),
                     'q':s[14:26].tolist(),'qd':s[26:38].tolist(),'applied_torque':s[38:50].tolist(),
                     'q_target':target.tolist()if np.isfinite(target).all()else None,'action':action.tolist(),
                     'policy_target':(QDEFAULT+.25*action).tolist(),
                     'observed_original_fall_gate':observed_fall_or_low,'observed_body_contact_gate':observed_body_contact,'persistent_fall_observation_end':observation_end,'state':phase,'fault':fault,'inference_ms':latency,'body_clearance':float(s[3]-ground) if np.isfinite(s[3]-ground) else None,
                     'height_scan_min':None,'height_scan_max':None,
                     'height_scan_overhead_points':int(extra.get('height_scan_overhead_count',0)),
                     'command_expired':command_expired,'command_age_sim_s':t-last_command_t,
                     'command_source':metadata.get('command_source','scripted simulation test'),'command_reason':command_reason,
                     'physical_test_completion_time_s':completion_time,
                     'navigation_envelope':None,
                     'sensor_feedback_used':sensor_evidence,'actor_inferred_this_frame':bool(np.isfinite(obs).all()),
                     'observation_source':metadata['observation_source'],
                     'simulation_safety':dict(source=safety_state.provenance.as_dict() if safety_state else None,
                         purpose='simulation_safety',stop=safety_result.stop,reason=safety_result.reason,
                         fault_damping=safety_result.request_fault_damping,
                         state_contract_valid=safety_result.state_contract_valid,
                         state_contract_error=safety_result.state_contract_error)}
                if ordinary_zero_receipt is not None:
                    ordinary_zero_receipt['actor_inferred_this_frame']=bool(np.isfinite(obs).all())
                    row['ordinary_zero_before_actor_observation']=ordinary_zero_receipt
                prepared_frame=finalize_actor_frame(row)
                sample_count+=1;last_row=row
                if recording:stream.write(json.dumps(prepared_frame,allow_nan=False)+'\n')
                if sample_count%10==0 or done:atomic_json(args.run/'state.json',row)
                if recording:observations.append(obs);actions.append(action)
                conn.sendall(response.tobytes())
                if done:break
    except Exception as e:
        fault=fault or f'{type(e).__name__}: {e}'
    finally:
        stream.close();sock.close()
        if scan_stream:scan_stream.close()
        try:Path(args.socket).unlink()
        except FileNotFoundError:pass
        if recording:np.savez_compressed(args.run/'observations_actions.npz',observations=np.asarray(observations),actions=np.asarray(actions))
        atomic_json(args.run/'worker_result.json',{'completed':bool(sample_count),'fault':fault,'samples':sample_count,
            'physical_test_completion_time_s':completion_time,
            'last_sim_time':last_row['sim_time']if sample_count else None,'cpu_forward_ms':{'p50':float(np.median(latencies)),'p95':float(np.percentile(latencies,95)),'max':float(max(latencies))}if latencies else {}})
