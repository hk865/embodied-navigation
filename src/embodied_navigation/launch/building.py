"""Production Building launch using canonical SLAM and project-owned assets."""

from embodied_navigation.project import project_root as _project_root
from pathlib import Path
import argparse, datetime, fcntl, json, os, signal, subprocess, sys, time
import xml.etree.ElementTree as ET

ROOT=_project_root()
CASES=('stairs_short','single_floor_turn','combined_floor1_inside','multi_floor2','single_floor_loop','second_stair_short','third_stair_short')
def write(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')

def prepare(case,label,mode='slam',backend_capture=False):
    # Reject unsupported provenance before reading assets or creating a run.
    if mode!='slam' or backend_capture:
        raise ValueError('Production Building requires SLAM; truth/capture-only mode is not supported')
    route=json.loads((ROOT/'assets/building/routes'/f'{case}.json').read_text())
    directory=ROOT/'runs'/label
    directory.mkdir(parents=True,exist_ok=False)
    spawn=[*route['start_world_ground'][:2],route['start_world_ground'][2]+.32,0.]
    origin=spawn[:3];timeout=160. if case in ('stairs_short','single_floor_turn') else 600.
    profile=json.loads((ROOT/'configs/scenarios/building/student_profile.json').read_text())
    profile.update(duration_s=timeout+15.,spawn=spawn,localization_mode='fast_livo2_async',navigation_ground_truth_used=False)
    write(directory/'runtime_context.json',dict(run_dir=str(directory),profile=profile,
          actor_kind='student',controller_kind='student',navigation_ground_truth_used=False,references={}))
    write(directory/'PCT_route.json',route)
    source=(ROOT/'assets/building/world.sdf.in').read_text().replace('${PROJECT_ROOT}',str(ROOT))
    world=ET.fromstring(source);node=world.find('world');node.set('name','building_navigation_slam')
    node.find("model[@name='go2']/pose").text=' '.join(map(str,[*spawn[:3],0,0,spawn[3]]))
    ET.indent(world);ET.ElementTree(world).write(directory/'world.sdf',encoding='unicode')
    for source,target in [('sensor_bridge.yaml','sensor_bridge.yaml'),('sensor_contract.json','sensor_contract.json'),('terrain_targets.json','terrain_target_manifest.json')]:
        (directory/target).write_bytes((ROOT/'assets/building'/source).read_bytes())
    (directory/'scan_params.json').write_bytes((ROOT/'configs/scenarios/building/scan.json').read_bytes())
    write(directory/'experiment_setup.json',dict(case=case,spawn_xyz_yaw=spawn,navigation_world_origin=origin,
          goal_world_body=route['world_body_points'][-1],timeout_sim_s=timeout,navigation_mode='slam',no_SLAM_or_global_matcher=False,
          navigation_ground_truth_used=False,route_origin='native PCT A* + physical support Z',
          adapter_version='world_twist_terminal_PD_v4',stair_speed_cap_mps=.25,planned_step_obstacle_filter=True,obstacle_cloud_pose_same_stamp=True,SCAN_start_duplicate_coalescing_m=.1,
          SCAN_twist_reference='world-axis velocity for SCAN; canonical body odom twist is child-axis',
          terminal_position_PD=dict(Kp_xy=4.,Kd_body_velocity_xy=.4,velocity_limits_xy=[.16,.12]),
          parking_contract='Original CaptureGrid .025m/.035rad dwell .6s; fixed5s 10Hz51poses .05m/.1rad'))
    import yaml
    parameters=yaml.safe_load((ROOT/'configs/localization_mapping/fastlivo.yaml').read_text())['/**']['ros__parameters']
    parameters['common'].update(img_topic='/demo/camera',lid_topic='/building/lidar_filtered',imu_topic='/livox/imu',use_gps=False,require_complete_imu=True)
    parameters['debug']['sensor_sync_diagnostics']=False
    parameters['backend']=dict(snapshot_directory=str(directory/'backend_spool'),queue_capacity=4,max_points=4096,max_ready_directories=16,min_interval_s=.5,max_interval_s=2.,min_translation_m=.8,min_rotation_rad=.2)
    parameters['publish'].update(publish_base_tf=False,odom_frame='odom',child_frame='base_link')
    write(directory/'building_fastlivo.json',{'/**':{'ros__parameters':parameters}})
    write(directory/'building_camera.json',yaml.safe_load((ROOT/'configs/localization_mapping/camera.yaml').read_text()))
    scan=json.loads((directory/'scan_params.json').read_text());scan['scan_planner_node']['ros__parameters']['grid_map.frame_id']='map';scan['scan_planner_node']['ros__parameters']['grid_map.sliding_map_frame_id']='building_scan_sliding_map';write(directory/'scan_params.json',scan)
    setup=json.loads((directory/'experiment_setup.json').read_text());setup.update(navigation_mode='slam',backend_capture_only=False,navigation_ground_truth_used=False,configured_spawn_prior=True,no_SLAM_or_global_matcher=False,backend_timing=dict(spool_poll_s=.02,correction_owner_poll_s=.02,status_period_s=1.))
    write(directory/'experiment_setup.json',setup)
    return directory,profile

def environment(directory,domain,socket):
    from embodied_navigation.common.runtime_environment import environment as ros_environment
    from embodied_navigation.common.communication.dds_environment import configure
    from embodied_navigation.common.config import read
    transport=read()['dds_transport']
    env=configure(ros_environment())
    for key in transport['remove_environment_keys']:env.pop(key,None)
    env.update(transport['environment'])
    env['FASTRTPS_DEFAULT_PROFILES_FILE']=str(ROOT/'src/embodied_navigation/common/communication/dds_transport.xml')
    write(directory/'transport_settings.json',dict(environment=transport['environment'],
        profile_file=str(ROOT/'src/embodied_navigation/common/communication/dds_transport.xml')))
    env.update(ROS_DOMAIN_ID=str(domain),GZ_IP='127.0.0.1',GZ_PARTITION='building_'+directory.name,
        BUILDING_SAFETY_SOCKET='/tmp/building_contacts.sock',TEACHER_SOCKET=socket,DEMO_RUN_DIR=str(directory),ROS_LOG_DIR=str(directory/'ros_logs'),
        TEACHER_ACTUATOR_LOG=str(directory/'actuator_physics.jsonl'),
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(ROOT/'install/actuators/lib'),
        PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',
        NUMEXPR_NUM_THREADS='1',EGL_PLATFORM='surfaceless',NAVIGATION_RECORD_RAW='1')
    env.pop('LIBGL_ALWAYS_SOFTWARE',None)
    return env

def run(directory,profile,args):
    socket='/tmp/building_policy.sock';env=environment(directory,args.domain,socket)
    children=[];stop=False;reason='wall_limit';terminal=None;began=time.monotonic()
    def request_stop(*unused):
        nonlocal stop
        stop=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,request_stop)
    def start(role,cmd,changes=None):
        log=(directory/(role+'.log')).open('x')
        process=subprocess.Popen(cmd,cwd=ROOT,env={**env,**(changes or {})},stdout=log,
            stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
        children.append((role,process,log))
        write(directory/'processes.json',[dict(role=n,pid=p.pid)for n,p,f in children])
        return process
    try:
        actor=start('student',['/opt/navigation-venv/bin/python3','-B','-m','embodied_navigation.simulation.truth_worker',
            '--run',str(directory),'--socket',socket],{'CUDA_VISIBLE_DEVICES':''})
        start('sensor_bridge',['/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge','--ros-args',
            '-p','config_file:='+str(directory/'sensor_bridge.yaml')])
        if args.mode=='slam':
            start('resources',['/usr/bin/python3','-B','-m','embodied_navigation.evaluation.process_resources'])
            start('camera_parameters',['/opt/ros/jazzy/lib/demo_nodes_cpp/parameter_blackboard','--ros-args','--params-file',str(directory/'building_camera.json')])
            start('lidar_filter',['/usr/bin/python3','-B','-m','embodied_navigation.localization_mapping.building_sensors'])
            start('slam',[str(ROOT/'install/ros/lib/fast_livo2_ros/fastlivo_mapping'),'--ros-args','--params-file',str(directory/'building_fastlivo.json'),'--params-file',str(directory/'building_camera.json'),'-r','/aft_mapped_to_init:=/building/raw_body_odom','-r','/cloud_registered_full:=/building/raw_cloud'],{'FASTLIVO_PIPELINE_MODE':'staged','FASTLIVO_IMAGE_COPY_OPT':'1','FASTLIVO_SERIAL_IMU_PRIORITY':'0','FASTLIVO_DISABLE_STATE_LOG':'1','FASTLIVO_INGRESS_TRACE':'0'})
            initial=[*json.loads((directory/'experiment_setup.json').read_text())['spawn_xyz_yaw'][:3],0.,0.,0.]
            start('backend',['/usr/bin/python3','-B','-m','embodied_navigation.localization_mapping.backend.run','--spool',str(directory/'backend_spool'),'--history',str(directory/'backend_history'),'--result',str(directory/'backend_result.json'),'--external-map',str(ROOT/'assets/building/building2_9.pcd'),'--initial-world-odom',*map(str,initial),'--map-world-offset','-0.000002','-0.000001','-0.1'])
            start('localization',['/usr/bin/python3','-B','-m','embodied_navigation.localization_mapping.building_runtime'])
        nav=start('navigation' ,['/usr/bin/python3','-B','-m','embodied_navigation.navigation.building_navigation'])
        scan=start('scan',[str(ROOT/'install/ros/lib/scan_planner/scan_planner_node'),
            '--ros-args','--params-file',str(ROOT/'third_party/scan/plan_manage/config/planner.yaml'),
            '--params-file',str(directory/'scan_params.json'),
            '-r','body_pose:=/building/scan_body_odom','-r','sensor_pose:=/demo/slam/lidar_odom',
            '-r','cloud:=/cloud_registered_full','-r','initial_path:=/truth/scan_reference',
            '-r','planning/bspline:=/truth/bspline','-r','planning/go2_execution_frozen:=/truth/execution_frozen'])
        while not(directory/'worker_ready').exists():
            if stop:reason='interrupted';return 130
            if actor.poll()is not None:raise RuntimeError('Student exited before ready')
            if time.monotonic()-began>60:raise RuntimeError('Student ready timeout')
            time.sleep(.1)
        gz=start('gazebo',['gz','sim','-s','-r','--headless-rendering',str(directory/'world.sdf'),'--seed','42'])
        deadline=args.duration if args.duration is not None else profile['duration_s']*1.7+30
        last_progress=0.
        while time.monotonic()-began<deadline:
            if stop:reason='interrupted';break
            dead=[n for n,p,f in children if p.poll()is not None]
            if dead:reason='child_exit:'+','.join(dead);break
            try:
                status=json.loads((directory/'truth_navigation_status.json').read_text())
                if time.monotonic()-last_progress>10:
                    print(json.dumps({k:status.get(k)for k in ('state','sim_s','goal_xy_error_m','goal_height_error_m','candidate_count','reason')},ensure_ascii=False),flush=True);last_progress=time.monotonic()
                if status['state']in ('failed','succeeded'):
                    if terminal is None:terminal=time.monotonic();reason=status['state']
                    if time.monotonic()-terminal>3:break
            except(OSError,ValueError):pass
            time.sleep(.2)
    finally:
        def wait_shutdown(process):
            try:process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        # Ask the actor to finish one full original inference/safety exchange
        # with response[13]=1. Native physics acknowledges it and requests stop;
        # neither side sees an artificial EOF or timeout from concurrent signals.
        physics_alive=any(n=='gazebo' and p.poll()is None for n,p,f in children)
        if physics_alive and actor.poll()is None:
            actor.send_signal(signal.SIGUSR1)
            try:actor.wait(timeout=12)
            except subprocess.TimeoutExpired:pass
        if not physics_alive and any(n=='gazebo'for n,p,f in children)and actor.poll()is None:
            try:actor.wait(timeout=2)
            except subprocess.TimeoutExpired:pass
        for n,p,f in children:
            if n=='gazebo' and p.poll()is None:
                p.send_signal(signal.SIGINT);wait_shutdown(p)
        for n,p,f in reversed(children):
            if p.poll()is None:p.send_signal(signal.SIGINT)
        for n,p,f in reversed(children):
            wait_shutdown(p);f.close()
        write(directory/'experiment_exit.json',dict(wall_s=time.monotonic()-began,reason=reason,
            exit_codes={n:p.returncode for n,p,f in children},navigation_ground_truth_used=False))
    print(json.dumps(dict(directory=str(directory),reason=reason)),flush=True)
    return 0 if reason=='succeeded' else 2

def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode',choices=('slam',),default='slam');ap.add_argument('--case',choices=CASES,default='multi_floor2')
    ap.add_argument('--label');ap.add_argument('--domain',type=int,default=226)
    ap.add_argument('--prepare-only',action='store_true');ap.add_argument('--check',action='store_true')
    ap.add_argument('--duration',type=float)
    ap.add_argument('--validation-lidar-gap',type=float,nargs=2,metavar=('START_S','DURATION_S'),help='Explicit test only: drop LiDAR by its original source timestamp; disabled by default')
    ap.add_argument('--validation-lidar-gap-after-tracking',type=float,nargs=2,metavar=('DELAY_S','DURATION_S'),help='Test only: arm source gap after confirmed nonzero tracking; disabled by default')
    args=ap.parse_args(argv)
    needed=[ROOT/'install/ros/local_setup.bash',ROOT/'install/ros/lib/scan_planner/scan_planner_node',
        ROOT/'install/actuators/lib/libteacher_actuator.so',ROOT/'models/student_model_4999_deterministic_jit.pt',
        ROOT/'assets/building/Building.dae',ROOT/'assets/building/building2_9.pcd',
        ROOT/'assets/meshes/L1_4D_Lidar.dae',
        ROOT/'install/ros/lib/libfast_livo2_core.so',ROOT/'install/backend/lib/liblivo_graph.so']
    if args.check:
        missing=[str(p)for p in needed if not p.is_file()]
        print(json.dumps(dict(ready=not missing,missing=missing,root=str(ROOT),
              backend_integrated=not missing,validation_scope='Dependency check only; see per-run navigation results',navigation_mode='slam',navigation_ground_truth_used=False),ensure_ascii=False,indent=2))
        return 2 if missing else 0
    if not 1<=args.domain<=232:ap.error('domain must be 1..232')
    if args.duration is not None and args.duration<=0:ap.error('duration must be positive')
    if args.validation_lidar_gap:
        start_s,duration_s=args.validation_lidar_gap
        if args.mode!='slam' or start_s<8 or not 0<duration_s<=3:
            ap.error('validation gap requires slam, start>=8s and duration in (0,3]s')
    if args.validation_lidar_gap_after_tracking:
        delay_s,duration_s=args.validation_lidar_gap_after_tracking
        if args.mode!='slam' or args.validation_lidar_gap or delay_s<1 or not 0<duration_s<=3:
            ap.error('relative validation gap requires slam, delay>=1s, duration in (0,3]s and no absolute gap')
    label=args.label or datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+args.case
    if not label or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-'for c in label):ap.error('Invalid run label')
    lock=Path('/tmp/building_navigation.lock').open('a+')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    directory,profile=prepare(args.case,label,args.mode)
    if args.validation_lidar_gap:
        setup=json.loads((directory/'experiment_setup.json').read_text());setup['validation_lidar_gap_s']=args.validation_lidar_gap;write(directory/'experiment_setup.json',setup)
    if args.validation_lidar_gap_after_tracking:
        setup=json.loads((directory/'experiment_setup.json').read_text());setup['validation_lidar_gap_after_tracking_s']=args.validation_lidar_gap_after_tracking;write(directory/'experiment_setup.json',setup)
    print(json.dumps(dict(stage='prepared',directory=str(directory),mode=args.mode,planned_length_m=json.loads((directory/'PCT_route.json').read_text())['length_m']),ensure_ascii=False),flush=True)
    return 0 if args.prepare_only else run(directory,profile,args)

if __name__=='__main__':raise SystemExit(main())
