"""Prepare original Building benchmark inputs without starting a simulator."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
from embodied_navigation.project import project_root, data_root
ROOT=project_root()
CONFIG=ROOT/'configs/robogauge'
CASES=('stairs_short','single_floor_turn','combined_floor1_inside','multi_floor2','single_floor_loop','second_stair_short','third_stair_short')

def actuator_path():
    return ROOT/'install/robogauge/lib/librobogauge_actuator.so'

def write(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

def prepare(case,label,mode='slam',backend_capture=False):
    if mode != 'slam' or backend_capture:
        raise ValueError('Only real-SLAM RoboGauge benchmark is supported')
    if case not in CASES:raise ValueError('Unknown benchmark case')
    if not label or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in label):
        raise ValueError('Invalid run label')
    import yaml
    assets=data_root()/'runtime_assets/assets'
    route=json.loads((CONFIG/'routes'/f'{case}.json').read_text())
    directory=data_root()/'runtime_assets/runs/robogauge'/label
    directory.mkdir(parents=True,exist_ok=False)
    spawn=[*route['start_world_ground'][:2],route['start_world_ground'][2]+.32,0.]
    timeout=160. if case in ('stairs_short','single_floor_turn') else 600.
    profile=json.loads((CONFIG/'deployment_profile.json').read_text())
    profile.update(duration_s=timeout+15.,spawn=spawn,localization_mode='fast_livo2_async',
                   navigation_ground_truth_used=False,actor_kind='robogauge_moe_cts',controller_kind='robogauge_moe_cts')
    write(directory/'runtime_context.json',dict(run_dir=str(directory),profile=profile,
          actor_kind='robogauge_moe_cts',controller_kind='robogauge_moe_cts',navigation_ground_truth_used=False,references={}))
    write(directory/'PCT_route.json',route)
    source=(CONFIG/'world.sdf.in').read_text().replace('${ASSET_ROOT}',str(assets)).replace('${ROBOGAUGE_ACTUATOR}',str(actuator_path()))
    world=ET.fromstring(source);node=world.find('world');node.set('name','building_navigation_robogauge')
    node.find("model[@name='go2']/pose").text=' '.join(map(str,[*spawn[:3],0,0,spawn[3]]))
    ET.indent(world);ET.ElementTree(world).write(directory/'world.sdf',encoding='unicode')
    for source,target in [('sensor_bridge.yaml','sensor_bridge.yaml'),('sensor_contract.json','sensor_contract.json'),('terrain_targets.json','terrain_target_manifest.json')]:
        (directory/target).write_bytes((CONFIG/source).read_bytes())
    scan=json.loads((ROOT/'configs/scenarios/building/scan.json').read_text())
    scan['scan_planner_node']['ros__parameters']['grid_map.frame_id']='map'
    scan['scan_planner_node']['ros__parameters']['grid_map.sliding_map_frame_id']='building_scan_sliding_map'
    write(directory/'scan_params.json',scan)
    parameters=yaml.safe_load((ROOT/'configs/localization_mapping/fastlivo.yaml').read_text())['/**']['ros__parameters']
    parameters['common'].update(img_topic='/demo/camera',lid_topic='/building/lidar_filtered',imu_topic='/livox/imu',use_gps=False,require_complete_imu=True)
    parameters['debug']['sensor_sync_diagnostics']=False
    parameters['backend']=dict(snapshot_directory=str(directory/'backend_spool'),queue_capacity=4,max_points=4096,max_ready_directories=16,min_interval_s=.5,max_interval_s=2.,min_translation_m=.8,min_rotation_rad=.2)
    parameters['publish'].update(publish_base_tf=False,odom_frame='odom',child_frame='base_link')
    write(directory/'building_fastlivo.json',{'/**':{'ros__parameters':parameters}})
    write(directory/'building_camera.json',yaml.safe_load((ROOT/'configs/localization_mapping/camera.yaml').read_text()))
    write(directory/'experiment_setup.json',dict(case=case,spawn_xyz_yaw=spawn,navigation_world_origin=spawn[:3],
          goal_world_body=route['world_body_points'][-1],timeout_sim_s=timeout,no_SLAM_or_global_matcher=False,
          navigation_ground_truth_used=False,navigation_mode='slam',backend_capture_only=False,configured_spawn_prior=True,
          route_origin='native PCT A* + physical support Z; frozen route, not online global replanning',
          adapter_version='world_twist_terminal_PD_v4',stair_speed_cap_mps=.25,planned_step_obstacle_filter=True,
          obstacle_cloud_pose_same_stamp=True,SCAN_start_duplicate_coalescing_m=.1,
          SCAN_twist_reference='world-axis velocity for SCAN; canonical body odom twist is child-axis',
          terminal_position_PD=dict(Kp_xy=4.,Kd_body_velocity_xy=.4,velocity_limits_xy=[.16,.12]),
          parking_contract='Original CaptureGrid .025m/.035rad dwell .6s; fixed5s 10Hz51poses .05m/.1rad',
          backend_timing=dict(spool_poll_s=.02,correction_owner_poll_s=.02,status_period_s=1.),
          actor_kind='robogauge_moe_cts',existing_native_COM_velocity_PID=False,
          motor_envelope='Go2HV 20.2 driving / 23.4 braking, full to 13.5rad/s then linear to zero at 30',
          robot_physics_scope='Existing Gazebo Go2 model; no claim of complete IsaacLab asset/solver equivalence'))
    return directory,profile
