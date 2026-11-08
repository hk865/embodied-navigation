"""Generate ordinary simulation/sensor/planner configuration from project assets.

No experiment admission, source archive, evidence receipt or outcome is read.
"""
import os
from pathlib import Path
import copy,json,math,subprocess,sys,xml.etree.ElementTree as ET
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from embodied_navigation.common.config import ROOT, read, path, loads
from embodied_navigation.common.runtime_context import write as write_context


def write(directory,name,value):
    (directory/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def prepare(directory,profile):
    directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=False)
    profile=copy.deepcopy(profile);profile['daily_navigation']=True
    profile['cascade']['daily_navigation']=True
    profile['r85_auto_reference_contract']['reference_file']='global_reference.json'
    profile['experiment']='navigation'
    profile['engineering_recording'].update(raw_source_capture=False,registered_cloud_archive='disabled')
    command=[sys.executable,'-B',str(ROOT/'src/embodied_navigation/simulation/world/prepare.py'),'--output',str(directory),
        '--terrain',profile['expected_asset']['terrain'],'--sensors','--camera-rate','10',
        '--render-engine','ogre2','--real-time-factor','1.0','--overview-fov',str(math.radians(80)),
        '--overview-pose',*map(str,profile['overview_pose'])]
    subprocess.run(command,check=True,stdout=subprocess.DEVNULL,env={**os.environ,"PYTHONPATH":str(ROOT/"src")+os.pathsep+os.environ.get("PYTHONPATH","")})
    tree=ET.parse(directory/'world.sdf');robot=tree.getroot().find("world/model[@name='go2']")
    robot.find('pose').text=' '.join(map(str,[*profile['spawn'][:3],0,0,profile['spawn'][3]]))
    plugins=[p for p in robot.findall('plugin')if'TeacherActuator'in p.get('name','')]
    if len(plugins)!=1:raise ValueError('Expected one joint actuator plugin')
    plugins[0].set('filename',str(path('actuator')));ET.indent(tree);tree.write(directory/'world.sdf',encoding='unicode')
    from embodied_navigation.perception.sensors.sampling import override_sensors
    override_sensors(directory,profile)
    asset=json.loads((directory/'asset_manifest.json').read_text());asset['spawn']=profile['spawn']
    write(directory,'asset_manifest.json',asset)
    sensors=json.loads((directory/'sensor_contract.json').read_text())
    slam=yaml.safe_load((ROOT/'configs/localization_mapping/fastlivo.yaml').read_text());parameters=slam['/**']['ros__parameters']
    parameters['preprocess']['scan_line']=sensors['lidar']['samples']['vertical']
    parameters['common'].update(img_topic='/demo/teacher/slam/image',lid_topic='/demo/slam/lidar_filtered',imu_topic='/demo/teacher/slam/imu')
    ri,rl,rc=[Rotation.from_euler('xyz',sensors[k]['body_rpy']).as_matrix()for k in ('imu','lidar','camera')]
    pi,pl,pc=[np.asarray(sensors[k]['body_xyz'])for k in ('imu','lidar','camera')];optical=np.array([[0,-1,0],[0,0,-1],[1,0,0]])
    parameters['extrin_calib'].update(extrinsic_T=(ri.T@(pl-pi)).tolist(),extrinsic_R=(ri.T@rl).reshape(-1).tolist(),Rcl=(optical@rc.T@rl).reshape(-1).tolist(),Pcl=(optical@rc.T@(pl-pc)).tolist())
    parameters['imu'].update(imu_int_frame=int(round(sensors['imu']['rate_hz']*3)),init_min_span=2.9,stationary_initialization_en=True)
    parameters['imu'].update(profile.get('slam_imu_online_estimation',{}))
    if profile.get('slam_visual_measurement_covariance')is not None:parameters['vio']['img_point_cov']=profile['slam_visual_measurement_covariance']
    write(directory,'navigation_fastlivo.yaml',slam)
    write(directory,'navigation_camera.yaml',yaml.safe_load((ROOT/'configs/localization_mapping/camera.yaml').read_text()))
    write(directory,'navigation_profile.json',profile);write(directory,'navigation_scenario.json',profile['original_scenario'])
    write(directory,'terrain_target_manifest.json',json.loads((ROOT/profile['terrain_target_manifest']).read_text()))
    from embodied_navigation.localization_mapping.known_scene_reference_builder import build
    write(directory,'known_scene_reference.json',build(directory/'world.sdf'))
    from embodied_navigation.planning.scan_runtime_parameters import parameters as scan_parameters
    write(directory,'v34_scan_overrides.yaml',{'scan_planner_node':{'ros__parameters':scan_parameters(profile)}})
    route=loads((ROOT/'assets/geometry/route.json').read_text())
    write(directory,'global_route.json',{k:route[k]for k in ['body_points','ground_points','T_navigation_world','physical_sources','endpoint_support_connectors']})
    write_context(directory,profile)
    return profile
