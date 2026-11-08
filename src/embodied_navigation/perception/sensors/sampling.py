"""Apply explicit sampling changes only to a fresh, unlaunched generated scene."""
from pathlib import Path
import copy
import hashlib
import json
import xml.etree.ElementTree as ET


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def shape(e):
    return (e.tag, tuple(sorted(e.attrib.items())), (e.text or '').strip(), tuple(shape(c) for c in e))


def override_sensors(run, profile):
    run = Path(run)
    if (run/'runtime_manifest.json').exists() or (run/'sensor_sampling_contract.json').exists():
        raise RuntimeError('Sampling changes require a fresh unlaunched scene')
    settings = profile['sensor_sampling']
    v, h, lr, cr = [settings[k] for k in ('vertical_lines','horizontal_samples','lidar_hz','camera_hz')]
    if v != 64 or h != 480 or lr not in (10,20,30) or cr not in (10,20,30):
        raise ValueError('Only reviewed64-line density/rate variants are allowed')
    tree = ET.parse(run/'world.sdf')
    world = tree.getroot().find('world')
    robot = world.find("model[@name='go2']")
    lidar = robot.find(".//sensor[@type='gpu_lidar']")
    camera = robot.find(".//sensor[@name='demo_rgb']")
    if lidar is None or camera is None:
        raise RuntimeError('Actual body sensors missing')
    before = shape(world)
    paths = [(lidar.find('lidar/scan/vertical/samples'),str(v)),
             (lidar.find('lidar/scan/horizontal/samples'),str(h)),
             (lidar.find('update_rate'),str(lr)),(camera.find('update_rate'),str(cr))]
    originals = [(e, e.text) for e, _ in paths]
    (run/'world_before_sensor_override.sdf').write_bytes((run/'world.sdf').read_bytes())
    for e, text in paths:
        e.text = text
    candidate = copy.deepcopy(world)
    for e, text in originals:
        e.text = text
    if shape(world) != before:
        raise RuntimeError('Unexpected mutation outside allowed sensor leaves')
    # Verify that restoring only the four allowed leaves recovers the entire
    # generated world, including physics, collisions, noise and extrinsics.
    tree.getroot().remove(world)
    tree.getroot().append(candidate)
    ET.indent(tree)
    tree.write(run/'world.sdf',encoding='unicode')
    sensors = json.loads((run/'sensor_contract.json').read_text())
    old_sensor_sha = sha(run/'sensor_contract.json')
    sensors['lidar']['rate_hz'] = lr
    sensors['lidar']['samples'] = {'horizontal':h,'vertical':v}
    sensors['camera']['rate_hz'] = cr
    sensors['scope'] = 'Frozen actual scene sampling inputs; effective rates require original-header measurement'
    (run/'sensor_contract.json').write_text(json.dumps(sensors,indent=2)+'\n')
    receipt = {'schema':'teacher_sensor_sampling_trial/v1','settings':settings,
        'before_world_sha256':sha(run/'world_before_sensor_override.sdf'),
        'after_world_sha256':sha(run/'world.sdf'),
        'before_sensor_contract_sha256':old_sensor_sha,
        'after_sensor_contract_sha256':sha(run/'sensor_contract.json'),
        'only_allowed_sensor_leaves_changed':True,
        'unchanged_physics_step_s':.005,'unchanged_native_PD_hz':200,'unchanged_Actor_hz':50,
        'unchanged_camera_K_D_geometry':True,'unchanged_lidar_fov_noise_extrinsics':True,
        'navigation_ground_truth_used':False,
        'actual_rates_verified':False,'nominal_30_is_not_measured_30':True}
    (run/'sensor_sampling_contract.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return receipt
