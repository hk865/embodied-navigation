"""Extract only static supplied scene geometry, never robot/dynamic poses."""
from pathlib import Path
import hashlib,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
def pose(element):
    if element is None:return np.eye(4)
    if element.get('relative_to'):raise ValueError('Relative SDF pose requires a reviewed resolver')
    v=np.asarray([float(x) for x in element.text.split()])
    if v.shape!=(6,) or not np.isfinite(v).all():raise ValueError('Invalid static geometry pose')
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',v[3:]).as_matrix();t[:3,3]=v[:3]
    return t
def build(world):
    world=Path(world);raw=world.read_bytes();boxes=[];excluded=[]
    for m in ET.fromstring(raw).findall('./world/model'):
        name=m.get('name','')
        if m.findtext('static')!='true' or not name.startswith(('floor_','ramp_','wall_','panel_','landmark_')):
            excluded.append(name);continue
        mt=pose(m.find('pose'))
        for link in m.findall('link'):
            lt=mt@pose(link.find('pose'))
            for c in link.findall('collision'):
                t=lt@pose(c.find('pose'));size=np.asarray([float(x) for x in c.findtext('geometry/box/size','').split()])
                v=link.find('visual')
                if size.shape!=(3,) or v is None:raise ValueError('Only checked static boxes supported')
                vs=np.asarray([float(x) for x in v.findtext('geometry/box/size','').split()])
                if not np.array_equal(size,vs) or not np.allclose(lt@pose(v.find('pose')),t,atol=1e-12):
                    raise ValueError('Static visual/collision geometry differs')
                corners=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*size/2
                points=corners@t[:3,:3].T+t[:3,3]
                boxes.append(dict(id=name+'/'+c.get('name'),model=name,T_world_box=t.tolist(),
                    size_m=size.tolist(),world_min=points.min(0).tolist(),world_max=points.max(0).tolist(),
                    surface_ids=[name+':'+axis+sign for axis in 'xyz' for sign in ('-','+')]))
    if not boxes:raise ValueError('Scene contains no supported static boxes')
    return dict(schema='given_scene_analytic_box_map/v1',source=dict(path=str(world),bytes=len(raw),
        sha256=hashlib.sha256(raw).hexdigest()),boxes=boxes,excluded_models=excluded,
        reference_kind='provided_simulation_scene_geometry',independent_sensor_map=False,robot_truth_pose_used=False)
