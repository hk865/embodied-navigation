"""Immutable goal geometry, frame conversion and measured-stamp dwell.

Arrival geometry is independent of collision planning. This module never
creates a ROS node, accepts a GT control input or generates robot commands.
Legacy waypoints retain their 3D open .22 m ball representation; new goals
explicitly preregister region bounds before execution.
"""
from dataclasses import dataclass, replace
import hashlib
import json
import math

import numpy as np
from scipy.spatial.transform import Rotation

IDENTITY = ((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))


def number(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int,float,np.integer,np.floating)):
        raise ValueError(name+' must be a finite number')
    if not math.isfinite(value):raise ValueError(name+' must be finite')
    return float(value)


def vector(value, size, name):
    if not isinstance(value, (list,tuple,np.ndarray)) or len(value)!=size:
        raise ValueError(name+' has invalid dimensions')
    return tuple(number(v,name) for v in value)


def basis(value):
    if not isinstance(value,(list,tuple,np.ndarray)) or len(value)!=3:
        raise ValueError('axes must contain three 3D vectors')
    axes=np.array([vector(v,3,'axes') for v in value],dtype=float)
    if axes.shape!=(3,3) or not np.allclose(axes@axes.T,np.eye(3),atol=1e-8,rtol=0) or not math.isclose(np.linalg.det(axes),1.,abs_tol=1e-8):
        raise ValueError('axes must be an orthonormal right-handed basis')
    return tuple(tuple(float(v) for v in row) for row in axes)


@dataclass(frozen=True)
class Goal:
    goal_id: str
    center: tuple
    kind: str
    axes: tuple=IDENTITY  # Rows are local along/lateral/normal axes in frame.
    radius: float=0.
    height_half_span: float=0.
    half_extents: tuple=(0.,0.,0.)
    dwell_sim_s: float=.4
    timeout_sim_s: float=90.
    route_surface_id: str|None=None
    legacy: bool=False
    control_extents: tuple|None=None  # Same center/axes/dwell; stricter stop band.

    def __post_init__(self):
        if not isinstance(self.goal_id,str) or not self.goal_id.strip():raise ValueError('goal_id required')
        if self.kind not in ('sphere','disc_prism','oriented_box'):raise ValueError('invalid kind')
        object.__setattr__(self,'center',vector(self.center,3,'center'))
        if max(map(abs,self.center))>1000:raise ValueError('goal center exceeds 1000 m frame bounds')
        object.__setattr__(self,'axes',basis(self.axes))
        object.__setattr__(self,'half_extents',vector(self.half_extents,3,'half_extents'))
        for name in ('radius','height_half_span','dwell_sim_s','timeout_sim_s'):
            object.__setattr__(self,name,number(getattr(self,name),name))
        if self.dwell_sim_s<=0 or self.timeout_sim_s<=self.dwell_sim_s:raise ValueError('invalid dwell/timeout')
        if self.kind in ('sphere','disc_prism') and self.radius<=0:raise ValueError('radius must be positive')
        if self.kind=='disc_prism' and self.height_half_span<=0:raise ValueError('height bound required')
        if self.kind=='oriented_box' and min(self.half_extents)<=0:raise ValueError('positive box extents required')
        if self.control_extents is not None:
            if self.legacy:raise ValueError('legacy waypoint control cannot be changed')
            outer=(self.radius,) if self.kind=='sphere' else ((self.radius,self.height_half_span) if self.kind=='disc_prism' else self.half_extents)
            control=vector(self.control_extents,len(outer),'control_band')
            if min(control)<=0 or any(i>o for i,o in zip(control,outer)):raise ValueError('control band must be positive and contained in outer region')
            object.__setattr__(self,'control_extents',control)

    def control_goal(self):
        if self.control_extents is None:return self
        if self.kind=='sphere':return replace(self,radius=self.control_extents[0],control_extents=None)
        if self.kind=='disc_prism':return replace(self,radius=self.control_extents[0],height_half_span=self.control_extents[1],control_extents=None)
        return replace(self,half_extents=self.control_extents,control_extents=None)

    def control_arrival_definition(self):
        return self.control_goal().definition()['arrival']

    def definition(self):
        arrival=dict(type=self.kind,dwell_sim_s=self.dwell_sim_s)
        if self.kind=='sphere':arrival['radius_m']=self.radius
        else:arrival['axes']=[list(row) for row in self.axes]
        if self.kind=='disc_prism':
            arrival.update(radius_m=self.radius,height_half_span_m=self.height_half_span)
        if self.kind=='oriented_box':arrival['half_extents_m']=list(self.half_extents)
        if self.control_extents is not None:arrival['control_band']=self.control_arrival_definition()
        return dict(goal_id=self.goal_id,center=list(self.center),arrival=arrival,
                    timeout_sim_s=self.timeout_sim_s,route_surface_id=self.route_surface_id,
                    legacy=self.legacy)


def parse_goal(raw):
    if not isinstance(raw,dict):raise ValueError('goal must be an object')
    gid=raw.get('goal_id')
    if not isinstance(gid,str) or not gid.strip() or len(gid)>100:raise ValueError('goal_id must be a nonempty string')
    arrival=raw.get('arrival')
    if not isinstance(arrival,dict):raise ValueError('arrival must be an explicit object')
    kind=arrival.get('type')
    if kind not in ('sphere','disc_prism','oriented_box'):raise ValueError('unknown arrival type')
    center=vector(raw.get('center'),3,'center')
    dwell=number(arrival.get('dwell_sim_s',.4),'dwell_sim_s')
    timeout=number(raw.get('timeout_sim_s',90.),'timeout_sim_s')
    if dwell<=0 or timeout<=dwell:raise ValueError('invalid dwell or timeout')
    axes=IDENTITY if kind=='sphere' else basis(arrival.get('axes',IDENTITY))
    r=number(arrival.get('radius_m',0.),'radius_m')
    h=number(arrival.get('height_half_span_m',0.),'height_half_span_m')
    ext=vector(arrival.get('half_extents_m',(0.,0.,0.)),3,'half_extents_m')
    if kind in ('sphere','disc_prism') and r<=0:raise ValueError('radius must be positive')
    if kind=='disc_prism' and h<=0:raise ValueError('height bound is mandatory')
    if kind=='oriented_box' and min(ext)<=0:raise ValueError('all box extents must be positive')
    surface=raw.get('route_surface_id')
    if surface is not None and (not isinstance(surface,str) or not surface.strip()):raise ValueError('invalid route_surface_id')
    control=None
    if 'control_band' in arrival:
        band=arrival['control_band']
        if not isinstance(band,dict):raise ValueError('control_band must be a geometry object')
        dimensions={'sphere':('radius_m',),'disc_prism':('radius_m','height_half_span_m'),'oriented_box':('half_extents_m',)}[kind]
        allowed=set(dimensions)|{'type','axes','dwell_sim_s'}
        if set(band)-allowed:raise ValueError('control_band cannot override center, timeout or geometry kind')
        if band.get('type',kind)!=kind:raise ValueError('control kind must match outer region')
        if number(band.get('dwell_sim_s',dwell),'control dwell')!=dwell:raise ValueError('control dwell must match outer region')
        if 'axes' in band and basis(band['axes'])!=axes:raise ValueError('control axes must match outer region')
        if kind=='oriented_box':control=vector(band.get('half_extents_m'),3,'control half extents')
        else:control=tuple(number(band.get(name),'control '+name) for name in dimensions)
    return Goal(gid,center,kind,axes,r,h,ext,dwell,timeout,surface,control_extents=control)


def parse_request(raw):
    if not isinstance(raw,dict):raise ValueError('request must be an object')
    rid=raw.get('request_id')
    if not isinstance(rid,str) or not 1<=len(rid)<=128:raise ValueError('request_id required, max128')
    if raw.get('frame_id')!='camera_init':raise ValueError('NAV request must already be in camera_init')
    if ('goals' in raw)==('waypoints' in raw):raise ValueError('exactly one of goals/waypoints is required')
    values=raw.get('goals',raw.get('waypoints'))
    if not isinstance(values,list) or not 1<=len(values)<=500:raise ValueError('goals must contain 1..500 entries')
    if 'waypoints' in raw:
        # Mirror the legacy parser's finite numeric conversion and bounds.
        points=np.asarray(values,dtype=float)
        if points.shape!=(len(values),3) or not np.isfinite(points).all() or abs(points).max()>1000:
            raise ValueError('invalid legacy waypoints')
        goals=tuple(Goal(f'waypoint-{i}',tuple(p),'sphere',radius=.22,legacy=True)
                    for i,p in enumerate(points))
    else:
        if raw.get('schema_version')!=2:raise ValueError('region goals require schema_version 2')
        goals=tuple(parse_goal(item) for item in values)
        if len({g.goal_id for g in goals})!=len(goals):raise ValueError('duplicate goal_id')
    return rid,goals


def definitions_sha256(goals):
    encoded=json.dumps([g.definition() for g in goals],sort_keys=True,separators=(',',':'),allow_nan=False).encode()
    return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class RigidTransform:
    rotation: tuple
    translation: tuple

    def __post_init__(self):
        # Rotation matrices and axis-row bases obey the same SO(3) predicate.
        object.__setattr__(self,'rotation',basis(self.rotation))
        object.__setattr__(self,'translation',vector(self.translation,3,'translation'))

    def point(self,p):
        return np.array(self.rotation)@np.array(vector(p,3,'position'))+np.array(self.translation)

    def goal(self,g):
        axes=np.array(g.axes)@np.array(self.rotation).T
        return replace(g,center=tuple(self.point(g.center)),axes=basis(axes))


def relative_world_transform(origin,scene_heading):
    """Freeze once from sensor-derived heading; never body heading every tick."""
    h=number(scene_heading,'scene_heading');c,s=math.cos(h),math.sin(h)
    return RigidTransform(((c,-s,0.),(s,c,0.),(0.,0.,1.)),vector(origin,3,'origin'))


def initial_truth_alignment(slam_position,slam_quaternion,truth_position,truth_quaternion):
    """Independent evaluation only. Caller stores this one initial SE3 forever."""
    qs=np.array(vector(slam_quaternion,4,'SLAM quaternion'))
    qg=np.array(vector(truth_quaternion,4,'truth quaternion'))
    if abs(np.linalg.norm(qs)-1.)>.01 or abs(np.linalg.norm(qg)-1.)>.01:raise ValueError('invalid quaternion norm')
    rs=Rotation.from_quat(qs).as_matrix();rg=Rotation.from_quat(qg).as_matrix()
    r=rs@rg.T;t=np.array(vector(slam_position,3,'SLAM position'))-r@vector(truth_position,3,'truth position')
    return RigidTransform(tuple(map(tuple,r)),tuple(t))


def contains(goal,p):
    p=np.array(vector(p,3,'raw measured position'));center=np.array(goal.center)
    if goal.kind=='sphere':return bool(math.hypot(*(p-center))<goal.radius)
    local=np.array(goal.axes)@(p-center)
    # Closed-region boundary roundoff only: <=16 machine eps at coordinate
    # scale (sub-picometre here), not a physical acceptance margin.
    eps=16*np.finfo(float).eps*max(1.,float(np.max(abs(p))),float(np.max(abs(center))))
    if goal.kind=='disc_prism':
        return bool(np.linalg.norm(local[:2])<=goal.radius+eps and abs(local[2])<=goal.height_half_span+eps)
    return bool(np.all(abs(local)<=np.array(goal.half_extents)+eps))


def contains_control(goal,p):
    return contains(goal.control_goal(),p)


class ArrivalWindow:
    """New-region measured-stamp dwell, independent of ROS wall timers.

    Invoke for a newly accepted raw odom or a protection event. Duplicate
    observations never advance the window; stale/hold/outside resets it.
    Legacy requests use the controller's existing timer semantics instead.
    """
    def __init__(self,goal,max_gap_sim_s=.2):
        self.goal=goal;self.max_gap_ns=round(number(max_gap_sim_s,'max_gap_sim_s')*1e9)
        if self.max_gap_ns<=0:raise ValueError('max_gap must be positive')
        self.since=None;self.last_stamp=None;self.reason='waiting'

    def reset(self,reason='reset'):
        self.since=None;self.reason=reason

    def observe(self,p,stamp_ns,*,fresh=True,protected=False):
        if type(fresh) is not bool or type(protected) is not bool:raise ValueError('health flags must be actual bool')
        if not isinstance(stamp_ns,(int,np.integer)) or isinstance(stamp_ns,(bool,np.bool_)) or stamp_ns<0:
            self.reset('invalid_stamp');raise ValueError('stamp_ns must be nonnegative integer')
        if not fresh or protected:
            self.reset('stale' if not fresh else 'protected');return False
        if self.last_stamp is not None and stamp_ns<=self.last_stamp:
            if stamp_ns<self.last_stamp:self.reset('out_of_order')
            else:self.reason='duplicate'
            return False
        gap=self.last_stamp is not None and stamp_ns-self.last_stamp>self.max_gap_ns
        self.last_stamp=int(stamp_ns)
        try:inside=contains_control(self.goal,p)
        except ValueError:
            self.reset('invalid_pose');raise
        if not inside:
            self.reset('outside');return False
        if gap:self.reset('gap_reset')
        if self.since is None:self.since=int(stamp_ns)
        arrived=stamp_ns-self.since>=round(self.goal.dwell_sim_s*1e9)
        self.reason='arrived' if arrived else 'dwell'
        return bool(arrived)
