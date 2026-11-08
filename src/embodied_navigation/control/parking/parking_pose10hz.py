"""Pose-only parking samples. High-rate motion never grants or vetoes parking.

Grid times are simulation/header nanoseconds. A grid uses the latest distinct
real pose at or before it, without interpolation. The first pose is immutable.
Missing bins remain missing; later data never resets the fixed window.
"""
import copy
import math

PERIOD_NS=100_000_000
DURATION_NS=5_000_000_000
SOURCE_AGE_NS=300_000_000
XY_LIMIT_M=.05
YAW_LIMIT_RAD=.1
CONFIG=dict(schema='parking_pose10hz_fixed_anchor/v1',period_ns=PERIOD_NS,
    duration_ns=DURATION_NS,xy_drift_m=XY_LIMIT_M,yaw_drift_rad=YAW_LIMIT_RAD,
    source_age_ns=SOURCE_AGE_NS,sampling='latest_distinct_header_at_or_before_grid',
    interpolation=False,high_frequency_motion_gate=False,adjacent_increment_gate=False)

def configuration(value):
    if value is None:return False
    from embodied_navigation.control.safety.safety_configuration import pose
    return pose(value)

def wrap(value):return math.atan2(math.sin(value),math.cos(value))

def from_feedback(feedback):
    """Use Controller/R86's accepted canonical SLAM body callback and its clocks."""
    from embodied_navigation.control.safety.feedback_source import require_canonical_slam_feedback
    require_canonical_slam_feedback(feedback)
    q=list(feedback['quaternion_wxyz'])
    row=dict(stamp_ns=feedback['stamp_ns'],position=list(feedback['position_world_xyz']),
        quaternion_xyzw=q[1:]+q[:1],body_velocity=list(feedback['origin_velocity_body']),
        received_sim_ns=feedback['callback_ros_clock_ns'],
        received_wall_ns=feedback['received_wall_ns'],source_received_wall_ns=feedback['received_wall_ns'],
        source_topic='/demo/slam/body_odom',navigation_ground_truth_used=False,
        pose_producer=feedback['pose_producer'])
    pose(row)
    return row


def pose(row):
    stamp=row['stamp_ns'];p=list(row['position']);q=list(row['quaternion_xyzw'])
    if (type(stamp)is not int or stamp<0 or len(p)!=3 or len(q)!=4
            or not all(type(v)in(int,float) and math.isfinite(v)for v in p+q)
            or not .98<=sum(v*v for v in q)<=1.02
            or row.get('source_topic')!='/demo/slam/body_odom'
            or type(row.get('navigation_ground_truth_used'))is not bool
            or (row['navigation_ground_truth_used']is False and row.get('pose_producer')!='fast_livo2_async_backend')):
        raise ValueError('Invalid original SLAM pose for 10Hz parking')
    yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]))
    return stamp,p,yaw

class PoseGrid:
    def __init__(self,start_ns,origin,*,duration_ns=DURATION_NS):
        stamp,p,yaw=pose(origin)
        if type(start_ns)is not int or not 0<=start_ns-stamp<=SOURCE_AGE_NS:
            raise ValueError('10Hz anchor must be a fresh causal actual pose')
        self.start_ns=start_ns;self.end_ns=None if duration_ns is None else start_ns+duration_ns
        self.origin=copy.deepcopy(origin);self.base=p;self.base_yaw=yaw
        self.pending=[copy.deepcopy(origin)];self.last_input=stamp;self.last_pose=(p,yaw)
        self.last_sample_stamp=stamp;self.next_grid=start_ns+PERIOD_NS
        self.samples=[];self.missing=[];self.motion_failure=None
        self._sample(start_ns,origin)

    def _sample(self,grid,row):
        stamp,p,yaw=pose(row)
        previous=self.samples[-1] if self.samples else None
        sample=dict(grid_ns=grid,source_stamp_ns=stamp,source_age_ns=grid-stamp,
            position=p,yaw_rad=yaw,source=copy.deepcopy(row),
            xy_from_anchor_m=math.hypot(p[0]-self.base[0],p[1]-self.base[1]),
            yaw_from_anchor_rad=wrap(yaw-self.base_yaw),
            adjacent_xy_m=None if previous is None else math.hypot(p[0]-previous['position'][0],p[1]-previous['position'][1]),
            adjacent_yaw_rad=None if previous is None else wrap(yaw-previous['yaw_rad']))
        self.samples.append(sample);self.last_sample_stamp=stamp
        if sample['xy_from_anchor_m']>XY_LIMIT_M or abs(sample['yaw_from_anchor_rad'])>YAW_LIMIT_RAD:
            self.motion_failure=self.motion_failure or dict(reason='parking_pose10hz_fixed_anchor_exceeded',sample=copy.deepcopy(sample))
        return sample

    def observe(self,row):
        stamp,p,yaw=pose(row)
        if stamp<self.last_input:raise ValueError('10Hz SLAM source clock went backwards')
        if stamp==self.last_input:
            if (p,yaw)!=self.last_pose:raise ValueError('Repeated SLAM header changed pose')
            return []
        self.last_input=stamp;self.last_pose=(p,yaw);self.pending.append(copy.deepcopy(row));events=[]
        while self.next_grid<=stamp and (self.end_ns is None or self.next_grid<=self.end_ns):
            grid=self.next_grid
            eligible=[r for r in self.pending if r['stamp_ns']<=grid]
            selected=eligible[-1] if eligible else None
            if selected is None or selected['stamp_ns']<=self.last_sample_stamp or grid-selected['stamp_ns']>SOURCE_AGE_NS:
                missing=dict(grid_ns=grid,reason='no_distinct_fresh_causal_pose')
                self.missing.append(missing);events.append(dict(kind='missing',data=missing))
            else:events.append(dict(kind='sample',data=self._sample(grid,selected)))
            self.next_grid+=PERIOD_NS
            # Keep only the latest preceding pose plus unconsumed future poses.
            self.pending=([eligible[-1]] if eligible else [])+[r for r in self.pending if r['stamp_ns']>grid]
        return events

    def evidence(self):
        complete=self.end_ns is not None and self.next_grid>self.end_ns
        available=complete and not self.missing and len(self.samples)==51
        return dict(schema=CONFIG['schema'],configuration=copy.deepcopy(CONFIG),start_ns=self.start_ns,end_ns=self.end_ns,
            origin=copy.deepcopy(self.origin),samples=copy.deepcopy(self.samples),missing=copy.deepcopy(self.missing),
            complete=complete,data_available=available,motion_failure=copy.deepcopy(self.motion_failure),
            passed=bool(available and self.motion_failure is None),
            max_xy_from_anchor_m=max(s['xy_from_anchor_m']for s in self.samples),
            max_abs_yaw_from_anchor_rad=max(abs(s['yaw_from_anchor_rad'])for s in self.samples),
            max_adjacent_xy_m=max((s['adjacent_xy_m']or 0.)for s in self.samples),
            max_abs_adjacent_yaw_rad=max(abs(s['adjacent_yaw_rad']or 0.)for s in self.samples),
            high_frequency_motion_used_for_decision=False)

class CaptureGrid:
    """Original 25mm/0.035rad geometric dwell, using only distinct 10Hz poses."""
    def __init__(self):self.grid=None;self.since=None;self.close=False;self.last=None
    def observe(self,row,goal,heading):
        if self.grid is None:
            self.grid=PoseGrid(row['stamp_ns'],row,duration_ns=None)
            events=[dict(kind='sample',data=self.grid.samples[0])]
        else:events=self.grid.observe(row)
        for event in events:
            if event['kind']=='missing':self.since=None;self.close=False;continue
            s=event['data'];p=s['position'];self.last=s
            self.close=math.hypot(p[0]-goal[0],p[1]-goal[1])<=.025 and abs(wrap(heading-s['yaw_rad']))<=.035
            if not self.close:self.since=None
            elif self.since is None:self.since=s['grid_ns']
        return self.ready
    @property
    def ready(self):return self.close and self.since is not None and self.last['grid_ns']-self.since>=600_000_000
    def evidence(self):
        if not self.ready:return None
        samples=[copy.deepcopy(s)for s in self.grid.samples if s['grid_ns']>=self.since]
        # A finite geometric certificate sufficient for the unchanged .6s dwell.
        return dict(schema='parking_pose10hz_geometric_capture/v1',samples=samples[-7:],
            capture_xy_m=.025,capture_yaw_rad=.035,capture_dwell_ns=600_000_000,
            instantaneous_speed_or_gyro_used=False)

def validate_capture(certificate,goal,heading,origin):
    if not isinstance(certificate,dict)or certificate.get('schema')!='parking_pose10hz_geometric_capture/v1':
        raise ValueError('No 10Hz geometric capture proof')
    samples=certificate.get('samples',[])
    if len(samples)!=7:raise ValueError('10Hz capture requires seven real geometric samples')
    previous_grid=previous_stamp=None
    for sample in samples:
        stamp,p,yaw=pose(sample['source']);grid=sample['grid_ns']
        if (sample['source_stamp_ns']!=stamp or not 0<=grid-stamp<=SOURCE_AGE_NS
                or previous_grid is not None and grid-previous_grid!=PERIOD_NS
                or previous_stamp is not None and stamp<=previous_stamp
                or math.hypot(p[0]-goal[0],p[1]-goal[1])>.025 or abs(wrap(heading-yaw))>.035):
            raise ValueError('10Hz geometric capture source/spacing/geometry differs')
        previous_grid,previous_stamp=grid,stamp
    if samples[-1]['source']!=origin:raise ValueError('T0 origin differs from selected 10Hz capture pose')
    return True
