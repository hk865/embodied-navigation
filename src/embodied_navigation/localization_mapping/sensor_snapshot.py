"""Bounded live SLAM pose/cloud pairing by actual source time and frame.

No origin audit, source manifest, message hash or recorded callback is required.
"""
from collections import OrderedDict
import threading,time
import numpy as np
from scipy.spatial.transform import Rotation
from embodied_navigation.common.integration_limits import SOURCE_JOIN_SLOT_MAX_AGE_NS


def stamp(message):
    return message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec


def transform(message):
    p=message.pose.pose.position;q=message.pose.pose.orientation
    position=np.array([p.x,p.y,p.z]);quaternion=np.array([q.x,q.y,q.z,q.w])
    if not np.isfinite(position).all() or not np.isfinite(quaternion).all() or abs(quaternion@quaternion-1)>.001:raise ValueError('Invalid SLAM pose')
    value=np.eye(4);value[:3,:3]=Rotation.from_quat(quaternion).as_matrix();value[:3,3]=position
    return value


def xyz(message):
    fields={f.name:f for f in message.fields};endian='>'if message.is_bigendian else'<'
    if message.row_step<message.width*message.point_step or len(message.data)<message.height*message.row_step:raise ValueError('Truncated cloud')
    values=[]
    for name in ('x','y','z'):
        f=fields[name]
        if f.datatype!=7 or f.count!=1 or f.offset<0 or f.offset+4>message.point_step:raise ValueError('Invalid cloud XYZ layout')
        values.append(np.ndarray((message.height,message.width),dtype=endian+'f4',buffer=message.data,offset=f.offset,strides=(message.row_step,message.point_step)).reshape(-1))
    result=np.column_stack(values).astype(float)
    if not len(result)or not np.isfinite(result).all():raise ValueError('Nonfinite/empty cloud')
    return result


class SensorSnapshots:
    def __init__(self,maximum_age_ns=SOURCE_JOIN_SLOT_MAX_AGE_NS):
        self.maximum_age=maximum_age_ns;self.slots=OrderedDict();self.lock=threading.Lock();self.last=0
    def put(self,role,message):
        if role not in ('body','registered'):return
        ns=stamp(message)
        with self.lock:
            self.slots.setdefault(ns,{})[role]=(message,time.monotonic_ns())
            while len(self.slots)>12:self.slots.popitem(last=False)
    def pop_latest(self,clock):
        now=time.monotonic_ns()
        with self.lock:
            available=[ns for ns,row in self.slots.items()if ns<=clock and ns>self.last and all(k in row for k in ('body','registered'))]
            if not available:return None
            ns=max(available);row=self.slots[ns]
            for key in list(self.slots):
                if key<=ns:del self.slots[key]
            self.last=ns
        if not 0<=clock-ns<=self.maximum_age or any(not 0<=now-wall<=self.maximum_age for _,wall in row.values()):return None
        body=row['body'][0];cloud=row['registered'][0]
        if body.header.frame_id!=cloud.header.frame_id or body.child_frame_id!='demo_slam_body':return None
        pose=transform(body);registered=xyz(cloud)
        # An ordinary rigid coordinate transform of actual post-LIO points;
        # no synthetic cloud, pose interpolation or old callback correspondence.
        local=(registered-pose[:3,3])@pose[:3,:3]
        return dict(source_ns=ns,body_points=local,T_odom_body=pose,registered_odom_xyz=registered,
            provenance=dict(source='live post-LIO pose/cloud',source_ns=ns,no_robot_truth_pose=True))


class BackgroundMatcher:
    def __init__(self,matcher):
        self.matcher=matcher;self.condition=threading.Condition();self.job=None;self.result=None;self.closed=False
        self.thread=threading.Thread(target=self._work,daemon=True,name='scene-matching');self.thread.start()
    def submit(self,job,current,generation):
        if job is None:return
        with self.condition:self.job=(job,np.array(current,copy=True),generation);self.condition.notify()
    def poll(self):
        with self.condition:value,self.result=self.result,None;return value
    def _work(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda:self.closed or self.job is not None)
                if self.closed:return
                (job,current,generation),self.job=self.job,None
            try:
                result=self.matcher.match(job['source_ns'],job['body_points'],job['T_odom_body'],current_c=current,provenance=job['provenance'])
                result.update(T_odom_body=job['T_odom_body'].tolist(),T_odom_body_source_ns=job['source_ns'],parent_generation=generation)
            except Exception as error:result=dict(accepted=False,source_ns=job['source_ns'],reason=str(error))
            with self.condition:self.result=result
    def close(self):
        with self.condition:self.closed=True;self.condition.notify()
        self.thread.join(timeout=5)
