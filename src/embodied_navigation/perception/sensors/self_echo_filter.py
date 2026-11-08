#!/usr/bin/env python3
"""Remove only Go2's known physical envelope from actual sensor scans.

The transform is the fixed URDF/scenario lidar extrinsic. No SLAM pose, world
trajectory or truth is read. The surviving records retain every original
field byte and the original acquisition timestamp.
"""

from embodied_navigation.project import project_root as _project_root
import argparse
import sys,os,uuid,base64,hashlib
from collections import OrderedDict
from embodied_navigation.localization_mapping.point_provenance import message_sha, decode, MAX_META
from collections import Counter
import copy
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from embodied_navigation.localization_mapping.geometry import sensor_transform


def self_mask(points_body):
    x,y,z=points_body.T
    trunk=(abs(x)<=.30)&(abs(y)<=.23)&(z>=-.10)&(z<=.20)
    moving_legs=(abs(x)<=.45)&(abs(y)<=.28)&(z>=-.36)&(z<=-.07)
    return trunk|moving_legs


def filter_records(msg, lidar,with_identity=False):
    fields={f.name:f for f in msg.fields}
    if any(name not in fields or fields[name].datatype!=7 or fields[name].count!=1 for name in ('x','y','z')):
        raise ValueError('sensor cloud requires scalar float32 XYZ')
    offsets=[fields[name].offset for name in ('x','y','z')]
    if any(offset<0 or offset+4>msg.point_step for offset in offsets)\
            or msg.row_step<msg.width*msg.point_step or len(msg.data)<msg.height*msg.row_step:
        raise ValueError('malformed sensor cloud field/row layout')
    endian='>' if msg.is_bigendian else '<'
    dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,'offsets':offsets,'itemsize':msg.point_step})
    values=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
        strides=(msg.row_step,msg.point_step)).reshape(-1)
    xyz=np.column_stack([values[name] for name in ('x','y','z')])
    offset,rotation=sensor_transform(lidar)
    finite=np.isfinite(xyz).all(axis=1)
    body=np.full(xyz.shape,np.nan,dtype=float)
    body[finite]=xyz[finite]@rotation.T+offset
    removed=self_mask(body)&finite
    raw=np.ndarray((msg.height,msg.width),dtype=np.dtype(f'V{msg.point_step}'),buffer=msg.data,
        strides=(msg.row_step,msg.point_step)).reshape(-1)
    result=(raw[~removed].tobytes(),int(removed.sum()),len(raw))
    return (*result,np.flatnonzero(~removed).astype("<u4")) if with_identity else result


class SelfEchoFilter(Node):
    def __init__(self, scenario):
        super().__init__('demo_slam_self_echo_filter')
        self.lidar=json.loads(Path(scenario).read_text())['sensors']['lidar']
        # FAST-LIVO2 requests reliable delivery. A sensor-data best-effort
        # publisher would silently have no compatible LiDAR subscriber.
        self.pub=self.create_publisher(PointCloud2,'/demo/slam/lidar_filtered',10)
        self.identity_pub=self.create_publisher(String,"/demo/slam/raw_point_identity",10)
        self.identity_epoch=uuid.uuid4().hex
        self.status=self.create_publisher(String,'/demo/slam/lidar_filter_status',10)
        self.create_subscription(PointCloud2,'/livox/lidar',self.receive,qos_profile_sensor_data)
        self.counts=Counter();self.last_stamp=None;self.last_wall=None;self.error=None
        self.create_timer(1.,self.report)

    def receive(self,msg):
        if msg.header.frame_id!=self.lidar['frame']:
            self.counts['rejected_frame']+=1;self.error='LiDAR frame does not match fixed physical extrinsic';return
        if msg.height*msg.width>65536 or len(msg.data)>2*1024*1024:
            self.counts['rejected_capacity']+=1;self.error='R60 original complete cloud exceeds source cap';return
        try:
            payload,removed,total,ordinals=filter_records(msg,self.lidar,with_identity=True)
        except ValueError as error:
            self.counts['rejected_format']+=1;self.error=str(error);return
        out=copy.deepcopy(msg);out.data=payload;out.height=1
        out.width=total-removed;out.row_step=out.width*out.point_step
        if total>65536 or out.width>65536:raise ValueError('R60 raw source count cap')
        ids=ordinals.tobytes()
        proof=dict(schema='raw_filtered_point_identity/v1',fingerprint_model='go2_ros_semantic/v1',producer_epoch=self.identity_epoch,
            producer_pid=os.getpid(),run_id=Path(os.environ['DEMO_RUN_DIR']).name,
            stamp_ns=int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec),
            raw_message_sha256=message_sha(msg),adapted_message_sha256=message_sha(out),
            raw_count=total,adapted_count=int(out.width),raw_ordinals_sha256=hashlib.sha256(ids).hexdigest(),
            raw_ordinals_base64=base64.b64encode(ids).decode('ascii'),navigation_ground_truth_used=False)
        self.identity_pub.publish(String(data=json.dumps(proof,allow_nan=False)))
        self.pub.publish(out)
        self.counts['scans']+=1;self.counts['input_points']+=total;self.counts['removed_self_points']+=removed
        self.last_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.last_wall=time.monotonic();self.error=None

    def report(self):
        self.status.publish(String(data=json.dumps({'input':'/livox/lidar','output':'/demo/slam/lidar_filtered',
            'transform':'fixed sensor-to-body physical extrinsic','counts':dict(self.counts),
            'last_stamp':self.last_stamp,'age':None if self.last_wall is None else time.monotonic()-self.last_wall,
            'ground_truth_used':False,'slam_pose_used':False,'error':self.error})))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--scenario',default=str(_project_root()/'configs/simulation/scenario.json'))
    args,ros=parser.parse_known_args();rclpy.init(args=ros);node=SelfEchoFilter(args.scenario)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    except _ros.RCLError:
        # SIGINT can invalidate the context between executor wait-set calls.
        # Errors while the context is live must still fail this required node.
        if rclpy.ok():raise
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
