#!/usr/bin/env python3
"""Publish this generation's lossless native archive for bag/RViz; never feeds SLAM.

Extra views retain archive source stamps and their measured 1 Hz cadence. Dynamic
TF is a labelled LIO odometry copy for viewing, not a new pose estimator.
"""
from __future__ import annotations
import argparse,hashlib,json,os,time
from pathlib import Path
import numpy as np

def stamp(message,ns,frame):
 message.header.stamp.sec=int(ns)//1_000_000_000;message.header.stamp.nanosec=int(ns)%1_000_000_000;message.header.frame_id=frame

def resolve_inside(root,name):
 p=(root/name).resolve()
 if not p.is_relative_to(root.resolve()):raise ValueError('Archive path escapes generation')
 return p

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--capture',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--budget-wall-s',type=float,required=True)
 args=parser.parse_args();capture=args.capture.resolve();archive=capture/'multicamera';args.output.mkdir(parents=True,exist_ok=False)
 import rclpy
 from rclpy.node import Node
 from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
 from rclpy.executors import ExternalShutdownException
 from sensor_msgs.msg import Image,CompressedImage,CameraInfo
 from nav_msgs.msg import Odometry
 from std_msgs.msg import String
 from geometry_msgs.msg import TransformStamped
 from tf2_ros import StaticTransformBroadcaster,TransformBroadcaster
 from scipy.spatial.transform import Rotation
 from embodied_navigation.simulation.dynanav.multicamera import validate_intrinsics
 rclpy.init();node=Node('dynanav_multicamera_archive_bridge');qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE)
 retained=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
 calibration_pub=node.create_publisher(String,'/dynanav/multicamera/calibration',retained)
 static=StaticTransformBroadcaster(node);dynamic=TransformBroadcaster(node);odom_count=0
 def odom(msg):
  nonlocal odom_count
  if msg.header.frame_id!='dynanav_lio_map' or msg.child_frame_id!='go2_imu':raise ValueError('Unexpected LIO frames for viewing TF')
  tf=TransformStamped();tf.header=msg.header;tf.child_frame_id=msg.child_frame_id
  p=msg.pose.pose.position;tf.transform.translation.x=p.x;tf.transform.translation.y=p.y;tf.transform.translation.z=p.z;tf.transform.rotation=msg.pose.pose.orientation
  dynamic.sendTransform(tf);odom_count+=1
 node.create_subscription(Odometry,'/dynanav/rtx_livo/aft_mapped_to_init',odom,qos)
 start=time.monotonic();cal=None;pubs={};counts={};cursor=0;pending='';first=None;last=None;last_save=0;error=None
 def save(final=False):
  value=dict(generation=str(capture),ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),source='native RGB/depth archive, original integer source stamps',archive_rate_hz=1.,extra_camera_slam_input=False,estimator_feedback_enabled=False,counts=counts,first_source_ns=first,last_source_ns=last,lio_derived_view_tf_messages=odom_count,static_tf_source='fixed authored calibration, imu to optical',dynamic_tf_source='unchanged exact LIO odometry message pose/time; native publish_base_tf=false',elapsed_wall_s=time.monotonic()-start,final=final,error=error)
  temp=args.output/'summary.tmp.json';temp.write_text(json.dumps(value,indent=2));temp.replace(args.output/'summary.json')
 try:
  with (args.output/'published_frames.jsonl').open('w') as journal:
   while rclpy.ok() and time.monotonic()-start<args.budget_wall_s and not (args.output/'stop_bridge').exists():
    rclpy.spin_once(node,timeout_sec=.02)
    if cal is None:
     path=archive/'calibration.json'
     if not path.exists():continue
     cal=json.loads(path.read_text())
     if cal['generation']!=str(capture):raise ValueError('Wrong native archive generation')
     transforms=[]
     for c in cal['cameras']:
      name=c['camera_id'];base='/dynanav/multicamera/'+name
      pubs[name]=(node.create_publisher(CompressedImage,base+'/image/compressed',qos),node.create_publisher(Image,base+'/depth',qos),node.create_publisher(CameraInfo,base+'/camera_info',qos));counts[name]=0
      R=np.array(c['R_camera_from_imu']);t=np.array(c['translation_camera_from_imu_m']);q=Rotation.from_matrix(R.T).as_quat();translation=-R.T@t
      for child in ([c['optical_frame'],'go2_camera_optical'] if name=='camera_0' else [c['optical_frame']]):
       tf=TransformStamped();tf.header.frame_id='go2_imu';tf.child_frame_id=child
       tf.transform.translation.x=float(translation[0]);tf.transform.translation.y=float(translation[1]);tf.transform.translation.z=float(translation[2])
       tf.transform.rotation.x=float(q[0]);tf.transform.rotation.y=float(q[1]);tf.transform.rotation.z=float(q[2]);tf.transform.rotation.w=float(q[3]);transforms.append(tf)
     static.sendTransform(transforms);manifest=String();manifest.data=json.dumps(cal);calibration_pub.publish(manifest)
     (args.output/'calibration.json').write_text(json.dumps(cal,indent=2))
    index=archive/'index.jsonl'
    if index.exists():
     with index.open() as f:f.seek(cursor);pending+=f.read();cursor=f.tell()
     lines=pending.split('\n');pending=lines.pop()
     for line in lines:
      if not line:continue
      row=json.loads(line);name=row['camera_id'];c=next(c for c in cal['cameras'] if c['camera_id']==name)
      if row['generation']!=str(capture) or row['time_evidence']['body_now_substituted']:raise ValueError('Wrong generation or rewritten source time')
      if 'native_camera_parameters' in row:validate_intrinsics(dict(row['native_camera_parameters'],cameraModel=row['native_camera_model']),c)
      data=resolve_inside(archive,row['rgb_path']).read_bytes()
      rgb=CompressedImage();stamp(rgb,row['stamp_ns'],c['optical_frame']);rgb.format='rgb8; png compressed rgb8';rgb.data=data
      with np.load(resolve_inside(archive,row['depth_path']),allow_pickle=False) as d:depth=np.ascontiguousarray(d['axial_depth_m'],dtype='<f4')
      if depth.shape!=(512,512):raise ValueError('Wrong native depth dimensions')
      dm=Image();stamp(dm,row['stamp_ns'],c['optical_frame']);dm.height,dm.width=depth.shape;dm.encoding='32FC1';dm.is_bigendian=False;dm.step=512*4;dm.data=depth.tobytes()
      info=CameraInfo();stamp(info,row['stamp_ns'],c['optical_frame']);info.width=512;info.height=512;fx,fy,cx,cy=c['intrinsics_fx_fy_cx_cy'];info.k=[fx,0.,cx,0.,fy,cy,0.,0.,1.];info.p=[fx,0.,cx,0.,0.,fy,cy,0.,0.,0.,1.,0.];info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.];info.d=[0.]*5;info.distortion_model='plumb_bob'
      for pub,msg in zip(pubs[name],(rgb,dm,info)):pub.publish(msg)
      counts[name]+=1;first=row['stamp_ns'] if first is None else min(first,row['stamp_ns']);last=row['stamp_ns'] if last is None else max(last,row['stamp_ns'])
      journal.write(json.dumps(dict(camera_id=name,source_stamp_ns=row['stamp_ns'],frame_id=c['optical_frame'],png_sha256=hashlib.sha256(data).hexdigest(),source_rgb_path=str(resolve_inside(archive,row['rgb_path'])),published_local=True,delivery_confirmed=False))+'\n');journal.flush()
    if time.monotonic()-last_save>1:save();last_save=time.monotonic()
 except (KeyboardInterrupt,ExternalShutdownException):pass
 except Exception as exc:error=repr(exc);raise
 finally:save(final=True);node.destroy_node();rclpy.try_shutdown()
 return 0
if __name__=='__main__':raise SystemExit(main())
