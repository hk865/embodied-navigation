#!/usr/bin/env python3
"""Finite read-only recorder: measured LIO map clouds, exact stamp LIO odometry pairs."""
import os,time,json,struct,argparse,math,re
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

parser=argparse.ArgumentParser(description='Finite read-only same-stage LIO cloud/pose recorder; no commands are published.')
parser.add_argument('--run-id',required=True)
parser.add_argument('--output-dir',type=Path,required=True)
parser.add_argument('--budget-wall-s',type=float,default=5400.0)
parser.add_argument('--save-every-wall-s',type=float,default=5.0)
args=parser.parse_args()
RUN_ID=args.run_id;OUT=args.output_dir;BUDGET=args.budget_wall_s;SAVE_EVERY=args.save_every_wall_s;VOX=.05
if not all(math.isfinite(v) and v>0 for v in (BUDGET,SAVE_EVERY)):parser.error('budget and save interval must be finite and positive')
OUT.mkdir(parents=True,exist_ok=True)
if any(OUT.iterdir()):raise SystemExit('Output directory is not empty; use a new current-generation collection directory.')
os.environ.setdefault('ROS_LOG_DIR',str(OUT/'ros_logs'))
cloud_topic='/dynanav/rtx_livo/cloud_registered_full'
pose_topic='/dynanav/rtx_livo/aft_mapped_to_init'
def stamp(m):return int(m.header.stamp.sec)*1000000000+int(m.header.stamp.nanosec)
class Recorder(Node):
 def __init__(self):
  super().__init__('read_only_lio_cloud_audit_'+re.sub('[^A-Za-z0-9_]','_',RUN_ID).lower())
  self.clouds={};self.poses={};self.vox={};self.events=[];self.received=0;self.bad=0;self.fields=None;self.latest_xyz=None;self.latest_event=None;self.start=time.monotonic();self.saves=0
  q=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE,history=HistoryPolicy.KEEP_LAST)
  self.create_subscription(PointCloud2,cloud_topic,self.cloud,q)
  self.create_subscription(Odometry,pose_topic,self.pose,q)
  self.log=open(OUT/'paired_source_stamps.jsonl','a',buffering=1)
 def cloud(self,m):
  self.received+=1;self.clouds[stamp(m)]=m;self.pair(stamp(m));self.prune()
 def pose(self,m):
  self.poses[stamp(m)]=m;self.pair(stamp(m));self.prune()
 def prune(self):
  for d in (self.clouds,self.poses):
   if len(d)>300:
    for k in sorted(d)[:-200]:d.pop(k,None)
 def pair(self,s):
  if s not in self.clouds or s not in self.poses:return
  c=self.clouds.pop(s);p=self.poses.pop(s)
  fs={f.name:f for f in c.fields}
  if not all(k in fs for k in ('x','y','z')):self.bad+=1;return
  dtmap={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
  endian='>' if c.is_bigendian else '<'
  dt=np.dtype({'names':['x','y','z'],'formats':[endian+dtmap[fs[k].datatype] for k in ('x','y','z')],'offsets':[fs[k].offset for k in ('x','y','z')],'itemsize':c.point_step})
  if c.row_step==c.width*c.point_step:a=np.frombuffer(c.data,dtype=dt,count=c.width*c.height)
  else:a=np.concatenate([np.frombuffer(memoryview(c.data)[i*c.row_step:i*c.row_step+c.width*c.point_step],dtype=dt) for i in range(c.height)])
  xyz=np.column_stack([a[k] for k in ('x','y','z')]).astype(np.float32)
  xyz=xyz[np.isfinite(xyz).all(axis=1)]
  ids=np.floor(xyz/VOX).astype(np.int32);_,ix=np.unique(ids,axis=0,return_index=True)
  for k,v in zip(ids[ix],xyz[ix]):self.vox.setdefault(tuple(map(int,k)),v)
  po=p.pose.pose;ev={'source_stamp_ns':s,'source_stamp_s':s/1e9,'cloud_frame':c.header.frame_id,'pose_frame':p.header.frame_id,'pose_child':p.child_frame_id,'stage':'LIO','exact_integer_stamp_pair':True,'cloud_point_count':len(xyz),'voxel_count':len(self.vox),'pose_xyz':[po.position.x,po.position.y,po.position.z],'pose_xyzw':[po.orientation.x,po.orientation.y,po.orientation.z,po.orientation.w],'recorded_wall_unix_s':time.time()}
  self.latest_xyz=xyz;self.latest_event=ev.copy()
  self.events.append(ev);self.log.write(json.dumps(ev)+'\n');self.fields=[{'name':f.name,'offset':f.offset,'datatype':f.datatype,'count':f.count} for f in c.fields]
 def save(self,final=False):
  if self.latest_xyz is not None:
   e=self.latest_event
   latest_tmp=OUT/'latest_paired_cloud.tmp.npz'
   np.savez_compressed(latest_tmp,xyz=self.latest_xyz,source_stamp_ns=np.asarray([e['source_stamp_ns']],dtype=np.int64),source_stamp_s=np.array(e['source_stamp_s']),lio_pose_xyz=np.asarray([e['pose_xyz']],dtype=np.float64),lio_pose_xyzw=np.asarray([e['pose_xyzw']],dtype=np.float64),run_id=np.array(RUN_ID),cloud_map_frame=np.array(e['cloud_frame']),pose_map_frame=np.array(e['pose_frame']),pose_child_frame=np.array(e['pose_child']),cloud_stage=np.array('LIO'),exact_integer_stamp_pair=np.array(True),data_semantics=np.array('single latest paired LIO deskewed map-frame cloud; no second transform; finite xyz; not accumulated'))
   os.replace(latest_tmp,OUT/'latest_paired_cloud.npz')
  xyz=np.asarray(list(self.vox.values()),dtype=np.float32).reshape(-1,3)
  stamps=np.asarray([e['source_stamp_ns'] for e in self.events],dtype=np.int64)
  poses=np.asarray([e['pose_xyz'] for e in self.events],dtype=np.float64).reshape(-1,3)
  voxel_counts=np.asarray([e['voxel_count'] for e in self.events],dtype=np.int64)
  voxel_first_source_ns=np.repeat(stamps,np.diff(np.r_[0,voxel_counts]))
  tmp=OUT/'measured_partial_cloud.tmp.npz';np.savez_compressed(tmp,xyz=xyz,voxel_m=np.array(VOX),source_stamp_ns=stamps,lio_pose_xyz=poses,voxel_first_source_ns=voxel_first_source_ns,run_id=np.array(RUN_ID),cloud_map_frame=np.array(self.events[-1]['cloud_frame'] if self.events else ''),cloud_stage=np.array('LIO'));os.replace(tmp,OUT/'measured_partial_cloud.npz')
  ply=OUT/'measured_partial_cloud.tmp.ply'
  with open(ply,'wb') as f:
   f.write(('ply\nformat binary_little_endian 1.0\ncomment measured LIO cloud; no second transform; partial current-run collection\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n'%len(xyz)).encode());f.write(xyz.astype('<f4').tobytes())
  os.replace(ply,OUT/'measured_partial_cloud.ply')
  summary={'run':RUN_ID,'ros_domain_id':os.environ.get('ROS_DOMAIN_ID'),'ground_truth_used':False,'cloud_map_frame':self.events[-1]['cloud_frame'] if self.events else None,'cloud_stage':'LIO','topics':{'cloud':cloud_topic,'pose':pose_topic},'input_semantics':'LIO deskewed map-frame cloud; no second transform; same-stage exact integer stamp paired with LIO odometry','voxel_m':VOX,'voxel_count':len(xyz),'received_clouds':self.received,'accepted_pairs':len(self.events),'invalid_clouds':self.bad,'latest_paired_cloud_npz':str(OUT/'latest_paired_cloud.npz'),'latest_paired_source_stamp_s':self.latest_event['source_stamp_s'] if self.latest_event else None,'pending_clouds':len(self.clouds),'first_source_stamp_s':stamps[0]/1e9 if len(stamps) else None,'last_source_stamp_s':stamps[-1]/1e9 if len(stamps) else None,'xyz_min':xyz.min(axis=0).tolist() if len(xyz) else None,'xyz_max':xyz.max(axis=0).tolist() if len(xyz) else None,'cloud_fields':self.fields,'elapsed_wall_s':time.monotonic()-self.start,'wall_budget_s':BUDGET,'final':final,'limitation':'Collection began after simulation start. This is a partial current-run cloud, not full-run map coverage; voxel occupancy does not establish traversability or room completeness.'}
  tmp=OUT/'summary.tmp.json';tmp.write_text(json.dumps(summary,indent=2));os.replace(tmp,OUT/'summary.json');print(json.dumps({k:summary[k] for k in ['run','accepted_pairs','voxel_count','first_source_stamp_s','last_source_stamp_s','elapsed_wall_s','final']},ensure_ascii=False),flush=True)
rclpy.init();n=Recorder();last=time.monotonic()
try:
 while rclpy.ok() and time.monotonic()-n.start<BUDGET and not (OUT/'stop_recorder').exists():
  rclpy.spin_once(n,timeout_sec=.1)
  if time.monotonic()-last>SAVE_EVERY:n.save();last=time.monotonic()
except (KeyboardInterrupt, ExternalShutdownException):
 pass
finally:
 n.save(final=True);n.log.close();n.destroy_node();rclpy.try_shutdown()
