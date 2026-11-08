"""Sole Building correction owner: pair LIO pose/cloud and capture one SE3 correction.

No native truth files or topics are read. Configured spawn is an explicit prior.
The old known_scene/localization_runtime is never launched with this owner.
"""
import copy,json,os,time,collections
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from .backend.correction import BackendResultReader
from .backend.geometry import checked_transform

def ns(msg):return msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
def transform(msg):
 p=msg.pose.pose;out=np.eye(4);out[:3,:3]=Rotation.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]).as_matrix();out[:3,3]=[p.position.x,p.position.y,p.position.z];return out
def set_pose(msg,T):
 p=msg.pose.pose;p.position.x,p.position.y,p.position.z=map(float,T[:3,3]);p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,Rotation.from_matrix(T[:3,:3]).as_quat())
class Localization(Node):
 def __init__(self):
  super().__init__('building_global_correction_owner');self.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
  self.run=Path(os.environ['DEMO_RUN_DIR']);setup=json.loads((self.run/'experiment_setup.json').read_text());self.origin=np.asarray(setup['navigation_world_origin'])
  self.C=np.eye(4);self.C[:3,3]=self.origin;self.navigation=np.eye(4);self.navigation[:3,3]=-self.origin
  self.corrected=setup.get('backend_capture_only')is not True;self.reader=BackendResultReader(self.run/'backend_result.json')
  self.poses=collections.OrderedDict();self.pending=collections.OrderedDict();self.generation=0;self.applied=0;self.localized=False;self.pairs=0;self.last=0;self.cloud_age=0
  sensor=json.loads((self.run/'sensor_contract.json').read_text())['lidar'];self.extrinsic=np.eye(4);self.extrinsic[:3,:3]=Rotation.from_euler('xyz',sensor['body_rpy']).as_matrix();self.extrinsic[:3,3]=sensor['body_xyz']
  prefix='/building/capture'if not self.corrected else'/demo/slam'
  self.body=self.create_publisher(Odometry,prefix+'/body_odom',qos_profile_sensor_data);self.lidar=self.create_publisher(Odometry,prefix+'/lidar_odom',qos_profile_sensor_data)
  self.scan=self.create_publisher(Odometry,'/building/scan_body_odom',qos_profile_sensor_data)
  self.cloudpub=self.create_publisher(PointCloud2,'/cloud_registered_full'if self.corrected else'/building/capture/cloud',qos_profile_sensor_data)
  self.applyfile=(self.run/'correction_applications.jsonl').open('x',buffering=1)
  self.tf=TransformBroadcaster(self);self.posefile=(self.run/'slam_poses.jsonl').open('x',buffering=1)
  self.create_subscription(Odometry,'/building/raw_body_odom',self.pose,qos_profile_sensor_data)
  self.create_subscription(PointCloud2,'/building/raw_cloud',self.cloud,qos_profile_sensor_data)
  self.create_timer(.02,self.poll);self.create_timer(1.,self.status)
 def pose(self,msg):
  self.poses[ns(msg)]=msg
  while len(self.poses)>32:self.poses.popitem(last=False)
  self.commit(ns(msg))
 def cloud(self,msg):
  self.pending[ns(msg)]=msg
  while len(self.pending)>4:self.pending.popitem(last=False)
  self.commit(ns(msg))
 def poll(self):
  proposal=self.reader.poll(self.get_clock().now().nanoseconds,self.generation)
  if proposal and proposal.get('accepted'):
   C=checked_transform(proposal['C_world_odom'])
   # Tracking proposals cannot silently jump between repeated floors.
   if np.linalg.norm(C[:3,3]-self.C[:3,3])>.5:return
   if Rotation.from_matrix(C[:3,:3]@self.C[:3,:3].T).magnitude()>.2:return
   self.C=C;self.generation+=1;self.applied+=1;self.localized=True
   applied_clock_ns=self.get_clock().now().nanoseconds
   self.applyfile.write(json.dumps(dict(generation=self.generation,sequence=proposal['sequence'],source_ns=proposal['source_ns'],applied_clock_ns=applied_clock_ns,source_to_apply_sim_ms=(applied_clock_ns-proposal['source_ns'])/1e6,backend_publish_to_apply_wall_ms=(time.time_ns()-proposal['published_wall_ns'])/1e6,navigation_ground_truth_used=False))+'\n')
 def commit(self,stamp):
  if stamp not in self.poses or stamp not in self.pending:return
  pose=self.poses.pop(stamp);cloud=self.pending.pop(stamp);Traw=transform(pose);C=self.C.copy();T=self.navigation@C@Traw
  body=copy.deepcopy(pose);body.header.frame_id='map';body.child_frame_id='base_link';set_pose(body,T)
  publish=self.localized or not self.corrected
  if publish:self.body.publish(body)
  sensor=copy.deepcopy(body);sensor.child_frame_id='lidar';set_pose(sensor,T@self.extrinsic)
  if publish:self.lidar.publish(sensor)
  scan=copy.deepcopy(body);v=np.array([body.twist.twist.linear.x,body.twist.twist.linear.y,body.twist.twist.linear.z]);w=np.array([body.twist.twist.angular.x,body.twist.twist.angular.y,body.twist.twist.angular.z]);scan.twist.twist.linear.x,scan.twist.twist.linear.y,scan.twist.twist.linear.z=map(float,T[:3,:3]@v);scan.twist.twist.angular.x,scan.twist.twist.angular.y,scan.twist.twist.angular.z=map(float,T[:3,:3]@w)
  if publish:self.scan.publish(scan)
  points=point_cloud2.read_points_numpy(cloud,field_names=('x','y','z'),skip_nans=True);A=self.navigation@C;points=points@A[:3,:3].T+A[:3,3];header=copy.deepcopy(cloud.header);header.frame_id='map'
  if publish:self.cloudpub.publish(point_cloud2.create_cloud_xyz32(header,points.astype(np.float32)))
  self.pairs+=1;self.last=stamp;self.cloud_age=(self.get_clock().now().nanoseconds-stamp)/1e6
  self.posefile.write(json.dumps(dict(stamp_ns=stamp,raw_T_odom_body=Traw.tolist(),T_world_body=(C@Traw).tolist(),T_nav_body=T.tolist(),generation=self.generation,source_age_ms=self.cloud_age,received_wall_ns=time.monotonic_ns(),navigation_ground_truth_used=False))+'\n')
  if self.corrected and publish:
   tf=TransformStamped();tf.header=body.header;tf.child_frame_id='odom';M=self.navigation@C;tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,M[:3,3]);tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,Rotation.from_matrix(M[:3,:3]).as_quat());self.tf.sendTransform(tf)
 def status(self):
  p=self.run/'slam_status.json';q=p.with_suffix('.tmp');q.write_text(json.dumps(dict(pairs=self.pairs,last_source_ns=self.last,source_age_ms=self.cloud_age,corrections_applied=self.applied,correction_owner='building_global_correction_owner',configured_spawn_prior=True,localized=self.localized,navigation_ground_truth_used=False))+'\n');q.replace(p)

def main():
 rclpy.init();node=Localization()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.status();node.posefile.close();node.applyfile.close();node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
