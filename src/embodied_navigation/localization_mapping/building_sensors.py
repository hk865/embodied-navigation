"""Building LiDAR self-echo filter. Geometry and original header only; no GT input."""
import copy,json,os,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation

class Sensors(Node):
 def __init__(self):
  super().__init__('building_lidar_geometry_filter')
  self.run=Path(os.environ['DEMO_RUN_DIR']);setup=json.loads((self.run/'experiment_setup.json').read_text());self.gap=setup.get('validation_lidar_gap_s');self.gap_after_tracking=setup.get('validation_lidar_gap_after_tracking_s');self.last_source_ns=0;self.motion_anchor=None;self.motion_count=0;self.gap_trigger=None;self.validation_dropped=0;self.first_dropped_ns=None;self.last_dropped_ns=None;cfg=json.loads((self.run/'sensor_contract.json').read_text())['lidar']
  self.rotation=Rotation.from_euler('xyz',cfg['body_rpy']).as_matrix();self.offset=np.asarray(cfg['body_xyz'])
  self.publisher=self.create_publisher(PointCloud2,'/building/lidar_filtered',qos_profile_sensor_data)
  self.create_subscription(PointCloud2,'/demo/teacher/raw_lidar',self.cloud,qos_profile_sensor_data)
  self.received=0;self.removed=0;self.errors=0;self.last=0;self.create_timer(1.,self.status)
 def cloud(self,msg):
  source_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec;self.last_source_ns=source_ns
  if self.gap is not None and self.gap[0]*1e9<=source_ns<(self.gap[0]+self.gap[1])*1e9:
   self.validation_dropped+=1;self.last_dropped_ns=source_ns
   if self.first_dropped_ns is None:self.first_dropped_ns=source_ns
   return
  if msg.width*msg.height>65536 or len(msg.data)>2*1024*1024:self.errors+=1;return
  try:
   records=point_cloud2.read_points(msg,skip_nans=False).reshape(-1)
   xyz=np.column_stack([records[k]for k in ('x','y','z')]);finite=np.isfinite(xyz).all(1);body=np.zeros_like(xyz);body[finite]=xyz[finite]@self.rotation.T+self.offset
   inside=(abs(body[:,0])<.5)&(abs(body[:,1])<.30)&(body[:,2]>-.45)&(body[:,2]<.30)
   keep=np.isfinite(xyz).all(1)&~inside
   out=point_cloud2.create_cloud(msg.header,msg.fields,records[keep],point_step=msg.point_step);self.publisher.publish(out)
   self.received+=1;self.removed+=int((~keep).sum());self.last=time.monotonic_ns()
  except(ValueError,KeyError,TypeError):self.errors+=1
 def status(self):
  if self.gap is None and self.gap_after_tracking is not None and self.last_source_ns:
   try:
    nav=json.loads((self.run/'truth_navigation_status.json').read_text())
    moving=nav['state']=='tracking'and any(nav['command'])and nav.get('pose_source_age_ms',1000)<300 and nav.get('body_speed_mps',0)>.08
    if moving:
     point=np.asarray(nav['pose_nav']);self.motion_count+=1
     if self.motion_anchor is None:self.motion_anchor=point.copy()
     displacement=float(np.linalg.norm(point-self.motion_anchor))
     if self.motion_count>=2 and displacement>=.1:
      self.gap=[self.last_source_ns/1e9+self.gap_after_tracking[0],self.gap_after_tracking[1]]
      self.gap_trigger=dict(tracking_samples=self.motion_count,measured_body_speed_mps=nav['body_speed_mps'],measured_SLAM_displacement_m=displacement,source_clock_s=self.last_source_ns/1e9,nav_clock_s=nav['clock_s'],command=nav['command'],navigation_ground_truth_used=False)
    else:self.motion_count=0;self.motion_anchor=None
   except(OSError,ValueError,KeyError):pass
  p=self.run/'lidar_filter_status.json';q=p.with_suffix('.tmp');q.write_text(json.dumps(dict(received=self.received,validation_lidar_gap_s=self.gap,validation_lidar_gap_after_tracking_s=self.gap_after_tracking,validation_dropped=self.validation_dropped,validation_gap_trigger=self.gap_trigger,first_dropped_ns=self.first_dropped_ns,last_dropped_ns=self.last_dropped_ns,removed=self.removed,errors=self.errors,last_receipt_wall_ns=self.last,navigation_ground_truth_used=False))+'\n');q.replace(p)

def main():
 rclpy.init();node=Sensors()
 try:rclpy.spin(node)
 except(KeyboardInterrupt,ExternalShutdownException):pass
 except RuntimeError:
  # rclpy can race take_message with its SIGINT context shutdown. Preserve
  # every healthy-context runtime error rather than masking sensor faults.
  if rclpy.ok():raise
 finally:node.status();node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
