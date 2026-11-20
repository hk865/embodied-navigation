"""Opt-in synthetic ROS integration test, no simulator or scene assets.

Source ROS + the newly-built SCAN overlay, then set DYNANAV_SCAN_TEST_PREFIX
and a free isolated ROS_DOMAIN_ID before executing this file with system Python.
Set DYNANAV_SCAN_TEST_CANCEL=1 to also exercise stale-request cancellation.
Set DYNANAV_SCAN_TEST_EMERGENCY=1 to inject a synthetic obstacle after three
motion revisions and verify an actual native emergency hold publication.
"""
import json, os, subprocess, tempfile, time
from pathlib import Path
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock

def main():
    root=Path(__file__).resolve().parents[3]
    prefix=os.environ.get('DYNANAV_SCAN_TEST_PREFIX')
    if not prefix or not os.environ.get('ROS_DOMAIN_ID'):
     raise RuntimeError('Set DYNANAV_SCAN_TEST_PREFIX and an isolated ROS_DOMAIN_ID; never run on a live domain')
    run=Path(tempfile.mkdtemp(prefix='dynanav-scan-smoke-'))
    mail=run/'interactive'; mail.mkdir()
    req=dict(version=1,run_id=run.name,seq=1,goal_xyz_m=[1.5,0,.4],frame_id='smoke_map',child_frame_id='imu',pose_source='livo_post_vio')
    (mail/'planner_request.json').write_text(json.dumps(req))
    (mail/'observation.json').write_text(json.dumps(dict(run_id=run.name,waypoints=dict(active_seq=1))))
    log=(run/'worker.log').open('w')
    proc=subprocess.Popen(['/usr/bin/python3',str(root/'scripts/run_dynanav_pct_scan.py'),'--run-dir',str(run),'--scan-prefix',prefix,'--body-support-offset','.4'],stdout=log,stderr=subprocess.STDOUT)
    rclpy.init(); node=rclpy.create_node('synthetic_scan_test_source')
    clockpub=node.create_publisher(Clock,'/clock',10)
    liopub=node.create_publisher(Odometry,'/dynanav/rtx_livo/aft_mapped_to_init',10)
    viopub=node.create_publisher(Odometry,'/dynanav/rtx_livo/post_vio_odometry',10)
    cloudpub=node.create_publisher(PointCloud2,'/dynanav/rtx_livo/cloud_registered_full',10)
    x,y=np.meshgrid(np.arange(-3,3.01,.04),np.arange(-3,3.01,.04))
    points=np.c_[x.ravel(),y.ravel(),np.zeros(x.size),np.linspace(0,50,x.size)].astype('float32')
    fields=[PointField(name=name,offset=i*4,datatype=7,count=1) for i,name in enumerate(['x','y','z','curvature'])]
    started=time.monotonic(); result={}; revisions=set(); child_pid=None
    emergency=bool(os.environ.get("DYNANAV_SCAN_TEST_EMERGENCY")); blocked=False; saw_hold=False
    try:
     for i in range(250):
      stamp=100_000_000_000+i*100_000_000
      clock=Clock(); clock.clock.sec=stamp//10**9;clock.clock.nanosec=stamp%10**9;clockpub.publish(clock)
      odom=Odometry();odom.header.frame_id='smoke_map';odom.child_frame_id='imu';odom.header.stamp=clock.clock;odom.pose.pose.position.z=.4;odom.pose.pose.orientation.w=1.
      liopub.publish(odom);viopub.publish(odom)
      cloudpub.publish(create_cloud(odom.header,fields,points))
      rclpy.spin_once(node,timeout_sec=.01)
      time.sleep(.09)
      if proc.poll() is not None: raise RuntimeError('worker exited '+str(proc.returncode))
      try: result=json.loads((mail/'planner_path.json').read_text())
      except (OSError,ValueError): continue
      if result.get('status')=='tracking':
       revisions.add(result['revision']); child_pid=result['provenance']['scan_process_id']
       if len(revisions)>=3:
        if not emergency: break
        if not blocked:
         wx,wy,wz=np.meshgrid(np.arange(.05,.81,.04),np.arange(-.35,.36,.04),np.arange(.2,.61,.04))
         wall=np.c_[wx.ravel(),wy.ravel(),wz.ravel(),np.linspace(0,50,wx.size)].astype('float32')
         points=np.vstack([points,wall]);blocked=True
      if blocked and result.get('status')=='hold':
       saw_hold=True;break
     print('RUN',run,'STATUS',result.get('status'),'REASON',result.get('reason'),'REVISIONS',sorted(revisions),'CHILD',child_pid)
     assert len(revisions)>=3, result
     if emergency:
      assert saw_hold, ('no native emergency hold from synthetic obstacle',result)
      assert result['revision'] > max(revisions)
      print('NATIVE_OBSTACLE_HOLD_OK',result['revision'])
     # Optional cancellation check; default exercises SIGTERM with an active child.
     if os.environ.get('DYNANAV_SCAN_TEST_CANCEL'):
      # Cancel is authoritative despite a leftover request.
      (mail/'observation.json').write_text(json.dumps(dict(run_id=run.name,waypoints=dict(active_seq=None))))
      time.sleep(.4)
      before=(mail/'planner_path.json').read_bytes();time.sleep(.3)
      assert before==(mail/'planner_path.json').read_bytes()
    finally:
     proc.terminate(); proc.wait(timeout=10)
     node.destroy_node();rclpy.shutdown();log.close()
     if child_pid:
      assert not Path('/proc/%s'%child_pid).exists(), 'orphan child'
     print('EXIT',proc.returncode,'CLEANUP_OK',not child_pid or not Path('/proc/%s'%child_pid).exists())
     print((run/'worker.log').read_text()[-1500:])

if __name__ == "__main__":
    main()
