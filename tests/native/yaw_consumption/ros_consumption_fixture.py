"""Real ROS transport with deliberate source-publication delay; no simulator."""
import json
import os
from pathlib import Path
import subprocess
import time
import uuid

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import String
import yaml

from embodied_navigation.simulation.dynanav.pct_scan_waypoints import PctScanWaypointTracker


class RosConsumptionRig:
    def __init__(self, root, prefix, repository):
        self.root=Path(root); self.mail=self.root/'interactive'; self.mail.mkdir()
        self.run_id=self.root.name; self.nonce=uuid.uuid4().hex
        self.namespace='/independent_yaw_'+self.nonce[:10]
        self.sent={}; self.replies=[]; self.latest=None; self.step_index=0
        self.pose_stamp=100.; self.reply_log=(self.root/'responses.jsonl').open('w')
        params=yaml.safe_load((Path(repository)/'third_party/scan/plan_manage/config/planner.yaml').read_text())['scan_planner_node']['ros__parameters']
        params.update({'use_sim_time':False,'fsm.navi_mode':3,
            'fsm.external_reference_protocol':True,'fsm.reference_run_id':self.run_id,
            'fsm.reference_epoch':1,'fsm.child_odometry_consumption_ack':True,
            'fsm.child_odometry_ack_nonce':self.nonce,'grid_map.frame_id':'acceptance_map',
            'grid_map.sliding_map_frame_id':'acceptance_grid','grid_map.body_height':.4,
            'grid_map.resolution':.08,'grid_map.cloud_is_world':True,
            'grid_map.need_extrinsic':False,'grid_map.obstacles_inflation_z_up':.12,
            'grid_map.obstacles_inflation_z_down':.12,'manager.max_acc':2.,
            'optimization.max_acc':2.,'optimization.acc_tolerance':2.,
            'diagnostics.voxel_trace_path':''})
        config=self.root/'native.yaml';config.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
        self.log=(self.root/'native.log').open('w')
        self.process=subprocess.Popen([str(Path(prefix)/'lib/scan_planner/scan_planner_node'),
            '--ros-args','--params-file',str(config),'-r','__ns:='+self.namespace],stdout=self.log,stderr=subprocess.STDOUT)
        self.node=rclpy.create_node('independent_delayed_inputs_'+self.nonce[:8])
        self.query_pub=self.node.create_publisher(String,self.namespace+'/rotation_query',10)
        self.body_pub=self.node.create_publisher(Odometry,self.namespace+'/body_pose',10)
        self.sensor_pub=self.node.create_publisher(Odometry,self.namespace+'/sensor_pose',10)
        self.cloud_pub=self.node.create_publisher(PointCloud2,self.namespace+'/cloud',10)
        self.sub=self.node.create_subscription(String,self.namespace+'/rotation_result',self.receive,10)
        self.fields=[PointField(name=name,offset=i*4,datatype=7,count=1) for i,name in enumerate(('x','y','z'))]
        x,y=np.meshgrid(np.arange(-2.,2.01,.1),np.arange(-2.,2.01,.1))
        self.floor=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)].astype('float32')
        self.controller=PctScanWaypointTracker(dict(slam_pose_source='livo_post_vio',speed_mps=.5,
            motion_end_s=None,interactive_mailbox=str(self.mail)))
        now=time.monotonic()
        self.controller._handle_request(dict(version=1,run_id=self.run_id,seq=1,action='yaw',
            pose_source='livo_post_vio',delta_yaw_rad=.03,created_wall_s=now,ttl_s=30.),now_wall=now)
        self.controls=[]

    def receive(self,message):
        reply=json.loads(message.data)
        reply['received_wall_s']=time.monotonic()
        self.replies.append(reply);self.latest=reply
        self.reply_log.write(json.dumps(reply)+'\n');self.reply_log.flush()
        (self.mail/'planner_yaw_space.json').write_text(json.dumps(reply))

    def spin_for(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(self.node,timeout_sec=min(.005,max(0.,end-time.monotonic())))
        if self.process.poll() is not None:
            raise AssertionError('Native process exited: '+str(self.process.returncode))

    def wait_connected(self):
        end=time.monotonic()+5.
        while time.monotonic()<end:
            if all(p.get_subscription_count()>0 for p in (self.query_pub,self.body_pub,self.sensor_pub,self.cloud_pub)):
                return
            self.spin_for(.01)
        raise AssertionError('Native subscriptions did not connect')

    def query(self,source_stamp):
        sample=len(self.sent)+1; ns=round(source_stamp*1e9)
        request=dict(version=1,run_id=self.run_id,seq=1,query_id=1,sample_id=sample,
            startup_nonce=self.nonce,frame_id='acceptance_map',child_frame_id='body',
            target_yaw_rad=.03,body_stamp_ns=ns,minimum_cloud_stamp_ns=ns)
        self.sent[sample]=request
        self.query_pub.publish(String(data=json.dumps(request)))
        return request

    def publish_inputs(self,source_stamp,blocked=False):
        message=Odometry();message.header.frame_id='acceptance_map';message.child_frame_id='body'
        ns=round(source_stamp*1e9);message.header.stamp.sec,message.header.stamp.nanosec=divmod(ns,10**9)
        message.pose.pose.position.z=.4;message.pose.pose.orientation.w=1.
        self.body_pub.publish(message);self.sensor_pub.publish(message)
        points=self.floor if not blocked else np.vstack([self.floor,np.array([[.18,0.,.4]],dtype='float32')])
        self.cloud_pub.publish(create_cloud(message.header,self.fields,points))
        self.pose_stamp=source_stamp

    def control(self,sim):
        now=time.monotonic()
        result=self.controller.tick(current_sim_s=sim,odom_stamp_s=self.pose_stamp,
            position=[0.,0.,.4],quat_wxyz=[1.,0.,0.,0.],estimated_velocity=[0.,0.,0.],
            measured_yaw_rate=0.,ready=True,now_wall=now,arrival_wall=now,
            frame_id='acceptance_map',child_frame_id='body',pose_source='livo_post_vio',
            imu_source_stamp_s=sim,imu_arrival_wall_s=now,imu_frame='go2_imu',imu_valid=True)
        self.controls.append(dict(sim=sim,command=result[0],reason=result[1]['reason'],
                                  source=self.pose_stamp))
        return result

    def close(self):
        self.process.terminate()
        try:self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=5)
        self.node.destroy_node();self.log.close();self.reply_log.close()
        (self.root/'controls.json').write_text(json.dumps(self.controls,indent=2))
