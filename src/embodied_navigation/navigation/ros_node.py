#!/usr/bin/env python3
"""Teacher-specific finite wrapper around existing measured-pose Go2 SCAN.

The shared controller consumes SLAM, actual registered clouds and checked SCAN
trajectories. This wrapper narrows scope, fixes duplicate-stamp acceptance and
requires measured stopping before obstacle recovery. It owns no joint actuator.
"""
from embodied_navigation.localization_mapping.known_scene_hold_guard import record_clear_release
import embodied_navigation.common.paths as _package_paths
import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
import numpy as np
from embodied_navigation.common.observation import api as _observer_api; _r87_diag = _observer_api('r87_diagnostic_probe')

HERE=Path(__file__).resolve().parent;ROOT=_package_paths.TEACHER_ROOT;DEMO=_package_paths.DEMO_ROOT;NAV=_package_paths.NAVIGATION_ROOT
from embodied_navigation.navigation import base_node as base
from embodied_navigation.common.runtime_context import load as load_context
from embodied_navigation.common.input_freshness import enabled as minimal_baseline_enabled, raw_imu_fresh, control_sources_stale, pose_cloud_fresh, wall_stale
from embodied_navigation.control.tracking.control_core import rotation_xyzw, body_tilt
from embodied_navigation.common.runtime_io import EvidenceWriter, latest_sensor_qos
from embodied_navigation.control.io import make_evidence_writer
from embodied_navigation.control.parking.transition_gate import TeacherHeadingGate
from embodied_navigation.control.safety.guard_audit import NativeGuardAudit, result_json
from std_msgs.msg import String
from embodied_navigation.navigation.mission.dynamic_stop_event import published_stop_event
from embodied_navigation.control.parking.navigation_stop_window import reset_transition, bind_transition_context

NATIVE_STEERING_GUARD=base.steering_obstacle_ahead
_SCAN_SHORT_INTAKE=os.environ.get('SCAN_SHORT_INTAKE','0')
if _SCAN_SHORT_INTAKE not in ('0','1'):raise ValueError('SCAN_SHORT_INTAKE must be 0 or 1')
SCAN_SHORT_INTAKE_ENABLED=_SCAN_SHORT_INTAKE=='1'


class RecordedPublisher:
    def __init__(self,publisher,run,node):self.publisher=publisher;self.run=run;self.node=node
    def publish(self,msg):
        data=json.loads(msg.data)
        data.update(mode=self.node.profile['controller_kind'],scope=self.node.profile['experiment'],navigation_ground_truth_used=False,
            policy_observations=self.node.profile['policy_observations'])
        data['known_scene_localization']=copy.deepcopy(getattr(self.node,'known_scene_inputs',None))
        data['known_scene_runtime_status']=copy.deepcopy(getattr(self.node,'known_scene_status',None))
        data['known_scene_scan_epoch']=copy.deepcopy(getattr(self.node,'known_scene_scan_epoch',None))
        bridge_receipt=getattr(self.node,'bridge_updated',None)
        data['bridge_callback_received_monotonic_wall']=(bridge_receipt if type(bridge_receipt) in (int,float) and math.isfinite(bridge_receipt) else None)
        data['dynamic_stop_event']=published_stop_event(self.node,data)
        data['last_obstacle_clear_release']=getattr(self.node,'last_clear_release',None)
        data['controller_kind']=self.node.profile['controller_kind']
        data['outer_velocity_controller']='Actual SLAM header-event geometric path/yaw PD and COM speed PI; causal IMU and actual Teacher input ACK; native SCAN collision guard'
        data['cascade_parking']=copy.deepcopy(getattr(self.node,'cascade_last_row',None))
        data['ordinary_terminal_failure_latched']=copy.deepcopy(getattr(self.node,'ordinary_terminal_failure_latched',None))
        data['path_admission']=dict(hold=getattr(self.node,'path_admission_hold',False),
            replan_pending=getattr(self.node,'path_admission_replan_pending',False),
            decision_sequence=getattr(self.node,'path_admission_sequence',0),
            last_decision=copy.deepcopy(getattr(self.node,'path_admission_last_decision',None)),
            active_reference_stamp=getattr(self.node,'active_reference_stamp',None),
            pending_reference_stamp=self.node.trajectory_association.reference_stamp,
            occupancy_or_terrain_support_certified=False)
        data['teacher_transition']=self.node.heading_gate.evidence(self.node.get_clock().now().nanoseconds/1e9,time.monotonic())
        if self.node.profile.get('arrival_stop_policy')=='after_measured_dwell':
            data['motion_limits']['pure_turn_rate_rad_s']=self.node.heading_gate.max_turn_rate
            data['arrival_stop_policy']='after_measured_dwell'
            data['arrival_stop_policy_source']='Actual raw SLAM original control-region ArrivalWindow; no tracking-pose arrival'
        self.publisher.publish(String(data=json.dumps(data,ensure_ascii=False,allow_nan=False)))
        self.node.evidence.append(self.run/'navigation_status.jsonl',{**data,'monotonic_wall':time.monotonic(),
            'ros_sim_time':self.node.get_clock().now().nanoseconds/1e9})
        self.node.evidence.atomic(self.run/'navigation_status.json',data)


class TeacherNavigation(base.Navigation):
    def __init__(self,run):
        self._bridge_safety_handoff=None
        self.run=run;receipt=load_context(run/'runtime_context.json');self.profile=receipt['profile']
        super().__init__()
        self.evidence=make_evidence_writer(self.run,self.profile);self.gyro_body=None;self.gyro_stamp=None;self.gyro_wall=None
        self.heading_gate=TeacherHeadingGate(self.profile)
        self.handoff_scope_sha256=self.run.name  # Live instance identity; never settings/source bytes.
        self.status_pub=RecordedPublisher(self.status_pub,run,self)
        self.bridge_updated=-math.inf
        self.guard_cloud=None;self.guard_cloud_receipt=None;self.pose_quaternion=None
        self.guard_sequence=0;self.pending_guard_row=None;self.trajectory_archive_reference=None
        self.last_actual_guard_clock_ns=None;self.guard_continuity_resets=0
        self.native_guard_audit=NativeGuardAudit(NATIVE_STEERING_GUARD)
        # This module instance belongs only to the independent Teacher process.
        # The shared camera-mode files and native geometry remain unchanged.
        # Native guard is called through an explicit subclass method; no globals are mutated.
    def destroy_node(self):
        try:
            handoff=getattr(self,'_bridge_safety_handoff',None)
            if handoff is not None:
                handoff.close(self,service_negative=base.rclpy.ok())
                (self.run/'bridge_safety_handoff.json').write_text(json.dumps(handoff.snapshot(),allow_nan=False)+'\n')
        finally:
            result=super().destroy_node()
        return result
    def create_subscription(self,msg_type,topic,callback,qos_profile,*args,**kwargs):
        if SCAN_SHORT_INTAKE_ENABLED and topic in ('/demo/navigation/reference_attempt',
                '/demo/navigation/bspline','/demo/navigation/trajectory_metadata','/demo/navigation/route_definition'):
            if os.environ.get('R88_ASYNC_PHYSICAL','0')!='1' or os.environ.get('R116_RECEIVE_PRESSURE','0')!='0':
                raise ValueError('SCAN short intake requires R88 and R116 disabled')
            options=self._r86_subscription_options()
            group=options.get('callback_group')
            if group is None:raise ValueError('SCAN short intake requires the original R86 group')
            kwargs['callback_group']=group
        if topic in ('/demo/slam/body_odom','/cloud_registered_full','/livox/imu'):
            qos_profile=latest_sensor_qos()
        if topic=='/demo/control/safety':
            qos_profile=1
            from embodied_navigation.control.safety.bridge_safety_handoff import selected, Handoff
            if selected():
                group=getattr(self,'_r86_intake_group',None)
                if group is None:raise ValueError('Bridge ingress requires the existing R86 intake group')
                self._bridge_safety_handoff=Handoff(self)
                kwargs['callback_group']=group
        subscription=super().create_subscription(msg_type,topic,callback,qos_profile,*args,**kwargs)
        if topic=='/demo/control/safety' and self._bridge_safety_handoff is not None:
            self._bridge_safety_handoff.bind_subscription(subscription)
        return subscription
    def create_causal_imu_subscription(self,callback):
        # Separate same-source history consumer. Original latest tilt stays at
        # depth one and retains sole ownership of physical safety state.
        from embodied_navigation.control.safety.imu_causal_buffer import enabled, FIFO_DEPTH
        if not enabled(self.profile):raise RuntimeError('causal_imu_buffer_not_enabled')
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        self.causal_imu_callback_group=MutuallyExclusiveCallbackGroup()
        qos=base.QoSProfile(depth=FIFO_DEPTH,reliability=base.ReliabilityPolicy.BEST_EFFORT)
        return super().create_subscription(base.Imu,'/livox/imu',callback,qos,
            callback_group=self.causal_imu_callback_group)
    def declare_parameter(self,name,value=None,*args,**kwargs):
        if name=='scenario_path':value=str(self.run/'navigation_scenario.json')
        if name=='max_speed':value=self.profile['max_speed_mps']
        if name in ('pose_timeout','cloud_timeout'):value=self.profile['pose_cloud_timeout_s']
        return super().declare_parameter(name,value,*args,**kwargs)
    def on_bridge_safety(self,msg):
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):return
        if (data.get('mode')!=self.profile['controller_kind'] or data.get('source')!='scan_slam'
            or data.get('state')not in ('hold','ready','failed')):return
        try:
            if minimal_baseline_enabled(self.profile):
                now=time.monotonic();receipt=float(data['monotonic_wall'])
                if wall_stale(self,'bridge_status_message',now,receipt,.3,
                        not -.05<=now-receipt<.3,future_tolerance=.05):return
            else:
                if not -.05<=time.monotonic()-float(data['monotonic_wall'])<.3:return
        except (ValueError,TypeError,KeyError):return
        receipt=time.monotonic()
        handoff=getattr(self,'_bridge_safety_handoff',None)
        if handoff is not None:
            handoff.receive(data,receipt);return
        self._accept_bridge_safety(data,receipt)
    def _accept_bridge_safety(self,data,receipt,*,force_protection=False,service_wall=None):
        self.bridge_updated=receipt;self.bridge_safety=data
        if (force_protection or self.state=='running')and self.apply_tilt_guard(receipt if service_wall is None else service_wall,self.get_clock().now().nanoseconds/1e9):
            self.reset_region_arrival('protected')
            # Do not feed bridge hold -> repeated zero -> bridge status back
            # into another zero. A moving command still stops immediately.
            if force_protection or any(self.command):self.publish_command()
            if self.state=='failed':self.publish_status()
    def on_imu(self,msg):
        now=time.monotonic();ros_now=self.get_clock().now().nanoseconds/1e9;q=msg.orientation
        intake=getattr(getattr(self,'imu_source_buffer',None),'safety_intake',None)
        original_raw=self.raw_imu if intake is None else intake.original_raw_imu
        accepted=original_raw.update([q.x,q.y,q.z,q.w],self.stamp(msg)/1e9,now,ros_now,
                                     msg.orientation_covariance[0]>=0)
        if accepted:
            if intake is not None:intake.note_original_accepted(round(ros_now*1e9))
            self.counts['imu']+=1;v=msg.angular_velocity
            gyro=self.imu_body_rotation@np.array([v.x,v.y,v.z])
            if np.isfinite(gyro).all():
                self.gyro_body=gyro.tolist();self.gyro_stamp=self.stamp(msg)/1e9;self.gyro_wall=now
        if self.state=='running'and self.apply_tilt_guard(now,ros_now):
            self.reset_region_arrival('protected')
            # Protection still stops a moving robot in this very callback.
            # Already-zero held commands use the unchanged 20Hz timer heartbeat.
            if any(self.command):self.publish_command()
            if self.state=='failed':self.publish_status()
    def apply_tilt_guard(self,now,ros_now):
        buffer=getattr(self,'imu_source_buffer',None)
        intake=getattr(buffer,'safety_intake',None)
        pending_peak=None if intake is None else intake.consume(self,now,ros_now)
        # A communication hold is not measured tilt and must not reset a
        # parked Teacher's heading/path every time a fresh cloud is delayed.
        # Physical IMU/SLAM tilt protections retain the original thresholds.
        slam_tilt=body_tilt(self.rotation);fresh=raw_imu_fresh(self,now,ros_now,self.raw_imu.fresh(now,ros_now))
        sources=[('SLAM',slam_tilt)]+([('raw_imu',self.raw_imu.tilt)]if fresh else [])
        # Recovery interruption/high/severe tilt is an event: a later level
        # sample from either port must not erase it.
        # Consume through the original thresholds and recovery state machine.
        if pending_peak is not None:sources.append(('raw_imu_pending_peak',pending_peak[4]))
        self.max_tilt=max(self.max_tilt,*(tilt for _,tilt in sources))
        severe=[name for name,tilt in sources if tilt>=.50]
        if self.bridge_safety.get('state')=='failed':severe.append('execution_bridge')
        if self.evidence.error:severe.append('evidence_writer')
        if severe:
            self.state='failed';self.tilt_source='+'.join(severe);self.message=self.tilt_source+'保护已锁定，停车'
            return True
        high=[name for name,tilt in sources if tilt>=.30]
        if high:
            if not self.tilt_hold:self.tilt_stops+=1
            self.tilt_hold=True;self.tilt_source='+'.join(high);self.tilt_clear_since=None
        if not fresh:
            self.message='原始IMU超时，停车';self.tilt_clear_since=None;return True
        if self.tilt_hold:
            if all(tilt<.18 for _,tilt in sources)and pose_cloud_fresh(self,'pose',now,ros_now,now-self.pose_updated<self.pose_timeout):
                if self.tilt_clear_since is None:self.tilt_clear_since=ros_now
                if ros_now-self.tilt_clear_since>=.8:
                    self.tilt_hold=False;self.tilt_source=None;reset_transition(self,'tilt_recovery')
                    self.samples=None;self.last_reference=-math.inf
            else:self.tilt_clear_since=None
            self.message='实际倾角保护，停车等待稳定';return True
        if wall_stale(self,'bridge_callback',now,self.bridge_updated,.30,now-self.bridge_updated>.30) or self.bridge_safety.get('state')!='ready':
            self.message='Teacher 执行命令门或 SLAM 传感器预热未就绪，停车'
            return True
        self.tilt_source=None;return False
    @_r87_diag.observe('teacher.odom')
    def on_odom(self,msg):
        stamp=self.stamp(msg);ns=self.get_clock().now().nanoseconds
        received=time.monotonic();q=msg.pose.pose.orientation
        v=msg.twist.twist.linear;w=msg.twist.twist.angular;p=msg.pose.pose.position
        # Preserve the original short-circuit order and its single ROS clock.
        # The observer records the branch actually taken, never a second gate.
        rejection=None
        if msg.header.frame_id!='camera_init':rejection='frame_id'
        elif msg.child_frame_id!='demo_slam_body':rejection='child_frame_id'
        elif stamp<=self.pose_stamp:rejection='source_not_advancing'
        elif not -.05e9<=ns-stamp<self.pose_timeout*1e9:rejection='header_age'
        elif not .98<=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<=1.02:rejection='quaternion_norm'
        elif not all(math.isfinite(x)for x in (p.x,p.y,p.z,v.x,v.y,v.z,w.x,w.y,w.z)):rejection='nonfinite_pose_or_twist'
        _r87_diag.mark('teacher.odom.decision',source_ns=stamp,decision_ros_ns=ns,
            prior_pose_stamp_ns=self.pose_stamp,header_age_ns=ns-stamp,
            timeout_s=self.pose_timeout,rejection=rejection)
        if rejection is not None:
            _r87_diag.anomaly('teacher_odom_rejected:'+rejection,source_ns=stamp)
            return
        with _r87_diag.span('teacher.odom.shared',source_ns=stamp):
            super().on_odom(msg)
        _r87_diag.mark('teacher.odom.shared_return',source_ns=stamp,accepted_stamp_ns=self.pose_stamp)
        if self.pose_stamp==stamp:self.pose_quaternion=[q.x,q.y,q.z,q.w]
        stop_zero=(not any(self.command) and not getattr(self.heading_gate,'controlled_braking',False)
                   and getattr(self,'_r86_positive_stop_allowed',True))
        if self.gyro_body is not None and received-self.gyro_wall<.3:
            if getattr(self.heading_gate,'handoff_active',False) and hasattr(self,'get_ack'):
                self.heading_gate.bind_zero_ack(self.get_ack(pose_stamp_ns=stamp),self.handoff_scope_sha256,
                    self.request_id,self.waypoint_index,ns,round(received*1e9))
            self.heading_gate.observe(stamp/1e9,received,[v.x,v.y,v.z],w.z,self.gyro_body,self.gyro_stamp,stop_zero,position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w],
                positive_qualification_allowed=getattr(self,'_r86_positive_stop_allowed',True))
        row={'stamp_ns':stamp,'frame_id':msg.header.frame_id,'child_frame_id':msg.child_frame_id,
            'position':[p.x,p.y,p.z],'quaternion':[q.x,q.y,q.z,q.w],
            'body_velocity':[v.x,v.y,v.z],'body_angular_velocity':[w.x,w.y,w.z],
            'received_monotonic_wall':received,'callback_ros_clock_ns':ns,'sim_age_at_callback_s':(ns-stamp)/1e9,
            'body_gyro':self.gyro_body,'gyro_stamp_s':self.gyro_stamp,'zero_velocity_command':not any(self.command),
            'heading_stop_zero_candidate':stop_zero,'controlled_braking':bool(getattr(self.heading_gate,'controlled_braking',False)),
            'measured_stop_evidence':self.heading_gate.evidence(ns/1e9,received)}
        self.evidence.append(self.run/'navigation_slam_poses.jsonl',row)
    def on_cloud(self,msg):
        stamp=self.stamp(msg);ns=self.get_clock().now().nanoseconds;received=time.monotonic()
        if getattr(self,'_r86_committing_bundle',None) is not None:
            from embodied_navigation.perception.sensors.feedback_mailbox import receipt_ns
            received=receipt_ns(self,'cloud',round(received*1e9))/1e9
        if stamp<=self.cloud_stamp:
            self.note_cloud_decision('rejected_duplicate_or_backward_header',decision_ros_clock_ns=ns,
                previous_accepted_stamp_ns=int(self.cloud_stamp));return
        if not -.05e9<=ns-stamp<self.cloud_timeout*1e9:
            self.note_cloud_decision('rejected_header_age',decision_ros_clock_ns=ns,
                actual_header_age_ns=ns-stamp,timeout_s=self.cloud_timeout);return
        super().on_cloud(msg)
        if self.cloud_stamp==stamp and self.cloud_input_context is not None:
            c=self.cloud_input_context
            row={'stamp_ns':stamp,'frame_id':msg.header.frame_id,'input_points':int(msg.width*msg.height),
                'filtered_points':c['filtered_points'],'self_filtered_points':c['self_filtered_points'],
                'nearest_slam_pose_stamp_ns':c['filtering_body_stamp_ns'],
                'received_monotonic_wall':received,'decoded_monotonic_wall':time.monotonic(),
                'callback_ros_clock_ns':ns,'sim_age_at_callback_s':(ns-stamp)/1e9,
                'filtered_xyz_float64_sha256':hashlib.sha256(np.asarray(self.cloud,dtype='<f8').tobytes()).hexdigest()}
            self.guard_cloud=self.cloud
            self.guard_cloud_receipt={**row,'filtering_body_pose':c['filtering_body_pose'].tolist(),
                'filtering_body_rotation':c['filtering_body_rotation'].tolist()}
            self.evidence.append(self.run/'navigation_cloud_history.jsonl',row)
    def note_cloud_decision(self,reason,**details):
        # A read-only hook into the actual rejection/acceptance branch. It never
        # retries a message, decodes another cloud, or changes a freshness gate.
        row=getattr(self,'pending_cloud_callback_row',None)
        if row is not None:row.update(reason=reason,decision_details=details)
    def accept_spline(self,msg,metadata):
        before=self.replans;super().accept_spline(msg,metadata)
        if self.replans==before or self.samples is None:return
        directory=self.run/'navigation_trajectories'
        array_path=directory/f'{self.replans:06d}_trajectory_{self.active_trajectory_id}.npz'
        coefficients=np.array([[p.x,p.y,p.z]for p in msg.pos_pts]);knots=np.array(msg.knots);samples=self.samples.copy()
        document={'trajectory_id':self.active_trajectory_id,'order':int(msg.order),'metadata':metadata,
            'reference_stamp':self.active_reference_stamp,'request_id':self.request_id,
            'waypoint_index':self.waypoint_index,'accepted_slam_stamp_ns':self.pose_stamp,
            'array_file':array_path.name,
            'source':'actual SCAN committed B-spline and associated metadata; sensor SLAM feedback'}
        self.trajectory_archive_reference=str(array_path.relative_to(self.run))
        def archive():
            directory.mkdir(exist_ok=True);np.savez_compressed(array_path,coefficients=coefficients,knots=knots,samples=samples)
            document['array_sha256']=hashlib.sha256(array_path.read_bytes()).hexdigest()
            array_path.with_suffix('.json').write_text(json.dumps(document,allow_nan=False,ensure_ascii=False)+'\n')
        self.evidence.enqueue(archive)
    def on_request(self,msg):
        try:
            requested=json.loads(msg.data);frozen=json.loads((self.run/'navigation_request.json').read_text())
            if requested!=frozen:raise ValueError('Only this run frozen SLAM-relative route is eligible')
        except (OSError,ValueError,TypeError)as error:
            self.last_rejected=str(error);self.publish_status();return
        super().on_request(msg)
    def measured_region_arrival(self):
        inside,arrived=super().measured_region_arrival()
        if self.profile.get('arrival_stop_policy')=='after_measured_dwell':
            # The original raw-SLAM ArrivalWindow, protection flags, integer
            # stamps, observation gap and dwell remain unchanged. Continue only
            # the checked SCAN route until that same window confirms arrival;
            # all native path-end/obstacle/tilt/stale stops still run below.
            return inside and arrived,arrived
        return inside,arrived
    def control(self,now):
        # Shared recovery already requires one clear second and a new checked
        # trajectory; additionally hold it until measured body motion is small.
        if self.evidence.error:
            self.state='failed';self.message=self.evidence.error;self.publish_command();return
        if self.profile.get('clear_guard_gap_max_sim_s')is not None and self.obstacle_hold:
            ns=self.get_clock().now().nanoseconds
            if not self.guard_continuity(ns,now):
                # Keep the existing physical protections active even when an
                # extra actual-source freshness check holds this callback.
                self.apply_tilt_guard(now,ns/1e9);self.reset_region_arrival('stale')
                self.stale_since=self.stale_since or now
                if now-self.stale_since>8.:
                    self.state='failed';self.message='实际SLAM/点云guard输入连续失联超过8秒，停车'
                self.publish_command();return
            if (self.obstacle_clear_since is not None and ns/1e9-self.obstacle_clear_since>=1.
                    and not now-self.last_obstacle_check>=.05):
                # A cached clear result cannot finish the original one-second
                # gate. Wait for the next already scheduled native evaluation;
                # do not run another geometry check or refresh input stamps.
                protected=self.apply_tilt_guard(now,ns/1e9)
                stale=control_sources_stale(self,now,ns/1e9,
                       (now-self.pose_updated>self.pose_timeout or now-self.cloud_updated>self.cloud_timeout
                        or not self.raw_imu.fresh(now,ns/1e9)))
                if protected or stale:
                    self.reset_region_arrival('protected'if protected else 'stale')
                    self.obstacle_clear_since=None
                self.publish_command();return
        if self.obstacle_hold and not self.heading_gate.stopped(self.get_clock().now().nanoseconds/1e9,now):
            self.obstacle_clear_since=None
        self.pending_guard_row=None
        try:super().control(now)
        finally:
            if self.pending_guard_row is not None:
                row=self.pending_guard_row;after_ns=self.get_clock().now().nanoseconds
                row.update(obstacle_hold_after=bool(self.obstacle_hold),obstacle_stops_after=int(self.obstacle_stops),clear_start_after_s=self.obstacle_clear_since,
                    clear_elapsed_after_s=None if self.obstacle_clear_since is None else after_ns/1e9-self.obstacle_clear_since,
                    after_control_ros_clock_ns=after_ns,command_after_control=list(self.command),
                    zero_requested_after_control=not any(self.command),command_ros_sim_time_s=self.last_command_time,
                    state_after_control=self.state,obstacle_resumes_after=self.obstacle_resumes,
                    command_source='actual sensor SLAM/SCAN controller; no mover or simulator pose navigation input',
                    evidence_queue_error=self.evidence.error)
                record_clear_release(self,row)
                self.evidence.append(self.run/'navigation_guard_history.jsonl',row)
                self.pending_guard_row=None
        if (self.state=='running' and not getattr(self,'reference_consistency_hold',False)
                and self.heading_gate.phase in ('pre_turn','settle')
                and not (getattr(self.heading_gate,'controlled_settle',False) or getattr(self.heading_gate,'controlled_braking',False))):
            self.message='Teacher 等待实际SLAM机身速度和IMU角速确认停车，保持零速度'

    def record_native_guard(self,cloud,pose,checked_target,steering_direction,route):
        ns=self.get_clock().now().nanoseconds;received=time.monotonic()
        previous_guard_ns=self.last_actual_guard_clock_ns
        if self.profile.get('clear_guard_gap_max_sim_s')is not None:
            if previous_guard_ns is not None and ns<=previous_guard_ns:
                self.state='failed';self.message='实际native guard时钟重复或倒退，停车'
                raise RuntimeError(self.message)
            if self.obstacle_hold and not self.guard_continuity(ns,received):
                raise RuntimeError('Actual native guard source ceased being fresh before evaluation')
        if (cloud is not self.guard_cloud or self.guard_cloud_receipt is None
                or self.guard_cloud_receipt['stamp_ns']!=self.cloud_stamp or self.pose_quaternion is None):
            raise RuntimeError('Actual native guard lacks an exact accepted cloud/SLAM input receipt')
        with _r87_diag.span('teacher.native_geometry',source_ns=self.pose_stamp):
            result=self.native_guard_audit(cloud,pose,checked_target,steering_direction,route)
        self.last_actual_guard_clock_ns=ns
        from embodied_navigation.control.safety.clear_guard_window import ClearGuardWindow
        if not hasattr(self,'_actual_clear_guard_window'):
            self._actual_clear_guard_window=ClearGuardWindow()
        if self.obstacle_clear_since is None:self._actual_clear_guard_window.clocks.clear()
        self._actual_clear_guard_ready=self._actual_clear_guard_window.observe(ns,bool(result[0]),True)
        self.guard_sequence+=1;c=self.guard_cloud_receipt
        self.pending_guard_row={'schema':'teacher_native_steering_guard/v1','sequence':self.guard_sequence,
            'compute_ros_clock_ns':ns,'compute_sim_time_s':ns/1e9,'compute_monotonic_wall':received,
            'request_id':self.request_id,'waypoint_index':self.waypoint_index,
            'trajectory_id':self.active_trajectory_id,'reference_stamp':self.active_reference_stamp,
            'trajectory_archive_file':self.trajectory_archive_reference,
            'control_pose_stamp_ns':int(self.pose_stamp),'control_pose':np.asarray(pose).tolist(),
            'control_quaternion':list(self.pose_quaternion),'control_rotation':self.rotation.tolist(),
            'tracking_pose':self.tracking_pose.tolist(),'control_pose_received_monotonic_wall':self.pose_updated,
            'cloud_header_stamp_ns':int(c['stamp_ns']),'cloud_frame_id':c['frame_id'],
            'cloud_received_monotonic_wall':c['received_monotonic_wall'],
            'cloud_callback_ros_clock_ns':c['callback_ros_clock_ns'],
            'cloud_filtering_body_stamp_ns':c['nearest_slam_pose_stamp_ns'],
            'cloud_filtering_body_pose':c['filtering_body_pose'],'cloud_filtering_body_rotation':c['filtering_body_rotation'],
            'filtered_xyz_float64_sha256':c['filtered_xyz_float64_sha256'],
            'filtered_points':c['filtered_points'],'self_filtered_points':c['self_filtered_points'],
            'checked_target':np.asarray(checked_target).tolist(),'steering_direction':np.asarray(steering_direction).tolist(),
            'goal':self.waypoints[self.waypoint_index].tolist(),
            'goal_corridor_result':result_json(self.native_guard_audit.calls[0][1]),
            'motion_corridor_result':result_json(self.native_guard_audit.calls[1][1]),
            'motion_corridor_target':self.native_guard_audit.calls[1][0].tolist(),
            'union_result':result_json(result),'obstacle_hold_before':bool(self.obstacle_hold),'obstacle_stops_before':int(self.obstacle_stops),
            'clear_start_before_s':self.obstacle_clear_since,'clear_elapsed_before_s':None if self.obstacle_clear_since is None else ns/1e9-self.obstacle_clear_since,
            'native_geometry_source':'unchanged control_core.steering_obstacle_ahead code object and original two obstacle_ahead evaluations'}
        reference=getattr(self,'_program_guard_route_reference',None)
        if reference is not None and route is getattr(self,'_program_guard_route',None):
            self.pending_guard_row.update(route_reference=dict(reference),
                route_float64_sha256=reference['route_float64_sha256'])
        else:
            # Legacy/unbound inputs retain the original complete replay row.
            self.pending_guard_row.update(route=np.asarray(route).tolist(),
                route_float64_sha256=hashlib.sha256(np.asarray(route,dtype='<f8').tobytes()).hexdigest())
        if self.profile.get('clear_guard_gap_max_sim_s')is not None:
            self.pending_guard_row.update(previous_actual_guard_clock_ns=previous_guard_ns,
                clear_guard_gap_max_sim_s=self.profile['clear_guard_gap_max_sim_s'],
                clear_guard_continuity_resets=self.guard_continuity_resets,
                clear_release_requires_this_callback_actual_guard=True)
        return result

    def guard_continuity(self,ns,wall):
        """V44 only: original clear timer cannot span missing actual checks."""
        maximum_ns=round(self.profile['clear_guard_gap_max_sim_s']*1e9)
        previous=self.last_actual_guard_clock_ns
        if previous is not None and ns<previous:
            self.state='failed';self.message='实际native guard时钟倒退，停车';self.obstacle_clear_since=None
            return False
        reset=previous is None or ns-previous>maximum_ns
        fresh_ns=round(self.profile['pose_cloud_timeout_s']*1e9)
        source_fresh=all(stamp>=0 and -50_000_000<=ns-stamp<fresh_ns for stamp in (self.pose_stamp,self.cloud_stamp))
        if reset or not source_fresh:
            before=self.obstacle_clear_since;self.obstacle_clear_since=None
            if before is not None:
                self.guard_continuity_resets+=1
                self.evidence.append(self.run/'navigation_guard_continuity_history.jsonl',{
                    'schema':'teacher_actual_guard_continuity/v1','compute_ros_clock_ns':ns,
                    'compute_monotonic_wall':wall,'previous_actual_guard_clock_ns':previous,
                    'actual_guard_gap_ns':None if previous is None else ns-previous,
                    'pose_stamp_ns':self.pose_stamp,'cloud_stamp_ns':self.cloud_stamp,
                    'clear_start_before_s':before,'clear_start_after_s':None,
                    'reason':'actual_source_stale'if not source_fresh else 'actual_guard_gap',
                    'extra_geometry_computed':False})
        if not source_fresh:self.message='实际SLAM/点云header超时，停车等待fresh guard输入'
        return source_fresh


def main():
    import rclpy
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    args,ros=p.parse_known_args();run=args.run.resolve();os.environ['DEMO_RUN_DIR']=str(run)
    rclpy.init(args=ros);node=TeacherNavigation(run)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.publish_command()
        node.event_archive.close()
        try:node.evidence.close()
        finally:
            if node.profile.get('dynamic_fixture'):
                (run/'navigation_guard_writer_receipt.json').write_text(json.dumps({
                    'schema':'teacher_native_steering_guard_writer/v1','expected_records':node.guard_sequence,
                    'status':'failed'if node.evidence.error else 'drained',
                    'queue_error':node.evidence.error,'silent_record_loss_permitted':False,
                    'clear_guard_continuity_resets':node.guard_continuity_resets,
                    'source':'actual dynamic controller shutdown after bounded evidence queue drain',
                    'navigation_verified':False},allow_nan=False,ensure_ascii=False)+'\n')
            node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
