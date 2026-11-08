"""The existing shuttle task state machine, separated from ROS and simulation.

This is the current supported task: relative-SLAM waypoints, settle, turn and
stop/failure. There is no invented global replanner or autonomous recovery.
"""
from dataclasses import dataclass, field
import numpy as np
from ..control.relative_pid import follow_segment, wrap_angle
from ..control.collision import braking_sweep, depth_collision

@dataclass(frozen=True)
class Observation:
    time_s: float
    received_wall_s: float
    body: np.ndarray
    rotation: np.ndarray
    velocity: np.ndarray
    yaw_unwrapped: float
    obstacles: np.ndarray
    sensor_points_finite: bool = True

@dataclass(frozen=True)
class TaskConfig:
    motion_start_s: float = 7.0
    motion_end_s: float = 270.0
    waypoint_tolerance_m: float = 0.22
    waypoint_settle_s: float = 0.5
    turn_tolerance_rad: float = 0.10
    turn_settle_s: float = 0.4

class NavigationTask:
    def __init__(self, route, certify_points, config=TaskConfig()):
        self.route = np.asarray(route, dtype=float)
        if self.route.ndim != 2 or self.route.shape[1] != 2 or len(self.route) < 2:
            raise ValueError("Route must contain at least two XY waypoints")
        if not all(certify_points(np.linspace(a,b,301)) for a,b in zip(self.route[:-1],self.route[1:])):
            raise ValueError("Route lacks measured support")
        self.certify_points = certify_points
        self.config = config
        self.stage = 0
        self.phase = "initializing"
        self.arrived = False
        self.failure = None
        self.settle_since = self.turn_since = self.turn_target = None
        self.stage_start_time = self.stale_since = self.safety_hold_since = None
        self.observation = None
        self.poses = []
        self.stages = []
        self.events = []
        self.relative_distance = 0.0

    def update(self, observation):
        if self.observation is not None:
            dt = observation.time_s - self.observation.time_s
            step = np.linalg.norm(observation.body - self.observation.body)
            self.relative_distance += float(np.linalg.norm((observation.body-self.observation.body)[:2]))
            if dt <= 0 or step > 0.02+0.65*dt:
                self.failure = "severe_relative_odometry_jump"
        if not observation.sensor_points_finite or not np.isfinite(observation.body).all() or not np.isfinite(observation.obstacles).all():
            self.failure = "invalid_sensor_or_odometry"
        if self.observation is None:
            self.phase = "go"
            self.stage_start_time = observation.time_s
        self.observation = observation
        self.poses.append(observation)
        # The original settling window is 0.6 s. Older samples are not controller state.
        self.poses = [p for p in self.poses if p.time_s >= observation.time_s-0.6]

    def step(self, time_s, wall_s):
        cmd = np.zeros(3)
        reason = "initializing"
        diag = {}
        latest = self.observation
        if latest is not None:
            body, rotation, velocity, yaw = latest.body, latest.rotation, latest.velocity, latest.yaw_unwrapped
            goal = self.route[min(self.stage+1,len(self.route)-1)]
            distance = float(np.linalg.norm(body[:2]-goal))
            window = np.array([p.body[:2] for p in self.poses])
            span = float(np.linalg.norm(np.ptp(window,axis=0))) if len(window)>=4 else 1.0
            if self.failure:
                reason = self.failure
            elif self.arrived:
                reason = "all_stages_completed"
            elif not self.config.motion_start_s < time_s < self.config.motion_end_s:
                reason = "outside_motion_window"
                if time_s >= self.config.motion_end_s:
                    self.failure = "motion_time_budget_exhausted"
            elif time_s-latest.time_s > 0.25 or wall_s-latest.received_wall_s > 0.5:
                reason = "freshness_hold"
                if self.stale_since is None:self.stale_since = time_s
                if time_s-self.stale_since > 1.0:self.failure = "persistent_SLAM_or_depth_staleness"
            else:
                self.stale_since = None
                if self.phase == "go" and distance <= self.config.waypoint_tolerance_m:
                    self.phase = "settling";self.settle_since = None
                    self.events.append({"time":time_s,"event":"waypoint_stop_requested","stage":self.stage,"distance_m":distance})
                if self.phase == "settling":
                    reason = "waypoint_settling"
                    if distance > 0.40:self.failure = "waypoint_stop_overshoot"
                    if span < 0.04 and distance <= self.config.waypoint_tolerance_m:
                        if self.settle_since is None:self.settle_since = time_s
                        if time_s-self.settle_since >= self.config.waypoint_settle_s:
                            self.stages.append({"stage":self.stage,"from":self.route[self.stage].tolist(),"to":goal.tolist(),"start_time":self.stage_start_time,"completed_time":time_s,"estimated_error_m":distance,"estimated_body":body.tolist(),"relative_distance_at_completion_m":self.relative_distance,"completed":True})
                            self.events.append({"time":time_s,"event":"waypoint_completed","stage":self.stage})
                            self.stage += 1;self.settle_since = None
                            if self.stage >= len(self.route)-1:
                                self.arrived=True;self.phase="done";reason="all_stages_completed"
                            else:
                                heading=float(np.arctan2(*(self.route[self.stage+1]-body[:2])[::-1]));delta=wrap_angle(heading-yaw)
                                if abs(delta)>3.0:delta=abs(delta)
                                self.turn_target=yaw+delta;self.turn_since=None;self.phase="turning";self.stage_start_time=time_s
                                self.events.append({"time":time_s,"event":"turn_started","stage":self.stage,"yaw_unwrapped_start":yaw,"target_yaw_unwrapped":self.turn_target})
                    else:self.settle_since=None
                elif self.phase == "turning":
                    error=float(self.turn_target-yaw);cmd[2]=float(np.clip(1.2*error,-0.15,0.15));reason="in_place_waypoint_turn"
                    if abs(error)<self.config.turn_tolerance_rad and span<0.05:
                        if self.turn_since is None:self.turn_since=time_s
                        if time_s-self.turn_since>=self.config.turn_settle_s:
                            self.phase="go";self.turn_since=None
                            self.events.append({"time":time_s,"event":"turn_completed","stage":self.stage,"yaw_error_rad":error})
                    else:self.turn_since=None
                elif self.phase == "go":
                    cmd,diag=follow_segment(body[:2],rotation[:2,:2],velocity[:2],yaw,self.route[self.stage],self.route[self.stage+1])
                    reason="relative_SLAM_multistage_PID"
                sweep,braking=braking_sweep(body[:2],rotation[:2,:2],cmd,velocity[:2])
                if not self.certify_points(sweep):cmd[:]=0;reason="nominal_map_collision_or_support_hold"
                danger,closest=depth_collision(sweep,latest.obstacles,rotation[:2,0],self.phase=="turning")
                if danger:cmd[:]=0;reason="live_depth_collision_hold"
                if not self.certify_points(body[None,:2]):
                    cmd[:]=0;self.failure="relative_pose_outside_measured_support";reason=self.failure
                diag.update(distance_to_waypoint_m=distance,pose_window_span_m=span,live_min_clearance_m=closest,braking_distance_m=braking)
        if reason in ("live_depth_collision_hold","nominal_map_collision_or_support_hold"):
            if self.safety_hold_since is None:self.safety_hold_since=time_s
            if time_s-self.safety_hold_since>12.0:self.failure="persistent_safety_hold:"+reason
        else:self.safety_hold_since=None
        if self.arrived or self.failure:cmd[:]=0
        return cmd,{"time":time_s,"request":cmd.tolist(),"reason":reason,"stage":self.stage,"phase":self.phase,**diag}
