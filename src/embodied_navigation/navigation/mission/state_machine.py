"""Mission transitions independent of ROS transport and simulation timing."""
from dataclasses import dataclass, field
import math
from embodied_navigation.navigation.mission.route_regions import transformed_route_goals, validate_route_regions
from embodied_navigation.navigation.mission.goal_regions import contains, contains_control, definitions_sha256, parse_goal


ACTIVE_STAGES = {'waiting_sensors', 'exploring', 'returning', 'saving_map', 'navigating'}


@dataclass
class ObstacleEvidence:
    hold_events: int = 0
    held: bool = False
    resumed: bool = False

    def observe(self, held, speed, obstacle):
        active_obstacle = (obstacle.get('active') and
                           obstacle.get('visible_phase') in {'entering', 'blocking', 'leaving'} and
                           obstacle.get('gazebo_updates', 0) > 0)
        if held and not self.held and active_obstacle:
            self.hold_events += 1
        # Replanning may take several zero-command updates after clearance.
        if self.hold_events and not held and speed > .025:
            self.resumed = True
        self.held = held

    @property
    def verified(self):
        return self.hold_events > 0 and self.resumed


@dataclass
class Mission:
    run_id: str = ''
    stage: str = 'idle'
    message: str = '等待启动完整演示。'
    origin: list | None = None
    current_request: str | None = None
    goal: list | None = None
    events: list = field(default_factory=list)
    completed_stages: list = field(default_factory=list)
    stage_started: float = 0.
    ready_since: float | None = None
    sensor_missing_since: float | None = None
    route_counter: int = 0
    scenario: dict = field(default_factory=dict)
    heading_alignment: dict | None = None
    goal_region: dict | None = None
    current_goals: list = field(default_factory=list)
    current_goals_sha256: str | None = None

    def enter(self, stage, now, message):
        self.stage, self.stage_started, self.message = stage, now, message
        self.events.append({'stage': stage, 'elapsed_s': round(now, 3), 'message': message})

    def start(self, run_id, scenario, now=0.):
        if self.stage in ACTIVE_STAGES:
            raise ValueError('已有任务正在运行。')
        validate_route_regions(scenario)
        self.run_id, self.scenario = run_id, scenario
        self.origin, self.current_request, self.goal = None, None, None
        self.goal_region, self.current_goals_sha256, self.current_goals = None, None, []
        self.heading_alignment = None
        self.events, self.completed_stages = [], []
        self.ready_since, self.sensor_missing_since = None, None
        self.route_counter = 0
        self.enter('waiting_sensors', now, '正在启动仿真，等待稳定的 SLAM、雷达、IMU 和 RGB 数据。')

    def stop(self, now):
        if self.stage in ACTIVE_STAGES:
            self.enter('stopped', now, '用户停止任务。')
        self.current_request = None

    def fail(self, now, reason):
        self.enter('failed', now, reason)
        self.current_request = None
        return [{'kind': 'stop_navigation'}]

    def route(self, name):
        points = self.scenario[name]
        if self.scenario.get('waypoint_reference') == 'relative_world_axes':
            angle = self.heading_alignment['yaw_camera_init_from_world']
            c, s = math.cos(angle), math.sin(angle)
            return [[self.origin[0]+c*p[0]-s*p[1], self.origin[1]+s*p[0]+c*p[1], self.origin[2]+p[2]] for p in points]
        if self.scenario.get('waypoint_reference', 'relative_initial_body') == 'relative_initial_body':
            return [[p[i]+self.origin[i] for i in range(3)] for p in points]
        return [list(p) for p in points]

    def request_route(self, name, now):
        points = self.route(name)
        if not points:
            return self.fail(now, f'场景没有配置 {name} 路线。')
        self.route_counter += 1
        self.current_request = f'{self.run_id}:{name}:{self.route_counter}'
        self.goal = points[-1]
        goals = transformed_route_goals(self.scenario, name, self.origin, self.heading_alignment)
        self.current_goals = [g.definition() for g in goals]
        self.current_goals_sha256 = definitions_sha256(goals) if goals else None
        self.goal_region = self.current_goals[-1] if goals else None
        if goals:
            return [{'kind': 'route', 'schema_version': 2, 'request_id': self.current_request,
                     'frame_id': self.scenario.get('frame_id', 'camera_init'),
                     'goals': self.current_goals}]
        return [{'kind': 'route', 'request_id': self.current_request,
                 'frame_id': self.scenario.get('frame_id', 'camera_init'), 'waypoints': points}]

    def region_completion_valid(self, status, pose):
        if status.get('goals_definition_sha256') != self.current_goals_sha256:
            return False
        if status.get('goals_definitions') != self.current_goals:
            return False
        if status.get('waypoint_index') != len(self.current_goals) or status.get('total') != len(self.current_goals):
            return False
        receipts = status.get('region_arrivals')
        if not isinstance(receipts, list) or len(receipts) != len(self.current_goals):
            return False
        previous = -1
        for raw, receipt in zip(self.current_goals, receipts):
            goal = parse_goal(raw)
            if not isinstance(receipt, dict) or receipt.get('goal_id') != goal.goal_id:
                return False
            if receipt.get('request_id') != self.current_request or receipt.get('goals_definition_sha256') != self.current_goals_sha256:
                return False
            stamp, start, dwell = (receipt.get(k) for k in ('stamp_ns', 'start_stamp_ns', 'dwell_ns'))
            if any(type(v) is not int for v in (stamp, start, dwell)):
                return False
            if start <= previous or stamp < start or dwell != stamp-start or dwell < round(goal.dwell_sim_s*1e9):
                return False
            if receipt.get('region_inside') is not True or receipt.get('protected') is not False or receipt.get('arrival_definition') != goal.definition()['arrival']:
                return False
            if receipt.get('reason') != 'arrived' or receipt.get('max_observation_gap_ns') != 200_000_000:
                return False
            try:
                if not contains_control(goal, receipt.get('raw_position')):
                    return False
                if 'control_band' in goal.definition()['arrival']:
                    if (receipt.get('control_region_inside') is not True or
                            receipt.get('control_arrival_definition') != goal.control_arrival_definition()):
                        return False
            except (TypeError, ValueError):
                return False
            previous = stamp
        try:
            return pose is not None and contains_control(parse_goal(self.goal_region), pose)
        except (TypeError, ValueError):
            return False

    def tick(self, now, sensors_ok, pose, map_points=0):
        if self.stage not in ACTIVE_STAGES:
            return []
        if self.stage == 'waiting_sensors':
            if now-self.stage_started > self.scenario.get('startup_timeout_s', 180.):
                return self.fail(now, '初始化超时：没有稳定的传感器和 SLAM 数据。')
            heading_ready = self.scenario.get('waypoint_reference') != 'relative_world_axes' or self.heading_alignment is not None
            if sensors_ok and heading_ready and pose is not None and map_points >= self.scenario.get('minimum_start_points', 100):
                if self.ready_since is None:
                    self.ready_since = now
                    self.origin = list(pose)
                elif math.dist(pose, self.origin) > .08:
                    self.ready_since, self.origin = now, list(pose)
                elif now-self.ready_since >= self.scenario.get('initial_stable_s', 3.):
                    self.enter('exploring', now, '沿规定路线探索，累计本次实际观测的彩色点云。')
                    return self.request_route('exploration', now)
            else:
                self.ready_since = None
            return []
        if not sensors_ok:
            self.sensor_missing_since = self.sensor_missing_since or now
            if now-self.sensor_missing_since > self.scenario.get('sensor_failure_grace_s', 3.):
                return self.fail(now, '传感器或 SLAM 断流，任务停止。')
        else:
            self.sensor_missing_since = None
        timeout = self.scenario.get('save_timeout_s', 30.) if self.stage == 'saving_map' else self.scenario.get('stage_timeout_s', 1500.)
        if now-self.stage_started > timeout:
            return self.fail(now, f'{self.stage} 阶段超时。')
        return []

    def navigation_result(self, status, pose, now):
        if self.stage not in {'exploring', 'returning', 'navigating'}:
            return []
        if status.get('request_id') != self.current_request:
            return []
        result = status.get('state')
        if result == 'failed':
            return self.fail(now, '导航失败：'+str(status.get('message', '未提供原因')))
        if result != 'succeeded':
            return []
        if self.goal_region is not None and not self.region_completion_valid(status, pose):
            return self.fail(now, '导航区域完成证据不匹配，或实际 SLAM 位姿未进入终点区域。')
        if self.goal_region is None and (pose is None or math.dist(pose, self.goal) > self.scenario.get('goal_tolerance_m', .30)):
            return self.fail(now, '导航报告完成，但实际 SLAM 位姿未达到本阶段终点。')
        completed = self.stage
        if completed == 'navigating' and self.scenario.get('require_dynamic_obstacle', True) and not status.get('dynamic_obstacle_verified', False):
            return self.fail(now, '已到导航终点，但缺少本次动态障碍响应与恢复证据。')
        self.completed_stages.append(completed)
        if completed == 'exploring':
            self.enter('returning', now, '探索航点已完成，沿规定返航路线返回初始位置。')
            return self.request_route('return_origin', now)
        if completed == 'returning':
            if self.goal_region is None and math.dist(pose, self.origin) > self.scenario.get('goal_tolerance_m', .30):
                return self.fail(now, '返航路线结束，但未回到本次初始化原点。')
            self.enter('saving_map', now, '已返回原点，正在保存本次探索的彩色 SLAM 地图。')
            return [{'kind': 'save_map'}]
        self.enter('completed', now, '已完成探索、返航、地图保存及 F1→F3 导航；详见独立验收指标。')
        self.current_request = None
        return [{'kind': 'stop_navigation'}, {'kind': 'obstacle', 'enabled': False}]

    def map_saved(self, success, point_count, now, message=''):
        if self.stage != 'saving_map':
            return []
        if not success or point_count < self.scenario.get('minimum_saved_points', 500):
            return self.fail(now, '彩色地图保存未通过：'+message)
        self.completed_stages.append('saving_map')
        self.enter('navigating', now, '使用 SLAM 位姿从 F1 前往 F3，启用实际移动障碍。')
        return [{'kind': 'obstacle', 'enabled': True}]+self.request_route('navigation_f1_f3', now)

    def snapshot(self):
        return {k: getattr(self, k) for k in ('run_id', 'stage', 'message', 'origin', 'current_request', 'goal', 'goal_region', 'current_goals', 'current_goals_sha256', 'events', 'completed_stages', 'heading_alignment')}
