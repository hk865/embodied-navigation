"""任务阶段迁移，与 ROS 传输和仿真时间解耦。

职责：维护一次任务运行内的 stage（idle/waiting_sensors/exploring/returning/
saving_map/navigating/completed/failed/stopped）、事件日志与路线目标，并给出
迁移动作（发布路线、保存地图、开关动态障碍、停止导航），由调用方执行。

边界：本模块不创建 ROS 节点、不订阅话题、不产生速度命令，也不判断物理安全；
到达证据只表示“本阶段终点已满足”，不构成运动许可。它也不读 configs/ 或 run 目录。

本检出未实例化：唯一子类是 ``ContinuousMission``（``continuous_route.py``），
而它没有构造点，且 ``continuous_route_contract.enabled`` 为 ``false``。默认
mission 角色是 ``mission_runtime.py`` 里的 ``class Mission(Node)``（同名不同物）；
state_machine 在本检出未实例化，不要据本模块推断默认行为。

时间：``now``/``stage_started`` 等由调用方注入，单位为秒；事件日志的
``elapsed_s`` 是 ``round(now, 3)``，不是阶段内耗时。本模块不读 ROS 时钟。
"""

from dataclasses import dataclass, field
import math
from embodied_navigation.navigation.mission.route_regions import (
    transformed_route_goals,
    validate_route_regions,
)
from embodied_navigation.navigation.mission.goal_regions import (
    contains,
    contains_control,
    definitions_sha256,
    parse_goal,
)


# 活动 stage 集合：只有这五个参与 tick 超时与导航结果处理；idle/completed/
# failed/stopped 是静止状态，stop() 也只在这五个 stage 内生效。
ACTIVE_STAGES = {"waiting_sensors", "exploring", "returning", "saving_map", "navigating"}


@dataclass
class ObstacleEvidence:
    """一次真实动态障碍 hold 与恢复的累计证据。

    ``verified`` 同时要求 hold 事件（障碍可见且仿真确实更新过）与之后真实的
    resume；只有 hold 不算响应。resume 判据是 ``speed > 0.025``（单位 m/s，
    硬编码）；代码与现有文档未说明该阈值的标定来源。该阈值容忍障碍清除后
    重规划期间的若干零速指令。

    字段：``hold_events`` 是 hold 上升沿次数，``held`` 记录上一帧保持标志，
    ``resumed`` 一旦置真即锁存，直到下一次任务重置。
    """

    hold_events: int = 0
    held: bool = False
    resumed: bool = False

    def observe(self, held, speed, obstacle):
        """记录一帧障碍观测，无返回值。"""
        active_obstacle = (
            obstacle.get("active")
            and obstacle.get("visible_phase") in {"entering", "blocking", "leaving"}
            and obstacle.get("gazebo_updates", 0) > 0
        )
        if held and not self.held and active_obstacle:
            self.hold_events += 1
        # 障碍清除后重规划可能还有若干零速指令，故只用速度阈值判定恢复。
        if self.hold_events and not held and speed > 0.025:
            self.resumed = True
        self.held = held

    @property
    def verified(self):
        return self.hold_events > 0 and self.resumed


@dataclass
class Mission:
    """一次任务运行的阶段机（传感器 -> 探索 -> 返航 -> 存图 -> 导航）。

    ``stage`` 是公开状态，``events``/``completed_stages`` 是迁移日志。启用区域
    的请求使用 schema-2 目标并保留定义哈希供到达回执校验；legacy 航点请求改用
    最终实测位姿容差。两条分支都不单独信任控制器上报的 success。

    本类在本检出没有实例化点（唯一子类 ``ContinuousMission`` 也无构造点），
    默认 mission 角色是 ``mission_runtime.py`` 的 ROS 节点类 ``Mission``。
    """

    run_id: str = ""
    stage: str = "idle"
    message: str = "等待启动完整演示。"
    # 初始化阶段冻结的实测原点（m，camera_init 系）；waiting_sensors 内会重置。
    origin: list | None = None
    # 当前路线请求 id，格式为 ``{run_id}:{name}:{route_counter}``。
    current_request: str | None = None
    # 本阶段终点（m）；区域请求时等于最后一个目标中心。
    goal: list | None = None
    # 迁移事件日志与已完成 stage 列表，都是对外可见的记录数据。
    events: list = field(default_factory=list)
    completed_stages: list = field(default_factory=list)
    # 当前 stage 进入时间，以及稳定/断流计时起点；单位秒，来自调用方时钟。
    stage_started: float = 0.0
    ready_since: float | None = None
    sensor_missing_since: float | None = None
    # 同名路线在本次运行内的请求序号，从 1 开始。
    route_counter: int = 0
    scenario: dict = field(default_factory=dict)
    # 仅 relative_world_axes 使用：初始化时冻结的航向对齐（rad）。
    heading_alignment: dict | None = None
    # schema-2 区域目标：终点区域、全部目标定义与定义 sha256（None 表示 legacy）。
    goal_region: dict | None = None
    current_goals: list = field(default_factory=list)
    current_goals_sha256: str | None = None

    def enter(self, stage, now, message):
        """进入一个 stage，记录进入时间与一条事件日志。"""
        self.stage, self.stage_started, self.message = stage, now, message
        self.events.append({"stage": stage, "elapsed_s": round(now, 3), "message": message})

    def start(self, run_id, scenario, now=0.0):
        """启动一次新运行：校验场景、重置计时与日志，进入 waiting_sensors。

        ``scenario`` 必须通过 ``validate_route_regions``；已在活动 stage 内重复
        启动会抛 ``ValueError``。``now`` 单位为秒。
        """
        if self.stage in ACTIVE_STAGES:
            raise ValueError("已有任务正在运行。")
        validate_route_regions(scenario)
        self.run_id, self.scenario = run_id, scenario
        self.origin, self.current_request, self.goal = None, None, None
        self.goal_region, self.current_goals_sha256, self.current_goals = None, None, []
        self.heading_alignment = None
        self.events, self.completed_stages = [], []
        self.ready_since, self.sensor_missing_since = None, None
        self.route_counter = 0
        self.enter("waiting_sensors", now, "正在启动仿真，等待稳定的 SLAM、雷达、IMU 和 RGB 数据。")

    def stop(self, now):
        """用户停止：仅在活动 stage 内进入 ``stopped``，并清空当前请求。"""
        if self.stage in ACTIVE_STAGES:
            self.enter("stopped", now, "用户停止任务。")
        self.current_request = None

    def fail(self, now, reason):
        """进入 ``failed`` 并返回停止动作；reason 原样写进 stage 消息与事件日志。"""
        self.enter("failed", now, reason)
        self.current_request = None
        return [{"kind": "stop_navigation"}]

    def route(self, name):
        """把场景航点变换到本次任务的导航帧（camera_init）。

        ``relative_world_axes`` 应用冻结的航向对齐与任务原点；
        ``relative_initial_body`` 只加原点；其他 reference 视为已是绝对坐标，
        原样返回。长度单位与场景一致（m），返回新列表，不修改 scenario。
        """
        points = self.scenario[name]
        if self.scenario.get("waypoint_reference") == "relative_world_axes":
            angle = self.heading_alignment["yaw_camera_init_from_world"]
            c, s = math.cos(angle), math.sin(angle)
            return [
                [
                    self.origin[0] + c * p[0] - s * p[1],
                    self.origin[1] + s * p[0] + c * p[1],
                    self.origin[2] + p[2],
                ]
                for p in points
            ]
        if (
            self.scenario.get("waypoint_reference", "relative_initial_body")
            == "relative_initial_body"
        ):
            return [[p[i] + self.origin[i] for i in range(3)] for p in points]
        return [list(p) for p in points]

    def request_route(self, name, now):
        """登记并构造一条路线请求，返回迁移动作列表。

        ``route_counter`` 自增生成 ``request_id``（``{run_id}:{name}:{counter}``），
        同时设置 ``goal``、``current_goals``/``goal_region`` 与定义哈希。存在预注册
        区域目标时返回 schema 2 的 ``goals`` 请求，否则退回 legacy ``waypoints``
        请求；空路线直接 ``fail``。
        """
        points = self.route(name)
        if not points:
            return self.fail(now, f"场景没有配置 {name} 路线。")
        self.route_counter += 1
        self.current_request = f"{self.run_id}:{name}:{self.route_counter}"
        self.goal = points[-1]
        goals = transformed_route_goals(self.scenario, name, self.origin, self.heading_alignment)
        self.current_goals = [g.definition() for g in goals]
        self.current_goals_sha256 = definitions_sha256(goals) if goals else None
        self.goal_region = self.current_goals[-1] if goals else None
        if goals:
            return [
                {
                    "kind": "route",
                    "schema_version": 2,
                    "request_id": self.current_request,
                    "frame_id": self.scenario.get("frame_id", "camera_init"),
                    "goals": self.current_goals,
                }
            ]
        return [
            {
                "kind": "route",
                "request_id": self.current_request,
                "frame_id": self.scenario.get("frame_id", "camera_init"),
                "waypoints": points,
            }
        ]

    def region_completion_valid(self, status, pose):
        """校验一次 succeeded 状态确实完成了本任务的区域目标。

        要求定义哈希与定义列表完全一致、每目标一条回执且 dwell 起点递增、
        未被保护，且最终实测位姿落在终点控制区域内。回执的
        ``max_observation_gap_ns`` 必须等于 200_000_000（0.2 s，硬编码）。
        任一字段类型或内容不符即返回 False（fail-closed）。
        """
        if status.get("goals_definition_sha256") != self.current_goals_sha256:
            return False
        if status.get("goals_definitions") != self.current_goals:
            return False
        if status.get("waypoint_index") != len(self.current_goals) or status.get("total") != len(
            self.current_goals
        ):
            return False
        receipts = status.get("region_arrivals")
        if not isinstance(receipts, list) or len(receipts) != len(self.current_goals):
            return False
        # 回执按目标顺序，start_stamp_ns 必须严格递增；dwell 必须等于 stamp-start。
        previous = -1
        for raw, receipt in zip(self.current_goals, receipts):
            goal = parse_goal(raw)
            if not isinstance(receipt, dict) or receipt.get("goal_id") != goal.goal_id:
                return False
            if (
                receipt.get("request_id") != self.current_request
                or receipt.get("goals_definition_sha256") != self.current_goals_sha256
            ):
                return False
            stamp, start, dwell = (
                receipt.get(k) for k in ("stamp_ns", "start_stamp_ns", "dwell_ns")
            )
            if any(type(v) is not int for v in (stamp, start, dwell)):
                return False
            if (
                start <= previous
                or stamp < start
                or dwell != stamp - start
                or dwell < round(goal.dwell_sim_s * 1e9)
            ):
                return False
            if (
                receipt.get("region_inside") is not True
                or receipt.get("protected") is not False
                or receipt.get("arrival_definition") != goal.definition()["arrival"]
            ):
                return False
            # 观测间隔上限固定 200 ms（硬编码），必须与回执逐值相等。
            if (
                receipt.get("reason") != "arrived"
                or receipt.get("max_observation_gap_ns") != 200_000_000
            ):
                return False
            try:
                if not contains_control(goal, receipt.get("raw_position")):
                    return False
                if "control_band" in goal.definition()["arrival"]:
                    if (
                        receipt.get("control_region_inside") is not True
                        or receipt.get("control_arrival_definition")
                        != goal.control_arrival_definition()
                    ):
                        return False
            except (TypeError, ValueError):
                return False
            previous = stamp
        try:
            return pose is not None and contains_control(parse_goal(self.goal_region), pose)
        except (TypeError, ValueError):
            return False

    def tick(self, now, sensors_ok, pose, map_points=0):
        """推进阶段计时与超时，返回需要调用方执行的迁移动作（可能为空）。

        ``waiting_sensors`` 需要连续满足：传感器正常、航向已对齐（仅
        ``relative_world_axes``）、有 SLAM 位姿、地图点数达到
        ``minimum_start_points``（默认 100，硬编码），并在 ``initial_stable_s``
        （默认 3.0 s）内位姿漂移不超过 0.08 m（硬编码），才发起探索路线；
        初始化超过 ``startup_timeout_s``（默认 180 s）则失败。后续 stage 在
        传感器断流超过 ``sensor_failure_grace_s``（默认 3.0 s）或阶段超时
        （存图默认 30 s，其他默认 1500 s）时 fail-closed。
        """
        if self.stage not in ACTIVE_STAGES:
            return []
        if self.stage == "waiting_sensors":
            # 等待启动的硬上限：进入 waiting_sensors 后默认 180 s（可被场景覆盖）。
            if now - self.stage_started > self.scenario.get("startup_timeout_s", 180.0):
                return self.fail(now, "初始化超时：没有稳定的传感器和 SLAM 数据。")
            heading_ready = (
                self.scenario.get("waypoint_reference") != "relative_world_axes"
                or self.heading_alignment is not None
            )
            if (
                sensors_ok
                and heading_ready
                and pose is not None
                and map_points >= self.scenario.get("minimum_start_points", 100)
            ):
                if self.ready_since is None:
                    self.ready_since = now
                    self.origin = list(pose)
                # 原点漂移超过 0.08 m（硬编码）就重新起算稳定窗口。
                elif math.dist(pose, self.origin) > 0.08:
                    self.ready_since, self.origin = now, list(pose)
                # 默认连续稳定 3.0 s 才发起探索，避免刚启动的位姿抖动。
                elif now - self.ready_since >= self.scenario.get("initial_stable_s", 3.0):
                    self.enter("exploring", now, "沿规定路线探索，累计本次实际观测的彩色点云。")
                    return self.request_route("exploration", now)
            else:
                self.ready_since = None
            return []
        if not sensors_ok:
            # 传感器断流默认宽限 3.0 s；超过即失败停止（fail-closed）。
            self.sensor_missing_since = self.sensor_missing_since or now
            if now - self.sensor_missing_since > self.scenario.get("sensor_failure_grace_s", 3.0):
                return self.fail(now, "传感器或 SLAM 断流，任务停止。")
        else:
            self.sensor_missing_since = None
        # 存图阶段默认 30 s，其余阶段默认 1500 s；均可由场景覆盖。
        timeout = (
            self.scenario.get("save_timeout_s", 30.0)
            if self.stage == "saving_map"
            else self.scenario.get("stage_timeout_s", 1500.0)
        )
        if now - self.stage_started > timeout:
            return self.fail(now, f"{self.stage} 阶段超时。")
        return []

    def navigation_result(self, status, pose, now):
        """解释控制器对当前路线请求的终态。

        ``failed`` 直接进入失败；``succeeded`` 还要求区域回执（或 legacy 终点的
        实测容差）成立。最终 ``navigating`` 阶段在场景
        ``require_dynamic_obstacle``（默认 True）时还要求动态障碍证据。
        返回迁移动作；请求尚未结束（request_id 不匹配或 state 非终态）时返回
        空列表。
        """
        if self.stage not in {"exploring", "returning", "navigating"}:
            return []
        if status.get("request_id") != self.current_request:
            return []
        result = status.get("state")
        if result == "failed":
            # 控制器失败直接终止；message 原样拼入原因，便于保留上游文本。
            return self.fail(now, "导航失败：" + str(status.get("message", "未提供原因")))
        if result != "succeeded":
            return []
        # succeeded 不等于完成：还必须通过区域回执或 legacy 实测容差校验。
        if self.goal_region is not None and not self.region_completion_valid(status, pose):
            return self.fail(now, "导航区域完成证据不匹配，或实际 SLAM 位姿未进入终点区域。")
        if self.goal_region is None and (
            pose is None or math.dist(pose, self.goal) > self.scenario.get("goal_tolerance_m", 0.30)
        ):
            return self.fail(now, "导航报告完成，但实际 SLAM 位姿未达到本阶段终点。")
        completed = self.stage
        # 默认要求本次运行有动态障碍响应与恢复证据；场景可显式关闭。
        if (
            completed == "navigating"
            and self.scenario.get("require_dynamic_obstacle", True)
            and not status.get("dynamic_obstacle_verified", False)
        ):
            return self.fail(now, "已到导航终点，但缺少本次动态障碍响应与恢复证据。")
        self.completed_stages.append(completed)
        if completed == "exploring":
            # 探索完成 -> 返航：进入 returning 并请求返航路线。
            self.enter("returning", now, "探索航点已完成，沿规定返航路线返回初始位置。")
            return self.request_route("return_origin", now)
        if completed == "returning":
            # legacy 返航线要求实测位姿回到本次初始化原点（默认容差 0.30 m）。
            if self.goal_region is None and math.dist(pose, self.origin) > self.scenario.get(
                "goal_tolerance_m", 0.30
            ):
                return self.fail(now, "返航路线结束，但未回到本次初始化原点。")
            self.enter("saving_map", now, "已返回原点，正在保存本次探索的彩色 SLAM 地图。")
            return [{"kind": "save_map"}]
        # navigating 完成 -> completed；停止导航并关闭动态障碍注入。
        self.enter("completed", now, "已完成探索、返航、地图保存及 F1→F3 导航；详见独立验收指标。")
        self.current_request = None
        return [{"kind": "stop_navigation"}, {"kind": "obstacle", "enabled": False}]

    def map_saved(self, success, point_count, now, message=""):
        """保存地图结果：只有成功且点数达到 ``minimum_saved_points``（默认 500）才继续。

        成功后进入 ``navigating``，返回启用动态障碍的动作并请求 F1→F3 路线；
        点数不足或失败进入 ``failed``（message 原样拼入原因）。
        """
        if self.stage != "saving_map":
            return []
        # 保存失败或点数不足都 fail-closed；默认至少 500 个彩色点。
        if not success or point_count < self.scenario.get("minimum_saved_points", 500):
            return self.fail(now, "彩色地图保存未通过：" + message)
        self.completed_stages.append("saving_map")
        # 地图确认保存后才进入 F1->F3 导航，并启用实际移动障碍。
        self.enter("navigating", now, "使用 SLAM 位姿从 F1 前往 F3，启用实际移动障碍。")
        return [{"kind": "obstacle", "enabled": True}] + self.request_route("navigation_f1_f3", now)

    def snapshot(self):
        """返回可 JSON 序列化的公开状态快照（不含计时器与场景副本）。"""
        return {
            k: getattr(self, k)
            for k in (
                "run_id",
                "stage",
                "message",
                "origin",
                "current_request",
                "goal",
                "goal_region",
                "current_goals",
                "current_goals_sha256",
                "events",
                "completed_stages",
                "heading_alignment",
            )
        }
