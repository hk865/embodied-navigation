"""固定 original46 路线进度与显式的按序通过证据。

职责：把普通区域完成从“停留”改为“实测通过”，但不改动任何区域体积或高度
边界；connector 与阶段终点的 ``ArrivalWindow`` 契约保持不变。本模块不接触
ROS、不注入仿真位姿，也没有控制权。

休眠状态：``continuous_route_contract.enabled`` 默认 false（当前
``configs/planning/settings.json`` 配置为 false）；``pid_support`` 仍在导入期
取用 ``configuration``/``RouteProgress``/``CausalHandoffProjectionController``，
但只有 enabled 为真才构造。唯一子类 ``ContinuousMission`` 在本检出没有构造点。
"""

import copy
import hashlib
import json
import math
import numpy as np

from embodied_navigation.navigation.mission.goal_regions import (
    contains,
    contains_control,
    parse_goal,
)
from embodied_navigation.navigation.mission.state_machine import Mission
from embodied_navigation.planning.reference.global_route_reference import StableProjectionController
from embodied_navigation.control.tracking.cascade_core import Controller

# 连续路线状态行的 schema（路线进度、投影与硬停点）。
SCHEMA = "original46_continuous_route/v1"
# 普通区域“按序通过”回执的 schema，用来与硬停点的 arrived 回执区分。
PASS = "original_region_ordered_pass/v1"
# 三条路线的跨层 connector id：这些目标必须等待地形提供方确认后才能前进。
CONNECTORS = {"exploration:11", "return_origin:5", "navigation_f1_f3:7"}


def sha(value):
    """对一条路线/请求记录做 canonical JSON（排序键、紧凑分隔、禁 NaN）后取 sha256。"""
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def configuration(profile):
    """读取可选的连续路线契约；``enabled`` 必须是 bool，否则抛 ValueError。

    缺席或 ``enabled: false`` 时保持普通区域路径；启用时要求已审阅的有界引导
    参数（1.0/128/0.8，硬编码并逐值校验）且 mission46_required 为 True，其他
    任务不能选中本路线。当前默认配置为关闭。
    """
    # 默认关闭；配置里的同名键会覆盖下面三个冻结上限。
    value = profile.get("continuous_route_contract", {"enabled": False})
    if not isinstance(value, dict) or type(value.get("enabled", False)) is not bool:
        raise ValueError("continuous_route_contract.enabled must be boolean")
    result = dict(
        enabled=value.get("enabled", False),
        guide_spacing_m=1.0,
        max_guide_points=128,
        max_projection_advance_m=0.8,
    )
    result.update(value)
    if result["enabled"]:
        # 三个上限是审阅过的冻结值（m / 点数 / m），不是可调参数。
        if (
            result["guide_spacing_m"] != 1.0
            or result["max_guide_points"] != 128
            or result["max_projection_advance_m"] != 0.8
        ):
            raise ValueError("V34 uses the reviewed bounded original46 settings")
        # 只允许 original46 任务启用；mission46_required 为 false 时直接拒绝。
        if profile.get("mission46_required") is not True:
            raise ValueError("continuous route is limited to original46 mission")
    return result


def hard_indices(goals):
    """返回必须按硬停点处理的目标下标（connector + 路线终点）。

    要求目标是显式 original 区域（不能是 legacy），且路线名、数量与
    ``"name:i"`` 的顺序完全匹配：exploration 18、return_origin 14、
    navigation_f1_f3 14（硬编码）。否则抛 ``ValueError``。
    """
    if not goals or any(g.legacy for g in goals):
        raise ValueError("continuous route requires explicit original regions")
    names = {g.goal_id.split(":")[0] for g in goals}
    # 每条 original46 路线的区域数（硬编码，与 route_regions 的区域表一致）。
    counts = {"exploration": 18, "return_origin": 14, "navigation_f1_f3": 14}
    if len(names) != 1 or next(iter(names)) not in counts:
        raise ValueError("unknown original route")
    name = next(iter(names))
    # 顺序也必须匹配，避免重排后靠 goal_id 集合蒙混过关。
    if len(goals) != counts[name] or [g.goal_id for g in goals] != [
        f"{name}:{i}" for i in range(counts[name])
    ]:
        raise ValueError("original route identity/order/count required")
    return tuple(i for i, g in enumerate(goals) if g.goal_id in CONNECTORS or i == len(goals) - 1)


class RouteProgress:
    """一条路线请求的固定 original46 进度账本。

    ``points`` 由首次实测位置与冻结目标中心拼成，``arc`` 是 XY 平面累计弧长
    （m），``progress`` 单调不减。``next_index`` 是下一个未覆盖目标，
    ``target_index`` 是下一个必须等待 mission_hold 确认的 connector 硬停点。
    所有时间戳都是调用方时钟域的整数 ns，本类不读时钟。
    """

    def __init__(self, request_id, goals, goal_hash, settings):
        self.request_id = request_id
        self.goals = tuple(goals)
        self.goal_hash = goal_hash
        self.settings = settings
        # 硬停点下标：三条路线的 connector 与路线终点。
        self.hard = hard_indices(self.goals)
        # next_index 是下一个未覆盖目标；target_index 是当前等待确认的硬停点。
        self.target_index = self.hard[0]
        self.next_index = 0
        self.points = None
        self.arc = None
        self.route_sha = None
        # 单调推进的弧长进度（m，XY 平面）。
        self.progress = 0.0
        # 上一条观测的源时间戳（ns）与位置（m）。
        self.last_stamp = None
        self.last_position = None
        # 账本激活时间与下一段计时起点（调用方时钟 ns）。
        self.activated_ns = None
        self.next_activated_ns = None
        self.last_projection = None
        # 逐目标回执：普通区域为 PASS，硬停点为 arrived。
        self.receipts = []

    def activate(self, start, stamp_ns, clock_ns):
        """用首个实测位置冻结路线账本（只激活一次）。

        ``start`` 为实测位置（m），``stamp_ns`` 为传感器时间戳、
        ``clock_ns`` 为调用方时钟，二者都必须是 int。``points = [start] +
        目标中心``，按 XY 平面计算段长；任一段 <= 1e-6 m（垂直或重复）即报错。
        ``route_sha`` 冻结这条实际路线。
        """
        if self.points is not None:
            return
        start = np.asarray(start, dtype=float)
        if (
            start.shape != (3,)
            or not np.isfinite(start).all()
            or type(stamp_ns) is not int
            or type(clock_ns) is not int
        ):
            raise ValueError("actual finite measured route activation required")
        # 只用 XY 平面段长：body 位置沿地面/平台走，z 不参与弧长定义。
        self.points = np.vstack([start, [g.center for g in self.goals]])
        lengths = np.linalg.norm(np.diff(self.points, axis=0)[:, :2], axis=1)
        if np.any(lengths <= 1e-6):
            raise ValueError("original route has a vertical/duplicate leg")
        self.arc = np.r_[0.0, np.cumsum(lengths)]
        self.route_sha = sha(self.points.tolist())
        self.activated_ns = self.next_activated_ns = clock_ns
        self.last_stamp = stamp_ns
        self.last_position = start.copy()

    def deadline_exceeded(self, clock_ns):
        """当前未覆盖目标是否超过 ``timeout_sim_s``（仿真秒，转成 ns 比较）。

        ``next_activated_ns`` 是上一段完成时的调用方时钟；未激活或目标已全部
        覆盖时返回 False。
        """
        return (
            self.next_index < len(self.goals)
            and self.next_activated_ns is not None
            and clock_ns - self.next_activated_ns
            > round(self.goals[self.next_index].timeout_sim_s * 1e9)
        )

    def project(self, pose):
        """有界最近 3D 投影；搜索窗由未覆盖序号与进度共同约束。

        账本独立于局部 SCAN 轨迹的弧坐标。用完整 xyz 距离、因果弧长窗口与
        “下一个未覆盖区域”限制，避免重叠的路线位置/楼层错误地授予前进。
        窗口：下界 ``progress-0.15``；上界取 ``target+1`` 顶点弧长、
        ``progress+0.8`` 与 ``next+1`` 顶点弧长 +0.25 三者最小值（均硬编码，m）。
        无候选时抛 ``ValueError``；返回投影字典并更新单调进度。
        """
        pose = np.asarray(pose, dtype=float)
        # 三个窗口余量都是硬编码米数：回看 0.15 m，前瞻 0.8 m / 0.25 m。
        low = max(0.0, self.progress - 0.15)
        high = min(
            self.arc[self.target_index + 1],
            self.progress + 0.8,
            self.arc[min(self.next_index + 1, len(self.arc) - 1)] + 0.25,
        )
        candidates = []
        for i, (a, b) in enumerate(zip(self.points[:-1], self.points[1:])):
            length = self.arc[i + 1] - self.arc[i]
            if self.arc[i + 1] < low or self.arc[i] > high:
                continue
            d = b - a
            u = float(
                np.clip(
                    (pose - a) @ d / (d @ d),
                    max(0.0, (low - self.arc[i]) / length),
                    min(1.0, (high - self.arc[i]) / length),
                )
            )
            point = a + u * d
            s = float(self.arc[i] + u * length)
            candidates.append((float((pose - point) @ (pose - point)), s, i, point))
        if not candidates:
            raise ValueError("no causal original-route projection")
        distance, s, segment, point = min(candidates, key=lambda x: (x[0], x[1], x[2]))
        # 进度只增不减，避免回退授予重复通过。
        self.progress = max(self.progress, s)
        self.last_projection = dict(
            arc_m=s,
            monotonic_progress_m=self.progress,
            distance_xyz_m=math.sqrt(distance),
            segment=segment,
            projected_xyz=point.tolist(),
        )
        return self.last_projection

    def observe(self, pose, stamp_ns, clock_ns, protected=False):
        """喂入一帧未保护观测，返回普通区域的按序通过回执或 None。

        首次调用用该帧激活账本。要求时间戳严格递增；``protected=True`` 或与
        上一帧间隔 > 200 ms（硬编码）时跳过，不产生通过证据。普通目标必须在
        控制区域内，且投影弧长与其有序顶点弧长相差 <= 0.5 m（硬编码），防止
        重叠楼层误判通过。回执 ``dwell_ns=0``、reason 为 ``passed``，并推进
        ``next_index``；硬停点不走这里。
        """
        if self.points is None:
            self.activate(pose, stamp_ns, clock_ns)
            return None
        if self.next_index >= len(self.goals) or stamp_ns <= self.last_stamp:
            return None
        old_stamp = self.last_stamp
        self.last_stamp = stamp_ns
        self.last_position = np.asarray(pose, dtype=float).copy()
        # 保护事件或观测间隔超过 200 ms 都不构成连续通过证据。
        if protected or stamp_ns - old_stamp > 200_000_000:
            return None
        projection = self.project(pose)
        index = self.next_index
        goal = self.goals[index]
        if index in self.hard or not contains_control(goal, pose):
            return None
        # 实测点必须同时接近有序顶点的 3D 区域与路线弧长；从另一个重叠楼层
        # 经过不能算作通过。
        if abs(projection["arc_m"] - float(self.arc[index + 1])) > 0.5:
            return None
        value = dict(
            schema=PASS,
            completion_semantics="ordered_pass_without_dwell",
            request_id=self.request_id,
            goal_id=goal.goal_id,
            waypoint_index=index,
            goals_definition_sha256=self.goal_hash,
            route_sha256=self.route_sha,
            stamp_ns=stamp_ns,
            start_stamp_ns=stamp_ns,
            dwell_ns=0,
            raw_position=np.asarray(pose, float).tolist(),
            region_inside=bool(contains(goal, pose)),
            control_region_inside=True,
            protected=False,
            reason="passed",
            max_observation_gap_ns=200_000_000,
            previous_observation_stamp_ns=old_stamp,
            arrival_definition=goal.definition()["arrival"],
            control_arrival_definition=goal.control_arrival_definition(),
            original_route_surface_id=goal.route_surface_id,
            route_vertex_arc_m=float(self.arc[index + 1]),
            route_projection=copy.deepcopy(projection),
            navigation_ground_truth_used=False,
            original_dwell_satisfied=False,
        )
        self.receipts.append(value)
        self.next_index += 1
        self.next_activated_ns = clock_ns
        return value

    def accept_hard_receipt(self, value, clock_ns):
        """接受一个 connector/终点硬停点的到达回执（必须按序且满足 dwell）。

        要求当前 ``next_index`` 正是 ``target_index`` 且 goal_id 匹配；回执
        reason 为 ``arrived`` 且 ``dwell_ns`` 不小于目标 ``dwell_sim_s``，
        否则抛 ``ValueError``。接受后推进 next_index，但 target_index 保持不动，
        直到独立 ``mission_hold`` 确认地形提供方回执。
        """
        if (
            self.next_index != self.target_index
            or value.get("goal_id") != self.goals[self.next_index].goal_id
        ):
            raise ValueError("hard arrival cannot skip ordered original regions")
        if value.get("reason") != "arrived" or value.get("dwell_ns", -1) < round(
            self.goals[self.next_index].dwell_sim_s * 1e9
        ):
            raise ValueError("original hard arrival dwell required")
        self.receipts.append(copy.deepcopy(value))
        self.next_index += 1
        self.next_activated_ns = clock_ns
        # target_index 停在 connector，直到独立 mission_hold 确认地形提供方回执。

    def release_hard_target(self):
        """确认后释放当前硬停点，返回新的 ``target_index``（无变化时原样返回）。"""
        if self.next_index > self.target_index and self.next_index < len(self.goals):
            self.target_index = next(i for i in self.hard if i >= self.next_index)
        return self.target_index

    def terminal_heading(self, index):
        """目标 ``index`` 处入段方向的航向角（rad，``atan2(dy, dx)``，仅 XY）。"""
        d = self.points[index + 1, :2] - self.points[index, :2]
        return math.atan2(d[1], d[0])

    def reference_points(self, end_index=None):
        """从当前未覆盖目标到指定硬停点的原始中心与固定间距引导点。

        ``end_index`` 默认当前 ``target_index``，必须落在
        ``[next_index, target_index]`` 内，否则抛 ValueError。每段按 1 m 间距
        （硬编码）取采样点，采样点需位于进度 +0.35 m（硬编码）之后（段终点
        总是保留）；总点数必须 <= 128（硬编码），超出报错。返回点列表（m）。
        """
        end = self.target_index if end_index is None else int(end_index)
        if not self.next_index <= end <= self.target_index:
            raise ValueError("local guide endpoint outside ordered current span")
        points = []
        for i in range(self.next_index, end + 1):
            a, b = self.points[i], self.points[i + 1]
            length = self.arc[i + 1] - self.arc[i]
            offsets = np.arange(1.0, length - 0.5 + 1e-8, 1.0)
            for offset in offsets:
                if self.arc[i] + offset > self.progress + 0.35:
                    points.append((a + (b - a) * offset / length).tolist())
            points.append(b.tolist())
        # 引导点数上限 128（硬编码），与 configuration 冻结的 max_guide_points 对齐。
        if not points or len(points) > 128:
            raise ValueError("bounded continuous guide count exceeded")
        return points

    def missed_pass_requires_replan(self):
        """判断是否漏过普通区域并需要重规划；不为跳过的覆盖授予通过。

        条件：已投影弧长 >= 未覆盖顶点弧长 + 0.20 m（硬编码）且实测位置不在该
        目标控制区域内。硬停点与已覆盖完不触发。这是恢复证据，不是合成的路径
        耗尽回执；局部执行路径可以丢弃，但路线进度保留。
        """
        if (
            self.last_projection is None
            or self.last_position is None
            or self.next_index in self.hard
            or self.next_index >= len(self.goals)
        ):
            return False
        vertex = float(self.arc[self.next_index + 1])
        return self.last_projection["arc_m"] >= vertex + 0.20 and not contains_control(
            self.goals[self.next_index], self.last_position
        )

    def guard_route(self):
        """返回截至当前硬停点的路线保护点（``target_index+2`` 个点的副本）。"""
        return self.points[: self.target_index + 2].copy()

    def status(self):
        """返回可 JSON 序列化的账本状态，供控制器状态行消费。

        包含 schema、请求/哈希、路线点与 sha、激活/下段时间戳、下一目标超时、
        进度、投影、通过/停留计数与固定语义字段；未激活时
        ``route_points_xyz`` 为 None。``navigation_ground_truth_used=False``
        表明证据来自实测 SLAM。
        """
        return dict(
            schema=SCHEMA,
            request_id=self.request_id,
            goals_definition_sha256=self.goal_hash,
            route_sha256=self.route_sha,
            route_points_xyz=None if self.points is None else self.points.tolist(),
            route_activation_stamp_ns=self.activated_ns,
            next_activated_ns=self.next_activated_ns if self.next_index < len(self.goals) else None,
            next_timeout_ns=round(self.goals[self.next_index].timeout_sim_s * 1e9)
            if self.next_index < len(self.goals)
            else None,
            next_coverage_index=self.next_index,
            hard_target_index=self.target_index,
            hard_stop_indices=list(self.hard),
            monotonic_progress_m=self.progress,
            measured_projection=copy.deepcopy(self.last_projection),
            ordinary_pass_count=sum(r.get("schema") == PASS for r in self.receipts),
            original_dwell_count=sum(r.get("reason") == "arrived" for r in self.receipts),
            completion_semantics="ordinary_ordered_pass_and_original_hard_stop_dwell",
            original_goal_geometry_unchanged=True,
            navigation_ground_truth_used=False,
        )


def completion_valid(status, raw_goals, request_id, goal_hash, pose):
    """新的完成门；显式不把 legacy 的“全部 dwell”算作 PASS。

    校验：状态行请求/哈希/目标列表/序号与计数一致；连续路线 schema、sha 与目标
    中心逐点吻合；硬停点集合等于 connector + 终点；每个回执的目标 id、定义、
    控制带、200 ms 间隔与时间戳链一致；硬停点要求 ``arrived`` 且 dwell 达标，
    普通区域要求 PASS、dwell 0、按序弧长与单调进度；最后实测位姿在终点控制区域。
    任何字段或类型异常都返回 False（fail-closed）。
    """
    try:
        goals = tuple(parse_goal(g) for g in raw_goals)
        if (
            status.get("request_id") != request_id
            or status.get("goals_definition_sha256") != goal_hash
            or status.get("goals_definitions") != raw_goals
            or status.get("waypoint_index") != len(goals)
            or status.get("total") != len(goals)
        ):
            return False
        route = status.get("continuous_route") or {}
        points = np.asarray(route.get("route_points_xyz"), dtype=float)
        if (
            route.get("schema") != SCHEMA
            or route.get("request_id") != request_id
            or route.get("goals_definition_sha256") != goal_hash
            or route.get("navigation_ground_truth_used") is not False
            or points.ndim != 2
            or points.shape[1] != 3
            or len(points) < len(goals) + 1
            or not np.isfinite(points).all()
            or sha(points.tolist()) != route.get("route_sha256")
            or not np.array_equal(points[1 : len(goals) + 1], np.asarray([g.center for g in goals]))
        ):
            return False
        # 前缀校验沿用同一条完整冻结路线；前缀止于必须的 connector，
        # 而不是新声明的普通硬停点。
        hard = set(route.get("hard_stop_indices", []))
        full_count = len(points) - 1
        expected_hard = {
            i
            for i in range(full_count)
            if f"{goals[0].goal_id.split(':')[0]}:{i}" in CONNECTORS or i == full_count - 1
        }
        if hard != expected_hard or len(goals) - 1 not in hard:
            return False
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0)[:, :2], axis=1))]
        receipts = status.get("region_arrivals")
        if not isinstance(receipts, list) or len(receipts) != len(goals):
            return False
        previous = -1
        previous_arc = 0.0
        for index, (g, r) in enumerate(zip(goals, receipts)):
            if (
                not isinstance(r, dict)
                or r.get("goal_id") != g.goal_id
                or r.get("request_id") != request_id
                or r.get("goals_definition_sha256") != goal_hash
                or r.get("protected") is not False
                or r.get("region_inside") is not True
                or r.get("control_region_inside") is not True
                or r.get("arrival_definition") != g.definition()["arrival"]
                or r.get("control_arrival_definition") != g.control_arrival_definition()
                or r.get("max_observation_gap_ns") != 200_000_000
                or not contains_control(g, r.get("raw_position"))
            ):
                return False
            stamp, start, dwell = (r.get(k) for k in ("stamp_ns", "start_stamp_ns", "dwell_ns"))
            if (
                any(type(v) is not int for v in (stamp, start, dwell))
                or start <= previous
                or stamp < start
                or dwell != stamp - start
            ):
                return False
            # 硬停点必须 arrived 且满足 dwell；普通区域则是 dwell=0 的 PASS。
            if index in hard:
                if (
                    r.get("schema") == PASS
                    or r.get("reason") != "arrived"
                    or dwell < round(g.dwell_sim_s * 1e9)
                ):
                    return False
            else:
                p = r.get("route_projection") or {}
                observed = r.get("previous_observation_stamp_ns")
                if (
                    r.get("schema") != PASS
                    or r.get("completion_semantics") != "ordered_pass_without_dwell"
                    or r.get("reason") != "passed"
                    or dwell != 0
                    or r.get("original_dwell_satisfied") is not False
                    or r.get("route_sha256") != route["route_sha256"]
                    or r.get("waypoint_index") != index
                    or type(observed) is not int
                    or not 0 < stamp - observed <= 200_000_000
                    or r.get("original_route_surface_id") != g.route_surface_id
                    or r.get("navigation_ground_truth_used") is not False
                    or not math.isclose(
                        r.get("route_vertex_arc_m", -1.0), arc[index + 1], abs_tol=1e-8
                    )
                    or abs(p.get("arc_m", -1.0) - arc[index + 1]) > 0.5
                    or p.get("monotonic_progress_m", -1.0) < previous_arc
                ):
                    return False
                previous_arc = p["monotonic_progress_m"]
            previous = stamp
        return pose is not None and contains_control(goals[-1], pose)
    except (TypeError, ValueError, KeyError, IndexError, AttributeError):
        return False


class ContinuousMission(Mission):
    """``Mission`` 的子类：用 ``completion_valid`` 替换区域完成校验。

    本检出没有构造点且 ``continuous_route_contract.enabled`` 默认 false，
    因此不改变默认主路径行为。
    """

    def region_completion_valid(self, status, pose):
        return completion_valid(
            status, self.current_goals, self.current_request, self.current_goals_sha256, pose
        )


class CausalHandoffProjectionController(StableProjectionController):
    """沿用现有实测 3D 局部路径交接，但不做 V33 的强制重置。

    重规划时局部弧长切换坐标系，独立固定路线账本不切换；PI、滤波、命令与
    nearest3D 交接仍沿用现有 cascade，有界因果投影保持不变。
    """

    def set_path(self, *args, **kwargs):
        """把路径设置直接委托给 ``cascade_core.Controller.set_path``（不做重置）。"""
        return Controller.set_path(self, *args, **kwargs)
