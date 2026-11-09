"""不可变目标几何、坐标系变换与基于实测时间戳的 dwell。

职责：定义 ``Goal``/``RigidTransform``/``ArrivalWindow``，把 schema-2 区域目标
解析成冻结几何，并在 camera_init 内做包含判定与 dwell 计时；目标几何与碰撞
规划相互独立。

边界：本模块不创建 ROS 节点、不接受真值控制输入、不产生机器人命令，也不写
文件或日志。长度单位统一为 m；几何字段的 dwell/timeout 为仿真秒，回执与窗口
使用整数纳秒戳。legacy 航点保留 3D 开放 .22 m 球表示；新目标必须在执行前
显式预注册区域边界。
"""

from dataclasses import dataclass, replace
import hashlib
import json
import math

import numpy as np
from scipy.spatial.transform import Rotation

# 3x3 单位阵；Goal.axes 的行是局部 along/lateral/normal 轴。
IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def number(value, name):
    """校验并返回有限数值；拒绝 bool、非数值类型与 NaN/±inf。"""
    if isinstance(value, (bool, np.bool_)) or not isinstance(
        value, (int, float, np.integer, np.floating)
    ):
        raise ValueError(name + " must be a finite number")
    if not math.isfinite(value):
        raise ValueError(name + " must be finite")
    return float(value)


def vector(value, size, name):
    """校验定长向量并逐分量转成有限 float 元组；长度不符抛 ValueError。"""
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != size:
        raise ValueError(name + " has invalid dimensions")
    return tuple(number(v, name) for v in value)


def basis(value):
    """校验正交右手基：3x3、``A Aᵀ = I``（atol 1e-8）且 ``det = 1``。"""
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != 3:
        raise ValueError("axes must contain three 3D vectors")
    axes = np.array([vector(v, 3, "axes") for v in value], dtype=float)
    if (
        axes.shape != (3, 3)
        or not np.allclose(axes @ axes.T, np.eye(3), atol=1e-8, rtol=0)
        or not math.isclose(np.linalg.det(axes), 1.0, abs_tol=1e-8)
    ):
        raise ValueError("axes must be an orthonormal right-handed basis")
    return tuple(tuple(float(v) for v in row) for row in axes)


@dataclass(frozen=True)
class Goal:
    """调用方坐标系内的不可变目标几何，构造时校验。

    ``center`` 与各长度字段单位为 m；``axes`` 的行是局部 along/lateral/normal 轴；
    ``radius``/``height_half_span`` 用于 sphere 与 disc_prism，
    ``half_extents`` 用于 oriented_box；``dwell_sim_s``/``timeout_sim_s`` 为
    仿真秒；``control_extents`` 是同心同轴同 dwell、但更严格的停止带（供控制侧
    到达判定）。``legacy`` 标记由旧 ``waypoints`` 解析出的 .22 m 球目标。
    """

    goal_id: str
    center: tuple
    kind: str
    axes: tuple = IDENTITY  # 行是坐标系内的 along/lateral/normal 轴。
    radius: float = 0.0
    height_half_span: float = 0.0
    half_extents: tuple = (0.0, 0.0, 0.0)
    dwell_sim_s: float = 0.4
    timeout_sim_s: float = 90.0
    route_surface_id: str | None = None
    legacy: bool = False
    control_extents: tuple | None = None  # 同心/同轴/同 dwell，但更严格的停止带。

    def __post_init__(self):
        if not isinstance(self.goal_id, str) or not self.goal_id.strip():
            raise ValueError("goal_id required")
        if self.kind not in ("sphere", "disc_prism", "oriented_box"):
            raise ValueError("invalid kind")
        object.__setattr__(self, "center", vector(self.center, 3, "center"))
        # 1000 m 帧边界（硬编码）：超出即视为坐标或单位错误。
        if max(map(abs, self.center)) > 1000:
            raise ValueError("goal center exceeds 1000 m frame bounds")
        object.__setattr__(self, "axes", basis(self.axes))
        object.__setattr__(self, "half_extents", vector(self.half_extents, 3, "half_extents"))
        for name in ("radius", "height_half_span", "dwell_sim_s", "timeout_sim_s"):
            object.__setattr__(self, name, number(getattr(self, name), name))
        # dwell 必须为正，且 timeout 必须严格大于 dwell（仿真秒）。
        if self.dwell_sim_s <= 0 or self.timeout_sim_s <= self.dwell_sim_s:
            raise ValueError("invalid dwell/timeout")
        if self.kind in ("sphere", "disc_prism") and self.radius <= 0:
            raise ValueError("radius must be positive")
        if self.kind == "disc_prism" and self.height_half_span <= 0:
            raise ValueError("height bound required")
        if self.kind == "oriented_box" and min(self.half_extents) <= 0:
            raise ValueError("positive box extents required")
        # 控制带必须为正且逐维包含在外层区域内。
        if self.control_extents is not None:
            # legacy 目标不允许再收窄控制带，避免改变旧航点语义。
            if self.legacy:
                raise ValueError("legacy waypoint control cannot be changed")
            outer = (
                (self.radius,)
                if self.kind == "sphere"
                else (
                    (self.radius, self.height_half_span)
                    if self.kind == "disc_prism"
                    else self.half_extents
                )
            )
            control = vector(self.control_extents, len(outer), "control_band")
            if min(control) <= 0 or any(i > o for i, o in zip(control, outer)):
                raise ValueError("control band must be positive and contained in outer region")
            object.__setattr__(self, "control_extents", control)

    def control_goal(self):
        """返回把外层区域收窄到控制带后的新 Goal（不修改原对象）。"""
        if self.control_extents is None:
            return self
        if self.kind == "sphere":
            return replace(self, radius=self.control_extents[0], control_extents=None)
        if self.kind == "disc_prism":
            return replace(
                self,
                radius=self.control_extents[0],
                height_half_span=self.control_extents[1],
                control_extents=None,
            )
        return replace(self, half_extents=self.control_extents, control_extents=None)

    def control_arrival_definition(self):
        """返回控制带的 arrival 定义，供回执逐字段比较。"""
        return self.control_goal().definition()["arrival"]

    def definition(self):
        """返回可 JSON 序列化的规范定义；字段由构造顺序决定，哈希消费它。"""
        arrival = dict(type=self.kind, dwell_sim_s=self.dwell_sim_s)
        if self.kind == "sphere":
            arrival["radius_m"] = self.radius
        else:
            arrival["axes"] = [list(row) for row in self.axes]
        if self.kind == "disc_prism":
            arrival.update(radius_m=self.radius, height_half_span_m=self.height_half_span)
        if self.kind == "oriented_box":
            arrival["half_extents_m"] = list(self.half_extents)
        if self.control_extents is not None:
            arrival["control_band"] = self.control_arrival_definition()
        return dict(
            goal_id=self.goal_id,
            center=list(self.center),
            arrival=arrival,
            timeout_sim_s=self.timeout_sim_s,
            route_surface_id=self.route_surface_id,
            legacy=self.legacy,
        )


def parse_goal(raw):
    """把 schema-2 目标字典解析为 ``Goal``（arrival 必须显式，构造时全量校验）。

    要求 ``goal_id`` 非空且 <=100 字符、``arrival`` 为对象且 type 为三种之一、
    ``center`` 为 3 维有限向量、``dwell_sim_s > 0`` 且
    ``timeout_sim_s > dwell``。``control_band`` 只允许覆盖几何维度/type/axes/
    dwell，不能改 center、timeout 或区域类型；dwell 与 axes 必须与外层一致，
    几何必须为正且包含在外层内。
    """
    if not isinstance(raw, dict):
        raise ValueError("goal must be an object")
    gid = raw.get("goal_id")
    if not isinstance(gid, str) or not gid.strip() or len(gid) > 100:
        raise ValueError("goal_id must be a nonempty string")
    arrival = raw.get("arrival")
    if not isinstance(arrival, dict):
        raise ValueError("arrival must be an explicit object")
    kind = arrival.get("type")
    if kind not in ("sphere", "disc_prism", "oriented_box"):
        raise ValueError("unknown arrival type")
    center = vector(raw.get("center"), 3, "center")
    dwell = number(arrival.get("dwell_sim_s", 0.4), "dwell_sim_s")
    timeout = number(raw.get("timeout_sim_s", 90.0), "timeout_sim_s")
    if dwell <= 0 or timeout <= dwell:
        raise ValueError("invalid dwell or timeout")
    axes = IDENTITY if kind == "sphere" else basis(arrival.get("axes", IDENTITY))
    r = number(arrival.get("radius_m", 0.0), "radius_m")
    h = number(arrival.get("height_half_span_m", 0.0), "height_half_span_m")
    ext = vector(arrival.get("half_extents_m", (0.0, 0.0, 0.0)), 3, "half_extents_m")
    if kind in ("sphere", "disc_prism") and r <= 0:
        raise ValueError("radius must be positive")
    if kind == "disc_prism" and h <= 0:
        raise ValueError("height bound is mandatory")
    if kind == "oriented_box" and min(ext) <= 0:
        raise ValueError("all box extents must be positive")
    surface = raw.get("route_surface_id")
    if surface is not None and (not isinstance(surface, str) or not surface.strip()):
        raise ValueError("invalid route_surface_id")
    control = None
    if "control_band" in arrival:
        band = arrival["control_band"]
        if not isinstance(band, dict):
            raise ValueError("control_band must be a geometry object")
        dimensions = {
            "sphere": ("radius_m",),
            "disc_prism": ("radius_m", "height_half_span_m"),
            "oriented_box": ("half_extents_m",),
        }[kind]
        allowed = set(dimensions) | {"type", "axes", "dwell_sim_s"}
        if set(band) - allowed:
            raise ValueError("control_band cannot override center, timeout or geometry kind")
        if band.get("type", kind) != kind:
            raise ValueError("control kind must match outer region")
        if number(band.get("dwell_sim_s", dwell), "control dwell") != dwell:
            raise ValueError("control dwell must match outer region")
        if "axes" in band and basis(band["axes"]) != axes:
            raise ValueError("control axes must match outer region")
        if kind == "oriented_box":
            control = vector(band.get("half_extents_m"), 3, "control half extents")
        else:
            control = tuple(number(band.get(name), "control " + name) for name in dimensions)
    return Goal(
        gid, center, kind, axes, r, h, ext, dwell, timeout, surface, control_extents=control
    )


def parse_request(raw):
    """解析一条导航请求：``goals``（schema_version 2）或 legacy ``waypoints`` 二选一。

    ``frame_id`` 必须是 ``camera_init``（本层不做坐标变换）；条目数 1..500。
    legacy 航点转成半径 .22 m、``legacy=True`` 的球目标；区域目标要求
    ``schema_version == 2`` 且 ``goal_id`` 不重复。返回
    ``(request_id, goals)``。
    """
    if not isinstance(raw, dict):
        raise ValueError("request must be an object")
    rid = raw.get("request_id")
    if not isinstance(rid, str) or not 1 <= len(rid) <= 128:
        raise ValueError("request_id required, max128")
    if raw.get("frame_id") != "camera_init":
        raise ValueError("NAV request must already be in camera_init")
    if ("goals" in raw) == ("waypoints" in raw):
        raise ValueError("exactly one of goals/waypoints is required")
    values = raw.get("goals", raw.get("waypoints"))
    if not isinstance(values, list) or not 1 <= len(values) <= 500:
        raise ValueError("goals must contain 1..500 entries")
    if "waypoints" in raw:
        # 与 legacy 解析器的有限数值转换和 1000 m 边界保持一致。
        points = np.asarray(values, dtype=float)
        if (
            points.shape != (len(values), 3)
            or not np.isfinite(points).all()
            or abs(points).max() > 1000
        ):
            raise ValueError("invalid legacy waypoints")
        goals = tuple(
            Goal(f"waypoint-{i}", tuple(p), "sphere", radius=0.22, legacy=True)
            for i, p in enumerate(points)
        )
    else:
        if raw.get("schema_version") != 2:
            raise ValueError("region goals require schema_version 2")
        goals = tuple(parse_goal(item) for item in values)
        if len({g.goal_id for g in goals}) != len(goals):
            raise ValueError("duplicate goal_id")
    return rid, goals


def definitions_sha256(goals):
    """对目标定义做 canonical JSON（排序键、紧凑分隔、禁 NaN）后取 sha256 十六进制。"""
    encoded = json.dumps(
        [g.definition() for g in goals], sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode()
    return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class RigidTransform:
    """刚体变换 ``p' = R p + t``，平移单位 m；构造时按 SO(3) 谓词校验 rotation。"""
    rotation: tuple
    translation: tuple

    def __post_init__(self):
        # 旋转矩阵与轴行基共用同一套 SO(3) 校验谓词。
        object.__setattr__(self, "rotation", basis(self.rotation))
        object.__setattr__(self, "translation", vector(self.translation, 3, "translation"))

    def point(self, p):
        """把一个 3D 点变换到目标帧（单位 m），返回 numpy 数组。"""
        return np.array(self.rotation) @ np.array(vector(p, 3, "position")) + np.array(
            self.translation
        )

    def goal(self, g):
        """变换目标中心与朝向：``axes' = axes · Rᵀ``，center 用 ``point``。"""
        axes = np.array(g.axes) @ np.array(self.rotation).T
        return replace(g, center=tuple(self.point(g.center)), axes=basis(axes))


def relative_world_transform(origin, scene_heading):
    """只用一次传感器航向冻结的变换；不要每帧用机体航向重建。

    ``scene_heading`` 为 ``yaw_camera_init_from_world``（rad），``origin`` 为
    初始化实测位置（m）。返回绕 Z 旋转 + 平移的 ``RigidTransform``。
    """
    h = number(scene_heading, "scene_heading")
    c, s = math.cos(h), math.sin(h)
    return RigidTransform(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)), vector(origin, 3, "origin"))


def initial_truth_alignment(slam_position, slam_quaternion, truth_position, truth_quaternion):
    """仅用于独立评估：由 SLAM 与真值初始位姿求一次 SE(3)，调用方永久保存。

    输入位置单位 m、四元数为 xyzw 顺序；两者范数须在 1±0.01 内，否则抛
    ``ValueError``。此函数不参与控制或到达判定。
    """
    qs = np.array(vector(slam_quaternion, 4, "SLAM quaternion"))
    qg = np.array(vector(truth_quaternion, 4, "truth quaternion"))
    if abs(np.linalg.norm(qs) - 1.0) > 0.01 or abs(np.linalg.norm(qg) - 1.0) > 0.01:
        raise ValueError("invalid quaternion norm")
    rs = Rotation.from_quat(qs).as_matrix()
    rg = Rotation.from_quat(qg).as_matrix()
    r = rs @ rg.T
    t = np.array(vector(slam_position, 3, "SLAM position")) - r @ vector(
        truth_position, 3, "truth position"
    )
    return RigidTransform(tuple(map(tuple, r)), tuple(t))


def contains(goal, p):
    """闭区域包含判定：sphere 用严格 ``<``，disc_prism/oriented_box 用 ``<=``。

    ``p`` 为实测位置（m），与 ``goal`` 同帧。边界只加机器精度 eps（约 16 个
    float eps 按坐标尺度缩放，亚皮米级），不是物理验收余量。
    """
    p = np.array(vector(p, 3, "raw measured position"))
    center = np.array(goal.center)
    if goal.kind == "sphere":
        return bool(math.hypot(*(p - center)) < goal.radius)
    local = np.array(goal.axes) @ (p - center)
    # 只补偿闭区域边界的浮点舍入：坐标尺度下约 16 个机器 eps（此处为亚皮米），
    # 不是物理验收余量。
    eps = 16 * np.finfo(float).eps * max(1.0, float(np.max(abs(p))), float(np.max(abs(center))))
    if goal.kind == "disc_prism":
        return bool(
            np.linalg.norm(local[:2]) <= goal.radius + eps
            and abs(local[2]) <= goal.height_half_span + eps
        )
    return bool(np.all(abs(local) <= np.array(goal.half_extents) + eps))


def contains_control(goal, p):
    """用更严格的停止带做包含判定（``goal.control_goal()``），供到达/通过证据用。"""
    return contains(goal.control_goal(), p)


class ArrivalWindow:
    """新区域目标的实测时间戳 dwell 窗口，独立于 ROS 墙钟定时器。

    对每一帧新接受的原始 odom 或一次保护事件调用 ``observe``；重复观测不推进
    窗口，stale/protected/越界会重置。legacy 请求沿用控制器已有的定时器语义。

    单位：``max_gap_sim_s`` 为仿真秒，时间戳为整数 ns。``reason`` 是诊断标签
    （waiting/reset/invalid_stamp/stale/protected/out_of_order/duplicate/
    invalid_pose/outside/gap_reset/dwell/arrived），会被回执消费方记录。
    """

    def __init__(self, goal, max_gap_sim_s=0.2):
        self.goal = goal
        # 仿真秒 -> 整数纳秒；必须为正，否则构造失败。
        self.max_gap_ns = round(number(max_gap_sim_s, "max_gap_sim_s") * 1e9)
        if self.max_gap_ns <= 0:
            raise ValueError("max_gap must be positive")
        self.since = None
        self.last_stamp = None
        self.reason = "waiting"

    def reset(self, reason="reset"):
        """清空窗口起点并记录重置原因（默认 "reset"）。"""
        self.since = None
        self.reason = reason

    def observe(self, p, stamp_ns, *, fresh=True, protected=False):
        """喂入一帧观测，返回是否已满足 dwell。

        ``p`` 为实测位置（m），``stamp_ns`` 为源时间戳（非负整数 ns）；
        ``fresh=False`` 或 ``protected=True`` 会重置窗口。乱序戳重置，重复戳只
        更新 ``reason="duplicate"`` 不推进；戳间隔超过 ``max_gap_ns`` 时先
        ``gap_reset`` 再重新起算。越界重置；满足 ``dwell_sim_s`` 时返回 True。
        """
        if type(fresh) is not bool or type(protected) is not bool:
            raise ValueError("health flags must be actual bool")
        if (
            not isinstance(stamp_ns, (int, np.integer))
            or isinstance(stamp_ns, (bool, np.bool_))
            or stamp_ns < 0
        ):
            self.reset("invalid_stamp")
            raise ValueError("stamp_ns must be nonnegative integer")
        if not fresh or protected:
            self.reset("stale" if not fresh else "protected")
            return False
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            if stamp_ns < self.last_stamp:
                self.reset("out_of_order")
            else:
                self.reason = "duplicate"
            return False
        gap = self.last_stamp is not None and stamp_ns - self.last_stamp > self.max_gap_ns
        self.last_stamp = int(stamp_ns)
        try:
            inside = contains_control(self.goal, p)
        except ValueError:
            self.reset("invalid_pose")
            raise
        if not inside:
            self.reset("outside")
            return False
        if gap:
            self.reset("gap_reset")
        if self.since is None:
            self.since = int(stamp_ns)
        arrived = stamp_ns - self.since >= round(self.goal.dwell_sim_s * 1e9)
        self.reason = "arrived" if arrived else "dwell"
        return bool(arrived)
