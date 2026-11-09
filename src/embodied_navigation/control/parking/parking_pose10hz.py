"""停车 10 Hz 位姿栅格：高频运动既不能授予、也不能否决停车。

栅格时间统一是仿真/header 纳秒（不是墙钟）。每个栅格点取该时刻或之前最近的一个
"不同" 的真实位姿，不做插值；首个位姿（T0 锚点）不可变；缺失的栅格保持缺失，
后续数据不会重置这个固定窗口。

边界：本模块只做采样与几何证据（PoseGrid / CaptureGrid），不判定停车业务成功，
也不读原始陀螺或 IMU；零速/零输入契约在 parking_zero_contract，阶段推进在
ordinary_parking 与 transition_gate。
"""

import copy
import math

# 10 Hz 栅格周期，单位纳秒（硬编码）。
PERIOD_NS = 100_000_000
# 固定窗口 5 s：50 个周期、51 个栅格点（含 T0），单位纳秒（硬编码）。
DURATION_NS = 5_000_000_000
# 单个栅格点允许的位姿源龄上限 300 ms，与导航全局 pose 新鲜度同量级（硬编码）。
SOURCE_AGE_NS = 300_000_000
# 相对 T0 锚点的漂移上限：平面 0.05 m、偏航 0.1 rad（硬编码）。
XY_LIMIT_M = 0.05
YAW_LIMIT_RAD = 0.1
# 冻结的契约快照，随证据一起输出，供消费端核对用的是哪一版栅格语义。
CONFIG = dict(
    schema="parking_pose10hz_fixed_anchor/v1",
    period_ns=PERIOD_NS,
    duration_ns=DURATION_NS,
    xy_drift_m=XY_LIMIT_M,
    yaw_drift_rad=YAW_LIMIT_RAD,
    source_age_ns=SOURCE_AGE_NS,
    sampling="latest_distinct_header_at_or_before_grid",
    interpolation=False,
    high_frequency_motion_gate=False,
    adjacent_increment_gate=False,
)


def configuration(value):
    """把 profile 里的 10 Hz 配置交给安全侧逐值校验。

    参数：value 为 profile 中的配置值；None 表示未配置。
    返回：校验结果（布尔），None 时固定为 False。
    """
    if value is None:
        return False
    from embodied_navigation.control.safety.safety_configuration import pose

    return pose(value)


def wrap(value):
    """把角度归一到闭区间 [-pi, pi]，单位 rad；只用于差值，不改绝对朝向。"""
    return math.atan2(math.sin(value), math.cos(value))


def from_feedback(feedback):
    """把 Controller/R86 接受的 canonical SLAM 机体回调整理成一行 10 Hz 位姿。

    参数：feedback 为 canonical SLAM 反馈字典。
    返回：位姿行字典；四元数由 wxyz 重排为 xyzw，并同时保留 ROS 时钟与接收墙钟。
    失败：require_canonical_slam_feedback 拒绝非 canonical 反馈（真值位姿、外来
    producer 等）时直接 raise，本函数不降级、不替换来源。
    """
    from embodied_navigation.control.safety.feedback_source import require_canonical_slam_feedback

    require_canonical_slam_feedback(feedback)
    q = list(feedback["quaternion_wxyz"])
    row = dict(
        stamp_ns=feedback["stamp_ns"],
        position=list(feedback["position_world_xyz"]),
        quaternion_xyzw=q[1:] + q[:1],
        body_velocity=list(feedback["origin_velocity_body"]),
        received_sim_ns=feedback["callback_ros_clock_ns"],
        received_wall_ns=feedback["received_wall_ns"],
        source_received_wall_ns=feedback["received_wall_ns"],
        # 来源 topic 与"未用导航真值"是硬编码契约；下面的 pose() 会复核这两项。
        source_topic="/demo/slam/body_odom",
        navigation_ground_truth_used=False,
        pose_producer=feedback["pose_producer"],
    )
    pose(row)
    return row


def pose(row):
    """校验一行原始 SLAM 位姿，取出 (stamp_ns, 位置, 偏航)。

    返回：三元组 (stamp_ns 纳秒整数, [x, y, z] 米, yaw 弧度)。
    校验：stamp 为非负整数；位置与四元数全有限；四元数范数在 [0.98, 1.02]（与导航侧
    位姿校验同一区间）；来源 topic 必须为 /demo/slam/body_odom；非真值输入时
    producer 必须是 fast_livo2_async_backend。
    失败：任一不满足即 raise ValueError；这里不修值、不跳过、不插值。
    """
    stamp = row["stamp_ns"]
    p = list(row["position"])
    q = list(row["quaternion_xyzw"])
    if (
        type(stamp) is not int
        or stamp < 0
        or len(p) != 3
        or len(q) != 4
        or not all(type(v) in (int, float) and math.isfinite(v) for v in p + q)
        or not 0.98 <= sum(v * v for v in q) <= 1.02
        or row.get("source_topic") != "/demo/slam/body_odom"
        or type(row.get("navigation_ground_truth_used")) is not bool
        or (
            row["navigation_ground_truth_used"] is False
            and row.get("pose_producer") != "fast_livo2_async_backend"
        )
    ):
        raise ValueError("Invalid original SLAM pose for 10Hz parking")
    # 机体偏航（Z-Y-X 欧拉序的 yaw 分量，单位 rad），只取 SLAM 位姿，不用 IMU yaw。
    yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]))
    return stamp, p, yaw


class PoseGrid:
    """以首个不可变的因果位姿为锚点，建立固定 10 Hz 采样栅格（无插值）。

    栅格点只接受"真实、新鲜、因果"的位姿：源龄 <= 300 ms，且栅格时刻不早于位姿
    stamp。窗口内缺失的点记为 missing，不会被后续数据补上。
    """

    def __init__(self, start_ns, origin, *, duration_ns=DURATION_NS):
        """建立栅格并立即采出 T0 样本。

        参数：start_ns 为 T0 栅格时间（纳秒，仿真/header 时钟）；origin 为 T0 位姿行；
        duration_ns 为窗口长度（纳秒），None 表示无终点（CaptureGrid 的滚动栅格）。
        要求：0 <= start_ns - origin.stamp_ns <= 300 ms，即锚点必须是新鲜的真实位姿，
        否则 raise。
        """
        stamp, p, yaw = pose(origin)
        if type(start_ns) is not int or not 0 <= start_ns - stamp <= SOURCE_AGE_NS:
            raise ValueError("10Hz anchor must be a fresh causal actual pose")
        self.start_ns = start_ns
        self.end_ns = None if duration_ns is None else start_ns + duration_ns
        self.origin = copy.deepcopy(origin)
        self.base = p
        self.base_yaw = yaw
        # pending：已收到但尚未被栅格消耗的位姿行（含 T0）。
        self.pending = [copy.deepcopy(origin)]
        self.last_input = stamp
        self.last_pose = (p, yaw)
        # 已采样的位姿 stamp 水位：保证同一位姿不会被两个栅格点重复使用。
        self.last_sample_stamp = stamp
        self.next_grid = start_ns + PERIOD_NS
        self.samples = []
        self.missing = []
        self.motion_failure = None
        self._sample(start_ns, origin)

    def _sample(self, grid, row):
        """把一个栅格点绑定到一行真实位姿，并计算漂移量。

        记录：相对 T0 锚点的平面漂移（m）与偏航漂移（rad）、与上一采样的相邻增量，
        以及源龄（纳秒）。漂移首次越界时锁存 motion_failure，之后不再覆盖原因。
        """
        stamp, p, yaw = pose(row)
        previous = self.samples[-1] if self.samples else None
        sample = dict(
            grid_ns=grid,
            source_stamp_ns=stamp,
            source_age_ns=grid - stamp,
            position=p,
            yaw_rad=yaw,
            source=copy.deepcopy(row),
            xy_from_anchor_m=math.hypot(p[0] - self.base[0], p[1] - self.base[1]),
            yaw_from_anchor_rad=wrap(yaw - self.base_yaw),
            adjacent_xy_m=None
            if previous is None
            else math.hypot(p[0] - previous["position"][0], p[1] - previous["position"][1]),
            adjacent_yaw_rad=None if previous is None else wrap(yaw - previous["yaw_rad"]),
        )
        self.samples.append(sample)
        self.last_sample_stamp = stamp
        # 越界即锁存首个失败原因（"or" 保证只保留第一次），后续样本不覆盖它。
        if (
            sample["xy_from_anchor_m"] > XY_LIMIT_M
            or abs(sample["yaw_from_anchor_rad"]) > YAW_LIMIT_RAD
        ):
            self.motion_failure = self.motion_failure or dict(
                reason="parking_pose10hz_fixed_anchor_exceeded", sample=copy.deepcopy(sample)
            )
        return sample

    def observe(self, row):
        """喂入一行新位姿，推进所有已到期的栅格点并返回事件列表。

        参数：row 为原始 SLAM 位姿行。
        返回：事件列表，元素为 {"kind": "sample"}（采样成功）或
        {"kind": "missing"}（该栅格无可用位姿）。
        拒绝（raise）：源时钟倒退；同一 stamp 重复出现但位姿被改动。
        语义：重复且完全相同的 header 直接忽略（返回空列表）；缺失栅格不会被后续
        数据补回。
        """
        stamp, p, yaw = pose(row)
        if stamp < self.last_input:
            raise ValueError("10Hz SLAM source clock went backwards")
        if stamp == self.last_input:
            if (p, yaw) != self.last_pose:
                raise ValueError("Repeated SLAM header changed pose")
            return []
        self.last_input = stamp
        self.last_pose = (p, yaw)
        self.pending.append(copy.deepcopy(row))
        events = []
        while self.next_grid <= stamp and (self.end_ns is None or self.next_grid <= self.end_ns):
            grid = self.next_grid
            # 只取栅格时刻"或之前"的位姿（因果），并取其中最近的一行（无插值）。
            eligible = [r for r in self.pending if r["stamp_ns"] <= grid]
            selected = eligible[-1] if eligible else None
            # 三个拒绝条件：没有前置位姿、与上一采样同一 header（不重复计数）、
            # 源龄超过 300 ms。任一成立即记 missing，不降级到更旧的位姿。
            if (
                selected is None
                or selected["stamp_ns"] <= self.last_sample_stamp
                or grid - selected["stamp_ns"] > SOURCE_AGE_NS
            ):
                missing = dict(grid_ns=grid, reason="no_distinct_fresh_causal_pose")
                self.missing.append(missing)
                events.append(dict(kind="missing", data=missing))
            else:
                events.append(dict(kind="sample", data=self._sample(grid, selected)))
            self.next_grid += PERIOD_NS
            # 只保留最近一个前置位姿与尚未消耗的未来位姿，避免 pending 无界增长。
            self.pending = ([eligible[-1]] if eligible else []) + [
                r for r in self.pending if r["stamp_ns"] > grid
            ]
        return events

    def evidence(self):
        """汇总栅格证据字典（供上层判定，不自行判定停车成功）。

        complete：窗口已跑完（next_grid 越过 end_ns）。
        data_available：窗口完整、无 missing、且正好 51 个样本。
        passed：data_available 且没有漂移越界记录。
        max_* 字段单位：平面 m、偏航 rad；max_adjacent_* 为相邻样本增量。
        """
        complete = self.end_ns is not None and self.next_grid > self.end_ns
        # 5 s / 100 ms 固定为 51 个栅格点；数量不符即视为证据不完整。
        available = complete and not self.missing and len(self.samples) == 51
        return dict(
            schema=CONFIG["schema"],
            configuration=copy.deepcopy(CONFIG),
            start_ns=self.start_ns,
            end_ns=self.end_ns,
            origin=copy.deepcopy(self.origin),
            samples=copy.deepcopy(self.samples),
            missing=copy.deepcopy(self.missing),
            complete=complete,
            data_available=available,
            motion_failure=copy.deepcopy(self.motion_failure),
            passed=bool(available and self.motion_failure is None),
            max_xy_from_anchor_m=max(s["xy_from_anchor_m"] for s in self.samples),
            max_abs_yaw_from_anchor_rad=max(abs(s["yaw_from_anchor_rad"]) for s in self.samples),
            max_adjacent_xy_m=max((s["adjacent_xy_m"] or 0.0) for s in self.samples),
            max_abs_adjacent_yaw_rad=max(abs(s["adjacent_yaw_rad"] or 0.0) for s in self.samples),
            high_frequency_motion_used_for_decision=False,
        )


class CaptureGrid:
    """原始 25 mm / 0.035 rad 几何驻留捕获，只使用互不相同的 10 Hz 位姿。

    在栅格上检查给定的目标位置与航向；缺失栅格会重置驻留计时。
    """

    def __init__(self):
        self.grid = None
        # since：本次连续驻留的起始栅格时间（纳秒）；None 表示尚未开始。
        self.since = None
        self.close = False
        self.last = None

    def observe(self, row, goal, heading):
        """喂入一行位姿并更新驻留状态。

        参数：goal 为目标位置（m，世界坐标）；heading 为目标航向（rad）。
        返回：当前是否已满足驻留就绪（见 ready）。
        语义：missing 事件会把连续驻留清零；离开容差同样清零。
        """
        if self.grid is None:
            self.grid = PoseGrid(row["stamp_ns"], row, duration_ns=None)
            events = [dict(kind="sample", data=self.grid.samples[0])]
        else:
            events = self.grid.observe(row)
        for event in events:
            if event["kind"] == "missing":
                self.since = None
                self.close = False
                continue
            s = event["data"]
            p = s["position"]
            self.last = s
            # 捕获容差是硬编码的 25 mm / 0.035 rad，与停车契约里的 capture_xy_m /
            # capture_yaw_rad 同值；这里只做几何判定，不看瞬时速度或陀螺。
            self.close = (
                math.hypot(p[0] - goal[0], p[1] - goal[1]) <= 0.025
                and abs(wrap(heading - s["yaw_rad"])) <= 0.035
            )
            if not self.close:
                self.since = None
            elif self.since is None:
                self.since = s["grid_ns"]
        return self.ready

    @property
    def ready(self):
        """驻留是否成立：仍 close，且连续驻留已达 600 ms（硬编码 6e8 ns）。"""
        return (
            self.close
            and self.since is not None
            and self.last["grid_ns"] - self.since >= 600_000_000
        )

    def evidence(self):
        """返回足以覆盖未改动的 0.6 s 驻留的有限几何证书；未就绪时返回 None。

        证书字段单位：位置 m、偏航 rad、驻留纳秒；显式声明未使用瞬时速度或陀螺，
        因此它只证几何，不证物理静止。
        """
        if not self.ready:
            return None
        samples = [copy.deepcopy(s) for s in self.grid.samples if s["grid_ns"] >= self.since]
        # 只截取最近 7 个样本（600 ms 驻留 + 1 个边界点），避免证书无限增长。
        return dict(
            schema="parking_pose10hz_geometric_capture/v1",
            samples=samples[-7:],
            capture_xy_m=0.025,
            capture_yaw_rad=0.035,
            capture_dwell_ns=600_000_000,
            instantaneous_speed_or_gyro_used=False,
        )


def validate_capture(certificate, goal, heading, origin):
    """校验几何捕获证书与选定的 T0 位姿是否一致。

    要求：7 个真实栅格样本、栅格间隔正好 100 ms、源龄 <= 300 ms、每点位置/航向分别
    在 25 mm / 0.035 rad 内，且最后一个样本的来源位姿等于 T0 的 origin。
    返回 True；任一条件不符即 raise ValueError（不修正、不放宽）。
    """
    if (
        not isinstance(certificate, dict)
        or certificate.get("schema") != "parking_pose10hz_geometric_capture/v1"
    ):
        raise ValueError("No 10Hz geometric capture proof")
    samples = certificate.get("samples", [])
    if len(samples) != 7:
        raise ValueError("10Hz capture requires seven real geometric samples")
    previous_grid = previous_stamp = None
    for sample in samples:
        stamp, p, yaw = pose(sample["source"])
        grid = sample["grid_ns"]
        # 逐样本复核"来源未改、源龄新鲜、栅格等距、stamp 严格递增、几何仍在容差内"。
        if (
            sample["source_stamp_ns"] != stamp
            or not 0 <= grid - stamp <= SOURCE_AGE_NS
            or previous_grid is not None
            and grid - previous_grid != PERIOD_NS
            or previous_stamp is not None
            and stamp <= previous_stamp
            or math.hypot(p[0] - goal[0], p[1] - goal[1]) > 0.025
            or abs(wrap(heading - yaw)) > 0.035
        ):
            raise ValueError("10Hz geometric capture source/spacing/geometry differs")
        previous_grid, previous_stamp = grid, stamp
    # T0 必须是本次捕获选中的那个位姿，否则捕获与后续零速窗口对不上同一个起点。
    if samples[-1]["source"] != origin:
        raise ValueError("T0 origin differs from selected 10Hz capture pose")
    return True
