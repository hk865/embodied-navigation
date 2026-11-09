"""版本化的因果 1 s navigation-settle 统计证据；legacy 停止判据独立保留。

本模块只根据已接受的 SLAM 位姿与因果 body gyro 计算 1 s 窗口内的静止统计量，
供 transition gate 判断“是否已停稳”；它不产生停止命令、不改变控制输出，也不
替代 legacy 的 300 ms 停止判据。NavigationStopWindow 的 SPEC 数值必须与
safety/safety_configuration.settle() 的期望值完全相等（构造时逐值校验）。
"""

from collections import deque
import math

SPEC = dict(
    schema="navigation_settle_statistics/v1",
    enabled=True,
    # 统计窗口 1.0 s；窗口内按时间梯形积分计算 gyro 的均值/RMS/峰值。
    window_s=1.0,
    # body gyro 角速度范数界限，单位 rad/s。
    gyro_mean_norm_radps=0.005,
    gyro_rms_norm_radps=0.05,
    gyro_peak_norm_radps=0.10,
    # 窗口内欧拉角：峰峰值 0.003 rad，净变化 0.0015 rad。
    euler_peak_to_peak_rad=0.003,
    euler_net_change_rad=0.0015,
    # 窗口内平面位置：任两点最大间距 0.003 m，首末净位移 0.002 m。
    xy_diameter_m=0.003,
    xy_net_change_m=0.002,
    # 只有 settle 阶段、且 purpose 属于下列转移时，才允许用统计窗释放。
    eligible_purposes=["request_start", "region_transition", "path_rebind", "heading_change"],
    eligible_phase="settle",
    # legacy 停止判据（teacher_transition 的 300 ms 窗口）仍然保留，两者是 OR 关系。
    legacy_stop_predicate_retained=True,
    # 动态恢复、路径准入、定位保持等场景继续使用 legacy 判据。
    dynamic_recovery_path_admission_localization_hold_use_legacy=True,
    source="Actual accepted SLAM pose and causal body gyro; no native/truth pose gate",
    # 每个样本的零命令/速度/角速度/配对新鲜度门槛沿用原实现，不因统计窗放宽。
    original_per_sample_zero_speed_yaw_pair_gap_freshness_retained=True,
)


def quaternion_euler(q):
    """把四元数 (x, y, z, w) 转成 XYZ 欧拉角 [roll, pitch, yaw]（rad）。

    asin 的输入先夹到 [-1, 1]，避免浮点误差导致定义域错误。
    """
    x, y, z, w = q
    return [
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    ]


def bind_transition_context(node, purpose=None):
    """把实际恢复归属写入 heading gate，优先于 reset 的名义 purpose。

    purpose 为 None 时按上下文推断：动态恢复事件、倾斜恢复、身份变化（未知）、
    动态恢复后的 drive 等。gate 没有 set_transition_context 接口时直接返回。
    """
    gate = node.heading_gate
    if not hasattr(gate, "set_transition_context"):
        return
    token = getattr(node, "_teacher_recovery_token", None)
    # 动态恢复：障碍恢复待处理，或当前请求/航点的恢复 token 仍处于
    # await_new_SCAN / aligning 相位。
    dynamic = (
        bool(getattr(node, "obstacle_resume_pending", False))
        or isinstance(token, dict)
        and token.get("request_id") == getattr(node, "request_id", None)
        and token.get("waypoint_index") == getattr(node, "waypoint_index", None)
        and token.get("phase") in ("await_new_SCAN", "aligning")
    )
    context = dict(
        request_id=getattr(node, "request_id", None),
        waypoint_index=getattr(node, "waypoint_index", None),
        dynamic_recovery=dynamic,
        dynamic_event_sequence=token.get("sequence")
        if dynamic and isinstance(token, dict)
        else None,
    )
    if dynamic:
        purpose = "dynamic_recovery"
    elif getattr(node, "tilt_hold", False):
        purpose = "tilt_recovery"
    elif purpose is None:
        old = gate.transition_context
        if (
            old.get("request_id") != context["request_id"]
            or old.get("waypoint_index") != context["waypoint_index"]
        ):
            purpose = "unknown"  # 身份变了，不能沿用旧 purpose 推断运动资格。
        elif old.get("dynamic_recovery") and gate.phase == "drive":
            # 旧上下文是动态恢复、但 gate 已回到 drive：按普通路线行驶处理。
            purpose = "route_drive"
        else:
            purpose = gate.transition_purpose
    gate.set_transition_context(purpose, context)


def reset_transition(node, purpose):
    """先重置 heading gate，再把当前请求/航点上下文重新绑定上去。"""
    node.heading_gate.reset()
    bind_transition_context(node, purpose)


class NavigationStopWindow:
    """1 s 静止统计窗口：只观察已接受样本，供 transition gate 读取。

    config 为 None 时窗口禁用（enabled=False，observe/metrics 空转）；非 None 时
    先交给 safety_configuration.settle() 精确校验，非法配置 raise。legacy 为
    teacher_transition 配置，仅用于每个样本的有效性门槛。rows 最多保留 128 个
    样本，metrics() 结果缓存到下一次观察。
    """

    def __init__(self, config, legacy):
        if config is not None:
            from embodied_navigation.control.safety.safety_configuration import settle

            settle(config)
        self.enabled = config is not None
        self.legacy = legacy
        self.rows = deque(maxlen=128)
        self._cached = None

    def clear(self):
        """清空样本与缓存；窗口重新从头累积。"""
        self.rows.clear()
        self._cached = None

    def observe(self, stamp, wall, velocity, wz, gyro, gyro_stamp, zero, position, quaternion):
        """喂入一个实测 SLAM/gyro 样本；单位随参数语义变化。

        stamp 为位姿头时间（sim/header s），wall 为接收时间（wall s），velocity 为
        body 速度（m/s），wz 为 SLAM yaw 角速度（rad/s），gyro 为 body 角速度
        （rad/s），gyro_stamp 为 gyro 头时间（s），zero=true 表示确为零命令，
        position 为世界坐标（m），quaternion 为 (x, y, z, w)。

        任一样本不满足有效性门槛就清空历史；时间戳不前进则忽略；相邻样本间隔
        超过 stop_pose_gap_max_s 也清空。窗口只保留左边界处的一个因果前驱样本。
        """
        if not self.enabled:
            return
        values = (
            [stamp, wall, wz, gyro_stamp]
            + list(velocity)
            + list(gyro)
            + list(position or [])
            + list(quaternion or [])
        )
        # 每样本有效性沿用 legacy teacher_transition 门槛：确为零命令、平面速度
        # 不超过 0.03 m/s、SLAM yaw 角速度不超过 0.05 rad/s、pose/gyro 配对不超过
        # 0.15 s（当前 configs/control/settings.json 的值）。
        valid = (
            len(position or []) == 3
            and len(quaternion or []) == 4
            and all(math.isfinite(v) for v in values)
            and zero is True
            and math.hypot(*velocity[:2]) <= self.legacy["stop_planar_speed_mps"]
            and abs(wz) <= self.legacy["stop_slam_yaw_rate_radps"]
            and abs(stamp - gyro_stamp) <= self.legacy["gyro_pose_pair_max_s"]
        )
        if not valid:
            self.clear()
            return
        if self.rows and stamp <= self.rows[-1]["t"]:
            return
        if self.rows and stamp - self.rows[-1]["t"] > self.legacy["stop_pose_gap_max_s"]:
            self.clear()
        self.rows.append(
            dict(t=stamp, wall=wall, g=list(gyro), p=list(position), a=quaternion_euler(quaternion))
        )
        # 只保留一个位于/早于窗口左边界的前驱样本，用于插值出窗口左端点。
        left = stamp - SPEC["window_s"]
        while len(self.rows) > 2 and self.rows[1]["t"] <= left:
            self.rows.popleft()
        self._cached = None

    def metrics(self):
        """返回窗口统计结果（dict）；证据不足 1 s 时 complete=False。

        完整窗口要求至少 3 个样本且实际跨度 >= 1.0 s（1e-9 是浮点边界松弛）。
        gyro 均值/RMS 用梯形积分，峰值包含左边界前驱（保守）；欧拉角先逐轴
        unwrap 再取峰峰值与净变化。7 个条件全部为 <= 时 passed=True。结果缓存在
        _cached，直到下一次 observe。
        """
        if self._cached is not None:
            return self._cached
        out = dict(
            schema=SPEC["schema"],
            complete=False,
            passed=False,
            samples=len(self.rows),
            source="SLAM_and_body_gyro",
        )
        if len(self.rows) < 3 or self.rows[-1]["t"] - self.rows[0]["t"] < 1.0 - 1e-9:
            return out
        rows = list(self.rows)
        left = rows[-1]["t"] - 1.0
        if rows[0]["t"] > left + 1e-9:
            return out
        # 逐轴展开欧拉角，避免 ±pi 跳变污染峰峰值与净变化。
        unwrapped = []
        for i, r in enumerate(rows):
            a = list(r["a"])
            if i:
                a = [
                    unwrapped[-1][j]
                    + math.atan2(
                        math.sin(a[j] - rows[i - 1]["a"][j]), math.cos(a[j] - rows[i - 1]["a"][j])
                    )
                    for j in range(3)
                ]
            unwrapped.append(a)
        # 在左边界处对前驱与首个窗内样本线性插值，得到精确 1 s 起点。
        first, second = rows[0], rows[1]
        alpha = (left - first["t"]) / (second["t"] - first["t"])
        alpha = max(0.0, min(1.0, alpha))
        window = [
            dict(
                t=left,
                g=[first["g"][j] + alpha * (second["g"][j] - first["g"][j]) for j in range(3)],
                p=[first["p"][j] + alpha * (second["p"][j] - first["p"][j]) for j in range(3)],
                a=[unwrapped[0][j] + alpha * (unwrapped[1][j] - unwrapped[0][j]) for j in range(3)],
            )
        ]
        window.extend(
            dict(t=r["t"], g=r["g"], p=r["p"], a=unwrapped[i])
            for i, r in enumerate(rows)
            if r["t"] > left
        )
        # 梯形积分求 gyro 均值与平方均值，再分别取范数。
        mean = [0.0, 0.0, 0.0]
        squares = [0.0, 0.0, 0.0]
        for prev, row in zip(window, window[1:]):
            dt = row["t"] - prev["t"]
            for j in range(3):
                mean[j] += 0.5 * dt * (prev["g"][j] + row["g"][j])
                squares[j] += 0.5 * dt * (prev["g"][j] ** 2 + row["g"][j] ** 2)
        norm = lambda v: math.sqrt(sum(x * x for x in v))
        peak = max(norm(r["g"]) for r in rows)  # 保守起见把左边界前驱也算进峰值。
        pp = [max(r["a"][j] for r in window) - min(r["a"][j] for r in window) for j in range(3)]
        net = [abs(window[-1]["a"][j] - window[0]["a"][j]) for j in range(3)]
        diameter = max(
            math.hypot(a["p"][0] - b["p"][0], a["p"][1] - b["p"][1]) for a in window for b in window
        )
        xy_net = math.hypot(
            window[-1]["p"][0] - window[0]["p"][0], window[-1]["p"][1] - window[0]["p"][1]
        )
        conditions = dict(
            gyro_mean=norm(mean) <= SPEC["gyro_mean_norm_radps"],
            gyro_rms=math.sqrt(sum(squares)) <= SPEC["gyro_rms_norm_radps"],
            gyro_peak=peak <= SPEC["gyro_peak_norm_radps"],
            angles_peak_to_peak=max(pp) <= SPEC["euler_peak_to_peak_rad"],
            angles_net=max(net) <= SPEC["euler_net_change_rad"],
            xy_diameter=diameter <= SPEC["xy_diameter_m"],
            xy_net=xy_net <= SPEC["xy_net_change_m"],
        )
        out.update(
            complete=True,
            passed=all(conditions.values()),
            window_first_s=left,
            window_last_s=rows[-1]["t"],
            original_predecessor_s=rows[0]["t"],
            actual_source_span_s=rows[-1]["t"] - rows[0]["t"],
            signed_gyro_mean_radps=mean,
            gyro_mean_norm_radps=norm(mean),
            gyro_rms_norm_radps=math.sqrt(sum(squares)),
            gyro_peak_norm_radps=peak,
            euler_peak_to_peak_rad=pp,
            euler_net_change_rad=net,
            xy_diameter_m=diameter,
            xy_net_change_m=xy_net,
            conditions=conditions,
            legacy_threshold_result_is_not_relabelled=True,
        )
        self._cached = out
        return out

    def evidence(self, sim_time, wall_time):
        """在 metrics() 结果上附加新鲜度，返回可直接写入证据的 dict。

        fresh 要求最后一帧 wall 时间在 0.3 s 内、sim 时间允许比最后一帧早最多
        0.05 s 或晚不到 0.3 s；只有 fresh 时 passed 才可能为 True。enabled 表示
        窗口是否启用。
        """
        out = dict(self.metrics())
        fresh = bool(
            self.rows
            and 0 <= wall_time - self.rows[-1]["wall"] < 0.3
            and -0.05 <= sim_time - self.rows[-1]["t"] < 0.3
        )
        out.update(fresh=fresh, passed=bool(out["passed"] and fresh), enabled=self.enabled)
        return out
