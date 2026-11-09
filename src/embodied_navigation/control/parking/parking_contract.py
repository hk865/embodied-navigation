"""显式批准的 R80“首个合格零 ACK”停车契约常量；不做 IO。

本模块只保存停车窗口的契约常量，并由 enabled(profile) 交给
safety/safety_configuration.parking() 做精确等值校验：任一被校验的数值、窗口锚点
或 10 Hz 采样策略与已批准值不同都会 raise。它不判定停车成功/失败，不产生零命令，
也不读写运行状态。LEGACY_SPEC 是保留的 v1 文档形状（旧的零 ACK 固定 5 s 模式），
运行时已不再选择它，只有 SPEC 是现役契约。
"""

import copy
import math
import numpy as np
from embodied_navigation.control.parking.parking_zero_contract import validate_zero_ack, integer

# 保留的 v1 旧契约（原始零 ACK 固定 5 s 模式）：现役运行时不再选择它（见下方
# MODE/SPEC），保留只为让旧文档形状仍可被检查。
LEGACY_MODE = "ordinary_zero_ack_fixed5"
LEGACY_SPEC = dict(
    schema="ordinary_terminal_zero_ack_fixed5/v1",
    enabled=True,
    T0="first_core_precise_pose_and_actual_zero_input_ACK_qualification_clock",
    duration_ns=5_000_000_000,
    reset_window=False,
    keep_legacy_arrival_window=True,
    capture_timeout="original_active_goal_timeout_s_from_separate_capture_begin",
    capture_xy_m=0.025,
    capture_yaw_rad=0.035,
    capture_speed_mps=0.03,
    capture_yawdot_radps=0.06,
    capture_dwell_ns=600_000_000,
    xy_drift_m=0.05,
    yaw_drift_rad=0.1,
    native_speed_mps=0.08,
    native_wz_radps=0.1,
    native_yawdot_radps=0.1,
    source_age_ns=300_000_000,
)
MODE = "ordinary_pose10hz_zero_ack_fixed5"
# 现役 SPEC 使用的 10 Hz 捕获配置（100 ms 栅格、5 s 窗口、0.05 m / 0.1 rad 漂移）。
# 下列 deepcopy 让 SPEC 与模块级 POSE_CONFIG 不共享嵌套对象。
from embodied_navigation.control.parking.parking_pose10hz import CONFIG as POSE_CONFIG

SPEC = dict(
    schema="ordinary_pose10hz_zero_ack_fixed5/v1",
    enabled=True,
    # T0 指“首个 10 Hz 几何捕获 + 真实零输入 ACK 的资格时钟”；这是标签字符串，不是时间值。
    T0="first_10hz_geometric_capture_and_actual_zero_ACK_qualification_clock",
    # 零保持固定窗口 5 s（纳秒）；reset_window=False 表示窗口锚点固定、不得重置。
    duration_ns=5_000_000_000,
    reset_window=False,
    keep_legacy_arrival_window=True,
    # 捕获超时标签：沿用原始 active goal 超时，从独立的 capture_begin 起算。
    capture_timeout="original_active_goal_timeout_s_from_separate_capture_begin",
    # 捕获判据：平面 0.025 m、航向 0.035 rad；驻留 0.6 s（纳秒）。
    capture_xy_m=0.025,
    capture_yaw_rad=0.035,
    capture_dwell_ns=600_000_000,
    # 10 Hz 位姿栅格，以及来源年龄上限 300 ms（纳秒）；均由 safety.pose() 精确校验。
    pose10hz=copy.deepcopy(POSE_CONFIG),
    source_age_ns=300_000_000,
)


def enabled(profile):
    """返回 profile 是否启用现役普通停车契约。

    参数 profile 为运行 profile（configs base → 启动器合并 → 节点）。
    键 ordinary_terminal_parking_contract 缺失时返回 False；存在时交给
    safety_configuration.parking() 逐值精确校验，非法配置会 raise（因此成功
    路径只会返回 True）。无副作用、无 IO。
    """
    value = profile.get("ordinary_terminal_parking_contract")
    if value is None:
        return False
    from embodied_navigation.control.safety.safety_configuration import parking

    return parking(value)
