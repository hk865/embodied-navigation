"""当前数值安全设置的“必须等于”校验器；不涉及放行、哈希或审批文档。

职责：接收调用方传入的 dict，逐值断言等于已审查的固定常量，并检查策略字符串。
边界：自己**不读** `configs/`、不写盘、不修改入参；值来自
`../parking/parking_contract.py`、`../parking/parking_pose10hz.py` 与
`../parking/navigation_stop_window.py` 传入的 profile 片段。所有常量都是
本文件硬编码的 reviewed 值，不是 launch 参数或运行时覆盖。
据模块 README，`clock()` 目前在全仓没有调用点，是一个未接线的校验器。
"""

import math


def _number(value, expected, name):
    """要求一个有限数值，且精确等于已审查的常量。

    `type(value) not in (int, float)` 会同时排除 `bool`；`value != expected`
    是精确相等（不做容差/四舍五入），任何偏离都按非法配置处理。
    失败时抛出带键名的 `ValueError`，便于定位是哪个设置被改过。
    """
    if type(value) not in (int, float) or not math.isfinite(value) or value != expected:
        raise ValueError("Invalid safety setting: " + name)


def parking(value):
    """校验固定窗停车数值、固定锚点与 10 Hz 姿态采样设置。

    要求 `enabled is True`；`duration_ns=5e9`（5 s）、捕获容差
    `capture_xy_m=0.025`/`capture_yaw_rad=0.035`、驻留 600 ms、源龄上限 300 ms。
    `reset_window` 必须为 `False`（锚点保持固定）。通过后再校验 `pose10hz`。
    返回 True；任一不符抛 `ValueError`。单位：ns / m / rad。
    """
    if not isinstance(value, dict) or value.get("enabled") is not True:
        raise ValueError("Fixed-window parking must be enabled")
    for key, number in dict(
        duration_ns=5_000_000_000,
        capture_xy_m=0.025,
        capture_yaw_rad=0.035,
        capture_dwell_ns=600_000_000,
        source_age_ns=300_000_000,
    ).items():
        _number(value.get(key), number, key)
    if value.get("reset_window") is not False:
        raise ValueError("Parking anchor must remain fixed")
    pose(value.get("pose10hz"))
    return True


def pose(value):
    """校验因果 10 Hz 姿态采样的周期/漂移与采样策略。

    数值常量：`period_ns=1e8`（100 ms）、`duration_ns=5e9`（5 s）、
    `xy_drift_m=0.05`、`yaw_drift_rad=0.1`、`source_age_ns=3e8`（300 ms）。
    策略字符串必须精确等于 `latest_distinct_header_at_or_before_grid`，且
    插值、高频运动门、相邻增量门都必须为 `False`。返回 True；失败抛 `ValueError`。
    """
    if not isinstance(value, dict):
        raise ValueError("Pose sampling settings required")
    for key, number in dict(
        period_ns=100_000_000,
        duration_ns=5_000_000_000,
        xy_drift_m=0.05,
        yaw_drift_rad=0.1,
        source_age_ns=300_000_000,
    ).items():
        _number(value.get(key), number, key)
    if (
        value.get("sampling") != "latest_distinct_header_at_or_before_grid"
        or value.get("interpolation") is not False
        or value.get("high_frequency_motion_gate") is not False
        or value.get("adjacent_increment_gate") is not False
    ):
        raise ValueError("Actual causal pose sampling required")
    return True


def clock(value):
    """校验时钟新鲜度/兜底数值与时钟保持策略。

    数值常量：`freshness_sim_and_wall_s=0.3`（300 ms）、
    `continuous_stale_fail_wall_s=8.0`（连续失联 8 s 后失败）。策略字符串必须为
    `exact_zero_before_math_and_native_geometry` 与 `latch_failure`，
    且 `source_receipt_refresh_permitted` 必须为 `False`。
    注意：据模块 README，本函数目前无调用点，期望的 `clock_protection` 键在
    `configs/control/settings.json` 中也不存在。
    """
    if not isinstance(value, dict):
        raise ValueError("Clock protection settings required")
    _number(value.get("freshness_sim_and_wall_s"), 0.3, "clock source freshness")
    _number(value.get("continuous_stale_fail_wall_s"), 8.0, "continuous stale stop")
    if (
        value.get("same_clock_policy") != "exact_zero_before_math_and_native_geometry"
        or value.get("backward_clock_policy") != "latch_failure"
        or value.get("source_receipt_refresh_permitted") is not False
    ):
        raise ValueError("Clock hold protection required")


def settle(value):
    """校验 settle 窗数值，以及 settle 相位与“实测停止”策略。

    要求 `enabled is True`；`window_s=1.0`（1 s 窗口），陀螺均值/均方根/峰值
    上限分别为 0.005 / 0.05 / 0.1 rad/s，欧拉角峰峰值 0.003 rad、净变化 0.0015 rad，
    XY 直径 0.003 m、净变化 0.002 m（单位随键名）。`eligible_phase` 必须为
    `settle`，`legacy_stop_predicate_retained` 必须为 `True`。
    返回 True；失败抛 `ValueError`。
    """
    if not isinstance(value, dict) or value.get("enabled") is not True:
        raise ValueError("Settle window required")
    for key, number in dict(
        window_s=1.0,
        gyro_mean_norm_radps=0.005,
        gyro_rms_norm_radps=0.05,
        gyro_peak_norm_radps=0.1,
        euler_peak_to_peak_rad=0.003,
        euler_net_change_rad=0.0015,
        xy_diameter_m=0.003,
        xy_net_change_m=0.002,
    ).items():
        _number(value.get(key), number, key)
    if (
        value.get("eligible_phase") != "settle"
        or value.get("legacy_stop_predicate_retained") is not True
    ):
        raise ValueError("Settle phase and measured stop protection required")
    return True
