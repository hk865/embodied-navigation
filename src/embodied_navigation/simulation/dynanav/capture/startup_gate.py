"""仿真开机主动热身迈步与停稳趋势门禁模块。

本模块为 DynaNav 仿真采集提供可配置的开机脱离冲击与停稳判据：
1. ``evaluate_trend_stability`` 基于最近窗口内的位置、姿态（对 yaw 做
   ``np.unwrap`` 防止 $\\pm\\pi$ 跳变）、足端支撑接触和关节角度/速度，利用
   最小二乘一次拟合斜率、去趋势残差标准差、前后半窗均值差与死区保护后的增量方向
   一致性指数 $\\eta$，拦截缓慢搓地漂移和高频剧烈晃动；
2. ``StartupWarmupController`` 实现 ``LAND_SETTLE -> WARMUP_WALK -> STOP_CONVERGE -> READY_FOR_SLAM``
   四阶段状态机，停稳前暂缓放行外部航点，并按配置决定是否暂缓向 SLAM 发布传感器流。

默认 ``enabled=False``，未启用时不改变任何既有采集逻辑；仅依赖纯 ``numpy`` 与标准库。
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass, field
import math
from typing import Any

import numpy as np

# 状态机四阶段常量定义
PHASE_LAND_SETTLE = "LAND_SETTLE"
PHASE_WARMUP_WALK = "WARMUP_WALK"
PHASE_STOP_CONVERGE = "STOP_CONVERGE"
PHASE_READY_FOR_SLAM = "READY_FOR_SLAM"


def _require_bool(value: Any, name: str) -> bool:
    """严格校验布尔配置项，拒绝整数或字符串隐式转换。"""
    if type(value) is not bool:
        raise ValueError(f"{name} 必须为布尔值，收到 {value!r}")
    return value


def _require_int(value: Any, name: str, *, min_value: int | None = None) -> int:
    """严格校验整数配置项，拒绝布尔值与浮点数。"""
    if type(value) is not int:
        raise ValueError(f"{name} 必须为整数，收到 {value!r}")
    if min_value is not None and value < min_value:
        raise ValueError(f"{name} 必须 >= {min_value}，收到 {value}")
    return value


def _require_float(
    value: Any,
    name: str,
    *,
    min_value: float | None = None,
    max_value: float | None = None,
    strict_min: bool = False,
) -> float:
    """严格校验有限浮点数配置项，拒绝布尔值与 NaN/Inf。"""
    if type(value) not in (int, float):
        raise ValueError(f"{name} 必须为有限数值，收到 {value!r}")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} 必须为有限数值，收到 {value!r}")
    if min_value is not None:
        if strict_min and not (number > min_value):
            raise ValueError(f"{name} 必须 > {min_value}，收到 {number}")
        if not strict_min and number < min_value:
            raise ValueError(f"{name} 必须 >= {min_value}，收到 {number}")
    if max_value is not None and number > max_value:
        raise ValueError(f"{name} 必须 <= {max_value}，收到 {number}")
    return number


def _reject_unknown_keys(data: Mapping[str, Any], allowed: set[str], label: str) -> None:
    """拒绝未知配置键，防止拼写错误静默回退到默认值。"""
    unknown = sorted(set(data.keys()) - allowed)
    if unknown:
        raise ValueError(f"{label} 包含未知配置键: {', '.join(unknown)}")


@dataclass(frozen=True)
class TrendStabilityConfig:
    """停稳趋势与白噪声门禁阈值配置（默认对应 200 Hz 下 1.0 s 窗口）。"""

    window_samples: int = 200
    max_pos_slope_mps: float = 0.005
    max_att_slope_radps: float = 0.003
    max_pos_resid_std_m: float = 0.003
    max_att_resid_std_rad: float = 0.003
    max_half_window_pos_diff_m: float = 0.005
    max_half_window_att_diff_rad: float = 0.004
    max_direction_consistency: float = 0.55
    eta_pos_deadband_m: float = 0.002
    eta_att_deadband_rad: float = 0.0015
    min_support_fraction: float = 0.98
    max_joint_speed_radps: float = 0.15
    max_joint_slope_radps: float = 0.03

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "window_samples",
            _require_int(self.window_samples, "window_samples", min_value=4),
        )
        object.__setattr__(
            self,
            "max_pos_slope_mps",
            _require_float(self.max_pos_slope_mps, "max_pos_slope_mps", min_value=0.0, strict_min=True),
        )
        object.__setattr__(
            self,
            "max_att_slope_radps",
            _require_float(self.max_att_slope_radps, "max_att_slope_radps", min_value=0.0, strict_min=True),
        )
        object.__setattr__(
            self,
            "max_pos_resid_std_m",
            _require_float(self.max_pos_resid_std_m, "max_pos_resid_std_m", min_value=0.0, strict_min=True),
        )
        object.__setattr__(
            self,
            "max_att_resid_std_rad",
            _require_float(self.max_att_resid_std_rad, "max_att_resid_std_rad", min_value=0.0, strict_min=True),
        )
        object.__setattr__(
            self,
            "max_half_window_pos_diff_m",
            _require_float(
                self.max_half_window_pos_diff_m, "max_half_window_pos_diff_m", min_value=0.0, strict_min=True
            ),
        )
        object.__setattr__(
            self,
            "max_half_window_att_diff_rad",
            _require_float(
                self.max_half_window_att_diff_rad, "max_half_window_att_diff_rad", min_value=0.0, strict_min=True
            ),
        )
        object.__setattr__(
            self,
            "max_direction_consistency",
            _require_float(
                self.max_direction_consistency,
                "max_direction_consistency",
                min_value=0.0,
                max_value=1.0,
                strict_min=True,
            ),
        )
        object.__setattr__(
            self,
            "eta_pos_deadband_m",
            _require_float(self.eta_pos_deadband_m, "eta_pos_deadband_m", min_value=0.0),
        )
        object.__setattr__(
            self,
            "eta_att_deadband_rad",
            _require_float(self.eta_att_deadband_rad, "eta_att_deadband_rad", min_value=0.0),
        )
        object.__setattr__(
            self,
            "min_support_fraction",
            _require_float(
                self.min_support_fraction,
                "min_support_fraction",
                min_value=0.0,
                max_value=1.0,
            ),
        )
        object.__setattr__(
            self,
            "max_joint_speed_radps",
            _require_float(
                self.max_joint_speed_radps, "max_joint_speed_radps", min_value=0.0, strict_min=True
            ),
        )
        object.__setattr__(
            self,
            "max_joint_slope_radps",
            _require_float(
                self.max_joint_slope_radps, "max_joint_slope_radps", min_value=0.0, strict_min=True
            ),
        )

    @classmethod
    def from_dict(cls, data: Mapping[str, Any] | None) -> "TrendStabilityConfig":
        """从配置字典严格构建 ``TrendStabilityConfig``。"""
        if data is None:
            return cls()
        if not isinstance(data, Mapping):
            raise ValueError("stability 配置必须为字典映射")
        allowed = {
            "window_samples",
            "max_pos_slope_mps",
            "max_att_slope_radps",
            "max_pos_resid_std_m",
            "max_att_resid_std_rad",
            "max_half_window_pos_diff_m",
            "max_half_window_att_diff_rad",
            "max_direction_consistency",
            "eta_pos_deadband_m",
            "eta_att_deadband_rad",
            "min_support_fraction",
            "max_joint_speed_radps",
            "max_joint_slope_radps",
        }
        _reject_unknown_keys(data, allowed, "TrendStabilityConfig")
        return cls(**dict(data))


@dataclass(frozen=True)
class TrendStabilityResult:
    """停稳门禁评估结果，所有字段均可直接 JSON 序列化。"""

    settled: bool
    failed_checks: tuple[str, ...]
    metrics: dict[str, float]

    def to_dict(self) -> dict[str, Any]:
        """转换为纯 Python 原生类型字典，便于写入 JSON/JSONL。"""
        return {
            "settled": bool(self.settled),
            "failed_checks": list(self.failed_checks),
            "metrics": {str(k): float(v) for k, v in self.metrics.items()},
        }


def _quat_wxyz_to_euler_zyx(quats_wxyz: np.ndarray) -> np.ndarray:
    """将 (N, 4) wxyz 四元数批量转为连续展开后的 ZYX 欧拉角 (roll, pitch, yaw)。"""
    norms = np.linalg.norm(quats_wxyz, axis=1, keepdims=True)
    if not np.isfinite(norms).all() or np.any(norms < 1e-9):
        raise ValueError("四元数模长无效或含非有限数值")
    q = quats_wxyz / norms
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]

    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    # 对姿态角沿时间轴解卷绕，避免在 ±pi 边界附近出现 2*pi 人为跳变
    euler = np.column_stack(
        [
            np.unwrap(roll),
            np.unwrap(pitch),
            np.unwrap(yaw),
        ]
    )
    return euler


def _ols_slope_and_resid_std(t_rel: np.ndarray, values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """计算二维数组 values (N, K) 每一列对居中时间 t_rel 的最小二乘斜率与去趋势残差标准差。"""
    denom = float(np.dot(t_rel, t_rel))
    if not math.isfinite(denom) or denom <= 0.0:
        raise ValueError("时间窗口方差为零或非有限")
    mean_y = np.mean(values, axis=0)
    centered_y = values - mean_y
    slope = (t_rel @ centered_y) / denom
    residuals = centered_y - np.outer(t_rel, slope)
    resid_std = np.std(residuals, axis=0, ddof=0)
    return slope, resid_std


def _direction_consistency_max(values: np.ndarray, deadband: float) -> tuple[float, float]:
    """计算方向一致性指数 η = |ΣΔy| / (Σ|Δy| + 1e-12)。

    返回 ``(gated_max, raw_max)``：只有当单轴净变化 ``|y[-1] - y[0]| > deadband`` 时
    才计入 ``gated_max``，防止零数值微量化抖动导致误报。
    """
    diffs = np.diff(values, axis=0)
    net_change = np.abs(np.sum(diffs, axis=0))
    total_path = np.sum(np.abs(diffs), axis=0)
    eta_raw = net_change / (total_path + 1e-12)
    active = net_change > deadband
    eta_gated = np.where(active, eta_raw, 0.0)
    return float(np.max(eta_gated)), float(np.max(eta_raw))


def evaluate_trend_stability(
    rows: Sequence[Mapping[str, Any]],
    config: TrendStabilityConfig | None = None,
) -> TrendStabilityResult:
    """评估最近采样窗口内机器人是否真正停稳（趋势 + 白噪声 + 支撑 + 关节门禁）。

    输入 ``rows`` 兼容 ``capture_go2_rgbd.py`` 采集中追加的每步字典结构，每条记录需包含：
    ``time_s``、``root_pose_wxyz=[pos, quat_wxyz]``、``contacts``、``q``、``qd``。
    """
    cfg = config if config is not None else TrendStabilityConfig()
    sample_count = len(rows)
    if sample_count < cfg.window_samples:
        return TrendStabilityResult(
            settled=False,
            failed_checks=("insufficient_samples",),
            metrics={
                "sample_count": float(sample_count),
                "required_samples": float(cfg.window_samples),
            },
        )

    window = rows[-cfg.window_samples :]
    try:
        t = np.asarray([float(r["time_s"]) for r in window], dtype=float)
        pos = np.asarray([r["root_pose_wxyz"][0] for r in window], dtype=float)
        quat = np.asarray([r["root_pose_wxyz"][1] for r in window], dtype=float)
        contacts = np.asarray([r["contacts"] for r in window], dtype=float)
        q = np.asarray([r["q"] for r in window], dtype=float)
        qd = np.asarray([r["qd"] for r in window], dtype=float)
    except (KeyError, IndexError, TypeError, ValueError):
        return TrendStabilityResult(
            settled=False,
            failed_checks=("malformed_rows",),
            metrics={"sample_count": float(sample_count)},
        )

    if (
        pos.ndim != 2
        or pos.shape[1] != 3
        or quat.ndim != 2
        or quat.shape[1] != 4
        or contacts.ndim != 2
        or contacts.shape[1] < 2
        or q.ndim != 2
        or q.shape[1] < 1
        or qd.shape != q.shape
    ):
        return TrendStabilityResult(
            settled=False,
            failed_checks=("malformed_rows",),
            metrics={"sample_count": float(sample_count)},
        )

    if not (
        np.isfinite(t).all()
        and np.isfinite(pos).all()
        and np.isfinite(quat).all()
        and np.isfinite(contacts).all()
        and np.isfinite(q).all()
        and np.isfinite(qd).all()
    ):
        return TrendStabilityResult(
            settled=False,
            failed_checks=("nonfinite_state",),
            metrics={"sample_count": float(sample_count)},
        )

    dt = np.diff(t)
    if np.any(dt <= 0.0) or (t[-1] - t[0]) <= 0.0:
        return TrendStabilityResult(
            settled=False,
            failed_checks=("invalid_timestamps",),
            metrics={"sample_count": float(sample_count)},
        )

    try:
        euler = _quat_wxyz_to_euler_zyx(quat)
    except ValueError:
        return TrendStabilityResult(
            settled=False,
            failed_checks=("nonfinite_state",),
            metrics={"sample_count": float(sample_count)},
        )

    t_rel = t - float(np.mean(t))
    pos_slope, pos_resid_std = _ols_slope_and_resid_std(t_rel, pos)
    att_slope, att_resid_std = _ols_slope_and_resid_std(t_rel, euler)
    joint_slope, _ = _ols_slope_and_resid_std(t_rel, q)

    mid = len(window) // 2
    half_pos_diff = float(np.linalg.norm(np.mean(pos[mid:], axis=0) - np.mean(pos[:mid], axis=0)))
    half_att_diff = float(np.max(np.abs(np.mean(euler[mid:], axis=0) - np.mean(euler[:mid], axis=0))))

    pos_eta_gated, pos_eta_raw = _direction_consistency_max(pos, cfg.eta_pos_deadband_m)
    att_eta_gated, att_eta_raw = _direction_consistency_max(euler, cfg.eta_att_deadband_rad)

    body_contact_max = float(np.max(contacts[:, 0]))
    support_fraction = float(np.mean(np.sum(contacts[:, 1:] > 0, axis=1) >= 3))
    joint_speed_p95 = float(np.percentile(np.abs(qd), 95))

    pos_slope_max = float(np.max(np.abs(pos_slope)))
    att_slope_max = float(np.max(np.abs(att_slope)))
    pos_resid_std_max = float(np.max(pos_resid_std))
    att_resid_std_max = float(np.max(att_resid_std))
    joint_slope_max = float(np.max(np.abs(joint_slope)))

    failed: list[str] = []
    if pos_slope_max > cfg.max_pos_slope_mps:
        failed.append("pos_slope")
    if att_slope_max > cfg.max_att_slope_radps:
        failed.append("att_slope")
    if pos_resid_std_max > cfg.max_pos_resid_std_m:
        failed.append("pos_resid_std")
    if att_resid_std_max > cfg.max_att_resid_std_rad:
        failed.append("att_resid_std")
    if half_pos_diff > cfg.max_half_window_pos_diff_m:
        failed.append("half_window_pos_diff")
    if half_att_diff > cfg.max_half_window_att_diff_rad:
        failed.append("half_window_att_diff")
    if pos_eta_gated > cfg.max_direction_consistency:
        failed.append("pos_direction_consistency")
    if att_eta_gated > cfg.max_direction_consistency:
        failed.append("att_direction_consistency")
    if body_contact_max > 0.0:
        failed.append("body_contact")
    if support_fraction < cfg.min_support_fraction:
        failed.append("support_fraction")
    if joint_speed_p95 > cfg.max_joint_speed_radps:
        failed.append("joint_speed_p95")
    if joint_slope_max > cfg.max_joint_slope_radps:
        failed.append("joint_slope")

    metrics: dict[str, float] = {
        "sample_count": float(sample_count),
        "window_samples": float(cfg.window_samples),
        "window_span_s": float(t[-1] - t[0]),
        "pos_slope_max_mps": pos_slope_max,
        "att_slope_max_radps": att_slope_max,
        "pos_resid_std_max_m": pos_resid_std_max,
        "att_resid_std_max_rad": att_resid_std_max,
        "half_window_pos_diff_m": half_pos_diff,
        "half_window_att_diff_rad": half_att_diff,
        "pos_direction_consistency": pos_eta_gated,
        "pos_direction_consistency_raw": pos_eta_raw,
        "att_direction_consistency": att_eta_gated,
        "att_direction_consistency_raw": att_eta_raw,
        "body_contact_max": body_contact_max,
        "support_fraction": support_fraction,
        "joint_speed_p95_radps": joint_speed_p95,
        "joint_slope_max_radps": joint_slope_max,
    }
    return TrendStabilityResult(
        settled=(len(failed) == 0),
        failed_checks=tuple(failed),
        metrics=metrics,
    )


@dataclass(frozen=True)
class StartupWarmupConfig:
    """开机热身迈步与停稳门禁配置（默认关闭，未启用时零侵入基线）。"""

    enabled: bool = False
    settle_s: float = 0.5
    walk_vx_mps: float = 0.35
    walk_duration_s: float = 1.5
    max_stop_wait_s: float = 12.0
    gate_slam_sensors_until_settled: bool = True
    stability: TrendStabilityConfig = field(default_factory=TrendStabilityConfig)

    def __post_init__(self) -> None:
        object.__setattr__(self, "enabled", _require_bool(self.enabled, "enabled"))
        object.__setattr__(
            self,
            "settle_s",
            _require_float(self.settle_s, "settle_s", min_value=0.0),
        )
        object.__setattr__(
            self,
            "walk_vx_mps",
            _require_float(self.walk_vx_mps, "walk_vx_mps", min_value=0.0, max_value=1.0),
        )
        object.__setattr__(
            self,
            "walk_duration_s",
            _require_float(self.walk_duration_s, "walk_duration_s", min_value=0.0),
        )
        object.__setattr__(
            self,
            "max_stop_wait_s",
            _require_float(self.max_stop_wait_s, "max_stop_wait_s", min_value=0.0, strict_min=True),
        )
        object.__setattr__(
            self,
            "gate_slam_sensors_until_settled",
            _require_bool(self.gate_slam_sensors_until_settled, "gate_slam_sensors_until_settled"),
        )
        if not isinstance(self.stability, TrendStabilityConfig):
            raise ValueError("stability 必须为 TrendStabilityConfig 实例")

    @classmethod
    def from_dict(cls, data: Mapping[str, Any] | None) -> "StartupWarmupConfig":
        """从实验配置字典严格构建 ``StartupWarmupConfig``。"""
        if data is None:
            return cls()
        if not isinstance(data, Mapping):
            raise ValueError("startup_warmup 配置必须为字典映射")
        allowed = {
            "enabled",
            "settle_s",
            "walk_vx_mps",
            "walk_duration_s",
            "max_stop_wait_s",
            "gate_slam_sensors_until_settled",
            "stability",
        }
        _reject_unknown_keys(data, allowed, "StartupWarmupConfig")
        kwargs = dict(data)
        if "stability" in kwargs:
            kwargs["stability"] = TrendStabilityConfig.from_dict(kwargs["stability"])
        return cls(**kwargs)


class StartupWarmupController:
    """管理开机热身迈步与停稳门禁的四阶段状态机。

    阶段转移顺序固定为：
    ``LAND_SETTLE -> WARMUP_WALK -> STOP_CONVERGE -> READY_FOR_SLAM``。
    若在 ``max_stop_wait_s`` 内仍未满足停稳判据，状态机将显式记录
    ``timed_out=True`` 与 ``settled_by_timeout=True``，并保留完整 ``last_stability``
    失败项与诊断指标后进入 ``READY_FOR_SLAM``，绝不将未通过的稳定性伪装为已通过。
    """

    def __init__(self, config: StartupWarmupConfig | None = None) -> None:
        self.config = config if config is not None else StartupWarmupConfig()
        self.phase: str = (
            PHASE_LAND_SETTLE if self.config.enabled else PHASE_READY_FOR_SLAM
        )
        self.t_start_s: float | None = None
        self.t_walk_start_s: float | None = None
        self.t_stop_start_s: float | None = None
        self.t_slam_start_s: float | None = None
        self.timed_out: bool = False
        self.settled_by_timeout: bool = False
        self.timeout_reason: str | None = None
        self.last_stability: TrendStabilityResult | None = None
        self.transitions: list[dict[str, Any]] = []

    @property
    def ready_for_slam(self) -> bool:
        """是否已结束开机热身与停稳阶段，允许外部航点/控制介入。"""
        return self.phase == PHASE_READY_FOR_SLAM

    @property
    def slam_sensors_enabled(self) -> bool:
        """当前是否允许向 SLAM 发布传感器流。"""
        if not self.config.enabled or not self.config.gate_slam_sensors_until_settled:
            return True
        return self.phase == PHASE_READY_FOR_SLAM

    def _record_transition(
        self,
        from_phase: str,
        to_phase: str,
        t_sim_s: float,
    ) -> dict[str, Any]:
        """构造并保存阶段转移诊断事件。"""
        event = {
            "event": "phase_transition",
            "from_phase": from_phase,
            "to_phase": to_phase,
            "t_sim_s": float(t_sim_s),
            "timed_out": bool(self.timed_out),
            "settled_by_timeout": bool(self.settled_by_timeout),
            "timeout_reason": self.timeout_reason,
            "stability": (
                self.last_stability.to_dict() if self.last_stability is not None else None
            ),
        }
        self.transitions.append(event)
        return event

    def step(
        self,
        t_sim_s: float,
        rows: Sequence[Mapping[str, Any]],
    ) -> tuple[np.ndarray, bool, dict[str, Any] | None]:
        """推进一次 50 Hz 控制步的状态机判定。

        返回 ``(warmup_cmd_xyz, slam_sensors_enabled, phase_transition_event)``。
        """
        t_now = float(t_sim_s)
        if not math.isfinite(t_now) or t_now < 0.0:
            raise ValueError(f"仿真时间 t_sim_s 必须非负且有限，收到 {t_sim_s!r}")

        if not self.config.enabled:
            if self.t_start_s is None:
                self.t_start_s = t_now
                self.t_slam_start_s = t_now
            return np.zeros(3, dtype=float), True, None

        if self.t_start_s is None:
            self.t_start_s = t_now

        start_phase = self.phase

        if self.phase == PHASE_LAND_SETTLE:
            if t_now - self.t_start_s >= self.config.settle_s:
                if self.config.walk_duration_s > 0.0:
                    self.phase = PHASE_WARMUP_WALK
                    self.t_walk_start_s = t_now
                else:
                    self.phase = PHASE_STOP_CONVERGE
                    self.t_stop_start_s = t_now

        elif self.phase == PHASE_WARMUP_WALK:
            assert self.t_walk_start_s is not None
            if t_now - self.t_walk_start_s >= self.config.walk_duration_s:
                self.phase = PHASE_STOP_CONVERGE
                self.t_stop_start_s = t_now

        elif self.phase == PHASE_STOP_CONVERGE:
            assert self.t_stop_start_s is not None
            stability = evaluate_trend_stability(rows, self.config.stability)
            self.last_stability = stability
            if stability.settled:
                self.phase = PHASE_READY_FOR_SLAM
                self.t_slam_start_s = t_now
                self.timed_out = False
                self.settled_by_timeout = False
                self.timeout_reason = None
            elif t_now - self.t_stop_start_s >= self.config.max_stop_wait_s:
                # 超时收口：保留真实未通过的 stability 指标，显式标记超时原因
                self.phase = PHASE_READY_FOR_SLAM
                self.t_slam_start_s = t_now
                self.timed_out = True
                self.settled_by_timeout = True
                self.timeout_reason = "max_stop_wait_exceeded"

        event: dict[str, Any] | None = None
        if self.phase != start_phase:
            event = self._record_transition(start_phase, self.phase, t_now)

        if self.phase == PHASE_WARMUP_WALK:
            cmd = np.array([self.config.walk_vx_mps, 0.0, 0.0], dtype=float)
        else:
            cmd = np.zeros(3, dtype=float)

        return cmd, self.slam_sensors_enabled, event

    def summary(self) -> dict[str, Any]:
        """导出各阶段精确仿真时间戳与最终稳定性诊断摘要。"""
        stop_converge_duration_s: float | None = None
        if self.t_stop_start_s is not None and self.t_slam_start_s is not None:
            stop_converge_duration_s = float(self.t_slam_start_s - self.t_stop_start_s)
        return {
            "enabled": bool(self.config.enabled),
            "phase": self.phase,
            "t_start_s": self.t_start_s,
            "t_walk_start_s": self.t_walk_start_s,
            "t_stop_start_s": self.t_stop_start_s,
            "t_slam_start_s": self.t_slam_start_s,
            "stop_converge_duration_s": stop_converge_duration_s,
            "timed_out": bool(self.timed_out),
            "settled_by_timeout": bool(self.settled_by_timeout),
            "timeout_reason": self.timeout_reason,
            "slam_sensors_enabled": bool(self.slam_sensors_enabled),
            "gate_slam_sensors_until_settled": bool(self.config.gate_slam_sensors_until_settled),
            "config": asdict(self.config),
            "final_stability": (
                self.last_stability.to_dict() if self.last_stability is not None else None
            ),
            "transitions": list(self.transitions),
        }
