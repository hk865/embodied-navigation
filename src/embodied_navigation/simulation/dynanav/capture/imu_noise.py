"""物理级 MEMS IMU 零偏、随机游走、白噪声与安装倾角注入模块。

本模块用于在仿真采集发布的 ``/dynanav/imu`` 流上叠加贴近真实 MEMS 惯导的误差模型：
1. 初始常值零偏（``gyro_bias_init_radps`` / ``accel_bias_init_mps2``）；
2. 布朗运动零偏随机游走（``gyro_bias_rw_radps_sqrts`` / ``accel_bias_rw_mps2_sqrts``）；
3. 高斯白噪声（按连续时间噪声密度 $\\sigma_d / \\sqrt{\\Delta t}$ 换算为单帧离散标准差）；
4. 可选微小安装旋转偏斜（``mount_tilt_rpy_rad``，按 ZYX 欧拉角 $R_z(\\psi) R_y(\\theta) R_x(\\phi)$ 作用）。

默认 ``enabled=False``，未启用时不修改任何原始 IMU 读数；仅依赖纯 ``numpy`` 与标准库。
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from typing import Any

import numpy as np


def _require_bool(value: Any, name: str) -> bool:
    """严格校验布尔配置项，拒绝非布尔类型隐式转换。"""
    if type(value) is not bool:
        raise ValueError(f"{name} 必须为布尔值，收到 {value!r}")
    return value


def _require_nonneg_int(value: Any, name: str) -> int:
    """严格校验非负整数（如随机数种子）。"""
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} 必须为非负整数，收到 {value!r}")
    return value


def _require_nonneg_float(value: Any, name: str) -> float:
    """严格校验非负有限浮点数。"""
    if type(value) not in (int, float):
        raise ValueError(f"{name} 必须为非负有限数值，收到 {value!r}")
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise ValueError(f"{name} 必须为非负有限数值，收到 {value!r}")
    return number


def _require_vec3(value: Any, name: str) -> tuple[float, float, float]:
    """严格校验长度为 3 的有限数值向量并冻结为三元组。"""
    if isinstance(value, (str, bytes)) or not isinstance(value, (Sequence, np.ndarray)):
        raise ValueError(f"{name} 必须为长度为 3 的有限数值序列，收到 {value!r}")
    items = list(value)
    if len(items) != 3:
        raise ValueError(f"{name} 长度必须为 3，收到 {len(items)}")
    parsed: list[float] = []
    for item in items:
        if isinstance(item, np.generic):
            item = item.item()
        if type(item) not in (int, float) or not math.isfinite(float(item)):
            raise ValueError(f"{name} 分量必须为有限数值，收到 {item!r}")
        parsed.append(float(item))
    return (parsed[0], parsed[1], parsed[2])


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """将小角度安装误差 (roll, pitch, yaw) 转换为 3x3 正交旋转矩阵 Rz @ Ry @ Rx。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rz @ ry @ rx


@dataclass(frozen=True)
class ImuNoiseConfig:
    """物理级 IMU 噪声与零偏注入配置（默认关闭，保持零侵入基线）。"""

    enabled: bool = False
    seed: int = 20261121
    gyro_bias_init_radps: tuple[float, float, float] = (0.015, -0.010, 0.020)
    accel_bias_init_mps2: tuple[float, float, float] = (0.10, -0.08, 0.10)
    gyro_noise_density_radps_sqrthz: float = 0.0005
    accel_noise_density_mps2_sqrthz: float = 0.005
    gyro_bias_rw_radps_sqrts: float = 0.0002
    accel_bias_rw_mps2_sqrts: float = 0.002
    mount_tilt_rpy_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        object.__setattr__(self, "enabled", _require_bool(self.enabled, "enabled"))
        object.__setattr__(self, "seed", _require_nonneg_int(self.seed, "seed"))
        object.__setattr__(
            self,
            "gyro_bias_init_radps",
            _require_vec3(self.gyro_bias_init_radps, "gyro_bias_init_radps"),
        )
        object.__setattr__(
            self,
            "accel_bias_init_mps2",
            _require_vec3(self.accel_bias_init_mps2, "accel_bias_init_mps2"),
        )
        object.__setattr__(
            self,
            "gyro_noise_density_radps_sqrthz",
            _require_nonneg_float(
                self.gyro_noise_density_radps_sqrthz, "gyro_noise_density_radps_sqrthz"
            ),
        )
        object.__setattr__(
            self,
            "accel_noise_density_mps2_sqrthz",
            _require_nonneg_float(
                self.accel_noise_density_mps2_sqrthz, "accel_noise_density_mps2_sqrthz"
            ),
        )
        object.__setattr__(
            self,
            "gyro_bias_rw_radps_sqrts",
            _require_nonneg_float(self.gyro_bias_rw_radps_sqrts, "gyro_bias_rw_radps_sqrts"),
        )
        object.__setattr__(
            self,
            "accel_bias_rw_mps2_sqrts",
            _require_nonneg_float(self.accel_bias_rw_mps2_sqrts, "accel_bias_rw_mps2_sqrts"),
        )
        object.__setattr__(
            self,
            "mount_tilt_rpy_rad",
            _require_vec3(self.mount_tilt_rpy_rad, "mount_tilt_rpy_rad"),
        )

    @classmethod
    def from_dict(cls, data: Mapping[str, Any] | None) -> "ImuNoiseConfig":
        """从实验配置字典严格构建 ``ImuNoiseConfig``，拒绝未知键与非法类型。"""
        if data is None:
            return cls()
        if not isinstance(data, Mapping):
            raise ValueError("imu_noise 配置必须为字典映射")
        allowed = {
            "enabled",
            "seed",
            "gyro_bias_init_radps",
            "accel_bias_init_mps2",
            "gyro_noise_density_radps_sqrthz",
            "accel_noise_density_mps2_sqrthz",
            "gyro_bias_rw_radps_sqrts",
            "accel_bias_rw_mps2_sqrts",
            "mount_tilt_rpy_rad",
        }
        unknown = sorted(set(data.keys()) - allowed)
        if unknown:
            raise ValueError(f"ImuNoiseConfig 包含未知配置键: {', '.join(unknown)}")
        return cls(**dict(data))


class ImuNoiseInjector:
    """确定性、非原地修改的物理级 IMU 误差注入器。"""

    def __init__(self, config: ImuNoiseConfig | None = None) -> None:
        self.config = config if config is not None else ImuNoiseConfig()
        self.rng = np.random.default_rng(self.config.seed)
        self.gyro_bias = np.array(self.config.gyro_bias_init_radps, dtype=float)
        self.accel_bias = np.array(self.config.accel_bias_init_mps2, dtype=float)
        self.R_mount = _rpy_to_matrix(*self.config.mount_tilt_rpy_rad)
        self.sample_index = 0

    def apply(self, reading: Mapping[str, Any], dt_s: float) -> tuple[dict[str, Any], dict[str, Any]]:
        """对单帧 IMU 读数施加零偏、随机游走、安装偏斜与白噪声，绝不原地修改输入。

        返回 ``(corrupted_reading, bias_truth_row)``。
        """
        dt = float(dt_s)
        if type(dt_s) not in (int, float) or not math.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"dt_s 必须为正有限数值，收到 {dt_s!r}")

        accel_raw = np.asarray(reading["linear_acceleration"], dtype=float)
        gyro_raw = np.asarray(reading["angular_velocity"], dtype=float)
        if accel_raw.shape != (3,) or gyro_raw.shape != (3,):
            raise ValueError("IMU reading 的 linear_acceleration 与 angular_velocity 必须为 3 维向量")
        if not (np.isfinite(accel_raw).all() and np.isfinite(gyro_raw).all()):
            raise ValueError("IMU reading 包含非有限加速度或角速度")

        # 浅拷贝字典并复制数组字段，保证调用方和原 reading 完全隔离
        corrupted: dict[str, Any] = dict(reading)
        if "orientation" in reading and reading["orientation"] is not None:
            corrupted["orientation"] = np.array(reading["orientation"], dtype=float, copy=True)

        time_val = float(reading["time"]) if "time" in reading and reading["time"] is not None else None

        if not self.config.enabled:
            corrupted["linear_acceleration"] = accel_raw.copy()
            corrupted["angular_velocity"] = gyro_raw.copy()
            bias_truth_row = {
                "enabled": False,
                "sample_index": self.sample_index,
                "time_s": time_val,
                "dt_s": dt,
                "gyro_bias_radps": [0.0, 0.0, 0.0],
                "accel_bias_mps2": [0.0, 0.0, 0.0],
                "gyro_noise_radps": [0.0, 0.0, 0.0],
                "accel_noise_mps2": [0.0, 0.0, 0.0],
                "gyro_raw_radps": gyro_raw.tolist(),
                "accel_raw_mps2": accel_raw.tolist(),
                "gyro_corrupted_radps": gyro_raw.tolist(),
                "accel_corrupted_mps2": accel_raw.tolist(),
            }
            self.sample_index += 1
            return corrupted, bias_truth_row

        sqrt_dt = math.sqrt(dt)
        inv_sqrt_dt = 1.0 / sqrt_dt

        # 1. 布朗运动更新当前零偏真值
        if self.config.gyro_bias_rw_radps_sqrts > 0.0:
            self.gyro_bias = self.gyro_bias + self.rng.normal(
                0.0, self.config.gyro_bias_rw_radps_sqrts * sqrt_dt, size=3
            )
        if self.config.accel_bias_rw_mps2_sqrts > 0.0:
            self.accel_bias = self.accel_bias + self.rng.normal(
                0.0, self.config.accel_bias_rw_mps2_sqrts * sqrt_dt, size=3
            )

        # 2. 施加可选安装倾角旋转 R_mount
        gyro_rot = self.R_mount @ gyro_raw
        accel_rot = self.R_mount @ accel_raw

        # 3. 按采样周期离散化白噪声并合成测量值
        gyro_white_std = self.config.gyro_noise_density_radps_sqrthz * inv_sqrt_dt
        accel_white_std = self.config.accel_noise_density_mps2_sqrthz * inv_sqrt_dt
        gyro_noise = (
            self.rng.normal(0.0, gyro_white_std, size=3)
            if gyro_white_std > 0.0
            else np.zeros(3, dtype=float)
        )
        accel_noise = (
            self.rng.normal(0.0, accel_white_std, size=3)
            if accel_white_std > 0.0
            else np.zeros(3, dtype=float)
        )

        gyro_corrupted = gyro_rot + self.gyro_bias + gyro_noise
        accel_corrupted = accel_rot + self.accel_bias + accel_noise

        corrupted["angular_velocity"] = gyro_corrupted
        corrupted["linear_acceleration"] = accel_corrupted

        bias_truth_row = {
            "enabled": True,
            "sample_index": self.sample_index,
            "time_s": time_val,
            "dt_s": dt,
            "gyro_bias_radps": self.gyro_bias.tolist(),
            "accel_bias_mps2": self.accel_bias.tolist(),
            "gyro_noise_radps": gyro_noise.tolist(),
            "accel_noise_mps2": accel_noise.tolist(),
            "gyro_raw_radps": gyro_raw.tolist(),
            "accel_raw_mps2": accel_raw.tolist(),
            "gyro_corrupted_radps": gyro_corrupted.tolist(),
            "accel_corrupted_mps2": accel_corrupted.tolist(),
        }
        self.sample_index += 1
        return corrupted, bias_truth_row
