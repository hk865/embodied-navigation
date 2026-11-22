"""Color-ScanContext：融合机体系 LiDAR 极坐标网格与相机 RGB/灰度颜色的视角重叠重排描述符。

在保留原始 ScanContext ``(rings, sectors)`` 极坐标划分的基础上，将视锥内实测
3D LiDAR 点投影到相机图像平面，赋予归一化颜色向量（优先使用 RGB 3 通道，缺失时
回退到单通道归一化灰度），并在每个极坐标栅格内统计均值颜色与有效观测掩码。

计算旋转对齐后的颜色距离 ``color_sc_distance(desc_a, desc_b, shift)`` 时，仅对
两帧同时存在有效相机颜色观测的交集栅格求和；若重叠栅格数少于 ``min_color_bins``
（如反向重访时前向相机视野无重叠），则返回 ``None``，从而只重排/过滤同向重叠视野
候选，绝不误伤反向 LiDAR 回环。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .keyframes import transform_points
from .pose_graph import invert
from .scan_context import ScanContextConfig
from .visual import VisualObservation

_MIN_CAMERA_DEPTH_M = 0.2
_LUMA_WEIGHTS = np.array([0.299, 0.587, 0.114], dtype=np.float64)


@dataclass(frozen=True)
class ColorScanContextDescriptor:
    """``(rings, sectors, C)`` 颜色极坐标网格与有效观测掩码。"""

    grid: np.ndarray  # (rings, sectors, C) float64, values in [0, 1]
    mask: np.ndarray  # (rings, sectors) bool
    channels: int


def yaw_to_shift(yaw_rad: float, config: ScanContextConfig) -> int:
    """将 ``shift_to_yaw`` 对应的旋转角逆映射为最邻近的列偏移整数 ``shift``。"""
    sector_step = 2.0 * np.pi / float(config.sectors)
    return int(np.rint(np.mod(yaw_rad, 2.0 * np.pi) / sector_step)) % config.sectors


def color_descriptor(
    points_body: np.ndarray,
    observation: VisualObservation | None,
    config: ScanContextConfig,
) -> ColorScanContextDescriptor | None:
    """将 ``points_body`` 投影到相机图像并构建 ``(rings, sectors)`` Color-ScanContext 描述子。"""
    if observation is None or len(points_body) == 0:
        return None
    pts = np.asarray(points_body, dtype=np.float64)
    radius = np.hypot(pts[:, 0], pts[:, 1])
    inside_range = radius < config.max_range_m
    if not np.any(inside_range):
        return None
    pts = pts[inside_range]
    radius = radius[inside_range]

    p_cam = transform_points(invert(observation.T_body_camera), pts)
    in_front = p_cam[:, 2] > _MIN_CAMERA_DEPTH_M
    if not np.any(in_front):
        return None
    p_cam = p_cam[in_front]
    pts = pts[in_front]
    radius = radius[in_front]

    projected = p_cam @ observation.K.T
    uv = projected[:, :2] / projected[:, 2:3]
    h, w = observation.gray.shape
    u = np.floor(uv[:, 0]).astype(np.int64)
    v = np.floor(uv[:, 1]).astype(np.int64)
    in_image = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    if not np.any(in_image):
        return None

    u = u[in_image]
    v = v[in_image]
    pts = pts[in_image]
    radius = radius[in_image]

    if observation.rgb is not None:
        colors = observation.rgb[v, u].astype(np.float64) / 255.0
        channels = 3
    else:
        gray_norm = np.clip(np.asarray(observation.gray, dtype=np.float64) / 255.0, 0.0, 1.0)
        colors = gray_norm[v, u, None]
        channels = 1

    azimuth = np.mod(np.arctan2(pts[:, 1], pts[:, 0]), 2.0 * np.pi)
    ring = np.minimum((radius / config.max_range_m * config.rings).astype(np.int64), config.rings - 1)
    sector = np.minimum((azimuth / (2.0 * np.pi) * config.sectors).astype(np.int64), config.sectors - 1)

    sums = np.zeros((config.rings, config.sectors, channels), dtype=np.float64)
    counts = np.zeros((config.rings, config.sectors), dtype=np.int64)
    np.add.at(sums, (ring, sector), colors)
    np.add.at(counts, (ring, sector), 1)
    mask = counts > 0
    if not np.any(mask):
        return None
    grid = np.zeros_like(sums)
    grid[mask] = sums[mask] / counts[mask, None]
    return ColorScanContextDescriptor(grid=grid, mask=mask, channels=channels)


def _match_channels(
    grid_a: np.ndarray,
    grid_b: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """当一侧为 RGB、另一侧仅灰度时统一降维到亮度通道比较。"""
    if grid_a.shape[2] == grid_b.shape[2]:
        return grid_a, grid_b
    if grid_a.shape[2] == 3:
        grid_a = (grid_a @ _LUMA_WEIGHTS)[:, :, None]
    if grid_b.shape[2] == 3:
        grid_b = (grid_b @ _LUMA_WEIGHTS)[:, :, None]
    return grid_a, grid_b


def color_sc_distance(
    desc_a: ColorScanContextDescriptor,
    desc_b: ColorScanContextDescriptor,
    shift: int = 0,
    *,
    min_color_bins: int = 12,
) -> float | None:
    """计算列偏移 ``shift`` 下重叠观测栅格的归一化均方根颜色距离 ``[0, 1]``。

    重叠有效栅格数不足 ``min_color_bins`` 时返回 ``None``（不惩罚视野无重叠的候选）。
    """
    if desc_a.grid.shape[:2] != desc_b.grid.shape[:2]:
        raise ValueError(
            f"ColorScanContext 网格尺寸不一致: {desc_a.grid.shape[:2]} vs {desc_b.grid.shape[:2]}"
        )
    col_shift = int(shift) % desc_b.grid.shape[1]
    b_grid = np.roll(desc_b.grid, -col_shift, axis=1)
    b_mask = np.roll(desc_b.mask, -col_shift, axis=1)
    overlap = desc_a.mask & b_mask
    if int(overlap.sum()) < min_color_bins:
        return None
    grid_a, grid_b = _match_channels(desc_a.grid, b_grid)
    diff = grid_a[overlap] - grid_b[overlap]
    per_bin_rms = np.sqrt(np.mean(diff * diff, axis=1))
    return float(np.clip(np.mean(per_bin_rms), 0.0, 1.0))


class ColorScanContextIndex:
    """按关键帧键索引 Color-ScanContext 描述子，用于 ScanContext 候选重排与颜色门控。"""

    def __init__(
        self,
        config: ScanContextConfig,
        *,
        min_color_bins: int = 12,
        color_weight: float = 0.25,
        max_color_distance: float | None = None,
    ) -> None:
        self.config = config
        self.min_color_bins = min_color_bins
        self.color_weight = float(color_weight)
        self.max_color_distance = max_color_distance
        self.descriptors: dict[object, ColorScanContextDescriptor] = {}

    def compute(self, keyframe) -> ColorScanContextDescriptor | None:
        if keyframe.visual is None:
            return None
        return color_descriptor(keyframe.points_body(), keyframe.visual, self.config)

    def add(self, key, desc: ColorScanContextDescriptor | None) -> None:
        if desc is not None:
            self.descriptors[key] = desc

    def add_keyframe(self, keyframe) -> ColorScanContextDescriptor | None:
        desc = self.compute(keyframe)
        self.add(keyframe.key, desc)
        return desc

    def rescore(
        self,
        query_desc: ColorScanContextDescriptor | None,
        sc_candidates: list[tuple[object, float, float]],
    ) -> tuple[list[tuple[object, float, float, float]], int, int]:
        """对 ScanContext 候选列表按 ``geom_dist + color_weight * color_dist`` 重排并可选过滤。

        返回 ``(ranked_candidates, rescored_count, rejected_count)``，其中每项为
        ``(key, geom_dist, yaw_rad, combined_dist)``。
        """
        ranked: list[tuple[object, float, float, float]] = []
        rescored_count = 0
        rejected_count = 0
        for key, geom_dist, yaw_rad in sc_candidates:
            cand_desc = self.descriptors.get(key)
            c_dist = None
            if query_desc is not None and cand_desc is not None:
                shift = yaw_to_shift(yaw_rad, self.config)
                c_dist = color_sc_distance(
                    query_desc,
                    cand_desc,
                    shift,
                    min_color_bins=self.min_color_bins,
                )
            if c_dist is not None:
                rescored_count += 1
                if self.max_color_distance is not None and c_dist > self.max_color_distance:
                    rejected_count += 1
                    continue
                combined = float(geom_dist) + self.color_weight * c_dist
            else:
                combined = float(geom_dist)
            ranked.append((key, float(geom_dist), float(yaw_rad), combined))
        ranked.sort(key=lambda row: row[3])
        return ranked, rescored_count, rejected_count
