"""有界纯图像地点召回索引；支持经典网格与视角不变局部特征聚合，绝不直接提供尺度位姿。"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from numbers import Real

import numpy as np

from .visual import ALLOWED_FEATURE_TYPES, VisualFeatures, extract_visual_features

# 按局部特征维度缓存确定性随机投影矩阵，使全局聚合描述子跨调用严格可复现。
_PROJECTION_CACHE: dict[int, np.ndarray] = {}


def _projection_matrix(dim: int, bins: int = 256) -> np.ndarray:
    proj = _PROJECTION_CACHE.get(dim)
    if proj is None:
        rng = np.random.default_rng(42 + dim)
        mat = rng.standard_normal((dim, bins)).astype(np.float32)
        mat /= np.maximum(np.linalg.norm(mat, axis=0, keepdims=True), 1e-6)
        _PROJECTION_CACHE[dim] = mat
        proj = mat
    return proj


def _count(name: str, value) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{name} 必须为正整数，当前收到 {value!r}")
    return value


def _positive(name: str, value) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} 必须为有限正数，当前收到 {value!r}")
    return float(value)


def _nonnegative(name: str, value) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value < 0:
        raise ValueError(f"{name} 必须为有限非负数，当前收到 {value!r}")
    return float(value)


@dataclass(frozen=True)
class VisualRetrievalConfig:
    """有界视觉召回与特征验证参数。"""

    max_entries: int = 2000
    candidates: int = 3
    max_distance: float = 0.25
    max_time_delta_s: float = 0.05
    min_contrast: float = 5.0
    # 特征类型：patch7x7（纯 numpy 基线）或 orb/akaze/brisk/sift（视角不变局部特征）。
    feature_type: str = "patch7x7"
    max_features: int = 800
    lidar_px_radius: float = 6.0
    verify_inlier_m: float = 0.15
    verify_min_inliers: int = 8
    verify_ratio_threshold: float = 0.78

    def __post_init__(self) -> None:
        # 在处理首帧传感器数据前即拦截非法 JSON 配置，避免无界内存增长或放行陈旧图像。
        _count("visual_retrieval.max_entries", self.max_entries)
        _count("visual_retrieval.candidates", self.candidates)
        dist = _nonnegative("visual_retrieval.max_distance", self.max_distance)
        if dist > 2.0:
            raise ValueError(f"visual_retrieval.max_distance 必须在 [0, 2] 内，当前收到 {self.max_distance!r}")
        _nonnegative("visual_retrieval.max_time_delta_s", self.max_time_delta_s)
        _nonnegative("visual_retrieval.min_contrast", self.min_contrast)
        if self.feature_type not in ALLOWED_FEATURE_TYPES:
            raise ValueError(
                f"visual_retrieval.feature_type 必须属于 {ALLOWED_FEATURE_TYPES}，当前收到 {self.feature_type!r}"
            )
        _count("visual_retrieval.max_features", self.max_features)
        _positive("visual_retrieval.lidar_px_radius", self.lidar_px_radius)
        _positive("visual_retrieval.verify_inlier_m", self.verify_inlier_m)
        _count("visual_retrieval.verify_min_inliers", self.verify_min_inliers)
        ratio = _positive("visual_retrieval.verify_ratio_threshold", self.verify_ratio_threshold)
        if ratio >= 1.0:
            raise ValueError(
                f"visual_retrieval.verify_ratio_threshold 必须在 (0, 1) 内，当前收到 {self.verify_ratio_threshold!r}"
            )


def _grid16_descriptor(gray: np.ndarray) -> np.ndarray | None:
    """从已矫正灰度图提取 16x16 零均值单位归一化块均值描述子。"""
    y = np.linspace(0, gray.shape[0], 17, dtype=int)
    x = np.linspace(0, gray.shape[1], 17, dtype=int)
    integral = np.pad(gray.cumsum(0).cumsum(1), ((1, 0), (1, 0)))
    blocks = (
        integral[y[1:, None], x[None, 1:]]
        - integral[y[:-1, None], x[None, 1:]]
        - integral[y[1:, None], x[None, :-1]]
        + integral[y[:-1, None], x[None, :-1]]
    )
    desc = (blocks / (np.diff(y)[:, None] * np.diff(x)[None, :])).ravel()
    desc -= desc.mean()
    norm = np.linalg.norm(desc)
    if norm <= 1e-8:
        return None
    return desc / norm


def _aggregate_local_descriptors(features: VisualFeatures) -> np.ndarray | None:
    """将整帧视角不变局部描述子无词典软聚合为 256 维 L2 归一化全局向量。

    仅使用图像局部描述子（不混入 LiDAR、里程计或真值），具备平移与旋转不变性。
    """
    raw = features.all_descriptors
    if raw is None or len(raw) < 8:
        return None
    if features.is_binary:
        # 二值描述子按位解包到 {-1, +1} 空间。
        x = np.unpackbits(raw, axis=1).astype(np.float32) * 2.0 - 1.0
    else:
        # 浮点描述子（如 SIFT）先做 RootSIFT 归一化再零中心化，抑制高频突发并拉开余弦区分度。
        x = raw.astype(np.float32)
        x = x / np.maximum(x.sum(axis=1, keepdims=True), 1e-6)
        x = np.sqrt(np.maximum(x, 0.0))
        x = x - x.mean(axis=1, keepdims=True)
    proj = _projection_matrix(x.shape[1], bins=256)
    vec = (np.maximum(x @ proj, 0.0) ** 2).mean(axis=0).astype(np.float64)
    vec = np.sign(vec) * np.sqrt(np.abs(vec))
    vec -= vec.mean()
    norm = np.linalg.norm(vec)
    if norm <= 1e-8:
        return None
    return vec / norm


def image_descriptor(
    keyframe,
    config: VisualRetrievalConfig | None = None,
    *,
    features: VisualFeatures | None = None,
) -> np.ndarray | None:
    """提取全局图像地点描述子；缺失图像、时间超限、过小或低对比度帧直接返回 None。"""
    c = config or VisualRetrievalConfig()
    observation = keyframe.visual
    if observation is None or abs(observation.stamp_ns - keyframe.stamp_ns) > c.max_time_delta_s * 1e9:
        return None
    gray = np.asarray(observation.gray, dtype=float)
    if min(gray.shape) < 16 or np.std(gray) < c.min_contrast:
        return None
    if c.feature_type == "patch7x7":
        return _grid16_descriptor(gray)
    local = (
        features
        if features is not None and features.feature_type == c.feature_type
        else extract_visual_features(
            keyframe,
            feature_type=c.feature_type,
            max_features=c.max_features,
            lidar_px_radius=c.lidar_px_radius,
        )
    )
    return _aggregate_local_descriptors(local)


class VisualIndex:
    """按最老条目淘汰（FIFO）的有界图像描述子与局部特征缓存索引。

    建图与先验重定位共用同一索引结构，由调用方通过 exclude 回调指定时间窗与近邻屏蔽策略。
    """

    def __init__(self, config: VisualRetrievalConfig | None = None) -> None:
        self.config = config or VisualRetrievalConfig()
        self.descriptors: OrderedDict = OrderedDict()
        self.local_features: OrderedDict = OrderedDict()

    def extract(self, keyframe) -> tuple[np.ndarray | None, VisualFeatures | None]:
        """单次提取当前关键帧的全局检索向量与（可选）局部 3D 支撑特征，供检索与验证复用。"""
        c = self.config
        observation = keyframe.visual
        if observation is None or abs(observation.stamp_ns - keyframe.stamp_ns) > c.max_time_delta_s * 1e9:
            return None, None
        gray = np.asarray(observation.gray, dtype=float)
        if min(gray.shape) < 16 or np.std(gray) < c.min_contrast:
            return None, None
        if c.feature_type == "patch7x7":
            return _grid16_descriptor(gray), None
        local = extract_visual_features(
            keyframe,
            feature_type=c.feature_type,
            max_features=c.max_features,
            lidar_px_radius=c.lidar_px_radius,
        )
        return _aggregate_local_descriptors(local), local

    def add(
        self,
        keyframe,
        *,
        precomputed: tuple[np.ndarray | None, VisualFeatures | None] | None = None,
    ) -> None:
        if precomputed is None:
            desc, local = self.extract(keyframe)
        else:
            desc, local = precomputed
        self.descriptors.pop(keyframe.key, None)
        self.local_features.pop(keyframe.key, None)
        if desc is None:
            return
        self.descriptors[keyframe.key] = desc
        if local is not None:
            self.local_features[keyframe.key] = local
        while len(self.descriptors) > self.config.max_entries:
            old_key, _ = self.descriptors.popitem(last=False)
            self.local_features.pop(old_key, None)

    def query(
        self,
        keyframe,
        exclude=None,
        *,
        precomputed_desc: np.ndarray | None = None,
    ) -> list[tuple]:
        desc = precomputed_desc if precomputed_desc is not None else image_descriptor(keyframe, self.config)
        if desc is None:
            return []
        keys = [key for key in self.descriptors if key != keyframe.key and (exclude is None or not exclude(key))]
        if not keys:
            return []
        # 余弦距离 1 - cos(theta)，范围截断在 [0, 2]。
        distances = np.clip(1.0 - np.vstack([self.descriptors[key] for key in keys]) @ desc, 0.0, 2.0)
        return [
            (keys[i], float(distances[i]))
            for i in np.argsort(distances, kind="stable")[: self.config.candidates]
            if distances[i] <= self.config.max_distance
        ]
