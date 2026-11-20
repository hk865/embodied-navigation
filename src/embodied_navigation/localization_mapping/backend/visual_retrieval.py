"""有界纯图像地点召回索引；描述子仅提议候选帧，绝不直接提供尺度位姿。"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from numbers import Real

import numpy as np


def _count(name: str, value) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{name} 必须为正整数，当前收到 {value!r}")
    return value


def _nonnegative(name: str, value) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value < 0:
        raise ValueError(f"{name} 必须为有限非负数，当前收到 {value!r}")
    return float(value)


@dataclass(frozen=True)
class VisualRetrievalConfig:
    """有界视觉召回索引参数。"""

    max_entries: int = 2000
    candidates: int = 3
    max_distance: float = 0.25
    max_time_delta_s: float = 0.05
    min_contrast: float = 5.0

    def __post_init__(self) -> None:
        # 在处理首帧传感器数据前即拦截非法 JSON 配置，避免无界内存增长或放行陈旧图像。
        _count("visual_retrieval.max_entries", self.max_entries)
        _count("visual_retrieval.candidates", self.candidates)
        dist = _nonnegative("visual_retrieval.max_distance", self.max_distance)
        if dist > 2.0:
            raise ValueError(f"visual_retrieval.max_distance 必须在 [0, 2] 内，当前收到 {self.max_distance!r}")
        _nonnegative("visual_retrieval.max_time_delta_s", self.max_time_delta_s)
        _nonnegative("visual_retrieval.min_contrast", self.min_contrast)


def image_descriptor(keyframe, config: VisualRetrievalConfig | None = None) -> np.ndarray | None:
    """从已矫正灰度图提取 16x16 零均值单位归一化块均值描述子。

    仅对亮度线性偏移/缩放具有不变性，不声称对大视角或剧烈光照变化鲁棒；
    缺失图像、时间戳不同步、分辨率过小或低对比度帧直接返回 None。
    描述子严格不混入 LiDAR、里程计或真值信息。
    """
    c = config or VisualRetrievalConfig()
    observation = keyframe.visual
    if observation is None or abs(observation.stamp_ns - keyframe.stamp_ns) > c.max_time_delta_s * 1e9:
        return None
    gray = np.asarray(observation.gray, dtype=float)
    if min(gray.shape) < 16 or np.std(gray) < c.min_contrast:
        return None
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


class VisualIndex:
    """按最老条目淘汰（FIFO）的有界图像描述子索引。

    建图与先验重定位共用同一索引结构，由调用方通过 exclude 回调指定时间窗与近邻屏蔽策略。
    """

    def __init__(self, config: VisualRetrievalConfig | None = None) -> None:
        self.config = config or VisualRetrievalConfig()
        self.descriptors: OrderedDict = OrderedDict()

    def add(self, keyframe) -> None:
        desc = image_descriptor(keyframe, self.config)
        self.descriptors.pop(keyframe.key, None)
        if desc is None:
            return
        self.descriptors[keyframe.key] = desc
        while len(self.descriptors) > self.config.max_entries:
            self.descriptors.popitem(last=False)

    def query(self, keyframe, exclude=None) -> list[tuple]:
        desc = image_descriptor(keyframe, self.config)
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
