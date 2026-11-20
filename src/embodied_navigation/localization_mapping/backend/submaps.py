"""连续实测 LiDAR 子图构建模块：每个融合点均保留其来源关键帧键，便于溯源与消融。"""

from __future__ import annotations

from dataclasses import dataclass
from numbers import Real

import numpy as np

from .keyframes import transform_points
from .pose_graph import invert, pose_difference

ALLOWED_ANCHOR_MODES = ("nearest", "candidate")


def _positive(name: str, value) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} 必须为有限正数，当前收到 {value!r}")
    return float(value)


@dataclass(frozen=True)
class SubmapConfig:
    """多帧实测子图参数；支持独立拆开验证查询侧、历史侧与锚点选择的影响。"""

    max_frames: int = 5
    max_translation_m: float = 2.0
    max_height_m: float = 0.5
    max_rotation_rad: float = 0.35
    max_gap_s: float = 3.0
    voxel_m: float = 0.15
    use_query_submap: bool = True
    use_history_submap: bool = True
    # "nearest" 沿用基线选空间最近邻作回环锚点；"candidate" 固定以候选帧作锚点。
    anchor_mode: str = "nearest"

    def __post_init__(self) -> None:
        # 外部 JSON 配置必须在处理第一帧前完成校验，拒绝零体素、空窗口或非法枚举值。
        if isinstance(self.max_frames, bool) or not isinstance(self.max_frames, int) or self.max_frames < 1:
            raise ValueError(f"submap.max_frames 必须为正整数，当前收到 {self.max_frames!r}")
        for name in ("max_translation_m", "max_height_m", "max_rotation_rad", "max_gap_s", "voxel_m"):
            _positive(f"submap.{name}", getattr(self, name))
        if not isinstance(self.use_query_submap, bool) or not isinstance(self.use_history_submap, bool):
            raise ValueError("submap.use_query_submap 与 use_history_submap 必须为布尔值")
        if self.anchor_mode not in ALLOWED_ANCHOR_MODES:
            raise ValueError(f"submap.anchor_mode 必须属于 {ALLOWED_ANCHOR_MODES}，当前收到 {self.anchor_mode!r}")


@dataclass(frozen=True)
class MeasuredSubmap:
    """锚点机体系下的融合实测点云及其逐点来源关键帧。"""

    points_body: np.ndarray
    source_keys: tuple
    point_sources: tuple


def build_submap(keyframes, anchor, config: SubmapConfig | None = None) -> MeasuredSubmap:
    """从锚点帧沿时间向后连续累积局部实测点云，遇到任一连续性预算超限即停止。

    相对变换仅使用前端原始里程计 ``T_odom_body``（不混入其他遍次的优化位姿），
    以限制单段轨迹漂移；体素内取质心保证重复观测不会重复加权。
    """
    c = config or SubmapConfig()
    frames = list(keyframes)
    index = next(i for i, frame in enumerate(frames) if frame.key == anchor.key)
    selected, clouds, labels = [], [], []
    previous = anchor
    for frame in reversed(frames[: index + 1]):
        delta = invert(anchor.T_odom_body) @ frame.T_odom_body
        translation, rotation = pose_difference(np.eye(4), delta)
        gap = (previous.stamp_ns - frame.stamp_ns) * 1e-9
        # 跨 epoch、时间倒退、时间间隔或相对运动超出预算时立刻截断累积窗口。
        if (
            frame.key[0] != anchor.key[0]
            or gap < 0
            or gap > c.max_gap_s
            or translation > c.max_translation_m
            or abs(frame.T_odom_body[2, 3] - anchor.T_odom_body[2, 3]) > c.max_height_m
            or rotation > c.max_rotation_rad
        ):
            break
        selected.append(frame.key)
        points = transform_points(delta, frame.points_body())
        clouds.append(points)
        labels.extend([frame.key] * len(points))
        previous = frame
        if len(selected) >= c.max_frames:
            break
    points = np.vstack(clouds)
    _, inverse, counts = np.unique(
        np.floor(points / c.voxel_m).astype(np.int64),
        axis=0,
        return_inverse=True,
        return_counts=True,
    )
    inverse = inverse.reshape(-1)
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, inverse, points)
    provenance = [set() for _ in counts]
    for cell, key in zip(inverse, labels):
        provenance[cell].add(key)
    return MeasuredSubmap(
        points_body=sums / counts[:, None],
        source_keys=tuple(selected),
        point_sources=tuple(tuple(sorted(keys)) for keys in provenance),
    )
