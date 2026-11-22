"""连续实测 LiDAR 子图构建模块：支持双向邻域、时空运动边界截断与按需逐点溯源。"""

from __future__ import annotations

from dataclasses import dataclass
from numbers import Real

import numpy as np

from .keyframes import transform_points, voxel_downsample
from .pose_graph import invert, pose_difference

ALLOWED_ANCHOR_MODES = ("nearest", "candidate")


def _positive(name: str, value) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} 必须为有限正数，当前收到 {value!r}")
    return float(value)


def _nonnegative_int(name: str, value) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} 必须为非负整数，当前收到 {value!r}")
    return value


def _epoch_of(key):
    """仅对 (epoch, sequence) 形式的元组键提取 epoch，避免把字符串首字符误当成 epoch。"""
    if isinstance(key, tuple) and len(key) >= 1:
        return key[0]
    return None


@dataclass(frozen=True)
class SubmapConfig:
    """多帧实测子图参数；支持独立拆开验证查询侧、历史侧、时空运动边界与锚点选择。"""

    max_frames: int = 5
    max_translation_m: float = 2.0
    max_height_m: float = 0.5
    max_rotation_rad: float = 0.35
    max_gap_s: float = 3.0
    voxel_m: float = 0.15
    use_query_submap: bool = True
    use_history_submap: bool = True
    # "nearest" 选空间最近邻作回环锚点；"candidate" 固定以候选帧作锚点（默认）。
    anchor_mode: str = "candidate"
    # 可选双向邻域半径（每侧最多取 submap_window 帧）与相对锚点的绝对时间/旋转上界。
    submap_window: int | None = None
    submap_max_span_s: float | None = None
    submap_max_yaw_span_deg: float | None = None

    def __post_init__(self) -> None:
        if isinstance(self.max_frames, bool) or not isinstance(self.max_frames, int) or self.max_frames < 1:
            raise ValueError(f"submap.max_frames 必须为正整数，当前收到 {self.max_frames!r}")
        for name in ("max_translation_m", "max_height_m", "max_rotation_rad", "max_gap_s", "voxel_m"):
            _positive(f"submap.{name}", getattr(self, name))
        if not isinstance(self.use_query_submap, bool) or not isinstance(self.use_history_submap, bool):
            raise ValueError("submap.use_query_submap 与 use_history_submap 必须为布尔值")
        if self.anchor_mode not in ALLOWED_ANCHOR_MODES:
            raise ValueError(f"submap.anchor_mode 必须属于 {ALLOWED_ANCHOR_MODES}，当前收到 {self.anchor_mode!r}")
        if self.submap_window is not None:
            _nonnegative_int("submap.submap_window", self.submap_window)
        if self.submap_max_span_s is not None:
            _positive("submap.submap_max_span_s", self.submap_max_span_s)
        if self.submap_max_yaw_span_deg is not None:
            _positive("submap.submap_max_yaw_span_deg", self.submap_max_yaw_span_deg)


@dataclass(frozen=True)
class MeasuredSubmap:
    """锚点机体系下的融合实测点云及其来源关键帧。"""

    points_body: np.ndarray
    source_keys: tuple
    point_sources: tuple


def build_submap(
    keyframes,
    anchor,
    config: SubmapConfig | None = None,
    *,
    excluded: set | None = None,
    exclude_keys: set | tuple | list | None = None,
    track_sources: bool = True,
) -> MeasuredSubmap:
    """围绕锚点帧连续累积局部实测点云，遇到排除断点或任一运动/时间预算超限即截断。

    相对变换仅使用前端原始里程计 ``T_odom_body``（不混入其他遍次的优化位姿），
    以限制单段轨迹漂移；体素内取质心保证重复观测不会重复加权。
    """
    c = config or SubmapConfig()
    effective_excluded = set(excluded) if excluded is not None else set()
    if exclude_keys is not None:
        effective_excluded.update(exclude_keys)
    frames = list(keyframes)
    index = next(i for i, frame in enumerate(frames) if frame.key == anchor.key)
    anchor_epoch = _epoch_of(anchor.key)
    rot_limit_rad = (
        float(np.radians(c.submap_max_yaw_span_deg))
        if c.submap_max_yaw_span_deg is not None
        else c.max_rotation_rad
    )

    selected = [anchor.key]
    clouds = [anchor.points_body()]
    labels = [anchor.key] * len(clouds[0]) if track_sources else []

    def _walk(candidates, *, forward: bool, limit: int) -> None:
        if limit <= 0:
            return
        previous = anchor
        taken = 0
        for frame in candidates:
            if frame.key in effective_excluded:
                # 遇到排除帧必须立刻截断，严禁跨过断点将前后两段拼接进同一子图。
                break
            gap = ((frame.stamp_ns - previous.stamp_ns) if forward else (previous.stamp_ns - frame.stamp_ns)) * 1e-9
            span_s = abs(anchor.stamp_ns - frame.stamp_ns) * 1e-9
            delta = invert(anchor.T_odom_body) @ frame.T_odom_body
            translation, rotation = pose_difference(np.eye(4), delta)
            if (
                _epoch_of(frame.key) != anchor_epoch
                or gap < 0
                or gap > c.max_gap_s
                or (c.submap_max_span_s is not None and span_s > c.submap_max_span_s)
                or translation > c.max_translation_m
                or abs(frame.T_odom_body[2, 3] - anchor.T_odom_body[2, 3]) > c.max_height_m
                or rotation > rot_limit_rad
            ):
                break
            selected.append(frame.key)
            points = transform_points(delta, frame.points_body())
            clouds.append(points)
            if track_sources:
                labels.extend([frame.key] * len(points))
            previous = frame
            taken += 1
            if taken >= limit:
                break

    if c.submap_window is None:
        _walk(reversed(frames[:index]), forward=False, limit=c.max_frames - 1)
    else:
        _walk(reversed(frames[:index]), forward=False, limit=c.submap_window)
        _walk(frames[index + 1 :], forward=True, limit=c.submap_window)

    points = np.vstack(clouds)
    if not track_sources:
        return MeasuredSubmap(
            points_body=voxel_downsample(points, c.voxel_m),
            source_keys=tuple(selected),
            point_sources=(),
        )
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
    point_sources = tuple(tuple(sorted(keys)) for keys in provenance)
    return MeasuredSubmap(
        points_body=sums / counts[:, None],
        source_keys=tuple(selected),
        point_sources=point_sources,
    )
