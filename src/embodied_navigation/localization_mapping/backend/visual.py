"""基于实测 LiDAR 支撑点的图像块匹配与刚体几何验证模块。

本模块要求输入的针孔图像已经过畸变矫正；相机坐标系采用光学系约定
（x 向右、y 向下、z 向前）。验证通过后输出的 SE(3) 变换将源关键帧机体系映射到目标关键帧机体系。
纯图像外观本身不输出无尺度的单目位姿，所有三维几何约束均严格绑定到实测 LiDAR 回波点。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from .pose_graph import invert


@dataclass(frozen=True)
class VisualObservation:
    """单帧关键帧携带的灰度图、相机内参、雷达到相机的外参及时间戳。"""

    gray: np.ndarray
    K: np.ndarray
    T_body_camera: np.ndarray
    stamp_ns: int

    def __post_init__(self) -> None:
        # 外部 ROS 消息或磁盘文件传入的数组必须先校验合法性，防止非法内参/外参污染后续投影。
        if self.gray.ndim != 2 or not np.isfinite(self.gray).all():
            raise ValueError("视觉图像必须为有限值的二维已矫正灰度图")
        if isinstance(self.stamp_ns, bool) or not isinstance(self.stamp_ns, (int, np.integer)):
            raise ValueError(f"视觉时间戳 stamp_ns 必须为整数标量，当前收到 {self.stamp_ns!r}")
        if (
            self.K.shape != (3, 3)
            or not np.isfinite(self.K).all()
            or self.K[0, 0] <= 0
            or self.K[1, 1] <= 0
            or not np.allclose(self.K[2], [0.0, 0.0, 1.0], atol=1e-6)
        ):
            raise ValueError("相机内参矩阵 K 非法（须满足 fx>0, fy>0 且末行为 [0, 0, 1]）")
        T = self.T_body_camera
        if (
            T.shape != (4, 4)
            or not np.isfinite(T).all()
            or not np.allclose(T[3], [0.0, 0.0, 0.0, 1.0])
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-5)
            or np.linalg.det(T[:3, :3]) < 0
        ):
            raise ValueError("相机外参矩阵 T_body_camera 非法")


@dataclass(frozen=True)
class VisualResult:
    """图像块匹配与实测 3D 点 RANSAC 刚体估计结果。"""

    status: str
    T_target_source: np.ndarray | None = None
    matches: int = 0
    inliers: int = 0
    rmse_m: float = float("inf")


def _features(keyframe, limit: int = 400) -> tuple[np.ndarray, np.ndarray]:
    """将关键帧实测 LiDAR 点投影到灰度图，提取最近深度且有纹理的 7x7 归一化图像块。"""
    from .keyframes import transform_points

    observation = keyframe.visual
    points = keyframe.points_body()
    camera = transform_points(invert(observation.T_body_camera), points)
    front = camera[:, 2] > 0.1
    camera, points = camera[front], points[front]
    if len(camera) == 0:
        return np.empty((0, 3), dtype=float), np.empty((0, 49), dtype=float)
    projected = camera @ observation.K.T
    pixels = np.rint(projected[:, :2] / projected[:, 2:]).astype(int)
    h, w = observation.gray.shape
    valid = (pixels[:, 0] >= 3) & (pixels[:, 0] < w - 3) & (pixels[:, 1] >= 3) & (pixels[:, 1] < h - 3)
    pixels, points, camera = pixels[valid], points[valid], camera[valid]
    if len(pixels) == 0:
        return np.empty((0, 3), dtype=float), np.empty((0, 49), dtype=float)
    # 在每个 7x7 像素栅格中仅保留深度最近的实测点，抑制遮挡穿透与强相关重复采样。
    order = np.argsort(camera[:, 2], kind="stable")
    _, first = np.unique(pixels[order] // 7, axis=0, return_index=True)
    selected = order[first]
    patches = np.array(
        [observation.gray[y - 3 : y + 4, x - 3 : x + 4].ravel() for x, y in pixels[selected]],
        dtype=float,
    ).reshape(-1, 49)
    patches -= patches.mean(axis=1, keepdims=True)
    norm = np.linalg.norm(patches, axis=1)
    textured = norm > 10.0
    selected, patches, norm = selected[textured], patches[textured], norm[textured]
    strongest = np.argsort(-norm, kind="stable")[:limit]
    return points[selected[strongest]], patches[strongest] / norm[strongest, None]


def _rigid(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """使用 SVD（Kabsch 算法）求解两组三维对应点的最小二乘刚体变换。"""
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - a).T @ (target - b))
    rotation = vt.T @ np.diag([1.0, 1.0, np.linalg.det(vt.T @ u.T)]) @ u.T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rotation, b - rotation @ a
    return T


def verify_visual(
    source,
    target,
    *,
    max_time_delta_s: float = 0.05,
    inlier_m: float = 0.12,
    min_inliers: int = 8,
) -> VisualResult:
    """双向比率测试图像块匹配 + 实测三维点刚体 RANSAC 校验。

    缺失观测、时间不同步、弱纹理、共线退化或内点不足时全部拒绝，不输出猜测位姿。
    """
    from .keyframes import transform_points

    if source.visual is None or target.visual is None:
        return VisualResult("missing_observation")
    if any(abs(k.visual.stamp_ns - k.stamp_ns) > max_time_delta_s * 1e9 for k in (source, target)):
        return VisualResult("time_mismatch")
    a, da = _features(source)
    b, db = _features(target)
    if min(len(a), len(b)) < min_inliers:
        return VisualResult("insufficient_texture")
    distances, indices = cKDTree(db).query(da, k=2)
    reverse = cKDTree(da).query(db, k=1)[1]
    valid = (
        (distances[:, 0] < 0.7 * distances[:, 1])
        & (distances[:, 0] < 0.7)
        & (reverse[indices[:, 0]] == np.arange(len(a)))
    )
    a, b = a[valid], b[indices[valid, 0]]
    if len(a) < min_inliers:
        return VisualResult("insufficient_matches", matches=len(a))
    rng = np.random.default_rng(0)
    best = np.zeros(len(a), dtype=bool)
    for _ in range(128):
        sample = rng.choice(len(a), 3, replace=False)
        # 剔除近似共线的三点采样，避免刚体旋转轴退化。
        if np.linalg.svd(a[sample] - a[sample].mean(axis=0), compute_uv=False)[1] < 0.02:
            continue
        T = _rigid(a[sample], b[sample])
        support = np.linalg.norm(transform_points(T, a) - b, axis=1) < inlier_m
        if support.sum() > best.sum():
            best = support
    if best.sum() < min_inliers or best.mean() < 0.5:
        return VisualResult("geometry_rejected", matches=len(a), inliers=int(best.sum()))
    if min(np.linalg.svd(p[best] - p[best].mean(axis=0), compute_uv=False)[1] for p in (a, b)) < 0.1:
        return VisualResult("degenerate_geometry", matches=len(a), inliers=int(best.sum()))
    T = _rigid(a[best], b[best])
    residual = np.linalg.norm(transform_points(T, a[best]) - b[best], axis=1)
    return VisualResult("verified", T, len(a), int(best.sum()), float(np.sqrt(np.mean(residual**2))))


def decode_image(message) -> np.ndarray:
    """将 ROS Image（mono8/rgb8/bgr8，含行对齐填充）解码为二维浮点/无符号灰度图，不依赖 cv_bridge。"""
    channels = {"mono8": 1, "rgb8": 3, "bgr8": 3}.get(message.encoding)
    if channels is None:
        raise ValueError("视觉输入仅支持已矫正的 mono8/rgb8/bgr8 编码")
    data = np.frombuffer(message.data, dtype=np.uint8)
    if message.step < message.width * channels or len(data) != message.height * message.step:
        raise ValueError("视觉图像行步长（step）或数据字节数非法")
    pixels = data.reshape(message.height, message.step)[:, : message.width * channels]
    if channels == 1:
        return pixels.copy()
    rgb = pixels.reshape(message.height, message.width, 3)
    if message.encoding == "bgr8":
        rgb = rgb[:, :, ::-1]
    return rgb @ np.array([0.299, 0.587, 0.114])
