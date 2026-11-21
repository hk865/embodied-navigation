"""基于实测 LiDAR 支撑点的多特征图像匹配与刚体几何验证模块。

支持五种特征前端：
- ``patch7x7``：纯 numpy/scipy 固定方向 7x7 归一化灰度块（无 OpenCV 依赖基线）
- ``orb`` / ``akaze`` / ``brisk`` / ``sift``：视角与旋转/尺度不变局部特征（按需调用 OpenCV）

本模块要求输入的针孔图像已经过畸变矫正；相机坐标系采用光学系约定
（x 向右、y 向下、z 向前）。验证通过后输出的 SE(3) 变换将源关键帧机体系映射到目标关键帧机体系。
纯图像外观本身不输出无尺度的单目位姿，所有三维几何约束均严格绑定到实测 LiDAR 回波点。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from .pose_graph import invert

ALLOWED_FEATURE_TYPES = ("patch7x7", "orb", "akaze", "brisk", "sift")
_BINARY_FEATURES = frozenset({"orb", "akaze", "brisk"})


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
class VisualFeatures:
    """单帧已提取的局部特征：包含绑定实测 LiDAR 的 3D 支撑点与原始局部描述子。"""

    feature_type: str
    points_body: np.ndarray  # (M, 3) float64，机体系下有实测 LiDAR 支撑的关键点 3D 坐标
    descriptors: np.ndarray  # (M, D)，用于 verify_visual 匹配的描述子
    all_descriptors: np.ndarray  # (K, D)，整幅图像所有关键点描述子，供全局检索聚合使用
    is_binary: bool


@dataclass(frozen=True)
class VisualResult:
    """图像特征匹配与实测 3D 点 RANSAC 刚体估计结果。"""

    status: str
    T_target_source: np.ndarray | None = None
    matches: int = 0
    inliers: int = 0
    rmse_m: float = float("inf")


def _import_cv2(feature_type: str):
    """按需导入 OpenCV，仅在启用视角不变特征时才要求安装 cv2。"""
    try:
        import cv2
    except ImportError as error:
        raise RuntimeError(f"feature_type={feature_type!r} 需要安装 opencv-python (cv2)") from error
    return cv2


def _create_cv_detector(feature_type: str, max_features: int):
    cv2 = _import_cv2(feature_type)
    if feature_type == "orb":
        return cv2.ORB_create(nfeatures=max_features)
    if feature_type == "akaze":
        return cv2.AKAZE_create()
    if feature_type == "brisk":
        return cv2.BRISK_create()
    if feature_type == "sift":
        return cv2.SIFT_create(nfeatures=max_features)
    raise ValueError(f"未知的视觉特征类型: {feature_type!r}")


def _project_lidar_to_image(keyframe) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """将关键帧实测 LiDAR 点投影到图像平面，返回视野内的 (uv, points_body, depth_m)。"""
    from .keyframes import transform_points

    observation = keyframe.visual
    points = keyframe.points_body()
    camera = transform_points(invert(observation.T_body_camera), points)
    front = camera[:, 2] > 0.1
    camera, points = camera[front], points[front]
    if len(camera) == 0:
        return np.empty((0, 2), dtype=float), np.empty((0, 3), dtype=float), np.empty((0,), dtype=float)
    projected = camera @ observation.K.T
    uv = projected[:, :2] / projected[:, 2:]
    h, w = observation.gray.shape
    valid = (uv[:, 0] >= 0.0) & (uv[:, 0] < float(w)) & (uv[:, 1] >= 0.0) & (uv[:, 1] < float(h))
    return uv[valid], points[valid], camera[valid, 2]


def _extract_patch7x7(keyframe, limit: int = 400) -> VisualFeatures:
    """纯 numpy 7x7 归一化图像块提取（保持原始基线与零 OpenCV 依赖）。"""
    observation = keyframe.visual
    uv, points, depth = _project_lidar_to_image(keyframe)
    if len(uv) == 0:
        empty_p = np.empty((0, 3), dtype=float)
        empty_d = np.empty((0, 49), dtype=float)
        return VisualFeatures("patch7x7", empty_p, empty_d, empty_d, False)
    pixels = np.rint(uv).astype(int)
    h, w = observation.gray.shape
    valid = (pixels[:, 0] >= 3) & (pixels[:, 0] < w - 3) & (pixels[:, 1] >= 3) & (pixels[:, 1] < h - 3)
    pixels, points, depth = pixels[valid], points[valid], depth[valid]
    if len(pixels) == 0:
        empty_p = np.empty((0, 3), dtype=float)
        empty_d = np.empty((0, 49), dtype=float)
        return VisualFeatures("patch7x7", empty_p, empty_d, empty_d, False)
    # 在每个 7x7 像素栅格中仅保留深度最近的实测点，抑制遮挡穿透与强相关重复采样。
    order = np.argsort(depth, kind="stable")
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
    descs = patches[strongest] / norm[strongest, None]
    return VisualFeatures("patch7x7", points[selected[strongest]], descs, descs, False)


def extract_visual_features(
    keyframe,
    *,
    feature_type: str = "patch7x7",
    max_features: int = 800,
    lidar_px_radius: float = 6.0,
) -> VisualFeatures:
    """提取指定类型的局部特征并将每个图像关键点绑定到邻域内深度最近的实测 LiDAR 3D 点。"""
    if feature_type not in ALLOWED_FEATURE_TYPES:
        raise ValueError(f"feature_type 必须属于 {ALLOWED_FEATURE_TYPES}，当前收到 {feature_type!r}")
    if feature_type == "patch7x7":
        return _extract_patch7x7(keyframe, limit=min(max_features, 400))

    observation = keyframe.visual
    gray_u8 = np.clip(np.rint(observation.gray), 0, 255).astype(np.uint8)
    detector = _create_cv_detector(feature_type, max_features)
    keypoints, desc = detector.detectAndCompute(gray_u8, None)
    is_binary = feature_type in _BINARY_FEATURES
    if not keypoints or desc is None or len(keypoints) == 0:
        empty_d = np.empty((0, 32), dtype=np.uint8 if is_binary else np.float32)
        return VisualFeatures(feature_type, np.empty((0, 3), dtype=float), empty_d, empty_d, is_binary)

    # 限制保留响应最强的前 max_features 个特征，防止 AKAZE/BRISK 在高纹理画面输出过多特征。
    if len(keypoints) > max_features:
        order = np.argsort([-kp.response for kp in keypoints], kind="stable")[:max_features]
        keypoints = [keypoints[i] for i in order]
        desc = desc[order]

    uv, points, depth = _project_lidar_to_image(keyframe)
    if len(uv) == 0:
        empty_d = np.empty((0, desc.shape[1]), dtype=desc.dtype)
        return VisualFeatures(feature_type, np.empty((0, 3), dtype=float), empty_d, desc, is_binary)

    kp_uv = np.array([kp.pt for kp in keypoints], dtype=float)
    tree = cKDTree(uv)
    neighbors_list = tree.query_ball_point(kp_uv, r=lidar_px_radius)
    pts3d: list[np.ndarray] = []
    keep_idx: list[int] = []
    for i, nbrs in enumerate(neighbors_list):
        if not nbrs:
            continue
        # 仅保留邻域内深度最近的实测 LiDAR 回波点，避免前景边缘误绑定背景墙体深度。
        nbrs_arr = np.asarray(nbrs, dtype=np.intp)
        nearest_j = nbrs_arr[int(np.argmin(depth[nbrs_arr]))]
        pts3d.append(points[nearest_j])
        keep_idx.append(i)
    if not keep_idx:
        empty_d = np.empty((0, desc.shape[1]), dtype=desc.dtype)
        return VisualFeatures(feature_type, np.empty((0, 3), dtype=float), empty_d, desc, is_binary)
    return VisualFeatures(
        feature_type=feature_type,
        points_body=np.asarray(pts3d, dtype=float),
        descriptors=desc[keep_idx],
        all_descriptors=desc,
        is_binary=is_binary,
    )


def _rigid(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """使用 SVD（Kabsch 算法）求解两组三维对应点的最小二乘刚体变换。"""
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - a).T @ (target - b))
    rotation = vt.T @ np.diag([1.0, 1.0, np.linalg.det(vt.T @ u.T)]) @ u.T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rotation, b - rotation @ a
    return T


def _match_descriptors(
    fa: VisualFeatures,
    fb: VisualFeatures,
    ratio_threshold: float,
) -> tuple[np.ndarray, np.ndarray]:
    """双向比率测试匹配：返回成对的实测三维点 (points_source, points_target)。"""
    if fa.feature_type == "patch7x7":
        distances, indices = cKDTree(fb.descriptors).query(fa.descriptors, k=2)
        reverse = cKDTree(fa.descriptors).query(fb.descriptors, k=1)[1]
        valid = (
            (distances[:, 0] < ratio_threshold * distances[:, 1])
            & (distances[:, 0] < 0.7)
            & (reverse[indices[:, 0]] == np.arange(len(fa.points_body)))
        )
        return fa.points_body[valid], fb.points_body[indices[valid, 0]]

    cv2 = _import_cv2(fa.feature_type)
    norm_type = cv2.NORM_HAMMING if fa.is_binary else cv2.NORM_L2
    bf = cv2.BFMatcher(norm_type, crossCheck=False)
    fwd = bf.knnMatch(fa.descriptors, fb.descriptors, k=2)
    bwd = bf.knnMatch(fb.descriptors, fa.descriptors, k=1)
    rev_best = np.full(len(fb.points_body), -1, dtype=np.intp)
    for pair in bwd:
        if pair:
            rev_best[pair[0].queryIdx] = pair[0].trainIdx

    src_pts, dst_pts = [], []
    for pair in fwd:
        if len(pair) < 2:
            continue
        m1, m2 = pair[0], pair[1]
        if m1.distance < ratio_threshold * m2.distance and rev_best[m1.trainIdx] == m1.queryIdx:
            src_pts.append(fa.points_body[m1.queryIdx])
            dst_pts.append(fb.points_body[m1.trainIdx])
    if not src_pts:
        return np.empty((0, 3), dtype=float), np.empty((0, 3), dtype=float)
    return np.asarray(src_pts, dtype=float), np.asarray(dst_pts, dtype=float)


def verify_visual(
    source,
    target,
    *,
    feature_type: str = "patch7x7",
    max_time_delta_s: float = 0.05,
    inlier_m: float = 0.12,
    min_inliers: int = 8,
    ratio_threshold: float = 0.7,
    max_features: int = 800,
    lidar_px_radius: float = 6.0,
    source_features: VisualFeatures | None = None,
    target_features: VisualFeatures | None = None,
) -> VisualResult:
    """双向比率测试图像特征匹配 + 实测三维点刚体 RANSAC 校验。

    缺失观测、时间不同步、弱纹理、共线退化或内点不足时全部拒绝，不输出猜测位姿。
    """
    from .keyframes import transform_points

    if source.visual is None or target.visual is None:
        return VisualResult("missing_observation")
    if any(abs(k.visual.stamp_ns - k.stamp_ns) > max_time_delta_s * 1e9 for k in (source, target)):
        return VisualResult("time_mismatch")

    fa = (
        source_features
        if source_features is not None and source_features.feature_type == feature_type
        else extract_visual_features(
            source, feature_type=feature_type, max_features=max_features, lidar_px_radius=lidar_px_radius
        )
    )
    fb = (
        target_features
        if target_features is not None and target_features.feature_type == feature_type
        else extract_visual_features(
            target, feature_type=feature_type, max_features=max_features, lidar_px_radius=lidar_px_radius
        )
    )
    if min(len(fa.points_body), len(fb.points_body)) < min_inliers:
        return VisualResult("insufficient_texture")

    a, b = _match_descriptors(fa, fb, ratio_threshold=ratio_threshold)
    if len(a) < min_inliers:
        return VisualResult("insufficient_matches", matches=len(a))

    rng = np.random.default_rng(0)
    best = np.zeros(len(a), dtype=bool)
    iterations = 128 if feature_type == "patch7x7" else 200
    for _ in range(iterations):
        sample = rng.choice(len(a), 3, replace=False)
        # 剔除近似共线的三点采样，避免刚体旋转轴退化。
        if np.linalg.svd(a[sample] - a[sample].mean(axis=0), compute_uv=False)[1] < 0.02:
            continue
        T = _rigid(a[sample], b[sample])
        support = np.linalg.norm(transform_points(T, a) - b, axis=1) < inlier_m
        if support.sum() > best.sum():
            best = support

    if int(best.sum()) < 3:
        return VisualResult("geometry_rejected", matches=len(a), inliers=int(best.sum()))
    T = _rigid(a[best], b[best])
    final_mask = np.linalg.norm(transform_points(T, a) - b, axis=1) < inlier_m
    if int(final_mask.sum()) < min_inliers:
        return VisualResult("geometry_rejected", matches=len(a), inliers=int(final_mask.sum()))
    # Deduplicate by 1 mm quantized 3D correspondence pair so multiple pixel features
    # bound to the same LiDAR return cannot inflate inlier count or geometric spread.
    a_in = a[final_mask]
    b_in = b[final_mask]
    _, uniq_idx = np.unique(
        np.round(np.hstack([a_in, b_in]) * 1000.0).astype(np.int64), axis=0, return_index=True
    )
    a_uniq = a_in[uniq_idx]
    b_uniq = b_in[uniq_idx]
    inlier_count = int(len(a_uniq))
    inlier_ratio = float(inlier_count) / float(len(a))
    # patch7x7 维持原 50% 内点率；多尺度视角不变特征在宽视角下允许 >=25% 或内点总数 >= 2*min_inliers。
    ratio_ok = (inlier_ratio >= 0.5) if feature_type == "patch7x7" else (
        inlier_ratio >= 0.25 or inlier_count >= 2 * min_inliers
    )
    if inlier_count < min_inliers or not ratio_ok:
        return VisualResult("geometry_rejected", matches=len(a), inliers=inlier_count)
    if min(np.linalg.svd(p - p.mean(axis=0), compute_uv=False)[1] for p in (a_uniq, b_uniq)) < 0.1:
        return VisualResult("degenerate_geometry", matches=len(a), inliers=inlier_count)
    T = _rigid(a_uniq, b_uniq)
    residual = np.linalg.norm(transform_points(T, a_uniq) - b_uniq, axis=1)
    if int(np.sum(residual < inlier_m)) < min_inliers:
        return VisualResult("geometry_rejected", matches=len(a), inliers=int(np.sum(residual < inlier_m)))
    return VisualResult("verified", T, len(a), inlier_count, float(np.sqrt(np.mean(residual**2))))


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
