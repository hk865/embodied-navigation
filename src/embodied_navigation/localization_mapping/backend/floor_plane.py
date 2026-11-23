"""宽跨度水平地面平面检测与相邻帧共面里程计调平模块。

核心物理依据：
- 四足机器人（如 Unitree Go2）正常行走时存在约 6°~8° 机身俯仰角，直接按原始 ``z_body`` 切片
  会把倾斜的地面点云切残，引入 3°~4° 法向偏差。
- 仅用粗姿态 ``R_odom_body``（或后端重力稳定后的 ``R_floor_hint``）做近中距 4 cm 高度直方图
  寻找地面主峰种子，再在完整 body 系点云 ``[min_radius_m, max_radius_m]`` 上迭代 PCA 扩展拟合，
  实测地面法向精度可达 ~0.01°~0.04°（比带噪 IMU 姿态高一个量级），帧间高度精度 ~3 mm。
- 相邻两帧同楼层共面关键帧 ``(a, b)`` 沿 ``n_a`` 方向的真实位移严格等于 ``h_b - h_a``；仅投影
  消除沿 ``n_a`` 的垂直残余漂移、完全保留旋转与水平运动，即可无楼层 ID 启发式地消除平地开环 Z 漂移。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class FloorPlaneMeasurement:
    """机体坐标系下的水平地面测量结果（平面方程 ``n · p + height_below_body_m = 0``）。"""

    height_below_body_m: float
    normal_body: np.ndarray
    residual_mad_m: float
    point_count: int
    span_min_m: float
    span_max_m: float
    inlier_sample_body: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float64))


def _fit_plane_pca(points: np.ndarray) -> tuple[np.ndarray, float] | None:
    """用 PCA 拟合平面 ``n · p + d = 0``（保证 ``n[2] > 0``），失败返回 None。"""
    if len(points) < 3:
        return None
    mean = points.mean(axis=0)
    centered = points - mean
    eigvals, eigvecs = np.linalg.eigh(centered.T @ centered)
    if not np.isfinite(eigvals).all() or eigvals[1] <= 1e-8:
        return None
    normal = eigvecs[:, 0].copy()
    if normal[2] < 0.0:
        normal = -normal
    norm = float(np.linalg.norm(normal))
    if norm <= 1e-9 or not np.isfinite(norm):
        return None
    normal /= norm
    d_plane = -float(normal @ mean)
    return normal, d_plane


def detect_floor_plane(
    points_body: np.ndarray,
    R_odom_body: np.ndarray,
    *,
    max_body_tilt_deg: float = 14.0,
    max_plane_world_tilt_deg: float = 6.0,
    min_radius_m: float = 1.0,
    max_radius_m: float = 15.0,
    min_z_level_m: float = -0.70,
    max_z_level_m: float = -0.15,
    min_points: int = 80,
    max_mad_m: float = 0.015,
    min_span_short_m: float = 1.8,
    min_span_long_m: float = 3.5,
) -> FloorPlaneMeasurement | None:
    """在机体坐标系中检测机器人脚下的宽跨度水平地面。

    步骤：
    1. 使用 ``R_odom_body``（或后端图稳定姿态 ``R_floor_hint``）把点云粗调平；
    2. 在近中距（``r <= 5 m``，倾角杠杆最小）做 4 cm 高度直方图取主峰确定性种子；
    3. 在完整宽倾角候选集（``[min_radius_m, max_radius_m]``）中做 4 轮 PCA 内点扩展，
       防止远场地面点被粗姿态倾角误切；
    4. 通过高度范围、水平倾角、残差 MAD 与旋转不变 2D 主轴跨度四道几何门控。
    """
    R = np.asarray(R_odom_body, dtype=np.float64)
    if R.shape == (4, 4):
        R = R[:3, :3]
    if R.shape != (3, 3) or not np.isfinite(R).all():
        return None
    if float(R[2, 2]) < math.cos(math.radians(max_body_tilt_deg)):
        return None

    pts = np.asarray(points_body, dtype=np.float64).reshape(-1, 3)
    if len(pts) < min_points:
        return None
    finite = np.all(np.isfinite(pts), axis=1)
    if not np.all(finite):
        pts = pts[finite]
        if len(pts) < min_points:
            return None

    p_lvl = pts @ R.T
    r_xy = np.hypot(p_lvl[:, 0], p_lvl[:, 1])
    min_h = max(0.0, -float(max_z_level_m))
    max_h = max(min_h + 1e-3, -float(min_z_level_m))
    cos_world_lim = math.cos(math.radians(max_plane_world_tilt_deg))

    # 1. 近中距（r <= 5 m）确定性高度直方图种子，最小化粗姿态残余倾角杠杆。
    seed_pool = np.flatnonzero(
        (r_xy >= min_radius_m)
        & (r_xy <= min(max_radius_m, 5.0))
        & (p_lvl[:, 2] >= min_z_level_m - 0.15)
        & (p_lvl[:, 2] <= max_z_level_m + 0.10)
    )
    if len(seed_pool) < max(30, min_points // 2):
        seed_pool = np.flatnonzero(
            (r_xy >= min_radius_m)
            & (r_xy <= max_radius_m)
            & (p_lvl[:, 2] >= min_z_level_m)
            & (p_lvl[:, 2] <= max_z_level_m)
        )
    if len(seed_pool) < max(30, min_points // 2):
        return None

    z_seed = p_lvl[seed_pool, 2]
    bins = np.floor(z_seed / 0.04).astype(np.int64)
    uniq, counts = np.unique(bins, return_counts=True)
    z_mode = (float(uniq[int(np.argmax(counts))]) + 0.5) * 0.04
    seed_mask = np.abs(z_seed - z_mode) <= 0.08
    if int(np.sum(seed_mask)) < max(25, min_points // 3):
        return None

    fit1 = _fit_plane_pca(pts[seed_pool[seed_mask]])
    if fit1 is None:
        return None
    normal, d_plane = fit1

    # 2. 在宽倾角全域候选集上迭代扩展内点并重拟合，避免粗姿态微倾裁掉远场真实地面。
    cand_idx = np.flatnonzero(
        (r_xy >= min_radius_m)
        & (r_xy <= max_radius_m)
        & (p_lvl[:, 2] >= min_z_level_m - 1.1)
        & (p_lvl[:, 2] <= max_z_level_m + 0.75)
    )
    if len(cand_idx) < min_points:
        return None
    cand_b = pts[cand_idx]

    inliers_b = cand_b
    for _ in range(4):
        inlier_mask = np.abs(cand_b @ normal + d_plane) <= 0.025
        inliers_b = cand_b[inlier_mask]
        if len(inliers_b) < min_points:
            return None
        fit = _fit_plane_pca(inliers_b)
        if fit is None:
            return None
        normal, d_plane = fit

    inlier_mask = np.abs(cand_b @ normal + d_plane) <= 0.025
    inliers_b = cand_b[inlier_mask]
    if len(inliers_b) < min_points:
        return None

    # 3. 高度、水平法向夹角与平面度 MAD 校验。
    if d_plane < min_h or d_plane > max_h:
        return None
    n_lvl = R @ normal
    if float(n_lvl[2]) < cos_world_lim:
        return None

    signed_residuals = inliers_b @ normal + d_plane
    mad_m = float(np.median(np.abs(signed_residuals)))
    if not np.isfinite(mad_m) or mad_m > max_mad_m:
        return None

    # 4. 计算平面内的旋转不变 2D PCA 短/长主轴跨度，过滤窄台阶与桌面误检。
    centered = inliers_b - inliers_b.mean(axis=0)
    ref = (
        np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(float(normal[0])) < 0.9
        else np.array([0.0, 1.0, 0.0], dtype=np.float64)
    )
    u_axis = np.cross(normal, ref)
    u_axis /= float(np.linalg.norm(u_axis))
    v_axis = np.cross(normal, u_axis)
    coords_2d = np.column_stack([centered @ u_axis, centered @ v_axis])
    _, eigvecs_2d = np.linalg.eigh(coords_2d.T @ coords_2d)
    pc_coords = coords_2d @ eigvecs_2d
    span_short = float(pc_coords[:, 0].max() - pc_coords[:, 0].min())
    span_long = float(pc_coords[:, 1].max() - pc_coords[:, 1].min())
    if span_short < min_span_short_m or span_long < min_span_long_m:
        return None

    height_below_body_m = float(d_plane)
    if not np.isfinite(height_below_body_m) or height_below_body_m <= 0.0:
        return None

    n_inliers = len(inliers_b)
    if n_inliers <= 48:
        inlier_sample = inliers_b.copy()
    else:
        sample_idx = np.linspace(0, n_inliers - 1, 48, dtype=np.intp)
        inlier_sample = inliers_b[sample_idx].copy()

    return FloorPlaneMeasurement(
        height_below_body_m=height_below_body_m,
        normal_body=normal,
        residual_mad_m=mad_m,
        point_count=int(n_inliers),
        span_min_m=span_short,
        span_max_m=span_long,
        inlier_sample_body=inlier_sample,
    )


def project_coplanar_odom_delta(
    delta_odom: np.ndarray,
    prev_floor: FloorPlaneMeasurement,
    cur_floor: FloorPlaneMeasurement,
    *,
    max_step_dz_m: float = 0.06,
    max_normal_disagreement_deg: float = 3.0,
) -> tuple[np.ndarray, bool]:
    """用相邻两帧实测共面地面几何对平地里程计边进行垂直调平。

    当 ``prev_floor`` 与 ``cur_floor`` 观测同一水平楼板时，``delta_odom[:3, 3]``
    沿 ``prev_floor.normal_body`` 的真实分量应等于 ``h_b - h_a``。若高度差超限、
    法向旋转不一致或变换后地面点残差大于 4 cm（例如上下楼梯换板），则安全跳过。
    返回 ``(delta_out, applied)``，绝不原地修改 ``delta_odom``。
    """
    R = np.array(delta_odom[:3, :3], dtype=np.float64, copy=True)
    t = np.array(delta_odom[:3, 3], dtype=np.float64, copy=True)
    n_a = prev_floor.normal_body
    n_b = cur_floor.normal_body

    # 校验两帧高度变化一致性，拒绝楼梯台阶或断层。
    dz_odom = float(n_a @ t)
    dz_floor = float(cur_floor.height_below_body_m - prev_floor.height_below_body_m)
    if abs(dz_odom - dz_floor) > max_step_dz_m:
        return delta_odom, False

    # 校验相对旋转映射后的法向一致性，拒绝斜坡/上下楼过渡段。
    n_b_in_a = R @ n_b
    cos_ang = float(np.clip(n_a @ n_b_in_a, -1.0, 1.0))
    if cos_ang < math.cos(math.radians(max_normal_disagreement_deg)):
        return delta_odom, False

    # 校验当前帧地面采样点经里程计变换后到前帧平面的直接距离。
    cur_sample = np.asarray(cur_floor.inlier_sample_body, dtype=np.float64).reshape(-1, 3)
    if len(cur_sample) == 0:
        return delta_odom, False
    pts_in_prev = cur_sample @ R.T + t
    med_plane_dist = float(np.median(np.abs(pts_in_prev @ n_a + prev_floor.height_below_body_m)))
    if not np.isfinite(med_plane_dist) or med_plane_dist > 0.04:
        return delta_odom, False

    # 仅投影消除沿 n_a 的垂直漂移，完整保留旋转 R 以免扰动 yaw。
    t_clean = t + (dz_floor - dz_odom) * n_a

    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = R
    out[:3, 3] = t_clean
    return out, True
