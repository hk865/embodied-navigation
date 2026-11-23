"""法向定向 3D 平面光束平差（BALM2 点簇充分统计量 + Matrix-Free 隐式 Schur 补 PCG）。

将多帧 3D 平面一致性约束与位姿图边、校准重力先验、Z 高度先验放入同一个稀疏
Levenberg-Marquardt 系统中联合求解。每帧体素平面面片的法向在聚类前强制朝向
观测传感器原点（``n · (-ray) > 0``），使跨楼层薄楼板两面（地下室天花板与二楼
地板）以及薄隔墙两面法向相反（点积 ≈ -1），从数学上杜绝异面误合并与压扁。
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import time

import numpy as np

from .accel import cpp_run_oriented_planar_ba
from .pose_graph import PoseGraph


@dataclass(frozen=True)
class PlanarBaResult:
    """单次法向定向 3D 平面 BA 求解结果摘要。"""

    poses: list[np.ndarray]
    converged: bool
    clusters_used: int
    obs_used: int
    accepted_steps: int
    error_before: float
    error_after: float
    wall_s: float
    used_cpp: bool


def refine_poses_oriented_planar_ba(
    graph: PoseGraph,
    points_body_list: list[np.ndarray],
    *,
    voxel_m: float = 1.0,
    min_frame_pts: int = 8,
    plane_ratio_max: float = 0.08,
    plane_thickness_max_m: float = 0.08,
    min_normal_cos: float = 0.85,
    max_plane_offset_m: float = 0.15,
    max_frames_per_cluster: int = 20,
    plane_sigma_m: float = 0.03,
    cauchy_scale: float = 10.0,
    outer_rounds: int = 2,
    inner_iterations: int = 8,
) -> PlanarBaResult:
    """对完整 ``PoseGraph`` 执行全局法向定向 3D 平面 BA。

    首节点（索引 0）保持锚定不动；完整保留位姿图中的里程计边、回环边、校准重力先验
    与 Z 高度先验，确保平面精化不会破坏全局尺度或丢弃既有图约束。
    """
    t0 = time.perf_counter()
    n_nodes = len(graph.poses)
    if n_nodes != len(points_body_list):
        raise ValueError(
            f"points_body_list length ({len(points_body_list)}) must match graph nodes ({n_nodes})"
        )
    if n_nodes <= 1:
        return PlanarBaResult(
            poses=[p.copy() for p in graph.poses],
            converged=True,
            clusters_used=0,
            obs_used=0,
            accepted_steps=0,
            error_before=0.0,
            error_after=0.0,
            wall_s=float(time.perf_counter() - t0),
            used_cpp=False,
        )

    result = cpp_run_oriented_planar_ba(
        graph.poses,
        points_body_list,
        graph.edges,
        graph.gravity_priors,
        graph.z_priors,
        n_fixed=1,
        gravity_world=graph.gravity_world,
        ba_voxel_m=voxel_m,
        min_frame_pts=min_frame_pts,
        plane_ratio_max=plane_ratio_max,
        plane_thickness_max_m=plane_thickness_max_m,
        min_normal_cos=min_normal_cos,
        max_plane_offset_m=max_plane_offset_m,
        max_frames_per_cluster=max_frames_per_cluster,
        plane_sigma_m=plane_sigma_m,
        cauchy_scale=cauchy_scale,
        outer_rounds=outer_rounds,
        inner_iterations=inner_iterations,
    )
    wall_s = float(time.perf_counter() - t0)
    if result is None:
        return PlanarBaResult(
            poses=[p.copy() for p in graph.poses],
            converged=False,
            clusters_used=0,
            obs_used=0,
            accepted_steps=0,
            error_before=0.0,
            error_after=0.0,
            wall_s=wall_s,
            used_cpp=False,
        )
    updated, info = result
    return PlanarBaResult(
        poses=updated,
        converged=bool(info["converged"]),
        clusters_used=int(info["clusters_used"]),
        obs_used=int(info["obs_used"]),
        accepted_steps=int(info["accepted_steps"]),
        error_before=float(info["error_before"]),
        error_after=float(info["error_after"]),
        wall_s=wall_s,
        used_cpp=True,
    )


def refine_window_oriented_planar_ba(
    graph: PoseGraph,
    points_body_source: list[np.ndarray] | Callable[[int], np.ndarray],
    window_size: int,
    *,
    voxel_m: float = 1.0,
    min_frame_pts: int = 8,
    plane_ratio_max: float = 0.08,
    plane_thickness_max_m: float = 0.08,
    min_normal_cos: float = 0.85,
    max_plane_offset_m: float = 0.15,
    max_frames_per_cluster: int = 20,
    plane_sigma_m: float = 0.03,
    cauchy_scale: float = 10.0,
    outer_rounds: int = 1,
    inner_iterations: int = 6,
) -> tuple[dict[int, np.ndarray], PlanarBaResult]:
    """对最新 ``window_size`` 个节点执行低延迟在线滑动窗口平面 BA。

    窗首之前的边界锚帧（最多 4 帧）以及所有与窗口存在位姿图边相连的历史节点均作为
    固定锚点（``n_fixed``）纳入局部子图：
    - 保留全部跨窗回环边与自由节点 Z 先验，不撕裂已有全局闭环；
    - 仅按需提取子图节点的 body 点云，避免每次滑窗遍历整张历史地图。
    """
    t0 = time.perf_counter()
    n_nodes = len(graph.poses)
    if not callable(points_body_source) and n_nodes != len(points_body_source):
        raise ValueError(
            f"points_body_list length ({len(points_body_source)}) must match graph nodes ({n_nodes})"
        )
    if n_nodes <= 2 or window_size <= 1:
        return {}, PlanarBaResult(
            poses=[],
            converged=True,
            clusters_used=0,
            obs_used=0,
            accepted_steps=0,
            error_before=0.0,
            error_after=0.0,
            wall_s=float(time.perf_counter() - t0),
            used_cpp=False,
        )

    start_idx = max(0, n_nodes - int(window_size))
    free_global = list(range(start_idx + 1, n_nodes))
    if not free_global:
        return {}, PlanarBaResult(
            poses=[],
            converged=True,
            clusters_used=0,
            obs_used=0,
            accepted_steps=0,
            error_before=0.0,
            error_after=0.0,
            wall_s=float(time.perf_counter() - t0),
            used_cpp=False,
        )
    free_set = set(free_global)
    # 固定窗首及紧邻的前 3 帧提供平面锚定，同时纳入与自由帧相连的历史回环邻居作为固定边界。
    fixed_set = set(range(max(0, start_idx - 3), start_idx + 1))
    for a, b, _z, _info, _rob in graph.edges:
        if a in free_set and b not in free_set:
            fixed_set.add(b)
        elif b in free_set and a not in free_set:
            fixed_set.add(a)

    fixed_global = sorted(fixed_set)
    ordered_global = fixed_global + free_global
    g2l = {g: l for l, g in enumerate(ordered_global)}
    n_fixed = len(fixed_global)

    sub_poses = [graph.poses[g] for g in ordered_global]
    if callable(points_body_source):
        sub_pts = [points_body_source(g) for g in ordered_global]
    else:
        sub_pts = [points_body_source[g] for g in ordered_global]
    sub_edges = [
        (g2l[a], g2l[b], Z_inv, info, rob)
        for a, b, Z_inv, info, rob in graph.edges
        if (a in g2l and b in g2l) and (a in free_set or b in free_set)
    ]
    sub_grav = [
        (g2l[ni], g_body, weight, rob)
        for ni, g_body, weight, rob in graph.gravity_priors
        if ni in free_set
    ]
    sub_zp = [
        (g2l[ni], target_z, weight, rob)
        for ni, target_z, weight, rob in graph.z_priors
        if ni in free_set
    ]

    result = cpp_run_oriented_planar_ba(
        sub_poses,
        sub_pts,
        sub_edges,
        sub_grav,
        sub_zp,
        n_fixed=n_fixed,
        gravity_world=graph.gravity_world,
        ba_voxel_m=voxel_m,
        min_frame_pts=min_frame_pts,
        plane_ratio_max=plane_ratio_max,
        plane_thickness_max_m=plane_thickness_max_m,
        min_normal_cos=min_normal_cos,
        max_plane_offset_m=max_plane_offset_m,
        max_frames_per_cluster=max_frames_per_cluster,
        plane_sigma_m=plane_sigma_m,
        cauchy_scale=cauchy_scale,
        outer_rounds=outer_rounds,
        inner_iterations=inner_iterations,
    )
    wall_s = float(time.perf_counter() - t0)
    if result is None:
        return {}, PlanarBaResult(
            poses=[],
            converged=False,
            clusters_used=0,
            obs_used=0,
            accepted_steps=0,
            error_before=0.0,
            error_after=0.0,
            wall_s=wall_s,
            used_cpp=False,
        )
    updated_local, info = result
    updated_map = {g: updated_local[g2l[g]] for g in free_global}
    return updated_map, PlanarBaResult(
        poses=updated_local,
        converged=bool(info["converged"]),
        clusters_used=int(info["clusters_used"]),
        obs_used=int(info["obs_used"]),
        accepted_steps=int(info["accepted_steps"]),
        error_before=float(info["error_before"]),
        error_after=float(info["error_after"]),
        wall_s=wall_s,
        used_cpp=True,
    )
