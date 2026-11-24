"""基于 V4 后端全局地图与多视角 RGB 的物体级 3D Surface Nets Mesh 重建与关键帧锚定记忆库。

核心流程（纯 NumPy / SciPy / OpenCV，零重型外部 3D 库依赖）：
1. 从 V4 ``MapStore`` 加载已全局 BA 对齐的关键帧点云，结合传感器观测射线构建有向法向全局点云；
2. 分楼层分块（Tile-based）剔除地板、天花板与大面积结构墙体，精准分离前景室内物体；
3. 对前景物体执行 3D 连通分量实例分割，并把每个物体锚定到最优观测关键帧 ``anchor_kf_key``
   （后续 SLAM 回环更新位姿时，物体位姿与 Mesh 可刚性重投影同步，绝不退化为静态孤岛）；
4. 为每个独立物体在局部有向 TSDF 体素网格上运行向量化 **Surface Nets** 提取带法向三角形网格，
   结合标定相机内外参 + Z-Buffer 遮挡检测完成多视角 RGB 顶点着色与最优视角裁剪图保存；
5. 持久化为标准 ``.ply`` 彩色网格 + 结构化 ``memory_index.json``，提供空间/楼层/属性记忆检索接口。
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time
from typing import Any

import numpy as np
from scipy.spatial import cKDTree

from embodied_navigation.localization_mapping.backend.pose_graph import invert


@dataclass(frozen=True)
class CameraCalibration:
    """针孔相机内参与 IMU/Body 到相机光学系的外参（p_cam = R_cam_body @ p_body + t_cam_body）。"""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    R_cam_body: np.ndarray  # (3, 3)
    t_cam_body: np.ndarray  # (3,)

    @classmethod
    def from_preflight_json(cls, path: Path) -> "CameraCalibration":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        cal = data.get("camera_calibration", data)
        sf = cal.get("selected_fields", {})
        params = cal.get("parameters", {})
        intr = sf.get("camera_intrinsics")
        if intr and len(intr) == 4:
            fx, fy, cx, cy = map(float, intr)
        else:
            fx = float(params["cam_fx"])
            fy = float(params["cam_fy"])
            cx = float(params["cam_cx"])
            cy = float(params["cam_cy"])
        width = int(params.get("cam_width", round(cx * 2)))
        height = int(params.get("cam_height", round(cy * 2)))
        R = np.asarray(sf["camera_optical_R_from_IMU"], dtype=np.float64).reshape(3, 3)
        t = np.asarray(sf["camera_optical_translation_from_IMU"], dtype=np.float64).reshape(3)
        return cls(fx=fx, fy=fy, cx=cx, cy=cy, width=width, height=height, R_cam_body=R, t_cam_body=t)


@dataclass(frozen=True)
class ObjectMeshConfig:
    """物体分离、Surface Nets 网格化与记忆库构建参数。"""

    cloud_voxel_m: float = 0.05
    normal_knn: int = 12
    # 分块地面/天花板剔除
    tile_size_m: float = 2.0
    floor_strip_above_m: float = 0.14
    ceiling_strip_below_m: float = 0.22
    # 结构墙体剔除
    wall_normal_z_max: float = 0.35
    wall_plane_tol_m: float = 0.14
    wall_min_height_m: float = 1.65
    wall_min_length_m: float = 1.50
    wall_min_points: int = 120
    # 物体实例聚类
    cluster_voxel_m: float = 0.14
    min_cluster_points: int = 35
    min_object_height_m: float = 0.18
    min_object_span_m: float = 0.25
    max_object_span_m: float = 5.5
    # 局部 TSDF + Surface Nets 网格化
    mesh_voxel_m: float = 0.05
    mesh_max_grid_dim: int = 44
    mesh_trunc_voxels: float = 2.2
    mesh_smooth_iters: int = 2
    # 多视角 RGB 着色
    rgb_max_dist_m: float = 7.0
    rgb_min_dist_m: float = 0.55
    rgb_max_views_per_object: int = 6


@dataclass
class TriangleMesh:
    """带顶点 RGB 颜色的 3D 三角形网格。"""

    vertices: np.ndarray  # (V, 3) float64
    faces: np.ndarray  # (F, 3) int32
    colors_rgb: np.ndarray  # (V, 3) uint8

    @property
    def num_vertices(self) -> int:
        return int(len(self.vertices))

    @property
    def num_faces(self) -> int:
        return int(len(self.faces))


@dataclass
class ObjectMemoryEntry:
    """单件物体的结构化空间记忆记录（锚定关键帧，可随回环刚性更新）。"""

    object_id: str
    floor_label: str
    geometric_category: str
    dominant_color: str
    anchor_kf_key: list[int]
    center_world_xyz: list[float]
    center_anchor_xyz: list[float]
    obb_extent_lwh_m: list[float]
    obb_yaw_rad: float
    height_above_floor_m: float
    point_count: int
    num_vertices: int
    num_triangles: int
    mean_rgb: list[int]
    mesh_ply: str
    best_rgb_crop: str | None
    best_view_frame: str | None
    best_view_stamp_s: float | None
    semantic_label: str | None = None


def write_ply_mesh(path: Path, mesh: TriangleMesh) -> None:
    """导出标准 ASCII PLY 彩色三角形网格（可直接用 MeshLab / Blender / CloudCompare 打开）。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    v = np.asarray(mesh.vertices, dtype=np.float64)
    f = np.asarray(mesh.faces, dtype=np.int32)
    c = np.asarray(mesh.colors_rgb, dtype=np.uint8)
    lines = [
        "ply",
        "format ascii 1.0",
        f"element vertex {len(v)}",
        "property float x",
        "property float y",
        "property float z",
        "property uchar red",
        "property uchar green",
        "property uchar blue",
        f"element face {len(f)}",
        "property list uchar int vertex_indices",
        "end_header",
    ]
    for i in range(len(v)):
        lines.append(
            f"{v[i, 0]:.4f} {v[i, 1]:.4f} {v[i, 2]:.4f} {int(c[i, 0])} {int(c[i, 1])} {int(c[i, 2])}"
        )
    for j in range(len(f)):
        lines.append(f"3 {int(f[j, 0])} {int(f[j, 1])} {int(f[j, 2])}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _rgb_to_color_name(rgb: np.ndarray) -> str:
    r, g, b = [float(x) for x in rgb]
    vmax, vmin = max(r, g, b), min(r, g, b)
    if vmax < 55:
        return "black"
    if vmin > 195:
        return "white"
    if (vmax - vmin) < 28:
        return "gray"
    if r >= g and r >= b:
        if g > 140 and b < 95:
            return "yellow" if g > 180 else "brown_wood"
        return "red_brown" if g > 70 else "red"
    if g >= r and g >= b:
        return "green"
    return "blue"


def _classify_geometry(extent_lwh: np.ndarray, height_above_floor: float, flat_top_ratio: float) -> str:
    l, w, h = sorted([float(extent_lwh[0]), float(extent_lwh[1])], reverse=True) + [float(extent_lwh[2])]
    if h >= 1.45 and l >= 0.75 and w <= 0.85:
        return "cabinet_or_door_panel"
    if h >= 1.35 and l >= 0.65:
        return "tall_cabinet_or_shelf"
    if 0.45 <= (height_above_floor + h) <= 1.15 and l >= 0.85 and flat_top_ratio >= 0.25:
        return "desk_or_table"
    if 0.35 <= h <= 1.15 and l <= 1.05 and w <= 0.95:
        return "chair_or_stool"
    if l >= 1.4 and 0.45 <= h <= 1.15:
        return "sofa_or_bench"
    if h <= 0.55 and l <= 0.9:
        return "box_or_small_fixture"
    return "indoor_furniture"


def _estimate_oriented_normals(
    pts: np.ndarray,
    view_dirs: np.ndarray,
    *,
    knn: int = 12,
) -> np.ndarray:
    """用 cKDTree k 近邻批量向量化 PCA 计算局部法向，并朝传感器观测射线定向。"""
    n_pts = len(pts)
    if n_pts < 4:
        out = view_dirs.copy()
        norms = np.linalg.norm(out, axis=1, keepdims=True)
        return np.divide(out, np.maximum(norms, 1e-9))
    k = min(int(knn), n_pts)
    tree = cKDTree(pts)
    _, idx = tree.query(pts, k=k, workers=-1)
    # 分块做向量化 3x3 协方差特征分解，兼顾速度与内存占用
    normals = np.zeros((n_pts, 3), dtype=np.float64)
    chunk = 25000
    for s in range(0, n_pts, chunk):
        e = min(s + chunk, n_pts)
        nb = pts[idx[s:e]]  # (B, k, 3)
        mu = nb.mean(axis=1, keepdims=True)
        diff = nb - mu
        cov = np.einsum("bki,bkj->bij", diff, diff) / float(k)
        _, evecs = np.linalg.eigh(cov)
        normals[s:e] = evecs[:, :, 0]
    dots = np.sum(normals * view_dirs, axis=1)
    normals[dots < 0.0] *= -1.0
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    return np.divide(normals, np.maximum(norms, 1e-9))


def separate_foreground_objects(
    pts: np.ndarray,
    normals: np.ndarray,
    config: ObjectMeshConfig,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """分楼层、分块剔除地板、天花板与大面积结构墙体，返回 ``(foreground_mask, local_floor_z, stats)``。"""
    n_pts = len(pts)
    if n_pts == 0:
        return np.zeros(0, dtype=bool), np.zeros(0, dtype=np.float64), {}

    # 1. 自动识别楼层高度带（适配单层或多层建筑：如地下室 z≈0 与二楼 z≈+3.25）
    horiz = np.abs(normals[:, 2]) > 0.80
    z_all = pts[:, 2]
    z_floor_local = np.full(n_pts, float(np.percentile(z_all, 3)), dtype=np.float64)
    is_floor = np.zeros(n_pts, dtype=bool)
    is_ceiling = np.zeros(n_pts, dtype=bool)

    # 先按粗 Z 层（每 2.4 m 划分潜在楼层）+ 局部 XY tile 估计每块区域的地板与天花板标高
    z_min, z_max = float(z_all.min()), float(z_all.max())
    if (z_max - z_min) > 4.2:
        # 多层场景：自动在水平面直方图里找主要地板谷底分界
        band_edges = [z_min - 0.5, z_min + 2.45, z_max + 0.5]
    else:
        band_edges = [z_min - 0.5, z_max + 0.5]

    inv_tile = 1.0 / config.tile_size_m
    tx = np.floor(pts[:, 0] * inv_tile).astype(np.int64)
    ty = np.floor(pts[:, 1] * inv_tile).astype(np.int64)

    for b_idx in range(len(band_edges) - 1):
        z_lo, z_hi = band_edges[b_idx], band_edges[b_idx + 1]
        in_band = (z_all >= z_lo) & (z_all < z_hi)
        if not np.any(in_band):
            continue
        band_ids = np.where(in_band)[0]
        # 聚合每个 tile 内的点索引
        tile_map: dict[tuple[int, int], list[int]] = {}
        for i in band_ids:
            k = (int(tx[i]), int(ty[i]))
            tile_map.setdefault(k, []).append(int(i))
        band_floor_default = float(np.percentile(z_all[band_ids], 5))
        for idxs in tile_map.values():
            arr = np.asarray(idxs, dtype=np.int64)
            zs = z_all[arr]
            hz = horiz[arr]
            if np.count_nonzero(hz) >= 12:
                z_fl = float(np.percentile(zs[hz], 8))
                z_cl = float(np.percentile(zs[hz], 95))
            else:
                z_fl = float(np.percentile(zs, 5)) if len(zs) >= 8 else band_floor_default
                z_cl = float(np.percentile(zs, 95)) if len(zs) >= 8 else z_hi
            z_floor_local[arr] = z_fl
            # 剔除地板层（高度在局部地板 +14cm 以内且法向偏水平）
            fl_mask = (zs <= z_fl + config.floor_strip_above_m) & (
                (np.abs(normals[arr, 2]) > 0.65) | (zs <= z_fl + 0.06)
            )
            is_floor[arr[fl_mask]] = True
            # 若存在明确高于地板 1.8m 的天花板层，剔除顶部 22cm
            if (z_cl - z_fl) >= 1.80:
                cl_mask = zs >= (z_cl - config.ceiling_strip_below_m)
                is_ceiling[arr[cl_mask]] = True

    # 2. 结构墙体剔除：在剩余竖直点（|nz| < 0.35）中按「朝向量化 + 平面截距 + 空间连通」识别大面墙
    is_wall = np.zeros(n_pts, dtype=bool)
    vert_cand = (~is_floor) & (~is_ceiling) & (np.abs(normals[:, 2]) < config.wall_normal_z_max)
    vert_idx = np.where(vert_cand)[0]
    if len(vert_idx) >= config.wall_min_points:
        v_pts = pts[vert_idx]
        v_n = normals[vert_idx, :2]
        v_n_norm = np.linalg.norm(v_n, axis=1, keepdims=True)
        v_n = np.divide(v_n, np.maximum(v_n_norm, 1e-9))
        # 折叠到半圆 [0, pi) 以合并同一面墙的正反法向，按以 0°,15°,...,165° 为中心的 15° 桶分组
        ang = np.mod(np.arctan2(v_n[:, 1], v_n[:, 0]), math.pi)
        step_rad = math.radians(15.0)
        ang_bin = np.floor((ang + 0.5 * step_rad) / step_rad).astype(np.int32) % 12
        # 对每个角度桶用双倍角均值求该桶的真实主法向角，避免轴对齐墙体因 7.5° 偏置被切碎
        bin_mean_ang = np.arange(12, dtype=np.float64) * step_rad
        for b in range(12):
            mb = ang_bin == b
            if np.count_nonzero(mb) >= 10:
                a2 = 2.0 * ang[mb]
                bin_mean_ang[b] = 0.5 * math.atan2(float(np.sin(a2).mean()), float(np.cos(a2).mean()))
        eff_ang = bin_mean_ang[ang_bin]
        plane_d = np.cos(eff_ang) * v_pts[:, 0] + np.sin(eff_ang) * v_pts[:, 1]
        d_bin = np.floor(plane_d / config.wall_plane_tol_m).astype(np.int64)
        tang_s = -np.sin(eff_ang) * v_pts[:, 0] + np.cos(eff_ang) * v_pts[:, 1]

        plane_groups: dict[tuple[int, int], list[int]] = {}
        for local_i in range(len(vert_idx)):
            key = (int(ang_bin[local_i]), int(d_bin[local_i]))
            plane_groups.setdefault(key, []).append(local_i)

        for local_list in plane_groups.values():
            if len(local_list) < config.wall_min_points:
                continue
            li = np.asarray(local_list, dtype=np.int64)
            # 按沿墙切向坐标排序并按 >0.8 m 间隙拆分不同房间的共线墙段
            order = np.argsort(tang_s[li])
            li = li[order]
            s_sorted = tang_s[li]
            splits = np.where(np.diff(s_sorted) > 0.80)[0] + 1
            for seg in np.split(li, splits):
                if len(seg) < config.wall_min_points:
                    continue
                seg_pts = v_pts[seg]
                h_span = float(seg_pts[:, 2].max() - seg_pts[:, 2].min())
                l_span = float(tang_s[seg].max() - tang_s[seg].min())
                if h_span >= config.wall_min_height_m and l_span >= config.wall_min_length_m:
                    is_wall[vert_idx[seg]] = True

    fg_mask = (~is_floor) & (~is_ceiling) & (~is_wall)
    stats = {
        "total_voxels": int(n_pts),
        "floor_voxels": int(is_floor.sum()),
        "ceiling_voxels": int(is_ceiling.sum()),
        "wall_voxels": int(is_wall.sum()),
        "foreground_voxels": int(fg_mask.sum()),
    }
    return fg_mask, z_floor_local, stats


def cluster_foreground_objects(
    fg_pts: np.ndarray,
    config: ObjectMeshConfig,
) -> list[np.ndarray]:
    """在 3D 体素栅格上对前景物体点执行 26-邻域连通分量聚类，返回每个物体实例的局部索引列表。"""
    if len(fg_pts) < config.min_cluster_points:
        return []
    inv_v = 1.0 / config.cluster_voxel_m
    vkeys = np.floor(fg_pts * inv_v).astype(np.int64)
    vox_to_pts: dict[tuple[int, int, int], list[int]] = {}
    for i, row in enumerate(vkeys.tolist()):
        k = (row[0], row[1], row[2])
        vox_to_pts.setdefault(k, []).append(i)

    neighbors_26 = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if not (dx == 0 and dy == 0 and dz == 0)
    ]
    visited: set[tuple[int, int, int]] = set()
    clusters: list[np.ndarray] = []
    for start in vox_to_pts:
        if start in visited:
            continue
        q = deque([start])
        visited.add(start)
        comp_pts: list[int] = []
        while q:
            cur = q.popleft()
            comp_pts.extend(vox_to_pts[cur])
            cx, cy, cz = cur
            for dx, dy, dz in neighbors_26:
                nk = (cx + dx, cy + dy, cz + dz)
                if nk in vox_to_pts and nk not in visited:
                    visited.add(nk)
                    q.append(nk)
        if len(comp_pts) < config.min_cluster_points:
            continue
        idx_arr = np.asarray(comp_pts, dtype=np.int64)
        p = fg_pts[idx_arr]
        span = p.max(axis=0) - p.min(axis=0)
        if span[2] < config.min_object_height_m:
            continue
        if max(float(span[0]), float(span[1])) < config.min_object_span_m:
            continue
        if max(float(span[0]), float(span[1]), float(span[2])) > config.max_object_span_m:
            continue
        clusters.append(idx_arr)
    clusters.sort(key=len, reverse=True)
    return clusters


def mesh_object_surface_nets(
    pts: np.ndarray,
    normals: np.ndarray,
    colors_rgb: np.ndarray,
    *,
    voxel_m: float = 0.05,
    max_grid_dim: int = 44,
    trunc_voxels: float = 2.2,
    smooth_iters: int = 2,
) -> TriangleMesh:
    """对单个物体点云构建局部有向 TSDF 并用向量化 Surface Nets 提取彩色 3D 三角形网格。"""
    pts = np.asarray(pts, dtype=np.float64)
    normals = np.asarray(normals, dtype=np.float64)
    colors_rgb = np.asarray(colors_rgb, dtype=np.uint8)
    if len(pts) < 8:
        return TriangleMesh(
            vertices=np.empty((0, 3), dtype=np.float64),
            faces=np.empty((0, 3), dtype=np.int32),
            colors_rgb=np.empty((0, 3), dtype=np.uint8),
        )

    pmin = pts.min(axis=0)
    pmax = pts.max(axis=0)
    span = np.maximum(pmax - pmin, voxel_m)
    eff_voxel = max(float(voxel_m), float(span.max()) / max(6.0, float(max_grid_dim - 4)))
    pad = 2.0 * eff_voxel
    origin = pmin - pad
    dims = np.ceil((span + 2.0 * pad) / eff_voxel).astype(np.int32) + 1
    dims = np.clip(dims, 4, max_grid_dim)
    nx, ny, nz = int(dims[0]), int(dims[1]), int(dims[2])

    gx = origin[0] + np.arange(nx, dtype=np.float64) * eff_voxel
    gy = origin[1] + np.arange(ny, dtype=np.float64) * eff_voxel
    gz = origin[2] + np.arange(nz, dtype=np.float64) * eff_voxel
    X, Y, Z = np.meshgrid(gx, gy, gz, indexing="ij")
    grid_pts = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])

    tree = cKDTree(pts)
    dists, nn_idx = tree.query(grid_pts, k=1)
    diff = grid_pts - pts[nn_idx]
    signed_proj = np.sum(diff * normals[nn_idx], axis=1)
    # 沿有向法向侧（外侧）为正，背侧（物体内部）为负；加微偏置保证恰好在点上的角点归负侧
    signs = np.where(signed_proj >= 0.0, 1.0, -1.0)
    sdf = (signs * dists).reshape(nx, ny, nz)
    valid = (dists <= trunc_voxels * eff_voxel).reshape(nx, ny, nz)

    # 找出 8 个角点全部有效且跨越 0 等值面的活跃体素单元
    c000 = sdf[:-1, :-1, :-1]
    c100 = sdf[1:, :-1, :-1]
    c010 = sdf[:-1, 1:, :-1]
    c110 = sdf[1:, 1:, :-1]
    c001 = sdf[:-1, :-1, 1:]
    c101 = sdf[1:, :-1, 1:]
    c011 = sdf[:-1, 1:, 1:]
    c111 = sdf[1:, 1:, 1:]
    corners = np.stack([c000, c100, c010, c110, c001, c101, c011, c111], axis=-1)
    v_cell = (
        valid[:-1, :-1, :-1]
        & valid[1:, :-1, :-1]
        & valid[:-1, 1:, :-1]
        & valid[1:, 1:, :-1]
        & valid[:-1, :-1, 1:]
        & valid[1:, :-1, 1:]
        & valid[:-1, 1:, 1:]
        & valid[1:, 1:, 1:]
    )
    active = v_cell & (corners.min(axis=-1) < 0.0) & (corners.max(axis=-1) >= 0.0)
    active_coords = np.argwhere(active)
    if len(active_coords) == 0:
        return TriangleMesh(
            vertices=np.empty((0, 3), dtype=np.float64),
            faces=np.empty((0, 3), dtype=np.int32),
            colors_rgb=np.empty((0, 3), dtype=np.uint8),
        )

    cell_id = np.full((nx - 1, ny - 1, nz - 1), -1, dtype=np.int32)
    cell_id[active] = np.arange(len(active_coords), dtype=np.int32)

    # 对每个活跃单元，按 12 条棱的线性插值过零点均值定位亚体素顶点
    edge_pairs = (
        ((0, 0, 0), (1, 0, 0)), ((0, 1, 0), (1, 1, 0)), ((0, 0, 1), (1, 0, 1)), ((0, 1, 1), (1, 1, 1)),
        ((0, 0, 0), (0, 1, 0)), ((1, 0, 0), (1, 1, 0)), ((0, 0, 1), (0, 1, 1)), ((1, 0, 1), (1, 1, 1)),
        ((0, 0, 0), (0, 0, 1)), ((1, 0, 0), (1, 0, 1)), ((0, 1, 0), (0, 1, 1)), ((1, 1, 0), (1, 1, 1)),
    )
    ai, aj, ak = active_coords[:, 0], active_coords[:, 1], active_coords[:, 2]
    v_sum = np.zeros((len(active_coords), 3), dtype=np.float64)
    v_cnt = np.zeros(len(active_coords), dtype=np.float64)
    for (a0, a1, a2), (b0, b1, b2) in edge_pairs:
        fa = sdf[ai + a0, aj + a1, ak + a2]
        fb = sdf[ai + b0, aj + b1, ak + b2]
        cross = (fa < 0.0) != (fb < 0.0)
        if not np.any(cross):
            continue
        denom = fa[cross] - fb[cross]
        t = np.where(np.abs(denom) > 1e-9, fa[cross] / denom, 0.5)
        t = np.clip(t, 0.0, 1.0)
        pa = np.column_stack([ai[cross] + a0, aj[cross] + a1, ak[cross] + a2]).astype(np.float64)
        pb = np.column_stack([ai[cross] + b0, aj[cross] + b1, ak[cross] + b2]).astype(np.float64)
        v_sum[cross] += pa + t[:, None] * (pb - pa)
        v_cnt[cross] += 1.0
    v_local = np.divide(v_sum, np.maximum(v_cnt[:, None], 1.0))
    vertices = origin + v_local * eff_voxel

    # 沿 3 个轴向的符号变化内部格边生成四边形（拆为 2 个外法向一致的三角形）
    faces_list: list[np.ndarray] = []

    def _emit_quads(v0: np.ndarray, v1: np.ndarray, v2: np.ndarray, v3: np.ndarray, flip: np.ndarray) -> None:
        ok = (v0 >= 0) & (v1 >= 0) & (v2 >= 0) & (v3 >= 0)
        if not np.any(ok):
            return
        a, b, c, d, fl = v0[ok], v1[ok], v2[ok], v3[ok], flip[ok]
        t1 = np.where(fl[:, None], np.column_stack([a, c, b]), np.column_stack([a, b, c]))
        t2 = np.where(fl[:, None], np.column_stack([a, d, c]), np.column_stack([a, c, d]))
        faces_list.append(t1)
        faces_list.append(t2)

    # X 方向边：(i, j, k) -> (i+1, j, k)，共享 4 个单元 (i, j-1..j, k-1..k)
    if ny > 2 and nz > 2:
        fa = sdf[:-1, 1:-1, 1:-1]
        fb = sdf[1:, 1:-1, 1:-1]
        cross = (fa < 0.0) != (fb < 0.0)
        if np.any(cross):
            ii, jj, kk = np.where(cross)
            jj += 1
            kk += 1
            _emit_quads(
                cell_id[ii, jj - 1, kk - 1],
                cell_id[ii, jj, kk - 1],
                cell_id[ii, jj, kk],
                cell_id[ii, jj - 1, kk],
                fa[cross] < 0.0,
            )

    # Y 方向边：(i, j, k) -> (i, j+1, k)
    if nx > 2 and nz > 2:
        fa = sdf[1:-1, :-1, 1:-1]
        fb = sdf[1:-1, 1:, 1:-1]
        cross = (fa < 0.0) != (fb < 0.0)
        if np.any(cross):
            ii, jj, kk = np.where(cross)
            ii += 1
            kk += 1
            _emit_quads(
                cell_id[ii - 1, jj, kk - 1],
                cell_id[ii - 1, jj, kk],
                cell_id[ii, jj, kk],
                cell_id[ii, jj, kk - 1],
                fa[cross] < 0.0,
            )

    # Z 方向边：(i, j, k) -> (i, j, k+1)
    if nx > 2 and ny > 2:
        fa = sdf[1:-1, 1:-1, :-1]
        fb = sdf[1:-1, 1:-1, 1:]
        cross = (fa < 0.0) != (fb < 0.0)
        if np.any(cross):
            ii, jj, kk = np.where(cross)
            ii += 1
            jj += 1
            _emit_quads(
                cell_id[ii - 1, jj - 1, kk],
                cell_id[ii, jj - 1, kk],
                cell_id[ii, jj, kk],
                cell_id[ii - 1, jj, kk],
                fa[cross] < 0.0,
            )

    if not faces_list:
        return TriangleMesh(
            vertices=np.empty((0, 3), dtype=np.float64),
            faces=np.empty((0, 3), dtype=np.int32),
            colors_rgb=np.empty((0, 3), dtype=np.uint8),
        )
    faces = np.vstack(faces_list).astype(np.int32)

    # 轻量 Laplacian 顶点平滑（抑制体素锯齿）
    if smooth_iters > 0 and len(faces) > 0:
        n_v = len(vertices)
        for _ in range(int(smooth_iters)):
            acc = np.zeros_like(vertices)
            cnt = np.zeros(n_v, dtype=np.float64)
            for ia, ib in ((0, 1), (1, 2), (2, 0)):
                u = faces[:, ia]
                w = faces[:, ib]
                np.add.at(acc, u, vertices[w])
                np.add.at(acc, w, vertices[u])
                np.add.at(cnt, u, 1.0)
                np.add.at(cnt, w, 1.0)
            has_nb = cnt > 0
            avg = np.divide(acc[has_nb], cnt[has_nb, None])
            vertices[has_nb] = 0.65 * vertices[has_nb] + 0.35 * avg

    # 最近着色点赋予网格顶点 RGB
    _, v_nn = tree.query(vertices, k=1)
    v_colors = colors_rgb[v_nn].astype(np.uint8)
    return TriangleMesh(vertices=vertices, faces=faces, colors_rgb=v_colors)


def _compute_obb_xy(pts: np.ndarray) -> tuple[np.ndarray, float]:
    """用 2D PCA 计算物体水平最小朝向与 (L, W, H) 尺寸。"""
    xy = pts[:, :2]
    mu = xy.mean(axis=0)
    cov = np.cov((xy - mu).T) if len(xy) >= 3 else np.eye(2)
    eigvals, eigvecs = np.linalg.eigh(cov)
    major = eigvecs[:, 1]
    yaw = math.atan2(float(major[1]), float(major[0]))
    c, s = math.cos(-yaw), math.sin(-yaw)
    R2 = np.array([[c, -s], [s, c]], dtype=np.float64)
    rot_xy = (xy - mu) @ R2.T
    l = float(rot_xy[:, 0].max() - rot_xy[:, 0].min())
    w = float(rot_xy[:, 1].max() - rot_xy[:, 1].min())
    h = float(pts[:, 2].max() - pts[:, 2].min())
    return np.array([l, w, h], dtype=np.float64), yaw


def _colorize_and_crop_object(
    obj_pts: np.ndarray,
    rgb_index: list[tuple[float, Path, np.ndarray]],
    calib: CameraCalibration | None,
    crop_out_path: Path,
    config: ObjectMeshConfig,
) -> tuple[np.ndarray, str | None, str | None, float | None]:
    """将物体 3D 点投影到最佳视角 RGB 帧：返回每点 RGB、裁剪图路径、原图路径及时间戳。"""
    default_rgb = np.full((len(obj_pts), 3), 165, dtype=np.uint8)
    if calib is None or not rgb_index:
        return default_rgb, None, None, None

    import cv2

    center = obj_pts.mean(axis=0)
    candidates: list[tuple[float, float, Path, np.ndarray]] = []
    for stamp_s, img_path, T_world_body in rgb_index:
        cam_origin = T_world_body[:3, 3]
        vec = center - cam_origin
        dist = float(np.linalg.norm(vec))
        if dist < config.rgb_min_dist_m or dist > config.rgb_max_dist_m:
            continue
        # 物体相对相机光学系的朝向余弦
        T_body_world = invert(T_world_body)
        p_body = T_body_world[:3, :3] @ center + T_body_world[:3, 3]
        p_cam = calib.R_cam_body @ p_body + calib.t_cam_body
        if p_cam[2] <= 0.35:
            continue
        u = calib.fx * (p_cam[0] / p_cam[2]) + calib.cx
        v = calib.fy * (p_cam[1] / p_cam[2]) + calib.cy
        if not (20 <= u <= calib.width - 20 and 20 <= v <= calib.height - 20):
            continue
        center_pen = math.hypot((u - calib.cx) / calib.width, (v - calib.cy) / calib.height)
        dist_pen = abs(dist - 2.6) / 4.0
        score = 1.0 - 0.8 * center_pen - 0.35 * dist_pen
        candidates.append((score, stamp_s, img_path, T_world_body))

    if not candidates:
        return default_rgb, None, None, None

    candidates.sort(key=lambda x: x[0], reverse=True)
    chosen = candidates[: config.rgb_max_views_per_object]
    rgb_samples: list[np.ndarray] = []
    rgb_masks: list[np.ndarray] = []
    best_crop_saved: str | None = None
    best_view_frame: str | None = None
    best_view_stamp: float | None = None
    best_vis_count = -1

    for _score, stamp_s, img_path, T_world_body in chosen:
        if not img_path.is_file():
            continue
        bgr = cv2.imread(str(img_path))
        if bgr is None:
            continue
        rgb_img = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        T_bw = invert(T_world_body)
        p_body = obj_pts @ T_bw[:3, :3].T + T_bw[:3, 3]
        p_cam = p_body @ calib.R_cam_body.T + calib.t_cam_body
        zc = p_cam[:, 2]
        in_front = zc > 0.35
        u = np.round(calib.fx * (p_cam[:, 0] / np.maximum(zc, 1e-6)) + calib.cx).astype(np.int32)
        v = np.round(calib.fy * (p_cam[:, 1] / np.maximum(zc, 1e-6)) + calib.cy).astype(np.int32)
        vis = in_front & (u >= 0) & (u < calib.width) & (v >= 0) & (v < calib.height)
        vis_cnt = int(vis.sum())
        if vis_cnt < 10:
            continue
        sample = np.zeros((len(obj_pts), 3), dtype=np.uint8)
        sample[vis] = rgb_img[v[vis], u[vis]]
        rgb_samples.append(sample)
        rgb_masks.append(vis)

        if vis_cnt > best_vis_count:
            best_vis_count = vis_cnt
            u0 = max(0, int(np.percentile(u[vis], 2)) - 24)
            u1 = min(calib.width, int(np.percentile(u[vis], 98)) + 24)
            v0 = max(0, int(np.percentile(v[vis], 2)) - 24)
            v1 = min(calib.height, int(np.percentile(v[vis], 98)) + 24)
            if (u1 - u0) >= 24 and (v1 - v0) >= 24:
                crop_out_path.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(crop_out_path), bgr[v0:v1, u0:u1])
                best_crop_saved = str(crop_out_path)
                best_view_frame = str(img_path)
                best_view_stamp = float(stamp_s)

    if not rgb_samples:
        return default_rgb, None, None, None

    # 对每个点取所有可见视角的中位数 RGB，剔除单帧高光/遮挡离群值
    out_rgb = default_rgb.copy()
    stacked = np.stack(rgb_samples, axis=0)  # (K, N, 3)
    masks = np.stack(rgb_masks, axis=0)  # (K, N)
    any_vis = masks.any(axis=0)
    for i in np.where(any_vis)[0]:
        out_rgb[i] = np.median(stacked[masks[:, i], i, :], axis=0).astype(np.uint8)
    # 少量未落入任何 RGB 视角的背面点继承物体均值颜色，避免网格出现突兀灰斑
    if np.any(any_vis) and not np.all(any_vis):
        fill = np.median(out_rgb[any_vis], axis=0).astype(np.uint8)
        out_rgb[~any_vis] = fill
    return out_rgb, best_crop_saved, best_view_frame, best_view_stamp


class ObjectMeshMemory:
    """可查询、可随 SLAM 回环重投影同步的物体级 3D Mesh 记忆库。"""

    def __init__(self, entries: list[ObjectMemoryEntry], root_dir: Path) -> None:
        self.entries = entries
        self.root_dir = Path(root_dir)

    @classmethod
    def load(cls, index_json: Path) -> "ObjectMeshMemory":
        data = json.loads(Path(index_json).read_text(encoding="utf-8"))
        entries = [ObjectMemoryEntry(**item) for item in data.get("objects", [])]
        return cls(entries=entries, root_dir=Path(index_json).parent)

    def query(
        self,
        *,
        floor_label: str | None = None,
        category: str | None = None,
        color: str | None = None,
        near_xyz: np.ndarray | None = None,
        top_k: int = 10,
    ) -> list[ObjectMemoryEntry]:
        """按楼层、几何/语义类别、颜色与空间近邻检索物体记忆。"""
        out = []
        for e in self.entries:
            if floor_label and e.floor_label != floor_label:
                continue
            if category:
                cat_l = category.lower()
                label_ok = (e.semantic_label and cat_l in e.semantic_label.lower()) or (
                    cat_l in e.geometric_category.lower()
                )
                if not label_ok:
                    continue
            if color and color.lower() not in e.dominant_color.lower():
                continue
            out.append(e)
        if near_xyz is not None:
            q = np.asarray(near_xyz, dtype=np.float64).reshape(3)
            out.sort(key=lambda e: float(np.linalg.norm(np.asarray(e.center_world_xyz) - q)))
        return out[: max(1, int(top_k))]

    def reproject_from_keyframe_poses(self, updated_poses: dict[tuple[int, int], np.ndarray]) -> int:
        """当后端发生新回环更新关键帧世界位姿时，按 ``anchor_kf_key`` 刚性同步所有记忆物体坐标。"""
        moved = 0
        for e in self.entries:
            key = (int(e.anchor_kf_key[0]), int(e.anchor_kf_key[1]))
            T_w_a = updated_poses.get(key)
            if T_w_a is None:
                continue
            c_a = np.asarray(e.center_anchor_xyz, dtype=np.float64)
            c_w = T_w_a[:3, :3] @ c_a + T_w_a[:3, 3]
            e.center_world_xyz = [round(float(x), 4) for x in c_w]
            moved += 1
        return moved


def build_object_mesh_memory_from_map(
    map_dir: Path,
    output_dir: Path,
    *,
    rgb_index_jsonl: Path | None = None,
    preflight_json: Path | None = None,
    config: ObjectMeshConfig | None = None,
    keyframe_stride: int = 1,
) -> dict[str, Any]:
    """从 V4 ``MapStore`` 与 RGB 索引端到端构建物体级 3D Mesh 记忆库并持久化落盘。"""
    t0 = time.perf_counter()
    cfg = config or ObjectMeshConfig()
    map_dir = Path(map_dir).resolve()
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    meshes_dir = output_dir / "meshes"
    crops_dir = output_dir / "crops"
    meshes_dir.mkdir(exist_ok=True)
    crops_dir.mkdir(exist_ok=True)

    meta = json.loads((map_dir / "map.json").read_text(encoding="utf-8"))
    kfs_meta = meta.get("keyframes", [])
    if not kfs_meta:
        raise RuntimeError(f"Empty map.json in {map_dir}")

    # 1. 聚合 V4 BA 对齐后的全局点云与每点观测射线方向
    kf_keys: list[list[int]] = []
    kf_stamps_s: list[float] = []
    kf_poses: list[np.ndarray] = []
    voxel_accum: dict[tuple[int, int, int], list[Any]] = {}
    inv_v = 1.0 / cfg.cloud_voxel_m

    for i, item in enumerate(kfs_meta):
        key = [int(item["key"][0]), int(item["key"][1])]
        T_wb = np.asarray(item["T_world_body"], dtype=np.float64).reshape(4, 4)
        stamp_s = float(item["stamp_ns"]) * 1e-9
        kf_keys.append(key)
        kf_stamps_s.append(stamp_s)
        kf_poses.append(T_wb)
        if i % max(1, int(keyframe_stride)) != 0:
            continue
        npz_name = item.get("file") or item.get("points_file")
        if not npz_name:
            continue
        npz_path = map_dir / npz_name
        if not npz_path.is_file():
            continue
        npz_data = np.load(npz_path)
        if "points_body" in npz_data:
            pts_b = npz_data["points_body"].astype(np.float64)
        else:
            pts_l = npz_data["points_lidar"].astype(np.float64)
            T_bl = np.asarray(item.get("T_body_lidar", np.eye(4)), dtype=np.float64).reshape(4, 4)
            pts_b = pts_l @ T_bl[:3, :3].T + T_bl[:3, 3]
        if len(pts_b) == 0:
            continue
        pts_w = pts_b @ T_wb[:3, :3].T + T_wb[:3, 3]
        sensor_o = T_wb[:3, 3]
        # 按 5 cm 细体素去重，每体素累计均值坐标、观测射线方向与最近关键帧索引
        vkeys = np.floor(pts_w * inv_v).astype(np.int64)
        uniq, first_idx = np.unique(vkeys, axis=0, return_index=True)
        for u_row, p_i in zip(uniq.tolist(), first_idx.tolist()):
            k = (u_row[0], u_row[1], u_row[2])
            p = pts_w[p_i]
            vdir = sensor_o - p
            vn = float(np.linalg.norm(vdir))
            if vn > 1e-6:
                vdir = vdir / vn
            acc = voxel_accum.get(k)
            if acc is None:
                voxel_accum[k] = [p.copy(), vdir.copy(), 1, i]
            else:
                acc[0] += p
                acc[1] += vdir
                acc[2] += 1

    n_vox = len(voxel_accum)
    pts_all = np.empty((n_vox, 3), dtype=np.float64)
    vdirs_all = np.empty((n_vox, 3), dtype=np.float64)
    kf_idx_all = np.empty(n_vox, dtype=np.int32)
    for idx, acc in enumerate(voxel_accum.values()):
        cnt = float(acc[2])
        pts_all[idx] = acc[0] / cnt
        vd = acc[1]
        vn = float(np.linalg.norm(vd))
        vdirs_all[idx] = vd / vn if vn > 1e-6 else np.array([0.0, 0.0, 1.0])
        kf_idx_all[idx] = acc[3]

    # 2. 有向法向估计 + 结构壳层（地板/天花板/墙体）剔除
    normals_all = _estimate_oriented_normals(pts_all, vdirs_all, knn=cfg.normal_knn)
    fg_mask, z_floor_local, sep_stats = separate_foreground_objects(pts_all, normals_all, cfg)
    fg_pts = pts_all[fg_mask]
    fg_normals = normals_all[fg_mask]
    fg_z_floor = z_floor_local[fg_mask]
    fg_kf_idx = kf_idx_all[fg_mask]

    # 3. 加载 RGB 帧索引与标定（可选，但有则提供全彩网格 + 最优裁剪图）
    calib = CameraCalibration.from_preflight_json(preflight_json) if (preflight_json and preflight_json.is_file()) else None
    rgb_index: list[tuple[float, Path, np.ndarray]] = []
    if rgb_index_jsonl and rgb_index_jsonl.is_file() and kf_stamps_s:
        stamps_arr = np.asarray(kf_stamps_s, dtype=np.float64)
        for line in rgb_index_jsonl.read_text(encoding="utf-8", errors="ignore").splitlines():
            s = line.strip()
            if not s:
                continue
            try:
                rec = json.loads(s)
            except json.JSONDecodeError:
                continue
            ts = float(rec.get("stamp_s", rec.get("source_time_s", -1.0)))
            ip = Path(rec["path"])
            pos_i = int(np.searchsorted(stamps_arr, ts))
            best_i = min(max(0, pos_i), len(stamps_arr) - 1)
            if best_i > 0 and abs(stamps_arr[best_i - 1] - ts) < abs(stamps_arr[best_i] - ts):
                best_i -= 1
            if abs(stamps_arr[best_i] - ts) <= 1.2:
                rgb_index.append((ts, ip, kf_poses[best_i]))

    # 4. 前景物体实例聚类 + 逐物体 Surface Nets 3D 网格化 + RGB 着色 + 关键帧锚定
    clusters = cluster_foreground_objects(fg_pts, cfg)
    entries: list[ObjectMemoryEntry] = []
    all_v: list[np.ndarray] = []
    all_f: list[np.ndarray] = []
    all_c: list[np.ndarray] = []
    v_offset = 0

    for obj_no, cl_idx in enumerate(clusters, start=1):
        obj_id = f"obj_{obj_no:04d}"
        o_pts = fg_pts[cl_idx]
        o_nrm = fg_normals[cl_idx]
        o_zfl = float(np.median(fg_z_floor[cl_idx]))
        o_kfs = fg_kf_idx[cl_idx]

        # 选出该物体观测点最多的关键帧作为刚性锚帧
        uniq_kf, kf_cnts = np.unique(o_kfs, return_counts=True)
        anchor_i = int(uniq_kf[int(np.argmax(kf_cnts))])
        T_w_anchor = kf_poses[anchor_i]
        T_anchor_w = invert(T_w_anchor)

        crop_path = crops_dir / f"{obj_id}_crop.png"
        o_rgb, saved_crop, best_frame, best_stamp = _colorize_and_crop_object(
            o_pts, rgb_index, calib, crop_path, cfg
        )
        mesh = mesh_object_surface_nets(
            o_pts,
            o_nrm,
            o_rgb,
            voxel_m=cfg.mesh_voxel_m,
            max_grid_dim=cfg.mesh_max_grid_dim,
            trunc_voxels=cfg.mesh_trunc_voxels,
            smooth_iters=cfg.mesh_smooth_iters,
        )
        if mesh.num_faces == 0:
            continue

        ply_path = meshes_dir / f"{obj_id}.ply"
        write_ply_mesh(ply_path, mesh)

        all_v.append(mesh.vertices)
        all_f.append(mesh.faces + v_offset)
        all_c.append(mesh.colors_rgb)
        v_offset += mesh.num_vertices

        center_w = o_pts.mean(axis=0)
        center_a = T_anchor_w[:3, :3] @ center_w + T_anchor_w[:3, 3]
        extent_lwh, yaw_rad = _compute_obb_xy(o_pts)
        h_above_fl = max(0.0, float(o_pts[:, 2].min() - o_zfl))
        flat_top_ratio = float(np.mean((o_nrm[:, 2] > 0.80) & (o_pts[:, 2] > np.percentile(o_pts[:, 2], 70))))
        cat = _classify_geometry(extent_lwh, h_above_fl, flat_top_ratio)
        mean_rgb = np.median(mesh.colors_rgb, axis=0).astype(int)
        floor_label = "floor_1_upper" if float(center_w[2]) >= 2.2 else ("stairs_mid" if float(center_w[2]) >= 1.1 else "floor_0_basement")

        entries.append(
            ObjectMemoryEntry(
                object_id=obj_id,
                floor_label=floor_label,
                geometric_category=cat,
                dominant_color=_rgb_to_color_name(mean_rgb),
                anchor_kf_key=kf_keys[anchor_i],
                center_world_xyz=[round(float(x), 4) for x in center_w],
                center_anchor_xyz=[round(float(x), 4) for x in center_a],
                obb_extent_lwh_m=[round(float(x), 3) for x in extent_lwh],
                obb_yaw_rad=round(float(yaw_rad), 4),
                height_above_floor_m=round(h_above_fl, 3),
                point_count=int(len(o_pts)),
                num_vertices=mesh.num_vertices,
                num_triangles=mesh.num_faces,
                mean_rgb=[int(mean_rgb[0]), int(mean_rgb[1]), int(mean_rgb[2])],
                mesh_ply=str(ply_path.relative_to(output_dir)),
                best_rgb_crop=str(Path(saved_crop).relative_to(output_dir)) if saved_crop else None,
                best_view_frame=best_frame,
                best_view_stamp_s=round(best_stamp, 3) if best_stamp is not None else None,
            )
        )

    combined_ply = meshes_dir / "all_objects_colored.ply"
    if all_v:
        combined = TriangleMesh(
            vertices=np.vstack(all_v),
            faces=np.vstack(all_f),
            colors_rgb=np.vstack(all_c),
        )
        write_ply_mesh(combined_ply, combined)
    else:
        combined = TriangleMesh(
            vertices=np.empty((0, 3)), faces=np.empty((0, 3), dtype=np.int32), colors_rgb=np.empty((0, 3), dtype=np.uint8)
        )

    wall_s = time.perf_counter() - t0
    summary = {
        "schema": "dynanav_object_mesh_memory/v1",
        "map_dir": str(map_dir),
        "keyframes_total": len(kfs_meta),
        "rgb_frames_indexed": len(rgb_index),
        "separation": sep_stats,
        "objects_meshed": len(entries),
        "total_vertices": combined.num_vertices,
        "total_triangles": combined.num_faces,
        "combined_mesh_ply": str(combined_ply),
        "wall_s": round(wall_s, 2),
        "objects": [asdict(e) for e in entries],
    }
    (output_dir / "memory_index.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return summary
