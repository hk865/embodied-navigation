"""重力对齐坐标系下的实测局部平面建模与回环表面一致性门控。

调用方必须提供重力对齐坐标系（机体 z 轴不能直接当作地面法向）。
稀疏、弯曲、跨边缘或斜坡点保持未知，不做插值填洞；本模块仅做回环准入门控，
不引入独立地面高度因子，也不推断语义楼层或可通行性。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from numbers import Real

import numpy as np
from scipy.spatial import cKDTree


def _positive(name: str, value) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} 必须为有限正数，当前收到 {value!r}")
    return float(value)


def _count(name: str, value) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{name} 必须为正整数，当前收到 {value!r}")
    return value


@dataclass(frozen=True)
class SurfaceConfig:
    """局部平面提取与覆盖一致性门控参数。"""

    cell_m: float = 0.3
    min_points: int = 6
    max_plane_rmse_m: float = 0.025
    min_spread_m: float = 0.025
    normal_tolerance_rad: float = 0.2
    merge_distance_m: float = 0.04
    correspondence_m: float = 0.3
    max_residual_m: float = 0.06
    min_coverage: float = 0.6
    min_patches: int = 1

    def __post_init__(self) -> None:
        for name in (
            "cell_m",
            "max_plane_rmse_m",
            "min_spread_m",
            "normal_tolerance_rad",
            "merge_distance_m",
            "correspondence_m",
            "max_residual_m",
        ):
            _positive(f"surface.{name}", getattr(self, name))
        _count("surface.min_points", self.min_points)
        _count("surface.min_patches", self.min_patches)
        cov = _positive("surface.min_coverage", self.min_coverage)
        if cov > 1.0:
            raise ValueError(f"surface.min_coverage 必须在 (0, 1] 内，当前收到 {self.min_coverage!r}")


@dataclass(frozen=True)
class SurfacePatch:
    """单个连通实测平面片（水平面或竖直面）。"""

    kind: str
    centroid: np.ndarray
    normal: np.ndarray
    points: np.ndarray
    source_keys: tuple
    height_m: float | None

    @property
    def support_count(self) -> int:
        return len(self.points)


@dataclass(frozen=True)
class SurfaceModel:
    """一组观测点云提取出的全部实测平面片与未归类点数。"""

    patches: tuple[SurfacePatch, ...]
    unknown_points: int
    observed_points: int


@dataclass(frozen=True)
class SurfaceComparison:
    """查询侧与目标侧局部平面的几何覆盖一致性结果。"""

    accepted: bool
    reason: str
    matched_patches: int
    compared_patches: int
    rmse_m: float
    coverage_by_kind: dict[str, float] = field(default_factory=dict)


def model_surfaces(points_world, source_keys=None, config: SurfaceConfig | None = None) -> SurfaceModel:
    """按体素格拟合局部平面并做六邻域连通合并；绝不填补未观测空洞。"""
    c = config or SurfaceConfig()
    points = np.asarray(points_world, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("surface 点云必须为有限 Nx3 数组")
    if source_keys is not None and len(source_keys) != len(points):
        raise ValueError("surface source_keys 长度必须与点云一致")
    cells: dict[tuple[int, int, int], list[int]] = {}
    for i, cell in enumerate(np.floor(points / c.cell_m).astype(np.int64)):
        cells.setdefault(tuple(cell), []).append(i)
    planar = {}
    for cell, indices in cells.items():
        group = points[indices]
        if len(group) < c.min_points:
            continue
        center = group.mean(axis=0)
        centered = group - center
        eigen, vectors = np.linalg.eigh(centered.T @ centered / len(group))
        # 最小特征值对应平面厚度方差，次小特征值保证平面有足够展布而非共线点。
        if eigen[1] < c.min_spread_m**2 or eigen[0] > c.max_plane_rmse_m**2:
            continue
        normal = vectors[:, 0]
        z = abs(normal[2])
        kind = (
            "horizontal"
            if z >= np.cos(c.normal_tolerance_rad)
            else "vertical"
            if z <= np.sin(c.normal_tolerance_rad)
            else None
        )
        if kind is not None:
            if normal[np.argmax(np.abs(normal))] < 0:
                normal = -normal
            planar[cell] = (indices, center, normal, kind)
    patches = []
    remaining = set(planar)
    while remaining:
        first = min(remaining)
        remaining.remove(first)
        indices, center, normal, kind = planar[first]
        members = list(indices)
        frontier = [first]
        while frontier:
            cell = frontier.pop()
            for axis in range(3):
                for direction in (-1, 1):
                    neighbor = list(cell)
                    neighbor[axis] += direction
                    neighbor = tuple(neighbor)
                    if neighbor not in remaining:
                        continue
                    other_indices, other_center, other_normal, other_kind = planar[neighbor]
                    if (
                        other_kind == kind
                        and abs(normal @ other_normal) >= np.cos(c.normal_tolerance_rad)
                        and abs((other_center - center) @ normal) <= c.merge_distance_m
                    ):
                        remaining.remove(neighbor)
                        frontier.append(neighbor)
                        members.extend(other_indices)
        observed = points[members].copy()
        centroid = observed.mean(axis=0)
        keys = () if source_keys is None else tuple(dict.fromkeys(source_keys[i] for i in members))
        patches.append(
            SurfacePatch(
                kind=kind,
                centroid=centroid,
                normal=normal,
                points=observed,
                source_keys=keys,
                height_m=float(centroid[2]) if kind == "horizontal" else None,
            )
        )
    support = sum(p.support_count for p in patches)
    return SurfaceModel(tuple(patches), len(points) - support, len(points))


def compare_surfaces(query: SurfaceModel, target: SurfaceModel, config: SurfaceConfig | None = None) -> SurfaceComparison:
    """校验查询侧各方向实测平面在目标子图中的覆盖率与法向残差。"""
    c = config or SurfaceConfig()
    if len(query.patches) < c.min_patches or len(target.patches) < c.min_patches:
        return SurfaceComparison(False, "insufficient_surface_observations", 0, len(query.patches), float("inf"))
    matched, errors, coverage = 0, [], {}
    for kind in ("horizontal", "vertical"):
        query_patches = [p for p in query.patches if p.kind == kind]
        if not query_patches:
            continue
        target_patches = [p for p in target.patches if p.kind == kind]
        total = sum(p.support_count for p in query_patches)
        accepted = 0
        if target_patches:
            target_points = np.vstack([p.points for p in target_patches])
            normals = np.vstack([np.tile(p.normal, (p.support_count, 1)) for p in target_patches])
            tree = cKDTree(target_points)
            for patch in query_patches:
                distance, index = tree.query(patch.points)
                residual = np.abs(np.einsum("ij,ij->i", patch.points - target_points[index], normals[index]))
                consistent = (
                    (distance <= c.correspondence_m)
                    & (residual <= c.max_residual_m)
                    & (np.abs(normals[index] @ patch.normal) >= np.cos(c.normal_tolerance_rad))
                )
                count = int(consistent.sum())
                accepted += count
                errors.extend(residual[consistent])
                matched += int(count / patch.support_count >= c.min_coverage)
        coverage[kind] = accepted / total
    rmse = float(np.sqrt(np.mean(np.square(errors)))) if errors else float("inf")
    ok = matched >= c.min_patches and all(v >= c.min_coverage for v in coverage.values())
    return SurfaceComparison(ok, "consistent" if ok else "surface_mismatch", int(matched), len(query.patches), rmse, coverage)
