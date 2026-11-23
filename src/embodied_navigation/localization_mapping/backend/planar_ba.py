"""Oriented-normal 3D planar bundle adjustment (BALM2 point-cluster Schur complement).

Refines a pose graph jointly with multi-frame planar consistency factors while
preserving thin two-sided indoor structures (floor slabs between storeys and
thin interior partition walls): every per-frame voxel patch normal is oriented
strictly toward its observing sensor origin before clustering, so opposite
faces of the same slab or wall have opposing normals (``n_a . n_b ~ -1``) and
never merge into a single plane.
"""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from .accel import cpp_run_oriented_planar_ba
from .pose_graph import PoseGraph


@dataclass(frozen=True)
class PlanarBaResult:
    """Summary of one oriented-normal 3D planar bundle adjustment run."""

    poses: list[np.ndarray]
    converged: bool
    clusters_used: int
    obs_used: int
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
    """Jointly optimize ``graph`` poses with oriented-normal planar BA factors.

    Node 0 stays anchored in the map frame. Odometry edges, loop edges, and
    calibrated gravity priors from ``graph`` are preserved in the same sparse
    Levenberg-Marquardt system so planar alignment cannot shear or drift away
    from the metric graph structure.
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
        error_before=float(info["error_before"]),
        error_after=float(info["error_after"]),
        wall_s=wall_s,
        used_cpp=True,
    )


def refine_window_oriented_planar_ba(
    graph: PoseGraph,
    points_body_list: list[np.ndarray],
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
    """Run fast online sliding-window oriented planar BA on the newest ``window_size`` nodes.

    Nodes immediately preceding the active window and any older nodes connected
    by pose-graph edges to the window are included as fixed anchors (``n_fixed``),
    so existing loop closures are preserved and planar patches anchor smoothly
    onto the prior map.
    """
    t0 = time.perf_counter()
    n_nodes = len(graph.poses)
    if n_nodes != len(points_body_list):
        raise ValueError(
            f"points_body_list length ({len(points_body_list)}) must match graph nodes ({n_nodes})"
        )
    if n_nodes <= 2 or window_size <= 1:
        return {}, PlanarBaResult(
            poses=[],
            converged=True,
            clusters_used=0,
            obs_used=0,
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
            error_before=0.0,
            error_after=0.0,
            wall_s=float(time.perf_counter() - t0),
            used_cpp=False,
        )
    free_set = set(free_global)
    # Anchor the boundary frame plus up to 3 immediately preceding frames for planar support.
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
    sub_pts = [points_body_list[g] for g in ordered_global]
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

    result = cpp_run_oriented_planar_ba(
        sub_poses,
        sub_pts,
        sub_edges,
        sub_grav,
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
        error_before=float(info["error_before"]),
        error_after=float(info["error_after"]),
        wall_s=wall_s,
        used_cpp=True,
    )
