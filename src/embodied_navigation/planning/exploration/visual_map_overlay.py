"""Read-only 2D occupancy map visual overlay and Set-of-Mark candidate projection.

Renders Go2's physical footprint rectangle, in-place turning safety circle,
same-floor historical trajectory, and 2D-reachable frontier / directional
candidate markers (F1..Fk, W1..Wm) directly onto operator 2D height charts.
Purely observational: never submits actions or gates navigation.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, label

GO2_LENGTH_M = 0.70
GO2_WIDTH_M = 0.31
GO2_TURN_RADIUS_M = 0.60


def apply_se3_xy(points_xy: np.ndarray, z_const: float, T: np.ndarray | None) -> np.ndarray:
    """Transform 2D ``(N, 2)`` points at elevation ``z_const`` by 4x4 ``T`` and return XY."""
    pts = np.asarray(points_xy, dtype=np.float64)
    if pts.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    pts = pts.reshape(-1, 2)
    if T is None:
        return pts.copy()
    mat = np.asarray(T, dtype=np.float64)
    if mat.shape != (4, 4) or not np.isfinite(mat).all():
        return pts.copy()
    xyz = np.column_stack([pts, np.full(len(pts), float(z_const), dtype=np.float64)])
    out = xyz @ mat[:3, :3].T + mat[:3, 3]
    return out[:, :2]


def robot_footprint_polygon(
    x: float,
    y: float,
    yaw: float,
    *,
    length_m: float = GO2_LENGTH_M,
    width_m: float = GO2_WIDTH_M,
) -> np.ndarray:
    """Return 4x2 world XY corners of the robot body rectangle (front-left, front-right, rear-right, rear-left)."""
    hl = 0.5 * float(length_m)
    hw = 0.5 * float(width_m)
    local = np.array(
        [
            [hl, hw],
            [hl, -hw],
            [-hl, -hw],
            [-hl, hw],
        ],
        dtype=np.float64,
    )
    c, s = math.cos(float(yaw)), math.sin(float(yaw))
    rot = np.array([[c, -s], [s, c]], dtype=np.float64)
    return local @ rot.T + np.array([float(x), float(y)], dtype=np.float64)


def turning_circle_clearance(
    x: float,
    y: float,
    occ_xy: np.ndarray,
    *,
    radius_m: float = GO2_TURN_RADIUS_M,
) -> dict:
    """Measure obstacle clearance inside the robot's in-place turning circle."""
    pts = np.asarray(occ_xy, dtype=np.float64)
    if pts.size == 0:
        return {
            "radius_m": float(radius_m),
            "safe": True,
            "obstacle_count": 0,
            "min_obstacle_dist_m": None,
        }
    pts = pts.reshape(-1, 2)
    dists = np.hypot(pts[:, 0] - float(x), pts[:, 1] - float(y))
    min_dist = float(np.min(dists))
    inside = int(np.sum(dists < float(radius_m)))
    return {
        "radius_m": float(radius_m),
        "safe": inside == 0,
        "obstacle_count": inside,
        "min_obstacle_dist_m": round(min_dist, 3),
    }


def _dedup_far(candidates: list[dict], min_sep_m: float) -> list[dict]:
    kept: list[dict] = []
    for item in candidates:
        x, y = item["xy"]
        if all(math.hypot(x - k["xy"][0], y - k["xy"][1]) >= min_sep_m for k in kept):
            kept.append(item)
    return kept


def extract_visual_candidates(
    ix: np.ndarray,
    iy: np.ndarray,
    codes: np.ndarray,
    resolution_m: float,
    *,
    robot_xyz: Iterable[float],
    level_z_m: float,
    trace_xy: np.ndarray | None = None,
    transform_se3: np.ndarray | None = None,
    min_clearance_m: float = 0.45,
    frontier_clearance_m: float = 0.50,
    turn_radius_m: float = GO2_TURN_RADIUS_M,
    min_frontier_cells: int = 8,
    max_frontiers: int = 6,
    max_directional: int = 4,
) -> list[dict]:
    """Project 2D-reachable frontier (F1..Fk) and directional (W1..Wm) waypoints from an occupancy grid.

    All returned ``xy`` coordinates live in the same frame as ``robot_xyz``
    (applying ``transform_se3`` from grid frame to robot frame when supplied).
    """
    ix_arr = np.asarray(ix, dtype=np.int64)
    iy_arr = np.asarray(iy, dtype=np.int64)
    code_arr = np.asarray(codes, dtype=np.int64)
    res = float(resolution_m)
    if ix_arr.size == 0 or res <= 0.0:
        return []

    robot = np.asarray(list(robot_xyz), dtype=np.float64)
    if robot.shape != (3,) or not np.isfinite(robot).all():
        return []

    # Express robot position in the grid's own coordinate frame for exact cell indexing.
    T_grid_to_nav = None
    T_nav_to_grid = None
    if transform_se3 is not None:
        mat = np.asarray(transform_se3, dtype=np.float64)
        if mat.shape == (4, 4) and np.isfinite(mat).all():
            T_grid_to_nav = mat
            T_nav_to_grid = np.linalg.inv(mat)
    robot_in_grid = (
        (T_nav_to_grid[:3, :3] @ robot + T_nav_to_grid[:3, 3])
        if T_nav_to_grid is not None
        else robot
    )

    min_x, max_x = int(ix_arr.min()), int(ix_arr.max())
    min_y, max_y = int(iy_arr.min()), int(iy_arr.max())
    # Pad by 1 cell so grid boundary unknown cells are explicit.
    ox = min_x - 1
    oy = min_y - 1
    w = (max_x - min_x) + 3
    h = (max_y - min_y) + 3
    if w <= 0 or h <= 0 or w * h > 4_000_000:
        return []

    grid = np.full((h, w), -1, dtype=np.int8)
    grid[iy_arr - oy, ix_arr - ox] = code_arr.astype(np.int8)

    free_mask = grid == 0
    occ_mask = grid == 1
    unk_mask = grid == -1
    if not np.any(free_mask):
        return []

    # Exact Euclidean distance from every cell to the nearest occupied cell (metres).
    if np.any(occ_mask):
        clearance_m = distance_transform_edt(~occ_mask) * res
    else:
        clearance_m = np.full((h, w), 10.0, dtype=np.float64)

    # Locate robot seed in the free connected component (allow small snap if robot sits near edge).
    rx_i = int(math.floor(robot_in_grid[0] / res)) - ox
    ry_i = int(math.floor(robot_in_grid[1] / res)) - oy
    labeled_free, _ = label(free_mask)
    seed_label = 0
    if 0 <= ry_i < h and 0 <= rx_i < w:
        seed_label = int(labeled_free[ry_i, rx_i])
    if seed_label == 0:
        fy, fx = np.nonzero(free_mask)
        d2 = (fx + ox + 0.5 - robot_in_grid[0] / res) ** 2 + (fy + oy + 0.5 - robot_in_grid[1] / res) ** 2
        nearest = int(np.argmin(d2))
        if d2[nearest] <= (2.5 / res) ** 2:
            seed_label = int(labeled_free[fy[nearest], fx[nearest]])
    if seed_label == 0:
        return []

    reachable_free = labeled_free == seed_label
    safe_reachable = reachable_free & (clearance_m >= min_clearance_m)
    if not np.any(safe_reachable):
        safe_reachable = reachable_free

    # Connected component of safe walkable space (prevents proposing points behind narrow gaps).
    labeled_safe, _ = label(safe_reachable)
    safe_y, safe_x = np.nonzero(safe_reachable)
    if 0 <= ry_i < h and 0 <= rx_i < w and labeled_safe[ry_i, rx_i] > 0:
        safe_seed = int(labeled_safe[ry_i, rx_i])
    else:
        d2_safe = (safe_x + ox + 0.5 - robot_in_grid[0] / res) ** 2 + (safe_y + oy + 0.5 - robot_in_grid[1] / res) ** 2
        safe_seed = int(labeled_safe[safe_y[int(np.argmin(d2_safe))], safe_x[int(np.argmin(d2_safe))]])
    walkable = labeled_safe == safe_seed
    walk_y, walk_x = np.nonzero(walkable)
    walk_xy_grid = np.column_stack([(walk_x + ox + 0.5) * res, (walk_y + oy + 0.5) * res])
    walk_clr = clearance_m[walk_y, walk_x]

    # Prefer frontier anchors with >= frontier_clearance_m, falling back to walkable cells.
    center_mask = walk_clr >= frontier_clearance_m
    anchor_xy_grid = walk_xy_grid[center_mask] if np.any(center_mask) else walk_xy_grid
    anchor_clr = walk_clr[center_mask] if np.any(center_mask) else walk_clr

    trace_arr = None
    if trace_xy is not None:
        t = np.asarray(trace_xy, dtype=np.float64)
        if t.size >= 2:
            trace_arr = t.reshape(-1, 2)

    # 1. Frontier clusters: reachable FREE cells 4-adjacent to UNKNOWN cells.
    struct4 = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool)
    frontier_mask = reachable_free & binary_dilation(unk_mask, structure=struct4)
    f_labels, f_count = label(frontier_mask, structure=np.ones((3, 3), dtype=bool))
    raw_frontiers: list[dict] = []
    for cid in range(1, f_count + 1):
        cy, cx = np.nonzero(f_labels == cid)
        if len(cx) < min_frontier_cells:
            continue
        mean_x = float(np.mean(cx + ox + 0.5) * res)
        mean_y = float(np.mean(cy + oy + 0.5) * res)
        # Pull frontier centroid back into the nearest connected centerline walkable cell.
        d_anchor = np.hypot(anchor_xy_grid[:, 0] - mean_x, anchor_xy_grid[:, 1] - mean_y)
        best_i = int(np.argmin(d_anchor))
        if d_anchor[best_i] > 2.5:
            continue
        ax_g, ay_g = float(anchor_xy_grid[best_i, 0]), float(anchor_xy_grid[best_i, 1])
        clr = float(anchor_clr[best_i])
        nav_xy = apply_se3_xy(np.array([[ax_g, ay_g]]), level_z_m, T_grid_to_nav)[0]
        dist_robot = float(math.hypot(nav_xy[0] - robot[0], nav_xy[1] - robot[1]))
        if dist_robot < 0.8:
            continue
        if trace_arr is not None:
            dist_trace = float(np.min(np.hypot(trace_arr[:, 0] - nav_xy[0], trace_arr[:, 1] - nav_xy[1])))
        else:
            dist_trace = dist_robot
        # Higher score = less visited (further from past trajectory), wider frontier, good clearance.
        score = dist_trace + 0.02 * min(len(cx), 80) + 0.3 * min(clr, 1.5) - 0.04 * dist_robot
        raw_frontiers.append(
            {
                "kind": "frontier",
                "xy": [round(float(nav_xy[0]), 2), round(float(nav_xy[1]), 2)],
                "z": round(float(robot[2]), 3),
                "clearance_m": round(clr, 2),
                "dist_m": round(dist_robot, 2),
                "trace_dist_m": round(dist_trace, 2),
                "turn_safe": bool(clr >= turn_radius_m),
                "frontier_cells": int(len(cx)),
                "_score": score,
            }
        )
    raw_frontiers.sort(key=lambda item: item["_score"], reverse=True)
    frontiers = _dedup_far(raw_frontiers, min_sep_m=1.6)[:max_frontiers]

    # 2. Local directional waypoints (W1..Wm) projected along 8 compass rays from robot.
    raw_dirs: list[dict] = []
    for k in range(8):
        ang = k * (math.pi / 4.0)
        ux, uy = math.cos(ang), math.sin(ang)
        furthest = None
        for step_m in np.arange(0.5, 4.1, 0.25):
            px = robot_in_grid[0] + ux * step_m
            py = robot_in_grid[1] + uy * step_m
            gx = int(math.floor(px / res)) - ox
            gy = int(math.floor(py / res)) - oy
            if not (0 <= gy < h and 0 <= gx < w):
                break
            if not walkable[gy, gx]:
                break
            if step_m >= 1.5:
                furthest = (px, py, float(clearance_m[gy, gx]), float(step_m))
        if furthest is not None:
            px, py, clr, step_m = furthest
            nav_xy = apply_se3_xy(np.array([[px, py]]), level_z_m, T_grid_to_nav)[0]
            if trace_arr is not None:
                dist_trace = float(np.min(np.hypot(trace_arr[:, 0] - nav_xy[0], trace_arr[:, 1] - nav_xy[1])))
            else:
                dist_trace = step_m
            raw_dirs.append(
                {
                    "kind": "direction",
                    "xy": [round(float(nav_xy[0]), 2), round(float(nav_xy[1]), 2)],
                    "z": round(float(robot[2]), 3),
                    "clearance_m": round(clr, 2),
                    "dist_m": round(step_m, 2),
                    "trace_dist_m": round(dist_trace, 2),
                    "turn_safe": bool(clr >= turn_radius_m),
                    "_score": dist_trace + 0.2 * step_m + 0.2 * min(clr, 1.5),
                }
            )
    raw_dirs.sort(key=lambda item: item["_score"], reverse=True)
    # Keep directional points separated from both each other and frontier markers.
    directions: list[dict] = []
    for item in raw_dirs:
        x, y = item["xy"]
        if all(math.hypot(x - f["xy"][0], y - f["xy"][1]) >= 1.5 for f in frontiers) and all(
            math.hypot(x - d["xy"][0], y - d["xy"][1]) >= 1.6 for d in directions
        ):
            directions.append(item)
        if len(directions) >= max_directional:
            break

    labeled: list[dict] = []
    for idx, item in enumerate(frontiers, 1):
        out = {k: v for k, v in item.items() if not k.startswith("_")}
        out["id"] = f"F{idx}"
        labeled.append(out)
    for idx, item in enumerate(directions, 1):
        out = {k: v for k, v in item.items() if not k.startswith("_")}
        out["id"] = f"W{idx}"
        labeled.append(out)
    return labeled


def _draw_overlay(
    ax,
    *,
    free_xy: np.ndarray,
    occ_xy: np.ndarray,
    robot_xyz: np.ndarray | None,
    yaw: float | None,
    turn_info: dict | None,
    trace_xy: np.ndarray | None,
    candidates: list[dict],
    local_half_span_m: float | None = None,
) -> None:
    from matplotlib.patches import Circle, Polygon

    if len(free_xy):
        ax.scatter(free_xy[:, 0], free_xy[:, 1], c="#cfe3ec", s=7 if local_half_span_m else 4, linewidths=0, zorder=1)
    if len(occ_xy):
        ax.scatter(occ_xy[:, 0], occ_xy[:, 1], c="#152330", s=10 if local_half_span_m else 5, linewidths=0, zorder=2)

    if trace_xy is not None and len(trace_xy) >= 2:
        ax.plot(trace_xy[:, 0], trace_xy[:, 1], color="#b23a9e", linewidth=1.6, alpha=0.85, zorder=4, label="visited path")
        ax.scatter(trace_xy[0, 0], trace_xy[0, 1], marker="s", s=30, color="#6d1c60", zorder=5)

    for item in candidates:
        cx, cy = item["xy"]
        if local_half_span_m is not None and robot_xyz is not None:
            if abs(cx - robot_xyz[0]) > local_half_span_m or abs(cy - robot_xyz[1]) > local_half_span_m:
                continue
        is_frontier = item["kind"] == "frontier"
        color = "#eb6b1f" if is_frontier else "#1f77d0"
        marker = "o" if is_frontier else "D"
        ax.scatter(cx, cy, c=color, marker=marker, s=60 if local_half_span_m else 42, edgecolors="white", linewidths=0.9, zorder=7)
        ax.annotate(
            item["id"],
            (cx, cy),
            textcoords="offset points",
            xytext=(6, 5),
            fontsize=9 if local_half_span_m else 8,
            fontweight="bold",
            color="#111111",
            bbox=dict(boxstyle="round,pad=0.18", fc="white", ec=color, alpha=0.88, lw=0.9),
            zorder=8,
        )

    if robot_xyz is not None and yaw is not None:
        rx, ry = float(robot_xyz[0]), float(robot_xyz[1])
        # Turning safety circle (green = clear for in-place scan, red/orange = tight)
        t_safe = bool(turn_info["safe"]) if turn_info is not None else True
        t_radius = float(turn_info["radius_m"]) if turn_info is not None else GO2_TURN_RADIUS_M
        t_color = "#1f9d55" if t_safe else "#d9381e"
        ax.add_patch(
            Circle(
                (rx, ry),
                t_radius,
                fill=False,
                edgecolor=t_color,
                linestyle="--",
                linewidth=1.8 if local_half_span_m else 1.3,
                zorder=9,
            )
        )
        # Actual Go2 body footprint rectangle (0.70 m x 0.31 m) rotated by yaw
        corners = robot_footprint_polygon(rx, ry, yaw)
        ax.add_patch(
            Polygon(
                corners,
                closed=True,
                facecolor="#ff5a4e",
                edgecolor="#8f1108",
                alpha=0.55,
                linewidth=1.5,
                zorder=10,
            )
        )
        # Highlight front nose edge + short heading arrow
        ax.plot(corners[:2, 0], corners[:2, 1], color="#ffe066", linewidth=2.6, zorder=11)
        arrow_len = 0.9 if local_half_span_m else 1.0
        ax.arrow(
            rx,
            ry,
            arrow_len * math.cos(yaw),
            arrow_len * math.sin(yaw),
            color="#c81e1e",
            width=0.035 if local_half_span_m else 0.028,
            length_includes_head=True,
            zorder=12,
        )
        if local_half_span_m is not None:
            ax.set_xlim(rx - local_half_span_m, rx + local_half_span_m)
            ax.set_ylim(ry - local_half_span_m, ry + local_half_span_m)


def render_annotated_2d_maps(
    ix: np.ndarray,
    iy: np.ndarray,
    codes: np.ndarray,
    resolution_m: float,
    *,
    robot_xyz: Iterable[float] | None,
    yaw: float | None,
    level_z_m: float,
    global_path: Path,
    local_path: Path | None,
    title: str,
    trace_xyz: np.ndarray | None = None,
    transform_se3: np.ndarray | None = None,
    turn_radius_m: float = GO2_TURN_RADIUS_M,
    local_half_span_m: float = 6.0,
) -> dict:
    """Render annotated global + local 2D occupancy PNGs and return visual summary & candidates."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ix_arr = np.asarray(ix, dtype=np.int64)
    iy_arr = np.asarray(iy, dtype=np.int64)
    code_arr = np.asarray(codes, dtype=np.int64)
    res = float(resolution_m)

    raw_xy = np.column_stack([(ix_arr + 0.5) * res, (iy_arr + 0.5) * res]) if ix_arr.size else np.zeros((0, 2))
    nav_xy = apply_se3_xy(raw_xy, level_z_m, transform_se3)
    free_xy = nav_xy[code_arr == 0] if ix_arr.size else np.zeros((0, 2))
    occ_xy = nav_xy[code_arr == 1] if ix_arr.size else np.zeros((0, 2))

    robot = None
    if robot_xyz is not None:
        r = np.asarray(list(robot_xyz), dtype=np.float64)
        if r.shape == (3,) and np.isfinite(r).all():
            robot = r

    # Filter trajectory to the same physical height band so multi-floor paths do not overlap.
    trace_xy = None
    if trace_xyz is not None:
        t = np.asarray(trace_xyz, dtype=np.float64)
        if t.ndim == 2 and t.shape[1] >= 3 and len(t) > 0:
            ref_z = float(robot[2]) if robot is not None else float(level_z_m)
            same_floor = np.abs(t[:, 2] - ref_z) <= 0.85
            if np.any(same_floor):
                trace_xy = t[same_floor, :2]
        elif t.ndim == 2 and t.shape[1] == 2 and len(t) > 0:
            trace_xy = t

    turn_info = None
    candidates: list[dict] = []
    if robot is not None:
        turn_info = turning_circle_clearance(robot[0], robot[1], occ_xy, radius_m=turn_radius_m)
        candidates = extract_visual_candidates(
            ix_arr,
            iy_arr,
            code_arr,
            res,
            robot_xyz=robot,
            level_z_m=level_z_m,
            trace_xy=trace_xy,
            transform_se3=transform_se3,
            turn_radius_m=turn_radius_m,
        )

    turn_note = ""
    if turn_info is not None:
        state_lbl = "CLEAR" if turn_info["safe"] else f"TIGHT (d_min={turn_info['min_obstacle_dist_m']}m)"
        turn_note = f" | Go2 0.70x0.31m + R={turn_radius_m:.2f}m turn circle: {state_lbl}"

    # 1. Global floor chart
    fig, ax = plt.subplots(figsize=(9.5, 9.5))
    _draw_overlay(
        ax,
        free_xy=free_xy,
        occ_xy=occ_xy,
        robot_xyz=robot,
        yaw=yaw,
        turn_info=turn_info,
        trace_xy=trace_xy,
        candidates=candidates,
        local_half_span_m=None,
    )
    ax.set(
        xlabel="map x (m)",
        ylabel="map y (m)",
        title=f"{title}{turn_note}\nOrange F#=unvisited frontier, Blue W#=open direction, Purple=path",
    )
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    fig.text(
        0.08,
        0.015,
        "Read-only visual aid. Absent cells remain unknown; stair climbing uses direct maneuver pulses.",
        fontsize=8,
    )
    fig.savefig(global_path, dpi=140)
    plt.close(fig)

    # 2. Local zoomed chart (+/- local_half_span_m around robot)
    local_written = None
    if local_path is not None and robot is not None and yaw is not None:
        fig_l, ax_l = plt.subplots(figsize=(7.5, 7.5))
        _draw_overlay(
            ax_l,
            free_xy=free_xy,
            occ_xy=occ_xy,
            robot_xyz=robot,
            yaw=yaw,
            turn_info=turn_info,
            trace_xy=trace_xy,
            candidates=candidates,
            local_half_span_m=local_half_span_m,
        )
        ax_l.set(
            xlabel="map x (m)",
            ylabel="map y (m)",
            title=f"{title} — Local ±{local_half_span_m:.0f} m{turn_note}",
        )
        ax_l.set_aspect("equal", adjustable="box")
        ax_l.grid(alpha=0.35)
        fig_l.savefig(local_path, dpi=140)
        plt.close(fig_l)
        local_written = str(local_path)

    return {
        "image": str(global_path),
        "local_image": local_written,
        "turning_circle": turn_info,
        "candidates": candidates,
    }
