"""Adapters bridging MapStore / SLAM backend loop updates to KeyframeSceneGraph."""

from __future__ import annotations

from bisect import bisect_left
from typing import Any

import numpy as np

from .scene_graph import KeyframeSceneGraph

MAX_CLEARANCE_M = 5.0
CLEARANCE_SECTORS = 36


def _key_to_id(key: Any) -> str:
    if isinstance(key, tuple) and len(key) == 2:
        return f"{int(key[0])}:{int(key[1])}"
    return str(key)


def _match_rgb_path(
    stamp_ns: int,
    rgb_index: dict[int, str] | None,
    stamps_sorted: list[int],
    max_dt_ns: int,
) -> str | None:
    if not rgb_index:
        return None
    if stamp_ns in rgb_index:
        return rgb_index[stamp_ns]
    if not stamps_sorted or max_dt_ns <= 0:
        return None
    pos = bisect_left(stamps_sorted, stamp_ns)
    candidates: list[tuple[int, int]] = []
    for idx in (pos - 1, pos):
        if 0 <= idx < len(stamps_sorted):
            candidate_ns = stamps_sorted[idx]
            dt = abs(candidate_ns - stamp_ns)
            if dt <= max_dt_ns:
                candidates.append((dt, candidate_ns))
    if not candidates:
        return None
    _, best_ns = min(candidates)
    return rgb_index[best_ns]


def estimate_keyframe_clearance_m(
    points_body: np.ndarray,
    *,
    z_min_m: float = -0.1,
    z_max_m: float = 1.2,
) -> float:
    """Estimate local horizontal clearance radius (5th-percentile sector minimum, capped at 5.0 m)."""
    pts = np.asarray(points_body, dtype=np.float64)
    if pts.size == 0:
        return MAX_CLEARANCE_M
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points_body must be an Nx3 array")

    finite_mask = np.isfinite(pts).all(axis=1)
    pts = pts[finite_mask]
    if len(pts) == 0:
        return MAX_CLEARANCE_M

    band = pts[(pts[:, 2] >= z_min_m) & (pts[:, 2] <= z_max_m)]
    if len(band) == 0:
        return MAX_CLEARANCE_M

    dists = np.hypot(band[:, 0], band[:, 1])
    valid = dists > 1e-3
    if not np.any(valid):
        return MAX_CLEARANCE_M
    band = band[valid]
    dists = dists[valid]

    angles = np.arctan2(band[:, 1], band[:, 0]) + np.pi
    sectors = np.clip(
        (angles / (2.0 * np.pi) * CLEARANCE_SECTORS).astype(np.int64),
        0,
        CLEARANCE_SECTORS - 1,
    )
    sector_mins = np.full(CLEARANCE_SECTORS, np.inf, dtype=np.float64)
    np.minimum.at(sector_mins, sectors, dists)
    occupied_mins = sector_mins[np.isfinite(sector_mins)]
    if len(occupied_mins) == 0:
        return MAX_CLEARANCE_M

    clearance = float(np.percentile(occupied_mins, 5))
    return min(MAX_CLEARANCE_M, max(0.0, clearance))


def build_scene_graph_from_map_store(
    map_store: Any,
    *,
    rgb_index: dict[int, str] | None = None,
    layered_map: Any = None,
    rgb_max_dt_ns: int = 50_000_000,
) -> KeyframeSceneGraph:
    """Construct a KeyframeSceneGraph from a backend MapStore and optionally sync LayeredMap anchors."""
    graph = KeyframeSceneGraph()
    stamps_sorted = sorted(rgb_index.keys()) if rgb_index else []

    for key in sorted(map_store.keyframes.keys()):
        kf = map_store.keyframes[key]
        kf_id = _key_to_id(key)
        pose = map_store.poses[key]
        clearance_m = estimate_keyframe_clearance_m(kf.points_body())
        rgb_path = _match_rgb_path(kf.stamp_ns, rgb_index, stamps_sorted, rgb_max_dt_ns)
        graph.add_keyframe(
            kf_id,
            pose,
            clearance_m=clearance_m,
            rgb_path=rgb_path,
        )

    graph.segment_floors()
    graph.partition_rooms()

    if layered_map is not None:
        for kf_id, pose in graph.keyframe_poses.items():
            if kf_id in layered_map.anchors:
                old_pose, old_rev = layered_map.anchors[kf_id]
                if not np.allclose(old_pose, pose, rtol=0.0, atol=1e-9):
                    layered_map.update_anchor(kf_id, pose, revision=old_rev + 1)

    return graph


def sync_backend_loop_update(
    scene_graph: KeyframeSceneGraph,
    poses_by_key: dict,
    *,
    layered_map: Any = None,
    revision: int = 1,
) -> dict[str, Any]:
    """Apply backend loop-closure poses to KeyframeSceneGraph and optional LayeredMap."""
    updated_poses = {
        _key_to_id(key): np.asarray(pose, dtype=np.float64)
        for key, pose in poses_by_key.items()
    }
    summary = scene_graph.reproject_from_poses(updated_poses)

    synced_anchors = 0
    if layered_map is not None:
        for kf_id, pose in updated_poses.items():
            if kf_id in layered_map.anchors:
                layered_map.update_anchor(kf_id, pose, revision=revision)
                synced_anchors += 1
    summary["synced_anchors"] = synced_anchors
    return summary
