"""Adapters bridging MapStore / SLAM backend loop updates to KeyframeSceneGraph."""

from __future__ import annotations

import json
import math
from bisect import bisect_left
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from ..backend.map_store import MapStore
from .core import validate_rigid_pose
from .scene_graph import KeyframeSceneGraph

MAX_CLEARANCE_M = 5.0
CLEARANCE_SECTORS = 36


@dataclass
class RgbIndexReport:
    index: dict[int, str]
    total_lines: int = 0
    loaded_count: int = 0
    skipped_count: int = 0
    skipped_lines: list[dict[str, Any]] = field(default_factory=list)


def _key_to_id(key: Any) -> str:
    if isinstance(key, (tuple, list)) and len(key) == 2:
        return f"{int(key[0])}:{int(key[1])}"
    return str(key)


def _parse_stamp_ns(row: dict[str, Any]) -> int | None:
    if "stamp_ns" in row and row["stamp_ns"] is not None:
        raw = row["stamp_ns"]
        if isinstance(raw, bool):
            raise ValueError("stamp_ns cannot be bool")
        val = float(raw)
        if not math.isfinite(val):
            raise ValueError("stamp_ns must be finite")
        return int(round(val)) if isinstance(raw, (float, str)) else int(raw)
    for field_name in ("source_time_s", "stamp_s"):
        if field_name in row and row[field_name] is not None:
            raw = row[field_name]
            if isinstance(raw, bool):
                raise ValueError(f"{field_name} cannot be bool")
            val = float(raw)
            if not math.isfinite(val):
                raise ValueError(f"{field_name} must be finite")
            return int(round(val * 1e9))
    return None


def _resolve_rgb_entry_path(raw_path: str, index_dir: Path) -> str | None:
    candidate = Path(raw_path)
    if candidate.is_absolute():
        if candidate.is_file():
            return str(candidate)
        archived = index_dir / candidate.name
        if archived.is_file():
            return str(archived.resolve())
        return None
    direct = index_dir / candidate
    if direct.is_file():
        return str(direct.resolve())
    by_name = index_dir / candidate.name
    if by_name.is_file():
        return str(by_name.resolve())
    return str(raw_path)


def load_rgb_index_report(jsonl_path: str | Path) -> RgbIndexReport:
    """Parse an RGB index JSONL tolerating corrupt lines, NaN stamps, and archived paths."""
    path = Path(jsonl_path).resolve()
    index_dir = path.parent
    index: dict[int, str] = {}
    skipped: list[dict[str, Any]] = []
    total_lines = 0

    with path.open("r", encoding="utf-8") as stream:
        for line_no, raw_line in enumerate(stream, start=1):
            line = raw_line.strip()
            if not line:
                continue
            total_lines += 1
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                skipped.append({"line": line_no, "reason": f"invalid_json: {exc.msg}"})
                continue
            if not isinstance(row, dict):
                skipped.append({"line": line_no, "reason": "row_not_object"})
                continue
            try:
                stamp_ns = _parse_stamp_ns(row)
            except (TypeError, ValueError, OverflowError) as exc:
                skipped.append({"line": line_no, "reason": f"invalid_timestamp: {exc}"})
                continue
            if stamp_ns is None:
                skipped.append({"line": line_no, "reason": "missing_timestamp"})
                continue
            raw_rgb = row.get("rgb_path") or row.get("path") or row.get("image_path")
            if not isinstance(raw_rgb, str) or not raw_rgb.strip():
                skipped.append({"line": line_no, "reason": "missing_path"})
                continue
            resolved = _resolve_rgb_entry_path(raw_rgb, index_dir)
            if resolved is None:
                skipped.append({"line": line_no, "reason": "missing_image_file"})
                continue
            index[stamp_ns] = resolved

    return RgbIndexReport(
        index=index,
        total_lines=total_lines,
        loaded_count=len(index),
        skipped_count=len(skipped),
        skipped_lines=skipped,
    )


def load_rgb_index(jsonl_path: str | Path) -> dict[int, str]:
    """Load an RGB timestamp->path mapping with resilient line-level parsing."""
    return load_rgb_index_report(jsonl_path).index


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


def estimate_keyframe_clearance_profile(
    points_body: np.ndarray,
    *,
    z_min_m: float = -0.1,
    z_max_m: float = 1.2,
) -> tuple[float | None, np.ndarray | None]:
    """Estimate horizontal clearance radius and per-sector obstacle distances.

    Returns ``(None, None)`` when ``points_body`` has no valid returns in the height band.
    """
    pts = np.asarray(points_body, dtype=np.float64)
    if pts.size == 0:
        return None, None
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("points_body must be an Nx3 array")

    finite_mask = np.isfinite(pts).all(axis=1)
    pts = pts[finite_mask]
    if len(pts) == 0:
        return None, None

    band = pts[(pts[:, 2] >= z_min_m) & (pts[:, 2] <= z_max_m)]
    if len(band) == 0:
        return None, None

    dists = np.hypot(band[:, 0], band[:, 1])
    valid = dists > 1e-3
    if not np.any(valid):
        return None, None
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
        return None, None

    clearance = float(np.percentile(occupied_mins, 5))
    return min(MAX_CLEARANCE_M, max(0.0, clearance)), sector_mins


def estimate_keyframe_clearance_m(
    points_body: np.ndarray,
    *,
    z_min_m: float = -0.1,
    z_max_m: float = 1.2,
) -> float | None:
    """Estimate local horizontal clearance radius (5th-percentile sector minimum, capped at 5.0 m).

    Returns ``None`` when the cloud provides no valid obstacle/free-space measurements.
    """
    clearance, _ = estimate_keyframe_clearance_profile(
        points_body, z_min_m=z_min_m, z_max_m=z_max_m
    )
    return clearance


def _normalize_anchor_inputs(
    anchor_to_keyframe_map: dict[str, Any] | None,
    T_keyframe_sensor: Any,
) -> tuple[dict[str, str], np.ndarray | None, dict[str, np.ndarray]]:
    normalized_map: dict[str, str] = {}
    if anchor_to_keyframe_map:
        for anchor_id, kf_key in anchor_to_keyframe_map.items():
            normalized_map[str(anchor_id)] = _key_to_id(kf_key)

    default_extrinsic: np.ndarray | None = None
    per_anchor_extrinsics: dict[str, np.ndarray] = {}
    if isinstance(T_keyframe_sensor, dict):
        for key, mat in T_keyframe_sensor.items():
            per_anchor_extrinsics[str(key)] = np.array(
                validate_rigid_pose(mat), dtype=np.float64, copy=True
            )
    elif T_keyframe_sensor is not None:
        default_extrinsic = np.array(
            validate_rigid_pose(T_keyframe_sensor), dtype=np.float64, copy=True
        )
    return normalized_map, default_extrinsic, per_anchor_extrinsics


def _build_scene_graph_from_stream(
    items: Iterable[tuple[Any, np.ndarray]],
    *,
    rgb_index: dict[int, str] | None = None,
    layered_map: Any = None,
    rgb_max_dt_ns: int = 50_000_000,
    anchor_to_keyframe_map: dict[str, Any] | None = None,
    T_keyframe_sensor: Any = None,
) -> KeyframeSceneGraph:
    normalized_map, default_ext, per_anchor_ext = _normalize_anchor_inputs(
        anchor_to_keyframe_map, T_keyframe_sensor
    )
    graph = KeyframeSceneGraph(
        anchor_to_keyframe_map=normalized_map,
        anchor_extrinsics=per_anchor_ext,
    )
    if default_ext is not None:
        for anchor_id in normalized_map:
            graph.anchor_extrinsics.setdefault(anchor_id, default_ext.copy())
    stamps_sorted = sorted(rgb_index.keys()) if rgb_index else []

    for kf, pose in items:
        kf_id = _key_to_id(kf.key)
        clearance_m, sector_mins = estimate_keyframe_clearance_profile(kf.points_body())
        rgb_path = _match_rgb_path(kf.stamp_ns, rgb_index, stamps_sorted, rgb_max_dt_ns)
        graph.add_keyframe(
            kf_id,
            pose,
            clearance_m=clearance_m,
            rgb_path=rgb_path,
            stamp_ns=int(kf.stamp_ns),
            epoch=int(kf.key[0]),
            sector_clearances=sector_mins,
        )

    graph.segment_floors()
    graph.partition_rooms()

    if layered_map is not None:
        identity = np.eye(4, dtype=np.float64)
        for anchor_id, (old_pose, old_rev) in list(layered_map.anchors.items()):
            kf_id = graph.anchor_to_keyframe_map.get(anchor_id, anchor_id)
            if kf_id not in graph.keyframe_poses:
                continue
            ext = per_anchor_ext.get(
                anchor_id,
                default_ext
                if default_ext is not None
                else graph.anchor_extrinsics.get(anchor_id, identity),
            )
            T_map_sensor = np.array(
                validate_rigid_pose(graph.keyframe_poses[kf_id] @ ext),
                dtype=np.float64,
                copy=True,
            )
            if not np.allclose(old_pose, T_map_sensor, rtol=0.0, atol=1e-9):
                layered_map.update_anchor(anchor_id, T_map_sensor, revision=old_rev + 1)

    return graph


def build_scene_graph_from_map_store(
    map_store: Any,
    *,
    rgb_index: dict[int, str] | None = None,
    layered_map: Any = None,
    rgb_max_dt_ns: int = 50_000_000,
    anchor_to_keyframe_map: dict[str, Any] | None = None,
    T_keyframe_sensor: Any = None,
) -> KeyframeSceneGraph:
    """Construct a KeyframeSceneGraph from a backend MapStore and optionally sync LayeredMap anchors."""
    stream = (
        (map_store.keyframes[key], map_store.poses[key])
        for key in sorted(map_store.keyframes.keys())
    )
    return _build_scene_graph_from_stream(
        stream,
        rgb_index=rgb_index,
        layered_map=layered_map,
        rgb_max_dt_ns=rgb_max_dt_ns,
        anchor_to_keyframe_map=anchor_to_keyframe_map,
        T_keyframe_sensor=T_keyframe_sensor,
    )


def build_scene_graph_from_map_dir(
    map_dir: str | Path,
    *,
    rgb_index: dict[int, str] | None = None,
    layered_map: Any = None,
    rgb_max_dt_ns: int = 50_000_000,
    anchor_to_keyframe_map: dict[str, Any] | None = None,
    T_keyframe_sensor: Any = None,
) -> KeyframeSceneGraph:
    """Stream saved MapStore keyframes from disk and discard clouds immediately after clearance extraction."""
    stream = (
        (kf, pose)
        for kf, pose, _ in MapStore.iter_keyframes_from_dir(Path(map_dir), include_descriptors=False)
    )
    return _build_scene_graph_from_stream(
        stream,
        rgb_index=rgb_index,
        layered_map=layered_map,
        rgb_max_dt_ns=rgb_max_dt_ns,
        anchor_to_keyframe_map=anchor_to_keyframe_map,
        T_keyframe_sensor=T_keyframe_sensor,
    )


def sync_backend_loop_update(
    scene_graph: KeyframeSceneGraph,
    poses_by_key: dict,
    *,
    layered_map: Any = None,
    revision: int | None = None,
    anchor_to_keyframe_map: dict[str, Any] | None = None,
    T_keyframe_sensor: Any = None,
) -> dict[str, Any]:
    """Atomically validate and apply backend loop-closure poses to KeyframeSceneGraph and LayeredMap."""
    updated_poses = {
        _key_to_id(key): np.array(validate_rigid_pose(pose), dtype=np.float64, copy=True)
        for key, pose in poses_by_key.items()
    }
    normalized_map, default_ext, per_anchor_ext = _normalize_anchor_inputs(
        anchor_to_keyframe_map, T_keyframe_sensor
    )
    merged_anchor_map = {**scene_graph.anchor_to_keyframe_map, **normalized_map}

    target_revision: int | None = None
    anchor_updates: list[tuple[str, np.ndarray]] = []

    if layered_map is not None:
        if revision is None:
            max_rev = max((rev for _, rev in layered_map.anchors.values()), default=0)
            target_revision = max_rev + 1
        else:
            if type(revision) is not int or revision < 0:
                raise ValueError("revision must be a nonnegative integer")
            target_revision = revision

        identity = np.eye(4, dtype=np.float64)
        for anchor_id, (old_pose, old_rev) in layered_map.anchors.items():
            kf_id = merged_anchor_map.get(anchor_id, anchor_id)
            if kf_id not in updated_poses:
                continue
            ext = per_anchor_ext.get(
                anchor_id,
                default_ext
                if default_ext is not None
                else scene_graph.anchor_extrinsics.get(anchor_id, identity),
            )
            T_map_sensor = np.array(
                validate_rigid_pose(updated_poses[kf_id] @ ext),
                dtype=np.float64,
                copy=True,
            )
            if target_revision < old_rev:
                raise ValueError(
                    f"stale revision {target_revision} < {old_rev} for anchor {anchor_id}"
                )
            if target_revision == old_rev:
                if np.allclose(old_pose, T_map_sensor, rtol=0.0, atol=1e-9):
                    continue
                raise ValueError(
                    f"conflicting pose at duplicate revision {target_revision} for anchor {anchor_id}"
                )
            anchor_updates.append((anchor_id, T_map_sensor))

    # Commit phase: all validations succeeded, so both graphs mutate together.
    if normalized_map:
        scene_graph.anchor_to_keyframe_map.update(normalized_map)
    if default_ext is not None:
        for anchor_id in normalized_map:
            scene_graph.anchor_extrinsics[anchor_id] = default_ext.copy()
        for anchor_id, _ in anchor_updates:
            scene_graph.anchor_extrinsics.setdefault(anchor_id, default_ext.copy())
    if per_anchor_ext:
        scene_graph.anchor_extrinsics.update(per_anchor_ext)

    summary = scene_graph.reproject_from_poses(updated_poses)
    if layered_map is not None:
        for anchor_id, T_map_sensor in anchor_updates:
            layered_map.update_anchor(anchor_id, T_map_sensor, revision=target_revision)
        summary["revision"] = target_revision
    summary["synced_anchors"] = len(anchor_updates)
    return summary
