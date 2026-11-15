#!/usr/bin/env python3
"""Read-only swept-area preflight for one short forward body move (CPU only).

This is a HIGH-LEVEL preparation check, not a replacement for the tracker's
front/staleness/obstacle gates. It answers two separate questions:

1. Cell clearance: does every cell intersecting the conservative 2D body sweep
   (whole rectangle, not sampled centre points) contain a FREE exported cell?
   Unknown/missing cells and occupied cells both reject.
2. Floor support: 2D free cells can come from rays and do not prove the floor,
   holes or body clearance. Unless bounded raw support points inside the sweep
   are supplied, the verdict is 'hold' with reason floor_support_not_established,
   even when every cell is free.

Poses must be post-VIO measured poses converted to the base frame by the caller
with the known T_map_imu -> T_map_base extrinsic; this module refuses to treat an
IMU origin as the base origin.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np

SCHEMA = "operator_grid_export/v1"
REQUIRED_FIELDS = ("ix", "iy", "code")


class PreflightError(ValueError):
    """Raised when the inputs cannot support a defensible preflight."""


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _load_grid(npz_path, manifest_path):
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    if manifest.get("schema") != SCHEMA:
        raise PreflightError("manifest schema mismatch")
    expected = manifest.get("npz_sha256")
    if expected is None:
        raise PreflightError("manifest has no npz_sha256")
    if str(expected) != _sha256(npz_path):
        raise PreflightError("npz sha256 does not match the manifest")
    with np.load(npz_path) as nodes:
        arrays = {}
        for key in REQUIRED_FIELDS:
            if key not in nodes:
                raise PreflightError("npz is missing %s" % key)
            value = nodes[key]
            if value.ndim != 1 or not np.issubdtype(value.dtype, np.integer):
                raise PreflightError("%s must be a one-dimensional integer array" % key)
            arrays[key] = value
    if len({value.size for value in arrays.values()}) != 1:
        raise PreflightError("ix/iy/code lengths differ")
    codes = set(int(code) for code in np.unique(arrays["code"]))
    if not codes.issubset({0, 1}):
        raise PreflightError("code array contains values outside {0 free, 1 occupied}")
    cells = {int(ix): None for ix in ()}
    cells = {(int(i), int(j)): int(c) for i, j, c in zip(arrays["ix"], arrays["iy"], arrays["code"])}
    resolution = float(manifest.get("resolution_m") or 0.0)
    if not resolution > 0.0:
        raise PreflightError("manifest resolution_m must be positive")
    return cells, resolution, manifest


def base_xy_from_imu(imu_xy, base_yaw, imu_mount_xy):
    """Planar T_map_base = T_map_imu @ inv(T_base_imu) for the IMU origin.

    imu_mount_xy is the measured IMU translation expressed in the base frame
    (x forward, y left). The IMU origin is not the base origin; this helper is the
    only accepted way to feed a post-VIO pose into the sweep check.
    """
    values = [float(imu_xy[0]), float(imu_xy[1]), float(base_yaw),
              float(imu_mount_xy[0]), float(imu_mount_xy[1])]
    if not all(math.isfinite(value) for value in values):
        raise PreflightError("non-finite pose or mount")
    cos_yaw, sin_yaw = math.cos(float(base_yaw)), math.sin(float(base_yaw))
    mount_x, mount_y = float(imu_mount_xy[0]), float(imu_mount_xy[1])
    base_x = float(imu_xy[0]) - (cos_yaw * mount_x - sin_yaw * mount_y)
    base_y = float(imu_xy[1]) - (sin_yaw * mount_x + cos_yaw * mount_y)
    return base_x, base_y


def _sweep_cells(base_xy, yaw, forward_m, half_width_m, step_m):
    """Every grid cell whose area intersects the conservative swept rectangle."""
    if not all(math.isfinite(float(value)) for value in
               [base_xy[0], base_xy[1], yaw, forward_m, half_width_m, step_m]):
        raise PreflightError("non-finite sweep parameter")
    if not forward_m > 0.0 or not half_width_m > 0.0 or not step_m > 0.0:
        raise PreflightError("sweep parameters must be positive")
    # Conservative rectangle: the body hull at the start plus the start->end
    # band, in the start heading frame, expanded by a numeric margin.
    reach = forward_m + half_width_m + step_m
    width = 2.0 * (half_width_m + step_m)
    cells = set()
    count_forward = int(math.ceil(reach / step_m)) + 1
    count_lateral = int(math.ceil(width / step_m)) + 1
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    for i in range(count_forward):
        along = i * step_m
        for j in range(count_lateral):
            across = -width / 2.0 + j * step_m
            x = base_xy[0] + cos_yaw * along - sin_yaw * across
            y = base_xy[1] + sin_yaw * along + cos_yaw * across
            cells.add((math.floor(x / step_m), math.floor(y / step_m)))
    return cells


def swept_area_preflight(
    npz_path,
    manifest_path,
    *,
    base_xy,
    base_yaw,
    forward_m,
    footprint_half_width_m,
    margin_m,
    source_stamp_s,
    pose_stamp_s,
    now_sim_s,
    max_pose_sim_age_s,
    support_points_xy_z=None,
    floor_anchor_m=None,
    min_support_points=50,
    free_below_m=0.10,
    clearance_m=0.15,
):
    """Return a fail-closed preflight verdict for one short forward body move.

    support_points_xy_z (optional, map-frame Nx3) may only come from a fresh real
    cloud whose pose pairing is recorded; when absent the verdict is a hold even
    if the cell sweep is clear. No ground truth is accepted anywhere.
    """
    for name, value in (("source_stamp_s", source_stamp_s), ("pose_stamp_s", pose_stamp_s),
                        ("now_sim_s", now_sim_s)):
        if value is None or not math.isfinite(float(value)):
            raise PreflightError("%s must be finite" % name)
    age = float(now_sim_s) - float(pose_stamp_s)
    if age < -1e-6 or age > float(max_pose_sim_age_s):
        raise PreflightError("post-VIO pose is stale or future for this preflight")
    if abs(float(source_stamp_s) - float(pose_stamp_s)) > 1.0:
        return {
            "verdict": "hold",
            "reasons": ["source_pose_stamp_mismatch"],
            "cells_checked": 0,
            "cell_counts": {"free": 0, "occupied": 0, "unknown": 0},
        }
    cells, resolution, manifest = _load_grid(npz_path, manifest_path)
    half_width = float(footprint_half_width_m) + float(margin_m)
    step = resolution * 0.5
    sweep = _sweep_cells(base_xy, base_yaw, float(forward_m), half_width, step)
    free = occupied = unknown = 0
    occupied_cells = []
    unknown_cells = []
    for key in sorted(sweep):
        code = cells.get(key)
        if code is None:
            unknown += 1
            unknown_cells.append(key)
        elif code == 1:
            occupied += 1
            occupied_cells.append(key)
        else:
            free += 1
    reasons = []
    if occupied:
        reasons.append("occupied_cell_in_sweep")
    if unknown:
        reasons.append("unknown_cell_in_sweep")
    floor_support = {"established": False, "reason": "floor_support_not_established",
                     "support_points_in_sweep": 0}
    if not reasons and support_points_xy_z is not None:
        points = np.asarray(support_points_xy_z, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
            raise PreflightError("support points must be a finite Nx3 array")
        if floor_anchor_m is None or not math.isfinite(float(floor_anchor_m)):
            raise PreflightError("floor_anchor_m is required with support points")
        anchor = float(floor_anchor_m)
        low = anchor - float(free_below_m)
        high = anchor + float(clearance_m)
        step = resolution * 0.5
        needed = set(sweep)
        covered = set()
        inside = points[(points[:, 2] >= low) & (points[:, 2] < high)]
        for x, y, _z in inside:
            covered.add((math.floor(x / step), math.floor(y / step)))
        covered &= needed
        floor_support = {
            "established": bool(len(covered) == len(needed) and inside.shape[0] >= int(min_support_points)),
            "reason": None,
            "support_points_in_sweep": int(inside.shape[0]),
            "cells_supported": int(len(covered)),
            "cells_required": int(len(needed)),
            "anchor_m": anchor,
            "band_m": [low, high],
        }
        if not floor_support["established"]:
            reasons.append("floor_support_insufficient")
    verdict = "pass" if not reasons and floor_support["established"] else "hold"
    return {
        "verdict": verdict,
        "reasons": reasons or ([] if verdict == "pass" else ["floor_support_not_established"]),
        "cells_checked": len(sweep),
        "cell_counts": {"free": free, "occupied": occupied, "unknown": unknown},
        "occupied_cells": occupied_cells[:32],
        "unknown_cells": unknown_cells[:32],
        "input": {
            "npz": str(npz_path),
            "npz_sha256": _sha256(npz_path),
            "manifest": str(manifest_path),
            "manifest_sha256": _sha256(manifest_path),
            "grid_frame": manifest.get("odometry_map_frame") or manifest.get("map_frame"),
            "resolution_m": resolution,
            "classification_floor_z_m": manifest.get("classification_floor_z_m"),
            "partial": manifest.get("partial"),
        },
        "pose": {"base_xy": [float(base_xy[0]), float(base_xy[1])],
                 "base_yaw_rad": float(base_yaw),
                 "source_stamp_s": float(source_stamp_s),
                 "pose_stamp_s": float(pose_stamp_s),
                 "now_sim_s": float(now_sim_s),
                 "pose_sim_age_s": round(age, 6)},
        "sweep": {"forward_m": float(forward_m),
                  "footprint_half_width_m": float(footprint_half_width_m),
                  "margin_m": float(margin_m),
                  "cell_step_m": step,
                  "conservative": "hull plus translation band, expanded by the margin; yaw uncertainty is not certified"},
        "floor_support": floor_support,
        "note": "2D free cells do not prove floor support, holes or body clearance; a hold without raw support evidence is intentional. This check never mutates the map and is not a substitute for the tracker gates.",
    }


def main(argv=None):
    raise SystemExit("import-only module; use swept_area_preflight() from a caller")


if __name__ == "__main__":
    main()
