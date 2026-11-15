#!/usr/bin/env python3
"""Read-only CPU audit of captured extra-depth frames. Never changes control.

Reports timing, bytes, masks and endpoint/angular coverage. Does NOT call an
endpoint floor support or assign floor/obstacle labels from environment assets.
The pose check compares two same-stamp render cameras and fixed robot mounts.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from embodied_navigation.simulation.dynanav.capture.nearfield import (
    MOUNT, R_BASE_OPTICAL, backproject, calibration_receipt, verify_reference_pair)


def rows(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def summary(values):
    a = np.asarray(values, dtype=float)
    return None if not len(a) else {"min": float(a.min()), "median": float(np.median(a)),
                                   "p95": float(np.percentile(a, 95)), "max": float(a.max())}


def camera_mount(rotation, mount):
    t = np.eye(4); t[:3, :3] = (rotation @ np.diag([1., -1., -1.])).T
    t[3, :3] = mount
    return t


def audit(capture):
    capture = Path(capture)
    paths = sorted((capture / "nearfield_depth").glob("frame_*.json"))
    records, sizes, counts = [], [], []
    for path in paths:
        record = json.loads(path.read_text())
        assert record["calibration"] == calibration_receipt(), "Calibration differs"
        assert record["control_enabled"] is False and record["slam_input"] is False
        verify_reference_pair(record["native_reference_time"], record["time_evidence"]["reference_after_reads"])
        data = Path(record["npz_path"])
        assert hashlib.sha256(data.read_bytes()).hexdigest() == record["npz_sha256"], "NPZ hash differs"
        with np.load(data, allow_pickle=False) as arrays:
            assert int(arrays["stamp_ns"]) == record["stamp_ns"]
            expected, _ = backproject(arrays["depth_m"], {
                "data": arrays["instance_ids"],
                "info": {"idToLabels": record["self_mask"]["instance_id_to_prim"]}})
            for name in expected:
                np.testing.assert_array_equal(arrays[name], expected[name], err_msg=name)
        records.append(record); sizes.append(data.stat().st_size)
        counts.append(record["diagnostic_point_coverage"]["endpoint_counts"])
    stamps = [r["stamp_ns"] for r in records]
    assert len(stamps) == len(set(stamps)) and stamps == sorted(stamps), "Duplicate/nonmonotonic frames"
    front = {round(r["rgb_source_s"] * 1e9): r for r in rows(capture / "camera_render_pose.jsonl")}
    downward = {r["stamp_ns"]: r for r in rows(capture / "nearfield_renderer_pose_diagnostic.jsonl")}
    rotation_front = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    local = camera_mount(R_BASE_OPTICAL, MOUNT) @ np.linalg.inv(camera_mount(rotation_front, [.3, 0., .13]))
    errors, rotations, world_positions = [], [], []
    for ns in stamps:
        if ns not in front or ns not in downward:
            continue
        actual = np.linalg.inv(downward[ns]["cameraViewTransform"])
        expected = local @ np.array(front[ns]["renderer_camera_world_matrix"])
        errors.append(np.linalg.norm(actual[3, :3] - expected[3, :3]))
        trace = np.trace(actual[:3, :3] @ expected[:3, :3].T)
        rotations.append(np.rad2deg(np.arccos(np.clip((trace - 1) / 2, -1, 1))))
        world_positions.append(actual[3, :3])
    return {
        "schema": "dynanav-nearfield-diagnostic-audit/v1", "frames": len(records),
        "all_archives_reprojected_and_hash_verified": bool(records),
        "native_delta_s": summary(np.diff(stamps) / 1e9),
        "capture_processing_ms": summary([r["capture_processing_ns"] / 1e6 for r in records]),
        "archive_ms": summary([r["archive_duration_ns"] / 1e6 for r in records]),
        "queue_residence_ms": summary([r["queue_residence_ns"] / 1e6 for r in records]),
        "acquisition_physics_lag_s": summary([r["acquisition_physics_s"] - r["source_time_s"] for r in records]),
        "bytes_per_frame": summary(sizes),
        "self_pixels": summary([r["self_mask"]["self_pixels"] for r in records]),
        "unknown_pixels": summary([r["self_mask"]["unknown_pixels"] for r in records]),
        "front_nearest_endpoint_m": summary([r["diagnostic_point_coverage"]["front_nearest_endpoint_m"] for r in records
                                               if r["diagnostic_point_coverage"]["front_nearest_endpoint_m"] is not None]),
        "last_endpoint_radial_angular_counts": counts[-1] if counts else None,
        "last_coverage_axes": ({k: records[-1]["diagnostic_point_coverage"][k]
                                 for k in ("radial_edges_m", "angular_edges_deg")} if records else None),
        "same_stamp_front_camera_pairs": len(errors), "relative_mount_position_error_m": summary(errors),
        "relative_mount_angle_error_deg": summary(rotations),
        "render_position_extent_m": np.ptp(world_positions, axis=0).tolist() if world_positions else None,
        "actual_floor_support_validated": False, "control_fusion_authorized": False,
        "caveat": "Endpoint coverage is not floor or obstacle clearance. Static stand is not commanded-motion validation.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.capture), indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
