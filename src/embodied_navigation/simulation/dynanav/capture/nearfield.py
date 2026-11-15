"""CPU contracts for an extra, explicitly simulated downward depth camera.

No stage/environment geometry, world pose, floor label, clearance, map or SLAM
input is consulted here. Unknown/self pixels are never rays of free space.
"""
from __future__ import annotations

import hashlib
import json
import numbers
from pathlib import Path
import time

import numpy as np

SCHEMA = "dynanav-sim-nearfield-depth/v1"
SOURCE = "simulated_downward_depth_camera"
RESOLUTION = (256, 256)
FOCAL_MM = 12.0
APERTURE_MM = 42.0
MOUNT = np.array([0.32, 0.0, 0.18])
PITCH_RAD = np.deg2rad(55.0)
# Columns are optical +x right, +y down, +z forward in base axes.
R_BASE_OPTICAL = np.array([[0., -np.sin(PITCH_RAD), np.cos(PITCH_RAD)],
                           [-1., 0., 0.],
                           [0., -np.cos(PITCH_RAD), -np.sin(PITCH_RAD)]])
ROBOT_ROOT = "/World/Go2"


def calibration_receipt():
    return {
        "frame_id": "go2_base", "optical_frame_id": "go2_nearfield_depth_optical",
        "resolution_wh": list(RESOLUTION), "focal_length_mm": FOCAL_MM,
        "aperture_mm": [APERTURE_MM, APERTURE_MM],
        "intrinsics_fx_fy_cx_cy": [256 * FOCAL_MM / APERTURE_MM] * 2 + [128., 128.],
        "pixel_coordinates": "integer column,row; principal point width/2,height/2",
        "R_base_from_optical": R_BASE_OPTICAL.tolist(), "translation_base_m": MOUNT.tolist(),
        "transform_source": "fixed authored robot sensor mount; no world or environment pose",
        "min_depth_m": .05, "max_depth_m": 5.0,
        "simulated_hardware": True, "physical_robot_requires_corresponding_downward_sensor": True,
        "sensor_model": "ideal noise-free pinhole axial depth, not an RGB estimate or LiDAR",
    }


def reference_pair(reference):
    if (not isinstance(reference, (tuple, list)) or len(reference) != 2
            or not all(isinstance(v, numbers.Integral) for v in reference)
            or reference[0] < 0 or reference[1] <= 0):
        raise ValueError("Missing or invalid native nearfield reference time")
    return tuple(int(v) for v in reference)


def verify_reference_pair(before, after):
    a, b = reference_pair(before), reference_pair(after)
    if a[0] * b[1] != b[0] * a[1]:
        raise ValueError("Nearfield annotator reads crossed render reference frames")


def validate_camera_parameters(parameters):
    """Validate native intrinsics only; never use renderer world pose."""
    expected = {
        "renderProductResolution": RESOLUTION, "cameraAperture": (APERTURE_MM, APERTURE_MM),
        "cameraFocalLength": FOCAL_MM, "cameraApertureOffset": (0., 0.),
        "metersPerSceneUnit": 1.,
    }
    for key, value in expected.items():
        actual = np.asarray(parameters.get(key, np.nan), dtype=float)
        if actual.shape != np.asarray(value).shape or not np.allclose(actual, value, rtol=1e-6, atol=1e-6):
            raise ValueError("Nearfield native calibration mismatch: " + key)
    if parameters.get("cameraModel") != "pinhole":
        raise ValueError("Nearfield renderer is not a pinhole camera")


def backproject(depth, segmentation):
    """Filter only exact robot prim paths using paired per-pixel renderer IDs.

    The legacy SDK instance_id_segmentation adapter exposes idToLabels strings.
    Missing IDs, BACKGROUND and UNLABELLED are unknown, even with finite depth.
    Environment prim labels are used only to distinguish nonrobot identity,
    never to infer floor, stairs, traversability or world geometry.
    """
    depth = np.asarray(depth, dtype=np.float32)
    if depth.shape != RESOLUTION[::-1]:
        raise ValueError("Nearfield depth shape mismatch")
    ids = np.asarray(segmentation.get("data"))
    if ids.shape != depth.shape or ids.dtype.kind not in "ui":
        raise ValueError("Nearfield instance pixels are not aligned uint IDs")
    labels = segmentation.get("info", {}).get("idToLabels")
    if not isinstance(labels, dict) or not labels:
        raise ValueError("Nearfield instance ID-to-prim mapping missing")
    self_ids, known_ids = [], []
    for index, label in labels.items():
        if not isinstance(label, str) or not label.startswith("/"):
            continue
        try:
            index = int(index)
        except (TypeError, ValueError):
            continue
        if label == ROBOT_ROOT or label.startswith(ROBOT_ROOT + "/"):
            self_ids.append(index)
        else:
            known_ids.append(index)
    valid_depth = np.isfinite(depth) & (depth > .05) & (depth < 5.0)
    self_mask = np.isin(ids, self_ids)
    valid = valid_depth & np.isin(ids, known_ids) & ~self_mask
    unknown = ~valid  # Including self occlusion: never fill the ray behind it.
    indices = np.flatnonzero(valid)
    v, u = np.unravel_index(indices, depth.shape)
    z = depth.flat[indices]
    focal = RESOLUTION[0] * FOCAL_MM / APERTURE_MM
    optical = np.column_stack(((u - 128.) * z / focal, (v - 128.) * z / focal, z))
    points = (optical @ R_BASE_OPTICAL.T + MOUNT).astype(np.float32)
    return {
        "depth_m": depth.copy(), "instance_ids": ids.astype(np.uint32, copy=True),
        "pixel_valid": valid, "pixel_self": self_mask, "pixel_unknown": unknown,
        "points_base_m": points, "point_pixel_flat_indices": indices.astype(np.int32),
    }, {
        "total_pixels": int(depth.size), "valid_depth_pixels": int(valid_depth.sum()),
        "valid_nonrobot_pixels": int(valid.sum()), "self_pixels": int(self_mask.sum()),
        "unknown_pixels": int(unknown.sum()), "robot_instance_ids": sorted(self_ids),
        "self_mask_source": "same-render-product instance_id_segmentation; exact robot prim subtree only",
        "self_mask_robot_root": ROBOT_ROOT, "mask_same_frame": True,
        "self_occlusion_is_unknown": True, "legacy_self_box_used": False,
        "environment_labels_used_for_floor": False,
    }


def point_coverage(points):
    """Diagnostic spatial counts, NOT floor proof or obstacle clearance.

    Azimuth subdivision prevents one center ray from posing as ±45° coverage.
    Actual floor support is deliberately left to a separately validated fitter.
    """
    points = np.asarray(points)
    r = np.linalg.norm(points[:, :2], axis=1)
    az = np.rad2deg(np.arctan2(points[:, 1], points[:, 0]))
    radial_edges = [0., .35, 1.0125, 1.675, 2.3375, 3.0]
    angular_edges = np.linspace(-45., 45., 10)
    counts, _, _ = np.histogram2d(r, az, bins=(radial_edges, angular_edges))
    front = np.abs(az) <= 45.
    return {"radial_edges_m": radial_edges, "angular_edges_deg": angular_edges.tolist(),
            "endpoint_counts": counts.astype(int).tolist(),
            "front_nearest_endpoint_m": float(r[front].min()) if np.any(front) else None,
            "floor_classification": "not_performed", "clearance_supported": False}


def write_frame(directory, arrays, record):
    """Immutable per-frame bytes first, then atomic pointer; never perception.json."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / ("frame_%019d.npz" % record["stamp_ns"])
    if path.exists():
        raise ValueError("Refusing overwrite of immutable nearfield frame")
    begin = time.perf_counter_ns()
    with path.open("xb") as handle:
        np.savez(handle, **arrays, stamp_ns=np.int64(record["stamp_ns"]))
    record = dict(record, npz_path=str(path.resolve()), npz_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  archive_duration_ns=time.perf_counter_ns() - begin)
    text = json.dumps(record, sort_keys=True, allow_nan=False)
    with (directory / (path.stem + ".json")).open("x", encoding="utf-8") as handle:
        handle.write(text)
    temporary = directory / "latest.json.tmp"
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(directory / "latest.json")
    return record
