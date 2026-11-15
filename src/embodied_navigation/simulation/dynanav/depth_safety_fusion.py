"""Opt-in, exact-frame RTX + simulated downward-depth safety observations.

Depth endpoints never enter FAST-LIVO2 or the native operator map. This consumer
keeps source clocks and self-mask evidence, reprojects archived raw depth, and
uses the existing floor/coverage algorithm without changing any threshold. A
missing, unpaired or expired camera frame leaves the original RTX observation
unchanged. The default is diagnostic-only, even when every check passes.
"""
from __future__ import annotations

import copy
import hashlib
import io
import json
import math
import numbers
from pathlib import Path

import numpy as np

from .capture import nearfield
from .rtx_deskewed_perception import (
    build_perception_payload, corridor_contract, fail_closed_payload, state_age_gate, verify_same_stage_pair, verify_topic_roles,
)

FUSION_SOURCE = "rtx_depth_safety_fusion_v1"
FUSION_SCHEMA = "dynanav-depth-safety-fusion/v1"
MAX_SIM_AGE_S = .25
MAX_WALL_AGE_S = .5
MAX_ARCHIVE_BYTES = 16 * 1024 * 1024


class DepthFusionError(ValueError):
    """Depth cannot safely supplement this native observation."""


def _integer(value, name):
    if isinstance(value, bool) or not isinstance(value, numbers.Integral) or value <= 0:
        raise DepthFusionError(name + " must be positive integer nanoseconds")
    return int(value)


def _number(value, name):
    try:
        result = float(value)
    except (ValueError, TypeError) as error:
        raise DepthFusionError(name + " must be finite") from error
    if not math.isfinite(result):
        raise DepthFusionError(name + " must be finite")
    return result


def _sha_json(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def _sha_points(value):
    return hashlib.sha256(np.asarray(value, dtype="<f8").tobytes()).hexdigest()


def _digest(value):
    return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def _age(stamp_ns, arrival, *, now_sim_s, now_wall_s):
    ok, reason, _ = state_age_gate(
        now_sim_s=now_sim_s, stamp_s=stamp_ns * 1e-9, arrival_wall_s=arrival,
        now_wall_s=now_wall_s, max_sim_age_s=MAX_SIM_AGE_S,
        max_wall_age_s=MAX_WALL_AGE_S,
    )
    if not ok:
        raise DepthFusionError(reason)


def _read_record(path):
    if path.stat().st_size > 2 * 1024 * 1024:
        raise DepthFusionError("depth_manifest_too_large")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise DepthFusionError("depth_manifest_not_object")
    return value


def _depth_frame_inputs(manifest_path, *, reference_stamp_ns, snapshot=None):
    """Read/select an exact frame and check its schema/stamp identity."""
    stamp = _integer(reference_stamp_ns, "reference_stamp_ns")
    pointer = Path(manifest_path).resolve()
    if snapshot is not None:
        from .async_depth_fusion import DepthFrameSnapshot
        if (type(snapshot) is not DepthFrameSnapshot or snapshot.pointer != str(pointer)
                or snapshot.stamp_ns != stamp):
            raise DepthFusionError("depth_snapshot_identity_mismatch")
    read_record = _read_record if snapshot is None else snapshot.read_record
    latest = read_record(pointer)
    latest_stamp = _integer(latest.get("stamp_ns"), "depth_stamp_ns")
    path = pointer
    record = latest
    if latest_stamp != stamp:
        path = pointer.parent / ("frame_%019d.json" % stamp)
        if snapshot is None and not path.is_file():
            raise DepthFusionError("depth_stamp_mismatch: latest_minus_lio_ns=%d" % (latest_stamp - stamp))
        record = read_record(path)
    if _integer(record.get("stamp_ns"), "depth_stamp_ns") != stamp:
        raise DepthFusionError("depth_stamp_mismatch")
    if record.get("schema") != nearfield.SCHEMA or record.get("source") != nearfield.SOURCE:
        raise DepthFusionError("depth_source_invalid")
    if record.get("frame_id") != "go2_base":
        raise DepthFusionError("depth_frame_invalid")
    if round(_number(record.get("source_time_s"), "source_time_s") * 1e9) != stamp:
        raise DepthFusionError("depth_source_clock_disagrees")
    return pointer, latest_stamp, path, record, stamp


def _depth_sensor_contract(record):
    """Shared calibration and native render-product metadata checks."""
    if record.get("calibration") != nearfield.calibration_receipt():
        raise DepthFusionError("depth_calibration_mismatch")
    # These fields are native render-product observations, never the loop clock.
    timing = record.get("time_evidence") or {}
    references = (record.get("native_reference_time"), record.get("depth_reference_time"),
                  record.get("mask_reference_time"), timing.get("event_reference"),
                  timing.get("reference_after_reads"))
    for reference in references:
        nearfield.verify_reference_pair(references[0], reference)
    if (timing.get("body_now_substituted") is not False
            or record.get("arrival_clock") != "perf_counter"
            or record.get("depth_type") != "distance_to_image_plane"
            or record.get("calibration_verified_from_render_parameters") is not True
            or record.get("environment_ground_truth_used") is not False
            or record.get("world_pose_used") is not False
            or record.get("slam_input") is not False or record.get("map_input") is not False):
        raise DepthFusionError("depth_native_reference_unverified")
    return timing


def _depth_archive_data(inputs, *, snapshot=None):
    """Read only the declared exact archive and verify its original hash."""
    pointer, latest_stamp, path, record, stamp = inputs
    archive = Path(str(record.get("npz_path", ""))).resolve()
    if archive.parent != pointer.parent or archive.name != "frame_%019d.npz" % stamp:
        raise DepthFusionError("depth_archive_path_mismatch")
    if snapshot is None:
        if archive.stat().st_size > MAX_ARCHIVE_BYTES:
            raise DepthFusionError("depth_archive_too_large")
        data = archive.read_bytes()
    else:
        data = snapshot.read_archive(archive)
        if len(data) > MAX_ARCHIVE_BYTES:
            raise DepthFusionError("depth_archive_too_large")
    if not _digest(record.get("npz_sha256")) or hashlib.sha256(data).hexdigest() != record["npz_sha256"]:
        raise DepthFusionError("depth_archive_sha_mismatch")
    return data


def _depth_mask_identity(provenance, labels):
    return (provenance.get("method") == "same_frame_instance_prim_path"
            and provenance.get("same_frame") is True
            and provenance.get("robot_prim_root") == nearfield.ROBOT_ROOT
            and provenance.get("mapping_sha256") == _sha_json(labels))


def _validate_depth_pointer_integrity(manifest_path, *, reference_stamp_ns, snapshot):
    """Check an unconsumed latest index, never age/admit its pixels as input.

    Reuses source/calibration/RP/archive identity validation. No backprojection,
    floor fit or clock substitution occurs here. Consuming this frame later
    still requires load_depth_frame's complete pixel checks and real age gate.
    """
    inputs = _depth_frame_inputs(manifest_path, reference_stamp_ns=reference_stamp_ns, snapshot=snapshot)
    record = inputs[3]
    if (not isinstance(record.get("time_evidence"), dict)
            or type(record.get("arrival_wall_s")) not in (int, float)):
        raise DepthFusionError("depth_metadata_incomplete")
    _number(record.get("arrival_wall_s"), "depth_arrival")
    _depth_sensor_contract(record)
    data = _depth_archive_data(inputs, snapshot=snapshot)
    provenance = record.get("self_mask")
    if not isinstance(provenance, dict):
        raise DepthFusionError("depth_mask_provenance_mismatch")
    labels = provenance.get("instance_id_to_prim")
    if (not isinstance(labels, dict) or not labels
            or any(not isinstance(value, str) for value in labels.values())
            or not _depth_mask_identity(provenance, labels)):
        raise DepthFusionError("depth_mask_provenance_mismatch")
    expected = {"self_mask_robot_root": nearfield.ROBOT_ROOT, "mask_same_frame": True,
                "self_occlusion_is_unknown": True, "legacy_self_box_used": False,
                "environment_labels_used_for_floor": False,
                "self_mask_source": "same-render-product instance_id_segmentation; exact robot prim subtree only"}
    if any(type(provenance.get(k)) is not type(v) or provenance[k] != v for k, v in expected.items()):
        raise DepthFusionError("depth_mask_metadata_invalid")
    pixels = nearfield.RESOLUTION[0] * nearfield.RESOLUTION[1]
    for key in ("total_pixels", "valid_depth_pixels", "valid_nonrobot_pixels", "self_pixels", "unknown_pixels"):
        value = provenance.get(key)
        if type(value) is not int or not 0 <= value <= pixels:
            raise DepthFusionError("depth_mask_metadata_invalid")
    ids = provenance.get("robot_instance_ids")
    if (provenance["total_pixels"] != pixels
            or provenance["valid_nonrobot_pixels"] + provenance["unknown_pixels"] != pixels
            or not isinstance(ids, list) or any(type(v) is not int or v < 0 for v in ids)):
        raise DepthFusionError("depth_mask_metadata_invalid")
    # ZIP inventory and scalar identity only; do not decompress depth arrays or
    # redo geometry on the receiving owner for an unrelated newer frame.
    try:
        with np.load(io.BytesIO(data), allow_pickle=False) as stored:
            required = {"stamp_ns", "depth_m", "instance_ids", "pixel_valid", "pixel_self", "pixel_unknown",
                        "points_base_m", "point_pixel_flat_indices"}
            if not required.issubset(stored.files):
                raise DepthFusionError("depth_archive_incomplete")
            native_stamp = stored["stamp_ns"]
            if (native_stamp.shape != () or native_stamp.dtype.kind not in "iu"
                    or int(native_stamp) != reference_stamp_ns):
                raise DepthFusionError("depth_archive_stamp_mismatch")
    except DepthFusionError:
        raise
    except Exception as error:
        # Only untrusted ZIP inventory/scalar parsing is normalized here.
        # Corrupt deflate, unsupported/encrypted ZIP and malformed NPY metadata
        # must invalidate cached evidence, not escape the receiving owner loop.
        # KeyboardInterrupt/SystemExit are BaseException and remain untouched.
        raise DepthFusionError("depth_archive_invalid:" + str(error)) from error
    return True


def _validate_depth_frame_integrity(inputs, *, snapshot=None):
    """Verify original calibration/RP/hash/raw pixels, without a freshness claim.

    load_depth_frame applies the original real clock/arrival gate before this
    exact-input validator. The unrelated latest index never calls this helper.
    """
    pointer, latest_stamp, path, record, stamp = inputs
    timing = _depth_sensor_contract(record)
    data = _depth_archive_data(inputs, snapshot=snapshot)
    with np.load(io.BytesIO(data), allow_pickle=False) as stored:
        arrays = {key: stored[key] for key in stored.files}
    native_stamp = arrays.get("stamp_ns")
    if (native_stamp is None or native_stamp.shape != () or native_stamp.dtype.kind not in "iu"
            or int(native_stamp) != stamp):
        raise DepthFusionError("depth_archive_stamp_mismatch")
    provenance = record.get("self_mask") or {}
    labels = provenance.get("instance_id_to_prim")
    recomputed, mask = nearfield.backproject(
        arrays.get("depth_m"), {"data": arrays.get("instance_ids"), "info": {"idToLabels": labels}}
    )
    for key, expected in recomputed.items():
        actual = arrays.get(key)
        if actual is None or actual.shape != expected.shape or not np.array_equal(actual, expected, equal_nan=True):
            raise DepthFusionError("depth_reprojection_or_mask_mismatch:" + key)
    if (any(provenance.get(k) != v for k, v in mask.items())
            or not _depth_mask_identity(provenance, labels)):
        raise DepthFusionError("depth_mask_provenance_mismatch")
    points = recomputed["points_base_m"]
    if len(points) == 0 or not np.isfinite(points).all():
        raise DepthFusionError("depth_no_finite_nonrobot_endpoints")
    # Keep the exact per-pixel ID mapping in the immutable raw manifest. The
    # producer has verified it above; control receives its SHA plus compact mask
    # statistics, not 200 KB of paths to parse and hash every control tick.
    compact_mask = {k: v for k, v in provenance.items() if k != "instance_id_to_prim"}
    return points, {
        "source": nearfield.SOURCE, "frame_id": "go2_base", "stamp_ns": stamp,
        "arrival_wall_s": _number(record["arrival_wall_s"], "depth_arrival"),
        "source_time_s": record["source_time_s"], "manifest_path": str(path),
        "manifest_sha256": _sha_json(record), "npz_sha256": record["npz_sha256"],
        "points_sha256": _sha_points(points), "points": len(points),
        "calibration": record["calibration"], "calibration_sha256": _sha_json(record["calibration"]),
        "self_mask": compact_mask, "self_mask_sha256": _sha_json(compact_mask),
        "segmentation_mapping_sha256": _sha_json(labels), "timing": timing,
        "native_reference_time": record["native_reference_time"],
        "depth_reference_time": record["depth_reference_time"],
        "mask_reference_time": record["mask_reference_time"],
        "latest_minus_lio_ns": latest_stamp - stamp,
        "native_stamp_preserved": True, "reprojection_verified": True,
    }



def load_depth_frame(manifest_path, *, reference_stamp_ns, now_sim_s, now_wall_s, snapshot=None):
    """Read exact-stamp evidence with the original identity, age and integrity gates."""
    inputs = _depth_frame_inputs(manifest_path, reference_stamp_ns=reference_stamp_ns, snapshot=snapshot)
    record, stamp = inputs[3], inputs[4]
    _age(stamp, record.get("arrival_wall_s"), now_sim_s=now_sim_s, now_wall_s=now_wall_s)
    return _validate_depth_frame_integrity(inputs, snapshot=snapshot)


def _native_source(points, payload, pair, pose_arrival_wall_s, *, now_sim_s, now_wall_s):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise DepthFusionError("native_base_points_invalid")
    stamp = _integer(pair.get("reference_stamp_ns"), "native_reference_stamp_ns")
    producer = payload.get("producer_metadata") or {}
    verify_topic_roles(producer.get("cloud_topic"), producer.get("pose_topic"))
    verify_same_stage_pair(
        cloud_frame=pair.get("map_frame"), pose_frame=pair.get("map_frame"),
        child_frame=pair.get("child_frame"), cloud_stamp_ns=pair.get("cloud_stamp_ns"),
        pose_stamp_ns=pair.get("pose_stamp_ns"), expected_map_frame=pair.get("map_frame"),
        expected_child_frame="go2_imu", stage=pair.get("stage"),
    )
    if any(_integer(pair.get(key), key) != stamp for key in
           ("reference_stamp_ns", "cloud_stamp_ns", "pose_stamp_ns")):
        raise DepthFusionError("native_pair_stamp_mismatch")
    if any(_integer(producer.get(key), key) != stamp for key in
           ("reference_stamp_ns", "cloud_stamp_ns", "pose_stamp_ns")):
        raise DepthFusionError("native_producer_stamp_mismatch")
    if (payload.get("source") != "rtx_lidar_cloud_registered_full"
            or payload.get("frame_id") != "go2_base" or producer.get("stage") != "LIO"
            or payload.get("cloud_points") != len(points)
            or round(_number(payload.get("stamp_s"), "native_stamp_s") * 1e9) != stamp):
        raise DepthFusionError("native_producer_contract_invalid")
    if not _digest(producer.get("binary_sha256")):
        raise DepthFusionError("native_binary_identity_missing")
    _age(stamp, payload.get("arrival_wall_s"), now_sim_s=now_sim_s, now_wall_s=now_wall_s)
    _age(stamp, pose_arrival_wall_s, now_sim_s=now_sim_s, now_wall_s=now_wall_s)
    return {
        "source": payload["source"], "frame_id": "go2_base", "stamp_ns": stamp,
        "arrival_wall_s": payload["arrival_wall_s"], "points": len(points),
        "points_sha256": _sha_points(points), "binary_sha256": producer["binary_sha256"],
        "producer_metadata": copy.deepcopy(producer), "pair": copy.deepcopy(pair),
        "pose_arrival_wall_s": float(pose_arrival_wall_s),
        "native_only": True,
    }


def native_source_envelope(points_base, *, stamp_s, arrival_wall_s, producer_metadata):
    """Native identity only, never a claimed floor or clearance measurement.

    The bridge already validated native layout/deskew and the exact LIO pose.
    Enabling fusion can then fit the joint real endpoints once; only a rejected
    depth input requires the separate native-only geometry fallback.
    """
    payload = fail_closed_payload(stamp_s=stamp_s, arrival_wall_s=arrival_wall_s,
                                  error="native_geometry_not_evaluated")
    payload.update(source="rtx_lidar_cloud_registered_full", cloud_points=len(points_base),
                   producer_metadata=copy.deepcopy(producer_metadata),
                   native_geometry_evaluated=False)
    return payload


def build_safety_fusion(native_points_base, native_payload, *, pair, pose_arrival_wall_s,
                        depth_manifest, now_sim_s, now_wall_s, enabled=False,
                        native_fallback_factory=None, depth_snapshot=None):
    """Return native fallback or a fully labelled joint safety-only observation.

    Valid diagnostic input also computes the candidate directions, but cannot
    affect the returned native clearance unless the separate enable flag is true.
    An optional native fallback factory is lazy: invalid depth calls it once,
    whereas joint geometry ambiguity stays fused unknown and never falls back.
    No retries, motion compensation, interpolation, body boxes or hole filling.
    """
    payload = copy.deepcopy(native_payload)
    diagnostic = {"schema": FUSION_SCHEMA, "enabled": bool(enabled), "accepted": False,
                  "control_used": False, "exact_integer_stamp_required": True,
                  "depth_enters_slam": False, "operator_map_native_only": True}
    payload["depth_safety_diagnostic"] = diagnostic
    try:
        if not isinstance(enabled, bool):
            raise DepthFusionError("fusion_enabled_must_be_boolean")
        native = _native_source(native_points_base, native_payload, pair, pose_arrival_wall_s,
                                now_sim_s=now_sim_s, now_wall_s=now_wall_s)
        depth_points, depth = load_depth_frame(
            depth_manifest, reference_stamp_ns=native["stamp_ns"],
            now_sim_s=now_sim_s, now_wall_s=now_wall_s,
            **({} if depth_snapshot is None else {"snapshot": depth_snapshot}),
        )
        points = np.concatenate([np.asarray(native_points_base, dtype=float), depth_points])
        candidate = build_perception_payload(
            points, stamp_s=native_payload["stamp_s"],
            arrival_wall_s=min(native["arrival_wall_s"], depth["arrival_wall_s"],
                               native["pose_arrival_wall_s"]),
            producer_metadata=native_payload["producer_metadata"],
        )
        diagnostic.update(accepted=True, reason=None, sources={"rtx": native, "depth": depth},
                          candidate_directions=copy.deepcopy(candidate["directions"]),
                          candidate_geometry=candidate["lidar_geometry"])
        if not enabled:
            return payload
        candidate["source"] = FUSION_SOURCE
        candidate["safety_geometry"] = candidate.pop("lidar_geometry")
        candidate["frame_chain"] = {
            "base": "go2_base", "rtx": copy.deepcopy(native_payload.get("frame_chain")),
            "depth": "fixed optical-to-base calibration; same native integer stamp",
            "operator_map": "native RTX cloud only; no depth endpoints or fabricated free rays",
        }
        candidate["safety_fusion"] = {
            "schema": FUSION_SCHEMA, "exact_integer_stamp": True,
            "reference_stamp_ns": native["stamp_ns"], "sources": {"rtx": native, "depth": depth},
            "native_cloud_points": native["points"], "depth_points": depth["points"],
            "depth_enters_slam": False, "operator_map_native_only": True,
            "interpolation_used": False, "unknown_filled": False,
        }
        diagnostic["control_used"] = True
        candidate["depth_safety_diagnostic"] = diagnostic
        candidate["note"] = ("Safety-only union of real native RTX and simulated downward-camera endpoints; "
                             "physical robot requires matching additional depth hardware. Unknown remains unknown.")
        validate_fusion_contract(candidate, now_sim_s=now_sim_s, now_wall_s=now_wall_s)
        return candidate
    except (ValueError, TypeError, OSError, KeyError, EOFError, AttributeError, OverflowError) as error:
        diagnostic.update(accepted=False, control_used=False, reason=str(error))
        if native_fallback_factory is not None:
            fallback = copy.deepcopy(native_fallback_factory())
            fallback["depth_safety_diagnostic"] = diagnostic
            return fallback
        return payload


def validate_fusion_source_ages(sources, *, now_sim_s, now_wall_s):
    """Recheck original source clocks after work; never renew an arrival."""
    if not isinstance(sources, dict) or set(sources) != {"rtx", "depth"}:
        raise DepthFusionError("fusion_sources_missing")
    stamps = []
    for name, source in sources.items():
        stamp = _integer(source.get("stamp_ns"), name + "_stamp_ns")
        stamps.append(stamp)
        _age(stamp, source.get("arrival_wall_s"), now_sim_s=now_sim_s, now_wall_s=now_wall_s)
    if stamps[0] != stamps[1]:
        raise DepthFusionError("fusion_source_stamp_mismatch")
    _age(stamps[0], sources["rtx"].get("pose_arrival_wall_s"),
         now_sim_s=now_sim_s, now_wall_s=now_wall_s)
    return True


def invalidate_fused_perception(payload, error):
    """An expired joint result is unknown, without another expensive native fit.

    Preserve both original sources, stamps, arrivals and measured geometry for
    audit. No stale clearance remains usable and no source is relabelled native.
    """
    rejected = copy.deepcopy(payload)
    rejected.update(clearance_m=None, observation_state="source_invalid_after_compute",
                    fail_closed=True, producer_error=str(error))
    for entry in rejected.get("directions", {}).values():
        entry.update(clearance_m=None, observation_state="insufficient_observation",
                     reason="source_invalid_after_compute:" + str(error))
    return rejected


def validate_fusion_contract(payload, *, now_sim_s, now_wall_s):
    """Tracker-side independent source/time checks; this never proves floor support."""
    if payload.get("source") != FUSION_SOURCE:
        raise DepthFusionError("fusion_source_unknown")
    fusion = payload.get("safety_fusion") or {}
    if (fusion.get("schema") != FUSION_SCHEMA or fusion.get("exact_integer_stamp") is not True
            or fusion.get("depth_enters_slam") is not False
            or fusion.get("operator_map_native_only") is not True
            or fusion.get("interpolation_used") is not False or fusion.get("unknown_filled") is not False):
        raise DepthFusionError("fusion_contract_invalid")
    stamp = _integer(fusion.get("reference_stamp_ns"), "fusion_reference_stamp_ns")
    if (payload.get("frame_id") != "go2_base"
            or round(_number(payload.get("stamp_s"), "fusion_stamp_s") * 1e9) != stamp):
        raise DepthFusionError("fusion_stamp_or_frame_invalid")
    sources = fusion.get("sources") or {}
    if set(sources) != {"rtx", "depth"}:
        raise DepthFusionError("fusion_sources_missing")
    for source in sources.values():
        if not isinstance(source, dict) or _integer(source.get("stamp_ns"), "source_stamp_ns") != stamp:
            raise DepthFusionError("fusion_source_stamp_mismatch")
        if source.get("frame_id") != "go2_base" or not _digest(source.get("points_sha256")):
            raise DepthFusionError("fusion_source_identity_invalid")
        _integer(source.get("points"), "source_points")
        _age(stamp, source.get("arrival_wall_s"), now_sim_s=now_sim_s, now_wall_s=now_wall_s)
    native, depth = sources["rtx"], sources["depth"]
    pair = native.get("pair") or {}
    producer = native.get("producer_metadata") or {}
    verify_topic_roles(producer.get("cloud_topic"), producer.get("pose_topic"))
    if (native.get("source") != "rtx_lidar_cloud_registered_full" or native.get("native_only") is not True
            or not _digest(native.get("binary_sha256")) or producer.get("stage") != "LIO"
            or producer.get("binary_sha256") != native["binary_sha256"]
            or pair.get("stage") != "LIO" or pair.get("child_frame") != "go2_imu"
            or not pair.get("map_frame")
            or any(_integer(pair.get(k), k) != stamp for k in ("reference_stamp_ns", "cloud_stamp_ns", "pose_stamp_ns"))
            or any(_integer(producer.get(k), k) != stamp for k in ("reference_stamp_ns", "cloud_stamp_ns", "pose_stamp_ns"))):
        raise DepthFusionError("fusion_native_contract_invalid")
    _age(stamp, native.get("pose_arrival_wall_s"), now_sim_s=now_sim_s, now_wall_s=now_wall_s)
    timing = depth.get("timing") or {}
    for reference in (depth.get("depth_reference_time"), depth.get("mask_reference_time"),
                      timing.get("event_reference"), timing.get("reference_after_reads")):
        nearfield.verify_reference_pair(depth.get("native_reference_time"), reference)
    if (timing.get("body_now_substituted") is not False
            or round(_number(depth.get("source_time_s"), "depth_source_time_s") * 1e9) != stamp):
        raise DepthFusionError("fusion_depth_clock_invalid")
    calibration = depth.get("calibration")
    mask = depth.get("self_mask") or {}
    if (depth.get("source") != nearfield.SOURCE or depth.get("native_stamp_preserved") is not True
            or depth.get("reprojection_verified") is not True
            or calibration != nearfield.calibration_receipt()
            or depth.get("calibration_sha256") != _sha_json(calibration)
            or depth.get("self_mask_sha256") != _sha_json(mask)
            or mask.get("mapping_sha256") != depth.get("segmentation_mapping_sha256")
            or "instance_id_to_prim" in mask
            or mask.get("mask_same_frame") is not True or mask.get("legacy_self_box_used") is not False
            or mask.get("self_mask_robot_root") != nearfield.ROBOT_ROOT
            or mask.get("self_occlusion_is_unknown") is not True
            or mask.get("environment_labels_used_for_floor") is not False
            or any(not _digest(depth.get(k)) for k in
                   ("npz_sha256", "manifest_sha256", "segmentation_mapping_sha256"))):
        raise DepthFusionError("fusion_depth_contract_invalid")
    if (payload.get("cloud_points") != native["points"] + depth["points"]
            or fusion.get("native_cloud_points") != native["points"] or fusion.get("depth_points") != depth["points"]
            or payload.get("arrival_wall_s") != min(native["arrival_wall_s"], depth["arrival_wall_s"], native["pose_arrival_wall_s"])):
        raise DepthFusionError("fusion_point_count_or_arrival_invalid")
    if payload.get("corridor") != corridor_contract():
        raise DepthFusionError("fusion_corridor_changed")
    for entry in (payload.get("directions") or {}).values():
        if entry.get("observation_state") != "measured":
            continue
        contract = corridor_contract()
        if (entry.get("azimuth_bins") != 18
                or _number(entry.get("azimuth_coverage_fraction"), "azimuth_fraction") < contract["required_azimuth_fraction"]
                or entry.get("radial_start_m") != contract["forward_m"][0]
                or entry.get("radial_bins_required") != contract["radial_bins"]
                or entry.get("drop_bins") != []):
            raise DepthFusionError("fusion_direction_coverage_invalid")
        counts = entry.get("radial_bin_points")
        if (not isinstance(counts, list) or len(counts) != contract["radial_bins"]
                or any(_number(n, "radial_points") < contract["min_radial_points_per_bin"] for n in counts)):
            raise DepthFusionError("fusion_radial_coverage_invalid")
    return True
