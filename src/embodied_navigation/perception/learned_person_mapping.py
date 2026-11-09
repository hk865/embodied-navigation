"""Convert saved RGB-only person predictions into conservative point evidence."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from embodied_navigation.perception.person_segmentation import project_masks


def load_geometry(path, image_shape):
    """Read geometry only; never access oracle states/IDs or archived poses."""
    with np.load(Path(path), allow_pickle=False) as data:
        stamp = data["source_ns"]
        if stamp.shape != () or not np.issubdtype(stamp.dtype, np.integer):
            raise ValueError("Expected integer source timestamp")
        source_ns = int(stamp)
        points = np.array(data["points_sensor"], dtype=float, copy=True)
        indices = data["pixel_flat_indices"].copy()
    if (
        source_ns < 0
        or points.ndim != 2
        or points.shape[1] != 3
        or not np.isfinite(points).all()
    ):
        raise ValueError("Expected nonnegative timestamp and finite Nx3 sensor points")
    if indices.shape != (len(points),) or not np.issubdtype(indices.dtype, np.integer):
        raise ValueError("One original integer pixel index per sensor point required")
    if (
        len(image_shape) != 2
        or any(int(x) <= 0 for x in image_shape)
        or np.any(indices < 0)
        or np.any(indices >= image_shape[0] * image_shape[1])
        or len(np.unique(indices)) != len(indices)
    ):
        raise ValueError("Pixel indices must be unique and match original image bounds")
    return source_ns, points, indices


def learned_person_evidence(
    masks, scores, pixel_flat_indices, source_ns, *, score_threshold=0.5
):
    """Only predicted person points are DYNAMIC; all other points stay UNKNOWN.

    Overlapping instances assign each sample once to the highest-scoring mask.
    IDs are scoped to one frame, never borrowed from oracle or 2D tracker IDs.
    """
    masks, scores = np.asarray(masks), np.asarray(scores)
    if (
        masks.dtype != bool
        or masks.ndim != 3
        or scores.shape != (len(masks),)
        or not np.isfinite(scores).all()
        or not 0 < score_threshold <= 1
    ):
        raise ValueError("Expected finite per-instance scores and boolean image masks")
    if source_ns < 0:
        raise ValueError("Nonnegative timestamp required")
    projected = project_masks(masks, pixel_flat_indices)
    states = np.zeros(projected.shape[1], dtype=np.uint8)
    identities = [""] * len(states)
    labels = {}
    instances = []
    for index in sorted(
        np.flatnonzero(scores >= score_threshold),
        key=lambda j: (-float(scores[j]), int(j)),
    ):
        owned = projected[index] & (states == 0)
        if not np.any(owned):
            continue
        identity = f"rgb-person:{source_ns}:{int(index)}"
        states[owned] = 2
        for j in np.flatnonzero(owned):
            identities[j] = identity
        labels[identity] = "person"
        instances.append(
            {
                "source_instance_id": identity,
                "prediction_mask_index": int(index),
                "score": float(scores[index]),
                "owned_point_count": int(np.sum(owned)),
            }
        )
    return states, tuple(identities), labels, instances


def pose_dependency_provenance(
    *, bag, topic, launch_receipt=None, preprocessing_receipt=None
):
    """Describe recorded estimator inputs; unknown evidence never means oracle-free.

    A native estimate without a GT-pose subscription may still depend on
    oracle semantic preprocessing. Topic names alone do not establish that
    preprocessing source, so producer and consumer receipts must agree.
    """
    import hashlib
    import json

    def read_receipt(path):
        if path is None:
            return {}, None
        path = Path(path).resolve(strict=True)
        content = path.read_bytes()
        return json.loads(content), {
            "path": str(path),
            "bytes": len(content),
            "sha256": hashlib.sha256(content).hexdigest(),
        }

    launch, launch_source = read_receipt(launch_receipt)
    producer, producer_source = read_receipt(preprocessing_receipt)
    inputs = list(launch.get("sensor_inputs", []))
    remaps = {}
    for argument in launch.get("command", []):
        if ":=" in argument:
            source, target = argument.split(":=", 1)
            remaps[source] = target
    effective_inputs = [remaps.get(topic, topic) for topic in inputs]
    preprocessing_source = "unknown"
    if launch_source and producer_source:
        frontend = producer.get("static_frontend_topic")
        if not frontend or frontend not in effective_inputs:
            raise ValueError(
                "Preprocessing receipt does not match effective estimator sensor input"
            )
        value = producer.get("mask_source")
        if value in ("sim_oracle", "learned", "geometric", "none"):
            preprocessing_source = value
    return {
        "pose_kind": "recorded native estimate, not archived GT-pose fallback",
        "bag": str(Path(bag).resolve()),
        "odometry_topic": topic,
        "native_mode": launch.get("mode", "unknown"),
        "native_core_sha256": launch.get("core_sha256"),
        "declared_direct_GT_pose_input": launch.get("ground_truth_input"),
        "configured_sensor_inputs": inputs,
        "effective_sensor_inputs": effective_inputs,
        "pose_preprocessing_mask_source": preprocessing_source,
        "end_to_end_oracle_free": False,
        "oracle_free_status": "false_due_to_sim_oracle_pose_preprocessing"
        if preprocessing_source == "sim_oracle"
        else "not_established",
        "consumer_launch_receipt": launch_source,
        "preprocessing_producer_receipt": producer_source,
        "evidence_scope": "Matched recorded producer/consumer receipts; not a full binary attestation. Producer diagnostic-map GT pose is not asserted as an estimator pose input.",
    }


def forward_pose_provenance(payload, pose_provenance):
    """Return a provenance-annotated view without altering masks, poses or IDs.

    The library's sim_oracle_used describes direct observation labels. This
    view also discloses transitive pose preprocessing in query/context sources.
    """
    import copy

    result = copy.deepcopy(payload)
    dependency = copy.deepcopy(pose_provenance)
    dependency["end_to_end_oracle_free"] = False
    source = dependency.get("pose_preprocessing_mask_source", "unknown")
    summary = result.setdefault("provenance_summary", {})
    direct = bool(
        summary.get(
            "direct_observation_sim_oracle_used", summary.get("sim_oracle_used", False)
        )
    )
    summary.update(
        direct_observation_sim_oracle_used=direct,
        pose_preprocessing_mask_source=source,
        end_to_end_oracle_free=False,
        # Unknown dependencies cannot silently produce a false oracle-free flag.
        sim_oracle_used=True if direct or source == "sim_oracle" else None,
        sim_oracle_used_scope="direct observations plus transitive pose preprocessing; unknown chain is null",
        pose_provenance=dependency,
    )
    result.update(
        pose_preprocessing_mask_source=source,
        end_to_end_oracle_free=False,
        pose_provenance=dependency,
        semantic_oracle_scope="Direct semantic masks only; estimated-pose preprocessing is disclosed separately",
        no_GT_pose_used_scope="No archived GT pose read by reconstruction; does not mean upstream oracle-free",
    )
    for obj in result.get("objects", []):
        for entry in [obj, *obj.get("history", [])]:
            entry.update(
                pose_preprocessing_mask_source=source,
                end_to_end_oracle_free=False,
                pose_provenance=copy.deepcopy(dependency),
            )
            entry.setdefault("provenance", {}).update(
                pose_preprocessing_mask_source=source,
                end_to_end_oracle_free=False,
                pose_provenance=copy.deepcopy(dependency),
            )
    return result
