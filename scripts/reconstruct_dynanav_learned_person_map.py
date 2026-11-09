#!/usr/bin/env python3
"""Offline learned person points and recorded LIO poses with upstream provenance."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import re
import sqlite3
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from embodied_navigation.localization_mapping.semantic_mapping import (
    LayeredMap,
    Observation,
    PointMask,
)
from embodied_navigation.perception.learned_person_mapping import (
    forward_pose_provenance,
    learned_person_evidence,
    load_geometry,
    pose_dependency_provenance,
)
from embodied_navigation.perception.person_segmentation import periodic_indices
from embodied_navigation.project import data_root


def load_script(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def write_json(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def read_exact_poses(args, reconstruction):
    # Decoder-only imports: no rclpy.init, node, subscription or sensor publisher.
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    databases = reconstruction.bag_files(args.bag)
    if args.bag.is_dir() and not (args.bag / "metadata.yaml").is_file():
        raise ValueError("Expected a finalized bag with metadata.yaml")
    poses = []
    counts = Counter()
    for file in databases:
        connection = sqlite3.connect(file.as_uri() + "?mode=ro&immutable=1", uri=True)
        try:
            topics = connection.execute(
                "select id,type,serialization_format from topics where name=?",
                (args.odom_topic,),
            ).fetchall()
            for identity, kind, encoding in topics:
                if kind != "nav_msgs/msg/Odometry" or encoding != "cdr":
                    raise ValueError("Expected CDR odometry")
                for message_id, received, blob in connection.execute(
                    "select id,timestamp,data from messages where topic_id=? order by timestamp,id",
                    (identity,),
                ):
                    message = deserialize_message(blob, Odometry)
                    pose = reconstruction.odometry_pose(
                        message,
                        map_frame=args.map_frame,
                        sensor_frame=args.sensor_frame,
                        storage=str(file),
                        message_id=message_id,
                        bag_timestamp_ns=received,
                    )
                    poses.append(pose)
                    counts[pose.error or "valid"] += 1
        finally:
            connection.close()
    if not poses:
        raise ValueError("No estimated poses on selected topic")
    return reconstruction.PoseIndex(poses), databases, dict(counts)


def reconstruct_mode(
    args,
    mode,
    selected,
    source_rows,
    predictions,
    model_receipt,
    pose_provenance,
    pose_index,
    reconstruction,
    query_module,
):
    output = Path(str(args.output_prefix) + "-" + mode)
    store = LayeredMap(
        map_id=output.name, frame_id=args.map_frame, max_observations=len(source_rows)
    )
    rows = []
    counts = Counter()
    inputs = []
    for index in selected:
        source = source_rows[index]
        file = Path(source["path"])
        prediction = predictions / (file.stem + ".npz")
        if digest(file) != source["sha256"]:
            raise ValueError("Keyframe changed since RGB-only inference")
        with np.load(prediction, allow_pickle=False) as data:
            source_ns = int(data["source_ns"])
            masks = data["masks"]
            scores = data["scores"]
            if (
                str(data["source_sha256"]) != source["sha256"]
                or str(data["weights_sha256"]) != model_receipt["weights"]["sha256"]
                or str(data["rgb_sha256"]) != source["rgb_sha256"]
                or float(data["mask_threshold"]) != model_receipt["thresholds"]["mask"]
            ):
                raise ValueError("Learned prediction cache provenance mismatch")
        ns, points, pixels = load_geometry(file, masks.shape[1:])
        if ns != source_ns or ns != source["source_ns"]:
            raise ValueError("Prediction/geometry timestamp mismatch")
        sequence = int(re.fullmatch(r"rgbd_(\d+)\.npz", file.name)[1])
        match = pose_index.match(source_ns, 0)
        row = {
            "keyframe": file.name,
            "source_ns": ns,
            "sequence": sequence,
            "status": match.status,
            "estimated_pose_source_ns": match.odom_source_ns,
            "delta_ns": match.delta_ns,
            "prediction_sha256": digest(prediction),
        }
        inputs.append(
            {
                "keyframe": str(file),
                "keyframe_sha256": source["sha256"],
                "prediction": str(prediction),
                "prediction_sha256": row["prediction_sha256"],
            }
        )
        if match.status == "matched":
            states, identities, labels, instances = learned_person_evidence(
                masks,
                scores,
                pixels,
                ns,
                score_threshold=model_receipt["thresholds"]["score"],
            )
            observation = Observation(
                epoch=1,
                sequence=sequence,
                source_ns=ns,
                pose_source_ns=ns,
                points_sensor=points,
                T_map_sensor=match.pose.transform,
                anchor_id=f"rgbd:{sequence}",
                frame_id=args.sensor_frame,
                calibration_id="rgbd:v1",
                pose_source="slam",
            )
            provenance = {
                "kind": "learned",
                "pose_provenance": pose_provenance,
                "pose_preprocessing_mask_source": pose_provenance[
                    "pose_preprocessing_mask_source"
                ],
                "end_to_end_oracle_free": False,
                "producer": "torchvision.maskrcnn_resnet50_fpn_v2",
                "instance_id_scope": "frame",
                "weights_sha256": model_receipt["weights"]["sha256"],
                "prediction_sha256": row["prediction_sha256"],
                "source_rgb_sha256": source["rgb_sha256"],
                "score_threshold": model_receipt["thresholds"]["score"],
                "mask_threshold": model_receipt["thresholds"]["mask"],
                "projection": "original RGB pixel_flat_indices; no oracle states, IDs or source pose",
            }
            mask = PointMask(
                epoch=1,
                sequence=sequence,
                source_ns=ns,
                produced_ns=ns,
                frame_id=args.sensor_frame,
                calibration_id="rgbd:v1",
                states=states,
                instance_ids=identities,
                labels=labels,
                provenance=provenance,
            )
            result = store.integrate(observation, mask, now_ns=ns)
            row.update(
                mask_status=result["mask_status"],
                persisted=result["persisted"],
                static_points=len(result["static_points"]),
                dynamic_points=len(result["dynamic_points"]),
                unknown_points=len(result["unknown_points"]),
                predicted_instances=instances,
                object_assignments=store.observations[observation.observation_id][
                    "object_ids"
                ],
                pose_storage=match.pose.storage,
                pose_message_id=match.pose.message_id,
            )
            if row["static_points"]:
                raise AssertionError("Unproven background entered static map")
        counts[row["status"]] += 1
        rows.append(row)
    if not counts["matched"]:
        raise ValueError("No exact estimated pose/keyframe matches")
    output.mkdir()
    scene = forward_pose_provenance(store.summary(), pose_provenance)
    scene.update(
        semantic_oracle=False,
        no_GT_pose_used=True,
        mask_source="learned",
        pose_source="slam",
        learned_accuracy_verified=False,
        complete_full_scene=False,
        production_navigation_connected=False,
        scope="Offline direct learned person masks with estimated poses; upstream pose preprocessing disclosed, not established end-to-end oracle-free; UNKNOWN background quarantined",
    )
    static = store.static_points()
    assert static.shape == (0, 3)
    with (output / "static_map.npz").open("xb") as stream:
        np.savez(
            stream,
            points=static,
            pose_source="slam",
            mask_source="learned",
            map_frame=args.map_frame,
        )
    write_json(output / "scene_map.json", scene)
    historical = forward_pose_provenance(
        store.query(label="person", include_expired=True, include_history=True),
        pose_provenance,
    )
    write_json(output / "person_history.json", historical)
    query = query_module.select(scene, label="person", include_expired=True)
    fresh = query_module.select(scene, label="person", include_expired=False)
    write_json(output / "person_query.json", query)
    write_json(output / "person_query_fresh.json", fresh)
    with (output / "person_context.txt").open("x") as stream:
        stream.write(query_module.context_text(query))
    tracks = [
        {
            "object_id": obj["object_id"],
            "retained_history_entries": len(obj["history"]),
            "first_retained_source_ns": obj["history"][0]["source_ns"],
            "last_source_ns": obj["source_ns"],
            "expired_at_final_query": obj["expired"],
        }
        for obj in historical["objects"]
    ]
    receipt = {
        "schema": "offline_learned_person_scene_reconstruction/v1",
        "mode": mode,
        "pose_source": "recorded native LIO estimate, exact header match",
        "pose_frame": args.map_frame,
        "pose_provenance": pose_provenance,
        "pose_preprocessing_mask_source": pose_provenance[
            "pose_preprocessing_mask_source"
        ],
        "end_to_end_oracle_free": False,
        "no_GT_pose_used_scope": "No archived GT pose used directly; upstream preprocessing may use oracle semantics",
        "sensor_frame": args.sensor_frame,
        "odom_topic": args.odom_topic,
        "pairing_counts": dict(counts),
        "selected_frames": len(selected),
        "available_frames": len(source_rows),
        "interpolation": False,
        "extrapolation": False,
        "pose_fallback": False,
        "oracle_states_or_instance_IDs_read": False,
        "mask_time_semantics": "Source-aligned offline integration: produced_ns=source_ns, now_ns=source_ns; not measured live mask latency or delivery freshness",
        "archived_T_map_sensor_read": False,
        "geometry_allowlist": ["source_ns", "points_sensor", "pixel_flat_indices"],
        "prediction_model_inference_frames_this_run": 0,
        "original_prediction_model_inference_frames": model_receipt[
            "inference_frames_this_run"
        ],
        "model_weights": model_receipt["weights"],
        "thresholds": model_receipt["thresholds"],
        "mask_provenance": {"kind": "learned", "instance_id_scope": "frame"},
        "association": {
            "method": "existing LayeredMap label+3D visible-centroid distance, one-to-one per frame",
            "association_distance_m": store.association_distance,
            "object_ttl_ns": store.object_ttl_ns,
            "retained_history_length": store.history_length,
            "oracle_or_2D_tracker_IDs_used": False,
        },
        "point_counts": scene["observation_point_counts"],
        "persistent_static_points": len(static),
        "person_IDs_allocated": len(store.objects),
        "person_IDs_fresh_at_final_query": len(fresh["objects"]),
        "known_scene_person_count_for_evaluation_only": args.scene_person_count,
        "allocated_ID_excess_above_known_scene_person_count": max(
            0, len(store.objects) - args.scene_person_count
        )
        if args.scene_person_count is not None
        else None,
        "ID_count_limit": "ID multiplicity may reflect fragmentation, false positives, centroid/pose drift or occlusion; not a3D re-ID or IDF1 result. Known scene count never enters association.",
        "tracks": tracks,
        "inputs": inputs,
        "frames": rows,
        "semantic_oracle": False,
        "no_GT_pose_used": True,
        "live_oracle_replaced": False,
        "production_navigation_connected": False,
        "schedule_scope": "all saved keyframes"
        if mode == "all"
        else "offline anchored2s selected keyframes from existing predictions, no new live2s or event-trigger validation; skipped frames have no new observation",
        "scope": "Offline learned dynamic-person scene context only; conservative UNKNOWN background cannot supply a learned static navigation map.",
    }
    write_json(output / "receipt.json", receipt)
    return output, receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evaluation", type=Path, required=True)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output-prefix", type=Path, required=True)
    parser.add_argument(
        "--mode", choices=("all", "periodic2s", "compare"), default="compare"
    )
    parser.add_argument(
        "--odom-topic", default="/dynanav/lio_observer/aft_mapped_to_init"
    )
    parser.add_argument("--map-frame", default="dynanav_lio_map")
    parser.add_argument("--sensor-frame", default="go2_imu")
    parser.add_argument(
        "--pose-launch-receipt",
        type=Path,
        help="Estimator launch receipt; defaults to launch.json beside the bag",
    )
    parser.add_argument(
        "--pose-preprocessing-receipt",
        type=Path,
        help="Producer receipt linking its static frontend topic to mask_source; absent means unknown",
    )
    parser.add_argument(
        "--scene-person-count",
        type=int,
        help="Evaluation annotation only; never used for association",
    )
    args = parser.parse_args(argv)
    args.output_prefix = args.output_prefix.expanduser().resolve()
    args.bag = args.bag.resolve()
    args.evaluation = args.evaluation.resolve()
    if not args.output_prefix.is_relative_to(data_root().resolve()):
        parser.error("Output must be below external project data root")
    modes = ["all", "periodic2s"] if args.mode == "compare" else [args.mode]
    if any(Path(str(args.output_prefix) + "-" + mode).exists() for mode in modes):
        parser.error("Output already exists; never overwrite learned evidence")
    if args.scene_person_count is not None and args.scene_person_count < 0:
        parser.error("Scene evaluation count must be nonnegative")
    model_receipt = json.loads((args.evaluation / "receipt.json").read_text())
    if not model_receipt.get("complete") or model_receipt.get(
        "inference_frames_this_run"
    ) != len(model_receipt["sources"]):
        parser.error("Expected complete original per-frame learned inference receipt")
    rows = sorted(model_receipt["sources"], key=lambda x: x["source_ns"])
    selected_periodic = periodic_indices([x["source_ns"] for x in rows])
    reconstruction = load_script(
        "learned_reconstruction_pose_io",
        ROOT / "scripts/reconstruct_dynanav_slam_map.py",
    )
    query = load_script("learned_scene_query", ROOT / "scripts/query_scene_map.py")
    index, databases, pose_counts = read_exact_poses(args, reconstruction)
    launch = args.pose_launch_receipt
    if launch is None:
        parent = args.bag.parent if args.bag.is_dir() else args.bag.parent.parent
        candidate = parent / "launch.json"
        launch = candidate if candidate.is_file() else None
    pose_provenance = pose_dependency_provenance(
        bag=args.bag,
        topic=args.odom_topic,
        launch_receipt=launch,
        preprocessing_receipt=args.pose_preprocessing_receipt,
    )
    output_records = []
    for mode in modes:
        selected = list(range(len(rows))) if mode == "all" else selected_periodic
        output, receipt = reconstruct_mode(
            args,
            mode,
            selected,
            rows,
            args.evaluation / "predictions",
            model_receipt,
            pose_provenance,
            index,
            reconstruction,
            query,
        )
        verification = {
            "source_receipt": {
                "path": str(args.evaluation / "receipt.json"),
                "sha256": digest(args.evaluation / "receipt.json"),
            },
            "estimated_pose_bags": [
                {"path": str(p), "sha256": digest(p)} for p in databases
            ],
            "pose_counts": pose_counts,
            "source_files": [
                {"path": str(p), "sha256": digest(p)}
                for p in [
                    Path(__file__),
                    ROOT
                    / "src/embodied_navigation/perception/learned_person_mapping.py",
                    ROOT
                    / "src/embodied_navigation/localization_mapping/semantic_mapping/core.py",
                    ROOT / "scripts/reconstruct_dynanav_slam_map.py",
                    ROOT / "scripts/query_scene_map.py",
                ]
            ],
        }
        write_json(output / "provenance.json", verification)
        output_records.append(
            {
                "output": str(output),
                "pairing": receipt["pairing_counts"],
                "person_IDs": receipt["person_IDs_allocated"],
                "point_counts": receipt["point_counts"],
            }
        )
    print(json.dumps(output_records, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
