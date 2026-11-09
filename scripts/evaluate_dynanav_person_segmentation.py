#!/usr/bin/env python3
"""Offline RGB-only person masks versus sparse oracle samples; never live control."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
import traceback
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from embodied_navigation.perception.person_segmentation import (
    MaskIoUTracker,
    confusion,
    load_evaluation_frame,
    metric_rates,
    periodic_indices,
    project_masks,
)
from embodied_navigation.project import data_root

OFFICIAL_WEIGHT_URL = (
    "https://download.pytorch.org/models/maskrcnn_resnet50_fpn_v2_coco-73cbd019.pth"
)


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def write_json(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def infer_all(args, frames, output, weight_sha):
    """Torch is deliberately lazy; only RGB tensors are passed to the model."""
    import torch
    import torchvision

    if args.device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("Explicit CUDA device requested but CUDA is unavailable")
    torch.set_num_threads(args.torch_threads)
    model = torchvision.models.detection.maskrcnn_resnet50_fpn_v2(
        weights=None,
        weights_backbone=None,
        box_score_thresh=0.05,
    )
    model.load_state_dict(
        torch.load(args.weights, map_location="cpu", weights_only=True), strict=True
    )
    model.eval().to(args.device)
    metadata = {
        "torch": torch.__version__,
        "torchvision": torchvision.__version__,
        "device": args.device,
        "torch_threads": args.torch_threads,
        "model": "torchvision.maskrcnn_resnet50_fpn_v2",
        "weights_auto_download": False,
        "model_person_label": 1,
        "model_box_score_thresh": 0.05,
        "model_transform": {
            "min_size": list(model.transform.min_size),
            "max_size": model.transform.max_size,
            "image_mean": model.transform.image_mean,
            "image_std": model.transform.image_std,
        },
        "CUDA_device_name": torch.cuda.get_device_name(args.device)
        if args.device.startswith("cuda")
        else None,
    }
    prediction_dir = output / "predictions"
    prediction_dir.mkdir()
    entries = {}
    with torch.inference_mode():
        for index, frame in enumerate(frames):
            # No oracle mask, path, pose, depth or sample index reaches this call.
            rgb = (
                torch.from_numpy(np.ascontiguousarray(frame.rgb))
                .permute(2, 0, 1)
                .float()
                .div(255)
            )
            if args.device.startswith("cuda"):
                torch.cuda.synchronize(args.device)
            started = time.perf_counter()
            prediction = model([rgb.to(args.device)])[0]
            person = prediction["labels"] == 1
            scores = prediction["scores"][person].detach().cpu().numpy()
            boxes = prediction["boxes"][person].detach().cpu().numpy()
            masks = (
                (prediction["masks"][person, 0] >= args.mask_threshold)
                .detach()
                .cpu()
                .numpy()
            )
            if args.device.startswith("cuda"):
                torch.cuda.synchronize(args.device)
            elapsed = time.perf_counter() - started
            target = prediction_dir / (frame.path.stem + ".npz")
            with target.open("xb") as stream:
                np.savez_compressed(
                    stream,
                    masks=masks,
                    scores=scores,
                    boxes=boxes,
                    source_ns=np.int64(frame.source_ns),
                    rgb_sha256=hashlib.sha256(frame.rgb.tobytes()).hexdigest(),
                    source_sha256=digest(frame.path),
                    weights_sha256=weight_sha,
                    mask_threshold=np.float64(args.mask_threshold),
                    inference_seconds=np.float64(elapsed),
                )
            entries[frame.path.name] = target
            print(
                json.dumps(
                    {
                        "frame": index + 1,
                        "total": len(frames),
                        "source_ns": frame.source_ns,
                        "person_scores": scores.tolist(),
                        "inference_seconds": elapsed,
                    }
                ),
                flush=True,
            )
    del model
    return entries, metadata


def reuse_predictions(args, frames, weight_sha):
    entries = {}
    for frame in frames:
        path = args.reuse_predictions / "predictions" / (frame.path.stem + ".npz")
        with np.load(path, allow_pickle=False) as data:
            if (
                int(data["source_ns"]) != frame.source_ns
                or str(data["source_sha256"]) != digest(frame.path)
                or str(data["rgb_sha256"])
                != hashlib.sha256(frame.rgb.tobytes()).hexdigest()
                or str(data["weights_sha256"]) != weight_sha
                or float(data["mask_threshold"]) != args.mask_threshold
            ):
                raise ValueError(f"Prediction cache provenance mismatch: {path}")
        entries[frame.path.name] = path
    source_receipt = json.loads((args.reuse_predictions / "receipt.json").read_text())
    return entries, {
        "reused_from": str(args.reuse_predictions),
        "source_model_runtime": source_receipt.get("model_runtime"),
        "model_was_executed_this_run": False,
    }


def evaluate_subset(args, frames, indices, predictions, mode):
    tracker = MaskIoUTracker(args.track_iou)
    counts, rows = Counter(), []
    for index in indices:
        frame = frames[index]
        with np.load(predictions[frame.path.name], allow_pickle=False) as data:
            masks, scores, boxes = data["masks"], data["scores"], data["boxes"]
            elapsed = float(data["inference_seconds"])
        if masks.shape[1:] != frame.rgb.shape[:2] or masks.dtype != bool:
            raise ValueError(
                "Prediction masks must remain in original RGB pixel coordinates"
            )
        active = scores >= args.score_threshold
        selected = masks[active]
        point_masks = project_masks(selected, frame.pixel_flat_indices)
        union_points = np.any(point_masks, axis=0)
        metrics = confusion(union_points, frame.states)
        counts.update(metrics)
        ids = tracker.update(selected, frame.source_ns)
        instances = [
            {
                "detection_index": int(j),
                "track_2d_id": int(ident),
                "score": float(scores[j]),
                "bbox_xyxy": boxes[j].tolist(),
                "mask_pixels": int(np.sum(masks[j])),
                "sampled_point_indices": np.flatnonzero(point_masks[k]).tolist(),
            }
            for k, (j, ident) in enumerate(zip(np.flatnonzero(active), ids))
        ]
        rows.append(
            {
                "keyframe": frame.path.name,
                "source_ns": frame.source_ns,
                "prediction": str(predictions[frame.path.name]),
                "prediction_sha256": digest(predictions[frame.path.name]),
                "all_returned_person_scores": scores.tolist(),
                "instances": instances,
                "predicted_person_mask_pixels": int(np.sum(np.any(selected, axis=0)))
                if len(selected)
                else 0,
                "predicted_person_sample_indices": np.flatnonzero(
                    union_points
                ).tolist(),
                "background_label": "unproven/UNKNOWN; not assigned STATIC",
                "oracle_person_samples": int(np.sum(frame.states == 2)),
                "sampled_metrics": metric_rates(metrics),
                "recorded_inference_seconds": elapsed,
            }
        )
    skipped = [i for i in range(len(frames)) if i not in set(indices)]
    result = {
        "mode": mode,
        "selected_frames": len(rows),
        "total_available_frames": len(frames),
        "sampled_micro_metrics": metric_rates(counts),
        "selected_inference_seconds_sum": sum(
            row["recorded_inference_seconds"] for row in rows
        ),
        "skipped_frames": len(skipped),
        "skipped_frames_with_oracle_person": sum(
            np.any(frames[i].states == 2) for i in skipped
        ),
        "skipped_oracle_person_samples": sum(
            int(np.sum(frames[i].states == 2)) for i in skipped
        ),
        "schedule": "all frames"
        if mode == "all"
        else "first frame at/after anchored2s deadlines using original source_ns; no propagation or interpolation",
        "score_scope": "Known sampled cloud pixels only, not dense-image ground truth; unscheduled frames have no new prediction and remain unobserved.",
        "latency_scope": "Offline per-frame inference timing; no live deadline or periodic mask delivery guarantee",
        "frames": rows,
    }
    # Convert numpy boolean sums without changing score values.
    result["skipped_frames_with_oracle_person"] = int(
        result["skipped_frames_with_oracle_person"]
    )
    return result


def visualizations(output, frames, result, predictions, args):
    from PIL import Image, ImageDraw

    rows = result["frames"]
    ranks = [
        sorted(rows, key=lambda r: r["oracle_person_samples"], reverse=True),
        sorted(rows, key=lambda r: r["sampled_metrics"]["fn"], reverse=True),
        sorted(rows, key=lambda r: r["sampled_metrics"]["fp"], reverse=True),
    ]
    chosen = []
    for rank in ranks:
        for row in rank[:2]:
            if row not in chosen:
                chosen.append(row)
    by_name = {f.path.name: f for f in frames}
    folder = output / "visualizations"
    folder.mkdir()
    for row in chosen:
        frame = by_name[row["keyframe"]]
        with np.load(predictions[frame.path.name], allow_pickle=False) as data:
            masks = data["masks"][data["scores"] >= args.score_threshold]
        union = (
            np.any(masks, axis=0) if len(masks) else np.zeros(frame.rgb.shape[:2], bool)
        )
        predicted = frame.rgb.copy()
        predicted[union] = (
            0.55 * predicted[union] + 0.45 * np.array([0, 255, 0])
        ).astype(np.uint8)
        oracle = frame.rgb.copy().reshape(-1, 3)
        oracle[frame.pixel_flat_indices[frame.states == 2]] = [255, 0, 0]
        oracle[frame.pixel_flat_indices[frame.states == 0]] = [255, 255, 0]
        oracle = oracle.reshape(frame.rgb.shape)
        width, height = frame.rgb.shape[1], frame.rgb.shape[0]
        canvas = Image.new("RGB", (width * 3, height + 40), "black")
        for j, image in enumerate([frame.rgb, predicted, oracle]):
            canvas.paste(Image.fromarray(image), (j * width, 40))
        draw = ImageDraw.Draw(canvas)
        draw.text(
            (5, 3),
            f"{frame.path.name} t={frame.source_ns / 1e9:.6f}s; TP/FP/FN={row['sampled_metrics']['tp']}/{row['sampled_metrics']['fp']}/{row['sampled_metrics']['fn']}",
            fill="white",
        )
        for j, label in enumerate(
            [
                "Actual raw RGB",
                "RGB-only predicted person",
                "Sparse oracle person samples red; UNKNOWN yellow",
            ]
        ):
            draw.text((j * width + 5, 20), label, fill="white")
        target = folder / (frame.path.stem + ".png")
        canvas.save(target)
        row["visualization"] = str(target)
    return {
        "selection": "top2 each by oracle person sample count, false negatives and false positives; unique frames",
        "files": [r["visualization"] for r in chosen],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--keyframes", type=Path, required=True)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--expected-sha256")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--device", default="cpu", help="Explicit cpu or cuda:N; no automatic fallback"
    )
    parser.add_argument(
        "--mode", choices=("all", "periodic2s", "compare"), default="compare"
    )
    parser.add_argument("--score-threshold", type=float, default=0.5)
    parser.add_argument("--mask-threshold", type=float, default=0.5)
    parser.add_argument("--track-iou", type=float, default=0.25)
    parser.add_argument("--torch-threads", type=int, default=4)
    parser.add_argument("--oracle-person-root", default="/World/DynamicPedestrian")
    parser.add_argument("--reuse-predictions", type=Path)
    args = parser.parse_args(argv)
    if (
        not all(
            0 < v <= 1
            for v in [args.score_threshold, args.mask_threshold, args.track_iou]
        )
        or args.torch_threads < 1
    ):
        parser.error(
            "Explicit thresholds must be within(0,1] and torch threads positive"
        )
    if args.device != "cpu" and not (
        args.device.startswith("cuda:") and args.device[5:].isdigit()
    ):
        parser.error("Device must be cpu or cuda:N")
    args.output = args.output.expanduser().resolve()
    args.weights = args.weights.expanduser().resolve()
    if not args.output.is_relative_to(data_root().resolve()) or args.output.exists():
        parser.error(
            "Output must be a new directory below PROJECT_DATA_ROOT/embodied-navigation"
        )
    if not args.weights.is_file():
        parser.error(
            "Explicit local weight file is missing; this entry never downloads"
        )
    weight_sha = digest(args.weights)
    if args.expected_sha256 and weight_sha != args.expected_sha256:
        parser.error("Local weight SHA256 does not match expected identity")
    frames = sorted(
        [
            load_evaluation_frame(p, args.oracle_person_root)
            for p in args.keyframes.glob("rgbd_*.npz")
        ],
        key=lambda f: f.source_ns,
    )
    if not frames:
        parser.error("No saved RGB keyframes")
    periodic = periodic_indices([f.source_ns for f in frames])
    inference_indices = (
        periodic if args.mode == "periodic2s" else list(range(len(frames)))
    )
    args.output.mkdir(parents=True)
    started = time.perf_counter()
    receipt = {
        "schema": "offline_RGB_only_person_segmentation/v1",
        "complete": False,
        "input_keyframes": str(args.keyframes.resolve()),
        "weights": {
            "path": str(args.weights),
            "bytes": args.weights.stat().st_size,
            "sha256": weight_sha,
            "official_reference_url": OFFICIAL_WEIGHT_URL,
        },
        "mode": args.mode,
        "command_line": sys.argv,
        "oracle_person_root": args.oracle_person_root,
        "thresholds": {
            "score": args.score_threshold,
            "mask": args.mask_threshold,
            "adjacent_2d_track_iou": args.track_iou,
        },
        "model_input": "RGB uint8 only -> float RGB[0,1]; no depth/pose/oracle labels/model prompt",
        "oracle_usage": "DYNAMIC person samples only for evaluation; UNKNOWN excluded, known STATIC supplies negative samples",
        "prediction_background": "unproven/UNKNOWN, never certified STATIC",
        "no_SDK_application_or_ROS": True,
        "images_uploaded": False,
        "source_files": [
            {"path": str(p), "sha256": digest(p)}
            for p in [
                Path(__file__),
                ROOT / "src/embodied_navigation/perception/person_segmentation.py",
            ]
        ],
        "sources": [
            {
                "path": str(f.path),
                "source_ns": f.source_ns,
                "sha256": digest(f.path),
                "rgb_sha256": hashlib.sha256(f.rgb.tobytes()).hexdigest(),
            }
            for f in frames
        ],
        "scope": "Offline diagnostic, sparse oracle comparison; not a live replacement for oracle, full-image segmentation benchmark, 3D re-identification or control validation",
    }
    try:
        selected = [frames[i] for i in inference_indices]
        predictions, runtime = (
            reuse_predictions(args, selected, weight_sha)
            if args.reuse_predictions
            else infer_all(args, selected, args.output, weight_sha)
        )
        receipt["model_runtime"] = runtime
        modes = ["all", "periodic2s"] if args.mode == "compare" else [args.mode]
        results = {
            mode: evaluate_subset(
                args,
                frames,
                list(range(len(frames))) if mode == "all" else periodic,
                predictions,
                mode,
            )
            for mode in modes
        }
        receipt["visualizations"] = visualizations(
            args.output, frames, results[modes[0]], predictions, args
        )
        for mode, value in results.items():
            write_json(args.output / (mode + ".json"), value)
        receipt.update(
            complete=True,
            inference_frames_this_run=0 if args.reuse_predictions else len(selected),
            prediction_reuse="compare reuses the same stored predictions for periodic subset; no second inference pass",
            results={
                m: {k: v for k, v in value.items() if k != "frames"}
                for m, value in results.items()
            },
            elapsed_wall_seconds=time.perf_counter() - started,
        )
    except BaseException as error:
        receipt.update(
            error=repr(error),
            traceback=traceback.format_exc(),
            elapsed_wall_seconds=time.perf_counter() - started,
        )
        write_json(args.output / "receipt.json", receipt)
        raise
    write_json(args.output / "receipt.json", receipt)
    print(
        json.dumps(
            {
                "output": str(args.output),
                "complete": True,
                "results": receipt["results"],
            }
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
