#!/usr/bin/env python3
"""Persistent local RGB-only person inference worker; no SDK app, ROS or downloads."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import signal
import sys
import time
import traceback
import uuid
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from embodied_navigation.perception.person_worker import (
    PersonTemporalEvidence,
    atomic_json,
    atomic_npz,
    event_log,
    rgb_image,
    rgb_sha256,
)
from embodied_navigation.project import data_root


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


class TorchvisionPersonModel:
    def __init__(self, args):
        import torch
        import torchvision

        self.torch = torch
        self.device = args.device
        self.mask_threshold = args.mask_threshold
        self.low_score = args.score_threshold
        self.tile_grid = args.tile_grid
        self.tile_mask_iou = args.tile_mask_iou
        self.total_forward_calls = 0
        self.last_request_metadata = {}
        if args.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("Explicit CUDA device unavailable")
        torch.set_num_threads(args.torch_threads)
        self.model = torchvision.models.detection.maskrcnn_resnet50_fpn_v2(
            weights=None, weights_backbone=None, box_score_thresh=0.05
        )
        self.model.load_state_dict(
            torch.load(args.weights, map_location="cpu", weights_only=True), strict=True
        )
        self.model.eval().to(args.device)
        self.metadata = {
            "torch": torch.__version__,
            "torchvision": torchvision.__version__,
            "device": args.device,
            "model": "torchvision.maskrcnn_resnet50_fpn_v2",
            "model_load_count": 1,
            "auto_download": False,
            "model_input": "RGB only, no depth/pose/oracle",
            "model_box_score_thresh": 0.05,
            "retained_mask_score_threshold": args.score_threshold,
            "mask_threshold": args.mask_threshold,
            "high_score_threshold": args.high_score,
            "background": "UNKNOWN",
            "tile_grid": args.tile_grid,
            "tile_mask_iou": args.tile_mask_iou,
        }

    def _predict_view(self, rgb):
        torch = self.torch
        started = time.monotonic_ns()
        started_wall = time.time_ns()
        with torch.inference_mode():
            tensor = (
                torch.from_numpy(rgb.copy())
                .permute(2, 0, 1)
                .float()
                .div(255)
                .to(self.device)
            )
            gpu_start = (
                torch.cuda.Event(enable_timing=True)
                if self.device.startswith("cuda")
                else None
            )
            gpu_end = (
                torch.cuda.Event(enable_timing=True) if gpu_start is not None else None
            )
            if gpu_start is not None:
                gpu_start.record()
            prediction = self.model([tensor])[0]
            if gpu_end is not None:
                gpu_end.record()
            person = prediction["labels"] == 1
            all_scores = prediction["scores"][person].detach().cpu().numpy()
            selected = person & (prediction["scores"] >= self.low_score)
            scores = prediction["scores"][selected].detach().cpu().numpy()
            boxes = prediction["boxes"][selected].detach().cpu().numpy()
            masks = (
                (prediction["masks"][selected, 0] >= self.mask_threshold)
                .detach()
                .cpu()
                .numpy()
            )
            if self.device.startswith("cuda"):
                torch.cuda.synchronize(self.device)
        self.total_forward_calls += 1
        timing = {
            "forward_call_index": self.total_forward_calls,
            "started_wall_ns": started_wall,
            "finished_wall_ns": time.time_ns(),
            "wall_seconds": (time.monotonic_ns() - started) / 1e9,
            "gpu_model_forward_seconds": gpu_start.elapsed_time(gpu_end) / 1000
            if gpu_start is not None
            else None,
            "retained_person_count": len(scores),
            "all_person_scores": all_scores.tolist(),
        }
        return masks, scores, boxes, all_scores, timing

    def predict(self, rgb, *, event_sink=None):
        from embodied_navigation.perception.person_tiling import (
            inference_views,
            lift_masks,
            merge_mask_candidates,
        )

        parts, timings, all_person_scores = [], [], []
        for name, crop in inference_views(rgb.shape, self.tile_grid):
            x0, y0, x1, y1 = crop
            masks, scores, boxes, raw_scores, timing = self._predict_view(
                rgb[y0:y1, x0:x1]
            )
            timing.update(view=name, crop_xyxy=list(crop))
            timings.append(timing)
            if event_sink is not None:
                event_sink(timing)
            all_person_scores.append(raw_scores)
            if self.tile_grid == 1:
                parts.append({"masks": masks, "scores": scores, "boxes": boxes})
            else:
                lifted, translated = lift_masks(masks, boxes, crop, rgb.shape)
                parts.append(
                    {
                        "masks": lifted,
                        "scores": scores,
                        "boxes": translated,
                        "view": name,
                        "crop": crop,
                    }
                )
        if self.tile_grid == 1:
            masks, scores, boxes = (
                parts[0][key] for key in ["masks", "scores", "boxes"]
            )
            merging = {
                "applied": False,
                "reason": "grid1 preserves original full-frame output",
            }
        else:
            masks, scores, boxes, merging = merge_mask_candidates(
                parts, self.tile_mask_iou
            )
        self.last_request_metadata = {
            "tile_grid": self.tile_grid,
            "model_forward_calls_this_request": len(timings),
            "model_forward_calls_total": self.total_forward_calls,
            "forward_passes": timings,
            "summed_gpu_model_forward_seconds": sum(
                t["gpu_model_forward_seconds"] for t in timings
            )
            if self.device.startswith("cuda")
            else None,
            "summed_forward_wall_seconds": sum(t["wall_seconds"] for t in timings),
            "tile_merge": merging,
        }
        return masks, scores, boxes, np.concatenate(all_person_scores)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--channel", type=Path, required=True)
    p.add_argument("--weights", type=Path, required=True)
    p.add_argument("--expected-sha256", required=True)
    p.add_argument("--device", default="cpu")
    p.add_argument("--score-threshold", type=float, default=0.1)
    p.add_argument("--high-score", type=float, default=0.5)
    p.add_argument("--mask-threshold", type=float, default=0.5)
    p.add_argument(
        "--tile-grid",
        type=int,
        choices=(1, 2),
        default=1,
        help="1: full frame; 2: full frame plus four quadrant forwards (experimental)",
    )
    p.add_argument("--tile-mask-iou", type=float, default=0.5)
    p.add_argument("--temporal-iou", type=float, default=0.25)
    p.add_argument("--temporal-max-gap", type=float, default=0.75)
    p.add_argument("--torch-threads", type=int, default=4)
    p.add_argument(
        "--max-requests",
        type=int,
        default=0,
        help="0 serves until a stop request; positive count for standalone smoke checks",
    )
    args = p.parse_args(argv)
    args.channel = args.channel.expanduser().resolve()
    args.weights = args.weights.expanduser().resolve()
    if not args.channel.is_relative_to(data_root().resolve()) or args.channel.exists():
        p.error("Channel must be a new directory below external project data root")
    if args.device != "cpu" and not (
        args.device.startswith("cuda:") and args.device[5:].isdigit()
    ):
        p.error("Use explicit cpu or cuda:N")
    if (
        not 0 < args.score_threshold <= args.high_score <= 1
        or not 0 < args.mask_threshold < 1
        or not 0 < args.temporal_iou <= 1
        or not 0 < args.tile_mask_iou <= 1
        or args.temporal_max_gap <= 0
        or args.torch_threads < 1
        or args.max_requests < 0
    ):
        p.error("Invalid experimental model/evidence parameters")
    args.channel.mkdir(parents=True, mode=0o700)
    (args.channel / "requests").mkdir()
    (args.channel / "results").mkdir()
    session = uuid.uuid4().hex
    log = args.channel / "events.jsonl"
    count = 0
    stop = [False]
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    state = {
        "schema": "RGB_only_person_worker/v1",
        "state": "loading",
        "session_id": session,
        "pid": os.getpid(),
        "model_sha256": None,
        "inference_count": 0,
        "model_load_count": 0,
        "source_files": {
            str(Path(__file__)): digest(Path(__file__)),
            str(ROOT / "src/embodied_navigation/perception/person_worker.py"): digest(
                ROOT / "src/embodied_navigation/perception/person_worker.py"
            ),
            str(ROOT / "src/embodied_navigation/perception/person_tiling.py"): digest(
                ROOT / "src/embodied_navigation/perception/person_tiling.py"
            ),
        },
    }
    atomic_json(args.channel / "state.json", state)
    event_log(log, "worker_start", session_id=session, pid=os.getpid())
    try:
        actual = digest(args.weights)
        if actual != args.expected_sha256:
            raise RuntimeError("Explicit local model SHA256 mismatch")
        state["model_sha256"] = actual
        event_log(log, "model_load_started", model_sha256=actual)
        model = TorchvisionPersonModel(args)
        state.update(model.metadata, state="ready")
        atomic_json(args.channel / "state.json", state)
        event_log(
            log,
            "model_ready",
            model_sha256=actual,
            **model.metadata,
        )
        temporal = PersonTemporalEvidence(
            high_score=args.high_score,
            minimum_iou=args.temporal_iou,
            max_gap_s=args.temporal_max_gap,
        )
        last_source = -1
        while (
            not stop[0]
            and not (args.channel / "stop.request.json").exists()
            and (not args.max_requests or count < args.max_requests)
        ):
            requests = sorted((args.channel / "requests").glob("*.npz"))
            if not requests:
                time.sleep(0.005)
                continue
            request = requests[0]
            request_id = request.stem
            state.update(state="busy", request_id=request_id)
            atomic_json(args.channel / "state.json", state)
            try:
                with np.load(request, allow_pickle=False) as data:
                    if set(data.files) != {
                        "rgb",
                        "source_ns",
                        "frame_id",
                        "request_id",
                        "session_id",
                        "rgb_sha256",
                        "reason",
                        "submitted_wall_ns",
                        "submitted_monotonic_ns",
                    }:
                        raise ValueError(
                            "Unexpected request fields; only RGB and source metadata are allowed"
                        )
                    pixels = rgb_image(data["rgb"])
                    source_ns = int(data["source_ns"])
                    frame_id = str(data["frame_id"])
                    reason = str(data["reason"])
                    if (
                        str(data["session_id"]) != session
                        or str(data["request_id"]) != request_id
                        or source_ns <= last_source
                        or str(data["rgb_sha256"]) != rgb_sha256(pixels)
                    ):
                        raise ValueError(
                            "Request source/session/pixel identity mismatch"
                        )
                    submitted_monotonic = int(data["submitted_monotonic_ns"])
                started = time.monotonic_ns()
                started_wall = time.time_ns()
                count += 1
                event_log(
                    log,
                    "inference_started",
                    request_id=request_id,
                    source_ns=source_ns,
                    frame_id=frame_id,
                    reason=reason,
                    inference_index=count,
                    rgb_sha256=rgb_sha256(pixels),
                )
                masks, scores, boxes, all_scores = model.predict(
                    pixels,
                    event_sink=lambda details, rid=request_id, ns=source_ns: event_log(
                        log,
                        "model_forward_finished",
                        request_id=rid,
                        source_ns=ns,
                        **details,
                    ),
                )
                finished = time.monotonic_ns()
                bands, support = temporal.update(masks, scores, source_ns)
                metadata = {
                    "request_id": request_id,
                    "source_ns": source_ns,
                    "frame_id": frame_id,
                    "session_id": session,
                    "model_sha256": actual,
                    "rgb_sha256": rgb_sha256(pixels),
                    "image_shape": list(pixels.shape),
                    "fresh_inference": True,
                    "inference_index": count,
                    "model_load_count": 1,
                    "reason": reason,
                    "inference_started_wall_ns": started_wall,
                    "inference_finished_wall_ns": time.time_ns(),
                    "inference_seconds": (finished - started) / 1e9,
                    "queue_seconds": (started - submitted_monotonic) / 1e9,
                    "score_threshold": args.score_threshold,
                    "high_score": args.high_score,
                    "mask_threshold": args.mask_threshold,
                    "temporal_iou": args.temporal_iou,
                    "temporal_max_gap_s": args.temporal_max_gap,
                    "temporal_evidence_scope": "Adjacent fresh model-mask 2D overlap; not geometry, 3D identity or safety proof",
                    "background": "UNKNOWN",
                    "model_input": "RGB_only",
                    "oracle_or_GT_input": False,
                    **model.last_request_metadata,
                }
                atomic_npz(
                    args.channel / "results" / (request_id + ".npz"),
                    masks=masks,
                    scores=scores,
                    boxes=boxes,
                    all_person_scores=all_scores,
                    evidence_bands=bands,
                    temporal_support=support,
                    metadata_json=np.asarray(json.dumps(metadata, allow_nan=False)),
                )
                last_source = source_ns
                state.update(
                    state="ready", inference_count=count, last_source_ns=source_ns
                )
                event_log(
                    log,
                    "result_published",
                    **metadata,
                    person_scores=scores.tolist(),
                    evidence_bands=bands.tolist(),
                    temporal_support=support.tolist(),
                )
            except BaseException as error:
                failure = {
                    "error": repr(error),
                    "request_id": request_id,
                    "background": "UNKNOWN",
                    "traceback": traceback.format_exc(),
                }
                atomic_json(
                    args.channel / "results" / (request_id + ".error.json"), failure
                )
                event_log(log, "request_error", **failure)
                raise
            finally:
                request.unlink(missing_ok=True)
                (args.channel / "request.lock").unlink(missing_ok=True)
            atomic_json(args.channel / "state.json", state)
        state.update(state="stopped", inference_count=count)
        atomic_json(args.channel / "state.json", state)
        event_log(log, "worker_stopped", inference_count=count, model_load_count=1)
        return 0
    except BaseException as error:
        state.update(
            state="error",
            error=repr(error),
            traceback=traceback.format_exc(),
            inference_count=count,
        )
        atomic_json(args.channel / "state.json", state)
        event_log(log, "worker_error", error=repr(error), inference_count=count)
        raise


if __name__ == "__main__":
    raise SystemExit(main())
