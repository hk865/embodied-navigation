"""Local exact-frame person inference IPC and experimental event policy.

This client module imports no torch, SDK or ROS and loads no model; the model
owner is scripts/run_dynanav_person_worker.py. Background is never STATIC.
"""

from __future__ import annotations

import hashlib
import json
import operator
import os
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

import numpy as np


class PersonWorkerError(RuntimeError):
    """The worker is unavailable or its identity does not match this client."""


def rgb_image(rgb):
    pixels = np.asarray(rgb)
    if (
        pixels.dtype != np.uint8
        or pixels.ndim != 3
        or pixels.shape[2] != 3
        or min(pixels.shape[:2]) <= 0
    ):
        raise ValueError("Expected nonempty HWC uint8 RGB")
    return np.ascontiguousarray(pixels)


def rgb_sha256(rgb):
    return hashlib.sha256(rgb_image(rgb).tobytes()).hexdigest()


def atomic_npz(path, **arrays):
    """Publish a complete new file atomically; never overwrite evidence."""
    path = Path(path)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        with temporary.open("xb") as stream:
            np.savez_compressed(stream, **arrays)
        # Same-filesystem hard-link publication is atomic and refuses overwrite.
        os.link(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path, value):
    """Replace mutable worker state atomically; immutable results use atomic_npz."""
    path = Path(path)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        with temporary.open("x") as stream:
            json.dump(value, stream, allow_nan=False)
            stream.write("\n")
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def event_log(path, event, **values):
    row = {
        "event": event,
        "wall_ns": time.time_ns(),
        "monotonic_ns": time.monotonic_ns(),
        **values,
    }
    with Path(path).open("a") as stream:
        stream.write(json.dumps(row, allow_nan=False) + "\n")


@dataclass(frozen=True)
class PersonMaskResult:
    masks: np.ndarray
    scores: np.ndarray
    boxes: np.ndarray
    evidence_bands: np.ndarray
    temporal_support: np.ndarray
    metadata: dict

    @property
    def source_ns(self):
        return self.metadata["source_ns"]

    @property
    def frame_id(self):
        return self.metadata["frame_id"]

    @property
    def model_sha256(self):
        return self.metadata["model_sha256"]

    @property
    def request_id(self):
        return self.metadata["request_id"]

    @property
    def inference_index(self):
        return self.metadata["inference_index"]

    @property
    def latency_s(self):
        return self.metadata["client_roundtrip_seconds"]


def decode_result(path, expected):
    """Validate exact request/source/session/model/pixel identity before use."""
    with np.load(path, allow_pickle=False) as data:
        metadata = json.loads(str(data["metadata_json"]))
        for key, value in expected.items():
            if metadata.get(key) != value:
                raise PersonWorkerError("Result identity mismatch: " + key)
        if (
            metadata.get("fresh_inference") is not True
            or metadata.get("inference_index", 0) < 1
        ):
            raise PersonWorkerError(
                "Result does not describe an executed model forward"
            )
        arrays = {
            k: data[k].copy()
            for k in ["masks", "scores", "boxes", "evidence_bands", "temporal_support"]
        }
    masks = arrays["masks"]
    n = len(masks)
    if (
        masks.dtype != bool
        or masks.shape != (n, *metadata["image_shape"][:2])
        or arrays["scores"].shape != (n,)
        or arrays["boxes"].shape != (n, 4)
        or arrays["evidence_bands"].shape != (n,)
        or arrays["temporal_support"].shape != (n,)
    ):
        raise PersonWorkerError("Result mask/score/band shapes are inconsistent")
    if (
        not np.isfinite(arrays["scores"]).all()
        or not np.isfinite(arrays["boxes"]).all()
        or not np.isin(
            arrays["evidence_bands"],
            ["high_confidence", "low_temporal_supported", "low_unconfirmed"],
        ).all()
    ):
        raise PersonWorkerError("Invalid model evidence payload")
    return PersonMaskResult(**arrays, metadata=metadata)


class PersonWorkerClient:
    """One capture client per worker; operational failure returns None/UNKNOWN."""

    def __init__(self, channel, *, expected_model_sha256, poll_s=0.005):
        self.channel = Path(channel)
        self.expected_model_sha256 = expected_model_sha256
        self.poll_s = poll_s
        self.session_id = None
        self.last_error = None
        self.last_request_ns = -1

    def status(self):
        try:
            return json.loads((self.channel / "state.json").read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            return {"state": "not_started"}

    def wait_ready(self, timeout_s=30):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            state = self.status()
            if state["state"] == "ready":
                if state.get("model_sha256") != self.expected_model_sha256:
                    raise PersonWorkerError("Worker model SHA256 mismatch")
                self.session_id = state["session_id"]
                return state
            if state["state"] in ("error", "stopped"):
                raise PersonWorkerError(
                    "Worker " + state["state"] + ": " + str(state.get("error"))
                )
            time.sleep(self.poll_s)
        raise TimeoutError("Person worker startup timeout")

    def predict(
        self, rgb, *, source_ns, frame_id="go2_camera_optical", reason, timeout_s=3
    ):
        pixels = rgb_image(rgb)
        source_ns = operator.index(source_ns)
        if (
            source_ns < 0
            or source_ns <= self.last_request_ns
            or not isinstance(frame_id, str)
            or not frame_id
        ):
            raise ValueError(
                "New nonnegative source stamp and nonempty frame ID required"
            )
        started = time.monotonic()
        deadline = started + timeout_s
        request_id = uuid.uuid4().hex
        try:
            while True:
                state = self.status()
                if state["state"] in ("error", "stopped"):
                    raise PersonWorkerError(
                        "Worker " + state["state"] + ": " + str(state.get("error"))
                    )
                if (
                    state["state"] == "ready"
                    and not (self.channel / "request.lock").exists()
                ):
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError("Worker not ready before request deadline")
                time.sleep(self.poll_s)
            if state.get("model_sha256") != self.expected_model_sha256:
                raise PersonWorkerError("Worker model identity changed")
            if self.session_id is None:
                self.session_id = state["session_id"]
            if state["session_id"] != self.session_id:
                raise PersonWorkerError("Worker session changed")
            expected = {
                "source_ns": source_ns,
                "frame_id": frame_id,
                "model_sha256": self.expected_model_sha256,
                "session_id": self.session_id,
                "request_id": request_id,
                "rgb_sha256": rgb_sha256(pixels),
                "image_shape": list(pixels.shape),
            }
            lock = self.channel / "request.lock"
            with lock.open("x") as stream:
                stream.write(request_id)
            try:
                atomic_npz(
                    self.channel / "requests" / (request_id + ".npz"),
                    rgb=pixels,
                    source_ns=np.int64(source_ns),
                    frame_id=np.asarray(frame_id),
                    request_id=np.asarray(request_id),
                    session_id=np.asarray(self.session_id),
                    rgb_sha256=np.asarray(expected["rgb_sha256"]),
                    reason=np.asarray(reason),
                    submitted_wall_ns=np.int64(time.time_ns()),
                    submitted_monotonic_ns=np.int64(time.monotonic_ns()),
                )
            except BaseException:
                lock.unlink(missing_ok=True)
                raise
            self.last_request_ns = source_ns
            event_log(
                self.channel / "client-events.jsonl",
                "request_submitted",
                **expected,
                reason=reason,
            )
            result_path = self.channel / "results" / (request_id + ".npz")
            error_path = self.channel / "results" / (request_id + ".error.json")
            while time.monotonic() < deadline:
                if result_path.exists():
                    result = decode_result(result_path, expected)
                    result.metadata["client_roundtrip_seconds"] = (
                        time.monotonic() - started
                    )
                    self.last_error = None
                    event_log(
                        self.channel / "client-events.jsonl",
                        "result_received",
                        **expected,
                        inference_index=result.inference_index,
                        latency_s=result.latency_s,
                    )
                    return result
                if error_path.exists():
                    raise PersonWorkerError(error_path.read_text())
                current = self.status()
                if current["state"] in ("error", "stopped"):
                    raise PersonWorkerError("Worker " + current["state"])
                time.sleep(self.poll_s)
            raise TimeoutError("Exact-frame person inference timed out")
        except (OSError, ValueError, PersonWorkerError, TimeoutError) as error:
            self.last_error = {
                "error": repr(error),
                "source_ns": source_ns,
                "frame_id": frame_id,
                "request_id": request_id,
                "background": "UNKNOWN",
            }
            if self.channel.exists():
                event_log(
                    self.channel / "client-events.jsonl",
                    "request_failed",
                    **self.last_error,
                )
            return None

    def stop(self):
        if self.channel.exists():
            atomic_json(
                self.channel / "stop.request.json",
                {"session_id": self.session_id, "wall_ns": time.time_ns()},
            )


class PersonTriggerPolicy:
    """Experimental source-time events; motion evidence must be non-GT."""

    def __init__(
        self,
        *,
        max_interval_s=2.0,
        min_interval_s=0.2,
        view_change_threshold=0.06,
        translation_threshold_m=0.15,
        rotation_threshold_rad=0.15,
    ):
        if (
            not 0 < min_interval_s <= max_interval_s
            or not 0 < view_change_threshold <= 1
            or min(translation_threshold_m, rotation_threshold_rad) <= 0
        ):
            raise ValueError("Positive experimental trigger parameters required")
        self.max_ns = round(max_interval_s * 1e9)
        self.min_ns = round(min_interval_s * 1e9)
        self.view_threshold = view_change_threshold
        self.translation_threshold = translation_threshold_m
        self.rotation_threshold = rotation_threshold_rad
        self.last_source_ns = None
        self.signature = None
        self.needs_low_confirmation = False

    @staticmethod
    def _signature(rgb):
        pixels = rgb_image(rgb)
        return (
            pixels[
                :: max(1, pixels.shape[0] // 32), :: max(1, pixels.shape[1] // 32)
            ].astype(np.float32)
            / 255.0
        )

    def decide(
        self,
        rgb,
        *,
        source_ns,
        estimated_motion_m=None,
        measured_rotation_rad=None,
        motion_source=None,
    ):
        source_ns = operator.index(source_ns)
        signature = self._signature(rgb)
        if source_ns < 0 or (
            self.last_source_ns is not None and source_ns <= self.last_source_ns
        ):
            raise ValueError("Source clock must increase")
        motion = [
            v for v in [estimated_motion_m, measured_rotation_rad] if v is not None
        ]
        if motion and (
            motion_source
            not in ("lio_estimate", "livo_estimate", "livo_post_vio", "raw_imu")
            or not np.isfinite(motion).all()
        ):
            raise ValueError(
                "Motion trigger requires finite non-GT estimator/IMU evidence"
            )
        change = (
            None
            if self.signature is None
            else (
                1.0
                if signature.shape != self.signature.shape
                else float(np.mean(abs(signature - self.signature)))
            )
        )
        age = None if self.last_source_ns is None else source_ns - self.last_source_ns
        moved = (
            estimated_motion_m is not None
            and abs(estimated_motion_m) >= self.translation_threshold
        ) or (
            measured_rotation_rad is not None
            and abs(measured_rotation_rad) >= self.rotation_threshold
        )
        if age is None:
            reason = "initial"
        elif age >= self.max_ns:
            reason = "max_interval"
        elif age >= self.min_ns and self.needs_low_confirmation:
            reason = "low_confidence_confirmation"
        elif age >= self.min_ns and (moved or change >= self.view_threshold):
            reason = "motion_new_view"
        else:
            reason = "not_due"
        return {
            "trigger": reason != "not_due",
            "reason": reason,
            "source_ns": source_ns,
            "since_last_inference_ns": age,
            "view_change": change,
            "estimated_motion_m": estimated_motion_m,
            "measured_rotation_rad": measured_rotation_rad,
            "motion_source": motion_source,
            "parameters_experimental": True,
        }

    def mark_inferred(self, rgb, *, source_ns, result=None):
        if result is not None and (
            result.source_ns != source_ns
            or result.metadata.get("rgb_sha256") != rgb_sha256(rgb)
        ):
            raise ValueError(
                "Only an exact-source successful inference may advance the trigger"
            )
        if self.last_source_ns is not None and source_ns <= self.last_source_ns:
            raise ValueError("Inference source clock must increase")
        self.signature = self._signature(rgb)
        self.last_source_ns = int(source_ns)
        self.needs_low_confirmation = result is not None and bool(
            np.any(result.evidence_bands == "low_unconfirmed")
        )


class PersonTemporalEvidence:
    """Adjacent fresh RGB-mask overlap support, never GT or 3D identity."""

    def __init__(self, *, high_score=0.5, minimum_iou=0.25, max_gap_s=0.75):
        self.high_score = high_score
        self.minimum_iou = minimum_iou
        self.max_gap_ns = round(max_gap_s * 1e9)
        self.previous = []
        self.last_source_ns = None

    def update(self, masks, scores, source_ns):
        masks = np.asarray(masks)
        scores = np.asarray(scores)
        if masks.dtype != bool or masks.ndim != 3 or scores.shape != (len(masks),):
            raise ValueError("Invalid temporal masks/scores")
        if self.last_source_ns is not None and source_ns <= self.last_source_ns:
            raise ValueError(
                "Temporal evidence needs distinct increasing source frames"
            )
        previous = (
            self.previous
            if self.last_source_ns is not None
            and source_ns - self.last_source_ns <= self.max_gap_ns
            else []
        )
        support = np.ones(len(masks), dtype=np.int64)
        used = set()
        for j in np.argsort(-scores):
            options = []
            for k, (old, count) in enumerate(previous):
                if k in used or old.shape != masks[j].shape:
                    continue
                union = np.count_nonzero(old | masks[j])
                overlap = np.count_nonzero(old & masks[j]) / union if union else 0.0
                if overlap >= self.minimum_iou:
                    options.append((overlap, k, count))
            if options:
                _, k, count = max(options)
                support[j] = count + 1
                used.add(k)
        bands = np.array(
            [
                "high_confidence"
                if score >= self.high_score
                else "low_temporal_supported"
                if count >= 2
                else "low_unconfirmed"
                for score, count in zip(scores, support)
            ],
            dtype="U32",
        )
        self.previous = [
            (mask.copy(), int(count)) for mask, count in zip(masks, support)
        ]
        self.last_source_ns = source_ns
        return bands, support
