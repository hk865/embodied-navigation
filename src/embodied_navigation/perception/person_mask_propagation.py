"""Bounded RGB optical-flow quarantine, never a new semantic model inference.

Only the last fresh positive learned candidate establishes the horizon. Empty
fresh detections do not erase it, and adjacent propagation cannot extend it.
Uncertain propagation asks the caller to withhold temporary frontend geometry;
no returned pixel or missing detection certifies STATIC/background/free space.
"""

from __future__ import annotations

import operator
import time
from dataclasses import dataclass

import numpy as np

from embodied_navigation.perception.person_worker import rgb_image


def _gray(rgb):
    return np.asarray(rgb @ np.array([0.299, 0.587, 0.114]), dtype=np.float32)


def _texture(gray, width=7):
    half = width // 2

    def average(a):
        padded = np.pad(a, half, mode="edge")
        integral = np.pad(padded, ((1, 0), (1, 0))).cumsum(0).cumsum(1)
        return (
            integral[width:, width:]
            - integral[:-width, width:]
            - integral[width:, :-width]
            + integral[:-width, :-width]
        ) / width**2

    return np.sqrt(
        np.maximum(
            0, average(gray.astype(float) ** 2) - average(gray.astype(float)) ** 2
        )
    )


def _sample(array, x, y):
    """Bilinear sample; validity is separately checked before use."""
    height, width = array.shape[:2]
    x, y = np.clip(x, 0, width - 1), np.clip(y, 0, height - 1)
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    x1, y1 = np.minimum(x0 + 1, width - 1), np.minimum(y0 + 1, height - 1)
    dx, dy = x - x0, y - y0
    if array.ndim == 3:
        dx, dy = dx[..., None], dy[..., None]
    return (
        (1 - dx) * (1 - dy) * array[y0, x0]
        + dx * (1 - dy) * array[y0, x1]
        + (1 - dx) * dy * array[y1, x0]
        + dx * dy * array[y1, x1]
    )


def farneback_bidirectional(previous_gray, current_gray):
    """Lazy local OpenCV only; no model, GPU, SDK, ROS or network initialization."""
    import cv2

    def flow(first, second):
        return cv2.calcOpticalFlowFarneback(
            first, second, None, 0.5, 3, 21, 3, 7, 1.5, 0
        )

    return flow(previous_gray, current_gray), flow(current_gray, previous_gray)


@dataclass(frozen=True)
class PropagatedPersonCandidate:
    quarantine_mask: np.ndarray
    quality_mask: np.ndarray
    withhold: bool
    receipt: dict


class PersonMaskPropagator:
    """Adjacent RGB flow with explicit quality gates and an immutable anchor.

    A successful propagation quarantines the whole warped candidate footprint:
    locally uncertain pixels in that footprint remain UNKNOWN/quarantined. If
    too little of it passes quality checks, ``withhold`` is true. The caller
    must then withhold geometry rather than silently using the empty/partial
    mask. An opt-in caller owns startup/expiry policy; an expired/no-candidate
    result is not evidence that the image contains no person.
    """

    def __init__(
        self,
        *,
        max_horizon_ns=500_000_000,
        max_step_ns=150_020_000,
        score_threshold=0.1,
        max_fb_error_px=1.5,
        max_photometric_error=30.0,
        min_texture_std=3.0,
        minimum_quality_fraction=0.5,
        minimum_mask_pixels=8,
        flow_function=None,
    ):
        for name, value in [
            ("max_horizon_ns", max_horizon_ns),
            ("max_step_ns", max_step_ns),
            ("minimum_mask_pixels", minimum_mask_pixels),
        ]:
            if isinstance(value, bool) or operator.index(value) < 1:
                raise ValueError(name + " must be a positive integer")
        if (
            not 0 < score_threshold <= 1
            or not 0 < minimum_quality_fraction <= 1
            or not np.isfinite(
                [max_fb_error_px, max_photometric_error, min_texture_std]
            ).all()
            or min(max_fb_error_px, max_photometric_error) <= 0
            or min_texture_std < 0
        ):
            raise ValueError("Invalid experimental optical-flow gates")
        self.max_horizon_ns, self.max_step_ns = int(max_horizon_ns), int(max_step_ns)
        self.score_threshold = score_threshold
        self.max_fb_error_px = max_fb_error_px
        self.max_photometric_error = max_photometric_error
        self.min_texture_std = min_texture_std
        self.minimum_quality_fraction = minimum_quality_fraction
        self.minimum_mask_pixels = minimum_mask_pixels
        self.flow_function = flow_function or farneback_bidirectional
        self.previous_rgb = self.previous_ns = self.anchor_ns = self.candidate = None
        self.failed = False

    def update(
        self,
        rgb,
        source_ns,
        *,
        fresh_masks=None,
        fresh_scores=None,
        fresh_mask_source_ns=None,
    ):
        started = time.monotonic_ns()
        pixels = rgb_image(rgb)
        source_ns = operator.index(source_ns)
        if source_ns < 0 or (
            self.previous_ns is not None and source_ns <= self.previous_ns
        ):
            raise ValueError("Adjacent RGB source stamps must strictly increase")
        height, width = pixels.shape[:2]
        seed = None
        fresh_empty = False
        if fresh_masks is not None:
            masks, scores = np.asarray(fresh_masks), np.asarray(fresh_scores)
            if (
                fresh_mask_source_ns != source_ns
                or masks.dtype != bool
                or masks.shape != (len(masks), height, width)
                or scores.shape != (len(masks),)
                or not np.isfinite(scores).all()
                or np.any((scores < 0) | (scores > 1))
            ):
                raise ValueError(
                    "Fresh learned candidates must exactly match this RGB frame"
                )
            selected = masks[scores >= self.score_threshold]
            seed = (
                np.any(selected, axis=0)
                if len(selected)
                else np.zeros((height, width), bool)
            )
            fresh_empty = not np.any(seed)
        elif fresh_scores is not None or fresh_mask_source_ns is not None:
            raise ValueError("Fresh scores/stamp require fresh masks")
        old_source = self.previous_ns
        anchor_age = None if self.anchor_ns is None else source_ns - self.anchor_ns
        receipt = {
            "schema": "bounded_RGB_flow_person_candidate_quarantine/v1",
            "source_ns": source_ns,
            "previous_rgb_source_ns": old_source,
            "anchor_fresh_positive_source_ns": self.anchor_ns,
            "anchor_age_ns": anchor_age,
            "fresh_empty_did_not_clear_anchor": fresh_empty
            and self.anchor_ns is not None,
            "model_inference_performed": False,
            "new_semantic_observation": False,
            "source_kind": "geometric_rgb_flow",
            "output_state": "UNKNOWN_quarantine",
            "background_certified_static": False,
            "three_dimensional_ID": False,
            "horizon_refreshes_only_from_fresh_positive_candidate": True,
            "parameters_experimental": True,
            "parameters": {
                "max_horizon_ns": self.max_horizon_ns,
                "max_step_ns": self.max_step_ns,
                "score_threshold": self.score_threshold,
                "max_fb_error_px": self.max_fb_error_px,
                "max_photometric_error": self.max_photometric_error,
                "min_texture_std": self.min_texture_std,
                "minimum_quality_fraction": self.minimum_quality_fraction,
                "minimum_mask_pixels": self.minimum_mask_pixels,
            },
        }
        output = quality = np.zeros((height, width), bool)
        withhold = False
        if seed is not None and np.any(seed):
            self.anchor_ns = source_ns
            self.candidate = seed.copy()
            self.failed = False
            output = quality = seed.copy()
            status = "fresh_positive_seed"
            receipt.update(
                anchor_fresh_positive_source_ns=source_ns,
                anchor_age_ns=0,
                source_kind="fresh_learned_candidate",
                fresh_empty_did_not_clear_anchor=False,
            )
        elif self.anchor_ns is None:
            status = "no_active_positive_candidate"
        elif anchor_age > self.max_horizon_ns:
            status = "positive_candidate_horizon_expired"
            self.anchor_ns = self.candidate = None
            self.failed = False
        elif self.failed:
            status, withhold = "prior_flow_failure_within_positive_horizon", True
        elif (
            old_source is None
            or source_ns - old_source > self.max_step_ns
            or self.previous_rgb.shape != pixels.shape
        ):
            status, withhold, self.failed = (
                "nonadjacent_or_changed_RGB_within_positive_horizon",
                True,
                True,
            )
        else:
            try:
                oldgray, gray = _gray(self.previous_rgb), _gray(pixels)
                forward, backward = (
                    np.asarray(a, dtype=float)
                    for a in self.flow_function(oldgray, gray)
                )
                if (
                    forward.shape != (height, width, 2)
                    or backward.shape != forward.shape
                    or not np.isfinite(forward).all()
                    or not np.isfinite(backward).all()
                ):
                    raise ValueError("Flow fields must be finite HxWx2")
                yy, xx = np.indices((height, width))
                px, py = xx + backward[..., 0], yy + backward[..., 1]
                in_bounds = (
                    (px >= 0) & (px <= width - 1) & (py >= 0) & (py <= height - 1)
                )
                output = (
                    _sample(self.candidate.astype(float), px, py) >= 0.5
                ) & in_bounds
                fb_error = np.linalg.norm(backward + _sample(forward, px, py), axis=2)
                photometric = np.abs(gray - _sample(oldgray, px, py))
                texture = np.minimum(_texture(gray), _sample(_texture(oldgray), px, py))
                quality = (
                    output
                    & (fb_error <= self.max_fb_error_px)
                    & (photometric <= self.max_photometric_error)
                    & (texture >= self.min_texture_std)
                )
                count = int(output.sum())
                fraction = float(quality.sum() / count) if count else 0.0
                receipt.update(
                    warped_candidate_pixels=count,
                    quality_pass_pixels=int(quality.sum()),
                    quality_fraction=fraction,
                    uncertain_warped_pixels=count - int(quality.sum()),
                    fb_error_median_px=float(np.median(fb_error[output]))
                    if count
                    else None,
                    photometric_error_median=float(np.median(photometric[output]))
                    if count
                    else None,
                )
                if (
                    count < self.minimum_mask_pixels
                    or fraction < self.minimum_quality_fraction
                ):
                    status, withhold, self.failed = (
                        "insufficient_flow_quality_within_positive_horizon",
                        True,
                        True,
                    )
                else:
                    status = "quality_gated_adjacent_propagation"
                    self.candidate = output.copy()
            except (ValueError, RuntimeError, ImportError) as error:
                status, withhold, self.failed = (
                    "flow_error_within_positive_horizon",
                    True,
                    True,
                )
                receipt["error"] = repr(error)
        self.previous_rgb, self.previous_ns = pixels.copy(), source_ns
        receipt.update(
            status=status,
            withhold=withhold,
            quarantine_pixels=int(output.sum()),
            elapsed_seconds=(time.monotonic_ns() - started) / 1e9,
            chained_horizon_not_refreshed=True,
        )
        return PropagatedPersonCandidate(
            output.copy(), quality.copy(), withhold, receipt
        )
