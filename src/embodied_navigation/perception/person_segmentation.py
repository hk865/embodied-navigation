"""Pure pixel correspondence and scoring for offline RGB-only person masks.

No torch, SDK, ROS, model download, pose input or static-background inference.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class EvaluationFrame:
    path: Path
    source_ns: int
    rgb: np.ndarray
    pixel_flat_indices: np.ndarray
    states: np.ndarray


def load_evaluation_frame(path, person_root="/World/DynamicPedestrian"):
    """Explicit allowlist; archived poses are never accessed, even for scoring."""
    path = Path(path)
    with np.load(path, allow_pickle=False) as data:
        stamp = data["source_ns"]
        if stamp.shape != () or not np.issubdtype(stamp.dtype, np.integer):
            raise ValueError("source_ns must be an integer scalar")
        source_ns = int(stamp)
        rgb = data["rgb"].copy()
        indices = data["pixel_flat_indices"].copy()
        states = data["states"].copy()
        if source_ns < 0 or rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("Expected nonnegative timestamp and HWC uint8 RGB")
        if indices.ndim != 1 or not np.issubdtype(indices.dtype, np.integer):
            raise ValueError(
                "pixel_flat_indices must be a one-dimensional integer array"
            )
        if (
            np.any(indices < 0)
            or np.any(indices >= rgb.shape[0] * rgb.shape[1])
            or len(np.unique(indices)) != len(indices)
        ):
            raise ValueError(
                "Sample pixels must be unique and in the original RGB bounds"
            )
        if (
            states.shape != indices.shape
            or not np.issubdtype(states.dtype, np.integer)
            or not np.isin(states, [0, 1, 2]).all()
        ):
            raise ValueError("Expected per-sample UNKNOWN/STATIC/DYNAMIC oracle states")
        # This dataset's DYNAMIC truth is the configured pedestrian. Refuse to
        # silently score other moving objects as person labels in another case.
        if np.any(states == 2):
            paths = data["instance_paths"]
            if paths.shape != states.shape or paths.dtype.kind != "U":
                raise ValueError(
                    "Dynamic person truth requires per-sample instance paths"
                )
            if any(
                not (p == person_root or p.startswith(person_root + "/"))
                for p in paths[states == 2]
            ):
                raise ValueError(
                    "Oracle DYNAMIC includes a non-person path; select a person-specific truth protocol"
                )
    return EvaluationFrame(path, source_ns, rgb, indices, states)


def periodic_indices(source_ns, interval_ns=2_000_000_000):
    """First frame at/after each anchored real-time deadline, without propagation."""
    stamps = [int(t) for t in source_ns]
    if (
        interval_ns <= 0
        or any(t < 0 for t in stamps)
        or any(b <= a for a, b in itertools.pairwise(stamps))
    ):
        raise ValueError(
            "Positive interval and strictly increasing source timestamps required"
        )
    selected, due = [], stamps[0] if stamps else None
    for index, stamp in enumerate(stamps):
        if stamp >= due:
            selected.append(index)
            due += ((stamp - due) // interval_ns + 1) * interval_ns
    return selected


def project_masks(masks, pixel_flat_indices):
    """One original RGB pixel per cloud sample; preserve original sample order."""
    masks, indices = np.asarray(masks), np.asarray(pixel_flat_indices)
    if (
        masks.dtype != bool
        or masks.ndim != 3
        or indices.ndim != 1
        or not np.issubdtype(indices.dtype, np.integer)
    ):
        raise ValueError("Expected boolean NxHxW masks and integer sample indices")
    if np.any(indices < 0) or np.any(indices >= masks.shape[1] * masks.shape[2]):
        raise ValueError("Sample index outside mask image")
    return masks.reshape(len(masks), masks.shape[1] * masks.shape[2])[:, indices]


def confusion(predicted_person, states):
    """Score known sampled oracle labels only; never turn background into STATIC."""
    predicted, truth = np.asarray(predicted_person), np.asarray(states)
    if (
        predicted.dtype != bool
        or predicted.shape != truth.shape
        or truth.ndim != 1
        or not np.isin(truth, [0, 1, 2]).all()
    ):
        raise ValueError("Expected matching per-sample masks and oracle states")
    known, positive = truth != 0, truth == 2
    return {
        "tp": int(np.sum(predicted & positive & known)),
        "fp": int(np.sum(predicted & ~positive & known)),
        "fn": int(np.sum(~predicted & positive & known)),
        "tn": int(np.sum(~predicted & ~positive & known)),
        "ignored_unknown": int(np.sum(~known)),
    }


def metric_rates(counts):
    tp, fp, fn = counts["tp"], counts["fp"], counts["fn"]
    return {
        **counts,
        "precision": tp / (tp + fp) if tp + fp else None,
        "recall": tp / (tp + fn) if tp + fn else None,
        "iou": tp / (tp + fp + fn) if tp + fp + fn else None,
    }


def mask_iou(first, second):
    union = np.count_nonzero(first | second)
    return np.count_nonzero(first & second) / union if union else 0.0


class MaskIoUTracker:
    """Greedy adjacent-selected-frame 2D mask association, not 3D re-identification."""

    def __init__(self, minimum_iou=0.25):
        if not 0 < minimum_iou <= 1:
            raise ValueError("IoU association threshold must be within (0,1]")
        self.minimum_iou = minimum_iou
        self.previous = []
        self.next_id = 1
        self.last_source_ns = -1

    def update(self, masks, source_ns):
        if source_ns <= self.last_source_ns:
            raise ValueError("Association timestamps must increase")
        masks = np.asarray(masks)
        if masks.dtype != bool or masks.ndim != 3:
            raise ValueError("Expected boolean instance masks")
        proposals = sorted(
            [
                (mask_iou(mask, old), j, ident)
                for j, mask in enumerate(masks)
                for ident, old in self.previous
            ],
            reverse=True,
        )
        ids, used = [None] * len(masks), set()
        for overlap, j, ident in proposals:
            if overlap >= self.minimum_iou and ids[j] is None and ident not in used:
                ids[j] = ident
                used.add(ident)
        for j in range(len(ids)):
            if ids[j] is None:
                ids[j] = self.next_id
                self.next_id += 1
        self.previous = [(ident, mask.copy()) for ident, mask in zip(ids, masks)]
        self.last_source_ns = source_ns
        return ids
