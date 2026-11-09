"""Pure crop lifting and mask-IoU deduplication for optional person inference."""

from __future__ import annotations

import numpy as np


def inference_views(image_shape, grid=1):
    height, width = image_shape[:2]
    if grid not in (1, 2) or min(height, width) < 2:
        raise ValueError("Grid1/2 and nontrivial image dimensions required")
    views = [("full", (0, 0, width, height))]
    if grid == 2:
        ys = (0, height // 2, height)
        xs = (0, width // 2, width)
        views += [
            (f"tile_{row}_{column}", (xs[column], ys[row], xs[column + 1], ys[row + 1]))
            for row in range(2)
            for column in range(2)
        ]
    return views


def lift_masks(masks, boxes, crop, image_shape):
    masks, boxes = np.asarray(masks), np.asarray(boxes)
    x0, y0, x1, y1 = crop
    height, width = image_shape[:2]
    if (
        masks.dtype != bool
        or masks.shape != (len(masks), y1 - y0, x1 - x0)
        or boxes.shape != (len(masks), 4)
    ):
        raise ValueError("Crop-local mask/box dimensions do not match")
    if not 0 <= x0 < x1 <= width or not 0 <= y0 < y1 <= height:
        raise ValueError("Crop outside source image")
    lifted = np.zeros((len(masks), height, width), bool)
    lifted[:, y0:y1, x0:x1] = masks
    translated = boxes.copy()
    translated[:, [0, 2]] += x0
    translated[:, [1, 3]] += y0
    return lifted, translated


def merge_mask_candidates(parts, minimum_iou=0.5):
    """Keep highest-scoring overlapping instance, preserving crop provenance."""
    if not parts or not 0 < minimum_iou <= 1:
        raise ValueError("Predictions and a valid IoU threshold required")
    masks = np.concatenate([p["masks"] for p in parts])
    scores = np.concatenate([p["scores"] for p in parts])
    boxes = np.concatenate([p["boxes"] for p in parts])
    origins = [
        {"view": p["view"], "crop_xyxy": list(p["crop"]), "local_instance_index": j}
        for p in parts
        for j in range(len(p["scores"]))
    ]
    kept = []
    suppressed = []
    for index in sorted(range(len(scores)), key=lambda i: (-float(scores[i]), i)):
        duplicate = None
        for old in kept:
            union = np.count_nonzero(masks[index] | masks[old])
            iou = np.count_nonzero(masks[index] & masks[old]) / union if union else 0.0
            if iou >= minimum_iou:
                duplicate = (old, float(iou))
                break
        if duplicate is None:
            kept.append(index)
        else:
            suppressed.append(
                {
                    "candidate_index": index,
                    "kept_candidate_index": duplicate[0],
                    "mask_iou": duplicate[1],
                }
            )
    return (
        masks[kept],
        scores[kept],
        boxes[kept],
        {
            "raw_candidate_count": len(scores),
            "retained_count": len(kept),
            "mask_iou_threshold": minimum_iou,
            "retained_origins": [origins[i] for i in kept],
            "suppressed": suppressed,
        },
    )
