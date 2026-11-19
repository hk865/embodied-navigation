"""Offline comparison of stair inflation and explicit body geometry.

This module never supplies a runtime permit. Semantic labels and the body pose
are caller inputs: no occupied voxel is reclassified just because it blocked a
trajectory. Unknown or non-stair cells retain SCAN's original inflation.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from embodied_navigation.planning.admission.validate_route_support import aabb_obb_disjoint


@dataclass(frozen=True)
class BodyBox:
    """One conservative body box, expressed in the same map frame as voxels.

    ``rotation`` columns are the local body axes. For a measured IMU pose,
    callers must apply the IMU-to-body extrinsic before constructing this box.
    This is not a foot-placement or dynamic-stability model.
    """

    center: np.ndarray
    half_extents: np.ndarray
    rotation: np.ndarray


def scan_inflation_hits(
    voxel_indices: np.ndarray,
    query_indices: np.ndarray,
    resolution: float,
    radius: float,
    inflation_up: float,
    inflation_down: float,
) -> np.ndarray:
    """Reproduce the current integer-grid inflation test, per occupied voxel.

    SCAN excludes XY offsets at exactly ``radius`` and rounds Z upward to a
    whole number of cells. ``query_indices`` are front/rear query cells.
    """
    offsets = query_indices[:, None, :] - voxel_indices[None, :, :]
    xy_inside = np.linalg.norm(offsets[:, :, :2] * resolution, axis=2) < radius
    z_inside = (offsets[:, :, 2] <= np.ceil(inflation_up / resolution)) & (
        offsets[:, :, 2] >= -np.ceil(inflation_down / resolution)
    )
    return np.any(xy_inside & z_inside, axis=0)


def body_voxel_hits(
    voxel_indices: np.ndarray, resolution: float, boxes: tuple[BodyBox, ...]
) -> np.ndarray:
    """Intersect finite occupied voxel cubes with the explicit oriented body.

    Voxel volume is retained; this is not a voxel-centre point containment test.
    Boxes may include conservative bounds for non-box physical collision shapes.
    """
    result = np.zeros(len(voxel_indices), dtype=bool)
    for i, index in enumerate(voxel_indices):
        lo = index * resolution
        hi = lo + resolution
        for box in boxes:
            if not aabb_obb_disjoint(
                lo, hi,
                {"c": box.center, "h": box.half_extents, "r": box.rotation},
                0.0,
            ):
                result[i] = True
                break
    return result


def compare_stair_collision(
    *,
    voxel_indices: np.ndarray,
    stair_mask: np.ndarray,
    query_indices: np.ndarray,
    boxes: tuple[BodyBox, ...],
    resolution: float,
    radius: float,
    inflation_up: float,
    inflation_down: float,
) -> dict:
    """Compare observed collisions only, preserving all source geometry.

    Caller must supply occupied voxel indices and an explicit boolean mask.
    This function does not infer stair identity from a collision, does not drop
    raw geometry, and does not certify unobserved space or the complete motion.
    """
    voxels = np.asarray(voxel_indices)
    queries = np.asarray(query_indices)
    stairs = np.asarray(stair_mask)
    # These are external data boundaries, not hidden internal fallbacks.
    if voxels.ndim != 2 or voxels.shape[1] != 3 or not np.issubdtype(voxels.dtype, np.integer):
        raise ValueError("voxel_indices must be an integer N x 3 array")
    if queries.ndim != 2 or queries.shape[1] != 3 or not np.issubdtype(queries.dtype, np.integer):
        raise ValueError("query_indices must be an integer M x 3 array")
    if stairs.dtype != bool or stairs.shape != (len(voxels),):
        raise ValueError("stair_mask must explicitly label every supplied occupied voxel")
    if (not np.all(np.isfinite([resolution, radius, inflation_up, inflation_down]))
            or resolution <= 0 or radius <= 0 or inflation_up < 0 or inflation_down < 0):
        raise ValueError("invalid grid or inflation dimensions")
    if not boxes:
        raise ValueError("explicit body geometry is required when removing stair inflation")
    _validate_boxes(boxes)
    inflated = scan_inflation_hits(voxels, queries, resolution, radius, inflation_up, inflation_down)
    body = body_voxel_hits(voxels, resolution, boxes)
    remaining = (inflated & ~stairs) | (body & stairs)
    return {
        "schema": "stair_collision_shadow/v1",
        "runtime_permit": False,
        "baseline_observed_collision": bool(inflated.any()),
        "candidate_observed_collision": bool(remaining.any()),
        "baseline_hit_indices": voxels[inflated].tolist(),
        "retained_non_stair_inflation_hits": voxels[inflated & ~stairs].tolist(),
        "raw_stair_body_hits": voxels[body & stairs].tolist(),
        "stair_voxels_without_inflation": int(stairs.sum()),
        "scope": "supplied occupied voxels only; no unknown-space, swept-motion, foot or stability certification",
    }


def _validate_boxes(boxes: tuple[BodyBox, ...]) -> None:
    for box in boxes:
        if (np.shape(box.center) != (3,) or np.shape(box.half_extents) != (3,)
                or np.shape(box.rotation) != (3, 3)
                or not np.all(np.isfinite(box.center))
                or not np.all(np.isfinite(box.half_extents))
                or np.any(box.half_extents <= 0)
                or not np.allclose(box.rotation.T @ box.rotation, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(box.rotation), 1.0, atol=1e-6)):
            raise ValueError("body box must have positive dimensions and a finite proper rotation")


def swept_body_collision(
    *,
    voxel_indices: np.ndarray,
    resolution: float,
    body_boxes: tuple[BodyBox, ...],
    reference_positions: np.ndarray,
    reference_rotations: np.ndarray,
    subdivisions: int = 8,
) -> dict:
    """Conservative continuous body check against ALL supplied occupied voxels.

    Supply the complete occupied map, not inflation-query contributors, and
    body boxes in one fixed body-reference frame (including shape offsets).
    Positions interpolate linearly; rotations use shortest-arc SO(3) Slerp.
    This certifies only that assumed interpolation, not arbitrary actual motion.

    Each interval midpoint OBB is enlarged in every local axis by
    ``translation_length/2 + 2*r*sin(rotation_angle/4)``. Here r bounds the
    distance of every shape point from the reference origin, including offsets.
    Thus every intermediate shape point lies inside the enlarged OBB. There is
    no unaccounted sampling gap, but overlap can be a conservative false positive.
    Non-stair SCAN inflation remains a separate required check; this checks raw
    geometry for every label and never supplies permission to execute.
    """
    voxels = np.asarray(voxel_indices)
    positions = np.asarray(reference_positions, dtype=float)
    rotations = np.asarray(reference_rotations, dtype=float)
    if voxels.ndim != 2 or voxels.shape[1] != 3 or not np.issubdtype(voxels.dtype, np.integer):
        raise ValueError("complete occupied map must be an integer N x 3 array")
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError("resolution must be finite and positive")
    if positions.ndim != 2 or positions.shape[1] != 3 or len(positions) < 2 or not np.isfinite(positions).all():
        raise ValueError("at least two finite reference positions are required")
    if rotations.shape != (len(positions), 3, 3) or not np.isfinite(rotations).all():
        raise ValueError("one finite reference rotation per position is required")
    if (not np.allclose(rotations.transpose(0, 2, 1) @ rotations, np.eye(3), atol=1e-6)
            or not np.allclose(np.linalg.det(rotations), 1., atol=1e-6)):
        raise ValueError("reference rotations must be proper orthonormal rotations")
    if not body_boxes:
        raise ValueError("explicit local body geometry is required")
    _validate_boxes(body_boxes)
    if isinstance(subdivisions, bool) or not isinstance(subdivisions, (int, np.integer)) or subdivisions < 1:
        raise ValueError("subdivisions must be a positive integer")
    hit = np.zeros(len(voxels), dtype=bool)
    margins = []
    for segment in range(len(positions) - 1):
        p0, p1 = positions[segment:segment + 2]
        pair = Rotation.from_matrix(rotations[segment:segment + 2])
        angle = (pair[0].inv() * pair[1]).magnitude() / subdivisions
        midpoint_times = (np.arange(subdivisions) + .5) / subdivisions
        midpoint_rotations = Slerp([0., 1.], pair)(midpoint_times).as_matrix()
        translation_bound = np.linalg.norm(p1 - p0) / (2 * subdivisions)
        for t, rotation in zip(midpoint_times, midpoint_rotations):
            midpoint = p0 + t * (p1 - p0)
            envelopes = []
            for box in body_boxes:
                radius = np.linalg.norm(box.center) + np.linalg.norm(box.half_extents)
                margin = translation_bound + 2 * radius * np.sin(angle / 4)
                margins.append(float(margin))
                envelopes.append(BodyBox(
                    midpoint + rotation @ box.center,
                    box.half_extents + margin,
                    rotation @ box.rotation,
                ))
            # Exact broad phase against the envelope AABBs retains the FULL
            # supplied map; it is not a front/rear-query neighbourhood crop.
            candidate = np.zeros(len(voxels), dtype=bool)
            lo = voxels * resolution
            hi = lo + resolution
            for box in envelopes:
                extent = np.abs(box.rotation) @ box.half_extents
                candidate |= np.all(hi >= box.center - extent, axis=1) & np.all(lo <= box.center + extent, axis=1)
            indices = np.flatnonzero(candidate & ~hit)
            hit[indices] |= body_voxel_hits(voxels[indices], resolution, tuple(envelopes))
    return {
        "schema": "body_sweep_shadow/v1",
        "runtime_permit": False,
        "conservative_observed_collision": bool(hit.any()),
        "hit_indices": voxels[hit].tolist(),
        "occupied_voxel_count": len(voxels),
        "segment_count": len(positions) - 1,
        "subdivisions": int(subdivisions),
        "max_envelope_padding_m": max(margins),
        "scope": "all supplied occupied voxels; map completeness is caller responsibility; "
                 "linear reference translation and shortest-arc SO(3) interpolation only; "
                 "no unknown-space, non-stair inflation, foot or stability certification",
    }
