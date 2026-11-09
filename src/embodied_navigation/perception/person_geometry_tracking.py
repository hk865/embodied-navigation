"""Bounded learned-person identity and quarantine evidence from measured poses.

This helper owns no ROS, SDK, detector, oracle IDs or archived pose reader.
Unmasked background is UNKNOWN. A quarantined map AABB is a visible-geometry
envelope, never a new image mask, complete body volume or free-space proof.
"""

from __future__ import annotations

import copy
import json
import math
import uuid
from dataclasses import dataclass
from numbers import Real

import numpy as np

from embodied_navigation.perception.person_segmentation import project_masks

UNKNOWN, DYNAMIC = 0, 2
VELOCITY_SEMANTICS = "estimated_map_visible_centroid_difference_not_rigid_body_velocity"


def _stamp(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer source-clock timestamp")
    return value


def _positive(value, name):
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(value)
        or value <= 0
    ):
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def _iou(first, second):
    if first is None or second is None or first.shape != second.shape:
        return 0.0
    union = np.count_nonzero(first | second)
    return float(np.count_nonzero(first & second) / union) if union else 0.0


@dataclass(frozen=True)
class MotionRegion:
    """Caller-supplied current 2D motion estimate for an existing internal ID.

    ``from_source_ns`` must match that track's preceding fresh mask or accepted
    motion region. Chaining never refreshes the detector/quarantine horizon.
    """

    track_id: str
    from_source_ns: int
    source_ns: int
    mask: np.ndarray
    quality: float
    producer: str = "rgb_optical_flow"
    kind: str = "geometric"

    def __post_init__(self):
        _stamp(self.from_source_ns, "from_source_ns")
        _stamp(self.source_ns, "source_ns")
        if self.source_ns <= self.from_source_ns:
            raise ValueError("motion region must advance its original source timestamp")
        mask = np.array(self.mask, copy=True)
        if mask.dtype != bool or mask.ndim != 2:
            raise ValueError("motion mask must be a boolean image")
        if (
            not isinstance(self.track_id, str)
            or not self.track_id
            or not isinstance(self.producer, str)
            or not self.producer
            or self.kind != "geometric"
            or isinstance(self.quality, bool)
            or not isinstance(self.quality, Real)
            or not math.isfinite(self.quality)
            or not 0 <= self.quality <= 1
        ):
            raise ValueError(
                "motion evidence requires an internal ID, geometric producer and quality in [0,1]"
            )
        mask.flags.writeable = False
        object.__setattr__(self, "mask", mask)


@dataclass(frozen=True)
class PersonTrackingResult:
    states: np.ndarray
    instance_ids: tuple
    labels: dict
    provenance: dict
    quarantine_volumes: tuple
    active_tracks: tuple
    receipt: dict


class PersonGeometryTracker:
    """Small, one-to-one person tracks with explicit source-time expiry.

    High-confidence masks are immediate conservative DYNAMIC evidence. Low
    masks stay UNKNOWN until associated geometry and actual temporal/2D support
    corroborate a candidate. This is not general re-identification or an
    accuracy guarantee. No observation, expiry or zero velocity admits STATIC.
    """

    def __init__(
        self,
        *,
        epoch=1,
        frame_id="go2_imu",
        calibration_id="rgbd:v1",
        map_frame_id="dynanav_lio_map",
        high_score_threshold=0.5,
        low_score_threshold=0.1,
        min_geometry_points=3,
        max_tracks=32,
        max_points=8192,
        max_detections=64,
        max_mask_age_ns=500_000_000,
        track_ttl_ns=3_000_000_000,
        quarantine_horizon_ns=500_000_000,
        max_causal_pose_age_ns=250_000_000,
        max_2d_age_ns=500_000_000,
        association_distance_m=0.65,
        association_max_distance_m=1.5,
        extent_gate_m=1.0,
        ambiguity_margin_m=0.10,
        minimum_iou=0.15,
        minimum_motion_quality=0.7,
        geometry_inlier_radius_m=1.0,
        max_geometry_extent_m=3.0,
        quarantine_margin_m=0.10,
        max_visible_speed_mps=2.0,
        max_prediction_distance_m=0.30,
    ):
        if type(epoch) is not int or epoch < 1:
            raise ValueError("epoch must be positive integer")
        if any(
            not isinstance(value, str) or not value
            for value in (frame_id, calibration_id, map_frame_id)
        ):
            raise ValueError("sensor/calibration/map frame identities are required")
        if (
            any(
                isinstance(v, bool) or not isinstance(v, Real)
                for v in (low_score_threshold, high_score_threshold)
            )
            or not 0 < low_score_threshold <= high_score_threshold <= 1
        ):
            raise ValueError("score thresholds must satisfy 0 < low <= high <= 1")
        if (
            any(
                isinstance(v, bool) or not isinstance(v, Real)
                for v in (minimum_iou, minimum_motion_quality)
            )
            or not 0 < minimum_iou <= 1
            or not 0 <= minimum_motion_quality <= 1
        ):
            raise ValueError("IoU/motion quality thresholds must be probabilities")
        for name, value in (
            ("min_geometry_points", min_geometry_points),
            ("max_tracks", max_tracks),
            ("max_points", max_points),
            ("max_detections", max_detections),
            ("max_mask_age_ns", max_mask_age_ns),
            ("track_ttl_ns", track_ttl_ns),
            ("quarantine_horizon_ns", quarantine_horizon_ns),
            ("max_causal_pose_age_ns", max_causal_pose_age_ns),
            ("max_2d_age_ns", max_2d_age_ns),
        ):
            if type(value) is not int or value < 1:
                raise ValueError(f"{name} must be positive integer")
            setattr(self, name, value)
        if quarantine_horizon_ns > track_ttl_ns:
            raise ValueError("quarantine horizon cannot exceed track TTL")
        for name, value in (
            ("association_distance_m", association_distance_m),
            ("association_max_distance_m", association_max_distance_m),
            ("extent_gate_m", extent_gate_m),
            ("ambiguity_margin_m", ambiguity_margin_m),
            ("geometry_inlier_radius_m", geometry_inlier_radius_m),
            ("max_geometry_extent_m", max_geometry_extent_m),
            ("quarantine_margin_m", quarantine_margin_m),
            ("max_visible_speed_mps", max_visible_speed_mps),
            ("max_prediction_distance_m", max_prediction_distance_m),
        ):
            setattr(self, name, _positive(value, name))
        if self.association_max_distance_m < self.association_distance_m:
            raise ValueError(
                "maximum association distance cannot be smaller than base distance"
            )
        self.epoch, self.frame_id, self.calibration_id, self.map_frame_id = (
            epoch,
            frame_id,
            calibration_id,
            map_frame_id,
        )
        self.high_score_threshold, self.low_score_threshold = (
            high_score_threshold,
            low_score_threshold,
        )
        self.minimum_iou, self.minimum_motion_quality = (
            minimum_iou,
            minimum_motion_quality,
        )
        self.tracker_id = uuid.uuid4().hex
        self.next_id, self.last_source_ns = 1, -1
        self.tracks, self.image_shape = {}, None

    def _pose(self, source_ns, transform, pose_source_ns, pose_source):
        if transform is None:
            if pose_source_ns is not None:
                raise ValueError("pose timestamp without a measured pose is invalid")
            return None, {"status": "missing", "source_ns": None, "age_ns": None}
        if pose_source != "slam":
            raise ValueError("only declared measured SLAM poses are accepted")
        _stamp(pose_source_ns, "pose_source_ns")
        if pose_source_ns > source_ns:
            raise ValueError("future pose cannot be used causally")
        matrix = np.array(transform, dtype=float, copy=True)
        if (
            matrix.shape != (4, 4)
            or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(
                matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6, rtol=0
            )
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-6, rtol=0)
        ):
            raise ValueError("measured pose must be finite rigid T_map_sensor")
        age = source_ns - pose_source_ns
        status = "exact" if age == 0 else "causal"
        if age > self.max_causal_pose_age_ns:
            matrix, status = None, "stale"
        return matrix, {"status": status, "source_ns": pose_source_ns, "age_ns": age}

    def _geometry(self, points_map, owned):
        if points_map is None or np.count_nonzero(owned) < self.min_geometry_points:
            return None
        visible = points_map[owned]
        inliers = (
            np.linalg.norm(visible - np.median(visible, axis=0), axis=1)
            <= self.geometry_inlier_radius_m
        )
        selected = visible[inliers]
        if len(selected) < self.min_geometry_points or len(selected) < len(visible) / 2:
            return None
        bounds = np.stack((selected.min(axis=0), selected.max(axis=0)))
        extent = bounds[1] - bounds[0]
        if np.max(extent) > self.max_geometry_extent_m:
            return None
        return {
            "center": selected.mean(axis=0),
            "bounds": bounds,
            "extent": extent,
            "points": len(selected),
            "visible_points": len(visible),
        }

    def _shift(self, track, source_ns):
        velocity = track.get("velocity")
        if velocity is None:
            return np.zeros(3)
        seconds = (
            min(max(0, source_ns - track["geometry_ns"]), self.quarantine_horizon_ns)
            * 1e-9
        )
        shift = np.asarray(velocity) * seconds
        length = np.linalg.norm(shift)
        if length > self.max_prediction_distance_m:
            shift *= self.max_prediction_distance_m / length
        return shift

    def current_quarantine(self, source_ns):
        """JSON-safe map AABBs; never project them into an exact current mask.

        Vectorized bounds: np.asarray([v['bounds_map'] for v in result],
        dtype=float).reshape(-1, 2, 3). Querying does not refresh or mutate tracks.
        """
        _stamp(source_ns, "source_ns")
        result = []
        for identity, track in sorted(self.tracks.items()):
            if track.get("geometry") is None:
                continue
            age = source_ns - track["geometry_ns"]
            pose_age = track["geometry_pose"]["age_ns"]
            valid_until = min(
                track["last_detection_ns"] + self.track_ttl_ns,
                track["geometry_ns"] + self.quarantine_horizon_ns - pose_age,
            )
            if (
                age < 0
                or source_ns < track["last_detection_ns"]
                or source_ns > valid_until
            ):
                continue
            shift = self._shift(track, source_ns)
            margin = (
                self.quarantine_margin_m + pose_age * 1e-9 * self.max_visible_speed_mps
            )
            bounds = track["geometry"]["bounds"] + shift
            bounds = bounds + np.array([[-margin] * 3, [margin] * 3])
            result.append(
                {
                    "track_id": identity,
                    "bounds_map": bounds.tolist(),
                    "source_ns": track["geometry_ns"],
                    "last_detection_ns": track["last_detection_ns"],
                    "valid_until_ns": valid_until,
                    "age_ns": age,
                    "map_frame_id": self.map_frame_id,
                    "pose_source_ns": track["geometry_pose"]["source_ns"],
                    "pose_age_ns": pose_age,
                    "pose_status": track["geometry_pose"]["status"],
                    "confirmed": track["confirmed"],
                    "predicted_translation_m": shift.tolist(),
                    "scope": "visible_geometry_quarantine_only_not_current_mask_or_complete_body_volume",
                }
            )
        return tuple(result)

    def query(self, source_ns):
        _stamp(source_ns, "source_ns")
        result = []
        for identity, track in sorted(self.tracks.items()):
            age = source_ns - track["last_detection_ns"]
            if not 0 <= age <= self.track_ttl_ns:
                continue
            geometry = track.get("geometry")
            result.append(
                {
                    "track_id": identity,
                    "confirmed": track["confirmed"],
                    "hits": track["hits"],
                    "last_detection_ns": track["last_detection_ns"],
                    "age_ns": age,
                    "expires_ns": track["last_detection_ns"] + self.track_ttl_ns,
                    "visible_centroid_map": None
                    if geometry is None
                    else geometry["center"].tolist(),
                    "visible_extent_map": None
                    if geometry is None
                    else geometry["extent"].tolist(),
                    "visible_centroid_velocity_map": copy.deepcopy(
                        track.get("velocity")
                    ),
                    "velocity_semantics": VELOCITY_SEMANTICS,
                    "motion_evidence": copy.deepcopy(track.get("motion_evidence")),
                    "last_pose": copy.deepcopy(track["last_pose"]),
                    "last_score": track["score"],
                    "last_band": track["band"],
                }
            )
        return tuple(result)

    def update(
        self,
        source_ns,
        points_sensor,
        pixel_flat_indices,
        *,
        masks=None,
        scores=None,
        mask_source_ns=None,
        produced_ns=None,
        now_ns=None,
        T_map_sensor=None,
        pose_source_ns=None,
        pose_source="slam",
        image_shape=None,
        temporal_support=None,
        motion_regions=(),
        detector_provenance=None,
    ):
        """Consume one source-ordered frame; no external instance IDs are accepted.

        All *_ns values are in the caller's source clock, not wall latency.
        Missing/stale/wrong-stamp masks produce UNKNOWN. Wall inference latency
        belongs in detector_provenance and is never fabricated here.
        """
        _stamp(source_ns, "source_ns")
        if source_ns <= self.last_source_ns:
            raise ValueError("tracking source timestamps must strictly increase")
        now_ns = source_ns if now_ns is None else _stamp(now_ns, "now_ns")
        produced_ns = (
            now_ns if produced_ns is None else _stamp(produced_ns, "produced_ns")
        )
        if now_ns < source_ns:
            raise ValueError("current source clock cannot precede observation")
        points = np.array(points_sensor, dtype=float, copy=True)
        pixels = np.asarray(pixel_flat_indices)
        if (
            points.ndim != 2
            or points.shape[1] != 3
            or len(points) > self.max_points
            or not np.isfinite(points).all()
            or pixels.shape != (len(points),)
            or not np.issubdtype(pixels.dtype, np.integer)
            or np.any(pixels < 0)
            or len(np.unique(pixels)) != len(pixels)
        ):
            raise ValueError(
                "finite bounded Nx3 points and unique original integer pixels are required"
            )
        matrix, pose = self._pose(source_ns, T_map_sensor, pose_source_ns, pose_source)
        points_map = (
            None if matrix is None else points @ matrix[:3, :3].T + matrix[:3, 3]
        )
        shape = self.image_shape if image_shape is None else tuple(image_shape)
        if masks is not None:
            masks, scores = np.asarray(masks), np.asarray(scores)
            if (
                masks.dtype != bool
                or masks.ndim != 3
                or len(masks) > self.max_detections
                or scores.shape != (len(masks),)
                or not np.isfinite(scores).all()
                or np.any(scores < 0)
                or np.any(scores > 1)
            ):
                raise ValueError(
                    "boolean MxHxW masks and finite per-instance probabilities are required"
                )
            if shape is not None and shape != masks.shape[1:]:
                raise ValueError("mask/image calibration shape mismatch")
            shape = masks.shape[1:]
        if shape is not None and (
            len(shape) != 2
            or any(type(x) is not int or x < 1 for x in shape)
            or np.any(pixels >= shape[0] * shape[1])
            or self.image_shape not in (None, shape)
        ):
            raise ValueError(
                "original image shape/pixels changed without a new tracker"
            )
        support = (
            np.zeros(0 if masks is None else len(masks), dtype=np.int64)
            if temporal_support is None
            else np.asarray(temporal_support)
        )
        if (
            support.shape != (0 if masks is None else len(masks),)
            or not np.issubdtype(support.dtype, np.integer)
            or np.any(support < 0)
        ):
            raise ValueError(
                "temporal_support must contain nonnegative counts per current detection"
            )
        detector = copy.deepcopy(detector_provenance or {})
        if detector.get("kind", "learned") != "learned":
            raise ValueError("detector provenance must be learned, never oracle")
        detector.setdefault("kind", "learned")
        detector.setdefault("producer", "external_rgb_person_model")
        json.dumps(detector, allow_nan=False)
        status = "missing" if masks is None else "accepted"
        if masks is not None:
            if type(mask_source_ns) is not int or mask_source_ns != source_ns:
                status = "mask_source_mismatch"
            elif not source_ns <= produced_ns <= now_ns:
                status = "mask_production_time_invalid"
            elif now_ns - source_ns > self.max_mask_age_ns:
                status = "mask_expired"

        tracks = {
            key: copy.deepcopy(value)
            for key, value in self.tracks.items()
            if source_ns - value["last_detection_ns"] <= self.track_ttl_ns
        }
        motion, motion_receipts = {}, []
        motion_regions = tuple(motion_regions)
        if any(not isinstance(region, MotionRegion) for region in motion_regions):
            raise ValueError(
                "motion_regions must contain explicit MotionRegion evidence"
            )
        if len({region.track_id for region in motion_regions}) != len(motion_regions):
            raise ValueError("at most one motion region is allowed per internal track")
        for region in motion_regions:
            track = tracks.get(region.track_id)
            reason = "accepted"
            if track is None:
                reason = "unknown_internal_track"
            elif region.source_ns != source_ns or shape != region.mask.shape:
                reason = "motion_identity_mismatch"
            elif region.from_source_ns != track["mask_ns"]:
                reason = "motion_chain_mismatch"
            elif now_ns - source_ns > self.max_mask_age_ns:
                reason = "motion_observation_expired"
            elif source_ns - track["last_detection_ns"] > self.quarantine_horizon_ns:
                reason = "motion_horizon_expired"
            elif region.quality < self.minimum_motion_quality:
                reason = "motion_quality_low"
            elif region.track_id in motion:
                reason = "duplicate_motion_track"
            if reason == "accepted":
                motion[region.track_id] = region
            motion_receipts.append(
                {
                    "track_id": region.track_id,
                    "status": reason,
                    "from_source_ns": region.from_source_ns,
                    "source_ns": region.source_ns,
                    "quality": region.quality,
                    "producer": region.producer,
                }
            )

        states = np.zeros(len(points), dtype=np.uint8)
        identities, labels, detections = [""] * len(points), {}, []
        if status == "accepted":
            projected = project_masks(masks, pixels)
            owned_points = np.zeros(len(points), dtype=bool)
            for index in sorted(
                range(len(masks)), key=lambda j: (-float(scores[j]), j)
            ):
                if scores[index] < self.low_score_threshold:
                    continue
                owned = projected[index] & ~owned_points
                owned_points |= owned
                if not np.any(owned):
                    continue
                detections.append(
                    {
                        "index": index,
                        "score": float(scores[index]),
                        "owned": owned,
                        "mask": masks[index].copy(),
                        "geometry": self._geometry(points_map, owned),
                        "support": int(support[index]),
                        "candidates": [],
                    }
                )
        for detection in detections:
            geometry = detection["geometry"]
            for identity, track in tracks.items():
                age = source_ns - track["last_detection_ns"]
                if age < 0:
                    continue
                previous_mask = (
                    motion[identity].mask if identity in motion else track["mask"]
                )
                overlap = (
                    _iou(detection["mask"], previous_mask)
                    if identity in motion or age <= self.max_2d_age_ns
                    else 0.0
                )
                old_geometry = track.get("geometry")
                geometric = geometry is not None and old_geometry is not None
                if geometric:
                    delta = float(
                        np.linalg.norm(
                            geometry["center"]
                            - old_geometry["center"]
                            - self._shift(track, source_ns)
                        )
                    )
                    extent_delta = float(
                        np.linalg.norm(geometry["extent"] - old_geometry["extent"])
                    )
                    gate = min(
                        self.association_max_distance_m,
                        self.association_distance_m
                        + self.max_visible_speed_mps * age * 1e-9,
                    )
                    # A nanometre-scale numeric margin keeps an exactly-on-gate
                    # competitor from disappearing solely through float rounding.
                    if delta > gate + 1e-9 or extent_delta > self.extent_gate_m + 1e-9:
                        continue
                    cost = max(0.0, delta + 0.15 * extent_delta - 0.15 * overlap)
                elif overlap >= self.minimum_iou:
                    cost = (1 - overlap) * self.association_distance_m
                else:
                    continue
                detection["candidates"].append((cost, identity, overlap, geometric))
            detection["candidates"].sort()
        # Mutual, separated best candidates only. Ambiguity does not silently
        # merge two people or use detector index order as a persistent ID.
        by_track = {}
        for index, detection in enumerate(detections):
            for candidate in detection["candidates"]:
                by_track.setdefault(candidate[1], []).append((candidate[0], index))
        assignments = {}
        for index, detection in enumerate(detections):
            candidates = detection["candidates"]
            if not candidates:
                continue
            if (
                len(candidates) > 1
                and candidates[1][0] - candidates[0][0] <= self.ambiguity_margin_m
            ):
                continue
            best = candidates[0]
            competitors = sorted(by_track[best[1]])
            if competitors[0][1] != index or (
                len(competitors) > 1
                and competitors[1][0] - competitors[0][0] <= self.ambiguity_margin_m
            ):
                continue
            assignments[index] = best

        detection_receipts = []
        next_id = self.next_id
        for index, detection in enumerate(detections):
            assigned = assignments.get(index)
            identity = assigned[1] if assigned is not None else None
            previous = tracks.get(identity)
            high = detection["score"] >= self.high_score_threshold
            geometric_temporal = (
                assigned is not None
                and assigned[3]
                and pose["status"] == "exact"
                and previous["last_pose"]["status"] == "exact"
                and (assigned[2] >= self.minimum_iou or detection["support"] >= 2)
            )
            confirmed = (
                high
                or (previous is not None and previous["confirmed"])
                or geometric_temporal
            )
            band = (
                "high_confidence"
                if high
                else "low_associated_confirmed_track"
                if previous is not None and previous["confirmed"]
                else "low_geometry_temporal_supported"
                if confirmed
                else "low_unconfirmed"
            )
            association = (
                "matched"
                if identity is not None
                else "ambiguous"
                if detection["candidates"]
                else "new"
            )
            can_allocate = (
                association != "ambiguous"
                and len(tracks) < self.max_tracks
                and (high or detection["geometry"] is not None)
            )
            if identity is None and can_allocate:
                identity = f"person-track:{self.tracker_id}:{self.epoch}:{next_id:06d}"
                next_id += 1
            if identity is not None:
                track = previous or {
                    "hits": 0,
                    "confirmed": False,
                    "geometry": None,
                    "velocity": None,
                }
                geometry, old = detection["geometry"], track.get("geometry")
                evidence = {
                    "available": False,
                    "reason": "requires_two_exact_measured_geometries",
                    "velocity_semantics": VELOCITY_SEMANTICS,
                }
                velocity = None
                if (
                    geometry is not None
                    and old is not None
                    and pose["status"] == "exact"
                    and track["geometry_pose"]["status"] == "exact"
                ):
                    duration = (source_ns - track["geometry_ns"]) * 1e-9
                    delta = geometry["center"] - old["center"]
                    observed = delta / duration
                    evidence = {
                        "available": True,
                        "from_source_ns": track["geometry_ns"],
                        "to_source_ns": source_ns,
                        "duration_s": duration,
                        "visible_centroid_delta_m": delta.tolist(),
                        "visible_centroid_velocity_map": observed.tolist(),
                        "visible_extent_delta_m": (
                            geometry["extent"] - old["extent"]
                        ).tolist(),
                        "velocity_semantics": VELOCITY_SEMANTICS,
                        "rigid_body_motion_verified": False,
                    }
                    if np.linalg.norm(observed) <= self.max_visible_speed_mps:
                        velocity = observed.tolist()
                track.update(
                    last_detection_ns=source_ns,
                    last_pose=copy.deepcopy(pose),
                    mask=detection["mask"],
                    mask_ns=source_ns,
                    hits=track["hits"] + 1,
                    confirmed=confirmed,
                    score=detection["score"],
                    band=band,
                    motion_evidence=evidence,
                    velocity=velocity,
                    detector_provenance=copy.deepcopy(detector),
                )
                if geometry is not None:
                    track.update(
                        geometry=geometry,
                        geometry_ns=source_ns,
                        geometry_pose=copy.deepcopy(pose),
                    )
                tracks[identity] = track
            if confirmed:
                point_id = (
                    identity
                    or f"person-detection:{self.tracker_id}:{source_ns}:{detection['index']}"
                )
                states[detection["owned"]] = DYNAMIC
                for point in np.flatnonzero(detection["owned"]):
                    identities[point] = point_id
                labels[point_id] = "person"
            detection_receipts.append(
                {
                    "prediction_mask_index": detection["index"],
                    "track_id": identity,
                    "score": detection["score"],
                    "band": band,
                    "association": association,
                    "point_count": int(np.count_nonzero(detection["owned"])),
                    "geometry_points": 0
                    if detection["geometry"] is None
                    else detection["geometry"]["points"],
                    "temporal_support": detection["support"],
                    "confirmed": confirmed,
                    "association_geometry_used": assigned[3]
                    if assigned is not None
                    else False,
                    "association_mask_iou": assigned[2]
                    if assigned is not None
                    else None,
                }
            )

        propagation = []
        if shape is not None:
            claims = np.zeros(len(points), dtype=np.int32)
            proposals = []
            for identity, region in motion.items():
                track = tracks[identity]
                if not track["confirmed"]:
                    continue
                selected = region.mask.reshape(-1)[pixels] & (states == UNKNOWN)
                claims += selected
                proposals.append((identity, region, selected))
            for identity, region, selected in proposals:
                selected &= claims == 1
                states[selected] = DYNAMIC
                for point in np.flatnonzero(selected):
                    identities[point] = identity
                if np.any(selected):
                    labels[identity] = "person"
                # A fresh detector update wins over a prior motion chain.
                if tracks[identity]["last_detection_ns"] != source_ns and np.any(
                    selected
                ):
                    tracks[identity].update(mask=region.mask.copy(), mask_ns=source_ns)
                propagation.append(
                    {
                        "track_id": identity,
                        "points": int(np.count_nonzero(selected)),
                        "from_source_ns": region.from_source_ns,
                        "source_ns": source_ns,
                        "age_since_detection_ns": source_ns
                        - tracks[identity]["last_detection_ns"],
                        "producer": region.producer,
                        "current_model_mask": False,
                    }
                )
        self.tracks, self.next_id, self.last_source_ns, self.image_shape = (
            tracks,
            next_id,
            source_ns,
            shape,
        )
        provenance = {
            "kind": "learned",
            "producer": "person_geometry_tracker/v1",
            "instance_id_scope": "epoch",
            "tracker_id": self.tracker_id,
            "detector": detector,
            "mask_status": status,
            "source_ns": source_ns,
            "produced_ns": produced_ns,
            "admitted_now_ns": now_ns,
            "pose": pose,
            "pose_source": "slam" if matrix is not None else "unavailable",
            "map_frame_id": self.map_frame_id,
            "high_score_threshold": self.high_score_threshold,
            "low_score_threshold": self.low_score_threshold,
            "background_policy": "UNKNOWN; absence, expiry and zero motion never imply STATIC",
            "motion_propagation": propagation,
            "motion_regions": motion_receipts,
            "velocity_semantics": VELOCITY_SEMANTICS,
            "motion_limitations": "Visibility/occlusion and SLAM pose error can change the centroid; no rigid-body velocity is established",
            "quarantine_scope": "map visible-geometry envelopes only; not exact current image masks",
            "upstream_pose_oracle_free_attested": False,
        }
        receipt = {
            "source_ns": source_ns,
            "mask_status": status,
            "pose": copy.deepcopy(pose),
            "detections": detection_receipts,
            "motion_regions": motion_receipts,
            "dynamic_points": int(np.count_nonzero(states == DYNAMIC)),
            "unknown_points": int(np.count_nonzero(states == UNKNOWN)),
            "static_points": 0,
            "active_tracks": len(tracks),
            "acceptance_scope": "helper evidence only; caller owns live integration and acceptance",
        }
        states.flags.writeable = False
        return PersonTrackingResult(
            states,
            tuple(identities),
            labels,
            provenance,
            self.current_quarantine(source_ns),
            self.query(source_ns),
            receipt,
        )
