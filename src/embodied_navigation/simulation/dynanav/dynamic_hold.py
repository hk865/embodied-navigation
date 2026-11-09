"""Experiment-only sensor-frame pedestrian hold/resume decision.

This gate consumes dynamic points selected from the final sampled, self-masked
cloud. It does not filter the raw navigation cloud, alter routes/maps/policies,
or establish general avoidance or SLAM navigation. The caller owns exact mask
association and the IMU-to-base calibration. Parameters below are provisional
experiment settings, not measured braking performance.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from numbers import Integral

import numpy as np


@dataclass(frozen=True)
class DynamicHoldConfig:
    recorded_cruise_mps: float = 0.15
    reaction_time_s: float = 0.4
    deceleration_mps2: float = 0.3
    body_front_m: float = 0.35
    margin_m: float = 0.20
    corridor_half_width_m: float = 0.40
    corridor_z_min_m: float = -0.30
    corridor_z_max_m: float = 1.50
    max_frame_age_s: float = 0.50
    clear_frames_required: int = 2
    clear_dwell_s: float = 0.0

    def __post_init__(self):
        values = asdict(self)
        if not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
            for v in values.values()
        ):
            raise ValueError("Dynamic hold parameters must be finite numbers")
        if any(
            values[name] < 0
            for name in (
                "recorded_cruise_mps",
                "reaction_time_s",
                "body_front_m",
                "margin_m",
                "clear_dwell_s",
            )
        ):
            raise ValueError(
                "Dynamic hold distances, speed and durations must be nonnegative"
            )
        if (
            self.deceleration_mps2 <= 0
            or self.corridor_half_width_m <= 0
            or self.max_frame_age_s <= 0
        ):
            raise ValueError(
                "Deceleration, corridor width and freshness limit must be positive"
            )
        if self.corridor_z_min_m >= self.corridor_z_max_m:
            raise ValueError("Dynamic hold height interval is empty")
        if (
            not isinstance(self.clear_frames_required, Integral)
            or self.clear_frames_required < 1
        ):
            raise ValueError("At least one distinct clear frame is required")

    @property
    def stop_distance_m(self):
        speed = self.recorded_cruise_mps
        return (
            self.body_front_m
            + self.margin_m
            + speed * self.reaction_time_s
            + speed * speed / (2 * self.deceleration_mps2)
        )


def _stamp(value):
    return (
        int(value)
        if isinstance(value, Integral) and not isinstance(value, bool) and value >= 0
        else None
    )


class DynamicHold:
    """Forward corridor gate; initial state is unknown/hold.

    IMU and base axes must already align (+X forward, +Y left, +Z up).
    ``points_base = dynamic_points_sensor + imu_origin_in_base_m``; the caller
    must transform axes first if necessary. Every clearance frame requires an
    explicit valid mask with exactly the cloud's source timestamp. Distinct
    consecutive clear frames provide release evidence; optional dwell is counted
    between those observed clear timestamps, never by waiting without observations.
    """

    def __init__(
        self,
        config=None,
        *,
        imu_origin_in_base_m,
        mask_provenance="caller_supplied_dynamic_mask",
    ):
        self.config = DynamicHoldConfig() if config is None else config
        if not isinstance(self.config, DynamicHoldConfig):
            raise TypeError("Expected DynamicHoldConfig")
        self.offset = np.asarray(imu_origin_in_base_m, dtype=float)
        if self.offset.shape != (3,) or not np.isfinite(self.offset).all():
            raise ValueError("An explicit finite IMU origin in base XYZ is required")
        self.offset = self.offset.copy()
        self.mask_provenance = str(mask_provenance)
        self.hold = True
        self._source_ns = None
        self._last_now_ns = None
        self._valid = False
        self._clear_count = 0
        self._clear_since_ns = None
        self._clear_elapsed_ns = 0
        self._frame = {}
        self._reason, self._status = "no_current_frame", "unknown"

    def _reset_clear(self):
        self._clear_count = 0
        self._clear_since_ns = None
        self._clear_elapsed_ns = 0

    def _finish(self, hold, reason, status, now_ns):
        previous = self.hold
        self.hold, self._reason, self._status = bool(hold), reason, status
        config = asdict(self.config)
        receipt = dict(self._frame)
        receipt_source = receipt.get("source_ns")
        receipt.update(
            hold=self.hold,
            reason=reason,
            status=status,
            transition="resume"
            if previous and not self.hold
            else "hold"
            if not previous and self.hold
            else "none",
            now_ns=now_ns,
            frame_age_ns=None
            if now_ns is None or receipt_source is None
            else now_ns - receipt_source,
            clear_frames=self._clear_count,
            clear_since_ns=self._clear_since_ns,
            clear_observed_duration_s=self._clear_elapsed_ns / 1e9,
            stop_distance_from_base_m=self.config.stop_distance_m,
            stopping_components_m={
                "body_front": self.config.body_front_m,
                "margin": self.config.margin_m,
                "reaction": self.config.recorded_cruise_mps
                * self.config.reaction_time_s,
                "braking": self.config.recorded_cruise_mps**2
                / (2 * self.config.deceleration_mps2),
            },
            parameters=config,
            parameters_status="provisional_experiment_values_not_measured_braking",
            imu_origin_in_base_m=self.offset.tolist(),
            mask_provenance=self.mask_provenance,
            scope="sensor_frame_pedestrian_hold_resume_experiment",
            raw_navigation_cloud_modified=False,
            general_avoidance_verified=False,
            SLAM_navigation_verified=False,
        )
        return self.hold, receipt

    def _unknown(self, reason, now_ns):
        self._valid = False
        self._reset_clear()
        return self._finish(True, reason, "unknown", now_ns)

    def update(
        self,
        dynamic_points_sensor,
        *,
        source_ns,
        mask_source_ns,
        now_ns,
        mask_valid,
        sampled_point_count=None,
    ):
        """Evaluate one exact-timestamp dynamic slice, preserving its input buffer.

        ``mask_valid`` must indicate successful current-frame mask production;
        an empty dynamic slice alone cannot establish it. ``None`` points or a
        missing/mismatched/stale mask cause unknown hold. Optional final sampled
        cloud count detects empty source observations. Duplicate stamps can hold
        but cannot add clearance evidence. Call ``decision`` between RGB-D frames.
        """
        source, mask_source, now = (
            _stamp(source_ns),
            _stamp(mask_source_ns),
            _stamp(now_ns),
        )
        previous_source = self._source_ns
        valid_flag = isinstance(mask_valid, (bool, np.bool_)) and bool(mask_valid)
        self._frame = {
            "source_ns": source,
            "mask_source_ns": mask_source,
            "mask_valid": valid_flag,
            "dynamic_points": None,
            "corridor_points": None,
            "nearest_forward_x_m": None,
            "nearest_body_front_clearance_m": None,
            "sampled_point_count": int(sampled_point_count)
            if isinstance(sampled_point_count, Integral)
            else None,
            "distinct_frame": source is not None and source != previous_source,
            "clear_gap_reset": False,
        }
        if source is None or now is None:
            return self._unknown("invalid_timestamp", now)
        if self._last_now_ns is not None and now < self._last_now_ns:
            return self._unknown("nonmonotonic_evaluation_time", now)
        self._last_now_ns = now
        if previous_source is not None and source < previous_source:
            return self._unknown("nonmonotonic_source_timestamp", now)
        if now < source:
            return self._unknown("future_observation", now)
        if now - source > round(self.config.max_frame_age_s * 1e9):
            return self._unknown("stale_observation", now)
        self._source_ns = source
        if not valid_flag or mask_source is None:
            return self._unknown("missing_or_invalid_current_mask", now)
        if mask_source != source:
            return self._unknown("mask_timestamp_mismatch", now)
        if sampled_point_count is not None and (
            not isinstance(sampled_point_count, Integral)
            or isinstance(sampled_point_count, bool)
            or sampled_point_count <= 0
        ):
            return self._unknown("empty_or_invalid_sampled_cloud", now)
        if dynamic_points_sensor is None:
            return self._unknown("missing_dynamic_points", now)
        try:
            points = np.asarray(dynamic_points_sensor, dtype=float)
        except (TypeError, ValueError):
            return self._unknown("invalid_dynamic_points", now)
        if points.ndim == 1 and points.size == 0:
            points = points.reshape(0, 3)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            return self._unknown("invalid_dynamic_points", now)
        if sampled_point_count is not None and len(points) > sampled_point_count:
            return self._unknown("dynamic_count_exceeds_sampled_cloud", now)
        self._frame["dynamic_points"] = len(points)
        base = points + self.offset
        config = self.config
        selected = (
            (base[:, 0] >= 0)
            & (np.abs(base[:, 1]) <= config.corridor_half_width_m)
            & (base[:, 2] >= config.corridor_z_min_m)
            & (base[:, 2] <= config.corridor_z_max_m)
        )
        corridor = base[selected]
        self._frame["corridor_points"] = len(corridor)
        nearest = float(corridor[:, 0].min()) if len(corridor) else None
        self._frame["nearest_forward_x_m"] = nearest
        self._frame["nearest_body_front_clearance_m"] = (
            None if nearest is None else nearest - config.body_front_m
        )
        self._valid = True
        if nearest is not None and nearest <= config.stop_distance_m:
            self._reset_clear()
            return self._finish(
                True, "dynamic_points_in_stopping_corridor", "blocked", now
            )
        if previous_source is not None and source - previous_source > round(
            config.max_frame_age_s * 1e9
        ):
            self._reset_clear()
            self._frame["clear_gap_reset"] = True
        if source != previous_source:
            if self._clear_since_ns is None:
                self._clear_since_ns = source
            self._clear_count += 1
            self._clear_elapsed_ns = source - self._clear_since_ns
        ready = (
            self._clear_count >= config.clear_frames_required
            and self._clear_elapsed_ns >= round(config.clear_dwell_s * 1e9)
        )
        if (self.hold or self._frame["clear_gap_reset"]) and not ready:
            return self._finish(
                True, "awaiting_consecutive_clear_frames", "clear_pending", now
            )
        return self._finish(False, "fresh_corridor_clear", "clear", now)

    def decision(self, *, now_ns):
        """Reuse evidence between sensor frames while enforcing its age limit."""
        now = _stamp(now_ns)
        if now is None:
            return self._unknown("invalid_timestamp", None)
        if self._last_now_ns is not None and now < self._last_now_ns:
            return self._unknown("nonmonotonic_evaluation_time", now)
        self._last_now_ns = now
        if self._source_ns is None or not self._valid:
            return self._unknown(
                self._reason if self._source_ns is not None else "no_current_frame", now
            )
        if now < self._source_ns or now - self._source_ns > round(
            self.config.max_frame_age_s * 1e9
        ):
            return self._unknown("stale_observation", now)
        return self._finish(self.hold, self._reason, self._status, now)
