"""Bounded repeat-geometry candidates from exact, caller-supplied SLAM poses.

Agreement in a voxel is temporary geometric evidence, never proof that an
object is permanently static. No semantic-negative, oracle, ROS or SDK input
is read. Exported points are actual last-support samples, not voxel means.
Retention and quarantine advance in observation source time, not wall time.
"""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass

import numpy as np


def _integer(value, name, minimum=0):
    if (
        not isinstance(value, (int, np.integer))
        or isinstance(value, (bool, np.bool_))
        or value < minimum
    ):
        raise ValueError(f"{name} must be an integer >= {minimum}")
    return int(value)


def _rigid_pose(value):
    pose = np.array(value, dtype=float, copy=True)
    if (
        pose.shape != (4, 4)
        or not np.isfinite(pose).all()
        or not np.allclose(pose[3], [0, 0, 0, 1], rtol=0, atol=1e-9)
        or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), rtol=0, atol=1e-6)
        or not np.isclose(np.linalg.det(pose[:3, :3]), 1, rtol=0, atol=1e-6)
    ):
        raise ValueError("A finite rigid measured T_map_sensor is required")
    return pose


@dataclass(frozen=True)
class _Support:
    source_ns: int
    point_index: int
    point_map: tuple


class TemporalGeometryCache:
    """One segment of bounded, retractable repeat-geometry evidence.

    ``excluded_indices`` identifies current DYNAMIC/person samples in the
    original input order. One excluded point invalidates its entire voxel.
    ``person_quarantine`` is a map-frame array of inclusive AABBs (K,2,3).
    Intersecting voxels lose all votes and remain blocked for quarantine TTL.
    No detection is not STATIC evidence: only distinct geometric observations
    can establish a candidate, subject to all configured bounds.
    """

    def __init__(
        self,
        *,
        voxel_size=0.1,
        min_distinct_frames=3,
        min_span_ns=500_000_000,
        retention_ns=300_000_000_000,
        quarantine_ttl_ns=2_000_000_000,
        max_voxels=200_000,
        max_supports_per_voxel=8,
        max_quarantine_voxels=50_000,
        max_person_regions=256,
        max_pose_age_ns=500_000_000,
    ):
        if not np.isfinite(voxel_size) or voxel_size <= 0:
            raise ValueError("voxel_size must be finite and positive")
        self.voxel_size = float(voxel_size)
        for name, value in (
            ("min_distinct_frames", min_distinct_frames),
            ("min_span_ns", min_span_ns),
            ("retention_ns", retention_ns),
            ("quarantine_ttl_ns", quarantine_ttl_ns),
            ("max_voxels", max_voxels),
            ("max_supports_per_voxel", max_supports_per_voxel),
            ("max_quarantine_voxels", max_quarantine_voxels),
            ("max_person_regions", max_person_regions),
            ("max_pose_age_ns", max_pose_age_ns),
        ):
            setattr(
                self,
                name,
                _integer(value, name, 3 if name == "min_distinct_frames" else 1),
            )
        if self.min_span_ns > self.retention_ns:
            raise ValueError("retention_ns must cover the required support span")
        if self.max_supports_per_voxel < self.min_distinct_frames:
            raise ValueError("support capacity must cover min_distinct_frames")
        self._voxels = {}
        self._quarantine = {}
        self._regions = {}
        self._global_quarantine_until_ns = -1
        self._last_source_ns = -1
        self._counts = Counter()

    def _keys(self, points):
        scaled = np.floor(np.asarray(points, dtype=float) / self.voxel_size)
        if not np.isfinite(scaled).all() or np.any(np.abs(scaled) >= 2**62):
            raise ValueError("Coordinates exceed representable voxel-key range")
        return scaled.astype(np.int64)

    @staticmethod
    def _inside(key, bounds):
        lower, upper = bounds
        return all(low <= value <= high for value, low, high in zip(key, lower, upper))

    def _confirmed(self, supports):
        return len(supports) >= self.min_distinct_frames and (
            supports[-1].source_ns - supports[0].source_ns >= self.min_span_ns
        )

    def _expire(self, source_ns, events):
        for key, supports in list(self._voxels.items()):
            while supports and source_ns - supports[0].source_ns > self.retention_ns:
                supports.popleft()
                events["retention_expired_supports"] += 1
            if not supports:
                del self._voxels[key]
                events["retention_expired_voxels"] += 1
        for key, until in list(self._quarantine.items()):
            if source_ns >= until:
                del self._quarantine[key]
                events["expired_quarantine_voxels"] += 1
        for region, until in list(self._regions.items()):
            if source_ns >= until:
                del self._regions[region]
                events["expired_person_regions"] += 1

    def _retract(self, key, events):
        supports = self._voxels.pop(key, None)
        if supports is not None:
            events["retracted_voxels"] += 1
            events["retracted_supports"] += len(supports)

    def _fail_closed(self, until, events):
        self._global_quarantine_until_ns = max(self._global_quarantine_until_ns, until)
        events["global_quarantine_activations"] += 1
        for key in list(self._voxels):
            self._retract(key, events)

    def update(
        self,
        points_sensor,
        T_map_sensor,
        source_ns,
        excluded_indices=None,
        person_quarantine=None,
        *,
        pose_source_ns,
        pose_source="slam",
        now_ns=None,
    ):
        """Return original admitted indices and a small JSON-compatible receipt.

        Missing/mismatched/non-SLAM poses fail before changing state; there is
        no GT fallback. ``now_ns`` optionally checks observation/pose age in
        the same source-clock domain. Without it this only checks exact source
        alignment, not runtime freshness or upstream pose authenticity.
        Duplicate source stamps can retract danger but never add support or
        return a second admitted observation. Backward source time is rejected:
        a caller must carry still-relevant delayed person AABBs into a current
        update, never replay an old frame as new positive evidence.
        """
        source_ns = _integer(source_ns, "source_ns")
        if pose_source != "slam" or pose_source_ns is None:
            raise ValueError("An exact measured SLAM pose is required; no fallback")
        if _integer(pose_source_ns, "pose_source_ns") != source_ns:
            raise ValueError("Pose stamp must exactly match observation source_ns")
        if source_ns < self._last_source_ns:
            raise ValueError("Backward source time requires a new cache segment")
        if now_ns is not None:
            age = _integer(now_ns, "now_ns") - source_ns
            if not 0 <= age <= self.max_pose_age_ns:
                raise ValueError("Observation/pose is future or stale in source clock")
        pose = _rigid_pose(T_map_sensor)
        points = np.asarray(points_sensor, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError("points_sensor must be finite Nx3")
        excluded = np.zeros(len(points), dtype=bool)
        if excluded_indices is not None:
            indices = np.asarray(excluded_indices)
            if indices.dtype == bool:
                if indices.shape != excluded.shape:
                    raise ValueError(
                        "Excluded bool mask must match original point count"
                    )
                excluded = indices.copy()
            elif indices.ndim == 1 and np.issubdtype(indices.dtype, np.integer):
                if np.any(indices < 0) or np.any(indices >= len(points)):
                    raise ValueError("Excluded index is outside original point order")
                excluded[indices] = True
            elif indices.shape != (0,):
                raise ValueError(
                    "Excluded values must be a point bool mask or integer indices"
                )
        boxes = (
            np.empty((0, 2, 3), dtype=float)
            if person_quarantine is None
            else np.asarray(person_quarantine, dtype=float)
        )
        if boxes.shape == (0,):
            boxes = np.empty((0, 2, 3), dtype=float)
        if boxes.ndim != 3 or boxes.shape[1:] != (2, 3) or not np.isfinite(boxes).all():
            raise ValueError("person_quarantine must be finite map-frame (K,2,3) AABBs")
        if np.any(boxes[:, 0] > boxes[:, 1]):
            raise ValueError("Person AABB minimum exceeds maximum")
        mapped = points @ pose[:3, :3].T + pose[:3, 3]
        keys = self._keys(mapped)
        regions = [tuple(map(tuple, self._keys(box))) for box in boxes]
        duplicate = source_ns == self._last_source_ns
        events = Counter()
        self._expire(source_ns, events)
        until = source_ns + self.quarantine_ttl_ns
        for key in {tuple(row) for row in keys[excluded]}:
            self._retract(key, events)
            if (
                key not in self._quarantine
                and len(self._quarantine) >= self.max_quarantine_voxels
            ):
                events["quarantine_voxel_capacity_rejections"] += 1
                self._fail_closed(until, events)
            else:
                self._quarantine[key] = until
        for region in regions:
            if (
                region not in self._regions
                and len(self._regions) >= self.max_person_regions
            ):
                events["person_region_capacity_rejections"] += 1
                self._fail_closed(until, events)
            else:
                self._regions[region] = until
        if regions and self._voxels:
            cached_keys = np.asarray(list(self._voxels), dtype=np.int64)
            retract = np.zeros(len(cached_keys), dtype=bool)
            for lower, upper in regions:
                retract |= np.all(cached_keys >= lower, axis=1) & np.all(
                    cached_keys <= upper, axis=1
                )
            for key in cached_keys[retract]:
                self._retract(tuple(key), events)
        blocked = excluded.copy()
        if source_ns < self._global_quarantine_until_ns:
            blocked[:] = True
        elif len(keys):
            blocked |= np.fromiter(
                (tuple(row) in self._quarantine for row in keys),
                dtype=bool,
                count=len(keys),
            )
            for lower, upper in self._regions:
                blocked |= np.all(keys >= lower, axis=1) & np.all(keys <= upper, axis=1)
        eligible = np.flatnonzero(~blocked)
        if not duplicate and len(eligible):
            unique, first, inverse = np.unique(
                keys[eligible], axis=0, return_index=True, return_inverse=True
            )
            group_sizes = np.bincount(inverse)
            for row, first_index, group_size in zip(unique, first, group_sizes):
                key = tuple(row)
                if key not in self._voxels:
                    if len(self._voxels) >= self.max_voxels:
                        events["voxel_capacity_rejections"] += 1
                        events["capacity_rejected_points"] += int(group_size)
                        continue
                    self._voxels[key] = deque()
                supports = self._voxels[key]
                if len(supports) >= self.max_supports_per_voxel:
                    supports.popleft()
                    events["support_capacity_evictions"] += 1
                index = int(eligible[first_index])
                supports.append(
                    _Support(source_ns, index, tuple(float(x) for x in mapped[index]))
                )
                events["support_votes_added"] += 1
        admitted = []
        if not duplicate:
            for index in eligible:
                supports = self._voxels.get(tuple(keys[index]))
                if supports and self._confirmed(supports):
                    admitted.append(int(index))
        self._last_source_ns = source_ns
        events["updates"] += 1
        events["duplicate_updates"] += int(duplicate)
        self._counts.update(events)
        receipt = {
            "schema": "temporal_repeat_geometry_candidate/v1",
            "source_ns": source_ns,
            "pose_source_ns": source_ns,
            "pose_source": "slam",
            "exact_source_alignment": True,
            "runtime_age_checked": now_ns is not None,
            "upstream_pose_authenticity_or_oracle_free_verified": False,
            "duplicate_source_no_vote": duplicate,
            "input_points": len(points),
            "excluded_points": int(excluded.sum()),
            "quarantined_points": int(blocked.sum()),
            "admitted_static_indices_count": len(admitted),
            "unknown_unadmitted_points": len(points)
            - len(admitted)
            - int(excluded.sum()),
            "events": dict(events),
            "counts": self.counts(),
            "parameters": self.parameters(),
            "representative": "last support's first original point; never an average",
            "scope": "temporary repeat-geometry candidate, not permanent semantic static or free space",
        }
        return np.asarray(admitted, dtype=np.int64), receipt

    def parameters(self):
        return {
            name: getattr(self, name)
            for name in (
                "voxel_size",
                "min_distinct_frames",
                "min_span_ns",
                "retention_ns",
                "quarantine_ttl_ns",
                "max_voxels",
                "max_supports_per_voxel",
                "max_quarantine_voxels",
                "max_person_regions",
                "max_pose_age_ns",
            )
        }

    def counts(self):
        """Counts as of the last update source time, with no implicit wall clock."""
        return {
            "last_source_ns": self._last_source_ns,
            "candidate_voxels": len(self._voxels),
            "confirmed_voxels": sum(self._confirmed(v) for v in self._voxels.values()),
            "retained_supports": sum(len(v) for v in self._voxels.values()),
            "quarantine_voxels": len(self._quarantine),
            "person_regions": len(self._regions),
            "global_quarantine_until_ns": self._global_quarantine_until_ns,
            "global_quarantine_active": self._last_source_ns
            < self._global_quarantine_until_ns,
            "cumulative": dict(self._counts),
        }

    def confirmed_records(self):
        """JSON-compatible provenance ledger, evaluated at last update source time.

        Support point_index refers to the original input of that source_ns.
        A caller must not treat an idle cache as current runtime evidence.
        """
        records = []
        for key, supports in sorted(self._voxels.items()):
            if not self._confirmed(supports):
                continue
            last = supports[-1]
            records.append(
                {
                    "voxel_key": [int(x) for x in key],
                    "point_map": list(last.point_map),
                    "representative_source_ns": last.source_ns,
                    "last_observed_source_ns": last.source_ns,
                    "representative_point_index": last.point_index,
                    "representative_method": "last_support_first_original_point",
                    "supports": [
                        {"source_ns": s.source_ns, "point_index": s.point_index}
                        for s in supports
                    ],
                    "support_span_ns": last.source_ns - supports[0].source_ns,
                    "scope": "repeat_geometry_candidate",
                }
            )
        return records

    def confirmed_voxel_points(self):
        return np.asarray(
            [r["point_map"] for r in self.confirmed_records()], dtype=float
        ).reshape(-1, 3)
