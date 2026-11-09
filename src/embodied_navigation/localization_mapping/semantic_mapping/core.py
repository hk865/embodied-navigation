"""Owned, provenance-bound point layers and a small queryable object map.

No ROS, simulator, detector or network imports. A mask is evidence for one
observation, never a permission to classify the next image as static.
"""

from __future__ import annotations

import copy
import json
import os
import tempfile
import uuid
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

UNKNOWN, STATIC, DYNAMIC = 0, 1, 2


def _integer(value, name, minimum=0):
    if type(value) is not int or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")


def _transform(value):
    result = np.array(value, dtype=float, copy=True)
    if (
        result.shape != (4, 4)
        or not np.isfinite(result).all()
        or not np.allclose(result[3], [0, 0, 0, 1], rtol=0, atol=1e-9)
        or not np.allclose(
            result[:3, :3].T @ result[:3, :3], np.eye(3), rtol=0, atol=1e-6
        )
        or not np.isclose(np.linalg.det(result[:3, :3]), 1, rtol=0, atol=1e-6)
    ):
        raise ValueError("pose must be a finite rigid T_map_sensor")
    result.flags.writeable = False
    return result


def _world(points, transform):
    return np.asarray(points) @ transform[:3, :3].T + transform[:3, 3]


@dataclass(frozen=True)
class Observation:
    epoch: int
    sequence: int
    source_ns: int
    pose_source_ns: int
    points_sensor: np.ndarray
    T_map_sensor: np.ndarray
    anchor_id: str
    frame_id: str
    calibration_id: str
    pose_source: str = "slam"
    pose_revision: int = 0

    def __post_init__(self):
        for name in (
            "epoch",
            "sequence",
            "source_ns",
            "pose_source_ns",
            "pose_revision",
        ):
            _integer(getattr(self, name), name, 1 if name == "epoch" else 0)
        if self.pose_source_ns != self.source_ns:
            raise ValueError("pose and measurement timestamps must match")
        if self.pose_source not in ("slam", "sim_oracle"):
            raise ValueError("pose_source must explicitly identify slam or sim_oracle")
        if not all(
            isinstance(v, str) and v
            for v in (self.anchor_id, self.frame_id, self.calibration_id)
        ):
            raise ValueError("anchor/frame/calibration identities are required")
        points = np.array(self.points_sensor, dtype=float, copy=True)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError("points_sensor must be finite Nx3")
        points.flags.writeable = False
        object.__setattr__(self, "points_sensor", points)
        object.__setattr__(self, "T_map_sensor", _transform(self.T_map_sensor))

    @property
    def observation_id(self):
        return f"{self.epoch}:{self.sequence}"


@dataclass(frozen=True)
class PointMask:
    epoch: int
    sequence: int
    source_ns: int
    produced_ns: int
    frame_id: str
    calibration_id: str
    states: np.ndarray
    provenance: dict
    instance_ids: tuple = ()
    labels: dict = field(default_factory=dict)

    def __post_init__(self):
        for name in ("epoch", "sequence", "source_ns", "produced_ns"):
            _integer(getattr(self, name), name, 1 if name == "epoch" else 0)
        states = np.array(self.states, copy=True)
        if (
            states.ndim != 1
            or not np.issubdtype(states.dtype, np.integer)
            or not np.isin(states, [UNKNOWN, STATIC, DYNAMIC]).all()
        ):
            raise ValueError("mask states must be integer UNKNOWN/STATIC/DYNAMIC")
        if self.instance_ids and (
            len(self.instance_ids) != len(states)
            or any(not isinstance(x, str) for x in self.instance_ids)
        ):
            raise ValueError(
                "instance_ids must match final point order; empty string means none"
            )
        provenance = copy.deepcopy(self.provenance)
        if (
            not isinstance(provenance, dict)
            or provenance.get("kind") not in ("sim_oracle", "learned", "geometric")
            or not isinstance(provenance.get("producer"), str)
            or not provenance["producer"]
        ):
            raise ValueError("mask provenance requires explicit kind and producer")
        json.dumps(provenance, allow_nan=False)
        if provenance.get("instance_id_scope", "frame") not in ("frame", "epoch"):
            raise ValueError("instance_id_scope must be frame or explicit epoch")
        if any(
            not isinstance(k, str) or not isinstance(v, str)
            for k, v in self.labels.items()
        ):
            raise ValueError("labels map frame-local instance strings to class strings")
        states = states.astype(np.uint8)
        states.flags.writeable = False
        object.__setattr__(self, "states", states)
        object.__setattr__(self, "instance_ids", tuple(self.instance_ids))
        object.__setattr__(self, "labels", copy.deepcopy(self.labels))
        object.__setattr__(self, "provenance", provenance)


class LayeredMap:
    """One epoch of bounded static contributions, transient points and object IDs.

    Persistent static geometry contains ONLY points carrying valid STATIC
    evidence. Unknown/dynamic points remain available in navigation_points.
    Caller supplies selected keyframes, not an unbounded camera stream.
    """

    def __init__(
        self,
        *,
        map_id=None,
        frame_id="map",
        voxel_size=0.08,
        max_observations=512,
        max_points_per_observation=8192,
        max_objects=256,
        history_length=32,
        max_mask_age_ns=2_000_000_000,
        object_ttl_ns=5_000_000_000,
        association_distance=1.0,
        movable_labels=("person", "pedestrian", "human"),
    ):
        for name, value in (
            ("max_observations", max_observations),
            ("max_points_per_observation", max_points_per_observation),
            ("max_objects", max_objects),
            ("history_length", history_length),
            ("max_mask_age_ns", max_mask_age_ns),
            ("object_ttl_ns", object_ttl_ns),
        ):
            _integer(value, name, 1)
        if (
            not np.isfinite([voxel_size, association_distance]).all()
            or min(voxel_size, association_distance) <= 0
        ):
            raise ValueError("positive finite voxel/association distances required")
        self.map_id, self.frame_id = map_id or str(uuid.uuid4()), frame_id
        self.voxel_size, self.association_distance = voxel_size, association_distance
        self.max_observations = max_observations
        self.max_points_per_observation = max_points_per_observation
        self.max_objects, self.history_length = max_objects, history_length
        self.max_mask_age_ns, self.object_ttl_ns = max_mask_age_ns, object_ttl_ns
        self.movable_labels = set(movable_labels)
        self.epoch = None
        self.revision = 0
        self.last_sequence = self.last_source_ns = -1
        self.anchors, self.observations, self.objects = {}, {}, {}
        self.epoch_instance_ids = {}
        self.next_object = 1
        self.capacity_rejections = 0
        self.mask_rejections = 0

    def _states(self, observation, mask, now_ns):
        reason = None
        if mask is None:
            reason = "missing_mask"
        elif (
            mask.epoch,
            mask.sequence,
            mask.source_ns,
            mask.frame_id,
            mask.calibration_id,
        ) != (
            observation.epoch,
            observation.sequence,
            observation.source_ns,
            observation.frame_id,
            observation.calibration_id,
        ):
            reason = "mask_identity_mismatch"
        elif len(mask.states) != len(observation.points_sensor):
            reason = "mask_point_count_mismatch"
        elif not observation.source_ns <= mask.produced_ns <= now_ns:
            reason = "mask_time_mismatch"
        elif now_ns - observation.source_ns > self.max_mask_age_ns:
            reason = "expired_mask"
        if reason:
            return np.zeros(len(observation.points_sensor), dtype=np.uint8), reason
        states = mask.states.copy()
        for instance in set(mask.instance_ids) - {""}:
            selected = np.array([x == instance for x in mask.instance_ids])
            if mask.labels.get(instance, "").lower() in self.movable_labels or np.any(
                states[selected] == DYNAMIC
            ):
                states[selected] = DYNAMIC
        return states, "accepted"

    def integrate(self, observation, mask=None, *, now_ns=None):
        now_ns = observation.source_ns if now_ns is None else now_ns
        _integer(now_ns, "now_ns")
        if self.epoch is not None and observation.epoch != self.epoch:
            raise ValueError("new clock epoch requires a new LayeredMap segment")
        if (
            observation.sequence <= self.last_sequence
            or observation.source_ns <= self.last_source_ns
        ):
            raise ValueError("duplicate/backward observation")
        if len(observation.points_sensor) > self.max_points_per_observation:
            raise ValueError(
                "point budget exceeded; producer must bound owned keyframes"
            )
        existing = self.anchors.get(observation.anchor_id)
        if existing is not None and (
            existing[1] != observation.pose_revision
            or not np.array_equal(existing[0], observation.T_map_sensor)
        ):
            raise ValueError("anchor changed without explicit update_anchor")
        if now_ns < observation.source_ns:
            raise ValueError("measurement is ahead of current clock")
        self.epoch = observation.epoch
        self.last_sequence, self.last_source_ns = (
            observation.sequence,
            observation.source_ns,
        )
        states, reason = self._states(observation, mask, now_ns)
        navigation = _world(observation.points_sensor, observation.T_map_sensor)
        persisted = len(self.observations) < self.max_observations
        if persisted:
            self.anchors[observation.anchor_id] = (
                observation.T_map_sensor,
                observation.pose_revision,
            )
            record = {
                "observation": observation,
                "states": states,
                "mask": copy.deepcopy(mask) if reason == "accepted" else None,
                "reason": reason,
                "object_ids": {},
            }
            self.observations[observation.observation_id] = record
            if reason == "accepted":
                self._associate(record)
                states = self._effective_states(record)
        else:
            self.capacity_rejections += 1
        if reason != "accepted":
            self.mask_rejections += 1
        self.revision += 1
        return {
            "observation_id": observation.observation_id,
            "map_revision": self.revision,
            "navigation_points": navigation,
            "navigation_points_sensor": observation.points_sensor.copy(),
            "static_points": navigation[states == STATIC],
            "dynamic_points": navigation[states == DYNAMIC],
            "unknown_points": navigation[states == UNKNOWN],
            "static_indices": np.flatnonzero(states == STATIC),
            "dynamic_indices": np.flatnonzero(states == DYNAMIC),
            "unknown_indices": np.flatnonzero(states == UNKNOWN),
            "mask_status": reason,
            "persisted": persisted,
        }

    def revise_mask(self, observation_id, mask, *, now_ns):
        """Apply delayed evidence only to its retained observation; replace its contribution."""
        _integer(now_ns, "now_ns")
        record = self.observations[observation_id]
        states, reason = self._states(record["observation"], mask, now_ns)
        if reason != "accepted":
            self.mask_rejections += 1
            return False
        for ident in record["object_ids"].values():
            track = self.objects[ident]
            track["history"] = [
                x for x in track["history"] if x["observation_id"] != observation_id
            ]
        record.update(states=states, mask=copy.deepcopy(mask), reason=reason)
        self._associate(record)
        self.revision += 1
        return True

    def _associate(self, record):
        obs, mask = record["observation"], record["mask"]
        used = set()
        for instance in sorted(set(mask.instance_ids) - {""}):
            indices = np.array([x == instance for x in mask.instance_ids])
            valid = indices & (record["states"] != UNKNOWN)
            if not np.any(valid):
                continue
            center = obs.points_sensor[valid].mean(axis=0)
            center_world = _world(center, self.anchors[obs.anchor_id][0])
            label = mask.labels.get(instance, "unknown")
            status = (
                "dynamic" if np.any(record["states"][valid] == DYNAMIC) else "static"
            )
            object_id = record["object_ids"].get(instance)
            scope = mask.provenance.get("instance_id_scope", "frame")
            stable_key = (
                (
                    obs.epoch,
                    mask.provenance["kind"],
                    mask.provenance["producer"],
                    instance,
                )
                if scope == "epoch"
                else None
            )
            if stable_key is not None:
                object_id = self.epoch_instance_ids.get(stable_key)
            candidates = []
            if object_id is None and stable_key is None:
                for ident, track in self.objects.items():
                    if not track["history"]:
                        continue
                    last = max(track["history"], key=lambda x: x["source_ns"])
                    if (
                        ident in used
                        or not (
                            track["label"] == label
                            or "unknown" in (track["label"], label)
                        )
                        or not 0
                        <= obs.source_ns - last["source_ns"]
                        <= self.object_ttl_ns
                    ):
                        continue
                    distance = float(
                        np.linalg.norm(center_world - self._entry_world(last))
                    )
                    if distance <= self.association_distance:
                        candidates.append((distance, ident))
                candidates.sort()
                # Ambiguous association creates a new ID instead of merging two entities.
                if candidates and (
                    len(candidates) == 1 or candidates[1][0] - candidates[0][0] > 1e-6
                ):
                    object_id = candidates[0][1]
            if object_id is None:
                if len(self.objects) >= self.max_objects:
                    self.capacity_rejections += 1
                    continue
                object_id = f"{self.map_id}:object:{self.next_object:06d}"
                self.next_object += 1
                self.objects[object_id] = {
                    "object_id": object_id,
                    "label": label,
                    "history": [],
                }
            if stable_key is not None:
                self.epoch_instance_ids[stable_key] = object_id
            used.add(object_id)
            record["object_ids"][instance] = object_id
            track = self.objects[object_id]
            entry = {
                "observation_id": obs.observation_id,
                "source_ns": obs.source_ns,
                "anchor_id": obs.anchor_id,
                "center_sensor": center.tolist(),
                "status": status,
                "label": label,
                "epoch": obs.epoch,
                "frame_id": obs.frame_id,
                "calibration_id": obs.calibration_id,
                "pose_source_ns": obs.pose_source_ns,
                "source_instance_id": instance,
                "instance_id_scope": scope,
                "pose_source": obs.pose_source,
                "provenance": copy.deepcopy(mask.provenance),
            }
            track["history"] = [
                x for x in track["history"] if x["observation_id"] != obs.observation_id
            ]
            track["history"].append(entry)
            track["history"] = sorted(track["history"], key=lambda x: x["source_ns"])[
                -self.history_length :
            ]
            track["label"] = track["history"][-1]["label"]
            track["ever_dynamic"] = (
                track.get("ever_dynamic", False) or status == "dynamic"
            )

    def _entry_world(self, entry):
        return _world(entry["center_sensor"], self.anchors[entry["anchor_id"]][0])

    def update_anchor(self, anchor_id, T_map_sensor, *, revision):
        """Reproject static contributions and object histories without reallocating IDs."""
        _integer(revision, "revision")
        old = self.anchors[anchor_id]
        if revision <= old[1]:
            raise ValueError("anchor revision must increase")
        self.anchors[anchor_id] = (_transform(T_map_sensor), revision)
        self.revision += 1

    def _effective_states(self, record):
        states = record["states"].copy()
        mask = record["mask"]
        if mask is not None and mask.instance_ids:
            for instance, ident in record["object_ids"].items():
                if self.objects[ident].get("ever_dynamic", False):
                    states[np.array([x == instance for x in mask.instance_ids])] = (
                        DYNAMIC
                    )
        return states

    def static_points(self):
        voxels = {}
        for record in self.observations.values():
            obs = record["observation"]
            points = _world(
                obs.points_sensor[self._effective_states(record) == STATIC],
                self.anchors[obs.anchor_id][0],
            )
            for point in points:
                voxels[tuple(np.floor(point / self.voxel_size).astype(np.int64))] = (
                    point
                )
        return np.array([voxels[key] for key in sorted(voxels)], dtype=float).reshape(
            -1, 3
        )

    def query(
        self,
        *,
        label=None,
        center=None,
        radius=None,
        now_ns=None,
        include_expired=False,
        include_history=True,
    ):
        now_ns = max(0, self.last_source_ns) if now_ns is None else now_ns
        _integer(now_ns, "now_ns")
        if (center is None) != (radius is None):
            raise ValueError("center and radius must be supplied together")
        if center is not None:
            center = np.asarray(center, dtype=float)
            if (
                center.shape != (3,)
                or not np.isfinite(center).all()
                or not np.isfinite(radius)
                or radius < 0
            ):
                raise ValueError("finite XYZ center/nonnegative radius required")
        results = []
        for ident, track in sorted(self.objects.items()):
            if not track["history"]:
                continue
            last = track["history"][-1]
            age = now_ns - last["source_ns"]
            status = "dynamic" if track.get("ever_dynamic") else last["status"]
            expired = status == "dynamic" and age > self.object_ttl_ns
            position = self._entry_world(last)
            if (
                (label is not None and track["label"] != label)
                or age < 0
                or expired
                and not include_expired
                or center is not None
                and np.linalg.norm(position - center) > radius
            ):
                continue
            entries = track["history"] if include_history else track["history"][-2:]
            history = [
                dict(copy.deepcopy(x), position=self._entry_world(x).tolist())
                for x in entries
            ]
            velocity = [0.0, 0.0, 0.0]
            if len(history) >= 2:
                a, b = history[-2:]
                dt = (b["source_ns"] - a["source_ns"]) * 1e-9
                if dt > 0:
                    velocity = ((np.array(b["position"]) - a["position"]) / dt).tolist()
            result = {
                "object_id": ident,
                "label": track["label"],
                "status": status,
                "expired": expired,
                "source_ns": last["source_ns"],
                "age_ns": age,
                "position": position.tolist(),
                "velocity": velocity,
                "velocity_semantics": "observed_visible_centroid_derivative_in_revised_map_not_rigid_body_velocity",
                "source_instance_id": last["source_instance_id"],
                "instance_id_scope": last["instance_id_scope"],
                "source_epoch": last["epoch"],
                "sensor_frame_id": last["frame_id"],
                "calibration_id": last["calibration_id"],
                "pose_source": last["pose_source"],
                "pose_source_ns": last["pose_source_ns"],
                "anchor_id": last["anchor_id"],
                "anchor_revision": self.anchors[last["anchor_id"]][1],
                "T_map_sensor": self.anchors[last["anchor_id"]][0].tolist(),
                "provenance": copy.deepcopy(last["provenance"]),
            }
            if include_history:
                result["history"] = history
            results.append(result)
        return {
            "map_id": self.map_id,
            "epoch": self.epoch,
            "frame_id": self.frame_id,
            "revision": self.revision,
            "query_clock_ns": now_ns,
            "objects": results,
        }

    def summary(self, *, now_ns=None):
        """Compact object/count metadata; never builds or serializes the static point map."""
        query = (
            self.query(now_ns=now_ns, include_expired=True, include_history=False)
            if self.epoch is not None
            else {
                "map_id": self.map_id,
                "epoch": None,
                "frame_id": self.frame_id,
                "revision": self.revision,
                "objects": [],
            }
        )
        mask_kinds = sorted(
            {
                r["mask"].provenance["kind"]
                for r in self.observations.values()
                if r["mask"]
            }
        )
        pose_sources = sorted(
            {r["observation"].pose_source for r in self.observations.values()}
        )
        totals = {"static": 0, "dynamic": 0, "unknown": 0}
        for record in self.observations.values():
            states = self._effective_states(record)
            for key, value in (
                ("static", STATIC),
                ("dynamic", DYNAMIC),
                ("unknown", UNKNOWN),
            ):
                totals[key] += int(np.sum(states == value))
        semantic_complete = totals["unknown"] == 0 and all(
            r["reason"] == "accepted" for r in self.observations.values()
        )
        labels_complete = semantic_complete
        for record in self.observations.values():
            mask = record["mask"]
            if mask is None or (len(record["states"]) and not mask.instance_ids):
                labels_complete = False
                continue
            if any(
                not instance
                or mask.labels.get(instance, "unknown").strip().lower()
                in ("", "unknown", "unlabeled")
                for instance in set(mask.instance_ids)
            ):
                labels_complete = False
        return dict(
            schema="layered_observed_map/v1",
            **query,
            source_ns=self.last_source_ns,
            geometry_included=False,
            observation_count=len(self.observations),
            observation_point_counts=totals,
            mask_rejections=self.mask_rejections,
            capacity_rejections=self.capacity_rejections,
            complete=bool(self.observations)
            and self.capacity_rejections == 0
            and semantic_complete,
            capacity_complete=self.capacity_rejections == 0,
            mask_coverage_complete=bool(self.observations) and semantic_complete,
            semantic_label_coverage_complete=bool(self.observations)
            and labels_complete,
            label_coverage_scope="retained known points have instance IDs and non-unknown labels; generic oracle classes allowed; not learned accuracy or full-scene coverage",
            semantic_coverage_complete=bool(self.observations) and semantic_complete,
            semantic_coverage_complete_is_legacy_mask_alias=True,
            provenance_summary={
                "mask_kinds": mask_kinds,
                "pose_sources": pose_sources,
                "sim_oracle_used": "sim_oracle" in mask_kinds
                or "sim_oracle" in pose_sources,
            },
        )

    def snapshot(self, *, now_ns=None, include_points=True):
        result = self.summary(now_ns=now_ns)
        result["objects"] = self.query(now_ns=now_ns, include_expired=True)["objects"]
        if include_points:
            points = self.static_points()
            result.update(
                geometry_included=True,
                static_points=points.tolist(),
                static_point_count=len(points),
            )
        result["observations"] = [
            {
                "observation_id": k,
                "source_ns": r["observation"].source_ns,
                "anchor_id": r["observation"].anchor_id,
                "mask_status": r["reason"],
                "pose_source": r["observation"].pose_source,
                "mask_provenance": copy.deepcopy(r["mask"].provenance)
                if r["mask"]
                else None,
                "static_count": int(np.sum(self._effective_states(r) == STATIC)),
                "dynamic_count": int(np.sum(self._effective_states(r) == DYNAMIC)),
                "unknown_count": int(np.sum(self._effective_states(r) == UNKNOWN)),
            }
            for k, r in self.observations.items()
        ]
        return result

    def save(self, path, *, now_ns=None, include_points=True):
        """Atomically publish one self-contained read-only map/query revision."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(
            self.snapshot(now_ns=now_ns, include_points=include_points), allow_nan=False
        ).encode()
        with tempfile.NamedTemporaryFile(
            dir=path.parent, prefix=".scene_map_", delete=False
        ) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)


def read_snapshot(path):
    """Load the committed structured representation; IDs persist in this artifact."""
    value = json.loads(Path(path).read_text())
    if value.get("schema") != "layered_observed_map/v1":
        raise ValueError("unknown map snapshot schema")
    return value
