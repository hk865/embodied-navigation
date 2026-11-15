"""OFF-by-default, bounded auxiliary fusion scheduling. No ROS or file writes.

This thread experiment permits callback service while geometry runs; it does
not promise parallel Python execution or a shorter geometry deadline. All input
bytes are owned. The main owner alone receives, submits, polls and publishes.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import threading
import time

import numpy as np

MANIFEST_LIMIT = 2 * 1024 * 1024
ARCHIVE_LIMIT = 16 * 1024 * 1024


def file_version(path):
    s = Path(path).stat()
    return (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)


def _read_owned(path, limit):
    path = Path(path)
    before = file_version(path)
    if before[2] > limit:
        raise ValueError("async_source_too_large")
    with path.open("rb") as handle:
        raw = handle.read(limit + 1)
    if len(raw) > limit or len(raw) != before[2] or file_version(path) != before:
        raise ValueError("async_source_changed_during_read")
    return raw, before


def _record(raw):
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError("depth_manifest_not_object")
    return value


def _stamp(record):
    stamp = record.get("stamp_ns")
    if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
        raise ValueError("depth_stamp_ns must be positive integer nanoseconds")
    return stamp


@dataclass(frozen=True)
class DepthFrameSnapshot:
    """Raw input, not a validation certificate. The normal loader validates it."""
    pointer: str
    stamp_ns: int
    latest_bytes: bytes
    record_path: str
    record_bytes: bytes
    archive_path: str
    archive_bytes: bytes
    canonical_path: str
    canonical_version: tuple
    archive_version: tuple
    freeze_wall_s: float

    @classmethod
    def capture(cls, manifest_path, stamp_ns):
        start = time.perf_counter()
        pointer = Path(manifest_path).resolve()
        latest, _ = _read_owned(pointer, MANIFEST_LIMIT)
        latest_record = _record(latest)
        latest_stamp = _stamp(latest_record)
        canonical = pointer.parent / ("frame_%019d.json" % stamp_ns)
        raw, version = _read_owned(canonical, MANIFEST_LIMIT)
        record = _record(raw)
        if _stamp(record) != stamp_ns:
            raise ValueError("depth_stamp_mismatch")
        if latest_stamp == stamp_ns and record != latest_record:
            raise ValueError("async_depth_pointer_record_mismatch")
        record_path = pointer if latest_stamp == stamp_ns else canonical
        record_raw = latest if latest_stamp == stamp_ns else raw
        archive = Path(str(record.get("npz_path", ""))).resolve()
        if archive.parent != pointer.parent or archive.name != "frame_%019d.npz" % stamp_ns:
            raise ValueError("depth_archive_path_mismatch")
        data, archive_version = _read_owned(archive, ARCHIVE_LIMIT)
        if file_version(canonical) != version:
            raise ValueError("async_source_changed_during_read")
        # Hash validation is repeated by load_depth_frame. Capture only makes a
        # bounded exact pair; unverified calibration/RP/mask are not trusted.
        return cls(str(pointer), int(stamp_ns), bytes(latest), str(record_path), bytes(record_raw),
                   str(archive), bytes(data), str(canonical), version, archive_version,
                   time.perf_counter() - start)

    def read_record(self, path):
        path = str(Path(path).resolve())
        if path == self.pointer:
            raw = self.latest_bytes
        elif path == self.canonical_path:
            raw = self.record_bytes
        else:
            raise ValueError("async_depth_snapshot_path_mismatch")
        if len(raw) > MANIFEST_LIMIT:
            raise ValueError("depth_manifest_too_large")
        return _record(raw)

    def read_archive(self, path):
        if str(Path(path).resolve()) != self.archive_path:
            raise ValueError("async_depth_snapshot_archive_mismatch")
        return self.archive_bytes

    def verify_current(self, authority=None):
        # This mutable high-water/cache belongs exclusively to the main owner.
        # The owned snapshot remains immutable and is not a safety certificate.
        authority = {} if authority is None else authority
        if (file_version(self.canonical_path) != self.canonical_version
                or file_version(self.archive_path) != self.archive_version):
            raise ValueError("async_depth_source_version_changed")
        latest, pointer_version = _read_owned(self.pointer, MANIFEST_LIMIT)
        record = _record(latest)
        latest_stamp = _stamp(record)
        if latest_stamp < max(self.stamp_ns, _stamp(_record(self.latest_bytes)),
                              authority.get("last_seen_stamp", 0)):
            raise ValueError("async_depth_pointer_rolled_back")
        old = _record(self.record_bytes)
        if (record.get("schema") != old.get("schema") or record.get("source") != old.get("source")
                or (latest_stamp == self.stamp_ns and record != old)):
            raise ValueError("async_depth_pointer_invalid")
        if latest_stamp != self.stamp_ns:
            canonical = Path(self.pointer).parent / ("frame_%019d.json" % latest_stamp)
            archive = canonical.with_suffix(".npz")
            key = (pointer_version, file_version(canonical), file_version(archive))
            if authority.get("validated_key") != key:
                begin = time.perf_counter()
                current = DepthFrameSnapshot.capture(self.pointer, latest_stamp)
                if current.latest_bytes != latest:
                    raise ValueError("async_depth_pointer_changed_during_recheck")
                from .depth_safety_fusion import _validate_depth_pointer_integrity
                _validate_depth_pointer_integrity(self.pointer, reference_stamp_ns=latest_stamp,
                                                  snapshot=current)
                if (file_version(self.pointer), file_version(canonical), file_version(archive)) != key:
                    raise ValueError("async_depth_pointer_changed_during_recheck")
                authority["validated_key"] = key
                authority["integrity_checks"] = authority.get("integrity_checks", 0) + 1
                authority["integrity_wall_ns"] = authority.get("integrity_wall_ns", 0) + round(
                    (time.perf_counter() - begin) * 1e9)
            else:
                authority["integrity_cache_hits"] = authority.get("integrity_cache_hits", 0) + 1
        # This exact frame is still fully checked by the worker/public loader;
        # newer index metadata must never replace its source, arrays or clocks.
        if (file_version(self.canonical_path) != self.canonical_version
                or file_version(self.archive_path) != self.archive_version):
            raise ValueError("async_depth_source_changed_during_recheck")
        authority["last_seen_stamp"] = latest_stamp
        return True


@dataclass(frozen=True)
class FusionSnapshot:
    generation: int
    sequence: int
    stamp_ns: int
    points_bytes: bytes
    points_shape: tuple
    pose_json: bytes
    pair_json: bytes
    config_json: bytes
    binary_sha256: str
    cloud_arrival_wall_s: float
    depth: DepthFrameSnapshot
    freeze_wall_s: float

    @classmethod
    def capture(cls, *, generation, sequence, points_map, pose, pair, config,
                binary_sha256, cloud_arrival_wall_s, depth_manifest):
        start = time.perf_counter()
        points = np.asarray(points_map, dtype="<f8")
        if points.ndim != 2 or points.shape[1] != 3 or not 0 < len(points) <= 100_000:
            raise ValueError("async_native_points_invalid")
        stamp = pair["reference_stamp_ns"]
        if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
            raise ValueError("async_stamp_invalid")
        encode = lambda value: json.dumps(value, sort_keys=True, allow_nan=False).encode()
        depth = DepthFrameSnapshot.capture(depth_manifest, stamp)
        return cls(generation, sequence, stamp, points.tobytes(), points.shape,
                   encode({k: pose[k] for k in ("position", "quat_xyzw", "arrival_wall_s")}),
                   encode(pair), encode(config), binary_sha256, float(cloud_arrival_wall_s),
                   depth, time.perf_counter() - start)

    @property
    def points(self):
        # Bytes backing makes setflags(write=True) impossible as well as normal
        # mutation. Each call returns a view of only our owned immutable bytes.
        return np.frombuffer(self.points_bytes, dtype="<f8").reshape(self.points_shape)

    def validate_ages(self, clock, wall):
        from .depth_safety_fusion import validate_fusion_source_ages
        depth = _record(self.depth.record_bytes)
        pose = json.loads(self.pose_json)
        validate_fusion_source_ages({
            "rtx": {"stamp_ns": self.stamp_ns, "arrival_wall_s": self.cloud_arrival_wall_s,
                    "pose_arrival_wall_s": pose["arrival_wall_s"]},
            "depth": {"stamp_ns": _stamp(depth), "arrival_wall_s": depth.get("arrival_wall_s")},
        }, now_sim_s=clock, now_wall_s=wall)


class ReceivedClock:
    """One atomic pair of values actually delivered by /clock, no extrapolation."""
    def __init__(self):
        self._lock = threading.Lock()
        self._sample = (None, None, 0)

    def update(self, stamp, arrival):
        with self._lock:
            old, _, generation = self._sample
            if old is not None and stamp < old:
                generation += 1
            self._sample = (stamp, arrival, generation)
            return generation

    def read(self, wall):
        with self._lock:
            stamp, arrival, generation = self._sample
        usable = (stamp is not None and arrival is not None and math.isfinite(stamp)
                  and math.isfinite(arrival) and 0 <= wall - arrival <= .25)
        return (stamp if usable else None), generation


def compute_fusion(snapshot, received_clock, wall_clock=time.perf_counter):
    """Same geometry/validation as the synchronous bridge, with frozen raw input."""
    from .depth_safety_fusion import build_safety_fusion, native_source_envelope
    from .rtx_deskewed_perception import build_perception_payload, map_points_to_base
    begin = wall_clock()
    now_sim, generation = received_clock.read(begin)
    if generation != snapshot.generation:
        raise ValueError("async_generation_changed")
    snapshot.validate_ages(now_sim, begin)
    pose, pair, config = map(json.loads, (snapshot.pose_json, snapshot.pair_json, snapshot.config_json))
    points = map_points_to_base(snapshot.points, pose["position"], pose["quat_xyzw"], config["imu_mount"])
    producer = dict(pair, cloud_topic=config["cloud_topic"], pose_topic=config["odometry_topic"],
                    binary_sha256=snapshot.binary_sha256)
    native = native_source_envelope(points, stamp_s=snapshot.stamp_ns * 1e-9,
                                    arrival_wall_s=snapshot.cloud_arrival_wall_s,
                                    producer_metadata=producer)
    payload = build_safety_fusion(
        points, native, pair=pair, pose_arrival_wall_s=pose["arrival_wall_s"],
        depth_manifest=snapshot.depth.pointer, depth_snapshot=snapshot.depth,
        now_sim_s=now_sim, now_wall_s=begin, enabled=True,
        native_fallback_factory=lambda: build_perception_payload(
            points, stamp_s=snapshot.stamp_ns * 1e-9,
            arrival_wall_s=snapshot.cloud_arrival_wall_s,
            producer_metadata=producer, source_topic=config["cloud_topic"]),
    )
    end = wall_clock()
    now_sim, generation = received_clock.read(end)
    if generation != snapshot.generation:
        raise ValueError("async_generation_changed")
    snapshot.validate_ages(now_sim, end)
    from .depth_safety_fusion import FUSION_SOURCE, validate_fusion_contract
    if payload.get("source") == FUSION_SOURCE:
        validate_fusion_contract(payload, now_sim_s=now_sim, now_wall_s=end)
    diagnostic = payload["depth_safety_diagnostic"]
    diagnostic.update(accepted_input=diagnostic.get("accepted") is True,
                      accepted_post_compute=diagnostic.get("accepted") is True,
                      computation_started_wall_s=begin, computation_finished_wall_s=end,
                      computation_wall_s=end - begin, post_compute_wall_s=end,
                      snapshot_freeze_wall_s=snapshot.freeze_wall_s,
                      async_generation=generation, async_sequence=snapshot.sequence,
                      consumer_source_sha256=config.get("consumer_source_sha256"),
                      native_cloud_evidence=config.get("native_cloud_evidence"),
                      depth_source_versions={"manifest": list(snapshot.depth.canonical_version),
                                             "archive": list(snapshot.depth.archive_version)},
                      clock_extrapolated=False)
    return payload


@dataclass(frozen=True)
class FusionResult:
    snapshot: FusionSnapshot
    payload: dict | None
    error: str | None


class LatestFusionWorker:
    """One active job + one pending job + one result; no executor or publishing."""
    def __init__(self, compute):
        self._compute = compute
        self._condition = threading.Condition()
        self._pending = self._result = self._active = None
        self._closed = False
        self._generation = 0
        self._last_submitted = (-1, -1)
        self._stats = Counter()
        self._thread = threading.Thread(target=self._run, name="aux-depth-fusion", daemon=True)
        self._thread.start()

    def submit(self, snapshot):
        with self._condition:
            if self._closed or snapshot.generation != self._generation:
                self._stats["submission_old_generation_or_closed"] += 1
                return False
            if (snapshot.sequence <= self._last_submitted[0]
                    or snapshot.stamp_ns <= self._last_submitted[1]):
                self._stats["submission_out_of_order"] += 1
                return False
            self._last_submitted = (snapshot.sequence, snapshot.stamp_ns)
            if self._pending is not None:
                self._stats["pending_replaced"] += 1
            self._pending = snapshot
            self._stats["submitted"] += 1
            self._condition.notify()
            return True

    def generation(self, value):
        with self._condition:
            if value != self._generation:
                self._generation = value
                self._stats["generation_changes"] += 1
                self._stats["pending_generation_discarded"] += int(self._pending is not None)
                self._stats["result_generation_discarded"] += int(self._result is not None)
                self._pending = self._result = None
                self._last_submitted = (-1, -1)

    def poll(self):
        with self._condition:
            result, self._result = self._result, None
            return result

    def stats(self):
        with self._condition:
            return dict(self._stats, pending=int(self._pending is not None),
                        active=int(self._active is not None), completed=int(self._result is not None),
                        closed=self._closed, worker_alive=self._thread.is_alive())

    def close(self, timeout=1.0):
        with self._condition:
            self._closed = True
            self._stats["shutdown_pending_discarded"] += int(self._pending is not None)
            self._stats["shutdown_result_discarded"] += int(self._result is not None)
            self._pending = self._result = None
            self._condition.notify()
        self._thread.join(timeout=timeout)
        return not self._thread.is_alive()

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._pending is not None)
                if self._closed:
                    return
                job, self._pending = self._pending, None
                self._active = job
            try:
                result = FusionResult(job, self._compute(job), None)
            except Exception as error:  # A failed job cannot kill the fail-closed owner loop.
                result = FusionResult(job, None, type(error).__name__ + ":" + str(error))
            with self._condition:
                self._active = None
                if self._closed or job.generation != self._generation:
                    self._stats["completion_old_generation_or_closed"] += 1
                    continue
                if self._result is not None:
                    self._stats["result_replaced"] += 1
                self._result = result
                self._stats["compute_errors" if result.error else "computed"] += 1
                if result.error and any(s in result.error for s in ("expired", "clock_unavailable", "in_future")):
                    self._stats["expired_jobs"] += 1


class FusionCoordinator:
    """Main-thread-only admission/publication gates around the compute worker."""
    def __init__(self, *, binary_sha256, source_paths=(), compute=None, wall_clock=time.perf_counter):
        self.owner = threading.get_ident()
        self.clock = ReceivedClock()
        self.wall_clock = wall_clock
        self.binary_sha256 = binary_sha256
        self.source_versions = tuple((str(Path(p).resolve()), file_version(p)) for p in source_paths)
        self.source_hashes = {path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
                              for path, _ in self.source_versions}
        self.generation = 0
        self.sequence = 0
        self.last_published = None
        self.last_invalidated = False
        self._published_order = (-1, -1)
        self.counters = Counter()
        self.last_error = None
        self._pointer_authority = {}
        self.worker = LatestFusionWorker(compute or (lambda job: compute_fusion(job, self.clock, wall_clock)))

    def _owner(self):
        if threading.get_ident() != self.owner:
            raise RuntimeError("async_fusion_single_owner_violation")

    def on_clock(self, stamp, arrival):
        self._owner()
        generation = self.clock.update(stamp, arrival)
        if generation != self.generation:
            self.generation = generation
            self._published_order = (-1, -1)
            self._pointer_authority.clear()
            self.worker.generation(generation)

    def submit(self, **inputs):
        self._owner()
        self.sequence += 1
        try:
            config = dict(inputs.pop("config"), consumer_source_sha256=self.source_hashes)
            snapshot = FusionSnapshot.capture(generation=self.generation, sequence=self.sequence,
                                              binary_sha256=self.binary_sha256, config=config, **inputs)
            now = self.wall_clock()
            source_clock, _ = self.clock.read(now)
            snapshot.validate_ages(source_clock, now)
            snapshot.depth.verify_current(self._pointer_authority)
            self.counters["freeze_wall_ns"] += round(snapshot.freeze_wall_s * 1e9)
            return self.worker.submit(snapshot)
        except (ValueError, OSError, TypeError, KeyError) as error:
            self.counters["snapshot_rejected"] += 1
            if "expired" in str(error):
                self.counters["expired_snapshots"] += 1
            self.last_error = str(error)
            return False

    def validate(self, result, *, cached=False):
        self._owner()
        if self.worker.stats()["closed"]:
            raise ValueError("async_worker_closed")
        job = result.snapshot
        if result.error:
            raise ValueError(result.error)
        if job.generation != self.generation:
            raise ValueError("async_generation_changed")
        if not cached and (job.sequence <= self._published_order[0] or job.stamp_ns <= self._published_order[1]):
            raise ValueError("async_result_out_of_order")
        if job.binary_sha256 != self.binary_sha256:
            raise ValueError("async_binary_identity_changed")
        if any(file_version(path) != version for path, version in self.source_versions):
            raise ValueError("async_consumer_source_version_changed")
        job.depth.verify_current(self._pointer_authority)
        now = self.wall_clock()
        source_clock, generation = self.clock.read(now)
        if generation != job.generation:
            raise ValueError("async_generation_changed")
        job.validate_ages(source_clock, now)
        from .depth_safety_fusion import validate_fusion_contract
        # This opt-in is for joint safety fusion only. A rejected source never
        # enables another source or grants permission to publish native fallback.
        validate_fusion_contract(result.payload, now_sim_s=source_clock, now_wall_s=now)
        return True

    def poll(self):
        self._owner()
        result = self.worker.poll()
        if result is None:
            return None
        try:
            self.validate(result)
        except (ValueError, OSError, TypeError, KeyError) as error:
            self.counters["publication_rejected"] += 1
            self.last_error = str(error)
            return None
        return result

    def published(self, result):
        self._owner()
        self.last_published = result
        self.last_invalidated = False
        self._published_order = (result.snapshot.sequence, result.snapshot.stamp_ns)
        self.counters["published"] += 1

    def stats(self):
        self._owner()
        pointer = {k: v for k, v in self._pointer_authority.items() if k != "validated_key"}
        return dict(self.counters, worker=self.worker.stats(), generation=self.generation,
                    pointer_authority=pointer, last_error=self.last_error, mode="experimental_thread_not_parallelism_guarantee")

    def close(self, timeout=1.0):
        self._owner()
        return self.worker.close(timeout)
