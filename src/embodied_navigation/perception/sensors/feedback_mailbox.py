"""Bounded immutable corrected-message handoff; no control state or hot-path I/O.

One intake callback group owns joining/verification. The control group owns
deserialization and every original business callback. Locks protect references
and counters only; verification, serialization and dispatch run outside them.
"""

from collections import OrderedDict, deque
from dataclasses import dataclass
import json
import copy
import hashlib
import math
import threading
import time
from embodied_navigation.common.observation import api as _observer_api

diag = _observer_api("r87_diagnostic_probe")
from embodied_navigation.common.observation import api as _observer_api

evidence = _observer_api("r118_feedback_evidence")

KEY = "r86_prepared_feedback"
SCHEMA = "r86_prepared_corrected_feedback/v1"
EXPIRED_KEY = "r86_expired_prefix"
EXPIRED_SCHEMA = "r86_expired_prefix/v1"
SLOTS, FRAMES, BYTES, PART_BYTES = 24, 16, 32 * 1024 * 1024, 4 * 1024 * 1024
ROLES = ("body", "cloud", "metadata")


def _scope_text(scope):
    """Only a bounded identity, never a message or point-cloud representation."""
    if isinstance(scope, bytes):
        return scope[:32].hex()
    if scope is None:
        return None
    return "<" + type(scope).__name__[:32] + ">"


def _bundle_mark(stage, bundle, **fields):
    """Diagnostic stage mark; scopes are reduced to bounded identities."""
    if diag.enabled():
        diag.mark(
            stage,
            bundle.source_ns,
            prepared_ns=bundle.prepared_ns,
            scope_body=_scope_text(bundle.scopes[0]),
            scope_cloud=_scope_text(bundle.scopes[1]),
            scope_metadata=_scope_text(bundle.scopes[2]),
            **fields,
        )


def enabled(profile):
    """Return the typed feature switch for this mailbox.

    A missing key or None means disabled; any other value must be a dict with a boolean
    ``enabled`` (anything else raises), so callers cannot half-enable the path.
    """
    if not isinstance(profile, dict):
        raise ValueError("Controller settings must be an object")
    option = profile.get(KEY)
    if option is None:
        return False
    if not isinstance(option, dict) or type(option.get("enabled")) is not bool:
        raise ValueError("Controller feature enabled flag must be boolean")
    return option["enabled"]


def expired_prefix_enabled(profile):
    """Exact typed S1 selection; absent or false keeps the original fatal path."""
    if not isinstance(profile, dict) or EXPIRED_KEY not in profile:
        return False
    option = profile[EXPIRED_KEY]
    if (
        type(option) is not dict
        or set(option) != {"enabled", "schema"}
        or type(option["enabled"]) is not bool
        or option["schema"] != EXPIRED_SCHEMA
    ):
        raise ValueError("Exact typed R86 expired-prefix option required")
    return option["enabled"]


def expired_prefix_timeout_ns(pose_timeout_s):
    """The same 300 ms freshness expression the original body gate uses."""
    if (
        type(pose_timeout_s) not in (int, float)
        or not math.isfinite(pose_timeout_s)
        or not pose_timeout_s > 0
    ):
        raise ValueError("Exact finite positive pose timeout required")
    return float(pose_timeout_s) * 1e9


def expired_prefix_length(batch, decision_clock_ns, timeout_ns):
    """Number of expired leading members of one fixed FIFO batch.

    The predicate is exactly 'decision_clock_ns - source_ns >= timeout_ns' as
    the original Teacher body gate evaluates it. It is monotone in source_ns,
    so the expired set is always a head prefix; scanning stops at the first
    member that is not expired. A future member (source_ns > clock) is never
    expired and therefore never retired, and the scan never looks past it.
    """
    if type(decision_clock_ns) is not int:
        raise ValueError("Exact integer decision clock required")
    if type(batch) is not tuple:
        raise ValueError("Fixed immutable batch tuple required")
    count = 0
    for bundle in batch:
        if type(bundle.source_ns) is not int:
            raise ValueError("Exact integer source stamp required")
        if decision_clock_ns - bundle.source_ns >= timeout_ns:
            count += 1
        else:
            break
    return count


@dataclass(frozen=True)
class Bundle:
    source_ns: int
    token: tuple
    generation: int
    body: bytes
    cloud: bytes
    metadata: bytes
    # Receipt is actual intake entry, never preparation or delayed commit time.
    arrivals_ns: tuple
    prepared_ns: int
    scopes: tuple

    @property
    def size(self):
        return len(self.body) + len(self.cloud) + len(self.metadata)


class Mailbox:
    def __init__(self, encode, decode, verify):
        self.evidence_probe = evidence.create()
        self.evidence_call_errors = 0
        self.evidence_exit_status = None
        self.encode, self.decode, self.verify = encode, decode, verify
        self.lock = threading.Lock()
        self.pending = OrderedDict()
        self.ready = deque()
        self.bytes = 0
        self.last_source = -1
        self.last_token = None
        self.fault = None
        self.scope = None
        self.inflight = {}
        self.preparing_bytes = 0
        self.counts = dict(
            arrivals=0,
            prepared=0,
            rejected=0,
            duplicates=0,
            committed=0,
            pending_evicted=0,
            retired_expired=0,
            retired_bytes=0,
            aborted_unprocessed=0,
        )
        # Source stamps already decided by this consumer; released exactly once
        # and used only for the accounting identity, never for dispatch.
        self._committed_sources = set()
        self._retired_sources = set()
        self.highwater_bytes = self.highwater_frames = 0
        self.events = deque(maxlen=256)
        self.diagnostic_batch_id = 0
        self.ready_notify = None

    def _evidence(self, stage, source_ns=-1, **fields):
        if self.evidence_probe is not None:
            try:
                self.evidence_probe.mark(stage, source_ns, **fields)
            except Exception:
                # The diagnostic endpoint cannot replace business exceptions.
                self.evidence_call_errors += 1

    def fail(self, reason):
        self._evidence(10)
        with diag.lock(self.lock, "mailbox.fail"):
            self.fault = self.fault or str(reason)[:240]
            diag.anomaly(self.fault)

    def fault_reason(self):
        with diag.lock(self.lock, "mailbox.fault_reason"):
            return self.fault

    def set_scope(self, scope):
        with diag.lock(self.lock, "mailbox.set_scope"):
            self.scope = scope

    def receive(self, role, source_ns, value, arrival_ns):
        """Freeze one ROS-owned part and queue it under its source stamp.

        The message is byte-encoded inside this call (never shared live), bounded
        before/after encoding, and a duplicate source stamp is never refreshed;
        a differing duplicate latches ``conflicting_duplicate_part``.
        """
        if role not in ROLES or type(source_ns) is not int or type(arrival_ns) is not int:
            self.fail("invalid_intake_identity")
            return
        self._evidence(1, source_ns, role=ROLES.index(role), arrival_ns=arrival_ns)
        with diag.lock(self.lock, "mailbox.receive_scope", source_ns):
            if self.fault:
                return
            arrival_scope = self.scope
        # Freeze the ROS-owned message before sharing it with another group.
        try:
            if role == "cloud" and (
                len(value.data) > PART_BYTES
                or len(value.fields) > 64
                or len(value.header.frame_id) > 256
                or any(len(f.name) > 64 for f in value.fields)
            ):
                raise ValueError("message_input_bound")
            if role == "body" and (
                len(value.header.frame_id) > 256 or len(value.child_frame_id) > 256
            ):
                raise ValueError("body_input_bound")
            with diag.span("mailbox.encode", source_ns, role=role):
                wire = self.encode(role, value)
            if type(wire) is not bytes or len(wire) > PART_BYTES:
                raise ValueError("message_byte_bound")
        except Exception as error:
            self.fail("freeze:" + type(error).__name__)
            return
        with diag.lock(self.lock, "mailbox.pending", source_ns):
            if self.fault:
                return
            self.counts["arrivals"] += 1
            if source_ns <= self.last_source:
                self.counts["duplicates"] += 1
                return
            slot = self.pending.get(source_ns, {})
            previous = slot.get(role)
            if previous is not None:
                # Never refresh a duplicate's receipt or replace its identity.
                self.counts["duplicates"] += 1
                if previous[0] != wire:
                    self.fault = "conflicting_duplicate_part"
                    diag.anomaly(self.fault, source_ns)
                return
            if self.bytes + len(wire) > BYTES:
                self.fault = "mailbox_byte_overflow"
                diag.anomaly(self.fault, source_ns)
                return
            if source_ns not in self.pending:
                while len(self.pending) >= SLOTS:
                    _, expired = self.pending.popitem(last=False)
                    self.bytes -= sum(len(part[0]) for part in expired.values())
                    self.counts["pending_evicted"] += 1
                self.pending[source_ns] = slot
            slot[role] = (wire, arrival_ns, arrival_scope)
            self.bytes += len(wire)
            self.highwater_bytes = max(self.highwater_bytes, self.bytes)
            while len(self.pending) > SLOTS:
                _, expired = self.pending.popitem(last=False)
                self.bytes -= sum(len(part[0]) for part in expired.values())
                self.counts["pending_evicted"] += 1
            if not all(k in slot for k in ROLES):
                return
            self._evidence(2, source_ns, retained_bytes=self.bytes)
            del self.pending[source_ns]
            self.preparing_bytes = sum(len(part[0]) for part in slot.values())
            diag.mark(
                "mailbox.parts_joined",
                source_ns,
                body_bytes=len(slot["body"][0]),
                cloud_bytes=len(slot["cloud"][0]),
                metadata_bytes=len(slot["metadata"][0]),
            )
        # No lock covers hash calculation, CDR conversion, or JSON decoding.
        try:

            def decode_part(role):
                with diag.span("mailbox.decode", source_ns, role=role):
                    return self.decode(role, slot[role][0])

            body, cloud, metadata = (decode_part(k) for k in ROLES)
            with diag.span("mailbox.verify_hash_total", source_ns):
                token = self.verify(body, cloud, metadata)
            if type(metadata["generation"]) is not int:
                raise ValueError("invalid_generation")
            bundle = Bundle(
                source_ns,
                token,
                metadata["generation"],
                *(slot[k][0] for k in ROLES),
                tuple(slot[k][1] for k in ROLES),
                time.monotonic_ns(),
                tuple(slot[k][2] for k in ROLES),
            )
            _bundle_mark(
                "mailbox.bundle_constructed",
                bundle,
                generation=bundle.generation,
                arrival_body_ns=bundle.arrivals_ns[0],
                arrival_cloud_ns=bundle.arrivals_ns[1],
                arrival_metadata_ns=bundle.arrivals_ns[2],
            )
        except (ValueError, KeyError, TypeError) as error:
            with diag.lock(self.lock, "mailbox.verify_rejected", source_ns):
                self.bytes -= self.preparing_bytes
                self.preparing_bytes = 0
                self.counts["rejected"] += 1
                self.events.append(
                    dict(source_ns=source_ns, event="verify_rejected", error=type(error).__name__)
                )
                diag.mark("mailbox.verify_rejected", source_ns, error=type(error).__name__)
            return
        except Exception as error:
            with diag.lock(self.lock, "mailbox.prepare_error", source_ns):
                self.bytes -= self.preparing_bytes
                self.preparing_bytes = 0
            self.fail("prepare:" + type(error).__name__)
            return
        with diag.lock(self.lock, "mailbox.ready_append", source_ns):
            self.bytes -= self.preparing_bytes
            self.preparing_bytes = 0
            if self.fault:
                return
            if self.last_token is not None and (
                token[0] != self.last_token[0] or token[1] <= self.last_token[1]
            ):
                self.counts["rejected"] += 1
                return
            if len(self.ready) + len(self.inflight) >= FRAMES or self.bytes + bundle.size > BYTES:
                self.fault = "complete_bundle_overflow"
                diag.anomaly(self.fault, source_ns)
                return
            diag.mark("mailbox.ready_append_before", source_ns, ready_count=len(self.ready))
            self.ready.append(bundle)
            self._evidence(3, bundle=bundle, ready_count=len(self.ready), retained_bytes=self.bytes)
            diag.mark("mailbox.ready_append_after", source_ns, ready_count=len(self.ready))
            self.bytes += bundle.size
            self.last_source, self.last_token = source_ns, token
            self.counts["prepared"] += 1
            self.highwater_bytes = max(self.highwater_bytes, self.bytes)
            self.highwater_frames = max(self.highwater_frames, len(self.ready))
            self.events.append(
                dict(
                    source_ns=source_ns,
                    event="prepared",
                    arrivals_ns=bundle.arrivals_ns,
                    prepared_ns=bundle.prepared_ns,
                )
            )
            for ns in list(self.pending):
                if ns <= source_ns:
                    expired = self.pending.pop(ns)
                    self.bytes -= sum(len(part[0]) for part in expired.values())
                    self.counts["pending_evicted"] += 1
        # No control work or guard trigger runs while holding the mailbox lock.
        if self.ready_notify is not None:
            self.ready_notify()

    def peek_ready(self):
        """Immutable head reference; the sole control owner is the consumer."""
        with diag.lock(self.lock, "mailbox.peek_ready"):
            return self.ready[0] if self.ready else None

    def peek_ready_prefix(self):
        with diag.lock(self.lock, "mailbox.peek_ready_prefix"):
            if len(self.ready) > FRAMES:
                raise ValueError("r104_ready_bound")
            return tuple(self.ready)

    def retained_bytes(self):
        with diag.lock(self.lock, "mailbox.retained_bytes"):
            return self.bytes

    def take_ready_prefix(self, expected):
        with diag.lock(self.lock, "mailbox.take_ready_prefix"):
            if len(expected) > FRAMES or len(self.ready) < len(expected):
                raise ValueError("r104_ready_prefix_bound")
            if any(self.ready[i] is not frame for i, frame in enumerate(expected)):
                raise ValueError("r104_ready_prefix_identity_changed")
            self.diagnostic_batch_id += 1
            for frame in expected:
                self.ready.popleft()
                self.inflight[frame.source_ns] = frame.size
                self._evidence(4, bundle=frame, batch_id=self.diagnostic_batch_id)
            return expected

    def take_batch(self, max_source_ns=None):
        if max_source_ns is not None and type(max_source_ns) is not int:
            raise ValueError("Exact integer clock threshold required")
        with diag.lock(self.lock, "mailbox.take_batch"):
            self.diagnostic_batch_id += 1
            batch_id = self.diagnostic_batch_id
            diag.mark("mailbox.take_batch_before", batch_id=batch_id, ready_count=len(self.ready))
            if max_source_ns is None:
                batch = tuple(self.ready)
                self.ready.clear()
            else:
                # One atomic FIFO prefix. The future suffix retains its original
                # bytes/receipts/scope and remains charged to the same bounds.
                prefix = []
                while self.ready and self.ready[0].source_ns <= max_source_ns:
                    prefix.append(self.ready.popleft())
                batch = tuple(prefix)
            self.inflight.update((frame.source_ns, frame.size) for frame in batch)
            diag.mark("mailbox.take_batch_after", batch_id=batch_id, batch_count=len(batch))
        if self.evidence_probe is not None:
            for bundle in batch:
                self._evidence(4, bundle=bundle, batch_id=batch_id)
        if diag.enabled():
            for index, bundle in enumerate(batch):
                if index >= FRAMES:
                    break
                _bundle_mark("mailbox.batch_member", bundle, batch_id=batch_id, index=index)
        return batch

    def retire_expired(self, batch, count, decision_clock_ns, timeout_ns):
        """Retire only the expired FIFO head; bytes stay charged once.

        No token, pose, receipt, scope or clock advances here. The caller skips
        exactly these members and still calls release_batch(batch) once, so each
        retired payload is released exactly once. Retired members remain in
        inflight until that release, so accounting stays closed at both points.
        """
        if type(count) is not int or count < 0 or count > len(batch):
            raise ValueError("Exact retired prefix length required")
        retired = []
        with diag.lock(self.lock, "mailbox.retire_expired"):
            for frame in batch[:count]:
                if decision_clock_ns - frame.source_ns < timeout_ns:
                    raise ValueError("retired prefix contains a non-expired member")
                self._evidence(8, bundle=frame, batch_id=self.diagnostic_batch_id)
                self.counts["retired_expired"] += 1
                self.counts["retired_bytes"] += frame.size
                self._retired_sources.add(frame.source_ns)
                self.events.append(
                    dict(
                        source_ns=frame.source_ns,
                        event="retired_expired",
                        decision_clock_ns=decision_clock_ns,
                        timeout_ns=timeout_ns,
                        arrivals_ns=frame.arrivals_ns,
                        prepared_ns=frame.prepared_ns,
                        token=frame.token,
                    )
                )
                diag.mark(
                    "mailbox.retired_expired",
                    frame.source_ns,
                    decision_clock_ns=decision_clock_ns,
                    timeout_ns=timeout_ns,
                )
                retired.append(frame)
        return tuple(retired)

    def accounting(self):
        """prepared = committed + retired_expired + aborted + ready + pending.

        A frame that is still in flight is only counted as pending when this
        consumer has not already decided it (committed/retired). release_batch
        moves every remaining undecided frame to aborted_unprocessed, so the
        identity closes both before and after the single release.
        """
        with diag.lock(self.lock, "mailbox.accounting"):
            committed = self.counts["committed"]
            retired = self.counts["retired_expired"]
            prepared = self.counts["prepared"]
            ready = len(self.ready)
            pending = sum(
                1
                for source_ns in self.inflight
                if source_ns not in self._committed_sources
                and source_ns not in self._retired_sources
            )
            aborted = self.counts["aborted_unprocessed"]
            target = committed + retired + aborted + ready + pending
            return dict(
                prepared=prepared,
                committed=committed,
                retired_expired=retired,
                aborted_unprocessed=aborted,
                ready=ready,
                pending=pending,
                inflight=len(self.inflight),
                rejected=self.counts["rejected"],
                retired_bytes=self.counts["retired_bytes"],
                target_total=target,
                closes=prepared == target,
            )

    def release_batch(self, batch):
        with diag.lock(self.lock, "mailbox.release_batch"):
            for frame in batch:
                self._evidence(9, bundle=frame, batch_id=self.diagnostic_batch_id)
                self.bytes -= self.inflight.pop(frame.source_ns)
                if (
                    frame.source_ns not in self._committed_sources
                    and frame.source_ns not in self._retired_sources
                ):
                    self.counts["aborted_unprocessed"] += 1
                self._committed_sources.discard(frame.source_ns)
                self._retired_sources.discard(frame.source_ns)

    def committed(self, bundle, commit_ns):
        self._evidence(7, bundle=bundle, batch_id=self.diagnostic_batch_id)
        with diag.lock(self.lock, "mailbox.committed", bundle.source_ns):
            self.counts["committed"] += 1
            self._committed_sources.add(bundle.source_ns)
            self.events.append(
                dict(
                    source_ns=bundle.source_ns,
                    event="committed",
                    prepared_ns=bundle.prepared_ns,
                    commit_ns=commit_ns,
                    arrivals_ns=bundle.arrivals_ns,
                )
            )

    def snapshot(self):
        with diag.lock(self.lock, "mailbox.snapshot"):
            return dict(
                schema=SCHEMA,
                fault=self.fault,
                counts=dict(self.counts),
                r118_observer_call_errors=self.evidence_call_errors,
                r118_evidence_exit_status=self.evidence_exit_status,
                pending=len(self.pending),
                ready=len(self.ready),
                retained_bytes=self.bytes,
                inflight_frames=len(self.inflight),
                inflight_bytes=sum(self.inflight.values()),
                preparing_bytes=self.preparing_bytes,
                highwater_bytes=self.highwater_bytes,
                highwater_frames=self.highwater_frames,
                bound_frames=FRAMES,
                bound_bytes=BYTES,
                events=list(self.events),
                source_receipts_refreshed=False,
                physics_or_navigation_PASS=False,
            )


def ros_mailbox(verify):
    from rclpy.serialization import serialize_message, deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2

    kinds = dict(body=Odometry, cloud=PointCloud2)

    def encode(role, value):
        if role == "metadata":
            return json.dumps(value, allow_nan=False, separators=(",", ":")).encode()
        return serialize_message(value)

    def decode(role, value):
        return json.loads(value) if role == "metadata" else deserialize_message(value, kinds[role])

    return Mailbox(encode, decode, verify)


def receipt_ns(node, role, fallback):
    """Freshness source only; do not replace heading/ACK consumption clocks."""
    bundle = getattr(node, "_r86_committing_bundle", None)
    return fallback if bundle is None else bundle.arrivals_ns[ROLES.index(role)]


def control_scope(node):
    """Called only by the control group; intake sees this immutable digest."""
    gate = getattr(node, "heading_gate", None)
    data = dict(
        request=getattr(node, "request_id", None),
        waypoint=getattr(node, "waypoint_index", None),
        transition=getattr(gate, "transition_context", None),
        purpose=getattr(gate, "transition_purpose", None),
        handoff=getattr(node, "handoff_scope_sha256", None),
        active=getattr(gate, "handoff_active", False),
        braking=getattr(gate, "controlled_braking", False),
        zero=not any(getattr(node, "command", [])),
    )
    return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).digest()


def check_negative_parking(node):
    """Retain every queued bad sample; never grant dwell, T0 or a math update.

    Reuse the unchanged ordinary-parking predicate on a copy. Only invalidation
    propagates. The original control call remains the sole positive transition
    owner, and the independent fixed-five-second verifier is unchanged.
    """
    core = getattr(node, "cascade", None)
    ordinary = getattr(core, "ordinary_parking", None)
    if ordinary is None or not ordinary.enabled:
        return
    from embodied_navigation.control.tracking.cascade_core import rotation, vector, wrap
    import numpy as np

    feedback, imu = node.feedback, node.paired_imu
    stamp = feedback["stamp_ns"]
    if stamp < ordinary.start_clock:
        return
    if imu is None or not 0 <= stamp - imu["stamp_ns"] <= 20_000_000:
        raise ValueError("queued_parking_pose_without_causal_IMU")
    p = vector(feedback["position_world_xyz"], 3)
    v = vector(feedback["origin_velocity_body"], 3)
    gyro = vector(imu["angular_velocity_body"], 3)
    R = rotation(feedback["quaternion_wxyz"])
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.asin(float(np.clip(-R[2, 0], -1, 1)))
    yaw = math.atan2(R[1, 0], R[0, 0])
    if max(abs(roll), abs(pitch)) > 0.65 or abs(math.cos(pitch)) < 0.4 or abs(math.cos(roll)) < 0.4:
        raise ValueError("queued_parking_unsafe_attitude")
    yawdot = (math.sin(roll) * gyro[1] + math.cos(roll) * gyro[2]) / math.cos(pitch)
    clock = node.get_clock().now().nanoseconds
    if ordinary.pose10hz:
        # The accepted commit stream is also replayed by the final audit.
        # This intake owns samples only; control owns handoff, ACK and T0.
        from embodied_navigation.control.parking.parking_pose10hz import from_feedback

        ordinary.observe_pose10hz_source(from_feedback(feedback))
        grid = ordinary.pose_grid
        failure = (
            "parking_pose10hz_data_unavailable"
            if grid is not None and grid.missing
            else "parking_pose10hz_fixed_anchor_exceeded"
            if grid is not None and grid.motion_failure
            else None
        )
        if failure:
            ordinary.failure, ordinary.stage = failure, "failed"
            core._protect(failure, fatal=True)
            # Preserve this successful source commit in its original journal.
            # A sampled motion/data failure is not an R86 source-integrity fault.
        return True
    diagnostic = getattr(core, "diagnostic_observe_only", None)
    distance = float(np.linalg.norm(core.goal[:2] - p[:2]))
    yaw_error = wrap(core.goal_yaw - yaw)
    if (
        distance > 0.3
        or abs(float(core.goal[2] - p[2])) > core.goal_z_tolerance
        or abs(yaw_error) > 0.6
    ):
        observed = ordinary.observe_business_rejection(
            "parking_fixed_target_lost",
            stamp,
            clock,
            dict(
                goal_distance_xy_m=distance,
                goal_z_error_m=float(core.goal[2] - p[2]),
                goal_yaw_error_rad=yaw_error,
                original_goal_z_tolerance_m=core.goal_z_tolerance,
                queued_pose_checked=True,
                positive_qualification_granted=False,
            ),
            diagnostic,
            terminal_zero_event=core.ordinary_zero_event,
        )
        if not observed:
            core._protect("parking_fixed_target_lost")
            ordinary.guard("parking_fixed_target_lost")
            return
    probe = copy.deepcopy(ordinary)
    if ordinary.diagnostic_failure_is_observed(diagnostic):
        probe.failure = None
    probe.update(stamp, clock, p, yaw, v, yawdot, gyro[2], distance, yaw_error)
    if ordinary.ready_stamp is None and probe.dwell is None:
        ordinary.dwell = None
    if probe.failure and probe.failure != ordinary.failure:
        observed = ordinary.observe_business_rejection(
            probe.failure,
            stamp,
            clock,
            dict(
                queued_pose_checked=True,
                feedback=copy.deepcopy(feedback),
                positive_qualification_granted=False,
            ),
            diagnostic,
        )
        if not observed:
            ordinary.failure, ordinary.stage = probe.failure, "failed"
            core._protect(probe.failure, fatal=True)
            raise ValueError("queued_parking:" + probe.failure)
