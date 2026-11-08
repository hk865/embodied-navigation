"""Same-source causal IMU FIFO, with optional owner-consumed safety mailbox."""
import copy
import time
from collections import Counter, deque
from threading import Lock

import numpy as np

FIFO_DEPTH = 64  # 200 Hz x original .25 s freshness = 50 messages; finite headroom.
HISTORY_DEPTH = 128
PAIR_NS = 20_000_000
EVENT_CAP = 64


def enabled(profile):
    config = profile.get('r85_imu_source_buffer', {})
    return isinstance(config, dict) and config.get('enabled') is True


class CausalImuBuffer:
    def __init__(self, node, validator_type):
        self.node = node
        self._lock = Lock()
        # Private receiver validator: never writes node.raw_imu, gyro, pose or
        # tilt. The opt-in mailbox is consumed only by the control owner.
        self.validator = validator_type(node.imu_body_rotation,
            node.imu_reference_rotation, timeout=node.raw_imu.timeout)
        self.history = deque(maxlen=HISTORY_DEPTH)
        self.recent = deque(maxlen=16)
        self.counts = Counter()
        self.events = set()
        self.last_event = None
        from embodied_navigation.control.safety.imu_mailbox import enabled as safety_enabled, SafetyIntake
        self.safety_intake = None
        if safety_enabled():
            # Preserve the original latest port's actual validator object.
            # Only guard consumption updates the separate safety reading view.
            reading_view = validator_type(node.imu_body_rotation,
                node.imu_reference_rotation, timeout=node.raw_imu.timeout)
            self.safety_intake = SafetyIntake(node.raw_imu, reading_view)

    def callback(self, msg, info):
        wall = time.monotonic_ns()
        clock = self.node.get_clock().now().nanoseconds
        with self._lock:
            stamp = int(self.node.stamp(msg))
            q, v = msg.orientation, msg.angular_velocity
            self.counts['fifo_callbacks'] += 1
            accepted = self.validator.update([q.x, q.y, q.z, q.w], stamp / 1e9,
                wall / 1e9, clock / 1e9, msg.orientation_covariance[0] >= 0)
            # Tilt safety accepts orientation independently of finite gyro.
            # The independent receiver only offers scalars; it never mutates
            # the original control owner's raw_imu, tilt hold or publications.
            if accepted and self.safety_intake is not None:
                self.safety_intake.offer(self.counts['fifo_callbacks'], stamp, wall,
                    clock, self.validator.tilt)
            gyro = self.node.imu_body_rotation @ np.array([v.x, v.y, v.z])
            accepted = bool(accepted and np.isfinite(gyro).all())
            reason = None if accepted else self.validator.last_rejected or 'nonfinite_gyro'
            meta = {}
            for key in ('source_timestamp', 'received_timestamp',
                        'publication_sequence_number', 'reception_sequence_number'):
                value = info.get(key) if isinstance(info, dict) else getattr(info, key, None)
                if type(value) is int:
                    meta['rmw_' + key] = value
            summary = dict(port='causal_fifo', stamp_ns=stamp, callback_wall_ns=wall,
                callback_ros_clock_ns=clock, accepted=accepted, reason=reason, **meta)
            self.recent.append(summary)
            self.counts['fifo_accepted' if accepted else 'fifo_rejected'] += 1
            if not accepted:
                return
            # Capture this actual callback receipt ONCE. Pair/consume never retimes it.
            # RMW timestamps retain their own clock domain; no fabricated conversion.
            row = dict(stamp_ns=stamp, received_wall_ns=wall, frame_id='body',
                angular_velocity_body=gyro.tolist(), source_frame_id=msg.header.frame_id,
                gyro_source_xyz=[v.x, v.y, v.z], orientation_xyzw=[q.x, q.y, q.z, q.w],
                orientation_covariance=list(msg.orientation_covariance),
                imu_to_body_rotation=self.node.imu_body_rotation.tolist(),
                callback_ros_clock_ns=clock, source_topic='/livox/imu',
                navigation_ground_truth_used=False, source_port='same_topic_causal_fifo',
                receipt_semantics='actual FIFO callback entry, not publication/RMW receipt',
                fifo_callback_sequence=self.counts['fifo_callbacks'], **meta)
            if stamp > clock:
                row['r85_original_future_header_history_only'] = True
            self.history.append(row)

    def note_latest(self, stamp, wall, clock, recorded, rejection):
        with self._lock:
            self.counts['latest_callbacks'] += 1
            self.counts['latest_history_recorded'] += int(recorded)
            self.recent.append(dict(port='original_latest', stamp_ns=stamp,
                callback_wall_ns=wall, callback_ros_clock_ns=clock,
                accepted_history_recorded=recorded, raw_rejection=rejection))

    def pair(self, pose_stamp, original, clock, *, snapshot=False):
        if snapshot:
            from embodied_navigation.control.parking.causal_join import scalar_snapshot
        if pose_stamp > clock:
            return None
        if original is not None:
            return scalar_snapshot(original) if snapshot else original  # Original port retains priority.
        with self._lock:
            options = [r for r in self.history if 0 <= pose_stamp-r['stamp_ns'] <= PAIR_NS]
            if not options:
                return None
            newest = max(r['stamp_ns'] for r in options)
            # A duplicate source header cannot acquire a fresher receipt on consume.
            selected = min((r for r in options if r['stamp_ns'] == newest),
                           key=lambda r: r['received_wall_ns'])
            return scalar_snapshot(selected) if snapshot else selected

    def event(self, pose_stamp, clock, wall, phase, pair):
        with self._lock:
            key = (pose_stamp, phase)
            if key in self.events:
                return
            row = dict(
                schema='same_source_causal_imu_timing/v1', phase=phase,
                pose_stamp_ns=pose_stamp, control_clock_ns=clock, observed_wall_ns=wall,
                original_pair_max_ns=PAIR_NS, source_topic='/livox/imu',
                paired_stamp_ns=None if pair is None else pair['stamp_ns'],
                paired_received_wall_ns=None if pair is None else pair['received_wall_ns'],
                paired_source_port=None if pair is None else pair.get('source_port', 'original_latest'),
                recent_callbacks=copy.deepcopy(list(self.recent)), counters=dict(self.counts),
                navigation_ground_truth_used=False)
            self.last_event = row  # Retain the final miss/recovery even after the append cap.
            if len(self.events) >= EVENT_CAP:
                self.counts['events_omitted_by_cap'] += 1
                return
            self.events.add(key)
        self.node.evidence.append(self.node.run/'imu_source_timing_events.jsonl', row)

    def finish(self):
        with self._lock:
            receipt = dict(
                schema='same_source_causal_imu_timing_receipt/v1', fifo_depth=FIFO_DEPTH,
                history_depth=HISTORY_DEPTH, event_cap=EVENT_CAP,
                recorded_events=len(self.events), counters=dict(self.counts),
                last_pairing_event=copy.deepcopy(self.last_event),
                original_latest_tilt_subscription_preserved=True,
                pair_max_ns=PAIR_NS, original_freshness_and_timestamp_checks_unchanged=True,
                navigation_ground_truth_used=False)
        self.node.evidence.atomic(self.node.run/'imu_source_timing_receipt.json', receipt)
        if self.safety_intake is not None:
            self.node.evidence.atomic(self.node.run/'r103_imu_safety_intake.json',
                self.safety_intake.receipt())
