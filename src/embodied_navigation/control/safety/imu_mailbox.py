"""Bounded IMU safety handoff: receiver samples, original control owner applies.

This mailbox never publishes, changes gyro/PID, or manufactures a receipt.
Each input retains its own validator/watermark. Only orientation-validated
samples enter it. Any tilt >= .18 interrupts recovery, so that unconsumed
maximum survives replacement of either port's latest state slot.
"""
from collections import Counter, deque
import math
import os
from threading import Lock

SWITCH = 'R103_IMU_SAFETY_INTAKE'
WINDOWS = 32


def enabled():
    value = os.environ.get(SWITCH, '0')
    if value not in ('0', '1'):
        raise ValueError(SWITCH + ' must be 0 or 1')
    return value == '1'


def scalar_fresh(stamp, receipt, timeout, now, ros_now):
    # The unchanged RawImuTilt.fresh inequalities, without calling a predicate
    # twice or confusing monotonic receipt time with ROS source time.
    return (math.isfinite(receipt) and 0 <= now-receipt <= timeout
            and -.10 <= ros_now-stamp <= timeout)


class SafetyIntake:
    def __init__(self, original_raw_imu=None, reading_view=None):
        self._lock = Lock()
        self.original_raw_imu = original_raw_imu
        self.reading_view = reading_view
        self.original_latest = None
        self.original_sequence = self.version = 0
        self.latest = self.peak = None
        self.last_consumed = 0
        self.counts = Counter()
        self.windows = deque(maxlen=WINDOWS)
        self.first_window = None

    def offer(self, sequence, stamp_ns, receipt_ns, callback_clock_ns, tilt,
              port='causal_fifo'):
        sample = (sequence, stamp_ns, receipt_ns, callback_clock_ns, tilt, port)
        with self._lock:
            # Duplicate source headers never acquire a later callback receipt.
            previous = self.latest if port == 'causal_fifo' else self.original_latest
            if previous is not None and stamp_ns <= previous[1]:
                self.counts['nonadvancing_offer'] += 1
                return
            if port == 'causal_fifo':
                self.latest = sample
            else:
                self.original_latest = sample
            self.version += 1
            self.counts['offered'] += 1
            self.counts['offered_' + port] += 1
            if tilt >= .18 and (self.peak is None or tilt > self.peak[4]):
                self.peak = sample

    def note_original_accepted(self, callback_clock_ns):
        # Called in the original on_imu accepted branch, before its guard.
        raw = self.original_raw_imu
        self.original_sequence += 1
        self.offer(self.original_sequence, round(raw.stamp*1e9),
            round(raw.updated*1e9), callback_clock_ns, raw.tilt, 'original_latest')

    @staticmethod
    def _port_observation(node, now, ros_now, original, original_fresh, merged_fresh):
        def ns(value):
            return None if not math.isfinite(value) else round(value*1e9)
        return dict(original_stamp_ns=ns(original[0]), original_receipt_ns=ns(original[1]),
            original_timeout_ns=round(original[2]*1e9), original_scalar_fresh=original_fresh,
            merged_stamp_ns=ns(node.raw_imu.stamp), merged_receipt_ns=ns(node.raw_imu.updated),
            merged_scalar_fresh=merged_fresh,
            old_port_stale_new_view_fresh=not original_fresh and merged_fresh,
            consume_wall_ns=round(now*1e9), consume_clock_ns=round(ros_now*1e9),
            r73_publication_sequence_before=getattr(node, 'publication_records', None))

    def consume(self, node, now, ros_now):
        # No intake callback invokes this method. All node writes remain in the
        # original mutually-exclusive control group.
        original_raw = self.original_raw_imu if self.original_raw_imu is not None else node.raw_imu
        original = (original_raw.stamp, original_raw.updated, original_raw.timeout)
        original_fresh = scalar_fresh(*original, now, ros_now)
        with self._lock:
            self.counts['guard_reads'] += 1
            self.counts['original_port_comparison_reads'] += 1
            samples = tuple(s for s in (self.latest, self.original_latest) if s is not None)
            peak = self.peak
            self.peak = None
            if not samples or self.version == self.last_consumed:
                self.counts['no_new_sample'] += 1
                merged_fresh = scalar_fresh(node.raw_imu.stamp, node.raw_imu.updated,
                    node.raw_imu.timeout, now, ros_now)
                gap = not original_fresh and merged_fresh
                self.counts['old_port_stale_new_view_fresh'] += int(gap)
                if gap:
                    # No sample was selected or retimed in this guard. The
                    # existing view's real prior receipt still remains fresh.
                    row = self._port_observation(node,now,ros_now,original,original_fresh,merged_fresh)
                    row.update(selected_this_guard=False, transferred=False,
                        guard_state_before=node.state, tilt_hold_before=node.tilt_hold)
                    self.windows.append(row)
                return None
            self.last_consumed = self.version
        before = (node.raw_imu.stamp, node.raw_imu.updated)
        before_fresh = scalar_fresh(*before, node.raw_imu.timeout, now, ros_now)
        if self.reading_view is not None:
            # Startup freshness keeps the original object until the first
            # actual guard read; no pre-running callback depends on a guard
            # being scheduled just to observe its original accepted IMU.
            node.raw_imu = self.reading_view
        raw = node.raw_imu
        fresh_samples = tuple(s for s in samples if scalar_fresh(
            s[1]/1e9, s[2]/1e9, raw.timeout, now, ros_now))
        sample = max(fresh_samples or samples, key=lambda s:(s[1], -s[2]))
        sequence, stamp_ns, receipt_ns, callback_clock_ns, tilt, port = sample
        stamp, receipt = stamp_ns/1e9, receipt_ns/1e9
        sample_fresh = scalar_fresh(stamp, receipt, raw.timeout, now, ros_now)
        transferred = sample_fresh and stamp > raw.stamp
        if transferred:
            raw.stamp, raw.updated, raw.tilt = stamp, receipt, tilt
            raw.max_tilt = max(raw.max_tilt, tilt)
            raw.last_rejected = None
        if peak is not None:
            raw.max_tilt = max(raw.max_tilt, peak[4])
        after_fresh = scalar_fresh(raw.stamp, raw.updated, raw.timeout, now, ros_now)
        old_port_gap = not original_fresh and after_fresh
        self.counts['old_port_stale_new_view_fresh'] += int(old_port_gap)
        self.counts['consumed'] += 1
        self.counts['transferred'] += int(transferred)
        self.counts['expired_or_future_receipt'] += int(not sample_fresh)
        self.counts['stale_to_fresh'] += int(not before_fresh and after_fresh)
        self.counts['peak_delivered'] += int(peak is not None)
        self.counts['severe_delivered'] += int(peak is not None and peak[4] >= .50)
        # Allocate an observation only for the first or a relevant transition.
        if self.first_window is None or not before_fresh or peak is not None or old_port_gap:
            row = dict(sequence=sequence, source_stamp_ns=stamp_ns,
                selected_port=port, selected_this_guard=True,
                callback_wall_ns=receipt_ns, callback_clock_ns=callback_clock_ns,
                consume_wall_ns=round(now*1e9), consume_clock_ns=round(ros_now*1e9),
                before_stamp_ns=None if not math.isfinite(before[0]) else round(before[0]*1e9),
                before_receipt_ns=None if not math.isfinite(before[1]) else round(before[1]*1e9),
                before_raw_scalar_fresh=before_fresh, sample_scalar_fresh=sample_fresh,
                after_raw_scalar_fresh=after_fresh, transferred=transferred,
                pending_peak=None if peak is None else list(peak),
                publication_count_before=node.counts.get('commands', 0),
                r73_publication_sequence_before=getattr(node, 'publication_records', None),
                guard_state_before=node.state, tilt_hold_before=node.tilt_hold)
            row.update(self._port_observation(node,now,ros_now,original,original_fresh,after_fresh))
            if self.first_window is None:
                self.first_window = row
            self.windows.append(row)
        return peak

    def receipt(self):
        with self._lock:
            return dict(schema='r103_imu_safety_intake/v1', enabled=True,
                retained_sample_slots=3, maximum_windows=WINDOWS,
                counts=dict(self.counts), first_window=self.first_window, windows=list(self.windows),
                latest=None if self.latest is None else list(self.latest),
                original_latest=None if self.original_latest is None else list(self.original_latest),
                pending_peak=None if self.peak is None else list(self.peak),
                last_consumed_mailbox_version=self.last_consumed,
                before_shadow='scalar raw state immediately before handoff, not a rerun of the OFF controller',
                original_comparison_scope='every guard consume entry at its fixed caller clocks; original validator scalars, not a rerun of the OFF controller',
                original_latest_rejection_observation='controller.on_imu note_latest reads intake.original_raw_imu.last_rejected when enabled',
                outcome_join='r73_publications sequence after r73_publication_sequence_before and consume_wall_ns; guard entry alone does not claim a publication',
                source_and_receipt_times_preserved=True,
                original_latest_validator_and_watermark_independent=True,
                no_direct_gyro_PID_or_pair_history_writes=True,
                original_gyro_and_history_acceptance_uses_original_latest_validator=True)
