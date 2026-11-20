"""Sensor-clock semantic admission; caller owns inference and timeout handling."""

from .core import _integer


class SemanticScheduler:
    def __init__(self, interval_ns=2_000_000_000, event_min_interval_ns=500_000_000):
        _integer(interval_ns, "interval_ns", 1)
        _integer(event_min_interval_ns, "event_min_interval_ns", 1)
        if event_min_interval_ns > interval_ns:
            raise ValueError("event interval may not exceed periodic interval")
        self.interval_ns = interval_ns
        self.event_min_interval_ns = event_min_interval_ns
        self.epoch = None
        self.pending = None
        self.last_requested_ns = self.last_seen_ns = None
        self.requested = self.skipped_busy = self.skipped_interval = 0

    def request(self, source_ns, *, epoch, event=False):
        _integer(source_ns, "source_ns")
        _integer(epoch, "epoch", 1)
        if type(event) is not bool:
            raise ValueError("event must be bool")
        if self.epoch is not None and epoch < self.epoch:
            raise ValueError("old sensor epoch")
        if self.epoch != epoch:
            self.epoch, self.pending = epoch, None
            self.last_requested_ns = self.last_seen_ns = None
        if self.last_seen_ns is not None and source_ns < self.last_seen_ns:
            raise ValueError("backward clock requires a new epoch")
        self.last_seen_ns = source_ns
        if self.pending is not None:
            self.skipped_busy += 1
            return False
        interval = self.event_min_interval_ns if event else self.interval_ns
        if (
            self.last_requested_ns is not None
            and source_ns - self.last_requested_ns < interval
        ):
            self.skipped_interval += 1
            return False
        self.pending = (epoch, source_ns)
        self.last_requested_ns = source_ns
        self.requested += 1
        return True

    def complete(self, source_ns, *, epoch):
        """Release only the admitted job; stale completion cannot clear a newer job."""
        if self.pending != (epoch, source_ns):
            return False
        self.pending = None
        return True
