"""Default-off positive-yield experiment on the sole main spinner.

This changes dispatch timing, not callback ownership or control computations.
One successful public spin_once is followed by a requested 100 us sleep; neither
the sleep nor the resulting dispatch delay has a realtime upper bound.
"""
from collections import deque
import json
import os
import threading
import time

import embodied_navigation.common.time.executor_selection as original

KEY = 'R90_SPINNER_POSITIVE_YIELD'
YIELD_SECONDS = .0001
SLOW_NS = 50_000_000
SLOW_CAPACITY = 64


def selected():
    value = os.environ.get(KEY, '0')
    if value not in ('0', '1'):
        raise ValueError(KEY + ' must be 0 or 1')
    return value == '1'


def make_executor(base, profile, num_threads=2):
    if not selected():
        return original.make_executor(base, profile, num_threads=num_threads)
    if not original.enabled(profile):
        raise ValueError('R90 requires the selected sole-owner R74 executor')

    class PositiveYieldExecutor(base):
        _r90_positive_yield_seconds = YIELD_SECONDS

        def __init__(self, *args, **kwargs):
            self._r74_cooperative_stop = threading.Event()
            self._r90_slow = deque(maxlen=SLOW_CAPACITY)
            self._r90_stats = dict(spin_calls=0, spin_wall_ns=0, spin_cpu_ns=0,
                spin_wall_max_ns=0, spin_cpu_max_ns=0, spin_exceptions=0,
                slow_records=0, slow_evicted=0, yield_calls=0,
                yield_wall_ns=0, yield_wall_max_ns=0, spinner_native_tid=None,
                observer_errors=0)
            super().__init__(*args, **kwargs)

        def spin(self):
            stats = self._r90_stats
            stats['spinner_native_tid'] = threading.get_native_id()
            while self.context.ok() and not self._r74_cooperative_stop.is_set():
                begin = cpu = None
                try:
                    begin, cpu = time.monotonic_ns(), time.thread_time_ns()
                except Exception:
                    stats['observer_errors'] += 1
                try:
                    self.spin_once()
                except BaseException:
                    stats['spin_exceptions'] += 1
                    raise
                finally:
                    stats['spin_calls'] += 1
                    try:
                        if begin is not None and cpu is not None:
                            cpu_elapsed = time.thread_time_ns() - cpu
                            end = time.monotonic_ns()
                            elapsed = end - begin
                            stats['spin_wall_ns'] += elapsed
                            stats['spin_cpu_ns'] += cpu_elapsed
                            stats['spin_wall_max_ns'] = max(stats['spin_wall_max_ns'], elapsed)
                            stats['spin_cpu_max_ns'] = max(stats['spin_cpu_max_ns'], cpu_elapsed)
                            if elapsed >= SLOW_NS:
                                stats['slow_records'] += 1
                                stats['slow_evicted'] += int(len(self._r90_slow) == SLOW_CAPACITY)
                                self._r90_slow.append(dict(begin_ns=begin, end_ns=end,
                                    wall_ns=elapsed, thread_cpu_ns=cpu_elapsed))
                    except Exception:
                        stats['observer_errors'] += 1
                begin = None
                try:
                    begin = time.monotonic_ns()
                except Exception:
                    stats['observer_errors'] += 1
                time.sleep(YIELD_SECONDS)
                stats['yield_calls'] += 1
                try:
                    if begin is not None:
                        elapsed = time.monotonic_ns() - begin
                        stats['yield_wall_ns'] += elapsed
                        stats['yield_wall_max_ns'] = max(stats['yield_wall_max_ns'], elapsed)
                except Exception:
                    stats['observer_errors'] += 1

        def shutdown(self, *args, **kwargs):
            self._r74_cooperative_stop.set()
            return super().shutdown(*args, **kwargs)

        def r90_statistics(self):
            return dict(self._r90_stats, slow_intervals=list(self._r90_slow),
                slow_capacity=SLOW_CAPACITY, slow_threshold_ns=SLOW_NS)

    return PositiveYieldExecutor(num_threads=num_threads)


def receipt(executor):
    actual = getattr(executor, '_r90_positive_yield_seconds', None)
    return dict(environment_key=KEY, environment_value=os.environ.get(KEY, '0'),
                enabled=actual is not None, yield_seconds=actual,
                actual_executor_class=type(executor).__qualname__,
                actual_executor_module=type(executor).__module__,
                worker_threads=getattr(getattr(executor, '_executor', None), '_max_workers', None),
                sole_spinner_owner=True, timing_change_only=True)


def finish(node, executor):
    """Exit-only, best-effort diagnostic flush; no change to shutdown outcome."""
    try:
        if not hasattr(executor, 'r90_statistics'):
            return
        value = dict(schema='r90_positive_yield_statistics/v1',
                     selection=receipt(executor), statistics=executor.r90_statistics())
        (node.run / 'r90_spinner_statistics.json').write_text(json.dumps(value) + '\n')
    except Exception:
        # The caller's original outcome remains authoritative if diagnostics fail.
        try:
            node._r90_statistics_write_failed = True
        except Exception:
            pass
