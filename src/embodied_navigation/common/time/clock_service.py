"""One non-waiting service of the original /clock before the original body gate.

No new subscription, clock value, future-frame retention, executor task or thread.
The original TimeSource callback, jump handling and PID/Teacher now reads remain.
"""
from collections import deque
import hashlib
import inspect
import json
import os
from pathlib import Path
import threading
import time

KEY = 'R114_CLOCK_SERVICE'
_value = os.environ.get(KEY, '0')
if _value not in ('0', '1'):
    raise ValueError(KEY + ' must be 0 or 1')
CLOCK_SERVICE_ENABLED = _value == '1'
CAPACITY = 64
COLUMNS = ('source_ns', 'begin_mono_ns', 'end_mono_ns', 'thread_cpu_ns', 'tid',
           'clock_before_ns', 'clock_after_ns', 'actual_message_clock_ns', 'outcome')


def _same_method(a, b):
    return (getattr(a, '__self__', None) is getattr(b, '__self__', None)
            and getattr(a, '__func__', a) is getattr(b, '__func__', b))


class Service:
    def __init__(self, node, executor, sub, original_callback, sources):
        self.enabled = True
        self.node, self.executor, self.sub = node, executor, sub
        self.group, self.control_group = sub.callback_group, node.default_callback_group
        self.callback, self.take = sub.callback, executor._take_subscription
        self.time_source = original_callback.__self__
        self.sources = sources
        self.rows = deque(maxlen=CAPACITY)
        self.counts = dict(calls=0, busy=0, empty=0, serviced=0, exceptions=0,
                           shutdown=0, rows=0, evicted=0, diagnostic_errors=0)

    def service(self, source_ns):
        """Called only by the original body method while R86 owns its control group."""
        self.counts['calls'] += 1
        begin, cpu = time.monotonic_ns(), time.thread_time_ns()
        before = self.node.get_clock().now().nanoseconds
        outcome, message = 'exception', None
        acquired = False
        try:
            bundle = getattr(self.node, '_r86_committing_bundle', None)
            if (bundle is None or bundle.source_ns != source_ns
                    or self.control_group._active_entity is None):
                raise RuntimeError('R114 service outside original R86 control-owned body dispatch')
            if self.executor._is_shutdown:
                outcome = 'shutdown'
                return
            if (self.sub.callback is not self.callback
                    or not _same_method(self.executor._take_subscription, self.take)
                    or self.sub.callback_group is not self.group
                    or self.time_source._clock_sub is not self.sub):
                raise RuntimeError('R114 original clock binding changed')
            if not self.group.beginning_execution(self.sub):
                outcome = 'busy'
                return
            acquired = True
            # Exactly one original take. Never write _executor_event: a previously
            # queued handler still owns its scheduling marker, not a cached message.
            execute = self.take(self.sub)
            if execute is None:
                outcome = 'empty'
                return
            coroutine = execute()
            if not inspect.iscoroutine(coroutine):
                raise RuntimeError('R114 unsupported clock execution object')
            try:
                try:
                    coroutine.send(None)
                except StopIteration:
                    pass
                else:
                    # No second send, scheduling or hidden await. Closing unwinds
                    # the existing probe ContextVar finally before fail-closed.
                    raise RuntimeError('R114 original clock unexpectedly suspended')
            finally:
                coroutine.close()
            message = self.time_source._last_time_set.nanoseconds
            outcome = 'serviced'
        except BaseException:
            self.counts['exceptions'] += 1
            raise
        finally:
            try:
                if acquired:
                    try:
                        self.group.ending_execution(self.sub)
                    finally:
                        self.executor.wake()
            except BaseException:
                self.counts['exceptions'] += 1
                outcome = 'exception'
                raise
            finally:
                if outcome != 'exception':
                    self.counts[outcome] += 1
                try:
                    after = self.node.get_clock().now().nanoseconds
                    row = (source_ns, begin, time.monotonic_ns(), time.thread_time_ns()-cpu,
                           threading.get_native_id(), before, after, message, outcome)
                    self.counts['rows'] += 1
                    self.counts['evicted'] += int(len(self.rows) == CAPACITY)
                    self.rows.append(row)
                except Exception:
                    self.counts['diagnostic_errors'] += 1

    def receipt(self):
        return dict(schema='r114_original_clock_service/v1', enabled=self.enabled,
                    environment_value=_value, process_fixed=True, default_off=True,
                    counts=dict(self.counts), columns=COLUMNS, tail=list(self.rows),
                    capacity=CAPACITY, sources=self.sources,
                    clock_group_distinct_from_control=True, take_limit_per_call=1,
                    old_R99_enabled=False, new_wait_or_future_frame_acceptance=False,
                    hard_wall_bound_claimed=False, actual_thread_is_calling_control_thread=True)


def install(node, executor):
    if not CLOCK_SERVICE_ENABLED:
        return None
    import rclpy.executors as execution
    import rclpy.time_source as time_source
    import rclpy.callback_groups as groups
    import rclpy.subscription as subscription
    import rclpy.clock as clock
    from rclpy.qos import ReliabilityPolicy
    if (os.environ.get('R99_CLOCK_FEEDBACK', '0') != '0'
            or getattr(node, '_r99_policy', None) is not None
            or getattr(node, '_r86_prepared_mailbox', None) is None):
        raise RuntimeError('R114 requires original R86 and old R99 OFF')
    sources = {}
    for module in (execution, time_source, groups, subscription, clock):
        path = Path(module.__file__).resolve()
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        sources[path.name] = dict(path=str(path), sha256=digest)
    matches = [s for s in node.subscriptions if s.topic_name == '/clock']
    if len(matches) != 1:
        raise RuntimeError('R114 requires exactly one original clock subscription')
    sub = matches[0]
    callback = inspect.unwrap(sub.callback)
    if (type(sub) is not subscription.Subscription or sub.raw
            or sub._callback_type is not subscription.Subscription.CallbackType.MessageOnly
            or not inspect.ismethod(callback) or type(callback.__self__) is not time_source.TimeSource
            or callback.__func__ is not time_source.TimeSource.clock_callback
            or inspect.iscoroutinefunction(callback)
            or callback.__self__._clock_sub is not sub
            or node.get_clock() not in callback.__self__._associated_clocks
            or not callback.__self__.ros_time_is_active
            or sub.qos_profile.depth != 1
            or sub.qos_profile.reliability != ReliabilityPolicy.BEST_EFFORT):
        raise RuntimeError('R114 original synchronous TimeSource/QoS differs')
    if (type(sub.callback_group) is not groups.MutuallyExclusiveCallbackGroup
            or sub.callback_group is not getattr(node, '_r74_clock_callback_group', None)
            or sub.callback_group is node.default_callback_group
            or type(node.default_callback_group) is not groups.MutuallyExclusiveCallbackGroup):
        raise RuntimeError('R114 original independent clock/control groups required')
    if (not _same_method(sub.callback, callback) or
            getattr(inspect.unwrap(executor._take_subscription), '__func__', inspect.unwrap(executor._take_subscription)) is not execution.Executor._take_subscription):
        raise RuntimeError('Clock service requires the original synchronous executor take')
    return Service(node, executor, sub, callback, sources)


def finish(node):
    service = getattr(node, '_r114_clock_service', None)
    if service is not None:
        # Exit-only observation. A diagnostic failure never revises business state.
        try:
            (Path(node.run)/'r114_clock_service.json').write_text(
                json.dumps(service.receipt(), allow_nan=False)+'\n')
        except Exception:
            service.counts['diagnostic_errors'] += 1
