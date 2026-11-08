"""One bounded, independent interpreter for the unchanged static certifier.

The owner alone submits/polls. No ROS objects or live navigation state cross
this boundary. Pipe operations are nonblocking; failure never computes inline.
"""
from __future__ import annotations

import contextlib
import hashlib
import json
import math
import os
import select
from pathlib import Path
import subprocess
import sys
import threading
import time

MAX_REQUEST_BYTES = 128 * 1024
MAX_RESULT_BYTES = 2 * 1024 * 1024
IO_CHUNK_BYTES = 64 * 1024
JOB_TIMEOUT_S = 8.0
THREAD_CAPS = dict(OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1',
                   NUMEXPR_NUM_THREADS='1', BLIS_NUM_THREADS='1')


def _module_receipt(module_path):
    path = Path(module_path).resolve()
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def _handoff_probe(role):
    setting = os.environ.get('R93_HANDOFF_PROBE', '0')
    if setting not in ('0', '1'):
        raise ValueError('R93_HANDOFF_PROBE must be 0 or 1')
    if setting == '0':
        return None
    run = Path(os.environ.get('DEMO_RUN_DIR', ''))
    if not run.is_absolute() or not run.is_dir():
        raise ValueError('R93_HANDOFF_PROBE requires existing absolute DEMO_RUN_DIR')
    try:
        from embodied_navigation.common.observation import api as _observer_api; create_probe = _observer_api('r93_worker_probe').create_probe
        return create_probe(role, run, __file__)
    except Exception:
        # Configuration was valid; unavailable observation cannot stop work.
        return None


def _encoded(value, limit):
    raw = json.dumps(value, allow_nan=False, separators=(',', ':')).encode('utf-8') + b'\n'
    if len(raw) > limit:
        raise ValueError('physical_worker_message_byte_bound')
    return raw


def _validate_token(token):
    if not (type(token) is int and 0 <= token < 2**63 or
            type(token) is str and 0 < len(token.encode('utf-8')) <= 512):
        raise ValueError('physical_worker_invalid_token')


def _validate_payload(payload):
    if type(payload) is not dict:
        raise ValueError('physical_worker_payload_object_required')
    points, knots = payload.get('pos_pts'), payload.get('knots')
    if (type(points) is not list or not 4 <= len(points) <= 512
            or type(knots) is not list or len(knots) != len(points) + 4
            or payload.get('order') != 3):
        raise ValueError('physical_worker_curve_input_bound')
    for point in points:
        if type(point) is not list or len(point) != 3:
            raise ValueError('physical_worker_point_shape')
        for value in point:
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError('physical_worker_nonfinite_point')
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in knots):
        raise ValueError('physical_worker_nonfinite_knot')


class Worker:
    """Single-flight service; call close explicitly during owner shutdown."""

    def __init__(self, *, eager_submit=False):
        if type(eager_submit) is not bool:
            raise ValueError('physical_worker_eager_submit_boolean_required')
        self.eager_submit = eager_submit
        self._r93_probe = _handoff_probe('parent')
        environment = dict(os.environ, **THREAD_CAPS)
        self._process = subprocess.Popen(
            [sys.executable, '-u', '-m', 'embodied_navigation.planning.admission.geometry_worker', '--worker']
            + (['--eager-submit'] if eager_submit else []),
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            close_fds=True, env=environment, bufsize=0)
        os.set_blocking(self._process.stdin.fileno(), False)
        os.set_blocking(self._process.stdout.fileno(), False)
        self._pending = None
        self._outgoing = b''
        self._incoming = bytearray()
        self._submitted_ns = None
        self._submit_transport_error = None
        self._closed = False
        self._failure_reported = False
        self.failure = None
        self.ready = False
        self.startup_receipt = None

    @property
    def pid(self):
        return self._process.pid

    @property
    def busy(self):
        return self._pending is not None

    def submit(self, token, payload):
        if self._closed or self.failure:
            raise RuntimeError('physical_worker_unavailable:' + str(self.failure))
        if self.busy:
            raise RuntimeError('physical_worker_single_flight_busy')
        _validate_token(token)
        _validate_payload(payload)
        # Encoding freezes the caller-owned object before returning. No point
        # clouds, node state or Python pickle transport are accepted here.
        if self._r93_probe is not None:
            self._r93_probe.new_job(token)
            self._r93_probe.begin('encode')
        raw = _encoded(dict(token=token, payload=payload), MAX_REQUEST_BYTES)
        if self._r93_probe is not None:
            self._r93_probe.end('encode', byte_count=len(raw))
        self._pending = token
        self._outgoing = raw
        self._submitted_ns = time.monotonic_ns()
        if self.eager_submit:
            try:
                self._flush_outgoing_once()
            except (OSError, ValueError, TypeError, OverflowError) as error:
                # Preserve the original poll result/error path, including its
                # token and the admission owner's global fail-closed action.
                # No retry or result consumption occurs during submission.
                self._submit_transport_error = self._transport_reason(error)
        return True

    @staticmethod
    def _transport_reason(error):
        return ('physical_worker_transport:' + type(error).__name__ + ':' + str(error))[:1024]

    def _flush_outgoing_once(self):
        if self._outgoing:
            count = None
            if self._r93_probe is not None:
                self._r93_probe.begin('send')
            try:
                count = os.write(self._process.stdin.fileno(), self._outgoing[:IO_CHUNK_BYTES])
                self._outgoing = self._outgoing[count:]
            except BlockingIOError:
                pass
            finally:
                if self._r93_probe is not None:
                    self._r93_probe.end('send', byte_count=count,
                        outcome=('error' if sys.exc_info()[0] is not None else
                                 'would_block' if count is None else 'returned'))
                    self._r93_probe.annotate(outgoing_remaining_bytes=len(self._outgoing))

    def _fail(self, reason):
        if self._r93_probe is not None:
            self._r93_probe.annotate(failure=str(reason)[:1024],
                outgoing_remaining_bytes=len(self._outgoing), incoming_bytes=len(self._incoming))
            self._r93_probe.finish('transport_or_compute_failure')
        self.failure = str(reason)[:1024]
        token = self._pending
        self._pending = None
        self._outgoing = b''
        self._incoming.clear()
        # Signal only this Popen child. Reaping is bounded in explicit close.
        if self._process.poll() is None:
            try:
                self._process.terminate()
            except ProcessLookupError:
                pass
        self._failure_reported = True
        return token, None, dict(error=self.failure, worker_pid=self.pid,
                                 received_monotonic_ns=time.monotonic_ns())

    def result_readable(self):
        """Readiness hint only; the original poll owns all reads and errors."""
        if self._closed or self._failure_reported or not self.busy:
            return False
        try:
            return bool(select.select([self._process.stdout.fileno()], [], [], 0)[0])
        except (OSError, ValueError):
            # Leave transport/error handling to the unchanged periodic poll.
            return False

    def poll(self):
        if self._r93_probe is None:
            return self._poll_once()
        row = self._r93_probe.begin('poll')
        try:
            return self._poll_once()
        finally:
            self._r93_probe.end('poll', row=row,
                outcome='error' if sys.exc_info()[0] is not None else 'returned')

    def _poll_once(self):
        if self._closed or self._failure_reported:
            return None
        if self._submit_transport_error is not None:
            reason = self._submit_transport_error
            self._submit_transport_error = None
            return self._fail(reason)
        try:
            self._flush_outgoing_once()
            # Bound work per call independently of the maximum response size.
            chunk = None
            if self._r93_probe is not None:
                self._r93_probe.begin('read')
            try:
                chunk = os.read(self._process.stdout.fileno(), IO_CHUNK_BYTES)
            except BlockingIOError:
                chunk = None
            finally:
                if self._r93_probe is not None:
                    self._r93_probe.end('read', byte_count=len(chunk) if chunk is not None else None,
                        outcome=('error' if sys.exc_info()[0] is not None else
                                 'would_block' if chunk is None else 'returned'))
            if chunk:
                self._incoming.extend(chunk)
                if len(self._incoming) > MAX_RESULT_BYTES:
                    return self._fail('physical_worker_result_byte_bound')
            # At most the one startup receipt plus the single job result.
            for _ in range(2):
                if b'\n' not in self._incoming:
                    break
                raw, _, remainder = self._incoming.partition(b'\n')
                self._incoming = bytearray(remainder)
                if self._r93_probe is not None:
                    self._r93_probe.begin('decode')
                result = json.loads(raw)
                if self._r93_probe is not None:
                    self._r93_probe.end('decode', byte_count=len(raw)+1)
                if type(result) is dict and result.get('event') == 'ready':
                    receipt = result.get('receipt')
                    if (self.ready or type(receipt) is not dict
                            or receipt.get('worker_pid') != self.pid
                            or receipt.get('eager_submit') is not self.eager_submit
                            or receipt.get('thread_caps') != THREAD_CAPS):
                        return self._fail('physical_worker_readiness_identity')
                    self.ready = True
                    self.startup_receipt = receipt
                    continue
                if (not self.ready or not self.busy or type(result) is not dict
                        or type(result.get('token')) is not type(self._pending)
                        or result.get('token') != self._pending
                        or type(result.get('metrics')) is not dict):
                    return self._fail('physical_worker_result_identity')
                metrics = result['metrics']
                if metrics.get('error'):
                    return self._fail('physical_worker_compute:' + str(metrics['error']))
                if type(result.get('proof')) is not dict:
                    return self._fail('physical_worker_result_missing_proof')
                if (metrics.get('worker_pid') != self.pid
                        or metrics.get('startup_receipt') != self.startup_receipt):
                    return self._fail('physical_worker_result_process_identity')
                if self._incoming:
                    return self._fail('physical_worker_result_framing')
                token = self._pending
                self._pending = None
                metrics.update(submitted_monotonic_ns=self._submitted_ns,
                               received_monotonic_ns=time.monotonic_ns())
                if self._r93_probe is not None:
                    self._r93_probe.annotate(result_validated_monotonic_ns=metrics['received_monotonic_ns'],
                        worker_pid=self.pid, result_frame_bytes=len(raw)+1)
                    self._r93_probe.finish('validated_result_returned')
                return token, result['proof'], metrics
            if chunk == b'' or self._process.poll() is not None:
                return self._fail('physical_worker_exited:' + str(self._process.poll()))
            if self.busy and time.monotonic_ns() - self._submitted_ns > int(JOB_TIMEOUT_S * 1e9):
                return self._fail('physical_worker_job_timeout')
            return None
        except (OSError, ValueError, TypeError, OverflowError) as error:
            return self._fail(self._transport_reason(error))

    def close(self):
        if self._r93_probe is not None:
            self._r93_probe.annotate(outgoing_remaining_bytes=len(self._outgoing),
                incoming_bytes=len(self._incoming), pending_at_close=self._pending is not None,
                worker_pid=self.pid)
        self._closed = True
        self._pending = None
        self._outgoing = b''
        self._submit_transport_error = None
        self._incoming.clear()
        terminated = killed = False
        if self._process.stdin is not None and not self._process.stdin.closed:
            self._process.stdin.close()
        try:
            self._process.wait(timeout=.25)
        except subprocess.TimeoutExpired:
            terminated = True
            try:
                self._process.terminate()
            except ProcessLookupError:
                pass
            try:
                self._process.wait(timeout=.25)
            except subprocess.TimeoutExpired:
                killed = True
                try:
                    self._process.kill()
                except ProcessLookupError:
                    pass
                try:
                    self._process.wait(timeout=.25)
                except subprocess.TimeoutExpired:
                    pass
        if self._process.stdout is not None and not self._process.stdout.closed:
            self._process.stdout.close()
        receipt = dict(worker_pid=self.pid, returncode=self._process.poll(),
                    exited=self._process.poll() is not None, terminated=terminated,
                    killed=killed, failure=self.failure, startup_receipt=self.startup_receipt)
        if self._r93_probe is not None:
            receipt['r93_worker_probe'] = self._r93_probe.snapshot()
        return receipt


def _serve(*, eager_submit=False):
    probe = _handoff_probe('child')
    try:
        return _serve_loop(eager_submit=eager_submit, probe=probe)
    finally:
        if probe is not None:
            probe.write_exit(sys.exc_info()[1])


def _serve_loop(*, eager_submit=False, probe=None):
    # Fresh interpreter imports only the static leaf, never the ROS controller.
    import embodied_navigation.planning.admission.static_geometry as geometry
    source = sys.stdin.buffer
    destination = sys.stdout.buffer
    receipt = dict(worker_pid=os.getpid(), worker_tid=threading.get_native_id(),
        eager_submit=eager_submit,
        process_group=os.getpgrp(), parent_pid=os.getppid(),
        affinity=sorted(os.sched_getaffinity(0)),
        thread_caps={key: os.environ.get(key) for key in THREAD_CAPS},
        modules=dict(worker=_module_receipt(__file__), geometry=_module_receipt(geometry.__file__)),
        startup_ready_monotonic_ns=time.monotonic_ns())
    destination.write(_encoded(dict(event='ready', receipt=receipt), MAX_RESULT_BYTES))
    destination.flush()
    while True:
        if probe is not None:
            probe.new_job()
            probe.begin('read')
        raw = source.readline(MAX_REQUEST_BYTES + 1)
        if probe is not None:
            probe.end('read', byte_count=len(raw))
        if not raw:
            if probe is not None:
                probe.eof()
            return
        token = None
        metrics = dict(worker_pid=os.getpid(), worker_tid=threading.get_native_id(),
                       startup_receipt=receipt)
        try:
            if probe is not None:
                probe.begin('decode')
            if len(raw) > MAX_REQUEST_BYTES or not raw.endswith(b'\n'):
                raise ValueError('physical_worker_request_byte_bound_or_framing')
            request = json.loads(raw)
            token, payload = request['token'], request['payload']
            _validate_token(token)
            if probe is not None:
                probe.set_token(token)
            _validate_payload(payload)
            if probe is not None:
                probe.end('decode')
                probe.begin('compute')
            wall, thread_cpu, process_cpu = time.monotonic_ns(), time.thread_time_ns(), time.process_time_ns()
            with contextlib.redirect_stdout(sys.stderr):
                proof = geometry.certify_physical_curve(payload)
            finished = time.monotonic_ns()
            metrics.update(compute_started_monotonic_ns=wall, compute_finished_monotonic_ns=finished,
                           compute_wall_ns=finished-wall,
                           compute_thread_cpu_ns=time.thread_time_ns()-thread_cpu,
                           compute_process_cpu_ns=time.process_time_ns()-process_cpu)
            if probe is not None:
                # Keep legacy compute metrics contiguous with the original call.
                # This wider diagnostic bracket includes those original reads.
                probe.end('compute')
                probe.begin('encode')
            result = _encoded(dict(token=token, proof=proof, metrics=metrics), MAX_RESULT_BYTES)
            if probe is not None:
                probe.end('encode', byte_count=len(result))
        except Exception as error:
            if probe is not None:
                probe.error(error)
                probe.begin('error_encode')
            metrics['error'] = (type(error).__name__ + ':' + str(error))[:1024]
            result = _encoded(dict(token=token, proof=None, metrics=metrics), MAX_RESULT_BYTES)
            if probe is not None:
                probe.end('error_encode', byte_count=len(result))
        if probe is not None:
            probe.begin('write')
        written = destination.write(result)
        if probe is not None:
            probe.end('write', byte_count=written)
            probe.begin('flush')
        destination.flush()
        if probe is not None:
            probe.end('flush')
            probe.finish('result_write_and_flush_returned')


if __name__ == '__main__':
    if sys.argv[1:] not in (['--worker'], ['--worker', '--eager-submit']):
        raise SystemExit('Internal worker entry only')
    _serve(eager_submit=sys.argv[1:] == ['--worker', '--eager-submit'])
