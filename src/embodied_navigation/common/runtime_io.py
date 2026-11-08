"""Bounded background I/O. Sensor callbacks never wait for disk durability.

Original acquisition/envelope stamps are retained. A delayed command therefore
expires at the unchanged consumer watchdog, rather than becoming fresh on write.
"""
from __future__ import annotations
import json
from pathlib import Path
import queue
import threading
import time


class EvidenceWriter:
    def __init__(self, capacity=4096):
        self.queue=queue.Queue(capacity);self.error=None;self.completed=0
        self.thread=threading.Thread(target=self._run,daemon=True,name='teacher-nav-evidence')
        self.thread.start()
    def enqueue(self, operation):
        if self.error:raise RuntimeError(self.error)
        try:self.queue.put_nowait(operation)
        except queue.Full:
            self.error='Navigation evidence queue overflow; refusing silent evidence loss'
            raise RuntimeError(self.error)
    def append(self,path,data):
        # Serialize immutable small records here; compression/file I/O is off
        # the ROS executor. JSON errors remain immediate and fail closed.
        line=json.dumps(data,allow_nan=False,ensure_ascii=False)+'\n';path=Path(path)
        def write():
            with path.open('a')as stream:stream.write(line)
        self.enqueue(write)
    def atomic(self,path,data):
        content=json.dumps(data,allow_nan=False,ensure_ascii=False)+'\n';path=Path(path)
        def write():
            tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(content);tmp.replace(path)
        self.enqueue(write)
    def _run(self):
        while True:
            operation=self.queue.get()
            try:
                if operation is None:return
                if not self.error:operation();self.completed+=1
            except Exception as exc:self.error=f'Navigation evidence write failed: {type(exc).__name__}: {exc}'
            finally:self.queue.task_done()
    def close(self):
        # Finite audit records drain on orderly shutdown; no ROS callback waits.
        self.queue.put(None);self.thread.join(timeout=10.)
        if self.thread.is_alive():self.error='Navigation evidence writer did not finish shutdown'
        if self.error:raise RuntimeError(self.error)


class LatestCommandWriter:
    """One in-flight write and one replaceable pending envelope, never a FIFO.

    Sequence gaps explicitly mean an intermediate envelope was superseded before
    transport. History contains every actually written envelope, not invented
    50 Hz deliveries. Zero requests replace pending motion immediately.
    """
    def __init__(self, writer, history=None):
        self.writer=writer;self.history=Path(history)if history else None
        self.condition=threading.Condition();self.pending=None;self.stopping=False
        self.error=None;self.superseded=0;self.written=0;self.max_delay_s=0.
        self.thread=threading.Thread(target=self._run,daemon=True,name='teacher-nav-command-file')
        self.thread.start()
    def submit(self,envelope):
        with self.condition:
            if self.error:raise RuntimeError(self.error)
            if self.stopping:raise RuntimeError('Command writer is stopping')
            if self.pending is not None:self.superseded+=1
            self.pending=envelope;self.condition.notify()
    def _run(self):
        while True:
            with self.condition:
                while self.pending is None and not self.stopping:self.condition.wait()
                if self.pending is None and self.stopping:return
                value=self.pending;self.pending=None
            try:
                begin=time.monotonic();self.writer.write(value);end=time.monotonic()
                delay=begin-value['monotonic_wall'];self.max_delay_s=max(self.max_delay_s,delay)
                self.written+=1
                if self.history:
                    receipt={**value,'transport_write_started_monotonic_wall':begin,
                        'transport_written_monotonic_wall':end,'transport_queue_delay_wall_s':delay,
                        'transport_write_duration_wall_s':end-begin,
                        'transport_superseded_envelopes':self.superseded}
                    with self.history.open('a')as stream:stream.write(json.dumps(receipt,allow_nan=False,ensure_ascii=False)+'\n')
            except Exception as exc:
                with self.condition:self.error=f'Command transport failed: {type(exc).__name__}: {exc}'
    def close(self,final_envelope=None):
        with self.condition:
            if final_envelope is not None:self.pending=final_envelope
            self.stopping=True;self.condition.notify()
        self.thread.join(timeout=2.)
        if self.thread.is_alive():raise RuntimeError('Command transport shutdown timed out; previous command expires at 300ms')
        if self.error:raise RuntimeError(self.error)


class BackgroundCheck:
    """ROS graph and immutable source checks run without starving callbacks."""
    def __init__(self,operation,period=.1):
        self.operation=operation;self.period=period;self.result=None;self.stop=threading.Event()
        self.thread=threading.Thread(target=self._run,daemon=True,name='teacher-nav-integrity')
        self.thread.start()
    def _run(self):
        while not self.stop.is_set():
            start=time.monotonic()
            try:data=self.operation();error=None
            except Exception as exc:data=None;error=f'{type(exc).__name__}: {exc}'
            # Age uses check START, never disguises a slow check as fresh.
            self.result={'started_wall':start,'completed_wall':time.monotonic(),'data':data,'error':error}
            self.stop.wait(max(0.,self.period-(time.monotonic()-start)))
    def close(self):
        self.stop.set();self.thread.join(timeout=2.)
        if self.thread.is_alive():raise RuntimeError('Integrity checker did not finish shutdown')


def latest_sensor_qos():
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    return QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)


from contextlib import contextmanager
@contextmanager
def bounded_shutdown_signals():
    import signal
    previous={}
    if threading.current_thread()is threading.main_thread():
        previous={sig:signal.signal(sig,signal.SIG_IGN)for sig in (signal.SIGINT,signal.SIGTERM)}
    try:yield
    finally:
        for sig,handler in previous.items():signal.signal(sig,handler)
