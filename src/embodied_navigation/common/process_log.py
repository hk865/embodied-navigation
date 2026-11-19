"""Bound subprocess logs without changing the child lifetime or exit status."""

from collections import deque
import json
import os
from pathlib import Path
import re
import selectors
import signal
import subprocess
import sys
import time

from embodied_navigation.common.runtime_telemetry import RuntimeTelemetry


# Match only this installed SDK warning, including its channel and source site.
_INVALID_FACE = re.compile(
    rb"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z \[[0-9,]+ms\] "
    rb"\[Warning\] \[omni\.physx\.plugin\] "
    + re.escape(
        b"PhysX warning: PxShape::getMaterialFromInternalFaceIndex received "
        b"0xFFFFffff as input - returning NULL., FILE "
        b"/builds/omniverse/physics/physx/source/physx/src/NpShape.cpp, LINE 598"
    )
    + rb"(?:\r?\n)?"
)
_LEVEL = re.compile(rb"\[(Verbose|Info|Warning|Error|Fatal)\]")
_CHUNK_BYTES = 64 * 1024
_SIGNALS = (signal.SIGINT, signal.SIGTERM)


def read_log_config(path):
    """Validate the four external file budgets before starting a child."""
    config = json.loads(Path(path).read_text())
    for key in ("head_bytes", "tail_bytes", "error_head_bytes", "error_tail_bytes"):
        if type(config[key]) is not int or config[key] <= 0:
            raise ValueError(f"{key} must be a positive integer byte budget")
    return config


class _HeadTail:
    def __init__(self, output, prefix, head_budget, tail_budget):
        self.paths = [output / f"{prefix}.{part}.log" for part in ("head", "tail")]
        for path in self.paths:
            path.touch(exist_ok=False)
        self.head_budget, self.tail_budget = head_budget, tail_budget
        self.head = bytearray()
        self.tail = deque()
        self.tail_size = self.total = 0
        self.dirty = False

    def append(self, data):
        self.total += len(data)
        take = min(len(data), self.head_budget - len(self.head))
        self.head.extend(data[:take])
        data = data[take:]
        if data:
            self.tail.append(data)
            self.tail_size += len(data)
        while self.tail_size > self.tail_budget:
            excess = self.tail_size - self.tail_budget
            chunk = self.tail.popleft()
            remove = min(excess, len(chunk))
            self.tail_size -= remove
            if remove < len(chunk):
                self.tail.appendleft(chunk[remove:])
        self.dirty = True

    def save(self):
        if self.dirty:
            # In-place bounded snapshots avoid a second full-size temporary log.
            self.paths[0].write_bytes(self.head)
            with self.paths[1].open("wb") as stream:
                stream.writelines(self.tail)
            self.dirty = False

    def status(self):
        retained = len(self.head) + self.tail_size
        return dict(total_bytes=self.total, retained_bytes=retained,
                    overwritten_bytes=self.total - retained,
                    head_bytes=len(self.head), tail_bytes=self.tail_size,
                    tail_start_byte=self.total - self.tail_size)


class _LogCapture:
    def __init__(self, output, config):
        self.output = output
        self.normal = _HeadTail(output, "stdout", config["head_bytes"], config["tail_bytes"])
        self.error = _HeadTail(output, "errors", config["error_head_bytes"], config["error_tail_bytes"])
        self.pending = {"stdout": bytearray(), "stderr": bytearray()}
        self.error_context = {"stdout": False, "stderr": True}
        self.fragment = {"stdout": False, "stderr": False}
        self.bytes = {"stdout": 0, "stderr": 0}
        self.warning = dict(total=0, suppressed=0, suppressed_bytes=0, first=None, last=None)
        self.recording_error = None

    def feed(self, source, data, *, eof=False):
        self.bytes[source] += len(data)
        pending = self.pending[source]
        pending.extend(data)
        while pending:
            newline = pending.find(b"\n")
            if 0 <= newline < _CHUNK_BYTES:
                size, complete = newline + 1, True
            elif len(pending) >= _CHUNK_BYTES:
                size, complete = _CHUNK_BYTES, False
            elif eof:
                size, complete = len(pending), True
            else:
                break
            record = bytes(pending[:size])
            del pending[:size]
            self.record(source, record, complete)

    def record(self, source, record, complete):
        continuation = self.fragment[source]
        self.fragment[source] = not complete
        if not continuation and complete and _INVALID_FACE.fullmatch(record):
            warning = self.warning
            warning["total"] += 1
            warning["last"] = record.decode("ascii")
            if warning["first"] is None:
                warning["first"] = warning["last"]
                self.normal.append(record)
            else:
                warning["suppressed"] += 1
                warning["suppressed_bytes"] += len(record)
            return
        if source == "stdout" and not continuation:
            level = _LEVEL.search(record)
            if level is not None:
                self.error_context[source] = level[1] in (b"Error", b"Fatal")
            elif record.startswith(b"Traceback (most recent call last):"):
                self.error_context[source] = True
        bucket = self.error if self.error_context[source] else self.normal
        bucket.append(record)

    def save(self, state):
        try:
            self.normal.save()
            self.error.save()
            RuntimeTelemetry().atomic(self.output / "summary.json", {
                **state, "normal": self.normal.status(), "error": self.error.status(),
                "target_warning": self.warning, "recording_error": self.recording_error,
                "input_bytes": sum(self.bytes.values()),
                "stdout_bytes": self.bytes["stdout"], "stderr_bytes": self.bytes["stderr"],
            })
        except OSError as error:
            # Log failure must not change navigation. Keep draining bounded buffers;
            # retry on the next checkpoint, and report the first failure once.
            if self.recording_error is None:
                self.recording_error = str(error)
                print(f"bounded log recording failed: {error}", file=sys.stderr, flush=True)


def run_logged(command, output_dir, config):
    """Run an owned process group; forward cancellation and return Popen's code.

    No duration budget is imposed. Only this invocation's new directory is used.
    A forced SIGKILL cannot flush the latest in-memory tail/count checkpoint.
    """
    output = Path(output_dir)
    output.mkdir()
    capture = _LogCapture(output, config)
    state = dict(child_pid=None, child_returncode=None, signals_forwarded=[],
                 signal_counts={str(int(s)): 0 for s in _SIGNALS}, config=config)
    previous = {s: signal.getsignal(s) for s in _SIGNALS}
    child = None
    pending_signals = []

    def forward(signum, _frame):
        if signum not in state["signals_forwarded"]:
            state["signals_forwarded"].append(signum)
        state["signal_counts"][str(signum)] += 1
        if child is None:
            if signum not in pending_signals:
                pending_signals.append(signum)
        else:
            try:
                os.killpg(child.pid, signum)
            except ProcessLookupError:
                pass  # The owned group may have exited just before this signal.

    for signum in _SIGNALS:
        signal.signal(signum, forward)
    try:
        child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 start_new_session=True)
        state["child_pid"] = child.pid
        for signum in pending_signals:
            try:
                os.killpg(child.pid, signum)
            except ProcessLookupError:
                pass
        capture.save(state)
        saved_at = time.monotonic()
        with selectors.DefaultSelector() as selector:
            for source in ("stdout", "stderr"):
                stream = getattr(child, source)
                selector.register(stream, selectors.EVENT_READ, source)
            while selector.get_map():
                for key, _ in selector.select(timeout=0.2):
                    data = os.read(key.fd, _CHUNK_BYTES)
                    capture.feed(key.data, data, eof=not data)
                    if not data:
                        selector.unregister(key.fileobj)
                        key.fileobj.close()
                if time.monotonic() - saved_at >= 1.0:
                    capture.save(state)
                    saved_at = time.monotonic()
        state["child_returncode"] = child.wait()
        capture.save(state)
        return child.returncode
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def exit_like_child(returncode):
    """Preserve signal termination as a signal, rather than sys.exit(-signal)."""
    if returncode < 0:
        signum = -returncode
        if signum not in (signal.SIGKILL, signal.SIGSTOP):
            signal.signal(signum, signal.SIG_DFL)
        os.kill(os.getpid(), signum)
    raise SystemExit(returncode)
