"""Optional diagnostic recording, separate from live control state transport."""

from pathlib import Path
import json


class RuntimeTelemetry:
    """In-memory no-op recorder; only :meth:`atomic` writes live status to disk."""

    recording_enabled = False
    error = None

    def enqueue(self, operation):
        # Callbacks handed to this API are optional archive writes. Control
        # transactions settle in memory and never wait on this recorder.
        return None

    def append(self, path, value, **options):
        return None

    def append_pid_snapshot(self, *args, **kwargs):
        return None

    def atomic(self, path, value):
        # This is live status transport, not a historical evidence archive.
        path = Path(path)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(value, allow_nan=False) + "\n")
        temporary.replace(path)

    def close(self):
        return None


class RuntimeEvents:
    """Real event counters without saving clouds or gating control on archives."""

    def __init__(self):
        self.count = 0

    def enqueue(self, arrays, metadata):
        self.count += 1
        return "obstacle_" + str(self.count)

    def status(self, **options):
        return dict(events=self.count, recording_enabled=False, error=None)

    def close(self, *args, **kwargs):
        return None


class NullHistory:
    """Always-empty history sink used when recording is disabled."""

    def __init__(self, *args, **kwargs):
        pass

    def submit(self, *args, **kwargs):
        return False

    def close(self):
        return dict(recording_enabled=False)

    def snapshot(self):
        return dict(recording_enabled=False)


class NullStream:
    """Optional history sink; live state and ACK use separate explicit writers."""

    def write(self, text):
        return len(text)

    def flush(self):
        pass

    def close(self):
        pass


class NoRecordingContext:
    """Pass-through projection context that persists nothing."""

    def __init__(self, *args, **kwargs):
        pass

    def install_height_path(self, *args, **kwargs):
        pass

    def project_evidence(self, value):
        return value

    def project_telemetry(self, value):
        return value

    def project_height(self, value):
        return value

    def close(self):
        return None
