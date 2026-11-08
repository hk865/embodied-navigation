"""Fail-closed, same-run navigation command admission for the Gazebo actor."""
import json
import time
from pathlib import Path
import numpy as np


def admit_command(record, *, run_id, source_ns, monotonic_now):
    command = np.asarray(record['command'], dtype=float)
    stamp = float(record['stamp_ns'])
    received = float(record['monotonic_wall'])
    age = (source_ns - stamp) / 1e9
    wall_age = monotonic_now - received
    if (record['run_id'] != run_id or command.shape != (3,)
            or not np.isfinite(command).all() or np.any(abs(command) > [.6, .2, .3])
            or not np.isfinite([stamp, received, source_ns, monotonic_now]).all()
            or not -.05 <= age < .3 or not 0 <= wall_age < .3):
        raise ValueError('command identity/bounds/freshness')
    return command


def read_command(path, *, run_id, source_ns, monotonic_now=None):
    record = {}
    try:
        record = json.loads(Path(path).read_text())
        command = admit_command(record, run_id=run_id, source_ns=source_ns,
                                monotonic_now=time.monotonic() if monotonic_now is None else monotonic_now)
        return command, record, None
    except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
        return np.zeros(3), record if isinstance(record, dict) else {}, str(error)
