"""Offline same-stamp actuator evidence audit; an inference receipt is not motion."""

from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np


def read_rows(path):
    """Read JSONL rows from one log file; return ``(rows, malformed_count)``.

    A missing file yields no rows and no malformed lines.
    """
    rows = []
    malformed = 0
    if not path.is_file():
        return rows, 0
    with path.open() as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError("Expected object")
                rows.append(row)
            except (ValueError, TypeError):
                malformed += 1
    return rows, malformed


def _index(rows, kind):
    """Index rows of one ``kind`` by rounded integer-nanosecond stamp (last row wins)."""
    indexed = {}
    duplicates = 0
    for row in rows:
        if row.get("kind") != kind:
            continue
        stamp = round(float(row["t"]) * 1e9)
        if stamp in indexed:
            duplicates += 1
        indexed[stamp] = row
    return indexed, duplicates


def audit(run):
    """Audit one run's logs for same-stamp policy/physics agreement and return a receipt.

    Telemetry qtarget is compared with the physics step of the same stamp (maximum
    absolute delta <= 1e-12). Motor torque is compared after the Go2HV envelope
    clamp (maximum absolute delta <= 1e-9). Any malformed line, duplicate stamp or
    unmatched eligible frame makes the execution evidence fail.
    """
    run = Path(run)
    telemetry, bad_t = read_rows(run / "telemetry.jsonl")
    physics, bad_p = read_rows(run / "actuator_physics.jsonl")
    exchanges, dup_ex = _index(physics, "policy_exchange")
    steps, dup_ph = _index(physics, "physics_step")
    deltas = []
    torque = []
    unmatched = []
    eligible = 0
    seen = set()
    duplicate_telemetry = 0
    for row in telemetry:
        stamp = round(float(row["world_sim_time"]) * 1e9)
        if stamp in seen:
            duplicate_telemetry += 1
        seen.add(stamp)
        exchange = exchanges.get(stamp)
        if exchange and (exchange["terminate"] or exchange["response_mode"]):
            continue
        eligible += 1
        if exchange is None or stamp not in steps:
            unmatched.append(stamp)
            continue
        target = np.asarray(row["q_target"], float)
        actual = np.asarray(steps[stamp]["qtarget"], float)
        if (
            target.shape != (12,)
            or actual.shape != (12,)
            or not np.isfinite([target, actual]).all()
        ):
            raise ValueError("Invalid qtarget evidence")
        deltas.append(float(np.max(abs(target - actual))))
    for row in steps.values():
        raw = np.asarray(row["pd_raw"], float)
        qd = np.asarray(row["qd"], float)
        actual = np.asarray(row["tau"], float)
        if (
            any(x.shape != (12,) for x in (raw, qd, actual))
            or not np.isfinite([raw, qd, actual]).all()
        ):
            raise ValueError("Invalid motor evidence")
        # Go2HV envelope: 20.2 Nm while raw torque and joint velocity share a sign,
        # otherwise 23.4 Nm, derated linearly to zero between 13.5 and 30 rad/s.
        limit = np.where(raw * qd > 0, 20.2, 23.4) * np.minimum(
            1, np.maximum(0, (30 - abs(qd)) / 16.5)
        )
        torque.append(float(np.max(abs(np.clip(raw, -limit, limit) - actual))))

    def obj(name):
        path = run / name
        return json.loads(path.read_text()) if path.is_file() else None

    status = obj("truth_navigation_status.json")
    worker = obj("worker_result.json")
    faults = sorted({str(row["fault"]) for row in telemetry if row.get("fault")})
    malformed = bad_t + bad_p
    duplicates = dup_ex + dup_ph + duplicate_telemetry
    matched = bool(
        eligible
        and len(deltas) == eligible
        and torque
        and not malformed
        and not duplicates
        and max(deltas) <= 1e-12
        and max(torque) <= 1e-9
    )
    xyz = np.asarray([r["position"] for r in telemetry], float)
    times = np.asarray([r["world_sim_time"] for r in telemetry], float)
    return dict(
        run=run.name,
        telemetry_frames=len(telemetry),
        physics_frames=len(steps),
        eligible_nonterminal_policy_frames=eligible,
        matched_policy_and_physics_frames=len(deltas),
        missing_same_stamp_frames=len(unmatched),
        first_missing_stamps_ns=unmatched[:10],
        malformed_log_lines=malformed,
        duplicate_timestamps=duplicates,
        maximum_same_stamp_qtarget_delta=max(deltas) if deltas else None,
        maximum_Go2HV_torque_delta=max(torque) if torque else None,
        execution_evidence_passed=matched,
        navigation_status=status,
        worker_result=worker,
        faults=faults,
        navigation_and_execution_passed=bool(
            matched
            and not faults
            and status
            and status.get("state") == "succeeded"
            and worker
            and worker.get("fault") is None
        ),
        sim_duration_s=float(times[-1] - times[0]) if len(times) else None,
        height_gain_m=float(xyz[-1, 2] - xyz[0, 2]) if len(xyz) else None,
        trajectory_length_xy_m=float(np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1).sum())
        if len(xyz)
        else None,
        scope="Offline logged execution evidence, not independent validation of simulator physics or success rate",
        full_training_physics_equivalence=False,
    )


def main(argv=None):
    """Print (and optionally write) the JSON receipt; return 0 only on full pass."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run", type=Path)
    ap.add_argument("--output", type=Path)
    args = ap.parse_args(argv)
    result = audit(args.run)
    text = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(text)
    print(text, end="")
    return 0 if result["navigation_and_execution_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
