#!/usr/bin/env python3
"""Offline-only native exploration audit; never an exploration input or controller."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np


def load_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def world_velocity(state):
    """Rotate native body-frame velocity to world XY for offline speed scoring."""
    q = state[:, 4:8]
    q = q / np.linalg.norm(q, axis=1)[:, None]
    vector = state[:, 8:11]
    imaginary = q[:, 1:]
    cross = np.cross(imaginary, vector)
    return vector + 2 * (q[:, :1] * cross + np.cross(imaginary, cross))


def longest_sample_window(mask, times, step):
    best = current = 0
    for index, value in enumerate(mask):
        current = current + 1 if value and (not current or times[index] - times[index - 1] <= step * 1.01) else int(value)
        best = max(best, current)
    return dict(samples=best, sample_coverage_s=best * step,
                endpoint_span_s=max(0, best - 1) * step)


def audit(run):
    control = load_rows(run / "capture/interactive_waypoint_control.jsonl")
    policy = load_rows(run / "capture/policy_frames.jsonl")
    receipts = load_rows(run / "capture/interactive/receipts.jsonl")
    result = json.loads((run / "capture/result.json").read_text())
    t = np.array([row["measured_time"] for row in policy])
    commands = np.array([row["actual_command"] for row in policy])
    requested = np.array([row["request"] for row in policy])
    controls = np.array([row["command"] for row in control])
    native = np.array([row["native_state64"] for row in policy])
    step = float(np.median(np.diff(t)))
    speed = np.linalg.norm(world_velocity(native)[:, :2], axis=1)
    nonzero = np.any(np.abs(commands) > 1e-10, axis=1)
    cruise = commands[:, 0] >= .95
    first = t[nonzero][0]
    start = (t >= first - 1) & (t < first)
    end = t >= t[-1] - 1
    pose = np.array([row["navigation_source"]["position"] if row.get("navigation_source") else [np.nan] * 3 for row in policy])
    def net(points):
        return float(np.linalg.norm(np.nanmean(points[end, :2], axis=0) - np.nanmean(points[start, :2], axis=0)))
    final_stop = np.array([row["reason"] == "interactive_stop" for row in control])
    source_files = json.loads((run / "capture/source_manifest.json").read_text())
    important = [name for name in source_files if name.endswith(("native_waypoints.py", "capture/control.py", "capture/frames.py", "robogauge/route_tracker.py"))]
    source_match = {name: hashlib.sha256((run / "capture/source_snapshot" / name).read_bytes()).hexdigest() == source_files[name] for name in important}
    return dict(
        run=str(run), capture_exit=int((run / "capture_exit.txt").read_text()),
        end_reason=result.get("end_reason"), error=result.get("error"),
        executed_physics_steps=result["executed_physics_steps"], final_policy_sim_s=float(t[-1]),
        loop_wall_s=result["loop_wall_seconds"],
        receipt_counts=dict(Counter(row["event"] for row in receipts)),
        reached_sequences=[row["seq"] for row in receipts if row["event"] == "waypoint_reached"],
        control_reason_counts=dict(Counter(row["reason"] for row in control)),
        control_policy_time_equal=bool(np.array_equal(t, [row["current_sim_s"] for row in control])),
        request_actual_max_difference=float(np.max(np.abs(requested - commands))),
        control_actual_max_difference=float(np.max(np.abs(controls - commands))),
        configured_command_ceiling_mps=result["experiment"]["speed_mps"],
        actual_command_forward_peak_mps=float(commands[:, 0].max()),
        nonzero_command_sample_coverage_s=float(nonzero.sum() * step),
        cruise_command_threshold_mps=.95,
        cruise_command_sample_coverage_s=float(cruise.sum() * step),
        native_world_xy_speed_during_cruise_command=dict(mean=float(speed[cruise].mean()), p95=float(np.percentile(speed[cruise], 95)), peak=float(speed[cruise].max())),
        native_world_xy_speed_peak_after_first_command_mps=float(speed[t >= first].max()),
        native_speed_ge_09_longest=longest_sample_window(speed >= .9, t, step),
        native_speed_ge_09_and_cruise_command_longest=longest_sample_window((speed >= .9) & cruise, t, step),
        settled_endpoint_window_s=1.0, estimated_net_xy_m=net(pose), native_offline_net_xy_m=net(native[:, 1:4]),
        terminal_zero_command_coverage_s=float(final_stop.sum() * step),
        terminal_commands_all_zero=bool(np.all(commands[final_stop] == 0)),
        last_second_native_speed3d_peak_mps=float(np.linalg.norm(native[end, 8:11], axis=1).max()),
        last_second_native_omega3d_peak_rps=float(np.linalg.norm(native[end, 11:14], axis=1).max()),
        archived_controller_source_hashes_match=source_match,
        truth_usage="Native state read only after all exploration decisions and run completion, for offline scoring. Chooser used front RGB, measured occupancy and estimated pose.",
        limitations=["Finite lobby/corridor-entrance exploration, not full basement or building coverage", "Command ceiling and short near-1m/s intervals do not establish sustained 1m/s cruise", "Contact and COM-velocity conventions remain unvalidated by the original capture"],
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    result = audit(args.run)
    (args.run / "native_exploration_audit.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
