#!/usr/bin/env python3
"""Summarize recorded DynaNav evidence without starting Isaac or altering a run.

Native velocities are post-step samples, weighted over their preceding physics
step and intersected with nonzero actual-command intervals. Missing evidence
stays null. A completed process is never treated as successful navigation.
"""

from __future__ import annotations

import argparse
import bisect
import itertools
import json
import math
import statistics
import sys
from pathlib import Path


def number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def vector(value, length=3):
    return (
        isinstance(value, list)
        and len(value) == length
        and all(number(x) for x in value)
    )


def read_stream(path):
    """Keep usable objects and report corrupt lines, including interrupted writes."""
    receipt = {"path": str(path), "status": "missing", "objects": 0, "invalid_lines": 0}
    rows = []
    if not path.is_file():
        return rows, receipt
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError("Expected JSON object")  # noqa: TRY004 - Keep the established ValueError corrupt-row handling contract.
            except ValueError:
                receipt["invalid_lines"] += 1
                receipt.setdefault("first_invalid_line", line_number)
                continue
            rows.append(row)
    receipt["objects"] = len(rows)
    receipt["status"] = (
        "partial" if receipt["invalid_lines"] else ("ok" if rows else "empty")
    )
    return rows, receipt


def period_from(rows, key, explicit=None):
    if number(explicit) and explicit > 0:
        return float(explicit), "result_metadata"
    stamps = [r[key] for r in rows if number(r.get(key))]
    gaps = [b - a for a, b in itertools.pairwise(stamps) if b > a]
    return (
        (statistics.median(gaps), "median_recorded_interval")
        if gaps
        else (None, "unavailable")
    )


def ordered(rows, time_key, validate):
    accepted, invalid, previous = [], 0, -math.inf
    for row in rows:
        stamp = row.get(time_key)
        if not number(stamp) or stamp <= previous or not validate(row):
            invalid += 1
            continue
        previous = stamp
        accepted.append(row)
    return accepted, invalid


def command_intervals(rows, period):
    intervals, gaps = [], 0
    for index, row in enumerate(rows):
        start = row["measured_time"]
        next_time = rows[index + 1]["measured_time"] if index + 1 < len(rows) else None
        if next_time is None:
            end = start + period if period is not None else start
        elif period is not None and next_time - start > 1.5 * period:
            end = start + period
            gaps += 1
        else:
            end = next_time
        if end > start and any(x != 0 for x in row["actual_command"]):
            if intervals and abs(intervals[-1][1] - start) < 1e-9:
                intervals[-1][1] = end
            else:
                intervals.append([start, end])
    return intervals, gaps


def weighted_stats(samples):
    if not samples:
        return {"average_mps": None, "p95_mps": None, "max_mps": None}
    duration = sum(weight for _, weight in samples)
    cumulative, p95 = 0.0, None
    for value, weight in sorted(samples):
        cumulative += weight
        if cumulative >= 0.95 * duration:
            p95 = value
            break
    return {
        "average_mps": sum(value * weight for value, weight in samples) / duration,
        "p95_mps": p95,
        "max_mps": max(value for value, _ in samples),
    }


def speed_metrics(truth, commands, result):
    truth, invalid_truth = ordered(truth, "time", lambda r: vector(r.get("velocity")))
    commands, invalid_commands = ordered(
        commands, "measured_time", lambda r: vector(r.get("actual_command"))
    )
    policy = result.get("policy") if isinstance(result.get("policy"), dict) else {}
    hz = policy.get("policy_hz")
    command_period, command_period_source = period_from(
        commands, "measured_time", 1 / hz if number(hz) and hz > 0 else None
    )
    physics_period, physics_period_source = period_from(
        truth, "time", result.get("physics_dt")
    )
    intervals, command_gaps = command_intervals(commands, command_period)
    interval_ends = [end for _, end in intervals]
    xy_samples, xyz_samples, covered, truth_gaps = [], [], 0.0, 0
    previous = None
    for row in truth:
        end = row["time"]
        if previous is None:
            start = end - physics_period if physics_period is not None else end
        else:
            start = previous
            if physics_period is not None and end - previous > 1.5 * physics_period:
                start = end - physics_period
                truth_gaps += 1
        previous = end
        index = bisect.bisect_right(interval_ends, start)
        weight = 0.0
        while index < len(intervals) and intervals[index][0] < end:
            left, right = intervals[index]
            weight += max(0.0, min(end, right) - max(start, left))
            index += 1
        if weight <= 0:
            continue
        velocity = row["velocity"]
        xy_samples.append((math.hypot(*velocity[:2]), weight))
        xyz_samples.append((math.sqrt(sum(x * x for x in velocity)), weight))
        covered += weight
    commanded = sum(end - start for start, end in intervals)
    return {
        "selection": "nonzero actual_command, held until next policy sample; gaps capped at one expected period",
        "weighting": "post-step native velocity over preceding physics step; intersect command intervals",
        "percentile": "duration-weighted nearest-rank",
        "command_period_s": command_period,
        "command_period_source": command_period_source,
        "physics_period_s": physics_period,
        "physics_period_source": physics_period_source,
        "command_records": len(commands),
        "invalid_command_records": invalid_commands,
        "native_velocity_records": len(truth),
        "invalid_native_velocity_records": invalid_truth,
        "command_gap_count": command_gaps,
        "native_gap_count": truth_gaps,
        "nonzero_command_intervals_s": intervals,
        "nonzero_command_duration_s": commanded if commands else None,
        "measured_duration_s": covered if truth and commands else None,
        "duration_coverage_fraction": min(1.0, covered / commanded)
        if commanded > 0
        else None,
        "status": "measured"
        if xy_samples
        else (
            "no_nonzero_commands"
            if commands and not intervals
            else "insufficient_evidence"
        ),
        "xy": weighted_stats(xy_samples),
        "three_dimensional": weighted_stats(xyz_samples),
    }


def native_displacement(rows):
    points, invalid = ordered(rows, "time", lambda r: vector(r.get("position")))
    return {
        "source": "simulator native pose; entire available truth stream",
        "position_records": len(points),
        "invalid_position_records": invalid,
        "start_xyz_m": points[0]["position"] if points else None,
        "end_xyz_m": points[-1]["position"] if points else None,
        "delta_z_m": points[-1]["position"][2] - points[0]["position"][2]
        if len(points) >= 2
        else None,
        "observed_duration_s": points[-1]["time"] - points[0]["time"]
        if len(points) >= 2
        else None,
    }


def mapping_metrics(rows, low_threshold):
    valid = [
        r
        for r in rows
        if isinstance(r.get("navigation"), int)
        and not isinstance(r["navigation"], bool)
        and r["navigation"] >= 0
    ]
    count = len(valid)
    return {
        "frames_with_valid_point_count": count,
        "invalid_point_count_records": len(rows) - count,
        "empty_rgbd_frames": sum(r["navigation"] == 0 for r in valid)
        if count
        else None,
        "low_count_rgbd_frames_including_empty": sum(
            r["navigation"] <= low_threshold for r in valid
        )
        if count
        else None,
        "low_nonempty_rgbd_frames": sum(
            0 < r["navigation"] <= low_threshold for r in valid
        )
        if count
        else None,
        "low_count_threshold_points": low_threshold,
        "keyframes_recorded": sum(r.get("keyframe") is True for r in rows),
        "full_scene_coverage_verified": False,
        "coverage_note": "Frame counts do not establish office or multi-floor spatial coverage",
    }


def navigation_metrics(result):
    tracker = (
        result.get("route_tracker")
        if isinstance(result.get("route_tracker"), dict)
        else {}
    )
    terminal = (
        result.get("navigation_terminal_request")
        if isinstance(result.get("navigation_terminal_request"), dict)
        else {}
    )
    reported = {
        name: value
        for name, value in [
            ("route_tracker", tracker.get("arrived")),
            ("terminal_request", terminal.get("arrived")),
        ]
        if isinstance(value, bool)
    }
    end_reason = result.get("end_reason")
    return {
        "arrival_reported": reported.get(
            "route_tracker", reported.get("terminal_request")
        ),
        "arrival_evidence": reported,
        "arrival_evidence_conflict": len(set(reported.values())) > 1,
        "tracker_phase": tracker.get("phase"),
        "tracker_failure": tracker.get("failure"),
        "terminal_request_failure": terminal.get("failure"),
        "source_terminal_reason": result.get("source_terminal_reason"),
        "end_reason": end_reason,
        "stopped_and_parked_reported": end_reason == "navigation_terminal_and_parked"
        if end_reason is not None
        else None,
        "capture_stage": result.get("stage"),
        "execution_error": result.get("error"),
        "process_exit_is_success_criterion": False,
        "mission_success_verified": False,
    }


def actuator_metrics(result):
    state = result.get("actuator_failure_state")
    if not isinstance(state, dict):
        return {
            "captured": False,
            "note": "actuator_failure_state was not captured; joint speed at failure is unknown",
        }
    qd = state.get("qd_isaac")
    names = state.get("dof_names")
    valid = isinstance(qd, list) and bool(qd) and all(number(x) for x in qd)
    named = isinstance(names, list) and valid and len(names) == len(qd)
    return {
        "captured": True,
        "physics_s": state.get("physics_s"),
        "joint_speed_records_valid": valid,
        "joint_speed_radps": qd if valid else None,
        "dof_names": names if named else None,
        "maximum_absolute_joint_speed_radps": max(abs(x) for x in qd)
        if valid
        else None,
        "capture_envelope_limit_radps": 30.0,
        "joints_above_capture_envelope": [
            names[i] if named else i for i, x in enumerate(qd) if abs(x) > 30
        ]
        if valid
        else None,
        "joint_speed_failure_reported": "Joint speed exceeds Go2HV motor envelope"
        in str(result.get("error", "")),
    }


def summarize(run_directory, low_count_threshold=1000):
    run = Path(run_directory).expanduser().resolve()
    if not run.is_dir():
        raise ValueError(f"Run directory does not exist: {run}")
    if (
        not isinstance(low_count_threshold, int)
        or isinstance(low_count_threshold, bool)
        or low_count_threshold < 0
    ):
        raise ValueError("Low-count threshold must be a nonnegative integer")
    result_path = run / "result.json"
    try:
        result = json.loads(result_path.read_text())
        if not isinstance(result, dict):
            raise ValueError("Expected result object")  # noqa: TRY004 - Keep the established ValueError result-validation contract.
        result_status = "ok"
    except (OSError, ValueError) as error:
        result = {}
        result_status = "missing" if not result_path.exists() else "invalid"
        result_error = str(error)
    truth, truth_receipt = read_stream(run / "truth_scoring_stream.jsonl")
    commands, command_receipt = read_stream(run / "policy_frames.jsonl")
    frames, frame_receipt = read_stream(run / "mapping/frames.jsonl")
    mapping = result.get("mapping") if isinstance(result.get("mapping"), dict) else {}
    experiment = (
        result.get("experiment") if isinstance(result.get("experiment"), dict) else {}
    )
    pose = mapping.get("pose_source", experiment.get("mapping_pose_source"))
    mask = mapping.get("mask_source", experiment.get("semantic_provider"))
    return {
        "schema": "dynanav-experiment-summary/v1",
        "run_directory": str(run),
        "evidence": {
            "result": {
                "path": str(result_path),
                "status": result_status,
                **({"error": result_error} if result_status != "ok" else {}),
            },
            "truth": truth_receipt,
            "commands": command_receipt,
            "mapping_frames": frame_receipt,
        },
        "provenance": {
            "velocity_and_displacement_source": "simulator_native_truth",
            "ground_truth_navigation": result.get("ground_truth_navigation"),
            "mapping_pose_source": pose,
            "mapping_mask_source": mask,
            "mapping_pose_source_evidence": "result.mapping"
            if "pose_source" in mapping
            else "experiment_configuration_only",
            "mapping_mask_source_evidence": "result.mapping"
            if "mask_source" in mapping
            else "experiment_configuration_only",
            "mapping_pose_oracle": "oracle" in pose if isinstance(pose, str) else None,
            "mapping_mask_oracle": "oracle" in mask if isinstance(mask, str) else None,
            "learned_perception": mapping.get("learned_perception"),
            "full_scene_coverage_verified": False,
        },
        "navigation": navigation_metrics(result),
        "commanded_motion": speed_metrics(truth, commands, result),
        "native_displacement": native_displacement(truth),
        "rgbd": mapping_metrics(frames, low_count_threshold),
        "actuator_failure": actuator_metrics(result),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "run_directory",
        type=Path,
        help="Capture directory containing result.json and JSONL journals",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="New JSON file; existing files are never overwritten. Default: stdout",
    )
    parser.add_argument("--low-count-threshold", type=int, default=1000)
    args = parser.parse_args(argv)
    try:
        summary = summarize(args.run_directory, args.low_count_threshold)
        serialized = (
            json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        )
        if args.output is None:
            sys.stdout.write(serialized)
        else:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(serialized)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
