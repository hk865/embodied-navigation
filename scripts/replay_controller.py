#!/usr/bin/env python3
"""Check the extracted driving law against an actual saved navigation run.

Frame selection uses logged target/heading/distance, never the recorded command
being tested. The simple timestamp-only join is reported separately because ROS
clock delivery and odometry callbacks can be ordered differently.
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from embodied_navigation.control.relative_pid import follow_segment, wrap_angle


def replay(document, tolerance=1e-10):
    poses = document["poses"]
    times = np.array([pose["time"] for pose in poses])
    route = np.asarray(document["route"])
    counts = dict(
        samples=0, unique_frame_matches=0, missing_frame=0, ambiguous_frame=0, command_mismatches=0
    )
    maximum_error = 0.0
    timestamp_errors = []

    def predict(pose, stage):
        rotation = np.asarray(pose["R"])
        return follow_segment(
            np.asarray(pose["body"])[:2],
            rotation[:2, :2],
            np.asarray(pose["velocity"])[:2],
            float(np.arctan2(rotation[1, 0], rotation[0, 0])),
            route[stage],
            route[stage + 1],
        )

    for command in document["commands"]:
        if command["reason"] != "relative_SLAM_multistage_PID":
            continue
        counts["samples"] += 1
        stage = command["stage"]
        before = np.searchsorted(times, command["time"], side="right") - 1
        if before >= 0:
            naive, _ = predict(poses[before], stage)
            timestamp_errors.append(float(np.max(np.abs(naive - command["request"]))))
        begin, end = np.searchsorted(times, [command["time"] - 0.25, command["time"] + 0.25])
        matches = []
        for index in range(begin, min(len(poses), end + 1)):
            new, diagnostic = predict(poses[index], stage)
            distance = np.linalg.norm(np.asarray(poses[index]["body"])[:2] - route[stage + 1])
            if (
                np.max(np.abs(np.asarray(diagnostic["target"]) - command["target"])) < tolerance
                and abs(wrap_angle(diagnostic["heading_error_rad"] - command["heading_error_rad"]))
                < tolerance
                and abs(distance - command["distance_to_waypoint_m"]) < tolerance
            ):
                matches.append(new)
        if not matches:
            counts["missing_frame"] += 1
            continue
        if len(matches) != 1:
            counts["ambiguous_frame"] += 1
            continue
        counts["unique_frame_matches"] += 1
        error = float(np.max(np.abs(matches[0] - command["request"])))
        maximum_error = max(maximum_error, error)
        counts["command_mismatches"] += error > tolerance
    return dict(
        **counts,
        max_command_difference=maximum_error,
        tolerance=tolerance,
        frame_join="unique logged lookahead target + heading error + waypoint distance; not request matching",
        timestamp_only_join=dict(
            samples=len(timestamp_errors),
            exact_samples=sum(error < tolerance for error in timestamp_errors),
            max_difference=max(timestamp_errors) if timestamp_errors else None,
        ),
        no_simulator_or_robot_started=True,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = replay(json.loads(args.result.read_text()))
    result["input_sha256"] = hashlib.sha256(args.result.read_bytes()).hexdigest()
    text = json.dumps(result, indent=2)
    print(text)
    if args.output:
        args.output.write_text(text + "\n")
    return int(
        any(result[key] for key in ("missing_frame", "ambiguous_frame", "command_mismatches"))
    )


if __name__ == "__main__":
    raise SystemExit(main())
