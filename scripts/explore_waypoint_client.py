#!/usr/bin/env python3
"""Operator-side client for the simulation-only interactive waypoint explorer.

The current static exploration entry uses NativeWaypointTracker and the
existing RouteTracker, with goto/cancel/stop. In that mode the legacy ttl_s
field is transport metadata, not an action deadline; no heartbeat or
perception/floor admission is required. yaw/maneuver/scan belong to the older
interactive controller and are not supported by the native-route adapter.

This is the DSH operator tool, not a controller. It publishes waypoints into the
run mailbox, keeps the operator heartbeat alive, reads the observation bundle and
receipts, and can build a coarse operator-side occupancy snapshot from the live
measured topics. It never writes a velocity command, never imports Isaac and
never asks the simulator for ground truth.

Typical loop (one waypoint per decision, local follower does the 50 Hz job):

    python3 scripts/explore_waypoint_client.py heartbeat --run-dir RUN --loop --duration 600
    python3 scripts/explore_waypoint_client.py goto --run-dir RUN --x 3.0 --y -1.5 --z -3.3 --ttl 30
    python3 scripts/explore_waypoint_client.py observe --run-dir RUN
    python3 scripts/explore_waypoint_client.py map --run-dir RUN --odometry-topic TOPIC --pose-source lio_estimate
    python3 scripts/explore_waypoint_client.py stop --run-dir RUN

`goto`, `yaw` and `maneuver` accept `--wait` (with `--timeout`) to block on
one structured action result; `scan` waits per step. A rejected,
cancelled, dropped, expired or held action and a global terminal receipt end
the wait immediately instead of being reported as still running, so an upper
layer never re-sends an action the tracker already refused.

Pose source of the map snapshot is recorded in its JSON: it is the measured
estimator odometry plus live depth, never a simulator pose and never the offline
ground-truth geometry.

Exit codes (`--wait` and `scan`, returned by `main`):

    0  the action completed (yaw_complete, maneuver_complete, waypoint_reached)
    1  the action was rejected, cancelled, dropped, expired or held, or the run
       latched a terminal failure; the structured result is printed to stdout
    2  no closing receipt arrived within the timeout: the action is still
       pending, so it must not be re-sent blindly
    3  the request was refused locally (invalid argument, missing mailbox,
       unavailable ROS); the error JSON is printed to stdout
"""

from __future__ import annotations

import argparse
import json
import math
import os

import numpy as np
import time
from pathlib import Path

PROTOCOL_VERSION = 1
DEFAULT_MAILBOX_NAME = "interactive"
# Mirrors the simulator-side gate: a ground-truth frame can never be a waypoint
# frame. Checked here as well so a mistaken client call fails before the file is
# written, not after the simulator has to reject it.
FORBIDDEN_POSE_SOURCES = (
    "native_truth",
    "native_pose",
    "simulator_native_pose",
    "ground_truth",
    "gt_diagnostic",
)
OCCUPIED = 1
FREE = 0

# Exit codes of the waiting CLI paths (--wait and scan). The timeout code is
# deliberately distinct from a failure so an upper layer can retry a pending
# action once, but can never re-send an action the tracker already rejected.
EXIT_OK = 0
EXIT_ACTION_FAILED = 1
EXIT_ACTION_TIMEOUT = 2
EXIT_USAGE = 3

# A /clock sample older than this wall age is not the current source clock: the
# snapshot must not pretend a now it does not have. 0.25 s matches the map
# freshness window, so a stopped clock degrades to the recorded-arrival path.
MAP_CLOCK_MAX_WALL_AGE_S = 0.25

EXIT_CODE_HELP = """exit codes:
  0  action completed (yaw_complete, maneuver_complete, waypoint_reached)
  1  action rejected/cancelled/dropped/expired/held, or the run latched a terminal
  2  no closing receipt within the timeout (still pending; do not re-send blindly)
  3  request refused locally (invalid argument, missing mailbox, unavailable ROS)
"""

# Recipient events that close one action successfully; each names the action seq.
ACTION_COMPLETION_EVENTS = ("yaw_complete", "maneuver_complete", "waypoint_reached")
# Receipt events that close one action unsuccessfully, keyed to their status.
# The tracker writes request_rejected for a refused request, action_cancelled
# for a local action it aborts, waypoint_dropped/waypoint_expired for follower
# failures and action_expired for a local action whose TTL elapsed.
ACTION_FAILURE_STATUS = {
    "request_rejected": "rejected",
    "maneuver_rejected": "rejected",
    "waypoint_rejected": "rejected",
    "yaw_rejected": "rejected",
    "waypoint_dropped": "dropped",
    "waypoint_failed": "failed",
    "waypoint_expired": "expired",
    "action_expired": "expired",
    "action_cancelled": "cancelled",
}
# Receipts that concern the whole run rather than one sequence number: the
# tracker writes "terminal" (latched failure, no seq) and "hold" (blocked tick,
# no seq).
GLOBAL_ACTION_STATUS = {"terminal": "terminal", "hold": "held"}


def mailbox_for(run_dir, mailbox=None):
    """Resolve the mailbox directory for one run."""
    return Path(mailbox) if mailbox else Path(run_dir) / DEFAULT_MAILBOX_NAME


def atomic_write_json(path, payload):
    """Write JSON through a same-directory replace so the simulator never reads half a file."""
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(payload, sort_keys=True, separators=(",", ":")))
    os.replace(temporary, path)


def read_json(path, default=None):
    """Read a JSON file, returning default when it does not exist or is invalid."""
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return default


def run_id_for(mailbox):
    """Run id used by the simulator for this mailbox (the run directory name)."""
    return Path(mailbox).parent.name


def state_path_for(mailbox):
    """Operator-owned sequence state, kept inside the mailbox for auditability."""
    return Path(mailbox) / "operator-state.json"


def next_sequence(mailbox):
    """Return the next strictly increasing request sequence number."""
    state = read_json(state_path_for(mailbox), {}) or {}
    last = state.get("last_sequence")
    if isinstance(last, bool) or not isinstance(last, int) or last < 0:
        last = 0
    return last + 1


def _number(value, name):
    """Return value as a finite float or raise."""
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(name + " must be a finite number") from error
    if not math.isfinite(number):
        raise ValueError(name + " must be a finite number")
    return number


def build_request(
    *,
    seq,
    action,
    goal=None,
    ttl_s=30.0,
    pose_source,
    run_id,
    created_wall_s,
    extra=None,
):
    """Build one request payload, validating it before it reaches the mailbox."""
    if action not in ("goto", "waypoint", "cancel", "stop", "yaw", "maneuver"):
        raise ValueError("unsupported action")
    if isinstance(seq, bool) or not isinstance(seq, int) or seq <= 0:
        raise ValueError("seq must be a positive integer")
    if not math.isfinite(ttl_s) or not 1.0 <= ttl_s <= 120.0:
        raise ValueError("ttl_s must be within [1, 120] s")
    payload = {
        "version": PROTOCOL_VERSION,
        "seq": seq,
        "action": action,
        "created_wall_s": float(created_wall_s),
        "ttl_s": float(ttl_s),
        "run_id": run_id,
    }
    extra = {} if extra is None else dict(extra)
    if action in ("goto", "waypoint", "yaw", "maneuver"):
        if pose_source in FORBIDDEN_POSE_SOURCES:
            raise ValueError(
                "a ground-truth waypoint frame is forbidden; the simulator rejects it too"
            )
    if action == "yaw":
        delta = _number(extra.get("delta_yaw_rad"), "delta_yaw_rad")
        if abs(delta) > 3.15:
            raise ValueError("delta_yaw_rad must be within +/-3.15 rad")
        payload["delta_yaw_rad"] = delta
        payload["pose_source"] = pose_source
        payload["map_frame_note"] = "live estimator map frame; operator-selected"
        return payload
    if action == "maneuver":
        forward = _number(extra.get("body_forward_mps", 0.0), "body_forward_mps")
        lateral = _number(extra.get("body_lateral_mps", 0.0), "body_lateral_mps")
        yaw_rate = _number(extra.get("yaw_rate_rps", 0.0), "yaw_rate_rps")
        duration = _number(extra.get("duration_s"), "duration_s")
        if abs(forward) > 1.0:
            raise ValueError("body_forward_mps must not exceed the 1.0 m/s ceiling")
        if abs(lateral) > 0.05:
            raise ValueError("body_lateral_mps must be within +/-0.05 m/s")
        if abs(yaw_rate) > 0.15:
            raise ValueError("yaw_rate_rps must be within +/-0.15 rad/s")
        if not 0.1 <= duration <= 2.0:
            raise ValueError("duration_s must be within [0.1, 2.0] s")
        payload["body_forward_mps"] = forward
        payload["body_lateral_mps"] = lateral
        payload["yaw_rate_rps"] = yaw_rate
        payload["duration_s"] = duration
        payload["pose_source"] = pose_source
        payload["map_frame_note"] = "body frame of the live estimator pose"
        return payload
    if action in ("goto", "waypoint"):
        if goal is None or len(goal) != 3:
            raise ValueError("goal must be an XYZ triple")
        xyz = [float(value) for value in goal]
        if not all(math.isfinite(value) for value in xyz):
            raise ValueError("goal must be three finite numbers")
        payload["goal_xyz_m"] = xyz
        payload["pose_source"] = pose_source
        payload["map_frame_note"] = "live estimator map frame; operator-selected"
    return payload


def submit(
    mailbox, *, action, goal=None, ttl_s=30.0, pose_source=None, run_id=None, extra=None
):
    """Publish one request and advance the operator sequence state."""
    mailbox = Path(mailbox)
    if not mailbox.is_dir():
        raise FileNotFoundError(f"mailbox does not exist: {mailbox}")
    seq = next_sequence(mailbox)
    payload = build_request(
        seq=seq,
        action=action,
        goal=goal,
        ttl_s=ttl_s,
        pose_source=pose_source,
        run_id=run_id if run_id is not None else run_id_for(mailbox),
        created_wall_s=time.perf_counter(),
        extra=extra,
    )
    atomic_write_json(mailbox / "request.json", payload)
    atomic_write_json(
        state_path_for(mailbox),
        {
            "last_sequence": seq,
            "last_action": action,
            "submitted_wall_s": payload["created_wall_s"],
        },
    )
    return payload


def write_heartbeat(mailbox, *, run_id=None):
    """Refresh the operator liveness file the simulator enforces."""
    mailbox = Path(mailbox)
    payload = {
        "wall_s": time.perf_counter(),
        "run_id": run_id if run_id is not None else run_id_for(mailbox),
        "client": "dsh-explore-waypoint-client",
    }
    atomic_write_json(mailbox / "heartbeat.json", payload)
    return payload


def latest_sim_frame(run_dir):
    """Describe the newest low-rate camera still written by the simulation."""
    directory = Path(run_dir) / "interactive_frames"
    try:
        images = sorted(directory.glob("rgb_*.png"))
    except OSError:
        return None
    if not images:
        return None
    index = directory / "index.jsonl"
    return {
        "latest": str(images[-1]),
        "count": len(images),
        "index": str(index) if index.exists() else None,
        "source": "rgb frame rendered by this simulation",
        "pose_included": False,
    }


def _receipt_run_matches(receipt, run_id):
    """Whether a receipt belongs to this run; an unknown run id is accepted."""
    declared = receipt.get("run_id")
    if not isinstance(run_id, str) or not run_id:
        # No trustworthy run id to compare against (mailbox outside a run dir).
        return True
    if declared is None:
        return True
    return isinstance(declared, str) and declared == run_id


def _same_sequence(receipt, seq):
    """Whether a receipt names exactly this action's sequence number."""
    value = receipt.get("seq")
    return isinstance(value, int) and not isinstance(value, bool) and value == seq


def _action_result(receipt, status, seq, *, success, run_id):
    """Return one structured action result: the receipt fields plus the verdict."""
    result = dict(receipt)
    declared_run = receipt.get("run_id")
    result["run_id"] = (
        declared_run if isinstance(declared_run, str) and declared_run else run_id
    )
    result["seq"] = seq
    result["status"] = status
    result["success"] = bool(success)
    result["reason"] = receipt.get("reason")
    return result


def _match_action_result(receipts, seq, run_id):
    """Return this sequence's verdict from one receipt tail, or None.

    A receipt is this action's result only when it names this run and this
    sequence number, so a receipt from another run or for an older/foreign seq
    is never mistaken for it. A terminal receipt is global (the tracker writes
    it without a seq) and always ends the wait. A hold carries no seq either, so
    it only blocks this action once the action was accepted and the hold is
    newer than that acceptance: a startup hold written before the request was
    consumed must not fail a request that is still being accepted.
    """
    last_seq_index = -1
    for index, receipt in enumerate(receipts):
        if _receipt_run_matches(receipt, run_id) and _same_sequence(receipt, seq):
            last_seq_index = index
    for index, receipt in enumerate(receipts):
        if not _receipt_run_matches(receipt, run_id):
            continue
        event = receipt.get("event")
        if not isinstance(event, str) or not event:
            continue
        if event == "terminal":
            return _action_result(
                receipt, GLOBAL_ACTION_STATUS["terminal"], seq, success=False, run_id=run_id
            )
        if _same_sequence(receipt, seq):
            if event in ACTION_COMPLETION_EVENTS:
                return _action_result(
                    receipt, "completed", seq, success=True, run_id=run_id
                )
            status = ACTION_FAILURE_STATUS.get(event)
            if status is None and event.endswith("_rejected"):
                status = "rejected"
            if status is not None:
                return _action_result(receipt, status, seq, success=False, run_id=run_id)
            continue
        if event == "hold" and last_seq_index >= 0 and index > last_seq_index:
            return _action_result(
                receipt, GLOBAL_ACTION_STATUS["hold"], seq, success=False, run_id=run_id
            )
    return None


def timeout_result(seq, timeout_s, run_id):
    """Return the structured result of a wait that expired without a receipt."""
    return {
        "event": "timeout",
        "status": "timeout",
        "success": False,
        "reason": "timeout",
        "seq": seq,
        "run_id": run_id,
        "timeout_s": timeout_s,
    }


def wait_for_action(mailbox, seq, timeout_s, poll_s=0.05):
    """Wait for the structured result of one local action.

    Returns immediately, before the timeout, when this seq is rejected or
    cancelled (request_rejected, maneuver_rejected, waypoint_failed, waypoint_dropped,
    waypoint_expired, action_expired, action_cancelled) or when the run latches
    a global terminal receipt. The result carries the receipt's own fields plus
    status (completed/rejected/failed/cancelled/dropped/expired/held/terminal/timeout),
    success, reason, seq and run_id, so a rejected or cancelled action can never
    be reported as still running.

    Receipts from another run or for a different sequence number are ignored,
    and a missing or corrupt receipts.jsonl simply means "no receipt yet" and
    never raises. An expired wait returns status timeout (not a failure), which
    the CLI maps to exit code 2 instead of 1.
    """
    if timeout_s <= 0:
        raise ValueError("timeout_s must be positive")
    if isinstance(seq, bool) or not isinstance(seq, int) or seq <= 0:
        raise ValueError("seq must be a positive integer")
    run_id = run_id_for(mailbox)
    deadline = time.perf_counter() + timeout_s
    while True:
        result = _match_action_result(tail_receipts(mailbox, 40), seq, run_id)
        if result is not None:
            return result
        if time.perf_counter() >= deadline:
            return timeout_result(seq, timeout_s, run_id)
        time.sleep(poll_s)


def action_exit_code(result):
    """Map one wait_for_action result to the documented CLI exit code."""
    if result.get("success"):
        return EXIT_OK
    if result.get("status") == "timeout":
        return EXIT_ACTION_TIMEOUT
    return EXIT_ACTION_FAILED


def read_observation(mailbox):
    """Return the latest observation bundle or None."""
    return read_json(Path(mailbox) / "observation.json")


def tail_receipts(mailbox, count=8):
    """Return the last count receipts as parsed objects; corruption is skipped."""
    path = Path(mailbox) / "receipts.jsonl"
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError):
        return []
    result = []
    for line in lines[-count:]:
        line = line.strip()
        if not line:
            continue
        try:
            parsed = json.loads(line)
        except json.JSONDecodeError:
            continue
        # A receipt is a JSON object. A truncated or foreign writer can leave a
        # bare scalar or list, which must never break the wait or the observe
        # output, so only objects are treated as receipts.
        if isinstance(parsed, dict):
            result.append(parsed)
    return result


def summarize_observation(observation):
    """Return a compact operator-facing summary of one observation bundle."""
    if not observation:
        return {"available": False}
    pose = observation.get("pose", {})
    waypoints = observation.get("waypoints", {})
    speed = observation.get("speed", {})
    link = observation.get("link", {})
    return {
        "available": True,
        "run_id": observation.get("run_id"),
        "sim_s": observation.get("sim_s"),
        "pose": {
            "source": pose.get("source"),
            "frame": pose.get("map_frame"),
            "position_xyz_m": pose.get("position_xyz_m"),
            "yaw_rad": pose.get("yaw_rad"),
            "sim_age_s": pose.get("sim_age_s"),
            "wall_age_s": pose.get("wall_age_s"),
            "fresh": pose.get("fresh"),
        },
        "measured_speed_mps": speed.get("pose_window_mps")
        if speed.get("pose_window_mps") is not None
        else speed.get("reported_twist_mps"),
        "command_ceiling_mps": speed.get("command_ceiling_mps"),
        "waypoints": {
            "active_seq": waypoints.get("active_seq"),
            "active_goal_xyz_m": waypoints.get("active_goal_xyz_m"),
            "queued": waypoints.get("queued"),
            "phase": waypoints.get("phase"),
            "distance_to_goal_m": waypoints.get("distance_to_goal_m"),
            "z_error_m": waypoints.get("z_error_m"),
            "completed_goals": waypoints.get("completed_goals"),
            "failed_goals": waypoints.get("failed_goals"),
            "last_failure": waypoints.get("last_failure"),
            "last_result": waypoints.get("last_result"),
            "terminal_reason": waypoints.get("terminal_reason"),
        },
        "last_command": observation.get("command", {}).get("last_request"),
        "link": link,
        "coverage": observation.get("coverage"),
        "map": observation.get("map", {}).get("provider"),
        "native_guards": observation.get("guards", {}).get("native"),
    }


class OccupancyGrid:
    """Offline geometry helper kept for analytic tests only.

    The map command no longer uses this class: it accumulates the live SLAM
    point cloud through dynanav.operator_map with an explicit extrinsic and
    stamp-paired measured pose. This class survives only as a small testable
    projection helper, and its output must not be used to choose waypoints.
    """

    def __init__(
        self,
        resolution_m=0.10,
        pose_floor_tolerance_m=0.12,
        min_obstacle_height_m=0.15,
        max_obstacle_height_m=2.2,
        max_range_m=12.0,
    ):
        if not 0.02 <= resolution_m <= 0.5:
            raise ValueError("resolution_m must be within [0.02, 0.5]")
        self.resolution = float(resolution_m)
        self.pose_floor_tolerance_m = float(pose_floor_tolerance_m)
        self.min_obstacle_height_m = float(min_obstacle_height_m)
        self.max_obstacle_height_m = float(max_obstacle_height_m)
        self.max_range_m = float(max_range_m)
        self.cells = {}
        self.pose_trail = []
        self.scans = 0
        self.points = 0

    def _index(self, x, y):
        return (
            int(math.floor(x / self.resolution)),
            int(math.floor(y / self.resolution)),
        )

    def add_pose(self, position_xyz):
        """Append one measured pose to the coverage trail."""
        self.pose_trail.append([float(value) for value in position_xyz])

    def add_points(self, points_xyz, pose_xyz, yaw_rad):
        """Accumulate one map-frame point batch observed from one measured pose."""
        px, py, pz = (float(value) for value in pose_xyz)
        self.scans += 1
        for point in points_xyz:
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if math.hypot(x - px, y - py) > self.max_range_m:
                continue
            self.points += 1
            dz = z - pz
            if dz < -self.pose_floor_tolerance_m:
                continue
            index = self._index(x, y)
            if abs(dz) <= self.pose_floor_tolerance_m:
                self.cells.setdefault(index, FREE)
            elif self.min_obstacle_height_m <= dz <= self.max_obstacle_height_m:
                self.cells[index] = OCCUPIED

    def frontier(self, limit=200, pose_xyz=None):
        """Return observed-free cells that touch a never-observed cell."""
        result = []
        for (ix, iy), code in self.cells.items():
            if code != FREE:
                continue
            touches_unknown = False
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    if (ix + dx, iy + dy) not in self.cells:
                        touches_unknown = True
                        break
                if touches_unknown:
                    break
            if touches_unknown:
                result.append((ix, iy))
        if pose_xyz is not None:
            px, py = float(pose_xyz[0]), float(pose_xyz[1])
            result.sort(
                key=lambda cell: math.hypot(
                    (cell[0] + 0.5) * self.resolution - px,
                    (cell[1] + 0.5) * self.resolution - py,
                )
            )
        cells = [
            {
                "xy_m": [
                    round((cell[0] + 0.5) * self.resolution, 3),
                    round((cell[1] + 0.5) * self.resolution, 3),
                ]
            }
            for cell in result[:limit]
        ]
        return {"cell_count": len(result), "cells": cells}

    def to_dict(self, pose_xyz=None):
        """Return the JSON receipt for this snapshot."""
        free_cells = [cell for cell, code in self.cells.items() if code == FREE]
        occupied_cells = [cell for cell, code in self.cells.items() if code == OCCUPIED]
        xs = [cell[0] for cell in self.cells] or [0]
        ys = [cell[1] for cell in self.cells] or [0]
        path_length = 0.0
        for previous, current in zip(self.pose_trail, self.pose_trail[1:]):
            path_length += math.hypot(
                current[0] - previous[0], current[1] - previous[1]
            )
        return {
            "kind": "operator_side_occupancy",
            "resolution_m": self.resolution,
            "scans": self.scans,
            "points": self.points,
            "free_cells": len(free_cells),
            "occupied_cells": len(occupied_cells),
            "observed_area_m2": round(len(self.cells) * self.resolution ** 2, 3),
            "free_area_m2": round(len(free_cells) * self.resolution ** 2, 3),
            "bounds_cells": {"min": [min(xs), min(ys)], "max": [max(xs), max(ys)]},
            "pose_samples": len(self.pose_trail),
            "measured_path_length_m": round(path_length, 3),
            "frontier": self.frontier(pose_xyz=pose_xyz),
            "map_source": "live_depth_points_transformed_by_measured_estimator_pose",
            "ground_truth_used": False,
            "note": (
                "operator aid only; not a base navigation input and not a globally "
                "registered map"
            ),
        }


def _cmd_heartbeat(args):
    """Write one heartbeat or keep beating for a bounded duration."""
    mailbox = mailbox_for(args.run_dir, args.mailbox)
    if not args.loop:
        payload = write_heartbeat(mailbox, run_id=args.run_id)
        print(json.dumps({"event": "heartbeat", "wall_s": payload["wall_s"]}))
        return 0
    deadline = time.perf_counter() + args.duration
    beats = 0
    while time.perf_counter() < deadline:
        write_heartbeat(mailbox, run_id=args.run_id)
        beats += 1
        time.sleep(max(0.05, args.interval))
    print(json.dumps({"event": "heartbeat_loop_finished", "beats": beats}))
    return 0


def _wait_for_submitted(args, payload):
    """Wait for one submitted action when --wait is given; return its exit code."""
    if not getattr(args, "wait", False):
        return EXIT_OK
    mailbox = mailbox_for(args.run_dir, args.mailbox)
    result = wait_for_action(mailbox, payload["seq"], args.timeout)
    print(json.dumps(result, sort_keys=True))
    return action_exit_code(result)


def _cmd_goto(args):
    """Submit one waypoint for the local follower."""
    mailbox = mailbox_for(args.run_dir, args.mailbox)
    payload = submit(
        mailbox,
        action="goto",
        goal=[args.x, args.y, args.z],
        ttl_s=args.ttl,
        pose_source=args.pose_source,
        run_id=args.run_id,
    )
    print(json.dumps(payload, sort_keys=True))
    return _wait_for_submitted(args, payload)


def _cmd_simple(action):
    """Build a cancel/stop handler."""

    def handler(args):
        mailbox = mailbox_for(args.run_dir, args.mailbox)
        payload = submit(mailbox, action=action, run_id=args.run_id)
        print(json.dumps(payload, sort_keys=True))
        return 0

    return handler


def _cmd_observe(args):
    """Print the observation summary and the receipt tail."""
    mailbox = mailbox_for(args.run_dir, args.mailbox)
    observation = read_observation(mailbox)
    summary = summarize_observation(observation)
    summary["receipts"] = tail_receipts(mailbox, args.receipts)
    summary["sim_frames"] = latest_sim_frame(mailbox.parent)
    if args.full and observation is not None:
        summary["observation"] = observation
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


def _cmd_yaw(args):
    """Submit one in-place scan turn (relative to the live measured yaw)."""
    mailbox = mailbox_for(args.run_dir, args.mailbox)
    payload = submit(
        mailbox,
        action="yaw",
        ttl_s=args.ttl,
        pose_source=args.pose_source,
        run_id=args.run_id,
        extra={"delta_yaw_rad": math.radians(args.delta_deg)},
    )
    print(json.dumps(payload, sort_keys=True))
    return _wait_for_submitted(args, payload)


def _cmd_maneuver(args):
    """Submit one short bounded body-frame local manoeuvre."""
    mailbox = mailbox_for(args.run_dir, args.mailbox)
    payload = submit(
        mailbox,
        action="maneuver",
        ttl_s=args.ttl,
        pose_source=args.pose_source,
        run_id=args.run_id,
        extra={
            "body_forward_mps": args.forward,
            "body_lateral_mps": args.lateral,
            "yaw_rate_rps": args.yaw_rate,
            "duration_s": args.duration,
        },
    )
    print(json.dumps(payload, sort_keys=True))
    return _wait_for_submitted(args, payload)


def _cmd_scan(args):
    """Turn in place in fixed increments and snapshot the observation bundle."""
    mailbox = mailbox_for(args.run_dir, args.mailbox)
    if not 1 <= args.steps <= 12:
        raise ValueError("steps must be within [1, 12]")
    if args.step_deg == 0.0 or abs(args.step_deg) > 180.0:
        raise ValueError("step_deg must be nonzero and within [-180, 180]")
    snapshots = Path(mailbox) / "scan"
    snapshots.mkdir(parents=True, exist_ok=True)
    results = []
    exit_code = EXIT_OK
    for index in range(args.steps):
        if index and args.settle_s > 0:
            time.sleep(args.settle_s)
        payload = submit(
            mailbox,
            action="yaw",
            ttl_s=args.ttl,
            pose_source=args.pose_source,
            run_id=args.run_id,
            extra={"delta_yaw_rad": math.radians(args.step_deg)},
        )
        outcome = wait_for_action(mailbox, payload["seq"], args.timeout_per_step)
        observation = read_observation(mailbox)
        if observation:
            atomic_write_json(snapshots / ("observation_%02d.json" % index), observation)
        terminal = (observation or {}).get("waypoints", {}).get("terminal_reason")
        results.append(
            {
                "step": index,
                "seq": payload["seq"],
                "status": outcome.get("status"),
                "success": outcome.get("success"),
                "outcome": outcome.get("event"),
                "reason": outcome.get("reason"),
                "yaw_error_rad": outcome.get("yaw_error_rad"),
                "terminal_reason": terminal,
            }
        )
        if not outcome.get("success"):
            # A rejected, cancelled or expired turn is a failure, and a pending
            # turn is a timeout: neither may be reported as a clean sweep.
            exit_code = action_exit_code(outcome)
        elif terminal is not None and exit_code == EXIT_OK:
            exit_code = EXIT_ACTION_FAILED
        if not outcome.get("success") or terminal is not None:
            break
    print(
        json.dumps(
            {
                "event": "scan_finished",
                "status": "completed" if exit_code == EXIT_OK else "incomplete",
                "success": exit_code == EXIT_OK,
                "exit_code": exit_code,
                "steps": results,
                "snapshots": str(snapshots),
                "sim_frames": latest_sim_frame(mailbox.parent),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return exit_code


def message_stamp_s(message):
    """Return one message header stamp in seconds."""
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


OPERATOR_CLOUD_MAX_POINTS = 100_000


def cloud_to_xyz(message):
    """Decode an entire bounded cloud. Oversize clouds are refused, not sampled."""
    import numpy as np

    fields = {field.name: field for field in message.fields}
    for name in ("x", "y", "z"):
        if name not in fields:
            raise ValueError("point cloud lacks field " + name)
        if fields[name].datatype != 7:
            raise ValueError("point cloud field " + name + " is not float32")
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [np.float32, np.float32, np.float32],
            "offsets": [fields["x"].offset, fields["y"].offset, fields["z"].offset],
            "itemsize": message.point_step,
        }
    )
    count = int(message.width) * int(message.height)
    if count > OPERATOR_CLOUD_MAX_POINTS:
        raise ValueError(
            f"operator cloud exceeds {OPERATOR_CLOUD_MAX_POINTS} points; "
            "refusing the whole cloud, not truncating or resampling"
        )
    if count <= 0:
        return np.zeros((0, 3), dtype=float)
    raw = np.frombuffer(message.data, dtype=dtype, count=count)
    xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(float)
    return xyz[np.isfinite(xyz).all(axis=1)]



def _native_header_stamp_ns(message):
    """Exact integer nanosecond header stamp (never a float round-trip)."""
    header = message.header
    seconds = int(header.stamp.sec)
    nanoseconds = int(header.stamp.nanosec)
    if seconds < 0 or nanoseconds < 0:
        raise ValueError("negative ROS header stamp")
    return seconds * 1000000000 + nanoseconds


def _store_lio_pose(state, message, map_pose, arrival_wall):
    """Keep the same-stage LIO pose keyed by its exact integer stamp."""
    stamp_ns = _native_header_stamp_ns(message)
    poses = state.setdefault("lio_poses", {})
    poses[stamp_ns] = {
        "stamp_ns": stamp_ns,
        "stamp_s": stamp_ns * 1e-9,
        "position": [
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        ],
        "quat_xyzw": [
            float(message.pose.pose.orientation.x),
            float(message.pose.pose.orientation.y),
            float(message.pose.pose.orientation.z),
            float(message.pose.pose.orientation.w),
        ],
        "map_pose": map_pose,
        "arrival_wall_s": float(arrival_wall),
        "frame_id": message.header.frame_id,
        "child_frame_id": message.child_frame_id,
    }
    if len(poses) > 512:
        for key in sorted(poses)[:-256]:
            del poses[key]


def _source_binary_identity(prefix):
    """Record the exact native binaries this consumer is reading."""
    if not prefix:
        return None
    import hashlib

    root = Path(prefix)
    identity = {"prefix": str(root)}
    for name, relative in (
        ("core_sha256", "install/ros/lib/libfast_livo2_core.so"),
        ("mapping_sha256", "install/ros/lib/fast_livo2_ros/fastlivo_mapping"),
    ):
        path = root / relative
        if path.is_file():
            identity[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return identity


def _deskewed_cloud_admission(message, arrival_wall, state, args, now_wall):
    """Admission for the native deskewed LIO full cloud: content plus exact pair.

    The cloud must carry the native PointXYZINormal layout (a raw rotating wire
    payload is refused by content, not by topic string), be labelled in the same
    odometry map frame as the paired pose, and pair with a pose of the identical
    integer stamp. A near stamp is a different pair and is never interpolated.
    """
    from embodied_navigation.simulation.dynanav.rtx_deskewed_perception import (
        DeskewedCloudError,
        RAW_WIRE_FIELDS,
        message_stamp_ns,
        state_age_gate,
        verify_cloud_layout,
        verify_same_stage_pair,
    )

    names = [field.name for field in message.fields]
    try:
        verify_cloud_layout(names)
    except DeskewedCloudError as error:
        reason = (
            "raw_wire_cloud_refused"
            if any(name in names for name in RAW_WIRE_FIELDS)
            else "cloud_payload_not_native"
        )
        return "reject", reason, {"error": str(error), "fields": names}
    declared = "" if message.header is None else str(message.header.frame_id)
    expected_map = state.get("odometry_map_frame")
    if not expected_map:
        return "reject", "odometry_map_frame_unknown", {"cloud_frame": declared}
    if declared != expected_map:
        return (
            "reject",
            "cloud_frame_mismatch",
            {"cloud_frame": declared, "odometry_map_frame": expected_map},
        )
    try:
        stamp_ns = message_stamp_ns(message)
    except DeskewedCloudError as error:
        return "reject", "cloud_stamp_invalid", {"error": str(error)}
    try:
        arrival = float(arrival_wall)
        now = float(now_wall)
    except (TypeError, ValueError):
        return "reject", "arrival_invalid", {"arrival_wall_s": arrival_wall}
    if not (math.isfinite(arrival) and math.isfinite(now)):
        return "reject", "arrival_invalid", {"arrival_wall_s": arrival, "now_wall_s": now}
    pose = (state.get("lio_poses") or {}).get(stamp_ns)
    if pose is None:
        wall_age = now - arrival
        if wall_age < min(CLOUD_PENDING_MAX_WALL_S, MAP_CLOUD_MAX_WALL_AGE_S):
            return (
                "wait_pose",
                "awaiting_exact_lio_pose",
                {"cloud_stamp_ns": stamp_ns, "wall_age_s": wall_age},
            )
        return (
            "reject",
            "pose_pair_missing_exact",
            {
                "cloud_stamp_ns": stamp_ns,
                "known_pose_stamps": len(state.get("lio_poses") or {}),
            },
        )
    try:
        pair = verify_same_stage_pair(
            cloud_frame=declared,
            pose_frame=pose["frame_id"],
            child_frame=pose["child_frame_id"],
            cloud_stamp_ns=stamp_ns,
            pose_stamp_ns=pose["stamp_ns"],
            expected_map_frame=expected_map,
            expected_child_frame=args.sensor_frame,
            stage=state.get("deskewed_stage"),
        )
    except DeskewedCloudError as error:
        return "reject", "pair_contract_violation", {"error": str(error)}
    now_sim, clock_note = _authoritative_sim_clock(state)
    if now_sim is None:
        return "reject", "sim_clock_unavailable", {"clock_note": clock_note}
    ok, reason, ages = state_age_gate(
        now_sim_s=now_sim,
        stamp_s=stamp_ns * 1e-9,
        arrival_wall_s=arrival,
        now_wall_s=now,
        max_sim_age_s=MAP_CLOUD_MAX_SIM_AGE_S,
        max_wall_age_s=MAP_CLOUD_MAX_WALL_AGE_S,
    )
    detail = {
        "cloud_stamp_ns": stamp_ns,
        "cloud_stamp_s": stamp_ns * 1e-9,
        "pair": pair,
        "clock_note": clock_note,
        **ages,
    }
    if not ok:
        return "reject", reason, detail
    detail["exact_pair"] = True
    return "ok", None, detail


def _decode_native_deskewed_cloud(message):
    """Decode the native PointXYZINormal cloud, refusing every raw wire layout."""
    import numpy as np

    from embodied_navigation.simulation.dynanav.rtx_deskewed_perception import (
        DeskewedCloudError,
        verify_cloud_layout,
        verify_deskew_marker,
    )

    verify_cloud_layout([field.name for field in message.fields])
    fields = {field.name: field for field in message.fields}
    count = int(message.width) * int(message.height)
    if count <= 0:
        raise DeskewedCloudError("empty deskewed cloud")
    if count > OPERATOR_CLOUD_MAX_POINTS:
        raise DeskewedCloudError(
            "operator cloud exceeds %d points; refusing the whole cloud, not resampling"
            % OPERATOR_CLOUD_MAX_POINTS
        )
    dtype = np.dtype(
        {
            "names": ["x", "y", "z", "curvature"],
            "formats": [np.float32, np.float32, np.float32, np.float32],
            "offsets": [
                fields["x"].offset,
                fields["y"].offset,
                fields["z"].offset,
                fields["curvature"].offset,
            ],
            "itemsize": message.point_step,
        }
    )
    raw = np.frombuffer(message.data, dtype=dtype, count=count)
    xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(float)
    curvature = np.asarray(raw["curvature"], dtype=float)
    finite = np.isfinite(xyz).all(axis=1) & np.isfinite(curvature)
    if not bool(finite.any()):
        raise DeskewedCloudError("deskewed cloud has no finite points")
    marker = verify_deskew_marker(curvature[finite])
    return xyz[finite], int((~finite).sum()), marker


MAP_CALLBACK_PUMP_LIMIT = 8


def _map_clock_snapshot(state, now_wall):
    """Describe the latest actually received clock, never guessed physics-now."""
    source = state.get("clock")
    arrival = state.get("clock_arrival_wall")
    age = None
    usable = False
    try:
        source, arrival = float(source), float(arrival)
        if math.isfinite(source) and math.isfinite(arrival):
            age = float(now_wall) - arrival
            usable = math.isfinite(age) and 0.0 <= age <= MAP_CLOCK_MAX_WALL_AGE_S
        else:
            source = arrival = None
    except (ValueError, TypeError):
        source = arrival = None
    return {"source_time_s": source, "arrival_wall_s": arrival,
            "clock_wall_age_s": age, "usable": usable,
            "scope": "latest received /clock callback, not physics-now",
            "clock_extrapolated": False}


def _bounded_map_callback_pump(state, *, spin_once, node, clock=None):
    """At most eight nonblocking spins; callbacks cache data, never consume it."""
    before = int(state.get("callback_count", 0))
    begin = _monotonic_now(clock)
    for _ in range(MAP_CALLBACK_PUMP_LIMIT):
        spin_once(node, timeout_sec=0.0)
    end = _monotonic_now(clock)
    return {"spin_calls": MAP_CALLBACK_PUMP_LIMIT,
            "callback_count": int(state.get("callback_count", 0)) - before,
            "max_spin_calls": MAP_CALLBACK_PUMP_LIMIT, "timeout_sec": 0.0,
            "wall_duration_s": end - begin, "started_wall_s": begin,
            "finished_wall_s": end}


def _consume_fixed_cloud_batch(pending, consume):
    """Isolate callback arrivals from the batch currently being consumed."""
    batch = tuple(pending)
    pending.clear()
    waiting = consume(batch)
    newly_received = len(pending)
    pending[:0] = waiting
    return {"batch_count": len(batch), "waiting_count": len(waiting),
            "received_during_batch": newly_received, "pending_count": len(pending)}


def _write_deskewed_perception(
    points_map, pose, args, state, *, stamp_s, arrival_wall, pair
):
    """Write native or explicitly enabled joint safety evidence, never map depth."""
    from embodied_navigation.simulation.dynanav.rtx_deskewed_perception import (
        DeskewedCloudError, build_perception_payload, fail_closed_payload, map_points_to_base,
    )

    build_started_wall = time.perf_counter()
    clock_before_build = _map_clock_snapshot(state, build_started_wall)
    depth_manifest = getattr(args, "nearfield_depth_manifest", None)
    fusion_enabled = getattr(args, "enable_nearfield_depth_fusion", False)
    native_cache = []

    def native_fallback():
        # A rejected depth input may need native evidence, but no native geometry
        # is computed eagerly in the enabled, valid-depth path.
        if not native_cache:
            native_cache.append(build_perception_payload(
                points_base, stamp_s=stamp_s, arrival_wall_s=arrival_wall,
                producer_metadata=producer, source_topic=args.cloud_topic,
            ))
        return native_cache[0]

    try:
        mount = getattr(args, "imu_mount", None)
        if mount is None:
            raise DeskewedCloudError("imu_mount_missing")
        points_base = map_points_to_base(points_map, pose["position"], pose["quat_xyzw"], mount)
        producer = {
            "stage": pair["stage"], "reference_stamp_ns": pair["reference_stamp_ns"],
            "cloud_stamp_ns": pair["cloud_stamp_ns"], "pose_stamp_ns": pair["pose_stamp_ns"],
            "cloud_topic": args.cloud_topic, "pose_topic": args.odometry_topic,
            "map_frame": pair["map_frame"], "child_frame": pair["child_frame"],
            "binary_sha256": state.get("source_binary_sha256"),
        }
        if depth_manifest is not None and fusion_enabled:
            from embodied_navigation.simulation.dynanav.depth_safety_fusion import native_source_envelope
            payload = native_source_envelope(points_base, stamp_s=stamp_s,
                                            arrival_wall_s=arrival_wall, producer_metadata=producer)
        else:
            payload = native_fallback()
    except (DeskewedCloudError, ValueError, AttributeError) as error:
        payload = fail_closed_payload(stamp_s=stamp_s, arrival_wall_s=arrival_wall, error=error)
    if depth_manifest is not None and ("producer_error" not in payload
                                      or payload.get("native_geometry_evaluated") is False):
        from embodied_navigation.simulation.dynanav.depth_safety_fusion import (
            FUSION_SOURCE, build_safety_fusion, invalidate_fused_perception,
            validate_fusion_contract, validate_fusion_source_ages,
        )
        now_sim, _ = _authoritative_sim_clock(state)
        payload = build_safety_fusion(
            points_base, payload, pair=pair,
            pose_arrival_wall_s=pose.get("arrival_wall_s"),
            depth_manifest=depth_manifest, now_sim_s=now_sim,
            now_wall_s=time.perf_counter(), enabled=fusion_enabled,
            native_fallback_factory=native_fallback if fusion_enabled else None,
        )
        # Refresh only real queued callbacks after expensive geometry. This
        # pump cannot consume recursively and cannot extend the fixed batch.
        computed_wall = time.perf_counter()
        before_pump = _map_clock_snapshot(state, computed_wall)
        pump = state.get("pump_callbacks")
        pump_evidence = pump() if callable(pump) else {
            "spin_calls": 0, "callback_count": 0, "max_spin_calls": MAP_CALLBACK_PUMP_LIMIT,
            "timeout_sec": 0.0, "note": "no callback pump installed",
        }
        post_wall = time.perf_counter()
        post_clock = _map_clock_snapshot(state, post_wall)
        diagnostic = payload["depth_safety_diagnostic"]
        diagnostic.update(
            accepted_input=diagnostic.get("accepted") is True,
            accepted_post_compute=False,
            post_compute_scope="source contract and age; geometry still independently required",
            computation_started_wall_s=build_started_wall,
            computation_finished_wall_s=computed_wall,
            computation_wall_s=computed_wall - build_started_wall,
            post_compute_wall_s=post_wall,
            clock_before_compute=clock_before_build,
            clock_after_compute_before_pump=before_pump,
            clock_after_pump=post_clock, post_compute_pump=pump_evidence,
        )
        if diagnostic["accepted_input"]:
            try:
                now_sim = post_clock["source_time_s"] if post_clock["usable"] else None
                validate_fusion_source_ages(diagnostic["sources"], now_sim_s=now_sim,
                                           now_wall_s=post_wall)
                if payload.get("source") == FUSION_SOURCE:
                    validate_fusion_contract(payload, now_sim_s=now_sim, now_wall_s=post_wall)
                diagnostic["accepted_post_compute"] = True
            except ValueError as error:
                diagnostic.update(accepted=False, control_used=False,
                                  reason="post_compute:" + str(error))
                if payload.get("source") == FUSION_SOURCE:
                    # A truly expired pair must remain unknown after pumping.
                    # Do not add a native fit to an already exhausted deadline.
                    payload = invalidate_fused_perception(payload, error)

    perception_path = Path(args.perception_out)
    perception_path.parent.mkdir(parents=True, exist_ok=True)
    atomic_write_json(perception_path, payload)
    directions = payload.get("directions") or {}
    return {
        "path": str(args.perception_out),
        "source": payload.get("source"),
        "depth_safety_diagnostic": payload.get("depth_safety_diagnostic"),
        "stamp_s": payload.get("stamp_s"),
        "arrival_wall_s": payload.get("arrival_wall_s"),
        "observation_state": payload.get("observation_state"),
        "clearance_m": payload.get("clearance_m"),
        "fail_closed": payload.get("fail_closed"),
        "front": directions.get("front"),
        "blind_azimuth_sectors_deg": (payload.get("safety_geometry") or payload.get("lidar_geometry") or {}).get(
            "blind_azimuth_sectors_deg"
        ),
        "near_field_floor_unobserved_m": (payload.get("safety_geometry") or payload.get("lidar_geometry") or {}).get(
            "near_field_floor_unobserved_m"
        ),
    }


def _async_fusion_requested(args):
    """Reject incompatible opt-ins before importing/starting ROS or a worker."""
    enabled = getattr(args, "enable_async_depth_fusion", False)
    if enabled and (getattr(args, "cloud_semantics", None) != "lio_deskewed_map"
                    or not getattr(args, "enable_nearfield_depth_fusion", False)
                    or getattr(args, "nearfield_depth_manifest", None) is None
                    or not getattr(args, "perception_out", None)):
        raise ValueError("--enable-async-depth-fusion requires native LIO semantics, "
                         "--enable-nearfield-depth-fusion, --nearfield-depth-manifest "
                         "and --perception-out")
    return enabled


def _publish_async_fusion(args, state):
    """Main owner is the only publisher; check again after serialization/IO.

    Expired cached evidence is made unknown without refreshing any original
    source stamp/arrival. The unchanged tracker independently gates every tick.
    """
    from embodied_navigation.simulation.dynanav.depth_safety_fusion import invalidate_fused_perception
    session = state["async_fusion"]
    result = session.poll()
    if result is not None:
        path = Path(args.perception_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + ".tmp")
        try:
            temporary.write_text(json.dumps(result.payload, sort_keys=True, separators=(",", ":")))
            session.validate(result)  # Latest real clock + source versions, immediately before replace.
            os.replace(temporary, path)
        except (ValueError, OSError, TypeError, KeyError) as error:
            session.counters["pre_replace_rejected"] += 1
            session.last_error = str(error)
            temporary.unlink(missing_ok=True)
        else:
            session.published(result)
            payload = result.payload
            receipt = {"path": str(path), "source": payload.get("source"),
                       "stamp_s": payload.get("stamp_s"), "arrival_wall_s": payload.get("arrival_wall_s"),
                       "observation_state": payload.get("observation_state"),
                       "clearance_m": payload.get("clearance_m"), "fail_closed": payload.get("fail_closed"),
                       "front": (payload.get("directions") or {}).get("front"),
                       "depth_safety_diagnostic": payload.get("depth_safety_diagnostic")}
            state["perception_samples"] = state.get("perception_samples", 0) + 1
            state["last_perception"] = receipt
    previous = session.last_published
    if previous is not None and not session.last_invalidated:
        try:
            session.validate(previous, cached=True)
        except (ValueError, OSError, TypeError, KeyError) as error:
            rejected = invalidate_fused_perception(previous.payload, error)
            atomic_write_json(args.perception_out, rejected)
            session.last_invalidated = True
            session.counters["cached_invalidated"] += 1
            session.last_error = str(error)
            state["last_perception"] = dict(state.get("last_perception") or {},
                                             clearance_m=None, fail_closed=True,
                                             observation_state=rejected["observation_state"])
    state["async_fusion_stats"] = session.stats()


def _operator_map_free_policy(cloud_semantics, requested="auto"):
    """Fail closed when a deskewed cloud lacks physical ray-origin evidence.

    Map/IMU coordinates identify point positions, not the LiDAR optical centre
    or the ray origins over a rotating scan. Do not carve those rays from the
    IMU mount. Floor endpoints remain observed evidence, not traversability.
    """
    if requested not in ("auto", "floor", "rays", "both", "none"):
        raise ValueError("unknown operator map free mode: %r" % (requested,))
    if cloud_semantics == "lio_deskewed_map":
        reason = "deskewed_cloud_without_physical_lidar_ray_origin_contract"
        if requested in ("rays", "both"):
            raise ValueError(
                "lio_deskewed_map cannot safely satisfy free mode %r: %s; "
                "use auto, floor or none" % (requested, reason)
            )
        effective = "floor" if requested == "auto" else requested
    else:
        reason = None
        effective = "both" if requested == "auto" else requested
    return {"requested": requested, "effective": effective,
            "ray_carving_disabled_reason": reason}


def _ingest_deskewed_cloud(
    message, arrival_wall, accumulator, state, args, *, count=True, clock=None
):
    """Apply one native deskewed LIO full cloud to the operator map (frame=map).

    The cloud already carries the same-stage map pose from FAST-LIVO2, so the
    operator map must not re-apply it: points enter as frame="map" and the paired
    T_map_base is passed explicitly for the floor/level gates only.
    """
    from embodied_navigation.simulation.dynanav.rtx_deskewed_perception import (
        DeskewedCloudError,
    )

    free_policy = _operator_map_free_policy(
        "lio_deskewed_map", getattr(args, "free_mode", "auto")
    )
    state["free_space_policy"] = free_policy
    now_write = _monotonic_now(clock)
    verdict, reason, detail = _cloud_admission(
        message, arrival_wall, state, args, now_write, accumulator
    )
    if verdict != "ok":
        return {
            "accepted": False,
            "reasons": [reason],
            "admission_rejected": True,
            "admission_detail": dict(
                detail,
                reason=reason,
                decision_wall_s=now_write,
                admission_stage="pre_add_frame",
            ),
        }
    stamp_ns = int(detail["cloud_stamp_ns"])
    stamp_s = stamp_ns * 1e-9
    pose = state["lio_poses"][stamp_ns]
    try:
        points_map, dropped, marker = _decode_native_deskewed_cloud(message)
    except DeskewedCloudError as error:
        return {
            "accepted": False,
            "reasons": ["cloud_payload_not_native"],
            "admission_detail": {
                "reason": "cloud_payload_not_native",
                "error": str(error),
                "decision_wall_s": now_write,
            },
        }
    sim_age = None if state["clock"] is None else state["clock"] - stamp_s
    wall_age = time.perf_counter() - float(arrival_wall)
    receipt = accumulator.add_frame(
        points_map,
        frame="map",
        cloud_stamp_s=stamp_s,
        pose=pose["map_pose"],
        pose_stamp_s=stamp_s,
        sim_age_s=sim_age,
        wall_age_s=wall_age,
        arrival_wall_s=float(arrival_wall),
        pose_arrival_wall_s=pose["arrival_wall_s"],
        pose_source=args.pose_source,
        cloud_source=args.cloud_topic,
        note="native_deskewed_lio_full_cloud frame=map; same integer stamp pair",
        free_mode=free_policy["effective"],
    )
    receipt["admission"] = dict(
        detail,
        decision_wall_s=now_write,
        admission_stage="pre_add_frame",
        dropped_nonfinite=dropped,
        deskew_marker=marker,
    )
    receipt["pair_contract"] = dict(detail["pair"])
    receipt["free_space_policy"] = dict(free_policy)
    # The exact same-stage pair is evidence on every accepted frame, independent
    # of whether a perception sample is also written.
    state["last_pair_contract"] = dict(detail["pair"])
    if getattr(args, "perception_out", None) and state.get("async_fusion") is not None:
        session = state["async_fusion"]
        import hashlib
        enqueue_started_wall = time.perf_counter()
        receipt["async_fusion_enqueued"] = session.submit(
            points_map=points_map, pose=pose, pair=detail["pair"],
            config={"imu_mount": args.imu_mount, "cloud_topic": args.cloud_topic,
                    "odometry_topic": args.odometry_topic,
                    "native_cloud_evidence": {
                        "fields": [field.name for field in message.fields],
                        "point_step": int(message.point_step), "width": int(message.width),
                        "height": int(message.height), "deskew_marker": marker,
                        "wire_sha256": hashlib.sha256(bytes(message.data)).hexdigest(),
                        "dropped_nonfinite": dropped}},
            cloud_arrival_wall_s=float(arrival_wall), depth_manifest=args.nearfield_depth_manifest,
        )
        receipt["async_enqueue_wall_s"] = time.perf_counter() - enqueue_started_wall
    elif getattr(args, "perception_out", None):
        receipt["perception"] = _write_deskewed_perception(
            points_map,
            pose,
            args,
            state,
            stamp_s=stamp_s,
            arrival_wall=float(arrival_wall),
            pair=detail["pair"],
        )
    if receipt.get("perception") is not None:
        # The perception sample is real evidence on every admitted frame, even
        # when the caller does not count map frames (pending-cloud retry path).
        state["perception_samples"] = state.get("perception_samples", 0) + 1
        state["last_perception"] = receipt["perception"]
        state["last_pair_contract"] = dict(detail["pair"])
    if count:
        state["frames"] = state.get("frames", 0) + 1
        state["last"] = receipt
        if receipt.get("accepted"):
            state["accepted"] = state.get("accepted", 0) + 1
        else:
            state["rejected"] = state.get("rejected", 0) + 1
    return receipt


def verify_cloud_frame(declared_frame, expected_frame):
    """Return the declared cloud frame, or raise when the message disagrees.

    The point cloud is transformed with an extrinsic written for one frame pair.
    The message header is the only trustworthy frame declaration, so a mismatch
    is refused instead of being relabelled by a command line flag.
    """
    declared = "" if declared_frame is None else str(declared_frame).strip()
    expected = "" if expected_frame is None else str(expected_frame).strip()
    if not expected:
        raise ValueError("expected sensor frame must be given")
    if declared != expected:
        raise ValueError(
            "point cloud frame " + repr(declared) + " does not match the expected "
            "sensor frame " + repr(expected) + "; refusing to relabel it"
        )
    return declared


def _authoritative_sim_clock(state):
    """Return the source sim clock only when it is a current sample.

    The map must not pretend a now it does not have: a missing, stale or
    non-finite /clock sample becomes None, and the snapshot then degrades to the
    recorded-arrival path where a stopped stream stays non-actionable. The
    arrival and the source stamp stay independent clocks; neither is derived
    from the other.
    """
    clock = state.get("clock")
    arrival = state.get("clock_arrival_wall")
    if clock is None:
        return None, "no_clock_topic_sample"
    try:
        clock_value = float(clock)
        arrival_value = float(arrival)
    except (TypeError, ValueError):
        return None, "clock_sample_unreadable"
    if not (math.isfinite(clock_value) and math.isfinite(arrival_value)):
        return None, "clock_sample_not_finite"
    age = time.perf_counter() - arrival_value
    if age < 0.0 or age > MAP_CLOCK_MAX_WALL_AGE_S:
        return None, "clock_sample_stale"
    return clock_value, "current_clock_sample"


def _operator_grid_source_contract(args, state, snapshot):
    """Keep configured topics, observed source labels and pairing evidence distinct.

    Sensor-frame coordinates do not identify a sensing modality, and a selected
    topic alone does not prove that an exact pair was consumed. Copy the runtime
    contract rather than inferring RGB-D/post-VIO provenance from old defaults.
    """
    return {
        "cloud_topic": args.cloud_topic,
        "odometry_topic": args.odometry_topic,
        "pose_source": args.pose_source,
        "snapshot_sources": dict(snapshot.get("sources") or {}),
        "cloud_input_semantics": state.get("cloud_semantics"),
        "deskewed_stage": state.get("deskewed_stage"),
        "free_space_policy": dict(state.get("free_space_policy") or {}),
        "last_pair": (
            None if state.get("last_pair_contract") is None
            else dict(state["last_pair_contract"])
        ),
    }


def _export_operator_grid(
    accumulator,
    level_index,
    level_z,
    export_dir,
    *,
    event,
    written_wall_s,
    map_frame,
    odometry_map_frame,
    source_clock_used,
    source_clock_note,
    frontier,
    partial,
    source_contract=None,
):
    """Write the real observed cells of one level as .npz plus a JSON manifest.

    Read-only diagnostic: the cells are copied from the accumulator grid and the
    map state is never touched. Unknown cells are not exported and are never
    written as free; a renderer must draw them as unknown.
    """
    import hashlib

    import numpy as np

    export_dir = Path(export_dir)
    export_dir.mkdir(parents=True, exist_ok=True)
    grid = accumulator.grid_for(level_index)
    if grid is None:
        cells = {}
    else:
        occupied_count, free_count = grid.counts()
        total = int(occupied_count) + int(free_count)
        if total > int(OPERATOR_GRID_MAX_CELLS):
            raise ValueError(
                "operator grid export refuses {} cells (limit {}); the export is "
                "bounded instead of allocating an unbounded file".format(
                    total, int(OPERATOR_GRID_MAX_CELLS)
                )
            )
        cells = grid.cells()
    keys = sorted(cells)
    int32_max = 2 ** 31 - 1
    for key in keys:
        if not (-int32_max - 1 <= int(key[0]) <= int32_max and -int32_max - 1 <= int(key[1]) <= int32_max):
            raise ValueError("cell index outside int32 range: %r" % (key,))
        if int(cells[key]) not in (0, 1):
            raise ValueError("unexpected cell code %r at %r" % (cells[key], key))
    ix = np.asarray([key[0] for key in keys], dtype=np.int32)
    iy = np.asarray([key[1] for key in keys], dtype=np.int32)
    code = np.asarray([int(cells[key]) for key in keys], dtype=np.int32)
    base_name = "level_%+.2f_%s" % (float(level_z), event)
    npz_path = export_dir / (base_name + ".npz")
    manifest_path = export_dir / (base_name + ".json")
    import tempfile
    import os

    with tempfile.NamedTemporaryFile(dir=str(export_dir), delete=False) as handle:
        np.savez(handle, ix=ix, iy=iy, code=code)
        temporary = Path(handle.name)
    os.replace(temporary, npz_path)
    digest = hashlib.sha256(npz_path.read_bytes()).hexdigest()
    free = int(np.count_nonzero(code == 0))
    occupied = int(np.count_nonzero(code == 1))
    resolution = float(accumulator.resolution_m)
    sources = dict(source_contract or {})
    if (sources.get("cloud_input_semantics") == "lio_deskewed_map"
            and sources.get("deskewed_stage") == "LIO"):
        source_note = "native deskewed LIO map-frame clouds plus same-stage LIO poses"
    else:
        source_note = "measured point clouds plus estimator poses (modality/stage not inferred)"
    manifest = {
        "schema": "operator_grid_export/v1",
        "event": event,
        "written_wall_s": float(written_wall_s),
        "source_clock_used": source_clock_used,
        "source_clock_note": source_clock_note,
        "map_frame": map_frame,
        "odometry_map_frame": odometry_map_frame,
        "level_index": int(level_index),
        "level_z_m": float(level_z),
        "resolution_m": resolution,
        "classification_floor_z_m": accumulator._level_floor_z.get(level_index),
        "cell_arrays": "ix, iy, code (int32, one row per observed cell)",
        "code_semantics": {"0": "free", "1": "occupied", "-1": "unknown, never written as free"},
        "counts": {"free": free, "occupied": occupied, "total": int(code.size), "unknown": "not_exported"},
        "bounds": {
            "min_ix": int(ix.min()) if ix.size else None,
            "max_ix": int(ix.max()) if ix.size else None,
            "min_iy": int(iy.min()) if iy.size else None,
            "max_iy": int(iy.max()) if iy.size else None,
            "note": "cell indices at resolution_m; the export is not cropped",
        },
        "coverage_area_m2": float(code.size) * resolution * resolution,
        "provenance": (
            "operator occupancy from " + source_note
            + " in the local estimator frame; not a FAST-LIVO Laser_map dump "
            "and not a global loop-closed map"
        ),
        "source_contract": sources,
        "free_space_semantics": {
            "free_mode": sources.get("free_space_policy", {}).get(
                "effective", accumulator.free_mode),
            "policy": dict(sources.get("free_space_policy") or {}),
            "free_is_traversability_certificate": False,
            "note": (
                "FREE may represent a floor return or a projected sensor ray, "
                "depending on free_mode; it does not certify continuous ground "
                "support, robot footprint clearance or a safe path. Unknown "
                "cells remain unobserved. Independent motion gates still apply."
            ),
        },
        "partial": bool(partial),
        "frontier": {
            "actionable": None if frontier is None else frontier.get("actionable"),
            "reasons": None if frontier is None else list(frontier.get("reasons") or []),
        },
        "pose_marker": None,
        "pose_marker_note": "no robot pose marker is drawn; the export contains cells only",
        "npz_sha256": digest,
        "npz_path": str(npz_path),
    }
    with tempfile.NamedTemporaryFile(
        dir=str(export_dir), mode="w", delete=False, encoding="utf-8"
    ) as manifest_handle:
        json.dump(manifest, manifest_handle, indent=2)
        manifest_temporary = Path(manifest_handle.name)
    os.replace(manifest_temporary, manifest_path)
    return {"npz": str(npz_path), "manifest": str(manifest_path), "sha256": digest,
            "counts": manifest["counts"], "resolution_m": resolution}


def _write_operator_map_snapshots(accumulator, out_dir, args, state, *, event):
    """Write every level snapshot plus a manifest, and return the manifest.

    Called both periodically while the accumulator is still running and once at
    the end, so the operator can read a fresh accumulated map at low rate instead
    of waiting for the process to exit. Each level keeps its own snapshot file;
    latest.json names the newest write, the level files, and the freshness facts.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    levels = {}
    now_sim_s, sim_clock_note = _authoritative_sim_clock(state)
    for level_index, level_z in accumulator.levels:
        snapshot = accumulator.snapshot(
            level_index,
            frontier_limit=args.frontier_limit,
            now_sim_s=now_sim_s,
        )
        path = out_dir / ("level_%+.2f.json" % level_z)
        atomic_write_json(path, snapshot)
        frontier = snapshot.get("frontier", {})
        entry = {
            "path": str(path),
            "actionable": frontier.get("actionable"),
            "reasons": frontier.get("reasons"),
            "cells": frontier.get("returned_cells"),
            "level_z_m": snapshot.get("level_z_m"),
            "accumulated_frames": snapshot.get("accumulated_frames"),
        }
        export_dir = getattr(args, "export_grid", None)
        if export_dir is not None:
            try:
                entry["grid_export"] = _export_operator_grid(
                    accumulator,
                    level_index,
                    level_z,
                    export_dir,
                    event=event,
                    written_wall_s=time.time(),
                    map_frame=accumulator.map_frame,
                    odometry_map_frame=state.get("odometry_map_frame"),
                    source_clock_used=now_sim_s,
                    source_clock_note=sim_clock_note,
                    frontier=frontier,
                    partial=event != "operator_map_accumulated",
                    source_contract=_operator_grid_source_contract(args, state, snapshot),
                )
            except Exception as error:  # noqa: BLE001 - diagnostic must not break the map
                entry["grid_export_error"] = str(error)
        levels["%+.2f" % level_z] = entry
    manifest = {
        "event": event,
        "written_wall_s": time.time(),
        "cloud_topic": args.cloud_topic,
        "odometry_topic": args.odometry_topic,
        "sensor_frame": args.sensor_frame,
        "base_frame": args.base_frame,
        "pose_source": args.pose_source,
        "free_space_policy": dict(state.get("free_space_policy") or {}),
        "extrinsic_sha256": accumulator.extrinsic.sha256,
        "source_clock_used": now_sim_s,
        "pose_frame_contract": {
            "base_frame": args.base_frame,
            "accepted": state.get("pose_frame_accepted"),
            "rejected": state.get("pose_frame_rejected"),
            "last_note": state.get("pose_frame_note"),
            "odometry_map_frame": state.get("odometry_map_frame"),
            "frame_rejections": state.get("frame_rejections"),
            "frame_rejection_reason": state.get("frame_rejection_reason"),
            "cloud_input_semantics": state.get("cloud_semantics", "sensor"),
            "pose_wait_rejections": state.get("pose_wait_rejections"),
            "last_pose_wait_rejection": state.get("last_pose_wait_rejection"),
            "pending_overflow_rejections": state.get("pending_overflow_rejections"),
            "admission_rejections": state.get("admission_rejections"),
            "last_cloud_rejection": state.get("last_cloud_rejection"),
        },
        "source_clock_note": sim_clock_note,
        "callback_pump": {"limit": MAP_CALLBACK_PUMP_LIMIT,
                          "latest_main": state.get("last_main_pump"),
                          "latest_fixed_batch": state.get("last_fixed_batch"),
                          "callbacks_only_cache_measurements": True},
        "deskewed_pair_contract": (
            None
            if state.get("cloud_semantics") != "lio_deskewed_map"
            else {
                "stage": state.get("deskewed_stage"),
                "cloud_topic": args.cloud_topic,
                "pose_topic": args.odometry_topic,
                "exact_integer_stamp_pair": True,
                "interpolation_allowed": False,
                "cloud_enters_map_as": "frame=map (no second transform)",
                "pair_transform": "T_map_imu published by FAST-LIVO2; T_map_base = T_map_imu @ inv(T_base_imu) once",
                "last_pair": state.get("last_pair_contract"),
                "source_binary": state.get("source_binary_identity"),
            }
        ),
        "perception": (
            None
            if not getattr(args, "perception_out", None)
            else {
                "path": str(args.perception_out),
                "samples": state.get("perception_samples", 0),
                "last": state.get("last_perception"),
            }
        ),
        "cloud_frame_declared": state["declared_frame"],
        "cloud_frame_mismatch": state["frame_mismatch"],
        "frames": state["frames"],
        "accepted_frames": state["accepted"],
        "rejected_frames": state["rejected"],
        "last_frame": state["last"],
        "levels": levels,
        "output_dir": str(out_dir),
        "ground_truth_used": False,
        "path_length_is_not_coverage": True,
        "snapshot_is_partial_until_final": event != "operator_map_accumulated",
        "note": (
            "operator aid from the live SLAM point cloud and the measured "
            "estimator pose, with an explicit sensor->base extrinsic and "
            "stamp pairing; never a navigation input for the base"
        ),
    }
    if "async_fusion_stats" in state:
        manifest["async_fusion"] = state["async_fusion_stats"]
    atomic_write_json(out_dir / "latest.json", manifest)
    return manifest


def _map_pose_from_odometry(pose, frame_id, child_frame_id, *, base_frame, imu_mount):
    """Return (T_map_base, note) or (None, refusal_reason) for one odometry sample.
    The operator map needs a pose whose child frame IS its base frame. The raw ROS
    cloud published as /dynanav/lidar_filtered carries IMU points with
    header.frame_id == "go2_imu"; only the interactive perception JSON is already
    transformed IMU->base. So the accumulator input frame is the sensor frame
    (frame="sensor", transformed once by the explicit Extrinsic), while the pose
    must be converted once from T_map_imu to T_map_base with the measured imu_mount.
    Labelling an IMU pose as the base pose would drop (or double-apply) that lever
    arm.

    Rules: unlabelled frames are refused; a child frame equal to the map base frame
    is used as is; a go2_imu pose is converted once with an explicitly supplied
    measured imu_mount translation (T_map_base = T_map_imu @ T_imu_base, axes
    aligned, T_imu_base = -imu_mount); any other child frame is refused. No ground
    truth pose is accepted and no mount is guessed.
    """
    if not isinstance(frame_id, str) or not frame_id.strip():
        return None, "odometry_map_frame_unlabelled"
    if not isinstance(child_frame_id, str) or not child_frame_id.strip():
        return None, "odometry_child_frame_unlabelled"
    if child_frame_id == base_frame:
        return np.asarray(pose, dtype=float), "child_frame_is_base"
    if child_frame_id != "go2_imu":
        return None, "odometry_child_frame_not_base:" + child_frame_id
    if imu_mount is None:
        return None, "imu_pose_without_measured_imu_mount"
    mount = np.asarray(imu_mount, dtype=float).reshape(-1)
    if mount.shape != (3,) or not np.isfinite(mount).all():
        return None, "imu_mount_invalid"
    pose = np.asarray(pose, dtype=float)
    conversion = np.eye(4)
    conversion[:3, 3] = -mount
    return pose @ conversion, "imu_pose_converted_with_measured_imu_mount"


# A cloud may arrive before the post-VIO pose for its own source stamp. The consumer
# loop keeps it pending for at most the existing map wall-freshness window instead
# of refusing it on arrival; the pose pairing window itself is unchanged (0.10 s).
# The consumer loop refuses a cloud before any geometry write when its original arrival or
# source stamp is already outside the map freshness windows; waiting only ever applies to
# a cloud whose pose pair has not arrived yet.
CLOUD_PENDING_MAX_WALL_S = 0.5
MAP_CLOUD_MAX_SIM_AGE_S = 0.25
MAP_CLOUD_MAX_WALL_AGE_S = 0.5
OPERATOR_GRID_MAX_CELLS = 200000
MAP_PAIR_MAX_DT_S = 0.10
CLOUD_PENDING_MAX_COUNT = 64


def _ingest_odometry(message, accumulator, state, args, arrival_wall):
    """Apply one odometry message to the operator map after the frame contract.

    The map needs a pose in T_map_base. A change of the estimator map frame, an
    unlabelled or unexpected child frame, or a pose that cannot be converted makes
    the whole generation untrusted: the geometry already accumulated may mix frames
    and must not stay actionable. Returns True when the pose was applied.
    """
    from embodied_navigation.simulation.dynanav.operator_map import (
        transform_from_position_quaternion,
    )

    stamp = message_stamp_s(message)
    pose = transform_from_position_quaternion(
        [
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        ],
        [
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        ],
    )
    frame_id = message.header.frame_id
    child_frame_id = message.child_frame_id
    expected = state.get("odometry_map_frame")
    if expected is not None and frame_id != expected:
        state["frame_rejections"] = state.get("frame_rejections", 0) + 1
        state["frame_rejection_reason"] = "odometry_map_frame_changed:" + str(frame_id)
        accumulator.mark_untrusted(
            "odometry_map_frame_changed", source=str(frame_id)
        )
        return False
    map_pose, note = _map_pose_from_odometry(
        pose,
        frame_id,
        child_frame_id,
        base_frame=args.base_frame,
        imu_mount=args.imu_mount,
    )
    state["pose_frame_note"] = note
    if map_pose is None:
        state["pose_frame_rejected"] = state.get("pose_frame_rejected", 0) + 1
        state["frame_rejection_reason"] = note
        accumulator.mark_untrusted(note, source=str(child_frame_id))
        return False
    state["odometry_map_frame"] = frame_id
    if state.get("pair_required"):
        # Same-stage LIO pose keyed by the exact integer stamp; the deskewed
        # cloud is only admitted with this exact key (no interpolation).
        _store_lio_pose(state, message, map_pose, arrival_wall)
    stamps = state.setdefault("pose_stamps", [])
    stamps.append(float(stamp))
    del stamps[:-200]
    state["pose_frame_accepted"] = state.get("pose_frame_accepted", 0) + 1
    accumulator.add_pose(
        stamp,
        map_pose,
        pose_source=args.pose_source,
        arrival_wall_s=arrival_wall,
    )
    return True


def _ingest_cloud(
    message, arrival_wall, accumulator, state, args, *, count=True, clock=None
):
    """Apply one raw ROS cloud to the operator map as a sensor-frame input.

    The raw /dynanav/lidar_filtered payload is IMU points (header go2_imu), so the
    accumulator receives frame="sensor" and the explicit Extrinsic performs the
    single sensor->base transform; the declared ROS frame is kept as provenance.
    A frame disagreement makes the generation untrusted instead of being relabelled.
    """
    if state.get("cloud_semantics") == "lio_deskewed_map":
        return _ingest_deskewed_cloud(
            message, arrival_wall, accumulator, state, args, count=count, clock=clock
        )
    declared_frame = message.header.frame_id
    state["declared_frame"] = declared_frame
    try:
        declared_frame = verify_cloud_frame(declared_frame, args.sensor_frame)
    except ValueError as error:
        state["frame_mismatch"] = str(error)
        state["frames"] = state.get("frames", 0) + 1
        state["rejected"] = state.get("rejected", 0) + 1
        state["frame_rejection_reason"] = "cloud_frame_mismatch"
        accumulator.mark_untrusted("cloud_frame_mismatch", source=str(declared_frame))
        return None
    points = cloud_to_xyz(message)
    # Decoding the cloud can take real time; the final admission check uses the clock
    # read right before the write, so a decode that crosses the deadline refuses the
    # cloud instead of writing stale geometry.
    now_write = _monotonic_now(clock)
    verdict, reason, detail = _cloud_admission(
        message, arrival_wall, state, args, now_write, accumulator
    )
    if verdict != "ok":
        return {
            "accepted": False,
            "reasons": [reason],
            "admission_rejected": True,
            "admission_detail": dict(
                detail,
                reason=reason,
                decision_wall_s=now_write,
                admission_stage="post_decode",
            ),
        }
    cloud_stamp = message_stamp_s(message)
    sim_age = None if state["clock"] is None else state["clock"] - cloud_stamp
    receipt = accumulator.add_frame(
        points,
        frame="sensor",
        cloud_stamp_s=cloud_stamp,
        sim_age_s=sim_age,
        wall_age_s=time.perf_counter() - arrival_wall,
        arrival_wall_s=arrival_wall,
        pose_source=args.pose_source,
        cloud_source=args.cloud_topic,
        note="ros_cloud_frame=" + str(declared_frame),
    )
    receipt["admission"] = dict(
        detail, decision_wall_s=now_write, admission_stage="pre_add_frame"
    )
    if count:
        state["frames"] = state.get("frames", 0) + 1
        state["last"] = receipt
        if receipt.get("accepted"):
            state["accepted"] = state.get("accepted", 0) + 1
        else:
            state["rejected"] = state.get("rejected", 0) + 1
    return receipt


def _monotonic_now(clock):
    """Return the current wall time from an injected clock or the real monotonic clock.

    The production loop passes time.perf_counter, so every admission check and every
    geometry write re-reads the real clock instead of reusing a batch-entry value; tests
    may inject a fixed or stepping clock.
    """
    return float(clock() if callable(clock) else time.perf_counter())


def _cloud_admission(message, arrival_wall, state, args, now_wall, accumulator=None):
    """Return (verdict, reason, detail) for one pending cloud before any write.

    verdict is "reject" (final, no geometry), "wait_pose" (only the pose pair is
    missing) or "ok". The checks use the cloud's original source stamp and original
    arrival against the current authoritative clocks; a stale loop time never renews a
    cloud, a missing/freshless clock is never replaced by a fabricated zero age, and the
    pairing window and both freshness windows are the existing ones.
    """
    if state.get("cloud_semantics") == "lio_deskewed_map":
        return _deskewed_cloud_admission(message, arrival_wall, state, args, now_wall)
    declared = None if message.header is None else message.header.frame_id
    try:
        verify_cloud_frame(declared, args.sensor_frame)
    except ValueError as error:
        return "reject", "cloud_frame_mismatch", {"error": str(error), "declared_frame": declared}
    stamp = message_stamp_s(message)
    if not math.isfinite(stamp):
        return "reject", "cloud_stamp_invalid", {"cloud_stamp_s": stamp}
    try:
        arrival = float(arrival_wall)
        now = float(now_wall)
    except (TypeError, ValueError):
        return "reject", "arrival_invalid", {"arrival_wall_s": arrival_wall}
    if not (math.isfinite(arrival) and math.isfinite(now)):
        return "reject", "arrival_invalid", {"arrival_wall_s": arrival, "now_wall_s": now}
    if arrival > now + 2e-6:
        return "reject", "arrival_in_future", {"arrival_wall_s": arrival, "now_wall_s": now}
    wall_age = now - arrival
    if wall_age > MAP_CLOUD_MAX_WALL_AGE_S:
        return "reject", "cloud_wall_expired", {"wall_age_s": wall_age}
    now_sim, clock_note = _authoritative_sim_clock(state)
    if now_sim is None:
        return "reject", "sim_clock_unavailable", {"clock_note": clock_note}
    sim_age = now_sim - stamp
    if sim_age < -2e-6:
        return "reject", "cloud_source_in_future", {"sim_age_s": sim_age, "cloud_stamp_s": stamp}
    if sim_age > MAP_CLOUD_MAX_SIM_AGE_S:
        return "reject", "cloud_sim_expired", {"sim_age_s": sim_age, "cloud_stamp_s": stamp}
    detail = {
        "wall_age_s": wall_age,
        "sim_age_s": sim_age,
        "cloud_stamp_s": stamp,
        "clock_note": clock_note,
    }
    if accumulator is not None:
        pairing = accumulator.preview_pose_pair(stamp)
        detail["pair_ok"] = bool(pairing.ok)
        detail["pair_dt_s"] = pairing.dt_s
        detail["pair_interpolated"] = bool(pairing.interpolated)
        if not pairing.ok:
            if wall_age < min(CLOUD_PENDING_MAX_WALL_S, MAP_CLOUD_MAX_WALL_AGE_S):
                return "wait_pose", "awaiting_pose_pair", detail
            return "reject", "pose_pair_deadline", detail
    return "ok", None, detail


def _sanitize_for_json(value):
    """Return a strict-JSON-safe copy: non-finite floats become None, never NaN."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _sanitize_for_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_sanitize_for_json(item) for item in value]
    return value


def _write_consumption_receipt(path, record):
    """Append one final cloud-consumption receipt; never between admission and add_frame."""
    if path is None:
        return
    import json as _json

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = _json.dumps(_sanitize_for_json(record), allow_nan=False)
    with open(target, "a", encoding="utf-8") as handle:
        handle.write(payload + "\n")


def _retry_pending_clouds(
    pending, accumulator, state, args, now_wall=None, clock=None, receipt_path=None
):
    """Admit or reject each pending cloud before any geometry is written.

    A cloud is written only when its own source stamp and arrival still pass the map
    freshness windows against the current authoritative clocks and a measured pose pair
    is already available. A cloud that is merely waiting for its pose stays pending
    (bounded by the wall window and the hard cache limit); everything else is refused
    once, with the specific reason, and no level, cell or accepted counter changes.
    """
    # Production never trusts a caller-supplied batch time for admission: the real
    # monotonic clock is re-read for every item unless a test injects one explicitly.
    if clock is None:
        clock = time.perf_counter
    remaining = []
    for message, arrival_wall in pending:
        # Re-read the clock for every item: the batch entry time can be stale by the
        # time a previous item was processed or a cloud was decoded.
        now = _monotonic_now(clock)
        verdict, reason, detail = _cloud_admission(
            message, arrival_wall, state, args, now, accumulator
        )
        if verdict == "wait_pose":
            if len(remaining) >= CLOUD_PENDING_MAX_COUNT:
                verdict, reason = "reject", "pending_overflow"
            else:
                remaining.append((message, arrival_wall))
                continue
        if verdict == "reject":
            state["frames"] = state.get("frames", 0) + 1
            state["rejected"] = state.get("rejected", 0) + 1
            if reason == "cloud_frame_mismatch":
                state["frame_rejection_reason"] = "cloud_frame_mismatch"
                accumulator.mark_untrusted(
                    "cloud_frame_mismatch", source=str(detail.get("declared_frame"))
                )
            elif reason == "pose_pair_deadline":
                state["pose_wait_rejections"] = state.get("pose_wait_rejections", 0) + 1
                state["last_pose_wait_rejection"] = dict(
                    detail, reason=reason, arrival_wall_s=float(arrival_wall)
                )
            elif reason == "pending_overflow":
                state["pending_overflow_rejections"] = (
                    state.get("pending_overflow_rejections", 0) + 1
                )
                state["last_pose_wait_rejection"] = dict(
                    detail, reason=reason, arrival_wall_s=float(arrival_wall)
                )
            else:
                state["admission_rejections"] = state.get("admission_rejections", 0) + 1
            state["last_cloud_rejection"] = dict(
                detail, reason=reason, arrival_wall_s=float(arrival_wall)
            )
            _write_consumption_receipt(
                receipt_path,
                {
                    "cloud_source_stamp_s": detail.get("cloud_stamp_s", message_stamp_s(message)),
                    "arrival_wall_s": float(arrival_wall),
                    "decision_wall_s": now,
                    "admission_stage_rejected": True,
                    "reason": reason,
                    "pair_ok": detail.get("pair_ok"),
                    "pair_dt_s": detail.get("pair_dt_s"),
                    "detail": detail,
                },
            )
            state["last_pose_wait_rejection"] = dict(state["last_cloud_rejection"])
            continue
        receipt = _ingest_cloud(
            message,
            arrival_wall,
            accumulator,
            state,
            args,
            count=False,
            clock=clock,
        )
        state["frames"] = state.get("frames", 0) + 1
        state["last"] = receipt
        if receipt is not None and receipt.get("accepted"):
            state["accepted"] = state.get("accepted", 0) + 1
            _write_consumption_receipt(
                receipt_path,
                {
                    "cloud_source_stamp_s": detail.get("cloud_stamp_s"),
                    "arrival_wall_s": float(arrival_wall),
                    "decision_wall_s": (receipt.get("admission") or {}).get("decision_wall_s", now),
                    "admission_stage": (receipt.get("admission") or {}).get("admission_stage"),
                    "admission_stage_rejected": False,
                    "accepted": True,
                    "pair_ok": detail.get("pair_ok"),
                    "pair_dt_s": detail.get("pair_dt_s"),
                    "reasons": [],
                    "floor_evidence": (receipt or {}).get("floor_evidence"),
                    "level_index": (receipt or {}).get("level_index"),
                    "level_z_m": (receipt or {}).get("level_z_m"),
                    "pose_stamp_s": (receipt or {}).get("pose_stamp_s"),
                    "paired_dt_s": (receipt or {}).get("paired_dt_s"),
                    "pairing_interpolated": (receipt or {}).get("pairing_interpolated"),
                    "pairing_bracket": (receipt or {}).get("pairing_bracket"),
                    "classification_floor_z_m": (receipt or {}).get(
                        "classification_floor_z_m"
                    ),
                    "level_floor_anchor_m": (receipt or {}).get("level_floor_anchor_m"),
                },
            )
        else:
            state["rejected"] = state.get("rejected", 0) + 1
            admission_detail = (receipt or {}).get("admission_detail")
            if admission_detail is not None:
                state["last_cloud_rejection"] = dict(admission_detail)
                state["last_pose_wait_rejection"] = dict(admission_detail)
                _write_consumption_receipt(
                    receipt_path,
                    {
                        "cloud_source_stamp_s": admission_detail.get("cloud_stamp_s"),
                        "arrival_wall_s": float(arrival_wall),
                        "decision_wall_s": admission_detail.get("decision_wall_s"),
                        "admission_stage": admission_detail.get("admission_stage"),
                        "admission_stage_rejected": True,
                        "accepted": False,
                        "reason": admission_detail.get("reason"),
                        "pair_ok": admission_detail.get("pair_ok"),
                        "pair_dt_s": admission_detail.get("pair_dt_s"),
                    },
                )
                continue
            state["pose_wait_rejections"] = state.get("pose_wait_rejections", 0) + 1
            state["last_cloud_rejection"] = {
                "cloud_stamp_s": detail.get("cloud_stamp_s"),
                "wall_age_s": detail.get("wall_age_s"),
                "reasons": list((receipt or {}).get("reasons") or []),
                "reason": "pose_pair_rejected",
            }
            _write_consumption_receipt(
                receipt_path,
                {
                    "cloud_source_stamp_s": detail.get("cloud_stamp_s"),
                    "arrival_wall_s": float(arrival_wall),
                    "decision_wall_s": now,
                    "admission_stage_rejected": False,
                    "accepted": False,
                    "pair_ok": detail.get("pair_ok"),
                    "pair_dt_s": detail.get("pair_dt_s"),
                    "reasons": list((receipt or {}).get("reasons") or []),
                    "floor_evidence": (receipt or {}).get("floor_evidence"),
                    "level_index": (receipt or {}).get("level_index"),
                },
            )
    return remaining


def _cmd_map(args):
    """Accumulate an operator map from the live SLAM cloud and measured pose."""
    from embodied_navigation.simulation.dynanav.operator_map import validate_map_cloud_source

    validate_map_cloud_source(args.cloud_topic)
    mailbox = mailbox_for(args.run_dir, args.mailbox)
    run_dir = mailbox.parent
    cloud_semantics = getattr(args, "cloud_semantics", "sensor_imu") or "sensor_imu"
    if cloud_semantics not in ("sensor_imu", "lio_deskewed_map"):
        raise ValueError("unknown cloud semantics: %r" % (cloud_semantics,))
    free_policy = _operator_map_free_policy(
        cloud_semantics, getattr(args, "free_mode", "auto")
    )
    roles = None
    source_identity = _source_binary_identity(getattr(args, "binary_prefix", None))
    if cloud_semantics == "lio_deskewed_map":
        from embodied_navigation.simulation.dynanav.rtx_deskewed_perception import (
            verify_topic_roles,
        )

        # The stage is fixed by the native topic roles: the deskewed full cloud
        # must be paired with the same-stage LIO pose, never with post-VIO odometry.
        roles = verify_topic_roles(args.cloud_topic, args.odometry_topic)
        if args.imu_mount is None:
            raise ValueError(
                "lio_deskewed_map requires the measured --imu-mount "
                "(IMU origin in the base frame); T_map_base is never guessed"
            )
    perceptions = getattr(args, "perception_out", None)
    if cloud_semantics == "lio_deskewed_map" and perceptions is not None:
        perceptions = Path(perceptions).expanduser().resolve()
        perceptions.parent.mkdir(parents=True, exist_ok=True)
    args.perception_out = perceptions
    depth_manifest = getattr(args, "nearfield_depth_manifest", None)
    if getattr(args, "enable_nearfield_depth_fusion", False) and depth_manifest is None:
        raise ValueError("--enable-nearfield-depth-fusion requires --nearfield-depth-manifest")
    if depth_manifest is not None:
        if cloud_semantics != "lio_deskewed_map" or perceptions is None:
            raise ValueError("nearfield safety requires native LIO semantics and --perception-out")
        args.nearfield_depth_manifest = Path(depth_manifest).expanduser().resolve()
    if args.extrinsic_translation is None:
        raise ValueError(
            "explicit sensor->base extrinsic is required "
            "(use --extrinsic-translation and --extrinsic-quaternion); "
            "no calibration is guessed"
        )
    async_requested = _async_fusion_requested(args)
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2

    from embodied_navigation.simulation.dynanav.operator_map import (
        Extrinsic,
        OccupancyAccumulator,
        transform_from_position_quaternion,
    )

    extrinsic = Extrinsic(
        translation=[float(value) for value in args.extrinsic_translation],
        quaternion_xyzw=[float(value) for value in args.extrinsic_quaternion],
        sensor_frame=args.sensor_frame,
        base_frame=args.base_frame,
    )
    accumulator = OccupancyAccumulator(
        extrinsic=extrinsic,
        resolution_m=args.resolution,
        free_mode=free_policy["effective"],
    )
    state = {
        "free_space_policy": free_policy,
        "cloud_semantics": cloud_semantics,
        "deskewed_stage": None if roles is None else roles["stage"],
        "pair_required": cloud_semantics == "lio_deskewed_map",
        "lio_poses": {},
        "source_binary_identity": source_identity,
        "source_binary_sha256": (
            None if source_identity is None else source_identity.get("core_sha256")
        ),
        "perception_samples": 0,
        "last_perception": None,
        "last_pair_contract": None,
        "clock": None,
        "clock_arrival_wall": None,
        "pose_frame_accepted": 0,
        "pose_frame_rejected": 0,
        "pose_frame_note": None,
        "odometry_map_frame": None,
        "frame_rejections": 0,
        "frame_rejection_reason": None,
        "pose_wait_rejections": 0,
        "last_pose_wait_rejection": None,
        "pose_stamps": [],
        "admission_rejections": 0,
        "pending_overflow_rejections": 0,
        "last_cloud_rejection": None,
        "frames": 0,
        "accepted": 0,
        "last": None,
        "rejected": 0,
        "frame_mismatch": None,
        "declared_frame": None,
    }
    pending = []
    pending_odometry = []
    async_session = None

    def message_stamp_s(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def on_odometry(message):
        state["callback_count"] = state.get("callback_count", 0) + 1
        arrival = time.perf_counter()
        if async_session is None:
            _ingest_odometry(message, accumulator, state, args, arrival)
        else:
            if len(pending_odometry) >= 512:
                pending_odometry.pop(0)
                async_session.counters["odometry_callback_overflow"] += 1
            pending_odometry.append((message, arrival))


    def on_cloud(message):
        state["callback_count"] = state.get("callback_count", 0) + 1
        if async_session is not None and len(pending) >= CLOUD_PENDING_MAX_COUNT:
            dropped, arrival = pending.pop(0)
            async_session.counters["cloud_callback_overflow"] += 1
            state["pending_overflow_rejections"] += 1
            state["last_cloud_rejection"] = {"reason": "async_callback_overflow",
                                               "cloud_stamp_s": message_stamp_s(dropped),
                                               "arrival_wall_s": arrival}
        pending.append((message, time.perf_counter()))

    def on_clock(message):
        state["callback_count"] = state.get("callback_count", 0) + 1
        state["clock"] = message.clock.sec + message.clock.nanosec * 1e-9
        state["clock_arrival_wall"] = time.perf_counter()
        if async_session is not None:
            generation = async_session.generation
            async_session.on_clock(state["clock"], state["clock_arrival_wall"])
            if async_session.generation != generation:
                # Old ROS caches cannot form a new-generation pair after a
                # source clock reset. Count discarded cache entries; immutable
                # source files and previous result evidence remain untouched.
                async_session.counters["rollback_cloud_cache_discarded"] += len(pending)
                async_session.counters["rollback_pose_cache_discarded"] += len(state["lio_poses"])
                async_session.counters["rollback_odometry_cache_discarded"] += len(pending_odometry)
                pending.clear()
                pending_odometry.clear()
                state["lio_poses"].clear()

    rclpy.init()
    node = rclpy.create_node("dsh_explore_map_accumulator")
    try:
        if async_requested:
            from embodied_navigation.simulation.dynanav import async_depth_fusion, depth_safety_fusion
            source_paths = [__file__, async_depth_fusion.__file__, depth_safety_fusion.__file__]
            if source_identity is not None:
                prefix = Path(source_identity["prefix"])
                source_paths.extend([prefix / "install/ros/lib/libfast_livo2_core.so",
                                     prefix / "install/ros/lib/fast_livo2_ros/fastlivo_mapping"])
            async_session = async_depth_fusion.FusionCoordinator(
                binary_sha256=state["source_binary_sha256"], source_paths=source_paths)
            state["async_fusion"] = async_session
        node.create_subscription(
            Odometry, args.odometry_topic, on_odometry, qos_profile_sensor_data
        )
        node.create_subscription(
            PointCloud2, args.cloud_topic, on_cloud, qos_profile_sensor_data
        )
        node.create_subscription(Clock, args.clock_topic, on_clock, qos_profile_sensor_data)
        deadline = time.perf_counter() + args.duration
        out_dir = Path(args.out) if args.out else run_dir / "operator_map"
        # The final-consumption receipt log lives here and may be written before the
        # first snapshot on a brand-new output directory, so create it up front and
        # keep the final admission -> add_frame path free of IO.
        out_dir.mkdir(parents=True, exist_ok=True)
        interval = float(getattr(args, "snapshot_interval", 0.0) or 0.0)
        last_snapshot = time.perf_counter()
        state["pump_callbacks"] = lambda: _bounded_map_callback_pump(
            state, spin_once=rclpy.spin_once, node=node, clock=time.perf_counter,
        )
        while time.perf_counter() < deadline:
            state["last_main_pump"] = state["pump_callbacks"]()
            if not pending and not state["last_main_pump"]["callback_count"]:
                # Only wait when the nonblocking pump found no work. Geometry
                # and its final freshness check never use a blocking spin.
                rclpy.spin_once(node, timeout_sec=0.05)
            if async_session is None:
                state["last_fixed_batch"] = _consume_fixed_cloud_batch(
                    pending,
                    lambda batch: _retry_pending_clouds(
                        batch, accumulator, state, args, clock=time.perf_counter,
                        receipt_path=out_dir / "cloud_consumption.jsonl",
                    ),
                )
            else:
                # Callbacks only cache. Preserve original arrivals while the
                # main owner applies native map work; one cloud per pump keeps
                # a backlog from monopolizing reception. Map geometry is NOT
                # moved into the worker, and depth never enters this map.
                odometry_batch = pending_odometry[:MAP_CALLBACK_PUMP_LIMIT]
                del pending_odometry[:len(odometry_batch)]
                for message, arrival in odometry_batch:
                    _ingest_odometry(message, accumulator, state, args, arrival)
                _publish_async_fusion(args, state)
                batch = pending[:1]
                del pending[:len(batch)]
                waiting = _retry_pending_clouds(
                    batch, accumulator, state, args, clock=time.perf_counter,
                    receipt_path=out_dir / "cloud_consumption.jsonl")
                pending.extend(waiting)
                state["last_fixed_batch"] = {"batch_count": len(batch),
                                             "waiting_count": len(waiting), "pending_count": len(pending)}
                _publish_async_fusion(args, state)
            if interval > 0 and time.perf_counter() - last_snapshot >= interval:
                # Keep a readable accumulated map on disk during the run: the
                # operator decides the next waypoint from it, not from a
                # snapshot that only appears when the process exits.
                _write_operator_map_snapshots(
                    accumulator, out_dir, args, state, event="operator_map_partial"
                )
                last_snapshot = time.perf_counter()
        out_dir = Path(args.out) if args.out else run_dir / "operator_map"
        summary = _write_operator_map_snapshots(
            accumulator, out_dir, args, state, event="operator_map_accumulated"
        )
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0
    finally:
        if async_session is not None:
            # Cancellation is bounded. A still-running daemon can finish only
            # owned CPU work; close removes its permission to publish results.
            joined = async_session.close(timeout=1.0)
            async_session.counters["shutdown_join_timed_out"] += int(not joined)
            try:
                _publish_async_fusion(args, state)
            except OSError as error:
                # Existing file freshness still stops the consumer. Cleanup of
                # ROS must not be skipped because shutdown invalidation IO failed.
                async_session.last_error = "shutdown_invalidation_io:" + str(error)
        # A TERM from the run harness makes rclpy shut the context down itself;
        # the map evidence is already on disk, so shutdown must never raise.
        try:
            node.destroy_node()
        finally:
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001 - already shut down by a signal
                pass


def _add_wait_options(parser):
    """Attach the shared --wait/--timeout pair and its documented exit codes."""
    parser.add_argument(
        "--wait",
        action="store_true",
        help=(
            "wait for this action's structured result; exit 0 done, "
            "1 rejected/failed, 2 timeout (see exit codes)"
        ),
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=20.0,
        help="seconds to wait with --wait (default: 20)",
    )


def build_parser():
    """Build the operator CLI.

    The run selector works in both positions (before or after the subcommand).
    It is registered on a shared parent parser with a suppressed default, so the
    subparser cannot overwrite a value given before the subcommand.
    """
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--run-dir", type=Path, help="experiment run directory", default=argparse.SUPPRESS
    )
    common.add_argument(
        "--mailbox",
        type=Path,
        help="mailbox directory override",
        default=argparse.SUPPRESS,
    )
    common.add_argument(
        "--run-id",
        help="run id override (defaults to run dir name)",
        default=argparse.SUPPRESS,
    )
    parser = argparse.ArgumentParser(
        description=__doc__,
        parents=[common],
        epilog=EXIT_CODE_HELP,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    heartbeat = subparsers.add_parser(
        "heartbeat", help="refresh operator liveness", parents=[common]
    )
    heartbeat.add_argument("--loop", action="store_true")
    heartbeat.add_argument("--interval", type=float, default=1.0)
    heartbeat.add_argument("--duration", type=float, default=600.0)
    heartbeat.set_defaults(handler=_cmd_heartbeat)

    goto = subparsers.add_parser("goto", help="submit one waypoint", parents=[common])
    goto.add_argument("--x", type=float, required=True)
    goto.add_argument("--y", type=float, required=True)
    goto.add_argument("--z", type=float, required=True)
    goto.add_argument("--ttl", type=float, default=30.0)
    goto.add_argument("--pose-source", default="lio_estimate")
    _add_wait_options(goto)
    goto.set_defaults(handler=_cmd_goto)

    yaw = subparsers.add_parser(
        "yaw", help="in-place scan turn", parents=[common]
    )
    yaw.add_argument("--delta-deg", type=float, required=True)
    yaw.add_argument("--ttl", type=float, default=30.0)
    yaw.add_argument("--pose-source", default="lio_estimate")
    _add_wait_options(yaw)
    yaw.set_defaults(handler=_cmd_yaw)

    maneuver = subparsers.add_parser(
        "maneuver", help="short bounded body-frame local move", parents=[common]
    )
    maneuver.add_argument("--forward", type=float, default=0.0)
    maneuver.add_argument("--lateral", type=float, default=0.0)
    maneuver.add_argument("--yaw-rate", type=float, default=0.0)
    maneuver.add_argument("--duration", type=float, required=True)
    maneuver.add_argument("--ttl", type=float, default=15.0)
    maneuver.add_argument("--pose-source", default="lio_estimate")
    _add_wait_options(maneuver)
    maneuver.set_defaults(handler=_cmd_maneuver)

    scan = subparsers.add_parser(
        "scan", help="turn in place and snapshot observations", parents=[common]
    )
    scan.add_argument("--steps", type=int, default=4)
    scan.add_argument("--step-deg", type=float, default=90.0)
    scan.add_argument("--timeout-per-step", type=float, default=20.0)
    scan.add_argument("--settle-s", type=float, default=1.0)
    scan.add_argument("--ttl", type=float, default=30.0)
    scan.add_argument("--pose-source", default="lio_estimate")
    scan.set_defaults(handler=_cmd_scan)

    for action in ("cancel", "stop"):
        simple = subparsers.add_parser(
            action, help=action + " the current waypoints", parents=[common]
        )
        simple.set_defaults(handler=_cmd_simple(action))

    observe = subparsers.add_parser(
        "observe", help="read the observation bundle", parents=[common]
    )
    observe.add_argument("--receipts", type=int, default=8)
    observe.add_argument("--full", action="store_true")
    observe.set_defaults(handler=_cmd_observe)

    mapping = subparsers.add_parser(
        "map", help="operator-side occupancy snapshot", parents=[common]
    )
    mapping.add_argument("--cloud-topic", default="/dynanav/lidar_filtered")
    mapping.add_argument("--odometry-topic", required=True)
    mapping.add_argument("--clock-topic", default="/clock")
    mapping.add_argument("--pose-source", default="lio_estimate")
    mapping.add_argument("--sensor-frame", default="go2_imu")
    mapping.add_argument("--base-frame", default="go2_base")
    mapping.add_argument(
        "--imu-mount",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help=(
            "measured IMU origin in the base frame (metres); required only when "
            "the odometry child frame is go2_imu, so T_map_base is computed once "
            "instead of labelling an IMU pose as the base pose"
        ),
    )
    mapping.add_argument("--extrinsic-translation", type=float, nargs=3)
    mapping.add_argument(
        "--extrinsic-quaternion", type=float, nargs=4, default=[0.0, 0.0, 0.0, 1.0]
    )
    mapping.add_argument(
        "--free-mode", choices=("auto", "floor", "rays", "both", "none"),
        default="auto",
        help="auto uses floor endpoints for lio_deskewed_map and legacy both "
        "for sensor_imu; LIO rays/both are refused without a physical ray-origin contract",
    )
    mapping.add_argument("--resolution", type=float, default=0.05)
    mapping.add_argument("--duration", type=float, default=10.0)
    mapping.add_argument(
        "--snapshot-interval",
        type=float,
        default=2.0,
        help="seconds between partial operator-map snapshots; 0 writes only at exit",
    )
    mapping.add_argument("--frontier-limit", type=int, default=200)
    mapping.add_argument(
        "--export-grid",
        type=Path,
        default=None,
        help="optional directory for real operator grid cell exports (.npz plus "
        "JSON manifest); default off, read-only diagnostic",
    )
    mapping.add_argument("--out", type=Path)
    mapping.add_argument(
        "--cloud-semantics",
        choices=("sensor_imu", "lio_deskewed_map"),
        default="sensor_imu",
        help=(
            "sensor_imu keeps the legacy sensor-frame raw-cloud path; "
            "lio_deskewed_map consumes FAST-LIVO2 native deskewed "
            "/cloud_registered_full with the same-stage /aft_mapped_to_init pose "
            "(exact integer stamp pair, no interpolation) and writes the "
            "LiDAR direction-coverage perception sample"
        ),
    )
    mapping.add_argument(
        "--perception-out",
        type=Path,
        default=None,
        help=(
            "write the LiDAR direction-coverage perception.json consumed by the "
            "interactive tracker; only for --cloud-semantics lio_deskewed_map"
        ),
    )
    mapping.add_argument(
        "--nearfield-depth-manifest", type=Path, default=None,
        help="optional nearfield_depth/latest.json; validate exact native camera/LIO "
        "stamp and raw pixels for diagnostics only unless fusion is separately enabled",
    )
    mapping.add_argument(
        "--enable-async-depth-fusion", action="store_true",
        help="EXPERIMENTAL, default off: one bounded auxiliary depth-fusion worker; "
             "requires explicit native LIO + depth fusion, no runtime acceptance implied",
    )
    mapping.add_argument(
        "--enable-nearfield-depth-fusion", action="store_true",
        help="enable validated safety-only depth endpoints after zero-motion sensor "
        "validation; default off, never changes the native RTX SLAM/operator map",
    )
    mapping.add_argument(
        "--binary-prefix",
        default=None,
        help="native build prefix whose core/mapping sha256 are recorded in every receipt",
    )
    mapping.set_defaults(handler=_cmd_map)
    return parser


def main(argv=None):
    """Entry point; returns the documented exit code (0, 1, 2 or 3)."""
    parser = build_parser()
    args = parser.parse_args(argv)
    # The selector uses SUPPRESS defaults, so it may be absent from the namespace
    # depending on which side of the subcommand it was given.
    args.run_dir = getattr(args, "run_dir", None)
    args.mailbox = getattr(args, "mailbox", None)
    args.run_id = getattr(args, "run_id", None)
    if args.mailbox is None and args.run_dir is None:
        parser.error("either --run-dir or --mailbox is required")
    try:
        return args.handler(args)
    except (FileNotFoundError, ValueError, PermissionError) as error:
        print(json.dumps({"error": type(error).__name__, "message": str(error)}))
        return EXIT_USAGE


if __name__ == "__main__":
    raise SystemExit(main())
