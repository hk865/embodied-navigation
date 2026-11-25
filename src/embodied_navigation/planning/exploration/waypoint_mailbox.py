"""Thread/process-safe mailbox transport, request validation, and receipt cursor."""

from __future__ import annotations

import fcntl
import json
import math
from pathlib import Path
import time
from typing import Any

from embodied_navigation.common.runtime_io import atomic_write_json
from embodied_navigation.common.state_contract import FORBIDDEN_POSE_SOURCES
from embodied_navigation.navigation.yaw_action import (
    LOCAL_ACTION_BOUNDS,
    yaw_request_parameters,
)

PROTOCOL_VERSION = 1
DEFAULT_MAILBOX_NAME = "interactive"

EXIT_OK = 0
EXIT_ACTION_FAILED = 1
EXIT_ACTION_TIMEOUT = 2
EXIT_USAGE = 3

ACTION_COMPLETION_EVENTS = (
    "yaw_complete",
    "maneuver_complete",
    "waypoint_reached",
    "cancel_accepted",
    "stop_accepted",
)

ACTION_FAILURE_STATUS = {
    "yaw_failed": "failed",
    "request_rejected": "rejected",
    "maneuver_rejected": "rejected",
    "waypoint_rejected": "rejected",
    "yaw_rejected": "rejected",
    "waypoint_dropped": "dropped",
    "waypoint_failed": "failed",
    "waypoint_expired": "expired",
    "action_expired": "expired",
    "action_cancelled": "cancelled",
    "action_preempted": "preempted",
}

GLOBAL_ACTION_STATUS = {"terminal": "terminal", "hold": "held"}


def mailbox_for(run_dir: str | Path, mailbox: str | Path | None = None) -> Path:
    """Resolve the mailbox directory for one run."""
    return Path(mailbox) if mailbox else Path(run_dir) / DEFAULT_MAILBOX_NAME


def read_json(path: str | Path, default: Any = None) -> Any:
    """Read a JSON file, returning default when it does not exist or is invalid."""
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return default


def run_id_for(mailbox: str | Path) -> str:
    """Run id used by the simulator for this mailbox (the run directory name)."""
    return Path(mailbox).parent.name


def state_path_for(mailbox: str | Path) -> Path:
    """Operator-owned sequence state, kept inside the mailbox for auditability."""
    return Path(mailbox) / "operator-state.json"


def inbox_dir_for(mailbox: str | Path) -> Path:
    """Ordered per-sequence request inbox directory."""
    return Path(mailbox) / "waypoint-inbox"


def lock_path_for(mailbox: str | Path) -> Path:
    """Advisory flock file serializing sequence allocation and inbox writes."""
    return Path(mailbox) / ".operator-state.lock"


def _max_inbox_sequence(mailbox: Path) -> int:
    inbox_dir = inbox_dir_for(mailbox)
    if not inbox_dir.is_dir():
        return 0
    highest = 0
    try:
        entries = list(inbox_dir.iterdir())
    except OSError:
        return 0
    for entry in entries:
        if entry.suffix != ".json" or not entry.stem.isdigit():
            continue
        highest = max(highest, int(entry.stem))
    return highest


def _next_sequence_unlocked(mailbox: Path) -> int:
    state = read_json(state_path_for(mailbox), {}) or {}
    last = state.get("last_sequence")
    if isinstance(last, bool) or not isinstance(last, int) or last < 0:
        last = 0
    last = max(last, _max_inbox_sequence(mailbox))
    return last + 1


def next_sequence(mailbox: str | Path) -> int:
    """Return the next strictly increasing request sequence number."""
    mailbox_path = Path(mailbox)
    if mailbox_path.is_dir():
        with lock_path_for(mailbox_path).open("a+", encoding="utf-8") as lock_handle:
            fcntl.flock(lock_handle.fileno(), fcntl.LOCK_SH)
            try:
                return _next_sequence_unlocked(mailbox_path)
            finally:
                fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
    return _next_sequence_unlocked(mailbox_path)


def _number(value: Any, name: str) -> float:
    """Return value as a finite float or raise."""
    if isinstance(value, bool):
        raise ValueError(name + " must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(name + " must be a finite number") from error
    if not math.isfinite(number):
        raise ValueError(name + " must be a finite number")
    return number


def _resolve_frozen_yaw_ceiling(mailbox: Path) -> float | None:
    exp_path = mailbox.parent / "experiment.json"
    if not exp_path.is_file():
        return None
    exp = read_json(exp_path)
    if not isinstance(exp, dict):
        return None
    yaw_section = exp.get("yaw_action")
    if isinstance(yaw_section, dict) and yaw_section.get("yaw_rate_rps") is not None:
        raw = yaw_section.get("yaw_rate_rps")
    else:
        raw = exp.get("interactive_yaw_max_rate_rps", exp.get("yaw_max_rate_rad_s"))
    if raw is None or isinstance(raw, bool) or not isinstance(raw, (int, float)):
        return None
    val = float(raw)
    if math.isfinite(val) and 0.0 < val <= 0.80:
        return val
    return None


def build_request(
    *,
    seq: int,
    action: str,
    goal: Any = None,
    ttl_s: float = 30.0,
    pose_source: str | None,
    run_id: str,
    created_wall_s: float,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Build one request payload, validating it before it reaches the mailbox."""
    if action not in ("goto", "waypoint", "cancel", "stop", "yaw", "maneuver"):
        raise ValueError("unsupported action")
    if isinstance(seq, bool) or not isinstance(seq, int) or seq <= 0:
        raise ValueError("seq must be a positive integer")
    if not math.isfinite(ttl_s) or not 1.0 <= ttl_s <= 120.0:
        raise ValueError("ttl_s must be within [1, 120] s")
    payload: dict[str, Any] = {
        "version": PROTOCOL_VERSION,
        "seq": seq,
        "action": action,
        "created_wall_s": float(created_wall_s),
        "ttl_s": float(ttl_s),
        "run_id": run_id,
    }
    extra = {} if extra is None else dict(extra)
    if "action_epoch" in extra and extra["action_epoch"] is not None:
        epoch = extra["action_epoch"]
        if type(epoch) is not int or epoch < 0:
            raise ValueError("action_epoch must be a non-negative integer")
        payload["action_epoch"] = epoch
    if "preempt" in extra:
        preempt = extra["preempt"]
        if type(preempt) is not bool:
            raise ValueError("preempt must be boolean")
        if preempt:
            if action not in ("goto", "waypoint", "yaw", "maneuver"):
                raise ValueError(
                    "preempt is only supported for goto, waypoint, yaw and maneuver"
                )
            payload["preempt"] = True
    if action in ("goto", "waypoint", "yaw", "maneuver"):
        if pose_source in FORBIDDEN_POSE_SOURCES:
            raise ValueError(
                "a ground-truth waypoint frame is forbidden; the simulator rejects it too"
            )
    if action == "yaw":
        payload.update(
            yaw_request_parameters(
                extra, max_yaw_rate_rps=extra.get("max_yaw_rate_rps")
            )
        )
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
        if abs(lateral) > LOCAL_ACTION_BOUNDS["lateral_mps"]:
            raise ValueError("body_lateral_mps must be within +/-0.05 m/s")
        if abs(yaw_rate) > LOCAL_ACTION_BOUNDS["yaw_rate_rps"]:
            raise ValueError("yaw_rate_rps must be within +/-0.15 rad/s")
        if not 0.1 <= duration <= LOCAL_ACTION_BOUNDS["maneuver_max_duration_s"]:
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
        reverse = extra.get("reverse", False)
        if type(reverse) is not bool:
            raise ValueError("reverse must be boolean")
        if reverse:
            payload["reverse"] = True
        payload["goal_xyz_m"] = xyz
        payload["pose_source"] = pose_source
        payload["map_frame_note"] = "live estimator map frame; operator-selected"
    return payload


def submit(
    mailbox: str | Path,
    *,
    action: str,
    goal: Any = None,
    ttl_s: float = 30.0,
    pose_source: str | None = None,
    run_id: str | None = None,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Serialize seq allocation and write both the ordered inbox entry and request.json."""
    mailbox_path = Path(mailbox)
    if not mailbox_path.is_dir():
        raise FileNotFoundError(f"mailbox does not exist: {mailbox_path}")

    resolved_extra = dict(extra) if extra is not None else {}
    if action == "yaw" and resolved_extra.get("max_yaw_rate_rps") is None:
        frozen_ceiling = _resolve_frozen_yaw_ceiling(mailbox_path)
        if frozen_ceiling is not None:
            resolved_extra["max_yaw_rate_rps"] = frozen_ceiling

    with lock_path_for(mailbox_path).open("a+", encoding="utf-8") as lock_handle:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX)
        try:
            seq = _next_sequence_unlocked(mailbox_path)
            payload = build_request(
                seq=seq,
                action=action,
                goal=goal,
                ttl_s=ttl_s,
                pose_source=pose_source,
                run_id=run_id if run_id is not None else run_id_for(mailbox_path),
                created_wall_s=time.perf_counter(),
                extra=resolved_extra or None,
            )
            payload["delivery"] = "inbox"
            inbox_dir = inbox_dir_for(mailbox_path)
            inbox_dir.mkdir(parents=True, exist_ok=True)
            atomic_write_json(inbox_dir / f"{seq:06d}.json", payload)
            atomic_write_json(mailbox_path / "request.json", payload)
            atomic_write_json(
                state_path_for(mailbox_path),
                {
                    "last_sequence": seq,
                    "last_action": action,
                    "submitted_wall_s": payload["created_wall_s"],
                },
            )
            return payload
        finally:
            fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)


def write_heartbeat(mailbox: str | Path, *, run_id: str | None = None) -> dict[str, Any]:
    """Refresh the operator liveness file the simulator enforces."""
    mailbox_path = Path(mailbox)
    payload = {
        "wall_s": time.perf_counter(),
        "run_id": run_id if run_id is not None else run_id_for(mailbox_path),
        "client": "dsh-explore-waypoint-client",
    }
    atomic_write_json(mailbox_path / "heartbeat.json", payload)
    return payload


def latest_sim_frame(run_dir: str | Path) -> dict[str, Any] | None:
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


def _receipt_run_matches(receipt: dict[str, Any], run_id: str) -> bool:
    """Whether a receipt belongs to this run; an unknown run id is accepted."""
    declared = receipt.get("run_id")
    if not isinstance(run_id, str) or not run_id:
        return True
    if declared is None:
        return True
    return isinstance(declared, str) and declared == run_id


def _same_sequence(receipt: dict[str, Any], seq: int) -> bool:
    """Whether a receipt names exactly this action's sequence number."""
    value = receipt.get("seq")
    return isinstance(value, int) and not isinstance(value, bool) and value == seq


def _action_result(
    receipt: dict[str, Any], status: str, seq: int, *, success: bool, run_id: str
) -> dict[str, Any]:
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


def _match_action_result(
    receipts: list[dict[str, Any]], seq: int, run_id: str
) -> dict[str, Any] | None:
    """Return this sequence's verdict from receipts, or None."""
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


def timeout_result(seq: int, timeout_s: float, run_id: str) -> dict[str, Any]:
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


class ReceiptCursor:
    """Incremental byte-offset cursor over receipts.jsonl (never truncates history)."""

    def __init__(self, mailbox: str | Path) -> None:
        self.path = Path(mailbox) / "receipts.jsonl"
        self.offset = 0
        self.remainder = b""
        self.receipts: list[dict[str, Any]] = []

    def poll(self) -> list[dict[str, Any]]:
        try:
            size = self.path.stat().st_size
        except OSError:
            return self.receipts
        if size < self.offset:
            self.offset = 0
            self.remainder = b""
            self.receipts.clear()
        try:
            with self.path.open("rb") as handle:
                handle.seek(self.offset)
                chunk = handle.read()
        except OSError:
            return self.receipts
        if not chunk:
            return self.receipts

        data = self.remainder + chunk
        self.offset += len(chunk)
        lines = data.split(b"\n")
        if data.endswith(b"\n"):
            self.remainder = b""
            complete_lines = lines[:-1]
        else:
            last = lines[-1]
            # If a writer omitted trailing newline but wrote a complete JSON dict, parse it;
            # otherwise keep partial bytes until newline arrives.
            parsed_last = None
            stripped = last.strip()
            if stripped:
                try:
                    candidate = json.loads(stripped.decode("utf-8"))
                    if isinstance(candidate, dict):
                        parsed_last = candidate
                except (UnicodeDecodeError, json.JSONDecodeError):
                    parsed_last = None
            if parsed_last is not None:
                complete_lines = lines
                self.remainder = b""
            else:
                complete_lines = lines[:-1]
                self.remainder = last

        for raw_line in complete_lines:
            text = raw_line.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            try:
                parsed = json.loads(text)
            except json.JSONDecodeError:
                continue
            if isinstance(parsed, dict):
                self.receipts.append(parsed)
        return self.receipts


def wait_for_action(
    mailbox: str | Path, seq: int, timeout_s: float, poll_s: float = 0.05
) -> dict[str, Any]:
    """Wait for the structured result of one action across the full receipt stream."""
    if timeout_s <= 0:
        raise ValueError("timeout_s must be positive")
    if isinstance(seq, bool) or not isinstance(seq, int) or seq <= 0:
        raise ValueError("seq must be a positive integer")
    run_id = run_id_for(mailbox)
    cursor = ReceiptCursor(mailbox)
    deadline = time.perf_counter() + timeout_s
    while True:
        receipts = cursor.poll()
        result = _match_action_result(receipts, seq, run_id)
        if result is not None:
            return result
        if time.perf_counter() >= deadline:
            return timeout_result(seq, timeout_s, run_id)
        time.sleep(poll_s)


def action_exit_code(result: dict[str, Any]) -> int:
    """Map one wait_for_action result to the documented CLI exit code."""
    if result.get("success"):
        return EXIT_OK
    if result.get("status") == "timeout":
        return EXIT_ACTION_TIMEOUT
    return EXIT_ACTION_FAILED


def read_observation(mailbox: str | Path) -> Any:
    """Return the latest observation bundle or None."""
    return read_json(Path(mailbox) / "observation.json")


def tail_receipts(mailbox: str | Path, count: int = 8) -> list[dict[str, Any]]:
    """Return the last count receipts as parsed objects; corruption is skipped."""
    path = Path(mailbox) / "receipts.jsonl"
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError):
        return []
    result: list[dict[str, Any]] = []
    selected = lines if count <= 0 else lines[-count:]
    for line in selected:
        line = line.strip()
        if not line:
            continue
        try:
            parsed = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            result.append(parsed)
    return result


def summarize_observation(observation: dict[str, Any] | None) -> dict[str, Any]:
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
            "current_epoch": waypoints.get("current_epoch"),
            "active_action_epoch": waypoints.get("active_action_epoch"),
            "action_epoch": waypoints.get("action_epoch"),
        },
        "last_command": observation.get("command", {}).get("last_request"),
        "link": link,
        "coverage": observation.get("coverage"),
        "map": observation.get("map", {}).get("provider"),
        "native_guards": observation.get("guards", {}).get("native"),
    }
