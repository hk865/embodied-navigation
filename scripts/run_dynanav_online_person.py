#!/usr/bin/env python3
"""Supervise one online-person DynaNav run: worker -> LIVO observer -> capture.

The three-terminal recipe in ``docs/experiments/dynanav-online-person-20261109.md``
requires the person worker to be ready first, then the LIVO observer with its
``ready.json`` handshake and ``launch.json`` receipt, then capture bound to that receipt. This script keeps
that order, stops every child it started (capture first) on success, failure,
timeout or SIGINT/SIGTERM, keeps one log per child under ``<run>/logs/`` and
writes ``supervision.json`` with argv, pids, exit codes and the error, if any.

The manual recipe sources ROS Jazzy in each terminal; the supervisor sources the
same ``--ros-setup`` file (default ``/opt/ros/jazzy/setup.bash``) and exports the
result to all children. ``--no-ros-setup`` keeps the inherited environment.

``--check`` validates repository files and the paths you supplied without
starting ROS, Isaac or a model (exit 0 ready / 2 not ready, mirroring
``start.py --check``); it does not create the run directory. ``--print-commands``
prints the equivalent shell commands and also creates nothing.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORKER_SCRIPT = ROOT / "scripts/run_dynanav_person_worker.py"
OBSERVER_SCRIPT = ROOT / "scripts/run_dynanav_lio.sh"
CAPTURE_SCRIPT = ROOT / "scripts/run_dynanav_experiment.py"
DEFAULT_WEIGHTS = "models/perception/maskrcnn_resnet50_fpn_v2_coco-73cbd019.pth"
DEFAULT_WEIGHTS_SHA256 = (
    "73cbd0190fcbe3ba339921fbce2c3a0b6bb9126c9a133c85e43a2a8e060a109e"
)
DEFAULT_BINARY_PREFIX = "build_cache/dynanav_post_vio_20261109T052245Z"
DEFAULT_ROS_SETUP = "/opt/ros/jazzy/setup.bash"
MISSING_CAPTURE_RESULT = "<required: --capture-result>"
MISSING_SLAM_REFERENCE = "<required: --slam-reference>"
LOG_DIR = "logs"


class ChildExit(RuntimeError):
    """A child process exited or failed while the supervisor expected it alive."""


class Interrupted(RuntimeError):
    """A termination signal arrived; the supervisor stops every child."""

    def __init__(self, signum: int):
        super().__init__("received signal " + str(signum))
        self.signum = signum


class Child:
    """One spawned process plus its log file and supervision record."""

    def __init__(self, name: str, argv: list, process, log_path: Path, stream):
        self.name = name
        self.argv = argv
        self.process = process
        self.log_path = log_path
        self.stream = stream
        self.exit_code = None
        self.stopped_by_supervisor = False

    def poll(self):
        return self.process.poll()

    @property
    def returncode(self):
        return self.process.returncode

    def record(self) -> dict:
        return {
            "name": self.name,
            "pid": self.process.pid,
            "argv": self.argv,
            "log": str(self.log_path),
            "exit_code": self.exit_code,
            "stopped_by_supervisor": self.stopped_by_supervisor,
        }


def project_data() -> Path:
    """``<PROJECT_DATA_ROOT>/embodied-navigation``, failing closed when unset."""
    root = os.environ.get("PROJECT_DATA_ROOT")
    if not root:
        raise SystemExit(
            "PROJECT_DATA_ROOT is unset; pass --data-root or set the environment"
        )
    return Path(root).expanduser().resolve() / "embodied-navigation"


def snapshot_parameter_file(run: Path) -> Path:
    """Bounded native keyframe snapshot parameters for the LIVO observer."""
    return run / "livo" / "snapshots.yaml"


def snapshots_yaml(run: Path) -> str:
    observer = run / "livo" / "observer"
    return (
        "/**:\n"
        "  ros__parameters:\n"
        '    backend.snapshot_directory: "' + str(observer / "native_keyframes") + '"\n'
        "    backend.max_ready_directories: 256\n"
        "    backend.min_interval_s: 1.0\n"
        "    backend.max_interval_s: 2.0\n"
    )


def ros_environment(setup: Path | None, base: dict | None = None) -> dict:
    """Source a ROS ``setup.bash`` once and return the resulting environment.

    ``None`` keeps the inherited environment (``--no-ros-setup``). A missing or
    failing setup file is an explicit error, not a silent fallback: the manual
    recipe sources ROS in every terminal, and a supervisor without it would run
    children with a different environment.
    """
    environment = dict(os.environ if base is None else base)
    if setup is None:
        return environment
    if not setup.is_file():
        raise RuntimeError(
            "ROS setup file not found: "
            + str(setup)
            + " (pass --ros-setup or --no-ros-setup explicitly)"
        )
    completed = subprocess.run(
        ["bash", "-c", 'set -e; source "$1" >/dev/null 2>&1; env -0', "_", str(setup)],
        capture_output=True,
        check=False,  # Inspect returncode below to include the setup failure context.
        env=environment,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "sourcing "
            + str(setup)
            + " failed with exit "
            + str(completed.returncode)
            + ": "
            + completed.stderr.decode("utf-8", "replace").strip()[:500]
        )
    sourced = {}
    for item in completed.stdout.decode("utf-8", "replace").split("\0"):
        key, separator, value = item.partition("=")
        if separator and key:
            sourced[key] = value
    environment.update(sourced)
    return environment


def child_environment(domain: int, data: Path, ros_env: dict | None = None) -> dict:
    """Environment shared by every child; ``data`` is the project data directory."""
    env = dict(os.environ if ros_env is None else ros_env)
    env["PROJECT_DATA_ROOT"] = str(Path(data).resolve().parent)
    env["PYTHONPATH"] = str(ROOT / "src") + (
        os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else ""
    )
    env["ROS_DOMAIN_ID"] = str(domain)
    env.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    env.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")
    return env


def build_plan(args) -> dict:
    """Pure plan: paths, the three argv lists and readiness targets."""
    run = Path(args.output).expanduser().resolve()
    data = Path(args.data_root).expanduser().resolve()
    channel = run / "person-worker"
    observer = run / "livo" / "observer"
    capture = run / "capture"
    launch_receipt = observer / "launch.json"
    weights = Path(args.weights).expanduser().resolve()
    binary_prefix = Path(args.binary_prefix).expanduser().resolve()
    capture_result = (
        Path(args.capture_result).expanduser().resolve()
        if args.capture_result
        else None
    )
    slam_reference = (
        Path(args.slam_reference).expanduser().resolve()
        if args.slam_reference
        else None
    )
    worker = [
        args.python,
        str(WORKER_SCRIPT),
        "--channel",
        str(channel),
        "--weights",
        str(weights),
        "--expected-sha256",
        args.expected_sha256,
        "--device",
        args.device,
        "--tile-grid",
        str(args.tile_grid),
    ]
    observer_argv = [
        "bash",
        str(OBSERVER_SCRIPT),
        "--mode",
        "livo",
        "--binary-prefix",
        str(binary_prefix),
        "--observer-namespace",
        args.observer_namespace,
        "--lidar-topic",
        args.lidar_topic,
        "--capture-result",
        str(capture_result) if capture_result else MISSING_CAPTURE_RESULT,
        "--parameter-override",
        str(snapshot_parameter_file(run)),
        "--output-dir",
        str(observer),
    ]
    capture_argv = [
        args.python,
        str(CAPTURE_SCRIPT),
        "--output",
        str(capture),
        "--route",
        args.route,
        "--speed",
        str(args.speed),
        "--duration",
        str(args.duration),
        "--render-backend",
        "usd",
        "--imu-hz",
        str(args.imu_hz),
        "--archive-rgb",
        "--slam-reference",
        str(slam_reference) if slam_reference else MISSING_SLAM_REFERENCE,
        "--odometry-topic",
        args.odometry_topic,
        "--slam-capture-remaining-m",
        "0.05",
        "--slam-capture-distance-m",
        "0.10",
        "--slam-final-capture-remaining-m",
        "0.15",
        "--slam-final-capture-distance-m",
        "0.20",
        "--pedestrian",
        "--pedestrian-scenario",
        "spawn-crossing",
        "--semantic-provider",
        "learned_rgb_person",
        "--person-worker",
        str(channel),
        "--pose-launch-receipt",
        str(launch_receipt),
        "--learned-evaluation-oracle",
        "--person-max-interval",
        "0.4",
        "--person-min-interval",
        "0.2",
    ]
    return {
        "run": run,
        "data": data,
        "channel": channel,
        "observer": observer,
        "capture": capture,
        "launch_receipt": launch_receipt,
        "capture_result": capture_result,
        "slam_reference": slam_reference,
        "worker": worker,
        "observer_argv": observer_argv,
        "capture_argv": capture_argv,
    }


def check_entries(plan: dict, python: str, weights: Path, ros_setup: Path | None) -> list:
    """Read-only readiness entries; no ROS sourcing, no simulator, no model load."""
    run = plan["run"]
    entries = [
        ("worker script", WORKER_SCRIPT, WORKER_SCRIPT.is_file()),
        ("observer script", OBSERVER_SCRIPT, OBSERVER_SCRIPT.is_file()),
        ("capture script", CAPTURE_SCRIPT, CAPTURE_SCRIPT.is_file()),
        ("python", Path(python), Path(python).is_file() or shutil.which(python) is not None),
        ("weights", weights, weights.is_file()),
        ("data root", plan["data"], plan["data"].is_dir()),
        (
            "observer binary prefix",
            plan["observer_argv"][plan["observer_argv"].index("--binary-prefix") + 1],
            Path(
                plan["observer_argv"][plan["observer_argv"].index("--binary-prefix") + 1]
            ).is_dir(),
        ),
        (
            "capture result (observer calibration)",
            plan["capture_result"] if plan["capture_result"] else MISSING_CAPTURE_RESULT,
            bool(plan["capture_result"] and plan["capture_result"].is_file()),
        ),
        (
            "SLAM reference",
            plan["slam_reference"] if plan["slam_reference"] else MISSING_SLAM_REFERENCE,
            bool(plan["slam_reference"] and plan["slam_reference"].is_file()),
        ),
        (
            "ROS setup",
            ros_setup if ros_setup is not None else "<--no-ros-setup>",
            ros_setup is None or ros_setup.is_file(),
        ),
        ("run directory is new", run, not run.exists()),
    ]
    return [
        {"name": name, "value": str(value), "present": bool(present)}
        for name, value, present in entries
    ]


def check(plan: dict, python: str, weights: Path, ros_setup: Path | None) -> int:
    entries = check_entries(plan, python, weights, ros_setup)
    ready = all(entry["present"] for entry in entries)
    print(
        json.dumps(
            {
                "stage": "check",
                "ready": ready,
                "run_dir": str(plan["run"]),
                "entries": entries,
                "note": "checks files and directories only; no ROS, Isaac, GPU or model load",
            },
            indent=2,
        )
    )
    return 0 if ready else 2


def shell_commands(plan: dict, domain: int, ros_setup: Path | None) -> str:
    lines = []
    if ros_setup is not None:
        lines.append("source " + shlex_quote(str(ros_setup)))
    lines += [
        "export PROJECT_DATA_ROOT=" + shlex_quote(str(plan["data"].parent)),
        'export PYTHONPATH="' + str(ROOT / "src") + '${PYTHONPATH:+:$PYTHONPATH}"',
        "export ROS_DOMAIN_ID=" + str(domain),
        "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
        "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4",
        'DYNANAV_RUN="' + str(plan["run"]) + '"',
        "mkdir -p " + shlex_quote(str(snapshot_parameter_file(plan["run"]).parent)),
    ]
    env = "\n".join(lines) + "\n"
    return (
        env
        + "\n# 1) worker (must become ready first)\n"
        + " ".join(shlex_quote(part) for part in plan["worker"])
        + "\n\n# 2) LIVO observer\ncat > "
        + shlex_quote(str(snapshot_parameter_file(plan["run"])))
        + " <<'EOF'\n"
        + snapshots_yaml(plan["run"])
        + "EOF\n"
        + " ".join(shlex_quote(part) for part in plan["observer_argv"])
        + "\n\n# 3) capture (starts Isaac; needs the observer launch receipt)\n"
        + " ".join(shlex_quote(part) for part in plan["capture_argv"])
        + "\n"
    )


def shlex_quote(value: str) -> str:
    return shlex.quote(str(value))


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def spawn(name: str, argv: list, env: dict, run: Path) -> Child:
    log_path = run / LOG_DIR / (name + ".log")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    stream = log_path.open("wb")
    try:
        process = subprocess.Popen(
            argv,
            cwd=str(ROOT),
            env=env,
            start_new_session=True,
            stdout=stream,
            stderr=subprocess.STDOUT,
        )
    except BaseException:
        stream.close()
        raise
    return Child(name, argv, process, log_path, stream)


def wait_ready(path: Path, timeout: float, child: Child, expect_state=None, others=()) -> None:
    """Wait for a readiness artifact, failing fast when the child exits."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for dependency in (child, *others):
            if dependency.poll() is not None:
                raise ChildExit(
                    dependency.name + " exited with " + str(dependency.returncode) + " before readiness"
                )
        if expect_state is None:
            if path.is_file():
                return
        else:
            try:
                state = json.loads(path.read_text())
            except (OSError, ValueError):
                state = {}
            if isinstance(state, dict) and state.get("state") == expect_state:
                return
        time.sleep(0.1)
    raise TimeoutError(child.name + " did not become ready within " + str(timeout) + "s")


def monitor_capture(capture: Child, others: list, timeout: float) -> int:
    """Wait for capture, failing when a supporting child dies or capture times out.

    Supporting children are polled *before* capture: when capture and a worker
    exit between two polls, a capture that happens to report 0 first must not
    mask the dead worker/observer.
    """
    deadline = time.monotonic() + timeout if timeout else None
    while True:
        dead = [other for other in others if other.poll() is not None]
        if dead:
            raise ChildExit(
                ", ".join(
                    other.name + " exited with " + str(other.returncode) for other in dead
                )
                + " while capture was running"
            )
        code = capture.poll()
        if code is not None:
            return code
        if deadline is not None and time.monotonic() > deadline:
            raise TimeoutError("capture exceeded --capture-timeout")
        time.sleep(0.2)


def validate_capture_result(path: Path) -> None:
    """Exit 0 is insufficient: require this run's truthful terminal receipt.

    Older capture receipts used ``stage``. Accept that only when ``outcome``
    is absent; a conflicting or incomplete outcome must never be overridden.
    """
    try:
        result = json.loads(path.read_text())
    except (OSError, ValueError) as failure:
        raise RuntimeError("capture exited 0 without a readable terminal result: " + str(path)) from failure
    if not isinstance(result, dict):
        raise RuntimeError("capture exited 0 but result.json is not an object")
    outcome = result.get("outcome", result.get("stage"))
    if outcome != "completed":
        raise RuntimeError(
            "capture exited 0 but result.json is not completed: " + repr(outcome)
        )


def stop(child: Child | None, grace: float = 10.0) -> None:
    """SIGINT the child's process group, then SIGKILL; record the result."""
    if child is None:
        return
    # The group may still contain grandchildren after its leader has exited.
    # Never skip it just because poll() returned an exit code.
    child.stopped_by_supervisor = child.poll() is None
    def signal_group(signum):
        try:
            os.killpg(child.process.pid, signum)
            return True
        except ProcessLookupError:
            return False

    def live_group_members():
        # Orphan zombies cannot run and may remain until the host's PID 1
        # reaps them. Do not burn the entire grace period waiting on those.
        if not signal_group(0):
            return False
        if not Path("/proc/self/stat").exists():
            return True
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
                if int(fields[2]) == child.process.pid and fields[0] != "Z":
                    return True
            except (OSError, ValueError, IndexError):
                continue
        return False

    try:
        if signal_group(signal.SIGINT):
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline:
                child.poll()  # Reap the leader while allowing descendants to stop.
                if not live_group_members():
                    break
                time.sleep(0.1)
            signal_group(signal.SIGKILL)
        child.exit_code = child.process.wait(timeout=grace)
    finally:
        child.stream.close()


def final_verdict(capture: Child | None, children: list, error, interrupted):
    """Decide success; a supporting child death must never be masked by capture 0.

    ``stopped_by_supervisor`` is set by :func:`stop`, so a normal post-capture
    SIGINT cleanup is not a failure; a worker/observer that exited on its own is.
    """
    unexpected = [
        {"name": child.name, "exit_code": child.exit_code}
        for child in children
        if child.name in ("worker", "observer") and not child.stopped_by_supervisor
    ]
    if error is None and interrupted is None and unexpected:
        error = "; ".join(
            item["name"] + " exited with " + str(item["exit_code"]) + " before the run finished"
            for item in unexpected
        )
    ok = (
        error is None
        and interrupted is None
        and capture is not None
        and capture.exit_code == 0
        and not unexpected
    )
    return ok, unexpected, error


def _signal_handler(signum, _frame):
    raise Interrupted(signum)


def supervise(plan: dict, args) -> int:
    run = plan["run"]
    setup = None if args.no_ros_setup else Path(args.ros_setup).expanduser().resolve()
    # Resolve the environment before creating the run directory: a missing or
    # failing ROS setup must not leave a half-created run that looks usable.
    env = child_environment(args.domain, plan["data"], ros_environment(setup))
    run.mkdir(parents=True, exist_ok=False)
    snapshot_parameter_file(run).parent.mkdir(parents=True, exist_ok=True)
    snapshot_parameter_file(run).write_text(snapshots_yaml(run))
    children = []
    capture = None
    error = None
    interrupted = None
    started = time.time()
    previous = {}
    cleanup_errors = []

    def start(name, argv):
        # Defer cancellation until registration without blocking OS signals:
        # a blocked mask would be inherited by the child across exec().
        pending = []
        saved = {signum: signal.signal(signum, lambda sig, _frame: pending.append(sig))
                 for signum in (signal.SIGINT, signal.SIGTERM)}
        try:
            child = spawn(name, argv, env, run)
            children.append(child)
        finally:
            for signum, handler in saved.items():
                signal.signal(signum, handler)
        if pending:
            raise Interrupted(pending[0])
        return child

    def during_cleanup(signum, _frame):
        nonlocal interrupted
        if interrupted is None:
            interrupted = signum

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.signal(signum, _signal_handler)
    try:
        worker = start("worker", plan["worker"])
        wait_ready(plan["channel"] / "state.json", args.startup_timeout, worker, "ready")
        observer = start("observer", plan["observer_argv"])
        wait_ready(plan["observer"] / "ready.json", args.startup_timeout, observer,
                   "ready", others=[worker])
        # launch.json is provenance, not readiness; require both artifacts.
        receipt = json.loads(plan["launch_receipt"].read_text())
        if not isinstance(receipt, dict):
            raise RuntimeError("observer launch receipt must be an object")
        capture = start("capture", plan["capture_argv"])
        capture_code = monitor_capture(capture, [worker, observer], args.capture_timeout)
        if capture_code == 0:
            validate_capture_result(plan["capture"] / "result.json")
    except Interrupted as signal_error:
        interrupted = signal_error.signum
    except BaseException as child_error:  # noqa: BLE001 - record, then always clean up
        error = repr(child_error)
    finally:
        for signum in previous:
            signal.signal(signum, during_cleanup)
        for child in reversed(children):
            try:
                stop(child)
            except BaseException as cleanup_error:
                cleanup_errors.append(child.name + ": " + repr(cleanup_error))
        if cleanup_errors:
            error = "; ".join(([error] if error else []) + cleanup_errors)
    ok, unexpected, error = final_verdict(capture, children, error, interrupted)
    try:
        write_json(
            run / "supervision.json",
            {
                "schema": "dynanav_online_supervision/v3",
                "run_dir": str(run),
                "domain": args.domain,
                "started_unix": started,
                "elapsed_s": round(time.time() - started, 3),
                "ros_setup": str(setup) if setup is not None else None,
                "effective_environment": {
                    key: env.get(key)
                    for key in (
                        "RMW_IMPLEMENTATION", "FASTDDS_BUILTIN_TRANSPORTS",
                        "DYNANAV_PUBLISH_TRACE", "FASTLIVO_INGRESS_TRACE",
                    )
                },
                "processes": [child.record() for child in children],
                "capture_exit_code": capture.exit_code if capture is not None else None,
                "unexpected_exits": unexpected,
                "cleanup_errors": cleanup_errors,
                "interrupted": interrupted,
                "error": error,
                "ok": ok,
                "scope": (
                    "technical process supervision only; not navigation or dynamic-perception acceptance"
                ),
            },
        )
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
    if interrupted is not None:
        return 128 + interrupted
    return 0 if ok else 1


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True, help="New run directory")
    p.add_argument("--data-root", type=Path)
    p.add_argument("--python", default=sys.executable, help="Interpreter for worker/capture")
    p.add_argument("--domain", type=int, default=116)
    p.add_argument("--route", default="basement-to-ground")
    p.add_argument("--speed", type=float, default=0.30)
    p.add_argument("--duration", type=float, default=45.0)
    p.add_argument("--imu-hz", type=int, choices=(100, 200), default=200)
    p.add_argument("--tile-grid", type=int, choices=(1, 2), default=1)
    p.add_argument("--device", default="cuda:0")
    p.add_argument("--weights")
    p.add_argument("--expected-sha256", default=DEFAULT_WEIGHTS_SHA256)
    p.add_argument("--binary-prefix")
    p.add_argument("--capture-result")
    p.add_argument("--slam-reference")
    p.add_argument("--observer-namespace", default="/dynanav/livo_navigation")
    p.add_argument("--lidar-topic", default="/dynanav/lidar_perception_filtered")
    p.add_argument("--odometry-topic", default="/dynanav/livo_navigation/post_vio_odometry")
    p.add_argument("--ros-setup", default=DEFAULT_ROS_SETUP)
    p.add_argument(
        "--no-ros-setup",
        action="store_true",
        help="do not source a ROS setup; children inherit this environment",
    )
    p.add_argument("--startup-timeout", type=float, default=120.0)
    p.add_argument(
        "--capture-timeout", type=float, default=0.0, help="0 disables the wall limit"
    )
    p.add_argument("--check", action="store_true")
    p.add_argument("--print-commands", action="store_true")
    return p


def main(argv=None) -> int:
    p = parser()
    args = p.parse_args(argv)
    if not (args.check or args.print_commands) and not (
        args.capture_result and args.slam_reference
    ):
        p.error("--capture-result and --slam-reference are required to run")
    data = (
        Path(args.data_root).expanduser().resolve()
        if args.data_root
        else project_data()
    )
    args.data_root = data
    args.weights = (
        Path(args.weights).expanduser().resolve()
        if args.weights
        else (data / DEFAULT_WEIGHTS).resolve()
    )
    args.binary_prefix = (
        Path(args.binary_prefix).expanduser().resolve()
        if args.binary_prefix
        else (data / DEFAULT_BINARY_PREFIX).resolve()
    )
    plan = build_plan(args)
    setup = None if args.no_ros_setup else Path(args.ros_setup).expanduser().resolve()
    if args.print_commands:
        sys.stdout.write(shell_commands(plan, args.domain, setup))
        return 0
    if args.check:
        return check(plan, args.python, args.weights, setup)
    try:
        return supervise(plan, args)
    except RuntimeError as failure:
        print("supervisor: " + str(failure), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
