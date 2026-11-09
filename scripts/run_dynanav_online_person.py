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

# 中文模块说明（首轮补充；上方 docstring 恢复 HEAD 原文，以保持 --help/--doc 文案不变）：
# 监督一次在线行人 DynaNav 运行：worker -> LIVO observer -> capture。
#
# ``docs/experiments/dynanav-online-person-20261109.md`` 的三终端流程要求行人
# worker 先就绪，再启动带 ``ready.json`` 握手和 ``launch.json`` 收据的 LIVO
# observer，最后启动绑定该收据的 capture。本脚本保持该顺序，在成功、失败、
# 超时或收到 SIGINT/SIGTERM 时停止它启动的每个子进程（先停 capture），
# 为每个子进程在 ``<run>/logs/`` 下保留一份日志，并把 argv、pid、退出码与
# 错误写入 ``supervision.json``。
#
# 手工流程会在每个终端 source ROS Jazzy；监督器 source 同一个 ``--ros-setup``
# 文件（默认 ``/opt/ros/jazzy/setup.bash``）并把结果传给所有子进程；
# ``--no-ros-setup`` 表示沿用继承环境。
#
# ``--check`` 只校验仓库文件与传入路径，不启动 ROS/Isaac/模型
# （就绪返回 0、不就绪返回 2，与 ``start.py --check`` 一致），且不创建
# run 目录；``--print-commands`` 只打印等价 shell 命令，同样不产生任何文件。


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
    """在监督器仍期望子进程存活时，该子进程提前退出或失败。"""


class Interrupted(RuntimeError):
    """收到终止信号；监督器据此停止所有子进程并记录信号号。"""

    def __init__(self, signum: int):
        super().__init__("received signal " + str(signum))
        self.signum = signum


class Child:
    """一个被拉起的子进程及其日志文件、监督记录。

    ``exit_code`` 在 ``stop()`` 中回填；``stopped_by_supervisor`` 标记该退出
    是否由监督器主动触发，用于区分正常收尾与异常早退。
    """

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
    """返回 ``<PROJECT_DATA_ROOT>/embodied-navigation``；未设置时直接失败。

    数据根目录必须显式给出（--data-root 或环境变量），不做隐式回退。
    """
    root = os.environ.get("PROJECT_DATA_ROOT")
    if not root:
        raise SystemExit(
            "PROJECT_DATA_ROOT is unset; pass --data-root or set the environment"
        )
    return Path(root).expanduser().resolve() / "embodied-navigation"


def snapshot_parameter_file(run: Path) -> Path:
    """LIVO observer 的有界原生关键帧快照参数文件路径。

    路径固定为 ``<run>/livo/snapshots.yaml``，由本脚本写入后经
    ``--parameter-override`` 传给 observer。
    """
    return run / "livo" / "snapshots.yaml"


def snapshots_yaml(run: Path) -> str:
    # 快照目录落在 observer 输出下；256 个就绪目录上限、1.0–2.0 s 的最小/最大
    # 间隔用于限制原生关键帧快照的落盘量与频率（单位：秒）。
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
    """source 一次 ROS ``setup.bash`` 并返回结果环境。

    ``None`` 表示沿用继承环境（--no-ros-setup）。setup 文件缺失或 source
    失败都是显式错误而非静默回退：手工流程每个终端都会 source ROS，
    监督器若跳过会让子进程运行在不同环境里。子进程以 ``env -0`` 的形式
    取回完整环境，避免逐行解析 shell 引号。
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
        # 不在 subprocess 层抛错：下面统一检查 returncode，把 stderr 尾部带进异常。
        check=False,
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
    """所有子进程共享的环境；``data`` 是项目数据目录（其父目录即 PROJECT_DATA_ROOT）。

    PYTHONPATH 前置本仓库 ``src``；ROS_DOMAIN_ID 由 --domain 指定；
    RMW/内置传输只在未设置时补默认值（rmw_fastrtps_cpp、UDPv4），不覆盖调用方环境。
    """
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
    """纯规划：路径、三段 argv 与就绪目标，不产生任何副作用。

    返回的 argv 直接交给 Popen；``--slam-capture-*`` 的 0.05/0.10/0.15/0.20 m
    与行人间隔 0.2–0.4 s 是本次在线采集的硬编码实验参数，不对应 CLI 开关。
    """
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
    """只读就绪检查项：不 source ROS、不启动仿真、不加载模型。

    每项返回 name/value/present；present 只表示该文件/目录当前存在，
    不代表内容有效或系统可运行。
    """
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
    # 输出一行 JSON 便于机器读取；0=就绪、2=不就绪，与 start.py --check 一致。
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
    # 只用于人工复现：打印的顺序与 supervise() 一致（worker -> observer ->
    # capture），并显式写出快照 yaml 的生成步骤；不执行任何命令。
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
    # 先写 .tmp 再原子替换；allow_nan=False 让非法浮点直接失败，而不是写出
    # 非标准 JSON 让下游解析器产生歧义。
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def spawn(name: str, argv: list, env: dict, run: Path) -> Child:
    # start_new_session=True 让子进程成为独立进程组组长，stop() 才能用
    # killpg 连后代一起收；stdout/stderr 合并写 <run>/logs/<name>.log。
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
    """等待就绪产物，子进程提前退出时立即失败。

    每 0.1 s 轮询一次；``expect_state`` 非空时要求 JSON 的 state 字段等于它，
    否则只要求文件存在。``others`` 中的支撑进程也会被检查，避免等待期间
    另一个关键子进程已死而无人发现。
    """
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
    """等待 capture 结束；支撑子进程死亡或 capture 超时都算失败。

    先轮询 worker/observer 再轮询 capture：若两者在两次轮询之间先后退出，
    capture 恰好先报 0 也不能掩盖已经死掉的支撑进程。
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
    """退出码 0 不足以判定成功：必须读到本次运行真实的终态收据。

    旧版 capture 收据用 ``stage`` 字段；仅在 ``outcome`` 缺失时才接受
    ``stage``，二者冲突或终态不完整时绝不覆盖为成功。
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
    """先向子进程组发 SIGINT，宽限期后发 SIGKILL，并记录结果。

    即使组长已退出也要处理整个进程组（可能仍有孙进程）；``grace`` 为
    SIGINT 后等待后代退出的秒数（默认 10 s），随后无条件 SIGKILL。
    """
    if child is None:
        return
    # 组长退出后组内仍可能有孙进程：不能因为 poll() 已有退出码就跳过清理。
    # stopped_by_supervisor 只记录“本次 stop 是否为主动停止”。
    child.stopped_by_supervisor = child.poll() is None
    def signal_group(signum):
        try:
            os.killpg(child.process.pid, signum)
            return True
        except ProcessLookupError:
            return False

    def live_group_members():
        # 孤儿僵尸进程不会再运行，可能一直留到宿主 PID 1 回收；因此判断存活时
        # 排除 Z 状态，避免把整个宽限期耗在等待僵尸上。
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
    """判定最终成功；支撑子进程死亡绝不能被 capture 的 0 退出码掩盖。

    ``stopped_by_supervisor`` 由 ``stop()`` 设置，因此 capture 结束后正常
    SIGINT 收尾不算失败；worker/observer 自行退出才算异常早退。
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
    # 运行期把 SIGINT/SIGTERM 转成 Interrupted，让 finally 统一清理。
    raise Interrupted(signum)


def supervise(plan: dict, args) -> int:
    run = plan["run"]
    setup = None if args.no_ros_setup else Path(args.ros_setup).expanduser().resolve()
    # 先解析环境再创建 run 目录：ROS setup 缺失或失败时不能留下一个看起来
    # 可用的半成品 run。run 目录用 exist_ok=False，重复运行会直接报错。
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
        # 在 spawn 期间把 SIGINT/SIGTERM 暂存到 pending 而不是屏蔽信号：
        # 被屏蔽的信号掩码会跨 exec() 继承给子进程。注册完成后再抛出，
        # 保证子进程已进入 children 列表、会被 finally 清理。
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
        # launch.json 是来源证据而非就绪信号：必须同时具备 ready.json 才继续。
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
                # 明确边界：本收据只证明进程监督发生过，不是导航或动态感知验收。
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
    # 参数默认值即本次实验基线：domain=116、speed=0.30 m/s、duration=45 s、
    # imu=200 Hz、cuda:0、单 tile；--capture-timeout=0 表示不限墙钟。
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
    # --check/--print-commands 是纯只读模式，不要求两份输入文件；真正监督运行
    # 时必须同时给出 capture 结果与 SLAM 参考，否则直接参数错误退出。
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
