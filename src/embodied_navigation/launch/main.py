"""Normal navigation process lifecycle. Development acceptance is a separate tool."""

# 中文模块说明（首轮补充；上方 docstring 恢复 HEAD 原文，以保持 --help/--doc 文案不变）：
# 普通导航的进程生命周期入口；开发验收是另一个工具。
#
# 职责：读取合并配置、检查依赖、加锁、准备 run 目录、构造子进程环境、启动
# `ros2 launch`、监控并按序关停，最后把退出情况写进 run 目录。
# 边界：不实现任务状态机、控制律、停车判断或安全阈值；`--check` 只做文件级依赖
# 检查，通过不代表 ROS/仿真可运行。业务状态由各模块自己持有。


from pathlib import Path

import argparse
import datetime as dt
import fcntl
import json
import math
import os
import signal
import subprocess
import time
import uuid

from embodied_navigation.common.config import ROOT, read, path


def environment(directory, domain, profile, socket):
    """构造普通导航运行所起进程的环境变量。

    顺序从"事实来源"向外叠加：先 source ROS Jazzy 基础环境与项目 install，再由
    项目 DDS 配置选中项目 Fast DDS 库；随后删除继承来的传输类变量并按 profile
    的 `dds_transport`（来源 `configs/common/settings.json`）重新设置，最后把
    FastRTPS profile 文件固定到项目 XML。接着写入本次运行相关的值（domain、
    Gazebo partition、socket、run 目录、actuator 插件路径）、单线程数值库设置，
    以及来自 `profile["pipeline"]`（`configs/localization_mapping/settings.json`）
    的 FASTLIVO 流水线开关和少量硬编码开关；最后叠加 profile 的 `features`
    映射。探针开关随后被强制改回 `"0"`，因此 features 条目无法在启动时打开诊断
    探针。返回新字典，不修改传入的 profile。
    """
    from embodied_navigation.common.runtime_environment import environment as ros_environment
    from embodied_navigation.common.communication.dds_environment import configure

    env = configure(ros_environment("/opt/ros/jazzy/setup.bash"))
    transport = profile["dds_transport"]

    # 先删除继承来的 FastDDS/FastRTPS 变量，再应用项目传输设置；profile XML 在之后固定。
    for key in transport["remove_environment_keys"]:
        env.pop(key, None)
    env.update(transport["environment"])
    env["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(
        ROOT / "src/embodied_navigation/common/communication/dds_transport.xml"
    )

    # 本次运行相关的值：domain 来自 --domain，GZ_PARTITION 用 run 目录名隔离 Gazebo 传输。
    env.update(
        ROS_DOMAIN_ID=str(domain),
        GZ_IP="127.0.0.1",
        GZ_PARTITION="navigation_" + directory.name,
        TEACHER_SOCKET=socket,
        TEACHER_ACTUATOR_LOG="/dev/null",
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(ROOT / "install/actuators/lib"),
        DEMO_RUN_DIR=str(directory),
        ROS_LOG_DIR=str(directory / "ros_logs"),
        PYTHONDONTWRITEBYTECODE="1",
        # 将数值库固定为单线程，避免多个子进程各自开线程池互相抢占 CPU。
        OMP_NUM_THREADS="1",
        OPENBLAS_NUM_THREADS="1",
        MKL_NUM_THREADS="1",
        NUMEXPR_NUM_THREADS="1",
        # FASTLIVO_PIPELINE_MODE/IMAGE_COPY_OPT 来自配置，其余 FASTLIVO_* 与 NAVIGATION_RECORD_RAW 为硬编码。
        FASTLIVO_PIPELINE_MODE=profile["pipeline"]["mode"],
        FASTLIVO_IMAGE_COPY_OPT=str(profile["pipeline"]["image_copy_opt"]),
        FASTLIVO_SERIAL_IMU_PRIORITY="1",
        FASTLIVO_LIO_JACOBIAN_THREADS="4",
        FASTLIVO_VIO_PATCH_THREADS="4",
        FASTLIVO_SURFACE_VALIDITY="0",
        NAVIGATION_RECORD_RAW="0",
    )
    # features 最后应用，可以覆盖上面的固定值；但下面的探针开关会被强制改回 '0'。
    env.update(profile.get("features", {}))
    for key in [
        "R87_DIAGNOSTIC_PROBE",
        "R89_THREAD_PROBE",
        "R96_CONTROL_CPU_PROBE",
        "R99_CLOCK_PROBE",
        "R107_SCAN_TRANSPORT_PROBE",
        "R118_SOURCE_AGE_PROBE",
        "MATCH_QUERY_CAPTURE",
    ]:
        # 普通启动一律关闭诊断/捕获探针，避免配置误开影响时序；building 入口另有一套设置。
        env[key] = "0"
    return env


def preflight_resources():
    """可选的开发者资源门；在拿锁和建 run 目录之前执行。

    用 1 分钟平均负载与 `0.9 * cpu_count` 比较（`cpu_count` 返回 `None` 时按 1
    算），并要求 `MemAvailable` 至少 4 GiB（`/proc/meminfo` 的单位是 kB，这里乘
    1024 换算成字节）。任一条件不满足都抛 `RuntimeError`。只由 `--resource-check`
    触发，不读历史存储账本或授权文件。
    """
    # 负载门限为 0.9 × 逻辑 CPU 数；cpu_count() 为 None 时按 1 处理。
    if os.getloadavg()[0] > 0.9 * (os.cpu_count() or 1):
        raise RuntimeError("CPU load protection: launch postponed")
    fields = {
        line.split(":", 1)[0]: line.split(":", 1)[1].strip()
        for line in Path("/proc/meminfo").read_text().splitlines()
    }
    # 4 GiB = 4 * 1024^3 字节；/proc/meminfo 的 kB 值先乘 1024。
    if int(fields["MemAvailable"].split()[0]) * 1024 < 4 * 1024**3:
        raise RuntimeError("Available memory below 4GiB")


def parse_arguments(argv=None):
    """解析启动参数，并在产生副作用之前拒绝非法上限。

    `--domain` 必须在 1..232 闭区间内（作为 ROS_DOMAIN_ID 传给子进程）。
    `--duration` 可选；给出时是本次运行的墙钟上限（秒），必须有限且严格为正。
    两种违规都经 `parser.error` 退出（打印用法并返回退出码 2），发生在读配置、
    查依赖、建锁文件或创建进程/socket 之前。`--label` 缺省为 `navigation`，
    后续会被过滤成安全的目录名片段。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--domain", type=int, default=231)
    parser.add_argument("--label", default="navigation")
    parser.add_argument(
        "--resource-check", action="store_true", help="Opt-in developer resource precheck"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional finite positive wall-clock run limit in seconds",
    )
    args = parser.parse_args(argv)
    # 越界 domain 与非法 duration 必须在任何文件/进程副作用之前拒绝。
    if not 1 <= args.domain <= 232:
        parser.error("Domain must be in 1..232")
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("Duration must be finite and positive")
    return args


def main(argv=None):
    """执行普通导航生命周期：检查、加锁、准备、启动、监控。

    `--check` 与 `--prepare-only` 允许在运行依赖缺失时执行，真正启动则不允许。
    `--check` 在缺依赖时返回 2，否则 0。正常停止（到达 `--duration` 或收到信号）
    返回 0；异常向上抛给调用方（`start.py`）。一旦有子进程被记录进 cleanup，
    清理块会把失败写进 `exit.json`；此前的失败（加锁、prepare）只向上传播。
    `--prepare-only` 在写完 run 文件后返回 0，已持有的锁仍由 finally 关闭。
    """
    args = parse_arguments(argv)
    # profile 由 common.config.read() 合并 configs/launch/navigation.json 列出的模块。
    profile = read()
    from embodied_navigation.launch.preflight import check

    dependencies = check()
    # --check / --prepare-only 允许缺依赖；普通启动先在这里失败，不建锁也不建目录。
    if not (args.check or args.prepare_only) and not dependencies["ready"]:
        raise RuntimeError("Missing runtime dependencies; run start.py --check for the full list")
    # 显式资源门在 --check 输出之前执行，因此 check 模式不会吞掉资源失败。
    if args.resource_check:
        preflight_resources()
    if args.check:
        print(json.dumps(dependencies, ensure_ascii=False, indent=2))
        return 0 if dependencies["ready"] else 2

    # 记录需要在 cleanup 释放的资源。成功拿到的锁一直保持打开，直到 finally 关闭。
    locks = []
    children = []
    logs = []
    stop = False
    failure = None
    try:
        # 三个固定锁名是跨进程契约：同机同角色的并发启动会在这里失败。
        # LOCK_NB 为非阻塞，拿不到锁直接抛 BlockingIOError，不排队等待。
        for name in [
            "go2_pid_physics_serial.lock",
            "go2_model_pid_simulation.lock",
            "go2_navigation_dispatch.lock",
        ]:
            stream = (Path("/tmp") / name).open("a+")
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            locks.append(stream)

        # run 目录名 = UTC 时间戳 + 过滤后的 label + 4 位随机十六进制。
        stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        # label 只保留字母数字与 _-，全部被过滤掉时回退为 navigation。
        label = "".join(c for c in args.label if c.isalnum() or c in "_-") or "navigation"
        from embodied_navigation.launch.prepare import prepare

        # prepare 返回的派生 profile 是后续运行配置，替换上面从 configs 读到的版本。
        directory = ROOT / "runs" / (stamp + "_" + label + "_" + uuid.uuid4().hex[:4])
        profile = prepare(directory, profile)
        print(
            json.dumps(dict(stage="prepared", directory=str(directory), acceptance_required=False)),
            flush=True,
        )
        if args.prepare_only:
            # 上面的锁已经持有；这里 return 仍会执行 finally。
            return 0

        # TEACHER_SOCKET 是本次运行的 Unix socket 路径，放 /tmp 并带随机后缀避免重名。
        socket = "/tmp/navigation_" + uuid.uuid4().hex[:12] + ".sock"
        env = environment(directory, args.domain, profile, socket)

        def requested_stop(signum, frame):
            """把监控循环的停止标志置位；这里不做其他动作。"""
            nonlocal stop
            stop = True

        # 旧处理器被保存但不再恢复，因为进程随后就退出。
        previous = {
            sig: signal.signal(sig, requested_stop)
            for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]
        }

        def start(role, command, overrides=None):
            """启动一个受管进程组，并登记到 cleanup 列表。

            日志以 `'x'` 模式打开，因为 run 目录是全新的；重名会直接失败而不是追加。
            子进程独占新的 session/进程组，便于按组发信号；同时记录其 `/proc`
            启动时刻，之后据此识别 PID 是否已被回收。`overrides` 覆盖共享 env。
            """
            log = (directory / (role + ".log")).open("x")
            logs.append(log)
            process = subprocess.Popen(
                command,
                cwd=ROOT,
                env={**env, **(overrides or {})},
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            # /proc/<pid>/stat 第 22 字段是时钟节拍表示的启动时刻；
            # 先 rfind(')') 跳过可能含空格的 comm 字段。
            raw = Path("/proc", str(process.pid), "stat").read_text()
            ticks = int(raw[raw.rfind(")") + 2 :].split()[19])
            children.append((role, process, ticks))
            # 每启动一个子进程就重写 processes.json，内容是 role/pid/start_ticks。
            (directory / "processes.json").write_text(
                json.dumps(
                    [dict(role=n, pid=p.pid, start_ticks=t) for n, p, t in children],
                    indent=2,
                )
                + "\n"
            )
            return process

        # 只起一个 ROS launch 进程；由它拥有并监督各组件进程。
        start(
            "launch",
            [
                "ros2",
                "launch",
                str(ROOT / "src/embodied_navigation/launch/launch/demo.launch.py"),
                "run_dir:=" + str(directory),
            ],
        )
        print(
            json.dumps(
                dict(
                    stage="running",
                    directory=str(directory),
                    pid=os.getpid(),
                    acceptance_required=False,
                )
            ),
            flush=True,
        )
        # 墙钟上限从 launch 启动后开始计时，不含加锁与 prepare 的时间。
        began = time.monotonic()
        while not stop:
            # 这里只 poll 直接子进程；组件退出由 cleanup 读取 component_exit.jsonl 汇总。
            for role, process, _ in children:
                if process.poll() is not None:
                    raise RuntimeError(role + " exited with " + str(process.returncode))
            if args.duration is not None and time.monotonic() - began >= args.duration:
                break
            # 0.2 s 轮询间隔是硬编码；更细粒度的就绪/超时由各组件自己负责。
            time.sleep(0.2)
        return 0
    except BaseException as error:
        # 记下失败供 exit.json 使用，然后让原始异常继续向上传播。
        failure = type(error).__name__ + ": " + str(error)
        raise
    finally:
        # 首轮 SIGINT 只发给启动时刻仍匹配的直接子进程；超时兜底才会向进程组发信号。
        for _, process, ticks in reversed(children):
            try:
                raw = Path("/proc", str(process.pid), "stat").read_text()
                actual = int(raw[raw.rfind(")") + 2 :].split()[19])
                # launch 拥有组件的子进程并会转发一次信号；若这里按组发，
                # launch 转发 SIGINT 时子进程会收到第二次，打断它们自己的 finally。
                if actual == ticks:
                    process.send_signal(signal.SIGINT)
            except (OSError, ProcessLookupError):
                pass

        # 优雅停止预算共享：每个子进程只等这 15 s 截止时间的剩余部分，且不少于 0.1 s。
        deadline = time.monotonic() + 15
        for _, process, _ in reversed(children):
            try:
                process.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                # 对进程组再试 SIGTERM，然后最多再等 5 s。
                # 这里的信号/等待错误被忽略；退出状态在下面统一检查。
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
                except (OSError, subprocess.TimeoutExpired):
                    pass

        # 先关日志与锁，再写退出汇总，保证 exit.json 落盘前文件句柄已释放。
        for log in logs:
            log.close()
        for stream in locks:
            stream.close()

        # 只汇总登记进 cleanup 的子进程；prepare 可能在任何子进程记录之前就失败。
        if children:
            from embodied_navigation.launch.exit_status import failed_exits

            process_codes = {role: p.poll() for role, p, _ in children}
            component_file = directory / "component_exit.jsonl"
            # 该 JSONL 由 launch 树在组件退出时可选写入；空行跳过。
            component_exits = (
                [
                    json.loads(line)
                    for line in component_file.read_text().splitlines()
                    if line.strip()
                ]
                if component_file.exists()
                else []
            )
            # 汇总父进程退出码与子组件退出记录；任一非零都算失败。
            unsuccessful = failed_exits(process_codes, component_exits)
            # 只有此前没有失败时才报告关停错误，保留原始异常作为权威原因。
            cleanup_error = (
                "Managed process shutdown failed: " + json.dumps(unsuccessful)
                if unsuccessful and failure is None
                else None
            )
            failure = failure or cleanup_error
            # exit.json 是本 run 的最终退出快照：error + 各进程退出码 + 组件退出记录。
            (directory / "exit.json").write_text(
                json.dumps(
                    dict(
                        error=failure,
                        process_exit_codes=process_codes,
                        component_exits=component_exits,
                    ),
                    indent=2,
                )
                + "\n"
            )
            print(
                json.dumps(dict(stage="stopped", directory=str(directory), error=failure)),
                flush=True,
            )
            # 关停本身失败且此前没有异常时，把它作为本次运行的错误抛出。
            if cleanup_error:
                raise RuntimeError(cleanup_error)
