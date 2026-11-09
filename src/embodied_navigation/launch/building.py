"""Production Building launch using canonical SLAM and project-owned assets."""

# 中文模块说明（首轮补充；上方 docstring 恢复 HEAD 原文，以保持 --help/--doc 文案不变）：
# Building 场景的生产启动入口；默认只做 SLAM 导航。
#
# 与普通主路径（`launch/main.py` → `demo.launch.py`）并列但独立：本文件自己准备
# run 目录、构造环境并用 `Popen` 管理全部子进程，不经过 `ros2 launch`。只接受
# `--mode slam`，在读取资产或创建 run 目录之前就拒绝 truth/capture-only 选择。
# 不实现导航、控制或安全算法。


from embodied_navigation.project import project_root as _project_root
from pathlib import Path
import argparse, datetime, fcntl, json, os, signal, subprocess, sys, time
import xml.etree.ElementTree as ET

ROOT = _project_root()
CASES = (
    "stairs_short",
    "single_floor_turn",
    "combined_floor1_inside",
    "multi_floor2",
    "single_floor_loop",
    "second_stair_short",
    "third_stair_short",
)


def write(path, data):
    """按输入映射顺序写缩进 JSON，并补一个换行。"""
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")


def prepare(case, label, mode="slam", backend_capture=False):
    """按当前 configs 与 assets 物化一个新的 Building run 目录。

    复制路线/传感/世界模板资产，为 Building 话题与项目核心配置 FAST-LIVO2，并
    限制 backend spool 的容量。导航使用 SLAM 位姿；actor 仍消费原生仿真状态。
    返回 `(directory, profile)`。不支持的模式在读取资产与建目录之前就抛
    `ValueError`。
    """
    # 在读取资产或创建 run 目录之前拒绝不支持的来源模式。
    if mode != "slam" or backend_capture:
        raise ValueError(
            "Production Building requires SLAM; truth/capture-only mode is not supported"
        )
    # 路线资产给出世界系起点；run 目录必须新建，同名已存在时直接失败。
    route = json.loads((ROOT / "assets/building/routes" / f"{case}.json").read_text())
    directory = ROOT / "runs" / label
    directory.mkdir(parents=True, exist_ok=False)
    # spawn 取起点前两位 x/y，world z 抬升 0.32 m，yaw=0；0.32 的来源与理由代码与现有文档未说明。
    spawn = [*route["start_world_ground"][:2], route["start_world_ground"][2] + 0.32, 0.0]
    # 导航世界原点取 spawn 的前 3 位。
    origin = spawn[:3]
    # 仿真超时（秒）：两个短用例 160 s，其余 600 s；该映射为硬编码。
    timeout = 160.0 if case in ("stairs_short", "single_floor_turn") else 600.0
    # profile 来自 configs/scenarios/building/student_profile.json，再覆盖本次运行必须的字段。
    profile = json.loads((ROOT / "configs/scenarios/building/student_profile.json").read_text())
    # duration_s 在用例超时上再加 15 s（秒）；加 15 的原因代码与现有文档未说明。
    profile.update(
        duration_s=timeout + 15.0,
        spawn=spawn,
        localization_mode="fast_livo2_async",
        navigation_ground_truth_used=False,
    )
    write(
        directory / "runtime_context.json",
        dict(
            run_dir=str(directory),
            profile=profile,
            actor_kind="student",
            controller_kind="student",
            navigation_ground_truth_used=False,
            references={},
        ),
    )
    write(directory / "PCT_route.json", route)
    # 世界由模板生成：替换 ${PROJECT_ROOT}，世界名固定，go2 位姿改为本次 spawn。
    source = (
        (ROOT / "assets/building/world.sdf.in").read_text().replace("${PROJECT_ROOT}", str(ROOT))
    )
    world = ET.fromstring(source)
    node = world.find("world")
    node.set("name", "building_navigation_slam")
    node.find("model[@name='go2']/pose").text = " ".join(map(str, [*spawn[:3], 0, 0, spawn[3]]))
    ET.indent(world)
    ET.ElementTree(world).write(directory / "world.sdf", encoding="unicode")
    # 传感/地形资产按目标文件名拷贝；scan_params.json 来自 configs/scenarios/building/scan.json。
    for source, target in [
        ("sensor_bridge.yaml", "sensor_bridge.yaml"),
        ("sensor_contract.json", "sensor_contract.json"),
        ("terrain_targets.json", "terrain_target_manifest.json"),
    ]:
        (directory / target).write_bytes((ROOT / "assets/building" / source).read_bytes())
    (directory / "scan_params.json").write_bytes(
        (ROOT / "configs/scenarios/building/scan.json").read_bytes()
    )
    # experiment_setup.json 汇总本次实验的静态约定；下方速度上限、PD 增益、停车契约等数值为硬编码。
    write(
        directory / "experiment_setup.json",
        dict(
            case=case,
            spawn_xyz_yaw=spawn,
            navigation_world_origin=origin,
            goal_world_body=route["world_body_points"][-1],
            timeout_sim_s=timeout,
            navigation_mode="slam",
            no_SLAM_or_global_matcher=False,
            navigation_ground_truth_used=False,
            route_origin="native PCT A* + physical support Z",
            adapter_version="world_twist_terminal_PD_v4",
            stair_speed_cap_mps=0.25,
            planned_step_obstacle_filter=True,
            obstacle_cloud_pose_same_stamp=True,
            SCAN_start_duplicate_coalescing_m=0.1,
            SCAN_twist_reference="world-axis velocity for SCAN; canonical body odom twist is child-axis",
            terminal_position_PD=dict(
                Kp_xy=4.0, Kd_body_velocity_xy=0.4, velocity_limits_xy=[0.16, 0.12]
            ),
            parking_contract="Original CaptureGrid .025m/.035rad dwell .6s; fixed5s 10Hz51poses .05m/.1rad",
        ),
    )
    # FAST-LIVO2 参数取 configs/localization_mapping/fastlivo.yaml 的 /** 节点段，再改写话题与 spool 上限。
    import yaml

    parameters = yaml.safe_load((ROOT / "configs/localization_mapping/fastlivo.yaml").read_text())[
        "/**"
    ]["ros__parameters"]
    # Building 话题：相机 /demo/camera、LiDAR /building/lidar_filtered、IMU /livox/imu；不用 GPS，要求完整 IMU。
    parameters["common"].update(
        img_topic="/demo/camera",
        lid_topic="/building/lidar_filtered",
        imu_topic="/livox/imu",
        use_gps=False,
        require_complete_imu=True,
    )
    # 生产运行关闭传感器同步诊断，避免额外诊断输出。
    parameters["debug"]["sensor_sync_diagnostics"] = False
    # backend spool 上限：队列 4 项、每项 4096 点、最多 16 个就绪目录；采样间隔 0.5–2.0 s。
    # 触发新样本的最小位移 0.8 m、最小转角 0.2 rad。
    parameters["backend"] = dict(
        snapshot_directory=str(directory / "backend_spool"),
        queue_capacity=4,
        max_points=4096,
        max_ready_directories=16,
        min_interval_s=0.5,
        max_interval_s=2.0,
        min_translation_m=0.8,
        min_rotation_rad=0.2,
    )
    # 不发布 base TF；odom/base_link 作为帧名交给下游组合。
    parameters["publish"].update(publish_base_tf=False, odom_frame="odom", child_frame="base_link")
    write(directory / "building_fastlivo.json", {"/**": {"ros__parameters": parameters}})
    # 相机参数来自 configs/localization_mapping/camera.yaml。
    write(
        directory / "building_camera.json",
        yaml.safe_load((ROOT / "configs/localization_mapping/camera.yaml").read_text()),
    )
    # 改写 SCAN 的网格地图帧：全局帧 map，滑动地图帧固定为 building_scan_sliding_map。
    scan = json.loads((directory / "scan_params.json").read_text())
    scan["scan_planner_node"]["ros__parameters"]["grid_map.frame_id"] = "map"
    scan["scan_planner_node"]["ros__parameters"]["grid_map.sliding_map_frame_id"] = (
        "building_scan_sliding_map"
    )
    write(directory / "scan_params.json", scan)
    # 二次更新 setup：显式写死 SLAM-only 标志、spawn 先验与 backend 轮询周期（秒）。
    setup = json.loads((directory / "experiment_setup.json").read_text())
    setup.update(
        navigation_mode="slam",
        backend_capture_only=False,
        navigation_ground_truth_used=False,
        configured_spawn_prior=True,
        no_SLAM_or_global_matcher=False,
        backend_timing=dict(spool_poll_s=0.02, correction_owner_poll_s=0.02, status_period_s=1.0),
    )
    write(directory / "experiment_setup.json", setup)
    return directory, profile


def environment(directory, domain, socket):
    """配置本次运行的 ROS domain、DDS profile 与运行路径。

    从 `common.config.read()` 重新读配置（不复用传入 profile），删掉列出的继承
    传输键后应用项目设置，按 `domain` 设 `ROS_DOMAIN_ID`，并把 `building_<run>`
    作为 Gazebo partition。实际传输设置会写进 `transport_settings.json`。
    不检查其他进程是否已占用同一 domain/partition。
    """
    from embodied_navigation.common.runtime_environment import environment as ros_environment
    from embodied_navigation.common.communication.dds_environment import configure
    from embodied_navigation.common.config import read

    # 先删除继承的传输键，再应用项目传输设置，最后固定 FastRTPS profile 文件。
    transport = read()["dds_transport"]
    env = configure(ros_environment())
    for key in transport["remove_environment_keys"]:
        env.pop(key, None)
    env.update(transport["environment"])
    env["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(
        ROOT / "src/embodied_navigation/common/communication/dds_transport.xml"
    )
    # 记录本次实际使用的传输设置与 profile 文件路径，便于事后核对。
    write(
        directory / "transport_settings.json",
        dict(
            environment=transport["environment"],
            profile_file=str(
                ROOT / "src/embodied_navigation/common/communication/dds_transport.xml"
            ),
        ),
    )
    # 本次运行的路径与开关：socket、日志、actuator 记录都落在 run 目录或固定 /tmp 路径。
    env.update(
        ROS_DOMAIN_ID=str(domain),
        GZ_IP="127.0.0.1",
        GZ_PARTITION="building_" + directory.name,
        BUILDING_SAFETY_SOCKET="/tmp/building_contacts.sock",
        TEACHER_SOCKET=socket,
        DEMO_RUN_DIR=str(directory),
        ROS_LOG_DIR=str(directory / "ros_logs"),
        TEACHER_ACTUATOR_LOG=str(directory / "actuator_physics.jsonl"),
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(ROOT / "install/actuators/lib"),
        PYTHONDONTWRITEBYTECODE="1",
        OMP_NUM_THREADS="1",
        OPENBLAS_NUM_THREADS="1",
        MKL_NUM_THREADS="1",
        NUMEXPR_NUM_THREADS="1",
        # 无头渲染走 surfaceless EGL；Building 入口默认记录原始输入（普通导航 main.py 为 "0"）。
        EGL_PLATFORM="surfaceless",
        NAVIGATION_RECORD_RAW="1",
    )
    # 避免继承的软件渲染开关覆盖 surfaceless EGL 的选择。
    env.pop("LIBGL_ALWAYS_SOFTWARE", None)
    return env


def run(directory, profile, args):
    """启动 Student actor 与 Building 各进程，并监督到关停。

    每个子进程独占自己的日志与 session；`processes.json` 在每次启动后重写
    （只含 role/pid，不记录 start_ticks）。正常监控结束时 `succeeded` 返回 0，
    其他原因返回 2。worker 就绪前被中断返回 130；初始化/关停错误可作为异常抛出。
    """
    # 学生策略 socket 与 BUILDING_SAFETY_SOCKET 都是固定 /tmp 路径。
    socket = "/tmp/building_policy.sock"
    env = environment(directory, args.domain, socket)
    children = []
    stop = False
    # reason 是退出汇总的一部分：wall_limit/interrupted/child_exit:<roles>/succeeded/failed。
    # terminal 记录首次看到终态的时刻，用于给下游留收尾时间。
    reason = "wall_limit"
    terminal = None
    began = time.monotonic()

    def request_stop(*unused):
        nonlocal stop
        stop = True

    # 收到 SIGINT/SIGTERM 只置停止标志，由监控循环退出后再统一关停。
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)

    def start(role, cmd, changes=None):
        """启动一个子进程并登记到 children；日志以 'x' 模式新建。"""
        log = (directory / (role + ".log")).open("x")
        process = subprocess.Popen(
            cmd,
            cwd=ROOT,
            env={**env, **(changes or {})},
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        children.append((role, process, log))
        write(directory / "processes.json", [dict(role=n, pid=p.pid) for n, p, f in children])
        return process

    try:
        # actor 关闭 CUDA（CUDA_VISIBLE_DEVICES 置空），学生策略走 CPU/JIT。
        actor = start(
            "student",
            [
                "/opt/navigation-venv/bin/python3",
                "-B",
                "-m",
                "embodied_navigation.simulation.truth_worker",
                "--run",
                str(directory),
                "--socket",
                socket,
            ],
            {"CUDA_VISIBLE_DEVICES": ""},
        )
        start(
            "sensor_bridge",
            [
                "/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge",
                "--ros-args",
                "-p",
                "config_file:=" + str(directory / "sensor_bridge.yaml"),
            ],
        )
        # slam 模式额外启动资源采样、相机参数、LiDAR 过滤、SLAM、backend 与定位进程。
        if args.mode == "slam":
            # 资源采样进程按模块启动（导入无副作用），每秒记录 /proc 的 CPU/RSS。
            start(
                "resources",
                [
                    "/usr/bin/python3",
                    "-B",
                    "-m",
                    "embodied_navigation.evaluation.process_resources",
                ],
            )
            # parameter_blackboard 加载相机参数文件，供 ROS 节点共享读取。
            start(
                "camera_parameters",
                [
                    "/opt/ros/jazzy/lib/demo_nodes_cpp/parameter_blackboard",
                    "--ros-args",
                    "--params-file",
                    str(directory / "building_camera.json"),
                ],
            )
            # 传感器预处理模块，产出 fastlivo 使用的 /building/lidar_filtered。
            start(
                "lidar_filter",
                [
                    "/usr/bin/python3",
                    "-B",
                    "-m",
                    "embodied_navigation.localization_mapping.building_sensors",
                ],
            )
            # SLAM 加载两份参数文件，并把输出话题改名为 /building/raw_*；下面覆盖流水线环境。
            start(
                "slam",
                [
                    str(ROOT / "install/ros/lib/fast_livo2_ros/fastlivo_mapping"),
                    "--ros-args",
                    "--params-file",
                    str(directory / "building_fastlivo.json"),
                    "--params-file",
                    str(directory / "building_camera.json"),
                    "-r",
                    "/aft_mapped_to_init:=/building/raw_body_odom",
                    "-r",
                    "/cloud_registered_full:=/building/raw_cloud",
                ],
                # 覆盖 environment() 的默认值：staged 流水线、启用图像拷贝，
                # 关闭串行 IMU 优先/状态日志/ingress trace。
                {
                    "FASTLIVO_PIPELINE_MODE": "staged",
                    "FASTLIVO_IMAGE_COPY_OPT": "1",
                    "FASTLIVO_SERIAL_IMU_PRIORITY": "0",
                    "FASTLIVO_DISABLE_STATE_LOG": "1",
                    "FASTLIVO_INGRESS_TRACE": "0",
                },
            )
            # backend 初始世界-odom 位姿取 experiment_setup 的 spawn 前 3 位，姿态置 0；
            # 地图-世界偏移是 3 个硬编码值。
            initial = [
                *json.loads((directory / "experiment_setup.json").read_text())["spawn_xyz_yaw"][:3],
                0.0,
                0.0,
                0.0,
            ]
            start(
                "backend",
                [
                    "/usr/bin/python3",
                    "-B",
                    "-m",
                    "embodied_navigation.localization_mapping.backend.run",
                    "--spool",
                    str(directory / "backend_spool"),
                    "--history",
                    str(directory / "backend_history"),
                    "--result",
                    str(directory / "backend_result.json"),
                    "--external-map",
                    str(ROOT / "assets/building/building2_9.pcd"),
                    "--initial-world-odom",
                    *map(str, initial),
                    "--map-world-offset",
                    "-0.000002",
                    "-0.000001",
                    "-0.1",
                ],
            )
            start(
                "localization",
                [
                    "/usr/bin/python3",
                    "-B",
                    "-m",
                    "embodied_navigation.localization_mapping.building_runtime",
                ],
            )
        # navigation 与 scan 在所有模式下启动；scan 读 vendored planner.yaml 与 run 内 scan_params.json。
        nav = start(
            "navigation",
            ["/usr/bin/python3", "-B", "-m", "embodied_navigation.navigation.building_navigation"],
        )
        # scan 的 remap 目标沿用带 truth 的历史话题名（兼容标签，不代表允许用仿真位姿导航）。
        scan = start(
            "scan",
            [
                str(ROOT / "install/ros/lib/scan_planner/scan_planner_node"),
                "--ros-args",
                "--params-file",
                str(ROOT / "third_party/scan/plan_manage/config/planner.yaml"),
                "--params-file",
                str(directory / "scan_params.json"),
                "-r",
                "body_pose:=/building/scan_body_odom",
                "-r",
                "sensor_pose:=/demo/slam/lidar_odom",
                "-r",
                "cloud:=/cloud_registered_full",
                "-r",
                "initial_path:=/truth/scan_reference",
                "-r",
                "planning/bspline:=/truth/bspline",
                "-r",
                "planning/go2_execution_frozen:=/truth/execution_frozen",
            ],
        )
        # 屏障：先等 worker_ready 文件出现，再启动 Gazebo，保证物理在策略 worker 就绪后开始。
        while not (directory / "worker_ready").exists():
            # 停止优先；actor 提前退出或 60 s 超时都视为失败。
            if stop:
                reason = "interrupted"
                return 130
            if actor.poll() is not None:
                raise RuntimeError("Student exited before ready")
            if time.monotonic() - began > 60:
                raise RuntimeError("Student ready timeout")
            time.sleep(0.1)
        # Gazebo 无头启动，并固定随机种子 42 以保证可复现。
        gz = start(
            "gazebo",
            [
                "gz",
                "sim",
                "-s",
                "-r",
                "--headless-rendering",
                str(directory / "world.sdf"),
                "--seed",
                "42",
            ],
        )
        # 默认墙钟上限 = duration_s × 1.7 + 30 s；--duration 给出时直接用它（秒）。
        # 1.7 与 30 的来源代码与文档未说明。
        deadline = args.duration if args.duration is not None else profile["duration_s"] * 1.7 + 30
        last_progress = 0.0
        # 监控循环：停止标志、任一子进程退出、到达墙钟上限都会结束。
        while time.monotonic() - began < deadline:
            if stop:
                reason = "interrupted"
                break
            dead = [n for n, p, f in children if p.poll() is not None]
            if dead:
                reason = "child_exit:" + ",".join(dead)
                break
            # 状态文件可能尚未写出或正在写入，读失败时跳过本轮。
            try:
                status = json.loads((directory / "truth_navigation_status.json").read_text())
                # 每 10 s 打一行进度 JSON（state/sim_s/误差/候选数/原因）。
                if time.monotonic() - last_progress > 10:
                    print(
                        json.dumps(
                            {
                                k: status.get(k)
                                for k in (
                                    "state",
                                    "sim_s",
                                    "goal_xy_error_m",
                                    "goal_height_error_m",
                                    "candidate_count",
                                    "reason",
                                )
                            },
                            ensure_ascii=False,
                        ),
                        flush=True,
                    )
                    last_progress = time.monotonic()
                # 进入终态后再等 3 s 收尾，给下游写完结果文件；首次终态时刻记在 terminal。
                if status["state"] in ("failed", "succeeded"):
                    if terminal is None:
                        terminal = time.monotonic()
                        reason = status["state"]
                    if time.monotonic() - terminal > 3:
                        break
            except (OSError, ValueError):
                pass
            time.sleep(0.2)
    finally:

        def wait_shutdown(process):
            """先等 12 s 优雅退出，再对进程组 SIGTERM 等 5 s，最后 SIGKILL。"""
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()

        # 先请 actor 完成一次完整的原推理/安全交互（response[13]=1）：原生物理确认后
        # 请求停止；双方都不会因并发信号看到人为 EOF 或超时。
        physics_alive = any(n == "gazebo" and p.poll() is None for n, p, f in children)
        if physics_alive and actor.poll() is None:
            actor.send_signal(signal.SIGUSR1)
            try:
                actor.wait(timeout=12)
            except subprocess.TimeoutExpired:
                pass
        # 物理已退出时只给 actor 2 s 收尾，不再发 SIGUSR1。
        if (
            not physics_alive
            and any(n == "gazebo" for n, p, f in children)
            and actor.poll() is None
        ):
            try:
                actor.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass
        # 先给 Gazebo SIGINT 并等它退出，再停其余进程。
        for n, p, f in children:
            if n == "gazebo" and p.poll() is None:
                p.send_signal(signal.SIGINT)
                wait_shutdown(p)
        # 其余进程按启动逆序发 SIGINT，再按同样顺序逐个等待退出并关闭日志。
        for n, p, f in reversed(children):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        for n, p, f in reversed(children):
            wait_shutdown(p)
            f.close()
        # experiment_exit.json 记录墙钟耗时、结束原因与各子进程退出码。
        write(
            directory / "experiment_exit.json",
            dict(
                wall_s=time.monotonic() - began,
                reason=reason,
                exit_codes={n: p.returncode for n, p, f in children},
                navigation_ground_truth_used=False,
            ),
        )
    # 只有 reason == succeeded 返回 0；interrupted、超时、子进程退出等均返回 2。
    print(json.dumps(dict(directory=str(directory), reason=reason)), flush=True)
    return 0 if reason == "succeeded" else 2


def main(argv=None):
    """Building 入口 CLI：依赖检查、参数校验、加锁、prepare，然后可选运行。

    `--check` 只做文件级依赖检查并输出 JSON（就绪 0，缺项 2），不校验
    domain/duration/validation 参数。普通路径在 `--check` 之后才校验参数，然后取
    `/tmp/building_navigation.lock` 非阻塞锁并准备 run 目录；`--prepare-only` 在
    打印 prepared 后返回 0，否则进入 :func:`run`。
    """
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mode", choices=("slam",), default="slam")
    ap.add_argument("--case", choices=CASES, default="multi_floor2")
    ap.add_argument("--label")
    ap.add_argument("--domain", type=int, default=226)
    ap.add_argument("--prepare-only", action="store_true")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--duration", type=float)
    ap.add_argument(
        "--validation-lidar-gap",
        type=float,
        nargs=2,
        metavar=("START_S", "DURATION_S"),
        help="Explicit test only: drop LiDAR by its original source timestamp; disabled by default",
    )
    ap.add_argument(
        "--validation-lidar-gap-after-tracking",
        type=float,
        nargs=2,
        metavar=("DELAY_S", "DURATION_S"),
        help="Test only: arm source gap after confirmed nonzero tracking; disabled by default",
    )
    args = ap.parse_args(argv)
    # 依赖清单是固定工程相对路径；--check 只判断 is_file，不验证内容。
    needed = [
        ROOT / "install/ros/local_setup.bash",
        ROOT / "install/ros/lib/scan_planner/scan_planner_node",
        ROOT / "install/actuators/lib/libteacher_actuator.so",
        ROOT / "models/student_model_4999_deterministic_jit.pt",
        ROOT / "assets/building/Building.dae",
        ROOT / "assets/building/building2_9.pcd",
        ROOT / "assets/meshes/L1_4D_Lidar.dae",
        ROOT / "install/ros/lib/libfast_livo2_core.so",
        ROOT / "install/backend/lib/liblivo_graph.so",
    ]
    # --check 在参数校验与加锁之前返回；backend_integrated 与 ready 同值，表示按完整 backend 处理。
    if args.check:
        missing = [str(p) for p in needed if not p.is_file()]
        print(
            json.dumps(
                dict(
                    ready=not missing,
                    missing=missing,
                    root=str(ROOT),
                    backend_integrated=not missing,
                    validation_scope="Dependency check only; see per-run navigation results",
                    navigation_mode="slam",
                    navigation_ground_truth_used=False,
                ),
                ensure_ascii=False,
                indent=2,
            )
        )
        return 2 if missing else 0
    # 这些校验在 --check 之后执行，因此 --check 会接受非法 domain/duration。
    if not 1 <= args.domain <= 232:
        ap.error("domain must be 1..232")
    if args.duration is not None and args.duration <= 0:
        ap.error("duration must be positive")
    # 绝对丢包实验只允许 slam：起始 ≥8 s、持续 (0,3] s。
    if args.validation_lidar_gap:
        start_s, duration_s = args.validation_lidar_gap
        if args.mode != "slam" or start_s < 8 or not 0 < duration_s <= 3:
            ap.error("validation gap requires slam, start>=8s and duration in (0,3]s")
    # 相对丢包实验：slam、delay ≥1 s、持续 (0,3] s，且不能与绝对实验同时给出。
    if args.validation_lidar_gap_after_tracking:
        delay_s, duration_s = args.validation_lidar_gap_after_tracking
        if (
            args.mode != "slam"
            or args.validation_lidar_gap
            or delay_s < 1
            or not 0 < duration_s <= 3
        ):
            ap.error(
                "relative validation gap requires slam, delay>=1s, duration in (0,3]s and no absolute gap"
            )
    # label 缺省为 UTC 时间戳 + 用例名；只允许字母数字与 _-，否则报错。
    label = (
        args.label
        or datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + args.case
    )
    if not label or any(
        c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in label
    ):
        ap.error("Invalid run label")
    # 单实例锁：同一时刻只允许一个 Building 运行；锁文件不显式关闭，进程退出时由内核释放。
    lock = Path("/tmp/building_navigation.lock").open("a+")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    directory, profile = prepare(args.case, label, args.mode)
    # 校验实验参数只写进 experiment_setup.json；默认不出现这两个键。
    if args.validation_lidar_gap:
        setup = json.loads((directory / "experiment_setup.json").read_text())
        setup["validation_lidar_gap_s"] = args.validation_lidar_gap
        write(directory / "experiment_setup.json", setup)
    if args.validation_lidar_gap_after_tracking:
        setup = json.loads((directory / "experiment_setup.json").read_text())
        setup["validation_lidar_gap_after_tracking_s"] = args.validation_lidar_gap_after_tracking
        write(directory / "experiment_setup.json", setup)
    # 打印 prepared 行（含路线长度），供上层/日志识别本次准备结果。
    print(
        json.dumps(
            dict(
                stage="prepared",
                directory=str(directory),
                mode=args.mode,
                planned_length_m=json.loads((directory / "PCT_route.json").read_text())["length_m"],
            ),
            ensure_ascii=False,
        ),
        flush=True,
    )
    # --prepare-only 在准备后返回 0；否则进入 run 并返回其结果。
    return 0 if args.prepare_only else run(directory, profile, args)


if __name__ == "__main__":
    raise SystemExit(main())
