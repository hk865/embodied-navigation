"""Bounded real-SLAM RoboGauge Building benchmark, separate from normal navigation.

Preparation and preflight are CPU-only. Running starts ROS/Gazebo and is explicit.
The complete process and shutdown logic comes from the archived deployment, with
project/data paths and module names made portable. No training project is used.
"""

from __future__ import annotations
import argparse
import datetime
import fcntl
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid
from embodied_navigation.project import project_root, data_root
from embodied_navigation.robot.unitree.robogauge.runtime import default_model_path, verify_model
from embodied_navigation.simulation.gazebo.robogauge.world import (
    CASES,
    CONFIG,
    actuator_path,
    prepare,
    write,
)

ROOT = project_root()


def process_commands(
    directory, *, model=None, actor_python=None, ros_python="/usr/bin/python3", socket_path
):
    directory = Path(directory)
    actor_python = actor_python or os.environ.get("NAVIGATION_ACTOR_PYTHON", sys.executable)
    model = default_model_path() if model is None else Path(model)
    initial = [
        *json.loads((directory / "experiment_setup.json").read_text())["spawn_xyz_yaw"][:3],
        0.0,
        0.0,
        0.0,
    ]
    py = lambda module, *args: [ros_python, "-B", "-m", module, *map(str, args)]
    return [
        (
            "actor",
            [
                actor_python,
                "-B",
                "-m",
                "embodied_navigation.simulation.gazebo.robogauge.worker",
                "--run",
                str(directory),
                "--socket",
                socket_path,
                "--model",
                str(model),
            ],
        ),
        (
            "sensor_bridge",
            [
                "/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge",
                "--ros-args",
                "-p",
                "config_file:=" + str(directory / "sensor_bridge.yaml"),
            ],
        ),
        (
            "camera_parameters",
            [
                "/opt/ros/jazzy/lib/demo_nodes_cpp/parameter_blackboard",
                "--ros-args",
                "--params-file",
                str(directory / "building_camera.json"),
            ],
        ),
        ("lidar_filter", py("embodied_navigation.localization_mapping.building_sensors")),
        (
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
        ),
        (
            "backend",
            py(
                "embodied_navigation.localization_mapping.backend.run",
                "--spool",
                directory / "backend_spool",
                "--history",
                directory / "backend_history",
                "--result",
                directory / "backend_result.json",
                "--external-map",
                data_root() / "runtime_assets/assets/building/building2_9.pcd",
                "--initial-world-odom",
                *initial,
                "--map-world-offset",
                "-0.000002",
                "-0.000001",
                "-0.1",
            ),
        ),
        ("localization", py("embodied_navigation.localization_mapping.building_runtime")),
        ("navigation", py("embodied_navigation.navigation.building_navigation")),
        (
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
        ),
        (
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
        ),
    ]


def preflight(model=None, actor_python=None):
    """File/dependency inspection only; never claims an end-to-end pass."""
    import xml.etree.ElementTree as ET

    assets = data_root() / "runtime_assets/assets"
    needed = [
        ROOT / "install/ros/local_setup.bash",
        ROOT / "install/ros/lib/fast_livo2_ros/fastlivo_mapping",
        ROOT / "install/ros/lib/scan_planner/scan_planner_node",
        ROOT / "install/ros/lib/libfast_livo2_core.so",
        ROOT / "install/backend/lib/liblivo_graph.so",
        actuator_path(),
        assets / "building/building2_9.pcd",
        assets / "building/building_triangles.npz",
        ROOT / "third_party/scan/plan_manage/config/planner.yaml",
        Path("/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge"),
        Path("/opt/ros/jazzy/lib/demo_nodes_cpp/parameter_blackboard"),
    ]
    world = ET.fromstring(
        (CONFIG / "world.sdf.in").read_text().replace("${ASSET_ROOT}", str(assets))
    )
    needed.extend(Path(n.text.removeprefix("file://")) for n in world.iter("uri"))
    actor_python = actor_python or os.environ.get("NAVIGATION_ACTOR_PYTHON", sys.executable)
    missing = sorted({str(p) for p in needed if not p.is_file()})
    if not shutil.which(actor_python):
        missing.append(actor_python)
    if not shutil.which("gz"):
        missing.append("gz (Gazebo Harmonic executable)")
    model_path = default_model_path() if model is None else Path(model)
    try:
        verify_model(model_path)
        model_error = None
    except (OSError, ValueError) as error:
        model_error = str(error)
    return dict(
        ready=not missing and model_error is None,
        missing=missing,
        model_error=model_error,
        actor_python=actor_python,
        model_path=str(model_path),
        navigation_ground_truth_used=False,
        scope="File/hash preflight only; Python imports and integrated navigation are not certified",
    )


def environment(directory, domain, socket_path, contact_socket):
    from embodied_navigation.common.runtime_environment import environment as ros_environment
    from embodied_navigation.common.communication.dds_environment import configure
    from embodied_navigation.common.config import read

    transport = read()["dds_transport"]
    env = configure(ros_environment())
    for key in transport["remove_environment_keys"]:
        env.pop(key, None)
    env.update(transport["environment"])
    env.update(
        FASTRTPS_DEFAULT_PROFILES_FILE=str(
            ROOT / "src/embodied_navigation/common/communication/dds_transport.xml"
        ),
        ROS_DOMAIN_ID=str(domain),
        GZ_IP="127.0.0.1",
        GZ_PARTITION="robogauge_" + directory.name,
        BUILDING_SAFETY_SOCKET=contact_socket,
        TEACHER_SOCKET=socket_path,
        DEMO_RUN_DIR=str(directory),
        TEACHER_ACTUATOR_LOG=str(directory / "actuator_physics.jsonl"),
        ROS_LOG_DIR=str(directory / "ros_logs"),
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(actuator_path().parent),
        CUDA_VISIBLE_DEVICES="",
        PYTHONDONTWRITEBYTECODE="1",
        OMP_NUM_THREADS="1",
        OPENBLAS_NUM_THREADS="1",
        MKL_NUM_THREADS="1",
        NUMEXPR_NUM_THREADS="1",
        EGL_PLATFORM="surfaceless",
        NAVIGATION_RECORD_RAW="1",
        FASTLIVO_PIPELINE_MODE="staged",
        FASTLIVO_IMAGE_COPY_OPT="1",
        FASTLIVO_SERIAL_IMU_PRIORITY="0",
        FASTLIVO_DISABLE_STATE_LOG="1",
        FASTLIVO_INGRESS_TRACE="0",
    )
    # Actor stays CPU; the renderer may use the host's graphics device.
    env.pop("LIBGL_ALWAYS_SOFTWARE", None)
    write(
        directory / "transport_settings.json",
        dict(
            environment=transport["environment"], profile_file=env["FASTRTPS_DEFAULT_PROFILES_FILE"]
        ),
    )
    return env


def run(directory, profile, args):
    token = uuid.uuid4().hex[:12]
    socket_path = "/tmp/robogauge_" + token + ".sock"
    contacts = "/tmp/robogauge_contacts_" + token + ".sock"
    env = environment(directory, args.domain, socket_path, contacts)
    commands = process_commands(
        directory, model=args.model, actor_python=args.actor_python, socket_path=socket_path
    )
    children = []
    stop = False
    reason = "wall_limit"
    terminal = None
    began = time.monotonic()
    actor = None

    def request_stop(*unused):
        nonlocal stop
        stop = True

    previous = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}

    def start(role, command):
        log = (directory / (role + ".log")).open("x")
        try:
            process = subprocess.Popen(
                command,
                cwd=ROOT,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
        except BaseException:
            log.close()
            raise
        children.append((role, process, log))
        write(directory / "processes.json", [dict(role=n, pid=p.pid) for n, p, _ in children])
        return process

    def wait_shutdown(process):
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()

    try:
        for role, command in commands:
            if role == "gazebo":
                while not (directory / "worker_ready").exists():
                    if stop:
                        reason = "interrupted"
                        return 130
                    dead = [n for n, p, _ in children if p.poll() is not None]
                    if dead:
                        raise RuntimeError("Startup child exit: " + ",".join(dead))
                    if time.monotonic() - began > 60:
                        raise RuntimeError("Actor ready timeout")
                    time.sleep(0.1)
            process = start(role, command)
            if role == "actor":
                actor = process
        deadline = args.duration if args.duration is not None else profile["duration_s"] * 1.7 + 30
        while time.monotonic() - began < deadline:
            if stop:
                reason = "interrupted"
                break
            # Read terminal evidence before child exits: successful shutdown is not a failure.
            try:
                status = json.loads((directory / "truth_navigation_status.json").read_text())
                if status["state"] in ("failed", "succeeded"):
                    if terminal is None:
                        terminal = time.monotonic()
                        reason = status["state"]
                    if time.monotonic() - terminal > 3:
                        break
            except (OSError, ValueError, KeyError):
                pass
            dead = [n for n, p, _ in children if p.poll() is not None]
            if dead:
                if terminal is None:
                    reason = "child_exit:" + ",".join(dead)
                break
            time.sleep(0.2)
    except BaseException as error:
        reason = type(error).__name__ + ": " + str(error)
        raise
    finally:
        physics_alive = any(n == "gazebo" and p.poll() is None for n, p, _ in children)
        if physics_alive and actor is not None and actor.poll() is None:
            actor.send_signal(signal.SIGUSR1)
            try:
                actor.wait(timeout=12)
            except subprocess.TimeoutExpired:
                pass
        for n, p, _ in children:
            if n == "gazebo" and p.poll() is None:
                p.send_signal(signal.SIGINT)
                wait_shutdown(p)
        for _, p, _ in reversed(children):
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        for _, p, log in reversed(children):
            wait_shutdown(p)
            log.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        # Only our newly allocated IPC endpoints are removed, never a shared socket.
        for path in (socket_path, contacts):
            try:
                Path(path).unlink()
            except FileNotFoundError:
                pass
        write(
            directory / "experiment_exit.json",
            dict(
                wall_s=time.monotonic() - began,
                reason=reason,
                exit_codes={n: p.returncode for n, p, _ in children},
                navigation_ground_truth_used=False,
            ),
        )
    return 0 if reason == "succeeded" else 2


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--case", choices=CASES, default="single_floor_turn")
    ap.add_argument("--label")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--prepare-only", action="store_true")
    ap.add_argument(
        "--run",
        action="store_true",
        help="Explicitly start ROS and Gazebo; omitted means preparation only",
    )
    ap.add_argument("--domain", type=int, default=226)
    ap.add_argument("--duration", type=float)
    ap.add_argument("--model", type=Path)
    ap.add_argument("--actor-python")
    args = ap.parse_args(argv)
    if not 1 <= args.domain <= 232:
        ap.error("domain must be 1..232")
    if args.duration is not None and not (0 < args.duration < float("inf")):
        ap.error("duration must be finite and positive")
    if args.prepare_only and args.run:
        ap.error("--prepare-only and --run are mutually exclusive")
    if args.check:
        result = preflight(args.model, args.actor_python)
        print(json.dumps(result, indent=2))
        return 0 if result["ready"] else 2
    if args.run:
        result = preflight(args.model, args.actor_python)
        if not result["ready"]:
            print(json.dumps(result, indent=2))
            return 2
    label = (
        args.label
        or datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + args.case
    )
    # Same advisory lock as the existing Building benchmark, plus project simulation locks.
    locks = []
    try:
        for name in (
            "building_navigation.lock",
            "go2_pid_physics_serial.lock",
            "go2_model_pid_simulation.lock",
            "go2_navigation_dispatch.lock",
        ):
            stream = (Path("/tmp") / name).open("a+")
            locks.append(stream)
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        directory, profile = prepare(args.case, label, mode="slam")
        write(
            directory / "launch_plan.json",
            dict(
                commands=[
                    dict(role=n, command=c)
                    for n, c in process_commands(
                        directory,
                        model=args.model,
                        actor_python=args.actor_python,
                        socket_path="/tmp/robogauge_<allocated>.sock",
                    )
                ],
                scope="Prepared only; not execution evidence",
            ),
        )
        print(
            json.dumps(
                dict(
                    stage="prepared",
                    directory=str(directory),
                    mode="slam",
                    navigation_ground_truth_used=False,
                )
            ),
            flush=True,
        )
        return run(directory, profile, args) if args.run else 0
    finally:
        for stream in locks:
            stream.close()


if __name__ == "__main__":
    raise SystemExit(main())
