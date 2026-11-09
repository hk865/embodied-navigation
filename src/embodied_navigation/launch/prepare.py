"""从工程资产与已合并配置生成普通导航本次运行所需的仿真/传感/规划配置。

生成物写入一个新的 run 目录，作为后续 `ros2 launch` 的输入。本模块不做实验
准入、不归档源码、不写验收凭据或结论；它只把 `configs`、`assets` 与 profile 里
的选择落到具体文件，并把少数启动器自有的键固定下来（见 :func:`_pin`）。
"""

import os
from pathlib import Path
import copy, json, math, subprocess, sys, xml.etree.ElementTree as ET
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from embodied_navigation.common.config import ROOT, read, path, loads
from embodied_navigation.common.runtime_context import write as write_context


def write(directory, name, value):
    """把 `value` 以缩进 JSON 写成 `<directory>/<name>`，末尾补一个换行。

    `allow_nan=False`：NaN/Infinity 不是合法 JSON，会直接抛 `ValueError`，而不是
    写出下游难以解析的文件。
    """
    (directory / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def _pin(mapping, key, value, reason, label):
    """强制写入一个启动器自有的值，并在配置值与目标不一致时报告。

    `prepare` 生成的是日常导航 run，因此有少数 profile 键归启动器所有，而不是
    由 `configs` 决定。这些键过去被静默覆盖；现在若配置里的值与目标值不同，会
    向 stderr 打一行以 `prepare: pinning` 开头的说明（附配置值、目标值与原因），
    让读者知道改配置文件里的这一项不会生效。键缺失时按 `<unset>` 报告。
    本函数只写内存中的 `mapping`，不写文件。
    """
    previous = mapping.get(key, "<unset>")
    if previous != value:
        print(
            "prepare: pinning "
            + label
            + " = "
            + repr(value)
            + " (config had "
            + repr(previous)
            + "): "
            + reason,
            file=sys.stderr,
        )
    mapping[key] = value


def prepare(directory, profile):
    """按 profile 与工程资产生成一个全新的 run 目录，返回固定后的 profile。

    `directory` 必须尚不存在（`mkdir(exist_ok=False)`），因此不会覆盖旧 run；
    profile 先深拷贝再对启动器自有的键调用 :func:`_pin`，调用方的字典不被修改。
    世界/传感/SCAN/SLAM 文件由现有的世界与传感准备代码生成，最后把 profile 写入
    `runtime_context.json`。任一步失败都会直接抛异常，不会留下"半成品即成功"的
    标记。返回值是写入 context 的同一份 profile。
    """
    # 先规范化路径再建目录；同名 run 已存在时直接失败，避免覆盖旧产物。
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    profile = copy.deepcopy(profile)
    # 以下键归启动器所有：目标值不受 configs 影响；不一致时 _pin 向 stderr 报告，但仍写入目标值。
    _pin(
        profile,
        "daily_navigation",
        True,
        "the native launch only runs ordinary daily navigation",
        "daily_navigation",
    )
    _pin(
        profile["cascade"],
        "daily_navigation",
        True,
        "the cascade layer is pinned to daily navigation",
        "cascade.daily_navigation",
    )
    _pin(
        profile["r85_auto_reference_contract"],
        "reference_file",
        "global_reference.json",
        "prepare fixes this file name in the profile/context; the global_reference.json entity itself is written by mission_runtime, not by prepare",
        "r85_auto_reference_contract.reference_file",
    )
    _pin(
        profile,
        "experiment",
        "navigation",
        "the native launch is not an experiment run",
        "experiment",
    )
    for key, value, reason in (
        (
            "raw_source_capture",
            False,
            "the native launch keeps the live Gazebo->ROS sensor chain; this pins only the optional raw-source archive off",
        ),
        (
            "registered_cloud_archive",
            "disabled",
            "the map archive role still runs; this pins only the extra registered-cloud history archive off",
        ),
    ):
        _pin(
            profile["engineering_recording"],
            key,
            value,
            reason,
            "engineering_recording." + key,
        )
    command = [
        sys.executable,
        "-B",
        str(ROOT / "src/embodied_navigation/simulation/world/prepare.py"),
        "--output",
        str(directory),
        "--terrain",
        profile["expected_asset"]["terrain"],
        "--sensors",
        "--camera-rate",
        "10",
        "--render-engine",
        "ogre2",
        "--real-time-factor",
        "1.0",
        "--overview-fov",
        str(math.radians(80)),
        "--overview-pose",
        *map(str, profile["overview_pose"]),
    ]
    # 调用仿真侧的世界生成脚本：相机 10 Hz、ogre2 渲染、实时因子 1.0；视场角按弧度传（80°）。
    # 子进程需要能 import 本包，因此把 <ROOT>/src 放在 PYTHONPATH 首位；check=True 让生成失败立即中止。
    subprocess.run(
        command,
        check=True,
        stdout=subprocess.DEVNULL,
        env={
            **os.environ,
            "PYTHONPATH": str(ROOT / "src") + os.pathsep + os.environ.get("PYTHONPATH", ""),
        },
    )
    tree = ET.parse(directory / "world.sdf")
    robot = tree.getroot().find("world/model[@name='go2']")
    # 用 profile 的 spawn 覆盖 go2 位姿：前 3 位是世界坐标 xyz（米），第 4 位是 yaw（弧度），roll/pitch 置 0。
    robot.find("pose").text = " ".join(map(str, [*profile["spawn"][:3], 0, 0, profile["spawn"][3]]))
    # 世界模板必须恰好有一个 TeacherActuator 插件；数量不符说明资产已变，直接失败。
    plugins = [p for p in robot.findall("plugin") if "TeacherActuator" in p.get("name", "")]
    if len(plugins) != 1:
        raise ValueError("Expected one joint actuator plugin")
    # 插件文件名固定为配置的 actuator 路径，使生成的 SDF 指向本次构建产物。
    plugins[0].set("filename", str(path("actuator")))
    ET.indent(tree)
    tree.write(directory / "world.sdf", encoding="unicode")
    from embodied_navigation.perception.sensors.sampling import override_sensors

    # 传感器契约由 profile 覆盖；随后把 spawn 回写 asset_manifest，保持两份产物一致。
    override_sensors(directory, profile)
    asset = json.loads((directory / "asset_manifest.json").read_text())
    asset["spawn"] = profile["spawn"]
    write(directory, "asset_manifest.json", asset)
    # SLAM 参数取自 configs/localization_mapping/fastlivo.yaml 的 /** 节点段；scan_line 用 LiDAR 垂直采样数。
    sensors = json.loads((directory / "sensor_contract.json").read_text())
    slam = yaml.safe_load((ROOT / "configs/localization_mapping/fastlivo.yaml").read_text())
    parameters = slam["/**"]["ros__parameters"]
    parameters["preprocess"]["scan_line"] = sensors["lidar"]["samples"]["vertical"]
    # 话题固定为普通导航的 /demo/teacher/slam/* 与 /demo/slam/lidar_filtered，与 launch 组件约定一致。
    parameters["common"].update(
        img_topic="/demo/teacher/slam/image",
        lid_topic="/demo/slam/lidar_filtered",
        imu_topic="/demo/teacher/slam/imu",
    )
    # 由传感器契约的 body_xyz（米）与 body_rpy（弧度）计算 IMU-LiDAR-相机外参；ri 是 IMU 到机体的旋转。
    ri, rl, rc = [
        Rotation.from_euler("xyz", sensors[k]["body_rpy"]).as_matrix()
        for k in ("imu", "lidar", "camera")
    ]
    pi, pl, pc = [np.asarray(sensors[k]["body_xyz"]) for k in ("imu", "lidar", "camera")]
    # 相机光学坐标系（x 右、y 下、z 前）到机体系的固定旋转；Rcl/Pcl 因此要经过 optical 转换。
    optical = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
    parameters["extrin_calib"].update(
        extrinsic_T=(ri.T @ (pl - pi)).tolist(),
        extrinsic_R=(ri.T @ rl).reshape(-1).tolist(),
        Rcl=(optical @ rc.T @ rl).reshape(-1).tolist(),
        Pcl=(optical @ rc.T @ (pl - pc)).tolist(),
    )
    # imu_int_frame 统计 IMU 采样数，按速率乘 3 取整（100 Hz 下为 300，约 3 s 静止初始化窗口）。
    # init_min_span=2.9 与 fastlivo.yaml 的默认值相同；初始化要求连续静止区间。
    parameters["imu"].update(
        imu_int_frame=int(round(sensors["imu"]["rate_hz"] * 3)),
        init_min_span=2.9,
        stationary_initialization_en=True,
    )
    # 允许 profile 覆盖 IMU 在线估计开关；键缺失时保持上游默认值。
    parameters["imu"].update(profile.get("slam_imu_online_estimation", {}))
    # 视觉量测协方差仅在 profile 显式给出时覆盖，None 表示沿用 fastlivo.yaml 的值。
    if profile.get("slam_visual_measurement_covariance") is not None:
        parameters["vio"]["img_point_cov"] = profile["slam_visual_measurement_covariance"]
    write(directory, "navigation_fastlivo.yaml", slam)
    write(
        directory,
        "navigation_camera.yaml",
        yaml.safe_load((ROOT / "configs/localization_mapping/camera.yaml").read_text()),
    )
    write(directory, "navigation_profile.json", profile)
    write(directory, "navigation_scenario.json", profile["original_scenario"])
    write(
        directory,
        "terrain_target_manifest.json",
        json.loads((ROOT / profile["terrain_target_manifest"]).read_text()),
    )
    # 已知场景参考由刚生成的 world.sdf 构建，而不是直接拷贝配置，保证与本次世界一致。
    from embodied_navigation.localization_mapping.known_scene_reference_builder import build

    write(directory, "known_scene_reference.json", build(directory / "world.sdf"))
    from embodied_navigation.planning.scan_runtime_parameters import parameters as scan_parameters

    write(
        directory,
        "v34_scan_overrides.yaml",
        {"scan_planner_node": {"ros__parameters": scan_parameters(profile)}},
    )
    # SCAN 覆盖参数由规划模块按 profile 计算，写成独立 yaml 供节点加载。
    route = loads((ROOT / "assets/geometry/route.json").read_text())
    write(
        directory,
        "global_route.json",
        {
            k: route[k]
            for k in [
                "body_points",
                "ground_points",
                "T_navigation_world",
                "physical_sources",
                "endpoint_support_connectors",
            ]
        },
    )
    # 最后写 runtime_context.json；它必须含 profile 与 run_dir，供子进程重新加载。
    write_context(directory, profile)
    return profile
