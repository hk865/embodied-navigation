"""只在 fresh、尚未启动的生成场景上应用已审核的采样覆盖。

职责：把本次 run 的 profile["sensor_sampling"] 中四个已审核值写进 world.sdf：
LiDAR 垂直/水平采样数、LiDAR 与相机 update_rate；同步本次 run 的
sensor_contract.json，并写 sensor_sampling_contract.json 作为覆盖证据。

边界：只允许改 world 中这四个叶子。物理步长、native PD 频率、Actor 频率、相机
K/D 几何、LiDAR FOV/噪声/外参都不动；不改已启动 run 的运行期参数，也不测量实际
发布频率（nominal 30 Hz 不等于实测 30 Hz）。本模块是 launch/prepare.py 的生成
步骤，不是运行期组件。
"""

from pathlib import Path
import copy
import hashlib
import json
import xml.etree.ElementTree as ET


def sha(p):
    """返回文件原始字节的 SHA-256 十六进制摘要（用于覆盖前后哈希证据）。"""
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def shape(e):
    """返回 XML 元素的结构签名：tag、按名排序的属性、去空白后的文本、递归子元素。

    用于证明除四个允许叶子外，整棵 world 未被改动。
    """
    return (
        e.tag,
        tuple(sorted(e.attrib.items())),
        (e.text or "").strip(),
        tuple(shape(c) for c in e),
    )


def override_sensors(run, profile):
    """把已审核的 64 线采样/频率覆盖应用到 fresh、未启动的 run。

    参数：
        run：本次 run 目录（Path 或 str）。
        profile：业务 profile；读取 profile["sensor_sampling"] 的 vertical_lines、
            horizontal_samples、lidar_hz、camera_hz 四个键。

    返回：receipt 字典（schema teacher_sensor_sampling_trial/v1），同时写入
    run/sensor_sampling_contract.json。

    副作用：先备份 world.sdf 为 world_before_sensor_override.sdf，再重写 world.sdf
    与 sensor_contract.json（两者都记录前后 sha256）。

    失败：
        run 已存在 runtime_manifest.json 或 sensor_sampling_contract.json →
        RuntimeError（只允许 fresh、未启动场景）；
        profile 缺 sensor_sampling 或四个键 → KeyError；
        四个值不是已审核组合（vertical_lines=64、horizontal_samples=480、
        lidar_hz/camera_hz ∈ {10,20,30}）→ ValueError；
        world 中找不到 go2 模型、gpu_lidar 或 demo_rgb 传感器 → RuntimeError；
        覆盖后整棵 world 的结构签名变化 → RuntimeError。
    """
    run = Path(run)
    # 这两个文件只要存在就说明 run 已启动或已采样过，因此覆盖必须是 fresh 场景。
    if (run / "runtime_manifest.json").exists() or (run / "sensor_sampling_contract.json").exists():
        raise RuntimeError("Sampling changes require a fresh unlaunched scene")
    # 四个允许覆盖的叶子值；当前 configs/control/settings.json 的 sensor_sampling
    # 为 64/480/30/30，由 launch/prepare.py 写进本次 profile。
    settings = profile["sensor_sampling"]
    v, h, lr, cr = [
        settings[k] for k in ("vertical_lines", "horizontal_samples", "lidar_hz", "camera_hz")
    ]
    if v != 64 or h != 480 or lr not in (10, 20, 30) or cr not in (10, 20, 30):
        raise ValueError("Only reviewed64-line density/rate variants are allowed")
    tree = ET.parse(run / "world.sdf")
    world = tree.getroot().find("world")
    robot = world.find("model[@name='go2']")
    # 只定位 go2 模型下的 gpu_lidar 与名为 demo_rgb 的相机；找不到即视为实际机体
    # 传感器缺失。
    lidar = robot.find(".//sensor[@type='gpu_lidar']")
    camera = robot.find(".//sensor[@name='demo_rgb']")
    if lidar is None or camera is None:
        raise RuntimeError("Actual body sensors missing")
    # 覆盖前先记录整棵 world 的结构签名。
    before = shape(world)
    # 四个允许改写的叶子：vertical/horizontal samples 与两个 update_rate。
    paths = [
        (lidar.find("lidar/scan/vertical/samples"), str(v)),
        (lidar.find("lidar/scan/horizontal/samples"), str(h)),
        (lidar.find("update_rate"), str(lr)),
        (camera.find("update_rate"), str(cr)),
    ]
    # 保存四个叶子的原文本，供后面的结构校验恢复使用。
    originals = [(e, e.text) for e, _ in paths]
    # 覆盖前保存 world 原始字节，供审计对比。
    (run / "world_before_sensor_override.sdf").write_bytes((run / "world.sdf").read_bytes())
    for e, text in paths:
        e.text = text
    candidate = copy.deepcopy(world)
    for e, text in originals:
        e.text = text
    if shape(world) != before:
        raise RuntimeError("Unexpected mutation outside allowed sensor leaves")
    # 只恢复这四个允许叶子后，整棵生成 world（含 physics、collisions、noise 与
    # extrinsics）必须回到覆盖前签名；随后用只改了四叶子的 candidate 取代它。
    tree.getroot().remove(world)
    tree.getroot().append(candidate)
    ET.indent(tree)
    tree.write(run / "world.sdf", encoding="unicode")
    # 同步 run 的 sensor_contract.json：只改 lidar 采样/频率与 camera 频率；
    # scope 字符串保持原样，作为“冻结的标称输入”说明。
    sensors = json.loads((run / "sensor_contract.json").read_text())
    old_sensor_sha = sha(run / "sensor_contract.json")
    sensors["lidar"]["rate_hz"] = lr
    sensors["lidar"]["samples"] = {"horizontal": h, "vertical": v}
    sensors["camera"]["rate_hz"] = cr
    sensors["scope"] = (
        "Frozen actual scene sampling inputs; effective rates require original-header measurement"
    )
    (run / "sensor_contract.json").write_text(json.dumps(sensors, indent=2) + "\n")
    # 覆盖证据：前后 world/sensor_contract 的 sha256，以及“只改允许叶子”的断言。
    # unchanged_* 三项是硬编码断言（物理步长 0.005 s、native PD 200 Hz、Actor
    # 50 Hz 未被本步骤改动），其来源与验证方式代码与现有文档未说明。
    # actual_rates_verified=False 与 nominal_30_is_not_measured_30=True 表示本步骤只
    # 写 nominal 值、不测量实际发布频率；navigation_ground_truth_used=False 表示
    # 未使用真值。
    receipt = {
        "schema": "teacher_sensor_sampling_trial/v1",
        "settings": settings,
        "before_world_sha256": sha(run / "world_before_sensor_override.sdf"),
        "after_world_sha256": sha(run / "world.sdf"),
        "before_sensor_contract_sha256": old_sensor_sha,
        "after_sensor_contract_sha256": sha(run / "sensor_contract.json"),
        "only_allowed_sensor_leaves_changed": True,
        "unchanged_physics_step_s": 0.005,
        "unchanged_native_PD_hz": 200,
        "unchanged_Actor_hz": 50,
        "unchanged_camera_K_D_geometry": True,
        "unchanged_lidar_fov_noise_extrinsics": True,
        "navigation_ground_truth_used": False,
        "actual_rates_verified": False,
        "nominal_30_is_not_measured_30": True,
    }
    (run / "sensor_sampling_contract.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return receipt
