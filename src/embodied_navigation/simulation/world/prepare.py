#!/usr/bin/env python3
"""Generate isolated Go2 assets. Never edit the camera or CHAMP assets."""

from embodied_navigation.project import project_root as _project_root
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as E

ROOT = _project_root()
DEMO = ROOT


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def rpy_quaternion(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def xml(p, e):
    E.indent(e)
    p.write_text(E.tostring(e, encoding="unicode") + "\n")


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--output", type=Path, default=ROOT / "simulation/generated")
    a.add_argument(
        "--terrain",
        choices=[
            "flat",
            "ramp_up",
            "ramp_down",
            "step05",
            "step10",
            "step05_continue",
            "step10_continue",
        ],
        default="flat",
    )
    a.add_argument("--sensors", action="store_true")
    a.add_argument("--camera-only", action="store_true")
    a.add_argument(
        "--overview-pose",
        type=float,
        nargs=6,
        default=None,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Override only the collision-free overview camera world pose",
    )
    a.add_argument(
        "--overview-fov",
        type=float,
        default=None,
        help="Optional diagnostic camera horizontal field of view in radians; no image warp",
    )
    a.add_argument(
        "--camera-rate",
        type=float,
        default=2.0,
        help="Actual vehicle RGB rate; observer camera remains 2 Hz",
    )
    a.add_argument(
        "--real-time-factor",
        type=float,
        default=1.0,
        help="Wall-time simulation speed; physics step and policy decimation stay fixed",
    )
    a.add_argument("--render-engine", choices=["ogre", "ogre2"], default="ogre2")
    args = a.parse_args()
    if args.overview_fov is not None and not 0 < args.overview_fov < math.pi:
        a.error("--overview-fov must be between 0 and pi radians")
    if not math.isfinite(args.camera_rate) or not 1 <= args.camera_rate <= 20:
        a.error("--camera-rate must be within 1..20 Hz")
    if not math.isfinite(args.real_time_factor) or not 0.1 <= args.real_time_factor <= 1:
        a.error("--real-time-factor must be within .1..1")
    args.output.mkdir(parents=True, exist_ok=True)
    source = ROOT / "assets/world/go2_converted.sdf"
    model = copy.deepcopy(E.parse(source).getroot().find("model"))
    for uri in model.iter("uri"):
        name = Path(uri.text or "").name
        mesh = ROOT / "assets/meshes" / name
        if mesh.is_file():
            uri.text = "file://" + str(mesh.resolve())
    # The only torque writer is the new plugin. Pose/odometry systems are read-only.
    for p in list(model.findall("plugin")):
        model.remove(p)
    qdefault = [-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5]
    mapping = [
        f"{leg}_{part}_joint"
        for leg in ["rf", "lf", "rh", "lh"]
        for part in ["hip", "upper_leg", "lower_leg"]
    ]
    lower = [-1.0472, -1.5708, -2.7227] * 2 + [-1.0472, -0.5236, -2.7227] * 2
    upper = [1.0472, 3.4907, -0.83776] * 2 + [1.0472, 4.5379, -0.83776] * 2
    for index, name in enumerate(mapping):
        j = model.find(f"joint[@name='{name}']")
        if j is None:
            raise ValueError(name)
        j.find("axis/limit/effort").text = "23.5"
        j.find("axis/limit/velocity").text = "30"
        j.find("axis/limit/lower").text = str(lower[index])
        j.find("axis/limit/upper").text = str(upper[index])
        dynamics = j.find("axis/dynamics")
        if dynamics is None:
            dynamics = E.SubElement(j.find("axis"), "dynamics")
        for field in ["damping", "friction", "spring_stiffness"]:
            e = dynamics.find(field)
            if e is None:
                e = E.SubElement(dynamics, field)
            e.text = "0"
    for link in model.findall("link"):
        for sensor in list(link.findall("sensor")):
            if (
                sensor.get("type") != "contact"
                and not args.sensors
                and not (args.camera_only and sensor.get("type") == "camera")
            ):
                link.remove(sensor)
            elif sensor.get("type") == "imu":
                sensor.find("update_rate").text = "200"
            elif sensor.get("type") == "camera":
                info = sensor.find("camera/camera_info_topic")
                if info is None:
                    info = E.SubElement(sensor.find("camera"), "camera_info_topic")
                info.text = "/demo/camera_info"
                # RGB is diagnostic; 2 Hz avoids competing with CPU inference.
                sensor.find("update_rate").text = str(args.camera_rate)
        for contact in link.findall("sensor[@type='contact']"):
            contact.find("update_rate").text = "200"
    # The original URDF contact friction was .2. Archive uses random .5..1.0;
    # deterministic matched baseline uses .7, never conceal this controlled change.
    for col in model.findall(".//collision"):
        for mu in col.findall("surface/friction/ode/mu") + col.findall("surface/friction/ode/mu2"):
            mu.text = ".7"
    # Flat tests are away from upper-deck overhead and scene walls. The
    # original (0,0) spawn has a third-floor deck above its height scan.
    x, y, z, yaw = 6, -0.7, 0.40, 0
    if args.terrain == "ramp_up":
        x, y, z = 4, 2, 0.60
    if args.terrain == "ramp_down":
        x, y, z, yaw = 8, 2, 1.0, math.pi
    pose = model.find("pose")
    if pose is None:
        pose = E.SubElement(model, "pose")
    pose.text = f"{x} {y} {z} 0 0 {yaw}"
    p = E.SubElement(
        model,
        "plugin",
        {"filename": "libteacher_actuator.so", "name": "teacher_sim::TeacherActuator"},
    )
    # Plugin source uses these native defaults too; persisted for review.
    for key, val in [
        ("kp", "25"),
        ("kd", ".5"),
        ("effort_limit", "23.5"),
        ("velocity_limit", "30"),
    ]:
        E.SubElement(p, key).text = val
    if args.sensors:
        # This system only reads physics joint components; it never writes a target.
        publisher = E.SubElement(
            model,
            "plugin",
            {
                "filename": "gz-sim-joint-state-publisher-system",
                "name": "gz::sim::systems::JointStatePublisher",
            },
        )
        E.SubElement(publisher, "topic").text = "/demo/teacher/physical_joint_states"
        for name in mapping:
            E.SubElement(publisher, "joint_name").text = name
    world = E.parse(ROOT / "assets/world/three_floors.sdf").getroot()
    w = world.find("world")
    w.set("name", "teacher_demo")
    w.find("physics/max_step_size").text = ".005"
    factor = w.find("physics/real_time_factor")
    if factor is None:
        factor = E.SubElement(w.find("physics"), "real_time_factor")
    factor.text = str(args.real_time_factor)
    if args.sensors or args.camera_only:
        # Hardware Ogre2 completed actual full sensors and natural shutdown.
        # Software Ogre1 is retained only as an explicit failed diagnostic.
        for engine in w.findall("plugin/render_engine"):
            engine.text = args.render_engine
    if not args.sensors and not args.camera_only:
        for wp in list(w.findall("plugin")):
            if any(n in wp.get("name", "") for n in ["Sensors", "Imu"]):
                w.remove(wp)
    if args.sensors or args.camera_only:
        # A separate static, collision-free observer shows actual leg motion.
        # It is never part of the robot or the policy observation geometry.
        overview_pose = (
            [8.5, -3.0, 1.4, 0, 0.3, math.pi / 2]
            if args.terrain.endswith("_continue")
            else [5, -3.2, 1.7, 0, 0.4, 0.95]
        )
        if args.overview_pose is not None:
            overview_pose = args.overview_pose
        overview_rate = 10 if args.terrain.endswith("_continue") else 2
        overview_fov = 1.8 if args.terrain.endswith("_continue") else 1.4
        if args.overview_fov is not None:
            overview_fov = args.overview_fov
        overview = E.fromstring(
            f'<model name="teacher_overview_camera"><static>true</static><pose>{" ".join(map(str, overview_pose))}</pose><link name="camera_link"><sensor name="overview" type="camera"><topic>/demo/teacher/overview</topic><always_on>true</always_on><update_rate>{overview_rate}</update_rate><gz_frame_id>teacher_overview_frame</gz_frame_id><camera><horizontal_fov>{overview_fov}</horizontal_fov><image><width>640</width><height>480</height><format>R8G8B8</format></image><clip><near>.1</near><far>100</far></clip><camera_info_topic>/demo/teacher/overview_info</camera_info_topic></camera></sensor></link></model>'
        )
        w.append(overview)
    if args.terrain.startswith("step"):
        height = 0.05 if args.terrain.startswith("step05") else 0.1
        length = 6 if args.terrain.endswith("_continue") else 2
        center_x = 7 + length / 2
        step = E.fromstring(
            f'<model name="teacher_low_step"><static>true</static><pose>{center_x} -.7 {height / 2} 0 0 0</pose><link name="link"><collision name="collision"><geometry><box><size>{length} 1.4 {height}</size></box></geometry></collision><visual name="visual"><geometry><box><size>{length} 1.4 {height}</size></box></geometry><material><ambient>.8 .5 .2 1</ambient><diffuse>.8 .5 .2 1</diffuse></material></visual></link></model>'
        )
        w.append(step)
    w.append(model)
    xml(args.output / "world.sdf", world)
    sensor_contract = {
        "schema_version": 1,
        "renderer": args.render_engine if args.sensors or args.camera_only else None,
        "scope": "read-only sensor diagnostics; privileged actor remains unchanged; no SLAM or navigation started",
        "joint_states": {
            "gz_topic": "/demo/teacher/physical_joint_states",
            "ros_topic": "/demo/control/measured_joint_states",
            "publisher": "gz::sim::systems::JointStatePublisher",
            "enabled": args.sensors,
            "reads": "native physics joint position/velocity",
            "rate_hz": 200,
            "actuation": False,
        },
    }
    if args.sensors or args.camera_only:
        sensor_contract["overview"] = {
            "gz_topic": "/demo/teacher/overview",
            "ros_topic": "/demo/teacher/overview",
            "frame": "teacher_overview_frame",
            "world_pose": overview_pose,
            "rate_hz": overview_rate,
            "width": 640,
            "height": 480,
            "horizontal_fov": overview_fov,
            "collision_geometry": False,
            "gz_camera_info_topic": "/demo/teacher/overview_info",
            "ros_camera_info_topic": "/demo/teacher/overview_info",
            "camera_model": "default pinhole; no SDF distortion configured; confirm with actual CameraInfo",
            "purpose": "observer only; never pair this image with vehicle SLAM camera intrinsics",
        }
    for link in model.findall("link"):
        for sensor in link.findall("sensor"):
            kind = sensor.get("type")
            if kind not in ["imu", "camera", "gpu_lidar"]:
                continue
            values = [float(v) for v in sensor.findtext("pose", "0 0 0 0 0 0").split()]
            key = "lidar" if kind == "gpu_lidar" else kind
            entry = {
                "type": kind,
                "link": link.get("name"),
                "sensor_name": sensor.get("name"),
                "body_xyz": values[:3],
                "body_rpy": values[3:],
                "rate_hz": float(sensor.findtext("update_rate", "0")),
                "frame": sensor.findtext("gz_frame_id"),
                "gz_topic": "/" + sensor.findtext("topic", "").lstrip("/"),
            }
            if kind == "imu":
                custom = sensor.find("imu/orientation_reference_frame/custom_rpy")
                localization = sensor.findtext("imu/orientation_reference_frame/localization")
                world_rpy = [float(v) for v in custom.text.split()] if custom is not None else None
                entry["ros_topic"] = "/livox/imu"
                entry["orientation_reference"] = {
                    "localization": localization,
                    "custom_parent_frame": custom.get("parent_frame")
                    if custom is not None
                    else None,
                    "world_rpy": world_rpy,
                    "world_quaternion": rpy_quaternion(world_rpy)
                    if world_rpy is not None
                    else None,
                    "body_imu_quaternion": rpy_quaternion(values[3:]),
                    "description": "actual generated SDF IMU CUSTOM/world reference",
                }
            elif kind == "gpu_lidar":
                entry["gz_points_topic"] = entry["gz_topic"] + "/points"
                entry["ros_topic"] = "/demo/teacher/raw_lidar"
                entry["cloud_registered_full"] = False
                entry["samples"] = {
                    "horizontal": int(sensor.findtext("lidar/scan/horizontal/samples", "0")),
                    "vertical": int(sensor.findtext("lidar/scan/vertical/samples", "0")),
                }
            else:
                entry["ros_topic"] = "/demo/camera"
                entry["gz_camera_info_topic"] = sensor.findtext("camera/camera_info_topic")
                entry["ros_camera_info_topic"] = "/demo/camera_info"
                entry["camera_model"] = (
                    "default pinhole; no SDF distortion configured; confirm with actual CameraInfo"
                )
                entry["horizontal_fov"] = float(sensor.findtext("camera/horizontal_fov"))
                entry["width"] = int(sensor.findtext("camera/image/width"))
                entry["height"] = int(sensor.findtext("camera/image/height"))
            sensor_contract[key] = entry
    (args.output / "sensor_contract.json").write_text(json.dumps(sensor_contract, indent=2) + "\n")
    bridge = []

    def bridged(gz_topic, ros_topic, gz_type, ros_type):
        bridge.append(
            {
                "gz_topic_name": gz_topic,
                "ros_topic_name": ros_topic,
                "gz_type_name": gz_type,
                "ros_type_name": ros_type,
                "direction": "GZ_TO_ROS",
                "qos_profile": "SENSOR_DATA",
                "publisher_queue": 5,
                "subscriber_queue": 5,
                "lazy": False,
            }
        )

    if args.sensors or args.camera_only:
        bridged("/demo/camera", "/demo/camera", "gz.msgs.Image", "sensor_msgs/msg/Image")
        bridged(
            "/demo/teacher/overview",
            "/demo/teacher/overview",
            "gz.msgs.Image",
            "sensor_msgs/msg/Image",
        )
        bridged(
            "/demo/camera_info",
            "/demo/camera_info",
            "gz.msgs.CameraInfo",
            "sensor_msgs/msg/CameraInfo",
        )
        bridged(
            "/demo/teacher/overview_info",
            "/demo/teacher/overview_info",
            "gz.msgs.CameraInfo",
            "sensor_msgs/msg/CameraInfo",
        )
        bridged("/clock", "/clock", "gz.msgs.Clock", "rosgraph_msgs/msg/Clock")
    if args.sensors:
        bridged("/imu/data", "/livox/imu", "gz.msgs.IMU", "sensor_msgs/msg/Imu")
        bridged(
            "/demo/teacher/physical_joint_states",
            "/demo/control/measured_joint_states",
            "gz.msgs.Model",
            "sensor_msgs/msg/JointState",
        )
        bridged(
            "/velodyne_points/points",
            "/demo/teacher/raw_lidar",
            "gz.msgs.PointCloudPacked",
            "sensor_msgs/msg/PointCloud2",
        )
    # JSON is valid YAML; bridge configuration is frozen inside this run.
    (args.output / "sensor_bridge.yaml").write_text(json.dumps(bridge, indent=2) + "\n")
    # Read-only ROS frame/joint tree, stripped of every actuation element.
    urdf = E.parse(ROOT / "assets/world/go2_measured.urdf").getroot()
    for mesh in urdf.iter("mesh"):
        name = Path(mesh.get("filename", "")).name
        asset = ROOT / "assets/meshes" / name
        if not asset.is_file():
            raise ValueError("Project robot mesh missing: " + name)
        mesh.set("filename", "file://" + str(asset.resolve()))
    for tag in ["ros2_control", "gazebo", "transmission"]:
        for e in list(urdf.findall(tag)):
            urdf.remove(e)
    xml(args.output / "frames.urdf", urdf)
    meta = {
        "terrain": args.terrain,
        "sensors": args.sensors,
        "camera_only": args.camera_only,
        "render_engine": args.render_engine if args.sensors or args.camera_only else None,
        "spawn": [x, y, z, yaw],
        "physics_step_s": 0.005,
        "decimation": 4,
        "policy_hz": 50,
        "real_time_factor": args.real_time_factor,
        "exclusive_writer": "teacher_sim::TeacherActuator",
        "disabled": ["CHAMP", "body_stabilizer", "ros2_control", "JointTrajectoryController"],
        "joint_mapping": dict(zip(["FR", "FL", "RR", "RL"], ["rf", "lf", "rh", "lh"])),
        "joint_names": mapping,
        "default_q": qdefault,
        "hard_lower": lower,
        "hard_upper": upper,
        "friction_baseline": 0.7,
        "model_mass_kg": sum(float(v.text) for v in model.findall("link/inertial/mass")),
        "base_inertial_pose": model.find("link[@name='base_link']/inertial/pose").text,
        "source_hashes": {str(s): sha(s) for s in [source, ROOT / "assets/world/three_floors.sdf"]},
        "world_sha256": sha(args.output / "world.sdf"),
        "true_stairs": False,
    }
    meta["sensor_contract_sha256"] = sha(args.output / "sensor_contract.json")
    meta["sensor_bridge_sha256"] = sha(args.output / "sensor_bridge.yaml")
    if args.terrain.endswith("_continue"):
        meta["step_fixture"] = {
            "front_x": 7,
            "back_x": 13,
            "center_y": -0.7,
            "width_m": 1.4,
            "height_m": height,
            "scope": "Same front edge/height/width as original, tread extended from2m to6m for continuous walking retest",
        }
    (args.output / "asset_manifest.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(json.dumps(meta))


if __name__ == "__main__":
    main()
