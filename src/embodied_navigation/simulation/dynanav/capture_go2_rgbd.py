"""Headless Isaac Sim probe of the DynaNav sensor/command interface.

Standalone diagnostic script (not a ROS node of the navigation stack): it opens
the USD stage given on the command line, spawns the Go2 articulation at the
environment-provided spawn pose, publishes IMU, RGB and depth-derived cloud on
``/dynanav/imu``, ``/dynanav/camera`` and ``/dynanav/lidar_filtered`` and
consumes approved body-twist commands from ``/dynanav/nav/approved_cmd`` for a
bounded number of physics steps. Every stage receipt is written to
``<output>/result.json``; an unexpected exception records its traceback there
and is re-raised so the caller sees a non-zero exit.
"""

import faulthandler
import json
import os
import pathlib
import sys
import time
import traceback

from embodied_navigation.perception.depth import depth_cloud_packet
from embodied_navigation.project import data_root, project_root

out = pathlib.Path(sys.argv[1])
out.mkdir(exist_ok=True)
experiment_path = os.environ.get("DYNANAV_EXPERIMENT")
experiment = (
    json.loads(pathlib.Path(experiment_path).read_text()) if experiment_path else {}
)
gt_diagnostic = experiment.get("control_mode") == "native_pose_diagnostic"
slam_control = experiment.get("control_mode") == "slam_pose_diagnostic"
robogauge = experiment.get("policy") == "robogauge"
use_fabric = experiment.get("render_backend", "fabric") == "fabric"
if (gt_diagnostic or slam_control) and not robogauge:
    raise ValueError(
        "Native-pose diagnostic requires the explicit RoboGauge experiment"
    )
result = {
    "stage": "startup",
    "experiment": experiment,
    "ground_truth_navigation": gt_diagnostic,
    "SLAM_control_enabled": not gt_diagnostic,
    "stair_speed_cap_enabled": False,
    "truth_usage": "policy proprioception, physics guards, diagnostic mapping and native-pose tracking"
    if gt_diagnostic
    else "policy proprioception, physical guards, diagnostic oracle map and scoring; LIO estimates alone drive the route"
    if slam_control
    else "policy proprioception, COM velocity PID and scoring; navigation consumes SLAM",
    "navigation_running": False,
    "policy_loaded": False,
    "rgb_rendered": False,
}


_last_receipt_wall = -float("inf")
_journals = {}


def save(force=False):
    # Receipt I/O must not run at the 200 Hz physics rate on the data volume.
    global _last_receipt_wall
    now = time.perf_counter()
    if not force and now - _last_receipt_wall < 0.25:
        return
    temporary = out / "result.json.tmp"
    temporary.write_text(json.dumps(result, indent=2))
    temporary.replace(out / "result.json")
    _last_receipt_wall = now


def journal(name, payload):
    if name not in _journals:
        _journals[name] = (out / name).open("a")
    _journals[name].write(json.dumps(payload) + "\n")


def close_journals():
    for stream in _journals.values():
        stream.close()


rgb_archive = None
rgb_frames_acquired = 0
save()
try:
    if experiment.get("archive_rgb", False):
        from embodied_navigation.simulation.dynanav.rgb_archive import RawRgbArchive

        if not out.resolve().is_relative_to(data_root().resolve()):
            raise ValueError(
                "Raw RGB archive must be below the external project data root"
            )
        rgb_archive = RawRgbArchive(out / "rgb_raw")
        result["rgb_archive"] = rgb_archive.receipt(acquired_frames=0)
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "width": 512,
            "height": 512,
            "renderer": "RaytracedLighting",
            "create_new_stage": False,
            "disable_viewport_updates": True,
            "multi_gpu": False,
            "sync_loads": False,
            "limit_cpu_threads": 8,
            "extra_args": [
                "--/app/settings/persistent=0",
                "--/app/extensions/registryEnabled=false",
                "--/app/fastShutdown=true",
                "--/rtx/hydra/supportMultiTickRate=false",
                f"--/app/useFabricSceneDelegate={'true' if use_fabric else 'false'}",
            ],
        }
    )
    from omni.kit.viewport.window import get_viewport_window_instances
    from omni.kit.widget.viewport.display_delegate import ViewportDisplayDelegate

    class HeadlessFixedDisplayDelegate(ViewportDisplayDelegate):
        def create(self, ui_frame, previous, *args, **kwargs):
            return None

        def destroy(self):
            pass

        @property
        def size(self):
            return (512, 512)

        def update(self, *args, **kwargs):
            return (512, 512)

    _headless_previous_delegates = []
    _headless_delegates = []
    for _viewport_window in list(get_viewport_window_instances()):
        _headless_previous_delegates.append(
            _viewport_window.viewport_widget.display_delegate
        )
        _delegate = HeadlessFixedDisplayDelegate(_viewport_window.viewport_api)
        _headless_delegates.append(_delegate)
        _viewport_window.viewport_widget.display_delegate = _delegate
    result["headless_fixed_display_delegate_count"] = len(_headless_delegates)
    result["rendering_dependencies_unloaded"] = False
    save()
    faulthandler.dump_traceback_later(25, repeat=True)
    import numpy as np
    import omni.usd
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, UsdGeom, UsdPhysics

    spawn = np.array(
        [
            float(os.environ.get("GO2_SPAWN_X", "-10.6")),
            float(os.environ.get("GO2_SPAWN_Y", "-14.5")),
            float(experiment.get("spawn", [0, 0, 0.4])[2]),
        ]
    )
    if experiment.get("spawn"):
        spawn = np.asarray(experiment["spawn"], dtype=float)
    spawn_yaw = float(experiment.get("spawn_yaw", os.environ.get("GO2_SPAWN_YAW", "0")))
    sim_rtf = float(os.environ.get("PROBE_RTF", ".5"))
    import isaacsim.core.experimental.utils.app as au
    from isaacsim.core.experimental.prims import Articulation, RigidPrim
    from isaacsim.core.simulation_manager import SimulationManager

    assert omni.usd.get_context().open_stage(sys.argv[2])
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    if experiment.get("basement_collision"):
        from embodied_navigation.simulation.dynanav.session_collision import (
            repair_basement_session,
        )

        result["session_collision"] = repair_basement_session(
            stage, config_path=experiment.get("collision_config_path")
        )
        save()
    pedestrian = None
    if experiment.get("pedestrian"):
        from embodied_navigation.simulation.dynanav.pedestrian import create_pedestrian

        pedestrian_config = experiment["pedestrian"]
        pedestrian = create_pedestrian(
            stage,
            pedestrian_config["asset"],
            start=pedestrian_config["start"],
            end=pedestrian_config["end"],
            speed_mps=pedestrian_config["speed_mps"],
            path_mode=pedestrian_config["path_mode"],
            animation_path=pedestrian_config["walk"],
        )
        result["pedestrian"] = pedestrian.report
    asset = str(
        data_root() / "runtime_assets/robot/go2_description/go2_description.usda"
    )
    prim = stage.DefinePrim("/World/Go2")
    prim.GetReferences().AddReference(asset)
    result["stage"] = "asset_loading"
    save()
    x = UsdGeom.Xformable(prim)
    ops = x.GetOrderedXformOps()
    translate = next(
        (o for o in ops if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None
    )
    (translate or x.AddTranslateOp()).Set(Gf.Vec3d(*spawn))
    roots = [
        str(p.GetPath())
        for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    result["articulation_roots"] = roots
    result["asset"] = asset
    save()
    assert len(roots) == 1
    # Force drive gains are expected zero in this converted asset; refuse competing drive ownership.
    drives = []
    for p in stage.Traverse():
        if p.IsA(UsdPhysics.RevoluteJoint):
            drive = UsdPhysics.DriveAPI.Get(p, "angular")
            stiff = drive.GetStiffnessAttr().Get() if drive else 0.0
            damp = drive.GetDampingAttr().Get() if drive else 0.0
            drives.append(
                {"path": str(p.GetPath()), "stiffness": stiff, "damping": damp}
            )
            assert float(stiff or 0) == 0 and float(damp or 0) == 0, (
                "Existing nonzero joint drive; refuse second actuator"
            )
    result["existing_drives"] = drives
    # Match documented training spawn properties; no gain/torque tuning.
    for p in stage.Traverse():
        if str(p.GetPath()).startswith("/World/Go2") and p.HasAPI(
            UsdPhysics.RigidBodyAPI
        ):
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(p)
            api.CreateLinearDampingAttr(0.0)
            api.CreateAngularDampingAttr(0.0)
            api.CreateMaxDepenetrationVelocityAttr(1.0)
    for path in roots:
        api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(path))
        api.CreateEnabledSelfCollisionsAttr(False)
        api.CreateSolverPositionIterationCountAttr(8)
        api.CreateSolverVelocityIterationCountAttr(4)
    result["training_spawn_properties"] = {
        "solver_position_iterations": 8,
        "solver_velocity_iterations": 4,
        "self_collisions": False,
        "linear_and_angular_damping": 0.0,
        "max_depenetration_velocity": 1.0,
        "source": "RL_for_unitree/assets/robots.py _common_spawn",
    }
    contact_state = [0, 0, 0, 0, 0]
    contact_events = []
    dynamic_contact_events = []
    contact_paths = {}
    callback_counter = 0
    for p in stage.Traverse():
        if str(p.GetPath()).startswith("/World/Go2") and p.HasAPI(
            UsdPhysics.RigidBodyAPI
        ):
            PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.0)

    def on_contact(headers, data, anchors=None):
        global callback_counter
        callback_counter += 1
        for h in headers:
            if h.num_contact_data <= 0:
                continue
            paths = [
                str(PhysicsSchemaTools.intToSdfPath(h.collider0)),
                str(PhysicsSchemaTools.intToSdfPath(h.collider1)),
            ]
            actors = [
                str(PhysicsSchemaTools.intToSdfPath(h.actor0)),
                str(PhysicsSchemaTools.intToSdfPath(h.actor1)),
            ]
            if (
                any(path.startswith("/World/DynamicPedestrian") for path in paths)
                and any(path.startswith("/World/Go2") for path in paths)
                and len(dynamic_contact_events) < 1000
            ):
                dynamic_contact_events.append(
                    {
                        "physics_s": float(SimulationManager.get_simulation_time()),
                        "colliders": paths,
                        "actors": actors,
                        "contact_count": int(h.num_contact_data),
                    }
                )
            for j, path in enumerate(paths):
                if not path.startswith("/World/Go2"):
                    continue
                key = None
                if actors[j] == "/World/Go2/Geometry/base":
                    key = 0
                for k, leg in enumerate(["FR", "FL", "RR", "RL"], 1):
                    if actors[j].split("/")[-1] == leg + "_calf" and path.endswith(
                        "/sphere"
                    ):
                        sphere = UsdGeom.Sphere(stage.GetPrimAtPath(path))
                        assert (
                            abs(float(sphere.GetRadiusAttr().Get()) - 0.022) < 1e-6
                        ), "Foot shape differs"
                        key = k
                contact_paths[path] = {"actor": actors[j], "group": key}
                if key is not None:
                    contact_state[key] += 1
            if len(contact_events) < 3000:
                ratios = []
                for pnt in data[
                    h.contact_data_offset : h.contact_data_offset + h.num_contact_data
                ]:
                    normal = np.array(pnt.normal)
                    impulse = np.array(pnt.impulse)
                    fn = float(abs(impulse @ normal))
                    ft = float(np.linalg.norm(impulse - (impulse @ normal) * normal))
                    ratios.append(
                        {
                            "normal_impulse": fn,
                            "tangent_impulse": ft,
                            "ratio": ft / fn if fn > 1e-8 else None,
                        }
                    )
                anchor_impulses = (
                    [
                        list(map(float, x.impulse))
                        for x in anchors[
                            h.friction_anchors_offset : h.friction_anchors_offset
                            + h.num_friction_anchors_data
                        ]
                    ]
                    if anchors is not None
                    else []
                )
                contact_events.append(
                    {
                        "physics_time": float(SimulationManager.get_simulation_time()),
                        "friction_anchor_impulses": anchor_impulses,
                        "colliders": paths,
                        "actors": actors,
                        "count": int(h.num_contact_data),
                        "impulses": ratios,
                    }
                )

    import omni.physx

    contact_sub = omni.physx.get_physx_simulation_interface().subscribe_full_contact_report_events(
        on_contact
    )
    # Sensors are mounted on the actual articulation base, never a separate rig.
    theta = float(
        experiment.get(
            "camera_yaw_rad", os.environ.get("GO2_CAMERA_YAW", "1.5707963267948966")
        )
    )
    sensor_yaw = np.array(
        [
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1.0],
        ]
    )
    Rcl = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]]) @ sensor_yaw.T
    base_path = "/World/Go2/Geometry/base"
    mount = np.array([0.3, 0.0, 0.13])
    imu_mount = np.array(
        UsdPhysics.MassAPI(stage.GetPrimAtPath(base_path)).GetCenterOfMassAttr().Get(),
        dtype=float,
    )
    dynamic_guard = None
    if experiment.get("dynamic_hold"):
        from embodied_navigation.simulation.dynanav.dynamic_hold import (
            DynamicHold,
            DynamicHoldConfig,
        )

        dynamic_guard = DynamicHold(
            DynamicHoldConfig(recorded_cruise_mps=experiment["speed_mps"]),
            imu_origin_in_base_m=imu_mount,
            mask_provenance="isaac_instance_oracle",
        )
    camera_path = base_path + "/DiagnosticCamera"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera.AddTransformOp().Set(
        Gf.Matrix4d(
            Gf.Matrix3d(
                *(np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]]) @ sensor_yaw.T)
                .flatten()
                .tolist()
            ),
            Gf.Vec3d(*mount),
        )
    )
    camera.GetHorizontalApertureAttr().Set(42.0)
    camera.GetVerticalApertureAttr().Set(42.0)
    camera.GetFocalLengthAttr().Set(24.0)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))
    from isaacsim.sensors.experimental.physics import IMU, IMUSensor

    imu_sensor = IMUSensor(
        IMU.create(
            base_path + "/DiagnosticIMU",
            translations=[imu_mount.tolist()],
            linear_acceleration_filter_size=1,
            angular_velocity_filter_size=1,
            orientation_filter_size=1,
        )
    )
    import omni.replicator.core as rep

    rep.orchestrator.set_capture_on_play(False)
    product = rep.create.render_product(camera_path, (512, 512))
    rgb_ann = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_ann.attach([product])
    depth_ann = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    depth_ann.attach([product])
    camera_params_ann = rep.AnnotatorRegistry.get_annotator("CameraParams")
    camera_params_ann.attach([product])
    oracle_mapper = None
    instance_ann = None
    if experiment.get("semantic_provider") == "isaac_instance_oracle":
        if not (gt_diagnostic or slam_control):
            raise ValueError("Oracle diagnostic maps require an explicit experiment")
        from embodied_navigation.simulation.dynanav.oracle_mapping import OracleMapping

        oracle_mapper = OracleMapping(stage, out / "mapping", experiment)
        instance_ann = rep.AnnotatorRegistry.get_annotator("instance_id_segmentation")
        instance_ann.attach([product])
    spectator = UsdGeom.Camera.Define(stage, base_path + "/PIDOverviewCamera")
    eye = np.array([-1.8, -1.35, 1.15])
    focus = np.array([0.15, 0.0, -0.1])
    spectator.AddTransformOp().Set(
        Gf.Matrix4d()
        .SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*focus), Gf.Vec3d(0, 0, 1))
        .GetInverse()
    )
    spectator.GetFocalLengthAttr().Set(22.0)
    spectator.GetHorizontalApertureAttr().Set(35.0)
    spectator.GetVerticalApertureAttr().Set(35.0)
    spectator.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))
    overview_product = rep.create.render_product(
        base_path + "/PIDOverviewCamera", (512, 512)
    )
    overview_ann = rep.AnnotatorRegistry.get_annotator("rgb")
    overview_ann.attach([overview_product])
    import subprocess

    from PIL import Image as PILImage
    from PIL import ImageDraw

    video_process = subprocess.Popen(
        [
            "/usr/bin/ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
            "1024x512",
            "-framerate",
            "10",
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            "libx264",
            "-threads",
            "2",
            "-preset",
            "ultrafast",
            "-crf",
            "23",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(out / "multiwaypoint05_overview_and_rgb.mp4"),
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=(out / "video_encoder.log").open("w"),
    )
    video_frames = 0

    result["sensor_installation"] = {
        "parent": base_path,
        "imu_translation_in_base": imu_mount.tolist(),
        "camera_optical_translation_from_IMU": (Rcl @ (imu_mount - mount)).tolist(),
        "camera_translation_in_base": mount.tolist(),
        "camera_optical_R_from_IMU": Rcl.tolist(),
        "custom_sensor_yaw_degrees": float(np.rad2deg(theta)),
        "camera_intrinsics": [512 * 24 / 42, 512 * 24 / 42, 256, 256],
        "modality": "global-shutter RGB-D, not rotating lidar",
        "truth_only_for_scoring": False,
        "truth_usage": result["truth_usage"],
        "max_simulation_real_time_factor": sim_rtf,
        "pacing_changes_physics_dt": False,
    }
    save()
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("actual_go2_rgbd_source")
    pub_imu = node.create_publisher(Imu, "/dynanav/imu", qos_profile_sensor_data)
    pub_img = node.create_publisher(Image, "/dynanav/camera", qos_profile_sensor_data)
    pub_cloud = node.create_publisher(
        PointCloud2, "/dynanav/lidar_filtered", qos_profile_sensor_data
    )
    pub_static_cloud = node.create_publisher(
        PointCloud2, "/dynanav/lidar_static", qos_profile_sensor_data
    )
    pub_depth = node.create_publisher(Image, "/dynanav/depth", qos_profile_sensor_data)
    pub_info = node.create_publisher(
        CameraInfo, "/dynanav/camera_info", qos_profile_sensor_data
    )
    latest_odom = [None]

    def odom_callback(m):
        latest_odom[0] = {
            "time": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            "position": [
                m.pose.pose.position.x,
                m.pose.pose.position.y,
                m.pose.pose.position.z,
            ],
            "orientation_wxyz": [
                m.pose.pose.orientation.w,
                m.pose.pose.orientation.x,
                m.pose.pose.orientation.y,
                m.pose.pose.orientation.z,
            ],
            "estimated_velocity": [
                m.twist.twist.linear.x,
                m.twist.twist.linear.y,
                m.twist.twist.linear.z,
            ],
            "frame_id": m.header.frame_id,
            "child_frame_id": m.child_frame_id,
            "arrival_wall": time.perf_counter(),
        }

    odom_sub = node.create_subscription(
        Odometry,
        experiment.get("odometry_topic", "/aft_mapped_to_init"),
        odom_callback,
        qos_profile_sensor_data,
    )
    latest_nav = [None]
    pub_clock = node.create_publisher(Clock, "/clock", 10)

    def nav_callback(m):
        latest_nav[0] = {
            "time": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            "request": [m.twist.linear.x, m.twist.linear.y, m.twist.angular.z],
            "arrival_wall": time.perf_counter(),
        }

    nav_sub = node.create_subscription(
        TwistStamped, "/dynanav/nav/approved_cmd", nav_callback, 10
    )

    def header(m, stamp, frame):
        ns = round(stamp * 1e9)
        m.header.stamp.sec = ns // 1000000000
        m.header.stamp.nanosec = ns % 1000000000
        m.header.frame_id = frame

    published = {"imu": 0, "rgb": 0, "cloud": 0}
    art = Articulation(roots[0])
    SimulationManager.enable_fabric(use_fabric)
    SimulationManager.setup_simulation(dt=0.005, device="cpu")
    import carb.settings

    settings = carb.settings.get_settings()
    result["runtime_render_path"] = {
        "requested": "fabric" if use_fabric else "usd",
        "physics_fabric_enabled": SimulationManager.is_fabric_enabled(),
        "fabric_scene_delegate": settings.get("/app/useFabricSceneDelegate"),
        "physics_update_to_usd": settings.get("/physics/updateToUsd"),
        "physics_suppress_readback": settings.get("/physics/suppressReadback"),
    }
    from isaacsim.core.rendering_manager import RenderingManager

    RenderingManager.set_dt(0.005)
    au.play()
    app.update()
    rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=True)
    result["dof_names"] = art.dof_names
    result["link_names"] = art.link_names
    save()
    contract = json.loads(
        (
            project_root()
            / "src/embodied_navigation/robot/unitree/policy/native/contract.json"
        ).read_text()
    )
    canonical = contract["joint_names"]
    assert len(art.dof_names) == 12 and set(art.dof_names) == set(canonical), (
        "Asset joint identity differs from navigation contract"
    )
    indices = [art.dof_names.index(n) for n in canonical]
    result["canonical_to_isaac_dof_indices"] = indices
    target = np.zeros(12, dtype=np.float32)
    for name, q in zip(canonical, contract["default_joint_positions"]):
        target[art.dof_names.index(name)] = q
    result["root_pose_before_spawn"] = [
        a.numpy().tolist() for a in art.get_world_poses()
    ]
    art.set_world_poses(
        positions=[spawn.tolist()],
        orientations=[[np.cos(spawn_yaw / 2), 0.0, 0.0, np.sin(spawn_yaw / 2)]],
    )
    result["root_pose_after_spawn"] = [
        a.numpy().tolist() for a in art.get_world_poses()
    ]
    result["root_velocity_before_reset"] = [
        a.numpy().tolist() for a in art.get_velocities()
    ]
    art.set_velocities(
        linear_velocities=[[0.0, 0.0, 0.0]], angular_velocities=[[0.0, 0.0, 0.0]]
    )
    result["root_velocity_after_reset"] = [
        a.numpy().tolist() for a in art.get_velocities()
    ]
    save()
    art.set_dof_positions(target[None, :])
    art.set_dof_velocities(np.zeros((1, 12), np.float32))
    art.set_dof_max_efforts(np.full((1, 12), 23.5))
    art.set_dof_max_velocities(np.full((1, 12), 30.0))
    calf_views = [
        RigidPrim(
            "/World/Go2/Geometry/base/"
            + leg
            + "_hip/"
            + leg
            + "_thigh/"
            + leg
            + "_calf",
            reset_xform_op_properties=False,
        )
        for leg in ["FR", "FL", "RR", "RL"]
    ]
    from embodied_navigation.simulation.dynanav.collision_bounds import export_bounds

    export_bounds(stage, out)
    result["link_coms_local"] = [v.numpy().tolist() for v in art.get_link_coms()]
    assert np.allclose(
        np.array(result["link_coms_local"][0][0][0]), imu_mount, atol=1e-6
    ), "USD and physical COM disagree"
    result["implicit_gains_runtime"] = [v.numpy().tolist() for v in art.get_dof_gains()]
    assert all(np.allclose(v.numpy(), 0) for v in art.get_dof_gains()), (
        "Competing drive after initialization"
    )
    import hashlib

    import torch

    sys.path.insert(0, str(project_root() / "src"))
    from embodied_navigation.robot.unitree.policy import speed_tiers
    from embodied_navigation.robot.unitree.policy.native.observation import (
        quaternion_rotation,
    )
    from embodied_navigation.robot.unitree.policy.student import (
        PID_SPEC,
        CorrectionOnlyPID,
        exact_student45,
    )

    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    hidden = torch.zeros(1, 1, 256)
    last_action = np.zeros(12)
    qdefault = np.array(contract["default_joint_positions"])
    policy_frames = []
    cmd = np.zeros(3)
    tracker = None
    if robogauge:
        from embodied_navigation.robot.unitree.robogauge.contract import go2hv_torque
        from embodied_navigation.robot.unitree.robogauge.runtime import Runtime

        actor = Runtime(canonical=canonical)
        assert np.allclose(qdefault, actor.qdefault), (
            "RoboGauge default joint pose differs"
        )
        model_path = actor.path
        result["policy"] = actor.metadata()
        pid = None
    else:
        model_path = project_root() / "models/student_model_4999_deterministic_jit.pt"
        actor = torch.jit.load(str(model_path), map_location="cpu").eval()
        pid = CorrectionOnlyPID(
            {
                "pid": PID_SPEC,
                "command_bounds": [0.3, 0.2, 0.3],
                "command_slew_acceleration": [0.6, 0.6, 0.8],
            },
            [0.6, 0.05, 0.15],
            speed_tier=speed_tiers.choice(0.5),
        )
        result["policy"] = {
            "path": str(model_path),
            "sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
            "device": "CPU",
            "observation": "reused exact_student45",
        }
    pedestrian_view = None
    if pedestrian is not None:
        previous_edit_target = stage.GetEditTarget()
        try:
            stage.SetEditTarget(stage.GetSessionLayer())
            pedestrian_view = RigidPrim(
                pedestrian.root_path, reset_xform_op_properties=False
            )
        finally:
            stage.SetEditTarget(previous_edit_target)
        assert pedestrian_view.is_physics_tensor_entity_valid(), (
            "Pedestrian native rigid-body view unavailable"
        )
    target_speed = float(experiment.get("speed_mps", 0.5))
    physical_speed_guard = float(experiment.get("physical_speed_guard_mps", 0.65))
    if gt_diagnostic:
        from embodied_navigation.navigation.robogauge.route_tracker import RouteTracker

        tracker = RouteTracker(
            experiment["waypoints_xyz"],
            spawn[:2].tolist(),
            speed=target_speed,
            motion_end=experiment["motion_end_s"],
            stop_on_stall=experiment.get("stop_on_stall", True),
        )
    if slam_control:
        from embodied_navigation.simulation.dynanav.experiment import (
            make_slam_controller,
        )

        tracker = make_slam_controller(experiment)
    stand_gate_done = False
    policy_stand_gate_done = False
    result["speed_test"] = {
        "target_mps": target_speed,
        "physical_speed_guard_mps": physical_speed_guard,
        "tilt_guard_rad": 0.5,
        "yaw_command_limit_radps": 0.15,
        "stair_specific_speed_limit": None,
        "native_COM_velocity_PID": not robogauge,
        "command_slew": not robogauge,
    }
    result["policy_loaded"] = True
    result["policy"].update(
        physics_hz=200,
        policy_hz=50,
        command_source="native_pose_diagnostic"
        if gt_diagnostic
        else "measured_lio_route"
        if slam_control
        else "/dynanav/nav/approved_cmd",
        ground_truth_navigation=gt_diagnostic,
        SLAM_control_enabled=not gt_diagnostic,
        state_time_semantics="Isaac post-step actual measurement time",
        contacts="native foot spheres; nonfoot contacts reported separately",
    )
    if experiment.get("basement_collision"):
        import omni.physx

        from embodied_navigation.simulation.dynanav.session_collision import (
            preflight_support,
        )

        result["support_preflight"] = preflight_support(
            omni.physx.get_physx_scene_query_interface(),
            config_path=experiment.get("collision_config_path"),
        )
    if (
        pedestrian is not None
        and experiment["pedestrian"].get("scenario") == "landing-crossing"
    ):
        from embodied_navigation.simulation.dynanav.pedestrian import (
            inspect_walk_support,
        )

        result["pedestrian_support"] = inspect_walk_support(
            omni.physx.get_physx_scene_query_interface(), pedestrian
        )
        save(force=True)
        if not result["pedestrian_support"]["passed"]:
            raise RuntimeError("Pedestrian three-point static support preflight failed")
    previous_completed_tau = np.zeros(12)
    save()
    rows = []
    sensor_rows = []
    camera_rows = []
    last_imu_time = None
    last_measurement_time = None
    loop_start = time.perf_counter()
    stop_detected_at = None
    park_since = None
    executed_steps = 0
    odometry_hold_started = None
    source_terminal_reason = None
    result["freshness_hold_events"] = []
    result["stage"] = "physics_running"
    save()
    au.play()

    for i in range(int(os.environ.get("PROBE_STEPS", "4400"))):
        q = art.get_dof_positions().numpy()[0]
        qd = art.get_dof_velocities().numpy()[0]
        if i % 4 == 0:
            for _callback_drain in range(8):
                rclpy.spin_once(node, timeout_sec=0.0)
            p0, r0 = art.get_world_poses()
            v0, w0 = art.get_velocities()
            pos0 = p0.numpy()[0]
            quat0 = r0.numpy()[0]
            R = quaternion_rotation(quat0)
            state = np.zeros(64)
            state[0] = float(SimulationManager.get_simulation_time())
            state[1:4] = pos0
            state[4:8] = quat0
            state[8:11] = R.T @ v0.numpy()[0]
            state[11:14] = R.T @ w0.numpy()[0]
            state[14:26] = q[indices]
            state[26:38] = qd[indices]
            state[38:50] = previous_completed_tau
            state[50:55] = contact_state
            tcontrol = i * 0.005
            if tcontrol >= 2.0 and not stand_gate_done:
                rr = rows[-100:]
                vv = np.array([r["root_velocity_world"][0][0] for r in rr])
                ww = np.array([r["root_velocity_world"][1][0] for r in rr])
                pp = np.array([r["root_pose_wxyz"][0] for r in rr])
                cc = np.array([r["contacts"] for r in rr])
                gate = {
                    "v95": float(np.percentile(np.linalg.norm(vv, axis=1), 95)),
                    "omega95": float(np.percentile(np.linalg.norm(ww, axis=1), 95)),
                    "xy_span_m": float(np.linalg.norm(np.ptp(pp[:, :2], axis=0))),
                    "body_contact_max": int(cc[:, 0].max()),
                    "support_fraction": float(
                        (np.sum(cc[:, 1:] > 0, axis=1) >= 3).mean()
                    ),
                }
                result["stand_gate"] = gate
                save()
                assert (
                    gate["v95"] < 0.02
                    and gate["omega95"] < 0.05
                    and gate["xy_span_m"] < 0.02
                    and gate["body_contact_max"] == 0
                    and gate["support_fraction"] > 0.9
                ), "Stand gate failed; refuse walking"
                stand_gate_done = True
            if robogauge and tcontrol >= 6.0 and not policy_stand_gate_done:
                rr = rows[-200:]
                vv = np.array([r["root_velocity_world"][0][0] for r in rr])
                ww = np.array([r["root_velocity_world"][1][0] for r in rr])
                pp = np.array([r["root_pose_wxyz"][0] for r in rr])
                cc = np.array([r["contacts"] for r in rr])
                gate = {
                    "v95": float(np.percentile(np.linalg.norm(vv, axis=1), 95)),
                    "omega95": float(np.percentile(np.linalg.norm(ww, axis=1), 95)),
                    "xy_span_m": float(np.linalg.norm(np.ptp(pp[:, :2], axis=0))),
                    "body_contact_max": int(cc[:, 0].max()),
                    "support_fraction": float(
                        (np.sum(cc[:, 1:] > 0, axis=1) >= 3).mean()
                    ),
                }
                result["policy_stand_gate"] = gate
                save()
                assert (
                    gate["v95"] < 0.02
                    and gate["omega95"] < 0.05
                    and gate["xy_span_m"] < 0.02
                    and gate["body_contact_max"] == 0
                    and gate["support_fraction"] > 0.9
                ), "RoboGauge policy stand gate failed"
                policy_stand_gate_done = True
            request = np.zeros(3)
            if (
                not gt_diagnostic
                and not slam_control
                and 7 <= tcontrol < float(os.environ.get("NAV_END_SIM", "38"))
                and latest_nav[0] is not None
                and state[0] - latest_nav[0]["time"] < 0.25
                and time.perf_counter() - latest_nav[0]["arrival_wall"] < 0.25
            ):
                request = np.array(latest_nav[0]["request"])
            if dynamic_guard is not None:
                held, hold_receipt = dynamic_guard.decision(
                    now_ns=round(state[0] * 1e9)
                )
                result["dynamic_hold"] = hold_receipt
                journal(
                    "dynamic_hold_control.jsonl",
                    {
                        "physics_s": state[0],
                        "hold": held,
                        "status": hold_receipt["status"],
                        "reason": hold_receipt["reason"],
                    },
                )
            else:
                held = False
            if tracker is not None:
                if held:
                    tracker.invalidate_measurement()
                if slam_control:
                    odom = latest_odom[0] or {}
                    gyro = (
                        locals()
                        .get("reading", {})
                        .get("angular_velocity", [0.0, 0.0, 0.0])
                    )
                    measured_command, control_receipt = tracker.tick(
                        state[0],
                        odom.get("time"),
                        odom.get("position"),
                        odom.get("orientation_wxyz"),
                        odom.get("estimated_velocity"),
                        float(gyro[2]),
                        ready=policy_stand_gate_done and not held,
                        now_wall=time.perf_counter(),
                        arrival_wall=odom.get("arrival_wall"),
                        frame_id=odom.get("frame_id"),
                        child_frame_id=odom.get("child_frame_id"),
                    )
                    request = np.asarray(measured_command)
                    result["slam_route_control"] = control_receipt
                    journal("slam_route_control.jsonl", control_receipt)
                else:
                    yaw = float(np.arctan2(R[1, 0], R[0, 0]))
                    request = np.asarray(
                        tracker.tick(
                            state[0],
                            pos0.tolist(),
                            yaw,
                            float(np.linalg.norm(v0.numpy()[0])),
                            float(w0.numpy()[0][2]),
                            ready=policy_stand_gate_done and not held,
                        )
                    )
                result["route_tracker"] = {
                    "phase": tracker.phase,
                    "stage": tracker.stage,
                    "arrived": tracker.arrived,
                    "failure": tracker.failure,
                    "events": tracker.events,
                }
                if tracker.arrived or tracker.failure:
                    source_terminal_reason = (
                        "route_arrived" if tracker.arrived else tracker.failure
                    )
            assert np.isfinite(request).all() and np.all(
                np.abs(request) <= [target_speed + 1e-6, 0.050001, 0.150001]
            ), "Invalid navigation command"
            if (
                not gt_diagnostic
                and not slam_control
                and 7 <= tcontrol < float(os.environ.get("NAV_END_SIM", "210"))
                and (np.any(request) or odometry_hold_started is not None)
            ):
                for _freshness_drain in range(4):
                    rclpy.spin_once(node, timeout_sec=0.0)
                odom_wall_age = (
                    time.perf_counter() - latest_odom[0]["arrival_wall"]
                    if latest_odom[0]
                    else float("inf")
                )
                odom_sim_age = (
                    state[0] - latest_odom[0]["time"]
                    if latest_odom[0]
                    else float("inf")
                )
                fresh = (
                    latest_odom[0] is not None
                    and odom_sim_age < 0.25
                    and odom_wall_age < 0.5
                )
                result["last_freshness_check"] = {
                    "physics_s": state[0],
                    "odom_s": latest_odom[0]["time"] if latest_odom[0] else None,
                    "wall_age_s": odom_wall_age,
                    "sim_age_s": odom_sim_age,
                    "fresh": fresh,
                }
                save()
                if not fresh:
                    request[:] = 0.0
                    if odometry_hold_started is None:
                        odometry_hold_started = time.perf_counter()
                        result["freshness_hold_events"].append(
                            {
                                "event": "hold_started",
                                "physics_s": state[0],
                                "wall_age_s": odom_wall_age,
                                "sim_age_s": odom_sim_age,
                            }
                        )
                    if (
                        time.perf_counter() - odometry_hold_started > 2.0
                        and source_terminal_reason is None
                    ):
                        source_terminal_reason = "persistent_odometry_staleness"
                        result["source_terminal_reason"] = source_terminal_reason
                elif odometry_hold_started is not None:
                    result["freshness_hold_events"].append(
                        {
                            "event": "freshness_recovered",
                            "physics_s": state[0],
                            "hold_wall_seconds": time.perf_counter()
                            - odometry_hold_started,
                        }
                    )
                    odometry_hold_started = None
            if pathlib.Path(
                os.environ.get("NAV_STOP_REQUEST", "/tmp/no_multistage_stop_request")
            ).exists():
                source_terminal_reason = (
                    source_terminal_reason or "external_stop_request"
                )
            if source_terminal_reason is not None:
                request[:] = 0.0
            if robogauge:
                cmd = request.copy()
                obs = actor.observation(state, cmd, last_action)
                ac = actor.infer(obs)
            else:
                corrected, _ = pid.update(
                    state[0],
                    request,
                    np.array([state[8], state[9], state[13]]),
                    "short_straight",
                    not np.any(request),
                )
                cmd += np.clip(
                    corrected - cmd,
                    -np.array([0.6, 0.6, 0.8]) * 0.02,
                    np.array([0.6, 0.6, 0.8]) * 0.02,
                )
                obs = exact_student45(state, cmd, last_action)
                with torch.inference_mode():
                    action, hidden = actor(torch.from_numpy(obs).reshape(1, 45), hidden)
                assert (
                    tuple(action.shape) == (1, 12)
                    and tuple(hidden.shape) == (1, 1, 256)
                    and torch.isfinite(action).all()
                    and torch.isfinite(hidden).all()
                ), "Policy output invalid"
                ac = action.numpy()[0]
            assert np.isfinite(ac).all()
            tcontrol = i * 0.005
            qc = qdefault + 0.25 * ac if tcontrol >= 0.1 else qdefault
            last_action = ac.copy() if tcontrol >= 0.1 else np.zeros(12)
            target[indices] = qc
            policy_frames.append(
                {
                    "measured_time": state[0],
                    "obs45": obs.tolist(),
                    "action": ac.tolist(),
                    "q_target_canonical": qc.tolist(),
                    "hidden_norm": float(torch.linalg.vector_norm(hidden)),
                    "request": request.tolist(),
                    "navigation_source": latest_odom[0]
                    if slam_control
                    else latest_nav[0],
                    "navigation_pose_source": "measured_lio"
                    if slam_control
                    else "native_truth"
                    if gt_diagnostic
                    else "ROS_approved_command",
                    "actual_command": cmd.tolist(),
                    "native_state64": state.tolist(),
                }
            )
            journal("policy_frames.jsonl", policy_frames[-1])
        pedestrian_target = None
        if pedestrian is not None:
            actor_time = max(
                0.0,
                float(SimulationManager.get_simulation_time())
                - experiment["pedestrian"]["start_delay_s"],
            )
            pedestrian_target = pedestrian.update(actor_time)
        contact_state[:] = [0, 0, 0, 0, 0]
        if robogauge:
            raw, lo, hi, tau = go2hv_torque(target, q, qd)
            from embodied_navigation.simulation.dynanav.experiment import (
                joint_speed_observation,
            )

            speed_readback = joint_speed_observation(
                art.dof_names,
                qd,
                readback_tolerance=experiment.get(
                    "joint_speed_readback_tolerance_radps", 0.0
                ),
            )
            previous_peak = result.get("joint_speed_peak", {}).get(
                "max_abs_radps", -1.0
            )
            if (
                speed_readback["max_abs_radps"] is None
                or speed_readback["max_abs_radps"] > previous_peak
            ):
                result["joint_speed_peak"] = speed_readback
            assert speed_readback["within_guard"], (
                "Joint speed exceeds Go2HV motor envelope and explicit readback tolerance"
            )
        else:
            raw = 25 * (target - q) - 0.5 * qd
            lo = np.maximum(23.5 * (-1 - qd / 30), -23.5)
            hi = np.minimum(23.5 * (1 - qd / 30), 23.5)
            assert np.all(lo <= hi), "DC motor envelope violation"
            tau = np.clip(raw, lo, hi)
        assert np.isfinite(tau).all()
        art.set_dof_efforts(tau[None, :])
        effort_before = art.get_dof_efforts().numpy()[0].copy()
        assert np.allclose(effort_before, tau, atol=1e-5), (
            "Effort readback differs before step"
        )
        time_before = float(SimulationManager.get_simulation_time())
        steps_before = SimulationManager.get_num_physics_steps()
        if (i + 1) % 10:
            SimulationManager.step(steps=1, update_fabric=use_fabric)
            if not use_fabric:
                omni.physx.get_physx_simulation_interface().fetch_results()
                omni.physx.get_physx_interface().update_transformations(
                    False, True, True, False
                )
        else:
            rep.orchestrator.step(
                rt_subframes=1, delta_time=0.005, pause_timeline=False
            )
        actual_time = float(SimulationManager.get_simulation_time())
        effort_after = art.get_dof_efforts().numpy()[0].copy()
        assert np.allclose(effort_after, tau, atol=1e-5), (
            "Effort readback differs after step"
        )
        previous_completed_tau = tau[indices].copy()
        result["last_clock_check"] = {
            "iteration": i,
            "before_s": time_before,
            "after_s": actual_time,
            "step_count_delta": int(
                SimulationManager.get_num_physics_steps() - steps_before
            ),
        }
        save()
        assert (
            abs(actual_time - time_before - 0.005) < 1e-6
            and SimulationManager.get_num_physics_steps() - steps_before == 1
        ), "Physics skipped or duplicate step"
        p, r = art.get_world_poses()
        v, w = art.get_velocities()
        pos = p.numpy()[0]
        quat = r.numpy()[0]
        result["last_state_before_guard"] = {
            "pos": pos.tolist(),
            "quat": quat.tolist(),
            "time": float(SimulationManager.get_simulation_time()),
            "iteration": i,
        }
        save()
        wq, xq, yq, zq = quat
        roll = np.arctan2(2 * (wq * xq + yq * zq), 1 - 2 * (xq * xq + yq * yq))
        pitch = np.arcsin(np.clip(2 * (wq * yq - zq * xq), -1, 1))
        if i >= 300 and contact_state[0]:
            result["body_contact_steps"] = result.get("body_contact_steps", 0) + 1
        assert i < 300 or (
            max(abs(roll), abs(pitch)) < 0.5
            and (
                contact_state[0] == 0
                or experiment.get("body_contact_report_only", False)
            )
        ), "Fall/body-contact gate"
        assert i < 400 or np.linalg.norm(v.numpy()[0]) < physical_speed_guard, (
            "Configured experiment physical overspeed bound"
        )
        assert pos[2] > float(
            experiment.get("minimum_base_z_m", 0.15)
        ) and np.linalg.norm(pos[:2] - spawn[:2]) < float(
            experiment.get(
                "maximum_displacement_m", os.environ.get("MAX_DISPLACEMENT", ".25")
            )
        ), "Position safety bound"
        reading = imu_sensor.get_data()
        it = float(reading["time"])
        result["last_imu_clock_check"] = {
            "imu_s": it,
            "physics_s": actual_time,
            "previous_imu_s": last_imu_time,
        }
        time_tolerance = (
            max(2e-6, 0.5 * float(abs(np.spacing(np.float32(actual_time))))) + 1e-7
        )
        result["last_imu_clock_check"]["quantization_aware_tolerance_s"] = (
            time_tolerance
        )
        save()
        assert abs(it - actual_time) < time_tolerance, (
            "IMU time differs from physical state beyond float32 quantization"
        )
        assert round(it / 0.005) == round(actual_time / 0.005), (
            "IMU tick differs from physics"
        )
        assert (
            last_imu_time is None
            or round(it / 0.005) - round(last_imu_time / 0.005) == 1
        ), "IMU skipped or repeated sample tick"
        last_imu_time = it
        Rb = quaternion_rotation(quat)
        com_local = np.array(result["link_coms_local"][0][0][0])
        sensor_velocity = v.numpy()[0] + np.cross(
            w.numpy()[0], Rb @ (imu_mount - com_local)
        )
        sensor_position = pos + Rb @ imu_mount
        camera_position = pos + Rb @ mount
        sensor_rows.append(
            {
                "time": it,
                "accel_body": np.asarray(reading["linear_acceleration"]).tolist(),
                "gyro_body": np.asarray(reading["angular_velocity"]).tolist(),
                "base_R_world": Rb.tolist(),
                "sensor_velocity_world": sensor_velocity.tolist(),
                "sensor_position_world": sensor_position.tolist(),
            }
        )
        if (i + 1) % experiment.get("imu_publish_stride", 2) == 0:
            clock = Clock()
            ns = round(it * 1e9)
            clock.clock.sec = ns // 1000000000
            clock.clock.nanosec = ns % 1000000000
            pub_clock.publish(clock)
            msg = Imu()
            header(msg, it, "go2_imu")
            aa = reading["linear_acceleration"]
            gg = reading["angular_velocity"]
            qq = reading["orientation"]
            msg.linear_acceleration.x = float(aa[0])
            msg.linear_acceleration.y = float(aa[1])
            msg.linear_acceleration.z = float(aa[2])
            msg.angular_velocity.x = float(gg[0])
            msg.angular_velocity.y = float(gg[1])
            msg.angular_velocity.z = float(gg[2])
            if experiment.get("imu_orientation_available", True):
                msg.orientation.w = float(qq[0])
                msg.orientation.x = float(qq[1])
                msg.orientation.y = float(qq[2])
                msg.orientation.z = float(qq[3])
            else:
                msg.orientation_covariance[0] = -1.0
            pub_imu.publish(msg)
            published["imu"] += 1
        if (i + 1) % 20 == 0:
            zz = np.asarray(depth_ann.get_data())
            rgb = np.asarray(rgb_ann.get_data())[..., :3]
            if rgb_archive is not None:
                rgb_frames_acquired += 1
                rgb_archive.write(
                    rgb,
                    source_ns=round(it * 1e9),
                    frame_index=video_frames,
                    frame_id="go2_camera_optical",
                )
                result["rgb_archive"] = rgb_archive.receipt(
                    acquired_frames=rgb_frames_acquired
                )
            result["rgb_rendered"] = True
            if video_frames in (0, 49):
                from embodied_navigation.simulation.dynanav.render_diagnostics import (
                    capture_stage_receipt,
                )

                (out / f"render_stage_{video_frames:04d}.json").write_text(
                    json.dumps(capture_stage_receipt(stage), indent=2)
                )
            camera_params = camera_params_ann.get_data()
            render_view = np.asarray(camera_params["cameraViewTransform"]).reshape(4, 4)
            render_world = np.linalg.inv(render_view)
            camera_pose_receipt = {
                "physics_s": actual_time,
                "renderer_view_matrix": render_view.tolist(),
                "renderer_camera_world_matrix": render_world.tolist(),
                "renderer_position": render_world[3, :3].tolist(),
                "renderer_forward": (-render_world[2, :3]).tolist(),
                "native_expected_position": camera_position.tolist(),
                "native_expected_forward": (Rb @ sensor_yaw[:, 0]).tolist(),
                "authored_usd_camera_world_matrix": np.asarray(
                    UsdGeom.Xformable(camera.GetPrim()).ComputeLocalToWorldTransform(0)
                ).tolist(),
            }
            result["camera_render_pose"] = camera_pose_receipt
            journal("camera_render_pose.jsonl", camera_pose_receipt)
            packet = depth_cloud_packet(zz, sensor_yaw, mount, imu_mount)
            xyz, keep, self_mask = (
                packet.points,
                packet.depth_keep_mask,
                packet.self_mask,
            )
            result["self_filter"] = {
                "body_box_min": [-0.5, -0.3, -0.45],
                "body_box_max": [0.5, 0.3, 0.3],
                "excluded_points_last": int(self_mask.sum()),
                "coordinates": "actual base axes, known fixed IMU offset",
            }
            result["depth_quality"] = {
                "points": len(xyz),
                "finite_fraction": float(keep.mean()),
                "reference_min_points": 1000,
                "diagnostic_allows_low_coverage": gt_diagnostic,
            }
            if len(xyz) <= 1000:
                result["low_coverage_rgbd_frames"] = (
                    result.get("low_coverage_rgbd_frames", 0) + 1
                )
            assert np.isfinite(xyz).all(), "Nonfinite RGB-D point"
            assert gt_diagnostic or len(xyz) > 1000, "Insufficient valid Go2 depth"
            # Complete obstacle cloud remains visible to navigation, including pedestrians.
            states = None
            if oracle_mapper is not None:
                transform = np.eye(4)
                transform[:3, :3] = Rb
                transform[:3, 3] = sensor_position
                states, frame_receipt = oracle_mapper.process(
                    packet=packet,
                    segmentation=instance_ann.get_data(),
                    source_ns=round(it * 1e9),
                    transform=transform,
                    rgb=rgb,
                    depth=zz,
                )
                result["mapping_frame"] = frame_receipt
                if dynamic_guard is not None:
                    from embodied_navigation.localization_mapping.semantic_mapping import (
                        DYNAMIC,
                    )

                    physics_ns = round(actual_time * 1e9)
                    held, hold_receipt = dynamic_guard.update(
                        xyz[states == DYNAMIC],
                        source_ns=physics_ns,
                        mask_source_ns=physics_ns,
                        now_ns=physics_ns,
                        mask_valid=frame_receipt["mask_valid"]
                        and frame_receipt["unknown"] == 0,
                        sampled_point_count=len(xyz),
                    )
                    hold_receipt["ros_sensor_stamp_ns"] = round(it * 1e9)
                    hold_receipt["clock_binding"] = (
                        "same validated physics tick; ROS IMU stamp retains float32 quantization"
                    )
                    result["dynamic_hold"] = hold_receipt
                    journal("dynamic_hold_frames.jsonl", hold_receipt)
                if pedestrian_view is not None:
                    from isaacsim.core.experimental.utils import (
                        backend as backend_utils,
                    )

                    with backend_utils.use_backend(
                        "tensor", raise_on_unsupported=True, raise_on_fallback=True
                    ):
                        person_position, person_orientation = (
                            pedestrian_view.get_world_poses()
                        )
                    native_person = person_position.numpy()[0].copy()
                    person_receipt = {
                        "physics_s": actual_time,
                        "target": pedestrian_target,
                        "native_position": native_person.tolist(),
                        "native_target_error_m": float(
                            np.linalg.norm(
                                native_person - pedestrian_target["position_m"]
                            )
                        ),
                        "readback_backend": "tensor",
                        "physics_step_delta": result["last_clock_check"][
                            "step_count_delta"
                        ],
                    }
                    result["pedestrian_readback"] = person_receipt
                    journal("pedestrian_native.jsonl", person_receipt)
            xyzi = np.zeros((len(xyz), 4), dtype="<f4")
            xyzi[:, :3] = xyz
            msg = PointCloud2()
            header(msg, it, "go2_imu")
            msg.height = 1
            msg.width = len(xyzi)
            msg.fields = [
                PointField(name=n, offset=j * 4, datatype=7, count=1)
                for j, n in enumerate(["x", "y", "z", "intensity"])
            ]
            msg.point_step = 16
            msg.row_step = 16 * msg.width
            msg.is_dense = True
            msg.data = xyzi.tobytes()
            pub_cloud.publish(msg)
            published["cloud"] += 1
            if states is not None:
                from embodied_navigation.localization_mapping.semantic_mapping import (
                    STATIC,
                )

                static_xyzi = xyzi[states == STATIC]
                static_msg = PointCloud2()
                header(static_msg, it, "go2_imu")
                static_msg.height, static_msg.width = 1, len(static_xyzi)
                static_msg.fields = msg.fields
                static_msg.point_step, static_msg.row_step = 16, 16 * len(static_xyzi)
                static_msg.is_dense = True
                static_msg.data = static_xyzi.tobytes()
                pub_static_cloud.publish(static_msg)
            depth_msg = Image()
            header(depth_msg, it, "go2_camera_optical")
            depth_msg.height, depth_msg.width = zz.shape
            depth_msg.encoding, depth_msg.step = "32FC1", zz.shape[1] * 4
            depth_msg.data = np.ascontiguousarray(zz, dtype="<f4").tobytes()
            pub_depth.publish(depth_msg)
            info = CameraInfo()
            header(info, it, "go2_camera_optical")
            info.height, info.width = zz.shape
            focal = 512 * 24 / 42
            info.k = [focal, 0.0, 256.0, 0.0, focal, 256.0, 0.0, 0.0, 1.0]
            info.p = [
                focal,
                0.0,
                256.0,
                0.0,
                0.0,
                focal,
                256.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ]
            info.distortion_model, info.d = "plumb_bob", [0.0] * 5
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            pub_info.publish(info)
            if (i + 1) in [1000, 1400, 1600] or (i + 1) % 1000 == 0:
                from PIL import Image as PILImage

                PILImage.fromarray(rgb).save(out / f"rgb_{i + 1}.png")
                np.savez_compressed(
                    out / f"depth_{i + 1}.npz",
                    depth=zz,
                    base_R_world=Rb,
                    base_position_world=pos,
                    camera_local_to_world=np.array(
                        UsdGeom.Xformable(
                            camera.GetPrim()
                        ).ComputeLocalToWorldTransform(0)
                    ),
                    imu_mount=imu_mount,
                    mount=mount,
                    imu_points=xyz,
                )
            overview = np.asarray(overview_ann.get_data())[..., :3]
            assert overview.shape == (512, 512, 3), "Invalid overview frame"
            panel = PILImage.fromarray(np.concatenate([overview, rgb], axis=1))
            draw = ImageDraw.Draw(panel)
            draw.rectangle((0, 0, 1024, 24), fill=(0, 0, 0))
            draw.text(
                (8, 6),
                f"Go2 | t={actual_time:.2f}s | target {target_speed:.2f}m/s | {'GT diagnostic' if gt_diagnostic else 'SLAM commands'}",
                fill=(255, 255, 255),
            )
            draw.text((522, 30), "Robot RGB input", fill=(255, 255, 255))
            video_process.stdin.write(np.asarray(panel, dtype=np.uint8).tobytes())
            video_frames += 1
            if video_frames % 10 == 0:
                panel.save(out / "latest_overview.jpg")
            msg = Image()
            header(msg, it, "go2_camera_optical")
            msg.height = 512
            msg.width = 512
            msg.encoding = "rgb8"
            msg.step = 1536
            msg.data = np.ascontiguousarray(rgb).tobytes()
            pub_img.publish(msg)
            published["rgb"] += 1
            camera_rows.append(
                {
                    "time": actual_time,
                    "published_time": it,
                    "valid_depth_points": len(xyz),
                    "finite_depth_fraction": float(keep.mean()),
                    "world_nonground_points_scoring_only": int(
                        (((xyz @ Rb.T + sensor_position)[:, 2]) > 0.12).sum()
                    ),
                }
            )
            journal("camera_frames.jsonl", camera_rows[-1])
        if (i + 1) % 10 == 0:
            target_wall = loop_start + (i + 1) * 0.005 / sim_rtf
            while time.perf_counter() < target_wall:
                rclpy.spin_once(
                    node,
                    timeout_sec=max(0.0, min(0.02, target_wall - time.perf_counter())),
                )
            rclpy.spin_once(node, timeout_sec=0)
        foot_xyz = []  # Runtime foot contact guards retained; optional truth-only rigid-prim queries removed.
        rows.append(
            {
                "foot_centers_world": foot_xyz,
                "contacts": contact_state.copy(),
                "contact_callback_count": callback_counter,
                "time_s": float(SimulationManager.get_simulation_time()),
                "q": q.tolist(),
                "qd": qd.tolist(),
                "tau_cmd": tau.tolist(),
                "root_pose_wxyz": [pos.tolist(), quat.tolist()],
                "root_velocity_world": [v.numpy().tolist(), w.numpy().tolist()],
            }
        )
        journal(
            "truth_scoring_stream.jsonl",
            {
                "time": actual_time,
                "wall_since_loop_start": time.perf_counter() - loop_start,
                "position": pos.tolist(),
                "quat_wxyz": quat.tolist(),
                "velocity": v.numpy()[0].tolist(),
                "omega": w.numpy()[0].tolist(),
                "contacts": contact_state.copy(),
            },
        )
        journal("imu_stream.jsonl", sensor_rows[-1])
        # Full evidence lives in JSONL streams; keep a bounded final receipt.
        del rows[:-500]
        del sensor_rows[:-500]
        del policy_frames[:-500]
        del camera_rows[:-500]
        executed_steps = i + 1
        stop_path = pathlib.Path(
            os.environ.get("NAV_STOP_REQUEST", "/tmp/no_multistage_stop_request")
        )
        if (i + 1) % 20 == 0 and (
            stop_path.exists() or source_terminal_reason is not None
        ):
            if stop_detected_at is None:
                stop_detected_at = actual_time
                result["navigation_terminal_request"] = (
                    json.loads(stop_path.read_text())
                    if stop_path.exists()
                    else {
                        "failure": None
                        if tracker is not None and tracker.arrived
                        else source_terminal_reason,
                        "arrived": bool(tracker is not None and tracker.arrived),
                    }
                )
                save()
            if (
                float(np.linalg.norm(v.numpy()[0])) < 0.02
                and float(np.linalg.norm(w.numpy()[0])) < 0.05
            ):
                if park_since is None:
                    park_since = actual_time
            else:
                park_since = None
            if (
                actual_time - stop_detected_at >= 5.0
                and park_since is not None
                and actual_time - park_since >= 1.0
            ):
                result["end_reason"] = "navigation_terminal_and_parked"
                break
    if oracle_mapper is not None:
        result["mapping"] = oracle_mapper.finish()
    close_journals()
    video_process.stdin.close()
    video_process.wait(timeout=15)
    result["executed_physics_steps"] = executed_steps
    result["video"] = {
        "path": str(out / "multiwaypoint05_overview_and_rgb.mp4"),
        "frames": video_frames,
        "fps": 10,
        "duration_s": video_frames / 10,
        "encoder_exit": video_process.returncode,
        "camera": "body-following third person plus actual RGB input; camera motion visualization only, never navigation input",
    }
    if rgb_archive is not None:
        result["rgb_archive"] = rgb_archive.receipt(
            acquired_frames=rgb_frames_acquired,
            completed=True,
            published_frames=published["rgb"],
            video_frames=video_frames,
        )
    result.update(
        stage="articulation_contact_probe_complete",
        contact_paths=contact_paths,
        contact_events=contact_events,
        dynamic_contact_events=dynamic_contact_events,
        policy_frames=policy_frames,
        rows=rows,
        sensor_rows=sensor_rows,
        camera_rows=camera_rows,
        published=published,
        latest_odom=latest_odom[0],
        loop_wall_seconds=time.perf_counter() - loop_start,
        physics_dt=0.005,
        policy_running=True,
        contact_contract_validated=False,
        com_velocity_convention_validated=False,
        scan_connected=False,
        navigation_running=False,
        navigation_command_source="native_pose_diagnostic"
        if gt_diagnostic
        else "measured_lio_route"
        if slam_control
        else "/dynanav/nav/approved_cmd",
    )
    save(force=True)
    node.destroy_node()
    rclpy.shutdown()
    au.stop()
    _headless_previous_delegates.clear()
    faulthandler.cancel_dump_traceback_later()
    app.close()
except BaseException as e:
    close_journals()
    if rgb_archive is not None:
        result["rgb_archive"] = rgb_archive.receipt(
            acquired_frames=rgb_frames_acquired,
            published_frames=locals().get("published", {}).get("rgb"),
            video_frames=locals().get("video_frames"),
        )
    if "qd" in locals():
        result["actuator_failure_state"] = {
            "q_isaac": q.tolist(),
            "qd_isaac": qd.tolist(),
            "dof_names": art.dof_names,
            "target_isaac": target.tolist(),
            "physics_s": float(SimulationManager.get_simulation_time()),
        }
    if "oracle_mapper" in locals() and oracle_mapper is not None:
        try:
            result["mapping"] = oracle_mapper.finish()
        except Exception as mapping_error:  # noqa: BLE001 - retain the primary simulation failure
            result["mapping_finalize_error"] = repr(mapping_error)
    if hasattr(e, "report"):
        result["failure_report"] = e.report
    if "video_process" in locals():
        try:
            video_process.stdin.close()
            video_process.wait(timeout=10)
        except (OSError, ValueError, subprocess.SubprocessError) as encoder_error:
            result["video_cleanup_error"] = repr(encoder_error)
    result.update(
        rows=locals().get("rows", []),
        sensor_rows=locals().get("sensor_rows", []),
        camera_rows=locals().get("camera_rows", []),
        policy_frames=locals().get("policy_frames", []),
        published=locals().get("published", {}),
        latest_odom=locals().get("latest_odom", [None])[0],
    )
    result["dynamic_contact_events"] = locals().get("dynamic_contact_events", [])
    result.update(error=repr(e), traceback=traceback.format_exc())
    save(force=True)
    print(result["traceback"], flush=True)
    if "app" in locals():
        app.close(exit_code=1)
    raise
