"""采集探针的相机、IMU 与渲染产品安装。

本模块负责物理传感器装置本身：相机安装几何与 authored USD 相机、Isaac IMU
传感器、replicator 的 RGB/深度/CameraParams 标注器，以及第三人称 overview
相机与渲染产品。只返回普通值对象（``CameraRig``/``RgbdRender``/
``OverviewRender``）与安装收据，不启动应用、不打开 stage。
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class CameraRig:
    """相机安装几何与诊断用 USD 相机。

    ``theta`` 为绕 base z 轴的相机偏航（rad），``sensor_yaw`` 是其旋转矩阵，
    ``Rcl`` 为相机光学系相对 IMU 系的旋转；``mount`` 为相机在 base 系的位置，
    ``imu_mount`` 取 articulation base 的质心。
    """

    theta: float
    sensor_yaw: np.ndarray
    Rcl: np.ndarray
    base_path: str
    mount: np.ndarray
    imu_mount: np.ndarray
    camera: object


@dataclass
class RgbdRender:
    """机器人 RGB/深度/CameraParams 三路输出的 replicator 图。"""

    rep: object
    product: object
    rgb_ann: object
    depth_ann: object
    camera_params_ann: object


@dataclass
class OverviewRender:
    """第三人称 overview 相机及其 RGB 标注器（仅用于录像/诊断）。"""

    spectator: object
    product: object
    ann: object


def create_camera_rig(stage, experiment):
    """把相机装到真实 articulation base 上，返回其安装几何。

    偏航角优先取 experiment 的 ``camera_yaw_rad``，否则取环境变量
    ``GO2_CAMERA_YAW``，默认 π/2 rad（90°）。安装位置 [0.3, 0, 0.13] 与
    下方焦距/aperture 是硬编码常量，代码与文档未说明其标定来源。
    """
    import os

    from pxr import Gf, UsdGeom, UsdPhysics

    # 传感器装在真实 articulation 的 base 上，不另建独立 rig；偏航角决定相机
    # 相对机身的朝向（默认 90°，与 RGB-D 前视安装一致）。
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
    # base 路径、相机安装偏移（m）与诊断相机名都是硬编码契约；下游按同名路径
    # 查找相机/IMU，改名会同时影响 frames 与 receipts。
    base_path = "/World/Go2/Geometry/base"
    mount = np.array([0.3, 0.0, 0.13])
    imu_mount = np.array(
        UsdPhysics.MassAPI(stage.GetPrimAtPath(base_path)).GetCenterOfMassAttr().Get(),
        dtype=float,
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
    # 焦距 24、水平/垂直 aperture 42 与 depth.py 的内参 focal=512*24/42
    # 同源；裁剪面 0.05..100（场景单位）与深度有效窗 0.05..20 m 不是同一限制。
    camera.GetHorizontalApertureAttr().Set(42.0)
    camera.GetVerticalApertureAttr().Set(42.0)
    camera.GetFocalLengthAttr().Set(24.0)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))
    return CameraRig(theta, sensor_yaw, Rcl, base_path, mount, imu_mount, camera)


def create_imu_sensor(rig):
    """在 articulation base 质心处安装 IMU 传感器。

    三个 filter_size 均为 1，表示不再做额外滑动平均；下游拿到的是单步
    物理量（加速度 m/s²、角速度 rad/s）。
    """
    from isaacsim.sensors.experimental.physics import IMU, IMUSensor

    base_path = rig.base_path
    imu_mount = rig.imu_mount
    imu_sensor = IMUSensor(
        IMU.create(
            base_path + "/DiagnosticIMU",
            translations=[imu_mount.tolist()],
            linear_acceleration_filter_size=1,
            angular_velocity_filter_size=1,
            orientation_filter_size=1,
        )
    )

    return imu_sensor


def create_rgbd_annotators(rig):
    """创建 RGB/深度/CameraParams 渲染产品与标注器。

    渲染分辨率为 512×512；深度使用 ``distance_to_image_plane``（沿光轴
    的米制距离，与 depth.py 的针孔反投影约定对应）。``capture_on_play=False``
    禁止随播放自动抓帧，采集节奏由上层逐步驱动。
    """
    import omni.replicator.core as rep

    camera_path = rig.base_path + "/DiagnosticCamera"
    rep.orchestrator.set_capture_on_play(False)
    product = rep.create.render_product(camera_path, (512, 512))
    rgb_ann = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_ann.attach([product])
    depth_ann = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    depth_ann.attach([product])
    camera_params_ann = rep.AnnotatorRegistry.get_annotator("CameraParams")
    camera_params_ann.attach([product])

    return RgbdRender(
        rep=rep,
        product=product,
        rgb_ann=rgb_ann,
        depth_ann=depth_ann,
        camera_params_ann=camera_params_ann,
    )


def create_overview(rig, stage, rep):
    """创建录像用的第三人称 overview 渲染产品。

    相机的位置/朝向（eye/focus）、焦距 22 与 aperture 35 是硬编码常量，
    只影响录像视角，不参与导航或标定计算；分辨率同为 512×512。
    """
    from pxr import Gf, UsdGeom

    base_path = rig.base_path
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

    return OverviewRender(
        spectator=spectator, product=overview_product, ann=overview_ann
    )


def sensor_installation_receipt(rig, truth_usage, sim_rtf):
    """返回传感器安装收据：安装变换、内参、仿真限制与真值用途。

    收据里的 ``camera_optical_translation_from_IMU`` 由 Rcl 把 base 系下的
    位移旋转到相机光学系；``camera_intrinsics`` 为 [fx, fy, cx, cy]，
    与 depth.py 的 ``focal_px=512*24/42``、``center_px=(256,256)`` 一致。
    ``truth_only_for_scoring=False`` 表明该采集不以真值作为导航输入。
    """
    base_path = rig.base_path
    imu_mount = rig.imu_mount
    Rcl = rig.Rcl
    mount = rig.mount
    theta = rig.theta

    return {
        "parent": base_path,
        "imu_translation_in_base": imu_mount.tolist(),
        "camera_optical_translation_from_IMU": (Rcl @ (imu_mount - mount)).tolist(),
        "camera_translation_in_base": mount.tolist(),
        "camera_optical_R_from_IMU": Rcl.tolist(),
        "custom_sensor_yaw_degrees": float(np.rad2deg(theta)),
        "camera_intrinsics": [512 * 24 / 42, 512 * 24 / 42, 256, 256],
        "modality": "global-shutter RGB-D, not rotating lidar",
        "truth_only_for_scoring": False,
        "truth_usage": truth_usage,
        "max_simulation_real_time_factor": sim_rtf,
        "pacing_changes_physics_dt": False,
    }
