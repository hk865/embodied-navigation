"""Camera, IMU and render-product installation for the capture probe.

This module owns the physical sensor rig: camera mount math and the authored USD
camera, the Isaac IMU sensor, the replicator RGB/depth/CameraParams annotators,
and the overview camera/render product. It returns plain value objects
(``CameraRig``/``RgbdRender``/``OverviewRender``) plus the sensor installation
receipt; it does not start the app or open the stage.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class CameraRig:
    """Camera mount geometry and the authored diagnostic camera."""

    theta: float
    sensor_yaw: np.ndarray
    Rcl: np.ndarray
    base_path: str
    mount: np.ndarray
    imu_mount: np.ndarray
    camera: object


@dataclass
class RgbdRender:
    """Replicator graph for the robot RGB/depth/CameraParams streams."""

    rep: object
    product: object
    rgb_ann: object
    depth_ann: object
    camera_params_ann: object


@dataclass
class OverviewRender:
    """Third-person overview camera and its RGB annotator."""

    spectator: object
    product: object
    ann: object


def create_camera_rig(stage, experiment):
    """Install the camera on the real articulation base and return its geometry."""
    import os

    from pxr import Gf, UsdGeom, UsdPhysics

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
    return CameraRig(theta, sensor_yaw, Rcl, base_path, mount, imu_mount, camera)


def create_imu_sensor(rig):
    """Install the IMU sensor at the articulation base COM."""
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
    """Create the RGB/depth/CameraParams render products and annotators."""
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
    """Create the third-person overview render product used for the video."""
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
    """Return the sensor installation receipt (mounts, intrinsics, limits)."""
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
