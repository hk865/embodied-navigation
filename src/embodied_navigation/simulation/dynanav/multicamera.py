"""Calibrated multi-view observation of ONE measured LIO map, never a pose fusion.

T_AB maps B into A. Cameras are ideal simulated pinholes on the base. Map XYZ
already lives in the estimator map frame; apply its inverse body pose once.
No angular cloud partition, scene geometry, renderer world pose, or oracle map.
"""
from __future__ import annotations
import numpy as np

from embodied_navigation.common.geometry import quaternion_matrix

RESOLUTION = (512, 512)
FOCAL_MM, APERTURE_MM = 24., 42.


def focal_length_for_hfov(horizontal_fov_deg=None):
    """USD focal/aperture units share a scale; retain the original exact ratio by default."""
    if horizontal_fov_deg is None:
        return FOCAL_MM
    angle = float(horizontal_fov_deg)
    if not np.isfinite(angle) or not 0. < angle < 180.:
        raise ValueError('Pinhole horizontal FOV must be finite and within 0..180 degrees')
    return float(APERTURE_MM / (2 * np.tan(np.deg2rad(angle / 2))))


def camera_calibration(index, imu_mount, *, primary_yaw=0., primary_mount=(.3, 0., .13), camera_count=6,
                       horizontal_fov_deg=None):
    if type(camera_count) is not int or camera_count not in range(3, 7):
        raise ValueError('Multi-view camera count must be an integer within 3..6')
    if index not in range(camera_count):
        raise ValueError('Multi-view index outside selected camera count')
    yaw = float(primary_yaw) + index * 2 * np.pi / camera_count
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    # Optical right/down/forward expressed in base coordinates.
    R_BC = rotation @ np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    mount = np.asarray(primary_mount, float) if index == 0 else rotation @ np.array([.3, 0., .13])
    imu_mount = np.asarray(imu_mount, float)
    if mount.shape != (3,) or imu_mount.shape != (3,) or not np.isfinite(np.r_[mount, imu_mount]).all():
        raise ValueError('Invalid fixed sensor mount')
    focal = focal_length_for_hfov(horizontal_fov_deg)
    f = RESOLUTION[0] * focal / APERTURE_MM
    return dict(camera_id=f'camera_{index}', camera_count=camera_count, yaw_step_deg=360./camera_count,
                yaw_base_rad=yaw, resolution_wh=list(RESOLUTION),
                intrinsics_fx_fy_cx_cy=[f, f, 256., 256.],
                R_camera_from_imu=R_BC.T.tolist(),
                translation_camera_from_imu_m=(R_BC.T @ (imu_mount - mount)).tolist(),
                R_base_from_optical=R_BC.tolist(), translation_base_m=mount.tolist(),
                imu_translation_base_m=imu_mount.tolist(), optical_frame=f'go2_multiview_{index}_optical',
                horizontal_fov_deg=float(np.rad2deg(2 * np.arctan(APERTURE_MM / (2 * focal)))),
                authored_focal_length=focal, authored_aperture=[APERTURE_MM, APERTURE_MM],
                transform_convention='p_camera=R_camera_from_imu @ p_imu + translation_camera_from_imu_m',
                simulated_hardware=True, sensor_model='ideal global shutter pinhole RGB and axial depth',
                min_depth_m=.05, max_depth_m=100.,
                calibration_source='fixed authored robot mounts and native intrinsics validation, not renderer world pose')


def validate_intrinsics(parameters, calibration=None):
    focal = FOCAL_MM if calibration is None else calibration['authored_focal_length']
    for key, expected in dict(renderProductResolution=RESOLUTION, cameraAperture=(APERTURE_MM, APERTURE_MM),
                              cameraFocalLength=focal, cameraApertureOffset=(0., 0.), metersPerSceneUnit=1.).items():
        actual = np.asarray(parameters.get(key, np.nan), float)
        expected = np.asarray(expected)
        if actual.shape != expected.shape or not np.allclose(actual, expected, atol=1e-6, rtol=1e-6):
            raise ValueError('Multi-view native calibration mismatch: ' + key)
    if parameters.get('cameraModel') != 'pinhole':
        raise ValueError('Multi-view requires the authored pinhole model')


def quaternion_rotation(xyzw):
    q = np.asarray(xyzw, float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1.) > 1e-3:
        raise ValueError('Expected a finite unit estimator quaternion in xyzw order')
    x, y, z, w = q / np.linalg.norm(q)
    return quaternion_matrix(w, x, y, z)


def project_shared_map(points_map, pose_xyz_xyzw, calibration, depth, *, depth_tolerance_m=.15):
    """Return original point indices for this view, permitting cross-view overlap.

    Frustum + nearest occupied pixel + same-frame measured axial-depth support.
    Missing/nonfinite depth is UNKNOWN, not visibility. Sparse depth and finite
    tolerance can reject valid edges or admit thin occluders: diagnostic only.
    """
    points = np.asarray(points_map, float)
    pose = np.asarray(pose_xyz_xyzw, float)
    depth = np.asarray(depth, float)
    if points.ndim != 2 or points.shape[1] != 3 or pose.shape != (7,) or not np.isfinite(pose).all():
        raise ValueError('Expected measured Nx3 map points and xyz+xyzw IMU pose')
    width, height = calibration['resolution_wh']
    if depth.shape != (height, width) or not np.isfinite(depth_tolerance_m) or depth_tolerance_m <= 0:
        raise ValueError('Expected paired axial depth and positive finite depth tolerance')
    R_WI = quaternion_rotation(pose[3:])
    R_CI = np.asarray(calibration['R_camera_from_imu'], float)
    t_CI = np.asarray(calibration['translation_camera_from_imu_m'], float)
    if R_CI.shape != (3, 3) or t_CI.shape != (3,) or not np.isfinite(R_CI).all() or not np.isfinite(t_CI).all() or not np.allclose(R_CI @ R_CI.T, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(R_CI), 1., atol=1e-6):
        raise ValueError('Invalid rigid camera calibration')
    camera = ((points - pose[:3]) @ R_WI) @ R_CI.T + t_CI
    fx, fy, cx, cy = calibration['intrinsics_fx_fy_cx_cy']
    finite = np.isfinite(camera).all(axis=1)
    z = camera[:, 2]
    candidate = finite & (z > calibration['min_depth_m']) & (z < calibration['max_depth_m'])
    indices = np.flatnonzero(candidate)
    uv = np.column_stack((fx * camera[indices, 0] / z[indices] + cx,
                          fy * camera[indices, 1] / z[indices] + cy))
    # Integer pixel centres and nearest-neighbour axial depth, no bilinear blend
    # across a foreground/background discontinuity.
    pix = np.floor(uv + .5).astype(np.int64)
    inside = (pix[:, 0] >= 0) & (pix[:, 0] < width) & (pix[:, 1] >= 0) & (pix[:, 1] < height)
    indices, pix, uv = indices[inside], pix[inside], uv[inside]
    frustum_count = len(indices)
    pixel = pix[:, 1] * width + pix[:, 0]
    zbuffer = np.full(width * height, np.inf)
    np.minimum.at(zbuffer, pixel, z[indices])
    observed = depth[pix[:, 1], pix[:, 0]]
    known = np.isfinite(observed) & (observed > calibration['min_depth_m']) & (observed < calibration['max_depth_m'])
    front = z[indices] <= zbuffer[pixel] + depth_tolerance_m
    supported = known & front & (abs(z[indices] - observed) <= depth_tolerance_m)
    return dict(point_indices=indices[supported], pixels_uv=uv[supported], depths_m=z[indices[supported]],
                counts=dict(input_points=len(points), frustum_points=frustum_count,
                            missing_depth=int((~known).sum()), depth_or_zbuffer_rejected=int((known & ~supported).sum()),
                            depth_supported_points=int(supported.sum())),
                semantics='per-camera visible subset of the same input cloud; overlapping indices allowed',
                tolerance_m=depth_tolerance_m, pose_used='measured LIO IMU pose, not simulator truth',
                depth_role='visibility/occlusion only, never inserted into LIO map, PCT or estimator',
                visibility_certified=False)


def nearest_pose(stamps_ns, poses, stamp_ns, *, maximum_delta_ns=30_000_000):
    stamps = np.asarray(stamps_ns)
    poses = np.asarray(poses, float)
    if stamps.dtype.kind not in 'iu' or stamps.ndim != 1 or not len(stamps) or poses.shape != (len(stamps), 7):
        raise ValueError('Missing measured timestamped xyz+xyzw LIO poses')
    if maximum_delta_ns < 0:
        raise ValueError('Negative pose time tolerance')
    # Subtraction on uint64 wraps for poses before the camera epoch. Convert
    # bounded nonnegative epochs to signed integers before measuring distance.
    limit = np.iinfo(np.int64).max
    if np.any(stamps < 0) or np.any(stamps > limit) or not 0 <= int(stamp_ns) <= limit:
        raise ValueError('Pose/camera epochs must be nonnegative int64 nanoseconds')
    stamps = stamps.astype(np.int64)
    index = int(np.argmin(abs(stamps - int(stamp_ns))))
    delta = int(stamps[index]) - int(stamp_ns)
    if abs(delta) > maximum_delta_ns:
        raise ValueError(f'No LIO pose within {maximum_delta_ns} ns of camera source stamp; nearest delta {delta} ns')
    return poses[index], dict(pose_stamp_ns=int(stamps[index]), camera_stamp_ns=int(stamp_ns),
                              signed_pose_camera_delta_ns=delta, interpolation=False,
                              source_timestamps_rewritten=False, maximum_delta_ns=maximum_delta_ns)
