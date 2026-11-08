"""Frame algebra shared by the SLAM adapter and its non-ROS tests."""
import numpy as np
from scipy.spatial.transform import Rotation


def sensor_transform(sensor):
    """Return T_body_sensor from an explicit measured/modelled extrinsic."""
    return (np.asarray(sensor['body_xyz'], dtype=float),
            Rotation.from_euler('xyz', sensor['body_rpy']).as_matrix())


def transform_state(position, quaternion_xyzw, imu, target):
    """T_world_target = T_world_imu * inverse(T_body_imu) * T_body_target."""
    p = np.asarray(position, dtype=float)
    q = np.asarray(quaternion_xyzw, dtype=float)
    if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-9:
        raise ValueError('non-finite pose or zero quaternion')
    p_bi, r_bi = sensor_transform(imu)
    p_bt, r_bt = sensor_transform(target)
    r_wi = Rotation.from_quat(q).as_matrix()
    r_wb = r_wi @ r_bi.T
    return p + r_wb @ (p_bt - p_bi), r_wb @ r_bt


def body_twist(previous, current, dt):
    """Finite-difference pose in current child axes, including offset rotation.

    `previous`/`current` are (position, rotation matrix). An SE(3) target-point
    transform is performed before differentiation; this preserves lever-arm
    velocity that is lost by copying the raw IMU velocity into a body message.
    """
    if not 0.0 < dt <= 1.0:
        raise ValueError('velocity requires increasing timestamps within 1 second')
    p0, r0 = previous
    p1, r1 = current
    linear = r1.T @ ((p1 - p0) / dt)
    angular_world = Rotation.from_matrix(r1 @ r0.T).as_rotvec() / dt
    return linear, r1.T @ angular_world


def shifted_pose_covariance(covariance, world_offset):
    """Propagate XYZ/fixed-axis orientation uncertainty to a shifted point."""
    x,y,z = world_offset
    skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]], dtype=float)
    jacobian = np.eye(6)
    jacobian[:3,3:] = -skew
    cov = np.asarray(covariance, dtype=float).reshape(6,6)
    return (jacobian @ cov @ jacobian.T).ravel().tolist()


def cloud_records(msg):
    """Read ROS PointCloud2 honoring endianness, point padding and row padding.

    Return float XYZ and camera RGB uint8. A cloud without an RGB field is
    rejected, because intensity or height colours are not a camera RGB map.
    """
    fields = {f.name: f for f in msg.fields}
    if not all(n in fields for n in ('x', 'y', 'z')):
        raise ValueError('missing XYZ fields')
    rgb_name = 'rgb' if 'rgb' in fields else 'rgba' if 'rgba' in fields else None
    if rgb_name is None:
        raise ValueError('no camera RGB/RGBA field')
    endian = '>' if msg.is_bigendian else '<'
    for name in ('x', 'y', 'z'):
        if fields[name].datatype != 7 or fields[name].count != 1:
            raise ValueError('XYZ must be scalar float32')
    if fields[rgb_name].datatype not in (6, 7) or fields[rgb_name].count != 1:
        raise ValueError('RGB must be packed uint32 or float32')
    names = ['x', 'y', 'z', 'rgb']
    offsets = [fields[n].offset for n in names[:3]] + [fields[rgb_name].offset]
    if any(o < 0 or o + 4 > msg.point_step for o in offsets):
        raise ValueError('field outside point_step')
    dtype = np.dtype({'names': names, 'formats': [endian+'f4']*3+[endian+'u4'],
                      'offsets': offsets, 'itemsize': msg.point_step})
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height * msg.row_step:
        raise ValueError('truncated or malformed PointCloud2')
    values = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                        strides=(msg.row_step, msg.point_step)).reshape(-1)
    xyz = np.column_stack([values[n] for n in names[:3]]).astype('<f4')
    packed = values['rgb'].astype(np.uint32)
    rgb = np.column_stack(((packed >> 16) & 255, (packed >> 8) & 255, packed & 255)).astype('u1')
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid], rgb[valid]
