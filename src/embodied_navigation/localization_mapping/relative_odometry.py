"""One-time configured alignment of SLAM odometry, preserving gravity.

There is no map registration or truth correction in this relative baseline.
"""

import numpy as np
from scipy.spatial.transform import Rotation
from ..navigation.task import Observation


class RelativeOdometry:
    """Relative SLAM odometry in the configured one-time-aligned navigation frame.

    ``update`` aligns the measured body pose and maps the supplied sensor points
    through the COM offset and body rotation/translation into that same frame.
    Only points inside the configured vertical strip become obstacle points.
    """

    def __init__(self, start_xy, initial_heading_rad, floor_z, com=(0.020153064, 0, -0.0051089553)):
        self.start = np.r_[start_xy, 0.35]
        self.heading = initial_heading_rad
        self.floor_z = floor_z
        self.com = np.asarray(com)
        self.alignment = self.origin = self.previous = self.previous_yaw = None
        self.yaw_unwrapped = 0.0

    def update(self, time_s, wall_s, imu_xyz, quaternion_xyzw, points_xyz):
        rotation = Rotation.from_quat(quaternion_xyzw).as_matrix()
        body_odom = np.asarray(imu_xyz) - rotation @ self.com
        if self.alignment is None:
            initial_yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
            self.alignment = Rotation.from_euler("z", self.heading - initial_yaw).as_matrix()
            self.origin = body_odom.copy()
        body = self.start + self.alignment @ (body_odom - self.origin)
        body_rotation = self.alignment @ rotation
        velocity = (
            np.zeros(3)
            if self.previous is None
            else (body - self.previous[1]) / (time_s - self.previous[0])
        )
        yaw = float(np.arctan2(body_rotation[1, 0], body_rotation[0, 0]))
        self.yaw_unwrapped = (
            yaw
            if self.previous_yaw is None
            else self.yaw_unwrapped
            + float(np.arctan2(np.sin(yaw - self.previous_yaw), np.cos(yaw - self.previous_yaw)))
        )
        self.previous_yaw = yaw
        self.previous = (time_s, body.copy())
        cloud = (np.asarray(points_xyz) - self.com) @ body_rotation.T + body
        obstacles = cloud[(cloud[:, 2] > self.floor_z + 0.12) & (cloud[:, 2] < body[2] + 0.5)]
        return Observation(
            time_s,
            wall_s,
            body,
            body_rotation,
            velocity,
            self.yaw_unwrapped,
            obstacles,
            bool(np.isfinite(points_xyz).all()),
        )
