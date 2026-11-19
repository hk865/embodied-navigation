"""完整姿态与纳秒时间戳记录的数值验收。"""
import unittest
from types import SimpleNamespace as N
import numpy as np
from scipy.spatial.transform import Rotation
from embodied_navigation.localization_mapping.evaluation.recording import odometry_record


class PoseRecordingTest(unittest.TestCase):
    def test_preserves_nanoseconds_quaternion_and_frame(self):
        q = Rotation.from_euler('xyz', [.3, -.4, 3.12]).as_quat()
        pose = N(position=N(x=1., y=2., z=3.), orientation=N(**dict(zip('xyzw', q))))
        message = N(header=N(stamp=N(sec=1794836000, nanosec=987654321), frame_id='odom'),
                    child_frame_id='imu', pose=N(pose=pose, covariance=list(range(36))),
                    twist=N(covariance=list(range(36))))
        row = odometry_record(message)
        self.assertEqual(row['stamp_ns'], 1794836000987654321)
        self.assertEqual(row['frame_id'], 'odom')
        self.assertEqual(row['child_frame_id'], 'imu')
        np.testing.assert_array_equal(row['quaternion_xyzw'], q)
        np.testing.assert_allclose(np.array(row['T_parent_child'])[:3,:3], Rotation.from_quat(q).as_matrix())
        np.testing.assert_allclose(Rotation.from_euler('xyz', row['rpy_xyz_rad']).as_matrix(), Rotation.from_quat(q).as_matrix())
        self.assertEqual(row['pose_covariance'], list(range(36)))
