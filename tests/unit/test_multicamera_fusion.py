import unittest
from dataclasses import replace
import numpy as np
from scipy.spatial.transform import Rotation
from embodied_navigation.localization_mapping.frontend.multicamera_fusion import (ShadowPacket,covariance_intersection,
    right_log_jacobian_inverse,fuse_shadow_packets)


def packet(camera='front'):
    return ShadowPacket(camera,'generation','actual_native_prior','frozen_map',100,100,100,
        'fixed_calibration_'+camera,np.zeros(6),np.eye(6),'native_internal',True)


class FusionContracts(unittest.TestCase):
    def test_six_identical_correlated_estimates_never_gain_sixfold_information(self):
        mean,covariance=covariance_intersection(np.ones((6,6)),[np.eye(6)*.5]*6)
        np.testing.assert_allclose(mean,1)
        np.testing.assert_allclose(covariance,np.eye(6)*.5)

    def test_complementary_information_is_weighted_without_independence(self):
        _,covariance=covariance_intersection([[0,0],[0,0]],[np.diag([1,100]),np.diag([100,1])])
        np.testing.assert_allclose(covariance,np.eye(2)*2/1.01)

    def test_rotation_covariance_transport_matches_finite_differences(self):
        phi=np.array([.03,-.08,.04]);epsilon=1e-7
        reference=Rotation.from_rotvec(phi)
        numerical=np.column_stack([((reference*Rotation.from_rotvec(np.eye(3)[i]*epsilon)).as_rotvec()-phi)/epsilon for i in range(3)])
        np.testing.assert_allclose(right_log_jacobian_inverse(phi),numerical,atol=2e-8)

    def test_different_epoch_prior_map_generation_and_covariance_sources_rejected(self):
        for change in [dict(source_stamp_ns=101),dict(map_stamp_ns=99),dict(prior_id='other'),
                       dict(map_id='other'),dict(generation='old'),
                       dict(covariance_source='ros_pose_placeholder'),dict(retained_update=False),
                       dict(tangent_convention='ros_position_first'),dict(calibration_id='')]:
            with self.subTest(change=change),self.assertRaises(ValueError):
                fuse_shadow_packets([packet(),replace(packet('rear'),**change)])

        for stamp in (-1,100.5,True,2**63):
            with self.subTest(stamp=stamp),self.assertRaises(ValueError):
                fuse_shadow_packets([replace(packet(),source_stamp_ns=stamp,prior_stamp_ns=stamp,map_stamp_ns=stamp)])

    def test_duplicate_camera_and_invalid_covariance_or_weights_rejected(self):
        with self.assertRaises(ValueError):fuse_shadow_packets([packet(),packet()])
        for matrix in [np.zeros((6,6)),np.eye(6)*np.nan,np.diag([1,1,1,1,1,-1])]:
            with self.assertRaises(ValueError):fuse_shadow_packets([replace(packet(),posterior_pose_covariance=matrix)])
        with self.assertRaises(ValueError):covariance_intersection([[0],[0]],[np.eye(1)]*2,[1,1])

    def test_output_is_shadow_only_and_keeps_original_prior_once(self):
        report=fuse_shadow_packets([packet(),packet('rear')])
        self.assertFalse(report['estimator_feedback_enabled'])
        self.assertFalse(report['confidence_calibrated'])
        np.testing.assert_allclose(report['covariance_in_common_prior_chart'],np.eye(6))


if __name__=='__main__':unittest.main()
