import math
import sys
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from embodied_navigation.robot.unitree.robogauge.contract import NATIVE_NAMES, go2hv_torque
from embodied_navigation.simulation.dynanav.capture.policy import compute_torque
from embodied_navigation.simulation.dynanav.capture.scene import initialize_articulation
from embodied_navigation.simulation.dynanav.experiment import joint_speed_observation


class JointReadbackTests(unittest.TestCase):
    def test_observed_small_solver_readback_margin_without_clamping(self):
        values = [0.0, -30.000137329101562]
        receipt = joint_speed_observation(
            ["hip", "calf"], values, readback_tolerance=0.001
        )
        self.assertTrue(receipt["within_guard"])
        self.assertEqual(receipt["worst_joint"], "calf")
        self.assertEqual(values[1], -30.000137329101562)
        self.assertFalse(
            joint_speed_observation(["hip", "calf"], values)["within_guard"]
        )

    def test_material_overspeed_or_nonfinite_still_rejected(self):
        for value in [30.01, -30.01, math.nan, math.inf]:
            self.assertFalse(
                joint_speed_observation(["calf"], [value], readback_tolerance=0.001)[
                    "within_guard"
                ]
            )

    def test_invalid_parameters_rejected(self):
        for tolerance in [-0.001, math.nan]:
            with self.assertRaises(ValueError):
                joint_speed_observation(["calf"], [0.0], readback_tolerance=tolerance)



class CaptureJointSpeedTests(unittest.TestCase):
    def test_nominal_exceedances_record_current_times_without_changing_torque_or_guard(self):
        result = {}
        read_physics_time_s = Mock(side_effect=[2.005, 2.015, 2.020])
        art = SimpleNamespace(dof_names=NATIVE_NAMES)
        q, target = np.zeros(12), np.full(12, 0.5)

        def torque(speed):
            qd = np.zeros(12)
            qd[3] = speed
            expected = go2hv_torque(target, q, qd)[3]
            command = compute_torque(
                art=art, q=q, qd=qd, target=target,
                experiment={"joint_speed_readback_tolerance_radps": 0.01},
                robogauge=True, result=result, read_physics_time_s=read_physics_time_s,
            )
            np.testing.assert_array_equal(command, expected)
            self.assertEqual(qd[3], speed)

        torque(30.0)
        self.assertNotIn("joint_speed_nominal_exceedances", result)
        read_physics_time_s.assert_not_called()
        torque(-30.007)
        torque(29.0)
        torque(30.002)
        expected = {
            "samples": 2, "first_physics_s": 2.005, "last_physics_s": 2.015,
            "peak_abs_radps": 30.007,
        }
        self.assertEqual(result["joint_speed_nominal_exceedances"], expected)
        with self.assertRaisesRegex(AssertionError, "Joint speed exceeds Go2HV motor envelope"):
            torque(-30.02)
        expected.update(samples=3, last_physics_s=2.020, peak_abs_radps=30.02)
        self.assertEqual(result["joint_speed_nominal_exceedances"], expected)
        self.assertEqual(read_physics_time_s.call_count, 3)

    def test_articulation_records_api_readback_after_setting_joint_limits(self):
        def warp_array(data):
            return SimpleNamespace(numpy=lambda: np.asarray(data))

        art = Mock(dof_names=list(NATIVE_NAMES))
        art.get_world_poses.return_value = [warp_array([[0.0, 0.0, 0.0]])]
        art.get_velocities.return_value = [warp_array([[0.0, 0.0, 0.0]])]
        art.get_link_coms.return_value = [warp_array([[[0.0, 0.0, 0.0]]])]
        art.get_dof_gains.return_value = [warp_array(np.zeros((1, 12)))] * 2
        art.is_physics_tensor_entity_valid.return_value = True
        actual = np.full((1, 12), 30.0)
        actual[0, 3] = 29.5
        art.get_dof_max_velocities.return_value = warp_array(actual)
        prims = ModuleType("isaacsim.core.experimental.prims")
        prims.RigidPrim = Mock()
        result = {}

        with patch.dict(sys.modules, {prims.__name__: prims}), patch(
            "embodied_navigation.simulation.dynanav.collision_bounds.export_bounds"
        ):
            initialize_articulation(art, None, np.zeros(3), 0.0, None, np.zeros(3), result, Mock())

        np.testing.assert_array_equal(
            art.set_dof_max_velocities.call_args.args[0], np.full((1, 12), 30.0)
        )
        art.get_dof_max_velocities.assert_called_once_with()
        calls = [call[0] for call in art.mock_calls]
        self.assertLess(calls.index("set_dof_max_velocities"), calls.index("get_dof_max_velocities"))
        self.assertEqual(result["dof_max_velocities_radps"], {
            "requested": [30.0] * 12, "readback": actual[0].tolist(),
            "physics_tensor_view_valid": True,
        })
