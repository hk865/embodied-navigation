import math
import unittest

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
