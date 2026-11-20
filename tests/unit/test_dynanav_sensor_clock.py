import unittest

import numpy as np

from embodied_navigation.simulation.dynanav.experiment import (
    float32_sensor_time_tolerance,
)


class SensorClockTests(unittest.TestCase):
    def test_full_horizon_float32_rounding_is_covered(self):
        for horizon in [12.0, 100.0, 300.0]:
            times = np.arange(0.015, horizon, 0.1)
            errors = np.abs(times.astype(np.float32).astype(float) - times)
            self.assertLessEqual(
                float(errors.max()), float32_sensor_time_tolerance(horizon)
            )

    def test_measured_microsecond_rounding_is_not_a_future_frame(self):
        tolerance = float32_sensor_time_tolerance(100.0)
        self.assertLess(2.441e-6, tolerance)
        self.assertLess(tolerance, 0.00001)
        self.assertGreater(0.001, tolerance)
