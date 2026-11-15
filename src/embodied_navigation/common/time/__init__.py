"""Shared sensor-clock quantization helpers."""

import math


def float32_time_half_ulp(time_s):
    """Half the float32 spacing at an already validated sensor/simulation time."""
    return math.ldexp(1.0, math.frexp(float(time_s))[1] - 25)


def float32_sensor_time_tolerance(duration_s):
    """Half a float32 clock ULP at the run horizon plus the existing numeric margin."""
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("A positive finite sensor clock horizon is required")
    return max(2e-6, float32_time_half_ulp(duration_s)) + 1e-7
