"""Responsibility-split implementation of the DynaNav RGB-D capture probe.

Importing this package has no side effects. ``capture_go2_rgbd.run`` constructs
the modules below explicitly in config -> session -> ROS -> mapping ->
physics/control loop -> receipts order.
"""

from embodied_navigation.simulation.dynanav.capture.receipts import (
    CaptureContext,
    Receipts,
    record_failure,
)

__all__ = ["CaptureContext", "Receipts", "record_failure"]
