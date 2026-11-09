"""Data exchanged across sensing, localization and control boundaries."""

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class SensorFrame:
    """One measured point cloud in its sensor frame at ``time_s`` (seconds)."""

    time_s: float
    frame_id: str
    points_xyz: np.ndarray


@dataclass(frozen=True)
class PoseEstimate:
    """Body pose/rotation/velocity in ``frame_id`` at ``time_s`` (SI units)."""

    time_s: float
    frame_id: str
    body_xyz: np.ndarray
    rotation: np.ndarray
    velocity_xyz: np.ndarray


@dataclass(frozen=True)
class VelocityCommand:
    """Body-frame velocity command (m/s forward/lateral, rad/s yaw rate)."""

    time_s: float
    forward_mps: float
    lateral_mps: float
    yaw_rate_rps: float
