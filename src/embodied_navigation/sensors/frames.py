"""Data exchanged across sensing, localization and control boundaries."""
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class SensorFrame:
    time_s: float
    frame_id: str
    points_xyz: np.ndarray

@dataclass(frozen=True)
class PoseEstimate:
    time_s: float
    frame_id: str
    body_xyz: np.ndarray
    rotation: np.ndarray
    velocity_xyz: np.ndarray

@dataclass(frozen=True)
class VelocityCommand:
    time_s: float
    forward_mps: float
    lateral_mps: float
    yaw_rate_rps: float
