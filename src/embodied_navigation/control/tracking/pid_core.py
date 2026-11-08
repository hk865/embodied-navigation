"""Header-event PID of measured SLAM XY/yaw into bounded body velocity only."""
import math
import numpy as np


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def slew(previous, desired, dt, acceleration):
    previous=np.asarray(previous,dtype=float);desired=np.asarray(desired,dtype=float)
    result=previous+np.clip(desired-previous,-np.asarray(acceleration)*max(0.,min(dt,.1)),
                            np.asarray(acceleration)*max(0.,min(dt,.1)))
    if np.linalg.norm(desired[:2])<1e-9:result[:2]=0.
    return result


