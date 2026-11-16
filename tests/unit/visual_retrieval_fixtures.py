"""Measured visual/lidar correspondences shared by independent acceptance cases."""
from dataclasses import replace

import numpy as np

from embodied_navigation.localization_mapping.backend.keyframes import Keyframe
from embodied_navigation.localization_mapping.backend.visual import VisualObservation


def visual_frame(sequence=0, *, epoch=1, stamp_ns=None, visual=True, seed=104):
    stamp = (sequence+1)*1_000_000_000 if stamp_ns is None else stamp_ns
    rng = np.random.default_rng(seed)
    gray = rng.integers(0,256,(160,160),dtype=np.uint8)
    intrinsic = np.array([[100.,0,80],[0,100,80],[0,0,1]])
    pixels = np.array([(x,y) for y in [25,50,75,100,125] for x in [25,50,75,100,125]])
    depth = rng.uniform(2,4,len(pixels))
    xyz = np.column_stack(((pixels[:,0]-80)*depth/100,(pixels[:,1]-80)*depth/100,depth))
    observation = VisualObservation(gray,intrinsic,np.eye(4),stamp) if visual else None
    return Keyframe((epoch,sequence),stamp,np.eye(4),np.eye(4),xyz,visual=observation)


def dense_visual_frame(sequence=0):
    """Three-dimensional measured returns with enough support for real ICP."""
    frame = visual_frame(sequence)
    rng = np.random.default_rng(231)
    xyz = rng.uniform([-.8, -.8, 2.], [.8, .8, 4.], (1200, 3))
    return replace(frame, points_lidar=xyz)
