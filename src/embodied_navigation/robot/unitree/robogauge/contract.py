"""MoE-CTS numerical contract; NumPy only, no simulator or training imports.

Derived from the November 8 Gazebo and Isaac adapters. State64 uses wxyz,
body-frame angular velocity, and the caller's named joint order.
"""
from __future__ import annotations
import numpy as np
from embodied_navigation.robot.unitree.policy.native.observation import quaternion_rotation

MODEL_SHA256 = 'bde5210a8114aa08bb0c611e49def1482faa1b2441673e5681374c30b935e32e'
OFFICIAL_NAMES = tuple(f'{leg}_{joint}_joint' for leg in ('FL', 'FR', 'RL', 'RR') for joint in ('hip', 'thigh', 'calf'))
NATIVE_NAMES = tuple(f'{leg}_{joint}_joint' for leg in ('FR', 'FL', 'RR', 'RL') for joint in ('hip', 'thigh', 'calf'))
Q0 = np.array([.1, .8, -1.5, -.1, .8, -1.5, .1, 1., -1.5, -.1, 1., -1.5])


def joint_indices(canonical=NATIVE_NAMES):
    canonical = tuple(canonical)
    if len(canonical) != 12 or set(canonical) != set(OFFICIAL_NAMES):
        raise ValueError('Expected the twelve unique named Go2 joints')
    to_official = np.array([canonical.index(name) for name in OFFICIAL_NAMES])
    return to_official, np.argsort(to_official)


ORDER, _ = joint_indices()
QDEFAULT = Q0[ORDER]


def observation(state, command, last_action, *unused, canonical=NATIVE_NAMES, **kwargs):
    """Return float32[45] and provenance, preserving the archived scales/order."""
    state = np.asarray(state, dtype=float)
    command = np.asarray(command, dtype=float)
    last_action = np.asarray(last_action, dtype=float)
    if state.shape != (64,) or command.shape != (3,) or last_action.shape != (12,):
        raise ValueError('Expected state64, command3, last_action12')
    if not all(np.isfinite(x).all() for x in (state, command, last_action)):
        raise ValueError('Nonfinite state, command or last action')
    indices, _ = joint_indices(canonical)
    gravity = quaternion_rotation(state[4:8]).T @ np.array([0., 0., -1.])
    obs = np.concatenate((state[11:14] * .25, gravity, command * [.5, 1., 1.],
                          state[14:26][indices] - Q0, state[26:38][indices] * .05,
                          last_action[indices])).astype(np.float32)
    if not np.isfinite(obs).all():
        raise ValueError('Observation overflow')
    return obs, {'terrain_input': False, 'COM_linear_velocity_input': False,
                 'joint_order': 'FL FR RL RR; hip thigh calf'}


def go2hv_torque(target, q, qd):
    """Archived Go2HV joint PD and speed envelope, not a COM velocity PID."""
    target, q, qd = (np.asarray(x, dtype=float) for x in (target, q, qd))
    if target.shape != q.shape or q.shape != qd.shape:
        raise ValueError('Target, q and qd must have equal shapes')
    if not all(np.isfinite(x).all() for x in (target, q, qd)):
        raise ValueError('Nonfinite motor input')
    raw = 25. * (target - q) - .5 * qd
    base = np.where(qd * raw > 0, 20.2, 23.4)
    limit = np.where(abs(qd) < 13.5, base, np.maximum(0., base * (30. - abs(qd)) / 16.5))
    return raw, -limit, limit, np.clip(raw, -limit, limit)
