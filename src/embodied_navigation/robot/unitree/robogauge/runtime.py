"""Verified CPU TorchScript actor with ten internal history frames.

Only the immutable export is loaded. Training configuration is never evaluated.
"""
from __future__ import annotations
import hashlib
from pathlib import Path
import numpy as np
from embodied_navigation.project import data_root
from .contract import MODEL_SHA256, NATIVE_NAMES, OFFICIAL_NAMES, Q0, joint_indices, observation


def default_model_path():
    return data_root() / 'runtime_assets/models/robogauge_policy.pt'


def verify_model(path):
    path = Path(path)
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    actual = digest.hexdigest()
    if actual != MODEL_SHA256:
        raise ValueError(f'RoboGauge model SHA256 mismatch: {actual}')
    return path


class Runtime:
    def __init__(self, path=None, canonical=NATIVE_NAMES):
        self.path = verify_model(default_model_path() if path is None else path)
        self.canonical = tuple(canonical)
        self.to_official, self.to_native = joint_indices(self.canonical)
        self.qdefault = Q0[self.to_native]
        import torch
        self.torch = torch
        torch.set_num_threads(1)
        self.model = torch.jit.load(str(self.path), map_location='cpu').eval()
        self.model.reset()
        with torch.inference_mode():
            for _ in range(10):
                self.model(torch.zeros(1, 45))
        self.reset()

    def reset(self):
        self.model.reset()
        self.calls = 0

    def observation(self, state, command, last_action):
        return observation(state, command, last_action, canonical=self.canonical)[0]

    def __call__(self, x):
        if tuple(x.shape) != (1, 45) or not self.torch.isfinite(x).all():
            raise ValueError('Expected one finite float observation [1,45]')
        with self.torch.inference_mode():
            y = self.model(x)
        if tuple(y.shape) != (1, 12) or not self.torch.isfinite(y).all():
            raise ValueError('Actor returned invalid action [1,12]')
        self.calls += 1
        return y.clamp(-100., 100.)[:, self.to_native]

    def infer(self, obs):
        obs = np.asarray(obs, dtype=np.float32)
        if obs.shape != (45,):
            raise ValueError('Expected observation45')
        return self(self.torch.from_numpy(obs).reshape(1, 45)).numpy()[0]

    def metadata(self):
        return dict(actor_kind='robogauge_moe_cts', controller_kind='robogauge_moe_cts',
                    model_path=str(self.path), model_sha256=MODEL_SHA256, inference_device='cpu',
                    observation_dimension=45, internal_history_frames=10, external_history=False,
                    policy_joint_order=list(OFFICIAL_NAMES), native_joint_order=list(self.canonical),
                    to_official_indices=self.to_official.tolist(), to_native_indices=self.to_native.tolist(),
                    command_scale=[.5, 1., 1.], angular_velocity_scale=.25, joint_velocity_scale=.05,
                    action_clip=100., action_scale=.25, last_action_unscaled=True,
                    existing_native_COM_velocity_PID=False, terrain_policy_input=False,
                    proprioception_only=True, reset_after_warmup=True, reset_before_run=True,
                    motor_envelope='Go2HV 20.2/23.4 Nm, 13.5/30 rad/s', physics_asset_equivalence=False)
