"""Policy loading and action generation for the capture probe.

This module owns the two policy families: the legacy Student/COM-velocity-PID
path and the explicit RoboGauge 45D body policy. ``load_policy`` reads the
contract default joints, the deterministic Student JIT model (or the RoboGauge
runtime) and records the unchanged ``result["policy"]`` receipt.
``PolicyDriver.step`` reproduces the original per-control-tick action and
target-joint computation, including the persistent hidden/last-action/command
state. ``compute_torque`` maps the target joints to the unchanged effort command
and, for RoboGauge, the joint-speed readback guard.
"""

import sys

import numpy as np

from embodied_navigation.project import project_root


def load_policy(experiment, *, canonical, contract, robogauge, result):
    """Load the experiment policy and record its receipt; return the driver."""
    import hashlib

    import torch

    sys.path.insert(0, str(project_root() / "src"))
    from embodied_navigation.robot.unitree.policy import speed_tiers
    from embodied_navigation.robot.unitree.policy.student import (
        PID_SPEC,
        CorrectionOnlyPID,
        exact_student45,
    )

    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    hidden = torch.zeros(1, 1, 256)
    last_action = np.zeros(12)
    qdefault = np.array(contract["default_joint_positions"])

    cmd = np.zeros(3)
    if robogauge:
        from embodied_navigation.robot.unitree.robogauge.runtime import Runtime

        actor = Runtime(canonical=canonical)
        assert np.allclose(qdefault, actor.qdefault), (
            "RoboGauge default joint pose differs"
        )
        model_path = actor.path
        result["policy"] = actor.metadata()
        pid = None
    else:
        model_path = project_root() / "models/student_model_4999_deterministic_jit.pt"
        actor = torch.jit.load(str(model_path), map_location="cpu").eval()
        pid = CorrectionOnlyPID(
            {
                "pid": PID_SPEC,
                "command_bounds": [0.3, 0.2, 0.3],
                "command_slew_acceleration": [0.6, 0.6, 0.8],
            },
            [0.6, 0.05, 0.15],
            speed_tier=speed_tiers.choice(0.5),
        )
        result["policy"] = {
            "path": str(model_path),
            "sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
            "device": "CPU",
            "observation": "reused exact_student45",
        }

    return PolicyDriver(
        experiment=experiment,
        robogauge=robogauge,
        actor=actor,
        pid=pid,
        torch=torch,
        exact_student45=exact_student45,
        hidden=hidden,
        last_action=last_action,
        cmd=cmd,
        qdefault=qdefault,
    )


class PolicyDriver:
    """Run the selected policy, owning hidden state, last action and command."""

    def __init__(
        self,
        *,
        experiment,
        robogauge,
        actor,
        pid,
        torch,
        exact_student45,
        hidden,
        last_action,
        cmd,
        qdefault,
    ):
        self.experiment = experiment
        self.robogauge = robogauge
        self.actor = actor
        self.pid = pid
        self.torch = torch
        self.exact_student45 = exact_student45
        self.hidden = hidden
        self.last_action = last_action
        self.cmd = cmd
        self.qdefault = qdefault

    def step(self, i, state, request):
        """Compute one 50 Hz action and the canonical target joint positions."""
        robogauge = self.robogauge
        actor = self.actor
        pid = self.pid
        torch = self.torch
        exact_student45 = self.exact_student45
        cmd = self.cmd
        hidden = self.hidden
        last_action = self.last_action
        qdefault = self.qdefault
        if robogauge:
            cmd = request.copy()
            obs = actor.observation(state, cmd, last_action)
            ac = actor.infer(obs)
        else:
            corrected, _ = pid.update(
                state[0],
                request,
                np.array([state[8], state[9], state[13]]),
                "short_straight",
                not np.any(request),
            )
            cmd += np.clip(
                corrected - cmd,
                -np.array([0.6, 0.6, 0.8]) * 0.02,
                np.array([0.6, 0.6, 0.8]) * 0.02,
            )
            obs = exact_student45(state, cmd, last_action)
            with torch.inference_mode():
                action, hidden = actor(torch.from_numpy(obs).reshape(1, 45), hidden)
            assert (
                tuple(action.shape) == (1, 12)
                and tuple(hidden.shape) == (1, 1, 256)
                and torch.isfinite(action).all()
                and torch.isfinite(hidden).all()
            ), "Policy output invalid"
            ac = action.numpy()[0]
        assert np.isfinite(ac).all()
        tcontrol = i * 0.005
        qc = qdefault + 0.25 * ac if tcontrol >= 0.1 else qdefault
        last_action = ac.copy() if tcontrol >= 0.1 else np.zeros(12)

        self.cmd = cmd
        self.hidden = hidden
        self.last_action = last_action
        hidden_norm = float(torch.linalg.vector_norm(hidden))
        return ac, obs, hidden_norm, qc


def compute_torque(*, art, target, q, qd, experiment, robogauge, result):
    """Map target joints to the unchanged effort command and speed guard."""
    if robogauge:
        from embodied_navigation.robot.unitree.robogauge.contract import (
            go2hv_torque,
        )

        raw, lo, hi, tau = go2hv_torque(target, q, qd)
        from embodied_navigation.simulation.dynanav.experiment import (
            joint_speed_observation,
        )

        speed_readback = joint_speed_observation(
            art.dof_names,
            qd,
            readback_tolerance=experiment.get(
                "joint_speed_readback_tolerance_radps", 0.0
            ),
        )
        previous_peak = result.get("joint_speed_peak", {}).get(
            "max_abs_radps", -1.0
        )
        if (
            speed_readback["max_abs_radps"] is None
            or speed_readback["max_abs_radps"] > previous_peak
        ):
            result["joint_speed_peak"] = speed_readback
        assert speed_readback["within_guard"], (
            "Joint speed exceeds Go2HV motor envelope and explicit readback tolerance"
        )
    else:
        raw = 25 * (target - q) - 0.5 * qd
        lo = np.maximum(23.5 * (-1 - qd / 30), -23.5)
        hi = np.minimum(23.5 * (1 - qd / 30), 23.5)
        assert np.all(lo <= hi), "DC motor envelope violation"
        tau = np.clip(raw, lo, hi)
    assert np.isfinite(tau).all()
    return tau
