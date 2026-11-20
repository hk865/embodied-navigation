"""Policy loading and action generation for the capture probe.

This module owns the RoboGauge 45D MoE-CTS body policy. ``load_policy`` loads
the RoboGauge runtime, verifies the default joint pose against the articulation
contract, and records ``result["policy"]``. ``PolicyDriver.step`` computes one
50 Hz action and target joint command, maintaining ``hidden``,
``last_action`` and ``cmd`` state. ``compute_torque`` maps target joints to the
Go2HV effort command and enforces the joint-speed readback guard.
"""

import numpy as np


def load_policy(experiment, *, canonical, contract, robogauge=True, result):
    """Load the RoboGauge policy and record its receipt; return the driver."""
    import torch

    if not robogauge:
        raise ValueError("Only the 'robogauge' policy is supported")
    from embodied_navigation.robot.unitree.robogauge.runtime import Runtime

    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    hidden = torch.zeros(1, 1, 256)
    last_action = np.zeros(12)
    qdefault = np.array(contract["default_joint_positions"])
    cmd = np.zeros(3)

    actor = Runtime(canonical=canonical)
    assert np.allclose(qdefault, actor.qdefault), (
        "RoboGauge default joint pose differs"
    )
    result["policy"] = actor.metadata()

    return PolicyDriver(
        experiment=experiment,
        actor=actor,
        torch=torch,
        hidden=hidden,
        last_action=last_action,
        cmd=cmd,
        qdefault=qdefault,
    )


class PolicyDriver:
    """Run the RoboGauge policy, owning hidden state, last action and command."""

    def __init__(
        self,
        *,
        experiment,
        actor,
        torch,
        hidden,
        last_action,
        cmd,
        qdefault,
    ):
        self.experiment = experiment
        self.actor = actor
        self.torch = torch
        self.hidden = hidden
        self.last_action = last_action
        self.cmd = cmd
        self.qdefault = qdefault

    def step(self, i, state, request):
        """Compute one 50 Hz action and the canonical target joint positions."""
        cmd = request.copy()
        obs = self.actor.observation(state, cmd, self.last_action)
        ac = self.actor.infer(obs)
        assert np.isfinite(ac).all()
        tcontrol = i * 0.005
        qc = self.qdefault + 0.25 * ac if tcontrol >= 0.1 else self.qdefault
        self.last_action = ac.copy() if tcontrol >= 0.1 else np.zeros(12)
        self.cmd = cmd
        hidden_norm = float(self.torch.linalg.vector_norm(self.hidden))
        return ac, obs, hidden_norm, qc


def compute_torque(
    *, art, target, q, qd, experiment, robogauge=True, result, read_physics_time_s=None
):
    """Map target joints to the Go2HV effort command and joint-speed guard."""
    if not robogauge:
        raise ValueError("Only the 'robogauge' policy is supported")
    from embodied_navigation.robot.unitree.robogauge.contract import go2hv_torque
    from embodied_navigation.simulation.dynanav.experiment import (
        joint_speed_observation,
    )

    _raw, _lo, _hi, tau = go2hv_torque(target, q, qd)
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
    if speed_readback["nominal_exceedance_radps"] > 0.0:
        physics_s = float(read_physics_time_s()) if read_physics_time_s is not None else None
        exceedances = result.get("joint_speed_nominal_exceedances")
        if exceedances is None:
            exceedances = {
                "samples": 0,
                "first_physics_s": physics_s,
                "peak_abs_radps": speed_readback["max_abs_radps"],
            }
            result["joint_speed_nominal_exceedances"] = exceedances
        exceedances["samples"] += 1
        exceedances["last_physics_s"] = physics_s
        exceedances["peak_abs_radps"] = max(
            exceedances["peak_abs_radps"], speed_readback["max_abs_radps"]
        )
    assert speed_readback["within_guard"], (
        "Joint speed exceeds Go2HV motor envelope and explicit readback tolerance"
    )
    assert np.isfinite(tau).all()
    return tau
