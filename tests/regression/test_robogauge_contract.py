"""CPU numerical fixtures. No simulator, ROS, network or model is required."""

import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from embodied_navigation.robot.unitree.robogauge.contract import (
    NATIVE_NAMES,
    OFFICIAL_NAMES,
    ORDER,
    Q0,
    QDEFAULT,
    joint_indices,
    observation,
    go2hv_torque,
)
from embodied_navigation.robot.unitree.robogauge.runtime import verify_model
from embodied_navigation.robot.unitree.policy.native.observation import (
    quaternion_rotation,
    CONTRACT,
)
from embodied_navigation.navigation.robogauge.commands import admit_command, read_command


class RoboGaugeContract(unittest.TestCase):
    def test_native_order_and_defaults(self):
        self.assertEqual(list(NATIVE_NAMES), CONTRACT["joint_names"])
        np.testing.assert_array_equal(QDEFAULT, CONTRACT["default_joint_positions"])
        np.testing.assert_array_equal(ORDER, [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])
        np.testing.assert_array_equal(ORDER[ORDER], np.arange(12))
        with self.assertRaises(ValueError):
            joint_indices(["bad"] * 12)

    def test_named_frame_and_action_mapping(self):
        rng = np.random.default_rng(8)
        canonical = rng.permutation(NATIVE_NAMES).tolist()
        to_policy, to_native = joint_indices(canonical)
        state = np.zeros(64)
        quat = np.array([0.7, 0.2, -0.3, 0.5])
        quat /= np.linalg.norm(quat)
        state[4:8] = quat
        rot = quaternion_rotation(quat)
        omega = np.array([0.17, -0.23, 0.31])
        state[11:14] = rot.T @ omega
        state[14:26] = rng.normal(size=12)
        state[26:38] = rng.normal(size=12)
        last = rng.normal(size=12)
        cmd = np.array([0.15, 0.02, -0.03])
        obs, _ = observation(state, cmd, last, canonical=canonical)
        self.assertEqual(obs.shape, (45,))
        self.assertEqual(obs.dtype, np.float32)
        np.testing.assert_allclose(obs[:3], (rot.T @ omega) * 0.25, rtol=1e-6)
        np.testing.assert_allclose(obs[3:6], rot.T @ [0, 0, -1], rtol=1e-6)
        np.testing.assert_allclose(obs[6:9], cmd * [0.5, 1, 1], rtol=1e-6)
        for j, name in enumerate(OFFICIAL_NAMES):
            i = canonical.index(name)
            self.assertAlmostEqual(float(obs[9 + j]), state[14 + i] - Q0[j], places=6)
            self.assertAlmostEqual(float(obs[21 + j]), state[26 + i] * 0.05, places=6)
            self.assertAlmostEqual(float(obs[33 + j]), last[i], places=6)
        action_policy = np.arange(12) * 0.37
        target = Q0[to_native] + 0.25 * action_policy[to_native]
        np.testing.assert_allclose(target[to_policy], Q0 + 0.25 * action_policy)

    def test_privileged_state_not_policy_input(self):
        state = np.zeros(64)
        state[4] = 1
        state[14:26] = QDEFAULT
        first, _ = observation(state, np.zeros(3), np.zeros(12))
        state[0:4] = [50, 4, 5, 6]
        state[8:11] = [50, -20, 30]
        state[38:64] = 100
        second, _ = observation(state, np.zeros(3), np.zeros(12))
        np.testing.assert_array_equal(first, second)
        np.testing.assert_array_equal(first[:9], [0, 0, 0, 0, 0, -1, 0, 0, 0])
        with self.assertRaises(ValueError):
            observation(state[:63], np.zeros(3), np.zeros(12))
        state[4] = np.nan
        with self.assertRaises(ValueError):
            observation(state, np.zeros(3), np.zeros(12))

    def test_motor_envelope_and_direction(self):
        qd = np.array([0, 13.5, -13.5, 21.75, -21.75, 30, -30, 35, -35], float)
        for sign in [-1, 1]:
            target = (np.full_like(qd, sign * 100) + 0.5 * qd) / 25
            raw, lo, hi, tau = go2hv_torque(target, np.zeros_like(qd), qd)
            base = np.where(raw * qd > 0, 20.2, 23.4)
            reference = base * np.minimum(1, np.maximum(0, (30 - abs(qd)) / 16.5))
            np.testing.assert_allclose(hi, reference, atol=1e-14)
            np.testing.assert_allclose(lo, -reference, atol=1e-14)
            np.testing.assert_allclose(tau, np.clip(raw, -reference, reference), atol=1e-14)

    def test_command_guard_and_zero_on_invalid(self):
        record = dict(
            run_id="run1", command=[0.2, 0, 0.1], stamp_ns=1_000_000_000, monotonic_wall=10.0
        )
        kw = dict(run_id="run1", source_ns=1_100_000_000, monotonic_now=10.1)
        np.testing.assert_array_equal(admit_command(record, **kw), record["command"])
        for field, value in [
            ("run_id", "other"),
            ("stamp_ns", 0),
            ("monotonic_wall", 9.0),
            ("command", [0.2, 0, 0.31]),
            ("command", [np.nan, 0, 0]),
        ]:
            with self.assertRaises(ValueError):
                admit_command({**record, field: value}, **kw)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "command.json"
            path.write_text("[]")
            cmd, _, error = read_command(path, **kw)
            np.testing.assert_array_equal(cmd, np.zeros(3))
            self.assertIsNotNone(error)
            path.write_text(json.dumps(record))
            cmd, _, error = read_command(path, **kw)
            self.assertIsNone(error)
            np.testing.assert_array_equal(cmd, record["command"])

    def test_bad_weights_rejected_without_importing_torch(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "policy.pt"
            path.write_bytes(b"not the approved policy")
            with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
                verify_model(path)


if __name__ == "__main__":
    unittest.main()
