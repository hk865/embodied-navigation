"""CPU-only scheduling contracts, not evidence of real RTX/Fabric behavior."""
import ast
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from embodied_navigation.simulation.dynanav.capture import lockstep


ROOT = Path(__file__).resolve().parents[2]


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class BoundedWaitTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.state = dict(simulation_s=10.2, physics_steps=2040,
                          fabric_s=10.2, timeline_s=10.2,
                          native_state_sha256_audit_only="unchanged")
        self.pumps = []

    def pump(self, remaining):
        self.pumps.append(remaining)
        self.clock.now += .02

    def wait(self, **overrides):
        args = dict(max_wait_s=.15, clock_published_wall_s=100.,
                    snapshot=lambda: dict(self.state), pump=self.pump,
                    ready=lambda now: (now >= 100.06, {"source_stamp_s": 10.}),
                    cancelled=lambda: None, stop_requested=lambda: False,
                    clock=self.clock)
        args.update(overrides)
        return lockstep.bounded_wait(**args)

    def test_lagged_source_can_finish_without_current_stamp_or_physics(self):
        receipt = self.wait()
        self.assertEqual(receipt["status"], "pipeline_hint_ready")
        self.assertEqual(receipt["clock_before"], receipt["clock_after"])
        self.assertEqual(receipt["last_pipeline_hint"]["source_stamp_s"], 10.)
        self.assertGreater(receipt["clock_before"]["simulation_s"], 10.)
        for key in ("physics_steps_added", "policy_ticks_added", "clock_messages_added"):
            self.assertEqual(receipt[key], 0)
        self.assertFalse(receipt["motion_permission"])

    def test_timeout_does_not_extend_clock_receipt_or_wait_for_future_source(self):
        receipt = self.wait(ready=lambda now: (False, {"reason": "not_yet_available"}),
                            clock_published_wall_s=99.9)
        self.assertEqual(receipt["status"], "timeout")
        self.assertAlmostEqual(receipt["deadline_wall_s"], 100.1)
        self.assertEqual(receipt["clock_before"], receipt["clock_after"])

    def test_overdue_clock_or_hold_deadline_never_pumps(self):
        for kwargs in ({"clock_published_wall_s": 99.7}, {"wall_deadline": 99.99}):
            with self.subTest(kwargs=kwargs):
                receipt = self.wait(**kwargs)
                self.assertEqual(receipt["status"], "timeout")
                self.assertEqual(receipt["render_pump_rounds"], 0)

    def test_existing_hold_deadline_shortens_wait_without_reset(self):
        receipt = self.wait(ready=lambda now: (False, {}), wall_deadline=100.04)
        self.assertEqual(receipt["status"], "timeout")
        self.assertEqual(receipt["deadline_wall_s"], 100.04)
        self.assertLess(receipt["render_pump_rounds"], 4)

    def test_blocking_render_overrun_is_timeout_never_ready(self):
        def blocked_pump(_):
            self.clock.now += .6
        receipt = self.wait(pump=blocked_pump)
        self.assertEqual(receipt["status"], "timeout")
        self.assertEqual(receipt["render_pump_rounds"], 1)

    def test_expensive_ready_check_cannot_claim_late_readiness(self):
        def ready(_):
            self.clock.now += .3
            return True, {}
        self.assertEqual(self.wait(ready=ready)["status"], "timeout")

    def test_every_clock_or_state_mutation_is_fatal_no_restamping(self):
        for key in self.state:
            original = dict(self.state)
            def changed(_):
                self.state[key] = "changed"
            with self.subTest(key=key), self.assertRaisesRegex(RuntimeError, "advanced") as error:
                self.wait(pump=changed)
            self.assertIn(key, str(error.exception))
            self.assertIn('"before"', str(error.exception))
            self.assertIn('"after"', str(error.exception))
            self.state = original

    def test_stop_and_cancel_do_not_pump(self):
        self.assertEqual(self.wait(stop_requested=lambda: True)["status"], "stop_or_terminal")
        def cancel():
            raise InterruptedError("user cancellation")
        with self.assertRaises(InterruptedError):
            self.wait(cancelled=cancel)
        self.assertEqual(self.pumps, [])

    def test_invalid_wait_or_clock_rejected(self):
        for value in (0, -.1, .21, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.wait(max_wait_s=value)
        for value in (101., float("nan")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.wait(clock_published_wall_s=value)


class SourceAndProfileTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "perception.json"
        self.tracker = SimpleNamespace(
            perception_path=self.path,
            config=dict(max_sim_age_s=.25, max_wall_age_s=.5, min_clearance_m=.55,
                        pose_loss_timeout_s=5., link_timeout_s=5.),
            _floor_supported=lambda entry, source: bool(entry and entry.get("floor_support_complete")),
            _hold_since_wall=95.1, _last_heartbeat_wall=99.,
            _action={"expires_wall_s": 102.}, _pending=[{"expires_wall_s": 103.}],
        )
        self.payload = dict(source="rtx_depth_safety_fusion_v1", stamp_s=10., arrival_wall_s=100.,
                            fail_closed=False, observation_state="measured", clearance_m=3.,
                            directions={"front": {"floor_support_complete": True}})
        self.odom = dict(time=10., arrival_wall=100.)

    def hint(self, **overrides):
        self.path.write_text(json.dumps(self.payload))
        args = dict(tracker=self.tracker, odom=self.odom, now_sim_s=10.2, now_wall_s=100.1)
        args.update(overrides)
        return lockstep.pipeline_hint(**args)

    def test_native_gate_is_reused_without_mutating_source_or_tracker(self):
        before = copy.deepcopy(self.payload), copy.deepcopy(self.odom), dict(vars(self.tracker))
        # Isolate scheduling from the separately tested full fusion validator.
        with patch("embodied_navigation.simulation.dynanav.depth_safety_fusion.validate_fusion_contract") as validator:
            ready, evidence = self.hint()
        self.assertTrue(ready)
        validator.assert_called_once_with(self.payload, now_sim_s=10.2, now_wall_s=100.1)
        self.assertEqual((self.payload, self.odom, vars(self.tracker)), before)
        self.assertFalse(evidence["motion_permission"])

    def test_old_future_unknown_and_invalid_fusion_remain_not_ready(self):
        for kwargs in ({"now_sim_s": 10.251}, {"now_wall_s": 100.501},
                       {"now_sim_s": 9.9}, {"now_wall_s": 99.9}):
            with self.subTest(kwargs=kwargs):
                self.assertFalse(self.hint(**kwargs)[0])
        # The actual validator rejects the intentionally incomplete fixture.
        self.assertFalse(self.hint()[0])
        with patch("embodied_navigation.simulation.dynanav.depth_safety_fusion.validate_fusion_contract"):
            for field, value in (("fail_closed", True), ("observation_state", "unknown"),
                                 ("clearance_m", None), ("clearance_m", .3)):
                original = copy.deepcopy(self.payload)
                self.payload[field] = value
                self.assertFalse(self.hint()[0])
                self.payload = original
            self.payload["directions"]["front"]["floor_support_complete"] = False
            self.assertFalse(self.hint()[0])

    def test_wall_deadlines_preserved(self):
        original = dict(vars(self.tracker))
        self.assertAlmostEqual(lockstep.original_wall_deadline(self.tracker), 100.1)
        self.assertEqual(vars(self.tracker), original)

    def profile(self):
        return dict(nonrealtime_sensor_lockstep=True, point_cloud_source="rtx_lidar",
                    render_backend="fabric", control_mode="interactive_waypoint_exploration",
                    interactive_simulation_only=True, interactive_perception_owner="rtx_deskewed_cloud",
                    simulated_nearfield_depth=True)

    def test_default_off_and_explicit_nonrealtime_contract(self):
        self.assertIsNone(lockstep.validate_profile({}))
        self.assertIsNone(lockstep.validate_profile({"nonrealtime_sensor_lockstep": False}))
        contract = lockstep.validate_profile(self.profile())
        self.assertFalse(contract["realtime"])
        self.assertFalse(contract["paused_clock_heartbeat"])
        self.assertFalse(contract["safety_thresholds_changed"])
        self.assertEqual(contract["max_wait_wall_s"], .15)

    def test_profile_refuses_dynamic_or_other_modes(self):
        for key, value in (("pedestrian", {"enabled": True}), ("render_backend", "usd"),
                           ("point_cloud_source", "rgbd"), ("simulated_nearfield_depth", False),
                           ("interactive_simulation_only", False), ("nonrealtime_sensor_lockstep", "yes"),
                           ("control_mode", "native_pose_diagnostic"), ("sensor_lockstep_max_wait_s", .21)):
            profile = self.profile()
            profile[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                lockstep.validate_profile(profile)

    def test_opt_in_source_manifest_and_no_extra_physics_control_or_clock_calls(self):
        entry = ROOT / "src/embodied_navigation/simulation/dynanav/capture_go2_rgbd.py"
        tree = ast.parse(entry.read_text())
        pump = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == "pump_lockstep")
        calls = [ast.unparse(node.func) for node in ast.walk(pump) if isinstance(node, ast.Call)]
        for expected in ("RenderingManager.render", "stream.publish_rtx_pending", "stream.process", "ros.spin_once"):
            self.assertIn(expected, calls)
        for forbidden in ("SimulationManager.step", "app.update", "control.tick", "tracker.tick", "ros.publish_imu_and_clock"):
            self.assertNotIn(forbidden, calls)
        experiment = ROOT / "src/embodied_navigation/simulation/dynanav/experiment.py"
        self.assertIn('"src/embodied_navigation/simulation/dynanav/capture/lockstep.py"', experiment.read_text())


if __name__ == "__main__":
    unittest.main()
