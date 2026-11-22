"""Unit tests for fast PD+I yaw control, preemptive action_epoch queue, and async sentinel LLM/VLM client."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import tempfile
import threading
import time
import unittest
from pathlib import Path

from embodied_navigation.navigation.yaw_action import (
    yaw_action_step,
    yaw_config,
    yaw_request_parameters,
)
from embodied_navigation.planning.exploration.llm_agent_client import (
    AsyncDecisionRunner,
    FastSentinelClient,
    LlmClientConfig,
    PROVIDER_PRESETS,
    SentinelVerdict,
    active_slam_health,
    build_visual_first_messages,
    parse_sentinel_verdict,
)
from embodied_navigation.simulation.dynanav.native_waypoints import NativeWaypointTracker

import sys

_SCRIPTS_DIR = Path(__file__).resolve().parents[2] / "scripts"
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))
import explore_waypoint_client as client  # noqa: E402


class FastYawControllerTests(unittest.TestCase):
    def test_default_config_matches_original_bounds(self):
        cfg = yaw_config({})
        self.assertEqual(cfg["yaw_rate_rps"], 0.15)
        self.assertEqual(cfg["yaw_kp"], 1.2)
        self.assertEqual(cfg["yaw_kd"], 0.0)
        self.assertEqual(cfg["yaw_ki"], 0.0)
        self.assertIsNone(cfg["yaw_max_accel_rps2"])

    def test_profile_knob_validation_bounds(self):
        cfg = yaw_config(
            {
                "interactive_yaw_max_rate_rps": 0.55,
                "interactive_yaw_kp": 2.4,
                "interactive_yaw_kd": 0.35,
                "interactive_yaw_ki": 0.40,
                "interactive_yaw_integral_band_rad": 0.18,
                "interactive_yaw_max_accel_rps2": 2.5,
            }
        )
        self.assertEqual(cfg["yaw_rate_rps"], 0.55)
        self.assertEqual(cfg["yaw_kp"], 2.4)
        self.assertEqual(cfg["yaw_kd"], 0.35)
        self.assertEqual(cfg["yaw_ki"], 0.40)
        self.assertEqual(cfg["yaw_max_accel_rps2"], 2.5)

        for bad_profile in (
            {"interactive_yaw_max_rate_rps": 0.0},
            {"interactive_yaw_max_rate_rps": 0.85},
            {"interactive_yaw_kp": 0.0},
            {"interactive_yaw_kp": 5.5},
            {"interactive_yaw_kd": -0.1},
            {"interactive_yaw_kd": 2.1},
            {"interactive_yaw_ki": -0.1},
            {"interactive_yaw_ki": 2.1},
            {"interactive_yaw_integral_band_rad": 0.0},
            {"interactive_yaw_max_accel_rps2": 0.0},
        ):
            with self.subTest(bad_profile=bad_profile):
                with self.assertRaises(ValueError):
                    yaw_config(bad_profile)

    def test_fast_preset_reaches_90_deg_much_faster_than_legacy(self):
        def simulate_turn(profile):
            cfg = dict(
                yaw_config({**profile, "interactive_yaw_completion_mode": "instant"}),
                max_sim_age_s=0.25,
                max_wall_age_s=0.5,
            )
            action = {"target_yaw_rad": None, "parameters": {"delta_yaw_rad": math.pi / 2}}
            yaw = 0.0
            wz_actual = 0.0
            dt = 0.02
            window = None
            for step in range(800):
                sim = 10.0 + step * dt
                ctx = {
                    "sim": sim,
                    "stamp": sim,
                    "raw_age": 0.0,
                    "wall_age": 0.0,
                    "distinct": True,
                    "measured_yaw_rate_rps": wz_actual,
                    "imu_yaw_rate": wz_actual,
                    "imu_source_stamp_s": sim,
                    "imu_arrival_wall_s": sim,
                    "imu_frame": "go2_imu",
                    "imu_valid": True,
                }
                cmd, window, done = yaw_action_step(
                    action, cfg, ctx, window, yaw=yaw, now_wall=sim, future_tolerance_s=2e-6
                )
                if done is not None:
                    return step * dt
                # First-order lag yaw plant.
                wz_actual += (cmd[2] - wz_actual) * 0.5
                yaw += wz_actual * dt
            return None

        legacy_s = simulate_turn({})
        fast_s = simulate_turn(
            {
                "interactive_yaw_max_rate_rps": 0.55,
                "interactive_yaw_kp": 2.4,
                "interactive_yaw_kd": 0.25,
            }
        )
        self.assertIsNotNone(legacy_s)
        self.assertIsNotNone(fast_s)
        self.assertLess(fast_s, legacy_s * 0.45)

    def test_imu_gyro_damping_reduces_rate_cleanly(self):
        cfg = dict(
            yaw_config(
                {
                    "interactive_yaw_max_rate_rps": 0.55,
                    "interactive_yaw_kp": 2.0,
                    "interactive_yaw_kd": 0.5,
                    "interactive_yaw_completion_mode": "instant",
                }
            ),
            max_sim_age_s=0.25,
            max_wall_age_s=0.5,
        )
        action = {"target_yaw_rad": None, "parameters": {"delta_yaw_rad": 0.1}}
        ctx = {"sim": 10.0, "measured_yaw_rate_rps": 0.4, "imu_valid": True}
        cmd, _, _ = yaw_action_step(
            action, cfg, ctx, None, yaw=0.0, now_wall=10.0, future_tolerance_s=2e-6
        )
        # 2.0 * 0.1 - 0.5 * 0.4 = 0.0 -> zero braking overshoot!
        self.assertAlmostEqual(cmd[2], 0.0, places=12)

    def test_bounded_integral_eliminates_bias_and_resets_on_sign_flip_or_band_exit(self):
        cfg = dict(
            yaw_config(
                {
                    "interactive_yaw_max_rate_rps": 0.55,
                    "interactive_yaw_kp": 1.0,
                    "interactive_yaw_ki": 1.0,
                    "interactive_yaw_integral_band_rad": 0.18,
                    "interactive_yaw_completion_mode": "instant",
                    "interactive_yaw_completion_tolerance_rad": 0.01,
                }
            ),
            max_sim_age_s=0.25,
            max_wall_age_s=0.5,
        )
        action = {"target_yaw_rad": None, "parameters": {"delta_yaw_rad": 0.25}}
        window = None
        # Outside band (target=0.25, yaw=0.0 -> error=0.25): integral stays 0.0.
        cmd_out, window, _ = yaw_action_step(
            action, cfg, {"sim": 10.0}, window, yaw=0.0, now_wall=10.0, future_tolerance_s=2e-6
        )
        self.assertAlmostEqual(cmd_out[2], 0.25, places=12)
        self.assertEqual(action["_control_state"]["integral"], 0.0)

        # Inside band (yaw=0.15 -> error=0.10): accumulates up to clamp |ki * integral| <= 0.08.
        for i in range(1, 20):
            sim = 10.0 + i * 0.1
            cmd_in, window, _ = yaw_action_step(
                action, cfg, {"sim": sim}, window, yaw=0.15, now_wall=sim, future_tolerance_s=2e-6
            )
        self.assertAlmostEqual(action["_control_state"]["integral"], 0.08, places=12)
        self.assertAlmostEqual(cmd_in[2], 0.10 + 0.08, places=12)

        # Sign flip (yaw=0.30 -> error=-0.05): strict anti-windup resets integral to 0.0!
        cmd_flip, window, _ = yaw_action_step(
            action, cfg, {"sim": 12.0}, window, yaw=0.30, now_wall=12.0, future_tolerance_s=2e-6
        )
        self.assertEqual(action["_control_state"]["integral"], 0.0)
        self.assertAlmostEqual(cmd_flip[2], -0.05, places=12)

        # Re-accumulate inside band (2 ticks of +error) then leave band (yaw=-0.05 -> error=0.30 > 0.18): resets to 0.0!
        yaw_action_step(
            action, cfg, {"sim": 12.1}, window, yaw=0.15, now_wall=12.1, future_tolerance_s=2e-6
        )
        yaw_action_step(
            action, cfg, {"sim": 12.2}, window, yaw=0.15, now_wall=12.2, future_tolerance_s=2e-6
        )
        self.assertGreater(action["_control_state"]["integral"], 0.0)
        yaw_action_step(
            action, cfg, {"sim": 12.3}, window, yaw=-0.05, now_wall=12.3, future_tolerance_s=2e-6
        )
        self.assertEqual(action["_control_state"]["integral"], 0.0)

    def test_slew_rate_limiter_caps_acceleration(self):
        cfg = dict(
            yaw_config(
                {
                    "interactive_yaw_max_rate_rps": 0.55,
                    "interactive_yaw_kp": 2.0,
                    "interactive_yaw_max_accel_rps2": 2.0,
                    "interactive_yaw_completion_mode": "instant",
                }
            ),
            max_sim_age_s=0.25,
            max_wall_age_s=0.5,
        )
        action = {"target_yaw_rad": None, "parameters": {"delta_yaw_rad": 1.0}}
        cmd1, w, _ = yaw_action_step(
            action, cfg, {"sim": 10.0}, None, yaw=0.0, now_wall=10.0, future_tolerance_s=2e-6
        )
        self.assertAlmostEqual(cmd1[2], 0.04, places=12)  # 2.0 * 0.02
        cmd2, w, _ = yaw_action_step(
            action, cfg, {"sim": 10.05}, w, yaw=0.0, now_wall=10.05, future_tolerance_s=2e-6
        )
        self.assertAlmostEqual(cmd2[2], 0.14, places=12)  # 0.04 + 2.0 * 0.05

    def test_yaw_request_parameters_and_tracker_honor_profile_rate_ceiling(self):
        with self.assertRaises(ValueError):
            yaw_request_parameters({"delta_yaw_rad": 0.5, "yaw_rate_rps": 0.4})
        params = yaw_request_parameters({"delta_yaw_rad": 0.5, "yaw_rate_rps": 0.4}, max_yaw_rate_rps=0.55)
        self.assertEqual(params["yaw_rate_rps"], 0.4)

        with tempfile.TemporaryDirectory() as tmp:
            mailbox = Path(tmp) / "run" / "mailbox"
            tracker = NativeWaypointTracker(
                {
                    "slam_pose_source": "lio_estimate",
                    "speed_mps": 0.5,
                    "motion_end_s": 200.0,
                    "interactive_mailbox": str(mailbox),
                    "interactive_yaw_max_rate_rps": 0.55,
                }
            )
            self.assertEqual(tracker.parameters["command_bounds"]["yaw_rate_rps"], 0.55)
            tracker._handle_request(
                {
                    "version": 1,
                    "run_id": "run",
                    "seq": 1,
                    "action": "yaw",
                    "pose_source": "lio_estimate",
                    "delta_yaw_rad": 0.5,
                    "yaw_rate_rps": 0.45,
                    "created_wall_s": 10.0,
                    "ttl_s": 30.0,
                },
                now_wall=10.0,
            )
            receipts = [json.loads(line) for line in tracker.receipts_path.read_text().splitlines()]
            self.assertEqual(receipts[-1]["event"], "yaw_accepted")
            self.assertEqual(receipts[-1]["parameters"]["yaw_rate_rps"], 0.45)


class PreemptiveQueueAndEpochTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.mailbox = Path(self.tmp.name) / "run" / "mailbox"
        self.adapter = NativeWaypointTracker(
            {
                "slam_pose_source": "lio_estimate",
                "speed_mps": 0.5,
                "motion_end_s": 200.0,
                "interactive_mailbox": str(self.mailbox),
            }
        )

    def _tick(self, stamp=10.0, pos=None):
        return self.adapter.tick(
            current_sim_s=stamp,
            odom_stamp_s=stamp,
            position=pos or [0.0, 0.0, 0.0],
            quat_wxyz=[1.0, 0.0, 0.0, 0.0],
            estimated_velocity=None,
            measured_yaw_rate=0.0,
            ready=True,
            now_wall=stamp,
            arrival_wall=stamp,
            frame_id="camera_init",
            child_frame_id="body",
            pose_source="lio_estimate",
        )

    def _receipts(self):
        return [json.loads(line) for line in self.adapter.receipts_path.read_text().splitlines()]

    def test_preempt_replaces_active_and_queued_goto_on_same_tick(self):
        self.adapter._handle_request(
            {
                "version": 1,
                "run_id": "run",
                "seq": 1,
                "action": "goto",
                "pose_source": "lio_estimate",
                "goal_xyz_m": [4.0, 0.0, 0.0],
                "action_epoch": 1,
            }
        )
        self._tick(10.0)
        self.assertEqual(self.adapter.active_waypoint["seq"], 1)

        # Queue a second waypoint behind seq=1.
        self.adapter._handle_request(
            {
                "version": 1,
                "run_id": "run",
                "seq": 2,
                "action": "goto",
                "pose_source": "lio_estimate",
                "goal_xyz_m": [6.0, 0.0, 0.0],
                "action_epoch": 1,
            }
        )
        self.assertEqual(len(self.adapter._pending), 1)

        # Write preempting seq=3 via request.json and tick once -> immediately switches!
        (self.mailbox / "request.json").write_text(
            json.dumps(
                {
                    "version": 1,
                    "run_id": "run",
                    "seq": 3,
                    "action": "goto",
                    "pose_source": "lio_estimate",
                    "goal_xyz_m": [0.0, 3.0, 0.0],
                    "preempt": True,
                    "action_epoch": 2,
                }
            ),
            encoding="utf-8",
        )
        _, result = self._tick(10.1)
        self.assertEqual(self.adapter.active_waypoint["seq"], 3)
        self.assertEqual(self.adapter.active_waypoint["goal_xyz_m"], [0.0, 3.0, 0.0])
        self.assertEqual(len(self.adapter._pending), 0)
        self.assertEqual(result["status"], "tracking")

        preempted = [r for r in self._receipts() if r["event"] == "action_preempted"]
        self.assertEqual([r["seq"] for r in preempted], [1, 2])
        self.assertTrue(all(r["preempted_by_seq"] == 3 for r in preempted))
        self.assertTrue(all(r["action_epoch"] == 2 for r in preempted))

        obs = json.loads(self.adapter.observation_path.read_text())
        self.assertEqual(obs["waypoints"]["action_epoch"], 2)
        self.assertEqual(obs["waypoints"]["active_seq"], 3)

    def test_stale_action_epoch_is_rejected_without_overwriting_active(self):
        self.adapter._handle_request(
            {
                "version": 1,
                "run_id": "run",
                "seq": 1,
                "action": "goto",
                "pose_source": "lio_estimate",
                "goal_xyz_m": [3.0, 0.0, 0.0],
                "action_epoch": 5,
            }
        )
        self._tick(10.0)
        self.adapter._handle_request(
            {
                "version": 1,
                "run_id": "run",
                "seq": 2,
                "action": "goto",
                "pose_source": "lio_estimate",
                "goal_xyz_m": [1.0, 1.0, 0.0],
                "preempt": True,
                "action_epoch": 4,
            }
        )
        self.assertEqual(self.adapter.active_waypoint["seq"], 1)
        self.assertEqual(self._receipts()[-1]["event"], "request_rejected")
        self.assertEqual(self._receipts()[-1]["reason"], "stale_action_epoch")

        # Malformed request with higher epoch must not advance _action_epoch.
        self.adapter._handle_request(
            {
                "version": 1,
                "run_id": "run",
                "seq": 3,
                "action": "goto",
                "pose_source": "lio_estimate",
                "goal_xyz_m": [float("nan"), 0.0, 0.0],
                "action_epoch": 99,
            }
        )
        self.assertEqual(self.adapter._action_epoch, 5)

    def test_client_build_request_cli_and_wait_support_preempt_and_epoch(self):
        payload = client.build_request(
            seq=1,
            action="goto",
            goal=[1.0, 2.0, 0.0],
            pose_source="lio_estimate",
            run_id="run",
            created_wall_s=1.0,
            extra={"preempt": True, "action_epoch": 3},
        )
        self.assertTrue(payload["preempt"])
        self.assertEqual(payload["action_epoch"], 3)

        parser = client.build_parser()
        parsed = parser.parse_args(
            [
                "--mailbox",
                str(self.mailbox),
                "goto",
                "--x",
                "1",
                "--y",
                "2",
                "--z",
                "0",
                "--preempt",
                "--action-epoch",
                "7",
            ]
        )
        self.assertTrue(parsed.preempt)
        self.assertEqual(parsed.action_epoch, 7)

        (self.mailbox / "receipts.jsonl").write_text(
            json.dumps(
                {
                    "event": "action_preempted",
                    "seq": 4,
                    "run_id": "run",
                    "preempted_by_seq": 5,
                    "reason": "preempted_by_goto",
                }
            )
            + "\n",
            encoding="utf-8",
        )
        outcome = client.wait_for_action(self.mailbox, 4, timeout_s=0.5, poll_s=0.01)
        self.assertEqual(outcome["status"], "preempted")
        self.assertFalse(outcome["success"])
        self.assertEqual(client.action_exit_code(outcome), client.EXIT_ACTION_FAILED)

    def test_preempt_switches_between_local_action_and_goto_cleanly(self):
        self.adapter._handle_request(
            {
                "version": 1,
                "run_id": "run",
                "seq": 1,
                "action": "maneuver",
                "pose_source": "lio_estimate",
                "body_forward_mps": 0.2,
                "duration_s": 1.0,
                "created_wall_s": 10.0,
                "ttl_s": 10.0,
                "action_epoch": 1,
            },
            now_wall=10.0,
        )
        cmd, _ = self._tick(10.0)
        self.assertAlmostEqual(cmd[0], 0.2)

        # Preempt active maneuver with goto -> emits action_preempted for seq=1 and tracks seq=2.
        self.adapter._handle_request(
            {
                "version": 1,
                "run_id": "run",
                "seq": 2,
                "action": "goto",
                "pose_source": "lio_estimate",
                "goal_xyz_m": [3.0, 0.0, 0.0],
                "preempt": True,
                "action_epoch": 2,
            },
            now_wall=10.1,
        )
        self._tick(10.1)
        self.assertEqual(self.adapter.active_waypoint["seq"], 2)
        self.assertIsNone(self.adapter._action)

        # Preempt active goto with yaw -> emits action_preempted for seq=2.
        self.adapter._handle_request(
            {
                "version": 1,
                "run_id": "run",
                "seq": 3,
                "action": "yaw",
                "pose_source": "lio_estimate",
                "delta_yaw_rad": 0.5,
                "created_wall_s": 10.2,
                "ttl_s": 10.0,
                "preempt": True,
                "action_epoch": 3,
            },
            now_wall=10.2,
        )
        self.assertIsNone(self.adapter.active_waypoint)
        self.assertEqual(self.adapter._action["seq"], 3)
        preempted = [r for r in self._receipts() if r["event"] == "action_preempted"]
        self.assertEqual([(r["seq"], r["preempted_by_seq"]) for r in preempted], [(1, 2), (2, 3)])


class _MockOpenAIServerState:
    def __init__(self) -> None:
        self.requests: list[dict] = []
        self.headers: list[dict[str, str]] = []
        self.response_content: str = '{"act":"CONT","target_id":null,"reason":"clear"}'
        self.delay_s: float = 0.0
        self.lock = threading.Lock()


def _start_mock_server(state: _MockOpenAIServerState) -> tuple[ThreadingHTTPServer, str]:
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length).decode("utf-8")
            with state.lock:
                state.requests.append(json.loads(raw))
                state.headers.append({k.lower(): v for k, v in self.headers.items()})
                delay = state.delay_s
                content = state.response_content
            if delay > 0:
                time.sleep(delay)
            body = json.dumps(
                {"choices": [{"message": {"role": "assistant", "content": content}}]}
            ).encode("utf-8")
            try:
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except BrokenPipeError:
                pass

        def log_message(self, format, *args):
            return

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    host, port = server.server_address
    return server, f"http://{host}:{port}/v1"


class FastSentinelClientAndRunnerTests(unittest.TestCase):
    def test_provider_presets_and_config_validation(self):
        self.assertFalse(PROVIDER_PRESETS["deepseek"]["supports_vision"])
        self.assertTrue(PROVIDER_PRESETS["qwen3.8-flash"]["supports_vision"])
        self.assertTrue(PROVIDER_PRESETS["qwen"]["supports_vision"])
        self.assertEqual(
            LlmClientConfig("qwen").model,
            LlmClientConfig("qwen3.8-flash").model,
        )
        with self.assertRaises(ValueError):
            LlmClientConfig(provider="openai_compatible", base_url="")
        with self.assertRaises(ValueError):
            LlmClientConfig(provider="deepseek", timeout_s=0.0)
        with self.assertRaises(ValueError):
            LlmClientConfig(provider="deepseek", max_tokens=0)

    def test_deepseek_text_only_and_qwen_vision_against_mock_server(self):
        state = _MockOpenAIServerState()
        server, base_url = _start_mock_server(state)
        self.addCleanup(server.shutdown)
        self.addCleanup(server.server_close)

        with tempfile.TemporaryDirectory() as tmp:
            rgb_file = Path(tmp) / "front.png"
            rgb_file.write_bytes(b"\x89PNG\r\n\x1a\nfakeimage")
            obs = {
                "pose": {"position_xyz_m": [1.234, -0.567, 0.41], "yaw_rad": 0.785},
                "waypoints": {"active_seq": 2, "phase": "cruise", "distance_to_goal_m": 2.1},
                "turning_circle": {"safe": True, "min_obstacle_dist_m": 0.92, "obstacle_count": 0},
            }
            candidates = [
                {"id": "F1", "kind": "frontier", "xy": [3.5, 1.2], "dist_m": 2.8},
                {"id": "W1", "kind": "directional", "xy": [2.1, -0.5], "dist_m": 1.1},
            ]
            health = active_slam_health({"stats": {"travel_since_last_loop_m": 5.0, "last_loop_age_s": 20.0}})

            # 1) DeepSeek: supports_vision=False -> pure text even if rgb_path is passed.
            state.response_content = '```json\n{"act":"PREEMPT","target_id":"F1","reason":"shorter frontier"}\n```'
            ds_client = FastSentinelClient(
                LlmClientConfig(provider="deepseek", base_url=base_url, api_key="ds-key")
            )
            ds_verdict = ds_client.evaluate(obs, candidates, health, rgb_path=rgb_file)
            self.assertEqual(ds_verdict.act, "PREEMPT")
            self.assertEqual(ds_verdict.target_id, "F1")
            self.assertFalse(ds_verdict.fallback)

            ds_req = state.requests[-1]
            self.assertEqual(ds_req["model"], "deepseek-chat")
            self.assertIsInstance(ds_req["messages"][1]["content"], str)
            self.assertLess(len(ds_req["messages"][0]["content"]) + len(ds_req["messages"][1]["content"]), 1400)
            self.assertEqual(state.headers[-1]["authorization"], "Bearer ds-key")

            # 2) Qwen3.8-Flash: supports_vision=True -> attaches base64 image_url.
            state.response_content = '{"act":"SPIN","target_id":null,"reason":"need loop"}'
            qw_client = FastSentinelClient(
                LlmClientConfig(provider="qwen3.8-flash", base_url=base_url, api_key="qw-key")
            )
            qw_verdict = qw_client.evaluate(obs, candidates, health, rgb_path=rgb_file)
            self.assertEqual(qw_verdict.act, "SPIN")
            self.assertFalse(qw_verdict.fallback)

            qw_req = state.requests[-1]
            self.assertEqual(qw_req["model"], "qwen3.8-flash")
            content = qw_req["messages"][1]["content"]
            self.assertIsInstance(content, list)
            self.assertEqual(content[0]["type"], "text")
            self.assertEqual(content[1]["type"], "image_url")
            self.assertTrue(content[1]["image_url"]["url"].startswith("data:image/png;base64,"))

    def test_timeout_and_malformed_json_fail_open_to_cont(self):
        v_bad = parse_sentinel_verdict("not valid json at all")
        self.assertEqual(v_bad.act, "CONT")
        self.assertTrue(v_bad.fallback)

        state = _MockOpenAIServerState()
        state.delay_s = 0.25
        server, base_url = _start_mock_server(state)
        self.addCleanup(server.shutdown)
        self.addCleanup(server.server_close)

        client_cfg = LlmClientConfig(
            provider="deepseek", base_url=base_url, api_key="k", timeout_s=0.05
        )
        runner = AsyncDecisionRunner(FastSentinelClient(client_cfg))
        self.addCleanup(runner.shutdown)
        runner.submit({"observation_summary": {}, "candidates": [], "backend_health": {}})

        deadline = time.perf_counter() + 2.0
        verdict = None
        while time.perf_counter() < deadline:
            verdict = runner.poll_latest()
            if verdict is not None:
                break
            time.sleep(0.01)
        self.assertIsNotNone(verdict)
        self.assertEqual(verdict.act, "CONT")
        self.assertTrue(verdict.fallback)
        stats = runner.stats()
        self.assertEqual(stats["calls"], 1)
        self.assertEqual(stats["timeouts"], 1)

    def test_async_runner_latest_wins_drops_stale_frames(self):
        started = threading.Event()
        release = threading.Event()
        seen: list[int] = []

        def slow_eval(payload: dict) -> SentinelVerdict:
            idx = payload["frame"]
            seen.append(idx)
            if idx == 1:
                started.set()
                release.wait(timeout=2.0)
            return SentinelVerdict(act="CONT", reason=f"frame_{idx}", latency_ms=5.0)

        runner = AsyncDecisionRunner(slow_eval)
        self.addCleanup(runner.shutdown)

        self.assertTrue(runner.submit({"frame": 1}))
        self.assertTrue(started.wait(timeout=1.0))

        # Submit frames 2, 3, 4 while frame 1 is still executing: 2 and 3 must be dropped!
        runner.submit({"frame": 2})
        runner.submit({"frame": 3})
        runner.submit({"frame": 4})
        release.set()

        deadline = time.perf_counter() + 2.0
        while runner.calls < 2 and time.perf_counter() < deadline:
            time.sleep(0.01)

        latest = runner.poll_latest()
        self.assertIsNotNone(latest)
        self.assertEqual(latest.reason, "frame_4")
        self.assertIsNone(runner.poll_latest())
        self.assertEqual(seen, [1, 4])
        stats = runner.stats()
        self.assertEqual(stats["calls"], 2)
        self.assertEqual(stats["dropped_stale"], 2)
        self.assertGreater(stats["p50_ms"], 0.0)
        self.assertGreater(stats["p95_ms"], 0.0)

    def test_active_slam_health_thresholds(self):
        ok = active_slam_health({"stats": {"travel_since_last_loop_m": 19.5, "last_loop_age_s": 100.0}})
        self.assertFalse(ok["needs_active_loop"])
        self.assertEqual(ok["recommended_action"], "none")

        over_travel = active_slam_health({"travel_since_last_loop_m": 25.0, "last_loop_age_s": 10.0})
        self.assertTrue(over_travel["needs_active_loop"])
        self.assertEqual(over_travel["recommended_action"], "active_relocalize_spin")

        over_age = active_slam_health({"stats": {"travel_since_last_loop_m": 2.0, "last_loop_age_s": 200.0}})
        self.assertTrue(over_age["needs_active_loop"])
        self.assertEqual(over_age["recommended_action"], "active_relocalize_spin")


if __name__ == "__main__":
    unittest.main()
