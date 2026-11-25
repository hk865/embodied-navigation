"""Comprehensive regression & acceptance tests for Codex gpt-6.2-astra audit fixes."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from unittest.mock import patch
import unittest

import numpy as np

from embodied_navigation.localization_mapping.backend.backend import (
    BackendConfig,
    Keyframe,
    MapStore,
    SlamBackend,
    descriptor,
)
from embodied_navigation.navigation.yaw_action import (
    yaw_action_step,
    yaw_config,
)
from embodied_navigation.planning.exploration import waypoint_mailbox
from embodied_navigation.planning.exploration.llm_agent_client import (
    ActiveSlamSpinTracker,
    AsyncDecisionRunner,
    FastSentinelClient,
    LlmClientConfig,
    SentinelVerdict,
    active_slam_health,
    build_visual_first_messages,
    parse_sentinel_verdict,
)
from embodied_navigation.planning.exploration.sentinel_coordinator import (
    SentinelCoordinator,
)
from embodied_navigation.planning.pct.measured_tomogram import MeasuredPctPlanner
from embodied_navigation.simulation.dynanav.capture.control import (
    ControlLoop,
    annotate_policy_receipt,
)
from embodied_navigation.simulation.dynanav.native_waypoints import NativeWaypointTracker
from embodied_navigation.simulation.dynanav.pct_scan_worker import PctPlanCoordinator

_SCRIPTS_DIR = Path(__file__).resolve().parents[2] / "scripts"
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))
import explore_waypoint_client as client  # noqa: E402


class _StreamingMockState:
    def __init__(self) -> None:
        self.requests: list[dict] = []
        self.mode: str = "sse_split_hang"
        self.sse_chunks: list[str] = ['{"act":"STOP",', '"reason":"blocked"}']
        self.heartbeat_interval_s: float = 0.02
        self.hang_event = threading.Event()
        self.request_seen = threading.Event()
        self.lock = threading.Lock()


def _start_streaming_server(state: _StreamingMockState) -> tuple[ThreadingHTTPServer, str]:
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def do_POST(self):
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length).decode("utf-8")
            with state.lock:
                state.requests.append(json.loads(raw))
                mode = state.mode
                chunks = list(state.sse_chunks)
                hb_s = state.heartbeat_interval_s
            state.request_seen.set()

            try:
                if mode == "sse_split_hang":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/event-stream")
                    self.send_header("Cache-Control", "no-cache")
                    self.end_headers()
                    for part in chunks:
                        payload = json.dumps(
                            {"choices": [{"delta": {"content": part}}]}
                        ).encode("utf-8")
                        self.wfile.write(b"data: " + payload + b"\n\n")
                        self.wfile.flush()
                        time.sleep(0.01)
                    state.hang_event.wait(timeout=5.0)
                elif mode == "sse_heartbeat_forever":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/event-stream")
                    self.end_headers()
                    while not state.hang_event.is_set():
                        self.wfile.write(b": heartbeat\n\n")
                        self.wfile.flush()
                        time.sleep(hb_s)
                elif mode == "block_headers":
                    state.hang_event.wait(timeout=5.0)
                    body = b'{"choices":[{"message":{"content":"{\\"act\\":\\"STOP\\",\\"reason\\":\\"late\\"}"}}]}'
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                else:
                    body = json.dumps(
                        {"choices": [{"message": {"content": "".join(chunks)}}]}
                    ).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

        def log_message(self, format, *args):
            return

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    host, port = server.server_address
    return server, f"http://{host}:{port}/v1"


class StreamingAndAsyncConcurrencyTests(unittest.TestCase):
    def test_sse_split_chunks_early_stop_before_eof(self):
        state = _StreamingMockState()
        state.mode = "sse_split_hang"
        state.sse_chunks = ['{"act":', '"STOP",', '"reason":"blocked"}']
        server, base_url = _start_streaming_server(state)
        self.addCleanup(state.hang_event.set)
        self.addCleanup(server.shutdown)
        self.addCleanup(server.server_close)

        cfg = LlmClientConfig(
            provider="deepseek", base_url=base_url, api_key="k-test", timeout_s=1.5, stream=True
        )
        client_obj = FastSentinelClient(cfg)
        t0 = time.perf_counter()
        verdict = client_obj.evaluate(
            {"pose": {"position_xyz_m": [0.0, 0.0, 0.0], "yaw_rad": 0.0}},
            [],
            {},
        )
        elapsed = time.perf_counter() - t0
        self.assertEqual(verdict.act, "STOP")
        self.assertEqual(verdict.reason, "blocked")
        self.assertFalse(verdict.fallback)
        self.assertLess(elapsed, 0.6)

    def test_slow_heartbeat_stream_enforces_monotonic_total_deadline(self):
        state = _StreamingMockState()
        state.mode = "sse_heartbeat_forever"
        state.heartbeat_interval_s = 0.015
        server, base_url = _start_streaming_server(state)
        self.addCleanup(state.hang_event.set)
        self.addCleanup(server.shutdown)
        self.addCleanup(server.server_close)

        cfg = LlmClientConfig(
            provider="deepseek", base_url=base_url, api_key="k-test", timeout_s=0.06
        )
        client_obj = FastSentinelClient(cfg)
        t0 = time.perf_counter()
        verdict = client_obj.evaluate({}, [], {})
        elapsed = time.perf_counter() - t0
        self.assertEqual(verdict.act, "CONT")
        self.assertTrue(verdict.fallback)
        self.assertIn("timeout", verdict.reason.lower())
        self.assertLess(elapsed, 0.30)

    def test_async_runner_never_publishes_stale_frame_verdict(self):
        f1_started = threading.Event()
        f1_release = threading.Event()
        f2_started = threading.Event()
        f2_release = threading.Event()

        def eval_fn(payload: dict) -> SentinelVerdict:
            idx = payload["frame"]
            if idx == 1:
                f1_started.set()
                f1_release.wait(timeout=2.0)
                return SentinelVerdict(act="STOP", reason="stale_stop", latency_ms=2.0)
            f2_started.set()
            f2_release.wait(timeout=2.0)
            return SentinelVerdict(act="CONT", reason="fresh_cont", latency_ms=2.0)

        runner = AsyncDecisionRunner(eval_fn)
        self.addCleanup(f1_release.set)
        self.addCleanup(f2_release.set)
        self.addCleanup(runner.shutdown)

        self.assertTrue(runner.submit({"frame": 1}, frame_seq=1, action_epoch=1))
        self.assertTrue(f1_started.wait(timeout=1.0))

        # Frame 2 submitted while frame 1 is still running; frame 2 blocks.
        self.assertTrue(runner.submit({"frame": 2}, frame_seq=2, action_epoch=1))
        f1_release.set()
        self.assertTrue(f2_started.wait(timeout=1.0))

        # Frame 1 finished with STOP, yet poll_latest MUST NOT return stale frame 1's STOP!
        self.assertIsNone(runner.poll_latest())

        f2_release.set()
        deadline = time.perf_counter() + 1.5
        latest = None
        while time.perf_counter() < deadline:
            latest = runner.poll_latest()
            if latest is not None:
                break
            time.sleep(0.01)
        self.assertIsNotNone(latest)
        self.assertEqual(latest.act, "CONT")
        self.assertEqual(latest.frame_seq, 2)
        self.assertEqual(latest.action_epoch, 1)

    def test_async_runner_shutdown_aborts_in_flight_http_and_never_publishes(self):
        state = _StreamingMockState()
        state.mode = "block_headers"
        server, base_url = _start_streaming_server(state)
        self.addCleanup(state.hang_event.set)
        self.addCleanup(server.shutdown)
        self.addCleanup(server.server_close)

        cfg = LlmClientConfig(
            provider="deepseek", base_url=base_url, api_key="k-test", timeout_s=5.0
        )
        runner = AsyncDecisionRunner(FastSentinelClient(cfg))
        self.assertTrue(runner.submit({"observation_summary": {}, "candidates": [], "backend_health": {}}, frame_seq=1))
        self.assertTrue(state.request_seen.wait(timeout=1.0))

        t0 = time.perf_counter()
        exited = runner.shutdown(timeout_s=0.8)
        elapsed = time.perf_counter() - t0
        self.assertTrue(exited)
        self.assertLess(elapsed, 0.8)
        state.hang_event.set()
        self.assertIsNone(runner.poll_latest())


class ProviderProtocolValidationAndHealthTests(unittest.TestCase):
    def test_three_provider_request_bodies_and_dual_image_order(self):
        with tempfile.TemporaryDirectory() as tmp:
            rgb = Path(tmp) / "front.png"
            lmap = Path(tmp) / "local_map.png"
            rgb.write_bytes(b"\x89PNG\r\n\x1a\nfront")
            lmap.write_bytes(b"\x89PNG\r\n\x1a\nlocalmap")

            # 1) DeepSeek: deepseek-flash, thinking disabled, pure text
            ds_cfg = LlmClientConfig(provider="deepseek", api_key="secret-ds")
            ds_msgs = build_visual_first_messages(
                {}, [{"id": "F1"}], {}, rgb_path=rgb, local_map_path=lmap, supports_vision=ds_cfg.supports_vision
            )
            ds_body = FastSentinelClient(ds_cfg).build_request_body(ds_msgs)
            self.assertEqual(ds_body["model"], "deepseek-flash")
            self.assertEqual(ds_body["thinking"], {"type": "disabled"})
            self.assertNotIn("enable_thinking", ds_body)
            self.assertTrue(ds_body["stream"])
            self.assertIsInstance(ds_msgs[1]["content"], str)

            # 2) Qwen: qwen3.8-flash, enable_thinking=False, dual images in front-then-map order
            qw_cfg = LlmClientConfig(provider="qwen", api_key="secret-qw")
            qw_msgs = build_visual_first_messages(
                {}, [{"id": "F1"}], {}, rgb_path=rgb, local_map_path=lmap, supports_vision=qw_cfg.supports_vision
            )
            qw_body = FastSentinelClient(qw_cfg).build_request_body(qw_msgs)
            self.assertEqual(qw_body["model"], "qwen3.8-flash")
            self.assertIs(qw_body["enable_thinking"], False)
            self.assertNotIn("thinking", qw_body)
            self.assertTrue(qw_body["stream"])
            content = qw_msgs[1]["content"]
            self.assertEqual(len(content), 3)
            self.assertEqual(content[0]["type"], "text")
            self.assertIn('"images":["front_rgb","local_topdown_candidates"]', content[0]["text"])
            self.assertEqual(content[1]["type"], "image_url")
            self.assertEqual(content[2]["type"], "image_url")
            self.assertNotEqual(content[1]["image_url"]["url"], content[2]["image_url"]["url"])

            # 3) openai_compatible: no vendor thinking flags unless extra_body is given
            oc_cfg = LlmClientConfig(
                provider="openai_compatible", base_url="http://127.0.0.1:8000/v1", api_key="secret-oc"
            )
            oc_body = FastSentinelClient(oc_cfg).build_request_body(qw_msgs)
            self.assertNotIn("thinking", oc_body)
            self.assertNotIn("enable_thinking", oc_body)

    def test_api_key_redacted_in_repr_and_safe_dict(self):
        secret = "sk-super-secret-token-value-12345"
        cfg = LlmClientConfig(provider="deepseek", api_key=secret)
        self.assertNotIn(secret, repr(cfg))
        self.assertNotIn(secret, str(cfg))
        safe = cfg.to_safe_dict()
        self.assertNotIn("api_key", safe)
        self.assertNotIn(secret, json.dumps(safe))
        self.assertTrue(safe["has_api_key"])

    def test_preempt_requires_valid_candidate_id_in_current_frame(self):
        for raw in (
            '{"act":"PREEMPT"}',
            '{"act":"PREEMPT","target_id":null}',
            '{"act":"PREEMPT","target_id":123}',
            '{"act":"PREEMPT","target_id":{"id":"F1"}}',
            '{"act":"PREEMPT","target_id":"F9"}',
        ):
            v = parse_sentinel_verdict(raw, valid_candidate_ids=["F1", "W1"])
            self.assertEqual(v.act, "CONT", raw)
            self.assertIsNone(v.target_id, raw)
            self.assertTrue(v.fallback, raw)

        ok = parse_sentinel_verdict(
            '{"act":"PREEMPT","target_id":"F1","reason":"closer"}',
            valid_candidate_ids=["F1", "W1"],
            frame_seq=3,
            action_epoch=2,
        )
        self.assertEqual(ok.act, "PREEMPT")
        self.assertEqual(ok.target_id, "F1")
        self.assertFalse(ok.fallback)
        self.assertEqual(ok.frame_seq, 3)
        self.assertEqual(ok.action_epoch, 2)

    def test_active_slam_health_states_and_spin_deduplication(self):
        unk = active_slam_health(None)
        self.assertEqual(unk["status"], "unknown")
        self.assertFalse(unk["needs_active_loop"])
        self.assertEqual(unk["recommended_action"], "none")

        # Real prior_localization backend stats
        cfg = BackendConfig()
        room = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]], dtype=float)
        prior_store = MapStore(0.2)
        prior_store.add(Keyframe((1, 0), 0, np.eye(4), np.eye(4), room), np.eye(4), descriptor(room, cfg.scan_context))
        reloc_stats = SlamBackend(cfg, prior_map=prior_store).observability_stats()
        prior = active_slam_health({"stats": reloc_stats})
        self.assertEqual(prior["status"], "prior_localization")
        self.assertFalse(prior["needs_active_loop"])
        self.assertEqual(prior["recommended_action"], "none")

        # Exact threshold is healthy
        at_thresh = active_slam_health({"stats": {"travel_since_last_loop_m": 20.0, "last_loop_age_s": 180.0}})
        self.assertEqual(at_thresh["status"], "healthy")
        self.assertFalse(at_thresh["needs_active_loop"])

        # Deduplication tracker prevents repeated SPIN on same 25 m state & in-flight spin
        tracker = ActiveSlamSpinTracker()
        status_25m = {"keyframes": 40, "loops": 0, "stats": {"travel_since_last_loop_m": 25.0, "last_loop_age_s": 10.0}}
        first = tracker.assess(status_25m, now_s=100.0)
        self.assertEqual(first["status"], "needs_active_loop")
        self.assertTrue(first["should_trigger_spin"])

        tracker.mark_spin_started(keyframes=40, loops=0, travel_m=25.0, now_s=100.0)
        during = tracker.assess(status_25m, now_s=101.0)
        self.assertFalse(during["should_trigger_spin"])
        self.assertEqual(during["suppressed_reason"], "spin_in_flight")

        tracker.mark_spin_finished()
        dup = tracker.assess(status_25m, now_s=102.0)
        self.assertFalse(dup["should_trigger_spin"])
        self.assertEqual(dup["suppressed_reason"], "duplicate_spin_state")


class YawChainSafetyAndPidFixTests(unittest.TestCase):
    def test_experiment_cli_check_freezes_fast_yaw_from_overlay(self):
        import contextlib
        import io
        from embodied_navigation.simulation.dynanav import experiment as exp_mod

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            scene = root / "office.usd"
            robot = root / "runtime_assets/robot/go2_description/go2_description.usda"
            overlay = root / "overlay.json"
            scene.write_text("usd", encoding="utf-8")
            robot.parent.mkdir(parents=True)
            robot.write_text("usda", encoding="utf-8")
            overlay.write_text(json.dumps({"interactive_yaw_max_rate_rps": 0.55}), encoding="utf-8")

            stdout = io.StringIO()
            with (
                patch.object(exp_mod, "data_root", return_value=root),
                patch("embodied_navigation.robot.unitree.robogauge.runtime.default_model_path", return_value=root / "robogauge_policy.pt"),
                patch("embodied_navigation.robot.unitree.robogauge.runtime.verify_model", return_value=None),
                contextlib.redirect_stdout(stdout),
            ):
                rc = exp_mod.main(
                    [
                        "--output",
                        str(root / "out_run"),
                        "--scene",
                        str(scene),
                        "--interactive",
                        "--interactive-pose-source",
                        "livo_post_vio",
                        "--experiment",
                        str(overlay),
                        "--check",
                    ]
                )
            self.assertEqual(rc, 0)
            payload = json.loads(stdout.getvalue())
            self.assertEqual(payload["profile"]["yaw_action"]["yaw_rate_rps"], 0.55)

    def test_fast_yaw_chain_through_capture_control_without_relaxing_maneuver(self):
        exp = {"interactive_yaw_max_rate_rps": 0.55, "speed_mps": 0.6}
        receipt = {"policy": {}}
        annotate_policy_receipt(
            receipt, exp, False, True, 0.6, 1.1, interactive_control=True
        )
        self.assertEqual(receipt["speed_test"]["yaw_command_limit_radps"], 0.55)
        self.assertEqual(receipt["speed_test"]["route_yaw_command_limit_radps"], 0.15)
        self.assertEqual(receipt["speed_test"]["maneuver_yaw_command_limit_radps"], 0.15)

        # NativeWaypointTracker allows 0.55 yaw, but keeps maneuver capped at 0.15
        with tempfile.TemporaryDirectory() as tmp:
            mailbox = Path(tmp) / "run" / "mailbox"
            tracker = NativeWaypointTracker(
                {
                    "slam_pose_source": "lio_estimate",
                    "speed_mps": 0.6,
                    "motion_end_s": 200.0,
                    "interactive_mailbox": str(mailbox),
                    "yaw_max_rate_rad_s": 0.55,
                }
            )
            tracker._handle_request(
                {
                    "version": 1,
                    "run_id": "run",
                    "seq": 1,
                    "action": "maneuver",
                    "pose_source": "lio_estimate",
                    "yaw_rate_rps": 0.30,
                    "duration_s": 0.5,
                    "created_wall_s": 10.0,
                    "ttl_s": 10.0,
                },
                now_wall=10.0,
            )
            receipts = [json.loads(line) for line in tracker.receipts_path.read_text().splitlines()]
            self.assertEqual(receipts[-1]["event"], "request_rejected")
            self.assertEqual(receipts[-1]["reason"], "invalid_yaw_rate_rps")

        # ControlLoop allows 0.55 ONLY when tracker.phase == "yaw", rejects 0.30 during maneuver/cruise
        class _FakeTensor:
            def __init__(self, arr):
                self._arr = np.asarray(arr, dtype=float)

            def numpy(self):
                return self._arr

        class _FakeArt:
            def get_world_poses(self):
                return _FakeTensor([[0.0, 0.0, 0.3]]), _FakeTensor([[1.0, 0.0, 0.0, 0.0]])

            def get_velocities(self):
                return _FakeTensor([[0.0, 0.0, 0.0]]), _FakeTensor([[0.0, 0.0, 0.0]])

        class _FakeTracker:
            def __init__(self, phase, cmd):
                self.phase = phase
                self.stage = 0
                self.arrived = False
                self.failure = None
                self.events = []
                self.terminal_reason = None
                self._cmd = cmd

            def observe_native_guards(self, **_kwargs):
                pass

            def tick(self, *_args, **_kwargs):
                return self._cmd, {"status": "tracking"}

        class _FakePolicy:
            def __init__(self):
                self.qdefault = np.zeros(12)
                self.cmd = np.zeros(3)

            def step(self, _i, _state, request):
                self.cmd = request.copy()
                return np.zeros(12), np.zeros(45), 0.0, np.zeros(12)

        class _FakeReceipts:
            def __init__(self):
                self.result = {"freshness_hold_events": []}

            def journal(self, _name, _payload):
                pass

            def save(self, force=False):
                pass

        import types

        sim_mod = types.ModuleType("isaacsim.core.simulation_manager")
        sim_mod.SimulationManager = types.SimpleNamespace(get_simulation_time=lambda: 10.0)
        obs_mod = types.ModuleType("embodied_navigation.robot.unitree.policy.native.observation")
        obs_mod.quaternion_rotation = lambda _q: np.eye(3)
        fake_modules = {
            "isaacsim": types.ModuleType("isaacsim"),
            "isaacsim.core": types.ModuleType("isaacsim.core"),
            "isaacsim.core.simulation_manager": sim_mod,
            "embodied_navigation.robot.unitree.policy.native.observation": obs_mod,
        }
        ros = types.SimpleNamespace(spin_once=lambda _t=0.0: None)
        rows = [
            {
                "root_velocity_world": [[[0.0, 0.0, 0.0]], [[0.0, 0.0, 0.0]]],
                "root_pose_wxyz": [[0.0, 0.0, 0.3]],
                "contacts": [0, 1, 1, 1, 1],
            }
            for _ in range(250)
        ]

        with patch.dict(sys.modules, fake_modules):
            # Active independent yaw -> 0.55 passes
            yaw_loop = ControlLoop(
                experiment=exp,
                gt_diagnostic=False,
                interactive_control=True,
                robogauge=True,
                target_speed=0.6,
                indices=list(range(12)),
                target=np.zeros(12),
                tracker=_FakeTracker("yaw", [0.0, 0.0, 0.55]),
                policy=_FakePolicy(),
                receipts=_FakeReceipts(),
                latest_nav=[None],
                latest_odom=[None],
            )
            yaw_loop.stand_gate_done = True
            yaw_loop.policy_stand_gate_done = True
            yaw_loop.tick(
                iteration=0,
                q=np.zeros(12),
                qd=np.zeros(12),
                art=_FakeArt(),
                rows=rows,
                contact_state=[0, 1, 1, 1, 1],
                previous_completed_tau=np.zeros(12),
                ros=ros,
                policy_frames=[],
            )

            # Active maneuver -> 0.30 MUST fail assertion!
            man_loop = ControlLoop(
                experiment=exp,
                gt_diagnostic=False,
                interactive_control=True,
                robogauge=True,
                target_speed=0.6,
                indices=list(range(12)),
                target=np.zeros(12),
                tracker=_FakeTracker("maneuver", [0.0, 0.0, 0.30]),
                policy=_FakePolicy(),
                receipts=_FakeReceipts(),
                latest_nav=[None],
                latest_odom=[None],
            )
            man_loop.stand_gate_done = True
            man_loop.policy_stand_gate_done = True
            with self.assertRaises(AssertionError):
                man_loop.tick(
                    iteration=0,
                    q=np.zeros(12),
                    qd=np.zeros(12),
                    art=_FakeArt(),
                    rows=rows,
                    contact_state=[0, 1, 1, 1, 1],
                    previous_completed_tau=np.zeros(12),
                    ros=ros,
                    policy_frames=[],
                )

    def test_stale_imu_does_not_reverse_drive_d_term_in_instant_or_dwell(self):
        for mode in ("instant", "dwell"):
            with self.subTest(mode=mode):
                cfg = dict(
                    yaw_config(
                        {
                            "interactive_yaw_max_rate_rps": 0.55,
                            "interactive_yaw_kp": 1.2,
                            "interactive_yaw_kd": 1.0,
                            "interactive_yaw_completion_mode": mode,
                        }
                    ),
                    max_sim_age_s=0.25,
                    max_wall_age_s=0.5,
                )
                action = {"target_yaw_rad": None, "parameters": {"delta_yaw_rad": 0.10}}
                stale_ctx = {
                    "sim": 10.0,
                    "stamp": 10.0,
                    "raw_age": 0.0,
                    "wall_age": 0.0,
                    "distinct": True,
                    "measured_yaw_rate_rps": 0.5,
                    "imu_yaw_rate": 0.5,
                    "imu_source_stamp_s": 8.0,  # 2.0 s stale!
                    "imu_arrival_wall_s": 8.0,
                    "imu_frame": "go2_imu",
                    "imu_valid": True,
                }
                cmd, window, done = yaw_action_step(
                    action, cfg, stale_ctx, None, yaw=0.0, now_wall=10.0, future_tolerance_s=2e-6
                )
                self.assertIsNone(window)
                self.assertIsNone(done)
                self.assertAlmostEqual(cmd[2], 0.12, places=12)

    def test_strict_slew_rate_irregular_1ms_pi_wrap_and_deadband_braking(self):
        cfg = dict(
            yaw_config(
                {
                    "interactive_yaw_max_rate_rps": 0.55,
                    "interactive_yaw_kp": 2.0,
                    "interactive_yaw_max_accel_rps2": 1.0,
                    "interactive_yaw_completion_mode": "dwell",
                    "interactive_yaw_control_tolerance_rad": 0.02,
                    "interactive_yaw_completion_tolerance_rad": 0.05,
                }
            ),
            max_sim_age_s=0.25,
            max_wall_age_s=0.5,
        )
        # 1) Irregular 1 ms step + duplicate/backward timestamp + deadband entry
        action = {"target_yaw_rad": None, "parameters": {"delta_yaw_rad": 0.5}}
        c1, w, _ = yaw_action_step(action, cfg, {"sim": 10.0}, None, yaw=0.0, now_wall=10.0, future_tolerance_s=2e-6)
        self.assertAlmostEqual(c1[2], 0.020, places=12)

        c2, w, _ = yaw_action_step(action, cfg, {"sim": 10.001}, w, yaw=0.0, now_wall=10.001, future_tolerance_s=2e-6)
        self.assertAlmostEqual(c2[2], 0.021, places=12)
        self.assertLessEqual(abs(c2[2] - c1[2]) / 0.001, 1.0 + 1e-9)

        # Non-monotonic/duplicate timestamp does not jump
        c_dup, w, _ = yaw_action_step(action, cfg, {"sim": 10.001}, w, yaw=0.0, now_wall=10.001, future_tolerance_s=2e-6)
        self.assertAlmostEqual(c_dup[2], c2[2], places=12)

        # Enter deadband 20 ms later -> decelerates cleanly within 1.0 rad/s^2 (not instant drop to 0)
        c3, w, _ = yaw_action_step(action, cfg, {"sim": 10.021}, w, yaw=0.49, now_wall=10.021, future_tolerance_s=2e-6)
        self.assertAlmostEqual(c3[2], 0.001, places=12)
        self.assertLessEqual(abs(c3[2] - c2[2]) / 0.020, 1.0 + 1e-9)

        # 2) Crossing +/- pi wrap respects slew limit
        wrap_action = {"target_yaw_rad": None, "parameters": {"delta_yaw_rad": 0.3}}
        cw1, ww, _ = yaw_action_step(
            wrap_action, cfg, {"sim": 20.0}, None, yaw=math.pi - 0.05, now_wall=20.0, future_tolerance_s=2e-6
        )
        cw2, _, _ = yaw_action_step(
            wrap_action, cfg, {"sim": 20.02}, ww, yaw=-math.pi + 0.02, now_wall=20.02, future_tolerance_s=2e-6
        )
        self.assertLessEqual(abs(cw2[2] - cw1[2]) / 0.02, 1.0 + 1e-9)

    def test_conditional_anti_windup_prevents_saturation_accumulation(self):
        cfg = dict(
            yaw_config(
                {
                    "interactive_yaw_max_rate_rps": 0.03,
                    "interactive_yaw_kp": 1.2,
                    "interactive_yaw_ki": 1.0,
                    "interactive_yaw_integral_band_rad": 0.18,
                    "interactive_yaw_completion_mode": "instant",
                    "interactive_yaw_completion_tolerance_rad": 0.005,
                }
            ),
            max_sim_age_s=0.25,
            max_wall_age_s=0.5,
        )
        action = {"target_yaw_rad": None, "parameters": {"delta_yaw_rad": 0.12}}
        window = None
        for i in range(10):
            sim = 10.0 + i * 0.1
            cmd, window, _ = yaw_action_step(
                action, cfg, {"sim": sim}, window, yaw=0.0, now_wall=sim, future_tolerance_s=2e-6
            )
            self.assertAlmostEqual(cmd[2], 0.03, places=12)
        self.assertAlmostEqual(action["_control_state"]["integral"], 0.0, places=12)

        # Once error shrinks below saturation, output drops immediately instead of staying pinned.
        cmd_small, _, _ = yaw_action_step(
            action, cfg, {"sim": 11.0}, window, yaw=0.11, now_wall=11.0, future_tolerance_s=2e-6
        )
        self.assertLess(cmd_small[2], 0.025)


class MailboxEpochCancelAndPlannerTests(unittest.TestCase):
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

    def _tick(self, stamp=10.0):
        return self.adapter.tick(
            current_sim_s=stamp,
            odom_stamp_s=stamp,
            position=[0.0, 0.0, 0.0],
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

    def test_concurrent_submit_unique_seq_and_ordered_inbox_consumption(self):
        barrier = threading.Barrier(2)
        results: list[dict] = []
        lock = threading.Lock()

        def worker(x):
            barrier.wait(timeout=2.0)
            p = waypoint_mailbox.submit(
                self.mailbox,
                action="goto",
                goal=[x, 0.0, 0.0],
                pose_source="lio_estimate",
            )
            with lock:
                results.append(p)

        t1 = threading.Thread(target=worker, args=(2.0,))
        t2 = threading.Thread(target=worker, args=(4.0,))
        t1.start()
        t2.start()
        t1.join(timeout=2.0)
        t2.join(timeout=2.0)
        self.assertEqual(sorted(r["seq"] for r in results), [1, 2])

        # Tick once: both seq=1 and seq=2 must be consumed from waypoint-inbox without dropping!
        self._tick(10.0)
        accepted = [r["seq"] for r in self._receipts() if r["event"] == "waypoint_accepted"]
        self.assertEqual(accepted, [1, 2])
        self.assertEqual(self.adapter.active_waypoint["seq"], 1)
        self.assertEqual(len(self.adapter._pending), 1)

    def test_epoch_isolation_and_distinct_current_vs_active_epoch(self):
        # Enable epoch=2 with seq=5
        self.adapter._handle_request(
            {
                "version": 1,
                "run_id": "run",
                "seq": 5,
                "action": "goto",
                "pose_source": "lio_estimate",
                "goal_xyz_m": [4.0, 0.0, 0.0],
                "action_epoch": 2,
            }
        )
        self._tick(10.0)

        # Missing epoch preempt or same-epoch preempt MUST be rejected once epoch > 0
        self.adapter._handle_request(
            {
                "version": 1,
                "run_id": "run",
                "seq": 6,
                "action": "goto",
                "pose_source": "lio_estimate",
                "goal_xyz_m": [1.0, 1.0, 0.0],
                "preempt": True,
            }
        )
        self.adapter._handle_request(
            {
                "version": 1,
                "run_id": "run",
                "seq": 7,
                "action": "goto",
                "pose_source": "lio_estimate",
                "goal_xyz_m": [1.0, 1.0, 0.0],
                "preempt": True,
                "action_epoch": 2,
            }
        )
        self.assertEqual(self.adapter.active_waypoint["seq"], 5)
        rejected = [r for r in self._receipts() if r["event"] == "request_rejected"]
        self.assertEqual([r["seq"] for r in rejected], [6, 7])
        self.assertTrue(all(r["reason"] == "stale_action_epoch" for r in rejected))

        # Queuing normal waypoint with epoch=3 keeps active_action_epoch=2 while current_epoch=3
        self.adapter._handle_request(
            {
                "version": 1,
                "run_id": "run",
                "seq": 8,
                "action": "goto",
                "pose_source": "lio_estimate",
                "goal_xyz_m": [6.0, 0.0, 0.0],
                "action_epoch": 3,
            }
        )
        self._tick(10.1)
        obs = json.loads(self.adapter.observation_path.read_text())
        self.assertEqual(obs["waypoints"]["active_seq"], 5)
        self.assertEqual(obs["waypoints"]["active_action_epoch"], 2)
        self.assertEqual(obs["waypoints"]["current_epoch"], 3)

    def test_cancel_and_large_batch_preempt_emit_all_terminal_receipts(self):
        for seq in range(1, 51):
            waypoint_mailbox.submit(
                self.mailbox,
                action="goto",
                goal=[float(seq), 0.0, 0.0],
                pose_source="lio_estimate",
                extra={"action_epoch": 1},
            )
        self._tick(10.0)
        # Preempt all 50 items (> 40 lines!)
        waypoint_mailbox.submit(
            self.mailbox,
            action="goto",
            goal=[0.0, 2.0, 0.0],
            pose_source="lio_estimate",
            extra={"preempt": True, "action_epoch": 2},
        )
        self._tick(10.1)
        res1 = waypoint_mailbox.wait_for_action(self.mailbox, 1, timeout_s=0.5, poll_s=0.01)
        self.assertEqual(res1["status"], "preempted")
        self.assertEqual(res1["seq"], 1)

        # Queue another waypoint and test cancel + stale stop CLI wait
        waypoint_mailbox.submit(
            self.mailbox,
            action="goto",
            goal=[2.0, 2.0, 0.0],
            pose_source="lio_estimate",
            extra={"action_epoch": 2},
        )
        self._tick(10.2)
        cancel_payload = waypoint_mailbox.submit(
            self.mailbox,
            action="cancel",
            pose_source="lio_estimate",
            extra={"action_epoch": 2},
        )
        self._tick(10.3)
        cancel_res = waypoint_mailbox.wait_for_action(
            self.mailbox, cancel_payload["seq"], timeout_s=0.5, poll_s=0.01
        )
        self.assertTrue(cancel_res["success"])
        self.assertEqual(cancel_res["event"], "cancel_accepted")
        cancelled = [r for r in self._receipts() if r["event"] == "action_cancelled"]
        self.assertGreaterEqual(len(cancelled), 2)

        # Stale-epoch stop via CLI --wait must fail with exit code 1
        stop_payload = waypoint_mailbox.submit(
            self.mailbox, action="stop", pose_source="lio_estimate", extra={"action_epoch": 1}
        )
        self._tick(10.4)
        stop_res = waypoint_mailbox.wait_for_action(
            self.mailbox, stop_payload["seq"], timeout_s=0.5, poll_s=0.01
        )
        self.assertFalse(stop_res["success"])
        self.assertEqual(stop_res["status"], "rejected")
        self.assertEqual(stop_res["reason"], "stale_action_epoch")

        # Verify CLI parser and main() support --wait on stop/cancel
        t_tick = threading.Timer(0.05, lambda: self._tick(10.5))
        t_tick.start()
        exit_code = client.main(
            ["--mailbox", str(self.mailbox), "stop", "--action-epoch", "1", "--wait", "--timeout", "1.0"]
        )
        t_tick.join(timeout=1.0)
        self.assertEqual(exit_code, client.EXIT_ACTION_FAILED)

        # Verify explore_waypoint_client.py remains slimmed below 2314 lines
        script_lines = (Path(__file__).resolve().parents[2] / "scripts" / "explore_waypoint_client.py").read_text().splitlines()
        self.assertLessEqual(len(script_lines), 2314)

    def test_pct_plan_coordinator_cancels_obsolete_goals_and_starts_latest(self):
        started: list[int] = []
        seq1_started = threading.Event()
        seq1_release = threading.Event()

        def mock_plan(seq_tag: int, *, cancel_event=None):
            started.append(seq_tag)
            if seq_tag == 1:
                seq1_started.set()
                cancel_event.wait(timeout=2.0)
                seq1_release.wait(timeout=2.0)
                return {"status": "cancelled", "points_xyz_m": []}
            return {"status": "ready", "points_xyz_m": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]}

        coord = PctPlanCoordinator(mock_plan, max_workers=1, retry_interval_s=0.0)
        self.addCleanup(seq1_release.set)
        self.addCleanup(coord.shutdown)

        self.assertTrue(coord.submit(1, 1, now_mono=10.0))
        self.assertTrue(seq1_started.wait(timeout=1.0))

        # Preempt seq=1 with seq=2, then immediately preempt seq=2 with seq=3
        coord.on_request_changed(2)
        coord.submit(2, 2, now_mono=10.05)
        self.assertTrue(coord.active_cancel.is_set())
        coord.on_request_changed(3)
        coord.submit(3, 3, now_mono=10.06)

        seq1_release.set()
        fut = None
        deadline = time.perf_counter() + 1.5
        while time.perf_counter() < deadline:
            fut = coord.poll(3, now_mono=time.monotonic())
            if fut is not None:
                break
            time.sleep(0.01)
        self.assertIsNotNone(fut)
        self.assertEqual(fut.result()["status"], "ready")
        # Obsolete seq=2 must NEVER have started on the worker!
        self.assertEqual(started, [1, 3])

    def test_measured_pct_planner_honors_cancel_event(self):
        pts = np.array(
            [[x, y, 0.0] for x in np.linspace(-0.5, 1.5, 15) for y in np.linspace(-0.5, 0.5, 9)],
            dtype=float,
        )
        planner = MeasuredPctPlanner(pts)
        ev = threading.Event()
        ev.set()
        res = planner.plan([0.0, 0.0, 0.32], [1.0, 0.0, 0.32], cancel_event=ev)
        self.assertFalse(res["found"])
        self.assertEqual(res["reason"], "cancelled")


class EndToEndSentinelCoordinatorTests(unittest.TestCase):
    def test_observation_sse_preempt_spin_stop_mailbox_receipt_loop(self):
        state = _StreamingMockState()
        state.mode = "sse_split_hang"
        server, base_url = _start_streaming_server(state)
        self.addCleanup(state.hang_event.set)
        self.addCleanup(server.shutdown)
        self.addCleanup(server.server_close)

        with tempfile.TemporaryDirectory() as tmp:
            mailbox = Path(tmp) / "run_e2e" / "interactive"
            rgb = Path(tmp) / "front.png"
            lmap = Path(tmp) / "local.png"
            rgb.write_bytes(b"\x89PNG\r\n\x1a\nfront")
            lmap.write_bytes(b"\x89PNG\r\n\x1a\nlocal")

            tracker = NativeWaypointTracker(
                {
                    "slam_pose_source": "livo_post_vio",
                    "speed_mps": 0.6,
                    "motion_end_s": 200.0,
                    "interactive_mailbox": str(mailbox),
                }
            )

            def tick_tracker(stamp):
                wall_now = time.perf_counter()
                return tracker.tick(
                    current_sim_s=stamp,
                    odom_stamp_s=stamp,
                    position=[0.0, 0.0, 0.3],
                    quat_wxyz=[1.0, 0.0, 0.0, 0.0],
                    estimated_velocity=None,
                    measured_yaw_rate=0.0,
                    ready=True,
                    now_wall=wall_now,
                    arrival_wall=wall_now,
                    frame_id="camera_init",
                    child_frame_id="body",
                    pose_source="livo_post_vio",
                )

            # Start initial goto seq=1
            waypoint_mailbox.submit(
                mailbox, action="goto", goal=[5.0, 0.0, 0.3], pose_source="livo_post_vio"
            )
            tick_tracker(10.0)
            self.assertEqual(tracker.active_waypoint["seq"], 1)

            cfg = LlmClientConfig(
                provider="qwen", base_url=base_url, api_key="k-e2e", timeout_s=1.5, stream=True
            )
            runner = AsyncDecisionRunner(FastSentinelClient(cfg))
            self.addCleanup(runner.shutdown)
            coord = SentinelCoordinator(
                mailbox, runner, pose_source="livo_post_vio", spin_delta_rad=math.pi
            )
            candidates = [{"id": "F1", "kind": "frontier", "xy": [2.0, 3.0], "z": 0.3, "dist_m": 3.6}]

            # 1) Stream PREEMPT F1
            with state.lock:
                state.sse_chunks = ['{"act":"PREEMPT",', '"target_id":"F1","reason":"better"}']
            coord.submit_observation(
                None,
                candidates,
                {"keyframes": 10, "loops": 0, "stats": {"travel_since_last_loop_m": 4.0, "last_loop_age_s": 5.0}},
                rgb_path=rgb,
                local_map_path=lmap,
                now_s=10.0,
            )
            deadline = time.perf_counter() + 1.5
            dispatched = None
            while time.perf_counter() < deadline:
                dispatched = coord.poll_and_dispatch(now_s=10.1)
                if dispatched is not None:
                    break
                time.sleep(0.01)
            self.assertIsNotNone(dispatched)
            self.assertTrue(dispatched["dispatched"])
            self.assertEqual(dispatched["payload"]["action"], "goto")
            self.assertEqual(dispatched["payload"]["goal_xyz_m"], [2.0, 3.0, 0.3])
            tick_tracker(10.1)
            self.assertEqual(tracker.active_waypoint["seq"], dispatched["payload"]["seq"])

            # 2) Stream SPIN -> yaw preempt
            with state.lock:
                state.sse_chunks = ['{"act":"SPIN",', '"target_id":null,"reason":"need loop"}']
            coord.submit_observation(
                None,
                candidates,
                {"keyframes": 30, "loops": 0, "stats": {"travel_since_last_loop_m": 25.0, "last_loop_age_s": 10.0}},
                rgb_path=rgb,
                local_map_path=lmap,
                now_s=11.0,
            )
            deadline = time.perf_counter() + 1.5
            dispatched_spin = None
            while time.perf_counter() < deadline:
                dispatched_spin = coord.poll_and_dispatch(now_s=11.1)
                if dispatched_spin is not None:
                    break
                time.sleep(0.01)
            self.assertIsNotNone(dispatched_spin)
            self.assertTrue(dispatched_spin["dispatched"])
            self.assertEqual(dispatched_spin["payload"]["action"], "yaw")
            tick_tracker(11.1)
            self.assertEqual(tracker.phase, "yaw")

            # 3) Stream STOP -> stop_accepted receipt
            with state.lock:
                state.sse_chunks = ['{"act":"STOP",', '"reason":"blocked"}']
            coord.submit_observation(
                None,
                candidates,
                {"keyframes": 31, "loops": 0, "stats": {"travel_since_last_loop_m": 25.0, "last_loop_age_s": 10.0}},
                rgb_path=rgb,
                local_map_path=lmap,
                now_s=12.0,
            )
            deadline = time.perf_counter() + 1.5
            dispatched_stop = None
            while time.perf_counter() < deadline:
                dispatched_stop = coord.poll_and_dispatch(now_s=12.1)
                if dispatched_stop is not None:
                    break
                time.sleep(0.01)
            self.assertIsNotNone(dispatched_stop)
            self.assertTrue(dispatched_stop["dispatched"])
            self.assertEqual(dispatched_stop["payload"]["action"], "stop")
            tick_tracker(12.1)
            receipt = waypoint_mailbox.wait_for_action(
                mailbox, dispatched_stop["payload"]["seq"], timeout_s=0.5, poll_s=0.01
            )
            self.assertTrue(receipt["success"])
            self.assertEqual(receipt["event"], "stop_accepted")
            self.assertEqual(tracker.terminal_reason, "interactive_stop")


if __name__ == "__main__":
    unittest.main()
