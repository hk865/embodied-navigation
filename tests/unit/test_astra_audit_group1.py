"""Unit tests for Astra audit Group 1 security and reliability fixes."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    HierarchicalMemoryStore,
    KeyframeSceneGraph,
    load_rgb_index_report,
)
from embodied_navigation.localization_mapping.semantic_mapping.backend_adapter import (
    _resolve_rgb_entry_path,
)
from embodied_navigation.planning.exploration import waypoint_mailbox
from embodied_navigation.planning.exploration.llm_agent_client import (
    AsyncDecisionRunner,
    FastSentinelClient,
    LlmClientConfig,
    MAX_IMAGE_UPLOAD_BYTES,
    SentinelVerdict,
    _detect_image_mime,
    _encode_image_part,
    _read_key_file,
    _redact_sensitive,
    _redact_url,
    build_visual_first_messages,
    validate_image_path,
)
from embodied_navigation.planning.exploration.sentinel_coordinator import (
    SentinelCoordinator,
)
from embodied_navigation.simulation.dynanav.native_waypoints import NativeWaypointTracker
from embodied_navigation.simulation.dynanav.pct_scan_worker import PctPlanCoordinator


class _ErrorBodyServerState:
    def __init__(self, status: int, body: str) -> None:
        self.status = status
        self.body = body


def _start_error_server(state: _ErrorBodyServerState) -> tuple[ThreadingHTTPServer, str]:
    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            length = int(self.headers.get("Content-Length", "0"))
            _ = self.rfile.read(length)
            raw = state.body.encode("utf-8")
            self.send_response(state.status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def log_message(self, format, *args):
            return

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    host, port = server.server_address
    return server, f"http://{host}:{port}/v1"


class CredentialRedactionAndTransportTests(unittest.TestCase):
    def test_repr_and_to_safe_dict_redact_extra_body_and_url(self) -> None:
        secret_a = "sk-extra-secret-alpha-111"
        secret_b = "tok-nested-bearer-222"
        secret_c = "pw-list-item-333"
        cfg = LlmClientConfig(
            provider="openai_compatible",
            base_url="https://user:pass@api.example.com:8443/v1?api_key=leakme#frag",
            api_key="sk-main-api-key",
            extra_body={
                "api_key": secret_a,
                "nested": {
                    "auth_header": secret_b,
                    "safe_field": "keep-me",
                    "items": [{"password": secret_c}, {"model_hint": "fast"}],
                },
                "cookie": "session=xyz",
                "credential_id": "cred-9",
            },
        )
        cfg_repr = repr(cfg)
        self.assertNotIn(secret_a, cfg_repr)
        self.assertNotIn(secret_b, cfg_repr)
        self.assertNotIn(secret_c, cfg_repr)
        self.assertNotIn("sk-main-api-key", cfg_repr)

        safe = cfg.to_safe_dict()
        self.assertEqual(safe["base_url"], "https://api.example.com:8443/v1")
        self.assertNotIn("user", safe["base_url"])
        self.assertNotIn("leakme", safe["base_url"])
        self.assertNotIn("frag", safe["base_url"])
        dumped = json.dumps(safe)
        for leaked in (secret_a, secret_b, secret_c, "sk-main-api-key", "session=xyz", "cred-9"):
            self.assertNotIn(leaked, dumped)
        self.assertEqual(safe["extra_body"]["api_key"], "[REDACTED]")
        self.assertEqual(safe["extra_body"]["nested"]["auth_header"], "[REDACTED]")
        self.assertEqual(safe["extra_body"]["nested"]["safe_field"], "keep-me")
        self.assertEqual(safe["extra_body"]["nested"]["items"][0]["password"], "[REDACTED]")
        self.assertEqual(safe["extra_body"]["nested"]["items"][1]["model_hint"], "fast")

        self.assertEqual(
            _redact_url("http://u:p@[::1]:8000/v1/chat?token=1#h"),
            "http://[::1]:8000/v1/chat",
        )
        self.assertEqual(
            _redact_sensitive({"TokenValue": "abc", "ok": [1, {"mySecret": "xyz"}]}),
            {"TokenValue": "[REDACTED]", "ok": [1, {"mySecret": "[REDACTED]"}]},
        )

    def test_host_override_does_not_inherit_preset_api_key(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            xdg = Path(tmp) / "xdg"
            auth_dir = xdg / "opencode"
            auth_dir.mkdir(parents=True)
            (auth_dir / "auth.json").write_text(
                json.dumps({"opencode": {"key": "sk-opencode-preset-secret"}}),
                encoding="utf-8",
            )
            with mock.patch.dict(
                os.environ,
                {
                    "DEEPSEEK_API_KEY": "sk-deepseek-preset-secret",
                    "OPENCODE_API_KEY": "sk-opencode-env-secret",
                    "XDG_DATA_HOME": str(xdg),
                },
                clear=True,
            ):
                # Default host inherits preset key
                default_ds = LlmClientConfig(provider="deepseek")
                self.assertEqual(default_ds.api_key, "sk-deepseek-preset-secret")

                # Overridden host with api_key=None must NOT receive preset key
                custom_ds = LlmClientConfig(
                    provider="deepseek",
                    base_url="https://other-host.example.com/v1",
                )
                self.assertIsNone(custom_ds.api_key)

                custom_oc = LlmClientConfig(
                    provider="opencode",
                    base_url="https://127.0.0.1:8443/v1",
                )
                self.assertIsNone(custom_oc.api_key)

                # Explicit api_key on overridden host is honored
                explicit = LlmClientConfig(
                    provider="deepseek",
                    base_url="https://other-host.example.com/v1",
                    api_key="sk-explicit",
                )
                self.assertEqual(explicit.api_key, "sk-explicit")

    def test_http_with_api_key_rejected_for_non_localhost_and_allowed_for_localhost(self) -> None:
        with self.assertRaisesRegex(ValueError, "insecure http"):
            LlmClientConfig(
                provider="openai_compatible",
                base_url="http://api.example.com:8000/v1",
                api_key="sk-should-fail",
            )
        # Without api_key, remote http is not sending a credential
        no_key = LlmClientConfig(
            provider="openai_compatible",
            base_url="http://api.example.com:8000/v1",
            api_key=None,
        )
        self.assertIsNone(no_key.api_key)

        for local_url in (
            "http://localhost:8000/v1",
            "http://127.0.0.1:8000/v1",
            "http://[::1]:8000/v1",
        ):
            cfg = LlmClientConfig(
                provider="openai_compatible",
                base_url=local_url,
                api_key="sk-local-ok",
            )
            self.assertEqual(cfg.api_key, "sk-local-ok")

    def test_http_error_verdict_never_leaks_response_body(self) -> None:
        state = _ErrorBodyServerState(401, '{"error":"raw secret echo sk-leak-body"}')
        server, base_url = _start_error_server(state)
        self.addCleanup(server.shutdown)
        self.addCleanup(server.server_close)

        cfg = LlmClientConfig(provider="deepseek", base_url=base_url, api_key="sk-local")
        client = FastSentinelClient(cfg)
        verdict = client.evaluate({}, [], {})
        self.assertEqual(verdict.act, "CONT")
        self.assertTrue(verdict.fallback)
        self.assertEqual(verdict.reason, "client_error:HTTP_401")
        self.assertNotIn("sk-leak-body", verdict.reason)


class OpenCodeXdgAndKeyFileTests(unittest.TestCase):
    def test_xdg_auth_providers_and_precedence(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            xdg = Path(tmp) / "xdg"
            auth_dir = xdg / "opencode"
            auth_dir.mkdir(parents=True)
            auth_file = auth_dir / "auth.json"

            # 1) Only opencode-go present -> opencode/opencode_go resolves it, opencode_zen stays None
            auth_file.write_text(
                json.dumps({"opencode-go": {"key": "sk-go-only"}}),
                encoding="utf-8",
            )
            with mock.patch.dict(os.environ, {"XDG_DATA_HOME": str(xdg)}, clear=True):
                self.assertEqual(LlmClientConfig(provider="opencode").api_key, "sk-go-only")
                self.assertEqual(LlmClientConfig(provider="opencode_go").api_key, "sk-go-only")
                self.assertIsNone(LlmClientConfig(provider="opencode_zen").api_key)

            # 2) Both opencode-go and opencode present -> opencode-go takes priority for opencode/opencode_go
            auth_file.write_text(
                json.dumps(
                    {
                        "opencode-go": {"key": "sk-go-pref"},
                        "opencode": {"key": "sk-zen-fallback"},
                    }
                ),
                encoding="utf-8",
            )
            with mock.patch.dict(os.environ, {"XDG_DATA_HOME": str(xdg)}, clear=True):
                self.assertEqual(LlmClientConfig(provider="opencode").api_key, "sk-go-pref")
                self.assertEqual(LlmClientConfig(provider="opencode_zen").api_key, "sk-zen-fallback")

            # 3) Only opencode present -> opencode_go falls back to opencode
            auth_file.write_text(
                json.dumps({"opencode": {"key": "sk-shared-opencode"}}),
                encoding="utf-8",
            )
            with mock.patch.dict(os.environ, {"XDG_DATA_HOME": str(xdg)}, clear=True):
                self.assertEqual(LlmClientConfig(provider="opencode_go").api_key, "sk-shared-opencode")

            # 4) Missing entry -> None
            auth_file.write_text(json.dumps({"other": {"key": "sk-other"}}), encoding="utf-8")
            with mock.patch.dict(os.environ, {"XDG_DATA_HOME": str(xdg)}, clear=True):
                self.assertIsNone(LlmClientConfig(provider="opencode").api_key)

    def test_read_key_file_exact_env_name_matching(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "keys.env"
            p.write_text(
                "# comment\n\nOTHER_KEY=sk-wrong\nexport OPENCODE_API_KEY='sk-right-export'\n",
                encoding="utf-8",
            )
            self.assertEqual(_read_key_file(p, env_name="OPENCODE_API_KEY"), "sk-right-export")
            self.assertIsNone(_read_key_file(p, env_name="DEEPSEEK_API_KEY"))

            bare = Path(tmp) / "bare.txt"
            bare.write_text("# header\nsk-bare-token==\n", encoding="utf-8")
            self.assertEqual(_read_key_file(bare, env_name="OPENCODE_API_KEY"), "sk-bare-token==")


class ImageAndRgbPathValidationTests(unittest.TestCase):
    def test_validate_image_path_whitelist_size_and_magic(self) -> None:
        self.assertEqual(MAX_IMAGE_UPLOAD_BYTES, 10 * 1024 * 1024)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "allowed"
            outside = Path(tmp) / "outside"
            root.mkdir()
            outside.mkdir()

            valid_png = root / "ok.png"
            valid_png.write_bytes(b"\x89PNG\r\n\x1a\npayload")
            valid_jpg = root / "ok.bin"
            valid_jpg.write_bytes(b"\xff\xd8\xff\xe0jpegdata")
            valid_webp = root / "ok.webp"
            valid_webp.write_bytes(b"RIFF\x10\x00\x00\x00WEBPVP8 ")
            fake_png = root / "fake.png"
            fake_png.write_text("not actually a png", encoding="utf-8")
            empty_png = root / "empty.png"
            empty_png.write_bytes(b"")
            oversized = root / "big.png"
            oversized.write_bytes(b"\x89PNG\r\n\x1a\n" + b"0" * 64)
            ext_png = outside / "ext.png"
            ext_png.write_bytes(b"\x89PNG\r\n\x1a\noutside")
            sym_escape = root / "escape.png"
            sym_escape.symlink_to(ext_png)

            self.assertEqual(_detect_image_mime(b"\x89PNG\r\n\x1a\n"), "image/png")
            self.assertEqual(_detect_image_mime(b"\xff\xd8\xff"), "image/jpeg")
            self.assertEqual(_detect_image_mime(b"RIFF0000WEBP"), "image/webp")
            self.assertIsNone(_detect_image_mime(b"plain text"))

            # Valid encodings
            png_part = _encode_image_part(valid_png, allowed_roots=[root])
            self.assertIsNotNone(png_part)
            self.assertTrue(png_part["image_url"]["url"].startswith("data:image/png;base64,"))

            jpg_part = _encode_image_part(valid_jpg, allowed_roots=[root])
            self.assertIsNotNone(jpg_part)
            self.assertTrue(jpg_part["image_url"]["url"].startswith("data:image/jpeg;base64,"))

            webp_part = _encode_image_part(valid_webp, allowed_roots=[root])
            self.assertIsNotNone(webp_part)
            self.assertTrue(webp_part["image_url"]["url"].startswith("data:image/webp;base64,"))

            # Rejections
            self.assertEqual(validate_image_path(fake_png, allowed_roots=[root])[1], "invalid_image_header")
            self.assertIsNone(_encode_image_part(fake_png, allowed_roots=[root]))
            self.assertEqual(validate_image_path(empty_png, allowed_roots=[root])[1], "invalid_file_size")
            self.assertEqual(
                validate_image_path(oversized, allowed_roots=[root], max_bytes=32)[1],
                "invalid_file_size",
            )
            self.assertEqual(validate_image_path(ext_png, allowed_roots=[root])[1], "outside_allowed_roots")
            self.assertEqual(validate_image_path(sym_escape, allowed_roots=[root])[1], "outside_allowed_roots")
            self.assertIsNone(_encode_image_part(sym_escape, allowed_roots=[root]))

    def test_backend_adapter_rejects_escape_paths(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            idx_dir = base / "frames"
            sub_dir = idx_dir / "cam0"
            sub_dir.mkdir(parents=True)
            outside_dir = base / "outside"
            outside_dir.mkdir()

            inside_img = sub_dir / "img_01.png"
            inside_img.write_bytes(b"\x89PNG\r\n\x1a\n1")
            archived_img = idx_dir / "img_02.png"
            archived_img.write_bytes(b"\x89PNG\r\n\x1a\n2")
            outside_img = outside_dir / "secret.png"
            outside_img.write_bytes(b"\x89PNG\r\n\x1a\nsecret")
            symlink_escape = idx_dir / "sym.png"
            symlink_escape.symlink_to(outside_img)

            self.assertEqual(
                _resolve_rgb_entry_path("cam0/img_01.png", idx_dir),
                str(inside_img.resolve()),
            )
            self.assertEqual(
                _resolve_rgb_entry_path("/old/machine/img_02.png", idx_dir),
                str(archived_img.resolve()),
            )
            self.assertIsNone(_resolve_rgb_entry_path(str(outside_img), idx_dir))
            self.assertIsNone(_resolve_rgb_entry_path("../outside/secret.png", idx_dir))
            self.assertIsNone(_resolve_rgb_entry_path("sym.png", idx_dir))
            self.assertIsNone(_resolve_rgb_entry_path("does_not_exist.png", idx_dir))

            jsonl = idx_dir / "index.jsonl"
            jsonl.write_text(
                "\n".join(
                    [
                        json.dumps({"stamp_ns": 100, "rgb_path": str(outside_img)}),
                        json.dumps({"stamp_ns": 200, "rgb_path": "../outside/secret.png"}),
                        json.dumps({"stamp_ns": 300, "rgb_path": "cam0/img_01.png"}),
                    ]
                ),
                encoding="utf-8",
            )
            report = load_rgb_index_report(jsonl)
            self.assertEqual(report.loaded_count, 1)
            self.assertEqual(report.skipped_count, 2)
            self.assertIn(300, report.index)


class SentinelMemoryAndBlacklistTests(unittest.TestCase):
    def test_build_visual_first_messages_serializes_bounded_l0_and_tracks_modalities(self) -> None:
        huge_l0 = {
            "map_revision": 7,
            "floors": [
                {"id": f"f{i}", "z": float(i) * 3.0, "rooms": 4}
                for i in range(10)
            ],
            "rooms": [
                {"id": f"f0_room_{i:02d}", "f": "f0", "c": [float(i), 1.0], "o": i, "a": "x" * 200}
                for i in range(20)
            ],
        }
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            rgb = root / "front.png"
            rgb.write_bytes(b"\x89PNG\r\n\x1a\nfront")
            msgs = build_visual_first_messages(
                {"memory_l0": huge_l0, "all_candidates_blacklisted": True},
                [{"id": "F1", "xy": [1.0, 2.0]}],
                {"status": "healthy"},
                rgb_path=rgb,
                supports_vision=True,
                allowed_image_roots=[root],
            )
            text_obj = json.loads(msgs[1]["content"][0]["text"])
            self.assertIn("memory_l0", text_obj)
            l0_out = text_obj["memory_l0"]
            self.assertEqual(l0_out["rev"], 7)
            self.assertLessEqual(len(l0_out["floors"]), 4)
            self.assertLessEqual(len(l0_out["rooms"]), 8)
            self.assertLessEqual(
                len(json.dumps(l0_out, separators=(",", ":")).encode("utf-8")),
                1024,
            )
            self.assertTrue(text_obj["all_candidates_blacklisted"])

            # Verify FastSentinelClient & AsyncDecisionRunner modality tracking
            state = _ErrorBodyServerState(200, '{"choices":[{"message":{"content":"{\\"act\\":\\"CONT\\",\\"reason\\":\\"ok\\"}"}}]}')
            server, base_url = _start_error_server(state)
            self.addCleanup(server.shutdown)
            self.addCleanup(server.server_close)

            client = FastSentinelClient(
                LlmClientConfig(provider="qwen", base_url=base_url, api_key="k", stream=False),
                allowed_image_roots=[root],
            )
            runner = AsyncDecisionRunner(client)
            self.addCleanup(runner.shutdown)
            runner.submit(
                {
                    "observation_summary": {"memory_l0": huge_l0},
                    "candidates": [{"id": "F1", "xy": [1.0, 2.0]}],
                    "backend_health": {},
                    "rgb_path": str(rgb),
                },
                frame_seq=1,
            )
            deadline = time.perf_counter() + 2.0
            verdict = None
            while time.perf_counter() < deadline:
                verdict = runner.poll_latest()
                if verdict is not None:
                    break
                time.sleep(0.01)
            self.assertIsNotNone(verdict)
            stats = runner.stats()
            self.assertEqual(
                stats["last_request_modalities"],
                {"images": ["front_rgb"], "has_memory_l0": True, "l0_rev": 7},
            )

    def test_sentinel_coordinator_never_falls_back_to_blacklisted_or_dispatches_blacklisted_preempt(self) -> None:
        graph = KeyframeSceneGraph()
        pose = np.eye(4, dtype=np.float64)
        graph.add_keyframe("kf_0", pose, clearance_m=2.0)
        graph.segment_floors()
        graph.partition_rooms()
        store = HierarchicalMemoryStore(graph)
        store.record_attempt("goto", [1.0, 0.0, 0.0], "failed", reason="blocked")

        captured: list[dict] = []
        queued_verdicts: list[SentinelVerdict] = []

        class _StubRunner:
            def submit(self, payload, **kwargs):
                captured.append({"payload": payload, "kwargs": kwargs})
                return True

            def poll_latest(self):
                return queued_verdicts.pop(0) if queued_verdicts else None

        with tempfile.TemporaryDirectory() as tmp:
            mailbox = Path(tmp) / "run" / "interactive"
            mailbox.mkdir(parents=True)
            coord = SentinelCoordinator(mailbox, _StubRunner(), memory_store=store)

            # 1) All candidates blacklisted -> candidate_list stays [] (NOT blocked_candidates!)
            info = coord.submit_observation(
                observation={"pose": {"position_xyz_m": [0.0, 0.0, 0.0]}},
                candidates=[{"id": "F1", "xy": [1.05, 0.0]}],
                backend_status={},
                rgb_path=str(Path(tmp) / "rgb.png"),
            )
            self.assertTrue(info["has_rgb"])
            self.assertTrue(info["has_memory_l0"])
            self.assertEqual(info["blacklisted_candidates"], 1)
            self.assertEqual(captured[0]["payload"]["candidates"], [])
            self.assertEqual(captured[0]["kwargs"]["candidate_ids"], [])
            self.assertTrue(captured[0]["payload"]["observation_summary"]["all_candidates_blacklisted"])
            self.assertEqual(captured[0]["payload"]["observation_summary"]["blacklisted_count"], 1)

            # 2) If a candidate was allowed at submit time, then blacklisted before verdict returns,
            # poll_and_dispatch must still block PREEMPT.
            info2 = coord.submit_observation(
                observation={"pose": {"position_xyz_m": [0.0, 0.0, 0.0]}},
                candidates=[{"id": "F2", "xy": [5.0, 0.0]}],
                backend_status={},
            )
            store.record_attempt("goto", [5.0, 0.0, 0.0], "stuck", reason="newly_blocked")
            queued_verdicts.append(
                SentinelVerdict(
                    act="PREEMPT",
                    target_id="F2",
                    reason="try_f2",
                    frame_seq=info2["frame_seq"],
                    action_epoch=info2["action_epoch"],
                )
            )
            dispatched = coord.poll_and_dispatch()
            self.assertIsNotNone(dispatched)
            self.assertFalse(dispatched["dispatched"])
            self.assertEqual(dispatched["reason"], "blacklisted_goal")
            self.assertIsNone(dispatched["payload"])


class MailboxInterleavingAndCoordinatorThrottleTests(unittest.TestCase):
    def test_inbox_interleaving_barrier_preserves_strict_sequence_without_skip(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            mailbox = Path(tmp) / "run" / "mailbox"
            tracker = NativeWaypointTracker(
                {
                    "slam_pose_source": "lio_estimate",
                    "speed_mps": 0.5,
                    "motion_end_s": 200.0,
                    "interactive_mailbox": str(mailbox),
                }
            )
            orig_drain = tracker._drain_inbox

            def interleaved_drain(now_wall=None):
                # Simulate _drain_inbox completing when inbox was empty, then two submits
                # land BEFORE poll_request reads request.json!
                orig_drain(now_wall)
                waypoint_mailbox.submit(
                    mailbox,
                    action="goto",
                    goal=[2.0, 0.0, 0.0],
                    pose_source="lio_estimate",
                )
                waypoint_mailbox.submit(
                    mailbox,
                    action="goto",
                    goal=[4.0, 0.0, 0.0],
                    pose_source="lio_estimate",
                )

            with mock.patch.object(tracker, "_drain_inbox", side_effect=interleaved_drain):
                tracker.poll_request(now_wall=10.0)

            # Second poll_request runs normal _drain_inbox and must consume 1 -> 2 cleanly
            tracker.poll_request(now_wall=10.05)

            receipts = [
                json.loads(line)
                for line in tracker.receipts_path.read_text(encoding="utf-8").splitlines()
            ]
            accepted_seqs = [r["seq"] for r in receipts if r["event"] == "waypoint_accepted"]
            self.assertEqual(accepted_seqs, [1, 2])
            self.assertFalse(
                any(r.get("reason") == "duplicate_or_stale_sequence" for r in receipts)
            )

    def test_pct_plan_coordinator_can_accept_or_preempt_honors_retry_window(self) -> None:
        gate = threading.Event()

        def slow_plan(tag: int, *, cancel_event=None):
            if tag == 1:
                gate.wait(timeout=2.0)
            return {"status": "ready"}

        coord = PctPlanCoordinator(slow_plan, max_workers=1, retry_interval_s=1.0)
        self.addCleanup(gate.set)
        self.addCleanup(coord.shutdown)

        self.assertTrue(coord.can_accept_or_preempt(1, now_mono=10.0))
        self.assertTrue(coord.submit(1, 1, now_mono=10.0))

        # Active future for seq=1: same seq -> False; different seq (preempt) -> True
        self.assertFalse(coord.can_accept_or_preempt(1, now_mono=10.1))
        self.assertTrue(coord.can_accept_or_preempt(2, now_mono=10.1))

        gate.set()
        deadline = time.perf_counter() + 2.0
        while not coord.active_future.done() and time.perf_counter() < deadline:
            time.sleep(0.01)

        # Done future not yet polled -> False
        self.assertFalse(coord.can_accept_or_preempt(2, now_mono=10.2))

        fut = coord.poll(1, now_mono=10.2)
        self.assertIsNotNone(fut)

        # Within retry_interval_s (1.0 s since last_attempt=10.0) -> False
        self.assertFalse(coord.can_accept_or_preempt(1, now_mono=10.5))
        self.assertFalse(coord.can_accept_or_preempt(1, now_mono=11.0))
        # Past retry_interval_s -> True
        self.assertTrue(coord.can_accept_or_preempt(1, now_mono=11.01))


if __name__ == "__main__":
    unittest.main()
