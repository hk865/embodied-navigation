"""Unit tests for L0-L3 hierarchical memory, failure ledger, STOP gate, and OpenCode presets."""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    HierarchicalMemoryStore,
    KeyframeSceneGraph,
)
from embodied_navigation.planning.exploration.llm_agent_client import (
    LlmClientConfig,
    PROVIDER_PRESETS,
)


def _pose(x: float, y: float, z: float) -> np.ndarray:
    mat = np.eye(4, dtype=np.float64)
    mat[:3, 3] = [x, y, z]
    return mat


def _build_two_floor_graph() -> KeyframeSceneGraph:
    graph = KeyframeSceneGraph()
    # Floor 0 (z = 0.0): Room A around x=0..3, Room B around x=6..9
    for idx, x in enumerate([0.0, 1.0, 2.0, 3.0]):
        graph.add_keyframe(
            f"kf_f0a_{idx}",
            _pose(x, 0.0, 0.0),
            clearance_m=1.8,
            rgb_path=f"/tmp/rgb_f0a_{idx}.jpg",
        )
    # Narrow doorway at x=4.5
    graph.add_keyframe("kf_door", _pose(4.5, 0.0, 0.0), clearance_m=0.45)
    for idx, x in enumerate([6.0, 7.0, 8.0]):
        graph.add_keyframe(
            f"kf_f0b_{idx}",
            _pose(x, 0.0, 0.0),
            clearance_m=1.8,
            rgb_path=f"/tmp/rgb_f0b_{idx}.jpg",
        )
    # Stairs climbing to Floor 1 (z = 3.2)
    for idx, z in enumerate([0.8, 1.6, 2.4]):
        graph.add_keyframe(f"kf_stair_{idx}", _pose(8.0, 1.0 + idx, z), clearance_m=1.0)
    for idx, x in enumerate([8.0, 7.0, 6.0]):
        graph.add_keyframe(
            f"kf_f1_{idx}",
            _pose(x, 4.0, 3.2),
            clearance_m=1.8,
            rgb_path=f"/tmp/rgb_f1_{idx}.jpg",
        )
    graph.segment_floors()
    graph.partition_rooms()
    return graph


class TestOpenCodeProviderConfig(unittest.TestCase):
    def test_opencode_go_and_zen_presets_registered(self) -> None:
        self.assertIn("opencode", PROVIDER_PRESETS)
        self.assertIn("opencode_go", PROVIDER_PRESETS)
        self.assertIn("opencode_zen", PROVIDER_PRESETS)
        self.assertEqual(PROVIDER_PRESETS["opencode"]["base_url"], "https://opencode.ai/zen/go/v1")
        self.assertEqual(PROVIDER_PRESETS["opencode_zen"]["base_url"], "https://opencode.ai/zen/v1")

    def test_opencode_blind_key_file_and_xdg_auth_fallback(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            key_file = tmp_path / "key.txt"
            key_file.write_text("OPENCODE_API_KEY=sk-test-file-token-12345\n", encoding="utf-8")

            with mock.patch.dict(
                os.environ,
                {"OPENCODE_API_KEY_FILE": str(key_file), "XDG_DATA_HOME": str(tmp_path / "xdg")},
                clear=True,
            ):
                cfg = LlmClientConfig(provider="opencode")
                self.assertEqual(cfg.api_key, "sk-test-file-token-12345")
                self.assertIsNotNone(cfg.session_id)
                safe = cfg.to_safe_dict()
                self.assertTrue(safe["has_api_key"])
                self.assertNotIn("sk-test-file-token-12345", json.dumps(safe))
                self.assertNotIn("sk-test-file-token-12345", repr(cfg))

            # Fallback to XDG opencode/auth.json when env vars are unset
            auth_dir = tmp_path / "xdg" / "opencode"
            auth_dir.mkdir(parents=True, exist_ok=True)
            (auth_dir / "auth.json").write_text(
                json.dumps({"opencode": {"type": "api", "key": "sk-xdg-auth-99999"}}),
                encoding="utf-8",
            )
            with mock.patch.dict(os.environ, {"XDG_DATA_HOME": str(tmp_path / "xdg")}, clear=True):
                cfg2 = LlmClientConfig(provider="opencode_zen")
                self.assertEqual(cfg2.api_key, "sk-xdg-auth-99999")
                self.assertEqual(cfg2.base_url, "https://opencode.ai/zen/v1")


class TestHierarchicalMemoryStore(unittest.TestCase):
    def test_l0_to_l3_retrieval_and_roundtrip(self) -> None:
        graph = _build_two_floor_graph()
        store = HierarchicalMemoryStore(graph)
        obj_chair = store.add_object("obj_chair_1", "office chair", "kf_f0a_1", [0.4, 0.2, 0.0])
        obj_tv = store.add_object("obj_tv_1", "whiteboard monitor", "kf_f1_1", [0.0, 0.3, 0.0])

        # L0 topology stays compact (<1 KB)
        l0 = store.l0_global_topology()
        l0_bytes = len(json.dumps(l0, separators=(",", ":")).encode("utf-8"))
        self.assertLess(l0_bytes, 1024)
        self.assertEqual(l0["rev"], 0)
        self.assertGreaterEqual(len(l0["floors"]), 2)

        # L1 ranks matching room first
        l1 = store.l1_submap_summaries(query="whiteboard", top_k=2)
        self.assertTrue(l1)
        self.assertEqual(l1[0]["room_id"], obj_tv.room_id)
        self.assertIn("whiteboard monitor", l1[0]["objects"])

        # L2 retrieves exact entity & hides stale entries by default
        l2 = store.l2_submap_entities(label="chair")
        self.assertEqual(len(l2["objects"]), 1)
        self.assertEqual(l2["objects"][0]["object_id"], obj_chair.object_id)

        store.mark_object_stale(obj_chair.object_id, reason="moved")
        self.assertEqual(len(store.l2_submap_entities(label="chair")["objects"]), 0)
        self.assertEqual(
            len(store.l2_submap_entities(label="chair", include_stale=True)["objects"]), 1
        )

        # L3 returns bounded on-demand RGB refs
        l3 = store.l3_on_demand_keyframes(object_id=obj_tv.object_id, max_images=2)
        self.assertGreaterEqual(len(l3), 1)
        self.assertLessEqual(len(l3), 2)
        self.assertEqual(l3[0]["rgb_path"], "/tmp/rgb_f1_1.jpg")

        # Persistence round-trip
        with tempfile.TemporaryDirectory() as tmp:
            saved = store.save(Path(tmp) / "memory.json")
            reloaded = HierarchicalMemoryStore.load(saved)
            self.assertEqual(reloaded.map_revision, store.map_revision)
            self.assertEqual(len(reloaded.graph.objects), 2)
            self.assertEqual(
                len(reloaded.l2_submap_entities(label="chair", include_stale=False)["objects"]), 0
            )

    def test_failure_ledger_blacklist_and_loop_invalidation(self) -> None:
        graph = _build_two_floor_graph()
        store = HierarchicalMemoryStore(graph)

        store.record_attempt(
            "goto",
            [1.0, 0.0, 0.0],
            "rejected",
            reason="narrow_corner",
            floor_id="floor_0",
            keyframe_id="kf_f0a_1",
        )
        # Repeat failure inside blacklist radius increments repeat counter
        store.record_attempt(
            "goto",
            [1.2, 0.1, 0.0],
            "timeout",
            reason="stuck",
            floor_id="floor_0",
            keyframe_id="kf_f0a_1",
        )

        cands = [
            {"id": "F1", "xyz": [1.1, 0.0, 0.0]},
            {"id": "F2", "xyz": [7.0, 0.0, 0.0]},
        ]
        allowed, blocked = store.filter_candidates(cands, floor_id="floor_0")
        self.assertEqual([c["id"] for c in allowed], ["F2"])
        self.assertEqual([c["id"] for c in blocked], ["F1"])
        self.assertEqual(blocked[0]["blacklist_hits"], 2)

        diag = store.diagnostic_metrics()
        self.assertEqual(diag["repeat_failure_attempts"], 1)
        self.assertAlmostEqual(diag["repeat_failure_rate"], 0.5)

        # Loop closure shifts kf_f0a_1 by 0.5 m -> invalidates stale failure entries
        updated = dict(graph.keyframe_poses)
        updated["kf_f0a_1"] = _pose(1.5, 0.0, 0.0)
        sync_rep = store.sync_poses(updated, failure_invalidation_shift_m=0.25)
        self.assertEqual(sync_rep["map_revision"], 1)
        self.assertEqual(sync_rep["invalidated_failures"], 2)
        allowed_after, blocked_after = store.filter_candidates(cands, floor_id="floor_0")
        self.assertEqual(len(allowed_after), 2)
        self.assertEqual(len(blocked_after), 0)

    def test_verify_stop_claim_prevents_premature_wrong_floor_and_stale_stops(self) -> None:
        graph = _build_two_floor_graph()
        store = HierarchicalMemoryStore(graph)
        obj = store.add_object("obj_printer", "printer", "kf_f1_1", [0.0, 0.0, 0.0])

        # Wrong floor (same XY, Z=0.0 vs Z=3.2) -> rejected
        res_floor = store.verify_stop_claim([7.0, 4.0, 0.0], target_label="printer")
        self.assertFalse(res_floor.allowed)
        self.assertEqual(res_floor.reason, "different_floor_z_mismatch")

        # Same floor but too far -> rejected
        res_far = store.verify_stop_claim([4.0, 4.0, 3.2], target_label="printer", stop_radius_m=1.5)
        self.assertFalse(res_far.allowed)
        self.assertEqual(res_far.reason, "target_beyond_stop_radius")

        # Same floor and within stop radius -> allowed
        res_ok = store.verify_stop_claim([6.5, 4.0, 3.2], target_label="printer", stop_radius_m=1.5)
        self.assertTrue(res_ok.allowed)
        self.assertEqual(res_ok.matched_object_id, obj.object_id)

        # Stale target -> rejected
        store.mark_object_stale(obj.object_id, reason="reobserved_absent")
        res_stale = store.verify_stop_claim([6.5, 4.0, 3.2], target_label="printer")
        self.assertFalse(res_stale.allowed)
        self.assertEqual(res_stale.reason, "target_evidence_stale")
        self.assertEqual(store.diagnostic_metrics()["stale_belief_rejects"], 1)

    def test_sentinel_coordinator_uses_memory_blacklist_and_l0(self) -> None:
        from embodied_navigation.planning.exploration.sentinel_coordinator import (
            SentinelCoordinator,
        )

        graph = _build_two_floor_graph()
        store = HierarchicalMemoryStore(graph)
        store.record_attempt("goto", [1.0, 0.0, 0.0], "stuck", reason="tight_corner")

        captured: list[dict] = []

        class _StubRunner:
            def submit(self, payload, **kwargs):
                captured.append({"payload": payload, "kwargs": kwargs})
                return True

            def poll_latest(self):
                return None

        with tempfile.TemporaryDirectory() as tmp:
            coord = SentinelCoordinator(Path(tmp), _StubRunner(), memory_store=store)
            info = coord.submit_observation(
                observation={"pose": {"position_xyz_m": [0.0, 0.0, 0.0]}},
                candidates=[
                    {"id": "F1", "xy": [1.1, 0.0]},
                    {"id": "F2", "xy": [7.0, 0.0]},
                ],
                backend_status={},
            )
            self.assertEqual(info["blacklisted_candidates"], 1)
            self.assertEqual(len(captured), 1)
            self.assertEqual(captured[0]["kwargs"]["candidate_ids"], ["F2"])
            self.assertIn("memory_l0", captured[0]["payload"]["observation_summary"])


if __name__ == "__main__":
    unittest.main()
