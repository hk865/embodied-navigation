"""IPC identity, event decisions and evidence bands without torch/SDK/ROS."""

import json
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np

from embodied_navigation.perception.person_worker import (
    PersonTemporalEvidence,
    PersonTriggerPolicy,
    PersonWorkerClient,
    atomic_json,
    atomic_npz,
)


class PersonWorkerTests(unittest.TestCase):
    def test_event_policy_initial_view_interval_and_untrusted_motion(self):
        rgb = np.zeros((32, 32, 3), np.uint8)
        policy = PersonTriggerPolicy()
        self.assertEqual(policy.decide(rgb, source_ns=0)["reason"], "initial")
        policy.mark_inferred(rgb, source_ns=0)
        self.assertFalse(policy.decide(rgb, source_ns=100_000_000)["trigger"])
        self.assertEqual(
            policy.decide(np.full_like(rgb, 255), source_ns=300_000_000)["reason"],
            "motion_new_view",
        )
        self.assertEqual(
            policy.decide(rgb, source_ns=2_000_000_000)["reason"], "max_interval"
        )
        self.assertEqual(
            policy.decide(
                rgb,
                source_ns=300_000_000,
                estimated_motion_m=0.2,
                motion_source="lio_estimate",
            )["reason"],
            "motion_new_view",
        )
        with self.assertRaises(ValueError):
            policy.decide(
                rgb,
                source_ns=400_000_000,
                estimated_motion_m=0.2,
                motion_source="sim_oracle",
            )
        self.assertEqual(
            policy.last_source_ns, 0
        )  # rejected/not-submitted events do not advance actual inference clock.

    def test_actual_post_vio_stage_is_not_relabelled(self):
        pixels = np.zeros((32, 32, 3), np.uint8)
        policy = PersonTriggerPolicy()
        policy.mark_inferred(pixels, source_ns=0)
        decision = policy.decide(
            pixels,
            source_ns=300_000_000,
            estimated_motion_m=0.2,
            motion_source="livo_post_vio",
        )
        self.assertEqual(decision["reason"], "motion_new_view")
        self.assertEqual(decision["motion_source"], "livo_post_vio")

    def test_temporal_low_support_is_distinct_and_expires(self):
        evidence = PersonTemporalEvidence()
        masks = np.zeros((2, 4, 4), bool)
        masks[0, :2, :2] = True
        masks[1, 2:, 2:] = True
        scores = np.array([0.2, 0.9])
        bands, support = evidence.update(masks, scores, 100)
        self.assertEqual(bands.tolist(), ["low_unconfirmed", "high_confidence"])
        bands, support = evidence.update(masks, scores, 200)
        self.assertEqual(bands.tolist(), ["low_temporal_supported", "high_confidence"])
        self.assertEqual(support.tolist(), [2, 2])
        bands, _ = evidence.update(masks, scores, 2_000_000_000)
        self.assertEqual(bands[0], "low_unconfirmed")

    def test_atomic_result_never_overwrites_existing_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "result.npz"
            atomic_npz(path, value=np.array([1]))
            with self.assertRaises(FileExistsError):
                atomic_npz(path, value=np.array([2]))
            with np.load(path) as data:
                self.assertEqual(data["value"].tolist(), [1])
            self.assertEqual(list(Path(temporary).glob("*.tmp")), [])

    def test_exact_frame_client_and_mismatched_result_quarantine(self):
        for bad in [False, True]:
            with self.subTest(bad=bad), tempfile.TemporaryDirectory() as temporary:
                folder = Path(temporary)
                (folder / "requests").mkdir()
                (folder / "results").mkdir()
                model = "a" * 64
                session = "unit-test-fake-worker"
                atomic_json(
                    folder / "state.json",
                    {"state": "ready", "model_sha256": model, "session_id": session},
                )
                rgb = np.zeros((4, 4, 3), np.uint8)

                def fake_worker(folder=folder, bad=bad, model=model, session=session):
                    deadline = time.monotonic() + 2
                    while time.monotonic() < deadline:
                        requests = list((folder / "requests").glob("*.npz"))
                        if requests:
                            break
                        time.sleep(0.002)
                    request = requests[0]
                    with np.load(request, allow_pickle=False) as data:
                        meta = {
                            "source_ns": int(data["source_ns"]) + (1 if bad else 0),
                            "frame_id": str(data["frame_id"]),
                            "model_sha256": model,
                            "session_id": session,
                            "request_id": request.stem,
                            "rgb_sha256": str(data["rgb_sha256"]),
                            "image_shape": [4, 4, 3],
                            "fresh_inference": True,
                            "inference_index": 1,
                            "test_double": True,
                        }
                    atomic_npz(
                        folder / "results" / (request.stem + ".npz"),
                        masks=np.ones((1, 4, 4), bool),
                        scores=np.array([0.9]),
                        boxes=np.array([[0.0, 0.0, 4.0, 4.0]]),
                        evidence_bands=np.array(["high_confidence"]),
                        temporal_support=np.array([1]),
                        metadata_json=np.asarray(json.dumps(meta)),
                    )
                    request.unlink()
                    (folder / "request.lock").unlink()

                worker = threading.Thread(target=fake_worker)
                worker.start()
                client = PersonWorkerClient(folder, expected_model_sha256=model)
                client.wait_ready(0.5)
                result = client.predict(
                    rgb,
                    source_ns=100,
                    frame_id="camera",
                    reason="unit_test",
                    timeout_s=1,
                )
                worker.join()
                if bad:
                    self.assertIsNone(result)
                    self.assertIn("source_ns", client.last_error["error"])
                else:
                    self.assertEqual(result.source_ns, 100)
                    self.assertEqual(result.masks.shape, (1, 4, 4))

    def test_timeout_returns_unknown_without_old_result(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            (folder / "requests").mkdir()
            (folder / "results").mkdir()
            atomic_json(
                folder / "state.json",
                {"state": "ready", "model_sha256": "a" * 64, "session_id": "test"},
            )
            client = PersonWorkerClient(folder, expected_model_sha256="a" * 64)
            self.assertIsNone(
                client.predict(
                    np.zeros((2, 2, 3), np.uint8),
                    source_ns=1,
                    reason="unit_timeout",
                    timeout_s=0.02,
                )
            )
            self.assertEqual(client.last_error["background"], "UNKNOWN")


if __name__ == "__main__":
    unittest.main()
