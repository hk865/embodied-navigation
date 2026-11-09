"""Check the boundary between online inference, exact poses and oracle scoring."""

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from embodied_navigation.simulation.dynanav.learned_mapping import LearnedMapping
from embodied_navigation.simulation.dynanav.learned_scoring import LearnedScoring


class Client:
    def wait_ready(self, **kwargs):
        return True

    def predict(self, rgb, **kwargs):
        return SimpleNamespace(
            masks=np.zeros((0, *rgb.shape[:2]), bool),
            scores=np.zeros(0),
            temporal_support=None,
            metadata={"actual_model": False},
        )


class Trigger:
    def decide(self, rgb, **kwargs):
        return {"trigger": True, "reason": "test"}

    def mark_inferred(self, *args, **kwargs):
        pass


class Tracker:
    def current_quarantine(self, source_ns):
        return ()

    def update(self, source_ns, points, pixels, **kwargs):
        assert kwargs["pose_source_ns"] == source_ns
        assert kwargs["pose_source"] == "slam"
        return SimpleNamespace(
            states=np.zeros(len(points), np.uint8),
            instance_ids=tuple("" for _ in points),
            labels={},
            quarantine_volumes=(),
            provenance={
                "kind": "learned",
                "producer": "test",
                "instance_id_scope": "epoch",
            },
            receipt={},
        )


class OnlineMappingBoundaryTests(unittest.TestCase):
    def profile(self):
        return {
            "person_worker": "unused",
            "person_model_sha256": "test",
            "person_max_interval_s": 0.4,
            "person_min_interval_s": 0.2,
            "map_keyframe_interval_ns": 1_000_000_000,
        }

    def packet(self):
        return SimpleNamespace(
            points=np.array([[1.0, 0.0, 0.0], [1.0, 0.2, 0.0], [1.0, 0.4, 0.0]]),
            pixel_flat_indices=np.array([0, 1, 2]),
        )

    def pose(self, ns, **extra):
        result = {
            "source_ns": ns,
            "time": ns / 1e9,
            "position": [0.0, 0.0, 0.0],
            "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
            "pose_source": "livo_post_vio",
            "frame_id": "dynanav_lio_map",
            "child_frame_id": "go2_imu",
        }
        result.update(extra)
        return result

    def test_geometry_waits_for_exact_post_pose_and_rejects_wrong_frame(self):
        with tempfile.TemporaryDirectory() as temporary:
            mapping = LearnedMapping(
                Path(temporary) / "map",
                self.profile(),
                client=Client(),
                trigger=Trigger(),
                tracker=Tracker(),
            )
            packet = self.packet()
            for ns in (100_000_000, 400_000_000, 700_000_000):
                mapping.process(
                    packet=packet,
                    source_ns=ns,
                    rgb=np.zeros((2, 2, 3), np.uint8),
                    depth=np.ones((2, 2)),
                )
                mapping.offer_pose(self.pose(ns, frame_id="wrong"))
                self.assertIn(ns, mapping.pending)
                self.assertEqual(
                    mapping.geometry.confirmed_voxel_points().shape, (0, 3)
                )
                mapping.offer_pose(self.pose(ns))
            self.assertEqual(mapping.matched, 3)
            self.assertEqual(len(mapping.geometry.confirmed_voxel_points()), 3)
            self.assertEqual(len(mapping.take_completed()), 3)
            with np.load(next((mapping.output / "keyframes").glob("*.npz"))) as frame:
                self.assertFalse(frame["pose_oracle"].item())
                self.assertFalse(frame["semantic_oracle"].item())
                np.testing.assert_array_equal(frame["T_map_sensor"], np.eye(4))

    def test_last_pose_does_not_fill_an_unmatched_persistent_observation(self):
        with tempfile.TemporaryDirectory() as temporary:
            mapping = LearnedMapping(
                Path(temporary) / "map",
                self.profile(),
                client=Client(),
                trigger=Trigger(),
                tracker=Tracker(),
            )
            mapping.process(
                packet=self.packet(),
                source_ns=200_000_000,
                rgb=np.zeros((2, 2, 3), np.uint8),
                depth=np.ones((2, 2)),
                latest_odom=self.pose(100_000_000),
            )
            mapping.offer_pose(self.pose(100_000_000))
            self.assertEqual(mapping.matched, 0)
            self.assertEqual(mapping.geometry.confirmed_voxel_points().shape, (0, 3))

    def test_failed_model_is_unknown_and_does_not_become_a_cache_hit(self):
        class Failed(Client):
            def predict(self, *args, **kwargs):
                raise TimeoutError("test worker deadline")

        with tempfile.TemporaryDirectory() as temporary:
            mapping = LearnedMapping(
                Path(temporary) / "map",
                self.profile(),
                client=Failed(),
                trigger=Trigger(),
                tracker=Tracker(),
            )
            states, keep, receipt, _ = mapping.process(
                packet=self.packet(),
                source_ns=100,
                rgb=np.zeros((2, 2, 3), np.uint8),
                depth=np.ones((2, 2)),
            )
            self.assertFalse(receipt["fresh_inference"])
            self.assertEqual(mapping.model_calls, 0)
            np.testing.assert_array_equal(states, 0)
            self.assertFalse(keep.any())
            mapping.offer_pose(self.pose(100))
            self.assertEqual(len(mapping.geometry.confirmed_voxel_points()), 0)

    def test_late_exact_pose_is_logged_instead_of_reentering_tracker(self):
        with tempfile.TemporaryDirectory() as temporary:
            mapping = LearnedMapping(
                Path(temporary) / "map",
                self.profile(),
                client=Client(),
                trigger=Trigger(),
                tracker=Tracker(),
            )
            for ns in (100_000_000, 400_000_000):
                mapping.process(
                    packet=self.packet(),
                    source_ns=ns,
                    rgb=np.zeros((2, 2, 3), np.uint8),
                    depth=np.ones((2, 2)),
                )
            mapping.offer_pose(self.pose(400_000_000))
            mapping.offer_pose(self.pose(100_000_000))
            self.assertEqual(mapping.matched, 1)
            self.assertEqual(mapping.last_integrated_ns, 400_000_000)
            self.assertIn(
                "nonmonotonic_exact_pose_match",
                (mapping.output / "unmatched.jsonl").read_text(),
            )

    def test_active_quarantine_with_stale_pose_withholds_frontend_candidates(self):
        class Boxes(Tracker):
            def current_quarantine(self, source_ns):
                return ({"bounds_map": [[0.9, -1, -1], [1.1, 1, 1]]},)

        with tempfile.TemporaryDirectory() as temporary:
            mapping = LearnedMapping(
                Path(temporary) / "map",
                self.profile(),
                client=Client(),
                trigger=Trigger(),
                tracker=Boxes(),
            )
            _states, keep, receipt, _ = mapping.process(
                packet=self.packet(),
                source_ns=400_000_000,
                rgb=np.zeros((2, 2, 3), np.uint8),
                depth=np.ones((2, 2)),
                latest_odom=self.pose(100_000_000),
            )
            self.assertFalse(keep.any())
            self.assertFalse(receipt["quarantine_pose_valid"])
            self.assertEqual(
                receipt["frontend_withheld_reason"],
                "active_person_quarantine_without_fresh_estimated_pose",
            )

    def test_scheduler_failure_cannot_retain_a_successful_prediction(self):
        class BadTrigger(Trigger):
            def mark_inferred(self, *args, **kwargs):
                raise ValueError("test mark failed")

        with tempfile.TemporaryDirectory() as temporary:
            mapping = LearnedMapping(
                Path(temporary) / "map",
                self.profile(),
                client=Client(),
                trigger=BadTrigger(),
                tracker=Tracker(),
            )
            _, keep, receipt, _ = mapping.process(
                packet=self.packet(),
                source_ns=100,
                rgb=np.zeros((2, 2, 3), np.uint8),
                depth=np.ones((2, 2)),
            )
            self.assertFalse(receipt["fresh_inference"])
            self.assertIsNotNone(receipt["model_error"])
            self.assertFalse(keep.any())
            self.assertEqual(mapping.model_calls, 0)

    def test_scoring_reports_person_leakage_without_changing_algorithm_outputs(self):
        with tempfile.TemporaryDirectory() as temporary:
            scoring = LearnedScoring(Path(temporary) / "score")
            oracle = np.array([2, 2, 1], np.uint8)
            candidates = np.array([2, 0, 0], np.uint8)
            keep = np.array([False, True, True])
            scoring.observe(
                100,
                oracle,
                candidates,
                keep,
                segmentation={
                    "data": np.array([[7, 7, 8]]),
                    "info": {
                        "idToLabels": {
                            "7": "/World/DynamicPedestrian/Body",
                            "8": "/Floor",
                        }
                    },
                },
                pixels=np.array([0, 1, 2]),
            )
            scoring.observe_completion(
                {
                    "source_ns": 100,
                    "static_indices": np.array([1, 2]),
                    "fresh_inference": True,
                    "tracked_ids": ("person:1", "", ""),
                    "tracked_states": np.array([2, 0, 0]),
                }
            )
            result = scoring.finish(
                [
                    {"representative_source_ns": 100, "representative_point_index": 1},
                    {"representative_source_ns": 100, "representative_point_index": 2},
                ]
            )
            self.assertEqual(result["frontend_person_retained_fraction"], 0.5)
            self.assertEqual(
                result["final_geometry_representatives"]["oracle_person"], 1
            )
            self.assertEqual(result["persistent_admissions"]["oracle_person"], 1)
            np.testing.assert_array_equal(keep, [False, True, True])
            np.testing.assert_array_equal(candidates, [2, 0, 0])


if __name__ == "__main__":
    unittest.main()
