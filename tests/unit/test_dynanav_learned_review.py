"""Independent regression cases reproduced during online learned-map review.

Only temporary files and small NumPy fixtures are used: no worker service,
model, ROS process, simulator, renderer or network connection is started.
"""

import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from embodied_navigation.simulation.dynanav.learned_mapping import LearnedMapping
from embodied_navigation.simulation.dynanav.learned_scoring import LearnedScoring


class _Client:
    def __init__(self, *, person=False):
        self.person = person
        self.calls = 0

    def wait_ready(self, **kwargs):
        return True

    def predict(self, rgb, *, source_ns, **kwargs):
        self.calls += 1
        count = int(self.person)
        return SimpleNamespace(
            masks=np.ones((count, *rgb.shape[:2]), dtype=bool),
            scores=np.full(count, 0.9),
            temporal_support=np.zeros(count, dtype=np.int64),
            metadata={"kind": "learned", "producer": "synthetic_review_fixture"},
        )


class _Trigger:
    def __init__(self, *, trigger=True, reject_result=False):
        self.trigger = trigger
        self.reject_result = reject_result

    def decide(self, rgb, **kwargs):
        return {"trigger": self.trigger, "reason": "synthetic_review"}

    def mark_inferred(self, *args, **kwargs):
        if self.reject_result:
            raise ValueError("Returned prediction failed exact-source validation")


class _QuarantineOnly:
    def current_quarantine(self, source_ns):
        return ({"bounds_map": [[0.5, -1, -1], [1.5, 1, 1]]},)


class _MappingFixture(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.packet = SimpleNamespace(
            points=np.array([[1.0, 0.0, 0.0], [1.0, 0.2, 0.0], [1.0, 0.4, 0.0]]),
            pixel_flat_indices=np.array([0, 1, 2]),
        )
        self.rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        self.depth = np.ones((2, 2))

    def mapping(self, name="map", **kwargs):
        return LearnedMapping(
            self.root / name,
            {
                "person_worker": "not-started",
                "person_model_sha256": "synthetic",
                "person_max_interval_s": 0.4,
                "person_min_interval_s": 0.2,
                "map_keyframe_interval_ns": 1_000_000_000,
            },
            client=kwargs.pop("client", _Client()),
            trigger=kwargs.pop("trigger", _Trigger()),
            **kwargs,
        )

    @staticmethod
    def pose(source_ns, position=(0.0, 0.0, 0.0)):
        return {
            "source_ns": source_ns,
            "time": source_ns / 1e9,
            "position": list(position),
            "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
            "pose_source": "livo_post_vio",
            "frame_id": "dynanav_lio_map",
            "child_frame_id": "go2_imu",
        }

    def process(self, mapping, source_ns, **kwargs):
        return mapping.process(
            packet=self.packet,
            source_ns=source_ns,
            rgb=self.rgb,
            depth=self.depth,
            **kwargs,
        )

    @staticmethod
    def context(mapping):
        lines = (mapping.output / "person_context.txt").read_text().splitlines()
        return {
            line.split("=", 1)[0]: json.loads(line.split("=", 1)[1])
            for line in lines
            if line.startswith(("sources=", "coverage="))
        }


class FrontendFailureReviewTests(_MappingFixture):
    def test_active_quarantine_blocks_unscheduled_frame_without_usable_pose(self):
        for name, pose in [
            ("missing", None),
            ("stale", self.pose(100_000_000)),
            ("future", self.pose(400_000_001)),
        ]:
            with self.subTest(name=name):
                client = _Client(person=True)
                mapping = self.mapping(
                    name,
                    client=client,
                    trigger=_Trigger(trigger=False),
                    tracker=_QuarantineOnly(),
                )
                states, keep, receipt, _ = self.process(
                    mapping, 400_000_000, latest_odom=pose
                )
                self.assertEqual(client.calls, 0)
                self.assertFalse(receipt["fresh_inference"])
                self.assertFalse(receipt["quarantine_pose_valid"])
                self.assertFalse(keep.any())
                np.testing.assert_array_equal(states, np.zeros(3, np.uint8))
                self.assertEqual(receipt["unknown_background_points"], 3)
                self.assertEqual(
                    receipt["frontend_withheld_reason"],
                    "active_person_quarantine_without_fresh_estimated_pose",
                )

    def test_unlabelled_bootstrap_does_not_certify_static_background(self):
        client = _Client(person=True)
        mapping = self.mapping(client=client, trigger=_Trigger(trigger=False))
        states, keep, receipt, _ = self.process(mapping, 100_000_000)
        self.assertEqual(client.calls, 0)
        np.testing.assert_array_equal(states, np.zeros(3, np.uint8))
        self.assertTrue(keep.all())
        self.assertFalse(receipt["background_certified_static"])
        self.assertEqual(len(mapping.geometry.confirmed_voxel_points()), 0)

    def test_rejected_returned_person_mask_is_not_fresh_or_consumed(self):
        mapping = self.mapping(
            client=_Client(person=True), trigger=_Trigger(reject_result=True)
        )
        states, keep, receipt, _ = self.process(mapping, 100_000_000)
        self.assertIn("exact-source validation", receipt["model_error"])
        self.assertFalse(receipt["fresh_inference"])
        self.assertIsNone(receipt["model_metadata"])
        self.assertEqual(mapping.model_calls, 0)
        self.assertEqual(mapping.model_errors, 1)
        np.testing.assert_array_equal(states, np.zeros(3, np.uint8))
        self.assertFalse(keep.any())
        mapping.offer_pose(self.pose(100_000_000))
        self.assertEqual(mapping.matched, 1)
        self.assertEqual(len(mapping.store.objects), 0)
        self.assertEqual(mapping.geometry.counts()["retained_supports"], 0)


class ExactPosePendingReviewTests(_MappingFixture):
    def test_late_old_exact_pose_does_not_reenter_monotonic_real_tracker(self):
        mapping = self.mapping()
        for stamp in [100_000_000, 400_000_000]:
            self.process(mapping, stamp)
        mapping.offer_pose(self.pose(400_000_000))
        mapping.offer_pose(self.pose(100_000_000))
        self.assertEqual(mapping.matched, 1)
        self.assertEqual(mapping.unmatched, 1)
        self.assertEqual(mapping.tracker.last_source_ns, 400_000_000)
        self.assertNotIn(100_000_000, mapping.pending)
        self.assertEqual(
            [x["source_ns"] for x in mapping.take_completed()], [400_000_000]
        )
        dropped = [
            json.loads(line)
            for line in (mapping.output / "unmatched.jsonl").read_text().splitlines()
        ]
        self.assertEqual(dropped[-1]["source_ns"], 100_000_000)
        self.assertEqual(dropped[-1]["reason"], "nonmonotonic_exact_pose_match")

    def test_pose_arriving_before_packet_keeps_exact_source_point_ledger(self):
        mapping = self.mapping(trigger=_Trigger(trigger=False))
        stamps = [100_000_000, 400_000_000, 700_000_000]
        for stamp in stamps:
            mapping.offer_pose(self.pose(stamp, position=(1.0, 0.0, 0.0)))
            self.process(mapping, stamp)
        self.assertEqual(mapping.matched, 3)
        self.assertEqual(mapping.pending, {})
        records = mapping.geometry.confirmed_records()
        self.assertEqual(len(records), 3)
        for point_index, record in enumerate(records):
            self.assertEqual(record["representative_source_ns"], stamps[-1])
            self.assertEqual(record["representative_point_index"], point_index)
            self.assertEqual([x["source_ns"] for x in record["supports"]], stamps)
            expected = self.packet.points[point_index] + np.array([1.0, 0.0, 0.0])
            np.testing.assert_array_equal(record["point_map"], expected)


class ScoringLedgerReviewTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.scoring = LearnedScoring(Path(temporary.name) / "score")

    def test_absent_evaluator_segmentation_is_unknown_without_pipeline_exception(self):
        oracle = np.zeros(3, dtype=np.uint8)
        candidates = oracle.copy()
        keep = np.ones(3, dtype=bool)
        self.scoring.observe(
            100,
            oracle,
            candidates,
            keep,
            segmentation=None,
            pixels=np.arange(3),
        )
        result = self.scoring.finish(
            [
                {"representative_source_ns": 100, "representative_point_index": i}
                for i in range(3)
            ]
        )
        self.assertIsNone(result["frontend_person_retained_fraction"])
        self.assertEqual(result["final_geometry_representatives"]["known_labels"], 0)
        self.assertEqual(
            result["final_geometry_representatives"]["unknown_or_unmatched"], 3
        )
        np.testing.assert_array_equal(keep, np.ones(3, dtype=bool))
        np.testing.assert_array_equal(candidates, np.zeros(3, dtype=np.uint8))

    def test_unlabelled_completion_retains_all_admissions_in_unknown_denominator(self):
        self.scoring.observe_completion(
            {
                "source_ns": 123,
                "static_indices": np.arange(7),
                "fresh_inference": False,
                "tracked_ids": ("",) * 7,
                "tracked_states": np.zeros(7, dtype=np.uint8),
            }
        )
        result = self.scoring.finish([])
        self.assertEqual(result["persistent_admissions"]["total"], 7)
        self.assertEqual(
            result["persistent_admissions"]["unknown_or_missing_labels"], 7
        )
        self.assertEqual(result["persistent_admissions"]["oracle_dynamic"], 0)
        missing = json.loads(
            (self.scoring.output / "missing_labels.jsonl").read_text().strip()
        )
        self.assertEqual(missing["admitted_unknown"], 7)

    def test_support_person_contamination_is_not_hidden_by_static_representative(self):
        stamps = [0, 250_000_000, 500_000_000]
        for stamp, label in zip(stamps, [2, 2, 1]):
            self.scoring.observe(
                stamp,
                np.array([label], dtype=np.uint8),
                np.zeros(1, dtype=np.uint8),
                np.ones(1, dtype=bool),
                segmentation={
                    "data": np.array([[7]]),
                    "info": {
                        "idToLabels": {
                            "7": "/World/DynamicPedestrian/Body"
                            if label == 2
                            else "/Floor"
                        }
                    },
                },
                pixels=np.array([0]),
            )
        records = [
            {
                "representative_source_ns": stamps[-1],
                "representative_point_index": 0,
                "supports": [
                    {"source_ns": stamp, "point_index": 0}
                    for stamp in [*stamps, 999_000_000]
                ],
            }
        ]
        before = copy.deepcopy(records)
        result = self.scoring.finish(records)
        self.assertEqual(result["final_geometry_representatives"]["oracle_dynamic"], 0)
        supports = result["final_retained_geometry_supports"]
        self.assertEqual(supports["total"], 4)
        self.assertEqual(supports["oracle_dynamic"], 2)
        self.assertEqual(supports["oracle_person"], 2)
        self.assertEqual(supports["unknown_or_missing_labels"], 1)
        self.assertEqual(supports["voxels_with_any_dynamic_support"], 1)
        self.assertEqual(records, before)


class AgentContextReviewTests(_MappingFixture):
    def test_empty_execution_does_not_claim_observed_exact_pose_or_model_calls(self):
        mapping = self.mapping(trigger=_Trigger(trigger=False))
        receipt = mapping.finish()
        context = self.context(mapping)
        self.assertEqual(receipt["exact_pose_matches"], 0)
        self.assertEqual(receipt["model_calls"], 0)
        self.assertFalse(context["sources"]["exact_live_post_vio_pose"])
        self.assertEqual(context["sources"]["actual_accepted_model_results"], 0)
        self.assertEqual(context["sources"]["exact_pose_matched_frames"], 0)
        self.assertFalse(context["coverage"]["mask_coverage_complete"])

    def test_context_discloses_all_frame_unknowns_pending_and_geometry_candidates(self):
        mapping = self.mapping(trigger=_Trigger(trigger=False))
        for stamp in [100_000_000, 400_000_000, 700_000_000]:
            self.process(mapping, stamp)
            mapping.offer_pose(self.pose(stamp))
        self.process(mapping, 800_000_000)
        mapping.finish()
        context = self.context(mapping)
        sources = context["sources"]
        self.assertEqual(sources["rgbd_stream_frames"], 4)
        self.assertEqual(sources["stream_point_count"], 12)
        self.assertEqual(sources["semantic_unknown_background_points"], 12)
        self.assertEqual(sources["actual_accepted_model_results"], 0)
        self.assertEqual(sources["exact_pose_matched_frames"], 3)
        self.assertEqual(sources["pending_unmatched_frames"], 1)
        self.assertEqual(sources["geometry_candidate_points"], 3)
        self.assertFalse(sources["full_stream_coverage_complete"])
        self.assertFalse(sources["taught_navigation_reference_upstream_oracle_free"])
        self.assertFalse(context["coverage"]["mask_coverage_complete"])
        self.assertFalse(context["coverage"]["semantic_label_coverage_complete"])


if __name__ == "__main__":
    unittest.main()
