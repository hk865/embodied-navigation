"""Observable support, retraction, bounded memory and exact-pose contracts."""

import json
import unittest

import numpy as np

from embodied_navigation.perception.temporal_geometry import TemporalGeometryCache


class TemporalGeometryTests(unittest.TestCase):
    def setUp(self):
        self.point = np.array([[0.01, 0.02, 0.03]])
        self.pose = np.eye(4)

    def update(self, cache, stamp, points=None, **kwargs):
        return cache.update(
            self.point if points is None else points,
            self.pose,
            stamp,
            pose_source_ns=stamp,
            **kwargs,
        )

    def confirmed(self, cache, points=None):
        for stamp in [0, 250_000_000, 500_000_000]:
            result = self.update(cache, stamp, points)
        return result

    def test_three_distinct_frames_and_span_required(self):
        cache = TemporalGeometryCache()
        for stamp in [0, 100_000_000, 200_000_000, 499_999_999]:
            indices, receipt = self.update(
                cache, stamp, np.repeat(self.point, 100, axis=0)
            )
            self.assertEqual(len(indices), 0)
        indices, receipt = self.update(cache, 500_000_000)
        np.testing.assert_array_equal(indices, [0])
        self.assertEqual(receipt["counts"]["confirmed_voxels"], 1)
        self.assertEqual(len(cache.confirmed_records()[0]["supports"]), 5)
        json.dumps(receipt, allow_nan=False)
        json.dumps(cache.confirmed_records(), allow_nan=False)

    def test_repeated_stamp_never_votes_or_readmits_but_can_retract(self):
        cache = TemporalGeometryCache()
        self.update(cache, 0)
        self.update(cache, 0)
        self.assertEqual(cache.counts()["retained_supports"], 1)
        self.update(cache, 250_000_000)
        self.update(cache, 500_000_000)
        indices, receipt = self.update(cache, 500_000_000)
        self.assertEqual(len(indices), 0)
        self.assertTrue(receipt["duplicate_source_no_vote"])
        self.update(cache, 500_000_000, excluded_indices=[0])
        self.assertEqual(len(cache.confirmed_voxel_points()), 0)
        self.assertEqual(cache.counts()["retained_supports"], 0)

    def test_one_dynamic_point_quarantines_entire_voxel_and_ttl_restarts_support(self):
        cache = TemporalGeometryCache(quarantine_ttl_ns=1_000_000_000)
        points = np.array([[0.01, 0.02, 0.03], [0.09, 0.04, 0.05]])
        self.confirmed(cache, points)
        indices, receipt = self.update(cache, 600_000_000, points, excluded_indices=[1])
        self.assertEqual(len(indices), 0)
        self.assertEqual(receipt["quarantined_points"], 2)
        self.assertEqual(receipt["unknown_unadmitted_points"], 1)
        self.update(cache, 1_599_999_999, points)
        self.assertEqual(cache.counts()["candidate_voxels"], 0)
        for stamp in [1_600_000_000, 1_850_000_000]:
            self.assertEqual(len(self.update(cache, stamp, points)[0]), 0)
        self.assertEqual(len(self.update(cache, 2_100_000_000, points)[0]), 2)
        self.assertEqual(
            cache.confirmed_records()[0]["supports"][0]["source_ns"], 1_600_000_000
        )

    def test_map_aabb_retracts_prior_points_even_with_empty_current_cloud(self):
        cache = TemporalGeometryCache()
        self.confirmed(cache)
        box = np.array([[[0.05, 0.05, 0.05], [0.06, 0.06, 0.06]]])
        self.update(cache, 600_000_000, np.empty((0, 3)), person_quarantine=box)
        self.assertEqual(len(cache.confirmed_records()), 0)
        indices, receipt = self.update(cache, 700_000_000)
        self.assertEqual(len(indices), 0)
        self.assertEqual(receipt["unknown_unadmitted_points"], 1)

    def test_world_voxel_uses_measured_transform_and_negative_floor(self):
        cache = TemporalGeometryCache()
        for stamp, translation in [(0, 0.0), (250_000_000, 1.0), (500_000_000, -2.0)]:
            pose = np.eye(4)
            pose[0, 3] = translation
            measured = np.array([[-0.01 - translation, 0.02, 0.03]])
            indices, _ = cache.update(measured, pose, stamp, pose_source_ns=stamp)
        np.testing.assert_array_equal(indices, [0])
        self.assertEqual(cache.confirmed_records()[0]["voxel_key"], [-1, 0, 0])
        np.testing.assert_allclose(
            cache.confirmed_voxel_points(), [[-0.01, 0.02, 0.03]]
        )
        moving = TemporalGeometryCache()
        for stamp, shift in [(0, 0), (250_000_000, 1), (500_000_000, 2)]:
            pose = np.eye(4)
            pose[0, 3] = shift
            indices, _ = moving.update(self.point, pose, stamp, pose_source_ns=stamp)
            self.assertEqual(len(indices), 0)

    def test_retention_removes_support_and_confirmation_is_not_latched(self):
        cache = TemporalGeometryCache(retention_ns=600_000_000)
        self.confirmed(cache)
        indices, receipt = self.update(cache, 700_000_000)
        self.assertEqual(len(indices), 0)
        self.assertEqual(receipt["events"]["retention_expired_supports"], 1)
        self.update(cache, 1_400_000_000, np.empty((0, 3)))
        self.assertEqual(cache.counts()["candidate_voxels"], 0)
        capped = TemporalGeometryCache(max_supports_per_voxel=3)
        self.confirmed(capped)
        self.assertEqual(len(self.update(capped, 600_000_000)[0]), 0)
        self.assertEqual(capped.counts()["cumulative"]["support_capacity_evictions"], 1)

    def test_voxel_capacity_refuses_and_reports_without_overwriting_existing(self):
        cache = TemporalGeometryCache(max_voxels=1)
        self.confirmed(cache)
        _, receipt = self.update(cache, 600_000_000, np.array([[1.0, 0, 0]]))
        self.assertEqual(receipt["events"]["voxel_capacity_rejections"], 1)
        self.assertEqual(receipt["events"]["capacity_rejected_points"], 1)
        self.assertEqual(cache.counts()["candidate_voxels"], 1)
        np.testing.assert_array_equal(cache.confirmed_voxel_points(), self.point)

    def test_quarantine_capacity_fail_closed_removes_unrelated_confirmed(self):
        for kind in ["voxels", "regions"]:
            cache = TemporalGeometryCache(max_quarantine_voxels=1, max_person_regions=1)
            self.confirmed(cache)
            points = np.array([[1.0, 0, 0], [2.0, 0, 0]])
            kwargs = (
                {"excluded_indices": [0, 1]}
                if kind == "voxels"
                else {
                    "person_quarantine": np.array(
                        [[[1, 0, 0], [1, 0, 0]], [[2, 0, 0], [2, 0, 0]]]
                    )
                }
            )
            _, receipt = self.update(cache, 600_000_000, points, **kwargs)
            self.assertEqual(cache.counts()["confirmed_voxels"], 0)
            self.assertGreater(receipt["events"]["global_quarantine_activations"], 0)
            self.assertEqual(len(self.update(cache, 700_000_000)[0]), 0)
            self.assertLessEqual(cache.counts()["quarantine_voxels"], 1)
            self.assertLessEqual(cache.counts()["person_regions"], 1)

    def test_exact_pose_age_and_invalid_input_refused_without_mutation(self):
        cache = TemporalGeometryCache()
        self.confirmed(cache)
        before = cache.confirmed_records()
        bad = [
            {"pose_source_ns": None},
            {"pose_source_ns": 1},
            {"pose_source_ns": 600_000_000, "pose_source": "sim_oracle"},
            {"pose_source_ns": 600_000_000, "now_ns": 1_100_000_001},
            {"pose_source_ns": 600_000_000, "now_ns": 599_999_999},
        ]
        for kwargs in bad:
            with self.assertRaises(ValueError):
                cache.update(self.point, self.pose, 600_000_000, **kwargs)
        with self.assertRaises(ValueError):
            self.update(cache, 499_999_999)
        with self.assertRaises(ValueError):
            self.update(cache, 600_000_000, excluded_indices=[3])
        with self.assertRaises(ValueError):
            self.update(cache, 600_000_000, np.array([[np.nan, 0, 0]]))
        self.assertEqual(cache.confirmed_records(), before)
        _, receipt = self.update(cache, 600_000_000, now_ns=1_100_000_000)
        self.assertTrue(receipt["runtime_age_checked"])

    def test_default_retention_preserves_unrevisited_route_geometry_for_300s(self):
        cache = TemporalGeometryCache()
        self.confirmed(cache)
        self.update(cache, 250_000_000_000, np.empty((0, 3)))
        self.assertEqual(len(cache.confirmed_voxel_points()), 1)
        self.assertEqual(
            cache.confirmed_records()[0]["last_observed_source_ns"], 500_000_000
        )
        _, receipt = self.update(cache, 300_500_000_001, np.empty((0, 3)))
        self.assertEqual(len(cache.confirmed_voxel_points()), 0)
        self.assertEqual(receipt["events"]["retention_expired_supports"], 3)

    def test_malformed_masks_boxes_and_rigid_pose_never_modify_state(self):
        cache = TemporalGeometryCache()
        self.confirmed(cache)
        before = cache.counts()
        for kwargs in [
            {"excluded_indices": np.array([], dtype=bool)},
            {"excluded_indices": [0.0]},
            {"person_quarantine": 1.0},
            {"person_quarantine": [[[1, 0, 0], [0, 0, 0]]]},
        ]:
            with self.assertRaises(ValueError):
                self.update(cache, 600_000_000, **kwargs)
        pose = np.eye(4)
        pose[0, 0] = 2
        with self.assertRaises(ValueError):
            cache.update(self.point, pose, 600_000_000, pose_source_ns=600_000_000)
        self.assertEqual(cache.counts(), before)
        self.update(cache, 600_000_000, excluded_indices=[], person_quarantine=[])

    def test_constructor_refuses_impossible_support_and_memory_bounds(self):
        for kwargs in [
            {"min_distinct_frames": 2},
            {"retention_ns": 499_999_999},
            {"max_supports_per_voxel": 2},
            {"max_voxels": 0},
            {"voxel_size": np.nan},
        ]:
            with self.assertRaises(ValueError):
                TemporalGeometryCache(**kwargs)

    def test_export_preserves_actual_last_point_source_index_and_owned_copies(self):
        cache = TemporalGeometryCache()
        points = np.array([[0.09, 0.08, 0.07], [0.01, 0.02, 0.03]])
        self.confirmed(cache, points)
        record = cache.confirmed_records()[0]
        self.assertEqual(record["representative_source_ns"], 500_000_000)
        self.assertEqual(record["representative_point_index"], 0)
        self.assertEqual([r["point_index"] for r in record["supports"]], [0, 0, 0])
        points[:] = 99
        record["point_map"][:] = [88, 88, 88]
        np.testing.assert_array_equal(
            cache.confirmed_voxel_points(), [[0.09, 0.08, 0.07]]
        )


if __name__ == "__main__":
    unittest.main()
