"""NumPy evidence-contract tests; no model, oracle labels, ROS or simulator."""

import unittest
from unittest.mock import patch

import numpy as np

from embodied_navigation.localization_mapping.semantic_mapping import (
    LayeredMap,
    Observation,
    PointMask,
)
from embodied_navigation.perception.person_geometry_tracking import (
    MotionRegion,
    PersonGeometryTracker,
)

MS = 1_000_000


def points():
    return np.array(
        [
            [0, 0, 0],
            [0.1, 0, 0],
            [9, 0, 0],
            [3, 0, 0],
            [3.1, 0, 0],
            [0, 0.1, 1],
            [0.1, 0.1, 1],
            [9, 0.1, 0],
            [3, 0.1, 1],
            [3.1, 0.1, 1],
        ],
        dtype=float,
    )


def masks():
    result = np.zeros((2, 2, 5), dtype=bool)
    result[0].flat[[0, 1, 5, 6]] = True
    result[1].flat[[3, 4, 8, 9]] = True
    return result


def update(tracker, stamp=0, **overrides):
    values = {
        "masks": masks(),
        "scores": np.array([0.9, 0.8]),
        "mask_source_ns": stamp,
        "produced_ns": stamp,
        "now_ns": stamp,
        "T_map_sensor": np.eye(4),
        "pose_source_ns": stamp,
        "detector_provenance": {"kind": "learned", "producer": "rgb-person-fixture"},
    }
    geometry = overrides.pop("points_sensor", points())
    pixels = overrides.pop("pixel_flat_indices", np.arange(10))
    values.update(overrides)
    return tracker.update(stamp, geometry, pixels, **values)


class PersonGeometryTrackingTests(unittest.TestCase):
    def test_person_dynamic_background_unknown_and_owned_sample_order(self):
        tracker = PersonGeometryTracker()
        cloud, pixel_order = points(), np.array([3, 0, 7, 5, 2, 9, 4, 1, 8, 6])
        result = update(
            tracker, points_sensor=cloud[pixel_order], pixel_flat_indices=pixel_order
        )
        np.testing.assert_array_equal(result.states, [2, 2, 0, 2, 0, 2, 2, 2, 2, 2])
        self.assertEqual(result.instance_ids[0], result.instance_ids[5])
        self.assertNotEqual(result.instance_ids[0], result.instance_ids[1])
        self.assertEqual(result.provenance["instance_id_scope"], "epoch")
        self.assertEqual(result.provenance["kind"], "learned")
        self.assertEqual(result.receipt["static_points"], 0)
        with self.assertRaises(ValueError):
            result.states[0] = 1

    def test_camera_egomotion_and_detection_order_do_not_change_ids(self):
        tracker = PersonGeometryTracker()
        first = update(tracker)
        transform = np.eye(4)
        transform[0, 3] = 0.4
        second = update(
            tracker,
            100 * MS,
            points_sensor=points() - [0.4, 0, 0],
            T_map_sensor=transform,
            masks=masks()[::-1],
            scores=np.array([0.8, 0.9]),
        )
        self.assertEqual(first.instance_ids, second.instance_ids)
        for track in second.active_tracks:
            np.testing.assert_allclose(
                track["visible_centroid_velocity_map"], [0, 0, 0], atol=1e-12
            )
            self.assertFalse(track["motion_evidence"]["rigid_body_motion_verified"])
            self.assertIn("not_rigid_body", track["velocity_semantics"])

    def test_visible_motion_is_measured_and_quarantine_prediction_is_bounded(self):
        tracker = PersonGeometryTracker(max_prediction_distance_m=0.03)
        first = update(tracker)
        moved = points()
        moved[[0, 1, 5, 6], 0] += 0.2
        second = update(tracker, 1000 * MS, points_sensor=moved)
        self.assertEqual(first.instance_ids[0], second.instance_ids[0])
        person = next(
            track
            for track in second.active_tracks
            if track["track_id"] == second.instance_ids[0]
        )
        np.testing.assert_allclose(person["visible_centroid_velocity_map"], [0.2, 0, 0])
        volume = next(
            volume
            for volume in tracker.current_quarantine(1400 * MS)
            if volume["track_id"] == second.instance_ids[0]
        )
        self.assertAlmostEqual(np.linalg.norm(volume["predicted_translation_m"]), 0.03)
        self.assertEqual(tracker.current_quarantine(1500 * MS + 1), ())
        self.assertEqual(len(tracker.query(2000 * MS)), 2)

    def test_occlusion_keeps_id_until_ttl_but_never_creates_world_box_dynamic_mask(
        self,
    ):
        tracker = PersonGeometryTracker()
        first = update(tracker)
        missing = update(
            tracker, 200 * MS, masks=None, scores=None, mask_source_ns=None
        )
        np.testing.assert_array_equal(missing.states, np.zeros(10))
        self.assertEqual(len(missing.quarantine_volumes), 2)
        expired_volume = update(
            tracker, 1000 * MS, masks=None, scores=None, mask_source_ns=None
        )
        self.assertEqual(expired_volume.quarantine_volumes, ())
        self.assertEqual(len(expired_volume.active_tracks), 2)
        return_seen = update(tracker, 2000 * MS)
        self.assertEqual(first.instance_ids, return_seen.instance_ids)
        after_ttl = update(tracker, 5100 * MS)
        self.assertNotEqual(first.instance_ids[0], after_ttl.instance_ids[0])

    def test_causal_or_stale_pose_is_disclosed_and_does_not_claim_exact_motion(self):
        tracker = PersonGeometryTracker()
        update(tracker)
        result = update(tracker, 100 * MS, pose_source_ns=0)
        self.assertEqual(
            result.receipt["pose"],
            {"status": "causal", "source_ns": 0, "age_ns": 100 * MS},
        )
        self.assertTrue(
            all(
                track["visible_centroid_velocity_map"] is None
                for track in result.active_tracks
            )
        )
        self.assertTrue(
            all(
                not track["motion_evidence"]["available"]
                for track in result.active_tracks
            )
        )
        self.assertTrue(
            all(
                volume["pose_status"] == "causal"
                for volume in result.quarantine_volumes
            )
        )
        self.assertTrue(
            all(
                volume["valid_until_ns"] == 500 * MS
                for volume in result.quarantine_volumes
            )
        )
        stale = update(tracker, 600 * MS, pose_source_ns=0)
        self.assertEqual(stale.receipt["pose"]["status"], "stale")
        self.assertEqual(stale.quarantine_volumes, ())
        self.assertFalse(np.any(stale.states == 1))

    def test_wrong_source_or_expired_mask_is_unknown_and_does_not_refresh_track(self):
        for fields, reason in [
            ({"mask_source_ns": 0}, "mask_source_mismatch"),
            ({"produced_ns": 99 * MS}, "mask_production_time_invalid"),
            ({"now_ns": 700 * MS}, "mask_expired"),
        ]:
            with self.subTest(reason=reason):
                tracker = PersonGeometryTracker()
                first = update(tracker)
                result = update(tracker, 100 * MS, **fields)
                self.assertEqual(result.receipt["mask_status"], reason)
                self.assertTrue(np.all(result.states == 0))
                self.assertEqual(
                    first.active_tracks[0]["last_detection_ns"],
                    result.active_tracks[0]["last_detection_ns"],
                )
                self.assertEqual(tracker.current_quarantine(700 * MS), ())

    def test_low_confidence_needs_actual_geometry_and_temporal_support(self):
        tracker = PersonGeometryTracker()
        low = update(tracker, scores=np.array([0.2, 0.2]))
        self.assertTrue(np.all(low.states == 0))
        self.assertTrue(all(not track["confirmed"] for track in low.active_tracks))
        supported = update(tracker, 100 * MS, scores=np.array([0.2, 0.2]))
        self.assertEqual(np.count_nonzero(supported.states == 2), 8)
        self.assertTrue(
            all(
                row["band"] == "low_geometry_temporal_supported"
                for row in supported.receipt["detections"]
            )
        )
        no_pose = PersonGeometryTracker()
        first = update(
            no_pose, scores=np.array([0.2, 0.2]), T_map_sensor=None, pose_source_ns=None
        )
        second = update(
            no_pose,
            100 * MS,
            scores=np.array([0.2, 0.2]),
            T_map_sensor=None,
            pose_source_ns=None,
            temporal_support=np.array([99, 99]),
        )
        self.assertTrue(np.all(first.states == 0) and np.all(second.states == 0))

    def test_ambiguous_3d_candidates_are_not_silently_merged(self):
        tracker = PersonGeometryTracker()
        first = update(tracker)
        combined = np.ones((1, 2, 5), dtype=bool)
        combined[0].flat[[2, 7]] = False
        centered = points()
        centered[[0, 1, 5, 6, 3, 4, 8, 9], 0] = 1.55
        result = update(
            tracker,
            1000 * MS,
            points_sensor=centered,
            masks=combined,
            scores=np.array([0.9]),
        )
        self.assertEqual(result.receipt["detections"][0]["association"], "ambiguous")
        self.assertIsNone(result.receipt["detections"][0]["track_id"])
        self.assertNotIn(result.instance_ids[0], set(first.instance_ids))
        self.assertEqual(len(result.active_tracks), 2)

    def test_motion_regions_are_current_2d_evidence_with_unextendable_horizon(self):
        tracker = PersonGeometryTracker()
        first = update(tracker)
        identity = first.instance_ids[0]
        mask = masks()[0]
        region = MotionRegion(identity, 0, 100 * MS, mask, 0.9)
        predicted = update(
            tracker,
            100 * MS,
            masks=None,
            scores=None,
            mask_source_ns=None,
            motion_regions=(region,),
        )
        self.assertEqual(np.count_nonzero(predicted.states == 2), 4)
        self.assertFalse(
            predicted.provenance["motion_propagation"][0]["current_model_mask"]
        )
        chained = MotionRegion(identity, 100 * MS, 400 * MS, mask, 0.9)
        result = update(
            tracker,
            400 * MS,
            masks=None,
            scores=None,
            mask_source_ns=None,
            motion_regions=(chained,),
        )
        self.assertEqual(np.count_nonzero(result.states == 2), 4)
        self.assertEqual(
            next(
                track for track in result.active_tracks if track["track_id"] == identity
            )["last_detection_ns"],
            0,
        )
        expired = MotionRegion(identity, 400 * MS, 600 * MS, mask, 0.9)
        result = update(
            tracker,
            600 * MS,
            masks=None,
            scores=None,
            mask_source_ns=None,
            motion_regions=(expired,),
        )
        self.assertTrue(np.all(result.states == 0))
        self.assertEqual(
            result.receipt["motion_regions"][0]["status"], "motion_horizon_expired"
        )
        self.assertEqual(result.quarantine_volumes, ())

    def test_low_quality_foreign_identity_and_flow_overlap_remain_unknown(self):
        for quality, identity_kind, reason in (
            (0.2, "known", "motion_quality_low"),
            (0.9, "foreign", "unknown_internal_track"),
        ):
            tracker = PersonGeometryTracker()
            first = update(tracker)
            identity = (
                first.instance_ids[0]
                if identity_kind == "known"
                else "/World/not-an-ID-source"
            )
            region = MotionRegion(identity, 0, 100 * MS, masks()[0], quality)
            result = update(
                tracker,
                100 * MS,
                masks=None,
                scores=None,
                mask_source_ns=None,
                motion_regions=(region,),
            )
            self.assertTrue(np.all(result.states == 0))
            self.assertEqual(result.receipt["motion_regions"][0]["status"], reason)
        tracker = PersonGeometryTracker()
        first = update(tracker)
        regions = tuple(
            MotionRegion(identity, 0, 100 * MS, masks()[0], 0.9)
            for identity in (first.instance_ids[0], first.instance_ids[3])
        )
        result = update(
            tracker,
            100 * MS,
            masks=None,
            scores=None,
            mask_source_ns=None,
            motion_regions=regions,
        )
        self.assertTrue(np.all(result.states == 0))

    def test_layered_map_uses_stable_internal_ids_and_cannot_gain_static_background(
        self,
    ):
        tracker = PersonGeometryTracker()
        store = LayeredMap(map_id="learned-tracks")
        for sequence, stamp in ((1, 0), (2, 100 * MS)):
            evidence = update(tracker, stamp)
            observation = Observation(
                1,
                sequence,
                stamp,
                stamp,
                points(),
                np.eye(4),
                str(sequence),
                "go2_imu",
                "rgbd:v1",
            )
            proof = PointMask(
                1,
                sequence,
                stamp,
                stamp,
                "go2_imu",
                "rgbd:v1",
                evidence.states,
                evidence.provenance,
                evidence.instance_ids,
                evidence.labels,
            )
            store.integrate(observation, proof)
        self.assertEqual(len(store.objects), 2)
        self.assertEqual(len(store.static_points()), 0)
        self.assertTrue(
            all(len(obj["history"]) == 2 for obj in store.query()["objects"])
        )
        self.assertFalse(store.summary()["provenance_summary"]["sim_oracle_used"])

    def test_input_ownership_and_no_gt_oracle_array_or_id_inputs(self):
        tracker = PersonGeometryTracker()
        images, cloud, transform = masks(), points(), np.eye(4)
        with patch("numpy.load", side_effect=AssertionError("archive or oracle read")):
            result = update(
                tracker, masks=images, points_sensor=cloud, T_map_sensor=transform
            )
        center = result.active_tracks[0]["visible_centroid_map"][:]
        images[:] = False
        cloud[:] = 999
        transform[:] = 999
        self.assertEqual(tracker.query(0)[0]["visible_centroid_map"], center)
        result.active_tracks[0]["visible_centroid_map"][:] = [99, 99, 99]
        self.assertEqual(tracker.query(0)[0]["visible_centroid_map"], center)
        for fields in (
            {"pose_source": "sim_oracle"},
            {"detector_provenance": {"kind": "sim_oracle"}},
        ):
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                update(PersonGeometryTracker(), **fields)
        with self.assertRaises(TypeError):
            tracker.update(
                100 * MS, points(), np.arange(10), instance_ids=["oracle"] * 10
            )

    def test_timestamps_shapes_capacity_and_nonrigid_poses_fail_closed(self):
        tracker = PersonGeometryTracker(max_tracks=1)
        result = update(tracker)
        self.assertEqual(len(result.active_tracks), 1)
        self.assertEqual(np.count_nonzero(result.states == 2), 8)
        for stamp in (0, -1):
            with self.assertRaises(ValueError):
                update(tracker, stamp)
        for fields in (
            {"pose_source_ns": 1},
            {"T_map_sensor": np.zeros((4, 4))},
            {"pixel_flat_indices": np.zeros(10, dtype=int)},
            {"image_shape": (3, 3)},
        ):
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                update(PersonGeometryTracker(), **fields)
        for fields in (
            {"track_ttl_ns": 1, "quarantine_horizon_ns": 2},
            {"high_score_threshold": True},
            {"max_tracks": 0},
        ):
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                PersonGeometryTracker(**fields)


if __name__ == "__main__":
    unittest.main()
