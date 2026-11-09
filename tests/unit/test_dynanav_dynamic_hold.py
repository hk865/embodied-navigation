"""Sensor-frame experimental gate checks; no simulator or navigation claims."""

import json
import unittest

import numpy as np

from embodied_navigation.simulation.dynanav.dynamic_hold import (
    DynamicHold,
    DynamicHoldConfig,
)


class DynamicHoldTests(unittest.TestCase):
    def guard(self, **parameters):
        return DynamicHold(
            DynamicHoldConfig(**parameters),
            imu_origin_in_base_m=(0.1, 0, 0.2),
            mask_provenance="test_current_frame_mask",
        )

    def update(self, guard, points=(), t=0, **kwargs):
        values = {
            "source_ns": t,
            "mask_source_ns": t,
            "now_ns": t,
            "mask_valid": True,
            "sampled_point_count": 100,
        }
        values.update(kwargs)
        return guard.update(points, **values)

    def test_provisional_recorded_cruise_formula_is_explicit_and_stable_during_hold(
        self,
    ):
        guard = self.guard()
        hold, receipt = self.update(guard, [[0.5, 0, 0]])
        self.assertTrue(hold)
        self.assertAlmostEqual(receipt["stop_distance_from_base_m"], 0.6475)
        self.assertAlmostEqual(sum(receipt["stopping_components_m"].values()), 0.6475)
        self.assertEqual(receipt["parameters"]["recorded_cruise_mps"], 0.15)
        self.assertIn("not_measured_braking", receipt["parameters_status"])
        self.assertFalse(receipt["general_avoidance_verified"])
        json.dumps(receipt, allow_nan=False)

    def test_explicit_sensor_offset_projection_and_input_buffer_preservation(self):
        guard = self.guard()
        points = np.asarray(
            [[0.50, 0.1, 0.0], [0.01, 0.5, 0.0], [0.01, 0.0, 2.0]], dtype=np.float32
        )
        original = points.copy()
        hold, receipt = self.update(guard, points)
        self.assertTrue(hold)
        self.assertEqual(receipt["corridor_points"], 1)
        self.assertAlmostEqual(receipt["nearest_forward_x_m"], 0.6)
        self.assertAlmostEqual(receipt["nearest_body_front_clearance_m"], 0.25)
        np.testing.assert_array_equal(points, original)
        self.assertFalse(receipt["raw_navigation_cloud_modified"])

    def test_stopping_threshold_is_inclusive_without_hidden_point_count(self):
        guard = self.guard()
        distance = guard.config.stop_distance_m - 0.1
        self.assertTrue(self.update(guard, [[distance, 0, 0]])[0])
        hold, _ = self.update(guard, [[distance + 1e-6, 0, 0]], t=100_000_000)
        self.assertTrue(hold)
        hold, receipt = self.update(guard, [[distance + 1e-6, 0, 0]], t=200_000_000)
        self.assertFalse(hold)
        self.assertEqual(receipt["transition"], "resume")

    def test_missing_mismatched_and_stale_masks_cause_unknown_hold(self):
        for extra, reason in [
            ({"mask_valid": False}, "missing_or_invalid_current_mask"),
            ({"mask_source_ns": None}, "missing_or_invalid_current_mask"),
            ({"mask_source_ns": 1}, "mask_timestamp_mismatch"),
            ({"now_ns": 500_000_001}, "stale_observation"),
        ]:
            hold, receipt = self.update(self.guard(), **extra)
            self.assertTrue(hold)
            self.assertEqual(receipt["status"], "unknown")
            self.assertEqual(receipt["reason"], reason)

    def test_clear_requires_distinct_consecutive_frames_and_unknown_resets_evidence(
        self,
    ):
        guard = self.guard()
        self.assertTrue(self.update(guard)[0])
        hold, receipt = self.update(guard)
        self.assertTrue(hold)
        self.assertEqual(receipt["clear_frames"], 1)
        self.assertFalse(receipt["distinct_frame"])
        self.assertTrue(self.update(guard, t=100_000_000, mask_valid=False)[0])
        self.assertTrue(self.update(guard, t=200_000_000)[0])
        hold, receipt = self.update(guard, t=300_000_000)
        self.assertFalse(hold)
        self.assertEqual(receipt["clear_frames"], 2)
        self.assertTrue(self.update(guard, [[0.2, 0, 0]], t=400_000_000)[0])

    def test_decision_between_frames_enforces_freshness_and_does_not_create_clear_evidence(
        self,
    ):
        guard = self.guard()
        self.assertTrue(guard.decision(now_ns=0)[0])
        self.update(guard)
        hold, receipt = guard.decision(now_ns=100_000_000)
        self.assertTrue(hold)
        self.assertEqual(receipt["clear_frames"], 1)
        self.assertFalse(self.update(guard, t=200_000_000)[0])
        self.assertFalse(guard.decision(now_ns=700_000_000)[0])
        hold, receipt = guard.decision(now_ns=700_000_001)
        self.assertTrue(hold)
        self.assertEqual(receipt["reason"], "stale_observation")
        self.assertEqual(receipt["clear_frames"], 0)

    def test_optional_dwell_requires_observed_clear_timestamps(self):
        guard = self.guard(clear_dwell_s=0.3)
        self.assertTrue(self.update(guard)[0])
        self.assertTrue(self.update(guard, t=100_000_000)[0])
        hold, receipt = guard.decision(now_ns=400_000_000)
        self.assertTrue(hold)
        self.assertAlmostEqual(receipt["clear_observed_duration_s"], 0.1)
        self.assertFalse(self.update(guard, t=400_000_000)[0])

    def test_large_clear_frame_gap_restarts_release_evidence(self):
        guard = self.guard()
        self.update(guard)
        hold, receipt = self.update(guard, t=600_000_000)
        self.assertTrue(hold)
        self.assertTrue(receipt["clear_gap_reset"])
        self.assertEqual(receipt["clear_frames"], 1)

    def test_clear_gap_after_resume_requires_release_evidence_again(self):
        guard = self.guard()
        self.update(guard)
        self.assertFalse(self.update(guard, t=100_000_000)[0])
        hold, receipt = self.update(guard, t=700_000_000)
        self.assertTrue(hold)
        self.assertTrue(receipt["clear_gap_reset"])
        self.assertEqual(receipt["clear_frames"], 1)
        self.assertEqual(receipt["transition"], "hold")
        self.assertFalse(self.update(guard, t=800_000_000)[0])

    def test_points_outside_forward_swept_corridor_allow_clear(self):
        guard = self.guard()
        points = [[-1, 0, 0], [0.1, 0.401, 0], [0.1, 0, 2], [0.1, 0, -1]]
        self.update(guard, points)
        hold, receipt = self.update(guard, points, t=100_000_000)
        self.assertFalse(hold)
        self.assertIsNone(receipt["nearest_forward_x_m"])

    def test_invalid_or_absent_points_and_empty_raw_observation_never_clear(self):
        for points, extra in [
            (None, {}),
            ([[float("nan"), 0, 0]], {}),
            ([[0, 0]], {}),
            (np.empty((0, 5)), {}),
            ([], {"sampled_point_count": 0}),
            ([[0.1, 0, 0]], {"sampled_point_count": 0}),
            ([[0.1, 0, 0], [0.2, 0, 0]], {"sampled_point_count": 1}),
        ]:
            hold, receipt = self.update(self.guard(), points, **extra)
            self.assertTrue(hold)
            self.assertEqual(receipt["status"], "unknown")

    def test_future_or_nonmonotonic_times_do_not_silently_clear_or_poison_recovery(
        self,
    ):
        guard = self.guard()
        hold, receipt = self.update(guard, t=100, now_ns=0)
        self.assertTrue(hold)
        self.assertEqual(receipt["reason"], "future_observation")
        self.assertTrue(self.update(guard, t=10)[0])
        self.assertFalse(self.update(guard, t=20)[0])
        hold, receipt = self.update(guard, t=15, now_ns=30)
        self.assertTrue(hold)
        self.assertEqual(receipt["reason"], "nonmonotonic_source_timestamp")
        self.assertTrue(self.update(guard, t=40)[0])
        self.assertFalse(self.update(guard, t=50)[0])

    def test_invalid_configuration_or_missing_explicit_offset_rejected(self):
        for values in (
            {"deceleration_mps2": 0},
            {"clear_frames_required": 0},
            {"clear_frames_required": 1.5},
            {"reaction_time_s": -1},
            {"max_frame_age_s": 0},
            {"corridor_z_min_m": 2},
            {"margin_m": float("nan")},
        ):
            with self.assertRaises(ValueError):
                DynamicHoldConfig(**values)
        with self.assertRaises(ValueError):
            DynamicHold(imu_origin_in_base_m=(0, float("nan"), 0))


if __name__ == "__main__":
    unittest.main()
