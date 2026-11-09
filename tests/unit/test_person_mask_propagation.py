"""Flow geometry is bounded and never becomes a fresh learned/static claim."""

import unittest

import numpy as np

from embodied_navigation.perception.person_mask_propagation import PersonMaskPropagator


class PersonMaskPropagationTests(unittest.TestCase):
    def texture(self):
        rng = np.random.default_rng(17)
        return rng.integers(0, 255, (24, 24, 3), dtype=np.uint8)

    @staticmethod
    def zero(first, second):
        return np.zeros((*first.shape, 2)), np.zeros((*first.shape, 2))

    def seed(self, helper, rgb, stamp=100_000_000):
        mask = np.zeros((1, 24, 24), bool)
        mask[:, 6:18, 6:18] = True
        return helper.update(
            rgb,
            stamp,
            fresh_masks=mask,
            fresh_scores=np.array([0.2]),
            fresh_mask_source_ns=stamp,
        )

    def test_current_original_pixels_shift_and_quality(self):
        rgb = self.texture()

        def translated(first, second):
            forward = np.zeros((*first.shape, 2))
            forward[..., 0] = 2
            return forward, -forward

        helper = PersonMaskPropagator(flow_function=translated)
        self.seed(helper, rgb)
        current = np.roll(rgb, 2, axis=1)
        result = helper.update(current, 200_000_000)
        expected = np.zeros((24, 24), bool)
        expected[6:18, 8:20] = True
        np.testing.assert_array_equal(result.quarantine_mask, expected)
        self.assertFalse(result.withhold)
        self.assertEqual(result.receipt["status"], "quality_gated_adjacent_propagation")
        self.assertFalse(result.receipt["new_semantic_observation"])
        self.assertFalse(result.receipt["background_certified_static"])

    def test_chaining_and_fresh_empty_never_extend_positive_horizon(self):
        rgb = self.texture()
        helper = PersonMaskPropagator(flow_function=self.zero)
        self.seed(helper, rgb)
        for n in range(2, 7):
            result = helper.update(
                rgb,
                n * 100_000_000,
                fresh_masks=np.zeros((0, 24, 24), bool),
                fresh_scores=np.empty(0),
                fresh_mask_source_ns=n * 100_000_000,
            )
            self.assertTrue(result.quarantine_mask.any())
            self.assertEqual(
                result.receipt["anchor_fresh_positive_source_ns"], 100_000_000
            )
        expired = helper.update(rgb, 600_000_001)
        self.assertFalse(expired.quarantine_mask.any())
        self.assertEqual(
            expired.receipt["status"], "positive_candidate_horizon_expired"
        )

    def test_inconsistent_flow_withholds_until_fresh_positive_or_expiry(self):
        rgb = self.texture()

        def inconsistent(first, second):
            forward, backward = self.zero(first, second)
            forward[..., 0] = 8
            return forward, backward

        helper = PersonMaskPropagator(flow_function=inconsistent)
        self.seed(helper, rgb)
        result = helper.update(rgb, 200_000_000)
        self.assertTrue(result.withhold)
        self.assertEqual(result.receipt["quality_fraction"], 0)
        self.assertTrue(helper.update(rgb, 300_000_000).withhold)
        refreshed = self.seed(helper, rgb, 400_000_000)
        self.assertFalse(refreshed.withhold)

    def test_flat_texture_and_missing_adjacency_are_explicit_unknown(self):
        flat = np.zeros((24, 24, 3), np.uint8)
        helper = PersonMaskPropagator(flow_function=self.zero)
        self.seed(helper, flat)
        self.assertTrue(helper.update(flat, 200_000_000).withhold)
        helper = PersonMaskPropagator(flow_function=self.zero)
        self.seed(helper, self.texture())
        result = helper.update(self.texture(), 300_000_000)
        self.assertEqual(
            result.receipt["status"],
            "nonadjacent_or_changed_RGB_within_positive_horizon",
        )
        self.assertTrue(result.withhold)

    def test_wrong_source_or_reversed_RGB_cannot_seed(self):
        helper = PersonMaskPropagator(flow_function=self.zero)
        rgb = self.texture()
        with self.assertRaises(ValueError):
            helper.update(
                rgb,
                100,
                fresh_masks=np.ones((1, 24, 24), bool),
                fresh_scores=np.ones(1),
                fresh_mask_source_ns=99,
            )
        self.seed(helper, rgb)
        with self.assertRaises(ValueError):
            helper.update(rgb, 100_000_000)
        self.assertFalse(
            helper.update(rgb, 200_000_000).receipt["model_inference_performed"]
        )


if __name__ == "__main__":
    unittest.main()
