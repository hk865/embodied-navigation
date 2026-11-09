"""Original-pixel crop lifting and score-ordered mask IoU deduplication."""

import unittest

import numpy as np

from embodied_navigation.perception.person_tiling import (
    inference_views,
    lift_masks,
    merge_mask_candidates,
)


class PersonTilingTests(unittest.TestCase):
    def test_grid2_has_full_plus_four_exact_quadrants(self):
        views = inference_views((5, 7, 3), 2)
        self.assertEqual(len(views), 5)
        coverage = np.zeros((5, 7), int)
        for _, (x0, y0, x1, y1) in views[1:]:
            coverage[y0:y1, x0:x1] += 1
        np.testing.assert_array_equal(coverage, np.ones((5, 7), int))
        self.assertEqual(inference_views((5, 7, 3), 1), [("full", (0, 0, 7, 5))])

    def test_lift_preserves_pixel_mask_and_translates_boxes(self):
        mask = np.zeros((1, 2, 3), bool)
        mask[0, 1, 2] = True
        lifted, boxes = lift_masks(
            mask, np.array([[0.0, 0.0, 3.0, 2.0]]), (3, 2, 6, 4), (5, 7, 3)
        )
        self.assertEqual(np.argwhere(lifted).tolist(), [[0, 3, 5]])
        np.testing.assert_array_equal(boxes, [[3.0, 2.0, 6.0, 4.0]])

    def test_duplicate_mask_keeps_higher_score_with_origin(self):
        masks = np.zeros((3, 4, 4), bool)
        masks[0, :2, :2] = True
        masks[1, :2, :2] = True
        masks[2, 2:, 2:] = True
        parts = [
            {
                "masks": masks,
                "scores": np.array([0.7, 0.9, 0.8]),
                "boxes": np.zeros((3, 4)),
                "view": "tile",
                "crop": (0, 0, 4, 4),
            }
        ]
        kept, scores, _, metadata = merge_mask_candidates(parts, 0.5)
        self.assertEqual(scores.tolist(), [0.9, 0.8])
        self.assertEqual(len(kept), 2)
        self.assertEqual(metadata["retained_origins"][0]["local_instance_index"], 1)
        self.assertEqual(metadata["suppressed"][0]["candidate_index"], 0)


if __name__ == "__main__":
    unittest.main()
