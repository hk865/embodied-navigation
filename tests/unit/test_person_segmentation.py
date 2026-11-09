"""Pixel mapping, sparse scoring and schedule tests require no learned model."""

import tempfile
import unittest
from pathlib import Path

import numpy as np

from embodied_navigation.perception.person_segmentation import (
    MaskIoUTracker,
    confusion,
    load_evaluation_frame,
    metric_rates,
    periodic_indices,
    project_masks,
)


class PersonSegmentationTests(unittest.TestCase):
    def test_projection_keeps_original_pixel_and_sample_order(self):
        masks = np.zeros((2, 2, 3), bool)
        masks[0].flat[[0, 5]] = True
        masks[1].flat[3] = True
        np.testing.assert_array_equal(
            project_masks(masks, np.array([5, 0, 3])),
            [[True, True, False], [False, False, True]],
        )
        self.assertEqual(
            project_masks(np.zeros((0, 2, 3), bool), np.array([5, 0, 3])).shape, (0, 3)
        )

    def test_unknown_is_ignored_and_absent_person_is_not_static_prediction(self):
        c = confusion(
            np.array([True, True, False, False, True]), np.array([2, 1, 2, 1, 0])
        )
        self.assertEqual(c, {"tp": 1, "fp": 1, "fn": 1, "tn": 1, "ignored_unknown": 1})
        self.assertEqual(metric_rates(c)["iou"], 1 / 3)
        rates = metric_rates(confusion(np.zeros(2, bool), np.array([1, 0])))
        self.assertIsNone(rates["precision"])
        self.assertIsNone(rates["recall"])
        self.assertIsNone(rates["iou"])

    def test_real_timestamp_schedule_is_anchored_and_never_propagates(self):
        stamps = [
            115000002,
            1115000010,
            2115000010,
            3115000010,
            4215000153,
            5215000153,
            6215000153,
        ]
        self.assertEqual(periodic_indices(stamps), [0, 2, 4, 6])
        self.assertEqual(periodic_indices([10, 19, 35, 51], 10), [0, 2, 3])
        with self.assertRaises(ValueError):
            periodic_indices([1, 1])

    def test_adjacent_2d_ids_are_one_to_one_and_reset_after_no_detection(self):
        tracker = MaskIoUTracker(0.2)
        m = np.zeros((1, 4, 4), bool)
        m[0, 1:3, 1:3] = True
        self.assertEqual(tracker.update(m, 1), [1])
        self.assertEqual(tracker.update(m, 2), [1])
        self.assertEqual(tracker.update(np.zeros((0, 4, 4), bool), 3), [])
        self.assertEqual(tracker.update(m, 4), [2])
        with self.assertRaises(ValueError):
            tracker.update(m, 4)

    def test_loader_does_not_read_pose_and_rejects_nonperson_dynamic_truth(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "rgbd_000001.npz"
            values = {
                "rgb": np.zeros((2, 3, 3), np.uint8),
                "source_ns": np.int64(123),
                "pixel_flat_indices": np.array([5, 0, 3]),
                "states": np.array([2, 1, 0], np.uint8),
                "instance_paths": np.array(
                    ["/World/DynamicPedestrian/Human", "/Wall", ""]
                ),
                "T_map_sensor": np.array([object()], dtype=object),
            }
            np.savez(path, **values)
            frame = load_evaluation_frame(path)
            self.assertEqual(frame.source_ns, 123)
            np.testing.assert_array_equal(frame.pixel_flat_indices, [5, 0, 3])
            values["instance_paths"] = np.array(["/World/OtherMover", "/Wall", ""])
            np.savez(path, **values)
            with self.assertRaisesRegex(ValueError, "non-person"):
                load_evaluation_frame(path)


if __name__ == "__main__":
    unittest.main()
