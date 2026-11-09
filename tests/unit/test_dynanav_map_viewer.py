"""Read-only map validation and isolated publisher contract, without ROS nodes."""

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

SCRIPT = Path(__file__).resolve().parents[2] / "scripts/view_dynanav_map.py"
spec = importlib.util.spec_from_file_location("dynanav_map_viewer", SCRIPT)
viewer = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = viewer
spec.loader.exec_module(viewer)


class Cloud:
    def __init__(self):
        self.header = SimpleNamespace(
            frame_id=None, stamp=SimpleNamespace(sec=None, nanosec=None)
        )


class Field:
    FLOAT32 = 7

    def __init__(self, **values):
        self.__dict__.update(values)


class MapViewerTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / "static_map.npz"
        self.points = np.asarray(
            [[1.25, 2.5, -0.125], [-3.5, 4.75, 6.125]], dtype=np.float64
        )

    def write(self, **changes):
        values = {
            "points": self.points,
            "map_frame": np.asarray("actual_estimated_map"),
        }
        values.update(changes)
        np.savez(self.path, **values)

    def test_actual_points_xyz_and_frame_without_filter_or_fake_acquisition_stamp(self):
        self.write(
            pose_source=np.asarray("post_vio_diagnostic_estimate"),
            semantic_oracle=np.asarray(True),
        )
        source = viewer.load_map(self.path)
        msg = viewer.cloud_message(source, Cloud, Field)
        self.assertEqual(msg.header.frame_id, "actual_estimated_map")
        self.assertEqual((msg.header.stamp.sec, msg.header.stamp.nanosec), (0, 0))
        self.assertEqual(
            (msg.width, msg.height, msg.point_step, msg.row_step), (2, 1, 12, 24)
        )
        self.assertEqual(
            [(f.name, f.offset, f.datatype, f.count) for f in msg.fields],
            [("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1)],
        )
        np.testing.assert_array_equal(
            np.frombuffer(msg.data, dtype="<f4").reshape(-1, 3),
            self.points.astype(np.float32),
        )
        self.assertEqual(
            viewer.summary(source)["metadata"],
            {"pose_source": "post_vio_diagnostic_estimate", "semantic_oracle": True},
        )

    def test_missing_frame_requires_explicit_fallback_existing_metadata_cannot_be_overridden(
        self,
    ):
        np.savez(self.path, points=self.points)
        with self.assertRaisesRegex(ValueError, "provide explicit --frame"):
            viewer.load_map(self.path)
        source = viewer.load_map(self.path, "caller_supplied_map")
        self.assertEqual(source.map_frame, "caller_supplied_map")
        self.assertIn("explicit CLI", source.frame_source)
        self.write()
        for frame in ["actual_estimated_map", "different_map"]:
            with (
                self.subTest(frame=frame),
                self.assertRaisesRegex(ValueError, "only permitted"),
            ):
                viewer.load_map(self.path, frame)
        self.write(map_frame=np.asarray(["invalid_nonscalar"]))
        with self.assertRaises(ValueError):
            viewer.load_map(self.path, "fallback_cannot_replace_invalid_metadata")

    def test_invalid_shape_nonfinite_dtype_and_float32_overflow_rejected(self):
        for points in [
            np.zeros(3),
            np.zeros((2, 4)),
            np.zeros((0, 3)),
            np.asarray([[1, np.nan, 3]]),
            np.asarray([[1, np.inf, 3]]),
            np.asarray([["1", "2", "3"]]),
            np.asarray([[True, False, True]]),
            np.asarray([[1e300, 2.0, 3.0]]),
        ]:
            self.write(points=points)
            with (
                self.subTest(shape=points.shape, dtype=str(points.dtype)),
                self.assertRaises(ValueError),
            ):
                viewer.load_map(self.path)

    def test_no_truth_array_read_and_no_missing_provenance_invented(self):
        self.write(GT_pose=np.asarray([object()], dtype=object))
        source = viewer.load_map(self.path)
        self.assertEqual(source.metadata, {})
        result = viewer.summary(source)
        self.assertNotIn("provenance", result["metadata"])
        self.assertFalse(result["GT_pose_read"])
        self.assertFalse(result["TF_or_control_published"])
        self.assertNotIn("GT_pose", json.dumps(result["metadata"]))

    def test_only_fixed_review_publisher_is_created(self):
        calls = []
        publisher = object()

        class Node:
            def create_publisher(self, kind, topic, qos):
                calls.append((kind, topic, qos))
                return publisher

        qos = object()
        self.assertIs(viewer.static_publisher(Node(), Cloud, qos), publisher)
        self.assertEqual(calls, [(Cloud, "/dynanav/review/static_map", qos)])
        self.assertNotIn("/cmd_vel", [call[1] for call in calls])

    def test_check_never_starts_ROS_and_duration_must_be_finite(self):
        self.write()
        with patch.object(
            viewer, "run_ros", side_effect=AssertionError("no ROS node in --check")
        ):
            self.assertEqual(viewer.main(["--map", str(self.path), "--check"]), 0)
        for duration in ["nan", "inf", "0", "-1"]:
            with self.subTest(duration=duration):
                self.assertEqual(
                    viewer.main(
                        ["--map", str(self.path), "--duration", duration, "--check"]
                    ),
                    2,
                )
        with self.assertRaises(SystemExit):
            viewer.main(["--map", str(self.path), "--topic", "/cmd_vel"])


if __name__ == "__main__":
    unittest.main()
