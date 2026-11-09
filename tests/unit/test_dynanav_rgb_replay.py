"""Original-RGB scheduling contract; fixtures are constructed, no ROS node runs."""

import hashlib
import importlib.util
import json
import signal
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

SCRIPT = Path(__file__).resolve().parents[2] / "scripts/replay_dynanav_rgb.py"
spec = importlib.util.spec_from_file_location("dynanav_rgb_replay", SCRIPT)
replay = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = replay
spec.loader.exec_module(replay)


def sdk_ns(seconds):
    return round(struct.unpack("f", struct.pack("f", seconds))[0] * 1e9)


def message():
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=None, nanosec=None), frame_id=None
        ),
        data=None,
    )


class RgbReplayTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.archive_dir = self.root / "rgb_raw"
        self.archive_dir.mkdir()
        self.pixels = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        self.stamps = [sdk_ns(value) for value in (0.115, 0.215, 0.315)]
        self.args = SimpleNamespace(
            image_topic="/dynanav/camera", clock_topic="/recorded_clock"
        )

    def write(self, index, stamp=None, **changes):
        values = {
            "rgb": self.pixels.copy(),
            "source_ns": np.int64(self.stamps[index] if stamp is None else stamp),
            "frame_index": np.int64(index),
            "frame_id": np.asarray("actual_optical_frame"),
            "encoding": np.asarray("rgb8"),
        }
        values.update(changes)
        path = self.archive_dir / f"{index:06d}.npz"
        np.savez(path, **values)
        return path

    def source(self, count=3):
        for index in range(count):
            self.write(index)
        return replay.load_archive(self.archive_dir)

    def test_exact_pixel_bytes_original_header_and_no_truth_arrays_loaded(self):
        # Loading this object-valued extra array would fail allow_pickle=False.
        path = self.write(0, truth_pose=np.asarray([object()], dtype=object))
        source = replay.load_archive(self.archive_dir)
        frame = source.frames[0]
        result = replay.fill_image(frame, message())
        self.assertEqual(
            result.header.stamp.sec * 10**9 + result.header.stamp.nanosec,
            self.stamps[0],
        )
        self.assertEqual(result.header.frame_id, "actual_optical_frame")
        self.assertEqual(
            (
                result.height,
                result.width,
                result.step,
                result.encoding,
                result.is_bigendian,
            ),
            (2, 3, 9, "rgb8", 0),
        )
        self.assertEqual(result.data, self.pixels.tobytes())
        self.assertEqual(
            frame.rgb_bytes_sha256, hashlib.sha256(result.data).hexdigest()
        )
        self.assertEqual(
            frame.source_sha256, hashlib.sha256(path.read_bytes()).hexdigest()
        )
        buffer = replay.ClockBuffer(source.frames)
        receipt = replay.receipt_document(
            source, source.frames, {}, buffer, self.args, status="validated_check_only"
        )
        self.assertTrue(receipt["NO_GT_fields_used"])
        self.assertEqual(receipt["source_fields_used"], list(replay.FIELDS))
        self.assertNotIn("truth_pose", json.dumps(receipt))

    def test_due_frames_once_in_acquisition_order_repeated_clock_and_future_wait(self):
        source = self.source()
        buffer = replay.ClockBuffer(source.frames, max_images_per_clock=2)
        published = []
        self.assertEqual(buffer.advance(self.stamps[0] - 1, published.append), 0)
        self.assertEqual(buffer.advance(self.stamps[0], published.append), 1)
        self.assertEqual(buffer.advance(self.stamps[0], published.append), 0)
        self.assertEqual(buffer.advance(self.stamps[2] - 1, published.append), 1)
        self.assertEqual(buffer.advance(self.stamps[2], published.append), 1)
        self.assertEqual(published, list(source.frames))
        self.assertTrue(buffer.complete)
        self.assertEqual(buffer.repeated_clocks, 1)
        self.assertEqual(buffer.first_clock_ns, self.stamps[0] - 1)

    def test_clock_reset_burst_and_gap_rejected_without_dropping_or_rewinding(self):
        source = self.source()
        buffer = replay.ClockBuffer(source.frames)
        published = []
        buffer.advance(self.stamps[0], published.append)
        with self.assertRaisesRegex(ValueError, "reset/backward"):
            buffer.advance(self.stamps[0] - 1, published.append)
        self.assertEqual(buffer.published_count, 1)
        self.assertEqual(published, [source.frames[0]])
        burst = replay.ClockBuffer(source.frames, max_images_per_clock=2)
        with self.assertRaisesRegex(ValueError, "due burst"):
            burst.advance(self.stamps[-1], published.append)
        self.assertEqual(burst.published_count, 0)
        gap = replay.ClockBuffer(source.frames, max_clock_gap_ns=1)
        gap.advance(self.stamps[0], lambda frame: None)
        with self.assertRaisesRegex(ValueError, "clock gap"):
            gap.advance(self.stamps[1], lambda frame: None)
        self.assertEqual(gap.published_count, 1)

    def test_no_subscriber_clock_buffer_preserves_frames_when_later_flushed(self):
        source = self.source()
        buffer = replay.ClockBuffer(source.frames)
        for stamp in self.stamps:
            buffer.observe_clock(stamp)
        self.assertEqual(buffer.published_count, 0)
        published = []
        self.assertEqual(buffer.publish_due(published.append), 3)
        self.assertEqual(published, list(source.frames))

    def test_source_filter_is_exact_frozen_subset_and_retains_original_indices(self):
        source = self.source()
        path = self.root / "received.json"
        path.write_text(
            json.dumps({"source_stamps_ns": [self.stamps[2], self.stamps[0]]})
        )
        selected, selection = replay.select_frames(source, path)
        self.assertEqual([f.frame_index for f in selected], [0, 2])
        self.assertEqual(
            [f.stamp_ns for f in selected], [self.stamps[0], self.stamps[2]]
        )
        self.assertEqual(
            selection["filter_sha256"], hashlib.sha256(path.read_bytes()).hexdigest()
        )
        buffer = replay.ClockBuffer(selected)
        buffer.advance(self.stamps[-1], lambda frame: None)
        receipt = replay.receipt_document(
            source, selected, selection, buffer, self.args, status="completed", ack=True
        )
        self.assertTrue(receipt["completion_confirmed"])
        self.assertTrue(receipt["all_scheduled_frames_published"])
        self.assertFalse(receipt["all_archive_frames_published"])
        self.assertEqual(
            (
                receipt["source_archive_count"],
                receipt["scheduled_count"],
                receipt["published_count"],
            ),
            (3, 2, 2),
        )
        for value in (
            [],
            [self.stamps[0]] * 2,
            [self.stamps[0] + 1],
            [float(self.stamps[0])],
            [True],
            {},
        ):
            path.write_text(json.dumps(value))
            with self.subTest(value=value), self.assertRaises(ValueError):
                replay.select_frames(source, path)

    def test_duplicate_backward_missing_index_invalid_header_NaN_and_shape_errors(self):
        self.write(0)
        self.write(1, stamp=self.stamps[0])
        with self.assertRaisesRegex(ValueError, "duplicate"):
            replay.load_archive(self.archive_dir)
        self.write(1, stamp=self.stamps[0] - 1)
        with self.assertRaisesRegex(ValueError, "backward"):
            replay.load_archive(self.archive_dir)
        (self.archive_dir / "000001.npz").unlink()
        self.write(2)
        with self.assertRaisesRegex(ValueError, "consecutive"):
            replay.load_archive(self.archive_dir)
        (self.archive_dir / "000002.npz").unlink()
        variants = [
            {"source_ns": np.asarray(float("nan"))},
            {"source_ns": np.int64(-1)},
            {"frame_index": np.int32(0)},
            {"rgb": np.full((2, 3, 3), np.nan)},
            {"rgb": np.zeros((2, 3), dtype=np.uint8)},
            {"rgb": np.zeros((0, 3, 3), dtype=np.uint8)},
            {"encoding": np.asarray("bgr8")},
            {"frame_id": np.asarray("")},
        ]
        for changes in variants:
            self.write(0, **changes)
            with self.subTest(changes=list(changes)), self.assertRaises(ValueError):
                replay.load_archive(self.archive_dir)
        self.write(0)
        self.write(1, rgb=np.zeros((3, 3, 3), dtype=np.uint8))
        with self.assertRaisesRegex(ValueError, "shape/frame"):
            replay.load_archive(self.archive_dir)

    def test_source_mutated_after_check_rejected_before_publish(self):
        source = self.source(1)
        self.write(0, rgb=np.zeros_like(self.pixels))
        buffer = replay.ClockBuffer(source.frames)
        with self.assertRaisesRegex(ValueError, "changed after validation"):
            buffer.advance(
                self.stamps[0], lambda frame: replay.fill_image(frame, message())
            )
        self.assertEqual(buffer.published_count, 0)

    def test_tail_clock_cancel_and_failed_ack_never_confirm_complete(self):
        source = self.source()
        selected, selection = replay.select_frames(source)
        buffer = replay.ClockBuffer(selected)
        buffer.advance(self.stamps[-1] - 1, lambda frame: None)
        receipt = replay.receipt_document(
            source,
            selected,
            selection,
            buffer,
            self.args,
            status="cancelled",
            error="SIGTERM",
        )
        self.assertFalse(receipt["completion_confirmed"])
        self.assertFalse(receipt["all_scheduled_frames_published"])
        self.assertFalse(receipt["last_clock_reached_final_scheduled_stamp"])
        cancellation = replay.Cancellation()
        cancellation.signal(signal.SIGTERM, None)
        with self.assertRaises(replay.Cancelled):
            cancellation.check()
        buffer.advance(self.stamps[-1], lambda frame: None)
        for status, ack in [
            ("cancelled", True),
            ("failed", False),
            ("validated_check_only", None),
        ]:
            receipt = replay.receipt_document(
                source, selected, selection, buffer, self.args, status=status, ack=ack
            )
            self.assertFalse(receipt["completion_confirmed"])

    def test_check_does_not_start_ROS_and_receipt_never_overwrites(self):
        self.source()
        path = self.root / "check.json"
        args = [
            "--archive-dir",
            str(self.archive_dir),
            "--receipt",
            str(path),
            "--check",
        ]
        with patch.object(
            replay, "run_ros", side_effect=AssertionError("must not start ROS")
        ):
            self.assertEqual(replay.main(args), 0)
        result = json.loads(path.read_text())
        self.assertEqual(result["status"], "validated_check_only")
        self.assertEqual(result["published_count"], 0)
        self.assertFalse(result["completion_confirmed"])
        before = path.read_bytes()
        self.assertEqual(replay.main(args), 2)
        self.assertEqual(before, path.read_bytes())
        with self.assertRaises(FileExistsError):
            replay.write_receipt(path, {})


if __name__ == "__main__":
    unittest.main()
