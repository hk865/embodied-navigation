"""Pure replay contract checks; importing the adapter must not require ROS."""

import hashlib
import importlib.util
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[2] / "scripts/replay_dynanav_imu.py"
spec = importlib.util.spec_from_file_location("dynanav_imu_replay", SCRIPT)
replay = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = replay
spec.loader.exec_module(replay)


def row(index=0):
    # Reproduce actual SDK float32 time representation without numpy or ROS.
    seconds = struct.unpack("f", struct.pack("f", 0.02 + index * 0.005))[0]
    return {
        "time": seconds,
        "accel_body": [index + 0.125, -index - 0.25, 9.8125],
        "gyro_body": [index * 0.0001, -0.0002, 0.0003],
        "base_R_world": {"malformed": "must never be used"},
        "sensor_velocity_world": "poison truth",
        "sensor_position_world": None,
        "orientation": [99, 99, 99, 99],
    }


def fake_message():
    vector = lambda: SimpleNamespace(x=123, y=456, z=789)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=None, nanosec=None), frame_id=""
        ),
        linear_acceleration=vector(),
        angular_velocity=vector(),
        orientation=SimpleNamespace(x=1, y=2, z=3, w=4),
        orientation_covariance=[999] * 9,
        angular_velocity_covariance=[999] * 9,
        linear_acceleration_covariance=[999] * 9,
    )


class ImuReplayTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.source_path = self.root / "sdk.jsonl"

    def write_source(self, rows):
        self.source_path.write_text("".join(json.dumps(value) + "\n" for value in rows))
        return replay.load_source(self.source_path)

    def test_all_200_actual_samples_per_second_preserved_at_100_clock_ticks(self):
        rows = [row(index) for index in range(200)]
        source = self.write_source(rows)
        buffer = replay.ClockSourceBuffer(source, max_messages_per_clock=2)
        published = []
        for index in range(1, 200, 2):
            count = buffer.advance(round(rows[index]["time"] * 1e9), published.append)
            self.assertEqual(count, 2)
            self.assertEqual(buffer.advance(buffer.last_clock_ns, published.append), 0)
        self.assertTrue(buffer.complete)
        self.assertEqual(buffer.published_count, 200)
        self.assertEqual(published, list(source.samples))
        self.assertEqual(len({sample.stamp_ns for sample in published}), 200)
        for sample, original in zip(published, rows):
            self.assertEqual(sample.stamp_ns, round(original["time"] * 1e9))
            self.assertEqual(sample.acceleration, tuple(original["accel_body"]))
            self.assertEqual(sample.angular_velocity, tuple(original["gyro_body"]))
        self.assertEqual(
            source.sha256, hashlib.sha256(self.source_path.read_bytes()).hexdigest()
        )
        receipt = replay.receipt_document(source, buffer, status="completed")
        self.assertTrue(receipt["NO_GT_fields_used"])
        self.assertEqual(receipt["largest_published_burst"], 2)
        self.assertEqual(
            receipt["source_fields_used"], ["time", "accel_body", "gyro_body"]
        )
        self.assertNotIn("poison truth", json.dumps(receipt))

    def test_poisoned_truth_values_are_never_read_or_retained(self):
        class SelectedOnly(dict):
            def __getitem__(self, key):
                if key not in replay.SELECTED_FIELDS:
                    raise AssertionError("truth field read")
                return super().__getitem__(key)

        class Poison:
            def __float__(self):
                raise AssertionError("truth value converted")

            def __iter__(self):
                raise AssertionError("truth value iterated")

        original = row()
        original.update(
            base_R_world=Poison(),
            sensor_position_world=Poison(),
            sensor_velocity_world=Poison(),
            orientation=Poison(),
        )
        sample = replay.sample_from_row(SelectedOnly(original))
        self.assertEqual(sample.acceleration, tuple(original["accel_body"]))
        self.assertEqual(
            set(sample.__slots__), {"stamp_ns", "acceleration", "angular_velocity"}
        )
        message = replay.fill_imu_message(sample, fake_message())
        self.assertEqual(message.header.frame_id, "go2_imu")
        self.assertEqual(
            message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
            sample.stamp_ns,
        )
        self.assertEqual(
            vars(message.linear_acceleration), dict(zip("xyz", sample.acceleration))
        )
        self.assertEqual(
            vars(message.angular_velocity), dict(zip("xyz", sample.angular_velocity))
        )
        self.assertEqual(vars(message.orientation), dict.fromkeys("xyzw", 0.0))
        self.assertEqual(message.orientation_covariance, [-1.0] + [0.0] * 8)

    def test_future_sample_waits_and_source_time_is_never_grid_snapped(self):
        source = self.write_source([row(2), row(3)])
        buffer = replay.ClockSourceBuffer(source)
        published = []
        self.assertEqual(
            buffer.advance(source.samples[0].stamp_ns - 1, published.append), 0
        )
        self.assertEqual(
            buffer.advance(source.samples[0].stamp_ns, published.append), 1
        )
        self.assertEqual(len(published), 1)
        self.assertEqual(
            buffer.advance(source.samples[1].stamp_ns - 1, published.append), 0
        )
        self.assertEqual(
            buffer.advance(source.samples[1].stamp_ns, published.append), 1
        )
        self.assertNotEqual(source.samples[0].stamp_ns, 30_000_000)

    def test_reset_and_excessive_clock_jump_fail_without_rewind_or_sample_loss(self):
        source = self.write_source([row(index) for index in range(4)])
        buffer = replay.ClockSourceBuffer(source, max_messages_per_clock=2)
        published = []
        buffer.advance(source.samples[1].stamp_ns, published.append)
        with self.assertRaisesRegex(ValueError, "reset/backward"):
            buffer.advance(source.samples[0].stamp_ns, published.append)
        self.assertEqual(buffer.published_count, 2)
        self.assertEqual(len(published), 2)
        buffer.advance(source.samples[3].stamp_ns, published.append)
        self.assertEqual(published, list(source.samples))
        fresh = replay.ClockSourceBuffer(source, max_messages_per_clock=2)
        with self.assertRaisesRegex(ValueError, "burst bound"):
            fresh.advance(source.samples[-1].stamp_ns, published.append)
        self.assertEqual(fresh.published_count, 0)
        self.assertEqual(len(published), 4)

    def test_clock_can_buffer_until_consumer_ready_without_losing_half_ticks(self):
        source = self.write_source([row(index) for index in range(6)])
        buffer = replay.ClockSourceBuffer(source, max_messages_per_clock=6)
        published = []
        for sample in source.samples:
            buffer.observe_clock(sample.stamp_ns)
        self.assertEqual(buffer.published_count, 0)
        self.assertEqual(buffer.publish_due(published.append), 6)
        self.assertEqual(published, list(source.samples))

    def test_missing_nonfinite_bad_shape_and_invalid_stamps_are_errors(self):
        cases = []
        for field in replay.SELECTED_FIELDS:
            value = row()
            del value[field]
            cases.append(value)
        for field, value in (
            ("time", True),
            ("time", -1),
            ("time", float("nan")),
            ("time", 2**31),
            ("accel_body", [1, 2]),
            ("accel_body", [1, float("inf"), 3]),
            ("gyro_body", [False, 0, 0]),
            ("gyro_body", "000"),
        ):
            value_row = row()
            value_row[field] = value
            cases.append(value_row)
        for value in cases:
            with self.subTest(row=value), self.assertRaises(ValueError):
                self.write_source([value])
        for rows in ([row(1), row(1)], [row(2), row(1)]):
            with (
                self.subTest(rows=rows),
                self.assertRaisesRegex(ValueError, "duplicate or backward"),
            ):
                self.write_source(rows)
        for data in ("", "\n", "{}\n\n", "[1,2,3]\n", "not JSON\n"):
            self.source_path.write_text(data)
            with self.subTest(data=data), self.assertRaises(ValueError):
                replay.load_source(self.source_path)

    def test_check_is_ros_free_and_receipt_is_created_exclusively(self):
        self.write_source([row(0), row(1)])
        receipt = self.root / "new/check.json"
        with patch.object(
            replay, "run_ros", side_effect=AssertionError("ROS must not start")
        ):
            self.assertEqual(
                replay.main(
                    [
                        "--source",
                        str(self.source_path),
                        "--receipt",
                        str(receipt),
                        "--check",
                    ]
                ),
                0,
            )
        document = json.loads(receipt.read_text())
        self.assertEqual(document["status"], "validated_check_only")
        self.assertEqual(document["published_count"], 0)
        self.assertFalse(document["all_source_samples_published"])
        before = receipt.read_bytes()
        self.assertEqual(
            replay.main(
                [
                    "--source",
                    str(self.source_path),
                    "--receipt",
                    str(receipt),
                    "--check",
                ]
            ),
            2,
        )
        self.assertEqual(receipt.read_bytes(), before)
        with self.assertRaises(FileExistsError):
            replay.write_receipt(receipt, {"overwrite": "forbidden"})


if __name__ == "__main__":
    unittest.main()
