"""Measured-reference evidence tests without ROS or simulator dependencies."""

import hashlib
import importlib.util
import json
import math
import sqlite3
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[2] / "scripts/build_dynanav_slam_route.py"
spec = importlib.util.spec_from_file_location("dynanav_slam_route_builder", SCRIPT)
builder = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = builder
spec.loader.exec_module(builder)


def message(
    stamp_ns=0,
    xyz=(0, 0, 0),
    quaternion=(0, 0, 0, 1),
    frame=builder.MAP_FRAME,
    child=builder.SENSOR_FRAME,
    velocity=(0.08, 0, 0),
):
    vec = lambda names, values: SimpleNamespace(**dict(zip(names, values)))

    class PoisonNative:
        def __getattr__(self, name):
            raise AssertionError("native/truth input must never be accessed")

    result = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000
            ),
            frame_id=frame,
        ),
        child_frame_id=child,
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=vec("xyz", xyz), orientation=vec("xyzw", quaternion)
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=vec("xyz", velocity), angular=vec("xyz", (0, 0, 0.001))
            )
        ),
        native_pose=PoisonNative(),
        T_map_sensor=PoisonNative(),
        truth=PoisonNative(),
    )
    return result


class BuilderTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.database = self.root / "bag.db3"
        self.messages = [message(0), message(1_000_000_000, (2, 0, 0.2))]
        self.messages += [
            message(stamp, (2 + (stamp % 3) * 0.001, 0, 0.2))
            for stamp in (1_250_000_000, 1_500_000_000, 1_750_000_000, 2_000_000_000)
        ]
        self.args = SimpleNamespace(
            bag=self.database,
            output=self.root / "reference.json",
            odom_topic=builder.ODOM_TOPIC,
            map_frame=builder.MAP_FRAME,
            sensor_frame=builder.SENSOR_FRAME,
            goal_stamp_ns=None,
            stop_dwell_s=1.0,
            stop_position_range_m=0.03,
            stop_yaw_range_rad=0.03,
            stop_max_sample_gap_s=0.25,
        )
        self.write_database()

    def write_database(self):
        if self.database.exists():
            self.database.unlink()
        with sqlite3.connect(self.database) as connection:
            connection.execute(
                "CREATE TABLE topics(id INTEGER,name TEXT,type TEXT,serialization_format TEXT)"
            )
            connection.execute(
                "CREATE TABLE messages(id INTEGER,topic_id INTEGER,timestamp INTEGER,data BLOB)"
            )
            connection.execute(
                "INSERT INTO topics VALUES(1,?, 'nav_msgs/msg/Odometry','cdr')",
                (builder.ODOM_TOPIC,),
            )
            connection.execute(
                "INSERT INTO topics VALUES(2,'/native/truth','nav_msgs/msg/Odometry','cdr')"
            )
            for index, _ in enumerate(self.messages):
                connection.execute(
                    "INSERT INTO messages VALUES(?,?,?,?)",
                    (index + 1, 1, index * 1000, str(index).encode()),
                )
            connection.execute(
                "INSERT INTO messages VALUES(999,2,0,?)", (b"poisoned truth blob",)
            )

    def decode(self, blob):
        if blob == b"poisoned truth blob":
            raise AssertionError("truth topic must never be decoded")
        return self.messages[int(blob)]

    def build(self):
        with patch.object(builder, "_decoder", return_value=self.decode):
            return builder.build_reference(self.args)

    def test_measured_correspondence_hash_and_no_native_truth_access(self):
        expected_hash = hashlib.sha256(self.database.read_bytes()).hexdigest()
        reference = self.build()
        self.assertEqual(reference["anchor"]["position_xyz_m"], [0.0, 0.0, 0.0])
        self.assertEqual(reference["anchor"]["source_message"]["message_id"], 1)
        self.assertEqual(reference["goals"][0]["stamp_ns"], 2_000_000_000)
        self.assertEqual(reference["goals"][0]["position_xyz_m"], [2.002, 0.0, 0.2])
        self.assertEqual(reference["source"]["files"][0]["sha256"], expected_hash)
        self.assertEqual(reference["source"]["message_count"], len(self.messages))
        self.assertTrue(reference["no_gt_pose_used"])
        self.assertTrue(reference["repeat_spawn_assumption"])
        self.assertFalse(reference["map_registration_enabled"])
        self.assertEqual(json.loads(self.args.output.read_text()), reference)
        # Estimator twist is retained and must not be falsely described as stopped.
        evidence = reference["goals"][0]["stop_evidence"]
        self.assertEqual(evidence["method"], "estimated_pose_window")
        self.assertEqual(evidence["estimated_twist_speed_at_endpoint_mps"], 0.08)
        self.assertFalse(evidence["twist_used_to_confirm_stop"])
        self.assertEqual(evidence["start_stamp_ns"], 1_000_000_000)
        self.assertEqual(evidence["sample_count"], 5)

    def test_explicit_ros_xyzw_conversion_and_invalid_pose_frames(self):
        q = (0, 0, math.sqrt(0.5), math.sqrt(0.5))
        pose = builder.odometry_pose(
            message(quaternion=q),
            map_frame=builder.MAP_FRAME,
            sensor_frame=builder.SENSOR_FRAME,
        )
        self.assertAlmostEqual(pose.quaternion_wxyz[0], math.sqrt(0.5))
        self.assertAlmostEqual(pose.quaternion_wxyz[3], math.sqrt(0.5))
        for changes in (
            {"frame": "world"},
            {"child": "base_link"},
            {"quaternion": (0, 0, 0, 0)},
            {"quaternion": (0, 0, 0, float("nan"))},
            {"xyz": (0, float("inf"), 0)},
            {"velocity": (float("nan"), 0, 0)},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                builder.odometry_pose(
                    message(**changes),
                    map_frame=builder.MAP_FRAME,
                    sensor_frame=builder.SENSOR_FRAME,
                )

    def test_duplicate_backward_and_invalid_samples_fail_without_output(self):
        for replacement in (
            message(1_000_000_000),
            message(500_000_000),
            message(1_250_000_000, frame="truth"),
        ):
            original = self.messages[2]
            self.messages[2] = replacement
            with self.subTest(replacement=replacement), self.assertRaises(ValueError):
                self.build()
            self.assertFalse(self.args.output.exists())
            self.messages[2] = original

    def test_goal_stamp_requires_exact_recorded_correspondence(self):
        self.args.goal_stamp_ns = [1_999_999_999]
        with self.assertRaisesRegex(ValueError, "exact measured"):
            self.build()
        self.assertFalse(self.args.output.exists())
        self.args.goal_stamp_ns = [2_000_000_000]
        result = self.build()
        self.assertEqual(result["goal_selection"]["method"], "exact_header_stamps")
        self.assertEqual(result["goals"][0]["source_message"]["message_id"], 6)

    def test_moving_or_unobserved_stop_window_is_rejected(self):
        for altered in (
            message(1_500_000_000, xyz=(1.9, 0, 0.2)),
            message(1_499_999_999, xyz=(2, 0, 0.2)),
        ):
            original = self.messages[3]
            self.messages[3] = altered
            with (
                self.subTest(altered=altered),
                self.assertRaisesRegex(ValueError, "not stopped|gap"),
            ):
                self.build()
            self.assertFalse(self.args.output.exists())
            self.messages[3] = original

    def test_existing_output_and_uncheckpointed_wal_are_never_overwritten(self):
        self.args.output.write_text("existing")
        with self.assertRaisesRegex(ValueError, "nonexistent"):
            self.build()
        self.assertEqual(self.args.output.read_text(), "existing")
        self.args.output.unlink()
        Path(str(self.database) + "-wal").write_bytes(b"live")
        with self.assertRaisesRegex(ValueError, "WAL"):
            self.build()
        self.assertFalse(self.args.output.exists())

    def test_mutating_source_during_decode_is_detected(self):
        original_decode = self.decode
        mutated = False

        def decode(blob):
            nonlocal mutated
            if not mutated:
                with self.database.open("ab") as stream:
                    stream.write(b"mutation")
                mutated = True
            return original_decode(blob)

        with (
            patch.object(builder, "_decoder", return_value=decode),
            self.assertRaisesRegex(ValueError, "changed"),
        ):
            builder.build_reference(self.args)
        self.assertFalse(self.args.output.exists())


if __name__ == "__main__":
    unittest.main()
