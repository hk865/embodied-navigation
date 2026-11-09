"""CPU evidence-contract tests; ROS decoding and a live simulator are not needed."""

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

SCRIPT = Path(__file__).resolve().parents[2] / "scripts/reconstruct_dynanav_slam_map.py"
spec = importlib.util.spec_from_file_location("dynanav_slam_reconstruction", SCRIPT)
replay = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = replay
spec.loader.exec_module(replay)


def pose(stamp=100, translation=(10, 20, 30), **kwargs):
    transform = np.eye(4)
    transform[:3, 3] = translation
    return replay.OdomPose(stamp, transform, "dynanav_lio_map", "go2_imu", **kwargs)


def message(
    *, frame="dynanav_lio_map", child="go2_imu", quaternion=(0, 0, 0, 1), xyz=(1, 2, 3)
):
    vector = lambda names, values: SimpleNamespace(**dict(zip(names, values)))
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=0, nanosec=100), frame_id=frame
        ),
        child_frame_id=child,
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=vector("xyz", xyz), orientation=vector("xyzw", quaternion)
            )
        ),
    )


class PairingTests(unittest.TestCase):
    def test_exact_default_explicit_numeric_tolerance_and_ambiguity(self):
        index = replay.PoseIndex([pose(99)])
        self.assertEqual(index.match(100).status, "unmatched_pose")
        match = index.match(100, 1)
        self.assertEqual(
            (match.status, match.odom_source_ns, match.delta_ns), ("matched", 99, -1)
        )
        self.assertEqual(index.match(101, 1).status, "unmatched_pose")
        for value in (-1, 1001, True, 1.0):
            with self.subTest(value=value), self.assertRaises(ValueError):
                index.match(100, value)
        self.assertEqual(
            replay.PoseIndex([pose(99), pose(101)]).match(100, 1).status,
            "ambiguous_pose",
        )
        self.assertEqual(
            replay.PoseIndex([pose(), pose(translation=(0, 0, 0))]).match(100).status,
            "duplicate_pose_stamp",
        )

    def test_invalid_exact_estimate_is_never_replaced_with_neighbor(self):
        for reason in ("frame_mismatch", "invalid_odom_pose"):
            index = replay.PoseIndex([pose(100, error=reason), pose(101)])
            result = index.match(100, 1)
            self.assertIsNone(result.pose)
            self.assertEqual(result.status, reason)

    def test_odometry_validates_sensor_and_map_frames_and_quaternion(self):
        kwargs = {"map_frame": "dynanav_lio_map", "sensor_frame": "go2_imu"}
        valid = replay.odometry_pose(message(), **kwargs)
        np.testing.assert_array_equal(valid.transform[:3, 3], [1, 2, 3])
        for fields, reason in [
            ({"frame": "map"}, "frame_mismatch"),
            ({"child": "base_link"}, "frame_mismatch"),
            ({"quaternion": (0, 0, 0, 0)}, "invalid_odom_pose"),
            ({"quaternion": (0, 0, 0, float("nan"))}, "invalid_odom_pose"),
            ({"xyz": (float("inf"), 0, 0)}, "invalid_odom_pose"),
        ]:
            with self.subTest(fields=fields):
                result = replay.odometry_pose(message(**fields), **kwargs)
                self.assertIsNone(result.transform)
                self.assertEqual(result.error, reason)


class ReconstructionTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.frames = self.directory / "keyframes"
        self.frames.mkdir()
        self.database = self.directory / "bag.db3"
        self.database.write_bytes(b"decoder is patched; storage digest remains real")
        self.args = SimpleNamespace(
            keyframes=self.frames,
            bag=self.database,
            output=self.directory / "estimated_map",
            observer_label="fixture-estimator-v1",
            observer_metadata=[],
            odom_topic=replay.ODOM_TOPIC,
            map_frame="dynanav_lio_map",
            sensor_frame="go2_imu",
            calibration_id="rgbd:v1",
            stamp_tolerance_ns=0,
            pedestrian_root=["/World/DynamicPedestrian"],
            epoch=1,
            map_id="fixture",
            voxel_size=0.08,
            max_points=8192,
            max_objects=4096,
        )

    def archive(self, sequence=1, stamp=100, **extra):
        path = self.frames / f"rgbd_{sequence:06d}.npz"
        # Accessing this object array with allow_pickle=False raises. A passing
        # reconstruction therefore proves the simulator transform was not read.
        fields = {
            "source_ns": np.int64(stamp),
            "semantic_oracle": np.bool_(True),
            "points_sensor": np.array([[1.0, 0, 0], [2.0, 0, 0], [3.0, 0, 0]]),
            "states": np.array([1, 2, 0], dtype=np.uint8),
            "instance_paths": np.array(
                ["/World/wall", "/World/DynamicPedestrian/mesh", ""]
            ),
            "T_map_sensor": np.array([object()], dtype=object),
        }
        fields.update(extra)
        np.savez_compressed(path, **fields)
        return path

    def run_reconstruction(self, poses):
        with patch.object(
            replay, "read_odometry", return_value=(poses, {"messages": len(poses)})
        ):
            return replay.reconstruct(self.args)

    def test_poisoned_truth_is_not_read_and_unmatched_early_frame_is_skipped(self):
        self.archive(stamp=50)
        self.archive(sequence=2, stamp=100)
        receipt = self.run_reconstruction([pose()])
        with np.load(
            self.args.output / "static_map.npz", allow_pickle=False
        ) as archive:
            np.testing.assert_array_equal(archive["points"], [[11, 20, 30]])
            self.assertEqual(archive["pose_source"].item(), "slam")
        scene = json.loads((self.args.output / "scene_map.json").read_text())
        self.assertEqual(scene["observation_count"], 1)
        self.assertEqual(
            scene["observation_point_counts"], {"static": 1, "dynamic": 1, "unknown": 1}
        )
        self.assertEqual(scene["provenance_summary"]["pose_sources"], ["slam"])
        self.assertEqual(
            {obj["label"] for obj in scene["objects"]}, {"scene_geometry", "person"}
        )
        self.assertTrue(receipt["no_GT_pose_used"])
        self.assertTrue(receipt["semantic_oracle"])
        self.assertFalse(receipt["complete_full_scene"])
        self.assertEqual(
            receipt["pairing"]["counts"], {"unmatched_pose": 1, "matched": 1}
        )
        self.assertEqual(len(receipt["keyframe_sources"]), 2)
        self.assertEqual(len(receipt["keyframe_sources"][0]["sha256"]), 64)
        self.assertNotIn("static_points", scene)
        self.assertNotIn("history", scene["objects"][0])

    def test_no_matching_estimate_does_not_create_output_or_truth_fallback(self):
        self.archive()
        with self.assertRaisesRegex(ValueError, "no valid keyframe"):
            self.run_reconstruction([pose(200)])
        self.assertFalse(self.args.output.exists())

    def test_recorded_labels_provenance_and_numeric_delta_survive(self):
        self.archive(
            labels_json=json.dumps(
                {"/World/wall": "wall", "/World/DynamicPedestrian/mesh": "person"}
            ),
            mask_provenance_json=json.dumps(
                dict(replay.MASK_PROVENANCE, semantic_calls=2)
            ),
        )
        self.args.stamp_tolerance_ns = 1
        receipt = self.run_reconstruction([pose(99)])
        scene = json.loads((self.args.output / "scene_map.json").read_text())
        wall = next(obj for obj in scene["objects"] if obj["label"] == "wall")
        self.assertEqual(wall["pose_source_ns"], 100)
        self.assertEqual(wall["provenance"]["estimated_pose_header_ns"], 99)
        self.assertEqual(wall["provenance"]["estimated_pose_stamp_delta_ns"], -1)
        self.assertEqual(wall["provenance"]["label_source"], "recorded_labels_json")
        self.assertEqual(wall["provenance"]["semantic_calls"], 2)
        self.assertEqual(receipt["pairing"]["keyframes"][0]["stamp_delta_ns"], -1)

    def test_wrong_provenance_and_duplicate_source_time_are_counted(self):
        self.archive(
            mask_provenance_json=json.dumps(
                dict(replay.MASK_PROVENANCE, kind="learned")
            )
        )
        self.archive(sequence=2)
        self.archive(sequence=3)
        receipt = self.run_reconstruction([pose()])
        self.assertEqual(
            receipt["pairing"]["counts"], {"invalid_keyframe": 2, "matched": 1}
        )
        self.assertEqual(receipt["retained_observations"], 1)
        errors = [
            row["error"] for row in receipt["pairing"]["keyframes"] if "error" in row
        ]
        self.assertTrue(any("provenance" in error for error in errors))
        self.assertTrue(any("duplicate/backward" in error for error in errors))

    def test_existing_output_is_never_overwritten(self):
        self.archive()
        self.args.output.mkdir()
        with self.assertRaisesRegex(ValueError, "nonexistent"):
            self.run_reconstruction([pose()])


if __name__ == "__main__":
    unittest.main()
