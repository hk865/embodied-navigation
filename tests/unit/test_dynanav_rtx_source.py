"""Offline selection/provenance contracts; no Isaac, ROS nodes or GPU work."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

from embodied_navigation.simulation.dynanav import experiment
from embodied_navigation.simulation.dynanav.operator_map import validate_map_cloud_source

ROOT = Path(__file__).resolve().parents[2]


class RtxSourceSelection(unittest.TestCase):
    def check_profile(self, *extra):
        with tempfile.TemporaryDirectory() as temporary:
            data = Path(temporary)
            scene = data / "office.usd"
            scene.touch()
            robot = data / "runtime_assets/robot/go2_description/go2_description.usda"
            robot.parent.mkdir(parents=True)
            robot.touch()
            output = data / "never_created"
            capture = io.StringIO()
            with patch.object(experiment, "data_root", return_value=data), patch(
                "embodied_navigation.robot.unitree.robogauge.runtime.verify_model"
            ), patch(
                "embodied_navigation.robot.unitree.robogauge.runtime.default_model_path",
                return_value=data / "policy.pt",
            ), contextlib.redirect_stdout(capture):
                self.assertEqual(experiment.main([
                    "--output", str(output), "--scene", str(scene), "--check", *extra
                ]), 0)
            self.assertFalse(output.exists())
            return json.loads(capture.getvalue())["profile"]

    def test_default_and_explicit_rgbd_disable_retired_oracle_semantics(self):
        for args in ((), ("--point-cloud-source", "rgbd")):
            profile = self.check_profile(*args)
            self.assertEqual(profile["point_cloud_source"], "rgbd")
            self.assertEqual(profile["semantic_provider"], "none")
            self.assertEqual(profile["mapping_pose_source"], "disabled")
            self.assertNotIn("lidar_frontend_topic", profile)
            self.assertEqual(profile["render_backend"], "usd")
            self.assertNotIn("requested_render_backend", profile)
            self.assertNotIn("render_backend_override_reason", profile)

    def test_rtx_selects_real_topic_and_disables_rgbd_semantic_mapping(self):
        profile = self.check_profile("--point-cloud-source", "rtx_lidar")
        self.assertEqual(profile["lidar_frontend_topic"], "/dynanav/lidar_rtx")
        self.assertEqual(profile["rtx_lidar_config"], "OS1_REV6_32ch10hz1024res")
        self.assertEqual(profile["lidar_frame_id"], "go2_imu")
        self.assertEqual(profile["semantic_provider"], "none")
        self.assertEqual(profile["mapping_pose_source"], "disabled")
        self.assertFalse(profile["mapping_enabled"])
        self.assertEqual(profile["lidar_scan_hz"], 10)

    def test_rtx_forces_physics_fabric_and_records_requested_backend(self):
        for extra, requested in (((), "usd"), (("--render-backend", "usd"), "usd"),
                                 (("--render-backend", "fabric"), "fabric")):
            with self.subTest(extra=extra):
                profile = self.check_profile("--point-cloud-source", "rtx_lidar", *extra)
                self.assertEqual(profile["render_backend"], "fabric")
                self.assertEqual(profile["requested_render_backend"], requested)
                self.assertIn("physics Fabric updates", profile["render_backend_override_reason"])
                self.assertIn("initial body transform", profile["render_backend_override_reason"])

    def test_rgbd_explicit_render_backend_still_respects_user_selection(self):
        profile = self.check_profile("--point-cloud-source", "rgbd", "--render-backend", "fabric")
        self.assertEqual(profile["render_backend"], "fabric")
        self.assertNotIn("requested_render_backend", profile)
        self.assertNotIn("render_backend_override_reason", profile)

    def test_rtx_refuses_depth_semantics_and_oracle_pedestrian_before_asset_checks(self):
        for extra in (
            ("--semantic-provider", "isaac_instance_oracle"),
            ("--semantic-provider", "learned_rgb_person"),
            ("--pedestrian",),
            ("--person-worker", "/not-used"),
            ("--learned-evaluation-oracle",),
        ):
            with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    experiment.main(["--output", "/not-used", "--point-cloud-source", "rtx_lidar", *extra])
                self.assertEqual(error.exception.code, 2)

    def test_sensor_override_keeps_imu_frame_and_retains_existing_camera_calibration(self):
        document = yaml.safe_load((ROOT / "configs/simulation/dynanav/fastlivo_rtx_os1_32.yaml").read_text())["/**"]["ros__parameters"]
        self.assertEqual(document["preprocess"], {
            "lidar_type": 3, "scan_line": 32, "feature_extract_enabled": False,
            "point_filter_num": 1,
        })
        self.assertEqual(document["extrin_calib"], {
            "extrinsic_T": [0.0, 0.0, 0.0],
            "extrinsic_R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        })
        self.assertEqual(document["common"], {
            "allow_ouster_lidar": True, "require_complete_imu": True,
            "same_pair_vio_handoff": True,
            "max_buffered_images": 32,
            "max_buffered_imu": 8192,
            "max_buffered_prop_imu": 8192,
        })

    def test_single_pose_operator_map_refuses_raw_rotating_source(self):
        for source in ("/dynanav/lidar_rtx", "lidar_rtx", "rtx_lidar"):
            with self.assertRaisesRegex(ValueError, "IMU deskew"):
                validate_map_cloud_source(source)
        self.assertEqual(validate_map_cloud_source("/dynanav/lidar_filtered"), "/dynanav/lidar_filtered")


if __name__ == "__main__":
    unittest.main()
