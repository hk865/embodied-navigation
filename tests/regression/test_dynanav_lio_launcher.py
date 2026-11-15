"""Launcher argv/provenance checks with fake ELF dependencies and runner only."""

import hashlib
import json
import os
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import yaml

from embodied_navigation.project import project_root


class ObserverOverride(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.global_data = self.directory / "external data"
        self.data = self.global_data / "embodied-navigation"
        self.prefix = self.data / "build_cache/fake native prefix"
        lib = self.prefix / "install/ros/lib"
        (lib / "fast_livo2_ros").mkdir(parents=True)
        (lib / "libfast_livo2_core.so").write_bytes(
            b"test-only common.allow_lio_only publish.post_vio_odometry_en common.allow_ouster_lidar "
            b"common.same_pair_vio_handoff common.max_buffered_images common.max_buffered_imu "
            b"common.max_buffered_prop_imu"
        )
        runner = lib / "fast_livo2_ros/fastlivo_mapping"
        runner.write_text(
            "#!/usr/bin/python3\n"
            "import json, os, pathlib, sys\n"
            "(pathlib.Path(os.environ['DEMO_RUN_DIR'])/'mock_argv.json').write_text(json.dumps(sys.argv[1:]))\n"
        )
        runner.chmod(0o755)
        tools = self.directory / "mock tools"
        tools.mkdir()
        (tools / "ldd").write_text("#!/bin/sh\nexit 0\n")
        (tools / "ldd").chmod(0o755)
        self.environment = os.environ.copy()
        self.environment.update(
            PROJECT_DATA_ROOT=str(self.global_data),
            PATH=str(tools) + os.pathsep + os.environ["PATH"],
            ROS_DOMAIN_ID="191",
        )
        self.script = project_root() / "scripts/run_dynanav_lio.sh"

    def invoke(self, *arguments):
        return subprocess.run(
            ["bash", str(self.script), "--binary-prefix", str(self.prefix), *arguments],
            env=self.environment,
            text=True,
            capture_output=True,
            check=False,
        )

    def test_default_has_one_parameter_file_and_no_override(self):
        output = self.data / "experiments/default run"
        result = self.invoke("--output-dir", str(output))
        self.assertEqual(result.returncode, 0, result.stderr)
        receipt = json.loads((output / "launch.json").read_text())
        self.assertEqual(receipt["observer_namespace"], "/dynanav/lio_observer")
        self.assertIn("__ns:=/dynanav/lio_observer", receipt["command"])
        self.assertFalse(any(token.startswith("/tf:=") for token in receipt["command"]))
        self.assertFalse(
            any(token.startswith("/post_vio_") for token in receipt["command"])
        )
        self.assert_lio_output_contract(receipt, "/dynanav/lio_observer")
        self.assertIsNone(receipt["parameter_override"])
        self.assertEqual(receipt["command"].count("--params-file"), 1)
        self.assertEqual(
            receipt["command"][1:], json.loads((output / "mock_argv.json").read_text())
        )

    def test_override_is_second_file_and_exact_path_hash_and_argv_survive_spaces(self):
        override = self.directory / "gravity experiment.yaml"
        override.write_text(
            "/**:\n  ros__parameters:\n    imu:\n      gravity_est_en: true\n"
        )
        output = self.data / "experiments/override run"
        result = self.invoke(
            "--parameter-override", str(override), "--output-dir", str(output)
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        receipt = json.loads((output / "launch.json").read_text())
        command = receipt["command"]
        parameters = [
            command[i + 1]
            for i, token in enumerate(command)
            if token == "--params-file"
        ]
        self.assertEqual(len(parameters), 2)
        self.assertTrue(parameters[0].endswith("/fastlivo_lio.yaml"))
        self.assertEqual(parameters[1], str(override.resolve()))
        self.assertEqual(
            receipt["parameter_override"],
            {
                "path": str(override.resolve()),
                "sha256": hashlib.sha256(override.read_bytes()).hexdigest(),
            },
        )
        self.assertEqual(
            command[1:], json.loads((output / "mock_argv.json").read_text())
        )

    def test_check_does_not_create_run_directory_or_start_runner(self):
        override = self.directory / "gravity.yaml"
        override.write_text("/**:\n  ros__parameters:\n    imu.gravity_est_en: true\n")
        output = self.data / "experiments/check run"
        result = self.invoke(
            "--check",
            "--parameter-override",
            str(override),
            "--output-dir",
            str(output),
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(output.exists())
        self.assertEqual(
            json.loads(result.stdout)["parameter_override"]["path"], str(override)
        )

    def test_rtx_preflight_selects_native_timed_sensor_profile_without_execution(self):
        output = self.data / "experiments/rtx check"
        result = self.invoke("--check", "--lidar-topic", "/dynanav/lidar_rtx", "--output-dir", str(output))
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertTrue(report["ready"])
        self.assertTrue(report["ouster_opt_in_parameter_present"])
        self.assertEqual(report["capability_errors"], [])
        self.assertFalse(output.exists())
        self.assertEqual(report["lidar_input_topic"], "/dynanav/lidar_rtx")
        self.assertEqual(report["measurement_source"], "RTX LiDAR range sensor; not RGB-D")
        self.assertEqual(report["lidar_point_contract"]["t"], "uint32 nanoseconds relative to scan-start header")
        self.assertEqual(report["lidar_point_contract"]["ring"], "uint8")
        self.assertFalse(report["lidar_point_contract"]["runtime_data_verified"])
        self.assertEqual(report["lidar_point_contract"]["capture_render_backend_required"], "fabric")
        self.assertTrue(report["lidar_point_contract"]["capture_physics_fabric_required"])
        self.assertFalse(report["lidar_point_contract"]["mount_updates_verified"])
        sensor = report["lidar_sensor_parameters"]
        self.assertTrue(sensor["path"].endswith("fastlivo_rtx_os1_32.yaml"))
        self.assertEqual(sensor["parameters"]["preprocess"]["lidar_type"], 3)
        self.assertEqual(sensor["sha256"], hashlib.sha256(Path(sensor["path"]).read_bytes()).hexdigest())

    def test_complete_scan_is_explicit_typed_livo_and_requires_binary_support(self):
        capture, _, blackboard, markers = self.livo_fixture()
        override = self.directory / "complete_scan.yaml"
        override.write_text("/**:\n  ros__parameters:\n    common.complete_scan_watermark: true\n")
        args = ("--check", "--mode", "livo", "--capture-result", str(capture),
                "--camera-blackboard", str(blackboard), "--lidar-topic", "/dynanav/lidar_rtx",
                "--parameter-override", str(override))
        result = self.invoke(*args)
        self.assertEqual(result.returncode, 2)
        self.assertIn("common.complete_scan_watermark", json.loads(result.stdout)["missing_rtx_features"])
        core = self.prefix / "install/ros/lib/libfast_livo2_core.so"
        core.write_bytes(core.read_bytes() + b" common.complete_scan_watermark")
        result = self.invoke(*args)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertTrue(report["complete_scan_watermark_requested"])
        self.assertEqual(report["subscribed_lidar_topic"], "/dynanav/lidar_rtx/complete_scan")
        result = self.invoke("--check", "--lidar-topic", "/dynanav/lidar_rtx", "--parameter-override", str(override))
        self.assertEqual(result.returncode, 2)
        self.assertTrue(all(not marker.exists() for marker in markers))

    def test_rtx_allows_explicit_same_pair_handoff_comparison_switch(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled):
                override = self.directory / "same pair handoff.yaml"
                override.write_text(yaml.safe_dump({"/**": {"ros__parameters": {
                    "common": {"same_pair_vio_handoff": enabled}
                }}}))
                result = self.invoke(
                    "--check", "--lidar-topic", "/dynanav/lidar_rtx",
                    "--parameter-override", str(override),
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                report = json.loads(result.stdout)
                self.assertTrue(report["ready"])
                self.assertIs(report["rtx_handoff_requested"], enabled)
                self.assertEqual(report["parameter_override"]["path"], str(override))
                self.assertEqual(report["lidar_sensor_parameters"]["parameters"]
                                 ["preprocess"]["lidar_type"], 3)

    def test_rtx_rejects_ouster_core_missing_requested_handoff_and_buffer_code(self):
        core = self.prefix / "install/ros/lib/libfast_livo2_core.so"
        core.write_bytes(b"common.allow_lio_only publish.post_vio_odometry_en common.allow_ouster_lidar")
        result = self.invoke("--check", "--lidar-topic", "/dynanav/lidar_rtx")
        self.assertEqual(result.returncode, 2, result.stderr)
        report = json.loads(result.stdout)
        self.assertFalse(report["ready"])
        self.assertTrue(report["ouster_opt_in_parameter_present"])
        self.assertIn("common.same_pair_vio_handoff", report["missing_rtx_features"])
        self.assertIn("a YAML setting alone", result.stderr)

    def test_rtx_rejects_legacy_core_and_explains_explicit_build_root_without_starting(self):
        core = self.prefix / "install/ros/lib/libfast_livo2_core.so"
        old_bytes = b"test-only common.allow_lio_only publish.post_vio_odometry_en"
        core.write_bytes(old_bytes)
        output = self.data / "experiments/unsupported rtx"
        for mode in ("lio", "livo"):
            extra = []
            if mode == "livo":
                capture, _, blackboard, markers = self.livo_fixture()
                extra = ["--capture-result", str(capture), "--camera-blackboard", str(blackboard)]
            for check in ([], ["--check"]):
                with self.subTest(mode=mode, check=check):
                    result = self.invoke(*check, "--mode", mode, "--lidar-topic", "/dynanav/lidar_rtx",
                                         "--output-dir", str(output), *extra)
                    self.assertEqual(result.returncode, 2, result.stderr)
                    report = json.loads(result.stdout)
                    self.assertFalse(report["ready"])
                    self.assertFalse(report["ouster_opt_in_parameter_present"])
                    self.assertEqual(len(report["capability_errors"]), 1)
                    self.assertIn("common.allow_ouster_lidar", result.stderr)
                    self.assertIn("--binary-prefix BUILD_ROOT", result.stderr)
                    self.assertIn("not install/ros itself", result.stderr)
                    self.assertEqual(Path(report["core"]), core)
                    self.assertEqual(core.read_bytes(), old_bytes)
                    self.assertFalse(output.exists())
            if mode == "livo":
                self.assertTrue(all(not marker.exists() for marker in markers))
        # The missing opt-in must not regress the legacy RGB-D preflight.
        result = self.invoke("--check")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(json.loads(result.stdout)["ouster_opt_in_parameter_present"])

    def test_readiness_reports_a_dead_mapper_clearly_and_cleans_up_probe(self):
        source = self.script.read_text().split("<<'PYREADY'\n", 1)[1].split("\nPYREADY", 1)[0]
        for failure in ("kill", "proc", "zombie"):
            with self.subTest(failure=failure):
                node = Mock()
                rclpy = types.ModuleType("rclpy")
                rclpy.init = Mock()
                rclpy.create_node = Mock(return_value=node)
                rclpy.shutdown = Mock()
                rclpy_node = types.ModuleType("rclpy.node")
                rclpy_node.NodeNameNonExistentError = type("NodeNameNonExistentError", (Exception,), {})
                preflight = {"mode": "lio", "observer_namespace": "/dynanav/lio_observer",
                             "lidar_input_topic": "/dynanav/lidar_rtx"}
                argv = ["readiness", str(self.directory), json.dumps(preflight), "123456", ""]
                with patch.dict(sys.modules, {"rclpy": rclpy, "rclpy.node": rclpy_node}), patch.object(
                    sys, "argv", argv
                ), patch("os.kill", side_effect=ProcessLookupError() if failure == "kill" else None), patch.object(
                    Path, "read_text", side_effect=FileNotFoundError() if failure == "proc" else None,
                    return_value="123456 (fake mapper) Z 0 0",
                ):
                    with self.assertRaises(SystemExit) as error:
                        exec(compile(source, str(self.script) + ":PYREADY", "exec"), {})
                self.assertIn("owned mapper exited before sensor readiness", str(error.exception))
                self.assertIn("runner.log", str(error.exception))
                self.assertNotIn("ProcessLookupError", str(error.exception))
                node.destroy_node.assert_called_once()
                rclpy.shutdown.assert_called_once()

    def _readiness_graph_case(self, certified, graph, ready, include_subscribed=True):
        source = self.script.read_text().split("<<'PYREADY'\n", 1)[1].split("\nPYREADY", 1)[0]
        node = Mock()
        node.get_subscriber_names_and_types_by_node.return_value = list(graph.items())
        rclpy = types.ModuleType("rclpy")
        rclpy.init = Mock()
        rclpy.create_node = Mock(return_value=node)
        rclpy.shutdown = Mock()
        rclpy.spin_once = Mock()
        rclpy_node = types.ModuleType("rclpy.node")
        rclpy_node.NodeNameNonExistentError = type("NodeNameNonExistentError", (Exception,), {})
        preflight = {"mode": "livo", "observer_namespace": "/dynanav/rtx_livo",
                     "lidar_input_topic": "/dynanav/lidar_rtx",
                     "subscribed_lidar_topic": "/dynanav/lidar_rtx/complete_scan" if certified else "/dynanav/lidar_rtx",
                     "complete_scan_watermark_requested": certified}
        expected_lidar = preflight["subscribed_lidar_topic"]
        if not include_subscribed:
            preflight.pop("subscribed_lidar_topic")
        argv = ["readiness", str(self.directory), json.dumps(preflight), "123456", "123457"]
        with patch.dict(sys.modules, {"rclpy": rclpy, "rclpy.node": rclpy_node}), patch.object(
            sys, "argv", argv
        ), patch("os.kill"), patch.object(Path, "read_text", return_value="123456 (fake owned process) S 0 0"), patch(
            "time.monotonic", side_effect=[0.0, 0.0, 11.0]
        ):
            if certified and not include_subscribed:
                with self.assertRaisesRegex(SystemExit, "complete scan readiness requires explicit subscribed_lidar_topic"):
                    exec(compile(source, str(self.script) + ":PYREADY", "exec"), {})
                rclpy.init.assert_not_called()
                rclpy.create_node.assert_not_called()
                self.assertFalse((self.directory / "ready.json").exists())
                return
            if ready:
                exec(compile(source, str(self.script) + ":PYREADY", "exec"), {})
            else:
                with self.assertRaisesRegex(SystemExit, "mapper input subscriptions not ready within 10s"):
                    exec(compile(source, str(self.script) + ":PYREADY", "exec"), {})
        node.get_subscriber_names_and_types_by_node.assert_called_once_with("dynanav_livo_observer", "/dynanav/rtx_livo")
        node.destroy_node.assert_called_once()
        rclpy.shutdown.assert_called_once()
        self.assertEqual((self.directory / "ready.json").exists(), ready)
        if ready:
            receipt = json.loads((self.directory / "ready.json").read_text())
            self.assertEqual(receipt["verified_sensor_subscriptions"], sorted([
                expected_lidar, "/dynanav/imu", "/dynanav/camera"]))
        else:
            self.assertFalse((self.directory / "ready.json.tmp").exists())

    def test_certified_readiness_accepts_actual_typed_dds_graph_without_raw_subscription(self):
        self._readiness_graph_case(True, {
            "/dynanav/lidar_rtx/complete_scan": ["dynanav_sensor_msgs/msg/CompleteLidarScan"],
            "/dynanav/imu": ["sensor_msgs/msg/Imu"], "/dynanav/camera": ["sensor_msgs/msg/Image"]}, True)

    def test_certified_readiness_rejects_raw_only_dds_graph(self):
        self._readiness_graph_case(True, {
            "/dynanav/lidar_rtx": ["sensor_msgs/msg/PointCloud2"],
            "/dynanav/imu": ["sensor_msgs/msg/Imu"], "/dynanav/camera": ["sensor_msgs/msg/Image"]}, False)

    def test_certified_readiness_rejects_missing_imu(self):
        self._readiness_graph_case(True, {
            "/dynanav/lidar_rtx/complete_scan": ["dynanav_sensor_msgs/msg/CompleteLidarScan"],
            "/dynanav/camera": ["sensor_msgs/msg/Image"]}, False)

    def test_certified_readiness_rejects_wrong_typed_topic(self):
        self._readiness_graph_case(True, {
            "/wrong/lidar_rtx/complete_scan": ["dynanav_sensor_msgs/msg/CompleteLidarScan"],
            "/dynanav/imu": ["sensor_msgs/msg/Imu"], "/dynanav/camera": ["sensor_msgs/msg/Image"]}, False)

    def test_certified_readiness_rejects_wrong_type_on_correct_topic(self):
        self._readiness_graph_case(True, {
            "/dynanav/lidar_rtx/complete_scan": ["sensor_msgs/msg/PointCloud2"],
            "/dynanav/imu": ["sensor_msgs/msg/Imu"], "/dynanav/camera": ["sensor_msgs/msg/Image"]}, False)

    def test_off_readiness_keeps_raw_graph_behavior_without_new_type_gate(self):
        self._readiness_graph_case(False, {
            "/dynanav/lidar_rtx": ["legacy_unchecked_type"],
            "/dynanav/imu": ["sensor_msgs/msg/Imu"], "/dynanav/camera": ["sensor_msgs/msg/Image"]}, True)

    def test_off_readiness_rejects_typed_only_graph(self):
        self._readiness_graph_case(False, {
            "/dynanav/lidar_rtx/complete_scan": ["dynanav_sensor_msgs/msg/CompleteLidarScan"],
            "/dynanav/imu": ["sensor_msgs/msg/Imu"], "/dynanav/camera": ["sensor_msgs/msg/Image"]}, False)

    def test_legacy_off_readiness_without_subscribed_metadata_keeps_original_raw_check(self):
        self._readiness_graph_case(False, {
            "/dynanav/lidar_rtx": ["legacy_unchecked_type"],
            "/dynanav/imu": ["sensor_msgs/msg/Imu"], "/dynanav/camera": ["sensor_msgs/msg/Image"]},
            True, include_subscribed=False)

    def test_certified_readiness_without_subscribed_metadata_fails_closed(self):
        self._readiness_graph_case(True, {
            "/dynanav/lidar_rtx/complete_scan": ["dynanav_sensor_msgs/msg/CompleteLidarScan"],
            "/dynanav/imu": ["sensor_msgs/msg/Imu"], "/dynanav/camera": ["sensor_msgs/msg/Image"]},
            False, include_subscribed=False)

    def test_rtx_launch_records_actual_sensor_override_and_enters_owned_readiness(self):
        python_shim = self.directory / "mock tools/python3"
        python_shim.write_text(
            "#!/usr/bin/python3\n"
            "import json, os, pathlib, sys\n"
            "if len(sys.argv) == 6 and sys.argv[1] == '-' and sys.argv[3].startswith('{'):\n"
            "    preflight = json.loads(sys.argv[3])\n"
            "    (pathlib.Path(sys.argv[2])/'mock_readiness.json').write_text(json.dumps(preflight))\n"
            "else:\n"
            "    os.execv(sys.executable, [sys.executable, *sys.argv[1:]])\n"
        )
        python_shim.chmod(0o755)
        output = self.data / "experiments/rtx launch"
        result = self.invoke("--lidar-topic", "/dynanav/lidar_rtx", "--output-dir", str(output))
        # The test mapper intentionally exits. This must not leave a ready flag
        # or report successful initialization; the probe here is fake and offline.
        self.assertNotEqual(result.returncode, 0)
        self.assertTrue((output / "mock_readiness.json").is_file())
        self.assertTrue((output / "shutdown.json").is_file())
        self.assertFalse((output / "ready.json").exists())
        receipt = json.loads((output / "launch.json").read_text())
        command = receipt["command"]
        self.assertIn("/dynanav/lidar_static:=/dynanav/lidar_rtx", command)
        self.assertEqual(command.count("--params-file"), 2)
        sensor = receipt["lidar_sensor_parameters"]
        self.assertIn(sensor["path"], command)
        self.assertEqual(receipt["sensor_inputs"], ["/dynanav/lidar_rtx", "/dynanav/imu"])
        self.assertEqual(receipt["measurement_source"], "RTX LiDAR range sensor; not RGB-D")
        self.assertEqual(command[1:], json.loads((output / "mock_argv.json").read_text()))

    def test_rtx_override_cannot_erase_return_timing_or_double_apply_mount(self):
        for parameter in (
            "preprocess.lidar_type: 0", "preprocess.scan_line: 64",
            "preprocess.feature_extract_enabled: true", "preprocess.point_filter_num: 2",
            "common.allow_ouster_lidar: false", "common.require_complete_imu: false",
            "extrin_calib.extrinsic_T: [0.1, 0.0, 0.0]",
        ):
            with self.subTest(parameter=parameter):
                override = self.directory / "bad rtx override.yaml"
                override.write_text("/**:\n  ros__parameters:\n    " + parameter + "\n")
                result = self.invoke("--check", "--lidar-topic", "/dynanav/lidar_rtx", "--parameter-override", str(override))
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertTrue(json.loads(result.stdout)["observer_contract_conflicts"])

    def test_rtx_livo_keeps_capture_calibration_separate_from_lidar_override(self):
        capture, installation, blackboard, markers = self.livo_fixture()
        result = self.invoke("--check", "--mode", "livo", "--capture-result", str(capture),
                             "--camera-blackboard", str(blackboard), "--lidar-topic", "/dynanav/lidar_rtx")
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["camera_calibration"]["parameters"]["extrin_calib"]["Pcl"],
                         installation["camera_optical_translation_from_IMU"])
        self.assertNotIn("Rcl", report["lidar_sensor_parameters"]["parameters"]["extrin_calib"])
        self.assertTrue(all(not marker.exists() for marker in markers))

    def test_override_cannot_enable_truth_or_redirect_observer_sensor_inputs(self):
        for parameters in (
            "common.use_gps: true",
            "common.lid_topic: /ground_truth/cloud",
        ):
            with self.subTest(parameters=parameters):
                override = self.directory / "incompatible.yaml"
                override.write_text(
                    "/**:\n  ros__parameters:\n    " + parameters + "\n"
                )
                result = self.invoke("--check", "--parameter-override", str(override))
                self.assertEqual(result.returncode, 2)
                report = json.loads(result.stdout)
                self.assertFalse(report["ready"])
                self.assertTrue(report["observer_contract_conflicts"])

    def livo_fixture(self):
        """Provide only a measured installation and executables that must stay idle."""
        installation = {
            "camera_optical_R_from_IMU": [
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0],
                [1.0, 0.0, 0.0],
            ],
            "camera_optical_translation_from_IMU": [0.03, 0.15, -0.28],
            "camera_translation_in_base": [0.32, 0.01, 0.25],
            "imu_translation_in_base": [0.04, -0.02, 0.1],
            # Integer optical centers must become double ROS parameters.
            "camera_intrinsics": [318.25, 319.5, 321, 241],
        }
        capture = self.directory / "capture result.json"
        capture.write_text(json.dumps({"sensor_installation": installation}))
        blackboard = self.directory / "fake camera blackboard"
        markers = []
        for name, executable in (
            ("mapper", self.prefix / "install/ros/lib/fast_livo2_ros/fastlivo_mapping"),
            ("blackboard", blackboard),
        ):
            marker = self.directory / (name + " unexpectedly started")
            executable.write_text(
                "#!/usr/bin/python3\n"
                "from pathlib import Path\n"
                f"Path({str(marker)!r}).write_text('unexpected execution')\n"
                "raise SystemExit(91)\n"
            )
            executable.chmod(0o755)
            markers.append(marker)
        return capture, installation, blackboard, markers

    def assert_lio_output_contract(self, report, namespace):
        self.assertIs(report["post_vio_odometry_enabled"], False)
        self.assertEqual(
            report["navigation_pose_topic"], namespace + "/aft_mapped_to_init"
        )
        self.assertEqual(report["navigation_pose_source"], "lio_estimate")
        self.assertIsNone(report["post_vio_status_topic"])
        self.assertIsNone(report["post_vio_output_contract"])

    def assert_livo_output_contract(self, report, namespace):
        self.assertIs(report["post_vio_odometry_enabled"], True)
        self.assertIs(report["post_vio_output_capability_present"], True)
        self.assertEqual(
            report["navigation_pose_topic"], namespace + "/post_vio_odometry"
        )
        self.assertEqual(report["navigation_pose_source"], "livo_post_vio")
        self.assertEqual(
            report["post_vio_status_topic"], namespace + "/post_vio_odometry_status"
        )
        contract = report["post_vio_output_contract"]
        self.assertEqual(contract["pose_source"], "livo_post_vio")
        self.assertEqual(contract["state_stage"], "post_vio")
        self.assertIs(contract["twist_available"], False)
        self.assertIs(contract["pose_covariance_available"], False)
        self.assertIs(contract["visual_update_on_every_image"], False)

    def test_transport_check_selects_profile_without_creating_run_or_starting_nodes(self):
        capture, _, blackboard, markers = self.livo_fixture()
        for key in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
                    "FASTDDS_ENVIRONMENT_FILE", "SKIP_DEFAULT_XML"):
            self.environment.pop(key, None)
        self.environment.update(RMW_IMPLEMENTATION="rmw_fastrtps_cpp", FASTDDS_BUILTIN_TRANSPORTS="UDPv4")
        output = self.data / "experiments/transport check"
        for policy in ("auto", "baseline"):
            with self.subTest(policy=policy):
                result = self.invoke("--check", "--mode", "livo", "--capture-result", str(capture),
                                     "--camera-blackboard", str(blackboard), "--output-dir", str(output),
                                     "--observer-transport", policy)
                self.assertEqual(result.returncode, 0, result.stderr)
                transport = json.loads(result.stdout)["observer_transport"]
                self.assertEqual(transport["policy"], policy)
                self.assertEqual(bool(transport["profile"]), policy == "auto")
                self.assertFalse(transport["socket_buffer_verified"])
                self.assertFalse(output.exists())
                self.assertTrue(all(not marker.exists() for marker in markers))

    def test_livo_check_uses_capture_camera_calibration_without_starting_nodes(self):
        capture, installation, blackboard, markers = self.livo_fixture()
        output = self.data / "experiments/livo check run"
        result = self.invoke(
            "--check",
            "--mode",
            "livo",
            "--capture-result",
            str(capture),
            "--camera-blackboard",
            str(blackboard),
            "--output-dir",
            str(output),
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(output.exists())
        self.assertTrue(all(not marker.exists() for marker in markers))
        report = json.loads(result.stdout)
        self.assertTrue(report["ready"])
        self.assertEqual(report["mode"], "livo")
        self.assert_livo_output_contract(report, "/dynanav/lio_observer")
        self.assertEqual(report["camera_blackboard"], str(blackboard))
        self.assertEqual(
            report["camera_parameter_service"],
            "/dynanav/lio_observer/parameter_blackboard/get_parameters",
        )
        self.assertTrue(report["parameter_file"].endswith("/fastlivo_livo.yaml"))
        calibration = report["camera_calibration"]
        self.assertEqual(calibration["source"], str(capture.resolve()))
        self.assertEqual(
            calibration["source_sha256"],
            hashlib.sha256(capture.read_bytes()).hexdigest(),
        )
        self.assertEqual(calibration["selected_fields"], installation)
        parameters = calibration["parameters"]
        for name, expected in zip(
            ("cam_fx", "cam_fy", "cam_cx", "cam_cy"),
            installation["camera_intrinsics"],
            strict=True,
        ):
            self.assertEqual(parameters[name], expected)
            self.assertIsInstance(parameters[name], float)
        self.assertEqual(
            parameters["extrin_calib"]["Rcl"],
            [
                value
                for row in installation["camera_optical_R_from_IMU"]
                for value in row
            ],
        )
        self.assertEqual(
            parameters["extrin_calib"]["Pcl"],
            installation["camera_optical_translation_from_IMU"],
        )

        native = yaml.safe_load(Path(report["parameter_file"]).read_text())["/**"][
            "ros__parameters"
        ]
        self.assertEqual(native["common"]["img_topic"], "/dynanav/camera")
        self.assertEqual(native["common"]["img_en"], 1)
        self.assertIs(native["common"]["allow_lio_only"], False)
        self.assertIs(native["common"]["use_gps"], False)
        self.assertEqual(native["vio"]["img_point_cov"], 100)
        self.assertEqual(native["vio"]["outlier_threshold"], 100)
        self.assertIs(native["vio"]["exposure_estimate_en"], True)
        self.assertIs(native["publish"]["post_vio_odometry_en"], True)

    def test_old_lio_core_remains_ready_without_post_vio_output_capability(self):
        core = self.prefix / "install/ros/lib/libfast_livo2_core.so"
        core.write_bytes(b"test-only common.allow_lio_only")
        output = self.data / "experiments/old lio core check"
        result = self.invoke("--check", "--output-dir", str(output))
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertTrue(report["ready"])
        self.assertIs(report["post_vio_output_capability_present"], False)
        self.assert_lio_output_contract(report, "/dynanav/lio_observer")
        self.assertFalse(output.exists())

    def test_livo_requires_native_post_vio_output_capability_without_execution(self):
        capture, _, blackboard, markers = self.livo_fixture()
        core = self.prefix / "install/ros/lib/libfast_livo2_core.so"
        core.write_bytes(b"test-only common.allow_lio_only")
        output = self.data / "experiments/missing post vio capability"
        result = self.invoke(
            "--check",
            "--mode",
            "livo",
            "--capture-result",
            str(capture),
            "--camera-blackboard",
            str(blackboard),
            "--output-dir",
            str(output),
        )
        self.assertEqual(result.returncode, 2, result.stderr)
        report = json.loads(result.stdout)
        self.assertFalse(report["ready"])
        self.assertIs(report["post_vio_output_capability_present"], False)
        self.assertFalse(output.exists())
        self.assertTrue(all(not marker.exists() for marker in markers))

    def test_livo_launch_remaps_final_outputs_and_records_availability_contract(self):
        capture, _, blackboard, _ = self.livo_fixture()
        # Fake the service and graph probes for argv/provenance isolation.
        # Lifecycle behavior itself has fault-injection tests in the supervisor
        # suite. The owned blackboard stays alive until launcher cleanup.
        blackboard.write_text(
            "#!/usr/bin/python3\nimport signal,sys,time\n"
            "signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))\n"
            "time.sleep(60)\n"
        )
        runner = self.prefix / "install/ros/lib/fast_livo2_ros/fastlivo_mapping"
        runner.write_text(
            "#!/usr/bin/python3\n"
            "import json, os, pathlib, sys\n"
            "(pathlib.Path(os.environ['DEMO_RUN_DIR'])/'mock_argv.json').write_text(json.dumps(sys.argv[1:]))\n"
        )
        python_shim = self.directory / "mock tools/python3"
        python_shim.write_text(
            "#!/usr/bin/python3\n"
            "import json, os, pathlib, sys\n"
            "if len(sys.argv) > 2 and sys.argv[1] == '-' and sys.argv[2].endswith('/camera_parameter_ready.json'):\n"
            "    pathlib.Path(sys.argv[2]).write_text(json.dumps({'test_only': 'fake camera service readiness'}))\n"
            "elif len(sys.argv) == 6 and sys.argv[1] == '-' and sys.argv[3].startswith('{'):\n"
            "    (pathlib.Path(sys.argv[2])/'ready.json').write_text(json.dumps({'state': 'ready', 'test_only': True}))\n"
            "else:\n"
            "    os.execv(sys.executable, [sys.executable, *sys.argv[1:]])\n"
        )
        python_shim.chmod(0o755)
        namespace = "/dynanav/post_vio_test"
        output = self.data / "experiments/fake post vio run"
        result = self.invoke(
            "--mode",
            "livo",
            "--observer-namespace",
            namespace,
            "--capture-result",
            str(capture),
            "--camera-blackboard",
            str(blackboard),
            "--output-dir",
            str(output),
        )
        # An estimator exiting independently (even with 0) is not a healthy
        # observer. Provenance still must describe the exact attempted launch.
        self.assertNotEqual(result.returncode, 0, result.stderr)
        terminal = json.loads((output / "shutdown.json").read_text())
        self.assertEqual(terminal["exit_code"], result.returncode)
        self.assertFalse((output / "ready.json").exists())
        receipt = json.loads((output / "launch.json").read_text())
        preflight = json.loads((output / "preflight.json").read_text())
        for report in (receipt, preflight):
            self.assert_livo_output_contract(report, namespace)
        command = receipt["command"]
        self.assertEqual(
            command[1:], json.loads((output / "mock_argv.json").read_text())
        )
        for topic in ("post_vio_odometry", "post_vio_odometry_status"):
            self.assertIn(f"/{topic}:={namespace}/{topic}", command)
        self.assertEqual(
            json.loads((output / "camera_parameter_ready.json").read_text()),
            {"test_only": "fake camera service readiness"},
        )

    def test_lio_custom_namespace_isolates_all_outputs_and_keeps_sensor_inputs(self):
        namespace = "/dynanav/isolated_lio_2"
        output = self.data / "experiments/namespaced lio run"
        result = self.invoke(
            "--observer-namespace", namespace, "--output-dir", str(output)
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        receipt = json.loads((output / "launch.json").read_text())
        preflight = json.loads((output / "preflight.json").read_text())
        command = receipt["command"]
        self.assertEqual(receipt["observer_namespace"], namespace)
        self.assertEqual(preflight["observer_namespace"], namespace)
        self.assertIsNone(preflight["camera_parameter_service"])
        self.assertIn("__ns:=" + namespace, command)
        self.assertEqual(
            command[1:], json.loads((output / "mock_argv.json").read_text())
        )
        native_outputs = (
            "cloud_registered_full",
            "cloud_registered",
            "visualization_marker",
            "cloud_visual_sub_map_before",
            "cloud_effected",
            "Laser_map",
            "aft_mapped_to_init",
            "path",
            "planner_normal",
            "voxels",
            "dyn_obj",
            "dyn_obj_removed",
            "dyn_obj_dbg_hist",
            "mavros/vision_pose/pose",
            "rgb_img",
            "LIVO2/imu_propagate",
            "planes",
            "tf",
        )
        actual_remaps = {
            token for token in command if token.startswith("/") and ":=" in token
        }
        self.assertEqual(
            actual_remaps,
            {f"/{topic}:={namespace}/{topic}" for topic in native_outputs},
        )
        self.assertEqual(
            receipt["sensor_inputs"], ["/dynanav/lidar_static", "/dynanav/imu"]
        )
        self.assertEqual(receipt["clock_input"], "/clock")
        self.assertIs(receipt["ground_truth_input"], False)
        self.assert_lio_output_contract(receipt, namespace)
        self.assert_lio_output_contract(preflight, namespace)

    def test_livo_check_parameter_service_follows_custom_namespace_without_execution(
        self,
    ):
        capture, _, blackboard, markers = self.livo_fixture()
        namespace = "/dynanav/isolated_livo_2"
        output = self.data / "experiments/namespaced livo check"
        result = self.invoke(
            "--check",
            "--mode",
            "livo",
            "--observer-namespace",
            namespace,
            "--capture-result",
            str(capture),
            "--camera-blackboard",
            str(blackboard),
            "--output-dir",
            str(output),
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertTrue(report["ready"])
        self.assertEqual(report["observer_namespace"], namespace)
        self.assert_livo_output_contract(report, namespace)
        self.assertEqual(
            report["camera_parameter_service"],
            namespace + "/parameter_blackboard/get_parameters",
        )
        self.assertFalse(output.exists())
        self.assertTrue(all(not marker.exists() for marker in markers))

    def test_malformed_observer_namespaces_are_rejected_without_output(self):
        output = self.data / "experiments/rejected namespace"
        for namespace in (
            "relative",
            "/",
            "/trailing/",
            "//double",
            "/has-dash",
            "/with space",
            "/123start",
        ):
            with self.subTest(namespace=namespace):
                result = self.invoke(
                    "--check",
                    "--observer-namespace",
                    namespace,
                    "--output-dir",
                    str(output),
                )
                self.assertEqual(result.returncode, 2)
                self.assertIn("namespace", result.stderr.lower())
                self.assertFalse(output.exists())

    def test_livo_requires_known_mode_and_readable_capture_result(self):
        output = self.data / "experiments/rejected mode"
        for arguments, error in (
            (("--mode", "visual"), "Mode must be lio or livo"),
            (("--mode", "livo"), "requires readable --capture-result"),
            (
                (
                    "--mode",
                    "livo",
                    "--capture-result",
                    str(self.directory / "missing result.json"),
                ),
                "requires readable --capture-result",
            ),
        ):
            with self.subTest(arguments=arguments):
                result = self.invoke("--check", "--output-dir", str(output), *arguments)
                self.assertEqual(result.returncode, 2)
                self.assertIn(error, result.stderr)
                self.assertFalse(output.exists())

    def test_livo_rejects_invalid_capture_calibration_without_starting_nodes(self):
        capture, installation, blackboard, markers = self.livo_fixture()
        output = self.data / "experiments/invalid calibration"
        invalid_fields = (
            ("camera_optical_R_from_IMU", [[2, 0, 0], [0, 1, 0], [0, 0, 1]]),
            ("camera_optical_translation_from_IMU", [0.03, 0.15, -0.29]),
            ("camera_intrinsics", [0, 319.5, 321, 241]),
            ("camera_intrinsics", [318.25, 319.5, 321]),
            ("imu_translation_in_base", [0.04, float("inf"), 0.1]),
        )
        for name, value in invalid_fields:
            with self.subTest(field=name, value=value):
                invalid = {**installation, name: value}
                capture.write_text(json.dumps({"sensor_installation": invalid}))
                result = self.invoke(
                    "--check",
                    "--mode",
                    "livo",
                    "--capture-result",
                    str(capture),
                    "--camera-blackboard",
                    str(blackboard),
                    "--output-dir",
                    str(output),
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(
                    "calibration" if name == "camera_intrinsics" else "camera",
                    result.stderr,
                )
                self.assertFalse(output.exists())
                self.assertTrue(all(not marker.exists() for marker in markers))

    def test_livo_override_cannot_redirect_camera_or_enable_gps(self):
        capture, _, blackboard, markers = self.livo_fixture()
        output = self.data / "experiments/rejected livo override"
        for parameters, conflict in (
            ("common.img_topic: /ground_truth/camera", "common.img_topic"),
            ("common:\n      use_gps: true", "common.use_gps"),
        ):
            with self.subTest(parameters=parameters):
                override = self.directory / "incompatible livo.yaml"
                override.write_text(
                    "/**:\n  ros__parameters:\n    " + parameters + "\n"
                )
                result = self.invoke(
                    "--check",
                    "--mode",
                    "livo",
                    "--capture-result",
                    str(capture),
                    "--camera-blackboard",
                    str(blackboard),
                    "--parameter-override",
                    str(override),
                    "--output-dir",
                    str(output),
                )
                self.assertEqual(result.returncode, 2, result.stderr)
                report = json.loads(result.stdout)
                self.assertFalse(report["ready"])
                self.assertIn(conflict, report["observer_contract_conflicts"])
                self.assertFalse(output.exists())
                self.assertTrue(all(not marker.exists() for marker in markers))


if __name__ == "__main__":
    unittest.main()
