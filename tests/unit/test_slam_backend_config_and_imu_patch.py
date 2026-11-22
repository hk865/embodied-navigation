"""SLAM backend config/loading, anisotropic information matrices and the IMU bias patch.

The backend writes a 6x6 information matrix per graph edge. This suite pins the
normalised surface: strict JSON loading of BackendConfig (nested scan_context,
per-axis sigmas), the exact diagonal of anisotropic_information and the
matrices the pose graph actually receives, the --config CLI option, and the
vendored FAST-LIVO2 patch that makes imu.b_gyr_cov / imu.b_acc_cov
configurable. The patch is only checked, never applied: the vendored sources
stay byte-identical.
"""
import importlib
import json
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

import slam_backend_fixtures as F
from embodied_navigation.localization_mapping.backend.backend import (
    BackendConfig,
    SlamBackend,
    anisotropic_information,
    information,
)
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe
from embodied_navigation.localization_mapping.backend.scan_context import ScanContextConfig

REPO_ROOT = Path(__file__).resolve().parents[2]
IMU_BIAS_PATCH = REPO_ROOT / "third_party/fast_livo2/patches/0011-imu-bias-covariance-params.patch"
NODE_MODULE = "embodied_navigation.localization_mapping.backend.node"


def _stub_ros_modules():
    """Minimal ROS stubs so the unit suite never needs rclpy installed."""

    def module(name, **attributes):
        stub = types.ModuleType(name)
        stub.__dict__.update(attributes)
        return stub

    return {
        "rclpy": module("rclpy"),
        "rclpy.node": module("rclpy.node", Node=type("Node", (), {})),
        "rclpy.callback_groups": module("rclpy.callback_groups", MutuallyExclusiveCallbackGroup=object),
        "rclpy.executors": module("rclpy.executors", MultiThreadedExecutor=object),
        "rclpy.signals": module("rclpy.signals", SignalHandlerOptions=types.SimpleNamespace(NO=0)),
        "geometry_msgs": module("geometry_msgs"),
        "geometry_msgs.msg": module("geometry_msgs.msg", TransformStamped=object),
        "nav_msgs": module("nav_msgs"),
        "nav_msgs.msg": module("nav_msgs.msg", Odometry=object),
        "std_msgs": module("std_msgs"),
        "std_msgs.msg": module("std_msgs.msg", String=object),
        "tf2_ros": module("tf2_ros", TransformBroadcaster=object),
    }


class AnisotropicInformationTests(unittest.TestCase):
    def test_diagonal_uses_pxy_pxy_pz_prp_prp_pyaw_order(self):
        matrix = anisotropic_information(0.5, 0.2, 0.03, 0.01)
        self.assertEqual(matrix.shape, (6, 6))
        expected = np.array([0.5**-2, 0.5**-2, 0.2**-2, 0.03**-2, 0.03**-2, 0.01**-2])
        np.testing.assert_array_equal(np.diag(matrix), expected)
        self.assertFalse((matrix - np.diag(np.diag(matrix))).any())

    def test_isotropic_information_is_the_anisotropic_special_case(self):
        np.testing.assert_array_equal(information(0.2, 0.02), anisotropic_information(0.2, 0.2, 0.02, 0.02))
        np.testing.assert_array_equal(information(0.2, 0.02), np.diag([0.2**-2] * 3 + [0.02**-2] * 3))

    def test_non_positive_non_finite_and_bool_sigmas_are_rejected(self):
        valid = [0.5, 0.2, 0.03, 0.01]
        for index in range(4):
            for bad in (0.0, -0.5, float("nan"), float("inf"), float("-inf"), True, 1e-200, 1e200):
                arguments = list(valid)
                arguments[index] = bad
                with self.assertRaises(ValueError, msg=f"sigma #{index} accepted {bad!r}"):
                    anisotropic_information(*arguments)


class BackendConfigLoadingTests(unittest.TestCase):
    def test_from_dict_parses_nested_scan_context_and_anisotropic_sigmas(self):
        config = BackendConfig.from_dict(
            {
                "scan_context": {"rings": 10, "sectors": 30, "max_range_m": 12.0},
                "odometry_sigma_m": 0.5,
                "odometry_sigma_z_m": 0.2,
                "odometry_sigma_yaw_rad": 0.01,
                "loop_sigma_m": 0.09,
                "loop_sigma_z_m": 0.03,
                "loop_sigma_rad": 0.015,
                "loop_sigma_yaw_rad": 0.004,
            }
        )
        self.assertEqual(config.scan_context, ScanContextConfig(rings=10, sectors=30, max_range_m=12.0))
        self.assertEqual(config.odometry_sigma_m, 0.5)
        np.testing.assert_array_equal(
            config.odometry_information(), anisotropic_information(0.5, 0.2, 0.02, 0.01)
        )
        np.testing.assert_array_equal(
            config.loop_information(), anisotropic_information(0.09, 0.03, 0.015, 0.004)
        )

    def test_from_dict_matches_default_constructor_for_empty_mapping(self):
        self.assertEqual(BackendConfig.from_dict({}), BackendConfig())

    def test_from_dict_rejects_unknown_top_level_and_scan_context_keys(self):
        for data in ({"odometry": 0.2}, {"scan_context": {"rings": 20, "sectors": 60, "unknown": 1.0}}):
            with self.assertRaises(ValueError, msg=repr(data)):
                BackendConfig.from_dict(data)

    def test_from_dict_rejects_non_mapping_input(self):
        for data in ([], ["odometry_sigma_m"], "config", 3, None):
            with self.assertRaises(ValueError, msg=repr(data)):
                BackendConfig.from_dict(data)

    def test_from_dict_rejects_invalid_sigmas(self):
        for key, value in (
            ("odometry_sigma_m", 0.0),
            ("odometry_sigma_rad", -0.02),
            ("loop_sigma_m", float("nan")),
            ("loop_sigma_rad", float("inf")),
            ("odometry_sigma_z_m", 0.0),
            ("odometry_sigma_yaw_rad", -1.0),
            ("loop_sigma_z_m", True),
            ("loop_sigma_yaw_rad", float("nan")),
        ):
            with self.assertRaises(ValueError, msg=f"{key}={value!r}"):
                BackendConfig.from_dict({key: value})

    def test_from_dict_rejects_invalid_scan_context_bounds_and_allows_zero_gaps(self):
        for bad_sc in (
            {"rings": 0},
            {"rings": True},
            {"rings": 2.5},
            {"sectors": -4},
            {"max_range_m": 0.0},
            {"max_range_m": float("inf")},
        ):
            with self.assertRaises(ValueError, msg=repr(bad_sc)):
                BackendConfig.from_dict({"scan_context": bad_sc})
        cfg = BackendConfig.from_dict({"exclude_recent": 0, "loop_submap_neighbors": 0, "min_loop_gap_s": 0.0})
        self.assertEqual(cfg.exclude_recent, 0)
        self.assertEqual(cfg.loop_submap_neighbors, 0)
        self.assertEqual(cfg.min_loop_gap_s, 0.0)

    def test_from_dict_rejects_invalid_steps_and_thresholds(self):
        for key, value in (
            ("icp_iterations", 0),
            ("icp_candidates", 0),
            ("min_fitness", 1.5),
            ("map_voxel_m", 0.0),
            ("submap_radius_m", -1.0),
            ("confirmations", 0),
        ):
            with self.assertRaises(ValueError, msg=f"{key}={value!r}"):
                BackendConfig.from_dict({key: value})

    def test_from_json_reads_the_same_strict_surface(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "backend.json"
            path.write_text(
                json.dumps(
                    {
                        "scan_context": {"rings": 12, "sectors": 36, "max_range_m": 15.0},
                        "loop_sigma_z_m": 0.04,
                        "loop_sigma_yaw_rad": 0.005,
                    }
                ),
                encoding="utf-8",
            )
            config = BackendConfig.from_json(path)
            self.assertEqual(config.scan_context, ScanContextConfig(rings=12, sectors=36, max_range_m=15.0))
            np.testing.assert_array_equal(
                config.loop_information(), anisotropic_information(0.05, 0.04, 0.01, 0.005)
            )
            path.write_text(json.dumps({"unknown_option": 1}), encoding="utf-8")
            with self.assertRaises(ValueError):
                BackendConfig.from_json(path)
            path.write_text(json.dumps([{"odometry_sigma_m": 0.2}]), encoding="utf-8")
            with self.assertRaises(ValueError):
                BackendConfig.from_json(path)


class IsotropicFallbackTests(unittest.TestCase):
    def test_none_overrides_keep_the_isotropic_sigmas(self):
        config = BackendConfig()
        np.testing.assert_array_equal(config.odometry_information(), information(0.2, 0.02))
        np.testing.assert_array_equal(config.loop_information(), information(0.05, 0.01))

    def test_partial_override_keeps_the_other_axes_isotropic(self):
        config = BackendConfig(odometry_sigma_z_m=0.05, loop_sigma_yaw_rad=0.004)
        np.testing.assert_array_equal(
            config.odometry_information(),
            np.diag([0.2**-2, 0.2**-2, 0.05**-2, 0.02**-2, 0.02**-2, 0.02**-2]),
        )
        np.testing.assert_array_equal(
            config.loop_information(),
            np.diag([0.05**-2, 0.05**-2, 0.05**-2, 0.01**-2, 0.01**-2, 0.004**-2]),
        )


class PoseGraphInformationTests(unittest.TestCase):
    """The matrices written into PoseGraph edges, not just the config helpers."""

    @classmethod
    def setUpClass(cls):
        cls.config = BackendConfig(
            odometry_sigma_m=0.5,
            odometry_sigma_z_m=0.2,
            odometry_sigma_rad=0.03,
            odometry_sigma_yaw_rad=0.01,
            loop_sigma_m=0.09,
            loop_sigma_z_m=0.03,
            loop_sigma_rad=0.015,
            loop_sigma_yaw_rad=0.004,
        )
        loop = F.closed_loop_with_drift(drift_m=1.5, step_m=3.0)
        rows = F.keyframe_stream(loop["scene"], loop["odom"], epoch=7)
        cls.backend = SlamBackend(cls.config)
        cls.events = [cls.backend.ingest(Keyframe(**row)) for row in rows]

    def test_odometry_edges_carry_the_anisotropic_odometry_information(self):
        expected = self.config.odometry_information()
        self.assertFalse(np.array_equal(expected, information(0.5, 0.03)))
        edges = [edge for edge in self.backend.graph.edges if not edge[4]]
        self.assertGreaterEqual(len(edges), 1)
        for edge in edges:
            self.assertEqual(edge[3].shape, (6, 6))
            np.testing.assert_array_equal(edge[3], expected)

    def test_loop_edges_carry_the_anisotropic_loop_information(self):
        expected = self.config.loop_information()
        edges = [edge for edge in self.backend.graph.edges if edge[4]]
        self.assertGreaterEqual(len(edges), 1, "fixture produced no loop closure")
        for edge in edges:
            self.assertEqual(edge[3].shape, (6, 6))
            np.testing.assert_array_equal(edge[3], expected)

    def test_fixture_closed_the_loop_through_ingest(self):
        self.assertTrue(any(event.get("loop") for event in self.events))
        self.assertFalse([event for event in self.events if "error" in event])


class BackendNodeConfigOptionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with mock.patch.dict(sys.modules, _stub_ros_modules()):
            sys.modules.pop(NODE_MODULE, None)
            cls.node = importlib.import_module(NODE_MODULE)

    def _args(self, *extra):
        return ["--spool", "spool", "--odometry-topic", "odometry", "--namespace", "backend", *extra]

    def test_config_defaults_to_none(self):
        self.assertIsNone(self.node.parse_args(self._args()).config)

    def test_config_accepts_a_json_path(self):
        args = self.node.parse_args(self._args("--config", "backend.json"))
        self.assertEqual(args.config, Path("backend.json"))

    def test_backend_node_publishes_stats_and_logs_config(self):
        published = []
        logged = []

        class _Pub:
            def publish(self, msg):
                published.append(msg)

        class _Logger:
            def info(self, text):
                logged.append(text)

            def warning(self, text):
                logged.append(f"WARN:{text}")

        class _String:
            def __init__(self, data=""):
                self.data = data

        class _FakeNode:
            def __init__(self, name):
                self._name = name

            def create_publisher(self, msg_type, topic, qos):
                return _Pub()

            def create_subscription(self, *a, **kw):
                return None

            def create_timer(self, *a, **kw):
                return None

            def get_logger(self):
                return _Logger()

        stubs = _stub_ros_modules()
        stubs["rclpy.node"].Node = _FakeNode
        stubs["std_msgs.msg"].String = _String
        stubs["tf2_ros"].TransformBroadcaster = lambda node: None
        with mock.patch.dict(sys.modules, stubs):
            sys.modules.pop(NODE_MODULE, None)
            node_mod = importlib.import_module(NODE_MODULE)
            args = node_mod.parse_args(self._args("--config", "frozen_backend.json"))
            backend = SlamBackend(BackendConfig.from_json(REPO_ROOT / "configs/localization_mapping/slam_backend_stabilized.json"))
            node = node_mod.BackendNode(args, backend)
            node.publish_status()

        self.assertTrue(any("frozen_backend.json" in entry and "zupt=True" in entry for entry in logged))
        self.assertEqual(len(published), 1)
        payload = json.loads(published[0].data)
        self.assertEqual(payload["config_path"], "frozen_backend.json")
        self.assertIn("stats", payload)
        self.assertEqual(payload["stats"]["travel_m"], 0.0)
        self.assertEqual(payload["stats"]["travel_since_last_loop_m"], 0.0)
        self.assertIsNone(payload["stats"]["last_loop_age_s"])
        self.assertFalse(payload["stats"]["zupt_active"])

    def test_malformed_keyframe_metadata_raises_value_error(self):
        from embodied_navigation.localization_mapping.backend.keyframes import read_keyframe

        with tempfile.TemporaryDirectory() as td:
            kf_dir = Path(td) / "kf_1_0"
            kf_dir.mkdir()
            (kf_dir / "points_lidar.pcd").write_text("DATA ascii\n0 0 0\n", encoding="utf-8")
            # JSON root is a list instead of dict
            (kf_dir / "metadata.json").write_text("[]", encoding="utf-8")
            with self.assertRaises(ValueError):
                read_keyframe(kf_dir)
            # Missing required stamp_ns field
            (kf_dir / "metadata.json").write_text(
                json.dumps({"schema": "dynanav.fast_livo2.keyframe.v1", "epoch": 1, "sequence": 0}),
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                read_keyframe(kf_dir)


class ImuBiasCovariancePatchTests(unittest.TestCase):
    def test_patch_file_exists(self):
        self.assertTrue(IMU_BIAS_PATCH.is_file(), IMU_BIAS_PATCH)

    def test_patch_applies_to_the_vendored_tree(self):
        result = subprocess.run(
            ["git", "apply", "--check", str(IMU_BIAS_PATCH)],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr or result.stdout)

    def test_stage_fast_livo2_source_applies_imu_bias_patch_without_mutating_vendor(self):
        from embodied_navigation.localization_mapping.fast_livo2_source import (
            FAST_LIVO2_BUILD_PATCHES,
            stage_fast_livo2_source,
        )

        self.assertIn("0011-imu-bias-covariance-params.patch", FAST_LIVO2_BUILD_PATCHES)
        vendor_cpp = REPO_ROOT / "third_party/fast_livo2/fast_livo2_core/src/LIVMapper.cpp"
        self.assertNotIn("imu.b_gyr_cov", vendor_cpp.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as td:
            build_root = Path(td) / "fast_livo2_stage"
            staged = stage_fast_livo2_source(REPO_ROOT / "third_party/fast_livo2", build_root)
            staged_cpp = (staged / "fast_livo2_core/src/LIVMapper.cpp").read_text(encoding="utf-8")
            staged_h = (staged / "fast_livo2_core/include/fast_livo2_core/core/LIVMapper.h").read_text(encoding="utf-8")
            self.assertIn("imu.b_gyr_cov", staged_cpp)
            self.assertIn("imu.b_acc_cov", staged_cpp)
            self.assertIn("b_gyr_cov", staged_h)
            receipt = json.loads((build_root / "source_stage.json").read_text(encoding="utf-8"))
            self.assertEqual(receipt["status"], "complete")
            self.assertEqual(receipt["patch_order"], list(FAST_LIVO2_BUILD_PATCHES))
            # Also verify staging inside a nested git checkout (Docker build_native.py path)
            # still patches the staged copy rather than silently skipping paths.
            git_repo = Path(td) / "outer_git_repo"
            git_repo.mkdir()
            subprocess.run(["git", "init", "-q"], cwd=git_repo, check=True)
            inside_stage = git_repo / "build/native/sources/fast_livo2_stage"
            staged_in_git = stage_fast_livo2_source(
                REPO_ROOT / "third_party/fast_livo2",
                inside_stage,
                allow_inside_repo=True,
            )
            self.assertIn(
                "imu.b_gyr_cov",
                (staged_in_git / "fast_livo2_core/src/LIVMapper.cpp").read_text(encoding="utf-8"),
            )
        self.assertNotIn("imu.b_gyr_cov", vendor_cpp.read_text(encoding="utf-8"))

    def test_build_native_plan_lists_imu_bias_patch(self):
        result = subprocess.run(
            [sys.executable, str(REPO_ROOT / "docker/build_native.py")],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=True,
        )
        plan = json.loads(result.stdout)
        self.assertIn(
            "third_party/fast_livo2/patches/0011-imu-bias-covariance-params.patch",
            plan["fast_livo2_build_patches"],
        )


if __name__ == "__main__":
    unittest.main()
