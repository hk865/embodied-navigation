"""Unit tests for bounded sliding-window frontend/planner configs, backend global map publisher, and RViz config."""

from __future__ import annotations

import importlib
import re
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe
from pct_budget_acceptance_fixtures import launch_without_external_processes

REPO_ROOT = Path(__file__).resolve().parents[2]
NODE_MODULE = "embodied_navigation.localization_mapping.backend.node"


def _stub_ros_with_pc2():
    def module(name, **attributes):
        stub = types.ModuleType(name)
        stub.__dict__.update(attributes)
        return stub

    class _Header:
        def __init__(self):
            self.frame_id = ""
            self.stamp = None

    class _PointField:
        FLOAT32 = 7

        def __init__(self, name="", offset=0, datatype=7, count=1):
            self.name = name
            self.offset = offset
            self.datatype = datatype
            self.count = count

    class _PointCloud2:
        def __init__(self):
            self.header = _Header()
            self.height = 0
            self.width = 0
            self.fields = []
            self.is_bigendian = False
            self.point_step = 0
            self.row_step = 0
            self.data = b""
            self.is_dense = False

    class _Stamp:
        def __init__(self, sec=0):
            self.sec = sec

    class _Clock:
        def now(self):
            return types.SimpleNamespace(to_msg=lambda: _Stamp(42))

    class _Logger:
        def info(self, text):
            pass

        def warning(self, text):
            pass

    class _String:
        def __init__(self, data=""):
            self.data = data

    return {
        "rclpy": module("rclpy"),
        "rclpy.node": module("rclpy.node", Node=object),
        "rclpy.callback_groups": module("rclpy.callback_groups", MutuallyExclusiveCallbackGroup=object),
        "rclpy.executors": module("rclpy.executors", MultiThreadedExecutor=object),
        "rclpy.signals": module("rclpy.signals", SignalHandlerOptions=types.SimpleNamespace(NO=0)),
        "geometry_msgs": module("geometry_msgs"),
        "geometry_msgs.msg": module("geometry_msgs.msg", TransformStamped=object),
        "nav_msgs": module("nav_msgs"),
        "nav_msgs.msg": module("nav_msgs.msg", Odometry=object),
        "std_msgs": module("std_msgs"),
        "std_msgs.msg": module("std_msgs.msg", String=_String),
        "sensor_msgs": module("sensor_msgs"),
        "sensor_msgs.msg": module("sensor_msgs.msg", PointCloud2=_PointCloud2, PointField=_PointField),
        "tf2_ros": module("tf2_ros", TransformBroadcaster=lambda node: None),
    }, _Clock, _Logger


class SlidingWindowAndGlobalMapTests(unittest.TestCase):
    def test_fastlivo_configs_enable_local_sliding_window(self):
        for name in ("fastlivo_lio.yaml", "fastlivo_livo.yaml", "fastlivo_livo_noisy_imu.yaml"):
            text = (REPO_ROOT / "configs/simulation/dynanav" / name).read_text(encoding="utf-8")
            with self.subTest(config=name):
                self.assertRegex(text, r"(?m)^\s*map_sliding_en:\s*true\s*$")
                self.assertRegex(text, r"(?m)^\s*half_map_size:\s*60\s*$")
                self.assertRegex(text, r"(?m)^\s*sliding_thresh:\s*8\.0\s*$")
                self.assertRegex(text, r"(?m)^\s*voxel_size:\s*0\.5\s*$")

    def test_rviz_live_config_has_zero_long_accumulation_and_subscribes_backend_global_map(self):
        text = (REPO_ROOT / "configs/visualization/dynanav_live_pointcloud.rviz").read_text(encoding="utf-8")
        self.assertIn("Value: /dynanav/backend/global_map", text)
        decays = [float(m.group(1)) for m in re.finditer(r"Decay Time:\s*([0-9.]+)", text)]
        self.assertGreaterEqual(len(decays), 4)
        self.assertIn(0.0, decays)
        self.assertLessEqual(max(decays), 2.0, f"Unbounded RViz Decay Time found: {decays}")

    def test_backend_node_publishes_deduplicated_global_map_and_snaps_on_loop_closure(self):
        stubs, ClockCls, LoggerCls = _stub_ros_with_pc2()
        published_by_topic: dict[str, list] = {}

        class _Pub:
            def __init__(self, topic):
                self.topic = topic

            def publish(self, msg):
                published_by_topic.setdefault(self.topic, []).append(msg)

        class _FakeNode:
            def __init__(self, name):
                self._clock = ClockCls()
                self._logger = LoggerCls()

            def create_publisher(self, msg_type, topic, qos):
                return _Pub(topic)

            def create_subscription(self, *a, **kw):
                return None

            def create_timer(self, *a, **kw):
                return None

            def get_clock(self):
                return self._clock

            def get_logger(self):
                return self._logger

        stubs["rclpy.node"].Node = _FakeNode
        with mock.patch.dict(sys.modules, stubs):
            sys.modules.pop(NODE_MODULE, None)
            node_mod = importlib.import_module(NODE_MODULE)
            args = node_mod.parse_args(
                [
                    "--spool",
                    "spool",
                    "--odometry-topic",
                    "/odom",
                    "--namespace",
                    "/dynanav/backend",
                    "--global-map-hz",
                    "0.5",
                    "--global-map-voxel-m",
                    "0.2",
                ]
            )
            backend = SlamBackend(BackendConfig())
            node = node_mod.BackendNode(args, backend)

            # Empty map publishes nothing
            self.assertFalse(node.publish_global_map())
            self.assertNotIn("/dynanav/backend/global_map", published_by_topic)

            # Add keyframe with duplicate points -> deduplicated PointCloud2 published
            pts = np.array([[1.0, 2.0, 0.5], [1.01, 2.01, 0.5], [4.0, -1.0, 0.2]], dtype=np.float64)
            backend.ingest(Keyframe((0, 0), 1_000_000_000, np.eye(4), np.eye(4), pts))
            self.assertTrue(node.publish_global_map())
            msgs = published_by_topic["/dynanav/backend/global_map"]
            self.assertEqual(len(msgs), 1)
            self.assertEqual(msgs[0].header.frame_id, "map")
            self.assertEqual(msgs[0].point_step, 12)
            self.assertEqual(msgs[0].width, 2)  # two points in same 0.2 m voxel deduplicated
            arr0 = np.frombuffer(msgs[0].data, dtype="<f4").reshape(-1, 3)
            self.assertEqual(arr0.shape, (2, 3))

            # Unchanged state skips redundant publish
            self.assertFalse(node.publish_global_map())
            self.assertEqual(len(msgs), 1)

            # Simulated loop closure shifts keyframe pose by +10 m in X and increments loop_count
            T_shifted = np.eye(4)
            T_shifted[0, 3] = 10.0
            backend.map.update_poses({(0, 0): T_shifted})
            backend.loop_count += 1
            # Immediate publish bypasses rate limit when loop_count increases
            self.assertTrue(node.publish_global_map())
            self.assertEqual(len(msgs), 2)
            arr1 = np.frombuffer(msgs[1].data, dtype="<f4").reshape(-1, 3)
            self.assertGreater(float(arr1[:, 0].min()), 10.5)

            # Disabling via --global-map-hz 0 disables publisher creation
            args_off = node_mod.parse_args(
                [
                    "--spool",
                    "spool",
                    "--odometry-topic",
                    "/odom",
                    "--namespace",
                    "/dynanav/backend",
                    "--global-map-hz",
                    "0",
                ]
            )
            node_off = node_mod.BackendNode(args_off, backend)
            self.assertIsNone(node_off.global_map_pub)
            self.assertFalse(node_off.publish_global_map(force=True))

    def test_launcher_sets_default_local_map_radius_and_honors_override(self):
        with tempfile.TemporaryDirectory(prefix="local-map-default-") as directory:
            done, commands = launch_without_external_processes(directory, [])
            self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
            invocations = [argv for argv in commands if argv[0] == "scripts/run_dynanav_pct_scan.py"]
            self.assertEqual(len(invocations), 2)
            for argv in invocations:
                self.assertIn("--local-map-radius-m", argv)
                self.assertEqual(argv[argv.index("--local-map-radius-m") + 1], "25.0")

        with tempfile.TemporaryDirectory(prefix="local-map-override-") as directory:
            done, commands = launch_without_external_processes(directory, ["--local-map-radius-m", "0"])
            self.assertEqual(done.returncode, 0, done.stdout + done.stderr)
            invocations = [argv for argv in commands if argv[0] == "scripts/run_dynanav_pct_scan.py"]
            self.assertEqual(len(invocations), 2)
            for argv in invocations:
                self.assertEqual(argv.count("--local-map-radius-m"), 1)
                self.assertEqual(argv[argv.index("--local-map-radius-m") + 1], "0")


if __name__ == "__main__":
    unittest.main()
