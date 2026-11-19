"""无 ROS 节点的退出并发回归：使用真实地图、位姿图与 trace 写盘。"""
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe
from embodied_navigation.localization_mapping.backend.map_store import MapStore
from embodied_navigation.localization_mapping.backend.trace import BackendTrace


class NodeStub:
    def __init__(self, name, *, parameter_overrides):
        self.parameter_overrides = parameter_overrides
        self.create_publisher = Mock(side_effect=lambda *a, **kw: Mock())
        self.create_subscription = Mock()
        self.create_timer = Mock(return_value=Mock())
        self.get_clock = Mock()
        self.get_logger = Mock()


def load_node():
    """仅替换 ROS 绑定；执行原模块的全部 Python 生命周期实现。"""
    bindings = {
        "rclpy": {},
        "rclpy.node": {"Node": NodeStub},
        "rclpy.parameter": {"Parameter": lambda name, **kw: SimpleNamespace(name=name, **kw)},
        "rclpy.callback_groups": {"MutuallyExclusiveCallbackGroup": Mock()},
        "rclpy.executors": {"MultiThreadedExecutor": Mock()},
        "rclpy.signals": {"SignalHandlerOptions": SimpleNamespace(NO=0)},
        "geometry_msgs.msg": {"TransformStamped": Mock()},
        "nav_msgs.msg": {"Odometry": Mock()},
        "std_msgs.msg": {"String": lambda **kw: SimpleNamespace(**kw)},
        "tf2_ros": {"TransformBroadcaster": Mock()},
    }
    modules = {}
    for name, fields in bindings.items():
        module = ModuleType(name)
        module.__dict__.update(fields)
        modules[name] = module
    name = "embodied_navigation.localization_mapping.backend._shutdown_test_node"
    source = Path(__file__).resolve().parents[2] / "src/embodied_navigation/localization_mapping/backend/node.py"
    spec = importlib.util.spec_from_file_location(name, source)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class BackendShutdownTest(unittest.TestCase):
    def test_clock_parameter_is_set_at_node_construction(self):
        module = load_node()
        with tempfile.TemporaryDirectory() as tmp:
            required = ["--spool", tmp, "--odometry-topic", "/odom", "--namespace", "/slam"]
            for extra, expected in (([], False), (["--use-sim-time"], True)):
                with self.subTest(use_sim_time=expected):
                    args = module.parse_args(required + extra)
                    self.assertIs(args.use_sim_time, expected)
                    node = module.BackendNode(args, Mock())
                    self.assertEqual(len(node.parameter_overrides), 1)
                    parameter = node.parameter_overrides[0]
                    self.assertEqual(parameter.name, "use_sim_time")
                    self.assertIs(parameter.value, expected)

    def test_close_waits_for_complete_ingest_and_rejects_queued_poll(self):
        module = load_node()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            spool = root / "spool"
            spool.mkdir()
            directories = [spool / f"kf_1_{i}" for i in (1, 2)]
            for directory in directories:
                directory.mkdir()
            config = BackendConfig()
            trace = BackendTrace(root / "trace", config)
            backend = SlamBackend(config, trace=trace)
            args = SimpleNamespace(spool=spool, namespace="/slam", odometry_topic="/odom",
                                   use_sim_time=False, poll_s=.2, save_map=root / "map", world_frame="map", odom_frame="odom")
            node = module.BackendNode(args, backend)
            frame = Keyframe((1, 1), 123456789, np.eye(4), np.eye(4),
                             np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]))
            entered, release, cancelled, waiting = (threading.Event() for _ in range(4))
            poll_lock = node._poll_lock

            class ObservedLock:
                def __enter__(self):
                    if poll_lock.locked():
                        waiting.set()
                    poll_lock.acquire()

                def __exit__(self, *exc):
                    poll_lock.release()

            node._poll_lock = ObservedLock()
            node._poll_timer.cancel.side_effect = cancelled.set
            original_add = backend.map.add

            def interrupted_add(keyframe, pose, descriptor):
                # 复现 keyframes 已写而 pose/descriptor 尚未提交的窗口。
                backend.map.keyframes[keyframe.key] = keyframe
                entered.set()
                if not release.wait(5):
                    raise TimeoutError("test did not release ingest")
                original_add(keyframe, pose, descriptor)

            with patch.object(module, "ready_directories", return_value=directories), \
                 patch.object(module, "read_keyframe", return_value=frame) as read, \
                 patch.object(backend.map, "add", side_effect=interrupted_add), \
                 ThreadPoolExecutor(max_workers=4) as workers:
                active = workers.submit(node.poll)
                try:
                    self.assertTrue(entered.wait(5), "ingest did not enter map mutation")
                    closing = workers.submit(node.close)
                    self.assertTrue(cancelled.wait(5), "close did not stop timer")
                    self.assertTrue(waiting.wait(5), "close did not wait for the active ingest")
                    queued = workers.submit(node.poll)
                    self.assertFalse(closing.done(), "save raced with incomplete ingest")
                    self.assertFalse((args.save_map / "map.json").exists())
                    self.assertFalse((trace.directory / "summary.json").exists())
                    # 里程计回调必须仍可与正在处理关键帧的 poll 并行。
                    with patch.object(module, "pose_matrix", return_value=np.eye(4)), \
                         patch.object(module, "write_pose"):
                        message = SimpleNamespace(header=SimpleNamespace(), pose=SimpleNamespace(pose=object()))
                        workers.submit(node.on_odometry, message).result(timeout=5)
                    node.odometry_pub.publish.assert_called_once_with(message)
                finally:
                    release.set()
                active.result(timeout=5)
                closing.result(timeout=5)
                queued.result(timeout=5)
                node.poll()
                node.close()
                self.assertEqual(read.call_count, 1)

            saved = MapStore.load(args.save_map)
            self.assertEqual(list(saved.keyframes), [frame.key])
            np.testing.assert_array_equal(saved.poses[frame.key], backend.graph.pose(frame.key))
            np.testing.assert_array_equal(saved.descriptors[frame.key], backend.map.descriptors[frame.key])
            rows = [json.loads(row) for row in (trace.directory / "events.jsonl").read_text().splitlines()]
            self.assertEqual([row["event"] for row in rows].count("final_graph"), 1)
            self.assertEqual(rows[-1]["event"], "final_graph")
            self.assertEqual(rows[-1]["graph"]["keys"], [[1, 1]])
            np.testing.assert_array_equal(rows[-1]["graph"]["poses"][0], saved.poses[frame.key])
            summary = json.loads((trace.directory / "summary.json").read_text())
            self.assertEqual(summary["submitted"], summary["written"])
            self.assertIsNone(summary["error"])
            self.assertTrue(directories[1].exists(), "shutdown must not ingest queued frames")

    def test_save_failure_still_records_final_graph_and_closes_trace(self):
        module = load_node()
        with tempfile.TemporaryDirectory() as tmp:
            config = BackendConfig()
            trace = BackendTrace(Path(tmp) / "trace", config)
            backend = SlamBackend(config, trace=trace)
            args = SimpleNamespace(spool=Path(tmp), namespace="/slam", odometry_topic="/odom",
                                   use_sim_time=False, poll_s=.2, save_map=Path(tmp) / "map")
            node = module.BackendNode(args, backend)
            with patch.object(backend, "save_map", side_effect=OSError("disk full")):
                with self.assertRaisesRegex(OSError, "disk full"):
                    node.close()
            rows = [json.loads(row) for row in (trace.directory / "events.jsonl").read_text().splitlines()]
            self.assertEqual(rows[-1]["event"], "final_graph")
            self.assertTrue((trace.directory / "summary.json").is_file())
            node.poll()

    def test_main_closes_before_executor_and_always_cleans_ros(self):
        module = load_node()
        for failure in (None, OSError("disk full")):
            with self.subTest(failure=failure):
                order = []
                node, executor = Mock(), Mock()
                node.close.side_effect = lambda: order.append("close")
                if failure:
                    def fail_close():
                        order.append("close")
                        raise failure
                    node.close.side_effect = fail_close
                node.destroy_node.side_effect = lambda: order.append("destroy")
                executor.spin_once.side_effect = KeyboardInterrupt
                executor.shutdown.side_effect = lambda: order.append("executor")
                module.rclpy.init = Mock()
                module.rclpy.shutdown = Mock(side_effect=lambda: order.append("ros"))
                args = SimpleNamespace(prior_map=None, trace_dir=None)
                with patch.object(module, "parse_args", return_value=args), \
                     patch.object(module, "BackendNode", return_value=node), \
                     patch.object(module, "MultiThreadedExecutor", return_value=executor), \
                     patch.object(module.signal, "signal"):
                    with self.assertRaises(OSError if failure else KeyboardInterrupt):
                        module.main([])
                self.assertEqual(order, ["close", "executor", "destroy", "ros"])


if __name__ == "__main__":
    unittest.main()
