"""CPU-only tests of the exact entrypoint lifecycle helpers, without ROS imports."""

import ast
from concurrent.futures import Future
from pathlib import Path
import signal
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from embodied_navigation.common.shutdown import ShutdownRequested, ShutdownSignals

ROOT = Path(__file__).resolve().parents[2]
SOURCES = [
    ROOT / "src/embodied_navigation/perception/sensors/sensor_gate.py",
    # The controller process entrypoint moved out of the former monolith; the
    # rclpy lifecycle assertions now read the module that owns them.
    ROOT / "src/embodied_navigation/navigation/pid_entrypoint.py",
]


def helpers(path):
    tree = ast.parse(path.read_text())
    names = {"_drain_executor"}
    selected = [
        n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name in names
    ]
    namespace = {}
    exec(
        compile(
            ast.fix_missing_locations(ast.Module(body=selected, type_ignores=[])), str(path), "exec"
        ),
        namespace,
    )
    return namespace


class ShutdownLifecycleTests(unittest.TestCase):
    def test_both_entries_disable_automatic_context_shutdown(self):
        for path in SOURCES:
            with self.subTest(path=path.name):
                tree = ast.parse(path.read_text())
                calls = [
                    n
                    for n in ast.walk(tree)
                    if isinstance(n, ast.Call) and ast.unparse(n.func) == "rclpy.init"
                ]
                self.assertEqual(len(calls), 1)
                options = {k.arg: ast.unparse(k.value) for k in calls[0].keywords}
                self.assertEqual(options["signal_handler_options"], "SignalHandlerOptions.NO")
                main = next(
                    n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "_main"
                )
                catches = [
                    ast.unparse(n.type)
                    for n in ast.walk(main)
                    if isinstance(n, ast.ExceptHandler) and n.type
                ]
                self.assertIn("_ShutdownRequested", catches)
                self.assertNotIn(
                    "(KeyboardInterrupt, rclpy.executors.ExternalShutdownException)", catches
                )
                self.assertNotIn("RuntimeError", catches)

    def test_only_signal_requests_normal_stop_and_repeat_does_not_interrupt_drain(self):
        for path in SOURCES:
            with self.subTest(path=path.name):
                with patch("signal.signal", return_value=signal.SIG_DFL) as install:
                    with ShutdownSignals() as shutdown:
                        self.assertFalse(shutdown.requested)
                        with self.assertRaises(ShutdownRequested):
                            shutdown.request(signal.SIGINT, None)
                        self.assertTrue(shutdown.requested)
                        self.assertTrue(shutdown.cleaning)
                        shutdown.request(signal.SIGTERM, None)
                    self.assertEqual(install.call_count, 4)

    def test_runtime_exception_is_not_suppressed_by_signal_scope(self):
        for path in SOURCES:
            with self.subTest(path=path.name), patch("signal.signal", return_value=signal.SIG_DFL):
                with self.assertRaisesRegex(RuntimeError, "real callback error"):
                    with ShutdownSignals() as shutdown:
                        raise RuntimeError("real callback error")

    def test_cleanup_signal_does_not_mask_existing_runtime_exception(self):
        for path in SOURCES:
            with self.subTest(path=path.name), patch("signal.signal", return_value=signal.SIG_DFL):
                with self.assertRaisesRegex(ValueError, "original failure"):
                    with ShutdownSignals() as shutdown:
                        try:
                            raise ValueError("original failure")
                        finally:
                            shutdown.begin_cleanup()
                            shutdown.request(signal.SIGTERM, None)

    def test_pool_drains_before_context_destruction_and_fetches_all_errors(self):
        drain = helpers(SOURCES[1])["_drain_executor"]
        events = []
        context = SimpleNamespace(alive=True)
        futures = [Future(), Future()]

        def join(wait):
            self.assertTrue(wait)
            self.assertTrue(context.alive)
            events.append("joined")
            futures[0].set_result(None)
            futures[1].set_exception(RuntimeError("real callback failure"))

        executor = SimpleNamespace(
            shutdown=lambda: events.append("executor_stopped"),
            _executor=SimpleNamespace(shutdown=join),
            _futures=futures,
            _pending_tasks={futures[1]: None},
        )
        with self.assertRaises(ExceptionGroup) as caught:
            drain(executor)
        self.assertEqual(events, ["executor_stopped", "joined"])
        self.assertEqual(len(caught.exception.exceptions), 1)
        self.assertEqual(str(caught.exception.exceptions[0]), "real callback failure")
        self.assertTrue(context.alive)

    def test_successful_callback_drain_has_no_error(self):
        drain = helpers(SOURCES[1])["_drain_executor"]
        future = Future()
        executor = SimpleNamespace(
            shutdown=lambda: None,
            _executor=SimpleNamespace(shutdown=lambda wait: future.set_result("finished")),
            _futures=[future],
            _pending_tasks={},
        )
        drain(executor)
        self.assertEqual(future.result(), "finished")


if __name__ == "__main__":
    unittest.main()
