"""Offline lifecycle checks for the DynaNav online-person supervisor.

These tests run real child processes (fake worker/observer/capture scripts) so
the supervision, timeout, early-exit, signal and cleanup paths are exercised
without ROS, Isaac, a GPU or a model.
"""

import contextlib
import importlib.util
import io
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest import mock
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts/run_dynanav_online_person.py"

FAKE_WORKER = '''\
import json, os, pathlib, signal, sys, time

channel = pathlib.Path(sys.argv[sys.argv.index("--channel") + 1])
dump = os.environ.get("FAKE_ENV_DUMP")
if dump:
    pathlib.Path(dump).write_text(
        json.dumps(
            {
                key: os.environ.get(key)
                for key in (
                    "PROJECT_DATA_ROOT",
                    "ROS_DOMAIN_ID",
                    "FAKE_ROS",
                    "PYTHONPATH",
                    "RMW_IMPLEMENTATION",
                )
            }
        )
    )
print("fake worker started", flush=True)
if os.environ.get("FAKE_WORKER") == "exit":
    sys.exit(2)
channel.mkdir(parents=True, exist_ok=True)
(channel / "state.json").write_text(json.dumps({"state": "ready"}))
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
while True:
    time.sleep(0.1)
'''

FAKE_OBSERVER = '''\
#!/usr/bin/env bash
set -u
out=""
while [ $# -gt 0 ]; do
  case "$1" in
    --output-dir) out="$2"; shift 2 ;;
    *) shift ;;
  esac
done
mkdir -p "$out"
printf '{}\\n' > "$out/launch.json"
echo "fake observer started"
if [ -n "${FAKE_OBSERVER_READY_DELAY:-}" ]; then sleep "$FAKE_OBSERVER_READY_DELAY"; fi
printf '{"state":"ready"}\\n' > "$out/ready.json"
trap 'exit 0' INT TERM
if [ -n "${FAKE_OBSERVER_LIFETIME:-}" ]; then
  sleep "$FAKE_OBSERVER_LIFETIME"
  exit 3
fi
while true; do sleep 0.1; done
'''

FAKE_CAPTURE = '''\
import json, os, pathlib, signal, sys, time

out = pathlib.Path(sys.argv[sys.argv.index("--output") + 1])
(out.parent / "capture-started").write_text(str(os.getpid()))
(out.parent / "capture-args.json").write_text(json.dumps(sys.argv[1:]))
print("fake capture started", flush=True)
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
time.sleep(float(sys.argv[sys.argv.index("--duration") + 1]))
code = int(os.environ.get("FAKE_CAPTURE_EXIT", "0"))
if code == 0:
    out.mkdir(parents=True, exist_ok=True)
    (out / "result.json").write_text(json.dumps({"outcome": "completed"}))
sys.exit(code)
'''

RUNNER = '''\
import importlib.util, sys
from pathlib import Path

spec = importlib.util.spec_from_file_location("supervisor_under_test", sys.argv[1])
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
fake = Path(sys.argv[2])
module.WORKER_SCRIPT = fake / "fake_worker.py"
module.OBSERVER_SCRIPT = fake / "fake_observer.sh"
module.CAPTURE_SCRIPT = fake / "fake_capture.py"
raise SystemExit(module.main(sys.argv[3:]))
'''


def load_module():
    spec = importlib.util.spec_from_file_location("online_person_supervisor", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SupervisorTest(unittest.TestCase):
    def setUp(self):
        self.module = load_module()
        self.temporary = tempfile.TemporaryDirectory()
        self.base = Path(self.temporary.name)
        self.fake = self.base / "fake"
        self.fake.mkdir()
        (self.fake / "fake_worker.py").write_text(FAKE_WORKER)
        (self.fake / "fake_capture.py").write_text(FAKE_CAPTURE)
        observer = self.fake / "fake_observer.sh"
        observer.write_text(FAKE_OBSERVER)
        observer.chmod(0o755)
        self.module.WORKER_SCRIPT = self.fake / "fake_worker.py"
        self.module.OBSERVER_SCRIPT = observer
        self.module.CAPTURE_SCRIPT = self.fake / "fake_capture.py"
        self.data = self.base / "data" / "embodied-navigation"
        self.run = self.base / "run"
        self.capture_result = self.base / "capture-result.json"
        self.slam_reference = self.base / "reference.json"
        self.capture_result.write_text("{}")
        self.slam_reference.write_text("{}")
        self.previous = {}
        for key in ("FAKE_ENV_DUMP", "FAKE_WORKER", "FAKE_CAPTURE_EXIT", "FAKE_OBSERVER_LIFETIME", "FAKE_OBSERVER_READY_DELAY"):
            if key in os.environ:
                self.previous[key] = os.environ.pop(key)

    def tearDown(self):
        for key in ("FAKE_ENV_DUMP", "FAKE_WORKER", "FAKE_CAPTURE_EXIT", "FAKE_OBSERVER_LIFETIME", "FAKE_OBSERVER_READY_DELAY"):
            os.environ.pop(key, None)
        os.environ.update(self.previous)
        self.temporary.cleanup()

    # -- helpers ---------------------------------------------------------
    def argv(self, *extra):
        return [
            "--output",
            str(self.run),
            "--data-root",
            str(self.data),
            "--weights",
            str(self.base / "weights.pth"),
            "--binary-prefix",
            str(self.base / "build_cache"),
            "--capture-result",
            str(self.capture_result),
            "--slam-reference",
            str(self.slam_reference),
            "--python",
            sys.executable,
            "--no-ros-setup",
            "--startup-timeout",
            "10",
            *extra,
        ]

    def run_supervisor(self, *extra):
        return self.module.main(self.argv(*extra))

    def supervision(self):
        return json.loads((self.run / "supervision.json").read_text())

    @staticmethod
    def assert_dead(test, pid):
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return
        test.fail("process " + str(pid) + " is still alive")

    # -- pure plan / read-only modes ------------------------------------
    def test_plan_keeps_worker_observer_capture_order_and_wiring(self):
        args = self.module.parser().parse_args(self.argv())
        args.data_root = self.data
        args.weights = self.base / "weights.pth"
        args.binary_prefix = self.base / "build_cache"
        plan = self.module.build_plan(args)
        worker, observer, capture = plan["worker"], plan["observer_argv"], plan["capture_argv"]
        self.assertTrue(worker[1].endswith("fake_worker.py"))
        self.assertEqual(worker[worker.index("--channel") + 1], str(plan["channel"]))
        self.assertEqual(observer[observer.index("--mode") + 1], "livo")
        self.assertEqual(observer[observer.index("--lidar-topic") + 1], "/dynanav/lidar_perception_filtered")
        self.assertEqual(observer[observer.index("--output-dir") + 1], str(plan["observer"]))
        self.assertEqual(capture[capture.index("--person-worker") + 1], str(plan["channel"]))
        self.assertEqual(capture[capture.index("--pose-launch-receipt") + 1], str(plan["launch_receipt"]))
        self.assertEqual(capture[capture.index("--semantic-provider") + 1], "learned_rgb_person")

    def test_snapshot_parameters_are_bounded_and_inside_the_run(self):
        args = self.module.parser().parse_args(self.argv())
        args.data_root = self.data
        args.weights = self.base / "weights.pth"
        args.binary_prefix = self.base / "build_cache"
        plan = self.module.build_plan(args)
        text = self.module.snapshots_yaml(plan["run"])
        self.assertIn(str(plan["observer"] / "native_keyframes"), text)
        self.assertIn("max_ready_directories: 256", text)
        self.assertEqual(self.module.snapshot_parameter_file(plan["run"]), plan["run"] / "livo" / "snapshots.yaml")

    def test_print_commands_without_required_paths_creates_nothing(self):
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            code = self.module.main(
                [
                    "--output",
                    str(self.run),
                    "--data-root",
                    str(self.data),
                    "--python",
                    sys.executable,
                    "--no-ros-setup",
                    "--print-commands",
                ]
            )
        self.assertEqual(code, 0)
        text = stream.getvalue()
        self.assertIn(self.module.MISSING_CAPTURE_RESULT, text)
        self.assertIn(self.module.MISSING_SLAM_REFERENCE, text)
        self.assertIn("mkdir -p", text)
        self.assertLess(text.index("mkdir -p"), text.index("cat >"))
        self.assertIn("# 1) worker", text)
        self.assertIn("# 3) capture", text)
        self.assertFalse(self.run.exists())

    def test_check_without_required_paths_reports_missing_and_creates_nothing(self):
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            code = self.module.main(
                [
                    "--output",
                    str(self.run),
                    "--data-root",
                    str(self.data),
                    "--python",
                    sys.executable,
                    "--no-ros-setup",
                    "--check",
                ]
            )
        self.assertEqual(code, 2)
        report = json.loads(stream.getvalue())
        self.assertFalse(report["ready"])
        missing = {entry["name"] for entry in report["entries"] if not entry["present"]}
        self.assertIn("capture result (observer calibration)", missing)
        self.assertIn("SLAM reference", missing)
        self.assertIn("weights", missing)
        self.assertFalse(self.run.exists())

    def test_check_accepts_existing_inputs(self):
        weights = self.base / "weights.pth"
        weights.write_bytes(b"weights")
        (self.base / "build_cache").mkdir()
        self.data.mkdir(parents=True)
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            code = self.module.main(
                [
                    "--output",
                    str(self.run),
                    "--data-root",
                    str(self.data),
                    "--weights",
                    str(weights),
                    "--binary-prefix",
                    str(self.base / "build_cache"),
                    "--capture-result",
                    str(self.capture_result),
                    "--slam-reference",
                    str(self.slam_reference),
                    "--python",
                    sys.executable,
                    "--no-ros-setup",
                    "--check",
                ]
            )
        self.assertEqual(code, 0)
        self.assertTrue(json.loads(stream.getvalue())["ready"])
        self.assertFalse(self.run.exists())

    def test_run_without_required_paths_is_an_argparse_error(self):
        with (
            contextlib.redirect_stderr(io.StringIO()),
            self.assertRaises(SystemExit) as raised,
        ):
            self.module.main(
                ["--output", str(self.run), "--data-root", str(self.data), "--no-ros-setup"]
            )
        self.assertEqual(raised.exception.code, 2)
        self.assertFalse(self.run.exists())

    # -- real child lifecycles ------------------------------------------
    def test_success_stops_children_and_keeps_logs_and_environment(self):
        dump = self.base / "env.json"
        os.environ["FAKE_ENV_DUMP"] = str(dump)
        setup = self.base / "fake_setup.sh"
        setup.write_text("export FAKE_ROS=1\n")
        code = self.module.main(
            [item for item in self.argv("--duration", "0.5") if item not in ("--no-ros-setup",)]
            + ["--ros-setup", str(setup)]
        )
        self.assertEqual(code, 0)
        report = self.supervision()
        self.assertTrue(report["ok"])
        self.assertEqual(report["schema"], "dynanav_online_supervision/v3")
        self.assertEqual(report["error"], None)
        self.assertEqual(report["capture_exit_code"], 0)
        names = {record["name"]: record for record in report["processes"]}
        self.assertEqual(set(names), {"worker", "observer", "capture"})
        self.assertTrue(names["worker"]["stopped_by_supervisor"])
        self.assertTrue(names["observer"]["stopped_by_supervisor"])
        self.assertFalse(names["capture"]["stopped_by_supervisor"])
        self.assertEqual(report["unexpected_exits"], [])
        expected = {
            "worker": "fake worker started",
            "observer": "fake observer started",
            "capture": "fake capture started",
        }
        for record in names.values():
            log = Path(record["log"])
            self.assertTrue(log.is_file(), str(log))
            self.assertIn(expected[record["name"]], log.read_text())
        environment = json.loads(dump.read_text())
        self.assertEqual(environment["PROJECT_DATA_ROOT"], str(self.data.parent))
        self.assertEqual(environment["ROS_DOMAIN_ID"], "116")
        self.assertEqual(environment["FAKE_ROS"], "1")
        self.assertIn(str(ROOT / "src"), environment["PYTHONPATH"])

    def test_zero_capture_exit_requires_completed_receipt(self):
        path = self.base / "result.json"
        for receipt in (None, "broken JSON", "[]", '{}',
                        '{"outcome":"running"}', '{"outcome":"failed"}',
                        '{"outcome":"running","stage":"completed"}'):
            with self.subTest(receipt=receipt):
                if receipt is not None:
                    path.write_text(receipt)
                with self.assertRaisesRegex(RuntimeError, "capture exited 0"):
                    self.module.validate_capture_result(path)
        for receipt in ({"outcome": "completed"}, {"stage": "completed"}):
            path.write_text(json.dumps(receipt))
            self.module.validate_capture_result(path)

    def test_sdk_zero_exit_with_running_result_is_technical_failure(self):
        fake = self.fake / "fake_capture.py"
        fake.write_text(FAKE_CAPTURE.replace('{"outcome": "completed"}', '{"outcome": "running"}'))
        self.assertEqual(self.run_supervisor("--duration", "0.1"), 1)
        report = self.supervision()
        self.assertFalse(report["ok"])
        self.assertEqual(report["capture_exit_code"], 0)
        self.assertIn("result.json is not completed", report["error"])
        self.assertEqual(report["cleanup_errors"], [])
        for child in report["processes"]:
            self.assert_dead(self, child["pid"])

    def test_effective_environment_records_only_diagnostic_whitelist(self):
        expected = {
            "RMW_IMPLEMENTATION": "test_rmw",
            "FASTDDS_BUILTIN_TRANSPORTS": "TEST_TRANSPORT",
            "DYNANAV_PUBLISH_TRACE": "1",
            "FASTLIVO_INGRESS_TRACE": "/tmp/test-trace",
        }
        with mock.patch.dict(os.environ, {**expected, "UNRELATED_SECRET": "must-not-record"}):
            self.assertEqual(self.run_supervisor("--duration", "0.1"), 0)
        report = self.supervision()
        self.assertEqual(report["effective_environment"], expected)
        self.assertNotIn("must-not-record", json.dumps(report))

    def test_capture_failure_is_reported_and_children_are_stopped(self):
        os.environ["FAKE_CAPTURE_EXIT"] = "3"
        code = self.run_supervisor("--duration", "0.3")
        self.assertEqual(code, 1)
        report = self.supervision()
        self.assertFalse(report["ok"])
        self.assertEqual(report["capture_exit_code"], 3)
        capture = {r["name"]: r for r in report["processes"]}["capture"]
        self.assert_dead(self, capture["pid"])

    def test_capture_timeout_stops_the_capture_process(self):
        code = self.run_supervisor("--duration", "30", "--capture-timeout", "0.4")
        self.assertEqual(code, 1)
        report = self.supervision()
        self.assertFalse(report["ok"])
        self.assertIn("capture exceeded --capture-timeout", report["error"])
        capture = {r["name"]: r for r in report["processes"]}["capture"]
        self.assertTrue(capture["stopped_by_supervisor"])
        self.assert_dead(self, capture["pid"])

    def test_worker_early_exit_is_detected_before_observer_starts(self):
        os.environ["FAKE_WORKER"] = "exit"
        code = self.run_supervisor("--duration", "30")
        self.assertEqual(code, 1)
        report = self.supervision()
        self.assertFalse(report["ok"])
        self.assertIn("worker exited with 2 before readiness", report["error"])
        self.assertEqual([r["name"] for r in report["processes"]], ["worker"])
        self.assertTrue((self.run / "logs" / "worker.log").is_file())

    def test_observer_early_exit_during_capture_is_detected(self):
        os.environ["FAKE_OBSERVER_LIFETIME"] = "0.5"
        code = self.run_supervisor("--duration", "30", "--capture-timeout", "15")
        self.assertEqual(code, 1)
        report = self.supervision()
        self.assertFalse(report["ok"])
        self.assertIn("observer exited with 3 while capture was running", report["error"])
        records = {r["name"]: r for r in report["processes"]}
        self.assertTrue(records["capture"]["stopped_by_supervisor"])
        self.assert_dead(self, records["capture"]["pid"])

    def test_launch_receipt_alone_does_not_start_capture(self):
        os.environ["FAKE_OBSERVER_READY_DELAY"] = "2"
        code = self.run_supervisor("--startup-timeout", "0.3")
        self.assertEqual(code, 1)
        self.assertTrue((self.run / "livo/observer/launch.json").exists())
        self.assertFalse((self.run / "capture-started").exists())
        self.assertIn("did not become ready", self.supervision()["error"])

    def test_partial_startup_spawn_failure_still_stops_worker(self):
        original = self.module.spawn
        def fail_observer(name, *args):
            if name == "observer":
                raise OSError("injected spawn failure")
            return original(name, *args)
        with mock.patch.object(self.module, "spawn", side_effect=fail_observer):
            self.assertEqual(self.run_supervisor(), 1)
        report = self.supervision()
        self.assertIn("injected spawn failure", report["error"])
        self.assert_dead(self, report["processes"][0]["pid"])

    def test_cancel_during_spawn_registers_child_before_cleanup(self):
        original = self.module.spawn
        def cancel_after_spawn(*args):
            child = original(*args)
            os.kill(os.getpid(), signal.SIGTERM)
            return child
        with mock.patch.object(self.module, "spawn", side_effect=cancel_after_spawn):
            self.assertEqual(self.run_supervisor(), 143)
        report = self.supervision()
        self.assertEqual(len(report["processes"]), 1)
        self.assert_dead(self, report["processes"][0]["pid"])

    def test_worker_death_while_observer_starts_prevents_capture(self):
        original = self.module.spawn
        worker = None
        def kill_worker_after_observer(name, *args):
            nonlocal worker
            child = original(name, *args)
            if name == "worker":
                worker = child
            elif name == "observer":
                worker.process.kill()
                worker.process.wait(timeout=5)
            return child
        with mock.patch.object(self.module, "spawn", side_effect=kill_worker_after_observer):
            self.assertEqual(self.run_supervisor(), 1)
        self.assertFalse((self.run / "capture-started").exists())
        self.assertIn("worker exited", self.supervision()["error"])

    def test_cleanup_error_does_not_skip_remaining_children_or_receipt(self):
        original = self.module.stop
        stopped = []
        def fail_after_stop(child):
            original(child)
            stopped.append(child.name)
            if child.name == "capture":
                raise OSError("injected cleanup error")
        with mock.patch.object(self.module, "stop", side_effect=fail_after_stop):
            self.assertEqual(self.run_supervisor("--duration", "0.1"), 1)
        self.assertEqual(stopped, ["capture", "observer", "worker"])
        self.assertIn("injected cleanup error", self.supervision()["error"])

    def test_repeated_cancellation_during_cleanup_preserves_receipt(self):
        original = self.module.stop
        def cancel_then_stop(child):
            os.kill(os.getpid(), signal.SIGTERM)
            os.kill(os.getpid(), signal.SIGINT)
            original(child)
        with mock.patch.object(self.module, "stop", side_effect=cancel_then_stop):
            self.assertEqual(self.run_supervisor("--duration", "0.1"), 143)
        self.assertEqual(self.supervision()["interrupted"], signal.SIGTERM)
        for child in self.supervision()["processes"]:
            self.assert_dead(self, child["pid"])

    def test_dead_leader_does_not_leave_ignoring_grandchild_alive(self):
        pidfile = self.base / "grandchild.pid"
        code = ("import subprocess,sys; p=subprocess.Popen([sys.executable,'-c',"
                "'import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); time.sleep(60)']); "
                "open(sys.argv[1],'w').write(str(p.pid))")
        child = self.module.spawn("orphan", [sys.executable, "-c", code, str(pidfile)],
                                  dict(os.environ), self.base)
        child.process.wait(timeout=5)
        pid = int(pidfile.read_text())
        try:
            self.module.stop(child, grace=0.3)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                stat = Path(f"/proc/{pid}/stat")
                if not stat.exists() or stat.read_text().rsplit(")", 1)[1].split()[0] == "Z":
                    break
                time.sleep(0.05)
            else:
                self.fail("grandchild survived process-group cleanup")
            self.assertFalse(child.stopped_by_supervisor)
            self.assertEqual(child.exit_code, 0)
        finally:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def test_monitor_capture_does_not_mask_an_already_dead_supporting_child(self):
        # Real Child wrappers around already-exited fake processes: capture is 0
        # while the observer has already failed with 7 (the old poll order
        # returned 0 before looking at the observer).
        stream = io.BytesIO()

        def finished(code):
            process = subprocess.Popen(
                [sys.executable, "-c", "import sys; sys.exit(" + str(code) + ")"]
            )
            process.wait(timeout=10)
            return process

        capture = self.module.Child(
            "capture", ["fake"], finished(0), self.base / "capture.log", stream
        )
        observer = self.module.Child(
            "observer", ["fake"], finished(7), self.base / "observer.log", io.BytesIO()
        )
        with self.assertRaises(self.module.ChildExit) as raised:
            self.module.monitor_capture(capture, [observer], 10)
        self.assertIn("observer exited with 7", str(raised.exception))

    def test_final_verdict_accepts_normal_sigint_cleanup(self):
        capture = types.SimpleNamespace(exit_code=0)
        children = [
            types.SimpleNamespace(name="worker", exit_code=0, stopped_by_supervisor=True),
            types.SimpleNamespace(name="observer", exit_code=0, stopped_by_supervisor=True),
        ]
        ok, unexpected, error = self.module.final_verdict(capture, children, None, None)
        self.assertTrue(ok)
        self.assertEqual(unexpected, [])
        self.assertIsNone(error)

    def test_final_verdict_rejects_completed_capture_with_failed_observer(self):
        capture = types.SimpleNamespace(exit_code=0)
        children = [
            types.SimpleNamespace(name="worker", exit_code=0, stopped_by_supervisor=True),
            types.SimpleNamespace(name="observer", exit_code=7, stopped_by_supervisor=False),
        ]
        ok, unexpected, error = self.module.final_verdict(capture, children, None, None)
        self.assertFalse(ok)
        self.assertEqual(unexpected, [{"name": "observer", "exit_code": 7}])
        self.assertIn("observer exited with 7", error)

    def test_final_verdict_rejects_completed_capture_with_failed_worker(self):
        capture = types.SimpleNamespace(exit_code=0)
        children = [
            types.SimpleNamespace(name="worker", exit_code=9, stopped_by_supervisor=False),
            types.SimpleNamespace(name="observer", exit_code=0, stopped_by_supervisor=True),
        ]
        ok, unexpected, error = self.module.final_verdict(capture, children, None, None)
        self.assertFalse(ok)
        self.assertEqual(unexpected, [{"name": "worker", "exit_code": 9}])
        self.assertIn("worker exited with 9", error)

    def test_final_verdict_rejects_nonzero_capture_and_interrupts(self):
        children = [
            types.SimpleNamespace(name="worker", exit_code=0, stopped_by_supervisor=True),
            types.SimpleNamespace(name="observer", exit_code=0, stopped_by_supervisor=True),
        ]
        ok, _, _ = self.module.final_verdict(
            types.SimpleNamespace(exit_code=3), children, None, None
        )
        self.assertFalse(ok)
        ok, _, _ = self.module.final_verdict(
            types.SimpleNamespace(exit_code=0), children, None, signal.SIGTERM
        )
        self.assertFalse(ok)
        ok, _, error = self.module.final_verdict(
            types.SimpleNamespace(exit_code=0), children, "child failed", None
        )
        self.assertFalse(ok)
        self.assertEqual(error, "child failed")

    def test_sigterm_stops_children_and_records_the_interrupt(self):
        runner = self.base / "runner.py"
        runner.write_text(RUNNER)
        process = subprocess.Popen(
            [
                sys.executable,
                str(runner),
                str(SCRIPT),
                str(self.fake),
                *self.argv("--duration", "30", "--capture-timeout", "30"),
            ],
            cwd=str(ROOT),
        )
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if (self.run / "capture-started").is_file():
                break
            if process.poll() is not None:
                self.fail("supervisor exited before capture started: " + str(process.returncode))
            time.sleep(0.05)
        else:
            process.kill()
            self.fail("capture never started")
        process.send_signal(signal.SIGTERM)
        self.assertEqual(process.wait(timeout=20), 128 + signal.SIGTERM)
        report = self.supervision()
        self.assertEqual(report["interrupted"], signal.SIGTERM)
        self.assertFalse(report["ok"])
        capture = {r["name"]: r for r in report["processes"]}["capture"]
        self.assertTrue(capture["stopped_by_supervisor"])
        self.assert_dead(self, capture["pid"])


class ObserverLauncherLifecycleTest(unittest.TestCase):
    """Execute the real launcher lifecycle with fake nodes and ROS graph only.

    Preflight and calibration validation have separate regression coverage;
    these tests inject failures after those checks, without starting ROS.
    """

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.base = Path(self.temporary.name)
        self.output = self.base / "observer"
        self.output.mkdir()
        package = self.base / "rclpy"
        package.mkdir()
        (package / "__init__.py").write_text(
            "import time\n"
            "def init(args): pass\n"
            "def shutdown(): pass\n"
            "def spin_once(node, timeout_sec): time.sleep(timeout_sec)\n"
            "class Node:\n"
            " def __init__(self): self.calls=0\n"
            " def get_subscriber_names_and_types_by_node(self,*args):\n"
            "  self.calls+=1\n"
            "  if self.calls==1:\n"
            "   from .node import NodeNameNonExistentError\n"
            "   raise NodeNameNonExistentError()\n"
            "  return [(t,[]) for t in ['/dynanav/lidar_static','/dynanav/imu','/dynanav/camera']]\n"
            " def destroy_node(self): pass\n"
            "def create_node(name): return Node()\n"
        )
        (package / "node.py").write_text("class NodeNameNonExistentError(Exception): pass\n")
        self.node = self.base / "fake_node.py"
        self.node.write_text(
            "import os,signal,sys,time\n"
            "open(sys.argv[1], 'w').write(str(os.getpid()))\n"
            "signal.signal(signal.SIGTERM, signal.SIG_IGN if sys.argv[3]=='ignore' else lambda *_: sys.exit(0))\n"
            "time.sleep(float(sys.argv[2]))\n"
            "sys.exit(7)\n"
        )
        self.process = None

    def tearDown(self):
        if self.process is not None:
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.process.wait(timeout=5)
            self.process.stderr.close()
        self.temporary.cleanup()

    def start(self, mapper_lifetime="60", blackboard_lifetime="60", mapper_signal="normal"):
        import shlex
        q = shlex.quote
        source = (ROOT / "scripts/run_dynanav_lio.sh").read_text()
        lifecycle = source[source.index("# Only PIDs created below"):]
        preflight = json.dumps({"observer_namespace": "/test", "mode": "livo",
                                "lidar_input_topic": "/dynanav/lidar_static"})
        # Only the already separately tested camera RPC is stubbed. The real
        # readiness Python, traps, wait -n and terminal receipt all execute.
        harness = "\n".join([
            "set -euo pipefail", "mode=livo", "output_dir=" + q(str(self.output)),
            "preflight_json=" + q(preflight),
            "python3() { if [[ $2 == */camera_parameter_ready.json ]]; then return 0; fi; command " + q(sys.executable) + " \"$@\"; }",
            "command=(" + " ".join(map(q, [sys.executable, str(self.node), str(self.base / "mapper.pid"), mapper_lifetime, mapper_signal])) + ")",
            "blackboard_command=(" + " ".join(map(q, [sys.executable, str(self.node), str(self.base / "blackboard.pid"), blackboard_lifetime, "normal"])) + ")",
            lifecycle,
        ])
        env = dict(os.environ, PYTHONPATH=str(self.base))
        self.process = subprocess.Popen(["bash", "-c", harness], env=env,
                                        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                        start_new_session=True)

    def assert_owned_stopped(self):
        for name in ("mapper", "blackboard"):
            file = self.base / (name + ".pid")
            if file.exists():
                SupervisorTest.assert_dead(self, int(file.read_text()))
        self.assertFalse((self.output / "ready.json").exists())
        return json.loads((self.output / "shutdown.json").read_text())

    def test_blackboard_death_stops_mapper_and_writes_terminal_receipt(self):
        self.start(blackboard_lifetime="0.8")
        self.assertEqual(self.process.wait(timeout=10), 7)
        self.assertEqual(self.assert_owned_stopped()["exit_code"], 7)

    def test_mapper_failure_before_readiness_stops_blackboard(self):
        self.start(mapper_lifetime="0")
        self.assertNotEqual(self.process.wait(timeout=10), 0)
        self.assertNotEqual(self.assert_owned_stopped()["exit_code"], 0)

    def test_repeated_cancel_kills_term_ignoring_mapper_within_bound(self):
        self.start(mapper_signal="ignore")
        deadline = time.monotonic() + 5
        while not (self.output / "ready.json").exists():
            self.assertIsNone(self.process.poll())
            self.assertLess(time.monotonic(), deadline)
            time.sleep(0.05)
        self.process.send_signal(signal.SIGTERM)
        time.sleep(0.2)
        self.process.send_signal(signal.SIGINT)
        self.assertEqual(self.process.wait(timeout=8), 143)
        self.assertEqual(self.assert_owned_stopped()["exit_code"], 143)


if __name__ == "__main__":
    unittest.main()
