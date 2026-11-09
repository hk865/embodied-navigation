"""Orchestration-level checks for the split DynaNav capture entry.

These tests run the real ``capture_go2_rgbd.run_capture`` orchestration with fake
SDK/ROS/PIL/physics dependencies (zero or one physics step). They do not start
Isaac Sim, ROS, a model, a GPU or ffmpeg; they exist because the responsibility
split moved ownership/cleanup into explicit objects, where a missing binding or
a cleanup error can only be caught by executing the real orchestration.
"""

import contextlib
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import types
import unittest
import unittest.mock
from pathlib import Path

import numpy as np

import embodied_navigation.simulation.dynanav.capture_go2_rgbd as entry


class SentinelError(RuntimeError):
    """Primary failure injected into the fake first tick."""


class _NumpyVector:
    """Minimal ``.numpy()`` wrapper matching the SDK return values."""

    def __init__(self, values):
        self._values = np.asarray(values, dtype=float)

    def numpy(self):
        return self._values


class FakeReceipts:
    """Receipts whose forced save fails; normal throttled saves are no-ops."""

    def __init__(self, output, experiment, gt_diagnostic, slam_control):
        self.output = output
        self.result = {"stage": "startup", "truth_usage": "fake"}
        self.force_failures = 0

    def save(self, force=False):
        if force:
            self.force_failures += 1
            raise OSError("receipt write failed")

    def journal(self, name, payload):
        return None

    def close_journals(self):
        return None


class FakeApp:
    def __init__(self):
        self.close_calls = []
        self.stop_calls = 0
        self.close_error = None

    def finish(self):
        self.stop_calls += 1

    def close(self, exit_code=None):
        self.close_calls.append(exit_code)
        if self.close_error is not None:
            raise self.close_error


class FakeCameraRig:
    def __init__(self):
        self.theta = 0.0
        self.mount = np.array([0.10, 0.0, 0.20])
        self.imu_mount = np.array([0.05, 0.0, 0.10])


class FakeContactMonitor:
    def __init__(self):
        self.contact_state = [0, 0, 0, 0, 0]
        self.contact_events = []
        self.dynamic_contact_events = []
        self.contact_paths = {}
        self.callback_count = 0

    def install(self, stage):
        return None

    def begin_step(self):
        return None


class FakeMapping:
    def __init__(self):
        self.learned_mapper = None
        self.learned_scorer = None
        self.finish_calls = 0
        self.finalize_failure_calls = 0
        self.finalize_failure_error = None

    def finish(self, result):
        self.finish_calls += 1
        result["mapping"] = {"fake": True}

    def finalize_failure(self, result):
        self.finalize_failure_calls += 1
        if self.finalize_failure_error is not None:
            raise self.finalize_failure_error
        result["mapping"] = {"partial": True}


class FakeVideo:
    def __init__(self):
        self.frames = 0
        self.returncode = 0
        self.close_calls = []
        self.close_error = None

    def write(self, *args, **kwargs):
        self.frames += 1

    def close(self, timeout=15):
        self.close_calls.append(timeout)
        if self.close_error is not None:
            raise self.close_error


class FakeArchive:
    def __init__(self):
        self.receipt_calls = 0
        self.receipt_error = None

    def receipt(self, **kwargs):
        self.receipt_calls += 1
        if self.receipt_error is not None:
            raise self.receipt_error
        return {"fake": True}


class FakeRos:
    def __init__(self):
        self.published = {"rgb": 0}
        self.latest_odom = [None]
        self.latest_nav = [None]
        self.destroy_calls = 0
        self.spin_calls = 0

    def destroy(self):
        self.destroy_calls += 1

    def spin_once(self, timeout_sec=0.0):
        self.spin_calls += 1

    def publish_imu_and_clock(self, *args):
        return None


class FakeArt:
    def __init__(self):
        self.dof_names = [f"joint_{index}" for index in range(12)]

    def get_dof_positions(self):
        return _NumpyVector([np.zeros(12)])

    def get_dof_velocities(self):
        return _NumpyVector([np.zeros(12)])


class FakeMeasurement:
    def __init__(self):
        self.tau = np.zeros(12)
        self.it = 0.001
        self.actual_time = 0.005
        self.reading = object()
        self.pos = np.zeros(3)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.v = _NumpyVector([[0.0, 0.0, 0.0]])
        self.w = _NumpyVector([[0.0, 0.0, 0.0]])
        self.previous_completed_tau = np.zeros(12)


class FakeStepper:
    """Fake stepper; appends an IMU row like the real one, then may fail."""

    def __init__(self, measurement, step_error=None, fail_after_step=False):
        self.measurement = measurement
        self.step_error = step_error
        self.fail_after_step = fail_after_step
        self.calls = []
        self.kwargs = {}

    def step(self, *, iteration, tau, q, qd, contact_state, sensor_rows):
        self.calls.append(iteration)
        self.measurement.previous_completed_tau = np.zeros(12)
        sensor_rows.append({"time": self.measurement.actual_time})
        if self.fail_after_step:
            return self.measurement
        if self.step_error is not None:
            raise self.step_error
        return self.measurement


class FakeControl:
    def __init__(self, **kwargs):
        self.tick_calls = []
        self.observe_calls = []
        self.terminate = False

    def tick(self, **kwargs):
        self.tick_calls.append(kwargs["iteration"])

    def observe_imu(self, reading):
        self.observe_calls.append(reading)

    def check_termination(self, **kwargs):
        return self.terminate


class FakeStream:
    def __init__(self, **kwargs):
        self.video_frames = 0
        self.rgb_frames_acquired = 0
        self.process_calls = 0

    def process(self, **kwargs):
        self.process_calls += 1


class Fakes:
    """Fresh fake dependency graph for one orchestration run."""

    def __init__(self):
        self.app = FakeApp()
        self.rig = FakeCameraRig()
        self.contact = FakeContactMonitor()
        self.mapping = FakeMapping()
        self.video = FakeVideo()
        self.ros = FakeRos()
        self.art = FakeArt()
        self.measurement = FakeMeasurement()
        self.control = FakeControl()
        self.stream = FakeStream()
        self.rgbd = types.SimpleNamespace(rep=object(), product=object())
        self.tracker = object()
        self.archive = None
        self.stepper = FakeStepper(self.measurement)
        self.previous_delegates = []


def _fake_mapping_session(fakes, args):
    """Real MappingSession binds itself on the context; mirror that ownership."""
    context = args[5]
    context.mapping = fakes.mapping
    return fakes.mapping


@contextlib.contextmanager
def fake_dependencies(fakes, stepper_class=None):
    """Patch every SDK/ROS/PIL factory called by ``run_capture``."""
    if stepper_class is None:
        stepper_class = lambda **kwargs: fakes.stepper

    def open_stage(scene, experiment, result, save):
        result["stage"] = "stage_opened"
        return object()

    patches = {
        "start_simulation_app": lambda use_fabric: fakes.app,
        "install_headless_display_delegates": lambda: ([], fakes.previous_delegates),
        "resolve_spawn": lambda experiment: ("spawn", 0.0, 1.0),
        "open_stage": open_stage,
        "setup_pedestrian": lambda stage, experiment, result: None,
        "spawn_articulation_asset": lambda stage, spawn, experiment, result, save: [
            object()
        ],
        "ContactMonitor": FakeContactMonitor,
        "create_camera_rig": lambda stage, experiment: fakes.rig,
        "create_dynamic_guard": lambda experiment, imu_mount: None,
        "create_imu_sensor": lambda rig: object(),
        "create_rgbd_annotators": lambda rig: fakes.rgbd,
        "sensor_installation_receipt": lambda rig, truth_usage, sim_rtf: {
            "installed": True
        },
        "MappingSession": lambda *args, **kwargs: _fake_mapping_session(fakes, args),
        "create_overview": lambda rig, stage, rep: object(),
        "OverviewVideo": lambda output: fakes.video,
        "create_ros_interface": lambda experiment, learned_mapper, learned_scorer, **kwargs: fakes.ros,
        "setup_physics": lambda roots, use_fabric, app, result, save: fakes.art,
        "initialize_articulation": lambda *args, **kwargs: (
            object(),
            object(),
            list(range(12)),
            np.zeros(12),
            np.zeros(12),
        ),
        "load_policy": lambda *args, **kwargs: object(),
        "create_pedestrian_view": lambda stage, pedestrian: None,
        "resolve_limits": lambda *args: (0.5, 1.0),
        "create_route_controller": lambda *args, **kwargs: fakes.tracker,
        "annotate_policy_receipt": lambda result, *args, **kwargs: result.__setitem__(
            "policy_annotation", True
        ),
        "run_preflights": lambda *args: None,
        "play": lambda: None,
        "ControlLoop": lambda **kwargs: fakes.control,
        "PhysicsStepper": stepper_class,
        "FrameStream": lambda **kwargs: fakes.stream,
        "update_pedestrian": lambda pedestrian, experiment: None,
        "compute_torque": lambda **kwargs: np.zeros(12),
        "open_rgb_archive": lambda output, experiment, result: fakes.archive,
        "stop": lambda app, delegates: (app.finish(), delegates.clear()),
    }
    missing = [name for name in patches if not hasattr(entry, name)]
    assert not missing, f"capture entry lost factory names: {missing}"
    originals = {name: getattr(entry, name) for name in patches}
    try:
        for name, replacement in patches.items():
            setattr(entry, name, replacement)
        yield fakes
    finally:
        for name, original in originals.items():
            setattr(entry, name, original)


@contextlib.contextmanager
def isaac_simulation_manager_stub():
    """Provide only the one Isaac symbol the orchestration imports directly."""
    created = [
        name
        for name in ("isaacsim", "isaacsim.core", "isaacsim.core.simulation_manager")
        if name not in sys.modules
    ]
    isaacsim = types.ModuleType("isaacsim")
    isaacsim.__path__ = []
    core = types.ModuleType("isaacsim.core")
    core.__path__ = []
    simulation_manager = types.ModuleType("isaacsim.core.simulation_manager")

    class SimulationManager:
        @staticmethod
        def get_simulation_time():
            return 0.0

    simulation_manager.SimulationManager = SimulationManager
    sys.modules.update(
        {
            "isaacsim": isaacsim,
            "isaacsim.core": core,
            "isaacsim.core.simulation_manager": simulation_manager,
        }
    )
    try:
        yield
    finally:
        for name in created:
            sys.modules.pop(name, None)


class CaptureOrchestrationTests(unittest.TestCase):
    def run_capture(self, fakes, steps, stepper_class=None, receipts_class=None):
        temporary = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, temporary, True)
        output = Path(temporary) / "run"
        environment = {
            "PROBE_STEPS": str(steps),
            "DYNANAV_EXPERIMENT": "",
        }
        extra_patch = (
            unittest.mock.patch.object(entry, "Receipts", receipts_class)
            if receipts_class is not None
            else contextlib.nullcontext()
        )
        with fake_dependencies(fakes, stepper_class), isaac_simulation_manager_stub():
            with unittest.mock.patch.dict(os.environ, environment), extra_patch:
                config = entry.parse_config(["capture_go2_rgbd", str(output), "/scene.usd"])
                entry.run_capture(config)
        return output, temporary

    def receipt(self, output):
        return json.loads((output / "result.json").read_text())

    def test_zero_step_normal_path_uses_real_stepper_signature_and_cleans_up(self):
        real_stepper = entry.PhysicsStepper

        class RecordingStepper(real_stepper):
            last_kwargs = None

            def __init__(self, **kwargs):
                type(self).last_kwargs = dict(kwargs)
                super().__init__(**kwargs)

        fakes = Fakes()
        output, _ = self.run_capture(fakes, 0, stepper_class=RecordingStepper)
        result = self.receipt(output)
        self.assertEqual(result["stage"], "articulation_contact_probe_complete")
        self.assertEqual(result["executed_physics_steps"], 0)
        self.assertEqual(result["outcome"], "completed")
        self.assertEqual(result["video"]["encoder_exit"], 0)
        self.assertIs(RecordingStepper.last_kwargs["mount"], fakes.rig.mount)
        self.assertIs(RecordingStepper.last_kwargs["imu_mount"], fakes.rig.imu_mount)
        self.assertEqual(fakes.ros.destroy_calls, 1)
        self.assertEqual(fakes.video.close_calls, [15])
        self.assertEqual(fakes.mapping.finish_calls, 1)
        self.assertEqual(fakes.app.stop_calls, 1)
        self.assertEqual(fakes.previous_delegates, [])

    def test_one_step_normal_path_keeps_loop_evidence(self):
        fakes = Fakes()
        output, _ = self.run_capture(fakes, 1)
        result = self.receipt(output)
        self.assertEqual(result["stage"], "articulation_contact_probe_complete")
        self.assertEqual(result["executed_physics_steps"], 1)
        self.assertEqual(len(result["rows"]), 1)
        self.assertEqual(result["rows"][0]["contact_callback_count"], 0)
        self.assertEqual(result["physics_dt"], 0.005)
        self.assertEqual(fakes.ros.destroy_calls, 1)
        self.assertEqual(fakes.video.close_calls, [15])
        self.assertEqual(fakes.mapping.finish_calls, 1)

    def test_first_tick_failure_records_primary_error_and_cleans_owned_resources(self):
        fakes = Fakes()
        sentinel = SentinelError("first tick sentinel")
        fakes.stepper = FakeStepper(fakes.measurement, step_error=sentinel)
        with self.assertRaises(SentinelError) as caught:
            self.run_capture(fakes, 1)
        self.assertIs(caught.exception, sentinel)
        # The temp dir is gone after the context; inspect the fake calls and the
        # receipt through a retained path instead (run_capture returns output).
        # Re-run is unnecessary: assert cleanup happened without masking.
        self.assertEqual(fakes.mapping.finalize_failure_calls, 1)
        self.assertEqual(fakes.video.close_calls, [10])
        self.assertEqual(fakes.ros.destroy_calls, 1)
        self.assertEqual(fakes.app.close_calls, [1])

    def test_first_tick_failure_receipt_contains_primary_error(self):
        fakes = Fakes()
        sentinel = SentinelError("first tick sentinel")
        fakes.stepper = FakeStepper(fakes.measurement, step_error=sentinel)
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "run"
            with fake_dependencies(fakes), isaac_simulation_manager_stub():
                with unittest.mock.patch.dict(
                    os.environ,
                    {"PROBE_STEPS": "1", "DYNANAV_EXPERIMENT": ""},
                ):
                    config = entry.parse_config(
                        ["capture_go2_rgbd", str(output), "/scene.usd"]
                    )
                    with self.assertRaises(SentinelError) as caught:
                        entry.run_capture(config)
            self.assertIs(caught.exception, sentinel)
            result = self.receipt(output)
        self.assertEqual(result["stage"], "physics_running")
        self.assertIn("SentinelError", result["error"])
        self.assertIn("first tick sentinel", result["error"])
        self.assertIn("SentinelError", result["traceback"])
        self.assertNotIn("AttributeError", result["error"])
        self.assertEqual(result["actuator_failure_state"]["dof_names"], fakes.art.dof_names)
        self.assertEqual(result["actuator_failure_state"]["q_isaac"], [0.0] * 12)
        self.assertNotIn("failure_cleanup_errors", result)

    def test_secondary_cleanup_failures_do_not_replace_primary_error(self):
        fakes = Fakes()
        sentinel = SentinelError("primary sentinel")
        fakes.stepper = FakeStepper(fakes.measurement, step_error=sentinel)
        fakes.mapping.finalize_failure_error = RuntimeError("mapping finalize failed")
        fakes.video.close_error = RuntimeError("encoder close failed")
        fakes.app.close_error = RuntimeError("app close failed")
        fakes.archive = FakeArchive()
        fakes.archive.receipt_error = RuntimeError("archive receipt failed")
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "run"
            with fake_dependencies(fakes), isaac_simulation_manager_stub():
                with unittest.mock.patch.dict(
                    os.environ,
                    {"PROBE_STEPS": "1", "DYNANAV_EXPERIMENT": ""},
                ):
                    config = entry.parse_config(
                        ["capture_go2_rgbd", str(output), "/scene.usd"]
                    )
                    with self.assertRaises(SentinelError) as caught:
                        entry.run_capture(config)
            self.assertIs(caught.exception, sentinel)
            result = self.receipt(output)
        self.assertIn("SentinelError", result["error"])
        self.assertIn("primary sentinel", result["traceback"])
        cleanup = "\n".join(result["failure_cleanup_errors"])
        self.assertIn("mapping", cleanup)
        self.assertIn("video", cleanup)
        self.assertIn("app_close", cleanup)
        self.assertIn("rgb_archive", cleanup)
        # Every owned resource is still attempted even after an earlier failure.
        self.assertEqual(fakes.ros.destroy_calls, 1)
        self.assertEqual(fakes.app.close_calls, [1])
        self.assertEqual(fakes.video.close_calls, [10])


    def test_receipt_write_failure_still_attempts_cleanup_and_keeps_primary(self):
        fakes = Fakes()
        sentinel = SentinelError("receipt failure sentinel")
        fakes.stepper = FakeStepper(fakes.measurement, step_error=sentinel)
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "run"
            receipts = FakeReceipts(output, {}, False, False)
            with fake_dependencies(fakes), isaac_simulation_manager_stub():
                with unittest.mock.patch.dict(
                    os.environ,
                    {"PROBE_STEPS": "1", "DYNANAV_EXPERIMENT": ""},
                ), unittest.mock.patch.object(entry, "Receipts", lambda *args: receipts):
                    config = entry.parse_config(
                        ["capture_go2_rgbd", str(output), "/scene.usd"]
                    )
                    with self.assertRaises(SentinelError) as caught:
                        entry.run_capture(config)
            self.assertIs(caught.exception, sentinel)
        self.assertGreaterEqual(receipts.force_failures, 1)
        self.assertEqual(fakes.app.close_calls, [1])
        self.assertEqual(fakes.video.close_calls, [10])
        self.assertEqual(fakes.ros.destroy_calls, 1)
        self.assertEqual(fakes.mapping.finalize_failure_calls, 1)


    def test_traceback_output_failure_keeps_primary_and_cleans_owned_resources(self):
        fakes = Fakes()
        sentinel = SentinelError("traceback output failure sentinel")
        fakes.stepper = FakeStepper(fakes.measurement, step_error=sentinel)
        with (
            unittest.mock.patch("builtins.print", side_effect=BrokenPipeError("closed stdout")),
            self.assertRaises(SentinelError) as caught,
        ):
            self.run_capture(fakes, 1)
        self.assertIs(caught.exception, sentinel)
        self.assertEqual(fakes.video.close_calls, [10])
        self.assertEqual(fakes.ros.destroy_calls, 1)
        self.assertEqual(fakes.app.close_calls, [1])


class FailureBoundaryTests(unittest.TestCase):
    def test_all_journals_close_even_when_first_close_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            receipts = entry.Receipts(Path(directory), {}, False, False)
            first = unittest.mock.Mock()
            first.close.side_effect = OSError("flush failed")
            second = unittest.mock.Mock()
            receipts._journals = {"first": first, "second": second}
            with self.assertRaises(OSError):
                receipts.close_journals()
            second.close.assert_called_once()

    def test_failure_report_property_does_not_skip_cleanup(self):
        class BadReport(SentinelError):
            @property
            def report(self):
                raise RuntimeError("report getter failed")
        with tempfile.TemporaryDirectory() as directory:
            receipts = entry.Receipts(Path(directory), {}, False, False)
            context = entry.CaptureContext(receipts, app=FakeApp(), video=FakeVideo(), ros=FakeRos())
            entry.record_failure(receipts, BadReport("primary"), context)
            self.assertEqual(context.app.close_calls, [1])
            self.assertEqual(context.ros.destroy_calls, 1)
            self.assertIn("failure_report", "\n".join(receipts.result["failure_cleanup_errors"]))

    def test_bad_exception_repr_and_unserializable_report_still_save(self):
        class BadRepresentation(SentinelError):
            def __repr__(self):
                raise RuntimeError("repr failed")
        with tempfile.TemporaryDirectory() as directory:
            receipts = entry.Receipts(Path(directory), {}, False, False)
            context = entry.CaptureContext(receipts, app=FakeApp(), video=FakeVideo())
            context.video.close_error = BadRepresentation("secondary")
            error = BadRepresentation("primary")
            error.report = {"bad": object()}
            entry.record_failure(receipts, error, context)
            result = json.loads((Path(directory) / "result.json").read_text())
            self.assertIn("BadRepresentation", result["error"])
            self.assertIn("video", "\n".join(result["failure_cleanup_errors"]))
            self.assertEqual(context.app.close_calls, [1])

    def test_ambiguous_evidence_does_not_skip_app_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            receipts = entry.Receipts(Path(directory), {}, False, False)
            context = entry.CaptureContext(receipts, app=FakeApp(), rows=np.ones(2))
            entry.record_failure(receipts, SentinelError("primary"), context)
            self.assertEqual(context.app.close_calls, [1])
            self.assertIn("runtime_evidence", "\n".join(receipts.result["failure_cleanup_errors"]))

    def test_initial_save_failure_runs_failure_handler(self):
        with tempfile.TemporaryDirectory() as directory:
            config = entry.CaptureConfig(Path(directory), "/scene.usd", {}, False, False, False, True)
            receipts = entry.Receipts(Path(directory), {}, False, False)
            sentinel = OSError("first write")
            with unittest.mock.patch.object(entry, "Receipts", return_value=receipts), unittest.mock.patch.object(
                receipts, "save", side_effect=sentinel
            ), unittest.mock.patch.object(entry, "record_failure", wraps=entry.record_failure) as failure:
                with self.assertRaises(OSError) as caught:
                    entry.run_capture(config)
            self.assertIs(caught.exception, sentinel)
            failure.assert_called_once()
            self.assertEqual(receipts.result["outcome"], "failed")

    def test_invalid_profile_writes_terminal_configuration_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            profile = Path(directory) / "profile.json"
            profile.write_text("{broken")
            with unittest.mock.patch.dict(os.environ, {"DYNANAV_EXPERIMENT": str(profile)}):
                with self.assertRaises(json.JSONDecodeError):
                    entry.run(["capture", str(output), "/scene.usd"])
            result = json.loads((output / "result.json").read_text())
            self.assertEqual(result["outcome"], "failed")
            self.assertEqual(result["stage"], "configuration")

    def test_keyboard_interrupt_has_terminal_receipt_and_cleanup(self):
        fakes = Fakes()
        with tempfile.TemporaryDirectory() as directory:
            config = entry.CaptureConfig(Path(directory), "/scene.usd", {}, False, False, False, True)
            with fake_dependencies(fakes), unittest.mock.patch.object(
                entry, "resolve_spawn", side_effect=KeyboardInterrupt()
            ):
                with self.assertRaises(KeyboardInterrupt):
                    entry.run_capture(config)
            result = json.loads((Path(directory) / "result.json").read_text())
            self.assertEqual(result["outcome"], "cancelled")
            self.assertEqual(fakes.app.close_calls, [1])


class PublishTraceTests(unittest.TestCase):
    @contextlib.contextmanager
    def ros(self, enabled, journal, *, frontend="/dynanav/lidar_perception_filtered"):
        from embodied_navigation.simulation.dynanav.capture.ros_interface import create_ros_interface

        class Message:
            def __init__(self, **kwargs):
                self.header = types.SimpleNamespace(
                    stamp=types.SimpleNamespace(sec=0, nanosec=0), frame_id="")
                self.__dict__.update(kwargs)

        publishers = {}
        def publisher(kind, topic, qos):
            publishers[topic] = unittest.mock.Mock()
            return publishers[topic]

        node = unittest.mock.Mock()
        node.create_publisher.side_effect = publisher
        rclpy = unittest.mock.Mock()
        rclpy.create_node.return_value = node
        modules = {"rclpy": rclpy}
        for name, attrs in {
            "geometry_msgs.msg": {"TwistStamped": Message},
            "nav_msgs.msg": {"Odometry": Message},
            "rclpy.qos": {"qos_profile_sensor_data": object()},
            "rclpy.signals": {"SignalHandlerOptions": types.SimpleNamespace(NO=0)},
            "rosgraph_msgs.msg": {"Clock": Message},
            "sensor_msgs.msg": dict.fromkeys(
                ["CameraInfo", "Image", "Imu", "PointCloud2", "PointField"], Message),
        }.items():
            module = types.ModuleType(name)
            module.__dict__.update(attrs)
            modules[name] = module
        with unittest.mock.patch.dict(sys.modules, modules), unittest.mock.patch.dict(
            os.environ, {"DYNANAV_PUBLISH_TRACE": enabled}
        ):
            yield create_ros_interface(
                {"lidar_frontend_topic": frontend}, object(), None, journal=journal
            ), publishers

    def test_trace_preserves_stamps_and_publish_order_for_selected_sensors(self):
        events = []
        def journal(name, row):
            self.assertEqual(name, "sensor_publish_trace.jsonl")
            events.append(row)
        with self.ros("1", journal) as (ros, publishers), unittest.mock.patch(
            "embodied_navigation.simulation.dynanav.capture.ros_interface.time.monotonic_ns",
            side_effect=[100, 200, 300, 400],
        ):
            for topic, publisher in publishers.items():
                publisher.publish.side_effect = lambda msg, topic=topic: events.append(
                    ("published", topic, msg))
            ros.publish_rgb(1.25, np.zeros((512, 512, 3), dtype=np.uint8))
            ros.publish_cloud(1.25, np.zeros((3, 3)), None,
                              np.array([True, False, True]), None, object())
        rows = [row for row in events if isinstance(row, dict)]
        self.assertEqual([row["event"] for row in rows],
                         ["publish_call", "publish_returned"] * 2)
        self.assertEqual([row["monotonic_ns"] for row in rows], [100, 200, 300, 400])
        self.assertTrue(all(row["source_ns"] == 1250000000 for row in rows))
        self.assertTrue(all(row["delivery_confirmed"] is False for row in rows))
        self.assertEqual(rows[0]["data_bytes"], 512 * 512 * 3)
        self.assertEqual(rows[2]["data_bytes"], 32)
        self.assertEqual(rows[2]["point_count"], 2)
        self.assertEqual(rows[2]["topic"], "/dynanav/lidar_perception_filtered")
        self.assertEqual([item["event"] if isinstance(item, dict) else item[0] for item in events],
                         ["publish_call", "published", "publish_returned",
                          "published", "publish_call", "published", "publish_returned"])
        image = publishers["/dynanav/camera"].publish.call_args.args[0]
        self.assertEqual((image.header.stamp.sec, image.header.stamp.nanosec), (1, 250000000))
        self.assertEqual((image.encoding, image.step), ("rgb8", 1536))

    def test_trace_disabled_by_default_keeps_publish_and_no_journal(self):
        journal = unittest.mock.Mock()
        with self.ros("", journal) as (ros, publishers):
            ros.publish_rgb(2.0, np.zeros((512, 512, 3), dtype=np.uint8))
            ros.publish_cloud(2.0, np.zeros((1, 3)), None,
                              np.array([True]), None, object())
        journal.assert_not_called()
        publishers["/dynanav/camera"].publish.assert_called_once()
        publishers["/dynanav/lidar_perception_filtered"].publish.assert_called_once()

    def test_trace_failure_propagates_and_never_claims_return(self):
        for fail_at in ("journal", "publisher"):
            with self.subTest(fail_at=fail_at):
                rows = []
                sentinel = OSError("trace boundary failure")
                def journal(name, row):
                    if fail_at == "journal":
                        raise sentinel
                    rows.append(row)
                with self.ros("1", journal) as (ros, publishers):
                    if fail_at == "publisher":
                        publishers["/dynanav/camera"].publish.side_effect = sentinel
                    with self.assertRaises(OSError) as caught:
                        ros.publish_rgb(2.0, np.zeros((512, 512, 3), dtype=np.uint8))
                self.assertIs(caught.exception, sentinel)
                self.assertFalse(any(row["event"] == "publish_returned" for row in rows))

    def test_trace_flush_is_visible_without_closing_journal(self):
        with tempfile.TemporaryDirectory() as directory:
            receipts = entry.Receipts(Path(directory), {}, False, False)
            self.addCleanup(receipts.close_journals)
            receipts.journal("sensor_publish_trace.jsonl", {"event": "publish_call"}, flush=True)
            self.assertEqual(json.loads((Path(directory) / "sensor_publish_trace.jsonl").read_text()),
                             {"event": "publish_call"})


class NativeShutdownBoundaryTests(unittest.TestCase):
    def test_close_system_exit_keeps_completed_receipt_and_restores_handlers(self):
        original = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
        fakes = Fakes()
        with tempfile.TemporaryDirectory() as directory:
            config = entry.CaptureConfig(Path(directory), "/scene.usd", {}, False, False, False, True)
            def sdk_start(use_fabric):
                signal.signal(signal.SIGINT, lambda *_: None)
                return fakes.app
            with fake_dependencies(fakes), isaac_simulation_manager_stub(), unittest.mock.patch.dict(
                os.environ, {"PROBE_STEPS": "0"}
            ), unittest.mock.patch.object(entry, "start_simulation_app", sdk_start), unittest.mock.patch.object(
                entry, "stop", side_effect=SystemExit(0)
            ):
                with self.assertRaises(SystemExit) as caught:
                    entry.run_capture(config)
            self.assertEqual(caught.exception.code, 0)
            result = json.loads((Path(directory) / "result.json").read_text())
        self.assertEqual(result["outcome"], "completed")
        self.assertTrue(result["app_close_requested"])
        self.assertFalse(result["app_close_returned"])
        self.assertEqual({s: signal.getsignal(s) for s in original}, original)

    def test_failure_close_system_exit_keeps_primary_and_terminal_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            receipts = entry.Receipts(Path(directory), {}, False, False)
            app = FakeApp()
            app.close_error = SystemExit(0)
            entry.record_failure(receipts, SentinelError("primary"), entry.CaptureContext(receipts, app=app))
            result = json.loads((Path(directory) / "result.json").read_text())
        self.assertEqual(result["outcome"], "failed")
        self.assertIn("primary", result["error"])
        self.assertFalse(result["app_close_returned"])
        self.assertIn("app_close", "\n".join(result["failure_cleanup_errors"]))

    def test_sdk_callback_cannot_swallow_cooperative_cancellation(self):
        for broken_pipe in (False, True):
            with self.subTest(broken_pipe=broken_pipe), tempfile.TemporaryDirectory() as directory:
                fakes = Fakes()
                swallowed = []
                def sdk_step(**kwargs):
                    # Model an SDK asyncio callback that catches BaseException.
                    try:
                        os.kill(os.getpid(), signal.SIGINT)
                    except BaseException as error:
                        swallowed.append(error)
                    if broken_pipe:
                        raise BrokenPipeError("ffmpeg stopped by process-group SIGINT")
                    return fakes.measurement
                fakes.stepper.step = sdk_step
                original_close = fakes.video.close
                def cleanup_close(timeout):
                    os.kill(os.getpid(), signal.SIGTERM)
                    original_close(timeout)
                fakes.video.close = cleanup_close
                config = entry.CaptureConfig(Path(directory), "/scene.usd", {}, False, False, False, True)
                with fake_dependencies(fakes), isaac_simulation_manager_stub(), unittest.mock.patch.dict(
                    os.environ, {"PROBE_STEPS": "1"}
                ):
                    with self.assertRaises(entry.CaptureCancellation) as caught:
                        entry.run_capture(config)
                result = json.loads((Path(directory) / "result.json").read_text())
                self.assertEqual(caught.exception.code, 130)
                self.assertEqual(swallowed, [])
                self.assertEqual(result["outcome"], "cancelled")
                self.assertEqual(result["cancellation_signal"], signal.SIGINT)
                self.assertEqual(fakes.video.close_calls, [10])
                self.assertEqual(fakes.app.close_calls, [1])
                if broken_pipe:
                    self.assertIn("BrokenPipeError", result["interrupted_operation_error"])
                    self.assertIn("ffmpeg stopped", result["interrupted_operation_traceback"])
                else:
                    self.assertNotIn("interrupted_operation_error", result)
                self.assertNotIn("failure_cleanup_errors", result)

    def test_subprocess_sdk_handler_replacement_and_nonreturning_native_close(self):
        tests_dir = str(Path(__file__).resolve().parent)
        for mode in ("SIGINT", "SIGTERM", "success"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                program = f"""
import os, signal, sys
from pathlib import Path
sys.path.insert(0, {tests_dir!r})
from test_dynanav_capture_orchestration import entry, Fakes, fake_dependencies, isaac_simulation_manager_stub
fakes = Fakes()
config = entry.CaptureConfig(Path({directory!r}), "/scene.usd", {{}}, False, False, False, True)
mode = {mode!r}
with fake_dependencies(fakes), isaac_simulation_manager_stub():
    def sdk_start(use_fabric):
        signal.signal(signal.SIGINT, lambda *_: os._exit(0))
        return fakes.app
    entry.start_simulation_app = sdk_start
    fakes.app.close = lambda **kwargs: os._exit(1)
    if mode == "success":
        os.environ["PROBE_STEPS"] = "0"
        entry.stop = lambda *args: os._exit(0)
    else:
        entry.resolve_spawn = lambda experiment: (os.kill(os.getpid(), getattr(signal, mode)), ("spawn", 0.0, 1.0))[1]
    entry.run_capture(config)
"""
                completed = subprocess.run([sys.executable, "-B", "-c", program],
                                           capture_output=True, text=True, timeout=15)
                self.assertEqual(completed.returncode, 0 if mode == "success" else 1,
                                 completed.stderr)
                result = json.loads((Path(directory) / "result.json").read_text())
                self.assertEqual(result["outcome"], "completed" if mode == "success" else "cancelled")
                self.assertTrue(result["app_close_requested"])
                self.assertFalse(result["app_close_returned"])
                if mode != "success":
                    self.assertIn(str(128 + getattr(signal, mode)), result["error"])


if __name__ == "__main__":
    unittest.main()
