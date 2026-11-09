"""Explicit-entry DynaNav Isaac RGB-D capture experiment.

Importing this module has no side effects: it only defines the argv/environment
contract plus ``run``/``main``. ``experiment.py`` and
``scripts/capture_dynanav.py`` keep executing it through ``runpy`` as
``__main__`` with ``sys.argv = ["capture_go2_rgbd", <output>, <scene>]``; the
DYNANAV_EXPERIMENT environment variable still supplies the experiment profile.

The implementation is split under ``capture/``:

- ``receipts``: result.json, journals, throttled save, failure receipt
- ``scene``: SimulationApp, USD stage, Go2 spawn, contacts, pedestrians
- ``sensors``: camera/IMU/render products and the installation receipt
- ``ros_interface``: node, ``/dynanav/*`` publishers/subscribers, messages
- ``mapping``: oracle/learned mapping, scoring, raw RGB archive
- ``policy``: Student/RoboGauge policy loading, actions and torque
- ``control``: 50 Hz control tick, routes, approval/freshness, termination
- ``physics``: 0.005 s step, clocks, effort/IMU/safety assertions
- ``frames``: 10 Hz RGB-D/cloud evidence and overview/RGB video panel
- ``video``: ffmpeg overview/RGB encoder

``run(argv)`` reads output/scene from ``argv[1]``/``argv[2]`` exactly as the old
script read them from ``sys.argv``; every assertion, threshold, cadence, ROS
topic/type/QoS, receipt key and cleanup order is preserved.
"""

import json
import os
import pathlib
import signal
import sys
import time
from dataclasses import dataclass

import numpy as np

from embodied_navigation.simulation.dynanav.capture.control import (
    ControlLoop,
    annotate_policy_receipt,
    create_dynamic_guard,
    create_route_controller,
    resolve_limits,
)
from embodied_navigation.simulation.dynanav.capture.frames import FrameStream
from embodied_navigation.simulation.dynanav.capture.mapping import (
    MappingSession,
    open_rgb_archive,
)
from embodied_navigation.simulation.dynanav.capture.physics import PhysicsStepper
from embodied_navigation.simulation.dynanav.capture.policy import (
    compute_torque,
    load_policy,
)
from embodied_navigation.simulation.dynanav.capture.receipts import (
    CaptureContext,
    Receipts,
    record_failure,
)
from embodied_navigation.simulation.dynanav.capture.ros_interface import (
    create_ros_interface,
)
from embodied_navigation.simulation.dynanav.capture.scene import (
    ContactMonitor,
    create_pedestrian_view,
    initialize_articulation,
    install_headless_display_delegates,
    open_stage,
    play,
    resolve_spawn,
    run_preflights,
    setup_pedestrian,
    setup_physics,
    spawn_articulation_asset,
    start_simulation_app,
    stop,
    update_pedestrian,
)
from embodied_navigation.simulation.dynanav.capture.sensors import (
    create_camera_rig,
    create_imu_sensor,
    create_overview,
    create_rgbd_annotators,
    sensor_installation_receipt,
)
from embodied_navigation.simulation.dynanav.capture.video import OverviewVideo


@dataclass(frozen=True)
class CaptureConfig:
    """Experiment profile flags plus the argv output/scene contract."""

    output: pathlib.Path
    scene: str
    experiment: dict
    gt_diagnostic: bool
    slam_control: bool
    robogauge: bool
    use_fabric: bool


def load_experiment():
    """Read the optional DYNANAV_EXPERIMENT profile (empty dict when absent)."""
    experiment_path = os.environ.get("DYNANAV_EXPERIMENT")
    return json.loads(pathlib.Path(experiment_path).read_text()) if experiment_path else {}


def parse_config(argv=None):
    """Parse the legacy argv contract and experiment profile flags.

    ``argv[1]`` is the output directory (created here, only when ``run`` is
    called) and ``argv[2]`` is the scene USD, exactly as the original script.
    """
    argv = sys.argv if argv is None else list(argv)
    out = pathlib.Path(argv[1])
    out.mkdir(exist_ok=True)
    experiment = load_experiment()
    gt_diagnostic = experiment.get("control_mode") == "native_pose_diagnostic"
    slam_control = experiment.get("control_mode") == "slam_pose_diagnostic"
    robogauge = experiment.get("policy") == "robogauge"
    use_fabric = experiment.get("render_backend", "fabric") == "fabric"
    if (gt_diagnostic or slam_control) and not robogauge:
        raise ValueError(
            "Native-pose diagnostic requires the explicit RoboGauge experiment"
        )
    return CaptureConfig(
        output=out,
        scene=argv[2],
        experiment=experiment,
        gt_diagnostic=gt_diagnostic,
        slam_control=slam_control,
        robogauge=robogauge,
        use_fabric=use_fabric,
    )


def run(argv=None):
    """Run one capture experiment; no side effects until this is called."""
    argv = sys.argv if argv is None else list(argv)
    try:
        config = parse_config(argv)
    except BaseException as error:
        # Parsing may fail after creating the output (invalid JSON/profile or
        # missing scene). Preserve the configuration error even if output I/O
        # itself is unavailable; no simulator resources exist at this point.
        try:
            receipts = Receipts(pathlib.Path(argv[1]), {}, False, False)
            receipts.result["stage"] = "configuration"
            record_failure(receipts, error, CaptureContext(receipts))
        except BaseException:
            pass
        raise
    return run_capture(config)


def main(argv=None):
    """Entry point used by ``runpy.run_module(..., run_name="__main__")``."""
    run(argv)



class CaptureCancellation(SystemExit):
    """Cancellation raised only at a Python orchestration boundary."""

    def __init__(self, signum):
        super().__init__(128 + signum)
        self.signum = signum


@dataclass
class CancellationState:
    """Latch signals without throwing into SDK/asyncio callbacks."""

    signum: int | None = None

    def request(self, signum, frame):
        if self.signum is None:
            self.signum = signum

    def check(self):
        if self.signum is not None:
            raise CaptureCancellation(self.signum)


def _restore_signal_handlers(handlers):
    """Restore caller handlers on returning Python paths without masking errors."""
    for signum, handler in handlers.items():
        try:
            signal.signal(signum, handler)
        except (ValueError, OSError):
            # A non-main-thread caller cannot own process signal handlers.
            pass

def run_capture(config):
    """Orchestrate config -> session -> ROS -> mapping -> loop -> receipts."""
    receipts = Receipts(
        config.output,
        config.experiment,
        config.gt_diagnostic,
        config.slam_control,
    )
    context = CaptureContext(receipts)
    original_signal_handlers = {}
    closing_app = False
    cancellation = CancellationState()
    try:
        receipts.save()
        context.rgb_archive = open_rgb_archive(
            config.output, config.experiment, receipts.result
        )
        receipts.save()

        original_signal_handlers = {
            signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)
        }
        app = start_simulation_app(config.use_fabric)
        context.app = app
        # SimulationApp replaces SIGINT with a handler that shuts down the SDK
        # and exits zero. Reclaim both signals before any capture work starts.
        for signum in original_signal_handlers:
            signal.signal(signum, cancellation.request)
        previous_delegates, headless_delegates = install_headless_display_delegates()
        receipts.result["headless_fixed_display_delegate_count"] = len(
            headless_delegates
        )
        receipts.result["rendering_dependencies_unloaded"] = False
        receipts.save()
        # The embedded Python traceback timer can fault while walking SDK threads.
        # Keep scalar receipts/journals; capture native failures with an external debugger.
        receipts.result["periodic_python_stack_dump"] = False

        cancellation.check()
        spawn, spawn_yaw, sim_rtf = resolve_spawn(config.experiment)
        cancellation.check()
        stage = open_stage(
            config.scene, config.experiment, receipts.result, receipts.save
        )
        cancellation.check()
        pedestrian = setup_pedestrian(stage, config.experiment, receipts.result)
        roots = spawn_articulation_asset(
            stage, spawn, config.experiment, receipts.result, receipts.save
        )
        contact_monitor = ContactMonitor()
        contact_monitor.install(stage)
        context.dynamic_contact_events = contact_monitor.dynamic_contact_events

        cancellation.check()
        camera_rig = create_camera_rig(stage, config.experiment)
        dynamic_guard = create_dynamic_guard(config.experiment, camera_rig.imu_mount)
        imu_sensor = create_imu_sensor(camera_rig)
        rgbd_annotators = create_rgbd_annotators(camera_rig)
        mapping = MappingSession(
            config.experiment,
            stage,
            config.output,
            rgbd_annotators.rep,
            rgbd_annotators.product,
            context,
            config.gt_diagnostic,
            config.slam_control,
        )
        cancellation.check()
        overview = create_overview(camera_rig, stage, rgbd_annotators.rep)
        video = OverviewVideo(config.output)
        context.video = video
        receipts.result["sensor_installation"] = sensor_installation_receipt(
            camera_rig, receipts.result["truth_usage"], sim_rtf
        )
        receipts.save()

        cancellation.check()
        ros = create_ros_interface(
            config.experiment, mapping.learned_mapper, mapping.learned_scorer,
            journal=lambda name, payload: receipts.journal(name, payload, flush=True),
        )
        context.ros = ros
        context.published = ros.published
        context.latest_odom = ros.latest_odom

        cancellation.check()
        art = setup_physics(
            roots, config.use_fabric, app, receipts.result, receipts.save
        )
        # The failure receipt records the actuator state from the same handle the
        # control loop uses; without this binding a first-tick error would raise
        # AttributeError while handling the real error.
        context.art = art
        cancellation.check()
        contract, canonical, indices, target, qdefault = initialize_articulation(
            art,
            stage,
            spawn,
            spawn_yaw,
            config.output,
            camera_rig.imu_mount,
            receipts.result,
            receipts.save,
        )
        cancellation.check()
        policy = load_policy(
            config.experiment,
            canonical=canonical,
            contract=contract,
            robogauge=config.robogauge,
            result=receipts.result,
        )
        pedestrian_view = create_pedestrian_view(stage, pedestrian)
        target_speed, physical_speed_guard = resolve_limits(config.experiment)
        tracker = create_route_controller(
            config.experiment,
            gt_diagnostic=config.gt_diagnostic,
            slam_control=config.slam_control,
            target_speed=target_speed,
            spawn=spawn,
        )
        annotate_policy_receipt(
            receipts.result,
            config.experiment,
            config.gt_diagnostic,
            config.slam_control,
            config.robogauge,
            target_speed,
            physical_speed_guard,
        )
        cancellation.check()
        run_preflights(config.experiment, pedestrian, receipts.result, receipts.save)
        previous_completed_tau = np.zeros(12)
        receipts.save()

        rows = []
        sensor_rows = []
        camera_rows = []
        policy_frames = []
        context.rows = rows
        context.sensor_rows = sensor_rows
        context.camera_rows = camera_rows
        context.policy_frames = policy_frames
        context.target = target
        loop_start = time.perf_counter()
        executed_steps = 0
        receipts.result["freshness_hold_events"] = []
        receipts.result["stage"] = "physics_running"
        receipts.save()
        cancellation.check()
        play()

        control = ControlLoop(
            experiment=config.experiment,
            gt_diagnostic=config.gt_diagnostic,
            slam_control=config.slam_control,
            robogauge=config.robogauge,
            target_speed=target_speed,
            indices=indices,
            target=target,
            tracker=tracker,
            dynamic_guard=dynamic_guard,
            policy=policy,
            receipts=receipts,
            latest_nav=ros.latest_nav,
            latest_odom=ros.latest_odom,
        )
        stepper = PhysicsStepper(
            art=art,
            experiment=config.experiment,
            indices=indices,
            use_fabric=config.use_fabric,
            imu_sensor=imu_sensor,
            receipts=receipts,
            spawn=spawn,
            physical_speed_guard=physical_speed_guard,
            imu_mount=camera_rig.imu_mount,
            mount=camera_rig.mount,
        )
        stream = FrameStream(
            experiment=config.experiment,
            out=config.output,
            stage=stage,
            gt_diagnostic=config.gt_diagnostic,
            mapping=mapping,
            ros=ros,
            video=video,
            rgb_annotators=rgbd_annotators,
            overview=overview,
            camera_rig=camera_rig,
            target_speed=target_speed,
            receipts=receipts,
            rgb_archive=context.rgb_archive,
        )
        context.stream = stream
        imu_stride = config.experiment.get("imu_publish_stride", 2)
        imu_orientation_available = config.experiment.get(
            "imu_orientation_available", True
        )
        from isaacsim.core.simulation_manager import SimulationManager

        for i in range(int(os.environ.get("PROBE_STEPS", "4400"))):
            cancellation.check()
            q = art.get_dof_positions().numpy()[0]
            qd = art.get_dof_velocities().numpy()[0]
            context.q, context.qd = q, qd
            cancellation.check()
            if i % 4 == 0:
                control.tick(
                    iteration=i,
                    q=q,
                    qd=qd,
                    art=art,
                    rows=rows,
                    contact_state=contact_monitor.contact_state,
                    previous_completed_tau=previous_completed_tau,
                    ros=ros,
                    policy_frames=policy_frames,
                )
            cancellation.check()
            pedestrian_target = update_pedestrian(pedestrian, config.experiment)
            contact_monitor.begin_step()
            tau = compute_torque(
                art=art,
                target=target,
                q=q,
                qd=qd,
                experiment=config.experiment,
                robogauge=config.robogauge,
                result=receipts.result,
            )
            measurement = stepper.step(
                iteration=i,
                tau=tau,
                q=q,
                qd=qd,
                contact_state=contact_monitor.contact_state,
                sensor_rows=sensor_rows,
            )
            cancellation.check()
            control.observe_imu(measurement.reading)
            previous_completed_tau = measurement.previous_completed_tau
            if (i + 1) % imu_stride == 0:
                ros.publish_imu_and_clock(
                    measurement.it, measurement.reading, imu_orientation_available
                )
            if (i + 1) % 20 == 0:
                stream.process(
                    iteration=i,
                    measurement=measurement,
                    pedestrian_view=pedestrian_view,
                    pedestrian_target=pedestrian_target,
                    dynamic_guard=dynamic_guard,
                    camera_rows=camera_rows,
                )
            cancellation.check()
            if (i + 1) % 10 == 0:
                target_wall = loop_start + (i + 1) * 0.005 / sim_rtf
                while time.perf_counter() < target_wall:
                    cancellation.check()
                    ros.spin_once(
                        max(
                            0.0,
                            min(0.02, target_wall - time.perf_counter()),
                        )
                    )
                ros.spin_once(0)
            foot_xyz = []  # Runtime foot contact guards retained; optional truth-only rigid-prim queries removed.
            rows.append(
                {
                    "foot_centers_world": foot_xyz,
                    "contacts": contact_monitor.contact_state.copy(),
                    "contact_callback_count": contact_monitor.callback_count,
                    "time_s": float(SimulationManager.get_simulation_time()),
                    "q": q.tolist(),
                    "qd": qd.tolist(),
                    "tau_cmd": measurement.tau.tolist(),
                    "root_pose_wxyz": [
                        measurement.pos.tolist(),
                        measurement.quat.tolist(),
                    ],
                    "root_velocity_world": [
                        measurement.v.numpy().tolist(),
                        measurement.w.numpy().tolist(),
                    ],
                }
            )
            receipts.journal(
                "truth_scoring_stream.jsonl",
                {
                    "time": measurement.actual_time,
                    "wall_since_loop_start": time.perf_counter() - loop_start,
                    "position": measurement.pos.tolist(),
                    "quat_wxyz": measurement.quat.tolist(),
                    "velocity": measurement.v.numpy()[0].tolist(),
                    "omega": measurement.w.numpy()[0].tolist(),
                    "contacts": contact_monitor.contact_state.copy(),
                },
            )
            receipts.journal("imu_stream.jsonl", sensor_rows[-1])
            # Full evidence lives in JSONL streams; keep a bounded final receipt.
            del rows[:-500]
            del sensor_rows[:-500]
            del policy_frames[:-500]
            del camera_rows[:-500]
            executed_steps = i + 1
            if control.check_termination(
                iteration=i,
                actual_time=measurement.actual_time,
                v=measurement.v,
                w=measurement.w,
            ):
                break

        cancellation.check()
        mapping.finish(receipts.result)
        receipts.close_journals()
        cancellation.check()
        video.close(timeout=15)
        context.video = None
        receipts.result["executed_physics_steps"] = executed_steps
        receipts.result["video"] = {
            "path": str(config.output / "multiwaypoint05_overview_and_rgb.mp4"),
            "frames": stream.video_frames,
            "fps": 10,
            "duration_s": stream.video_frames / 10,
            "encoder_exit": video.returncode,
            "camera": "body-following third person plus actual RGB input; camera motion visualization only, never navigation input",
        }
        if context.rgb_archive is not None:
            receipts.result["rgb_archive"] = context.rgb_archive.receipt(
                acquired_frames=stream.rgb_frames_acquired,
                completed=True,
                published_frames=ros.published["rgb"],
                video_frames=stream.video_frames,
            )
        receipts.result.update(
            stage="articulation_contact_probe_complete",
            contact_paths=contact_monitor.contact_paths,
            contact_events=contact_monitor.contact_events,
            dynamic_contact_events=contact_monitor.dynamic_contact_events,
            policy_frames=policy_frames,
            rows=rows,
            sensor_rows=sensor_rows,
            camera_rows=camera_rows,
            published=ros.published,
            latest_odom=ros.latest_odom[0],
            loop_wall_seconds=time.perf_counter() - loop_start,
            physics_dt=0.005,
            policy_running=True,
            contact_contract_validated=False,
            com_velocity_convention_validated=False,
            scan_connected=False,
            navigation_running=False,
            navigation_command_source="native_pose_diagnostic"
            if config.gt_diagnostic
            else config.experiment.get("navigation_command_source", "measured_lio_route")
            if config.slam_control
            else "/dynanav/nav/approved_cmd",
        )
        receipts.save(force=True)
        cancellation.check()
        ros.destroy()
        context.ros = None
        # Native fast shutdown may os._exit without returning. Completion is
        # capture completion, not proof that native cleanup finished.
        receipts.result.update(
            outcome="completed", app_close_requested=True, app_close_returned=False,
        )
        receipts.save(force=True)
        cancellation.check()
        closing_app = True
        stop(app, previous_delegates)
        context.app = None
        receipts.result["app_close_returned"] = True
        receipts.save(force=True)
    except BaseException as error:
        if cancellation.signum is not None:
            receipts.result["cancellation_signal"] = cancellation.signum
            if not isinstance(error, CaptureCancellation):
                # Group cancellation can kill ffmpeg before the main loop
                # observes the latch. Keep that I/O error as evidence, while
                # reporting the already-requested cancellation as the outcome.
                cancelled = CaptureCancellation(cancellation.signum)
                record_failure(receipts, cancelled, context, interrupted_error=error)
                raise cancelled from error
        if closing_app and isinstance(error, SystemExit) and error.code in (None, 0):
            # A successful SDK close may terminate Python instead of returning.
            raise
        record_failure(receipts, error, context)
        raise
    finally:
        _restore_signal_handlers(original_signal_handlers)


if __name__ == "__main__":
    main()
