"""Explicit-entry DynaNav Isaac RGB-D capture experiment.

Importing this module has no side effects: it only defines the argv/environment
contract plus ``run``/``main``. ``experiment.py`` keeps executing it through ``runpy`` as
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

from itertools import count
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
    # Appended with a default so the original positional construction contract of
    # this dataclass (and every existing caller) stays unchanged.
    interactive_control: bool = False


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
    mode = experiment.get("control_mode")
    gt_diagnostic = mode == "native_pose_diagnostic"
    # The interactive waypoint explorer is a measured-pose mode: it reuses the
    # measured-SLAM plumbing for mapping and pose labels, while the tracker that
    # actually drives the base is created separately below.
    interactive_control = mode == "interactive_waypoint_exploration"
    slam_control = mode in ("slam_pose_diagnostic", "interactive_waypoint_exploration")
    robogauge = experiment.get("policy") == "robogauge"
    use_fabric = experiment.get("render_backend", "fabric") == "fabric"
    from embodied_navigation.simulation.dynanav.capture.lockstep import validate_profile
    validate_profile(experiment)
    if experiment.get("point_cloud_source", "rgbd") == "rtx_lidar" and not use_fabric:
        raise ValueError("RTX LiDAR requires Fabric physics updates so attached sensors follow the body")
    if (gt_diagnostic or slam_control) and not robogauge:
        raise ValueError(
            "Native-pose diagnostic requires the explicit RoboGauge experiment"
        )
    if interactive_control and gt_diagnostic:
        raise ValueError(
            "Interactive waypoint exploration cannot be a ground-truth navigation mode"
        )
    return CaptureConfig(
        output=out,
        scene=argv[2],
        experiment=experiment,
        gt_diagnostic=gt_diagnostic,
        slam_control=slam_control,
        interactive_control=interactive_control,
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
        from embodied_navigation.simulation.dynanav.capture.lockstep import validate_profile
        lockstep_config = validate_profile(config.experiment)
        if lockstep_config is not None:
            receipts.result["sensor_lockstep"] = dict(lockstep_config)
        context.rgb_archive = open_rgb_archive(
            config.output, config.experiment, receipts.result
        )
        receipts.save()

        original_signal_handlers = {
            signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)
        }
        rtx_lidar_enabled = config.experiment.get("point_cloud_source", "rgbd") == "rtx_lidar"
        app = (start_simulation_app(config.use_fabric, rtx_lidar=True)
               if rtx_lidar_enabled else start_simulation_app(config.use_fabric))
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

        lidar_asset = None
        if rtx_lidar_enabled:
            from embodied_navigation.simulation.dynanav.capture.lidar import prepare_lidar_asset
            lidar_asset, conversion = prepare_lidar_asset(config.output)
            receipts.result["rtx_lidar_asset"] = conversion
            receipts.result["truth_usage"] = ("policy proprioception, physical guards and isolated offline scoring; "
                                               "never SLAM points, estimator pose or waypoint input")

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
        if rtx_lidar_enabled:
            from embodied_navigation.simulation.dynanav.capture.lidar import RtxLidar
            rtx_lidar = RtxLidar(camera_rig, lidar_asset)
            rgbd_annotators = create_rgbd_annotators(
                camera_rig, include_depth=bool(config.experiment.get("multicamera_observation", False)),
                recover_invalid_native_frames=(config.experiment.get("interactive_controller") in ("native_route", "pct_scan")),
            )
        else:
            rtx_lidar = None
            rgbd_annotators = create_rgbd_annotators(camera_rig)
        from embodied_navigation.simulation.dynanav.capture.multicamera_sensor import create_multicamera
        multicamera = create_multicamera(stage, camera_rig, rgbd_annotators, config.experiment, config.output)
        from embodied_navigation.simulation.dynanav.capture.nearfield_sensor import create_nearfield_depth
        nearfield_depth = create_nearfield_depth(stage, camera_rig, config.experiment)
        if nearfield_depth is not None:
            from embodied_navigation.simulation.dynanav.capture.nearfield import calibration_receipt
            receipts.result["simulated_nearfield_depth"] = calibration_receipt()
            receipts.result["nearfield_depth_control_enabled"] = False
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
        if rtx_lidar is not None:
            receipts.result["sensor_installation"].update(
                modality="RGB plus physical IMU plus RTX rotating LiDAR; depth point cloud disabled",
                lidar=rtx_lidar.installation(),
            )
        receipts.save()

        cancellation.check()
        ros = create_ros_interface(
            config.experiment, mapping.learned_mapper, mapping.learned_scorer,
            journal=lambda name, payload: receipts.journal(name, payload, flush=True),
        )
        receipts.result["sensor_recording_qos"] = {
            "reliable_source_writers": os.environ.get("DYNANAV_RECORD_RELIABLE") == "1",
            "source_history_depth": 128 if os.environ.get("DYNANAV_RECORD_RELIABLE") == "1" else 5,
            "scope": "sensor writer DDS delivery only; does not change estimator measurement values or rates",
        }
        context.ros = ros
        context.published = ros.published
        context.latest_odom = ros.latest_odom

        cancellation.check()
        physics_options = {"rtx_lidar": True} if rtx_lidar is not None else {}
        art = setup_physics(
            roots, config.use_fabric, app, receipts.result, receipts.save, **physics_options
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
            interactive_control=config.interactive_control,
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
            interactive_control=config.interactive_control,
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
        if rtx_lidar is not None:
            from isaacsim.core.simulation_manager import SimulationManager
            rtx_lidar.minimum_scan_start_ns = round(float(SimulationManager.get_simulation_time()) * 1e9)
            rtx_lidar.pending.clear()
            rtx_lidar.scans = 0
            rtx_lidar.last_key = None
            rgbd_annotators.minimum_stamp_ns = rtx_lidar.minimum_scan_start_ns
            rgbd_annotators.rtx_cache.clear()
            rgbd_annotators.rtx_last_native_ns = None
            if nearfield_depth is not None:
                nearfield_depth.reset_start(rtx_lidar.minimum_scan_start_ns)
            if multicamera is not None:
                multicamera.reset_start(rtx_lidar.minimum_scan_start_ns)
            receipts.result["rtx_minimum_scan_start_ns"] = rtx_lidar.minimum_scan_start_ns

        control = ControlLoop(
            experiment=config.experiment,
            gt_diagnostic=config.gt_diagnostic,
            slam_control=config.slam_control,
            interactive_control=config.interactive_control,
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
        def capture_render_outputs():
            rtx_lidar.capture()
            rgbd_annotators.capture_rtx_frame(rtx_lidar.minimum_scan_start_ns)
            if multicamera is not None:
                multicamera.capture(rtx_lidar.minimum_scan_start_ns)

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
            sync_usd_for_camera=rtx_lidar is not None,
            capture_render_outputs=capture_render_outputs if rtx_lidar is not None else None,
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
            rtx_lidar=rtx_lidar,
            nearfield_depth=nearfield_depth,
            multicamera=multicamera,
        )
        context.stream = stream
        imu_stride = config.experiment.get("imu_publish_stride", 2)
        imu_orientation_available = config.experiment.get(
            "imu_orientation_available", True
        )
        from isaacsim.core.simulation_manager import SimulationManager

        last_clock_published_wall = None
        iterations = (count() if config.experiment.get("duration_s", 0) is None
                      else range(int(os.environ.get("PROBE_STEPS", "4400"))))
        for i in iterations:
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
            if rtx_lidar is not None and (i + 1) % 10 == 0:
                rtx_lidar.capture()
            control.observe_imu(measurement.reading)
            previous_completed_tau = measurement.previous_completed_tau
            if (i + 1) % imu_stride == 0:
                if lockstep_config is not None:
                    # Conservative local boundary, not a downstream receipt or
                    # a replacement for any original source arrival.
                    last_clock_published_wall = time.perf_counter()
                ros.publish_imu_and_clock(
                    measurement.it, measurement.reading, imu_orientation_available
                )
            if rtx_lidar is not None:
                stream.publish_rtx_pending(measurement.actual_time)
            if (i + 1) % 20 == 0:
                while rtx_lidar is None or rgbd_annotators.has_rtx_frame():
                    stream.process(
                        iteration=i,
                        measurement=measurement,
                        pedestrian_view=pedestrian_view,
                        pedestrian_target=pedestrian_target,
                        dynamic_guard=dynamic_guard,
                        camera_rows=camera_rows,
                    )
                    if rtx_lidar is None:
                        break
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
            if lockstep_config is not None and (i + 1) % 20 == 0:
                from isaacsim.core.rendering_manager import RenderingManager
                from embodied_navigation.simulation.dynanav.capture.lockstep import (
                    bounded_wait, original_wall_deadline, pipeline_hint,
                    simulation_snapshot,
                )

                def pump_lockstep(remaining_wall_s):
                    # Only flush actual native completion events and consume
                    # their original queues. No IMU/clock, control or policy tick.
                    RenderingManager.render()
                    capture_render_outputs()
                    stream.publish_rtx_pending(measurement.actual_time)
                    while rgbd_annotators.has_rtx_frame():
                        cancellation.check()
                        stream.process(
                            iteration=i, measurement=measurement,
                            pedestrian_view=pedestrian_view,
                            pedestrian_target=pedestrian_target,
                            dynamic_guard=dynamic_guard, camera_rows=camera_rows,
                        )
                    for _lockstep_callback in range(8):
                        cancellation.check()
                        ros.spin_once(0.0)
                    # A tiny wall wait lets independent ROS/SLAM consumers run;
                    # it does not move simulation time or renew a source.
                    ros.spin_once(min(0.002, remaining_wall_s))

                def stop_lockstep():
                    return bool(
                        control.source_terminal_reason is not None
                        or tracker.terminal_reason is not None
                        or pathlib.Path(os.environ.get(
                            "NAV_STOP_REQUEST", "/tmp/no_multistage_stop_request"
                        )).exists()
                    )

                evidence = bounded_wait(
                    max_wait_s=lockstep_config["max_wait_wall_s"],
                    clock_published_wall_s=last_clock_published_wall,
                    snapshot=lambda: simulation_snapshot(art),
                    pump=pump_lockstep,
                    ready=lambda now: pipeline_hint(
                        tracker=tracker, odom=ros.latest_odom[0],
                        now_sim_s=measurement.actual_time, now_wall_s=now,
                    ),
                    cancelled=cancellation.check, stop_requested=stop_lockstep,
                    wall_deadline=original_wall_deadline(tracker),
                )
                evidence["iteration"] = i
                evidence["elapsed_wall_s"] = evidence["finished_wall_s"] - evidence["started_wall_s"]
                receipts.journal("sensor_lockstep.jsonl", evidence)
                receipts.result["sensor_lockstep_last_wait"] = evidence
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

        if rtx_lidar is not None:
            receipts.result["rgb_native_capture"] = {
                "backend": "per-render-product native NEW_FRAME event",
                "callbacks": rgbd_annotators.rtx_callback_count,
                "invalid_reference_count": rgbd_annotators.rtx_invalid_reference_count,
                "invalid_frame_recovery": rgbd_annotators.recover_invalid_native_frames,
                "invalid_frame_count": rgbd_annotators.native_frame_drop_count,
                "last_invalid_frame": rgbd_annotators.last_native_frame_drop,
                "verified_rgb_frames_missing": ros.published.get("rgb", 0) == 0,
                "current_body_stamp_substituted": False,
            }
            rgbd_annotators.rtx_listener = None
        if multicamera is not None:
            receipts.result["multicamera_capture"] = multicamera.close()
        if rtx_lidar is not None and ros.published.get("rtx_lidar", 0) == 0:
            raise RuntimeError("RTX source emitted no verified complete scans; RGB-D fallback is disabled")
        if (rtx_lidar is not None and ros.published.get("rgb", 0) == 0
                and not rgbd_annotators.recover_invalid_native_frames):
            raise RuntimeError("RTX mode emitted no verified native RGB frames")
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
            # Connection evidence comes from a native spline actually consumed
            # by the follower, not merely a configured planner or ROS graph.
            scan_connected=any(
                event.get("event") == "native_scan_reference"
                and event.get("provenance", {}).get("local_planner") == "native_SCAN"
                for event in receipts.result.get("route_tracker", {}).get("events", [])
            ),
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
