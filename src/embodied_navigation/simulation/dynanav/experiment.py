"""Explicit, reproducible Isaac diagnostics. Native pose is never called SLAM."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import runpy
import sys
from pathlib import Path

from embodied_navigation.project import data_root


def float32_sensor_time_tolerance(duration_s):
    """Half a float32 clock ULP at the run horizon plus the existing numeric margin."""
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("A positive finite sensor clock horizon is required")
    half_ulp = math.ldexp(1.0, math.frexp(duration_s)[1] - 25)
    return max(2e-6, half_ulp) + 1e-7


def joint_speed_observation(
    names, velocities, *, nominal_limit=30.0, readback_tolerance=0.0
):
    """Report measured speeds; never clamp state, actuator torque or engine limits."""
    values = tuple(float(value) for value in velocities)
    if not values or len(values) != len(names):
        raise ValueError("One measured speed per named joint is required")
    if (
        not math.isfinite(nominal_limit)
        or nominal_limit <= 0
        or not math.isfinite(readback_tolerance)
        or readback_tolerance < 0
    ):
        raise ValueError(
            "A positive nominal limit and nonnegative finite tolerance are required"
        )
    finite = all(math.isfinite(value) for value in values)
    index = max(range(len(values)), key=lambda j: abs(values[j])) if finite else None
    peak = abs(values[index]) if finite else None
    return {
        "nominal_limit_radps": nominal_limit,
        "readback_tolerance_radps": readback_tolerance,
        "abort_limit_radps": nominal_limit + readback_tolerance,
        "max_abs_radps": peak,
        "worst_joint": names[index] if finite else None,
        "nominal_exceedance_radps": max(0.0, peak - nominal_limit) if finite else None,
        "within_guard": finite and peak <= nominal_limit + readback_tolerance,
        "state_or_motor_command_clamped": False,
    }


def make_profile(speed_mps=1.0, duration_s=90.0, *, interactive=False):
    """First physical stair flight; speed is a command ceiling, not achieved speed."""
    if not math.isfinite(speed_mps) or not 0 <= speed_mps <= 1.0:
        raise ValueError("Diagnostic speed must be finite and within 0..1 m/s")
    maximum_duration_s = 1800.0 if interactive else 300.0
    if not math.isfinite(duration_s) or not 8 <= duration_s <= maximum_duration_s:
        raise ValueError(
            f"Diagnostic duration must be finite and within 8..{maximum_duration_s:g} s"
        )
    return {
        "schema": "dynanav-diagnostic/v1",
        "policy": "robogauge",
        "control_mode": "native_pose_diagnostic",
        "spawn": [-14.53, -7.5, -2.85],
        "spawn_yaw": math.pi,
        "waypoints_xyz": [[-22.0, -7.5, -1.1441]],
        "speed_mps": speed_mps,
        "motion_end_s": duration_s - 5.0,
        "duration_s": duration_s,
        "basement_collision": True,
        "minimum_base_z_m": -3.15,
        "maximum_displacement_m": 15.0,
        "physical_speed_guard_mps": max(0.65, speed_mps + 0.5),
        "joint_speed_readback_tolerance_radps": 0.001,
        "body_contact_report_only": True,
        "camera_yaw_rad": 0.0,
        "mapping_pose_source": "disabled",
        "stair_specific_speed_limit": None,
        "stop_on_stall": True,
        "stall_window_s": 8.0,
        "stall_min_progress_m": 0.02,
        "semantic_interval_ns": 2_000_000_000,
        "map_keyframe_interval_ns": 1_000_000_000,
        "semantic_provider": "none",
        "notes": "Isolated simulation, direct policy command, no COM PID/slew; tilt/motor/clock checks retained. Base contact is report-only in this isolated diagnostic; tilt and motor bounds still abort.",
    }


def configure_point_cloud_source(profile, source, semantic_provider=None):
    """Select an actual sensor pipeline without relabelling RGB-D as LiDAR."""
    if source not in ("rgbd", "rtx_lidar"):
        raise ValueError("point-cloud source must be rgbd or rtx_lidar")
    selected_semantics = semantic_provider or "none"
    if selected_semantics != "none":
        raise ValueError("unsupported semantic provider")
    profile["point_cloud_source"] = source
    profile["semantic_provider"] = selected_semantics
    profile.update(mapping_enabled=False, mapping_pose_source="disabled")
    if source == "rtx_lidar":
        profile.update(
            lidar_frontend_topic="/dynanav/lidar_rtx",
            rtx_lidar_config="OS1_REV6_32ch10hz1024res",
            lidar_sensor_model="official Ouster OS1 Rev6 32-channel 10 Hz RTX model",
            lidar_frame_id="go2_imu",
            lidar_scan_hz=10.0,
            lidar_scan_line=32,
            lidar_timestamp_contract="scan-start header plus per-return uint32 t in nanoseconds",
            lidar_motion_compensation="estimator IMU deskew only; no ground-truth deskew",
            mapping_disabled_reason="RGB-D semantic projection is incompatible with RTX LiDAR returns",
        )
    return profile


def slam_odometry_topic(pose_source, requested=None):
    """Bind post-VIO control to its dedicated final-state output, never pre-VIO."""
    if pose_source not in ("lio_estimate", "livo_post_vio"):
        raise ValueError("unsupported measured navigation pose source")
    post_vio = pose_source == "livo_post_vio"
    topic = (
        requested
        if requested is not None
        else (
            "/dynanav/lio_observer/post_vio_odometry"
            if post_vio
            else "/dynanav/lio_observer/aft_mapped_to_init"
        )
    )
    if (
        not isinstance(topic, str)
        or not topic.startswith("/")
        or any(c.isspace() for c in topic)
    ):
        raise ValueError("measured navigation odometry topic must be absolute")
    if post_vio and topic.rsplit("/", 1)[-1] != "post_vio_odometry":
        raise ValueError(
            "post-VIO navigation requires the dedicated /post_vio_odometry final topic"
        )
    return topic


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene", type=Path)
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Body command ceiling; no stair-specific reduction",
    )
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--rtf", type=float, default=1.0)
    parser.add_argument(
        "--check",
        action="store_true",
        help="Validate assets/profile without starting Isaac",
    )
    parser.add_argument(
        "--spawn-yaw", type=float,
        help="Initial simulation heading in radians; does not select a navigation target",
    )
    parser.add_argument(
        "--render-backend", choices=("fabric", "usd"), default="usd",
        help="RGB-D render backend; RTX always uses Fabric and records the requested choice",
    )
    parser.add_argument(
        "--odometry-topic",
        help="Explicit estimator output; default follows reference source",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help=(
            "Simulation-only operator waypoint exploration: measured pose only, "
            "waypoints arrive through a local mailbox directory"
        ),
    )
    parser.add_argument(
        "--interactive-controller", choices=("native_route", "pct_scan"),
        default="native_route", help="Route backend; pct_scan uses measured PCT and native SCAN",
    )
    parser.add_argument(
        "--interactive-mailbox",
        type=Path,
        help="Mailbox directory for operator waypoints (default <output>/interactive)",
    )
    parser.add_argument(
        "--interactive-pose-source",
        choices=("lio_estimate", "livo_post_vio"),
        help="Measured estimator source that may drive interactive waypoints",
    )
    parser.add_argument("--interactive-waypoint-ttl-s", type=float, default=30.0)
    parser.add_argument("--interactive-link-timeout-s", type=float, default=5.0)
    parser.add_argument("--interactive-pose-loss-timeout-s", type=float, default=5.0)
    parser.add_argument("--interactive-max-queue", type=int, default=4)
    parser.add_argument("--interactive-max-segment-m", type=float, default=8.0)
    parser.add_argument("--interactive-min-segment-m", type=float, default=0.15)
    parser.add_argument(
        "--interactive-capture-remaining-m", type=float, default=0.20
    )
    parser.add_argument("--interactive-capture-distance-m", type=float, default=0.30)
    parser.add_argument(
        "--interactive-forward-turn-mode",
        action="store_true",
        help=(
            "explicit production forward+turn policy: the deterministic follower "
            "steers with yaw only and never emits a lateral body command; default "
            "off keeps the existing holonomic follower and its side-direction gate"
        ),
    )
    parser.add_argument(
        "--imu-hz",
        type=int,
        choices=(100, 200),
        default=200,
        help="Publish recorded native IMU samples; 100 Hz reproduces legacy decimation",
    )
    parser.add_argument(
        "--point-cloud-source",
        choices=("rgbd", "rtx_lidar"),
        default="rgbd",
        help="Actual cloud sensor: legacy RGB-D or official 32-channel 10 Hz RTX Ouster",
    )
    parser.add_argument(
        "--semantic-provider",
        choices=("none",),
        help="Default: none",
    )
    args = parser.parse_args(argv)
    interactive_requested = args.interactive or args.interactive_mailbox is not None
    if interactive_requested and args.interactive_pose_source is None:
        parser.error(
            "--interactive requires --interactive-pose-source (measured estimator only)"
        )
    if args.interactive_controller == "pct_scan" and (not interactive_requested or args.interactive_pose_source != "livo_post_vio"):
        parser.error("pct_scan requires --interactive --interactive-pose-source livo_post_vio")
    profile = make_profile(args.speed, args.duration, interactive=interactive_requested)
    if args.spawn_yaw is not None:
        if not math.isfinite(args.spawn_yaw):
            parser.error("--spawn-yaw must be finite")
        profile["spawn_yaw"] = args.spawn_yaw
    try:
        configure_point_cloud_source(profile, args.point_cloud_source, args.semantic_provider)
    except ValueError as error:
        parser.error(str(error))
    profile["imu_publish_hz"] = args.imu_hz
    profile["imu_publish_stride"] = 200 // args.imu_hz
    profile["imu_orientation_available"] = False
    profile["render_backend"] = args.render_backend
    if args.point_cloud_source == "rtx_lidar":
        # RTX/FSD can return frames while still using the initial body transform
        # when physics Fabric updates are disabled. A live sensor stream alone
        # does not establish that its mount follows the moving articulation.
        profile.update(
            requested_render_backend=args.render_backend,
            render_backend="fabric",
            render_backend_override_reason=(
                "RTX-mounted camera and LiDAR require physics Fabric updates; "
                "FSD alone can leave sensor poses at the initial body transform"
            ),
        )
    from embodied_navigation.simulation.dynanav.session_collision import (
        CONFIG_PATH as DEFAULT_COLLISION_CONFIG_PATH,
        load_config as load_collision_config,
    )

    collision_config = load_collision_config(DEFAULT_COLLISION_CONFIG_PATH)
    profile["route"] = "first-flight"
    profile["waypoints_xyz"] = [[-22.0, -7.5, -1.1441]]
    profile["collision_config_path"] = str(DEFAULT_COLLISION_CONFIG_PATH)
    profile["route_geometry_provenance"] = collision_config["provenance"]
    if not math.isfinite(args.rtf) or args.rtf <= 0:
        parser.error("--rtf must be positive and finite")
    data = data_root()
    scene = (
        args.scene
        or data
        / "dynanav/documents_codex_20261108/codex/2026-11-05/task-2/full_asset_render/bundle/office.usd"
    )
    from embodied_navigation.robot.unitree.robogauge.runtime import (
        default_model_path,
        verify_model,
    )

    verify_model(default_model_path())
    robot = data / "runtime_assets/robot/go2_description/go2_description.usda"
    if not scene.is_file() or not robot.is_file():
        parser.error("Required office scene or robot USD is missing")
    output = args.output.expanduser().resolve()
    if not output.is_relative_to(data.resolve()):
        parser.error(
            "Write large experiment outputs below PROJECT_DATA_ROOT/embodied-navigation"
        )
    if output.exists():
        parser.error("Output already exists; choose a new experiment directory")
    if interactive_requested:
        mailbox = (
            args.interactive_mailbox.expanduser().resolve()
            if args.interactive_mailbox is not None
            else output / "interactive"
        )
        if mailbox.exists():
            parser.error("Interactive mailbox directory must not exist yet")
        post_vio = args.interactive_pose_source == "livo_post_vio"
        profile.update(
            control_mode="interactive_waypoint_exploration",
            interactive_controller=args.interactive_controller,
            route="interactive_waypoint_exploration",
            waypoints_xyz=[],
            slam_pose_source=args.interactive_pose_source,
            navigation_pose_source=(
                "measured_livo_post_vio" if post_vio else "measured_lio"
            ),
            navigation_command_source="interactive_waypoint_exploration_measured_pose",
            odometry_topic=slam_odometry_topic(
                args.interactive_pose_source, args.odometry_topic
            ),
            slam_twist_available=not post_vio,
            slam_speed_source="pose_window" if post_vio else "twist",
            slam_pose_speed_window_s=0.5,
            slam_max_sim_age_s=0.25,
            slam_max_wall_age_s=0.5,
            interactive_mailbox=str(mailbox),
            interactive_capture_remaining_m=args.interactive_capture_remaining_m,
            interactive_capture_distance_m=args.interactive_capture_distance_m,
            interactive_simulation_only=True,
            interactive_require_perception=False,
            interactive_future_tolerance_s=2e-6,
            interactive_gt_usage=(
                "ground truth only for optional isolated oracle diagnostics and "
                "offline scoring; never a waypoint frame or a pose input"
            ),
            route_geometry_provenance="operator_selected_from_live_sensors_and_SLAM_only",
            navigation_scope=(
                "measured "
                + ("post-VIO" if post_vio else "LIO")
                + " operator waypoints in the live estimator map frame; "
                "simulation-only exploration, no global localization claim"
            ),
        )
        profile["notes"] = (
            "Static simulation native route exploration; configured attitude, "
            "base-speed and position envelopes are diagnostic only. "
            "Finite-pose, effort, motor and clock checks remain enforced; "
            "body contact follows body_contact_report_only. "
            "These physics readbacks are not navigation inputs."
        )
    if args.check:
        print(
            json.dumps(
                {
                    "assets_checked": True,
                    "model_hash_checked": True,
                    "simulation_started": False,
                    "controller_config_checked": bool(interactive_requested),
                    "profile": profile,
                },
                indent=2,
            )
        )
        return 0
    output.mkdir(parents=True)
    profile_path = output / "experiment.json"
    profile_path.write_text(json.dumps(profile, indent=2))
    from embodied_navigation.project import project_root

    root = project_root()
    sources = [
        "src/embodied_navigation/simulation/dynanav/experiment.py",
        "src/embodied_navigation/simulation/dynanav/capture_go2_rgbd.py",
        "src/embodied_navigation/simulation/dynanav/capture/__init__.py",
        "src/embodied_navigation/simulation/dynanav/capture/control.py",
        "src/embodied_navigation/simulation/dynanav/capture/frames.py",
        "src/embodied_navigation/simulation/dynanav/capture/lidar.py",
        "src/embodied_navigation/simulation/dynanav/capture/physics.py",
        "src/embodied_navigation/simulation/dynanav/capture/policy.py",
        "src/embodied_navigation/simulation/dynanav/capture/receipts.py",
        "src/embodied_navigation/simulation/dynanav/capture/ros_interface.py",
        "src/embodied_navigation/simulation/dynanav/capture/scene.py",
        "src/embodied_navigation/simulation/dynanav/capture/sensors.py",
        "src/embodied_navigation/simulation/dynanav/capture/video.py",
        "scripts/query_scene_map.py",
        "src/embodied_navigation/localization_mapping/semantic_mapping/query.py",
        "src/embodied_navigation/simulation/dynanav/render_diagnostics.py",
        "src/embodied_navigation/simulation/dynanav/session_collision.py",
        "src/embodied_navigation/simulation/dynanav/native_waypoints.py",
        "src/embodied_navigation/simulation/dynanav/pct_scan_waypoints.py",
        "src/embodied_navigation/simulation/dynanav/pct_scan_worker.py",
        "src/embodied_navigation/planning/pct/measured_tomogram.py",
        "src/embodied_navigation/planning/exploration/route_information.py",
        "src/embodied_navigation/planning/exploration/waypoint_candidates.py",
        "src/embodied_navigation/planning/exploration/os1_calibration.json",
        "scripts/prepare_dynanav_waypoint_candidates.py",
        "scripts/dynanav_exploration.py",
        "scripts/run_dynanav_pct_scan.py",
        "src/embodied_navigation/simulation/dynanav/rtx_deskewed_perception.py",
        "src/embodied_navigation/simulation/dynanav/operator_map.py",
        "scripts/explore_waypoint_client.py",
        "configs/simulation/dynanav_basement_to_ground_collision.json",
        "src/embodied_navigation/localization_mapping/semantic_mapping/core.py",
        "src/embodied_navigation/localization_mapping/semantic_mapping/scheduler.py",
        "src/embodied_navigation/perception/depth.py",
        "src/embodied_navigation/navigation/robogauge/route_tracker.py",
        "src/embodied_navigation/robot/unitree/robogauge/contract.py",
        "src/embodied_navigation/robot/unitree/robogauge/runtime.py",
        "configs/simulation/dynanav_basement_collision.json",
    ]
    manifest = {}
    for relative in sources:
        content = (root / relative).read_bytes()
        manifest[relative] = hashlib.sha256(content).hexdigest()
        target = output / "source_snapshot" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(content)
    (output / "source_manifest.json").write_text(json.dumps(manifest, indent=2))
    os.environ.update(
        DYNANAV_EXPERIMENT=str(profile_path),
        PROBE_STEPS=str(round(args.duration / 0.005)),
        PROBE_RTF=str(args.rtf),
        NAV_STOP_REQUEST=str(output / "stop_request.json"),
    )
    sys.argv = ["capture_go2_rgbd", str(output), str(scene)]
    runpy.run_module(
        "embodied_navigation.simulation.dynanav.capture_go2_rgbd", run_name="__main__"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
