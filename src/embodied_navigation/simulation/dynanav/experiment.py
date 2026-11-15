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
from embodied_navigation.common.time import float32_sensor_time_tolerance


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


def make_profile(speed_mps=1.0, duration_s=90.0, *, interactive=False, until_stopped=False):
    """First physical stair flight; speed is a command ceiling, not achieved speed."""
    if not math.isfinite(speed_mps) or not 0 <= speed_mps <= 1.0:
        raise ValueError("Diagnostic speed must be finite and within 0..1 m/s")
    if until_stopped and not interactive:
        raise ValueError("Unbounded duration requires interactive exploration")
    maximum_duration_s = 1800.0 if interactive else 300.0
    if not math.isfinite(duration_s) or not 8 <= duration_s <= maximum_duration_s:
        raise ValueError(
            f"Diagnostic duration must be finite and within 8..{maximum_duration_s:g} s"
        )
    duration_s = None if until_stopped else duration_s
    return {
        "schema": "dynanav-diagnostic/v1",
        "policy": "robogauge",
        "control_mode": "native_pose_diagnostic",
        "spawn": [-14.53, -7.5, -2.85],
        "spawn_yaw": math.pi,
        "waypoints_xyz": [[-22.0, -7.5, -1.1441]],
        "speed_mps": speed_mps,
        "motion_end_s": None if duration_s is None else duration_s - 5.0,
        "duration_s": duration_s,
        "basement_collision": True,
        "minimum_base_z_m": -3.15,
        "maximum_displacement_m": 15.0,
        "physical_speed_guard_mps": max(0.65, speed_mps + 0.5),
        "joint_speed_readback_tolerance_radps": 0.001,
        "body_contact_report_only": True,
        "camera_yaw_rad": 0.0,
        "mapping_pose_source": "simulator_native_pose_oracle",
        "stair_specific_speed_limit": None,
        "stop_on_stall": True,
        "stall_window_s": 8.0,
        "stall_min_progress_m": 0.02,
        "semantic_interval_ns": 2_000_000_000,
        "map_keyframe_interval_ns": 1_000_000_000,
        "semantic_provider": "isaac_instance_oracle",
        "notes": "Isolated simulation, direct policy command, no COM PID/slew; tilt/motor/clock checks retained. Base contact is report-only in this isolated diagnostic; tilt and motor bounds still abort.",
    }


def configure_point_cloud_source(profile, source, semantic_provider=None):
    """Select an actual sensor pipeline without relabelling RGB-D as LiDAR."""
    if source not in ("rgbd", "rtx_lidar"):
        raise ValueError("point-cloud source must be rgbd or rtx_lidar")
    selected_semantics = semantic_provider or (
        "none" if source == "rtx_lidar" else "isaac_instance_oracle"
    )
    if selected_semantics not in ("none", "isaac_instance_oracle", "learned_rgb_person"):
        raise ValueError("unsupported semantic provider")
    if source == "rtx_lidar" and selected_semantics != "none":
        raise ValueError(
            "RTX LiDAR does not support RGB-D oracle/learned semantic mapping; "
            "omit --semantic-provider or select none"
        )
    profile["point_cloud_source"] = source
    profile["semantic_provider"] = selected_semantics
    if selected_semantics == "none":
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


def make_slam_controller(profile):
    """Use the same pure controller construction in --check and the live loop."""
    from embodied_navigation.simulation.dynanav.slam_route import SlamRouteTracker

    return SlamRouteTracker(
        profile["slam_route_reference"],
        speed_mps=profile["speed_mps"],
        motion_end_s=profile["motion_end_s"],
        repeat_spawn_assumption=profile["repeat_spawn_assumption"],
        map_registration_enabled=profile["map_registration_enabled"],
        max_sim_age_s=0.25,
        max_wall_age_s=0.5,
        future_tolerance_s=profile["slam_future_tolerance_s"],
        stop_on_stall=profile.get("stop_on_stall", True),
        speed_source=profile["slam_speed_source"],
        pose_speed_window_s=profile["slam_pose_speed_window_s"],
        capture_remaining_m=profile.get("slam_capture_remaining_m", 0.20),
        capture_distance_m=profile.get("slam_capture_distance_m", 0.30),
        final_capture_remaining_m=profile.get("slam_final_capture_remaining_m"),
        final_capture_distance_m=profile.get("slam_final_capture_distance_m"),
    )


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
    duration = parser.add_mutually_exclusive_group()
    duration.add_argument("--duration", type=float, default=90.0)
    duration.add_argument("--until-stopped", action="store_true",
                          help="Interactive exploration without a total time budget")
    parser.add_argument("--rtf", type=float, default=1.0)
    parser.add_argument("--multicamera-observation", action="store_true",
                        help="Independent native RGB/depth views for DSH; primary single-camera SLAM unchanged")
    parser.add_argument("--multicamera-count", type=int, choices=(3,4,5,6), default=6,
                        help="Equally spaced cameras; default optics retain the existing 82.37-degree HFOV")
    parser.add_argument("--multicamera-horizontal-fov", type=float, default=None,
                        help="Explicit experimental pinhole HFOV for all views including camera_0; requires matching frontend calibration")
    parser.add_argument(
        "--simulated-nearfield-depth", action="store_true",
        help="Extra ideal downward depth hardware, diagnostic only; no SLAM/control fusion",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Validate assets/profile without starting Isaac",
    )
    from embodied_navigation.simulation.dynanav.routes import ROUTE_NAMES, get_route

    parser.add_argument("--route", choices=ROUTE_NAMES, default="first-flight")
    parser.add_argument(
        "--spawn-yaw", type=float,
        help="Initial simulation heading in radians; does not select a navigation target",
    )
    parser.add_argument(
        "--render-backend", choices=("fabric", "usd"), default="usd",
        help="RGB-D render backend; RTX always uses Fabric and records the requested choice",
    )
    parser.add_argument(
        "--archive-rgb",
        action="store_true",
        help="Archive every raw renderer RGB frame before video overlays (uncompressed NPZ)",
    )
    parser.add_argument(
        "--pedestrian",
        action="store_true",
        help="Local skeletal pedestrian with oracle point-cloud hold/resume",
    )
    parser.add_argument("--pedestrian-asset", type=Path)
    parser.add_argument("--pedestrian-walk", type=Path)
    parser.add_argument(
        "--pedestrian-path", choices=("one_way", "sine", "linear"), default="one_way"
    )
    parser.add_argument("--pedestrian-delay", type=float, default=4.0)
    parser.add_argument(
        "--slam-reference",
        type=Path,
        help="Previous measured LIO route; repeat-spawn lab assumption, no global registration",
    )
    parser.add_argument(
        "--odometry-topic",
        help="Explicit estimator output; default follows reference source",
    )
    parser.add_argument(
        "--slam-capture-remaining-m",
        type=float,
        default=0.20,
        help="Explicit measured-reference along-segment capture; old default retained",
    )
    parser.add_argument(
        "--slam-capture-distance-m",
        type=float,
        default=0.30,
        help="Explicit measured-reference XY capture; verify platform turn margin first",
    )
    parser.add_argument(
        "--slam-final-capture-remaining-m",
        type=float,
        help="Final goal only: along-segment capture; omitted inherits --slam-capture-remaining-m",
    )
    parser.add_argument(
        "--slam-final-capture-distance-m",
        type=float,
        help="Final goal only: XY capture; omitted inherits --slam-capture-distance-m",
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
        "--interactive-perception-owner",
        choices=("fail_closed", "rtx_deskewed_cloud"),
        default="fail_closed",
        help=(
            "who writes the interactive perception sample: fail_closed keeps the "
            "legacy per-frame not_connected payload; rtx_deskewed_cloud leaves "
            "perception.json to the native deskewed-cloud producer (explicit RTX opt-in)"
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
        "--pedestrian-scenario",
        choices=("spawn-crossing", "landing-crossing"),
        default="spawn-crossing",
    )
    parser.add_argument("--pedestrian-speed", type=float)
    parser.add_argument(
        "--point-cloud-source",
        choices=("rgbd", "rtx_lidar"),
        default="rgbd",
        help="Actual cloud sensor: legacy RGB-D or official 32-channel 10 Hz RTX Ouster",
    )
    parser.add_argument(
        "--nonrealtime-sensor-lockstep", action="store_true",
        help=("Opt-in static simulation benchmark: bounded render/consumer waits "
              "at sensor periods; not an asynchronous real-time pipeline fix. "
              "All original source stamps, wall ages and safety gates remain."),
    )
    parser.add_argument("--sensor-lockstep-max-wait-s", type=float, default=0.15,
                        help="Per-period nonrealtime wait cap in wall seconds, (0, 0.20]")
    parser.add_argument(
        "--semantic-provider",
        choices=("none", "isaac_instance_oracle", "learned_rgb_person"),
        help="Default: isaac_instance_oracle for RGB-D; none for RTX LiDAR",
    )
    parser.add_argument(
        "--person-worker",
        type=Path,
        help="Existing local worker channel; no model is downloaded",
    )
    parser.add_argument(
        "--pose-launch-receipt",
        type=Path,
        help="Actual observer launch receipt for the learned frontend binding",
    )
    parser.add_argument("--person-max-interval", type=float, default=0.4)
    parser.add_argument("--person-min-interval", type=float, default=0.2)
    parser.add_argument(
        "--learned-evaluation-oracle",
        action="store_true",
        help="Isolated oracle scoring sink; never an algorithm input",
    )
    args = parser.parse_args(argv)
    interactive_requested = args.interactive or args.interactive_mailbox is not None
    if interactive_requested and args.interactive_pose_source is None:
        parser.error(
            "--interactive requires --interactive-pose-source (measured estimator only)"
        )
    if args.interactive_controller == "pct_scan" and (not interactive_requested or args.interactive_pose_source != "livo_post_vio"):
        parser.error("pct_scan requires --interactive --interactive-pose-source livo_post_vio")
    if interactive_requested and args.slam_reference is not None:
        parser.error(
            "--interactive cannot be combined with --slam-reference; the explorer "
            "accepts operator waypoints, not a prerecorded route"
        )
    if args.until_stopped and not interactive_requested:
        parser.error("--until-stopped requires --interactive")
    profile = make_profile(args.speed, args.duration, interactive=interactive_requested,
                           until_stopped=args.until_stopped)
    if args.spawn_yaw is not None:
        if not math.isfinite(args.spawn_yaw):
            parser.error("--spawn-yaw must be finite")
        profile["spawn_yaw"] = args.spawn_yaw
    try:
        configure_point_cloud_source(profile, args.point_cloud_source, args.semantic_provider)
    except ValueError as error:
        parser.error(str(error))
    if args.point_cloud_source == "rtx_lidar" and args.pedestrian:
        parser.error("RTX LiDAR cannot use the RGB-D oracle pedestrian hold/mapping pipeline")
    if args.simulated_nearfield_depth and args.point_cloud_source != "rtx_lidar":
        parser.error("--simulated-nearfield-depth requires --point-cloud-source rtx_lidar")
    if args.multicamera_observation and args.point_cloud_source != "rtx_lidar":
        parser.error("--multicamera-observation requires --point-cloud-source rtx_lidar")
    if args.multicamera_count != 6 and not args.multicamera_observation:
        parser.error("--multicamera-count requires --multicamera-observation")
    if args.multicamera_horizontal_fov is not None:
        if not args.multicamera_observation:
            parser.error("--multicamera-horizontal-fov requires --multicamera-observation")
        from .multicamera import focal_length_for_hfov
        try:
            focal_length_for_hfov(args.multicamera_horizontal_fov)
        except ValueError as error:
            parser.error(str(error))
    profile["multicamera_horizontal_fov_deg"] = args.multicamera_horizontal_fov
    profile["multicamera_count"] = args.multicamera_count
    profile["multicamera_observation"] = args.multicamera_observation
    profile["simulated_nearfield_depth"] = args.simulated_nearfield_depth
    profile["nearfield_depth_control_enabled"] = False
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
    profile["archive_rgb"] = args.archive_rgb
    route = get_route(args.route)
    profile["route"] = route["name"]
    profile["waypoints_xyz"] = route["waypoints_xyz"]
    profile["collision_config_path"] = route["config_path"]
    profile["route_geometry_provenance"] = route["provenance"]
    if args.pedestrian:
        if not math.isfinite(args.pedestrian_delay) or args.pedestrian_delay < 0:
            parser.error("--pedestrian-delay must be finite and nonnegative")
        if args.render_backend != "usd":
            parser.error(
                "The live pedestrian diagnostic requires the measured USD render path"
            )
        asset = args.pedestrian_asset or Path(
            "/home/hyh001/isaacsim/extscache/isaacsim.replicator.agent.core-1.6.8+110.1.2/data/test_scenes/test_biped/biped_demo_meters.usd"
        )
        walk = args.pedestrian_walk or Path(
            "/home/hyh001/isaacsim/extscache/omni.anim.retarget.core-110.1.0+110.1.1.lx64.r.cp312.u7f4/data/rigs/Human/assets/walk.skelanim.usd"
        )
        if not asset.is_file() or not walk.is_file():
            parser.error("Local human and compatible walk assets are required")
        profile["pedestrian"] = {
            "asset": str(asset.resolve()),
            "walk": str(walk.resolve()),
            "asset_sha256": hashlib.sha256(asset.read_bytes()).hexdigest(),
            "walk_sha256": hashlib.sha256(walk.read_bytes()).hexdigest(),
            "start": [-15.25, -9.5, -3.3],
            "end": [-15.25, -5.5, -3.3],
            "speed_mps": 0.6,
            "start_delay_s": args.pedestrian_delay,
            "path_mode": args.pedestrian_path,
        }
        if args.pedestrian_scenario == "landing-crossing":
            profile["pedestrian"].update(
                start=[-22.0, -8.7, -1.4906366737798542],
                end=[-22.0, -6.3, -1.4906366737798542],
                speed_mps=0.3,
            )
        if args.pedestrian_speed is not None:
            profile["pedestrian"]["speed_mps"] = args.pedestrian_speed
        profile["pedestrian"]["scenario"] = args.pedestrian_scenario
        from embodied_navigation.simulation.dynanav.pedestrian import PedestrianPath

        PedestrianPath(
            profile["pedestrian"]["start"],
            profile["pedestrian"]["end"],
            profile["pedestrian"]["speed_mps"],
            profile["pedestrian"]["path_mode"],
        )
        profile["pedestrian_roots"] = ["/World/DynamicPedestrian"]
        profile["dynamic_hold"] = True
        profile["navigation_scope"] = (
            "native_pose_route_with_oracle_dynamic_hold; not SLAM navigation or general avoidance"
        )
    if args.slam_reference:
        from embodied_navigation.simulation.dynanav.slam_route import (
            load_slam_route_reference,
        )

        profile["slam_route_reference"] = load_slam_route_reference(args.slam_reference)
        profile["slam_pose_source"] = profile["slam_route_reference"]["pose_source"]
        post_vio = profile["slam_pose_source"] == "livo_post_vio"
        profile["navigation_pose_source"] = (
            "measured_livo_post_vio" if post_vio else "measured_lio"
        )
        profile["navigation_command_source"] = (
            "measured_livo_post_vio_route" if post_vio else "measured_lio_route"
        )
        profile["slam_twist_available"] = profile["slam_route_reference"]["source"].get(
            "twist_available", True
        )
        profile["slam_route_reference_path"] = str(args.slam_reference.resolve())
        profile["slam_route_reference_sha256"] = hashlib.sha256(
            args.slam_reference.read_bytes()
        ).hexdigest()
        profile["control_mode"] = "slam_pose_diagnostic"
        profile["odometry_topic"] = slam_odometry_topic(
            profile["slam_pose_source"], args.odometry_topic
        )
        profile["repeat_spawn_assumption"] = True
        profile["map_registration_enabled"] = False
        profile["slam_speed_source"] = "pose_window"
        profile["slam_pose_speed_window_s"] = 0.5
        profile["slam_capture_remaining_m"] = args.slam_capture_remaining_m
        profile["slam_capture_distance_m"] = args.slam_capture_distance_m
        profile["slam_final_capture_remaining_m"] = args.slam_final_capture_remaining_m
        profile["slam_final_capture_distance_m"] = args.slam_final_capture_distance_m
        profile["slam_future_tolerance_s"] = float32_sensor_time_tolerance(
            args.duration
        )
        profile["slam_timestamp_precision"] = (
            "SDK float32 sensor clock; half-ULP horizon bound plus numerical margin"
        )
        profile["navigation_scope"] = (
            "measured "
            + ("post-VIO" if post_vio else "LIO")
            + " route in repeat-spawn lab; no native route-pose input or global localization"
        )
    if args.slam_reference:
        make_slam_controller(profile)
    if args.semantic_provider == "learned_rgb_person":
        if (
            not args.slam_reference
            or profile.get("slam_pose_source") != "livo_post_vio"
        ):
            parser.error(
                "Online learned mapping requires an explicit post-VIO measured reference"
            )
        if args.person_worker is None:
            parser.error("Online learned mapping requires --person-worker")
        if not (
            math.isfinite(args.person_min_interval)
            and math.isfinite(args.person_max_interval)
            and 0 < args.person_min_interval <= args.person_max_interval
        ):
            parser.error("Person event intervals must be finite and 0 < min <= max")
        if args.pose_launch_receipt is None or not args.pose_launch_receipt.is_file():
            parser.error(
                "Online learned mapping requires an actual --pose-launch-receipt"
            )
        launch_path = args.pose_launch_receipt.expanduser().resolve()
        launch = json.loads(launch_path.read_text())
        if (
            launch.get("lidar_input_topic") != "/dynanav/lidar_perception_filtered"
            or launch.get("navigation_pose_topic") != profile["odometry_topic"]
            or launch.get("ground_truth_input") is not False
        ):
            parser.error(
                "Observer launch must bind the learned candidate cloud and selected post-VIO output, without GT pose input"
            )
        if str(launch.get("ros_domain_id")) != os.environ.get("ROS_DOMAIN_ID", "0"):
            parser.error("Observer launch receipt belongs to a different ROS domain")
        profile["pose_launch_receipt"] = str(launch_path)
        profile["pose_launch_receipt_sha256"] = hashlib.sha256(
            launch_path.read_bytes()
        ).hexdigest()
        channel = args.person_worker.expanduser().resolve()
        state_path = channel / "state.json"
        if not state_path.is_file():
            parser.error(
                "Person worker state.json is missing; start the explicit local worker first"
            )
        worker_state = json.loads(state_path.read_text())
        expected_model = (
            "73cbd0190fcbe3ba339921fbce2c3a0b6bb9126c9a133c85e43a2a8e060a109e"
        )
        if (
            worker_state.get("model_sha256") != expected_model
            or worker_state.get("state") != "ready"
        ):
            parser.error(
                "Person worker must be ready with the explicitly selected model hash"
            )
        profile.update(
            semantic_provider="learned_rgb_person",
            mapping_pose_source="live_livo_post_vio_exact",
            person_worker=str(channel),
            person_model_sha256=expected_model,
            person_max_interval_s=args.person_max_interval,
            person_min_interval_s=args.person_min_interval,
            oracle_evaluation=args.learned_evaluation_oracle,
            lidar_frontend_topic="/dynanav/lidar_perception_filtered",
            navigation_scope="measured post-VIO repeat-spawn route with online RGB candidate hold/resume; not general avoidance",
            model_timing="synchronous local online inference pauses physics; realtime deadline not certified",
        )
    elif (
        args.person_worker is not None
        or args.pose_launch_receipt is not None
        or args.learned_evaluation_oracle
    ):
        parser.error(
            "Worker/oracle-evaluation options require --semantic-provider learned_rgb_person"
        )
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
        if (
            args.interactive_perception_owner != "fail_closed"
            and args.point_cloud_source != "rtx_lidar"
        ):
            parser.error(
                "native deskewed-cloud perception requires --point-cloud-source rtx_lidar"
            )
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
            interactive_perception_owner=args.interactive_perception_owner,
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
        if not profile.get("pedestrian"):
            profile["notes"] = (
                "Static simulation native route exploration; configured attitude, "
                "base-speed and position envelopes are diagnostic only. "
                "Finite-pose, effort, motor and clock checks remain enforced; "
                "body contact follows body_contact_report_only. "
                "These physics readbacks are not navigation inputs."
            )
    if args.nonrealtime_sensor_lockstep:
        from embodied_navigation.simulation.dynanav.capture.lockstep import validate_profile
        profile["nonrealtime_sensor_lockstep"] = True
        profile["sensor_lockstep_max_wait_s"] = args.sensor_lockstep_max_wait_s
        try:
            profile["sensor_lockstep_contract"] = validate_profile(profile)
        except ValueError as error:
            parser.error(str(error))
    if args.check:
        print(
            json.dumps(
                {
                    "assets_checked": True,
                    "model_hash_checked": True,
                    "simulation_started": False,
                    "controller_config_checked": bool(
                        args.slam_reference or interactive_requested
                    ),
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
        "src/embodied_navigation/common/time/__init__.py",
        "src/embodied_navigation/simulation/dynanav/experiment.py",
        "src/embodied_navigation/simulation/dynanav/capture_go2_rgbd.py",
        "src/embodied_navigation/simulation/dynanav/capture/__init__.py",
        "src/embodied_navigation/simulation/dynanav/capture/control.py",
        "src/embodied_navigation/simulation/dynanav/capture/frames.py",
        "src/embodied_navigation/simulation/dynanav/capture/lidar.py",
        "src/embodied_navigation/simulation/dynanav/capture/lockstep.py",
        "src/embodied_navigation/simulation/dynanav/capture/mapping.py",
        "src/embodied_navigation/simulation/dynanav/capture/perception.py",
        "src/embodied_navigation/simulation/dynanav/capture/physics.py",
        "src/embodied_navigation/simulation/dynanav/capture/policy.py",
        "src/embodied_navigation/simulation/dynanav/capture/receipts.py",
        "src/embodied_navigation/simulation/dynanav/capture/ros_interface.py",
        "src/embodied_navigation/simulation/dynanav/capture/scene.py",
        "src/embodied_navigation/simulation/dynanav/capture/sensors.py",
        "src/embodied_navigation/simulation/dynanav/capture/nearfield.py",
        "src/embodied_navigation/simulation/dynanav/capture/nearfield_sensor.py",
        "src/embodied_navigation/simulation/dynanav/capture/video.py",
        "src/embodied_navigation/simulation/dynanav/oracle_mapping.py",
        "src/embodied_navigation/simulation/dynanav/learned_mapping.py",
        "src/embodied_navigation/simulation/dynanav/learned_scoring.py",
        "src/embodied_navigation/perception/person_worker.py",
        "src/embodied_navigation/perception/person_tiling.py",
        "src/embodied_navigation/perception/person_geometry_tracking.py",
        "src/embodied_navigation/perception/temporal_geometry.py",
        "src/embodied_navigation/perception/learned_person_mapping.py",
        "src/embodied_navigation/perception/person_segmentation.py",
        "scripts/run_dynanav_person_worker.py",
        "scripts/query_scene_map.py",
        "src/embodied_navigation/localization_mapping/semantic_mapping/query.py",
        "src/embodied_navigation/simulation/dynanav/rgb_archive.py",
        "src/embodied_navigation/simulation/dynanav/render_diagnostics.py",
        "src/embodied_navigation/simulation/dynanav/pedestrian.py",
        "src/embodied_navigation/simulation/dynanav/dynamic_hold.py",
        "src/embodied_navigation/simulation/dynanav/session_collision.py",
        "src/embodied_navigation/simulation/dynanav/routes.py",
        "src/embodied_navigation/simulation/dynanav/slam_route.py",
        "src/embodied_navigation/simulation/dynanav/exploration_waypoints.py",
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
        "src/embodied_navigation/simulation/dynanav/depth_safety_fusion.py",
        "src/embodied_navigation/simulation/dynanav/operator_map.py",
        "scripts/explore_waypoint_client.py",
        "scripts/audit_dynanav_nearfield_depth.py",
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
        PROBE_RTF=str(args.rtf),
        NAV_STOP_REQUEST=str(output / "stop_request.json"),
    )
    if profile["duration_s"] is not None:
        os.environ["PROBE_STEPS"] = str(round(profile["duration_s"] / 0.005))
    sys.argv = ["capture_go2_rgbd", str(output), str(scene)]
    runpy.run_module(
        "embodied_navigation.simulation.dynanav.capture_go2_rgbd", run_name="__main__"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
