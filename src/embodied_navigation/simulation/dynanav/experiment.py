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


def make_profile(speed_mps=1.0, duration_s=90.0):
    """First physical stair flight; speed is a command ceiling, not achieved speed."""
    if not math.isfinite(speed_mps) or not 0 <= speed_mps <= 1.0:
        raise ValueError("Diagnostic speed must be finite and within 0..1 m/s")
    if not math.isfinite(duration_s) or not 8 <= duration_s <= 300:
        raise ValueError("Diagnostic duration must be finite and within 8..300 s")
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
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--rtf", type=float, default=1.0)
    parser.add_argument(
        "--check",
        action="store_true",
        help="Validate assets/profile without starting Isaac",
    )
    from embodied_navigation.simulation.dynanav.routes import ROUTE_NAMES, get_route

    parser.add_argument("--route", choices=ROUTE_NAMES, default="first-flight")
    parser.add_argument("--render-backend", choices=("fabric", "usd"), default="usd")
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
        "--semantic-provider",
        choices=("isaac_instance_oracle", "learned_rgb_person"),
        default="isaac_instance_oracle",
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
    profile = make_profile(args.speed, args.duration)
    profile["imu_publish_hz"] = args.imu_hz
    profile["imu_publish_stride"] = 200 // args.imu_hz
    profile["imu_orientation_available"] = False
    profile["render_backend"] = args.render_backend
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
    if args.check:
        print(
            json.dumps(
                {
                    "assets_checked": True,
                    "model_hash_checked": True,
                    "simulation_started": False,
                    "controller_config_checked": bool(args.slam_reference),
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
        "src/embodied_navigation/simulation/dynanav/capture/mapping.py",
        "src/embodied_navigation/simulation/dynanav/capture/physics.py",
        "src/embodied_navigation/simulation/dynanav/capture/policy.py",
        "src/embodied_navigation/simulation/dynanav/capture/receipts.py",
        "src/embodied_navigation/simulation/dynanav/capture/ros_interface.py",
        "src/embodied_navigation/simulation/dynanav/capture/scene.py",
        "src/embodied_navigation/simulation/dynanav/capture/sensors.py",
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
