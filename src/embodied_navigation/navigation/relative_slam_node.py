"""ROS adapter for the extracted relative-SLAM navigation task.

Requires matching cloud/odometry stamps and measured map support. Importing or
running --help never starts a robot. Only explicit main() creates the ROS node.
"""

import argparse, json, time, signal, os
from pathlib import Path
import numpy as np
from scipy import ndimage
from .task import NavigationTask, TaskConfig
from ..control.collision import certified_support
from ..localization_mapping.relative_odometry import RelativeOdometry


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-grid", type=Path, required=True)
    parser.add_argument("--spawn-prior", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--route", type=Path, required=True)
    parser.add_argument(
        "--motion-end", type=float, default=float(os.environ.get("NAV_END_SIM", "270"))
    )
    args = parser.parse_args(argv)
    import yaml, rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from geometry_msgs.msg import TwistStamped

    route = yaml.safe_load(args.route.read_text())["waypoints_xy"]
    grid = np.load(args.map_grid)
    origin = grid["lo"]
    resolution = float(grid["res"])
    radius = int(np.ceil(0.65 / resolution))
    gx, gy = np.ogrid[-radius : radius + 1, -radius : radius + 1]
    disk = (gx * gx + gy * gy) <= (0.65 / resolution) ** 2
    # Free cells are support after 0.65 m disk erosion, excluding obstacles
    # after dilation by that same disk.
    free = ndimage.binary_erosion(grid["support"], structure=disk) & ~ndimage.binary_dilation(
        grid["obstacle"], structure=disk
    )
    task = NavigationTask(
        route,
        lambda points: certified_support(points, origin, resolution, free),
        TaskConfig(motion_end_s=args.motion_end),
    )
    prior = json.loads(args.spawn_prior.read_text())
    # Pass grid floor + 0.32 m as floor_z for the obstacle-height filter.
    relative = RelativeOdometry(
        route[0], prior["initial_path_heading_map"], float(grid["floor"]) + 0.32
    )
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node("go2_relative_slam_navigation")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    pub = node.create_publisher(TwistStamped, "/dynanav/nav/approved_cmd", 10)
    raw = {}
    odoms = {}
    processed = set()
    commands = []
    poses = []
    last_save = 0.0

    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9

    def save():
        payload = {
            "mode": "relative_SLAM_multistage_PID",
            "route": route,
            "stage_records": task.stages,
            "events": task.events,
            "commands": commands,
            "poses": poses,
            "arrived": task.arrived,
            "failure": task.failure,
            "relative_SLAM_cumulative_distance_m": task.relative_distance,
            "truth_used_by_navigation": False,
            "map_registration_enabled": False,
        }
        (args.output / "result.json").write_text(json.dumps(payload, indent=2))
        if task.arrived or task.failure:
            (args.output / "stop_request.json").write_text(
                json.dumps({"arrived": task.arrived, "failure": task.failure, "required_park_s": 5})
            )

    def pair(key):
        if key in processed or key not in raw or key not in odoms:
            return
        processed.add(key)
        message = odoms[key]
        t = stamp(message)
        # Frozen startup settle guard: pairs earlier than 5.8 s are not the
        # measured body pose used for navigation.
        if t < 5.8:
            return
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        observation = relative.update(
            t, time.perf_counter(), [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], raw[key]
        )
        task.update(observation)
        poses.append(
            {
                "time": t,
                "body": observation.body.tolist(),
                "R": observation.rotation.tolist(),
                "velocity": observation.velocity.tolist(),
                "stage": task.stage,
                "phase": task.phase,
            }
        )
        for cache in (raw, odoms):
            # After a usable pair, discard cached stamps more than 2 s older than it.
            for old in list(cache):
                if old < key - 2000000000:
                    cache.pop(old, None)

    def cloud(message):
        key = round(stamp(message) * 1e9)
        raw[key] = point_cloud2.read_points_numpy(
            message, field_names=["x", "y", "z"], skip_nans=True
        )
        pair(key)

    def odom(message):
        key = round(stamp(message) * 1e9)
        odoms[key] = message
        pair(key)

    def tick():
        nonlocal last_save
        now = node.get_clock().now().nanoseconds * 1e-9
        command, record = task.step(now, time.perf_counter())
        message = TwistStamped()
        ns = round((task.observation.time_s if task.observation is not None else now) * 1e9)
        message.header.stamp.sec = ns // 1000000000
        message.header.stamp.nanosec = ns % 1000000000
        message.header.frame_id = "go2_base"
        message.twist.linear.x, message.twist.linear.y, message.twist.angular.z = map(
            float, command
        )
        pub.publish(message)
        # Record at most one command row per 0.05 s tick and save the result file
        # at most once per second; both are evidence, not control inputs.
        if not commands or now - commands[-1]["time"] > 0.05:
            commands.append(record)
        if time.perf_counter() - last_save > 1:
            last_save = time.perf_counter()
            save()

    node.create_subscription(PointCloud2, "/dynanav/lidar_filtered", cloud, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/aft_mapped_to_init", odom, qos_profile_sensor_data)
    node.create_timer(
        0.02, tick, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME)
    )
    signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
