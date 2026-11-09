"""ROS interface for the capture probe: node, topics, callbacks, publishers.

Owning module for ``rclpy``: it initialises the context/node, creates the
unchanged ``/dynanav/*`` publishers and the odometry/approved-command
subscribers, defines the two callbacks, and turns sensor-loop data into ROS
messages. Topic names, message types and QoS (``qos_profile_sensor_data``) are
identical to the original probe. Everything is created by
``create_ros_interface`` at run time, never at import time.
"""

import os
import time
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np


@dataclass
class RosInterface:
    """Handles to the rclpy node, publishers, callbacks and latest samples."""

    node: Any
    header: Callable
    latest_odom: list
    latest_nav: list
    published: dict
    spin_once: Callable
    drain: Callable
    publish_imu_and_clock: Callable
    publish_cloud: Callable
    publish_depth_and_info: Callable
    publish_rgb: Callable
    destroy: Callable



def _publish_sensor(publisher, message, topic, journal=None, *, point_count=None):
    """Trace the local publish boundary, never DDS delivery or acknowledgement.

    Only scalar metadata is journalled: two rows per selected sensor frame.
    Journal and publisher failures deliberately propagate to capture cleanup.
    """
    if journal is None:
        publisher.publish(message)
        return
    metadata = {
        "topic": topic,
        "source_ns": message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
        "data_bytes": len(message.data),
        "point_count": point_count,
        "delivery_confirmed": False,
    }
    before = time.monotonic_ns()
    journal("sensor_publish_trace.jsonl", {
        **metadata, "event": "publish_call", "monotonic_ns": before,
    })
    publisher.publish(message)
    after = time.monotonic_ns()
    journal("sensor_publish_trace.jsonl", {
        **metadata, "event": "publish_returned", "monotonic_ns": after,
        "before_publish_monotonic_ns": before,
    })

def create_ros_interface(experiment, learned_mapper, learned_scorer, *, journal=None):
    """Create the capture ROS node exactly as the original probe did."""
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField

    trace_enabled = os.environ.get("DYNANAV_PUBLISH_TRACE") == "1"
    if trace_enabled and journal is None:
        raise ValueError("DYNANAV_PUBLISH_TRACE=1 requires a journal callback")
    trace_journal = journal if trace_enabled else None
    frontend_topic = experiment.get(
        "lidar_frontend_topic",
        "/dynanav/lidar_perception_filtered" if learned_mapper is not None
        else "/dynanav/lidar_static",
    )

    def publish_cloud_message(publisher, message, topic):
        _publish_sensor(
            publisher, message, topic,
            trace_journal if topic == frontend_topic else None,
            point_count=message.width * message.height,
        )

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("actual_go2_rgbd_source")
    pub_imu = node.create_publisher(Imu, "/dynanav/imu", qos_profile_sensor_data)
    pub_img = node.create_publisher(Image, "/dynanav/camera", qos_profile_sensor_data)
    pub_cloud = node.create_publisher(
        PointCloud2, "/dynanav/lidar_filtered", qos_profile_sensor_data
    )
    pub_static_cloud = node.create_publisher(
        PointCloud2, "/dynanav/lidar_static", qos_profile_sensor_data
    )
    pub_perception_cloud = (
        node.create_publisher(
            PointCloud2, "/dynanav/lidar_perception_filtered", qos_profile_sensor_data
        )
        if learned_mapper is not None
        else None
    )
    pub_depth = node.create_publisher(Image, "/dynanav/depth", qos_profile_sensor_data)
    pub_info = node.create_publisher(
        CameraInfo, "/dynanav/camera_info", qos_profile_sensor_data
    )
    latest_odom = [None]

    def odom_callback(m):
        latest_odom[0] = {
            "source_ns": m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec,
            "time": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            "position": [
                m.pose.pose.position.x,
                m.pose.pose.position.y,
                m.pose.pose.position.z,
            ],
            "orientation_wxyz": [
                m.pose.pose.orientation.w,
                m.pose.pose.orientation.x,
                m.pose.pose.orientation.y,
                m.pose.pose.orientation.z,
            ],
            "estimated_velocity": None
            if experiment.get("slam_pose_source") == "livo_post_vio"
            else [
                m.twist.twist.linear.x,
                m.twist.twist.linear.y,
                m.twist.twist.linear.z,
            ],
            "pose_source": experiment.get("slam_pose_source", "lio_estimate"),
            "frame_id": m.header.frame_id,
            "child_frame_id": m.child_frame_id,
            "arrival_wall": time.perf_counter(),
        }

        if learned_mapper is not None:
            learned_mapper.offer_pose(latest_odom[0])
            for completion in learned_mapper.take_completed():
                if learned_scorer is not None:
                    learned_scorer.observe_completion(completion)

    odom_sub = node.create_subscription(
        Odometry,
        experiment.get("odometry_topic", "/aft_mapped_to_init"),
        odom_callback,
        qos_profile_sensor_data,
    )
    latest_nav = [None]
    pub_clock = node.create_publisher(Clock, "/clock", 10)

    def nav_callback(m):
        latest_nav[0] = {
            "time": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            "request": [m.twist.linear.x, m.twist.linear.y, m.twist.angular.z],
            "arrival_wall": time.perf_counter(),
        }

    nav_sub = node.create_subscription(
        TwistStamped, "/dynanav/nav/approved_cmd", nav_callback, 10
    )

    def header(m, stamp, frame):
        ns = round(stamp * 1e9)
        m.header.stamp.sec = ns // 1000000000
        m.header.stamp.nanosec = ns % 1000000000
        m.header.frame_id = frame
    published = {"imu": 0, "rgb": 0, "cloud": 0}

    def spin_once(timeout_sec=0.0):
        rclpy.spin_once(node, timeout_sec=timeout_sec)

    def drain(times):
        for _ in range(times):
            rclpy.spin_once(node, timeout_sec=0.0)

    def publish_imu_and_clock(it, reading, orientation_available):
        clock = Clock()
        ns = round(it * 1e9)
        clock.clock.sec = ns // 1000000000
        clock.clock.nanosec = ns % 1000000000
        pub_clock.publish(clock)
        msg = Imu()
        header(msg, it, "go2_imu")
        aa = reading["linear_acceleration"]
        gg = reading["angular_velocity"]
        qq = reading["orientation"]
        msg.linear_acceleration.x = float(aa[0])
        msg.linear_acceleration.y = float(aa[1])
        msg.linear_acceleration.z = float(aa[2])
        msg.angular_velocity.x = float(gg[0])
        msg.angular_velocity.y = float(gg[1])
        msg.angular_velocity.z = float(gg[2])
        if orientation_available:
            msg.orientation.w = float(qq[0])
            msg.orientation.x = float(qq[1])
            msg.orientation.y = float(qq[2])
            msg.orientation.z = float(qq[3])
        else:
            msg.orientation_covariance[0] = -1.0
        pub_imu.publish(msg)
        published["imu"] += 1


    def publish_cloud(it, xyz, states, frontend_keep, oracle_mapper, learned_mapper):
        xyzi = np.zeros((len(xyz), 4), dtype="<f4")
        xyzi[:, :3] = xyz
        msg = PointCloud2()
        header(msg, it, "go2_imu")
        msg.height = 1
        msg.width = len(xyzi)
        msg.fields = [
            PointField(name=n, offset=j * 4, datatype=7, count=1)
            for j, n in enumerate(["x", "y", "z", "intensity"])
        ]
        msg.point_step = 16
        msg.row_step = 16 * msg.width
        msg.is_dense = True
        msg.data = xyzi.tobytes()
        publish_cloud_message(pub_cloud, msg, "/dynanav/lidar_filtered")
        published["cloud"] += 1
        if oracle_mapper is not None:
            from embodied_navigation.localization_mapping.semantic_mapping import (
                STATIC,
            )

            static_xyzi = xyzi[states == STATIC]
            static_msg = PointCloud2()
            header(static_msg, it, "go2_imu")
            static_msg.height, static_msg.width = 1, len(static_xyzi)
            static_msg.fields = msg.fields
            static_msg.point_step, static_msg.row_step = 16, 16 * len(static_xyzi)
            static_msg.is_dense = True
            static_msg.data = static_xyzi.tobytes()
            publish_cloud_message(pub_static_cloud, static_msg, "/dynanav/lidar_static")
        if learned_mapper is not None:
            filtered_xyzi = xyzi[frontend_keep]
            filtered_msg = PointCloud2()
            header(filtered_msg, it, "go2_imu")
            filtered_msg.height, filtered_msg.width = 1, len(filtered_xyzi)
            filtered_msg.fields = msg.fields
            filtered_msg.point_step, filtered_msg.row_step = (
                16,
                16 * len(filtered_xyzi),
            )
            filtered_msg.is_dense = True
            filtered_msg.data = filtered_xyzi.tobytes()
            publish_cloud_message(pub_perception_cloud, filtered_msg, "/dynanav/lidar_perception_filtered")


    def publish_depth_and_info(it, zz):
        depth_msg = Image()
        header(depth_msg, it, "go2_camera_optical")
        depth_msg.height, depth_msg.width = zz.shape
        depth_msg.encoding, depth_msg.step = "32FC1", zz.shape[1] * 4
        depth_msg.data = np.ascontiguousarray(zz, dtype="<f4").tobytes()
        pub_depth.publish(depth_msg)
        info = CameraInfo()
        header(info, it, "go2_camera_optical")
        info.height, info.width = zz.shape
        focal = 512 * 24 / 42
        info.k = [focal, 0.0, 256.0, 0.0, focal, 256.0, 0.0, 0.0, 1.0]
        info.p = [
            focal,
            0.0,
            256.0,
            0.0,
            0.0,
            focal,
            256.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        info.distortion_model, info.d = "plumb_bob", [0.0] * 5
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        pub_info.publish(info)


    def publish_rgb(it, rgb):
        msg = Image()
        header(msg, it, "go2_camera_optical")
        msg.height = 512
        msg.width = 512
        msg.encoding = "rgb8"
        msg.step = 1536
        msg.data = np.ascontiguousarray(rgb).tobytes()
        _publish_sensor(pub_img, msg, "/dynanav/camera", trace_journal)
        published["rgb"] += 1

    def destroy():
        node.destroy_node()
        rclpy.shutdown()

    return RosInterface(
        node=node,
        header=header,
        latest_odom=latest_odom,
        latest_nav=latest_nav,
        published=published,
        spin_once=spin_once,
        drain=drain,
        publish_imu_and_clock=publish_imu_and_clock,
        publish_cloud=publish_cloud,
        publish_depth_and_info=publish_depth_and_info,
        publish_rgb=publish_rgb,
        destroy=destroy,
    )
