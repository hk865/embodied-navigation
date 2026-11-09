#!/usr/bin/env python3
"""Adapt FAST-LIVO2 IMU states to explicit body/lidar poses and valid twists."""

from embodied_navigation.project import project_root as _project_root
import argparse
import sys
import os
import uuid
import base64
import hashlib
from collections import OrderedDict
from embodied_navigation.localization_mapping.point_provenance import message_sha, decode, MAX_META
import json
import time
from collections import Counter
from pathlib import Path

import numpy as np
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation

from embodied_navigation.localization_mapping.geometry import (
    transform_state,
    body_twist,
    shifted_pose_covariance,
)


class OdomAdapter(Node):
    def __init__(self, scenario):
        super().__init__("demo_slam_odom_adapter")
        self.sensors = json.loads(Path(scenario).read_text())["sensors"]
        self.body = {"body_xyz": [0, 0, 0], "body_rpy": [0, 0, 0]}
        self.odom_pubs = {
            name: self.create_publisher(Odometry, f"/demo/slam/{name}_odom", 10)
            for name in ("body", "lidar")
        }
        self.status_pub = self.create_publisher(String, "/demo/slam/status", 10)
        self.tf = TransformBroadcaster(self)
        self.identity_pending = OrderedDict()
        self.identity_native = OrderedDict()
        self.identity_pub = self.create_publisher(String, "/demo/slam/odom_identity", 10)
        self.create_subscription(String, "/demo/slam/point_identity", self.on_point_identity, 10)
        self.previous = None
        self.last_wall = None
        self.last_stamp = None
        self.counts = Counter()
        self.last_error = None
        self.twist_valid = False
        self.create_subscription(
            Odometry, "/aft_mapped_to_init", self.receive, qos_profile_sensor_data
        )
        self.create_timer(0.5, self.status)

    def on_point_identity(self, msg):
        d = decode(msg.data, "post_lio_source_point_commit/v1")
        ns = d["stamp_ns"]
        if type(ns) is not int or ns <= 0:
            raise ValueError("R60 mapper commit integer stamp")
        # Full point refs are not needed by this adapter; keep only exact parent.
        self.identity_native[ns] = {
            k: d[k]
            for k in (
                "producer_epoch",
                "producer_pid",
                "commit",
                "stamp_ns",
                "mapper_message_sha256",
                "run_id",
                "fingerprint_model",
            )
        }
        while len(self.identity_native) > 12:
            self.identity_native.popitem(last=False)
        self.try_identity(ns)

    def try_identity(self, ns):
        if ns not in self.identity_pending or ns not in self.identity_native:
            return
        native = self.identity_native[ns]
        actual = self.identity_pending[ns]
        if (
            native["mapper_message_sha256"] != actual["mapper_message_sha256"]
            or native["run_id"] != Path(os.environ["DEMO_RUN_DIR"]).name
        ):
            raise ValueError("R60 adapter actual mapper/commit differs")
        proof = {
            **native,
            **actual,
            "schema": "mapper_body_lidar_commit/v1",
            "adapter_pid": os.getpid(),
            "navigation_ground_truth_used": False,
        }
        self.identity_pub.publish(String(data=json.dumps(proof, allow_nan=False)))
        del self.identity_pending[ns]
        del self.identity_native[ns]

    def receive(self, raw):
        stamp = raw.header.stamp.sec + raw.header.stamp.nanosec * 1e-9
        if raw.header.frame_id != "camera_init":
            self.counts["rejected_frame"] += 1
            self.last_error = f"Unexpected raw frame {raw.header.frame_id}"
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.counts["rejected_timestamp"] += 1
            self.last_error = "SLAM timestamp did not increase; restart a new run after clock reset"
            return
        p, q = raw.pose.pose.position, raw.pose.pose.orientation
        pose = [p.x, p.y, p.z]
        quat = [q.x, q.y, q.z, q.w]
        states = {}
        try:
            for name, target in [("body", self.body), ("lidar", self.sensors["lidar"])]:
                states[name] = transform_state(pose, quat, self.sensors["imu"], target)
        except ValueError as exc:
            self.counts["rejected_pose"] += 1
            self.last_error = str(exc)
            return
        dt = stamp - self.last_stamp if self.last_stamp is not None else None
        velocity_valid = self.previous is not None and 0 < dt <= 1
        identity_outputs = {}
        for name, (position, rotation) in states.items():
            out = Odometry()
            out.header = raw.header
            out.child_frame_id = "demo_slam_body" if name == "body" else "demo_slam_lidar"
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(
                float, position
            )
            orientation = Rotation.from_matrix(rotation).as_quat()
            (
                out.pose.pose.orientation.x,
                out.pose.pose.orientation.y,
                out.pose.pose.orientation.z,
                out.pose.pose.orientation.w,
            ) = map(float, orientation)
            # The legacy message leaves covariance unset. Preserve nonzero
            # covariance; otherwise report conservative nonzero uncertainty.
            if any(raw.pose.covariance):
                out.pose.covariance = shifted_pose_covariance(
                    raw.pose.covariance, position - np.asarray(pose)
                )
            else:
                for i in range(6):
                    out.pose.covariance[7 * i] = 0.01 if i < 3 else 0.0025
            if velocity_valid:
                linear, angular = body_twist(self.previous[name], states[name], dt)
                out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(
                    float, linear
                )
                out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = (
                    map(float, angular)
                )
            for i in range(6):
                out.twist.covariance[7 * i] = 0.04 if velocity_valid else 1e6
            self.odom_pubs[name].publish(out)
            identity_outputs[name + "_message_sha256"] = message_sha(out)
            # Adapter-specific frames avoid conflicting with CHAMP odom->base
            # and its URDF sensor tree. Consumers use explicit SLAM odometry.
            transform = TransformStamped()
            transform.header = out.header
            transform.child_frame_id = out.child_frame_id
            transform.transform.translation.x = out.pose.pose.position.x
            transform.transform.translation.y = out.pose.pose.position.y
            transform.transform.translation.z = out.pose.pose.position.z
            transform.transform.rotation = out.pose.pose.orientation
            self.tf.sendTransform(transform)
        ns = int(raw.header.stamp.sec) * 1_000_000_000 + int(raw.header.stamp.nanosec)
        self.identity_pending[ns] = dict(mapper_message_sha256=message_sha(raw), **identity_outputs)
        while len(self.identity_pending) > 12:
            self.identity_pending.popitem(last=False)
        self.try_identity(ns)
        self.previous = states
        self.twist_valid = velocity_valid
        self.last_stamp = stamp
        self.last_wall = time.monotonic()
        self.counts["accepted"] += 1
        if velocity_valid:
            self.counts["valid_twist"] += 1
        self.last_error = None

    def status(self):
        age = None if self.last_wall is None else time.monotonic() - self.last_wall
        self.status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "source": "/aft_mapped_to_init",
                        "frame_id": "camera_init",
                        "state_reference": "IMU",
                        "body_reference": "body center / trunk origin",
                        "twist_reference": "current child frame, SLAM pose finite difference",
                        "stamp": self.last_stamp,
                        "message_age": age,
                        "twist_valid": self.twist_valid,
                        "healthy": age is not None and age < 2.0 and self.last_error is None,
                        "counts": dict(self.counts),
                        "error": self.last_error,
                        "ground_truth_used": False,
                    },
                    allow_nan=False,
                )
            )
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario", default=str(_project_root() / "configs/simulation/scenario.json")
    )
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = OdomAdapter(args.scenario)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except _ros.RCLError:
        # SIGINT can invalidate the context between executor wait-set calls.
        # Errors while the context is live must still fail this required node.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
