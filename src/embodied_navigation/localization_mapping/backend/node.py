"""ROS 2 wrapper: spool -> SlamBackend -> map->odom TF, corrected odometry, status.

    python -m embodied_navigation.localization_mapping.backend.node \\
        --spool <dir> --odometry-topic <topic> --namespace <ns> \\
        [--prior-map <dir>] [--save-map <dir>] [--world-frame map] [--odom-frame camera_init]

Every ``--poll-s`` seconds the ready ``kf_*`` directories are ingested in
numeric order and then deleted, so the FAST-LIVO2 exporter never reaches its
``max_ready_directories`` bound. A directory that cannot be decoded is renamed
to ``rejected_kf_*`` (outside that bound) and reported in the status.

Outputs once C = T_world_odom exists: TF ``world_frame -> odom_frame``. Always:
``<ns>/corrected_odometry`` (each input odometry pose left-multiplied by C, or
by identity before the first adoption; twist is body-frame and unchanged) and
``<ns>/status`` (JSON). SIGINT/SIGTERM save the map when ``--save-map`` is set
and exit 0.
"""

from __future__ import annotations

import argparse
import json
import shutil
import signal
import threading
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .backend import BackendConfig, SlamBackend
from .keyframes import read_keyframe, ready_directories
from .map_store import MapStore
from .trace import BackendTrace, graph_snapshot


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D-path point-cloud SLAM backend node")
    parser.add_argument("--spool", type=Path, required=True)
    parser.add_argument("--odometry-topic", required=True)
    parser.add_argument("--namespace", required=True)
    parser.add_argument("--prior-map", type=Path)
    parser.add_argument("--save-map", type=Path)
    parser.add_argument("--world-frame", default="map")
    parser.add_argument("--odom-frame", default="camera_init")
    parser.add_argument("--poll-s", type=float, default=0.2)
    parser.add_argument("--trace-dir", type=Path)
    parser.add_argument("--use-sim-time", action="store_true")
    return parser.parse_args(argv)


def pose_matrix(pose) -> np.ndarray:
    T = np.eye(4)
    q = pose.orientation
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return T


def write_pose(pose, T: np.ndarray) -> None:
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in T[:3, 3])
    x, y, z, w = Rotation.from_matrix(T[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = x, y, z, w


class BackendNode(Node):
    def __init__(self, args: argparse.Namespace, backend: SlamBackend):
        super().__init__("slam_backend", parameter_overrides=[
            Parameter("use_sim_time", value=args.use_sim_time),
        ])
        self.args = args
        self.backend = backend
        self.spool = args.spool.resolve()
        self._poll_lock = threading.Lock()
        self._stopping = threading.Event()
        self._closed = False
        namespace = args.namespace.rstrip("/")
        self.odometry_pub = self.create_publisher(Odometry, f"{namespace}/corrected_odometry", 10)
        self.status_pub = self.create_publisher(String, f"{namespace}/status", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, args.odometry_topic, self.on_odometry, 50, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self._poll_timer = self.create_timer(
            args.poll_s, self.poll, callback_group=MutuallyExclusiveCallbackGroup()
        )

    def on_odometry(self, message: Odometry) -> None:
        C = self.backend.T_world_odom
        write_pose(message.pose.pose, (np.eye(4) if C is None else C) @ pose_matrix(message.pose.pose))
        message.header.frame_id = self.args.world_frame
        self.odometry_pub.publish(message)

    def poll(self) -> None:
        with self._poll_lock:
            # 已排队的回调也必须在锁内检查，退出后不能再写地图或 trace。
            if self._stopping.is_set():
                return
            for directory in ready_directories(self.spool):
                if self._stopping.is_set():
                    return
                self.ingest(directory)
            self.publish_tf()
            self.publish_status()

    def close(self) -> None:
        self._stopping.set()
        self._poll_timer.cancel()
        # 不依赖 executor.shutdown 等待回调；保存前让当前关键帧完整提交。
        with self._poll_lock:
            if self._closed:
                return
            self._closed = True
            try:
                if self.args.save_map is not None:
                    self.backend.save_map(self.args.save_map)
            finally:
                if self.backend.trace is not None:
                    try:
                        self.backend._record("final_graph", graph=graph_snapshot(self.backend.graph))
                    finally:
                        self.backend.trace.close()

    def ingest(self, directory: Path) -> None:
        directory = directory.resolve()
        if directory.parent != self.spool:
            raise RuntimeError(f"refusing to remove {directory} outside {self.spool}")
        try:
            keyframe = read_keyframe(directory)
        except ValueError as error:
            self.backend.last_error = str(error)
            directory.rename(directory.with_name("rejected_" + directory.name))
            return
        event = self.backend.ingest(keyframe)
        if "error" in event:
            self.get_logger().warning(event["error"])
        shutil.rmtree(directory)

    def publish_tf(self) -> None:
        C = self.backend.T_world_odom
        if C is None:
            return
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.args.world_frame
        message.child_frame_id = self.args.odom_frame
        t = message.transform
        t.translation.x, t.translation.y, t.translation.z = (float(v) for v in C[:3, 3])
        x, y, z, w = Rotation.from_matrix(C[:3, :3]).as_quat()
        t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = x, y, z, w
        self.tf.sendTransform(message)

    def publish_status(self) -> None:
        status = {
            "adopted": self.backend.T_world_odom is not None,
            "loops": self.backend.loop_count,
            "keyframes": len(self.backend.map.keyframes),
            "last_error": self.backend.last_error,
            "trace_error": self.backend.trace_error or (self.backend.trace.error if self.backend.trace else None),
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main(argv=None) -> int:
    args = parse_args(argv)
    prior = None if args.prior_map is None else MapStore.load(args.prior_map)
    config = BackendConfig()
    trace = None if args.trace_dir is None else BackendTrace(args.trace_dir, config)
    backend = SlamBackend(config, prior_map=prior, trace=trace)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = BackendNode(args, backend)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        try:
            node.close()
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
