"""ROS 2 wrapper: spool -> SlamBackend -> map->odom TF, corrected odometry, status.

    python -m embodied_navigation.localization_mapping.backend.node \\
        --spool <dir> --odometry-topic <topic> --namespace <ns> \\
        [--prior-map <dir>] [--save-map <dir>] [--config <json>] \\
        [--world-frame map] [--odom-frame camera_init]

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
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .backend import BackendConfig, SlamBackend
from .keyframes import read_keyframe, ready_directories
from .map_store import MapStore


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D-path point-cloud SLAM backend node")
    parser.add_argument("--spool", type=Path, required=True)
    parser.add_argument("--odometry-topic", required=True)
    parser.add_argument("--namespace", required=True)
    parser.add_argument("--prior-map", type=Path)
    parser.add_argument("--save-map", type=Path)
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--status-file", type=Path, default=None)
    parser.add_argument("--world-frame", default="map")
    parser.add_argument("--odom-frame", default="camera_init")
    parser.add_argument("--poll-s", type=float, default=0.2)
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
        super().__init__("slam_backend")
        self.args = args
        self.backend = backend
        self.spool = args.spool.resolve()
        self._last_odom_header = None  # (frame_id, stamp) from incoming odometry
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
        cfg = backend.config
        config_label = "defaults" if args.config is None else str(args.config)
        self.get_logger().info(
            f"SLAM backend initialized: config={config_label} "
            f"zupt={cfg.zupt_enabled} icp_gravity_sigma_rad={cfg.icp_gravity_sigma_rad} "
            f"graph_gravity_sigma_rad={cfg.graph_gravity_sigma_rad} "
            f"min_loop_travel_m={cfg.min_loop_travel_m} min_loop_spacing_m={cfg.min_loop_spacing_m} "
            f"loop_max_jump_translation_m={cfg.loop_max_jump_translation_m}"
        )

    def on_odometry(self, message: Odometry) -> None:
        in_frame = message.header.frame_id or self.args.odom_frame
        self._last_odom_header = (in_frame, message.header.stamp)
        C = self.backend.T_world_odom
        write_pose(message.pose.pose, (np.eye(4) if C is None else C) @ pose_matrix(message.pose.pose))
        message.header.frame_id = self.args.world_frame
        self.odometry_pub.publish(message)

    def poll(self) -> None:
        if self.spool.is_dir():
            for directory in ready_directories(self.spool):
                self.ingest(directory)
        self.publish_tf()
        self.publish_status()

    def _quarantine(self, directory: Path) -> None:
        target = directory.with_name("rejected_" + directory.name)
        if not target.exists():
            directory.rename(target)

    def ingest(self, directory: Path) -> None:
        directory = directory.resolve()
        if directory.parent != self.spool:
            raise RuntimeError(f"refusing to remove {directory} outside {self.spool}")
        try:
            cfg = self.backend.config
            load_visual = cfg.use_visual_retrieval or cfg.use_visual_verification or cfg.color_sc_enabled
            keyframe = read_keyframe(directory, load_visual=load_visual)
        except ValueError as error:
            self.backend.last_error = str(error)
            self._quarantine(directory)
            return
        event = self.backend.ingest(keyframe)
        if "error" in event:
            self.get_logger().warning(event["error"])
            self._quarantine(directory)
            return
        shutil.rmtree(directory)

    def publish_tf(self) -> None:
        C = self.backend.T_world_odom
        if C is None:
            return
        odom_header = self._last_odom_header
        message = TransformStamped()
        message.header.stamp = odom_header[1] if odom_header is not None else self.get_clock().now().to_msg()
        message.header.frame_id = self.args.world_frame
        message.child_frame_id = odom_header[0] if odom_header is not None else self.args.odom_frame
        t = message.transform
        t.translation.x, t.translation.y, t.translation.z = (float(v) for v in C[:3, 3])
        x, y, z, w = Rotation.from_matrix(C[:3, :3]).as_quat()
        t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = x, y, z, w
        self.tf.sendTransform(message)

    def publish_status(self) -> None:
        C = self.backend.T_world_odom
        status = {
            "adopted": C is not None,
            "loops": self.backend.loop_count,
            "keyframes": len(self.backend.map.keyframes),
            "last_error": self.backend.last_error,
            "config_path": None if self.args.config is None else str(self.args.config),
            "world_frame": self.args.world_frame,
            "odom_frame": self._last_odom_header[0] if self._last_odom_header is not None else self.args.odom_frame,
            "T_world_odom": None if C is None else C.tolist(),
            "stats": self.backend.observability_stats(),
        }
        payload = json.dumps(status, allow_nan=False)
        self.status_pub.publish(String(data=payload))
        status_file = getattr(self.args, "status_file", None)
        if status_file is not None:
            path = Path(status_file)
            tmp = path.with_suffix(path.suffix + ".tmp")
            tmp.write_text(payload + "\n", encoding="utf-8")
            tmp.replace(path)


def main(argv=None) -> int:
    args = parse_args(argv)
    prior = None if args.prior_map is None else MapStore.load(args.prior_map)
    config = BackendConfig() if args.config is None else BackendConfig.from_json(args.config)
    backend = SlamBackend(config, prior_map=prior)
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
        timer = getattr(node, "_poll_timer", None)
        if timer is not None:
            if hasattr(timer, "cancel"):
                timer.cancel()
            if hasattr(node, "destroy_timer"):
                node.destroy_timer(timer)
        executor.shutdown()
        # Drain any final ready keyframes flushed by the frontend only after
        # the executor has stopped so no poll callback can race on the spool.
        if node.spool.is_dir():
            for directory in ready_directories(node.spool):
                node.ingest(directory)
        node.destroy_node()
        rclpy.shutdown()
        if args.save_map is not None:
            backend.save_map(args.save_map)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
