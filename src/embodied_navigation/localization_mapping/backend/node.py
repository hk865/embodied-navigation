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
from collections import deque
from dataclasses import replace
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformBroadcaster

from .backend import BackendConfig, SlamBackend
from .keyframes import read_keyframe, ready_directories
from .map_store import MapStore
from .trace import BackendTrace, graph_snapshot
from .visual import VisualObservation, decode_image
from .submaps import SubmapConfig
from .surfaces import SurfaceConfig
from .visual_retrieval import VisualRetrievalConfig
from .scan_context import ScanContextConfig


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
    parser.add_argument("--evidence-config", type=Path)
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
        self.visual_input = getattr(args, 'visual_input', None)
        self.visual_images = deque(maxlen=32)
        self.visual_lock = threading.Lock()
        self.visual_calibration = None
        self.visual_input_error = None
        visual_group = MutuallyExclusiveCallbackGroup()
        if self.visual_input:
            if self.visual_input.get('rectified') is not True:
                raise ValueError('visual input must explicitly declare rectified=true')
            # Explicit optical-frame extrinsic and rectified image topics are
            # required; no assumed mounting orientation or guessed calibration.
            self.visual_extrinsic = np.asarray(self.visual_input['T_body_camera'], dtype=float)
            VisualObservation(np.zeros((1, 1)), np.eye(3), self.visual_extrinsic, 0)
            group = visual_group
            self.create_subscription(Image, self.visual_input['image_topic'], self.on_image, qos_profile_sensor_data, callback_group=group)
            self.create_subscription(CameraInfo, self.visual_input['camera_info_topic'], self.on_camera_info, qos_profile_sensor_data, callback_group=group)
        namespace = args.namespace.rstrip("/")
        self.odometry_pub = self.create_publisher(Odometry, f"{namespace}/corrected_odometry", 10)
        self.status_pub = self.create_publisher(String, f"{namespace}/status", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, args.odometry_topic, self.on_odometry, 50, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.create_timer(args.poll_s, self.poll, callback_group=MutuallyExclusiveCallbackGroup())

    def on_image(self, message):
        try:
            gray = decode_image(message)
            stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
            with self.visual_lock:
                self.visual_images.append((stamp, message.header.frame_id, gray))
        except ValueError as error:
            self.visual_input_error = str(error)

    def on_camera_info(self, message):
        # P is the rectified camera projection matrix; raw distortion is not
        # silently ignored. Image and calibration optical frame must agree.
        projection = np.asarray(message.p, dtype=float).reshape(3, 4)
        if np.any(projection[:, 3] != 0):
            self.visual_input_error = 'stereo projection offsets unsupported; provide monocular rectified optical frame'
            with self.visual_lock:
                self.visual_calibration = None
            return
        with self.visual_lock:
            self.visual_calibration = (message.header.frame_id, message.height, message.width, projection[:, :3].copy())

    def attach_visual(self, keyframe):
        with self.visual_lock:
            calibration = self.visual_calibration
            images = list(self.visual_images)
        if keyframe.visual is not None or calibration is None or not images:
            return keyframe
        stamp, frame, gray = min(images, key=lambda image: abs(image[0]-keyframe.stamp_ns))
        calibration_frame, height, width, K = calibration
        if frame != calibration_frame or gray.shape != (height, width):
            self.visual_input_error = 'visual image/calibration frame or dimensions mismatch'
            return keyframe
        if abs(stamp-keyframe.stamp_ns) > self.visual_input.get('max_time_delta_s', .05)*1e9:
            self.visual_input_error = 'visual image/keyframe time mismatch'
            return keyframe
        try:
            observation = VisualObservation(gray, K, self.visual_extrinsic, stamp)
        except ValueError as error:
            self.visual_input_error = str(error)
            return keyframe
        self.visual_input_error = None
        return replace(keyframe, visual=observation)

    def on_odometry(self, message: Odometry) -> None:
        C = self.backend.T_world_odom
        write_pose(message.pose.pose, (np.eye(4) if C is None else C) @ pose_matrix(message.pose.pose))
        message.header.frame_id = self.args.world_frame
        self.odometry_pub.publish(message)

    def poll(self) -> None:
        for directory in ready_directories(self.spool):
            self.ingest(directory)
        self.publish_tf()
        self.publish_status()

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
        if self.visual_input:
            keyframe = self.attach_visual(keyframe)
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
            "visual_input_error": self.visual_input_error,
            "trace_error": self.backend.trace_error or (self.backend.trace.error if self.backend.trace else None),
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main(argv=None) -> int:
    args = parse_args(argv)
    prior = None if args.prior_map is None else MapStore.load(args.prior_map)
    evidence = {} if args.evidence_config is None else json.loads(args.evidence_config.read_text())
    row = dict(evidence.get('backend', {}))
    for name, constructor in [('measured_submap', SubmapConfig), ('surface', SurfaceConfig), ('scan_context', ScanContextConfig), ('visual_retrieval', VisualRetrievalConfig)]:
        if name in row:
            row[name] = constructor(**row[name])
    config = BackendConfig(**row)
    args.visual_input = evidence.get('visual_input')
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
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        if args.save_map is not None:
            backend.save_map(args.save_map)
        if trace is not None:
            backend._record("final_graph", graph=graph_snapshot(backend.graph))
            trace.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
