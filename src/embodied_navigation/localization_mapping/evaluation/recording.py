"""只读采集LIO、post-VIO与在线后端完整姿态；不发布控制或真值。"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import signal
import time

import numpy as np
from scipy.spatial.transform import Rotation

from embodied_navigation.common.runtime_io import EvidenceWriter

DEFAULT_TOPICS = {
    "frontend_lio": "/dynanav/rtx_livo/aft_mapped_to_init",
    "frontend_post_vio": "/dynanav/rtx_livo/post_vio_odometry",
    "backend_online": "/dynanav/backend/corrected_odometry",
}


def odometry_record(message):
    """原时间戳/四元数保留，RPY只作为完整姿态的可读派生值。"""
    pose = message.pose.pose
    xyz = [pose.position.x, pose.position.y, pose.position.z]
    q = pose.orientation
    xyzw = [q.x, q.y, q.z, q.w]
    rotation = Rotation.from_quat(xyzw)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rotation.as_matrix(), xyz
    stamp = message.header.stamp
    return {"stamp_ns": stamp.sec * 1_000_000_000 + stamp.nanosec,
            "frame_id": message.header.frame_id, "child_frame_id": message.child_frame_id,
            "position_xyz_m": xyz, "quaternion_xyzw": xyzw,
            "rpy_xyz_rad": rotation.as_euler("xyz").tolist(),
            "rpy_convention": "scipy extrinsic xyz; radians; quaternion is authoritative",
            "T_parent_child": T.tolist(),
            "pose_covariance": list(message.pose.covariance),
            "twist_covariance": list(message.twist.covariance),
            "covariance_note": "raw published values; no claim that backend covariance was transformed"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--configuration", type=Path, required=True)
    args = parser.parse_args(argv)
    config = json.loads(args.configuration.read_text())
    # ROS仅在运行入口导入，纯姿态转换可以独立验收。
    import rclpy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    args.output_dir.mkdir(parents=True, exist_ok=False)
    writer = EvidenceWriter()
    counts = Counter()
    latest = {}
    errors = []
    stopping = False
    def request_stop(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("slam_pose_recorder")
    subscriptions = []

    def save_record(name, row):
        counts[name] += 1
        if "stamp_ns" in row:
            latest[name] = row["stamp_ns"]
        try:
            writer.append(args.output_dir / f"{name}.jsonl", row)
        except Exception as error:
            # 只记录采集失效，不改变定位/规划/控制。
            reason = f"{type(error).__name__}: {error}"
            if not errors or errors[-1] != reason:
                errors.append(reason)
                node.get_logger().error(reason)

    def receive(name, topic, message):
        try:
            row = odometry_record(message)
        except ValueError as error:
            errors.append(f"{name}: {error}")
            return
        row.update({"topic": topic, "received_monotonic_ns": time.monotonic_ns(),
                    "received_unix_ns": time.time_ns()})
        save_record(name, row)

    for name, topic in config["topics"].items():
        subscriptions.append(node.create_subscription(
            Odometry, topic, lambda message, n=name, t=topic: receive(n, t, message), 100))
    subscriptions.append(node.create_subscription(
        String, config["status_topic"],
        lambda message: save_record("backend_status", {
            "received_monotonic_ns": time.monotonic_ns(), "payload": message.data}), 100))

    def health(final=False):
        document = {"topics": config["topics"], "counts": dict(counts),
                    "last_source_stamp_ns": latest, "errors": errors,
                    "writer_error": writer.error, "written": writer.completed,
                    "pending": writer.queue.qsize(), "final": final,
                    "ground_truth_subscribed": False}
        (args.output_dir / "health.json").write_text(json.dumps(document, ensure_ascii=False, indent=2))
    health()
    last = time.monotonic()
    try:
        while rclpy.ok() and not stopping and not (args.output_dir / "stop_recorder").exists():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - last >= 2:
                health()
                last = time.monotonic()
    finally:
        try:
            writer.close()
        except RuntimeError as error:
            errors.append(str(error))
        health(final=True)
        node.destroy_node()
        rclpy.shutdown()
    return int(bool(errors or writer.error))


if __name__ == "__main__":
    raise SystemExit(main())
