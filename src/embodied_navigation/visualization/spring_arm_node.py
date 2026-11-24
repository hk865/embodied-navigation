"""ROS 2 游戏风格 Spring-Arm 防穿墙多机位相机节点。

订阅后端修正里程计（或前端里程计）与去重全局点云，实时维护 3D 障碍体素栅格并广播三路相机 TF：
- ``rviz_cam_spring``: 第三人称游戏防穿墙/防穿天花板跟随相机
- ``rviz_cam_overhead``: 当前楼层垂直俯视追踪相机（配合 ``/dynanav/viz/cutaway_map`` 切除上层天花板）
- ``rviz_cam_isometric``: 45° 等轴双层立体追踪相机
"""

from __future__ import annotations

import argparse
import math
import struct
import time
import numpy as np

from embodied_navigation.localization_mapping.backend.node import pack_pointcloud2_xyz
from embodied_navigation.visualization.spring_arm_camera import (
    CameraPose,
    SpringArmCameraConfig,
    SpringArmCameraController,
    filter_floor_cutaway,
)


def unpack_pointcloud2_xyz(msg) -> np.ndarray:
    """从 ROS 2 ``sensor_msgs/PointCloud2`` 高效解析 (N, 3) float32 点云。"""
    n_pts = int(msg.width) * int(msg.height)
    if n_pts <= 0 or not msg.data:
        return np.empty((0, 3), dtype=np.float32)
    offsets = {f.name: (int(f.offset), int(f.datatype)) for f in msg.fields}
    if not all(k in offsets for k in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    step = int(msg.point_step)
    ox, tx = offsets["x"]
    oy, ty = offsets["y"]
    oz, tz = offsets["z"]
    if tx == 7 and ty == 7 and tz == 7 and ox == 0 and oy == 4 and oz == 8 and step == 12:
        return np.frombuffer(msg.data, dtype="<f4").reshape(-1, 3).copy()
    if tx == 7 and ty == 7 and tz == 7 and step >= 12:
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        n_avail = min(n_pts, len(raw) // step)
        if n_avail <= 0:
            return np.empty((0, 3), dtype=np.float32)
        buf = raw[: n_avail * step].reshape(n_avail, step)
        x = np.frombuffer(buf[:, ox : ox + 4].tobytes(), dtype="<f4")
        y = np.frombuffer(buf[:, oy : oy + 4].tobytes(), dtype="<f4")
        z = np.frombuffer(buf[:, oz : oz + 4].tobytes(), dtype="<f4")
        return np.column_stack((x, y, z))
    pts = np.empty((n_pts, 3), dtype=np.float32)
    data = bytes(msg.data)
    for i in range(n_pts):
        base = i * step
        pts[i, 0] = struct.unpack_from("<f", data, base + ox)[0]
        pts[i, 1] = struct.unpack_from("<f", data, base + oy)[0]
        pts[i, 2] = struct.unpack_from("<f", data, base + oz)[0]
    return pts


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _make_tf_msg(parent_frame: str, pose: CameraPose, stamp, TransformStamped):
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent_frame
    tf.child_frame_id = pose.frame_id
    tf.transform.translation.x = float(pose.position_xyz[0])
    tf.transform.translation.y = float(pose.position_xyz[1])
    tf.transform.translation.z = float(pose.position_xyz[2])
    tf.transform.rotation.x = float(pose.quaternion_xyzw[0])
    tf.transform.rotation.y = float(pose.quaternion_xyzw[1])
    tf.transform.rotation.z = float(pose.quaternion_xyzw[2])
    tf.transform.rotation.w = float(pose.quaternion_xyzw[3])
    return tf


def run_spring_arm_node(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="DynaNav Spring-Arm 防穿墙多机位相机 TF 与楼层切片节点")
    parser.add_argument("--odom-topic", default="/dynanav/backend/corrected_odometry")
    parser.add_argument("--fallback-odom-topic", default="/dynanav/rtx_livo/post_vio_odometry")
    parser.add_argument("--cloud-topic", default="/dynanav/backend/global_map")
    parser.add_argument("--fixed-frame", default="map")
    parser.add_argument("--arm-length-m", type=float, default=4.2)
    parser.add_argument("--min-arm-length-m", type=float, default=0.95)
    parser.add_argument("--pitch-deg", type=float, default=20.0)
    parser.add_argument("--overhead-height-m", type=float, default=13.5)
    parser.add_argument("--cutaway-above-m", type=float, default=1.85)
    parser.add_argument("--cutaway-below-m", type=float, default=0.90)
    parser.add_argument("--iso-distance-m", type=float, default=11.0)
    args, ros_unknown = parser.parse_known_args(argv)

    import rclpy
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry, Path
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker

    cfg = SpringArmCameraConfig(
        desired_arm_length_m=args.arm_length_m,
        min_arm_length_m=args.min_arm_length_m,
        pitch_deg=args.pitch_deg,
        overhead_height_m=args.overhead_height_m,
        cutaway_above_robot_m=args.cutaway_above_m,
        cutaway_below_robot_m=args.cutaway_below_m,
        iso_distance_m=args.iso_distance_m,
    )

    class SpringArmCameraNode(Node):
        def __init__(self) -> None:
            super().__init__("dynanav_spring_arm_camera")
            self.ctrl = SpringArmCameraController(cfg)
            self.fixed_frame = args.fixed_frame
            self._has_primary_odom = False
            self._latest_cloud: np.ndarray | None = None
            self._latest_robot_z: float | None = None
            self._last_cutaway_wall_s = 0.0
            self._trail = Path()
            self._trail.header.frame_id = self.fixed_frame
            self._last_trail_xy: np.ndarray | None = None

            self.tf_pub = self.create_publisher(TFMessage, "/tf", 30)
            self.trail_pub = self.create_publisher(Path, "/dynanav/viz/robot_trail", 10)
            self.marker_pub = self.create_publisher(Marker, "/dynanav/viz/robot_marker", 10)
            latched_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.cutaway_pub = self.create_publisher(PointCloud2, "/dynanav/viz/cutaway_map", latched_qos)

            best_effort = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(Odometry, args.odom_topic, self._on_primary_odom, 20)
            self.create_subscription(Odometry, args.odom_topic, self._on_primary_odom, best_effort)
            if args.fallback_odom_topic and args.fallback_odom_topic != args.odom_topic:
                self.create_subscription(Odometry, args.fallback_odom_topic, self._on_fallback_odom, best_effort)
            self.create_subscription(PointCloud2, args.cloud_topic, self._on_cloud, latched_qos)
            self.create_subscription(PointCloud2, args.cloud_topic, self._on_cloud, best_effort)
            self.get_logger().info(
                f"Spring-Arm camera active: odom={args.odom_topic} cloud={args.cloud_topic} fixed={self.fixed_frame}"
            )

        def _on_primary_odom(self, msg: Odometry) -> None:
            self._has_primary_odom = True
            self._handle_odom(msg)

        def _on_fallback_odom(self, msg: Odometry) -> None:
            if not self._has_primary_odom:
                self._handle_odom(msg)

        def _handle_odom(self, msg: Odometry) -> None:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            pos = np.array([p.x, p.y, p.z], dtype=np.float64)
            yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
            stamp = msg.header.stamp
            stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            self._latest_robot_z = float(pos[2])

            cams = self.ctrl.update(pos, yaw, stamp_s)
            parent = msg.header.frame_id or self.fixed_frame
            tfs = [_make_tf_msg(parent, cam, stamp, TransformStamped) for cam in cams.values()]
            self.tf_pub.publish(TFMessage(transforms=tfs))

            # 更新高亮机器人标记（胶囊机身 + 前向箭头）
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = parent
            marker.ns = "robot"
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = msg.pose.pose
            marker.scale.x = 0.75
            marker.scale.y = 0.28
            marker.scale.z = 0.25
            marker.color.r = 1.0
            marker.color.g = 0.45
            marker.color.b = 0.12
            marker.color.a = 1.0
            self.marker_pub.publish(marker)

            # 更新轨迹丝带（每 0.2 m 采样一点）
            if self._last_trail_xy is None or float(np.linalg.norm(pos - self._last_trail_xy)) >= 0.20:
                ps = PoseStamped()
                ps.header.stamp = stamp
                ps.header.frame_id = parent
                ps.pose = msg.pose.pose
                self._trail.header.stamp = stamp
                self._trail.header.frame_id = parent
                self._trail.poses.append(ps)
                self._last_trail_xy = pos.copy()
                self.trail_pub.publish(self._trail)

            self._maybe_publish_cutaway(stamp, parent)

        def _on_cloud(self, msg: PointCloud2) -> None:
            pts = unpack_pointcloud2_xyz(msg)
            if len(pts) == 0:
                return
            self._latest_cloud = pts
            self.ctrl.grid.replace_points(pts)
            self._maybe_publish_cutaway(msg.header.stamp, msg.header.frame_id or self.fixed_frame, force=True)

        def _maybe_publish_cutaway(self, stamp, frame_id: str, *, force: bool = False) -> None:
            if self._latest_cloud is None or self._latest_robot_z is None:
                return
            now_s = time.monotonic()
            if not force and (now_s - self._last_cutaway_wall_s) < 0.25:
                return
            self._last_cutaway_wall_s = now_s
            sliced = filter_floor_cutaway(
                self._latest_cloud,
                self._latest_robot_z,
                above_m=cfg.cutaway_above_robot_m,
                below_m=cfg.cutaway_below_robot_m,
            )
            if len(sliced) == 0:
                return
            cloud_msg = pack_pointcloud2_xyz(
                sliced,
                frame_id=frame_id,
                stamp=stamp,
                PointCloud2=PointCloud2,
                PointField=PointField,
            )
            self.cutaway_pub.publish(cloud_msg)

    rclpy.init(args=ros_unknown)
    node = SpringArmCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(run_spring_arm_node())
