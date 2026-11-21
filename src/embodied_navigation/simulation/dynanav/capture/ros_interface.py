"""采集探针的 ROS 接口：节点、话题、回调与发布者。

本模块独占 ``rclpy`` 的初始化/销毁：在运行期由 ``create_ros_interface``
创建 context 与节点、原样创建 ``/dynanav/*`` 发布者以及里程计/已批准指令
订阅者，定义两个回调，并把传感器循环拿到的数据转换成 ROS 消息。话题名、
消息类型与 QoS（``qos_profile_sensor_data``）与原探针保持一致；import 本模块
不会创建任何 ROS 实体。
"""

import os
import time
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np


@dataclass
class RosInterface:
    """rclpy 节点、发布/订阅回调与最新样本的句柄集合。

    ``latest_odom``/``latest_nav`` 是单元素列表，回调原地写入，读取方拿到的
    是最近一次样本（不是队列）；``published`` 只是本进程内的发布计数。
    """

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
    publish_rtx_cloud: Callable | None = None
    publish_truth: Callable | None = None
    publish_truth_vio: Callable | None = None
    publish_truth_deskewed_and_lio: Callable | None = None
    last_clock_ns: list | None = None



def _publish_sensor(publisher, message, topic, journal=None, *, point_count=None, metadata_message=None):
    """记录“本地 publish 调用”边界，不代表 DDS 已投递或收到确认。

    仅记录标量元数据，每个被跟踪的传感器帧写两行（publish_call/
    publish_returned），``delivery_confirmed`` 恒为 False。journal 或
    publisher 抛异常时故意向上传播，由 capture 的清理路径收尾。
    """
    if journal is None:
        publisher.publish(message)
        return
    # Typed envelopes publish unchanged; their embedded cloud supplies trace metadata.
    observed = message if metadata_message is None else metadata_message
    metadata = {
        "topic": topic,
        "source_ns": observed.header.stamp.sec * 1_000_000_000 + observed.header.stamp.nanosec,
        "data_bytes": len(observed.data),
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

_SHUTDOWN_ACK_BUDGET_NS = 5_000_000_000


def _wait_for_recording_acknowledgments(node, publishers, receipt):
    """Give DDS a shared, bounded wall-clock window to acknowledge queued data."""
    from rclpy.duration import Duration
    from rclpy.qos import ReliabilityPolicy

    def subscribers(publisher):
        return {
            "matched_count": publisher.get_subscription_count(),
            "discovered_reliable_nodes": sorted({
                f"{endpoint.node_namespace.rstrip('/')}/{endpoint.node_name}"
                for endpoint in node.get_subscriptions_info_by_topic(publisher.topic_name)
                if endpoint.qos_profile.reliability == ReliabilityPolicy.RELIABLE
            }),
        }

    started_ns = time.monotonic_ns()
    deadline_ns = started_ns + _SHUTDOWN_ACK_BUDGET_NS
    receipt.update(
        status="running", budget_wall_s=_SHUTDOWN_ACK_BUDGET_NS / 1e9,
        scope="DDS writer acknowledgments only; verifies neither recorder receipt nor bag persistence",
        topics={},
    )
    for publisher in publishers:
        if publisher is None:
            continue
        evidence = {"wait_called": False, "wait_wall_s": 0.0}
        receipt["topics"][publisher.topic_name] = evidence
        try:
            before = subscribers(publisher)
            evidence["subscribers_before"] = before
            if before["matched_count"] == 0:
                evidence["status"] = "no_matched_subscribers"
                continue
            wait_started_ns = time.monotonic_ns()
            remaining_ns = max(0, deadline_ns - wait_started_ns)
            evidence.update(wait_called=True, wait_timeout_wall_s=remaining_ns / 1e9)
            try:
                acknowledged = publisher.wait_for_all_acked(Duration(nanoseconds=remaining_ns))
                evidence["wait_returned"] = acknowledged
            finally:
                evidence["wait_wall_s"] = (time.monotonic_ns() - wait_started_ns) / 1e9
            after = subscribers(publisher)
            evidence["subscribers_after"] = after
            if not acknowledged:
                evidence["status"] = "timeout"
            elif after["matched_count"] == 0:
                evidence["status"] = "no_matched_subscribers_after_wait"
            elif not (before["discovered_reliable_nodes"] and after["discovered_reliable_nodes"]):
                evidence["status"] = "no_reliable_subscriber_observed"
            else:
                evidence["status"] = "wait_succeeded"
        except Exception as error:
            evidence.update(status="error", error=repr(error))
    receipt.update(status="finished", elapsed_wall_s=(time.monotonic_ns() - started_ns) / 1e9)


def create_ros_interface(
    experiment, learned_mapper, learned_scorer, *, journal=None, shutdown_receipt=None
):
    """按原探针的语义创建采集 ROS 节点。

    ``experiment`` 是本次运行的配置映射：``odometry_topic`` 缺省为
    ``/aft_mapped_to_init``；``slam_pose_source`` 决定里程计是否带速度
    （livo_post_vio 为纯位姿，速度置 None）；``lidar_frontend_topic`` 缺省时
    按是否启用 learned_mapper 在 filtered/static 之间选择。``journal`` 非空且
    环境变量 ``DYNANAV_PUBLISH_TRACE=1`` 时，才跟踪前端点云的发布边界。
    """
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
    from embodied_navigation.simulation.dynanav.truth_deskew import (
        TRUTH_DESKEWED_CLOUD_TOPIC,
        TRUTH_LIO_ODOM_TOPIC,
        TRUTH_POSE_SOURCE,
        TRUTH_VIO_ODOM_TOPIC,
        truth_deskewed_cloud_message,
        truth_odometry_message,
    )

    rtx_enabled = experiment.get("point_cloud_source", "rgbd") == "rtx_lidar"
    truth_mode = experiment.get("slam_pose_source") == TRUTH_POSE_SOURCE

    # Optional recording profile: reliable source writers permit bag retransmission
    # of large fragmented sensor messages. Default/native subscribers remain unchanged.
    reliable_recording = os.environ.get("DYNANAV_RECORD_RELIABLE") == "1"
    sensor_qos = qos_profile_sensor_data
    if reliable_recording:
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        sensor_qos = QoSProfile(depth=128, reliability=ReliabilityPolicy.RELIABLE)
    if shutdown_receipt is None:
        shutdown_receipt = {}
    shutdown_receipt["status"] = "pending" if reliable_recording else "disabled"

    # 发布追踪是显式实验开关：开启后必须提供 journal，否则直接拒绝启动，
    # 避免把“未记录”误当成“已投递”。
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

    # 交给采集编排统一处理 SIGINT/SIGTERM：rclpy 不安装自己的信号处理器，
    # 这样关停顺序（先停传感器循环再销毁节点）由上层保证。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("actual_go2_rtx_source" if rtx_enabled else "actual_go2_rgbd_source")
    pub_imu = node.create_publisher(Imu, "/dynanav/imu", sensor_qos)
    pub_img = node.create_publisher(Image, "/dynanav/camera", sensor_qos)
    pub_cloud = node.create_publisher(
        PointCloud2, "/dynanav/lidar_filtered", sensor_qos
    )
    pub_static_cloud = node.create_publisher(
        PointCloud2, "/dynanav/lidar_static", sensor_qos
    )
    pub_perception_cloud = (
        node.create_publisher(
            PointCloud2, "/dynanav/lidar_perception_filtered", sensor_qos
        )
        if learned_mapper is not None
        else None
    )
    pub_depth = node.create_publisher(Image, "/dynanav/depth", sensor_qos)
    pub_info = node.create_publisher(
        CameraInfo, "/dynanav/camera_info", sensor_qos
    )
    pub_rtx = node.create_publisher(PointCloud2, "/dynanav/lidar_rtx", sensor_qos) if rtx_enabled else None
    pub_complete_rtx = None
    if rtx_enabled and os.environ.get("DYNANAV_RTX_COMPLETE_SCAN") == "1":
        from dynanav_sensor_msgs.msg import CompleteLidarScan
        from .lidar import COMPLETE_SCAN_TOPIC
        pub_complete_rtx = node.create_publisher(CompleteLidarScan, COMPLETE_SCAN_TOPIC, sensor_qos)
    pub_truth_cloud = (
        node.create_publisher(PointCloud2, TRUTH_DESKEWED_CLOUD_TOPIC, sensor_qos)
        if truth_mode
        else None
    )
    pub_truth_lio = (
        node.create_publisher(Odometry, TRUTH_LIO_ODOM_TOPIC, sensor_qos)
        if truth_mode
        else None
    )
    pub_truth_vio = (
        node.create_publisher(Odometry, TRUTH_VIO_ODOM_TOPIC, sensor_qos)
        if truth_mode
        else None
    )
    latest_odom = [None]

    def odom_callback(m):
        # 只保留最近一帧里程计；source_ns 用消息头时间，arrival_wall 用本机
        # 单调时钟，二者分属不同时间域，消费方不能直接相减。
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
            # 纯位姿后端（livo_post_vio / backend_corrected / simulation_truth）不提供可信速度，显式置 None，
            # 而不是把 Twist 里的零当成“静止”。
            "estimated_velocity": None
            if experiment.get("slam_pose_source") in ("livo_post_vio", "backend_corrected", "simulation_truth")
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

        # learned 建图在里程计回调里顺带推进：投递位姿并取走已完成的建图结果；
        # scorer 只观察完成事件，不参与 ROS 回调的返回值。
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
    # /clock 使用默认可靠 QoS（depth=10），由采集循环按传感器时间戳发布。
    pub_clock = node.create_publisher(Clock, "/clock", 10)
    pub_truth = (node.create_publisher(Odometry, "/dynanav/truth/base_odometry", 10)
                 if (experiment.get("record_simulation_truth", False)
                     and experiment.get("control_mode") == "native_pose_diagnostic") else None)

    def nav_callback(m):
        # 已批准指令按“导航请求”保存：request=[vx, vy, wz]（m/s、m/s、rad/s）。
        latest_nav[0] = {
            "time": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            "request": [m.twist.linear.x, m.twist.linear.y, m.twist.angular.z],
            "arrival_wall": time.perf_counter(),
        }

    nav_sub = node.create_subscription(
        TwistStamped, "/dynanav/nav/approved_cmd", nav_callback, 10
    )

    def header(m, stamp, frame):
        # 传感器时间统一为浮点秒，这里一次性拆成 ROS 的 sec/nanosec；
        # round 避免浮点乘法截断出少 1 ns 的边界误差。
        ns = round(stamp * 1e9)
        m.header.stamp.sec = ns // 1000000000
        m.header.stamp.nanosec = ns % 1000000000
        m.header.frame_id = frame
    published = {"imu": 0, "rgb": 0, "cloud": 0}
    last_clock_ns = [None]

    def spin_once(timeout_sec=0.0):
        rclpy.spin_once(node, timeout_sec=timeout_sec)

    def drain(times):
        for _ in range(times):
            rclpy.spin_once(node, timeout_sec=0.0)

    def publish_imu_and_clock(it, reading, orientation_available, *, publish_imu: bool = True):
        # 先发布 /clock 再发布 IMU，保证订阅方看到的时间不落后于同批数据；
        # it 为本迭代的传感器时间（秒），不是墙钟。
        clock = Clock()
        ns = round(it * 1e9)
        clock.clock.sec = ns // 1000000000
        clock.clock.nanosec = ns % 1000000000
        pub_clock.publish(clock)
        last_clock_ns[0] = ns
        if not publish_imu:
            return
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
            # 惯导约定：orientation_covariance[0] = -1 表示姿态字段不可用，
            # 下游必须改用角速度积分，不能把默认四元数当成有效姿态。
            msg.orientation_covariance[0] = -1.0
        pub_imu.publish(msg)
        published["imu"] += 1


    def publish_cloud(it, xyz, states, frontend_keep, oracle_mapper, learned_mapper):
        # 输入 xyz 为 N×3（来自 frames.depth_cloud_packet，已在 IMU 系），
        # 这里补成 x/y/z/intensity 四个 float32 字段（point_step=16）；
        # intensity 列没有上游数据，保持 0，本函数不计算强度。
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
        # oracle 建图只消费静态点：STATIC 标签来自上游语义状态，不在此处判据。
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
        # learned 建图消费前端过滤后的点，frontend_keep 是与 xyz 等长的布尔掩码；
        # 仅这条 filtered 话题的发布边界会被 DYNANAV_PUBLISH_TRACE 跟踪。
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


    def publish_rtx_cloud(packet):
        if pub_rtx is None:
            raise ValueError("RTX publisher not explicitly configured")
        from .lidar import pointcloud_message
        message = pointcloud_message(packet)
        publish_cloud_message(pub_rtx, message, "/dynanav/lidar_rtx")
        published["cloud"] += 1
        published["rtx_lidar"] = published.get("rtx_lidar", 0) + 1
        if pub_complete_rtx is not None:
            from .lidar import complete_scan_message
            envelope = complete_scan_message(packet, message)
            if envelope is not None:
                _publish_sensor(
                    pub_complete_rtx, envelope, COMPLETE_SCAN_TOPIC, trace_journal,
                    point_count=message.width * message.height, metadata_message=message,
                )
                published["rtx_complete_scan"] = published.get("rtx_complete_scan", 0) + 1

    def publish_depth_and_info(it, zz):
        # 深度图按 32FC1（米）发布，frame 为相机光学系；图像尺寸取自数组形状，
        # 因此不强制 512x512。
        depth_msg = Image()
        header(depth_msg, it, "go2_camera_optical")
        depth_msg.height, depth_msg.width = zz.shape
        depth_msg.encoding, depth_msg.step = "32FC1", zz.shape[1] * 4
        depth_msg.data = np.ascontiguousarray(zz, dtype="<f4").tobytes()
        pub_depth.publish(depth_msg)
        publish_camera_info(it, *zz.shape)

    def publish_camera_info(it, height, width):
        info = CameraInfo()
        header(info, it, "go2_camera_optical")
        info.height, info.width = height, width
        # 内参沿用原探针公式 focal = 512*24/42（约 292.57 px），主点固定在
        # 图像中心 (256, 256)，零畸变；这些常数对应的标定来源代码与文档均未说明。
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
        # RGB 固定 512x512 rgb8（step=1536），尺寸在本函数里硬编码，
        # 不做缩放；与相机内参的 512 像素假设保持一致。
        msg = Image()
        header(msg, it, "go2_camera_optical")
        msg.height = 512
        msg.width = 512
        msg.encoding = "rgb8"
        msg.step = 1536
        msg.data = np.ascontiguousarray(rgb).tobytes()
        _publish_sensor(pub_img, msg, "/dynanav/camera", trace_journal)
        if rtx_enabled:
            publish_camera_info(it, msg.height, msg.width)
        published["rgb"] += 1

    def publish_truth(
        stamp, position, quaternion_wxyz, velocity_com_world, omega_world, com_local
    ):
        if pub_truth is None:
            return
        msg = Odometry()
        header(msg, stamp, "simulation_world")
        msg.child_frame_id = "go2_base_truth"
        xyz, quat = msg.pose.pose.position, msg.pose.pose.orientation
        xyz.x, xyz.y, xyz.z = map(float, position)
        quat.w, quat.x, quat.y, quat.z = map(float, quaternion_wxyz)
        from embodied_navigation.robot.unitree.policy.native.observation import quaternion_rotation

        rotation = quaternion_rotation(quaternion_wxyz)
        # PhysX reports COM velocity. Odometry twist is at the base origin in the child frame.
        velocity_base_world = np.asarray(velocity_com_world) - np.cross(
            omega_world, rotation @ com_local
        )
        linear = rotation.T @ velocity_base_world
        angular = rotation.T @ omega_world
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = map(float, linear)
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = map(float, angular)
        pub_truth.publish(msg)
        published["truth"] = published.get("truth", 0) + 1

    def publish_truth_vio(stamp_ns, pos_wi, quat_wi_wxyz):
        if pub_truth_vio is None:
            return
        msg = truth_odometry_message(stamp_ns, pos_wi, quat_wi_wxyz)
        pub_truth_vio.publish(msg)
        published["truth_vio"] = published.get("truth_vio", 0) + 1

    def publish_truth_deskewed_and_lio(scan):
        if pub_truth_lio is None or pub_truth_cloud is None:
            return
        lio_msg = truth_odometry_message(
            scan.cloud_stamp_ns, scan.pair_pos_wi, scan.pair_quat_wi_wxyz
        )
        cloud_msg = truth_deskewed_cloud_message(scan)
        # 先发布等价 LIO 位姿，紧接着发布同戳世界系去畸变点云。
        pub_truth_lio.publish(lio_msg)
        pub_truth_cloud.publish(cloud_msg)
        published["truth_lio"] = published.get("truth_lio", 0) + 1
        published["truth_cloud"] = published.get("truth_cloud", 0) + 1

    def destroy():
        # 正常及异常收尾都已退出采集循环，不再推进物理。DDS 仍可发送队列中的数据。
        try:
            if reliable_recording:
                writers = (
                    pub_img, pub_complete_rtx, pub_rtx, pub_info, pub_imu,
                    pub_clock, pub_truth, pub_truth_cloud, pub_truth_lio, pub_truth_vio,
                )
                if not rtx_enabled:
                    writers += (pub_depth, pub_cloud, pub_static_cloud, pub_perception_cloud)
                _wait_for_recording_acknowledgments(node, writers, shutdown_receipt)
        finally:
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
        publish_rtx_cloud=publish_rtx_cloud,
        publish_truth=publish_truth,
        publish_truth_vio=publish_truth_vio,
        publish_truth_deskewed_and_lio=publish_truth_deskewed_and_lio,
        last_clock_ns=last_clock_ns,
    )
