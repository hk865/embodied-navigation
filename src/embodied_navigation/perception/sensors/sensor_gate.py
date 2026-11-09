#!/usr/bin/env python3
"""Warm up actual sensor SLAM before permitting a bounded route request."""

# 中文模块说明（首轮补充；上方 docstring 恢复 HEAD 原文，以保持 --help/--doc 文案不变）：
# 在允许有界路线请求之前预热实际传感器 SLAM，并持续给出健康门。
#
# 节点订阅 IMU、原始 LiDAR、图像、CameraInfo、SLAM body odom 与配准点云，校验帧、
# 尺寸、内参与时间戳递增，检查 image 与 CameraInfo 的 50 ms 配对以及发布者图；结果
# 写入 run/navigation_sensor_gate.json 并追加 history，同时向
# /demo/teacher/navigation/sensor_health 发布 ready/hold/failed。
#
# 边界：只读传感器与 SLAM 输出，不写导航状态机、不发路线请求；不读 SLAM 世界位姿或
# 真值做过滤（ground_truth_navigation_used 恒为 False）。节点用 use_sim_time 与单一
# rclpy ROS TimeSource /clock，避免第二个时钟队列滞后。

from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import time
import sys
from embodied_navigation.common.runtime_io import BackgroundCheck, latest_sensor_qos
from embodied_navigation.common.runtime_telemetry import RuntimeTelemetry
from embodied_navigation.common.input_freshness import enabled as minimal_baseline_enabled

# 健康门状态 topic；tick 在这里发布 ready/hold/failed 的 JSON。
TOPIC = "/demo/teacher/navigation/sensor_health"


class ImageInfoPairs:
    """有界的原始 stamp 图像/Info 配对缓存；新的未配对 Info 不会抹掉旧样本。"""

    def __init__(self, capacity=32):
        """capacity 为每个缓存保留的最大 stamp 数（默认硬编码 32）。"""
        self.capacity = capacity
        self.images = {}
        self.infos = {}

    def observe(self, name, stamp, frame, width, height, wall):
        """按源 stamp 缓存一条 image 或 camera_info 样本。

        参数 name 为 "image" 或 "camera_info"；stamp 为 header 纳秒；frame/width/
        height 为消息元数据；wall 为单调墙钟接收秒。存储元组为
        (frame_id, width, height, received_wall_s)，只保留最新 capacity 个 stamp，
        超出时逐出最小 stamp。返回 None。
        """
        cache = self.images if name == "image" else self.infos
        cache[stamp] = (frame, width, height, wall)
        # 只按最小 stamp 逐出：缓存满时丢掉最旧的配对候选。
        while len(cache) > self.capacity:
            cache.pop(min(cache))

    def latest_fresh(self, clock_ns, wall_now, sim_limit=0.3, wall_limit=0.3, observe_wall=False):
        """返回最近一对时间窗内配对的 image/CameraInfo；找不到返回 None。

        参数：
            clock_ns：当前 ROS 仿真时钟（纳秒）；None 时直接返回 None。
            wall_now：当前单调墙钟秒。
            sim_limit：仿真时钟最大源龄（秒，默认 0.3）。
            wall_limit：墙钟最大接收龄（秒，默认 0.3）。
            observe_wall：True 时只要求接收龄非负、不检查上限（诊断观察模式）。
        规则硬编码：图像源龄落在 [-50 ms, sim_limit)；CameraInfo 必须与图像同
        frame/宽高、stamp 差 <= 50 ms 且源龄同窗口；接收龄非负且（observe_wall 或
        小于 wall_limit）。按图像 stamp 从新到旧搜索，返回第一张有候选的图像中差异
        最小的配对及其年龄。
        """
        if clock_ns is None:
            return None
        # 从新到旧找第一张满足窗口的图像；-50 ms 是未来容差，吸收 sim 时钟与 header
        # 的微小时序差。
        for image_stamp, image in sorted(self.images.items(), reverse=True):
            if not (
                -50_000_000 <= clock_ns - image_stamp < round(sim_limit * 1e9)
                and 0 <= wall_now - image[3]
                and (observe_wall or wall_now - image[3] < wall_limit)
            ):
                continue
            # 只与同 frame/宽高、stamp 差 <= 50 ms 的 Info 配对；多个候选取 |差| 最小。
            candidates = [
                (abs(stamp - image_stamp), stamp, info)
                for stamp, info in self.infos.items()
                if info[:3] == image[:3]
                and abs(stamp - image_stamp) <= 50_000_000
                and -50_000_000 <= clock_ns - stamp < round(sim_limit * 1e9)
                and 0 <= wall_now - info[3]
                and (observe_wall or wall_now - info[3] < wall_limit)
            ]
            if candidates:
                difference, info_stamp, info = min(candidates)
                return {
                    "image_stamp_ns": image_stamp,
                    "camera_info_stamp_ns": info_stamp,
                    "pair_stamp_difference_ns": difference,
                    "frame_id": image[0],
                    "width": image[1],
                    "height": image[2],
                    "image_wall_age_s": wall_now - image[3],
                    "camera_info_wall_age_s": wall_now - info[3],
                }
        return None


class WarmupRecovery:
    """累计真实初始化历史；当前版本恢复判据仍严格。

    孤立的 cloud 缺口不会抹掉 FAST-LIVO 已经初始化所用的样本；但它仍会立即停止
    运动并重置 fresh recovery 样本。
    """

    def __init__(self, warmup_s=2.0, recovery_samples=2, recovery_span_s=0.1, requirements=None):
        """参数单位为秒与样本数；requirements 缺省为 body_odom/cloud 各 10 样本、2 s。

        Gate 从 profile 传入 sensor_warmup_sim_s、recovery_fresh_pose_samples、
        recovery_min_pose_span_sim_s 与 warmup_history（当前 config 含 6 个源）。
        """
        # 秒统一换算成纳秒，避免浮点比较。
        self.warmup_ns = round(warmup_s * 1e9)
        self.recovery_samples = recovery_samples
        self.recovery_span_ns = round(recovery_span_s * 1e9)
        # complete 是累计初始化历史；samples 是当前恢复段的 fresh pose 样本数。
        self.complete = False
        self.good_since = None
        self.first_pose = None
        self.last_pose = None
        self.samples = 0
        # 缺省要求是硬编码的 10 样本/2 s；生产路径由 profile 的 warmup_history 覆盖。
        self.requirements = requirements or {
            "body_odom": {"min_samples": 10, "min_span_sim_s": warmup_s},
            "cloud": {"min_samples": 10, "min_span_sim_s": warmup_s},
        }
        self.history = {}

    def record(self, name, stamp_ns):
        """记录一条源 stamp 到累计初始化历史，并重算 complete。

        只处理 requirements 中的源；stamp 严格大于该源上次记录才累加样本。返回
        None。complete = 所有要求源都达到 min_samples 且 last-first >=
        min_span_sim_s。
        """
        if name not in self.requirements:
            return
        previous = self.history.get(name)
        if previous is None:
            self.history[name] = {
                "first_stamp_ns": stamp_ns,
                "last_stamp_ns": stamp_ns,
                "samples": 1,
            }
        # 重复或回退的 stamp 不累加样本，也不推进 last_stamp。
        elif stamp_ns > previous["last_stamp_ns"]:
            previous["last_stamp_ns"] = stamp_ns
            previous["samples"] += 1
        self.complete = all(
            name in self.history
            and self.history[name]["samples"] >= rule["min_samples"]
            and self.history[name]["last_stamp_ns"] - self.history[name]["first_stamp_ns"]
            >= round(rule["min_span_sim_s"] * 1e9)
            for name, rule in self.requirements.items()
        )

    def observe(self, healthy, clock_ns, pose_stamp_ns):
        """更新 fresh 恢复段并返回当前是否满足恢复放行条件。

        healthy 为 False 或时钟/位姿 stamp 缺失时，立即清零 good_since/first_pose/
        last_pose/samples 并返回 False（运动必须立即停止）。healthy 时按 pose stamp
        严格递增累计 samples，返回 complete 且 samples >= recovery_samples 且
        last_pose-first_pose >= recovery_span_ns。
        """
        # 任何不健康或缺失都重置恢复段：这是“立即停止运动”的语义。
        if not healthy or clock_ns is None or pose_stamp_ns is None:
            self.good_since = None
            self.first_pose = None
            self.last_pose = None
            self.samples = 0
            return False
        if self.good_since is None:
            self.good_since = clock_ns
        # 只有更新的 pose stamp 才计数，避免同一帧重复计入。
        if self.last_pose is None or pose_stamp_ns > self.last_pose:
            self.samples += 1
            self.last_pose = pose_stamp_ns
            if self.first_pose is None:
                self.first_pose = pose_stamp_ns
        return (
            self.complete
            and self.samples >= self.recovery_samples
            and self.last_pose - self.first_pose >= self.recovery_span_ns
        )


def atomic(path, data):
    """通过同目录 .tmp 文件写 JSON 再原子替换，读者不会看到半截文本。

    使用 allow_nan=False、ensure_ascii=False 并追加换行；写或替换失败时异常向上抛。
    """
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, allow_nan=False, ensure_ascii=False) + "\n")
    tmp.replace(path)


from embodied_navigation.common.shutdown import (
    ShutdownRequested as _ShutdownRequested,
    ShutdownSignals as _ShutdownSignals,
)


def _main(shutdown_signals):
    """按冻结的 run context 构造 ROS 门节点并 spin 到关停。

    从 run/runtime_context.json 取 profile，从 sensor_contract.json 取相机标称频率，
    从 navigation_camera.yaml 取 K 的 fx/cx/fy/cy。关停时写最后一次 hold 状态并关闭
    后台图检查。
    """
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from sensor_msgs.msg import Image, CameraInfo, Imu, PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    import numpy as np
    import yaml
    from sensor_msgs_py import point_cloud2

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    args, ros = parser.parse_known_args()
    run = args.run.resolve()
    from embodied_navigation.common.runtime_context import load as load_context

    # 三类运行期输入：业务 profile、本次 run 的相机标称频率、本次 run 的相机 K。
    profile = load_context(run / "runtime_context.json")["profile"]
    minimal_baseline = minimal_baseline_enabled(profile)
    actual_camera_rate = json.loads((run / "sensor_contract.json").read_text())["camera"]["rate_hz"]
    calib = yaml.safe_load((run / "navigation_camera.yaml").read_text())["/**"]["ros__parameters"]
    # 信号由外层 ShutdownSignals 管理，因此不让 rclpy 安装信号处理器。
    rclpy.init(args=ros, signal_handler_options=SignalHandlerOptions.NO)

    class Gate(Node):
        """传感器/SLAM 健康门节点。"""

        def __init__(self):
            """建立 6 路传感器订阅、健康发布器、后台图检查与 10 Hz tick。"""
            super().__init__("teacher_navigation_sensor_gate")
            # 全部传感器与 SLAM 使用仿真时钟；本节点只信 rclpy 的单一 /clock 源。
            self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
            self.clock_ns = None
            self.clock_wall = None
            self.stamps = {}
            self.walls = {}
            self.counts = {}
            self.errors = {}
            self.pairs = ImageInfoPairs()
            self.failure = None
            self.last_report = None
            self.warmup = WarmupRecovery(
                profile["sensor_warmup_sim_s"],
                profile["recovery_fresh_pose_samples"],
                profile["recovery_min_pose_span_sim_s"],
                profile["warmup_history"],
            )
            self.evidence = RuntimeTelemetry()
            self.pub = self.create_publisher(String, TOPIC, 10)
            # 6 个源统一用 best-effort depth=1（只保留最新样本），匹配高频传感流。
            for name, typ, topic in [
                ("imu", Imu, "/livox/imu"),
                ("raw_lidar", PointCloud2, "/demo/teacher/raw_lidar"),
                ("image", Image, "/demo/camera"),
                ("camera_info", CameraInfo, "/demo/camera_info"),
                ("body_odom", Odometry, "/demo/slam/body_odom"),
                ("cloud", PointCloud2, "/cloud_registered_full"),
            ]:
                self.create_subscription(
                    typ, topic, lambda m, n=name: self.receive(n, m), latest_sensor_qos()
                )
            # 发布者图检查放在后台线程（周期 0.1 s）；tick 用 STEADY_TIME 墙钟 10 Hz，
            # 不受仿真时钟暂停影响。
            self.graph_check = BackgroundCheck(self.graph)
            self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def sync_clock(self):
            """把 rclpy 的 ROS 时钟同步到本地；回退视为失败。"""
            # 只用 rclpy 唯一的 ROS TimeSource /clock(depth=1)：此前第二个自定义
            # clock 队列曾比当前 ROS 时钟滞后 460 ms。
            ns = self.get_clock().now().nanoseconds
            # 时钟回退立即置 failure；后续 tick 以 failed 状态 fail-closed。
            if self.clock_ns is not None and ns < self.clock_ns:
                self.failure = "Simulation clock moved backward"
            # 只在时钟前进时更新 clock_ns 与对应的单调墙钟接收时刻。
            if self.clock_ns is None or ns > self.clock_ns:
                self.clock_ns = ns
                self.clock_wall = time.monotonic()

        def receive(self, name, msg):
            """校验一条传感器消息；通过则更新时间戳、计数与预热历史。

            所有源先同步时钟，并只接受 header stamp 严格递增的消息；校验失败时记录
            errors[name] 并返回，不更新时间戳。具体校验见各分支。
            """
            self.sync_clock()
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            # 重复/回退 stamp 直接丢弃：不更新 stamps、不计数，也不进入错误统计。
            if stamp <= self.stamps.get(name, -1):
                return
            error = None
            if name == "body_odom":
                # body odom 必须是 camera_init 下的 demo_slam_body，位姿 7 值有限，
                # 四元数范数平方落在 [0.98, 1.02]。
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                if (
                    msg.header.frame_id != "camera_init"
                    or msg.child_frame_id != "demo_slam_body"
                    or not all(math.isfinite(v) for v in values)
                    or not 0.98 <= sum(v * v for v in values[3:]) <= 1.02
                ):
                    error = "Invalid measured SLAM body pose/frame"
            elif name == "cloud":
                # 配准点云必须非空且位于 camera_init；随后解码 XYZ 并要求 >=20 个有限点。
                if msg.header.frame_id != "camera_init" or msg.width * msg.height < 20:
                    error = "Registered cloud is empty or in wrong frame"
                else:
                    try:
                        xyz = point_cloud2.read_points_numpy(
                            msg, field_names=("x", "y", "z"), skip_nans=True
                        )
                        if len(xyz) < 20 or not np.isfinite(xyz).all():
                            error = "Registered cloud XYZ invalid"
                    except (ValueError, TypeError):
                        # 解码失败按不可解码处理，不让异常逃出回调。
                        error = "Registered cloud cannot be decoded"
            elif name == "image":
                if (
                    msg.header.frame_id != "demo_camera_optical_frame"
                    or msg.width != 640
                    or msg.height != 480
                    or not msg.data
                ):
                    error = "Vehicle image frame/dimensions invalid"
                else:
                    # 校验通过后写入配对缓存，wall 用单调墙钟接收时刻。
                    self.pairs.observe(
                        name, stamp, msg.header.frame_id, msg.width, msg.height, time.monotonic()
                    )
            elif name == "camera_info":
                # 期望 K 来自本次 run 的 navigation_camera.yaml：
                # [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]。
                expected = [
                    calib["cam_fx"],
                    0.0,
                    calib["cam_cx"],
                    0.0,
                    calib["cam_fy"],
                    calib["cam_cy"],
                    0.0,
                    0.0,
                    1.0,
                ]
                if (
                    msg.header.frame_id != "demo_camera_optical_frame"
                    or msg.width != 640
                    or msg.height != 480
                    or msg.distortion_model != "plumb_bob"
                    or len(msg.d) < 4
                    or any(abs(v) > 1e-12 for v in msg.d)
                    # Gazebo 的 float32 K 量化约 1.065e-5 px；2e-5 px 容差不改变
                    # 几何边界。
                    or not np.allclose(msg.k, expected, rtol=0, atol=2e-5)
                ):
                    error = "Actual vehicle CameraInfo does not match pinhole K/zero D"
                else:
                    self.pairs.observe(
                        name, stamp, msg.header.frame_id, msg.width, msg.height, time.monotonic()
                    )
            elif name == "raw_lidar":
                # 原始 LiDAR 必须在 velodyne 帧且点数 >=20。
                if msg.header.frame_id != "velodyne" or msg.width * msg.height < 20:
                    error = "Raw actual LiDAR invalid"
            elif name == "imu":
                # IMU 只要求角速度与线加速度共 6 个分量有限。
                values = [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ]
                if not all(math.isfinite(v) for v in values):
                    error = "Actual IMU nonfinite"
            # 有错只记该源最近错误并返回；stamps/walls/counts/预热历史都不更新。
            if error:
                self.errors[name] = error
                return
            self.errors.pop(name, None)
            self.stamps[name] = stamp
            self.walls[name] = time.monotonic()
            self.counts[name] = self.counts.get(name, 0) + 1
            self.warmup.record(name, stamp)

        def graph(self):
            """由后台线程调用：校验各 topic 的实际发布者节点是否符合预期。

            返回 topic -> {expected, publishers, passed}；passed 要求恰好一个发布者且
            node_name/node_namespace 相符。known_scene localization contract 启用时
            追加两条 raw topic，并改变 /cloud_registered_full 的期望节点。
            """
            expected = {
                "/aft_mapped_to_init": "laserMapping",
                "/demo/slam/body_odom": "demo_slam_odom_adapter",
                "/cloud_registered_full": (
                    "demo_slam_odom_adapter"
                    if profile.get("known_scene_localization_contract", {}).get("enabled")
                    else "laserMapping"
                ),
                "/demo/slam/lidar_filtered": "demo_slam_self_echo_filter",
                "/demo/teacher/slam/image": "teacher_slam_actual_sensor_relay",
                "/demo/teacher/slam/imu": "teacher_slam_actual_sensor_relay",
            }
            results = {}
            # 期望的发布链是硬编码的节点名；contract 启用时追加 raw 适配链路。
            if profile.get("known_scene_localization_contract", {}).get("enabled"):
                expected.update(
                    {
                        "/demo/slam/raw_body_odom": "demo_slam_raw_odom_adapter",
                        "/demo/slam/raw_registered_full": "laserMapping",
                    }
                )
            for topic, name in expected.items():
                infos = self.get_publishers_info_by_topic(topic)
                results[topic] = {
                    "expected": name,
                    "publishers": [i.node_namespace + "/" + i.node_name for i in infos],
                    "passed": len(infos) == 1
                    and infos[0].node_name == name
                    and infos[0].node_namespace == "/",
                }
            return results

        def tick(self):
            """10 Hz 健康门主循环：汇总原因、写证据并发布 ready/hold/failed。"""
            self.sync_clock()
            now = time.monotonic()
            reasons = []
            ages = {}
            ns = self.clock_ns
            wall_observations = []
            if self.evidence.error:
                self.failure = self.evidence.error
            if self.failure:
                reasons.append(self.failure)
            clock_age = None if self.clock_wall is None else now - self.clock_wall
            # 非诊断模式下 clock 超过 300 ms（硬编码）未更新即判 stale；minimal 分支
            # 依赖 input_freshness.enabled()，当前恒为 False，不可达。
            if (
                minimal_baseline
                and clock_age is not None
                and math.isfinite(clock_age)
                and clock_age >= 0.3
            ):
                wall_observations.append(
                    {"field": "clock_wall_s", "age_s": clock_age, "original_limit_s": 0.3}
                )
            if (
                self.clock_wall is None
                or (not minimal_baseline and clock_age >= 0.3)
                or (minimal_baseline and (not math.isfinite(clock_age) or clock_age < 0))
            ):
                reasons.append("Actual simulation clock stale")
            for name in ("imu", "raw_lidar", "image", "camera_info", "body_odom", "cloud"):
                wall_age = None if name not in self.walls else now - self.walls[name]
                sim_age = (
                    None
                    if ns is None or name not in self.stamps
                    else (ns - self.stamps[name]) / 1e9
                )
                ages[name] = {
                    "wall_s": wall_age,
                    "sim_s": sim_age,
                    "stamp_ns": self.stamps.get(name),
                }
                # 2 Hz RGB 不能用 300 ms 的运动反馈看门狗。相机频率取自本次 run 的
                # sensor_contract.json：<10 Hz 时用 2.0 s 墙钟 / 0.75 s sim，否则
                # 用 0.3/0.3。
                wall_limit, sim_limit = (
                    ((2.0, 0.75) if actual_camera_rate < 10 else (0.3, 0.3))
                    if name in ("image", "camera_info")
                    else (0.3, 0.3)
                )
                if (
                    minimal_baseline
                    and wall_age is not None
                    and math.isfinite(wall_age)
                    and wall_age >= wall_limit
                ):
                    wall_observations.append(
                        {
                            "field": name + "_wall_s",
                            "age_s": wall_age,
                            "original_limit_s": wall_limit,
                        }
                    )
                wall_bad = (
                    wall_age is None
                    or not 0 <= wall_age
                    or not minimal_baseline
                    and wall_age >= wall_limit
                    or minimal_baseline
                    and not math.isfinite(wall_age)
                )
                # sim 源龄窗口为 [-0.05, sim_limit)：50 ms 未来容差。
                if wall_bad or sim_age is None or not -0.05 <= sim_age < sim_limit:
                    reasons.append(name + " missing/stale")
                if name in self.errors:
                    reasons.append(self.errors[name])
            # 图像与 CameraInfo 必须配到 50 ms 内；相机频率 <10 Hz 时配对窗口放宽到
            # 0.75 s sim / 2.0 s wall。
            pair = self.pairs.latest_fresh(
                ns,
                now,
                sim_limit=0.75 if actual_camera_rate < 10 else 0.3,
                wall_limit=2.0 if actual_camera_rate < 10 else 0.3,
                observe_wall=minimal_baseline,
            )
            if pair is None:
                reasons.append("Vehicle image and actual CameraInfo are not paired within 50ms")
            checked = self.graph_check.result
            graph = {} if checked is None or checked["data"] is None else checked["data"]
            # 图检查年龄用 started_wall（检查开始时刻），慢检查不会被当成新鲜。
            graph_age = None if checked is None else now - checked["started_wall"]
            if (
                minimal_baseline
                and graph_age is not None
                and math.isfinite(graph_age)
                and graph_age >= 0.3
            ):
                wall_observations.append(
                    {"field": "publisher_graph_wall_s", "age_s": graph_age, "original_limit_s": 0.3}
                )
            if (
                checked is None
                or checked["error"]
                or (not minimal_baseline and graph_age >= 0.3)
                or (minimal_baseline and (not math.isfinite(graph_age) or graph_age < 0))
            ):
                reasons.append("Actual SLAM publisher graph check missing/stale")
            for topic, value in graph.items():
                if not value["passed"]:
                    reasons.append("Unexpected actual SLAM publisher chain: " + topic)
            # 只有当前 tick 完全没有 reasons 才算健康；否则恢复段被重置。
            warmed = self.warmup.observe(not reasons, ns, self.stamps.get("body_odom"))
            if not warmed and not reasons:
                reasons.append(
                    "Waiting for fresh SLAM recovery samples"
                    if self.warmup.complete
                    else "Waiting for cumulative actual sensor SLAM initialization history"
                )
            # state 三态：有 failure → failed；warmed → ready；其余 → hold（fail-closed）。
            status = {
                "schema": 1,
                "mode": profile["controller_kind"],
                "state": "failed" if self.failure else "ready" if warmed else "hold",
                "ready": warmed,
                "reason": "; ".join(reasons)
                if reasons
                else "Actual vehicle RGB/K, IMU, LiDAR and sensor SLAM healthy",
                "monotonic_wall": now,
                "ros_sim_time_ns": ns,
                "good_since_sim_ns": self.warmup.good_since,
                "ages": ages,
                "initial_warmup_complete": self.warmup.complete,
                "recovery_fresh_pose_samples": self.warmup.samples,
                "initialization_history": self.warmup.history,
                "initialization_requirements": self.warmup.requirements,
                "clock_source": "single rclpy ROS TimeSource /clock, KEEP_LAST depth=1",
                "graph_check_started_wall": None if checked is None else checked["started_wall"],
                "graph_check_duration_wall_s": None
                if checked is None
                else checked["completed_wall"] - checked["started_wall"],
                "recovery_first_pose_stamp_ns": self.warmup.first_pose,
                "recovery_last_pose_stamp_ns": self.warmup.last_pose,
                "actual_image_camera_info_pair": pair,
                "counts": self.counts,
                "publisher_graph": graph,
                "ground_truth_navigation_used": False,
                "overview_used_for_vio": False,
                "camera_intrinsics_scale": "Original640x480 K; FAST-LIVO2 internally applies scale=0.5",
            }
            # minimal_baseline 分支只记录墙钟观测；enabled() 恒 False，当前不可达。
            if minimal_baseline:
                status["simulation_wall_age_observations"] = wall_observations
                if not reasons:
                    status["reason"] = (
                        "Simulation baseline: sensor structure/simulation-time valid; original wall ages observed"
                    )
            # 原子写当前状态、追加历史，并把同一份 status 发布到 sensor_health。
            self.evidence.atomic(run / "navigation_sensor_gate.json", status)
            self.evidence.append(run / "navigation_sensor_gate_history.jsonl", status)
            self.pub.publish(String(data=json.dumps(status, allow_nan=False)))
            self.last_report = status

    node = Gate()
    try:
        rclpy.spin(node)
    except _ShutdownRequested:
        pass
    finally:
        # 关停：写一次 hold 状态（覆盖原 ready 报告），再关闭后台检查与证据文件。
        shutdown_signals.begin_cleanup()
        if node.last_report:
            status = {
                **node.last_report,
                "ready": False,
                "state": "hold",
                "reason": "Sensor SLAM gate stopped",
                "monotonic_wall": time.monotonic(),
            }
            node.evidence.atomic(run / "navigation_sensor_gate.json", status)
        node.graph_check.close()
        node.evidence.close()
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    """用 ShutdownSignals 管理进程信号，进入 _main 并在退出时恢复原处理。"""
    with _ShutdownSignals() as shutdown_signals:
        _main(shutdown_signals)


if __name__ == "__main__":
    main()
