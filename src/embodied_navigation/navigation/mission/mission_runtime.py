"""Publish the current global goal/reference after live localization is ready.

There is no experiment score, evidence capture, archived source or PASS output.
"""

# 中文模块说明（首轮补充；上方 docstring 恢复 HEAD 原文，以保持 --help/--doc 文案不变）：
# 在实时定位就绪后发布当前全局目标与参考。
#
# 职责：以 ``mission`` 角色运行的 ROS 节点。等 SLAM 位姿、IMU 与
# localization/SCAN epoch/传感器健康三份状态同时就绪后，在测得静止时冻结一个
# schema-2 目标，原子写出 ``navigation_anchor.json`` 与
# ``navigation_request.json``，再以 1 Hz 发布请求与瞬态 anchor；当
# ``(planner_epoch, pid, localization_generation)`` 变化时重写
# ``global_reference.json``。
#
# 边界：不做实验评分、不采集证据、不归档源码、不输出 PASS；不产生速度命令，
# 也不参与准入与控制。rclpy 与消息类型在 ``main()`` 内延迟导入，import 本模块
# 不会创建节点。进程角色由 ``control/launch/components.py`` 以 ``mission`` 启动。


from pathlib import Path
import argparse, copy, json, math, time
import numpy as np
from embodied_navigation.common.runtime_context import load
from embodied_navigation.common.config import ROOT


def atomic(path, value):
    """以同目录临时文件 + ``replace`` 原子写出 JSON（``allow_nan=False``）。

    写的是 run 目录下的运行产物而不是日志；失败向上抛异常，不会留下半个目标文件。
    """
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, allow_nan=False) + "\n")
    temporary.replace(path)


def main():
    """进程入口：解析 ``--run``，读取本次 run 的 profile 与路线，启动节点。

    ``--run``（必填）指向 run 目录；读取其中的 ``runtime_context.json``（只取
    ``profile``）与 ``global_route.json``。``body_points`` 必须是 (N,3)、N>=2
    的有限数组，否则抛 ``ValueError``。未识别参数原样交给 ``rclpy.init``；
    节点类定义在函数内，保证 import 本模块不触发 ROS 初始化。
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String, Bool
    from embodied_navigation.common.runtime_io import latest_sensor_qos

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    args, ros = parser.parse_known_args()
    directory = args.run.resolve()
    profile = load(directory / "runtime_context.json")["profile"]
    route = json.loads((directory / "global_route.json").read_text())
    # global_route.json 是 prepare 为本 run 生成的路线（源 assets/geometry/route.json）。
    points = np.asarray(route["body_points"], float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError("Invalid global route geometry")

    class Mission(Node):
        """``teacher_navigation_request`` 节点：门控、冻结目标、发布请求与参考。

        发布：``/demo/navigation/request``（String JSON，depth 10）、
        ``/demo/teacher/navigation/anchor``（TRANSIENT_LOCAL，depth 1，后加入者
        仍能取到最后一帧）、``/demo/navigation/stop``（Bool，退出时发 True）。
        订阅：SLAM body odom、IMU，以及 localization/epoch/health 三份 JSON 状态。
        10 Hz 稳态定时器（``ClockType.STEADY_TIME``）驱动 ``tick``；
        ``use_sim_time`` 为 True，节点时钟走仿真时间，而新鲜度判定另外混用墙钟。
        """

        def __init__(self):
            super().__init__("teacher_navigation_request")
            # use_sim_time=True：节点 get_clock() 返回仿真/ROS 时间。
            self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
            self.pose = None
            self.imu = None
            self.localization = None
            self.epoch = None
            self.health = None
            self.received = {}
            self.initial_samples = []
            self.request = None
            self.anchor = None
            self.reference_scope = None
            self.last_emit = -math.inf
            # 请求 id 固定为 "<run 目录名>:navigation:1"，本次运行只发布一个目标。
            self.request_id = directory.name + ":navigation:1"
            self.request_pub = self.create_publisher(String, "/demo/navigation/request", 10)
            # anchor 用 TRANSIENT_LOCAL depth=1：只保留最后一帧，晚加入者仍能取到。
            self.anchor_pub = self.create_publisher(
                String,
                "/demo/teacher/navigation/anchor",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            self.stop_pub = self.create_publisher(Bool, "/demo/navigation/stop", 10)
            # 位姿用 BEST_EFFORT depth 10（传感器流允许丢帧）；IMU 用共享的最新样本 QoS。
            self.create_subscription(
                Odometry,
                "/demo/slam/body_odom",
                self.on_pose,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.create_subscription(Imu, "/livox/imu", self.on_imu, latest_sensor_qos())
            # 三份状态都以 String(JSON) 上报，统一按 role 记录收到时间。
            for role, topic in [
                ("localization", "/demo/navigation/localization_status"),
                ("epoch", "/demo/navigation/scan_epoch_ready"),
                ("health", "/demo/teacher/navigation/sensor_health"),
            ]:
                self.create_subscription(
                    String, topic, lambda message, k=role: self.on_json(k, message), 10
                )
            # 10 Hz 稳态墙钟定时器：不受仿真时间暂停或快慢影响。
            self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def on_json(self, role, message):
            """按 role 记录一份 JSON 状态文档（localization/epoch/health）。

            解析失败静默丢弃该帧（不更新新鲜度）；成功时记录墙钟
            ``time.monotonic()``，供 ``ready`` 的 0.3 s 新鲜度判定。
            """
            try:
                data = json.loads(message.data)
            except (ValueError, TypeError):
                return
            setattr(self, role, data)
            self.received[role] = time.monotonic()

        def on_imu(self, message):
            """保留最新 IMU 样本及其接收墙钟时间（``time.monotonic()``，秒）。"""
            self.imu = (message, time.monotonic())

        def on_pose(self, message):
            """保留一帧有限数值的 SLAM body 位姿及接收墙钟时间。

            ``stamp_ns`` 为消息头时间（ns）；位置单位 m、四元数顺序 xyzw、
            速度单位 m/s。任一分量非有限即丢弃该帧。记录中的
            ``navigation_ground_truth_used=False`` 是本节点只消费实测 SLAM
            反馈的来源标签。
            """
            ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            p = message.pose.pose.position
            q = message.pose.pose.orientation
            v = message.twist.twist.linear
            value = dict(
                stamp_ns=ns,
                position=[p.x, p.y, p.z],
                quaternion_xyzw=[q.x, q.y, q.z, q.w],
                body_velocity=[v.x, v.y, v.z],
                frame_id=message.header.frame_id,
                child_frame_id=message.child_frame_id,
                source_topic="/demo/slam/body_odom",
                navigation_ground_truth_used=False,
            )
            if not np.isfinite(
                value["position"] + value["quaternion_xyzw"] + value["body_velocity"]
            ).all():
                return
            self.pose = (value, time.monotonic())

        def ready(self, clock, now):
            """门控实时位姿/IMU 与三份状态回执的新鲜度。

            位姿与 IMU 时间戳必须在节点时钟当前时刻之前 300 ms 内
            （``300_000_000`` ns，硬编码）；两帧样本与三份状态回执的接收墙钟龄
            必须小于 0.3 s（硬编码）。定位须已 initialized 且
            ``correction_transition`` 为假；epoch 须 ready 且
            ``localization_generation`` 与定位 generation 一致；健康状态须 ready。
            全部满足才返回 True，否则 False（不报错、不等待）。
            """
            if self.pose is None or self.imu is None:
                return False
            # 三份状态回执的墙钟龄都必须小于 0.3 s（硬编码）。
            if any(
                not 0 <= now - self.received.get(role, -math.inf) < 0.3
                for role in ["localization", "epoch", "health"]
            ):
                return False
            source, received = self.pose
            imu, imu_wall = self.imu
            stamp = imu.header.stamp.sec * 1_000_000_000 + imu.header.stamp.nanosec
            # 300 ms 因果窗口：样本时间戳不得超前于节点时钟，也不能过期。
            if not (
                0 <= clock - source["stamp_ns"] <= 300_000_000
                and 0 <= clock - stamp <= 300_000_000
                and 0 <= now - received < 0.3
                and 0 <= now - imu_wall < 0.3
            ):
                return False
            return (
                self.localization.get("initialized") is True
                and self.localization.get("correction_transition") is False
                and self.epoch.get("ready") is True
                and self.epoch.get("localization_generation") == self.localization.get("generation")
                and self.health.get("ready") is True
            )

        def initialize_goal(self, clock, now):
            """在测得机体静止后推导并冻结目标（每次运行只做一次）。

            静止判据：机体速度 <= 0.03 m/s 且角速度 <= 0.05 rad/s（均硬编码）；
            需连续 >= 10 帧、首尾时间戳跨度 >= 0.8 s。阈值不满足会清空样本重计。
            目标几何来自 run 的 ``profile["mission"]``（由 configs 合并并 pin 进
            ``runtime_context.json``）：中心 = 当前实测位置 +
            ``relative_offset_xyz``（m），到达区与停止带均为 disc_prism；
            ``timeout_sim_s`` 等一并冻结。随后原子替换
            ``navigation_anchor.json``（schema=1）与
            ``navigation_request.json``（schema_version=2），请求此后不再改变。
            """
            value = self.pose[0]
            imu = self.imu[0]
            # 硬编码静止阈值 0.03 m/s / 0.05 rad/s；任一超限即清空累计。
            if (
                np.linalg.norm(value["body_velocity"]) > 0.03
                or np.linalg.norm(
                    [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]
                )
                > 0.05
            ):
                self.initial_samples = []
                return
            if not self.initial_samples or value["stamp_ns"] > self.initial_samples[-1]["stamp_ns"]:
                self.initial_samples.append(copy.deepcopy(value))
            # 只保留最近 32 帧；至少 10 帧且首尾跨度 >= 0.8 s 才认为稳定。
            self.initial_samples = self.initial_samples[-32:]
            if (
                len(self.initial_samples) < 10
                or self.initial_samples[-1]["stamp_ns"] - self.initial_samples[0]["stamp_ns"]
                < 800_000_000
            ):
                return
            # profile["mission"] 的键由 configs/launch/settings.json 提供（prepare pin）。
            cfg = profile["mission"]
            origin = value["position"]
            goal = dict(
                goal_id=cfg["goal_id"],
                center=(np.asarray(origin) + cfg["relative_offset_xyz"]).tolist(),
                arrival=dict(
                    type="disc_prism",
                    axes=np.eye(3).tolist(),
                    radius_m=cfg["arrival_radius_m"],
                    height_half_span_m=cfg["arrival_height_half_span_m"],
                    dwell_sim_s=cfg["dwell_sim_s"],
                    control_band=dict(
                        type="disc_prism",
                        axes=np.eye(3).tolist(),
                        radius_m=cfg["control_radius_m"],
                        height_half_span_m=cfg["arrival_height_half_span_m"],
                        dwell_sim_s=cfg["dwell_sim_s"],
                    ),
                ),
                timeout_sim_s=cfg["timeout_sim_s"],
                route_surface_id=cfg["surface_id"],
                legacy=False,
            )
            self.goal = goal
            self.anchor = dict(
                schema=1,
                experiment="navigation",
                run_dir=str(directory),
                origin=origin,
                yaw=0.0,
                pose_stamp_ns=value["stamp_ns"],
                frame_id="camera_init",
                source="/demo/slam/body_odom",
                ground_truth_navigation_used=False,
                initial_actual_slam_source=value,
                registered_route_camera_init_xyz=[origin] + route["body_points"],
                T_navigation_world=route["T_navigation_world"],
            )
            self.request = dict(
                schema_version=2, request_id=self.request_id, frame_id="camera_init", goals=[goal]
            )
            # 两份文件各自原子替换；内容互不依赖，均可被下游独立读取。
            atomic(directory / "navigation_anchor.json", self.anchor)
            atomic(directory / "navigation_request.json", self.request)

        def update_reference(self):
            """按 ``(planner_epoch, pid, localization_generation)`` 每 scope 只发一次参考。

            epoch/pid/generation 必须都是 >=1 的 int，否则本轮跳过。文档以
            ``r85_pct_scan_global_reference/v1`` schema 原子写入
            ``global_reference.json``，并由 ``attach_reference_heading(profile,
            document)`` 附加终端航向契约。``physical_body_ground_offset_m``（0.32）
            与 ``scan_transport_offset_m``（0.4）在本文件硬编码（单位 m）；
            代码与现有文档未说明其标定来源。
            """
            epoch = self.epoch.get("planner_epoch")
            generation = self.localization.get("generation")
            pid = (self.epoch.get("planner_identity") or {}).get("pid")
            if any(type(v) is not int or v < 1 for v in [epoch, generation, pid]):
                return
            # 同一个 epoch/pid/generation 内参考保持不变；scope 变化才重写。
            scope = (epoch, pid, generation)
            if scope == self.reference_scope:
                return
            # 两个物理偏移是硬编码常量（m），不是从 configs 读取。
            document = dict(
                schema="r85_pct_scan_global_reference/v1",
                run_id=directory.name,
                frame_id="camera_init",
                epoch=epoch,
                localization_generation=generation,
                reference_id=self.request_id
                + ":epoch:"
                + str(epoch)
                + ":generation:"
                + str(generation),
                body_points=route["body_points"],
                ground_points=route["ground_points"],
                source_route_file=str(ROOT / "assets/geometry/route.json"),
                physical_body_ground_offset_m=0.32,
                scan_transport_offset_m=0.4,
                navigation_ground_truth_used=False,
                physical_sources=route["physical_sources"],
                T_navigation_world=route["T_navigation_world"],
                endpoint_support_connectors=route["endpoint_support_connectors"],
                task_goal_body_xyz=self.goal["center"],
                original_F3_goal_body_xyz=self.goal["center"],
                reference_scope="full_route",
            )
            from embodied_navigation.control.parking.terminal_heading_contract import (
                attach_reference_heading,
            )

            document = attach_reference_heading(profile, document)
            atomic(directory / "global_reference.json", document)
            self.reference_scope = scope

        def tick(self):
            """一个控制步：先过新鲜度门，再冻结目标、更新参考并限频发布。

            未就绪直接返回；``request`` 尚未冻结时尝试 ``initialize_goal``，
            仍未冻结则跳过发布。``anchor``/``request`` 以 1 Hz（硬编码 1.0 s）
            发布，避免重复刷屏并为迟到订阅者保活。
            """
            now = time.monotonic()
            clock = self.get_clock().now().nanoseconds
            if not self.ready(clock, now):
                return
            if self.request is None:
                self.initialize_goal(clock, now)
            if self.request is None:
                return
            self.update_reference()
            # 1 Hz 重复发布：两个 JSON 都是幂等快照，只为迟到订阅者保活。
            if now - self.last_emit >= 1.0:
                self.anchor_pub.publish(String(data=json.dumps(self.anchor)))
                self.request_pub.publish(String(data=json.dumps(self.request)))
                self.last_emit = now

    rclpy.init(args=ros)
    node = Mission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            # 退出时向控制端发停止请求；节点随后销毁，不再发布任何参考。
            node.stop_pub.publish(Bool(data=True))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
