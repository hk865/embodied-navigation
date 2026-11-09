#!/usr/bin/env python3
"""Original IMU/RGB passthrough with process-local writer selection.

No application queue, payload copy, timestamp repair or sensor retirement.
Timing stays in bounded memory and is saved once on exit, not per message.
"""

# 中文模块说明（首轮补充；上方 docstring 恢复 HEAD 原文，以保持 --help/--doc 文案不变）：
# IMU/RGB 原样转发，并在进程内选择 writer 的 publication mode。
#
# 没有应用层队列、payload 拷贝、时间戳修复或传感器淘汰；时序数据只保存在有界内存里，
# 退出时保存一次，而不是每消息写盘。转发保留原始字段与 header 时间戳。
#
# 开关：--mode sync|async 只改两条输出 writer 的 publishMode；--run-dir 必需，用于
# 写 sensor_relay_loaded.json 与 sensor_relay_exit.json 证据。


import argparse
from collections import deque
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET


# 输入到输出的固定映射；只覆盖这两条 writer，不改输入 topic。
TOPICS = {"imu": "/demo/teacher/slam/imu", "image": "/demo/teacher/slam/image"}
# Fast DDS profile XML 的命名空间。
NS = "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"


def configure_transport(run, mode):
    """保留原 transport，只改本 relay 两个输出 topic 的 publication mode。

    参数 run 为本次 run 目录；mode 只接受 "sync"/"async"（由 argparse choices 保证）。
    从 FASTRTPS_DEFAULT_PROFILES_FILE 读原始 XML，追加两个 publisher profile，
    写到 run/sensor_relay_transport.xml，并设置本进程的三个 Fast DDS 环境变量。
    返回 transport 证据 dict（基线/新 XML 路径与 sha256）。
    失败：原 XML 缺 profiles 节点或已存在同名 profile → ValueError；目标文件已存在
    → FileExistsError（x 模式独占创建）。
    """
    # 原始 transport 文件路径来自环境变量；文件不存在时由 Path.read_bytes 抛错。
    original = Path(os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"]).resolve()
    raw = original.read_bytes()
    ET.register_namespace("", NS)
    tree = ET.fromstring(raw)
    profiles = tree.find("{%s}profiles" % NS)
    if profiles is None:
        raise ValueError("Missing original transport profiles")
    # 两个输出 topic 各追加一个 publisher profile；同名 profile 已存在说明调用方
    # 已配置过，拒绝继续。
    for topic in TOPICS.values():
        if any(p.get("profile_name") == topic for p in profiles):
            raise ValueError("Unexpected pre-existing relay writer profile")
        writer = ET.SubElement(profiles, "{%s}publisher" % NS, profile_name=topic)
        qos = ET.SubElement(writer, "{%s}qos" % NS)
        publish = ET.SubElement(qos, "{%s}publishMode" % NS)
        ET.SubElement(publish, "{%s}kind" % NS).text = {
            "sync": "SYNCHRONOUS",
            "async": "ASYNCHRONOUS",
        }[mode]
    # x 模式：同一 run 目录重复启动 relay 会失败，而不是覆盖已有证据。
    target = run / "sensor_relay_transport.xml"
    with target.open("xb") as stream:
        stream.write(ET.tostring(tree, encoding="utf-8", xml_declaration=True))
    # AUTO 让 topic XML 决定模式，同时保留 rmw 原有的 memory/data-sharing 默认值；
    # QoS-from-XML=1 会连这些一起覆盖，因此不能用。
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(target)
    os.environ["RMW_FASTRTPS_PUBLICATION_MODE"] = "AUTO"
    os.environ["RMW_FASTRTPS_USE_QOS_FROM_XML"] = "0"
    return dict(
        mode=mode,
        baseline=str(original),
        baseline_sha256=hashlib.sha256(raw).hexdigest(),
        xml=str(target),
        xml_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        reliability_history_depth_unchanged=True,
        application_queue_added=False,
    )


class PublishTiming:
    """每个 stream 的有界发布时序证据（不改变发布行为）。

    每行累计 entered/returned/errors 与 wall/CPU 总时长、最大 wall 时长；first 保存
    前 4 个样本、last 保存最近 4 个、slowest 保存 wall 最慢的样本。
    """

    def __init__(self):
        self.rows = {
            n: dict(
                entered=0,
                returned=0,
                errors=0,
                wall_total_ns=0,
                cpu_total_ns=0,
                wall_max_ns=0,
                first_source_ns=None,
                last_source_ns=None,
            )
            for n in TOPICS
        }
        self.first = {n: [] for n in TOPICS}
        # first/last/slowest 合计每 stream 最多 9 个样本（4+4+1），保证内存有界。
        self.last = {n: deque(maxlen=4) for n in TOPICS}
        self.slowest = {n: None for n in TOPICS}
        self.inflight = None

    def publish(self, name, msg, publisher, stamp):
        """发布一条消息并记录 wrapper 层面的时序（不重试、不复制 payload）。

        参数 name 为 "imu"/"image"，stamp 为原 header 纳秒；返回 None。
        publish 异常会先累加 errors 再原样抛出；finally 保证每个 enter 都有对应的
        return 记录。
        """
        row = self.rows[name]
        # wall 用 monotonic_ns，CPU 用 thread_time_ns（线程 CPU 时间，不含阻塞等待）。
        wall, cpu = time.monotonic_ns(), time.thread_time_ns()
        row["entered"] += 1
        if row["first_source_ns"] is None:
            row["first_source_ns"] = stamp
        row["last_source_ns"] = stamp
        # inflight 只在 publish 调用期间非 None，供状态查询当前卡在哪条流。
        self.inflight = (name, stamp, wall, cpu)
        returned = False
        try:
            publisher.publish(msg)
            row["returned"] += 1
            returned = True
        except BaseException:
            # 任何异常（含 BaseException）都先记账再重抛，不吞错。
            row["errors"] += 1
            raise
        finally:
            end_wall, end_cpu = time.monotonic_ns(), time.thread_time_ns()
            elapsed = end_wall - wall
            row["wall_total_ns"] += elapsed
            row["cpu_total_ns"] += end_cpu - cpu
            sample = (stamp, wall, end_wall, cpu, end_cpu, returned)
            # 用 >= 更新最慢样本：同值后来者会替换前者。
            if elapsed >= row["wall_max_ns"]:
                row["wall_max_ns"] = elapsed
                self.slowest[name] = sample
            if len(self.first[name]) < 4:
                self.first[name].append(sample)
            self.last[name].append(sample)
            self.inflight = None

    def snapshot(self):
        """返回可 JSON 序列化的时序快照；不改变任何状态。

        endpoint 说明测量边界是 relay publish wrapper 的 enter/return，不是 callback
        入口或 DDS take/ACK；bounded_samples_per_stream=9 与 hot_path_IO=False 是
        固定证据字段。
        """
        return dict(
            rows=self.rows,
            first=self.first,
            last={n: list(v) for n, v in self.last.items()},
            slowest=self.slowest,
            inflight=self.inflight,
            sample_fields=[
                "source_ns",
                "publish_wrapper_enter_monotonic_ns",
                "publish_return_monotonic_ns",
                "enter_thread_cpu_ns",
                "return_thread_cpu_ns",
                "returned",
            ],
            endpoint="relay publish wrapper enter / publish return; not callback entry or DDS take/ACK",
            bounded_samples_per_stream=9,
            hot_path_IO=False,
        )


def make_relay(Node, Image, Imu, String, sensor_qos):
    """构造 Relay 节点类；ROS 类型由 main 在 import 后注入。

    参数 Node/Image/Imu/String 为 rclpy 与 sensor_msgs/std_msgs 类型；
    sensor_qos 为订阅用 QoS profile（main 传 qos_profile_sensor_data）。
    返回 Relay 类；实例化后订阅 /livox/imu 与 /demo/camera，发布到 TOPICS。
    """

    class Relay(Node):
        """把实际 IMU/RGB 原样转发到 Teacher SLAM 输入 topic 的节点。"""

        def __init__(self):
            """建立 2 路订阅/发布、状态 topic 与 1 Hz 上报定时器。"""
            super().__init__("teacher_slam_actual_sensor_relay")
            self.counts = {"imu": 0, "image": 0}
            self.rejected = {"imu": 0, "image": 0}
            self.stamps, self.walls, self.pubs = {}, {}, {}
            self.timing = PublishTiming()
            # 输入是实际传感器 topic，输出是 TOPICS 里的 Teacher SLAM 输入；发布
            # depth 10 与订阅 sensor_qos 保持原样。
            for name, typ, src in [("imu", Imu, "/livox/imu"), ("image", Image, "/demo/camera")]:
                self.pubs[name] = self.create_publisher(typ, TOPICS[name], 10)
                self.create_subscription(typ, src, lambda m, n=name: self.receive(n, m), sensor_qos)
            self.status = self.create_publisher(String, "/demo/teacher/slam/relay_status", 10)
            # 1 Hz 状态上报，不阻塞转发。
            self.create_timer(1.0, self.report)

        def receive(self, name, msg):
            """转发一条 IMU 或图像消息。

            只接受 header stamp 严格递增的消息；时间戳不递增（重复/回退）时累加
            rejected 并丢弃，不改写 payload 或时间戳。publish 异常向上抛。
            """
            # 用原始 header 组成纳秒时间戳；不做跨时钟域换算。
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if stamp <= self.stamps.get(name, -1):
                self.rejected[name] += 1
                return
            self.timing.publish(name, msg, self.pubs[name], stamp)
            # 以下字段沿用原始 publication-return 语义：publish 返回后才记为最新。
            self.stamps[name] = stamp
            self.walls[name] = time.monotonic()
            self.counts[name] += 1

        def report(self):
            """发布 1 Hz JSON 状态：计数、最近时间戳/墙钟年龄与发布时序。"""
            self.status.publish(
                String(
                    data=json.dumps(
                        # wall_age 为单调墙钟接收年龄（秒）；
                        # overview_used/ground_truth_used 固定为 False。
                        dict(
                            source="actual Teacher sensor messages",
                            counts=self.counts,
                            rejected=self.rejected,
                            stamp_ns=self.stamps,
                            wall_age={n: time.monotonic() - v for n, v in self.walls.items()},
                            field_payload_and_stamp_unchanged=True,
                            overview_used=False,
                            ground_truth_used=False,
                            publish_timing=self.timing.rows,
                            publish_inflight=self.timing.inflight,
                        )
                    )
                )
            )

    return Relay


def main():
    """解析 --run-dir/--mode，配置 transport 后启动 relay 并 spin。

    configure_transport 必须在 import rclpy 与 rclpy.init 之前完成，才能让本进程的
    Fast DDS 读取新 XML。启动时读一次实际 writer QoS 写
    sensor_relay_loaded.json，退出时写 sensor_relay_exit.json；证据文件写失败只打
    stderr，不阻止运行。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--mode", choices=["sync", "async"], required=True)
    args, ros = parser.parse_known_args()
    run = args.run_dir.resolve()
    # 先配置 transport 再导入/初始化 rclpy，确保环境变量在 participant 创建前生效。
    transport = configure_transport(run, args.mode)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, Imu
    from std_msgs.msg import String

    rclpy.init(args=ros)
    node = make_relay(Node, Image, Imu, String, qos_profile_sensor_data)()
    writer_qos = {}
    try:
        # 启动时复用既有只读 ABI 助手读一次实际 writer QoS；这是证据，不是新的
        # 控制/传感器门，也不逐消息校验。
        try:
            from embodied_navigation.perception.sensors.async_writers import read_writer

            writer_qos = {n: read_writer(p) for n, p in dict(node.pubs, status=node.status).items()}
        except Exception as error:
            # 读失败只记 read_error 作为启动证据，不让 relay 退出。
            writer_qos = dict(read_error=type(error).__name__ + ":" + str(error))
        # 证据文件独占创建（x）；写失败只打 stderr，不阻止 relay 运行。
        try:
            with (run / "sensor_relay_loaded.json").open("x") as stream:
                json.dump(
                    dict(
                        pid=os.getpid(),
                        source=str(Path(__file__).resolve()),
                        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                        transport=transport,
                        actual_writers=writer_qos,
                        ground_truth_used=False,
                    ),
                    stream,
                    allow_nan=False,
                )
        except OSError as error:
            print("Relay startup evidence unavailable: " + str(error), file=sys.stderr)
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # 退出时只写一个有界快照；缺失的退出证据保持未知（不推断）。
        try:
            with (run / "sensor_relay_exit.json").open("x") as stream:
                json.dump(
                    dict(
                        pid=os.getpid(),
                        counts=node.counts,
                        rejected=node.rejected,
                        stamp_ns=node.stamps,
                        publish_timing=node.timing.snapshot(),
                        actual_writers=writer_qos,
                        transport=transport,
                    ),
                    stream,
                    allow_nan=False,
                )
        except OSError as error:
            print("Relay exit evidence unavailable: " + str(error), file=sys.stderr)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
