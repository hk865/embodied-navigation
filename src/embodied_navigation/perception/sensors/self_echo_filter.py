#!/usr/bin/env python3
"""只从实际传感器扫描中剔除 Go2 已知的物理包络。

变换使用固定的 URDF/scenario LiDAR 外参；不读 SLAM 位姿、世界轨迹或真值。保留下来的
记录保留每个原始字段字节与原始采集时间戳。节点把 /livox/lidar 过滤后发到
/demo/slam/lidar_filtered，并逐帧发布身份证明；不写文件。

边界：只做几何过滤与身份证明，不做点云配准、定位或控制；包络是固定的机体尺寸，
不是动态障碍物判断。
"""

from embodied_navigation.project import project_root as _project_root
import argparse
import sys
import os
import uuid
import base64
import hashlib
from collections import OrderedDict
from embodied_navigation.localization_mapping.point_provenance import message_sha, decode, MAX_META
from collections import Counter
import copy
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from embodied_navigation.localization_mapping.geometry import sensor_transform


def self_mask(points_body):
    """返回机体坐标系中的点是否落在 Go2 已知物理包络内的布尔掩码。

    参数 points_body：形状 (N,3) 的机体系坐标，单位米。
    返回：形状 (N,) 的 bool 掩码，躯干与摆动腿两段包络取并集。
    阈值均为硬编码：躯干 |x|<=0.30、|y|<=0.23、z∈[-0.10,0.20]；摆动腿
    |x|<=0.45、|y|<=0.28、z∈[-0.36,-0.07]（米）。代码与现有文档未说明标定方法。
    """
    x, y, z = points_body.T
    trunk = (abs(x) <= 0.30) & (abs(y) <= 0.23) & (z >= -0.10) & (z <= 0.20)
    moving_legs = (abs(x) <= 0.45) & (abs(y) <= 0.28) & (z >= -0.36) & (z <= -0.07)
    return trunk | moving_legs


def filter_records(msg, lidar, with_identity=False):
    """剔除一帧 PointCloud2 中落在自车包络内的点，返回原始字节视图。

    参数：
        msg：输入 PointCloud2；要求 x/y/z 为 count=1 的 float32 标量字段。
        lidar：scenario 的 sensors.lidar 字典，提供固定 frame 与传感器外参。
        with_identity：为 True 时额外返回保留点在原云中的序号（little-endian u4）。

    返回：(payload bytes, removed_count, total_count[, ordinals])；payload 直接由
    原始点字节组成，未重排或改写字段。
    失败：XYZ 字段缺失、类型/计数不符，字段偏移越界，或 row_step/height 与 data
    长度不自洽时抛 ValueError。
    非有限点不参与包络判定，也不会被删除，因此会原样保留。
    """
    fields = {f.name: f for f in msg.fields}
    if any(
        name not in fields or fields[name].datatype != 7 or fields[name].count != 1
        for name in ("x", "y", "z")
    ):
        raise ValueError("sensor cloud requires scalar float32 XYZ")
    offsets = [fields[name].offset for name in ("x", "y", "z")]
    if (
        any(offset < 0 or offset + 4 > msg.point_step for offset in offsets)
        or msg.row_step < msg.width * msg.point_step
        or len(msg.data) < msg.height * msg.row_step
    ):
        raise ValueError("malformed sensor cloud field/row layout")
    # 按原始字段偏移构造结构化 dtype；itemsize 取 point_step，保持每点原始字节宽度。
    endian = ">" if msg.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [endian + "f4"] * 3,
            "offsets": offsets,
            "itemsize": msg.point_step,
        }
    )
    # 以 row_step/point_step 为步长零拷贝读取，再展平；strides 不把行间 padding 当点。
    values = np.ndarray(
        (msg.height, msg.width),
        dtype=dtype,
        buffer=msg.data,
        strides=(msg.row_step, msg.point_step),
    ).reshape(-1)
    xyz = np.column_stack([values[name] for name in ("x", "y", "z")])
    # 固定外参来自 scenario/URDF，不读 SLAM 位姿或世界轨迹。
    offset, rotation = sensor_transform(lidar)
    # 非有限点单独屏蔽：既不参与包络判定，也不会被删除。
    finite = np.isfinite(xyz).all(axis=1)
    body = np.full(xyz.shape, np.nan, dtype=float)
    body[finite] = xyz[finite] @ rotation.T + offset
    removed = self_mask(body) & finite
    # 同一掩码作用在原始 point_step 宽度的字节视图上，保证保留点逐字节不变。
    raw = np.ndarray(
        (msg.height, msg.width),
        dtype=np.dtype(f"V{msg.point_step}"),
        buffer=msg.data,
        strides=(msg.row_step, msg.point_step),
    ).reshape(-1)
    result = (raw[~removed].tobytes(), int(removed.sum()), len(raw))
    return (*result, np.flatnonzero(~removed).astype("<u4")) if with_identity else result


class SelfEchoFilter(Node):
    """实际 LiDAR 自车点剔除节点。

    输入 /livox/lidar，输出 /demo/slam/lidar_filtered；另发 1 Hz 状态
    /demo/slam/lidar_filter_status 与逐帧身份 /demo/slam/raw_point_identity。
    """

    def __init__(self, scenario):
        """从 scenario JSON 读取固定 LiDAR 配置并建立发布/订阅。

        scenario 为 --scenario 传入的 JSON 路径；生产 launch 传
        run/navigation_scenario.json。不读文件以外的外部状态。
        """
        super().__init__("demo_slam_self_echo_filter")
        self.lidar = json.loads(Path(scenario).read_text())["sensors"]["lidar"]
        # FAST-LIVO2 请求 reliable 交付；若这里用 sensor-data 的 best-effort
        # publisher，LiDAR 订阅端会因 QoS 不兼容而静默收不到数据。
        self.pub = self.create_publisher(PointCloud2, "/demo/slam/lidar_filtered", 10)
        self.identity_pub = self.create_publisher(String, "/demo/slam/raw_point_identity", 10)
        # 每个进程启动时生成一次 epoch，用于身份证明区分重启。
        self.identity_epoch = uuid.uuid4().hex
        self.status = self.create_publisher(String, "/demo/slam/lidar_filter_status", 10)
        # 输入用 sensor-data best-effort；该 QoS 配置适用于高频传感器流。
        self.create_subscription(PointCloud2, "/livox/lidar", self.receive, qos_profile_sensor_data)
        self.counts = Counter()
        self.last_stamp = None
        self.last_wall = None
        self.error = None
        # 1 Hz 状态上报走独立 topic，不阻塞过滤路径。
        self.create_timer(1.0, self.report)

    def receive(self, msg):
        """过滤一帧实际 LiDAR 并发布过滤结果与身份证明。

        返回 None。失败路径（frame 不匹配、点数/字节超上限、字段格式非法）只累加
        对应 rejected_* 计数并记录 self.error，不发布；成功时更新计数与时间戳。
        """
        # frame 必须与固定物理外参一致；不一致说明上游不是同一传感器/坐标系。
        if msg.header.frame_id != self.lidar["frame"]:
            self.counts["rejected_frame"] += 1
            self.error = "LiDAR frame does not match fixed physical extrinsic"
            return
        # R60 原始完整点云上限（硬编码）：点数 65536、data 2 MiB。
        if msg.height * msg.width > 65536 or len(msg.data) > 2 * 1024 * 1024:
            self.counts["rejected_capacity"] += 1
            self.error = "R60 original complete cloud exceeds source cap"
            return
        try:
            payload, removed, total, ordinals = filter_records(msg, self.lidar, with_identity=True)
        except ValueError as error:
            self.counts["rejected_format"] += 1
            self.error = str(error)
            return
        # 深拷贝保留原 header/字段/时间戳，只替换 data/height/width/row_step。
        out = copy.deepcopy(msg)
        out.data = payload
        out.height = 1
        out.width = total - removed
        out.row_step = out.width * out.point_step
        # R60 原始源点数上限；超过时直接抛错让节点失败，而不是静默截断。
        if total > 65536 or out.width > 65536:
            raise ValueError("R60 raw source count cap")
        ids = ordinals.tobytes()
        # 身份证明：producer epoch/pid、run 名（来自 DEMO_RUN_DIR）、原/适配消息
        # sha256、保留点序号，以及未使用真值的声明。
        proof = dict(
            schema="raw_filtered_point_identity/v1",
            fingerprint_model="go2_ros_semantic/v1",
            producer_epoch=self.identity_epoch,
            producer_pid=os.getpid(),
            run_id=Path(os.environ["DEMO_RUN_DIR"]).name,
            stamp_ns=int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec),
            raw_message_sha256=message_sha(msg),
            adapted_message_sha256=message_sha(out),
            raw_count=total,
            adapted_count=int(out.width),
            raw_ordinals_sha256=hashlib.sha256(ids).hexdigest(),
            raw_ordinals_base64=base64.b64encode(ids).decode("ascii"),
            navigation_ground_truth_used=False,
        )
        # 先发身份证明再发点云，下游可把过滤结果与同一帧证明配对。
        self.identity_pub.publish(String(data=json.dumps(proof, allow_nan=False)))
        self.pub.publish(out)
        self.counts["scans"] += 1
        self.counts["input_points"] += total
        self.counts["removed_self_points"] += removed
        # header 时间戳换算成秒（时钟域由上游决定，本节点不判断 sim/wall）；
        # last_wall 是单调墙钟接收时刻，只用于状态里的 age。
        self.last_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_wall = time.monotonic()
        self.error = None

    def report(self):
        """发布 1 Hz JSON 状态：计数、最近时间戳/墙钟年龄与当前错误。

        age 为单调墙钟年龄（秒），last_wall 为 None 时为 None；不修改过滤状态。
        """
        self.status.publish(
            String(
                data=json.dumps(
                    {
                        "input": "/livox/lidar",
                        "output": "/demo/slam/lidar_filtered",
                        "transform": "fixed sensor-to-body physical extrinsic",
                        "counts": dict(self.counts),
                        "last_stamp": self.last_stamp,
                        "age": None
                        if self.last_wall is None
                        else time.monotonic() - self.last_wall,
                        "ground_truth_used": False,
                        "slam_pose_used": False,
                        "error": self.error,
                    }
                )
            )
        )


def main():
    """解析参数、初始化 ROS 并 spin；收到关停信号时正常退出。

    --scenario 默认 configs/simulation/scenario.json；生产 launch 会显式传
    run/navigation_scenario.json。未知参数原样交给 rclpy.init。
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario", default=str(_project_root() / "configs/simulation/scenario.json")
    )
    args, ros = parser.parse_known_args()
    rclpy.init(args=ros)
    node = SelfEchoFilter(args.scenario)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except _ros.RCLError:
        # SIGINT 可能使 context 在 executor wait-set 之间失效；若 context 仍可用，
        # 其它错误必须让这个必需节点失败。
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
