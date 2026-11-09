"""有界、非阻塞、只传接触值的进程间通信；不含位置、姿态或速度。

职责：发送端 best-effort 投递定长报文，接收端只保留严格更新的最新样本。
边界：不落盘、不做几何或安全判定；邮箱缺失/满时丢样本而不是阻塞调用方。
"""

import errno
import math
import socket
import struct
import time

# 线格式固定为小端（"<" 前缀，无对齐填充）：stamp_ns(u64)、5 个接触标量(f64)、
# 故障标志(u64)。五个值只是接触计数/标志，不是位姿。收发表双方必须共用此布局。
PACKET = struct.Struct("<Q5dQ")


class ContactPublisher:
    """best-effort 数据报发送端；邮箱缺失或已满时直接丢弃该样本。"""

    def __init__(self, endpoint):
        """按 `endpoint` 建立非阻塞 `AF_UNIX`/`SOCK_DGRAM` 套接字；`dropped` 计数丢弃量。"""
        self.endpoint = str(endpoint)
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.dropped = 0

    def publish(self, stamp_ns, contacts, fault=False):
        """发送一个报文；可恢复的发送错误返回 False 并计入 `dropped`。

        `stamp_ns` 为整数时间戳（由调用方提供）；`contacts` 为 5 个标量；`fault`
        转为 0/1 的整型标志。只有预期的非阻塞邮箱条件（无监听者、连接被拒、
        would-block、缓冲区满）会被吞掉并计数；其他 `OSError` 原样抛出，避免
        真实的传输故障被隐藏。
        """
        packet = PACKET.pack(stamp_ns, *contacts, int(bool(fault)))
        try:
            self.socket.sendto(packet, self.endpoint)
            return True
        except OSError as error:
            if error.errno not in (errno.ENOENT, errno.ECONNREFUSED, errno.EAGAIN, errno.ENOBUFS):
                raise
            self.dropped += 1
            return False

    def close(self):
        """关闭本端套接字；不等待发送完成（数据报本身不保证送达）。"""
        self.socket.close()


class ContactMailbox:
    """只保留最新样本的接收端；`drain` 只接受严格更新的合法报文。"""

    def __init__(self, endpoint):
        """绑定 `endpoint`；初始状态为“无样本 + `fault=True`”，即未收到样本前视为故障。"""
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        # 8192 字节接收缓冲：容量有限，队列满时发送端会丢样本（best-effort）。
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
        self.socket.setblocking(False)
        self.socket.bind(str(endpoint))
        self.stamp_ns = -1
        self.receipt_wall_ns = 0
        self.contacts = (0.0,) * 5
        self.fault = True

    def drain(self):
        """一次最多消费 64 个排队报文，只保留最新的合法样本。

        `stamp_ns` 不前进（回退或重复）、报文长度不符、接触值非有限或为负的样本
        一律忽略，因此损坏样本不会把已有的好样本挤掉。新样本到达时会更新
        `receipt_wall_ns`（单调墙钟，ns）。
        """
        for _ in range(64):
            try:
                packet = self.socket.recv(PACKET.size + 1)
            except BlockingIOError:
                break
            if len(packet) != PACKET.size:
                continue
            stamp, *values = PACKET.unpack(packet)
            contacts, fault = values[:5], values[5]
            if stamp <= self.stamp_ns or not all(math.isfinite(v) and v >= 0 for v in contacts):
                continue
            self.stamp_ns, self.contacts, self.fault = stamp, tuple(contacts), bool(fault)
            self.receipt_wall_ns = time.monotonic_ns()

    def fresh(self, clock_ns):
        """要求无故障、仿真龄在 [-50, 300) ms 且墙钟收据龄在 [0, 300) ms。

        `clock_ns` 是调用方的仿真/ROS 时钟（ns）。允许 -50 ms 未来容差；两个时间窗
        都是硬编码常量，不是配置项。任一条件不满足即返回 False（调用方按过期处理）。
        """
        return (
            not self.fault
            and -50_000_000 <= clock_ns - self.stamp_ns < 300_000_000
            and 0 <= time.monotonic_ns() - self.receipt_wall_ns < 300_000_000
        )

    def close(self):
        self.socket.close()
