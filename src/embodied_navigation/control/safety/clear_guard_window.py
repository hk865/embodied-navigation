"""解除障碍保持前，必须已有真实的、新鲜的实际几何观测量。

职责：用已评估的 guard 时钟记录“连续无障碍”窗口，决定是否允许解除保持。
边界：只消费调用方给出的时钟与布尔结果，不评估点云、不做 I/O、不发布命令。
阈值（300 ms 采样间隔、1.35 s 尾窗、10 样本/1 s）均为本文件硬编码常量。
"""

from collections import deque


class ClearGuardWindow:
    """维护最近一次真实 guard 评估的时钟序列；非线程安全，由调用方串行化。"""

    def __init__(self):
        self.clocks = deque()

    def observe(self, clock_ns, blocked, evaluated):
        """喂入一次真实 guard 评估；只有完整解禁窗口才返回 True。

        参数 `clock_ns`：本次评估的整数时钟（ns，非负）；`blocked`：是否检测到
        障碍；`evaluated`：本帧是否真的做了几何评估。阻塞样本清空窗口；未评估
        样本直接返回 False 且不改窗口；已评估样本间隔超过 300 ms 会重新起窗。
        解禁条件：窗口内至少 10 个新鲜样本、时间跨度至少 1 s，且最近 1.35 s
        内没有阻塞样本。时钟非法或未前进时抛 `ValueError`。
        """
        if blocked:
            self.clocks.clear()
            return False
        if not evaluated:
            return False
        if type(clock_ns) is not int or clock_ns < 0:
            raise ValueError("Actual guard clock required")
        if self.clocks and clock_ns <= self.clocks[-1]:
            raise ValueError("Guard clock did not advance")
        # 300_000_000 ns = 300 ms：评估间隔超过它说明中间有空档，窗口重起。
        if self.clocks and clock_ns - self.clocks[-1] > 300_000_000:
            self.clocks.clear()
        self.clocks.append(clock_ns)
        # 1_350_000_000 ns = 1.35 s 尾窗：更早的样本不再参与“无阻塞”判定。
        while self.clocks and self.clocks[0] < clock_ns - 1_350_000_000:
            self.clocks.popleft()
        # 10 个样本 / 1 s 跨度是既有解禁阈值（硬编码），不是配置项。
        return len(self.clocks) >= 10 and self.clocks[-1] - self.clocks[0] >= 1_000_000_000
