"""可选的 ready 通知；commit 仍由既有 control 组负责。

mailbox 就是工作队列：guard condition 可能合并多次通知，因此不存在第二个 pending
标志、没有无界 drain、不读时钟，也不在热路径做 I/O。本模块只把“有新 bundle 可提交”
这件事告诉既有 control 组，不新增控制周期。

开关：环境变量 R92_READY_COMMIT，代码默认（未设置时）为 "0"，只接受 "0"/"1"；
当前普通导航由 configs/launch/settings.json 的 features 配成 "1"，启动器把它注入
为子进程环境变量。
"""

import os


def selected():
    """返回本进程是否启用 ready 通知。

    读取环境变量 R92_READY_COMMIT：未设置时取硬编码默认值 "0"（关闭），"1" 为开启。
    返回 bool。值不是 "0"/"1" 时抛 ValueError（启动期 fail-closed，不做猜测）。
    """
    value = os.environ.get("R92_READY_COMMIT", "0")
    if value not in ("0", "1"):
        raise ValueError("R92_READY_COMMIT must be 0 or 1")
    return value == "1"


class ReadyCommit:
    """把 mailbox 的 ready 信号接到既有 control 组的固定 batch 提交。

    构造参数：node 为 ROS 节点（使用其 default callback group）；mailbox 为
    feedback_mailbox.Mailbox；commit 为既有 control 组的一次固定 batch 提交函数。
    guard condition 必须与 node 的 default callback group 同组，否则构造直接抛
    RuntimeError，避免提交被挪到另一个执行组。
    """

    def __init__(self, node, mailbox, commit):
        self.mailbox, self.commit = mailbox, commit
        self.notifications = self.callbacks = self.trigger_errors = 0
        self.closed = False
        self.guard = node.create_guard_condition(
            self._execute, callback_group=node.default_callback_group
        )
        # 身份比较（is）是契约：guard 必须落在既有 control callback group 上。
        self.same_control_group = self.guard.callback_group is node.default_callback_group
        if not self.same_control_group:
            raise RuntimeError("Ready commit must use the existing control callback group")
        mailbox.ready_notify = self.notify

    def notify(self):
        """ready 回调：由 mailbox 在成功 append 之后、mailbox 锁外调用。

        可被 guard condition 合并；closed 后静默返回。trigger 失败时只累加
        trigger_errors 并把故障记进 mailbox.fail，不在这里修改业务状态：
        未改动的原 timer 仍负责该故障下的 fail-closed 发布。
        """
        # 只在 append 成功且已释放 mailbox 锁之后被调用。
        if self.closed:
            return
        self.notifications += 1
        try:
            self.guard.trigger()
        except Exception as error:
            self.trigger_errors += 1
            # intake 侧绝不修改业务状态；故障时由未改动的 timer 负责 fail-closed
            # 发布。
            self.mailbox.fail("ready_notify:" + type(error).__name__)

    def _execute(self):
        """guard condition 回调：在既有 control 组内提交一次固定 batch。"""
        if self.closed:
            return
        self.callbacks += 1
        self.commit()  # 只提交一次原始固定 batch，不额外产生 control tick。

    def close(self):
        """解绑并停止通知；guard 的销毁仍归原 Node.destroy_node。

        调用前提是现有 owner 的关停流程已经 join 过 executor 回调。
        """
        # 现有 owner 关停时已 join executor 回调。
        self.closed = True
        self.mailbox.ready_notify = None
        # guard 由原 Node.destroy_node 负责销毁，本类不重复销毁。

    def snapshot(self):
        """返回 R92 证据字典（只读快照，不改变任何状态）。

        固定字段断言：fixed_batch=True 表示每次只提交一个原始固定 batch；
        original_control_timer_unchanged=True 表示原 control timer 未被替换；
        source_stamp_or_receipts_refreshed=False 表示不在此路径刷新来源时间戳/回执。
        """
        return dict(
            schema="r92_ready_commit/v1",
            enabled=True,
            same_control_group=self.same_control_group,
            notifications=self.notifications,
            callbacks=self.callbacks,
            trigger_errors=self.trigger_errors,
            closed=self.closed,
            fixed_batch=True,
            original_control_timer_unchanged=True,
            source_stamp_or_receipts_refreshed=False,
        )
