"""固定的单向安全交接：ingress 只记录，默认回调组拥有控制权。

职责：把 `/demo/control/safety` 上的 bridge 安全事件从订阅组交给控制组，由控制组
按原有停车/保护路径应用；并给发布路径提供一个“先 reserve 再改发布状态”的线性化点。
边界：没有 FIFO、JSON 拷贝、worker 或 timer；锁不包住控制计算与 I/O。环境变量
`BRIDGE_SAFETY_INGRESS` 决定是否启用（缺省 "0"，只接受 "0"/"1"）。
"""

import os
import threading
import time

# 环境变量开关；缺省关闭，非 "0"/"1" 直接抛错而不是猜测。
KEY = "BRIDGE_SAFETY_INGRESS"


def selected():
    """读取 ingress 开关；未设置时为 False，非法取值抛 `ValueError`。"""
    value = os.environ.get(KEY, "0")
    if value not in ("0", "1"):
        raise ValueError(KEY + " must be 0 or 1")
    return value == "1"


class Handoff:
    """把一次 bridge 安全发布与原始控制组线性化。

    ingress 在短锁内只记录事件；由 guard condition 在原始控制组里 service。
    `reserve` 在已有 negative 待处理时拒绝为发布放行；reservation 之后才到的
    negative 仍留在队列里，由 owner 后续 service。内部槽位固定为 3 个
    （hold / failed / latest），不按帧累积。
    """

    def __init__(self, node):
        """保存节点、建 guard condition，并断言它属于默认（控制）回调组。

        失败即 `RuntimeError`：交接一旦跑到别的组，“控制组独占控制”的假设就不成立。
        """
        self.node = node
        self.lock = threading.Lock()
        self.latest = None
        self.hold = None
        self.failed = None
        self.terminal = False
        self.last_state = None
        self.sequence = 0
        self.scheduled = False
        self.servicing = False
        self.closed = False
        self.reserved = False
        self.subscription = None
        # 计数用于 snapshot 与运维诊断；只增不减，键名与回执 schema 一并稳定。
        self.counts = dict(
            received=0,
            hold_edges=0,
            hold_coalesced=0,
            failed_edges=0,
            services=0,
            negative_applied=0,
            ready_expired=0,
            reservations=0,
            cancelled_publications=0,
            negative_after_reservation=0,
            trigger_errors=0,
            owner_errors=0,
            closed_receives=0,
            shutdown_unapplied_negative=0,
        )
        # guard 必须挂在默认控制组：这是“默认组拥有控制”的唯一机制保证。
        self.guard = node.create_guard_condition(
            self._execute, callback_group=node.default_callback_group
        )
        if self.guard.callback_group is not node.default_callback_group:
            raise RuntimeError("Bridge handoff must use the original control group")

    def bind_subscription(self, subscription):
        """登记 ingress 订阅；它必须位于 R86 intake 组，否则 `RuntimeError`。"""
        if subscription.callback_group is not self.node._r86_intake_group:
            raise RuntimeError("Bridge ingress subscription is not in the R86 intake group")
        self.subscription = subscription

    def _owner(self, node):
        """校验调用方节点就是构造时的 owner；错节点直接拒绝。"""
        if node is not self.node:
            raise RuntimeError("bridge_safety_handoff_wrong_owner")

    def _pending(self):
        """是否还有未 service 的事件（任一槽非空）。"""
        return self.hold is not None or self.failed is not None or self.latest is not None

    def _notify(self):
        """有 pending 且尚未调度时触发 guard；触发失败转入 prepared mailbox。"""
        with self.lock:
            notify = not self.closed and self._pending() and not self.scheduled
            if notify:
                self.scheduled = True
        if notify:
            try:
                self.guard.trigger()
            except Exception as error:
                with self.lock:
                    self.counts["trigger_errors"] += 1
                self.node._r86_prepared_mailbox.fail("bridge_safety_notify:" + type(error).__name__)

    def receive(self, data, receipt):
        """记录一条 ingress 事件；不修改 `data`，不在此应用保护。

        `data["state"]` 目前有 ready/hold/failed 三条路径：ready 只进 latest；
        hold 在边沿计数并进 hold 槽；failed 会锁存 `terminal`。reservation 之后
        到达的 hold/failed 计入 `negative_after_reservation`，但仍排给 owner。
        """
        # data 是既有回调已解析好的私有 dict；这里只读，绝不修改。
        with self.lock:
            if self.closed:
                self.counts["closed_receives"] += 1
                return
            # sequence 单调递增，service 时按它给 hold/failed 排序（先到先处理）。
            self.sequence += 1
            self.counts["received"] += 1
            record = (self.sequence, data, receipt)
            state = data["state"]
            # 只在状态边沿（上一状态不是 hold）计数，避免持续 hold 刷爆计数。
            if state == "hold" and self.last_state != "hold":
                self.counts["hold_edges"] += 1
                if self.reserved:
                    self.counts["negative_after_reservation"] += 1
                if not self.terminal:
                    if self.hold is None:
                        self.hold = record
                    else:
                        self.counts["hold_coalesced"] += 1
            if state == "failed" and not self.terminal:
                self.terminal = True
                self.failed = record
                self.counts["failed_edges"] += 1
                if self.reserved:
                    self.counts["negative_after_reservation"] += 1
            # failed 之后不再更新 latest；但该 failed 自身仍会被记录。
            if not self.terminal or state == "failed" and self.failed is record:
                self.latest = record
            self.last_state = state
        self._notify()

    def _take(self, negative_only=False):
        """在锁内取走本批事件，返回 `(events, negative)`。

        negative 事件（hold/failed）按 sequence 升序合并；若 latest 的序号不晚于
        最后一个 negative，它已被覆盖因此丢弃。`negative_only` 或已 terminal 时
        不再返回 positive 的 latest。空列表表示本轮无事可做。
        """
        with self.lock:
            self.scheduled = False
            events = sorted(
                [x for x in (self.hold, self.failed) if x is not None], key=lambda x: x[0]
            )
            self.hold = self.failed = None
            if events:
                if self.latest is not None and self.latest[0] <= events[-1][0]:
                    self.latest = None
                return events, True
            if negative_only or self.terminal:
                return [], False
            value = self.latest
            self.latest = None
            return ([] if value is None else [value]), False

    def service(self, node, negative_only=False):
        """在控制组里应用一批事件；返回本批是否包含 negative 事件。

        已在 servicing 或已 close 时返回 False（不重入）。READY 事件在交接里等待
        不能获得租约：收据与源时间窗都须在 0.3 s 内，否则计 `ready_expired` 并丢弃。
        negative 事件先走原有 ordinary/terminal 的 guard（`execution_bridge_protection`），
        再调用 `node._accept_bridge_safety(force_protection=True)`；owner 异常会计
        `owner_errors`、写入 prepared mailbox 并继续抛出。
        """
        self._owner(node)
        if self.servicing or self.closed:
            return False
        events, negative = self._take(negative_only)
        if not events:
            return False
        self.servicing = True
        try:
            with self.lock:
                self.counts["services"] += 1
            for _, data, receipt in events:
                now = time.monotonic()
                if not negative and data["state"] == "ready":
                    # READY 在交接里排队不会“变新鲜”：收据与源时间都按 0.3 s 窗口
                    # 判定，过期就丢弃而不是补一个更新时间戳。
                    if not (
                        0 <= now - receipt <= 0.3
                        and -0.05 <= now - float(data["monotonic_wall"]) < 0.3
                    ):
                        with self.lock:
                            self.counts["ready_expired"] += 1
                        continue
                if negative:
                    # 这是已接受的 bridge 保护事件，即使上一速度已经是零也要走原
                    # ordinary owner guard；绝不改它的阈值或 T0 时间。
                    core = getattr(node, "cascade", None)
                    ordinary = getattr(core, "ordinary_parking", None)
                    terminal = getattr(node, "terminal_parking_owner", None)
                    # request_id 已变说明 terminal owner 属于上一请求，不能用它 apply。
                    if getattr(node, "terminal_parking_request", None) != getattr(
                        node, "request_id", None
                    ):
                        terminal = None
                    if ordinary is not None:
                        ordinary.guard("execution_bridge_protection")
                    if terminal is not None and terminal is not ordinary:
                        terminal.guard("execution_bridge_protection")
                node._accept_bridge_safety(
                    data, receipt, force_protection=negative, service_wall=now
                )
                if negative:
                    with self.lock:
                        self.counts["negative_applied"] += 1
        except Exception as error:
            with self.lock:
                self.counts["owner_errors"] += 1
            node._r86_prepared_mailbox.fail("bridge_safety_owner:" + type(error).__name__)
            raise
        finally:
            self.servicing = False
            self._notify()
        return negative

    def _execute(self):
        """guard 回调：在控制组里 service 一次。"""
        self.service(self.node)

    def reserve(self, node):
        """发布前的线性化点：有 pending 或已 close/terminal 时拒绝返回 False。"""
        self._owner(node)
        with self.lock:
            if self.closed or self.terminal or self.hold is not None or self.failed is not None:
                return False
            self.reserved = True
            self.counts["reservations"] += 1
            return True

    def end_reservation(self, node):
        """结束 reservation；之后到达的 negative 可正常排队。"""
        self._owner(node)
        with self.lock:
            self.reserved = False

    def cancel_publication(self, node):
        """取消一次发布：标记 pid_row，应用 pending negative，必要时走原 None 发布路径。

        即使旧命令已是零也要执行原 None 路径——ordinary parking/ACK/T0 保护是业务
        事件，不是“速度是否为零”的测试。返回本次补发的发布结果（或 None）。
        """
        self._owner(node)
        with self.lock:
            self.counts["cancelled_publications"] += 1
        row = getattr(node, "pid_row", None)
        if row is not None:
            row["bridge_safety_publication_cancelled"] = True
        # 用命令计数判断 service 是否真的发布过；没有变化说明还需要补一次原路径。
        before = node.counts["commands"]
        self.service(node, negative_only=True)
        # 即使旧命令已经是零，也必须走原来的 None 路径：ordinary 停车/ACK/T0
        # 保护是业务事件，不能用“速度是否非零”来替代。
        if node.counts["commands"] == before:
            return node.publish_command()
        return None

    def close(self, node, *, service_negative=True):
        """关闭交接；默认先把 pending negative 走原保护路径应用。

        shutdown 时 executor worker 已 join，因此只处理 negative，不应用 pending
        READY。`service_negative=False` 时统计未应用的 negative 数并保留队列。
        """
        self._owner(node)
        if self.closed:
            return
        # shutdown 时 owner 已 join executor worker：pending negative 仍走原保护路径，
        # 绝不应用 pending READY。
        if service_negative:
            self.service(node, negative_only=True)
        with self.lock:
            if not service_negative:
                self.counts["shutdown_unapplied_negative"] = sum(
                    x is not None for x in (self.hold, self.failed)
                )
            self.closed = True
            self.latest = None
            self.scheduled = False
            if service_negative:
                self.hold = self.failed = None

    def snapshot(self):
        """只读状态快照（schema `bridge_safety_handoff/v1`），用于回执与诊断。"""
        with self.lock:
            return dict(
                schema="bridge_safety_handoff/v1",
                enabled=True,
                closed=self.closed,
                same_control_group=self.guard.callback_group is self.node.default_callback_group,
                ingress_group_is_r86=(
                    self.subscription is not None
                    and self.subscription.callback_group is self.node._r86_intake_group
                ),
                ingress_callback_bound_to_node=(
                    self.subscription is not None
                    and getattr(self.subscription.callback, "__self__", None) is self.node
                ),
                counts=dict(self.counts),
                pending_latest=self.latest is not None,
                pending_hold=self.hold is not None,
                pending_failed=self.failed is not None,
                terminal_failed_latched=self.terminal,
                # slots=3 固定为 hold/failed/latest；0.3 s 是 READY 的收据/源时间窗。
                slots=3,
                source_and_callback_timeout_s=0.3,
                reservation_before_publication_mutation=True,
                lock_contains_control_or_IO=False,
            )
