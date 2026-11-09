"""有界且不可变的 corrected-message 交接；不持有控制状态，也不在热路径做 I/O。

一个 intake callback group 负责 join 与校验；control 组负责反序列化与所有原始业务
回调。锁只保护引用与计数；校验、序列化与派发都在锁外执行。

边界：只暂存 corrected message 的字节并做完整性/单调性校验，不写导航状态机；提交
仍由 control 组负责（可选 ready 通知见 ready_commit.py）。容量是硬编码的 24 slots /
16 frames / 32 MiB 总量 / 4 MiB 单部件。
"""

from collections import OrderedDict, deque
from dataclasses import dataclass
import json
import copy
import hashlib
import math
import threading
import time
from embodied_navigation.common.observation import api as _observer_api

diag = _observer_api("r87_diagnostic_probe")
from embodied_navigation.common.observation import api as _observer_api

evidence = _observer_api("r118_feedback_evidence")

# profile 键与证据 schema 名；EXPIRED_* 是 R86 过期前缀选择的精确类型契约。
KEY = "r86_prepared_feedback"
SCHEMA = "r86_prepared_corrected_feedback/v1"
EXPIRED_KEY = "r86_expired_prefix"
EXPIRED_SCHEMA = "r86_expired_prefix/v1"
# 容量硬编码：24 个待 join 的源 stamp、16 个已准备 frame、32 MiB 总保留量、4 MiB
# 单部件（输入 bound 与 wire bound 共用）。ROLES 顺序是隐式契约：slot 完成判断、
# arrivals_ns/scopes 索引与 decode 顺序都依赖它。
SLOTS, FRAMES, BYTES, PART_BYTES = 24, 16, 32 * 1024 * 1024, 4 * 1024 * 1024
ROLES = ("body", "cloud", "metadata")


def _scope_text(scope):
    """只返回有界身份，绝不返回消息或点云表示。

    bytes 取前 32 字节的 hex；None 返回 None；其它类型返回 <类型名前 32 字符>。
    """
    if isinstance(scope, bytes):
        return scope[:32].hex()
    if scope is None:
        return None
    return "<" + type(scope).__name__[:32] + ">"


def _bundle_mark(stage, bundle, **fields):
    """写一条诊断 stage mark；scope 被降为有界身份。仅 diag.enabled() 时执行。"""
    if diag.enabled():
        diag.mark(
            stage,
            bundle.source_ns,
            prepared_ns=bundle.prepared_ns,
            scope_body=_scope_text(bundle.scopes[0]),
            scope_cloud=_scope_text(bundle.scopes[1]),
            scope_metadata=_scope_text(bundle.scopes[2]),
            **fields,
        )


def enabled(profile):
    """返回该 mailbox 的类型化特性开关。

    profile 不是 dict 时抛 ValueError。KEY 缺键或为 None 表示关闭；存在则必须是带
    严格 bool enabled 的 dict，否则抛 ValueError，调用方无法“部分启用”这条路径。
    """
    if not isinstance(profile, dict):
        raise ValueError("Controller settings must be an object")
    option = profile.get(KEY)
    if option is None:
        return False
    if not isinstance(option, dict) or type(option.get("enabled")) is not bool:
        raise ValueError("Controller feature enabled flag must be boolean")
    return option["enabled"]


def expired_prefix_enabled(profile):
    """精确的 typed S1 选择；缺键或 false 保持原 fatal 路径。

    profile 不是 dict 或缺 EXPIRED_KEY 返回 False。存在时必须是恰好含 enabled 与
    schema 两个键的 dict、enabled 为严格 bool、schema 等于 EXPIRED_SCHEMA，否则抛
    ValueError。当前 configs/control/settings.json 未定义该键，因此默认关闭。
    """
    if not isinstance(profile, dict) or EXPIRED_KEY not in profile:
        return False
    option = profile[EXPIRED_KEY]
    if (
        type(option) is not dict
        or set(option) != {"enabled", "schema"}
        or type(option["enabled"]) is not bool
        or option["schema"] != EXPIRED_SCHEMA
    ):
        raise ValueError("Exact typed R86 expired-prefix option required")
    return option["enabled"]


def expired_prefix_timeout_ns(pose_timeout_s):
    """返回与原 body gate 相同的 freshness 超时（纳秒）。

    pose_timeout_s 必须是有限正数（int/float），否则抛 ValueError；返回 float
    pose_timeout_s * 1e9。
    """
    if (
        type(pose_timeout_s) not in (int, float)
        or not math.isfinite(pose_timeout_s)
        or not pose_timeout_s > 0
    ):
        raise ValueError("Exact finite positive pose timeout required")
    return float(pose_timeout_s) * 1e9


def expired_prefix_length(batch, decision_clock_ns, timeout_ns):
    """返回一个固定 FIFO batch 中过期的头部成员数。

    谓词与原始 Teacher body gate 完全一致：decision_clock_ns - source_ns >=
    timeout_ns。该谓词对 source_ns 单调，因此过期集总是头前缀；扫描在第一个未过期
    成员处停止。未来成员（source_ns > clock）永不过期、也不会被退休，扫描不会越过
    它继续找。decision_clock_ns 必须是严格 int，batch 必须是 tuple，每个
    source_ns 必须是严格 int，否则抛 ValueError。
    """
    if type(decision_clock_ns) is not int:
        raise ValueError("Exact integer decision clock required")
    if type(batch) is not tuple:
        raise ValueError("Fixed immutable batch tuple required")
    count = 0
    for bundle in batch:
        if type(bundle.source_ns) is not int:
            raise ValueError("Exact integer source stamp required")
        if decision_clock_ns - bundle.source_ns >= timeout_ns:
            count += 1
        else:
            break
    return count


@dataclass(frozen=True)
class Bundle:
    """一个已完整 join 并校验通过的不可变 corrected-message 帧。"""

    # 源 header stamp（纳秒）；排序、去重与过期判定都以它为准。
    source_ns: int
    # verify 返回的去重令牌；ready append 要求 (类别, 序号) 严格推进。
    token: tuple
    # metadata 中的 generation，必须是 int（构造前校验）。
    generation: int
    # 三个角色的原始编码字节：body/cloud 为 CDR，metadata 为 JSON。
    body: bytes
    cloud: bytes
    metadata: bytes
    # 实际 intake 进入时刻（纳秒），不是准备或延迟 commit 时间。
    arrivals_ns: tuple
    # bundle 构造时刻（time.monotonic_ns）。
    prepared_ns: int
    # control 侧算出的 scope digest 元组，按 ROLES 顺序。
    scopes: tuple

    @property
    def size(self):
        """返回 body+cloud+metadata 三部分字节数之和。"""
        return len(self.body) + len(self.cloud) + len(self.metadata)


class Mailbox:
    """有界、不可变的 corrected-message 交接邮箱。

    容量与角色顺序见模块顶部常量；锁只保护引用/计数，编码、校验与 ready 通知都在
    锁外。fault 一旦置位即锁存（保留首个原因），后续 intake 直接返回。
    """

    def __init__(self, encode, decode, verify):
        """参数为三个纯函数：encode(role, msg)->bytes、decode(role, bytes)->msg、
        verify(body, cloud, metadata)->token。构造函数不访问 ROS 或文件系统。
        """
        self.evidence_probe = evidence.create()
        self.evidence_call_errors = 0
        self.evidence_exit_status = None
        self.encode, self.decode, self.verify = encode, decode, verify
        self.lock = threading.Lock()
        self.pending = OrderedDict()
        self.ready = deque()
        self.bytes = 0
        self.last_source = -1
        self.last_token = None
        self.fault = None
        self.scope = None
        self.inflight = {}
        self.preparing_bytes = 0
        # counts 是交接对账的唯一来源：arrivals/prepared/rejected/duplicates/committed 等。
        self.counts = dict(
            arrivals=0,
            prepared=0,
            rejected=0,
            duplicates=0,
            committed=0,
            pending_evicted=0,
            retired_expired=0,
            retired_bytes=0,
            aborted_unprocessed=0,
        )
        # 本消费者已经决定过的源 stamp；只释放一次，且只用于对账恒等式，不参与派发。
        self._committed_sources = set()
        self._retired_sources = set()
        self.highwater_bytes = self.highwater_frames = 0
        # events 是 256 条的有界事件环，供证据/诊断；diagnostic_batch_id 只用于关联日志。
        self.events = deque(maxlen=256)
        self.diagnostic_batch_id = 0
        # ready_notify 由 control 侧注入（见 ready_commit.ReadyCommit）；始终在锁外调用。
        self.ready_notify = None

    def _evidence(self, stage, source_ns=-1, **fields):
        """记录 r118 证据；probe 自身异常只累加 evidence_call_errors，不替换业务异常。"""
        if self.evidence_probe is not None:
            try:
                self.evidence_probe.mark(stage, source_ns, **fields)
            except Exception:
                # 诊断端点不能替代业务异常。
                self.evidence_call_errors += 1

    def fail(self, reason):
        """锁存首个故障原因（截断 240 字符）并上报诊断；后续 fail 不覆盖首个原因。"""
        self._evidence(10)
        with diag.lock(self.lock, "mailbox.fail"):
            self.fault = self.fault or str(reason)[:240]
            diag.anomaly(self.fault)

    def fault_reason(self):
        """加锁返回锁存的故障原因；无故障时为 None。"""
        with diag.lock(self.lock, "mailbox.fault_reason"):
            return self.fault

    def set_scope(self, scope):
        """加锁替换 intake 看到的不可变 scope digest（由 control 组计算）。"""
        with diag.lock(self.lock, "mailbox.set_scope"):
            self.scope = scope

    def receive(self, role, source_ns, value, arrival_ns):
        """冻结一个由 ROS 持有的消息部件，并按源 stamp 入队。

        参数：role ∈ ROLES；source_ns/arrival_ns 必须是严格 int；value 为 ROS 消息；
        arrival_ns 为实际到达时刻（纳秒）。返回 None。消息在本次调用内编码为字节
        （绝不共享 live 对象），编码前后都做容量上限；重复源 stamp 不刷新收据/身份；
        同 role 但字节不同的重复会锁存 conflicting_duplicate_part。达到容量上限时
        逐出最旧 pending 或锁存故障。

        失败：身份非法 → fail(invalid_intake_identity)；编码/上限失败 →
        fail(freeze:<异常类型>)；wire 不是 bytes 或超过单部件上限 → fail；邮箱已
        fault 时静默返回。
        """
        if role not in ROLES or type(source_ns) is not int or type(arrival_ns) is not int:
            self.fail("invalid_intake_identity")
            return
        self._evidence(1, source_ns, role=ROLES.index(role), arrival_ns=arrival_ns)
        with diag.lock(self.lock, "mailbox.receive_scope", source_ns):
            if self.fault:
                return
            arrival_scope = self.scope
        # 在把消息交给另一个 callback group 之前，先把它冻结成字节。
        try:
            # cloud 的输入上限：data 字节、字段数、frame 名与字段名长度（硬编码）。
            if role == "cloud" and (
                len(value.data) > PART_BYTES
                or len(value.fields) > 64
                or len(value.header.frame_id) > 256
                or any(len(f.name) > 64 for f in value.fields)
            ):
                raise ValueError("message_input_bound")
            # body 的输入上限：frame_id 与 child_frame_id 长度（硬编码 256）。
            if role == "body" and (
                len(value.header.frame_id) > 256 or len(value.child_frame_id) > 256
            ):
                raise ValueError("body_input_bound")
            with diag.span("mailbox.encode", source_ns, role=role):
                wire = self.encode(role, value)
            # 编码结果必须是 bytes，且不超过 4 MiB 单部件上限。
            if type(wire) is not bytes or len(wire) > PART_BYTES:
                raise ValueError("message_byte_bound")
        except Exception as error:
            # 冻结失败只记录故障类型，不保存异常文本或消息内容。
            self.fail("freeze:" + type(error).__name__)
            return
        # pending 锁：计数、去重、容量与 slot 写入。
        with diag.lock(self.lock, "mailbox.pending", source_ns):
            if self.fault:
                return
            self.counts["arrivals"] += 1
            # 已准备/提交过的更旧源 stamp 一律按重复处理，不刷新。
            if source_ns <= self.last_source:
                self.counts["duplicates"] += 1
                return
            slot = self.pending.get(source_ns, {})
            previous = slot.get(role)
            if previous is not None:
                # 同一源同一 role 的重复：不刷新收据/身份；字节不同则锁存故障。
                self.counts["duplicates"] += 1
                if previous[0] != wire:
                    self.fault = "conflicting_duplicate_part"
                    diag.anomaly(self.fault, source_ns)
                return
            # 总保留字节超过 32 MiB 即锁存故障，不再接收任何部件。
            if self.bytes + len(wire) > BYTES:
                self.fault = "mailbox_byte_overflow"
                diag.anomaly(self.fault, source_ns)
                return
            # 超过 24 个 pending 源时逐出最旧（OrderedDict 头部），扣减字节并计数。
            if source_ns not in self.pending:
                while len(self.pending) >= SLOTS:
                    _, expired = self.pending.popitem(last=False)
                    self.bytes -= sum(len(part[0]) for part in expired.values())
                    self.counts["pending_evicted"] += 1
                self.pending[source_ns] = slot
            # 每个 role 存 (wire bytes, arrival_ns, scope)；三个 role 齐了才继续。
            slot[role] = (wire, arrival_ns, arrival_scope)
            self.bytes += len(wire)
            self.highwater_bytes = max(self.highwater_bytes, self.bytes)
            while len(self.pending) > SLOTS:
                _, expired = self.pending.popitem(last=False)
                self.bytes -= sum(len(part[0]) for part in expired.values())
                self.counts["pending_evicted"] += 1
            if not all(k in slot for k in ROLES):
                return
            self._evidence(2, source_ns, retained_bytes=self.bytes)
            del self.pending[source_ns]
            self.preparing_bytes = sum(len(part[0]) for part in slot.values())
            diag.mark(
                "mailbox.parts_joined",
                source_ns,
                body_bytes=len(slot["body"][0]),
                cloud_bytes=len(slot["cloud"][0]),
                metadata_bytes=len(slot["metadata"][0]),
            )
        # 哈希计算、CDR 转换与 JSON 解码都不在锁内执行。
        try:

            def decode_part(role):
                with diag.span("mailbox.decode", source_ns, role=role):
                    return self.decode(role, slot[role][0])

            body, cloud, metadata = (decode_part(k) for k in ROLES)
            with diag.span("mailbox.verify_hash_total", source_ns):
                token = self.verify(body, cloud, metadata)
            # generation 必须是 int；否则按校验拒绝处理。
            if type(metadata["generation"]) is not int:
                raise ValueError("invalid_generation")
            bundle = Bundle(
                source_ns,
                token,
                metadata["generation"],
                *(slot[k][0] for k in ROLES),
                tuple(slot[k][1] for k in ROLES),
                time.monotonic_ns(),
                tuple(slot[k][2] for k in ROLES),
            )
            _bundle_mark(
                "mailbox.bundle_constructed",
                bundle,
                generation=bundle.generation,
                arrival_body_ns=bundle.arrivals_ns[0],
                arrival_cloud_ns=bundle.arrivals_ns[1],
                arrival_metadata_ns=bundle.arrivals_ns[2],
            )
        except (ValueError, KeyError, TypeError) as error:
            # 校验失败（非故障）：归还 preparing 字节、rejected+1 并记事件，邮箱继续可用。
            with diag.lock(self.lock, "mailbox.verify_rejected", source_ns):
                self.bytes -= self.preparing_bytes
                self.preparing_bytes = 0
                self.counts["rejected"] += 1
                self.events.append(
                    dict(source_ns=source_ns, event="verify_rejected", error=type(error).__name__)
                )
                diag.mark("mailbox.verify_rejected", source_ns, error=type(error).__name__)
            return
        except Exception as error:
            # 其它异常视为准备期故障：先归还 preparing 字节，再锁存 prepare:<类型>。
            with diag.lock(self.lock, "mailbox.prepare_error", source_ns):
                self.bytes -= self.preparing_bytes
                self.preparing_bytes = 0
            self.fail("prepare:" + type(error).__name__)
            return
        # ready 锁：扣 preparing 字节、令牌单调性检查与 ready 队列容量检查。
        with diag.lock(self.lock, "mailbox.ready_append", source_ns):
            self.bytes -= self.preparing_bytes
            self.preparing_bytes = 0
            if self.fault:
                return
            # 去重令牌必须严格推进：token[0] 变化或 token[1] 不大于上次都拒绝。
            if self.last_token is not None and (
                token[0] != self.last_token[0] or token[1] <= self.last_token[1]
            ):
                self.counts["rejected"] += 1
                return
            # ready+inflight 超过 16 frames 或总字节超过 32 MiB 即锁存故障。
            if len(self.ready) + len(self.inflight) >= FRAMES or self.bytes + bundle.size > BYTES:
                self.fault = "complete_bundle_overflow"
                diag.anomaly(self.fault, source_ns)
                return
            diag.mark("mailbox.ready_append_before", source_ns, ready_count=len(self.ready))
            self.ready.append(bundle)
            self._evidence(3, bundle=bundle, ready_count=len(self.ready), retained_bytes=self.bytes)
            diag.mark("mailbox.ready_append_after", source_ns, ready_count=len(self.ready))
            self.bytes += bundle.size
            self.last_source, self.last_token = source_ns, token
            self.counts["prepared"] += 1
            self.highwater_bytes = max(self.highwater_bytes, self.bytes)
            self.highwater_frames = max(self.highwater_frames, len(self.ready))
            self.events.append(
                dict(
                    source_ns=source_ns,
                    event="prepared",
                    arrivals_ns=bundle.arrivals_ns,
                    prepared_ns=bundle.prepared_ns,
                )
            )
            for ns in list(self.pending):
                if ns <= source_ns:
                    expired = self.pending.pop(ns)
                    self.bytes -= sum(len(part[0]) for part in expired.values())
                    self.counts["pending_evicted"] += 1
        # 持锁期间不执行 control 工作、不触发 guard；ready_notify 在锁外调用。
        if self.ready_notify is not None:
            self.ready_notify()

    def peek_ready(self):
        """返回 ready 队首的不可变引用（空则 None）；唯一 control 消费者是调用方。"""
        with diag.lock(self.lock, "mailbox.peek_ready"):
            return self.ready[0] if self.ready else None

    def peek_ready_prefix(self):
        """返回 ready 的不可变 tuple 快照；超过 FRAMES 时抛 ValueError(r104_ready_bound)。"""
        with diag.lock(self.lock, "mailbox.peek_ready_prefix"):
            if len(self.ready) > FRAMES:
                raise ValueError("r104_ready_bound")
            return tuple(self.ready)

    def retained_bytes(self):
        """返回当前保留字节数（加锁读）。"""
        with diag.lock(self.lock, "mailbox.retained_bytes"):
            return self.bytes

    def take_ready_prefix(self, expected):
        """取走此前 peek 到的同一批 ready 前缀。

        expected 必须是 peek_ready_prefix 返回的 tuple；校验长度上限、ready 数量与
        对象身份（is 比较），任一不符抛 ValueError。成功后移入 inflight 并记证据，
        返回 expected。
        """
        with diag.lock(self.lock, "mailbox.take_ready_prefix"):
            if len(expected) > FRAMES or len(self.ready) < len(expected):
                raise ValueError("r104_ready_prefix_bound")
            if any(self.ready[i] is not frame for i, frame in enumerate(expected)):
                raise ValueError("r104_ready_prefix_identity_changed")
            self.diagnostic_batch_id += 1
            for frame in expected:
                self.ready.popleft()
                self.inflight[frame.source_ns] = frame.size
                self._evidence(4, bundle=frame, batch_id=self.diagnostic_batch_id)
            return expected

    def take_batch(self, max_source_ns=None):
        """取走 ready 中的一个原子批次。

        max_source_ns 为 None 时取走全部并清空；否则只取 FIFO 头中 source_ns <=
        阈值的成员，未来后缀保留原字节/收据/scope 并继续计入同一上限。返回 tuple；
        阈值必须是严格 int（或 None），否则抛 ValueError。
        """
        if max_source_ns is not None and type(max_source_ns) is not int:
            raise ValueError("Exact integer clock threshold required")
        with diag.lock(self.lock, "mailbox.take_batch"):
            self.diagnostic_batch_id += 1
            batch_id = self.diagnostic_batch_id
            diag.mark("mailbox.take_batch_before", batch_id=batch_id, ready_count=len(self.ready))
            if max_source_ns is None:
                batch = tuple(self.ready)
                self.ready.clear()
            else:
                # 一个原子 FIFO 前缀；未来后缀保留原始字节/收据/scope，并继续计入
                # 同一上限。
                prefix = []
                while self.ready and self.ready[0].source_ns <= max_source_ns:
                    prefix.append(self.ready.popleft())
                batch = tuple(prefix)
            self.inflight.update((frame.source_ns, frame.size) for frame in batch)
            diag.mark("mailbox.take_batch_after", batch_id=batch_id, batch_count=len(batch))
        if self.evidence_probe is not None:
            for bundle in batch:
                self._evidence(4, bundle=bundle, batch_id=batch_id)
        if diag.enabled():
            for index, bundle in enumerate(batch):
                if index >= FRAMES:
                    break
                _bundle_mark("mailbox.batch_member", bundle, batch_id=batch_id, index=index)
        return batch

    def retire_expired(self, batch, count, decision_clock_ns, timeout_ns):
        """只退休已过期的 FIFO 头部成员；字节仍只计一次。

        这里不推进 token、pose、receipt、scope 或 clock。调用方跳过这些成员，但仍会
        调用一次 release_batch(batch)，因此每个被退休的 payload 恰好释放一次。退休
        成员在 release 之前一直留在 inflight，所以两个时点的对账都闭合。
        count 必须是 [0, len(batch)] 内的严格 int，且前缀内出现未过期成员时抛
        ValueError；返回被退休成员的 tuple。
        """
        if type(count) is not int or count < 0 or count > len(batch):
            raise ValueError("Exact retired prefix length required")
        retired = []
        with diag.lock(self.lock, "mailbox.retire_expired"):
            for frame in batch[:count]:
                if decision_clock_ns - frame.source_ns < timeout_ns:
                    raise ValueError("retired prefix contains a non-expired member")
                self._evidence(8, bundle=frame, batch_id=self.diagnostic_batch_id)
                self.counts["retired_expired"] += 1
                self.counts["retired_bytes"] += frame.size
                self._retired_sources.add(frame.source_ns)
                self.events.append(
                    dict(
                        source_ns=frame.source_ns,
                        event="retired_expired",
                        decision_clock_ns=decision_clock_ns,
                        timeout_ns=timeout_ns,
                        arrivals_ns=frame.arrivals_ns,
                        prepared_ns=frame.prepared_ns,
                        token=frame.token,
                    )
                )
                diag.mark(
                    "mailbox.retired_expired",
                    frame.source_ns,
                    decision_clock_ns=decision_clock_ns,
                    timeout_ns=timeout_ns,
                )
                retired.append(frame)
        return tuple(retired)

    def accounting(self):
        """返回对账字典；其恒等式为
        prepared = committed + retired_expired + aborted + ready + pending。

        inflight 中的帧只有在本消费者尚未决定（未 committed/retired）时才计入
        pending。release_batch 把剩余未决定帧移入 aborted_unprocessed，因此单次
        release 前后恒等式都闭合。
        """
        with diag.lock(self.lock, "mailbox.accounting"):
            committed = self.counts["committed"]
            retired = self.counts["retired_expired"]
            prepared = self.counts["prepared"]
            ready = len(self.ready)
            pending = sum(
                1
                for source_ns in self.inflight
                if source_ns not in self._committed_sources
                and source_ns not in self._retired_sources
            )
            aborted = self.counts["aborted_unprocessed"]
            target = committed + retired + aborted + ready + pending
            return dict(
                prepared=prepared,
                committed=committed,
                retired_expired=retired,
                aborted_unprocessed=aborted,
                ready=ready,
                pending=pending,
                inflight=len(self.inflight),
                rejected=self.counts["rejected"],
                retired_bytes=self.counts["retired_bytes"],
                target_total=target,
                closes=prepared == target,
            )

    def release_batch(self, batch):
        """释放 take_batch/take_ready_prefix 返回的帧并结算 inflight 字节。

        对每个 frame：bytes -= inflight 中登记的大小（未登记会 KeyError）；若该帧既未
        committed 也未 retired，则计入 aborted_unprocessed；最后从两个已决定集合中
        移除。返回 None。
        """
        with diag.lock(self.lock, "mailbox.release_batch"):
            for frame in batch:
                self._evidence(9, bundle=frame, batch_id=self.diagnostic_batch_id)
                self.bytes -= self.inflight.pop(frame.source_ns)
                if (
                    frame.source_ns not in self._committed_sources
                    and frame.source_ns not in self._retired_sources
                ):
                    self.counts["aborted_unprocessed"] += 1
                self._committed_sources.discard(frame.source_ns)
                self._retired_sources.discard(frame.source_ns)

    def committed(self, bundle, commit_ns):
        """记录一次成功提交：committed+1、加入已决定集合并写事件（锁内）。"""
        self._evidence(7, bundle=bundle, batch_id=self.diagnostic_batch_id)
        with diag.lock(self.lock, "mailbox.committed", bundle.source_ns):
            self.counts["committed"] += 1
            self._committed_sources.add(bundle.source_ns)
            self.events.append(
                dict(
                    source_ns=bundle.source_ns,
                    event="committed",
                    prepared_ns=bundle.prepared_ns,
                    commit_ns=commit_ns,
                    arrivals_ns=bundle.arrivals_ns,
                )
            )

    def snapshot(self):
        """返回 R118/R86 证据快照（加锁读，不改状态）。

        包含 fault、counts、pending/ready/inflight 与 retained/preparing/highwater
        计数、上界常量与有界 events；source_receipts_refreshed=False 与
        physics_or_navigation_PASS=False 是固定证据字段。
        """
        with diag.lock(self.lock, "mailbox.snapshot"):
            return dict(
                schema=SCHEMA,
                fault=self.fault,
                counts=dict(self.counts),
                r118_observer_call_errors=self.evidence_call_errors,
                r118_evidence_exit_status=self.evidence_exit_status,
                pending=len(self.pending),
                ready=len(self.ready),
                retained_bytes=self.bytes,
                inflight_frames=len(self.inflight),
                inflight_bytes=sum(self.inflight.values()),
                preparing_bytes=self.preparing_bytes,
                highwater_bytes=self.highwater_bytes,
                highwater_frames=self.highwater_frames,
                bound_frames=FRAMES,
                bound_bytes=BYTES,
                events=list(self.events),
                source_receipts_refreshed=False,
                physics_or_navigation_PASS=False,
            )


def ros_mailbox(verify):
    """构造使用 ROS 序列化的 Mailbox。

    运行期才 import rclpy 与消息类型：body/cloud 用 CDR serialize_message，
    metadata 用 JSON（禁 NaN、紧凑分隔符）。返回 Mailbox；verify 由调用方提供。
    """
    from rclpy.serialization import serialize_message, deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2

    kinds = dict(body=Odometry, cloud=PointCloud2)

    def encode(role, value):
        if role == "metadata":
            return json.dumps(value, allow_nan=False, separators=(",", ":")).encode()
        return serialize_message(value)

    def decode(role, value):
        return json.loads(value) if role == "metadata" else deserialize_message(value, kinds[role])

    return Mailbox(encode, decode, verify)


def receipt_ns(node, role, fallback):
    """仅作为 freshness 来源；不要用它替换 heading/ACK 的消费时钟。

    从 node._r86_committing_bundle 取该 role 的 arrivals_ns，无 bundle 时返回
    fallback。node 必须由 control 侧提供 _r86_committing_bundle 属性。
    """
    bundle = getattr(node, "_r86_committing_bundle", None)
    return fallback if bundle is None else bundle.arrivals_ns[ROLES.index(role)]


def control_scope(node):
    """只由 control 组调用；intake 只看到这个不可变 digest。

    读取 node.request_id、waypoint_index、heading_gate 的 transition_context/
    transition_purpose/handoff_active/controlled_braking、handoff_scope_sha256 与
    command，序列化后取 sha256 并返回 bytes。隐式契约：这些属性名由
    navigation/control 侧提供，改名会静默失效。
    """
    gate = getattr(node, "heading_gate", None)
    # 读取 control 侧属性构成 scope 摘要；command 为空时 zero=True。
    data = dict(
        request=getattr(node, "request_id", None),
        waypoint=getattr(node, "waypoint_index", None),
        transition=getattr(gate, "transition_context", None),
        purpose=getattr(gate, "transition_purpose", None),
        handoff=getattr(node, "handoff_scope_sha256", None),
        active=getattr(gate, "handoff_active", False),
        braking=getattr(gate, "controlled_braking", False),
        zero=not any(getattr(node, "command", [])),
    )
    # sort_keys + allow_nan=False 保证同一业务状态得到同一条 digest。
    return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).digest()


def check_negative_parking(node):
    """保留每个入队的坏样本；绝不授予 dwell、T0 或数学更新。

    在 ordinary-parking 对象的副本上复用未改动的谓词，只有“失效”会向业务状态传播。
    原始 control 调用仍是唯一的正向转换 owner，独立的固定五秒 verifier 不变。

    参数 node 必须提供 cascade（含 ordinary_parking）、feedback、paired_imu 与
    get_clock 等 control 侧属性。返回：pose10hz 分支返回 True（该样本已消费）；
    未启用、样本早于停车起点等路径隐式返回 None。失败：无因果 IMU、姿态不安全或
    副本谓词失效时抛 ValueError（queued_parking:<failure>）；node.feedback/
    node.paired_imu 缺失时由属性访问抛 AttributeError。
    """
    core = getattr(node, "cascade", None)
    ordinary = getattr(core, "ordinary_parking", None)
    if ordinary is None or not ordinary.enabled:
        return
    # 运行期 import control 侧数学工具；本模块导入期不依赖 control。
    from embodied_navigation.control.tracking.cascade_core import rotation, vector, wrap
    import numpy as np

    feedback, imu = node.feedback, node.paired_imu
    stamp = feedback["stamp_ns"]
    if stamp < ordinary.start_clock:
        return
    # 因果配对窗 20 ms（对应 cascade maximum_gyro_pair_gap_s 默认 0.02）；IMU 缺失或
    # 不在窗口内直接抛错。
    if imu is None or not 0 <= stamp - imu["stamp_ns"] <= 20_000_000:
        raise ValueError("queued_parking_pose_without_causal_IMU")
    p = vector(feedback["position_world_xyz"], 3)
    v = vector(feedback["origin_velocity_body"], 3)
    gyro = vector(imu["angular_velocity_body"], 3)
    R = rotation(feedback["quaternion_wxyz"])
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.asin(float(np.clip(-R[2, 0], -1, 1)))
    yaw = math.atan2(R[1, 0], R[0, 0])
    # 硬编码安全姿态门：|roll|/|pitch| > 0.65 rad 或 cos(roll)/cos(pitch) < 0.4
    # 视为不安全。
    if max(abs(roll), abs(pitch)) > 0.65 or abs(math.cos(pitch)) < 0.4 or abs(math.cos(roll)) < 0.4:
        raise ValueError("queued_parking_unsafe_attitude")
    yawdot = (math.sin(roll) * gyro[1] + math.cos(roll) * gyro[2]) / math.cos(pitch)
    clock = node.get_clock().now().nanoseconds
    if ordinary.pose10hz:
        # 已接受的 commit 流也会被最终审计重放；本 intake 只拥有样本，handoff/ACK/T0
        # 仍归 control。
        from embodied_navigation.control.parking.parking_pose10hz import from_feedback

        ordinary.observe_pose10hz_source(from_feedback(feedback))
        grid = ordinary.pose_grid
        # 失效优先级：grid.missing → data_unavailable；grid.motion_failure →
        # fixed_anchor_exceeded。
        failure = (
            "parking_pose10hz_data_unavailable"
            if grid is not None and grid.missing
            else "parking_pose10hz_fixed_anchor_exceeded"
            if grid is not None and grid.motion_failure
            else None
        )
        if failure:
            ordinary.failure, ordinary.stage = failure, "failed"
            core._protect(failure, fatal=True)
            # 保留这次成功源提交到原 journal；采样得到的运动/数据失效不是 R86 源
            # 完整性故障。
        return True
    diagnostic = getattr(core, "diagnostic_observe_only", None)
    distance = float(np.linalg.norm(core.goal[:2] - p[:2]))
    yaw_error = wrap(core.goal_yaw - yaw)
    # 硬编码停车目标门：平面距离 >0.3 m、z 误差超过 core.goal_z_tolerance，或
    # |yaw 误差| >0.6 rad 视为目标丢失。
    if (
        distance > 0.3
        or abs(float(core.goal[2] - p[2])) > core.goal_z_tolerance
        or abs(yaw_error) > 0.6
    ):
        observed = ordinary.observe_business_rejection(
            "parking_fixed_target_lost",
            stamp,
            clock,
            dict(
                goal_distance_xy_m=distance,
                goal_z_error_m=float(core.goal[2] - p[2]),
                goal_yaw_error_rad=yaw_error,
                original_goal_z_tolerance_m=core.goal_z_tolerance,
                queued_pose_checked=True,
                positive_qualification_granted=False,
            ),
            diagnostic,
            terminal_zero_event=core.ordinary_zero_event,
        )
        if not observed:
            core._protect("parking_fixed_target_lost")
            ordinary.guard("parking_fixed_target_lost")
            return
    # 在副本上运行原始谓词；只有失效传播，不改动 ordinary 的正常正向转换路径。
    probe = copy.deepcopy(ordinary)
    if ordinary.diagnostic_failure_is_observed(diagnostic):
        probe.failure = None
    probe.update(stamp, clock, p, yaw, v, yawdot, gyro[2], distance, yaw_error)
    # ready_stamp 尚未建立且副本未给出 dwell 时，保持 ordinary.dwell 为 None。
    if ordinary.ready_stamp is None and probe.dwell is None:
        ordinary.dwell = None
    if probe.failure and probe.failure != ordinary.failure:
        observed = ordinary.observe_business_rejection(
            probe.failure,
            stamp,
            clock,
            dict(
                queued_pose_checked=True,
                feedback=copy.deepcopy(feedback),
                positive_qualification_granted=False,
            ),
            diagnostic,
        )
        if not observed:
            # 副本谓词失效且未被观察接受：写 failed 并向调用方抛出。
            ordinary.failure, ordinary.stage = probe.failure, "failed"
            core._protect(probe.failure, fatal=True)
            raise ValueError("queued_parking:" + probe.failure)
