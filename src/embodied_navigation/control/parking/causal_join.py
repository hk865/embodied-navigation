"""面向已有、新鲜普通 typed 零保持的有界预提交 IMU 因果配对。

不新增命令/来源/ACK，不伪装时钟，也不建第二个负载队列；只把已发布的 source 行
与实测配对 IMU 样本做有界 FIFO 关联，并记录计数与一次性事件。默认关闭，环境
变量 R104_PARKING_CAUSAL_JOIN 在模块导入时读取一次。
"""

import math
import os
import time
from collections import Counter
from numbers import Real, Integral
from embodied_navigation.control.parking.parking_zero_contract import exact_zero
from embodied_navigation.robot.unitree.bridge import validate_controller_source

# 模块导入时只读一次；缺省 "0"（关闭）；非法值直接 raise，不静默降级。
_OPTION = os.environ.get("R104_PARKING_CAUSAL_JOIN", "0")
if _OPTION not in ("0", "1"):
    raise ValueError("R104_PARKING_CAUSAL_JOIN must be 0 or 1")
PARKING_CAUSAL_JOIN_ENABLED = _OPTION == "1"
EVENT_CAP = 256  # 事件列表上限；超出后只累加 events_omitted 计数。
# 与 robot/unitree/bridge 的 envelope 完全一致（隐式契约）：只快照这些标量字段。
SOURCE_KEYS = (
    "navigation_ground_truth_used",
    "command_after_slew",
    "source_pose_stamp_ns",
    "paired_imu_stamp_ns",
    "control_stamp_ns",
    "source_pose_received_wall_ns",
    "source_imu_received_wall_ns",
    "cascade_sequence",
    "ordinary_terminal_zero_event",
    "ordinary_zero_event_id",
    "ordinary_zero_event_sequence",
    "request_id",
    "goal_sha256",
    "path_sha256",
    "cascade_mode",
    "ordinary_parking_stage",
    "source_pose_received_monotonic_wall",
    "paired_imu_received_monotonic_wall",
    "healthy",
    "reference_kind",
    "reference_sha256",
    "terminal_reference",
    "coordinate_frame_sha256",
    "generation",
)


def scalar_snapshot(value, *, limit=512):
    """对小原生标量记录做有界深拷贝；不复制负载数组。

    最多 limit 个节点、深度 8、字符串 1024、键 128；超界或遇到非标量类型会 raise
    （错误码 r104_scalar_snapshot_bound / r104_scalar_key /
    r104_non_scalar_snapshot）。Integral/Real 分别归一化为 int/float。
    """
    remaining = [limit]

    def visit(item, depth=0):
        remaining[0] -= 1
        if remaining[0] < 0 or depth > 8:
            raise ValueError("r104_scalar_snapshot_bound")
        if item is None or type(item) in (bool, int, float):
            return item
        if isinstance(item, Integral):
            return int(item)
        if isinstance(item, Real):
            return float(item)
        if type(item) is str and len(item) <= 1024:
            return item
        if type(item) is dict:
            if any(type(k) is not str or len(k) > 128 for k in item):
                raise ValueError("r104_scalar_key")
            return {k: visit(v, depth + 1) for k, v in item.items()}
        if type(item) in (list, tuple):
            return type(item)(visit(v, depth + 1) for v in item)
        raise ValueError("r104_non_scalar_snapshot")

    return visit(value)


def selected_pair(node, stamp, clock, *, snapshot=True):
    """按与 PID on_odom 相同的 port 优先级和 FIFO 因果查找配对 IMU。

    stamp 晚于 clock 直接返回 None（拒绝未来帧）；在 imu_history 中从最新往回找
    第一个不晚于 stamp、且最多早 20 ms 的样本；有 imu_source_buffer 时优先用它
    的 pair()。
    snapshot=True 时返回标量快照，否则返回原始记录。
    """
    if stamp > clock:
        return None
    pair = next(
        (r for r in reversed(node.imu_history) if 0 <= stamp - r["stamp_ns"] <= 20_000_000), None
    )
    if node.imu_source_buffer is not None:
        return node.imu_source_buffer.pair(stamp, pair, clock, snapshot=snapshot)
    return None if pair is None else (scalar_snapshot(pair) if snapshot else pair)


class ParkingCausalJoin:
    """把已发布 source 行有界 FIFO 地关联到实测配对 IMU 样本。

    只累计计数与一次性事件；绝不产生零发布、资格或第二个负载队列。
    """

    enabled = True

    def __init__(self):
        self.waiting = None
        self.published_source = None
        self.bindings = {}
        self.events = []
        self.counts = Counter()
        self.closed = False
        self.window = None

    def published(self, source):
        """记录最近一次实际 DDS publish 返回后写出的 source 行。

        仅当该行带普通终端零事件时才快照；快照只保留 SOURCE_KEYS 中的标量字段
        （上限 2048），以隔离后续 PID-row 的原地修改。
        """
        # 只有真实 DDS publish 返回后才会调用，此时 source 刚写入或被显式保留。
        self.published_source = (
            None
            if source is None or source.get("ordinary_terminal_zero_event") is None
            else scalar_snapshot({k: source[k] for k in SOURCE_KEYS if k in source}, limit=2048)
        )

    def event(self, event, bundle=None, **fields):
        """累计一次事件；超过 EVENT_CAP 后只增加 events_omitted 计数。"""
        self.counts[event] += 1
        if len(self.events) >= EVENT_CAP:
            self.counts["events_omitted"] += 1
            return
        row = dict(event=event, observed_wall_ns=time.monotonic_ns(), **fields)
        if bundle is not None:
            row.update(
                source_ns=bundle.source_ns,
                token=bundle.token,
                original_arrivals_ns=bundle.arrivals_ns,
                prepared_ns=bundle.prepared_ns,
            )
        self.events.append(row)

    def ordinary_scope(self, node):
        """当前是否处于普通停车 zero_hold 且已有 ready 与零事件。"""
        core = getattr(node, "cascade", None)
        ordinary = getattr(core, "ordinary_parking", None)
        return bool(
            ordinary is not None
            and ordinary.enabled
            and ordinary.stage == "zero_hold"
            and ordinary.ready_stamp is not None
            and core.ordinary_zero_event is not None
        )

    def owner(self, node):
        """当前 owner 身份元组；任一分量变化都会让等待/绑定失效。"""
        core = node.cascade
        ordinary = core.ordinary_parking
        return (
            node.run.name,
            node.request_id,
            node.waypoint_index,
            node._r86_scope,
            tuple(node.automatic_reference.scope),
            core.goal_sha,
            core.path_sha,
            core.ordinary_zero_event["event_id"],
            id(core),
            id(ordinary),
            ordinary.ready_stamp,
            ordinary.handoff_clock,
            node.known_scene_inputs["generation"],
            node.state,
            id(ordinary.first_actor_zero_ack),
            ordinary.first_actor_zero_ack["qualification_clock_ns"],
            core.row.get("reference_kind"),
            core.row.get("reference_sha256"),
        )

    def health(self, node, clock, wall):
        """确认普通零保持 owner、命令、保护与来源仍成立；不符即 raise。

        覆盖：普通 zero_hold 作用域、节点状态、evidence 错误、owner 绑定、失败/
        保护/障碍/倾斜锁存、精确零命令、tilt/bridge 守卫、generation 就绪、位姿/
        点云/IMU/bridge 新鲜度、首窗口证明与已发布 source 身份。异常由 prepare
        捕获并转为 fault。
        """
        core = node.cascade
        ordinary = core.ordinary_parking
        if (
            not self.ordinary_scope(node)
            or node.state not in ("running", "succeeded")
            or node.evidence.error
            or node.terminal_parking_owner is not ordinary
            or node.terminal_parking_request != node.request_id
            or ordinary.failure
            or core.protected
            or core.failure_latched
            or core.row.get("mode") != "ordinary_zero_hold"
            or core.row.get("protection_active") is not False
            or node.obstacle_hold
            or node.tilt_hold
            or getattr(node, "ordinary_terminal_failure_latched", None)
            or not exact_zero(node.command)
            or not exact_zero(core.command)
        ):
            raise ValueError("ordinary_zero_owner_or_command_changed")
        # 没有正控制 tick 运行时也要保留原安全输入与物理/bridge 保护；实际来源回执
        # 不做重新计时。
        if node.apply_tilt_guard(wall / 1e9, clock / 1e9):
            raise ValueError("original_tilt_or_bridge_guard")
        if not node._known_generation_ready():
            raise ValueError("original_generation_not_ready")
        if not (
            0 <= clock - node.pose_stamp < node.pose_timeout * 1e9
            and 0 <= clock - node.cloud_stamp < node.cloud_timeout * 1e9
            and 0 <= wall - node.pose_updated * 1e9 < node.pose_timeout * 1e9
            and 0 <= wall - node.cloud_updated * 1e9 < node.cloud_timeout * 1e9
            and node.raw_imu.fresh(wall / 1e9, clock / 1e9)
            and 0 <= wall - node.bridge_updated * 1e9 < 300_000_000
            and node.bridge_safety.get("state") == "ready"
        ):
            raise ValueError("original_source_expired")
        if self.window is not None and ordinary.first_actor_zero_ack != self.window["proof"]:
            raise ValueError("first_fixed_window_proof_changed")
        source = self.published_source
        validate_controller_source(source, node.command, clock, wall, node.run.name)
        if (
            source.get("ordinary_terminal_zero_event") != core.ordinary_zero_event
            or source.get("request_id") != node.request_id
            or source.get("goal_sha256") != core.goal_sha
            or source.get("path_sha256") != core.path_sha
            or source.get("cascade_mode") != "ordinary_zero_hold"
            or source.get("ordinary_parking_stage") != "zero_hold"
            or source.get("reference_kind") != core.row.get("reference_kind")
            or source.get("reference_sha256") != core.row.get("reference_sha256")
        ):
            raise ValueError("published_zero_source_identity_changed")

    def bundle_valid(self, node, bundle, clock, wall):
        """校验一个待提交 bundle 的因果、新鲜度、作用域与世代。"""
        if bundle.source_ns > clock:
            raise ValueError("future_pose_original_rejection")
        if not 0 <= clock - bundle.source_ns < node.pose_timeout * 1e9:
            raise ValueError("bundle_header_expired")
        if not all(
            0 <= wall - arrival < min(node.pose_timeout, node.cloud_timeout) * 1e9
            for arrival in bundle.arrivals_ns
        ):
            raise ValueError("bundle_original_receipt_expired")
        if not all(s == node._r86_scope for s in bundle.scopes):
            raise ValueError("bundle_scope_changed")
        if bundle.generation != node.known_scene_inputs["generation"]:
            raise ValueError("bundle_generation_changed")

    def pair_valid(self, node, bundle, pair, clock, wall):
        """校验配对 IMU 样本：不晚于 source 20 ms，控制/接收都在 300 ms 内。"""
        stamp = pair["stamp_ns"]
        receipt = pair["received_wall_ns"]
        gyro = pair["angular_velocity_body"]
        if (
            type(stamp) is not int
            or type(receipt) is not int
            or not 0 <= bundle.source_ns - stamp <= 20_000_000
            or not 0 <= clock - stamp <= 300_000_000
            or not 0 <= wall - receipt <= 300_000_000
            or len(gyro) != 3
            or not all(math.isfinite(x) for x in gyro)
        ):
            raise ValueError("pair_original_causality_or_freshness")

    def after_fixed_window(self, node, bundle):
        """按原 T0 校验既有首个证明；绝不重新授予资格。

        window 为 None 时先快照首个零 ACK 证明并固定窗口结束时间
        （qualification_clock_ns + 5 s）；同一窗口内证明或零事件变化会 raise。
        返回 bundle 是否晚于固定窗口结束。
        """
        core = node.cascade
        ordinary = core.ordinary_parking
        proof = ordinary.first_actor_zero_ack
        if proof is None:
            return False
        if self.window is None:
            proof = scalar_snapshot(proof, limit=1024)
            start = proof.get("qualification_clock_ns")
            native = proof.get("ack_native_clock_ns")
            if (
                type(start) is not int
                or type(native) is not int
                or not 0 <= native <= start
                or proof.get("observation_command_exact_zero") is not True
                or not exact_zero(proof.get("actual_observation_command_body", []))
            ):
                return False
            # 固定 5 s 窗口：只读既有证明，不做新资格。
            self.window = dict(
                proof=proof,
                end_ns=start + 5_000_000_000,
                event=scalar_snapshot(core.ordinary_zero_event),
            )
        if proof != self.window["proof"] or core.ordinary_zero_event != self.window["event"]:
            raise ValueError("first_fixed_window_identity_changed")
        return bundle.source_ns > self.window["end_ns"]

    def deadlines(self, node, bundle):
        """计算等待该 bundle 配对 IMU 的 sim/wall 截止时间（纳秒）。

        取各路来源 TTL 与 300 ms 来源年龄中的最早值；任一来源到期即失败，不会
        延长原窗口。
        """
        source = self.published_source
        sim = min(
            node.pose_stamp + int(node.pose_timeout * 1e9),
            node.cloud_stamp + int(node.cloud_timeout * 1e9),
            int(node.raw_imu.stamp * 1e9) + int(node.raw_imu.timeout * 1e9),
            source["source_pose_stamp_ns"] + 300_000_000,
            bundle.source_ns + int(node.pose_timeout * 1e9),
        )
        wall = min(
            int(node.pose_updated * 1e9) + int(node.pose_timeout * 1e9),
            int(node.cloud_updated * 1e9) + int(node.cloud_timeout * 1e9),
            int(node.raw_imu.updated * 1e9) + int(node.raw_imu.timeout * 1e9),
            source["source_pose_received_wall_ns"] + 300_000_000,
            source["source_imu_received_wall_ns"] + 300_000_000,
            min(bundle.arrivals_ns) + int(min(node.pose_timeout, node.cloud_timeout) * 1e9),
        )
        return sim, wall

    def prepare(self, node, mailbox):
        """返回 None 表示走原路径；否则只取固定的 ready 前缀。

        先做纯资格投影（不跑 guard/control 回调）：普通作用域、首个证明存在、快照
        全部晚于固定 5 s 窗口且尚未配对；随后做 health/owner/截止时间校验，只为确实
        缺配对 IMU 的 bundle 建立一次等待，并在其配对到达后提交固定前缀。任何
        fault 都会 mailbox.fail(...) 并返回空元组。
        """
        self.bindings = {}
        if not self.waiting and not self.ordinary_scope(node):
            return None
        snapshot = mailbox.peek_ready_prefix()
        clock = node.get_clock().now().nanoseconds
        wall = time.monotonic_ns()
        if not self.waiting:
            # 在 guard/control 回调之前做纯资格投影：所有已完整配对的普通流量与整个
            # 原始首窗口都走原路径，包括其原有失败行为。
            if not self.ordinary_scope(node) or not snapshot:
                return None
            proof = node.cascade.ordinary_parking.first_actor_zero_ack
            if not isinstance(proof, dict) or type(proof.get("qualification_clock_ns")) is not int:
                return None
            end = proof["qualification_clock_ns"] + 5_000_000_000
            if any(b.source_ns <= end or b.source_ns > clock for b in snapshot):
                return None
            if all(
                selected_pair(node, b.source_ns, clock, snapshot=False) is not None
                for b in snapshot
            ):
                return None
            if self.window is not None and node.cascade.ordinary_zero_event != self.window["event"]:
                self.window = None
            try:
                if not self.after_fixed_window(node, snapshot[0]):
                    return None
            except (ValueError, KeyError, TypeError, AttributeError):
                return None
        try:
            if not self.waiting:
                try:
                    self.health(node, clock, wall)
                except (ValueError, KeyError, TypeError, AttributeError):
                    return None
            else:
                self.health(node, clock, wall)
                if self.owner(node) != self.waiting["owner"]:
                    raise ValueError("waiting_owner_or_scope_changed")
                if (
                    clock >= self.waiting["deadline_clock_ns"]
                    or wall >= self.waiting["deadline_wall_ns"]
                ):
                    raise ValueError("original_frozen_wait_deadline_expired")
            # R99 关闭：不能用“缺配对”掩盖位于未来的后缀帧。
            if any(b.source_ns > clock for b in snapshot):
                if self.waiting:
                    raise ValueError("future_pose_original_rejection")
                return None
            if self.waiting and (
                not snapshot
                or snapshot[0].source_ns != self.waiting["source_ns"]
                or snapshot[0].token != self.waiting["token"]
            ):
                raise ValueError("waiting_head_changed")
            owner = self.owner(node)
            prefix = []
            for bundle in snapshot:
                self.bundle_valid(node, bundle, clock, wall)
                pair = selected_pair(node, bundle.source_ns, clock)
                if pair is None:
                    if self.waiting is None:
                        # 在原固定窗口之前/边界，使用未改动的立即失败路径；不塞入隐藏的
                        # 负样本。
                        if not self.after_fixed_window(node, bundle):
                            self.bindings = {}
                            return None
                        sim_deadline, wall_deadline = self.deadlines(node, bundle)
                        self.waiting = dict(
                            owner=owner,
                            source_ns=bundle.source_ns,
                            token=bundle.token,
                            started_wall_ns=wall,
                            started_clock_ns=clock,
                            deadline_clock_ns=sim_deadline,
                            deadline_wall_ns=wall_deadline,
                        )
                        self.event(
                            "wait",
                            bundle,
                            clock_ns=clock,
                            retained_bytes=mailbox.retained_bytes(),
                            deadline_clock_ns=sim_deadline,
                            deadline_wall_ns=wall_deadline,
                            original_fixed_window_end_ns=self.window["end_ns"],
                        )
                    break
                self.pair_valid(node, bundle, pair, clock, wall)
                if self.waiting is not None and self.waiting["source_ns"] == bundle.source_ns:
                    self.event(
                        "paired_after_wait",
                        bundle,
                        clock_ns=clock,
                        pair_stamp_ns=pair["stamp_ns"],
                        pair_original_receipt_ns=pair["received_wall_ns"],
                    )
                    self.waiting = None
                prefix.append(bundle)
                self.bindings[bundle.source_ns] = (bundle.token, owner, pair)
            return mailbox.take_ready_prefix(tuple(prefix))
        except (ValueError, KeyError, TypeError, AttributeError) as error:
            self.event(
                "fault",
                reason=str(error),
                waiting_source=None if self.waiting is None else self.waiting["source_ns"],
            )
            mailbox.fail("r104_causal_join:" + str(error))
            self.waiting = None
            self.bindings = {}
            return ()

    def bound_pair(self, node, stamp, clock, wall):
        """在提交 body 前核对绑定未变，并返回 (True, pair)。"""
        bundle = getattr(node, "_r86_committing_bundle", None)
        binding = self.bindings.get(stamp)
        if binding is None:
            return False, None
        if (
            bundle is None
            or bundle.source_ns != stamp
            or bundle.token != binding[0]
            or self.owner(node) != binding[1]
        ):
            raise ValueError("r104_pair_binding_changed_before_body")
        self.bundle_valid(node, bundle, clock, wall)
        self.pair_valid(node, bundle, binding[2], clock, wall)
        return True, binding[2]

    def committed(self, bundle):
        """bundle 已提交：弹出绑定并记录一次 commit 事件。"""
        if bundle.source_ns in self.bindings:
            pair = self.bindings.pop(bundle.source_ns)[2]
            self.event(
                "commit",
                bundle,
                pair_stamp_ns=pair["stamp_ns"],
                pair_original_receipt_ns=pair["received_wall_ns"],
            )

    def end_batch(self):
        """每个批次结束后清空绑定表。"""
        self.bindings = {}

    def finish(self, node):
        """结束 join：释放未处理批次，并同步写出 r104 JSON 证据。

        仅在确实发生过 wait/fault 时把 mailbox 中未处理的批次取出并 release；然后
        写 run/r104_parking_causal_join.json（本目录唯一的同步写盘），并声明
        source_receipts_refreshed/new_zero_publications/positive_qualification_granted
        全为 False。重复调用无操作。
        """
        if self.closed:
            return
        self.closed = True
        mailbox = node._r86_prepared_mailbox
        if mailbox is not None and (self.counts["wait"] or self.counts["fault"]):
            batch = mailbox.take_batch()
            mailbox.release_batch(batch)
            self.event(
                "exit_unprocessed",
                source_ns=None if self.waiting is None else self.waiting["source_ns"],
                aborted_frames=len(batch),
            )
        self.waiting = None
        self.bindings = {}
        (node.run / "r104_parking_causal_join.json").write_text(
            __import__("json").dumps(
                dict(
                    schema="r104_parking_causal_join/v1",
                    enabled=True,
                    event_cap=EVENT_CAP,
                    counts=dict(self.counts),
                    events=self.events,
                    accounting=None if mailbox is None else mailbox.accounting(),
                    source_receipts_refreshed=False,
                    new_zero_publications=False,
                    positive_qualification_granted=False,
                ),
                allow_nan=False,
            )
            + "\n"
        )
