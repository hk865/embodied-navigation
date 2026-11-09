"""把已发布的 stop 事件绑定到精确的 guard 证据；本模块没有控制权。

职责：``published_stop_event`` 只在 token 与节点/状态行一致时复制事件；
``audited_dynamic`` 在 verify 失败时附加有界失败窗口审计；
``select_event_preroll`` 校验 stop 事件与 guard 记录的身份、因果序与 300 ms
原生 SCAN 预滚证据。

边界：不创建 stop、不改命令/状态、不写文件；只读节点已发布的 token 与传入
记录。``published_stop_event`` 被 ``navigation/ros_node.py`` 使用；
``audited_dynamic``/``select_event_preroll`` 在本检出没有调用点。
"""

from contextvars import ContextVar
from functools import wraps
import copy
import hashlib
import json
import time

# stop 事件 token 的固定 schema 标识。
SCHEMA = "teacher_dynamic_stop_event/v1"
# token 必须携带的字段；多一个或少一个都视为不匹配。
FIELDS = (
    "request_id",
    "waypoint_index",
    "sequence",
    "clock_ns",
    "cloud_source_ns",
    "cloud_sha256",
    "trajectory_id",
    "reference_stamp",
    "stop_count",
)
# 失败窗口审计的字节上限（判断时另留 512 B 余量）。
AUDIT_LIMIT_B = 8192
# 审计保留的 guard 引用上限；注意 select_event_preroll 当前用字面量 32，未共享该常量。
AUDIT_RECORD_LIMIT = 32
# 每次 verify 调用一个审计字典：用 ContextVar 而非全局变量，便于并发隔离。
_AUDIT = ContextVar("dynamic_stop_window_audit", default=None)


def published_stop_event(node, status):
    """仅当 token 同时匹配两个 owner 时复制已发布的 stop 事件。

    token 必须带齐 ``FIELDS``、与节点当前 request_id/obstacle_stops 一致，并与
    已发布状态行一致；否则返回 None，调用方不能绑定无关阻塞。返回深拷贝并附加
    ``schema``。
    """
    # 两个 owner：节点自身的 request_id/obstacle_stops，以及状态行。
    token = getattr(node, "_r57_blocked_guard_token", None)
    if (
        not isinstance(token, dict)
        or any(key not in token for key in FIELDS)
        or token.get("request_id") != node.request_id
        or token.get("request_id") != status.get("request_id")
        or type(token.get("stop_count")) is not int
        or token["stop_count"] != node.obstacle_stops
        or type(status.get("obstacle_stops")) is not int
        or token["stop_count"] != status.get("obstacle_stops")
    ):
        return None
    event = {key: copy.deepcopy(token[key]) for key in FIELDS}
    event["reference_stamp"] = list(event["reference_stamp"])
    return dict(event, schema=SCHEMA)


def _update(**values):
    """把证据字段合并进当前失败窗口审计（若存在）。"""
    audit = _AUDIT.get()
    if audit is not None:
        audit.update(values)


def _reference(record):
    """对一条 guard/状态源行生成紧凑、可复现的引用。

    返回序号、计算时钟、源文件偏移/行长与行 sha256；字段全部来自 record，
    不重新计算，便于事后按记录定位源行。
    """
    data = record["data"]
    return dict(
        sequence=data.get("sequence"),
        clock_ns=data.get("compute_ros_clock_ns"),
        source_offset=record.get("source_offset"),
        source_length=record.get("source_length"),
        source_line_sha256=record.get("source_line_sha256"),
    )


def audited_dynamic(function):
    """包装 verify 调用；失败时附加有界失败窗口审计并原样重抛。

    审计含 schema、request_id、提交上下文与开始/失败墙钟 ns。序列化超过
    ``AUDIT_LIMIT_B - 512``（即 7680 B）时先丢弃 ``selected_guard_refs``，
    仍超限则收敛为 request_id/error 的哈希与长度摘要。
    审计字典挂在异常对象的 ``dynamic_window_audit`` 属性上；ContextVar 在
    finally 中复位。
    """

    @wraps(function)
    def verify(
        run,
        request_id,
        status_rows,
        pose_rows,
        guard_records,
        obstacle_records,
        *,
        audit_context=None,
    ):
        # 审计字典只活在一次 verify 调用内，不落盘、不跨调用共享。
        audit = dict(
            schema="teacher_dynamic_failure_window/v1",
            request_id=request_id,
            submission=copy.deepcopy(audit_context or {}),
            verify_started_wall_ns=time.monotonic_ns(),
        )
        saved = _AUDIT.set(audit)
        try:
            return function(
                run, request_id, status_rows, pose_rows, guard_records, obstacle_records
            )
        except Exception as error:
            audit.update(
                verify_failed_wall_ns=time.monotonic_ns(),
                error=type(error).__name__ + ": " + str(error),
            )
            # 字节上限 8192，判断时留 512 B 余量；先丢候选引用，再收敛为哈希摘要。
            raw = json.dumps(audit, allow_nan=False).encode()
            if len(raw) > AUDIT_LIMIT_B - 512:
                audit.pop("selected_guard_refs", None)
                audit["selected_guard_refs_omitted_for_byte_cap"] = True
            if len(json.dumps(audit, allow_nan=False).encode()) > AUDIT_LIMIT_B - 512:
                error_raw = audit["error"].encode()
                request_raw = str(request_id).encode()
                audit = dict(
                    schema=audit["schema"],
                    request_id_prefix=request_raw[:256].decode(errors="ignore"),
                    request_id_bytes=len(request_raw),
                    request_id_sha256=hashlib.sha256(request_raw).hexdigest(),
                    error=error_raw[:512].decode(errors="ignore"),
                    error_bytes=len(error_raw),
                    error_sha256=hashlib.sha256(error_raw).hexdigest(),
                    verify_started_wall_ns=audit["verify_started_wall_ns"],
                    verify_failed_wall_ns=audit["verify_failed_wall_ns"],
                    audit_omitted_for_byte_cap=True,
                )
            # 审计挂到异常对象上，原异常继续向上抛。
            error.dynamic_window_audit = audit
            raise
        finally:
            # 复位 ContextVar，避免审计串到下一条记录。
            _AUDIT.reset(saved)

    return verify


def select_event_preroll(request_id, held, guards, verify_record):
    """校验 stop 事件与 guard 记录的绑定，返回 ``(preroll, trigger, event)``。

    ``held`` 是控制器真实停车期间的状态行（第一帧为停车边沿），``guards`` 是
    计算时钟域内的 guard 记录，``verify_record`` 对每条记录做来源验证。事件
    缺失/冲突或记录不连续都抛 ``ValueError``，绝不回退到附近阻塞。

    窗口：以事件时钟为右界、向前 300 ms 为左界（硬编码 ns），用因果前驱补齐
    左边界；要求至少 3 条记录、跨度 >= 300 ms、最后一条正是绑定事件，且相邻
    记录序号 +1、``previous_actual_guard_clock_ns`` 衔接、间隔在
    (0, 300 ms]。
    """
    if not held:
        raise ValueError("Actual controller did not stop for the moving obstacle")
    status = held[0]["data"]
    received_edge = held[0]["received_sim_ns"]
    event = status.get("dynamic_stop_event")
    _update(received_edge_ns=received_edge, stop_event=copy.deepcopy(event))
    # 事件字段必须不多不少，schema 也必须匹配。
    if (
        not isinstance(event, dict)
        or set(event) != set(FIELDS) | {"schema"}
        or event["schema"] != SCHEMA
    ):
        raise ValueError("Actual dynamic stop event token missing or malformed")
    # sequence/stop_count 从 1 起；其余身份字段允许 0。
    for key in ("waypoint_index", "sequence", "clock_ns", "cloud_source_ns", "stop_count"):
        if type(event[key]) is not int or event[key] < (
            1 if key in ("sequence", "stop_count") else 0
        ):
            raise ValueError("Actual dynamic stop event integer identity differs: " + key)
    # reference_stamp 为 [sec, nsec]，nsec 必须落在 [0, 1e9)；sha256 为 64 位小写十六进制。
    reference = event["reference_stamp"]
    if (
        not isinstance(reference, (list, tuple))
        or len(reference) != 2
        or any(type(value) is not int for value in reference)
        or reference[0] < 0
        or not 0 <= reference[1] < 1_000_000_000
        or type(event["trajectory_id"]) not in (int, str)
        or event["trajectory_id"] in (None, "")
        or not isinstance(event["cloud_sha256"], str)
        or len(event["cloud_sha256"]) != 64
        or any(c not in "0123456789abcdef" for c in event["cloud_sha256"])
    ):
        raise ValueError("Actual dynamic stop event source/reference identity malformed")
    # 事件计算时钟不得晚于接收它的仿真边沿（因果序）。
    if (
        not isinstance(request_id, str)
        or not request_id
        or event["request_id"] != request_id
        or type(received_edge) is not int
        or event["clock_ns"] > received_edge
    ):
        raise ValueError("Actual dynamic stop event request or causal receive edge differs")
    # 所有 held 状态行必须携带同一事件，不能拼接不同 hold。
    for record in held:
        data = record["data"]
        if (
            json.dumps(data.get("dynamic_stop_event"), sort_keys=True, allow_nan=False)
            != json.dumps(event, sort_keys=True, allow_nan=False)
            or type(data.get("obstacle_stops")) is not int
            or data["obstacle_stops"] != event["stop_count"]
            or type(data.get("waypoint_index")) is not int
            or data["waypoint_index"] != event["waypoint_index"]
        ):
            raise ValueError("Actual held statuses mix conflicting stop events")
    matches = [r for r in guards if r["data"].get("sequence") == event["sequence"]]
    if len(matches) != 1:
        raise ValueError("Actual dynamic stop event guard missing or duplicated")
    trigger = matches[0]
    data = trigger["data"]
    # guard 侧字段名与事件字段名一一对应，逐项同时比较类型与值。
    pairs = {
        "request_id": "request_id",
        "waypoint_index": "waypoint_index",
        "sequence": "sequence",
        "clock_ns": "compute_ros_clock_ns",
        "cloud_source_ns": "cloud_header_stamp_ns",
        "cloud_sha256": "filtered_xyz_float64_sha256",
        "trajectory_id": "trajectory_id",
    }
    for key, source in pairs.items():
        if type(data.get(source)) is not type(event[key]) or data[source] != event[key]:
            raise ValueError("Actual stop event/guard identity mismatch: " + key)
    # 事件必须对应 0->1 且 stop_count 恰好 +1 的那条 guard。
    guard_reference = data.get("reference_stamp")
    if (
        not isinstance(guard_reference, (list, tuple))
        or any(type(value) is not int for value in guard_reference)
        or tuple(guard_reference) != tuple(reference)
        or data.get("union_result", {}).get("blocked") is not True
        or data.get("obstacle_hold_before") is not False
        or data.get("obstacle_hold_after") is not True
        or type(data.get("obstacle_stops_before")) is not int
        or type(data.get("obstacle_stops_after")) is not int
        or data["obstacle_stops_before"] + 1 != event["stop_count"]
        or data["obstacle_stops_after"] != event["stop_count"]
    ):
        raise ValueError("Actual stop event is not the bound false-to-true counted guard")
    verify_record(trigger)
    event_edge = event["clock_ns"]
    # 用因果前驱覆盖完整的 300 ms 真实区间；采样相位不能丢掉左边界观测。
    # 左边界之前最后一条记录提供因果前驱；没有则直接用边界值。
    boundary = event_edge - 300_000_000
    predecessors = [r for r in guards if r["data"]["compute_ros_clock_ns"] <= boundary]
    lower = predecessors[-1]["data"]["compute_ros_clock_ns"] if predecessors else boundary
    # 预滚窗口内的记录，以及窗口外的前后邻居（只做审计引用）。
    preroll = [r for r in guards if lower <= r["data"]["compute_ros_clock_ns"] <= event_edge]
    before = [r for r in guards if r["data"]["compute_ros_clock_ns"] < lower]
    after = [r for r in guards if r["data"]["compute_ros_clock_ns"] > event_edge]
    span = (
        (preroll[-1]["data"]["compute_ros_clock_ns"] - preroll[0]["data"]["compute_ros_clock_ns"])
        if preroll
        else 0
    )
    # 先记录审计字段（若在 audited_dynamic 内），再做证据断言。
    _update(
        event_edge_ns=event_edge,
        window_lower_ns=lower,
        window_upper_ns=event_edge,
        selected_count=len(preroll),
        selected_span_ns=span,
        event_guard_ref=_reference(trigger),
        guard_source_file=trigger.get("source_file"),
        selected_guard_refs=[_reference(r) for r in preroll[:32]],
        selected_guard_refs_truncated=len(preroll) > 32,
        outside_before=_reference(before[-1]) if before else None,
        outside_after=_reference(after[0]) if after else None,
    )
    # 至少 3 条且跨度 >= 300 ms，否则窗口证据不完整。
    if len(preroll) < 3 or span < 300_000_000:
        raise ValueError("Actual dynamic hold lacks the full300ms native SCAN pre-roll")
    if preroll[-1] is not trigger:
        raise ValueError("Actual pre-roll does not end at the bound stop event")
    # 相邻记录序号 +1、前驱时钟衔接、间隔在 (0, 300 ms] 内。
    for previous, current in zip(preroll, preroll[1:]):
        left, right = previous["data"], current["data"]
        if (
            type(left.get("sequence")) is not int
            or type(right.get("sequence")) is not int
            or right["sequence"] != left["sequence"] + 1
            or type(right.get("previous_actual_guard_clock_ns")) is not int
            or right["previous_actual_guard_clock_ns"] != left["compute_ros_clock_ns"]
            or not 0 < right["compute_ros_clock_ns"] - left["compute_ros_clock_ns"] <= 300_000_000
        ):
            raise ValueError("Actual pre-roll has a missing or conflicting guard record")
    for record in preroll:
        if type(record["data"].get("compute_ros_clock_ns")) is not int:
            raise ValueError("Actual pre-roll compute clock is not an integer stamp")
        verify_record(record)
    return preroll, trigger, copy.deepcopy(event)
