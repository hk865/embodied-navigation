"""显式的 R16 部署扩展：协调的 yaw 积分卸载，没有停止权。

事件哈希是身份标识，不是 DDS 已发布的证明；原 accepted 来源、实际发布、因果
ACK 与物理停止闸门仍然是 owner。本模块默认关闭，只在 profile 的
r85_pre_zero_integral_unload 与既有 active 锁航向刹车配置同时成立时启用。
"""

import copy
import hashlib
import json
import math

KEY = "pre_zero_integral_unload"  # 控制源里的卸载事件字段名
PROFILE_KEY = "r85_pre_zero_integral_unload"  # 启用该扩展的 profile 键
KIND = "coordinated_yaw_toward_zero_v1"
# 唯一被批准的配置：teacher/student 每秒释放的积分量，以及 10 s 机动预算（秒）。
CONFIG = dict(
    enabled=True,
    kind=KIND,
    teacher_rate_integral_per_s=0.1,
    student_rate_integral_per_s=0.04,
    maneuver_timeout_s=10.0,
)
# 事件与回执的版本化 schema，二者必须成对出现在同一个控制源里。
EVENT_SCHEMA = "r85_owned_pre_zero_integral_unload_event/v1"
RECEIPT_SCHEMA = "r85_pre_zero_yaw_integral_update/v1"


def digest(value):
    """对卸载事件的规范 JSON 形式求 SHA256（纯身份，不代表发布）。"""
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def valid_sha(value):
    """仅当是 64 位小写十六进制摘要字符串时返回 True。"""
    return (
        isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)
    )


def configuration(profile):
    """返回选中的协调卸载配置；不存在时返回 None。

    profile 缺键或值恰为 {"enabled": false} 时关闭扩展，保留原 cascade。其余值
    必须与唯一已评审的 CONFIG 完全相等，且原有的 quiet 锁航向刹车部件必须仍然
    启用，否则 raise。
    """
    value = profile.get(PROFILE_KEY)
    if value is None:
        return None
    # 只有“显式关闭”这一种形态被接受为关闭。
    if value == {"enabled": False} and type(value["enabled"]) is bool:
        return None
    if (
        not isinstance(value, dict)
        or set(value) != set(CONFIG)
        or value != CONFIG
        or type(value["enabled"]) is not bool
        or any(type(value[k]) not in (int, float) for k in CONFIG if k not in ("enabled", "kind"))
    ):
        raise ValueError("Exact explicit coordinated unload extension configuration required")
    # 卸载依赖原 active 转向刹车 + quiet 交接 + 外部航向门 + locked_heading_pd_v1 参考。
    cascade = profile.get("cascade", {})
    if (
        profile.get("r85_active_turn_braking") != {"enabled": True}
        or profile.get("r85_quiet_handoff_target") != {"enabled": True}
        or cascade.get("external_heading_gate") is not True
        or cascade.get("r85_turn_brake_reference") != {"kind": "locked_heading_pd_v1"}
    ):
        raise ValueError("Coordinated unload requires original active locked-heading quiet braking")
    return copy.deepcopy(value)


def make_event(run_id, scope, handoff, path_sha256, goal_sha256, zero_count, previous_zero):
    """构造并校验一条受控的预零卸载事件。

    所有时间都是控制时钟纳秒；到期时间取“原 deadline”与“固定 10 s 机动预算”中
    较早者。event_id 对整个记录做哈希，因此身份与内容不会分叉。
    """
    event = dict(
        schema=EVENT_SCHEMA,
        kind=KIND,
        run_id=run_id,
        scope_sha256=scope,
        context=copy.deepcopy(handoff["context"]),
        purpose=handoff["purpose"],
        brake_entry_clock_ns=round(handoff["brake_entry_control_s"] * 1e9),
        original_deadline_clock_ns=round(handoff["deadline_s"] * 1e9),
        initial_path_sha256=path_sha256,
        goal_sha256=goal_sha256,
        entry_zero_handoff_count=zero_count,
        previous_zero_identity=copy.deepcopy(previous_zero),
    )
    # 固定 10 s 预算（硬编码），并受原 deadline 约束，取两者较早者。
    event["expires_clock_ns"] = min(
        event["brake_entry_clock_ns"] + 10_000_000_000, event["original_deadline_clock_ns"]
    )
    # 身份覆盖全部字段：后续任何修改都会改变哈希。
    event["event_id"] = digest(event)
    return validate_event(event, run_id, scope)


def validate_event(value, run_id=None, scope=None, clock_ns=None):
    """校验一条受控卸载事件并返回防御性副本。

    检查精确的 schema/字段形状、内容寻址的 event_id、上下文身份、时钟顺序/到期
    与 previous-zero 链。可选 run_id/scope/clock_ns 会把事件绑定到当前 owner，并
    让超出 10 s 或来源 deadline 窗口的机动 fail closed。
    """
    e = copy.deepcopy(value)
    keys = {
        "schema",
        "kind",
        "run_id",
        "scope_sha256",
        "context",
        "purpose",
        "brake_entry_clock_ns",
        "original_deadline_clock_ns",
        "initial_path_sha256",
        "goal_sha256",
        "expires_clock_ns",
        "event_id",
        "entry_zero_handoff_count",
        "previous_zero_identity",
    }
    # 字段集合必须完全一致，不接受额外或缺失字段。
    if not isinstance(e, dict) or set(e) != keys:
        raise ValueError("Owned unload event shape differs")
    claimed = e.pop("event_id")
    if not valid_sha(claimed) or claimed != digest(e):
        raise ValueError("Owned unload event identity differs")
    e["event_id"] = claimed
    c = e["context"]
    if (
        e["schema"] != EVENT_SCHEMA
        or e["kind"] != KIND
        or not isinstance(e["run_id"], str)
        or not e["run_id"]
        or not all(valid_sha(e[k]) for k in ("scope_sha256", "initial_path_sha256", "goal_sha256"))
        or not isinstance(c, dict)
        or set(c) != {"request_id", "waypoint_index", "dynamic_recovery", "dynamic_event_sequence"}
        or not isinstance(c["request_id"], str)
        or not c["request_id"].startswith(e["run_id"] + ":")
        or type(c["waypoint_index"]) is not int
        or c["waypoint_index"] < 0
        or type(c["dynamic_recovery"]) is not bool
        or (not c["dynamic_recovery"] and c["dynamic_event_sequence"] is not None)
        or (
            c["dynamic_recovery"]
            and c["dynamic_event_sequence"] is not None
            and (type(c["dynamic_event_sequence"]) is not int or c["dynamic_event_sequence"] <= 0)
        )
        or (c["dynamic_recovery"] and e["purpose"] != "dynamic_recovery")
        or not isinstance(e["purpose"], str)
        or not e["purpose"]
        or any(
            type(e[k]) is not int or e[k] < 0
            for k in ("brake_entry_clock_ns", "expires_clock_ns", "original_deadline_clock_ns")
        )
        or e["original_deadline_clock_ns"] < e["brake_entry_clock_ns"]
        or e["expires_clock_ns"]
        != min(e["brake_entry_clock_ns"] + 10_000_000_000, e["original_deadline_clock_ns"])
    ):
        raise ValueError("Owned unload event binding invalid")
    # previous-zero 链：count=0 必须无前驱，其余必须指向前一条事件且 count 连续。
    count = e["entry_zero_handoff_count"]
    previous = e["previous_zero_identity"]
    if type(count) is not int or count < 0:
        raise ValueError("Unload edge count invalid")
    if count == 0:
        if previous is not None:
            raise ValueError("First owned event has a previous zero")
    elif (
        not isinstance(previous, dict)
        or set(previous) != {"event_id", "zero_control_clock_ns", "zero_handoff_count"}
        or not valid_sha(previous["event_id"])
        or type(previous["zero_control_clock_ns"]) is not int
        or not 0 <= previous["zero_control_clock_ns"] < e["brake_entry_clock_ns"]
        or type(previous["zero_handoff_count"]) is not int
        or previous["zero_handoff_count"] != count
    ):
        raise ValueError("Previous owned zero identity invalid")
    if run_id is not None and e["run_id"] != run_id:
        raise ValueError("Unload run differs")
    if scope is not None and e["scope_sha256"] != scope:
        raise ValueError("Unload scope differs")
    # 机动时钟必须落在 [brake_entry, expires) 内；过期或未来都拒绝。
    if clock_ns is not None and not e["brake_entry_clock_ns"] <= clock_ns < e["expires_clock_ns"]:
        raise ValueError("Prezero integral unload maneuver expired or future")
    return e


def successor(previous, current):
    """判断 current 是否为 previous 的直接后继事件。

    要求同 run/scope、count 恰好 +1、previous_zero 指向旧 event，且旧
    brake_entry <= 零时刻 < 新 brake_entry。
    """
    old = validate_event(previous)
    new = validate_event(current)
    zero = new["previous_zero_identity"]
    return bool(
        new["run_id"] == old["run_id"]
        and new["scope_sha256"] == old["scope_sha256"]
        and new["entry_zero_handoff_count"] == old["entry_zero_handoff_count"] + 1
        and zero is not None
        and zero["event_id"] == old["event_id"]
        and old["brake_entry_clock_ns"]
        <= zero["zero_control_clock_ns"]
        < new["brake_entry_clock_ns"]
    )


def toward_zero(value, rate, dt):
    """按 rate*dt 向零释放积分量（保留符号，到 0 截止）。

    rate/dt 必须是有限正数，否则 raise；value 为 0 时返回 0.0。
    """
    if not all(math.isfinite(float(v)) for v in (value, rate, dt)) or rate <= 0 or dt <= 0:
        raise ValueError("Finite positive release rate/dt required")
    return (
        math.copysign(max(0.0, abs(float(value)) - float(rate) * float(dt)), float(value))
        if value
        else 0.0
    )


def receipt(
    event,
    side,
    before,
    after,
    dt,
    clock_ns,
    path_sha256,
    goal_sha256,
    input_frame_serial=None,
    source_control_clock_ns=None,
):
    """构造并校验一条卸载递推回执（teacher 或 student 侧）。

    每侧释放率固定：teacher 0.1、student 0.04（单位与 yaw 积分同量纲，每秒）。
    teacher 的 source clock 就是 input clock；student 必须另给已接受的来源控制
    时钟。会校验事件时间窗与回执自洽性，失败 raise。
    """
    source_clock = clock_ns if side == "teacher" else source_control_clock_ns
    e = validate_event(event, clock_ns=source_clock)
    # 两侧释放率是冻结常量，必须与 CONFIG 一致。
    rate = 0.1 if side == "teacher" else 0.04
    r = dict(
        schema=RECEIPT_SCHEMA,
        kind=KIND,
        event=e,
        side=side,
        yaw_integral_before=float(before),
        yaw_integral_after=float(after),
        rate_integral_per_s=rate,
        dt_s=float(dt),
        input_clock_ns=clock_ns,
        input_frame_serial=input_frame_serial,
        source_control_clock_ns=source_clock,
        path_sha256=path_sha256,
        goal_sha256=goal_sha256,
        exact_yaw_I_zero=float(after) == 0.0,
        error_accumulation_enabled=False,
        new_deployment_math=True,
    )
    validate_receipt(r)
    return r


def validate_receipt(r, event=None, side=None, scope=None, clock_ns=None, frame=None):
    """校验一条卸载递推回执，并核对可选的事件/侧别/时钟/帧绑定。

    检查 schema、字段形状、side 与速率绑定、before/after 递推关系、dt 窗口
    （0 < dt <= 0.3 s 的原始积分头间隔；student 还必须是 50 Hz PID 的约 20 ms）、
    输入时钟与来源控制时钟的因果间隔、teacher 同钟、student 帧序号、path/goal
    哈希一致等；任何不符 raise ValueError。
    """
    keys = {
        "schema",
        "kind",
        "event",
        "side",
        "yaw_integral_before",
        "yaw_integral_after",
        "rate_integral_per_s",
        "dt_s",
        "input_clock_ns",
        "input_frame_serial",
        "source_control_clock_ns",
        "path_sha256",
        "goal_sha256",
        "exact_yaw_I_zero",
        "error_accumulation_enabled",
        "new_deployment_math",
    }
    if not isinstance(r, dict) or set(r) != keys:
        raise ValueError("Actual unload recurrence receipt missing")
    if type(r["source_control_clock_ns"]) is not int:
        raise ValueError("Actual accepted Teacher control clock missing")
    e = validate_event(r["event"], scope=scope, clock_ns=r["source_control_clock_ns"])
    # 来源控制时钟不晚于输入时钟且落后 <=300 ms；teacher 必须同钟（见下方大条件）。
    rate = 0.1 if r["side"] == "teacher" else 0.04 if r["side"] == "student" else None
    if (
        r["schema"] != RECEIPT_SCHEMA
        or r["kind"] != KIND
        or rate is None
        or r["rate_integral_per_s"] != rate
        or r["new_deployment_math"] is not True
        or r["error_accumulation_enabled"] is not False
        or any(
            type(r[k]) not in (int, float) or not math.isfinite(r[k])
            for k in ("yaw_integral_before", "yaw_integral_after", "dt_s")
        )
        or type(r["input_clock_ns"]) is not int
        or type(r["source_control_clock_ns"]) is not int
        or not 0 <= r["input_clock_ns"] - r["source_control_clock_ns"] <= 300_000_000
        or (r["side"] == "teacher" and r["source_control_clock_ns"] != r["input_clock_ns"])
        or r["yaw_integral_after"] != toward_zero(r["yaw_integral_before"], rate, r["dt_s"])
        or type(r["exact_yaw_I_zero"]) is not bool
        or r["exact_yaw_I_zero"] != (r["yaw_integral_after"] == 0.0)
        or not valid_sha(r["path_sha256"])
        or r["goal_sha256"] != e["goal_sha256"]
    ):
        raise ValueError("Actual unload recurrence/binding differs")
    # 0.3 s 是原积分头间隔上限；student 必须落在 50 Hz PID 的 20 ms 附近。
    if not 0 < r["dt_s"] <= 0.3:
        raise ValueError("Original integral header gap exceeded in unload receipt")
    if r["side"] == "student" and not 0.019999 <= r["dt_s"] <= 0.020001:
        raise ValueError("Unload receipt is not original 50Hz PID dt")
    if r["side"] == "student" and (
        type(r["input_frame_serial"]) is not int or r["input_frame_serial"] <= 0
    ):
        raise ValueError("Actual Student unload frame missing")
    # teacher 回执不允许带原生帧序号；student 必须有正整数帧序号。
    if r["side"] == "teacher" and r["input_frame_serial"] is not None:
        raise ValueError("Teacher receipt has native frame")
    if event is not None and e != validate_event(event):
        raise ValueError("Unload actual event differs")
    if side is not None and r["side"] != side:
        raise ValueError("Unload receipt side differs")
    if clock_ns is not None and r["input_clock_ns"] != clock_ns:
        raise ValueError("Unload actual input clock differs")
    if frame is not None and r["input_frame_serial"] != frame:
        raise ValueError("Unload actual frame differs")
    return r


def zero_observation(scope, clock, frame, dt, actual_I):
    """构造一条“原版精确零 I”观察记录（不应用卸载、无原生事件权）。

    记录本身不是卸载回执，交给 validate_native_extension 校验；失败 raise。
    """
    r = dict(
        schema="r85_pre_zero_yaw_integral_observation/v1",
        kind="original_exact_zero_I_observation",
        unload_applied=False,
        native_event_authority=False,
        run_id=scope,
        input_clock_ns=clock,
        input_frame_serial=frame,
        dt_s=dt,
        actual_yaw_integral=float(actual_I),
    )
    validate_native_extension(r, scope, clock, frame)
    return r


def validate_native_extension(r, scope, clock, frame):
    """校验原版精确零 I 观察记录；其他形态按 student 回执校验。

    kind=original_exact_zero_I_observation 时要求固定字段集合、run/时钟/帧绑定、
    50 Hz dt 与 actual_yaw_integral==0；否则委托 validate_receipt 并按 student
    侧别校验。
    """
    if isinstance(r, dict) and r.get("kind") == "original_exact_zero_I_observation":
        keys = {
            "schema",
            "kind",
            "unload_applied",
            "native_event_authority",
            "run_id",
            "input_clock_ns",
            "input_frame_serial",
            "dt_s",
            "actual_yaw_integral",
        }
        if (
            set(r) != keys
            or r["schema"] != "r85_pre_zero_yaw_integral_observation/v1"
            or r["unload_applied"] is not False
            or r["native_event_authority"] is not False
            or not valid_sha(scope)
            or r["run_id"] != scope
            or type(r["input_clock_ns"]) is not int
            or r["input_clock_ns"] != clock
            or type(r["input_frame_serial"]) is not int
            or frame <= 0
            or r["input_frame_serial"] != frame
            or type(r["dt_s"]) not in (int, float)
            or not 0.019999 <= r["dt_s"] <= 0.020001
            or type(r["actual_yaw_integral"]) not in (int, float)
            or r["actual_yaw_integral"] != 0.0
        ):
            raise ValueError("Actual original-zero integral observation differs")
        return r
    return validate_receipt(r, side="student", scope=scope, clock_ns=clock, frame=frame)


class NativeUnload:
    """单个卸载事件的授权器：把已接受控制源里的事件绑定到当前 owner。

    config 为 None（扩展关闭）时 authorize 恒返回 None。event 只允许保持同一条
    事件或前进到直接后继，不允许中途换一条新事件。
    """

    def __init__(self, scope, scope_sha, run_id):
        self.config = configuration(scope["profile"])
        self.scope_sha = scope_sha
        self.run_id = run_id
        self.event = None

    def authorize(self, evidence, request, clock_ns):
        """尝试授权一次卸载；返回绑定 dict，或 None 表示当前不适用。

        evidence 必须未被拒绝、request 非空，且控制源处于 controlled_braking；
        随后校验来源/位姿/IMU 时钟的因果与 300 ms 新鲜度、事件时间窗、cascade
        模式与零交接计数/上下文/目标/路径/命令帧一致。扩展关闭、evidence 被拒绝/
        request 为空、或控制源不在 controlled_braking 时返回 None（不改状态）；其余
        不符一律 raise。
        """
        if self.config is None:
            return None
        if (evidence or {}).get("rejected") is not False or not any(request):
            return None
        envelope = evidence.get("actual_original_envelope") or {}
        source = envelope.get("controller_source") or {}
        if source.get("heading_transition_stage") != "controlled_braking":
            return None
        source_clock = source.get("control_stamp_ns")
        pose_clock = source.get("source_pose_stamp_ns")
        imu_clock = source.get("paired_imu_stamp_ns")
        if (
            any(type(v) is not int for v in (source_clock, pose_clock, imu_clock))
            or not 0 <= clock_ns - pose_clock <= 300_000_000
            or not 0 <= pose_clock - imu_clock <= 20_000_000
            or not pose_clock <= source_clock <= clock_ns
            or evidence.get("physics_clock_ns") != clock_ns
            or (evidence.get("actual_read_attempt") or {}).get("read_clock_ns") != clock_ns
        ):
            raise ValueError("Original accepted unload source/input clock differs or expired")
        # Teacher 自己拥有固定的 10 s deadline。仍新鲜的 accepted 旧帧可能在正式零
        # 边沿之后到达；绝不把它的时间改写成原生时钟。
        e = validate_event(source.get(KEY), self.run_id, self.scope_sha, source_clock)
        if (
            envelope.get("healthy") is not True
            or envelope.get("stop_requested") is not False
            or envelope.get("acceptance", {}).get("sha256") != self.scope_sha
            or source.get("cascade_mode") != "brake"
            or type(source.get("heading_zero_handoff_count")) is not int
            or source["heading_zero_handoff_count"] != e["entry_zero_handoff_count"]
            or source.get("heading_transition_context") != e["context"]
            or (source.get("request_id"), source.get("waypoint_index"))
            != (e["context"]["request_id"], e["context"]["waypoint_index"])
            or source.get("goal_sha256") != e["goal_sha256"]
            or not valid_sha(source.get("path_sha256"))
            or list(source.get("command_after_slew", [])) != list(request)
            or (evidence.get("actual_read_attempt") or {}).get("status") != "accepted"
        ):
            raise ValueError("Accepted unload source context/frame differs")
        if self.event is not None and e != self.event and not successor(self.event, e):
            raise ValueError("Unload pending event may never renew")
        self.event = e
        return dict(
            event=e,
            path_sha256=source["path_sha256"],
            goal_sha256=source["goal_sha256"],
            source_control_clock_ns=source_clock,
        )
