"""普通末端零输入身份契约：纯校验，不含控制器、Actor 或原生安全实现。

职责：把"这一帧确实是普通末端零输入"表达成可复核的四层证据——事件身份
（make_event/validate_event）、输入帧授权（authorize_frame）、Actor 观测前
快照（before_actor_observation）、观测与 ACK 复核（validate_zero_observation/
validate_zero_ack）。每层只校验身份、顺序与时钟因果。

边界：不产生控制量、不做 IO、不替代原生安全判定，也不判定停车业务成功。调用方
必须先通过自己的生产者/来源/健康/TTL 门，再调用这里的校验；这里只回答"证据是否
自洽且因果"。
"""

import hashlib
import json
import math
from numbers import Real

# v1：不带 terminal reference 的普通零事件；v2 起额外绑定类型化的末端参考身份。
SCHEMA = "ordinary_terminal_zero_event/v1"
TERMINAL_SCHEMA = "ordinary_terminal_zero_event/v2"
# v2 专有的三元身份；v1 事件里出现其中任一键即视为冒充末端参考。
TERMINAL_IDENTITY = ("event_domain", "reference_kind", "reference_sha256")
# 事件 id 的确定性派生键：同一组 (run, request, goal, path, 序号, 时钟) 只得到同一 id。
IDENTITY = (
    "run_id",
    "request_id",
    "goal_sha256",
    "path_sha256",
    "event_sequence",
    "ready_pose_stamp_ns",
    "handoff_clock_ns",
    "created_cascade_sequence",
)
# 输入帧身份：把一份授权/ACK 绑定到唯一一帧实际控制器输入（frame_serial 是帧内
# 局部序号，不是生产者 stamp）。
FRAME_SCHEMA = "ordinary_zero_validated_input_frame/v1"
FRAME_IDENTITY = ("frame_serial", "input_clock_ns", "producer_sequence")
# v1 与 v2 共用的事件身份键（不含 v2 才有的 reference_* 字段）。
CONTROL_IDENTITY = (
    "run_id",
    "request_id",
    "event_sequence",
    "ready_pose_stamp_ns",
    "handoff_clock_ns",
    "created_cascade_sequence",
)


def integer(value):
    """校验并返回一个非负整数 stamp/序列号。

    拒绝 bool：Python 里 True 是 int，不显式排除就会冒充 1。
    """
    if type(value) is not int or value < 0:
        raise ValueError("Expected original nonnegative integer stamp/sequence")
    return value


def exact_zero(values):
    """仅当三个元素都是有限实数 0 时返回 True（同样排除 bool）。

    用于命令向量的"精确零"判定：0.0 与 -0.0 都算 0，非零或非有限值都不算。
    """
    return len(values) == 3 and all(
        isinstance(x, Real) and not isinstance(x, bool) and math.isfinite(x) and x == 0
        for x in values
    )


def event_id(identity):
    """按固定格式确定性派生出事件 id。

    格式：run_id:request_id:zero:<event_sequence>:<created_cascade_sequence>。
    同一身份重复派生必须得到同一字符串，任一字段变化都会改变 id。
    """
    return (
        str(identity["run_id"])
        + ":"
        + str(identity["request_id"])
        + ":zero:"
        + str(identity["event_sequence"])
        + ":"
        + str(identity["created_cascade_sequence"])
    )


def identity_keys(marker):
    """按 schema 返回该事件当前应携带的身份键集合。

    v1：不允许携带任何 terminal 身份键（出现即 raise，防止 v1 冒充 v2）。
    v2：event_domain、reference_kind 必须正确且 path_sha256 必须为空，否则 raise。
    """
    if marker.get("schema") == SCHEMA:
        if any(k in marker for k in TERMINAL_IDENTITY):
            raise ValueError("v1 cannot carry terminal reference identity")
        return CONTROL_IDENTITY
    if marker.get("schema") == TERMINAL_SCHEMA:
        from embodied_navigation.control.parking.terminal_reference import KIND

        if (
            marker.get("event_domain") != TERMINAL_SCHEMA
            or marker.get("reference_kind") != KIND
            or marker.get("path_sha256") is not None
        ):
            raise ValueError("Invalid v2 terminal event domain or fake path")
        return CONTROL_IDENTITY
    raise ValueError("Unknown ordinary zero event schema")


def make_event(
    run_id,
    request_id,
    goal_sha256,
    path_sha256,
    event_sequence,
    ready_pose_stamp_ns,
    handoff_clock_ns,
    created_cascade_sequence,
    *,
    terminal_reference=None,
):
    """构造普通末端零事件；8 个位置参数按 IDENTITY 顺序组成确定性身份。

    参数：run_id/request_id 为控制器身份；goal_sha256/path_sha256 为路径与目标哈希
    （v2 要求 path_sha256 为 None）；event_sequence 为事件序号；ready_pose_stamp_ns
    为停车就绪位姿的仿真时间戳（纳秒）；handoff_clock_ns 为交给零速手的控制时钟
    （纳秒）；created_cascade_sequence 为创建时的级联序号；terminal_reference 为
    v2 才允许的类型化末端参考。
    返回：带 schema 与 event_id 的事件字典；v2 分支额外做参考校验，并要求 run/
    request/goal 归属一致。
    失败：参考非法或归属不符即 raise，不降级成 v1。
    """
    seed = dict(
        zip(
            IDENTITY,
            (
                run_id,
                request_id,
                goal_sha256,
                path_sha256,
                event_sequence,
                ready_pose_stamp_ns,
                handoff_clock_ns,
                created_cascade_sequence,
            ),
        )
    )
    # 不给参考即 v1 事件：身份只有事件自身的 run/request/路径/时钟。
    if terminal_reference is None:
        return dict(schema=SCHEMA, **seed, event_id=event_id(seed))
    from embodied_navigation.control.parking.terminal_reference import validate_reference, KIND

    value = validate_reference(terminal_reference)
    if (
        path_sha256 is not None
        or terminal_reference["run_id"] != run_id
        or terminal_reference["request_id"] != request_id
        or terminal_reference["fixed_goal_sha256"] != goal_sha256
    ):
        raise ValueError("v2 terminal event lost reference owner")
    seed.update(event_domain=TERMINAL_SCHEMA, reference_kind=KIND, reference_sha256=value)
    return dict(schema=TERMINAL_SCHEMA, **seed, event_id=event_id(seed))


def validate_event(marker, lineage, run_id, command, native_clock_ns, *, source_max_age_ns):
    """校验一份普通末端零事件是否就是当前控制器的有效零事件。

    前置：调用方必须先通过自己的生产者/来源/健康/TTL 门；本函数不重复这些门。
    参数：marker 为事件字典；lineage 为控制器血缘（要求 navigation_ground_truth_used
    为 False 且命令为精确零）；run_id 为当前 run；command 为最终机体命令（三元素，
    m/s 与 rad/s 同序）；native_clock_ns 为本次控制时钟（纳秒）；source_max_age_ns
    为调用方给出的源龄上限（纳秒）。
    返回：事件字典的浅拷贝。
    失败：schema、身份、当前事件绑定、序号/时钟顺序任一不符即 raise。
    """
    if not isinstance(marker, dict) or marker.get("schema") not in (SCHEMA, TERMINAL_SCHEMA):
        raise ValueError("Not an ordinary terminal zero event")
    if not isinstance(lineage, dict) or lineage.get("navigation_ground_truth_used") is not False:
        raise ValueError("Zero event has no original non-GT controller lineage")
    # 事件必须绑定 slew 之后的"最终机体命令"为精确零，否则不能算零输入证据。
    if not exact_zero(command) or not exact_zero(lineage.get("command_after_slew", [])):
        raise ValueError("Ordinary zero event must bind exact final body command")
    seed = {k: marker[k] for k in identity_keys(marker)}
    if marker.get("event_id") != event_id(seed):
        raise ValueError("Ordinary zero event identity changed")
    if lineage.get("ordinary_zero_event_id") != marker["event_id"] or integer(
        lineage.get("ordinary_zero_event_sequence")
    ) != integer(marker["event_sequence"]):
        raise ValueError("Zero event is not the current controller event")
    if (
        marker["run_id"] != run_id
        or not isinstance(marker["request_id"], str)
        or not marker["request_id"].startswith(run_id + ":")
    ):
        raise ValueError("Wrong zero event run/request")
    for key in ("request_id",):
        if marker[key] != lineage.get(key):
            raise ValueError("Zero event lost current controller identity: " + key)
    if marker["schema"] == TERMINAL_SCHEMA:
        from embodied_navigation.control.parking.terminal_reference import validate_fields

        record = validate_fields(lineage, run_id)
        if record is None or marker["reference_sha256"] != lineage["reference_sha256"]:
            raise ValueError("v2 zero event lost typed terminal lineage")
    elif any(k in lineage for k in ("reference_kind", "terminal_reference", "reference_sha256")):
        raise ValueError("v1 zero event cannot impersonate a terminal reference")
    # 事件序号与创建序号都必须为正；下面统一转成非负整数后再比较顺序。
    seq, ready, handoff, created = (
        integer(marker[k])
        for k in (
            "event_sequence",
            "ready_pose_stamp_ns",
            "handoff_clock_ns",
            "created_cascade_sequence",
        )
    )
    pose = integer(lineage["source_pose_stamp_ns"])
    gyro = integer(lineage["paired_imu_stamp_ns"])
    control = integer(lineage["control_stamp_ns"])
    current = integer(lineage["cascade_sequence"])
    clock = integer(native_clock_ns)
    # 顺序契约：就绪位姿 -> 当前位姿 -> 控制时间；handoff 也必须早于本次控制，
    # 且当前级联序号不得早于事件创建序号（防止用旧事件冒充当前帧）。
    if (
        seq < 1
        or created < 1
        or current < created
        or not ready <= pose <= control
        or not ready <= handoff <= control
    ):
        raise ValueError("Zero handoff/source ordering invalid")
    # 两个不同的问题：源龄 0..source_max_age_ns（调用方传入，300 ms 量级）；
    # pose 与配对陀螺的因果配对上限 20 ms（硬编码，与 150 ms 停车配对窗不同）。
    if (
        not 0 <= clock - pose <= integer(source_max_age_ns)
        or not 0 <= pose - gyro <= 20_000_000
        or control > clock
    ):
        raise ValueError("Ordinary zero source is stale/future or not causal")
    if lineage.get("ordinary_parking_stage") not in ("handoff_zero", "zero_hold"):
        raise ValueError("Zero event not owned by current ordinary terminal stage")
    return dict(marker)


def frame_identity(frame):
    """取出当前已接受输入帧的身份三元组。

    ``FRAME_IDENTITY`` 的每个键都必须存在；frame_serial 与 producer_sequence 必须
    为正，input_clock_ns 为非负整数。这份身份就是把 ACK 证据绑定到"某一具体输入帧"
    的依据，缺一不可。
    失败：帧不是字典、字段缺失或数值非法即 raise。
    """
    if not isinstance(frame, dict):
        raise ValueError("No current accepted input-frame identity")
    result = {key: frame[key] for key in FRAME_IDENTITY}
    for key in ("frame_serial", "input_clock_ns", "producer_sequence"):
        integer(result[key])
    if result["frame_serial"] < 1 or result["producer_sequence"] < 1:
        raise ValueError("Input frame/producer sequence must be positive")
    return result


def authorize_frame(marker, lineage, run_id, command, frame, *, source_max_age_ns):
    """为"这一帧"签发零输入授权；只在该帧通过生产者/来源/健康门之后调用。

    参数：marker/lineage/run_id/command 同 validate_event；frame 为当前已接受的输入帧；
    source_max_age_ns 为源龄上限（纳秒）。
    返回：带 schema、事件、帧身份、级联序号与三源 stamp 的授权字典。
    失败：事件校验失败或帧身份非法即 raise。
    使用方约定（由包装层负责，本模块不做）：每次请求前清空授权（含重复时钟），并在
    拒绝、provider 覆盖、动态停止/恢复或换请求时撤销授权。frame_serial 是帧内局部
    序号，永远不是生产者 stamp。
    """
    current = frame_identity(frame)
    event = validate_event(
        marker,
        lineage,
        run_id,
        command,
        current["input_clock_ns"],
        source_max_age_ns=source_max_age_ns,
    )
    return dict(
        schema=FRAME_SCHEMA,
        event=event,
        frame=current,
        cascade_sequence=lineage["cascade_sequence"],
        source_pose_stamp_ns=lineage["source_pose_stamp_ns"],
        paired_imu_stamp_ns=lineage["paired_imu_stamp_ns"],
        control_stamp_ns=lineage["control_stamp_ns"],
    )


def before_actor_observation(cmd, request, authorized_frame, clock_ns, *, frame):
    """在 Actor 观测构造之前，把本帧命令就地置零并出具快照。

    参数：cmd 为可变的三元素机体命令（m/s、rad/s，会被就地改写为 0.0）；
    request 为本次请求命令；authorized_frame 为 authorize_frame 的结果；
    clock_ns 为本次控制时钟（纳秒）；frame 为当前输入帧身份。
    返回：观测前快照字典（含被改写前的 slew 命令）；无授权时返回 None。
    失败：授权属于别的帧、request 非零（恢复/非零请求不得被零接管）、时钟与观测帧
    不一致、时钟早于 handoff、或 cmd 非法/非有限即 raise（不掩盖畸形 Actor 命令）。
    边界：这里只证明"速度输入为零"；Actor、previous_action、targets 与原生 PD/故障
    处理仍归原所有者。
    """
    if authorized_frame is None:
        return None
    if (
        not isinstance(authorized_frame, dict)
        or authorized_frame.get("schema") != FRAME_SCHEMA
        or frame_identity(frame) != authorized_frame.get("frame")
    ):
        raise ValueError("Zero authorization belongs to another input frame")
    if not exact_zero(request):
        raise ValueError("A resumed/nonzero request cannot receive zero takeover")
    clock = integer(clock_ns)
    if clock != authorized_frame["frame"]["input_clock_ns"]:
        raise ValueError("Zero authorization clock differs from observation frame")
    authorized_event = authorized_frame["event"]
    if clock < integer(authorized_event["handoff_clock_ns"]):
        raise ValueError("Actor zero takeover precedes its handoff")
    if len(cmd) != 3 or not all(
        isinstance(x, Real) and not isinstance(x, bool) and math.isfinite(x) for x in cmd
    ):
        raise ValueError("Do not conceal a malformed/nonfinite Actor command")
    before = [float(x) for x in cmd]
    # 就地置零：调用方随后构造 Actor 观测时，看到的就是精确零速度命令。
    cmd[:] = [0.0, 0.0, 0.0]
    # 快照只声明"速度输入已被置零"：Actor、previous_action、targets 与原生 PD/故障
    # 处理仍由各自原所有者负责，本契约不改写它们。
    result = dict(
        schema="ordinary_zero_before_actor_observation/v1",
        event_id=authorized_event["event_id"],
        run_id=authorized_event["run_id"],
        request_id=authorized_event["request_id"],
        goal_sha256=authorized_event["goal_sha256"],
        path_sha256=authorized_event["path_sha256"],
        ready_pose_stamp_ns=authorized_event["ready_pose_stamp_ns"],
        handoff_clock_ns=authorized_event["handoff_clock_ns"],
        created_cascade_sequence=authorized_event["created_cascade_sequence"],
        event_sequence=authorized_event["event_sequence"],
        input_clock_ns=clock,
        frame_serial=authorized_frame["frame"]["frame_serial"],
        producer_sequence=authorized_frame["frame"]["producer_sequence"],
        source_pose_stamp_ns=authorized_frame["source_pose_stamp_ns"],
        paired_imu_stamp_ns=authorized_frame["paired_imu_stamp_ns"],
        control_stamp_ns=authorized_frame["control_stamp_ns"],
        cascade_sequence=authorized_frame["cascade_sequence"],
        previous_slew_command=before,
        observed_command_body=[0.0, 0.0, 0.0],
        before_observation_construction=True,
        policy_action_replaced=False,
        previous_action_cleared=False,
        native_fault_policy_changed=False,
    )
    if authorized_event["schema"] == TERMINAL_SCHEMA:
        result.update({k: authorized_event[k] for k in TERMINAL_IDENTITY})
    return result


def validate_zero_observation(proof, marker, input_clock_ns):
    """独立复核"这一实际 Actor 输入帧"，不依赖它之后的 ACK。

    参数：proof 为 before_actor_observation 的快照；marker 为当前零事件；
    input_clock_ns 为该帧输入时钟（纳秒）。
    返回：快照字典的浅拷贝。
    失败：身份/事件 id 不符、时钟非因果或过期、缺少 Actor 构造与推理证据、或命令
    不是精确零时 raise。
    """
    if (
        not isinstance(proof, dict)
        or proof.get("schema") != "ordinary_zero_before_actor_observation/v1"
    ):
        raise ValueError("Missing actual ordinary zero observation receipt")
    for key in (*identity_keys(marker), "event_id"):
        if proof.get(key) != marker.get(key):
            raise ValueError("Zero observation event identity differs: " + key)
    if marker.get("event_id") != event_id({key: marker[key] for key in identity_keys(marker)}):
        raise ValueError("Zero observation event hash differs")
    frame_identity({key: proof[key] for key in FRAME_IDENTITY})
    for key in (
        "event_sequence",
        "ready_pose_stamp_ns",
        "handoff_clock_ns",
        "created_cascade_sequence",
        "source_pose_stamp_ns",
        "paired_imu_stamp_ns",
        "control_stamp_ns",
        "cascade_sequence",
    ):
        integer(proof[key])
    clock = integer(input_clock_ns)
    pose = proof["source_pose_stamp_ns"]
    control = proof["control_stamp_ns"]
    # 这里把 300 ms 源龄与 20 ms pose/gyro 配对写成硬编码字面量（与
    # validate_event 由调用方传入 source_max_age_ns 的口径不同）。
    if (
        proof["input_clock_ns"] != clock
        or not marker["ready_pose_stamp_ns"] <= marker["handoff_clock_ns"] <= clock
        or not marker["ready_pose_stamp_ns"] <= pose <= control <= clock
        or not 0 <= clock - pose <= 300_000_000
        or not 0 <= pose - proof["paired_imu_stamp_ns"] <= 20_000_000
        or proof["cascade_sequence"] < marker["created_cascade_sequence"]
    ):
        raise ValueError("Zero observation input/source clocks are not causal and fresh")
    # 三个必须为 True 的布尔：确实在观测构造之前置零、观测命令精确零、本帧确实跑了
    # Actor 推理；缺任一即"没有真实零输入观测"。
    if not all(
        proof.get(key) is True
        for key in (
            "before_observation_construction",
            "observation_command_exact_zero",
            "actor_inferred_this_frame",
        )
    ):
        raise ValueError("Zero observation lacks actual Actor construction/inference proof")
    # 三个必须为 False：不得替换策略动作、不得清空 previous_action、不得改原生故障策略。
    if not all(
        proof.get(key) is False
        for key in (
            "policy_action_replaced",
            "previous_action_cleared",
            "native_fault_policy_changed",
        )
    ):
        raise ValueError("Ordinary zero altered Actor/native ownership")
    if not exact_zero(proof["observed_command_body"]) or not exact_zero(
        proof["actual_observation_command_body"]
    ):
        raise ValueError("Actual ordinary observation was not exact zero")
    return dict(proof)


def validate_zero_ack(ack, marker, clock_ns):
    """用当前零事件复核一份已完成的真实零输入 ACK。

    前置：ACK 的新鲜度/顺序与来源控制仍由调用方负责。
    参数：ack 为实际零输入 ACK；marker 为当前零事件；clock_ns 为当前控制时钟（纳秒）。
    返回：观测快照加上 ACK 原生时钟与 ACK 序号。
    失败：ACK 未携带类型化观测快照、身份不符、时钟不因果、或任一命令向量非精确零，
    都 raise。
    边界：这份证据只包含已知速度与身份，不包含原生导航位姿。
    """
    proof = ack.get("ordinary_zero_before_actor_observation")
    if (
        not isinstance(proof, dict)
        or proof.get("schema") != "ordinary_zero_before_actor_observation/v1"
    ):
        raise ValueError("No typed zero observation receipt in actual ACK")
    for key in (*identity_keys(marker), "event_id"):
        if proof.get(key) != marker.get(key):
            raise ValueError("Actual zero ACK lost event identity: " + key)
    if marker.get("event_id") != event_id({key: marker[key] for key in identity_keys(marker)}):
        raise ValueError("Actual zero ACK marker identity changed")
    frame_identity({key: proof[key] for key in FRAME_IDENTITY})
    for key in (
        "event_sequence",
        "ready_pose_stamp_ns",
        "handoff_clock_ns",
        "created_cascade_sequence",
        "source_pose_stamp_ns",
        "paired_imu_stamp_ns",
        "control_stamp_ns",
        "cascade_sequence",
    ):
        integer(proof[key])
    stamp = integer(ack["stamp_ns"])
    native = integer(ack["ack_native_clock_ns"])
    clock = integer(clock_ns)
    validate_zero_observation(proof, marker, stamp)
    # handoff <= 本帧输入时钟 < ACK 原生时钟 <= 当前控制时钟：三段严格递增，
    # 保证 ACK 描述的是交接之后、且不晚于当前控制周期的那一帧。
    if (
        proof["input_clock_ns"] != stamp
        or not marker["handoff_clock_ns"] <= stamp < native <= clock
        or not marker["ready_pose_stamp_ns"] <= proof["source_pose_stamp_ns"] <= stamp
        or not 0 <= proof["source_pose_stamp_ns"] - proof["paired_imu_stamp_ns"] <= 20_000_000
        or not proof["source_pose_stamp_ns"] <= proof["control_stamp_ns"] <= stamp
        or proof["cascade_sequence"] < marker["created_cascade_sequence"]
        or ack.get("navigation_ground_truth_used") is not False
    ):
        raise ValueError("Actual zero ACK is not a causal current input frame")
    if not all(
        proof.get(key) is True
        for key in (
            "before_observation_construction",
            "observation_command_exact_zero",
            "actor_inferred_this_frame",
        )
    ):
        raise ValueError("Actual zero ACK has no verified Actor observation input")
    if not all(
        exact_zero(values)
        for values in (
            ack["requested_command_body"],
            ack["applied_command_body"],
            proof["observed_command_body"],
            proof["actual_observation_command_body"],
        )
    ):
        raise ValueError("Actual zero ACK velocity/observation was not exact zero")
    return dict(proof, ack_native_clock_ns=native, ack_sequence=integer(ack["sequence"]))
