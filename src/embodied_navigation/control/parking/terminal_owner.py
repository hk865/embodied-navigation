"""在真实最终区域到达之后启用的 R105 末端捕获 owner（可选，无 SCAN）。

默认关闭（环境变量缺省 "0"）。启用后，节点在最后一个航点真实到达、且原始 goal
deadline 未过时，创建一个只依赖新鲜实测位姿/IMU 的固定目标末端参考与 Controller，
并把该 control tick 交给它；区域到达成功仍是独立事实，本模块不伪造 SCAN 路径、
轨迹或 bundle。
"""

import copy, os, time
import numpy as np
from embodied_navigation.control.parking.terminal_reference import KIND, digest, validate_reference
from embodied_navigation.localization_mapping.known_scene_hold_guard import _frame

# 模块导入时只读一次；缺省 "0"（关闭）；非法值直接 raise，不静默降级。
_OPTION = os.environ.get("R105_TERMINAL_PARKING_OWNER", "0")
if _OPTION not in ("0", "1"):
    raise ValueError("R105_TERMINAL_PARKING_OWNER must be 0 or 1")
TERMINAL_PARKING_OWNER_ENABLED = _OPTION == "1"


def selected(node):
    """该节点实例是否已绑定可选的 R105 末端 owner。

    通过隐式属性 _r105_terminal_owner_enabled 判断；属性不存在视为未选。
    """
    return getattr(node, "_r105_terminal_owner_enabled", False)


def invalidate(node, reason):
    """对已选中的 owner：在已推进后锁存失败，并清空其上下文。

    先看 owner 是否已推进（已有 ready_stamp、首次零 ACK，或 cascade 已产生
    ordinary_zero_event）；满足则调用 owner.guard(reason)，把失败写入
    ordinary_terminal_failure_latched（带请求 id、原因、控制时钟），并把
    node.state 置为 failed。最后无论是否推进都清空 _r105_terminal_context。
    """
    if not selected(node):
        return
    owner = getattr(node, "terminal_parking_owner", None)
    core = getattr(node, "cascade", None)
    if (
        owner is not None
        and getattr(node, "terminal_parking_request", None) == node.request_id
        and (
            owner.ready_stamp is not None
            or owner.first_actor_zero_ack is not None
            or core is not None
            and core.ordinary_zero_event is not None
        )
    ):
        owner.guard(reason)
        node.ordinary_terminal_failure_latched = dict(
            request_id=node.request_id,
            reason=owner.failure or reason,
            source="r105_coordinate_owner_invalidated",
            control_stamp_ns=node.get_clock().now().nanoseconds,
        )
        node.state = "failed"
    node._r105_terminal_context = None


def validate_context(node):
    """每次 owner tick 前重查参考、所有权、目标、坐标系与截止时间。

    校验参考哈希、Controller 身份与 terminal_reference、不得借用路径/轨迹/
    bundle、请求与目标定义、航点世界坐标、坐标系哈希、自动参考 generation、固定
    终端航向，以及控制时钟未过 deadline；任一项变化 raise ValueError。
    """
    context = node._r105_terminal_context
    core = node.cascade
    record = context["record"]
    validate_reference(record, context["sha256"])
    index = record["waypoint_index"]
    if (
        core is not context["core"]
        or core.terminal_reference != record
        or core.path is not None
        or core.path_sha is not None
        or core.path_id is not None
        or node.path_receipt is not None
        or node.active_trajectory_id is not None
        or node.samples is not None
        or node.request_id != record["request_id"]
        or node.goals_definition_sha256 != record["goals_definition_sha256"]
        or not np.array_equal(node.waypoints[index], record["fixed_goal"]["position_world_xyz"])
        or digest(_frame(node)) != record["coordinate_frame_sha256"]
        or node.automatic_reference.scope[2] != record["generation"]
        or node.fixed_terminal_heading(index, node.segment_start)
        != record["fixed_goal"]["heading_rad"]
        or node.get_clock().now().nanoseconds >= record["deadline_clock_ns"]
    ):
        raise ValueError("Terminal capture owner/coordinate/goal/deadline changed")
    return record


def control(node, now):
    """返回 True 表示本 control tick 由末端 owner 接管；区域成功仍是独立事实。

    已有上下文时先 validate_context 再 control_parking(now)。尚无上下文时：若
    node.final_bundle 非 None，说明走原有已检查的 SCAN 停车，返回 False；否则要求
    最终区域已完成且原 goal deadline 未过，旧 owner 未被 qualified/failed。

    首次创建前先消费安全 intake：受保护、来源不新鲜或障碍保持时只发布零命令并
    返回 True，持续超过 8 s 抛错；随后要求配对 IMU 与反馈同戳，否则继续发布零
    命令等待。全部通过后构造固定目标参考记录、Controller、final_bundle，并写一条
    created 证据，最后 control_parking(now)。
    """
    if not selected(node) or node.state != "succeeded":
        return False
    context = getattr(node, "_r105_terminal_context", None)
    if context is None and node.final_bundle is not None:
        return False  # 保留原有已检查的 SCAN 停车路径。
    ns = node.get_clock().now().nanoseconds
    if context is not None:
        validate_context(node)
        node.control_parking(now)
        return True
    if not node.region_arrivals or node.waypoint_index != len(node.waypoints):
        raise ValueError("Terminal owner lacks final-region completion")
    index = len(node.waypoints) - 1
    arrival = node.region_arrivals[-1]
    # 截止时间沿用原始 active goal 的超时（仿真秒换算成纳秒），不另设终端预算。
    deadline = arrival["goal_activated_ros_clock_ns"] + round(node.goals[index].timeout_sim_s * 1e9)
    if ns >= deadline:
        raise ValueError("Original terminal goal deadline exceeded")
    old = getattr(node, "terminal_parking_owner", None)
    if old is not None and getattr(node, "terminal_parking_request", None) == node.request_id:
        if old.failure or old.ready_stamp is not None or old.first_actor_zero_ack is not None:
            raise ValueError("Cannot replace an existing qualified/failed terminal owner")
    # 即使没有可用的新执行路径，也消费真实安全输入（tilt/bridge 保护）。
    protected = node.apply_tilt_guard(now, ns / 1e9)
    # 位姿/点云/IMU 与 bridge 必须同代且新鲜（300 ms）。
    fresh = (
        node._known_generation_ready()
        and node.pose_stamp == node.cloud_stamp == node.known_scene_inputs["source_ns"]
        and 0 <= ns - node.pose_stamp < 300_000_000
        and all(
            0 <= now - stamp < 0.3
            for stamp in (node.pose_updated, node.cloud_updated, node.bridge_updated)
        )
        and node.raw_imu.fresh(now, ns / 1e9)
        and node.bridge_safety.get("state") == "ready"
    )
    if protected or not fresh or node.obstacle_hold:
        node.publish_command()
        node.stale_since = node.stale_since or now
        # 连续 8 s 没有可用终端来源：锁存失败（硬编码 8.0 s，与原始实现一致）。
        if now - node.stale_since > 8.0:
            raise ValueError("Original terminal sources unavailable beyond 8 seconds")
        return True
    node.stale_since = None
    if node.paired_imu is None and node.imu_source_buffer is not None:
        node.paired_imu = copy.deepcopy(node.imu_source_buffer.pair(node.pose_stamp, None, ns))
    imu = node.paired_imu
    feedback = node.feedback
    # 配对 IMU 必须不晚于位姿 20 ms；控制时钟与接收 wall 时间都在 300 ms 内。
    if (
        imu is None
        or feedback is None
        or feedback["stamp_ns"] != node.pose_stamp
        or not 0 <= node.pose_stamp - imu["stamp_ns"] <= 20_000_000
        or not 0 <= ns - imu["stamp_ns"] < 300_000_000
        or not 0 <= time.monotonic_ns() - imu["received_wall_ns"] < 300_000_000
    ):
        node.publish_command()
        return True  # original task/goal deadline remains fixed
    if (
        node.samples is not None
        or node.path_receipt is not None
        or node.active_trajectory_id is not None
    ):
        raise ValueError("Pathless terminal owner cannot borrow an unbundled trajectory")
    runtime = node.automatic_reference
    if (
        runtime is None
        or runtime.scope is None
        or runtime.scope[2] != node.known_scene_inputs["generation"]
    ):
        raise ValueError("Terminal goal heading lacks the current registered automatic reference")
    from embodied_navigation.control.tracking.cascade_core import Controller
    from embodied_navigation.common.input_freshness import enabled as minimal_enabled
    from embodied_navigation.control.parking.parking_contract import enabled as ordinary_enabled

    if not ordinary_enabled(node.profile):
        raise ValueError("R105 requires original ordinary parking contract")
    goal = node.waypoints[index]
    start = node.pose.copy()
    yaw = node.fixed_terminal_heading(index, start)
    key = (node.request_id, index)
    core = Controller(node.profile["cascade"], goal.tolist(), yaw, ":".join(map(str, key)))
    core.minimal_simulation_baseline = minimal_enabled(node.profile)
    record = dict(
        schema=KIND,
        run_id=node.run.name,
        request_id=node.request_id,
        waypoint_index=index,
        goals_definition_sha256=node.goals_definition_sha256,
        region_receipt=copy.deepcopy(arrival),
        fixed_goal=copy.deepcopy(core.goal_record),
        fixed_goal_sha256=core.goal_sha,
        generation=node.known_scene_inputs["generation"],
        coordinate_frame=copy.deepcopy(_frame(node)),
        coordinate_frame_sha256=digest(_frame(node)),
        source_pose_stamp_ns=node.pose_stamp,
        source_cloud_stamp_ns=node.cloud_stamp,
        paired_imu_stamp_ns=imu["stamp_ns"],
        source_pose_received_wall_ns=feedback["received_wall_ns"],
        source_cloud_received_wall_ns=round(node.cloud_updated * 1e9),
        source_imu_received_wall_ns=imu["received_wall_ns"],
        created_control_clock_ns=ns,
        created_wall_ns=time.monotonic_ns(),
        deadline_clock_ns=deadline,
        path_geometry_present=False,
        navigation_ground_truth_used=False,
    )
    core.configure_terminal_reference(record)
    # 单次 control-owner 事务：不跨回调边界，也不伪造 SCAN ID。
    node.cascade = core
    node.cascade_key = key
    node.cascade_path_id = None
    node._r105_terminal_context = dict(record=record, sha256=validate_reference(record), core=core)
    node.final_bundle = dict(
        kind=KIND,
        index=index,
        start=start,
        trajectory_id=None,
        array=None,
        path_receipt=None,
        reference_stamp=None,
    )
    # 走 evidence 抽象；默认 RuntimeTelemetry.append 为空操作，默认运行不落盘。
    node.evidence.append(
        node.run / "r105_terminal_owner.jsonl",
        dict(
            event="created",
            reference=record,
            reference_sha256=node._r105_terminal_context["sha256"],
            region_fact_preserved=True,
            SCAN_motion_bundle_fabricated=False,
            first_T0_set=False,
        ),
    )
    node.control_parking(now)
    return True
