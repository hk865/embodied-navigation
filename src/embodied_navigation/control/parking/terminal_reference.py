"""诚实的固定目标末端参考：绝不是 SCAN 路径或轨迹。

本模块为“无 SCAN 路径的固定目标末端捕获”提供带类型、可哈希、可复核的参考记录，
只做纯校验与哈希，无 IO。记录明确声明 path_geometry_present=False 与
navigation_ground_truth_used=False，禁止冒充已检查的 SCAN 几何；所有时间戳均为
控制时钟与 wall 时钟纳秒。
"""

import hashlib
import json
import math

KIND = "terminal_fixed_goal/v1"
# 写入命令 lineage 的五个顶层字段；前三个用于判断是否存在 typed 参考。
FIELDS = (
    "reference_kind",
    "reference_sha256",
    "terminal_reference",
    "coordinate_frame_sha256",
    "generation",
)


def digest(value):
    """返回规范 JSON 形式的 SHA256 十六进制摘要。

    规范形式为 sort_keys + 紧凑分隔符 + allow_nan=False；出现 NaN/Inf 会直接
    raise，因此摘要不会掩盖非有限数值。
    """
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def validate_reference(record, expected_sha=None):
    """校验一条终端固定目标参考记录，返回其内容哈希。

    参数 expected_sha 非 None 时，同时要求重算哈希与它相等。校验覆盖：schema、
    禁止 SCAN 几何/真值位（path_geometry_present、navigation_ground_truth_used
    必须为 False）、全部标量为非负整数、位姿与点云同一 stamp、IMU 配对不超过
    20 ms、创建时钟在位姿之后且小于 300 ms、未过 deadline、三路来源的接收
    wall 时间都在 300 ms 内、region_receipt 属同一任务且确实 arrived/inside/
    protected=False/stamp<=位姿、fixed_goal 有限且哈希一致、坐标系哈希且第 0 位
    等于 generation。任一不符 raise ValueError。
    """
    if not isinstance(record, dict) or record.get("schema") != KIND:
        raise ValueError("No typed terminal fixed-goal reference")
    if (
        record.get("path_geometry_present") is not False
        or record.get("navigation_ground_truth_used") is not False
    ):
        raise ValueError("Terminal reference cannot impersonate checked SCAN geometry")
    for k in (
        "generation",
        "source_pose_stamp_ns",
        "source_cloud_stamp_ns",
        "paired_imu_stamp_ns",
        "created_control_clock_ns",
        "created_wall_ns",
        "source_pose_received_wall_ns",
        "source_cloud_received_wall_ns",
        "source_imu_received_wall_ns",
        "waypoint_index",
        "deadline_clock_ns",
    ):
        if type(record.get(k)) is not int or record[k] < 0:
            raise ValueError("Invalid terminal original scalar: " + k)
    pose = record["source_pose_stamp_ns"]
    clock = record["created_control_clock_ns"]
    wall = record["created_wall_ns"]
    # 位姿/点云同 stamp；IMU 最多早 20 ms；创建时钟晚于位姿但 <300 ms 且未过截止。
    if (
        pose != record["source_cloud_stamp_ns"]
        or not 0 <= pose - record["paired_imu_stamp_ns"] <= 20_000_000
        or not 0 <= clock - pose < 300_000_000
        or clock >= record["deadline_clock_ns"]
        or not all(
            0 <= wall - record[k] < 300_000_000
            for k in (
                "source_pose_received_wall_ns",
                "source_cloud_received_wall_ns",
                "source_imu_received_wall_ns",
            )
        )
    ):
        raise ValueError("Terminal creation sources not originally causal/fresh")
    arrival = record["region_receipt"]
    # 到达回执必须来自同一请求/航点/目标定义，且是已到达、区域内外均在内、未被保护。
    if (
        arrival.get("request_id") != record["request_id"]
        or arrival.get("waypoint_index") != record["waypoint_index"]
        or arrival.get("goals_definition_sha256") != record["goals_definition_sha256"]
        or arrival.get("region_inside") is not True
        or arrival.get("control_region_inside") is not True
        or arrival.get("protected") is not False
        or arrival.get("reason") != "arrived"
        or type(arrival.get("stamp_ns")) is not int
        or arrival["stamp_ns"] > pose
    ):
        raise ValueError("Terminal owner lacks actual same-task final region receipt")
    goal = record["fixed_goal"]
    # 固定目标位置与航向必须有限，并与创建时哈希一致；坐标系同样按哈希绑定。
    xyz = goal["position_world_xyz"]
    yaw = goal["heading_rad"]
    if (
        len(xyz) != 3
        or not all(math.isfinite(v) for v in (*xyz, yaw))
        or digest(goal) != record["fixed_goal_sha256"]
    ):
        raise ValueError("Terminal immutable fixed goal changed")
    if (
        digest(record["coordinate_frame"]) != record["coordinate_frame_sha256"]
        or record["coordinate_frame"][0] != record["generation"]
    ):
        raise ValueError("Terminal coordinate identity changed")
    value = digest(record)
    if expected_sha is not None and value != expected_sha:
        raise ValueError("Terminal reference hash changed")
    return value


def validate_fields(data, run_id=None):
    """从命令源/lineage 字典中提取并校验终端参考，返回记录或 None。

    顶层三个参考字段都不存在时返回 None；但若 schema 是
    actual_SLAM_terminal_fixed_goal_command_source/v1，缺失 typed 参考会 raise。
    随后禁止 SCAN 命令源 schema、要求 reference_kind=KIND，并重新校验记录哈希与
    lineage 身份：不得带 path 字段、不得声明 path_geometry_present、reference_sha256/
    generation/coordinate_frame_sha256/请求与目标哈希必须一致；给了 run_id 时记录
    必须属于该 run。
    """
    if not any(k in data for k in FIELDS[:3]):
        if data.get("schema") == "actual_SLAM_terminal_fixed_goal_command_source/v1":
            raise ValueError("Terminal command source missing typed reference")
        return None
    if data.get("reference_kind") != KIND:
        raise ValueError("Unknown terminal reference kind")
    if data.get("schema") == "actual_SLAM_SCAN_cascade_command_source/v1":
        raise ValueError("Terminal reference cannot use a SCAN command source schema")
    record = data.get("terminal_reference")
    value = validate_reference(record, data.get("reference_sha256"))
    if (
        any(
            data.get(k) is not None
            for k in ("path_sha256", "path_id", "trajectory_id", "trajectory_archive_file")
        )
        or data.get("path_geometry_present") is True
        or data.get("reference_sha256") != value
        or data.get("generation") != record["generation"]
        or data.get("coordinate_frame_sha256") != record["coordinate_frame_sha256"]
        or run_id is not None
        and record["run_id"] != run_id
        or data.get("request_id", record["request_id"]) != record["request_id"]
        or data.get("goal_sha256", data.get("fixed_goal_sha256")) != record["fixed_goal_sha256"]
    ):
        raise ValueError("Terminal lineage identity differs")
    return record


def fields(record):
    """构造写入 lineage 的五个顶层字段，其中哈希由记录重算并校验。"""
    return dict(
        reference_kind=KIND,
        reference_sha256=validate_reference(record),
        terminal_reference=record,
        coordinate_frame_sha256=record["coordinate_frame_sha256"],
        generation=record["generation"],
    )
