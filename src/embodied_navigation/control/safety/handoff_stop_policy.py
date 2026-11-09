"""在“不兼容的非固定尾段”只请求停止旧路径驾驶；不授予任何运动。

职责：核对原路径证明、交接状态与空间配置，输出 stop-request-only 回执。
边界：不替换路径、不放宽准入、不刷新源时间；`motion_authorized=False`、
`path_replacement_authorized=False` 是硬编码语义。调用点是
`../../navigation/pid_support.py`。
"""

import math


def tail_stop_request(
    proof, handoff, spatial_config, pose_ns, cloud_ns, *, capture_active=False, failure_active=False
):
    """为不兼容的非固定旧路径尾段返回仅请求停止的回执。

    参数：`proof` 为原路径准入证明；`handoff` 为交接状态；`spatial_config`
    提供 `enabled` 与 `arc_length_m`（原预览弧长，单位 m）；`pose_ns`/`cloud_ns`
    是整数源时间戳，必须仍等于交接时捕获的时间戳（不允许借新时间刷新）。
    `capture_active`/`failure_active` 为 True 时条件不成立。

    `remaining_m` 必须为正且不超过原预览弧长；这里不授权替换路径或运动。
    所有条件按布尔表求值与返回；`remaining_m`/`arc` 提取失败时按“不满足”处理
    （两者为 None，相关条件为 False）。
    """
    old = (handoff or {}).get("old_reference", {})
    try:
        remaining = float(old["remaining_m"])
        arc = float(spatial_config["arc_length_m"])
        finite = math.isfinite(remaining) and math.isfinite(arc)
    except (KeyError, TypeError, ValueError, OverflowError):
        remaining = arc = None
        finite = False
    # 每个 conditions 项都必须是 True 才允许“仅请求停止”；任一项为 False 即整体
    # required=False，调用方不会据此发停车请求。这里只做布尔核对，不改原证明。
    conditions = dict(
        original_geometry=proof.get("original_geometry_allowed") is True,
        original_ordered_leg=proof.get("ordered_current_leg", {}).get("allowed") is True,
        original_direction=proof.get("direction_coherence", {}).get("allowed") is True,
        spatial_reference_enabled=spatial_config.get("enabled") is True,
        # 原证明须已被拒绝且原因正是 handoff_not_ready；两者缺一不可。
        original_rejection=proof.get("allowed") is False
        and proof.get("reason") == "handoff_not_ready",
        required=handoff is not None and handoff.get("required") is True,
        incompatible=handoff is not None and handoff.get("heading_compatible") is False,
        replacement_context=handoff is not None
        and handoff.get("eligibility", {}).get("replacement_allowed") is True,
        nonfixed_endpoint=old.get("endpoint_is_fixed_goal") is False,
        # 尾段必须有限、为正且落在原预览弧长内；越界即视为不是“短尾段”。
        original_short_tail=finite and 0 < remaining <= arc,
        # 源时间戳必须仍是交接时的那一个（整数且 >0），防止用新样本掩盖旧证明。
        unchanged_pose=handoff is not None
        and type(pose_ns) is int
        and pose_ns > 0
        and handoff.get("source_pose_stamp_ns") == pose_ns,
        unchanged_cloud=handoff is not None
        and type(cloud_ns) is int
        and cloud_ns > 0
        and handoff.get("source_cloud_stamp_ns") == cloud_ns,
        no_capture=capture_active is False,
        no_failure=failure_active is False,
    )
    # 回执把每个条件原样带给消费端；三个 *_authorized / bypassed 字段恒为 False，
    # 明确本回执只请求停止，不授予运动、不替换路径、不绕过 Teacher 交接。
    return dict(
        schema="nonfixed_tail_original_stop_request/v1",
        required=all(conditions.values()),
        conditions=conditions,
        remaining_m=remaining,
        original_preview_arc_m=arc,
        source_pose_stamp_ns=pose_ns,
        source_cloud_stamp_ns=cloud_ns,
        path_id=old.get("path_id"),
        actual_source_times_refreshed=False,
        stop_request_only=True,
        path_replacement_authorized=False,
        motion_authorized=False,
        Teacher_transition_bypassed=False,
    )
