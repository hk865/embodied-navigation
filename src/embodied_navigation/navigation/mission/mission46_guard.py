"""独立的、保持时间戳的 controller 协调门（mission46）。

职责：``mission_hold`` 读取 run 目录下的 ``mission46_control.json`` 与
``mission46_terrain_ack.json``，校验协调新鲜度、请求身份与 connector 的地形
提供方回执，返回 ``(hold, reason)``。本模块不产生自己的时间戳，只使用调用方
传入的 ``now_ns``/``wall_s``。

边界：只读 run 内 JSON，不写文件、不发命令、不改变控制律；任何缺失、过期、
格式错误或身份不符都返回 ``hold=True``（fail-closed）。由
``navigation/pid_control.py`` 在 profile.mission46_required 为真时调用。
"""

import hashlib
import json
from pathlib import Path

# 从任务路线接受的 connector goal id：goal_id -> (from_layer, to_layer)。
# 其余硬停点不在本表，mission_hold 不对它们要求地形回执。
TRANSITIONS = {
    "exploration:11": ("lower12", "upper23"),
    "return_origin:5": ("upper23", "lower12"),
    "navigation_f1_f3:7": ("lower12", "upper23"),
}


def mission_hold(run, request_id, arrivals, goal_hash, now_ns, wall_s):
    """返回 ``(hold, reason)``：协调不新鲜或未验证时保持。

    ``now_ns``/``wall_s`` 是调用方的当前时间；控制文件的 ``sim_ns`` 最多旧
    300 ms（``300000000`` ns），``monotonic_wall`` 最多旧 0.3 s，任一越界即
    hold。控制文件还须 run_id 匹配、``navigation_ground_truth_used`` 为 False、
    ``hold`` 为 bool。``hold=True`` 时返回其 reason；请求身份未释放（与
    ``request_id`` 不符）也 hold。存在 connector 到达时，还要求 terrain ack：
    证据文件必须在 run 内、sha256 匹配，ack 的 run/request/goal/回执/哈希/层间
    方向/passed/真值标签/生效时间全部一致。函数本身不返回时间戳。
    """
    run = Path(run).resolve()
    try:
        control = json.loads((run / "mission46_control.json").read_text())
        # 300 ms 仿真时钟 / 0.3 s 墙钟：硬编码新鲜度窗口。
        if (
            control.get("run_id") != run.name
            or type(control.get("sim_ns")) is not int
            or not 0 <= now_ns - control["sim_ns"] <= 300000000
            or not 0 <= wall_s - control["monotonic_wall"] <= 0.3
            or control.get("navigation_ground_truth_used") is not False
            or type(control.get("hold")) is not bool
        ):
            return True, "mission coordination expired or malformed"
        if control["hold"]:
            return True, str(control.get("reason", "mission transition hold"))
        if control.get("current_request") != request_id:
            return True, "mission request identity is not released"
        # 未到达 connector 时不需要 terrain ack；只检查最后一个 connector。
        connectors = [r for r in arrivals if r.get("goal_id") in TRANSITIONS]
        if connectors:
            arrival = connectors[-1]
            ack = json.loads((run / "mission46_terrain_ack.json").read_text())
            before, after = TRANSITIONS[arrival["goal_id"]]
            # 证据文件必须位于 run 目录内，防越界读取。
            evidence = Path(ack["source_evidence_file"]).resolve()
            if (
                not evidence.is_relative_to(run)
                or not evidence.is_file()
                or hashlib.sha256(evidence.read_bytes()).hexdigest()
                != ack["source_evidence_sha256"]
                or ack.get("run_id") != run.name
                or ack.get("request_id") != request_id
                or ack.get("goal_id") != arrival["goal_id"]
                or ack.get("arrival_receipt") != arrival
                or ack.get("goals_definition_sha256") != goal_hash
                or ack.get("from_layer") != before
                or ack.get("to_layer") != after
                or ack.get("passed") is not True
                or ack.get("navigation_ground_truth_used") is not False
                # effective_sim_ns 必须落在回执时间与当前时间之间（因果）。
                or type(ack.get("effective_sim_ns")) is not int
                or not arrival["stamp_ns"] <= ack["effective_sim_ns"] <= now_ns
            ):
                return True, "connector awaits verified causal Actor provider acknowledgment"
        return False, "fresh mission release"
    except (OSError, KeyError, ValueError, TypeError):
        # 任何读取/解析/键错误都 fail-closed 为 hold，不猜测放行。
        return True, "mission input or connector acknowledgment not ready"
