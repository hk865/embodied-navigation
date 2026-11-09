"""按原评估顺序登记的 bridge veto 原因码；判定权限不变。

职责：给既有的 reason 字符串附加稳定 code，供消费端统计/日志使用。
边界：谓词与状态变化仍留在 `CommandGate.snapshot`；本模块只做登记，不改变
评估顺序、不接受绕过（没有 bypass 模式），也不产生新判据。
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class GateSpec:
    """一个 veto 码的登记项。

    `code` 是稳定码；`effect` 沿用原 bridge 的“发零命令”效果；`predicate_owner`
    指向真正的判据位置（`CommandGate.snapshot`），本模块不复制谓词。
    """

    code: str
    effect: str = "original_bridge_zero_command"
    predicate_owner: str = "CommandGate.snapshot"


# 下列顺序即原 bridge 的评估顺序；新增码只能追加，不能重排既有条目。
GATE_SPECS = tuple(
    GateSpec(code)
    for code in (
        "latched_failure",
        "cascade_provenance",
        "champ_feedback_freshness",
        "champ_failed",
        "sensor_anchor_publisher_chain",
        "sensor_warmup",
        "immutable_slam_anchor",
        "route_fence",
        "publisher_graph_freshness",
        "slam_publisher_identity",
        "command_publisher_identity",
        "slam_pose_validity",
        "two_valid_slam_poses",
        "command_validity",
        "dual_source_age",
    )
)
GATES = {spec.code: spec for spec in GATE_SPECS}


class GateReasons(list):
    """保留原 list 的真值语义、顺序、重复项与字符串。

    继承 `list` 是为了让既有消费端按“非空即拒绝”判断的行为完全不变；
    `events` 只是附加的结构化记录。未知 code 会因 `GATES[code]` 索引失败
    抛 `KeyError`，这是有意的（拼错码不应静默通过）。
    """

    def __init__(self):
        super().__init__()
        self.events = []

    def reject(self, code, reason):
        """追加原 reason 与结构化事件；不改判据，也不去重。"""
        spec = GATES[code]
        self.append(reason)
        self.events.append(dict(code=code, reason=reason, effect=spec.effect))
