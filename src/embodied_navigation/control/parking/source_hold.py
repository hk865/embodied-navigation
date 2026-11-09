"""在进入任何零交接之前，选择原有的严格目标包络保护。

R106 预资格来源保持默认关闭。本模块只做纯判定：当停车因目标包络问题被拒时，
返回 True 后由调用方记录保持并回到原有的严格 _protect / guard 路径；本模块
不修改失败原因、来源时间戳、截止时间、驻留状态或 T0，也不授予任何正资格。
"""

import os

# 模块导入时只读一次；缺省 "0"（关闭）；非法值直接 raise，不静默降级。
_OPTION = os.environ.get("R106_PREQUALIFICATION_SOURCE_HOLD", "0")
if _OPTION not in ("0", "1"):
    raise ValueError("R106_PREQUALIFICATION_SOURCE_HOLD must be 0 or 1")
PREQUALIFICATION_SOURCE_HOLD_ENABLED = _OPTION == "1"
# 哨兵：调用方未传参表示“是否存在 typed 零事件未知”，与 None（确认不存在）区分。
UNSPECIFIED_ZERO_EVENT = object()
# 仅这两个目标包络类原因允许进入保持；两串与 ordinary_parking 的业务失败原因同源。
TARGET_ENVELOPE_REASONS = frozenset(
    ("parking_fixed_target_lost", "capture_fixed_goal_out_of_range")
)


def eligible(owner, reason, terminal_zero_event):
    """仅当零交接前的严格目标包络保持成立时返回 True。

    参数 owner 为 OrdinaryParking 实例，只读取其状态字段；reason 为业务失败原因；
    terminal_zero_event 为终端零事件，UNSPECIFIED_ZERO_EVENT 表示未知。所有预资格
    字段必须仍未置位、且不存在终端零事件，因此本判定不会重新解释一条已经开始的
    零路径。返回 True 时调用方仍走原有严格保护路径。
    """
    # 缺省参数表示未知，不能当作“已确认不存在 typed 事件”的证据。
    return bool(
        owner.prequalification_source_hold_enabled
        and reason in TARGET_ENVELOPE_REASONS
        and owner.enabled
        and owner.stage in ("capture", "controlled_deceleration")
        and owner.failure is None
        and owner.diagnostic_business_failure_reason is None
        and owner.diagnostic_business_observation_count == 0
        and terminal_zero_event is None
        and all(
            value is None
            for value in (
                owner.ready_stamp,
                owner.ready_clock,
                owner.handoff_clock,
                owner.first_zero_publication_clock,
                owner.first_actor_zero_ack,
            )
        )
    )
