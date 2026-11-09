"""在未改动的物理门之前附加一层输出限制。

职责：对 exact-zero 起步（上一实际命令平移恰为零）的 drive 帧，按原输出大小
给期望平移做等比上限，避免起步瞬间放大平移。
边界：不改变来源、控制器状态、路径或停止判据；yaw 与源时间不动，原生 guard
仍必须在后续链路执行。调用点是 `../../navigation/pid_support.py`。
"""

import math
import numpy as np


def constrain_exactzero_launch(previous, desired, original, mode, rotation_xy):
    """对 exact-zero 起步的本体平移设上限，且不放大原输出。

    参数：`previous` 上一实际命令、`desired` 期望本体命令、`original` 原
    after-slew 输出（3 维，含 yaw）；`mode` 为控制模式字符串；`rotation_xy`
    是 2x2 机体到世界方向的旋转。返回 `(constrained_command, receipt)`。

    yaw 与源时间保持不变；下游仍必须执行原生 guard。输入非有限、形状不符、
    方向相反或浮点舍入会放大输出时抛 `ValueError`。本函数无 I/O、无状态。
    """
    p, d, o = (np.asarray(v, dtype=float) for v in (previous, desired, original))
    r = np.asarray(rotation_xy, dtype=float)
    if (
        any(v.shape != (3,) or not np.isfinite(v).all() for v in (p, d, o))
        or r.shape != (2, 2)
        or not np.isfinite(r).all()
    ):
        raise ValueError("launch constraint requires finite exact current vectors")
    if not isinstance(mode, str) or not mode:
        raise ValueError("launch mode invalid")
    out = o.copy()
    alpha = None
    # applied 只在 drive 且上一实际命令平移严格为零（exact-zero 起步）时为真；
    # 其他情况一律原样返回原输出，reason 固定为 outside_exactzero_drive_launch。
    applied = mode == "drive" and bool(np.all(p[:2] == 0.0))
    reason = "outside_exactzero_drive_launch"
    if applied:
        if d[0] <= 0.0:
            alpha = 0.0
            out[:2] = 0.0
            reason = "no_positive_forward_launch_translation"
        else:
            if any(o[i] * d[i] < 0.0 for i in (0, 1)):
                raise ValueError("original launch output has opposing direction")
            # 等比上限：alpha = min(1, |original_i| / |desired_i|)，保持方向不变，
            # 同时保证约束后的分量不超过原输出；alpha 不会超过 1。
            ratios = [abs(o[i]) / abs(d[i]) for i in (0, 1) if d[i] != 0.0]
            alpha = min([1.0, *ratios])
            if not math.isfinite(alpha) or not 0.0 <= alpha <= 1.0:
                raise ValueError("launch scale invalid")
            out[:2] = alpha * d[:2]
            # 浮点乘法可能因舍入把分量放大到原输出之上；最多 4 次 nextafter
            # 向零收缩 alpha，若仍放大则抛错（宁拒绝也不放大）。
            for _ in range(4):
                if np.all(np.abs(out[:2]) <= np.abs(o[:2])):
                    break
                alpha = math.nextafter(alpha, 0.0)
                out[:2] = alpha * d[:2]
            if not np.all(np.abs(out[:2]) <= np.abs(o[:2])):
                raise ValueError("launch rounding would enlarge original output")
            reason = "uniform_direction_preserving_body_cap"
            # 这是既有的世界方向更新边界：旋转后平移 <=1e-9 视为低于更新门限，
            # 输出归零；不要改用另一个 guard 方向发布微小平移。
            if np.linalg.norm(r @ out[:2]) <= 1e-9:
                alpha = 0.0
                out[:2] = 0.0
                reason = "below_original_world_direction_update_boundary"
    # 回执固定声明：yaw 未变、源时间未刷新、未新增运动授权，下游原生 guard 仍需执行。
    receipt = dict(
        schema="exactzero_launch_output_constraint/v1",
        applied=applied,
        reason=reason,
        alpha=alpha,
        previous_actual_command=p.tolist(),
        desired_body_command=d.tolist(),
        original_after_slew=o.tolist(),
        constrained_body_command=out.tolist(),
        yaw_changed=False,
        source_times_refreshed=False,
        new_motion_authorization=False,
        original_native_guard_still_required=True,
    )
    return out, receipt
