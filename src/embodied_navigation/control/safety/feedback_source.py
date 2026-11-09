"""canonical SLAM 控制反馈的来源/时间门；不含任何几何阈值。

职责：只校验停车反馈的身份与时间域，不判断位置误差、漂移或新鲜度。
边界：不做几何计算，不修改反馈，不写盘；新鲜度仍由其他既有门负责。
唯一调用点是 `../parking/parking_pose10hz.from_feedback`。
"""


def require_canonical_slam_feedback(feedback):
    """若 `feedback` 不是合法的非真值 canonical SLAM 状态则抛 `ValueError`。

    要求：`frame_id == "camera_init"`、`pose_producer ==
    "fast_livo2_async_backend"`、`navigation_ground_truth_used is False`
    （必须是布尔 False，缺省或字符串都不接受）。三个时间戳
    `stamp_ns` / `callback_ros_clock_ns` / `received_wall_ns` 必须是非负
    `int`（类型严格，`bool` 也不算），且 header 不得晚于其 ROS 回调时刻。
    失败即抛异常，由调用方决定停车；本函数无副作用。
    """
    if (
        feedback.get("frame_id") != "camera_init"
        or feedback.get("navigation_ground_truth_used") is not False
        or feedback.get("pose_producer") != "fast_livo2_async_backend"
    ):
        raise ValueError("Control feedback is not accepted canonical SLAM state")
    # header 与 callback 同属 ROS 时间域；receipt 是单调墙钟。两者不能相减，
    # 跨域比较必须显式转换（见 docs/glossary.md 的“时钟域”）。既有新鲜度门不变。
    for key in ("stamp_ns", "callback_ros_clock_ns", "received_wall_ns"):
        if type(feedback.get(key)) is not int or feedback[key] < 0:
            raise ValueError("Invalid control feedback clock: " + key)
    if feedback["stamp_ns"] > feedback["callback_ros_clock_ns"]:
        raise ValueError("SLAM header is in the future of its ROS callback")
