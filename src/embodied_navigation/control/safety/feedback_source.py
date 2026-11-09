"""Source/time gate for canonical SLAM control feedback. No geometry thresholds."""


def require_canonical_slam_feedback(feedback):
    """Raise unless ``feedback`` is canonical non-truth SLAM state with valid clocks.

    Requires ``camera_init``, ``fast_livo2_async_backend`` and
    ``navigation_ground_truth_used is False``; the three clock stamps must be
    non-negative integers and the header may not be newer than its callback.
    """
    if (
        feedback.get("frame_id") != "camera_init"
        or feedback.get("navigation_ground_truth_used") is not False
        or feedback.get("pose_producer") != "fast_livo2_async_backend"
    ):
        raise ValueError("Control feedback is not accepted canonical SLAM state")
    # Header and callback use ROS time. Receipt uses monotonic wall time; never
    # subtract one clock domain from the other. Existing freshness gates remain.
    for key in ("stamp_ns", "callback_ros_clock_ns", "received_wall_ns"):
        if type(feedback.get(key)) is not int or feedback[key] < 0:
            raise ValueError("Invalid control feedback clock: " + key)
    if feedback["stamp_ns"] > feedback["callback_ros_clock_ns"]:
        raise ValueError("SLAM header is in the future of its ROS callback")
