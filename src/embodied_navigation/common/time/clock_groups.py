"""R74 仅装配期的 /clock 回调组路由（最小仿真）。

这里不安装时钟生产者、不做时间戳变换、不建 timer、不访问私有 TimeSource、
也不加逐消息观察者。原 TimeSource 回调/QoS 及其严格时间/来源消费方保持
原有取值与规则；路由只影响 /clock 订阅挂在哪个回调组。"""

from embodied_navigation.common.input_freshness import enabled as minimal_baseline_enabled

KEY = "r74_independent_ros_clock"
CONFIG = {"enabled": True, "schema": "r74_independent_clock_callback_group/v1"}


def enabled(profile):
    if not isinstance(profile, dict):
        raise ValueError("Controller settings must be an object")
    option = profile.get(KEY)
    if option is None:
        return False
    if not isinstance(option, dict) or type(option.get("enabled")) is not bool:
        raise ValueError("Controller feature enabled flag must be boolean")
    return option["enabled"]


def subscription_options(node, topic, options):
    # 其他话题、以及未启用 R74 时的普通 /clock 调用都原样返回；只有启用了
    # r74_independent_ros_clock 且话题恰为 /clock 时才分配专用回调组。
    if topic != "/clock" or not enabled(getattr(node, "profile", None)):
        return options
    # 调用方已显式指定回调组时不覆盖，直接拒绝，避免两套分组语义叠加。
    if options.get("callback_group") is not None:
        raise ValueError("Do not replace an explicitly grouped clock subscription")
    group = getattr(node, "_r74_clock_callback_group", None)
    if group is None:
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

        group = MutuallyExclusiveCallbackGroup()
        node._r74_clock_callback_group = group
        # 收据只证明“装配期做了什么”，clock_messages_observed=False 明确它
        # 不能当作消息已收到的证据。
        node._r74_clock_setup_receipt = dict(
            schema=CONFIG["schema"],
            enabled=True,
            topic="/clock",
            callback_group="dedicated_MutuallyExclusiveCallbackGroup",
            setup_only=True,
            clock_messages_observed=False,
            original_callback_QoS_timestamps_preserved=True,
            no_new_subscription_or_clock_producer=True,
        )
    routed = dict(options)
    routed["callback_group"] = group
    return routed
