"""R74 setup-only public /clock callback-group routing for minimal simulation.

No clock producer, timestamp transformation, timer, private TimeSource access or
per-message observer is installed here. The original TimeSource callback/QoS
and its strict time/source consumers keep their values and rules.
"""

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
    # Other topics and unselected ordinary /clock calls pass through unchanged.
    if topic != "/clock" or not enabled(getattr(node, "profile", None)):
        return options
    if options.get("callback_group") is not None:
        raise ValueError("Do not replace an explicitly grouped clock subscription")
    group = getattr(node, "_r74_clock_callback_group", None)
    if group is None:
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

        group = MutuallyExclusiveCallbackGroup()
        node._r74_clock_callback_group = group
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
