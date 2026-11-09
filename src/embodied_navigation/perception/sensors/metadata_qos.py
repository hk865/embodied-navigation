"""corrected metadata 订阅深度的选择器。

职责：读取 profile 中 r74_corrected_metadata_latest_only 开关，给出 corrected
metadata 话题的订阅深度。启用时为 1（只保留最新一条样本），未启用或缺键时为 10
（普通 typed QoS 深度）。

边界：只决定深度，不改变话题名、可靠性或持久性；本模块不导入 ROS，也不在运行期
重读配置。被 localization_mapping/known_scene_inputs.py 用于创建 corrected metadata
订阅。
"""

# profile 键名；当前 configs/control/settings.json 以 enabled=true、
# schema=r74_corrected_metadata_keep_last1_delivery/v1 启用它。
KEY = "r74_corrected_metadata_latest_only"


def subscription_depth(profile):
    """返回 corrected metadata 的订阅深度。

    参数：
        profile：本次 run 的业务 profile（dict）；只读取 KEY 一个键。
    返回：
        1 表示 latest-only（启用），10 表示普通深度（未启用或缺键）。
    失败：
        KEY 存在但不是 dict，或其 enabled 不是严格 bool 时抛 ValueError；
        schema 等其它字段在这里不校验。
    """
    option = profile.get(KEY)
    if option is None:
        # 缺键与显式关闭共用 10：硬编码的普通 typed QoS 深度，代码与现有文档未说明
        # 该数值的选取依据。
        return 10
    if not isinstance(option, dict) or type(option.get("enabled")) is not bool:
        raise ValueError("Latest-metadata option must be boolean")
    # depth=1 对应配置 schema 名里的 keep_last1：订阅端只保留最新一条。
    return 1 if option["enabled"] else 10
