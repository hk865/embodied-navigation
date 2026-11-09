"""默认关闭的 R74 公共 spin_once/零让出仿真实验。

它不是 Executor.spin 的通用等价替代：被选中的 executor 只有一个主循环
owner；公共 spin_once 在迭代之间与 sleep(0) 期间释放 is_spinning。禁止
启动第二个或嵌套 spinner。配置键为 ``r74_cooperative_executor``，未配置时
保持默认关闭。"""

import threading
import time

KEY = "r74_cooperative_executor"
SCHEMA = "r74_spin_once_zero_yield/v1"


def enabled(profile):
    # 只接受严格的布尔 enabled；字段缺失视为关闭，类型不对直接报错而不是
    # 猜测默认值。
    if not isinstance(profile, dict):
        raise ValueError("Controller settings must be an object")
    option = profile.get(KEY)
    if option is None:
        return False
    if not isinstance(option, dict) or type(option.get("enabled")) is not bool:
        raise ValueError("Controller feature enabled flag must be boolean")
    return option["enabled"]


def make_executor(base, profile, num_threads=2):
    # R74 本身不改变 executor 类型，只返回同一 base 的实例；真正替换 spin
    # 语义的实验在 cooperative_executor.py 中。num_threads 默认 2 由调用方决定。
    return base(num_threads=num_threads)
