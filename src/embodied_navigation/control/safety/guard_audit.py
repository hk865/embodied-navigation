"""截获未改动原生转向门里两次 `obstacle_ahead` 的调用结果。

职责：让调用方看到原生走廊检查的真实输入与返回值，用于审计/回执。
边界：本模块不做 I/O，也不额外做走廊评估；只用一个私有函数命名空间替换
`obstacle_ahead`，原生转向门的代码、默认参数与闭包均原样执行。
"""

from types import FunctionType


class NativeGuardAudit:
    """在只替换 `obstacle_ahead` 的前提下运行未改动的原生 guard。

    `__call__` 会清空调用记录、执行重建出来的函数，并要求恰好发生两次已审查的
    走廊检查，否则抛 `RuntimeError`；每次截获的条目是
    `(checked_target_copy, obstacle_ahead_result)`。构造依赖传入函数
    `__globals__["obstacle_ahead"]` 存在（隐式契约，见模块 README）。
    """

    def __init__(self, native_guard):
        """保存原生函数与其 `obstacle_ahead`，重建一个只换该名字的函数副本。"""
        self.calls = []
        self.native_obstacle = native_guard.__globals__["obstacle_ahead"]
        # 复制 globals 后只替换 obstacle_ahead；code/defaults/closure 全部沿用原对象，
        # 因此原生 guard 的默认值与闭包行为不变。
        namespace = dict(native_guard.__globals__)
        namespace["obstacle_ahead"] = self._capture
        self.instrumented = FunctionType(
            native_guard.__code__,
            namespace,
            native_guard.__name__,
            native_guard.__defaults__,
            native_guard.__closure__,
        )
        self.instrumented.__kwdefaults__ = native_guard.__kwdefaults__

    def _capture(self, cloud, pose, target, route, **kwargs):
        """原生 `obstacle_ahead` 的代理：先调用原函数，再记录参数与结果。

        `target` 会深拷贝一份（`target.copy()`），避免调用方后续修改影响审计记录。
        """
        result = self.native_obstacle(cloud, pose, target, route, **kwargs)
        self.calls.append((target.copy(), result))
        return result

    def __call__(self, cloud, pose, checked_target, steering_direction, route):
        """执行一次原生 guard；两次走廊检查没有都跑到就抛 `RuntimeError`。

        参数原样透传给原生函数：`cloud` 点云、`pose` 位姿、`checked_target`
        被检查目标、`steering_direction` 转向方向、`route` 路线。返回值即原生
        函数的返回值；调用记录可通过 `self.calls` 读取。
        """
        self.calls = []
        result = self.instrumented(cloud, pose, checked_target, steering_direction, route)
        if len(self.calls) != 2:
            raise RuntimeError(
                "Native steering guard did not execute its two reviewed corridor checks"
            )
        return result


def result_json(result):
    """把 `obstacle_ahead` 三元组结果转成 JSON 友好字典。

    `result` 约定为 `(blocked, clearance_m, point_count)`；`clearance_m` 可能为
    `None`（无有效量测），`point_count` 转成 `int`。本函数只做类型转换。
    """
    return {
        "blocked": bool(result[0]),
        "clearance_m": None if result[1] is None else float(result[1]),
        "point_count": int(result[2]),
    }
