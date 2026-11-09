"""默认关闭的唯一主 spinner 正向让出实验。

它只改变调度时序，不改变回调所有权或控制计算：每次成功的公共 spin_once
之后请求一次 100 µs sleep，该 sleep 与实际调度延迟都没有实时上界。
开关由环境变量 ``R90_SPINNER_POSITIVE_YIELD``（只接受 "0"/"1"）控制，
并要求同时启用 R74 的唯一 owner executor。"""

from collections import deque
import json
import os
import threading
import time

import embodied_navigation.common.time.executor_selection as original

KEY = "R90_SPINNER_POSITIVE_YIELD"
YIELD_SECONDS = 0.0001  # 每次让出请求的 sleep 时长（s）
SLOW_NS = 50_000_000  # 单次 spin_once 墙钟超过 50 ms 记一条慢调用样本
SLOW_CAPACITY = 64  # 慢调用环形样本上限，超出后淘汰最旧记录


def selected():
    # 只接受 "0"/"1"；其他取值直接报错，避免拼写错误被静默当成关闭。
    value = os.environ.get(KEY, "0")
    if value not in ("0", "1"):
        raise ValueError(KEY + " must be 0 or 1")
    return value == "1"


def make_executor(base, profile, num_threads=2):
    """返回被选中的 executor；R90 只额外包一层正向让出的 spin 循环。

    R74 未启用时拒绝启动 R90（它依赖唯一 owner 前提）；启用时返回
    ``PositiveYieldExecutor``，其统计在 ``r90_statistics()`` 中取出。
    """
    if not selected():
        return original.make_executor(base, profile, num_threads=num_threads)
    if not original.enabled(profile):
        raise ValueError("R90 requires the selected sole-owner R74 executor")

    class PositiveYieldExecutor(base):
        _r90_positive_yield_seconds = YIELD_SECONDS

        def __init__(self, *args, **kwargs):
            # stop 事件与慢调用环形缓冲都跨 spin 循环保留；统计字段在
            # shutdown 之后仍可读取，供收据落盘。
            self._r74_cooperative_stop = threading.Event()
            self._r90_slow = deque(maxlen=SLOW_CAPACITY)
            self._r90_stats = dict(
                spin_calls=0,
                spin_wall_ns=0,
                spin_cpu_ns=0,
                spin_wall_max_ns=0,
                spin_cpu_max_ns=0,
                spin_exceptions=0,
                slow_records=0,
                slow_evicted=0,
                yield_calls=0,
                yield_wall_ns=0,
                yield_wall_max_ns=0,
                spinner_native_tid=None,
                observer_errors=0,
            )
            super().__init__(*args, **kwargs)

        def spin(self):
            # 唯一 spinner：记录原生线程 id，循环直到 context 失效或 shutdown
            # 置位；异常计数后原样重抛，不吞掉业务异常。
            stats = self._r90_stats
            stats["spinner_native_tid"] = threading.get_native_id()
            while self.context.ok() and not self._r74_cooperative_stop.is_set():
                begin = cpu = None
                try:
                    begin, cpu = time.monotonic_ns(), time.thread_time_ns()
                except Exception:
                    stats["observer_errors"] += 1
                try:
                    self.spin_once()
                except BaseException:
                    stats["spin_exceptions"] += 1
                    raise
                finally:
                    stats["spin_calls"] += 1
                    # 观测本身失败只累加 observer_errors，不影响 spin 结果；
                    # 慢调用样本在超过 SLOW_NS 时记录并计入淘汰数。
                    try:
                        if begin is not None and cpu is not None:
                            cpu_elapsed = time.thread_time_ns() - cpu
                            end = time.monotonic_ns()
                            elapsed = end - begin
                            stats["spin_wall_ns"] += elapsed
                            stats["spin_cpu_ns"] += cpu_elapsed
                            stats["spin_wall_max_ns"] = max(stats["spin_wall_max_ns"], elapsed)
                            stats["spin_cpu_max_ns"] = max(stats["spin_cpu_max_ns"], cpu_elapsed)
                            if elapsed >= SLOW_NS:
                                stats["slow_records"] += 1
                                stats["slow_evicted"] += int(len(self._r90_slow) == SLOW_CAPACITY)
                                self._r90_slow.append(
                                    dict(
                                        begin_ns=begin,
                                        end_ns=end,
                                        wall_ns=elapsed,
                                        thread_cpu_ns=cpu_elapsed,
                                    )
                                )
                    except Exception:
                        stats["observer_errors"] += 1
                # 每轮成功后主动 sleep 100 µs，让出 CPU；这是实验的“正向让出”，
                # 时长不保证精确，也不构成实时性承诺。
                begin = None
                try:
                    begin = time.monotonic_ns()
                except Exception:
                    stats["observer_errors"] += 1
                time.sleep(YIELD_SECONDS)
                stats["yield_calls"] += 1
                try:
                    if begin is not None:
                        elapsed = time.monotonic_ns() - begin
                        stats["yield_wall_ns"] += elapsed
                        stats["yield_wall_max_ns"] = max(stats["yield_wall_max_ns"], elapsed)
                except Exception:
                    stats["observer_errors"] += 1

        def shutdown(self, *args, **kwargs):
            self._r74_cooperative_stop.set()
            return super().shutdown(*args, **kwargs)

        def r90_statistics(self):
            return dict(
                self._r90_stats,
                slow_intervals=list(self._r90_slow),
                slow_capacity=SLOW_CAPACITY,
                slow_threshold_ns=SLOW_NS,
            )

    return PositiveYieldExecutor(num_threads=num_threads)


def receipt(executor):
    """报告实际拥有唯一 spinner 的 executor，以及 R90 是否改变时序。

    ``enabled`` 以实例上是否存在 R90 标记为准，而不是只看环境变量；
    ``timing_change_only=True`` 声明本次干预不改变回调所有权。"""
    actual = getattr(executor, "_r90_positive_yield_seconds", None)
    return dict(
        environment_key=KEY,
        environment_value=os.environ.get(KEY, "0"),
        enabled=actual is not None,
        yield_seconds=actual,
        actual_executor_class=type(executor).__qualname__,
        actual_executor_module=type(executor).__module__,
        worker_threads=getattr(getattr(executor, "_executor", None), "_max_workers", None),
        sole_spinner_owner=True,
        timing_change_only=True,
    )


def finish(node, executor):
    """仅在退出时尽力写诊断文件，不改变 shutdown 结果。

    写 ``<run>/r90_spinner_statistics.json``；任何异常都被吞掉并只置
    ``node._r90_statistics_write_failed``，调用方原有退出结果仍为准。"""
    try:
        if not hasattr(executor, "r90_statistics"):
            return
        value = dict(
            schema="r90_positive_yield_statistics/v1",
            selection=receipt(executor),
            statistics=executor.r90_statistics(),
        )
        (node.run / "r90_spinner_statistics.json").write_text(json.dumps(value) + "\n")
    except Exception:
        # 诊断写盘失败时，调用方原有退出结果仍然为准。
        try:
            node._r90_statistics_write_failed = True
        except Exception:
            pass
