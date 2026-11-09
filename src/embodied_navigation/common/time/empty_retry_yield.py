"""默认关闭、锁定源码版本的 executor 空转重试协作实验。

这个隔离候选不 import ROS：只复制已安装 executor 的两个方法，并保留其原有
表达式不变。在安装方公共 spin_once(None) 内，每 8 次完整的“无 handler”循环
请求一次 100 µs sleep；实际 sleep 可能更长，shutdown 也可能与最后一次检查
竞争。sleep 放在原 timeout 检查之后、ExitStack 之外，其异常原样传播；
热路径不取时钟、不记录、不写日志、不做 I/O。

安装必须在首次 spin 之前；快照必须在 spinner 停止之后；恢复只允许在
shutdown 清空缓存的生成器之后，不支持运行中恢复或复用。
"""

from __future__ import annotations

import ast
import hashlib
import inspect
from pathlib import Path
import textwrap
import threading
import time
import types


EMPTY_RETRY_INTERVAL = 8  # 连续空转多少轮后请求一次 sleep
SLEEP_SECONDS = 0.0001  # 请求的 sleep 时长（s）
_ATTRIBUTE = "_r91_empty_retry_yield"
_MISSING = object()


def _statements(source):
    return ast.parse(source).body


def _compile(original, *, ready):
    # 直接从运行中 executor 的源码做 AST 改写：先断言关键表达式/结构仍是
    # 预期版本（否则报错拒绝安装），再注入计数与 sleep。返回值附带改写后
    # 源码及其 sha256，作为本次安装的源码证据。
    tree = ast.parse(textwrap.dedent(inspect.getsource(original)))
    function = tree.body[0]
    if ready:
        loops = [node for node in function.body if isinstance(node, ast.While)]
        assert len(loops) == 1
        loop = loops[0]
        assert (
            ast.unparse(loop.test)
            == "not yielded_work and (not self._is_shutdown) and (not condition())"
        )
        # 该节点位于 ExitStack 与所有 ready-entity 扫描之后；只有在这里插入
        # sleep 才不会改变原有资源清理与超时顺序。
        timeout_check = loop.body[-1]
        assert isinstance(timeout_check, ast.If)
        assert ast.unparse(timeout_check.body[0]) == "raise TimeoutException()"
        assert "timeout_nsec == 0" in ast.unparse(timeout_check.test)
        function.body[1:1] = _statements("_r91_state = self._r91_empty_retry_yield")
        loop.body += _statements("""
if _r91_state._active and not yielded_work and not self._is_shutdown:
    if _r91_get_ident() == _r91_state.owner_ident:
        _r91_state._empty_streak += 1
        if _r91_state._empty_streak == 8:
            _r91_state._empty_streak = 0
            _r91_sleep(0.0001)
            _r91_state.yield_count += 1
""")
    else:
        assert ast.unparse(function.body[0]) == "self._enter_spin()"
        assert len(function.body) == 2 and isinstance(function.body[1], ast.Try)
        scope = function.body[1]
        assert (
            len(scope.body) == 1
            and ast.unparse(scope.body[0]) == "self._spin_once_impl(timeout_sec)"
        )
        assert len(scope.finalbody) == 1 and ast.unparse(scope.finalbody[0]) == "self._exit_spin()"
        # _enter_spin 必须留在 try 之外：嵌套/并发进入被拒绝时不能清掉当前
        # 活跃调用方的状态；在它成功之前也不做任何状态修改。
        scope.body[0:0] = _statements("""
self._r91_empty_retry_yield._active = timeout_sec is None and _r91_get_ident() == self._r91_empty_retry_yield.owner_ident
self._r91_empty_retry_yield._empty_streak = 0
""")
        scope.finalbody[0:0] = _statements("""
self._r91_empty_retry_yield._active = False
self._r91_empty_retry_yield._empty_streak = 0
""")
    ast.fix_missing_locations(tree)
    namespace = dict(original.__globals__)
    namespace.update(_r91_get_ident=threading.get_ident, _r91_sleep=time.sleep)
    exec(compile(tree, original.__code__.co_filename + ":r91_empty_retry_yield", "exec"), namespace)
    source = ast.unparse(tree)
    return namespace[original.__name__], {
        "source": source,
        "sha256": hashlib.sha256(source.encode()).hexdigest(),
    }


class Candidate:
    """固定状态与一个累计干预计数，不保留事件历史。

    ``owner_ident``/``owner_tid`` 标识安装方线程；``yield_count`` 只统计
    “注入的 sleep 成功返回”次数，不是内核唤醒或已分发回调数。
    """

    __slots__ = (
        "enabled",
        "owner_ident",
        "owner_tid",
        "yield_count",
        "_active",
        "_empty_streak",
        "source",
        "transformed",
        "executor",
        "_originals",
        "restored",
    )

    def __init__(self, enabled):
        self.enabled = enabled
        self.owner_ident = threading.get_ident()
        self.owner_tid = threading.get_native_id()
        self.yield_count = 0
        self._active = False
        self._empty_streak = 0
        self.source = None
        self.transformed = {}
        self.executor = None
        self._originals = {}
        self.restored = False

    def snapshot(self):
        # 只允许在 spinner 停止后取快照，否则计数与状态可能在读取过程中变化。
        if self.executor is not None and self.executor.is_spinning:
            raise RuntimeError("snapshot requires stopped spinner")
        return dict(
            schema="r91_empty_retry_yield/v1",
            enabled=self.enabled,
            empty_retry_interval=EMPTY_RETRY_INTERVAL,
            sleep_seconds=SLEEP_SECONDS,
            yield_count=self.yield_count,
            yield_count_definition="successful returns from the injected sleep requesting 100 us; not kernel wakes or dispatched callbacks",
            owner_tid=self.owner_tid,
            source=self.source,
            transformed=self.transformed,
            restored=self.restored,
        )

    def restore(self):
        # 恢复前提：executor 已 shutdown 且缓存生成器已清空；运行中替换方法
        # 不受支持，直接拒绝而不是尝试热恢复。
        if self.executor is None or self.restored:
            return
        if not self.executor._is_shutdown or self.executor._cb_iter is not None:
            raise RuntimeError(
                "restore requires shutdown and cleared generator; live reuse unsupported"
            )
        for name, previous in self._originals.items():
            if previous is _MISSING:
                delattr(self.executor, name)
            else:
                setattr(self.executor, name, previous)
        self.restored = True


def install(executor, enabled=False):
    """enabled=False 时实例与所有原方法完全不变。

    启用时先校验 executor 未在 spin、未缓存生成器、未重复安装，并确认两个
    待改写方法来自同一安装文件；随后用 AST 生成替换方法并绑定到实例。
    """
    candidate = Candidate(enabled)
    if not enabled:
        return candidate
    if executor.is_spinning or getattr(executor, "_cb_iter", None) is not None:
        raise RuntimeError("install before first spin on a stopped executor")
    if hasattr(executor, _ATTRIBUTE):
        raise RuntimeError("candidate already installed")
    originals = {
        name: getattr(executor, name).__func__
        for name in ("_wait_for_ready_callbacks", "spin_once")
    }
    path = Path(inspect.getfile(originals["_wait_for_ready_callbacks"]))
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if any(Path(inspect.getfile(f)) != path for f in originals.values()):
        raise RuntimeError("unsupported installed executor implementation")
    replacements = {}
    for name, function in originals.items():
        replacement, audit = _compile(function, ready=name == "_wait_for_ready_callbacks")
        replacements[name] = replacement
        candidate.transformed[name] = audit
    candidate.source = dict(path=str(path), sha256=digest)
    candidate.executor = executor
    for name in (*replacements, _ATTRIBUTE):
        candidate._originals[name] = executor.__dict__.get(name, _MISSING)
    setattr(executor, _ATTRIBUTE, candidate)
    for name, function in replacements.items():
        setattr(executor, name, types.MethodType(function, executor))
    return candidate
