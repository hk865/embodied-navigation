"""Default-OFF, source-pinned experimental executor empty-retry cooperation.

This isolated candidate never imports ROS.  It copies only two installed methods,
leaving their original expressions intact.  Every eight complete no-handler loops
inside the installing owner's public spin_once(None) request a 100 us sleep.
Actual sleep duration can be longer; shutdown can race with the final check.
The sleep is after the original timeout check and outside ExitStack.  Its failure
propagates unchanged.  There are no hot-path clocks, records, logging or I/O.

Install before first spin. Snapshot after the spinner stops. Restore only after
shutdown clears the cached generator; live restoration/reuse is unsupported.
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


EMPTY_RETRY_INTERVAL = 8
SLEEP_SECONDS = 0.0001
_ATTRIBUTE = '_r91_empty_retry_yield'
_MISSING = object()


def _statements(source):
    return ast.parse(source).body


def _compile(original, *, ready):
    tree = ast.parse(textwrap.dedent(inspect.getsource(original)))
    function = tree.body[0]
    if ready:
        loops = [node for node in function.body if isinstance(node, ast.While)]
        assert len(loops) == 1
        loop = loops[0]
        assert ast.unparse(loop.test) == 'not yielded_work and (not self._is_shutdown) and (not condition())'
        # This final original node is after ExitStack and all ready-entity scans.
        timeout_check = loop.body[-1]
        assert isinstance(timeout_check, ast.If)
        assert ast.unparse(timeout_check.body[0]) == 'raise TimeoutException()'
        assert 'timeout_nsec == 0' in ast.unparse(timeout_check.test)
        function.body[1:1] = _statements('_r91_state = self._r91_empty_retry_yield')
        loop.body += _statements('''
if _r91_state._active and not yielded_work and not self._is_shutdown:
    if _r91_get_ident() == _r91_state.owner_ident:
        _r91_state._empty_streak += 1
        if _r91_state._empty_streak == 8:
            _r91_state._empty_streak = 0
            _r91_sleep(0.0001)
            _r91_state.yield_count += 1
''')
    else:
        assert ast.unparse(function.body[0]) == 'self._enter_spin()'
        assert len(function.body) == 2 and isinstance(function.body[1], ast.Try)
        scope = function.body[1]
        assert len(scope.body) == 1 and ast.unparse(scope.body[0]) == 'self._spin_once_impl(timeout_sec)'
        assert len(scope.finalbody) == 1 and ast.unparse(scope.finalbody[0]) == 'self._exit_spin()'
        # Keep _enter_spin outside try: a rejected nested/concurrent entry cannot
        # clear the active caller's scope. No state changes precede its success.
        scope.body[0:0] = _statements('''
self._r91_empty_retry_yield._active = timeout_sec is None and _r91_get_ident() == self._r91_empty_retry_yield.owner_ident
self._r91_empty_retry_yield._empty_streak = 0
''')
        scope.finalbody[0:0] = _statements('''
self._r91_empty_retry_yield._active = False
self._r91_empty_retry_yield._empty_streak = 0
''')
    ast.fix_missing_locations(tree)
    namespace = dict(original.__globals__)
    namespace.update(_r91_get_ident=threading.get_ident, _r91_sleep=time.sleep)
    exec(compile(tree, original.__code__.co_filename + ':r91_empty_retry_yield', 'exec'), namespace)
    source = ast.unparse(tree)
    return namespace[original.__name__], {
        'source': source, 'sha256': hashlib.sha256(source.encode()).hexdigest()}


class Candidate:
    """Fixed state and one cumulative intervention count; no event history."""
    __slots__ = ('enabled', 'owner_ident', 'owner_tid', 'yield_count', '_active',
                 '_empty_streak', 'source', 'transformed', 'executor',
                 '_originals', 'restored')

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
        if self.executor is not None and self.executor.is_spinning:
            raise RuntimeError('snapshot requires stopped spinner')
        return dict(schema='r91_empty_retry_yield/v1', enabled=self.enabled,
                    empty_retry_interval=EMPTY_RETRY_INTERVAL,
                    sleep_seconds=SLEEP_SECONDS, yield_count=self.yield_count,
                    yield_count_definition='successful returns from the injected sleep requesting 100 us; not kernel wakes or dispatched callbacks',
                    owner_tid=self.owner_tid, source=self.source,
                    transformed=self.transformed, restored=self.restored)

    def restore(self):
        if self.executor is None or self.restored:
            return
        if not self.executor._is_shutdown or self.executor._cb_iter is not None:
            raise RuntimeError('restore requires shutdown and cleared generator; live reuse unsupported')
        for name, previous in self._originals.items():
            if previous is _MISSING:
                delattr(self.executor, name)
            else:
                setattr(self.executor, name, previous)
        self.restored = True


def install(executor, enabled=False):
    """OFF leaves the instance and all original methods exactly unchanged."""
    candidate = Candidate(enabled)
    if not enabled:
        return candidate
    if executor.is_spinning or getattr(executor, '_cb_iter', None) is not None:
        raise RuntimeError('install before first spin on a stopped executor')
    if hasattr(executor, _ATTRIBUTE):
        raise RuntimeError('candidate already installed')
    originals = {name: getattr(executor, name).__func__
                 for name in ('_wait_for_ready_callbacks', 'spin_once')}
    path = Path(inspect.getfile(originals['_wait_for_ready_callbacks']))
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if any(Path(inspect.getfile(f)) != path for f in originals.values()):
        raise RuntimeError('unsupported installed executor implementation')
    replacements = {}
    for name, function in originals.items():
        replacement, audit = _compile(function, ready=name == '_wait_for_ready_callbacks')
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
