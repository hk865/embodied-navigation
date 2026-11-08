"""Capture the two results of the unchanged native steering guard.

This module performs no I/O and no additional corridor evaluation. A private
function namespace intercepts only the two original obstacle_ahead calls. The
original steering guard code, defaults and closure execute unchanged.
"""
from types import FunctionType


class NativeGuardAudit:
    def __init__(self, native_guard):
        self.calls = []
        self.native_obstacle = native_guard.__globals__['obstacle_ahead']
        namespace = dict(native_guard.__globals__)
        namespace['obstacle_ahead'] = self._capture
        self.instrumented = FunctionType(native_guard.__code__, namespace,
            native_guard.__name__, native_guard.__defaults__, native_guard.__closure__)
        self.instrumented.__kwdefaults__ = native_guard.__kwdefaults__

    def _capture(self, cloud, pose, target, route, **kwargs):
        result = self.native_obstacle(cloud, pose, target, route, **kwargs)
        self.calls.append((target.copy(), result))
        return result

    def __call__(self, cloud, pose, checked_target, steering_direction, route):
        self.calls = []
        result = self.instrumented(cloud, pose, checked_target, steering_direction, route)
        if len(self.calls) != 2:
            raise RuntimeError('Native steering guard did not execute its two reviewed corridor checks')
        return result


def result_json(result):
    return {'blocked': bool(result[0]), 'clearance_m': None if result[1] is None else float(result[1]),
            'point_count': int(result[2])}
