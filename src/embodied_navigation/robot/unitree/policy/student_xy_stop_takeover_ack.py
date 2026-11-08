"""Same-frame ACK proof for the explicit owned-event XY stop mode.

The original Student contract remains a base identity.  This extension declares
new control state and a post-legacy-slew XY override; it does not grant quiet,
formal-zero, parking, fixed5, source-freshness, or Actor/GRU acceptance credit.
"""
import copy
import hashlib
import importlib.util
from pathlib import Path

import numpy as np

FLAG = 'xy_stop_takeover_enabled'
KEY = 'student_xy_stop_takeover'
REQUIRED = KEY + '_required'
SCHEMA = 'student_xy_stop_takeover/v1'
_helper_module = None


def split_configuration(value):
    """Strip only the typed selected mode flag before original strict checks."""
    if not isinstance(value, dict) or FLAG not in value:
        return value, False
    if type(value[FLAG]) is not bool:
        raise ValueError('XY stop mode selection requires an explicit boolean')
    original = copy.deepcopy(value)
    enabled = original.pop(FLAG)
    return original, enabled


def enabled(value):
    return isinstance(value, dict) and value.get(FLAG) is True


def _vector(value):
    out = np.asarray(value, dtype=float)
    if out.shape != (3,) or not np.isfinite(out).all():
        raise ValueError('Finite same-frame XY takeover vector required')
    return out


def _same(first, second):
    if not np.array_equal(_vector(first), _vector(second)):
        raise ValueError('Exact same-frame XY takeover command differs')


def helper(extension):
    """Load the actual adjacent helper after checking its claimed snapshot SHA.

    Runtime preparation must independently bind this helper and this ACK module
    in scope.references, SOURCE_BINDINGS and the run source archive.  A claimed
    helper hash by itself is deliberately not described as deployment proof.
    """
    global _helper_module
    identity = extension.get('helper_identity') if isinstance(extension, dict) else None
    if not isinstance(identity, dict):
        raise ValueError('Actual XY takeover helper identity missing')
    expected = identity.get('sha256')
    path = Path(__file__).with_name('student_xy_stop_takeover.py')
    if _helper_module is None:
        if not isinstance(expected, str) or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('Actual adjacent XY takeover helper SHA differs')
        spec = importlib.util.spec_from_file_location('actual_ack_xy_stop_takeover_helper', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        if module.SOURCE_SHA256 != expected:
            raise ValueError('Loaded XY takeover helper identity differs')
        _helper_module = module
    elif _helper_module.SOURCE_SHA256 != expected:
        raise ValueError('XY takeover helper changed within actual ACK process')
    return _helper_module


def validate_copies(first, second, config, scope, stamp, frame, requested,
                    applied, corrected, correction, pid=None):
    """Verify selected proof, its actual Actor input and the original PID shadow.

    Source/event/native-history and state recurrence checks belong to the
    independently bound helper's validate_receipt.  This function adds the
    existing ACK's same-frame inference binding and truthful residual split.
    """
    present = any(key in obj for obj in (first, second) for key in (KEY, REQUIRED))
    if not enabled(config):
        if present:
            raise ValueError('Disabled original ACK path cannot carry a takeover proof')
        return None
    if (first.get(REQUIRED) is not True or second.get(REQUIRED) is not True
            or not isinstance(first.get(KEY), dict) or first[KEY] != second.get(KEY)):
        raise ValueError('Selected XY stop mode requires identical PID/Actor/ACK proof')
    q = first[KEY]
    if (q.get('schema') != SCHEMA or q.get('enabled') is not True
            or q.get('new_control_math') is not True
            or q.get('base_PID_math_claim') is not False
            or q.get('stop_or_ACK_credit') is not False
            or q.get('pid_terms_are_shadow') is not True
            or type(q.get('active')) is not bool):
        raise ValueError('Explicit new XY mode/shadow/no-credit declaration differs')
    helper(q).validate_receipt(q, scope, stamp, frame, requested, pid=pid)
    if (q.get('scope_sha256') != scope
            or type(q.get('input_clock_ns')) is not int or q['input_clock_ns'] != stamp
            or type(q.get('input_frame_serial')) is not int or q['input_frame_serial'] != frame):
        raise ValueError('XY stop proof is not the actual ACK input frame')
    legacy = _vector(q.get('legacy_after_slew_shadow_body'))
    actual = _vector(q.get('actual_after_takeover_body'))
    _same(actual, applied)
    shadow = q.get('pid_before_stop_override')
    if not isinstance(shadow, dict):
        raise ValueError('Original pre-override PID evaluation shadow missing')
    _same(shadow.get('requested_command_body'), requested)
    _same(shadow.get('corrected_before_legacy_slew_body'), corrected)
    _same(shadow.get('pid_correction'), correction)
    if pid is not None:
        # The primary PID fields truthfully describe this single evaluation;
        # they are not mislabeled as the persistent state after the XY mode.
        if any(key not in pid or pid[key] != value for key, value in shadow.items()):
            raise ValueError('Primary PID terms differ from their declared shadow')
        _same(pid.get('final_after_legacy_slew_body'), actual)
    active = q['active']
    if q.get('bypassed_xy_axes') != [active, active, False] or q.get('yaw_unchanged') is not True:
        raise ValueError('XY mode axes or unchanged-yaw declaration differs')
    if actual[2] != legacy[2]:
        raise ValueError('XY takeover changed original yaw input')
    if active:
        if np.any(_vector(requested)[:2]) or np.any(actual[:2]):
            raise ValueError('Active XY takeover requires exact zero request and actual input XY')
    else:
        _same(actual, legacy)
    return q


def split_residual(corrected, applied, extension):
    """Separate original worker slew from the new explicit takeover effect."""
    corrected, applied = _vector(corrected), _vector(applied)
    if extension is None:
        return corrected - applied, None
    legacy = _vector(extension['legacy_after_slew_shadow_body'])
    return corrected - legacy, legacy - applied
