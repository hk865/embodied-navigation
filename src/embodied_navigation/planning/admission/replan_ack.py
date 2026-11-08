"""Default-OFF acknowledgement of a completed early local replan request.

Only the original synchronous request's successful return acknowledges it.
This switch changes no source, planner admission, retry interval or motion gate.
"""
import os

KEY = 'R103_EARLY_REQUEST_ACK'
_OPTION = os.environ.get(KEY, '0')
if _OPTION not in ('0', '1'):
    raise ValueError(KEY + ' must be 0 or 1')
_ENABLED = _OPTION == '1'


def enabled():
    return _ENABLED
