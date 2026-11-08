"""Default-OFF replacement admission for a recorded exhausted drive path."""
import os

KEY = 'R103_EXHAUSTED_DRIVE_REPLACEMENT'
_OPTION = os.environ.get(KEY, '0')
if _OPTION not in ('0', '1'):
    raise ValueError(KEY + ' must be 0 or 1')
_ENABLED = _OPTION == '1'


def enabled():
    return _ENABLED
