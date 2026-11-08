"""Default-off R74 public spin_once/zero-yield simulation experiment.

This is not a generally equivalent replacement for Executor.spin. The selected
executor has one sole main-loop owner; public spin_once releases is_spinning
between iterations and during sleep(0). No second or nested spinner is allowed.
"""
import threading
import time

KEY = 'r74_cooperative_executor'
SCHEMA = 'r74_spin_once_zero_yield/v1'


def enabled(profile):
    if not isinstance(profile,dict):raise ValueError('Controller settings must be an object')
    option=profile.get(KEY)
    if option is None:return False
    if not isinstance(option,dict)or type(option.get('enabled'))is not bool:
        raise ValueError('Controller feature enabled flag must be boolean')
    return option['enabled']


def make_executor(base,profile,num_threads=2):
    return base(num_threads=num_threads)
