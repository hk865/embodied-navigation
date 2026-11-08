"""Explicit disabled observation port for the normal runtime.

No experiment package is imported, no timer/thread/file is created here. Business
operations passed through an observation hook are still invoked exactly once.
The optional historical observers live outside src, under experiments.
"""
from contextlib import nullcontext

def _nothing(*args,**kwargs):return None
def _disabled(*args,**kwargs):return False
def _decorate(*args,**kwargs):return lambda function:function
def _parsed(context,value):return value
def _consumer_call(node,ns,name,operation,*args):return operation(*args)
def _receipt_copy(function,receipt,*args,**kwargs):return function(receipt)
def _lock(original,*args,**kwargs):return original
def _summary(*args,**kwargs):return None
def _unsupported(*args,**kwargs):
    raise ValueError('This diagnostic mode belongs to experiments, outside normal navigation')
class _UnavailableObserver:
    def __init__(self,*args,**kwargs):_unsupported()

class DisabledObservation:
    ENABLED=False
    OPTION=READ_OPTION='0'
    RECORDER=PROBE=_binding=None
    STAGES=()
    enabled=selected=staticmethod(_disabled)
    observe=staticmethod(_decorate)
    span=zone=staticmethod(lambda *args,**kwargs:nullcontext())
    lock=staticmethod(_lock)
    parsed=staticmethod(_parsed)
    consumer_call=staticmethod(_consumer_call)
    receipt_copy=staticmethod(_receipt_copy)
    summary=staticmethod(_summary)
    create=setup=install=detach=finish=flush=flush_once=staticmethod(_nothing)
    create_probe=retry_published=staticmethod(_nothing)
    CorridorObserver=_UnavailableObserver
    decode_snapshot=sha_text=staticmethod(_unsupported)
    bind=mark=anomaly=note=record=trigger=read_before=future_wait=hard_fault=staticmethod(_nothing)
    begin=end=outer=ignored=associating=associated=exception=accepting=returned=done=staticmethod(_nothing)
    consumer_entry=consumer_arrival=consumer_finish=staticmethod(_nothing)
    producer_begin=producer_stage=producer_metadata=producer_finish=staticmethod(_nothing)
    controller=bridge_received=envelope=reader_result=staticmethod(_nothing)
    odom_callback=cloud_callback=parking_protection=staticmethod(_nothing)
    sample=guard_window_begin=guard_window_end=staticmethod(_nothing)
    Policy=None

_DISABLED=DisabledObservation()
def api(name):
    """Return the normal no-recording port; names identify former hook sites."""
    return _DISABLED
