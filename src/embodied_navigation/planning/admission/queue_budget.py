"""Fixed business FIFO budget, independent of diagnostic packages."""
import os
KEY="R97_DIAGNOSTIC_QUEUE_BYTES"
ORIGINAL_BYTES=2*1024*1024
MAX_DIAGNOSTIC_BYTES=4*1024*1024
def selection(profile):
    raw=os.environ.get(KEY)
    if raw is not None and raw != str(ORIGINAL_BYTES):
        raise ValueError('Normal navigation requires the original 2MiB intake queue')
    return dict(environment_key=KEY,environment_value=raw,requested_bytes=None if raw is None else ORIGINAL_BYTES,
        effective_bytes=ORIGINAL_BYTES,original_shadow_bytes=ORIGINAL_BYTES,
        maximum_diagnostic_bytes=MAX_DIAGNOSTIC_BYTES,diagnostic_enabled=False,
        diagnostic_simulation_authorized=False,observation_enabled=False,
        max_events=32,max_event_bytes=1024*1024,control_or_navigation_thresholds_changed=False)
