"""Normal-source freshness adapters; diagnostic relaxation is not a runtime mode.

Normal navigation computes every freshness/staleness verdict itself from the
real sensor and clock receipts and passes that boolean in as ``original_*``.
The helpers below are pass-through adapters: they do not interpret their
arguments or apply any tolerance, timeout or clock conversion, and always return
the caller's own verdict, so normal-source validation cannot be weakened here.
"""


def enabled(profile):
    """Reject the legacy R73 diagnostic relaxation and always report disabled.

    ``profile`` is expected to be a mapping (callers pass a profile dict); this
    helper performs no isinstance guard of its own.  For
    ``r73_minimal_diagnostic_baseline``, a missing key/``None`` or a value
    comparing equal to ``{'enabled': False}`` is accepted.  This
    compatibility check uses equality, so ``{'enabled': 0}`` is also accepted.
    Non-equal values, including ``{'enabled': True}`` or a dict with extra keys,
    raise ``ValueError``.  The return value is always ``False``: normal navigation
    never enables the relaxation.
    """
    option = profile.get('r73_minimal_diagnostic_baseline')
    if option not in (None, {'enabled': False}):
        raise ValueError('Diagnostic source relaxation belongs to experiments, not normal navigation')
    return False


def wall_stale(node, field, now, receipt, limit, original_stale, **options):
    """Pass through the caller-computed wall-clock staleness verdict.

    ``original_stale`` is computed by the caller from the wall receipt/deadline
    comparison; this adapter returns it unchanged.  ``**options`` retains
    call-site compatibility (for example ``future_tolerance=``) and is ignored;
    only ``original_stale`` determines the result.
    """
    return original_stale


def wall_stale_ns(node, field, now_ns, receipt_ns, limit_ns, original_stale):
    """Nanosecond-integer twin of :func:`wall_stale`; returns the caller's verdict.

    The time arguments (``now_ns``, ``receipt_ns``, ``limit_ns``) are integer
    nanoseconds.  The caller already compared the integer window and nothing is
    recomputed here.
    """
    return original_stale


def raw_imu_fresh(node, now, ros_now, original_fresh):
    """Return the caller's raw-IMU freshness verdict unchanged.

    Callers pass ``node.raw_imu.fresh(now, ros_now)`` (or the equivalent
    already-computed boolean) as ``original_fresh``; no tolerance, timeout or
    clock conversion is applied here.
    """
    return original_fresh


def pose_cloud_fresh(node, source, now, ros_now, original_fresh):
    """Return the caller's pose/registered-cloud freshness verdict unchanged.

    ``source`` is the caller's selector (``'pose'`` or ``'cloud'``); it is not
    interpreted here, and the caller-computed ``original_fresh`` is returned
    as-is.
    """
    return original_fresh


def control_sources_stale(node, now, ros_now, original_stale):
    """Return the caller's combined control-source staleness verdict unchanged.

    Callers combine the pose/cloud/raw-IMU timeouts themselves and pass the
    resulting boolean as ``original_stale``; this adapter does not re-evaluate
    any source.
    """
    return original_stale
