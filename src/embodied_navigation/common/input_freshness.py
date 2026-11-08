"""Normal-source freshness adapters; diagnostic relaxation is not a runtime mode."""
def enabled(profile):
    option=profile.get('r73_minimal_diagnostic_baseline')
    if option not in (None,{'enabled':False}):
        raise ValueError('Diagnostic source relaxation belongs to experiments, not normal navigation')
    return False

def wall_stale(node,field,now,receipt,limit,original_stale,**options):return original_stale
def wall_stale_ns(node,field,now_ns,receipt_ns,limit_ns,original_stale):return original_stale
def raw_imu_fresh(node,now,ros_now,original_fresh):return original_fresh
def pose_cloud_fresh(node,source,now,ros_now,original_fresh):return original_fresh
def control_sources_stale(node,now,ros_now,original_stale):return original_stale
