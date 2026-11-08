"""Current numerical safety settings; no release, hash, or approval documents."""
import math

def _number(value, expected, name):
    if type(value) not in (int,float) or not math.isfinite(value) or value != expected:
        raise ValueError('Invalid safety setting: '+name)

def parking(value):
    if not isinstance(value,dict) or value.get('enabled') is not True:
        raise ValueError('Fixed-window parking must be enabled')
    for key,number in dict(duration_ns=5_000_000_000,capture_xy_m=.025,
                           capture_yaw_rad=.035,capture_dwell_ns=600_000_000,
                           source_age_ns=300_000_000).items(): _number(value.get(key),number,key)
    if value.get('reset_window') is not False:raise ValueError('Parking anchor must remain fixed')
    pose(value.get('pose10hz'))
    return True

def pose(value):
    if not isinstance(value,dict):raise ValueError('Pose sampling settings required')
    for key,number in dict(period_ns=100_000_000,duration_ns=5_000_000_000,
                           xy_drift_m=.05,yaw_drift_rad=.1,source_age_ns=300_000_000).items():
        _number(value.get(key),number,key)
    if (value.get('sampling')!='latest_distinct_header_at_or_before_grid'
        or value.get('interpolation') is not False or value.get('high_frequency_motion_gate') is not False
        or value.get('adjacent_increment_gate') is not False):raise ValueError('Actual causal pose sampling required')
    return True

def clock(value):
    if not isinstance(value,dict):raise ValueError('Clock protection settings required')
    _number(value.get('freshness_sim_and_wall_s'),.3,'clock source freshness')
    _number(value.get('continuous_stale_fail_wall_s'),8.,'continuous stale stop')
    if (value.get('same_clock_policy')!='exact_zero_before_math_and_native_geometry'
        or value.get('backward_clock_policy')!='latch_failure'
        or value.get('source_receipt_refresh_permitted') is not False):raise ValueError('Clock hold protection required')

def settle(value):
    if not isinstance(value,dict) or value.get('enabled') is not True:raise ValueError('Settle window required')
    for key,number in dict(window_s=1.,gyro_mean_norm_radps=.005,gyro_rms_norm_radps=.05,
        gyro_peak_norm_radps=.1,euler_peak_to_peak_rad=.003,euler_net_change_rad=.0015,
        xy_diameter_m=.003,xy_net_change_m=.002).items():_number(value.get(key),number,key)
    if value.get('eligible_phase')!='settle' or value.get('legacy_stop_predicate_retained') is not True:
        raise ValueError('Settle phase and measured stop protection required')
    return True
