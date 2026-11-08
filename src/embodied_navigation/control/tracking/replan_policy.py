"""Distinguish a protected command zero from exhausted SCAN geometry.

Used only by the independent V24 Teacher controller. This does not authorize
movement, extend freshness, infer arrival, or alter the explicit native path
end/obstacle/constraint-replan handling.
"""


def low_command_is_exhausted_path(*, cascade_mode, exhausted, has_geometric_progress):
    """The caller already checked alignment, command size and request age.

    Progress here is the existing checked-spline arc/extent predicate, not a
    timer or a prediction that the physical robot has progressed. Unknown or
    protected modes never justify replacing an active route from command zero.
    """
    if type(exhausted) is not bool or type(has_geometric_progress) is not bool:
        raise ValueError('Actual checked path exhaustion/progress booleans required')
    if cascade_mode not in ('drive', 'path_end_hold'):
        return False
    # The unchanged cascade emits this mode only after the measured projection
    # has <.05m checked arc remaining and the endpoint is not the mission goal.
    # Its existing geometric stop threshold must not wait for a different
    # legacy steering endpoint threshold before asking for the next path.
    if cascade_mode == 'path_end_hold':
        return True
    return exhausted or not has_geometric_progress
