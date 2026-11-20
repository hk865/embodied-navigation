"""Deterministic observations and baseline outputs for diagnostic-only changes."""
from embodied_navigation.simulation.dynanav.pct_scan_waypoints import (
    ContinuousScanFollower, atomic_json,
)

IDENTITY = dict(run_id="diagnostic-acceptance", seq=7, frame_id="camera_init",
                child_frame_id="body", pose_source="livo_post_vio")


def make_follower(mailbox):
    return ContinuousScanFollower(mailbox, IDENTITY, [4., 0., 0.],
                                  speed=1., motion_end=None)


def path_payload(revision, provenance=None, **changes):
    row = dict(version=1, **IDENTITY, revision=revision, status="tracking",
               source_stamp_s=10. + revision * .1,
               points_xyz_m=[[0., 0., 0.], [1., .1, 0.], [4., 0., 0.]],
               provenance=provenance or dict(local_planner="native_SCAN",
                                             global_planner="native_PCT_Astar"))
    row.update(changes)
    return row


def command_sequence(mailbox):
    follower = make_follower(mailbox)
    steps = [
        (None, 10., [0., 0., 0.], 0., 0.),
        (path_payload(0), 10.1, [.1, .05, 0.], .08, .2),
        (None, 10.2, [.3, .08, 0.], .08, .2),
        (path_payload(1, points_xyz_m=[[.2, 0., 0.], [1., .1, 0.], [4., 0., 0.]]),
         10.3, [.4, .05, 0.], .08, .2),
        (path_payload(2, status="hold", reason="native_scan_emergency_stop"),
         10.4, [.4, .05, 0.], .08, .2),
        (path_payload(3), 10.5, [.5, .05, 0.], .08, .2),
        (None, 10.6, [3.9, 0., 0.], 0., .03),
        (None, 11.2, [3.9, 0., 0.], 0., .03),
    ]
    result = []
    for payload, stamp, position, yaw, measured_speed in steps:
        if payload is not None:
            atomic_json(mailbox / "planner_path.json", payload)
        command = follower.tick(stamp, position, yaw, measured_speed, 0.)
        result.append(dict(command=command, phase=follower.phase, failure=follower.failure,
                           arrived=follower.arrived, revision=follower.revision))
    return result

# Captured from fixed 511cde3 before implementation review.
BASELINE_COMMANDS = [{'command': [0.0, 0.0, 0.0], 'phase': 'planning', 'failure': None, 'arrived': False, 'revision': -1}, {'command': [1.0, -0.019896894568307663, -0.07485076252496863], 'phase': 'cruise', 'failure': None, 'arrived': False, 'revision': 0}, {'command': [1.0, -0.02487111821038458, -0.09722683190787373], 'phase': 'cruise', 'failure': None, 'arrived': False, 'revision': 0}, {'command': [1.0, -0.012391274375984815, -0.021122934757035795], 'phase': 'cruise', 'failure': None, 'arrived': False, 'revision': 1}, {'command': [0.0, 0.0, 0.0], 'phase': 'planning', 'failure': None, 'arrived': False, 'revision': 2}, {'command': [1.0, -0.0, 0.01573492199292962], 'phase': 'cruise', 'failure': None, 'arrived': False, 'revision': 3}, {'command': [0.0, 0.0, 0.0], 'phase': 'brake', 'failure': None, 'arrived': False, 'revision': 3}, {'command': [0.0, 0.0, 0.0], 'phase': 'arrived', 'failure': None, 'arrived': True, 'revision': 3}]


def large_diagnostic_values(root):
    """Find retained large evidence values in the tracker-owned Python graph."""
    seen, found = set(), []
    stack = [root]
    while stack:
        value = stack.pop()
        if id(value) in seen:
            continue
        seen.add(id(value))
        if isinstance(value, (str, bytes)):
            marker = b"diagnostic-payload-" if isinstance(value, bytes) else "diagnostic-payload-"
            if marker in value:
                found.append(value)
        elif isinstance(value, dict):
            stack.extend(value.values())
        elif isinstance(value, (list, tuple)):
            stack.extend(value)
        elif value.__class__.__module__.startswith("embodied_navigation."):
            stack.extend(vars(value).values())
    return found
