#!/usr/bin/env python3
"""Project entrypoint for sensor exploration evidence and native SOP operations.

  python3 scripts/dynanav_exploration.py COMMAND --help

sop      Current project SOP: status, snapshot, wait, note, verify-exit;
         launch/goto/cancel/stop have the explicitly requested operational effect.
collect  Record exact paired live LIO clouds (ROS environment and explicit domain).
view     Render retained cloud XY/XZ/YZ/3D with a measured pose, without ROS.
review   Audit one frozen generation with post-VIO trajectory and height grids.
roi      Inspect an explicit XY crop in YZ/oblique views, without ROS.

multicamera  View the selected independent RGBs and project the shared measured LIO map.

candidates  Prepare read-only PCT-route utility for explicit DSH candidates.

No command uses the historical task directory as a code dependency. Data paths
remain explicit; the archive is provenance, not the default execution path.
"""
from pathlib import Path
import os
import subprocess
import sys


REPO = Path(__file__).resolve().parents[1]
SKILL = REPO / '.agents/skills/dynanav-explore/scripts'
TOOLS = REPO / 'scripts/dynanav_exploration'
COMMANDS = {
    'sop': SKILL / 'dynanav_sop.py',
    'collect': SKILL / 'record_current_lio_cloud.py',
    'view': SKILL / 'render_live_3d_views.py',
    'review': TOOLS / 'review_generation.py',
    'roi': TOOLS / 'review_roi.py',
    'multicamera': TOOLS / 'multicamera_snapshot.py',
    'observe-multicamera': TOOLS / 'multicamera_observe.py',
    'multicamera-ros': TOOLS / 'multicamera_ros_bridge.py',
    'shadow-audit': TOOLS / 'audit_multicamera_shadow.py',
    'candidates': REPO / 'scripts/prepare_dynanav_waypoint_candidates.py',
}


def command_line(arguments):
    command, *tail = arguments
    if command not in COMMANDS:
        raise ValueError(f'Unknown command {command!r}; choose {", ".join(COMMANDS)}')
    if command == 'sop' and not any(v == '--repo' or v.startswith('--repo=') for v in tail):
        tail = ['--repo', str(REPO), *tail]
    return [sys.executable, '-B', str(COMMANDS[command]), *tail]


def main(arguments=None):
    arguments = list(sys.argv[1:] if arguments is None else arguments)
    if not arguments or arguments[0] in ('-h', '--help'):
        print(__doc__)
        return 0
    try:
        command = command_line(arguments)
    except ValueError as error:
        print(error, file=sys.stderr)
        return 2
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    return subprocess.call(command, env=environment)


if __name__ == '__main__':
    raise SystemExit(main())
