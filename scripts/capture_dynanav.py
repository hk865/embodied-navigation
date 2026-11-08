#!/usr/bin/env python3
"""Explicit Isaac/DynaNav RGB-D capture entry. Help/check do not start Isaac."""
import argparse
import json
import os
import runpy
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from embodied_navigation.project import data_root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, help='Scene USD; otherwise use configured local office bundle')
    parser.add_argument('--output', type=Path, help='New run directory in the data volume')
    parser.add_argument('--steps', type=int, default=4400, help='0.005-second physics steps, original default 4400')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    data = data_root()
    scene = args.scene or data / 'dynanav/documents_codex_20261108/codex/2026-11-05/task-2/full_asset_render/bundle/office.usd'
    requirements = {'scene': scene, 'robot_usd': data / 'runtime_assets/robot/go2_description/go2_description.usda', 'student_model': ROOT / 'models/student_model_4999_deterministic_jit.pt'}
    present = {key: value.is_file() for key, value in requirements.items()}
    if args.check:
        print(json.dumps(dict(files=present, required_runtime='Isaac Sim 6.0.1 RC; compatibility not tested here', started_simulation=False), indent=2))
        return 0 if all(present.values()) else 2
    if not all(present.values()):
        parser.error('Missing scene, robot asset or Student weight; run --check')
    if args.output is None or args.output.exists():
        parser.error('Provide --output pointing to a new data-volume run directory')
    if args.steps <= 0:
        parser.error('--steps must be positive')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ['PROBE_STEPS'] = str(args.steps)
    sys.argv = ['capture_go2_rgbd', str(args.output), str(scene)]
    runpy.run_module('embodied_navigation.simulation.dynanav.capture_go2_rgbd', run_name='__main__')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
