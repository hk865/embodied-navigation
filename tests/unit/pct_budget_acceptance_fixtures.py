"""Transport fixtures: real worker/planner; fake ROS/process ownership only."""
from contextlib import ExitStack
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
from embodied_navigation.simulation.dynanav import pct_scan_worker as worker
from embodied_navigation.planning.pct.measured_tomogram import MeasuredPctPlanner
from scan_worker_acceptance_fixtures import (
    WorkerRig, Process, ImmediatePool, ros_modules, Odometry, header, point)

REPO = Path(worker.__file__).resolve().parents[4]


def sparse_multilayer_points(width=938, height=808):
    # 600 measured patch points across 4 disjoint heights and one remote return.
    # The remote return expands observed bounds but cannot invent support.
    points = [[(x+.5)*.15, (y+.5)*.15, float(z)]
              for z in range(4) for y in range(5) for x in range(30)]
    return np.asarray(points + [[(width-.5)*.15, (height-.5)*.15, 0.]])


def build_native(directory):
    for command in (["cmake", "-S", str(REPO / 'third_party/pct'), "-B", str(directory),
                     "-DCMAKE_BUILD_TYPE=Release"],
                    ["cmake", "--build", str(directory), "--parallel", "2"]):
        subprocess.run(command, check=True, capture_output=True, text=True, timeout=120)
    return Path(directory) / 'pct_astar'


class BudgetWorkerRig(WorkerRig):
    def observe(self, source=10.5, position=(0., 0., .4)):
        message = Odometry()
        message.header = header(source)
        message.pose.pose.position = point(*position)
        self.last_odom = message
        self.node.receive('post_vio_odometry', message)
        self.node.receive('aft_mapped_to_init', copy.deepcopy(message))
        fields = ('x', 'y', 'z', 'curvature')
        cloud = NS(header=header(source), fields=[NS(name=n) for n in fields])
        cloud.points = np.asarray([((x+.5)*.15, (y+.5)*.15, 0., float(x))
            for y in range(3) for x in range(30)], dtype=[(n, 'f4') for n in fields])
        self.node.receive('cloud_registered_full', cloud)
        return message

    def execute(self, binary, extra_cli=(), *, legacy=False):
        self.constructed, self.kwargs, self.frozen_at_construction = [], [], []

        def planner(*args, **kwargs):
            # Spy calls the unmodified constructor and the resulting real plan().
            self.frozen_at_construction.append(json.loads(
                (self.mailbox / 'planner_worker_parameters.json').read_text()))
            self.kwargs.append(kwargs)
            result = MeasuredPctPlanner(*args, **kwargs)
            self.constructed.append(result)
            return result

        def popen(command, **kwargs):
            process = Process(command)
            self.processes.append(process)
            return process

        def drive(node):
            self.node = node
            self.prepare()

        argv = ['--run-dir', str(self.run), '--scan-prefix', str(self.prefix),
                '--speed', '.5', '--body-support-offset', '.4',
                '--pct-binary', str(binary), *extra_cli]
        with ExitStack() as stack:
            stack.enter_context(patch.dict('sys.modules', ros_modules(drive)))
            stack.enter_context(patch.object(worker, 'scan_source_digest', return_value='acceptance'))
            stack.enter_context(patch.object(worker, 'ThreadPoolExecutor', ImmediatePool))
            stack.enter_context(patch.object(worker, 'subprocess',
                NS(**{**vars(worker.subprocess), 'Popen': popen})))
            stack.enter_context(patch.object(worker.time, 'monotonic', side_effect=lambda: self.wall))
            stack.enter_context(patch.object(worker.signal, 'signal'))
            stack.enter_context(patch(
                'embodied_navigation.planning.pct.measured_tomogram.MeasuredPctPlanner', planner))
            if legacy:
                args = NS(run_dir=str(self.run), mailbox='interactive', scan_prefix=str(self.prefix),
                    speed=.5, body_support_offset=.4, pct_start_blind_radius=0.,
                    pct_binary=binary, reference_response_timeout_s=10.)
                worker.run(args)
            else:
                worker.main(argv)


def launch_without_external_processes(directory, extra_cli):
    """Execute real shell parsing/dispatch with process/ROS environment stubs.

    Only two shared /tmp operational filenames are relocated in this copy.
    Launcher branches, arguments and dispatch are otherwise byte-for-byte.
    No ROS, Isaac, real planner or estimator process starts.
    """
    root = Path(directory)
    (root / 'scripts').mkdir(parents=True)
    (root / 'build').mkdir()
    (root / 'scan').mkdir()
    (root / 'build/capture_message_env.sh').write_text('')
    (root / 'scan/setup.bash').write_text('')
    config = root / 'configs/simulation/dynanav'
    config.mkdir(parents=True)
    for name in ('fastlivo_livo.yaml', 'camera_rgbd.yaml', 'fastlivo_rtx_os1_32.yaml'):
        shutil.copyfile(REPO / 'configs/simulation/dynanav' / name, config / name)
    launcher = (REPO / 'scripts/run_dynanav_native_exploration.sh').read_text()
    for old in ('/tmp/embodied-navigation-isaac-exclusive.lock',
                '/tmp/embodied-navigation-native-exploration-current'):
        launcher = launcher.replace(old, str(root / Path(old).name))
    script = root / 'scripts/run_dynanav_native_exploration.sh'
    script.write_text(launcher)
    (root / '.env.local').write_text('\n'.join([
        'PROJECT_DATA_ROOT=' + str(root), 'BUILD_PREFIX=' + str(root / 'build'),
        'SCAN_PREFIX=' + str(root / 'scan'), 'ROS_DOMAIN_ID=230',
        'NAVIGATION_BACKEND=pct_scan']) + '\n')
    (root / 'scripts/run_dynanav_lio.sh').write_text('''#!/usr/bin/env bash
while (($#)); do
    if [[ "$1" == --output-dir ]]; then out=$2; break; fi
    shift
done
mkdir -p "$out"
printf '{}\\n' > "$out/ready.json"
sleep 1
''')
    harness = root / 'python_transport.py'
    harness.write_text('''import json, pathlib, subprocess, sys, time
root = pathlib.Path(__file__).parent
args = sys.argv[1:]
if args and args[0] == 'scripts/run_dynanav_pct_scan.py':
    from unittest.mock import patch
    from embodied_navigation.simulation.dynanav import pct_scan_worker as worker
    with patch.object(worker, 'build_scan'), patch.object(worker, 'run'):
        worker.main(args[1:])
if args and args[0] == '-':
    raise SystemExit(subprocess.run([sys.executable, *args], stdin=sys.stdin).returncode)
with (root/'argv.jsonl').open('a') as stream:
    stream.write(json.dumps(args)+'\\n')
if args and args[0] == 'scripts/run_with_bounded_log.py':
    run = pathlib.Path(args[args.index('--output')+1])
    (run/'interactive').mkdir(parents=True)
    time.sleep(.5)
''')
    prelude = root / 'prelude.bash'
    prelude.write_text('''source() {
    if [[ "$1" == /opt/ros/jazzy/setup.bash ]]; then return 0; fi
    builtin source "$@"
}
pgrep() { return 1; }
flock() { return 0; }
python3() { command /usr/bin/python3 "$ACCEPTANCE_TRANSPORT" "$@"; }
function /usr/bin/python3 { command /usr/bin/python3 "$ACCEPTANCE_TRANSPORT" "$@"; }
''')
    env = dict(os.environ, BASH_ENV=str(prelude), ACCEPTANCE_TRANSPORT=str(harness),
               PYTHONPATH=str(REPO / 'src'))
    for key in ('MULTICAMERA_OBSERVATION', 'MULTICAMERA_HORIZONTAL_FOV', 'PRIOR_MAP'):
        env.pop(key, None)
    done = subprocess.run(['bash', str(script), str(root / 'run'), '--until-stopped', *extra_cli],
        cwd=root, env=env, capture_output=True, text=True, timeout=10)
    commands = [json.loads(line) for line in (root / 'argv.jsonl').read_text().splitlines()] \
        if (root / 'argv.jsonl').exists() else []
    return done, commands

