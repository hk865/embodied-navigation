"""Compile the actual local initialization block without viewing or duplicating it.

Run on a staged SCAN tree. This is an offline numerical harness, not a planner
substitute, and its synthetic fixtures are not exact runtime failure replays.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--scan-source', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--baseline-source', type=Path)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=False)
manager = a.scan_source / 'plan_manage/src/planner_manager.cpp'
text = manager.read_text()
replan = text.index('bool SCANPlannerManager::reboundReplan(')
start = text.index('            PolynomialTraj gl_traj;', replan)
end = text.index('        } else // Initial path generated from previous trajectory.', start)
(a.output / 'local_initialization.inc').write_text(text[start:end])
gstart = text.index('    if ((start_pt - local_target_pt).norm() < 0.05)', replan)
gend = text.index('    auto t_start', gstart)
(a.output / 'local_guard.inc').write_text(text[gstart:gend])
report = {'source_sha256': hashlib.sha256(manager.read_bytes()).hexdigest(),
          'scope': 'actual production local initialization and close-goal guard; synthetic numerical fixtures, not exact runtime failure replay'}
if a.baseline_source:
    base = (a.baseline_source / 'plan_manage/src/planner_manager.cpp').read_text()
    def dynamic_function(s):
        start = s.index('bool SCANPlannerManager::checkDynamicFeasibility(')
        return s[start:s.index('void SCANPlannerManager::reparamBspline(', start)]
    def without_initializer(s):
        r = s.index('bool SCANPlannerManager::reboundReplan(')
        begin = s.index('            PolynomialTraj gl_traj;', r)
        finish = s.index('        } else // Initial path generated from previous trajectory.', begin)
        return s[:begin] + '<LOCAL_INITIALIZER>' + s[finish:]
    report['all_code_outside_local_initializer_unchanged'] = without_initializer(text) == without_initializer(base)
    assert report['all_code_outside_local_initializer_unchanged'], 'Changes outside local initializer'
    report['dynamic_feasibility_function_unchanged'] = dynamic_function(text) == dynamic_function(base)
    assert report['dynamic_feasibility_function_unchanged'], 'Dynamic feasibility gate changed'
command = ['g++', '-std=c++17', '-O2', '-I/usr/include/eigen3', '-I'+str(a.scan_source/'traj_utils/include'),
           '-I'+str(a.output), str(Path(__file__).with_name('acceptance.cpp')),
           str(a.scan_source/'traj_utils/src/polynomial_traj.cpp'), '-o', str(a.output/'acceptance')]
build = subprocess.run(command, capture_output=True, text=True)
(a.output/'build.log').write_text(build.stdout+build.stderr)
report['build_returncode'] = build.returncode
if build.returncode == 0:
    run = subprocess.run([str(a.output/'acceptance')], capture_output=True, text=True, timeout=30)
    (a.output/'result.log').write_text(run.stdout+run.stderr)
    report['test_returncode'] = run.returncode
(a.output/'report.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report))
