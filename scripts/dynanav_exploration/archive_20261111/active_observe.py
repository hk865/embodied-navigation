import os, pathlib, runpy
os.environ['DYNANAV_LIVE_RUN']='/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/native_exploration_20261111T091222Z'
os.environ['DYNANAV_EVIDENCE_ROOT']='/home/hyh001/Documents/Codex/2026-11-11/task/active_evidence'
os.environ['DYNANAV_CLOUD_DIR']='/home/hyh001/Documents/Codex/2026-11-11/task/coverage_audit/active_live_cloud'
runpy.run_path(str(pathlib.Path(__file__).with_name('long_observe.py')),run_name='__main__')
