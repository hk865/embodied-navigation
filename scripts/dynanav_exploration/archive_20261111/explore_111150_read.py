import os,pathlib,runpy,sys
os.environ['DYNANAV_LIVE_RUN']='/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/native_exploration_20261111T111150Z'
os.environ['DYNANAV_EVIDENCE_ROOT']='/home/hyh001/Documents/Codex/2026-11-11/task/explore_111150_evidence'
os.environ['DYNANAV_CLOUD_DIR']='/home/hyh001/Documents/Codex/2026-11-11/task/coverage_audit/live_111150'
script='long_observe.py'
if '--status' in sys.argv:
 sys.argv.remove('--status');script='active_status.py'
runpy.run_path(str(pathlib.Path(__file__).with_name(script)),run_name='__main__')
