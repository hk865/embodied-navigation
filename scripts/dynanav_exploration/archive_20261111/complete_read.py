import os,pathlib,runpy,sys
run=pathlib.Path('/tmp/embodied-navigation-native-exploration-current').read_text().strip()
os.environ['DYNANAV_LIVE_RUN']=run
os.environ['DYNANAV_EVIDENCE_ROOT']='/home/hyh001/Documents/Codex/2026-11-11/task/complete_evidence'
os.environ['DYNANAV_CLOUD_DIR']='/home/hyh001/Documents/Codex/2026-11-11/task/coverage_audit/complete_live_cloud'
script='long_observe.py'
if '--status' in sys.argv:
 sys.argv.remove('--status');script='active_status.py'
runpy.run_path(str(pathlib.Path(__file__).with_name(script)),run_name='__main__')
