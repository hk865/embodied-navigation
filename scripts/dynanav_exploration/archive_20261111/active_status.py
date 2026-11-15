"""Read-only native-client feedback, including displayed vertical error."""
import json,os,pathlib,subprocess,time
r=pathlib.Path(os.environ.get('DYNANAV_LIVE_RUN','/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/native_exploration_20261111T091222Z'))
s=subprocess.check_output(['python3','scripts/explore_waypoint_client.py','observe','--run-dir',str(r/'capture'),'--receipts','4'],text=True)
o=json.loads(s);raw=json.loads((r/'capture/interactive/observation.json').read_text());p=o.get('pose',{}).get('position_xyz_m');g=o.get('waypoints',{}).get('active_goal_xyz_m')
d={'sim_s':o.get('sim_s'),'pose':o.get('pose'),'command':o.get('last_command'),'speed':o.get('measured_speed_mps'),'waypoints':o.get('waypoints'),'vertical_error_m':abs(p[2]-g[2]) if p and g else None,'failure':raw.get('failure'),'receipts':o.get('receipts'),'observation_file_wall_age_s':time.time()-(r/'capture/interactive/observation.json').stat().st_mtime}
out=pathlib.Path(os.environ.get('DYNANAV_EVIDENCE_ROOT','/home/hyh001/Documents/Codex/2026-11-11/task/active_evidence'));out.mkdir(exist_ok=True)
with (out/'status_log.jsonl').open('a') as f:f.write(json.dumps(d)+'\n')
print(json.dumps(d))
