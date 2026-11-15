import io, json, os, pathlib, shutil, subprocess, sys, time
from PIL import Image
os.environ['MPLCONFIGDIR']='/tmp/dynanav-mpl'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
RUN=pathlib.Path(os.environ.get('DYNANAV_LIVE_RUN','/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/native_exploration_20261111T080816Z'))
OUT=pathlib.Path(os.environ.get('DYNANAV_EVIDENCE_ROOT','/home/hyh001/Documents/Codex/2026-11-11/task/live_evidence'))
OUT.mkdir(exist_ok=True)
tag=sys.argv[1]
d=OUT/tag;d.mkdir(exist_ok=True)
s=subprocess.check_output(['python3','scripts/explore_waypoint_client.py','observe','--run-dir',str(RUN/'capture'),'--receipts','8'],text=True)
o=json.loads(s);(d/'observation.json').write_text(s)
shutil.copy2(RUN/'capture/interactive/observation.json',d/'raw_native_observation.json')
m=json.loads((RUN/'capture/operator_map/latest.json').read_text());(d/'map.json').write_text(json.dumps(m,indent=2))
if o.get('sim_frames',{}).get('latest'):
    for attempt in range(10):
        data=pathlib.Path(o['sim_frames']['latest']).read_bytes()
        try:
            Image.open(io.BytesIO(data)).verify()
            (d/'rgb.png').write_bytes(data)
            for line in reversed(pathlib.Path(o['sim_frames']['index']).read_text().splitlines()):
                try: frame_row=json.loads(line)
                except ValueError: continue
                if frame_row.get('path')==o['sim_frames']['latest']:
                    (d/'rgb_source.json').write_text(json.dumps(frame_row,indent=2))
                    break
            break
        except (OSError,ValueError,SyntaxError):
            if attempt==9: raise
            time.sleep(.05)
fig,ax=plt.subplots(figsize=(9,9))
for key,l in m.get('levels',{}).items():
    p=pathlib.Path(l['grid_export']['npz']);shutil.copy2(p,d/p.name);shutil.copy2(p.with_suffix('.json'),d/p.with_suffix('.json').name)
    a=np.load(p);res=l['grid_export']['resolution_m'];x=(a['ix']+.5)*res;y=(a['iy']+.5)*res
    ax.scatter(x,y,c=np.where(a['code']==0,'#c9dfe8','#141923'),s=2)
    bounds=[float(x.min()),float(x.max()),float(y.min()),float(y.max())] if x.size and y.size else None
    print('level',key,'counts',l['grid_export']['counts'],'bounds',bounds,'empty_export',not bool(x.size))
p=o.get('pose',{}).get('position_xyz_m');yaw=o.get('pose',{}).get('yaw_rad')
if p is not None:
    ax.plot(p[0],p[1],'ro');ax.arrow(p[0],p[1],np.cos(yaw),np.sin(yaw),width=.04,color='r')
ax.set_aspect('equal');ax.grid();ax.set_xticks(np.arange(-20,31,1));ax.set_yticks(np.arange(-20,31,1));ax.set_xlim(-12,15);ax.set_ylim(-12,18);ax.set_title(f'Height-chart overlay (not physical floor count) / {tag} / sim {o.get("sim_s")}')
fig.savefig(d/'map.png',dpi=130)
print(json.dumps(o,ensure_ascii=False))
print('EVIDENCE',str(d))
