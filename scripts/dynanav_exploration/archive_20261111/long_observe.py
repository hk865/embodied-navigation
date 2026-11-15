"""Read-only sensor/evidence snapshot and same-generation 3D views."""
import argparse,json,os,pathlib,shutil,subprocess,sys
ROOT=pathlib.Path('/home/hyh001/Documents/Codex/2026-11-11/task')
RUN=os.environ.get('DYNANAV_LIVE_RUN','/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/native_exploration_20261111T083457Z')
EVIDENCE=pathlib.Path(os.environ.get('DYNANAV_EVIDENCE_ROOT',str(ROOT/'long_evidence')))
CLOUD=pathlib.Path(os.environ.get('DYNANAV_CLOUD_DIR',str(ROOT/'coverage_audit/long_live_cloud')))
p=argparse.ArgumentParser();p.add_argument('tag');p.add_argument('--roi-span',type=float,default=20);p.add_argument('--z-span',type=float,default=10);p.add_argument('--azim',type=float,default=-58);p.add_argument('--elev',type=float,default=28);a=p.parse_args()
env=dict(os.environ,DYNANAV_LIVE_RUN=RUN,DYNANAV_EVIDENCE_ROOT=str(EVIDENCE),MPLCONFIGDIR='/tmp/live_views_mpl')
r=subprocess.run([sys.executable,str(ROOT/'snapshot_live.py'),a.tag],env=env,capture_output=True,text=True)
if r.returncode:print(r.stdout);print(r.stderr);raise SystemExit(r.returncode)
d=EVIDENCE/a.tag
o=json.loads((d/'observation.json').read_text());m=json.loads((d/'map.json').read_text());f=m.get('last_frame') or {}
print(json.dumps({'sim_s':o.get('sim_s'),'pose':o.get('pose'),'command':o.get('last_command'),'measured_speed_mps':o.get('measured_speed_mps'),'waypoints':o.get('waypoints'),'receipts':o.get('receipts',[])[-4:],'rgb':str(d/'rgb.png'),'map_last':{k:f.get(k) for k in ['accepted','reasons','level_index','classification_floor_z_m','floor_evidence']},'levels':{k:{'frames':v.get('accumulated_frames'),'counts':v.get('grid_export',{}).get('counts')} for k,v in m.get('levels',{}).items()}},ensure_ascii=False))
if o.get('pose',{}).get('position_xyz_m') is not None:
    shutil.copy2(CLOUD/'measured_partial_cloud.npz',d/'measured_cloud.npz')
    for name in ['latest_paired_cloud.npz','summary.json']:
        if (CLOUD/name).exists():shutil.copy2(CLOUD/name,d/name)
    c=subprocess.run([sys.executable,str(ROOT/'coverage_audit/render_live_3d_views.py'),'--cloud',str(d/'measured_cloud.npz'),'--observation',str(d/'raw_native_observation.json'),'--output-dir',str(d),'--prefix','scene','--expected-run-id',pathlib.Path(RUN).name,'--observation-run-id',pathlib.Path(RUN).name,'--roi-span',str(a.roi_span),'--z-span',str(a.z_span),'--azim',str(a.azim),'--elev',str(a.elev)],env=env,capture_output=True,text=True)
    print(c.stdout)
    if c.returncode:print(c.stderr);raise SystemExit(c.returncode)
