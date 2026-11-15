#!/usr/bin/env python3
"""Freeze native SOP, all independent RGB views and their true source-time relation.

Read-only observation adapter; never selects targets, sends motion or reads truth.
"""
from __future__ import annotations
import argparse,hashlib,json,subprocess,sys
from pathlib import Path
REPO=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(Path(__file__).resolve().parent))
from multicamera_snapshot import build_snapshot

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-dir',type=Path,required=True);p.add_argument('--evidence-dir',type=Path,required=True);p.add_argument('--cloud-dir',type=Path,required=True);p.add_argument('--tag',required=True);p.add_argument('--roi-span',type=float,default=20.);p.add_argument('--z-span',type=float,default=12.)
 a=p.parse_args();run=a.run_dir.resolve();evidence=a.evidence_dir.resolve()
 cmd=[sys.executable,str(REPO/'scripts/dynanav_exploration.py'),'sop','--run-dir',str(run),'--evidence-dir',str(evidence),'snapshot','--tag',a.tag,'--cloud-dir',str(a.cloud_dir),'--roi-span',str(a.roi_span),'--z-span',str(a.z_span)]
 result=subprocess.run(cmd,capture_output=True,text=True,timeout=150)
 if result.returncode:raise RuntimeError(result.stdout+result.stderr)
 destination=evidence/a.tag
 raw=json.loads((destination/'raw_native_observation.json').read_text());report=build_snapshot(run/'capture',destination/'multicamera',images_only=True)
 pose_stamp=(raw.get('pose') or {}).get('stamp_ns',raw.get('source_ns'))
 times=[r['stamp_ns'] for r in report['source_frames']]
 combined=dict(generation=str(run),native_snapshot=str(destination/'snapshot_manifest.json'),multicamera_snapshot=str(destination/'multicamera/snapshot.json'),actual_rgb_grid=str(destination/'multicamera/view_grid.png'),camera_count=report['camera_count'],camera_source_stamps_ns=times,maximum_camera_skew_ns=max(times)-min(times),pose_source_stamp_ns=pose_stamp,pose_minus_camera_ns=None if pose_stamp is None else [pose_stamp-t for t in times],source_times_rewritten=False,pose_fusion_implemented=False,extra_views_feed_formal_localization=False,ground_truth_used=False,notes=['SOP and camera archive are separately read; source offsets are reported, not hidden or retimestamped','Actual independent RGBs must be viewed by the model, along with local/full 3D and state'])
 combined['input_sha256']={str(f.relative_to(destination)):hashlib.sha256(f.read_bytes()).hexdigest() for f in destination.rglob('*') if f.is_file() and f.suffix in ('.json','.png')}
 (destination/'multicamera_observation.json').write_text(json.dumps(combined,indent=2));print(json.dumps(combined))
 return 0
if __name__=='__main__':raise SystemExit(main())
