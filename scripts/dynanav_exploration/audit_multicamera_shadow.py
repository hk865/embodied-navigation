#!/usr/bin/env python3
"""Read-only audit and CI diagnostics for frozen native multi-camera shadow replays."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

REPO=Path(__file__).resolve().parents[2]
if (REPO/'src').is_dir():sys.path.insert(0,str(REPO/'src'))
from embodied_navigation.localization_mapping.frontend.multicamera_fusion import ShadowPacket,fuse_shadow_packets


def read_lines(path):return [json.loads(line) for line in path.read_text().splitlines() if line]
def file_info(path):return dict(path=str(path),bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def audit(capture,snapshots,replays,output):
    capture,snapshots=capture.resolve(),snapshots.resolve()
    if output.exists():raise ValueError('Use a new audit output directory')
    archive=capture/'multicamera'
    calibration=json.loads((archive/'calibration.json').read_text())
    if calibration['generation']!=str(capture):raise ValueError('Calibration generation mismatch')
    calibrations={row['camera_id']:row for row in calibration['cameras']}
    records=read_lines(archive/'index.jsonl')
    images={}
    for row in records:
        if row['generation']!=str(capture) or row['time_evidence']['body_now_substituted']:
            raise ValueError('Wrong generation or rewritten camera source time')
        images.setdefault(row['stamp_ns'],{})[row['camera_id']]=row
    summary=dict(generation=str(capture),covariance_source='actual native full 19D LIO state, physical 18D marginal',
        exposure_prior='independent per-image N(1,0.1) model for exposure mode; not calibrated hardware uncertainty',
        estimator_feedback_enabled=False,confidence_calibrated=False,modes={})
    integrity={};fused=[]
    for replay in replays:
        rows=read_lines(replay);mode=replay.stem
        cameras={key:[] for key in calibrations}
        for frame in rows:
            stamp=frame['source_stamp_ns'];source=Path(frame['native_snapshot']).resolve()
            if frame['generation']!=str(capture) or not source.is_relative_to(snapshots):
                raise ValueError('Replay generation/snapshot path mismatch')
            if not frame['native_snapshot_unchanged'] or abs(frame['native_target_stamp_ns']-stamp)>1:
                raise ValueError('Shared state/geometry mutated or native float epoch belongs to another image')
            source_info=integrity.setdefault(str(source),file_info(source))
            selected=images[stamp]
            if set(selected)!=set(calibrations):raise ValueError('Incomplete same-source selected-camera image set')
            packets=[]
            for camera in frame['cameras']:
                camera_id=camera['camera_id'];cameras[camera_id].append(camera)
                path=(archive/selected[camera_id]['rgb_path']).resolve()
                if not path.is_relative_to(archive):raise ValueError('Camera path escapes archive')
                integrity.setdefault(str(path),file_info(path))
                if not camera['retained_update'] or camera.get('error'):continue
                delta=np.asarray(camera['delta_rotation_body_position_world'])
                packets.append(ShadowPacket(camera_id,str(capture),source_info['sha256'],source_info['sha256'],
                    stamp,stamp,stamp,hashlib.sha256(json.dumps(calibrations[camera_id],sort_keys=True).encode()).hexdigest(),
                    delta,np.asarray(camera['posterior_pose_covariance']),'native_internal',True))
            item=dict(mode=mode,source_stamp_ns=stamp,native_snapshot_sha256=source_info['sha256'],
                valid_camera_count=len(packets),camera_source_skew_ns=0,
                native_float_source_delta_ns=frame['native_target_stamp_ns']-stamp)
            if packets:
                result=fuse_shadow_packets(packets)
                item.update({key:value.tolist() if isinstance(value,np.ndarray) else value for key,value in result.items()})
            fused.append(item)
        result=dict(frames=len(rows),source_interval_ns=[rows[0]['source_stamp_ns'],rows[-1]['source_stamp_ns']],
            max_camera_source_skew_ns=0,
            max_native_float_conversion_delta_ns=max(abs(r['native_target_stamp_ns']-r['source_stamp_ns']) for r in rows),
            shared_snapshot_unchanged=all(r['native_snapshot_unchanged'] for r in rows),cameras={})
        for camera,values in cameras.items():
            valid=[row for row in values if row['retained_update'] and not row.get('error')]
            covariances=[np.asarray(row['posterior_physical_covariance']) for row in valid]
            pose_matrices=[np.asarray(row['posterior_pose_covariance']) for row in valid]
            # Checking SPD establishes a numeric property, not statistical calibration.
            for covariance in covariances:np.linalg.cholesky(.5*(covariance+covariance.T))
            result['cameras'][camera]=dict(retained_visual_frames=len(valid),
                tracked_point_frame_instances=sum(row.get('tracked_points',0) for row in values),
                tracked_points_range=[min(row.get('tracked_points',0) for row in values),max(row.get('tracked_points',0) for row in values)],
                native_errors=[row['error'] for row in values if row.get('error')],
                positive_definite_physical_marginals=len(covariances),
                inverse_exposure_range=[min(row.get('inverse_exposure',1) for row in values),max(row.get('inverse_exposure',1) for row in values)],
                model_rotation_std_rad_range=[float(min(np.sqrt(np.diag(p)[:3]).min() for p in pose_matrices)),float(max(np.sqrt(np.diag(p)[:3]).max() for p in pose_matrices))] if valid else None,
                model_position_std_m_range=[float(min(np.sqrt(np.diag(p)[3:]).min() for p in pose_matrices)),float(max(np.sqrt(np.diag(p)[3:]).max() for p in pose_matrices))] if valid else None)
            if camera=='camera_0':
                for row in values:
                    if not any(key in row for key in ('single_vs_multi_pose_delta_max_difference','single_vs_six_pose_delta_max_difference')):
                        raise ValueError('Missing independent front-camera control result')
                result['single_front_vs_multi_max_pose_delta_difference']=max(row.get('single_vs_multi_pose_delta_max_difference',row.get('single_vs_six_pose_delta_max_difference',0)) for row in values)
                result['single_front_vs_multi_max_physical_covariance_difference']=max(row.get('single_vs_multi_covariance_max_difference',row.get('single_vs_six_covariance_max_difference',0)) for row in values)
        summary['modes'][mode]=result
    output.mkdir(parents=True)
    (output/'audit.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
    (output/'source_integrity.json').write_text(json.dumps(list(integrity.values()),indent=2))
    (output/'ci_diagnostics.jsonl').write_text(''.join(json.dumps(row,allow_nan=False)+'\n' for row in fused))
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--snapshots',type=Path,required=True)
    parser.add_argument('--replay',type=Path,action='append',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=audit(args.capture,args.snapshots,args.replay,args.output)
    print(json.dumps(dict(output=str(args.output),modes=list(result['modes'])),ensure_ascii=False))
    return 0


if __name__=='__main__':raise SystemExit(main())
