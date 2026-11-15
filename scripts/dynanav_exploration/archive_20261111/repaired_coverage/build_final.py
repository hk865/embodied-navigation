#!/usr/bin/env python3
"""Build the final read-only repaired-generation coverage evidence package."""
from pathlib import Path
import csv
import hashlib
import importlib.util
import json
import shutil
import subprocess
import sys

import numpy as np

ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('render',ROOT/'render_coverage.py')
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
OUT=ROOT/'final';OUT.mkdir(exist_ok=True)
PRESERVED=ROOT/'final_input_preservation'
EVIDENCE=ROOT.parent/'repaired_evidence'


def read(path):return json.loads(path.read_text())
def write(path,value):path.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n')
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    cloud_path=PRESERVED/'measured_partial_cloud.npz';cloud=np.load(cloud_path)
    summary=read(PRESERVED/'summary.json');obs=read(EVIDENCE/'rfinal/observation.json')
    old=read(EVIDENCE/'r27/observation.json');oldmeta=read(EVIDENCE/'r27/scene_view_metadata.json')
    rgb=read(EVIDENCE/'rfinal/rgb_source.json');latest_map=read(EVIDENCE/'rfinal/map.json')
    assert str(cloud['run_id'])==r.RUN_ID and str(cloud['cloud_stage'])=='LIO'
    assert summary['final'] is True and len(cloud['xyz'])==summary['voxel_count']
    assert np.isfinite(cloud['xyz']).all() and np.isfinite(cloud['lio_pose_xyz']).all()
    assert np.all(np.diff(cloud['source_stamp_ns'])>0)
    nav_path,trace=r.navigation_trace(float('inf'))
    with (OUT/'actual_postvio_trajectory.csv').open('w') as handle:
        writer=csv.writer(handle);writer.writerow(['source_time_s','x_m','y_m','z_m','yaw_rad']);writer.writerows(trace)
    receipts=r.receipt_audit(obs);write(OUT/'waypoint_receipt_audit.json',receipts)
    assert (receipts['goto_accepted_count'],receipts['goto_reached_count'],receipts['goto_failed_count'],receipts['goto_canceled_count'])==(26,24,1,1)
    assert next(a for a in receipts['actions'] if a['seq']==27)['failure_reason']=='route_low_progress_stall'
    assert obs['pose']['fresh'] is False and obs['pose']['position_xyz_m'] is None
    shutil.copy2(EVIDENCE/'rfinal/rgb.png',OUT/'final_rgb.png')
    config=read(ROOT/'r24_physical_config.json')
    lower,upper=config['regions']
    lower['evidence']=lower['evidence'].replace('as of r24','at failure termination')
    upper.update(floor_z_band_m=[3.1,4.9],point_z_band_m=[3.1,6.8],trace_z_band_m=[3.3,5.3])
    upper['evidence']='Current-generation RGB r16-r19 confirms two stair flights, landing, open red door and white-tile corridor. RGB r20-r26b confirms the upper hall, north blue-wall seating, reception desk and east seating side. Full actual postVIO traces the south edge, reception west-side aisle to the north seating view, north cross-hall segment, and east side down to the failed seq27 approach. r27 and final RGB are dark/occluded, limiting the final forward view. These absolute mapped z bands are only display selections; spatial z variation is not a claimed true physical floor grade. The run terminated with route_low_progress_stall before room-complete exploration or lower north-hall traversal.'
    upper['annotations']=[
        {'text':'Lateral / north-side openings\nremote observed, untraversed','xy_m':[-4,16],'text_xy_m':[-10,27]},
        {'text':'East seating side traversed\nwall-side / furniture backs incomplete','xy_m':[18,8],'text_xy_m':[13,27],'ha':'center'},
        {'text':'Reception enclosure / concave side\nnot entered; opaque backs remain','xy_m':[10,7],'text_xy_m':[14,21],'ha':'center'},
        {'text':'Seq27 failed approach\ndark final forward RGB','xy_m':[14.06,2.29],'text_xy_m':[9,-9],'ha':'center'},
    ]
    write(OUT/'physical_region_config.json',config)
    subprocess.run([sys.executable,str(ROOT/'render_physical_slices.py'),'--snapshot','rfinal',
                    '--config',str(OUT/'physical_region_config.json'),'--out',str(OUT/'physical_regions'),
                    '--cloud-file',str(cloud_path),'--trajectory-file',str(OUT/'actual_postvio_trajectory.csv')],check=True)
    for region in ['lower','upper']:
        shutil.copy2(OUT/'physical_regions'/f'{region}_observed_region_physical_slice_2d.png',OUT/f'{region}_region_2d.png')
    reference={'xyz_m':old['pose']['position_xyz_m'],'yaw_rad':old['pose']['yaw_rad'],'source_stamp_s':oldmeta['pose_source_stamp_s'],
               'snapshot':'r27','fresh_at_r27_only':old['pose']['fresh']}
    note=(f'Final cloud through {summary["last_source_stamp_s"]:.1f}s; actual trace ends {trace[-1,0]:.1f}s '
          f'(delta {summary["last_source_stamp_s"]-trace[-1,0]:.1f}s).\n'
          f'Black x + 1m yaw arrow = old r27 pose at {reference["source_stamp_s"]:.1f}s '
          f'(delta {summary["last_source_stamp_s"]-reference["source_stamp_s"]:.1f}s). Final observation fresh=false; no final XYZ is fabricated.')
    view=r.plot_multiview(cloud,trace,'rfinal',OUT/'measured_structure_multiview.png',reference,note)
    remote=r.plot_remote_overview(cloud,trace,'rfinal',OUT/'distant_measured_return_overview.png')
    r.plot_trajectory_profile(trace,'rfinal',OUT/'actual_postvio_elevation_profile.png')
    lower_grid=read(EVIDENCE/'rfinal/level_+0.04_operator_map_partial.json')
    snapshot_times={'final_observation_sim_s':obs['sim_s'],'final_observation_fresh':False,'final_observation_xyz_m':None,
        'final_RGB_source_s':rgb['source_time_s'],'cloud_first_s':summary['first_source_stamp_s'],
        'cloud_last_s':summary['last_source_stamp_s'],'actual_trace_last_source_s':float(trace[-1,0]),
        'old_r27_pose_source_s':reference['source_stamp_s'],
        'cloud_minus_actual_trace_last_s':float(summary['last_source_stamp_s']-trace[-1,0]),
        'cloud_minus_old_r27_pose_s':float(summary['last_source_stamp_s']-reference['source_stamp_s']),
        'final_grid_last_exact_pair_s':latest_map['last_frame']['stamps']['cloud'],
        'same_frame_synchrony_claimed':False}
    gaps=[
        {'id':'L1','layer':'lower','kind':'observed_but_untraversed','description_zh':'主厅+y长轴及北端只有远距回波，本代未实际巡行。'},
        {'id':'U1','layer':'upper','kind':'observed_but_untraversed','description_zh':'北侧与西侧侧向开口/延伸空间有部分远距回波，未进入；观察到的轮廓不是建筑完整边界。'},
        {'id':'U2','layer':'upper','kind':'untraversed_or_incomplete_view','description_zh':'主厅已沿南边、接待台左侧至北端、北横向及东侧部分走动；靠墙周边、座椅/绿植背面仍缺少近距反向视角。'},
        {'id':'O1','layer':'upper','kind':'occluded','description_zh':'接待台围合/凹侧内部、实体台面与家具背面无法由外侧回波证明已完整观察。'},
        {'id':'F1','layer':'upper','kind':'failed_unfinished_approach','description_zh':'seq27向[14,1.5,4.61]接近但在约[14.06,2.29]低进展停滞失败；父线程current_report离线交叉审计支持近黑色带支架座椅/家具表面遮挡，并报告1286.855..1302.195s底座接触、四足接触及约0.1rad姿态，无自由落体。末段具体collider pair未留存，精确物体尚未认证。','causal_audit_source':'Parent current_report offline cross-evidence; this coverage auditor did not read truth or native policy state.'},
        {'id':'R1','layer':'unassigned','kind':'remote_high_z_returns','description_zh':'y>40m高z远距回波保留在原云与远距overview，物理区域和楼层未建立；不作新增房间或楼层覆盖。'},
    ]
    record={'schema':'repaired_final_coverage/v1','run_id':r.RUN_ID,'outcome':'exploration_failed_incomplete',
        'failure':'route_low_progress_stall','failure_seq':27,'process_exit_0_is_exploration_success':False,
        'scope':'Only this generation RGB, measured same-stage exact paired LIO cloud, exported SLAM grid, postVIO navigation fields and waypoint receipts.',
        'ground_truth_used':False,'controls_sent_by_auditor':False,'original_scene_or_old_map_used':False,
        'source_times':snapshot_times,'old_reference_pose':reference,
        'cloud':{'file':str(cloud_path),'sha256':digest(cloud_path),'paired_scans':summary['accepted_pairs'],
            'voxel_count':len(cloud['xyz']),'voxel_m':float(cloud['voxel_m']),
            'stage':'LIO','frame':'dynanav_lio_map','pairing':'same-stage exact integer stamp; already map-frame points; no second transform',
            'full_raw_points_preserved':True},
        'actual_trajectory':{'file':str(OUT/'actual_postvio_trajectory.csv'),'source_file':str(nav_path),
            'source_file_sha256':digest(nav_path),'pose_source':'livo_post_vio','map_frame':'dynanav_lio_map','child_frame':'go2_imu',
            'samples':len(trace),'first_source_s':float(trace[0,0]),'last_source_s':float(trace[-1,0]),
            'estimated_path_length_m':float(np.linalg.norm(np.diff(trace[:,1:4],axis=0),axis=1).sum()),
            'maximum_source_gap_s':float(np.diff(trace[:,0]).max()),'distinct_measurements_only':True,'waypoint_lines_substituted':False,
            'xyz_min_m':trace[:,1:4].min(0).tolist(),'xyz_max_m':trace[:,1:4].max(0).tolist()},
        'waypoints':receipts,'physical_layers':config,'physical_slice_record':str(OUT/'physical_regions/physical_slice_record.json'),
        'lower_height_chart_label_stats':{'level_index':lower_grid['level_index'],'resolution_m':lower_grid['resolution_m'],
            'counts':lower_grid['counts'],'label_area_m2':{k:lower_grid['counts'][k]*lower_grid['resolution_m']**2 for k in ['free','occupied','total']},
            'meaning':'Only projected label area; not support, traversability, room area or completion percentage.'},
        'render':view,'remote_return_overview':remote,'gaps':gaps,
        'limitations':['No original building boundary or completeness denominator is known; no whole-building coverage percentage.',
            'Trajectory is the complete available aligned postVIO navigation record (5.9-1297.0s); missing endpoints are not filled or replaced with LIO/truth.',
            'Gray/blank z-slice pixels are not classified free/unknown; only SLAM export carries explicit floor/occupied labels.',
            'Free/floor labels do not certify continuous physical support or footprint clearance.',
            'Stair returns above a height chart may appear occupied without being impassable.',
            'Cloud, RGB, estimated trajectory and old reference pose have different source times.',
            'The final observation has no fresh XYZ; the old r27 pose is explicitly stale at termination.',
            'Four extreme tail points remain in raw NPZ and do not establish motion, another floor or large area.',
            'Upper measured z varies spatially; display bands do not establish true physical floor grade.']}
    write(OUT/'coverage_record.json',record)
    write(OUT/'source_timing_and_stale_pose.json',snapshot_times)
    write(OUT/'remaining_gaps.json',{'run_id':r.RUN_ID,'outcome':'exploration_failed_incomplete','gaps':gaps})
    write(OUT/'deliverables.json',{'run_id':r.RUN_ID,'files':[str(p) for p in OUT.rglob('*') if p.is_file()]})
    print(json.dumps({'out':str(OUT),'trajectory':record['actual_trajectory'],'waypoint_counts':{k:receipts[k] for k in ['goto_accepted_count','goto_reached_count','goto_failed_count','goto_canceled_count']}},indent=2))


if __name__=='__main__':main()
