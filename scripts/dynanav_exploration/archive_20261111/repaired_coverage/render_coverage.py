#!/usr/bin/env python3
"""Read-only evidence rendering for the single repaired generation.

Only measured LIO exports, snapshot RGB metadata, and postVIO navigation
fields are read. This script sends no ROS or robot command.
"""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator
import numpy as np

RUN_ID = 'native_exploration_20261111T102621Z'
ROOT = Path('/home/hyh001/Documents/Codex/2026-11-11/task')
RUN = Path('/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments') / RUN_ID


def review_step(snapshot):
    return 27 if snapshot=='rfinal' else int(snapshot[1:].rstrip('b'))


def read_json(path):
    return json.loads(path.read_text())


def navigation_trace(cutoff):
    """Whitelist navigation fields; no policy state, motion truth or scene data."""
    path = RUN / 'capture/interactive_waypoint_control.jsonl'
    samples = {}
    for line in path.open():
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        t = row.get('odom_stamp_s')
        xyz = row.get('aligned_position_xyz_m')
        if (row.get('pose_source') != 'livo_post_vio'
                or row.get('no_gt_pose_used') is not True
                or row.get('frame_id') != 'dynanav_lio_map'
                or row.get('child_frame_id') != 'go2_imu'
                or not isinstance(t, (float, int)) or t > cutoff
                or xyz is None or not np.isfinite(xyz).all()):
            continue
        # Duplicate control ticks retain the same measured odometry stamp.
        samples[t] = [t, *xyz, row.get('aligned_yaw_rad', np.nan)]
    trace = np.array([samples[t] for t in sorted(samples)], dtype=float)
    if not len(trace):
        raise ValueError('No current-generation postVIO measurements')
    return path, trace


def receipt_audit(obs):
    path=RUN/'capture/interactive/receipts.jsonl'
    all_rows=[]
    for line in path.open():
        try:all_rows.append(json.loads(line))
        except json.JSONDecodeError:continue
    # Receipts have no source time. Bind to the exact final receipt already
    # present in this immutable observation; do not import later outcomes.
    snapshot_rows=obs.get('receipts',[])
    if not snapshot_rows:rows=[]
    else:
        anchor=snapshot_rows[-1]
        matches=[i for i,row in enumerate(all_rows) if row==anchor]
        if len(matches)!=1:raise ValueError('Ambiguous snapshot receipt anchor')
        rows=all_rows[:matches[0]+1]
    actions={};active=None
    for r in rows:
        seq=r['seq'];event=r['event']
        if event=='waypoint_accepted':
            actions[seq]={'seq':seq,'action':'goto','goal_xyz_m':r['goal_xyz_m'],
                          'accepted':True,'reached':False,'state':'accepted_active_or_pending'}
            active=seq
        elif event=='waypoint_reached':
            if seq not in actions:raise ValueError('Reached without acceptance')
            actions[seq].update(reached=True,state='reached')
            if active==seq:active=None
        elif event=='cancel_accepted':
            actions[seq]={'seq':seq,'action':'cancel','accepted':True,'applied_to_active_seq':active,
                          'state':'cancel_acknowledged'}
            if active is not None:
                actions[active].update(state='accepted_then_cancel_acknowledged_not_reached',cancel_seq=seq)
            active=None
        elif event=='stop_accepted':
            actions[seq]={'seq':seq,'action':'stop','accepted':True,'state':'stop_acknowledged'}
        elif event=='waypoint_failed':
            if seq not in actions:raise ValueError('Failed without acceptance')
            actions[seq].update(state='failed',failed=True,failure_reason=r.get('reason'))
            if active==seq:active=None
    return {'source':str(path),'generation_binding':'path under this generation; anchor from immutable snapshot receipts',
            'timestamp_limitation':'receipts lack time; prefix ends at exact final receipt recorded in snapshot',
            'receipt_prefix_count':len(rows),'actions':[actions[k] for k in sorted(actions)],
            'goto_accepted_count':sum(a['action']=='goto' for a in actions.values()),
            'goto_reached_count':sum(a.get('reached',False) for a in actions.values()),
            'goto_failed_count':sum(a.get('failed',False) for a in actions.values()),
            'goto_canceled_count':sum(a.get('cancel_seq') is not None for a in actions.values()),
            'active_seq_after_prefix':active,
            'not_coverage_evidence':'Accepted/reached targets are execution facts, not observed-area or room-completion metrics.'}


def plot_grid(grid, manifest, trace, snapshot, out, label):
    res = manifest['resolution_m']
    ix, iy, code = [grid[k] for k in ['ix', 'iy', 'code']]
    # Extent is a declared local evidence ROI, not a building boundary.
    roi = [-11, 11, -12, 24]
    lo_x, hi_x, lo_y, hi_y = np.round(np.array(roi) / res).astype(int)
    img = np.full((hi_y-lo_y, hi_x-lo_x), -1, dtype=np.int8)
    inside = (ix >= lo_x) & (ix < hi_x) & (iy >= lo_y) & (iy < hi_y)
    img[iy[inside]-lo_y, ix[inside]-lo_x] = code[inside]
    fig, ax = plt.subplots(figsize=(9, 13))
    ax.imshow(img+1, origin='lower', extent=roi, interpolation='none',
              cmap=ListedColormap(['#eeeeee', '#b9dce7', '#242c39']), vmin=0, vmax=2)
    ax.plot(trace[:, 1], trace[:, 2], color='#e25e34', lw=1.6, zorder=4)
    ax.scatter(trace[0, 1], trace[0, 2], c='#247548', s=42, zorder=5)
    ax.scatter(trace[-1, 1], trace[-1, 2], c='#c53222', s=48, zorder=5)
    ax.arrow(trace[-1,1],trace[-1,2], np.cos(trace[-1,4]),np.sin(trace[-1,4]),
             color='#c53222',width=.045,head_width=.3,length_includes_head=True,zorder=5)
    stair_note=('B  First flight + landing traversed\nupper continuation outside lower slice'
                if review_step(snapshot)>=14 else
                'B  South stair: partial rising returns\nlanding / continuation occluded')
    for xy, text, txtxy in [
        ((4,15), 'A  Far +y hall: remote returns\nnear/reverse views incomplete', (10.4,18)),
        ((-4,-7.5), stair_note, (-10.6,-10.9)),
        ((-5,-.2), 'C  North stair: partial returns\nfar side / upper space unconfirmed', (-10.6,5)),
    ]:
        ax.annotate(text, xy=xy, xytext=txtxy, fontsize=9,
                    ha='right' if txtxy[0]>0 else 'left',
                    arrowprops={'arrowstyle':'->','color':'#795532'},
                    bbox={'boxstyle':'round,pad=.3','fc':'#fff8df','ec':'#b79863','alpha':.95})
    ax.set(xlim=roi[:2], ylim=roi[2:], xlabel='estimated map x (m)', ylabel='estimated map y (m)',
           title=f'Lower region evidence / {snapshot}\n{label}; actual postVIO trace through {trace[-1,0]:.1f} s')
    ax.set_aspect('equal'); ax.grid(alpha=.2)
    ax.legend(handles=[
        Line2D([0],[0],color='#eeeeee',lw=8,label='not exported / unknown in this ROI'),
        Line2D([0],[0],color='#b9dce7',lw=8,label='floor-return label'),
        Line2D([0],[0],color='#242c39',lw=8,label='occupied / elevated-return label'),
        Line2D([0],[0],color='#e25e34',lw=2,label='actual postVIO XY trace (all heights)'),
    ],loc='upper left',fontsize=8)
    fig.text(.06,.025,'Orange trace is the full measured XY projection, including stair and landing transitions above this slice.\n'
             'Grid labels are projected measurements; they certify neither physical support nor route clearance.\n'
             'Stair returns above this height band may appear occupied. Unknown is not free.\n'
             'The fixed ROI is only a display/counting window; no global building coverage percentage is computed.',fontsize=8)
    fig.tight_layout(rect=[0,.07,1,1]); fig.savefig(out,dpi=180); plt.close(fig)
    counts={str(k):int(np.sum(img==k)) for k in [-1,0,1]}
    return {'xy_m':roi,'resolution_m':res,'cell_counts':counts,
            'label_area_m2':{k:v*res*res for k,v in counts.items()},
            'unknown_semantics':'complement of exported cells within this explicit ROI only'}


def plot_multiview(cloud, trace, snapshot, out, reference_pose=None, source_note=None):
    xyz = cloud['xyz']; finite=np.isfinite(xyz).all(1)
    # Raw NPZ is never changed; the fixed display bounds retain the local structure.
    # Bounds follow the current generation's measured domain, excluding only
    # remote extreme tails from this local display. All raw points remain.
    roi=[[-12,24],[-22,34],[-1,10]]
    keep=finite.copy()
    for j,(lo,hi) in enumerate(roi): keep &= (xyz[:,j]>=lo)&(xyz[:,j]<=hi)
    pts=xyz[keep]
    if len(pts)>150000:
        pts=pts[np.random.default_rng(230).choice(len(pts),150000,replace=False)]
    fig=plt.figure(figsize=(16,12))
    for i,(az,el) in enumerate([(-58,28),(120,28),(-30,70),(35,10)],1):
        ax=fig.add_subplot(2,2,i,projection='3d',computed_zorder=False)
        sc=ax.scatter(*pts.T,c=pts[:,2],cmap='viridis',vmin=-1,vmax=10,s=.25,alpha=.42,rasterized=True,zorder=1)
        # Deliberate overlay makes the actual trajectory readable through walls.
        # It must not be interpreted as a line-of-sight or obstacle-free route.
        ax.plot(trace[:,1],trace[:,2],trace[:,3],color='#ff714f',lw=2,zorder=10)
        ax.scatter(*trace[-1,1:4],c='#ff3526',s=30,zorder=11)
        if reference_pose is not None:
            ax.scatter(*reference_pose['xyz_m'],c='#161616',s=38,marker='x',zorder=12)
            if 'yaw_rad' in reference_pose:
                ax.quiver(*reference_pose['xyz_m'],np.cos(reference_pose['yaw_rad']),
                          np.sin(reference_pose['yaw_rad']),0,color='#161616',length=1,
                          arrow_length_ratio=.25,zorder=13)
        ax.view_init(el,az); ax.set(xlim=roi[0],ylim=roi[1],zlim=roi[2],
            xlabel='map x (m)',ylabel='map y (m)',zlabel='map z (m)',title=f'azimuth {az}, elevation {el}')
        ax.set_box_aspect([36,56,15.4])
        for axis in [ax.xaxis,ax.yaxis,ax.zaxis]:axis.set_major_locator(MaxNLocator(5))
    upper_note=('physical upper region partially observed' if review_step(snapshot)>=19
                else 'physical upper-floor extent remains unconfirmed')
    fig.suptitle(f'Measured LIO structure / {snapshot}\nActual postVIO trajectory in orange; {upper_note}',fontsize=15)
    fig.colorbar(sc,ax=fig.axes,shrink=.55,pad=.025,label='measured return z (m)')
    fig.text(.04,.025,f'Fixed local display ROI x[-12,24], y[-22,34], z[-1,10] m. {int(keep.sum()):,}/{len(xyz):,} points inside; '
             f'{int((~keep).sum()):,} outside preserved in raw NPZ.\n'
             'Orange trace deliberately overlays cloud for visibility; visibility is not line-of-sight or obstacle clearance. Vertical scale x1.40.\n'
             'Point projections are not free-space fill, support evidence or completion claims. Cloud and pose snapshot times differ.',fontsize=9)
    if source_note:
        fig.text(.04,.085,source_note,fontsize=9,color='#8b3022')
    fig.savefig(out,dpi=160);plt.close(fig)
    return {'display_roi_xyz_m':roi,'raw_point_count':len(xyz),'inside_display_count':int(keep.sum()),
            'outside_display_count':int((~keep).sum()),'display_sample_count':len(pts),
            'distant_tail_count_abs_x_gt_30_or_y_lt_minus30_or_z_lt_minus5':int(((np.abs(xyz[:,0])>30)|(xyz[:,1]<-30)|(xyz[:,2]<-5)).sum())}


def plot_remote_overview(cloud,trace,snapshot,out):
    xyz=cloud['xyz'];keep=np.isfinite(xyz).all(1)
    roi=[[-12,24],[-22,70],[-1,11]]
    for j,(lo,hi) in enumerate(roi):keep&=(xyz[:,j]>=lo)&(xyz[:,j]<=hi)
    pts=xyz[keep]
    if len(pts)>250000:pts=pts[np.random.default_rng(231).choice(len(pts),250000,replace=False)]
    fig=plt.figure(figsize=(13,12));gs=fig.add_gridspec(2,2,width_ratios=[1,1.35],hspace=.27,wspace=.27)
    ax=fig.add_subplot(gs[:,0]);sc=ax.scatter(pts[:,0],pts[:,1],c=pts[:,2],vmin=-1,vmax=11,cmap='viridis',s=.25,alpha=.7)
    ax.plot(trace[:,1],trace[:,2],color='#f2693b',lw=1.4);ax.scatter(*trace[-1,1:3],c='#e63e29',s=25)
    ax.set(xlim=roi[0],ylim=roi[1],xlabel='map x (m)',ylabel='map y (m)',title='Measured XY: broad return domain');ax.set_aspect('equal');ax.grid(alpha=.2)
    ax.annotate('Remote high-z returns\nnot visited / no floor assignment',xy=(-7,57),xytext=(-9,38),fontsize=8,
                bbox={'fc':'#fff6dc','ec':'#ae8e51','boxstyle':'round,pad=.4'},arrowprops={'arrowstyle':'->','color':'#947439'})
    side=fig.add_subplot(gs[0,1]);side.scatter(pts[:,1],pts[:,2],c=pts[:,2],vmin=-1,vmax=11,cmap='viridis',s=.25,alpha=.5)
    side.plot(trace[:,2],trace[:,3],color='#f2693b',lw=1.6);side.set(xlim=roi[1],ylim=roi[2],xlabel='map y (m)',ylabel='map z (m)',title='Measured YZ: high-z remote returns');side.grid(alpha=.2)
    ax3=fig.add_subplot(gs[1,1],projection='3d',computed_zorder=False)
    ax3.scatter(*pts.T,c=pts[:,2],vmin=-1,vmax=11,cmap='viridis',s=.18,alpha=.4,zorder=1)
    ax3.plot(trace[:,1],trace[:,2],trace[:,3],color='#f2693b',lw=2,zorder=10)
    ax3.set(xlim=roi[0],ylim=roi[1],zlim=roi[2],xlabel='map x (m)',ylabel='map y (m)',zlabel='map z (m)',title='Broad measured 3D domain')
    ax3.set_box_aspect([36,92,20]);ax3.view_init(27,-62)
    for axis in [ax3.xaxis,ax3.yaxis,ax3.zaxis]:axis.set_major_locator(MaxNLocator(4))
    fig.suptitle(f'Current-generation distant-return overview / {snapshot}\nActual postVIO trace in orange; measured extent is not traversal coverage',fontsize=14)
    fig.colorbar(sc,ax=[ax,side,ax3],shrink=.55,pad=.025,label='measured return z (m)')
    far=(xyz[:,1]>40)&np.isfinite(xyz).all(1)
    fig.text(.05,.035,f'{int(far.sum()):,} measured points at y>40m retained; their physical region/floor is not established. '
             f'Display bounds x[-12,24], y[-22,70], z[-1,11].\n'
             f'{int((~keep).sum())} points outside this display remain in raw NPZ. No fill, physical support, clearance or completion inference.',fontsize=9)
    fig.savefig(out,dpi=170);plt.close(fig)
    return {'display_roi_xyz_m':roi,'inside_display_count':int(keep.sum()),'outside_display_count':int((~keep).sum()),
            'remote_y_gt_40_count':int(far.sum()),'remote_y_gt_40_xyz_min_m':xyz[far].min(0).tolist() if far.any() else None,
            'remote_y_gt_40_xyz_max_m':xyz[far].max(0).tolist() if far.any() else None}


def plot_trajectory_profile(trace,snapshot,out):
    fig,axes=plt.subplots(1,2,figsize=(13,4.7))
    axes[0].plot(trace[:,0],trace[:,3],color='#db5c32',lw=1.4)
    axes[0].set(xlabel='postVIO source time (s)',ylabel='estimated pose-origin z (m)',title='Actual estimated elevation history')
    sc=axes[1].scatter(trace[:,2],trace[:,3],c=trace[:,0],s=3,cmap='plasma')
    axes[1].set(xlabel='estimated map y (m)',ylabel='estimated pose-origin z (m)',title='Elevation versus y (color = source time)')
    fig.colorbar(sc,ax=axes[1],label='source time (s)')
    for ax in axes:ax.grid(alpha=.25)
    fig.suptitle(f'Current generation / {snapshot}: actual postVIO measurements')
    fig.text(.05,.025,'Pose origin is go2_imu, not floor height. Elevation alone does not establish a separate physical storey.',fontsize=9)
    fig.tight_layout(rect=[0,.07,1,.95]);fig.savefig(out,dpi=180);plt.close(fig)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--snapshot',default='r07');args=ap.parse_args()
    evidence=ROOT/'repaired_evidence'/args.snapshot;out=ROOT/'repaired_coverage'/args.snapshot
    out.mkdir(parents=True,exist_ok=True)
    obs=read_json(evidence/'observation.json');summary=read_json(evidence/'summary.json')
    scene=read_json(evidence/'scene_view_metadata.json');rgb=read_json(evidence/'rgb_source.json')
    if summary['run']!=RUN_ID or scene['run_id']!=RUN_ID:raise ValueError('Wrong generation')
    trace_path,trace=navigation_trace(scene['pose_source_stamp_s'])
    receipts=receipt_audit(obs)
    (out/'receipt_audit.json').write_text(json.dumps(receipts,indent=2,ensure_ascii=False)+'\n')
    with (out/'actual_postvio_trajectory.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['source_time_s','x_m','y_m','z_m','yaw_rad']);w.writerows(trace)
    manifest_paths=sorted(evidence.glob('level*_operator_map_partial.json'))
    manifests=[read_json(p) for p in manifest_paths]
    grids=[]
    for p,m in zip(manifest_paths,manifests):
        grids.append({'path':str(p),'manifest':m,'npz':np.load(p.with_suffix('.npz'))})
    lower=[v for v in grids if v['manifest']['level_index']==0]
    # Names such as +0.01 and +0.02 are numeric chart keys, not physical floors.
    if not lower:raise ValueError('No level_index=0 snapshot export')
    grid_entry=max(lower,key=lambda v:v['manifest']['source_clock_used'])
    m=grid_entry['manifest']
    roi_metrics=plot_grid(grid_entry['npz'],m,trace,args.snapshot,out/'lower_region_2d.png',
                          f'level_index=0, estimated pose-height chart {m["level_z_m"]:.3f} m')
    cloud=np.load(evidence/'measured_cloud.npz')
    render=plot_multiview(cloud,trace,args.snapshot,out/'measured_structure_multiview.png')
    remote=plot_remote_overview(cloud,trace,args.snapshot,out/'distant_measured_return_overview.png')
    plot_trajectory_profile(trace,args.snapshot,out/'actual_postvio_elevation_profile.png')
    record={
        'schema':'repaired_coverage_record/v1','run_id':RUN_ID,'snapshot':args.snapshot,
        'read_only':True,'ground_truth_used':False,'raw_files_modified':False,
        'sources':{'evidence_dir':str(evidence),'navigation_log':str(trace_path),
            'navigation_pose_source':'livo_post_vio','trajectory_frame':'dynanav_lio_map',
            'trajectory_child_frame':'go2_imu','cloud_stage':'LIO',
            'cloud_input':'already-map-frame deskewed points; no additional transform',
            'cloud_pair':'exact integer source stamp with same-stage LIO odometry'},
        'source_times_s':{'rgb':rgb['source_time_s'],'cloud_first':summary['first_source_stamp_s'],
            'cloud_last':summary['last_source_stamp_s'],'postvio_snapshot':scene['pose_source_stamp_s'],
            'grid_clock':m['source_clock_used'],'not_same_frame':True},
        'receipt_audit':receipts,
        'trajectory':{'samples':len(trace),'first_source_s':float(trace[0,0]),'last_source_s':float(trace[-1,0]),
            'length_m':float(np.linalg.norm(np.diff(trace[:,1:4],axis=0),axis=1).sum()),
            'maximum_source_sample_gap_s':float(np.diff(trace[:,0]).max()),
            'xyz_min_m':trace[:,1:4].min(0).tolist(),'xyz_max_m':trace[:,1:4].max(0).tolist(),
            'csv':str(out/'actual_postvio_trajectory.csv'),'semantics':'actual distinct measured postVIO samples, not waypoint lines'},
        'lower_region':{'status':'partially observed and partially traversed','grid_level_index':0,
            'physical_layer_assignment':'observed current pose-height region, not whole building floor',
            'grid_global_counts':m['counts'],'grid_global_labeled_area_m2':m['counts']['total']*m['resolution_m']**2,
            'roi_metrics':roi_metrics},
        'upper_region':{'status':('physically entered corridor / upper region; extent and traversal incomplete'
                                 if review_step(args.snapshot)>=19 else
                                 'physical upper-region extent not established by this snapshot'),
            'grid_level_indices_observed':sorted(set(v['manifest']['level_index'] for v in grids)),
            'separate_physical_upper_grid_available':False,'coverage_percentage':None,
            'rule':'height-chart drift at level_index=0 is never a second physical floor'},
        'render':render,
        'remote_return_overview':remote,
        'limitations':['Floor/free labels are not proof of continuous physical support or robot clearance.',
            'Occupied stair projections above the current height band are not a no-go verdict.',
            'Two distant tail points do not prove robot descent, a new floor or large coverage.',
            'Collector started at 4.4s; cumulative point cloud is a partial current-run map.',
            'No original scene boundary is known; no whole-building completion denominator is available.'],
        'gaps':[{'id':'A','region':'lower hall +y long axis','state':'remote returns only; close and reverse viewpoints incomplete',
                 'new_view_type':'closer viewpoint down hall and reverse view toward occluding wall/door sides'},
                {'id':'B','region':'south stair from west fore-room','state':'partial rising tread/edge returns; far landing/turn occluded',
                 'new_view_type':'view aligned with stair from observed near platform, then changed side/height if actually visited'},
                {'id':'C','region':'north stair and upper-side space','state':'partial elevated returns; endpoint and physical upper region unconfirmed',
                 'new_view_type':'view aligned with stair and later a view beyond its observed occluding edge'}]
    }
    physical_config=ROOT/'repaired_coverage'/f'{args.snapshot}_physical_config.json'
    if physical_config.exists():
        record['physical_display_regions']=read_json(physical_config)
        record['physical_display_region_record']=str(out/'physical_regions/physical_slice_record.json')
    if review_step(args.snapshot)>=19:
        record['gaps']=[
            {'id':'L1','region':'lower hall +y long axis','state':'remote measured returns, not actually traversed',
             'new_view_type':'closer along-axis and reverse viewpoints'},
            {'id':'U1','region':'upper +y long axis','state':'remote measured returns, no upper actual traversal in this snapshot',
             'new_view_type':'along-axis closer viewpoint and reverse views around furniture'},
            {'id':'U2','region':'upper +x hall wing','state':'remote measured returns, actual south-edge traversal incomplete',
             'new_view_type':'closer lateral and reverse views'},
            {'id':'O1','region':'furniture backs / concave curved boundaries / far corners',
             'state':'occluded or incompletely viewed; blank z-slice pixels are not an unknown-space classification',
             'new_view_type':'changed side viewpoints beyond the observed occluding edges'}]
    (out/'coverage_record.json').write_text(json.dumps(record,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'out':str(out),'trace_samples':len(trace),'last_source_s':trace[-1,0],
                      'trace_length_m':record['trajectory']['length_m'],'grid_area_m2':record['lower_region']['grid_global_labeled_area_m2']},indent=2))


if __name__=='__main__':main()
