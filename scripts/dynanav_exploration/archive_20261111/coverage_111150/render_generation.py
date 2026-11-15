#!/usr/bin/env python3
"""Read-only, explicitly bound current-generation coverage audit and rendering.

No ROS publisher, simulator launcher, truth, scene geometry or previous run
input is used. Geometry/region defaults are intentionally absent.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import textwrap

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.colors import ListedColormap
from matplotlib.ticker import MaxNLocator
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np


def read(path):return json.loads(path.read_text())
def write(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def bound_path(path,binding):
    resolved=Path(path).resolve()
    if not any(resolved.is_relative_to(Path(p).resolve()) for p in binding['allowed_input_roots']):
        raise ValueError(f'Input outside explicitly bound roots: {resolved}')
    return resolved


def load_trace(path,binding,cutoff):
    # Read only navigation-produced estimated fields, never native policy state.
    rows={}
    for line in path.open():
        try:row=json.loads(line)
        except json.JSONDecodeError:continue
        t=row.get('odom_stamp_s');xyz=row.get('aligned_position_xyz_m');yaw=row.get('aligned_yaw_rad')
        if (row.get('pose_source')!=binding['pose_source'] or row.get('no_gt_pose_used') is not True
            or row.get('frame_id')!=binding['map_frame'] or row.get('child_frame_id')!=binding['pose_child_frame']
            or not isinstance(t,(int,float)) or t>cutoff or xyz is None or yaw is None):continue
        values=[t,*xyz,yaw]
        if np.isfinite(values).all():rows[t]=values
    trace=np.asarray([rows[t] for t in sorted(rows)],dtype=float)
    if not len(trace):raise ValueError('No available aligned postVIO navigation samples')
    return trace


def receipts(path,observation):
    raw=[]
    for line in path.open():
        try:raw.append(json.loads(line))
        except json.JSONDecodeError:continue
    recorded=observation.get('receipts',[])
    if not recorded:prefix=[]
    else:
        matches=[i for i,v in enumerate(raw) if v==recorded[-1]]
        if len(matches)!=1:raise ValueError('Ambiguous snapshot receipt anchor')
        prefix=raw[:matches[0]+1]
    actions={};active=None
    for row in prefix:
        event=row['event'];seq=row['seq']
        if event=='waypoint_accepted':
            actions[seq]={'seq':seq,'action':'goto','goal_xyz_m':row['goal_xyz_m'],'accepted':True,'state':'accepted'};active=seq
        elif event=='waypoint_reached':
            actions[seq].update(state='reached',reached=True)
            if active==seq:active=None
        elif event=='waypoint_failed':
            actions[seq].update(state='failed',failed=True,reason=row.get('reason'))
            if active==seq:active=None
        elif event=='cancel_accepted':
            actions[seq]={'seq':seq,'action':'cancel','accepted':True,'applied_to_seq':active}
            if active is not None:actions[active].update(state='canceled_not_reached',canceled_by_seq=seq)
            active=None
        elif event=='stop_accepted':
            actions[seq]={'seq':seq,'action':'stop','accepted':True}
            if active is not None:actions[active].update(state='interrupted_by_stop_not_reached',stopped_by_seq=seq)
            active=None
        elif event=='waypoint_rejected':
            actions[seq]={'seq':seq,'action':'goto','accepted':False,'state':'rejected','reason':row.get('reason')}
    return {'prefix_count':len(prefix),'actions':[actions[k] for k in sorted(actions)],
        'goto_accepted':sum(a['action']=='goto' and a.get('accepted',False) for a in actions.values()),
        'goto_reached':sum(a.get('reached',False) for a in actions.values()),
        'goto_failed':sum(a.get('failed',False) for a in actions.values()),
        'goto_canceled':sum(a.get('canceled_by_seq') is not None for a in actions.values()),
        'active_seq':active,'limitation':'No receipt times; prefix ends at exact last receipt already present in snapshot. Execution facts are not room coverage.'}


def segments(trace,mask=None,max_gap=.5):
    if len(trace)<2:return np.empty((0,2,3))
    keep=np.diff(trace[:,0])<=max_gap
    if mask is not None:keep&=mask[:-1]&mask[1:]
    return np.stack([trace[:-1,1:4],trace[1:,1:4]],axis=1)[keep]


def measured_length(trace,axes=(0,1,2)):
    seg=segments(trace)
    return float(np.linalg.norm((seg[:,1]-seg[:,0])[:,list(axes)],axis=1).sum()) if len(seg) else 0.


def reference_text(reference):
    if reference is None:return 'No pose marker available.'
    if reference.get('reference_kind')=='last_actual_postvio':
        return (f'Black x/yaw = last actual navigation postVIO source {reference["source_s"]:.3f}s; terminal fresh-pose payload unavailable, '
                f'freshness not asserted; cloud minus pose={reference["cloud_minus_pose_s"]:.3f}s; obs minus pose={reference["observation_minus_pose_s"]:.3f}s.')
    return (f'Black x/yaw = {reference["snapshot"]} source {reference["source_s"]:.3f}s; '
            f'fresh_at_that_snapshot={reference["fresh_at_snapshot"]}; stale_at_current_snapshot={reference["stale_at_current_snapshot"]}; '
            f'cloud minus pose={reference["cloud_minus_pose_s"]:.3f}s; obs minus pose={reference["observation_minus_pose_s"]:.3f}s.')


def bounds_from_points(xyz):
    # Full measured domain: no percentile cut or implicit previous-scene ROI.
    lo=xyz.min(0);hi=xyz.max(0);pad=np.maximum((hi-lo)*.025,.25)
    return np.stack([lo-pad,hi+pad],axis=1)


def sample(xyz,limit):
    return xyz if len(xyz)<=limit else xyz[np.random.default_rng(232).choice(len(xyz),limit,replace=False)]


def draw_trace_2d(ax,trace,axes,mask=None,color='#e76a30'):
    seg=segments(trace,mask)
    if len(seg):ax.add_collection(LineCollection(seg[:,:,list(axes)],colors=color,linewidths=1.2,zorder=7))


def draw_trace_3d(ax,trace,mask=None):
    seg=segments(trace,mask)
    if len(seg):ax.add_collection3d(Line3DCollection(seg,colors='#e76a30',linewidths=1.5,zorder=10))


def draw_gap_annotations(ax,config,region_id):
    for gap in config.get('annotations',[]):
        if gap['region_id']!=region_id:continue
        ax.annotate(gap['label'],xy=gap['anchor_xy_m'],xytext=gap['label_xy_m'],fontsize=8,color='#8c2020',
                    bbox={'boxstyle':'round,pad=.3','fc':'white','ec':'#a33a3a','alpha':.94},
                    arrowprops={'arrowstyle':'->','color':'#8c2020','lw':1.1},zorder=25)


def marker(ax,reference,axes=None):
    if reference is None:return
    xyz=np.asarray(reference['xyz_m']);yaw=reference.get('yaw_rad')
    if axes is None:
        ax.scatter(*xyz,color='#131313',marker='x',s=35,zorder=15)
        if yaw is not None:ax.quiver(*xyz,np.cos(yaw),np.sin(yaw),0,color='#131313',length=1,zorder=16)
    else:
        ax.scatter(xyz[axes[0]],xyz[axes[1]],color='#131313',marker='x',s=35,zorder=15)
        if yaw is not None:
            delta=np.array([np.cos(yaw),np.sin(yaw),0.])
            ax.arrow(xyz[axes[0]],xyz[axes[1]],delta[axes[0]],delta[axes[1]],color='#131313',
                     head_width=.15,length_includes_head=True,zorder=16)


def render_views(xyz,trace,reference,run_id,name,dest,roi=None):
    finite=np.isfinite(xyz).all(1);keep=finite.copy();bounds=bounds_from_points(xyz[finite]) if roi is None else np.asarray(roi,dtype=float)
    trace_mask=None
    if roi is not None:
        for i,(lo,hi) in enumerate(bounds):keep&=(xyz[:,i]>=lo)&(xyz[:,i]<=hi)
        trace_mask=np.ones(len(trace),dtype=bool)
        for i,(lo,hi) in enumerate(bounds):trace_mask&=(trace[:,i+1]>=lo)&(trace[:,i+1]<=hi)
    selected=xyz[keep];pts=sample(selected,150000);vmin=float(selected[:,2].min());vmax=float(selected[:,2].max())
    fig=plt.figure(figsize=(15,12));panels=[(0,1,'XY'),(0,2,'XZ'),(1,2,'YZ')]
    for i,(a,b,title) in enumerate(panels,1):
        ax=fig.add_subplot(2,2,i);sc=ax.scatter(pts[:,a],pts[:,b],c=pts[:,2],cmap='viridis',vmin=vmin,vmax=vmax,s=.3,alpha=.55,rasterized=True)
        draw_trace_2d(ax,trace,(a,b),trace_mask);marker(ax,reference,(a,b));ax.set(xlim=bounds[a],ylim=bounds[b],xlabel=f'map {"xyz"[a]} (m)',ylabel=f'map {"xyz"[b]} (m)',title=title)
        ax.grid(alpha=.2)
        if title=='XY':ax.set_aspect('equal',adjustable='box')
    ax=fig.add_subplot(2,2,4,projection='3d',computed_zorder=False)
    ax.scatter(*pts.T,c=pts[:,2],cmap='viridis',vmin=vmin,vmax=vmax,s=.25,alpha=.4,zorder=1,rasterized=True)
    draw_trace_3d(ax,trace,trace_mask)
    marker(ax,reference);ax.set(xlim=bounds[0],ylim=bounds[1],zlim=bounds[2],xlabel='map x (m)',ylabel='map y (m)',zlabel='map z (m)',title='Measured 3D')
    ax.view_init(28,-58);ax.set_box_aspect(np.maximum(np.diff(bounds).ravel(),.5))
    for axis in [ax.xaxis,ax.yaxis,ax.zaxis]:axis.set_major_locator(MaxNLocator(4))
    fig.suptitle(f'{run_id} / {name}\nMeasured LIO points + actual postVIO trace',fontsize=14)
    fig.subplots_adjust(left=.06,right=.87,bottom=.12,top=.88,hspace=.28,wspace=.20)
    cax=fig.add_axes([.92,.24,.015,.50]);fig.colorbar(sc,cax=cax,label='measured return z (m)')
    ref=reference_text(reference)
    note=(f'{ref} Orange line deliberately overlays returns; not visibility, support or clearance proof. '
          f'{len(selected):,}/{len(xyz):,} input voxel points in display, {len(pts):,} plotted. Full input voxels retained (5 cm provenance), not all raw sensor returns. '
          'Local views clip both returns and actual trace samples to the same 3-axis ROI. '
          'Measured footprint and trajectory length are not room completion. Different sensor source times remain separate.')
    fig.text(.04,.025,'\n'.join(textwrap.wrap(note,170)),fontsize=8)
    fig.savefig(dest,dpi=160);plt.close(fig)
    fig=plt.figure(figsize=(14,11))
    for i,(az,el) in enumerate([(-58,28),(120,28),(-30,70),(35,10)],1):
        ax=fig.add_subplot(2,2,i,projection='3d',computed_zorder=False)
        sc=ax.scatter(*pts.T,c=pts[:,2],cmap='viridis',vmin=vmin,vmax=vmax,s=.25,alpha=.4,zorder=1,rasterized=True)
        draw_trace_3d(ax,trace,trace_mask)
        marker(ax,reference);ax.view_init(el,az);ax.set(xlim=bounds[0],ylim=bounds[1],zlim=bounds[2],xlabel='map x (m)',ylabel='map y (m)',zlabel='map z (m)',title=f'azimuth {az}, elevation {el}')
        ax.set_box_aspect(np.maximum(np.diff(bounds).ravel(),.5))
        for axis in [ax.xaxis,ax.yaxis,ax.zaxis]:axis.set_major_locator(MaxNLocator(4))
    fig.suptitle(f'{run_id} / {name}: four measured 3D views',fontsize=14)
    fig.subplots_adjust(left=.04,right=.87,bottom=.10,top=.88,hspace=.18,wspace=.15)
    cax=fig.add_axes([.92,.25,.015,.50]);fig.colorbar(sc,cax=cax,label='return z (m)')
    note=ref+' Orange = actual measured postVIO samples, overlaid for visibility. No support, clearance or completion claim.'
    fig.text(.04,.025,'\n'.join(textwrap.wrap(note,155)),fontsize=8)
    angles=dest.with_name(dest.stem+'_3d_angles.png');fig.savefig(angles,dpi=160);plt.close(fig)
    valid_trace=np.isfinite(trace[:,0])
    return {'bounds_xyz_m':bounds.tolist(),'raw_points':len(xyz),'input_voxel_points':len(xyz),'legacy_raw_points_key_semantics':'display-input voxel points; not all raw sensor returns','selected_points':len(selected),'plotted_points':len(pts),'sampling_seed':232,'raw_points_outside_display':int((~keep).sum()),'actual_trace_samples_in_display':int(valid_trace.sum()) if trace_mask is None else int((trace_mask&valid_trace).sum()),'trace_crop_is_display_only':roi is not None,'views':[str(dest),str(angles)]}


def render_region(xyz,trace,region,run_id,dest):
    required=['region_id','name','assignment_evidence_refs','display_xy_roi_m','display_point_z_band_m','display_pose_z_band_m']
    if any(k not in region for k in required) or not region['assignment_evidence_refs']:raise ValueError('Physical regions require explicit current-generation assignment evidence')
    roi=np.asarray(region['display_xy_roi_m'],float);z=np.asarray(region['display_point_z_band_m'],float);tz=np.asarray(region['display_pose_z_band_m'],float)
    mask=(xyz[:,0]>=roi[0])&(xyz[:,0]<=roi[1])&(xyz[:,1]>=roi[2])&(xyz[:,1]<=roi[3])&(xyz[:,2]>=z[0])&(xyz[:,2]<=z[1]);pts=xyz[mask]
    selected=(trace[:,3]>=tz[0])&(trace[:,3]<=tz[1]);fig,ax=plt.subplots(figsize=(10,12));fig.subplots_adjust(bottom=.18,top=.89)
    sc=ax.scatter(pts[:,0],pts[:,1],c=pts[:,2],cmap='viridis',s=.35,alpha=.6,rasterized=True)
    draw_trace_2d(ax,trace,(0,1),color='#9b9b9b');draw_trace_2d(ax,trace,(0,1),selected)
    ax.set(xlim=roi[:2],ylim=roi[2:],xlabel='estimated map x (m)',ylabel='estimated map y (m)',title=region['name']);ax.set_aspect('equal');ax.grid(alpha=.2)
    ax.legend(handles=[Line2D([0],[0],color='#9b9b9b',lw=1.5,label='actual postVIO, all heights'),Line2D([0],[0],color='#e76a30',lw=1.5,label='actual samples in region pose-z band')],fontsize=8)
    fig.suptitle(f'{run_id}: '+region.get('figure_semantics','explicitly assigned physical-region display slice'),fontsize=12);fig.colorbar(sc,ax=ax,shrink=.65,label='measured return z (m)')
    note=f'Point z=[{z[0]:.3f}, {z[1]:.3f}] m; go2_imu trace z=[{tz[0]:.3f}, {tz[1]:.3f}] m. No fill; blank pixels are no plotted return, not an unknown/free classification. Display z bands are not a uniform floor or physical support proof.'
    fig.text(.07,.115,'\n'.join(textwrap.wrap(note,105)),fontsize=8,va='top');fig.savefig(dest,dpi=170);plt.close(fig)
    return {'region_id':region['region_id'],'points_in_slice':len(pts),'actual_samples_in_pose_z_band':int(selected.sum()),'image':str(dest),'configuration':region}


def render_grid(grid,manifest,trace,reference,run_id,dest):
    res=manifest['resolution_m'];ix=grid['ix'];iy=grid['iy'];code=grid['code']
    # Bounds come only from this snapshot's actual grid and trajectory.
    xy=np.stack([ix,iy],axis=1)*res
    lo=np.minimum(xy.min(0),trace[:,1:3].min(0))-1
    hi=np.maximum(xy.max(0)+res,trace[:,1:3].max(0))+1
    fig,ax=plt.subplots(figsize=(9,12));ax.set_facecolor('#efefef')
    x0,y0=np.floor(lo/res).astype(int);x1,y1=np.ceil(hi/res).astype(int)
    if (x1-x0)*(y1-y0)<=20000000:
        cells=np.full((y1-y0,x1-x0),-1,dtype=np.int8)
        cells[iy-y0,ix-x0]=code
        ax.imshow(cells+1,origin='lower',extent=[x0*res,x1*res,y0*res,y1*res],
                  cmap=ListedColormap(['#efefef','#b9dce7','#283243']),vmin=0,vmax=2,interpolation='none')
    else:
        # Sparse fallback retains export cells without allocating a huge ROI.
        for value,color in [(0,'#b9dce7'),(1,'#283243')]:
            mask=code==value;ax.scatter((ix[mask]+.5)*res,(iy[mask]+.5)*res,c=color,s=1.2,marker='s',linewidths=0,rasterized=True)
    draw_trace_2d(ax,trace,(0,1));marker(ax,reference,(0,1))
    ax.set(xlim=[lo[0],hi[0]],ylim=[lo[1],hi[1]],xlabel='estimated map x (m)',ylabel='estimated map y (m)',
           title=f'{run_id}\nHeight chart level_index={manifest["level_index"]}, chart z={manifest["level_z_m"]:.3f} m')
    ax.set_aspect('equal');ax.grid(alpha=.2)
    ax.legend(handles=[Line2D([0],[0],color='#b9dce7',lw=6,label='floor/free label'),Line2D([0],[0],color='#283243',lw=6,label='occupied/elevated-return label'),Line2D([0],[0],color='#e76a30',lw=2,label='actual postVIO XY trace, all heights')],fontsize=8)
    fig.text(.06,.03,'Automatically bounded by current grid + actual trajectory. Gray = no exported cell in this explicit ROI.\n'
             'Grid labels are not physical support or clearance proof; elevated stair returns may be occupied.\n'
             'Height-chart count is not physical floor count. No whole-building completion percentage.',fontsize=8)
    fig.tight_layout(rect=[0,.07,1,1]);fig.savefig(dest,dpi=180);plt.close(fig)
    return {'explicit_display_xy_roi_m':[float(lo[0]),float(hi[0]),float(lo[1]),float(hi[1])],'image':str(dest)}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--binding',required=True,type=Path);ap.add_argument('--snapshot',required=True);ap.add_argument('--regions',type=Path);ap.add_argument('--reference-snapshot');ap.add_argument('--marker-from-postvio',action='store_true');ap.add_argument('--cloud-dir',type=Path,help='Explicit same-generation frozen collector directory when final snapshot has no cloud copy');args=ap.parse_args()
    binding=read(args.binding);run_id=binding['run_id']
    if not run_id or not re.fullmatch(r'[A-Za-z0-9_-]+',args.snapshot):raise ValueError('Explicit run and snapshot binding required')
    evidence=bound_path(Path(binding['evidence_root'])/args.snapshot,binding);out=Path(binding['output_root'])/args.snapshot;out.mkdir(parents=True,exist_ok=True)
    cloud_dir=bound_path(args.cloud_dir,binding) if args.cloud_dir else evidence
    cloud_path=bound_path(cloud_dir/('measured_partial_cloud.npz' if args.cloud_dir else 'measured_cloud.npz'),binding);summary=read(bound_path(cloud_dir/'summary.json',binding));obs=read(bound_path(evidence/'observation.json',binding))
    meta=read(bound_path(evidence/'scene_view_metadata.json',binding)) if (evidence/'scene_view_metadata.json').exists() else None
    rgb=read(bound_path(evidence/'rgb_source.json',binding));rgb_copy=bound_path(evidence/'rgb.png',binding);rgb_original=bound_path(rgb['path'],binding)
    if sha(rgb_copy)!=sha(rgb_original):raise ValueError('RGB snapshot copy differs from this-run source image')
    a=np.load(cloud_path,allow_pickle=False)
    if str(a['run_id'])!=run_id or summary['run']!=run_id or str(a['cloud_stage'])!=binding['cloud_stage'] or str(a['cloud_map_frame'])!=binding['map_frame']:raise ValueError('Generation/stage/frame mismatch')
    if meta and meta['run_id']!=run_id:raise ValueError('Reference pose generation mismatch')
    if meta and not (meta['estimated_pose_map_frame']==binding['map_frame'] and meta['estimated_pose_child_frame']==binding['pose_child_frame'] and meta['estimated_pose_source']==binding['pose_source']):raise ValueError('Estimated marker frame/source mismatch')
    xyz=a['xyz'];stamps=a['source_stamp_ns'];assert np.isfinite(xyz).all() and np.isfinite(a['lio_pose_xyz']).all() and np.all(np.diff(stamps)>0)
    latest=np.load(bound_path(cloud_dir/'latest_paired_cloud.npz',binding),allow_pickle=False)
    assert str(latest['run_id'])==run_id and bool(latest['exact_integer_stamp_pair']) and str(latest['cloud_stage'])==binding['cloud_stage']
    assert str(latest['cloud_map_frame'])==binding['map_frame'] and str(latest['pose_map_frame'])==binding['map_frame'] and str(latest['pose_child_frame'])==binding['pose_child_frame']
    nav=bound_path(Path(binding['run_root'])/'capture/interactive_waypoint_control.jsonl',binding)
    # A marker may be stale. It never truncates later actual measurements.
    cutoff=obs['sim_s'];trace=load_trace(nav,binding,cutoff)
    with (out/'actual_postvio_trajectory.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['source_time_s','x_m','y_m','z_m','yaw_rad']);w.writerows(trace)
    receipt_path=bound_path(Path(binding['run_root'])/'capture/interactive/receipts.jsonl',binding);audit=receipts(receipt_path,obs);write(out/'receipt_audit.json',audit)
    reference=None
    if args.marker_from_postvio:
        reference={'snapshot':args.snapshot,'reference_kind':'last_actual_postvio','source_s':float(trace[-1,0]),'xyz_m':trace[-1,1:4].tolist(),'yaw_rad':float(trace[-1,4]),'fresh_at_snapshot':None,'stale_at_current_snapshot':None,'freshness_not_asserted':True}
    elif meta and meta.get('pose_origin_xyz_m') is not None:
        reference={'snapshot':args.snapshot,'source_s':meta['pose_source_stamp_s'],'xyz_m':meta['pose_origin_xyz_m'],'yaw_rad':meta.get('pose_yaw_rad'),'fresh_at_snapshot':meta.get('pose_fresh_flag')}
    elif args.reference_snapshot:
        oldpath=bound_path(Path(binding['evidence_root'])/args.reference_snapshot/'scene_view_metadata.json',binding);old=read(oldpath)
        if old['run_id']!=run_id:raise ValueError('Old reference must be from this generation')
        if not (old['estimated_pose_map_frame']==binding['map_frame'] and old['estimated_pose_child_frame']==binding['pose_child_frame'] and old['estimated_pose_source']==binding['pose_source']):raise ValueError('Reference marker frame/source mismatch')
        reference={'snapshot':args.reference_snapshot,'source_s':old['pose_source_stamp_s'],'xyz_m':old['pose_origin_xyz_m'],'yaw_rad':old.get('pose_yaw_rad'),'fresh_at_snapshot':old.get('pose_fresh_flag'),'stale_at_current_snapshot':True}
    if reference is not None:
        if reference.get('reference_kind')!='last_actual_postvio':reference['stale_at_current_snapshot']=reference['snapshot']!=args.snapshot or reference['fresh_at_snapshot'] is not True or obs.get('pose',{}).get('fresh') is not True
        reference['cloud_minus_pose_s']=float(stamps[-1]/1e9-reference['source_s'])
        reference['observation_minus_pose_s']=float(obs['sim_s']-reference['source_s'])
    views=render_views(xyz,trace,reference,run_id,'full measured domain',out/'global_measured_multiview.png')
    local=None
    if meta and meta.get('local_roi_xyz_min_m') is not None:
        localroi=np.stack([meta['local_roi_xyz_min_m'],meta['local_roi_xyz_max_m']],axis=1)
        local=render_views(xyz,trace,reference,run_id,'snapshot-declared local ROI',out/'local_measured_multiview.png',localroi)
    regions=[]
    if args.regions:
        region_config=read(args.regions)
        if region_config['run_id']!=run_id:raise ValueError('Region configuration generation mismatch')
        for region in region_config['physical_regions']:
            region=dict(region)
            if region.get('display_bounds_mode')=='current_grid_and_trajectory':
                grid_xy=[]
                for p in evidence.glob('level*_operator_map_*.json'):
                    gm=read(bound_path(p,binding));gn=np.load(bound_path(p.with_suffix('.npz'),binding));res=gm['resolution_m']
                    grid_xy.append(np.stack([gn['ix'],gn['iy']],axis=1)*res)
                all_xy=np.concatenate([trace[:,1:3],*grid_xy]);lo=all_xy.min(0)-1;hi=all_xy.max(0)+1
                region['display_xy_roi_m']=[float(lo[0]),float(hi[0]),float(lo[1]),float(hi[1])]
            for evidence_ref in region['assignment_evidence_refs']:
                ref_path=bound_path(Path(binding['evidence_root'])/evidence_ref.split(':',1)[0],binding)
                if not ref_path.exists():raise ValueError('Missing current-generation region evidence')
            if region.get('renderer')=='relative_measured_surface':
                external=read(out/'upper_observed_record.json')
                if external['run_id']!=run_id or external['snapshot']!=args.snapshot:raise ValueError('Relative-region display must use this same snapshot and generation')
                regions.append({'region_id':region['region_id'],'image':external['bodyheight_image'],'configuration':region,'relative_measured_surface_display':external})
            else:regions.append(render_region(xyz,trace,region,run_id,out/f'{region["region_id"]}_region_2d.png'))
    grid_stats=[]
    for manifest_path in sorted(evidence.glob('level*_operator_map_*.json')):
        manifest_path=bound_path(manifest_path,binding)
        m=read(manifest_path);source=m.get('npz_path')
        if source and not Path(source).resolve().is_relative_to(Path(binding['run_root']).resolve()):raise ValueError('Grid provenance belongs to another run')
        grid_path=bound_path(manifest_path.with_suffix('.npz'),binding)
        if m.get('npz_sha256') and sha(grid_path)!=m['npz_sha256']:raise ValueError('Grid snapshot hash mismatch')
        pair=m.get('source_contract',{}).get('last_pair')
        if pair and not (pair['stage']==binding['cloud_stage'] and pair['cloud_stamp_ns']==pair['pose_stamp_ns']==pair['reference_stamp_ns'] and pair['map_frame']==binding['map_frame'] and pair['child_frame']==binding['pose_child_frame']):raise ValueError('Grid same-stage exact pairing metadata mismatch')
        grid=np.load(grid_path);res=m['resolution_m'];grid_view=render_grid(grid,m,trace,reference,run_id,out/f'height_chart_{m["level_index"]}_{manifest_path.stem}_2d.png');grid_stats.append({'manifest':str(manifest_path),'level_index':m['level_index'],'pose_height_chart_m':m['level_z_m'],'counts':{str(int(k)):int((grid['code']==k).sum()) for k in np.unique(grid['code'])},'labeled_projection_area_m2':len(grid['code'])*res**2,'not_physical_floor_count':True,'not_support_or_completion':True,'not_additive_physical_area':True,'display':grid_view})
    record={'schema':'coverage_review/v1','generation':binding,'snapshot':args.snapshot,'read_only':True,'source_generation_assertions_passed':True,'ground_truth_used':False,
        'source_timing':{'rgb_source_s':rgb['source_time_s'],'cloud_first_ns':int(stamps[0]),'cloud_last_ns':int(stamps[-1]),'postvio_first_s':float(trace[0,0]),'postvio_last_s':float(trace[-1,0]),'same_frame_synchrony_claimed':False},
        'pose_marker':reference,'final_observation_pose':obs.get('pose'),'assets':[{'kind':'measured_cloud','path':str(cloud_path),'sha256':sha(cloud_path),'run_id':str(a['run_id'])},{'kind':'navigation_log','path':str(nav),'sha256':sha(nav),'generation_binding':'canonical run root'}],
        'trajectory':{'source':binding['pose_source'],'frame':binding['map_frame'],'child_frame':binding['pose_child_frame'],'samples':len(trace),'estimated_length_3d_m':measured_length(trace),'estimated_length_xy_m':measured_length(trace,(0,1)),'length_excludes_missing_interval_chords':True,'max_source_gap_s':float(np.diff(trace[:,0]).max()) if len(trace)>1 else None,'missing_intervals_s':[[float(trace[i,0]),float(trace[i+1,0])] for i in np.where(np.diff(trace[:,0])>.5)[0]],'no_interpolation_or_waypoint_substitution':True,'length_is_coverage':False},
        'cloud_pairing':{'stage':str(a['cloud_stage']),'map_frame':str(a['cloud_map_frame']),'pairs':len(stamps),'exact_same_stage_pair':True,'no_second_transform':True,'latest_paired_cloud_source_ns':latest['source_stamp_ns'].tolist(),'latest_pair_and_cumulative_cloud_may_have_different_save_times':True},'grid_height_charts':grid_stats,'full_domain_render':views,'local_render':local,'physical_regions':regions,
        'limitations':['Review checks are evidence audits, not robot execution admission gates.','Height-chart count is not physical floor count.','Grid free label area is not physical support, traversability or room completeness.','No-return slice pixels, SLAM unknown, untraversed space and occluded surfaces are distinct.','Remote measured returns are retained and do not establish a new floor or complete coverage.','Missing fresh pose is not replaced or retimestamped.']}
    write(out/'coverage_record.json',record);print(json.dumps({'out':str(out),'run_id':run_id,'points':len(xyz),'trace_samples':len(trace),'physical_regions':len(regions)},indent=2))


if __name__=='__main__':main()
