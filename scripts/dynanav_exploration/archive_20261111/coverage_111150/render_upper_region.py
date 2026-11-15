#!/usr/bin/env python3
"""Upper observed-region display relative to an explicitly measured surface.

The surface model is a display reference in estimated map coordinates, never
navigation support, clearance, a physical true-world slope, or a motion gate.
"""
import argparse
from pathlib import Path
import textwrap
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
import render_generation as core


def main():
    p=argparse.ArgumentParser();p.add_argument('--binding',type=Path,required=True);p.add_argument('--snapshot',required=True);p.add_argument('--model',type=Path,required=True);p.add_argument('--first-source',type=float,required=True);p.add_argument('--reference-snapshot');p.add_argument('--cloud-dir',type=Path);p.add_argument('--marker-from-postvio',action='store_true');a=p.parse_args()
    b=core.read(a.binding);src=core.bound_path(Path(b['evidence_root'])/a.snapshot,b);out=Path(b['output_root'])/a.snapshot;out.mkdir(parents=True,exist_ok=True)
    cloud_dir=core.bound_path(a.cloud_dir,b) if a.cloud_dir else src
    cp=core.bound_path(cloud_dir/('measured_partial_cloud.npz' if a.cloud_dir else 'measured_cloud.npz'),b);latest_path=core.bound_path(cloud_dir/'latest_paired_cloud.npz',b);cloud=np.load(cp);latest=np.load(latest_path);obs=core.read(src/'observation.json');seed=core.read(a.model)
    reference_snapshot=a.snapshot
    if (src/'scene_view_metadata.json').exists():meta=core.read(src/'scene_view_metadata.json')
    elif a.reference_snapshot:
        reference_snapshot=a.reference_snapshot;meta=core.read(core.bound_path(Path(b['evidence_root'])/a.reference_snapshot/'scene_view_metadata.json',b))
    else:raise ValueError('Missing marker metadata requires an explicit same-generation reference snapshot')
    assert str(cloud['run_id'])==str(latest['run_id'])==b['run_id']==meta['run_id'] and bool(latest['exact_integer_stamp_pair'])
    assert str(cloud['cloud_stage'])==str(latest['cloud_stage'])==b['cloud_stage'] and str(cloud['cloud_map_frame'])==str(latest['cloud_map_frame'])==b['map_frame']
    assert seed['run_id']==b['run_id'] and seed['map_frame']==b['map_frame']
    assert meta['estimated_pose_map_frame']==b['map_frame'] and meta['estimated_pose_child_frame']==b['pose_child_frame'] and meta['estimated_pose_source']==b['pose_source']
    c=np.array(seed['coefficients_z_equals_ax_plus_by_plus_c']);fit_sources=[{'snapshot':a.snapshot,'path':str(latest_path),'source_stamp_s':float(latest['source_stamp_s']),'sha256':core.sha(latest_path)}];fit_clouds=[latest['xyz']]
    for snap in seed.get('additional_fit_evidence_snapshots',[]):
        fp=core.bound_path(Path(b['evidence_root'])/snap/'latest_paired_cloud.npz',b);fa=np.load(fp)
        assert str(fa['run_id'])==b['run_id'] and str(fa['cloud_stage'])==b['cloud_stage'] and str(fa['cloud_map_frame'])==b['map_frame'] and bool(fa['exact_integer_stamp_pair'])
        fit_clouds.append(fa['xyz']);fit_sources.append({'snapshot':snap,'path':str(fp),'source_stamp_s':float(fa['source_stamp_s']),'sha256':core.sha(fp)})
    q=np.concatenate(fit_clouds);roi=np.array(seed['fit_candidate_xy_roi_m'],dtype=float)
    # Extend the explicitly observed display as this generation moves onward.
    # The full-domain renderer independently preserves every measured point.
    if meta.get('local_roi_xyz_min_m') is not None:
        lo=meta['local_roi_xyz_min_m'];hi=meta['local_roi_xyz_max_m']
        roi=np.array([min(roi[0],lo[0]),max(roi[1],hi[0]),min(roi[2],lo[1]),max(roi[3],hi[1])])
    xmin,xmax,ymin,ymax=roi
    def surface(x):return x[:,:2]@c[:2]+c[2]
    m=np.isfinite(q).all(1)&(q[:,0]>=xmin)&(q[:,0]<=xmax)&(q[:,1]>=ymin)&(q[:,1]<=ymax)&(abs(q[:,2]-surface(q))<.10)
    for _ in range(3):
        c=np.linalg.lstsq(np.c_[q[m,:2],np.ones(m.sum())],q[m,2],rcond=None)[0];m=np.isfinite(q).all(1)&(q[:,0]>=xmin)&(q[:,0]<=xmax)&(q[:,1]>=ymin)&(q[:,1]<=ymax)&(abs(q[:,2]-surface(q))<.06)
    fit=q[m];res=fit[:,2]-surface(fit);support=[float(fit[:,0].min()),float(fit[:,0].max()),float(fit[:,1].min()),float(fit[:,1].max())]
    xyz=cloud['xyz'];h=xyz[:,2]-surface(xyz);inroi=(xyz[:,0]>=xmin)&(xyz[:,0]<=xmax)&(xyz[:,1]>=ymin)&(xyz[:,1]<=ymax)
    post=cloud['voxel_first_source_ns']>=round(a.first_source*1e9)
    candidate=inroi&(h>=-.12)&(h<=1.80);body=inroi&post&(h>=.15)&(h<=1.50);floor=inroi&post&(abs(h)<=.08);full=candidate&post
    unsupported=(xyz[:,0]<support[0])|(xyz[:,0]>support[1])|(xyz[:,1]<support[2])|(xyz[:,1]>support[3])
    excluded_early=int((candidate&~post).sum());outside_support=int((full&unsupported).sum())
    trace=core.load_trace(core.bound_path(Path(b['run_root'])/'capture/interactive_waypoint_control.jsonl',b),b,obs['sim_s']);upper_actual_start_s=1288.7
    tm=(trace[:,0]>=upper_actual_start_s)&(trace[:,1]>=xmin)&(trace[:,1]<=xmax)&(trace[:,2]>=ymin)&(trace[:,2]<=ymax)
    ref={'snapshot':reference_snapshot,'source_s':meta['pose_source_stamp_s'],'xyz_m':meta['pose_origin_xyz_m'],'yaw_rad':meta['pose_yaw_rad'],'fresh_at_snapshot':meta['pose_fresh_flag'],'stale_at_current_snapshot':reference_snapshot!=a.snapshot or meta['pose_fresh_flag'] is not True or obs.get('pose',{}).get('fresh') is not True,'cloud_minus_pose_s':cloud['source_stamp_ns'][-1]/1e9-meta['pose_source_stamp_s'],'observation_minus_pose_s':obs['sim_s']-meta['pose_source_stamp_s']}
    if a.marker_from_postvio:
        ref={'snapshot':a.snapshot,'reference_kind':'last_actual_postvio','source_s':float(trace[-1,0]),'xyz_m':trace[-1,1:4].tolist(),'yaw_rad':float(trace[-1,4]),'fresh_at_snapshot':None,'stale_at_current_snapshot':None,'freshness_not_asserted':True,'cloud_minus_pose_s':cloud['source_stamp_ns'][-1]/1e9-trace[-1,0],'observation_minus_pose_s':obs['sim_s']-trace[-1,0]}
    fig,ax=plt.subplots(figsize=(11,12));fig.subplots_adjust(left=.09,right=.84,bottom=.20,top=.88)
    ax.scatter(xyz[floor,0],xyz[floor,1],c='#d2d2d2',s=.35,alpha=.45,rasterized=True)
    sc=ax.scatter(xyz[body,0],xyz[body,1],c=h[body],cmap='viridis',vmin=.15,vmax=1.5,s=.9,alpha=.75,rasterized=True)
    core.draw_trace_2d(ax,trace,(0,1),tm);core.marker(ax,ref,(0,1));ax.set(xlim=(xmin,xmax),ylim=(ymin,ymax),xlabel='estimated map x (m)',ylabel='estimated map y (m)',title='Upper reception hall / body-height measured returns');ax.set_aspect('equal');ax.grid(alpha=.2)
    ax.add_patch(Rectangle((support[0],support[2]),support[1]-support[0],support[3]-support[2],fill=False,edgecolor='#666666',linestyle='--',linewidth=.9))
    annotation_path=Path(b['output_root'])/'gap_annotations.json';annotations=core.read(annotation_path) if annotation_path.exists() else {'annotations':[]}
    if annotations.get('run_id',b['run_id'])!=b['run_id']:raise ValueError('Gap annotation generation mismatch')
    core.draw_gap_annotations(ax,annotations,'upper_observed')
    ax.legend(handles=[Line2D([0],[0],color='#d2d2d2',lw=5,label='measured returns near fitted surface'),Line2D([0],[0],color='#e76a30',lw=2,label='actual upper postVIO samples')],fontsize=8)
    cax=fig.add_axes([.88,.30,.018,.40]);fig.colorbar(sc,cax=cax,label='return z minus fitted measured surface (m)')
    fig.suptitle(f'{b["run_id"]} / {a.snapshot}\nCloud through {cloud["source_stamp_ns"][-1]/1e9:.1f}s; marker {ref["source_s"]:.1f}s',fontsize=13)
    fit_stamp_text=', '.join(f'{v["source_stamp_s"]:.1f}' for v in fit_sources)
    note=(f'Measured-map display reference z={c[0]:.5f}x {c[1]:+.5f}y {c[2]:+.5f}; fit from current-generation exact LIO pairs [{fit_stamp_text}]s, {len(fit):,} inliers. Body band +0.15..+1.50m; gray measured surface band +/-0.08m. First-seen >= {a.first_source:.1f}s filter omits {excluded_early:,} earlier candidate voxels, including possible valid upper returns. Fit-inlier XY bbox x[{support[0]:.1f},{support[1]:.1f}],y[{support[2]:.1f},{support[3]:.1f}]; {outside_support:,} crop points lie outside it. Orange trace uses confirmed upper-stage source times >= {upper_actual_start_s:.1f}s. G codes = observed untraversed / occluded evidence gaps, not blank-pixel unknown labels; sources in gap_annotations.json. Residual scanlines/ceiling overlap may remain, not semantic object labels. Blank = no plotted return after filters, not free/unknown. The map fit is not support or clearance proof. '+core.reference_text(ref))
    fig.text(.065,.035,'\n'.join(textwrap.wrap(note,112)),fontsize=8,va='bottom');dest=out/'upper_observed_bodyheight_2d.png';fig.savefig(dest,dpi=170);plt.close(fig)
    display_trace=trace.copy();display_trace[~tm,0]=np.nan
    # Keep full original sample ordering; NaN stamps suppress excluded segments.
    views=core.render_views(xyz[full],display_trace,ref,b['run_id'],'upper measured relative-height crop',out/'upper_measured_multiview.png')
    record={'run_id':b['run_id'],'snapshot':a.snapshot,'physical_region':'upper_observed','assignment_evidence_refs':['s24/rgb.png','s25/rgb.png','s25/observation.json',f'{a.snapshot}/rgb.png'],'assignment_status':'partially_observed_and_traversed','cloud_source':str(cp),'cloud_source_sha256':core.sha(cp),'cloud_last_s':float(cloud['source_stamp_ns'][-1]/1e9),'surface_fit_latest_pair_s':float(latest['source_stamp_s']),'coefficients_z_equals_ax_plus_by_plus_c':c.tolist(),'fit_inliers':len(fit),'fit_median_abs_residual_m':float(np.median(abs(res))),'fit_p95_abs_residual_m':float(np.quantile(abs(res),.95)),'explicit_display_xy_roi_m':roi.tolist(),'point_first_seen_min_s':a.first_source,'measured_surface_points':int(floor.sum()),'measured_bodyheight_points':int(body.sum()),'measured_crop_points':int(full.sum()),'actual_upper_samples':int(tm.sum()),'pose_marker':ref,'bodyheight_image':str(dest),'multiviews':views,'full_measured_cloud_retained_elsewhere':True,'omitted_earlier_points_are_not_unknown_or_obstacles':True,'not_a_support_clearance_motion_gate_or_completion_claim':True}
    record.update(map_frame=b['map_frame'],actual_upper_stage_start_source_s=upper_actual_start_s,actual_upper_trace_selected_by_source_stage_not_height_band=True,gap_annotations_source=str(annotation_path),display_bounds_source='union of current-generation initial upper fit ROI and reference-snapshot-declared local ROI',surface_fit_latest_pair_npz_sha256=core.sha(latest_path),surface_fit_sources=fit_sources,fit_inlier_xy_bbox_m=support,fit_xy_bbox_is_not_a_complete_surface_support_hull=True,relative_height_candidate_points=int(candidate.sum()),earlier_first_seen_candidate_points_omitted=excluded_early,near_reference_surface_earlier_points_omitted=int((inroi&~post&(abs(h)<=.06)).sum()),crop_points_outside_fit_inlier_xy_bbox=outside_support,not_a_complete_semantic_upper_segmentation=True,point_first_seen_min_ns=round(a.first_source*1e9))
    core.write(out/'upper_observed_record.json',record);print(str(dest))


if __name__=='__main__':main()
