#!/usr/bin/env python3
"""Read-only local measured-return inspection, explicitly bound to one snapshot."""
import argparse
import json
import textwrap
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import review_generation as core


def main(argv=None):
    p=argparse.ArgumentParser();p.add_argument('--binding',type=Path,required=True);p.add_argument('--snapshot',required=True)
    p.add_argument('--xy',type=float,nargs=4,required=True);p.add_argument('--label',required=True);args=p.parse_args(argv)
    b=core.read(args.binding);core.validate_snapshot(args.label);src=core.bound_path(Path(b['evidence_root'])/args.snapshot,b);out=core.output_directory(b,args.snapshot)
    cp=core.snapshot_cloud(src,b);a=np.load(cp,allow_pickle=False);m=core.read(src/'scene_view_metadata.json');o=core.read(src/'observation.json')
    assert str(a['run_id'])==b['run_id']==m['run_id'] and str(a['cloud_stage'])==b['cloud_stage'] and str(a['cloud_map_frame'])==b['map_frame']
    xyz=a['xyz'];x0,x1,y0,y1=args.xy
    if not np.isfinite(args.xy).all() or x1<=x0 or y1<=y0:raise ValueError('xy must be finite x_min x_max y_min y_max with increasing bounds')
    mask=np.isfinite(xyz).all(1)&(xyz[:,0]>=x0)&(xyz[:,0]<=x1)&(xyz[:,1]>=y0)&(xyz[:,1]<=y1);pts=xyz[mask]
    if not len(pts):raise ValueError('Explicit ROI has no measured points')
    trace=core.load_trace(core.bound_path(Path(b['run_root'])/'capture/interactive_waypoint_control.jsonl',b),b,o['sim_s'])
    tm=(trace[:,1]>=x0)&(trace[:,1]<=x1)&(trace[:,2]>=y0)&(trace[:,2]<=y1)
    ref={'snapshot':args.snapshot,'source_s':m['pose_source_stamp_s'],'xyz_m':m['pose_origin_xyz_m'],'yaw_rad':m['pose_yaw_rad'],'fresh_at_snapshot':m['pose_fresh_flag'],'stale_at_current_snapshot':o['pose']['fresh'] is not True,'cloud_minus_pose_s':a['source_stamp_ns'][-1]/1e9-m['pose_source_stamp_s'],'observation_minus_pose_s':o['sim_s']-m['pose_source_stamp_s']}
    z0,z1=pts[:,2].min(),pts[:,2].max();pad=max((z1-z0)*.04,.1);plot=core.sample(pts,150000)
    fig=plt.figure(figsize=(15,7));ax=fig.add_subplot(121)
    sc=ax.scatter(plot[:,1],plot[:,2],c=plot[:,2],cmap='viridis',s=.6,alpha=.65,rasterized=True)
    core.draw_trace_2d(ax,trace,(1,2),tm);core.marker(ax,ref,(1,2));ax.set(xlim=(y0,y1),ylim=(z0-pad,z1+pad),xlabel='map y (m)',ylabel='map z (m)',title=f'YZ projection / x in [{x0:g}, {x1:g}] m');ax.grid(alpha=.2)
    ax=fig.add_subplot(122,projection='3d',computed_zorder=False);ax.scatter(*plot.T,c=plot[:,2],cmap='viridis',s=.5,alpha=.5,rasterized=True,zorder=1)
    seg=core.segments(trace,tm)
    if len(seg):ax.add_collection3d(core.Line3DCollection(seg,colors='#e76a30',linewidths=2,zorder=10))
    core.marker(ax,ref);ax.set(xlim=(x0,x1),ylim=(y0,y1),zlim=(z0-pad,z1+pad),xlabel='map x (m)',ylabel='map y (m)',zlabel='map z (m)',title='Measured ROI / oblique');ax.view_init(23,-25);ax.set_box_aspect((x1-x0,y1-y0,z1-z0+2*pad))
    cax=fig.add_axes([.94,.29,.014,.43]);fig.colorbar(sc,cax=cax,label='measured return z (m)')
    fig.suptitle(f'{b["run_id"]} / {args.snapshot} / {args.label}\nLIO measured cloud {a["source_stamp_ns"][0]/1e9:.1f}–{a["source_stamp_ns"][-1]/1e9:.1f} s; postVIO marker {ref["source_s"]:.1f} s',fontsize=13)
    note=core.reference_text(ref)+' XY ROI is an inspection crop, not a target or route. All measured ROI z retained. Black x = go2_imu origin, 1 m horizontal yaw arrow; orange = actual postVIO within ROI. Blank pixels are no plotted return, not free/unknown labels. Low return surfaces do not certify robot support or clearance.'
    fig.text(.045,.025,'\n'.join(textwrap.wrap(note,170)),fontsize=8,va='bottom');fig.subplots_adjust(bottom=.20,top=.86,right=.90,wspace=.15)
    dest=out/(args.label+'_yz_oblique.png');fig.savefig(dest,dpi=170);plt.close(fig)
    record={'run_id':b['run_id'],'snapshot':args.snapshot,'source_cloud':str(cp),'source_cloud_sha256':core.sha(cp),'cloud_source_first_s':float(a['source_stamp_ns'][0]/1e9),'cloud_source_last_s':float(a['source_stamp_ns'][-1]/1e9),'pose_marker':ref,'explicit_xy_roi_m':args.xy,'measured_z_range_m':[float(z0),float(z1)],'points_in_roi':len(pts),'points_plotted':len(plot),'actual_postvio_samples_in_roi':int(tm.sum()),'not_a_navigation_target':True,'no_support_or_clearance_claim':True,'image':str(dest)}
    core.write(out/(args.label+'_record.json'),record);print(json.dumps(record,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
