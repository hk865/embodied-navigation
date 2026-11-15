#!/usr/bin/env python3
"""Lower-stage display: explicitly time-limited before first ascent."""
from pathlib import Path
import textwrap
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import render_generation as core

base=Path(__file__).parent;b=core.read(base/'generation_binding.json');src=core.bound_path(Path(b['evidence_root'])/'coverage_final',b);out=base/'coverage_final'
cloud=np.load(src/'measured_cloud.npz');xyz=cloud['xyz'];obs=core.read(src/'observation.json');trace=core.load_trace(core.bound_path(Path(b['run_root'])/'capture/interactive_waypoint_control.jsonl',b),b,obs['sim_s'])
cutoff=774.7;lower_trace=trace[trace[:,0]<=cutoff];lo=lower_trace[:,1:3].min(0)-np.array([7.,2.5]);hi=lower_trace[:,1:3].max(0)+np.array([2.,3.]);roi=[float(lo[0]),float(hi[0]),float(lo[1]),float(hi[1])]
zlo,zhi=-.497432,1.5;stage=cloud['voxel_first_source_ns']<=round(cutoff*1e9);mask=stage&(xyz[:,0]>=lo[0])&(xyz[:,0]<=hi[0])&(xyz[:,1]>=lo[1])&(xyz[:,1]<=hi[1])&(xyz[:,2]>=zlo)&(xyz[:,2]<=zhi);pts=xyz[mask]
fig,ax=plt.subplots(figsize=(10,12));fig.subplots_adjust(left=.10,right=.84,bottom=.21,top=.89)
sc=ax.scatter(pts[:,0],pts[:,1],c=pts[:,2],cmap='viridis',s=.45,alpha=.65,rasterized=True);core.draw_trace_2d(ax,lower_trace,(0,1));ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='estimated map x (m)',ylabel='estimated map y (m)',title='Lower-stage observed hall / north corridor / near side room');ax.set_aspect('equal');ax.grid(alpha=.2)
gaps=core.read(base/'gap_annotations.json');core.draw_gap_annotations(ax,gaps,'lower_observed');ax.legend(handles=[Line2D([0],[0],color='#e76a30',lw=2,label='actual lower-stage postVIO'),Line2D([0],[0],color='#8c2020',lw=1,label='G: observed untraversed / occluded')],fontsize=7,loc='center left')
cax=fig.add_axes([.89,.33,.017,.38]);fig.colorbar(sc,cax=cax,label='measured return z (m)')
fig.suptitle(f'{b["run_id"]} / final lower-stage display\nPoints first observed <= {cutoff:.1f}s, before first ascent',fontsize=12)
note=(f'Only voxels first seen <= {cutoff:.1f}s and actual postVIO samples through that source time are shown. Display z=[{zlo:.3f},{zhi:.3f}]m; XY bounds come from this lower-stage actual trajectory plus current-evidence context margins. Upper-stage north-corridor returns and later actual trajectory are excluded by time, not silently treated as lower floor. This conservative subset can omit valid lower surfaces first seen later. No fill: blank = no plotted return after filters, not free/unknown/support. G002 furniture/cabinet backs, G003 closed door backs, G009 measured side recess not traversed; source evidence in gap_annotations.json. Full unfiltered input cloud (5 cm voxel provenance) and complete actual trajectory are preserved separately; not all raw sensor returns.')
fig.text(.06,.035,'\n'.join(textwrap.wrap(note,110)),fontsize=8,va='bottom');dest=out/'lower_observed_region_2d.png';fig.savefig(dest,dpi=170);plt.close(fig)
record={'run_id':b['run_id'],'region_id':'lower_observed','selection_semantics':'before-first-ascent observation-stage display; not a complete semantic floor segmentation','point_first_seen_max_s':cutoff,'point_first_seen_max_ns':round(cutoff*1e9),'actual_trace_selected_by_source_time_not_z':True,'actual_source_interval_s':[float(lower_trace[0,0]),float(lower_trace[-1,0])],'actual_samples':len(lower_trace),'points_displayed':len(pts),'raw_final_cloud_points':len(xyz),'explicit_xy_roi_m':roi,'display_z_band_m':[zlo,zhi],'stage_evidence_refs':['s15/rgb.png','s15/observation.json','s16/rgb.png'],'cutoff_selection':'parent-authorized conservative pre-ascent boundary; navigation samples around this time remain at lower-stage position','gaps_source':str(base/'gap_annotations.json'),'image':str(dest),'valid_later_lower_returns_can_be_omitted':True,'full_raw_and_global_preserved':True,'not_support_clearance_or_completion':True}
core.write(out/'lower_stage_record.json',record)
print(record)
