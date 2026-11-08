
from embodied_navigation.project import project_root as _project_root
from pathlib import Path
import json,numpy as np,subprocess,struct,os
R=_project_root();P=R/'build/pct_routes';P.mkdir(parents=True,exist_ok=True);data=np.load(R/'assets/building/building_tomogram_safe.npz');a=data['data'].astype(float);res=float(data['resolution']);center=data['center'];L,H,W=a.shape[1:];offset=np.array([H//2,W//2]);ground=np.nan_to_num(a[3],nan=-100.);cost=np.nan_to_num(a[0],nan=1e6);ceiling=np.nan_to_num(a[4],nan=1e6)
# Keep actual upstream cost/gateway semantics; reject missing support/headroom before search.
cost[~np.isfinite(a[3])|((ceiling-ground)<.7)]=1e6
dt=a[0,1:]-a[0,:-1];dh=abs(ground[1:]-ground[:-1]);gateway=np.zeros_like(ground);gateway[:-1][(dt<-8)&(dh<.1)]=2;gateway[1:][(dt>8)&(dh<.1)]=-2
with(P/'pct_map.bin').open('wb')as f:
 f.write(struct.pack('<iii',L,H,W));f.write(struct.pack('<d',res))
 for m in [cost,ground,gateway]:f.write(np.asarray(m,dtype='<f8').tobytes())
tri=np.load(R/'assets/building/building_triangles.npz')['triangles'];ta=tri[:,0];u=tri[:,1]-ta;v=tri[:,2]-ta;det=u[:,0]*v[:,1]-u[:,1]*v[:,0];keep=abs(det)>1e-10;ta,u,v,det=ta[keep],u[keep],v[keep],det[keep]
def support(xy,expect):
 q=xy-ta[:,:2];s=(q[:,0]*v[:,1]-q[:,1]*v[:,0])/det;t=(u[:,0]*q[:,1]-u[:,1]*q[:,0])/det;z=ta[:,2]+s*u[:,2]+t*v[:,2];ok=(s>=-1e-8)&(t>=-1e-8)&(s+t<=1+1e-8);z=np.r_[z[ok],0.];z=z[abs(z-expect)<.16]
 if len(z)==0:raise ValueError('No physical surface matches PCT ground '+str((xy,expect)))
 return float(z.max())
def node(pos):
 ij=np.rint((np.asarray(pos[:2])-center)/res+offset).astype(int);heights=ground[:,ij[0],ij[1]]-.1;eligible=np.flatnonzero((abs(heights-pos[2])<.22)&(cost[:,ij[0],ij[1]]<=20))
 if not len(eligible):raise ValueError('Endpoint not in traversable height-matched PCT slice: '+str(pos))
 k=min(eligible,key=lambda k:(abs(heights[k]-pos[2]),cost[k,ij[0],ij[1]]));return [int(k),int(ij[1]),int(ij[0])]
def reduce(points,epsilon=.045):
 if len(points)<3:return points
 line=points[-1]-points[0];den=line@line;t=np.clip((points-points[0])@line/den,0,1)if den>0 else np.zeros(len(points));dist=np.linalg.norm(points-(points[0]+t[:,None]*line),axis=1);i=int(dist.argmax())
 if dist[i]<=epsilon:return points[[0,-1]]
 return np.vstack([reduce(points[:i+1],epsilon)[:-1],reduce(points[i:],epsilon)])
cases={
 'stairs_short':([-5.2633744,6.6,0.],[-5.2633744,-4.5,1.47075]),
 'single_floor_turn':([-3.,-4.5,2.98303],[3.,3.,2.98303]),
 'multi_flight_floor1':([-5.2633744,6.6,0.],[-3.,-4.5,2.98303]),
 'combined_floor1_inside':([-5.2633744,6.6,0.],[3.,3.,2.98303]),
 'multi_floor2':([-5.2633744,6.6,0.],[-3.,-4.5,4.43303]),
 'roof_connectivity_probe':([-5.2633744,6.6,0.],[-3.,-4.5,5.98303])}
summary={}
for name,(start,end)in cases.items():
 try:
  st,en=node(start),node(end);query=P/(name+'_query.txt');query.write_text(' '.join(map(str,st+en)));out=P/(name+'_native_path.json');proc=subprocess.run([str(R/'install/pct/pct_astar'),str(P/'pct_map.bin'),str(query),str(out)],capture_output=True,text=True);(P/(name+'_native_search.log')).write_text(proc.stdout+proc.stderr);result=json.loads(out.read_text())
  if not result['found']:summary[name]={'found':False,'start':start,'end':end,'start_node':st,'end_node':en,'partial_path_nodes':len(result['cells_layer_row_col'])};continue
  cells=np.vstack([[st[0],st[2],st[1]],result['cells_layer_row_col']]).astype(int);xy=(cells[:,1:]-offset)*res+center;expected=ground[cells[:,0],cells[:,1],cells[:,2]]-.1;g=np.array([np.r_[p,support(p,e)]for p,e in zip(xy,expected)]);g[0]=start;g[-1]=end
  reduced=reduce(g);uniform=[reduced[0]]
  for p,q in zip(reduced[:-1],reduced[1:]):
   for u0 in np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(q-p)/.6))+1))[1:]:uniform.append(p+(q-p)*u0)
  gref=np.array(uniform);body=gref+[0,0,.32];route={'schema':'native_PCT_Astar_physical_reference/v1','source':'Upstream native Astar::Init/Search/DecideLayer/GetResultMatrix; compatibility fixes: valid zero-cost support + all height-continuous candidate layers (<=.20m); unknowns/headroom masked explicitly; no GTSAM global optimizer (native SCAN used downstream)','start_world_ground':start,'goal_world_ground':end,'cells_layer_row_col':cells.tolist(),'raw_PCT_ground_heights_before_physical_link_offset':(expected+.1).tolist(),'raw_world_ground_path':g.tolist(),'world_ground_points':gref.tolist(),'world_body_points':body.tolist(),'start_node':st,'goal_node':en,'layers_used':sorted(set(cells[:,0].tolist())),'length_m':float(np.linalg.norm(np.diff(gref,axis=0),axis=1).sum()),'coordinate_note':'PCT [slice, X grid row, Y grid column], real elev_g gives raw Z; physical model translation [-2e-6,-1e-6,-.1]; highest physical support hit in matched PCT height band +/- .16m (avoids underside instead of top); never slice*height','postprocess':'Generic3D RDP tolerance45mm then <=.6m spacing; preserves original endpoints, no hand waypoint insertion','execution_success':False};(P/(name+'_route.json')).write_text(json.dumps(route,indent=2));summary[name]={k:route[k]for k in ['start_world_ground','goal_world_ground','length_m','layers_used']};summary[name]['found']=True
 except Exception as e:summary[name]={'found':False,'error':str(e)}
(P/'PCT_route_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
