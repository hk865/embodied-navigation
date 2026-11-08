def export_bounds(stage,out):
 from pxr import Usd,UsdGeom,UsdPhysics,Gf
 import numpy as np,json
 bounds=[];unsupported=[];planes=[]
 for p in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies()):
  path=str(p.GetPath())
  if path.startswith('/World/Go2') or not p.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is False:continue
  mat=UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)
  if p.IsA(UsdGeom.Plane):planes.append({'path':path,'axis':str(p.GetAttribute('axis').Get()),'local_to_world':list(map(list,mat))});continue
  if p.IsA(UsdGeom.Mesh):v=np.array(UsdGeom.Mesh(p).GetPointsAttr().Get(),float)
  elif p.IsA(UsdGeom.Boundable):
   ex=UsdGeom.Boundable(p).GetExtentAttr().Get()
   if ex is None:unsupported.append(path);continue
   lo,hi=np.array(ex,float);v=np.array([[x,y,z] for x in [lo[0],hi[0]] for y in [lo[1],hi[1]] for z in [lo[2],hi[2]]])
  else:unsupported.append(path);continue
  if not len(v):unsupported.append(path);continue
  # Convex hull/decomposition cooking remains within the full authored point bound.
  w=np.array([mat.Transform(Gf.Vec3d(*q)) for q in v]);lo=w.min(0);hi=w.max(0);bounds.append({'path':path,'world_bbox':[lo.tolist(),hi.tolist()]})
 (out/'complete_collision_bounds.json').write_text(json.dumps({'source':'all enabled non-robot collider prims including instance proxies; all authored mesh points independent of face triangulation, visibility or purpose','bounds':bounds,'planes':planes,'unsupported':unsupported,'certificate_not_robot_localization':True},indent=2))
