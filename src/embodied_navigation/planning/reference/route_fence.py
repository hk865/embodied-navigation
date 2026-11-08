"""Finite 3D tube around the frozen sensor-registered scene prior."""
import math
def tube_evidence(position,points,horizontal_radius=.45,height_error=.3):
    if len(position)!=3 or len(points)<2 or not all(math.isfinite(x)for p in [position,*points]for x in p):
        raise ValueError('Finite registered map corridor required')
    candidates=[]
    for i,(a,b)in enumerate(zip(points[:-1],points[1:])):
        d=[b[j]-a[j]for j in range(3)];n=sum(x*x for x in d)
        if n<1e-12:continue
        u=max(0.,min(1.,sum((position[j]-a[j])*d[j]for j in range(3))/n))
        nearest=[a[j]+u*d[j]for j in range(3)]
        horizontal=math.hypot(position[0]-nearest[0],position[1]-nearest[1])
        vertical=abs(position[2]-nearest[2])
        candidates.append((sum((position[j]-nearest[j])**2 for j in range(3)),i,u,nearest,horizontal,vertical))
    if not candidates:raise ValueError('Nondegenerate registered corridor required')
    _,i,u,point,h,z=min(candidates)
    return dict(inside=h<=horizontal_radius and z<=height_error,segment=i,fraction=u,
        nearest_camera_init_xyz=point,horizontal_error_m=h,height_error_m=z,
        horizontal_radius_m=horizontal_radius,maximum_height_error_m=height_error,
        navigation_ground_truth_used=False)
