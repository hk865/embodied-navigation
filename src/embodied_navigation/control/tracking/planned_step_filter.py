"""Explain only planned uphill terrain in the original body-frame obstacle gate.

Static connected 3D PCT geometry and current SLAM pose are the only inputs.
Unknown regions keep the original gate. Planned ground and uphill treads are
not walls; all original distance/count limits and the physical clearance guard remain.
The full point cloud still goes unchanged to native SCAN occupancy checking.
"""
import numpy as np
from .route_progress import ConnectedRouteProgress

class PlannedStepObstacle:
    def __init__(self, body_reference):
        self.progress=ConnectedRouteProgress(body_reference)
        self.points=self.progress.points
        self.ground=self.points-[0.,0.,.32]
        self.ignored=0;self.remaining=0

    def _height(self, query):
        route=self.progress
        first=max(0,route.segment-2);last=route.segment
        while last+1<len(route.length)and route.arc[last+1]<=route.progress+1.3:last+=1
        a=self.ground[first:last+1];delta=self.ground[first+1:last+2]-a
        offset=query[:,None,:2]-a[None,:,:2]
        denom=np.maximum(np.sum(delta[:,:2]**2,axis=1),1e-12)
        u=np.clip(np.sum(offset*delta[None,:,:2],axis=2)/denom,0.,1.)
        projected=a[None,:,:]+u[:,:,None]*delta[None,:,:]
        distance=np.linalg.norm(projected[:,:,:2]-query[:,None,:2],axis=2)
        best=distance.argmin(axis=1);indices=np.arange(len(query))
        return projected[indices,best,2],distance[indices,best]

    def obstacle(self, cloud, position, rotation, advance=True):
        if advance:self.progress.update(position)
        self.ignored=0;self.remaining=0
        local=(cloud-position)@rotation
        mask=(local[:,0]>.10)&(local[:,0]<.55)&(abs(local[:,1])<.27)&(local[:,2]>-.10)&(local[:,2]<.50)
        ids=np.flatnonzero(mask)
        if not len(ids):return False
        self.remaining=len(ids)
        # Large/unknown structures stay blocked; do not grow work with clutter.
        if len(ids)>512:return True
        ground_here,body_distance=self._height(np.asarray(position)[None,:])
        if body_distance[0]>.5 or abs(position[2]-(ground_here[0]+.32))>.65:return len(ids)>=4
        ground_ahead,distance=self._height(cloud[ids])
        rise=ground_ahead-ground_here[0]
        # Subtract only a connected planned rise projected on the same body-up
        # axis. Also recognize the planned support surface within half the .08m
        # grid cell; low posture over a tread is handled by the independent
        # original .15m physical clearance guard, not mistaken for a wall.
        planned_ground=(distance<=.5)&(cloud[ids,2]<=ground_ahead+.04)
        planned_rise=(distance<=.5)&(rise>0)&(local[ids,2]-rise*rotation[2,2]<=-.10)
        explained=planned_ground|planned_rise
        self.ignored=int(explained.sum());self.remaining=int((~explained).sum())
        return self.remaining>=4
