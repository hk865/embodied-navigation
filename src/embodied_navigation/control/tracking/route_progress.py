"""Monotone progress on connected 3D segments, including folded/closed routes."""
import numpy as np

class ConnectedRouteProgress:
    def __init__(self, points, maximum_distance=.65, forward_arc=1.3):
        self.points=np.asarray(points,dtype=float)
        if self.points.ndim!=2 or self.points.shape[1]!=3 or len(self.points)<2 or not np.isfinite(self.points).all():
            raise ValueError('finite 3D route required')
        self.delta=np.diff(self.points,axis=0)
        self.length=np.linalg.norm(self.delta,axis=1)
        if np.any(self.length<1e-6):raise ValueError('coalesce duplicate route points first')
        self.arc=np.r_[0.,np.cumsum(self.length)]
        self.distance=float(maximum_distance);self.forward_arc=float(forward_arc)
        self.progress=0.;self.segment=0

    def update(self, position):
        position=np.asarray(position,dtype=float)
        # A smoothed SCAN turn may pass within the existing .5m route fence
        # without visiting the exact polyline vertex. Witness only consecutive
        # nearby vertices, in 3D and within the same bounded connected arc.
        limit=self.progress+self.forward_arc
        while self.segment+1<len(self.points)-1 and self.arc[self.segment+1]<=limit and np.linalg.norm(position-self.points[self.segment+1])<=.5 and np.dot(self.delta[self.segment],self.delta[self.segment+1])<.95*self.length[self.segment]*self.length[self.segment+1]:
            self.progress=max(self.progress,float(self.arc[self.segment+1]));self.segment+=1
        best_distance=float('inf');best_progress=self.progress
        # Search a connected local prefix only. A later revisit of the same XYZ
        # or the same XY on another floor is never a global-nearest shortcut.
        for i in range(self.segment,len(self.length)):
            if self.arc[i]>self.progress+self.forward_arc:break
            u=float(np.clip(np.dot(position-self.points[i],self.delta[i])/self.length[i]**2,0.,1.))
            progress=min(self.arc[i]+u*self.length[i],self.progress+self.forward_arc)
            projected=self.points[i]+(progress-self.arc[i])/self.length[i]*self.delta[i]
            distance=float(np.linalg.norm(position-projected))
            if distance<best_distance:
                best_distance=distance;best_progress=progress
            elif distance>best_distance+.05:break
        if best_distance<=self.distance:
            self.progress=max(self.progress,best_progress)
            self.segment=min(len(self.length)-1,int(np.searchsorted(self.arc,self.progress,side='right')-1))
        return self.progress

    @property
    def remaining(self):return float(self.arc[-1]-self.progress)
