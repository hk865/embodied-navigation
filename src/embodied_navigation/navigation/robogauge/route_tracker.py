"""Pure bounded native-pose route controller; no simulator or policy side effects."""
import math
wrap=lambda a:math.atan2(math.sin(a),math.cos(a))
clip=lambda x,b:max(-b,min(b,x))
class RouteTracker:
 def __init__(self,points,start_xy,speed=.15,motion_end=230.,stop_on_stall=True):
  self.points=points;self.start=start_xy;self.speed=speed;self.motion_end=motion_end;self.stage=0;self.phase='waiting';self.failure=None;self.arrived=False;self.events=[];self.since=None;self.phase_start=None;self.stall=None;self.stop_on_stall=stop_on_stall
 def invalidate_measurement(self):
  self.since=None;self.stall=None
 def tick(self,t,p,yaw,speed,omega,ready=True):
  command=[0.,0.,0.]
  if self.failure or self.arrived:return command
  if t>=self.motion_end:self.failure='route_motion_budget';return command
  if not ready or t<7:return command
  goal=self.points[self.stage];origin=self.start if self.stage==0 else self.points[self.stage-1][:2]
  dx=goal[0]-origin[0];dy=goal[1]-origin[1];length=math.hypot(dx,dy);u=(dx/length,dy/length);normal=(-u[1],u[0]);heading=math.atan2(dy,dx)
  error=wrap(heading-yaw);remaining=(goal[0]-p[0])*u[0]+(goal[1]-p[1])*u[1];cross=(p[0]-origin[0])*normal[0]+(p[1]-origin[1])*normal[1];distance=math.hypot(goal[0]-p[0],goal[1]-p[1]);z_error=abs(p[2]-goal[2])
  if self.phase=='waiting':self.phase='cruise';self.phase_start=t
  if self.phase=='cruise':
   if abs(cross)>.6:self.failure='route_cross_track_bound';return command
   if remaining<-.35:self.failure='route_waypoint_overshoot';return command
   if remaining<=.20 and distance<=.30 and z_error<=.18:
    self.phase='brake';self.phase_start=t;self.since=None;self.stall=None;self.events.append({'event':'entered_corner_capture','stage':self.stage,'time':t,'position':list(p),'distance':distance,'z_error':z_error})
   else:
    command[0]=min(self.speed,max(.04,.5*max(remaining,0.))) if abs(error)<.25 else 0.
    correction=-.5*cross;world=(correction*normal[0],correction*normal[1]);command[1]=clip(-math.sin(yaw)*world[0]+math.cos(yaw)*world[1],.05);command[2]=clip(.8*error,.15)
    if command[0]>0:
     along=(p[0]-origin[0])*u[0]+(p[1]-origin[1])*u[1]
     if self.stall is None:self.stall=(t,along)
     elif t-self.stall[0]>=8.:
      if along-self.stall[1]<.02:
       self.events.append({'event':'observed_low_progress_stall','stage':self.stage,'time':t,'window_start_s':self.stall[0],'forward_progress_m':along-self.stall[1],'position':list(p),'request':command.copy(),'stop_on_stall':self.stop_on_stall})
       if self.stop_on_stall:self.failure='route_low_progress_stall'
      self.stall=(t,along)
    else:self.stall=None
    return command
  if self.phase=='brake':
   if distance>.40:self.failure='corner_braking_drift';return command
   command[2]=clip(.8*error,.15)
   if speed<.04 and z_error<=.18:
    if self.since is None:self.since=t
    if t-self.since>=.5:
     self.events.append({'event':'corner_stopped','stage':self.stage,'time':t,'position':list(p),'distance':distance,'z_error':z_error})
     if self.stage==len(self.points)-1:self.arrived=True;self.phase='arrived';return [0.,0.,0.]
     self.phase='turn';self.phase_start=t;self.since=None
   else:self.since=None
   if t-self.phase_start>12.:self.failure='corner_brake_timeout'
   return command
  if self.phase=='turn':
   nextgoal=self.points[self.stage+1];nextheading=math.atan2(nextgoal[1]-goal[1],nextgoal[0]-goal[0]);next_error=wrap(nextheading-yaw)
   if distance>.45:self.failure='corner_turn_drift';return command
   command[2]=clip(.8*next_error,.15)
   if abs(next_error)<.10 and abs(omega)<.06 and speed<.04:
    if self.since is None:self.since=t
    if t-self.since>=.5:
     self.events.append({'event':'turn_completed','stage':self.stage,'time':t,'position':list(p),'heading_error':next_error});self.stage+=1;self.phase='cruise';self.phase_start=t;self.since=None;self.stall=None
   else:self.since=None
   if t-self.phase_start>40.:self.failure='corner_turn_timeout'
   return command
  return command
