"""Independent physical-progress watchdog; never supplies a navigation pose."""
import math

def wrap(v):return (v+math.pi)%(2*math.pi)-math.pi
class MotionProgressMonitor:
 def __init__(self):self.phase=None
 def reset(self):self.phase=None
 def observe(self,t,phase,state,position,yaw,target,heading,row,active,adjustment=None):
  if not active:self.reset();return None
  along=math.cos(heading)*position[0]+math.sin(heading)*position[1]
  if self.phase!=phase:
   self.phase=phase;self.last=t;self.last_along_time=t;self.along=along;self.goal=None;self.best_distance=math.dist(position[:2],target);self.best_heading=abs(wrap(heading-yaw))
  if row:
   if along-self.along>=.02:self.along=along;self.last=t;self.last_along_time=t
   if adjustment is not None:
    gy,gh=adjustment;key=(round(gy,3),round(gh,3))
    lateral=abs(position[1]-gy);angle=abs(wrap(gh-yaw))
    if self.goal!=key:self.goal=key;self.lateral=lateral;self.angle=angle
    if self.lateral-lateral>=.005:self.lateral=lateral;self.last=t
    if self.angle-angle>=.02:self.angle=angle;self.last=t
   if t-self.last_along_time>30.:return 'ROW_REPOSITION_TIMEOUT'
  else:
   distance=math.dist(position[:2],target);angle=abs(wrap(heading-yaw))
   if self.best_distance-distance>=.02:self.best_distance=distance;self.last=t
   if self.best_heading-angle>=.02:self.best_heading=angle;self.last=t
  if t-self.last>8.:return 'SUSTAINED_NO_PROGRESS'
  return None
