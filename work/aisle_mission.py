"""Single-aisle mission. Pose input is provided by the selected caller (reference or external SLAM).
Obstacle occupancy NEVER causes an end-of-row transition.
"""
import math
import numpy as np

TURN_CENTER=(5.60,-1.70)
TURN_BAY_BOUNDS=(4.70,6.80,-2.80,-.65)
def wrap(a): return (a+math.pi)%(2*math.pi)-math.pi

class AisleMission:
 def __init__(self,start=(-3.,-2.15)):
  self.start=np.array(start,float); self.exit=np.array((4.95,-2.15)); self.turn=np.array(TURN_CENTER);self.reset()
 def reset(self):
  self.phase='OUT';self.confirm=None;self.done=False;self.events={};self.blocked=False
 def permitted(self,pos):
  x,y=pos[:2]
  xmin,xmax,ymin,ymax=TURN_BAY_BOUNDS
  return (-5.9<x<xmin and abs(y+2.15)<.19) or (xmin<=x<xmax and ymin<y<ymax)
 def step(self,pos,yaw,t,linear_clear,turn_clear):
  zero=(0.,0.,0.,True)
  if not self.permitted(pos):return 'FAULT_AISLE_ID_BOUNDARY',zero
  if self.done:return 'DONE',zero
  # All XY waypoints remain in this aisle or its assigned end bay.
  target={'OUT':self.exit,'ENTER_BAY':self.turn,'EXIT_BAY':self.exit,'RETURN':self.start}.get(self.phase)
  if self.phase=='TURN':
   if not turn_clear:self.confirm=None;return 'WAIT_TURN_SPACE',zero
   error=wrap(math.pi-yaw)
   if abs(error)<.035:
    self.events['turned_time']=t;self.phase='EXIT_BAY';return 'TURN_COMPLETE',zero
   # The experimental gait drifts while turning. Hold the turn centre through
   # joint-driven velocity commands rather than changing the body transform.
   velocity=(self.turn-np.asarray(pos[:2]))*.45
   speed=float(np.linalg.norm(velocity))
   if speed>.035:velocity*=.035/speed
   c,s=math.cos(yaw),math.sin(yaw)
   return 'TURNING',(float(c*velocity[0]+s*velocity[1]),float(-s*velocity[0]+c*velocity[1]),float(np.clip(.8*error,-.2,.2)),False)
  delta=target-np.asarray(pos[:2]);distance=float(np.linalg.norm(delta))
  if distance<.085:
   if self.phase=='OUT':self.phase='ENTER_BAY';return 'ENTER_END_BAY',zero
   if self.phase=='ENTER_BAY':
    # Map position in THIS end bay plus observed free turn space, never short front range alone.
    if not turn_clear:self.confirm=None;return 'WAIT_END_CONFIRMATION',zero
    if self.confirm is None:self.confirm=t
    if t-self.confirm>=.8:self.phase='TURN';self.events['end_confirmed_time']=t
    return 'CONFIRM_END_BAY',zero
   if self.phase=='EXIT_BAY':self.phase='RETURN';return 'REENTER_SAME_AISLE',zero
   self.done=True;self.events['home_time']=t;self.events['home_error_m']=distance;return 'DONE',zero
  if not linear_clear:
   self.blocked=True;return 'WAIT_OBSTACLE_OR_COVERAGE',zero
  heading=0. if self.phase in ('OUT','ENTER_BAY') else math.pi
  # World error -> robot body command. Return is forward walking after a real 180 degree turn.
  velocity=delta*.65;speed=float(np.linalg.norm(velocity))
  if speed>.10:velocity*=.10/speed
  c,s=math.cos(yaw),math.sin(yaw)
  vx=c*velocity[0]+s*velocity[1];vy=-s*velocity[0]+c*velocity[1]
  return self.phase,(float(np.clip(vx,-.05,.11)),float(np.clip(vy,-.06,.06)),float(np.clip(.8*wrap(heading-yaw),-.15,.15)),False)
