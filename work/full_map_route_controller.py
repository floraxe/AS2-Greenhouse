"""Surveyed aisle 2 -> far headland -> aisle 3 -> entrance -> start.
No search planner or simulator pose access. Controller receives estimated pose.
"""
import math
from aisle_mission import wrap

AISLE2_Y=-.825
AISLE3_Y=.400
START23=(-3.,AISLE2_Y)

class FixedRouteMission:
    def __init__(self):
        self.start=START23
        self.stages=[
            ('AISLE2_OUT',(5.60,AISLE2_Y),0.),
            ('FAR_TURN_LEFT',(5.60,AISLE2_Y),math.pi/2),
            ('FAR_TRANSFER',(5.60,AISLE3_Y),math.pi/2),
            ('FAR_TURN_RETURN',(5.60,AISLE3_Y),math.pi),
            ('AISLE3_RETURN',(-5.60,AISLE3_Y),math.pi),
            ('ENTRY_TURN_RIGHT',(-5.60,AISLE3_Y),-math.pi/2),
            ('ENTRY_TRANSFER',(-5.60,AISLE2_Y),-math.pi/2),
            ('ENTRY_TURN_HOME',(-5.60,AISLE2_Y),0.),
            ('CLOSE_START',START23,0.)]
        self.reset()

    def reset(self):
        self.index=0; self.done=False; self.events={}; self.confirm=None
        self.phase=self.stages[0][0]
        self.plan_status=dict(valid=False,reason='FIXED_ROUTE_WAIT_START',path=[])

    def permitted(self,pos):
        x,y=pos[:2]
        if not all(math.isfinite(v) for v in (x,y)):return False
        # Explicit task envelope, not an assertion that the body fits a gap.
        return ((-4.70<x<4.70 and min(abs(y-AISLE2_Y),abs(y-AISLE3_Y))<.15)
                or (4.70<=x<6.80 and -1.60<y<1.20)
                or (-6.80<x<=-4.70 and -1.60<y<1.20))

    def step(self,pos,yaw,t,linear_clear,turn_clear,row_observation=None):
        zero=(0.,0.,0.,True)
        if not self.permitted(pos):return 'FAULT_ROUTE23_BOUNDARY',zero
        if self.done:return 'DONE',zero
        name,target,heading=self.stages[self.index];self.phase=name
        dx,dy=target[0]-pos[0],target[1]-pos[1]
        distance=math.hypot(dx,dy);angle=wrap(heading-yaw)
        turning='TURN' in name and name!='AISLE3_RETURN'
        self.plan_status=dict(valid=True,reason='FIXED_CENTERLINE_NO_ASTAR',path=[list(pos[:2]),list(target)])
        if turning:
            if not turn_clear:
                self.confirm=None
                return 'WAIT_TURN_SPACE',zero
            arrived=distance<.06 and abs(angle)<.025
        else:
            arrived=distance<.05 and abs(angle)<.06
        if name=='CLOSE_START':arrived=distance<.012 and abs(angle)<.025
        if name=='CLOSE_START' and row_observation and row_observation.get('valid'):
            arrived=abs(dx)<.012 and abs(row_observation['center_error_body_y_m'])<.015 and abs(row_observation['heading_error_rad'])<.025 and abs(angle)<.12
        if arrived:
            if self.confirm is None:self.confirm=t
            if t-self.confirm<.5:return 'CONFIRM_'+name,zero
            self.events[name.lower()+'_time']=t;self.index+=1;self.confirm=None
            if self.index==len(self.stages):
                self.done=True;self.events['same_heading_home_time']=t
                return 'DONE',zero
            self.phase=self.stages[self.index][0]
            return 'NEXT_ROUTE_STAGE',zero
        self.confirm=None
        if not turning and not linear_clear:return 'WAIT_OBSTACLE_OR_COVERAGE',zero
        c,s=math.cos(heading),math.sin(heading)
        along=c*dx+s*dy;cross=-s*dx+c*dy
        if name=='CLOSE_START' and along<=-.08:
            return 'WAIT_HOME_OVERSHOOT_REVIEW',zero
        # Stronger lateral correction, slower forward speed in narrow aisles.
        speed=self.tracking_speed(pos) if not turning else .025
        longitudinal=max(-.025,min(speed,.65*along))
        if name=='CLOSE_START' and abs(along)>.012:
            # Measured joint-driven model has a low-speed propulsion dead zone.
            # Stop inside the tight goal tolerance; use a bounded drive pulse
            # outside it, with the normal raw swept-body safety check retained.
            longitudinal=math.copysign(max(.055,abs(longitudinal)),along)
        if not turning:longitudinal*=max(0.,math.cos(angle))*max(.25,1.-abs(cross)/.15)
        lateral=max(-.055,min(.055,1.2*cross))
        wx=c*longitudinal-s*lateral;wy=s*longitudinal+c*lateral
        if turning:
            wx=.7*dx;wy=.7*dy
            norm=math.hypot(wx,wy)
            if norm>.035:wx*=.035/norm;wy*=.035/norm
        cy,sy=math.cos(yaw),math.sin(yaw)
        omega=max(-(.20 if turning else .12),min(.20 if turning else .12,.9*angle))
        return ('ROUTE_TURNING' if turning else 'FIXED_'+name),(cy*wx+sy*wy,-sy*wx+cy*wy,omega,False)
