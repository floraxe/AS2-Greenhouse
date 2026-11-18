"""Predefined whole-map mission. Truth localization, mocked arm handshake."""
import math
import numpy as np

START=(-5.8,-2.15)
AISLES=[-2.15,-.15,1.85,3.8,5.75]

def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi

def route():
    goals=[]
    for row,y in enumerate(AISLES):
        sign=1 if row%2==0 else -1
        if row:goals.append(dict(x=-sign*5.8,y=y,job=None,aisle=row+1))
        for x in [-sign*2.5,sign*2.5]:
            goals.append(dict(x=x,y=y,job=sum(g['job'] is not None for g in goals)+1,aisle=row+1))
        goals.append(dict(x=sign*5.8,y=y,job=None,aisle=row+1))
    goals.extend([dict(x=5.8,y=-4.5,job=None,aisle=0),dict(x=-5.8,y=-4.5,job=None,aisle=0),dict(x=START[0],y=START[1],job=None,aisle=0)])
    return goals

class Mission:
    def __init__(self,bypass=False):
        self.goals=route();self.index=0;self.state='ALIGN';self.phase='ALIGN'
        self.finished=False;self.jobs=[];self.events=[];self.heading=0.;self.anchor=None
        self.timer=0.;self.stable=0.;self.work=0.;self.timeout=0.;self.bypass=None
        self.enable_bypass=bypass;self.bypasses=0;self.visited=set();self.job_error=None
        self.sensor_wait=0.;self.obstacle_wait=0.;self.failed=False

    def event(self,t,name,**extra):self.events.append(dict(t=round(t,3),event=name,**extra))
    def advance(self,pos,t):
        self.visited.add(self.goals[self.index]['aisle']);self.index+=1
        self.phase='ALIGN';self.anchor=None;self.timer=0.;self.stable=0.;self.timeout=0.
        if self.index==len(self.goals):self.finished=True;self.state='DONE';self.event(t,'MISSION_COMPLETE')

    def command(self,pos,yaw,velocity,guard,dt,t,turn_clear=True,posture_ok=True):
        zero=(0.,0.,0.,True)
        if self.finished:self.state='DONE';return zero
        if self.failed:self.state='FAULT';return zero
        g=self.goals[self.index];target=np.array([g['x'],g['y']]);xy=np.asarray(pos[:2])
        c,s=math.cos(yaw),math.sin(yaw);R=np.array([[c,-s],[s,c]])
        if guard['state']=='UNKNOWN':
            if self.state!='WAIT_SENSOR':self.event(t,'WAIT_SENSOR')
            self.sensor_wait+=dt;self.state='WAIT_SENSOR';return zero
        if self.phase=='ALIGN':
            if self.anchor is None:
                self.heading=math.atan2(target[1]-xy[1],target[0]-xy[0]);self.anchor=xy+R@np.array([-.052,0.])
            angle=wrap(self.heading-yaw)
            if abs(angle)<.018:self.phase='DRIVE';self.anchor=None
            else:
                if not turn_clear:self.state='WAIT_TURN_CLEARANCE';return zero
                err=R.T@(self.anchor-R@np.array([-.052,0.])-xy)
                self.state='TURN';return float(np.clip(.7*err[0],-.055,.055)),float(np.clip(.7*err[1],-.055,.055)),float(np.clip(.8*angle,-.22,.22)),False
        if self.phase=='DRIVE':
            delta=target-xy;angle=wrap(self.heading-yaw)
            if np.linalg.norm(delta)<.065 and abs(angle)<.05:
                self.event(t,'ARRIVED',goal=self.index,job=g['job'],position_error_m=float(np.linalg.norm(delta)),yaw_error_deg=math.degrees(abs(angle)))
                if g['job'] is None:self.advance(pos,t);return zero
                self.phase='SETTLE';self.timer=0.;self.stable=0.;self.timeout=0.
            else:
                if self.enable_bypass and self.index==0:
                    if self.bypass is None:
                        from lidar_simple_bypass import SimpleBypass
                        self.bypass=SimpleBypass(START[1],g['x'])
                    vx,vy,omega,halt=self.bypass.command(pos,yaw,guard)
                    self.bypasses=self.bypass.completed_bypasses;self.state=self.bypass.state
                    return vx,vy,omega,halt
                if guard['state']=='STOP':
                    if self.state!='WAIT_OBSTACLE':self.event(t,'WAIT_OBSTACLE',goal=self.index)
                    self.obstacle_wait+=dt;self.state='WAIT_OBSTACLE';return zero
                if self.state=='WAIT_OBSTACLE':self.event(t,'OBSTACLE_CLEAR_RESUME',goal=self.index)
                err=R.T@delta;self.state='CRUISE'
                return float(np.clip(.7*err[0],-.04,.14)),float(np.clip(.7*err[1],-.06,.06)),float(np.clip(.9*angle,-.2,.2)),False
        self.timeout+=dt
        if not posture_ok:
            self.stable=0.;self.state='WAIT_POSTURE'
            if self.timeout>20:self.failed=True;self.event(t,'JOB_POSTURE_TIMEOUT',job=g['job'])
            return zero
        if self.phase=='SETTLE':
            self.state='SETTLING_AT_JOB'
            stable=np.linalg.norm(velocity[:2])<.035 and np.linalg.norm(target-xy)<.10 and abs(wrap(self.heading-yaw))<.06
            self.stable=self.stable+dt if stable else 0.
            if self.stable>=.8:
                self.phase='WORK';self.work=0.;self.event(t,'ARM_START_MOCK',job=g['job'])
            if self.timeout>20:self.failed=True;self.event(t,'JOB_SETTLE_TIMEOUT',job=g['job'])
        elif self.phase=='WORK':
            self.state='WORKING_MOCK';self.work+=dt
            if np.linalg.norm(target-xy)>.14:
                self.failed=True;self.event(t,'JOB_POSITION_LOST',job=g['job'])
            elif self.work>=4.:
                result=dict(job=g['job'],aisle=g['aisle'],position_error_m=float(np.linalg.norm(target-xy)),yaw_error_deg=math.degrees(abs(wrap(self.heading-yaw))),work_time_s=self.work,feedback='mock_complete')
                self.jobs.append(result);self.event(t,'ARM_DONE_MOCK',**result);self.advance(pos,t)
        return zero
