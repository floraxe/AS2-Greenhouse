"""Static-scene, point-cloud checked shift/pass/return prototype.

No obstacle IDs or object positions are supplied to the planner. Uses known route
center/width, flat ground and synthetic odometry; not full dynamic navigation.
"""
import math
import numpy as np


class SimpleBypass:
    def __init__(self,route_y=-2.15,end_x=3.5):
        self.route_y=route_y;self.end_x=end_x;self.state='CRUISE'
        self.waypoints=[];self.memory={};self.map=np.empty((0,3));self.ground=np.empty((0,3))
        self.completed_bypasses=0;self.plans=0;self.last_scan=-1;self.last_path=[]

    def observe(self,points,scan):
        if scan==self.last_scan:return
        self.last_scan=scan
        p=np.asarray(points).reshape(-1,3)
        self.ground=p[p[:,2]<.025]
        for xyz in p[(p[:,2]>=.025)&(p[:,2]<1.5)]:
            self.memory[tuple(np.round(xyz/.035).astype(int))]=xyz
        self.map=np.array(list(self.memory.values())).reshape(-1,3)

    def clear_segment(self,a,b,z,coverage=False):
        p=self.map
        n=max(2,int(np.linalg.norm(np.asarray(b)-np.asarray(a))/.08)+1)
        for t in np.linspace(0,1,n):
            x,y=(1-t)*np.asarray(a)+t*np.asarray(b)
            for lo,hi,w,front in [(-.32,.18,.30,.36),(.13,.50,.12,.08)]:
                hit=(p[:,2]>=z+lo)&(p[:,2]<=z+hi)&(p[:,0]>=x-.48)&(p[:,0]<=x+front)&(np.abs(p[:,1]-y)<=w)
                if np.any(hit):return False
            if coverage:
                # Front-side ground support only; the scene is assumed flat.
                if np.count_nonzero(np.linalg.norm(self.ground[:,:2]-[x+.45,y],axis=1)<.32)<2:return False
        return True

    def plan(self,pos):
        p=self.map
        relevant=(p[:,0]>pos[0]+.35)&(p[:,0]<pos[0]+2.8)&(np.abs(p[:,1]-self.route_y)<.30)&(p[:,2]<pos[2]+.50)
        obstacle=p[relevant]
        if len(obstacle)<2:return False
        end=float(np.max(obstacle[:,0])+.95)
        if end>self.end_x-.4:return False
        signs=[-1,1] if np.mean(obstacle[:,1])>=self.route_y else [1,-1]
        for sign in signs:
            side=self.route_y+sign*.30
            path=[np.asarray(pos[:2]),np.array([pos[0],side]),np.array([end,side]),np.array([end,self.route_y])]
            if all(self.clear_segment(a,b,pos[2],coverage=(i==0)) for i,(a,b) in enumerate(zip(path,path[1:]))):
                self.waypoints=[(path[1],'BYPASS_SHIFT'),(path[2],'BYPASS_PASS'),(path[3],'BYPASS_RETURN')]
                self.last_path=[x.tolist() for x in path];self.plans+=1
                return True
        return False

    def command(self,pos,yaw,guard):
        if guard['state']=='UNKNOWN':self.state='WAIT_SENSOR';return 0.,0.,0.,True
        if not self.waypoints and guard.get('clearance_m') is not None and guard['clearance_m']<=1.10:
            if not self.plan(pos):self.state='WAIT_NO_SPACE';return 0.,0.,0.,True
        if self.waypoints:
            target,phase=self.waypoints[0]
            if np.linalg.norm(target-np.asarray(pos[:2]))<.045:
                self.waypoints.pop(0)
                if not self.waypoints:
                    self.completed_bypasses+=1;self.state='CRUISE'
                    return 0.,0.,0.,True
                target,phase=self.waypoints[0]
            if not self.clear_segment(pos[:2],target,pos[2]):
                self.state='WAIT_PATH_BLOCKED';return 0.,0.,0.,True
            self.state=phase
            delta=target-np.asarray(pos[:2])
            wx=float(np.clip(.65*delta[0],-.035,.09));wy=float(np.clip(.7*delta[1],-.055,.055))
        else:
            if guard['state']=='STOP':self.state='WAIT_NO_SPACE';return 0.,0.,0.,True
            self.state='CRUISE'
            wx=min(.10,max(.025,.7*(self.end_x-float(pos[0]))))
            wy=float(np.clip(.7*(self.route_y-pos[1]),-.055,.055))
        c,s=math.cos(yaw),math.sin(yaw)
        return c*wx+s*wy,-s*wx+c*wy,float(np.clip(-.8*yaw,-.18,.18)),False
