"""Measured floor stabilization, no simulator/reference pose input.
Flat local ground is an explicit assumption; valid plane must come from lidar.
The virtual origin is the ground projection of base_link, X projected forward.
"""
import math
import numpy as np

def projected_heading_rotation(R):
    yaw=math.atan2(R[1,0],R[0,0]);c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,s,0.],[-s,c,0.],[0.,0.,1.]])@R

def floor_transform(plane):
    a,b,c=np.asarray(plane,dtype=float)
    n=np.array([-a,-b,1.]);norm=np.linalg.norm(n);n/=norm
    if not np.isfinite(n).all() or np.linalg.norm([a,b])>.35 or not .18<-c/norm<.5:
        raise ValueError('Unreliable measured ground plane')
    x=np.array([1.,0.,0.])-n[0]*n;x/=np.linalg.norm(x)
    y=np.cross(n,x);R=np.stack([x,y,n]);T=np.eye(4);T[:3,:3]=R;T[2,3]=-c/norm
    return T

class FloorStabilizer:
    def __init__(self):self.T=None;self.time=None;self.floor_stamp=None
    def observe(self,plane,stamp):
        self.T=floor_transform(plane);self.time=float(stamp);self.floor_stamp=float(stamp)
        return self.T.copy()
    def gyro(self,w,stamp):
        if self.T is None:return None
        dt=float(stamp)-self.time
        if not 0<=dt<=.05 or stamp-self.floor_stamp>.4:return None
        w=np.asarray(w,dtype=float);angle=np.linalg.norm(w)*dt
        if not np.isfinite(w).all() or np.linalg.norm(w)>2.:return None
        if angle>1e-12:
            v=w/np.linalg.norm(w);K=np.array([[0.,-v[2],v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]])
            self.T[:3,:3]=projected_heading_rotation(self.T[:3,:3]@(np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*K@K))
        self.time=float(stamp)
        return self.T.copy()
    def points(self,lidar):
        if self.T is None:raise ValueError('No measured floor')
        b=np.asarray(lidar,dtype=float)+[.3,0.,.32]
        return np.ascontiguousarray(b@self.T[:3,:3].T+self.T[:3,3],dtype='<f4')
