"""Stable presentation cameras and actual planar trajectory; not sensor input."""
import math
import numpy as np
from pxr import UsdGeom,Gf

def camera(stage,path,eye,aim,focal=25):
    c=UsdGeom.Camera.Define(stage,path);c.CreateFocalLengthAttr(focal)
    c.CreateHorizontalApertureAttr(24);c.CreateVerticalApertureAttr(13.5)
    c.CreateClippingRangeAttr(Gf.Vec2f(.05,100))
    UsdGeom.Xformable(c).MakeMatrixXform().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*aim),Gf.Vec3d(0,0,1)).GetInverse())
    return path

class Visuals:
    def __init__(self,stage):
        self.stage=stage;self.xy=None;self.mode='overview';self.trail=[];self.visible=True
        self.curve=UsdGeom.BasisCurves.Define(stage,'/World/ActualTrajectory')
        self.curve.CreateTypeAttr('linear');self.curve.CreateWidthsAttr([.035]);self.curve.SetWidthsInterpolation(UsdGeom.Tokens.constant)
        self.curve.CreateDisplayColorAttr([Gf.Vec3f(1.,.36,.035)])
        camera(stage,'/World/ShowOverview',(0,-10,25),(0,.5,0),24)
        camera(stage,'/World/ShowBypass',(-4.6,-5.0,5.9),(-4.3,-2.15,.1),30)
        camera(stage,'/World/ShowWork',(-2.8,-5.0,4.4),(-2.5,-2.15,.18),28)
        camera(stage,'/World/ShowBlock',(.0,-5.1,4.9),(.9,-2.15,.15),27)
        camera(stage,'/World/ShowPause',(3.2,-5.0,4.7),(3.2,-2.15,.15),27)
        camera(stage,'/World/ShowSensor',(4.8,-5.0,4.7),(4.8,-2.15,.15),25)
        self.update([-5.8,-2.15,.3],.2)
    def reset(self):self.trail=[];self.xy=None
    def restore(self,mode):
        if mode=='overview':camera(self.stage,'/World/ShowOverview',(0,-10,25),(0,.5,0),24)
        elif mode=='bypass':camera(self.stage,'/World/ShowBypass',(-4.6,-5.0,5.9),(-4.3,-2.15,.1),30)
    def update(self,pos,dt):
        p=np.asarray(pos[:2],dtype=float)
        if self.xy is None:self.xy=p.copy()
        self.xy+=(1-math.exp(-dt/1.4))*(p-self.xy)
        # Fixed world orientation and fixed height: no body bob or yaw transferred.
        x,y=self.xy;camera(self.stage,'/World/ShowFollow',(x-.3,y-4.0,4.6),(x+.3,y,.15),28)
        if not self.trail or np.linalg.norm(p-np.asarray(self.trail[-1][:2]))>.025:
            self.trail.append((float(p[0]),float(p[1]),.036))
            points=self.trail if len(self.trail)>1 else self.trail*2
            self.curve.CreateCurveVertexCountsAttr([len(points)]);self.curve.CreatePointsAttr(points)
        self.curve.GetVisibilityAttr().Set('inherited' if self.visible else 'invisible')
    def path(self):return dict(overview='/World/ShowOverview',bypass='/World/ShowBypass',follow='/World/ShowFollow',free='/World/FreeCamera')[self.mode]
