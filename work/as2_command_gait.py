"""Command-driven experimental foot trajectory. Only 12 joint targets are actuated."""
import math
import numpy as np
from as2_leg_controller import standing_targets
class CommandGait:
    def __init__(self,names):
        self.names=names;self.phase=0.;self.vx=0.;self.vy=0.;self.yaw_rate=0.
    def reset(self):self.phase=0.;self.vx=0.;self.vy=0.;self.yaw_rate=0.
    def step(self,dt,vx,yaw_rate,stop=False,vy=0.):
        if stop:self.vx=0.;self.vy=0.;self.yaw_rate=0.;return standing_targets(self.names)
        vy=float(np.clip(vy,-.08,.08));vx=float(np.clip(vx,-.12,.16));yaw_rate=float(np.clip(yaw_rate,-.35,.35))
        self.vx+=float(np.clip(vx-self.vx,-.18*dt,.18*dt));self.yaw_rate+=float(np.clip(yaw_rate-self.yaw_rate,-.5*dt,.5*dt))
        self.vy+=float(np.clip(vy-self.vy,-.18*dt,.18*dt))
        if abs(self.vy)<.001 and abs(self.vx)<.001 and abs(self.yaw_rate)<.003:return standing_targets(self.names)
        self.phase+=dt
        q=standing_targets(self.names)
        for leg in ['FL','FR','RL','RR']:
            side=1 if leg[1]=='L' else -1;hip_x=.2119 if leg[0]=='F' else -.2119
            a=side*.1054;h=-side*.1;z0=-.265
            nominal_y=a*math.cos(h)-z0*math.sin(h);nominal_z=a*math.sin(h)+z0*math.cos(h)
            phase=(self.phase/.9+(0 if leg in ('FL','RR') else .5))%1
            if phase<.3:
                f=phase/.3;shape=-.5+f*f*(3-2*f);lift=.05*math.sin(math.pi*f)
            else:shape=.5-(phase-.3)/.7;lift=0.
            stride_x=(self.vx-self.yaw_rate*(side*.06+nominal_y))*(.1/.12)
            stride_y=(self.vy+self.yaw_rate*hip_x)*(.1/.12)
            x=shape*stride_x;y=nominal_y+shape*stride_y;z=nominal_z+lift
            sagittal_z=-math.sqrt(max(.01,y*y+z*z-a*a))
            hip=math.atan2(z,y)-math.atan2(sagittal_z,a);hip=(hip+math.pi)%(2*math.pi)-math.pi
            L1=.212;L2=.21344
            knee=-math.acos(np.clip((x*x+sagittal_z*sagittal_z-L1*L1-L2*L2)/(2*L1*L2),-1,1))
            thigh=math.atan2(-x,-sagittal_z)-math.atan2(L2*math.sin(knee),L1+L2*math.cos(knee))
            for kind,value,low,high in [('hip',hip,-.785,.785),('thigh',thigh,-2,3.492),('calf',knee,-2.7677,-.8133)]:
                q[self.names.index(leg+'_'+kind+'_joint')]=np.clip(value,low,high)
        return q
