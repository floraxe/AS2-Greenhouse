"""Experimental flat-ground joint-position trot. No root motion or external support."""
import math
import numpy as np
PARAMETERS={'period_s':.9,'swing_fraction':.3,'stride_m':.10,'lift_m':.05,'foot_z_m':-.265,'hip_kp':200.,'thigh_kp':200.,'calf_kp':300.,'hip_kd':2*math.sqrt(5),'thigh_kd':2*math.sqrt(5),'calf_kd':3*math.sqrt(5),'effort_limits_nm':[50,50,75]}
def standing_targets(names):
    return np.array([.9 if 'thigh' in n else -1.8 if 'calf' in n else .1 if n.startswith(('FR','RR')) else -.1 for n in names],dtype=np.float32)
def gait_targets(names,time_s):
    q=standing_targets(names)
    for i,n in enumerate(names):
        phase=(time_s/.9+(0 if n[:2] in ('FL','RR') else .5))%1
        if phase<.3:
            f=phase/.3;x=-.05+.1*(f*f*(3-2*f));z=-.265+.05*math.sin(math.pi*f)
        else:
            f=(phase-.3)/.7;x=.05-.1*f;z=-.265
        L1=.212;L2=.21344;knee=-math.acos(np.clip((x*x+z*z-L1*L1-L2*L2)/(2*L1*L2),-1,1))
        thigh=math.atan2(-x,-z)-math.atan2(L2*math.sin(knee),L1+L2*math.cos(knee))
        if 'thigh' in n:q[i]=thigh
        if 'calf' in n:q[i]=knee
    return q
