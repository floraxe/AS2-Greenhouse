"""Two measured near-parallel trellis lines define initial row heading.
No world pose input. This is an initial surveyed-row prior, not general SLAM.
"""
import math
import numpy as np
def line(points,sign):
    # Long nearly horizontal side structures, such as the two trellis rails.
    mask=(points[:,0]>-1.5)&(points[:,0]<1.5)&(points[:,1]*sign>.3)&(points[:,1]*sign<.85)&(points[:,2]>-.1)&(points[:,2]<1.3)
    xy=points[mask,:2];rng=np.random.default_rng(42);best=None
    if len(xy)<80:return dict(valid=False,reason='INSUFFICIENT_POINTS',points=len(xy))
    for _ in range(250):
        a,b=xy[rng.choice(len(xy),2,replace=False)]
        if abs(a[0]-b[0])<.8:continue
        slope=(b[1]-a[1])/(b[0]-a[0])
        if abs(slope)>.2:continue
        offset=a[1]-slope*a[0]
        keep=abs(xy[:,1]-slope*xy[:,0]-offset)<.012
        bins=len(np.unique(np.floor(xy[keep,0]/.15)))
        if bins<8:continue
        score=int(keep.sum())
        if best is None or score>best[0]:best=(score,keep)
    if best is None:return dict(valid=False,reason='NO_LONG_STRUCTURE_LINE')
    selected=xy[best[1]];slope,offset=np.polyfit(selected[:,0],selected[:,1],1)
    residual=float(np.sqrt(np.mean((selected[:,1]-slope*selected[:,0]-offset)**2)))
    return dict(valid=True,points=len(xy),inliers=len(selected),span_m=float(np.ptp(selected[:,0])),rms_m=residual,axis_angle_body_deg=math.degrees(math.atan(slope)),intercept_m=float(offset))

def observe_row_heading(points):
    left,right=line(points,1),line(points,-1)
    result=dict(valid=False,left=left,right=right)
    if not left['valid'] or not right['valid']:return result
    a,b=left['axis_angle_body_deg'],right['axis_angle_body_deg']
    if min(left['inliers'],right['inliers'])<100 or min(left['span_m'],right['span_m'])<1.5:return result
    if max(left['rms_m'],right['rms_m'])>.008 or abs(a-b)>.5:return result
    result.update(valid=True,body_heading_rad=math.radians(-(a+b)/2),agreement_deg=abs(a-b))
    return result
