"""Measured paired bed ends, with uncut continuation and observed free-ground tests."""
import math
import numpy as np


def observe_bed_end(points,heights,gap,angle=0.):
    p=np.asarray(points,dtype=float);h=np.asarray(heights,dtype=float)
    if p.ndim!=2 or p.shape[1]<3 or h.ndim!=1 or len(p)!=len(h) or len(p)<40:
        return dict(valid=False,reason='NO_END_SCAN')
    if not math.isfinite(gap) or gap<=0 or not math.isfinite(angle):
        return dict(valid=False,reason='INVALID_END_GEOMETRY')
    c,s=math.cos(angle),math.sin(angle)
    along=c*p[:,0]+s*p[:,1];cross=-s*p[:,0]+c*p[:,1]
    finite=np.isfinite(p).all(axis=1)&np.isfinite(h)
    low=finite&(h>.045)&(h<.19)
    ends=[];counts=[];free_counts=[]
    for sign in (-1,1):
        side=(sign*cross>gap/2+.015)&(sign*cross<gap/2+.23)
        # Keep the full measured scan in the continuation test. The old 1.5 m
        # analysis window must never manufacture a physical termination.
        support=low&side&(along>-3.)&(along<1.5)
        x=along[support]
        if len(x)<30 or np.ptp(x)<.4:
            return dict(valid=False,reason='INSUFFICIENT_PAIRED_END_SUPPORT')
        edge=float(np.quantile(x,.995))
        if edge>1.30:
            return dict(valid=False,reason='END_AT_ANALYSIS_BOUNDARY')
        if np.count_nonzero(low&side&(along>edge+.12))>=3:
            return dict(valid=False,reason='BED_CONTINUES_BEYOND_CANDIDATE')
        if np.count_nonzero(abs(x-edge)<.08)<6:
            return dict(valid=False,reason='SPARSE_END_SUPPORT')
        # Missing returns alone are not an end. Require visible floor beyond
        # the candidate in the same bed-side strip, spanning several bins.
        floor=finite&side&(abs(h)<.025)&(along>edge+.15)&(along<edge+.85)
        fx=along[floor]
        bins=np.unique(np.floor((fx-edge-.15)/.14).astype(int))
        if len(fx)<8 or len(bins)<3:
            return dict(valid=False,reason='NO_OBSERVED_GROUND_BEYOND_END')
        ends.append(edge);counts.append(len(x));free_counts.append(len(fx))
    if abs(ends[0]-ends[1])>.10:
        return dict(valid=False,reason='ENDS_DISAGREE',ends=ends)
    return dict(valid=True,reason='PAIRED_LOW_BED_ENDS_WITH_FREE_GROUND',
                distance_body_m=sum(ends)/2,ends=ends,counts=counts,
                free_ground_counts=free_counts,termination_verified=True)
