"""Ground-referenced low crop-bed edges estimate local row center.
Only measured lidar + fixed extrinsics; no robot/world reference pose input.
"""
import math
import numpy as np

def estimate_row_center(lidar):
    p=np.asarray(lidar,dtype=float).reshape(-1,3)+np.array((.3,0,.32))
    out=dict(valid=False,reason='ROW_EDGES_UNOBSERVED')
    if len(p)<100 or not np.isfinite(p).all():return out
    radius=np.linalg.norm(p[:,:2],axis=1);floor=(p[:,2]<-.22)&(radius>.4)&(radius<3.)
    if floor.sum()<30:return out
    A=np.column_stack((p[floor,:2],np.ones(int(floor.sum()))));z=p[floor,2]
    for _ in range(4):
        plane=np.linalg.lstsq(A,z,rcond=None)[0];error=abs(A@plane-z);keep=error<max(.005,float(np.quantile(error,.7)));A=A[keep];z=z[keep]
    rms=float(np.sqrt(np.mean((A@plane-z)**2)))
    if rms>.02 or np.linalg.norm(plane[:2])>.35:return dict(out,reason='ROW_FLOOR_UNRELIABLE')
    height=p[:,2]-np.column_stack((p[:,:2],np.ones(len(p))))@plane
    selected=(height>.03)&(height<.08)&(abs(p[:,0])<2.)
    def fit(sign):
        side=selected&(p[:,1]*sign>.20)&(p[:,1]*sign<.70)
        xy=p[side,:2];hh=height[side]
        if len(xy)<25:return None
        rng=np.random.default_rng(37);best=None
        # Same 500 seeded samples and first-best tie policy, solved in batches.
        ids=np.array([rng.choice(len(xy),3,replace=False) for _ in range(500)])
        design0=np.stack((xy[ids,0],hh[ids],np.ones_like(hh[ids])),axis=2)
        eligible=(np.ptp(xy[ids,0],axis=1)>=.7)&(np.ptp(hh[ids],axis=1)>=.015)
        candidates=np.flatnonzero(eligible)
        if len(candidates):
            candidates=candidates[np.linalg.cond(design0[candidates])<=1e5]
        if len(candidates):
            coefficients=np.linalg.solve(design0[candidates],xy[ids[candidates],1][...,None])[...,0]
            for (slope,gain,intercept) in coefficients:
                if abs(slope)>.58 or not .1<sign*gain<1.5 or not .20<sign*(intercept+gain*.05)<.70:continue
                good=abs(xy[:,1]-slope*xy[:,0]-gain*hh-intercept)<.008
                bins=len(np.unique(np.floor(xy[good,0]/.15)))
                if bins<8:continue
                score=int(good.sum())
                if best is None or score>best[0]:best=(score,good)
        if best is None:return None
        points=xy[best[1]];heights=hh[best[1]]
        if np.ptp(heights)<.015:return None
        design=np.column_stack((points[:,0],heights,np.ones(len(points))));target=points[:,1]
        for _ in range(3):
            coef=np.linalg.lstsq(design,target,rcond=None)[0];residual=abs(design@coef-target)
            keep=residual<max(.004,float(np.quantile(residual,.8)))
            design=design[keep];target=target[keep]
        m,gain,b=coef
        if not .10<sign*gain<1.5 or abs(m)>.58:return None
        rms=float(np.sqrt(np.mean((design@coef-target)**2)))
        if rms>.008 or len(target)<25 or np.linalg.cond(design)>1e6:return None
        # At an entrance only part of the bank is visible. Require sufficient
        # measured information rather than a fixed long-span test alone.
        covariance=np.linalg.inv(design.T@design)*max(rms,.003)**2
        reference=np.array((0.,.05,1.))
        center_sigma=float(math.sqrt(max(0.,reference@covariance@reference)))
        heading_sigma=float(math.sqrt(max(0.,covariance[0,0]))/(1+m*m))
        if center_sigma>.01 or heading_sigma>math.radians(.5):return None
        # Intersect each bank plane at the SAME measured height. This removes
        # range-dependent slope bias caused by projecting an inclined bank into XY.
        return dict(slope=float(m),intercept=float(b+gain*.05),height_gain=float(gain),reference_height_m=.05,center_sigma_m=center_sigma,heading_sigma_deg=math.degrees(heading_sigma),inliers=len(target),span_m=float(np.ptp(design[:,0])),rms_m=rms)
    left,right=fit(1),fit(-1);out.update(left=left,right=right,floor_rms_m=rms,floor_plane=plane.tolist())
    if not left or not right:return out
    disagreement=abs(math.atan(left['slope'])-math.atan(right['slope']))
    slope=(left['slope']+right['slope'])/2;center=(left['intercept']+right['intercept'])/2
    width=(left['intercept']-right['intercept'])/math.sqrt(1+slope*slope)
    if disagreement>math.radians(1.) or min(left['span_m'],right['span_m'])<.8 or not .45<width<1.10:return dict(out,reason='ROW_EDGE_GEOMETRY_INCONSISTENT')
    return dict(out,valid=True,reason='MEASURED_ROW_CENTER',center_error_body_y_m=center,heading_error_rad=math.atan(slope),observed_bank_gap_m=width)
