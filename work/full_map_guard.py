"""Full-map raw-cloud guard using the measured 3D gait envelope.

Points have body XYZ and world-height as fourth column. No semantic filtering.
The original two-aisle 2D guard remains unchanged.
"""
from pathlib import Path
import math,json,hashlib
import numpy as np

ROOT=Path('E:/AS2_Greenhouse/work')
meta=json.loads((ROOT/'full_map_speed_collision_volume.json').read_text())
for name,digest in meta['source_sha256'].items():
    if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Collision model changed; rebuild full-map volume')
volume_path=ROOT/'full_map_speed_collision_volume.npz'
if hashlib.sha256(volume_path.read_bytes()).hexdigest()!=meta['volume_sha256']:raise RuntimeError('Collision volume checksum mismatch')
profile=np.load(volume_path);ORIGIN=profile['origin'];RES=float(profile['resolution'])
HARD=profile['occupied'];INFLATED=profile['inflated']
def dilate(v,axis,n):
    pads=[(0,0)]*3;pads[axis]=(n,n);a=np.pad(v,pads)
    a=np.cumsum(a,axis=axis,dtype=np.int32);pads=[(0,0)]*3;pads[axis]=(1,0);a=np.pad(a,pads)
    x=[slice(None)]*3;y=x.copy();x[axis]=slice(2*n+1,None);y[axis]=slice(None,-2*n-1)
    return a[tuple(x)]-a[tuple(y)]>0
CORE=HARD.copy()
for axis in range(3):CORE=dilate(CORE,axis,2)
CAUTIOUS=HARD.copy()
for axis in range(3):CAUTIOUS=dilate(CAUTIOUS,axis,6)
# Receding preview extends the full gait body by 0.25m (over 3s at 0.075m/s).
# The next commanded 0.6s sweep is checked separately; lateral targets are renewed.
extra=int(math.ceil(.25/RES))
def preview_grid(grid):
    a=np.pad(grid,((0,extra),(0,0),(0,0)))
    prefix=np.pad(np.cumsum(a,axis=0,dtype=np.int32),((1,0),(0,0),(0,0)))
    indices=np.arange(len(a));return (prefix[indices+1]-prefix[np.maximum(0,indices-extra)])>0
TIGHT=HARD.copy()
for axis in range(3):TIGHT=dilate(TIGHT,axis,4)
LEAF_CORE=HARD.copy()
for axis in range(3):LEAF_CORE &= ~dilate(~LEAF_CORE,axis,4)
LEAF_PREVIEW=preview_grid(LEAF_CORE)
HARD_PREVIEW=preview_grid(CORE)
GRIDS={.02:(TIGHT,preview_grid(TIGHT)),.05:(INFLATED,preview_grid(INFLATED)),.03:(CAUTIOUS,preview_grid(CAUTIOUS))}

_BOUNDS={}
def grid_bounds(grid):
    key=id(grid)
    if key not in _BOUNDS:
        lo=[];hi=[]
        for axis in range(3):
            present=np.flatnonzero(np.any(grid,axis=tuple(i for i in range(3) if i!=axis)))
            if not len(present):_BOUNDS[key]=(grid,None,None);return None,None
            lo.append(ORIGIN[axis]+present[0]*RES-1e-9);hi.append(ORIGIN[axis]+(present[-1]+1)*RES+1e-9)
        _BOUNDS[key]=(grid,np.array(lo),np.array(hi))
    return _BOUNDS[key][1:]

def swept_relevance(points,grid,dx,dy,angle):
    low,high=grid_bounds(grid)
    if low is None:return points[:0]
    lower=low.copy();upper=high.copy();radius=float(np.linalg.norm(np.maximum(abs(low[:2]),abs(high[:2]))))
    lower[:2]+=np.minimum([dx,dy],0)-radius*abs(angle);upper[:2]+=np.maximum([dx,dy],0)+radius*abs(angle)
    return points[np.all((points>=lower)&(points<=upper),axis=1)]

def hits(points,grid):
    low,high=grid_bounds(grid)
    if low is None or not len(points):return 0
    points=points[np.all((points>=low)&(points<=high),axis=1)]
    if not len(points):return 0
    indices=np.floor((points-ORIGIN)/RES).astype(np.int64)
    valid=np.all((indices>=0)&(indices<grid.shape),axis=1)
    return int(grid[tuple(indices[valid].T)].sum()) if valid.any() else 0

def split_obstacles(p):
    valid=(p[:,3]>.008)&(p[:,3]<1.1)&(abs(p[:,0])<1.2)&(abs(p[:,1])<.8)
    soft=np.zeros(len(p),dtype=bool)
    if p.shape[1]==5:
        # No foot/low-seedling points can ever be softened, even with a bad mask.
        soft=(p[:,4]>.5)&(p[:,3]>.17)&(p[:,2]>-.17)&(abs(p[:,1])>.14)
    return p[valid&~soft,:3],p[valid&soft,:3]

def _sweep(points,dx,dy,angle,age,preview=False,margin=.05,escape=False):
    p=np.asarray(points,dtype=float)
    out=dict(safe=False,reason='FULL_MAP_CLOUD_UNOBSERVED',tracking_margin_m=margin,hard_core_margin_m=.01,envelope='54_SHAPES_7200_GAIT_POSES_3D',translation_body_m=[float(dx),float(dy)],angle_rad=float(angle))
    if margin not in GRIDS:return dict(out,reason='INVALID_TRACKING_MARGIN')
    if p.ndim!=2 or p.shape[1] not in (4,5) or len(p)<100 or not np.isfinite(p).all():return out
    if not math.isfinite(age) or not 0<=age<=.4:return out
    if not all(map(math.isfinite,(dx,dy,angle))) or math.hypot(dx,dy)>.055 or abs(angle)>.03:return dict(out,reason='FULL_MAP_SWEEP_LIMIT')
    radius=np.linalg.norm(p[:,:2],axis=1);ground=(p[:,3]<.008)&(radius>.3)&(radius<2.)
    quadrants=np.floor((np.arctan2(p[ground,1],p[ground,0])+math.pi)/(math.pi/2)).astype(int).clip(0,3)
    out.update(ground_points=int(ground.sum()),quadrants=len(set(quadrants)))
    if out['ground_points']<32 or out['quadrants']<4:return out
    obstacle,foliage=split_obstacles(p)
    obstacle=swept_relevance(obstacle,GRIDS[margin][1 if preview else 0],dx,dy,angle)
    foliage=swept_relevance(foliage,LEAF_PREVIEW if preview else LEAF_CORE,dx,dy,angle)
    steps=1 if abs(dx)+abs(dy)+abs(angle)<1e-12 else max(17,math.ceil((math.hypot(dx,dy)+1.5*abs(angle))/.002)+1)
    max_hits=max_hard=max_deep=0;initial_hits=final_hits=0
    for fraction in np.linspace(0.,1.,steps):
        a=angle*fraction;c,s=math.cos(a),math.sin(a)
        q=obstacle.copy();px=q[:,0]-dx*fraction;py=q[:,1]-dy*fraction
        q[:,0]=c*px+s*py;q[:,1]=-s*px+c*py
        leaf=foliage.copy();lx=leaf[:,0]-dx*fraction;ly=leaf[:,1]-dy*fraction
        leaf[:,0]=c*lx+s*ly;leaf[:,1]=-s*lx+c*ly
        max_deep=max(max_deep,hits(leaf,LEAF_PREVIEW if preview else LEAF_CORE))
        count=hits(q,GRIDS[margin][1 if preview else 0]);max_hits=max(max_hits,count);final_hits=count;max_hard=max(max_hard,hits(q,HARD_PREVIEW if preview else CORE))
        if fraction==0:initial_hits=count
        if max_hard>0 or max_deep>0 or (not escape and max_hits>=3):break
    out.update(obstacle_points=max_hits,physical_envelope_points=max_hard,leaf_deep_points=max_deep,foliage_points=len(foliage),leaf_contact_allowance_m=.02,steps=steps)
    out.update(initial_margin_points=initial_hits,final_margin_points=final_hits,clearance_escape=escape)
    safe=(abs(dx)<1e-9 and not preview and max_hard==0 and max_deep==0) if escape else (max_hits<3 and max_hard==0 and max_deep==0)
    return dict(out,safe=safe,reason='FULL_MAP_3D_SWEEP_CLEAR' if safe else 'FULL_MAP_3D_SWEEP_OCCUPIED')

def evaluate_local_sweep(points,dx,dy,angle,age,margin=.05):return _sweep(points,dx,dy,angle,age,False,margin)
def evaluate_row_preview(points,dx,dy,angle,age,margin=.05):return _sweep(points,dx,dy,angle,age,True,margin)

def target_clear(points,target_y,target_angle,age,margin,final_goal=False):
    q=np.asarray(points,dtype=float).copy();x=q[:,0].copy();y=q[:,1]-target_y
    c,z=math.cos(target_angle),math.sin(target_angle);q[:,0]=c*x+z*y;q[:,1]=-z*x+c*y
    result=evaluate_local_sweep(q,0.,0.,0.,age,margin)
    if final_goal or not result['safe']:return result
    obs,leaves=split_obstacles(q)
    hard=hits(obs,HARD_PREVIEW);soft=hits(obs,GRIDS[margin][1]);deep=hits(leaves,LEAF_PREVIEW)
    safe=hard==0 and soft<3 and deep==0
    return dict(result,safe=safe,reason='TARGET_FORECAST_CLEAR' if safe else 'TARGET_FORECAST_OCCUPIED',forecast_m=.25,physical_envelope_points=hard,obstacle_points=soft,leaf_deep_points=deep)

def evaluate_clearance_escape(points,dy,angle,age,margin=.05,target_y=None,target_angle=None,final_goal=False):
    if target_y is None or target_angle is None or not all(map(math.isfinite,(target_y,target_angle))) or abs(target_y)>.10 or abs(target_angle)>.30 or dy*target_y<0 or abs(dy)>abs(target_y)+.001:
        return dict(safe=False,reason='CLEARANCE_ESCAPE_TARGET_INVALID')
    destination=target_clear(points,target_y,target_angle,age,margin,final_goal)
    if not destination['safe']:return dict(safe=False,reason='CLEARANCE_ESCAPE_DESTINATION_OCCUPIED')
    result=_sweep(points,0.,dy,angle,age,False,margin,True)
    result.update(target_y_m=target_y,target_angle_rad=target_angle,destination_clear=True)
    return result

PLAN_XY_MAX=np.maximum(abs(ORIGIN[:2]),abs(ORIGIN[:2]+np.array(HARD_PREVIEW.shape[:2])*RES))
PLAN_RADIUS=float(np.linalg.norm(PLAN_XY_MAX))+.10+.055+.01

def choose_row_clearance_target(points,observation,age,previous=None,final_goal=False):
    if not observation.get('valid'):return dict(valid=False,reason='NO_ROW_OBSERVATION')
    p=np.asarray(points,dtype=float)
    radius=np.linalg.norm(p[:,:2],axis=1)
    relevant=((p[:,3]<.008)&(radius<2.2))|((p[:,3]>.008)&(p[:,3]<1.1)&(radius<PLAN_RADIUS))
    if relevant.sum()>=100:p=p[relevant]
    center=float(observation['center_error_body_y_m']);angle=float(observation['heading_error_rad'])
    turn=float(np.clip(.9*angle*.6,-.027,.027))
    # Check targets ahead even when current-body inflation is occupied. A target
    # is usable only if BOTH the destination preview and commanded transition pass.
    reliable=all((observation.get(side) or {}).get('center_sigma_m',1)<=.002 and (observation.get(side) or {}).get('heading_sigma_deg',90)<=.3 for side in ('left','right'))
    precise=reliable and all(observation[side].get('center_sigma_m',1)<=.001 and observation[side].get('heading_sigma_deg',90)<=.1 for side in ('left','right')) and observation.get('floor_rms_m',1)<.005
    partial=observation.get('sensing_mode')=='SINGLE_BANK_WITH_RECENT_MEASURED_GAP'
    if partial:
        reliable=observation.get('partial_center_sigma_m',1)<=.002 and observation.get('partial_heading_sigma_deg',90)<=.15 and observation.get('gap_sigma_m',1)<=.004
        precise=reliable and observation.get('partial_center_sigma_m',1)<=.001 and observation.get('partial_heading_sigma_deg',90)<=.1 and observation.get('gap_sigma_m',1)<=.001 and observation.get('floor_rms_m',1)<.005
    margins=(.05,.03,.02) if precise else (.05,.03) if reliable else (.05,)
    offsets=(0.,-.01,.01,-.02,.02,-.03,.03,-.04,.04,-.05,.05,-.06,.06,-.07,.07,-.08,.08)
    choices=[]
    if previous and previous.get('valid') and not final_goal:
        held=float(previous.get('offset_from_row_center_m',0.));yaw=float(previous.get('yaw_offset_rad',0.))
        if abs(held)<=.08 and abs(yaw)<=.20:choices.extend((margin,held,yaw) for margin in margins)
    choices.extend((margin,offset,0.) for offset in offsets for margin in margins if (margin,offset,0.) not in choices)
    # Coupled lateral/heading poses only when the straight body preview has no solution.
    choices.extend((margin,offset,yaw) for yaw in (.06,-.06,.08,-.08,.10,-.10,.12,-.12,.14,-.14,.16,-.16,.18,-.18,.20,-.20) for offset in (0.,-.02,.02,-.04,.04,-.06,.06,-.08,.08) for margin in margins)
    for margin,offset,yaw in choices:
        target=center+offset;desired_angle=angle+yaw
        if abs(target)>.10 or abs(desired_angle)>.30:continue
        if not target_clear(p,target,desired_angle,age,margin,final_goal)['safe']:continue
        turn=float(np.clip(.9*desired_angle*.6,-.027,.027));dy=float(np.clip(1.4*target*.6,-.024,.024))
        transition=evaluate_local_sweep(p,0.,dy,turn,age,margin)
        escape_required=not transition['safe']
        if escape_required and not evaluate_clearance_escape(p,dy,turn,age,margin,target_y=target,target_angle=desired_angle,final_goal=final_goal)['safe']:continue
        lateral_first=escape_required or not evaluate_local_sweep(p,.045,dy,turn,age,margin)['safe']
        return dict(valid=True,escape_required=escape_required,lateral_first=lateral_first,reason='CENTER_3D_CLEAR' if offset==0 and yaw==0 else 'COUPLED_SAME_AISLE_3D_CLEARANCE',offset_from_row_center_m=offset,yaw_offset_rad=yaw,error_body_y_m=target,error_heading_rad=desired_angle,tracking_margin_m=margin,final_goal=final_goal)
    return dict(valid=False,reason='NO_SAFE_3D_SAME_AISLE_TARGET')
