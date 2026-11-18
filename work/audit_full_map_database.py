"""Audit actual RTAB-Map saved optimized poses and loop constraints.
Official storage: Admin.opt_ids/opt_poses zlib + trailing rows/cols/type;
Node.pose remains RAW odometry. Reference trajectory is evaluation only.
"""
import bisect,json,math,sqlite3,struct,sys,zlib
from contextlib import closing
from pathlib import Path
import numpy as np
from audit_rtabmap_database import audit

def unpack_mat(blob,dtype):
    if not blob or len(blob)<13:raise ValueError('Missing compressed optimized graph')
    h,w,kind=struct.unpack('<3i',blob[-12:])
    if h<=0 or w<=0 or h*w>1000000:raise ValueError('Invalid compressed dimensions')
    expected=h*w*np.dtype(dtype).itemsize
    obj=zlib.decompressobj();raw=obj.decompress(blob[:-12],expected+1)
    if len(raw)!=expected or not obj.eof or obj.unconsumed_tail:raise ValueError('Invalid optimized graph payload')
    return np.frombuffer(raw,dtype=dtype)

def transform(blob):
    if not blob or len(blob)!=48:raise ValueError('Invalid transform size')
    m=np.eye(4);m[:3,:]=np.frombuffer(blob,dtype='<f4').reshape(3,4)
    if not np.isfinite(m).all() or np.max(abs(m[:3,:3].T@m[:3,:3]-np.eye(3)))>.01:raise ValueError('Invalid rotation')
    return m

def pose2(x,y,yaw):
    c,s=math.cos(yaw),math.sin(yaw);m=np.eye(4);m[:2,:2]=((c,-s),(s,c));m[:2,3]=(x,y);return m

def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi

def initial_alignment(rows):
    """Use the first READY frozen sensor alignment, never a startup placeholder.

    This is a recorded known-start transform, not a fit to the reference path.
    """
    for row in rows:
        navigation=row.get('navigation') or {}
        aligned=navigation.get('initial_alignment')
        if navigation.get('valid') and aligned and aligned.get('frozen',False):
            return aligned
    return None

def evaluate(database,trace):
    structural=audit(database)
    with closing(sqlite3.connect(database.resolve().as_uri()+'?mode=ro',uri=True)) as db:
        nodes={i:t for i,t in db.execute('SELECT id,stamp FROM Node')}
        compressed=db.execute('SELECT opt_ids,opt_poses FROM Admin LIMIT 1').fetchone()
        links=list(db.execute('SELECT from_id,to_id,type,transform FROM Link WHERE type IN (1,2,3)'))
    rows=json.loads(trace.read_text(encoding='utf-8'));times=[r['t'] for r in rows]
    arrival=next((r['t'] for r in rows if r['state']=='DONE'),None)
    evaluation_end=arrival if arrival is not None else rows[-1]['t']
    aligned=initial_alignment(rows)
    result=dict(database=str(database),structural=structural,mission_completed=arrival is not None,full_slam_verified=False)
    if not aligned:return dict(result,reason='NO_RECORDED_INITIAL_ALIGNMENT')
    def row_at(t):
        k=bisect.bisect_left(times,t);candidates=rows[max(0,k-1):min(len(rows),k+1)]
        if not candidates:return None
        r=min(candidates,key=lambda v:abs(v['t']-t))
        return r if abs(r['t']-t)<=.25 else None
    evidence=[];seen=set();unmatched=[]
    temporal={tuple(sorted(e['nodes'])):e for e in structural['global_events']}
    for a,b,kind,blob in links:
        key=(min(a,b),max(a,b),kind)
        if key in seen:continue
        seen.add(key)
        if a not in nodes or b not in nodes:
            unmatched.append(dict(nodes=[a,b],type=kind,reason='MISSING_NODE'));continue
        ra,rb=row_at(nodes[a]),row_at(nodes[b])
        if ra is None or rb is None:
            unmatched.append(dict(nodes=[a,b],type=kind,reason='NO_TIMESTAMP_MATCHED_REFERENCE'));continue
        ta=pose2(*ra['position'][:2],ra['yaw']);tb=pose2(*rb['position'][:2],rb['yaw'])
        measured=transform(blob);relative=np.linalg.inv(ta)@tb
        xy=float(np.linalg.norm(relative[:2,3]-measured[:2,3]))
        angle=abs(wrap(math.atan2(relative[1,0],relative[0,0])-math.atan2(measured[1,0],measured[0,0])))
        newer=ra if nodes[a]>=nodes[b] else rb
        prior=temporal.get(tuple(sorted((a,b))),{})
        return_phase=True  # Full-map active revisits; unchanged temporal and geometric gates.
        active=(newer['state'].startswith('FIXED_') or newer['state']=='ROUTE_TURNING') and return_phase
        eligible=bool(kind==1 and active and max(nodes[a],nodes[b])<evaluation_end and prior.get('long_baseline_candidate'))
        evidence.append(dict(nodes=[a,b],type=kind,time_separation_s=abs(nodes[a]-nodes[b]),newer_state=newer['state'],reference_translation_residual_m=xy,reference_yaw_residual_deg=math.degrees(angle),eligible_global=eligible,geometrically_consistent=bool(xy<=.10 and angle<=math.radians(5.))))
    result['constraints']=evidence
    result['unmatched_constraints']=unmatched
    result['inconsistent_constraints']=[e for e in evidence if not e['geometrically_consistent']]
    result['all_saved_closures_checked_and_consistent']=bool(evidence and not unmatched and not result['inconsistent_constraints'])
    accepted=[e for e in evidence if e['eligible_global'] and e['geometrically_consistent']]
    result['verified_active_global_constraints']=len(accepted)
    if not compressed or not compressed[0] or not compressed[1]:return dict(result,reason='OPTIMIZED_POSES_NOT_SAVED')
    ids=unpack_mat(compressed[0],'<i4');flat=unpack_mat(compressed[1],'<f4')
    if len(flat)!=len(ids)*12:raise ValueError('Optimized id/pose count mismatch')
    origin=pose2(*aligned['task_from_map']);errors=[]
    for node,values in zip(ids,flat.reshape(-1,12)):
        t=nodes.get(int(node));r=row_at(t) if t is not None else None
        if r is None or t>evaluation_end:continue
        mapped=origin@transform(values.astype('<f4').tobytes())
        errors.append(float(np.linalg.norm(mapped[:2,3]-r['position'][:2])))
    result['optimized_pose_samples']=len(errors)
    if errors:result['optimized_position_error_m']=dict(mean=float(np.mean(errors)),p95=float(np.percentile(errors,95)),maximum=max(errors))
    result['scope']='Independent planar constraint and optimized-pose evaluation against recorded simulator reference; not hardware acceptance'
    # Explicit experiment thresholds, not universal guarantees of correctness.
    result['criteria']=dict(global_translation_residual_max_m=.10,global_yaw_residual_max_deg=5.,optimized_position_p95_max_m=.15)
    result['full_slam_verified']=bool(structural['structural_checks_passed'] and result['all_saved_closures_checked_and_consistent'] and arrival is not None and accepted and errors and np.percentile(errors,95)<=.15)
    return result

if __name__=='__main__':
    root=Path('E:/AS2_Greenhouse');database=Path(sys.argv[1])
    trace=root/'outputs/route23_live'/'场景静止轨迹.json'
    result=evaluate(database,trace)
    (root/'outputs/route23_validation/database_geometry_audit.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('structural','constraints')},indent=2))
