"""Read-only DB structural/temporal audit; not proof of geometric loop correctness.
RTAB-Map DBDriverSqlite3 stores Transform as 12 native floats (3x4).
Node.pose is odometry pose, NOT optimized graph pose.
"""
import json,sqlite3,struct,math,sys
from pathlib import Path
from contextlib import closing

def pose(blob):
    if not blob or len(blob)!=48:return None
    a=struct.unpack('<12f',blob)
    if not all(math.isfinite(v) for v in a):return None
    r=[a[0:3],a[4:7],a[8:11]]
    err=max(abs(sum(r[k][i]*r[k][j] for k in range(3))-(i==j)) for i in range(3) for j in range(3))
    det=r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])-r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])+r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0])
    if err>.01 or abs(det-1)>.01:return None
    return (a[3],a[7],a[11],math.atan2(a[4],a[0]))

def audit(path):
    if not path.exists():raise FileNotFoundError(path)
    with closing(sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True)) as db:
        integrity=db.execute('PRAGMA quick_check').fetchone()[0]
        nodes={i:dict(stamp=t,pose=pose(p),map_id=m) for i,t,p,m in db.execute('SELECT id,stamp,pose,map_id FROM Node')}
        links=list(db.execute('SELECT from_id,to_id,type,transform FROM Link'))
    kinds={};unique={};bad=[];adj={i:set() for i in nodes}
    for a,b,k,tr in links:
        kinds[k]=kinds.get(k,0)+1
        unique.setdefault((min(a,b),max(a,b),k),(a,b,tr))
        if a not in nodes or b not in nodes or a==b:bad.append([a,b,k]);continue
        adj[a].add(b);adj[b].add(a)
    reached=set();todo=[min(nodes)] if nodes else []
    while todo:
        i=todo.pop()
        if i in reached:continue
        reached.add(i);todo.extend(adj[i]-reached)
    # Travel separation derives only from raw Node odometry and stays diagnostic.
    prefix={};travel=0.;prev=None
    for i,n in sorted(nodes.items(),key=lambda it:it[1]['stamp']):
        if prev and n['pose'] and prev['map_id']==n['map_id'] and prev['pose']:
            travel+=math.dist(n['pose'][:2],prev['pose'][:2])
        prefix[i]=travel;prev=n
    events=[];invalid=[]
    for (lo,hi,k),(a,b,tr) in unique.items():
        if k not in (0,1,2,3):continue
        if pose(tr) is None:invalid.append([a,b,k])
        if k!=1 or a not in nodes or b not in nodes:continue
        age=abs(nodes[a]['stamp']-nodes[b]['stamp']);distance=abs(prefix[a]-prefix[b])
        events.append(dict(nodes=[a,b],stamp_a=nodes[a]['stamp'],stamp_b=nodes[b]['stamp'],age_s=age,raw_odometry_travel_separation_m=distance,long_baseline_candidate=bool(age>=30 and distance>=5),transform_valid=pose(tr) is not None))
    return dict(database=str(path),integrity=integrity,node_count=len(nodes),directed_link_counts=kinds,unique_link_counts={str(k):sum(key[2]==k for key in unique) for k in kinds},connected_nodes=len(reached),invalid_endpoints=bad,invalid_transforms=invalid,invalid_node_poses=[i for i,n in nodes.items() if n['pose'] is None],global_events=events,structural_checks_passed=bool(nodes and integrity=='ok' and len(reached)==len(nodes) and not bad and not invalid and all(n['pose'] for n in nodes.values())),geometric_loop_accuracy_verified=False,limits=['Node.pose is raw odometry; not optimized ground-truth error','Long temporal/travel separation is necessary evidence, not proof of a correct loop','Match trajectory event time to active mission; reject post-DONE repeats'])

if __name__=='__main__':
    p=Path(sys.argv[1]).expanduser()
    result=audit(p)
    output=p.with_suffix('.audit.json');output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print('DATABASE_AUDIT '+json.dumps({k:result[k] for k in ('node_count','unique_link_counts','structural_checks_passed','geometric_loop_accuracy_verified')}));print('Long-baseline global candidates: '+str(sum(e['long_baseline_candidate'] for e in result['global_events'])));print('Saved audit: '+str(output))
