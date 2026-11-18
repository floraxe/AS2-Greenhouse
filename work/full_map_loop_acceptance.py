"""Event audit, not a guarantee of geometric correctness of a loop link."""
class LoopAcceptance:
    def __init__(self):
        self.nodes={};self.seen=set();self.events=[]
    def observe(self,row,travel,finished=False):
        nav=row.get('navigation') or {};f=nav.get('fusion') or {}
        ref=int(f.get('ref_id') or 0);candidate=int(f.get('loop_closure_id') or 0)
        if ref>0:self.nodes.setdefault(ref,(row['t'],travel))
        key=(ref,candidate)
        if ref<=0 or candidate<=0 or key in self.seen:return
        self.seen.add(key);prior=self.nodes.get(candidate)
        return_phase=True  # Full-map revisits may occur on any selected aisle or headland.
        active=not finished and return_phase and nav.get('valid',False) and (row['state'].startswith(('FIXED_','ONLINE_')) or row['state'] in ('TURNING','ENTRANCE_TURNING','ROUTE_TURNING'))
        age=row['t']-prior[0] if prior else None
        separation=travel-prior[1] if prior else None
        accepted=bool(active and prior and age>=30. and separation>=5.)
        self.events.append(dict(t=row['t'],ref_id=ref,candidate_id=candidate,state=row['state'],candidate_age_s=age,travel_since_candidate_m=separation,accepted=accepted))
    @property
    def count(self):return sum(e['accepted'] for e in self.events)
