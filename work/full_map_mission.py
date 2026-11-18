"""Surveyed coverage of selected aisles; estimated pose only."""
import json,math
from pathlib import Path
from full_map_route_controller import FixedRouteMission

CONFIG=Path(__file__).with_name('full_map_route_config.json')

class FullMapMission(FixedRouteMission):
    def __init__(self,config=None,cruise_speed=None,selected_aisles=None):
        self.config=config or json.loads(CONFIG.read_text(encoding='utf-8'))
        if selected_aisles is not None:
            import copy
            self.config=copy.deepcopy(self.config)
            self.config['selected_aisles_from_right']=list(selected_aisles)
            self.config['excluded_aisles_from_right']=[a['id'] for a in self.config['aisles'] if a['id'] not in selected_aisles]
            for a in self.config['aisles']:a['selected']=a['id'] in selected_aisles
        motion=Path(__file__).with_name('full_map_motion_config.json')
        default=json.loads(motion.read_text())['cruise_speed_mps'] if motion.exists() else .075
        self.cruise_speed=float(default if cruise_speed is None else cruise_speed)
        if self.cruise_speed not in (.075,.10,.12):raise ValueError('Unvalidated cruise speed')
        self.aisles={a['id']:a for a in self.config['aisles']}
        self.selected=tuple(self.config['selected_aisles_from_right'])
        self.start=tuple(self.config['start_xy_m']);self.stages=[];self.stage_aisles=[]
        self.row_names=set();self.coverage_stage_names={}
        far,entry=self.config['headland_x_m']['far'],self.config['headland_x_m']['entry']
        def add(name,xy,yaw,lane):
            self.stages.append((name,tuple(xy),yaw));self.stage_aisles.append(lane)
        def connect(prefix,x,source,dest,incoming,outgoing):
            sy,dy=self.aisles[source]['center_y_m'],self.aisles[dest]['center_y_m']
            transverse=math.copysign(math.pi/2,dy-sy)
            add(prefix+'_TURN_ACROSS',(x,sy),transverse,source)
            add(prefix+'_TRANSFER',(x,dy),transverse,None)
            add(prefix+'_TURN_ALONG',(x,dy),outgoing,dest)
        for k,lane in enumerate(self.selected):
            heading=0. if k%2==0 else math.pi;x=far if k%2==0 else entry
            name=f'AISLE{lane}_'+('OUT' if k%2==0 else 'IN')
            add(name,(x,self.aisles[lane]['center_y_m']),heading,lane)
            self.row_names.add(name);self.coverage_stage_names[lane]=name
            if k+1<len(self.selected):connect(f'LINK_{lane}_{self.selected[k+1]}',x,lane,self.selected[k+1],heading,math.pi-heading)
        first,last=self.selected[0],self.selected[-1]
        if len(self.selected)%2:
            connect('HOME_LINK',far,last,first,0.,math.pi)
            add('HOME_IN',(entry,self.start[1]),math.pi,first);self.row_names.add('HOME_IN')
            add('HOME_TURN_ALIGN',(entry,self.start[1]),0.,first)
        else:
            # Aisle5 finishes at the entry/bottom headland. Traverse directly
            # to aisle2 here, align +X, then approach the original start.
            connect('HOME_LINK',entry,last,first,math.pi,0.)
        add('CLOSE_START',self.start,0.,first);self.row_names.add('CLOSE_START')
        self.reset()

    def tracking_speed(self,pos):
        # Slow only for a turn or the last 35 cm, including the home approach.
        if pos is None:return .075
        name,target,heading=self.stages[min(self.index,len(self.stages)-1)]
        if 'TURN' in name:return .075
        along=(target[0]-pos[0])*math.cos(heading)+(target[1]-pos[1])*math.sin(heading)
        return .075 if abs(along)<.35 else self.cruise_speed

    def home_approach_pose(self,navigation_pose,row_control_pose):
        """Keep the surveyed start's longitudinal coordinate in the SLAM frame.

        Paired-row fusion can translate the propagated control frame when it
        anchors a bed end. That frame is useful for row-centering, but its x
        origin is no longer the known SLAM start origin. Mixing that x value
        with the surveyed start goal causes premature home arrival.
        """
        pose=list(row_control_pose)
        if self.stages[min(self.index,len(self.stages)-1)][0]=='CLOSE_START':
            pose[0]=float(navigation_pose[0])
        return pose

    def end_anchor_target(self):
        """Only a row stage whose destination is beyond its approached bed end."""
        if self.done or self.current_aisle in (None,6):return None
        name,target,heading=self.stages[self.index]
        if name not in self.row_names or name=='CLOSE_START':return None
        lo,hi=self.config['crop_x_extent_m']
        if math.cos(heading)>.99 and target[0]>=hi:return hi
        if math.cos(heading)<-.99 and target[0]<=lo:return lo
        return None

    def row_command_speed(self,velocity,pos):
        """Preserve cruise; retain bounded reverse pulses only within 8 cm home."""
        close=self.stages[self.index][0]=='CLOSE_START'
        remaining=abs(pos[0]-self.start[0])
        if velocity<0:return max(-.055,velocity) if close and remaining<.08 else 0.
        if velocity==0:return 0.
        ceiling=self.tracking_speed(pos)
        floor=.055 if close and remaining<.35 else ceiling
        return max(floor,min(ceiling,velocity))

    @property
    def current_aisle(self):
        return self.stage_aisles[min(self.index,len(self.stage_aisles)-1)]

    def is_row_phase(self):return self.phase in self.row_names

    def needs_paired_edges(self):
        return self.is_row_phase() and self.current_aisle!=6

    def permitted(self,pos):
        x,y=map(float,pos[:2])
        if not all(map(math.isfinite,(x,y))):return False
        entry,far=self.config['headland_x_m']['entry'],self.config['headland_x_m']['far']
        if x<=entry-1.2 or x>=far+1.2:return False
        low=self.aisles[self.selected[0]]['center_y_m']-.15
        high=self.aisles[self.selected[-1]]['center_y_m']+.15
        crop_min,crop_max=self.config['crop_x_extent_m']
        if x<=crop_min-.5 or x>=crop_max+.5:return low<y<high
        lane=self.current_aisle
        if lane is None:return False
        return abs(y-self.aisles[lane]['center_y_m'])<.15

    def step(self,pos,yaw,t,linear_clear,turn_clear,row_observation=None):
        previous=self.index
        state,command=super().step(pos,yaw,t,linear_clear,turn_clear,row_observation)
        if self.index!=previous and previous<len(self.stages):
            name=self.stages[previous][0]
            if name in self.coverage_stage_names.values():
                lane=self.stage_aisles[previous];self.events[f'aisle_{lane}_endpoint_time']=t
        if state=='FAULT_ROUTE23_BOUNDARY':state='FAULT_FULL_MAP_CORRIDOR_BOUNDARY'
        self.plan_status['reason']='AISLES_2_3_LOOP_NO_ASTAR' if self.selected==(2,3) else 'SERPENTINE_AISLES_2_TO_5_BOTTOM_RETURN_NO_ASTAR'
        self.plan_status['current_aisle']=self.current_aisle
        return state,command

    def corridor_metrics(self, rows):
        """Truth-based route scoring, restricted to the configured crop length."""
        low, high = self.config['crop_x_extent_m']
        stage_lane = dict(zip((stage[0] for stage in self.stages), self.stage_aisles))
        peaks = {}
        bay_offset = None
        samples = 0
        for row in rows:
            x, y = row['position'][:2]
            if not low <= x <= high:
                offset = abs(y - self.start[1])
                bay_offset = offset if bay_offset is None else max(offset, bay_offset)
                continue
            lane = stage_lane.get(row.get('phase'))
            if lane is None:
                # Older traces may lack a phase. Preserve the geometric definition.
                lane = min(self.selected, key=lambda i: abs(y - self.aisles[i]['center_y_m']))
            error = abs(y - self.aisles[lane]['center_y_m'])
            peaks[lane] = max(error, peaks.get(lane, 0.0))
            samples += 1
        return dict(crop_x_extent_m=[low, high], corridor_samples=samples,
                    per_aisle_max_m=peaks, max_corridor_lateral_error_m=max(peaks.values(), default=None),
                    max_bay_lateral_offset_m=bay_offset)

    def coverage(self,rows):
        """Independent spatial coverage along the cultivated length, not event counts."""
        import numpy as np
        x0,x1=self.config['crop_x_extent_m'];n=max(1,round((x1-x0)/.1))
        bins={lane:set() for lane in self.selected};wrong=0;excluded=0
        for r in rows:
            x,y=r['position'][:2]
            if not x0<=x<=x1:continue
            lane=min(self.aisles,key=lambda a:abs(y-self.aisles[a]['center_y_m']))
            if lane==1:wrong+=1
            if lane not in self.selected:excluded+=1;continue
            if lane in bins and abs(y-self.aisles[lane]['center_y_m'])<.15:
                bins[lane].add(min(n-1,int((x-x0)/(x1-x0)*n)))
        return {'per_aisle':{str(a):{'covered_bins':len(bins[a]),'total_bins':n,'fraction':len(bins[a])/n} for a in self.selected},'forbidden_aisle_1_samples':wrong,'excluded_aisle_samples':excluded,'all_selected_aisles_covered':all(len(v)==n for v in bins.values())}
