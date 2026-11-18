"""Fuse current paired-bank observations into surveyed-row control pose.

Longitudinal progress remains SLAM-derived. Raw SLAM is retained separately
for mapping accuracy assessment; no simulator pose is used here.
"""
import math
from collections import deque
from slam_navigation_feedback import compose, inverse

class RowPropagationClock:
    """Exclude only continuously observed, commanded, stationary intervals.

    The anchor bounds TOTAL estimated drift throughout each stop, rather than
    accepting an arbitrarily long sequence of small moving increments.
    Invalid localization and unsampled intervals still consume the allowance.
    """
    def __init__(self):
        self.previous=None;self.anchor=None;self.blocked=False;self.credit=0.;self.stationary_candidate=None

    def observe(self,t,position,yaw,valid,command):
        stopped=bool(command.get('stop',False) and all(command.get(k)!=None and abs(command[k])<1e-8 for k in ('vx','vy','omega')))
        good=bool(valid and position is not None and yaw is not None and
                  all(math.isfinite(float(v)) for v in (*position[:2],yaw,t)))
        credited=False
        if not stopped:
            self.anchor=None;self.blocked=False;self.stationary_candidate=None
        elif not good:
            self.anchor=None;self.blocked=True;self.stationary_candidate=None
        elif self.previous is not None:
            pt,pp,py,pvalid,pstop=self.previous
            dt=t-pt
            if pvalid and pstop and 0.<dt<=.5:
                if self.blocked:
                    candidate=self.stationary_candidate
                    if (candidate is None or math.dist(position[:2],candidate[1:3])>.001
                            or abs((yaw-candidate[3]+math.pi)%(2*math.pi)-math.pi)>.002):
                        self.stationary_candidate=(t,*position[:2],yaw)
                    elif t-candidate[0]>=2.:
                        # Re-establish stationarity after gait settling or a
                        # localization interruption. The proving interval is
                        # counted in full, never credited retroactively.
                        self.anchor=(*position[:2],yaw);self.blocked=False;self.stationary_candidate=None
                else:
                    if self.anchor is None:self.anchor=(*pp,py)
                    drift=math.dist(position[:2],self.anchor[:2])
                    angle=abs((yaw-self.anchor[2]+math.pi)%(2*math.pi)-math.pi)
                    if drift<=.005 and angle<=.01:
                        self.credit+=dt;credited=True
                    else:self.blocked=True;self.stationary_candidate=None
            elif not pstop:
                self.anchor=(*position[:2],yaw);self.blocked=False
            else:
                # A gap cannot certify stationarity; require a new command
                # interval before crediting another stopped interval.
                self.blocked=True;self.stationary_candidate=None
        self.previous=(t,tuple(position[:2]) if good else None,yaw,good,stopped)
        return t-self.credit,dict(stationary_credit_s=self.credit,
                                  current_interval_credited=credited,
                                  stationary_credit_blocked=self.blocked)

class RowControlFrame:
    """Continuous SLAM-to-control frame across bank loss and headland turns."""
    def __init__(self):
        self.transform=(0.,0.,0.);self.last_fix=None;self.last_fix_phase=None;self.previous=None;self.phase=None
        self.map_rebase=(0.,0.,0.)
        self.yaw_offset=0.;self.end_pairs=deque(maxlen=30);self.current_pair=False

    def _anchor_position(self,raw_xy,measured_xy,rotation=None):
        angle=self.transform[2] if rotation is None else rotation
        c,s=math.cos(angle),math.sin(angle)
        self.transform=(measured_xy[0]-c*raw_xy[0]+s*raw_xy[1],measured_xy[1]-s*raw_xy[0]-c*raw_xy[1],angle)

    def rebase(self,cumulative):
        if tuple(cumulative)==self.map_rebase:return
        delta=compose(cumulative,inverse(self.map_rebase))
        self.transform=compose(self.transform,inverse(delta))
        self.yaw_offset=(self.yaw_offset-delta[2]+math.pi)%(2*math.pi)-math.pi
        self.end_pairs.clear()
        if self.last_fix:
            t,p=self.last_fix
            self.last_fix=(t,compose(delta,(*p,0.))[:2])
        self.map_rebase=tuple(cumulative)

    def anchor_end(self,raw_position,raw_yaw,control_position,control_yaw,endpoint_x,heading,observation):
        if not observation.get('valid'):return control_position,dict(applied=False)
        x=endpoint_x-math.cos(heading)*observation['distance_body_m']
        if abs(x-control_position[0])>1.5:return control_position,dict(applied=False,reason='END_INNOVATION_TOO_LARGE')
        out=list(control_position);out[0]=x
        calibrated=False
        if self.current_pair:
            self.end_pairs.append((tuple(raw_position[:2]),tuple(out[:2])))
            if len(self.end_pairs)>=8:
                pairs=list(self.end_pairs);n=len(pairs)
                a=[sum(v[0][i] for v in pairs)/n for i in (0,1)]
                b=[sum(v[1][i] for v in pairs)/n for i in (0,1)]
                va=sum(sum((v[0][i]-a[i])**2 for i in (0,1)) for v in pairs)
                vb=sum(sum((v[1][i]-b[i])**2 for i in (0,1)) for v in pairs)
                dot=sum((u[0]-a[0])*(v[0]-b[0])+(u[1]-a[1])*(v[1]-b[1]) for u,v in pairs)
                cross=sum((u[0]-a[0])*(v[1]-b[1])-(u[1]-a[1])*(v[0]-b[0]) for u,v in pairs)
                angle=math.atan2(cross,dot);c,s=math.cos(angle),math.sin(angle)
                residual=math.sqrt(sum(((c*(u[0]-a[0])-s*(u[1]-a[1]))-(v[0]-b[0]))**2+((s*(u[0]-a[0])+c*(u[1]-a[1]))-(v[1]-b[1]))**2 for u,v in pairs)/n)
                if va/n>.005 and vb/n>.005 and .8<math.sqrt(vb/va)<1.2 and residual<.025:
                    self._anchor_position(raw_position,out,angle);calibrated=True
        if not calibrated:self._anchor_position(raw_position,out)
        return out,dict(applied=True,source='MEASURED_PAIRED_BED_ENDS',x_m=x,observation=observation,translation_rotation_calibrated=calibrated,position_rotation_rad=self.transform[2],yaw_offset_rad=self.yaw_offset)

    def update(self,position,yaw,row_y,heading,observation,age,t,phase,paired):
        raw=(*position[:2],yaw)
        predicted=compose(self.transform,raw)
        predicted=(*predicted[:2],(yaw+self.yaw_offset+math.pi)%(2*math.pi)-math.pi)
        self.current_pair=False
        diagnostic=dict(applied=False,reason='WAIT_ROW_FRAME')
        if phase!=self.phase:self.previous=None;self.phase=phase
        if paired:
            # Validate innovation against the already corrected frame, never
            # against a drifting raw map frame when a new aisle is entered.
            out,angle,diagnostic=row_control_pose([*predicted[:2],0.],predicted[2],row_y,heading,observation,age,self.previous)
            if diagnostic['applied']:
                self._anchor_position(position,out)
                self.yaw_offset=(angle-yaw+math.pi)%(2*math.pi)-math.pi
                self.current_pair=True
                self.last_fix=(t,tuple(position[:2]))
                self.last_fix_phase=phase
                self.previous=dict(diagnostic)
                return out,angle,dict(diagnostic,source='CURRENT_BANKS',ready=True)
        if self.last_fix:
            elapsed=t-self.last_fix[0];distance=math.dist(position[:2],self.last_fix[1])
            # A newly entered row has no fresh bank fix until the robot has
            # traversed the headland. Do not count the legitimate transverse
            # transfer against the same-row 2.5m observation-loss allowance.
            limit=2.5 if paired and self.last_fix_phase==phase else 8.
            if 0<=elapsed<=240. and distance<=limit:
                return [*predicted[:2],0.],predicted[2],dict(applied=True,ready=True,reason='PROPAGATED_ROW_FRAME',source='SLAM_WITH_LAST_ROW_FRAME',age_s=elapsed,distance_m=distance)
            return [*predicted[:2],0.],predicted[2],dict(applied=False,ready=False,reason='ROW_FRAME_PROPAGATION_LIMIT')
        return list(position),yaw,dict(diagnostic,ready=not paired)

def row_control_pose(position, yaw, row_y, heading, observation, age, previous=None):
    diagnostic={'applied': False, 'reason': 'NO_CURRENT_PAIRED_BANKS'}
    pose=list(position)
    if not observation.get('valid') or observation.get('sensing_mode')!='CURRENT_TWO_BANKS' or not 0<=age<=.3:
        return pose,yaw,diagnostic
    for side in ('left','right'):
        bank=observation.get(side) or {}
        if not bank or bank.get('rms_m',1)>.003 or bank.get('center_sigma_m',1)>.003 or bank.get('span_m',0)<.8:
            return pose,yaw,dict(diagnostic,reason='BANK_QUALITY_INSUFFICIENT')
    lateral=observation['center_error_body_y_m']
    angle=observation['heading_error_rad']
    measured_y=row_y-math.cos(heading)*lateral
    measured_yaw=(heading-angle+math.pi)%(2*math.pi)-math.pi
    delta_y=measured_y-position[1]
    delta_yaw=(measured_yaw-yaw+math.pi)%(2*math.pi)-math.pi
    # Once a correction is established, test its innovation rather than the
    # accumulated raw-SLAM drift. This does not enlarge the route corridor.
    previous=previous or {}
    # Validate the measured robot-to-row pose itself. A change in raw ICP yaw
    # during the gait must not invalidate a stable, well observed row pose.
    measured_step=abs(measured_y-previous['measured_y_m']) if previous.get('applied') and 'measured_y_m' in previous else 0.
    if not all(math.isfinite(v) for v in (lateral,angle,delta_y,delta_yaw)) or abs(lateral)>.15 or abs(angle)>.25 or measured_step>.03 or (not previous.get('applied') and abs(delta_y)>.25):
        return pose,yaw,dict(diagnostic,reason='ROW_SLAM_DISAGREEMENT')
    pose[1]=measured_y
    return pose,measured_yaw,dict(applied=True,reason='CURRENT_PAIRED_BANK_CONTROL_FUSION',lateral_correction_m=delta_y,heading_correction_rad=delta_yaw,measured_y_m=measured_y)
