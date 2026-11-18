"""Task destinations + observed-map A* + existing joint-driven motion controller.
No saved route polyline, no geometry access; task/geofence priors remain explicit.
"""
import math
import numpy as np
from aisle_mission import AisleMission, wrap


class OnlineAisleMission(AisleMission):
    def __init__(self, start, planner):
        super().__init__(start)
        self.planner = planner
        self.plan_status = dict(valid=False, reason='WAIT_LIVE_GRID', path=[])

    def permitted(self,pos):
        x,y=pos[:2]
        return super().permitted(pos) or (-6.80<x<=-4.70 and -2.80<y<-.65)

    def step(self, pos, yaw, t, linear_clear, turn_clear):
        zero=(0.,0.,0.,True)
        if not self.permitted(pos):return 'FAULT_AISLE_ID_BOUNDARY',zero
        front_targets={'FRONT_APPROACH':(-4.95,-2.15),'FRONT_ENTER':(-5.60,-1.70),
                       'FRONT_EXIT':(-4.95,-2.15),'CLOSE_START':self.start}
        if self.phase=='FRONT_TURN':
            if not turn_clear:return 'WAIT_TURN_SPACE',zero
            error=wrap(-yaw)
            if abs(error)<.035:
                self.phase='FRONT_EXIT';self.events['entrance_turned_time']=t
                return 'ENTRANCE_TURN_COMPLETE',zero
            v=(np.asarray((-5.60,-1.70))-np.asarray(pos[:2]))*.45
            speed=float(np.linalg.norm(v))
            if speed>.035:v*=.035/speed
            c,s=math.cos(yaw),math.sin(yaw)
            return 'ENTRANCE_TURNING',(float(c*v[0]+s*v[1]),float(-s*v[0]+c*v[1]),float(np.clip(.8*error,-.2,.2)),False)
        if self.phase in front_targets:
            if not self.permitted(pos):return 'FAULT_AISLE_ID_BOUNDARY',zero
            target=np.asarray(front_targets[self.phase])
            if np.linalg.norm(target-np.asarray(pos[:2]))<.085:
                if self.phase=='FRONT_APPROACH':self.phase='FRONT_ENTER'
                elif self.phase=='FRONT_ENTER':
                    if not turn_clear:return 'WAIT_END_CONFIRMATION',zero
                    if self.confirm is None:self.confirm=t
                    if t-self.confirm<.8:return 'CONFIRM_ENTRANCE_BAY',zero
                    self.phase='FRONT_TURN';self.events['entrance_confirmed_time']=t
                elif self.phase=='FRONT_EXIT':self.phase='CLOSE_START'
                else:
                    self.done=True;self.events['same_heading_home_time']=t
                    return 'DONE',zero
                return 'NEXT_ENTRANCE_STAGE',zero
            if not linear_clear:return 'WAIT_OBSTACLE_OR_COVERAGE',zero
            state,command=self.phase,(0.,0.,0.,False)
        else:
            state, command = super().step(pos, yaw, t, linear_clear, turn_clear)
            if self.done:
                # Continue to the real entrance headland and return to the
                # start facing the original direction, allowing visual revisit.
                self.done=False;self.phase='FRONT_APPROACH';self.confirm=None
                self.events['opposite_heading_start_revisit']=t
                return 'CONTINUE_TO_ENTRANCE_BAY',zero
            target = {'OUT':self.exit, 'ENTER_BAY':self.turn,
                      'EXIT_BAY':self.exit, 'RETURN':self.start}.get(self.phase)
        if command[3] or self.phase in ('TURN','FRONT_TURN'):
            return state, command
        self.plan_status = self.planner.plan(pos[:2], target, t)
        if not self.plan_status['valid']:
            return 'WAIT_ONLINE_PATH', (0.,0.,0.,True)
        path = self.plan_status['path']
        # Short lookahead along adjacent free cells, no straight-line shortcuts.
        look = np.asarray(path[0])
        for point in path[1:]:
            # A polyline can bend around a corner. Check the entire shortcut
            # before steering toward a lookahead point, not just its endpoint.
            core=self.planner.core if hasattr(self.planner,'core') else None
            if core is not None:
                delta=np.asarray(point)-np.asarray(pos[:2])
                steps=max(1,int(np.linalg.norm(delta)/(core.res*.4)))
                if any(not core.free[core.cell(np.asarray(pos[:2])+delta*a)] for a in np.linspace(0.,1.,steps+1)):
                    break
            look = np.asarray(point)
            if np.linalg.norm(look-np.asarray(pos[:2])) >= .20:
                break
        velocity = (look-np.asarray(pos[:2]))*.65
        speed = float(np.linalg.norm(velocity))
        if speed > .10:
            velocity *= .10/speed
        c, s = math.cos(yaw), math.sin(yaw)
        heading = 0. if self.phase in ('OUT','ENTER_BAY','FRONT_EXIT','CLOSE_START') else math.pi
        return 'ONLINE_'+self.phase, (float(np.clip(c*velocity[0]+s*velocity[1],-.05,.11)),
                                      float(np.clip(-s*velocity[0]+c*velocity[1],-.06,.06)),
                                      float(np.clip(.8*wrap(heading-yaw),-.15,.15)), False)
