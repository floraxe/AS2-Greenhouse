"""Full-map-only causal translation age compensation from measured odometry.
No reference trajectory or command velocity is used. Health gates stay upstream.
"""
import math
import time
from online_slam_feedback import OnlineSlamFeedback, RosOnlineSlamFeedback
from slam_navigation_feedback import compose,inverse,wrap


def compensate_position(position, yaw, history, age):
    diagnostic=dict(applied=False, horizon_s=0., source='PAST_MEASURED_ODOMETRY')
    if not history or not 0. <= age <= .3:
        return list(position), diagnostic
    stamp=history[-1][0]
    recent=[entry for entry in history if stamp-entry[0]<=.6]
    if len(recent)<3 or stamp-recent[0][0]<.3:
        return list(position), diagnostic
    ts=[entry[0]-stamp for entry in recent];mean_t=sum(ts)/len(ts)
    mx=sum(entry[1][0] for entry in recent)/len(recent)
    my=sum(entry[1][1] for entry in recent)/len(recent)
    den=sum((t-mean_t)**2 for t in ts)
    if den<=1e-12:return list(position),diagnostic
    vx=sum((t-mean_t)*(entry[1][0]-mx) for t,entry in zip(ts,recent))/den
    vy=sum((t-mean_t)*(entry[1][1]-my) for t,entry in zip(ts,recent))/den
    if not math.isfinite(vx+vy) or math.hypot(vx,vy)>.15:
        return list(position),diagnostic
    angle=yaw-history[-1][1][2];c,s=math.cos(angle),math.sin(angle)
    dx,dy=(c*vx-s*vy)*age,(s*vx+c*vy)*age
    return [position[0]+dx,position[1]+dy,position[2]],dict(diagnostic,applied=True,
        horizon_s=age, fit_span_s=stamp-recent[0][0], samples=len(recent),
        translation_m=[dx,dy], speed_mps=math.hypot(vx,vy))


def planar_map_tf(transform):
    """Accept only the estimated map->odom edge, never simulated body TFs."""
    if transform.header.frame_id != 'map' or transform.child_frame_id != 'odom':
        return None
    t=transform.transform.translation;q=transform.transform.rotation
    values=(t.x,t.y,t.z,q.x,q.y,q.z,q.w)
    if not all(math.isfinite(v) for v in values):return None
    if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01:return None
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    return (t.x,t.y,yaw)


class FullMapSlamFeedback(OnlineSlamFeedback):
    def graph_correction(self,stamp,correction,wall=None):
        wall=time.monotonic() if wall is None else wall
        if not self.config_ok or not all(math.isfinite(v) for v in correction):return
        if self.graph and stamp<=self.graph[0]:return
        if stamp<=getattr(self,'last_candidate_stamp',-1.):return
        self.last_candidate_stamp=stamp
        old=self.correction
        jump=bool(old and (math.dist(correction[:2],old[:2])>.25 or abs(wrap(correction[2]-old[2]))>.35))
        pending=getattr(self,'pending_map_revision',None)
        if not jump:
            self.pending_map_revision=None
            return super().graph_correction(stamp,correction,wall)
        if pending is None:
            pending=dict(start=stamp,wall=wall,stable_since=stamp,count=0,last=tuple(correction),old=tuple(old))
        consistent=math.dist(correction[:2],pending['last'][:2])<=.03 and abs(wrap(correction[2]-pending['last'][2]))<=.02
        if not consistent:pending.update(stable_since=stamp,count=0)
        pending.update(last=tuple(correction),count=pending['count']+1)
        self.pending_map_revision=pending
        event=dict(stamp=stamp,old=list(old),candidate=list(correction),translation_jump_m=math.dist(correction[:2],old[:2]),angle_jump_rad=abs(wrap(correction[2]-old[2])),stable_count=pending['count'],stable_span_s=stamp-pending['stable_since'])
        # Stability supports operational recovery, not proof of loop geometry.
        # Bound automatic recovery and preserve the continuous local frame.
        bounded=event['translation_jump_m']<=2. and event['angle_jump_rad']<=.7
        accepted=bounded and pending['count']>=3 and stamp-pending['stable_since']>=.5
        event.update(status='ACCEPTED_CONTINUOUS_CONTROL' if accepted else 'WAIT_STABLE_REVISION' if bounded else 'REVIEW_LARGE_REVISION',geometry_verified=False)
        self.map_revision_diagnostic=event
        if not accepted:return
        delta=compose(compose(compose(self.task_from_map,correction),inverse(old)),inverse(self.task_from_map))
        self.map_rebase=compose(delta,getattr(self,'map_rebase',(0.,0.,0.)))
        self.map_revision=getattr(self,'map_revision',0)+1
        self.correction=tuple(correction);self.graph=(stamp,wall);self.match_count+=1
        self.pending_map_revision=None

    def snapshot(self,sim_time,wall=None):
        # Freeze the map/task origin once at the declared stationary start.
        # RTAB-Map may choose a translated/rotated map origin during bootstrap.
        # Use measured SLAM only; simulator pose is never read here.
        if (getattr(self,'known_start_pending',False)
                and sim_time >= getattr(self,'known_start_not_before',0.)
                and self.config_ok and self.graph and len(self.history)>=4):
            from slam_navigation_feedback import compose,inverse,wrap
            recent=[v for v in self.history if self.history[-1][0]-v[0]<=1.]
            latest=self.history[-1]
            if len(recent)>=4 and latest[0]-recent[0][0]>=.5 and all(math.dist(v[1][:2],latest[1][:2])<.015 and abs(wrap(v[1][2]-latest[1][2]))<.01 for v in recent):
                map_pose=compose(self.correction,latest[1])
                self.task_from_map=compose(self.task_anchor,inverse(map_pose))
                self.row_alignment=dict(method='STATIONARY_SLAM_AT_SURVEYED_START',stamp=latest[0],heading_rad=self.task_anchor[2],task_from_map=self.task_from_map,task_from_odom=compose(self.task_from_map,self.correction),agreement_deg=None,frozen=True)
                self.known_start_pending=False
        out=super().snapshot(sim_time,wall)
        out['map_revision']=getattr(self,'map_revision',0)
        out['map_rebase']=getattr(self,'map_rebase',(0.,0.,0.))
        out['map_revision_review']=getattr(self,'map_revision_diagnostic',None)
        pending=getattr(self,'pending_map_revision',None)
        if pending and not self.fault:
            elapsed=sim_time-pending['start']
            out.update(valid=False,reason='MAP_REVISION_REVIEW_REQUIRED' if elapsed>15. else 'WAIT_STABLE_MAP_REVISION',recovery_wait_s=elapsed)
        if getattr(self,'known_start_pending',False):
            out.update(valid=False,reason='WAIT_STATIONARY_START_ALIGNMENT')
        out['correction_source']=getattr(self,'correction_source','MAP_GRAPH_BRIDGE')
        if out.get('valid'):
            original=list(out['position'])
            out['position'],out['age_compensation']=compensate_position(original,out['yaw'],self.history,out['odom_age_s'])
            out['uncompensated_position']=original
            out['estimated_position_stamp']=sim_time if out['age_compensation']['applied'] else self.history[-1][0]
        return out


class RosFullMapSlamFeedback(RosOnlineSlamFeedback):
    def __init__(self,node,task_anchor=(-3.,-2.15,0.),initial_row_heading=False,base_frame='base_link'):
        super().__init__(node,task_anchor,initial_row_heading,base_frame)
        self.filter=FullMapSlamFeedback(task_anchor,initial_row_heading,lidar_primary=initial_row_heading)
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        self.tf_diagnostic=dict(received=0,accepted=0,last_stamp=None,source='RTABMAP_MAP_TO_ODOM_TF')
        self.tf_publishers=[];self.next_tf_inventory=0.
        qos=QoSProfile(depth=32,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        self.tf_map_sub=node.create_subscription(TFMessage,'/tf',self._tf_map,qos)

    def _tf_map(self,msg):
        for transform in msg.transforms:
            pose=planar_map_tf(transform)
            if pose is None:continue
            self.tf_diagnostic['received']+=1
            stamp=self._stamp(transform.header)
            previous=self.filter.graph
            self.filter.graph_correction(stamp,pose)
            if self.filter.graph is not previous:
                self.filter.correction_source='RTABMAP_MAP_TO_ODOM_TF'
                self.tf_diagnostic.update(accepted=self.tf_diagnostic['accepted']+1,last_stamp=stamp)

    def snapshot(self,sim_time):
        out=super().snapshot(sim_time)
        now=time.monotonic()
        if now>=self.next_tf_inventory:
            self.tf_publishers=[dict(namespace=p.node_namespace,node=p.node_name)
                for p in self.node.get_publishers_info_by_topic('/tf')]
            self.next_tf_inventory=now+5.
        out['tf_correction']=dict(self.tf_diagnostic)
        out['tf_publishers']=self.tf_publishers
        if out.get('valid') and not self.tf_diagnostic['accepted']:
            out.update(valid=False,reason='WAIT_NATIVE_MAP_TF')
        return out
