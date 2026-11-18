"""Planar odometry + time-paired map fixes, independent of simulator truth.

The map anchor is an explicit surveyed/known-start configuration. No alignment
to the current simulator pose and no ground-truth fallback are permitted here.
"""
import math
import time
import json
from pathlib import Path
from collections import deque

def wrap(angle):
    return (angle+math.pi)%(2*math.pi)-math.pi

def compose(a,b):
    c,s=math.cos(a[2]),math.sin(a[2])
    return (a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],wrap(a[2]+b[2]))

def inverse(p):
    c,s=math.cos(p[2]),math.sin(p[2])
    return (-c*p[0]-s*p[1],s*p[0]-c*p[1],wrap(-p[2]))

def localization_config_valid(values,expected_database):
    return (len(values)==3 and values[0].type==1 and values[0].bool_value
            and values[1].type==4 and values[1].string_value=='false'
            and values[2].type==4 and Path(values[2].string_value).name==expected_database)

class SlamNavigationFeedback:
    def __init__(self,map_anchor=(0.,0.,0.),task_anchor=(-3.,-2.15,0.)):
        self.expected_start=tuple(map_anchor)
        self.start_checked=False
        self.start_rejection=None
        self.task_from_map=compose(task_anchor,inverse(map_anchor))
        self.history=deque(maxlen=200)
        self.pending=None
        self.fix=None
        self.correction=None
        self.match_count=0
        self.fault=None
        self.last_clock=None
        self.config_ok=False
        self.rejection=None

    def odometry(self,stamp,pose,covariance,wall=None):
        wall=time.monotonic() if wall is None else wall
        if not all(math.isfinite(v) for v in (*pose,*covariance)):
            self.rejection='INVALID_ODOMETRY';return
        if any(v<0 for v in covariance) or max(covariance[:2])>.0625 or covariance[2]>.03:
            self.rejection='ODOMETRY_UNCERTAIN';return
        if self.history and stamp<=self.history[-1][0]:
            return
        if self.history:
            prev=self.history[-1]
            dt=stamp-prev[0]
            if dt<1. and (math.hypot(pose[0]-prev[1][0],pose[1]-prev[1][1])>.25+.5*dt or abs(wrap(pose[2]-prev[1][2]))>.35+dt):
                self.fault='ODOMETRY_JUMP';return
        self.history.append((stamp,tuple(pose),wall))
        self.rejection=None
        self._pair()

    def localization(self,stamp,pose,covariance,wall=None):
        wall=time.monotonic() if wall is None else wall
        if not self.config_ok:return
        if not all(math.isfinite(v) for v in (*pose,*covariance)) or any(v<0 for v in covariance) or max(covariance[:2])>.0625 or covariance[2]>.03:
            self.match_count=0;self.rejection='MAP_FIX_UNCERTAIN';return
        if self.fix and stamp<=self.fix[0]:return
        self.pending=(stamp,tuple(pose),wall)
        self._pair()

    def _pair(self):
        if not self.pending or not self.history:return
        fix=self.pending
        odom=min(self.history,key=lambda row:abs(row[0]-fix[0]))
        if abs(odom[0]-fix[0])>.05:return
        if not self.start_checked:
            if math.hypot(fix[1][0]-self.expected_start[0],fix[1][1]-self.expected_start[1])>.35 or abs(wrap(fix[1][2]-self.expected_start[2]))>.35:
                self.start_rejection='KNOWN_START_POSE_MISMATCH';self.pending=None;return
            self.start_checked=True;self.start_rejection=None
        correction=compose(fix[1],inverse(odom[1]))
        if self.correction and (math.hypot(correction[0]-self.correction[0],correction[1]-self.correction[1])>.25 or abs(wrap(correction[2]-self.correction[2]))>.35):
            self.fault='MAP_CORRECTION_JUMP';self.pending=None;return
        self.correction=correction;self.fix=fix;self.pending=None
        self.match_count+=1;self.rejection=None

    def snapshot(self,sim_time,wall=None):
        wall=time.monotonic() if wall is None else wall
        if self.last_clock is not None and sim_time<self.last_clock-.05:
            self.fault='CLOCK_RESET_RESTART_REQUIRED'
        self.last_clock=sim_time
        out=dict(valid=False,reason='WAIT_SLAM_CONFIG',source='SLAM',matches=self.match_count)
        if self.fault:out['reason']=self.fault;return out
        if not self.config_ok:return out
        if not self.history:out['reason']='WAIT_ODOMETRY';return out
        odom=self.history[-1]
        if not self.fix or not self.correction or self.match_count<2:
            out['reason']=self.start_rejection or self.rejection or 'WAIT_MAP_MATCHES';return out
        out.update(odom_age_s=sim_time-odom[0],map_fix_age_s=sim_time-self.fix[0])
        if sim_time-odom[0]<-.3 or sim_time-self.fix[0]<-.3:
            out['reason']='FUTURE_TIMESTAMP';return out
        if sim_time-odom[0]>.6 or wall-odom[2]>3.:
            self.match_count=0;out['reason']='ODOMETRY_TIMEOUT';return out
        if sim_time-self.fix[0]>3. or wall-self.fix[2]>12.:
            self.match_count=0;out['reason']='MAP_MATCH_TIMEOUT';return out
        if self.rejection:out['reason']=self.rejection;return out
        pose=compose(self.task_from_map,compose(self.correction,odom[1]))
        out.update(valid=True,reason='SLAM_READY',position=[pose[0],pose[1],0.],yaw=pose[2])
        return out

class RosSlamNavigationFeedback:
    def __init__(self,node):
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rcl_interfaces.srv import GetParameters
        from rclpy.qos import qos_profile_sensor_data
        alignment=json.loads(Path('E:/AS2_Greenhouse/work/slam_navigation_alignment.json').read_text(encoding='utf-8'))
        self.filter=SlamNavigationFeedback(alignment['map_anchor_xyyaw'],alignment['task_anchor_xyyaw'])
        self.expected_database=alignment['map_database'];self.node=node
        self.GetParameters=GetParameters
        self.client=node.create_client(GetParameters,'/as2/slam/rtabmap/get_parameters')
        self.future=None;self.next_query=0.
        self.config_response=None
        self.odom_sub=node.create_subscription(Odometry,'/as2/odom',self._odom,qos_profile_sensor_data)
        self.fix_sub=node.create_subscription(PoseWithCovarianceStamped,'/as2/slam/localization_pose',self._fix,qos_profile_sensor_data)

    @staticmethod
    def _pose(p):
        q=p.orientation
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01:
            return (float('nan'),)*3
        return (p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

    @staticmethod
    def _stamp(header):
        return header.stamp.sec+header.stamp.nanosec/1e9

    def _odom(self,msg):
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':return
        self.filter.odometry(self._stamp(msg.header),self._pose(msg.pose.pose),[msg.pose.covariance[i] for i in (0,7,35)])

    def _fix(self,msg):
        if msg.header.frame_id!='map':return
        self.filter.localization(self._stamp(msg.header),self._pose(msg.pose.pose),[msg.pose.covariance[i] for i in (0,7,35)])

    def snapshot(self,sim_time):
        wall=time.monotonic()
        if self.future and self.future.done():
            try:
                values=self.future.result().values
                subscriptions={(v.node_namespace,v.node_name) for v in self.node.get_subscriptions_info_by_topic('/as2/lidar/odometry_points')}
                filtered_input_verified={('/','icp_odometry'),('/as2/slam','rtabmap')}.issubset(subscriptions)
                self.filter.config_ok=localization_config_valid(values,self.expected_database) and filtered_input_verified
                self.config_response=dict(strict_pose_publication=values[0].bool_value if len(values)>0 else None,
                                          incremental_memory=values[1].string_value if len(values)>1 else None,
                                          database=values[2].string_value if len(values)>2 else None,filtered_input_verified=filtered_input_verified)
            except Exception:
                self.filter.config_ok=False
            self.future=None;self.next_query=wall+3.
        if not self.future and wall>=self.next_query and self.client.service_is_ready():
            self.future=self.client.call_async(self.GetParameters.Request(names=['pub_loc_pose_only_when_localizing','Mem/IncrementalMemory','database_path']))
        return self.filter.snapshot(sim_time,wall)
