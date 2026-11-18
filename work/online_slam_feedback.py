"""Online mapping feedback: map-graph correction + ICP odometry, no truth fallback.
Mapping is initialized at the configured start. It does not wait for an old-map
localization match, which cannot exist while exploring a new map.
"""
import json
import math
import time
from pathlib import Path
from slam_navigation_feedback import SlamNavigationFeedback, compose, inverse, wrap


class OnlineSlamFeedback(SlamNavigationFeedback):
    def __init__(self, task_anchor=(-3.,-2.15,0.), initial_row_heading=False, lidar_primary=False):
        super().__init__(task_anchor=task_anchor)
        self.task_anchor=tuple(task_anchor)
        self.graph = None
        self.mapping_state = None
        self.last_visual_ok = None
        self.lidar_primary=bool(lidar_primary)
        self.row_heading_required=initial_row_heading
        self.row_alignment=None
        self.row_observations=[]

    def observe_initial_row(self, result, stamp, wall=None):
        if not self.row_heading_required or self.row_alignment is not None:return False
        if not result.get('valid') or not self.config_ok or not self.graph or len(self.history)<4:return False
        wall=time.monotonic() if wall is None else wall
        odom=self.history[-1]
        if abs(stamp-odom[0])>.4 or wall-odom[2]>3.:return False
        recent=[entry for entry in self.history if odom[0]-entry[0]<=1.]
        if len(recent)<4 or odom[0]-recent[0][0]<.5:return False
        if any(math.dist(entry[1][:2],odom[1][:2])>.015 or abs(wrap(entry[1][2]-odom[1][2]))>.01 for entry in recent):return False
        if self.row_observations and stamp<=self.row_observations[-1][0]:return False
        heading=float(result['body_heading_rad'])
        if not math.isfinite(heading) or abs(heading)>.2:return False
        self.row_observations.append((stamp,heading));self.row_observations=self.row_observations[-3:]
        if len(self.row_observations)<3:return False
        angles=[entry[1] for entry in self.row_observations]
        if max(angles)-min(angles)>math.radians(.25):return False
        heading=sum(angles)/len(angles)
        map_pose=compose(self.correction,odom[1])
        # Only once, at the known stationary start. Never align to simulator truth.
        self.task_from_map=compose((*self.task_anchor[:2],heading),inverse(map_pose))
        self.row_alignment=dict(method='INITIAL_TWO_MEASURED_ROWS',stamp=stamp,heading_rad=heading,task_from_map=self.task_from_map,task_from_odom=compose(self.task_from_map,self.correction),agreement_deg=result['agreement_deg'],frozen=True)
        return True

    def graph_correction(self, stamp, correction, wall=None):
        wall = time.monotonic() if wall is None else wall
        if not self.config_ok or not all(math.isfinite(v) for v in correction):
            return
        if self.graph and stamp <= self.graph[0]:
            return
        if self.correction and (math.dist(correction[:2],self.correction[:2]) > .25 or
                                abs(wrap(correction[2]-self.correction[2])) > .35):
            self.fault = 'MAP_CORRECTION_JUMP'
            return
        self.correction = tuple(correction)
        self.graph = (stamp, wall)
        self.match_count += 1

    def snapshot(self, sim_time, wall=None):
        wall = time.monotonic() if wall is None else wall
        out = dict(valid=False, reason='WAIT_FUSION_CONFIG', source='ONLINE_SLAM',
                   map_updates=self.match_count, fusion=self.mapping_state)
        if self.last_clock is not None and sim_time < self.last_clock-.05:
            self.fault = 'CLOCK_RESET_RESTART_REQUIRED'
        self.last_clock = sim_time
        if self.fault:
            out['reason'] = self.fault
            return out
        if not self.config_ok:
            return out
        if not self.history or not self.graph or self.match_count < 2:
            out['reason'] = 'WAIT_ODOMETRY_AND_MAP_GRAPH'
            return out
        odom = self.history[-1]
        odom_age, graph_age = sim_time-odom[0], sim_time-self.graph[0]
        out.update(odom_age_s=odom_age, graph_age_s=graph_age)
        if min(odom_age, graph_age) < -.3:
            out['reason'] = 'FUTURE_TIMESTAMP'
            return out
        if odom_age > .6 or wall-odom[2] > 3.:
            out['reason'] = 'ODOMETRY_TIMEOUT'
            return out
        if graph_age > 3. or wall-self.graph[1] > 12.:
            out['reason'] = 'MAP_GRAPH_TIMEOUT'
            return out
        state = self.mapping_state
        if state and state.get('visual_words',0)>=10:
            if self.last_visual_ok is None or state['stamp']>self.last_visual_ok[0]:
                self.last_visual_ok=(state['stamp'],state['wall'])
        if (not state or state['ref_id'] <= 0 or not (-.3 <= sim_time-state['stamp'] <= 3.) or wall-state['wall'] > 12. or
            self.last_visual_ok is None or (not self.lidar_primary and (sim_time-self.last_visual_ok[0]>3. or wall-self.last_visual_ok[1]>12.))):
            out['reason'] = 'WAIT_VISUAL_FEATURES'
            return out
        out['visual_quality']='CURRENT' if state['visual_words']>=10 else ('LIDAR_DOMINANT_RGB_WEAK' if self.lidar_primary else 'BRIEF_WEAK_FRAME')
        out['initial_alignment']=self.row_alignment
        if self.row_heading_required and self.row_alignment is None:
            out['reason']='WAIT_INITIAL_ROW_HEADING'
            return out
        pose = compose(self.task_from_map, compose(self.correction, odom[1]))
        if not self.start_checked:
            if math.dist(pose[:2], self.task_anchor[:2]) > .35 or abs(wrap(pose[2]-self.task_anchor[2])) > .35:
                out['reason'] = 'KNOWN_START_POSE_MISMATCH'
                return out
            self.start_checked = True
        out.update(valid=True, reason='ONLINE_SLAM_READY', position=[pose[0],pose[1],0.], yaw=pose[2])
        return out


class RosOnlineSlamFeedback:
    def __init__(self, node, task_anchor=(-3.,-2.15,0.), initial_row_heading=False,base_frame='base_link'):
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TransformStamped
        from std_msgs.msg import String
        from rcl_interfaces.srv import GetParameters
        from rclpy.qos import qos_profile_sensor_data
        from slam_navigation_feedback import RosSlamNavigationFeedback
        self.filter = OnlineSlamFeedback(task_anchor,initial_row_heading,lidar_primary=initial_row_heading)
        self.node = node
        self.base_frame=base_frame
        self.GetParameters = GetParameters
        self._pose = RosSlamNavigationFeedback._pose
        self._stamp = RosSlamNavigationFeedback._stamp
        self.client = node.create_client(GetParameters, '/as2/slam/rtabmap/get_parameters')
        self.future = None
        self.next_query = 0.
        self.config_response = None
        self.subs = [node.create_subscription(Odometry, '/as2/odom', self._odom, qos_profile_sensor_data),
                     node.create_subscription(TransformStamped, '/as2/slam/map_correction', self._graph, qos_profile_sensor_data),
                     node.create_subscription(String, '/as2/slam/online_state', self._state, qos_profile_sensor_data)]

        if initial_row_heading:
            from sensor_msgs.msg import PointCloud2
            self.subs.append(node.create_subscription(PointCloud2,'/as2/lidar/points',self._initial_cloud,qos_profile_sensor_data))

    def _initial_cloud(self,msg):
        if self.filter.row_alignment is not None or not self.filter.history:return
        if msg.header.frame_id!='as2_lidar' or msg.point_step!=12 or msg.is_bigendian:return
        import numpy as np
        from row_heading_alignment import observe_row_heading
        points=np.frombuffer(bytes(msg.data),dtype='<f4').reshape(-1,3)
        if not np.isfinite(points).all():return
        self.filter.observe_initial_row(observe_row_heading(points),self._stamp(msg.header))

    def _odom(self, msg):
        if msg.header.frame_id == 'odom' and msg.child_frame_id == self.base_frame:
            self.filter.odometry(self._stamp(msg.header), self._pose(msg.pose.pose),
                                 [msg.pose.covariance[i] for i in (0,7,35)])

    def _graph(self, msg):
        from geometry_msgs.msg import Pose
        if msg.header.frame_id != 'map' or msg.child_frame_id != 'odom':
            return
        p = Pose()
        p.position.x = msg.transform.translation.x
        p.position.y = msg.transform.translation.y
        p.orientation = msg.transform.rotation
        self.filter.graph_correction(self._stamp(msg.header), self._pose(p))

    def _state(self, msg):
        try:
            state = json.loads(msg.data)
            if all(k in state for k in ('stamp','ref_id','visual_words','loop_closure_id','proximity_detection_id')):
                state['wall'] = time.monotonic()
                self.filter.mapping_state = state
        except (ValueError, TypeError):
            pass

    def snapshot(self, sim_time):
        wall = time.monotonic()
        if self.future and self.future.done():
            try:
                v = self.future.result().values
                subs = {(s.node_namespace,s.node_name) for s in self.node.get_subscriptions_info_by_topic('/as2/lidar/odometry_points')}
                map_subs = {(s.node_namespace,s.node_name) for s in self.node.get_subscriptions_info_by_topic('/as2/lidar/points')}
                rgb_subs = {(s.node_namespace,s.node_name) for s in self.node.get_subscriptions_info_by_topic('/as2/camera/image_raw')}
                ok = (len(v)==5 and v[0].type==4 and v[0].string_value=='true' and
                      v[1].type==1 and v[1].bool_value and v[2].type==1 and v[2].bool_value and
                      v[3].type==4 and Path(v[3].string_value).name.startswith('greenhouse_fusion_') and
                      v[4].type==1 and not v[4].bool_value and
                      ('/','icp_odometry') in subs and ('/as2/slam','rtabmap') in map_subs and
                      ('/as2/slam','rtabmap') in rgb_subs)
                self.filter.config_ok = ok
                self.config_response = dict(valid=ok)
            except Exception as exc:
                self.filter.config_ok = False
                self.config_response = dict(valid=False, error=str(exc))
            self.future = None
            self.next_query = wall+3.
        if not self.future and wall >= self.next_query and self.client.service_is_ready():
            names = ['Mem/IncrementalMemory','subscribe_rgb','gen_depth','database_path','pub_loc_pose_only_when_localizing']
            self.future = self.client.call_async(self.GetParameters.Request(names=names))
        return self.filter.snapshot(sim_time, wall)
