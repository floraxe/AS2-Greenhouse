"""Photo-guided greenhouse: assigned aisle, observed end bay, physical turn, forward return.
Native sensors; optional --slam-control consumes external estimated pose, never truth fallback.
"""
from pathlib import Path
import argparse,os,tempfile,json,time,traceback,math
import as2_json as json
ROOT=Path('E:/AS2_Greenhouse');OUT=ROOT/'outputs/lab_greenhouse';OUT.mkdir(exist_ok=True)
os.environ['WARP_CACHE_PATH']=str(ROOT/'runtime/warp_cache');tempfile.tempdir=str(ROOT/'runtime/tmp')
p=argparse.ArgumentParser();p.add_argument('--test',action='store_true');p.add_argument('--ros',action='store_true');p.add_argument('--ros-domain-id',type=int,default=0);p.add_argument('--ros-self-check',action='store_true');p.add_argument('--slam-control',action='store_true');p.add_argument('--odom-benchmark',action='store_true');p.add_argument('--record-lidar',action='store_true');p.add_argument('--lidar-detail',action='store_true');p.add_argument('--odometry-filter',action='store_true');p.add_argument('--record-structure-only',action='store_true');p.add_argument('--benchmark-distance',type=float,default=1.2);p.add_argument('--odom-error-limit',type=float,default=0.);p.add_argument('--obstacle-test',action='store_true');p.add_argument('--sensor-check',action='store_true');p.add_argument('--preview',action='store_true');p.add_argument('--lean',action='store_true');p.add_argument('--output-dir',type=Path);p.add_argument('--cruise-speed',type=float,choices=[.075,.10,.12]);p.add_argument('--fixture-file',type=Path);p.add_argument('--home-approach-distance',type=float,default=1.25);p.add_argument('--terrain-profile',choices=['flat','rough_mild','rough_moderate'],default='flat');p.add_argument('--terrain-seed',type=int,default=0);p.add_argument('--limit',type=float,default=300);args,_=p.parse_known_args()
# New mode is explicit, leaving the accepted saved-map workflow unchanged.
if '--fast-runtime' in os.sys.argv:p.error('Low-density mode was removed; use the full-density launcher.')
args.online_fusion = '--online-fusion' in os.sys.argv
args.level_odometry = '--level-odometry' in os.sys.argv
args.route23 = True  # Dedicated full-map runner uses the validated no-search pipeline.
args.complete_loop_test = '--complete-loop-test' in os.sys.argv
terrain_info={'profile':'flat','source':'UNCHANGED_FLAT_GROUND_BASELINE'}
auto_ready_since=None;auto_start_used=False
args.headland_test = '--headland-test' in os.sys.argv
args.two_aisle_loop = '--two-aisle-loop' in os.sys.argv
args.home_approach_test = '--home-approach-test' in os.sys.argv
home_spawn=(-3.-args.home_approach_distance,-.825)
if not .5<=args.home_approach_distance<=10.:p.error('Home approach distance must be between 0.5 and 10 metres')
selected_route=(2,3) if args.two_aisle_loop else (2,3,4,5)
excluded_route=[a for a in range(1,7) if a not in selected_route]
if args.two_aisle_loop and args.headland_test:p.error('Two-aisle loop and endpoint fixture are separate modes')
args.near_end_test = '--near-end-test' in os.sys.argv
args.endpoint_drift_test = '--endpoint-drift-test' in os.sys.argv
if args.endpoint_drift_test and not args.headland_test:p.error('Drift injection is restricted to the endpoint test')
test_spawn=(-19.8,2.875) if args.near_end_test else (19.8,-.825)
test_yaw=math.pi if args.near_end_test else 0.
args.home_actuator_test = '--home-actuator-test' in os.sys.argv
args.micro_test = '--micro-recovery-test' in os.sys.argv or args.headland_test or args.home_actuator_test
args.full_map_fixture = '--full-map-stop-fixture' in os.sys.argv
args.micro_test = args.micro_test or args.full_map_fixture
if args.home_approach_test and args.micro_test:p.error('Home approach uses a stationary known start, not a teleporting fixture')
if args.online_fusion:
 args.slam_control=True;args.odometry_filter=True
if args.output_dir:
 OUT=args.output_dir;OUT.mkdir(parents=True,exist_ok=True)
if args.slam_control and not args.ros:p.error('--slam-control requires --ros')
if args.odom_benchmark and args.slam_control:p.error('Benchmark stimulus is separate from SLAM navigation')
ros_source=None
if args.ros:
 from full_map_fast_ros_sensors import prepare_ros,SensorPublisher
 prepare_ros(args.ros_domain_id)
print('LAB_BOOT',flush=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':args.test or args.sensor_check or args.preview,'width':1280,'height':720,'multi_gpu':False},experience=str(ROOT/'work/as2_ros_minimal.kit') if args.ros or args.lean else '')
try:
 import numpy as np,omni.ui as ui,omni.usd
 from pxr import UsdGeom,UsdPhysics,Gf,Vt
 from isaacsim.core.experimental.utils.impl.app import enable_extension
 enable_extension('isaacsim.sensors.experimental.physics')
 from isaacsim.sensors.experimental.physics import IMU,IMUSensor
 from isaacsim.core.simulation_manager import SimulationManager as SM
 from as2_integrated_scene import setup
 from as2_command_gait import CommandGait
 from aisle_mission import AisleMission
 from turn_space_guard import evaluate_turn_space
 from full_map_leaf_scene import build,BASE,START,END
 if args.route23:
  from full_map_mission import FullMapMission as FixedRouteMission
  START23=test_spawn if args.headland_test and args.online_fusion else (-3.,-.825)
  START=START23
 from showcase_visuals import camera
 import omni.replicator.core as rep
 imu_author=None
 def scene_hook(stage):
  global imu_author,layout
  layout=build(stage)
  if args.terrain_profile!='flat':
   from synthetic_terrain import install as install_synthetic_terrain
   global terrain_info
   terrain_info=install_synthetic_terrain(stage,args.terrain_profile,args.terrain_seed)
  layout['terrain']=terrain_info
  if args.route23:
   ops=UsdGeom.Xformable(stage.GetPrimAtPath('/World/AS2')).GetOrderedXformOps()
   ops[0].Set(Gf.Vec3d(*( (*test_spawn,.4) if args.headland_test else (2.7705,-.80309,.4) if args.micro_test else (*( home_spawn if args.home_approach_test else START),.4))))
   if args.near_end_test:
    UsdGeom.Xformable(stage.GetPrimAtPath('/World/AS2')).AddRotateZOp().Set(180.)
   camera(stage,'/World/Route23View',(-42.,13.,38.),(0.,1.4,.35),16)
   layout.update(route_start=list(START),selected_aisles_from_right=list(selected_route),excluded_aisles_from_right=excluded_route,mode='selected aisle serpentine then bottom headland to aisle2 and home; no A*')
   (OUT/'full_map_layout.json').write_text(json.dumps(layout,indent=2),encoding='utf-8')
  imu_author=IMU.create(BASE+'/LabIMU',translations=[[0,0,0]],orientations=[[1,0,0,0]],linear_acceleration_filter_size=1,angular_velocity_filter_size=1)
 art,names,q,stage=setup(app,True,True,scene_hook=scene_hook);gait=CommandGait(names);imu=IMUSensor(imu_author);mission=FixedRouteMission(cruise_speed=args.cruise_speed,selected_aisles=selected_route) if args.route23 else AisleMission(START)
 if args.home_approach_test:
  mission.index=next(i for i,v in enumerate(mission.stages) if v[0]=='CLOSE_START');mission.phase='CLOSE_START'
 # Establish the simulation initial condition BEFORE publishing sensors.
 # Articulation joint initialization and the first fall can displace the base.
 # Settle those constraints first, then place the standing robot at the declared
 # surveyed pose once. This is fixture setup, never a navigation correction.
 startup_time=0.;startup_initialization=None
 if (args.online_fusion or args.home_approach_test or args.terrain_profile!='flat') and not args.micro_test:
  for _ in range(720):
   art.set_dof_position_targets(q[None]);SM.step()
  startup_time=float(imu.get_data()['time'])
  initial_xy=home_spawn if args.home_approach_test else START
  initial_ground_z=0.
  if args.terrain_profile!='flat':
   from synthetic_terrain import height as synthetic_ground_height
   initial_ground_z=synthetic_ground_height(initial_xy[0],initial_xy[1],args.terrain_profile,args.terrain_seed)
  initial_base_z=.296+initial_ground_z
  art.set_world_poses(positions=[[*initial_xy,initial_base_z]],orientations=[[1.,0.,0.,0.]])
  art.set_velocities(linear_velocities=np.zeros((1,3),np.float32),angular_velocities=np.zeros((1,3),np.float32))
  gait.reset()
  startup_initialization=dict(method='PRE_SENSOR_STANDING_PLACEMENT',warmup_steps=720,
   sensor_start_time_s=startup_time,declared_position=[*initial_xy,initial_base_z],
   measured_terrain_height_m=initial_ground_z,
   declared_yaw_rad=0.,navigation_truth_feedback=False,placement_before_sensor_publication=True)
  (OUT/'startup_initialization.json').write_text(json.dumps(startup_initialization,indent=2),encoding='utf-8')
 if args.headland_test:
  import copy
  short_config=copy.deepcopy(mission.config);short_config['headland_x_m']={'far':21.25,'entry':-21.25}
  short_config['start_xy_m']=[-3.,-.825]
  mission=FixedRouteMission(config=short_config,cruise_speed=args.cruise_speed)
  if args.near_end_test:
   mission.index=12;mission.phase=mission.stages[12][0]
 from full_map_contact_audit import ContactAudit
 contact_audit=ContactAudit(OUT,layout['bed_rectangles_xy_m'])
 camera_path=BASE+'/LabRGBCamera';camera(stage,camera_path,(.34,.058,.10),(1.34,.058,.045),12)
 stage.GetPrimAtPath(camera_path).SetCustomData({'model':'ideal pinhole for preflight, HFOV90deg, not calibrated AS2 optical camera','resolution':[640,360]})
 product=rep.create.render_product(camera_path,(640,360));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(product)
 body_to_camera=np.linalg.inv(np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(camera_path)).GetLocalTransformation()))
 depth_ann=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');depth_ann.attach(product);depth_data=None;frame_matrix=None
 if args.ros:ros_source=SensorPublisher(UsdGeom.Xformable(stage.GetPrimAtPath(camera_path)).GetLocalTransformation(),args.ros_self_check,args.odometry_filter,level_odometry=args.level_odometry)
 slam_feedback=None
 if args.slam_control:
  if args.online_fusion:
   from full_map_online_feedback import RosFullMapSlamFeedback as RosOnlineSlamFeedback
   # The first route direction is surveyed (+X). Requiring a fresh stationary
   # row-heading estimate created a startup interlock: motion waited for the
   # estimate while the estimate waited for motion/edge agreement. Use the
   # declared known start pose and keep lidar-primary weak-visual operation.
   slam_feedback=RosOnlineSlamFeedback(ros_source.node,(*( home_spawn if args.home_approach_test else START),test_yaw if args.headland_test else 0.),initial_row_heading=False,base_frame='base_stabilized' if args.level_odometry else 'base_link')
   if args.route23:
    slam_feedback.filter.lidar_primary=True
    slam_feedback.filter.known_start_pending=True
    slam_feedback.filter.row_alignment=dict(method='SURVEYED_START_PENDING',stamp=0.,heading_rad=0.,task_from_map=slam_feedback.filter.task_from_map,task_from_odom=slam_feedback.filter.task_from_map,agreement_deg=None,frozen=False)
   if args.route23:
    from full_map_route_map_monitor import FullMapRouteMapMonitor as RouteMapMonitor
    online_planner=RouteMapMonitor(ros_source.node,START)
   else:
    from online_grid_planner import RosGridPlanner
    from online_aisle_mission import OnlineAisleMission
    online_planner=RosGridPlanner(ros_source.node)
    mission=OnlineAisleMission(START,online_planner)
  else:
   from slam_navigation_feedback import RosSlamNavigationFeedback
   slam_feedback=RosSlamNavigationFeedback(ros_source.node)
 camera_meta={'width':640,'height':360,'fx':320.,'fy':320.,'cx':320.,'cy':180.,'model':'ideal pinhole','not_calibrated_to_hardware':True}
 from omni.physx import get_physx_scene_query_interface
 query=get_physx_scene_query_interface()
 directions=np.array([[math.cos(el)*math.cos(az),math.cos(el)*math.sin(az),math.sin(el)] for az in np.linspace(-math.pi,math.pi,720,endpoint=False) for el in np.radians(np.arange(-40,40.1,2.5))])
 lidar_visual=UsdGeom.Points.Define(stage,'/World/LabPointCloud');lidar_visual.CreateWidthsAttr([.012]);lidar_visual.SetWidthsInterpolation(UsdGeom.Tokens.constant)
 run_wall_start=time.perf_counter();lidar_visible=False;cloud=np.empty((0,3));body_cloud=np.empty((0,3));guard_points=np.empty((0,4));measured_surface_height=np.empty(0);measured_body_height=None;cloud_time=-1.;last_rgb_time=-1.;rgb_data=None
 imu_count=0;last_imu_time=-1;imu_frames=[];times={'imu':[],'lidar':[],'rgb':[]};rows=[];t=startup_time;state='READY';running=args.test;phase='OUT';wait_start=None;fault=None;attention_reason=None;reset_requested=False;view_mode='aisle';roof=False;finished=False
 if args.online_fusion:
  # Micro fixtures are placed at t=3; do not anchor to their pre-placement pose.
  slam_feedback.filter.known_start_not_before=5. if args.micro_test else startup_time+3.
 recorded_scans=[];recorded_scan_meta=[];benchmark_origin=None;benchmark_done_time=None
 milestones={};dt=1/240;last_status=-1.;distance=0.;prev=None;camera_count=0
 from full_map_loop_acceptance import LoopAcceptance
 loop_acceptance=LoopAcceptance();navigation_loop_events=0
 from full_map_row_pose import RowPropagationClock
 row_propagation_clock=RowPropagationClock()
 recovery_started=None;recovery_diagnostic={};row_preview_diagnostic={};row_clearance={}
 fixture={};fixture_finished_at=None;micro_initialized=False;micro_release=0.;progress_watch=None;row_target_memory=None;row_target_phase=None;last_row_valid_time=-1e9;row_plan_scan=None;row_plan_cache={};near_field_memory=__import__('full_map_near_field_memory').NearFieldMemory();row_tracker=__import__('full_map_row_tracking').RowObservationTracker();foliage_memory=__import__('full_map_visual_foliage').VisualFoliageMemory();motion_monitor=__import__('full_map_progress_monitor').MotionProgressMonitor()
 local_row_observation={};raw_row_observation={};ray_surface_counts={};combined_row_scan=None;combined_row_cache={}
 from omni.kit.viewport.utility import get_active_viewport
 viewport=get_active_viewport()
 def set_view(mode):
  global view_mode
  view_mode=mode
  path={'follow':'/World/FollowView','overview':('/World/Route23View' if args.route23 else '/World/LabOverview'),'aisle':'/World/LabAisle','interior':'/World/LabInterior','camera':camera_path}.get(mode)
  if viewport and path:viewport.camera_path=path
 def start():
  global running,attention_reason
  if not finished and not fault:
   attention_reason=None;motion_monitor.reset();running=True
 def pause():
  global running
  running=False
 def reset():
  global reset_requested
  reset_requested=True
 def points():
  global lidar_visible
  lidar_visible=not lidar_visible
 def roof_toggle():
  global roof
  roof=not roof;stage.GetPrimAtPath('/World/RoofFilm').GetAttribute('visibility').Set('inherited' if roof else 'invisible')
 def pose():
  pp,qq=art.get_world_poses();pos=pp.numpy()[0];w,x,y,z=qq.numpy()[0]
  yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y));pitch=math.asin(np.clip(2*(w*y-z*x),-1,1));return pos,yaw,roll,pitch
 def scan():
  global cloud,cloud_time,body_cloud,local_row_observation,raw_row_observation,ray_surface_counts,measured_surface_height,measured_body_height
  matrix=np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(BASE)).ComputeLocalToWorldTransform(0));origin=np.array([.30,0,.32])@matrix[:3,:3]+matrix[3,:3];pts=[];ray_surface_counts={'foliage':0,'stems':0,'beds':0,'other':0};side_compare={} if args.full_map_fixture and t>3. and not (OUT/'ray_sidedness.json').exists() else None
  for d in directions@matrix[:3,:3]:
   hit=query.raycast_closest(tuple(origin),tuple(d),9.,True)
   if side_compare is not None:
    old=query.raycast_closest(tuple(origin),tuple(d),9.)
    if hit['hit'] and (not old['hit'] or old['distance']-hit['distance']>.01):
     name=str(hit.get('collision','unknown'));side_compare[name]=side_compare.get(name,0)+1
   if hit['hit'] and hit['distance']>.05 and not str(hit.get('collision','')).startswith('/World/AS2'):
    hp=hit['position'];pts.append((hp[0],hp[1],hp[2]));surface=str(hit.get('collision',''));category='foliage' if surface.endswith('/Foliage') else 'stems' if surface.endswith('/Stems') else 'beds' if '/Beds/' in surface else 'other';ray_surface_counts[category]+=1
  if side_compare is not None:(OUT/'ray_sidedness.json').write_text(json.dumps(side_compare,indent=2),encoding='utf-8')
  cloud=np.asarray(pts,dtype=float).reshape(-1,3);cloud_time=t;times['lidar'].append(t)
  inv=np.linalg.inv(matrix);body_cloud=cloud@inv[:3,:3]+inv[3,:3]
  if args.route23:
   from full_map_fast_row_observer import estimate_row_center
   local_row_observation=estimate_row_center(body_cloud-np.array((.3,0,.32)));raw_row_observation=dict(local_row_observation)
   from floor_stabilization import floor_transform
   try:
    observed_floor=floor_transform(raw_row_observation['floor_plane']);measured_surface_height=(body_cloud@observed_floor[:3,:3].T+observed_floor[:3,3])[:,2];measured_body_height=float(observed_floor[2,3])
   except (KeyError,ValueError):measured_surface_height=np.full(len(body_cloud),np.nan);measured_body_height=None
   if args.headland_test:
    from full_map_lidar_structure_filter import filter_lidar_structure
    near,near_info=filter_lidar_structure(body_cloud-np.array((.3,0,.32)),context_range=2.)
    enriched,enriched_info=filter_lidar_structure(body_cloud-np.array((.3,0,.32)))
    (OUT/'headland_filter_diagnostic.json').write_text(json.dumps(dict(phase=mission.phase,near=near_info,enriched=enriched_info),indent=2),encoding='utf-8')
    np.savez_compressed(OUT/('cloud_'+str(mission.index)+'.npz'),points=body_cloud-np.array((.3,0,.32)))
   if args.micro_test:np.savez_compressed(OUT/'fixture_lidar_latest.npz',points=body_cloud-np.array((.3,0,.32)))
  if args.record_lidar:
   pp,qq=art.get_world_poses();p_ref=pp.numpy()[0];q_ref=qq.numpy()[0]
   matrix_quat=Gf.Matrix4d(matrix.tolist()).ExtractRotationQuat();iv=matrix_quat.GetImaginary();mw=matrix_quat.GetReal();mx,my,mz=iv
   matrix_yaw=math.atan2(2*(mw*mz+mx*my),1-2*(my*my+mz*mz))
   rw,rx,ry,rz=q_ref;reference_yaw=math.atan2(2*(rw*rz+rx*ry),1-2*(ry*ry+rz*rz))
   recording=np.asarray(body_cloud-np.array([.30,0,.32]),dtype=np.float32)
   if args.record_structure_only:
    from full_map_lidar_structure_filter import filter_lidar_structure
    recording,record_filter_info=filter_lidar_structure(recording)
    if not record_filter_info['valid']:raise RuntimeError(str(record_filter_info))
   recorded_scans.append(recording)
   recorded_scan_meta.append(dict(sensor_time=max(0.,last_imu_time),loop_time=t,physics_position=p_ref.tolist(),physics_quaternion_wxyz=q_ref.tolist(),usd_position=matrix[3,:3].tolist(),usd_yaw=matrix_yaw,physics_yaw=reference_yaw,translation_difference_m=float(np.linalg.norm(matrix[3,:3]-p_ref)),yaw_difference_rad=float((matrix_yaw-reference_yaw+math.pi)%(2*math.pi)-math.pi)))
  if ros_source:ros_source.cloud(cloud,matrix,max(0.,last_imu_time))
  lidar_visual.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(np.asarray(cloud,dtype=np.float32)));lidar_visual.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant).Set([[.18,.53,.40]]);lidar_visual.GetVisibilityAttr().Set('inherited' if lidar_visible else 'invisible')
 def guard(sign,pos,yaw):
  if t-cloud_time>.4 or len(cloud)<100:return False,'STALE_OR_EMPTY_CLOUD',0
  if measured_body_height is None or len(measured_surface_height)!=len(cloud):return False,'MEASURED_FLOOR_UNAVAILABLE',0
  # Actual point support in the direction of travel is mandatory, including reverse.
  if args.slam_control:
   # Local occupancy stays in the observed body frame, independent of map drift.
   # Ground height is inferred from the measured lidar floor plane.
   forward=body_cloud[:,0]*sign;side=body_cloud[:,1]
  else:
   dx=cloud[:,:2]-pos[:2];c,s=math.cos(yaw),math.sin(yaw);forward=(dx[:,0]*c+dx[:,1]*s)*sign;side=-dx[:,0]*s+dx[:,1]*c
  ground=(measured_surface_height<.025)&(forward>.25)&(forward<2)&(abs(side)<.6)
  if np.count_nonzero(ground)<8:return False,'UNOBSERVED_DIRECTION',int(ground.sum())
  hit=(measured_surface_height>.025)&(measured_surface_height<measured_body_height+.5)&(forward>.25)&(forward<.9)&(abs(side)<.285)
  if np.count_nonzero(hit)>=3:return False,'OBSTACLE_IN_TRAVEL_DIRECTION',int(ground.sum())
  return True,'CLEAR',int(ground.sum())
 turn_diagnostic={}
 def turn_space(pos):
  global turn_diagnostic
  from full_map_turn_guard import evaluate_turn_sweep
  stage_name,_,target_heading=mission.stages[min(mission.index,len(mission.stages)-1)]
  if 'TURN' not in stage_name:
   turn_diagnostic=dict(safe=True,reason='NOT_A_TURN_STAGE');return True
  delta=(target_heading-nav_yaw+math.pi)%(2*math.pi)-math.pi
  turn_diagnostic=evaluate_turn_sweep(body_cloud,raw_row_observation.get('floor_plane'),t-cloud_time,delta)
  return turn_diagnostic['safe']
 person_path='/World/DemoPerson';person_origin=None;person_created=None;person_stop_time=None;person_event=False
 def remove_person():
  global person_created,person_origin
  if stage.GetPrimAtPath(person_path):stage.RemovePrim(person_path)
  person_created=None;person_origin=None
 def add_person():
  global person_origin,person_created
  remove_person();pp,heading,_,_=pose();person_origin=np.array([pp[0]+1.45*math.cos(heading),pp[1]+1.45*math.sin(heading),0.]);person_created=t
  group=UsdGeom.Xform.Define(stage,person_path);group.AddTranslateOp().Set(Gf.Vec3d(*person_origin))
  for name,z,r,h in [('Body',1.05,.20,.54),('LeftLeg',.36,.07,.50),('RightLeg',.36,.07,.50),('Head',1.62,.11,.02)]:
   c=UsdGeom.Capsule.Define(stage,person_path+'/'+name);c.CreateRadiusAttr(r);c.CreateHeightAttr(h);c.CreateAxisAttr('Z');c.AddTranslateOp().Set(Gf.Vec3d(0,-.105 if name=='LeftLeg' else .105 if name=='RightLeg' else 0,z));c.CreateDisplayColorAttr([Gf.Vec3f(.18,.23,.30) if name!='Head' else Gf.Vec3f(.5,.36,.26)]);UsdPhysics.CollisionAPI.Apply(c.GetPrim())
 def update_person():
  if person_created is not None:
   shift=min(.15,max(0,t-person_created)*.06);p=person_origin.copy();p[0]-=shift
   UsdGeom.Xformable(stage.GetPrimAtPath(person_path)).GetOrderedXformOps()[0].Set(Gf.Vec3d(*p))
 def snapshot(name,path):
  rp=rep.create.render_product(path,(1280,720));ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach(rp)
  for _ in range(3):rep.orchestrator.step(delta_time=0,rt_subframes=2)
  a=np.asarray(ann.get_data())
  if a.ndim!=3:raise RuntimeError('Empty rendered preview')
  from PIL import Image
  Image.fromarray(a[:,:,:3]).save(OUT/name);ann.detach(rp);rp.destroy()
 def capture_now():
  snapshot('当前场景预览.png',{'overview':('/World/Route23View' if args.route23 else '/World/LabOverview'),'interior':'/World/LabInterior','aisle':'/World/LabAisle','camera':camera_path}.get(view_mode,'/World/LabOverview'))
 label=None
 if not(args.test or args.sensor_check or args.preview):
  window=ui.Window(('AS2 | TWO AISLE SLAM LOOP 2 > 3 > HOME' if args.two_aisle_loop else 'AS2 | SLAM AISLES 2 TO 5 + BOTTOM RETURN') if args.online_fusion else 'AS2 | LAB GREENHOUSE',width=480,height=510,position_x=16,position_y=48,visible=True,flags=ui.WINDOW_FLAGS_NO_CLOSE)
  with window.frame:
   with ui.VStack(spacing=5):
    label=ui.Label('Ready',height=180,word_wrap=True)
    with ui.HStack(height=30):ui.Button('Start / Resume',clicked_fn=start);ui.Button('Pause',clicked_fn=pause);ui.Button('Reset (restart required)' if args.slam_control else 'Reset',clicked_fn=reset,enabled=not args.slam_control)
    with ui.HStack(height=30):ui.Button('Follow robot',clicked_fn=lambda:set_view('follow'));ui.Button('Overview',clicked_fn=lambda:set_view('overview'))
    with ui.HStack(height=30):ui.Button('Interior',clicked_fn=lambda:set_view('interior'));ui.Button('RGB camera',clicked_fn=lambda:set_view('camera'))
    with ui.HStack(height=30):ui.Button('Points on/off',clicked_fn=points);ui.Button('Roof on/off',clicked_fn=roof_toggle)
    ui.Button('Save current image',clicked_fn=capture_now,height=28)
    ui.Label(('2 OUT > 3 IN > bottom transfer > 2 HOME. Only aisles 2 and 3.' if args.two_aisle_loop else '2 OUT > 3 IN > 4 OUT > 5 IN > bottom transfer > 2 HOME. Aisles 1 and 6 excluded.') if args.route23 else 'Same aisle only. Turn only inside verified end bay.',word_wrap=True,height=43)
    with ui.HStack(height=28):ui.Button('Person ahead',clicked_fn=add_person);ui.Button('Remove obstacle',clicked_fn=remove_person)
 camera(stage,'/World/FollowView',(START[0]-2.,START[1]-1.,7.),(START[0],START[1],.42),24)
 set_view('follow' if args.route23 else 'aisle')
 stage.GetRootLayer().Export(str(OUT/'当前温室场景.usda'))
 while app.is_running():
  if (OUT/'stop_requested.txt').exists():
   running=False;state='OPERATOR_STOP_SAVED';break
  if reset_requested:
   art.set_world_poses(positions=[[*START,.4]],orientations=[[1,0,0,0]]);art.set_velocities(linear_velocities=np.zeros((1,3),np.float32),angular_velocities=np.zeros((1,3),np.float32));art.set_dof_positions(q[None]);art.set_dof_velocities(np.zeros((1,12),np.float32));gait.reset();running=False;finished=False;phase='OUT';state='READY';fault=None;reset_requested=False;milestones={};wait_start=None;cloud_time=-1.;mission.reset();remove_person()
  update_person()
  if ros_source:ros_source.pump()
  nav_status=slam_feedback.snapshot(max(0.,last_imu_time)) if slam_feedback else {'valid':True,'source':'REFERENCE','reason':'SIMULATOR_REFERENCE'}
  row_frame_time,row_clock_diagnostic=row_propagation_clock.observe(t,nav_status.get('position'),nav_status.get('yaw'),nav_status.get('valid',False),rows[-1]['command'] if rows else {})
  revision_review=nav_status.get('map_revision_review')
  if revision_review and revision_review.get('stamp')!=locals().get('last_revision_log_stamp'):
   with (OUT/'map_revision_events.jsonl').open('a',encoding='utf-8') as revision_stream:
    revision_stream.write(json.dumps(revision_review)+'\n')
   last_revision_log_stamp=revision_review['stamp']
  if args.complete_loop_test and not auto_start_used:
   if nav_status.get('valid'):
    if auto_ready_since is None:auto_ready_since=t
    if t-auto_ready_since>=1.:running=True;auto_start_used=True
   else:auto_ready_since=None
  pos,yaw,roll,pitch=pose();sign=1;vx=vy=omega=0.;stop=True;reason='PAUSED';support=0
  if view_mode=='follow':
   camera(stage,'/World/FollowView',(float(pos[0])-2.,float(pos[1])-1.,7.),(float(pos[0]),float(pos[1]),float(pos[2])+.12),24)
  speed_pose=nav_status.get('position') if args.slam_control else pos
  cruise_speed=mission.tracking_speed(speed_pose)
  sweep_horizon=min(.6,.045/cruise_speed)
  if len(times['lidar'])==0 or t-times['lidar'][-1]>=.199:scan()
  if len(times['rgb'])==0 or t-times['rgb'][-1]>=.199:
   rep.orchestrator.step(delta_time=0,rt_subframes=1);a=np.asarray(rgb.get_data())
   if a.ndim==3:
    rgb_data=a[:,:,:3].copy();depth_data=np.asarray(depth_ann.get_data()).copy();frame_matrix=np.asarray(UsdGeom.Xformable(stage.GetPrimAtPath(camera_path)).ComputeLocalToWorldTransform(0));last_rgb_time=t;times['rgb'].append(t);camera_count+=1
    if ros_source:ros_source.image(rgb_data,max(0.,last_imu_time))
  if args.test and args.sensor_check and t>=19 and 'preflight_done' not in milestones:
   art.set_world_poses(positions=[[*START,.4]],orientations=[[1,0,0,0]]);art.set_velocities(linear_velocities=np.zeros((1,3),np.float32),angular_velocities=np.zeros((1,3),np.float32));art.set_dof_positions(q[None]);art.set_dof_velocities(np.zeros((1,12),np.float32));gait.reset();prev=None;milestones['preflight_done']=t;pos,yaw,roll,pitch=pose()
  monitored_odom=ros_source.latest_odometry if ros_source else None
  odom_error=float(np.linalg.norm(np.asarray(monitored_odom['position'][:2])+START-pos[:2])) if monitored_odom else None
  if args.route23 and args.online_fusion and monitored_odom and slam_feedback.filter.row_alignment:
   from slam_navigation_feedback import compose
   odom_task=compose(slam_feedback.filter.row_alignment['task_from_odom'],(*monitored_odom['position'][:2],monitored_odom['yaw']))
   odom_error=float(np.linalg.norm(np.asarray(odom_task[:2])-pos[:2]))
  nav_pos=np.array(nav_status['position']) if args.slam_control and nav_status['valid'] else None
  nav_yaw=nav_status.get('yaw') if args.slam_control else yaw
  if args.micro_test and t>=3. and not micro_initialized:
   fixture=(dict(position=[*test_spawn,.296],quaternion_wxyz=[0.,0.,0.,1.] if args.near_end_test else [1.,0.,0.,0.],mission_index=12 if args.near_end_test else 0,scope='20m endpoint online validation') if args.headland_test else json.loads((args.fixture_file if args.fixture_file else ROOT/('outputs/full_map_stop_aisle4_20261030/fixture.json' if args.full_map_fixture else 'outputs/precision_experiments/home_actuator_fixture.json' if args.home_actuator_test else 'outputs/route23_validation/micro_fixture.json')).read_text(encoding='utf-8')))
   art.set_world_poses(positions=[fixture['position']],orientations=[fixture['quaternion_wxyz']])
   art.set_velocities(linear_velocities=np.zeros((1,3),np.float32),angular_velocities=np.zeros((1,3),np.float32))
   art.set_dof_positions(q[None]);art.set_dof_velocities(np.zeros((1,12),np.float32));gait.reset();gait.phase=float(fixture.get('initial_gait_phase_s',0.))
   micro_initialized=True;micro_release=t+.5;cloud_time=-1.;progress_watch=None;pos,yaw,roll,pitch=pose()
   mission.index=fixture.get('mission_index',0);mission.phase=mission.stages[mission.index][0]
  if t<startup_time+3 or (args.micro_test and t<micro_release):state='SETTLING'
  elif args.sensor_check and t<19:
   if t<5:state='STATIC_IMU'
   elif t<11:vx=.075;stop=False;state='FORWARD_SENSOR_TEST'
   elif t<14:state='STATIC_AFTER_WALK'
   elif t<17:omega=.06;stop=False;state='SMALL_YAW_SENSOR_TEST'
   else:state='SENSOR_TEST_COMPLETE'
  elif args.odom_benchmark:
   if benchmark_origin is None and t>=5:benchmark_origin=pos.copy()
   if benchmark_origin is not None:
    progress=float(pos[0]-benchmark_origin[0])
    if progress>=args.benchmark_distance and benchmark_done_time is None:benchmark_done_time=t
    if benchmark_done_time is None:
     safe,reason,support=guard(1,pos,yaw)
     state='BENCHMARK_FORWARD' if safe else 'BENCHMARK_OBSTACLE_WAIT'
     if safe:vx=.075;vy=float(np.clip((START[1]-pos[1])*.65,-.035,.035));omega=float(np.clip(-yaw*.8,-.15,.15));stop=False
    else:
     state='BENCHMARK_COMPLETE';finished=True
   else:state='BENCHMARK_SETTLING'
  elif running and not finished and not fault and args.odometry_filter and ros_source and not ros_source.last_structure_filter.get('valid',False):
   state='WAIT_ODOMETRY_STRUCTURE';reason=ros_source.last_structure_filter.get('reason','WAIT_FILTERED_CLOUD')
  elif running and not finished and not fault and args.odometry_filter and not args.slam_control and (not monitored_odom or not monitored_odom.get('valid',False) or last_imu_time-monitored_odom['stamp']>.6 or time.monotonic()-monitored_odom['received_wall']>3.):
   state='WAIT_ODOMETRY_INPUT';reason='WAIT_FRESH_ICP_OUTPUT'
  elif running and not finished and not fault and args.odom_error_limit>0 and odom_error is not None and odom_error>args.odom_error_limit:
   state='WAIT_ODOMETRY_ERROR';reason='MAPPING_VALIDATION_ERROR_LIMIT'
  elif running and not finished and not fault and args.slam_control and not nav_status['valid']:
   state='WAIT_SLAM';reason=nav_status['reason']
  elif running and not finished and not fault:
   if args.obstacle_test and not person_event and pos[0]>-2.5:
    add_person();person_event=True;milestones['person_added']=t
   safe,reason,support=guard(1,pos,yaw)
   if args.route23 and args.online_fusion:
    target=mission.stages[mission.index][1] if not mission.done else START
    online_planner.set_alignment(slam_feedback.filter.task_from_map)
    map_clear,map_reason=online_planner.check(nav_pos,target,max(0.,last_imu_time),local_ground_observed=mission.is_row_phase() and support>=8 and 0<=t-cloud_time<=.4)
    nav_status['route_grid_check']=dict(online_planner.last_check)
    if not map_clear:safe=False;reason=map_reason
   recovery_diagnostic={};row_preview_diagnostic={};row_clearance={}
   if args.route23:
    from full_map_guard import evaluate_local_sweep,evaluate_row_preview,choose_row_clearance_target,evaluate_clearance_escape
    raw_guard_points=np.column_stack((body_cloud,measured_surface_height))
    estimated_pose=(*nav_status['position'][:2],nav_status['yaw']) if args.slam_control else (*pos[:2],yaw)
    guard_points=near_field_memory.points(raw_guard_points,raw_row_observation.get('floor_plane'),estimated_pose,cloud_time,t)
    guard_points=foliage_memory.classify(guard_points,raw_row_observation.get('floor_plane'),estimated_pose,rgb_data,body_to_camera,cloud_time,last_rgb_time,t)
    if mission.needs_paired_edges():
     if not raw_row_observation.get('valid'):
      if combined_row_scan!=(mission.phase,cloud_time):
       from full_map_fast_row_observer import estimate_row_center
       combined_row_cache=estimate_row_center(guard_points[:,:3]-[.3,0,.32]);combined_row_scan=(mission.phase,cloud_time)
      combined_row=combined_row_cache
     else:combined_row=raw_row_observation
     local_row_observation=row_tracker.resolve(raw_row_observation,combined_row,mission.phase,cloud_time,estimated_pose,mission.stages[mission.index][1][1],mission.stages[mission.index][2])
    if local_row_observation.get('valid'):last_row_valid_time=t
    if mission.needs_paired_edges():
     if row_target_phase!=mission.phase:row_target_memory=None;row_target_phase=mission.phase
     if row_plan_scan!=(mission.phase,cloud_time):
      row_plan_cache=choose_row_clearance_target(guard_points,local_row_observation,t-cloud_time,previous=row_target_memory,final_goal=mission.phase=='CLOSE_START' and abs(estimated_pose[0]-START[0])<.15);row_plan_scan=(mission.phase,cloud_time)
     row_clearance=dict(row_plan_cache) if t-cloud_time<=.4 else dict(valid=False,reason='STALE_CLOUD')
     if row_clearance.get('valid'):row_target_memory=dict(row_clearance)
    if mission.needs_paired_edges() and local_row_observation.get('valid') and reason=='OBSTACLE_IN_TRAVEL_DIRECTION':
     lv=float(np.clip(1.4*row_clearance.get('error_body_y_m',local_row_observation['center_error_body_y_m']),-.04,.04))
     lw=float(np.clip(.9*row_clearance.get('error_heading_rad',local_row_observation['heading_error_rad']),-.045,.045))
     row_preview_diagnostic=evaluate_local_sweep(guard_points,cruise_speed*sweep_horizon,lv*sweep_horizon,lw*sweep_horizon,t-cloud_time,margin=row_clearance.get('tracking_margin_m',.05))
     if row_preview_diagnostic['safe']:
      safe=True;reason='MEASURED_ROW_PREVIEW_CLEAR'
   control_pos=nav_pos if args.slam_control else pos
   controller_yaw=nav_yaw
   if args.endpoint_drift_test and args.slam_control:
    if 'drift_test_start' not in locals():drift_test_start=t
    drift_fraction=float(np.clip((t-drift_test_start)/4.,0.,1.))
    nav_pos=np.asarray(nav_pos).copy();nav_pos[:2]+=np.array([1.27,-.57])*drift_fraction
    nav_yaw=(nav_yaw+.48*drift_fraction+math.pi)%(2*math.pi)-math.pi
    control_pos=nav_pos;controller_yaw=nav_yaw
    nav_status['fault_injection']=dict(test_only=True,fraction=drift_fraction,translation_m=[1.27*drift_fraction,-.57*drift_fraction],yaw_rad=.48*drift_fraction,injected_position=nav_pos.tolist(),injected_yaw=nav_yaw,excluded_from_accuracy_acceptance=True)
   row_pose_fusion=dict(applied=False,reason='NOT_PAIRED_ROW_PHASE')
   if args.slam_control:
    from full_map_row_pose import RowControlFrame
    if 'row_control_frame' not in locals():row_control_frame=RowControlFrame()
    row_control_frame.rebase(nav_status.get('map_rebase',(0.,0.,0.)))
    stage_target=mission.stages[mission.index]
    control_pos,controller_yaw,row_pose_fusion=row_control_frame.update(control_pos,controller_yaw,stage_target[1][1],stage_target[2],local_row_observation,t-cloud_time,row_frame_time,mission.phase,mission.needs_paired_edges())
    row_pose_fusion['propagation_clock']=row_clock_diagnostic
    control_pos=np.asarray(control_pos)
    end_anchor=dict(applied=False)
    end_x=mission.end_anchor_target()
    if end_x is not None and abs(control_pos[0]-end_x)<3. and abs(((controller_yaw-stage_target[2]+math.pi)%(2*math.pi))-math.pi)<.12:
     from full_map_end_observer import observe_bed_end
     end_scan=observe_bed_end(body_cloud,measured_surface_height,mission.aisles[mission.current_aisle]['width_m'],stage_target[2]-controller_yaw)
     control_pos,end_anchor=row_control_frame.anchor_end(nav_pos,nav_yaw,control_pos,controller_yaw,end_x,stage_target[2],end_scan)
     control_pos=np.asarray(control_pos)
    nav_status['end_anchor']=end_anchor
    nav_status['control_position']=control_pos.tolist();nav_status['control_yaw']=controller_yaw
   nav_status['row_control_fusion']=row_pose_fusion
   cruise_speed=mission.tracking_speed(control_pos)
   sweep_horizon=min(.6,.045/cruise_speed)
   if args.micro_test and fixture.get('controller_pose_bias'):
    bias=fixture['controller_pose_bias'];control_pos=pos.copy();control_pos[:2]+=bias[:2];controller_yaw=nav_yaw+bias[2]
   if args.route23:
    mission_control_pos=control_pos
    if args.slam_control and mission.phase=='CLOSE_START':
     # The row-control frame may carry a translated x origin after measured
     # bed-end anchoring. Keep final along-row distance in the known SLAM
     # start frame; paired-row fusion still supplies lateral position/yaw.
     mission_control_pos=mission.home_approach_pose(nav_pos,control_pos)
     nav_status['home_approach_longitudinal_source']='ONLINE_SLAM_NAVIGATION'
    # Row edges refine localization; a missing secondary observation must not
    # veto a healthy primary estimated-pose controller on a surveyed aisle.
    row_fallback=bool(mission.needs_paired_edges() and not local_row_observation.get('valid'))
    held_target=row_target_memory if row_target_phase==mission.phase and row_target_memory else {}
    fallback_margin=held_target.get('tracking_margin_m',.05) if t-last_row_valid_time<=1.5 else .05
    if row_fallback:
     mission_control_pos=np.asarray(control_pos).copy()
     mission_control_pos[1]-=math.cos(mission.stages[mission.index][2])*held_target.get('offset_from_row_center_m',0.)
     fallback_preview=evaluate_row_preview(guard_points,.045,0.,0.,t-cloud_time,margin=fallback_margin)
     if fallback_preview['safe']:safe=True;reason='PRIMARY_LOCALIZATION_RAW_3D_CLEAR'
    row_pose_rejected=args.slam_control and not row_pose_fusion.get('ready',True)
    if row_pose_rejected:
     state='WAIT_ROW_POSE';reason=row_pose_fusion['reason'];vx=vy=omega=0.;stop=True
    else:
     state,(vx,vy,omega,stop)=mission.step(mission_control_pos,controller_yaw,t,safe,turn_space(pos),row_observation=local_row_observation)
     if state=='WAIT_HOME_OVERSHOOT_REVIEW':
      attention_reason='HOME_OVERSHOOT_BEYOND_8CM';running=False
    if state=='FAULT_FULL_MAP_CORRIDOR_BOUNDARY' and args.slam_control:
     # An estimated-pose disagreement is recoverable. Keep the window and
     # sensor processing alive; physical boundary/contact supervision below
     # remains independent and still terminates a real boundary violation.
     state='WAIT_ESTIMATED_CORRIDOR';reason='ESTIMATED_POSE_OUTSIDE_ASSIGNED_CORRIDOR';vx=vy=omega=0.;stop=True
    row_phase=mission.needs_paired_edges()
    map_permitted=not args.online_fusion or map_clear
    row_ok=local_row_observation.get('valid',False)
    if row_phase and not mission.done and map_permitted and not row_pose_rejected:
     if row_ok and row_clearance.get("valid"):
      lateral=row_clearance.get('error_body_y_m',local_row_observation['center_error_body_y_m']);angle=row_clearance.get('error_heading_rad',local_row_observation['heading_error_rad'])
      local_vy=float(np.clip(1.4*lateral,-.04,.04));local_w=float(np.clip(.9*angle,-.045,.045))
      clearance_escape=bool(row_clearance.get("escape_required") or row_clearance.get("lateral_first"))
      moving_forward=not stop and not clearance_escape
      can_recover=clearance_escape or (not safe and (abs(lateral)>.005 or abs(angle)>.005))
      if moving_forward or can_recover:
       local_vx=mission.row_command_speed(vx,control_pos) if moving_forward else 0.
       recovery_diagnostic=evaluate_local_sweep(guard_points,local_vx*sweep_horizon,local_vy*sweep_horizon,local_w*sweep_horizon,t-cloud_time,margin=row_clearance.get('tracking_margin_m',.05))
       if clearance_escape:recovery_diagnostic=evaluate_clearance_escape(guard_points,local_vy*sweep_horizon,local_w*sweep_horizon,t-cloud_time,margin=row_clearance['tracking_margin_m'],target_y=lateral,target_angle=angle,final_goal=row_clearance.get('final_goal',False))
       if can_recover:
        if recovery_started is None:recovery_started=t
        # Physical progress is evaluated centrally, including intended lateral/heading correction.
       else:recovery_started=None
       if recovery_diagnostic['safe']:
        vx,vy,omega,stop=local_vx,local_vy,local_w,False
        milestones.setdefault('local_row_tracking_used',t)
        if can_recover:state='RECOVER_ROW_CENTER';milestones.setdefault('micro_heading_recovery_used',t)
       else:state='WAIT_LOCAL_SWEEP';vx=vy=omega=0.;stop=True;reason=recovery_diagnostic['reason']
     elif row_ok:
      state='WAIT_ROW_CLEARANCE';vx=vy=omega=0.;stop=True;reason=row_clearance.get('reason','NO_SAFE_ROW_TARGET')
     elif row_fallback:
      if not stop:
       vx=mission.row_command_speed(vx,control_pos)
       vy=float(np.clip(vy,-.04,.04));omega=float(np.clip(omega,-.045,.045))
       recovery_diagnostic=evaluate_local_sweep(guard_points,vx*sweep_horizon,vy*sweep_horizon,omega*sweep_horizon,t-cloud_time,margin=fallback_margin)
       row_preview_diagnostic=evaluate_row_preview(guard_points,vx*sweep_horizon,vy*sweep_horizon,omega*sweep_horizon,t-cloud_time,margin=fallback_margin)
       if recovery_diagnostic['safe'] and row_preview_diagnostic['safe']:
        state='FIXED_ROW_PRIMARY_LOCALIZATION';reason='ROW_OBSERVATION_MISSING_PRIMARY_POSE_RAW_SWEEP_CLEAR'
        milestones.setdefault('primary_localization_row_fallback_used',t)
       else:state='WAIT_LOCAL_SWEEP';vx=vy=omega=0.;stop=True;reason='PRIMARY_POSE_RAW_SWEEP_OCCUPIED'

   else:
    state,(vx,vy,omega,stop)=mission.step(control_pos,nav_yaw,t,safe,turn_space(pos))
   if mission.is_row_phase() and not mission.needs_paired_edges() and not stop:
    from full_map_guard import evaluate_local_sweep
    recovery_diagnostic=evaluate_local_sweep(guard_points,vx*sweep_horizon,vy*sweep_horizon,omega*sweep_horizon,t-cloud_time)
    if not recovery_diagnostic['safe']:
     state='WAIT_LOCAL_SWEEP';vx=vy=omega=0.;stop=True;reason=recovery_diagnostic['reason']
    else:milestones.setdefault('outer_lane_3d_tracking_used',t)
   phase=mission.phase
   if args.micro_test and fixture.get('turn_transfer_test') and mission.index>=fixture['transfer_mission_index'] and fixture['position'][1]-pos[1]>=fixture['required_transfer_m']:
    finished=True;mission.done=True;state='TURN_TRANSFER_TEST_COMPLETE';vx=vy=omega=0.;stop=True
    milestones['turn_transfer_test_complete']=t
   if args.micro_test and not args.headland_test and not fixture.get('turn_transfer_test') and (pos[0]-fixture['position'][0])*fixture.get('direction',1)>=fixture.get('required_progress_m',.45) and abs(pos[1]-fixture.get('desired_row_y',-.825))<.015:
    finished=True;mission.done=True;state='MICRO_RECOVERY_TEST_COMPLETE';vx=vy=omega=0.;stop=True
    milestones['micro_test_cleared_x']=float(pos[0])
   if args.headland_test and ((args.near_end_test and mission.index>=16 and pos[0]>-19.8 and abs(pos[1]+.825)<.04) or (not args.near_end_test and mission.index>=4 and pos[0]<19.8 and abs(pos[1]-.4)<.04)):
    finished=True;mission.done=True;state='HEADLAND_TEST_COMPLETE';vx=vy=omega=0.;stop=True
   if args.route23 and args.online_fusion and not map_clear:
    state='WAIT_ROUTE_MAP';vx=vy=omega=0.;stop=True
   if state in ('WAIT_TURN_SPACE','WAIT_END_CONFIRMATION'):reason=turn_diagnostic.get('reason','TURN_CHECK_UNAVAILABLE')
   if args.online_fusion and state=='WAIT_ONLINE_PATH':reason=mission.plan_status['reason']
   if state.startswith('WAIT') and person_created is not None:
    if person_stop_time is None:person_stop_time=t;milestones['obstacle_stop']=t;milestones['phase_when_blocked']=phase
    if args.obstacle_test and t-person_stop_time>4:remove_person();milestones['obstacle_removed']=t
   if person_stop_time is not None and person_created is None and not stop and 'resumed' not in milestones:milestones['resumed']=t
   finished=mission.done;milestones.update(mission.events)
   if state.startswith('FAULT'):fault=state
  else:
   state='FAULT' if fault else 'DONE' if finished else 'PAUSED_NEEDS_ATTENTION' if attention_reason else 'PAUSED'
   if attention_reason:reason=attention_reason
  # Waiting for ROS/SLAM/map readiness is intentional inactivity, not a
  # physical no-progress fault. Once a route command is active the watchdog
  # remains enabled and still catches genuine stalls.
  monitored_motion=running and not finished and not fault and not state.startswith(('WAIT','CONFIRM_','PAUSED')) and state not in ('NEXT_ROUTE_STAGE','SETTLING','BENCHMARK_SETTLING','STATIC_IMU','STATIC_AFTER_WALK','SENSOR_TEST_COMPLETE')
  progress_stage=mission.stages[min(mission.index,len(mission.stages)-1)]
  desired_adjustment=None
  if mission.is_row_phase() and row_clearance.get('valid') and (state.startswith('RECOVER') or row_clearance.get('lateral_first')):
   h=progress_stage[2];desired_adjustment=(progress_stage[1][1]+math.cos(h)*row_clearance['offset_from_row_center_m'],h+row_clearance.get('yaw_offset_rad',0.))
  progress_fault=motion_monitor.observe(t,mission.phase,state,pos,yaw,progress_stage[1],progress_stage[2],mission.is_row_phase(),monitored_motion,desired_adjustment)
  if progress_fault:
   np.savez_compressed(OUT/'blocked_raw_cloud.npz',body_points=body_cloud,world_points=cloud,guard_points=guard_points)
   (OUT/'attention.json').write_text(json.dumps(dict(t=t,state=state,phase=mission.phase,reason=reason,row_clearance=row_clearance,sweep=recovery_diagnostic,navigation=nav_status),indent=2),encoding='utf-8')
   vx=vy=omega=0.;stop=True
   if args.test:
    fault=progress_fault;state='FAULT'
   else:
    attention_reason=progress_fault;running=False;state='PAUSED_NEEDS_ATTENTION'
    reason=progress_fault;motion_monitor.reset()
    (OUT/'trajectory_checkpoint.json').write_text(json.dumps(rows),encoding='utf-8')
    print('LAB_ATTENTION',json.dumps(dict(t=t,reason=progress_fault,phase=mission.phase,action='Saved diagnostics; paused for inspection. Window remains open.')),flush=True)
  contact_audit.save(t)
  if contact_audit.forbidden:fault='FORBIDDEN_PLANT_OR_RIGID_CONTACT';stop=True;state='FAULT';vx=vy=omega=0.
  if not mission.permitted(pos):fault='NO_CROSS_OR_AISLE_BOUNDARY';stop=True;state='FAULT'
  if pos[2]<.18 or abs(roll)>.4 or abs(pitch)>.4:fault='POSTURE';stop=True;state='FAULT'
  if prev is not None:distance+=float(np.linalg.norm(pos[:2]-prev))
  prev=pos[:2].copy()
  rows.append(dict(ray_surface_counts=dict(ray_surface_counts),visual_foliage=dict(foliage_memory.diagnostic),local_row=dict(local_row_observation),row_clearance=dict(row_clearance) if 'row_clearance' in locals() else {},command=dict(vx=vx,vy=vy,omega=omega,stop=stop),t=round(t,4),posture=dict(roll_rad=float(roll),pitch_rad=float(pitch),height_m=float(pos[2])),position=pos.tolist(),yaw=float(yaw),state=state,phase=phase,guard=reason,rear_or_front_ground_points=support,turn_check=dict(turn_diagnostic),navigation=nav_status,odometry=monitored_odom,odometry_reference_error_m=odom_error,navigation_position=nav_pos.tolist() if nav_pos is not None else None,slam_vs_reference_error_m=float(np.linalg.norm(nav_pos[:2]-pos[:2])) if nav_pos is not None else None))
  if args.online_fusion:
   ps=mission.plan_status
   rows[-1]['planner']={k:v for k,v in ps.items() if k!='path'}
   rows[-1]['planner']['path_points']=len(ps.get('path',[]))
   loop_acceptance.observe(rows[-1],distance,finished)
   navigation_loop_events=loop_acceptance.count
   rows[-1]['navigation_loop_events']=navigation_loop_events
  if args.route23:
   rows[-1]['row_recovery']=dict(recovery_diagnostic);rows[-1]['local_row']=dict(local_row_observation);rows[-1]['row_preview']=dict(row_preview_diagnostic);rows[-1]['row_clearance']=dict(row_clearance)
   rows[-1]['route_aisle']=mission.current_aisle
   rows[-1]['command']={'vx':float(vx),'vy':float(vy),'omega':float(omega),'stop':bool(stop)}
  for _ in range(24):
   contact_audit.time=t
   art.set_dof_position_targets(gait.step(dt,vx,omega,stop,vy)[None]);SM.step();t+=dt
   f=imu.get_data()
   if f['time']>last_imu_time:
    last_imu_time=float(f['time']);imu_count+=1
    if ros_source:ros_source.imu(f)
    if t>3:imu_frames.append(dict(t=t,sensor_time=last_imu_time,a=np.asarray(f['linear_acceleration']).tolist(),w=np.asarray(f['angular_velocity']).tolist(),phase=state))
    times['imu'].append(last_imu_time)
  app.update()
  if ros_source:ros_source.pump()
  contact_audit.save(t)
  if contact_audit.forbidden:fault='FORBIDDEN_PLANT_OR_RIGID_CONTACT';finished=False;stop=True;state='FAULT'
  if label:
   comparison=rows[-1].get('slam_vs_reference_error_m') if args.slam_control else odom_error
   comparison_name='SLAM vs reference' if args.slam_control else 'Raw odometry vs reference'
   comparison_text='waiting' if comparison is None else f'{comparison*100:.1f} cm'
   label.text=f"{state}  |  {t:.1f} s\nx {pos[0]:.2f} m  |  RGB {camera_count}  cloud {len(cloud)}  IMU {imu_count}\n{reason}  |  Headland transfers only\nNavigation: {nav_status['source']} | {nav_status['reason']}\n{comparison_name}: {comparison_text} (simulation)"
  if label and args.online_fusion:
   label.text+=f"\nRoute: {mission.plan_status['reason']} | grid {online_planner.core.revision}\nVisual words: {(nav_status.get('fusion') or {}).get('visual_words',0)}"
   label.text+=f" | Matches: {(nav_status.get('fusion') or {}).get('global_loop_events',0)} | Travel candidates: {navigation_loop_events}"
  if label and args.online_fusion:
   label.text+='\nVisual mode: '+nav_status.get('visual_quality','initializing')
  if label and args.route23:
   label.text+=('\nLiDAR row offset: %.1f cm' % (100*local_row_observation['center_error_body_y_m']) if local_row_observation.get('valid') else '\nRow observation: '+local_row_observation.get('reason','waiting'))
  if t>=4 and args.preview:
   snapshot('温室全景.png','/World/Route23View' if args.route23 else '/World/LabOverview');snapshot('温室通道.png','/World/LabAisle');snapshot('温室内部.png','/World/LabInterior');snapshot('相机画面.png',camera_path)
   if not args.test:break
   args.preview=False
  rows[-1]['performance']={'profile':'full_density_720x33','cruise_speed_mps':mission.cruise_speed,
                          'wall_elapsed_s':time.perf_counter()-run_wall_start,
                          'real_time_factor':t/max(.001,time.perf_counter()-run_wall_start),
                          'rays_per_scan':len(directions),'physics_hz':240,'rgb_hz':5,'lidar_hz':5}
  if label:label.text+='\nRuntime: '+('FULL DENSITY %.3f m/s' % cruise_speed)+' | RTF %.2f' % rows[-1]['performance']['real_time_factor']
  if t-last_status>=5:
   print('LAB_STATUS',json.dumps(rows[-1]),flush=True);last_status=t
   (OUT/'live_status.json').write_text(json.dumps(rows[-1],indent=2),encoding='utf-8')
   if args.complete_loop_test and int(t//5)%12==0:
    (OUT/'trajectory_checkpoint.json').write_text(json.dumps(rows),encoding='utf-8')
  if args.sensor_check and not args.test and t>=19:break
  if args.complete_loop_test and ((finished and not args.home_approach_test) or fault or (t>=2400. and not attention_reason)):break
  if args.headland_test and finished:break
  if finished and fixture_finished_at is None:fixture_finished_at=t
  fixture_settled=not args.micro_test or not finished or t-fixture_finished_at>=fixture.get('settle_after_done_s',0.)
  if args.home_approach_test and finished:fixture_settled=t-fixture_finished_at>=3.
  if args.home_approach_test and finished and fixture_settled:break
  if args.test and (fault or (finished and fixture_settled and (not args.odom_benchmark or t-benchmark_done_time>=2)) or t>=startup_time+args.limit):break
 if args.record_lidar and recorded_scans:
  scan_offsets=np.cumsum([0]+[len(a) for a in recorded_scans])
  np.savez_compressed(OUT/('structured_lidar_samples.npz' if args.record_structure_only else 'raw_lidar_samples.npz'),points=np.concatenate(recorded_scans),offsets=scan_offsets,sensor_times=[v['sensor_time'] for v in recorded_scan_meta])
  (OUT/'scan_pose_reference.json').write_text(json.dumps(recorded_scan_meta),encoding='utf-8')
  (OUT/'imu_samples.json').write_text(json.dumps(imu_frames),encoding='utf-8')
  check=dict(frames=len(recorded_scan_meta),max_usd_physics_position_difference_m=max(v['translation_difference_m'] for v in recorded_scan_meta),max_usd_physics_yaw_difference_rad=max(abs(v['yaw_difference_rad']) for v in recorded_scan_meta),scope='Sensor-generation consistency only, not odometry accuracy acceptance.')
  (OUT/'sensor_consistency.json').write_text(json.dumps(check,indent=2),encoding='utf-8');print('SENSOR_CONSISTENCY',json.dumps(check),flush=True)
 if rgb_data is not None:
  from PIL import Image
  Image.fromarray(rgb_data).save(OUT/'RGB最新帧.png')
 projection={}
 if frame_matrix is not None and depth_data is not None and len(cloud):
  pc=cloud@np.linalg.inv(frame_matrix)[:3,:3]+np.linalg.inv(frame_matrix)[3,:3];zz=-pc[:,2];valid=zz>.1;u=np.zeros(len(pc));v=np.zeros(len(pc));u[valid]=320*pc[valid,0]/zz[valid]+320;v[valid]=180-320*pc[valid,1]/zz[valid]
  valid&=(u>1)&(u<638)&(v>1)&(v<358);ids=np.flatnonzero(valid);sample=depth_data[v[ids].astype(int),u[ids].astype(int)].reshape(-1);ok=np.isfinite(sample)&(sample>.1);residual=np.abs(sample[ok]-zz[ids][ok]);projection={'projected_points':len(ids),'finite_depth_samples':int(ok.sum()),'median_depth_residual_m':float(np.median(residual)) if len(residual) else None,'note':'ideal rendered depth is diagnostic only, not RGB sensor output or control input; edge/occlusion samples included'}
  from PIL import Image,ImageDraw
  overlay=Image.fromarray(rgb_data);draw=ImageDraw.Draw(overlay)
  for ix in ids:
   if np.isfinite(depth_data[int(v[ix]),int(u[ix])]) and abs(float(depth_data[int(v[ix]),int(u[ix])])-zz[ix])<.10:draw.ellipse((u[ix]-1,v[ix]-1,u[ix]+1,v[ix]+1),fill=(0,220,120))
  overlay.save(OUT/'雷达相机投影检查.png')
 stats={}
 for sensor,stamps in times.items():
  diffs=np.diff(stamps);stats[sensor]=dict(frames=len(stamps),rate_hz=float(1/np.median(diffs)) if len(diffs) else 0,monotonic=bool(np.all(diffs>0)))
 static=[f for f in imu_frames if f['phase']=='STATIC_IMU'];turn=[f for f in imu_frames if f['phase']=='SMALL_YAW_SENSOR_TEST']
 contact_audit.save(t,force=True)
 checks={'rgb_nonempty':rgb_data is not None,'rgb_not_flat':bool(rgb_data is not None and np.std(rgb_data)>8),'lidar_nonempty':len(cloud)>100,'imu_finite':bool(imu_frames and all(np.isfinite(f['a']).all() and np.isfinite(f['w']).all() for f in imu_frames)),'timestamps_monotonic':all(s['monotonic'] for s in stats.values()),'imu_fresh':len(times['imu'])>100}
 report={'runtime_performance':rows[-1].get('performance'),'forbidden_contact':contact_audit.forbidden,'visual_foliage':foliage_memory.diagnostic,'sensor_checks':checks,'lidar_camera_projection':projection,'terrain':terrain_info,'scope':'data stream and motion preflight only; calibration and SLAM NOT accepted','sensor_streams':stats,'camera':camera_meta,'lidar_model':f'generic 360deg snapshot collision raycast, {len(directions)} rays/scan; NOT calibrated L2, no per-point scan time','imu_model':'native PhysX IMU specific force, no added bias/noise','static_acceleration_norm_mps2':float(np.median([np.linalg.norm(f['a']) for f in static])) if static else None,'static_angular_speed_rps':float(np.median([np.linalg.norm(f['w']) for f in static])) if static else None,'turn_median_gyro_z':float(np.median([f['w'][2] for f in turn])) if turn else None,'simulation_time_s':t,'completed':finished,'fault':fault,'attention_reason':attention_reason,'travel_m':distance,'milestones':milestones,'minimum_x':min(r['position'][0] for r in rows),'maximum_x':max(r['position'][0] for r in rows),'passed':bool(all(checks.values()) and not fault and (finished if args.test else True)),'limitations':[('SLAM estimated pose drives route; simulator truth only for diagnostic and independent safety checks' if args.slam_control else 'truth localization, no SLAM'),'measured local lidar floor plane; flat-ground assumption','synthetic floor profiles are engineering stress tests and not measured field terrain' if args.terrain_profile!='flat' else 'flat-ground baseline','ROS sensor publishing enabled' if args.ros else 'no ROS publishing yet','camera ideal test intrinsics, not AS2 calibrated','generic ray lidar snapshot not L2 scan simulation','pipeline placement and length provisional',('same-aisle end confirmed from SLAM estimate + point support' if args.slam_control else 'same-aisle end confirmed from truth pose + point support; not SLAM localization'),'person fixture is collision geometry; no semantic human recognition']}
 if args.odom_benchmark:
  report['scope']='Controlled short physical motion stimulus and sensor-generation consistency; NOT SLAM odometry acceptance'
  report['benchmark']={'target_forward_m':args.benchmark_distance,'reference_origin':benchmark_origin.tolist() if benchmark_origin is not None else None,'done_time':benchmark_done_time,'actual_forward_m':float(pos[0]-benchmark_origin[0]) if benchmark_origin is not None else None}
 if args.ros and args.odometry_filter and not args.slam_control:
  report['scope']='Controlled reference drive for mapping/odometry validation; not SLAM-guided navigation'
  report['passed']=bool(all(checks.values()) and not fault and finished)
  report['max_odometry_reference_error_m']=max((v['odometry_reference_error_m'] for v in rows if v['odometry_reference_error_m'] is not None),default=None)
 if args.slam_control:
  valid_rows=[v for v in rows if v['navigation']['valid']]
  report['navigation_source']='SLAM'
  report['passed']=bool(all(checks.values()) and not fault and finished and valid_rows)
  report['slam_control']={'valid_pose_samples':len(valid_rows),'waiting_samples':sum(v['state']=='WAIT_SLAM' for v in rows),'max_reference_comparison_error_m':max((v['slam_vs_reference_error_m'] for v in valid_rows),default=None),'known_start_alignment_only':True,'truth_pose_not_used_as_route_feedback':True,'independent_truth_boundary_and_posture_supervision':True}
 if args.online_fusion:
  report['scope']='Live RGB appearance + lidar geometry RTAB-Map mapping, observed-grid planning; live acceptance required'
  report['online_planning']={'algorithm':'A* on inflated observed-free live occupancy grid','prior':'assigned aisle, end bay and task destinations; not unrestricted frontier exploration','dynamic_obstacle_test':False,'visual_odometry':False,'visual_words_max':max(((r['navigation'].get('fusion') or {}).get('visual_words',0) for r in rows),default=0),'grid_revisions':online_planner.core.revision}
  report['online_planning']['global_loop_events']=max(((r['navigation'].get('fusion') or {}).get('global_loop_events',0) for r in rows),default=0)
  report['online_planning']['proximity_events']=max(((r['navigation'].get('fusion') or {}).get('proximity_events',0) for r in rows),default=0)
  report['online_planning']['same_heading_return']='same_heading_home_time' in milestones
  report['navigation_and_mapping_completed']=report['passed']
  report['online_planning']['global_loops_after_travel_and_return']=navigation_loop_events
  report['online_planning']['loop_event_evidence']=loop_acceptance.events
  report['online_planning']['loop_policy']='Before DONE, moving, candidate seen >=30s and >=5m of diagnostic travel earlier; geometry still needs graph verification'
  report['visual_global_loop_verified']=navigation_loop_events>0
  report['passed']=bool(report['passed'] and report['visual_global_loop_verified'] and report['online_planning']['same_heading_return'] and online_planner.core.revision>0)
 if args.route23:
  corridor_metrics=mission.corridor_metrics(rows)
  report['route23']={'aisles_from_right':list(mission.selected),'excluded_aisles':excluded_route,'return':f'bottom headland from aisle{mission.selected[-1]} to aisle2','astar_used':False,'swap_only_in_headlands':True,'scope':'physical reference-controller test, NOT live SLAM acceptance' if not args.slam_control else 'live estimated-pose route tracking; pending graph and error audit', 'max_corridor_lateral_error_m':corridor_metrics['max_corridor_lateral_error_m']}
  report.update(corridor_metrics)
  if args.online_fusion:
   report['online_planning']['algorithm']='Fixed surveyed centerlines + heading/cross-track feedback; NO A*'
   report['online_planning']['prior']=f'aisles {list(mission.selected)}; excluded {excluded_route}; bottom headland return to aisle2; known start'
   report['scope']='RGB/lidar online mapping + fixed-route estimated-pose control; global loop geometry requires independent database review'
   report['global_loop_geometry_verified']=False
   report['full_slam_acceptance_pending_database_review']=True
   report['route_tracking_and_mapping_passed']=bool(report['navigation_and_mapping_completed'])
   report['passed']=False
 if args.micro_test:
  report['scope']='Short PhysX turn and transverse-travel regression; REFERENCE control, not SLAM acceptance' if fixture.get('turn_transfer_test') else 'Short PhysX measured-row centering fixture; possibly biased REFERENCE input, not live SLAM acceptance'
  if args.headland_test and args.slam_control:
   report['scope']='Local online SLAM endpoint control test; no full-map or global loop acceptance'
  if args.endpoint_drift_test:
   report['scope']='Online endpoint control FAULT INJECTION; excluded from SLAM accuracy acceptance'
   report['fault_injection']=dict(translation_m=[1.27,-.57],yaw_rad=.48,ramp_s=4.,injection_point='navigation pose entering local control fusion; raw RTAB-Map and route-map monitor remain uninjected',not_full_slam_fault_reproduction=True)
  report['fixture']=fixture
  report['passed']=bool(not fault and finished and ('local_row_tracking_used' in milestones or 'outer_lane_3d_tracking_used' in milestones or 'turn_transfer_test_complete' in milestones or args.headland_test) and all(checks.values()))
 if args.home_approach_test:
  report['scope']=f'Local {args.home_approach_distance:g}m home approach, NOT a full-route or loop acceptance'
  report['home_approach']={'completed':finished,'known_stationary_start':list(home_spawn),'distance_m':args.home_approach_distance,'target':list(START),'online_slam':args.slam_control,'no_teleport_after_start':True,'database_audit_pending':bool(args.online_fusion)}
  report['passed']=False  # Dedicated local evidence must be reviewed separately.
 if args.headland_test and args.online_fusion:
  report['scope']='Short online SLAM headland turn and transfer; not complete-loop acceptance'
 report['full_map']={'coverage':mission.coverage(rows),'stages':mission.stages,'config':mission.config,'completed':finished,'pending_database_audit':True}
 (OUT/('短距离刺激验证.json' if args.odom_benchmark else '传感器验证.json' if args.sensor_check else '往返验证.json' if args.test else '场景检查.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
 (OUT/('短距离参考轨迹.json' if args.odom_benchmark else '传感器轨迹.json' if args.sensor_check else '往返轨迹.json' if args.test else '场景静止轨迹.json')).write_text(json.dumps(rows),encoding='utf-8');print('LAB_RESULT',json.dumps(report,ensure_ascii=False),flush=True)
 depth_ann.detach(product);rgb.detach(product)
except Exception:
 traceback.print_exc();(OUT/'error.txt').write_text(traceback.format_exc(),encoding='utf-8');raise
finally:
 if ros_source:ros_source.close()
 app.close()


