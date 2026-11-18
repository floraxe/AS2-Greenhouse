"""Raw sensor publishing for ROS2 Humble. No truth odometry or world->base TF.
Known pose is used only to generate sensor-coordinate observations in the simulator.
"""
from pathlib import Path
import os,sys,ctypes,math,json,time
ROOT=Path('E:/AS2_Greenhouse');CORE=Path('E:/isaacsim/exts/isaacsim.ros2.core/humble')
_handles=[]
def prepare_ros(domain_id=0):
 os.environ.update(ROS_DISTRO='humble',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',ROS_DOMAIN_ID=str(domain_id),ROS_LOCALHOST_ONLY='0',AMENT_PREFIX_PATH=str(CORE),FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'work/fastdds_vmware_windows.xml'),ROS_LOG_DIR=str(ROOT/'runtime/ros_logs'))
 os.environ['PATH']=str(CORE/'lib')+os.pathsep+os.environ.get('PATH','')
 _handles.append(os.add_dll_directory(str(CORE/'lib')))
 sys.path.insert(0,str(CORE/'rclpy'))
 for n in ['rcutils.dll','rmw_fastrtps_cpp.dll','rcl.dll']:_handles.append(ctypes.WinDLL(str(CORE/'lib'/n)))
 import rclpy
 return rclpy

def stamp(seconds):
 from builtin_interfaces.msg import Time
 ns=round(float(seconds)*1e9);m=Time();m.sec=ns//1000000000;m.nanosec=ns%1000000000;return m

class SensorPublisher:
 def __init__(self,camera_local,self_check=False,odometry_filter=False,level_odometry=False):
  import numpy as np,rclpy
  from sensor_msgs.msg import Image,CameraInfo,PointCloud2,PointField,Imu
  from rosgraph_msgs.msg import Clock
  from nav_msgs.msg import Odometry
  from tf2_msgs.msg import TFMessage
  from geometry_msgs.msg import TransformStamped
  from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
  self.np=np;self.rclpy=rclpy;self.Image=Image;self.CameraInfo=CameraInfo;self.PointCloud2=PointCloud2;self.PointField=PointField;self.Imu=Imu;self.Clock=Clock
  rclpy.init();self.node=rclpy.create_node('as2_sim_sensor_source');self.self_check=self_check
  self.sent={k:0 for k in ('points','image','camera_info','imu','clock','tf_static')};self.seen={k:0 for k in self.sent};self.first={};self.last={};self.errors=[]
  self.odometry_filter=odometry_filter;self.last_structure_filter={};self.latest_odometry=None
  self.level_odometry=bool(level_odometry);self.floor=None
  self.TFMessage=TFMessage;self.TransformStamped=TransformStamped
  if self.level_odometry:
   from floor_stabilization import FloorStabilizer
   self.floor=FloorStabilizer()
   self.floor_tfpub=self.node.create_publisher(TFMessage,'/tf',100)
  def odometry_monitor(msg):
   q=msg.pose.pose.orientation;p=msg.pose.pose.position
   covariance=[msg.pose.covariance[i] for i in (0,7,35)]
   valid=all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w,*covariance)) and all(v>=0 for v in covariance) and max(covariance[:2])<=.0625 and covariance[2]<=.03
   self.latest_odometry={'valid':bool(valid),'covariance':[float(v) for v in covariance],'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,'position':[p.x,p.y,p.z],'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),'received_wall':time.monotonic()}
  self.odom_monitor=self.node.create_subscription(Odometry,'/as2/odom',odometry_monitor,qos_profile_sensor_data)
  if odometry_filter:self.sent['odometry_points']=0;self.seen['odometry_points']=0
  sensor_qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT)
  self.pubs={}
  for k,typ,topic in [('points',PointCloud2,'/as2/lidar/points'),('image',Image,'/as2/camera/image_raw'),('camera_info',CameraInfo,'/as2/camera/camera_info'),('imu',Imu,'/as2/imu/data'),('clock',Clock,'/clock')]:
   self.pubs[k]=self.node.create_publisher(typ,topic,sensor_qos)
   if self_check:self.node.create_subscription(typ,topic,lambda msg,key=k:self.observe(key,msg),qos_profile_sensor_data)
  if odometry_filter:
   self.pubs['odometry_points']=self.node.create_publisher(PointCloud2,'/as2/lidar/odometry_points',sensor_qos)
   if self_check:self.node.create_subscription(PointCloud2,'/as2/lidar/odometry_points',lambda msg:self.observe('odometry_points',msg),qos_profile_sensor_data)
  tq=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.tfpub=self.node.create_publisher(TFMessage,'/tf_static',tq)
  if self_check:self.node.create_subscription(TFMessage,'/tf_static',lambda msg:self.observe('tf_static',msg),tq)
  def tf(child,position,quaternion):
   a=TransformStamped();a.header.frame_id='base_link';a.header.stamp=stamp(0);a.child_frame_id=child
   a.transform.translation.x=float(position[0]);a.transform.translation.y=float(position[1]);a.transform.translation.z=float(position[2]);q=a.transform.rotation;q.x=float(quaternion[0]);q.y=float(quaternion[1]);q.z=float(quaternion[2]);q.w=float(quaternion[3]);return a
  # USD camera axes (+X right,+Y up,-Z forward) -> ROS optical (+X right,+Y down,+Z forward).
  from pxr import Gf
  optical=np.diag([1.,-1.,-1.,1.])@np.asarray(camera_local)
  quat=Gf.Matrix4d(optical.tolist()).ExtractRotationQuat();v=quat.GetImaginary()
  self.static_msg=TFMessage(transforms=[tf('as2_lidar',(.30,0,.32),(0,0,0,1)),tf('as2_imu',(0,0,0),(0,0,0,1)),tf('as2_camera_optical',optical[3,:3],(*v,quat.GetReal()))])
  self.tfpub.publish(self.static_msg);self.sent['tf_static']=1
  print('ROS_SENSORS_READY: lidar/image/camera_info/imu/clock/static_tf; no ground-truth odometry',flush=True)
 def emit(self,key,msg):
  self.pubs[key].publish(msg);self.sent[key]+=1
 def observe(self,key,msg):
  self.seen[key]+=1
  if key=='tf_static':
   if len(msg.transforms)!=3:self.errors.append('static transform count')
   return
  st=msg.clock if key=='clock' else msg.header.stamp;ns=st.sec*1000000000+st.nanosec
  if key in self.last and ns<=self.last[key]:self.errors.append('non-increasing '+key)
  self.first.setdefault(key,ns);self.last[key]=ns
  if key in ('points','odometry_points') and (msg.point_step!=12 or len(msg.data)!=msg.width*12 or msg.header.frame_id!=('base_stabilized' if key=='odometry_points' and self.level_odometry else 'as2_lidar')):self.errors.append('cloud format')
  if key=='image' and (msg.width!=640 or msg.height!=360 or len(msg.data)!=640*360*3):self.errors.append('image format')
  if key=='imu' and msg.orientation_covariance[0]!=-1.:self.errors.append('truth orientation must not be published')
 def clock(self,seconds):
  m=self.Clock();m.clock=stamp(seconds);self.emit('clock',m)
 def floor_tf(self,T,seconds):
  from pxr import Gf
  a=self.TransformStamped();a.header.frame_id='base_stabilized';a.header.stamp=stamp(seconds);a.child_frame_id='base_link'
  a.transform.translation.z=float(T[2,3]);matrix=T.T
  q=Gf.Matrix4d(matrix.tolist()).ExtractRotationQuat();v=q.GetImaginary()
  a.transform.rotation.x=float(v[0]);a.transform.rotation.y=float(v[1]);a.transform.rotation.z=float(v[2]);a.transform.rotation.w=float(q.GetReal())
  self.floor_tfpub.publish(self.TFMessage(transforms=[a]))
 def imu(self,frame):
  if self.floor is not None:
   T=self.floor.gyro(frame['angular_velocity'],float(frame['time']))
   # Reserve the exact scan timestamp for its new measured plane. IMU TF is
   # interpolated when the next 240Hz sample arrives; no future measurement used.
   if T is not None:self.floor_tf(T,max(0.,float(frame['time'])-1e-9))
  m=self.Imu();m.header.stamp=stamp(frame['time']);m.header.frame_id='as2_imu';m.orientation.w=1.;m.orientation_covariance[0]=-1.
  a=frame['linear_acceleration'];w=frame['angular_velocity']
  m.linear_acceleration.x=float(a[0]);m.linear_acceleration.y=float(a[1]);m.linear_acceleration.z=float(a[2]);m.angular_velocity.x=float(w[0]);m.angular_velocity.y=float(w[1]);m.angular_velocity.z=float(w[2]);self.emit('imu',m);self.clock(frame['time'])
 def cloud(self,world_points,base_matrix,seconds):
  np=self.np;inv=np.linalg.inv(base_matrix);local=np.asarray(world_points)@inv[:3,:3]+inv[3,:3]-np.array([.30,0,.32]);local=np.asarray(local,dtype='<f4')
  if not np.isfinite(local).all():raise ValueError('Non-finite lidar points')
  m=self.PointCloud2();m.header.stamp=stamp(seconds);m.header.frame_id='as2_lidar';m.height=1;m.width=len(local);m.is_bigendian=False;m.is_dense=True;m.point_step=12;m.row_step=12*len(local)
  m.fields=[self.PointField(name=n,offset=i*4,datatype=self.PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))];m.data=local.tobytes();self.emit('points',m)
  if self.odometry_filter:
   from lidar_structure_filter import filter_lidar_structure
   points,self.last_structure_filter=filter_lidar_structure(local)
   if self.last_structure_filter['valid'] and self.floor is not None:
    try:
     T=self.floor.observe(self.last_structure_filter['floor_plane_body'],seconds)
     points=self.floor.points(points);self.floor_tf(T,seconds)
    except ValueError:
     self.last_structure_filter.update(valid=False,reason='FLOOR_STABILIZATION_INVALID')
   if self.last_structure_filter['valid']:
    filtered=self.PointCloud2();filtered.header=type(m.header)();filtered.header.stamp=m.header.stamp;filtered.header.frame_id='base_stabilized' if self.level_odometry else m.header.frame_id;filtered.height=1;filtered.width=len(points);filtered.is_bigendian=False;filtered.is_dense=True;filtered.point_step=12;filtered.row_step=len(points)*12;filtered.fields=m.fields;filtered.data=points.tobytes();self.emit('odometry_points',filtered)

 def image(self,pixels,seconds):
  np=self.np;a=np.ascontiguousarray(pixels,dtype=np.uint8);m=self.Image();m.header.stamp=stamp(seconds);m.header.frame_id='as2_camera_optical';m.width=640;m.height=360;m.encoding='rgb8';m.is_bigendian=0;m.step=640*3;m.data=a.tobytes();self.emit('image',m)
  c=self.CameraInfo();c.header=m.header;c.width=640;c.height=360;c.distortion_model='plumb_bob';c.d=[0.]*5;c.k=[320.,0.,320.,0.,320.,180.,0.,0.,1.];c.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.];c.p=[320.,0.,320.,0.,0.,320.,180.,0.,0.,0.,1.,0.];self.emit('camera_info',c)
 def pump(self):
  # Drain local test callbacks; remote DDS publishing is handled by the ROS middleware.
  for _ in range(90 if self.self_check else 12):self.rclpy.spin_once(self.node,timeout_sec=0.)
 def close(self):
  self.pump()
  result={'published':self.sent,'local_received':self.seen,'format_or_time_errors':sorted(set(self.errors)),'time_source':'native PhysX IMU physics time for all streams','frames':{'points':'as2_lidar','image':'as2_camera_optical','imu':'as2_imu','static_parent':'base_link'},'truth_odometry_published':False,'depth_image_published':False,'lidar_model':'generic 360deg instantaneous PhysX rays, no per-point time/intensity','imu_orientation':'unavailable, covariance[0]=-1; only raw acceleration and gyro','measured_floor_stabilization':self.level_odometry,'odometry_structure_filter':self.last_structure_filter,'cross_system_sensor_reception':'verified by user on original streams; filtered stream pending validation','local_check_passed':all(n>0 for n in self.seen.values()) and not self.errors if self.self_check else None}
  (ROOT/'outputs/ros_sensor_validation.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print('ROS_SENSOR_RESULT',json.dumps(result),flush=True)
  self.node.destroy_node();self.rclpy.shutdown()
