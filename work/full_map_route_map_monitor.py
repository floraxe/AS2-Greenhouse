"""Full-map route grid check with bounded single-cell raster-gap support.
Raw lidar body-width and turn-sweep guards remain mandatory in the caller.
This monitor does not claim full footprint clearance from a coarse grid.
"""
import math,time
import numpy as np

GRID_MEMORY_BUDGET_BYTES=8*1024*1024

def grid_metadata(width,height,resolution,data_length,budget=GRID_MEMORY_BUDGET_BYTES):
    """Bound the actual int16 allocation, independent of the map's yaw."""
    cells=int(width)*int(height)
    info=dict(width=int(width),height=int(height),resolution_m=float(resolution),
              cells=cells,data_length=int(data_length),array_bytes=cells*np.dtype(np.int16).itemsize,
              budget_bytes=int(budget))
    if width<=0 or height<=0 or not math.isfinite(resolution) or not .02<=resolution<=.20:
        return 'GRID_DIMENSIONS_INVALID',info
    if info['array_bytes']>budget:return 'GRID_RESOURCE_BUDGET_EXCEEDED',info
    if data_length!=cells:return 'GRID_DATA_INVALID',info
    return None,info

def isolated_free_surround(grid,ix,iy,resolution):
    """A <=5cm unknown cell surrounded by eight explicitly free cells.

    This is only a coarse-grid continuity inference, never obstacle clearance.
    The caller must also have fresh measured ground and full raw-cloud guards.
    """
    if not 0.<resolution<=.05000001:return False
    if not (0<iy<grid.shape[0]-1 and 0<ix<grid.shape[1]-1):return False
    if grid[iy,ix]!=-1:return False
    patch=grid[iy-1:iy+2,ix-1:ix+2].copy();patch[1,1]=0
    return bool(np.all(patch==0))

class FullMapRouteMapMonitor:
    def __init__(self,node,anchor):
        from nav_msgs.msg import OccupancyGrid,Path
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        self.core=self;self.revision=0;self.grid=None;self.anchor=np.array(anchor)
        self.wall=None;self.stamp=None;self.error=None;self.last_check={};self.last_grid_meta={}
        self.Path,self.PoseStamped=Path,PoseStamped
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        self.sub=node.create_subscription(OccupancyGrid,'/as2/slam/map',self.update,qos)
        self.pub=node.create_publisher(Path,'/as2/planner/path',1)

    def update(self,msg):
        q=msg.info.origin.orientation
        if msg.header.frame_id!='map' or abs(q.x)+abs(q.y)+abs(q.z)>1e-6 or abs(q.w-1)>1e-6:
            self.error='GRID_FRAME_INVALID';return
        self.error,self.last_grid_meta=grid_metadata(msg.info.width,msg.info.height,msg.info.resolution,len(msg.data))
        if self.error:return
        try:self.grid=np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
        except ValueError:self.error='GRID_DATA_INVALID';return
        self.res=msg.info.resolution;self.origin=np.array((msg.info.origin.position.x,msg.info.origin.position.y))
        self.stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        self.wall=time.monotonic();self.revision+=1;self.error=None

    def set_alignment(self,transform):
        if not all(math.isfinite(float(v)) for v in transform):raise ValueError('Invalid map/task alignment')
        self.alignment=tuple(transform)

    def world_to_map(self,point):
        tx,ty,theta=getattr(self,'alignment',(*self.anchor,0.))
        dx,dy=point[0]-tx,point[1]-ty;c,s=math.cos(theta),math.sin(theta)
        return np.array((c*dx+s*dy,-s*dx+c*dy))

    def check(self,pos,target,t,turning=False,local_ground_observed=False):
        self.last_check=dict(position=list(pos[:2]),target=list(target),samples=[],supported_gaps=[],local_ground_observed=bool(local_ground_observed),grid=getattr(self,'last_grid_meta',{}))
        if self.error:return False,self.error
        if self.grid is None or self.wall is None:return False,'WAIT_LIVE_GRID'
        if not -.3<=t-self.stamp<=3. or time.monotonic()-self.wall>12.:return False,'GRID_TIMEOUT'
        delta=np.asarray(target)-np.asarray(pos[:2]);length=float(np.linalg.norm(delta))
        end=np.asarray(pos[:2])+delta*min(1.,.35/max(length,1e-9))
        for point in np.linspace(pos[:2],end,max(2,int(min(length,.35)/(.4*self.res))+1)):
            ix,iy=np.floor((self.world_to_map(point)-self.origin)/self.res).astype(int)
            self.last_check['samples'].append(dict(distance_m=float(np.linalg.norm(point-np.asarray(pos[:2]))),cell=[int(ix),int(iy)],value=int(self.grid[iy,ix]) if 0<=iy<self.grid.shape[0] and 0<=ix<self.grid.shape[1] else None))
            if not (0<=iy<self.grid.shape[0] and 0<=ix<self.grid.shape[1]):return False,'ROUTE_NOT_OBSERVED'
            value=self.grid[iy,ix]
            if value<0:
                if local_ground_observed and not turning and isolated_free_surround(self.grid,ix,iy,self.res):
                    self.last_check['supported_gaps'].append([int(ix),int(iy)])
                else:return False,'ROUTE_NOT_OBSERVED'
            if value>=50:return False,'ROUTE_OCCUPIED'
        path=self.Path();path.header.frame_id='map'
        from as2_ros_sensors import stamp
        path.header.stamp=stamp(t)
        for xy in (pos[:2],target):
            pose=self.PoseStamped();pose.header=path.header
            mx,my=self.world_to_map(xy);pose.pose.position.x=float(mx);pose.pose.position.y=float(my);pose.pose.orientation.w=1.
            path.poses.append(pose)
        self.pub.publish(path)
        return True,'FIXED_ROUTE_ISOLATED_GRID_GAP_SUPPORTED' if self.last_check['supported_gaps'] else 'FIXED_ROUTE_MAP_CLEAR'
