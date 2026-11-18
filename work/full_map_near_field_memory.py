"""Short-term measured obstacle memory; estimated planar pose + lidar floor only."""
from collections import deque
import math
import numpy as np
from floor_stabilization import floor_transform

def voxel_first_indices(cells):
 cells=np.asarray(cells,dtype=np.int32)
 if not len(cells):return np.empty(0,dtype=np.int64)
 low=cells.min(axis=0).astype(np.int64);high=cells.max(axis=0).astype(np.int64)
 spans=high-low+1
 if int(spans[0])*int(spans[1])*int(spans[2])>np.iinfo(np.int64).max:
  return np.unique(cells,axis=0,return_index=True)[1]
 shifted=cells.astype(np.int64)-low
 keys=(shifted[:,0]*spans[1]+shifted[:,1])*spans[2]+shifted[:,2]
 return np.unique(keys,return_index=True)[1]

class NearFieldMemory:
 def __init__(self):self.scans=deque();self.last_scan=None;self.last_pose=None;self.cached=np.empty((0,3))
 def points(self,raw,plane,pose,stamp,now):
  raw=np.asarray(raw,dtype=float);pose=np.asarray(pose,dtype=float)
  if plane is None or stamp<0 or now-stamp>.4:return raw
  T=floor_transform(plane);c,s=math.cos(pose[2]),math.sin(pose[2]);R=np.array([[c,-s],[s,c]])
  changed=False
  if self.last_scan!=stamp:
   changed=True
   if self.last_pose is not None and (np.linalg.norm(pose[:2]-self.last_pose[:2])>.2 or abs((pose[2]-self.last_pose[2]+math.pi)%(2*math.pi)-math.pi)>.2):self.scans.clear()
   level=raw[:,:3]@T[:3,:3].T+T[:3,3]
   keep=(level[:,2]>.008)&(level[:,2]<1.1)&(np.linalg.norm(level[:,:2],axis=1)<1.5)
   q=level[keep].copy();q[:,:2]=q[:,:2]@R.T+pose[:2]
   self.scans.append((stamp,q));self.last_scan=stamp;self.last_pose=pose.copy()
  while self.scans and now-self.scans[0][0]>5.:self.scans.popleft();changed=True
  if changed:
   self.cached=np.concatenate([v for _,v in self.scans]) if self.scans else np.empty((0,3))
   if len(self.cached):
    ids=voxel_first_indices(np.floor(self.cached/.005).astype(np.int32));self.cached=self.cached[ids]
  if not len(self.cached):return raw
  q=self.cached.copy();q[:,:2]=(q[:,:2]-pose[:2])@R
  q=q[np.linalg.norm(q[:,:2],axis=1)<1.2]
  body=(q-T[:3,3])@T[:3,:3]
  return np.r_[raw,np.c_[body,q[:,2]]]
