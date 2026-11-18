"""Near-field structural points for ICP only; raw obstacle observations stay intact.
The floor plane is estimated from measured points, never simulator world pose.
Assumes an approximately level greenhouse floor and known sensor mounting height.
"""
import numpy as np

def filter_lidar_structure(lidar_points,min_height=.10,max_range=2.,context_range=4.5):
    lidar=np.asarray(lidar_points,dtype=np.float32).reshape(-1,3)
    info=dict(valid=False,reason='INSUFFICIENT_FLOOR_SUPPORT',input_points=len(lidar),output_points=0)
    if len(lidar)<100 or not np.isfinite(lidar).all():return np.empty((0,3),np.float32),info
    body=lidar.astype(float)+[.3,0,.32]
    radius=np.linalg.norm(body[:,:2],axis=1)
    floor=(body[:,2]<-.22)&(radius<3.)&(radius>.4)
    if floor.sum()<30:return np.empty((0,3),np.float32),info
    A=np.column_stack((body[floor,:2],np.ones(int(floor.sum()))));z=body[floor,2]
    for _ in range(4):
        plane=np.linalg.lstsq(A,z,rcond=None)[0]
        error=np.abs(A@plane-z);keep=error<max(.005,float(np.quantile(error,.7)))
        A=A[keep];z=z[keep]
    residual=float(np.sqrt(np.mean((A@plane-z)**2)))
    if residual>.02 or np.linalg.norm(plane[:2])>.35:
        info['reason']='FLOOR_PLANE_UNRELIABLE';return np.empty((0,3),np.float32),info
    heights=body[:,2]-np.column_stack((body[:,:2],np.ones(len(body))))@plane
    points=lidar[(heights>min_height)&(heights<2.)&(radius<max_range)]
    # Preserve near-field detail and a stable, coarse far context across headlands.
    # Selection depends only on measured range/height, never task pose.
    near_count=len(points)
    far=lidar[(heights>min_height)&(heights<2.5)&(radius>=max_range)&(radius<context_range)]
    if len(far):
        _,ix=np.unique(np.floor(far/.10).astype(np.int32),axis=0,return_index=True)
        far=far[np.sort(ix)]
        points=np.concatenate((points,far))
    info.update(near_points=near_count,context_points=len(far),context_range_m=context_range)
    info.update(output_points=len(points),floor_rms_m=residual,range_m=max_range,min_height_m=min_height,floor_plane_body=plane.tolist())
    if len(points)<80:
        info['reason']='INSUFFICIENT_STRUCTURAL_POINTS';return np.empty((0,3),np.float32),info
    info.update(valid=True,reason='STRUCTURE_POINTS_READY')
    return np.ascontiguousarray(points,dtype='<f4'),info
