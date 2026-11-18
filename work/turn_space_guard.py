"""Observable full-surround turning clearance; unknown coverage fails closed."""
import numpy as np

def evaluate_turn_space(cloud, position, age):
    points=np.asarray(cloud,dtype=float).reshape(-1,3)
    result=dict(safe=False,reason='STALE_OR_EMPTY_CLOUD',ground_points=0,
                observed_quadrants=0,obstacle_points=0,sweep_radius_m=.64)
    if age>.4 or len(points)<100:
        return result
    delta=points[:,:2]-np.asarray(position[:2])
    radius=np.linalg.norm(delta,axis=1)
    ground=(points[:,2]<.025)&(radius>.3)&(radius<2.)
    bearings=np.arctan2(delta[ground,1],delta[ground,0])
    quadrants=np.floor((bearings+np.pi)/(np.pi/2)).astype(int).clip(0,3)
    obstacles=(points[:,2]>.025)&(points[:,2]<1.1)&(radius<.64)
    result.update(ground_points=int(ground.sum()),observed_quadrants=len(set(quadrants)),
                  obstacle_points=int(obstacles.sum()))
    if result['obstacle_points']>=3:
        result['reason']='OBSTACLE_INSIDE_TURN_SWEEP'
        result['nearest_obstacle_m']=float(radius[obstacles].min())
    elif result['observed_quadrants']<4 or result['ground_points']<32:
        result['reason']='TURN_GROUND_COVERAGE_INCOMPLETE'
    else:
        result.update(safe=True,reason='TURN_SPACE_CLEAR')
    return result
