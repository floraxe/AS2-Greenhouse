"""Measured-floor 3D turn sweep, using the complete frozen gait envelope.

No renderer identities, world poses, crop masks or simulator truth are inputs.
"""
import math
import numpy as np
from floor_stabilization import floor_transform
from full_map_guard import INFLATED, ORIGIN, RES, dilate, grid_bounds, hits

# The route controller may correct its endpoint position while turning.
# Cover its 6cm arrival tolerance in both horizontal axes, in addition to the
# existing 5cm tracking margin on the 54-shape / 4320-gait-pose envelope.
TURN_VOLUME = dilate(dilate(INFLATED, 0, math.ceil(.06 / RES)), 1, math.ceil(.06 / RES))
for _axis in range(3):
    TURN_VOLUME = dilate(TURN_VOLUME, _axis, 1)
LOW, HIGH = grid_bounds(TURN_VOLUME)
RADIUS = float(np.linalg.norm(np.maximum(abs(LOW[:2]), abs(HIGH[:2]))))


def evaluate_turn_sweep(body_points, plane, age, yaw_delta):
    out = dict(safe=False, reason='TURN_CLOUD_UNOBSERVED', ground_points=0,
               observed_quadrants=0, obstacle_points=0, physical_envelope_points=0,
               envelope='54_SHAPES_7200_GAIT_POSES_3D', tracking_margin_m=.05,
               endpoint_translation_allowance_m=.06, sweep_radius_m=RADIUS)
    p = np.asarray(body_points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 100 or not np.isfinite(p).all():
        return out
    if not math.isfinite(age) or not 0 <= age <= .4:
        return dict(out, reason='TURN_CLOUD_STALE')
    if not math.isfinite(yaw_delta) or abs(yaw_delta) > math.pi + 1e-6:
        return dict(out, reason='TURN_ANGLE_INVALID')
    try:
        transform = floor_transform(plane)
    except (ValueError, TypeError):
        return dict(out, reason='TURN_MEASURED_FLOOR_UNAVAILABLE')
    q = p @ transform[:3, :3].T
    height = q[:, 2] + transform[2, 3]
    radius = np.linalg.norm(q[:, :2], axis=1)
    ground = (height < .025) & (height > -.025) & (radius > .3) & (radius < 2.)
    quadrants = np.floor((np.arctan2(q[ground, 1], q[ground, 0]) + math.pi) /
                         (math.pi / 2)).astype(int).clip(0, 3)
    out.update(ground_points=int(ground.sum()), observed_quadrants=len(set(quadrants)),
               measured_body_height_m=float(transform[2, 3]),
               envelope_top_above_floor_m=float(HIGH[2] + transform[2, 3]),
               yaw_delta_rad=float(yaw_delta))
    if out['ground_points'] < 32 or out['observed_quadrants'] < 4:
        return dict(out, reason='TURN_GROUND_COVERAGE_INCOMPLETE')
    # This is a conservative spatial broad phase, not semantic suppression.
    # Points above the robot remain in SLAM, but cannot hit its 3D turn volume.
    relevant = ((height > .008) & (q[:, 2] >= LOW[2]) & (q[:, 2] <= HIGH[2]) &
                (radius <= RADIUS))
    obstacles = q[relevant]
    steps = max(2, int(math.ceil(abs(yaw_delta) * RADIUS / .002)) + 1)
    # Additional 5mm voxel dilation covers <2mm movement between angular samples.
    # INFLATED already exceeds the physical body by 5cm in every axis.
    count = 0
    if len(obstacles):
        for angle in np.linspace(0., yaw_delta, steps):
            c, s = math.cos(angle), math.sin(angle)
            rotated = obstacles.copy()
            rotated[:, 0] = c * obstacles[:, 0] + s * obstacles[:, 1]
            rotated[:, 1] = -s * obstacles[:, 0] + c * obstacles[:, 1]
            count = hits(rotated, TURN_VOLUME)
            if count:
                break
    out.update(candidate_points=len(obstacles), obstacle_points=count,
               physical_envelope_points=count, angular_samples=steps)
    return dict(out, safe=count == 0,
                reason='TURN_3D_SWEEP_CLEAR' if count == 0 else 'TURN_3D_SWEEP_OCCUPIED')
