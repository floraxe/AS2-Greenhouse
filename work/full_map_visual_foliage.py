"""RGB-projected foliage evidence with local measured-point memory.

No scene names, semantic ground truth, depth image, or reference pose input.
RGB green appearance is a simulation prototype, not a trained stem segmenter.
"""
from collections import deque
import math
import numpy as np
from floor_stabilization import floor_transform


class VisualFoliageMemory:
    def __init__(self):
        self.frames = deque()
        self.last_stamp = None
        self.keys = np.empty(0, dtype=np.int64)
        self.last_pose = None
        self.diagnostic = {}

    @staticmethod
    def encode(cells):
        p = np.asarray(cells, dtype=np.int64) + 4096
        return (p[:, 0] * 8192 + p[:, 1]) * 8192 + p[:, 2]

    def classify(self, points, plane, pose, rgb, body_to_camera, stamp, image_stamp, now):
        p = np.asarray(points, dtype=float)
        result = np.c_[p[:, :4], np.zeros(len(p))]
        if plane is None or now-stamp > .4 or rgb is None:
            return result
        T = floor_transform(plane)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        rotation = np.array([[c, -s], [s, c]])
        level = p[:, :3] @ T[:3, :3].T + T[:3, 3]
        task = level.copy()
        task[:, :2] = level[:, :2] @ rotation.T + pose[:2]
        direct = np.zeros(len(p), dtype=bool)
        if self.last_stamp != stamp:
            if self.last_pose is not None and (np.linalg.norm(np.array(pose[:2])-self.last_pose[:2]) > .2 or
                    abs((pose[2]-self.last_pose[2]+math.pi) % (2*math.pi)-math.pi) > .2):
                self.frames.clear()
            if abs(image_stamp-stamp) <= .03:
                cam = p[:, :3] @ body_to_camera[:3, :3] + body_to_camera[3, :3]
                depth = -cam[:, 2]
                valid = depth > .10
                u = np.zeros(len(p), dtype=int)
                v = np.zeros(len(p), dtype=int)
                u[valid] = (320*cam[valid, 0]/depth[valid]+320).astype(int)
                v[valid] = (180-320*cam[valid, 1]/depth[valid]).astype(int)
                valid &= (u >= 0) & (u < 640) & (v >= 0) & (v < 360)
                indices = np.flatnonzero(valid)
                if len(indices):
                    image = np.asarray(rgb)
                    color = image[v[indices], u[indices], :3].astype(float)
                    green = (color[:, 1] > 1.22*color[:, 0]) & (color[:, 1] > 1.10*color[:, 2]) & (color[:, 1] > 35)
                    # Nearest projected lidar point per pixel: no synthetic depth input.
                    pixel = v[indices]*640+u[indices]
                    nearest = np.full(640*360, np.inf)
                    np.minimum.at(nearest, pixel, depth[indices])
                    green &= depth[indices] <= nearest[pixel]+.03
                    direct[indices[green]] = True
                direct &= (np.linalg.norm(level[:, :2], axis=1) < 2.5)
                direct &= (level[:, 2] > .17) & (level[:, 2] < 1.1)
            self.frames.append((stamp, self.encode(np.floor(task[direct]/.025))))
            while self.frames and now-self.frames[0][0] > 20.:
                self.frames.popleft()
            base = np.unique(np.concatenate([keys for _, keys in self.frames])) if self.frames else np.empty(0, dtype=np.int64)
            neighbors = np.array([i*8192*8192+j*8192+k for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)], dtype=np.int64)
            self.keys = np.unique((base[:, None]+neighbors).ravel())
            self.last_stamp, self.last_pose = stamp, np.array(pose)
        if len(self.keys):
            query = self.encode(np.floor(task/.025))
            loc = np.searchsorted(self.keys, query)
            inside = loc < len(self.keys)
            matched = np.zeros(len(p), dtype=bool)
            matched[inside] = self.keys[loc[inside]] == query[inside]
            # Feet/low seedlings and center-lane objects are never softened.
            foot_safe_height = max(.17, T[2, 3]-.17+.03)
            matched &= (level[:, 2] > foot_safe_height) & (level[:, 2] < 1.1) & (abs(p[:, 1]) > .14)
            result[:, 4] = matched
        self.diagnostic = dict(source='ACTUAL_RGB_GREEN_PROJECTION_AND_MEASURED_LIDAR_MEMORY',
                               directly_colored_points=int(direct.sum()),
                               foliage_points=int(result[:, 4].sum()),
                               points=len(p), memory_frames=len(self.frames))
        return result
