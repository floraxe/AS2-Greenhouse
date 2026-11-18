"""Observed-free A* in a surveyed task envelope. No simulator geometry input.

The task supplies a destination; an unseen destination is approached only as far
as currently observed free space permits. Unknown space is never traversable.
"""
import heapq
import math
import numpy as np


class OnlineGridPlanner:
    def __init__(self, radius=.285, task_anchor=(-3., -2.15)):
        self.radius = radius
        self.anchor = np.asarray(task_anchor)
        self.grid = None
        self.stamp = None
        self.revision = 0

    def update(self, data, width, height, resolution, origin, stamp):
        if not (.02 <= resolution <= .20) or width * height > 500000:
            raise ValueError('Unsupported grid dimensions/resolution')
        values = np.asarray(data, dtype=np.int16).reshape(height, width)
        if np.any((values < -1) | (values > 100)):
            raise ValueError('Invalid occupancy')
        self.grid = values
        self.res = resolution
        self.origin = np.asarray(origin[:2], float)
        self.stamp = stamp
        self.revision += 1
        # Inflate occupied and unknown cells. A complete body footprint must be
        # observed, not just its centre. Also treat the map boundary as unknown.
        blocked = (values < 0) | (values >= 50)
        n = int(math.ceil(self.radius / resolution))
        padded = np.pad(blocked, n, constant_values=True)
        inflated = np.zeros_like(blocked)
        for dy in range(-n, n + 1):
            for dx in range(-n, n + 1):
                if math.hypot(dx, dy) * resolution <= self.radius + resolution * .71:
                    inflated |= padded[n+dy:n+dy+height, n+dx:n+dx+width]
        yy, xx = np.indices(values.shape)
        wx = self.origin[0] + (xx+.5)*resolution + self.anchor[0]
        wy = self.origin[1] + (yy+.5)*resolution + self.anchor[1]
        # Prior: one assigned aisle and its headland, never an adjacent aisle.
        envelope = (((wx > -5.9) & (wx < 4.70) & (np.abs(wy+2.15) < .185)) |
                    ((wx >= 4.70) & (wx < 6.80) & (wy > -2.80) & (wy < -.65)) |
                    ((wx > -6.80) & (wx <= -4.70) & (wy > -2.80) & (wy < -.65)))
        self.free = (~inflated) & envelope

    def cell(self, task_xy):
        p = np.floor((np.asarray(task_xy)-self.anchor-self.origin)/self.res).astype(int)
        return (int(p[1]), int(p[0]))

    def xy(self, cell):
        y, x = cell
        return (self.origin + (np.array([x, y])+.5)*self.res + self.anchor).tolist()

    def plan(self, start, goal, sim_time):
        out = dict(valid=False, reason='WAIT_LIVE_GRID', revision=self.revision, path=[])
        if self.grid is None:
            return out
        if not (-.3 <= sim_time-self.stamp <= 3.):
            out['reason'] = 'GRID_TIMEOUT'
            return out
        src = self.cell(start)
        h, w = self.free.shape
        def permitted(c):
            return 0 <= c[0] < h and 0 <= c[1] < w and self.free[c]
        if not permitted(src):
            out['reason'] = 'START_FOOTPRINT_NOT_OBSERVED_FREE'
            return out
        # Best-first search with a rolling observable goal when the destination
        # is still outside the current map. No unknown-cost shortcut is used.
        target = np.asarray(goal)
        def heuristic(c):
            return math.dist(self.xy(c), target)
        queue = [(heuristic(src), 0., src)]
        costs = {src: 0.}
        parents = {}
        best = src
        reached = False
        while queue:
            _, cost, cur = heapq.heappop(queue)
            if cost > costs[cur]+1e-8:
                continue
            if heuristic(cur) < heuristic(best):
                best = cur
            if heuristic(cur) < max(.065, self.res*.8):
                best = cur
                reached = True
                break
            for dy, dx in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt = (cur[0]+dy, cur[1]+dx)
                if not permitted(nxt):
                    continue
                if dx and dy and (not permitted((cur[0]+dy,cur[1])) or not permitted((cur[0],cur[1]+dx))):
                    continue
                trial = cost + self.res*math.hypot(dx,dy)
                if trial < costs.get(nxt, float('inf')):
                    costs[nxt] = trial
                    parents[nxt] = cur
                    heapq.heappush(queue, (trial+heuristic(nxt), trial, nxt))
        if not reached and heuristic(src)-heuristic(best) < .20:
            out['reason'] = 'WAIT_OBSERVED_PROGRESS'
            return out
        cells = [best]
        while cells[-1] != src:
            cells.append(parents[cells[-1]])
        out.update(valid=True, reason='GOAL_PATH' if reached else 'OBSERVED_ROLLING_PATH',
                   path=[self.xy(c) for c in reversed(cells)], goal_observed=reached)
        return out


class RosGridPlanner:
    def __init__(self, node):
        from nav_msgs.msg import OccupancyGrid, Path
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self.core = OnlineGridPlanner()
        self.node = node
        self.Path, self.PoseStamped = Path, PoseStamped
        self.last_wall = None
        self.error = None
        self.cached = None
        self.cached_key = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.sub = node.create_subscription(OccupancyGrid, '/as2/slam/map', self.update, qos)
        self.pub = node.create_publisher(Path, '/as2/planner/path', 1)

    def update(self, msg):
        import time
        q = msg.info.origin.orientation
        if msg.header.frame_id != 'map' or abs(q.x)+abs(q.y)+abs(q.z) > 1e-6 or abs(q.w-1) > 1e-6:
            self.error = 'GRID_FRAME_OR_ROTATION_UNSUPPORTED'
            return
        try:
            self.core.update(msg.data, msg.info.width, msg.info.height, msg.info.resolution,
                             (msg.info.origin.position.x, msg.info.origin.position.y),
                             msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
            self.last_wall = time.monotonic()
            self.error = None
            self.cached_key = None
        except (ValueError, TypeError) as exc:
            self.error = str(exc)

    def plan(self, start, goal, sim_time):
        import time
        if self.error or self.last_wall is None or time.monotonic()-self.last_wall > 12.:
            return dict(valid=False, reason=self.error or 'WAIT_FRESH_GRID', path=[])
        if self.core.stamp is None or sim_time-self.core.stamp > 3.:
            return dict(valid=False, reason='GRID_TIMEOUT', path=[])
        key = (self.core.revision, self.core.cell(start), tuple(goal))
        if key != self.cached_key:
            self.cached = self.core.plan(start, goal, sim_time)
            self.cached_key = key
            path = self.Path()
            from as2_ros_sensors import stamp
            path.header.frame_id = 'map'
            path.header.stamp = stamp(sim_time)
            for xy in self.cached['path']:
                p = self.PoseStamped()
                p.header = path.header
                p.pose.position.x = xy[0]-self.core.anchor[0]
                p.pose.position.y = xy[1]-self.core.anchor[1]
                p.pose.orientation.w = 1.
                path.poses.append(p)
            self.pub.publish(path)
        return self.cached
