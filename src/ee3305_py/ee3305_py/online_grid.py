"""ROS-independent geometry for incremental SLAM navigation.

Unknown space is never traversable. Coordinates always refer to the current
grid metadata, so map growth and origin shifts do not reuse stale indices.
"""
from collections import deque
from dataclasses import dataclass
from heapq import heappop, heappush
from math import ceil, cos, floor, hypot, isfinite, sin, sqrt


@dataclass
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: list

    def __post_init__(self):
        if (self.width <= 0 or self.height <= 0 or self.resolution <= 0
                or not all(isfinite(v) for v in (self.resolution, self.origin_x, self.origin_y))
                or len(self.data) != self.width * self.height):
            raise ValueError('Invalid occupancy-grid dimensions or resolution')

    def cell(self, x, y):
        return (floor((x - self.origin_x) / self.resolution),
                floor((y - self.origin_y) / self.resolution))

    def xy(self, c, r):
        return (self.origin_x + (c + .5) * self.resolution,
                self.origin_y + (r + .5) * self.resolution)

    def index(self, c, r):
        return r * self.width + c

    def inside(self, c, r):
        return 0 <= c < self.width and 0 <= r < self.height

    def free(self, c, r, limit=90):
        return self.inside(c, r) and 0 <= self.data[self.index(c, r)] < limit

    def neighbors(self, c, r):
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nc, nr = c + dc, r + dr
            if not self.free(nc, nr):
                continue
            if dc and dr and (not self.free(c + dc, r)
                              or not self.free(c, r + dr)):
                continue
            yield nc, nr


def _distances(grid, sources, maximum):
    """Conservative distance to source cell area, using an octile flood.

    Octile distance / 1.083 never exceeds Euclidean centre distance. Subtract
    a half cell diagonal to include obstacle cell area, not just its centre.
    """
    distances = [float('inf')] * len(grid.data)
    heap = []
    for idx in sources:
        distances[idx] = 0.
        heappush(heap, (0., idx))
    cutoff = (maximum + grid.resolution / sqrt(2)) * 1.083
    while heap:
        dist, idx = heappop(heap)
        if dist != distances[idx] or dist > cutoff:
            continue
        c, r = idx % grid.width, idx // grid.width
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nc, nr = c + dc, r + dr
            if not grid.inside(nc, nr):
                continue
            ni = grid.index(nc, nr)
            nd = dist + hypot(dc, dr) * grid.resolution
            if nd < distances[ni] and nd <= cutoff:
                distances[ni] = nd
                heappush(heap, (nd, ni))
    return [max(0., x / 1.083 - grid.resolution / sqrt(2)) for x in distances]


def inflate(grid, robot_radius=.18, inflation_radius=.4, free_threshold=25,
            occupied_threshold=65):
    if not 0 <= free_threshold < occupied_threshold <= 100:
        raise ValueError('Invalid occupancy thresholds')
    if robot_radius <= 0 or inflation_radius <= robot_radius:
        raise ValueError('Inflation must extend beyond the positive robot radius')
    obstacles = [i for i, v in enumerate(grid.data) if v >= occupied_threshold]
    unknown = [i for i, v in enumerate(grid.data)
               if v < 0 or free_threshold < v < occupied_threshold]
    obstacle_dist = _distances(grid, obstacles, inflation_radius)
    unknown_dist = _distances(grid, unknown, robot_radius)
    output = []
    for i, value in enumerate(grid.data):
        if value < 0 or free_threshold < value < occupied_threshold:
            output.append(-1)
        elif value >= occupied_threshold or obstacle_dist[i] <= robot_radius:
            output.append(100)
        elif unknown_dist[i] <= robot_radius:
            output.append(100)
        else:
            c, r = i % grid.width, i // grid.width
            # Outside the published grid is also unknown.
            edge = min(c + .5, r + .5, grid.width - c - .5,
                       grid.height - r - .5) * grid.resolution
            if edge <= robot_radius:
                output.append(100)
            else:
                fraction = (inflation_radius - obstacle_dist[i]) / (
                    inflation_radius - robot_radius)
                output.append(round(80 * max(0., fraction)))
    return Grid(grid.width, grid.height, grid.resolution,
                grid.origin_x, grid.origin_y, output)


def reachable(grid, start):
    """Connected known-free cells and their shortest geometric distances."""
    if not grid.free(*start):
        return {}
    distances = {start: 0.}
    queue = [(0., start)]
    while queue:
        distance, cell = heappop(queue)
        if distance != distances[cell]:
            continue
        for nxt in grid.neighbors(*cell):
            nd = distance + hypot(nxt[0] - cell[0], nxt[1] - cell[1]) * grid.resolution
            if nd < distances.get(nxt, float('inf')):
                distances[nxt] = nd
                heappush(queue, (nd, nxt))
    return distances


def select_target(raw, costmap, robot, goal, excluded=(), goal_weight=1.,
                  min_cluster=3, stand_off=.4, min_distance=.3):
    """Return (world target, is_final_goal), or None if exploration is exhausted.

    Frontiers are clustered in the raw map, then projected onto reachable
    costmap cells within stand_off. This keeps the footprint out of unknown
    cells, unlike driving directly onto a raw frontier boundary.
    """
    distances = reachable(costmap, costmap.cell(*robot))
    gc = costmap.cell(*goal)
    if gc in distances:
        return goal, True
    frontiers = set()
    for r in range(raw.height):
        for c in range(raw.width):
            # Cartographer's newly observed boundary can still have intermediate
            # probability. Use it to locate information boundaries, but only
            # select waypoints from the strictly traversable inflated map.
            if not 0 <= raw.data[raw.index(c, r)] < 65:
                continue
            if any(not raw.inside(c + dc, r + dr)
                   or raw.data[raw.index(c + dc, r + dr)] < 0
                   for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1))):
                frontiers.add((c, r))
    best = None
    radius = ceil(stand_off / raw.resolution)
    offsets = [(dc, dr) for dc in range(-radius, radius + 1)
               for dr in range(-radius, radius + 1)
               if hypot(dc, dr) * raw.resolution <= stand_off]
    while frontiers:
        seed = frontiers.pop()
        cluster, todo = [seed], deque([seed])
        while todo:
            c, r = todo.popleft()
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = c + dc, r + dr
                if nxt in frontiers:
                    frontiers.remove(nxt); cluster.append(nxt); todo.append(nxt)
        if len(cluster) < min_cluster:
            continue
        candidates = set()
        for c, r in cluster:
            for dc, dr in offsets:
                cell = c + dc, r + dr
                if cell in distances:
                    candidates.add(cell)
        for cell in candidates:
            xy = costmap.xy(*cell)
            if hypot(xy[0] - robot[0], xy[1] - robot[1]) < min_distance:
                continue
            if any(hypot(xy[0] - x, xy[1] - y) < .4 for x, y in excluded):
                continue
            score = distances[cell] + goal_weight * hypot(xy[0] - goal[0], xy[1] - goal[1])
            ranked = score, xy
            if best is None or ranked < best:
                best = ranked
    return (best[1], False) if best else None


def trajectory_clear(grid, pose, linear, angular, horizon=.8):
    """Sample a command's arc in the footprint-inflated map, including now."""
    x, y, yaw = pose
    steps = max(1, ceil(abs(linear) * horizon / (grid.resolution / 3)),
                ceil(abs(angular) * horizon / .05))
    for k in range(steps + 1):
        t = horizon * k / steps
        if abs(angular) < 1e-8:
            px, py = x + linear * t * cos(yaw), y + linear * t * sin(yaw)
        else:
            px = x + linear / angular * (sin(yaw + angular * t) - sin(yaw))
            py = y - linear / angular * (cos(yaw + angular * t) - cos(yaw))
        if not grid.free(*grid.cell(px, py)):
            return False
    return True
