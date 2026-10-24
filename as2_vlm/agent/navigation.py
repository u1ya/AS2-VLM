"""Deterministic navigation: belief occupancy grid, A*, waypoint following, corridor check.

The grid is built from what the robot *believes* (static floor plan + semantic map door
and obstacle states), never from simulator truth. The VLM never produces velocities.
"""
import heapq
import math

from .sim_world import point_segment_distance, rect_edges, wrap

BLOCKING_DOOR_STATES = {"closed", "locked"}
BLOCKING_OBJECT_STATES = {"blocking"}


class OccupancyGrid:
    def __init__(self, bounds, resolution=0.1):
        self.x0, self.y0, x1, y1 = bounds
        self.res = resolution
        self.nx = max(1, int(math.ceil((x1 - self.x0) / resolution)))
        self.ny = max(1, int(math.ceil((y1 - self.y0) / resolution)))
        self.cells = bytearray(self.nx * self.ny)

    def index(self, x, y):
        i, j = int((x - self.x0) / self.res), int((y - self.y0) / self.res)
        return (i, j) if 0 <= i < self.nx and 0 <= j < self.ny else None

    def center(self, i, j):
        return self.x0 + (i + 0.5) * self.res, self.y0 + (j + 0.5) * self.res

    def occupied_cell(self, i, j):
        return not (0 <= i < self.nx and 0 <= j < self.ny) or self.cells[j * self.nx + i] != 0

    def free(self, x, y):
        cell = self.index(x, y)
        return cell is not None and not self.occupied_cell(*cell)

    def _cells_near(self, xmin, ymin, xmax, ymax):
        i0 = max(0, int((xmin - self.x0) / self.res))
        i1 = min(self.nx - 1, int((xmax - self.x0) / self.res))
        j0 = max(0, int((ymin - self.y0) / self.res))
        j1 = min(self.ny - 1, int((ymax - self.y0) / self.res))
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                yield i, j

    def mark_segment(self, x1, y1, x2, y2, inflate):
        for i, j in self._cells_near(min(x1, x2) - inflate, min(y1, y2) - inflate,
                                     max(x1, x2) + inflate, max(y1, y2) + inflate):
            cx, cy = self.center(i, j)
            if point_segment_distance(cx, cy, x1, y1, x2, y2) <= inflate:
                self.cells[j * self.nx + i] = 1

    def mark_circle(self, x, y, radius):
        for i, j in self._cells_near(x - radius, y - radius, x + radius, y + radius):
            cx, cy = self.center(i, j)
            if math.hypot(cx - x, cy - y) <= radius:
                self.cells[j * self.nx + i] = 1

    def mark_border(self, inflate):
        x1, y1 = self.x0 + self.nx * self.res, self.y0 + self.ny * self.res
        for seg in ((self.x0, self.y0, x1, self.y0), (x1, self.y0, x1, y1),
                    (x1, y1, self.x0, y1), (self.x0, y1, self.x0, self.y0)):
            self.mark_segment(*seg, inflate)

    def line_free(self, a, b):
        distance = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(1, int(distance / (self.res * 0.5)))
        return all(self.free(a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps)
                   for k in range(steps + 1))

    def nearest_free(self, x, y, max_radius=0.8):
        if self.free(x, y):
            return x, y
        best = None
        for i, j in self._cells_near(x - max_radius, y - max_radius, x + max_radius, y + max_radius):
            if not self.occupied_cell(i, j):
                cx, cy = self.center(i, j)
                d = math.hypot(cx - x, cy - y)
                if d <= max_radius and (best is None or d < best[0]):
                    best = (d, cx, cy)
        return None if best is None else (best[1], best[2])


def astar(grid, start, goal):
    s, g = grid.index(*start), grid.index(*goal)
    if s is None or g is None or grid.occupied_cell(*s) or grid.occupied_cell(*g):
        return None
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]

    def h(c):
        dx, dy = abs(c[0] - g[0]), abs(c[1] - g[1])
        return max(dx, dy) + 0.4142 * min(dx, dy)

    open_heap = [(h(s), 0.0, s)]
    came, cost = {s: None}, {s: 0.0}
    while open_heap:
        _, c, cell = heapq.heappop(open_heap)
        if cell == g:
            path = []
            while cell is not None:
                path.append(grid.center(*cell))
                cell = came[cell]
            path.reverse()
            path[0], path[-1] = tuple(start), tuple(goal)
            return path
        if c > cost[cell]:
            continue
        for di, dj, w in moves:
            n = (cell[0] + di, cell[1] + dj)
            if grid.occupied_cell(*n):
                continue
            if di and dj and (grid.occupied_cell(cell[0] + di, cell[1]) or grid.occupied_cell(cell[0], cell[1] + dj)):
                continue  # no corner cutting
            nc = c + w
            if nc < cost.get(n, math.inf):
                cost[n], came[n] = nc, cell
                heapq.heappush(open_heap, (nc + h(n), nc, n))
    return None


def smooth(grid, path):
    if len(path) <= 2:
        return list(path)
    result, anchor = [path[0]], 0
    while anchor < len(path) - 1:
        nxt = len(path) - 1
        while nxt > anchor + 1 and not grid.line_free(path[anchor], path[nxt]):
            nxt -= 1
        result.append(path[nxt])
        anchor = nxt
    return result


def advance_waypoint(path, index, pose, reach=0.15):
    """Next waypoint index: advance when the current one is reached or already passed."""
    while index < len(path) - 1:
        wx, wy = path[index]
        nx, ny = path[index + 1]
        distance = math.hypot(wx - pose.x, wy - pose.y)
        passed = (pose.x - wx) * (nx - wx) + (pose.y - wy) * (ny - wy) > 0 and distance < 0.4
        if distance >= reach and not passed:
            break
        index += 1
    return index


def path_length(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


class Navigator:
    """Plans on the robot's belief. Rebuilds the grid when the semantic map version changes."""

    def __init__(self, motion, static_map, scene_graph, resolution=0.1):
        self.motion = motion
        self.static_map = static_map
        self.graph = scene_graph
        self.resolution = resolution
        self.inflate = motion.robot_radius_m + 0.15
        self._cache = {}

    def _bounds(self, extra_points):
        if self.static_map:
            return tuple(self.static_map["bounds"])
        points = [n.coords[:2] for n in self.graph.get_all_nodes()] + [list(p) for p in extra_points]
        xs, ys = [p[0] for p in points] or [0.0], [p[1] for p in points] or [0.0]
        return min(xs) - 4, min(ys) - 4, max(xs) + 4, max(ys) + 4

    def blockers(self, ignore=()):
        """Semantic nodes that currently block motion in the robot's belief."""
        result = []
        for node in self.graph.get_all_nodes():
            if node.id in ignore:
                continue
            if node.category == "door" and node.state in BLOCKING_DOOR_STATES:
                result.append(node)
            elif node.state in BLOCKING_OBJECT_STATES:
                result.append(node)
        return result

    def signature(self, ignore=()):
        """Changes only when the set/state/position of blocking beliefs changes."""
        return tuple((n.id, n.state, round(n.coords[0], 1), round(n.coords[1], 1)) for n in self.blockers(ignore))

    def grid(self, ignore=(), extra_points=()):
        key = (self.signature(ignore), tuple(sorted(ignore)),
               None if self.static_map else tuple((round(p[0]), round(p[1])) for p in extra_points))
        if key in self._cache:
            return self._cache[key]
        grid = OccupancyGrid(self._bounds(extra_points), self.resolution)
        inflate = self.inflate
        if self.static_map:
            grid.mark_border(inflate)
            for wall in self.static_map.get("walls", []):
                grid.mark_segment(*wall, inflate)
            for door in self.static_map.get("doors", []):
                if door.get("exterior"):
                    grid.mark_segment(*door["segment"], inflate)
            for item in self.static_map.get("furniture", []):
                for edge in rect_edges(*item["center"], *item["half"]):
                    grid.mark_segment(*edge, inflate)
        for node in self.blockers(ignore):
            segment = node.extra.get("segment")
            if segment:
                grid.mark_segment(*segment, inflate)
            else:
                grid.mark_circle(node.coords[0], node.coords[1], float(node.extra.get("radius", 0.4)) + inflate)
        if len(self._cache) > 16:
            self._cache.clear()
        self._cache[key] = grid
        return grid

    def plan(self, start, goal, ignore=()):
        grid = self.grid(ignore, (start, goal))
        s = grid.nearest_free(*start[:2], max_radius=0.6)
        g = grid.nearest_free(*goal[:2], max_radius=0.6)
        if s is None or g is None:
            return None
        raw = astar(grid, s, g)
        if raw is None:
            return None
        path = smooth(grid, raw)
        path[0] = (start[0], start[1])  # path ends at the goal, or its nearest free cell
        return [(round(x, 3), round(y, 3)) for x, y in path]

    def path_still_valid(self, path, from_index, ignore=()):
        grid = self.grid(ignore, (path[0], path[-1]))
        return all(grid.line_free(a, b) for a, b in zip(path[from_index:], path[from_index + 1:]))

    def is_free(self, x, y):
        return self.grid((), ((x, y),)).free(x, y)

    def nearest_free(self, x, y, max_radius=0.8):
        return self.grid((), ((x, y),)).nearest_free(x, y, max_radius)

    def wall_between(self, a, b):
        """True when a static wall segment crosses the straight line a->b."""
        if not self.static_map:
            return False
        ax, ay = a[:2]
        bx, by = b[:2]
        for x1, y1, x2, y2 in self.static_map.get("walls", []):
            d1 = (bx - ax) * (y1 - ay) - (by - ay) * (x1 - ax)
            d2 = (bx - ax) * (y2 - ay) - (by - ay) * (x2 - ax)
            d3 = (x2 - x1) * (ay - y1) - (y2 - y1) * (ax - x1)
            d4 = (x2 - x1) * (by - y1) - (y2 - y1) * (bx - x1)
            if d1 * d2 < 0 and d3 * d4 < 0:
                return True
        return False


def follow_command(pose, target, final_yaw, is_last, limits):
    """Rotate-then-go controller. Returns (vx, vy, vyaw, status)."""
    dx, dy = target[0] - pose.x, target[1] - pose.y
    distance = math.hypot(dx, dy)
    if is_last and distance < limits.goal_tolerance_m:
        if final_yaw is None:
            return 0.0, 0.0, 0.0, "arrived"
        error = wrap(final_yaw - pose.yaw)
        if abs(error) < limits.yaw_tolerance_rad:
            return 0.0, 0.0, 0.0, "arrived"
        return 0.0, 0.0, turn_rate(error, limits), "aligning"
    error = wrap(math.atan2(dy, dx) - pose.yaw)
    if abs(error) > 0.45:
        return 0.0, 0.0, turn_rate(error, limits), "turning"
    speed = min(limits.max_vx, 0.6 * distance + 0.1) * max(0.0, math.cos(error)) ** 2
    return speed, 0.0, max(-limits.max_vyaw, min(limits.max_vyaw, 2.0 * error)), "driving"


def turn_rate(error, limits):
    rate = max(-limits.max_vyaw, min(limits.max_vyaw, 1.6 * error))
    return math.copysign(max(abs(rate), 0.15), error)


def corridor_obstacle(scan, limits, extra_reach=0.0, heading=0.0):
    """Nearest return inside the body-width corridor along `heading` (body frame, radians, e.g. the
    direction to the current waypoint): (forward_m, bearing_rad) or None. Measured from body centre."""
    reach = limits.robot_radius_m + limits.stop_distance_m + extra_reach
    half = limits.robot_half_width_m + 0.02
    best = None
    for angle, distance in zip(scan.angles, scan.ranges):
        if not math.isfinite(distance) or distance <= 0.05:
            continue
        relative = angle - heading
        forward, lateral = distance * math.cos(relative), distance * math.sin(relative)
        if 0 < forward < reach and abs(lateral) < half and (best is None or forward < best[0]):
            best = (forward, angle)
    return best
