"""Advanced global planner with dynamic obstacle handling.

Provides:
- Jump Point Search (JPS) for faster pathfinding
- Dynamic obstacle costmap integration
- Proactive rerouting around detected obstacles
- Path validity checking and recovery

This replaces the basic A* planner for more responsive navigation.
"""

import heapq
import logging
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

log = logging.getLogger(__name__)


@dataclass
class GridNode:
    x: int
    y: int
    g: float = 0.0
    h: float = 0.0
    f: float = 0.0
    parent: Optional['GridNode'] = None
    direction: Tuple[int, int] = (0, 0)

    def __lt__(self, other: 'GridNode') -> bool:
        return self.f < other.f


class DynamicPlanner:
    """Advanced path planner with JPS and dynamic obstacle support."""

    def __init__(
        self,
        grid_size: int = 2400,
        resolution: float = 0.02,
        inflation_radius: int = 5,
        dynamic_inflation: int = 3,
    ) -> None:
        self.grid_size = grid_size
        self.resolution = resolution
        self.inflation_radius = inflation_radius
        self.dynamic_inflation = dynamic_inflation

        self._occ_grid: Optional[np.ndarray] = None
        self._dynamic_obs: Set[Tuple[int, int]] = set()
        self._last_plan_time: float = 0.0
        self._plan_count: int = 0

    def update_static_map(self, occupancy: np.ndarray) -> None:
        """Update the static occupancy grid."""
        self._occ_grid = occupancy.copy()

    def update_dynamic_obstacles(
        self,
        pose: Tuple[float, float, float],
        scan: List[Tuple[float, float]],
        grid_origin: Tuple[float, float],
    ) -> None:
        """Update dynamic obstacles from live LiDAR scan."""
        self._dynamic_obs.clear()

        if not scan:
            return

        rx, ry, _ = pose
        origin_x, origin_y = grid_origin

        for angle, dist in scan:
            if dist <= 0.12 or dist >= 8.0:
                continue

            global_angle = pose[2] + angle
            wx = pose[0] + dist * math.cos(global_angle)
            wy = pose[1] + dist * math.sin(global_angle)

            gx = int((wx - origin_x) / self.resolution)
            gy = int((wy - origin_y) / self.resolution)

            for dy in range(-self.dynamic_inflation, self.dynamic_inflation + 1):
                for dx in range(-self.dynamic_inflation, self.dynamic_inflation + 1):
                    self._dynamic_obs.add((gx + dx, gy + dy))

    def is_cell_free(self, x: int, y: int) -> bool:
        """Check if a cell is free (static + dynamic obstacles)."""
        if self._occ_grid is None:
            return True

        h, w = self._occ_grid.shape
        if not (0 <= x < w and 0 <= y < h):
            return False

        if self._occ_grid[y, x] > 0:
            return False

        if (x, y) in self._dynamic_obs:
            return False

        return True

    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        grid_origin: Tuple[float, float],
        use_jps: bool = True,
    ) -> List[Tuple[float, float]]:
        """Plan a path from start to goal."""
        if self._occ_grid is None:
            log.warning("No occupancy grid available for planning")
            return []

        start_g = self._world_to_grid(start[0], start[1], grid_origin)
        goal_g = self._world_to_grid(goal[0], goal[1], grid_origin)

        start_g = self._find_nearest_free(start_g)
        goal_g = self._find_nearest_free(goal_g)

        if start_g is None or goal_g is None:
            log.warning("Could not find free start or goal cell")
            return []

        path_cells = self._jps_plan(start_g, goal_g) if use_jps else self._astar_plan(start_g, goal_g)

        if not path_cells:
            log.warning("No path found from %s to %s", start_g, goal_g)
            return []

        path_world = [self._grid_to_world(gx, gy, grid_origin) for gx, gy in path_cells]

        self._last_plan_time = self._plan_count
        self._plan_count += 1

        return self._optimize_path(path_world)

    def _world_to_grid(
        self,
        wx: float,
        wy: float,
        origin: Tuple[float, float],
    ) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates."""
        gx = int((wx - origin[0]) / self.resolution)
        gy = int((wy - origin[1]) / self.resolution)
        return gx, gy

    def _grid_to_world(
        self,
        gx: int,
        gy: int,
        origin: Tuple[float, float],
    ) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates."""
        wx = gx * self.resolution + origin[0]
        wy = gy * self.resolution + origin[1]
        return wx, wy

    def _find_nearest_free(self, cell: Tuple[int, int], radius: int = 15) -> Optional[Tuple[int, int]]:
        """Find nearest free cell within radius."""
        sx, sy = cell

        if self.is_cell_free(sx, sy):
            return cell

        for r in range(1, radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dy) != r and abs(dx) != r:
                        continue
                    nx, ny = sx + dx, sy + dy
                    if self.is_cell_free(nx, ny):
                        return nx, ny

        return None

    def _jps_plan(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        """Jump Point Search - faster than A* on grid maps."""
        if not self.is_cell_free(*start) or not self.is_cell_free(*goal):
            return []

        open_set: List[GridNode] = [GridNode(start[0], start[1], h=self._heuristic(start, goal))]
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_scores: Dict[Tuple[int, int], float] = {start: 0.0}
        visited: Set[Tuple[int, int]] = set()

        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ]

        while open_set:
            current = heapq.heappop(open_set)
            cpos = (current.x, current.y)

            if cpos in visited:
                continue
            visited.add(cpos)

            if cpos == goal:
                return self._reconstruct_path(came_from, cpos)

            for dx, dy in directions:
                is_diagonal = dx != 0 and dy != 0

                jump_point = self._jump(
                    current.x, current.y, dx, dy, goal, visited
                )

                if jump_point is None:
                    continue

                jx, jy = jump_point
                jpos = (jx, jy)

                move_cost = math.hypot(jx - current.x, jy - current.y)
                tentative_g = g_scores.get(cpos, float('inf')) + move_cost

                if jpos not in g_scores or tentative_g < g_scores[jpos]:
                    g_scores[jpos] = tentative_g
                    h = self._heuristic(jpos, goal)
                    f = tentative_g + h

                    node = GridNode(jx, jy, g=tentative_g, h=h, f=f, parent=cpos)
                    heapq.heappush(open_set, node)
                    came_from[jpos] = cpos

        return []

    def _jump(
        self,
        x: int,
        y: int,
        dx: int,
        dy: int,
        goal: Tuple[int, int],
        visited: Set[Tuple[int, int]],
    ) -> Optional[Tuple[int, int]]:
        """JPS jump function - find next jump point."""
        is_diagonal = dx != 0 and dy != 0

        while True:
            x += dx
            y += dy

            if not self.is_cell_free(x, y):
                return None

            if (x, y) == goal:
                return (x, y)

            if visited and (x, y) in visited:
                return None

            if is_diagonal:
                if self._is_forced(x - dx, y, dx, dy) and not self.is_cell_free(x - dx, y):
                    return (x, y)
                if self._is_forced(x, y - dy, dx, dy) and not self.is_cell_free(x, y - dy):
                    return (x, y)
            else:
                if dx != 0:
                    if self._is_forced(x, y - 1, dx, dy) and not self.is_cell_free(x, y - 1):
                        return (x, y)
                    if self._is_forced(x, y + 1, dx, dy) and not self.is_cell_free(x, y + 1):
                        return (x, y)
                else:
                    if self._is_forced(x - 1, y, dx, dy) and not self.is_cell_free(x - 1, y):
                        return (x, y)
                    if self._is_forced(x + 1, y, dx, dy) and not self.is_cell_free(x + 1, y):
                        return (x, y)

            if not self.is_cell_free(x + dx, y) or not self.is_cell_free(x, y + dy):
                return (x, y)

    def _is_forced(self, x: int, y: int, dx: int, dy: int) -> bool:
        """Check if a neighbor is forced."""
        if self.is_cell_free(x, y):
            return False

        is_diagonal = dx != 0 and dy != 0

        if is_diagonal:
            return not self.is_cell_free(x - dx, y) or not self.is_cell_free(x, y - dy)
        else:
            return False

    def _astar_plan(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        """Standard A* fallback."""
        if not self.is_cell_free(*start) or not self.is_cell_free(*goal):
            return []

        open_set: List[Tuple[float, int, int]] = [(0.0, start[0], start[1])]
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_scores: Dict[Tuple[int, int], float] = {start: 0.0}
        visited: Set[Tuple[int, int]] = set()

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_set:
            _, x, y = heapq.heappop(open_set)
            current = (x, y)

            if current in visited:
                continue
            visited.add(current)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for dx, dy in directions:
                nx, ny = x + dx, y + dy

                if not self.is_cell_free(nx, ny):
                    continue

                move_cost = math.hypot(dx, dy)
                tentative_g = g_scores.get(current, float('inf')) + move_cost

                if tentative_g < g_scores.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = current
                    g_scores[(nx, ny)] = tentative_g
                    h = self._heuristic((nx, ny), goal)
                    heapq.heappush(open_set, (tentative_g + h, nx, ny))

        return []

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Octile distance heuristic."""
        return math.hypot(abs(a[0] - b[0]), abs(a[1] - b[1]))

    def _reconstruct_path(
        self,
        came_from: Dict[Tuple[int, int], Tuple[int, int]],
        current: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        """Reconstruct path from goal to start."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _optimize_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Remove redundant waypoints from path."""
        if len(path) < 3:
            return path

        optimized = [path[0]]
        last_direction = None

        for i in range(1, len(path) - 1):
            p_prev = optimized[-1]
            p_curr = path[i]
            p_next = path[i + 1]

            dir_curr = (p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
            dir_next = (p_next[0] - p_curr[0], p_next[1] - p_curr[1])

            cross = dir_curr[0] * dir_next[1] - dir_curr[1] * dir_next[0]
            dot = dir_curr[0] * dir_next[0] + dir_curr[1] * dir_next[1]

            if abs(cross) > 0.01 or dot < 0:
                optimized.append(p_curr)

        optimized.append(path[-1])

        final = [optimized[0]]
        for pt in optimized[1:]:
            dist = math.hypot(pt[0] - final[-1][0], pt[1] - final[-1][1])
            if dist > 0.15:
                final.append(pt)

        if final[-1] != path[-1]:
            final.append(path[-1])

        return final


def inflate_obstacles(
    occupancy: np.ndarray,
    radius: int,
) -> np.ndarray:
    """Inflate obstacles for robot clearance."""
    if radius <= 0:
        return occupancy.copy()

    h, w = occupancy.shape
    out = occupancy.copy()

    kernel_size = 2 * radius + 1
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)

    try:
        from scipy.ndimage import binary_dilation
        obstacles = occupancy > 0
        inflated = binary_dilation(obstacles, structure=kernel)
        out = inflated.astype(np.uint8)
    except ImportError:
        for y in range(h):
            for x in range(w):
                if occupancy[y, x] > 0:
                    y0 = max(0, y - radius)
                    y1 = min(h, y + radius + 1)
                    x0 = max(0, x - radius)
                    x1 = min(w, x + radius + 1)
                    out[y0:y1, x0:x1] = 1

    return out


def nearest_free_point(
    occupancy: np.ndarray,
    point: Tuple[int, int],
    max_radius: int = 20,
) -> Optional[Tuple[int, int]]:
    """Find nearest free cell to given point."""
    sx, sy = point
    h, w = occupancy.shape

    if 0 <= sx < w and 0 <= sy < h and occupancy[sy, sx] == 0:
        return point

    for r in range(1, max_radius + 1):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if abs(dy) != r and abs(dx) != r:
                    continue
                nx, ny = sx + dx, sy + dy
                if 0 <= nx < w and 0 <= ny < h and occupancy[ny, nx] == 0:
                    return (nx, ny)

    return None


def downsample_path(
    path: List[Tuple[float, float]],
    min_spacing: float = 0.1,
) -> List[Tuple[float, float]]:
    """Downsample path to minimum spacing between points."""
    if not path:
        return []

    if len(path) < 2:
        return path.copy()

    result = [path[0]]

    for pt in path[1:]:
        last = result[-1]
        dist = math.hypot(pt[0] - last[0], pt[1] - last[1])
        if dist >= min_spacing:
            result.append(pt)

    if result[-1] != path[-1]:
        result.append(path[-1])

    return result
