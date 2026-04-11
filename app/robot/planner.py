"""A* path planner on a 2-D occupancy grid.

All functions are pure (no side effects) and operate on numpy arrays so
they can be called from any thread.
"""

import heapq
import math
from typing import Dict, List, Optional, Tuple

import numpy as np


def inflate_occupancy(occupancy: np.ndarray, radius: int) -> np.ndarray:
    """Grow every occupied cell by *radius* cells (obstacle inflation)."""
    if radius <= 0:
        return occupancy.copy()
    h, w = occupancy.shape
    out = occupancy.copy()
    for y, x in np.argwhere(occupancy > 0):
        y0, y1 = max(0, y - radius), min(h, y + radius + 1)
        x0, x1 = max(0, x - radius), min(w, x + radius + 1)
        out[y0:y1, x0:x1] = 1
    return out


def astar(
    occupancy: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
) -> List[Tuple[int, int]]:
    """Return a cell path from *start* to *goal*, or [] if unreachable.

    *occupancy* is a 2-D uint8 array where 1 = obstacle, 0 = free.
    Coordinates are (col, row) i.e. (x, y) in grid space.
    """
    h, w = occupancy.shape
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
        return []
    if occupancy[sy, sx] or occupancy[gy, gx]:
        return []
    if start == goal:
        return [start]

    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap: list = [(0.0, start)]
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    g_score: Dict[Tuple[int, int], float] = {start: 0.0}
    visited: set = set()
    neighbours = [(-1, 0), (1, 0), (0, -1), (0, 1),
                  (-1, -1), (-1, 1), (1, -1), (1, 1)]

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        cx, cy = current
        for dx, dy in neighbours:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < w and 0 <= ny < h):
                continue
            if occupancy[ny, nx]:
                continue
            tentative = g_score[current] + math.hypot(dx, dy)
            if tentative < g_score.get((nx, ny), float("inf")):
                came_from[(nx, ny)] = current
                g_score[(nx, ny)] = tentative
                f = tentative + heuristic((nx, ny), goal)
                heapq.heappush(open_heap, (f, (nx, ny)))
    return []


def simplify_path(
    path: List[Tuple[float, float]],
    spacing: float = 0.20,
) -> List[Tuple[float, float]]:
    """Downsample a world-coordinate path so consecutive points are ≥ *spacing* m apart."""
    if not path:
        return []
    out = [path[0]]
    for pt in path[1:]:
        if math.hypot(pt[0] - out[-1][0], pt[1] - out[-1][1]) >= spacing:
            out.append(pt)
    if out[-1] != path[-1]:
        out.append(path[-1])
    return out
