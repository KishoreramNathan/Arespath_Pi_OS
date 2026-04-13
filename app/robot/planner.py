"""A* path planner on a 2-D occupancy grid.

All functions are pure (no side effects) and operate on numpy arrays so
they can be called from any thread.
"""

import heapq
import math
from typing import Dict, List, Optional, Tuple

import numpy as np


def inflate_occupancy(occupancy: np.ndarray, radius: int) -> np.ndarray:
    """Grow every occupied cell by *radius* cells using fast numpy convolution."""
    if radius <= 0:
        return occupancy.copy()
    # Build a square structuring element and use max-pooling via conv
    d = 2 * radius + 1
    kernel = np.ones((d, d), dtype=np.float32)
    from numpy.lib.stride_tricks import sliding_window_view  # numpy ≥ 1.20
    try:
        # Pad the map, then compute the max in each (d×d) window
        padded = np.pad(occupancy.astype(np.float32), radius, mode="constant", constant_values=0)
        windows = sliding_window_view(padded, (d, d))
        return (windows.max(axis=(-2, -1)) > 0).astype(np.uint8)
    except Exception:
        # Fallback: simple Python loop (works on any numpy version)
        h, w = occupancy.shape
        out = occupancy.copy()
        for y, x in np.argwhere(occupancy > 0):
            y0, y1 = max(0, y - radius), min(h, y + radius + 1)
            x0, x1 = max(0, x - radius), min(w, x + radius + 1)
            out[y0:y1, x0:x1] = 1
        return out


def nearest_free_cell(
    occupancy: np.ndarray,
    cell: Tuple[int, int],
    radius: int = 10,
) -> Optional[Tuple[int, int]]:
    """Return *cell* if free, else the nearest free cell within *radius*, else None.

    This lets the planner start/end outside a cell that became occupied due to
    obstacle inflation (e.g. rover backed up but the map stamp overlaps its pose).
    """
    gx, gy = cell
    h, w = occupancy.shape
    if 0 <= gx < w and 0 <= gy < h and not occupancy[gy, gx]:
        return cell
    best: Optional[Tuple[int, int]] = None
    best_d = float("inf")
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            nx, ny = gx + dx, gy + dy
            if 0 <= nx < w and 0 <= ny < h and not occupancy[ny, nx]:
                d = math.hypot(dx, dy)
                if d < best_d:
                    best_d, best = d, (nx, ny)
    return best


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
