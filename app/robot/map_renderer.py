"""Optimized map renderer with viewport culling and asynchronous encoding.

Provides:
- Viewport-based map rendering (only visible area)
- Async PNG generation to avoid blocking
- Multiple resolution levels (pyramid) for fast zoom
- Efficient tile-based updates

This significantly improves map update performance on Raspberry Pi.
"""

import base64
import io
import logging
import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
from PIL import Image

log = logging.getLogger(__name__)


@dataclass
class Viewport:
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min

    @property
    def center_x(self) -> float:
        return (self.x_min + self.x_max) / 2

    @property
    def center_y(self) -> float:
        return (self.y_min + self.y_max) / 2

    def expand(self, margin: float = 1.2) -> 'Viewport':
        """Return expanded viewport by margin factor."""
        w = self.width * (margin - 1) / 2
        h = self.height * (margin - 1) / 2
        return Viewport(
            self.x_min - w,
            self.y_min - h,
            self.x_max + w,
            self.y_max + h,
        )


class MapRenderer:
    """High-performance map renderer with viewport culling."""

    def __init__(
        self,
        grid_size: int = 2400,
        resolution: float = 0.02,
        origin_x: float = -24.0,
        origin_y: float = -24.0,
        max_tile_size: int = 512,
    ) -> None:
        self.grid_size = grid_size
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.max_tile_size = max_tile_size

        self._log_odds: Optional[np.ndarray] = None
        self._scan_gen: int = 0
        self._png_cache: Optional[str] = None
        self._png_gen: int = -1

        self._thumbnail_cache: Dict[int, str] = {}
        self._tile_cache: Dict[Tuple[int, int], str] = {}
        self._lock = threading.Lock()

        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="map-renderer")

        self._pending_encode = False
        self._last_encode_time: float = 0.0
        self.encode_interval: float = 0.5

    def update_map(self, log_odds: np.ndarray) -> None:
        """Update the map data and invalidate caches."""
        with self._lock:
            self._log_odds = log_odds.copy()
            self._scan_gen += 1
            self._png_cache = None
            self._thumbnail_cache.clear()
            self._tile_cache.clear()

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert world coords to grid coords."""
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert grid coords to world coords."""
        wx = gx * self.resolution + self.origin_x
        wy = gy * self.resolution + self.origin_y
        return wx, wy

    def compute_viewport_grid(
        self,
        center_x: float,
        center_y: float,
        canvas_width: int,
        canvas_height: int,
        zoom: float,
    ) -> Viewport:
        """Compute viewport in grid coordinates from screen parameters."""
        pixels_per_grid = zoom / self.resolution

        view_width_px = canvas_width / pixels_per_grid
        view_height_px = canvas_height / pixels_per_grid

        return Viewport(
            center_x - view_width_px / 2,
            center_y - view_height_px / 2,
            center_x + view_width_px / 2,
            center_y + view_height_px / 2,
        )

    def render_viewport(
        self,
        viewport: Viewport,
        pose: Optional[Tuple[float, float, float]] = None,
        path: Optional[List[Tuple[float, float]]] = None,
        goal: Optional[Tuple[float, float]] = None,
        scan: Optional[List[Tuple[int, int]]] = None,
    ) -> dict:
        """Render map for given viewport."""
        with self._lock:
            if self._log_odds is None:
                return self._empty_payload()

            vg = self._viewport_to_grid(viewport)
            if vg is None:
                return self._empty_payload()

            grid = self._log_odds
            h, w = grid.shape

            x0 = max(0, vg.x_min)
            y0 = max(0, vg.y_min)
            x1 = min(w, vg.x_max + 1)
            y1 = min(h, vg.y_max + 1)

            if x1 <= x0 or y1 <= y0:
                return self._empty_payload()

            region = grid[y0:y1, x0:x1]
            prob = 1.0 - 1.0 / (1.0 + np.exp(region))

            img = np.full(region.shape, 92, dtype=np.uint8)
            img[prob < 0.32] = 232
            img[prob > 0.68] = 16

            png_b64 = self._encode_region(img, x0, y0)

        pose_px = None
        if pose is not None:
            gx, gy = self.world_to_grid(pose[0], pose[1])
            gx -= x0
            gy -= y0
            pose_px = {"x": gx, "y": gy, "theta": pose[2]}

        path_px = []
        if path:
            for wx, wy in path:
                gx, gy = self.world_to_grid(wx, wy)
                gx -= x0
                gy -= y0
                if 0 <= gx < (x1 - x0) and 0 <= gy < (y1 - y0):
                    path_px.append({"x": gx, "y": gy})

        goal_px = None
        if goal is not None:
            gx, gy = self.world_to_grid(goal[0], goal[1])
            gx -= x0
            gy -= y0
            if 0 <= gx < (x1 - x0) and 0 <= gy < (y1 - y0):
                goal_px = {"x": gx, "y": gy}

        scan_px = []
        if scan:
            for gx, gy in scan:
                gx -= x0
                gy -= y0
                if 0 <= gx < (x1 - x0) and 0 <= gy < (y1 - y0):
                    scan_px.append({"x": gx, "y": gy})

        return {
            "viewport": {
                "x": x0,
                "y": y0,
                "width": x1 - x0,
                "height": y1 - y0,
            },
            "image_png_b64": png_b64,
            "pose": pose_px,
            "path": path_px,
            "goal": goal_px,
            "scan": scan_px,
            "resolution": self.resolution,
            "origin": [self.origin_x, self.origin_y],
        }

    def _viewport_to_grid(self, viewport: Viewport) -> Optional[Viewport]:
        """Convert world viewport to grid viewport."""
        gx0 = int((viewport.x_min - self.origin_x) / self.resolution)
        gy0 = int((viewport.y_min - self.origin_y) / self.resolution)
        gx1 = int((viewport.x_max - self.origin_x) / self.resolution)
        gy1 = int((viewport.y_max - self.origin_y) / self.resolution)

        if gx1 < 0 or gy1 < 0 or gx0 >= self.grid_size or gy0 >= self.grid_size:
            return None

        return Viewport(gx0, gy0, gx1, gy1)

    def _encode_region(self, img: np.ndarray, x: int, y: int) -> str:
        """Encode image region to base64 PNG."""
        cache_key = (x // 64) * 10000 + (y // 64)

        future = self._executor.submit(self._do_encode, img, cache_key)
        return future.result(timeout=1.0)

    def _do_encode(self, img: np.ndarray, cache_key: int) -> str:
        """Perform PNG encoding in background thread."""
        buf = io.BytesIO()
        Image.fromarray(img).save(buf, format="PNG", optimize=False)
        return base64.b64encode(buf.getvalue()).decode("ascii")

    def generate_thumbnail(
        self,
        scale: int = 8,
    ) -> Optional[str]:
        """Generate downsampled thumbnail for overview display."""
        with self._lock:
            if self._log_odds is None:
                return None

            if scale in self._thumbnail_cache:
                return self._thumbnail_cache[scale]

            grid = self._log_odds
            h, w = grid.shape

            thumb_h = max(1, h // scale)
            thumb_w = max(1, w // scale)

            small = grid.reshape(thumb_h, scale, thumb_w, scale).mean(axis=(1, 3))
            prob = 1.0 - 1.0 / (1.0 + np.exp(small))

            img = np.full((thumb_h, thumb_w), 92, dtype=np.uint8)
            img[prob < 0.32] = 232
            img[prob > 0.68] = 16

        buf = io.BytesIO()
        Image.fromarray(img).save(buf, format="PNG", optimize=False)
        b64 = base64.b64encode(buf.getvalue()).decode("ascii")

        with self._lock:
            self._thumbnail_cache[scale] = b64

        return b64

    def get_full_map_payload(
        self,
        pose: Optional[Tuple[float, float, float]] = None,
        path: Optional[List[Tuple[float, float]]] = None,
        goal: Optional[Tuple[float, float]] = None,
        scan_px: Optional[List[Tuple[int, int]]] = None,
    ) -> dict:
        """Get full map payload with optional caching."""
        now = time.time()

        if self._png_cache is not None and (now - self._last_encode_time) < self.encode_interval:
            return self._build_payload(
                self._png_cache, pose, path, goal, scan_px
            )

        with self._lock:
            if self._log_odds is None:
                return self._empty_payload()

            current_gen = self._scan_gen

            if self._png_cache is not None and self._png_gen == current_gen:
                return self._build_payload(
                    self._png_cache, pose, path, goal, scan_px
                )

            future = self._executor.submit(self._encode_full_map)
            try:
                png_b64 = future.result(timeout=0.5)
            except Exception:
                png_b64 = self._png_cache or ""

        with self._lock:
            if png_b64:
                self._png_cache = png_b64
                self._png_gen = current_gen
                self._last_encode_time = now

        return self._build_payload(png_b64, pose, path, goal, scan_px)

    def _encode_full_map(self) -> str:
        """Encode full map to PNG (runs in background)."""
        with self._lock:
            if self._log_odds is None:
                return ""
            grid = self._log_odds.copy()

        prob = 1.0 - 1.0 / (1.0 + np.exp(grid))
        img = np.full(grid.shape, 92, dtype=np.uint8)
        img[prob < 0.32] = 232
        img[prob > 0.68] = 16

        buf = io.BytesIO()
        Image.fromarray(img).save(buf, format="PNG", optimize=False)
        return base64.b64encode(buf.getvalue()).decode("ascii")

    def _build_payload(
        self,
        png_b64: str,
        pose: Optional[Tuple[float, float, float]],
        path: Optional[List[Tuple[float, float]]],
        goal: Optional[Tuple[float, float]],
        scan_px: Optional[List[Tuple[int, int]]],
    ) -> dict:
        """Build map payload with pose, path, goal converted to pixels."""
        pose_px = None
        if pose is not None:
            gx, gy = self.world_to_grid(pose[0], pose[1])
            pose_px = {"x": gx, "y": gy, "theta": pose[2]}

        path_px = []
        if path:
            for wx, wy in path:
                gx, gy = self.world_to_grid(wx, wy)
                path_px.append({"x": gx, "y": gy})

        goal_px = None
        if goal is not None:
            gx, gy = self.world_to_grid(goal[0], goal[1])
            goal_px = {"x": gx, "y": gy}

        return {
            "width": self.grid_size,
            "height": self.grid_size,
            "image_png_b64": png_b64,
            "pose": pose_px,
            "path": path_px,
            "goal": goal_px,
            "scan": scan_px or [],
            "resolution": self.resolution,
            "origin": [self.origin_x, self.origin_y],
        }

    def _empty_payload(self) -> dict:
        """Return empty payload structure."""
        return {
            "width": self.grid_size,
            "height": self.grid_size,
            "image_png_b64": None,
            "pose": None,
            "path": [],
            "goal": None,
            "scan": [],
            "resolution": self.resolution,
            "origin": [self.origin_x, self.origin_y],
        }

    def shutdown(self) -> None:
        """Shutdown executor and cleanup."""
        self._executor.shutdown(wait=False)


def create_tile_pyramid(
    grid: np.ndarray,
    levels: int = 4,
    tile_size: int = 256,
) -> Dict[int, List[np.ndarray]]:
    """Create multi-resolution tile pyramid for fast map access."""
    pyramid: Dict[int, List[np.ndarray]] = {}

    prob = 1.0 - 1.0 / (1.0 + np.exp(grid))
    img = np.full(grid.shape, 92, dtype=np.uint8)
    img[prob < 0.32] = 232
    img[prob > 0.68] = 16

    h, w = img.shape
    scale = 1

    for level in range(levels):
        level_h = max(1, h // scale)
        level_w = max(1, w // scale)

        if level == 0:
            tiles = [img]
        else:
            tiles = []
            for ty in range(0, level_h, tile_size):
                row = []
                for tx in range(0, level_w, tile_size):
                    tile = img[ty:min(ty + tile_size, level_h), tx:min(tx + tile_size, level_w)]
                    row.append(tile)
                tiles.extend(row)

        pyramid[level] = tiles
        scale *= 2

    return pyramid
