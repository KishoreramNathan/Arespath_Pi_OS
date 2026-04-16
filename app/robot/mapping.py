"""Occupancy grid map using log-odds Bayesian updates.

The grid is a 2-D numpy array of log-odds values.  Every lidar ray updates
free cells (negative hit) and the endpoint cell (positive hit).  A Bresenham
line tracer is used to identify the free cells along each ray.

Public API
──────────
    map.update_from_scan(pose, scan)    → Bayesian update from one lidar frame
    map.world_to_grid(x, y)            → (gx, gy) cell coordinates
    map.grid_to_world(gx, gy)          → (x, y) world coordinates
    map.in_bounds(gx, gy)              → bool
    map.clear()                         → reset to uniform prior
    map.occupancy_for_planner()        → binary numpy array for A*
    map.map_payload(pose, path, goal)  → JSON-friendly dict with PNG thumbnail
    map.save(name)                     → write PNG + YAML + NPY to maps/
    map.load(name)                     → load NPY from maps/
"""

import base64
import io
import json
import logging
import math
from typing import List, Optional, Tuple

import numpy as np
import yaml
from PIL import Image

from app import config
from app.robot.state import Pose

log = logging.getLogger(__name__)


class OccupancyGridMap:
    def __init__(self) -> None:
        self.resolution = config.MAP_RESOLUTION_M
        self.size = config.MAP_SIZE_CELLS
        self.origin_x = config.MAP_ORIGIN_X_M
        self.origin_y = config.MAP_ORIGIN_Y_M
        self.log_odds = np.zeros((self.size, self.size), dtype=np.float32)
        self._scan_count = 0
        self._png_cache = None
        self._png_cache_time = 0.0
        self._png_cache_mtime = 0.0

    # ── Coordinate helpers ────────────────────────────────────────────────────

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        return (
            gx * self.resolution + self.origin_x,
            gy * self.resolution + self.origin_y,
        )

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.size and 0 <= gy < self.size

    # ── Map operations ────────────────────────────────────────────────────────

    def clear(self) -> None:
        self.log_odds.fill(0.0)
        self._scan_count = 0

    def update_from_scan(self, pose: Pose, scan: List[Tuple[float, float]]) -> None:
        rx, ry = self.world_to_grid(pose.x, pose.y)
        if not self.in_bounds(rx, ry):
            return
        
        self._scan_count += 1
        
        for angle, dist in scan:
            global_angle = pose.theta + angle
            wx = pose.x + dist * math.cos(global_angle)
            wy = pose.y + dist * math.sin(global_angle)
            gx, gy = self.world_to_grid(wx, wy)
            if not self.in_bounds(gx, gy):
                continue
            for cx, cy in self._bresenham(rx, ry, gx, gy)[:-1]:
                if self.in_bounds(cx, cy):
                    self.log_odds[cy, cx] = np.clip(
                        self.log_odds[cy, cx] + config.FREE_HIT,
                        config.LOG_ODDS_MIN, config.LOG_ODDS_MAX,
                    )
            self.log_odds[gy, gx] = np.clip(
                self.log_odds[gy, gx] + config.OCCUPIED_HIT,
                config.LOG_ODDS_MIN, config.LOG_ODDS_MAX,
            )

    def _bresenham(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        points: List[Tuple[int, int]] = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        max_steps = dx - dy + 4
        for _ in range(max(1, max_steps)):
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return points

    # ── Derived views ─────────────────────────────────────────────────────────

    def as_probability(self) -> np.ndarray:
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))

    def as_display_image(self) -> np.ndarray:
        """Grey (unknown) / white (free) / dark (occupied) uint8 array."""
        prob = self.as_probability()
        img = np.full(prob.shape, 92, dtype=np.uint8)
        img[prob < config.MAP_FREE_THRESHOLD] = 232
        img[prob > config.MAP_OCCUPIED_THRESHOLD] = 16
        return img

    def occupancy_for_planner(self) -> np.ndarray:
        prob = self.as_probability()
        occ = np.zeros_like(prob, dtype=np.uint8)
        occ[prob > config.MAP_OCCUPIED_THRESHOLD] = 1
        return occ

    # ── Persistence ───────────────────────────────────────────────────────────

    def save(self, name: str) -> dict:
        png_path  = config.MAPS_DIR / f"{name}.png"
        yaml_path = config.MAPS_DIR / f"{name}.yaml"
        json_path = config.MAPS_DIR / f"{name}.json"
        npy_path  = config.MAPS_DIR / f"{name}.npy"

        Image.fromarray(self.as_display_image()).save(png_path)
        np.save(npy_path, self.log_odds)
        yaml_path.write_text(yaml.safe_dump({
            "image": png_path.name,
            "resolution": float(self.resolution),
            "origin": [float(self.origin_x), float(self.origin_y), 0.0],
            "negate": 0,
            "occupied_thresh": float(config.MAP_OCCUPIED_THRESHOLD),
            "free_thresh": float(config.MAP_FREE_THRESHOLD),
        }))
        json_path.write_text(json.dumps({
            "size": self.size,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "resolution": self.resolution,
        }, indent=2))
        log.info("Map saved: %s", name)
        return {"png": str(png_path), "yaml": str(yaml_path),
                "json": str(json_path), "npy": str(npy_path)}

    def load(self, name: str) -> None:
        npy_path = config.MAPS_DIR / f"{name}.npy"
        if not npy_path.exists():
            raise FileNotFoundError(f"Map not found: {npy_path}")
        loaded = np.load(npy_path)
        if loaded.shape != (self.size, self.size):
            raise ValueError(
                f"Map shape {loaded.shape} mismatches grid ({self.size}×{self.size})"
            )
        self.log_odds = loaded.astype(np.float32)
        log.info("Map loaded: %s", name)

    # ── Payload for dashboard ─────────────────────────────────────────────────

    def map_payload(
        self,
        pose: Optional[Pose] = None,
        path: Optional[list] = None,
        goal: Optional[Tuple[float, float]] = None,
        scan_px: Optional[list] = None,
    ) -> dict:
        import time
        now = time.monotonic()
        thumb_size = 800
        if self._png_cache is None or (now - self._png_cache_time) > 0.5:
            img = self.as_display_image()
            thumb = Image.fromarray(img).resize((thumb_size, thumb_size), Image.Resampling.LANCZOS)
            buf = io.BytesIO()
            thumb.save(buf, format="PNG", optimize=False)
            self._png_cache = base64.b64encode(buf.getvalue()).decode("ascii")
            self._png_cache_time = now
            self._thumb_size = thumb_size

        scale = self.size / thumb_size  # grid to thumbnail scale

        pose_px = None
        if pose is not None:
            gx, gy = self.world_to_grid(pose.x, pose.y)
            pose_px = {"x": gx / scale, "y": gy / scale, "theta": pose.theta}

        path_px = []
        if path:
            for wx, wy in path:
                gx, gy = self.world_to_grid(wx, wy)
                path_px.append({"x": gx / scale, "y": gy / scale})

        goal_px = None
        if goal is not None:
            gx, gy = self.world_to_grid(goal[0], goal[1])
            goal_px = {"x": gx / scale, "y": gy / scale}

        return {
            "width":           thumb_size,
            "height":          thumb_size,
            "grid_size":       self.size,
            "image_png_b64":   self._png_cache,
            "pose":            pose_px,
            "path":            path_px,
            "goal":            goal_px,
            "scan":            scan_px or [],
            "resolution":      self.resolution,
            "origin":          [self.origin_x, self.origin_y],
        }
