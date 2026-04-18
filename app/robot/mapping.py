"""Occupancy grid map using log-odds Bayesian updates.

The grid is a 2-D numpy array of log-odds values. Every lidar ray updates
free cells (negative hit) and the endpoint cell (positive hit). A Bresenham
line tracer is used to identify the free cells along each ray.

Public API
─────────
    map.update_from_scan(pose, scan)           → Bayesian update from one lidar frame
    map.world_to_grid(x, y)                    → (gx, gy) cell coordinates
    map.grid_to_world(gx, gy)                  → (x, y) world coordinates
    map.in_bounds(gx, gy)                      → bool
    map.clear()                                → reset to uniform prior
    map.occupancy_for_planner()                → binary numpy array for A*
    map.overlay_scan_on_occupancy(...)         → transient live-obstacle overlay
    map.map_payload(pose, path, goal)          → JSON-friendly dict with PNG thumbnail
    map.save(name)                             → write PNG + YAML + NPY to maps/
    map.load(name)                             → load NPY from maps/

STABILITY FIX (v6)
──────────────────
  The PNG cache is now generation-based: every call to update_from_scan()
  increments _scan_gen. The PNG encoder records the generation it was
  built from; a cached PNG is reused only if the generation hasn't changed
  since it was built.  This prevents the old TTL + _dirty race that caused
  walls and obstacles to randomly redraw/flicker.
"""

import base64
import io
import json
import logging
import math
import threading
import time
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
        self.size       = config.MAP_SIZE_CELLS
        self.origin_x   = config.MAP_ORIGIN_X_M
        self.origin_y   = config.MAP_ORIGIN_Y_M
        self.log_odds   = np.zeros((self.size, self.size), dtype=np.float32)
        self._scan_gen: int = 0          # incremented on every scan update
        self._png_gen:  int = -1         # generation when PNG was last encoded
        self._png_cache: Optional[str]   = None
        self._ui_png_gen: int = -1
        self._ui_png_scale: int = 1
        self._ui_png_cache: Optional[str] = None
        self._has_observations: bool = False
        self._lock      = threading.Lock()

    # ── Coordinate conversions ───────────────────────────────────────────────

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

    # ── Map mutation ─────────────────────────────────────────────────────────

    def clear(self) -> None:
        with self._lock:
            self.log_odds.fill(0.0)
            self._scan_gen += 1
            self._has_observations = False

    def has_observations(self) -> bool:
        with self._lock:
            return self._has_observations

    @staticmethod
    def lidar_origin_for_pose(pose: Pose) -> Tuple[float, float]:
        cos_t = math.cos(pose.theta)
        sin_t = math.sin(pose.theta)
        lx = pose.x + cos_t * config.LIDAR_OFFSET_X_M - sin_t * config.LIDAR_OFFSET_Y_M
        ly = pose.y + sin_t * config.LIDAR_OFFSET_X_M + cos_t * config.LIDAR_OFFSET_Y_M
        return lx, ly

    @staticmethod
    def scan_point_to_world(pose: Pose, angle: float, dist: float) -> Tuple[float, float]:
        lx, ly = OccupancyGridMap.lidar_origin_for_pose(pose)
        global_angle = pose.theta + angle
        wx = lx + dist * math.cos(global_angle)
        wy = ly + dist * math.sin(global_angle)
        return wx, wy

    def update_from_scan(self, pose: Pose, scan: List[Tuple[float, float]]) -> None:
        lidar_x, lidar_y = self.lidar_origin_for_pose(pose)
        rx, ry = self.world_to_grid(lidar_x, lidar_y)
        if not self.in_bounds(rx, ry):
            return

        updates_free: List[Tuple[int, int]] = []
        updates_occ:  List[Tuple[int, int]] = []

        for angle, dist in scan:
            wx, wy = self.scan_point_to_world(pose, angle, dist)
            gx, gy = self.world_to_grid(wx, wy)
            if not self.in_bounds(gx, gy):
                continue
            for cx, cy in self._bresenham(rx, ry, gx, gy)[:-1]:
                if self.in_bounds(cx, cy):
                    updates_free.append((cx, cy))
            updates_occ.append((gx, gy))

        if not updates_free and not updates_occ:
            return

        with self._lock:
            self._scan_gen += 1
            for cx, cy in updates_free:
                self.log_odds[cy, cx] = np.clip(
                    self.log_odds[cy, cx] + config.FREE_HIT,
                    config.LOG_ODDS_MIN,
                    config.LOG_ODDS_MAX,
                )
            for gx, gy in updates_occ:
                self.log_odds[gy, gx] = np.clip(
                    self.log_odds[gy, gx] + config.OCCUPIED_HIT,
                    config.LOG_ODDS_MIN,
                    config.LOG_ODDS_MAX,
                )
            self._has_observations = True

    # ── Bresenham ────────────────────────────────────────────────────────────

    def _bresenham(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        points: List[Tuple[int, int]] = []
        dx = abs(x1 - x0);  dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        for _ in range(max(1, dx + abs(dy) + 2)):
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy: err += dy; x += sx
            if e2 <= dx: err += dx; y += sy
        return points

    # ── Probability / display helpers ────────────────────────────────────────

    @staticmethod
    def _probability_from_log_odds(log_odds: np.ndarray) -> np.ndarray:
        return 1.0 - 1.0 / (1.0 + np.exp(log_odds))

    def as_probability(self) -> np.ndarray:
        with self._lock:
            snap = self.log_odds.copy()
        return self._probability_from_log_odds(snap)

    @staticmethod
    def _display_image_from_log_odds(log_odds: np.ndarray) -> np.ndarray:
        prob = OccupancyGridMap._probability_from_log_odds(log_odds)
        img  = np.full(prob.shape, 92, dtype=np.uint8)   # unknown = grey
        img[prob < config.MAP_FREE_THRESHOLD]     = 232  # free = white-ish
        img[prob > config.MAP_OCCUPIED_THRESHOLD] = 16   # occupied = dark
        return img

    def as_display_image(self) -> np.ndarray:
        with self._lock:
            snap = self.log_odds.copy()
        return self._display_image_from_log_odds(snap)

    # ── PNG generation (generation-gated cache — no flicker) ─────────────────

    def _generate_png(self) -> str:
        """Return a base-64 PNG of the current map.

        The PNG is regenerated only when _scan_gen has advanced past the
        generation recorded when the last PNG was built.  This guarantees
        that the static map layer (walls, free space) never flickers between
        renders caused by other state changes (robot pose, scan dots, etc.).
        """
        with self._lock:
            current_gen = self._scan_gen
            if self._png_cache is not None and self._png_gen == current_gen:
                return self._png_cache
            snap = self.log_odds.copy()

        # Encode outside the lock — this can take ~30 ms for a 2400×2400 grid
        img_arr = self._display_image_from_log_odds(snap)
        buf = io.BytesIO()
        Image.fromarray(img_arr).save(buf, format="PNG", optimize=False)
        png_b64 = base64.b64encode(buf.getvalue()).decode("ascii")

        with self._lock:
            # Only store if the grid hasn't been updated while we were encoding
            if self._png_gen != current_gen:
                self._png_cache = png_b64
                self._png_gen   = current_gen
            return self._png_cache or png_b64

    def _generate_ui_png(self, scale: int = 4) -> str:
        scale = max(1, int(scale))
        with self._lock:
            current_gen = self._scan_gen
            if (
                self._ui_png_cache is not None
                and self._ui_png_gen == current_gen
                and self._ui_png_scale == scale
            ):
                return self._ui_png_cache
            snap = self.log_odds.copy()

        if scale > 1:
            h, w = snap.shape
            ds_h = max(1, h // scale)
            ds_w = max(1, w // scale)
            trimmed = snap[: ds_h * scale, : ds_w * scale]
            reduced = trimmed.reshape(ds_h, scale, ds_w, scale).mean(axis=(1, 3))
        else:
            reduced = snap

        img_arr = self._display_image_from_log_odds(reduced)
        buf = io.BytesIO()
        Image.fromarray(img_arr).save(buf, format="PNG", optimize=False)
        png_b64 = base64.b64encode(buf.getvalue()).decode("ascii")

        with self._lock:
            self._ui_png_cache = png_b64
            self._ui_png_gen = current_gen
            self._ui_png_scale = scale
            return self._ui_png_cache

    def localize_scan_match(
        self,
        pose: Pose,
        scan: List[Tuple[float, float]],
        xy_window_m: float = 0.12,
        theta_window_rad: float = 0.20,
        xy_step_m: float = 0.04,
        theta_step_rad: float = 0.08,
        stride: int = 6,
    ) -> Optional[Pose]:
        if not scan:
            return None

        with self._lock:
            if not self._has_observations:
                return None
            log_odds = self.log_odds

        best_score = float("-inf")
        best_pose: Optional[Pose] = None
        sampled = scan[::max(1, int(stride))]

        x_offsets = np.arange(-xy_window_m, xy_window_m + 1e-6, xy_step_m)
        y_offsets = np.arange(-xy_window_m, xy_window_m + 1e-6, xy_step_m)
        t_offsets = np.arange(-theta_window_rad, theta_window_rad + 1e-6, theta_step_rad)

        for dtheta in t_offsets:
            theta = pose.theta + float(dtheta)
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
            ray_points = []
            for angle, dist in sampled:
                if not (config.LIDAR_MIN_RANGE_M <= dist <= config.LIDAR_MAX_RANGE_M):
                    continue
                rx = dist * math.cos(angle)
                ry = dist * math.sin(angle)
                ray_points.append((rx, ry, cos_t, sin_t))

            if not ray_points:
                continue

            for dx in x_offsets:
                base_x = pose.x + float(dx)
                for dy in y_offsets:
                    base_y = pose.y + float(dy)
                    score = 0.0
                    hits = 0
                    for rx, ry, cos_v, sin_v in ray_points:
                        wx = (
                            base_x
                            + cos_v * config.LIDAR_OFFSET_X_M
                            - sin_v * config.LIDAR_OFFSET_Y_M
                            + cos_v * rx
                            - sin_v * ry
                        )
                        wy = (
                            base_y
                            + sin_v * config.LIDAR_OFFSET_X_M
                            + cos_v * config.LIDAR_OFFSET_Y_M
                            + sin_v * rx
                            + cos_v * ry
                        )
                        gx, gy = self.world_to_grid(wx, wy)
                        if not self.in_bounds(gx, gy):
                            continue
                        val = float(log_odds[gy, gx])
                        score += val
                        if val > 0.5:
                            hits += 1
                    if hits < 3:
                        continue
                    score += hits * 2.0
                    if score > best_score:
                        best_score = score
                        best_pose = Pose(base_x, base_y, theta)

        return best_pose

    # ── Planner helpers ───────────────────────────────────────────────────────

    def occupancy_for_planner(self) -> np.ndarray:
        with self._lock:
            snap = self.log_odds.copy()
        prob = self._probability_from_log_odds(snap)
        occ  = np.zeros_like(prob, dtype=np.uint8)
        occ[prob > config.MAP_OCCUPIED_THRESHOLD] = 1
        return occ

    def overlay_scan_on_occupancy(
        self,
        pose:      Pose,
        scan:      List[Tuple[float, float]],
        occupancy: Optional[np.ndarray] = None,
        stride:    int = 2,
    ) -> np.ndarray:
        """Overlay the current live scan as transient obstacles for planning.

        Does *not* mutate the stored SLAM map. Used during mission mode so
        the saved/static map remains stable while dynamic obstacles are still
        considered by the planner.
        """
        if occupancy is None:
            occupancy = self.occupancy_for_planner()
        else:
            occupancy = occupancy.copy()

        if not scan:
            return occupancy

        step = max(1, int(stride))
        for angle, dist in scan[::step]:
            if not (config.LIDAR_MIN_RANGE_M <= dist <= config.LIDAR_MAX_RANGE_M):
                continue
            wx, wy = self.scan_point_to_world(pose, angle, dist)
            gx, gy = self.world_to_grid(wx, wy)
            if self.in_bounds(gx, gy):
                occupancy[gy, gx] = 1
        return occupancy

    # ── Persistence ──────────────────────────────────────────────────────────

    def save(self, name: str) -> dict:
        png_path  = config.MAPS_DIR / f"{name}.png"
        yaml_path = config.MAPS_DIR / f"{name}.yaml"
        json_path = config.MAPS_DIR / f"{name}.json"
        npy_path  = config.MAPS_DIR / f"{name}.npy"
        Image.fromarray(self.as_display_image()).save(png_path)
        with self._lock:
            snap = self.log_odds.copy()
        np.save(npy_path, snap)
        yaml_path.write_text(yaml.safe_dump({
            "image":          png_path.name,
            "resolution":     float(self.resolution),
            "origin":         [float(self.origin_x), float(self.origin_y), 0.0],
            "negate":         0,
            "occupied_thresh": float(config.MAP_OCCUPIED_THRESHOLD),
            "free_thresh":    float(config.MAP_FREE_THRESHOLD),
        }))
        json_path.write_text(json.dumps({
            "size":     self.size,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "resolution": self.resolution,
        }, indent=2))
        log.info("Map saved: %s", name)
        return {"png": str(png_path), "yaml": str(yaml_path), "json": str(json_path), "npy": str(npy_path)}

    def load(self, name: str) -> None:
        npy_path = config.MAPS_DIR / f"{name}.npy"
        if not npy_path.exists():
            raise FileNotFoundError(f"Map not found: {npy_path}")
        loaded = np.load(npy_path)
        if loaded.shape != (self.size, self.size):
            raise ValueError(f"Map shape {loaded.shape} mismatches grid ({self.size}×{self.size})")
        with self._lock:
            self.log_odds = loaded.astype(np.float32)
            self._scan_gen += 1  # force PNG re-encode on next request
            self._has_observations = True
        log.info("Map loaded: %s", name)

    # ── Payload (sent to browser via Socket.IO / REST) ────────────────────────

    def map_payload(
        self,
        pose:    Optional[Pose]  = None,
        path:    Optional[list]  = None,
        goal:    Optional[Tuple[float, float]] = None,
        scan_px: Optional[list]  = None,
        render_scale: int = 4,
        mission_waypoints: Optional[list] = None,
    ) -> dict:
        scale = max(1, int(render_scale))
        png_b64 = self._generate_ui_png(scale=scale)

        render_resolution = self.resolution * scale
        render_width = max(1, self.size // scale)
        render_height = max(1, self.size // scale)

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

        mission_px = []
        if mission_waypoints:
            for wx, wy in mission_waypoints:
                gx, gy = self.world_to_grid(wx, wy)
                mission_px.append({"x": gx / scale, "y": gy / scale})

        scan_scaled = []
        for pt in scan_px or []:
            scan_scaled.append({"x": pt["x"] / scale, "y": pt["y"] / scale})

        return {
            "width":         render_width,
            "height":        render_height,
            "image_png_b64": png_b64,
            "pose":          pose_px,
            "path":          path_px,
            "goal":          goal_px,
            "scan":          scan_scaled,
            "mission_waypoints": mission_px,
            "resolution":    render_resolution,
            "origin":        [self.origin_x, self.origin_y],
            "render_scale":  scale,
        }
