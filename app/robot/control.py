"""RobotRuntime — Simplified Robust Navigation Stack.

Focus: Mapping, Path Making, Path Following, Stop & Replan.

Key Features:
• Sensor Fusion: Telemetry (odometry) + LiDAR ICP correction
• Simple A* path planner
• Direct waypoint following (no complex splines)
• Multi-waypoint mission support (x1,y1 → x2,y2 → x3,y3)
• Stable obstacle detection (5 sec wait before replan)

Thread Model:
  robot-ctrl (20 Hz)  — odometry, localization, nav/motor commands
  cmd-worker          — event-driven command serializer
  serial-reader       — background telemetry reader
  map-worker          — SLAM map updates
  monitor             — heartbeat + reconnect watchdog
"""

import logging
import math
import threading
import time
from typing import List, Optional, Tuple

from app import config
from app.robot.command_bus import DriveCommandQueue
from app.robot.lidar import LidarLocalizer, LidarManager
from app.robot.mapping import OccupancyGridMap
from app.robot.planner import astar, inflate_occupancy, nearest_free_cell, simplify_path
from app.robot.poi import PoiManager
from app.robot.runtime_cfg import rtcfg
from app.robot.serial_bridge import ArduinoBridge
from app.robot.state import Pose, RobotState, wrap_angle

log = logging.getLogger(__name__)

_CONTROL_HZ = 20.0
_CONTROL_DT = 1.0 / _CONTROL_HZ


class RobotRuntime:
    def __init__(self) -> None:
        self.state = RobotState()
        self.map = OccupancyGridMap()
        self.lidar = LidarManager()
        self.localizer = LidarLocalizer()
        self.bridge = ArduinoBridge(on_telemetry=self._on_telemetry)
        self.queue = DriveCommandQueue(self.bridge)
        self.poi = PoiManager(config.POIS_FILE)
        self.lock = threading.Lock()

        self._running = False
        self._ctrl_thread: Optional[threading.Thread] = None
        self._monitor_thread: Optional[threading.Thread] = None
        self._socketio = None

        self._last_ticks: Optional[Tuple[int, int]] = None
        self._last_tel_ts: Optional[float] = None

        self._last_heartbeat: float = time.time()

        self._obstacle_detected_at: float = 0.0
        self._is_obstacle_stable: bool = False

        self._mission_waypoints: List[Tuple[float, float]] = []
        self._current_waypoint_idx: int = 0

    def set_socketio(self, sio) -> None:
        self._socketio = sio

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self.bridge.connect()
        with self.lock:
            self.state.connected = self.bridge.ser is not None
        self.lidar.start()
        self.queue.start()

        self._ctrl_thread = threading.Thread(
            target=self._ctrl_loop, daemon=True, name="robot-ctrl"
        )
        self._ctrl_thread.start()

        self._monitor_thread = threading.Thread(
            target=self._monitor_loop, daemon=True, name="robot-monitor"
        )
        self._monitor_thread.start()

        log.info("RobotRuntime started (simplified navigation)")

    def stop(self) -> None:
        self._running = False
        self.queue.stop()
        self.bridge.close()
        self.lidar.stop()
        log.info("RobotRuntime stopped")

    def _on_telemetry(self, data: dict) -> None:
        with self.lock:
            self.state.connected = True
            self.state.last_update = time.time()
            self.state.telemetry_age_s = 0.0
            self.state.left_ticks = int(data.get("left_ticks", self.state.left_ticks))
            self.state.right_ticks = int(data.get("right_ticks", self.state.right_ticks))
            self.state.left_rpm = float(data.get("left_rpm", self.state.left_rpm))
            self.state.right_rpm = float(data.get("right_rpm", self.state.right_rpm))
            self.state.left_pwm = int(data.get("left_pwm", self.state.left_pwm))
            self.state.right_pwm = int(data.get("right_pwm", self.state.right_pwm))
            bv = data.get("battery_v")
            self.state.battery_v = None if bv in (None, "") else float(bv)
            self.state.last_error = self.bridge.last_error

    def arm(self, armed: bool) -> None:
        with self.lock:
            self.state.armed = bool(armed)
            if not self.state.armed:
                self.state.linear_cmd = 0.0
                self.state.angular_cmd = 0.0
        if not armed:
            self.queue.push_stop()

    def set_manual_command(self, linear: float, angular: float) -> None:
        with self.lock:
            if not self.state.armed:
                return
            self.state.linear_cmd = max(-1.0, min(1.0, linear))
            self.state.angular_cmd = max(-1.0, min(1.0, angular))
            self.state.mode = "pilot"
        left_pwm, right_pwm = self._cmd_to_pwm(linear, angular)
        self.queue.push_manual(left_pwm, right_pwm)

    def force_stop(self) -> None:
        with self.lock:
            self.state.linear_cmd = 0.0
            self.state.angular_cmd = 0.0
        self.queue.push_stop()

    def heartbeat(self) -> None:
        self._last_heartbeat = time.time()

    def set_mode(self, mode: str) -> None:
        aliases = {"drive": "pilot", "navigate": "mission",
                   "pilot": "pilot", "mission": "mission"}
        mode = aliases.get(mode, mode)
        with self.lock:
            if mode not in ("pilot", "mission"):
                raise ValueError(f"invalid mode: {mode!r}")
            self.state.mode = mode
            if mode != "mission":
                self._cancel_nav_locked()

    def set_goal(self, x: float, y: float) -> None:
        """Set single goal or start of multi-waypoint mission."""
        with self.lock:
            self.state.mode = "mission"
            nav = self.state.nav
            nav.active = True
            nav.goal = (float(x), float(y))
            nav.status = "planning"
            nav.path = []
            nav.current_target = None
            self._obstacle_detected_at = 0.0
            self._is_obstacle_stable = False
            self._mission_waypoints = [(float(x), float(y))]
            self._current_waypoint_idx = 0

    def add_waypoint(self, x: float, y: float) -> None:
        """Add waypoint to current mission queue."""
        with self.lock:
            self._mission_waypoints.append((float(x), float(y)))

    def set_mission_waypoints(self, waypoints: List[Tuple[float, float]]) -> None:
        """Set entire mission waypoints: [(x1,y1), (x2,y2), (x3,y3), ...]"""
        if not waypoints:
            return
        with self.lock:
            self.state.mode = "mission"
            nav = self.state.nav
            nav.active = True
            nav.goal = waypoints[0]
            nav.status = "planning"
            nav.path = []
            nav.current_target = None
            self._mission_waypoints = list(waypoints)
            self._current_waypoint_idx = 0
            self._obstacle_detected_at = 0.0
            self._is_obstacle_stable = False
        log.info("Mission set with %d waypoints", len(waypoints))

    def cancel_navigation(self) -> None:
        with self.lock:
            self._cancel_nav_locked()

    def _cancel_nav_locked(self) -> None:
        nav = self.state.nav
        nav.active = False
        nav.goal = None
        nav.path = []
        nav.current_target = None
        nav.status = "idle"
        self.state.linear_cmd = 0.0
        self.state.angular_cmd = 0.0
        self._mission_waypoints = []
        self._current_waypoint_idx = 0
        self.queue.push_stop()

    def start_mapping(self, clear: bool = False) -> None:
        with self.lock:
            if clear:
                self.map.clear()
            self.state.mapping = True

    def stop_mapping(self) -> None:
        with self.lock:
            self.state.mapping = False

    def clear_map(self) -> None:
        with self.lock:
            self.map.clear()
            self.state.nav.path = []
        self.localizer.reset()

    def save_map(self, name: str) -> dict:
        return self.map.save(name)

    def load_map(self, name: str) -> None:
        self.map.load(name)
        self.localizer.reset()

    def set_pose(self, x: float, y: float, theta: float) -> None:
        with self.lock:
            self.state.pose = Pose(x, y, theta)
            self.bridge.reset_odometry()
            self._last_ticks = None
            self._last_tel_ts = None
            self.state.nav.status = "pose set"
        self.localizer.reset()

    def poi_add(self, label: str, kind: str, x: float, y: float, note: str = "") -> dict:
        return self.poi.add(label, kind, x, y, note)

    def poi_update(self, poi_id: str, **fields) -> Optional[dict]:
        return self.poi.update(poi_id, **fields)

    def poi_remove(self, poi_id: str) -> bool:
        return self.poi.remove(poi_id)

    def poi_list(self) -> list:
        return self.poi.list()

    def poi_navigate(self, poi_id: str) -> bool:
        coords = self.poi.navigate_to(poi_id)
        if coords is None:
            return False
        self.set_goal(coords[0], coords[1])
        return True

    def get_status(self) -> dict:
        with self.lock:
            self.state.lidar_connected = self.lidar.connected
            self.state.latest_scan_points = len(self.lidar.get_scan())
            self.state.last_error = self.bridge.last_error or self.state.last_error
            s = self.state.to_dict()
            s["mission_waypoints"] = len(self._mission_waypoints)
            s["current_waypoint"] = self._current_waypoint_idx + 1
            s["waypoint_total"] = len(self._mission_waypoints)
        if self._obstacle_detected_at > 0:
            elapsed = time.time() - self._obstacle_detected_at
            wait_s = rtcfg.get("obstacle_wait_before_replan_s")
            remaining = max(0.0, wait_s - elapsed)
            s["obstacle_wait_remaining_s"] = round(remaining, 1)
        else:
            s["obstacle_wait_remaining_s"] = None
        return s

    def get_lidar_payload(self) -> dict:
        with self.lock:
            points = self.lidar.scan_cartesian(config.LIDAR_RENDER_MAX_POINTS)
            return {
                "connected": self.lidar.connected,
                "error": self.lidar.error,
                "points": points,
                "obstacle_stop": self.state.obstacle_stop,
                "max_range": config.LIDAR_MAX_RANGE_M,
            }

    def get_map_payload(self) -> dict:
        with self.lock:
            scan_px = self._scan_to_map_pixels(self.lidar.get_scan())
            payload = self.map.map_payload(
                self.state.pose,
                self.state.nav.path,
                self.state.nav.goal,
                scan_px=scan_px,
            )
        pois_px = []
        for poi in self.poi.list():
            gx, gy = self.map.world_to_grid(poi["x"], poi["y"])
            pois_px.append({**poi, "px": gx, "py": gy})
        payload["pois"] = pois_px
        return payload

    def _scan_to_map_pixels(self, scan: List[Tuple[float, float]]) -> list:
        out = []
        if not scan:
            return out
        step = max(1, len(scan) // 360)
        pose = self.state.pose
        for angle, dist in scan[::step]:
            if dist <= 0:
                continue
            global_angle = pose.theta + angle
            wx = pose.x + dist * math.cos(global_angle)
            wy = pose.y + dist * math.sin(global_angle)
            gx, gy = self.map.world_to_grid(wx, wy)
            if self.map.in_bounds(gx, gy):
                out.append({"x": gx, "y": gy})
        return out

    def _ctrl_loop(self) -> None:
        """Main 20 Hz control loop - simple and robust."""
        import queue as _queue
        _map_q: "_queue.Queue[tuple]" = _queue.Queue(maxsize=2)

        def _map_worker() -> None:
            while self._running:
                try:
                    pose_snap, scan_snap = _map_q.get(timeout=0.2)
                    self.map.update_from_scan(pose_snap, scan_snap)
                except _queue.Empty:
                    pass
                except Exception as exc:
                    log.warning("Map worker error: %s", exc)

        _map_thread = threading.Thread(target=_map_worker, daemon=True, name="map-worker")
        _map_thread.start()

        next_tick = time.perf_counter()
        while self._running:
            try:
                with self.lock:
                    scan = self.lidar.get_scan()
                    self.state.lidar_connected = self.lidar.connected
                    self.state.latest_scan_points = len(scan)

                    self._update_odometry_with_icp(scan)

                    if self.state.mapping and scan:
                        from app.robot.state import Pose as _Pose
                        p = self.state.pose
                        try:
                            _map_q.put_nowait((p, scan))
                        except _queue.Full:
                            pass

                    self.state.obstacle_stop = self._check_obstacle(scan)
                    mode = self.state.mode
                    nav_active = self.state.nav.active

                if mode == "mission" and nav_active:
                    self._nav_step(scan)
                else:
                    self._manual_step()

            except Exception as exc:
                log.exception("Control loop error: %s", exc)

            next_tick += _CONTROL_DT
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _update_odometry_with_icp(self, scan: List[Tuple[float, float]]) -> None:
        """Update pose using odometry + LiDAR ICP fusion."""
        self._update_odometry()

        if config.LIDAR_LOCALIZATION_ENABLED and scan and len(scan) > 20:
            corrected = self.localizer.correct(self.state.pose, scan)
            icp_weight = rtcfg.get("lidar_icp_weight")
            self.state.pose.x = self.state.pose.x * (1 - icp_weight) + corrected.x * icp_weight
            self.state.pose.y = self.state.pose.y * (1 - icp_weight) + corrected.y * icp_weight
            self.state.pose.theta = wrap_angle(
                self.state.pose.theta * (1 - icp_weight) + corrected.theta * icp_weight
            )

    def _update_odometry(self) -> None:
        """Pure odometry from wheel encoders."""
        ticks = (self.state.left_ticks, self.state.right_ticks)
        now = self.state.last_update
        if self._last_ticks is None:
            self._last_ticks = ticks
            self._last_tel_ts = now
            return
        dl = ticks[0] - self._last_ticks[0]
        dr = ticks[1] - self._last_ticks[1]
        self._last_ticks = ticks
        dt = now - (self._last_tel_ts or now)
        self._last_tel_ts = now
        if dt <= 0:
            return
        circ = 2.0 * math.pi * config.WHEEL_RADIUS_M
        left_dist = circ * (dl / config.TICKS_PER_WHEEL_REV)
        right_dist = circ * (dr / config.TICKS_PER_WHEEL_REV)
        ds = 0.5 * (left_dist + right_dist)
        dtheta = (right_dist - left_dist) / config.WHEEL_BASE_M
        mid = self.state.pose.theta + 0.5 * dtheta
        self.state.pose.x += ds * math.cos(mid)
        self.state.pose.y += ds * math.sin(mid)
        self.state.pose.theta = wrap_angle(self.state.pose.theta + dtheta)

    def _check_obstacle(self, scan: List[Tuple[float, float]]) -> bool:
        """Check if obstacle in front."""
        cone = math.radians(30)
        stop_dist = rtcfg.get("obstacle_stop_distance_m")
        for angle, dist in scan:
            if abs(angle) < cone and 0 < dist < stop_dist:
                return True
        return False

    def _manual_step(self) -> None:
        if self.queue.manual_override_active():
            return
        if self.state.linear_cmd != 0.0 or self.state.angular_cmd != 0.0:
            self.state.linear_cmd = 0.0
            self.state.angular_cmd = 0.0
            self.queue.push_stop()

    def _nav_step(self, scan: List[Tuple[float, float]]) -> None:
        """Simple waypoint following navigation."""
        if not self.state.armed:
            self.queue.push_stop()
            self.state.nav.status = "waiting for arm"
            return

        goal = self.state.nav.goal
        if goal is None:
            self._cancel_nav_locked()
            return

        dist_to_goal = math.hypot(
            goal[0] - self.state.pose.x, goal[1] - self.state.pose.y
        )

        if dist_to_goal <= config.GOAL_TOLERANCE_M:
            self.queue.push_stop()
            self._advance_to_next_waypoint()
            return

        now = time.time()
        obstacle_wait = rtcfg.get("obstacle_wait_before_replan_s")

        if self.state.obstacle_stop:
            if self._obstacle_detected_at == 0.0:
                self._obstacle_detected_at = now
                log.info("Obstacle detected - waiting %.0f s", obstacle_wait)
                self.queue.push_stop()

            elapsed = now - self._obstacle_detected_at
            remaining = obstacle_wait - elapsed

            if elapsed < obstacle_wait:
                self.state.nav.status = f"obstacle: waiting {remaining:.0f}s"
                return

            log.info("Obstacle stable for %.0f s - replanning", obstacle_wait)
            self._obstacle_detected_at = 0.0
            self.state.nav.last_plan_time = 0.0
            self.state.nav.path = []

        if not self.state.nav.path or (now - self.state.nav.last_plan_time) > rtcfg.get("nav_replan_seconds"):
            self._plan_path(scan)

        if not self.state.nav.path:
            self.state.nav.status = "no path found"
            self.queue.push_stop()
            return

        self._follow_path()

    def _advance_to_next_waypoint(self) -> None:
        """Move to next waypoint in mission or complete mission."""
        if self._current_waypoint_idx < len(self._mission_waypoints) - 1:
            self._current_waypoint_idx += 1
            next_wp = self._mission_waypoints[self._current_waypoint_idx]
            self.state.nav.goal = next_wp
            self.state.nav.path = []
            self.state.nav.last_plan_time = 0.0
            self.state.nav.status = f"waypoint {self._current_waypoint_idx + 1}/{len(self._mission_waypoints)}"
            log.info("Advancing to waypoint %d/%d: (%.2f, %.2f)",
                     self._current_waypoint_idx + 1, len(self._mission_waypoints),
                     next_wp[0], next_wp[1])
        else:
            self.state.nav.status = "mission complete"
            self.state.nav.active = False
            self.state.nav.goal = None
            log.info("Mission complete!")

    def _plan_path(self, scan: Optional[List[Tuple[float, float]]] = None) -> None:
        """Simple A* path planning."""
        goal = self.state.nav.goal
        if goal is None:
            return

        occ = self.map.occupancy_for_planner()
        if scan:
            occ = self.map.overlay_scan_on_occupancy(self.state.pose, scan, occupancy=occ)

        occ_inf = inflate_occupancy(occ, config.PLANNER_INFLATION_CELLS)

        start_raw = self.map.world_to_grid(self.state.pose.x, self.state.pose.y)
        goal_raw = self.map.world_to_grid(goal[0], goal[1])

        start = nearest_free_cell(occ_inf, start_raw, radius=8)
        goal_cell = nearest_free_cell(occ_inf, goal_raw, radius=8)

        if start is None or goal_cell is None:
            log.warning("No free cell found near start or goal")
            return

        cells = astar(occ_inf, start, goal_cell)
        if not cells:
            log.warning("No path found")
            return

        world_path = [self.map.grid_to_world(x, y) for x, y in cells]
        self.state.nav.path = simplify_path(world_path, spacing=0.15)
        self.state.nav.last_plan_time = time.time()
        log.info("Path planned with %d points", len(self.state.nav.path))

    def _follow_path(self) -> None:
        """Simple direct path following."""
        path = list(self.state.nav.path)
        if not path:
            return

        pose = self.state.pose

        while len(path) > 1:
            dist = math.hypot(path[0][0] - pose.x, path[0][1] - pose.y)
            if dist < config.WAYPOINT_REACHED_M:
                path.pop(0)
            else:
                break

        if not path:
            return

        target = path[0]
        self.state.nav.current_target = target

        dx = target[0] - pose.x
        dy = target[1] - pose.y
        target_angle = math.atan2(dy, dx)

        angle_diff = wrap_angle(target_angle - pose.theta)

        linear = config.MAX_LINEAR_MPS * 0.7
        angular = config.NAV_ANGULAR_GAIN * angle_diff

        angular = max(-config.MAX_ANGULAR_RADPS, min(config.MAX_ANGULAR_RADPS, angular))

        if abs(angle_diff) > config.ROTATE_IN_PLACE_THRESHOLD_RAD:
            linear *= 0.3

        self.state.linear_cmd = linear / config.MAX_LINEAR_MPS
        self.state.angular_cmd = angular / config.MAX_ANGULAR_RADPS

        lp, rp = self._cmd_to_pwm(linear / config.MAX_LINEAR_MPS, angular / config.MAX_ANGULAR_RADPS)
        self.queue.push_nav(lp, rp)

        self.state.nav.status = "following path"

    def _cmd_to_pwm(self, linear: float, angular: float) -> Tuple[int, int]:
        """Convert normalized commands to PWM values."""
        lmps = linear * config.MAX_LINEAR_MPS
        rads = angular * config.MAX_ANGULAR_RADPS
        lw = lmps - 0.5 * config.WHEEL_BASE_M * rads
        rw = lmps + 0.5 * config.WHEEL_BASE_M * rads
        max_speed = config.MAX_LINEAR_MPS + 0.5 * config.WHEEL_BASE_M * config.MAX_ANGULAR_RADPS
        if max_speed == 0:
            return 0, 0
        scale = config.MAX_PWM / max_speed
        left_pwm_raw = int(lw * scale)
        right_pwm_raw = int(rw * scale)
        left_pwm = max(-config.MAX_PWM, min(config.MAX_PWM, left_pwm_raw))
        right_pwm = max(-config.MAX_PWM, min(config.MAX_PWM, right_pwm_raw))
        return left_pwm, right_pwm

    def _monitor_loop(self) -> None:
        """Monitor loop for connectivity and heartbeat."""
        while self._running:
            with self.lock:
                age = time.time() - self.state.last_update
                self.state.telemetry_age_s = age
                self.state.connected = (
                    self.bridge.ser is not None
                    and age <= config.TELEMETRY_TIMEOUT_S
                )
                self.state.last_error = self.bridge.last_error or self.state.last_error

            if self.bridge.ser is None:
                self.bridge.connect()

            hb_age = time.time() - self._last_heartbeat
            if hb_age > config.HEARTBEAT_TIMEOUT_S:
                if self.state.armed:
                    log.warning("Heartbeat lost - sending stop")
                    self.queue.push_stop()
            time.sleep(0.25)
