"""RobotRuntime — integrates all robot subsystems.

Thread model
────────────
  robot-ctrl  (20 Hz)   — odometry update, map update, obstacle detection,
                           nav step or manual step → pushes to DriveCommandQueue
  cmd-worker  (50 Hz)   — DriveCommandQueue flushes commands to ArduinoBridge
  serial-reader          — ArduinoBridge background reader (calls _on_telemetry)
  monitor                — reconnect watchdog + heartbeat watchdog + telemetry age

All public methods are safe to call from any Flask handler thread.
"""

import logging
import math
import threading
import time
from typing import List, Optional, Tuple

from app import config
from app.robot.command_bus import DriveCommandQueue
from app.robot.lidar import LidarManager
from app.robot.mapping import OccupancyGridMap
from app.robot.planner import astar, inflate_occupancy, simplify_path
from app.robot.serial_bridge import ArduinoBridge
from app.robot.state import NavigationState, Pose, RobotState, wrap_angle

log = logging.getLogger(__name__)


class RobotRuntime:
    def __init__(self) -> None:
        self.state = RobotState()
        self.map = OccupancyGridMap()
        self.lidar = LidarManager()
        self.bridge = ArduinoBridge(on_telemetry=self._on_telemetry)
        self.queue = DriveCommandQueue(self.bridge)
        self.lock = threading.Lock()

        self._running = False
        self._ctrl_thread: Optional[threading.Thread] = None
        self._monitor_thread: Optional[threading.Thread] = None

        # Odometry helpers
        self._last_ticks: Optional[Tuple[int, int]] = None
        self._last_tel_ts: Optional[float] = None

        # Manual command override timer (managed here for display, enforced in queue)
        self._manual_expires_at: float = 0.0

        # Frontend heartbeat watchdog
        self._last_heartbeat: float = time.time()

    # ── Lifecycle ─────────────────────────────────────────────────────────────

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
        log.info("RobotRuntime started")

    def stop(self) -> None:
        self._running = False
        self.queue.stop()
        self.bridge.close()
        self.lidar.stop()
        log.info("RobotRuntime stopped")

    # ── Telemetry callback (called from serial-reader thread) ─────────────────

    def _on_telemetry(self, data: dict) -> None:
        with self.lock:
            self.state.connected = True
            self.state.last_update = time.time()
            self.state.telemetry_age_s = 0.0
            self.state.left_ticks  = int(data.get("left_ticks",  self.state.left_ticks))
            self.state.right_ticks = int(data.get("right_ticks", self.state.right_ticks))
            self.state.left_rpm    = float(data.get("left_rpm",  self.state.left_rpm))
            self.state.right_rpm   = float(data.get("right_rpm", self.state.right_rpm))
            self.state.left_pwm    = int(data.get("left_pwm",   self.state.left_pwm))
            self.state.right_pwm   = int(data.get("right_pwm",  self.state.right_pwm))
            bv = data.get("battery_v")
            self.state.battery_v = None if bv in (None, "") else float(bv)
            self.state.last_error = self.bridge.last_error

    # ── Public command API ────────────────────────────────────────────────────

    def arm(self, armed: bool) -> None:
        with self.lock:
            self.state.armed = bool(armed)
            if not self.state.armed:
                self.state.linear_cmd = 0.0
                self.state.angular_cmd = 0.0
        if not armed:
            self.queue.push_stop()

    def set_manual_command(self, linear: float, angular: float) -> None:
        """Called by REST /api/manual.  Push as high-priority manual command."""
        with self.lock:
            if not self.state.armed:
                return
            self.state.linear_cmd = max(-1.0, min(1.0, linear))
            self.state.angular_cmd = max(-1.0, min(1.0, angular))
            self.state.mode = "pilot"
            self._manual_expires_at = time.time() + config.MANUAL_COMMAND_TIMEOUT_S
        left_pwm, right_pwm = self._cmd_to_pwm(linear, angular)
        self.queue.push_manual(left_pwm, right_pwm)

    def force_stop(self) -> None:
        """Immediately stop motors, clear commands, stay armed."""
        with self.lock:
            self.state.linear_cmd = 0.0
            self.state.angular_cmd = 0.0
        self.queue.push_stop()

    def heartbeat(self) -> None:
        """Called by Socket.IO heartbeat event to reset watchdog."""
        self._last_heartbeat = time.time()

    # ── Mode / navigation API ─────────────────────────────────────────────────

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
        with self.lock:
            self.state.mode = "mission"
            nav = self.state.nav
            nav.active = True
            nav.goal = (float(x), float(y))
            nav.status = "planning"
            nav.path = []
            nav.current_target = None
            nav.last_plan_time = 0.0

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
        self.queue.push_stop()

    # ── Mapping API ───────────────────────────────────────────────────────────

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

    def save_map(self, name: str) -> dict:
        return self.map.save(name)

    def load_map(self, name: str) -> None:
        self.map.load(name)

    def set_pose(self, x: float, y: float, theta: float) -> None:
        with self.lock:
            self.state.pose = Pose(x, y, theta)
            self.bridge.reset_odometry()
            self._last_ticks = None
            self._last_tel_ts = None
            self.state.nav.status = "start pose updated"

    # ── Status / data API ─────────────────────────────────────────────────────

    def get_status(self) -> dict:
        with self.lock:
            self.state.lidar_connected = self.lidar.connected
            self.state.latest_scan_points = len(self.lidar.get_scan())
            self.state.last_error = self.bridge.last_error or self.state.last_error
            return self.state.to_dict()

    def get_lidar_payload(self) -> dict:
        with self.lock:
            points = self.lidar.scan_cartesian(config.LIDAR_RENDER_MAX_POINTS)
            avoidance_route = self._compute_avoidance_route(self.lidar.get_scan())
            return {
                "connected":       self.lidar.connected,
                "error":           self.lidar.error,
                "points":          points,
                "obstacle_stop":   self.state.obstacle_stop,
                "max_range":       config.LIDAR_MAX_RANGE_M,
                "avoidance_route": avoidance_route,
            }

    def _compute_avoidance_route(self, scan) -> list:
        """Compute a simple obstacle-free detour route in robot frame.

        Robot frame convention used here (matches YDLIDAR X2 output):
          angle=0   → directly forward (x-axis)
          angle>0   → left of robot  (+y)
          angle<0   → right of robot (-y)
          x = dist * cos(angle)  (forward component)
          y = dist * sin(angle)  (lateral component, +ve = left)

        Returns list of {x, y} waypoints or [] when path is clear.
        """
        if not scan or not self.state.obstacle_stop:
            return []

        # ── Find nearest obstacle in the forward cone (±40°) ─────────────────
        cone = math.radians(40)
        nearest_dist  = config.LIDAR_MAX_RANGE_M
        nearest_angle = 0.0
        for angle, dist in scan:
            if dist <= 0:
                continue
            norm_angle = math.atan2(math.sin(angle), math.cos(angle))  # wrap to ±π
            if abs(norm_angle) < cone and dist < config.OBSTACLE_STOP_DISTANCE_M * 3.0:
                if dist < nearest_dist:
                    nearest_dist  = dist
                    nearest_angle = norm_angle

        if nearest_dist >= config.LIDAR_MAX_RANGE_M:
            # Nothing found in cone despite obstacle_stop flag — rare, return empty
            return []

        # Obstacle position in robot frame
        obs_x = nearest_dist * math.cos(nearest_angle)
        obs_y = nearest_dist * math.sin(nearest_angle)

        # ── Choose detour side: whichever has more lateral clearance ─────────
        left_pts  = [d for a, d in scan if  0.25 < math.atan2(math.sin(a), math.cos(a)) < 1.57]
        right_pts = [d for a, d in scan if -1.57 < math.atan2(math.sin(a), math.cos(a)) < -0.25]
        left_clear  = min(left_pts,  default=config.LIDAR_MAX_RANGE_M)
        right_clear = min(right_pts, default=config.LIDAR_MAX_RANGE_M)

        # +1 = go left, -1 = go right
        side = 1.0 if left_clear >= right_clear else -1.0
        side_label = "left" if side > 0 else "right"

        # ── Build detour waypoints ────────────────────────────────────────────
        # Width of detour: half obstacle size estimate + safety margin
        detour_y = side * max(0.40, nearest_dist * 0.60)
        clear_dist = left_clear if side > 0 else right_clear

        wp1 = {"x": round(max(0.05, obs_x * 0.20), 3),  "y": round(detour_y, 3)}
        wp2 = {"x": round(obs_x + 0.20, 3),              "y": round(detour_y, 3)}
        wp3 = {"x": round(obs_x + 0.45, 3),              "y": 0.0}

        return [
            {"side": side_label, "clear_m": round(clear_dist, 2), **wp1},
            wp2,
            wp3,
        ]

    def get_map_payload(self) -> dict:
        with self.lock:
            scan_px = self._scan_to_map_pixels(self.lidar.get_scan())
            return self.map.map_payload(
                self.state.pose,
                self.state.nav.path,
                self.state.nav.goal,
                scan_px=scan_px,
            )

    def _scan_to_map_pixels(self, scan: List[Tuple[float, float]]) -> list:
        out = []
        if not scan:
            return out
        step = max(1, len(scan) // 180)
        pose = self.state.pose
        for angle, dist in scan[::step]:
            wx = pose.x + dist * math.cos(pose.theta + angle)
            wy = pose.y + dist * math.sin(pose.theta + angle)
            gx, gy = self.map.world_to_grid(wx, wy)
            if self.map.in_bounds(gx, gy):
                out.append({"x": gx, "y": gy})
        return out

    # ── 20 Hz control loop ────────────────────────────────────────────────────

    def _ctrl_loop(self) -> None:
        dt = 1.0 / config.CONTROL_HZ
        while self._running:
            t0 = time.time()
            try:
                with self.lock:
                    scan = self.lidar.get_scan()
                    self.state.lidar_connected = self.lidar.connected
                    self.state.latest_scan_points = len(scan)
                    self._update_odometry()
                    if self.state.mapping and scan:
                        self.map.update_from_scan(self.state.pose, scan)
                    self.state.obstacle_stop = self._obstacle_in_cone(scan)
                    if self.state.mode == "mission" and self.state.nav.active:
                        self._nav_step(scan)
                    else:
                        self._manual_step()
            except Exception as exc:
                log.exception("Control loop error: %s", exc)
            elapsed = time.time() - t0
            time.sleep(max(0.0, dt - elapsed))

    def _update_odometry(self) -> None:
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
        left_dist  = circ * (dl / config.TICKS_PER_WHEEL_REV)
        right_dist = circ * (dr / config.TICKS_PER_WHEEL_REV)
        ds = 0.5 * (left_dist + right_dist)
        dtheta = (right_dist - left_dist) / config.WHEEL_BASE_M
        mid = self.state.pose.theta + 0.5 * dtheta
        self.state.pose.x += ds * math.cos(mid)
        self.state.pose.y += ds * math.sin(mid)
        self.state.pose.theta = wrap_angle(self.state.pose.theta + dtheta)

    def _obstacle_in_cone(self, scan: List[Tuple[float, float]]) -> bool:
        cone = math.radians(25)
        for angle, dist in scan:
            if abs(angle) < cone and dist < config.OBSTACLE_STOP_DISTANCE_M:
                return True
        return False

    def _manual_step(self) -> None:
        if not self.state.armed:
            return
        # Manual commands are pushed by set_manual_command(); the queue's
        # idle watchdog auto-sends STOP if nothing arrives within WATCHDOG_IDLE_S.

    def _nav_step(self, scan: List[Tuple[float, float]]) -> None:
        if not self.state.armed:
            self.queue.push_stop()
            self.state.nav.status = "waiting for arm"
            return
        goal = self.state.nav.goal
        if goal is None:
            self._cancel_nav_locked()
            return
        dist_to_goal = math.hypot(goal[0] - self.state.pose.x, goal[1] - self.state.pose.y)
        if dist_to_goal <= config.GOAL_TOLERANCE_M:
            self.queue.push_stop()
            self.state.nav.status = "goal reached"
            self.state.nav.active = False
            self.state.linear_cmd = 0.0
            self.state.angular_cmd = 0.0
            return
        should_plan = (
            not self.state.nav.path
            or self.state.nav.status == "planning"
            or (time.time() - self.state.nav.last_plan_time) > config.NAV_REPLAN_SECONDS
        )
        if should_plan:
            self._plan_path()
            if not self.state.nav.path:
                self.state.nav.status = "no path"
                self.queue.push_stop()
                return
        target = self._lookahead_point()
        self.state.nav.current_target = target
        dx = target[0] - self.state.pose.x
        dy = target[1] - self.state.pose.y
        heading_err = wrap_angle(math.atan2(dy, dx) - self.state.pose.theta)
        linear  = min(1.0, config.NAV_LINEAR_GAIN * math.hypot(dx, dy))
        angular = max(-1.0, min(1.0, config.NAV_ANGULAR_GAIN * heading_err))
        if abs(heading_err) > config.ROTATE_IN_PLACE_THRESHOLD_RAD:
            linear *= 0.20
        if self.state.obstacle_stop and linear > 0:
            self.queue.push_stop()
            self.state.nav.status = "blocked by obstacle"
            return
        self.state.linear_cmd  = linear
        self.state.angular_cmd = angular
        self.state.nav.status  = "following path"
        left_pwm, right_pwm = self._cmd_to_pwm(linear, angular)
        self.queue.push_nav(left_pwm, right_pwm)

    def _plan_path(self) -> None:
        occ = self.map.occupancy_for_planner()
        occ = inflate_occupancy(occ, config.PLANNER_INFLATION_CELLS)
        start = self.map.world_to_grid(self.state.pose.x, self.state.pose.y)
        goal  = self.map.world_to_grid(self.state.nav.goal[0], self.state.nav.goal[1])
        cells = astar(occ, start, goal)
        self.state.nav.last_plan_time = time.time()
        if not cells:
            self.state.nav.path = []
            return
        world = [self.map.grid_to_world(x, y) for x, y in cells]
        self.state.nav.path = simplify_path(world)

    def _lookahead_point(self) -> Tuple[float, float]:
        pose = self.state.pose
        path = self.state.nav.path
        while path and math.hypot(
            path[0][0] - pose.x, path[0][1] - pose.y
        ) < config.WAYPOINT_REACHED_M:
            path.pop(0)
        if not path:
            return self.state.nav.goal
        for pt in path:
            if math.hypot(pt[0] - pose.x, pt[1] - pose.y) >= config.PATH_LOOKAHEAD_M:
                return pt
        return path[-1]

    def _cmd_to_pwm(self, linear: float, angular: float) -> Tuple[int, int]:
        lmps = linear  * config.MAX_LINEAR_MPS
        rads = angular * config.MAX_ANGULAR_RADPS
        lw = lmps - 0.5 * config.WHEEL_BASE_M * rads
        rw = lmps + 0.5 * config.WHEEL_BASE_M * rads
        max_speed = config.MAX_LINEAR_MPS + 0.5 * config.WHEEL_BASE_M * config.MAX_ANGULAR_RADPS
        if max_speed == 0:
            return 0, 0
        scale = config.MAX_PWM / max_speed
        return (
            max(-config.MAX_PWM, min(config.MAX_PWM, int(lw * scale))),
            max(-config.MAX_PWM, min(config.MAX_PWM, int(rw * scale))),
        )

    # ── Monitor loop (reconnect + watchdogs) ──────────────────────────────────

    def _monitor_loop(self) -> None:
        while self._running:
            time.sleep(0.5)
            with self.lock:
                age = time.time() - self.state.last_update
                self.state.telemetry_age_s = age
                self.state.connected = (
                    self.bridge.ser is not None
                    and age <= config.TELEMETRY_TIMEOUT_S
                )
                self.state.last_error = self.bridge.last_error or self.state.last_error

            # Reconnect if serial dropped
            if self.bridge.ser is None:
                self.bridge.connect()

            # Frontend heartbeat watchdog — auto-stop if client gone
            if (time.time() - self._last_heartbeat) > config.HEARTBEAT_TIMEOUT_S:
                if self.state.armed:
                    log.warning("Heartbeat lost — sending stop")
                    self.queue.push_stop()
