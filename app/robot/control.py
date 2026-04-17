"""RobotRuntime — ROS-like navigation stack for Arespath Rover.

v7 Architecture (ROS Nav Stack inspired):
─────────────────────────────────────────
• Sensor Processing: LiDAR + radar fusion, dynamic obstacle detection
• Localization: ICP-corrected odometry (sensor fusion)
• Global Planner: JPS pathfinding with dynamic obstacle integration
• Local Planner: Spline-smoothed trajectory with velocity profiling
• Controller: Pure pursuit + PID heading control
• Motor Control: Smooth velocity commands with acceleration limiting

Thread Model:
  robot-ctrl (50 Hz)  — odometry, localization, nav/motor commands
  cmd-worker          — event-driven command serializer
  serial-reader       — background telemetry reader
  map-pusher (10 Hz)  — WebSocket map payload push
  planner-worker      — async path planning
  monitor             — heartbeat + reconnect watchdog

This is the main orchestration layer that integrates all subsystems into
a cohesive ROS-like navigation stack.
"""

import logging
import math
import queue
import threading
import time
from typing import List, Optional, Tuple

from app import config
from app.robot.command_bus import DriveCommandQueue
from app.robot.lidar import LidarLocalizer, LidarManager
from app.robot.map_renderer import MapRenderer
from app.robot.mapping import OccupancyGridMap
from app.robot.planner_adv import DynamicPlanner, inflate_obstacles, nearest_free_point
from app.robot.poi import PoiManager
from app.robot.trajectory import PathSmoother, SmoothedPath, TrajectoryPoint, TrajectoryTracker
from app.robot.runtime_cfg import rtcfg
from app.robot.serial_bridge import ArduinoBridge
from app.robot.state import Pose, RobotState, wrap_angle

log = logging.getLogger(__name__)

_CONTROL_HZ = 50.0
_CONTROL_DT = 1.0 / _CONTROL_HZ


class RobotRuntime:
    def __init__(self) -> None:
        self.state = RobotState()
        self.map = OccupancyGridMap()
        self.map_renderer = MapRenderer(
            grid_size=config.MAP_SIZE_CELLS,
            resolution=config.MAP_RESOLUTION_M,
            origin_x=config.MAP_ORIGIN_X_M,
            origin_y=config.MAP_ORIGIN_Y_M,
        )
        self.lidar = LidarManager()
        self.localizer = LidarLocalizer()
        self.bridge = ArduinoBridge(on_telemetry=self._on_telemetry)
        self.queue = DriveCommandQueue(self.bridge)
        self.poi = PoiManager(config.POIS_FILE)
        self.lock = threading.Lock()

        self._running = False
        self._ctrl_thread: Optional[threading.Thread] = None
        self._monitor_thread: Optional[threading.Thread] = None
        self._map_pusher: Optional[threading.Thread] = None
        self._planner_thread: Optional[threading.Thread] = None
        self._map_pusher_event = threading.Event()
        self._socketio = None

        self._last_ticks: Optional[Tuple[int, int]] = None
        self._last_tel_ts: Optional[float] = None

        self._manual_expires_at: float = 0.0
        self._last_heartbeat: float = time.time()

        self._obstacle_reverse_until: float = 0.0
        self._obstacle_blocked_at: float = 0.0
        self._replanning_after_obs: bool = False

        self._last_map_push: float = 0.0
        self._map_push_interval: float = 0.1

        self._planner = DynamicPlanner(
            grid_size=config.MAP_SIZE_CELLS,
            resolution=config.MAP_RESOLUTION_M,
            inflation_radius=config.PLANNER_INFLATION_CELLS,
        )

        self._path_smoother = PathSmoother(
            resolution=0.05,
            max_velocity=config.MAX_LINEAR_MPS,
            max_acceleration=rtcfg.get("nav_max_acceleration"),
            lookahead=rtcfg.get("trajectory_lookahead_m"),
        )

        self._trajectory_tracker = TrajectoryTracker(
            lookahead_min=rtcfg.get("lookahead_min_m"),
            lookahead_max=rtcfg.get("lookahead_max_m"),
            lookahead_gain=rtcfg.get("lookahead_gain"),
            kp_angular=rtcfg.get("kp_angular"),
            ki_angular=rtcfg.get("ki_angular"),
            kd_angular=rtcfg.get("kd_angular"),
            kp_linear=rtcfg.get("kp_linear"),
        )

        self._current_trajectory: List[TrajectoryPoint] = []
        self._current_smoothed_path: SmoothedPath = SmoothedPath()
        self._planner_request_queue: queue.Queue = queue.Queue(maxsize=2)
        self._planner_result: Optional[List[Tuple[float, float]]] = None

        self._target_linear: float = 0.0
        self._target_angular: float = 0.0
        self._current_linear: float = 0.0
        self._current_angular: float = 0.0

        self._last_replan_time: float = 0.0
        self._path_valid: bool = False

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

        self._planner_thread = threading.Thread(
            target=self._planner_loop, daemon=True, name="planner-worker"
        )
        self._planner_thread.start()

        self._ctrl_thread = threading.Thread(
            target=self._ctrl_loop, daemon=True, name="robot-ctrl"
        )
        self._ctrl_thread.start()

        self._map_pusher = threading.Thread(
            target=self._map_push_loop, daemon=True, name="map-pusher"
        )
        self._map_pusher.start()

        self._monitor_thread = threading.Thread(
            target=self._monitor_loop, daemon=True, name="robot-monitor"
        )
        self._monitor_thread.start()

        log.info("RobotRuntime started (v7 ROS-like navigation stack)")

    def stop(self) -> None:
        self._running = False
        self._map_pusher_event.set()
        self.queue.stop()
        self.bridge.close()
        self.lidar.stop()
        self.map_renderer.shutdown()
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
                self._current_linear = 0.0
                self._current_angular = 0.0
        if not armed:
            self.queue.push_stop()

    def set_manual_command(self, linear: float, angular: float) -> None:
        with self.lock:
            if not self.state.armed:
                return
            self.state.linear_cmd = max(-1.0, min(1.0, linear))
            self.state.angular_cmd = max(-1.0, min(1.0, angular))
            self.state.mode = "pilot"
            self._manual_expires_at = time.time() + config.MANUAL_COMMAND_TIMEOUT_S
            self._current_trajectory = []
            self._path_valid = False
        left_pwm, right_pwm = self._cmd_to_pwm(linear, angular)
        self.queue.push_manual(left_pwm, right_pwm)

    def force_stop(self) -> None:
        with self.lock:
            self.state.linear_cmd = 0.0
            self.state.angular_cmd = 0.0
            self._current_linear = 0.0
            self._current_angular = 0.0
            self._current_trajectory = []
            self._path_valid = False
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
        with self.lock:
            self.state.mode = "mission"
            nav = self.state.nav
            nav.active = True
            nav.goal = (float(x), float(y))
            nav.status = "planning"
            nav.path = []
            nav.current_target = None
            nav.last_plan_time = 0.0
            self._obstacle_blocked_at = 0.0
            self._obstacle_reverse_until = 0.0
            self._replanning_after_obs = False
            self._current_trajectory = []
            self._current_smoothed_path = SmoothedPath()
            self._path_valid = False
            self._trajectory_tracker.reset()

        try:
            self._planner_request_queue.put_nowait((time.time(), (x, y)))
        except queue.Full:
            log.warning("Planner queue full, dropping request")

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
        self._current_linear = 0.0
        self._current_angular = 0.0
        self._current_trajectory = []
        self._current_smoothed_path = SmoothedPath()
        self._path_valid = False
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
        self._planner.update_static_map(self.map.occupancy_for_planner())
        self._replanning_after_obs = False
        self._obstacle_blocked_at = 0.0
        self._obstacle_reverse_until = 0.0

    def save_map(self, name: str) -> dict:
        return self.map.save(name)

    def load_map(self, name: str) -> None:
        self.map.load(name)
        self.localizer.reset()
        self._planner.update_static_map(self.map.occupancy_for_planner())

    def set_pose(self, x: float, y: float, theta: float) -> None:
        with self.lock:
            self.state.pose = Pose(x, y, theta)
            self.bridge.reset_odometry()
            self._last_ticks = None
            self._last_tel_ts = None
            self.state.nav.status = "start pose updated"
        self.localizer.reset()
        self._replanning_after_obs = False
        self._obstacle_blocked_at = 0.0
        self._obstacle_reverse_until = 0.0
        self._current_trajectory = []
        self._path_valid = False

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
        if self._obstacle_blocked_at > 0:
            elapsed = time.time() - self._obstacle_blocked_at
            remaining = max(0.0, rtcfg.get("obstacle_wait_before_replan_s") - elapsed)
            s["obstacle_wait_remaining_s"] = round(remaining, 1)
        else:
            s["obstacle_wait_remaining_s"] = None
        return s

    def get_lidar_payload(self) -> dict:
        with self.lock:
            points = self.lidar.scan_cartesian(config.LIDAR_RENDER_MAX_POINTS)
            avoidance_route = self._compute_avoidance_route(self.lidar.get_scan())
            return {
                "connected": self.lidar.connected,
                "error": self.lidar.error,
                "points": points,
                "obstacle_stop": self.state.obstacle_stop,
                "max_range": config.LIDAR_MAX_RANGE_M,
                "avoidance_route": avoidance_route,
            }

    def _compute_avoidance_route(self, scan) -> list:
        if not scan or not self.state.obstacle_stop:
            return []
        cone = math.radians(40)
        nearest_dist = config.LIDAR_MAX_RANGE_M
        nearest_angle = 0.0
        for angle, dist in scan:
            if dist <= 0:
                continue
            norm_angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(norm_angle) < cone and dist < rtcfg.get("obstacle_stop_distance_m") * 3.0:
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_angle = norm_angle
        if nearest_dist >= config.LIDAR_MAX_RANGE_M:
            return []
        obs_x = nearest_dist * math.cos(nearest_angle)
        obs_y = nearest_dist * math.sin(nearest_angle)
        left_pts = [d for a, d in scan if 0.25 < math.atan2(math.sin(a), math.cos(a)) < 1.57]
        right_pts = [d for a, d in scan if -1.57 < math.atan2(math.sin(a), math.cos(a)) < -0.25]
        left_clear = min(left_pts, default=config.LIDAR_MAX_RANGE_M)
        right_clear = min(right_pts, default=config.LIDAR_MAX_RANGE_M)
        side = 1.0 if left_clear >= right_clear else -1.0
        side_label = "left" if side > 0 else "right"
        detour_y = side * max(0.40, nearest_dist * 0.60)
        clear_dist = left_clear if side > 0 else right_clear
        wp1 = {"x": round(max(0.05, obs_x * 0.20), 3), "y": round(detour_y, 3)}
        wp2 = {"x": round(obs_x + 0.20, 3), "y": round(detour_y, 3)}
        wp3 = {"x": round(obs_x + 0.45, 3), "y": 0.0}
        return [
            {"side": side_label, "clear_m": round(clear_dist, 2), **wp1},
            wp2,
            wp3,
        ]

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

    def _planner_loop(self) -> None:
        """Background planner worker thread."""
        log.info("Planner worker started")
        while self._running:
            try:
                request_time, goal = self._planner_request_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                start = (self.state.pose.x, self.state.pose.y)
                raw_path = self._planner.plan(
                    start=start,
                    goal=goal,
                    grid_origin=(config.MAP_ORIGIN_X_M, config.MAP_ORIGIN_Y_M),
                    use_jps=True,
                )

                if raw_path and len(raw_path) >= 2:
                    smoothed = self._path_smoother.smooth(raw_path)
                    TrajectoryTracker._trajectory_cache = smoothed.points
                    path_for_state = [(pt.x, pt.y) for pt in smoothed.points]

                    with self.lock:
                        self._current_smoothed_path = smoothed
                        self._current_trajectory = smoothed.points
                        self._planner_result = path_for_state
                        self._path_valid = True
                        self.state.nav.path = path_for_state
                        self.state.nav.status = "following path"
                        self.state.nav.last_plan_time = time.time()
                else:
                    with self.lock:
                        self._current_trajectory = []
                        self._current_smoothed_path = SmoothedPath()
                        self._planner_result = []
                        self._path_valid = False
                        self.state.nav.path = []
                        self.state.nav.status = "no path found"

            except Exception as exc:
                log.exception("Planner error: %s", exc)
                with self.lock:
                    self.state.nav.status = f"planner error: {exc}"

        log.info("Planner worker stopped")

    def _ctrl_loop(self) -> None:
        """Main 50 Hz control loop."""
        import queue as _queue
        _map_q: "_queue.Queue[tuple]" = _queue.Queue(maxsize=2)

        def _map_worker() -> None:
            while self._running:
                try:
                    pose_snap, scan_snap = _map_q.get(timeout=0.2)
                    self.map.update_from_scan(pose_snap, scan_snap)
                    self.map_renderer.update_map(self.map.log_odds)
                except _queue.Empty:
                    pass
                except Exception as exc:
                    log.warning("Map worker error: %s", exc)

        _map_thread = threading.Thread(target=_map_worker, daemon=True, name="map-worker")
        _map_thread.start()

        next_tick = time.perf_counter()
        while self._running:
            t0 = time.perf_counter()
            try:
                with self.lock:
                    scan = self.lidar.get_scan()
                    self.state.lidar_connected = self.lidar.connected
                    self.state.latest_scan_points = len(scan)
                    self._update_odometry()
                    pose_snap = (self.state.pose.x, self.state.pose.y, self.state.pose.theta)

                if config.LIDAR_LOCALIZATION_ENABLED and scan:
                    corrected = self.localizer.correct(self.state.pose, scan)
                    with self.lock:
                        self.state.pose.x = corrected.x
                        self.state.pose.y = corrected.y
                        self.state.pose.theta = corrected.theta
                        pose_snap = (corrected.x, corrected.y, corrected.theta)

                with self.lock:
                    do_map = self.state.mapping
                if do_map and scan:
                    from app.robot.state import Pose as _Pose
                    p = _Pose(*pose_snap)
                    try:
                        _map_q.put_nowait((p, scan))
                    except _queue.Full:
                        pass

                with self.lock:
                    self.state.obstacle_stop = self._obstacle_in_cone(scan)
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
        left_dist = circ * (dl / config.TICKS_PER_WHEEL_REV)
        right_dist = circ * (dr / config.TICKS_PER_WHEEL_REV)
        ds = 0.5 * (left_dist + right_dist)
        dtheta = (right_dist - left_dist) / config.WHEEL_BASE_M
        mid = self.state.pose.theta + 0.5 * dtheta
        self.state.pose.x += ds * math.cos(mid)
        self.state.pose.y += ds * math.sin(mid)
        self.state.pose.theta = wrap_angle(self.state.pose.theta + dtheta)

    def _obstacle_in_cone(self, scan: List[Tuple[float, float]]) -> bool:
        cone_deg = rtcfg.get("obstacle_detection_cone_deg")
        cone = math.radians(cone_deg)
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
        """Advanced navigation with smooth trajectory following."""
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
            self.state.nav.status = "goal reached"
            self.state.nav.active = False
            self.state.nav.path = []
            self.state.nav.current_target = None
            self.state.linear_cmd = 0.0
            self.state.angular_cmd = 0.0
            self._current_linear = 0.0
            self._current_angular = 0.0
            self._obstacle_blocked_at = 0.0
            self._obstacle_reverse_until = 0.0
            self._replanning_after_obs = False
            return

        now = time.time()

        if self.state.obstacle_stop and not self._replanning_after_obs:
            if self._obstacle_blocked_at == 0.0:
                self._obstacle_blocked_at = now
                log.info("Obstacle detected — waiting %.0f s before replanning",
                         rtcfg.get("obstacle_wait_before_replan_s"))

            elapsed = now - self._obstacle_blocked_at
            wait_s = rtcfg.get("obstacle_wait_before_replan_s")

            if elapsed < wait_s:
                self._smooth_velocity(0.0, 0.0, _CONTROL_DT)
                lp, rp = self._cmd_to_pwm(self._current_linear, self._current_angular)
                self.queue.push_nav(lp, rp)
                remaining = wait_s - elapsed
                self.state.nav.status = f"obstacle: waiting {remaining:.0f} s"
                return

            if self._obstacle_reverse_until == 0.0:
                self._obstacle_reverse_until = now + rtcfg.get("obstacle_replan_reverse_s")
                log.info("Obstacle still present after %.0f s — reversing %.1f s",
                         wait_s, rtcfg.get("obstacle_replan_reverse_s"))

            if now < self._obstacle_reverse_until:
                self._smooth_velocity(config.OBSTACLE_REVERSE_SPEED, 0.0, _CONTROL_DT)
                lp, rp = self._cmd_to_pwm(self._current_linear, self._current_angular)
                self.queue.push_nav(lp, rp)
                self.state.nav.status = "obstacle: reversing to replan"
                return

            log.info("Reverse done — replanning around transient obstacle")
            self._obstacle_blocked_at = 0.0
            self._obstacle_reverse_until = 0.0
            self._replanning_after_obs = True
            self.state.nav.last_plan_time = 0.0
            self.state.nav.path = []
            self._current_trajectory = []
            self._path_valid = False
            self.state.nav.status = "replanning around obstacle"

            try:
                self._planner_request_queue.put_nowait((now, goal))
            except queue.Full:
                pass
            return

        if self._obstacle_blocked_at != 0.0:
            elapsed = now - self._obstacle_blocked_at
            log.info("Obstacle cleared after %.1f s — resuming path", elapsed)
            self._obstacle_blocked_at = 0.0
            self._obstacle_reverse_until = 0.0
            self._replanning_after_obs = False
            self.state.nav.last_plan_time = 0.0
            self._path_valid = False

        should_plan = (
            not self._path_valid
            or self._replanning_after_obs
            or (now - self.state.nav.last_plan_time) > rtcfg.get("nav_replan_seconds")
        )
        if should_plan and not self._replanning_after_obs:
            try:
                self._planner_request_queue.put_nowait((now, goal))
            except queue.Full:
                pass

        self._trajectory_tracker.set_trajectory(self._current_trajectory)

        pose = (self.state.pose.x, self.state.pose.y, self.state.pose.theta)
        linear_raw, angular_raw = self._trajectory_tracker.compute_cmd(
            pose=pose,
            trajectory=self._current_trajectory,
            current_velocity=self._current_linear,
            dt=_CONTROL_DT,
        )

        self._smooth_velocity(linear_raw, angular_raw, _CONTROL_DT)

        self.state.linear_cmd = self._current_linear
        self.state.angular_cmd = self._current_angular
        self.state.nav.status = "following path"

        lp, rp = self._cmd_to_pwm(self._current_linear, self._current_angular)
        self.queue.push_nav(lp, rp)

    def _smooth_velocity(
        self,
        target_linear: float,
        target_angular: float,
        dt: float,
    ) -> None:
        """Apply smooth velocity limiting for gradual acceleration/deceleration."""
        max_linear_accel = rtcfg.get("nav_max_acceleration")
        max_angular_accel = rtcfg.get("nav_max_angular_accel")

        linear_diff = target_linear - self._current_linear
        angular_diff = target_angular - self._current_angular

        max_linear_change = max_linear_accel * dt
        max_angular_change = max_angular_accel * dt

        if abs(linear_diff) > max_linear_change:
            linear_diff = math.copysign(max_linear_change, linear_diff)
        if abs(angular_diff) > max_angular_change:
            angular_diff = math.copysign(max_angular_change, angular_diff)

        self._current_linear += linear_diff
        self._current_angular += angular_diff

    def _cmd_to_pwm(self, linear: float, angular: float) -> Tuple[int, int]:
        """Convert normalized commands to PWM values."""
        lmps = linear * rtcfg.get("max_linear_mps")
        rads = angular * config.MAX_ANGULAR_RADPS
        lw = lmps - 0.5 * config.WHEEL_BASE_M * rads
        rw = lmps + 0.5 * config.WHEEL_BASE_M * rads
        max_speed = config.MAX_LINEAR_MPS + 0.5 * config.WHEEL_BASE_M * config.MAX_ANGULAR_RADPS
        if max_speed == 0:
            return 0, 0
        scale = config.MAX_PWM / max_speed
        left_pwm = max(-config.MAX_PWM, min(config.MAX_PWM, int(lw * scale)))
        right_pwm = max(-config.MAX_PWM, min(config.MAX_PWM, int(rw * scale)))
        return left_pwm, right_pwm

    def _map_push_loop(self) -> None:
        """Push map updates via Socket.IO."""
        _last_png_gen: int = -1

        while self._running:
            self._map_pusher_event.wait(timeout=0.10)
            self._map_pusher_event.clear()

            if not self._running:
                break

            try:
                with self.lock:
                    scan_px = self._scan_to_map_pixels(self.lidar.get_scan())
                    current_gen = self.map._scan_gen

                payload = self.get_map_payload()

                if current_gen == _last_png_gen and payload.get("image_png_b64"):
                    payload.pop("image_png_b64", None)
                else:
                    _last_png_gen = current_gen

                if self._socketio:
                    self._socketio.emit("map_update", payload)

                self._last_map_push = time.time()
            except Exception as exc:
                log.debug("Map push error: %s", exc)

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
                    log.warning("Heartbeat lost (%.1f s) — sending stop", hb_age)
                    self.queue.push_stop()
                    with self.lock:
                        if self.state.nav.active:
                            self._cancel_nav_locked()
            time.sleep(0.25)
