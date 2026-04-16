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
from app.robot.lidar import LidarLocalizer, LidarManager
from app.robot.mapping import OccupancyGridMap
from app.robot.poi import PoiManager
from app.robot.planner import astar, inflate_occupancy, nearest_free_cell, simplify_path
from app.robot.runtime_cfg import rtcfg
from app.robot.serial_bridge import ArduinoBridge
from app.robot.state import NavigationState, Pose, RobotState, wrap_angle

log = logging.getLogger(__name__)


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

        # Odometry helpers
        self._last_ticks: Optional[Tuple[int, int]] = None
        self._last_tel_ts: Optional[float] = None

        # Manual command override timer (managed here for display, enforced in queue)
        self._manual_expires_at: float = 0.0

        # Frontend heartbeat watchdog
        self._last_heartbeat: float = time.time()

        # Obstacle avoidance recovery state
        self._obstacle_reverse_until: float = 0.0   # deadline for reverse phase
        self._obstacle_blocked_at:    float = 0.0   # when obstacle was first detected
        self._replanning_after_obs:   bool  = False  # True while following the post-obstacle replan

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
        self.localizer.reset()
        self._replanning_after_obs   = False
        self._obstacle_blocked_at    = 0.0
        self._obstacle_reverse_until = 0.0

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
            self.state.nav.status = "start pose updated"
        self.localizer.reset()
        self._replanning_after_obs   = False
        self._obstacle_blocked_at    = 0.0
        self._obstacle_reverse_until = 0.0

    # ── POI / waypoint API (delegates to PoiManager) ─────────────────────────

    def poi_add(self, label: str, kind: str, x: float, y: float,
                note: str = "") -> dict:
        return self.poi.add(label, kind, x, y, note)

    def poi_update(self, poi_id: str, **fields) -> Optional[dict]:
        return self.poi.update(poi_id, **fields)

    def poi_remove(self, poi_id: str) -> bool:
        return self.poi.remove(poi_id)

    def poi_list(self) -> list:
        return self.poi.list()

    def poi_navigate(self, poi_id: str) -> bool:
        """Set navigation goal to the POI's location."""
        coords = self.poi.navigate_to(poi_id)
        if coords is None:
            return False
        self.set_goal(coords[0], coords[1])
        return True

    # ── Status / data API ─────────────────────────────────────────────────────

    def get_status(self) -> dict:
        with self.lock:
            self.state.lidar_connected = self.lidar.connected
            self.state.latest_scan_points = len(self.lidar.get_scan())
            self.state.last_error = self.bridge.last_error or self.state.last_error
            s = self.state.to_dict()
        # Obstacle countdown (seconds remaining before forced replan)
        if self._obstacle_blocked_at > 0:
            elapsed   = time.time() - self._obstacle_blocked_at
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
            if abs(norm_angle) < cone and dist < rtcfg.get("obstacle_stop_distance_m") * 3.0:
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
            payload = self.map.map_payload(
                self.state.pose,
                self.state.nav.path,
                self.state.nav.goal,
                scan_px=scan_px,
            )
        # Add POIs as pixel coordinates for canvas rendering
        pois_px = []
        for poi in self.poi.list():
            gx, gy = self.map.world_to_grid(poi["x"], poi["y"])
            pois_px.append({**poi, "px": gx, "py": gy})
        payload["pois"] = pois_px
        return payload

    def _scan_to_map_pixels(self, scan: List[Tuple[float, float]]) -> list:
        """Convert live lidar scan to map pixel coordinates.

        Uses the same world→grid transform as OccupancyGridMap.update_from_scan
        so live scan dots align exactly with the mapped obstacles.
        """
        out = []
        if not scan:
            return out
        step = max(1, len(scan) // 720)   # up to 720 points for density
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

    # ── 20 Hz control loop ────────────────────────────────────────────────────

    def _ctrl_loop(self) -> None:
        """20 Hz control loop.

        Fixes vs v3
        ───────────
        * Map updates run in a separate daemon thread so heavy numpy work
          never blocks the 20 Hz control tick → eliminates UI glitch when
          mapping + pilot control run simultaneously.
        * ICP localization is applied outside the main lock so it cannot
          starve the telemetry callback thread.
        * Obstacle detection and nav/pilot dispatch always run, regardless
          of LIDAR_LOCALIZATION_ENABLED state.
        """
        import queue as _queue

        # Dedicated map-update worker (decouples heavy numpy from ctrl tick)
        _map_q: "_queue.Queue[tuple]" = _queue.Queue(maxsize=2)

        def _map_worker() -> None:
            while self._running:
                try:
                    pose_snap, scan_snap = _map_q.get(timeout=0.5)
                    self.map.update_from_scan(pose_snap, scan_snap)
                except _queue.Empty:
                    pass
                except Exception as exc:
                    log.warning("Map worker error: %s", exc)

        import threading as _threading
        _map_thread = _threading.Thread(
            target=_map_worker, daemon=True, name="map-worker"
        )
        _map_thread.start()

        dt = 1.0 / config.CONTROL_HZ
        while self._running:
            t0 = time.time()
            try:
                # ── 1. Snapshot lidar + update odometry (under lock) ──────────
                with self.lock:
                    scan = self.lidar.get_scan()
                    self.state.lidar_connected = self.lidar.connected
                    self.state.latest_scan_points = len(scan)
                    self._update_odometry()
                    pose_snap = (self.state.pose.x,
                                 self.state.pose.y,
                                 self.state.pose.theta)

                # ── 2. ICP localization (outside lock — can be slow) ──────────
                if config.LIDAR_LOCALIZATION_ENABLED and scan:
                    corrected = self.localizer.correct(self.state.pose, scan)
                    with self.lock:
                        self.state.pose.x     = corrected.x
                        self.state.pose.y     = corrected.y
                        self.state.pose.theta = corrected.theta
                        pose_snap = (corrected.x, corrected.y, corrected.theta)

                # ── 3. Queue map update (non-blocking) ────────────────────────
                with self.lock:
                    do_map = self.state.mapping
                if do_map and scan:
                    from app.robot.state import Pose as _Pose
                    p = _Pose(*pose_snap)
                    try:
                        _map_q.put_nowait((p, scan))
                    except _queue.Full:
                        pass  # drop frame — map worker still catching up

                # ── 4. Obstacle detection + motion dispatch ───────────────────
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
            if abs(angle) < cone and dist < rtcfg.get("obstacle_stop_distance_m"):
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

        dist_to_goal = math.hypot(
            goal[0] - self.state.pose.x, goal[1] - self.state.pose.y
        )
        if dist_to_goal <= config.GOAL_TOLERANCE_M:
            self.queue.push_stop()
            self.state.nav.status  = "goal reached"
            self.state.nav.active  = False
            self.state.linear_cmd  = 0.0
            self.state.angular_cmd = 0.0
            self._obstacle_blocked_at    = 0.0
            self._obstacle_reverse_until = 0.0
            self._replanning_after_obs   = False
            return

        now = time.time()

        # ══════════════════════════════════════════════════════════════════════
        # Obstacle state machine
        # ──────────────────────────────────────────────────────────────────────
        # Phase 1  [0 … OBSTACLE_WAIT_BEFORE_REPLAN_S]:
        #   Stop and wait.  If obstacle clears naturally → resume immediately.
        #
        # Phase 2  [patience expired]:
        #   Brief reverse so the rover's body exits the stop zone.
        #
        # Phase 3  [reverse done]:
        #   Stamp the obstacle into the occupancy map, set _replanning_after_obs,
        #   and RETURN.  The next tick will drop into normal-nav and replan.
        #
        # Guard: _replanning_after_obs skips the wait loop so the rover can
        #   follow the newly planned path even if obstacle_stop is momentarily
        #   still True (it clears within 1–2 ticks as the rover moves away).
        # ══════════════════════════════════════════════════════════════════════
        if self.state.obstacle_stop and not self._replanning_after_obs:
            # ── Record first detection time ───────────────────────────────────
            if self._obstacle_blocked_at == 0.0:
                self._obstacle_blocked_at = now
                log.info(
                    "Obstacle detected — waiting %.0f s before replanning",
                    rtcfg.get("obstacle_wait_before_replan_s"),
                )

            elapsed = now - self._obstacle_blocked_at
            wait_s  = rtcfg.get("obstacle_wait_before_replan_s")

            # ── Phase 1: hold still, show countdown ──────────────────────────
            if elapsed < wait_s:
                self.queue.push_stop()
                remaining = wait_s - elapsed
                self.state.nav.status  = f"obstacle: waiting {remaining:.0f} s"
                self.state.linear_cmd  = 0.0
                self.state.angular_cmd = 0.0
                return

            # ── Phase 2: arm the reverse timer ────────────────────────────────
            if self._obstacle_reverse_until == 0.0:
                self._obstacle_reverse_until = now + rtcfg.get("obstacle_replan_reverse_s")
                log.info(
                    "Obstacle still present after %.0f s — reversing %.1f s",
                    wait_s, rtcfg.get("obstacle_replan_reverse_s"),
                )

            if now < self._obstacle_reverse_until:
                lp, rp = self._cmd_to_pwm(config.OBSTACLE_REVERSE_SPEED, 0.0)
                self.queue.push_nav(lp, rp)
                self.state.nav.status  = "obstacle: reversing to replan"
                self.state.linear_cmd  = config.OBSTACLE_REVERSE_SPEED
                self.state.angular_cmd = 0.0
                return

            # ── Phase 3: reverse done — stamp scan, arm replan, RETURN ────────
            #   *** The critical fix: RETURN here instead of falling through. ***
            #   Falling through caused _obstacle_blocked_at to be re-set to `now`
            #   on the very next tick (obstacle still in cone), restarting the
            #   5-second wait and preventing the A* plan from ever running.
            log.info("Reverse done — stamping obstacle into map and replanning")
            if scan:
                self.map.update_from_scan(self.state.pose, scan)
            self._obstacle_blocked_at    = 0.0
            self._obstacle_reverse_until = 0.0
            self._replanning_after_obs   = True   # skip wait loop next tick
            self.state.nav.last_plan_time = 0.0   # force immediate replan
            self.state.nav.path          = []
            self.state.nav.status        = "replanning around obstacle"
            return  # ← RETURN: planner fires on the next tick

        # ── Obstacle cleared naturally (before patience expired) ──────────────
        if self._obstacle_blocked_at != 0.0:
            elapsed = now - self._obstacle_blocked_at
            log.info("Obstacle cleared after %.1f s — resuming path", elapsed)
            self._obstacle_blocked_at    = 0.0
            self._obstacle_reverse_until = 0.0
            self._replanning_after_obs   = False
            self.state.nav.last_plan_time = 0.0   # fresh plan from new pose
            self.state.nav.path          = []

        # ── Normal navigation ─────────────────────────────────────────────────
        should_plan = (
            not self.state.nav.path
            or self._replanning_after_obs
            or (now - self.state.nav.last_plan_time) > rtcfg.get("nav_replan_seconds")
        )
        if should_plan:
            # Always stamp the current scan before planning so the map has
            # the latest obstacle information (critical for post-obstacle replan).
            if scan:
                self.map.update_from_scan(self.state.pose, scan)
            self._plan_path()
            self._replanning_after_obs = False   # clear whether or not plan succeeded
            if not self.state.nav.path:
                self.state.nav.status = "no path — obstacle blocking route"
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
        self.state.linear_cmd  = linear
        self.state.angular_cmd = angular
        self.state.nav.status  = "following path"
        lp, rp = self._cmd_to_pwm(linear, angular)
        self.queue.push_nav(lp, rp)

    def _plan_path(self) -> None:
        """Run A* from current pose to goal.

        If the rover's grid cell (or the goal cell) falls inside an inflated
        obstacle zone — which can happen when the rover backed up into a cell
        that was stamped occupied — nearest_free_cell() finds the closest
        unblocked cell so A* is never given an occupied start/goal.
        """
        occ     = self.map.occupancy_for_planner()
        occ_inf = inflate_occupancy(occ, config.PLANNER_INFLATION_CELLS)

        start_raw = self.map.world_to_grid(self.state.pose.x, self.state.pose.y)
        goal_raw  = self.map.world_to_grid(
            self.state.nav.goal[0], self.state.nav.goal[1]
        )

        # Nudge start/goal to nearest free cell if inflation overlaps them
        start = nearest_free_cell(occ_inf, start_raw, radius=8)
        goal  = nearest_free_cell(occ_inf, goal_raw,  radius=8)

        if start is None or goal is None:
            log.warning("_plan_path: start or goal surrounded by obstacles")
            self.state.nav.path = []
            self.state.nav.last_plan_time = time.time()
            return

        cells = astar(occ_inf, start, goal)
        self.state.nav.last_plan_time = time.time()
        if not cells:
            log.warning("_plan_path: A* found no path from %s to %s", start, goal)
            self.state.nav.path = []
            return
        world = [self.map.grid_to_world(x, y) for x, y in cells]
        self.state.nav.path = simplify_path(world)
        log.info("_plan_path: new path with %d waypoints", len(self.state.nav.path))

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
        lmps = linear  * rtcfg.get("max_linear_mps")
        rads = angular * config.MAX_ANGULAR_RADPS
        # Differential drive: turn left → right wheel faster, left wheel slower
        # angular > 0 means turn left: left PWM decreases, right PWM increases
        lw = lmps - 0.5 * config.WHEEL_BASE_M * rads   # left motor
        rw = lmps + 0.5 * config.WHEEL_BASE_M * rads   # right motor
        max_speed = config.MAX_LINEAR_MPS + 0.5 * config.WHEEL_BASE_M * config.MAX_ANGULAR_RADPS
        if max_speed == 0:
            return 0, 0
        scale = config.MAX_PWM / max_speed
        # NOTE: left/right are NOT swapped here — the Arduino wiring is correct.
        # Previously these were returning (left, right) but the motor labels on
        # the Arduino match (left_pwm, right_pwm) directly.
        left_pwm  = max(-config.MAX_PWM, min(config.MAX_PWM, int(lw * scale)))
        right_pwm = max(-config.MAX_PWM, min(config.MAX_PWM, int(rw * scale)))
        return left_pwm, right_pwm

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
