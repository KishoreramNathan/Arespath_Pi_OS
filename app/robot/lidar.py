"""Lidar manager — wraps YDLIDAR X2 via the official Python binding.

If the ``ydlidar`` C-extension is not available (simulator / dev machine),
a mock scan is generated so the rest of the stack can be exercised without
real hardware.

Public API
──────────
    manager.start()                    → begins background scan loop
    manager.stop()                     → signals the loop to exit
    manager.get_scan()                 → list[(angle_rad, dist_m)]
    manager.scan_cartesian(limit)      → list[{x, y}] in robot frame
    manager.connected                  → bool
    manager.error                      → str | None
"""

import logging
import math
import threading
import time
from typing import List, Optional, Tuple

from app import config

log = logging.getLogger(__name__)

Scan = List[Tuple[float, float]]   # (angle_rad, dist_m)


class LidarManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._scan: Scan = []
        self.connected = False
        self.error: Optional[str] = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="lidar"
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    # ── Data access ───────────────────────────────────────────────────────────

    def get_scan(self) -> Scan:
        with self._lock:
            return list(self._scan)

    def scan_cartesian(self, limit: Optional[int] = None) -> List[dict]:
        points = self.get_scan()
        if limit and len(points) > limit:
            step = max(1, len(points) // limit)
            points = points[::step]
        return [
            {"x": round(d * math.cos(a), 4), "y": round(d * math.sin(a), 4)}
            for a, d in points
        ]

    # ── Background thread ─────────────────────────────────────────────────────

    def _loop(self) -> None:
        try:
            import ydlidar  # type: ignore
        except Exception as exc:
            self.error = f"ydlidar binding unavailable: {exc}"
            log.warning("LiDAR not available — using mock scan: %s", exc)
            self._mock_loop()
            return
        self._hardware_loop()

    def _hardware_loop(self) -> None:
        import ydlidar  # type: ignore  # noqa: F811

        try:
            ydlidar.os_init()
            laser = ydlidar.CYdLidar()
            laser.setlidaropt(ydlidar.LidarPropSerialPort, config.LIDAR_SERIAL_PORT)
            laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, config.LIDAR_BAUDRATE)
            laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
            laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
            laser.setlidaropt(ydlidar.LidarPropScanFrequency, config.LIDAR_SCAN_FREQUENCY)
            laser.setlidaropt(ydlidar.LidarPropSampleRate, config.LIDAR_SAMPLE_RATE)
            laser.setlidaropt(ydlidar.LidarPropSingleChannel, config.LIDAR_SINGLE_CHANNEL)
            laser.setlidaropt(ydlidar.LidarPropMaxRange, config.LIDAR_MAX_RANGE_M)
            laser.setlidaropt(ydlidar.LidarPropMinRange, config.LIDAR_MIN_RANGE_M)

            if not laser.initialize():
                self.error = "Failed to initialise YDLIDAR"
                self.connected = False
                log.error(self.error)
                self._mock_loop()
                return
            if not laser.turnOn():
                self.error = "Failed to start YDLIDAR motor"
                self.connected = False
                log.error(self.error)
                self._mock_loop()
                return

            self.connected = True
            self.error = None
            log.info("YDLIDAR started on %s", config.LIDAR_SERIAL_PORT)
            scan_obj = ydlidar.LaserScan()

            while self._running and ydlidar.os_isOk():
                if not laser.doProcessSimple(scan_obj):
                    time.sleep(0.05)
                    continue
                points: Scan = []
                for i in range(scan_obj.points.size()):
                    pt = scan_obj.points[i]
                    angle = float(pt.angle)
                    dist = float(pt.range)
                    if not (math.isfinite(dist) and math.isfinite(angle)):
                        continue
                    if config.LIDAR_MIN_RANGE_M <= dist <= config.LIDAR_MAX_RANGE_M:
                        points.append((angle, dist))
                with self._lock:
                    self._scan = points
                time.sleep(0.01)

            laser.turnOff()
            laser.disconnecting()
            self.connected = False

        except Exception as exc:
            self.error = str(exc)
            self.connected = False
            log.exception("LiDAR hardware loop crashed: %s", exc)
            self._mock_loop()

    def _mock_loop(self) -> None:
        """Emit a simple circular mock scan so the UI is exercisable."""
        log.info("LiDAR mock mode active")
        angle_step = 2.0 * math.pi / 180
        t = 0.0
        while self._running:
            points: Scan = []
            for i in range(180):
                angle = -math.pi + i * angle_step
                # Vary distance gently to give the radar something to draw
                dist = 2.5 + 0.8 * math.sin(t + angle * 2)
                dist = max(config.LIDAR_MIN_RANGE_M, min(config.LIDAR_MAX_RANGE_M, dist))
                points.append((angle, dist))
            with self._lock:
                self._scan = points
            t += 0.15
            time.sleep(0.17)   # ~6 Hz mock update


# ═══════════════════════════════════════════════════════════════════════════════
# LiDAR-based pose correction (lightweight 2-D ICP)
# ═══════════════════════════════════════════════════════════════════════════════

import numpy as np  # noqa: E402  (already imported above via math/time)
from app.robot.runtime_cfg import rtcfg


class LidarLocalizer:
    """Point-to-point ICP that corrects odometry on slippery surfaces.

    Usage::

        localizer = LidarLocalizer()
        corrected_pose = localizer.correct(odom_pose, current_scan, map_scan)

    *odom_pose*    – Pose dataclass (x, y, theta) from wheel odometry.
    *current_scan* – list[(angle_rad, dist_m)] fresh from LidarManager.
    *map_scan*     – list[(angle_rad, dist_m)] reference cloud (previous scan or
                     map-derived points).  If None/empty the odom pose is returned
                     unchanged (first frame bootstrap).

    The corrected pose is blended with the odometry pose using
    ``config.LIDAR_ICP_WEIGHT`` so a single bad scan doesn't teleport the rover.
    """

    def __init__(self) -> None:
        self._ref_cloud: np.ndarray | None = None   # (N,2) xy in world frame
        self._last_run: float = 0.0

    # ── public ────────────────────────────────────────────────────────────────

    def correct(self, pose, current_scan: list) -> "Pose":  # type: ignore[name-defined]
        """Return pose corrected by ICP (or original pose if ICP not ready)."""
        from app import config
        from app.robot.state import Pose, wrap_angle

        now = time.time()
        if (now - self._last_run) < config.LIDAR_ICP_INTERVAL_S:
            return pose  # rate-limit ICP

        pts = self._scan_to_world(current_scan, pose)
        if pts is None or len(pts) < config.LIDAR_ICP_MIN_POINTS:
            self._ref_cloud = pts
            self._last_run = now
            return pose

        if self._ref_cloud is None or len(self._ref_cloud) < config.LIDAR_ICP_MIN_POINTS:
            self._ref_cloud = pts
            self._last_run = now
            return pose

        dx, dy, dtheta = self._icp(pts, self._ref_cloud)
        self._ref_cloud = pts   # update reference to current scan
        self._last_run = now

        w = rtcfg.get("lidar_icp_weight")
        corrected = Pose(
            x     = pose.x     + w * dx,
            y     = pose.y     + w * dy,
            theta = wrap_angle(pose.theta + w * dtheta),
        )
        return corrected

    def reset(self) -> None:
        """Call when map is cleared or robot is manually relocated."""
        self._ref_cloud = None

    # ── internals ─────────────────────────────────────────────────────────────

    @staticmethod
    def _scan_to_world(scan: list, pose) -> "np.ndarray | None":
        """Convert polar scan to world-frame (N,2) xy array."""
        if not scan:
            return None
        pts = []
        cos_t = math.cos(pose.theta)
        sin_t = math.sin(pose.theta)
        for angle, dist in scan:
            # robot frame
            rx = dist * math.cos(angle)
            ry = dist * math.sin(angle)
            # world frame
            wx = pose.x + cos_t * rx - sin_t * ry
            wy = pose.y + sin_t * rx + cos_t * ry
            pts.append((wx, wy))
        return np.array(pts, dtype=np.float32)

    @staticmethod
    def _icp(src: np.ndarray, ref: np.ndarray) -> tuple:
        """Simplified point-to-point ICP returning (dx, dy, dtheta)."""
        from app import config

        s = src.copy()
        total_dx = 0.0
        total_dy = 0.0
        total_dtheta = 0.0

        for _ in range(config.LIDAR_ICP_MAX_ITERATIONS):
            # Find nearest neighbours (brute force — small scans are fast)
            diffs   = ref[np.newaxis, :, :] - s[:, np.newaxis, :]  # (Ns, Nr, 2)
            dists2  = (diffs ** 2).sum(axis=2)                       # (Ns, Nr)
            nn_idx  = dists2.argmin(axis=1)                          # (Ns,)
            nn_dist = dists2[np.arange(len(s)), nn_idx] ** 0.5

            # Reject far correspondences
            mask = nn_dist < config.LIDAR_ICP_MAX_CORRESP_M
            if mask.sum() < 5:
                break
            matched_src = s[mask]
            matched_ref = ref[nn_idx[mask]]

            # Compute centroids
            cs = matched_src.mean(axis=0)
            cr = matched_ref.mean(axis=0)

            # SVD-based rotation
            H   = (matched_src - cs).T @ (matched_ref - cr)
            try:
                U, _, Vt = np.linalg.svd(H)
            except np.linalg.LinAlgError:
                break
            R   = Vt.T @ U.T
            # Ensure proper rotation (det = +1)
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T

            t   = cr - R @ cs
            dtheta_step = math.atan2(float(R[1, 0]), float(R[0, 0]))

            # Apply transform to src cloud
            s = (R @ s.T).T + t

            total_dx     += float(t[0])
            total_dy     += float(t[1])
            total_dtheta += dtheta_step

            if abs(t[0]) < config.LIDAR_ICP_TOLERANCE_M and \
               abs(t[1]) < config.LIDAR_ICP_TOLERANCE_M:
                break

        return total_dx, total_dy, total_dtheta
