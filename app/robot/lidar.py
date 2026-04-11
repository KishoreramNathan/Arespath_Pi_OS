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
