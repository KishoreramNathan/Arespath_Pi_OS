"""Arduino serial bridge.

Non-blocking design:
  • A background reader thread decodes every inbound line and calls
    ``on_telemetry`` with a plain dict.
  • ``send()`` is protected by a lock so any thread can call it safely.
  • Auto-reconnect is handled by ``RobotRuntime``'s monitor loop, not here.

Supported inbound frames
    TEL,<ms>,<l_ticks>,<r_ticks>,<l_rpm>,<r_rpm>,<l_pwm>,<r_pwm>,<batt_v>
    ACK <cmd>
    PONG ARESPATH
    ERR ...

Outbound commands (newline terminated):
    PING            — probe handshake
    SET <l> <r>     — set left/right PWM (-255 … 255)
    STOP            — stop both motors immediately
    RESET_ODOM      — zero encoder tick counters
"""

import glob
import logging
import threading
import time
from typing import Callable, Optional

import serial

from app import config

log = logging.getLogger(__name__)


class ArduinoBridge:
    def __init__(self, on_telemetry: Callable[[dict], None]) -> None:
        self.on_telemetry = on_telemetry
        self.ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._running = False
        self._reader: Optional[threading.Thread] = None
        self.last_error: Optional[str] = None

    # ── Connection ────────────────────────────────────────────────────────────

    def _candidate_ports(self) -> list:
        seen: list = []
        for port in config.SERIAL_CANDIDATES:
            if port not in seen:
                seen.append(port)
        for port in sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*")):
            if port not in seen:
                seen.append(port)
        return seen

    def connect(self) -> bool:
        for port in self._candidate_ports():
            try:
                ser = serial.Serial(port, config.SERIAL_BAUD, timeout=0.2)
                time.sleep(2.0)          # allow Arduino to boot after DTR reset
                ser.reset_input_buffer()
                ser.write(b"PING\n")
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if "PONG" in line or "ARES" in line or line.startswith("OK"):
                    self.ser = ser
                    self.last_error = None
                    self._running = True
                    self._reader = threading.Thread(
                        target=self._reader_loop,
                        daemon=True,
                        name="serial-reader",
                    )
                    self._reader.start()
                    log.info("Arduino connected on %s", port)
                    return True
                ser.close()
            except Exception as exc:
                self.last_error = str(exc)
                log.debug("Serial probe failed on %s: %s", port, exc)
        log.warning("No Arduino found — running without hardware")
        return False

    def close(self) -> None:
        self._running = False
        if self.ser:
            try:
                self.stop()
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    # ── Reader thread ─────────────────────────────────────────────────────────

    def _reader_loop(self) -> None:
        while self._running:
            try:
                if not self.ser:
                    break
                raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    continue
                self._parse_line(raw)
            except serial.SerialException as exc:
                self.last_error = str(exc)
                log.warning("Serial read error: %s", exc)
                time.sleep(0.5)
            except Exception as exc:
                self.last_error = str(exc)
                time.sleep(0.1)

    def _parse_line(self, raw: str) -> None:
        if raw.startswith("TEL,"):
            self._parse_tel(raw)
        elif raw.startswith("ACK") or raw.startswith("OK") or raw.startswith("PONG"):
            pass  # acknowledged — no action needed
        elif raw.startswith("FAILSAFE"):
            log.warning("Arduino failsafe: %s", raw)
            self.last_error = raw
        elif raw.startswith("ERR"):
            self.last_error = raw
            log.warning("Arduino error: %s", raw)

    def _parse_tel(self, raw: str) -> None:
        parts = raw.split(",")
        if len(parts) < 7:
            self.last_error = f"Short TEL frame: {raw!r}"
            return
        try:
            data: dict = {
                "t_ms":        int(parts[1]),
                "left_ticks":  int(parts[2]),
                "right_ticks": int(parts[3]),
                "left_rpm":    float(parts[4]),
                "right_rpm":   float(parts[5]),
                "left_pwm":    int(parts[6]) if parts[6] else 0,
                "right_pwm":   int(parts[7]) if len(parts) > 7 and parts[7] else 0,
                "battery_v":   float(parts[8]) if len(parts) > 8 and parts[8] else None,
            }
            self.on_telemetry(data)
        except (ValueError, IndexError) as exc:
            self.last_error = f"TEL parse error: {exc} | {raw!r}"

    # ── Write helpers ─────────────────────────────────────────────────────────

    def send(self, text: str) -> bool:
        """Send a newline-terminated command. Thread-safe."""
        with self._lock:
            if not self.ser:
                return False
            try:
                self.ser.write((text.strip() + "\n").encode("utf-8"))
                return True
            except Exception as exc:
                self.last_error = str(exc)
                log.warning("Serial send error: %s", exc)
                self.ser = None          # signal reconnect needed
                return False

    def set_pwm(self, left: int, right: int) -> bool:
        left = max(-255, min(255, int(left)))
        right = max(-255, min(255, int(right)))
        return self.send(f"SET {left} {right}")

    def stop(self) -> bool:
        return self.send("STOP")

    def reset_odometry(self) -> bool:
        return self.send("RESET_ODOM")
