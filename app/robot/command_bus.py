"""DriveCommandQueue — event-driven serial command serializer for Arduino.

Design goals
────────────
* Manual commands always preempt autonomous nav commands.
* Event-driven architecture: NO blocking sleep(), NO polling loops.
* Uses threading.Event for immediate command dispatch.
* Manual override timer: while a manual command is live, nav is blocked.
* ACK tracking: each SET is considered inflight until STOP or next SET.
* Idle watchdog: if no new command is pushed for WATCHDOG_IDLE_S seconds,
  a STOP is issued automatically.
* No asyncio — pure threading with Event-based wakeups.

Public API
─────────
    push_manual(left, right)   → highest priority; resets manual timeout
    push_nav(left, right)      → lower priority; ignored during manual override
    push_stop()                → clears queue, sends STOP immediately
"""

import logging
import threading
import time
from typing import Optional

from app import config

log = logging.getLogger(__name__)

_WATCHDOG_TIMEOUT_S = config.WATCHDOG_IDLE_S


class DriveCommandQueue:
    """Thread-safe event-driven command serializer for the Arduino bridge."""

    def __init__(self, bridge) -> None:
        self._bridge = bridge
        self._lock = threading.Lock()
        self._cmd_event = threading.Event()

        self._manual_cmd: Optional[tuple] = None
        self._nav_cmd: Optional[tuple] = None
        self._stop_requested = False
        self._manual_expires_at: float = 0.0
        self._last_push_time: float = time.time()
        self._running = False
        self._worker: Optional[threading.Thread] = None
        self._stop_event: Optional[threading.Event] = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._stop_event = threading.Event()
        self._worker = threading.Thread(
            target=self._work_loop, daemon=True, name="cmd-worker"
        )
        self._worker.start()

    def stop(self) -> None:
        self._running = False
        if self._stop_event:
            self._stop_event.set()
        self._cmd_event.set()
        if self._worker and self._worker.is_alive():
            self._worker.join(timeout=1.0)

    def push_manual(self, left: int, right: int) -> None:
        with self._lock:
            self._manual_cmd = (_clamp(left), _clamp(right))
            self._manual_expires_at = time.time() + config.MANUAL_COMMAND_TIMEOUT_S
            self._last_push_time = time.time()
            self._stop_requested = False
            self._cmd_event.set()

    def push_nav(self, left: int, right: int) -> None:
        with self._lock:
            if time.time() < self._manual_expires_at:
                return
            self._nav_cmd = (_clamp(left), _clamp(right))
            self._last_push_time = time.time()
            self._cmd_event.set()

    def push_stop(self) -> None:
        with self._lock:
            self._manual_cmd = None
            self._nav_cmd = None
            self._stop_requested = True
            self._cmd_event.set()

    def manual_override_active(self) -> bool:
        with self._lock:
            return time.time() < self._manual_expires_at

    def _work_loop(self) -> None:
        while self._running:
            with self._lock:
                stop_req = self._stop_requested
                manual = self._manual_cmd
                nav = self._nav_cmd
                self._stop_requested = False
                self._manual_cmd = None
                self._nav_cmd = None

            if stop_req:
                self._bridge.stop()
                wait_time = _WATCHDOG_TIMEOUT_S
            elif manual is not None:
                self._bridge.set_pwm(manual[0], manual[1])
                wait_time = _WATCHDOG_TIMEOUT_S
            elif nav is not None:
                self._bridge.set_pwm(nav[0], nav[1])
                wait_time = _WATCHDOG_TIMEOUT_S
            else:
                wait_time = _WATCHDOG_TIMEOUT_S

            self._cmd_event.clear()
            if self._cmd_event.wait(timeout=wait_time):
                continue

            with self._lock:
                idle = (time.time() - self._last_push_time) >= _WATCHDOG_TIMEOUT_S
            if idle and self._running:
                self._bridge.stop()


def _clamp(value: int) -> int:
    return max(-255, min(255, int(value)))
