"""DriveCommandQueue — serialises drive commands onto the Arduino serial link.

Design goals
────────────
* Manual commands always preempt autonomous nav commands.
* Throttle sends to ≥ 12 ms apart (the Arduino processes at ~80 Hz).
* Manual override timer: while a manual command is live, nav is blocked.
* ACK tracking: each SET is considered inflight until STOP or next SET.
* Idle watchdog: if no new command is pushed for WATCHDOG_IDLE_S seconds,
  a STOP is issued automatically.
* No asyncio — pure threading.

Public API
──────────
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

_MIN_SEND_INTERVAL = 0.012      # 12 ms — ~83 Hz ceiling on Arduino side


class DriveCommandQueue:
    """Thread-safe command serialiser for the Arduino bridge."""

    def __init__(self, bridge) -> None:
        """
        :param bridge: ArduinoBridge instance (must have set_pwm / stop methods).
        """
        self._bridge = bridge
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)

        # Pending command slots (None = nothing pending)
        self._manual_cmd: Optional[tuple] = None   # (left, right)
        self._nav_cmd: Optional[tuple] = None       # (left, right)
        self._stop_requested = False

        # Timing state
        self._manual_expires_at: float = 0.0
        self._last_send_time: float = 0.0
        self._last_push_time: float = time.time()

        self._running = False
        self._worker: Optional[threading.Thread] = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._worker = threading.Thread(
            target=self._work_loop, daemon=True, name="cmd-worker"
        )
        self._worker.start()

    def stop(self) -> None:
        self._running = False
        with self._cond:
            self._cond.notify_all()

    # ── Push API ──────────────────────────────────────────────────────────────

    def push_manual(self, left: int, right: int) -> None:
        """Queue a manual (highest-priority) drive command."""
        with self._cond:
            self._manual_cmd = (_clamp(left), _clamp(right))
            self._manual_expires_at = time.time() + config.MANUAL_COMMAND_TIMEOUT_S
            self._last_push_time = time.time()
            self._stop_requested = False
            self._cond.notify()

    def push_nav(self, left: int, right: int) -> None:
        """Queue an autonomous nav command (ignored during manual override)."""
        with self._cond:
            if time.time() < self._manual_expires_at:
                return   # manual override is active
            self._nav_cmd = (_clamp(left), _clamp(right))
            self._last_push_time = time.time()
            self._cond.notify()

    def push_stop(self) -> None:
        """Immediately flush all pending commands and send STOP."""
        with self._cond:
            self._manual_cmd = None
            self._nav_cmd = None
            self._stop_requested = True
            self._cond.notify()

    def manual_override_active(self) -> bool:
        with self._lock:
            return time.time() < self._manual_expires_at

    # ── Worker thread ─────────────────────────────────────────────────────────

    def _work_loop(self) -> None:
        while self._running:
            with self._cond:
                self._cond.wait(timeout=config.WATCHDOG_IDLE_S)
                if not self._running:
                    break

                stop_req = self._stop_requested
                manual = self._manual_cmd
                nav = self._nav_cmd
                now = time.time()
                idle = (now - self._last_push_time) >= config.WATCHDOG_IDLE_S

                # Consume pending commands
                self._stop_requested = False
                if manual is not None:
                    self._manual_cmd = None
                if nav is not None:
                    self._nav_cmd = None

            # Enforce send rate limit
            elapsed = time.time() - self._last_send_time
            if elapsed < _MIN_SEND_INTERVAL:
                time.sleep(_MIN_SEND_INTERVAL - elapsed)

            if stop_req or idle:
                self._bridge.stop()
                self._last_send_time = time.time()
            elif manual is not None:
                self._bridge.set_pwm(manual[0], manual[1])
                self._last_send_time = time.time()
            elif nav is not None:
                self._bridge.set_pwm(nav[0], nav[1])
                self._last_send_time = time.time()


def _clamp(value: int) -> int:
    return max(-255, min(255, int(value)))
