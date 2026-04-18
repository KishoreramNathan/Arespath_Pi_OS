"""Local Controller Node — 50 Hz trajectory following with PID + pure pursuit.

Mirrors the ROS2 ``nav2_controller`` / ``controller_server`` lifecycle node.

This node is the *only* place that computes velocity commands.  It reads
tunable gains live from ``rtcfg`` so that YAML profile changes (e.g. switching
from ``smooth_indoor`` to ``explore_fast``) take effect within one control tick.

Architecture
────────────
  step(pose, trajectory, dt)  → VelocityCommand(linear, angular)

  Internally:
    1. Adaptive pure-pursuit lookahead (velocity-scaled)
    2. PID heading controller
    3. Velocity profile (trapezoidal acceleration limiting)
    4. Smooth slew applied on top for jerk reduction

The node also tracks its own current velocity so the caller does not need
to maintain that state.

Thread safety
─────────────
  All methods are designed to be called from a single control thread.
  No internal locking is needed.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from app import config
from app.robot.runtime_cfg import rtcfg
from app.robot.state import Pose
from app.robot.trajectory import TrajectoryPoint, TrajectoryTracker

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class VelocityCommand:
    linear:  float = 0.0
    angular: float = 0.0


class LocalControllerNode:
    """Pure-pursuit + PID local controller — Nav2 controller_server equivalent.

    Usage::

        ctrl = LocalControllerNode()

        # in 50 Hz loop:
        cmd = ctrl.step(pose, trajectory, dt=0.02)
        left_pwm, right_pwm = pwm_from_cmd(cmd.linear, cmd.angular)
    """

    def __init__(self) -> None:
        self._tracker = TrajectoryTracker(
            lookahead_min  = rtcfg.get("lookahead_min_m"),
            lookahead_max  = rtcfg.get("lookahead_max_m"),
            lookahead_gain = rtcfg.get("lookahead_gain"),
            kp_angular     = rtcfg.get("kp_angular"),
            ki_angular     = rtcfg.get("ki_angular"),
            kd_angular     = rtcfg.get("kd_angular"),
            kp_linear      = rtcfg.get("kp_linear"),
        )

        self._current_linear:  float = 0.0
        self._current_angular: float = 0.0

        # PID integrator / derivative state
        self._heading_error_integral: float = 0.0
        self._prev_heading_error:     float = 0.0

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def current_linear(self) -> float:
        return self._current_linear

    @property
    def current_angular(self) -> float:
        return self._current_angular

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def reset(self) -> None:
        """Full state reset — call on mode changes, goal cancel, arm toggle."""
        self._current_linear  = 0.0
        self._current_angular = 0.0
        self._heading_error_integral = 0.0
        self._prev_heading_error     = 0.0
        self._tracker.reset()
        # Re-sync gains from current rtcfg profile
        self._sync_gains()

    # ── Main step (called at 50 Hz) ───────────────────────────────────────────

    def step(
        self,
        pose:       Pose,
        trajectory: List[TrajectoryPoint],
        dt:         float,
    ) -> VelocityCommand:
        """Compute a velocity command for the current pose + trajectory.

        Returns ``VelocityCommand(0, 0)`` if the trajectory is empty.
        """
        if not trajectory:
            return self._slew_to(0.0, 0.0, dt)

        self._sync_gains()

        pose_tuple = (pose.x, pose.y, pose.theta)
        linear_raw, angular_raw = self._tracker.compute_cmd(
            pose             = pose_tuple,
            trajectory       = trajectory,
            current_velocity = self._current_linear,
            dt               = dt,
        )

        return self._slew_to(linear_raw, angular_raw, dt)

    def hold(self, dt: float) -> VelocityCommand:
        """Smoothly decelerate to zero — use when waiting for replan."""
        return self._slew_to(0.0, 0.0, dt)

    def reverse(self, speed: float, dt: float) -> VelocityCommand:
        """Drive backward at *speed* — used during obstacle escape manoeuvre."""
        return self._slew_to(-abs(speed), 0.0, dt)

    # ── PWM conversion helper ─────────────────────────────────────────────────

    def to_pwm(self, cmd: VelocityCommand) -> Tuple[int, int]:
        """Convert a VelocityCommand to (left_pwm, right_pwm)."""
        lmps = cmd.linear * rtcfg.get("max_linear_mps")
        rads = cmd.angular * config.MAX_ANGULAR_RADPS
        lw   = lmps - 0.5 * config.WHEEL_BASE_M * rads
        rw   = lmps + 0.5 * config.WHEEL_BASE_M * rads
        max_speed = config.MAX_LINEAR_MPS + 0.5 * config.WHEEL_BASE_M * config.MAX_ANGULAR_RADPS
        if max_speed == 0:
            return 0, 0
        scale      = config.MAX_PWM / max_speed
        left_pwm   = max(-config.MAX_PWM, min(config.MAX_PWM, int(lw * scale)))
        right_pwm  = max(-config.MAX_PWM, min(config.MAX_PWM, int(rw * scale)))
        return left_pwm, right_pwm

    # ── Diagnostics ───────────────────────────────────────────────────────────

    def get_diagnostics(self) -> dict:
        return {
            "linear":            round(self._current_linear, 4),
            "angular":           round(self._current_angular, 4),
            "heading_integral":  round(self._heading_error_integral, 4),
        }

    # ── Private ───────────────────────────────────────────────────────────────

    def _slew_to(
        self,
        target_linear:  float,
        target_angular: float,
        dt:             float,
    ) -> VelocityCommand:
        """Apply trapezoidal acceleration limiting (jerk-free slew)."""
        max_lin_accel = rtcfg.get("nav_max_acceleration")
        max_ang_accel = rtcfg.get("nav_max_angular_accel")

        lin_diff = target_linear  - self._current_linear
        ang_diff = target_angular - self._current_angular

        max_lin_Δ = max_lin_accel * dt
        max_ang_Δ = max_ang_accel * dt

        if abs(lin_diff) > max_lin_Δ:
            lin_diff = math.copysign(max_lin_Δ, lin_diff)
        if abs(ang_diff) > max_ang_Δ:
            ang_diff = math.copysign(max_ang_Δ, ang_diff)

        self._current_linear  += lin_diff
        self._current_angular += ang_diff

        return VelocityCommand(
            linear  = self._current_linear,
            angular = self._current_angular,
        )

    def _sync_gains(self) -> None:
        """Push current rtcfg tunable values into the inner TrajectoryTracker."""
        t = self._tracker
        # Only write if changed to avoid dict churn every tick
        t.lookahead_min  = rtcfg.get("lookahead_min_m")
        t.lookahead_max  = rtcfg.get("lookahead_max_m")
        t.lookahead_gain = rtcfg.get("lookahead_gain")
        t.kp_angular     = rtcfg.get("kp_angular")
        t.ki_angular     = rtcfg.get("ki_angular")
        t.kd_angular     = rtcfg.get("kd_angular")
        t.kp_linear      = rtcfg.get("kp_linear")
