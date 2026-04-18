"""Local controller node inspired by Nav2 controller_server."""

from dataclasses import dataclass

from app.robot.runtime_cfg import rtcfg
from app.robot.trajectory import TrajectoryTracker


@dataclass(frozen=True)
class VelocityCommand:
    linear: float = 0.0
    angular: float = 0.0


class ControllerNode:
    def __init__(self) -> None:
        self._tracker = TrajectoryTracker(
            lookahead_min=rtcfg.get("lookahead_min_m"),
            lookahead_max=rtcfg.get("lookahead_max_m"),
            lookahead_gain=rtcfg.get("lookahead_gain"),
            kp_angular=rtcfg.get("kp_angular"),
            ki_angular=rtcfg.get("ki_angular"),
            kd_angular=rtcfg.get("kd_angular"),
            kp_linear=rtcfg.get("kp_linear"),
        )
        self._current_linear = 0.0
        self._current_angular = 0.0

    @property
    def current_linear(self) -> float:
        return self._current_linear

    @property
    def current_angular(self) -> float:
        return self._current_angular

    def reset(self) -> None:
        self._current_linear = 0.0
        self._current_angular = 0.0
        self._tracker.reset()

    def hold(self, dt: float) -> VelocityCommand:
        return self._slew_to(0.0, 0.0, dt)

    def reverse(self, linear_speed: float, dt: float) -> VelocityCommand:
        return self._slew_to(linear_speed, 0.0, dt)

    def track(self, pose, trajectory, dt: float) -> VelocityCommand:
        self._sync_tunables()
        self._tracker.set_trajectory(trajectory)
        linear_raw, angular_raw = self._tracker.compute_cmd(
            pose=pose,
            trajectory=trajectory,
            current_velocity=self._current_linear,
            dt=dt,
        )
        return self._slew_to(linear_raw, angular_raw, dt)

    def _sync_tunables(self) -> None:
        self._tracker.lookahead_min = rtcfg.get("lookahead_min_m")
        self._tracker.lookahead_max = rtcfg.get("lookahead_max_m")
        self._tracker.lookahead_gain = rtcfg.get("lookahead_gain")
        self._tracker.kp_angular = rtcfg.get("kp_angular")
        self._tracker.ki_angular = rtcfg.get("ki_angular")
        self._tracker.kd_angular = rtcfg.get("kd_angular")
        self._tracker.kp_linear = rtcfg.get("kp_linear")

    def _slew_to(self, target_linear: float, target_angular: float, dt: float) -> VelocityCommand:
        max_linear_change = rtcfg.get("nav_max_acceleration") * dt
        max_angular_change = rtcfg.get("nav_max_angular_accel") * dt

        linear_diff = target_linear - self._current_linear
        angular_diff = target_angular - self._current_angular

        if abs(linear_diff) > max_linear_change:
            linear_diff = max_linear_change if linear_diff > 0.0 else -max_linear_change
        if abs(angular_diff) > max_angular_change:
            angular_diff = max_angular_change if angular_diff > 0.0 else -max_angular_change

        self._current_linear += linear_diff
        self._current_angular += angular_diff
        return VelocityCommand(self._current_linear, self._current_angular)
