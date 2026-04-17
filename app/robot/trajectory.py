"""Path smoothing and trajectory generation module.

Implements ROS-like trajectory planning with:
- Cubic spline interpolation for smooth paths
- Velocity profiling based on path curvature
- Dynamic obstacle costmap integration

This module provides the foundation for smooth, natural robot movement.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class Waypoint:
    x: float
    y: float
    theta: float = 0.0
    v: float = 0.0


@dataclass
class TrajectoryPoint:
    x: float
    y: float
    theta: float
    v: float
    kappa: float = 0.0
    dt: float = 0.0


@dataclass 
class SmoothedPath:
    points: List[TrajectoryPoint] = field(default_factory=list)
    total_length: float = 0.0
    total_time: float = 0.0


class PathSmoother:
    """Cubic spline-based path smoother with velocity profiling."""

    def __init__(
        self,
        resolution: float = 0.05,
        max_velocity: float = 0.4,
        max_acceleration: float = 0.3,
        max_curvature: float = 2.0,
        lookahead: float = 0.3,
    ) -> None:
        self.resolution = resolution
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_curvature = max_curvature
        self.lookahead = lookahead

    def smooth(self, raw_path: List[Tuple[float, float]]) -> SmoothedPath:
        """Convert raw waypoints to smooth trajectory with velocity profile."""
        if len(raw_path) < 2:
            return SmoothedPath()

        smoothed = self._cubic_spline_interpolate(raw_path)
        curvatures = self._compute_curvatures(smoothed)
        self._apply_velocity_profile(smoothed, curvatures)
        self._compute_time_parameters(smoothed)

        path = SmoothedPath(
            points=smoothed,
            total_length=self._compute_path_length(smoothed),
            total_time=smoothed[-1].dt if smoothed else 0.0,
        )
        return path

    def _cubic_spline_interpolate(self, points: List[Tuple[float, float]]) -> List[TrajectoryPoint]:
        """Interpolate path using cubic splines for smooth curves."""
        if len(points) < 2:
            return []

        xs = np.array([p[0] for p in points])
        ys = np.array([p[1] for p in points])
        n = len(points)

        if n == 2:
            return self._linear_interpolate(points)

        distances = np.zeros(n)
        distances[1:] = np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))

        if distances[-1] < 0.01:
            return self._linear_interpolate(points)

        total_length = float(distances[-1])
        distances /= total_length

        num_samples = max(int(total_length / max(self.resolution, 1e-3)) + 1, n * 3)
        t_new = np.linspace(0, 1, num_samples)

        try:
            from scipy.interpolate import CubicSpline
            cx = CubicSpline(distances, xs)
            cy = CubicSpline(distances, ys)
            x_interp = cx(t_new)
            y_interp = cy(t_new)
        except ImportError:
            x_interp = np.interp(t_new, distances, xs)
            y_interp = np.interp(t_new, distances, ys)

        trajectory = []
        for i in range(len(x_interp)):
            x, y = x_interp[i], y_interp[i]

            if i < len(x_interp) - 1:
                dx = x_interp[i + 1] - x
                dy = y_interp[i + 1] - y
                theta = math.atan2(dy, dx)
            elif i > 0:
                dx = x - x_interp[i - 1]
                dy = y - y_interp[i - 1]
                theta = math.atan2(dy, dx)
            else:
                theta = 0.0

            trajectory.append(TrajectoryPoint(x=x, y=y, theta=theta, v=0.0))

        return trajectory

    def _linear_interpolate(self, points: List[Tuple[float, float]]) -> List[TrajectoryPoint]:
        """Fallback linear interpolation."""
        result = []
        for idx in range(len(points) - 1):
            x0, y0 = points[idx]
            x1, y1 = points[idx + 1]
            dx = x1 - x0
            dy = y1 - y0
            segment = math.hypot(dx, dy)
            steps = max(1, int(segment / max(self.resolution, 1e-3)))
            theta = math.atan2(dy, dx) if segment > 1e-6 else (result[-1].theta if result else 0.0)
            for step in range(steps):
                ratio = step / steps
                result.append(
                    TrajectoryPoint(
                        x=x0 + dx * ratio,
                        y=y0 + dy * ratio,
                        theta=theta,
                        v=0.0,
                    )
                )
        last_x, last_y = points[-1]
        last_theta = result[-1].theta if result else 0.0
        result.append(TrajectoryPoint(x=last_x, y=last_y, theta=last_theta, v=0.0))
        return result

    def _compute_curvatures(self, trajectory: List[TrajectoryPoint]) -> np.ndarray:
        """Compute discrete curvature at each point using finite differences."""
        n = len(trajectory)
        if n < 3:
            return np.zeros(n)

        kappas = np.zeros(n)
        for i in range(1, n - 1):
            p0, p1, p2 = trajectory[i - 1], trajectory[i], trajectory[i + 1]

            dx1 = p1.x - p0.x
            dy1 = p1.y - p0.y
            dx2 = p2.x - p1.x
            dy2 = p2.y - p1.y

            d1 = math.hypot(dx1, dy1)
            d2 = math.hypot(dx2, dy2)

            if d1 < 1e-6 or d2 < 1e-6:
                continue

            cross = dx1 * dy2 - dy1 * dx2
            ds = 0.5 * (d1 + d2)

            if ds > 1e-6:
                kappas[i] = abs(cross) / (d1 * d2) if d1 * d2 > 1e-9 else 0.0

        kappas[0] = kappas[1] if n > 1 else 0.0
        kappas[-1] = kappas[-2] if n > 1 else 0.0

        return kappas

    def _apply_velocity_profile(
        self,
        trajectory: List[TrajectoryPoint],
        curvatures: np.ndarray,
    ) -> None:
        """Apply velocity profile based on curvature (slow down for sharp turns)."""
        n = len(trajectory)
        if n == 0:
            return

        v_max = self.max_velocity
        v_min = 0.05

        kappa_max = self.max_curvature
        speed_factors = np.clip(1.0 - (curvatures / kappa_max), 0.15, 1.0)

        velocities = v_max * speed_factors

        velocities = np.clip(velocities, v_min, v_max)

        forward_pass = velocities.copy()
        for i in range(1, n):
            dv_max = self.max_acceleration * self.resolution / max(velocities[i - 1], 0.01)
            forward_pass[i] = min(forward_pass[i], forward_pass[i - 1] + dv_max)

        backward_pass = forward_pass.copy()
        for i in range(n - 2, -1, -1):
            dv_max = self.max_acceleration * self.resolution / max(backward_pass[i + 1], 0.01)
            backward_pass[i] = min(backward_pass[i], backward_pass[i + 1] + dv_max)

        if n > 1:
            backward_pass[-1] = 0.0

        if n > 0:
            start_dist = math.hypot(
                trajectory[1].x - trajectory[0].x,
                trajectory[1].y - trajectory[0].y
            ) if n > 1 else self.resolution
            ramp_dist = (v_max * 0.5) / self.max_acceleration
            ramp_ratio = min(1.0, start_dist / max(ramp_dist, 0.01))
            backward_pass[0] = v_max * ramp_ratio

        for i, pt in enumerate(trajectory):
            trajectory[i].v = max(0.0, backward_pass[i])
            trajectory[i].kappa = curvatures[i]

    def _compute_time_parameters(self, trajectory: List[TrajectoryPoint]) -> None:
        """Compute cumulative time at each point."""
        t = 0.0
        for i, pt in enumerate(trajectory):
            if i > 0 and pt.v > 0.01:
                dist = math.hypot(pt.x - trajectory[i - 1].x, pt.y - trajectory[i - 1].y)
                dt = dist / pt.v
            else:
                dt = 0.02
            t += dt
            pt.dt = t


class TrajectoryTracker:
    """Pure pursuit controller with dynamic lookahead and PID heading control."""

    def __init__(
        self,
        lookahead_min: float = 0.2,
        lookahead_max: float = 0.8,
        lookahead_gain: float = 0.8,
        kp_angular: float = 2.5,
        ki_angular: float = 0.1,
        kd_angular: float = 0.3,
        kp_linear: float = 1.0,
        rotate_in_place_threshold: float = 0.8,
    ) -> None:
        self.lookahead_min = lookahead_min
        self.lookahead_max = lookahead_max
        self.lookahead_gain = lookahead_gain
        self.kp_angular = kp_angular
        self.ki_angular = ki_angular
        self.kd_angular = kd_angular
        self.kp_linear = kp_linear
        self.rotate_in_place_threshold = rotate_in_place_threshold

        self._prev_heading_error = 0.0
        self._heading_integral = 0.0
        self._prev_linear_error = 0.0
        self._last_target_idx = 0

    def compute_cmd(
        self,
        pose: Tuple[float, float, float],
        trajectory: List[TrajectoryPoint],
        current_velocity: float = 0.0,
        dt: float = 0.02,
    ) -> Tuple[float, float]:
        """Compute velocity commands using pure pursuit + PID."""
        px, py, ptheta = pose

        if not trajectory:
            return 0.0, 0.0

        target_idx = self._find_lookahead_target(px, py, current_velocity, len(trajectory))

        if target_idx is None:
            return 0.0, 0.0

        target = trajectory[target_idx]
        target_x, target_y = target.x, target.y
        target_v = target.v

        dx = target_x - px
        dy = target_y - py
        dist_to_target = math.hypot(dx, dy)

        if dist_to_target < 0.05:
            return 0.0, 0.0

        alpha = math.atan2(dy, dx) - ptheta
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        ld = max(self.lookahead_min, min(self.lookahead_max, dist_to_target * self.lookahead_gain))

        if dist_to_target > 0.01:
            curvature = (2.0 * math.sin(alpha)) / dist_to_target
        else:
            curvature = 0.0

        heading_error = alpha
        self._heading_integral = np.clip(self._heading_integral + heading_error * dt, -0.5, 0.5)
        heading_derivative = (heading_error - self._prev_heading_error) / max(dt, 0.001)
        self._prev_heading_error = heading_error

        angular_raw = (
            self.kp_angular * heading_error
            + self.ki_angular * self._heading_integral
            + self.kd_angular * heading_derivative
        )

        angular_cmd = float(np.clip(angular_raw, -1.5, 1.5))

        linear_raw = target_v * self.kp_linear

        if abs(heading_error) > self.rotate_in_place_threshold:
            linear_cmd = linear_raw * 0.15
        else:
            linear_cmd = linear_raw * (1.0 - abs(heading_error) / math.pi)

        linear_cmd = float(np.clip(linear_cmd, -0.4, 0.4))

        return linear_cmd, angular_cmd

    def _find_lookahead_target(
        self,
        px: float,
        py: float,
        velocity: float,
        trajectory_length: int,
    ) -> Optional[int]:
        """Find the lookahead target index on the trajectory."""
        if trajectory_length == 0:
            return None

        dynamic_lookahead = max(
            self.lookahead_min,
            min(self.lookahead_max, abs(velocity) * 0.8 + self.lookahead_min)
        )

        nearest_idx = 0
        nearest_dist = float('inf')

        start_idx = min(max(self._last_target_idx, 0), max(trajectory_length - 1, 0))
        for i in range(start_idx, trajectory_length):
            pt = self._get_trajectory_point(i)
            if pt is None:
                continue
            dist = math.hypot(pt.x - px, pt.y - py)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_idx = i

        target_idx = nearest_idx
        for i in range(nearest_idx, trajectory_length):
            pt = self._get_trajectory_point(i)
            if pt is None:
                continue
            dist = math.hypot(pt.x - px, pt.y - py)
            if dist >= dynamic_lookahead:
                target_idx = i
                break

        self._last_target_idx = target_idx
        return target_idx

    _trajectory_cache: List[TrajectoryPoint] = []

    def set_trajectory(self, trajectory: List[TrajectoryPoint]) -> None:
        """Cache trajectory for tracking."""
        TrajectoryTracker._trajectory_cache = trajectory

    def _get_trajectory_point(self, idx: int) -> Optional[TrajectoryPoint]:
        """Get cached trajectory point."""
        if 0 <= idx < len(TrajectoryTracker._trajectory_cache):
            return TrajectoryTracker._trajectory_cache[idx]
        return None

    def reset(self) -> None:
        """Reset PID state."""
        self._prev_heading_error = 0.0
        self._heading_integral = 0.0
        self._prev_linear_error = 0.0


def smooth_path(
    raw_path: List[Tuple[float, float]],
    resolution: float = 0.05,
    max_velocity: float = 0.4,
) -> List[TrajectoryPoint]:
    """Convenience function to smooth a raw path."""
    smoother = PathSmoother(
        resolution=resolution,
        max_velocity=max_velocity,
    )
    smoothed = smoother.smooth(raw_path)
    return smoothed.points


def compute_curvature_at_point(
    p0: Tuple[float, float],
    p1: Tuple[float, float],
    p2: Tuple[float, float],
) -> float:
    """Compute curvature at p1 given three consecutive points."""
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = p2

    dx1 = x1 - x0
    dy1 = y1 - y0
    dx2 = x2 - x1
    dy2 = y2 - y1

    d1 = math.hypot(dx1, dy1)
    d2 = math.hypot(dx2, dy2)

    if d1 < 1e-6 or d2 < 1e-6:
        return 0.0

    cross = dx1 * dy2 - dy1 * dx2
    ds = 0.5 * (d1 + d2)

    if ds > 1e-6:
        return abs(cross) / (d1 * d2)

    return 0.0
