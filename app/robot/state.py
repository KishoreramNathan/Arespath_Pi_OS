"""Shared robot state dataclasses.

All fields are plain Python types so `to_dict()` is always JSON-safe.
"""
import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


def wrap_angle(angle: float) -> float:
    """Wrap *angle* to the range (−π, π]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # radians, robot-forward = +X axis, CCW positive

    def to_dict(self) -> dict:
        return {"x": round(self.x, 4), "y": round(self.y, 4), "theta": round(self.theta, 4)}


@dataclass
class NavigationState:
    active: bool = False
    goal: Optional[Tuple[float, float]] = None
    path: List[Tuple[float, float]] = field(default_factory=list)
    mission_waypoints: List[Tuple[float, float]] = field(default_factory=list)
    mission_index: int = 0
    status: str = "idle"
    current_target: Optional[Tuple[float, float]] = None
    last_plan_time: float = 0.0

    def to_dict(self) -> dict:
        return {
            "active": self.active,
            "goal": list(self.goal) if self.goal else None,
            "path": [list(p) for p in self.path],
            "mission_waypoints": [list(p) for p in self.mission_waypoints],
            "mission_index": self.mission_index,
            "status": self.status,
            "current_target": list(self.current_target) if self.current_target else None,
        }


@dataclass
class RobotState:
    # Modes: "pilot" (manual) or "mission" (autonomous)
    mode: str = "pilot"
    armed: bool = False
    connected: bool = False            # Arduino serial link alive
    lidar_connected: bool = False
    mapping: bool = False
    obstacle_stop: bool = False

    pose: Pose = field(default_factory=Pose)
    nav: NavigationState = field(default_factory=NavigationState)

    # Telemetry from Arduino
    left_ticks: int = 0
    right_ticks: int = 0
    left_rpm: float = 0.0
    right_rpm: float = 0.0
    left_pwm: int = 0
    right_pwm: int = 0
    battery_v: Optional[float] = None

    # Normalised command echoed back to frontend
    linear_cmd: float = 0.0
    angular_cmd: float = 0.0

    latest_scan_points: int = 0
    last_update: float = field(default_factory=time.time)
    telemetry_age_s: float = 0.0
    last_error: Optional[str] = None

    def to_dict(self) -> dict:
        return {
            "mode": self.mode,
            "armed": self.armed,
            "connected": self.connected,
            "lidar_connected": self.lidar_connected,
            "mapping": self.mapping,
            "obstacle_stop": self.obstacle_stop,
            "pose": self.pose.to_dict(),
            "nav": self.nav.to_dict(),
            "left_ticks": self.left_ticks,
            "right_ticks": self.right_ticks,
            "left_rpm": round(self.left_rpm, 2),
            "right_rpm": round(self.right_rpm, 2),
            "left_pwm": self.left_pwm,
            "right_pwm": self.right_pwm,
            "battery_v": round(self.battery_v, 2) if self.battery_v is not None else None,
            "linear_cmd": round(self.linear_cmd, 3),
            "angular_cmd": round(self.angular_cmd, 3),
            "latest_scan_points": self.latest_scan_points,
            "last_update": self.last_update,
            "telemetry_age_s": round(self.telemetry_age_s, 2),
            "last_error": self.last_error,
        }
