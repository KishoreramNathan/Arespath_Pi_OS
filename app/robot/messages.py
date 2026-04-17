"""Lightweight ROS-like message types for the navigation stack.

The runtime remains pure Python and thread-based, but these dataclasses give
the subsystems a stable contract similar to ROS messages.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple


ScanPoint = Tuple[float, float]
WorldPoint = Tuple[float, float]


@dataclass(frozen=True)
class RangeScanFrame:
    stamp: float
    points: List[ScanPoint] = field(default_factory=list)
    source: str = "lidar"
    sequence: int = 0


@dataclass(frozen=True)
class FusedSensorFrame:
    stamp: float
    pose: Tuple[float, float, float]
    lidar_points: List[ScanPoint] = field(default_factory=list)
    radar_points: List[ScanPoint] = field(default_factory=list)
    obstacle_distance_m: Optional[float] = None
    obstacle_in_path: bool = False


@dataclass(frozen=True)
class PlannerRequest:
    stamp: float
    start: WorldPoint
    goal: WorldPoint
    pose: Tuple[float, float, float]
    occupancy: object
    scan: List[ScanPoint] = field(default_factory=list)


@dataclass(frozen=True)
class PlannerResult:
    stamp: float
    goal: WorldPoint
    path: List[WorldPoint] = field(default_factory=list)
    status: str = "idle"
