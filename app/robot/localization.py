"""Dedicated localization node for odometry + LiDAR fusion.

This is the Python equivalent of a compact ROS localization pipeline:
- wheel odometry integration
- LiDAR scan-to-scan ICP correction
- scan-to-map anchoring against the occupancy grid
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from app import config
from app.robot.lidar import LidarLocalizer
from app.robot.state import Pose


@dataclass(frozen=True)
class LocalizationEstimate:
    pose: Pose
    source: str


class LocalizationNode:
    def __init__(self) -> None:
        self._scan_localizer = LidarLocalizer()
        self._last_ticks: Optional[Tuple[int, int]] = None
        self._last_stamp: Optional[float] = None
        self._last_source: str = "odom"

    @property
    def source(self) -> str:
        return self._last_source

    def reset(self) -> None:
        self._last_ticks = None
        self._last_stamp = None
        self._last_source = "odom"
        self._scan_localizer.reset()

    # ── Backward-compatibility proxies ────────────────────────────────────────
    # control.py calls these directly on self.localizer.  They delegate to the
    # internal LidarLocalizer so the old call-sites keep working unchanged.

    def correct(self, pose: Pose, scan: List[Tuple[float, float]]) -> Pose:
        """ICP scan-to-scan correction proxy → LidarLocalizer.correct()."""
        return self._scan_localizer.correct(pose, scan)

    def anchor_to_map(self, pose: Pose, scan: List[Tuple[float, float]], occ_map) -> Pose:
        """Scan-to-map anchoring proxy → LidarLocalizer.anchor_to_map()."""
        return self._scan_localizer.anchor_to_map(pose, scan, occ_map)

    def update(
        self,
        pose: Pose,
        left_ticks: int,
        right_ticks: int,
        stamp: float,
        scan: List[Tuple[float, float]],
        occ_map,
    ) -> LocalizationEstimate:
        estimate = Pose(pose.x, pose.y, pose.theta)
        estimate = self._integrate_odometry(estimate, left_ticks, right_ticks, stamp)
        source = "odom"

        if config.LIDAR_LOCALIZATION_ENABLED and scan:
            estimate = self._scan_localizer.correct(estimate, scan)
            source = "odom+icp"
            if occ_map.has_observations():
                estimate = self._scan_localizer.anchor_to_map(estimate, scan, occ_map)
                source = "odom+icp+map"

        self._last_source = source
        return LocalizationEstimate(pose=estimate, source=source)

    def _integrate_odometry(
        self,
        pose: Pose,
        left_ticks: int,
        right_ticks: int,
        stamp: float,
    ) -> Pose:
        ticks = (left_ticks, right_ticks)
        if self._last_ticks is None:
            self._last_ticks = ticks
            self._last_stamp = stamp
            return pose

        dl = ticks[0] - self._last_ticks[0]
        dr = ticks[1] - self._last_ticks[1]
        self._last_ticks = ticks

        dt = stamp - (self._last_stamp or stamp)
        self._last_stamp = stamp
        if dt <= 0.0:
            return pose

        circ = 2.0 * math.pi * config.WHEEL_RADIUS_M
        left_dist = circ * (dl / config.TICKS_PER_WHEEL_REV)
        right_dist = circ * (dr / config.TICKS_PER_WHEEL_REV)
        ds = 0.5 * (left_dist + right_dist)
        dtheta = (right_dist - left_dist) / config.WHEEL_BASE_M
        mid = pose.theta + 0.5 * dtheta

        return Pose(
            x=pose.x + ds * math.cos(mid),
            y=pose.y + ds * math.sin(mid),
            theta=((pose.theta + dtheta + math.pi) % (2.0 * math.pi)) - math.pi,
        )
