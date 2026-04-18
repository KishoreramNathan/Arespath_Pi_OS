"""Authoritative telemetry + LiDAR localization for mission mode.

This module is the single pose-estimation path used by the mission map,
mapping, and planning stack.  It exists to keep the browser mission map in the
same frame as the LiDAR radar view and the planner.

Inputs
------
- wheel telemetry (encoder ticks + timestamp)
- LiDAR scan
- occupancy map for scan-to-map anchoring

Output
------
- a fused Pose estimate in the world/map frame
- a short source string describing which stages were applied

The key design goal is to avoid double-integrating odometry.  Earlier revisions
updated ``state.pose`` from wheel odometry and then fed that already-updated
pose into a second localization stage, which could drift and rotate the rover
icon incorrectly on the mission map.  This class owns the fused pose estimate so
mapping, planning, and UI all consume the same authoritative transform.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from app import config
from app.robot.lidar import LidarLocalizer
from app.robot.runtime_cfg import rtcfg
from app.robot.state import Pose, wrap_angle


@dataclass(frozen=True)
class LocalizationEstimate:
    pose: Pose
    source: str
    stamp: float


class TelemetryLidarLocalizer:
    """Fuse encoder telemetry with LiDAR scan matching into one pose stream."""

    def __init__(self) -> None:
        self._scan_localizer = LidarLocalizer()
        self._pose_estimate: Optional[Pose] = None
        self._last_ticks: Optional[Tuple[int, int]] = None
        self._last_stamp: Optional[float] = None
        self._last_source: str = "odom"
        self._last_estimate: Optional[LocalizationEstimate] = None

    @property
    def source(self) -> str:
        return self._last_source

    def reset(self, pose: Optional[Pose] = None) -> None:
        self._pose_estimate = None if pose is None else Pose(pose.x, pose.y, pose.theta)
        self._last_ticks = None
        self._last_stamp = None
        self._last_source = "odom"
        self._last_estimate = None
        self._scan_localizer.reset()

    def get_topic_msg(self) -> dict:
        est = self._last_estimate
        if est is None:
            return {"source": self._last_source, "active": True}
        return {
            "source": est.source,
            "active": True,
            "stamp": round(est.stamp, 3),
            "pose": est.pose.to_dict(),
        }

    def update(
        self,
        fallback_pose: Pose,
        left_ticks: int,
        right_ticks: int,
        stamp: float,
        scan: List[Tuple[float, float]],
        occ_map,
    ) -> LocalizationEstimate:
        base = self._pose_estimate or Pose(fallback_pose.x, fallback_pose.y, fallback_pose.theta)
        estimate = self._integrate_odometry(base, left_ticks, right_ticks, stamp)
        source = "odom"

        if config.LIDAR_LOCALIZATION_ENABLED and scan:
            corrected = self._scan_localizer.correct(estimate, scan)
            estimate = Pose(corrected.x, corrected.y, corrected.theta)
            source = "telemetry+lidar"

            if occ_map is not None and occ_map.has_observations():
                anchored = occ_map.localize_scan_match(
                    estimate,
                    scan,
                    xy_window_m=max(0.10, 0.55 * config.MAP_RESOLUTION_M),
                    theta_window_rad=0.30,
                    xy_step_m=max(0.02, 0.25 * config.MAP_RESOLUTION_M),
                    theta_step_rad=0.06,
                    stride=4,
                )
                if anchored is not None:
                    estimate = self._blend_pose(
                        estimate,
                        anchored,
                        weight=max(0.35, min(0.70, rtcfg.get("lidar_icp_weight"))),
                    )
                    source = "telemetry+lidar+map"

        self._pose_estimate = Pose(estimate.x, estimate.y, estimate.theta)
        self._last_source = source
        self._last_estimate = LocalizationEstimate(
            pose=self._pose_estimate,
            source=source,
            stamp=stamp if stamp > 0 else time.time(),
        )
        return self._last_estimate

    def _integrate_odometry(
        self,
        pose: Pose,
        left_ticks: int,
        right_ticks: int,
        stamp: float,
    ) -> Pose:
        ticks = (int(left_ticks), int(right_ticks))
        if self._last_ticks is None:
            self._last_ticks = ticks
            self._last_stamp = stamp
            return Pose(pose.x, pose.y, pose.theta)

        prev_stamp = self._last_stamp if self._last_stamp is not None else stamp
        dt = stamp - prev_stamp
        self._last_stamp = stamp
        dl = ticks[0] - self._last_ticks[0]
        dr = ticks[1] - self._last_ticks[1]
        self._last_ticks = ticks

        if dt <= 0.0:
            return Pose(pose.x, pose.y, pose.theta)

        circ = 2.0 * math.pi * config.WHEEL_RADIUS_M
        left_dist = circ * (dl / config.TICKS_PER_WHEEL_REV)
        right_dist = circ * (dr / config.TICKS_PER_WHEEL_REV)
        ds = 0.5 * (left_dist + right_dist)
        dtheta = (right_dist - left_dist) / config.WHEEL_BASE_M
        mid_theta = pose.theta + 0.5 * dtheta

        return Pose(
            x=pose.x + ds * math.cos(mid_theta),
            y=pose.y + ds * math.sin(mid_theta),
            theta=wrap_angle(pose.theta + dtheta),
        )

    @staticmethod
    def _blend_pose(current: Pose, anchored: Pose, weight: float) -> Pose:
        return Pose(
            x=current.x + (anchored.x - current.x) * weight,
            y=current.y + (anchored.y - current.y) * weight,
            theta=wrap_angle(current.theta + wrap_angle(anchored.theta - current.theta) * weight),
        )
