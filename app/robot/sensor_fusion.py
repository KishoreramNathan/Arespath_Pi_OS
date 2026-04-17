"""Sensor processing module for ROS-like navigation.

The current rover already has LiDAR integrated. Radar is treated as optional so
the rest of the stack can use the same fused interface now and accept a real
radar feed later without changing planner/controller code.
"""

import math
from typing import List, Optional, Tuple

from app.robot.messages import FusedSensorFrame, RangeScanFrame, ScanPoint


class SensorFusion:
    """Fuse range sensors into a timestamped frame for planning/control."""

    def __init__(self, stop_distance_m: float, cone_deg: float) -> None:
        self.stop_distance_m = float(stop_distance_m)
        self.cone_rad = math.radians(float(cone_deg))

    def build_frame(
        self,
        lidar_frame: Optional[RangeScanFrame],
        pose: Tuple[float, float, float],
        radar_points: Optional[List[ScanPoint]] = None,
    ) -> FusedSensorFrame:
        lidar_points = list(lidar_frame.points) if lidar_frame else []
        radar_points = list(radar_points or [])

        min_dist = None
        for angle, dist in lidar_points + radar_points:
            if dist <= 0.0:
                continue
            wrapped = math.atan2(math.sin(angle), math.cos(angle))
            if abs(wrapped) > self.cone_rad:
                continue
            if min_dist is None or dist < min_dist:
                min_dist = dist

        stamp = lidar_frame.stamp if lidar_frame else 0.0
        return FusedSensorFrame(
            stamp=stamp,
            pose=pose,
            lidar_points=lidar_points,
            radar_points=radar_points,
            obstacle_distance_m=min_dist,
            obstacle_in_path=min_dist is not None and min_dist < self.stop_distance_m,
        )
