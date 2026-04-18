"""ROS-like 2-D frame transform helpers.

This module centralizes robot-base, LiDAR, and world-frame conversions so
mapping, localization, rendering, and planning all use identical geometry.
"""

import math
from typing import Tuple

from app import config


def _pose_components(pose) -> Tuple[float, float, float]:
    if hasattr(pose, "x") and hasattr(pose, "y") and hasattr(pose, "theta"):
        return float(pose.x), float(pose.y), float(pose.theta)
    return float(pose[0]), float(pose[1]), float(pose[2])


def transform_robot_to_world(pose, local_x: float, local_y: float) -> Tuple[float, float]:
    x, y, theta = _pose_components(pose)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    wx = x + cos_t * local_x - sin_t * local_y
    wy = y + sin_t * local_x + cos_t * local_y
    return wx, wy


def lidar_origin_world(pose) -> Tuple[float, float]:
    return transform_robot_to_world(
        pose,
        config.LIDAR_OFFSET_X_M,
        config.LIDAR_OFFSET_Y_M,
    )


def scan_point_to_world(pose, angle: float, distance: float) -> Tuple[float, float]:
    lidar_x, lidar_y = lidar_origin_world(pose)
    _, _, theta = _pose_components(pose)
    global_angle = theta + angle
    wx = lidar_x + distance * math.cos(global_angle)
    wy = lidar_y + distance * math.sin(global_angle)
    return wx, wy
