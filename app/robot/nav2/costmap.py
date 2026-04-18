"""Costmap snapshot helper inspired by Nav2 costmap servers."""

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from app.robot.mapping import OccupancyGridMap
from app.robot.state import Pose


@dataclass(frozen=True)
class CostmapSnapshot:
    occupancy: np.ndarray
    pose: Pose
    scan: List[Tuple[float, float]]
    origin: Tuple[float, float]
    resolution: float

    @classmethod
    def from_mapping(
        cls,
        occ_map: OccupancyGridMap,
        pose: Pose,
        scan: List[Tuple[float, float]],
    ) -> "CostmapSnapshot":
        occupancy = occ_map.occupancy_for_planner()
        if scan:
            occupancy = occ_map.overlay_scan_on_occupancy(pose, scan, occupancy=occupancy)
        return cls(
            occupancy=occupancy,
            pose=pose,
            scan=list(scan),
            origin=(occ_map.origin_x, occ_map.origin_y),
            resolution=occ_map.resolution,
        )
