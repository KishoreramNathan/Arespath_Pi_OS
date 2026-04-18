"""Global planner node inspired by Nav2 planner_server."""

import time
from dataclasses import dataclass, field

from app import config
from app.robot.messages import PlannerRequest, PlannerResult
from app.robot.planner_adv import DynamicPlanner
from app.robot.runtime_cfg import rtcfg
from app.robot.trajectory import PathSmoother, SmoothedPath, TrajectoryTracker


@dataclass
class PlanningArtifacts:
    result: PlannerResult
    smoothed: SmoothedPath = field(default_factory=SmoothedPath)


class PlannerNode:
    def __init__(self) -> None:
        self._planner = DynamicPlanner(
            grid_size=config.MAP_SIZE_CELLS,
            resolution=config.MAP_RESOLUTION_M,
            inflation_radius=config.PLANNER_INFLATION_CELLS,
        )
        self._smoother = PathSmoother(
            resolution=rtcfg.get("path_smoothing_resolution"),
            max_velocity=rtcfg.get("max_linear_mps"),
            max_acceleration=rtcfg.get("nav_max_acceleration"),
            max_curvature=rtcfg.get("max_path_curvature"),
            lookahead=rtcfg.get("trajectory_lookahead_m"),
        )

    def plan(self, request: PlannerRequest) -> PlanningArtifacts:
        self._sync_tunables()
        self._planner.update_static_map(request.occupancy)
        self._planner.update_dynamic_obstacles(
            pose=request.pose,
            scan=request.scan,
            grid_origin=(config.MAP_ORIGIN_X_M, config.MAP_ORIGIN_Y_M),
        )
        raw_path = self._planner.plan(
            start=request.start,
            goal=request.goal,
            grid_origin=(config.MAP_ORIGIN_X_M, config.MAP_ORIGIN_Y_M),
            use_jps=True,
        )

        if not raw_path or len(raw_path) < 2:
            return PlanningArtifacts(
                result=PlannerResult(
                    stamp=time.time(),
                    goal=request.goal,
                    path=[],
                    status="no_path",
                )
            )

        smoothed = self._smoother.smooth(raw_path)
        TrajectoryTracker._trajectory_cache = smoothed.points
        path = [(pt.x, pt.y) for pt in smoothed.points]
        return PlanningArtifacts(
            result=PlannerResult(
                stamp=time.time(),
                goal=request.goal,
                path=path,
                status="ok",
            ),
            smoothed=smoothed,
        )

    def _sync_tunables(self) -> None:
        self._smoother.resolution = rtcfg.get("path_smoothing_resolution")
        self._smoother.max_velocity = rtcfg.get("max_linear_mps")
        self._smoother.max_acceleration = rtcfg.get("nav_max_acceleration")
        self._smoother.max_curvature = rtcfg.get("max_path_curvature")
        self._smoother.lookahead = rtcfg.get("trajectory_lookahead_m")
