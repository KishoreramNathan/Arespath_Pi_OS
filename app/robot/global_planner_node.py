"""Global Planner Node — async JPS path planning with YAML-profile support.

Mirrors the ROS2 ``nav2_planner`` / ``planner_server`` lifecycle node.

Architecture
────────────
  • Runs in its own daemon thread (``planner-worker``), decoupled from the
    50 Hz control loop.
  • Accepts planning requests via a ``LatestQueue`` — only the most recent
    request is ever processed, matching Nav2's behaviour.
  • Publishes the smoothed path back through an output queue that the
    control loop polls without blocking.
  • All tunable parameters are read live from ``rtcfg`` on each plan so that
    YAML profile changes take effect immediately.

Thread safety
─────────────
  Only ``request_plan()`` and ``get_latest_path()`` are called from the
  control thread; all heavy planning work runs on the worker thread.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from app import config
from app.robot.ipc import LatestQueue
from app.robot.messages import PlannerRequest
from app.robot.nav2.costmap import CostmapSnapshot
from app.robot.nav2.planner_node import PlannerNode
from app.robot.planner_adv import DynamicPlanner
from app.robot.runtime_cfg import rtcfg
from app.robot.state import Pose
from app.robot.trajectory import PathSmoother, SmoothedPath, TrajectoryPoint, TrajectoryTracker

log = logging.getLogger(__name__)


@dataclass
class PlanResult:
    goal:      Tuple[float, float]
    path:      List[Tuple[float, float]]         # (x, y) waypoints in world coords
    trajectory: List[TrajectoryPoint]            # velocity-profiled points
    smoothed:  SmoothedPath
    stamp:     float


class GlobalPlannerNode:
    """Async JPS global planner — Nav2 planner_server equivalent.

    Usage::

        planner = GlobalPlannerNode()
        planner.start()

        # from control loop:
        planner.request_plan(goal, start, pose, occupancy, scan)
        result = planner.get_latest_result()  # non-blocking
        if result:
            trajectory = result.trajectory

        planner.stop()
    """

    def __init__(self) -> None:
        # Core planning objects
        self._planner = DynamicPlanner(
            grid_size  = config.MAP_SIZE_CELLS,
            resolution = config.MAP_RESOLUTION_M,
        )
        self._path_smoother = PathSmoother(
            resolution      = rtcfg.get("path_smoothing_resolution"),
            max_velocity    = rtcfg.get("max_linear_mps"),
            max_acceleration= rtcfg.get("nav_max_acceleration"),
            max_curvature   = rtcfg.get("max_path_curvature"),
        )
        self._nav2_planner = PlannerNode()

        # Thread-safe queues
        self._request_queue: LatestQueue[PlannerRequest] = LatestQueue(maxsize=1)
        self._result_queue:  LatestQueue[PlanResult]     = LatestQueue(maxsize=1)

        self._thread:   Optional[threading.Thread] = None
        self._running:  bool = False

        # Diagnostics
        self._last_plan_duration_s: float = 0.0
        self._plan_count: int = 0

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._worker_loop, daemon=True, name="planner-worker"
        )
        self._thread.start()
        log.info("GlobalPlannerNode: started")

    def stop(self) -> None:
        self._running = False
        # unblock the worker
        try:
            self._request_queue.put_latest(None)  # type: ignore[arg-type]
        except Exception:
            pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        log.info("GlobalPlannerNode: stopped")

    # ── Public API (called from control thread) ───────────────────────────────

    def request_plan(
        self,
        goal:       Tuple[float, float],
        start:      Tuple[float, float],
        pose:       Pose,
        occupancy,                            # 2-D numpy array
        scan:       List[Tuple[float, float]],
        stamp:      Optional[float] = None,
    ) -> None:
        """Submit an async planning request (non-blocking, latest-only)."""
        self._request_queue.put_latest(
            PlannerRequest(
                stamp     = stamp if stamp is not None else time.time(),
                start     = start,
                goal      = goal,
                pose      = (pose.x, pose.y, pose.theta),
                occupancy = occupancy,
                scan      = scan,
            )
        )

    def get_latest_result(self) -> Optional[PlanResult]:
        """Return the most recent completed plan (non-blocking)."""
        try:
            return self._result_queue.get(timeout=0.0)
        except (queue.Empty, Exception):
            return None

    def get_diagnostics(self) -> dict:
        return {
            "plan_count":          self._plan_count,
            "last_plan_duration_s": round(self._last_plan_duration_s, 4),
        }

    # ── Worker thread ─────────────────────────────────────────────────────────

    def _worker_loop(self) -> None:
        log.info("GlobalPlannerNode worker started")
        while self._running:
            try:
                request = self._request_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if request is None:
                break

            try:
                self._process_request(request)
            except Exception as exc:
                log.exception("GlobalPlannerNode planning error: %s", exc)

        log.info("GlobalPlannerNode worker stopped")

    def _process_request(self, req: PlannerRequest) -> None:
        t0 = time.perf_counter()

        # Reload tunables on each plan — profile changes take effect immediately
        self._path_smoother.resolution       = rtcfg.get("path_smoothing_resolution")
        self._path_smoother.max_velocity     = rtcfg.get("max_linear_mps")
        self._path_smoother.max_acceleration = rtcfg.get("nav_max_acceleration")
        self._path_smoother.max_curvature    = rtcfg.get("max_path_curvature")

        self._planner.update_static_map(req.occupancy)
        self._planner.update_dynamic_obstacles(
            pose        = req.pose,
            scan        = req.scan,
            grid_origin = (config.MAP_ORIGIN_X_M, config.MAP_ORIGIN_Y_M),
        )

        raw_path = self._planner.plan(
            start       = req.start,
            goal        = req.goal,
            grid_origin = (config.MAP_ORIGIN_X_M, config.MAP_ORIGIN_Y_M),
            use_jps     = True,
        )

        dt = time.perf_counter() - t0
        self._last_plan_duration_s = dt
        self._plan_count += 1

        if not raw_path or len(raw_path) < 2:
            log.debug(
                "GlobalPlannerNode: no path found goal=%s (%.1f ms)",
                req.goal, dt * 1000,
            )
            self._result_queue.put_latest(
                PlanResult(
                    goal       = req.goal,
                    path       = [],
                    trajectory = [],
                    smoothed   = SmoothedPath(),
                    stamp      = time.time(),
                )
            )
            return

        smoothed  = self._path_smoother.smooth(raw_path)
        # Share trajectory cache with TrajectoryTracker (legacy hook)
        TrajectoryTracker._trajectory_cache = smoothed.points

        path_wpts = [(pt.x, pt.y) for pt in smoothed.points]
        log.debug(
            "GlobalPlannerNode: path found %d pts goal=%s (%.1f ms)",
            len(path_wpts), req.goal, dt * 1000,
        )

        self._result_queue.put_latest(
            PlanResult(
                goal       = req.goal,
                path       = path_wpts,
                trajectory = smoothed.points,
                smoothed   = smoothed,
                stamp      = time.time(),
            )
        )
