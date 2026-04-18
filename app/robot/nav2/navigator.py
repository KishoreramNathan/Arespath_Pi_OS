"""Nav2-like navigator coordinator for missions and recoveries."""

from typing import List, Optional, Tuple

from app.robot.nav2.behavior_tree import RecoveryDecision, RecoveryManager
from app.robot.nav2.lifecycle import LifecycleNode
from app.robot.nav2.waypoint_follower import WaypointFollower


Waypoint = Tuple[float, float]


class NavigatorServer:
    def __init__(self) -> None:
        self.lifecycle = LifecycleNode("bt_navigator")
        self._waypoints = WaypointFollower()
        self._recovery = RecoveryManager()

    def activate(self) -> None:
        self.lifecycle.activate()

    def shutdown(self) -> None:
        self.lifecycle.shutdown()

    def set_waypoints(self, waypoints: List[Waypoint]) -> Optional[Waypoint]:
        return self._waypoints.set_waypoints(waypoints)

    def clear(self) -> None:
        self._waypoints.clear()

    def advance_waypoint(self) -> Optional[Waypoint]:
        return self._waypoints.advance()

    def current_goal(self) -> Optional[Waypoint]:
        return self._waypoints.current_goal()

    def mission_waypoints(self) -> List[Waypoint]:
        return self._waypoints.waypoints()

    def mission_index(self) -> int:
        return self._waypoints.current_index()

    def mission_progress(self) -> Tuple[int, int]:
        return self._waypoints.progress()

    def recovery_decision(
        self,
        obstacle_active: bool,
        blocked_elapsed_s: float,
        wait_before_replan_s: float,
        reverse_active: bool,
    ) -> RecoveryDecision:
        return self._recovery.decide(
            obstacle_active=obstacle_active,
            blocked_elapsed_s=blocked_elapsed_s,
            wait_before_replan_s=wait_before_replan_s,
            reverse_active=reverse_active,
        )
