"""Nav2-style waypoint follower state machine."""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple


Waypoint = Tuple[float, float]


@dataclass
class WaypointMission:
    waypoints: List[Waypoint] = field(default_factory=list)
    current_index: int = 0

    def current_goal(self) -> Optional[Waypoint]:
        if 0 <= self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return None


class WaypointFollower:
    def __init__(self) -> None:
        self._mission = WaypointMission()

    def set_waypoints(self, waypoints: List[Waypoint]) -> Optional[Waypoint]:
        self._mission = WaypointMission(list(waypoints), 0)
        return self._mission.current_goal()

    def clear(self) -> None:
        self._mission = WaypointMission()

    def current_goal(self) -> Optional[Waypoint]:
        return self._mission.current_goal()

    def current_index(self) -> int:
        return self._mission.current_index

    def waypoints(self) -> List[Waypoint]:
        return list(self._mission.waypoints)

    def advance(self) -> Optional[Waypoint]:
        if not self._mission.waypoints:
            return None
        next_index = self._mission.current_index + 1
        if next_index >= len(self._mission.waypoints):
            self._mission.current_index = len(self._mission.waypoints)
            return None
        self._mission.current_index = next_index
        return self._mission.current_goal()

    def progress(self) -> Tuple[int, int]:
        total = len(self._mission.waypoints)
        current = min(self._mission.current_index + 1, total) if total else 0
        return current, total
