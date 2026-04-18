"""Dedicated Localization Node — ROS-like lifecycle + pose publication.

Architecture
────────────
This module is the single source of truth for robot pose estimation.
It mirrors the ROS2 ``amcl`` / ``robot_localization`` node pattern:

  configure()  → allocate resources, load params from rtcfg
  activate()   → start accepting odometry + scan callbacks
  spin_once()  → fuse inputs and publish a LocalizationEstimate
  deactivate() → freeze pose, stop publishing
  cleanup()    → release resources

Inputs (written by the control loop before calling spin_once):
  set_odometry(left_ticks, right_ticks, stamp)
  set_scan(scan)
  set_map(occ_map)

Output:
  estimate → LocalizationEstimate(pose, source, stamp, covariance_diag)

The node also publishes a lightweight ``/localization`` topic-style dict
via its ``get_topic_msg()`` method, consumed by the WebSocket status push.

Backward compatibility
──────────────────────
The older ``LocalizationNode`` in ``localization.py`` is still present and
now exposes ``correct()`` / ``anchor_to_map()`` proxies so that existing
call-sites in ``control.py`` continue to work.  New code should use this
module exclusively.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional, Tuple

from app import config
from app.robot.lidar import LidarLocalizer
from app.robot.runtime_cfg import rtcfg
from app.robot.state import Pose

log = logging.getLogger(__name__)


# ── Lifecycle states (mirrors ROS2 Managed Node) ──────────────────────────────

class NodeState(Enum):
    UNCONFIGURED = auto()
    INACTIVE     = auto()
    ACTIVE       = auto()
    FINALIZED    = auto()


# ── Output topic message ───────────────────────────────────────────────────────

@dataclass
class LocalizationEstimate:
    pose:             Pose
    source:           str        # "odom" | "odom+icp" | "odom+icp+map"
    stamp:            float      # time.time() when estimated
    covariance_diag:  Tuple[float, float, float] = (0.01, 0.01, 0.005)


# ── The node ──────────────────────────────────────────────────────────────────

class LocalizationLifecycleNode:
    """ROS-style managed localization node.

    Usage inside RobotRuntime::

        self.loc_node = LocalizationLifecycleNode()
        self.loc_node.configure()
        self.loc_node.activate()

        # in _ctrl_loop:
        self.loc_node.set_odometry(left_ticks, right_ticks, stamp)
        self.loc_node.set_scan(scan)
        self.loc_node.set_map(self.map)
        estimate = self.loc_node.spin_once(self.state.pose)
        if estimate:
            self.state.pose = estimate.pose
    """

    def __init__(self) -> None:
        self._state = NodeState.UNCONFIGURED
        self._lock  = threading.Lock()

        # Sub-component — created in configure()
        self._scan_localizer: Optional[LidarLocalizer] = None

        # Pending sensor data
        self._left_ticks:  int   = 0
        self._right_ticks: int   = 0
        self._stamp:       float = 0.0
        self._scan:        List[Tuple[float, float]] = []
        self._occ_map      = None

        # Odometry integration state
        self._last_ticks: Optional[Tuple[int, int]] = None
        self._last_stamp: Optional[float]           = None
        self._last_source: str                      = "odom"

        # Covariance model (grows with distance, shrinks with ICP)
        self._cov = [0.01, 0.01, 0.005]   # x, y, theta variance

        self._last_estimate: Optional[LocalizationEstimate] = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    @property
    def lifecycle_state(self) -> NodeState:
        return self._state

    def configure(self) -> bool:
        if self._state != NodeState.UNCONFIGURED:
            log.warning("LocalizationLifecycleNode: configure() called in state %s", self._state)
            return False
        self._scan_localizer = LidarLocalizer()
        self._state = NodeState.INACTIVE
        log.info("LocalizationLifecycleNode: configured")
        return True

    def activate(self) -> bool:
        if self._state != NodeState.INACTIVE:
            log.warning("LocalizationLifecycleNode: activate() called in state %s", self._state)
            return False
        self._state = NodeState.ACTIVE
        log.info("LocalizationLifecycleNode: active")
        return True

    def deactivate(self) -> bool:
        if self._state != NodeState.ACTIVE:
            return False
        self._state = NodeState.INACTIVE
        log.info("LocalizationLifecycleNode: deactivated")
        return True

    def cleanup(self) -> None:
        self._state = NodeState.FINALIZED
        self._scan_localizer = None
        log.info("LocalizationLifecycleNode: finalized")

    def reset_pose(self) -> None:
        """Reset odometry integration (call after set_pose or map load)."""
        with self._lock:
            self._last_ticks = None
            self._last_stamp = None
            self._last_source = "odom"
            self._cov = [0.01, 0.01, 0.005]
            if self._scan_localizer:
                self._scan_localizer.reset()

    # ── Sensor inputs ─────────────────────────────────────────────────────────

    def set_odometry(self, left_ticks: int, right_ticks: int, stamp: float) -> None:
        with self._lock:
            self._left_ticks  = left_ticks
            self._right_ticks = right_ticks
            self._stamp       = stamp

    def set_scan(self, scan: List[Tuple[float, float]]) -> None:
        with self._lock:
            self._scan = scan

    def set_map(self, occ_map) -> None:
        with self._lock:
            self._occ_map = occ_map

    # ── Main tick ─────────────────────────────────────────────────────────────

    def spin_once(self, current_pose: Pose) -> Optional[LocalizationEstimate]:
        """Fuse odometry + LiDAR and return an updated pose estimate.

        Returns None if the node is not active or no new data is available.
        """
        if self._state != NodeState.ACTIVE:
            return None

        with self._lock:
            left_ticks  = self._left_ticks
            right_ticks = self._right_ticks
            stamp       = self._stamp
            scan        = list(self._scan)
            occ_map     = self._occ_map

        # 1. Integrate wheel odometry
        pose = self._integrate_odometry(current_pose, left_ticks, right_ticks, stamp)
        source = "odom"

        # Update covariance: grows with motion
        ds = math.hypot(pose.x - current_pose.x, pose.y - current_pose.y)
        self._cov[0] = min(1.0, self._cov[0] + 0.001 * ds)
        self._cov[1] = min(1.0, self._cov[1] + 0.001 * ds)
        self._cov[2] = min(0.5, self._cov[2] + 0.0005 * abs(pose.theta - current_pose.theta))

        # 2. ICP scan correction
        if (
            config.LIDAR_LOCALIZATION_ENABLED
            and scan
            and self._scan_localizer is not None
        ):
            try:
                icp_weight = rtcfg.get("lidar_icp_weight")
                corrected  = self._scan_localizer.correct(pose, scan)
                # Blend based on icp_weight
                pose = Pose(
                    x     = pose.x     + icp_weight * (corrected.x     - pose.x),
                    y     = pose.y     + icp_weight * (corrected.y     - pose.y),
                    theta = _blend_angle(pose.theta, corrected.theta, icp_weight),
                )
                source = "odom+icp"
                # ICP reduces uncertainty
                shrink = max(0.3, 1.0 - icp_weight * 0.5)
                self._cov[0] *= shrink
                self._cov[1] *= shrink
                self._cov[2] *= shrink

                # 3. Map anchoring
                if occ_map is not None and occ_map.has_observations():
                    pose   = self._scan_localizer.anchor_to_map(pose, scan, occ_map)
                    source = "odom+icp+map"
                    self._cov[0] *= 0.8
                    self._cov[1] *= 0.8
                    self._cov[2] *= 0.8

            except Exception as exc:
                log.debug("LocalizationLifecycleNode ICP error: %s", exc)

        self._last_source = source

        estimate = LocalizationEstimate(
            pose=pose,
            source=source,
            stamp=time.time(),
            covariance_diag=(self._cov[0], self._cov[1], self._cov[2]),
        )
        self._last_estimate = estimate
        return estimate

    # ── Topic message for WebSocket ───────────────────────────────────────────

    def get_topic_msg(self) -> dict:
        """Return a lightweight dict for the /localization topic."""
        est = self._last_estimate
        if est is None:
            return {"source": "none", "active": self._state == NodeState.ACTIVE}
        return {
            "source":           est.source,
            "active":           self._state == NodeState.ACTIVE,
            "stamp":            est.stamp,
            "cov_x":            round(est.covariance_diag[0], 4),
            "cov_y":            round(est.covariance_diag[1], 4),
            "cov_theta":        round(est.covariance_diag[2], 4),
        }

    # ── Backward-compat proxies (used by older control.py call sites) ─────────

    def correct(self, pose: Pose, scan: List[Tuple[float, float]]) -> Pose:
        """Direct ICP correction proxy — delegates to LidarLocalizer."""
        if self._scan_localizer is None:
            return pose
        return self._scan_localizer.correct(pose, scan)

    def anchor_to_map(self, pose: Pose, scan: List[Tuple[float, float]], occ_map) -> Pose:
        """Map-anchoring proxy — delegates to LidarLocalizer."""
        if self._scan_localizer is None:
            return pose
        return self._scan_localizer.anchor_to_map(pose, scan, occ_map)

    # ── Private: odometry integration ─────────────────────────────────────────

    def _integrate_odometry(
        self,
        pose:        Pose,
        left_ticks:  int,
        right_ticks: int,
        stamp:       float,
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

        circ       = 2.0 * math.pi * config.WHEEL_RADIUS_M
        left_dist  = circ * (dl / config.TICKS_PER_WHEEL_REV)
        right_dist = circ * (dr / config.TICKS_PER_WHEEL_REV)
        ds         = 0.5 * (left_dist + right_dist)
        dtheta     = (right_dist - left_dist) / config.WHEEL_BASE_M
        mid        = pose.theta + 0.5 * dtheta

        return Pose(
            x     = pose.x + ds * math.cos(mid),
            y     = pose.y + ds * math.sin(mid),
            theta = ((pose.theta + dtheta + math.pi) % (2.0 * math.pi)) - math.pi,
        )


# ── Helper ────────────────────────────────────────────────────────────────────

def _blend_angle(a: float, b: float, t: float) -> float:
    """Blend two angles with weight t, handling wrap-around correctly."""
    diff = math.atan2(math.sin(b - a), math.cos(b - a))
    raw  = a + t * diff
    return ((raw + math.pi) % (2.0 * math.pi)) - math.pi
