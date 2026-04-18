"""Minimal ROS 2 lifecycle node model."""

from enum import Enum


class LifecycleState(str, Enum):
    UNCONFIGURED = "unconfigured"
    INACTIVE = "inactive"
    ACTIVE = "active"
    FINALIZED = "finalized"


class LifecycleNode:
    def __init__(self, name: str) -> None:
        self.name = name
        self.state = LifecycleState.UNCONFIGURED

    def configure(self) -> LifecycleState:
        self.state = LifecycleState.INACTIVE
        return self.state

    def activate(self) -> LifecycleState:
        if self.state == LifecycleState.UNCONFIGURED:
            self.configure()
        self.state = LifecycleState.ACTIVE
        return self.state

    def deactivate(self) -> LifecycleState:
        if self.state != LifecycleState.FINALIZED:
            self.state = LifecycleState.INACTIVE
        return self.state

    def shutdown(self) -> LifecycleState:
        self.state = LifecycleState.FINALIZED
        return self.state
