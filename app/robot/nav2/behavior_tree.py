"""Small behavior-tree-style recovery decisions."""

from dataclasses import dataclass


@dataclass(frozen=True)
class RecoveryDecision:
    status: str
    should_wait: bool = False
    should_reverse: bool = False
    should_replan: bool = False


class RecoveryManager:
    def decide(
        self,
        obstacle_active: bool,
        blocked_elapsed_s: float,
        wait_before_replan_s: float,
        reverse_active: bool,
    ) -> RecoveryDecision:
        if not obstacle_active:
            return RecoveryDecision(status="clear")
        if blocked_elapsed_s < wait_before_replan_s:
            return RecoveryDecision(status="waiting", should_wait=True)
        if reverse_active:
            return RecoveryDecision(status="reversing", should_reverse=True)
        return RecoveryDecision(status="replan", should_replan=True)
