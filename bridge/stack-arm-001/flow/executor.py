"""Skill execution interface + executors for stack-arm-001.

SkillExecutor is the seam the relay/bridge depends on. The physics backend is
imported lazily so a missing optional engine can never break the payment path.
"""

from __future__ import annotations


class SkillResult:
    def __init__(self, success: bool, message: str, metrics: dict | None = None):
        self.success = success
        self.message = message
        self.metrics = metrics or {}

    def to_dict(self) -> dict:
        return {
            "success": self.success,
            "message": self.message,
            "metrics": self.metrics,
        }


class SkillExecutor:
    def execute(self, skill_id: str, params: dict) -> SkillResult:
        raise NotImplementedError


class MuJoCoExecutor(SkillExecutor):
    """Default Tier 1 executor: physics-backed ``stack_box`` on stack-arm-001."""

    def __init__(self, engine: str = "mujoco"):
        self.engine = engine
        from simulator import MuJoCoSimulator
        self.sim = MuJoCoSimulator()
        self.supported = {"stack_box"}

    def execute(self, skill_id: str, params: dict) -> SkillResult:
        if skill_id not in self.supported:
            return SkillResult(False, f"unsupported_skill:{skill_id}")
        res = self.sim.stack_box(params or {})
        return SkillResult(res.success, res.reason, res.metrics)
