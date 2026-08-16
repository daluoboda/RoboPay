"""Fail-closed contract for the priced CR3 three-tag inspection skill."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .course import MAX_DURATION_SECONDS


PROFILE_ID = "dobot.cra.cr3-mujoco-webots-inspection.v1"
ROBOT_ID = "dobot-cr3-sim-01"
INSPECTION_SKILL = "inspect_three_tags"
STOP_SKILL = "stop"
ALLOWED_SKILLS = frozenset({INSPECTION_SKILL, STOP_SKILL})


@dataclass(frozen=True)
class InspectionRequest:
    skill_id: str
    max_duration_sec: float = MAX_DURATION_SECONDS


class ContractError(ValueError):
    """Raised before a Tunnel-verified action can reach the simulator."""


def validate_action(robot_id: Any, action: Any, skill_id: Any, params: Any) -> InspectionRequest:
    """Allow only the registered, bounded action tuple for this CR3 profile."""

    if robot_id != ROBOT_ID:
        raise ContractError("unknown robot")
    if not isinstance(action, str) or not isinstance(skill_id, str) or action != skill_id:
        raise ContractError("action must exactly match the registered skill")
    if action not in ALLOWED_SKILLS:
        raise ContractError("unregistered skill")
    if not isinstance(params, dict):
        raise ContractError("params must be an object")
    if params:
        raise ContractError("this fixed inspection skill does not accept caller-controlled motion parameters")
    return InspectionRequest(skill_id=action)
