from __future__ import annotations

import pytest

from dobot_cra_sim.contracts import ContractError, INSPECTION_SKILL, ROBOT_ID, STOP_SKILL, validate_action


def test_only_registered_skill_and_exact_robot_are_accepted() -> None:
    assert validate_action(ROBOT_ID, INSPECTION_SKILL, INSPECTION_SKILL, {}).skill_id == INSPECTION_SKILL
    assert validate_action(ROBOT_ID, STOP_SKILL, STOP_SKILL, {}).skill_id == STOP_SKILL
    for action, skill, params in [
        ("", "", {}),
        ("unknown", "unknown", {}),
        (INSPECTION_SKILL, "unknown", {}),
        (INSPECTION_SKILL, INSPECTION_SKILL, {"duration": 100}),
        (INSPECTION_SKILL, INSPECTION_SKILL, []),
    ]:
        with pytest.raises(ContractError):
            validate_action(ROBOT_ID, action, skill, params)
    with pytest.raises(ContractError):
        validate_action("some-other-robot", INSPECTION_SKILL, INSPECTION_SKILL, {})
