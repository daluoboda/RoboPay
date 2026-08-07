from __future__ import annotations

from dobot_cra_sim.contracts import INSPECTION_SKILL, ROBOT_ID, STOP_SKILL, validate_action
from dobot_cra_sim.runtime import run_mujoco_episode


def test_real_mujoco_measures_all_three_tags() -> None:
    result = run_mujoco_episode(validate_action(ROBOT_ID, INSPECTION_SKILL, INSPECTION_SKILL, {}))
    assert result["success"] is True
    assert result["completion_reason"] == "all_three_tags_observed"
    assert {entry["target_id"] for entry in result["observed_tags"]} == {"amber_tag", "cyan_tag", "violet_tag"}
    assert all(entry["measured_error_m"] <= 0.045 for entry in result["observed_tags"])
    assert result["finite_state"] is True
    assert result["unexpected_contact_observed"] is False


def test_real_mujoco_stop_is_not_inspection() -> None:
    result = run_mujoco_episode(validate_action(ROBOT_ID, STOP_SKILL, STOP_SKILL, {}))
    assert result["success"] is True
    assert result["safe_stop_applied"] is True
    assert result["completion_reason"] == "safe_stopped"
    assert result["observed_tags"] == []
