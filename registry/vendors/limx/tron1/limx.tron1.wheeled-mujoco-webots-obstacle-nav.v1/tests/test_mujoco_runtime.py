from limx_tron1_sim.contracts import NAVIGATION_SKILL, STOP_SKILL, NavigationRequest
from limx_tron1_sim.runtime import run_mujoco_episode


def test_official_model_and_policy_complete_measured_course() -> None:
    result = run_mujoco_episode(NavigationRequest(NAVIGATION_SKILL))
    assert result["success"] is True
    assert result["model_variant"] == "WF_TRON1A"
    assert result["low_level_controller"] == "limx-isaacgym-onnx-policy"
    assert result["waypoints_completed"] == result["waypoints_total"] == 7
    assert len(result["detected_obstacles"]) == 3
    assert result["collision"] is False
    assert result["minimum_clearance_m"] > 0.05


def test_stop_does_not_start_navigation() -> None:
    result = run_mujoco_episode(NavigationRequest(STOP_SKILL))
    assert result == {
        "success": True,
        "skill": STOP_SKILL,
        "message": "safe stop acknowledged; zero velocity command retained",
        "simulator": "mujoco",
        "stopped": True,
    }
