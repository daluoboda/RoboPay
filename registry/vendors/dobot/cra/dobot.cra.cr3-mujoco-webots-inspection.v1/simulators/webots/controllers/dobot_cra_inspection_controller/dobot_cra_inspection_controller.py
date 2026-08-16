"""Webots execution adapter for the bounded, planned Dobot CR3 inspection task."""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

from controller import Supervisor


PROFILE_ROOT = Path(__file__).resolve().parents[4]
BRIDGE_ROOT = PROFILE_ROOT / "bridge"
if str(BRIDGE_ROOT) not in sys.path:
    sys.path.insert(0, str(BRIDGE_ROOT))

from dobot_cra_sim.assets import JOINT_NAMES
from dobot_cra_sim.contracts import INSPECTION_SKILL, STOP_SKILL
from dobot_cra_sim.course import COURSE_ID, MAX_DURATION_SECONDS, fingerprint
from dobot_cra_sim.kinematics import finite_joint_vector, tool_position
from dobot_cra_sim.task import InspectionTask


PLAN_INTERVAL_SECONDS = 0.12
TRACE_INTERVAL_SECONDS = 0.25
MIN_TOOL_HEIGHT_M = 0.15


def _paths() -> tuple[dict, Path]:
    config_path = Path(os.environ["DOBOT_CR3_WEBOTS_CONFIG_PATH"])
    result_path = Path(os.environ["DOBOT_CR3_WEBOTS_RESULT_PATH"])
    return json.loads(config_path.read_text(encoding="utf-8")), result_path


def main() -> int:
    parameters, result_path = _paths()
    course = parameters.get("course")
    if (
        parameters.get("skill_id") not in {INSPECTION_SKILL, STOP_SKILL}
        or not isinstance(course, dict)
        or parameters.get("course_hash") != fingerprint()
        or float(parameters.get("max_duration_sec", -1.0)) != MAX_DURATION_SECONDS
    ):
        raise RuntimeError("Webots CR3 controller received an invalid fixed inspection request")
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    self_node = robot.getSelf()
    self_node.enableContactPointsTracking(timestep, True)
    motors = [robot.getDevice(name) for name in JOINT_NAMES]
    sensors = [robot.getDevice(f"{name}_sensor") for name in JOINT_NAMES]
    if any(device is None for device in [*motors, *sensors]):
        raise RuntimeError("Generated Dobot CR3 Webots model is missing vendor joint motors or sensors")
    for motor in motors:
        motor.setVelocity(1.6)
    for sensor in sensors:
        sensor.enable(timestep)

    task = InspectionTask()
    desired = [0.0] * len(JOINT_NAMES)
    elapsed_ms = 0
    next_plan_ms = 0
    next_trace_ms = 0
    min_tool_height = float("inf")
    unexpected_contact = False
    trajectory: list[dict[str, object]] = []
    completion_reason: str | None = None
    while elapsed_ms < int(MAX_DURATION_SECONDS * 1000) and robot.step(timestep) != -1:
        elapsed = elapsed_ms / 1000.0
        measured_joints = [float(sensor.getValue()) for sensor in sensors]
        measured_tool = tool_position(measured_joints)
        min_tool_height = min(min_tool_height, measured_tool[2])
        unexpected_contact = unexpected_contact or bool(self_node.getContactPoints(includeDescendants=True))
        if elapsed_ms >= next_plan_ms:
            task.update(measured_tool, elapsed)
            if task.complete:
                completion_reason = "all_three_tags_observed"
                break
            if parameters["skill_id"] == STOP_SKILL:
                completion_reason = "safe_stopped"
                break
            desired = list(task.plan(measured_joints, measured_tool))
            next_plan_ms += int(PLAN_INTERVAL_SECONDS * 1000)
        for motor, target in zip(motors, desired):
            motor.setPosition(target)
        if elapsed_ms >= next_trace_ms:
            trajectory.append({"time_seconds": round(elapsed, 3), "tool_position_m": [round(value, 5) for value in measured_tool]})
            next_trace_ms += int(TRACE_INTERVAL_SECONDS * 1000)
        if not finite_joint_vector(measured_joints):
            completion_reason = "nonfinite_simulator_state"
            break
        if measured_tool[2] < MIN_TOOL_HEIGHT_M:
            completion_reason = "unsafe_tool_height"
            break
        elapsed_ms += timestep
    if completion_reason is None:
        completion_reason = "inspection_timeout"
    final_joints = [float(sensor.getValue()) for sensor in sensors]
    final_tool = tool_position(final_joints)
    success = completion_reason in {"all_three_tags_observed", "safe_stopped"}
    result = {
        "simulator_engine": "Webots",
        "robot_model": "Dobot CR3 vendor URDF converted to Webots R2025a",
        "task": parameters["skill_id"],
        "course_id": COURSE_ID,
        "course_hash": fingerprint(),
        "status": "success" if success else "failure",
        "success": success,
        "completion_reason": completion_reason,
        "safe_stop_applied": completion_reason == "safe_stopped",
        "sim_duration_seconds": round(elapsed_ms / 1000.0, 4),
        "observed_tags": task.observed,
        "remaining_tags": sorted(task.pending),
        "final_tool_position_m": [round(value, 5) for value in final_tool],
        "final_joint_positions_rad": [round(value, 5) for value in final_joints],
        "minimum_tool_height_m": round(min_tool_height, 5),
        "unexpected_contact_observed": unexpected_contact,
        "course": course,
        "measured_tool_trajectory": trajectory,
        "finite_state": finite_joint_vector(final_joints),
        "planner": "nearest-unobserved tag coverage using iterative damped-least-squares IK from Webots position sensors",
        "state_authority": "Webots vendor-joint position sensors evaluated through pinned vendor CR3 forward kinematics",
    }
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    hold_seconds = max(0.0, float(os.environ.get("DOBOT_CR3_WEBOTS_VIEWER_HOLD_SECONDS", "0")))
    until = robot.getTime() + hold_seconds
    while robot.getTime() < until and robot.step(timestep) != -1:
        for motor, target in zip(motors, final_joints):
            motor.setPosition(target)
    robot.simulationQuit(0 if success else 1)
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
