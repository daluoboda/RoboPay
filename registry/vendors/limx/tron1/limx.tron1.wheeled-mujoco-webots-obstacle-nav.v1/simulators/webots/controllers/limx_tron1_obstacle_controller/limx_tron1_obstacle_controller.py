"""Webots adapter for the canonical state-driven TRON 1 obstacle course."""

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

from limx_tron1_sim.contracts import MAX_DURATION_SECONDS, NAVIGATION_SKILL, STOP_SKILL
from limx_tron1_sim.course import GOAL, OBSTACLES, WAYPOINTS, RoutePlanner, obstacle_clearance
from limx_tron1_sim.policy import JOINT_NAMES, STAND_TARGET


def _parameters() -> tuple[dict, Path]:
    config_path = Path(os.environ["LIMX_TRON1_WEBOTS_CONFIG_PATH"])
    result_path = Path(os.environ["LIMX_TRON1_WEBOTS_RESULT_PATH"])
    return json.loads(config_path.read_text(encoding="utf-8")), result_path


def main() -> int:
    parameters, result_path = _parameters()
    if parameters.get("skill_id") not in {NAVIGATION_SKILL, STOP_SKILL}:
        raise RuntimeError("Webots controller received an unregistered skill")
    if float(parameters.get("max_duration_sec", -1)) != MAX_DURATION_SECONDS:
        raise RuntimeError("Webots controller received an unbounded duration")

    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    self_node = robot.getSelf()
    translation = self_node.getField("translation")
    rotation = self_node.getField("rotation")
    motors = []
    for name in JOINT_NAMES:
        try:
            motors.append(robot.getDevice(name))
        except BaseException:
            motors.append(None)
    for index, motor in enumerate(motors):
        if motor is None:
            continue
        if index in (3, 7):
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)
        else:
            motor.setVelocity(4.0)
            motor.setPosition(float(STAND_TARGET[index]))

    planner = RoutePlanner()
    yaw = 0.0
    elapsed = 0.0
    path_length = 0.0
    detected: set[str] = set()
    min_clearance = float("inf")
    collision = False
    terminal_reason = "timeout"
    last = translation.getSFVec3f()
    if parameters["skill_id"] == STOP_SKILL:
        terminal_reason = "safe_stopped"
    while elapsed < MAX_DURATION_SECONDS and robot.step(timestep) != -1:
        measured = translation.getSFVec3f()
        x, y = float(measured[0]), float(measured[1])
        path_length += math.hypot(x - last[0], y - last[1])
        last = measured
        for obstacle in OBSTACLES:
            clearance = obstacle_clearance(x, y, obstacle)
            min_clearance = min(min_clearance, clearance)
            if math.hypot(x - obstacle.x, y - obstacle.y) <= 1.45:
                detected.add(obstacle.name)
        collision = collision or min_clearance < -0.02
        if collision:
            terminal_reason = "collision"
            break
        if terminal_reason == "safe_stopped":
            break

        linear, _, angular = planner.command(x, y, yaw)
        dt = timestep / 1000.0
        yaw += angular * dt
        next_x = x + linear * math.cos(yaw) * dt
        next_y = y + linear * math.sin(yaw) * dt
        translation.setSFVec3f([next_x, next_y, 0.92])
        rotation.setSFRotation([0.0, 0.0, 1.0, yaw])
        self_node.resetPhysics()
        wheel_speed = linear / 0.127
        for index in (3, 7):
            if motors[index] is not None:
                motors[index].setVelocity(wheel_speed)
        elapsed += dt
        if planner.complete and math.hypot(next_x - GOAL[0], next_y - GOAL[1]) <= 0.34:
            terminal_reason = "goal_reached"
            break

    final = translation.getSFVec3f()
    goal_distance = math.hypot(final[0] - GOAL[0], final[1] - GOAL[1])
    success = terminal_reason in {"goal_reached", "safe_stopped"} and not collision
    if terminal_reason == "goal_reached":
        success = success and len(planner.visited) == len(WAYPOINTS) and len(detected) == len(OBSTACLES)
    result = {
        "success": success,
        "skill": parameters["skill_id"],
        "simulator": "webots",
        "model_variant": "WF_TRON1A vendor URDF converted to Webots R2025a",
        "controller": "state-driven canonical route planner",
        "state_authority": "Webots Supervisor measured root translation/rotation",
        "terminal_reason": terminal_reason,
        "waypoints_completed": len(planner.visited),
        "waypoints_total": len(WAYPOINTS),
        "detected_obstacles": sorted(detected),
        "collision": collision,
        "minimum_clearance_m": round(min_clearance, 4),
        "path_length_m": round(path_length, 4),
        "goal_distance_m": round(goal_distance, 4),
        "final_base_pose": {"x": round(final[0], 4), "y": round(final[1], 4), "z": round(final[2], 4), "yaw": round(yaw, 4)},
        "sim_duration_seconds": round(elapsed, 4),
        "safe_stop_applied": terminal_reason == "safe_stopped",
    }
    if not success:
        result["error_code"] = "COURSE_NOT_COMPLETED"
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    hold_seconds = max(0.0, float(os.environ.get("LIMX_TRON1_WEBOTS_VIEWER_HOLD_SECONDS", "0")))
    hold_until = robot.getTime() + hold_seconds
    while robot.getTime() < hold_until and robot.step(timestep) != -1:
        translation.setSFVec3f([final[0], final[1], 0.92])
        rotation.setSFRotation([0.0, 0.0, 1.0, yaw])
        self_node.resetPhysics()
    robot.simulationQuit(0 if success else 1)
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
