"""Actuator-level Webots controller for the official LimX WF_TRON1A.

Webots integrates the converted vendor model at 500 Hz.  The pinned LimX
ONNX policy is evaluated at its published 50 Hz decimation and the resulting
torques are retained between policy ticks.  The robot root is never written.
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np
from controller import Supervisor


PROFILE_ROOT = Path(__file__).resolve().parents[4]
BRIDGE_ROOT = PROFILE_ROOT / "bridge"
if str(BRIDGE_ROOT) not in sys.path:
    sys.path.insert(0, str(BRIDGE_ROOT))

from limx_tron1_sim.contracts import MAX_DURATION_SECONDS, NAVIGATION_SKILL, STOP_SKILL
from limx_tron1_sim.course import GOAL, OBSTACLES, WAYPOINTS, RoutePlanner, obstacle_clearance
from limx_tron1_sim.policy import JOINT_NAMES, LimXOnnxPolicy


SETTLE_SECONDS = 2.0
POLICY_PERIOD_SECONDS = 0.020
MIN_SAFE_BASE_HEIGHT_M = 0.44
MAX_SAFE_TILT_RAD = 0.90
WEBOTS_LINEAR_COMMAND_SCALE = 0.55
WEBOTS_YAW_COMMAND_SCALE = 0.55


def _parameters() -> tuple[dict, Path]:
    config_path = Path(os.environ["LIMX_TRON1_WEBOTS_CONFIG_PATH"])
    result_path = Path(os.environ["LIMX_TRON1_WEBOTS_RESULT_PATH"])
    return json.loads(config_path.read_text(encoding="utf-8")), result_path


def _required_device(robot: Supervisor, name: str):
    device = robot.getDevice(name)
    if device is None:
        raise RuntimeError(f"official WF_TRON1A Webots model is missing device {name}")
    return device


def _quaternion_wxyz_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.asarray(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=np.float64,
    )


def _joint_velocity(q: np.ndarray, previous_q: np.ndarray | None, dt: float) -> np.ndarray:
    if previous_q is None:
        return np.zeros(8, dtype=np.float64)
    delta = q - previous_q
    for index in (3, 7):
        delta[index] = (delta[index] + math.pi) % (2.0 * math.pi) - math.pi
    return delta / dt


def main() -> int:
    parameters, result_path = _parameters()
    skill_id = parameters.get("skill_id")
    if skill_id not in {NAVIGATION_SKILL, STOP_SKILL}:
        raise RuntimeError("Webots controller received an unregistered skill")
    if float(parameters.get("max_duration_sec", -1)) != MAX_DURATION_SECONDS:
        raise RuntimeError("Webots controller received an unbounded duration")

    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0
    if timestep > 2:
        raise RuntimeError("WF_TRON1A torque control requires a Webots basicTimeStep of 2 ms or less")
    policy_steps = max(1, round(POLICY_PERIOD_SECONDS / dt))

    gps = _required_device(robot, "ground_truth/state gps")
    imu = _required_device(robot, "ground_truth/state inertial")
    gyro = _required_device(robot, "ground_truth/state gyro")
    gps.enable(timestep)
    imu.enable(timestep)
    gyro.enable(timestep)
    motors = {name: _required_device(robot, name) for name in JOINT_NAMES}
    joint_sensors = {name: _required_device(robot, f"{name}_sensor") for name in JOINT_NAMES}
    for sensor in joint_sensors.values():
        sensor.enable(timestep)
    for name, motor in motors.items():
        motor.setPosition(float("inf"))
        motor.setVelocity(40.0 if "wheel" in name else 15.0)
        motor.setTorque(0.0)

    self_node = robot.getSelf()
    self_node.enableContactPointsTracking(timestep, True)
    policy = LimXOnnxPolicy()
    planner = RoutePlanner()
    elapsed = 0.0
    step_index = 0
    path_length = 0.0
    detected: set[str] = set()
    min_clearance = float("inf")
    collision = False
    max_tilt = 0.0
    max_contact_points = 0
    terminal_reason = "timeout"
    last_position: tuple[float, float] | None = None
    initial_position: tuple[float, float, float] | None = None
    previous_q: np.ndarray | None = None
    last_actions = np.zeros(8, dtype=np.float64)
    command_samples: list[dict] = []
    progress_path = result_path.with_name("tron1_webots_progress.json")
    progress_path.unlink(missing_ok=True)
    next_progress_at = 5.0

    while elapsed < MAX_DURATION_SECONDS and robot.step(timestep) != -1:
        measured = gps.getValues()
        roll, pitch, yaw = imu.getRollPitchYaw()
        x, y, z = float(measured[0]), float(measured[1]), float(measured[2])
        if initial_position is None:
            initial_position = (x, y, z)
        if last_position is not None:
            path_length += math.hypot(x - last_position[0], y - last_position[1])
        last_position = (x, y)
        max_tilt = max(max_tilt, abs(float(roll)), abs(float(pitch)))
        max_contact_points = max(max_contact_points, len(self_node.getContactPoints(True)))

        for obstacle in OBSTACLES:
            clearance = obstacle_clearance(x, y, obstacle)
            min_clearance = min(min_clearance, clearance)
            if math.hypot(x - obstacle.x, y - obstacle.y) <= 1.45:
                detected.add(obstacle.name)
        collision = collision or min_clearance < -0.02
        if collision:
            terminal_reason = "collision"
            break

        q = np.asarray([float(joint_sensors[name].getValue()) for name in JOINT_NAMES], dtype=np.float64)
        dq = _joint_velocity(q, previous_q, dt)
        previous_q = q.copy()
        angular_velocity = np.asarray(gyro.getValues(), dtype=np.float64)
        quaternion_wxyz = _quaternion_wxyz_from_rpy(float(roll), float(pitch), float(yaw))
        if not all(
            math.isfinite(value)
            for value in [x, y, z, roll, pitch, yaw, *q, *dq, *angular_velocity, *quaternion_wxyz]
        ):
            terminal_reason = "non_finite_state"
            break
        if elapsed > SETTLE_SECONDS and (
            z < MIN_SAFE_BASE_HEIGHT_M or max(abs(roll), abs(pitch)) > MAX_SAFE_TILT_RAD
        ):
            terminal_reason = "unsafe_base_state"
            break
        if skill_id == STOP_SKILL:
            terminal_reason = "safe_stopped"
            break

        route_command = (0.0, 0.0, 0.0) if elapsed < SETTLE_SECONDS else planner.command(x, y, yaw)
        command = (
            WEBOTS_LINEAR_COMMAND_SCALE * route_command[0],
            route_command[1],
            WEBOTS_YAW_COMMAND_SCALE * route_command[2],
        )
        if step_index % policy_steps == 0:
            last_actions = policy.actions(q, dq, quaternion_wxyz, angular_velocity, command)
        torques = policy.action_torques(last_actions, q, dq)
        for index, name in enumerate(JOINT_NAMES):
            motors[name].setTorque(float(torques[index]))

        if len(command_samples) < 140 and int(elapsed * 2) > len(command_samples):
            command_samples.append(
                {
                    "t": round(elapsed, 2),
                    "x": round(x, 3),
                    "y": round(y, 3),
                    "z": round(z, 3),
                    "roll": round(float(roll), 3),
                    "pitch": round(float(pitch), 3),
                    "yaw": round(float(yaw), 3),
                    "command_linear_mps": round(float(command[0]), 3),
                    "command_yaw_rad_s": round(float(command[2]), 3),
                    "left_wheel_torque_nm": round(float(torques[3]), 3),
                    "right_wheel_torque_nm": round(float(torques[7]), 3),
                }
            )
        if elapsed >= next_progress_at:
            progress_path.write_text(
                json.dumps(
                    {
                        "sim_duration_seconds": round(elapsed, 3),
                        "x": round(x, 4),
                        "y": round(y, 4),
                        "z": round(z, 4),
                        "roll": round(float(roll), 4),
                        "pitch": round(float(pitch), 4),
                        "yaw": round(float(yaw), 4),
                        "waypoints_completed": len(planner.visited),
                        "collision": collision,
                    },
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
            next_progress_at += 5.0
        elapsed += dt
        step_index += 1
        if planner.complete and math.hypot(x - GOAL[0], y - GOAL[1]) <= 0.34:
            terminal_reason = "goal_reached"
            break

    for motor in motors.values():
        motor.setTorque(0.0)
    final = gps.getValues()
    final_roll, final_pitch, final_yaw = imu.getRollPitchYaw()
    goal_distance = math.hypot(float(final[0]) - GOAL[0], float(final[1]) - GOAL[1])
    displacement = math.hypot(float(final[0]) - initial_position[0], float(final[1]) - initial_position[1])
    success = terminal_reason in {"goal_reached", "safe_stopped"} and not collision
    if terminal_reason == "goal_reached":
        success = (
            success
            and len(planner.visited) == len(WAYPOINTS)
            and len(detected) == len(OBSTACLES)
            and displacement > 4.0
            and goal_distance <= 0.34
        )
    result = {
        "success": success,
        "skill": skill_id,
        "simulator": "webots",
        "model_variant": "WF_TRON1A vendor URDF converted to Webots R2025a",
        "controller": "pinned LimX Isaac Gym ONNX policy at 50 Hz with 500 Hz torque hold",
        "actuation": "torque commands on all eight vendor joints; no root-pose writes",
        "state_authority": "Webots GPS, IMU, gyro, joint sensors, contacts and physics-derived base motion",
        "physics_frequency_hz": round(1.0 / dt, 1),
        "policy_frequency_hz": round(1.0 / (policy_steps * dt), 1),
        "supervisor_root_writes": 0,
        "terminal_reason": terminal_reason,
        "waypoints_completed": len(planner.visited),
        "waypoints_total": len(WAYPOINTS),
        "detected_obstacles": sorted(detected),
        "collision": collision,
        "minimum_clearance_m": round(min_clearance, 4),
        "path_length_m": round(path_length, 4),
        "physical_displacement_m": round(displacement, 4),
        "goal_distance_m": round(goal_distance, 4),
        "final_base_pose": {
            "x": round(float(final[0]), 4),
            "y": round(float(final[1]), 4),
            "z": round(float(final[2]), 4),
            "roll": round(float(final_roll), 4),
            "pitch": round(float(final_pitch), 4),
            "yaw": round(float(final_yaw), 4),
        },
        "max_tilt_rad": round(max_tilt, 4),
        "max_contact_points": max_contact_points,
        "sim_duration_seconds": round(elapsed, 4),
        "safe_stop_applied": terminal_reason == "safe_stopped",
        "command_samples": command_samples,
    }
    if not success:
        result["error_code"] = "COURSE_NOT_COMPLETED"
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    hold_seconds = max(0.0, float(os.environ.get("LIMX_TRON1_WEBOTS_VIEWER_HOLD_SECONDS", "0")))
    hold_until = robot.getTime() + hold_seconds
    while robot.getTime() < hold_until and robot.step(timestep) != -1:
        for motor in motors.values():
            motor.setTorque(0.0)
    robot.simulationQuit(0 if success else 1)
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
