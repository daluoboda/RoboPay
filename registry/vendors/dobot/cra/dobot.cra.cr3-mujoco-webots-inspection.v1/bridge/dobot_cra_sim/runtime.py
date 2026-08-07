"""Real MuJoCo execution of the bounded Dobot CR3 inspection task."""

from __future__ import annotations

import math
import time
from threading import Event
from typing import Any

from .contracts import INSPECTION_SKILL, STOP_SKILL, InspectionRequest
from .course import COURSE_ID, MAX_DURATION_SECONDS, fingerprint, spec
from .kinematics import finite_joint_vector
from .model import TOOL_BODY, joint_addresses, load_mujoco_model
from .task import InspectionTask


PLAN_INTERVAL_SECONDS = 0.12
TRACE_INTERVAL_SECONDS = 0.25
MIN_TOOL_HEIGHT_M = 0.15


def run_mujoco_episode(
    request: InspectionRequest,
    *,
    stop_event: Event | None = None,
    viewer: bool = False,
    viewer_hold_seconds: float = 0.0,
) -> dict[str, Any]:
    """Execute an online three-target task using vendor joints and MuJoCo state."""

    import mujoco

    model = load_mujoco_model()
    data = mujoco.MjData(model)
    actuators, qpos_addresses, _ = joint_addresses(model)
    tool_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, TOOL_BODY)
    if tool_id < 0:
        raise RuntimeError("Converted vendor CR3 model is missing Link6")
    mujoco.mj_forward(model, data)

    task = InspectionTask()
    desired = [float(data.qpos[address]) for address in qpos_addresses]
    next_plan_time = 0.0
    next_trace_time = 0.0
    viewer_context = None
    if viewer:
        import mujoco.viewer

        viewer_context = mujoco.viewer.launch_passive(model, data)
        viewer_context.cam.lookat[:] = (0.0, -0.15, 0.42)
        viewer_context.cam.distance = 1.55
        viewer_context.cam.azimuth = 140.0
        viewer_context.cam.elevation = -24.0

    state: dict[str, Any] = {
        "failure_reason": None,
        "safe_stop_applied": False,
        "minimum_tool_height_m": float("inf"),
        "peak_actuator_force_nm": 0.0,
        "unexpected_contact_observed": False,
        "trajectory": [],
    }
    try:
        if request.skill_id == STOP_SKILL:
            state["safe_stop_applied"] = True
            state["failure_reason"] = "safe_stopped"
        elif request.skill_id != INSPECTION_SKILL:
            raise ValueError("MuJoCo CR3 runtime only accepts registered skills")
        while state["failure_reason"] is None and data.time < request.max_duration_sec:
            if stop_event is not None and stop_event.is_set():
                state["safe_stop_applied"] = True
                state["failure_reason"] = "safe_stopped"
                break
            measured_joints = [float(data.qpos[address]) for address in qpos_addresses]
            tool_position = tuple(float(value) for value in data.xpos[tool_id])
            if data.time >= next_plan_time:
                task.update(tool_position, float(data.time))
                if task.complete:
                    break
                desired = list(task.plan(measured_joints, tool_position))
                next_plan_time += PLAN_INTERVAL_SECONDS
            data.ctrl[actuators] = desired
            mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            tool_position = tuple(float(value) for value in data.xpos[tool_id])
            state["minimum_tool_height_m"] = min(state["minimum_tool_height_m"], tool_position[2])
            state["peak_actuator_force_nm"] = max(
                state["peak_actuator_force_nm"], max(abs(float(data.actuator_force[index])) for index in actuators)
            )
            state["unexpected_contact_observed"] = state["unexpected_contact_observed"] or data.ncon > 0
            if data.time >= next_trace_time:
                state["trajectory"].append(
                    {"time_seconds": round(float(data.time), 3), "tool_position_m": [round(value, 5) for value in tool_position]}
                )
                next_trace_time += TRACE_INTERVAL_SECONDS
            if not finite_joint_vector(data.qpos) or not finite_joint_vector(data.qvel):
                state["failure_reason"] = "nonfinite_simulator_state"
                break
            if tool_position[2] < MIN_TOOL_HEIGHT_M:
                state["failure_reason"] = "unsafe_tool_height"
                break
            if viewer_context is not None:
                viewer_context.sync()
                time.sleep(model.opt.timestep)
    finally:
        mujoco.mj_forward(model, data)

    final_tool_position = tuple(float(value) for value in data.xpos[tool_id])
    completed = task.complete and state["failure_reason"] is None
    safe_stop_confirmed = request.skill_id == STOP_SKILL and state["safe_stop_applied"]
    if not completed and not safe_stop_confirmed and state["failure_reason"] is None:
        state["failure_reason"] = "inspection_timeout"
    if viewer_context is not None:
        deadline = time.monotonic() + max(0.0, viewer_hold_seconds)
        while viewer_context.is_running() and time.monotonic() < deadline:
            viewer_context.sync()
            time.sleep(0.02)
        viewer_context.close()
    return {
        "simulator_engine": "MuJoCo",
        "robot_model": "Dobot CR3 vendor URDF compiled by MuJoCo with profile-owned bounded position actuators",
        "task": request.skill_id,
        "course_id": COURSE_ID,
        "course_hash": fingerprint(),
        "status": "success" if completed or safe_stop_confirmed else "failure",
        "success": completed or safe_stop_confirmed,
        "completion_reason": (
            "all_three_tags_observed" if completed else "safe_stopped" if safe_stop_confirmed else state["failure_reason"]
        ),
        "safe_stop_applied": state["safe_stop_applied"],
        "sim_duration_seconds": round(float(data.time), 4),
        "observed_tags": task.observed,
        "remaining_tags": sorted(task.pending),
        "final_tool_position_m": [round(value, 5) for value in final_tool_position],
        "final_joint_positions_rad": [round(float(data.qpos[address]), 5) for address in qpos_addresses],
        "minimum_tool_height_m": round(float(state["minimum_tool_height_m"]), 5),
        "peak_actuator_force_nm": round(float(state["peak_actuator_force_nm"]), 5),
        "unexpected_contact_observed": state["unexpected_contact_observed"],
        "course": {**spec(), "course_hash": fingerprint()},
        "measured_tool_trajectory": state["trajectory"],
        "finite_state": finite_joint_vector(data.qpos) and finite_joint_vector(data.qvel),
        "planner": "nearest-unobserved tag coverage using iterative damped-least-squares IK from measured vendor joint state",
        "state_authority": "MuJoCo Link6 body pose, vendor-joint qpos, actuator force, contacts, and finite dynamic state",
        "viewer_enabled": viewer,
    }
