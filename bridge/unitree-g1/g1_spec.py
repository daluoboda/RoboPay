"""unitree-g1 --- engine-independent robot spec and skill plan.

Single source of truth shared by every physics backend. G1 is a 29-DOF
humanoid robot with free base (floating). Locomotion is controlled through
Mujoco actuators on the root body and joint position targets.

Link lengths, joint limits, the trajectory keyframes, the scene table and
the pass/fail thresholds all live here.
"""
from __future__ import annotations

import math

# ---------------------------------------------------------------- geometry --
# G1 humanoid robot specifications (29 DOF)
BASE_HEIGHT = 0.05           # base plate height from ground (m)
HIP_HEIGHT = 0.45            # hip joint height (m)
THIGH_LEN = 0.22             # thigh link length
SHIN_LEN = 0.22              # shin link length
CALF_LEN = 0.18              # calf link length
FOOT_LEN = 0.05              # foot length
TOTAL_HEIGHT = 1.05          # total robot height (m)

# Joint limits (radians)
HIP_ROLL_MIN = -0.43         # hip roll
HIP_ROLL_MAX = 0.43
HIP_PITCH_MIN = -1.57        # hip pitch
HIP_PITCH_MAX = 1.57
HIP_YAW_MIN = -0.43          # hip yaw
HIP_YAW_MAX = 0.43
KNEE_MIN = 0.0               # knee (always positive)
KNEE_MAX = 2.8               # knee max bend
ANKLE_PITCH_MIN = -0.52      # ankle pitch
ANKLE_PITCH_MAX = 0.52

# G1 has 29 DOF total:
# - 6 DOF per leg (hip_roll, hip_pitch, hip_yaw, knee, ankle_pitch, ankle_roll)
# - 2 DOF per arm (shoulder_pitch, elbow)
# - 1 DOF head_yaw
# Total: 12 (legs) + 4 (arms) + 1 (head) + 10 (torso/waist) = 27-29 DOF

JOINT_COUNT = 29
LEG_JOINTS_PER_SIDE = 6      # hip_roll, hip_pitch, hip_yaw, knee, ankle_pitch, ankle_roll
ARM_JOINTS_PER_SIDE = 2      # shoulder_pitch, elbow

TIMESTEP = 0.002

# --------------------------------------------------------- trajectory plan --
STAGE_STEPS = {
    "init": 10,               # initialize posture
    "move_forward": 150,      # walk forward N steps
    "stop": 30,               # bring to rest
}
NOMINAL_STEPS = sum(STAGE_STEPS.values())
DEFAULT_BUDGET = 300         # generous budget

# --------------------------------------------------------------- decisions --
WALK_SPEED_MIN = 0.0         # m/s
WALK_SPEED_MAX = 1.0         # m/s
WALK_SPEED_DEFAULT = 0.5
WALK_DURATION_MIN = 0.1      # seconds
WALK_DURATION_MAX = 10.0
GOAL_THRESHOLD = 0.3         # meters - distance to consider goal reached

# ------------------------------------------------------------- scene table --
SCENES = {
    "move_forward": {
        "durationSec": 3.0,
        "speed": 0.5,
        "obstacles": [],
        "budget": DEFAULT_BUDGET,
    },
    "navigate_obstacle": {
        "goal_x": 3.0,
        "goal_y": 0.0,
        "obstacles": [(1.5, 0.5), (2.0, -0.5)],
        "budget": DEFAULT_BUDGET,
    },
    "stop": {
        "durationSec": 0.0,
        "speed": 0.0,
        "budget": 50,
    },
}
ALIASES = {
    "forward": "move_forward",
    "walk": "move_forward",
    "obstacle": "navigate_obstacle",
    "nav": "navigate_obstacle",
}


def resolve_scene(params: dict | None):
    """(display_name, scene_key, scene_dict) for a skill parameter block."""
    params = params or {}
    name = str(params.get("skill", params.get("object", "move_forward")))
    key = ALIASES.get(name, name)
    if key not in SCENES:
        key = "move_forward"
    scene = dict(SCENES[key])
    if "durationSec" in params:
        scene["durationSec"] = float(params["durationSec"])
    if "speed" in params:
        scene["speed"] = float(params["speed"])
    if "goal_x" in params:
        scene["goal_x"] = float(params["goal_x"])
    if "goal_y" in params:
        scene["goal_y"] = float(params["goal_y"])
    return name, key, scene


# ------------------------------------------------------------------ result --
class WalkResult:
    def __init__(self, success: bool, reason: str, metrics: dict):
        self.success = success
        self.reason = reason
        self.metrics = metrics

    def to_dict(self) -> dict:
        return {"success": self.success, "reason": self.reason,
                "metrics": self.metrics}

    def __repr__(self) -> str:                        # pragma: no cover
        return f"WalkResult({self.success}, {self.reason!r}, {self.metrics})"


class BudgetExhausted(Exception):
    """Raised when the hard step budget runs out mid-trajectory."""


def build_metrics(*, engine, scene_key, stage, start_pos, end_pos,
                  steps, budget, wall_time, note) -> dict:
    """Identical metric schema for every backend."""
    delta = [round(end_pos[i] - start_pos[i], 4) + 0.0 for i in range(3)]
    return {
        "robotId": "unitree-g1",
        "skillId": "move_forward" if scene_key == "move_forward" else "navigate_obstacle",
        "engine": engine,
        "scene": scene_key,
        "stage": stage,
        "positionStart": [round(v, 4) + 0.0 for v in start_pos],
        "positionEnd": [round(v, 4) + 0.0 for v in end_pos],
        "positionDelta": delta,
        "distanceTraveled": round(math.sqrt(delta[0]**2 + delta[1]**2), 4),
        "stepsUsed": steps,
        "stepBudget": budget,
        "simTime": round(steps * TIMESTEP, 4),
        "wallTime": round(wall_time, 4),
        "note": note,
    }
