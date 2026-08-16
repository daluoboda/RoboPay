"""push-arm-001 --- MuJoCo backend for the RoboPay Tier 1 skill bridge.

Aligned with fabric/stack/sort/fetch so all arms are a sim-to-sim comparison.

`push_box` is a genuine contact push, distinct from the pick-and-place skills:
the gripper closes around the box and *translates it along the floor* by
sustained contact -- there is no lift and the equality grasp constraint is
never engaged, so the box stays a free body the whole time and is only moved by
friction/contact with the pads. The PHYSICS is real (gravity, friction, contact
dynamics solved by MuJoCo); the CONTROLLER is deterministic; the FAILURES are
real (`unreachable` stretches + measures shortfall, `collision` aborts on a
genuine contact, `timeout` exhausts the budget, `offtarget` means the box did
not reach the target zone).

Public surface consumed by flow/executor.py:
    MuJoCoSimulator().push_box(params) -> PickResult(success, reason, metrics)
"""
from __future__ import annotations

import time

import mujoco
import numpy as np

from arm_spec import (
    ARM_JOINTS, BASE_H, LINK1, LINK2, GRIP_MID, FINGER_OPEN, FINGER_CLOSED,
    CUBE_HALF, CUBE_MASS, CUBE_FRICTION, FINGER_HALF_X, FINGER_HALF_Z, PAD_HALF,
    GRASP_FORCE_MIN, TIMESTEP, DEFAULT_BUDGET, UNREACHABLE_GAP, WORK_R,
    OBSTACLE_RADIUS, OBSTACLE_HALF_H,
    PickResult, BudgetExhausted, build_metrics,
    solve, blend, aperture_at,
)

ENGINE = "mujoco"
ROBOT_ID = "push-arm-001"
SKILL_ID = "push_box"

# Target zone on the floor, +x of the box.
TARGET_ZONE_XY = (0.44, 0.0)
PUSH_TOL = 0.08
STAGE_STEPS = {
    "approach": 70, "close": 50, "push": 80, "settle": 40,
}
NOMINAL_STEPS = sum(STAGE_STEPS.values())
GRASP_WZ = CUBE_HALF + GRIP_MID


# ----------------------------------------------------------------- scene table
SCENES = {
    "pushable":   {"box": (0.35, 0.0), "obstacle": None, "budget": DEFAULT_BUDGET},
    "unreachable": {"box": (0.95, 0.0), "obstacle": None, "budget": DEFAULT_BUDGET},
    "collision":   {"box": (0.35, 0.0), "obstacle": (0.27, 0.0), "budget": DEFAULT_BUDGET},
    "timeout":     {"box": (0.35, 0.0), "obstacle": None, "budget": 60},
}
ALIASES = {"far_box": "unreachable", "blocked_box": "collision", "slow_box": "timeout"}


def resolve_scene(params: dict | None):
    params = params or {}
    name = str(params.get("object", "pushable"))
    key = ALIASES.get(name, name)
    if key not in SCENES:
        key = "pushable"
    scene = dict(SCENES[key])
    if "maxSteps" in params:
        scene["budget"] = int(params["maxSteps"])
    return name, key, scene


# ------------------------------------------------------ closed-form keyframes
# Finger sits just behind the box (smaller x) at box-centre height, then sweeps
# forward; the closed pads translate the box by sustained contact.
KEYFRAMES = {
    "home": solve(0.20, 0.42),
    "push_start": solve(0.30, GRASP_WZ),
    "push_end": solve(0.41, GRASP_WZ),
    "stretch": {"pan": 0.0, "shoulder": 0.0, "elbow": 0.0, "wristp": 0.0},
}
for _k, _v in KEYFRAMES.items():
    if _v is None:                                    # pragma: no cover
        raise RuntimeError(f"keyframe {_k} is not solvable -- check link geometry")


def _model_xml(box_xy, obstacle_xy) -> str:
    cx, cy = box_xy
    obstacle = ""
    if obstacle_xy is not None:
        ox, oy = obstacle_xy
        obstacle = f"""
    <body name="obstacle" pos="{ox} {oy} 0">
      <geom name="obstacle_g" type="cylinder"
            size="{OBSTACLE_RADIUS} {OBSTACLE_HALF_H}" pos="0 0 {OBSTACLE_HALF_H}"
            rgba="0.80 0.25 0.25 1" contype="8" conaffinity="22"/>
    </body>"""
    return f"""
<mujoco model="push-arm-001">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{TIMESTEP}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default>
    <joint damping="2" armature="0.01"/>
    <geom solref="0.006 1" solimp="0.95 0.99 0.001"/>
  </default>

  <worldbody>
    <light pos="0.4 0 1.6" dir="0 0 -1" diffuse="0.9 0.9 0.9"/>
    <geom name="floor" type="plane" size="2 2 0.05" rgba="0.16 0.18 0.22 1"
          contype="1" conaffinity="6" friction="1.0 0.01 0.001"/>

    <body name="base" pos="0 0 0">
      <geom name="base_g" type="cylinder" size="0.07 0.025" pos="0 0 0.025"
            rgba="0.25 0.27 0.32 1" contype="16" conaffinity="8"/>
      <body name="column" pos="0 0 0.05">
        <joint name="pan" type="hinge" axis="0 0 1" range="-3.1416 3.1416"/>
        <geom name="column_g" type="capsule" fromto="0 0 0 0 0 0.35" size="0.035"
              rgba="0.30 0.32 0.38 1" contype="16" conaffinity="8"/>
        <body name="upper" pos="0 0 0.35">
          <joint name="shoulder" type="hinge" axis="0 1 0" range="-2.0 2.0"/>
          <geom name="upper_g" type="capsule" fromto="0 0 0 {LINK1} 0 0" size="0.030"
                rgba="0.85 0.55 0.18 1" contype="16" conaffinity="8"/>
          <body name="fore" pos="{LINK1} 0 0">
            <joint name="elbow" type="hinge" axis="0 1 0" range="-2.6 2.6"/>
            <geom name="fore_g" type="capsule" fromto="0 0 0 {LINK2} 0 0" size="0.026"
                  rgba="0.85 0.55 0.18 1" contype="16" conaffinity="8"/>
            <body name="wrist" pos="{LINK2} 0 0">
              <joint name="wristp" type="hinge" axis="0 1 0" range="-2.8 2.8"/>
              <geom name="wrist_g" type="box" size="0.032 0.030 0.018"
                    rgba="0.30 0.32 0.38 1" contype="16" conaffinity="8"/>
              <site name="grip_site" pos="0 0 -{GRIP_MID}" size="0.006"
                    rgba="0.9 0.9 0.2 0.4"/>
              <body name="finger_l" pos="0 0 -{GRIP_MID}">
                <joint name="grip_l" type="slide" axis="0 1 0" range="0.012 0.060"/>
                <geom name="finger_l_g" type="box"
                      size="{FINGER_HALF_X} {PAD_HALF} {FINGER_HALF_Z}"
                      rgba="0.90 0.90 0.92 1" contype="4" conaffinity="45"
                      friction="{CUBE_FRICTION} 0.05 0.001"
                      solref="0.02 1" solimp="0.90 0.95 0.001"/>
              </body>
              <body name="finger_r" pos="0 0 -{GRIP_MID}">
                <joint name="grip_r" type="slide" axis="0 -1 0" range="0.012 0.060"/>
                <geom name="finger_r_g" type="box"
                      size="{FINGER_HALF_X} {PAD_HALF} {FINGER_HALF_Z}"
                      rgba="0.90 0.90 0.92 1" contype="4" conaffinity="45"
                      friction="{CUBE_FRICTION} 0.05 0.001"
                      solref="0.02 1" solimp="0.90 0.95 0.001"/>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>

    <body name="box" pos="{cx} {cy} {CUBE_HALF}">
      <freejoint name="box_free"/>
      <geom name="box_g" type="box" size="{CUBE_HALF} {CUBE_HALF} {CUBE_HALF}"
            mass="{CUBE_MASS}" rgba="0.20 0.70 0.45 1"
            contype="2" conaffinity="45" friction="{CUBE_FRICTION} 0.05 0.001"
            solref="0.02 1" solimp="0.90 0.95 0.001"/>
      <site name="box_site" pos="0 0 0" size="0.006" rgba="0.2 0.9 0.5 0.4"/>
    </body>{obstacle}
  </worldbody>

  <equality>
    <connect name="grasp" site1="box_site" site2="grip_site" active="false"/>
  </equality>
</mujoco>
"""


# --------------------------------------------------------------- simulator --
class MuJoCoSimulator:
    ROBOT_ID = ROBOT_ID
    SKILL_ID = SKILL_ID
    ENGINE = ENGINE

    def __init__(self):
        self.model = None
        self.data = None
        self._steps = 0
        self._budget = SCENES["pushable"]["budget"]

    def _build(self, scene: dict):
        xml = _model_xml(scene["box"], scene["obstacle"])
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        m = self.model
        self._qadr, self._vadr = {}, {}
        for name in ARM_JOINTS + ("grip_l", "grip_r"):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            self._qadr[name] = m.jnt_qposadr[jid]
            self._vadr[name] = m.jnt_dofadr[jid]

        def gid(n):
            return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)

        self._box_geom = gid("box_g")
        self._finger_geoms = {gid("finger_l_g"), gid("finger_r_g")}
        self._obs_geom = gid("obstacle_g") if scene["obstacle"] else -1
        self._arm_geoms = {gid(n) for n in
                           ("base_g", "column_g", "upper_g", "fore_g", "wrist_g")}
        self._box_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "box")
        self._grip_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grip_site")
        self._eq_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "grasp")
        self._pose = dict(KEYFRAMES["home"])
        self._grip = FINGER_OPEN
        self._steps = 0
        self._peak_force = 0.0
        self._hold_forces = []
        self._contact_samples = 0
        self._collisions = 0
        self._apply(self._pose, self._grip)
        mujoco.mj_forward(self.model, self.data)

    def _apply(self, pose: dict, grip: float):
        d = self.data
        for name in ARM_JOINTS:
            d.qpos[self._qadr[name]] = pose[name]
            d.qvel[self._vadr[name]] = 0.0
        for name in ("grip_l", "grip_r"):
            d.qpos[self._qadr[name]] = grip
            d.qvel[self._vadr[name]] = 0.0

    def _tick(self, pose: dict, grip: float):
        if self._steps >= self._budget:
            raise BudgetExhausted
        self._apply(pose, grip)
        mujoco.mj_step(self.model, self.data)
        self._apply(pose, grip)
        self._steps += 1
        self._pose, self._grip = pose, grip
        if self._obs_geom >= 0 and self._obstacle_contact():
            self._collisions += 1

    def _run(self, target: dict, n: int, grip: float, abort_on_collision=True):
        start = dict(self._pose)
        for i in range(1, n + 1):
            self._tick(blend(start, target, i / n), grip)
            if abort_on_collision and self._collisions:
                return False
        return True

    def _hold(self, n: int, grip: float, sample: bool = False):
        for _ in range(n):
            self._tick(dict(self._pose), grip)
            if sample:
                f, _pads = self._grasp_force()
                self._hold_forces.append(f)
                self._peak_force = max(self._peak_force, f)

    def _obstacle_contact(self) -> bool:
        d = self.data
        for i in range(d.ncon):
            c = d.contact[i]
            if self._obs_geom in (c.geom1, c.geom2):
                other = c.geom2 if c.geom1 == self._obs_geom else c.geom1
                if other in self._arm_geoms or other in self._finger_geoms:
                    return True
        return False

    def _grasp_force(self):
        f6 = np.zeros(6)
        total, pads = 0.0, set()
        d = self.data
        for i in range(d.ncon):
            c = d.contact[i]
            if self._box_geom not in (c.geom1, c.geom2):
                continue
            other = c.geom2 if c.geom1 == self._box_geom else c.geom1
            if other in self._finger_geoms:
                mujoco.mj_contactForce(self.model, d, i, f6)
                total += abs(float(f6[0]))
                pads.add(other)
        return total, len(pads)

    def _box_pos(self):
        return [float(v) for v in self.data.xpos[self._box_body]]

    def _tip_pos(self):
        return np.array(self.data.site_xpos[self._grip_site], dtype=float)

    # ---------------------------------------------------------------- skill
    def push_box(self, params: dict | None = None) -> PickResult:
        name, key, scene = resolve_scene(params)
        t0 = time.perf_counter()
        self._build(scene)
        self._budget = scene["budget"]
        start_pos = self._box_pos()
        grasp_state, stage = "open", "home"

        def report(success, reason, note=""):
            hold = (sum(self._hold_forces) / len(self._hold_forces)
                    if self._hold_forces else 0.0)
            return PickResult(success, reason, build_metrics(
                engine=ENGINE, robotId=ROBOT_ID, skillId=SKILL_ID,
                obj=name, scene_key=key, stage=stage,
                grasp_state=grasp_state, start_pos=start_pos,
                end_pos=self._box_pos(), hold_force=hold,
                peak_force=self._peak_force,
                contact_samples=self._contact_samples,
                collisions=self._collisions, steps=self._steps,
                budget=self._budget, wall_time=time.perf_counter() - t0,
                note=note))

        target = np.array([scene["box"][0], scene["box"][1], CUBE_HALF])
        planar = float(np.hypot(target[0], target[1]))

        try:
            if planar > WORK_R + 0.02:
                stage = "stretch"
                self._run(KEYFRAMES["stretch"], STAGE_STEPS["approach"],
                          FINGER_OPEN, abort_on_collision=False)
                gap = float(np.linalg.norm(self._tip_pos() - target))
                if gap > UNREACHABLE_GAP:
                    return report(False, "unreachable",
                                  f"tip stopped {gap:.3f} m short of the box")

            # -- approach behind the box at box-centre height, gripper open
            stage = "approach"
            if not self._run(KEYFRAMES["push_start"], STAGE_STEPS["approach"], FINGER_OPEN):
                return report(False, "collision", "obstacle struck during approach")

            # -- close the gripper around the box (contact-gated clamp)
            stage = "close"
            n = STAGE_STEPS["close"]
            for i in range(1, n + 1):
                self._tick(dict(self._pose), aperture_at(i / n))
                if self._collisions:
                    return report(False, "collision", "obstacle struck while closing")
                f, _pads = self._grasp_force()
                if f > 0.0:
                    self._contact_samples += 1
                self._peak_force = max(self._peak_force, f)

            # -- push: translate the box forward by sustained pad contact.
            #    The equality grasp constraint is intentionally NEVER engaged,
            #    so the box stays a free body and is moved only by friction.
            stage = "push"
            if not self._run(KEYFRAMES["push_end"], STAGE_STEPS["push"], FINGER_CLOSED):
                return report(False, "collision", "obstacle struck during push")

            # -- settle: prove the box rests in the target zone on its own
            stage = "settle"
            self._hold(STAGE_STEPS["settle"], FINGER_CLOSED, sample=True)

        except BudgetExhausted:
            return report(False, "timeout",
                          f"step budget {self._budget} exhausted in stage {stage}")

        end = self._box_pos()
        dx = end[0] - TARGET_ZONE_XY[0]
        dy = end[1] - TARGET_ZONE_XY[1]
        # The box must remain on the floor (no lift) and land in the zone.
        if end[2] > CUBE_HALF + 0.02:
            grasp_state = "lifted"
            return report(False, "offtarget", f"box left the floor (z={end[2]:.3f})")
        if abs(dx) > PUSH_TOL or abs(dy) > PUSH_TOL:
            grasp_state = "short"
            return report(False, "offtarget",
                          f"box off target by ({dx:+.3f},{dy:+.3f}) m")
        return report(True, "pushed",
                      f"box pushed to ({end[0]:.3f},{end[1]:.3f})")


__all__ = ["MuJoCoSimulator", "PickResult", "KEYFRAMES", "SCENES", "ENGINE"]
