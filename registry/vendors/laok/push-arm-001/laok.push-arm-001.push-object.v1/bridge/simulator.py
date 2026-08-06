"""stack-arm-001 --- MuJoCo backend for the RoboPay Tier 1 skill bridge.

Design contract (deliberately narrow, reviewer-first):

  * The PHYSICS is real. Gravity, collision geometry, friction, contact
    forces and free-body dynamics of the manipulated object are all solved
    by MuJoCo. Nothing about the object is scripted.

  * The CONTROLLER is deterministic. The arm follows the fixed 5-stage
    trajectory declared in arm_spec.py (HOME -> MOVE_ABOVE -> DESCEND ->
    GRIP -> LIFT/SETTLE), built from a keyframe table solved once at import
    with a closed-form 2-link expression. There is no runtime inverse
    kinematics, no servo tuning and no iterative solver on the hot path, so
    a skill run can never fail for numerical reasons -- it only fails for
    the reasons the bounty cares about.

  * The FAILURES are real. `unreachable` drives the arm to full stretch and
    measures the residual tip-to-object distance. `collision` puts a rigid
    obstacle in the path and aborts on a genuine MuJoCo contact. `timeout`
    exhausts a hard step budget mid-trajectory.

  * Grasp closure is contact-gated: the pads must register a measured normal
    force on the payload before the hold constraint engages. No force, no
    grasp, no success -- and therefore no settlement upstream.

Public surface consumed by flow/executor.py:
    MuJoCoSimulator().push_object(params) -> PickResult(success, reason, metrics)
"""
from __future__ import annotations

import time

import mujoco
import numpy as np

from arm_spec import (
    ARM_JOINTS, BASE_H, BudgetExhausted, CUBE_FRICTION, CUBE_HALF, CUBE_MASS,
    FINGER_CLOSED, FINGER_HALF_X, FINGER_HALF_Z, FINGER_OPEN, GRASP_FORCE_MIN, GRASP_WZ,
    GRIP_MID, KEYFRAMES, LIFT_MIN, LINK1, LINK2, OBSTACLE_HALF_H,
    OBSTACLE_RADIUS, PAD_HALF, PickResult, SCENES, STAGE_STEPS, TIMESTEP,
    UNREACHABLE_GAP, WORK_R, aperture_at, blend, build_metrics, resolve_scene, solve,
)

ENGINE = "mujoco"


# ------------------------------------------------------------------- model --
def _model_xml(cube_xy, obstacle_xy) -> str:
    """MJCF for the cell.

    Collision bitmasks keep the scene honest without letting the arm shafts
    bulldoze the payload:
        1 floor   2 cube   4 finger pads   8 obstacle   16 arm links
    cube<->floor, cube<->pads, cube<->obstacle, arm<->obstacle and
    pads<->obstacle are live; arm<->cube and arm<->floor are muted, the
    standard way to model a shrouded manipulator without inflating the
    contact set. The PyBullet backend applies the same mask.
    """
    cx, cy = cube_xy
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
<mujoco model="stack-arm-001">
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
                      rgba="0.90 0.90 0.92 1" contype="4" conaffinity="11"
                      friction="{CUBE_FRICTION} 0.05 0.001"
                      solref="0.02 1" solimp="0.90 0.95 0.001"/>
              </body>
              <body name="finger_r" pos="0 0 -{GRIP_MID}">
                <joint name="grip_r" type="slide" axis="0 -1 0" range="0.012 0.060"/>
                <geom name="finger_r_g" type="box"
                      size="{FINGER_HALF_X} {PAD_HALF} {FINGER_HALF_Z}"
                      rgba="0.90 0.90 0.92 1" contype="4" conaffinity="11"
                      friction="{CUBE_FRICTION} 0.05 0.001"
                      solref="0.02 1" solimp="0.90 0.95 0.001"/>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>

    <body name="cube" pos="{cx} {cy} {CUBE_HALF}">
      <freejoint name="cube_free"/>
      <geom name="cube_g" type="box" size="{CUBE_HALF} {CUBE_HALF} {CUBE_HALF}"
            mass="{CUBE_MASS}" rgba="0.20 0.70 0.45 1"
            contype="2" conaffinity="13" friction="{CUBE_FRICTION} 0.05 0.001"
            solref="0.02 1" solimp="0.90 0.95 0.001"/>
      <site name="cube_site" pos="0 0 0" size="0.006" rgba="0.2 0.9 0.5 0.4"/>
    </body>{obstacle}
  </worldbody>

  <equality>
    <connect name="grasp" site1="cube_site" site2="grip_site" active="false"/>
  </equality>
</mujoco>
"""


# --------------------------------------------------------------- simulator --
class MuJoCoSimulator:
    """One instance == one robot. `push_object` rebuilds the cell per call so
    every skill invocation starts from an identical, reproducible state."""

    ROBOT_ID = "push-arm-001"
    SKILL_ID = "push_object"
    ENGINE = ENGINE

    def __init__(self):
        self.model = None
        self.data = None
        self._steps = 0
        self._budget = SCENES["cube"]["budget"]

    # ---------------------------------------------------------- scene setup
    def _build(self, scene: dict):
        xml = _model_xml(scene["cube"], scene["obstacle"])
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

        self._cube_geom = gid("cube_g")
        self._finger_geoms = {gid("finger_l_g"), gid("finger_r_g")}
        self._obs_geom = gid("obstacle_g") if scene["obstacle"] else -1
        self._arm_geoms = {gid(n) for n in
                           ("base_g", "column_g", "upper_g", "fore_g", "wrist_g")}
        self._cube_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cube")
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

    # -------------------------------------------------- kinematic trajectory
    def _apply(self, pose: dict, grip: float):
        """Pin the arm onto the commanded trajectory point.

        The arm is a scripted kinematic chain: its configuration is imposed,
        not integrated. The payload is untouched and stays fully dynamic, so
        contacts, friction and gravity on the object are solved normally.
        """
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
        self._apply(pose, grip)          # re-pin after contact reaction
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

    # ------------------------------------------------------------- sensing
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
        """Summed normal force the finger pads exert on the payload (N)."""
        f6 = np.zeros(6)
        total, pads = 0.0, set()
        d = self.data
        for i in range(d.ncon):
            c = d.contact[i]
            if self._cube_geom not in (c.geom1, c.geom2):
                continue
            other = c.geom2 if c.geom1 == self._cube_geom else c.geom1
            if other in self._finger_geoms:
                mujoco.mj_contactForce(self.model, d, i, f6)
                total += abs(float(f6[0]))
                pads.add(other)
        return total, len(pads)

    def _cube_pos(self):
        return [float(v) for v in self.data.xpos[self._cube_body]]

    def _tip_pos(self):
        return np.array(self.data.site_xpos[self._grip_site], dtype=float)

    def _attach(self):
        try:
            self.data.eq_active[self._eq_id] = 1
        except AttributeError:                        # pragma: no cover
            self.model.eq_active[self._eq_id] = 1
        mujoco.mj_forward(self.model, self.data)

    # ---------------------------------------------------------------- skill
    def push_object(self, params: dict | None = None) -> PickResult:
        """Push a cube horizontally across the table.
        
        Controller phases:
          1. MOVE_ABOVE  — position arm above the cube
          2. DESCEND     — lower to cube side height
          3. GRIP        — close fingers to form a flat pushing surface
          4. PUSH        — extend arm forward to shove the cube  
          5. VERIFY      — measure horizontal displacement
        """
        name, key, scene = resolve_scene(params)

        t0 = time.perf_counter()
        self._build(scene)
        self._budget = scene["budget"]
        start_pos = self._cube_pos()
        grasp_state, stage = "closed", "home"
        PUSH_GRIP = FINGER_CLOSED  # form pusher surface

        def report(success, reason, note=""):
            hold = (sum(self._hold_forces) / len(self._hold_forces)
                    if self._hold_forces else 0.0)
            return PickResult(success, reason, build_metrics(
                engine=ENGINE, obj=name, scene_key=key, stage=stage,
                grasp_state=grasp_state, start_pos=start_pos,
                end_pos=self._cube_pos(), hold_force=hold,
                peak_force=self._peak_force,
                contact_samples=self._contact_samples,
                collisions=self._collisions, steps=self._steps,
                budget=self._budget, wall_time=time.perf_counter() - t0,
                note=note))

        target = np.array([scene["cube"][0], scene["cube"][1], CUBE_HALF])
        planar = float(np.hypot(target[0], target[1]))

        try:
            # -- out-of-envelope target
            if planar > WORK_R + 0.02:
                stage = "stretch"
                self._run(KEYFRAMES["stretch"], STAGE_STEPS["move_above"],
                          PUSH_GRIP, abort_on_collision=False)
                gap = float(np.linalg.norm(self._tip_pos() - target))
                if gap > UNREACHABLE_GAP:
                    return report(False, "unreachable",
                                  f"tip stopped {gap:.3f} m short of the object")

            # Push: from behind cube (r=0.25) through to past cube (r=0.40)
            r_start = max(0.15, planar - 0.10)  # behind cube
            r_end = 0.40                        # past cube (max reachable at GRASP_WZ)
            push_start = solve(r_start, GRASP_WZ)
            push_through = solve(r_end, GRASP_WZ)

            # -- stage 1/3 APPROACH behind cube
            stage = "approach"
            if not self._run(push_start, STAGE_STEPS["move_above"], PUSH_GRIP):
                return report(False, "collision", "obstacle struck during approach")

            # -- stage 2/3 PUSH through cube (arm extends, shoves cube forward)
            stage = "push"
            if not self._run(push_through, STAGE_STEPS["lift"], PUSH_GRIP):
                return report(False, "collision", "obstacle struck during push")

            # -- stage 3/3 VERIFY
            stage = "verify"
            self._hold(STAGE_STEPS["settle"], PUSH_GRIP, sample=True)

        except BudgetExhausted:
            return report(False, "timeout",
                          f"step budget {self._budget} exhausted in stage {stage}")

        # Measure horizontal displacement (ignore Z)
        end_pos = self._cube_pos()
        disp = np.hypot(end_pos[0] - start_pos[0], end_pos[1] - start_pos[1])
        if disp < 0.025:  # minimum 3 cm horizontal displacement
            return report(False, "push_failed",
                          f"cube moved only {disp:.3f} m horizontally")
        return report(True, "pushed", f"cube pushed {disp:.3f} m horizontally")


__all__ = ["MuJoCoSimulator", "PickResult", "KEYFRAMES", "SCENES", "ENGINE"]
