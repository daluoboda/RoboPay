"""sort-arm-001 — MuJoCo backend for object routing service (Tier 1).

Paid robot service: routes an incoming object to a client-specified destination bin.
The controller accepts a `target_bin` parameter ("A" or "B") and executes a
full pick → lift → route → release → verify pipeline. The payment is settled
only after the object is confirmed at the correct bin.

Key differentiator from pick_object and pick_and_stack:
  * Destination is PARAMETER-DRIVEN (client chooses bin A or B), not hardcoded.
  * Metrics track routing correctness and placement accuracy per bin.

Public surface:
    MuJoCoSimulator().pick_and_sort(params) -> PickResult(success, reason, metrics)
"""
from __future__ import annotations

import time
import mujoco
import numpy as np

from arm_spec import (
    ARM_JOINTS, BudgetExhausted, CUBE_FRICTION, CUBE_HALF, CUBE_MASS,
    FINGER_CLOSED, FINGER_HALF_X, FINGER_HALF_Z, FINGER_OPEN, GRASP_FORCE_MIN,
    GRASP_WZ, GRIP_MID, KEYFRAMES, LINK1, LINK2, OBSTACLE_HALF_H,
    OBSTACLE_RADIUS, PAD_HALF, PickResult, STAGE_STEPS, TIMESTEP,
    UNREACHABLE_GAP, WORK_R, aperture_at, blend, build_metrics, solve,
)

ENGINE = "mujoco"

# Bin positions on table (destination targets for routing)
BIN_A_XY = (0.22, 0.10)  # bin A
BIN_B_XY = (0.22, -0.10)  # bin B
INCOMING_XY = (0.35, 0.0)  # default incoming object position

SORT_STEPS = {"route": 100, "release": 30, "verify": 50}


# ------------------------------------------------------------------- model --
def _model_xml(cube_xy, obstacle_xy) -> str:
    """Single-cube MJCF with object routing bins (visual markers only)."""
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
    # Bin visual markers (non-physical, just visual targets)
    bax, bay = BIN_A_XY
    bbx, bby = BIN_B_XY
    bin_markers = f"""
    <!-- bin A marker (blue) -->
    <body name="bin_a" pos="{bax} {bay} 0.001">
      <geom name="bin_a_marker" type="cylinder" size="0.035 0.003" pos="0 0 0.001"
            rgba="0.20 0.40 0.90 0.6" contype="0" conaffinity="0"/>
      <site name="bin_a_site" pos="0 0 0.03" size="0.008" rgba="0.2 0.4 0.9 0.4"/>
    </body>
    <!-- bin B marker (orange) -->
    <body name="bin_b" pos="{bbx} {bby} 0.001">
      <geom name="bin_b_marker" type="cylinder" size="0.035 0.003" pos="0 0 0.001"
            rgba="0.90 0.55 0.20 0.6" contype="0" conaffinity="0"/>
      <site name="bin_b_site" pos="0 0 0.03" size="0.008" rgba="0.9 0.5 0.2 0.4"/>
    </body>"""

    return f"""
<mujoco model="sort-arm-001">
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

    {bin_markers}

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

    <!-- incoming object -->
    <body name="cube" pos="{cx} {cy} {CUBE_HALF}">
      <freejoint name="cube_free"/>
      <geom name="cube_g" type="box" size="{CUBE_HALF} {CUBE_HALF} {CUBE_HALF}"
            mass="{CUBE_MASS}" rgba="0.60 0.60 0.60 1"
            contype="2" conaffinity="13" friction="{CUBE_FRICTION} 0.05 0.001"
            solref="0.02 1" solimp="0.90 0.95 0.001"/>
      <site name="cube_site" pos="0 0 0" size="0.006" rgba="0.6 0.9 0.6 0.4"/>
    </body>{obstacle}
  </worldbody>

  <equality>
    <connect name="grasp" site1="cube_site" site2="grip_site" active="false"/>
  </equality>
</mujoco>
"""


# --------------------------------------------------------------- simulator --
class MuJoCoSimulator:
    """Paid object routing service. pick_and_sort(params) routes the incoming
    object to the client-requested bin (A or B), verifies placement, and
    returns routing accuracy metrics."""

    ROBOT_ID = "sort-arm-001"
    SKILL_ID = "pick_and_sort"
    ENGINE = ENGINE

    def __init__(self):
        self.model = None
        self.data = None
        self._steps = 0
        self._budget = 350

    # ---------------------------------------------------------- scene setup
    def _build(self, scene: dict):
        xml = _model_xml(
            scene.get("cube", list(INCOMING_XY)),
            scene.get("obstacle"),
        )
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        m = self.model

        def gid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
        def bid(n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)

        self._qadr, self._vadr = {}, {}
        for name in ARM_JOINTS + ("grip_l", "grip_r"):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            self._qadr[name] = m.jnt_qposadr[jid]
            self._vadr[name] = m.jnt_dofadr[jid]

        self._cube_geom = gid("cube_g")
        self._cube_body = bid("cube")
        self._bin_a_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "bin_a_site")
        self._bin_b_site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "bin_b_site")
        self._finger_geoms = {gid("finger_l_g"), gid("finger_r_g")}
        self._obs_geom = gid("obstacle_g") if scene.get("obstacle") else -1
        self._arm_geoms = {gid(n) for n in ("base_g", "column_g", "upper_g", "fore_g", "wrist_g")}
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

    # ---- trajectory helpers ----
    def _apply(self, pose, grip):
        d = self.data
        for name in ARM_JOINTS:
            d.qpos[self._qadr[name]] = pose[name]
            d.qvel[self._vadr[name]] = 0.0
        for name in ("grip_l", "grip_r"):
            d.qpos[self._qadr[name]] = grip
            d.qvel[self._vadr[name]] = 0.0

    def _tick(self, pose, grip):
        if self._steps >= self._budget:
            raise BudgetExhausted
        self._apply(pose, grip)
        mujoco.mj_step(self.model, self.data)
        self._apply(pose, grip)
        self._steps += 1
        self._pose, self._grip = pose, grip
        if self._obs_geom >= 0 and self._obstacle_contact():
            self._collisions += 1

    def _run(self, target, n, grip, abort_on_collision=True):
        start = dict(self._pose)
        for i in range(1, n + 1):
            self._tick(blend(start, target, i / n), grip)
            if abort_on_collision and self._collisions:
                return False
        return True

    def _hold(self, n, grip, sample=False):
        for _ in range(n):
            self._tick(dict(self._pose), grip)
            if sample:
                f, _ = self._grasp_force()
                self._hold_forces.append(f)
                self._peak_force = max(self._peak_force, f)

    # ---- sensing ----
    def _obstacle_contact(self):
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

    def _bin_pos(self, bin_id):
        sid = self._bin_a_site if bin_id == "A" else self._bin_b_site
        return np.array(self.data.site_xpos[sid], dtype=float)

    def _tip_pos(self):
        return np.array(self.data.site_xpos[self._grip_site], dtype=float)

    def _attach(self):
        try:
            self.data.eq_active[self._eq_id] = 1
        except AttributeError:
            self.model.eq_active[self._eq_id] = 1
        mujoco.mj_forward(self.model, self.data)

    def _detach(self):
        try:
            self.data.eq_active[self._eq_id] = 0
        except AttributeError:
            self.model.eq_active[self._eq_id] = 0
        mujoco.mj_forward(self.model, self.data)

    # ---------------------------------------------------- pick_and_sort skill
    def pick_and_sort(self, params: dict | None = None) -> PickResult:
        """Object routing service: pick incoming object → route to target bin.
        
        params.target_bin: "A" or "B" — client's routing decision.
        
        Phases: move_above → descend → grip → lift → route_to_bin → release → verify
        """
        p = params or {}
        target_bin = p.get("target_bin", "A")
        if target_bin not in ("A", "B"):
            target_bin = "A"

        scene = {
            "cube": p.get("cube_xy", list(INCOMING_XY)),
            "obstacle": p.get("obstacle_xy"),
        }

        t0 = time.perf_counter()
        self._build(scene)
        self._budget = p.get("budget", 450)
        start_pos = self._cube_pos()
        bin_target = self._bin_pos(target_bin)
        grasp_state, stage = "open", "home"
        # Highest the payload ever rises above its start height. A routing run
        # ends with the object back on the table, so objectLifted is ~0 by
        # design -- peakLift is the metric that proves the arm actually carried
        # it through the air instead of dragging it across the surface.
        self._peak_lift = 0.0

        def report(success, reason, note=""):
            end_pos = self._cube_pos()
            end_xy = np.array([end_pos[0], end_pos[1]])
            bin_xy = np.array([bin_target[0], bin_target[1]])
            accuracy = float(np.linalg.norm(end_xy - bin_xy))
            routed = success and accuracy < 0.07  # within 7cm of bin center
            return PickResult(success, reason, build_metrics(
                engine=ENGINE, obj="route", scene_key=f"to_bin_{target_bin}",
                stage=stage, grasp_state=grasp_state,
                start_pos=start_pos, end_pos=end_pos,
                hold_force=(sum(self._hold_forces) / len(self._hold_forces)
                            if self._hold_forces else 0.0),
                peak_force=self._peak_force,
                contact_samples=self._contact_samples,
                collisions=self._collisions, steps=self._steps,
                budget=self._budget, wall_time=time.perf_counter() - t0,
                note=f"{note} | target_bin={target_bin} routed={routed} "
                     f"accuracy={accuracy:.4f}m",
                extra={
                    "targetBin": target_bin,
                    "routed": bool(routed),
                    "accuracy": round(accuracy, 4),
                    "peakLift": round(float(self._peak_lift), 4),
                }))

        cube_xy = np.array([start_pos[0], start_pos[1]])
        planar = float(np.hypot(cube_xy[0], cube_xy[1]))

        try:
            # Envelope check
            if planar > WORK_R + 0.02:
                stage = "stretch"
                return report(False, "unreachable", "object out of workspace")

            # 1/7 MOVE_ABOVE
            stage = "move_above"
            if not self._run(KEYFRAMES["above"], STAGE_STEPS["move_above"], FINGER_OPEN):
                return report(False, "collision", "obstacle during approach")

            # 2/7 DESCEND
            stage = "descend"
            if not self._run(KEYFRAMES["grasp"], STAGE_STEPS["descend"], FINGER_OPEN):
                return report(False, "collision", "obstacle during descent")

            # 3/7 GRIP — contact-gated
            stage = "grip"
            n = STAGE_STEPS["grip"]
            for i in range(1, n + 1):
                self._tick(dict(self._pose), aperture_at(i / n))
                if self._collisions:
                    return report(False, "collision", "obstacle during grip")
                f, _ = self._grasp_force()
                if f > 0.0:
                    self._contact_samples += 1
                self._peak_force = max(self._peak_force, f)

            force, pads = self._grasp_force()
            self._peak_force = max(self._peak_force, force)
            if self._peak_force < GRASP_FORCE_MIN or pads < 2:
                grasp_state = "slipped"
                return report(False, "grasp_failed",
                              f"pads={pads} force={self._peak_force:.3f}N")
            self._attach()
            grasp_state = "attached"

            # 4/7 LIFT
            stage = "lift"
            if not self._run(KEYFRAMES["lift"], STAGE_STEPS["lift"], FINGER_CLOSED):
                return report(False, "collision", "obstacle during lift")
            self._peak_lift = max(self._peak_lift,
                                  self._cube_pos()[2] - start_pos[2])

            # Compute custom keyframes for target bin
            bin_xy = np.array([bin_target[0], bin_target[1]])
            r_bin = float(np.hypot(bin_xy[0], bin_xy[1]))
            pan_bin = float(np.arctan2(bin_xy[1], bin_xy[0]))
            route_above = solve(r_bin, GRASP_WZ + 0.14, pan_bin)
            route_at = solve(r_bin, GRASP_WZ, pan_bin)

            # 5/7 ROUTE_TO_BIN
            stage = "route_to_bin"
            if not self._run(route_above, SORT_STEPS["route"], FINGER_CLOSED):
                return report(False, "collision", "obstacle during route")
            self._peak_lift = max(self._peak_lift,
                                  self._cube_pos()[2] - start_pos[2])

            # 6/7 RELEASE at bin
            stage = "release"
            if not self._run(route_at, SORT_STEPS["release"], FINGER_CLOSED):
                return report(False, "collision", "obstacle during release")

            grasp_state = "released"
            self._detach()
            for _ in range(10):
                self._tick(dict(self._pose), FINGER_OPEN)

            # 7/7 VERIFY placement accuracy
            stage = "verify"
            self._hold(SORT_STEPS["verify"], FINGER_OPEN, sample=False)

        except BudgetExhausted:
            return report(False, "timeout",
                          f"budget {self._budget} exhausted in {stage}")

        # Routing accuracy check
        end_pos = self._cube_pos()
        end_xy = np.array([end_pos[0], end_pos[1]])
        bin_xy = np.array([bin_target[0], bin_target[1]])
        accuracy = float(np.linalg.norm(end_xy - bin_xy))

        if accuracy > 0.07:
            return report(False, "misrouted",
                          f"object {accuracy:.3f}m from bin {target_bin} center")

        return report(True, "routed",
                      f"object routed to bin {target_bin}, accuracy {accuracy:.3f}m")


__all__ = ["MuJoCoSimulator", "PickResult", "ENGINE"]
