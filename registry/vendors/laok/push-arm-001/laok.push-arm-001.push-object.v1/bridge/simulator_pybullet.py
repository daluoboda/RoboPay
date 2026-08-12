"""push-arm-001 --- PyBullet backend (sim-to-sim cross-check).

Same robot, same skill, same trajectory, different physics engine.

Everything that defines the robot and the skill -- link lengths, gripper
geometry, the push keyframes, stage step counts, the scene table and every
pass/fail threshold -- is imported from arm_spec.py, exactly as the MuJoCo
backend does. The only thing that differs below is how the world is assembled
and stepped. That is what makes the sim-to-sim test meaningful: if both
engines agree on the verdict, the failure reason, the grasp state and the
displacement the payload kept, then `push_object` is a property of the robot
definition, not of one simulator's quirks.

Nothing here is a re-derivation of the skill. The controller phases
(APPROACH -> PUSH -> VERIFY), the constant-rate blade stroke, the velocity
gated rest detector and the four success gates are structurally identical to
simulator.MuJoCoSimulator.push_object -- read them side by side.

Two engine-parameterisation notes, because they are the only places where the
same physics has to be spelled differently:

  * FRICTION. MuJoCo takes the element-wise MAXIMUM of the two geoms'
    coefficients for a contact pair; Bullet takes the PRODUCT of the two
    bodies' lateralFriction. The MJCF gives the payload, the pads and the
    floor mu = CUBE_FRICTION / CUBE_FRICTION / 1.0, i.e. a pair coefficient of
    CUBE_FRICTION everywhere it matters. PAD_FRICTION and FLOOR_FRICTION below
    are chosen so Bullet's product lands on that same pair coefficient instead
    of squaring it.

  * DAMPING. MuJoCo's <freejoint> is explicitly exempt from the <joint>
    defaults, so the payload has zero linear/angular damping. Bullet applies
    0.04 of each by default, so both are zeroed here. Sleeping is disabled for
    the same reason: MuJoCo has no such mechanism and the rest detector must
    read real velocities, not a deactivated body's zeros.

PyBullet ships as a source distribution only, so it builds on Linux CI but
usually not on a bare Windows box. Import is lazy and every consumer is
expected to skip when `available()` is False.

Public surface (identical to simulator.MuJoCoSimulator):
    PyBulletSimulator().push_object(params) -> PickResult
"""
from __future__ import annotations

import math
import os
import tempfile
import time

from arm_spec import (
    AIRBORNE_MAX, ARM_JOINTS, BudgetExhausted, CUBE_FRICTION, CUBE_HALF,
    CUBE_MASS, FINGER_HALF_X, FINGER_HALF_Z, FINGER_OPEN, GRIP_MID, KEYFRAMES,
    LINK1, LINK2, OBSTACLE_HALF_H, OBSTACLE_RADIUS, PAD_HALF, PUSH_CONTACT_MIN,
    PUSH_GRIP, PUSH_MIN, PUSH_PEAK_MAX, PUSH_R_END, PUSH_STANDOFF, PUSH_STEPS,
    PUSH_WZ, PickResult, SCENES, SETTLE_ANG_EPS, SETTLE_LIN_EPS,
    SETTLE_MAX_STEPS, SETTLE_QUIET_STEPS, STAGE_STEPS, TIMESTEP,
    UNREACHABLE_GAP, WORK_R, blend, build_metrics, ramp, resolve_scene, solve,
)

ENGINE = "pybullet"

# Collision groups mirror the MJCF contype/conaffinity bitmasks in
# simulator.py exactly:  1 floor  2 cube  4 pads  8 obstacle  16 arm links.
# MuJoCo pairs geoms on (contype1 & conaffinity2) OR (contype2 & conaffinity1);
# Bullet uses AND of the two directions. The table below is symmetric -- every
# pair is either enabled both ways or disabled both ways -- so the two rules
# select the identical contact set.
G_FLOOR, M_FLOOR = 1, 6
G_CUBE, M_CUBE = 2, 13
G_PAD, M_PAD = 4, 11
G_OBSTACLE, M_OBSTACLE = 8, 22
G_ARM, M_ARM = 16, 8

# See the FRICTION note in the module docstring.
PAD_FRICTION = 1.0      # * CUBE_FRICTION == the MJCF pad<->payload pair mu
FLOOR_FRICTION = 1.0    # MJCF: floor friction="1.0 0.01 0.001"

# Bullet's soft-contact parameters, matched to the MJCF solref/solimp regime:
# a 0.1 kg payload settles at m*g/k ~= 0.12 mm of penetration, an order of
# magnitude below the 0.20 mm of blade advance per push step, so the stroke
# always resolves as a shove and never as a penetration pop.
CONTACT_STIFFNESS = 8000.0
CONTACT_DAMPING = 80.0

_GRIP_JOINTS = ("grip_l", "grip_r")


def available() -> bool:
    """True when the PyBullet wheel is importable in this environment."""
    try:
        import pybullet  # noqa: F401
    except Exception:
        return False
    return True


# --------------------------------------------------------------------- URDF --
def _inertial(mass: float) -> str:
    i = max(1e-5, mass * 0.01)
    return (f'<inertial><mass value="{mass}"/>'
            f'<inertia ixx="{i}" ixy="0" ixz="0" iyy="{i}" iyz="0" izz="{i}"/>'
            f'</inertial>')


def _cyl_link(name, length, radius, mass, rgba, along_x=False) -> str:
    """Capsule-ish link. URDF cylinders lie along +Z, so links that run along
    the arm's +X axis are rotated by pi/2 about +Y, matching the MJCF fromto."""
    rpy = "0 1.5707963 0" if along_x else "0 0 0"
    off = f'{length / 2} 0 0' if along_x else f'0 0 {length / 2}'
    geom = f'<cylinder length="{length}" radius="{radius}"/>'
    return f"""
  <link name="{name}">
    {_inertial(mass)}
    <visual><origin xyz="{off}" rpy="{rpy}"/><geometry>{geom}</geometry>
      <material name="{name}_m"><color rgba="{rgba}"/></material></visual>
    <collision><origin xyz="{off}" rpy="{rpy}"/><geometry>{geom}</geometry></collision>
  </link>"""


def _box_link(name, sx, sy, sz, mass, rgba) -> str:
    geom = f'<box size="{sx} {sy} {sz}"/>'
    return f"""
  <link name="{name}">
    {_inertial(mass)}
    <visual><geometry>{geom}</geometry>
      <material name="{name}_m"><color rgba="{rgba}"/></material></visual>
    <collision><geometry>{geom}</geometry></collision>
  </link>"""


def _joint(name, jtype, parent, child, xyz, axis, lo, hi) -> str:
    return f"""
  <joint name="{name}" type="{jtype}">
    <parent link="{parent}"/><child link="{child}"/>
    <origin xyz="{xyz}" rpy="0 0 0"/><axis xyz="{axis}"/>
    <limit lower="{lo}" upper="{hi}" effort="200" velocity="10"/>
  </joint>"""


def _robot_urdf() -> str:
    """The same kinematic chain the MJCF declares, in URDF form.

    Every dimension is read from arm_spec, so a link length can only be
    changed in one place -- which is precisely what TestSpecIsSingleSource
    asserts on machines where PyBullet itself cannot be built.
    """
    return f"""<?xml version="1.0"?>
<robot name="push-arm-001">
  <link name="base">
    {_inertial(1.0)}
    <visual><origin xyz="0 0 0.025"/><geometry><cylinder length="0.05" radius="0.07"/></geometry>
      <material name="base_m"><color rgba="0.25 0.27 0.32 1"/></material></visual>
    <collision><origin xyz="0 0 0.025"/><geometry><cylinder length="0.05" radius="0.07"/></geometry></collision>
  </link>
{_cyl_link("column", 0.35, 0.035, 1.0, "0.30 0.32 0.38 1")}
{_cyl_link("upper", LINK1, 0.030, 0.8, "0.85 0.55 0.18 1", along_x=True)}
{_cyl_link("fore", LINK2, 0.026, 0.6, "0.85 0.55 0.18 1", along_x=True)}
{_box_link("wrist", 0.064, 0.060, 0.036, 0.3, "0.30 0.32 0.38 1")}
{_box_link("finger_l", 2 * FINGER_HALF_X, 2 * PAD_HALF, 2 * FINGER_HALF_Z, 0.05,
           "0.90 0.90 0.92 1")}
{_box_link("finger_r", 2 * FINGER_HALF_X, 2 * PAD_HALF, 2 * FINGER_HALF_Z, 0.05,
           "0.90 0.90 0.92 1")}
{_joint("pan", "revolute", "base", "column", "0 0 0.05", "0 0 1", -3.1416, 3.1416)}
{_joint("shoulder", "revolute", "column", "upper", "0 0 0.35", "0 1 0", -2.0, 2.0)}
{_joint("elbow", "revolute", "upper", "fore", f"{LINK1} 0 0", "0 1 0", -2.6, 2.6)}
{_joint("wristp", "revolute", "fore", "wrist", f"{LINK2} 0 0", "0 1 0", -2.8, 2.8)}
{_joint("grip_l", "prismatic", "wrist", "finger_l", f"0 0 -{GRIP_MID}", "0 1 0", 0.012, 0.060)}
{_joint("grip_r", "prismatic", "wrist", "finger_r", f"0 0 -{GRIP_MID}", "0 -1 0", 0.012, 0.060)}
</robot>
"""


# --------------------------------------------------------------- simulator --
class PyBulletSimulator:
    """Drop-in twin of MuJoCoSimulator running on Bullet."""

    ROBOT_ID = "push-arm-001"
    SKILL_ID = "push_object"
    ENGINE = ENGINE

    def __init__(self):
        if not available():                           # pragma: no cover
            raise RuntimeError("pybullet is not installed in this environment")
        import pybullet
        self._p = pybullet
        self._cid = None
        self._urdf_path = None
        self._steps = 0
        self._budget = SCENES["cube"]["budget"]

    # ---------------------------------------------------------- scene setup
    def _build(self, scene: dict):
        p = self._p
        self._teardown()
        self._cid = p.connect(p.DIRECT)
        c = self._cid
        p.setGravity(0, 0, -9.81, physicsClientId=c)
        p.setTimeStep(TIMESTEP, physicsClientId=c)
        p.setPhysicsEngineParameter(numSolverIterations=80, physicsClientId=c)

        # ground plane
        plane_shape = p.createCollisionShape(p.GEOM_PLANE, physicsClientId=c)
        self.floor = p.createMultiBody(0, plane_shape, physicsClientId=c)
        p.changeDynamics(self.floor, -1, lateralFriction=FLOOR_FRICTION,
                         restitution=0.0, physicsClientId=c)
        p.setCollisionFilterGroupMask(self.floor, -1, G_FLOOR, M_FLOOR,
                                      physicsClientId=c)

        # robot
        fd, path = tempfile.mkstemp(suffix=".urdf", text=True)
        with os.fdopen(fd, "w") as fh:
            fh.write(_robot_urdf())
        self._urdf_path = path
        self.robot = p.loadURDF(path, [0, 0, 0], useFixedBase=True,
                                physicsClientId=c)

        self._jidx = {}
        for j in range(p.getNumJoints(self.robot, physicsClientId=c)):
            info = p.getJointInfo(self.robot, j, physicsClientId=c)
            self._jidx[info[1].decode()] = j
            # kinematic pinning: no motor should fight the scripted pose
            p.setJointMotorControl2(self.robot, j, p.VELOCITY_CONTROL,
                                    force=0, physicsClientId=c)
        self._pad_links = {self._jidx["grip_l"], self._jidx["grip_r"]}
        self._wrist_link = self._jidx["wristp"]

        for name, j in self._jidx.items():
            pad = name in _GRIP_JOINTS
            p.setCollisionFilterGroupMask(self.robot, j,
                                          G_PAD if pad else G_ARM,
                                          M_PAD if pad else M_ARM,
                                          physicsClientId=c)
            if pad:
                p.changeDynamics(self.robot, j, lateralFriction=PAD_FRICTION,
                                 restitution=0.0,
                                 contactStiffness=CONTACT_STIFFNESS,
                                 contactDamping=CONTACT_DAMPING,
                                 physicsClientId=c)
        p.setCollisionFilterGroupMask(self.robot, -1, G_ARM, M_ARM, physicsClientId=c)

        # payload
        cx, cy = scene["cube"]
        half = [CUBE_HALF] * 3
        cshape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=c)
        vshape = p.createVisualShape(p.GEOM_BOX, halfExtents=half,
                                     rgbaColor=[0.20, 0.70, 0.45, 1],
                                     physicsClientId=c)
        self.cube = p.createMultiBody(CUBE_MASS, cshape, vshape,
                                      [cx, cy, CUBE_HALF], physicsClientId=c)
        p.changeDynamics(self.cube, -1, lateralFriction=CUBE_FRICTION,
                         restitution=0.0, linearDamping=0.0, angularDamping=0.0,
                         contactStiffness=CONTACT_STIFFNESS,
                         contactDamping=CONTACT_DAMPING,
                         activationState=4,  # ACTIVATION_STATE_DISABLE_DEACTIVATION; avoid pybullet API compat issue
                         physicsClientId=c)
        p.setCollisionFilterGroupMask(self.cube, -1, G_CUBE, M_CUBE, physicsClientId=c)

        # obstacle
        self.obstacle = None
        if scene["obstacle"] is not None:
            ox, oy = scene["obstacle"]
            oshape = p.createCollisionShape(p.GEOM_CYLINDER, radius=OBSTACLE_RADIUS,
                                            height=2 * OBSTACLE_HALF_H,
                                            physicsClientId=c)
            ovis = p.createVisualShape(p.GEOM_CYLINDER, radius=OBSTACLE_RADIUS,
                                       length=2 * OBSTACLE_HALF_H,
                                       rgbaColor=[0.80, 0.25, 0.25, 1],
                                       physicsClientId=c)
            self.obstacle = p.createMultiBody(0, oshape, ovis,
                                              [ox, oy, OBSTACLE_HALF_H],
                                              physicsClientId=c)
            p.setCollisionFilterGroupMask(self.obstacle, -1, G_OBSTACLE,
                                          M_OBSTACLE, physicsClientId=c)

        self._pose = dict(KEYFRAMES["home"])
        self._grip = FINGER_OPEN
        self._steps = 0
        self._peak_force = 0.0
        self._hold_forces = []
        self._contact_samples = 0
        self._collisions = 0
        self._constraint = None
        self._apply(self._pose, self._grip)

    def _teardown(self):
        if self._cid is not None:
            try:
                self._p.disconnect(physicsClientId=self._cid)
            except Exception:                          # pragma: no cover
                pass
            self._cid = None
        if self._urdf_path and os.path.exists(self._urdf_path):
            try:
                os.unlink(self._urdf_path)
            except OSError:                            # pragma: no cover
                pass
            self._urdf_path = None

    def __del__(self):                                 # pragma: no cover
        self._teardown()

    # -------------------------------------------------- kinematic trajectory
    def _apply(self, pose: dict, grip: float):
        """Pin the arm onto the commanded trajectory point.

        The arm is a scripted kinematic chain: its configuration is imposed,
        not integrated. The payload is untouched and stays fully dynamic, so
        contacts, friction and gravity on the object are solved normally.
        """
        p, c = self._p, self._cid
        for name in ARM_JOINTS:
            p.resetJointState(self.robot, self._jidx[name], pose[name], 0.0,
                              physicsClientId=c)
        for name in _GRIP_JOINTS:
            p.resetJointState(self.robot, self._jidx[name], grip, 0.0,
                              physicsClientId=c)

    def _tick(self, pose: dict, grip: float):
        if self._steps >= self._budget:
            raise BudgetExhausted
        self._apply(pose, grip)
        self._p.stepSimulation(physicsClientId=self._cid)
        self._apply(pose, grip)          # re-pin after contact reaction
        self._steps += 1
        self._pose, self._grip = pose, grip
        if self.obstacle is not None and self._obstacle_contact():
            self._collisions += 1

    def _run(self, target: dict, n: int, grip: float, abort_on_collision=True):
        start = dict(self._pose)
        for i in range(1, n + 1):
            self._tick(blend(start, target, i / n), grip)
            if abort_on_collision and self._collisions:
                return False
        return True

    def _hold(self, n: int, grip: float, sample: bool = False):
        """Freeze the arm for n steps. Mirrors the MuJoCo twin's surface; the
        push plan uses the velocity-gated `_settle_to_rest` instead."""
        for _ in range(n):
            self._tick(dict(self._pose), grip)
            if sample:
                f, _pads = self._grasp_force()
                self._hold_forces.append(f)
                self._peak_force = max(self._peak_force, f)

    # ------------------------------------------------------------- sensing
    def _obstacle_contact(self) -> bool:
        pts = self._p.getContactPoints(bodyA=self.robot, bodyB=self.obstacle,
                                       physicsClientId=self._cid)
        return bool(pts)

    def _grasp_force(self):
        """Summed normal force the finger pads exert on the payload (N)."""
        pts = self._p.getContactPoints(bodyA=self.robot, bodyB=self.cube,
                                       physicsClientId=self._cid)
        total, pads = 0.0, set()
        for pt in pts:
            link = pt[3]
            if link in self._pad_links:
                total += abs(float(pt[9]))     # normalForce
                pads.add(link)
        return total, len(pads)

    def _cube_pos(self):
        pos, _orn = self._p.getBasePositionAndOrientation(
            self.cube, physicsClientId=self._cid)
        return [float(v) for v in pos]

    def _cube_speed(self):
        """(linear m/s, angular rad/s) of the payload free body."""
        lin, ang = self._p.getBaseVelocity(self.cube, physicsClientId=self._cid)
        return (math.sqrt(sum(float(v) ** 2 for v in lin)),
                math.sqrt(sum(float(w) ** 2 for w in ang)))

    def _tip_pos(self):
        st = self._p.getLinkState(self.robot, self._wrist_link,
                                  computeForwardKinematics=True,
                                  physicsClientId=self._cid)
        pos, orn = st[4], st[5]
        rot = self._p.getMatrixFromQuaternion(orn)
        off = (0.0, 0.0, -GRIP_MID)
        return [pos[i] + sum(rot[3 * i + k] * off[k] for k in range(3))
                for i in range(3)]

    def _attach(self):
        """Weld the payload to the gripper.

        The Bullet twin of the MJCF's `<equality><connect name="grasp">`, which
        `push_object` leaves inactive -- a push is a shove, not a grasp. Kept so
        the two scene definitions stay feature-for-feature equivalent (and
        contract-tested, so it cannot rot).
        """
        p, c = self._p, self._cid
        self._constraint = p.createConstraint(
            self.robot, self._wrist_link, self.cube, -1,
            p.JOINT_POINT2POINT, [0, 0, 0],
            parentFramePosition=[0, 0, -GRIP_MID],
            childFramePosition=[0, 0, 0], physicsClientId=c)
        p.changeConstraint(self._constraint, maxForce=200, physicsClientId=c)

    def _sweep(self, target: dict, n: int, grip: float):
        """Constant-rate blade stroke, sampling the payload contact as it goes.

        `ramp` rather than `blend`: smoothstep's mid-stroke rate is 1.5x its
        mean and that spike is what launches the payload instead of shoving it.
        Contact force is recorded HERE, during the stroke, because that is the
        only window in which the blade is actually loading the payload.
        """
        start = dict(self._pose)
        peak_z = self._cube_pos()[2]
        for i in range(1, n + 1):
            self._tick(ramp(start, target, i / n), grip)
            f, pads = self._grasp_force()
            if pads:
                self._contact_samples += 1
                self._hold_forces.append(f)
                self._peak_force = max(self._peak_force, f)
            peak_z = max(peak_z, self._cube_pos()[2])
            if self._collisions:
                return False, peak_z
        return True, peak_z

    def _settle_to_rest(self, grip: float):
        """Hold the arm still until the shoved payload has actually stopped.

        Rest is gated on VELOCITY, not on per-step displacement: a payload at
        the apex of a ballistic arc is momentarily slow, and a displacement
        gate would certify it "at rest" in mid-air.

        Returns (settled, steps_waited, peak_z_seen).
        """
        quiet = 0
        peak_z = self._cube_pos()[2]
        for i in range(1, SETTLE_MAX_STEPS + 1):
            self._tick(dict(self._pose), grip)
            peak_z = max(peak_z, self._cube_pos()[2])
            lin, ang = self._cube_speed()
            quiet = quiet + 1 if (lin < SETTLE_LIN_EPS and ang < SETTLE_ANG_EPS) else 0
            if quiet >= SETTLE_QUIET_STEPS:
                return True, i, peak_z
        return False, SETTLE_MAX_STEPS, peak_z

    # ---------------------------------------------------------------- skill
    def push_object(self, params: dict | None = None) -> PickResult:
        """Shove a payload horizontally across the table.

        Phase for phase the MuJoCo twin: APPROACH the closed blade to
        PUSH_STANDOFF behind the payload at PUSH_WZ, stroke out to PUSH_R_END
        at constant rate while sampling the blade/payload contact, then wait
        for the payload to come to rest before reading its displacement.

        Success needs sustained measured contact, the payload at rest ON the
        table, and at least PUSH_MIN of horizontal travel.
        """
        name, key, scene = resolve_scene(params)

        t0 = time.perf_counter()
        self._build(scene)
        self._budget = scene["budget"]
        start_pos = self._cube_pos()
        grasp_state, stage = "closed", "home"

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

        target = (scene["cube"][0], scene["cube"][1], CUBE_HALF)
        planar = math.hypot(target[0], target[1])

        try:
            # -- out-of-envelope target
            if planar > WORK_R + 0.02:
                stage = "stretch"
                self._run(KEYFRAMES["stretch"], STAGE_STEPS["move_above"],
                          PUSH_GRIP, abort_on_collision=False)
                gap = math.dist(self._tip_pos(), target)
                if gap > UNREACHABLE_GAP:
                    return report(False, "unreachable",
                                  f"tip stopped {gap:.3f} m short of the object")

            # Blade path, derived from the scene rather than hand-typed: start
            # PUSH_STANDOFF behind the payload's near face and stroke out to the
            # far edge of the envelope. Both radii are solved at PUSH_WZ, where
            # the pad bottoms clear the table instead of ploughing through it.
            r_start = max(0.15, planar - CUBE_HALF - PUSH_STANDOFF)
            push_start = solve(r_start, PUSH_WZ)
            push_end = solve(PUSH_R_END, PUSH_WZ)
            if push_start is None or push_end is None:    # pragma: no cover
                return report(False, "unreachable",
                              f"push path r={r_start:.3f}->{PUSH_R_END:.3f} "
                              f"is not solvable at wrist z={PUSH_WZ:.3f}")

            # -- stage 1/3 APPROACH: park the blade behind the payload
            stage = "approach"
            if not self._run(push_start, STAGE_STEPS["move_above"], PUSH_GRIP):
                return report(False, "collision", "obstacle struck during approach")

            # -- stage 2/3 PUSH: constant-rate stroke, contact sampled per step
            stage = "push"
            swept, sweep_peak_z = self._sweep(push_end, PUSH_STEPS, PUSH_GRIP)
            if not swept:
                return report(False, "collision", "obstacle struck during push")

            # -- stage 3/3 VERIFY: never measure a payload that is still moving
            stage = "verify"
            settled, waited, settle_peak_z = self._settle_to_rest(PUSH_GRIP)
            peak_z = max(sweep_peak_z, settle_peak_z)

        except BudgetExhausted:
            return report(False, "timeout",
                          f"step budget {self._budget} exhausted in stage {stage}")

        if not settled:
            return report(False, "unsettled",
                          f"payload still moving after {waited} settle steps")

        end_pos = self._cube_pos()
        disp = math.hypot(end_pos[0] - start_pos[0], end_pos[1] - start_pos[1])

        # Three independent gates, identical to the MuJoCo backend's.
        if self._contact_samples < PUSH_CONTACT_MIN:
            return report(False, "push_failed",
                          f"blade loaded the payload on only "
                          f"{self._contact_samples} steps (need "
                          f"{PUSH_CONTACT_MIN}) -- no sustained contact")
        if peak_z > PUSH_PEAK_MAX:
            return report(False, "push_failed",
                          f"payload was launched to z={peak_z:.4f} m "
                          f"(tumble apex is {PUSH_PEAK_MAX:.4f} m) -- struck, "
                          f"not pushed")
        if end_pos[2] > AIRBORNE_MAX:
            return report(False, "push_failed",
                          f"payload came to rest off the table at "
                          f"z={end_pos[2]:.3f} m")
        if disp < PUSH_MIN:
            return report(False, "push_failed",
                          f"payload moved only {disp:.3f} m horizontally")
        return report(True, "pushed",
                      f"payload pushed {disp:.3f} m horizontally over "
                      f"{self._contact_samples} contact steps "
                      f"(peak {self._peak_force:.2f} N), stayed on the table "
                      f"(peak z {peak_z:.4f} m) and came to rest at "
                      f"z={end_pos[2]:.3f} m after {waited} settle steps")


__all__ = ["PyBulletSimulator", "available", "ENGINE"]
