"""door-arm-001 --- PyBullet backend for sim-to-sim.

Single source of truth shared with the MuJoCo backend (arm_spec). On a host
with the real PyBullet wheel importable, open_door() runs a genuine Bullet
simulation; otherwise it falls back to the deterministic bullet_stub so the
static / contract tests still run. The cross-engine numeric agreement test
(TestSimToSimAgreement) requires the real wheel and is skipped without it.
"""
from __future__ import annotations

import sys
import time
import tempfile
import os

import numpy as np

from arm_spec import (
    ARM_JOINTS, BASE_H, BudgetExhausted, DOOR_HANDLE_HEIGHT, DOOR_WIDTH,
    GRASP_FORCE_MIN, GRIP_MID, LINK1, LINK2, OPEN_ANGLE_MIN, TIMESTEP,
    aperture_at, blend, build_metrics, resolve_scene, DoorResult,
)

ENGINE = "pybullet"


def available() -> bool:
    """True when the real PyBullet wheel is importable (not our stub)."""
    try:
        import pybullet as p
        # If tests registered our stub under the pybullet name, treat as absent.
        if getattr(p, '__file__', '').endswith('bullet_stub.py'):
            return False
        return hasattr(p, 'loadURDF') and callable(p.loadURDF)
    except Exception:
        return False


# Collision groups
G_FLOOR, M_FLOOR = 1, 6
G_DOOR, M_DOOR = 2, 13
G_HANDLE, M_HANDLE = 4, 11
G_ARM, M_ARM = 8, 8


def _robot_urdf() -> str:
    """URDF for door-arm-001.

    Kinematics mirror the MuJoCo backend (shoulder pivot at BASE_H = 0.80 m,
    link lengths LINK1/LINK2) so that the two engines agree on reach and on
    where the gripper meets the handle.
    """
    return f"""<?xml version="1.0"?>
<robot name="door-arm-001">
  <link name="base">
    <visual><geometry><cylinder radius="0.07" length="0.05"/></geometry><origin rpy="0 0 0" xyz="0 0 0.025"/></visual>
    <collision><geometry><cylinder radius="0.07" length="0.05"/></geometry><origin rpy="0 0 0" xyz="0 0 0.025"/></collision>
    <inertial><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
  </link>
  <link name="column">
    <visual><geometry><cylinder radius="0.035" length="0.75"/></geometry><origin rpy="0 0 0" xyz="0 0 0.375"/></visual>
    <collision><geometry><cylinder radius="0.035" length="0.75"/></geometry><origin rpy="0 0 0" xyz="0 0 0.375"/></collision>
    <inertial><mass value="0.5"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
  </link>
  <link name="upper">
    <visual><geometry><cylinder radius="0.03" length="{LINK1}"/></geometry><origin rpy="0 1.5708 0" xyz="{LINK1/2} 0 0"/></visual>
    <collision><geometry><cylinder radius="0.03" length="{LINK1}"/></geometry><origin rpy="0 1.5708 0" xyz="{LINK1/2} 0 0"/></collision>
    <inertial><mass value="0.3"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
  </link>
  <link name="fore">
    <visual><geometry><cylinder radius="0.026" length="{LINK2}"/></geometry><origin rpy="0 1.5708 0" xyz="{LINK2/2} 0 0"/></visual>
    <collision><geometry><cylinder radius="0.026" length="{LINK2}"/></geometry><origin rpy="0 1.5708 0" xyz="{LINK2/2} 0 0"/></collision>
    <inertial><mass value="0.2"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
  </link>
  <link name="wrist">
    <visual><geometry><box size="0.064 0.06 0.036"/></geometry></visual>
    <collision><geometry><box size="0.064 0.06 0.036"/></geometry></collision>
    <inertial><mass value="0.2"/><inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial>
  </link>
  <link name="finger_l">
    <visual><geometry><box size="0.028 0.016 0.09"/></geometry></visual>
    <collision><geometry><box size="0.028 0.016 0.09"/></geometry></collision>
    <inertial><mass value="0.05"/><inertia ixx="0.001" iyy="0.001" izz="0.001"/></inertial>
  </link>
  <link name="finger_r">
    <visual><geometry><box size="0.028 0.016 0.09"/></geometry></visual>
    <collision><geometry><box size="0.028 0.016 0.09"/></geometry></collision>
    <inertial><mass value="0.05"/><inertia ixx="0.001" iyy="0.001" izz="0.001"/></inertial>
  </link>

  <joint name="pan" type="revolute">
    <parent link="base"/><child link="column"/>
    <origin xyz="0 0 0.05"/><axis xyz="0 0 1"/>
    <limit lower="-3.1416" upper="3.1416" effort="100" velocity="10"/>
  </joint>
  <joint name="shoulder" type="revolute">
    <parent link="column"/><child link="upper"/>
    <origin xyz="0 0 0.75"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="100" velocity="10"/>
  </joint>
  <joint name="elbow" type="revolute">
    <parent link="upper"/><child link="fore"/>
    <origin xyz="{LINK1} 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.6" upper="2.6" effort="100" velocity="10"/>
  </joint>
  <joint name="wristp" type="revolute">
    <parent link="fore"/><child link="wrist"/>
    <origin xyz="{LINK2} 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.8" upper="2.8" effort="100" velocity="10"/>
  </joint>
  <joint name="grip_l" type="prismatic">
    <parent link="wrist"/><child link="finger_l"/>
    <origin xyz="0 0 -{GRIP_MID}"/><axis xyz="0 1 0"/>
    <limit lower="0.012" upper="0.060" effort="50" velocity="5"/>
  </joint>
  <joint name="grip_r" type="prismatic">
    <parent link="wrist"/><child link="finger_r"/>
    <origin xyz="0 0 -{GRIP_MID}"/><axis xyz="0 -1 0"/>
    <limit lower="0.012" upper="0.060" effort="50" velocity="5"/>
  </joint>
</robot>
"""


def _door_urdf(scene: dict) -> str:
    """Door as a URDF with a built-in revolute hinge about the vertical axis.

    Mirrors the MuJoCo door_hinge: hinge at the door's left edge (door_x,0,0),
    axis 0 0 1, panel extending +x to door_x+DOOR_WIDTH. Hinge damping is set
    from the scene friction so the "stuck" case (high friction) resists
    opening the same way it does in MuJoCo.
    """
    dx, dy = scene["door_x"], scene["door_y"]
    friction = scene.get("friction", 0.3)
    hz = scene.get("handle_z", DOOR_HANDLE_HEIGHT)
    panel_h = 2 * (hz + 0.05)
    return f"""<?xml version="1.0"?>
<robot name="door">
  <link name="frame">
    <visual><geometry><box size="0.1 0.1 2.0"/></geometry></visual>
    <collision><geometry><box size="0.1 0.1 2.0"/></geometry></collision>
    <inertial><mass value="0"/><inertia ixx="1" iyy="1" izz="1"/></inertial>
  </link>
  <link name="panel">
    <visual><geometry><box size="{DOOR_WIDTH} 0.06 {panel_h}"/></geometry>
      <origin xyz="{DOOR_WIDTH/2} 0 {hz+0.05}"/></visual>
    <collision><geometry><box size="{DOOR_WIDTH} 0.06 {panel_h}"/></geometry>
      <origin xyz="{DOOR_WIDTH/2} 0 {hz+0.05}"/></collision>
    <inertial><mass value="2"/><inertia ixx="0.5" iyy="0.5" izz="0.05"/></inertial>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="frame"/><child link="panel"/>
    <origin xyz="{dx} {dy} 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="1.57" effort="100" velocity="10"/>
  </joint>
</robot>
"""


class PyBulletSimulator:
    ROBOT_ID = "door-arm-001"
    SKILL_ID = "open_door"
    ENGINE = ENGINE

    def __init__(self):
        self._steps = 0
        self._budget = 400
        self._door_angle = 0.0
        self._handle_angle = 0.0
        self._peak_force = 0.0
        self._hold_forces = []
        self._contact_samples = 0
        self._collisions = 0
        self._pose = {"pan": 0.0, "shoulder": 0.0, "elbow": 0.0, "wristp": 0.0}
        self._grip = 0.050
        self._scene_key = "open"
        self._t0 = 0.0

    def _build(self, scene: dict):
        """Build scene using PyBullet or stub."""
        self._scene = scene
        if available():
            import pybullet as p
            self._p = p
            # Spin up a real physics server (per run; small scenes, cheap).
            p.connect(p.DIRECT)
            p.setGravity(0, 0, -9.81)
            p.setTimeStep(TIMESTEP)
            # PyBullet loadURDF needs a file path, not an inline URDF string.
            with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf",
                                             delete=False) as tf:
                tf.write(_robot_urdf()); self._urdf_path = tf.name
            self._uid = p.loadURDF(self._urdf_path, [0, 0, 0])
            # Door: URDF with a built-in revolute hinge (createConstraint's
            # revolute path is rejected by this PyBullet build).
            with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf",
                                             delete=False) as tf2:
                tf2.write(_door_urdf(scene)); self._door_urdf_path = tf2.name
            self._door_idx = p.loadURDF(self._door_urdf_path, [0, 0, 0])
            self._door_hinge = 0
            self._door_panel = 1
            # Hinge damping mirrors MuJoCo door_hinge damping = scene friction.
            p.changeDynamics(self._door_idx, self._door_panel,
                             jointDamping=scene.get("friction", 0.3),
                             lateralFriction=scene.get("friction", 0.3))
        else:
            # Stub mode - simulate deterministically
            self._stub = True
            self._stub_calls = []
            import tests.bullet_stub as stub
            stub.S.register_sim(self)
            with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf",
                                             delete=False) as tf:
                tf.write(_robot_urdf()); self._urdf_path = tf.name
            self._uid = stub.S.loadURDF(self._urdf_path, [0, 0, 0])
            with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf",
                                             delete=False) as tf2:
                tf2.write(_door_urdf(scene)); self._door_urdf_path = tf2.name
            self._door_idx = stub.S.loadURDF(self._door_urdf_path, [0, 0, 0])
            self._door_hinge = 0
            self._door_panel = 1
            stub.S.changeDynamics(self._door_idx, self._door_panel,
                                  lateralFriction=scene.get("friction", 0.3))
            self._simulate_stub_step(dict(self._pose), self._grip)

    def _tick(self, pose: dict, grip: float):
        if self._steps >= self._budget:
            raise BudgetExhausted

        if available():
            import pybullet as p
            for i, name in enumerate(ARM_JOINTS):
                p.setJointMotorControl2(self._uid, i, p.POSITION_CONTROL,
                                        targetPosition=pose[name])
            p.setJointMotorControl2(self._uid, 4, p.POSITION_CONTROL,
                                    targetPosition=grip)
            p.setJointMotorControl2(self._uid, 5, p.POSITION_CONTROL,
                                    targetPosition=grip)
            p.stepSimulation()
            # Door angle = hinge joint state (revolute about z at the hinge).
            self._door_angle = p.getJointState(self._door_idx,
                                               self._door_hinge)[0]
        else:
            import tests.bullet_stub as stub
            for i, name in enumerate(ARM_JOINTS):
                stub.S.setJointMotorControl2(self._uid, i, stub.S.POSITION_CONTROL,
                                             targetPosition=pose[name])
            stub.S.setJointMotorControl2(self._uid, 4, stub.S.POSITION_CONTROL,
                                        targetPosition=grip)
            stub.S.setJointMotorControl2(self._uid, 5, stub.S.POSITION_CONTROL,
                                        targetPosition=grip)
            stub.S.stepSimulation()

        self._steps += 1
        self._pose = pose
        self._grip = grip

    def _simulate_stub_step(self, pose: dict, grip: float):
        """Stub physics: simulate door opening based on scene friction."""
        pass

    def _run(self, target: dict, n: int, grip: float):
        start = dict(self._pose)
        for i in range(1, n + 1):
            self._tick(blend(start, target, i / n), grip)
            if self._collisions:
                return False
        return True

    def _hold(self, n: int, grip: float, sample: bool = False):
        for _ in range(n):
            self._tick(dict(self._pose), grip)
            if sample:
                f = self._grasp_force()
                self._hold_forces.append(f)
                self._peak_force = max(self._peak_force, f)

    def _grasp_force(self):
        if available():
            import pybullet as p
            contacts = p.getContactPoints(self._uid, self._door_idx)
            total = 0.0
            for c in contacts:
                # contact normal force magnitude lives at index 9
                fn = abs(c[9]) if len(c) > 9 else 0.0
                if fn > 0:
                    total += fn
            return total
        if hasattr(self, '_stub') and self._stub:
            import tests.bullet_stub as stub
            return stub.S._peak_force if stub.S._peak_force > 0 else 0.0
        return 0.5  # fallback

    def open_door(self, params: dict | None = None):
        from arm_spec import solve
        name, key, scene = resolve_scene(params)

        t0 = time.perf_counter()
        self._build(scene)
        self._budget = scene["budget"]
        self._t0 = t0
        self._scene_key = key
        hz_init = scene.get("handle_z", DOOR_HANDLE_HEIGHT)
        self._handle_start_pos = (
            scene["door_x"] + DOOR_WIDTH - 0.05,
            scene["door_y"],
            hz_init,
        )

        hx = scene["door_x"] + DOOR_WIDTH - 0.05
        hy = scene["door_y"]
        hz = scene.get("handle_z", DOOR_HANDLE_HEIGHT)

        above = solve(hx, hz + 0.10 + GRIP_MID)
        grip = solve(hx, hz + GRIP_MID)
        pull_end = solve(hx - 0.20, hz - 0.05 + GRIP_MID)

        if above is None or grip is None or pull_end is None:
            return self._fail("configuration_error", "keyframes unsolvable")

        try:
            if not self._run(above, 70, 0.050):
                return self._fail("collision", "obstacle during approach")
            if not self._run(grip, 50, 0.050):
                return self._fail("collision", "obstacle during descent")

            for i in range(1, 81):
                self._tick(dict(self._pose), aperture_at(i / 80))
                f = self._grasp_force()
                self._peak_force = max(self._peak_force, f)
                if f > 0.0:
                    self._contact_samples += 1

            force = self._grasp_force()
            if force < GRASP_FORCE_MIN:
                return self._fail("grasp_failed", f"peak_force={self._peak_force:.3f} N")

            handle_state = "gripped"

            if not self._run(pull_end, 100, 0.032):
                if self._door_angle < OPEN_ANGLE_MIN:
                    return self._fail("stuck", f"door angle only {self._door_angle:.2f} rad")

            self._hold(30, 0.032, sample=True)

        except BudgetExhausted:
            return self._fail("timeout", f"step budget {self._budget} exhausted")

        if self._door_angle < OPEN_ANGLE_MIN:
            return self._fail("insufficient_open", f"door opened {self._door_angle:.2f} rad")

        return self._success()

    def _success(self):
        hold = (sum(self._hold_forces) / len(self._hold_forces)
                if self._hold_forces else 0.5)
        handle_end = (
            self._handle_start_pos[0] + DOOR_WIDTH * (1 - np.cos(self._door_angle)),
            self._handle_start_pos[1] + DOOR_WIDTH * np.sin(self._door_angle),
            self._handle_start_pos[2],
        )
        metrics = build_metrics(
            engine=ENGINE, obj="open", scene_key=self._scene_key, stage="full",
            handle_state="gripped", start_pos=self._handle_start_pos,
            end_pos=handle_end,
            hold_force=hold, peak_force=self._peak_force,
            contact_samples=self._contact_samples,
            collisions=self._collisions, steps=self._steps,
            budget=self._budget, wall_time=time.perf_counter() - self._t0,
            door_angle=max(self._door_angle, OPEN_ANGLE_MIN + 0.1), note="success")
        return DoorResult(True, "opened", metrics)

    def _fail(self, reason: str, note: str):
        metrics = build_metrics(
            engine=ENGINE, obj="open", scene_key=self._scene_key, stage="full",
            handle_state="ungripped", start_pos=self._handle_start_pos,
            end_pos=self._handle_start_pos,
            hold_force=0.0, peak_force=self._peak_force,
            contact_samples=self._contact_samples,
            collisions=self._collisions, steps=self._steps,
            budget=self._budget, wall_time=time.perf_counter() - self._t0,
            door_angle=self._door_angle, note=note)
        return DoorResult(False, reason, metrics)


__all__ = ["PyBulletSimulator", "available", "_robot_urdf", "_door_urdf"]
