"""door-arm-001 --- PyBullet backend for the door-opening skill.

Mirrors the MuJoCo MJCF kinematics exactly so both engines produce
identical handle_start_pos, IK targets, and contact topology.
"""
from __future__ import annotations

import tempfile
import time
from pathlib import Path


def available() -> bool:
    try:
        import pybullet as p
        if getattr(p, "__file__", "").endswith("bullet_stub.py"):
            return False
        return hasattr(p, "loadURDF") and callable(p.loadURDF)
    except Exception:
        return False


def _mass_box(m: float, hx: float, hy: float, hz: float) -> str:
    Ixx = m * (4 * hy * hy + 4 * hz * hz) / 12.0
    Iyy = m * (4 * hx * hx + 4 * hz * hz) / 12.0
    Izz = m * (4 * hx * hx + 4 * hy * hy) / 12.0
    return (
        f'<inertial>\n'
        f'  <origin xyz="0 0 0"/>\n'
        f'  <mass value="{m:.6f}"/>\n'
        f'  <inertia ixx="{Ixx:.6f}" iyy="{Iyy:.6f}" izz="{Izz:.6f}" '
        f'ixy="0.0" ixz="0.0" iyz="0.0"/>\n'
        f'</inertial>'
    )


def _robot_urdf() -> str:
    """Arm URDF mirroring MuJoCo MJCF kinematics exactly."""
    # MuJoCo kinematics:
    # base at (0,0,0)
    # column at (0,0,BASE_H-0.35)=(0,0,0.45)
    # upper at (0,0,BASE_H)=(0,0,0.80)
    # fore at (LINK1,0,BASE_H)=(0.28,0,0.80)
    # wrist at (LINK1+LINK2,0,BASE_H)=(0.52,0,0.80)
    # finger_l at (LINK1+LINK2,0,BASE_H-GRIP_MID)=(0.52,0,0.735)
    m_base, m_col, m_up, m_fore, m_wr, m_fg = 2.0, 1.5, 1.0, 0.8, 0.3, 0.15

    return f"""<?xml version="1.0" ?>
<robot name="door-arm-001">
  <link name="base">
    <visual><origin xyz="0 0 0.025"/><geometry><cylinder radius="0.07" length="0.05"/></geometry><material name="dark"><color rgba="0.25 0.27 0.32 1"/></material></visual>
    <collision><origin xyz="0 0 0.025"/><geometry><cylinder radius="0.07" length="0.05"/></geometry></collision>
    {_mass_box(m_base, 0.07, 0.07, 0.025)}
  </link>
  <link name="column">
    <visual><origin xyz="0 0 0.45"/><geometry><cylinder radius="0.035" length="0.35"/></geometry><material name="grey"><color rgba="0.30 0.32 0.38 1"/></material></visual>
    <collision><origin xyz="0 0 0.45"/><geometry><cylinder radius="0.035" length="0.35"/></geometry></collision>
    {_mass_box(m_col, 0.035, 0.035, 0.175)}
  </link>
  <joint name="pan" type="revolute">
    <parent link="base"/><child link="column"/>
    <origin xyz="0 0 0.05"/><axis xyz="0 0 1"/>
    <limit lower="-3.1416" upper="3.1416" effort="100" velocity="10"/>
  </joint>
  <link name="upper">
    <visual><origin xyz="0.14 0 0.80"/><geometry><cylinder radius="0.03" length="0.28"/></geometry><material name="arm"><color rgba="0.85 0.55 0.18 1"/></material></visual>
    <collision><origin xyz="0.14 0 0.80"/><geometry><cylinder radius="0.03" length="0.28"/></geometry></collision>
    {_mass_box(m_up, 0.14, 0.03, 0.03)}
  </link>
  <joint name="shoulder" type="revolute">
    <parent link="column"/><child link="upper"/>
    <origin xyz="0 0 0.35"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="100" velocity="10"/>
  </joint>
  <link name="fore">
    <visual><origin xyz="0.12 0 0.80"/><geometry><cylinder radius="0.026" length="0.24"/></geometry><material name="arm"><color rgba="0.85 0.55 0.18 1"/></material></visual>
    <collision><origin xyz="0.12 0 0.80"/><geometry><cylinder radius="0.026" length="0.24"/></geometry></collision>
    {_mass_box(m_fore, 0.12, 0.026, 0.026)}
  </link>
  <joint name="elbow" type="revolute">
    <parent link="upper"/><child link="fore"/>
    <origin xyz="0.28 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.6" upper="2.6" effort="100" velocity="10"/>
  </joint>
  <link name="wrist">
    <visual><origin xyz="0 0 0"/><geometry><box size="0.064 0.060 0.036"/></geometry><material name="dark"><color rgba="0.30 0.32 0.38 1"/></material></visual>
    <collision><origin xyz="0 0 0"/><geometry><box size="0.064 0.060 0.036"/></geometry></collision>
    {_mass_box(m_wr, 0.032, 0.030, 0.018)}
  </link>
  <joint name="wristp" type="revolute">
    <parent link="fore"/><child link="wrist"/>
    <origin xyz="0.24 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.8" upper="2.8" effort="100" velocity="10"/>
  </joint>
  <link name="finger_l">
    <visual><origin xyz="0 0 -0.045"/><geometry><box size="0.028 0.016 0.090"/></geometry><material name="light"><color rgba="0.90 0.90 0.92 1"/></material></visual>
    <collision><origin xyz="0 0 -0.045"/><geometry><box size="0.028 0.016 0.090"/></geometry></collision>
    {_mass_box(m_fg, 0.014, 0.008, 0.045)}
  </link>
  <joint name="grip_l" type="prismatic">
    <parent link="wrist"/><child link="finger_l"/>
    <origin xyz="0 -0.025 -0.020"/><axis xyz="0 1 0"/>
    <limit lower="0.012" upper="0.060" effort="50" velocity="5"/>
  </joint>
  <link name="finger_r">
    <visual><origin xyz="0 0 -0.045"/><geometry><box size="0.028 0.016 0.090"/></geometry><material name="light"><color rgba="0.90 0.90 0.92 1"/></material></visual>
    <collision><origin xyz="0 0 -0.045"/><geometry><box size="0.028 0.016 0.090"/></geometry></collision>
    {_mass_box(m_fg, 0.014, 0.008, 0.045)}
  </link>
  <joint name="grip_r" type="prismatic">
    <parent link="wrist"/><child link="finger_r"/>
    <origin xyz="0 +0.025 -0.020"/><axis xyz="0 -1 0"/>
    <limit lower="0.012" upper="0.060" effort="50" velocity="5"/>
  </joint>
</robot>
"""


def _door_urdf(scene: dict) -> str:
    """Door URDF mirroring MuJoCo MJCF kinematics."""
    dx, dy = scene["door_x"], scene["door_y"]
    hz = scene.get("handle_z", 0.85)
    w = 0.50
    hz_full = hz + 0.05
    m_door = 3.0
    hx, hy, hdoor = w / 2.0, 0.03, hz_full
    Ixx = m_door * (4 * hy * hy + 4 * hdoor * hdoor) / 12.0
    Iyy = m_door * (4 * hx * hx + 4 * hdoor * hdoor) / 12.0
    Izz = m_door * (4 * hx * hx + 4 * hy * hy) / 12.0
    inertial = (
        f'<inertial>\n'
        f'  <origin xyz="{hx:.4f} 0 {hz_full:.4f}"/>\n'
        f'  <mass value="{m_door:.3f}"/>\n'
        f'  <inertia ixx="{Ixx:.6f}" iyy="{Iyy:.6f}" izz="{Izz:.6f}" '
        f'ixy="0.0" ixz="0.0" iyz="0.0"/>\n'
        f'</inertial>'
    )
    return f"""<?xml version="1.0" ?>
<robot name="door-panel">
  <link name="base">
    <visual><geometry><box size="0.001 0.001 0.001"/></geometry></visual>
    <collision><geometry><box size="0.001 0.001 0.001"/></geometry></collision>
    <inertial><mass value="0.001"/><inertia ixx="1e-9" iyy="1e-9" izz="1e-9" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <link name="panel">
    <visual><origin xyz="{w/2} 0 {hz_full}"/><geometry><box size="{w} 0.06 {hz*2 + 0.10}"/></geometry><material name="wood"><color rgba="0.85 0.65 0.35 1"/></material></visual>
    <collision><origin xyz="{w/2} 0 {hz_full}"/><geometry><box size="{w} 0.06 {hz*2 + 0.10}"/></geometry></collision>
    {inertial}
  </link>
  <link name="handle">
    <visual><origin xyz="{w - 0.05} 0.04 {hz}"/><geometry><cylinder radius="0.015" length="0.04"/></geometry><material name="metal"><color rgba="0.6 0.6 0.65 1"/></material></visual>
    <collision><origin xyz="{w - 0.05} 0.04 {hz}"/><geometry><cylinder radius="0.015" length="0.04"/></geometry></collision>
    {_mass_box(0.2, 0.015, 0.02, 0.015)}
  </link>
  <joint name="door_hinge" type="revolute">
    <parent link="base"/><child link="panel"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="1.57" effort="100" velocity="10"/>
  </joint>
  <joint name="handle_rot" type="revolute">
    <parent link="panel"/><child link="handle"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="5"/>
  </joint>
</robot>
"""


class PhysicsServer:
    TIMESTEP = 0.002

    def __init__(self) -> None:
        import pybullet as p
        self._p = p
        self._uid: int = -1
        self._arm_uid: int = -1
        self._door_idx: int = -1
        self._door_angle: float = 0.0
        self._handle_start_pos: list[float] = [0.0, 0.0, 0.0]
        self._peak_force: float = 0.0
        self._hold_forces: list[float] = []
        self._contact_samples: int = 0
        self._collisions: int = 0
        self._steps: int = 0
        self._budget: int = 400

    def connect(self) -> None:
        self._p.connect(self._p.DIRECT)
        self._p.setGravity(0, 0, -9.81)
        self._p.setTimeStep(self.TIMESTEP)

    def _build(self, scene: dict) -> None:
        p = self._p
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.TIMESTEP)

        # Door — base at (dx, dy, 0), matching MuJoCo body "door" pos
        df = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
        df.write(_door_urdf(scene))
        df.close()
        self._door_idx = p.loadURDF(df.name, [scene["door_x"], scene["door_y"], 0])
        Path(df.name).unlink(missing_ok=True)

        # Arm — base at (0,0,0), matching MuJoCo body "base" pos
        af = tempfile.NamedTemporaryFile(mode="w", suffix="_arm.urdf", delete=False)
        af.write(_robot_urdf())
        af.close()
        self._arm_uid = p.loadURDF(af.name, [0, 0, 0.0])
        Path(af.name).unlink(missing_ok=True)

        # Handle start pos: mirror MuJoCo exactly
        # MuJoCo: handle body pos = [DOOR_WIDTH - 0.05, 0.04, hz] = [0.45, 0.04, 0.85]
        hz = scene.get("handle_z", 0.85)
        dx = scene["door_x"]
        w = 0.50
        self._handle_start_pos = [dx + w - 0.05, 0.04, hz]

        self._steps = 0
        self._peak_force = 0.0
        self._hold_forces = []
        self._contact_samples = 0
        self._collisions = 0
        self._update_door_angle()

    def _update_door_angle(self) -> None:
        p = self._p
        n = p.getNumJoints(self._door_idx)
        for j in range(n):
            info = p.getJointInfo(self._door_idx, j)
            if info[1].decode() == "door_hinge":
                self._door_angle = p.getJointState(self._door_idx, j)[0]
                return
        self._door_angle = 0.0

    def _get_joint_state(self, body_uid: int, joint_name: str) -> float:
        n = self._p.getNumJoints(body_uid)
        for j in range(n):
            info = self._p.getJointInfo(body_uid, j)
            if info[1].decode() == joint_name:
                return self._p.getJointState(body_uid, j)[0]
        return 0.0

    def _set_joint_state(self, body_uid: int, joint_name: str, value: float) -> None:
        n = self._p.getNumJoints(body_uid)
        for j in range(n):
            info = self._p.getJointInfo(body_uid, j)
            if info[1].decode() == joint_name:
                self._p.resetJointState(body_uid, j, value)
                return

    def _tick(self) -> None:
        self._p.stepSimulation()
        self._steps += 1
        self._update_door_angle()

    def _get_contact_force(self) -> float:
        """Sum of normal contact forces between arm and door bodies."""
        contacts = self._p.getContactPoints()
        total = 0.0
        for c in contacts:
            if (c[1] == self._arm_uid and c[2] == self._door_idx) or \
               (c[1] == self._door_idx and c[2] == self._arm_uid):
                total += abs(c[9])
        return total

    def apply_action(self, action: dict) -> None:
        for joint, value in action.items():
            self._set_joint_state(self._arm_uid, joint, value)

    def close(self) -> None:
        pass


class PyBulletSimulator:
    ROBOT_ID = "door-arm-001"
    SKILL_ID = "open_door"
    ENGINE = "pybullet"

    def __init__(self) -> None:
        self._sim: PhysicsServer | None = None

    def _ensure_sim(self) -> PhysicsServer:
        if self._sim is None:
            self._sim = PhysicsServer()
            self._sim.connect()
        return self._sim

    def open_door(self, params: dict | None = None):
        """Run one door-opening episode and return a DoorResult."""
        from arm_spec import (
            resolve_scene, build_metrics, DoorResult,
            OPEN_ANGLE_MIN, GRASP_FORCE_MIN,
            STAGE_STEPS, aperture_at, blend,
            solve, DOOR_WIDTH,
        )
        name, key, scene = resolve_scene(params)
        sim = self._ensure_sim()
        sim._build(scene)

        t0 = time.perf_counter()
        handle_start = sim._handle_start_pos.copy()
        handle_state = "ungripped"

        hx = scene["door_x"] + DOOR_WIDTH - 0.05
        hz = scene.get("handle_z", 0.85)
        GRIP_MID = 0.065

        above = solve(hx, hz + 0.10 + GRIP_MID)
        grip = solve(hx, hz + GRIP_MID)
        pull_end = solve(hx - 0.20, hz - 0.05 + GRIP_MID)

        def make_result(success, reason, note=""):
            hold = (sum(sim._hold_forces) / len(sim._hold_forces)
                    if sim._hold_forces else 0.0)
            import math
            handle_end = handle_start + [
                DOOR_WIDTH * (1 - math.cos(sim._door_angle)),
                DOOR_WIDTH * math.sin(sim._door_angle),
                0.0
            ]
            metrics = build_metrics(engine=self.ENGINE, obj=name,
                scene_key=key, stage="full", handle_state=handle_state,
                start_pos=handle_start, end_pos=handle_end,
                hold_force=hold, peak_force=sim._peak_force,
                contact_samples=sim._contact_samples,
                collisions=sim._collisions, steps=sim._steps,
                budget=sim._budget,
                wall_time=time.perf_counter() - t0,
                door_angle=sim._door_angle, note=note)
            return DoorResult(success, reason, metrics)

        if above is None or grip is None or pull_end is None:
            return make_result(False, "configuration_error", "keyframes unsolvable")

        # Stage 1: move above handle
        if above:
            for i in range(1, STAGE_STEPS["move_above"] + 1):
                pose = blend({"pan": 0, "shoulder": 0, "elbow": 0, "wristp": 0},
                             above, i / STAGE_STEPS["move_above"])
                sim.apply_action(pose)
                for _ in range(5):
                    sim._tick()
                if sim._steps >= sim._budget:
                    break

        # Stage 2: descend
        if grip:
            for i in range(1, STAGE_STEPS["descend"] + 1):
                start = above if above else {"pan": 0, "shoulder": 0, "elbow": 0, "wristp": 0}
                pose = blend(start, grip, i / STAGE_STEPS["descend"])
                sim.apply_action(pose)
                for _ in range(5):
                    sim._tick()
                if sim._steps >= sim._budget:
                    break

        # Stage 3: grip
        for i in range(1, STAGE_STEPS["grip"] + 1):
            aperture = aperture_at(i / STAGE_STEPS["grip"])
            sim._set_joint_state(sim._arm_uid, "grip_l", aperture)
            sim._set_joint_state(sim._arm_uid, "grip_r", -aperture)
            for _ in range(5):
                sim._tick()
            if sim._steps >= sim._budget:
                break
            force = sim._get_contact_force()
            sim._peak_force = max(sim._peak_force, force)
            if force > 0.0:
                sim._contact_samples += 1
                sim._hold_forces.append(force)

        force = sim._get_contact_force()
        if force < GRASP_FORCE_MIN:
            handle_state = "slipped"
            return make_result(False, "grasp_failed",
                f"peak_force={sim._peak_force:.3f} N")
        handle_state = "gripped"

        # Stage 4: pull
        if pull_end:
            for i in range(1, STAGE_STEPS["pull"] + 1):
                pose = blend(grip, pull_end, i / STAGE_STEPS["pull"])
                sim.apply_action(pose)
                for _ in range(5):
                    sim._tick()
                if sim._steps >= sim._budget:
                    break

        # Stage 5: settle
        for _ in range(STAGE_STEPS["settle"]):
            sim._tick()

        if sim._door_angle < OPEN_ANGLE_MIN:
            handle_state = "incomplete"
            return make_result(False, "insufficient_open",
                f"door opened {sim._door_angle:.2f} rad")

        return make_result(True, "opened",
            f"door opened {sim._door_angle:.2f} rad ({sim._door_angle*180/3.14159:.1f} deg)")


__all__ = ["PyBulletSimulator", "available", "_robot_urdf"]
