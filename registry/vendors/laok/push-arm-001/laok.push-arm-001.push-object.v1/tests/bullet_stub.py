"""A minimal stand-in for the `pybullet` module.

Purpose: exercise every PyBullet call the push backend makes -- names, keyword
arguments, return-tuple indices -- on machines where the real wheel cannot be
built (PyBullet is source-only and needs a compiler on Windows).

This is a CONTRACT check, not a physics check. It deliberately does not model
dynamics; it parses the backend's own URDF for the joint ordering, drives the
payload with a one-line kinematic shove model, and returns plausible sensor
tuples so every branch of `push_object` can be walked end to end. The real
physics agreement is asserted by TestSimToSimAgreement, which runs on CI where
PyBullet is importable.

The shove model is deliberately crude and deliberately honest about it: while
the closed blade's front face is inside the payload's footprint, the payload is
teleported to sit against the blade. That is enough to reach the success path
(sustained contact samples, a real displacement, a velocity that decays to zero
when the stroke stops) without pretending to be a solver.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET

import arm_spec

DIRECT = 2
GEOM_PLANE = 3
GEOM_BOX = 4
GEOM_CYLINDER = 5
VELOCITY_CONTROL = 6
JOINT_POINT2POINT = 7
ACTIVATION_STATE_DISABLE_DEACTIVATION = 8

# tunables used by the tests to drive different outcomes
BLADE_FORCE = 6.0           # N reported per pad while the blade is loaded
OBSTACLE_HIT_STEP = 20      # step at which a blocked scene registers contact


class _State:
    def __init__(self):
        self.reset()

    def reset(self):
        self.next_id = 100
        self.joint_names = []
        self.joints = {}
        self.robot = None
        self.cube = None
        self.cube_pos = [0.0, 0.0, 0.0]
        self.cube_vel = [0.0, 0.0, 0.0]
        self.obstacle = None
        self.steps = 0
        self.attached = False
        self.calls = []


S = _State()


def _new_id():
    S.next_id += 1
    return S.next_id


def _log(name):
    S.calls.append(name)


# ------------------------------------------------------------------ session
def connect(mode, **kw):
    _log("connect")
    S.reset()
    return 0


def disconnect(physicsClientId=0):
    _log("disconnect")


def setGravity(x, y, z, physicsClientId=0):
    _log("setGravity")


def setTimeStep(dt, physicsClientId=0):
    _log("setTimeStep")


def setPhysicsEngineParameter(physicsClientId=0, **kw):
    _log("setPhysicsEngineParameter")


# ------------------------------------------------------------------- shapes
def createCollisionShape(shapeType, physicsClientId=0, **kw):
    _log("createCollisionShape")
    return _new_id()


def createVisualShape(shapeType, physicsClientId=0, **kw):
    _log("createVisualShape")
    return _new_id()


def createMultiBody(baseMass, baseCollisionShapeIndex=-1,
                    baseVisualShapeIndex=-1, basePosition=(0, 0, 0),
                    physicsClientId=0, **kw):
    _log("createMultiBody")
    bid = _new_id()
    if baseMass > 0:                       # the payload is the only dynamic body
        S.cube = bid
        S.cube_pos = list(basePosition)
        S.cube_vel = [0.0, 0.0, 0.0]
    elif baseCollisionShapeIndex >= 0 and basePosition != (0, 0, 0) \
            and list(basePosition) != [0, 0, 0]:
        S.obstacle = bid
    return bid


def changeDynamics(bodyUniqueId, linkIndex, physicsClientId=0, **kw):
    _log("changeDynamics")


def setCollisionFilterGroupMask(bodyUniqueId, linkIndexA, collisionFilterGroup,
                                collisionFilterMask, physicsClientId=0):
    _log("setCollisionFilterGroupMask")


# -------------------------------------------------------------------- robot
def loadURDF(path, basePosition=(0, 0, 0), useFixedBase=False,
             physicsClientId=0, **kw):
    """Parse the real URDF so joint ordering comes from the backend itself."""
    _log("loadURDF")
    root = ET.parse(path).getroot()
    S.joint_names = [j.get("name") for j in root.findall("joint")]
    S.joints = {i: 0.0 for i in range(len(S.joint_names))}
    S.robot = _new_id()
    return S.robot


def getNumJoints(bodyUniqueId, physicsClientId=0):
    return len(S.joint_names)


def getJointInfo(bodyUniqueId, jointIndex, physicsClientId=0):
    name = S.joint_names[jointIndex].encode()
    return (jointIndex, name, 0, -1, -1, 0, 0.0, 0.0,
            -3.15, 3.15, 200.0, 10.0, b"link", (0, 0, 1), (0, 0, 0),
            (0, 0, 0, 1), -1)


def setJointMotorControl2(bodyUniqueId, jointIndex, controlMode,
                          physicsClientId=0, **kw):
    _log("setJointMotorControl2")


def resetJointState(bodyUniqueId, jointIndex, targetValue,
                    targetVelocity=0.0, physicsClientId=0):
    S.joints[jointIndex] = targetValue


def stepSimulation(physicsClientId=0):
    S.steps += 1
    prev = list(S.cube_pos)
    if S.attached:
        S.cube_pos = list(_tip())
    elif _blade_loaded():
        S.cube_pos[0] = _blade_front_x() + arm_spec.CUBE_HALF
    S.cube_vel = [(S.cube_pos[i] - prev[i]) / arm_spec.TIMESTEP
                  for i in range(3)]


# ------------------------------------------------------------------ sensing
def _pose():
    idx = {n: i for i, n in enumerate(S.joint_names)}
    return {j: S.joints[idx[j]] for j in arm_spec.ARM_JOINTS}


def _tip():
    x, y, z = arm_spec.forward(_pose())
    return [x, y, z - arm_spec.GRIP_MID]


def _grip_value():
    idx = {n: i for i, n in enumerate(S.joint_names)}
    return S.joints[idx["grip_l"]]


def _blade_front_x():
    """World x of the closed pads' leading face."""
    return arm_spec.forward(_pose())[0] + arm_spec.FINGER_HALF_X


def _blade_loaded() -> bool:
    """True while the closed blade's front face is inside the payload."""
    if S.cube is None:
        return False
    if _grip_value() > arm_spec.PUSH_GRIP + 1e-6:
        return False                                  # pads not a solid blade
    wx, wy, wz = arm_spec.forward(_pose())
    if wz - arm_spec.GRIP_MID - arm_spec.FINGER_HALF_Z > \
            S.cube_pos[2] + arm_spec.CUBE_HALF:
        return False                                  # blade rides over it
    if abs(wy - S.cube_pos[1]) > arm_spec.CUBE_HALF + arm_spec.PAD_HALF:
        return False                                  # blade is off to one side
    front = wx + arm_spec.FINGER_HALF_X
    return (S.cube_pos[0] - arm_spec.CUBE_HALF <= front
            <= S.cube_pos[0] + arm_spec.CUBE_HALF)


def getContactPoints(bodyA=None, bodyB=None, physicsClientId=0):
    if bodyB == S.obstacle and S.obstacle is not None:
        if S.steps >= OBSTACLE_HIT_STEP:
            return [(0, bodyA, bodyB, 3, -1, (0, 0, 0), (0, 0, 0),
                     (0, 0, 1), 0.0, 12.0)]
        return []
    if bodyB == S.cube and S.cube is not None:
        if S.attached or _blade_loaded():
            idx = {n: i for i, n in enumerate(S.joint_names)}
            return [(0, bodyA, bodyB, idx["grip_l"], -1, (0, 0, 0), (0, 0, 0),
                     (1, 0, 0), 0.0, BLADE_FORCE),
                    (0, bodyA, bodyB, idx["grip_r"], -1, (0, 0, 0), (0, 0, 0),
                     (1, 0, 0), 0.0, BLADE_FORCE)]
    return []


def getBasePositionAndOrientation(bodyUniqueId, physicsClientId=0):
    return tuple(S.cube_pos), (0.0, 0.0, 0.0, 1.0)


def getBaseVelocity(bodyUniqueId, physicsClientId=0):
    return tuple(S.cube_vel), (0.0, 0.0, 0.0)


def getLinkState(bodyUniqueId, linkIndex, computeForwardKinematics=False,
                 physicsClientId=0):
    x, y, z = arm_spec.forward(_pose())
    frame = (x, y, z)
    orn = (0.0, 0.0, 0.0, 1.0)     # wrist pitch sums to zero by construction
    return (frame, orn, (0, 0, 0), (0, 0, 0, 1), frame, orn)


def getMatrixFromQuaternion(orn, physicsClientId=0):
    return (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)


# -------------------------------------------------------------- constraints
def createConstraint(parentBodyUniqueId, parentLinkIndex, childBodyUniqueId,
                     childLinkIndex, jointType, jointAxis,
                     parentFramePosition, childFramePosition,
                     physicsClientId=0, **kw):
    _log("createConstraint")
    S.attached = True
    return _new_id()


def changeConstraint(userConstraintUniqueId, physicsClientId=0, **kw):
    _log("changeConstraint")
