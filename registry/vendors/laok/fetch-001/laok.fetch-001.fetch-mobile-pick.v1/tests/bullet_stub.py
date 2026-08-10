"""A minimal stand-in for the `pybullet` module.

Purpose: exercise every PyBullet call the fetch-001 backend makes -- names,
keyword arguments, return-tuple indices -- on machines where the real wheel
cannot be built (PyBullet is source-only and needs a compiler on Windows).

This is a CONTRACT check, not a physics check. It deliberately does not model
dynamics; it parses the backend's own URDF for the joint ordering and returns
plausible sensor tuples so the 7-phase controller can be walked end to end.
The real physics agreement is asserted by TestSimToSimAgreement, which runs on
CI where PyBullet is importable.

Two deliberate simplifications, both safe for a contract check:
  * bodies are classified by the collision shape they were built from
    (plane -> floor, box+mass -> cube A, box+massless -> shelf,
    cylinder+massless -> obstacle), which is how the fetch cell differs from a
    single-object cell -- a shelf and an obstacle are both static bodies and
    must not be confused with each other;
  * after `removeConstraint` the released cube simply stays where it was let
    go. The placement verdict in the backend is decided by the released
    height and the XY offset to the shelf, both of which are already fixed at
    release time, so freezing cannot flip a verdict that real gravity would
    not have flipped.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import arm_spec

DIRECT = 2
GEOM_PLANE = 3
GEOM_BOX = 4
GEOM_CYLINDER = 5
VELOCITY_CONTROL = 6
JOINT_POINT2POINT = 7

# tunables used by the tests to drive different outcomes
PAD_FORCE = 6.0             # N reported per pad once the gripper is closed
OBSTACLE_HIT_STEP = 20      # step at which a blocked scene registers contact
GRASP_REACH = 0.05          # m, pad-to-cube distance that still counts as held


class _State:
    def __init__(self):
        self.reset()

    def reset(self):
        self.next_id = 100
        self.shape_types = {}
        self.joint_names = []
        self.joints = {}
        self.robot = None
        self.cube_a = None
        self.shelf = None
        self.obstacle = None
        self.pos = {}
        self.steps = 0
        self.attached = False
        self.constraints = set()
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
    sid = _new_id()
    S.shape_types[sid] = shapeType
    return sid


def createVisualShape(shapeType, physicsClientId=0, **kw):
    _log("createVisualShape")
    sid = _new_id()
    S.shape_types[sid] = shapeType
    return sid


def createMultiBody(baseMass, baseCollisionShapeIndex=-1,
                    baseVisualShapeIndex=-1, basePosition=(0, 0, 0),
                    physicsClientId=0, **kw):
    """Classify the body from its collision shape, exactly as the fetch cell
    builds it: plane floor, dynamic box cube A, static box shelf, static
    cylinder obstacle."""
    _log("createMultiBody")
    bid = _new_id()
    S.pos[bid] = list(basePosition)
    shape = S.shape_types.get(baseCollisionShapeIndex)
    if shape == GEOM_BOX and baseMass > 0:
        S.cube_a = bid
    elif shape == GEOM_BOX:
        S.shelf = bid
    elif shape == GEOM_CYLINDER:
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
    if S.attached and S.cube_a is not None:
        S.pos[S.cube_a] = list(_tip())


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


def getContactPoints(bodyA=None, bodyB=None, physicsClientId=0):
    if bodyB is not None and bodyB == S.obstacle:
        if S.steps >= OBSTACLE_HIT_STEP:
            return [(0, bodyA, bodyB, 3, -1, (0, 0, 0), (0, 0, 0),
                     (0, 0, 1), 0.0, 12.0)]
        return []
    if bodyB is not None and bodyB == S.cube_a:
        closed = _grip_value() <= arm_spec.FINGER_CLOSED + 1e-6
        near = math.dist(_tip(), S.pos[S.cube_a]) < GRASP_REACH
        if closed and near:
            idx = {n: i for i, n in enumerate(S.joint_names)}
            return [(0, bodyA, bodyB, idx["grip_l"], -1, (0, 0, 0), (0, 0, 0),
                     (0, 1, 0), 0.0, PAD_FORCE),
                    (0, bodyA, bodyB, idx["grip_r"], -1, (0, 0, 0), (0, 0, 0),
                     (0, -1, 0), 0.0, PAD_FORCE)]
    return []


def getBasePositionAndOrientation(bodyUniqueId, physicsClientId=0):
    return tuple(S.pos.get(bodyUniqueId, [0.0, 0.0, 0.0])), (0.0, 0.0, 0.0, 1.0)


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
    cid = _new_id()
    S.constraints.add(cid)
    S.attached = True
    return cid


def changeConstraint(userConstraintUniqueId, physicsClientId=0, **kw):
    _log("changeConstraint")


def removeConstraint(userConstraintUniqueId, physicsClientId=0):
    _log("removeConstraint")
    S.constraints.discard(userConstraintUniqueId)
    S.attached = False
