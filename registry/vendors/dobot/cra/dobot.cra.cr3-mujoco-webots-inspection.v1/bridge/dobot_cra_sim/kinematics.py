"""Small dependency-free CR3 FK/IK planner derived from the pinned URDF."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence
from xml.etree import ElementTree

from .assets import JOINT_NAMES, VENDOR_URDF


Matrix = tuple[tuple[float, float, float, float], ...]
Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class JointSpec:
    name: str
    origin_xyz: Vector3
    origin_rpy: Vector3
    axis: Vector3
    lower: float
    upper: float


def _identity() -> Matrix:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _matmul(left: Matrix, right: Matrix) -> Matrix:
    return tuple(
        tuple(sum(left[row][index] * right[index][column] for index in range(4)) for column in range(4))
        for row in range(4)
    )


def _xyz_rpy(xyz: Vector3, rpy: Vector3) -> Matrix:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]),
        (-sp, cp * sr, cp * cr, xyz[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


def _axis_angle(axis: Vector3, angle: float) -> Matrix:
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm <= 1e-12:
        raise RuntimeError("Vendor CR3 URDF declares a zero-length joint axis")
    x, y, z = x / norm, y / norm, z / norm
    cosine, sine, one_minus = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return (
        (cosine + x * x * one_minus, x * y * one_minus - z * sine, x * z * one_minus + y * sine, 0.0),
        (y * x * one_minus + z * sine, cosine + y * y * one_minus, y * z * one_minus - x * sine, 0.0),
        (z * x * one_minus - y * sine, z * y * one_minus + x * sine, cosine + z * z * one_minus, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _float3(value: str) -> Vector3:
    parts = tuple(float(item) for item in value.split())
    if len(parts) != 3:
        raise RuntimeError("Vendor CR3 URDF has an invalid 3-vector")
    return parts  # type: ignore[return-value]


def joint_specs(path: Path = VENDOR_URDF) -> tuple[JointSpec, ...]:
    """Read the six actuated-joint transforms and limits directly from the vendor URDF."""

    root = ElementTree.parse(path).getroot()
    by_name = {node.attrib.get("name"): node for node in root.findall("joint")}
    specs: list[JointSpec] = []
    for name in JOINT_NAMES:
        node = by_name.get(name)
        if node is None or node.attrib.get("type") != "revolute":
            raise RuntimeError(f"Vendor CR3 URDF is missing revolute {name}")
        origin = node.find("origin")
        axis = node.find("axis")
        limit = node.find("limit")
        if origin is None or axis is None or limit is None:
            raise RuntimeError(f"Vendor CR3 URDF has an incomplete {name}")
        specs.append(
            JointSpec(
                name=name,
                origin_xyz=_float3(origin.attrib["xyz"]),
                origin_rpy=_float3(origin.attrib.get("rpy", "0 0 0")),
                axis=_float3(axis.attrib["xyz"]),
                lower=float(limit.attrib["lower"]),
                upper=float(limit.attrib["upper"]),
            )
        )
    return tuple(specs)


SPECS = joint_specs()
JOINT_LIMITS = tuple((spec.lower, spec.upper) for spec in SPECS)


def tool_transform(joints: Sequence[float]) -> Matrix:
    """Forward kinematics for the vendor Link6 frame, without simulator state writes."""

    if len(joints) != len(SPECS):
        raise ValueError("CR3 forward kinematics requires six joint values")
    transform = _identity()
    for spec, angle in zip(SPECS, joints):
        transform = _matmul(transform, _xyz_rpy(spec.origin_xyz, spec.origin_rpy))
        transform = _matmul(transform, _axis_angle(spec.axis, float(angle)))
    return transform


def tool_position(joints: Sequence[float]) -> Vector3:
    transform = tool_transform(joints)
    return transform[0][3], transform[1][3], transform[2][3]


def distance(first: Vector3, second: Vector3) -> float:
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(first, second)))


def numerical_position_jacobian(joints: Sequence[float], epsilon: float = 1e-4) -> tuple[Vector3, ...]:
    """Return a 3x6 numerical Jacobian from vendor FK, evaluated at measured joints."""

    values = list(float(value) for value in joints)
    columns: list[Vector3] = []
    for index in range(len(values)):
        plus, minus = values.copy(), values.copy()
        plus[index] += epsilon
        minus[index] -= epsilon
        upper, lower = tool_position(plus), tool_position(minus)
        columns.append(tuple((upper[row] - lower[row]) / (2.0 * epsilon) for row in range(3)))  # type: ignore[arg-type]
    return tuple(columns)


def _solve_3x3(matrix: list[list[float]], vector: Vector3) -> Vector3:
    """Gaussian elimination for the positive-definite DLS normal equation."""

    augmented = [row[:] + [float(vector[index])] for index, row in enumerate(matrix)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise RuntimeError("CR3 inspection Jacobian became singular")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        factor = augmented[column][column]
        augmented[column] = [value / factor for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                value - factor * pivot_value for value, pivot_value in zip(augmented[row], augmented[column])
            ]
    return augmented[0][3], augmented[1][3], augmented[2][3]


def damped_least_squares_step(
    joints: Sequence[float], target: Vector3, *, damping: float = 0.025, max_step_rad: float = 0.035
) -> tuple[float, ...]:
    """Plan one bounded joint step from measured state to a Cartesian inspection target."""

    position = tool_position(joints)
    error = tuple(target[index] - position[index] for index in range(3))
    jacobian = numerical_position_jacobian(joints)
    normal = [
        [sum(jacobian[column][row] * jacobian[column][other] for column in range(6)) for other in range(3)]
        for row in range(3)
    ]
    for index in range(3):
        normal[index][index] += damping * damping
    dual = _solve_3x3(normal, error)  # (J J^T + λ²I)^-1 error
    raw = [sum(jacobian[column][row] * dual[row] for row in range(3)) for column in range(6)]
    planned: list[float] = []
    for index, value in enumerate(raw):
        lower, upper = JOINT_LIMITS[index]
        delta = max(-max_step_rad, min(max_step_rad, value))
        planned.append(max(lower, min(upper, float(joints[index]) + delta)))
    return tuple(planned)


def solve_tool_position(
    current: Sequence[float], target: Vector3, *, iterations: int = 180, tolerance_m: float = 0.008
) -> tuple[float, ...]:
    """Plan an inspection posture from measured joints using iterative DLS IK.

    This runs only on a local Python vector. It never overwrites simulator
    state; the simulator's position actuators remain responsible for motion.
    """

    candidate = tuple(float(value) for value in current)
    for _ in range(iterations):
        if distance(tool_position(candidate), target) <= tolerance_m:
            return candidate
        candidate = damped_least_squares_step(candidate, target, max_step_rad=0.045)
    return candidate


def finite_joint_vector(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)
