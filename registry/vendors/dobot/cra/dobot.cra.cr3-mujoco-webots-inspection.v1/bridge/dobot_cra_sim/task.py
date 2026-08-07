"""Stateful three-tag coverage policy shared by the two simulator adapters."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

from .course import INSPECTION_TARGETS, STABLE_SAMPLES_REQUIRED, TARGET_TOLERANCE_M
from .kinematics import Vector3, distance, solve_tool_position


@dataclass
class InspectionTask:
    """Select the nearest unobserved tag and solve it from measured joints."""

    pending: dict[str, Vector3] = field(
        default_factory=lambda: {target_id: position for target_id, position, _ in INSPECTION_TARGETS}
    )
    observed: list[dict[str, object]] = field(default_factory=list)
    current_target_id: str | None = None
    stable_samples: int = 0

    def _select(self, tool_position_m: Vector3) -> str | None:
        if not self.pending:
            self.current_target_id = None
            return None
        if self.current_target_id not in self.pending:
            self.current_target_id = min(
                self.pending,
                key=lambda target_id: distance(tool_position_m, self.pending[target_id]),
            )
            self.stable_samples = 0
        return self.current_target_id

    def update(self, tool_position_m: Vector3, elapsed_seconds: float) -> None:
        """Advance the task only from an engine-reported tool-center position."""

        target_id = self._select(tool_position_m)
        if target_id is None:
            return
        error = distance(tool_position_m, self.pending[target_id])
        self.stable_samples = self.stable_samples + 1 if error <= TARGET_TOLERANCE_M else 0
        if self.stable_samples < STABLE_SAMPLES_REQUIRED:
            return
        self.observed.append(
            {
                "target_id": target_id,
                "time_seconds": round(float(elapsed_seconds), 3),
                "measured_error_m": round(error, 5),
                "measured_tool_position_m": [round(value, 5) for value in tool_position_m],
            }
        )
        del self.pending[target_id]
        self.current_target_id = None
        self.stable_samples = 0

    def plan(self, measured_joints: Sequence[float], measured_tool_position_m: Vector3) -> tuple[float, ...]:
        """Return an online DLS IK command for the currently selected physical tag."""

        target_id = self._select(measured_tool_position_m)
        if target_id is None:
            return tuple(float(value) for value in measured_joints)
        return solve_tool_position(measured_joints, self.pending[target_id])

    @property
    def complete(self) -> bool:
        return not self.pending
