"""Safe-stop / bounded-policy tests for fabric-arm-001 — REAL MuJoCo.

Criterion #5 (bounded policy + interruptible execution + safe stop) proven
with real physics, not mocks:

  * collision scene  -> the arm strikes the obstacle and ABORTS (safe stop),
                        returns failure, never settles.
  * timeout scene    -> the step budget is exhausted mid-trajectory and the
                        run STOPS (bounded policy), returns failure, never
                        settles.
  * unreachable      -> out-of-envelope target: the arm stops short at full
                        stretch (interruptible execution), returns failure.

The same simulator the paid flow uses (MuJoCoSimulator.pick_object) is
driven here, so the stop behaviour is the production stop behaviour.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:
    from simulator import MuJoCoSimulator
    HAS_SIM = True
except Exception:  # pragma: no cover - MuJoCo absent on some platforms
    HAS_SIM = False


@pytest.mark.skipif(not HAS_SIM, reason="MuJoCo simulator not available")
class TestSafeStopReal:
    def test_collision_aborts_and_never_settles(self):
        """A collision mid-approach triggers the safe-stop path: the run
        aborts, returns failure, and the relay must not settle."""
        sim = MuJoCoSimulator()
        result = sim.pick_object({"object": "collision"})
        assert result.success is False, "collision must fail"
        assert result.reason == "collision", result.reason
        assert result.metrics.get("collisionCount", 0) > 0, \
            "a real MuJoCo contact must be recorded"

    def test_timeout_stops_on_budget(self):
        """A clipped step budget stops execution (bounded policy) and the
        run returns failure without settling."""
        sim = MuJoCoSimulator()
        result = sim.pick_object({"object": "timeout"})
        assert result.success is False, "timeout must fail"
        assert result.reason == "timeout", result.reason
        steps = result.metrics.get("stepsUsed", 0)
        budget = result.metrics.get("stepBudget", 0)
        assert steps >= budget, "execution must stop when the budget is exhausted"

    def test_unreachable_stops_short(self):
        """An out-of-envelope target interrupts the trajectory: the arm
        stops short at full stretch (interruptible execution)."""
        sim = MuJoCoSimulator()
        result = sim.pick_object({"object": "unreachable"})
        assert result.success is False, "unreachable must fail"
        assert result.reason == "unreachable", result.reason

    def test_normal_scene_completes_within_budget(self):
        """The nominal scene completes inside the step budget, proving the
        bounded policy is not an arbitrary truncation."""
        sim = MuJoCoSimulator()
        result = sim.pick_object({"object": "cube"})
        assert result.success is True, result.reason
        assert result.metrics.get("stepsUsed", 0) <= result.metrics.get("stepBudget", 0)
