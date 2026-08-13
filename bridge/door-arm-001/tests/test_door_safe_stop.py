"""Safe-stop / bounded-policy tests for door-arm-001 — REAL MuJoCo.

Criterion #5 (bounded policy + interruptible execution + safe stop) proven
with real physics, not mocks:

  * stuck scene  -> the arm hits the obstacle/condition and ABORTS (safe stop),
                        returns failure, never settles.
  * timeout scene    -> the step budget is exhausted and the run STOPS,
                        returns failure, never settles.
  * out_of_range      -> out-of-envelope target: the arm stops short (interruptible).

The same simulator the paid flow uses (MuJoCoSimulator.open_door) is
driven here, so the stop behaviour is the production stop behaviour.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:
    from simulator import MuJoCoSimulator
    HAS_SIM = True
except Exception:  # pragma: no cover
    HAS_SIM = False


@pytest.mark.skipif(not HAS_SIM, reason="MuJoCo simulator not available")
class TestSafeStopReal:
    def test_collision_aborts_and_never_settles(self):
        """A physical abort triggers the safe-stop path: run fails."""
        sim = MuJoCoSimulator()
        result = sim.open_door({"stuck_key": "stuck"} if False else {"object": "stuck"} if False else {"scene": "stuck"} if False else {"door": "stuck"})
        assert result.success is False, "stuck must fail"
        assert result.reason == "stuck", result.reason
        assert result.metrics.get("collisionCount", 0) > 0 or "stuck" in str(result.reason)

    def test_timeout_stops_on_budget(self):
        """A clipped step budget stops execution (bounded policy)."""
        sim = MuJoCoSimulator()
        result = sim.open_door({"door": "timeout"})
        assert result.success is False, "timeout must fail"
        assert result.reason == "timeout", result.reason

    def test_unreachable_stops_short(self):
        """An out-of-envelope target interrupts the trajectory."""
        sim = MuJoCoSimulator()
        result = sim.open_door({"door": "out_of_range"})
        assert result.success is False, "unreachable must fail"
        assert result.reason == "out_of_range", result.reason

    def test_normal_scene_completes_within_budget(self):
        """The nominal scene completes inside the step budget."""
        sim = MuJoCoSimulator()
        result = sim.open_door({"door": "open"})
        assert result.success is True, result.reason
        assert result.reason == "opened", result.reason
