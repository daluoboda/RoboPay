"""Sim-to-sim: the same paid routing skill on two independent physics engines.

Two layers of checking:

  * Static (always runs, no PyBullet needed) -- proves both backends are
    generated from the one robot spec: identical joint chain, identical link
    offsets, identical cell layout, identical executor contract. This is what
    catches a drifting URDF on a machine where PyBullet cannot be built.

  * Dynamic (runs wherever PyBullet is importable, i.e. Linux CI) -- runs
    pick_and_sort on MuJoCo and on Bullet and requires the two engines to
    agree on the verdict, the failure reason, the grasp state, the routing
    decision and the carry height.

PyBullet publishes a source distribution only, so it compiles on Linux CI but
generally not on a stock Windows box. The dynamic layer skips there rather
than pretending to pass.

Settlement safety: this robot's bridge is hard-bound to the MuJoCo backend
(laok_sort_arm_001_zenoh_bridge.Bridge instantiates MuJoCoSimulator directly),
so there is no engine switch to abuse. The settlement rule it applies is
`settlementEligible = PickResult.success`, which TestSettlementIsGatedOnPhysics
pins down against the real bridge; every failing PyBullet case asserted below
therefore cannot settle either. test_bridge.py already covers the MuJoCo
failure path end-to-end through the bridge.
"""
import math
import os
import sys
import unittest
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
BRIDGE_DIR = os.path.join(HERE, "..", "bridge")
for _p in (BRIDGE_DIR, HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import arm_spec  # noqa: E402
import simulator as mjmod  # noqa: E402
import simulator_pybullet as pbsim  # noqa: E402
from laok_sort_arm_001_zenoh_bridge import Bridge, params_hash  # noqa: E402
from simulator import MuJoCoSimulator  # noqa: E402
from simulator_pybullet import PyBulletSimulator  # noqa: E402

# Real scenarios, expressed the way a paying client would express them:
# every entry is a physical situation, not a code branch.
CASES = {
    "route_a": {"target_bin": "A"},
    "route_b": {"target_bin": "B"},
    "unreachable": {"cube_xy": [2.0, 0.0]},          # outside WORK_R envelope
    "collision": {"obstacle_xy": [0.27, 0.0]},       # column across the approach
    "timeout": {"budget": 60},                       # clipped step budget
    "grasp_failed": {"cube_xy": [0.28, 0.0]},        # pads close on empty air
}
SUCCESS_CASES = ("route_a", "route_b")
FAILURE_REASONS = {
    "unreachable": "unreachable",
    "collision": "collision",
    "timeout": "timeout",
    "grasp_failed": "grasp_failed",
}

PAYEE = "0x742d35Cc6634C0532925a3b844Bc454e4438f44e"
ROBOT = "sort-arm-001-demo-001"


def _paid(action_id, idem, auth, params):
    """Minimal valid Pay-to-Actuate envelope (mirrors tests/test_bridge.py)."""
    pay = {
        "provider": "x402", "network": "eip155:84532",
        "asset": "0x036CbD53842c5426634e7929541eC2318f3dCF7e",
        "amount": "100000", "payTo": PAYEE,
        "authorizationId": auth, "verified": True, "status": "authorized",
        "settled": False, "issuedAt": "2099-01-01T00:00:00Z",
        "expiresAt": "2099-01-01T00:05:00Z",
    }
    return {"actionId": action_id, "robotId": ROBOT,
            "skillId": "sort_arm_pick_and_sort", "params": params,
            "paramsHash": params_hash(params), "idempotencyKey": idem,
            "payment": pay}


class TestSpecIsSingleSource(unittest.TestCase):
    """No physics required -- both backends must describe the same machine."""

    def setUp(self):
        self.urdf = ET.fromstring(pbsim._robot_urdf())

    def test_urdf_is_wellformed_and_named(self):
        self.assertEqual(self.urdf.get("name"), "sort-arm-001")

    def test_joint_chain_matches_mjcf(self):
        names = [j.get("name") for j in self.urdf.findall("joint")]
        self.assertEqual(names,
                         list(arm_spec.ARM_JOINTS) + ["grip_l", "grip_r"])

    def test_link_offsets_come_from_the_spec(self):
        origins = {j.get("name"): j.find("origin").get("xyz")
                   for j in self.urdf.findall("joint")}
        self.assertEqual(origins["elbow"].split()[0], str(arm_spec.LINK1))
        self.assertEqual(origins["wristp"].split()[0], str(arm_spec.LINK2))
        self.assertEqual(origins["grip_l"].split()[2], f"-{arm_spec.GRIP_MID}")
        self.assertEqual(origins["grip_r"].split()[2], f"-{arm_spec.GRIP_MID}")

    def test_gripper_axes_are_opposed(self):
        axes = {j.get("name"): j.find("axis").get("xyz")
                for j in self.urdf.findall("joint")}
        self.assertEqual(axes["grip_l"], "0 1 0")
        self.assertEqual(axes["grip_r"], "0 -1 0")

    def test_finger_pads_are_spec_sized(self):
        links = {l.get("name"): l for l in self.urdf.findall("link")}
        size = links["finger_l"].find("collision/geometry/box").get("size")
        self.assertEqual(size, f"{2 * arm_spec.FINGER_HALF_X} "
                               f"{2 * arm_spec.PAD_HALF} "
                               f"{2 * arm_spec.FINGER_HALF_Z}")

    def test_backends_share_one_contract(self):
        for cls in (MuJoCoSimulator, PyBulletSimulator):
            self.assertEqual(cls.ROBOT_ID, "sort-arm-001")
            self.assertEqual(cls.SKILL_ID, "pick_and_sort")
            self.assertTrue(callable(cls.pick_and_sort))
        self.assertEqual({mjmod.ENGINE, pbsim.ENGINE}, {"mujoco", "pybullet"})

    def test_keyframes_are_solved_not_guessed(self):
        """Grasp frame must place the pads around the object's centre of mass."""
        _x, _y, z = arm_spec.forward(arm_spec.KEYFRAMES["grasp"])
        self.assertAlmostEqual(z - arm_spec.GRIP_MID, arm_spec.CUBE_HALF, places=3)


class TestSceneLayoutIsShared(unittest.TestCase):
    """The routing cell is declared twice (MJCF and URDF world); drift is a bug.

    simulator_pybullet mirrors the bin/lane layout instead of importing it, so
    that the Bullet backend never drags MuJoCo into its import graph. This test
    is the lock that keeps the two declarations identical.
    """

    def test_bin_positions_match(self):
        self.assertEqual(tuple(pbsim.BIN_A_XY), tuple(mjmod.BIN_A_XY))
        self.assertEqual(tuple(pbsim.BIN_B_XY), tuple(mjmod.BIN_B_XY))

    def test_incoming_lane_matches(self):
        self.assertEqual(tuple(pbsim.INCOMING_XY), tuple(mjmod.INCOMING_XY))

    def test_sort_stage_lengths_match(self):
        self.assertEqual(pbsim.SORT_STEPS, mjmod.SORT_STEPS)

    def test_bin_site_height_matches_the_mjcf(self):
        """MuJoCo reads data.site_xpos; the Bullet twin recomputes it."""
        sim = MuJoCoSimulator()
        sim.pick_and_sort({"cube_xy": [2.0, 0.0]})   # cheap build, no motion
        for bin_id in ("A", "B"):
            mj_site = sim._bin_pos(bin_id)
            pb_site = (pbsim.BIN_A_XY if bin_id == "A" else pbsim.BIN_B_XY)
            self.assertAlmostEqual(float(mj_site[0]), pb_site[0], places=6)
            self.assertAlmostEqual(float(mj_site[1]), pb_site[1], places=6)
            self.assertAlmostEqual(float(mj_site[2]),
                                   pbsim.BIN_BODY_Z + pbsim.BIN_SITE_Z, places=6)

    def test_route_tolerance_matches(self):
        """Both backends must call the same landing radius a success."""
        src = open(os.path.join(BRIDGE_DIR, "simulator.py"),
                   encoding="utf-8").read()
        msg = (f"simulator.py no longer decides routing at "
               f"{pbsim.ROUTE_TOLERANCE} m -- update ROUTE_TOLERANCE in "
               f"simulator_pybullet.py to match")
        self.assertIn(f"accuracy < {pbsim.ROUTE_TOLERANCE}", src, msg)
        self.assertIn(f"accuracy > {pbsim.ROUTE_TOLERANCE}", src, msg)


@unittest.skipIf(pbsim.available(), "real pybullet present; stub not needed")
class TestPyBulletBackendContract(unittest.TestCase):
    """Walk every PyBullet call the backend makes, without PyBullet.

    Catches misspelled functions, wrong keyword names and wrong return-tuple
    indices on developer machines where the wheel cannot be built. Physics
    agreement is asserted separately by TestSimToSimAgreement on CI.
    """

    def setUp(self):
        import bullet_stub as stub
        self._saved = sys.modules.get("pybullet")
        sys.modules["pybullet"] = stub
        self.stub = stub

    def tearDown(self):
        if self._saved is None:
            sys.modules.pop("pybullet", None)
        else:                                          # pragma: no cover
            sys.modules["pybullet"] = self._saved

    def _run(self, case):
        return PyBulletSimulator().pick_and_sort(dict(CASES[case]))

    def test_success_path_completes(self):
        r = self._run("route_a")
        self.assertTrue(r.success, r.to_dict())
        self.assertEqual(r.reason, "routed")
        self.assertEqual(r.metrics["engine"], "pybullet")
        self.assertEqual(r.metrics["graspState"], "released")
        self.assertEqual(r.metrics["targetBin"], "A")
        self.assertTrue(r.metrics["routed"])
        self.assertLess(r.metrics["accuracy"], pbsim.ROUTE_TOLERANCE)
        self.assertGreater(r.metrics["peakLift"], 0.05)
        self.assertGreater(r.metrics["contactForce"], 0.0)
        self.assertEqual(r.metrics["collisionCount"], 0)

    def test_second_bin_is_parameter_driven(self):
        r = self._run("route_b")
        self.assertTrue(r.success, r.to_dict())
        self.assertEqual(r.metrics["targetBin"], "B")
        self.assertEqual(r.metrics["scene"], "to_bin_B")
        self.assertTrue(r.metrics["routed"])

    def test_unreachable_path_completes(self):
        r = self._run("unreachable")
        self.assertFalse(r.success)
        self.assertEqual(r.reason, "unreachable")
        self.assertEqual(r.metrics["stage"], "stretch")

    def test_collision_path_completes(self):
        r = self._run("collision")
        self.assertFalse(r.success)
        self.assertEqual(r.reason, "collision")
        self.assertGreater(r.metrics["collisionCount"], 0)

    def test_timeout_path_completes(self):
        r = self._run("timeout")
        self.assertFalse(r.success)
        self.assertEqual(r.reason, "timeout")
        self.assertEqual(r.metrics["stepsUsed"], r.metrics["stepBudget"])

    def test_grasp_failed_path_completes(self):
        r = self._run("grasp_failed")
        self.assertFalse(r.success)
        self.assertEqual(r.reason, "grasp_failed")
        self.assertEqual(r.metrics["graspState"], "slipped")

    def test_metric_schema_matches_mujoco(self):
        mj = MuJoCoSimulator().pick_and_sort({})
        bt = self._run("route_a")
        self.assertEqual(set(mj.metrics), set(bt.metrics))

    def test_trajectory_accounting_matches_mujoco(self):
        """The plan is scripted, so both engines must spend the same steps."""
        mj = MuJoCoSimulator().pick_and_sort({})
        bt = self._run("route_a")
        for field in ("stepsUsed", "stepBudget", "stage", "scene",
                      "robotId", "skillId", "object"):
            self.assertEqual(mj.metrics[field], bt.metrics[field], field)

    def test_routing_evidence_fields_are_promoted(self):
        """execution-mapping.yaml resolves successEvidence against real keys."""
        bt = self._run("route_a")
        for field in ("targetBin", "routed", "accuracy", "peakLift"):
            self.assertIn(field, bt.metrics)

    def test_constraint_and_urdf_calls_were_made(self):
        self._run("route_a")
        for call in ("loadURDF", "createConstraint", "changeConstraint",
                     "removeConstraint", "setCollisionFilterGroupMask",
                     "setJointMotorControl2"):
            self.assertIn(call, self.stub.S.calls, call)

    def test_failed_runs_are_never_settlement_eligible(self):
        """The bridge settles on PickResult.success; failures must be False."""
        for case in FAILURE_REASONS:
            r = self._run(case)
            self.assertFalse(r.success, case)
            self.assertFalse(r.metrics["routed"], case)


class TestSettlementIsGatedOnPhysics(unittest.TestCase):
    """Pin the settlement rule the sim-to-sim argument leans on.

    The bridge derives `settlementEligible` from the simulator verdict alone,
    so a failure on ANY backend is unsettleable. Verified here against the real
    bridge with the MuJoCo backend it is hard-bound to; test_bridge.py carries
    the wider payment-gate coverage.
    """

    def test_physical_failure_does_not_settle(self):
        b = Bridge(ROBOT, PAYEE, ":memory:")
        code, _ = b.request_action(
            _paid("s2s-fail", "k-s2s-fail", "auth-s2s-fail",
                  {"cube_xy": [2.0, 0.0]}))
        self.assertEqual(code, 202)
        rec = b.actions["s2s-fail"]
        self.assertEqual(rec["status"], "error")
        self.assertFalse(rec["settlementEligible"])
        self.assertEqual(rec["reason"], "unreachable")

    def test_physical_success_settles(self):
        b = Bridge(ROBOT, PAYEE, ":memory:")
        code, _ = b.request_action(
            _paid("s2s-ok", "k-s2s-ok", "auth-s2s-ok", {"target_bin": "B"}))
        self.assertEqual(code, 202)
        rec = b.actions["s2s-ok"]
        self.assertEqual(rec["status"], "success")
        self.assertTrue(rec["settlementEligible"])
        self.assertEqual(rec["metrics"]["targetBin"], "B")


@unittest.skipUnless(pbsim.available(),
                     "pybullet not importable (source-only wheel; runs in CI)")
class TestSimToSimAgreement(unittest.TestCase):
    """One skill definition, two engines, one verdict."""

    @classmethod
    def setUpClass(cls):
        cls.mj = {c: MuJoCoSimulator().pick_and_sort(dict(p))
                  for c, p in CASES.items()}
        cls.bt = {c: PyBulletSimulator().pick_and_sort(dict(p))
                  for c, p in CASES.items()}

    def test_verdicts_agree(self):
        for c in CASES:
            self.assertEqual(self.mj[c].success, self.bt[c].success,
                             f"{c}: mujoco={self.mj[c].to_dict()} "
                             f"bullet={self.bt[c].to_dict()}")

    def test_failure_reasons_agree(self):
        for c in CASES:
            self.assertEqual(self.mj[c].reason, self.bt[c].reason, c)
        for c, reason in FAILURE_REASONS.items():
            self.assertEqual(self.bt[c].reason, reason, c)

    def test_grasp_state_agrees(self):
        for c in CASES:
            self.assertEqual(self.mj[c].metrics["graspState"],
                             self.bt[c].metrics["graspState"], c)
        for c in SUCCESS_CASES:
            self.assertEqual(self.bt[c].metrics["graspState"], "released", c)

    def test_routing_decision_agrees(self):
        for c in CASES:
            self.assertEqual(self.mj[c].metrics["routed"],
                             self.bt[c].metrics["routed"], c)
            self.assertEqual(self.mj[c].metrics["targetBin"],
                             self.bt[c].metrics["targetBin"], c)
        for c in SUCCESS_CASES:
            self.assertLess(self.bt[c].metrics["accuracy"],
                            pbsim.ROUTE_TOLERANCE, c)
            self.assertLess(self.mj[c].metrics["accuracy"],
                            pbsim.ROUTE_TOLERANCE, c)

    def test_carry_height_agrees(self):
        """objectLifted is ~0 by design (the object is put back down), so the
        engines are compared on peakLift -- the height it actually travelled."""
        for c in SUCCESS_CASES:
            a = self.mj[c].metrics["peakLift"]
            b = self.bt[c].metrics["peakLift"]
            self.assertGreater(a, arm_spec.LIFT_MIN, c)
            self.assertGreater(b, arm_spec.LIFT_MIN, c)
            self.assertLess(abs(a - b), 0.03,
                            f"{c}: peak lift mismatch mujoco={a} bullet={b}")

    def test_final_object_height_agrees(self):
        for c in SUCCESS_CASES:
            a = self.mj[c].metrics["objectLifted"]
            b = self.bt[c].metrics["objectLifted"]
            self.assertLess(abs(a - b), 0.03,
                            f"{c}: lift mismatch mujoco={a} bullet={b}")

    def test_both_engines_measure_contact_force(self):
        for eng in (self.mj, self.bt):
            for c in SUCCESS_CASES:
                self.assertGreater(eng[c].metrics["contactForce"], 0.0, c)
                self.assertGreaterEqual(eng[c].metrics["contactSamples"], 1, c)

    def test_scripted_trajectory_costs_the_same_steps(self):
        for c in SUCCESS_CASES:
            self.assertEqual(self.mj[c].metrics["stepsUsed"],
                             self.bt[c].metrics["stepsUsed"], c)

    def test_metric_schema_is_identical(self):
        for c in CASES:
            self.assertEqual(set(self.mj[c].metrics), set(self.bt[c].metrics), c)

    def test_engine_tag_is_reported(self):
        for c in CASES:
            self.assertEqual(self.mj[c].metrics["engine"], "mujoco", c)
            self.assertEqual(self.bt[c].metrics["engine"], "pybullet", c)

    def test_failures_never_settle_on_either_engine(self):
        """The bridge is hard-bound to MuJoCo, so the PyBullet guarantee is
        that it produces the same unsettleable verdict for the same scene."""
        for c in FAILURE_REASONS:
            self.assertFalse(self.bt[c].success, c)
            self.assertFalse(self.bt[c].metrics["routed"], c)
            b = Bridge(ROBOT, PAYEE, ":memory:")
            b.request_action(_paid(f"s2s-{c}", f"k-s2s-{c}", f"auth-s2s-{c}",
                                   dict(CASES[c])))
            self.assertFalse(b.actions[f"s2s-{c}"]["settlementEligible"], c)


if __name__ == "__main__":
    unittest.main()
