"""Sim-to-sim: the same pick-and-stack skill on two independent physics engines.

Two layers of checking:

  * Static (always runs, no PyBullet needed) -- proves both backends are
    generated from the one robot spec: identical joint chain, identical link
    offsets, identical stack plan, identical executor contract, and -- driven
    through a stub `pybullet` module -- that every PyBullet call the backend
    makes exists with the right name, keyword arguments and return-tuple
    indices. This is what catches a drifting URDF on a machine where PyBullet
    cannot be built.

  * Dynamic (runs wherever PyBullet is importable, i.e. Linux CI) -- runs
    pick_and_stack on MuJoCo and on Bullet and requires the two engines to
    agree on the verdict, the failure reason, the grasp state, the lift
    distance and the stack geometry.

PyBullet publishes a source distribution only, so it compiles on Linux CI but
generally not on a stock Windows box. The dynamic layer skips there rather
than pretending to pass.

Settlement safety: the bridge (laok_stack_arm_001_zenoh_bridge._execute) sets
`settlementEligible = bool(result.success)`, so "a failure never settles" is
exactly "a failed run never reports success". The dynamic layer asserts that
invariant for every PyBullet failure scenario; the MuJoCo side of the same
gate is covered end-to-end by TestRealMuJoCoCorrelated in test_bridge.py,
which drives the real Bridge and asserts settlementEligible is False.
"""
import os
import sys
import unittest
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "bridge"))
sys.path.insert(0, HERE)

import arm_spec  # noqa: E402
import simulator_pybullet as pbsim  # noqa: E402

try:  # the static layer must not require a physics engine at all
    import simulator as mjsim
    from simulator import MuJoCoSimulator
    HAVE_MUJOCO = True
except Exception:  # pragma: no cover
    mjsim = None
    MuJoCoSimulator = None
    HAVE_MUJOCO = False

# The real failure parameters simulator.py reacts to -- not synthetic flags.
CASES = {
    "cube": {},                                # nominal: A stacked on B
    "unreachable": {"cube": [0.95, 0]},        # A outside the work envelope
    "collision": {"obstacle": [0.27, 0]},      # column blocks the approach
    "timeout": {"budget": 30},                 # step budget clipped mid-approach
}
CASE_NAMES = tuple(CASES)
FAILING = ("unreachable", "collision", "timeout")


class TestSpecIsSingleSource(unittest.TestCase):
    """No physics required -- both backends must describe the same machine."""

    def setUp(self):
        self.urdf = ET.fromstring(pbsim._robot_urdf())

    def test_urdf_is_wellformed_and_named(self):
        self.assertEqual(self.urdf.get("name"), "stack-arm-001")

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

    def test_shoulder_pivot_sits_at_the_spec_height(self):
        """base plate + column must add up to BASE_H, or forward() lies."""
        origins = {j.get("name"): j.find("origin").get("xyz")
                   for j in self.urdf.findall("joint")}
        pan_z = float(origins["pan"].split()[2])
        shoulder_z = float(origins["shoulder"].split()[2])
        self.assertAlmostEqual(pan_z + shoulder_z, arm_spec.BASE_H, places=9)

    def test_gripper_axes_are_opposed(self):
        axes = {j.get("name"): j.find("axis").get("xyz")
                for j in self.urdf.findall("joint")}
        self.assertEqual(axes["grip_l"], "0 1 0")
        self.assertEqual(axes["grip_r"], "0 -1 0")

    def test_finger_pads_are_spec_sized(self):
        links = {l.get("name"): l for l in self.urdf.findall("link")}
        for name in ("finger_l", "finger_r"):
            size = links[name].find("collision/geometry/box").get("size")
            sx, sy, sz = (float(v) for v in size.split())
            self.assertAlmostEqual(sx, 2 * arm_spec.FINGER_HALF_X, places=9)
            self.assertAlmostEqual(sy, 2 * arm_spec.PAD_HALF, places=9)
            self.assertAlmostEqual(sz, 2 * arm_spec.FINGER_HALF_Z, places=9)

    def test_keyframes_are_solved_not_guessed(self):
        """Grasp frame must place the pads around cube A's centre of mass."""
        _x, _y, z = arm_spec.forward(arm_spec.KEYFRAMES["grasp"])
        self.assertAlmostEqual(z - arm_spec.GRIP_MID, arm_spec.CUBE_HALF, places=3)

    def test_stack_target_is_reachable_and_solved(self):
        """The place pose over cube B is solved from the spec, not tabulated."""
        import math
        r_b = math.hypot(*pbsim.CUBE_B_POS)
        at_b = arm_spec.solve(r_b, arm_spec.GRASP_WZ)
        above_b = arm_spec.solve(r_b, arm_spec.GRASP_WZ + 0.14)
        self.assertIsNotNone(at_b)
        self.assertIsNotNone(above_b)
        x, y, z = arm_spec.forward(at_b)
        self.assertAlmostEqual(math.hypot(x, y), r_b, places=6)
        self.assertAlmostEqual(z - arm_spec.GRIP_MID, arm_spec.CUBE_HALF, places=6)

    @unittest.skipUnless(HAVE_MUJOCO, "mujoco not installed")
    def test_backends_share_one_contract(self):
        for cls in (MuJoCoSimulator, pbsim.PyBulletSimulator):
            self.assertEqual(cls.ROBOT_ID, "stack-arm-001")
            self.assertEqual(cls.SKILL_ID, "pick_and_stack")
            self.assertTrue(callable(cls.pick_and_stack))
        self.assertEqual({MuJoCoSimulator.ENGINE, pbsim.PyBulletSimulator.ENGINE},
                         {"mujoco", "pybullet"})

    @unittest.skipUnless(HAVE_MUJOCO, "mujoco not installed")
    def test_stack_plan_cannot_drift_between_backends(self):
        """Stage budgets and the stacking base position are one definition."""
        self.assertEqual(pbsim.STACK_STEPS, mjsim.STACK_STEPS)
        self.assertEqual(tuple(pbsim.CUBE_B_POS), tuple(mjsim.CUBE_B_POS))


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
        return pbsim.PyBulletSimulator().pick_and_stack(dict(CASES[case]))

    def test_success_path_completes(self):
        r = self._run("cube")
        self.assertTrue(r.success, r.to_dict())
        self.assertEqual(r.reason, "stacked")
        self.assertEqual(r.metrics["engine"], "pybullet")
        self.assertEqual(r.metrics["graspState"], "stacked")
        self.assertEqual(r.metrics["stage"], "verify")
        self.assertGreater(r.metrics["objectLifted"], arm_spec.LIFT_MIN)
        self.assertGreater(r.metrics["contactForce"], 0.0)

    def test_stack_evidence_fields_are_promoted(self):
        """execution-mapping successEvidence resolves against real metric keys."""
        m = self._run("cube").metrics
        for key in ("stackStable", "a_z", "b_z", "stackOffsetXY"):
            self.assertIn(key, m)
        self.assertTrue(m["stackStable"])
        self.assertGreater(m["a_z"], m["b_z"] + 0.02)
        self.assertLess(m["stackOffsetXY"], 0.06)

    def test_unreachable_path_completes(self):
        r = self._run("unreachable")
        self.assertFalse(r.success)
        self.assertEqual(r.reason, "unreachable")
        self.assertEqual(r.metrics["stage"], "stretch")

    def test_collision_path_completes(self):
        r = self._run("collision")
        self.assertFalse(r.success)
        self.assertEqual(r.reason, "collision")
        self.assertGreaterEqual(r.metrics["collisionCount"], 1)

    def test_timeout_path_completes(self):
        r = self._run("timeout")
        self.assertFalse(r.success)
        self.assertEqual(r.reason, "timeout")
        self.assertEqual(r.metrics["stepBudget"], 30)

    def test_failures_never_report_success(self):
        """settlementEligible is bool(success); a failed run must never lie."""
        for case in FAILING:
            r = self._run(case)
            self.assertFalse(r.success, case)
            self.assertFalse(r.metrics["stackStable"], case)
            self.assertNotEqual(r.metrics["graspState"], "stacked", case)

    @unittest.skipUnless(HAVE_MUJOCO, "mujoco not installed")
    def test_metric_schema_matches_mujoco(self):
        mj = MuJoCoSimulator().pick_and_stack({})
        bt = self._run("cube")
        self.assertEqual(set(mj.metrics), set(bt.metrics))
        self.assertEqual(mj.metrics["robotId"], bt.metrics["robotId"])
        self.assertEqual(mj.metrics["skillId"], bt.metrics["skillId"])

    def test_pybullet_call_surface_is_exercised(self):
        self._run("cube")
        for call in ("loadURDF", "createMultiBody",
                     "createCollisionShape", "createVisualShape",
                     "changeDynamics", "setCollisionFilterGroupMask",
                     "setJointMotorControl2", "createConstraint",
                     "changeConstraint", "removeConstraint", "disconnect"):
            self.assertIn(call, self.stub.S.calls, call)


@unittest.skipUnless(pbsim.available() and HAVE_MUJOCO,
                     "pybullet not importable (source-only wheel; runs in CI)")
class TestSimToSimAgreement(unittest.TestCase):
    """One skill definition, two solvers, one verdict."""

    @classmethod
    def setUpClass(cls):
        cls.mj = {c: MuJoCoSimulator().pick_and_stack(dict(p))
                  for c, p in CASES.items()}
        cls.bt = {c: pbsim.PyBulletSimulator().pick_and_stack(dict(p))
                  for c, p in CASES.items()}

    def test_verdicts_agree(self):
        for c in CASE_NAMES:
            self.assertEqual(self.mj[c].success, self.bt[c].success,
                             f"{c}: mujoco={self.mj[c].to_dict()} "
                             f"bullet={self.bt[c].to_dict()}")

    def test_failure_reasons_agree(self):
        for c in CASE_NAMES:
            self.assertEqual(self.mj[c].reason, self.bt[c].reason, c)

    def test_grasp_state_agrees(self):
        for c in CASE_NAMES:
            self.assertEqual(self.mj[c].metrics["graspState"],
                             self.bt[c].metrics["graspState"], c)

    def test_lift_distance_agrees(self):
        a = self.mj["cube"].metrics["objectLifted"]
        b = self.bt["cube"].metrics["objectLifted"]
        self.assertGreater(a, arm_spec.LIFT_MIN)
        self.assertGreater(b, arm_spec.LIFT_MIN)
        self.assertLess(abs(a - b), 0.03, f"lift mismatch: mujoco={a} bullet={b}")

    def test_stack_geometry_agrees(self):
        for c in CASE_NAMES:
            self.assertEqual(self.mj[c].metrics["stackStable"],
                             self.bt[c].metrics["stackStable"], c)
        a, b = self.mj["cube"].metrics, self.bt["cube"].metrics
        self.assertLess(abs(a["a_z"] - b["a_z"]), 0.03,
                        f"stack height mismatch: mujoco={a['a_z']} bullet={b['a_z']}")
        self.assertLess(b["stackOffsetXY"], 0.06)

    def test_both_engines_measure_contact_force(self):
        for eng in (self.mj, self.bt):
            self.assertGreater(eng["cube"].metrics["contactForce"], 0.0)
            self.assertEqual(eng["cube"].metrics["contactForce"] > 0,
                             eng["cube"].success)

    def test_metric_schema_is_identical(self):
        for c in CASE_NAMES:
            self.assertEqual(set(self.mj[c].metrics), set(self.bt[c].metrics), c)

    def test_engine_tag_is_reported(self):
        self.assertEqual(self.mj["cube"].metrics["engine"], "mujoco")
        self.assertEqual(self.bt["cube"].metrics["engine"], "pybullet")

    def test_failures_never_settle_on_either_engine(self):
        """The bridge settles on `success` alone, so a failure must never
        report success on either engine.

        The Bridge is hard-wired to the MuJoCo backend (it cannot be asked for
        a PyBullet run), so the end-to-end settlement gate is asserted by
        TestRealMuJoCoCorrelated.test_relay_real_stack_failure_no_settle in
        test_bridge.py. Here we prove the value that gate reads is False on
        BOTH engines for every physical failure.
        """
        for c in FAILING:
            for engine, res in (("mujoco", self.mj[c]), ("pybullet", self.bt[c])):
                self.assertFalse(res.success, f"{engine}/{c}")
                self.assertFalse(res.metrics["stackStable"], f"{engine}/{c}")
        self.assertTrue(self.mj["cube"].success)
        self.assertTrue(self.bt["cube"].success)


if __name__ == "__main__":
    unittest.main()
