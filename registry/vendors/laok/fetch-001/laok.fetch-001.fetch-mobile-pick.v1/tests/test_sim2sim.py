"""fetch-001 sim-to-sim: the same skill on two independent physics engines.

`fetch_mobile_pick` is a 7-phase pick-AND-place: approach, descend, contact-
gated grip, lift, traverse to the shelf, place + release, verify. A single
engine can make any of those phases look good by accident. Two engines
agreeing on the verdict, the failure reason, the grasp state and the lift
distance cannot.

Two layers of checking:

  * Static (always runs, no PyBullet needed) -- proves both backends are
    generated from the one robot spec: identical joint chain, identical link
    offsets, identical placement heights, identical executor contract, and a
    stubbed walk through every PyBullet call the backend makes. This is what
    catches a drifting URDF on a machine where PyBullet cannot be built.

  * Dynamic (runs wherever PyBullet is importable, i.e. Linux CI) -- runs
    fetch_mobile_pick on MuJoCo and on Bullet for the same six scenes and
    requires the two engines to agree.

PyBullet publishes a source distribution only, so it compiles on Linux CI but
generally not on a stock Windows box. The dynamic layer skips there rather
than pretending to pass.

Settlement note: this profile's Bridge is hard-bound to the MuJoCo backend
(`laok_fetch_001_zenoh_bridge` imports MuJoCoSimulator directly), so there is
no paid code path that can reach PyBullet. The "failure never settles"
guarantee for the paid path is covered by
`test_bridge.py::TestRealMuJoCoCorrelated::test_relay_real_fetch_failure_no_settle`;
what this file adds is that a PyBullet failure is a genuine `success == False`
result -- i.e. the second engine reproduces the failure rather than papering
over it -- plus a static assertion that the paid path really is MuJoCo-only.
"""
import os
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "bridge"))

import arm_spec  # noqa: E402
import simulator as mjsim  # noqa: E402
import simulator_pybullet as pbsim  # noqa: E402
from simulator import MuJoCoSimulator  # noqa: E402
from simulator_pybullet import PyBulletSimulator  # noqa: E402

try:
    import mujoco  # noqa: F401
    HAVE_MUJOCO = True
except Exception:                                      # pragma: no cover
    HAVE_MUJOCO = False

# Every case is a physical scene, not a code branch. Parameters are the raw
# scene dicts the skill accepts (see simulator.fetch_mobile_pick), and each one
# was confirmed against the MuJoCo backend before being pinned here.
CASES = {
    # nominal pick-and-place onto the shelf
    "nominal": {},
    # cube A parked outside the 0.35 m envelope -> refused before any motion
    "unreachable": {"cube_a": [0.95, 0.0]},
    # pillar planted on the approach line -> struck during move_above_a
    "collision": {"obstacle": [0.27, 0.0]},
    # budget below the 450-step trajectory -> exhausted mid-approach
    "timeout": {"budget": 60},
    # cube A reachable but 70 mm off the programmed grasp pose -> pads close
    # on air
    "grasp_failed": {"cube_a": [0.28, 0.0]},
    # shelf displaced in Y while the arm still places at pan=0 -> release
    # happens off the shelf
    "place_failed": {"shelf": [0.28, 0.15]},
}
EXPECTED_REASON = {
    "nominal": "placed",
    "unreachable": "unreachable",
    "collision": "collision",
    "timeout": "timeout",
    "grasp_failed": "grasp_failed",
    "place_failed": "place_failed",
}
EXPECTED_GRASP = {
    "nominal": "placed",
    "unreachable": "open",
    "collision": "open",
    "timeout": "open",
    "grasp_failed": "slipped",
    "place_failed": "off_target",
}
FAILING = tuple(k for k in CASES if k != "nominal")


class TestSpecIsSingleSource(unittest.TestCase):
    """No physics required -- both backends must describe the same machine."""

    def setUp(self):
        self.urdf = ET.fromstring(pbsim._robot_urdf())

    def test_urdf_is_wellformed_and_named(self):
        self.assertEqual(self.urdf.get("name"), "fetch-001")

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

    def test_shoulder_pivot_sits_at_spec_height(self):
        """base(0.05) + column(0.35) must land on arm_spec.BASE_H."""
        origins = {j.get("name"): j.find("origin").get("xyz")
                   for j in self.urdf.findall("joint")}
        pan_z = float(origins["pan"].split()[2])
        shoulder_z = float(origins["shoulder"].split()[2])
        self.assertAlmostEqual(pan_z + shoulder_z, arm_spec.BASE_H, places=6)

    def test_gripper_axes_are_opposed(self):
        axes = {j.get("name"): j.find("axis").get("xyz")
                for j in self.urdf.findall("joint")}
        self.assertEqual(axes["grip_l"], "0 1 0")
        self.assertEqual(axes["grip_r"], "0 -1 0")

    def test_gripper_travel_matches_the_aperture_plan(self):
        limits = {j.get("name"): j.find("limit") for j in self.urdf.findall("joint")}
        for name in ("grip_l", "grip_r"):
            lo = float(limits[name].get("lower"))
            hi = float(limits[name].get("upper"))
            self.assertLessEqual(lo, arm_spec.FINGER_CLOSED)
            self.assertGreaterEqual(hi, arm_spec.FINGER_OPEN)

    def test_backends_share_one_contract(self):
        for cls in (MuJoCoSimulator, PyBulletSimulator):
            self.assertEqual(cls.ROBOT_ID, "fetch-001")
            self.assertEqual(cls.SKILL_ID, "fetch_mobile_pick")
            self.assertTrue(callable(cls.fetch_mobile_pick))
        self.assertEqual(MuJoCoSimulator.ENGINE, "mujoco")
        self.assertEqual(PyBulletSimulator.ENGINE, "pybullet")

    def test_skill_signatures_are_identical(self):
        import inspect
        mj = inspect.signature(MuJoCoSimulator.fetch_mobile_pick)
        bt = inspect.signature(PyBulletSimulator.fetch_mobile_pick)
        self.assertEqual(list(mj.parameters), list(bt.parameters))

    def test_keyframes_are_solved_not_guessed(self):
        """The grasp frame must place the pads around cube A's centre."""
        _x, _y, z = arm_spec.forward(arm_spec.KEYFRAMES["grasp"])
        self.assertAlmostEqual(z - arm_spec.GRIP_MID, arm_spec.CUBE_HALF, places=3)

    def test_place_heights_match_mujoco_backend(self):
        """Both backends must release cube A at exactly the same height.

        PLACE_WZ / CLEAR_WZ are re-derived in the PyBullet module (importing
        simulator.py would drag MuJoCo into a PyBullet-only environment), so
        this test is the drift guard that keeps the two derivations equal.
        """
        self.assertEqual(pbsim.PLACE_WZ, mjsim.PLACE_WZ)
        self.assertEqual(pbsim.CLEAR_WZ, mjsim.CLEAR_WZ)
        self.assertEqual(pbsim.SHELF_XY, mjsim.SHELF_XY)

    def test_place_height_is_derived_from_the_shelf_top(self):
        self.assertAlmostEqual(arm_spec.SHELF_TOP,
                               arm_spec.SHELF_H + arm_spec.SHELF_HALF, places=9)
        self.assertAlmostEqual(
            pbsim.PLACE_WZ,
            arm_spec.SHELF_TOP + arm_spec.CUBE_HALF + arm_spec.GRIP_MID + 0.004,
            places=9)

    def test_second_engine_stands_alone(self):
        """The PyBullet backend must not depend on MuJoCo or on simulator.py.

        A CI box that installs only PyBullet has to be able to import it, and
        more importantly the two backends must not be able to share a bug.
        """
        src = open(pbsim.__file__, encoding="utf-8").read()
        self.assertNotIn("import mujoco", src)
        self.assertNotIn("from simulator import", src)
        self.assertNotIn("import simulator\n", src)

    @unittest.skipUnless(HAVE_MUJOCO, "mujoco not installed")
    def test_paid_path_is_mujoco_only(self):
        """Documents why there is no PyBullet settlement test here.

        The bridge instantiates MuJoCoSimulator directly, so no paid request
        can ever be served by the second engine. Failure-never-settles for the
        paid path is asserted in test_bridge.py.
        """
        from laok_fetch_001_zenoh_bridge import Bridge
        b = Bridge("fetch-001-demo-001",
                   "0x742d35Cc6634C0532925a3b844Bc454e4438f44e", ":memory:")
        self.assertIsInstance(b.sim, MuJoCoSimulator)
        self.assertEqual(b.sim.ENGINE, "mujoco")


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
        return PyBulletSimulator().fetch_mobile_pick(dict(CASES[case]))

    def test_success_path_completes(self):
        r = self._run("nominal")
        self.assertTrue(r.success, r.to_dict())
        self.assertEqual(r.reason, "placed")
        self.assertEqual(r.metrics["engine"], "pybullet")
        self.assertEqual(r.metrics["robotId"], "fetch-001")
        self.assertEqual(r.metrics["skillId"], "fetch_mobile_pick")
        self.assertEqual(r.metrics["graspState"], "placed")
        self.assertGreater(r.metrics["objectLifted"], arm_spec.LIFT_MIN)
        self.assertGreater(r.metrics["contactForce"], 0.0)
        self.assertTrue(r.metrics["placeStable"])
        self.assertGreater(r.metrics["a_z"], r.metrics["shelf_z"] + 0.02)

    def test_every_failure_path_completes(self):
        for case in FAILING:
            with self.subTest(case=case):
                r = self._run(case)
                self.assertFalse(r.success, r.to_dict())
                self.assertEqual(r.reason, EXPECTED_REASON[case])
                self.assertEqual(r.metrics["graspState"], EXPECTED_GRASP[case])
                self.assertFalse(r.metrics["placeStable"])

    def test_unreachable_is_refused_before_any_motion(self):
        r = self._run("unreachable")
        self.assertEqual(r.metrics["stage"], "stretch")
        self.assertEqual(r.metrics["stepsUsed"], 0)

    def test_timeout_respects_the_supplied_budget(self):
        r = self._run("timeout")
        self.assertEqual(r.metrics["stepBudget"], 60)
        self.assertEqual(r.metrics["stepsUsed"], 60)

    def test_collision_is_counted_not_just_reported(self):
        r = self._run("collision")
        self.assertGreater(r.metrics["collisionCount"], 0)

    @unittest.skipUnless(HAVE_MUJOCO, "mujoco not installed")
    def test_metric_schema_matches_mujoco(self):
        for case in CASES:
            with self.subTest(case=case):
                mj = MuJoCoSimulator().fetch_mobile_pick(dict(CASES[case]))
                bt = self._run(case)
                self.assertEqual(set(mj.metrics), set(bt.metrics))

    def test_constraint_and_urdf_calls_were_made(self):
        self._run("nominal")
        for call in ("loadURDF", "createConstraint", "changeConstraint",
                     "removeConstraint", "setCollisionFilterGroupMask",
                     "setJointMotorControl2", "createMultiBody"):
            self.assertIn(call, self.stub.S.calls, call)

    def test_release_actually_detaches(self):
        """A place that never lets go is not a place."""
        self._run("nominal")
        self.assertFalse(self.stub.S.attached)
        self.assertEqual(self.stub.S.constraints, set())

    def test_shelf_and_obstacle_are_distinct_bodies(self):
        self._run("collision")
        self.assertIsNotNone(self.stub.S.shelf)
        self.assertIsNotNone(self.stub.S.obstacle)
        self.assertNotEqual(self.stub.S.shelf, self.stub.S.obstacle)
        self.assertNotEqual(self.stub.S.shelf, self.stub.S.cube_a)


@unittest.skipUnless(pbsim.available(),
                     "pybullet not importable (source-only wheel; runs in CI)")
class TestSimToSimAgreement(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.mj = {c: MuJoCoSimulator().fetch_mobile_pick(dict(p))
                  for c, p in CASES.items()}
        cls.bt = {c: PyBulletSimulator().fetch_mobile_pick(dict(p))
                  for c, p in CASES.items()}

    def test_verdicts_agree(self):
        for c in CASES:
            self.assertEqual(self.mj[c].success, self.bt[c].success,
                             f"{c}: mujoco={self.mj[c].to_dict()} "
                             f"bullet={self.bt[c].to_dict()}")

    def test_verdicts_match_the_pinned_expectations(self):
        """Agreement is worthless if both engines agree on the wrong thing."""
        for c in CASES:
            self.assertEqual(self.mj[c].reason, EXPECTED_REASON[c], f"mujoco/{c}")
            self.assertEqual(self.bt[c].reason, EXPECTED_REASON[c], f"bullet/{c}")

    def test_failure_reasons_agree(self):
        for c in CASES:
            self.assertEqual(self.mj[c].reason, self.bt[c].reason, c)

    def test_grasp_state_agrees(self):
        for c in CASES:
            self.assertEqual(self.mj[c].metrics["graspState"],
                             self.bt[c].metrics["graspState"], c)

    def test_lift_distance_agrees(self):
        a = self.mj["nominal"].metrics["objectLifted"]
        b = self.bt["nominal"].metrics["objectLifted"]
        self.assertGreater(a, arm_spec.LIFT_MIN)
        self.assertGreater(b, arm_spec.LIFT_MIN)
        self.assertLess(abs(a - b), 0.03, f"lift mismatch: mujoco={a} bullet={b}")

    def test_placement_agrees(self):
        for c in CASES:
            self.assertEqual(self.mj[c].metrics["placeStable"],
                             self.bt[c].metrics["placeStable"], c)
        for eng in (self.mj, self.bt):
            m = eng["nominal"].metrics
            self.assertGreater(m["a_z"], m["shelf_z"] + 0.02)
            self.assertLess(m["xyOffset"], 0.12)

    def test_both_engines_measure_contact_force(self):
        for eng in (self.mj, self.bt):
            self.assertGreater(eng["nominal"].metrics["contactForce"], 0.0)
            self.assertEqual(eng["nominal"].metrics["contactForce"] > 0,
                             eng["nominal"].success)

    def test_metric_schema_is_identical(self):
        for c in CASES:
            self.assertEqual(set(self.mj[c].metrics), set(self.bt[c].metrics), c)

    def test_engine_tag_is_reported(self):
        self.assertEqual(self.mj["nominal"].metrics["engine"], "mujoco")
        self.assertEqual(self.bt["nominal"].metrics["engine"], "pybullet")

    def test_failures_are_real_failures_on_both_engines(self):
        """A failure on either engine must read as a failure, never a success.

        The paid path is MuJoCo-only (see TestSpecIsSingleSource
        .test_paid_path_is_mujoco_only), and test_bridge.py already proves a
        MuJoCo failure leaves settlementEligible False. This closes the loop
        for the second engine: PyBullet reproduces each failure instead of
        quietly succeeding, so no scene can settle on one engine and fail on
        the other.
        """
        for c in FAILING:
            self.assertFalse(self.bt[c].success, f"bullet/{c}")
            self.assertFalse(self.mj[c].success, f"mujoco/{c}")
            self.assertFalse(self.bt[c].metrics["placeStable"], f"bullet/{c}")


if __name__ == "__main__":
    unittest.main()
