"""Behavioral tests for the laok fetch-001 RoboPay bridge.

Exercises the Pay-to-Actuate contract end-to-end against the Bridge core:
402 challenge, async accepted, MuJoCo execution, params-hash, expiry,
idempotency, and replay protection.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "bridge"))
from laok_fetch_001_zenoh_bridge import Bridge, params_hash  # noqa: E402

PAYEE = "0x742d35Cc6634C0532925a3b844Bc454e4438f44e"
ROBOT = "fetch-001-demo-001"


def paid(action_id, idem, auth, verified=True, expired=False, amount="100000", params=None):
    if params is None:
        params = {}
    pay = {
        "provider": "x402", "network": "eip155:84532",
        "asset": "0x036CbD53842c5426634e7929541eC2318f3dCF7e",
        "amount": amount, "payTo": PAYEE,
        "authorizationId": auth, "verified": verified, "status": "authorized",
        "settled": False, "issuedAt": "2099-01-01T00:00:00Z",
        "expiresAt": "2000-01-01T00:00:00Z" if expired else "2099-01-01T00:05:00Z",
    }
    return {"actionId": action_id, "robotId": ROBOT, "skillId": "fetch_mobile_pick",
            "params": params, "paramsHash": params_hash(params),
            "idempotencyKey": idem, "payment": pay}


@pytest.fixture
def bridge():
    return Bridge(ROBOT, PAYEE, ":memory:")


def test_skill_discoverable(bridge):
    sk = bridge.list_skills()["skills"][0]
    assert sk["skillId"] == "fetch_mobile_pick"
    assert sk["paymentRequired"] is True
    assert sk["amount"] == "100000"


def test_unpaid_challenged(bridge):
    code, resp = bridge.request_action(
        {"actionId": "a", "robotId": ROBOT, "skillId": "fetch_mobile_pick",
         "params": {}, "paramsHash": params_hash({}), "idempotencyKey": "k", "payment": {}})
    assert code == 402


def test_valid_accepted_and_executes(bridge):
    code, resp = bridge.request_action(paid("act1", "k1", "auth1"))
    assert code == 202
    assert resp["bodyStatus"] == "accepted"
    r = bridge.actions["act1"]
    assert r["status"] == "success"
    assert r["settlementEligible"] is True


def test_params_hash_mismatch(bridge):
    env = paid("act2", "k2", "auth2")
    env["paramsHash"] = "0" * 64
    code, resp = bridge.request_action(env)
    assert code == 400
    assert resp["errorCode"] == "PARAMS_HASH_MISMATCH"


def test_expired(bridge):
    code, resp = bridge.request_action(paid("act3", "k3", "auth3", expired=True))
    assert code == 402
    assert resp["errorCode"] == "PAYMENT_EXPIRED"


def test_duplicate_no_rerun(bridge):
    bridge.request_action(paid("act4", "k4", "auth4"))
    code, resp = bridge.request_action(paid("act4", "k4", "auth4"))
    assert code == 409
    assert resp["errorCode"] == "DUPLICATE"
    assert resp["cachedStatus"] == "success"


def test_idempotency_conflict(bridge):
    bridge.request_action(paid("act5", "k5", "auth5"))
    code, resp = bridge.request_action(paid("act5b", "k5", "auth5"))
    assert code == 409
    assert resp["errorCode"] == "IDEMPOTENCY_CONFLICT"


def test_auth_replay(bridge):
    bridge.request_action(paid("act6", "k6", "auth6"))
    code, resp = bridge.request_action(paid("act7", "k7", "auth6"))
    assert code == 409
    assert resp["errorCode"] == "PAYMENT_AUTHORIZATION_REPLAY"

def test_txhash_format_validation():
    import re
    TXHASH_RE = re.compile(r"^0x[0-9a-fA-F]{64}$")
    assert TXHASH_RE.match("0x" + "f" * 64)
    assert TXHASH_RE.match("0x" + "a" * 64)
    assert TXHASH_RE.match("0x" + "0" * 64)
    assert not TXHASH_RE.match("0x" + "g" * 64)
    assert not TXHASH_RE.match("abc")
    assert not TXHASH_RE.match("0x" + "a" * 63)
    assert not TXHASH_RE.match("0x" + "a" * 65)


def pi_paid(action_id, idem, auth, amount="100000000"):
    params = {}
    pay = {
        "provider": "x402", "network": "pi-testnet",
        "asset": "native",
        "amount": amount, "payTo": PAYEE,
        "authorizationId": auth, "verified": True, "status": "authorized",
        "settled": False, "issuedAt": "2099-01-01T00:00:00Z",
        "expiresAt": "2099-01-01T00:05:00Z",
    }
    return {"actionId": action_id, "robotId": ROBOT, "skillId": "fetch_mobile_pick",
            "params": params, "paramsHash": params_hash(params),
            "idempotencyKey": idem, "payment": pay}


def test_pi_payment_accepted(bridge):
    """Pi Testnet payment is gated, accepted, executes, and triggers settlement."""
    code, resp = bridge.request_action(pi_paid("pi1", "kp1", "auth-pi1"))
    assert code == 202
    assert resp["bodyStatus"] == "accepted"
    r = bridge.actions["pi1"]
    assert r["status"] == "success"
    assert r["settlementEligible"] is True



import unittest

try:
    import mujoco  # noqa: F401
    HAVE_MUJOCO = True
except Exception:
    HAVE_MUJOCO = False

from simulator import MuJoCoSimulator  # noqa: E402


@unittest.skipUnless(HAVE_MUJOCO, "mujoco not installed")
class TestRealMuJoCoCorrelated(unittest.TestCase):
    """The bridge settles ONLY when the REAL physics backend succeeds."""

    def test_real_fetch_succeeds(self):
        res = MuJoCoSimulator().fetch_mobile_pick({})
        self.assertTrue(res.success, msg=f"mujoco sim failed: {res.reason}")
        self.assertEqual(res.metrics.get("engine"), "mujoco")
        self.assertEqual(res.metrics.get("collisionCount"), 0)
        self.assertEqual(res.metrics.get("graspState"), "placed")
        self.assertGreater(res.metrics.get("objectLifted", 0), 0.05)
        self.assertTrue(res.metrics.get("placeStable"), "cube A must rest on shelf")

    def test_relay_real_fetch_settles(self):
        b = Bridge(ROBOT, PAYEE, ":memory:")
        code, _ = b.request_action(paid("muj-fetch", "k-muj-fetch", "auth-muj-fetch"))
        self.assertEqual(code, 202)
        r = b.actions["muj-fetch"]
        self.assertEqual(r["status"], "success")
        self.assertTrue(r["settlementEligible"], "real sim success must settle")
        self.assertEqual(r["metrics"].get("engine"), "mujoco")



    def test_real_fetch_fails(self):
        res = MuJoCoSimulator().fetch_mobile_pick({'cube_a': [2.0, 0.0]})
        self.assertFalse(res.success, msg="expected physical failure, got " + str(getattr(res, 'reason', '')))
        self.assertEqual(res.metrics.get("engine"), "mujoco")

    def test_relay_real_fetch_failure_no_settle(self):
        b = Bridge(ROBOT, PAYEE, ":memory:")
        code, _ = b.request_action(
            paid("fail-fetch", "kfail-fetch", "authfail-fetch", params={'cube_a': [2.0, 0.0]}))
        self.assertEqual(code, 202)
        r = b.actions["fail-fetch"]
        self.assertNotEqual(r["status"], "success", "physical failure must not read success")
        self.assertFalse(r["settlementEligible"], "failure must never settle")

if __name__ == "__main__":
    unittest.main()
