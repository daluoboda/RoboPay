"""Structural and privacy validation for the laok stack-arm-001 profile."""
import os

import yaml

PROFILE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PROFILE_ID = "laok.stack-arm-001.pick-and-stack.v1"


def _load(name):
    with open(os.path.join(PROFILE, name)) as f:
        return yaml.safe_load(f)


def test_profile_id_consistent():
    for fn in ("robot.profile.yaml", "skills.yaml", "execution-mapping.yaml",
               "functions.yaml", "payment-policy.yaml"):
        assert _load(fn)["profileId"] == PROFILE_ID


def test_robot_profile_fields():
    rp = _load("robot.profile.yaml")
    assert rp["vendor"] == "laok"
    assert rp["robotModel"] == "stack-arm-001"
    assert rp["submission"]["tier"] == 1
    assert rp["submission"]["scope"] == "simulated"
    assert rp["runtime"]["transport"] == "zenoh"
    assert rp["runtime"]["actionTopic"] == "robot/tunnel/action"
    assert rp["runtime"]["resultTopic"] == "robot/tunnel/result"


def test_payment_policy_network():
    pp = _load("payment-policy.yaml")
    assert pp["provider"] == "x402"
    assert pp["network"] == "eip155:84532"
    assert pp["asset"] == "0x036CbD53842c5426634e7929541eC2318f3dCF7e"
    sk = pp["policies"][0]
    assert sk["amount"] == "100000"
    assert sk["displayAmount"] == "0.10 USDC"
    assert sk["executionGate"]["rejectAlreadySettled"] is True
    assert "success" in sk["settlement"]["eligibleOnlyAfterResultStatus"]


def test_no_secrets_in_profiles():
    # Only actual secret VALUES are forbidden; env-var NAMES (e.g.
    # ROBOT_PRIVATE_KEY) are expected and safe.
    bad = ("mnemonic", "0x742d35", "0x240420")
    for fn in ("robot.profile.yaml", "skills.yaml", "execution-mapping.yaml",
               "functions.yaml", "payment-policy.yaml"):
        text = open(os.path.join(PROFILE, fn)).read().lower()
        for token in bad:
            assert token not in text, f"leaked secret token {token} in {fn}"


def test_action_envelope_example():
    import json
    env = json.load(open(os.path.join(PROFILE, "examples", "action-envelope.pick-place.json")))
    assert env["skillId"] == "sort_arm_pick_and_sort"
    assert env["payment"]["network"] == "eip155:84532"
    assert env["payment"]["amount"] == "100000"
