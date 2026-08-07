from __future__ import annotations

import hashlib
import json
from pathlib import Path

import yaml

from dobot_cra_sim.contracts import INSPECTION_SKILL, PROFILE_ID, ROBOT_ID, STOP_SKILL


ROOT = Path(__file__).resolve().parents[1]


def _yaml(name: str) -> dict:
    return yaml.safe_load((ROOT / name).read_text(encoding="utf-8"))


def test_registry_and_payment_policy_do_not_drift() -> None:
    profile = _yaml("robot.profile.yaml")
    skills = _yaml("skills.yaml")
    policy = _yaml("payment-policy.yaml")
    mapping = _yaml("execution-mapping.yaml")
    catalog = json.loads((ROOT / "skill-catalog.json").read_text(encoding="utf-8"))
    skill_ids = {entry["skillId"] for entry in skills["skills"]}
    assert profile["profileId"] == PROFILE_ID
    assert profile["robotId"] == ROBOT_ID
    assert {INSPECTION_SKILL, STOP_SKILL} == skill_ids
    assert {entry["skill_id"] for entry in catalog} == skill_ids
    assert {entry["skillId"] for entry in policy["policies"]} == skill_ids
    assert set(mapping["skills"]) == skill_ids
    assert {entry["priceUSDC"] for entry in policy["policies"]} == {"0.001"}
    assert {entry["price_usdc"] for entry in catalog} == {"0.001"}
    source = ROOT / profile["modelIdentity"]["urdf"]
    assert hashlib.sha256(source.read_bytes()).hexdigest() == profile["modelIdentity"]["urdfSha256"]


def test_profile_docs_and_public_action_examples_are_present() -> None:
    """Keep the reviewer-facing profile material coupled to the registry."""
    docs = ROOT / "docs"
    assert (docs / "README.md").is_file()
    assert (docs / "validation-report.md").is_file()
    assert (docs / "evidence" / "evidence-manifest.yaml").is_file()

    inspect_example = json.loads(
        (ROOT / "examples" / "action-envelope.inspect_three_tags.json").read_text(
            encoding="utf-8"
        )
    )
    stop_example = json.loads(
        (ROOT / "examples" / "action-envelope.stop.json").read_text(encoding="utf-8")
    )
    for example, skill_id in ((inspect_example, INSPECTION_SKILL), (stop_example, STOP_SKILL)):
        assert example["action"] == skill_id
        assert example["robot_id"] == ROBOT_ID
        assert example["action_id"] == example["idempotency_key"]
        assert example["params"] == {}
