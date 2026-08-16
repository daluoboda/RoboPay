"""Run the same bounded CR3 inspection request in real MuJoCo and Webots."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from dobot_cra_sim.contracts import INSPECTION_SKILL, ROBOT_ID, validate_action
from dobot_cra_sim.course import COURSE_ID, TARGET_TOLERANCE_M, fingerprint, spec
from dobot_cra_sim.runtime import run_mujoco_episode
from dobot_cra_sim.webots import run_webots_episode


def _checks(result: dict[str, Any], engine: str) -> dict[str, bool]:
    expected_ids = {target["id"] for target in spec()["targets"]}
    observed = result.get("observed_tags", [])
    observed_ids = {entry.get("target_id") for entry in observed if isinstance(entry, dict)}
    errors = [float(entry.get("measured_error_m", float("inf"))) for entry in observed if isinstance(entry, dict)]
    return {
        "engine_reported_success": result.get("success") is True,
        "correct_simulator": result.get("simulator_engine") == engine,
        "canonical_course_id": result.get("course_id") == COURSE_ID,
        "canonical_course_hash": result.get("course_hash") == fingerprint(),
        "all_three_tags_observed": observed_ids == expected_ids and len(observed) == len(expected_ids),
        "measured_target_error_bounded": bool(errors) and all(error <= TARGET_TOLERANCE_M for error in errors),
        "no_unexpected_contact": result.get("unexpected_contact_observed") is False,
        "safe_tool_height": float(result.get("minimum_tool_height_m", 0.0)) >= 0.15,
        "finite_state": result.get("finite_state") is True,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true", help="show the real MuJoCo and Webots desktop scenes")
    args = parser.parse_args()
    request = validate_action(ROBOT_ID, INSPECTION_SKILL, INSPECTION_SKILL, {})
    mujoco_result = run_mujoco_episode(request, viewer=args.viewer, viewer_hold_seconds=20.0 if args.viewer else 0.0)
    webots_result = run_webots_episode(request, viewer=args.viewer)
    mujoco_checks = _checks(mujoco_result, "MuJoCo")
    webots_checks = _checks(webots_result, "Webots")
    success = all(mujoco_checks.values()) and all(webots_checks.values())
    report = {
        "task": "dobot_cr3_three_tag_tool_center_inspection_sim2sim",
        "status": "success" if success else "failure",
        "success": success,
        "shared_policy": {
            "policy_id": COURSE_ID,
            "action": INSPECTION_SKILL,
            "state_authority": "vendor joint state and Link6/tool-center pose measured independently by each engine",
            "planner": "nearest-unobserved target selection plus iterative damped-least-squares IK; no prerecorded joint trajectory",
        },
        "course_contract": {**spec(), "course_hash": fingerprint()},
        "acceptance_checks": {"mujoco": mujoco_checks, "webots": webots_checks},
        "mujoco": mujoco_result,
        "webots": webots_result,
        "note": "MuJoCo compiles the pinned Dobot CR3 vendor URDF. Webots converts the same pinned URDF at runtime. Both execute the same profile-owned Cartesian tag task and each reports its own measured outcome.",
    }
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    artifact = Path(__file__).resolve().parents[1] / "artifacts" / "sim2sim_result.json"
    artifact.parent.mkdir(parents=True, exist_ok=True)
    artifact.write_text(rendered + "\n", encoding="utf-8")
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
