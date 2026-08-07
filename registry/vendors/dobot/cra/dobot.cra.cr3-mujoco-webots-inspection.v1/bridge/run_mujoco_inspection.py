"""Open the real Dobot CR3 MuJoCo inspection scene for visual review."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from dobot_cra_sim.contracts import INSPECTION_SKILL, ROBOT_ID, validate_action
from dobot_cra_sim.runtime import run_mujoco_episode


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true", help="keep the completed native MuJoCo scene visible")
    parser.add_argument("--hold-seconds", type=float, default=180.0, help="keep the completed scene visible")
    parser.add_argument("--json-output", type=Path, help="write the measured result JSON for CI evidence")
    args = parser.parse_args()
    request = validate_action(ROBOT_ID, INSPECTION_SKILL, INSPECTION_SKILL, {})
    result = run_mujoco_episode(
        request,
        viewer=args.viewer,
        viewer_hold_seconds=max(0.0, args.hold_seconds) if args.viewer else 0.0,
    )
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
