"""Open the illuminated, real Webots Dobot CR3 inspection scene."""

from __future__ import annotations

import json

from dobot_cra_sim.contracts import INSPECTION_SKILL, ROBOT_ID, validate_action
from dobot_cra_sim.webots import run_webots_episode


def main() -> int:
    request = validate_action(ROBOT_ID, INSPECTION_SKILL, INSPECTION_SKILL, {})
    result = run_webots_episode(request, viewer=True)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
