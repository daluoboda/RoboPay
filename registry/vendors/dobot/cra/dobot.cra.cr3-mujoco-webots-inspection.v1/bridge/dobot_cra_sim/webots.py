"""Execute the converted vendor CR3 in an actual Webots process."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path
from typing import Any

from .assets import PROFILE_ROOT
from .contracts import InspectionRequest
from .course import fingerprint, spec, webots_markers_vrml


WEBOTS_ROOT = PROFILE_ROOT / "simulators" / "webots"
SCENE_TEMPLATE = WEBOTS_ROOT / "scenes" / "cr3_inspection_template.wbt"
GENERATED_SCENE = WEBOTS_ROOT / "scenes" / "cr3_inspection.generated.wbt"
CONFIG_PATH = PROFILE_ROOT / "artifacts" / "generated" / "cr3_webots_config.json"
RESULT_PATH = PROFILE_ROOT / "artifacts" / "generated" / "cr3_webots_result.json"


def _webots_executable(configured: str | None = None) -> str:
    executable = configured or os.environ.get("WEBOTS_EXE") or shutil.which("webots")
    if not executable:
        raise FileNotFoundError("Webots R2025a is required; set WEBOTS_EXE when it is not on PATH")
    return executable


def render_scene() -> Path:
    """Render the canonical physical tag geometry into the launchable Webots world."""

    begin = "# BEGIN PROFILE_INSPECTION_MARKERS (rendered from dobot_cra_sim.course)"
    end = "# END PROFILE_INSPECTION_MARKERS"
    template = SCENE_TEMPLATE.read_text(encoding="utf-8")
    if template.count(begin) != 1 or template.count(end) != 1:
        raise RuntimeError("Dobot CR3 Webots inspection-scene markers are missing or ambiguous")
    prefix, remaining = template.split(begin, 1)
    _, suffix = remaining.split(end, 1)
    GENERATED_SCENE.write_text(
        f"{prefix}{begin}\n{webots_markers_vrml()}\n{end}{suffix}", encoding="utf-8"
    )
    return GENERATED_SCENE


def run_webots_episode(
    request: InspectionRequest, *, webots_executable: str | None = None, viewer: bool = False
) -> dict[str, Any]:
    """Run Webots against its own motor/sensor state and return its result JSON."""

    from generate_webots_proto import generate

    generate()
    scene = render_scene()
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    CONFIG_PATH.write_text(
        json.dumps(
            {
                "skill_id": request.skill_id,
                "max_duration_sec": request.max_duration_sec,
                "course": spec(),
                "course_hash": fingerprint(),
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    RESULT_PATH.unlink(missing_ok=True)
    environment = os.environ.copy()
    bridge_path = str(PROFILE_ROOT / "bridge")
    environment["PYTHONPATH"] = bridge_path + os.pathsep + environment.get("PYTHONPATH", "")
    environment["DOBOT_CR3_WEBOTS_CONFIG_PATH"] = str(CONFIG_PATH)
    environment["DOBOT_CR3_WEBOTS_RESULT_PATH"] = str(RESULT_PATH)
    command = [_webots_executable(webots_executable)]
    if viewer:
        environment.setdefault("DOBOT_CR3_WEBOTS_VIEWER_HOLD_SECONDS", "300")
        command.extend(["--mode=realtime", "--stdout", "--stderr"])
    else:
        command.extend(["--batch", "--mode=fast", "--no-rendering", "--stdout", "--stderr"])
    command.append(str(scene))
    completed = subprocess.run(command, env=environment, capture_output=True, text=True, check=False)
    if not RESULT_PATH.is_file():
        raise RuntimeError(
            "Webots did not produce the Dobot CR3 measured result JSON. "
            f"returncode={completed.returncode}; stderr={completed.stderr[-1000:]}"
        )
    result = json.loads(RESULT_PATH.read_text(encoding="utf-8"))
    result["webots_return_code"] = completed.returncode
    if completed.returncode != 0:
        result["success"] = False
        result["status"] = "failure"
        result.setdefault("completion_reason", "webots_process_failure")
    return result
