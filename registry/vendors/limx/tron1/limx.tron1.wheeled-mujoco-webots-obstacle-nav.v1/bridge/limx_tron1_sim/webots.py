"""Execute the same canonical course in a real Webots R2025a process."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any

from .contracts import NavigationRequest
from .course import OBSTACLES, WAYPOINTS, webots_course_vrml
from .model import PROFILE_ROOT


WEBOTS_ROOT = PROFILE_ROOT / "simulators" / "webots"
SCENE_TEMPLATE = WEBOTS_ROOT / "scenes" / "tron1_obstacle_course_template.wbt"
GENERATED_SCENE = WEBOTS_ROOT / "scenes" / "tron1_obstacle_course.generated.wbt"
CONFIG_PATH = PROFILE_ROOT / "artifacts" / "generated" / "tron1_webots_config.json"
RESULT_PATH = PROFILE_ROOT / "artifacts" / "generated" / "tron1_webots_result.json"


def _webots_executable(configured: str | None = None) -> str:
    executable = configured or os.environ.get("WEBOTS_EXE") or shutil.which("webots")
    if not executable:
        raise FileNotFoundError("Webots R2025a is required; set WEBOTS_EXE when it is not on PATH")
    return executable


def render_scene() -> Path:
    begin = "# BEGIN PROFILE_OBSTACLE_COURSE"
    end = "# END PROFILE_OBSTACLE_COURSE"
    template = SCENE_TEMPLATE.read_text(encoding="utf-8")
    if template.count(begin) != 1 or template.count(end) != 1:
        raise RuntimeError("TRON 1 Webots course markers are missing or ambiguous")
    prefix, remaining = template.split(begin, 1)
    _, suffix = remaining.split(end, 1)
    GENERATED_SCENE.write_text(f"{prefix}{begin}\n{webots_course_vrml()}\n{end}{suffix}", encoding="utf-8")
    return GENERATED_SCENE


def run_webots_episode(
    request: NavigationRequest, *, webots_executable: str | None = None, viewer: bool = False
) -> dict[str, Any]:
    from generate_webots_proto import generate

    generate()
    scene = render_scene()
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    CONFIG_PATH.write_text(
        json.dumps(
            {
                "skill_id": request.skill_id,
                "max_duration_sec": request.max_duration_sec,
                "obstacles": [obstacle.__dict__ for obstacle in OBSTACLES],
                "waypoints": WAYPOINTS,
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    RESULT_PATH.unlink(missing_ok=True)
    environment = os.environ.copy()
    bridge_path = str(PROFILE_ROOT / "bridge")
    environment["PYTHONPATH"] = bridge_path + os.pathsep + environment.get("PYTHONPATH", "")
    environment["LIMX_TRON1_WEBOTS_CONFIG_PATH"] = str(CONFIG_PATH)
    environment["LIMX_TRON1_WEBOTS_RESULT_PATH"] = str(RESULT_PATH)
    command = [_webots_executable(webots_executable)]
    if viewer:
        environment.setdefault("LIMX_TRON1_WEBOTS_VIEWER_HOLD_SECONDS", "300")
        command.extend(["--mode=realtime", "--stdout", "--stderr"])
    else:
        command.extend(["--batch", "--mode=fast", "--no-rendering", "--stdout", "--stderr"])
    command.append(str(scene))
    if viewer:
        process = subprocess.Popen(command, env=environment)
        deadline = time.monotonic() + request.max_duration_sec + 45.0
        while time.monotonic() < deadline and not RESULT_PATH.is_file():
            return_code = process.poll()
            if return_code is not None:
                raise RuntimeError(
                    "Webots viewer exited before producing its measured TRON 1 result JSON. "
                    f"returncode={return_code}"
                )
            time.sleep(0.1)
        if not RESULT_PATH.is_file():
            process.terminate()
            raise RuntimeError("Webots viewer timed out before producing its terminal result JSON")
        result = json.loads(RESULT_PATH.read_text(encoding="utf-8"))
        result["webots_return_code"] = None
        result["webots_viewer_pid"] = process.pid
        return result

    completed = subprocess.run(command, env=environment, capture_output=True, text=True, check=False)
    if not RESULT_PATH.is_file():
        raise RuntimeError(
            "Webots did not produce its measured TRON 1 result JSON. "
            f"returncode={completed.returncode}; stderr={completed.stderr[-1200:]}"
        )
    result = json.loads(RESULT_PATH.read_text(encoding="utf-8"))
    result["webots_return_code"] = completed.returncode
    if completed.returncode != 0:
        result["success"] = False
        result.setdefault("terminal_reason", "webots_process_failure")
    return result
