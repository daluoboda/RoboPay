"""Generate a Webots R2025a PROTO from the pinned Dobot CR3 vendor URDF."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from dobot_cra_sim.assets import PROFILE_ROOT, prepare_webots_urdf


DEFAULT_OUTPUT = PROFILE_ROOT / "simulators" / "webots" / "generated" / "DobotCR3.proto"


def generate(output: Path = DEFAULT_OUTPUT) -> Path:
    """Convert the vendor CR3 URDF without modifying its geometry or limits."""

    source = prepare_webots_urdf()
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            sys.executable,
            "-m",
            "urdf2webots.importer",
            "--input",
            str(source),
            "--output",
            str(output),
            "--box-collision",
            "--link-to-def",
            "--joint-to-def",
            "--target",
            "R2025a",
        ],
        check=True,
    )
    rendered = output.read_text(encoding="utf-8").replace("\\", "/")
    header, separator, body = rendered.partition("\n")
    if not header.startswith("#VRML_SIM") or not separator:
        raise RuntimeError("urdf2webots did not generate a valid R2025a Dobot CR3 PROTO")
    provenance = (
        "# Generated from the vendor-published Dobot CR3 URDF.\n"
        "# Source: Dobot-Arm/DOBOT_6Axis_ROS2_V4 at 0f67ed938c0cec4ed0808af759ddbb608e573dbe (MIT).\n"
        "# This converted Webots PROTO is profile-generated, not vendor-supplied.\n"
    )
    output.write_text(header + "\n" + provenance + body, encoding="utf-8")
    if "\\" in output.read_text(encoding="utf-8"):
        raise RuntimeError("Generated Dobot CR3 PROTO contains a Windows-only mesh path")
    return output


if __name__ == "__main__":
    print(generate())
