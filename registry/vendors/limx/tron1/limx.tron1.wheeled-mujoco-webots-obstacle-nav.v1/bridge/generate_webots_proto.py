"""Generate a Webots R2025a PROTO from LimX's pinned WF_TRON1A URDF."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

from limx_tron1_sim.assets import prepare_webots_urdf
from limx_tron1_sim.model import MESHES, PROFILE_ROOT


DEFAULT_OUTPUT = PROFILE_ROOT / "simulators" / "webots" / "generated" / "LimXTRON1.proto"


def generate(output: Path = DEFAULT_OUTPUT) -> Path:
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
    relative_meshes = Path(os.path.relpath(MESHES, output.parent)).as_posix()
    rendered = rendered.replace(MESHES.resolve().as_posix(), relative_meshes)
    rendered = "\n".join(
        "# Extracted from the pinned vendor WF_TRON1A URDF"
        if line.startswith("# Extracted from:")
        else line
        for line in rendered.splitlines()
    ) + "\n"
    header, separator, body = rendered.partition("\n")
    if not header.startswith("#VRML_SIM") or not separator:
        raise RuntimeError("urdf2webots did not generate a valid R2025a TRON 1 PROTO")
    provenance = (
        "# Generated from limxdynamics/robot-description WF_TRON1A at "
        "469df8dbb56802b127ca8e2c5df23360c6c5488d (Apache-2.0).\n"
        "# Conversion is profile-generated; geometry and joint limits remain vendor supplied.\n"
    )
    rendered = header + "\n" + provenance + body
    if "C:/" in rendered or "\\" in rendered:
        raise RuntimeError("generated TRON 1 PROTO contains a host-specific mesh path")
    output.write_text(rendered, encoding="utf-8")
    return output


if __name__ == "__main__":
    print(generate())
