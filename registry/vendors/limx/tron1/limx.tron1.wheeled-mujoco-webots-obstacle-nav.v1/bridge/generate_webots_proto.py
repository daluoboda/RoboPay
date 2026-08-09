"""Generate a Webots R2025a PROTO from LimX's pinned WF_TRON1A URDF."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

from limx_tron1_sim.assets import prepare_webots_urdf
from limx_tron1_sim.model import MESHES, PROFILE_ROOT


DEFAULT_OUTPUT = PROFILE_ROOT / "simulators" / "webots" / "generated" / "LimXTRON1.proto"


def _replace_once(rendered: str, source: str, replacement: str, label: str) -> str:
    if rendered.count(source) != 1:
        raise RuntimeError(f"generated TRON 1 PROTO has unexpected {label} collision structure")
    return rendered.replace(source, replacement)


def _apply_vendor_collision_frames(rendered: str) -> str:
    """Restore URDF collision origins lost by urdf2webots --box-collision.

    Webots cylinders are Y-aligned.  The transforms below are the pinned
    WF_TRON1A URDF collision origins expressed in that convention.  The wheel
    contact radius is expanded from the URDF's 0.127 m proxy to the measured
    0.130 m outer radius of the pinned vendor STL, so the rendered tire and
    contact surface coincide.  No visual geometry is changed.
    """
    replacements = (
        (
            "base",
            "    boundingObject Box {\n       size 0.270000 0.260000 0.190000\n    }",
            "    boundingObject Transform {\n"
            "      translation 0.030000 0 -0.072000\n"
            "      children [ Box { size 0.270000 0.260000 0.190000 } ]\n"
            "    }",
        ),
        (
            "left abad",
            "          name \"abad_L_Link\"\n          boundingObject Cylinder {\n            radius 0.05\n            height 0.05\n          }",
            "          name \"abad_L_Link\"\n"
            "          boundingObject Transform {\n"
            "            translation 0.030000 0 0\n"
            "            rotation 0.707388 0.706825 0.000281 3.141029\n"
            "            children [ Cylinder { radius 0.050 height 0.050 } ]\n"
            "          }",
        ),
        (
            "right abad",
            "          name \"abad_R_Link\"\n          boundingObject Cylinder {\n            radius 0.05\n            height 0.05\n          }",
            "          name \"abad_R_Link\"\n"
            "          boundingObject Transform {\n"
            "            translation 0.030000 0 0\n"
            "            rotation 0.707388 0.706825 0.000281 3.141029\n"
            "            children [ Cylinder { radius 0.050 height 0.050 } ]\n"
            "          }",
        ),
        (
            "left hip",
            "                name \"hip_L_Link\"\n                boundingObject Cylinder {\n                  radius 0.035\n                  height 0.15\n                }",
            "                name \"hip_L_Link\"\n"
            "                boundingObject Transform {\n"
            "                  translation -0.100000 -0.030000 -0.140000\n"
            "                  rotation 0.862807 0 -0.505534 1.570796\n"
            "                  children [ Cylinder { radius 0.035 height 0.15 } ]\n"
            "                }",
        ),
        (
            "right hip",
            "                name \"hip_R_Link\"\n                boundingObject Cylinder {\n                  radius 0.035\n                  height 0.15\n                }",
            "                name \"hip_R_Link\"\n"
            "                boundingObject Transform {\n"
            "                  translation -0.100000 0.030000 -0.140000\n"
            "                  rotation 0.862807 0 -0.505534 1.570796\n"
            "                  children [ Cylinder { radius 0.035 height 0.15 } ]\n"
            "                }",
        ),
        (
            "left knee",
            "                      name \"knee_L_Link\"\n                      boundingObject Cylinder {\n                        radius 0.015\n                        height 0.26\n                      }",
            "                      name \"knee_L_Link\"\n"
            "                      boundingObject Transform {\n"
            "                        translation 0.078000 0 -0.120000\n"
            "                        rotation 0.852525 0 0.522687 1.570796\n"
            "                        children [ Cylinder { radius 0.015 height 0.26 } ]\n"
            "                      }",
        ),
        (
            "right knee",
            "                      name \"knee_R_Link\"\n                      boundingObject Cylinder {\n                        radius 0.015\n                        height 0.26\n                      }",
            "                      name \"knee_R_Link\"\n"
            "                      boundingObject Transform {\n"
            "                        translation 0.078000 0 -0.120000\n"
            "                        rotation 0.852525 0 0.522687 1.570796\n"
            "                        children [ Cylinder { radius 0.015 height 0.26 } ]\n"
            "                      }",
        ),
        (
            "left wheel",
            "                            name \"wheel_L_Link\"\n                            boundingObject Cylinder {\n                              radius 0.127\n                              height 0.05\n                            }",
            "                            name \"wheel_L_Link\"\n"
            "                            contactMaterial \"tron1 wheel\"\n"
            "                            boundingObject Transform {\n"
            "                              translation 0 0.007070 0\n"
            "                              children [ Cylinder { radius 0.130 height 0.050 } ]\n"
            "                            }",
        ),
        (
            "right wheel",
            "                            name \"wheel_R_Link\"\n                            boundingObject Cylinder {\n                              radius 0.127\n                              height 0.05\n                            }",
            "                            name \"wheel_R_Link\"\n"
            "                            contactMaterial \"tron1 wheel\"\n"
            "                            boundingObject Transform {\n"
            "                              translation 0 -0.007070 0\n"
            "                              children [ Cylinder { radius 0.130 height 0.050 } ]\n"
            "                            }",
        ),
    )
    for label, source, replacement in replacements:
        rendered = _replace_once(rendered, source, replacement, label)
    return rendered


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
    rendered = _apply_vendor_collision_frames(rendered)
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
