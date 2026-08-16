"""Canonical three-tag inspection task rendered identically in both engines."""

from __future__ import annotations

from hashlib import sha256
import json
from typing import Any

from .kinematics import Vector3


COURSE_ID = "dobot-cr3-three-tag-tool-center-inspection-v1"
COURSE_VERSION = 1
TARGET_TOLERANCE_M = 0.045
STABLE_SAMPLES_REQUIRED = 8
MAX_DURATION_SECONDS = 20.0

# Tags are fixed profile-owned inspection markers in the CR3 base frame. The
# task planner computes IK online from measured joint state; no target joint
# trajectory is stored or replayed.
INSPECTION_TARGETS: tuple[tuple[str, Vector3, tuple[float, float, float]], ...] = (
    ("amber_tag", (-0.21434, -0.11116, 0.65239), (1.0, 0.62, 0.05)),
    ("cyan_tag", (0.08990, -0.21113, 0.56701), (0.05, 0.76, 0.96)),
    ("violet_tag", (-0.09494, -0.21166, 0.62337), (0.66, 0.26, 0.95)),
)


def spec() -> dict[str, Any]:
    return {
        "course_id": COURSE_ID,
        "course_version": COURSE_VERSION,
        "frame": "Dobot CR3 base frame; Link6 origin is the measured tool center",
        "targets": [
            {"id": target_id, "position_m": list(position), "color_rgb": list(color)}
            for target_id, position, color in INSPECTION_TARGETS
        ],
        "success_thresholds": {
            "target_tolerance_m": TARGET_TOLERANCE_M,
            "stable_samples_per_tag": STABLE_SAMPLES_REQUIRED,
            "require_all_tags": True,
            "require_finite_joint_state": True,
        },
    }


def fingerprint() -> str:
    return sha256(json.dumps(spec(), sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()


def mujoco_visuals_xml() -> str:
    """Render non-colliding tag markers. They have no success authority."""

    markers = [
        '    <geom name="inspection_workbench" type="box" pos="0 -0.18 -0.045" size="0.52 0.52 0.04" '
        'rgba="0.10 0.12 0.16 1" contype="1" conaffinity="0"/>',
    ]
    for target_id, position, color in INSPECTION_TARGETS:
        markers.append(
            f'    <geom name="{target_id}" type="sphere" pos="{position[0]} {position[1]} {position[2]}" '
            f'size="0.035" rgba="{color[0]} {color[1]} {color[2]} 1" contype="0" conaffinity="0"/>'
        )
        markers.append(
            f'    <geom name="{target_id}_halo" type="cylinder" pos="{position[0]} {position[1]} {position[2] - 0.055}" '
            f'size="0.055 0.004" rgba="{color[0]} {color[1]} {color[2]} 0.45" contype="0" conaffinity="0"/>'
        )
    return "\n".join(markers)


def webots_markers_vrml() -> str:
    """Render the same visual marker coordinates in the Webots world."""

    nodes = [
        '''Solid {
  translation 0 -0.18 -0.045
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.10 0.12 0.16 roughness 0.72 }
      geometry Box { size 1.04 1.04 0.08 }
    }
  ]
  boundingObject Box { size 1.04 1.04 0.08 }
}'''
    ]
    for target_id, position, color in INSPECTION_TARGETS:
        nodes.append(
            f'''DEF {target_id.upper()} Solid {{
  translation {position[0]} {position[1]} {position[2]}
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor {color[0]} {color[1]} {color[2]} emissiveColor {color[0] * 0.25} {color[1] * 0.25} {color[2] * 0.25} }}
      geometry Sphere {{ radius 0.035 }}
    }}
  ]
}}'''
        )
    return "\n".join(nodes)
