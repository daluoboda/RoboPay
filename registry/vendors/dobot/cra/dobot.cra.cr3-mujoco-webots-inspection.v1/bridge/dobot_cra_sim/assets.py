"""Vendor-model provenance and simulator input preparation."""

from __future__ import annotations

from hashlib import sha256
from pathlib import Path


PROFILE_ROOT = Path(__file__).resolve().parents[2]
VENDOR_ROOT = PROFILE_ROOT / "vendor" / "DOBOT_6Axis_ROS2_V4"
VENDOR_URDF = VENDOR_ROOT / "cr3_robot.urdf"
VENDOR_MESH_ROOT = VENDOR_ROOT / "meshes" / "cr3"
GENERATED_ROOT = PROFILE_ROOT / "artifacts" / "generated"
SOURCE_REPOSITORY = "https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4"
SOURCE_COMMIT = "0f67ed938c0cec4ed0808af759ddbb608e573dbe"
JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))

def vendor_urdf_sha256() -> str:
    """Return the pinned vendor URDF digest used by both simulators."""

    return sha256(VENDOR_URDF.read_bytes()).hexdigest()


def _render_urdf(*, mujoco_extensions: str, mesh_prefix: str) -> str:
    """Keep vendor geometry intact while resolving its ROS package URLs locally."""

    if not VENDOR_URDF.is_file() or not VENDOR_MESH_ROOT.is_dir():
        raise FileNotFoundError("Pinned Dobot CR3 vendor URDF or meshes are missing")
    text = VENDOR_URDF.read_text(encoding="utf-8")
    text = text.replace(
        "package://dobot_rviz/meshes/cr3/",
        mesh_prefix,
    )
    marker = '<robot\n  name="cr3_robot">'
    if marker not in text:
        raise RuntimeError("Pinned Dobot CR3 URDF has an unexpected robot root")
    return text.replace(marker, f"{marker}\n{mujoco_extensions}", 1)


def prepare_mujoco_urdf() -> Path:
    """Generate the vendor URDF plus a minimal MuJoCo compiler overlay.

    The compiler overlay only declares simulator units and conservative joint
    dynamics.  ``model.compiled_mjcf_path`` appends the bounded position
    actuators after MuJoCo has converted this URDF to canonical MJCF; that is
    the format in which the actuators are actually consumed by the engine.
    """

    extensions = f'''  <!-- Profile-owned dynamics/control overlay; vendor links, joints, limits, inertia and meshes remain unchanged. -->
  <mujoco>
    <compiler angle="radian" balanceinertia="true" autolimits="true"
              meshdir="../../vendor/DOBOT_6Axis_ROS2_V4/meshes/cr3"/>
    <option timestep="0.002" gravity="0 0 -9.81"/>
    <default>
      <joint armature="0.01" damping="0.25" frictionloss="0.01"/>
    </default>
  </mujoco>'''
    generated = _render_urdf(mujoco_extensions=extensions, mesh_prefix="")
    GENERATED_ROOT.mkdir(parents=True, exist_ok=True)
    output = GENERATED_ROOT / "cr3_robot_mujoco.urdf"
    output.write_text(generated, encoding="utf-8")
    return output


def prepare_webots_urdf() -> Path:
    """Generate the geometry-preserving URDF source consumed by urdf2webots."""

    generated = _render_urdf(
        mujoco_extensions="",
        mesh_prefix="../../vendor/DOBOT_6Axis_ROS2_V4/meshes/cr3/",
    )
    GENERATED_ROOT.mkdir(parents=True, exist_ok=True)
    output = GENERATED_ROOT / "cr3_robot_webots.urdf"
    output.write_text(generated, encoding="utf-8")
    return output
