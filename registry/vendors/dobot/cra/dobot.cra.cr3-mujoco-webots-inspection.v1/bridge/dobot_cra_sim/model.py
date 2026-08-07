"""Load the vendor CR3 model with a small, documented simulation overlay."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .assets import GENERATED_ROOT, JOINT_NAMES, prepare_mujoco_urdf
from .course import mujoco_visuals_xml


TOOL_BODY = "Link6"


def _actuator_xml() -> str:
    actuators = "\n".join(
        f'    <position name="{joint}_position" joint="{joint}" kp="100" dampratio="1" '
        'forcelimited="true" forcerange="-100 100"/>'
        for joint in JOINT_NAMES
    )
    return f"""  <actuator>
{actuators}
  </actuator>"""


def compiled_mjcf_path() -> Path:
    """Compile the vendor URDF once, then add only profile-owned actuators."""

    import mujoco

    GENERATED_ROOT.mkdir(parents=True, exist_ok=True)
    converted = GENERATED_ROOT / "cr3_vendor_converted.mjcf"
    vendor_model = mujoco.MjModel.from_xml_path(str(prepare_mujoco_urdf()))
    mujoco.mj_saveLastXML(str(converted), vendor_model)
    text = converted.read_text(encoding="utf-8")
    if text.count("</mujoco>") != 1 or text.count("</worldbody>") != 1:
        raise RuntimeError("MuJoCo did not emit a valid converted CR3 MJCF")
    text = text.replace("</worldbody>", mujoco_visuals_xml() + "\n  </worldbody>")
    converted.write_text(text.replace("</mujoco>", _actuator_xml() + "\n</mujoco>"), encoding="utf-8")
    return converted


def load_mujoco_model() -> Any:
    """Return a real MuJoCo model built from the pinned Dobot CR3 URDF."""

    import mujoco

    return mujoco.MjModel.from_xml_path(str(compiled_mjcf_path()))


def joint_addresses(model: Any) -> tuple[list[int], list[int], list[int]]:
    """Resolve exactly the six vendor joints and profile-owned position drives."""

    import mujoco

    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES]
    actuator_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_position")
        for name in JOINT_NAMES
    ]
    if any(index < 0 for index in [*joint_ids, *actuator_ids]):
        raise RuntimeError("CR3 model did not expose every vendor joint and profile-owned actuator")
    return actuator_ids, [int(model.jnt_qposadr[index]) for index in joint_ids], joint_ids
