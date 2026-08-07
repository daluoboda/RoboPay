from __future__ import annotations

from pathlib import Path

from dobot_cra_sim.assets import vendor_urdf_sha256
from generate_webots_proto import generate


def test_webots_proto_is_generated_from_the_pinned_vendor_urdf(tmp_path: Path) -> None:
    output = generate(tmp_path / "DobotCR3.proto")
    content = output.read_text(encoding="utf-8")
    assert content.startswith("#VRML_SIM")
    assert "Dobot-Arm/DOBOT_6Axis_ROS2_V4" in content
    assert "0f67ed938c0cec4ed0808af759ddbb608e573dbe" in content
    assert "\\" not in content
    assert vendor_urdf_sha256() == "6a790fefbba0871f91819a8f8a29a5780b5952026d98396179b4eeb907859e66"
