"""Keep in-place turns from becoming false no-progress failures on Humble."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config"
HUMBLE_PLUGIN_MANIFEST = Path("/opt/ros/humble/share/nav2_controller/plugins.xml")


@pytest.mark.parametrize("profile, radius", [
    ("curve_tuning_waffle_pi.yaml", 0.12),
    ("curve_tuning_simulation.yaml", 0.15),
    ("simulation.yaml", 0.15),
])
def test_controller_counts_in_place_heading_progress(profile, radius):
    parameters = yaml.safe_load((CONFIG / profile).read_text())["controller_server"]["ros__parameters"]
    checker = parameters["progress_checker"]
    assert parameters["progress_checker_plugin"] == "progress_checker"
    assert checker["plugin"] == "nav2_controller::PoseProgressChecker"
    assert checker["required_movement_radius"] == radius
    assert checker["required_movement_angle"] == 0.35
    assert checker["movement_time_allowance"] == 20.0
    # In the recorded Sector 3 stall the robot turned ~0.9 rad but translated
    # ~0.10 m; the former is progress for a differential-drive robot.
    assert 0.10 < checker["required_movement_radius"]
    assert checker["required_movement_angle"] < 0.9


def test_required_humble_progress_plugin_is_installed():
    assert HUMBLE_PLUGIN_MANIFEST.is_file()
    registered = {item.attrib["type"] for item in ET.parse(HUMBLE_PLUGIN_MANIFEST).iter("class")}
    assert "nav2_controller::PoseProgressChecker" in registered
