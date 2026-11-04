"""Keep in-place turns from becoming false no-progress failures on Humble."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from smart_vision_amr.map_utils import inflate_cells_footprint


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


@pytest.mark.parametrize("profile", [
    "curve_tuning_waffle_pi.yaml", "classroom_demo_operator_20261104.yaml",
])
def test_classroom_dwb_enters_final_yaw_turn_at_goal_checker_distance(profile):
    nav = yaml.safe_load((CONFIG / profile).read_text())
    controller = nav["controller_server"]["ros__parameters"]
    assert controller["FollowPath"]["xy_goal_tolerance"] == controller[
        "general_goal_checker"]["xy_goal_tolerance"] == 0.25


def test_physical_goal_turn_and_clearance_profiles_agree():
    nav = yaml.safe_load((CONFIG / "curve_tuning_waffle_pi.yaml").read_text())
    controller = nav["controller_server"]["ros__parameters"]
    follower = controller["FollowPath"]
    planner = yaml.safe_load((CONFIG / "classroom_hardware_nodes.yaml").read_text())
    astar = planner["/astar_planner"]["ros__parameters"]
    curve = planner["/path_follower"]["ros__parameters"]

    # A robot within the 25 cm Home goal band must enter DWB's final yaw turn.
    assert follower["xy_goal_tolerance"] == controller["general_goal_checker"]["xy_goal_tolerance"] == 0.25
    assert follower["trans_stopped_velocity"] == follower["min_speed_xy"] == 0.02
    assert follower["min_speed_theta"] == 0.12
    assert follower["min_speed_theta"] < follower["max_vel_theta"]
    assert follower["max_vel_x"] == follower["max_speed_xy"] == curve["curve_max_linear_velocity"] == 0.24

    # The farthest rotating arc of the physical robot was measured at about
    # 17 cm. Every physical planner and costmap must use the same bound.
    radius = astar["robot_radius"]
    assert radius == 0.17
    assert nav["local_costmap"]["local_costmap"]["ros__parameters"]["robot_radius"] == radius
    assert nav["global_costmap"]["global_costmap"]["ros__parameters"]["robot_radius"] == radius
    for name in ("local_costmap", "global_costmap"):
        costmap = nav[name][name]["ros__parameters"]
        inflation = costmap["inflation_layer"]["inflation_radius"]
        assert radius < inflation <= radius + costmap["resolution"]
    # A* treats the source and destination as complete 5 cm squares, so
    # 3 cells leave only 15 cm of possible center clearance and may overlap.
    cells = astar["obstacle_inflation_cells"]
    assert cells == 4
    assert (cells - 1) * 0.05 < radius < cells * 0.05
    assert curve["recovery_robot_radius"] == radius
    assert curve["recovery_rear_margin"] == 0.01
    assert astar["live_route_confirmation_scans"] == 3


def test_operator_sim_nav_profile_matches_physical_motion_limits():
    physical = yaml.safe_load((CONFIG / "curve_tuning_waffle_pi.yaml").read_text())
    demo = yaml.safe_load((CONFIG / "classroom_demo_operator_20261104.yaml").read_text())
    for name in ("controller_server", "local_costmap", "global_costmap"):
        assert demo[name] == physical[name]


def test_live_cell_mask_uses_exact_seventeen_centimeter_footprint():
    hit = (10, 10)
    mask = inflate_cells_footprint({hit}, 25, 25, .17 / .05)
    assert (14, 10) in mask  # Full-cell gap is 15 cm: overlap.
    assert (15, 10) not in mask  # Gap is 20 cm: no geometric overlap.
    assert (13, 13) in mask  # Diagonal gap is 14.1 cm.
    assert (14, 13) not in mask  # Diagonal gap is 18.0 cm.
