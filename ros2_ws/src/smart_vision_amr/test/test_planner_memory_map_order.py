"""Latched map and memory overlays may arrive in either ROS callback order."""

from copy import deepcopy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32MultiArray

from test_planner_failure_memory import FailureHarness, refresh
from test_planner_recovery import PlannerHarness, make_map, make_goal


def open_map():
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.width = msg.info.height = 30
    msg.info.resolution = 0.1
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * 900
    return msg


def test_latched_memory_before_map_is_used_by_first_goal(monkeypatch):
    clock = {"steady": 100.0, "ros": 100.0}
    monkeypatch.setattr("smart_vision_amr.astar_planner.time.monotonic",
                        lambda: clock["steady"])
    node = FailureHarness(clock)
    node.on_memory(Int32MultiArray(data=[15 * 30 + 15]))
    assert node.remembered == set()

    node.on_map(open_map())
    assert node.remembered == {(15, 15)}
    assert node.pending_memory_indices is None
    refresh(node)
    goal = make_goal(stamp=100)
    goal.pose.position.x, goal.pose.position.y = 2.55, 1.55
    node.on_goal(goal)

    assert node.planned_cells
    assert (15, 15) not in node.planned_cells
    assert node.path_pub.publish.call_args.args[0].poses


def test_latest_empty_memory_before_first_map_supersedes_nonempty():
    node = PlannerHarness()
    node.on_memory(Int32MultiArray(data=[7]))
    node.on_memory(Int32MultiArray(data=[]))
    node.on_map(make_map())
    assert node.remembered == set()
    assert node.pending_memory_indices is None


def test_new_map_clears_old_cells_but_identical_reload_keeps_them():
    node = PlannerHarness()
    original = make_map()
    node.on_map(original)
    node.on_memory(Int32MultiArray(data=[7]))
    assert node.remembered == {(2, 1)}

    same_content = deepcopy(original)
    same_content.info.map_load_time.sec = 42
    node.on_map(same_content)
    assert node.remembered == {(2, 1)}

    replacement = deepcopy(same_content)
    replacement.info.origin.position.x = 1.0
    node.on_map(replacement)
    assert node.remembered == set()
    node.on_memory(Int32MultiArray(data=[7]))
    assert node.remembered == {(2, 1)}

    changed_content = deepcopy(replacement)
    changed_content.data[0] = 100
    node.on_map(changed_content)
    assert node.remembered == set()


def test_memory_indices_are_decoded_only_against_received_map_dimensions():
    node = PlannerHarness()
    node.on_memory(Int32MultiArray(data=[-1, 7, 15, 999]))
    node.on_map(make_map())
    assert node.remembered == {(2, 1)}
