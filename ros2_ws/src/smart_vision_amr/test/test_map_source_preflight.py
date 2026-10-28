"""An old SLAM map must never share /map with saved-map navigation."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[4]/'scripts/check_map_publishers.py'
spec = importlib.util.spec_from_file_location('check_map_publishers', SCRIPT)
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


class Graph:
    def __init__(self, publishers=(), nodes=()):
        self.publishers = publishers
        self.nodes = nodes
        self.destroyed = False

    def get_publishers_info_by_topic(self, name):
        assert name == '/map'
        return self.publishers

    def get_node_names_and_namespaces(self):
        return self.nodes

    def destroy_node(self):
        self.destroyed = True


def publisher(name):
    return SimpleNamespace(node_namespace='/', node_name=name)


def test_preflight_finds_remote_cartographer_even_when_map_server_is_also_running():
    graph = Graph([publisher('cartographer_occupancy_grid_node'), publisher('map_server')],
                  [('cartographer_node', '/')])
    assert guard.conflicting_sources(graph) == [
        '/cartographer_node', '/cartographer_occupancy_grid_node', '/map_server']


def test_preflight_finds_slam_node_before_first_map_and_ignores_other_robot_nodes():
    assert guard.conflicting_sources(Graph(nodes=[('slam_toolbox', '/'), ('turtlebot3_node', '/')])) == [
        '/slam_toolbox']
    assert guard.conflicting_sources(Graph(nodes=[('turtlebot3_node', '/'), ('lidar_node', '/')])) == []


def test_preflight_refuses_conflicting_source_with_actionable_error(capsys):
    graph = Graph([publisher('cartographer_occupancy_grid_node')])
    with patch.object(guard.rclpy, 'init'), patch.object(guard.rclpy, 'create_node', return_value=graph), \
            patch.object(guard.rclpy, 'ok', return_value=True), patch.object(guard.rclpy, 'shutdown'):
        result = guard.main(['--domain-id', '16', '--discovery-seconds', '0'])
    assert result == 1
    assert graph.destroyed
    output = capsys.readouterr().err
    assert 'cartographer_occupancy_grid_node' in output
    assert 'SLAM/Cartographer' in output


def test_preflight_allows_clean_domain(capsys):
    graph = Graph(nodes=[('turtlebot3_node', '/')])
    with patch.object(guard.rclpy, 'init'), patch.object(guard.rclpy, 'create_node', return_value=graph), \
            patch.object(guard.rclpy, 'ok', return_value=True), patch.object(guard.rclpy, 'shutdown'):
        result = guard.main(['--domain-id', '16', '--discovery-seconds', '0'])
    assert result == 0
    assert not capsys.readouterr().err
