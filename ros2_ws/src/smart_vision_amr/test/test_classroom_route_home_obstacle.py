"""Home fixture acceptance must demand a real, collision-free bypass."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[4]
spec = importlib.util.spec_from_file_location(
    'verify_classroom_route', ROOT/'scripts/verify_classroom_route.py')
route = importlib.util.module_from_spec(spec)
spec.loader.exec_module(route)


def fake_map(wall_row=None):
    info = SimpleNamespace(width=50, height=50, resolution=.05)
    data = [0]*(info.width*info.height)
    for row in range(info.height):
        data[row*info.width] = 100
    if wall_row is not None:
        for column in range(info.width):
            data[wall_row*info.width+column] = 100
    return SimpleNamespace(info=info, data=data)


def pinched_map(wall_end_column):
    info = SimpleNamespace(width=90, height=55, resolution=.05)
    data = [0]*(info.width*info.height)
    # The old path runs horizontally through a corridor that widens to a
    # room. A box at its mouth seals the inflated passage; farther into the
    # room a full-size box and its hidden-depth strip can be bypassed.
    for row in (0, 20):
        for column in range(wall_end_column+1):
            data[row*info.width+column] = 100
    return SimpleNamespace(info=info, data=data)


def to_cell(info, x, y):
    return int(x/info.resolution), int(y/info.resolution)


def to_world(info, cell):
    return ((cell[0]+.5)*info.resolution,
            (cell[1]+.5)*info.resolution)


def test_home_fixture_selects_only_unobserved_clear_active_path():
    path = [(index*.05, .5) for index in range(41)]
    diagnostics = {}
    selected = route.select_path_obstacle(
        path, (.4, .5), (2., .5), fake_map(),
        to_cell, to_world, diagnostics=diagnostics)
    assert selected is not None
    assert selected['x'] == path[selected['index']][0]
    assert .6 <= selected['robot_distance_m'] <= 1.
    assert selected['approach_along_path_m'] < -.25
    assert selected['static_wall_distance_m'] >= .4
    assert diagnostics['reason'] == 'SELECTED'

    occupied = fake_map(wall_row=16)
    diagnostics = {}
    assert route.select_path_obstacle(
        path, (.1, .5), (2., .5), occupied,
        to_cell, to_world, diagnostics=diagnostics) is None
    assert diagnostics['reason'] == 'NO_0P4_M_STATIC_WALL_CLEARANCE'

    info = fake_map().info
    observed_cell = to_cell(info, 1.2, .5)
    observed_index = observed_cell[1]*info.width+observed_cell[0]
    diagnostics = {}
    assert route.select_path_obstacle(
        path, (.4, .5), (2., .5), fake_map(),
        to_cell, to_world, [observed_index], diagnostics) is None
    assert diagnostics['reason'] == 'PATH_ALREADY_OBSERVED'


def test_fixture_rejects_sealed_path_site_and_selects_a_bypassable_one():
    path = [(index*.05, .5) for index in range(65)]
    robot, goal = (1., .5), (3.2, .5)
    grid = pinched_map(wall_end_column=24)
    assert not route.box_bypass_route_exists(
        grid, robot, goal, (1.75, .5), (1., 0.), to_cell, to_world)
    assert route.box_bypass_route_exists(
        grid, robot, goal, (1.90, .5), (1., 0.), to_cell, to_world)
    diagnostics = {}
    selected = route.select_path_obstacle(
        path, robot, goal, grid, to_cell, to_world, diagnostics=diagnostics)
    assert selected is not None and abs(selected['x']-1.90) < 1e-9
    assert diagnostics['sealed_candidates_rejected'] >= 1
    assert diagnostics['bypass_routes_checked'] > diagnostics['sealed_candidates_rejected']
    assert diagnostics['reason'] == 'SELECTED'

    sealed = pinched_map(wall_end_column=27)
    diagnostics = {}
    assert route.select_path_obstacle(
        path, robot, goal, sealed, to_cell, to_world,
        diagnostics=diagnostics) is None
    assert diagnostics['reason'] == 'NO_BYPASSABLE_BOX_SITE'
    assert diagnostics['sealed_candidates_rejected'] == diagnostics['bypass_routes_checked'] > 0


def test_waiting_for_deletion_is_never_a_bypass():
    obstacle = (.75, .5)
    start = (.1, .5)
    tangent = (1., 0.)
    clearance = route.conservative_body_clearance(start, obstacle)
    passed, _, _ = route.physical_bypass_proven(
        start, obstacle, tangent, start, 10, clearance)
    assert not passed
    # Moving after the obstruction has been deleted would still leave this
    # predicate false while the model is present.
    behind = (.30, .9)
    passed, _, _ = route.physical_bypass_proven(
        behind, obstacle, tangent, start, 10, .05)
    assert not passed
    ahead = (1.0, .95)
    passed, along, travel = route.physical_bypass_proven(
        ahead, obstacle, tangent, start, 10, .03)
    assert passed and along >= .2 and travel >= .4
    assert route.physical_bypass_proven(
        ahead, obstacle, tangent, start, 10, .01)[0]
    assert not route.physical_bypass_proven(
        ahead, obstacle, tangent, start, 10, -.001)[0]
    assert not route.physical_bypass_proven(
        ahead, obstacle, tangent, start, 0, .03)[0]


def test_gazebo_box_clearance_is_nearest_square_surface_not_corner_radius():
    box = (0., 0.)
    # The 25 cm box side begins 12.5 cm from its center. A robot whose center
    # is 38 cm out has 1.5 cm of clearance for its 24 cm bounding disc.
    assert abs(route.conservative_body_clearance((.380, 0.), box)-.015) < 1e-9
    assert abs(route.conservative_body_clearance((.370, 0.), box)-.005) < 1e-9
    assert abs(route.conservative_body_clearance((.360, 0.), box)+.005) < 1e-9
    # At a corner, the nearest square point is the corner itself.
    expected = ((.350-.125)**2+(.350-.125)**2)**.5-.24
    assert abs(route.conservative_body_clearance((.350, .350), box)-expected) < 1e-9
    assert route.conservative_body_clearance((0., 0.), box) < 0.


def test_ros_observation_clear_and_fresh_stationary_stop_are_separate():
    samples = [{'t': 2.1, 'near_obstacle': 0},
               {'t': 2.2, 'near_obstacle': 0},
               {'t': 2.3, 'near_obstacle': 0}]
    assert route.observation_cleared(samples, 1.)
    assert not route.observation_cleared(samples[:2], 1.)
    assert not route.observation_cleared(
        [samples[0], {'t': 2.2, 'near_obstacle': 1}, samples[2]], 1.)

    commands = [{'t': t, 'linear': 0., 'angular': 0.}
                for t in (9.6, 9.7, 9.8)]
    poses = [{'t': t, 'x': 1., 'y': 2., 'yaw': .01}
             for t in (10.25, 10.5, 11.)]
    assert route.stationary_stop_proven(commands, poses, 10., 11.2)
    assert not route.stationary_stop_proven(
        commands, [*poses[:2], {**poses[2], 'yaw': .08}], 10., 11.2)
    assert not route.stationary_stop_proven(commands, poses[:2], 10., 11.2)


def test_sector_marker_ids_are_distinct_from_navigation_pass():
    assert [route.expected_marker_id(name) for name in
            ('Sector 1', 'Sector 2', 'Sector 3', 'Home')] == [1, 2, 3, 0]
    assert route.expected_marker_id('Sector1Registered') == 1
    assert route.expected_marker_id('Sector1Candidate') == 1


def test_rear_camera_rgb8_stride_decodes_expected_marker():
    import cv2
    import numpy as np
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    marker = cv2.aruco.drawMarker(dictionary, 3, 100)
    mono = np.full((240, 320), 255, dtype=np.uint8)
    mono[70:170, 110:210] = marker
    image = cv2.cvtColor(mono, cv2.COLOR_GRAY2RGB)
    frame = SimpleNamespace(height=240, width=320, step=320*3,
                            encoding='rgb8', data=image.tobytes())
    corners, ids, _ = route.detect_raw_aruco(
        frame, dictionary, parameters, cv2, np)
    assert ids is not None and ids.flatten().tolist() == [3]
    assert len(corners) == 1
