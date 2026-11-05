"""Static warehouse consistency checks; no ROS, Gazebo or motion requests."""
import importlib.util
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import pytest


ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
spec=importlib.util.spec_from_file_location('warehouse_generator',ROOT/'scripts/generate_warehouse_demo.py')
warehouse=importlib.util.module_from_spec(spec)
spec.loader.exec_module(warehouse)


@pytest.fixture
def copied_scene(tmp_path):
    for folder in ('worlds','maps','config'):
        source=ROOT/warehouse.PACKAGE/folder
        destination=tmp_path/warehouse.PACKAGE/folder
        destination.mkdir(parents=True)
        for path in source.glob('*warehouse*'):
            shutil.copy2(path,destination/path.name)
    for model in (ROOT/'gazebo_models').glob('smart_vision_warehouse_aruco_*'):
        shutil.copytree(model,tmp_path/'gazebo_models'/model.name)
    return tmp_path


def test_repository_world_map_goals_and_rear_marker_projection_are_consistent():
    report=warehouse.validate(ROOT)
    assert report['passed'] and report['connected_goal_cells']==11
    assert report['static_inflation_cells']==5 and report['robot_radius_m']==.22
    assert report['marker_static_line_of_sight_verified']
    assert [c['id']for c in report['camera_projection_checks']]==list(range(11))
    assert min(c['black_edge_pixels']for c in report['camera_projection_checks'])>30
    assert all(c['ideal_pixel_detection']==[c['id']]for c in report['camera_projection_checks'])
    assert min(a['minimum_static_gap_m']for a in report['actor_full_segment_clearance'])>.40
    assert not report['closed_loop_navigation_tested'] and not report['gazebo_rendered_camera_tested']


def test_map_corruption_is_rejected_not_accepted_as_a_new_world(copied_scene):
    path=copied_scene/warehouse.PACKAGE/'maps/warehouse_logistics.pgm'
    data=np.asarray(Image.open(path)).copy();data[100,100]=254-data[100,100]
    Image.fromarray(data).save(path)
    with pytest.raises(AssertionError,match='PGM/static collision raster mismatch'):
        warehouse.validate(copied_scene)


def test_laser_plane_must_be_solid_despite_visual_rack(copied_scene):
    path=copied_scene/warehouse.PACKAGE/'worlds/warehouse_logistics.world'
    tree=ET.parse(path)
    box=tree.getroot().find("world/model[@name='warehouse_rack_01']/link/collision/geometry/box/size")
    box.text='4.8 .9 .10'
    tree.write(path)
    with pytest.raises(AssertionError,match='laser plane not solid'):
        warehouse.validate(copied_scene)


def test_world_marker_pose_cannot_drift_from_the_web_goal(copied_scene):
    path=copied_scene/warehouse.PACKAGE/'worlds/warehouse_logistics.world'
    tree=ET.parse(path)
    board=tree.getroot().find("world/include[name='warehouse_marker_07']/pose")
    board.text='.798 -17.5 .15 0 0 0'  # Back-facing instead of the required yaw pi.
    tree.write(path)
    with pytest.raises(AssertionError,match='World marker pose'):
        warehouse.validate(copied_scene)


def test_dynamic_actor_initial_positions_are_free_in_static_map():
    data=np.asarray(Image.open(ROOT/warehouse.PACKAGE/'maps/warehouse_logistics.pgm'))
    _,_,_,actors=warehouse.layout()
    for actor in actors:
        assert actor['waypoints'][0]['z']==0.
        for waypoint in actor['waypoints']:
            x=int((waypoint['x']-warehouse.ORIGIN[0])/warehouse.RESOLUTION)
            y=int((waypoint['y']-warehouse.ORIGIN[1])/warehouse.RESOLUTION)
            assert data[warehouse.HEIGHT-1-y,x]==254


def test_actor_static_clearance_is_full_segment_not_just_waypoints():
    box=dict(x_min=-.5,x_max=.5,y_min=-.5,y_max=.5)
    assert warehouse.segment_rectangle_distance((-1.,0),(1.,0),box)==0.
    assert warehouse.segment_rectangle_distance((-1.,1),(1.,1),box)==pytest.approx(.5)
