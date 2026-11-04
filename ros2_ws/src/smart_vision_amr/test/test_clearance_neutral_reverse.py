"""A clearance preference cannot hide a fully swept-clear inflation exit.

These are pure selector probes, not actual breadcrumb or driving replay. The
recorded map/hits/pose are frozen; the ideal straight history is synthesized.
"""
from hashlib import sha256
import json
from math import cos, hypot, pi, sin
from pathlib import Path
from unittest.mock import Mock

import pytest

from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.grid import Grid
from smart_vision_amr.map_utils import inflate_cells, inflate_cells_footprint
from smart_vision_amr.reverse_recovery import (
    Breadcrumb, GridGeometry, _raw_clearance, select_reverse_egress,
)
from test_reverse_recovery import grids, select, static_parallel_wall_scene, trail


ROOT = Path(__file__).resolve().parents[4]
FIXTURE = ROOT/'benchmarks/driving_parameter_tuning_20261104/recovery_audit/fixture.json'


@pytest.fixture(scope='module')
def recorded_gate():
    data = json.loads(FIXTURE.read_text())
    image = ROOT/data['map']['image']
    assert sha256(image.read_bytes()).hexdigest() == data['map']['image_sha256']
    with image.open('rb') as stream:
        assert stream.readline() == b'P5\n'
        width, height = map(int, stream.readline().split())
        assert stream.readline() == b'255\n'
        pixels = stream.read()
    occupied = {(x, y) for y in range(height) for x in range(width)
                if pixels[(height-y-1)*width+x] != 254}
    observed, memory, reserved_observed, reserved_memory = (
        {tuple(c) for c in data[key]} for key in ('observed_cells', 'remembered_cells',
            'reserved_observed_cells', 'reserved_remembered_cells'))
    raw_cells = occupied | observed | memory
    raw = Grid(tuple(tuple((x, y) in raw_cells for x in range(width)) for y in range(height)))
    static_cells = inflate_cells(occupied, width, height, 5)
    static = Grid(tuple(tuple((x, y) in static_cells for x in range(width)) for y in range(height)))
    full = (inflate_cells_footprint(reserved_observed, width, height, .22/.05)
            | inflate_cells(reserved_memory, width, height, 5))
    measured = (inflate_cells_footprint(observed, width, height, .22/.05)
                | inflate_cells(memory, width, height, 5))
    inflated = Grid(tuple(tuple((x, y) in full|static_cells for x in range(width))
                          for y in range(height)))
    pose = tuple(data['pose'])
    backward = -cos(pose[2]), -sin(pose[2])
    # Necessary-gate counterfactual only: actual breadcrumbs were not logged.
    history = [Breadcrumb(100.-distance/.1, pose[0]+distance*backward[0],
                         pose[1]+distance*backward[1], pose[2])
               for distance in (.30,.275,.25,.225,.20,.175,.15,.125,.10,.075,.05,.025,0.)]
    geometry = GridGeometry(data['map']['resolution'], *data['map']['origin'])
    def reachable(cell):
        return bool(AStarPlanner().plan(static, cell, tuple(data['goal_cell']), measured))
    kwargs = dict(now=100., robot_radius=.22, clearance_margin=.01,
                  min_distance=.10, max_distance=.30, current_pose=pose,
                  blocking_cells=set(), allow_parallel_static_egress=True)
    return history, raw, inflated, geometry, reachable, kwargs, data


def test_recorded_clear_sweep_minor_distance_dip_is_not_a_collision(recorded_gate):
    history, raw, inflated, geometry, reachable, kwargs, data = recorded_gate
    oracle = Mock(wraps=reachable)
    assert select_reverse_egress(history, raw, inflated, geometry, oracle, **kwargs) is None
    oracle.assert_not_called()  # Original rejection occurs at the first tiny dip.
    candidate = select_reverse_egress(history, raw, inflated, geometry, oracle,
        **kwargs, clearance_neutral_egress_enabled=True)
    assert candidate is not None
    assert .10 <= candidate.distance_m <= .30
    assert not inflated.is_static_obstacle(candidate.target_cell)
    assert oracle.called and reachable(candidate.target_cell)
    assert candidate.start_clearance_m == pytest.approx(.2470643497505647, abs=1e-12)
    occupied = tuple((x, y) for y,row in enumerate(raw.rows) for x,v in enumerate(row) if v)
    pose = data['pose']
    samples = [_raw_clearance(raw, geometry, occupied,
        pose[0]-distance/1000*cos(pose[2]), pose[1]-distance/1000*sin(pose[2]))
        for distance in range(301)]
    assert min(samples) == pytest.approx(.24558805807891357, abs=1e-12)
    assert samples[0]-min(samples) > .0014
    # 1mm grid + half-step Lipschitz reserve certifies all intermediate points.
    assert min(samples)-.0005 > .22+.01+.01/2


@pytest.mark.parametrize('oracle', [lambda _:False, lambda _:(_ for _ in ()).throw(ValueError('unavailable'))])
def test_neutral_exit_still_requires_retained_goal_oracle(recorded_gate, oracle):
    history, raw, inflated, geometry, _, kwargs, _ = recorded_gate
    assert select_reverse_egress(history, raw, inflated, geometry, oracle,
        **kwargs, clearance_neutral_egress_enabled=True) is None


def test_neutral_is_not_available_at_an_uninflated_start():
    raw, inflated, history = static_parallel_wall_scene()
    cleared = Grid(tuple(tuple(False for _ in row) for row in inflated.rows))
    assert select(history, raw=raw, inflated=cleared, geometry=GridGeometry(.05,0.,0.),
        robot_radius=.24, clearance_margin=.03, clearance_neutral_egress_enabled=True) is None


def test_neutral_does_not_waive_focused_blocker_gain(recorded_gate):
    history, raw, inflated, geometry, reachable, kwargs, data = recorded_gate
    kwargs = dict(kwargs, blocking_cells={tuple(data['observed_cells'][0])})
    # With a nominated physical blocker this remains the original focused
    # monotonic-and-gain test, even if the caller mistakenly enables the flag.
    original = select_reverse_egress(history, raw, inflated, geometry, reachable, **kwargs)
    modified = select_reverse_egress(history, raw, inflated, geometry, reachable,
        **kwargs, clearance_neutral_egress_enabled=True)
    assert modified == original


@pytest.mark.parametrize('kind', ['rear_hit','body_overlap','unsafe_axis','stale','curved','jump','pose_drift','heading_drift','no_full_exit'])
def test_neutral_preserves_all_physical_history_and_endpoint_checks(recorded_gate, kind):
    history, raw, inflated, geometry, reachable, kwargs, data = recorded_gate
    history, kwargs = list(history), dict(kwargs)
    if kind in ('rear_hit','body_overlap','unsafe_axis'):
        rows = [list(row) for row in raw.rows]
        if kind == 'body_overlap':
            cell = tuple(data['start_cell'])
        elif kind == 'rear_hit':
            pose=data['pose']; distance=.10
            cell=(int((pose[0]-distance*cos(pose[2])+1.65)/.05),
                  int((pose[1]-distance*sin(pose[2])+5.25)/.05))
        else:
            # A real raw square intersects the requested straight reverse ray.
            pose=data['pose']; distance=.16
            cell=(int((pose[0]-distance*cos(pose[2])+1.65)/.05),
                  int((pose[1]-distance*sin(pose[2])+5.25)/.05))
        rows[cell[1]][cell[0]]=True
        raw=Grid(tuple(tuple(row) for row in rows))
    elif kind == 'stale':
        kwargs['now']=102.
    elif kind == 'curved':
        p=history[-2]; history[-2]=Breadcrumb(p.stamp,p.x+.04,p.y,p.yaw)
    elif kind == 'jump':
        history=[history[0],history[-1]]
    elif kind == 'pose_drift':
        pose=data['pose'];kwargs['current_pose']=(pose[0]+.06,pose[1],pose[2])
    elif kind == 'heading_drift':
        pose=data['pose'];kwargs['current_pose']=(pose[0],pose[1],pose[2]+.16)
    elif kind == 'no_full_exit':
        inflated=Grid(tuple(tuple(True for _ in row) for row in inflated.rows))
    assert select_reverse_egress(history, raw, inflated, geometry, reachable,
        **kwargs, clearance_neutral_egress_enabled=True) is None


@pytest.mark.parametrize('value', [1,0,'true',None,float('nan')])
def test_neutral_flag_is_strict_boolean(value):
    assert select(clearance_neutral_egress_enabled=value) is None


def test_default_false_is_identical_to_existing_selector():
    for settings in ({}, {'allow_parallel_static_egress':True}, {'min_clearance_gain':.05}):
        assert select(**settings) == select(**settings, clearance_neutral_egress_enabled=False)


def test_neutral_still_checks_commanded_axis_separately_from_clear_breadcrumbs():
    raw, inflated = grids(occupied={(0, 10), (5, 7)})
    history = [Breadcrumb(7.+i, .67-.10*i, 1.07 if i < 3 else 1.05, pi)
               for i in range(4)]
    geometry = GridGeometry(.1, 0., 0.)
    # The 2cm lateral history drift is valid. Its raw sweep is clear, but the
    # actual BackUp axis y=1.05 comes within .25m of the side square, below
    # .24 body + .02 margin + .005 sample reserve. Neutral preference cannot
    # authorize this shortcut through the physical chassis envelope.
    assert select(history, raw=raw, inflated=inflated, geometry=geometry,
        robot_radius=.24, clearance_neutral_egress_enabled=True) is None
