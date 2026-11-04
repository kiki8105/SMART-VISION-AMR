"""A verified recent position corridor, not past yaw equality, guides retreat.

These controlled maps isolate the selector. Source-stamp/frame/jump and paired
projection validation are independently covered by the odometry adapter tests.
Every accepted reverse still uses the same physical commanded-axis checks.
"""
from math import hypot, pi
from unittest.mock import Mock

import pytest

from smart_vision_amr.grid import Grid
from smart_vision_amr.reverse_recovery import Breadcrumb, GridGeometry, _raw_clearance, select_reverse_egress
from smart_vision_amr.odometry_trail import OdometryTrail
from test_reverse_recovery import grids, select, trail


def curve():
    # Positions remain within 2 cm of the current straight reverse axis even
    # though the approach accumulated the ~.38 rad recorded heading change.
    return [Breadcrumb(7.,.65,1.07,pi+.38), Breadcrumb(8.,.55,1.065,pi+.30),
            Breadcrumb(9.,.45,1.055,pi+.20), Breadcrumb(10.,.35,1.05,pi)]


def test_verified_position_corridor_replaces_historical_heading_heuristic():
    history=curve()
    assert select(history) is None
    candidate=select(history,verified_command_corridor=True)
    assert candidate is not None
    assert candidate.distance_m==pytest.approx(.30)
    assert (candidate.target_x,candidate.target_y)==pytest.approx((.65,1.05))
    assert candidate.start_clearance_m>=.22+.02+.005
    assert candidate.target_clearance_m>=.22+.02+.005


def test_stationary_in_place_heading_samples_do_not_erase_real_position_trace():
    history=curve()
    latest=history.pop()
    history.extend([Breadcrumb(9.5,latest.x,latest.y,pi+.35),latest])
    assert select(history) is None
    candidate=select(history,verified_command_corridor=True)
    assert candidate is not None and candidate.distance_m==pytest.approx(.30)


def test_clear_curved_breadcrumb_sweep_cannot_authorize_colliding_straight_chord():
    raw,inflated=grids(occupied={(0,10),(5,7)})
    history=[Breadcrumb(7.+i,.67-.10*i,1.05 if i<3 else 1.03,
                        pi+.38*(3-i)/3) for i in range(4)]
    geometry=GridGeometry(.1,0.,0.)
    occupied=tuple((x,y) for y,row in enumerate(raw.rows) for x,value in enumerate(row) if value)
    required=.22+.02+.005
    # Independent dense probe establishes that the past curved body corridor
    # itself is clear. The commanded y=1.03 ray comes within only23cm of the
    # side square, below the unchanged24.5cm physical requirement.
    for a,b in zip(history,history[1:]):
        count=max(1,int(hypot(b.x-a.x,b.y-a.y)/.001)+1)
        assert min(_raw_clearance(raw,geometry,occupied,
            a.x+(b.x-a.x)*i/count,a.y+(b.y-a.y)*i/count)
            for i in range(count+1))>=required
    assert _raw_clearance(raw,geometry,occupied,.55,1.03)==pytest.approx(.23)
    oracle=Mock(return_value=True)
    assert select(history,raw=raw,inflated=inflated,geometry=geometry,
        robot_radius=.22,route=oracle,clearance_neutral_egress_enabled=True,
        verified_command_corridor=True) is None
    oracle.assert_not_called()


@pytest.mark.parametrize('fault',['lateral','raw_hit','stale','source_order','history_age',
                                  'pose_jump','current_heading','no_endpoint','no_goal'])
def test_verified_corridor_does_not_relax_physical_temporal_or_goal_gates(fault):
    history=curve();raw,inflated=grids();settings={}
    if fault=='lateral':
        p=history[-2];history[-2]=Breadcrumb(p.stamp,p.x,p.y+.04,p.yaw)
    elif fault=='raw_hit':
        raw,inflated=grids(occupied={(0,10),(5,10)})
    elif fault=='stale':settings['now']=12.
    elif fault=='source_order':
        p=history[-2];history[-2]=Breadcrumb(10.,p.x,p.y,p.yaw)
    elif fault=='history_age':settings['max_history_age']=.5
    elif fault=='pose_jump':history=[history[0],history[-1]]
    elif fault=='current_heading':settings['current_pose']=(history[-1].x,history[-1].y,pi+.16)
    elif fault=='no_endpoint':inflated=Grid(tuple(tuple(True for _ in row) for row in inflated.rows))
    elif fault=='no_goal':settings['route']=lambda _:False
    assert select(history,raw=raw,inflated=inflated,verified_command_corridor=True,**settings) is None


@pytest.mark.parametrize('bad',[None,1,0,'true',float('nan')])
def test_verified_corridor_requires_strict_boolean_opt_in(bad):
    assert select(verified_command_corridor=bad) is None


def test_default_false_retains_original_selector_behavior():
    for history in (trail(),curve()):
        assert select(history)==select(history,verified_command_corridor=False)


def test_continuous_stationary_source_keeps_actual_old_spatial_anchor_but_real_gap_revokes_it():
    source=OdometryTrail()
    # A real3s approach is followed by53s of received source/receipt-stamped
    # stillness. The older travelled points must not receive invented times.
    for index in range(561):
        at=44.+index*.1
        x=.65-.01*min(index,30)
        sample=(at,round(at*1e9),x,1.05,pi,'odom')
        assert source.observe(sample,'base_link')
    paired=(100.,100_000_000_000,.35,1.05,pi,'odom')
    projected=source.project((.35,1.05,pi),paired,now=100.)
    assert projected[0].stamp==44.
    assert projected[-2].stamp<48.
    assert projected[-1].stamp==100.
    raw,inflated=grids()
    settings=dict(now=100.,robot_radius=.22,max_history_age=90.,max_sample_gap=30.)
    geometry=GridGeometry(.1,0.,0.)
    assert select_reverse_egress(projected,raw,inflated,geometry,lambda _:True,**settings) is None
    assert select_reverse_egress(projected,raw,inflated,geometry,lambda _:True,
        **settings,verified_command_corridor=True) is not None
    assert source.observe((100.76,100_760_000_000,.35,1.05,pi,'odom'),'base_link')
    after_gap=source.project((.35,1.05,pi),
        (100.76,100_760_000_000,.35,1.05,pi,'odom'),now=100.76)
    assert source.last_reset_reason=='unobserved_gap'
    assert len(after_gap)==1
    assert select_reverse_egress(after_gap,raw,inflated,geometry,lambda _:True,
        **dict(settings,now=100.76),verified_command_corridor=True) is None
