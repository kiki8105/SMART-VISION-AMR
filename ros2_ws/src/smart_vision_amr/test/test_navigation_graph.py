from smart_vision_amr.navigation_graph import goal_consumers_ready


def test_passive_recorders_do_not_make_navigation_ready():
    assert not goal_consumers_ready(['flight_recorder'])
    assert not goal_consumers_ready(['flight_recorder', 'astar_planner'])
    assert not goal_consumers_ready(['path_follower'])
    assert goal_consumers_ready(['flight_recorder', 'astar_planner', 'path_follower'])
