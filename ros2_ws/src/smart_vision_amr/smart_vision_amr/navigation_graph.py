"""Readiness must come from command consumers, never passive observers."""

REQUIRED_GOAL_CONSUMERS = frozenset({'astar_planner', 'path_follower'})


def goal_consumers_ready(node_names):
    return REQUIRED_GOAL_CONSUMERS.issubset(set(node_names))
