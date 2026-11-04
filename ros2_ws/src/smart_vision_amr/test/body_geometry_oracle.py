"""Independent rectangle geometry for tests; no production inflation helpers."""
from math import atan2, cos, sin

import numpy as np


def full_cell_body_mask(cells, info, radius, margin=0.):
    """Circle centers anywhere in a candidate square versus occupied squares."""
    if not cells:
        return set()
    yy, xx = np.indices((info.height, info.width))
    candidates = np.column_stack((xx.ravel(), yy.ravel()))
    lower = candidates*info.resolution
    upper = lower+info.resolution
    obstacle_lower = np.array(sorted(cells), dtype=float)*info.resolution
    obstacle_upper = obstacle_lower+info.resolution
    # Direct closed-rectangle separation, independent of the production
    # integer offset kernel and its construction/iteration bounds.
    dx = np.maximum(np.maximum(lower[:,None,0]-obstacle_upper[None,:,0],
                               obstacle_lower[None,:,0]-upper[:,None,0]), 0.)
    dy = np.maximum(np.maximum(lower[:,None,1]-obstacle_upper[None,:,1],
                               obstacle_lower[None,:,1]-upper[:,None,1]), 0.)
    occupied = np.any(np.hypot(dx,dy) <= radius+margin+1e-12, axis=1)
    return {tuple(map(int, cell)) for cell in candidates[occupied]}


def _segment_rectangle_distance(first, last, lower, upper):
    first, last = np.asarray(first), np.asarray(last)
    delta = last-first
    enter, leave = np.zeros(len(lower)), np.ones(len(lower))
    possible = np.ones(len(lower), dtype=bool)
    for axis in (0,1):
        if abs(delta[axis]) < 1e-14:
            possible &= (lower[:,axis] <= first[axis]) & (first[axis] <= upper[:,axis])
        else:
            first_hit = (lower[:,axis]-first[axis])/delta[axis]
            last_hit = (upper[:,axis]-first[axis])/delta[axis]
            enter = np.maximum(enter, np.minimum(first_hit,last_hit))
            leave = np.minimum(leave, np.maximum(first_hit,last_hit))
    if np.any(possible & (enter <= leave)):
        return 0.
    def point_distance(point):
        return float(np.hypot(*np.maximum(np.maximum(lower-point,point-upper),0.).T).min())
    minimum = min(point_distance(first),point_distance(last))
    squared = float(delta@delta)
    if squared:
        for x,y in ((0,0),(0,1),(1,0),(1,1)):
            corner = np.column_stack(((upper if x else lower)[:,0],
                                      (upper if y else lower)[:,1]))
            along = np.clip(((corner-first)*delta).sum(axis=1)/squared,0.,1.)
            offsets = first+along[:,None]*delta-corner
            minimum = min(minimum,float(np.hypot(*offsets.T).min()))
    return minimum


def continuous_raw_body_gap(node, points):
    """Exact polyline distance to raw static/unknown/live/memory cell areas."""
    info = node.map[0].info
    q = info.origin.orientation
    yaw = atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    c,s = cos(yaw),sin(yaw)
    local = np.array([(c*(x-info.origin.position.x)+s*(y-info.origin.position.y),
                       -s*(x-info.origin.position.x)+c*(y-info.origin.position.y))
                      for x,y in points])
    radius = node.params['robot_radius']
    raw = {(x,y) for y in range(info.height) for x in range(info.width)
           if node.raw_static_grid.is_static_obstacle((x,y))}
    raw |= node.observed|node.remembered
    lower = np.array(sorted(raw),dtype=float)*info.resolution
    upper = lower+info.resolution
    # Distant rectangles cannot affect a body-clearance assertion. This
    # bounds temporary arrays without changing any near-body raw geometry.
    low,high = local.min(axis=0)-radius-.05,local.max(axis=0)+radius+.05
    selected = np.all((upper >= low)&(lower <= high),axis=1)
    lower,upper = lower[selected],upper[selected]
    boundary = min(float(local[:,0].min()),float(local[:,1].min()),
                   info.width*info.resolution-float(local[:,0].max()),
                   info.height*info.resolution-float(local[:,1].max()))
    minimum = boundary
    if len(lower):
        minimum = min(minimum,*( _segment_rectangle_distance(a,b,lower,upper)
                                  for a,b in zip(local,local[1:])))
    return minimum-radius
