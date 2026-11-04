#!/usr/bin/env python3
"""Recheck recorded paired actor inputs without ROS nodes, Gazebo or commands."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

from nav_msgs.msg import OccupancyGrid

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'ros2_ws/src/smart_vision_amr'))
from inject_classroom_web_obstacle import (movement_step_readiness,
    path_clearance_from_box, select_fixed_web_obstacle)
from smart_vision_amr.map_utils import cell_to_world, world_to_cell
from smart_vision_amr.web_mission_server import load_saved_map


def segment_rectangle_distance(first, last, low, high):
    delta = (last[0]-first[0],last[1]-first[1])
    enter,leave = 0.,1.
    for axis in (0,1):
        if abs(delta[axis])<1e-14:
            if not low[axis]<=first[axis]<=high[axis]:
                break
        else:
            a,b=(low[axis]-first[axis])/delta[axis],(high[axis]-first[axis])/delta[axis]
            enter,leave=max(enter,min(a,b)),min(leave,max(a,b))
    else:
        if enter<=leave:
            return 0.
    def point_gap(point):
        return math.hypot(max(low[0]-point[0],0.,point[0]-high[0]),
                          max(low[1]-point[1],0.,point[1]-high[1]))
    result=min(point_gap(first),point_gap(last))
    square=delta[0]**2+delta[1]**2
    if square:
        for x in (low[0],high[0]):
            for y in (low[1],high[1]):
                along=max(0.,min(1.,((x-first[0])*delta[0]+(y-first[1])*delta[1])/square))
                result=min(result,math.hypot(first[0]+along*delta[0]-x,
                                             first[1]+along*delta[1]-y))
    return result


def run():
    folder=Path(__file__).parent
    data=json.loads((folder/'fixture.json').read_text())
    map_path=ROOT/'ros2_ws/src/smart_vision_amr/maps'/data['map']['yaml']
    assert hashlib.sha256(map_path.read_bytes()).hexdigest()==data['map']['yaml_sha256']
    assert hashlib.sha256(map_path.with_suffix('.pgm').read_bytes()).hexdigest()==data['map']['image_sha256']
    metadata,pixels=load_saved_map(map_path)
    grid=OccupancyGrid();grid.header.frame_id='map'
    grid.info.width,grid.info.height=metadata['width'],metadata['height']
    grid.info.resolution=metadata['resolution']
    grid.info.origin.position.x,grid.info.origin.position.y=data['map']['origin'][:2]
    grid.info.origin.orientation.w=1.;grid.data=pixels.flatten().tolist()
    scene_path=ROOT/data['proposed_scene_path']
    assert hashlib.sha256(scene_path.read_bytes()).hexdigest()==data['proposed_scene_sha256']
    proposed=json.loads(scene_path.read_text())
    reports={}
    for label,scene in [('original',data['original_scene']),('proposed',proposed)]:
        center=(scene['x'],scene['y']);tangent=scene['path_tangent'];side=scene['movement_side']
        positions=[(center[0]-.025*step*side*tangent[1],center[1]+.025*step*side*tangent[0]) for step in range(5)]
        reports[label]={'scene':scene,'cases':{},'actor_positions':positions}
        world_min=float('inf');world_nearest=None
        for box in data['world_boxes']:
            lo,hi=box['bounds_xyz']
            low=(lo[0]-.125,lo[1]-.125);high=(hi[0]+.125,hi[1]+.125)
            distance=min(segment_rectangle_distance(a,b,low,high) for a,b in zip(positions,positions[1:]))
            if distance<world_min:
                world_min,world_nearest=distance,{'model':box['model'],'link':box['link']}
        reports[label]['whole_box_sweep_explicit_world_AABB_gap_m']=world_min
        reports[label]['nearest_world_collision']=world_nearest
        for profile,case in data['cases'].items():
            observed=case['observed_snapshot']['cells'];remembered=case['remembered_snapshot']['cells']
            observed_xy=[(index,cell_to_world(grid.info,(index%grid.info.width,index//grid.info.width))) for index in observed]
            diagnostics={};candidate=select_fixed_web_obstacle(case['path']['points'],case['actor_search_event']['robot_xy'],
                data['goal_xy'],grid,world_to_cell,cell_to_world,observed,diagnostics,
                scenario=scene,remembered_indices=remembered,robot_radius=.22,obstacle_inflation_cells=5)
            sweep=[]
            for a,b in zip(positions,positions[1:]):
                ready,metrics=movement_step_readiness(a,b,case['actor_search_event']['robot_xy'],0.,robot_radius=.22)
                sweep.append({'passed_at_recorded_initial_Home_only':ready,**metrics})
            reports[label]['cases'][profile]={'diagnostics':diagnostics,'candidate':candidate,
                'nearest_observed':min(({'index':index,'cell':[index%86,index//86],
                    'center':xy,'center_distance_m':math.dist(center,xy)} for index,xy in observed_xy),
                    key=lambda item:item['center_distance_m']),
                'initial_physical_body_gap_m':path_clearance_from_box([case['actor_search_event']['robot_xy']],center,.22),
                'four_box_moves_with_frozen_initial_robot':sweep}
    assert reports['original']['cases']['sensor_rates']['candidate']
    assert reports['original']['cases']['wheel_asymmetry']['diagnostics']['reason']=='FIXED_SCENE_TOO_CLOSE_TO_OBSERVED_CELL'
    for case in reports['proposed']['cases'].values():
        assert case['candidate'] and case['diagnostics']['input_status']=='VALID'
        assert case['candidate']['initial_path_clearance_m']<-.01
        assert len(case['candidate']['preflight']['route_cells_by_step'])==5
        assert all(item['passed_at_recorded_initial_Home_only'] for item in case['four_box_moves_with_frozen_initial_robot'])
    assert reports['proposed']['whole_box_sweep_explicit_world_AABB_gap_m']>0
    return {'scope':data['scope'],'input_sha256':{
                'fixture.json':hashlib.sha256((folder/'fixture.json').read_bytes()).hexdigest(),
                'proposed_scene':data['proposed_scene_sha256'],
                'production_injector':hashlib.sha256((ROOT/'scripts/inject_classroom_web_obstacle.py').read_bytes()).hexdigest()},
            'reports':reports,'limits':data['limits']}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=run();args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({label:{profile:case['diagnostics']['input_status'] for profile,case in result['cases'].items()}
                      for label,result in report['reports'].items()},indent=2))


if __name__=='__main__':
    main()
