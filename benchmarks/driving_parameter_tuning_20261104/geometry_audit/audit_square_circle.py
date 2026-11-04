#!/usr/bin/env python3
"""Offline counterfactual masks only; never instantiate a ROS node or change policy."""
import argparse
import hashlib
import json
import math
from datetime import datetime
from pathlib import Path
import sys

import numpy as np

REPO=Path(__file__).resolve().parents[3]
PACKAGE=REPO/'ros2_ws/src/smart_vision_amr'
sys.path[:0]=[str(PACKAGE),str(PACKAGE/'test'),str(REPO/'benchmarks/driving_parameter_tuning_20261104')]
from smart_vision_amr.grid import Grid
from smart_vision_amr.map_utils import inflate_cells,inflate_cells_footprint,cell_to_world
from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.path_geometry import smooth_grid_path
from test_occlusion_reserve_incident import load_incident
from screen_planner import continuous_square_clearance,point_square_distance
from test_local_detour_planner import path_box_clearance

RADIUS=.22

def rectangles(cells,info):
    cells=np.asarray(sorted(cells),dtype=float).reshape((-1,2))
    lower=cells*info.resolution+np.asarray([info.origin.position.x,info.origin.position.y])
    return lower,lower+info.resolution

def point_gap(point,cells,info):
    if not cells:return None
    lower,upper=rectangles(cells,info)
    return float(point_square_distance([point],lower,upper)[0])-RADIUS

def inspect_snapshot(label,node,record,*,metadata=None,observed=None,remembered=None,pose=None,truth=None):
    info=node.map[0].info
    w,h=info.width,info.height
    start,goal=tuple(record['start_cell']),tuple(record['goal_cell'])
    observed=set(node.observed if observed is None else observed)
    remembered=set(node.remembered if remembered is None else remembered)
    raw={(x,y) for y,row in enumerate(node.raw_static_grid.rows) for x,v in enumerate(row) if v}
    exact_static=inflate_cells_footprint(raw,w,h,RADIUS/info.resolution)
    # Preserve the whole-center-cell condition against outside-map unknown too.
    exact_static.update((x,y) for y in range(h) for x in range(w)
        if min(x,y,w-x-1,h-y-1)*info.resolution<=RADIUS)
    grids={'square5':node.map[1],
        'circle22_fullcells':Grid(tuple(tuple((x,y) in exact_static for x in range(w)) for y in range(h)))}
    memory_masks={'square5':inflate_cells(remembered,w,h,5),
        'circle22_fullcells':inflate_cells_footprint(remembered,w,h,RADIUS/info.resolution)}
    observed_mask=inflate_cells_footprint(observed,w,h,RADIUS/info.resolution)
    hard_raw=raw|observed|remembered
    lower,upper=rectangles(hard_raw,info)
    bounds=[info.origin.position.x,info.origin.position.y,
        info.origin.position.x+w*info.resolution,info.origin.position.y+h*info.resolution]
    rows=[]
    for static_name,grid in grids.items():
        for memory_name,memory_mask in memory_masks.items():
            blocked=memory_mask|observed_mask
            route=AStarPlanner().plan(grid,start,goal,blocked)
            row={'static':static_name,'memory':memory_name,
                'start_static_blocked':grid.is_static_obstacle(start),
                'start_memory_blocked':start in memory_mask,
                'start_live_blocked':start in observed_mask,
                'route_cells':len(route),'full_segments_verified':False}
            if route:
                points,_=smooth_grid_path(route,lambda c:grid.is_static_obstacle(c) or c in blocked,
                    spacing=1.,shortcut_distance=16.,iterations=2)
                world=[cell_to_world(info,(x-.5,y-.5)) for x,y in points]
                gap=continuous_square_clearance(world,lower,upper)-RADIUS
                gap=min(gap,min(min(x-bounds[0],bounds[2]-x,y-bounds[1],bounds[3]-y) for x,y in world)-RADIUS)
                boxgap=path_box_clearance(world,record['actor']['bounds'])-RADIUS
                row.update(raw_occupied_unknown_live_memory_gap_m=gap,physical_box_gap_m=boxgap,
                    full_segments_verified=gap>=-1e-10 and boxgap>=-1e-10,
                    path_length_m=sum(math.dist(a,b) for a,b in zip(world,world[1:])))
            rows.append(row)
    pose=pose or record.get('pose_world',cell_to_world(info,start))
    diagnostics={}
    for name,p in [('reported_pose',pose),('truth',truth)]:
        if p is not None:
            diagnostics[name]={'xy':p[:2],'static_gap_m':point_gap(p[:2],raw,info),
                'memory_gap_m':point_gap(p[:2],remembered,info),
                'live_gap_m':point_gap(p[:2],observed,info),
                'physical_box_gap_m':path_box_clearance([p[:2],p[:2]],record['actor']['bounds'])-RADIUS}
    return {'label':label,'metadata':metadata or {},'start':start,'goal':goal,
        'actor_bounds':record['actor']['bounds'],
        'observed_cells':sorted(observed),'remembered_cells':sorted(remembered),
        'square_memory_cells':len(memory_masks['square5']),
        'circle_memory_cells':len(memory_masks['circle22_fullcells']),
        'memory_square_only_cells':len(memory_masks['square5']-memory_masks['circle22_fullcells']),
        'continuous_start':diagnostics,'counterfactual_masks':rows}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--baseline-directory',type=Path)
    parser.add_argument('--diagnostics',type=Path)
    args=parser.parse_args()
    if bool(args.baseline_directory) != bool(args.diagnostics):
        parser.error('--baseline-directory and --diagnostics must be supplied together')
    if args.output.exists():
        parser.error('Refusing to overwrite audit output')
    rows=[]
    for path in sorted((PACKAGE/'test/fixtures').glob('occlusion_reserve*20261104.json')):
        node,record=load_incident(path)
        rows.append(inspect_snapshot(path.name,node,record,
            metadata={'fixture_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                      'pose_note':'recorded pose where present; otherwise nominal start-cell center, not live truth'}))
        if record.get('later_memory_block_snapshot'):
            later=record['later_memory_block_snapshot']
            rows.append(inspect_snapshot(path.name+':later_memory',node,record,
                observed={tuple(c) for c in later['observed_cells']},
                remembered={tuple(c) for c in later['remembered_cells']},
                pose=cell_to_world(node.map[0].info,tuple(record['start_cell'])),
                metadata={'pose_note':'start-cell center only; fixture has no synchronized later pose/truth'}))
    if args.baseline_directory is not None:
        # Latest baseline: durable memory plus the most recent independent
        # observed/truth recorder samples, with all acquisition offsets disclosed.
        root=args.baseline_directory
        diagnostic_path=args.diagnostics
        memory=[json.loads(l) for l in (root/'memory_observer.jsonl').read_text().splitlines()]
        timeline=[json.loads(l) for l in (root/'safety_timeline.jsonl').read_text().splitlines()]
        actor=json.loads((root/'web_obstacle_injection.json').read_text())
        for diagnostic in json.loads(diagnostic_path.read_text())['all_diagnostics'][:2]:
            at=diagnostic['wall_time'];data=diagnostic['diagnostic']['start_certificate']
            snapshot={}
            for kind in ('observed','remembered'):
                t,sample=max((datetime.fromisoformat(r['wall_utc']).timestamp(),r) for r in memory
                    if r.get('kind')==kind and datetime.fromisoformat(r['wall_utc']).timestamp()<=at)
                snapshot[kind]={'age_s':at-t,'cells':[(i%86,i//86) for i in sample['cells']]}
            truth=max((r for r in timeline if r.get('kind')=='truth' and r['wall_time']<=at),key=lambda r:r['wall_time'])
            box=max((r for r in actor['events'] if r['event'] in ('obstacle_spawned','obstacle_moved') and r['at']<=at),key=lambda r:r['at'])
            node,record=load_incident()
            record['start_cell']=data['start'];record['goal_cell']=[27,65]
            record['actor']['bounds']=[box['x']-.125,box['y']-.125,box['x']+.125,box['y']+.125]
            rows.append(inspect_snapshot('latest_baseline:'+str(at),node,record,
                observed=snapshot['observed']['cells'],remembered=snapshot['remembered']['cells'],
                pose=data['start_tf_pose'],truth=[truth['x'],truth['y']],metadata={
                    'source_trial':str(root),'diagnostic_wall_epoch':at,
                    'diagnostic_start_tf_stamp_ns':data['start_tf_stamp_ns'],
                    'diagnostic_gate':data['first_failed_reason'],
                    'diagnostic_memory_blocked_measured':data['measured_remembered_inflated_start'],
                    'diagnostic_memory_blocked_with_virtual_reserve':data['remembered_inflated_start'],
                    'observed_receipt_age_s':snapshot['observed']['age_s'],
                    'memory_update_age_s':snapshot['remembered']['age_s'],
                    'truth_receipt_age_s':at-truth['wall_time'],
                    'warning':'Independent observer queue; not an exact atomic dump of planner state. Durable memory contents match reported count.'}))
    result={'scope':'Offline diagnostic; no ROS node/motors, no runtime policy/config edits',
        'radius_m':RADIUS,'resolution_m':.05,
        'source_sha256':{str(path.relative_to(REPO)):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (PACKAGE/'maps/classroom_20261030_cleaned.pgm',
                         PACKAGE/'smart_vision_amr/map_utils.py',
                         PACKAGE/'smart_vision_amr/astar.py',
                         PACKAGE/'smart_vision_amr/path_geometry.py')},
        'policy_isolation':'Hard observed and remembered cells retained; virtual occlusion extensions excluded to isolate geometry; no start relaxation or certificate bypass. Exact masks conservatively cover all positions in both source and candidate cells.',
        'snapshots':rows}
    output=args.output
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(output)
    for row in rows:
        print(row['label'],'extra',row['memory_square_only_cells'],
            [(x['static'],x['memory'],x['start_memory_blocked'],x['route_cells'],x['full_segments_verified']) for x in row['counterfactual_masks']],
            row['continuous_start'])

if __name__=='__main__':main()
