#!/usr/bin/env python3
"""Pure, bounded recorded geometry replay; never creates ROS processes."""
import argparse
from hashlib import sha256
import json
from math import hypot
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'ros2_ws/src/smart_vision_amr'))
sys.path.insert(0, str(ROOT/'benchmarks/driving_parameter_tuning_20261104/fine_geometry_feasibility_20261105'))
from geometry_tools import load_saved_map
from smart_vision_amr.grid import Grid
from smart_vision_amr.metric_connector import plan_connector, occupied_rectangles, polyline_clearance
from smart_vision_amr.path_geometry import path_progress


def replay(case):
    folder = Path(__file__).parent
    common = folder.parent/'physical_transfer_20261105/common_coherent_v2_geometry_tracking_inputs.json'
    source = json.loads(common.read_text())
    reference = [tuple(p) for p in source['owned_tracking']['path']['points']]
    snapshot = source['snapshot']
    pose = snapshot['planner']['details']['start_certificate_diagnostics']['fresh_pose']
    observed = {tuple(c) for c in snapshot['observed_snapshot_cells']}
    remembered = {tuple(c) for c in snapshot['remembered_snapshot_cells']}
    input_path = common
    if case == 'stress':
        input_path = folder/'stress_first_no_path.json'
        source = json.loads(input_path.read_text())
        pose = source['planner']['details']['start_certificate_diagnostics']['fresh_pose']
        reference = [tuple(p) for p in source['last_published_nonempty_path']['points']]
        observed = {(i%86, i//86) for i in source['observed_observer']['cells']}
        remembered = {(i%86, i//86) for i in source['remembered_observer']['cells']}
    metadata, pixels = load_saved_map(ROOT/'ros2_ws/src/smart_vision_amr/maps/classroom_20261030_cleaned.yaml')
    grid = Grid(tuple(tuple(v<0 or v>=65 for v in row) for row in pixels))
    resolution = metadata['resolution']
    ox, oy = metadata['origin']['x'], metadata['origin']['y']
    def local(point):
        return (point[0]-ox)/resolution, (point[1]-oy)/resolution
    progress, traveled, tails = path_progress(reference, pose[:2]), 0., []
    for index, (a,b) in enumerate(zip(reference,reference[1:]),1):
        traveled += hypot(b[0]-a[0], b[1]-a[1])
        if progress+.30 <= traveled <= progress+1.20:
            tails.append([local(p) for p in reference[index:]])
        if len(tails)>=10:
            break
    result = plan_connector(grid,observed|remembered,local(pose),tails,
        radius=.22/resolution,spacing=.01/resolution,margin=0.,
        lateral=.11/resolution,preferred=.45/resolution,gain=1.5,
        max_seconds=.06,max_expansions=1500)
    return {
        'case':case, 'scope':'Pure recorded geometry replay; no ROS/Gazebo/physical drive',
        'input':str(input_path.relative_to(ROOT)),
        'input_sha256':sha256(input_path.read_bytes()).hexdigest(),
        'implementation_sha256':sha256((ROOT/'ros2_ws/src/smart_vision_amr/smart_vision_amr/metric_connector.py').read_bytes()).hexdigest(),
        'body_radius_m':.22,'added_hard_padding_m':0.,'budget_ms':60.,'max_expansions':1500,
        'pose':pose,'observed_cells':len(observed),'remembered_cells':len(remembered),
        'result':result.reason,'helper_duration_ms':result.duration_s*1000.,'expansions':result.expansions,
        'helper_timing_excludes':'Input file decode; adapter fresh pose/snapshot/raw acquisition TF and raw endpoint sweep; Path publication',
        'path_world':[(ox+p[0]*resolution,oy+p[1]*resolution) for p in result.points],
        'minimum_original_square_body_gap_m':(polyline_clearance(result.points,
            occupied_rectangles(grid,observed|remembered),grid.width,grid.height)*resolution-.22
            if result.points else None),
        'gazebo_pass_claimed':False,'physical_robot_pass_claimed':False}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=('common','stress'),required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        parser.error('Output exists; use a new path')
    report=replay(args.case)
    with args.output.open('x') as output:
        json.dump(report,output,indent=2);output.write('\n')
    print(json.dumps({k:v for k,v in report.items() if k!='path_world'}))
