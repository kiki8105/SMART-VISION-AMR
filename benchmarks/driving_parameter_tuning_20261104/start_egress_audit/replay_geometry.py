#!/usr/bin/env python3
"""Read-only exact cell-area geometry; no ROS/executor/Gazebo/drive output.

A fixture contains the planner's reconstructed stale15 snapshot and the
independently observed empty0. Real occupied/unknown and measured obstacle
squares are never removed from the corresponding scenario's collision check.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'ros2_ws/src/smart_vision_amr'))
from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.grid import Grid
from smart_vision_amr.map_utils import cell_to_world,inflate_cells,inflate_cells_footprint
from smart_vision_amr.path_geometry import path_progress,point_at_progress,route_cells


def point_distance(point,lower,upper):
    if not len(lower):return float('inf')
    x,y=point
    return float(np.hypot(np.maximum(np.maximum(lower[:,0]-x,x-upper[:,0]),0.),
                          np.maximum(np.maximum(lower[:,1]-y,y-upper[:,1]),0.)).min())


def segment_distance(first,last,lower,upper):
    """Exact segment/closed axis-aligned rectangles distance."""
    if not len(lower):return float('inf')
    first,last=np.asarray(first),np.asarray(last)
    delta=last-first
    entry,leave=np.zeros(len(lower)),np.ones(len(lower))
    intersects=np.ones(len(lower),dtype=bool)
    for axis in (0,1):
        if abs(delta[axis])<1e-14:
            intersects&=(lower[:,axis]<=first[axis])&(first[axis]<=upper[:,axis])
        else:
            a=(lower[:,axis]-first[axis])/delta[axis]
            b=(upper[:,axis]-first[axis])/delta[axis]
            entry=np.maximum(entry,np.minimum(a,b));leave=np.minimum(leave,np.maximum(a,b))
    if np.any(intersects&(entry<=leave)):return 0.
    best=min(point_distance(first,lower,upper),point_distance(last,lower,upper))
    square=float(delta@delta)
    if square:
        for ix,iy in ((0,0),(0,1),(1,0),(1,1)):
            corners=np.column_stack(((upper if ix else lower)[:,0],(upper if iy else lower)[:,1]))
            fraction=np.clip(((corners-first)*delta).sum(axis=1)/square,0.,1.)
            gap=first+fraction[:,None]*delta-corners
            best=min(best,float(np.hypot(gap[:,0],gap[:,1]).min()))
    return best


def reserve_depth(hits,reference,start,width,height,resolution,extension,trigger):
    """Archived production reserve geometry, replayed against saved reference."""
    reserved=set(hits);origin=(start[0]+.5,start[1]+.5)
    progress=path_progress(reference,origin)
    for cell in hits:
        center=(cell[0]+.5,cell[1]+.5)
        if math.dist(center,origin)>trigger/resolution:continue
        along=path_progress(reference,center);point=point_at_progress(reference,along)
        if along<progress-1. or math.dist(center,point)>6.:continue
        ahead=point_at_progress(reference,along+1.)
        delta=(ahead[0]-point[0],ahead[1]-point[1]);length=math.hypot(*delta)
        if length<1e-9:continue
        end=(center[0]+extension/resolution*delta[0]/length,
             center[1]+extension/resolution*delta[1]/length)
        reserved.update(c for c in route_cells([center,end]) if 0<=c[0]<width and 0<=c[1]<height)
    return reserved


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture',type=Path,default=Path(__file__).with_name('fixture.json'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('Refusing to overwrite audit output')
    data=json.loads(args.fixture.read_text());metadata=data['map'];parameters=data['parameters']
    frozen=Path(__file__).with_name('continuous_start_egress_before_memory_wake.py')
    helper_key='ros2_ws/src/smart_vision_amr/smart_vision_amr/continuous_start_egress.py'
    assert hashlib.sha256(frozen.read_bytes()).hexdigest()==data['source_files_sha256'][helper_key]
    spec=importlib.util.spec_from_file_location('smart_vision_amr._frozen_start_egress_audit',frozen)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    certified_start_only_edges=module.certified_start_only_edges
    image=ROOT/metadata['image']
    assert hashlib.sha256(image.read_bytes()).hexdigest()==metadata['sha256']
    with image.open('rb') as stream:
        assert stream.readline()==b'P5\n';width,height=map(int,stream.readline().split())
        assert stream.readline()==b'255\n';pixels=stream.read()
    assert (width,height)==(metadata['width'],metadata['height'])
    assert len(pixels)==width*height and set(pixels)<={0,205,254}
    raw=Grid(tuple(tuple(pixels[(height-y-1)*width+x]!=254 for x in range(width)) for y in range(height)))
    occupied={(x,y) for y,row in enumerate(raw.rows) for x,blocked in enumerate(row) if blocked}
    static=inflate_cells(occupied,width,height,parameters['static_inflation_cells'])
    grid=Grid(tuple(tuple((x,y) in static for x in range(width)) for y in range(height)))
    resolution=metadata['resolution'];ox,oy,angle=metadata['origin'];assert angle==0.
    info=SimpleNamespace(resolution=resolution,origin=SimpleNamespace(position=SimpleNamespace(x=ox,y=oy),orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
    pose=data['pose'];start=tuple(data['start_cell']);goal=tuple(data['goal_cell'])
    observed,remembered=({tuple(cell) for cell in data[name]} for name in ('observed_cells','planner_remembered_cells'))
    radius=parameters['body_radius_m'];margin=parameters['start_egress_margin_m']
    reference=[((x-ox)/resolution,(y-oy)/resolution) for x,y in data['reference_path_record']['points']]
    ro,rm=(reserve_depth(hits,reference,start,width,height,resolution,parameters['occlusion_extension_m'],parameters['occlusion_trigger_m']) for hits in (observed,remembered))
    live=inflate_cells_footprint(ro,width,height,radius/resolution)
    memory=inflate_cells(rm,width,height,parameters['memory_inflation_cells'])
    full=live|memory
    measured=(inflate_cells_footprint(observed,width,height,radius/resolution)
              |inflate_cells(remembered,width,height,parameters['memory_inflation_cells']))
    counts={'observed_raw':len(observed),'planner_memory_raw':len(remembered),'observed_inflated':len(live),'remembered_inflated':len(memory),'dynamic_union':len(full),'measured_union':len(measured),'virtual_only':len(full-measured)}
    assert counts==data['expected_counts'],(counts,data['expected_counts'])
    bounds=(ox,oy,ox+width*resolution,oy+height*resolution)
    def rectangles(cells):
        lower=np.asarray(sorted(cells),dtype=float).reshape((-1,2))*resolution+[ox,oy]
        return lower,lower+resolution
    lower,upper=rectangles(occupied|observed|remembered)
    boundary=lambda p:min(p[0]-bounds[0],bounds[2]-p[0],p[1]-bounds[1],bounds[3]-p[1])
    clearance=lambda p:min(point_distance(p,lower,upper),boundary(p))
    sweep=lambda a,b:min(segment_distance(a,b,lower,upper),boundary(a),boundary(b))
    center=cell_to_world(info,start)
    pose_cells=((pose[0]-ox)/resolution,(pose[1]-oy)/resolution)
    helper=lambda blocked:sorted([list(a),list(b)] for a,b in certified_start_only_edges(raw,grid,observed|remembered,blocked,start,pose_cells,radius/resolution,margin/resolution))
    neighbors=[]
    for dx in (-1,0,1):
        for dy in (-1,0,1):
            if not(dx or dy):continue
            cell=(start[0]+dx,start[1]+dy);point=cell_to_world(info,cell)
            neighbors.append({'cell':cell,'full_inflation_blocked':cell in full or cell in static,'measured_inflation_blocked':cell in measured or cell in static,'direct_actual_pose_sweep_clearance_m':sweep(pose[:2],point),'via_nominal_center_sweep_clearance_m':min(sweep(pose[:2],center),sweep(center,point))})
    prefixes=[]
    for cell in ((36,82),(34,83),(33,84)):
        point=cell_to_world(info,cell)
        tail=AStarPlanner().plan(grid,cell,goal,measured)
        whole=[pose[:2],point]+[cell_to_world(info,c) for c in tail[1:]]
        minimum=min(sweep(a,b) for a,b in zip(whole,whole[1:]))
        prefixes.append({'exit_cell':cell,'prefix_length_m':math.dist(pose[:2],point),'prefix_center_clearance_m':sweep(pose[:2],point),'endpoint_outside_full_inflation':cell not in full and cell not in static,'endpoint_outside_measured_inflation':cell not in measured and cell not in static,'full_hard_virtual_tail_cells':len(AStarPlanner().plan(grid,cell,goal,full)),'measured_hard_tail_cells':len(tail),'full_measured_hard_prefix_and_tail_center_clearance_m':minimum,'full_measured_hard_prefix_and_tail_body_gap_m':minimum-radius,'note':'Offline geometric comparison only, no deployed forward escape relaxation. Virtual depth may be soft only under existing separately gated policy.'})
    normal_empty=AStarPlanner().plan(grid,start,goal,inflate_cells_footprint(observed,width,height,radius/resolution))
    empty_lower,empty_upper=rectangles(occupied|observed)
    points=[pose[:2]]+[cell_to_world(info,c) for c in normal_empty]
    empty_min=min(segment_distance(a,b,empty_lower,empty_upper) for a,b in zip(points,points[1:]))
    contributors=sorted(c for c in remembered if max(abs(c[0]-start[0]),abs(c[1]-start[1]))<=5)
    result={'scope':data['scope'],'fixture_sha256':hashlib.sha256(args.fixture.read_bytes()).hexdigest(),'reconstructed_counts':counts,'raw_start_center_clearance_m':clearance(pose[:2]),'raw_start_body_gap_m':clearance(pose[:2])-radius,'nominal_cell_center_clearance_m':clearance(center),'legacy_halo_quantization_requirement_m':radius+resolution*math.sqrt(2)/2,'continuous_helper_full_allowed_edges':helper(full),'continuous_helper_measured_allowed_edges':helper(measured),'neighbors':neighbors,'start_memory_inflation_contributors':contributors,'offline_prefixes':prefixes,'observer_empty0_normal_astar':{'route_cells':len(normal_empty),'raw_occupied_unknown_observed_body_gap_m':empty_min-radius,'start_inflation_exception_used':False,'memory_source':'independent empty0 received at wall1793802779.377984, not used by the stale15 planner callback'},'conclusion':['The reported continuous22cm body is raw-free; the global masks are reconstructed exactly.','All eight adjacent cells remain memory-inflated. Existing one-cell selected-edge certification returns empty even if its remembered-inflation adapter gate were removed.','Do not delete confirmed memory or weaken its inflation from this audit. Fix processing/wake of relevant queued memory15-to-empty0 first.','An ordinary unchanged A* route exists once the producer-confirmed empty memory is consumed; no new forward exception is required for that snapshot.'],'limitations':data['limitations']}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('reconstructed_counts','raw_start_body_gap_m','continuous_helper_full_allowed_edges','observer_empty0_normal_astar')},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
