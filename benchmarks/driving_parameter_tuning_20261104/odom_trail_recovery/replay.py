#!/usr/bin/env python3
"""Replay actual odom through old/new selector; no ROS, executor or drive.

Free-map screening isolates the changed corridor policy using identical real
odometry. Only the late87.85s snapshot has archived dynamic geometry; report
its original stale15 and independently confirmed0 separately, without calling
any offline selection a completed physical or Gazebo reverse.
"""
import argparse, gzip, hashlib, importlib.util, json, math, sys
from dataclasses import asdict
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/smart_vision_amr'))
from smart_vision_amr.odometry_trail import OdometryTrail
from smart_vision_amr.reverse_recovery import select_reverse_egress, GridGeometry
from smart_vision_amr.grid import Grid
from smart_vision_amr.astar import AStarPlanner
from smart_vision_amr.map_utils import inflate_cells, inflate_cells_footprint


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
    return module


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('Refusing to overwrite a result')
    source=HERE/'actual_odom_input.json.gz'
    data=json.loads(gzip.decompress(source.read_bytes()))
    old=load('old_reverse_corridor',HERE/'reverse_recovery_before_command_corridor.py')
    samples=data['odom'];out=[];old_map_geometry=GridGeometry(.1,-5.,-5.)
    free=Grid(tuple((False,)*100 for _ in range(100)))
    for probe in data['probes']:
        history=OdometryTrail()
        actual=[p for p in samples if p['monotonic']<=probe['monotonic']]
        for point in actual:
            history.observe((point['monotonic'],point['stamp_ns'],point['x'],point['y'],point['yaw'],point['frame_id']),point['child_frame_id'])
        latest=actual[-1]
        paired=(latest['monotonic'],latest['stamp_ns'],latest['x'],latest['y'],latest['yaw'],latest['frame_id'])
        projected=history.project(probe['current_map_pose'],paired,now=probe['monotonic'])
        options=dict(now=probe['monotonic'],robot_radius=.22,clearance_margin=.01,
                     max_history_age=90.,max_sample_gap=30.,current_pose=probe['current_map_pose'])
        a=old.select_reverse_egress(projected,free,free,old_map_geometry,lambda cell:True,**options)
        b=select_reverse_egress(projected,free,free,old_map_geometry,lambda cell:True,
                               verified_command_corridor=True,**options)
        out.append({'probe_t':probe['t'],'source_samples':len(actual),'original_map_trail_count':len(probe['old_map_trail']),
                    'source_verified_projected_count':len(projected),'controlled_free_geometry_old':None if a is None else asdict(a),
                    'controlled_free_geometry_new':None if b is None else asdict(b)})
        if probe==data['probes'][-1]:late=(probe,projected,options)
    # Exact recorded late snapshot; no unseen early scene reconstruction.
    path=HERE.parent/'start_egress_audit/fixture.json';record=json.loads(path.read_text());meta=record['map'];par=record['parameters']
    image=ROOT/meta['image'];assert hashlib.sha256(image.read_bytes()).hexdigest()==meta['sha256']
    with image.open('rb') as stream:
        assert stream.readline()==b'P5\n';width,height=map(int,stream.readline().split());assert stream.readline()==b'255\n';pixels=stream.read()
    static={(x,y) for y in range(height) for x in range(width) if pixels[(height-y-1)*width+x]!=254}
    inflation=inflate_cells(static,width,height,5)
    raw_static=Grid(tuple(tuple((x,y) in static for x in range(width)) for y in range(height)))
    inflated_static=Grid(tuple(tuple((x,y) in inflation for x in range(width)) for y in range(height)))
    geometry=GridGeometry(meta['resolution'],*meta['origin'])
    observed={tuple(c) for c in record['observed_cells']}
    exact=[]
    for label,memory in [('recorded_stale15',{tuple(c) for c in record['planner_remembered_cells']}),('independent_confirmed0',set())]:
        actual=static|observed|memory
        raw=Grid(tuple(tuple((x,y) in actual for x in range(width)) for y in range(height)))
        blocked=(inflate_cells_footprint(observed,width,height,.22/.05)
                 |inflate_cells(memory,width,height,5))
        full=Grid(tuple(tuple(inflated_static.rows[y][x] or (x,y) in blocked for x in range(width)) for y in range(height)))
        planner=AStarPlanner();goal=tuple(record['goal_cell'])
        oracle=lambda cell:bool(planner.plan(inflated_static,cell,goal,blocked))
        probe,projected,options=late
        a=old.select_reverse_egress(projected,raw,full,geometry,oracle,**options)
        b=select_reverse_egress(projected,raw,full,geometry,oracle,verified_command_corridor=True,**options)
        exact.append({'case':label,'raw_observed':len(observed),'raw_memory':len(memory),
                      'old':None if a is None else asdict(a),'new':None if b is None else asdict(b)})
    files=[source,HERE/'reverse_recovery_before_command_corridor.py',ROOT/'ros2_ws/src/smart_vision_amr/smart_vision_amr/reverse_recovery.py',ROOT/'ros2_ws/src/smart_vision_amr/smart_vision_amr/odometry_trail.py',path,image]
    result={'scope':__doc__,'input_source_sha256':data['source_sha256'],'free_geometry_actual_trace_ab':out,
            'late_recorded_measured_geometry_ab':exact,'file_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
            'limitations':['No collision geometry exists for early29/36s probes; free-grid cases isolate corridor eligibility only.',
                           'Late measured geometry keeps static/unknown/observed/memory hard. Original virtual-reserve depth is not added to this measured-only endpoint screen.',
                           'No action/stop handshake or closed-loop execution is simulated by this pure replay.']}
    args.output.write_text(json.dumps(result,indent=2))
    print(json.dumps({'free_cases':[(r['probe_t'],bool(r['controlled_free_geometry_old']),bool(r['controlled_free_geometry_new'])) for r in out],'late_cases':exact},indent=2))


if __name__=='__main__':main()
