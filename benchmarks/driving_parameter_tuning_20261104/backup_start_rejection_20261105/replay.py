#!/usr/bin/env python3
"""First actual local025 reverse refusal: pure recorded-cell/source replay."""
import argparse,json,sys,hashlib,math
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/smart_vision_amr'))
from smart_vision_amr.grid import Grid
from smart_vision_amr.map_utils import inflate_cells
from smart_vision_amr.reverse_recovery import Breadcrumb,GridGeometry,select_reverse_egress
from smart_vision_amr.astar import AStarPlanner

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():p.error('Refusing to overwrite a result')
    source=HERE/'fixture.json';data=json.loads(source.read_text());image=ROOT/'ros2_ws/src/smart_vision_amr/maps/classroom_20261030_cleaned.pgm'
    with image.open('rb') as f:
        assert f.readline()==b'P5\n';width,height=map(int,f.readline().split());assert f.readline()==b'255\n';pixels=f.read()
    static={(x,y) for y in range(height) for x in range(width) if pixels[(height-y-1)*width+x]!=254}
    observed={(i%width,i//width) for i in data['observed_record']['cells']};memory={(i%width,i//width) for i in data['remembered_record']['cells']}
    inflation=inflate_cells(static,width,height,5);blocked={tuple(c) for c in data['blocked_dynamic_cells']}
    occupied=static|observed|memory;full=inflation|blocked
    raw=Grid(tuple(tuple((x,y) in occupied for x in range(width)) for y in range(height)))
    inflated=Grid(tuple(tuple((x,y) in full for x in range(width)) for y in range(height)))
    grid=Grid(tuple(tuple((x,y) in inflation for x in range(width)) for y in range(height)))
    diag=data['probe']['details']['reverse_probe_diagnostics'];pose=diag['current_pose'];geometry=GridGeometry(.05,-1.65,-5.25)
    history=[Breadcrumb(*p) for p in diag['recent_trail']];planner=AStarPlanner();calls=[];reasons=[]
    def oracle(cell):
        route=planner.plan(grid,cell,(27,65),blocked);calls.append([list(cell),len(route)]);return bool(route)
    def trace(frame,event,arg):
        if frame.f_code.co_name=='select_reverse_egress' and event=='return':
            reasons.append({'line':frame.f_lineno,'start_clearance_m':frame.f_locals.get('start_clearance'),'required_m':frame.f_locals.get('required'),'returned_none':arg is None})
        return trace
    sys.settrace(trace)
    try:result=select_reverse_egress(history,raw,inflated,geometry,oracle,now=data['probe']['monotonic'],robot_radius=.22,clearance_margin=.01,max_history_age=90.,max_sample_gap=30.,current_pose=pose,verified_command_corridor=True)
    finally:sys.settrace(None)
    closest=[]
    for x,y in observed|memory:
        lx,ly=-1.65+x*.05,-5.25+y*.05
        gap=math.hypot(max(lx-pose[0],pose[0]-lx-.05,0.),max(ly-pose[1],pose[1]-ly-.05,0.))
        closest.append({'cell':[x,y],'distance_m':gap,'observed':(x,y) in observed,'remembered':(x,y) in memory})
    report={'scope':__doc__,'counts':data['counts'],'first_return':reasons,'candidate':str(result),'goal_oracle_calls':calls,'closest_raw_cells':sorted(closest,key=lambda c:c['distance_m'])[:6],
      'independent_chord_screen':data['straight_chord_candidates'],'source_sha256':{str(x.relative_to(ROOT)):hashlib.sha256(x.read_bytes()).hexdigest() for x in (source,image,ROOT/'ros2_ws/src/smart_vision_amr/smart_vision_amr/reverse_recovery.py')},
      'limitations':['Cell/source reconstruction matches actual24observed/11memory/422conservative mask; no ROS/node/drive started.','First return is current raw occupied square clearance, before historical polyline, monotonicity, gain or goal-oracle evaluation.']}
    args.output.write_text(json.dumps(report,indent=2));print(json.dumps({'first_return':reasons,'goal_calls':len(calls),'candidate':str(result)}))
if __name__=='__main__':main()
