"""Two bounded pure planner queries; no ROS init, commands, or runtime edits."""
import json,math,heapq,time,hashlib,sys,argparse
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'ros2_ws/src/smart_vision_amr'))
from smart_vision_amr.grid import Grid
from smart_vision_amr.map_utils import inflate_cells,inflate_cells_circular
from smart_vision_amr.continuous_start_egress import certified_start_only_edges
from geometry_tools import load_saved_map,polyline_gap

def _segment_rectangle_distance(a,b,lower,upper):
 return polyline_gap([a,b],lower,upper,0.,(-1e6,-1e6,1e6,1e6))

def forbidden_start_edges(start,allowed):
 forbidden={(start,(start[0]+dx,start[1]+dy)) for dx in (-1,0,1) for dy in (-1,0,1)
            if (dx or dy) and (start,(start[0]+dx,start[1]+dy)) not in allowed}
 forbidden.update(((start[0]+dx,start[1]+dy),start) for dx in (-1,0,1) for dy in (-1,0,1) if dx or dy)
 for dx in (-1,1):
  for dy in (-1,1):
   a,b=(start[0]+dx,start[1]),(start[0],start[1]+dy)
   forbidden.update(((a,b),(b,a)))
 return forbidden

parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
if args.output.exists(): parser.error('Output already exists; use a new path')
PREP_STARTED=time.perf_counter()
fixture=ROOT/'benchmarks/driving_parameter_tuning_20261104/physical_transfer_20261105/common_coherent_v2_geometry_tracking_inputs.json'
f=json.loads(fixture.read_text());snap=f['snapshot'];d=snap['planner']['details'];cert=d['start_certificate_diagnostics'];pose=tuple(cert['fresh_pose'][:2]);goal=(-.25,-1.98)
obs={tuple(c) for c in snap['observed_snapshot_cells']};mem={tuple(c) for c in snap['remembered_snapshot_cells']}
m,a=load_saved_map(ROOT/'ros2_ws/src/smart_vision_amr/maps/classroom_20261030_cleaned.yaml')
w,h,res=m['width'],m['height'],m['resolution'];ox,oy=m['origin']['x'],m['origin']['y'];radius=.22
static={(x,y) for y in range(h) for x in range(w) if a[y,x]<0 or a[y,x]>=65};allraw=static|obs|mem
raw=Grid(tuple(tuple((x,y) in static for x in range(w)) for y in range(h)))
static_mask=inflate_cells(static,w,h,5);grid=Grid(tuple(tuple((x,y) in static_mask for x in range(w)) for y in range(h)))
# Exact point-to-rectangle distance on a 5mm lattice: raw square boundaries,
# cardinal projections, and rectangle corners all belong to this lattice.
unit=res/10; occupied=np.zeros((h*10+1,w*10+1),dtype=bool)
for x,y in allraw:occupied[y*10:y*10+11,x*10:x*10+11]=True
clearance=distance_transform_edt(~occupied,sampling=unit)
yy,xx=np.indices(clearance.shape);clearance=np.minimum(clearance,np.minimum(np.minimum(xx,w*10-xx),np.minimum(yy,h*10-yy))*unit)
lo=np.array(sorted(allraw))*res+[ox,oy];hi=lo+res

def exact_gap(points):
 return min(_segment_rectangle_distance(p,q,lo,hi) for p,q in zip(points,points[1:]))-radius

def search(mode):
 start_t=time.perf_counter();fine=mode=='B';spacing=res/5 if fine else res
 width,height=(w*5,h*5) if fine else (w,h)
 to_world=lambda p:(ox+(p[0]+.5)*spacing,oy+(p[1]+.5)*spacing)
 to_cell=lambda p:(max(0,min(width-1,round((p[0]-ox)/spacing-.5))),max(0,min(height-1,round((p[1]-oy)/spacing-.5))))
 start=to_cell(pose) if fine else tuple(cert['start']);end=to_cell(goal)
 margin=.02 if fine else 0.
 # Fine candidate centers are odd lattice indices; coarse centers are5mod10.
 scale=2 if fine else 10;offset=1 if fine else 5
 point_clear=lambda p:float(clearance[p[1]*scale+offset,p[0]*scale+offset])
 dynamic=inflate_cells_circular(obs|mem,w,h,radius/res) if not fine else set()
 allowed=(certified_start_only_edges(raw,grid,obs|mem,dynamic,start,((pose[0]-ox)/res,(pose[1]-oy)/res),radius/res,0.) if not fine else set())
 forbidden=forbidden_start_edges(start,allowed) if not fine else set()
 def blocked(p):
  if not 0<=p[0]<width or not 0<=p[1]<height:return True
  if fine:return point_clear(p)<radius+margin-1e-10
  return (p in static_mask or p in dynamic) and p!=start
 def swept(p,q):
  x0,y0=p[0]*scale+offset,p[1]*scale+offset;x1,y1=q[0]*scale+offset,q[1]*scale+offset
  steps=max(abs(x1-x0),abs(y1-y0));dx=(x1-x0)//steps;dy=(y1-y0)//steps
  c=min(float(clearance[y0+i*dy,x0+i*dx]) for i in range(steps+1))
  # Independent Lipschitz reserve covers all points between 5mm samples.
  reserve=math.hypot(dx,dy)*unit/2
  return c-reserve>=radius+margin-1e-10
 def heuristic(p):return math.hypot(p[0]-end[0],p[1]-end[1])
 if blocked(start) or blocked(end):return {'feasible':False,'reason':'blocked_endpoint','start':start,'goal':end}
 queue=[(heuristic(start),0.,start)];best={start:0.};came={};exp=0
 while queue:
  _,g,p=heapq.heappop(queue)
  if g!=best.get(p):continue
  exp+=1
  if p==end:
   cells=[p]
   while p in came:p=came[p];cells.append(p)
   cells.reverse();points=[pose]+[to_world(p) for p in cells]+[goal]
   length=sum(math.dist(p,q) for p,q in zip(points,points[1:]))
   gap=exact_gap(points)
   return {'feasible':gap>=margin-1e-9,'searched_route':True,'margin_target_m':margin,'nodes':len(cells),'expansions':exp,'length_m':length,'exact_whole_path_body_gap_m':gap,'duration_s':time.perf_counter()-start_t,'first_points':points[:8],'world_points':points,'start':start,'goal':end,'allowed_start_edges':sorted(allowed),'minimum_node_body_gap_m':min(point_clear(p)-radius for p in cells)}
  for dx,dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,-1),(-1,1),(1,1)]:
   q=(p[0]+dx,p[1]+dy)
   # Same bounded physical room/goal region for both queries.
   wx,wy=to_world(q)
   if not -.8<=wx<=1.1 or not -2.8<=wy<=-.55:continue
   if blocked(q) or (p,q) in forbidden:continue
   if dx and dy and (blocked((p[0]+dx,p[1])) or blocked((p[0],p[1]+dy))):continue
   if not swept(p,q):continue
   penalty=1.5*max(0.,(.45-point_clear(q))/.45)
   cost=g+math.hypot(dx,dy)*(1.+penalty)
   if cost<best.get(q,math.inf):best[q]=cost;came[q]=p;heapq.heappush(queue,(cost+heuristic(q),cost,q))
 return {'feasible':False,'reason':'no_path','expansions':exp,'duration_s':time.perf_counter()-start_t,'allowed_start_edges':sorted(allowed)}
PREPARATION_DURATION=time.perf_counter()-PREP_STARTED
report={'preparation_duration_s':PREPARATION_DURATION,'search_duration_note':'Per-case duration includes search and final exact validation; excludes shared input/field construction reported separately.', 'scope':'OFFLINE two bounded alternatives only; no closed-loop/physical pass','fixture':str(fixture.relative_to(ROOT)),'fixture_sha256':hashlib.sha256(fixture.read_bytes()).hexdigest(),'map_sha256':hashlib.sha256((ROOT/f['map_pgm']).read_bytes()).hexdigest(),'radius_m':radius,'occupied_square_geometry_unchanged':True,'current_pose':pose,'exact_goal':goal,'rectangle':[-.8,-2.8,1.1,-.55],'A':search('A'),'B':search('B')}
with args.output.open('x') as stream: stream.write(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:({kk:vv for kk,vv in v.items() if kk!='world_points'} if k in ['A','B'] else v) for k,v in report.items()},ensure_ascii=False))
