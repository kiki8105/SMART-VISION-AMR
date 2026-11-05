import cProfile,pstats,math,time,json,statistics
import numpy as np
from smart_vision_amr.braking_envelope import BrakingEnvelope
from smart_vision_amr.scan_outlier_guard import swept_body_decision,rebase_scan_points
points=tuple((i,.6*math.cos(i*math.tau/360),.6*math.sin(i*math.tau/360)) for i in range(360))
points=np.asarray(points,dtype=float)
cloud=np.concatenate((points,points))
b=BrakingEnvelope(reaction_seconds=.25)
u=dict(longitudinal_uncertainty=.025,lateral_uncertainty=.002,yaw_uncertainty=.1)
rows=[]
def run():
 for i in range(30):
  v,w=.18+i*1.e-6,.013+i*1.e-6
  u['longitudinal_uncertainty']=.025+i*1.e-6
  t=time.perf_counter(); b.limit(cloud,v,w,**u);a=time.perf_counter();b.check_feedback(cloud,.08,.013,linear_delta=.02,angular_delta=.10,**u);c=time.perf_counter()
  for vv,ww in [(v,w),(.04,.02),(.03,.013)]:
   for p in [points,points]: swept_body_decision(p,vv,ww,**u)
  d=time.perf_counter();b.limit(cloud,.03,.013+i*1.e-6,**u);e=time.perf_counter()
  rows.append([a-t,c-a,d-c,e-d])
p=cProfile.Profile();p.runcall(run);pstats.Stats(p).strip_dirs().sort_stats('cumtime').print_stats(15)
print(json.dumps({name:{'median_ms':statistics.median(row[i] for row in rows)*1000,'max_ms':max(row[i] for row in rows)*1000} for i,name in enumerate(['requested_brake','measured_brake','raw_sweeps','final_brake'])}))
