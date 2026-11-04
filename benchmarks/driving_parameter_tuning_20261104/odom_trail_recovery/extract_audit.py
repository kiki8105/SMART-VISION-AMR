import json,math,bisect,argparse
from pathlib import Path
parser=argparse.ArgumentParser(description='Extract actual odom/TF diagnostics without ROS')
parser.add_argument('--timeline',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
if args.output.exists():parser.error('Refusing to overwrite an audit')
p=args.timeline
rows=[json.loads(l) for l in p.open()]
od=[r for r in rows if r['kind']=='odom'];mos=[r for r in rows if r['kind']=='tf_stamp' and r['parent']=='map' and r['child']=='odom']
probes=[];last=None
for r in rows:
 d=r.get('details',{}).get('reverse_probe_diagnostics')
 if d and d!=last:probes.append((r,d));last=d
out={'input':str(p),'scope':'Recorded odometry projection only; not raw collision/goal reachability proof','odom_count':len(od),'map_odom_count':len(mos),'probes':[]}
for row,diag in probes:
 t=row['monotonic'];hist=[q for q in od if t-90<=q['monotonic']<=t];current=hist[-1];cp=diag['current_pose']
 rot=cp[2]-current['yaw'];c,s=math.cos(rot),math.sin(rot)
 tx=cp[0]-c*current['x']+s*current['y'];ty=cp[1]-s*current['x']-c*current['y']
 good=[];behind=[];first_reject=None
 for q in reversed(hist[:-1]):
  dx,dy=q['x']-current['x'],q['y']-current['y'];back=-dx*math.cos(current['yaw'])-dy*math.sin(current['yaw']);lat=abs(dx*math.sin(current['yaw'])-dy*math.cos(current['yaw']));ang=(q['yaw']-current['yaw']+math.pi)%(2*math.pi)-math.pi
  if .095<=back<=.305:
   v={'t':q['t'],'age_s':t-q['monotonic'],'backward_m':back,'lateral_m':lat,'yaw_delta_rad':ang,'odom':[q['x'],q['y'],q['yaw']],'map_current_projection':[tx+c*q['x']-s*q['y'],ty+s*q['x']+c*q['y'],rot+q['yaw']]}
   behind.append(v)
   if lat<=.03 and abs(ang)<=.15:good.append(v)
 recent_mos=[q for q in mos if t-5<=q['monotonic']<=t];corrections=[]
 for a,b in zip(recent_mos,recent_mos[1:]):
  dp=math.hypot(b['x']-a['x'],b['y']-a['y']);da=(b['yaw']-a['yaw']+math.pi)%(2*math.pi)-math.pi
  if dp>.008 or abs(da)>.02:corrections.append({'t':b['t'],'source_s':b['stamp_ns']/1e9,'shift_m':dp,'yaw_shift_rad':da})
 item={'probe_t':row['t'],'recorded_trail_count':diag['trail_count'],'recorded_trail':diag['recent_trail'],'current_map_pose':cp,'current_odom':current,'current_frame_transform':[tx,ty,rot],'good_axis_candidates_count':len(good),'nearest_good_axis_candidate':good[0] if good else None,'nearest_10cm_odom_pose':behind[0] if behind else None,'map_to_odom_corrections_last5s':sorted(corrections,key=lambda x:x['shift_m'],reverse=True)[:5]}
 out['probes'].append(item)
 print(json.dumps(item))
args.output.write_text(json.dumps(out,indent=2))
