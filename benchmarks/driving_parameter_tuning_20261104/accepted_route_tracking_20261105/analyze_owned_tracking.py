import argparse,json,math,bisect,hashlib
from pathlib import Path

CASES={'v2':Path('/tmp/classroom-recovery-replaced-v2-20261104/25_web_inventory_sector3_moving_obstacle'),
       'local025':Path('/tmp/classroom-recovery-local025-20261105/25_web_inventory_sector3_moving_obstacle')}

parser=argparse.ArgumentParser(description='Compare only acknowledged FollowPath intervals with recorded actual motion.')
parser.add_argument('--trial', action='append', metavar='NAME=STAGE_DIRECTORY')
parser.add_argument('--output', type=Path, default=Path('/tmp/owned_follow_tracking_comparison_20261105.json'))
parser.add_argument('--window', type=float, default=40., help='Seconds from timeline start; default preserves original first40s audit')
args=parser.parse_args()
if not math.isfinite(args.window) or args.window<=0:parser.error('--window must be finite and positive')
if args.output.exists():parser.error('Output already exists; use a new path')
if args.trial:
    CASES={}
    for item in args.trial:
        name,sep,path=item.partition('=')
        if not sep or not name or not path or name in CASES:parser.error('--trial requires unique NAME=STAGE_DIRECTORY')
        CASES[name]=Path(path).expanduser()

def load(path):
    if not path.exists(): path=Path(str(path.parent)+'.safety_timeline.jsonl')
    raw=path.read_bytes();return path,[json.loads(l) for l in raw.splitlines() if l.strip()],hashlib.sha256(raw).hexdigest()

def nearest(points,p):
    result=None;progress=0
    for i,(a,b) in enumerate(zip(points,points[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        if not length:continue
        u=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length**2))
        q=(a[0]+u*dx,a[1]+u*dy);dist=math.dist(p,q)
        record={'distance_m':dist,'signed_m':(dx*(p[1]-q[1])-dy*(p[0]-q[0]))/length,
                'segment':i,'progress_m':progress+u*length,'projection':q}
        if result is None or dist<result['distance_m']:result=record
        progress+=length
    return result

def box_gap(p,box):
    x,y=box;return math.hypot(max(abs(p[0]-x)-.125,0),max(abs(p[1]-y)-.125,0))-.22

def atlast(rows,at):
    i=bisect.bisect_right([r['wall_time'] for r in rows],at)-1
    return rows[i] if i>=0 else None

def stat(vs):
    vs=sorted(vs)
    return {'n':len(vs),'min':vs[0] if vs else None,'p50':vs[len(vs)//2] if vs else None,
            'p95':vs[min(len(vs)-1,int(len(vs)*.95))] if vs else None,'max':vs[-1] if vs else None}

report={'scope':'Offline acknowledged FollowPath intervals; not a closed-loop pass. Latest receipt-bounded TF composition estimates map pose; Gazebo truth is independent. Map path points rounded to0.1mm by recorder.','window_seconds':args.window,'cases':{}}
for label,folder in CASES.items():
    path,rows,sha=load(folder/'safety_timeline.jsonl')
    injection=json.loads((folder/'web_obstacle_injection.json').read_text())
    events=injection['events'];spawn=next(v for v in events if v['event']=='obstacle_spawned');removed=next(v for v in events if v['event']=='obstacle_removed')
    moves=[dict(at=v['at'],x=v['x'],y=v['y'],step=v.get('step',0)) for v in events if v['event'] in ('obstacle_spawned','obstacle_moved')]
    paths={v['stamp_ns']:v for v in rows if v['kind']=='path' and v['count']>1}
    maprows=[r for r in rows if r['kind']=='tf_stamp' and r['parent']=='map' and r['child']=='odom']
    odomrows=[r for r in rows if r['kind']=='tf_stamp' and r['parent']=='odom' and r['child']=='base_footprint']
    owned=None;witness=[];intervals=[];active_start=None;last_mode=None
    for v in rows:
        if v['t']>args.window:break
        if v['kind']=='path' and v['count']==0:
            if owned and active_start is not None:intervals.append({'stamp_ns':owned,'start_t':active_start,'end_t':v['t'],'ended_by':'empty_path'})
            owned=None;active_start=None
        elif v['kind']=='navigation':
            details=v.get('details') or {};stamp=details.get('path_stamp')
            ns=stamp.get('sec',0)*10**9+stamp.get('nanosec',0) if isinstance(stamp,dict) else None
            follow_ack=(v.get('status')=='MOVING' and v.get('message') in (
                'Following A* route', 'Accepted equivalent remaining route without restarting FollowPath') and ns in paths)
            if follow_ack:
                if owned!=ns:
                    if owned and active_start is not None:intervals.append({'stamp_ns':owned,'start_t':active_start,'end_t':v['t'],'ended_by':'new_owned_ack'})
                    owned=ns;active_start=v['t']
            elif (v.get('status') in ('FAILED','SUCCEEDED','CANCELED','WAIT_REPLAN','WAIT_CANCEL','RECOVERING')
                  or details.get('heading_alignment_phase') in ('WAIT_STOP','SPIN')
                  or (v.get('status')=='MOVING' and 'Rotating in place' in str(v.get('message','')))):
                if owned and active_start is not None:intervals.append({'stamp_ns':owned,'start_t':active_start,'end_t':v['t'],'ended_by':v.get('status')})
                owned=None;active_start=None
        elif v['kind']=='truth' and owned is not None:
            if not spawn['at']<=v['wall_time']<removed['at']:continue
            actor=max((m for m in moves if m['at']<=v['wall_time']),key=lambda m:m['at'])
            t=nearest(paths[owned]['points'],(v['x'],v['y']))
            m,o=atlast(maprows,v['wall_time']),atlast(odomrows,v['wall_time'])
            if m is None or o is None:continue
            ma,oa=v['wall_time']-m['wall_time'],v['wall_time']-o['wall_time']
            if not(0<=ma<=.25 and 0<=oa<=.10):continue
            c,s=math.cos(m['yaw']),math.sin(m['yaw'])
            estimate=(m['x']+c*o['x']-s*o['y'],m['y']+s*o['x']+c*o['y'])
            e=nearest(paths[owned]['points'],estimate)
            box=(actor['x'],actor['y']);q=t['projection'];vector=(box[0]-q[0],box[1]-q[1]);d=math.hypot(*vector)
            toward=((v['x']-q[0])*vector[0]+(v['y']-q[1])*vector[1])/d if d else 0
            pmin=min(box_gap((a[0]+(b[0]-a[0])*i/max(1,int(math.dist(a,b)/.001)+1),a[1]+(b[1]-a[1])*i/max(1,int(math.dist(a,b)/.001)+1)),box)
                     for a,b in zip(paths[owned]['points'],paths[owned]['points'][1:]) for i in range(max(1,int(math.dist(a,b)/.001)+1)+1))
            witness.append({'t':v['t'],'wall_time':v['wall_time'],'path_stamp_ns':owned,'box_step':actor['step'],'box_xy':box,
                'truth_xy':[v['x'],v['y']],'estimated_xy':estimate,'map_tf_age_s':ma,'odom_tf_age_s':oa,
                'truth_cross_track':t,'estimated_cross_track':e,'truth_estimate_error_m':math.dist((v['x'],v['y']),estimate),
                'truth_body_box_gap_m':box_gap((v['x'],v['y']),box),'estimated_body_box_gap_m':box_gap(estimate,box),
                'projection_body_box_gap_m':box_gap(q,box),'whole_published_path_box_gap_sampled_1mm_m':pmin,'truth_toward_box_deviation_m':toward})
    summaries=[]
    for ns in dict.fromkeys(w['path_stamp_ns'] for w in witness):
        values=[w for w in witness if w['path_stamp_ns']==ns]
        summaries.append({'stamp_ns':ns,'published_t':paths[ns]['t'],'count':paths[ns]['count'],'first_xy':paths[ns]['points'][0],
            'intervals':[v for v in intervals if v['stamp_ns']==ns],'truth_cross_track':stat([w['truth_cross_track']['distance_m'] for w in values]),
            'estimated_cross_track':stat([w['estimated_cross_track']['distance_m'] for w in values]),
            'truth_estimate_error':stat([w['truth_estimate_error_m'] for w in values]),'truth_box_gap':stat([w['truth_body_box_gap_m'] for w in values]),
            'min_gap_witness':min(values,key=lambda w:w['truth_body_box_gap_m']),
            'max_cross_witness':max(values,key=lambda w:w['truth_cross_track']['distance_m'])})
    report['cases'][label]={'timeline':str(path),'timeline_sha256':sha,'injection':str(folder/'web_obstacle_injection.json'),
        'source_radii_m':{'robot':.22,'box_half_x':.125,'box_half_y':.125},'actor_moves':moves,'accepted_intervals':intervals,
        'aggregate_truth_cross_track':stat([w['truth_cross_track']['distance_m'] for w in witness]),
        'aggregate_estimated_cross_track':stat([w['estimated_cross_track']['distance_m'] for w in witness]),
        'aggregate_estimate_error':stat([w['truth_estimate_error_m'] for w in witness]),'per_path':summaries,'witness_samples':witness}
out=args.output;out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2))
print(out)
for name,data in report['cases'].items():
 print(name,'all',data['aggregate_truth_cross_track'],data['aggregate_estimated_cross_track'],data['aggregate_estimate_error'])
 for v in data['per_path']:
  w=v['min_gap_witness'];print('path',v['stamp_ns'],'n',v['truth_cross_track']['n'],'truth_xte',v['truth_cross_track'],'est_xte',v['estimated_cross_track'],'minGap',w['truth_body_box_gap_m'],'pathFullGap',w['whole_published_path_box_gap_sampled_1mm_m'],'projectionGap',w['projection_body_box_gap_m'],'estErr',w['truth_estimate_error_m'],'witness_t',w['t'])
