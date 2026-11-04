#!/usr/bin/env python3
"""Installed Nav2 footprint collision checker on immutable captured inputs.

No ROS node, executor, Gazebo or velocity publisher is started. The fine-grid
control keeps every old occupied world square. The isolated re-marking changes
only the raster footprint of one identified raw endpoint, which is retained.
"""
import argparse,gzip,hashlib,json,math,subprocess,tempfile
from pathlib import Path

HERE=Path(__file__).resolve().parent

def raw(v):
    return 255 if v<0 else 254 if v==100 else 253 if v==99 else 0 if v==0 else 1+round((v-1)*251/97)

def square_distance(x,y,left,bottom,res):
    return math.hypot(max(left-x,x-left-res,0.),max(bottom-y,y-bottom-res,0.))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--ros-prefix',type=Path,default=Path('/opt/ros/humble'))
    args=p.parse_args()
    if args.output.exists():p.error('Refusing to overwrite a result')
    source=HERE/'fixture.json.gz';fixture=json.loads(gzip.decompress(source.read_bytes()))
    cm=fixture['costmap']['data'];info=cm['info'];w,h=info['width'],info['height'];res=info['resolution'];origin=info['origin']['position'];ox,oy=origin['x'],origin['y']
    base=fixture['current_base_tf'];x,y,a=base['x'],base['y'],base['yaw'];point=fixture['retained_endpoint']['odom_xy']
    cx,cy=int((point[0]-ox)/res),int((point[1]-oy)/res);values=[raw(v) for v in cm['data']]
    assert values[cy*w+cx]==254
    poses=[(x-i*.001*math.cos(a),y-i*.001*math.sin(a),a) for i in range(108)]
    fine=[value for row in range(h) for twice_y in range(2) for col in range(w) for value in [values[row*w+col]]*2]
    remark=list(fine)
    for yy in range(2*cy,2*cy+2):
        for xx in range(2*cx,2*cx+2):remark[yy*(2*w)+xx]=0
    px,py=int((point[0]-ox)/(res/2)),int((point[1]-oy)/(res/2));remark[py*(2*w)+px]=254
    assert (2*cx<=px<2*cx+2 and 2*cy<=py<2*cy+2)
    cases=[('captured_5cm',w,h,res,values),('2p5cm_same_world_areas',w*2,h*2,res/2,fine),('2p5cm_identified_point_remarked',w*2,h*2,res/2,remark)]
    with tempfile.TemporaryDirectory(prefix='nav2-backup-collision-') as build:
        binary=Path(build)/'footprint_probe'
        prefix=args.ros_prefix
        cmd=['g++','-std=c++17','-O0',str(HERE/'footprint_probe.cpp'),'-o',str(binary),'-I'+str(prefix/'include')]
        cmd += ['-I'+str(d) for d in (prefix/'include').iterdir() if d.is_dir() and d.name not in ('dds','idl')]
        cmd += ['-I/usr/include/eigen3','-L'+str(prefix/'lib'),'-Wl,-rpath,'+str(prefix/'lib'),'-lnav2_costmap_2d_core']
        subprocess.run(cmd,check=True,capture_output=True,text=True)
        results=[]
        for name,width,height,spacing,cost in cases:
            body=fixture['local_footprint']
            text=' '.join(map(str,[width,height,spacing,ox,oy,*cost,len(body),*[v for pt in body for v in pt],len(poses),*[v for pose in poses for v in pose]]))
            run=subprocess.run([str(binary)],input=text,capture_output=True,text=True,check=True)
            scores=[float(line.split()[1]) for line in run.stdout.splitlines()]
            hits=[list(map(int,line.split())) for line in run.stderr.splitlines()]
            first=next((i for i,s in enumerate(scores) if s>=254),None)
            actual_steps=[i for i in range(len(scores)) if i%10==0]
            results.append({'case':name,'resolution':spacing,'score_at_current':scores[0],'first_rejected_distance_m':None if first is None else first*.001,
              'actual_10hz_command_samples':{str(i*.001):scores[i] for i in actual_steps},'scores_1mm':scores,'lethal_raster_hits':hits})
    old_bounds=[ox+cx*res,oy+cy*res,res];new_bounds=[ox+px*res/2,oy+py*res/2,res/2]
    result={'scope':__doc__,'warning_wall':fixture['warning_wall'],'costmap_received_before_warning_s':fixture['warning_wall']-fixture['costmap']['received_wall_time_ns']/1e9,
      'body_radius_m':.22,'old_lethal_cell':[cx,cy],'old_cell_bounds':[old_bounds[0],old_bounds[1],old_bounds[0]+res,old_bounds[1]+res],
      'retained_endpoint':fixture['retained_endpoint'],'new_point_cell_bounds':[new_bounds[0],new_bounds[1],new_bounds[0]+res/2,new_bounds[1]+res/2],
      'minimum_full_circle_body_gap_old_cell_m':min(square_distance(px_,py_,*old_bounds)-.22 for px_,py_,_ in poses),
      'minimum_full_circle_body_gap_new_point_cell_m':min(square_distance(px_,py_,*new_bounds)-.22 for px_,py_,_ in poses),
      'results':results,'limitations':fixture['limitations'],
      'sha256':{q.name:hashlib.sha256(q.read_bytes()).hexdigest() for q in (source,HERE/'footprint_probe.cpp',args.ros_prefix/'lib/libnav2_costmap_2d_core.so')}}
    args.output.write_text(json.dumps(result,indent=2))
    print(json.dumps({r['case']:r['first_rejected_distance_m'] for r in results}))

if __name__=='__main__':main()
