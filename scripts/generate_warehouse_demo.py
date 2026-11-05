#!/usr/bin/python3
"""Generate an aligned warehouse world/map, ten rear-camera goals and traffic.

The PGM is a conservative raster of the exact static collision rectangles.
Moving actors are deliberately absent from the static map. This is a synthetic
simulation map, not SLAM evidence or proof of closed-loop navigation.
"""
import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import yaml

from generate_aruco_models import create_model, save_xml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = Path('ros2_ws/src/smart_vision_amr')
RESOLUTION = .05
ORIGIN = (-5., -27.5, 0.)
WIDTH, HEIGHT = 200, 610
RADIUS = .22
MARKER_SIZE = .15
CAMERA_HEIGHT = .15
CAMERA_REAR_OFFSET = .08
CAMERA_HFOV = 1.085595


def element(parent, tag, text=None, **attrs):
    value = ET.SubElement(parent, tag, attrs)
    if text is not None:
        value.text = str(text)
    return value


def pose_text(pose):
    return ' '.join(f'{value:.9f}' for value in pose)


def marker_model(identifier):
    return f'smart_vision_warehouse_aruco_4x4_50_id_{identifier}'


def layout():
    fixed = [
        dict(name='warehouse_wall_west', x_min=-5., x_max=-4.85, y_min=-27.5, y_max=3., height=2.8),
        dict(name='warehouse_wall_east', x_min=4.85, x_max=5., y_min=-27.5, y_max=3., height=2.8),
        dict(name='warehouse_wall_south', x_min=-4.85, x_max=4.85, y_min=-27.5, y_max=-27.35, height=2.8),
        dict(name='warehouse_wall_north', x_min=-4.85, x_max=4.85, y_min=2.85, y_max=3., height=2.8),
        dict(name='warehouse_home_dock', x_min=2.45, x_max=3.15, y_min=2.35, y_max=2.85, height=1.1),
    ]
    markers = [dict(id=0, name='HOME', x=2.8, y=1., yaw=-math.pi/2, camera='rear')]
    boards = [dict(id=0, pose=[2.8, 2.352, CAMERA_HEIGHT, 0., 0., math.pi/2])]
    for identifier in range(1, 11):
        y = -2.5*identifier
        fixed.append(dict(name=f'warehouse_rack_{identifier:02d}', x_min=-4., x_max=.8,
                          y_min=y-.45, y_max=y+.45, height=2.4))
        markers.append(dict(id=identifier, name=f'Sector {identifier}', x=1.7, y=y, yaw=0., camera='rear'))
        # Board collision lies inside the plinth; the printed face is 0.1mm
        # outside x=.8, avoiding a competing coplanar textured surface.
        boards.append(dict(id=identifier, pose=[.798, y, CAMERA_HEIGHT, 0., 0., math.pi]))
    actors = [
        dict(model_name='warehouse_cart_1', collision_radius_m=.375, speed_mps=.12,
             dwell_seconds=2., loop='ping_pong', max_step_m=.03,
             waypoints=[dict(x=1.3, y=-6.25, z=0., yaw=0.), dict(x=3.6, y=-6.25, z=0., yaw=0.)]),
        dict(model_name='warehouse_cart_2', collision_radius_m=.375, speed_mps=.10,
             dwell_seconds=2.5, loop='ping_pong', max_step_m=.03,
             waypoints=[dict(x=3.4, y=-10.5, z=0., yaw=math.pi/2),
                        dict(x=3.4, y=-15.5, z=0., yaw=math.pi/2)]),
        dict(model_name='warehouse_pedestrian_1', collision_radius_m=.22, speed_mps=.10,
             dwell_seconds=3., loop='ping_pong', max_step_m=.025,
             waypoints=[dict(x=1.5, y=-21.25, z=0., yaw=0.), dict(x=3.7, y=-21.25, z=0., yaw=0.)]),
    ]
    return fixed, markers, boards, actors


def box_shape(link, name, size, pose, color, *, collision=True):
    for kind in (('collision', 'visual') if collision else ('visual',)):
        shape = element(link, kind, name=name+'_'+kind)
        element(shape, 'pose', pose_text(pose))
        box = element(element(shape, 'geometry'), 'box')
        element(box, 'size', pose_text(size))
        if kind == 'visual':
            material = element(shape, 'material')
            element(material, 'ambient', color)
            element(material, 'diffuse', color)


def cylinder_shape(link, name, radius, length, pose, color, *, collision=False):
    for kind in (('collision', 'visual') if collision else ('visual',)):
        shape = element(link, kind, name=name+'_'+kind)
        element(shape, 'pose', pose_text(pose))
        geometry = element(element(shape, 'geometry'), 'cylinder')
        element(geometry, 'radius', radius)
        element(geometry, 'length', length)
        if kind == 'visual':
            material = element(shape, 'material')
            element(material, 'ambient', color)
            element(material, 'diffuse', color)


def build_world(fixed, boards, actors):
    root = ET.Element('sdf', version='1.6')
    world = element(root, 'world', name='warehouse_logistics')
    element(world, 'gravity', '0 0 -9.81')
    physics = element(world, 'physics', name='warehouse_physics', default='true', type='ode')
    element(physics, 'max_step_size', '.002')
    element(physics, 'real_time_factor', '1')
    element(physics, 'real_time_update_rate', '500')
    scene = element(world, 'scene')
    element(scene, 'ambient', '.65 .65 .65 1')
    element(scene, 'background', '.80 .84 .88 1')
    element(scene, 'shadows', 'false')
    light = element(world, 'light', name='warehouse_sun', type='directional')
    element(light, 'cast_shadows', 'false')
    element(light, 'diffuse', '.85 .85 .85 1')
    element(light, 'specular', '.1 .1 .1 1')
    element(light, 'direction', '-.3 -.4 -1')
    gui = element(world, 'gui', fullscreen='0')
    camera = element(gui, 'camera', name='user_camera')
    # Aim at the warehouse centre (0,-12.25,0), not the world origin.
    element(camera, 'pose', '17 1 24 0 .839043 -2.479525')
    element(camera, 'view_controller', 'orbit')
    # The state plugin exposes the actor's actual physics models and truth.
    plugin = element(world, 'plugin', name='gazebo_ros_state', filename='libgazebo_ros_state.so')
    ros = element(plugin, 'ros'); element(ros, 'namespace', '/gazebo')
    element(plugin, 'update_rate', '20')
    floor = element(world, 'model', name='warehouse_ground')
    element(floor, 'static', 'true')
    link = element(floor, 'link', name='floor')
    body = element(link, 'collision', name='floor_collision')
    plane = element(element(body, 'geometry'), 'plane')
    element(plane, 'normal', '0 0 1'); element(plane, 'size', '40 70')
    include = element(world, 'include')
    element(include, 'uri', 'model://warehouse_floor_surface')
    element(include, 'pose', '0 -12.25 .002 0 0 0')
    for rectangle in fixed:
        name = rectangle['name']
        model = element(world, 'model', name=name)
        element(model, 'static', 'true')
        x = (rectangle['x_min']+rectangle['x_max'])/2
        y = (rectangle['y_min']+rectangle['y_max'])/2
        width = rectangle['x_max']-rectangle['x_min']
        depth = rectangle['y_max']-rectangle['y_min']
        element(model, 'pose', pose_text((x,y,0,0,0,0)))
        link = element(model, 'link', name='structure')
        rack = name.startswith('warehouse_rack_')
        # A solid low plinth agrees with the map at every laser height used
        # by the robot (.12-.17m), while upper levels still look like racks.
        solid_height = .45 if rack else rectangle['height']
        box_shape(link, 'mapped_solid', (width,depth,solid_height),
                  (0,0,solid_height/2,0,0,0), '.27 .31 .35 1' if rack else '.64 .67 .70 1')
        if rack:
            for ix, xx in enumerate((-width/2+.06, 0., width/2-.06)):
                for iy, yy in enumerate((-depth/2+.045, depth/2-.045)):
                    box_shape(link, f'upright_{ix}_{iy}', (.075,.075,2.4), (xx,yy,1.2,0,0,0),
                              '.16 .28 .40 1', collision=False)
            for level, zz in enumerate((.50,1.30,2.10)):
                box_shape(link, f'shelf_{level}', (width,.87,.065), (0,0,zz,0,0,0), '.92 .36 .10 1', collision=False)
                for pallet in range(5):
                    px = -width/2+.49+pallet*.95
                    box_shape(link, f'pallet_{level}_{pallet}', (.82,.75,.08), (px,0,zz+.075,0,0,0),
                              '.55 .39 .22 1', collision=False)
                    box_shape(link, f'load_{level}_{pallet}', (.73,.66,.48), (px,0,zz+.355,0,0,0),
                              ('.67 .56 .40 1' if (level+pallet)%2 else '.74 .65 .49 1'), collision=False)
        elif name == 'warehouse_home_dock':
            box_shape(link, 'charging_panel', (.55,.04,.35), (0,-depth/2-.001,.78,0,0,0),
                      '.12 .20 .26 1', collision=False)
    for board in boards:
        include = element(world, 'include')
        element(include, 'uri', 'model://'+marker_model(board['id']))
        element(include, 'name', f'warehouse_marker_{board["id"]:02d}')
        element(include, 'pose', pose_text(board['pose']))
    for actor in actors:
        start = actor['waypoints'][0]
        model = element(world, 'model', name=actor['model_name'])
        element(model, 'static', 'false')
        element(model, 'pose', pose_text((start['x'],start['y'],start['z'],0,0,start['yaw'])))
        link = element(model, 'link', name='body')
        element(link, 'kinematic', 'true'); element(link, 'gravity', 'false')
        inertia = element(link, 'inertial'); element(inertia, 'mass', '30')
        tensor = element(inertia, 'inertia')
        for axis in ('ixx','iyy','izz'): element(tensor, axis, '1')
        if 'cart' in actor['model_name']:
            box_shape(link, 'cart_body', (.60,.45,.65), (0,0,.40,0,0,0), '.16 .43 .55 1')
            box_shape(link, 'cart_load', (.50,.37,.28), (0,0,.865,0,0,0), '.72 .60 .43 1', collision=False)
            for ix, xx in enumerate((-.20,.20)):
                for iy, yy in enumerate((-.19,.19)):
                    cylinder_shape(link, f'wheel_{ix}_{iy}', .07,.04,
                                   (xx,yy,.07,math.pi/2,0,0), '.10 .12 .14 1')
        else:
            cylinder_shape(link, 'person_collision', .22,1.65, (0,0,.825,0,0,0), '.90 .37 .08 1', collision=True)
            for i, yy in enumerate((-.10,.10)):
                cylinder_shape(link, f'leg_{i}', .065,.70, (0,yy,.35,0,0,0), '.15 .22 .31 1')
            box_shape(link, 'vest', (.27,.37,.55), (0,0,1.05,0,0,0), '1 .48 .08 1', collision=False)
            cylinder_shape(link, 'helmet', .14,.18, (0,0,1.60,0,0,0), '.96 .94 .81 1')
    return root


def raster(fixed):
    """Occupied iff the original square has positive-area static overlap."""
    data = np.full((HEIGHT,WIDTH),254,dtype=np.uint8)
    for box in fixed:
        lo_x = max(0, math.floor((box['x_min']-ORIGIN[0])/RESOLUTION+1e-8))
        hi_x = min(WIDTH, math.ceil((box['x_max']-ORIGIN[0])/RESOLUTION-1e-8))
        lo_y = max(0, math.floor((box['y_min']-ORIGIN[1])/RESOLUTION+1e-8))
        hi_y = min(HEIGHT, math.ceil((box['y_max']-ORIGIN[1])/RESOLUTION-1e-8))
        data[HEIGHT-hi_y:HEIGHT-lo_y,lo_x:hi_x]=0
    return data


def segment_rectangle_distance(a, b, box):
    """Exact planar segment-to-original-rectangle distance (metres)."""
    lo_x, hi_x, lo_y, hi_y = (box[k] for k in ('x_min','x_max','y_min','y_max'))
    dx,dy=b[0]-a[0],b[1]-a[1]
    entry,leave=0.,1.
    intersects=True
    for origin,delta,low,high in ((a[0],dx,lo_x,hi_x),(a[1],dy,lo_y,hi_y)):
        if abs(delta)<1e-15:
            if not low<=origin<=high:intersects=False;break
        else:
            first,last=sorted(((low-origin)/delta,(high-origin)/delta))
            entry,leave=max(entry,first),min(leave,last)
            if entry>leave:intersects=False;break
    if intersects:return 0.
    point_box=lambda p:math.hypot(max(lo_x-p[0],p[0]-hi_x,0.),max(lo_y-p[1],p[1]-hi_y,0.))
    distances=[point_box(a),point_box(b)];length2=dx*dx+dy*dy
    for x,y in ((lo_x,lo_y),(lo_x,hi_y),(hi_x,lo_y),(hi_x,hi_y)):
        t=0. if length2==0 else max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/length2))
        distances.append(math.hypot(x-a[0]-t*dx,y-a[1]-t*dy))
    return min(distances)


def font(size):
    return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',size)


def floor_asset(root, markers):
    dest=root/'gazebo_models/warehouse_floor_surface'
    for folder in ('meshes','materials/scripts','materials/textures'):(dest/folder).mkdir(parents=True,exist_ok=True)
    image=Image.new('RGB',(800,2440),(183,189,191));draw=ImageDraw.Draw(image)
    pix=lambda x,y:((x+5)*80,(3-y)*80)
    for x in (1.02,4.45):draw.line([pix(x,-27.2),pix(x,2.3)],fill=(230,190,54),width=5)
    for y in np.arange(-26.5,1.,1.3):draw.line([pix(2.85,y),pix(2.85,y+.6)],fill=(231,234,229),width=4)
    for marker in markers[1:]:
        x,y=pix(1.7,marker['y']);draw.rounded_rectangle((x-48,y-28,x+48,y+28),radius=5,fill=(57,80,89))
        draw.text((x,y),f'S{marker["id"]:02d}',anchor='mm',font=font(26),fill=(247,247,238))
        draw.rectangle((*pix(.90,marker['y']+.39),*pix(2.35,marker['y']-.39)),outline=(226,189,64),width=3)
    draw.text(pix(-1.7,1.6),'SMART VISION',anchor='mm',font=font(25),fill=(63,79,84))
    draw.text(pix(-1.7,1.1),'LOGISTICS',anchor='mm',font=font(17),fill=(76,93,98))
    draw.text(pix(3.75,1.35),'HOME',anchor='mm',font=font(17),fill=(50,74,86))
    image.save(dest/'materials/textures/warehouse_floor.png')
    (dest/'materials/scripts/floor.material').write_text('''material SmartVision/WarehouseFloor
{ technique { pass { lighting off texture_unit { texture warehouse_floor.png filtering bilinear } } } }
''')
    (dest/'meshes/floor.obj').write_text('''v -5 -15.25 0
v 5 -15.25 0
v 5 15.25 0
v -5 15.25 0
vt 0 0
vt 1 0
vt 1 1
vt 0 1
vn 0 0 1
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
''')
    model=ET.Element('sdf',version='1.6');m=element(model,'model',name=dest.name);element(m,'static','true')
    v=element(element(m,'link',name='floor'),'visual',name='labels');mesh=element(element(v,'geometry'),'mesh')
    element(mesh,'uri','model://warehouse_floor_surface/meshes/floor.obj')
    script=element(element(v,'material'),'script')
    for part in ('scripts','textures'):element(script,'uri',f'model://warehouse_floor_surface/materials/{part}')
    element(script,'name','SmartVision/WarehouseFloor');element(v,'cast_shadows','false')
    save_xml(model,dest/'model.sdf')
    config=ET.Element('model');element(config,'name','Warehouse floor markings');element(config,'version','1.0');element(config,'sdf','model.sdf',version='1.6')
    save_xml(config,dest/'model.config')


def preview(root, fixed, markers, actors):
    image=Image.new('RGB',(850,1420),'#f0f3f4');draw=ImageDraw.Draw(image)
    scale=40.;pix=lambda x,y:(180+(x+5)*scale,100+(3-y)*scale)
    for box in fixed:
        draw.rectangle((*pix(box['x_min'],box['y_max']),*pix(box['x_max'],box['y_min'])),fill='#4b6273' if 'rack' in box['name'] else '#929ca3')
    for marker in markers:
        x,y=pix(marker['x'],marker['y']);draw.ellipse((x-7,y-7,x+7,y+7),fill='#ffae43')
        draw.text((x+14,y), 'HOME' if marker['id']==0 else f'S{marker["id"]:02d}',anchor='lm',font=font(18),fill='#193d50')
    colors=('#1b9b9b','#635eb2','#d86a49')
    for actor,color in zip(actors,colors):
        pts=[pix(p['x'],p['y'])for p in actor['waypoints']];draw.line(pts,fill=color,width=5)
        for x,y in pts:draw.ellipse((x-5,y-5,x+5,y+5),fill=color)
    draw.text((55,25),'SMART VISION | WAREHOUSE',font=font(26),fill='#193d50')
    draw.text((55,62),'10 sectors / 3 moving actors / 5 cm generated static map',font=font(15),fill='#536b78')
    draw.text((55,1355),'Geometry preview, not a Gazebo camera capture or navigation result.',font=font(14),fill='#536b78')
    (root/'docs').mkdir(parents=True,exist_ok=True)
    image.save(root/'docs/warehouse_logistics_plan.png')
    # Deterministic isometric drawing from the same full static rectangles.
    out=Image.new('RGB',(1500,1000),'#eef2f4');d=ImageDraw.Draw(out)
    projection=lambda x,y,z:(780+26*x+13*(y+12.25),650+5*x-15*(y+12.25)-90*z)
    floor=[projection(x,y,0)for x,y in ((-5,-27.5),(5,-27.5),(5,3),(-5,3))]
    d.polygon(floor,fill='#c7cecf')
    for box in sorted(fixed,key=lambda b:('rack' in b['name'],-(b['y_min']+b['y_max']))):
        x0,x1,y0,y1,h=*(box[k]for k in ('x_min','x_max','y_min','y_max')),box['height']
        # Omit the east/front walls in this explanatory cutaway only.
        if box['name'] in ('warehouse_wall_east','warehouse_wall_south'):continue
        p=[projection(x,y,z)for z in (0,h)for x,y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1))]
        rack='rack' in box['name']
        d.polygon([p[0],p[1],p[5],p[4]],fill='#486579' if rack else '#9ca8ae')
        d.polygon([p[1],p[2],p[6],p[5]],fill='#38596e' if rack else '#b0b9bd')
        d.polygon(p[4:],fill='#8396a0' if rack else '#c4ccce')
        if rack:
            for z in (.5,1.3,2.1):d.line([projection(x1,y0,z),projection(x1,y1,z)],fill='#f19b45',width=4)
            identifier=int(box['name'][-2:]);d.text(projection(x1+.2,(y0+y1)/2,.15),f'S{identifier:02d}',font=font(14),fill='#15384c')
    for actor,color in zip(actors,colors):
        pts=[projection(p['x'],p['y'],.03)for p in actor['waypoints']];d.line(pts,fill=color,width=4)
        x,y=pts[0];d.rounded_rectangle((x-9,y-13,x+9,y+3),radius=3,fill=color)
    d.text((65,45),'SMART VISION | LOGISTICS WAREHOUSE',font=font(30),fill='#193d50')
    d.text((65,90),'10 racks, rear-camera ArUco stations, moving carts and a worker',font=font(19),fill='#536b78')
    d.text((65,930),'Generated geometry cutaway. Gazebo contains all four collision walls.',font=font(17),fill='#536b78')
    out.save(root/'docs/warehouse_logistics_overview.png')


def validate(root):
    fixed,markers,boards,actors=layout()
    pkg=root/PACKAGE
    data=np.asarray(Image.open(pkg/'maps/warehouse_logistics.pgm'))
    if not np.array_equal(data,raster(fixed)):raise AssertionError('PGM/static collision raster mismatch')
    map_config=yaml.safe_load((pkg/'maps/warehouse_logistics.yaml').read_text())
    if (map_config['image']!='warehouse_logistics.pgm' or map_config['resolution']!=RESOLUTION
            or map_config['origin']!=list(ORIGIN)):
        raise AssertionError('Map YAML/geometry frame mismatch')
    if yaml.safe_load((pkg/'config/aruco_warehouse_markers.yaml').read_text())['markers']!=markers:
        raise AssertionError('Marker goal configuration/geometry mismatch')
    world=ET.parse(pkg/'worlds/warehouse_logistics.world').getroot().find('world')
    for box in fixed:
        model=world.find(f"model[@name='{box['name']}']")
        xyz=list(map(float,model.findtext('pose').split()))
        shape=model.find("link/collision[@name='mapped_solid_collision']")
        size=list(map(float,shape.findtext('geometry/box/size').split()))
        actual=(xyz[0]-size[0]/2,xyz[0]+size[0]/2,xyz[1]-size[1]/2,xyz[1]+size[1]/2)
        if not np.allclose(actual,[box[k]for k in ('x_min','x_max','y_min','y_max')],atol=1e-8):raise AssertionError('World box/map mismatch')
        if size[2] < .17:raise AssertionError('Rack laser plane not solid')
    # Existing 5-cell square inflation is deliberately used for connectivity;
    # the validation does not silently shrink the production footprint.
    blocked=np.flipud(data<128)
    inflated=blocked.copy()
    padded=np.pad(blocked,5,constant_values=True)
    for dy in range(11):
        for dx in range(11):inflated|=padded[dy:dy+HEIGHT,dx:dx+WIDTH]
    cells=[(int((m['x']-ORIGIN[0])/RESOLUTION),int((m['y']-ORIGIN[1])/RESOLUTION))for m in markers]
    if any(inflated[y,x] for x,y in cells):raise AssertionError('A goal is occupied after 5-cell inflation')
    visited={cells[0]};queue=deque(cells[:1])
    while queue:
        x,y=queue.popleft()
        for a,b in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            if 0<=a<WIDTH and 0<=b<HEIGHT and not inflated[b,a] and (a,b)not in visited:
                visited.add((a,b));queue.append((a,b))
    if any(c not in visited for c in cells):raise AssertionError('A sector is disconnected with 5-cell inflation')
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    camera=[]
    for marker,board in zip(markers,boards):
        instance=world.find(f"include[name='warehouse_marker_{marker['id']:02d}']")
        if (instance is None or instance.findtext('uri')!='model://'+marker_model(marker['id'])
                or not np.allclose(list(map(float,instance.findtext('pose').split())),board['pose'],atol=1e-8)):
            raise AssertionError('World marker pose/observation configuration mismatch')
        image=cv2.imread(str(root/'gazebo_models'/marker_model(marker['id'])/'materials/textures'/(marker_model(marker['id'])+'.png')),0)
        camera_xy=(marker['x']-CAMERA_REAR_OFFSET*math.cos(marker['yaw']),marker['y']-CAMERA_REAR_OFFSET*math.sin(marker['yaw']))
        bx,by=board['pose'][:2]
        distance=math.hypot(bx-camera_xy[0],by-camera_xy[1])
        # The front of each generated board points along its local -X.
        normal=(-math.cos(board['pose'][5]),-math.sin(board['pose'][5]))
        if normal[0]*(camera_xy[0]-bx)+normal[1]*(camera_xy[1]-by)<=0:
            raise AssertionError('Marker is back-facing the rear camera')
        # Test the printed surface rather than the centre embedded in a rack.
        printed=(bx+.0021*normal[0],by+.0021*normal[1])
        if min(segment_rectangle_distance(camera_xy,printed,b)for b in fixed)<1e-6:
            raise AssertionError('A static collision occludes the marker')
        if min(segment_rectangle_distance((marker['x'],marker['y']),
                (marker['x'],marker['y']),b)for b in fixed)<RADIUS:
            raise AssertionError('Goal circular body overlaps a static collision')
        fx=320/(2*math.tan(CAMERA_HFOV/2));black_pixels=MARKER_SIZE*fx/distance
        # Independent pixel test at the actual ideal camera's nominal size;
        # it is not a rendered rear-camera observation.
        side=round(.20*fx/distance);small=cv2.resize(image,(side,side),interpolation=cv2.INTER_AREA)
        frame=np.full((240,320),170,dtype=np.uint8);frame[120-side//2:120-side//2+side,160-side//2:160-side//2+side]=small
        _,ids,_=cv2.aruco.detectMarkers(frame,dictionary)
        detected=[] if ids is None else ids.flatten().tolist()
        if detected != [marker['id']]:raise AssertionError(f'Ideal projected marker unreadable: {marker["id"]}')
        camera.append(dict(id=marker['id'],camera_xy=list(camera_xy),marker_distance_m=distance,black_edge_pixels=black_pixels,ideal_pixel_detection=detected))
    actor_clearance=[]
    for actor in actors:
        model=world.find(f"model[@name='{actor['model_name']}']")
        if model.findtext('static')!='false' or model.findtext('link/kinematic')!='true' or not model.findall('link/collision'):
            raise AssertionError('Actor must have a real kinematic collision body')
        points=[(p['x'],p['y'])for p in actor['waypoints']]
        clearance=min(segment_rectangle_distance(a,b,box)-actor['collision_radius_m']
                      for a,b in zip(points,points[1:])for box in fixed)
        if clearance<.02:raise AssertionError('Actor swept circumscribed body intersects fixed geometry')
        actor_clearance.append(dict(model_name=actor['model_name'],minimum_static_gap_m=clearance))
    return dict(passed=True,kind='static_geometry_and_ideal_pixel_checks_only',resolution=RESOLUTION,map_width=WIDTH,map_height=HEIGHT,
                static_rectangles=len(fixed),sector_ids=list(range(1,11)),home_id=0,connected_goal_cells=len(cells),
                reachable_four_connected_cells=len(visited),static_inflation_cells=5,robot_radius_m=RADIUS,
                dynamic_actors_omitted_from_static_map=True,marker_dictionary='DICT_4X4_50',marker_length_m=MARKER_SIZE,
                camera_projection_checks=camera,actor_full_segment_clearance=actor_clearance,
                marker_static_line_of_sight_verified=True,closed_loop_navigation_tested=False,gazebo_rendered_camera_tested=False,
                limits=['PGM is generated from static geometry, not SLAM.',
                        'Actor movement requires the separate Gazebo state-service runner.',
                        'Ideal texture downsampling does not replace rendered rear-camera detection or closed-loop arrival tests.'])


def generate(root):
    fixed,markers,boards,actors=layout();pkg=root/PACKAGE
    for folder in ('worlds','maps','config'):(pkg/folder).mkdir(parents=True,exist_ok=True)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for identifier in range(11):
        target=root/'gazebo_models'/marker_model(identifier)
        with tempfile.TemporaryDirectory(prefix='warehouse-marker-') as tmp:
            source=Path(tmp)/target.name
            create_model(source,'DICT_4X4_50',dictionary,identifier,MARKER_SIZE,.025,600)
            if target.exists():shutil.rmtree(target)
            shutil.copytree(source,target)
    floor_asset(root,markers)
    save_xml(build_world(fixed,boards,actors),pkg/'worlds/warehouse_logistics.world')
    Image.fromarray(raster(fixed)).save(pkg/'maps/warehouse_logistics.pgm')
    configs={
        pkg/'maps/warehouse_logistics.yaml':dict(image='warehouse_logistics.pgm',mode='trinary',resolution=RESOLUTION,origin=list(ORIGIN),negate=0,occupied_thresh=.65,free_thresh=.196),
        pkg/'config/aruco_warehouse_markers.yaml':dict(markers=markers),
        pkg/'config/warehouse_dynamic_routes.yaml':dict(frame_id='world',robot_model_name='waffle_pi',robot_radius_m=RADIUS,robot_clearance_m=.15,update_rate_hz=10.,fixed_obstacles=fixed,actors=actors),
    }
    pixel=lambda x,y:[(x-ORIGIN[0])/RESOLUTION,HEIGHT-(y-ORIGIN[1])/RESOLUTION]
    configs[pkg/'config/warehouse_floorplan.yaml']=dict(map=dict(width=WIDTH,height=HEIGHT,resolution=RESOLUTION,origin=list(ORIGIN)),style='architectural',label='물류 창고 · 10개 섹터',display_rotation_deg=0.,
        free_paths=[[pixel(-4.85,2.85),pixel(4.85,2.85),pixel(4.85,-27.35),pixel(-4.85,-27.35)]],
        occupied_paths=[[pixel(b['x_min'],b['y_max']),pixel(b['x_max'],b['y_max']),pixel(b['x_max'],b['y_min']),pixel(b['x_min'],b['y_min'])]for b in fixed],obstacle_paths=[])
    for path,value in configs.items():path.write_text(yaml.safe_dump(value,allow_unicode=True,sort_keys=False))
    preview(root,fixed,markers,actors)
    result=validate(root)
    files=[pkg/'worlds/warehouse_logistics.world',pkg/'maps/warehouse_logistics.pgm',*configs]
    result['generated_file_sha256']={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()for p in files}
    result['generator_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    result['model_asset_sha256']={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
        for folder in sorted((root/'gazebo_models').glob('*warehouse*'))
        if folder.is_dir() for p in sorted(folder.rglob('*'))if p.is_file()}
    result['marker_world_poses']=boards
    path=pkg/'config/warehouse_layout_manifest.json';path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,default=ROOT)
    parser.add_argument('--check-only','--check',action='store_true')
    args=parser.parse_args();root=args.output_root.resolve()
    result=validate(root)if args.check_only else generate(root)
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
