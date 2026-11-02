// Render the real map drawing with a small synthetic OccupancyGrid frame.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const source=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/app.js','utf8');
const drawing=source.slice(source.indexOf('  function polygonPath('),source.indexOf('  function updateCamera(){'));
const elements=new Map();
function element(name='div',attrs={}){
  return{name,attrs,children:[],hidden:false,textContent:'',dataset:{},clientWidth:420,clientHeight:420,
    classList:{toggle(){}},setAttribute(key,value){this.attrs[key]=String(value);},
    replaceChildren(){this.children=[];},append(...items){this.children.push(...items);}};
}
const $=selector=>{if(!elements.has(selector))elements.set(selector,element());return elements.get(selector);};
const svg=(name,attrs={},text='')=>{const node=element(name,attrs);node.textContent=text;return node;};
const snapshot={map:{width:4,height:3,resolution:1,origin:{x:0,y:0,yaw:0},version:7,
                    preview:false,configured_match:true,raw_available:true},
  observed_obstacles:{map_version:7,cells:[1],age:.2},
  remembered_obstacles:{map_version:7,cells:[1,6]},
  path:[{x:.5,y:.5},{x:2.5,y:1.5}],
  path_stamp:{sec:23,nanosec:4},
  local_detour:{available:true,state:'ACTIVE',plan_kind:'LOCAL_DETOUR',path_stamp:{sec:23,nanosec:4}},
  mission:{state:'NAVIGATING'},navigation:{planner_status:'NAVIGATING'},markers:[],
  robot:{pose:{x:.5,y:.5,yaw:0}}};
const layout={width:4,height:3,version:7,style:'architectural',label:'강의실 평면도',
  display_rotation_deg:-1.57646181885,
  free_paths:[[[0,0],[4,0],[4,3],[0,3]]],occupied_paths:[],obstacle_paths:[]};
const sandbox={$,svg,snapshot,mapLayout:layout,mapVersion:7,mapLayoutLoading:false,
  showRawMap:false,mapImageUrl:'/api/map.png',lastPath:[],lastMonitoredPathGoal:null,draft:[],
  currentMission:()=>snapshot.mission,homepoint:()=>null,markerLabel:()=>'',
  isPhysicalClassroom:()=>true,terminal:new Set()};
vm.createContext(sandbox);vm.runInContext(drawing+'\nglobalThis.draw=drawMap;globalThis.point=rotateMapPoint;globalThis.cells=obstacleCellPath;globalThis.cache=cacheMonitoredPath;',sandbox);
function descendants(root,name){return root.children.flatMap(child=>[
  ...(child.name===name?[child]:[]),...descendants(child,name)]);}
const canvas=$('#map-svg');
sandbox.draw();
let obstacle=descendants(canvas,'path').find(el=>el.attrs.class==='map-live-obstacles');
assert.equal(obstacle.attrs.d,'M1 2h1v1h-1Z','ROS bottom-left row becomes top-left pixel');
let remembered=descendants(canvas,'path').find(el=>el.attrs.class==='map-remembered-obstacles');
assert.equal(remembered.attrs.d,'M1 2h1v1h-1ZM2 1h1v1h-1Z','confirmed cells share the live map transform');
assert.match(canvas.children[0].attrs.transform,/rotate\(-1\.57646181885 2 1\.5\)/,'floor uses display rotation');
assert.match(canvas.children[1].attrs.transform,/rotate\(-1\.57646181885 2 1\.5\)/,'memory uses identical rotation');
assert.match(canvas.children[2].attrs.transform,/rotate\(-1\.57646181885 2 1\.5\)/,'live obstacles use identical rotation');
let path=descendants(canvas,'polyline').find(el=>el.attrs['aria-label']==='국소 우회경로');
assert.equal(path.attrs.stroke,'#d27b29');
assert.equal($('#map-detour-legend').hidden,false);
assert.equal($('#map-route-legend').hidden,true);
assert.equal($('#map-obstacle-legend').hidden,false);
assert.equal($('#map-remembered-obstacle-legend').hidden,false);
snapshot.local_detour.path_stamp={sec:23,nanosec:5};
sandbox.draw();
path=descendants(canvas,'polyline').find(el=>el.attrs['aria-label']==='계획 경로');
assert.ok(path,'a newer local status must not recolor the previous global path');
assert.equal($('#map-detour-legend').hidden,true);
snapshot.path_stamp={sec:23,nanosec:5};
sandbox.draw();
path=descendants(canvas,'polyline').find(el=>el.attrs['aria-label']==='국소 우회경로');
assert.ok(path,'the matching local Path can be colored after either topic arrival order');
snapshot.local_detour.path_stamp={sec:23,nanosec:4};
sandbox.draw();
path=descendants(canvas,'polyline').find(el=>el.attrs['aria-label']==='계획 경로');
assert.ok(path,'an old status must not recolor the newer global Path');
snapshot.path_stamp={sec:23,nanosec:4};
sandbox.draw();
const rotated=sandbox.point(3,1,snapshot.map,-Math.PI/2);
assert.ok(Math.abs(rotated[0]-1.5)<1e-9&&Math.abs(rotated[1]-.5)<1e-9,
  'map overlays share the same pixel-space rotation as the floor');

snapshot.observed_obstacles={map_version:7,cells:[],age:.1};
snapshot.local_detour={available:true,state:'NONE',plan_kind:'GLOBAL'};
sandbox.draw();
assert.equal(descendants(canvas,'path').some(el=>el.attrs.class==='map-live-obstacles'),false);
assert.equal($('#map-obstacle-legend').hidden,true);
assert.equal(descendants(canvas,'path').some(el=>el.attrs.class==='map-remembered-obstacles'),true,
  'confirmed memory remains visible when the live scan no longer sees it');
assert.equal($('#map-remembered-obstacle-legend').hidden,false);
path=descendants(canvas,'polyline').find(el=>el.attrs['aria-label']==='계획 경로');
assert.equal(path.attrs.stroke,'var(--accent)');

snapshot.observed_obstacles={map_version:6,cells:[1],age:.1};
sandbox.draw();
assert.equal($('#map-obstacle-legend').hidden,true,'old-map cells never overlay a new map');
snapshot.observed_obstacles={map_version:7,cells:[1],age:3};
sandbox.draw();
assert.equal($('#map-obstacle-legend').hidden,true,'stale obstacle stream must disappear');
assert.equal($('#map-remembered-obstacle-legend').hidden,false,'confirmed memory does not expire with live scans');

snapshot.observed_obstacles={map_version:7,cells:[1],age:.1};
sandbox.showRawMap=true;
sandbox.draw();
const raw=canvas.children[0];
assert.equal(raw.children[0].name,'image');
assert.equal(raw.children[0].attrs.href,'/api/map/raw.png?v=7','raw view uses the original SLAM archive');
assert.match(raw.attrs.transform,/rotate\(-1\.57646181885 2 1\.5\)/,
  'raw-map artwork has the same presentation rotation as the floorplan');
assert.match(canvas.children[1].attrs.transform,/rotate\(-1\.57646181885 2 1\.5\)/,
  'remembered obstacle pixels stay registered to the raw map');
assert.match(canvas.children[2].attrs.transform,/rotate\(-1\.57646181885 2 1\.5\)/,
  'live obstacle pixels stay registered to the raw map');
assert.equal($('#map-obstacle-legend').hidden,false);
assert.equal($('#map-remembered-obstacle-legend').hidden,false);
snapshot.remembered_obstacles={map_version:7,cells:[]};
sandbox.draw();
assert.equal(descendants(canvas,'path').some(el=>el.attrs.class==='map-remembered-obstacles'),false,
  'a free-confirmed memory removal clears the overlay');
assert.equal($('#map-remembered-obstacle-legend').hidden,true);
snapshot.remembered_obstacles={map_version:6,cells:[6]};
sandbox.draw();
assert.equal($('#map-remembered-obstacle-legend').hidden,true,'old-map memory is never drawn');
snapshot.remembered_obstacles={map_version:7,cells:[6]};
snapshot.map.preview=true;
sandbox.draw();
assert.equal($('#map-remembered-obstacle-legend').hidden,true,'a saved preview cannot claim live memory');
snapshot.map.preview=false;

snapshot.mission.state='IDLE';
sandbox.draw();
assert.equal(descendants(canvas,'polyline').length,0,'an inactive mission has no old route');
snapshot.observed_route={monitoring:true,phase:'READY',active:true,plan_kind:'LOCAL_DETOUR',path_stamp:{sec:23,nanosec:4},goal_stamp:{sec:22,nanosec:1},goal:{x:2.5,y:1.5,frame_id:'map'}};
sandbox.draw();
assert.ok(descendants(canvas,'polyline').some(el=>el.attrs['aria-label']==='국소 우회경로'),
  'direct ROS simulation route is visible for read-only monitoring');
assert.ok(descendants(canvas,'g').some(el=>el.attrs['aria-label']==='현재 모의 주행 목표'),
  'direct ROS simulation goal is visible even without a matching configured marker');
snapshot.markers=[{id:1,name:'Sector 1',x:1.2,y:1.3,yaw:0,camera:'rear'},
                  {id:3,name:'Sector 3',x:2.5,y:1.5,yaw:-1.4,camera:'rear'}];
for(const rawMode of [false,true]){
  sandbox.showRawMap=rawMode;
  sandbox.draw();
  const headings=descendants(canvas,'path').filter(el=>el.attrs.class==='map-location-heading');
  assert.equal(headings.length,2,'both marker arrows are drawn in both map modes');
  assert.match(headings[0].attrs.transform,/rotate\(-1\.57646181885\)$/,
    'Sector 1 still uses its actual yaw and map presentation rotation');
  assert.match(headings[1].attrs.transform,/rotate\(90\)$/,
    'Sector 3 arrow points straight down on screen in both map modes');
  const sector3Tooltip=descendants(canvas,'title').find(el=>el.textContent.includes('ArUco ID 3'));
  assert.match(sector3Tooltip.textContent,/화면 화살표 ↓는 표시용 · 실제 차체 전방 yaw -80°/);
  assert.doesNotMatch($('#map-view-state').textContent,/Sector 3 ↓ 표시용/);
  assert.equal($('#map-heading-legend').textContent,'화살표 = 차체 전방');
  assert.match($('#map-heading-legend').title,/실제 주행 목표 yaw는 -80°/);
}
assert.equal(snapshot.markers[1].yaw,-1.4,'drawing does not change the real ROS goal heading');
assert.equal(snapshot.observed_route.goal.x,2.5,'drawing does not change the goal coordinates');
assert.ok(descendants(canvas,'g').some(el=>el.attrs.class==='map-location current'),
  'a direct ROS goal at a sector highlights its marker as the current destination');
assert.ok(descendants(canvas,'g').some(el=>el.attrs['aria-label']==='현재 차량 위치'),
  'the current robot is a distinct map SVG element');
assert.ok(descendants(canvas,'g').some(el=>el.attrs['aria-label']==='현재 차량'),
  'the current robot has an explicit visible badge');
snapshot.observed_route={monitoring:true,phase:'PLANNING',active:false,
  goal_stamp:{sec:22,nanosec:1},goal:{x:2.5,y:1.5,frame_id:'map'}};
snapshot.navigation.planner_status='PLANNING';
sandbox.draw();
let pending=descendants(canvas,'polyline').find(el=>el.attrs['aria-label']==='모의 주행 경로');
assert.equal(pending.attrs.stroke,'#87919b',
  'a published path to the current ROS goal is visible in gray during planning');
assert.equal(sandbox.cache(snapshot),true,'a current-goal candidate may be cached for replan display');
snapshot.path=[];
snapshot.observed_route.phase='WAIT_REPLAN';
snapshot.navigation.planner_status='WAIT_REPLAN';
sandbox.draw();
let prior=descendants(canvas,'polyline').find(el=>el.attrs['aria-label']==='이전 경로 · 현재 주행 아님');
assert.equal(prior.attrs.stroke,'#87919b','the last same-goal route stays gray after Path clears');
snapshot.observed_route.goal={x:1,y:2,frame_id:'map'};
sandbox.draw();
assert.equal(descendants(canvas,'polyline').length,0,
  'a path cached for a previous goal is hidden');
snapshot.observed_route.goal={x:2.5,y:1.5,frame_id:'map'};
snapshot.observed_route.goal_stamp={sec:24,nanosec:1};
sandbox.draw();
assert.equal(descendants(canvas,'polyline').length,0,
  'a cached route stamped before a new goal is hidden');
snapshot.path=[{x:.5,y:.5},{x:2.5,y:1.5}];
sandbox.draw();
assert.equal(descendants(canvas,'polyline').length,0,
  'a published path stamped before a new goal is not shown as its pending route');
const externalSource=source.slice(source.indexOf('  function externalRouteDisplay('),source.indexOf('  function renderStatus(){'));
const operationSource=source.slice(source.indexOf('  function renderOperation(){'),source.indexOf('  function requestTime('));
sandbox.terminal.add('IDLE');
sandbox.markerLabel=id=>`Sector ${id}`;
sandbox.renderFailureMemory=()=>{};
sandbox.failureReason=value=>value||'';
sandbox.document={createElement:name=>element(name)};
snapshot.operation={label:'임무 대기',level:'info',detail:'저장된 웹 임무 대기'};
snapshot.route_preview=[{order:1,name:'Sector 2'},{order:2,name:'Sector 3'},{order:3,name:'Home'}];
vm.runInContext(externalSource+operationSource+'\nglobalThis.external=externalRouteDisplay;globalThis.operation=renderOperation;',sandbox);
snapshot.observed_route={monitoring:true,phase:'PLANNING',goal:{x:2.5,y:1.5,frame_id:'map'}};
assert.equal(sandbox.external(snapshot).name,'Sector 3');
assert.match(sandbox.external(snapshot).reason,/경로를 계산/);
sandbox.operation();
assert.equal($('#active-route').children.length,1,'a direct ROS goal replaces the saved web route');
assert.equal($('#active-route').children[0].textContent,'외부 ROS 목표 · Sector 3');
snapshot.path=[];
snapshot.observed_route={monitoring:true,stale:true,reason:'모의 주행 시험 종료 또는 연결 대기 · /clock, /scan, /odom',phase:'STALE',goal:null};
assert.equal(sandbox.external(snapshot).stale,true);
sandbox.operation();
assert.equal($('#active-route').children.length,1,'stale ROS monitoring must not show the saved web route');
assert.equal($('#active-route').children[0].textContent,'외부 ROS 경로 · 시험 종료/연결 대기');
sandbox.draw();
assert.equal(descendants(canvas,'polyline').length,0,'a stopped simulation has no claimed current route');
assert.equal(descendants(canvas,'g').some(el=>el.attrs['aria-label']==='현재 모의 주행 목표'),false);
assert.match($('#map-view-state').textContent,/모의 주행 종료\/연결 대기/);
snapshot.observed_route={active:false};
sandbox.operation();
assert.equal($('#active-route').children.length,3,'the saved web route returns outside ROS monitoring');
sandbox.lastPath=[];
sandbox.lastMonitoredPathGoal=null;
sandbox.draw();
assert.equal(descendants(canvas,'polyline').length,0,'completed monitored route is removed');
const markerSource=source.slice(source.indexOf('  function renderMarkerAlignment('),source.indexOf('  function externalRouteDisplay('));
sandbox.delivery=()=>({active:false});
vm.runInContext(markerSource+'\nglobalThis.markerPanel=renderMarkerAlignment;',sandbox);
snapshot.mission={state:'CONFIRMING',current_marker_id:3};
snapshot.marker_alignment={confirmation_mode:'recognized',state:'CENTERING',marker_id:3,camera:'rear',
  center_error:.2,soft_center_tolerance:.3,soft_center_timeout:3,soft_center_result:null,frames:1,required:3};
assert.equal(sandbox.markerPanel(),true,'recognized mode exposes the short centering phase');
assert.match($('#marker-alignment-phase').textContent,/중앙 보정/);
assert.match($('#marker-alignment-metrics').textContent,/중심 오차 오른쪽 10\.0%/);
assert.match($('#marker-alignment-metrics').textContent,/허용 중심 오차 ±15%/);
snapshot.marker_alignment.state='CONFIRMED';
snapshot.marker_alignment.soft_center_result='centered';
assert.equal(sandbox.markerPanel(),true);
assert.match($('#marker-alignment-phase').textContent,/중앙 확인 완료/);
assert.match($('#marker-alignment-metrics').textContent,/중앙 보정 완료/);
snapshot.marker_alignment.state='FAILED';
snapshot.marker_alignment.soft_center_result='out_of_band';
snapshot.marker_alignment.reason='중앙 범위 밖 · 목표를 유지합니다.';
assert.equal(sandbox.markerPanel(),true);
assert.match($('#marker-alignment-metrics').textContent,/도착 미확인/);
console.log('PASS: observed cells, removal/staleness, exact detour Path stamp, display rotation and stale path.');
