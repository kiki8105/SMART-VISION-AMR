// Real rendering fragments with local fixtures; no fetch, ROS, or actuation.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const app=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/app.js','utf8');
const messages=app.slice(app.indexOf('  function missionHoldReason('),app.indexOf('  function renderMarkerAlignment('));
const route=app.slice(app.indexOf('    const publishedPath=Array.isArray(snapshot.path)'),app.indexOf('    const home=homepoint(),markers='));
assert.ok(messages&&route,'both production rendering fragments are present');
const elements=new Map(),drawn=[];
const $=selector=>{if(!elements.has(selector))elements.set(selector,{hidden:false,textContent:'',value:0,dataset:{},classList:{toggle(){}}});return elements.get(selector);};
const terminal=new Set(['IDLE','COMPLETED','CANCELED','ERROR','HOME_REACHED']);
const context={$,snapshot:null,terminal,lastPath:[],map:{preview:false,configured_match:true},
 currentMission:()=>context.snapshot.mission,samePathStamp:(a,b)=>JSON.stringify(a)===JSON.stringify(b),
 xy:point=>[point.x,point.y],svg:(name,attrs)=>({name,attrs}),canvas:{append(...items){drawn.push(...items);}}};
vm.createContext(context);vm.runInContext(messages+'\nfunction drawRoute(){\n'+route+'\n}\nglobalThis.render=renderLocalization;',context);
function fixture(){return{mission:{state:'NAVIGATING',reason:'구역으로 이동 중',estop_latched:false},
 localization:{state:'IDLE',active:false,available:false,reason:'차체 중심 반경 30cm 이상의 회전 공간이 필요합니다.'},
 localization_health:{level:'ok',autonomous_allowed:true,reasons:[],metrics:{}},
 safety:{latched:false,mode:'auto'},navigation:{status:'ACTIVE',planner_status:'READY',message:''},
 path:[{x:0,y:0},{x:1,y:1}],path_stamp:{sec:1,nanosec:2},local_detour:{available:false}};}
context.snapshot=fixture();context.render();
assert.equal($('#localization-mission-state').hidden,true,'unavailable optional rotation does not fabricate a mission stop');
assert.match($('#localization-reason').textContent,/^선택 도구 시작 조건.*30cm/);
assert.equal($('#localization-health-title').textContent,'위치 신뢰도 정상');
context.snapshot.mission.state='WAIT_REPLAN';context.snapshot.navigation.planner_status='WAIT_REPLAN';
context.snapshot.navigation.message='Planner has no safe route; waiting for replan';context.render();
assert.equal($('#localization-mission-state').hidden,false);
assert.match($('#localization-mission-state').textContent,/현재 임무.*통과할 경로를 찾지 못해/);
assert.doesNotMatch($('#localization-mission-state').textContent,/30cm|Planner|Nav2/);
assert.match($('#localization-reason').textContent,/^선택 도구 시작 조건/);
context.snapshot=fixture();context.snapshot.mission={state:'IDLE'};
context.snapshot.navigation.planner_status='WAIT_REPLAN';
context.snapshot.observed_route={monitoring:true,phase:'WAIT_REPLAN',reason:'관측 중인 경로에 통로가 없어 다시 확인합니다.'};
context.render();assert.match($('#localization-mission-state').textContent,/관측 중인 경로에 통로가 없어/);
context.snapshot=fixture();context.snapshot.mission.state='CONFIRMING';
context.snapshot.marker_alignment={state:'HOLD',reason:'라이다 회전 여유를 재확인하며 정지합니다.'};context.render();
assert.match($('#localization-mission-state').textContent,/라이다 회전 여유를 재확인/);
context.snapshot.safety.scan_guard_hold=true;context.render();
assert.match($('#localization-mission-state').textContent,/가까운 장애물 관측/);
context.snapshot=fixture();context.snapshot.localization={state:'ROTATING',active:true,reason:'현재 회전을 멈추고 관측을 다시 확인합니다.'};context.render();
assert.equal($('#localization-reason').textContent,context.snapshot.localization.reason,'an active operation retains its actual reason');

function draw(){drawn.length=0;vm.runInContext('drawRoute();',context);return drawn.find(item=>item.attrs.stroke!=='var(--panel)');}
context.snapshot=fixture();let path=draw();
assert.equal(path.attrs['stroke-dasharray'],'none');assert.equal($('#map-route-legend').hidden,false);
context.lastPath=context.snapshot.path;context.snapshot.path=[];context.snapshot.mission.state='WAIT_REPLAN';
context.snapshot.navigation.planner_status='WAIT_REPLAN';path=draw();
assert.equal(path.attrs.stroke,'#87919b');assert.equal(path.attrs['stroke-dasharray'],'5 5');
assert.equal($('#map-replan-legend').hidden,false);assert.equal($('#map-route-legend').hidden,true);
context.snapshot=fixture();context.snapshot.local_detour={available:true,state:'ACTIVE',plan_kind:'LOCAL_DETOUR',path_stamp:context.snapshot.path_stamp};path=draw();
assert.equal(path.attrs.stroke,'#d27b29');assert.equal(path.attrs['stroke-dasharray'],'9 4');
assert.equal($('#map-detour-legend').hidden,false);assert.equal($('#map-replan-legend').hidden,true);
const html=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/index.html','utf8');
assert.match(html,/선택 도구 · 회전으로 위치 다시 확인/);assert.match(html,/이 도구를 실행할 필요가 없습니다/);
assert.match(html,/현재 우회경로 · 주황 점선/);assert.match(html,/재계획 중 · 회색 점선/);
assert.match(html,/id="localization-mission-state"[^>]*role="status"/);
console.log('PASS: optional rotation conditions stay separate from actual replan/hold reasons; gray prior routes and orange active detours have distinct legends.');
