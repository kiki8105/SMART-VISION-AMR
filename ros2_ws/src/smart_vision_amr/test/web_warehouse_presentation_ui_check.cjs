// Presentation changes text only on an explicit, healthy domain-178 simulation.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const source=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/app.js','utf8');
const helpers=source.slice(source.indexOf('  function warehousePresentationEnabled('),
  source.indexOf('  function initialPoseValues(){'));
const elements=new Map();
function element(text=''){return {textContent:text,hidden:false,disabled:true,children:[],
  classList:{contains:()=>false},querySelector:()=>null};}
const $=selector=>{if(!elements.has(selector))elements.set(selector,element());return elements.get(selector);};
const data=()=>({connection:{domain_id:178,use_sim_time:true},
  mission:{state:'PAUSED',current_marker_id:10,estop_latched:false,route:[10],completed_ids:[]},
  manual:{state:'ACTIVE',active:true},health:{ready:true,missing:[]},safety:{},
  localization:{active:false},localization_health:{level:'healthy'},
  operation:{phase:'manual',level:'info'},marker_alignment:{state:'IDLE'},
  path:[{x:1,y:2},{x:3,y:4}],events:[{message:'조이스틱 개입 원본 기록'}]});
const sandbox={$ ,snapshot:data(),window:{location:{search:'?presentation=1'}},URLSearchParams,
  currentMission:()=>sandbox.snapshot.mission,markerLabel:id=>`Sector ${id}`};
vm.createContext(sandbox);
vm.runInContext(helpers+'\nglobalThis.enabled=warehousePresentationEnabled;'+
  'globalThis.manual=warehousePresentationManual;globalThis.render=renderWarehousePresentation;',sandbox);
for(const [domain,sim,query,expected] of [[178,true,'?presentation=1',true],
  [178,true,'',false],[178,true,'?presentation=0',false],[16,false,'?presentation=1',false],
  [16,true,'?presentation=1',false],[176,true,'?presentation=1',false],[178,false,'?presentation=1',false]]){
  const candidate=data();candidate.connection={domain_id:domain,use_sim_time:sim};
  assert.equal(sandbox.enabled(candidate,query),expected);
}
$('#state-pill').textContent='수동 우선';
$('#mission-reason').textContent='수동 조종을 우선합니다.';
$('#operation-phase').textContent='수동 조종 우선';
$('#operation-diagnostics').children=[element('센서 진단 원본')];
$('#event-log').children=[element('조이스틱 개입 원본 기록')];
$('#failure-memory-reason').textContent='실제 장애물 해소 대기';
$('#dispatch-blockers').children=[element('조이스틱을 놓고 자동주행 연결이 복구될 때까지 기다리세요.'),
  element('접수된 요청 물품 적재 확인')];
$('#startup-stop-state').textContent='조이스틱 우선';
$('#startup-stop-detail').textContent='조이스틱을 놓고 제어권 반환을 기다리세요.';
const row=element(),routeText=element('일시정지');
row.classList.contains=name=>name==='current';row.querySelector=()=>routeText;
$('#route-editor').children=[row];
const before=JSON.stringify(sandbox.snapshot);
sandbox.render();
assert.equal($('.joystick-panel').hidden,true);
assert.equal($('#state-pill').textContent,'경로 주행 중');
assert.equal($('#mission-title').textContent,'경로 주행 중 · Sector 10');
assert.equal($('#operation-phase').textContent,'경로 주행 중');
assert.equal(routeText.textContent,'경로 주행 중');
assert.equal($('#startup-stop-state').textContent,'주행 중');
assert.doesNotMatch($('#dispatch-blockers').children[0].textContent,/조이스틱/);
assert.equal($('#dispatch-blockers').children[1].textContent,'접수된 요청 물품 적재 확인');
assert.equal($('#operation-diagnostics').children[0].textContent,'센서 진단 원본');
assert.equal($('#event-log').children[0].textContent,'조이스틱 개입 원본 기록');
assert.equal($('#failure-memory-reason').textContent,'실제 장애물 해소 대기');
assert.equal($('#dispatch-start').disabled,true,'presentation must not enable an action');
assert.equal(JSON.stringify(sandbox.snapshot),before,'real mission, route, authority and logs are immutable');

for(const modify of [d=>{d.health.ready=false;d.health.missing=['최신 /scan'];},
  d=>{d.mission.estop_latched=true;},d=>{d.safety.latched=true;},
  d=>{d.safety.estop_latched=true;},d=>{d.safety.pending='reset';},
  d=>{d.localization.active=true;},d=>{d.localization_health.level='critical';},
  d=>{d.operation.level='critical';},d=>{d.failure_memory={state:'SENSOR_HOLD'};},
  d=>{d.failure_memory={state:'CONTROLLER_HOLD'};},d=>{d.mission.state='ERROR';},
  ...['STALE_SCAN','POSE_UNAVAILABLE','TF_UNAVAILABLE','INVALID_SCAN',
    'BRAKING_FEEDBACK_UNAVAILABLE','CONTROL_DEADLINE'].map(code=>d=>{d.safety.scan_guard_code=code;})]){
  sandbox.snapshot=data();modify(sandbox.snapshot);
  $('#state-pill').textContent='실제 오류 표시';
  $('#mission-reason').textContent='센서·비상정지 원본 안내';
  const original=JSON.stringify(sandbox.snapshot);
  assert.equal(sandbox.manual(sandbox.snapshot,'?presentation=1'),false);
  sandbox.render();
  assert.equal($('#state-pill').textContent,'실제 오류 표시');
  assert.equal($('#mission-reason').textContent,'센서·비상정지 원본 안내');
  assert.equal(JSON.stringify(sandbox.snapshot),original);
}
sandbox.snapshot=data();sandbox.window.location.search='';
$('#state-pill').textContent='수동 우선';sandbox.render();
assert.equal($('.joystick-panel').hidden,false);
assert.equal($('#state-pill').textContent,'수동 우선');
console.log('Warehouse presentation opt-in, neutral labels, error visibility and state immutability PASS');
