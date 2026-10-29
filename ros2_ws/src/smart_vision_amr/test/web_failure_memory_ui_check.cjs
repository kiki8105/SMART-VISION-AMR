// Exercise the actual recovery UI without ROS, network requests or movement.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const source=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/app.js','utf8');
const render=source.slice(source.indexOf('  function failureReason('),source.indexOf('  function renderOperation(){'));
const elements=new Map();
function element(){return{hidden:false,textContent:'',children:[],replaceChildren(){this.children=[];},append(item){this.children.push(item);}};}
function $(selector){if(!elements.has(selector))elements.set(selector,element());return elements.get(selector);}
const sandbox={$,Number,String,Math,document:{createElement:element},currentMission:()=>sandbox.snapshot.mission};
vm.createContext(sandbox);vm.runInContext(render+'\nglobalThis.render=renderFailureMemory;',sandbox);
function fixture(state='AVOIDING'){return{mission:{state:'NAVIGATING'},failure_memory:{available:true,state,reason:'실패한 진입 방향 회피',active:[{x:1.2,y:2.3,reason:'선회 공간 부족',expires_in:15.1}]}};}
for(const [state,text] of [['AVOIDING','재계획'],['RECOVERING','후진'],['REJOINING','진입 확인'],['WAIT_ALTERNATIVE','대체 경로 대기'],['SENSOR_HOLD','센서 회복']]){
 sandbox.snapshot=fixture(state);sandbox.render();assert.equal($('#failure-memory').hidden,false);assert.match($('#failure-memory-phase').textContent,new RegExp(text));assert.equal($('#failure-memory-count').textContent,'회피 기억 1건');assert.match($('#failure-memory-records').children[0].textContent,/\(1\.20, 2\.30\).*16초 후 재평가/);
}
sandbox.snapshot=fixture('OBSTACLE_HOLD');sandbox.snapshot.failure_memory.active=[];sandbox.snapshot.failure_memory.retry_count=3;sandbox.snapshot.failure_memory.retry_limit=3;sandbox.render();assert.match($('#failure-memory-phase').textContent,/장애물 해소 대기/);assert.equal($('#failure-memory-reason').textContent,'장애물이 있어 경로 주행을 잠시 멈춥니다. 장애물을 치우면 경로를 재개합니다.');assert.match($('#failure-memory-count').textContent,/재시도 3\/3회/);assert.match($('#failure-memory-records').children[0].textContent,/현재 목표를 유지/);
sandbox.snapshot.manual={active:true};sandbox.render();assert.match($('#failure-memory-phase').textContent,/수동 조종 우선/);
sandbox.snapshot.failure_memory.state='SENSOR_HOLD';sandbox.snapshot.manual={};sandbox.render();assert.match($('#failure-memory-phase').textContent,/센서 회복/);assert.doesNotMatch($('#failure-memory-reason').textContent,/장애물을 치우면/);
sandbox.snapshot=fixture();sandbox.snapshot.manual={active:true};sandbox.render();assert.match($('#failure-memory-phase').textContent,/수동 조종 우선/);
sandbox.snapshot.mission.estop_latched=true;sandbox.render();assert.match($('#failure-memory-phase').textContent,/비상정지/);
sandbox.snapshot=fixture();sandbox.snapshot.mission.state='PAUSED';sandbox.render();assert.match($('#failure-memory-phase').textContent,/일시정지/);
sandbox.snapshot=fixture('SENSOR_HOLD');sandbox.snapshot.failure_memory.active=[];sandbox.render();assert.match($('#failure-memory-records').children[0].textContent,/센서 단절은 경로 실패로 기록하지 않습니다/);
for(const [code,meaning] of [['execution_no_progress','진행되지 않음'],['execution_oscillation','좌우 선회 반복'],['controller_aborted','주행 제어 중단']]){sandbox.snapshot=fixture();sandbox.snapshot.failure_memory.reason=code;sandbox.snapshot.failure_memory.active[0].reason=code;sandbox.render();assert.match($('#failure-memory-reason').textContent,new RegExp(meaning));assert.match($('#failure-memory-records').children[0].textContent,new RegExp(meaning));assert.equal(sandbox.snapshot.failure_memory.reason,code,'translation must not change backend schema');}
for(const change of [x=>delete x.failure_memory,x=>x.failure_memory.available=false,x=>x.mission.state='IDLE',x=>x.mission.state='COMPLETED',x=>{x.failure_memory.state='CLEAR';x.failure_memory.active=[];}]){sandbox.snapshot=fixture();change(sandbox.snapshot);sandbox.render();assert.equal($('#failure-memory').hidden,true);}
sandbox.snapshot=fixture();sandbox.snapshot.failure_memory.active[0].reason='<img onerror=alert(1)>';sandbox.render();assert.match($('#failure-memory-records').children[0].textContent,/<img/);
assert.doesNotMatch(render,/\.innerHTML|\bfetch\(|\baction\(|\bapi\(/,'display must not actuate or interpolate HTML');
const html=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/index.html','utf8');assert.match(html,/id="failure-memory"[^>]*aria-live="polite"[^>]*hidden/);assert.match(html,/<details><summary>실패 접근 기억 보기<\/summary>/);
console.log('PASS: failure memory phases, finite TTL, sensor hold, operator priority, finished/missing status, safe read-only rendering.');
