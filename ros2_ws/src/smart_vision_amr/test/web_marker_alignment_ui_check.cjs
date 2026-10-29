// Render the actual alignment panel without ROS, network I/O or motor commands.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const source=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/app.js','utf8');
const render=source.slice(source.indexOf('  function renderMarkerAlignment('),source.indexOf('  function renderStatus(){'));
const elements=new Map();
function $(selector){if(!elements.has(selector)){const classes=new Set();elements.set(selector,{hidden:false,textContent:'',classList:{toggle(name,value){if(value)classes.add(name);else classes.delete(name);},contains:name=>classes.has(name)}});}return elements.get(selector);}
const sandbox={$,Number,String,Math,currentMission:()=>sandbox.snapshot.mission,delivery:()=>sandbox.snapshot.delivery||{}};
vm.createContext(sandbox);vm.runInContext(render+'\nglobalThis.render=renderMarkerAlignment;',sandbox);
function fixture(state){return{mission:{state:'CONFIRMING',current_marker_id:2},delivery:{active:true},manual:{state:'IDLE'},marker_alignment:{state,reason:'실제 정렬 상태',camera:'front',marker_id:2,center_error:.2,frames:1,required:3,max_seconds:45}};}
for(const [state,expected] of [['SEARCHING','저속 회전'],['CENTERING','중앙 정렬'],['SETTLING','안정 확인'],['CONFIRMED','확인 완료'],['FAILED','일시정지']]){
 sandbox.snapshot=fixture(state);assert.equal(sandbox.render(),true);assert.equal($('#marker-alignment').hidden,false);
 assert.match($('#marker-alignment-phase').textContent,state==='SEARCHING'?/저속 회전/:new RegExp(expected));
 assert.equal($('#marker-alignment-camera').textContent,'전방 카메라 · ID 2');
 assert.match($('#marker-alignment-metrics').textContent,/오른쪽 10\.0%.*영상 폭 기준.*1 \/ 3 프레임/);
 if(['SEARCHING','CENTERING','SETTLING'].includes(state))assert.match($('#marker-alignment-detail').textContent,/45초.*일시정지.*조이스틱.*비상정지/);
 if(state==='FAILED')assert.match($('#marker-alignment-detail').textContent,/배송 재개.*자동 출발하지 않습니다/);
}
sandbox.snapshot=fixture('SETTLING');sandbox.snapshot.marker_alignment.center_error=0;sandbox.snapshot.marker_alignment.required_frames=4;delete sandbox.snapshot.marker_alignment.required;sandbox.render();assert.match($('#marker-alignment-metrics').textContent,/중심 오차 0\.0%.*1 \/ 4/);
sandbox.snapshot.marker_alignment.center_error=null;sandbox.render();assert.match($('#marker-alignment-metrics').textContent,/측정 대기/);assert.doesNotMatch($('#marker-alignment-metrics').textContent,/0\.0%/,'missing error must not look centered');
sandbox.snapshot=fixture('CENTERING');sandbox.snapshot.marker_alignment.camera='rear';sandbox.snapshot.marker_alignment.center_error=-.1;sandbox.render();assert.match($('#marker-alignment-camera').textContent,/후방/);assert.match($('#marker-alignment-metrics').textContent,/왼쪽 5\.0%/);
sandbox.snapshot=fixture('FAILED');sandbox.snapshot.mission.state='PAUSED';sandbox.snapshot.delivery.active=false;sandbox.render();assert.match($('#marker-alignment-detail').textContent,/수동 임무 재개/);
sandbox.snapshot=fixture('CENTERING');sandbox.snapshot.manual.state='ACTIVE';sandbox.render();assert.match($('#marker-alignment-phase').textContent,/수동 조종 우선/);assert.match($('#marker-alignment-detail').textContent,/조이스틱이 우선/);
sandbox.snapshot.mission.estop_latched=true;sandbox.render();assert.match($('#marker-alignment-phase').textContent,/비상정지/);assert.match($('#marker-alignment-detail').textContent,/해제한 뒤 임무를 별도로 재개/);
sandbox.snapshot=fixture('CENTERING');sandbox.snapshot.mission.state='PAUSED';sandbox.render();assert.match($('#marker-alignment-phase').textContent,/일시정지/);assert.doesNotMatch($('#marker-alignment-detail').textContent,/저속 회전합니다/);
for(const change of [x=>delete x.marker_alignment,x=>x.marker_alignment.state='IDLE',x=>x.marker_alignment.state='UNRECOGNIZED',x=>x.mission.current_marker_id=3,x=>x.mission.state='NAVIGATING']){sandbox.snapshot=fixture('CENTERING');change(sandbox.snapshot);assert.equal(sandbox.render(),false);assert.equal($('#marker-alignment').hidden,true);}
sandbox.snapshot=fixture('SEARCHING');assert.equal(sandbox.render({homeMode:true}),false);
const html=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/index.html','utf8');assert.match(html,/id="marker-alignment"[^>]*aria-live="polite"/);assert.ok(html.indexOf('id="marker-alignment"')>html.indexOf('id="confirmation-status"'));
assert.doesNotMatch(render,/\.innerHTML|\bfetch\(|\baction\(|\bapi\(/,'alignment display must never actuate or interpolate HTML');
console.log('PASS: marker phases, selected camera, normalized error units, stable frame count, timeout/resume guidance, missing data, stale target and manual/estop priority.');
