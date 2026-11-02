// Check that the compact pre-arm guide only reports ready for live backend gates.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const app=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/app.js','utf8');
const html=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/index.html','utf8');
const source=app.slice(app.indexOf('  function explainReadiness('),app.indexOf('  function renderDispatchReadiness('));
const elements=new Map();
function element(){return{dataset:{},textContent:'',children:[],replaceChildren(){this.children=[];},append(...items){this.children.push(...items);}};}
const $=selector=>{if(!elements.has(selector))elements.set(selector,element());return elements.get(selector);};
const sandbox={$,document:{createElement:element},initialized:true,disconnected:false,
 currentMission:()=>sandbox.snapshot.mission,snapshot:null,Math,Number,String,Array};
vm.createContext(sandbox);vm.runInContext(source+'\nglobalThis.render=renderPrearmOverview;',sandbox);
function fixture(){return{connection:{connected:true,domain_id:16,use_sim_time:false},
 map:{preview:false,configured_match:true},health:{ready:true,missing:[],map:{configured_match:true}},
 robot:{pose:{x:0,y:0,yaw:0},pose_age:.1,scan_age:.1,odom_age:.1},
 localization_health:{autonomous_allowed:true},mission:{state:'IDLE',estop_latched:false},safety:{latched:false,pending:null}};}
sandbox.snapshot=fixture();sandbox.render();
assert.match(html,/class="prearm-breakdown"/,'full details stay available without crowding the main screen');
assert.equal($('#prearm-overview').dataset.state,'ready');
assert.match($('#prearm-summary').textContent,/출발 조건 충족/);
assert.equal($('#prearm-checks').children.length,5);
sandbox.snapshot.mission.estop_latched=true;sandbox.snapshot.safety.latched=true;sandbox.render();
assert.equal($('#prearm-overview').dataset.state,'waiting','E-stop cannot be hidden by healthy sensors');
assert.match($('#prearm-summary').textContent,/정지 유지/);
sandbox.snapshot=fixture();sandbox.snapshot.health.ready=false;sandbox.snapshot.health.missing=['위치 신뢰도: 위치 추정 오차 범위가 큼'];sandbox.snapshot.localization_health.autonomous_allowed=false;sandbox.render();
assert.equal($('#prearm-overview').dataset.state,'waiting');
assert.match($('#prearm-checks').children[2].children[1].textContent,/위치 추정 오차 범위/);
sandbox.snapshot=fixture();sandbox.snapshot.health.ready=false;sandbox.snapshot.health.missing=['최신 유효한 /battery_state 전압 (offline)'];sandbox.render();
assert.equal($('#prearm-overview').dataset.state,'waiting');
assert.match($('#prearm-checks').children[3].children[1].textContent,/battery_state/);
sandbox.snapshot=fixture();sandbox.disconnected=true;sandbox.render();
assert.equal($('#prearm-overview').dataset.state,'waiting','stale data after web disconnect cannot appear ready');
console.log('PASS: compact pre-arm guide mirrors live health, battery, localization, E-stop, and connection gates.');
