const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const app=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/app.js','utf8');
const controls=app.slice(app.indexOf('  function renderControls(){'),app.indexOf('  function renderStatus(){'));
const status=app.slice(app.indexOf('  function renderStatus(){'),app.indexOf('  const requestStates='));
const els=new Map();
const element=()=>({disabled:false,hidden:false,checked:true,textContent:'',title:'',classList:{toggle(){}},replaceChildren(){},append(){}});
const $=sel=>{if(!els.has(sel))els.set(sel,element());return els.get(sel);};
const sandbox={$,document:{querySelectorAll:()=>[],createElement:element},ManualJoystick:{setState(){}},VehiclePower:{render(){}},initialized:true,disconnected:false,busy:false,localizationStarting:false,draft:[1],dirty:false,lastPath:[],terminal:new Set(['IDLE','COMPLETED','CANCELED','ERROR','HOME_REACHED']),states:{IDLE:'임무 대기',PAUSED:'일시정지',ESTOP:'비상정지'},currentMission:()=>sandbox.snapshot.mission,delivery:()=>sandbox.snapshot.delivery,homepoint:()=>null,markerLabel:id=>'Sector '+id,Math,Date,Number,String,Array};
vm.createContext(sandbox);vm.runInContext(controls+'\n'+status+'\nglobalThis.check=()=>{renderControls();renderStatus();};',sandbox);
function fixture(state='IDLE',mission='IDLE') {return{connection:{connected:true},health:{ready:true},manual:{state,available:true,owner:state!=='IDLE',active:['ACTIVE','ACQUIRING'].includes(state)},mission:{state:mission,estop_latched:false,can_return_home:true,current_index:0,current_marker_id:1,route:[1],completed_ids:[]},delivery:{active:false,home_ready:true,pending_count:1},robot:{speed:{linear:0,angular:0},pose:{x:0,y:0},pose_age:.1,scan_age:.1,odom_age:.1},events:[]};}
for(const phase of ['IDLE','ACQUIRING','ACTIVE','RELEASING','WAIT_REPLAN']) {
 sandbox.snapshot=fixture(phase);sandbox.check();const locked=phase!=='IDLE';
 for(const id of ['#start-mission','#dispatch-start','#set-home','#delivery-resume','#confirm-replace','#domain-input','#sim-time-input','#connection-form button[type=submit]'])assert.equal($(id).disabled,locked,id+' '+phase);
 assert.equal($('#return-home').disabled,false,'explicit home stays available '+phase);assert.equal($('#emergency-stop').disabled,false,'estop stays available '+phase);
 if(['ACQUIRING','ACTIVE'].includes(phase))assert.equal($('#state-pill').textContent,'수동 우선');
 if(['RELEASING','WAIT_REPLAN'].includes(phase))assert.equal($('#state-pill').textContent,'임무 재개 준비');
}
sandbox.snapshot=fixture('ACTIVE','PAUSED');sandbox.check();assert.equal($('#pause-mission').disabled,false,'explicit pause must interrupt manual auto-resume');
sandbox.snapshot.mission.estop_latched=true;sandbox.check();assert.equal($('#state-pill').textContent,'비상정지','estop overrides manual badge');
console.log('PASS: operator controls locked during every manual phase; home/pause/estop retained; hold/resume badges correct; estop takes display priority.');
