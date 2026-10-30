// Exercise actual guide rendering/handlers; no ROS, network or vehicle I/O.
const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const source=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/app.js','utf8');
const controls=source.slice(source.indexOf('  function renderControls(){'),source.indexOf('  function renderLocalization(){'));
const actions=source.slice(source.indexOf("  $('#localization-start').addEventListener"),source.indexOf("  $('#dispatch-start').addEventListener"));
const safety=source.slice(source.indexOf("  $('#emergency-stop').addEventListener"),source.indexOf("  $('#confirm-replace').addEventListener"));
const elements=new Map(),listeners=new Map(),calls=[],pending=[];
let reads=0,releases=0;
function element(){return{disabled:false,hidden:false,checked:false,textContent:'',title:'',value:0,dataset:{},attributes:{},children:[],classList:{toggle(){}},
 setAttribute(key,value){this.attributes[key]=value;},replaceChildren(){this.children=[];},append(child){this.children.push(child);},
 scrollIntoView(){this.scrolled=true;},focus(){this.focused=true;},showModal(){this.open=true;},close(){this.open=false;},
 addEventListener(name,handler){listeners.set(this.selector+':'+name,handler);},
 click(){if(!this.disabled){const result=listeners.get(this.selector+':click')?.();if(result?.then)pending.push(result);}}};}
const $=selector=>{if(!elements.has(selector)){const item=element();item.selector=selector;elements.set(selector,item);}return elements.get(selector);};
const sandbox={$,document:{querySelectorAll:()=>[],createElement:element},ManualJoystick:{setState(){},release(){releases++;}},
 initialized:true,disconnected:false,busy:false,localizationStarting:false,draft:[1],dirty:false,
 terminal:new Set(['IDLE','COMPLETED','CANCELED','ERROR','HOME_REACHED']),
 currentMission:()=>sandbox.snapshot.mission,delivery:()=>sandbox.snapshot.delivery,
 async action(path,body){calls.push({path,body,starting:sandbox.localizationStarting});return true;},
 async api(path,body){calls.push({path,body});return{};},async refresh(){reads++;},notify(){}};
vm.createContext(sandbox);vm.runInContext(controls+'\n'+actions+'\n'+safety+'\nglobalThis.render=renderControls;',sandbox);
function fixture(){return{connection:{connected:true,domain_id:16,use_sim_time:false},
 mission:{state:'IDLE',estop_latched:false},manual:{state:'IDLE'},map:{preview:false},
 health:{ready:false,missing:['위치 신뢰도: 초기 위치 안정 확인 중'],map:{configured_match:true,configured_file:'/maps/classroom_20261027_cleaned.yaml'}},
 localization_health:{level:'warning',reasons:['initializing','position_std_m_elevated'],allow_localization:true,autonomous_allowed:false},
 localization:{state:'IDLE',active:false,available:true,startup_home_available:true,home_seed_available:false,reason:'초기 위치 안정 확인 중'},
 safety:{latched:false,stop_acknowledged:true},robot:{pose:{x:-.11,y:-.79},pose_age:.1,scan_age:.1,odom_age:.1,speed:{linear:0,angular:0}},
 delivery:{active:false,pending_count:1,home_ready:true,registered_homepoint:{x:-.11,y:-.79,yaw:0}}};}
async function click(id){$(id).click();while(pending.length)await pending.shift();}
function confirmHome(){ $('#startup-confirm-home').checked=true;listeners.get('#startup-confirm-home:change')(); }
async function run(){
 sandbox.snapshot=fixture();$('#startup-recovery').hidden=true;sandbox.render();
 await click('#startup-recovery-open');
 assert.equal($('#startup-recovery').hidden,false);assert.equal($('#startup-recovery').focused,true);
 assert.equal($('#startup-recovery-open').attributes['aria-expanded'],'true');
 assert.equal(calls.length,0,'opening the guide must not send a command');assert.equal(releases,0,'opening does not alter joystick ownership');
 assert.equal($('#startup-localize-home').disabled,true);assert.equal($('#startup-seed-home').disabled,true);
 await click('#localization-start-home');assert.equal(calls.length,0,'the existing Home button also requires actual-home confirmation');
 confirmHome();assert.equal($('#confirm-home-location').checked,true);assert.equal($('#startup-localize-home').disabled,false);
 await click('#startup-localize-home');assert.deepEqual(calls.map(item=>item.path),['/api/localization/start-home']);assert.equal(calls[0].body.confirm_actual_home,true);assert.equal(calls[0].starting,true);

 calls.length=0;sandbox.snapshot=fixture();sandbox.snapshot.mission.state='ESTOP';sandbox.snapshot.mission.estop_latched=true;
 sandbox.snapshot.safety.latched=true;sandbox.snapshot.localization.home_seed_available=true;
 sandbox.snapshot.localization.home_seed_reason='등록 홈의 실제 라이다 일치 확인';sandbox.render();await click('#startup-recovery-open');
 assert.equal($('#startup-seed-home').disabled,true);confirmHome();await click('#startup-seed-home');
 assert.equal(calls.length,1);assert.equal(calls[0].path,'/api/localization/seed-home');assert.equal(calls[0].body.confirm_actual_home,true);
 assert.equal(sandbox.snapshot.mission.estop_latched,true,'pose prior is not an estop reset');
 await click('#startup-open-reset');assert.equal($('#reset-dialog').open,true);assert.equal(calls.length,1,'opening reset must not reset or rotate');
 assert.equal($('#startup-confirm-home').checked,false,'opening reset clears prior Home consent');
 await click('#confirm-reset');assert.equal(calls.length,2);assert.equal(calls[1].path,'/api/emergency-stop');assert.equal(calls[1].body.reset,true);
 assert.equal($('#reset-dialog').open,false);assert.equal(calls.filter(item=>item.path.includes('localization/start')).length,0,'reset never starts localization');

 calls.length=0;sandbox.snapshot=fixture();sandbox.disconnected=true;sandbox.render();
 assert.match($('#startup-recovery-summary').textContent,/통신 확인/);assert.match($('#startup-inputs-detail').textContent,/통신 수신 중단/);
 assert.equal($('#startup-localize-home').disabled,true);assert.equal($('#startup-localize-general').disabled,true);assert.equal($('#startup-open-reset').disabled,true);
 assert.equal($('#startup-confirm-home').checked,false);await click('#startup-localize-general');assert.equal(calls.length,0);
 sandbox.disconnected=false;sandbox.snapshot=fixture();sandbox.snapshot.robot.odom_age=1.2;sandbox.render();
 assert.match($('#startup-stop-detail').textContent,/최신 odom/);assert.equal($('#startup-localize-general').disabled,true);
 sandbox.snapshot=fixture();sandbox.snapshot.localization_health.reasons=['odom_tf_unavailable','initializing'];sandbox.snapshot.localization_health.explanations=['차체 오도메트리 TF 대기'];sandbox.render();
 assert.match($('#startup-inputs-detail').textContent,/차체 오도메트리 TF 대기/);assert.equal($('#startup-step-inputs').dataset.state,'waiting');
 sandbox.snapshot=fixture();sandbox.snapshot.localization.startup_home_available=false;sandbox.snapshot.localization.startup_home_reason='360도 라이다와 30cm 회전 공간을 확인하세요.';sandbox.render();confirmHome();
 assert.equal($('#startup-localize-home').disabled,true);assert.match($('#startup-localize-detail').textContent,/30cm 회전 공간/);
 sandbox.snapshot=fixture();sandbox.snapshot.manual.state='ACTIVE';sandbox.render();assert.equal($('#startup-localize-home').disabled,true);assert.equal($('#startup-localize-general').disabled,true);assert.match($('#startup-stop-detail').textContent,/조이스틱/);
 sandbox.snapshot=fixture();sandbox.render();confirmHome();assert.equal($('#startup-localize-home').disabled,false);
 sandbox.snapshot.manual.state='ACTIVE';sandbox.render();assert.equal($('#startup-confirm-home').checked,false);assert.equal($('#confirm-home-location').checked,false);
 sandbox.snapshot.manual.state='IDLE';sandbox.render();assert.equal($('#startup-localize-home').disabled,true,'manual release cannot restore old Home consent');
 for(const change of [data=>data.robot.speed.linear=.12,data=>data.robot.speed.angular=.2,data=>data.robot.odom_age=.7,data=>data.robot.scan_age=.7]){
  sandbox.snapshot=fixture();sandbox.render();confirmHome();change(sandbox.snapshot);sandbox.render();
  assert.equal($('#startup-confirm-home').checked,false,'movement or lost fresh evidence invalidates Home consent');
  sandbox.snapshot.robot.speed={linear:0,angular:0};sandbox.snapshot.robot.odom_age=sandbox.snapshot.robot.scan_age=.1;sandbox.render();
  assert.equal($('#startup-localize-home').disabled,true,'stopping again does not restore earlier physical-location confirmation');
  assert.equal($('#seed-home-pose').disabled,true);
 }
 sandbox.snapshot=fixture();sandbox.render();confirmHome();sandbox.snapshot.mission.state='NAVIGATING';sandbox.render();assert.equal($('#startup-confirm-home').checked,false);
 sandbox.snapshot=fixture();sandbox.snapshot.mission.state='NAVIGATING';sandbox.render();assert.equal($('#startup-recovery-open').hidden,true);assert.equal($('#startup-localize-general').disabled,true);

 sandbox.snapshot=fixture();sandbox.snapshot.localization={state:'ROTATING',active:true,available:true,progress:.4,control:{state:'WAIT_HEALTH'},reason:'위치 감시 응답을 기다리며 정지합니다.'};sandbox.render();
 assert.equal($('#startup-stop-localization').hidden,false);assert.equal($('#startup-stop-localization').disabled,false);
 assert.equal($('#startup-localize-progress').value,.4);assert.match($('#startup-localize-detail').textContent,/위치 감시 응답/);
 sandbox.snapshot.robot.speed.angular=.2;sandbox.render();assert.equal($('#startup-confirm-home').checked,false);assert.equal(sandbox.snapshot.localization.state,'ROTATING','clearing Home consent never cancels an accepted sweep');
 sandbox.busy=true;sandbox.render();assert.equal($('#startup-stop-localization').disabled,false);await click('#startup-stop-localization');assert.equal(calls.at(-1).path,'/api/localization/stop');sandbox.busy=false;
 calls.length=0;sandbox.snapshot=fixture();sandbox.snapshot.localization.state='SUCCEEDED';sandbox.snapshot.health.ready=true;sandbox.render();
 assert.match($('#startup-recovery-summary').textContent,/완료.*자동 출발하지 않습니다/);assert.equal(calls.length,0,'success must not start or resume any mission');
 await click('#startup-recheck');assert.equal(reads,2,'state recheck and earlier stop refresh are reads');assert.equal(calls.length,0);
 confirmHome();sandbox.snapshot.delivery.registered_homepoint.x=.2;sandbox.render();assert.equal($('#startup-confirm-home').checked,false,'changed Home invalidates physical-location consent');
 await click('#startup-recovery-close');assert.equal($('#startup-recovery').hidden,true);assert.equal($('#startup-recovery-open').attributes['aria-expanded'],'false');
 const html=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/index.html','utf8'),css=fs.readFileSync('ros2_ws/src/smart_vision_amr/web/styles.css','utf8');
 assert.match(html,/id="startup-recovery"[^>]*tabindex="-1"[^>]*hidden/);assert.match(html,/id="startup-confirm-home"[^>]*type="checkbox"/);
 assert.match(css,/@media\(max-width:650px\).*startup-recovery-steps\{grid-template-columns:minmax\(0,1fr\)/);
 assert.doesNotMatch(actions+safety,/\.innerHTML/);
 console.log('PASS: guide opens without actuation, live blocked-step reasons, Home consent, separate seed/reset/localize actions, fresh-stop/manual/disconnect interlocks and no automatic departure.');
}
run().catch(error=>{console.error(error);process.exitCode=1;});
