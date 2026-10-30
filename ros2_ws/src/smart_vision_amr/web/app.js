'use strict';
(() => {
  const $ = selector => document.querySelector(selector);
  const states = {IDLE:'임무 대기',NAVIGATING:'마커로 이동',MOVING:'마커로 이동',WAIT_REPLAN:'재계획 대기',CONFIRMING:'표식 확인 중',ALIGNING:'관측 방향 정렬',WAIT_MARKER:'표식 확인 대기',PAUSED:'일시정지',COMPLETED:'임무 완료',CANCELED:'임무 취소',ERROR:'임무 확인 필요',ESTOP:'비상정지',STARTING:'시작 준비',RETURNING_HOME:'홈 복귀 중',HOME_REACHED:'홈 도착'};
  const terminal = new Set(['IDLE','COMPLETED','CANCELED','ERROR','HOME_REACHED']);
  const ns = 'http://www.w3.org/2000/svg';
  let snapshot = null, draft = [], dirty = false, initialized = false, busy = false;
  let lastPath = [], mapImageUrl = '', mapVersion = null, toastTimer = null, cameraLoading = false, frontCameraLoading = false, disconnected = false;
  let mapLayout = null, mapLayoutLoading = false, mapLayoutGeneration = 0, showRawMap = false;
  let pollRunning = false, cameraTimer = null, editorKey = '', deliveryKey = '', previousTripId = null;
  let accessMode = 'code';
  let localizationStarting = false;
  const svg = (name, attrs = {}, text = '') => { const el = document.createElementNS(ns,name); for(const [key,value] of Object.entries(attrs)) el.setAttribute(key,value); if(text) el.textContent=text; return el; };
  const icon = name => { const el=svg('svg',{class:'icon','aria-hidden':'true'}); el.append(svg('use',{href:'#'+name})); return el; };
  const same = (a,b) => JSON.stringify(a) === JSON.stringify(b);
  function notify(message,error=false){clearTimeout(toastTimer);$('#toast').textContent=message;$('#toast').classList.toggle('error',error);$('#toast').hidden=false;toastTimer=setTimeout(()=>$('#toast').hidden=true,5500);}
  function showPair(){ManualJoystick.release();if(!$('#pair-dialog').open) $('#pair-dialog').showModal();}
  function setAccessMode(mode){
    accessMode=mode==='domain'?'domain':'code';const domain=accessMode==='domain',input=$('#pair-code-input');
    $('#pair-title').textContent=domain?'로봇 운용자 연결':'PC에 연결';
    $('#pair-intro').textContent=domain?'로봇과 같은 ROS Domain ID를 입력하세요. 연결 후 로봇 운용 기능을 사용할 수 있습니다.':'웹 서버를 실행한 PC에 표시된 접속 코드를 입력하세요.';
    $('#pair-credential-label').textContent=domain?'ROS Domain ID':'접속 코드';
    input.type=domain?'number':'text';input.autocomplete=domain?'off':'one-time-code';
    if(domain){input.min='0';input.max='232';input.step='1';input.removeAttribute('maxlength');}
    else{input.removeAttribute('min');input.removeAttribute('max');input.removeAttribute('step');input.maxLength=16;}
    $('#pairing-code-row').hidden=domain;$('#inventory-code-row').hidden=domain;
    $('#phone-domain-row').hidden=!domain;$('#inventory-domain-row').hidden=!domain;
  }
  async function api(path,body){
    const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),12000);
    try {
      const response=await fetch(path,{method:body===undefined?'GET':'POST',headers:body===undefined?{}:{'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body),cache:'no-store',signal:controller.signal,credentials:'same-origin'});
      const data=await response.json();
      if(!response.ok){if(data.pairing_required)showPair();throw new Error(data.error||`요청 실패 (${response.status})`);}
      return data;
    } finally {clearTimeout(timer);}
  }
  async function action(path,body={},message=''){
    ManualJoystick.release();
    if(busy)return false;busy=true;renderControls();
    try{await api(path,body);if(message)notify(message);await refresh();return true;}
    catch(error){notify(error.name==='AbortError'?'응답이 지연되고 있습니다. 차량 상태와 최근 기록을 확인하세요.':error.message,true);return false;}
    finally{busy=false;renderControls();}
  }
  function currentMission(){return snapshot?.mission||{state:'IDLE',route:[],pending_route:null,current_index:0,completed_ids:[]};}
  function isPhysicalClassroom(){
    const connection=snapshot?.connection||{},configured=String(snapshot?.health?.map?.configured_file||'');
    return Number(connection.domain_id)===16&&!connection.use_sim_time&&configured.endsWith('/classroom_20261027_cleaned.yaml');
  }
  function delivery(){return snapshot?.delivery||{homepoint:null,active:false,home_ready:false,requests:[],pending_count:0};}
  function homepoint(){return delivery().homepoint||currentMission().home;}
  function savedRoute(){const mission=currentMission();return mission.pending_route??mission.route??[];}
  function markerById(id){return (snapshot?.markers||[]).find(marker=>Number(marker.id)===Number(id));}
  // Display names come from the selected map's marker configuration, not mission order.
  function markerLabel(id){return String(markerById(id)?.name||'').trim()||`ArUco ${id}`;}
  function routeDescription(index){
    const mission=currentMission(),sameActive=same(draft,mission.route||[]);
    if(!sameActive||dirty)return index===draft.length-1?'마지막 방문':'방문 위치';
    if(index<(mission.completed_ids||[]).length)return '표식 확인 완료';
    if(!mission.returning_home&&index===mission.current_index&&!terminal.has(mission.state)&&mission.state!=='ESTOP')return states[mission.state]||mission.state;
    return index===draft.length-1?'마지막 방문':'방문 위치';
  }
  function renderEditor(){
    const missionState=currentMission();
    const key=JSON.stringify([draft,dirty,missionState.state,missionState.returning_home,missionState.current_index,missionState.completed_ids,missionState.pending_route,busy,delivery().active,snapshot?.markers]);
    if(editorKey===key)return;editorKey=key;
    const list=$('#route-editor');list.replaceChildren();
    draft.forEach((id,index)=>{
      const row=document.createElement('li');row.className='route-row';
      const mission=currentMission(),sameActive=same(draft,mission.route||[])&&!dirty;
      row.classList.toggle('current',sameActive&&!mission.returning_home&&index===mission.current_index&&!terminal.has(mission.state));
      row.classList.toggle('done',sameActive&&index<(mission.completed_ids||[]).length);
      const number=document.createElement('span');number.className='marker-number';number.textContent=String(id);
      const copy=document.createElement('div');copy.className='route-copy';const title=document.createElement('strong');title.textContent=markerLabel(id);title.title=`ArUco ID ${id}`;const detail=document.createElement('small');detail.textContent=`${index+1}. ${routeDescription(index)}`;copy.append(title,detail);
      const controls=document.createElement('div');controls.className='route-actions';
      [['up','위로',index===0],['down','아래로',index===draft.length-1],['close','삭제',false]].forEach(([name,label,disabled])=>{
        const button=document.createElement('button');button.type='button';button.setAttribute('aria-label',`순서 ${index+1} ${label}`);button.dataset.edgeDisabled=String(disabled);button.disabled=disabled||busy;button.append(icon(name));
        button.addEventListener('click',()=>{if(delivery().active)return;if(name==='close')draft.splice(index,1);else{const other=index+(name==='up'?-1:1);[draft[index],draft[other]]=[draft[other],draft[index]];}dirty=!same(draft,savedRoute());renderEditor();renderControls();drawMap();});controls.append(button);
      });
      row.append(number,copy,controls);list.append(row);
    });
    $('#route-empty').hidden=draft.length>0;
    $('#route-save-status').textContent=dirty?'저장하지 않은 변경':currentMission().pending_route?'새 순서 저장됨':'저장된 순서';
    $('#save-route').disabled=!initialized||!dirty||busy;
    $('#pending-note').hidden=currentMission().pending_route==null;
    $('#pending-note').textContent=missionState.returning_home?'새 순서는 홈 복귀 완료 후 다음 임무 시작에 적용됩니다. 복귀를 재개하면 기존 홈으로 이동합니다.':'새 순서가 저장되었습니다. 현재 임무를 멈춘 뒤 새 순서로 시작할 수 있습니다.';
  }
  function renderMarkerButtons(){
    const container=$('#add-markers'),ids=(snapshot?.markers||[]).map(marker=>Number(marker.id));
    const key=JSON.stringify(ids.map(id=>[id,markerLabel(id)]));
    if(container.dataset.markers===key)return;
    container.dataset.markers=key;container.replaceChildren();
    ids.forEach(id=>{const button=document.createElement('button');button.type='button';button.textContent=markerLabel(id);button.title=`ArUco ID ${id}`;button.setAttribute('aria-label',`${markerLabel(id)} 추가`);button.addEventListener('click',()=>{if(delivery().active)return;if(draft.length>=50){notify('한 임무에는 최대 50개의 마커를 등록할 수 있습니다.',true);return;}draft.push(id);dirty=!same(draft,savedRoute());renderEditor();renderControls();drawMap();});container.append(button);});
  }
  function renderControls(){
    const mission=currentMission(),paired=initialized&&!disconnected,active=!terminal.has(mission.state)&&mission.state!=='ESTOP',pending=mission.pending_route!=null,shipping=!!delivery().active,manualBusy=!!snapshot?.manual&&snapshot.manual.state!=='IDLE',localizing=localizationStarting||!!snapshot?.localization?.active;
    // A joystick press still has priority: the server ends localization before
    // granting manual control. Localization must never trap the operator input.
    ManualJoystick.setState(snapshot,{connected:paired});
    $('#start-mission').disabled=localizing||manualBusy||shipping||busy||!paired||!snapshot?.health?.ready||draft.length===0||dirty||mission.estop_latched||(active&&mission.state!=='PAUSED');
    $('#start-mission span').textContent=mission.state==='PAUSED'&&mission.returning_home?'홈 복귀 재개':mission.state==='PAUSED'&&pending?'새 순서로 시작':mission.state==='PAUSED'?'수동 임무 재개':'수동 임무 시작';
    $('#pause-mission').disabled=busy||!paired||(!active&&!localizing)||(mission.state==='PAUSED'&&!manualBusy&&!localizing);
    const canReturn=!!mission.can_return_home&&!mission.estop_latched&&!mission.returning_home;
    $('#return-home').disabled=busy||!paired||!canReturn;
    $('#cancel-mission').disabled=busy||!paired||(!active&&!localizing)||!canReturn;
    $('#emergency-stop').disabled=!paired;
    $('#manual-lock-note').hidden=!shipping;
    const d=delivery(),speed=snapshot?.robot?.speed||{};
    const stationary=Math.abs(Number(speed.linear)||0)<=0.02&&Math.abs(Number(speed.angular)||0)<=0.05;
    const homeSensorsReady=!!snapshot?.connection?.connected&&['pose_age','scan_age','odom_age'].every(key=>typeof snapshot?.robot?.[key]==='number'&&snapshot.robot[key]<2);
    $('#set-home').disabled=localizing||manualBusy||busy||!paired||shipping||!terminal.has(mission.state)||!homeSensorsReady||!snapshot?.robot?.pose||!stationary||mission.estop_latched;
    // Restoring a registered map coordinate does not depend on the current TF.
    $('#reset-home').disabled=localizing||manualBusy||busy||!paired||shipping||!terminal.has(mission.state)||mission.estop_latched||d.registered_homepoint===null;
    $('#load-confirmed').disabled=busy||!paired||shipping||!d.pending_count;
    $('#dispatch-start').disabled=localizing||manualBusy||busy||!paired||shipping||!d.home_ready||!d.pending_count||!snapshot?.health?.ready||d.dispatch_ready===false||mission.estop_latched||!$('#load-confirmed').checked;
    $('#delivery-resume').hidden=!shipping||mission.state!=='PAUSED';
    $('#delivery-resume').disabled=localizing||manualBusy||busy||!paired||!snapshot?.health?.ready||mission.estop_latched;
    const safety=snapshot?.safety||{};
    $('#emergency-stop span').textContent=mission.estop_latched&&safety.pending!=='reset'&&safety.stop_acknowledged?'비상정지 해제':mission.estop_latched&&safety.pending!=='reset'?'비상정지 다시 요청':'비상정지';
    const seed=snapshot?.localization||{};
    $('#seed-home-pose').disabled=busy||!paired||!$('#confirm-home-location').checked||!seed.home_seed_available||seed.home_seed_pending;
    $('#home-seed-note').textContent=seed.home_seed_result||seed.home_seed_reason||'등록 홈과 센서 상태를 확인하고 있습니다.';
    $('#confirm-reset').disabled=busy||!paired||!!seed.home_seed_pending||safety.pending==='reset';
    $('#confirm-replace').disabled=localizing||manualBusy||busy||!paired;
    $('#domain-input').disabled=localizing||manualBusy||busy;
    $('#sim-time-input').disabled=localizing||manualBusy||busy;
    $('#connection-form button[type=submit]').disabled=localizing||manualBusy||busy;
    $('#save-route').disabled=shipping||busy||!paired||!dirty;
    document.querySelectorAll('#add-markers button').forEach(button=>button.disabled=busy||shipping);
    document.querySelectorAll('#route-editor button').forEach(button=>button.disabled=busy||shipping||button.dataset.edgeDisabled==='true');
    $('#localization-start').disabled=localizing||manualBusy||busy||!paired||mission.estop_latched||!snapshot?.localization?.available;
    $('#localization-start-home').disabled=localizing||manualBusy||busy||!paired||mission.estop_latched||!snapshot?.localization?.startup_home_available;
    $('#localization-start-home').title=snapshot?.localization?.startup_home_reason||'실제 등록 홈에 로봇이 있을 때만 사용하세요.';
    $('#localization-stop').hidden=!snapshot?.localization?.active;
    // Stop and emergency-stop remain usable while a different request is pending.
    $('#localization-stop').disabled=!paired||!snapshot?.localization?.active;
    renderDispatchReadiness({paired,localizing,manualBusy});
    renderStartupRecovery();
    renderInitialPose();
  }
  function initialPoseValues(){
    const values=['#initial-pose-x','#initial-pose-y','#initial-pose-yaw'].map(selector=>String($(selector).value||'').trim());
    if(values.some(value=>value===''))return null;
    const [x,y,yaw_deg]=values.map(Number);
    return [x,y,yaw_deg].every(Number.isFinite)&&yaw_deg>=-180&&yaw_deg<=180?{x,y,yaw_deg}:null;
  }
  function renderInitialPose(){
    const loc=snapshot?.localization?.manual_pose||{},robot=snapshot?.robot||{},home=delivery().registered_homepoint;
    const connected=initialized&&!disconnected&&snapshot?.connection?.connected===true;
    const context=JSON.stringify([snapshot?.connection?.domain_id,snapshot?.connection?.use_sim_time,snapshot?.map_version,snapshot?.health?.map?.configured_file]);
    const panel=$('#initial-pose-panel');
    if(context!==panel.dataset.contextKey||!connected){$('#initial-pose-confirm').checked=false;panel.dataset.contextKey=context;}
    const mapMatched=snapshot?.health?.map?.configured_match!==false&&snapshot?.map?.configured_match!==false;
    const tfBlocked=(snapshot?.health?.missing||[]).includes('최신 map→base TF');
    const freshPose=connected&&mapMatched&&!tfBlocked&&Number.isFinite(robot.pose_age)&&robot.pose_age<2&&robot.pose&&[robot.pose.x,robot.pose.y,robot.pose.yaw].every(Number.isFinite);
    const confirmed=!!$('#initial-pose-confirm').checked,values=initialPoseValues();
    const available=connected&&loc.available===true&&!loc.pending&&!busy;
    $('#initial-pose-home').disabled=!connected||!home||![home.x,home.y,home.yaw].every(Number.isFinite)||busy||!!loc.pending;
    $('#initial-pose-submit').disabled=!available||!confirmed||!values;
    const state=String(loc.state||'IDLE').toUpperCase(),verified=state==='VERIFIED'&&loc.verified===true&&freshPose;
    const badge=$('#initial-pose-state');
    badge.dataset.state=verified?'verified':state==='FAILED'?'failed':'waiting';
    badge.textContent=verified?'AMCL · TF 확인됨':loc.pending||state==='WAITING'?'AMCL · TF 확인 중':state==='FAILED'?'초기 위치 미확인':state==='VERIFIED'?'TF 재확인 필요':freshPose?'현재 위치 추정 중':'초기 위치 입력 대기';
    const result=loc.pending||state==='WAITING'?'초기 위치를 보냈습니다. AMCL 위치와 최신 map→base TF 응답을 확인하는 중입니다.':
      state==='FAILED'?loc.result||'초기 위치를 확인하지 못했습니다. 센서·시계·AMCL 상태를 확인한 뒤 다시 입력하세요.':
      state==='VERIFIED'&&!freshPose?'이전에 초기 위치를 확인했지만 현재 map→base TF가 지연됩니다. /scan·/odom·AMCL을 확인하세요.':
      verified?'AMCL 위치와 최신 map→base TF를 확인했습니다. 아래 추정 좌표가 실제 로봇과 일치하는지 확인하세요. 임무는 자동으로 시작되지 않습니다.':
      !connected?'차량 연결을 확인하세요. 초기 위치 요청은 전송되지 않았습니다.':
      loc.reason||'지도에서 실제 로봇의 위치·방향을 입력하거나 등록 홈 좌표를 불러오세요.';
    $('#initial-pose-result').textContent=result;
    const pose=robot.pose;
    $('#initial-pose-observed').textContent=freshPose?`현재 추정 위치 x ${pose.x.toFixed(2)} · y ${pose.y.toFixed(2)} m · 차체 방향 ${(pose.yaw*180/Math.PI).toFixed(1)}°`:'현재 추정 위치 · 최신 map→base TF 대기';
  }
  function explainReadiness(raw){
    const reasons=Array.isArray(raw)?raw.filter(reason=>typeof reason==='string'&&reason.trim()):[];
    const mismatch=reasons.includes('설정 지도와 다른 /map');
    const robot=snapshot?.robot||{},map=snapshot?.map||{};
    const hasFreshSensors=['scan_age','odom_age'].every(key=>typeof robot[key]==='number'&&robot[key]<2);
    const physicalClassroom=Number(snapshot?.connection?.domain_id)===16&&!snapshot?.connection?.use_sim_time;
    const expected=String(snapshot?.health?.map?.configured_file||'').split('/').pop()||'설정된 저장 지도';
    return [...new Set(reasons.filter(reason=>!(mismatch&&reason==='/map')).map(reason=>{
      if(reason==='설정 지도와 다른 /map')return `설정 지도와 다른 /map 수신: SLAM·다른 지도 발행을 종료하고 ${expected}로 Nav2 위치 추정을 실행하세요.`;
      if(reason==='/map')return `운용 지도 /map 대기: ${expected}로 Nav2 위치 추정을 실행하세요.`;
      if(reason==='최신 map→base TF'){
        if(mismatch)return '최신 map→base TF 대기: 먼저 지도 불일치를 해결한 뒤 AMCL 위치 추정을 확인하세요.';
        if(!hasFreshSensors||!snapshot?.map||map.preview)return '최신 map→base TF 대기: /map·/scan·/odom 수신과 AMCL(map→odom) 실행을 확인하세요.';
        if(physicalClassroom)return '최신 map→base TF 대기: 위의 초기 위치 지정에 실제 지도 좌표와 차체 방향을 입력하세요. 차량이 실제 등록 홈에 있고 후방 카메라가 0번 마커를 볼 때만 등록 홈 좌표를 사용하세요.';
        return '최신 map→base TF 대기: 위의 초기 위치 지정에 실제 차량의 지도 좌표와 차체 방향을 입력하세요.';
      }
      return reason;
    }))];
  }
  function renderDispatchReadiness({paired,localizing,manualBusy}){
    const d=delivery(),mission=currentMission(),missing=[];
    if(!paired)missing.push('웹 연결을 확인하고 차량과 같은 Domain으로 접속하세요.');
    if(mission.estop_latched)missing.push('비상정지를 해제하세요.');
    if(localizing)missing.push('라이다 위치 확인이 끝난 뒤 지도에서 실제 위치와 방향을 확인하세요.');
    if(manualBusy)missing.push('조이스틱을 놓고 자동주행 연결이 복구될 때까지 기다리세요.');
    if(Array.isArray(d.dispatch_missing))missing.push(...d.dispatch_missing.filter(reason=>typeof reason==='string'&&reason.trim()));
    else{
      if(!d.home_ready)missing.push(d.homepoint?'홈에서 정지·대기 중인지 확인하세요. 홈 좌표가 잘못되었다면 등록된 홈으로 복원하세요.':'지도에 등록된 홈을 복원하거나 실제 홈 위치에서 Homepoint를 설정하세요.');
      if(!snapshot?.health?.ready){
        const sensors=snapshot?.health?.missing||[];
        if(sensors.length)missing.push(...sensors);
        else missing.push('차량·지도·위치 추정·카메라 연결을 확인하세요.');
      }
    }
    if(!d.active&&!d.pending_count)missing.push('접수된 재고 부족 요청이 없습니다.');
    if(!d.active&&d.pending_count&&!$('#load-confirmed').checked)missing.push('요청 물품을 적재한 뒤 위의 적재 확인란을 체크하세요.');
    if(busy)missing.push('현재 요청을 처리하고 있습니다.');
    const reasons=explainReadiness(missing),list=$('#dispatch-blockers');list.replaceChildren();
    reasons.forEach(reason=>{const item=document.createElement('li');item.textContent=reason;list.append(item);});
    const ready=reasons.length===0&&!d.active;
    $('#dispatch-readiness').classList.toggle('ready',ready);
    $('#dispatch-blockers').hidden=reasons.length===0;
    $('#dispatch-readiness-title').textContent=d.active?'배송 진행 중 · 새 요청은 자동으로 이어집니다.':ready?'출발 준비 완료 · 적재 완료 · 출발을 누르세요.':'출발 대기 · 아래 조건을 확인하세요.';
    $('#dispatch-start').title=reasons.join('\n');
  }
  function setHomeConfirmation(checked){
    $('#startup-confirm-home').checked=checked===true;
    $('#confirm-home-location').checked=checked===true;
    if(checked!==true){$('#seed-home-pose').disabled=true;$('#startup-seed-home').disabled=true;$('#startup-localize-home').disabled=true;}
  }
  function openStartupRecovery(){
    setHomeConfirmation(false);
    const panel=$('#startup-recovery');panel.hidden=false;renderControls();
    panel.scrollIntoView({behavior:'smooth',block:'start'});panel.focus({preventScroll:true});
  }
  function renderStartupRecovery(){
    const mission=currentMission(),loc=snapshot?.localization||{},robot=snapshot?.robot||{},safety=snapshot?.safety||{},quality=snapshot?.localization_health||{};
    const panel=$('#startup-recovery'),paired=initialized&&!disconnected&&snapshot?.connection?.connected===true;
    const waiting=terminal.has(mission.state)||['PAUSED','ESTOP'].includes(mission.state)||!!loc.active;
    const home=delivery().registered_homepoint,connection=snapshot?.connection||{};
    const homeKey=JSON.stringify([connection.domain_id,connection.use_sim_time,home,snapshot?.health?.map?.configured_file]);
    if(panel.dataset.homeKey!==homeKey||!paired){setHomeConfirmation(false);panel.dataset.homeKey=homeKey;}
    for(const id of ['#startup-recovery-open','#startup-recovery-open-dispatch']){
      $(id).hidden=!waiting;$(id).setAttribute('aria-expanded',String(!panel.hidden));
    }
    const fresh=(name,limit)=>typeof robot[name]==='number'&&Number.isFinite(robot[name])&&robot[name]>=0&&robot[name]<limit;
    const speed=robot.speed||{},stopped=paired&&fresh('odom_age',.5)&&Number.isFinite(speed.linear)&&Number.isFinite(speed.angular)&&Math.abs(speed.linear)<.025&&Math.abs(speed.angular)<.06;
    const latched=!!mission.estop_latched||!!safety.latched,zeroAck=paired&&safety.stop_acknowledged===true;
    const manual=!!snapshot?.manual&&snapshot.manual.state!=='IDLE';
    // A physical-location confirmation cannot survive operator motion, a
    // running mission, or loss of fresh stationary evidence. Clearing consent
    // never cancels an already accepted localization operation.
    if(!waiting||manual||!stopped||!fresh('scan_age',.5))setHomeConfirmation(false);
    const confirmed=$('#startup-confirm-home').checked;
    const inputFaults=(quality.reasons||[]).filter(reason=>/scan_unavailable|odom_unavailable|odom_tf_unavailable|map_tf_unavailable|map_unavailable|scan_map_check_unavailable/.test(reason));
    const inputs=paired&&fresh('scan_age',.5)&&fresh('odom_age',.5)&&fresh('pose_age',2)&&!!robot.pose&&snapshot?.map&&!snapshot.map.preview&&snapshot?.health?.map?.configured_match!==false&&!inputFaults.length&&(quality.allow_localization===true||snapshot?.health?.ready===true);
    const sensorReasons=explainReadiness(snapshot?.health?.missing||[]).filter(reason=>!/카메라|ArUco|위치 신뢰도|위치 확인 완료/.test(reason));
    const step=(name,ready,label,detail)=>{
      $('#startup-step-'+name).dataset.state=ready?'ready':'waiting';
      $('#startup-'+name+'-state').textContent=label;
      $('#startup-'+name+'-detail').textContent=detail;
    };
    step('stop',stopped&&!loc.active&&!manual,loc.active?'위치 확인 진행 중':manual?'조이스틱 우선':stopped?'실제 odom 정지 확인':'정지 확인 대기',
      !paired?'차량 통신이 연결되지 않아 정지를 확인할 수 없습니다. 연결과 로봇 실행 상태를 확인하세요.':manual?'조이스틱을 놓고 제어권 반환을 기다리세요.':loc.active?'저속 위치 확인 중입니다. 필요하면 아래 회전 중지를 누르세요.':
      (stopped?'최신 odom에서 차체 정지를 확인했습니다. ':'최신 odom의 선속도·각속도가 정지 상태인지 확인해야 합니다. ')+(zeroAck?'속도 제어기의 0 명령 발행 응답도 확인했습니다.':'0 명령 발행 응답은 별도로 확인 중입니다. 비상정지가 필요하면 하단 비상정지를 사용하세요.'));
    step('inputs',!!inputs,inputs?'입력 확인':'입력 확인 대기',!paired?'통신 수신 중단 · 연결 설정과 PC·로봇의 실행 상태를 확인하세요.':
      inputFaults.length?'위치 감시 입력 대기: '+(quality.explanations||inputFaults).join(' · ')+'. 센서·좌표 변환·두 장치의 시계를 확인하세요.':
      !fresh('scan_age',.5)||!fresh('odom_age',.5)?'최신 라이다·오도메트리 수신 대기 · 통신과 센서 시각을 확인하세요.':
      sensorReasons.length?sensorReasons.join(' · '):inputs?'라이다·오도메트리·운용 지도 응답을 확인했습니다. 위치 신뢰도와 회전 허용은 다음 단계에서 별도로 확인합니다.':'운용 지도와 좌표 변환을 기다립니다. 등록 홈이 맞다면 아래의 초기 위치 입력 조건을 확인하세요.');
    $('#startup-home-position').textContent=home&&['x','y','yaw'].every(key=>Number.isFinite(home[key]))?`등록 홈 x ${home.x.toFixed(2)} · y ${home.y.toFixed(2)} m · 차체 방향 ${(home.yaw*180/Math.PI).toFixed(1)}°`:'등록 홈 좌표를 확인하고 있습니다.';
    $('#startup-confirm-home').disabled=!waiting||!paired||!stopped||!fresh('scan_age',.5)||manual||!!loc.active||busy;
    step('home',confirmed,confirmed?'실제 홈 확인됨':'직접 확인 필요',!confirmed?'홈이 아니거나 위치·방향이 확실하지 않으면 체크하지 마세요. 위의 초기 위치 지정에 실제 지도 좌표를 직접 입력할 수 있습니다.':
      loc.home_seed_pending?'등록 홈 초기 위치 입력 검증 중 · 비상정지는 유지됩니다.':loc.home_seed_result||
      (latched?loc.home_seed_reason||'홈 초기 위치만 입력할 조건을 확인합니다.':'홈 위치 입력은 비상정지 중 복구용입니다. 해제 상태에서는 5단계 자세 초기화를 사용하세요.'));
    $('#startup-seed-home').disabled=!waiting||!paired||!stopped||!confirmed||$('#seed-home-pose').disabled;
    step('reset',paired&&!latched,!paired?'정지 제어 응답 대기':latched?safety.pending==='reset'?'해제 검증 중':'비상정지 유지':'해제 상태 · 자동 출발 없음',
      !paired?'통신 복구 후 차량의 정지 응답을 확인하세요.':latched?(safety.result||'주변 안전과 입력 준비 상태를 확인한 뒤 해제 창에서 직접 확인·해제를 누르세요.')+' 홈 초기 위치 입력만으로 해제되지 않습니다.':'비상정지가 해제되어 있습니다. 위치 확인과 임무 출발은 별도 요청이 필요합니다.');
    $('#startup-open-reset').disabled=!paired||busy||!latched||!zeroAck||!!loc.home_seed_pending||!!safety.pending;
    const complete=paired&&loc.state==='SUCCEEDED'&&snapshot?.health?.ready===true&&!latched;
    step('localize',complete,loc.active?'진행 중':complete?'위치 확인 완료':loc.state==='FAILED'||loc.state==='CANCELED'?'중단 · 원인 확인':'명시적 시작 필요',
      loc.active?loc.reason||'정지 응답과 전용 회전 제어권을 확인하고 있습니다.':complete?'지도상의 위치·방향을 실제 차량과 비교하세요. 적재 확인 후 출발 또는 재개를 직접 누르세요.':
      latched?'비상정지 해제를 먼저 확인하세요. 자동으로 해제하거나 회전하지 않습니다.':
      !paired?'차량 통신 복구가 먼저 필요합니다.':manual?'조이스틱을 놓고 제어권 반환을 기다리세요.':
      ['FAILED','CANCELED'].includes(loc.state)?(loc.reason||'위치 확인이 완료되지 않았습니다.')+' 원인을 해결한 뒤 다시 시작하세요.':
      !loc.startup_home_available?loc.startup_home_reason||loc.reason||'위치 확인 서비스와 회전 공간을 확인하세요.':'실제 등록 홈을 확인하고 자세 초기화를 시작하세요. 홈이 아니라면 일반 위치 확인을 선택하세요.');
    $('#startup-localize-home').disabled=!waiting||!paired||!stopped||!confirmed||$('#localization-start-home').disabled;
    $('#startup-localize-general').disabled=!waiting||!paired||!stopped||$('#localization-start').disabled;
    $('#startup-stop-localization').hidden=!loc.active;
    $('#startup-stop-localization').disabled=$('#localization-stop').disabled;
    $('#startup-localize-progress').hidden=!loc.active;
    $('#startup-localize-progress').value=Number.isFinite(loc.progress)?Math.max(0,Math.min(1,loc.progress)):0;
    $('#startup-recheck').disabled=busy;
    $('#startup-recovery-summary').textContent=!paired?'차량 통신 확인이 먼저 필요합니다. 초기화 동작을 보내지 않았습니다.':!waiting?'현재 임무가 진행 중입니다. 이 안내에서 초기화하지 말고 먼저 임무 상태를 확인하세요.':
      loc.active?'위치 확인 중 · 현재 단계와 중단 사유를 아래에서 확인하세요.':complete?'위치 확인 완료 · 자동 출발하지 않습니다. 출발 조건과 적재를 확인하세요.':
      !stopped?'1단계 · 실제 차량 정지와 최신 odom을 확인하세요.':!inputs?'2단계 · 센서·지도·좌표 변환 대기 이유를 확인하세요.':latched?'4단계 · 필요하면 홈 초기 위치를 지정한 뒤 비상정지를 명시적으로 해제하세요.':'3·5단계 · 실제 홈 여부를 확인한 뒤 위치 확인을 시작하세요.';
  }
  function renderLocalization(){
    const localization=snapshot?.localization||{},active=!!localization.active,state=localization.state||'IDLE';
    const labels={IDLE:'위치 확인 대기',ACQUIRING:'회전 준비',INITIALIZING:'지도 전체 위치 후보 탐색',ROTATING:'저속 회전 · 위치 추정',RETURNING:'등록 홈 방향 복귀',SETTLING:'정지 · 추정 결과 확인',SUCCEEDED:'위치 추정 완료',FAILED:'위치 추정 실패',CANCELED:'위치 확인 중지'};
    const waitingHealth=active&&localization.control?.state==='WAIT_HEALTH';
    const label=$('#localization-state');label.textContent=waitingHealth?'위치 감시 복구 대기 · 정지':labels[state]||'위치 확인 상태 점검';label.classList.toggle('active',active);label.classList.toggle('success',state==='SUCCEEDED');label.classList.toggle('error',state==='FAILED');
    $('#localization-progress').hidden=!active;$('#localization-progress').value=Number.isFinite(localization.progress)?Math.max(0,Math.min(1,localization.progress)):0;
    const fallback=state==='SUCCEEDED'?'지도에 표시된 위치와 실제 위치가 맞는지 확인하세요. 임무 출발은 별도로 눌러주세요.':active?'주변 라이다와 저장 지도를 대조하고 있습니다.':'차량·저장 지도·위치 추정 노드를 실행한 뒤 사용할 수 있습니다.';
    $('#localization-reason').textContent=localization.reason||fallback;
    const accuracy=[];
    if(active&&Number.isFinite(localization.rotated_degrees))accuracy.push(`누적 회전 ${Math.abs(localization.rotated_degrees).toFixed(0)}°`);
    if(Number.isFinite(localization.position_std))accuracy.push(`위치 표준편차 ${localization.position_std.toFixed(2)} m`);
    if(Number.isFinite(localization.yaw_std_degrees))accuracy.push(`방향 표준편차 ${localization.yaw_std_degrees.toFixed(1)}°`);
    $('#localization-accuracy').hidden=accuracy.length===0;$('#localization-accuracy').textContent=accuracy.join(' · ');
    const health=snapshot?.localization_health||{},reasons=Array.isArray(health.reasons)?health.reasons:[],metrics=health.metrics||{};
    const stopped=!!currentMission().estop_latched||!!snapshot?.safety?.latched;
    const stale=reasons.some(reason=>/unavailable|stale|not_updating/.test(reason))||health.level==='unknown'||!health.level;
    const covariance=reasons.some(reason=>['position_std_m_elevated','yaw_std_rad_elevated','position_std_m_high','yaw_std_rad_high','position_std_high','yaw_std_high','position_high','yaw_high'].includes(reason));
    let kind='ready',title='위치 신뢰도 정상',detail='지도에서 실제 위치와 방향을 확인한 뒤 임무를 시작하세요.';
    if(stopped){kind='stopped';title='비상정지 유지';detail='원인과 실제 정지를 확인하세요. 실제 등록 홈에 있으면 비상정지 해제 창에서 홈 위치를 확인하고 초기 위치만 입력할 수 있습니다. 센서가 안정된 뒤 비상정지를 해제하고 위치 확인을 별도로 실행하세요.';}
    else if(stale){kind='waiting';title='위치 입력 수신 대기';detail='라이다·오도메트리·AMCL과 측정 시각의 좌표 변환을 확인하세요. 입력이 복구될 때까지 정지합니다.';}
    else if(covariance){kind='warning';title='위치 불확실성 높음';detail=health.allow_localization===true?(health.autonomous_allowed===true?'위치 경고로 감속합니다. 정지한 뒤 ':'주행은 대기합니다. ')+'주변 회전 공간을 확보하고 자세 초기화 또는 일반 위치 확인을 실행하세요. 실제 홈에 있을 때만 자세 초기화를 사용하세요.':'라이다·오도메트리 입력부터 복구하세요. 입력이 확인된 뒤 위치 확인을 실행할 수 있습니다.';}
    else if(reasons.includes('initializing')){kind='waiting';title='초기 위치 안정 확인 중';detail=health.allow_localization===true?'센서 입력은 준비되었습니다. 정지 상태의 AMCL 갱신과 연속 안정 확인을 기다리세요. 수렴하지 않으면 위치 확인을 실행하세요.':'최신 센서와 측정 시각의 좌표 변환이 준비될 때까지 기다리세요.';}
    else if(health.level==='critical'||health.autonomous_allowed!==true){kind='warning';title='위치 검증 필요';detail=(health.explanations||reasons).join(' · ')||'저장 지도와 실제 위치를 확인하고 위치 확인을 실행하세요.';}
    const values=[];
    if(Number.isFinite(metrics.position_std_m))values.push(`현재 위치 표준편차 ${metrics.position_std_m.toFixed(3)} m`);
    if(Number.isFinite(metrics.yaw_std_rad))values.push(`방향 ${(metrics.yaw_std_rad*180/Math.PI).toFixed(1)}°`);
    const tfStates={READY:'준비됨',WAITING_FOR_SCAN:'라이다 대기',SCAN_STALE_OR_CLOCK_SKEW:'라이다 시각 확인 필요',WAITING_FOR_MEASUREMENT_TF:'측정 시각 좌표 변환 대기',MEASUREMENT_TF_TIMEOUT:'측정 시각 좌표 변환 시간 초과'};
    const refreshStates={WAITING_FOR_INPUTS:'입력 대기',WAITING_FOR_NEW_SCAN:'새 라이다 대기',WAITING_FOR_MOTION_OR_ACQUISITION:'갱신 조건 대기',REQUESTED:'새 AMCL 결과 요청 중',RESPONDED:'요청 응답 수신',TIMEOUT:'응답 시간 초과',SERVICE_UNAVAILABLE:'갱신 서비스 대기',STATIONARY_LIMIT_REACHED:'정지 갱신 한도 · 위치 확인 필요',REQUEST_FAILED:'갱신 요청 실패'};
    if(tfStates[metrics.scan_tf_state])values.push(`${tfStates[metrics.scan_tf_state]}${Number.isFinite(metrics.scan_tf_wait_ms)&&metrics.scan_tf_wait_ms>0?` · ${metrics.scan_tf_wait_ms.toFixed(0)} ms`:''}`);
    if(refreshStates[metrics.amcl_refresh_state])values.push(refreshStates[metrics.amcl_refresh_state]);
    if(active&&localization.control?.reason)detail+=' '+localization.control.reason;
    const panel=$('#localization-health');panel.dataset.state=kind;
    $('#localization-health-title').textContent=title;
    $('#localization-health-detail').textContent=detail;
    $('#localization-health-values').hidden=values.length===0;$('#localization-health-values').textContent=values.join(' · ');
  }
  function renderMarkerAlignment({homeMode=false,manualHolding=false,manualResuming=false}={}){
    const alignment=snapshot?.marker_alignment,mission=currentMission(),panel=$('#marker-alignment');
    const phases={SEARCHING:'표식 탐색 · 저속 회전',CENTERING:'카메라 중앙 정렬',WALL_ALIGNING:'후방 카메라 · 벽 정면 정렬',HOLD:'벽 방향 확인 · 정지 유지',SETTLING:'정지 · 안정 확인',CONFIRMED:'표식·차체 정렬 확인 완료',FAILED:'표식 확인 일시정지'};
    const state=String(alignment?.state||'IDLE').toUpperCase();
    const expected=mission.current_marker_id??snapshot?.detection?.expected_id;
    const sameTarget=expected==null||alignment?.marker_id==null||Number(expected)===Number(alignment.marker_id);
    const visible=!!alignment&&!!phases[state]&&!homeMode&&sameTarget&&['CONFIRMING','PAUSED','WAIT_MARKER','COMPLETED'].includes(mission.state);
    panel.hidden=!visible;if(!visible)return false;
    const stopped=!!mission.estop_latched||!!snapshot?.safety?.latched;
    const manual=manualHolding||manualResuming||!!snapshot?.manual?.active||['ACQUIRING','ACTIVE','RELEASING','WAIT_REPLAN'].includes(snapshot?.manual?.state);
    panel.classList.toggle('failed',state==='FAILED');panel.classList.toggle('confirmed',state==='CONFIRMED'&&!stopped&&!manual);panel.classList.toggle('stopped',stopped);
    $('#marker-alignment-phase').textContent=stopped?'비상정지 · 표식 정렬 중지':manual?'수동 조종 우선 · 표식 확인 대기':mission.state==='PAUSED'&&state!=='FAILED'?'임무 일시정지 · 표식 확인 대기':phases[state];
    const camera=alignment.camera==='front'?'전방 카메라':alignment.camera==='rear'?'후방 카메라':'카메라 확인 중';
    $('#marker-alignment-camera').textContent=camera+(Number.isInteger(alignment.marker_id)?` · ID ${alignment.marker_id}`:'');
    const metrics=[],error=alignment.center_error,required=alignment.required??alignment.required_frames;
    if(typeof error==='number'&&Number.isFinite(error)){
      const offset=Math.abs(error)*50;
      metrics.push(offset<0.05?'중심 오차 0.0%':`중심 오차 ${error<0?'왼쪽':'오른쪽'} ${offset.toFixed(1)}%`);
      metrics.push('영상 폭 기준');
    }else metrics.push('중심 오차 측정 대기');
    if(Number.isFinite(alignment.frames)&&Number.isFinite(required)&&required>0)metrics.push(`안정 확인 ${Math.max(0,Math.min(required,alignment.frames))} / ${required} 프레임`);
    if(alignment.wall_required){
      metrics.push(alignment.wall_reliable&&Number.isFinite(alignment.wall_heading_error_rad)?`벽 정면 각도 오차 ${(Math.abs(alignment.wall_heading_error_rad)*180/Math.PI).toFixed(1)}°`:'벽 정면 확인 대기');
      if(Number.isFinite(alignment.wall_samples)&&Number.isFinite(alignment.wall_required_samples))metrics.push(`벽 확인 ${alignment.wall_samples} / ${alignment.wall_required_samples}`);
    }
    $('#marker-alignment-metrics').textContent=metrics.join(' · ');
    const max=typeof alignment.max_seconds==='number'&&Number.isFinite(alignment.max_seconds)?`최대 ${alignment.max_seconds}초 안에 확인하지 못하면 임무를 일시정지합니다. `:'';
    let detail=alignment.reason||({SEARCHING:'목표 표식을 찾기 위해 제자리에서 저속 회전합니다.',CENTERING:'표식이 영상 중앙에 오도록 제자리에서 방향을 맞춥니다.',WALL_ALIGNING:'후방 카메라가 표식 벽을 정면으로 보고 차체의 앞뒤 축이 벽과 직각이 되도록 회전합니다.',HOLD:'라이다로 표식 벽의 방향을 확인할 때까지 정지합니다.',SETTLING:'차량을 멈추고 새 영상과 벽 방향을 연속 확인합니다.',CONFIRMED:'안정된 표식과 차체 방향을 확인했습니다. 다음 임무 상태를 확인하세요.',FAILED:'정해진 시간 안에 표식 정렬을 확인하지 못했습니다.'}[state]);
    if(stopped)detail='주변을 확인하고 비상정지를 해제한 뒤 임무를 별도로 재개하세요.';
    else if(manual)detail='조이스틱이 우선합니다. 표식 확인 단계는 유지되며, 조이스틱을 놓은 뒤 임무 상태를 확인하세요.';
    else if(state==='FAILED')detail+=' 표식과 카메라 방향을 확인한 뒤 '+(delivery().active?'‘배송 재개’':'‘수동 임무 재개’')+'를 누르세요. 자동 출발하지 않습니다.';
    else if(mission.state==='PAUSED')detail='표식 정렬 동작을 멈췄습니다. 주변을 확인한 뒤 임무를 재개하세요.';
    else if(state!=='CONFIRMED')detail+=' '+max+'조이스틱과 비상정지는 항상 우선합니다.';
    $('#marker-alignment-detail').textContent=detail;
    return true;
  }
  function renderStatus(){
    const mission=currentMission(),connection=snapshot.connection||{},robot=snapshot.robot||{},detection=snapshot.detection||{},navigation=snapshot.navigation||{};
    VehiclePower.render($('#vehicle-power'),robot.power);
    $('#domain-label').textContent=`Domain ${connection.domain_id??'—'}`;$('#clock-label').textContent=connection.use_sim_time?'/clock':'실제 시간';$('#connection-dot').classList.toggle('online',!!connection.connected&&!!snapshot.health?.ready);
    const homeMode=!!mission.returning_home||mission.state==='HOME_REACHED';
    const manualPhase=snapshot.manual?.state||'IDLE',manualHolding=['ACQUIRING','ACTIVE'].includes(manualPhase),manualResuming=['RELEASING','WAIT_REPLAN'].includes(manualPhase);
    const missionTitle=mission.returning_home&&mission.state==='PAUSED'?'홈 복귀 일시정지':mission.returning_home&&mission.state==='WAIT_REPLAN'?'홈 복귀 · 재계획 대기':states[mission.state]||mission.state||'임무 대기';
    const localizing=!!snapshot.localization?.active;
    const title=mission.estop_latched?states.ESTOP:localizing?'라이다 위치 확인 중':manualHolding?'수동 우선':manualResuming?'임무 재개 준비':missionTitle;
    $('#state-pill').textContent=title;$('#state-pill').classList.toggle('warning',localizing||manualHolding||manualResuming||['WAIT_REPLAN','WAIT_MARKER','CONFIRMING','PAUSED'].includes(mission.state));$('#state-pill').classList.toggle('error',['ESTOP','ERROR'].includes(mission.state));
    $('#mission-title').textContent=title+(!localizing&&!homeMode&&mission.current_marker_id!=null?` · ${markerLabel(mission.current_marker_id)}`:'');
    $('#mission-reason').textContent=mission.estop_latched?mission.reason||'비상정지가 적용되어 있습니다.':localizing?snapshot.localization.reason||'저속으로 회전하며 저장 지도에서 현재 위치를 추정합니다.':manualHolding?'수동 조종을 우선합니다. 목표·방문 순서·완료 기록은 유지됩니다.':manualResuming?snapshot.manual?.reason||'보관된 목표로 자동 주행을 다시 연결하고 있습니다.':mission.reason||navigation.message||'마커 순서를 저장한 뒤 임무를 시작하세요.';
    renderLocalization();
    const home=homepoint(),pose=robot.pose;
    $('#mission-progress').textContent=mission.returning_home&&home&&pose?`홈까지 ${Math.hypot(home.x-pose.x,home.y-pose.y).toFixed(2)} m`:mission.state==='HOME_REACHED'?'HOME':`${(mission.completed_ids||[]).length} / ${(mission.route||[]).length}`;
    $('#home-position').textContent=home?`Homepoint · x ${Number(home.x).toFixed(2)} · y ${Number(home.y).toFixed(2)} m`:'Homepoint: 미설정';
    $('#home-position').title=home?`고정 대기 위치 · 방향 ${(Number(home.yaw||0)*180/Math.PI).toFixed(1)}°`:'';
    const age=robot.camera_age;$('#camera-age').textContent=typeof age==='number'&&Number.isFinite(age)?`${age.toFixed(1)}초 전`:'수신 대기';
    const frontAge=robot.front_camera_age;$('#front-camera-age').textContent=typeof frontAge==='number'&&Number.isFinite(frontAge)?`${frontAge.toFixed(1)}초 전`:'수신 대기';
    $('#expected-id').textContent=homeMode?'출발 홈으로 복귀':detection.expected_id==null?'확인 대상 —':`확인 대상 ID ${detection.expected_id} · ${detection.expected_camera==='front'?'전방':'후방'}`;
    const expectedIds=detection.expected_camera==='front'?detection.front_ids:detection.ids;
    $('#detected-ids').textContent=(detection.expected_camera==='front'?'전방':'후방')+' · '+(expectedIds?.length?`검출 ${expectedIds.join(', ')}`:'검출 없음');
    const approachingMarker=detection.expected_camera==='front'&&['NAVIGATING','MOVING'].includes(mission.state)&&expectedIds?.includes(Number(detection.expected_id));
    $('#confirmation-status').textContent=homeMode?'홈 복귀는 저장된 위치와 방향으로 도착을 확인합니다.':detection.confirmed?'표식 확인 완료':mission.state==='CONFIRMING'&&detection.expected_camera==='front'&&!(typeof robot.front_camera_age==='number'&&robot.front_camera_age<2)?'전방 카메라 영상 대기 · 도착 후 표식 확인이 필요합니다.':mission.state==='CONFIRMING'?`연속 확인 ${detection.confirmations??0} / ${detection.required??3}`:approachingMarker?`전방에서 ID ${detection.expected_id} 검출 · 목표 위치로 접근 중`:'관측 위치 도착 후 표식을 확인합니다.';
    $('#confirmation-status').classList.toggle('confirmed',!homeMode&&!!detection.confirmed);
    $('#confirmation-status').hidden=renderMarkerAlignment({homeMode,manualHolding,manualResuming});
    $('#pose-label').textContent=pose?`x ${pose.x.toFixed(2)} · y ${pose.y.toFixed(2)} m`:'위치 수신 대기';
    $('#connection-banner').hidden=!!snapshot.health?.ready||!!snapshot.operation;
    if(!snapshot.health?.ready){
      const missing=snapshot.health?.missing||[],reasons=explainReadiness(missing);
      const title=missing.includes('설정 지도와 다른 /map')?'지도 확인 필요':missing.includes('최신 map→base TF')?'위치 추정 대기':'ROS 데이터 수신 대기';
      $('#connection-banner').textContent=reasons.length?`${title}: ${reasons.join(' · ')}`:'ROS 데이터 수신 대기. 같은 Domain의 차량·지도·주행 노드가 실행 중인지 확인하세요.';
    }
    if(snapshot.map)$('#map-label').textContent=`${snapshot.map.configured_match===false?'설정 지도 불일치':snapshot.map.preview?'저장 지도 · ROS 연결 대기':snapshot.map.frame_id||'map'} · ${Number(snapshot.map.resolution).toFixed(2)} m/cell`;
    if(Array.isArray(snapshot.path)&&snapshot.path.length)lastPath=snapshot.path;
    const waiting=mission.state==='WAIT_REPLAN'||navigation.planner_status==='WAIT_REPLAN';
    $('#path-label').hidden=!waiting;$('#path-label').textContent=lastPath.length?'재계획 대기 · 이전 경로':'재계획 대기 · 안전 정지';
    const eventList=$('#event-list');eventList.replaceChildren();const events=(snapshot.events||[]).slice(-20).reverse();$('#event-count').textContent=String(events.length);
    events.forEach(event=>{const li=document.createElement('li');li.className=String(event.level||'').toLowerCase();const time=document.createElement('time');const stamp=typeof event.at==='number'?new Date(event.at*1000):new Date(event.at);time.textContent=Number.isNaN(stamp.getTime())?'—':stamp.toLocaleTimeString('ko-KR',{hour12:false});const text=document.createElement('span');text.textContent=event.message;li.append(time,text);eventList.append(li);});
  }
  const requestStates={PENDING:'접수',QUEUED:'대기',IN_PROGRESS:'운송 중',DELIVERED:'배송 완료',CANCELED:'취소'};
  function failureReason(value){
    return {execution_no_progress:'이동 명령에도 진행되지 않음',execution_oscillation:'같은 위치에서 좌우 선회 반복',controller_aborted:'주행 제어 중단 · 다른 접근 검토'}[value]||value||'';
  }
  function renderFailureMemory(){
    const f=snapshot.failure_memory,m=currentMission(),panel=$('#failure-memory');
    const phases={AVOIDING:'실패 접근 회피 · 재계획',WAIT_ALTERNATIVE:'목표 유지 · 대체 경로 대기',RECOVERING:'안전 후진 중',REJOINING:'새 경로 진입 확인 중',SENSOR_HOLD:'센서 회복 대기',OBSTACLE_HOLD:'장애물 해소 대기'};
    const valid=!!f&&f.available===true&&['NAVIGATING','RETURNING_HOME','WAIT_REPLAN','PAUSED','ESTOP'].includes(m.state);
    const records=valid&&Array.isArray(f.active)?f.active:[];
    panel.hidden=!valid||(!phases[f.state]&&!records.length);
    if(panel.hidden)return;
    const held=m.estop_latched||snapshot.safety?.latched;
    const manual=snapshot.manual?.active||['ACQUIRING','ACTIVE','RELEASING','WAIT_REPLAN'].includes(snapshot.manual?.state);
    $('#failure-memory-phase').textContent=held?'비상정지 · 실패 접근 기억 유지':manual?'수동 조종 우선 · 실패 접근 기억 유지':m.state==='PAUSED'?'일시정지 · 실패 접근 기억 유지':phases[f.state]||'실패 접근 기억 유지';
    const retry=Number.isInteger(f.retry_count)&&Number.isInteger(f.retry_limit)&&f.retry_limit>0?` · 재시도 ${f.retry_count}/${f.retry_limit}회`:'';
    $('#failure-memory-count').textContent=`회피 기억 ${records.length}건${retry}`;
    $('#failure-memory-reason').textContent=f.state==='OBSTACLE_HOLD'?'장애물이 있어 경로 주행을 잠시 멈춥니다. 장애물을 치우면 경로를 재개합니다.':failureReason(f.reason)||'같은 실패 구간·진입 방향의 반복을 줄입니다.';
    const list=$('#failure-memory-records');list.replaceChildren();
    records.slice(0,100).forEach(item=>{const li=document.createElement('li');const ttl=typeof item.expires_in==='number'&&Number.isFinite(item.expires_in)?`${Math.max(0,Math.ceil(item.expires_in))}초 후 재평가`:'유지 시간 확인 중';const position=Number.isFinite(item.x)&&Number.isFinite(item.y)?`(${item.x.toFixed(2)}, ${item.y.toFixed(2)}) · `:'';li.textContent=`${position}${failureReason(item.reason)||'실패 접근'} · ${ttl}`;list.append(li);});
    if(!records.length){const li=document.createElement('li');li.textContent=f.state==='SENSOR_HOLD'?'센서 단절은 경로 실패로 기록하지 않습니다.':f.state==='OBSTACLE_HOLD'?'현재 목표를 유지하고 실제 장애물 해소를 기다립니다.':'현재 남아 있는 실패 접근 제한은 없습니다.';list.append(li);}
  }
  function renderOperation(){
    renderFailureMemory();
    const o=snapshot.operation;if(!o)return;
    $('#state-pill').textContent=o.label||'임무 대기';
    $('#state-pill').classList.toggle('warning',o.level==='warning');
    $('#state-pill').classList.toggle('error',o.level==='critical');
    $('#operation-summary').dataset.level=o.level||'info';
    $('#operation-phase').textContent=o.label||'임무 대기';
    $('#operation-detail').textContent=failureReason(o.detail);
    $('#operation-action-title').textContent=o.next_action?.title||'다음 조치';
    $('#operation-action').textContent=o.next_action?.action||'';
    const target=o.next_action?.target,button=$('#operation-action-button');
    button.hidden=!['connection','localize','camera','safety'].includes(target);
    button.textContent={connection:'연결 설정',localize:'초기 위치 지정으로 이동',camera:'영상 확인',safety:'정지 제어 확인'}[target]||'확인하기';
    button.onclick=()=>{if(target==='connection')$('#connection-open').click();else if(target==='localize'){$('#initial-pose-panel').scrollIntoView({behavior:'smooth',block:'center'});$('#initial-pose-x').focus({preventScroll:true});}else if(target==='camera')$('.camera-panel').scrollIntoView({behavior:'smooth',block:'center'});else $('#emergency-stop').focus();};
    const diagnostics=$('#operation-diagnostics');diagnostics.replaceChildren();
    const items=[...(snapshot.health?.missing||[]),...(snapshot.health?.warnings||[])];
    const q=snapshot.localization_health;if(q?.level&&q.level!=='unknown')items.push(`위치 신뢰도 ${{ok:'정상',healthy:'정상',warning:'주의',critical:'정지'}[q.level]||q.level}: ${(q.explanations||q.reasons||[]).join(' · ')||'정상'}`);
    if(!items.length)items.push('필수 센서·주행 준비 조건 충족');
    items.forEach(text=>{const li=document.createElement('li');li.textContent=text;diagnostics.append(li);});
    const s=snapshot.safety||{};
    $('#safety-ack').textContent=s.pending==='reset'?'비상정지 해제 검증 중 · 자동 출발하지 않습니다.':s.latched?(s.stop_acknowledged?'정지 명령 0 발행 확인 · 실제 정지는 차량에서 확인하세요.':s.result||'정지 요청 중 · 제어 응답 대기'):s.result||'정지 제어 대기';
    const route=$('#active-route');route.replaceChildren();
    (snapshot.route_preview||[]).forEach(step=>{const li=document.createElement('li');li.textContent=`${step.order}. ${step.name}${step.completed?' ✓':step.current?' · 현재 목표':''}`;if(step.current)li.setAttribute('aria-current','step');li.classList.toggle('completed',!!step.completed);route.append(li);});
  }
  function requestTime(value){const time=typeof value==='number'?new Date(value*1000):new Date(value);return Number.isNaN(time.getTime())?'—':time.toLocaleString('ko-KR',{month:'numeric',day:'numeric',hour:'2-digit',minute:'2-digit'});}
  function renderDelivery(){
    const d=delivery(),mission=currentMission(),requests=d.requests||[],trip=d.active_trip,tripId=trip?.id??null;
    if(tripId!==previousTripId){$('#load-confirmed').checked=false;previousTripId=tripId;}
    $('#request-count').textContent=String(d.pending_count||0);
    $('#dispatch-description').textContent=d.active?'배송 중 새 요청이 들어오면 현재 방문 목록 뒤에 자동으로 추가합니다.':`출발 대기 요청 ${d.pending_count||0}건 · 운용자 적재 확인 후 출발`;
    $('#delivery-home-state').textContent=d.home_ready?'Homepoint에서 대기 중':d.homepoint?'Homepoint 복귀·대기 위치 지정됨':'Homepoint를 먼저 지정하세요';
    $('#delivery-home-position').textContent=d.homepoint?`x ${Number(d.homepoint.x).toFixed(2)} · y ${Number(d.homepoint.y).toFixed(2)} m · 방향 ${(Number(d.homepoint.yaw||0)*180/Math.PI).toFixed(1)}°`:'정지 상태에서 현재 위치를 고정 대기 위치로 저장하세요.';
    if(d.homepoint_custom)$('#delivery-home-state').textContent+=' · 사용자 지정';
    const registered=d.registered_homepoint;
    $('#reset-home').title=registered?`등록된 홈 x ${Number(registered.x).toFixed(2)}, y ${Number(registered.y).toFixed(2)} m로 복원합니다. 차량 이동이나 현재 위치 재설정은 하지 않습니다.`:'지도에 등록된 홈 좌표로 복원합니다. 차량 이동이나 현재 위치 재설정은 하지 않습니다.';
    $('#trip-status').textContent=d.active?(mission.returning_home?'Homepoint 복귀 중':`운송 중 · ${requests.filter(r=>r.trip_id===tripId&&r.status==='DELIVERED').length} / ${trip?.request_ids?.length||0}건 완료`):d.home_ready?'HOME · 출발 대기':'Homepoint 확인 필요';
    $('#trip-status').classList.toggle('active',!!d.active);
    $('#delivery-policy').textContent=d.active&&d.accepting_updates?'추가 요청 자동 접수 중 · 현재 배송 목표 뒤에 이어서 방문합니다. 모두 완료하면 Homepoint로 자동 복귀합니다.':d.active?'Homepoint 복귀 중입니다. 새 요청은 다음 운송을 위해 접수합니다.':'Homepoint에서 적재를 확인하면 출발합니다. 운송 중 추가된 요청은 접수 순서대로 이어서 방문하며, 모든 목표 완료 후 Homepoint로 돌아옵니다.';
    const key=JSON.stringify([requests,snapshot.markers]);if(key===deliveryKey)return;deliveryKey=key;
    const list=$('#delivery-requests');list.replaceChildren();$('#request-empty').hidden=requests.length>0;
    const shown=[...requests].sort((a,b)=>{const done=x=>['DELIVERED','CANCELED'].includes(x.status)?1:0;return done(a)-done(b);}).slice(0,100);
    shown.forEach(request=>{
      const row=document.createElement('li');row.className='request-card';
      const identity=document.createElement('div');identity.className='request-identity';const who=document.createElement('strong');who.textContent=`사용자 ${request.user_id}`;const when=document.createElement('small');when.textContent=requestTime(request.created_at);identity.append(who,when);
      const details=document.createElement('div');details.className='request-details';const title=document.createElement('strong');title.textContent=`${request.item} · ${request.quantity}개`;const destination=document.createElement('span');destination.textContent=`목적지 ${markerLabel(request.marker_id)}`;destination.title=`ArUco ID ${request.marker_id}`;details.append(title,destination);if(request.note){const note=document.createElement('small');note.textContent=request.note;details.append(note);}
      const status=document.createElement('span');status.className='request-state '+String(request.status||'').toLowerCase();status.textContent=requestStates[request.status]||request.status;
      row.append(identity,details,status);list.append(row);
    });
  }
  async function refresh(){
    if(pollRunning)return;pollRunning=true;
    try{
      const data=await api('/api/state');disconnected=false;
      if(data.connection?.access_mode&&data.connection.access_mode!==accessMode)setAccessMode(data.connection.access_mode);
      if(snapshot?.connection?.domain_id!==undefined&&snapshot.connection.domain_id!==data.connection?.domain_id){lastPath=[];mapVersion=null;mapImageUrl='';mapLayout=null;mapLayoutGeneration++;initialized=false;dirty=false;}
      snapshot=data;
      if(!initialized||!dirty){draft=[...(data.mission?.pending_route??data.mission?.route??[])];initialized=true;}
      const nextMapVersion=data.map?.version??data.map_version;
      if(data.map&&mapVersion!==nextMapVersion){mapVersion=nextMapVersion;mapImageUrl=`/api/map.png?v=${encodeURIComponent(mapVersion)}`;loadMapLayout(data.map,mapVersion);}
      renderMarkerButtons();renderEditor();renderDelivery();renderStatus();renderOperation();renderControls();drawMap();
    } catch(error){disconnected=true;VehiclePower.render($('#vehicle-power'),snapshot?.robot?.power,true);$('#connection-dot').classList.remove('online');$('#connection-banner').hidden=false;$('#connection-banner').textContent=error.message;$('#state-pill').textContent='웹 연결 확인 필요';renderControls();}
    finally{pollRunning=false;}
  }
  function loadMapLayout(map,version){
    const generation=++mapLayoutGeneration;mapLayout=null;mapLayoutLoading=true;
    // A layout is only a display layer. Planning always uses the original occupancy map.
    api(`/api/map-layout?v=${encodeURIComponent(version)}`).then(layout=>{
      if(generation!==mapLayoutGeneration||String(layout.version)!==String(version)||layout.width!==map.width||layout.height!==map.height)return;
      if(Array.isArray(layout.free_paths)&&Array.isArray(layout.occupied_paths))mapLayout=layout;
    }).catch(()=>{}).finally(()=>{if(generation===mapLayoutGeneration){mapLayoutLoading=false;drawMap();}});
  }
  function polygonPath(rings){
    return rings.filter(ring=>Array.isArray(ring)&&ring.length>=3&&ring.every(point=>Array.isArray(point)&&point.length>=2&&point.slice(0,2).every(Number.isFinite))).map(ring=>`M${ring.map(point=>`${point[0]},${point[1]}`).join('L')}Z`).join('');
  }
  function layoutBounds(layout,map){
    let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity;
    for(const rings of [layout.free_paths,layout.occupied_paths,layout.obstacle_paths||[]])for(const ring of rings)for(const point of ring){
      if(!Array.isArray(point)||!point.slice(0,2).every(Number.isFinite))continue;
      minX=Math.min(minX,point[0]);minY=Math.min(minY,point[1]);maxX=Math.max(maxX,point[0]);maxY=Math.max(maxY,point[1]);
    }
    // Ignore the unknown border for presentation, while retaining pixel/map coordinates.
    if(![minX,minY,maxX,maxY].every(Number.isFinite)||maxX<=minX||maxY<=minY)return{x:0,y:0,width:map.width,height:map.height};
    minX=Math.max(0,minX-1);minY=Math.max(0,minY-1);maxX=Math.min(map.width,maxX+1);maxY=Math.min(map.height,maxY+1);
    return{x:minX,y:minY,width:maxX-minX,height:maxY-minY};
  }
  function drawMap(){
    const area=$('#map-area'),canvas=$('#map-svg'),map=snapshot?.map;canvas.replaceChildren();$('#map-empty').hidden=!!map;
    $('#map-view-plan').setAttribute('aria-pressed',String(!showRawMap));$('#map-view-raw').setAttribute('aria-pressed',String(showRawMap));
    $('#map-view-plan').disabled=!map;$('#map-view-raw').disabled=!map;$('#map-view-state').hidden=!map;
    if(!map||!map.width||!map.height)return;
    const width=area.clientWidth,height=area.clientHeight,pad=width<450?42:52;
    const layoutReady=mapLayout&&mapLayout.width===map.width&&mapLayout.height===map.height&&String(mapLayout.version)===String(mapVersion),useLayout=layoutReady&&!showRawMap,architectural=useLayout&&mapLayout.style==='architectural';
    const bounds=useLayout?layoutBounds(mapLayout,map):{x:0,y:0,width:map.width,height:map.height},sidePad=useLayout?Math.min(96,Math.max(32,width*.17)):pad,topPad=useLayout?48:pad,bottomPad=useLayout?42:pad;
    if(width<=2*sidePad||height<=topPad+bottomPad)return;
    const scale=Math.min((width-2*sidePad)/bounds.width,(height-topPad-bottomPad)/bounds.height),left=(width-bounds.width*scale)/2-bounds.x*scale,top=topPad+(height-topPad-bottomPad-bounds.height*scale)/2-bounds.y*scale;
    canvas.setAttribute('viewBox',`0 0 ${width} ${height}`);
    area.classList.toggle('floorplan-mode',!!useLayout);
    area.classList.toggle('architectural-mode',!!architectural);
    $('#map-view-plan').textContent=layoutReady&&mapLayout.style==='architectural'?'평면도':'도면';
    $('#map-view-state').textContent=showRawMap?'원본 점유 지도':useLayout?(mapLayout.label||(architectural?'강의실 평면도':'정리된 도면')):mapLayoutLoading?'도면 준비 중':'원본 지도';
    $('#map-view-state').title='표시 방식만 변경됩니다. 실제 지도 좌표와 주행 데이터는 동일합니다.';
    $('#map-localization-state').hidden=!map.preview&&map.configured_match!==false;
    $('#map-localization-state').textContent=map.configured_match===false?'지도 연결 확인 필요':'저장 지도 · 위치 추정 대기';
    if(useLayout){
      const floor=svg('g',{transform:`translate(${left},${top}) scale(${scale})`,'aria-label':architectural?'실제 지도 좌표에 맞춘 강의실 평면도':'원본 지도의 벽과 자유 공간을 표시한 도면'});
      floor.append(svg('path',{d:polygonPath(mapLayout.free_paths),class:'floorplan-floor','fill-rule':'evenodd'}));
      // Architectural wall widths come from the drawing geometry, without a second outline.
      floor.append(svg('path',{d:polygonPath(mapLayout.occupied_paths),class:'floorplan-wall','fill-rule':architectural?'nonzero':'evenodd'}));
      if(Array.isArray(mapLayout.obstacle_paths))floor.append(svg('path',{d:polygonPath(mapLayout.obstacle_paths),class:'floorplan-obstacle','fill-rule':'nonzero','vector-effect':'non-scaling-stroke'}));
      canvas.append(floor);
    }else canvas.append(svg('image',{x:left,y:top,width:map.width*scale,height:map.height*scale,href:mapImageUrl,preserveAspectRatio:'none',style:'image-rendering:pixelated'}));
    const origin=map.origin||{},yaw=Number(origin.yaw)||0,c=Math.cos(yaw),s=Math.sin(yaw),res=map.resolution;
    const xy=point=>{const dx=point.x-(origin.x||0),dy=point.y-(origin.y||0);return[left+(c*dx+s*dy)/res*scale,top+(map.height-(-s*dx+c*dy)/res)*scale];};
    const activePath=Array.isArray(snapshot.path)?snapshot.path:[],waiting=currentMission().state==='WAIT_REPLAN'||snapshot.navigation?.planner_status==='WAIT_REPLAN';
    const path=map.preview||map.configured_match===false?[]:activePath.length?activePath:lastPath;
    if(path.length){const coordinates=path.map(xy).filter(point=>point.every(Number.isFinite));canvas.append(svg('polyline',{points:coordinates.map(p=>p.join(',')).join(' '),fill:'none',stroke:'var(--panel)','stroke-width':7,opacity:0.8,'stroke-linejoin':'round','stroke-linecap':'round'}));canvas.append(svg('polyline',{points:coordinates.map(p=>p.join(',')).join(' '),fill:'none',stroke:waiting||!activePath.length?'#87919b':'var(--accent)','stroke-width':3,'stroke-linejoin':'round','stroke-linecap':'round','stroke-dasharray':waiting||!activePath.length?'5 5':'none'}));}
    const home=homepoint(),markers=snapshot.markers||[],cards=[],physicalClassroom=isPhysicalClassroom();
    const observationHeadings=markers.filter(marker=>Number.isFinite(marker.yaw)).map(marker=>{const[x,y]=xy(marker),angle=marker.yaw-yaw;return{x,y,dx:Math.cos(angle),dy:-Math.sin(angle)};});
    const labelPosition=(x,y,w)=>{
      const h=32,preferLeft=x<width/2,candidates=preferLeft?[[x-w-16,y-h/2],[x+16,y-h/2],[x-w/2,y-48],[x-w/2,y+16]]:[[x+16,y-h/2],[x-w-16,y-h/2],[x-w/2,y-48],[x-w/2,y+16]];
      const choices=candidates.map(([cx,cy],index)=>{const box={x:Math.max(12,Math.min(width-w-12,cx)),y:Math.max(42,Math.min(height-h-36,cy)),w,h};const overlap=cards.reduce((sum,other)=>sum+Math.max(0,Math.min(box.x+w+5,other.x+other.w)-Math.max(box.x-5,other.x))*Math.max(0,Math.min(box.y+h+5,other.y+other.h)-Math.max(box.y-5,other.y)),0),coveredPins=markers.map(xy).filter(([px,py])=>px>box.x-9&&px<box.x+w+9&&py>box.y-9&&py<box.y+h+9).length,coveredHeadings=observationHeadings.reduce((count,heading)=>count+[13,19,25].filter(distance=>{const px=heading.x+heading.dx*distance,py=heading.y+heading.dy*distance;return px>box.x-5&&px<box.x+w+5&&py>box.y-5&&py<box.y+h+5;}).length,0);return{...box,cost:overlap*100+coveredPins*1000+coveredHeadings*1000+Math.abs(box.x-cx)+Math.abs(box.y-cy)+index*5};});
      choices.sort((a,b)=>a.cost-b.cost);cards.push(choices[0]);return choices[0];
    };
    const drawLocation=(point,label,id,isHome=false)=>{
      const[x,y]=xy(point);if(![x,y].every(Number.isFinite))return;
      const ordered=id!=null&&draft.includes(Number(id)),current=id!=null&&currentMission().current_marker_id!=null&&!currentMission().returning_home&&Number(currentMission().current_marker_id)===Number(id)&&!terminal.has(currentMission().state),group=svg('g',{class:`map-location${isHome?' home':''}${current?' current':''}`}),labelWidth=Math.max(76,Math.min(154,40+Array.from(label).reduce((sum,char)=>sum+(/[\u3131-\uD79D]/.test(char)?12:7),0))),box=labelPosition(x,y,labelWidth);
      const hasHeading=Number.isFinite(point.yaw),heading=hasHeading?point.yaw-yaw:0,headingDirection=['오른쪽','오른쪽 위','위쪽','왼쪽 위','왼쪽','왼쪽 아래','아래쪽','오른쪽 아래'][((Math.round(heading/(Math.PI/4))%8)+8)%8],headingDegrees=hasHeading?Math.atan2(Math.sin(point.yaw),Math.cos(point.yaw))*180/Math.PI:0;
      const cameraRole=point.camera==='front'?'전방':'후방';
      const directionDescription=physicalClassroom?`차체 전방 ${headingDirection} · yaw ${headingDegrees.toFixed(0)}° · ${cameraRole} 카메라는 화살표 ${cameraRole==='front'?'방향':'반대 방향'}을 봅니다`:`관측 방향 ${headingDirection} · yaw ${headingDegrees.toFixed(0)}°`;
      group.append(svg('title',{},`${label}${id==null?'':` · ArUco ID ${id}`} · x ${Number(point.x).toFixed(2)}, y ${Number(point.y).toFixed(2)} m${hasHeading?` · ${directionDescription}`:''}${ordered?' · 방문 순서 '+draft.flatMap((entry,index)=>entry===Number(id)?[index+1]:[]).join(', '):''}`));
      const endX=Math.max(box.x+8,Math.min(box.x+box.w-8,x)),endY=Math.max(box.y+8,Math.min(box.y+box.h-8,y));
      group.append(svg('line',{x1:x,y1:y,x2:endX,y2:endY,class:'map-location-leader'}));
      // For the physical classroom this is base/front yaw; each marker declares its observing camera.
      // In simulator profiles the same arrow retains the saved camera-observation heading meaning.
      if(hasHeading)group.append(svg('path',{d:'M 13 0 H 25 M 20 -4 L 25 0 L 20 4',transform:`translate(${x},${y}) rotate(${-heading*180/Math.PI})`,class:'map-location-heading'}));
      group.append(svg('circle',{cx:x,cy:y,r:6,class:'map-location-point'}));
      if(isHome)group.append(svg('circle',{cx:x,cy:y,r:10,class:'map-home-ring'}));
      group.append(svg('rect',{x:box.x,y:box.y,width:box.w,height:box.h,rx:9,class:'map-location-card'}));
      group.append(svg('rect',{x:box.x+6,y:box.y+6,width:20,height:20,rx:5,class:'map-location-number-bg'}));
      group.append(svg('text',{x:box.x+16,y:box.y+21,'text-anchor':'middle',class:'map-location-number'},id==null?'H':String(id)));
      group.append(svg('text',{x:box.x+33,y:box.y+21,class:'map-location-name'},label));canvas.append(group);
    };
    markers.forEach(marker=>drawLocation(marker,markerLabel(marker.id),marker.id,!!home&&Math.hypot(home.x-marker.x,home.y-marker.y)<0.08));
    if(home&&!markers.some(marker=>Math.hypot(home.x-marker.x,home.y-marker.y)<0.08))drawLocation(home,'홈 · 대기',null,true);
    const pose=map.preview||map.configured_match===false?null:snapshot.robot?.pose;if(pose){const[x,y]=xy(pose),group=svg('g',{transform:`translate(${x},${y}) rotate(${-(pose.yaw-yaw)*180/Math.PI})`});group.append(svg('circle',{r:15,fill:'var(--accent)',opacity:0.12}));group.append(svg('circle',{r:10,fill:'var(--panel)',stroke:'var(--accent)','stroke-width':2}));group.append(svg('path',{d:'M 14 0 L -4 -5 L -1 0 L -4 5 Z',fill:'var(--accent)'}));canvas.append(group);}
    const pixelsPerMetre=scale/res,scaleMetres=[0.1,0.2,0.5,1,2,5,10].filter(value=>value*pixelsPerMetre<=90).pop()||0.1,scalePixels=scaleMetres*pixelsPerMetre,scaleGroup=svg('g',{transform:`translate(24,${height-23})`,class:'map-scale'});
    scaleGroup.append(svg('path',{d:`M 0 -4 V 0 H ${scalePixels} V -4`,fill:'none',stroke:'currentColor','stroke-width':1.5}));scaleGroup.append(svg('text',{x:scalePixels/2,y:-9,'text-anchor':'middle','font-size':10},`${scaleMetres} m`));canvas.append(scaleGroup);
  }
  function updateCamera(){
    if(document.hidden||disconnected||!initialized)return;
    const age=snapshot.robot?.camera_age;
    if(typeof age!=='number'||!Number.isFinite(age)||age>5){$('#camera-empty').hidden=false;$('#camera-image').hidden=true;}
    else if(!cameraLoading){cameraLoading=true;$('#camera-image').src=`/api/camera.jpg?t=${Date.now()}`;}
    const frontAge=snapshot.robot?.front_camera_age;
    if(typeof frontAge!=='number'||!Number.isFinite(frontAge)||frontAge>5){$('#front-camera-empty').hidden=false;$('#front-camera-image').hidden=true;}
    else if(!frontCameraLoading){frontCameraLoading=true;$('#front-camera-image').src=`/api/camera/front.jpg?t=${Date.now()}`;}
  }
  $('#camera-image').addEventListener('load',()=>{cameraLoading=false;$('#camera-image').hidden=false;$('#camera-empty').hidden=true;});
  $('#camera-image').addEventListener('error',()=>{cameraLoading=false;$('#camera-image').hidden=true;$('#camera-empty').hidden=false;});
  $('#front-camera-image').addEventListener('load',()=>{frontCameraLoading=false;$('#front-camera-image').hidden=false;$('#front-camera-empty').hidden=true;});
  $('#front-camera-image').addEventListener('error',()=>{frontCameraLoading=false;$('#front-camera-image').hidden=true;$('#front-camera-empty').hidden=false;});
  $('#map-view-plan').addEventListener('click',()=>{showRawMap=false;drawMap();});
  $('#map-view-raw').addEventListener('click',()=>{showRawMap=true;drawMap();});
  $('#load-confirmed').addEventListener('change',renderControls);
  $('#localization-start').addEventListener('click',async()=>{
    if($('#localization-start').disabled)return;
    localizationStarting=true;renderControls();
    try{await action('/api/localization/start',{},'저속 회전으로 위치 확인을 시작합니다. 완료 후 지도에서 차량 위치를 확인하세요.');}
    finally{localizationStarting=false;renderControls();}
  });
  $('#localization-start-home').addEventListener('click',async()=>{
    if($('#localization-start-home').disabled)return;
    if(!$('#startup-confirm-home').checked){openStartupRecovery();notify('실제 등록 홈 위치·방향을 확인한 뒤 확인란을 선택하세요.');return;}
    localizationStarting=true;renderControls();
    try{await action('/api/localization/start-home',{confirm_actual_home:true},'등록 홈을 초기 가설로 입력하고 위치 확인 뒤 홈 방향으로 복귀합니다. 완료 후 자동 출발하지 않습니다.');}
    finally{localizationStarting=false;renderControls();}
  });
  $('#localization-stop').addEventListener('click',async()=>{
    if($('#localization-stop').disabled)return;
    try{await api('/api/localization/stop',{});notify('위치 확인 회전 중지를 요청했습니다.');await refresh();}
    catch(error){notify(error.message,true);}
  });
  $('#startup-recovery-open').addEventListener('click',openStartupRecovery);
  $('#startup-recovery-open-dispatch').addEventListener('click',openStartupRecovery);
  $('#startup-recovery-close').addEventListener('click',()=>{
    $('#startup-recovery').hidden=true;setHomeConfirmation(false);renderControls();$('#startup-recovery-open').focus();
  });
  $('#startup-confirm-home').addEventListener('change',()=>{setHomeConfirmation($('#startup-confirm-home').checked);renderControls();});
  $('#startup-recheck').addEventListener('click',async()=>{await refresh();if(!disconnected)notify('현재 준비 상태를 다시 확인했습니다. 센서·통신 문제는 표시된 원인부터 해결하세요.');});
  $('#startup-connection').addEventListener('click',()=>$('#connection-open').click());
  $('#startup-seed-home').addEventListener('click',()=>{if(!$('#startup-seed-home').disabled&&$('#startup-confirm-home').checked)$('#seed-home-pose').click();});
  $('#startup-open-reset').addEventListener('click',()=>{if(!$('#startup-open-reset').disabled)$('#emergency-stop').click();});
  $('#startup-localize-home').addEventListener('click',()=>{if(!$('#startup-localize-home').disabled&&$('#startup-confirm-home').checked)$('#localization-start-home').click();});
  $('#startup-localize-general').addEventListener('click',()=>{if(!$('#startup-localize-general').disabled)$('#localization-start').click();});
  $('#startup-stop-localization').addEventListener('click',()=>{if(!$('#startup-stop-localization').disabled)$('#localization-stop').click();});
  $('#initial-pose-home').addEventListener('click',()=>{
    const home=delivery().registered_homepoint;
    if(!home||![home.x,home.y,home.yaw].every(Number.isFinite))return;
    $('#initial-pose-x').value=home.x.toFixed(2);
    $('#initial-pose-y').value=home.y.toFixed(2);
    $('#initial-pose-yaw').value=(Math.atan2(Math.sin(home.yaw),Math.cos(home.yaw))*180/Math.PI).toFixed(1);
    $('#initial-pose-confirm').checked=false;
    renderControls();
    notify('등록 홈 좌표를 입력란에 채웠습니다. 실제 위치·방향을 확인한 뒤 적용하세요.');
    $('#initial-pose-confirm').focus();
  });
  for(const selector of ['#initial-pose-x','#initial-pose-y','#initial-pose-yaw'])$(selector).addEventListener('input',()=>{$('#initial-pose-confirm').checked=false;renderControls();});
  $('#initial-pose-confirm').addEventListener('change',renderControls);
  $('#initial-pose-form').addEventListener('submit',async event=>{
    event.preventDefault();
    const pose=initialPoseValues();
    if(!pose||!$('#initial-pose-confirm').checked||$('#initial-pose-submit').disabled){notify('실제 로봇의 지도 좌표·방향과 확인란을 확인하세요.',true);return;}
    if(await action('/api/localization/initial-pose',{...pose,confirm:true},'초기 위치를 보냈습니다. AMCL 위치와 TF 확인 결과를 기다립니다.')){
      $('#initial-pose-confirm').checked=false;
      renderControls();
    }
  });
  $('#dispatch-start').addEventListener('click',async()=>{if(await action('/api/dispatch/start',{loaded:true},'적재 확인 완료. 접수 순서대로 배송을 시작합니다.'))$('#load-confirmed').checked=false;});
  $('#set-home').addEventListener('click',()=>action('/api/home/set',{},'현재 위치를 고정 Homepoint로 저장했습니다.'));
  $('#reset-home').addEventListener('click',()=>action('/api/home/reset',{},'지도에 등록된 홈으로 복원했습니다. 차량의 실제 위치는 별도로 확인하세요.'));
  $('#delivery-resume').addEventListener('click',()=>action('/api/mission/resume',{},'배송을 재개합니다.'));
  $('#save-route').addEventListener('click',async()=>{const route=[...draft];if(await action('/api/mission',{route},'마커 순서를 저장했습니다.')){dirty=!same(draft,savedRoute());await refresh();}});
  $('#start-mission').addEventListener('click',()=>{const mission=currentMission();if(mission.state==='PAUSED'&&mission.pending_route!=null&&!mission.returning_home)$('#replace-dialog').showModal();else action(mission.state==='PAUSED'?'/api/mission/resume':'/api/mission/start');});
  $('#pause-mission').addEventListener('click',()=>action('/api/mission/pause',{},'임무 일시정지를 요청했습니다.'));
  $('#return-home').addEventListener('click',()=>action('/api/mission/home',{},'현재 운송을 종료하고 Homepoint로 복귀합니다.'));
  $('#cancel-mission').addEventListener('click',()=>action('/api/mission/cancel',{},'미완료 요청을 보류하고 Homepoint로 복귀합니다.'));
  $('#emergency-stop').addEventListener('click',async()=>{ManualJoystick.release();if(currentMission().estop_latched&&snapshot?.safety?.stop_acknowledged&&snapshot?.safety?.pending!=='reset'){setHomeConfirmation(false);renderControls();$('#reset-dialog').showModal();return;}try{await api('/api/emergency-stop',{reset:false});notify('비상정지를 요청했습니다.');await refresh();}catch(error){notify(error.message,true);}});
  $('#confirm-home-location').addEventListener('change',()=>{setHomeConfirmation($('#confirm-home-location').checked);renderControls();});
  $('#seed-home-pose').addEventListener('click',()=>action('/api/localization/seed-home',{confirm_actual_home:$('#confirm-home-location').checked},'등록 홈 초기 위치 입력을 요청했습니다. 비상정지는 유지됩니다.'));
  $('#confirm-reset').addEventListener('click',async()=>{if(await action('/api/emergency-stop',{reset:true},'비상정지 해제를 요청했습니다. 제어·센서 검증 응답 후 해제됩니다.'))$('#reset-dialog').close();});
  $('#confirm-replace').addEventListener('click',async()=>{if(await action('/api/mission/restart',{},'새 순서로 시작합니다.'))$('#replace-dialog').close();});
  $('[data-close-dialog="connection-dialog"]').addEventListener('click',()=>$('#connection-dialog').close());
  document.querySelectorAll('[data-close-dialog]').forEach(button=>button.addEventListener('click',()=>$('#'+button.dataset.closeDialog).close()));
  $('#connection-open').addEventListener('click',()=>{
    ManualJoystick.release();
    const connection=snapshot?.connection||{};$('#domain-input').value=connection.domain_id??76;$('#sim-time-input').checked=!!connection.use_sim_time;$('#connection-error').textContent='';
    const domain=accessMode==='domain';$('#phone-access').hidden=!domain&&!connection.pairing_code;$('#pairing-code').textContent=connection.pairing_code||'—';$('#phone-domain').textContent=connection.domain_id??'—';$('#lan-links').replaceChildren();
    const d=delivery();$('#inventory-access').hidden=!domain&&!d.inventory_join_code;$('#inventory-join-code').textContent=d.inventory_join_code||'—';$('#inventory-domain').textContent=connection.domain_id??'—';$('#inventory-links').replaceChildren();(d.inventory_urls||[]).forEach(url=>{const link=document.createElement('a');link.href=url;link.textContent=url;$('#inventory-links').append(link);});
    (connection.lan_urls||[location.origin]).forEach(url=>{const link=document.createElement('a');link.href=url;link.textContent=url;$('#lan-links').append(link);});$('#connection-dialog').showModal();
  });
  $('#connection-form').addEventListener('submit',async event=>{event.preventDefault();ManualJoystick.release();const button=event.submitter;button.disabled=true;try{await api('/api/connection',{domain_id:Number($('#domain-input').value),use_sim_time:$('#sim-time-input').checked});initialized=false;dirty=false;lastPath=[];mapVersion=null;await refresh();$('#connection-dialog').close();notify('차량 연결을 변경했습니다.');}catch(error){$('#connection-error').textContent=error.message;}finally{button.disabled=false;}});
  $('#pair-form').addEventListener('submit',async event=>{event.preventDefault();const button=event.submitter;button.disabled=true;try{const value=$('#pair-code-input').value.trim();await api('/api/pair',accessMode==='domain'?{domain_id:Number(value)}:{code:value});$('#pair-dialog').close();$('#pair-code-input').value='';$('#pair-error').textContent='';await refresh();}catch(error){$('#pair-error').textContent=error.message;}finally{button.disabled=false;}});
  $('#pair-dialog').addEventListener('cancel',event=>event.preventDefault());
  new ResizeObserver(drawMap).observe($('#map-area'));
  document.addEventListener('visibilitychange',()=>{if(!document.hidden){refresh();updateCamera();}});
  (async()=>{try{const access=await api('/api/access');setAccessMode(access.mode);}catch{setAccessMode('code');}await refresh();setInterval(refresh,700);cameraTimer=setInterval(updateCamera,700);})();
  window.addEventListener('pagehide',()=>clearInterval(cameraTimer));
})();
