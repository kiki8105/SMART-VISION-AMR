'use strict';
(() => {
  const $ = selector => document.querySelector(selector);
  const labels = {PENDING:'접수',QUEUED:'대기',IN_PROGRESS:'운송 중',DELIVERED:'배송 완료',CANCELED:'취소'};
  let authenticated=false,requestBusy=false,refreshBusy=false,toastTimer=null;
  let snapshot=null,markerKey='',historyKey='',submission=null,authGeneration=0,pendingAction=null;
  let accessMode='code';

  function setAccessMode(mode){
    accessMode=mode==='domain'?'domain':'code';const domain=accessMode==='domain',input=$('#join-code');
    $('#signin-intro').textContent=domain?'사용자 번호와 로봇의 ROS Domain ID로 연결하세요. 사용자 0은 로봇 운용자 전용입니다.':'운용자가 안내한 재고 관리자 접속 코드를 입력하세요. 사용자 0은 로봇 운용자 전용입니다.';
    $('#join-credential-label').textContent=domain?'ROS Domain ID':'재고 관리자 접속 코드';
    input.type=domain?'number':'text';input.autocomplete=domain?'off':'one-time-code';
    if(domain){input.min='0';input.max='232';input.step='1';input.removeAttribute('maxlength');}
    else{input.removeAttribute('min');input.removeAttribute('max');input.removeAttribute('step');input.maxLength=16;}
  }
  const signInNotice=()=>accessMode==='domain'?'사용자 번호와 로봇의 ROS Domain ID를 입력하세요.':'운용자가 안내한 재고 관리자 접속 코드로 연결하세요.';

  function notify(message,error=false){
    clearTimeout(toastTimer);$('#toast').textContent=message;$('#toast').classList.toggle('error',error);$('#toast').hidden=false;
    toastTimer=setTimeout(()=>$('#toast').hidden=true,6000);
  }
  function showSignIn(){
    authenticated=false;authGeneration+=1;$('#signin-panel').hidden=false;$('#manager-workspace').hidden=true;$('#identity').hidden=true;
    pendingAction=null;if($('#request-action-dialog').open)$('#request-action-dialog').close();
    $('#vehicle-power').hidden=true;
    $('#service-state').textContent='관리자 연결 필요';
  }
  async function api(path,body){
    const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),12000);
    try{
      const response=await fetch(path,{method:body===undefined?'GET':'POST',headers:body===undefined?{}:{'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body),credentials:'same-origin',cache:'no-store',signal:controller.signal});
      const data=await response.json();
      if(!response.ok){if(response.status===401)showSignIn();throw new Error(data.error||`요청 실패 (${response.status})`);}
      return data;
    }finally{clearTimeout(timer);}
  }
  function formatTime(value){
    const date=typeof value==='number'?new Date(value*1000):new Date(value);
    return Number.isNaN(date.getTime())?'—':date.toLocaleString('ko-KR',{month:'numeric',day:'numeric',hour:'2-digit',minute:'2-digit'});
  }
  const markerLabel=marker=>String(marker?.name||'').trim()||`ArUco ${marker?.id??'—'}`;
  const clearableCount=()=>Number(snapshot?.service?.clearable_history_count)||0;
  const deferredCount=()=>Number(snapshot?.service?.deferred_history_count)||0;
  function updateClearButton(){
    $('#clear-history').disabled=!authenticated||requestBusy;
  }
  function showClearResult(result){
    const count=Number(result.removed_count)||0;
    const deferred=Number(result.deferred_count)||0;
    const cleared=count?`배송 기록 ${count}건을 즉시 삭제했습니다.`:'즉시 삭제할 완료·취소 기록이 없습니다.';
    const pending=deferred?`현재 운행에 연결된 기록 ${deferred}건은 홈 도착 확인 후 자동 삭제됩니다.`:
      result.service?.clear_pending?'현재 운행에서 완료되는 기록은 홈 도착 확인 후 자동 삭제됩니다.':'';
    const home=result.home_return;
    const homeMessage=typeof home?.message==='string'&&home.message.trim()?home.message.trim():'홈 복귀 결과를 확인할 수 없습니다.';
    const output=$('#clear-history-result');
    output.textContent=[cleared,pending,homeMessage].filter(Boolean).join(' ');
    output.classList.toggle('warning',!['already_home','requested'].includes(home?.status));
    output.hidden=false;
    notify(cleared);
  }
  function newRequestId(){
    if(typeof crypto.randomUUID==='function')return crypto.randomUUID();
    const bytes=new Uint8Array(16);crypto.getRandomValues(bytes);return Array.from(bytes,value=>value.toString(16).padStart(2,'0')).join('');
  }
  function openRequestAction(request){
    if(requestBusy||!authenticated)return;
    const cancel=request.status==='PENDING';
    pendingAction={id:request.id,type:cancel?'cancel':'delete'};
    $('#request-action-title').textContent=cancel?'접수된 요청을 취소할까요?':'이전 배송 내역을 삭제할까요?';
    $('#request-action-description').textContent=cancel?
      `“${request.item}” 요청을 취소합니다. 아직 배차되지 않은 요청만 취소할 수 있습니다.`:
      `“${request.item}”의 완료·취소 내역을 삭제합니다. 진행 중 운행에 포함된 기록은 홈 도착 확인 후 삭제됩니다. 이 작업은 차량을 움직이지 않습니다.`;
    $('#request-action-confirm').textContent=cancel?'요청 취소':'내역 삭제';
    $('#request-action-dialog').showModal();
  }
  function closeRequestAction(){pendingAction=null;$('#request-action-dialog').close();}
  function render(data){
    snapshot=data;authenticated=true;$('#signin-panel').hidden=true;$('#manager-workspace').hidden=false;$('#identity').hidden=false;
    updateClearButton();
    $('#user-label').textContent=`사용자 ${data.user_id}`;
    const service=data.service||{};
    $('#vehicle-power').hidden=false;VehiclePower.render($('#vehicle-power'),service.power);
    $('#service-state').textContent=service.active?'로봇 배송 중':service.home_ready?'로봇 Homepoint 대기':'로봇 상태 확인';
    $('#service-state').classList.toggle('warning',!service.active&&!service.home_ready);
    $('#connection-notice').hidden=false;
    $('#connection-notice').textContent=service.active?'요청을 보내면 현재 배송 목록 뒤에 접수됩니다. 운용자의 배송 상태를 확인하세요.':service.home_ready?'요청을 보내면 운용자가 물품 적재 후 배송을 시작합니다.':'부족 요청은 접수할 수 있습니다. 출발 준비는 로봇 운용자가 확인합니다.';
    const markers=data.markers||[],key=JSON.stringify(markers);
    if(key!==markerKey){
      markerKey=key;const select=$('#marker-id'),previous=select.value;select.replaceChildren();
      const prompt=document.createElement('option');prompt.value='';prompt.textContent='위치를 선택하세요';select.append(prompt);
      markers.forEach(marker=>{const option=document.createElement('option');option.value=String(marker.id);option.textContent=markerLabel(marker);option.title=`ArUco ID ${marker.id}`;select.append(option);});
      if(markers.some(marker=>String(marker.id)===previous))select.value=previous;
    }
    const requests=data.requests||[],nextHistoryKey=JSON.stringify([requests,markers]);
    if(nextHistoryKey===historyKey)return;historyKey=nextHistoryKey;
    $('#history-count').textContent=`${requests.length}건`;$('#history-empty').hidden=requests.length>0;const list=$('#request-history');list.replaceChildren();
    [...requests].reverse().forEach(request=>{
      const row=document.createElement('li');row.className='inventory-request';
      const heading=document.createElement('div');heading.className='inventory-request-heading';const title=document.createElement('strong');title.textContent=`${request.item} · ${request.quantity}개`;const actions=document.createElement('div');actions.className='inventory-request-actions';const status=document.createElement('span');status.className='request-state '+String(request.status||'').toLowerCase();status.textContent=labels[request.status]||request.status;actions.append(status);
      if(['PENDING','DELIVERED','CANCELED'].includes(request.status)){
        const button=document.createElement('button');button.type='button';button.className='request-row-action';button.textContent='×';button.disabled=requestBusy||Boolean(request.deletion_pending);
        button.title=request.deletion_pending?'홈 도착 후 삭제 예약됨':request.status==='PENDING'?'접수 취소':'이전 내역 삭제';
        button.setAttribute('aria-label',`${request.item}: ${button.title}`);
        button.addEventListener('click',()=>openRequestAction(request));actions.append(button);
      }
      heading.append(title,actions);
      const details=document.createElement('p');const marker=markers.find(marker=>Number(marker.id)===Number(request.marker_id));details.textContent=`${markerLabel(marker||{id:request.marker_id})} · ${formatTime(request.created_at)}`;details.title=`ArUco ID ${request.marker_id}`;
      row.append(heading,details);if(request.note){const note=document.createElement('p');note.className='request-note';note.textContent=request.note;row.append(note);}
      if(request.deletion_pending){const pending=document.createElement('p');pending.className='request-delete-pending';pending.textContent='홈 도착 확인 후 삭제 예정';row.append(pending);}
      list.append(row);
    });
  }
  async function refresh(){
    if(refreshBusy||!authenticated)return;refreshBusy=true;const generation=authGeneration;
    try{const data=await api('/api/inventory/state');if(authenticated&&generation===authGeneration)render(data);}
    catch(error){VehiclePower.render($('#vehicle-power'),snapshot?.service?.power,true);$('#connection-notice').hidden=false;$('#connection-notice').textContent=error.name==='AbortError'?'연결 응답이 지연되고 있습니다.':error.message;$('#service-state').textContent='연결 확인 필요';}
    finally{refreshBusy=false;}
  }
  $('#signin-form').addEventListener('submit',async event=>{
    event.preventDefault();const button=event.submitter;button.disabled=true;$('#signin-error').textContent='';
    try{const value=$('#join-code').value.trim(),credential=accessMode==='domain'?{domain_id:Number(value)}:{code:value};await api('/api/inventory/pair',{user_id:Number($('#user-id').value),...credential});authenticated=true;$('#join-code').value='';await refresh();}
    catch(error){$('#signin-error').textContent=error.name==='AbortError'?'연결이 지연되고 있습니다. 다시 시도하세요.':error.message;}
    finally{button.disabled=false;}
  });
  $('#logout').addEventListener('click',async()=>{
    try{await api('/api/inventory/logout',{});snapshot=null;submission=null;historyKey='';$('#clear-history-result').hidden=true;$('#request-form').reset();showSignIn();$('#connection-notice').textContent=signInNotice();}
    catch(error){notify(error.message,true);}
  });
  $('#request-action-cancel').addEventListener('click',closeRequestAction);
  $('#request-action-dialog').addEventListener('close',()=>pendingAction=null);
  $('#request-action-confirm').addEventListener('click',async()=>{
    if(requestBusy||!authenticated||!pendingAction)return;
    const action=pendingAction;closeRequestAction();requestBusy=true;updateClearButton();
    $('#submit-request').disabled=true;$('#logout').disabled=true;
    document.querySelectorAll('.request-row-action').forEach(button=>button.disabled=true);
    try{
      const result=await api(action.type==='cancel'?'/api/inventory/requests/cancel':'/api/inventory/requests/delete',{id:action.id});
      historyKey='';render(result);
      notify(action.type==='cancel'?'접수된 요청을 취소했습니다.':result.deletion?.deferred?'홈 도착 확인 후 해당 내역이 삭제됩니다.':'해당 이전 내역을 삭제했습니다.');
    }catch(error){notify(error.name==='AbortError'?'응답이 지연되었습니다. 요청 목록을 다시 확인하세요.':error.message,true);}
    finally{requestBusy=false;updateClearButton();$('#submit-request').disabled=false;$('#logout').disabled=false;document.querySelectorAll('.request-row-action').forEach(button=>button.disabled=button.title==='홈 도착 후 삭제 예약됨');}
  });
  $('#clear-history').addEventListener('click',()=>{
    if(requestBusy||!authenticated)return;
    const count=clearableCount();
    $('#clear-history-count').textContent=`${count}건`;
    $('#clear-history-deferred-count').textContent=`${deferredCount()}건`;
    $('#clear-history-dialog').showModal();
  });
  $('#clear-history-cancel').addEventListener('click',()=>$('#clear-history-dialog').close());
  $('#clear-history-confirm').addEventListener('click',async()=>{
    if(requestBusy||!authenticated)return;
    $('#clear-history-dialog').close();
    requestBusy=true;updateClearButton();$('#submit-request').disabled=true;$('#logout').disabled=true;
    $('#clear-history-result').hidden=true;
    document.querySelectorAll('.request-row-action').forEach(button=>button.disabled=true);
    try{
      const result=await api('/api/inventory/history/clear',{});
      historyKey='';render(result);showClearResult(result);
    }catch(error){notify(error.name==='AbortError'?'기록 초기화 응답이 지연되었습니다. 목록을 새로 확인하세요.':error.message,true);}
    finally{
      requestBusy=false;updateClearButton();$('#submit-request').disabled=false;$('#logout').disabled=false;
      document.querySelectorAll('.request-row-action').forEach(button=>button.disabled=button.title==='홈 도착 후 삭제 예약됨');
    }
  });
  $('#request-form').addEventListener('submit',async event=>{
    event.preventDefault();if(requestBusy||!authenticated)return;
    const payload={marker_id:Number($('#marker-id').value),item:$('#item').value.trim(),quantity:Number($('#quantity').value),note:$('#note').value.trim()};
    if(!payload.item){$('#request-error').textContent='필요한 물품을 입력하세요.';return;}
    const signature=JSON.stringify(payload);
    if(!submission||submission.signature!==signature)submission={signature,id:newRequestId()};
    requestBusy=true;updateClearButton();$('#logout').disabled=true;$('#submit-request').disabled=true;$('#request-error').textContent='';
    try{
      await api('/api/inventory/requests',{...payload,client_request_id:submission.id});
      submission=null;$('#item').value='';$('#note').value='';$('#quantity').value='1';notify('부족 요청을 접수했습니다.');await refresh();
    }catch(error){$('#request-error').textContent=error.name==='AbortError'?'응답이 지연되었습니다. 같은 내용으로 다시 보내면 중복 접수되지 않습니다.':error.message;}
    finally{requestBusy=false;updateClearButton();$('#submit-request').disabled=false;$('#logout').disabled=false;document.querySelectorAll('.request-row-action').forEach(button=>button.disabled=button.title==='홈 도착 후 삭제 예약됨');}
  });
  document.addEventListener('visibilitychange',()=>{if(!document.hidden)refresh();});
  (async()=>{
    try{const access=await api('/api/access');setAccessMode(access.mode);}catch{setAccessMode('code');}
    try{const auth=await api('/api/inventory/auth');if(auth.access_mode)setAccessMode(auth.access_mode);if(auth.authenticated){authenticated=true;await refresh();}else{showSignIn();$('#connection-notice').textContent=signInNotice();}}
    catch(error){showSignIn();$('#connection-notice').textContent=error.message;}
  })();
  setInterval(refresh,1800);
})();
