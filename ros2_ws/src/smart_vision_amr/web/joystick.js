'use strict';
// Only the operator page loads this control. The server owns command priority and
// expires a missing heartbeat; browser timers are not the motion safety boundary.
const ManualJoystick = (() => {
  const pad = document.querySelector('#manual-pad');
  const knob = document.querySelector('#manual-knob');
  const badge = document.querySelector('#manual-state');
  const reason = document.querySelector('#manual-reason');
  const linearLabel = document.querySelector('#manual-linear');
  const angularLabel = document.querySelector('#manual-angular');
  const limitsLabel = document.querySelector('#manual-limits');
  const keys = new Set();
  const arrows = new Set(['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight']);
  let allowed = false, local = null, manual = null, online = false;
  let maxLinear = 0.12, maxAngular = 0.60, lastError = '';
  let releaseGeneration = 0, releasing = false;

  function render() {
    const state = local ? local.armed ? 'active' : 'acquiring' : releasing || ['RELEASING','WAIT_REPLAN'].includes(manual?.state) ? 'releasing' : !online ? 'waiting' : manual?.active && !manual.owner ? 'other' : allowed ? 'idle' : 'unavailable';
    badge.dataset.state = state;
    badge.textContent = {active:'수동 우선', acquiring:'조종권 연결 중', releasing:'임무 재개 대기', waiting:'연결 대기', other:'다른 운용자 조종 중', idle:'수동 조종 대기', unavailable:'조종 불가'}[state];
    if (!local && manual?.state === 'WAIT_REPLAN') badge.textContent = '임무 재개 준비';
    pad.disabled = !allowed && !local;
    pad.classList.toggle('engaged', !!local);
    const x = local?.x || 0, y = local?.y || 0;
    knob.style.transform = `translate(${x * 48}px, ${y * 48}px)`;
    const v = local?.armed ? -y * maxLinear : 0;
    const w = local?.armed ? -x * maxAngular : 0;
    linearLabel.replaceChildren(document.createTextNode(`${Math.abs(v) < 0.0005 ? '0.00' : v.toFixed(2)} `), unit('m/s'));
    angularLabel.replaceChildren(document.createTextNode(`${Math.abs(w) < 0.0005 ? '0.00' : w.toFixed(2)} `), unit('rad/s'));
    limitsLabel.textContent = `최대 ${maxLinear.toFixed(2)} m/s · ${maxAngular.toFixed(2)} rad/s`;
    reason.textContent = lastError || (local ? local.armed ? '수동 입력이 우선합니다. 손을 놓으면 보관된 목표로 임무를 이어갑니다.' : '기존 임무를 보관하고 자동 주행 명령을 차단하는 중입니다.' : releasing ? '수동 명령을 종료하고 기존 임무를 다시 연결합니다.' : manual?.reason || (allowed ? '임무 중에도 개입할 수 있습니다. 임무와 방문 순서는 유지됩니다.' : '차량 연결과 비상정지 상태를 확인하세요.'));
  }
  function unit(text) { const el = document.createElement('small'); el.textContent = text; return el; }
  function setState(snapshot, {connected = false} = {}) {
    online = connected;
    manual = snapshot?.manual || null;
    const limits = manual?.limits || {};
    if (Number.isFinite(limits.max_linear) && limits.max_linear > 0) maxLinear = limits.max_linear;
    if (Number.isFinite(limits.max_angular) && limits.max_angular > 0) maxAngular = limits.max_angular;
    allowed = connected && !!manual && !snapshot?.mission?.estop_latched && (typeof manual.available === 'boolean' ? manual.available : !!snapshot?.connection?.connected) && (manual.state === 'IDLE' || manual.owner);
    if (!allowed && local) release();
    render();
  }
  function payload(control, active, linear = 0, angular = 0) {
    return {control_id:control.id, seq:++control.seq, active, linear, angular};
  }
  async function post(body, keepalive = false) {
    const abort = new AbortController();
    const timeout = setTimeout(() => abort.abort(), 800);
    try {
      const response = await fetch('/api/manual', {method:'POST', credentials:'same-origin', cache:'no-store', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body), keepalive, signal:abort.signal});
      const data = await response.json();
      if (!response.ok) throw new Error(data.error || `조종 요청 실패 (${response.status})`);
      return data;
    } finally { clearTimeout(timeout); }
  }
  function updateAck(control, data) {
    if (local !== control) return;
    const status = data.manual;
    if (status && (status.owner === false || !['ACQUIRING', 'ACTIVE'].includes(status.state))) {
      lastError = '조종 세션이 종료됐습니다. 놓았다가 다시 누르세요.';
      release();
      return;
    }
    if (status?.available === false || (status?.active && status.owner === false) || ['RELEASING','WAIT_REPLAN','IDLE'].includes(status?.state)) {
      lastError = status.reason || '조종 입력이 종료되었습니다. 상태를 확인한 뒤 다시 눌러주세요.';
      if (status.available === false) allowed = false;
      release();
      return;
    }
    // Only the server's ACTIVE acknowledgement permits a nonzero velocity.
    control.armed = status?.state === 'ACTIVE';
    render();
  }
  async function send(control) {
    if (local !== control || control.inFlight) return;
    control.inFlight = true;
    const body = payload(control, true, control.armed ? -control.y * maxLinear : 0, control.armed ? -control.x * maxAngular : 0);
    try { updateAck(control, await post(body)); }
    catch (error) {
      if (local !== control) return;
      lastError = error.name === 'AbortError' ? '조종 응답이 지연되어 수동 입력을 종료했습니다. 연결을 확인한 뒤 다시 누르세요.' : error.message;
      release();
    } finally { control.inFlight = false; }
  }
  function begin(kind, pointerId = null) {
    if (!allowed || local || releasing) return false;
    lastError = '';
    const id = globalThis.crypto?.randomUUID?.() || `manual-${Date.now()}-${Math.random().toString(36).slice(2)}`;
    const control = {id, seq:0, kind, pointerId, x:0, y:0, armed:false, inFlight:false, timer:null};
    local = control;
    control.timer = setInterval(() => send(control), 100);
    send(control);
    render();
    return true;
  }
  function release() {
    const control = local;
    if (!control) return;
    local = null;
    keys.clear();
    clearInterval(control.timer);
    if (control.pointerId != null && pad.hasPointerCapture(control.pointerId)) pad.releasePointerCapture(control.pointerId);
    const generation = ++releaseGeneration;
    releasing = true;
    render();
    // A higher sequence closes this press, even if its acquire/heartbeat is late.
    post(payload(control, false), true).catch(error => {
      if (generation === releaseGeneration) lastError = error.name === 'AbortError' ? '연결이 지연되어 서버의 수동 입력 만료를 기다립니다.' : error.message;
    }).finally(() => {
      if (generation === releaseGeneration) { releasing = false; render(); }
    });
  }
  function setVector(x, y) {
    if (!local) return;
    const length = Math.hypot(x, y);
    if (length < 0.10) { x = 0; y = 0; }
    else {
      const magnitude = (Math.min(1, length) - 0.10) / 0.90;
      x = x / length * magnitude; y = y / length * magnitude;
    }
    local.x = x; local.y = y;
    render();
  }
  function movePointer(event) {
    if (!local || local.kind !== 'pointer' || local.pointerId !== event.pointerId) return;
    const rect = pad.getBoundingClientRect(), radius = Math.min(rect.width, rect.height) / 2 - 26;
    setVector((event.clientX - rect.left - rect.width / 2) / radius, (event.clientY - rect.top - rect.height / 2) / radius);
  }
  pad.addEventListener('pointerdown', event => {
    if (!event.isPrimary || event.button !== 0 || !begin('pointer', event.pointerId)) return;
    event.preventDefault();
    pad.focus({preventScroll:true});
    pad.setPointerCapture(event.pointerId);
    movePointer(event);
  });
  pad.addEventListener('pointermove', event => { if (local?.pointerId === event.pointerId) { event.preventDefault(); movePointer(event); } });
  for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) pad.addEventListener(name, event => { if (local?.pointerId === event.pointerId) release(); });
  pad.addEventListener('contextmenu', event => event.preventDefault());
  function moveKeys() {
    setVector(Number(keys.has('ArrowRight')) - Number(keys.has('ArrowLeft')), Number(keys.has('ArrowDown')) - Number(keys.has('ArrowUp')));
  }
  pad.addEventListener('keydown', event => {
    if (['Escape', ' ', 'Spacebar'].includes(event.key)) { event.preventDefault(); release(); return; }
    if (!arrows.has(event.key)) return;
    event.preventDefault();
    // A held key must be pressed again after stop, blur or a lost connection.
    if (!local && event.repeat) return;
    if (!local && !begin('keyboard')) return;
    if (local?.kind !== 'keyboard') return;
    keys.add(event.key); moveKeys();
  });
  window.addEventListener('keyup', event => {
    if (local?.kind !== 'keyboard' || !arrows.has(event.key)) return;
    event.preventDefault(); keys.delete(event.key);
    if (keys.size) moveKeys(); else release();
  });
  pad.addEventListener('blur', release);
  window.addEventListener('blur', release);
  window.addEventListener('pagehide', release);
  document.addEventListener('visibilitychange', () => { if (document.hidden) release(); });
  document.addEventListener('keydown', event => { if (event.key === 'Escape') release(); });
  render();
  return Object.freeze({setState, release});
})();
